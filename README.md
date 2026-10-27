## speakerguardd - feed-forward speaker protection

speakerguardd is a userspace daemon, written in Rust, that protects the
speakers of machines whose amplifier has no current or voltage sense lines.
The first such machine is the MacBook Neo (J700): a fixed-gain MAX98360A per
channel on the AOP's serializer, and a "Speaker Sense" capture stream
(`hw:AppleJ700,2`) that returns the words the amplifier receives.

It is the sibling of Asahi Linux's speakersafetyd and speaks the same
interlock to the kernel: the card (`snd-soc-macaudio`) holds the speaker
volume at a safe level until a daemon locks the volume controls and the
"Speaker Volume Unlock" control and keeps writing to it while the speakers
play; if the daemon stops for any reason, the card locks the volume again
within 250 ms.  The configuration files use the same keys, so a speaker
description reads the same way in both projects.

### How it works

Every period (4096 frames) the sense words of each channel are scaled to the
amplifier's output voltage (`vs_scale` volts at full scale) and squared into
the power in the coil (`z_nominal`).  That power drives a two-stage lumped
thermal model - voice coil into magnet (`tr_coil` K/W, `tau_coil` s), magnet
into ambient (`tr_magnet`, `tau_magnet`) - and the coil temperature drives a
governor: gain reduction starts `t_window` below the working limit
(`t_limit - t_headroom`), grows linearly across the window to
`t_reduction_max` dB at the limit (and on beyond it), is released only once
the coil has cooled `t_hysteresis` below where reduction began, and is
released slowly.  A windowed power budget (`p_limit_1s`, `p_limit_60s`: the
mean power over the last second and the last minute) reduces the gain at
once when a burst would exceed what the pair is rated for, before the coil
has warmed up.  The larger of the two reductions is written to the card's
speaker volume control. Ordinary model requests keep using that same ramp,
including requests above the old 7 dB handoff on V/I machines. The daemon
uses the attenuation needed by the most restrictive governor and onset
reserve, rounded up to the card's steps. The unlocked control range is the
limit (201 half-dB steps, or 100.5 dB, on the supported cards), rather than
a fixed model threshold.

A hard model or sense fault, lost observations, failed control write or
unrepresentable attenuation still gives the lock back to the kernel.
After a range fault, the target must fit with 3 dB to spare before the lock
is taken back. Hard-temperature latches and the requirement for two good
sense periods after bad data still apply. The old `max_reduction` setting
and `-m` option are accepted for configuration and recording compatibility;
they no longer cause a handoff during normal attenuation.

There is no feedback: the model can only be as good as its constants.  The
J700's constants - thermal resistances and time constants, coil temperature
limits, volts at full scale, coil resistance and the power budgets - are the
machine's own loudspeaker-manager tuning, read back through the public
AudioUnit API and recorded in the bring-up provenance record.  Without the
daemon the kernel locks the J700's speakers 20 dB below the amplifier's
full scale; with it, the model governs up to full scale.

### Delayed wakeups

A delayed wake can leave complete periods queued in the sense capture.
The loop drains up to eight retained periods, for at most 10 ms, before
considering a renewal. It measures those frames at their verified capture
positions instead of first charging their time as unknown. A hard fault
ends the drain immediately. Missing frames and uncertain position edges
still count at full scale for their actual duration; a delay does not
justify assuming that the music stayed quiet.

The packaged service currently uses normal scheduling and does not pin
memory. Higher priority and memory locking can reduce jitter, but cannot
prove a deadline under I/O stalls and are not needed for the attenuation
policy. The kernel lease is unchanged.

### Missing or invalid speaker data

Only a complete PCM period with finite voltage, power and model state counts
as measured.  Empty, short or overrun periods, capture errors and stream
reopens are not; configuration scalars must be finite, with positive
physical constants and valid window/budget domains.

Time the daemon could not measure is not ignored and is not fatal either.
The rule is deliberately simple:

* The thermal model has one account, and every step advances it by exactly
  the time stepped.  Time is either covered by measured sense frames or
  charged as a full-scale square wave at the least reduction the volume
  controls held at any time since the last verified boundary (widened by the
  500 ms the card can keep playing samples converted at an earlier gain).
  There are no tolerances, no partial credits and no assumptions about when
  inside an uncertain interval anything played.  Frames over time already
  charged add nothing.
* Frames are placed by the capture's backlog, read between two clock reads.
  The position is whole frames, so their end is known only to within a
  frame and whatever the two reads leave open.  Only the frames that
  certainly played inside the placed span are credited, at the most
  energetic of their possible alignments; the uncertain slivers at either
  end are charged.
* A verified boundary is the end of measured frames, or two looks at closed
  speakers with no sample-rate change event between them.  The events are
  drained before each look's sample-rate read and again after it, so a
  stream that comes and goes while the read takes its time is caught; an
  event after the read is kept for the next look.  A quiet look vouches
  only for the time up to 5 ms before it, where an event could still be on
  its way.  The card's change events are required: without them nothing
  between two looks is known, and the daemon never takes the lock.
* The power budgets keep the power before the card's gain.  Each measured
  frame counts at the most reduction it can have been converted at: any
  value the controls held from 500 ms before it played (the reduction the
  card applies, rounded up to its steps; after a failed write, the old
  value or the new one rounded up, since each control's write is atomic),
  the card's own safe level
  from the moment the daemon's lease may have run out, and anything for the
  controls' value before the daemon set one.  A reduction too deep to have
  delivered a frame that loud did not convert it; a word is taken a step
  above itself; nothing before the gain is louder than full scale, and a
  frame the card may have muted counts as full scale.  A budget window that
  cuts a step counts as much of the step's energy as full scale fits in
  the part it covers.
* Every renewal of the lock, including the one before the blocking sense
  open, goes through the same decision: the account brought up to date,
  then the hard limit, the reduction limit, the model's freshness and the
  sense data's.  The check is made again after the renewal itself, which
  can block, and a renewal that left the model behind is given up again.
* A stream silent for 500 ms is closed and reopened.  Without a complete
  period for longer than that, the daemon gives the card's lock up (macaudio
  locks the volume at once and stops streams converted at the old gain).
  The lock is taken back after two complete periods in a row.  Any failure to
  give the lock up is fatal: the exit gives it up too.

Known costs of that rule, accepted on purpose:

* A stream after idle is charged at full scale from the last look at closed
  speakers to its first frames (the closed speakers are looked at every
  25 ms, and opening the sense capture takes about 100 ms on the J700), and
  the end of a stream from its last frames to the look that sees it closed.
  On the J700 at 0 dB that is a few degrees on the coil model per stream.
* A sense outage is charged at full scale at the least gain since the last
  verified boundary until periods return, so the lock comes back later after
  one than the speaker strictly needs.
* For half a second after a gain change the budgets may count a frame at
  the earlier, deeper reduction, which slows a budget's release.  After a
  start in the middle of playback the samples still playing count as loud
  as they can have been, a reduction of up to about 1 dB for a second or
  two on the J700.
* A frame of every period is charged at full scale (the position's
  uncertainty): about 3 mW on the J700's coils.
* After a start without a clean stop the budgets assume a minute of full
  scale (up to about 7 dB on the J700, easing over the minute after the
  start), and after a clean stop the time since it counts as full scale
  before the gain.

Exits are left for what the daemon cannot recover from (no card, a
configuration error, a nonfinite model, a lock it cannot give up); the
speakers are then at the card's safe level.  The unit restarts the daemon
(`Restart=on-failure`, five attempts a minute).  A restart resumes the model
only from the state a clean stop writes to `/run/speakerguardd/state`
(`-s`): SIGTERM gives the lock up, waits out the 500 ms drain, accounts every
moment and only then writes it, marked `released 1`, its coverage and its
write named by one epoch.  It is resumed once: the run that resumes it
overwrites it (or, failing that, removes it) before it can unlock, and does
not unlock while it can do neither; the state is read only once the daemon
holds the card's lock.  The time since it is charged at the card's safe
level, and its two budget means are placed at full scale as late in their
windows as the energy can have come.  The clock reads that pair the boot
time with the daemon's own clock, at the final save and at the restore, are
bounded: whatever time lies between them is charged once more at full scale
and full gain.  Any other state (a crash, an older version's file, a state
whose model trails its write) or none starts both coil and magnet from
independent upper bounds. Each starts at its hard limit, with the possible
queued full-scale drain added and a floor at the kernel safe level's steady
state. The fault stays latched until the model cools under its working
ceiling, about 25 s on the J700. A hot magnet after another OS ran need not
have the steady-state relationship to the coil. This assumes each actual
node was at or below its hard limit before the queued drain. A daemon that
is late renewing the lock is covered by the kernel's 250 ms lock timeout,
and the late time is charged like any other.

### Speaker profile coverage

The status below is the status shipped in these files. A public thermal
model or a matching speaker count does not qualify a machine. Every added
profile explicitly sets `qualified = false`; governing it requires
`--allow-unqualified`. The existing J700 qualification is unchanged.
The new profiles select `unified` with `guard = asahi`, keeping the
measured-power guard alongside feed-forward protection.

"In service" marks the three machines this daemon has governed since
2026-10-25 (an M1 Pro J316s, an M2 J413 and an M4 Pro J616s), with the
feed-forward model primary under the asahi guard, each change reviewed
before it was installed.  They still say `qualified = false` until a
formal qualification changes it, so governing them also takes
`--allow-unqualified`.

There are 35 profile families covering 53 board variants, including 31
new drafts. The filename uses
the family name, for example `apple/j714.conf` for J714s/c. Match the actual
ALSA card ID before any use; a missing matching profile remains an error.
Board identities come from the [Asahi device list](https://asahilinux.org/docs/hw/devices/device-list/).
Suffixes in the table expand to full board names, for example J714s and J714c.
The output count is the number of separately modelled amplifier outputs.

| Machine | Board | Outputs | Status | Public source |
| --- | --- | --- | --- | --- |
| Mac Pro M2 Ultra | J180d | 2 | untested | [Asahi J180](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j180.conf) |
| Mac mini M1 | J274 | 1 | untested | [Asahi J274](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j274.conf) |
| MacBook Pro 13-inch M1 | J293 | 4 | untested | [Asahi J293](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j293.conf) |
| MacBook Air 13-inch M1 | J313 | 2 | untested | [Asahi J313](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j313.conf) |
| MacBook Pro 14-inch M1 Pro/Max | J314s/c | 6 | untested | [Asahi J314](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j314.conf) |
| MacBook Pro 16-inch M1 Pro/Max | J316s/c | 6 | in service, not yet qualified | [existing profile](conf/apple/j316.conf) |
| Mac Studio M1 Max/Ultra | J375c/d | 1 | untested | [Asahi J375](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j375.conf) |
| MacBook Air 13-inch M2 | J413 | 4 | in service, not yet qualified | [existing profile](conf/apple/j413.conf) |
| MacBook Pro 14-inch M2 Pro/Max | J414s/c | 6 | untested | [Asahi J414](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j414.conf) |
| MacBook Air 15-inch M2 | J415 | 6 | untested | [Asahi J415](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j415.conf) |
| MacBook Pro 16-inch M2 Pro/Max | J416s/c | 6 | untested | [Asahi J416](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j416.conf) |
| Mac mini M2 | J473 | 1 | untested | [Asahi J473](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j473.conf) |
| Mac mini M2 Pro | J474s | 1 | untested | [Asahi J474](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j474.conf) |
| Mac Studio M2 Max/Ultra | J475c/d | 1 | untested | [Asahi J475](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j475.conf) |
| MacBook Pro 13-inch M2 | J493 | 4 | untested | [Asahi J493](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j493.conf) |
| MacBook Pro 14-inch M3 | J504 | 6 | untested | [Asahi J504](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j504.conf) |
| MacBook Pro 14-inch M3 Pro/Max | J514s/c/m | 6 | untested | [Asahi J514](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j514.conf) |
| MacBook Pro 16-inch M3 Pro/Max | J516s/c/m | 6 | untested | [Asahi J516](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j516.conf) |
| Mac Studio M3 Ultra / M4 Max | J575d/c | 1 | untested | [Asahi J575](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j575.conf); M4 Max uses sibling assumptions |
| MacBook Pro 14-inch M4 | J604 | 6 | untested | [Apple count](https://support.apple.com/en-us/121552), [Asahi J504](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j504.conf) sibling assumptions |
| MacBook Air 13-inch M3 | J613 | 4 | untested | [Asahi J613](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j613.conf) |
| MacBook Pro 14-inch M4 Pro/Max | J614s/c | 6 | untested | [Apple count](https://support.apple.com/en-us/121553), [Asahi J504](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j504.conf) sibling assumptions |
| MacBook Air 15-inch M3 | J615 | 6 | untested | [Asahi J615](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j615.conf) |
| MacBook Pro 16-inch M4 Pro/Max | J616s/c | 6 | in service, not yet qualified | [existing profile](conf/apple/j616.conf) |
| MacBook Neo A18 Pro | J700 | 2 | qualified | [existing profile](conf/apple/j700.conf), [Apple specs](https://www.apple.com/macbook-neo/specs/) |
| MacBook Pro 14-inch M5 | J704 | 6 | untested | [Apple count](https://support.apple.com/en-us/125405), [Asahi J504](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j504.conf) sibling assumptions |
| MacBook Air 13-inch M4 | J713 | 4 | untested | [Apple count](https://support.apple.com/en-us/122209), [Asahi J413](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j413.conf) sibling assumptions |
| MacBook Pro 14-inch M5 Pro/Max | J714s/c | 6 | untested | [Apple count](https://support.apple.com/en-us/126318), [Asahi J504](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j504.conf) sibling assumptions |
| MacBook Air 15-inch M4 | J715 | 6 | untested | [Apple count](https://support.apple.com/en-us/122210), [Asahi J415](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j415.conf) sibling assumptions |
| MacBook Pro 16-inch M5 Pro/Max | J716s/c | 6 | untested | [Apple count](https://support.apple.com/en-us/126319), [Asahi J316](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j316.conf) thermal / [J616](conf/apple/j616.conf) routing assumptions |
| Mac mini M4 / M4 Pro | J773g/s | 1 | untested | [Apple count](https://support.apple.com/en-us/121555), [Asahi J375](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j375.conf) sibling assumptions |
| Mac Studio M5 Max/Ultra | J775c/d | 1 | untested | [Apple count](https://www.apple.com/mac-studio/specs/), [Asahi J375](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j375.conf) sibling assumptions |
| MacBook Air 13-inch M5 | J813 | 4 | untested | [Apple count](https://www.apple.com/macbook-air/specs/), [Asahi J413](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j413.conf) sibling assumptions |
| MacBook Air 15-inch M5 | J815 | 6 | untested | [Apple count](https://www.apple.com/macbook-air/specs/), [Asahi J415](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j415.conf) sibling assumptions |
| Mac mini M6 / M5 Pro | J873g/s | 1 | untested | [Apple count](https://support.apple.com/en-us/128108), [Asahi J375](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j375.conf) sibling assumptions |

The Asahi links pin speakersafetyd 2.0.2, the last MIT release. Each new
file cites its numeric sources and labels borrowed fields. See
[profile provenance and format](PROFILES.md) for amplifier gains, V/I
mapping, resistance assumptions, thermal limits and licensing. Apple
specs establish counts for later machines, not their amp or sense mapping.
These drafts make no claim of macOS parity or validated loudness.

The following machines have internal speakers but no profile in this
branch. The [Asahi iMac data](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/conf/apple/j456.conf)
has thermal constants but no PCM, controls or V/I map, and the
[public kernel](https://github.com/aurora-silicon/linux/blob/070c8e4d43a40ac464fe8bd491c090688a92b63a/sound/soc/apple/macaudio.c)
excludes SSM3515 from its sense route. Inventing a working feedback path
would misrepresent that hardware. These need a supported feed-forward tap
and a calibrated amplifier scale before a runnable profile can be added.

| Machine | Board | Status | Missing public data |
| --- | --- | --- | --- |
| iMac M1, four ports | J456 | skipped | SSM3515 sense/tap route and scale |
| iMac M1, two ports | J457 | skipped | SSM3515 sense/tap route and scale |
| iMac M3, two ports | J433 | skipped | Compatible amp/tap mapping and scale |
| iMac M3, four ports | J434 | skipped | Compatible amp/tap mapping and scale |
| iMac M4, two ports | J623 | skipped | Compatible amp/tap mapping and scale |
| iMac M4, four ports | J624 | skipped | Compatible amp/tap mapping and scale |

M6 coverage is the publicly documented Mac mini, J873g. There are no
speculative profiles for unannounced board identities. iPhones and iPads
are outside this Mac profile inventory.

### Helping validate an untested profile

Start with read-only information, with the current speaker protection
running and no test playback. On Linux, substitute the actual card ID for
`AppleJ314` in these commands:

```sh
tr '\0' '\n' < /proc/device-tree/compatible
uname -r
cat /proc/asound/cards /proc/asound/pcm
arecord -l
amixer -c AppleJ314 contents
systemctl status speakerguardd.service speakersafetyd.service --no-pager
journalctl -u speakerguardd.service -u speakersafetyd.service -n 200 --no-pager
```

Include the board, kernel and daemon revision, exact profile, PCM format,
channel count, amp gain control range, sense switches, volume range and
current protection service. Compare the names and slots with the profile
and its linked UCM/graph sources. Do not change mixer values, enable a
raw speaker route, stop protection, or use test tones for this check.

For a recording, send a short existing speakerguardd `--record FILE`
capture and its corresponding profile and journal through a maintainer's
agreed private channel. The recording contains the words the amplifier
sees and can contain recoverable audio. Review it and redact identifying
logs before sharing; do not publish private audio, serial numbers or lab
records. If the current protector is speakersafetyd, its existing
blackbox capture and journal are useful initial evidence too.

`--record` belongs to the governing daemon in this version. It is not a
read-only capture mode, and `--shadow` still lets the primary governor
write controls. Do not start a second daemon, restart protection, or opt
into an untested profile just to collect evidence. A new recording needs
a separately reviewed handoff with protection continuously in place.
Idle evidence is welcome; do not raise the volume or start loud playback.
Only an already approved, protected session can provide meaningful
nonzero sense data.

Maintainers should check slot order, polarity, V*I sign, scale, impedance,
thermal behaviour, suspend/resume and fallback from the recording. Use
`--replay FILE --model j314 -c conf --csv replay.csv` for offline review.
A successful replay certificate describes the recording, not hardware
safety. Silent samples cannot establish polarity or coil resistance.
Qualification requires evidence for every role and board variant, a
review of the constants, and an explicit change to `qualified`; parsing,
read-only checks and a sibling match alone are insufficient. No loud
playback is part of this contribution procedure.

**Across a reboot (`boot_resume`, off by default).**  `/run` dies with the
boot, so by default every boot starts latched at the coils' limit (on the
Neo, an 11.9 dB first lock released over 7.6 min).  With `boot_resume =
idle` in `[Globals]`, a clean stop made while systemd shuts the system down
also writes its state to `/var/lib/speakerguardd/state` (`--persist`).  The
daemon makes the directory itself, after the card is locked; the unit adds
nothing that could fail before the daemon runs.  The record is checksummed
and bound to the machine (`/etc/machine-id`), the parsed configuration
and the boot that wrote it. Version 4 requires an independent RTC age bound
and refuses older records. The checksum covers the version
header too, so a damaged header cannot promote an older record. It is written and synced only when the
system clock is synchronized.  The first start of the next boot (no state
under `/run`) resumes it only if all of the following hold:

* the journal lists the writing boot as the one just before this one;
* the machine and configuration match and the checksum holds;
* no temperature is under the ambient;
* the system clock passes synchronization checks at both ends and agrees
  with an independent, accurate RTC present at both ends;
* the age interval covers RTC quantization, read placement, up to 0.1% RTC
  rate uncertainty and the snapshot pairing, and is no shorter than uptime.

The timestamp is read between two synchronization queries while a
`TFD_TIMER_CANCEL_ON_SET` timer watches for a clock step. An unsynchronized
query, a step, a pending leap transition or a failed watch rejects the
sample. Overlapping setters can restore the offset before both timerfd
notifications and evade this watch. Realtime alone therefore cannot authorize
idle restore. The entire independent RTC interval is included even when it
agrees with realtime; a missing RTC keeps the hot latch. The guarded sample
must finish within 50 ms of boot time, on both
save and restore. Restore also bounds the whole wall-clock read to 50 ms.
The shutdown record keeps the whole delay from the model snapshot through
the clock reads. Missing or backward boot-time reads cannot produce a
clean snapshot or a persistent pairing. These bounds give an interval of
possible snapshot ages.
The restored coil, magnet and governing peaks bound that entire interval,
including interior coil extrema. A one-microkelvin upward allowance covers
the temperature record's six-decimal rounding. The legacy model uses an invariant upper
bound that also covers its magnet-first numerical steps. Full-scale power
before the gain is charged to the budgets through the longest possible age.

Every start removes the file and syncs its directory before it can unlock,
and does not unlock while it cannot, with the option or without and with
`--persist ''`. A stop that is not a shutdown writes nothing.

#### The contract for idle restore

Setting `boot_resume = idle` asserts all of these operating conditions.
They are requirements on the installation and its owners, not facts proved
by the record or by `journalctl --list-boots`.

* Every intervening Linux boot must leave a retained boot entry in the same
  persistent journal consulted at restore. Journal storage must work from
  early boot through shutdown and retain every boot until this record is
  consumed. Volatile storage, unwritable persistent storage, vacuuming,
  journal replacement or a boot that leaves no messages break this contract.
  Adjacency among retained entries does not prove that no boot is missing.
* Every speaker protection owner must participate in the same durable
  invalidation protocol, using the same persistent path before it can drive
  the speakers. This includes alternate governors, recovery systems and
  runs with persistence disabled. Redirecting the path must first durably
  invalidate the old path. After the final shutdown save no later owner
  may drive the speakers above the card's safe level in that writing boot.
  systemd reporting `stopping` does not establish this last-owner property.
* No other OS may drive these speakers between the saved state and restore.
  Linux cannot see that history. Every Apple Silicon Mac can boot another OS;
  merely calling an installation Linux-only does not establish continuity.
* The independent RTC must remain accurate and its rate error stay within
  0.1% across shutdown and boot. It must not be adjusted with the system
  realtime clock during capture. Agreement does not detect both clocks
  moved forward together.
  Clock administration must preserve this contract; the sample watch is
  local to each read, not a record of clock history while Linux was down.

If any condition cannot be guaranteed, keep `boot_resume = latched`.
After a known break, durably remove the old record before enabling idle
restore again. The default remains latched. The regression suite retains
synthetic missing-boot, later-owner and synchronized-but-wrong-clock cases
as examples outside this contract. They deliberately show that the parser
can accept such histories; no journal predicate proves their absence.

An unsupported other-OS history restored a J700 model cold enough to exceed
its limit by 58 K in review. This is a model calculation, not a measured
speaker temperature. Idle restore must not be used on dual-boot machines.

### Machines with current and voltage sense

The same daemon can protect the Apple laptops whose amplifiers report the
speaker voltage and current (TAS2764-class SN012776, S16 sense words, 14 V
and 3.75 A at full scale): `conf/apple/j316.conf` (MacBook Pro 16" M1 Pro),
`j413.conf` (MacBook Air M2) and `j616.conf` (MacBook Pro 16" M4 Pro).  It
turns the VSENSE and ISENSE switches on, sets the amplifier gain to its
maximum (the level the model assumes, `amp_gain_dbv`) and writes one volume
control per speaker, linked by `group`.

None of the three is qualified on the machine yet, and their files say
`qualified = false`.  The daemon refuses to govern them without
`--allow-unqualified`, and udev starts it only on the J700, so installing
the package never displaces speakersafetyd.  The unit never stops it
either: one daemon holds the card's interlock, whichever takes it first,
and a configuration this daemon refuses to govern exits with status 78
before it touches the card, which the unit does not restart on. Keep the
current protection running. A change of governing daemon needs a reviewed
handoff with fallback and rollback available. `--allow-unqualified` is an
explicit risk opt-in, not a qualification procedure. Keep speakersafetyd
installed and available for fallback.

On those machines the files' governor is `unified` under `guard = asahi`
(below): the feed-forward model is primary, as in macOS, where the
signal-driven model always runs and V/I sense checks it, and the asahi
governor's measured-power model is a second, independent limit on every
speaker.  This is what all three machines have run since 2026-10-25 under
review r4.  `--governor asahi --guard none` gives the asahi governor alone,
the behaviour Asahi Linux's speakersafetyd ships, reimplemented here from its
description (credit to the Asahi Linux contributors; speakersafetyd is MIT
through 2.0.2).  The measured power (V*I, integrated per sample) drives the
two-node model with speakersafetyd's constants (`vi_*`, `z_nominal`).  The
reduction ramps from 0 at `t_limit - t_headroom - t_window` to
speakersafetyd's `min_gain` at the working limit, with peak-hold
hysteresis.  It differs only toward protection:

* It runs inside the same account as the J700's model and by the same
  rule: time not covered by measured frames is charged at full scale at
  each speaker's least reduction since the last verified boundary, instead
  of skipped.
* The hard limit is checked at every sample, for the coil and the magnet.
  A crossing latches.  Negative V*I beyond 1 mW after it in the same
  stretch leaves the model at the limit at least, so a bad sample cannot
  hide the crossing, while a latched model cools as it should through
  sense noise (a few LSB, which makes nearly half the samples of a silent
  period negative) or no negative samples at all.
* After a hand-off, once what was converted at the old gain has drained,
  uncovered time is charged at each control's value or the card's safe
  level, whichever is more, until the lock is renewed: the card holds its
  safe level while the daemon does not hold the lock.
* A period whose mean V*I is below -0.01 W is bad sense data: uncovered
  time, charged at full scale, never measured.  The card goes to the kernel
  at once (which mutes these amplifiers), and two good periods in a row are
  needed to take the lock back.  The wait for them survives a clean stop
  and restart.
* The governor's state (temperatures, holds, gain, latch) is saved with the
  rest of the model and resumed only from a clean stop.  A missing state,
  or a run of speakersafetyd since boot (its flag file under /run), starts
  every model latched, not at speakersafetyd's 119 C: coil and magnet at
  the hard limit with the half second of drain at full scale held on top.
  On these speakers that keeps the card at its safe level for about a
  minute after every boot (58 s on the J413, 84 s on the J316), mostly the
  time their magnets take to cool from the limit.
* A current line that reads far too little for its voltage (a dead or
  switched-off line, where speakersafetyd sees no power at all) is lost
  for the period.  That is a 10 ms block with at least 0.1 V rms whose
  current is under a tenth of what the speaker's nominal impedance would
  draw, or a 1 ms block under a hundredth (a loud program's lost line,
  caught within a millisecond).  A live line through a reactive load
  stays well under both down to 20 Hz.  The measured-power models then
  take V^2 over the speaker's smallest impedance, the most current there
  can be, until the line reads again, and the daemon logs both changes
  (once a change has held for a few periods; the bound follows the line
  at once).  The short block counts only when two in a row read lost.
  What a line lost too briefly for a block to show may have under-read
  at a period's end or start (the most current less what was read) is
  charged unless the next or last period shows the line reading, and at
  once when the speakers close or measurement stops.  Below the blocks'
  floor (0.1 V rms, or the line's own noise times the ratio) a dead line
  cannot be told from a quiet one, and it under-reads there at most that
  voltage squared over the smallest impedance.
* A line that reads low but plausibly, or a dead one carrying a DC
  offset, is lost the same way.  Over the last 3 s of program with at
  least 20 mA rms below 120 Hz, the low band's resistance (sum(v i) over
  sum(i^2)) above 1.35 times the smallest impedance, raised with the
  measured-power model's coil temperature at the coil's temperature
  coefficient, is more than a live coil can show.  On the J616s's woofers
  at full volume it is 1.28 to 1.49 times; a line reading half doubles
  it.  The power is bounded from the first stretch R is over the bound
  and stays bounded until R reads under 0.9 of the least bound since
  the first fault, on program the card played (not at its safe level, whose words
  are the sense's noise).  The least bound: the bound rises with the
  model's coil, which the bounded line itself heats, and that must not
  end the bound. Recovery clears the fault but retains that reference.
  The model can still carry excess heat from the earlier bound, so its
  higher temperature must not hide a second fault. This can keep the
  detector more conservative after a recovery. Nothing else clears it;
  a passage without bass is no
  evidence that the line reads again.  Only the logged verdict waits: at
  once from 1.25 times the bound, else after R has stayed over it for a
  second of program.  And over a period with at least 0.1 V rms and the
  current the dead-line test passes, or 0.3 V and 20 mA, current that
  does not follow the voltage (sum(v i) under 0.2 of sqrt(sum(v^2)
  sum(i^2)); a live load of up to 60 degrees keeps it over 0.4) is no
  reading, at once from 0.5 V rms and after 2 s of such program under
  it; from 0.3 V and 20 mA its power is bounded meanwhile.  What V and I cannot tell apart is a line reading a fraction
  of its current on a program with no low-band current from a higher
  impedance: the tweeters, and treble-only programs on the woofers.
* `z_min` in a speaker's section sets its smallest impedance (a unit's
  own DC resistance) when it is lower than the default, the smaller of
  `z_nominal` and `r_ff`.

At the J413 tweeter's sensitivity a full-scale charge heats its model by
about 2 C per millisecond, so stream edges and sense hiccups cost much more
there than on the J700.  Qualification on the machine has to show what
that means in use.

### Shadow mode and the unified governor

`--shadow` runs every governor the machine supports on the same account
and writes one JSON line a second to `shadow.jsonl` next to the state file:
each model's temperatures, each governor's reduction, what each control
holds, and each estimator's power.  A shadow changes nothing the
governing model does.

The `unified` governor is a look-ahead regulator on the feed-forward
model, after the design of macOS's speaker protection: power from the
sensed voltage against the speaker's reference resistance (`r_ff`) drives
the two-node model with macOS's open-loop constants, and the regulator
lets through the most power that keeps the coil at or under the working
limit over the next `horizon_s`.  It attacks at once and releases over
`release_s`.  It predicts the program from the frames last measured at a
gain the card is known to have applied.  A charge, which stands for time
no frames measured, heats every model at the worst case but is not the
program, and frames the card held at its safe level say nothing about it.

V/I machines also keep a ceiling for a full-scale onset. A quiet period
does not bound the next period. Before writing a gain, the guard projects
full-scale power at the proposed gain and at any louder gain still in the
queue, using the amplifier's maximum voltage and the smallest impedance.
It covers the unobserved time, the old lease and half-second queue bound,
a possible renewed lease and the allowed control delay. It checks both
thermal nodes and the
safe-level tail after handoff. Unified also checks its feed-forward model.
The prediction reserves the whole possible energy dose at once, so idle
gaps in the kernel's playing-time lease cannot hide a later onset.
The projection uses model copies, so it adds no energy to the account.

This ceiling can reduce even a quiet program. It can need more attenuation
than the old `max_reduction` threshold. Both the governor request and this
ceiling must fit the unlocked hardware range. The ceiling only adds
attenuation to an otherwise permitted write. If the old queued audio
already uses the available headroom, the guard gives up the lease. Mute
does not erase those earlier gains. J700 has no V/I path and keeps its
existing behavior. None of this qualifies a draft profile for hardware.

On the V/I machines it governs under the asahi guard (`governor = unified`
and `guard = asahi` in their files, or `--governor unified --guard asahi`).
The guard needs V/I sense on every speaker:

* each speaker's control gets the larger of the two governors'
  reductions, group-linked, every period, and either one's hard latch
  hands the card to the kernel, so the card is never less protected than
  under the asahi governor alone;
* the regulator then holds the feed-forward model only.  The
  measured-power side is the guard's: unified's own measured-power model
  still runs, latches and is logged;
* a dead or switched-off current line is bounded as under the asahi
  governor alone (above), so the guard keeps speakersafetyd's constants
  on it instead of leaving the feed-forward model alone;
* a period that is not a measurement (mean V*I below -0.01 W, or
  nonfinite) stays a fault as above, because the feed-forward model's
  voltage comes from the same words.

Without the guard it writes the card only with `--experimental-unified`.
Resistance thermometry from low-band V/I is logged only: without a pilot
tone, content alone does not carry a trustworthy coil resistance.

### Recording and replay

`--record FILE` keeps what the guard sees and decides: every sample-rate
change and change event, every sense open, every complete period's words
with the most reduction the card may have applied over it (its safe level
whenever the lock was not held throughout), the frames read at each close,
every lost or unusable period, failed control writes, the lock taken and
handed back with the reason, the governing latch, every span of its account
(measured, charged or cooled), each look at closed speakers that charged
with the change events it saw, and the model the guard started from.

`--record-max-mb` (512 by default, MiB) is a rolling budget: the active
`FILE` and `FILE.previous` each target half that size. At a limit, the
recorder finishes the current guard tick, closes that segment with an early-end
marker, replaces `FILE.previous`, and starts a new `FILE`. Every rotation logs
a warning. Protection keeps running. Records are never split: each segment may
exceed its target by its start state and one tick's records (including a
bounded close drain). A temporary `FILE.next` also exists while rotating.
These three sidecar names are reserved for the recorder; do not use them for
other files. This budget does not include evidence archived outside the ring.

A new segment includes a fresh model snapshot and an explicit rotation-loss
record. It keeps recent electrical evidence, but does not retain the preceding
capture backlog, interlock or decision context. Replay refuses to certify
rotated segments; they are diagnostic windows, not complete live-run replays.
Save both segments outside the ring before they are replaced if a full run is
needed. Existing tools that watch one inode must stop and re-admit on rotation.
A stale `.next` or `.previous` at startup is preserved; recording is declined
with a warning rather than pairing two runs. The live recorder hands complete
records to a bounded writer queue. A full queue or writer failure drops
recording; filesystem calls never run in the governing tick. The writer logs
open, write and rotation errors. Ring files without an early-end marker are
unsealed and replay refuses to certify them, including after a failed
rotation or crash. Protection continues throughout.

`--replay FILE --model NAME` runs the guard itself on the recording,
offline: the speakers open and close where they did, the periods arrive
when they were read, losses and failed reads recur at their times, the
looks at closed speakers come where the live ones did and see the same
events, and a simulated card keeps the kernel's lease, notice and safe
level.  The
program before the card's gain is rebuilt from the words and the recorded
gain, then played through the gains the replayed guard writes (rounded up
to the control's step, `--gain-db` louder if asked) and clipped at full
scale.  `--governor` replays another governor on the same program,
`--csv FILE` writes a line per period.

The replay reports its account: how much was measured, charged and cooled,
with every uncovered interval listed as charged, not measured.  It lists
what it cannot reproduce: lost frames, bad sense data, failed reads and
opens, failed control writes and the daemon's own lateness (neither is
replayed), periods played louder than recorded where the recording cannot
say what the program was, and an early end.  With the recorded governor at
0 dB it also compares its lock and latch transitions with the live ones.
Its account is compared with the live one in every case, span by whole
span to a frame at each end: time the live daemon charged that the replay
measured or cooled, time it measured that the replay cooled, or a replayed
account that ends early, all refuse the certificate.  It certifies a recording only when none of that is there, the
transitions match and nothing it measured was lost or rejected in the
recording; otherwise it says why and exits with status 3.  Recordings from
before 0.1.10 (format 2) are not read.  A certificate is about the
recording, not the speakers: it says the replay saw what the live daemon
saw.

### Randomized interleavings

`src/harness.rs` runs the real guard on the J700 model against a simulated
world: streams of
any length (shorter than a period too), a program before the card's gain,
copy chunks converted at the gain the card really held and played through a
queue of up to 341 ms, the lease in playing time, macaudio stopping old-gain
streams when its lock goes, control writes that fail or land partly, rounded
to their steps, sample-rate reads that fail or block, change events with
latency, a capture that stalls, shortens or blocks on its position, a
daemon descheduled between any two calls, pauses between paired clock reads,
and restarts (clean, crashed, the state damaged, removed or rewritten as the
old format).  An oracle written apart from the daemon integrates what was
really played into its own thermal model and keeps the true power before
the gain.  After every pass it asserts that the model is never cooler than
the oracle's, that its budgets never hold less energy before the gain, that
while the daemon holds the lease the card never applies less reduction than
the true state needs, and that what cannot be proven ends with the card at
its safe level (no lock without events, none after a failed rate read, no
renewal blind past the limit, a latched start from anything but a clean
stop).  Failures shrink to a minimal scenario printed as Rust; each one it
found is a test of its own.

    cargo test --release harness                      # 200 cases
    SPEAKERGUARDD_CASES=1000000 cargo test --release harness

`src/harness/vi.rs` does the same on the J316 and the J413, with the asahi
governor writing a volume control per speaker, or the unified governor
under the asahi guard, and the unified models in shadow or governing.  Each stream has a share for each speaker and plays a
sine or a square into a load of random impedance and phase, so V*I goes
negative over part of every cycle.  The sense words are S16 V and I with up
to 20 LSB of noise and fixed offsets, and periods can be bad sense data or
reach the daemon late.  The gain is the amplifier's own or a copy queue's.
Its oracle integrates the true power into the measured-power model and
both feed-forward ones.  On top of the J700 world's properties it checks
that no lock follows bad sense data before two good periods or holds a
latched model, and, after a quiet tail as long as the daemon's own models
need to cool, that the latch has cleared and the lock is back.  Under the
guard it checks that the card is never less protective than speakersafetyd
alone on the true state, and that the lock is never held with the
feed-forward truth at its hard limit.  Any case can lose its current line
for a while, after which the measured-power models are checked against
the truth less what the dead-line bound can miss.  Some cases end in a
full-scale onset, at the top of a staircase or inside a partial write
failure.  A long class runs steady programs for minutes, long enough to
heat the feed-forward truth, under the asahi governor and the guard with
the current lines live and dead, and checks that the measured-power truth
never reaches its limit.

    SPEAKERGUARDD_VI_CASES=100000 cargo test --release harness::vi
    SPEAKERGUARDD_VI_LONG=1 cargo test --release vi_long     # 120 x 120 s

The recurrent-line tests in `src/harness/vi/recurrent.rs` keep the 768
synthetic adversarial cases paired with each speaker's baseline peak.
They include fault, recovery, another fault, control failures and changes
between bass and treble. The late-mute witness preserves all 3,999 oracle
and gain rows before 40 s, then mutes the tweeters and raises the woofer
program over 60.25 to 65 s. Its early-mute control runs too. These controls
and the long class assert each speaker's true V/I limit over every oracle
sample, including after a low-line fault and during hand-offs. The cold
burst/quiet engine test requires the deployed 0.3 V / 20 mA PF gate.

The broad family retains both baseline and reviewed-parent peak tables and
all input fingerprints. Its hard gate is each speaker's absolute V/I limit.
Changes relative to either historical table are reported separately; a
previous crossing is never an exemption. The earlier relative-peak predicate
is not the continuous thermal criterion. These are software model checks,
not physical speaker qualification. Nonzero twin operation remains disabled
and unqualified.

    cargo test --offline --locked --release recurrent
    cargo test --offline --locked --release cold_burst

### Continuous limits and the listening release gate

The original interleaving properties cover accounted or armed states. They
can pass while a true model temperature crosses its limit elsewhere in the
program. `src/harness/continuous.rs` adds a separate check of both thermal
nodes over every interval, including interior maxima, with no armed-state
mask. It retains all 30,144 paired inputs: 144 long programs, 20,000 V/I
seeds and 10,000 Legacy seeds. All 865 cases that fail either historical
source run as default regressions, including all 17 newly crossing cases.
The complete campaign is an additional mandatory acceptance run:

    PAIR_METRICS=continuous.csv cargo test --offline --locked --release harness::continuous::all_30144_continuous_cases_stay_within_limits -- --ignored --exact

The input fingerprints, complete counterexample rows, new-crossing rows and
historical limits are in `src/harness/`; see `README-continuous.md` there.
The historical archive contains 72 shared rows and two packaged-Neo rows
from a separate source/fixture. Those two rows are never counted as reruns
of a different fixture. The original Euler/RK4 property oracles remain;
the continuous check uses exact interval extrema and a parallel exact
Legacy state.

Thermal checks and the ordinary-listening budget must pass on the same
candidate. Listening acceptance uses the seven retained archives at cubic
30%, 60% and recorded level across three source comparisons (63 replays),
plus the retained quiet schedule. Cost must stay within 1 dB; J616 must stay
at exactly zero change at 30% and 60%. Comparisons include per-speaker means
and common normal-playing intervals. Private recordings and detailed replay
records stay outside this repository.

The current full-dose onset reserve still fails that listening budget. The
complete continuous campaign must also pass before release; narrower
property passes are insufficient. This branch is not ready to deploy. A
quiet capture is past audio, not a guarantee about upcoming output. No
quiet-history exception or reserve cap is used to weaken onset protection.

### Relation to speakersafetyd

speakersafetyd protects the speakers of the other Apple laptops from the
amplifiers' own current and voltage sense (V/ISENSE on the TAS/SN parts,
S16 sense words): it measures the power, so its model only integrates.  It
needs the VSENSE/ISENSE switches and an amplifier gain control, and on any
error it panics and is restarted cold by systemd.  The J700 has none of
that: a fixed-gain MAX98360A and a digital tap of the S32 words the
amplifier receives, so the current must be inferred (feed-forward).

For speakersafetyd to cover the J700 it would need, in order: an optional
current channel (I = V / z_nominal when `is_chan` is absent), S32 sense
words, optional amplifier controls, and then the parts that matter only
without measured current: the power budgets, charging unmeasured time, and
keeping the model across restarts.  The configuration keys here are
speakersafetyd's, so the J700 file would carry over.  That is a series to
propose upstream rather than a beta change; until it lands, this daemon is
the J700's protection.

### Building

Static, from any host:

    cargo build --release --target aarch64-unknown-linux-musl

The binary talks to the ALSA kernel interface directly (no alsa-lib).

### Installing

    make install

installs the binary, `conf/apple/*.conf`, the systemd unit and the udev
rule that starts it when a supported card's sense PCM appears.

Keep the unit's `RuntimeDirectory=speakerguardd` and
`RuntimeDirectoryPreserve=yes` when packaging: they give the daemon
`/run/speakerguardd` for its model state and keep it across restarts.  A unit
without them leaves the state unwritten, and every restart starts cold.

The PCM udev event and `multi-user.target` can both precede ALSA card
discovery. The daemon retries a missing card for up to 30 seconds, using a
monotonic deadline and 100 ms polling, before returning the discovery error.
This happens before loading the model or taking the interlock: the kernel
keeps the speakers at its safe level throughout the wait. Other discovery
errors (including permission and ioctl failures) still fail immediately.
`Restart=on-failure` and the saved thermal state remain in effect.

The unit explicitly passes `--wait-for-card 30`; 30 seconds is also the
binary's default for existing packaged units and manual starts. Override
`--wait-for-card <SECONDS>` to change the bound, or use `0` for a single
discovery attempt. A card that never appears still causes a failed start;
this does not bypass any protection check.

### Testing startup

On Linux, `cargo test --locked` includes simulated discovery deadlines and
the guard's existing fail-safe tests. The discovery tests also run without
Linux or audio devices:

    rustc --edition=2021 --test src/startup.rs -o /tmp/speakerguardd-startup-tests
    /tmp/speakerguardd-startup-tests
    python3 -m unittest discover -s tests -p 'test_*.py'

For the Neo, see [the three-reboot procedure](tests/neo-three-boots.md).
Its collector only reads service, journal, package and state information;
it never plays audio or performs a reboot.

### Running by hand

    speakerguardd -v

The default data directory is `/usr/share/speakerguardd`. The macaudio card
ID comes from the device tree through ALSA (`AppleJ413` selects
`apple/j413.conf`); that packaged file selects the governor, guard and
qualification state. A plain `speakerguardd` uses those choices and waits
up to 30 seconds for discovery. Unqualified profiles still require the
deliberate `--allow-unqualified` opt-in. The packaged unit omits redundant
default arguments; local units may continue to use them.

`-C <card id>` selects a card (by default the one the macaudio driver
registered, never the AOP card that shares its `AppleJ` prefix), `-s <file>` the
model state file (`-s ''` for none).  `--governor legacy|asahi|unified`
overrides the configuration's governor, `--guard asahi|none` its guard, and
`-m` retains its historical recording value; `--shadow` turns the shadow log on,
and `--experimental-unified` and `--allow-unqualified` are the two opt-ins
described above.

### Strict profile bounds

The full-scale hot-start regression now runs for 20 simulated minutes by
default on every installed profile. It checks every effective gain interval,
including lease expiry, from independently hot coil and magnet nodes.
The onset tests also cover every V/I profile from ambient and hot starts,
with Asahi alone and unified plus Asahi.

The legacy governor on V/I profiles now reserves full-scale energy using
its own feed-forward state and resistance before each output write. Its
Asahi shadow cannot supply that bound. The existing V/I reserve remains in effect too. The FF reserve shares the
V/I onset calculation and includes old queued audio, a possible renewed lease, idle
gaps and the kernel safe-level tail. It changes neither measured heat nor
power-budget history. Existing hard latches still apply. A range fault uses the hardware limit and its re-arm margin.
J700 keeps its legacy floor policy and independent hot-node initialization.

A draft profile whose declared fallback does not fit the reserve stays
locked. Passing these software sweeps does not qualify a machine or its
thermal constants. The output ceiling may attenuate quiet content too.

The listening-cost release gate is
`ordinary_quiet_programme_costs_at_most_one_db`. It applies a deterministic
quiet programme at normalized cubic 30% and 60% and measures each output's
mean attenuation during active playback after the first minute. The deployed
baseline has zero attenuation in those cold-start cases. The current standing
reserve fails this gate, so this branch is not ready for release. A proportional
bound needs enforceable knowledge of upcoming playback energy or a sufficiently
short enforced response deadline. The sense capture's backlog contains past
audio and cannot certify the next period's energy.


The strict legacy sweep uses the profile's declared amplifier maximum even
when it exceeds the sense full scale. Startup, uncovered time and budget
caps use that same configured bound. A saturated voltage word cannot prove
that the amplifier stopped at the sense rail; those frames use the declared
power bound in both thermal paths. The profiles remain unqualified.


The recurrent-line gate retains all 768 inputs and both historical reference
tables. Every output must meet its own hard limit, including cases that were
already over at a reference revision. The log also reports every increase
above a reviewed-parent peak. Composed changes to attenuation and handoff
can change those below-limit peaks; the historical comparison remains
visible alongside the absolute result. The listening-cost gate remains a
separate release requirement.

Runtime snapshots use model generation 3. A clean stop written before the
independent hot-node and voltage bounds is not evidence that its saved model
bounded the speakers. Those older snapshots are refused and startup stays
latched. Readers from before generation 3 also refuse it on rollback. PR #20's
Legacy clean-stop writer already uses generation 3; its synthetic writer
fixture remains readable here, and this branch does not introduce generation 4.
Historical
recordings remain readable, but an untrusted initial state starts the replay
latched and makes it diagnostic. Profile fingerprints are unchanged.
