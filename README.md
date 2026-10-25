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
speaker volume control.  If it would exceed `--max-reduction` (20 dB by
default: a model asking for more is wrong), the daemon stops renewing the
lock and the card falls back to its own safe level; the daemon keeps
modelling from the sense data and takes the lock back once the reduction is
3 dB below the limit again.

The J700 converts on copy, ahead of a playback queue of up to 341 ms, so a
written reduction reaches only what is converted after it.  Audio already
queued at a lower gain keeps playing, and only giving the lock up stops it
(macaudio stops the old-gain stream at once).  The daemon gives the lock up
for that reason too, in two cases, each logged as itself:
* Any program at all could take a coil within 3 K of its limit before its
  next look, through the least reduction the card may be playing.  This is
  what it has to assume while it cannot see.
* The coils' thermal reduction has risen more than 3 dB over that least
  reduction, and the audio converted at it, at the level the program was
  last seen to deliver, could take a coil there in the time it can still
  play: until a queue's length (341 ms) after the controls stopped holding
  it.
It takes the lock back after two whole periods.

A daemon that is held up (a call that blocks, the scheduler) cannot write
anything: the card keeps converting at the last reduction written until the
kernel's lease runs out and macaudio stops the stream.  So the daemon never
writes less than a floor: the reduction at which full scale for a lease, a
period and the stop, behind a copy queue (0.68 s on the J700), keeps every
coil 3 K under its limit.  It rises with the coil, and the governor takes
over from it at about 110 C.  This costs loudness at the top of the volume
range, accepted on purpose:

| Coil model (its magnet where a steady program puts it) | 50 C | 70 C | 85 C | 100 C | 110 C |
| --- | ---: | ---: | ---: | ---: | ---: |
| Floor | 0.5 dB | 1.5 dB | 2.5 dB | 3.9 dB | 5.0 dB |
| Floor on the J700's 0.5 dB steps | 0.5 dB | 2.0 dB | 3.0 dB | 4.0 dB | 5.5 dB |
| Governor | 0 dB | 0 dB | 0 dB | 0 dB | 6.0 dB |

The card rounds every reduction up to its step, the floor's too.  A cold
J700 never plays at the amplifier's full scale, but 0.5 dB under it.  A
loud program that warms the coils to 85-100 C plays 3-4 dB under it where
the governor alone would take nothing yet.  In return, loud bursts
after quiet passages are not stopped (no hand-off in any of the programs the
seventh review measured, where the queue rule alone stopped up to 60 a
minute, each an XRUN and about 170 ms at the card's safe level), and a
daemon held up for anything up to the lease no longer takes a coil past its
limit.

While the daemon is held up (descheduled, blocked in a call, waiting for a
page) it can decide nothing, and the card's 250 ms lease bounds what plays,
which the floor allows for.  To make that rarer the unit runs it at a
real-time priority below PipeWire's
(`CPUSchedulingPolicy=fifo`, priority 50) and it locks its memory
(`mlockall`).  Every ten minutes it logs how long it was held up: the gaps
between lock renewals, the latency of each kind of call, and the major page
faults.  That measures, without a sound, whether a shorter lease is needed.

There is no feedback: the model can only be as good as its constants.  The
J700's constants - thermal resistances and time constants, coil temperature
limits, volts at full scale, coil resistance and the power budgets - are the
machine's own loudspeaker-manager tuning, read back through the public
AudioUnit API and recorded in the bring-up provenance record.  Without the
daemon the kernel locks the J700's speakers 20 dB below the amplifier's
full scale; with it, the model governs up to full scale.

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
  card applies, rounded up to its steps; after a failed write, anything
  from the old value to the new one rounded up), the card's own safe level
  from the moment the daemon's lease may have run out, and anything for the
  controls' value before the daemon set one.  A reduction too deep to have
  delivered a frame that loud did not convert it; a word is taken a step
  above itself; nothing before the gain is louder than full scale.  A budget window that
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
* The floor under every write (see above): 0.5 dB on a cold coil, rising
  to 5 dB at 110 C.

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
whose model trails its write) or none starts every speaker at its coil limit
plus what the samples a dead run left queued can add, latched, so the card
stays with the kernel until the model has cooled under its working ceiling
(about 7 s on the J700).  A daemon that is late renewing the lock is covered by the kernel's
250 ms lock timeout, and the late time is charged like any other.

### Randomized interleavings

`src/harness.rs` runs the real guard against a simulated world: streams of
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
stop).  Where macaudio stops old-gain audio promptly, it also asserts that
the true coil never reaches its limit unless the daemon was held up
(descheduled, blocked in a call or gone) for at least 85 ms, ending no
longer before the crossing than the lease, a period and the stop.  That
bound is the kernel's lease, not the daemon's, and the crossings it exempts
are counted.  A safe write that fails falls back to the controls' minimum,
as the daemon does.  Scenarios start cold
or hot, at 16 kHz or at the J700's own 48 kHz and 4096 frames, with a prompt
or a slow stop.  Failures shrink to a minimal scenario printed as Rust; each
one it found is a test of its own.

    cargo test --release harness                      # 200 cases
    SPEAKERGUARDD_CASES=1000000 cargo test --release harness

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

    speakerguardd -c /usr/share/speakerguardd -v

`-C <card id>` selects a card (by default the one the macaudio driver
registered, never the AOP card that shares its `AppleJ` prefix), `-m <dB>`
the reduction at which the card's safe level takes over (the floor under
every write is not counted against it, so on a warm coil a `-m` under about
4.5 dB can be exceeded by the floor without a hand-off: more reduction, not
less), `-s <file>` the
model state file (`-s ''` for none).  `-V` prints the version and the
source commit, which the daemon also logs when it starts (`make` takes the
commit from git; a package sets `SPEAKERGUARDD_COMMIT`).

The daemon writes the controls to the card's safe value at its start and
before each unlock.  The card accepts that value while the volume is locked.
Until the first unlock the speakers play at the card's safe level, as they
do without the daemon, and an unlock never lets anything louder through.
At the first ping the daemon reads the controls' range with the lock lifted
and checks the card's safe level (the locked maximum against it) against
`safe_reduction`.  A card that is louder there than the configuration says
is never unlocked: the speakers stay at its safe level.

The hot-start backport bounds the J700 coil and magnet independently,
including the possible queued full-scale drain. An unknown prior session
keeps the hard latch until this envelope has cooled. The existing floor,
queue rules and kernel safe level remain in effect. A no-device 20-minute
full-scale regression starts both truth nodes at 135 C.

The controls are held at minimum before the first unlock and each retake.
Even the kernel safe level can add heat when both nodes start at the hard
limit. The minimum write prevents that startup dose. Kernel fallback,
copy-queue accounting and the running output floor are unchanged.
