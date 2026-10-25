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
whose model trails its write) or none starts every speaker at its coil limit
plus what the samples a dead run left queued can add, latched, so the card
stays with the kernel until the model has cooled under its working ceiling
(about 7 s on the J700).  A daemon that is late renewing the lock is covered by the kernel's
250 ms lock timeout, and the late time is charged like any other.

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
before it touches the card, which the unit does not restart on.  Governing
one of them takes an explicit enable: mask speakersafetyd (`systemctl mask
--now speakersafetyd`) and start this daemon with `--allow-unqualified`, or
use a drop-in that adds `Conflicts=speakersafetyd.service` together with
`OnFailure=speakersafetyd.service`.  To go back, unmask speakersafetyd and
reboot.  Keep speakersafetyd installed.

On those machines the governor that writes the card is `asahi`, the
behaviour Asahi Linux's speakersafetyd ships, reimplemented here from its
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
  it.  And over a period with at least 0.1 V and 20 mA rms, current that
  does not follow the voltage (sum(v i) under 0.2 of sqrt(sum(v^2)
  sum(i^2)); a live load of up to 60 degrees keeps it over 0.4) is no
  reading.  What V and I cannot tell apart is a line reading a fraction
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

On the V/I machines it governs under the asahi guard (`--governor unified
--guard asahi`, or `guard = asahi` in the configuration).  The guard
needs V/I sense on every speaker:

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
with the change events it saw, and the model the guard started from.  It stops at `--record-max-mb` (512 by default) and says so in
the file.

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
the reduction at which the card's safe level takes over, `-s <file>` the
model state file (`-s ''` for none).  `--governor legacy|asahi|unified`
overrides the configuration's governor, `--guard asahi|none` its guard, and
`-m` defaults to its `max_reduction`; `--shadow` turns the shadow log on,
and `--experimental-unified` and `--allow-unqualified` are the two opt-ins
described above.
