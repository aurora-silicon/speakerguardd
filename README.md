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

Time the daemon could not measure while the speakers play is not ignored and
is not fatal either:

* A period may be up to 100 ms late.  Beyond that, the model is charged for
  the unmeasured time at the worst case: a full-scale square wave at the gain
  the card currently applies.  A short gap (an xrun, stream start latency, a
  PipeWire reconfiguration) therefore costs at most a brief extra reduction,
  never protection.
* A stream silent for 500 ms is closed and reopened.  Without a complete
  period for longer than that, the daemon stops renewing the card's volume
  lock, and the kernel drops the speakers to its safe level within 250 ms.
  The lock is taken back after two complete periods in a row.

Exits are left for what the daemon cannot recover from (no card, a
configuration error, a nonfinite model); the speakers are then at the card's
safe level.  The unit restarts the daemon (`Restart=on-failure`, five
attempts a minute), and the model state it writes once a second to
`/run/speakerguardd/state` (`-s`), cooled by the time between the two runs,
makes the new run start as hot as the old one stopped.  An unreadable state
file is taken as a speaker at its working temperature limit.  These checks
cannot make scheduling and ioctls atomic: a daemon that is late renewing the
lock is covered by the kernel's 250 ms lock timeout, and the late time is
charged to the model like any other gap.

### Building

Static, from any host:

    cargo build --release --target aarch64-unknown-linux-musl

The binary talks to the ALSA kernel interface directly (no alsa-lib).

### Installing

    make install

installs the binary, `conf/apple/*.conf`, the systemd unit and the udev
rule that starts it when a supported card's sense PCM appears.

### Running by hand

    speakerguardd -c /usr/share/speakerguardd -v

`-C <card id>` selects a card, `-m <dB>` the reduction at which the card's
safe level takes over, `-s <file>` the model state file (`-s ''` for none).
