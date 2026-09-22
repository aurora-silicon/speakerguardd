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
play. The kernel lease expires after 250 ms without renewal. For copy-time
software gain, the matching kernel must also invalidate and stop queued
speaker samples on revocation; lowering a mixer value alone is insufficient.
Scheduling and firmware add stop latency, which still requires measurement.
The configuration files use the same keys, so a speaker
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
has warmed up. The larger of the two reductions is written to the card's
speaker volume control. Because sense samples already contain the applied
gain, a power-budget violation adds attenuation to that gain. When the model
reaches `--max-reduction`, the daemon releases its lease and retains the
kernel's fallback while cooling. The option must be finite and within
(0, 20] dB, matching the J700 fallback.

There is no physical temperature/current feedback: the model depends on its constants. The
J700's constants - thermal resistances and time constants, coil temperature
limits, volts at full scale, coil resistance and the power budgets - are the
machine's own loudspeaker-manager tuning, read back through the public
AudioUnit API and recorded in the bring-up provenance record.  Without the
daemon the kernel locks the J700's speakers 20 dB below the amplifier's
full scale; a healthy daemon may allow more output according to its model.

No lease is acquired before a complete, timely sense period. Missing capture,
overrun, stale backlog, control errors, nonfinite state, or a lost lease fail
closed. Nonblocking short transfers are accumulated into a full period under
one absolute 200 ms deadline. Idle playback releases the lease. Each process
starts its thermal estimates at the configured hard temperature limit,
because restarting does not cool a speaker. This can keep output at the
fallback level during initial cooling.

These thermal/electrical estimates use recorded native tuning parameters.
They are not a validated displacement limiter. Full-scale acoustic safety,
stop latency, and restart behavior require authorized hardware validation;
host tests establish arithmetic and failure handling only.

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

`-C <card id>` selects a card, `-m <dB>` the reduction at which the daemon
gives up.
