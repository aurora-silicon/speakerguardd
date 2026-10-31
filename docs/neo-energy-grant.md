# Neo energy grant: review and lead test plan

The Neo's user PCM is S32_LE, but its DMA payload is binary32 with 0.5 at wire
full scale. The new experimental format `0x1000000e` identifies those immutable
payloads. It is not ordinary ALSA FLOAT_LE. The daemon accepts it only for the
AppleJ700 two-channel feed-forward profile and only with the existing complete
kernel qualification, ownership, epoch and transfer checks.

The kernel's integer-only meter bills the actual emitted float word. Every
cyclic repeat is copied and charged anew. Copy-time volume is never treated as
an independent hardware cap: Neo copies cannot be cached raw or refunded when a
later control value becomes safe. Exhaustion and expiry mute subsequent private
copies; previously submitted copies remain paid debt. The production grant
model already reserves queued/unspent energy and downstream residuals and uses
absolute cumulative ceilings, not additive top-ups. Its temperature proof is in
`playback_grant.rs` and the companion kernel's
`Documentation/sound/soc/apple-neo-energy-grant.rst`.

**No real Neo transfer is qualified by this work.** The kernel returns NONE
until a uniform bound through LEAP/AOP and the analogue amplifier exists. Its
certificate must cover arbitrary prior state, all allowed transitions and the
residual tail; a successful tone probe is insufficient. An available bytes
control, supported payload format or packaged card match alone cannot remove
the current floor. Unqualified kernels/cards keep the existing reserve. No
thermal limit, listening assertion, protection service or ownership check is
relaxed. This branch is based on the evidence-start RC (#28), not the deployed
interim line (#29). Keep the PR draft and do not install it without independent
safety review.

## Software acceptance

- Run the companion kernel's KUnit configuration, including real macaudio
  control handlers, immutable-copy ownership, integer/float metering, stale
  grant, owner death, expiry, exhausted credit and failed-stop quarantine.
- Run this daemon's full standard suite and explicit continuous grant campaign.
  The two existing ordinary-listening failures remain required failures, not
  expected failures. Account sample attenuation as well as mixer attenuation.
- Retain the exact source hashes, complete logs, failed attempts and results.
  The reference C host adapter executes the pinned kernel meter/payload helper;
  its synthetic certificate grants no hardware permission.
- Review the physical transfer and thermal premises independently before
  changing the kernel's NONE result. Never provide a force-enable option.

## Lead-owned Neo procedure, after review

1. Through the controller, identify `device=neo`, task, exact kernel and daemon
   commits, protection owner, selected card and speaker profile. Take a policy
   approved hardware lease. Preserve the known-good kernel, daemon and bootguard
   fallback and prepare an atomic rollback.
2. Start muted or on a null output. Read the card capability and every transfer
   premise. Run an unqualified-path check: no grant, original safe volume,
   original floor, original daemon-death behavior. Keep exactly one governing
   protection daemon at every point.
3. Qualify the output bound with the lead's approved measurement procedure at
   the lowest usable level. Cover payload-to-tap mapping, sample signs, full
   scale, rate/route transitions, previously populated state, muted refills and
   the entire residual tail. A finite measurement alone is not a universal
   certificate; pair it with a reviewed bound. No direct coefficient guessing
   or inverse reconstruction from sense-only historical recordings.
4. With a reviewed certificate and both PRs still pinned, exercise stop, stall,
   retained control FD, stale replay, credit exhaustion, timeout and owner
   death in a null-output or muted setup first. Log submitted/completed/refund
   counters and verify that pending paid energy is never forgiven. Check that
   the old magic ping cannot add energy or extend the grant's absolute expiry.
   Do not disable or weaken protection to inject a fault.
5. Snapshot the user's saved sink volume/mute. At the lowest usable level,
   confirm the actual user convolver route, sense continuity, no lock lapses,
   no daemon restarts and mute on exhausted credit. Restore those saved values
   after each run. Never install a persistent volume override.
6. Only the lead proceeds to the requested sustained-pink test at 100%: use
   identical source, graph, mic geometry and electrical metering for a cooled
   reference and the sustained run. Record 10-second acoustic RMS, card gain,
   sample gain/energy, coil/magnet states and all residual debt. Compare the
   pre-thermal-limiting portion against the cooled reference; require <=1 dB
   loss and explain any interval where the thermal model actually limits.
   A passing synthetic reference or mixer gain alone is not an acoustic pass.
7. Retain logs and the failure/rollback evidence. Restore the prior kernel and
   reviewed daemon if any physical premise, account invariant or temperature
   bound fails. Keep the current governing protection active throughout the
   hand-off. Do not merge or change distro package pins from this procedure.

No target access, playback, microphone capture or deployment was performed by
the implementation worker. Independent review, physical qualification and the
<=1 dB measurement remain separate acceptance work.
