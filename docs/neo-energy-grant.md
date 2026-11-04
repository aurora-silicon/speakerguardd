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
   retained control FD, stale replay, credit exhaustion, timeout and arbitrary
   owner death in KUnit and the host adapter. Log submitted/completed/refund
   counters and verify that pending paid energy is never forgiven. Check that
   the old magic ping cannot add energy or extend the grant's absolute expiry.
   Do not kill, pause, stop or disable the live governing daemon to inject a
   fault, even with the sink muted. Exactly one protection daemon governs the
   target throughout the lead's measurements.
5. Snapshot the user's saved sink volume/mute. At the lowest usable level,
   confirm the actual user convolver route, sense continuity, no lock lapses,
   no daemon restarts and mute on exhausted credit. Restore those saved values
   after each run. Never install a persistent volume override.
6. Only the lead proceeds to the requested sustained-pink test at 100%, using
   the exact protocol below. A passing synthetic reference or mixer gain alone
   is not an acoustic pass.
7. Retain logs and the failure/rollback evidence. Restore the prior kernel and
   reviewed daemon if any physical premise, account invariant or temperature
   bound fails. Keep the current governing protection active throughout the
   hand-off. Do not merge or change distro package pins from this procedure.

No target access, playback, microphone capture or deployment was performed by
the implementation worker. Independent review, physical qualification and the
<=1 dB measurement remain separate acceptance work.

## Exact sustained-pink comparison for the lead

This protocol is for a future reviewed, physically qualified candidate. With
this PR's NONE result, run only the protected fallback/control variant and
report the grant comparison as unmeasured. Do not turn NONE into a certificate
to execute the protocol. The two ordinary-listening suite failures also remain
open acceptance failures.

### Request and freeze the experiment

Submit each target operation through the controller's `hw_request`, with
`device="neo"`, `capability="execute"`, the lead's task identity, an explicit
timeout and the exact command/script and artifact paths. Wait for policy
approval and resource/ownership admission. Use the controller's supported
upload/download capabilities for scripts and private evidence. Do not open a
separate transport. The lead owns these requests; this document grants no
deployment permission.

Before any sound, retain a private manifest with:

- Controller request and approval, device/card identities, boot ID, kernel
  commit/build hash, daemon commit/binary hash, effective service command and
  sole protection owner. Preserve the current known-good boot entry, its
  bootguard fallback and the reviewed daemon for rollback.
- Complete qualified transfer certificate and review identity, route/epoch,
  slot map, `tail_us`, profile/model hash and every unchanged working/hard
  limit. Record the actual payload format, qualified flags and live receipts;
  the presence of the BYTES control alone does not qualify anything.
- PipeWire/WirePlumber/UCM versions, graph/config hashes, speaker sink identity,
  route and graph gains, microphone device/gain/processing and fixed geometry.
  Use the same HQ array or fixed external microphone for all runs. Keep mic
  automatic gain/noise processing fixed; record its state.
- A licensed, fixed, stereo 48 kHz, 10-second pink block near -6 dBFS RMS,
  with no sample exceeding full scale. Retain provenance, SHA256, measured RMS,
  peak and any source clipping. Use exactly this block for every trial; do not
  normalize each trial separately. Freeze the repeated files once, off-target:

  ```sh
  ffmpeg -nostdin -n -stream_loop 1 -i pink-10s.wav -t 20 \
    -ar 48000 -ac 2 -c:a pcm_s32le pink-reference.wav
  ffmpeg -nostdin -n -stream_loop 23 -i pink-10s.wav -t 240 \
    -ar 48000 -ac 2 -c:a pcm_s32le pink-sustain.wav
  sha256sum pink-10s.wav pink-reference.wav pink-sustain.wav
  ```

Verify the 20-second reference is the sample-identical prefix of the sustained
file. A single 240-second playback avoids reopen gaps between repeats. Retain
the source and private microphone recordings locally, not in a public PR.

Ensure the already reviewed governing daemon records format 4 receipts and
grants (`--record <fresh-private-path> --record-max-mb 512`); preserve any
existing ring and sidecars first using `docs/recording-retention.md`. Do not
start a second daemon to record. Size/copy the ring so the complete experiment
survives rotation. Retain all `E`, `G`, `C`, `H`, `A`, `L`, `X` and `Z` records.
If recording fails, protection continues, but that trial has incomplete
acceptance evidence. A sense-only replay cannot certify a grant run.

### Cool, check low level, then record one trial

First use muted/null output for the qualification checks, then the lowest
usable volume for route and microphone checks. The 100% setting below is only
for the lead's explicitly approved loudness trial. Never write raw ALSA gain,
unlock or grant controls to obtain a louder reference.

With the protection daemon governing, allow at least 10 minutes with no test
programme, then require all available model nodes to change by less than
0.2 C over 60 seconds. Record those cooled states and ambient conditions. For
every subsequent trial, recover to within 1 C of those states with the same
stability criterion. These define comparable thermal starts, not new thermal
limits. Do not reset, edit or delete model/persistence state to simulate cooling.

The following is the command sequence inside a policy-approved Neo execute
request. The lead supplies `AUDIO_USER`/`AUDIO_UID` for the configured audio
server, its numeric `SINK` object for `audio_effect.j700-convolver`, a fresh
private `RUN` directory, and `PINK_WAV`/`SECONDS_AUDIO` = reference/20 or
sustain/240. The stimulus path must be readable by the audio client.
Run the shell as the approved operator; only the audio client uses its account.

```bash
set -euo pipefail
: "${AUDIO_USER:?}" "${AUDIO_UID:?}" "${SINK:?}" "${RUN:?}"
: "${PINK_WAV:?}" "${SECONDS_AUDIO:?}"
umask 077
mkdir -m 700 "$RUN"                 # must be fresh; preserve failed trials
audio() { runuser -u "$AUDIO_USER" -- env \
  XDG_RUNTIME_DIR="/run/user/$AUDIO_UID" "$@"; }
audio wpctl inspect "$SINK" > "$RUN/sink.txt"
rg -q 'node.name = "audio_effect.j700-convolver"' "$RUN/sink.txt"
audio wpctl get-volume "$SINK" > "$RUN/volume.before"
saved_volume=$(awk '{print $2}' "$RUN/volume.before")
saved_mute=0
if rg -q MUTED "$RUN/volume.before"; then saved_mute=1; fi
player= sampler= capture=
cleanup() {
  # These are only this trial's player, sampler and recorder, never the guard.
  for child in "$player" "$sampler" "$capture"; do
    if [ -n "$child" ]; then kill -TERM "$child" 2>/dev/null || true; fi
  done
  wait || true
  audio wpctl set-volume "$SINK" "$saved_volume"
  audio wpctl set-mute "$SINK" "$saved_mute"
  audio wpctl get-volume "$SINK" > "$RUN/volume.after"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
systemctl is-active --quiet speakerguardd
systemctl show speakerguardd -p MainPID -p NRestarts > "$RUN/guard.before"
cat /proc/sys/kernel/random/boot_id > "$RUN/boot-id"
sha256sum "$PINK_WAV" > "$RUN/stimulus.sha256"
journalctl -b -u speakerguardd -o short-monotonic > "$RUN/journal.before"
journalctl -b -k -o short-monotonic > "$RUN/kernel.before"
audio wpctl set-mute "$SINK" 1
audio wpctl set-volume "$SINK" 1.0
arecord -q -D hw:AppleJ700AOP,1 -t raw -f FLOAT_LE -c 2 -r 48000 \
  -d "$((SECONDS_AUDIO + 4))" "$RUN/mic.f32le" \
  2> "$RUN/capture.stderr" &
capture=$!
sleep 1
audio wpctl set-mute "$SINK" 0
cat /proc/uptime > "$RUN/player-start.bracket.before"
audio pw-play --target audio_effect.j700-convolver "$PINK_WAV" \
  > "$RUN/player.stdout" 2> "$RUN/player.stderr" &
player=$!
(
  while kill -0 "$player" 2>/dev/null; do
    cat /proc/uptime
    if ! systemctl is-active --quiet speakerguardd; then
      touch "$RUN/INVALID-protection"
      audio wpctl set-mute "$SINK" 1
      kill -TERM "$player" 2>/dev/null || true
      break
    fi
    cat /run/speakerguardd/state
    amixer -c AppleJ700 cget name='Speaker Playback Volume'
    amixer -c AppleJ700 cget name='Speaker Volume Unlock'
    if ! amixer -c AppleJ700 cget name='Speaker Energy Grant'; then
      printf 'energy receipt unavailable; protected control trial only\n'
    fi
    sleep 0.25
  done
) > "$RUN/telemetry.txt" 2> "$RUN/telemetry.stderr" &
sampler=$!
wait "$player"; player=
cat /proc/uptime > "$RUN/player-end.bracket.after"
wait "$sampler"; sampler=
wait "$capture"; capture=
systemctl show speakerguardd -p MainPID -p NRestarts > "$RUN/guard.after"
journalctl -b -u speakerguardd -o short-monotonic > "$RUN/journal.after"
journalctl -b -k -o short-monotonic > "$RUN/kernel.after"
```

Retain the controller result/exit and stderr, including ALSA overruns and
missing controls. The current NONE kernel may have no optional control at all;
that makes the control trial unqualified, not an invitation to add one. A
guard PID/restart change, lock lapse, hard latch, missing receipt/state interval,
unexpected route/epoch, clipping at capture or lost capture invalidates a
loudness trial. Stop only the test player and mute its sink on such a failure;
leave protection governing. If a controller timeout prevents the restoration
trap, use the retained `volume.before` in a new approved restoration request.
Compare `volume.after` with it, including both muted and unmuted cases.

Run three cooled 20-second references, each after recovery. Then run three
240-second sustained trials with identical recovery between them. Repeat the
same protected protocol for the known-good control if an A/B comparison is
needed; each candidate has its own cooled reference. Do not change graphs or
route gains between references and sustained trials.

### Align, calculate and decide

Align each microphone recording to the first emitted source frame using the
known repeated source (cross-correlation/acoustic onset), retaining the offset
and its uncertainty. `/proc/uptime` and receipt boottime align state/journal
events; the player launch bracket alone is not an acoustic timestamp. Fixed
geometry and unchanged capture processing are required. Keep a second of
pre-roll for room/noise measurement.

For each channel compute DC-removed RMS on source-aligned [5,15) seconds for
each reference. Average the three **powers**, then convert to dBFS to obtain
`L_cooled`. For each sustained trial use [5+10j,15+10j), j=0..22. Each window
has the same phase/content of the ten-second block and excludes stream edges.
Compute `loss_db = L_cooled - 20*log10(rms_sustained)` for both channels. Also
report every 10-second RMS and peak, the initial/final thermal states and the
worst loss per trial/channel. Do not average away the worst sustained window.

Decode receipts/grants using `include/uapi/sound/apple-speaker.h` and
`src/playback.rs`, with Q20 normalized frames at 48 kHz. For every window
report source/submitted/completed/refunded deltas, pending `submitted-completed`,
unspent ceiling, expiry, grant sample gains, residual reservation, card gain
and every applicable thermal model/limit. Include copy-time sample attenuation
in the loss account; zero mixer attenuation alone proves no improvement.
No pending or residual debt may disappear across a stream/epoch change.

Mark the first production thermal-governor limit decision from the recorded
models/decisions. If those records cannot distinguish genuine model limiting
from floor/grant attenuation, the exemption is unproved: report that interval
as unclassified, never as a pass. Use all complete windows strictly before
that first decision for the <=1 dB test. Retain and explain all later windows;
do not exempt them merely because output got quieter. If limiting begins
before the first complete window, this trial supplies no pre-limit witness.

Acceptance requires every pre-limit window in both channels and all three
trials to lose at most 1 dB against the cooled reference, including measurement
uncertainty. Require the three cooled references to agree within 0.3 dB per
channel, no capture clipping and at least 20 dB signal/noise separation;
otherwise improve measurement repeatability and report inconclusive. Keep the
unchanged hard/working limits, all ledger invariants and protection checks
passing. A synthetic 0 dB result cannot satisfy this acoustic criterion.

On a safety/account failure mute and stop the test source immediately, retain
all evidence, then use the lead's approved orderly rollback to the preserved
kernel/daemon with the existing safe-level/bootguard handoff. Do not power-cut
the Neo or disable either protection path. Restore the saved sink volume and
mute and verify the known-good governing service. Rollback/deployment details
and machine-specific service overrides stay private and local. Neither PR is
merged and no package pin is changed by this procedure.
