# J716c integration

The J716c (MacBook Pro 16-inch, M5 Max) is a T6050 Mac. Its speaker path uses
the T6050 configuration of Linux's `t8152_aop_audio` driver (`apple,t6050-aop-audio`,
which shares the `apple,t8142-aop-audio` transport entry: S24_LE playback in
32-bit containers, S16_LE sense, PipeWire non-batched sense override, and no
extra speaker headroom).

| | J716c |
|---|---|
| Amps | 6 SN012776 (`i2c1` left: 0x38/0x39/0x3a, `i2c3` right: 0x3b/0x3c/0x3d) |
| Order | LW1, RW1, LW2, RW2, LT, RT |
| Sense | 12 channels, `[V, I]` per amp (vmon slot below imon: 0/2, 4/6, 8/10, 12/14, 16/18, 20/22) |
| Tuning | AID41, SPP3, no LSID |
| Plugin | `6ch-vi` |

AID41 uses SPP3, which is in `units_at_parity`. As on every SPP3 Mac, J716c
needs no board allowlist entry in `gate.json`; runtime **qualification** is
derived from verified provisioning of the machine's own bundle and a passing
silent hardware self-check (so manual provisioning or a previously switched
system can still reach v2 playback). Automatic boot-time **migration** in
`aurora-speaker-upgrade` separately requires `AID41` playback rows on record
in `gate.json`.

## Fixture

`packaging/session/fixtures/j716.json` holds `wiring.py`'s reading of the
compiled `t6050-j716c.dtb` and the lane structure of AID41's graph with generic
box names and no tuning values: woofers on unit channels 0, 1, 2 and 3,
tweeters on 4 and 5. `gen.py` generates `packaging/pipewire/sgv2-j716.conf` and
`packaging/wireplumber/60-sgv2-j716.conf` from it.

The derived wiring hash is `cbba18ec53f0a442`. That is the same hash the Mac's
own self-check derives from the live device tree and the machine's own tuning
bundle.

## What has been checked on the hardware

These results come from one J716c (16-inch M5 Max) running kernels built off
`aurora-silicon/linux` `main` (`d420ae2bd61a`) with the `t8152_aop_audio`
`SenseRing` fix (`c3feb10d6265`), T6050 PMP (loader hand-off via `aura`
`2aa4d909`), AOP audio, jack codec, six SN012776 amps, and v2-only
`apple,j716-macaudio` (`df9f3c9377f6`, trial kernel `7.3.0-ewb298+`, plus
earlier GPU-coexistence checks on `7.3.0-ewb293+`):

- `aurora-speaker-upgrade` collected and compiled this machine's `AID41`
  (`speaker_id=0`) tuning bundle with unit-bound calibration active (SPP3,
  6 channels, 12 sense channels, 48 kHz) and matched the six amplifiers'
  operating gain readback from the device tree.
- `speakerguardd-selfcheck` derived wiring `cbba18ec53f0a442` (`method=derived`)
  and passed both the regmap slot check and the silent 12-channel V/I sense
  switch test across all six SN012776 amplifiers.
- `speakerguardd-selfcheck verify` passed all four checks on `7.3.0-ewb298+`
  (v2-only `apple,j716-macaudio` + `SenseRing`):
  - `kernel`: `AppleJ716: v2 interface only`
  - `hardware`: `j716c 7.3.0-ewb298+ wiring cbba18ec53f0a442 (derived)`
  - `voicing`: `cec53151459a (AID41)` within +0.051 / -0.048 dB of reference
    (sliders 0.7 / 0.46 / 0.25 within +0.137 / -0.031 dB)
  - `graph`: `512-frame quantum locked; 5 silent pilot starts, no xrun on
    sgv2.sense.j716, sgv2.speaker.j716`
- 12-channel V/I sense slot-alignment stress probe on `7.3.0-ewb298+`: 10/10
  consecutive L and R 200 Hz bursts — across fresh starts, two 13 s idle
  `pwrd -> pw0 -> pwrd` cycles, and a mid-burst `kill -9` supervisor restart —
  stayed locked with zero channel shift (`sense_delay = 1600`, `health = 0x0`,
  `0,0,0` faults across all six amps, zero cross-channel leakage).
- Full `AID41` protected playback ladder on `7.3.0-ewb298+` (`0.3 / 0.6 / 1.0`
  for `60 / 60 / 300 s`, `-6 dBFS` pink noise, 3-ch `AppleJ716HPAI` mic capture,
  `kill -9` at +150 s of the 1.0 hold):
  - `39,762` telemetry records, `lost = 0`, `health_or = 0x0`,
    `sense_delay = 1600..1632`, `0,0,0` amp fault status on all six amps.
  - Open-loop model max: `103.9 / 103.8 / 103.9 / 103.8 C` (woofers),
    `58.0 / 58.1 C` (tweeters); model power max `7.74 / 7.66 / 7.74 / 7.66 W`
    (woofers), `0.30 / 0.31 W` (tweeters).
  - Closed-loop V/I sense fresh max: `87.6 / 86.4 / 84.1 / 85.9 C` (woofers),
    `45.8 / 47.9 C` (tweeters); median coil resistance stayed aligned across
    all six channels both before `kill -9`
    (`3.895 / 3.884 / 3.917 / 3.928 / 3.421 / 3.428 ohm`) and after `kill -9`
    (`4.008 / 3.996 / 4.015 / 4.040 / 3.426 / 3.452 ohm`).
  - Internal 3-ch mic capture during the `1.0` hold: `[-39.2, -35.9, -36.5] dBFS`.
  - Supervisor `kill -9` at +150 s withdrew protection immediately (`owner gone`)
    and re-granted in `1.254 s` with `0,0,0` faults across all six amps.
- Reboot persistence: a second boot reused the bound blob and self-check record
  without re-running the switch test and brought `speakerguardd-v2.service` up
  directly in `protection ready`.
- v2-only fail-closed behavior: `Speaker Volume Unlock` and
  `Speaker Protection Interface` are absent; with `speakerguardd-v2.service`
  stopped (checked both immediately and after >35 s), all six speaker volumes
  stay at `0` (mute), `amixer cset` on speaker volume and amp gain returns
  `Operation not permitted`, and raw playback to `hw:AppleJ716,1` fails
  immediately (`Input/output error`, incrementing `speaker_starts_refused`)
  without reverting to legacy unlock.
- System sleep (`s2idle`, ~6 min, `7.3.0-ewb293+`): `speakerguardd-v2-sleep.service`
  unlinked the amps, closed the PCMs, and stopped the supervisor before freeze;
  after resume it cleared the hold, restarted the supervisor with saved thermal
  state, and post-resume stereo playback (440 Hz L / 660 Hz R, slider 0.45)
  measured +23.3 dB / +21.4 dB vs -44.9 dB at 550 Hz on `AppleJ716HPAI` with
  `0,0,0` fault status across all six amps.
- `AID41` is recorded in `packaging/session/gate.json` (`playback_rows`) with
  `loud-step` and `thermal-ramp` coverage.
