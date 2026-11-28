# J873g integration

The J873g (Mac mini, M6) is a T8152 desktop Mac with a single full-range
internal speaker driven by one SN012776 (`ti,tas2764`) amplifier on I2C3
(`codec@38`) and an AOP-fed speaker stream (`apple,t8152-aop-audio`, S24_LE
playback, no extra speaker headroom).

Unlike the M5 laptops (J813, J815), the J873g ADT's `audio-speaker` declares
`speaker-protection = 0` and zero RX sense slots: no voltage or current sense
reaches the AP. The device tree omits `"sense"` from `aop_audio` `dma-names`
(aurora-silicon/linux#3434), so `macaudio` omits the Speaker Sense PCM and
`wiring.py` derives a sense-less 1-channel wiring (`sense_path = false`,
`amps = 1`, `sense_slots = 0`, plugin `sgv2#1ch`, voicing `sgv2#voicing-1ch`).

| | J873g |
|---|---|
| SoC / transport | T8152, `apple,t8152-aop-audio` (`aop-dma`, S24_LE) |
| Amps | 1 SN012776 (`codec@38`, amp level from Aura/ADT `apple,speaker-config`) |
| Sense | None (no `"sense"` DMA channel, no Speaker Sense PCM) |
| Tuning | AID48, `Atsp` (feed-forward thermal protection), no LSID |
| Plugins | `sgv2#voicing-1ch` -> `sgv2#1ch` |

## Prerequisites

- **Aura:** `dt_prepare_t8152()` must run `dt_set_speaker_safety()`,
  `kboot_audio_amp_config()`, and `kboot_audio_speaker_id()` so `codec@38`
  receives `apple,speaker-config`, `apple,bop-config`, and
  `apple,amp-dcblocker-config` from the Mac's ADT (aurora-silicon/aura#548 /
  #115 / #287).
- **Linux:** `apple,t8152-aop-audio` 3-channel binding, `macaudio` DT-driven
  AOP sense derivation and `macaudio_j873_cfg` enablement behind
  `"Speaker Protection"` (aurora-silicon/linux#3434).
- **macOS parity attestation for 26B123/26B131:** Landed in
  aurora-silicon/speakerguardd#286 (`packaging/session/parity.json` `26B123`
  and `26B131` blocks). The J873g `AID48` voicing key
  (`463b0cb03eb03b1c378da4bd4e82f036a7c50a4a6c1dafb1b07c071afb33b7b4`, law
  `5740f31784993ecbe6e1236cb26341430ca0c64e66fb596847ee5e0a9b28d3b0`) passes
  `aurora-speaker-tuning voicing-check --attestations` (`compiler-gaps: none`,
  worst louder `+0.143 dB` / quieter `-0.011 dB` at full volume; `+0.173 /
  -0.011 dB` across sliders `0.7 / 0.46 / 0.25`).

## Fixture

`packaging/session/fixtures/j873.json` holds `wiring.py`'s reading of the
compiled `t8152-j873g.dtb` (aurora-silicon/linux#3434) and the single
full-range lane structure of AID48's `Atsp` graph with generic box names and
no tuning values. `gen.py` generates `sgv2-j873.conf` and `60-sgv2-j873.conf`
from it.

The derived wiring hash is `d1d62e8a0aaaef3d`, matching the hash
`speakerguardd-selfcheck` derives on a live J873g.

## Hardware verification

Tested on Mac mini (M6, J873g, firmware 27.2) with Aura `1.0.0b17-gc83bcb7f`
(aurora-silicon/aura#548), Linux `7.3.0-aurora-j873-speaker+`
(aurora-silicon/linux#3434 `18a0e3bcd3f7`), and `speakerguardd` v2 `5bd2d652`
using the Mac's own `AID48` (`26B123`) bundle compiled with `--voicing-pre`:

- **Silent self-check:** `speakerguardd-selfcheck run --switch-interface`
  passed (`derived-nosense`, `wiring d1d62e8a0aaaef3d`, `params 23c445a123c3`),
  `Amp Gain Volume` capped at 15, `Speaker Volume` locked at floor, no Speaker
  Sense PCM.
- **420 s protected playback ladder (`0.3 / 0.6 / 1.0` for `60 / 60 / 300 s`,
  `-8 dBFS` RMS stereo pink noise) through `sgv2#voicing-1ch` -> `sgv2#1ch`:**
  - `39,990` engine records (`512`-frame quantum), `lost = 0`, `health_or = 0x0`.
  - Open-loop coil model climbed from `50.00 C` to `114.99 C` max (magnet max
    `84.52 C`, modeled power max `3.402 W`, gain `0.00 dB`), cooling smoothly
    after playback stopped.
  - `kill -9` on `speakerguardd` at `300.2 s` immediately logged
    `Speaker protection withdrawn: owner gone` and dropped `Speaker Volume` to
    `0`; on restart the supervisor offered the saved thermal state (`106.45 C`
    -> `105.63 C`) and re-granted in `0.488 s` (`0` starts refused).
  - Idle released cleanly to `Speaker Protection = 0`, `Speaker Volume = 0`.
  - `Amp Fault Status = 0,0,0` throughout, `0` kernel audio/DART errors, `0`
    failed units.
