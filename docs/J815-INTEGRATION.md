# J815 integration

The J815 (MacBook Air 15-inch, M5) is a T8142 Mac like the J813. It uses the
same T8142 configuration of Linux's `t8152_aop_audio` driver and the same
transport entry (`apple,t8142-aop-audio`: S24_LE playback in 32-bit
containers, S16_LE sense, non-batched sense, no extra speaker headroom). See
[J813-INTEGRATION.md](J813-INTEGRATION.md) for the transport and the
capture-first silent check.

Where it differs from the J813 is the speaker system and the tuning:

| | J813 | J815 |
|---|---|---|
| Amps | 4 SN012776 | 6 SN012776 |
| Order | LW, RW, LT, RT | LW1, RW1, LT, RT, LW2, RW2 |
| Sense | 8 channels | 12 channels, `[V, I]` per amp (vmon slot below imon) |
| Tuning | AID42, LSM2 (with LSID) | AID43, SPP3, no LSID |
| Plugin | `4ch` | `6ch-vi` |

Because AID43 uses SPP3, which is in `units_at_parity`, the J815 qualifies
the same way as every other SPP3 Mac. It needs no entry in `gate.json`, and
it has none. The J815's speaker tuning is the same one macOS uses on the
J615 and J415, the earlier 15-inch Airs.

## Fixture

`packaging/session/fixtures/j815.json` holds wiring.py's reading of the
compiled `t8142-j815.dtb` (aurora-silicon/linux#1534) and the lane structure
of AID43's graph with generic box names and no tuning values: woofers on unit
channels 0, 1, 4 and 5, tweeters on 2 and 3. `gen.py` generates
`sgv2-j815.conf` and `60-sgv2-j815.conf` from it.

The derived wiring hash is `725f518662e42388`. That is the same hash the
Mac's own self-check derives from the live device tree and the owner's
collected AID43 bundle.

## What has been checked on the hardware

These results come from one J815 with its own AID43 bundle, collected from
its macOS install (26B123, and later 26B131), on 7.3.0 J815 kernels:

- `aurora-speaker-upgrade --check` passed. The blob compiled as SPP3 with 6
  channels, 12 sense channels at 48 kHz, and no refusals. The silent hardware
  switch test passed. The voicing self-check came out within ±0.017 dB of the
  macOS 25G83 Audio Unit references.
- After the switch, the supervisor logged `self-check: j815 wiring
  725f518662e42388 (derived)` and then `protection ready: every check holds,
  granted while a stream plays`. Protection was granted and released for
  each stream, and the owner reports normal playback through the protected
  graph.
- After the Mac's boot firmware moved from the macOS 27.0 stub to the 27.2
  stub, the check and the switch passed again. The boot after that switch
  was silent because `/etc/speakerguardd` had been created as `0700`, which
  is not specific to the J815 (#186).

Not checked yet: a long run at full volume with thermal logging, and the
behavior when a stream starts or restarts late. The J813 notes on late
capture apply here too.

## Attribution

Thomas Glanzmann (`sithglan`) authored the T8142 Linux audio support that
the J815 shares with the J813. Acelogic added the T8142 transport description
and the continuous-capture fix. Ryan Murray and the Aurora contributors
designed the v2 wiring, self-check and kernel ownership.
