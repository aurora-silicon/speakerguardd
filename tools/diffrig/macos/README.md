# macOS reference host

Ownership: lane C.

This host runs the Mac's own protection unit (`aufx` `spp3`, `lsm2` or `atsp`, by `appl`) offline. It goes through the public AudioComponent API and never opens an audio device or plays anything. It applies that Mac's own configuration, read at run time from its tuning folder:
- the strip preset (ClassInfo);
- the spp3 sidecar;
- the property-strip box properties.

It renders the rig's signals in host slices and records, every slice:
- every read-only meter parameter the unit publishes, global and per part;
- the rendered audio;
- the telemetry it was fed.

Traces made with Apple tuning are Apple-derived data. They never enter this repository; keep them in a private directory (mode 700). Only the generator, the tools and synthetic-parameter results are committed.

## Files

| File | Purpose |
|---|---|
| `rig_host.m` | Offline host. It sets the preset, `--sidecar`, `--prop ID=u32`, `--plistprop ID=plist` and `--cfprop ID=json`. V/I comes from a file (`--tele`) or from the rig's speaker plant (`--plant plant.json`; every constant is required). `--pace X` keeps rendering at or below X times real time so the units' asynchronous log writers keep up. |
| `prefshim.c` | `DYLD_INSERT_LIBRARIES` shim, for this unsigned host process only. It answers the units' CoreFoundation preference lookups from `RIGPREF_<key>` environment variables, so their debug logging can be switched on without writing the user's defaults. It also redirects the log directory (`RIGREDIR_FROM` to `RIGREDIR_TO`). With `RIGPREF_TRACE=1` it reports every key looked up. |
| `au_discover.m` | Lists a unit's element counts, custom properties (with CF values) and read-only meters. |
| `capture.py` | The case list. It runs every case through `rig_host` and writes `trace.jsonl` in `../schema.json`, the per-quantum meter CSV, the audio and the unit's own logs. |
| `lsm2logs.py` | Decoder for the lsm2 native logs (record framing per the lsm2 behaviour specification). |

## Use

```sh
make                                   # builds rig_host, prefshim.dylib, au_discover
python3 ../signals/gen_signals.py "$SGV2_RIG_DIR/signals" [program.wav]
SGV2_RIG_DIR=... SGV2_RIG_BIN=$PWD python3 capture.py "$SGV2_RIG_DIR/traces" [case-substring ...]
python3 ../run.py "$SGV2_RIG_DIR/traces" "$SGV2_RIG_DIR/signals" OUT   # the Linux side, then compare
```

`capture.py` needs numpy. A case whose unit log has an `*_OVERWRITE.txt` file is rerun at a lower pace, because records were lost.

## Observed host facts

- spp3 and lsm2 instantiate and render offline from their factories; no device is needed.
- Open-loop spp3 results are bit-identical between macOS 26.6 and 27.0, and between an M2 and an M4 Pro host.
- The spp3 telemetry bus is 12 interleaved channels: voltage then current, per part.
- Without the unit's calibration object, spp3 ignores telemetry entirely.
- lsm2 has no telemetry bus on the MacBook Neo tuning.
- spp3 debug logging is a bit mask: 1 V/I, 2 coil temperature, 4 predicted displacement, 8 closed-loop displacement/error/resistance, 16 coefficients, 32 more displacement, 64 and 128 capacitance, 256 power management.
