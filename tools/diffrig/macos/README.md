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
| `rig_host.m` | Offline host. It sets the preset, `--sidecar`, `--prop ID=u32`, `--plistprop ID=plist` and `--cfprop ID=json`. Telemetry comes from a file (`--tele`) or, for the existing SPP3 V/I path, the rig's speaker plant (`--plant plant.json`; every constant is required). `--pace X` keeps rendering at or below X times real time so the units' asynchronous log writers keep up. |
| `prefshim.c` | `DYLD_INSERT_LIBRARIES` shim, for this unsigned host process only. It answers the units' CoreFoundation preference lookups from `RIGPREF_<key>` environment variables, so their debug logging can be switched on without writing the user's defaults. It also redirects the log directory (`RIGREDIR_FROM` to `RIGREDIR_TO`). With `RIGPREF_TRACE=1` it reports every key looked up. |
| `au_discover.m` | Lists a unit's element counts, custom properties (with CF values) and read-only meters. |
| `capture.py` | The case list. It runs every case through `rig_host` and writes `trace.jsonl` in `../schema.json`, the per-quantum meter CSV, the audio and the unit's own logs. |
| `capture_edge.py` | Edge-case list: atsp on each AID's own preset (no meters; the trace carries the audio gain and per-slice sample-gain extremes), spp3 closed-loop sense faults and gates (`spp3-cl-edge-*`), and fractional-Rampout AIDs (`spp3-rampout-*`). Its spp3 closed-loop cases decode the unit's coil-temperature and controller logs into `clsp.npz` and require every record (they rerun at a lower pace otherwise). Outputs are gzipped; `compare.py` and `run.py` read the `.gz` forms. |
| `test_rig_host.py` | Exit-status tests for `rig_host` and `prefshim` after `make`: argument bounds, refused or unreadable `--cfprop`, render errors (`--test-fail-at`), short renders and whole-component redirects. It renders a short synthetic signal offline into a temporary directory and deletes it. |
| `lsm2logs.py` | Decoder for the lsm2 native logs (record framing per the lsm2 behaviour specification). |

`rig_host` options used by the edge cases:
- the plant's `"schedule"`: `[{"t": s, "i_gain"|"v_gain": x or [x per speaker], "fixed_temp": T or null}, ...]` changes a sense field from that time on (dead, scaled or implausible V/I; a fixed sensed coil temperature; null returns to the dynamic plant);
- `--event FRAME:reset|FRAME:sleep=S|FRAME:tsjump=N` calls AudioUnitReset, idles the host for S wall-clock seconds, or offsets the render timestamps by N frames, before the first slice at or after FRAME.

## Use

```sh
make                                   # builds rig_host, prefshim.dylib, au_discover
python3 ../signals/gen_signals.py "$SGV2_RIG_DIR/signals" [program.wav]
SGV2_RIG_DIR=... SGV2_RIG_BIN=$PWD python3 capture.py "$SGV2_RIG_DIR/traces" [case-substring ...]
python3 ../run.py "$SGV2_RIG_DIR/traces" "$SGV2_RIG_DIR/signals" OUT   # the Linux side, then compare
```

`rig_host` exits nonzero when a `--cfprop` is refused or does not parse, when a render call fails, or when fewer frames were rendered than read; `capture.py` also checks `render_error` and `frames` in the meta. Both refuse to run as root. `capture.py --self-test` checks the case plumbing without a Mac. `capture.py` needs numpy. `capture_edge.py OUT [substring ...]` runs the edge list the same way. A case whose unit log has an `*_OVERWRITE.txt` file is rerun at a lower pace, because records were lost.

### LSM2 telemetry input

For an LSM2 reference with a telemetry input, add `--lsm2-tele` and explicitly
set `--tele-ch` to the channel count under investigation. The native unit must
accept that format and initialize successfully. The host configures two input
buses and verifies the count and the second bus's format. LSM2 receives planar float32 telemetry at the program sample rate; the `--tele` file and the recorded `.tele.f32`
remain interleaved. Existing runs without this option keep their original
bus configuration and format.

For example, append `--lsm2-tele --tele-ch 4 --tele feedback.f32` to a
four-channel LSM2 invocation with its matching strip and property files.
The option cannot be combined with `--no-tele`, another protection unit, or
`--plant`: the plant's V/I pair layout is not established for LSM2.

This mode requires one complete telemetry pull for every render call, with
the matching frame count and timestamp. A missing, repeated or malformed
pull, a short telemetry file, or a failed lifecycle/configuration operation
fails the run. Metadata records `tele_layout`, `tele_channels_supplied`,
`tele_callbacks`, `tele_frames_delivered` and `tele_rejected_callbacks` so
input delivery can be checked independently. Bus and format readbacks also run after lifecycle
events.

On a macOS 27.2 (26B123) host with J813 tuning, four-channel and eight-channel
telemetry formats both initialized. Distinct test signals appeared in the
native signal log as one output and one telemetry signal per audio channel;
only the first four supplied telemetry channels appeared in that log. This
observation does not establish physical units or a contract for other builds.
The current `lsm2logs.py` decoder does not support that signal-log layout.

Callback delivery alone does not establish that every supplied channel was
used internally or that the native estimator accepted the samples. Verify an estimator response before using a capture as a system-identification or closed-loop reference.
This option does not change the engine or board qualification gates.

## Observed host facts

- spp3 and lsm2 instantiate and render offline from their factories; no device is needed.
- Open-loop spp3 results are bit-identical between macOS 26.6 and 27.0, and between an M2 and an M4 Pro host.
- The spp3 telemetry bus is 12 interleaved channels: voltage then current, per part.
- Without the unit's calibration object, spp3 ignores telemetry entirely.
- lsm2 has no telemetry bus on the MacBook Neo tuning.
- spp3 debug logging is a bit mask: 1 V/I, 2 coil temperature, 4 predicted displacement, 8 closed-loop displacement/error/resistance, 16 coefficients, 32 more displacement, 64 and 128 capacitance, 256 power management.
