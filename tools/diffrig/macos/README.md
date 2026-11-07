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
| `prestage_rig.py` | Pre-stage rig: the compiled blob's pre-stage on the graph's own lane routing vs the macOS graph with only its filter and compressor boxes running (every other Audio Unit box a wire, both sides), sines and pink per unit lane. |
| `box_host.m` | Offline host for one box of a speaker graph, any `aufx` subtype (`--subtype`, `--factory`): the strip preset (ClassInfo), `--set ID=VALUE` before initialisation, whole signal in host slices; writes the audio and the unit's parameter inventory, latency and tail. |
| `graph_render.py` | Whole-graph macOS reference: reads an AID's `.dspg` and strip at run time, walks the wires, renders every Audio Unit box through `box_host` and the built-in boxes (`nfnp`, `split`/`chsp`, `join`/`chjn`, `sum`/`summ`, `dbgn`) itself, at full system volume unless `--param` says otherwise. Stops at the protection unit by default (the output is the unit's input per unit channel); `--bypass BOX` treats a box as a wire. Needs numpy. Outputs are Apple-derived: keep them out of the repository. |
| `au_discover.m` | Lists a unit's element counts, custom properties (with CF values) and read-only meters. |
| `capture.py` | The case list. It runs every case through `rig_host` and writes `trace.jsonl` in `../schema.json`, the per-quantum meter CSV, the audio and the unit's own logs. |
| `capture_edge.py` | Edge-case list: atsp on each AID's own preset (no meters; the trace carries the audio gain and per-slice sample-gain extremes), spp3 closed-loop sense faults and gates (`spp3-cl-edge-*`), and fractional-Rampout AIDs (`spp3-rampout-*`). Its spp3 closed-loop cases decode the unit's coil-temperature and controller logs into `clsp.npz` and require every record (they rerun at a lower pace otherwise). Outputs are gzipped; `compare.py` and `run.py` read the `.gz` forms. |
| `test_rig_host.py` | Exit-status tests for `rig_host` and `prefshim` after `make`: argument bounds, refused or unreadable `--cfprop`, render errors (`--test-fail-at`), short renders and whole-component redirects. It renders a short synthetic signal offline into a temporary directory and deletes it. |
| `lsm2logs.py` | Decoder for the lsm2 native logs, with explicit signal layout, strict framing checks and optional JSON/NPZ artifacts. |
| `test_lsm2logs.py` | Command-level replay of synthetic log files, including malformed inputs; retains inputs, commands, results and decoded artifacts. |

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
Use the explicit `output-telemetry` signal-log layout to decode those captures.

Callback delivery alone does not establish that every supplied channel was
used internally or that the native estimator accepted the samples. Verify an estimator response before using a capture as a system-identification or closed-loop reference.
This option does not change the engine or board qualification gates.

### Decoding native signal logs

For the observed output-plus-telemetry format, each `LsmSignalLog` record is
channel-major: output channel 0's `n` samples, telemetry channel 0's `n` samples,
then the same pair for each remaining channel. Its payload is `2 * N * n`
float32 words. Select it explicitly:

```sh
python3 -I tools/diffrig/macos/lsm2logs.py PRIVATE_LOG_DIR \
  --signal-layout output-telemetry --strict \
  --summary PRIVATE_SUMMARY.json --signals PRIVATE_SIGNALS.npz
```

The NPZ contains `output` and `telemetry` arrays shaped `(frames, channels)` and
`record_start` frame offsets. JSON records input hashes, channel/record/frame
counts, nonfinite payload counts, decoder hash and NumPy version. These files
contain native data; keep them private whenever the input uses Apple tuning.

The default `output` layout preserves the original `N * n` payload format.
Neither mode guesses the layout: plausible sample values can make an incorrect
selection appear well-framed. Establish the format with independently tagged
inputs before drawing conclusions. Logged telemetry proves which samples were
logged, not their physical units or acceptance by an estimator.

`--strict` rejects partial float words, truncated records, empty/header-only
logs, invalid channel/frame counts and multiple files for one log type. All
recognized logs are checked before artifacts are written. Signal export also
requires a complete `LsmSignalLog`. The default Python API and CLI still allow a
trailing partial record for existing capture workflows. Nonfinite payload
samples are preserved and counted; framing success is not model validation.
Complete frame counts must fit signed 64-bit offsets in either mode; oversized
or cumulatively overflowing counts are rejected before NumPy conversion.
Check the reported overwrite-marker count separately before treating a capture
as complete.

The synthetic command-level replay uses NumPy 2.2.6 in CI:

```sh
python3 -I tools/diffrig/macos/test_lsm2logs.py --out NEW_REPLAY_DIRECTORY
```

Its failure model covers channel/block-order mistakes, explicit-layout misuse,
partial/empty/ambiguous inputs, invalid or overflowing counts, missing signal data, loss of
nonfinite samples and legacy trailing-record compatibility. Every case retains
its exact input, command, stdout/stderr and verdict in `RESULT.json`; CI uploads
that directory. The fixtures are invented and contain no native tuning.

## Observed host facts

- spp3 and lsm2 instantiate and render offline from their factories; no device is needed.
- Open-loop spp3 results are bit-identical between macOS 26.6 and 27.0, and between an M2 and an M4 Pro host.
- The spp3 telemetry bus is 12 interleaved channels: voltage then current, per part.
- Without the unit's calibration object, spp3 ignores telemetry entirely.
- lsm2 has no telemetry bus on the MacBook Neo tuning.
- spp3 debug logging is a bit mask: 1 V/I, 2 coil temperature, 4 predicted displacement, 8 closed-loop displacement/error/resistance, 16 coefficients, 32 more displacement, 64 and 128 capacitance, 256 power management.
