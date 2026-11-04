# Differential rig

Ownership: lane C. The rig is the acceptance test for the v2 engine: the Mac's own protection unit and `sgv2-core` get identical, deterministic input, and their per-slice internal state and audio must agree within the tolerances of the behaviour specification.

```
signals/gen_signals.py ──► program signals (seeded; regenerate, never commit the audio)
        │
        ├──► macos/  (on a Mac, offline) ──► reference traces + audio + recorded telemetry   [private: Apple-derived]
        │
        └──► sgv2-render (crates/sgv2-render) with the same signals and the recorded telemetry ──► sgv2 traces + audio
                                     │
                              compare.py / run.py ──► per-case pass/fail, worst error per field, plots
```

- `schema.json`: one JSON line per host slice, from both sides.
- `compare.py`: matches slices that end on the same input frame and checks every field the reference carries, the mode flags and the audio. A reference field must have a tolerance and be present on the sgv2 side, or be in the explicit not-compared tables (`NOT_COMPARED`, `GLOBAL_NOT_COMPARED`), which the report lists with their reasons. The report gives per-field compare and missing counts; a case with no reference slices or no compared values fails, and any non-finite audio sample fails. `--self-test` runs in CI.
- `run.py`: runs `sgv2-render` for every macOS case directory, with the same run-time and case properties (`--cfprop`, fixtures from `--cal`, checked against the SHA-256 in `run.meta.json`), and compares; writes `summary.md`. `--self-test` runs in CI.
- `signals/`: generator and `manifest.json`, with the SHA-256 of each generated file.

Tolerances: 0.01 dB gain, 0.02 °C temperature, 1e-3 relative power, 1e-5 linear audio below limiting. Mode transitions must land in the same slice.

Golden vectors made with a Mac's tuning are Apple-derived data and stay outside this repository. Only synthetic-parameter vectors may be committed here.
