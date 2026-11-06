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

## Case variants and controller dumps (spp3)

- `cargo run --release -p sgv2-render --example blob-patch -- IN OUT key[=value] ...` derives the blob a capture used from a compiled blob. It covers the 64009 SpTS records (`calch=C:R,T`, `nocal`), the shortened SafeTlim test sidecars (`safe_time_min=`), the patched Rampout sidecar (`rampout=`) and property 64007 (`start=`). Like every blob, the outputs carry Mac tuning: keep them out of git.
- `cargo run --release -p sgv2-render --example spp3-dump -- BLOB PROGRAM.f32 CH[,CH..] [SENSE.f32 [RESETS]]` writes the per-update controller state as f64 values. They line up with the reference `clsp.npz` `ctl` records: smoothed and selected gain, Tlim_eff, the FF, coil and magnet nodes, Pcoil, Pdrv, T_CL, the accepted window and the pilot target.
- `run.py` passes each case's `events` to `sgv2-render --events`, which replays `FRAME:reset` as `host_reset` (AudioUnitReset).

## Never-above-macOS gate

`never_above.py` is the safety gate for the whole speaker path, voicing chain included. It does not match slices: for every tuning, required case, volume and driver it checks that the signal v2 sends to the amplifier is never harder on the driver than the signal macOS sends. Per driver it compares band levels (third octaves, 20 Hz-20 kHz, whole-case and loudest short-time frame), sample peak, sliding RMS (10 ms to 10 s), and, through its own linear moving-coil and two-node thermal models filled from the tuning's own fields, peak excursion, peak velocity and peak coil and magnet temperature. v2 must be at or below macOS on every one, within 0.05 dB, 1e-3 relative or 0.02 °C.

It fails closed: a tuning missing a required case or volume, a driver without a complete model, a torn or non-finite render, or renders of different lengths all fail.

- `never_above.py signals OUT` writes the required stereo case set (third-octave sines at 0 and -12 dBFS, anti-phase low tones, bursts, a square, a log sweep, pink noise, a 60 s pink thermal case).
- Each side's renderer runs every case through the full chain at each required volume and writes the per-driver amplifier-input audio; the manifest (format in the script's docstring) pairs them with the per-driver models. Renders, models and manifests made with a Mac's tuning are Apple-derived and stay outside this repository.
- `never_above.py gate MANIFEST --report OUT.json` gives the verdict. Integrators run it on every Mac's tuning before merging a change to the voicing chain, the pre-stage or a protection unit; CI runs `--self-test` on synthetic data.
