# Conventions for the v2 branch

## Branches and merging

- `v2` is the integration branch. Work lands through pull requests into `v2`, as merge commits. Never force-push `v2` or anyone else's branch.
- Name branches `v2/<lane>/<topic>`, for example `v2/tuning/plist`, `v2/engine/neon-thermal`, `v2/rig/aurender` or `v2/kernel/lease`.
- Every PR gets a review. A PR that changes behaviour states its parity numbers (rig cases and worst error per field) in its body.
- PR bodies end with the generator attribution line the project uses.

## Ownership

| Path | Lane |
|---|---|
| `crates/aurora-speaker-tuning` | A: tuning reader |
| `crates/sgv2-core`, `crates/sgv2-lv2`, `packaging/pipewire`, `packaging/wireplumber` | B: in-graph engine |
| `crates/sgv2-render`, `tools/diffrig` | C: differential rig |
| `crates/sgv2-supervisor/src/kernel.rs`, `packaging/systemd`, `packaging/selinux` | D: kernel and amp interface |
| `crates/sgv2-supervisor` (rest), `docs`, CI, workspace | shared; changes go through review |

The interfaces between lanes are:
- `sgv2_core::ProtectionUnit`, `Io`, `Health` and `Telemetry`;
- the parameter blob (`sgv2_core::blob`);
- the trace schema (`tools/diffrig/schema.json`);
- `sgv2_supervisor::kernel::Kernel`.

Change an interface in its own PR, and tell the other lanes.

## Behaviour and parameters

- Behaviour comes only from the behaviour specification. Mark every gap `TODO(SPEC)` with what is missing. Do not fill it with a guess, a margin or a heuristic.
- Parameters come only from the Mac's own tuning bundle, at runtime. No parameter values from any Mac are written into code, tests or docs.
- Mark work owned by another lane `TODO(lane X)`.

## Clean publish

- No Apple files or bytes in the repository: no tuning folders, strips, graphs, property-list presets, calibration records, or traces made with Apple tuning. `tools/check-clean.sh` enforces this in CI. Tests use synthetic data.
- No copied or decompiled code. Write every implementation from the specification.
- Commit messages and docs describe what the code does. They carry no lab notes, private paths, hosts or task identifiers.

## Code

- Rust, edition 2021, `cargo fmt` (see `rustfmt.toml`) and `cargo clippy -D warnings` clean.
- `sgv2-core` is `no_std`. `process` must not allocate (`tests/rt_alloc.rs`), lock, block, log or panic.
- NEON kernels live beside a scalar reference, and tests require them to agree as the specification allows.
- `unsafe` blocks carry a `SAFETY:` comment.

## Attribution

Code derived from aurora-audio-dsp by saku (MIT) keeps the header `Based on aurora-audio-dsp by saku (MIT)` with saku's copyright line and the license notice (`crates/aurora-speaker-tuning/LICENSE-aurora-audio-dsp`). Its commits carry `Co-authored-by: saku <69316447+sakutty@users.noreply.github.com>`.
