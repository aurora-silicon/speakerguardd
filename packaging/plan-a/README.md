# Single-board Plan A test packages

The shipped `packaging/session/gate.json` stays all off. After the board's
owner approves its numerical and acoustic gates, a test package may select
exactly one physical board:

```
python3 packaging/plan-a/gate.py j316s /build/gate.json
```

Install that output as `/usr/share/speakerguardd/gate.json` in the package;
all other policy keys remain unchanged. Do not edit the source default.
Choices are j616s, j316s and j413. Give each package a separate release.
This flag selects deployment only; it does not assert acoustic qualification.
Rebuild from the owner's passing source SHA before deployment if DSP laws
changed since the test snapshot.

For j413 pass `--features sgv2-core/lowflow-conservative` to **both** the
compiler and LV2 build (and the blob-info example). The artifact encodes the
build flavor and the other flavor refuses it. A single workspace cargo build
of all shipped crates with this feature keeps the two components consistent.
Do not distribute tuning bundles or compiled per-machine blobs. Provisioning
compiles from the target's own bundle and binds it to that unit.

Package `sgv2-lv2`'s `blob-info` example for verification. Its
`--summary BLOB BOARD` mode checks the artifact and both plugin loaders,
prints no tuning parameters, and returns failure on mismatch or absence.
The legacy detailed output is for local diagnostics only.

Upgrade and rollback require a controlled reboot so the compiler, plugin,
checked blob and generated graph switch together before playback. Use the
existing kernel bootguard one-shot fallback; keep the known kernel default
until acceptance. Never bypass the protection grant or run two supervisors.
