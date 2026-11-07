# Optional native LSM2 amplifier selectors

A saved strip can omit `PowerAmpModel` even though the native AudioUnit exposes
it. The optional `lsm2_native_state` manifest record names
`lsm2-native-state.plist`, with its byte length and SHA-256. It can supply only
missing global `PowerAmpModel` IDs for the graph's active channels. Existing
values and all other parameters remain unchanged. It does not enable any DSP
feature or change the LSM2 qualification gate.

This record checks the evidence's self-consistency. It is not an attestation of
its author or of how the capture was made. The bundle and its collection process
must still come from a trusted source. Native parameter values are private
machine data; no native tuning or calibration belongs in public fixtures.

The binary or XML plist has these fields:

- `schema`: integer `1`.
- `method`: `AudioUnitGetProperty-ClassInfo`.
- `identity_before` and `identity_after`: equal dictionaries containing
  `macos_build`, `board`, and `unit_id`, matching the source manifest. The unit ID
  uses `aurora-speaker-unit-v1\0` plus the serial bytes, as in `machine::unit_id`.
- `source_fingerprint`: SHA-256 of `aurora-lsm2-native-source-v1\0` followed by
  compact JSON arrays `[path, byte_length, sha256]` for every original manifest
  record, sorted by path. This includes source files, shared files, calibration,
  and masker lattice, but excludes the native-state record itself. Paths must
  be canonical ASCII bundle paths; duplicates are refused.
- `runtime_overrides`: an empty array.
- `channels` and `sample_rate`: positive integers describing the captured
  native stream. They must match the graph's active channels and compile rate.
- `canonical_state`: raw bytes returned by native `ClassInfo`, encoded as a
  standalone plist, and `canonical_sha256`: their SHA-256.
- `parameter_queries`: dictionaries with `stage`, `id`,
  `parameter_info_status`, `get_parameter_status`, and `value`. Every active
  amplifier selector is queried in global scope, element zero, and must have
  exactly one successful query at each of
  `before-init`, `after-init`, and `after-render`. The values must match the
  canonical state. Status fields are integer zero; values are finite numbers.
  No queries for inactive channels or other parameters are accepted.

The native component and preset version must match the selected strip. Every
original scope/element/ID must be present with identical float32 bits in the
canonical state. The only eligible additions are global IDs
`1058848 + 1024 * channel`, with integer values 0, 1, or 2. Other additions are
ignored. Compilation of a supplemented configuration requires a known runtime
unit ID matching the manifest, even when no per-unit calibration is used.

An absent record leaves the existing missing-selector refusal intact. A
malformed, changed, foreign, or incomplete declared record is an error; it is
never ignored in favor of a default.

## Failure model and command-level validation

The command-level fixture checks unchanged compilation with no supplement,
missing-selector refusal, successful missing-only projection, and byte-identical
compiled output. It rejects changed original flags/selectors, different native
builds/boards/units, source or canonical hash mismatches, malformed parameters,
failed or missing queries, invalid selectors, runtime overrides, unknown runtime
identity, a declared record on another protection unit, empty identity fields,
and a different channel count/sample rate. A valid snapshot must still leave
unsupported DSP flags refused.

Run `cargo test -p aurora-speaker-tuning --test native_lsm2_e2e -- --nocapture`.
The test retains generated synthetic bundles, exact CLI commands, output,
compiled blobs, and a JSON result under its printed artifact directory. Set
`AURORA_TUNING_E2E_OUT` to a new directory to choose that location.

The current public installer does not produce this optional artifact. A producer
must record the native API results and the matching bundle's provenance before
this loader can use them. A capture from another build or unit is refused; the
remaining SystemID, temperature, compensation, bank, inductance, and power-limit
refusals still apply.
