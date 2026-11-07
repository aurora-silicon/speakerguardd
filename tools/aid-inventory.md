# AID inventory tool

Run `aid-inventory.py` in a dedicated checkout on an approved build host. It
accepts a tuning root as its positional argument and reads the existing data
in place. It builds both feature variants, compiles both ordinary and
`--voicing-pre` blobs, runs `voicing-render --info`, and repeats the entire
sweep. The two structural results must match before it writes the reports.

```sh
python3 tools/aid-inventory.py /path/to/private/Tunings \
  --private-dir /path/to/private/new-inventory-run \
  --out-dir /path/to/public-report \
  --source-revision SOURCE_COMMIT \
  --expected-aids 40 \
  --newer-root 27.2-BUILD=/path/to/private/newer/Tunings
```

Optional `--bundle label=/path/to/private/bundle` arguments add rig-specific
selections without changing the exclusive host-AID counts. The known board
mapping is used where available; unknown AIDs use the existing provisioning
sweep's development-board convention. Optional bundles use their own private
board/unit binding, which is never exported. The host-root sweep uses
development provenance and does not certify production provenance or per-unit
calibration binding.

Additional `--newer-root` inputs are swept independently with both builds and
compile modes. The release comparison keeps a 25G83 parity-reference column,
marks AIDs absent from it, and retains separate counts for each newer root.
The reference root is never replaced by the newer data. Unknown `lsid` and
`dbcontrol` graph boxes remain explicit `unimplemented` rows. Capability text
is labelled separately from actual observed compiler refusal reasons; an
off-path unsupported box is not falsely reported as an emitted refusal.

The tool exports only explicitly selected structural metadata. Raw diagnostics,
graph defaults, blobs, rendered samples and instrumentation sources stay under
the required private directory. Never commit that directory. The optional
bundle label must be a public-safe label supplied by the operator.

Temporary exact-match overlays add counters to the copied build sources and
install the probe example only while building. The original sources are
restored in a `finally` block. The copies retained under the private directory
show exactly what was instrumented. An unfamiliar source layout refuses
instrumentation instead of guessing an anchor. No production DSP calculations
change, and the repository's core/plugin sources are untouched after building.

`configured_bounds` means the compiled box contains a bound;
`exercised_bounds` means its counter fired during the synthetic full-chain
program. The program has quiet, loud and release segments, runs offline, and
has no speaker output. Counter hits cover nontrivial CFreak slow release and
true-peak scaling, the
type-15 runtime, type-18 conservative dispatcher and enabled conservative LowFlow lanes.
Input-amplitude clamps are tagged only when an actual sample exceeds the
existing public-code bound. Other configured bounds are retained as open items
without fabricated hit counts.

An accepted compile or an empty chain gap list is insufficient evidence of
macOS exactness. The report reserves `exact` for bypass and simple structural
operations. Active DSP without an independent qualification certificate has
an explicit parity-unverified gap. The JSON distinguishes saved, enabled and
running tmst type counts, so a disabled type-15/18 slot does not imply that its
law is on the render path. The report never treats a quieter protective bound
as exact parity.

The JSON includes each AID, every selected chain box, both compile modes for
both builds, per-box discrete compressor controls, tmst type counts, emitted
TODO(SPEC) counts and a per-law AID rollup. It also states evidence limits:
this inventory uses the default selection, one sample rate and one short
program; source-only TODOs and histories not exercised are not called covered.

Run the synthetic privacy/classification checks on the build host with:

```sh
python3 tools/test_aid_inventory.py
```

When only a subset of reference inputs is available, the report assembler
keeps unexecuted AIDs explicit and never counts source-loading failures as
AID refusals:

```sh
python3 tools/aid-inventory-merge.py --catalogue STRUCTURAL_CATALOGUE.json \
  --measured FIRST_REPORT.json --measured SECOND_REPORT.json --out-dir REPORT_DIR
```

Its inputs are the public-safe inventory JSON reports and a structural
catalogue with release membership, reference/newer declarations and optional
saved-control metadata. It rejects conflicting measured rows and preserves
the distinction between actual execution and declarations alone.
