#!/bin/bash
# tools/fuzz.sh: build and run the cargo-fuzz targets, host only.
#
# usage: tools/fuzz.sh SECONDS WORKDIR [crate/target ...]
#
# Runs every target (or the ones named, e.g. sgv2-supervisor/selfcheck) for
# SECONDS each, one after the other, with its corpus in WORKDIR/corpus/<target>
# and any crash in WORKDIR/artifacts/<target>/. Needs cargo-fuzz; a stable
# toolchain works (RUSTC_BOOTSTRAP=1 lets cargo-fuzz pass -Zsanitizer).
# Extra libFuzzer flags: FUZZ_ARGS (default -rss_limit_mb=1024).
#
# The crates' synthetic seeds: `cargo test --lib -- --ignored` in
# crates/aurora-speaker-tuning/fuzz with SEED_DIR=WORKDIR/corpus. Never seed
# a corpus from a Mac's own tuning files and never commit one that was.
set -euo pipefail
secs=$1 work=$2
shift 2
root=$(git rev-parse --show-toplevel)
export RUSTC_BOOTSTRAP=1
args=${FUZZ_ARGS:--rss_limit_mb=1024}
# crate, the cargo-fuzz build flags, and its targets. The tuning pipeline
# and the parsers have no unsafe code of their own, so they run without a
# sanitizer (several times faster); `observe` checks arithmetic overflow.
plan=(
  "sgv2-core|-O|blob spp3 atsp lsm2 handoff"
  "sgv2-supervisor|-O|selfcheck thermal procfs blobcheck"
  "sgv2-supervisor|-O -a|observe"
  "aurora-speaker-tuning|-O -s none|tuning plist aupreset dspg formats"
)
want() {
  [[ $# -le 2 ]] && return 0
  local t
  for t in "${@:3}"; do [[ $t == "$1/$2" ]] && return 0; done
  return 1
}
mkdir -p "$work"
for row in "${plan[@]}"; do
  IFS='|' read -r crate flags targets <<<"$row"
  dir=$root/crates/$crate
  tdir=$work/target/$crate${flags// /}
  for t in $targets; do
    want "$crate" "$t" "$@" || continue
    # shellcheck disable=SC2086
    (cd "$dir" && cargo fuzz build $flags --target-dir "$tdir" "$t")
    mkdir -p "$work/corpus/$t" "$work/artifacts/$t"
    bin=$(find "$tdir" -path "*/release/$t" -type f | head -n1)
    # shellcheck disable=SC2086
    "$bin" "$work/corpus/$t" -max_total_time="$secs" -detect_leaks=0 -print_final_stats=1 \
      -artifact_prefix="$work/artifacts/$t/" $args 2>&1 | tail -n 8
  done
done
