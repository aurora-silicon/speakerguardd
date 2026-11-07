#!/bin/bash
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
# One-command promotion measurement. Run on buildbox; no hardware audio access.
set -euo pipefail
SELF=$(readlink -f "$0")
BASE=${AID_SWEEP_BASE:-/srv/aurora/fast/o/spp3v}
REPO=${AID_SWEEP_REPO:-$BASE/sg}
TUNINGS=${AID_SWEEP_TUNINGS:-$BASE/tainted/tunings-26B8125}
TOOLS=$(dirname "$SELF")
if [[ ${1:-} != --slot-held ]]; then
    exec /srv/aurora/fast/buildslot bash "$SELF" --slot-held "$@"
fi
shift
[[ $# == 1 ]] || { echo 'usage: aidsweep.sh <git-ref>' >&2; exit 2; }
# --slot-held is for the controller's already-acquired buildslot only.
test ! -e /srv/aurora/fast/STOP
AVAILABLE=$(df -Pk "$BASE" | awk 'END {print $4}')
(( AVAILABLE >= 45 * 1024 * 1024 )) || { echo 'build volume below 45 GiB' >&2; exit 3; }
export RUSTUP_HOME=${RUSTUP_HOME:-/srv/aurora/fast/o/LSX-farm/rustup}
export CARGO_HOME=${CARGO_HOME:-/srv/aurora/fast/o/LSX-farm/cargo}
export PATH="$CARGO_HOME/bin:/usr/local/bin:/usr/bin:/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
umask 077
mkdir -p "$BASE/aid-sweeps" "$BASE/tainted/aid-sweeps"
# Serialize worktree administration/publication while preserving every run.
exec 8>"$BASE/aid-sweeps/lock"
flock 8
git -C "$REPO" fetch origin
REV=$(git -C "$REPO" rev-parse --verify "$1^{commit}")
RUN=$(mktemp -d "$BASE/tainted/aid-sweeps/${REV:0:12}.XXXXXXXX")
TREE=$RUN/source
OUT=$RUN/public
# A dedicated checkout prevents overlays/restoration racing any lane's source.
git -C "$REPO" worktree add --detach "$TREE" "$REV" >"$RUN/worktree.log" 2>&1
mkdir -p "$TREE/tools"
for FILE in aid-inventory.py aid-inventory-probe.rs aid-inventory-sweep.py; do
    cp "$TOOLS/$FILE" "$TREE/tools/$FILE"
done
# Preserve the exact observation tooling used, independent of the measured ref.
sha256sum "$TOOLS/aid-inventory.py" "$TOOLS/aid-inventory-probe.rs" "$TOOLS/aid-inventory-sweep.py" >"$RUN/tooling.sha256"
if ! python3 "$TREE/tools/aid-inventory-sweep.py" "$TUNINGS" \
    --source-revision "$REV" --private-dir "$RUN/private" --out-dir "$OUT" \
    >"$RUN/run.log" 2>&1; then
    echo "sweep failed; private evidence preserved: $RUN" >&2
    exit 1
fi
# Refuse publication if observation overlays were not fully restored.
git -C "$TREE" diff --exit-code -- crates >"$RUN/restoration.diff"
# Separate changes in observation tooling from changes in measured source.
# A corrected scope must never overwrite or compare equal to the old report.
TOOLING=$(cut -d ' ' -f1 "$RUN/tooling.sha256" | sha256sum | cut -c1-12)
PUBLISH=$BASE/aid-sweeps/$REV/$TOOLING
mkdir -p "$PUBLISH"
if [[ -e $PUBLISH/aid-sweep.md ]]; then
    cmp "$PUBLISH/aid-sweep.md" "$OUT/aid-sweep.md"
    cmp "$PUBLISH/aid-sweep.json" "$OUT/aid-sweep.json"
fi
cp "$OUT/aid-sweep.md" "$OUT/aid-sweep.json" "$OUT/verification.json" "$PUBLISH/"
chmod 755 "$BASE/aid-sweeps" "$BASE/aid-sweeps/$REV" "$PUBLISH"
chmod 644 "$PUBLISH/aid-sweep.md" "$PUBLISH/aid-sweep.json" "$PUBLISH/verification.json"
tail -1 "$RUN/run.log"
echo "Reports: $PUBLISH"
echo "Private evidence: $RUN"
