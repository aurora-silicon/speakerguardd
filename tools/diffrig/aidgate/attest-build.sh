#!/bin/bash
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
# attest-build.sh BUILD BOARD IPSW_URL: add macOS BUILD's block to packaging/session/parity.json without booting a Mac
# on it, when its units load on the reference host (README, "Builds without a Mac"). Steps:
#  1. FETCHER (a Linux Mac with liblzfse + python3-cryptography, e.g. ssh m2): ipsw_units.py reads AudioDSP + Tunings
#     from the restore image by HTTP ranges into a private dir; their hashes go to units.json (hashes only).
#  2. carry-over: if AudioDSP's sha and every tuning key equal an attested build's, the block is that build's entries
#     under this build's name (no render needed); the reference names both builds.
#  3. otherwise, HOST (the newest-macOS reference host, under a host-heavy slot): an ad-hoc signed private copy of
#     AudioDSP is loaded by box_host (AUDIODSP_PATH) and attest_refs.sh renders every variant. A binary newer than
#     the host's macOS does not load: the run stops and says that this build needs a host on it.
#  4. GATE (buildbox): attest.py gate --build BUILD merges the block (same law keeps the others).
# Env: FETCHER (ssh host, default m2; FETCH_SSH_OPTS for extra ssh options), BB (buildbox work dir), BIN (voicing-render + aurora-speaker-tuning + REV),
#      BOXHOST (box_host built from tools/diffrig/macos), PRIV (private tainted dir), SLOT (host-heavy slot dir),
#      PROG (gate program base path on the buildbox, .f32 + .json), PROG_LOCAL (a local copy of the .f32 for the host),
#      GATE_PY (buildbox python with numpy, default python3), SKIP_FETCH=1 (reuse PRIV/BUILD from an earlier run).
set -euo pipefail
BUILD=${1:?build}; BOARD=${2:?board, e.g. j316s}; URL=${3:?restore image URL}
G=$(cd "$(dirname "$0")" && pwd); R=$(cd "$G/../../.." && pwd)
FETCHER=${FETCHER:-m2}; read -r -a FOPT <<< "${FETCH_SSH_OPTS:-}"; BB=${BB:?buildbox work dir}; BIN=${BIN:?bins dir on the buildbox}; BOXHOST=${BOXHOST:?box_host}
PRIV=${PRIV:?private dir}; SLOT=${SLOT:?host-heavy slot dir}; PROG=${PROG:?gate program base on the buildbox}
D=$PRIV/$BUILD; mkdir -p "$D"; chmod 700 "$PRIV" "$D"
log(){ echo "$(date +%H:%M:%S) attest-build $BUILD: $*"; }
GATE_PY=${GATE_PY:-python3}
# 1. fetch
if [ "${SKIP_FETCH:-0}" != 1 ]; then
ssh "${FOPT[@]}" "$FETCHER" "rm -rf /var/tmp/attest-build && mkdir -p /var/tmp/attest-build"
tar -C "$R" -cf - tools/diffrig/aidgate/ipsw_units.py packaging/upgrade/aurora_speaker_upgrade | ssh "${FOPT[@]}" "$FETCHER" "tar -C /var/tmp/attest-build -xf -"
ssh "${FOPT[@]}" "$FETCHER" "cd /var/tmp/attest-build && python3 -I tools/diffrig/aidgate/ipsw_units.py $BUILD $BOARD out --ipsw '$URL'" > "$D/units.json"
ssh "${FOPT[@]}" "$FETCHER" "tar -C /var/tmp/attest-build/out -cf - ." | tar -C "$D" -xf -; ssh "${FOPT[@]}" "$FETCHER" "rm -rf /var/tmp/attest-build"
fi
log "units $(python3 -c "import json;print(json.load(open('$D/units.json'))['AudioDSP']['sha256'][:16])")"
# variants (keys under this build) on the buildbox
ssh buildbox "rm -rf $BB/$BUILD && mkdir -p $BB/$BUILD"; tar -C "$D" -cf - Tunings | ssh buildbox "tar -C $BB/$BUILD -xf -"
ssh buildbox "cd $BB/wt && python3 -I tools/diffrig/aidgate/attest.py variants --bin $BIN --build $BUILD --tunings $BB/$BUILD/Tunings --out $BB/$BUILD/variants.json" >/dev/null
scp -q "buildbox:$BB/$BUILD/variants.json" "$D/variants.json"
# 2. carry-over: same units and same tuning keys as an attested release
python3 - "$R/tools/diffrig/aidgate/units.json" "$D/units.json" "$D/variants.json" "$R/packaging/session/parity.json" <<'PY' && exit 0 || true
import json, sys
table = json.load(open(sys.argv[1])) if __import__("os").path.exists(sys.argv[1]) else {}
u = json.load(open(sys.argv[2]))["AudioDSP"]["sha256"]; v = json.load(open(sys.argv[3]))
keys = {x["key"] for x in v["variants"] if x.get("key")}
doc = json.load(open(sys.argv[4]))
for r in doc["releases"]:
    if table.get(r["build"]) == u and keys <= {e["key"] for e in r["entries"]}:
        print(f"carry-over: units and tunings equal {r['build']}: re-gate to add the block under {v['build']}")
        sys.exit(0)   # the gate below still runs with the attested build's refs if the operator chooses
sys.exit(1)
PY
# 3. render on the host with this build's units
cp "$D/AudioDSP" "$D/AudioDSP-signed"; codesign -f -s - "$D/AudioDSP-signed" 2>/dev/null
probe=$(AUDIODSP_PATH="$D/AudioDSP-signed" "$BOXHOST" --strip /dev/null --stage x --subtype tmst \
         --factory AUToneMeisterFactory --channels 1 --meta /dev/null --inventory 2>&1 || true)
case "$probe" in
  *"newer than running OS"*|*"Symbol not found"*|*"AUDIODSP_PATH: dlopen"*)
    log "units do not load on this host: $BUILD needs a reference host on $BUILD"; exit 3;;
esac
mkdir -p "$D/refs" "$D/scr"
AUDIODSP_PATH="$D/AudioDSP-signed" BOXHOST="$BOXHOST" TUNINGS_NEW="$D/Tunings" TUNINGS_HOST="$D/Tunings" SCRATCH="$D/scr" \
  DEST="$D/refs" PROG="${PROG_LOCAL:?local copy of the gate program .f32}" JOBS=${JOBS:-2} bash "$G/attest_refs.sh" "$SLOT" "$D/variants.json"
ssh buildbox "rm -rf $BB/$BUILD/refs && mkdir -p $BB/$BUILD/refs"; tar -C "$D/refs" -cf - . | ssh buildbox "tar -C $BB/$BUILD/refs -xf -"
# 4. gate and merge
scp -q "$R/packaging/session/parity.json" "buildbox:$BB/$BUILD/parity.json"
ssh buildbox "cd $BB/wt && $GATE_PY -I tools/diffrig/aidgate/attest.py gate --variants $BB/$BUILD/variants.json --mac $BB/$BUILD/refs --bin $BIN --prog $PROG --reference 'macOS $BUILD Audio Units (its AudioDSP.component, read from the $BUILD restore image, rendered on a $(sw_vers -buildVersion) host)' --out $BB/$BUILD/parity.json --details $BB/$BUILD/details" | tail -1
scp -q "buildbox:$BB/$BUILD/parity.json" "$R/packaging/session/parity.json"
python3 - "$R/tools/diffrig/aidgate/units.json" "$BUILD" "$D/units.json" <<'PY'
import json, os, sys
p = sys.argv[1]; t = json.load(open(p)) if os.path.exists(p) else {}
t[sys.argv[2]] = json.load(open(sys.argv[3]))["AudioDSP"]["sha256"]
json.dump(dict(sorted(t.items())), open(p, "w"), indent=1); open(p, "a").write("\n")
PY
log "block merged into packaging/session/parity.json; commit it with units.json"
