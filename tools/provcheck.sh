#!/bin/bash
# tools/provcheck.sh: offline provisioning check, no hardware.
#
# usage: tools/provcheck.sh WT OUTDIR BUNDLE...
#
# For the speakerguardd checkout WT, compile each installer bundle the way
# speakerguardd-provision does on its Mac: board and unit from the bundle's
# own manifest, binding negatives (another unit, another board must refuse),
# the lsm2 masker lattice, and the graph (wiring.py with the board fixture's
# device tree and the bundle's real tuning trace). Then every board fixture's
# graph, and on macOS every AID in the host's Tunings (development mode;
# PROVCHECK_TUNINGS overrides the directory). OUTDIR holds Apple data: keep
# it out of git. One row per bundle on stdout.
set -u
WT=$1; OUT=$2; shift 2
mkdir -p "$OUT"
TGT=${CARGO_TARGET_DIR:-$WT/target}
( cd "$WT" && CARGO_TARGET_DIR=$TGT cargo build -q --release -p aurora-speaker-tuning --bin aurora-speaker-tuning ) >"$OUT/build.txt" 2>&1 \
  || { echo "BUILD FAIL (see $OUT/build.txt)"; exit 1; }
CLI=$TGT/release/aurora-speaker-tuning
FX=$WT/packaging/session/fixtures
WIRING=$WT/packaging/session/wiring.py

for B in "$@"; do
  M=$(tar -xzOf "$B" audio-tuning/manifest.json 2>/dev/null)
  BOARD=$(printf %s "$M" | python3 -c 'import json,sys;d=json.load(sys.stdin);print(d.get("board") or "")')
  UNIT=$(printf %s "$M" | python3 -c 'import json,sys;d=json.load(sys.stdin);print(d.get("unit_id") or "")')
  AID=$(tar -xzOf "$B" aurora-speaker-tuning.json | python3 -c 'import json,sys;print(json.load(sys.stdin)["aid"])')
  TAG=${BOARD:-unbound}-$AID
  # Board as provisioning sees it: the bundle's, else the collection's compatible (unbound legacy bundle).
  PB=${BOARD:-$(tar -xzOf "$B" aurora-speaker-tuning.json | python3 -c 'import json,sys;print(json.load(sys.stdin)["compatible"].split(",")[1])')}
  ARGS=(--board "$PB"); [ -n "$UNIT" ] && ARGS+=(--unit-id "$UNIT")
  "$CLI" compile "${ARGS[@]}" --bundle "$B" -o "$OUT/$TAG.blob" >"$OUT/$TAG.compile.txt" 2>&1; RC=$?
  NW=$(/usr/bin/grep -ci 'warning' "$OUT/$TAG.compile.txt"); NW=$(/usr/bin/grep -oE '[0-9]+ warning' "$OUT/$TAG.compile.txt" | tail -1 | cut -d' ' -f1); NW=${NW:-0}
  if [ $RC -eq 0 ]; then C=OK; else C="FAIL($(/usr/bin/grep -m1 -iE 'error|refus' "$OUT/$TAG.compile.txt" | cut -c1-110))"; fi
  # Binding: own unit -> "unit calibration in use"; another unit must refuse; another board must refuse.
  BIND=-
  if [ -n "$UNIT" ]; then
    OTHER=$(printf 'aurora-prov-other' | shasum -a 256 | cut -d' ' -f1)
    "$CLI" compile --board "$PB" --unit-id "$OTHER" --bundle "$B" -o "$OUT/neg.blob" >"$OUT/$TAG.neg-unit.txt" 2>&1; NU=$?
    "$CLI" compile --board j000 --unit-id "$UNIT" --bundle "$B" -o "$OUT/neg.blob" >"$OUT/$TAG.neg-board.txt" 2>&1; NB=$?
    rm -f "$OUT/neg.blob"
    CAL=$(/usr/bin/grep -c 'unit calibration in use' "$OUT/$TAG.compile.txt")
    BIND="own:$([ $RC -eq 0 ] && { [ "$CAL" -gt 0 ] && echo cal || echo nocal; } || echo -) other-unit:$([ $NU -ne 0 ] && echo refused || echo ACCEPTED) other-board:$([ $NB -ne 0 ] && echo refused || echo ACCEPTED)"
  else
    BIND="unbound(open loop)"
  fi
  # Lattice: in the bundle (manifest record), and what an lsm2 blob carries.
  LAT="bundle:$(printf %s "$M" | python3 -c 'import json,sys;d=json.load(sys.stdin).get("lsm2_mask_lattice");print(d["entries"] if d else "none")')"
  /usr/bin/grep -q 'mask-lattice-absent' "$OUT/$TAG.compile.txt" && LAT="$LAT,ABSENT-warn"
  if [ $RC -eq 0 ]; then
    "$CLI" blob "$OUT/$TAG.blob" >"$OUT/$TAG.blob.txt" 2>&1
    U=$(sed -n 's/.*unit: \([0-9]*\).*/\1/p' "$OUT/$TAG.blob.txt" | head -1)
    KIND=$(python3 -c 'import sys;print(int(sys.argv[1]).to_bytes(4,"big").decode("latin1"))' "${U:-0}" 2>/dev/null)
    LAT="$LAT,unit:$KIND"
    if /usr/bin/grep -qi 'mask_lattice' "$OUT/$TAG.blob.txt"; then
      LAT="$LAT,blob:$(python3 -c 'import re,sys;t=open(sys.argv[1]).read();m=re.search(r"mask_lattice[^\[]*\[([^\]]*)\]",t,re.S);print(len([x for x in m.group(1).split(",") if x.strip()]) if m else "?")' "$OUT/$TAG.blob.txt")"
    fi
  fi
  # Graph: the board fixture's device tree + this bundle's real tuning trace (wiring.py derive).
  G=-
  FB=$(echo "$PB" | sed 's/s$//')
  if [ -f "$FX/$FB.json" ]; then
    python3 -c 'import json,sys;json.dump(json.load(open(sys.argv[1]))["hw"],open(sys.argv[2],"w"))' "$FX/$FB.json" "$OUT/$TAG.hw.json"
    "$CLI" dump --json --bundle "$B" >"$OUT/$TAG.dump.json" 2>"$OUT/$TAG.dump.err" || "$CLI" dump --json "$B" >"$OUT/$TAG.dump.json" 2>"$OUT/$TAG.dump.err"
    python3 -I "$WIRING" describe --hw "$OUT/$TAG.hw.json" --tuning-json "$OUT/$TAG.dump.json" >"$OUT/$TAG.wiring.txt" 2>&1 && G=OK || G="FAIL($(tail -1 "$OUT/$TAG.wiring.txt" | cut -c1-90))"
  else
    G="no fixture $FB"
  fi
  printf '%s\t%s\tcompile=%s\twarn=%s\tbind=%s\tlattice=%s\tgraph=%s\n' "$PB" "$AID" "$C" "$NW" "$BIND" "$LAT" "$G"
done

# Board fixtures: every fixture's graph (gen.py's derivation) + the shipped-file check.
FXR=$(cd "$WT/packaging/session" && python3 -I -c '
import sys; sys.path.insert(0,".")
import gen
bad=[]
for m in gen.machines():
    try: gen.machine(m)
    except SystemExit as e: bad.append(str(e))
print("fixtures=%d bad=%d %s" % (len(gen.machines()), len(bad), "; ".join(bad)))' 2>&1 | tail -1)
printf 'fixtures\t-\t%s\n' "$FXR"

# Host sweep: every AID in this host's macOS Tunings, compiled in development
# mode (the reader + compiler on every speaker graph Apple ships here).
TR=${PROVCHECK_TUNINGS:-/System/Library/Audio/Tunings}; [ -d "$TR" ] || exit 0; ok=0; bad=(); n=0
for d in "$TR"/AID*; do
  a=${d##*/AID}; n=$((n+1))
  if "$CLI" compile --development --board j473 --tunings-root "$TR" --aid "$a" -o "$OUT/host.blob" >"$OUT/host-AID$a.txt" 2>&1; then ok=$((ok+1))
  else bad+=("AID$a:$(/usr/bin/grep -m1 '^error' "$OUT/host-AID$a.txt" | cut -c1-90)"); fi
done
rm -f "$OUT/host.blob"
printf 'host-AIDs\t-\tok=%d/%d\n' "$ok" "$n"
for b in "${bad[@]}"; do printf '  %s\n' "$b"; done
