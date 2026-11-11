#!/bin/bash
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
# macOS side of attest.py (host only, under a host-heavy slot): for every tuning in
# VARIANTS (attest.py variants), gr.py renders the gate program through the installed
# Audio Units with that tuning's strip and property strip, keeping only the unit input.
# Outputs are Apple-derived: private dirs only.
#   usage: attest_refs.sh SLOTDIR VARIANTS.json
# env: BOXHOST (box_host with --propstrip), TUNINGS_NEW / TUNINGS_HOST (the trees, as in
# host_refs.sh: an AID the newer tree has renders from it), SCRATCH (local work),
# DEST (rsync target, default buildbox:/srv/aurora/fast/o/sgchk/tainted/mac-attest),
# PROG (the gate program, gate_prog.py), JOBS (parallel renders, default 2),
# SLIDER (volume slider position, default 1: full volume; any other value renders at the
# variant's (mvsc, mvol) pair for that slider from attest.py variants, into its own DEST).
set -u
SLOT=${1:?slot dir}; VARIANTS=${2:?variants.json}
[ -f "$SLOT/owner" ] || { echo "need a host-heavy slot"; exit 2; }
G=$(cd "$(dirname "$0")" && pwd)
BOXHOST=${BOXHOST:?box_host built from tools/diffrig/macos/box_host.m}
T27=${TUNINGS_NEW:-/Volumes/AuroraS500/reference/tunings/macos-27.2-26B8125}
T25=${TUNINGS_HOST:-/System/Library/Audio/Tunings}
SCR=${SCRATCH:?scratch}
DEST=${DEST:-buildbox:/srv/aurora/fast/o/sgchk/tainted/mac-attest}
PROG=${PROG:?gate program}
SLIDER=${SLIDER:-1}
export G BOXHOST T27 T25 SCR DEST PROG
python3 -c '
import json, sys
sl = float(sys.argv[2])
for v in json.load(open(sys.argv[1]))["variants"]:
    if v.get("key"):
        n = "AID%d" % v["aid"] + ("" if v["speaker_id"] is None else "-spid%d" % v["speaker_id"])
        if sl == 1.0:
            mv = (1.0, 0.0)
        else:
            pts = [p for p in v.get("volume") or [] if abs(p[0] - sl) < 1e-9]
            if not pts:
                print("no volume law point at slider", sl, "for", n, file=sys.stderr)
                continue
            mv = tuple(pts[0])
        print(n, v["aid"], v["strip"], v.get("propstrip") or "-", repr(mv[0]), repr(mv[1]))
' "$VARIANTS" "$SLIDER" | xargs -P "${JOBS:-2}" -L 1 bash -c '
  name=$0 aid=$1 strip=$2 prop=$3 mvsc=$4 mvol=$5
  free=$(df -g / | awk "END{print \$4}"); [ "$free" -lt 20 ] && { echo "disk guard"; exit 3; }
  tree=$T27; [ -d "$T27/AID$aid" ] || tree=$T25
  w=$SCR/$name; rm -rf "$w"; mkdir -p "$w/boxes"
  pa=(); [ "$prop" != "-" ] && pa=(--propstrip "$prop")
  t0=$(date +%s)
  if nice -n 15 python3 $G/gr.py --tunings $tree --aid $aid --strip "$strip" "${pa[@]}" --in $PROG \
        --param mvsc=$mvsc --param mvol=$mvol --out $w/unit_in.f32 --meta $w/meta.json --keep $w/boxes --box-host $BOXHOST > $w/gr.log 2>&1; then rc=0; else rc=$?; fi
  echo "{\"aid\":$aid,\"rc\":$rc,\"tree\":\"$(basename $tree)\",\"strip\":\"$strip\",\"propstrip\":\"$prop\",\"seconds\":$(( $(date +%s)-t0 ))}" > $w/status.json
  rm -rf $w/boxes
  timeout 600 rsync -a -e "ssh -o ConnectTimeout=10 -o BatchMode=yes" "$w/" "$DEST/$name/" && rm -rf "$w"
  echo "$(date +%H:%M:%S) $name rc=$rc"
'
echo DONE
