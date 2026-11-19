#!/bin/bash
# macOS side of the per-AID full-chain gate (host only, under a host-heavy slot).
# For every AID: gr.py whole-graph render of the program up to the protection
# unit through the installed 25G83 Audio Units (box_host), keeping each AU box's
# output for per-box attribution; copy to the buildbox; delete the local audio.
# Outputs are Apple-derived: private dirs only.  usage: host_refs.sh SLOTDIR AID...
set -u
SLOT=${1:?slot dir}; shift
[ -f "$SLOT/owner" ] && grep -q '^SW-SWEEP' "$SLOT/owner" || { echo "need a host-heavy slot owned by SW-SWEEP"; exit 2; }
G=$(cd "$(dirname "$0")" && pwd)
BOXHOST=${BOXHOST:-/Volumes/AuroraS500/work/speakerguardd/VAT/tools/diffrig/macos/box_host}
T27=${TUNINGS_NEW:-/Volumes/AuroraS500/reference/tunings/macos-27.2-26B8125}
T25=${TUNINGS_HOST:-/System/Library/Audio/Tunings}
SCR=${SWSWEEP_SCRATCH:?scratch}
LOG=${GATE_LOG:-/Volumes/AuroraS500/traces/tainted/audio/20261107-sw-sweep-gate/host-refs.log}
DEST=${GATE_DEST:-buildbox:/srv/aurora/fast/o/swsweep/tainted/mac}
PROG=${GATE_PROG:-$G/prog.f32}
for a in "$@"; do
  free=$(df -g / | awk 'END{print $4}'); [ "$free" -lt 20 ] && { echo "disk guard"; exit 3; }
  mp=$(memory_pressure | tail -1 | grep -o '[0-9]*%' | tr -d %); [ "${mp:-50}" -lt 15 ] && sleep 30
  tree=$T27; [ -d "$T27/AID$a" ] || tree=$T25
  w=$SCR/AID$a; rm -rf "$w"; mkdir -p "$w/boxes"
  t0=$(date +%s)
  if python3 $G/gr.py --tunings $tree --aid $a --in $PROG --out $w/unit_in.f32 --meta $w/meta.json \
        --keep $w/boxes --box-host $BOXHOST > $w/gr.log 2>&1; then rc=0; else rc=$?; fi
  echo "{\"aid\":$a,\"rc\":$rc,\"tree\":\"$(basename $tree)\",\"seconds\":$(( $(date +%s)-t0 ))}" > $w/status.json
  rm -f $w/boxes/*.in.f32
  timeout 600 rsync -a -e 'ssh -o ConnectTimeout=10 -o BatchMode=yes' "$w/" "$DEST/AID$a/" && rm -rf "$w"
  echo "$(date +%H:%M:%S) AID$a rc=$rc $(tail -c 300 $SCR/AID$a/gr.log 2>/dev/null | tr '\n' ' ')" | tee -a $LOG
done
echo DONE
