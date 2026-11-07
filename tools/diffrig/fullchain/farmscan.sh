#!/bin/bash
# farmscan.sh NAME REF [MERGE_REF ...]: spp3 full-chain safety scan of speakerguardd REF on the arm64 farm
# (M1 Pro node ~/bvb-scan; aarch64 static builds from the buildbox). Run from the host.
# macOS references: mac_refs.py output in $R/tainted/scan/macos (host, once).
# Results: /Volumes/AuroraS500/traces/tainted/audio/20261106-spp3-fullchain-scan/NAME/<mac>/report.md
set -eu
N=$1; REF=$2; shift 2; MERGES="$*"
R=/Volumes/AuroraS500/work/speakerguardd/BVB-research
OUT=/Volumes/AuroraS500/traces/tainted/audio/20261106-spp3-fullchain-scan/$N
SSH="timeout 900 ssh -o ConnectTimeout=10"
NODE=${NODE:-m1pro}; RD=bvb-scan
# 1. aarch64 binaries of REF
timeout 1800 ssh -o ConnectTimeout=10 -n buildbox "/srv/aurora/fast/o/bvb-scan/a64build.sh $N $REF $MERGES" | tail -2
# 2. stage the node (refs/progs/ts/bundles once; bins + scanner every run)
$SSH -n $NODE "mkdir -p $RD/bins/$N $RD/bundles"
nice -n 15 rsync -a --exclude feeds.f32 --exclude logs $R/tainted/scan/macos $R/tainted/scan/progs $NODE:$RD/
for m in m2 m1pro j616s; do
  nice -n 15 rsync -a /Volumes/AuroraS500/reference/calibration/$m-20261106/tainted/*bound.tar.gz $NODE:$RD/bundles/$m.tar.gz
done
timeout 300 ssh -o ConnectTimeout=10 -n buildbox "cat /srv/aurora/fast/o/aud-rig/tainted/voice-g1/ts.json" | $SSH $NODE "cat > $RD/ts.json"
timeout 300 ssh -o ConnectTimeout=10 -n buildbox "tar -C /srv/aurora/fast/o/bvb-scan/bins/$N -cf - ." | $SSH $NODE "tar -C $RD/bins/$N -xf -"
rsync -a $R/scan/scan.py $NODE:$RD/
# 3. scan the three Macs in parallel, nice 19
$SSH -n $NODE "cd $RD && for m in m2 m1pro j616s; do nice -n 19 python3 scan.py --mac macos/\$m --progs progs --bundle bundles/\$m.tar.gz --bins bins/$N --ts ts.json --name \$m --out out/$N/\$m > out-$N-\$m.log 2>&1 & done; wait; grep -h RESULT out-$N-*.log"
# 4. bring back the small results (no audio)
mkdir -p $OUT && chmod 700 $(dirname $OUT)
rsync -a --include='*/' --include='report.md' --include='scan.json' --include='chain-info.json' --exclude='*' $NODE:$RD/out/$N/ $OUT/
$SSH -n $NODE "rm -rf $RD/out/$N/*/*/out.f32 $RD/out/$N/*/*/trace.jsonl"
cat $OUT/*/report.md | grep -E 'RESULT|INFORMATIONAL'
