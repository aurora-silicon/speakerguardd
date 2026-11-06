#!/bin/bash
# a64build.sh NAME REF [MERGE_REF ...]: static aarch64-musl sgv2-render, aurora-speaker-tuning and voicing-render
# of speakerguardd REF -> /srv/aurora/fast/o/bvb-scan/bins/NAME (runs on the buildbox; LSX's toolchain)
set -e
L=/srv/aurora/fast/o/LSX-farm; B=/srv/aurora/fast/o/bvb-scan; N=$1; REF=$2; shift 2
test -e /srv/aurora/fast/STOP && { echo STOP present; exit 3; }
export RUSTUP_HOME=$L/rustup CARGO_HOME=$L/cargo PATH=$L/cargo/bin:$PATH CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER=rust-lld RUSTFLAGS="-C linker-flavor=ld.lld"
mkdir -p $B/bins
[ -d $B/src ] || { git clone -q $L/src $B/src && git -C $B/src remote set-url origin https://github.com/aurora-silicon/speakerguardd.git; }
cd $B/src && git fetch -q origin
W=$B/wt/$N; rm -rf $W; git worktree prune; git worktree add -q --detach $W $REF
cd $W
for m in "$@"; do git -c user.name=BVB -c user.email=bvb@invalid merge -q --no-edit $m || { git merge --abort; echo "CONFLICT merging $m"; exit 4; }; done
git log --oneline -1
CARGO_TARGET_DIR=$B/a64target nice cargo build --release -q --target aarch64-unknown-linux-musl -p sgv2-render -p aurora-speaker-tuning --bin sgv2-render --bin aurora-speaker-tuning --example voicing-render
T=$B/a64target/aarch64-unknown-linux-musl/release; O=$B/bins/$N; mkdir -p $O
cp $T/sgv2-render $T/aurora-speaker-tuning $T/examples/voicing-render $O/
cd $B/src; git worktree remove --force $W
echo built $O
