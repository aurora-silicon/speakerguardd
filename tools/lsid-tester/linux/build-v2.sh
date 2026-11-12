#!/bin/sh
# SPDX-License-Identifier: MIT
# LSID tester kit, Linux step 1: build speakerguardd v2's tuning compiler.
# Run as your normal user (not root):   sh linux/build-v2.sh [--src DIR] [--rev REV]
#
# Default: clones https://github.com/aurora-silicon/speakerguardd (branch v2)
# into the kit's work/speakerguardd and builds aurora-speaker-tuning there.
# --src DIR uses your own checkout as it is (nothing is checked out or
# changed in it); so does running the kit from tools/lsid-tester inside one. Either way the commit must contain 4d55e1d5 (v2 with the
# clippy 1.100 fixes; compiler behaviour is the same as 26b88eac).
# Needs git and a Rust toolchain (cargo); about 2-4 minutes on an M-series Mac.
set -eu
KIT=$(cd "$(dirname "$0")/.." && pwd -P)
MIN=4d55e1d5
SRC=""
REV=origin/v2
while [ $# -gt 0 ]; do
    case $1 in
        --src) SRC=$2; shift 2 ;;
        --rev) REV=$2; shift 2 ;;
        *) echo "usage: sh linux/build-v2.sh [--src DIR] [--rev REV]" >&2; exit 2 ;;
    esac
done
[ "$(id -u)" != 0 ] || { echo "build as your normal user, not root" >&2; exit 2; }
command -v cargo >/dev/null 2>&1 || { echo "cargo not found: install Rust (rustup or your distribution's rust/cargo package)" >&2; exit 1; }

# Inside a speakerguardd checkout (tools/lsid-tester), build that checkout as it is.
if [ -z "$SRC" ] && [ -d "$KIT/../../crates/aurora-speaker-tuning" ] && git -C "$KIT/../.." rev-parse --git-dir >/dev/null 2>&1; then
    SRC=$(cd "$KIT/../.." && pwd -P)
fi
if [ -z "$SRC" ]; then
    SRC=$KIT/work/speakerguardd
    if [ ! -d "$SRC/.git" ]; then
        mkdir -p "$KIT/work"
        git clone --quiet --branch v2 https://github.com/aurora-silicon/speakerguardd "$SRC"
    else
        git -C "$SRC" fetch --quiet origin v2
    fi
    git -C "$SRC" -c advice.detachedHead=false checkout --quiet --detach "$REV"
fi
HEAD=$(git -C "$SRC" rev-parse HEAD)
if ! git -C "$SRC" merge-base --is-ancestor "$MIN" HEAD 2>/dev/null; then
    echo "speakerguardd at $HEAD does not contain $MIN: update the checkout (git fetch; v2 branch)" >&2
    exit 1
fi
DIRTY=$(git -C "$SRC" status --porcelain --untracked-files=no | wc -l | tr -d ' ')
(cd "$SRC" && cargo build --release --locked -p aurora-speaker-tuning --bin aurora-speaker-tuning)
BIN=$SRC/target/release/aurora-speaker-tuning
[ -x "$BIN" ] || { echo "build finished but $BIN is missing" >&2; exit 1; }
mkdir -p "$KIT/work"
printf 'SRC=%s\nREV=%s\nDIRTY=%s\nBIN=%s\n' "$SRC" "$HEAD" "$DIRTY" "$BIN" > "$KIT/work/build.env"   # read with sed, never sourced
echo "built aurora-speaker-tuning at speakerguardd $HEAD ($DIRTY modified tracked files)"
echo "next: sudo bash linux/run-linux.sh"
