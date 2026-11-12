#!/bin/sh
# SPDX-License-Identifier: MIT
# LSID tester kit, macOS part. Run as your normal user (no sudo) on the Mac's
# own macOS, from the kit directory:   sh macos/run-macos.sh
#
# Silent: it sets the built-in speakers to volume 0 and mute, checks both,
# and only then plays digital silence (all-zero samples) for 30 s while it
# reads back what macOS shows through public interfaces. It restores your
# volume and mute at the end (--leave-muted keeps them at 0/muted).
#
# Options: --seconds N (5..120, default 30), --leave-muted, --out FILE,
#          --prebuilt (use macos/lsidprobe-arm64, built from the same source, when
#          the Command Line Tools are not installed; its sha256 goes in the log)
# Output: one text log (default ./lsid-tester-macos-<build>-<UTC time>.log).
# Nothing leaves the Mac; you decide what to send.
set -u
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH LC_ALL=C

KIT=$(cd "$(dirname "$0")/.." && pwd -P)
SECONDS_ARG=30
LEAVE=""
OUT=""
PREBUILT=0
while [ $# -gt 0 ]; do
    case $1 in
        --seconds) SECONDS_ARG=$2; shift 2 ;;
        --leave-muted) LEAVE=--leave-muted; shift ;;
        --out) OUT=$2; shift 2 ;;
        --prebuilt) PREBUILT=1; shift ;;
        *) echo "usage: sh macos/run-macos.sh [--seconds N] [--leave-muted] [--prebuilt] [--out FILE]" >&2; exit 2 ;;
    esac
done

if [ "$(uname -s)" != Darwin ]; then echo "run this on macOS (the Linux part is linux/run-linux.sh)" >&2; exit 2; fi
if [ "$(id -u)" = 0 ]; then echo "run this as your normal user, not with sudo" >&2; exit 2; fi

BUILD=$(sw_vers -buildVersion)
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
[ -n "$OUT" ] || OUT="$PWD/lsid-tester-macos-$BUILD-$STAMP.log"
WORK=$(mktemp -d "${TMPDIR:-/tmp}/lsid-tester.XXXXXX") || exit 3
trap 'rm -rf "$WORK"' EXIT INT TERM

exec 3>&1
say() { printf '%s\n' "$*" >> "$OUT"; printf '%s\n' "$*" >&3; }
: > "$OUT"; chmod 600 "$OUT"

say "# LSID tester kit 20261113, macOS part"
say "DATA kit_sha256=$(cd "$KIT" && cat README.md linux/run-linux.sh macos/run-macos.sh macos/lsidprobe.c redact-check.sh | shasum -a 256 | cut -c1-64)"
say "DATA utc=$STAMP"
say "DATA macos=$(sw_vers -productVersion) build=$BUILD kernel=$(uname -r)"
say "DATA hw.model=$(sysctl -n hw.model 2>/dev/null) hw.target=$(sysctl -n hw.targettype 2>/dev/null || echo -) arch=$(uname -m)"

# 1. Build the probe (our own source, macos/lsidprobe.c) with the system compiler,
#    or with --prebuilt use the binary shipped beside it.
if [ "$PREBUILT" = 1 ]; then
    if [ -x "$KIT/macos/lsidprobe-arm64" ] && cp "$KIT/macos/lsidprobe-arm64" "$WORK/lsidprobe"; then
        say "RESULT build PASS prebuilt lsidprobe-arm64 sha256 $(shasum -a 256 "$WORK/lsidprobe" | cut -c1-64) (expected: see macos/SHA256SUMS)"
    else
        say "RESULT build FAIL --prebuilt: macos/lsidprobe-arm64 is missing"
        sh "$KIT/redact-check.sh" "$OUT" >&3 2>&1; exit 1
    fi
elif ! xcrun --find clang >/dev/null 2>&1; then
    say "RESULT build FAIL no C compiler: run 'xcode-select --install' (Command Line Tools) and run this again, or add --prebuilt"
    sh "$KIT/redact-check.sh" "$OUT" >&3 2>&1
    exit 1
elif xcrun clang -O2 -Wall -framework CoreAudio -framework AudioToolbox -framework IOKit -framework CoreFoundation \
        -o "$WORK/lsidprobe" "$KIT/macos/lsidprobe.c" > "$WORK/build.txt" 2>&1; then
    say "RESULT build PASS lsidprobe.c $(shasum -a 256 "$KIT/macos/lsidprobe.c" | cut -c1-16) ($(xcrun clang --version | head -1))"
else
    say "RESULT build FAIL $(tail -3 "$WORK/build.txt" | tr '\n' ' ')"
    sh "$KIT/redact-check.sh" "$OUT" >&3 2>&1
    exit 1
fi

# 2. The probe: device tree AID, Audio Unit inventory, IORegistry state, muted zero stream.
T0=$(date '+%Y-%m-%d %H:%M:%S')
"$WORK/lsidprobe" --seconds "$SECONDS_ARG" $LEAVE > "$WORK/probe.txt" 2>&1
PRC=$?
T1=$(date '+%Y-%m-%d %H:%M:%S')
cat "$WORK/probe.txt" >> "$OUT"; grep '^RESULT' "$WORK/probe.txt" >&3
say "DATA probe_exit=$PRC window=$T0..$T1"

AID=$(sed -n 's/^DATA aid=//p' "$WORK/probe.txt" | head -1)
case "$AID" in
    37|42|49|50) say "DATA lsid_expected_by_aid=yes (AID$AID: speakerguardd v2's LSID list is 37, 42, 49, 50)" ;;
    "") say "DATA lsid_expected_by_aid=unknown (no AID)" ;;
    *) say "DATA lsid_expected_by_aid=no (AID$AID)" ;;
esac

# 3. This Mac's tuning folder, as hashes only (the files stay private).
if [ -n "$AID" ]; then
    D="/System/Library/Audio/Tunings/AID$AID"
    if [ -d "$D" ]; then
        N=$(find "$D" -type f | wc -l | tr -d ' ')
        H=$(cd "$D" && find . -type f | sort | while read -r f; do shasum -a 256 "$f"; done | shasum -a 256 | cut -c1-64)
        L=$(find "$D" -type f -exec grep -l -a 'lsid' {} + 2>/dev/null | wc -l | tr -d ' ')
        say "RESULT tuning_folder INFO AID$AID: $N files, tree sha256 $H, $L file(s) mention 'lsid'"
    else
        say "RESULT tuning_folder INFO no per-Mac folder $D (older shared layout)"
    fi
fi

# 4. The unified log for the stream window: audio daemons only, lines that
#    mention the estimator, protection or thermal state. Numbers are kept.
if /usr/bin/log show --start "$T0" --end "$T1" --info --debug --style compact \
        --predicate 'process == "coreaudiod" OR senderImagePath CONTAINS[c] "AudioDSP" OR senderImagePath CONTAINS[c] "AppleSN012776" OR senderImagePath CONTAINS[c] "AppleEmbeddedAudio"' \
        > "$WORK/log.txt" 2>/dev/null; then
    TOTAL=$(grep -c -v -E '^(Filtering|Timestamp)' "$WORK/log.txt")
    PAT='lsid|lsm2|loudspeaker|systemid|system id|sysid|speakerprot|speaker prot|thermal|temperature|rdc|resistance|estimat|impedance|pilot'
    grep -i -E "$PAT" "$WORK/log.txt" | sed -E 's/^[0-9-]+ [0-9:.]+ +[A-Za-z]+ +//; s/\[[0-9]+:[0-9a-fx]+\]//; s/0x[0-9a-f]{6,}/0x_/g' \
        | cut -c1-240 | sort | uniq -c | sort -rn > "$WORK/matched.txt"
    M=$(awk '{s+=$1} END {print s+0}' "$WORK/matched.txt")
    say "RESULT unified_log INFO $TOTAL audio log lines in the window, $M mention estimator/protection/thermal words"
    head -40 "$WORK/matched.txt" | sed 's/^ */LOG /' >> "$OUT"
else
    say "RESULT unified_log SKIP log show failed"
fi

# 5. Verdicts.
FAILS=$(grep -c '^RESULT [^ ]* FAIL' "$OUT")
if grep -q '^RESULT component.lsid INFO registered' "$OUT"; then
    say "RESULT lsid_component INFO macOS $BUILD registers a standalone lsid Audio Unit"
else
    say "RESULT lsid_component INFO macOS $BUILD registers no standalone lsid Audio Unit"
fi
say "RESULT summary $( [ "$FAILS" = 0 ] && echo PASS || echo FAIL ) $FAILS failing step(s); probe exit $PRC"
echo >&3
sh "$KIT/redact-check.sh" "$OUT" >&3 2>&1
RC=$?
echo "log: $OUT" >&3
[ "$FAILS" = 0 ] && [ $RC = 0 ]
