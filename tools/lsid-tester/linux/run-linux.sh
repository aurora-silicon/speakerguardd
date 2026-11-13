#!/bin/bash
# SPDX-License-Identifier: MIT
# LSID tester kit, Linux step 2:   sudo bash linux/run-linux.sh [--bundle PATH] [--collection TARBALL] [--out FILE]
#
# Silent. Nothing here plays sound: the self-check's streams are all-zero
# samples, and speakerguardd v2 keeps LSID speakers muted whatever happens.
#   1. identify, validate and compile this Mac's speaker tuning bundle with the
#      compiler built by linux/build-v2.sh (speakerguardd v2 >= 4d55e1d5);
#   2. voicing-check (offline render checks; no audio device);
#   3. print the speaker wiring derived from the device tree and the tuning;
#   4. aurora-speaker-upgrade --check (dry run: collect, compile, self-check in
#      /run; nothing stored or switched), if installed;
#   5. speakerguardd-selfcheck verify on the installed v2 blob (zero samples
#      only), if speakerguardd v2 is installed and set up;
#   6. v2's runtime state from the journal, SELinux denials.
# Work files (the compiled blob, the tuning dump) go to a private temporary
# directory that is deleted at exit. The log holds results, numbers, hashes and
# error text only; redact-check.sh runs on it at the end.
#
# --bundle PATH       the tuning bundle (directory or .tar.gz from collect-macos.sh);
#                     default: the one aurora-speaker-upgrade stored, else the system firmware
# --collection FILE   also pass this collection tarball to aurora-speaker-upgrade --check
# --out FILE          log path (default ./lsid-tester-linux-<UTC time>.log)
set -u
export LC_ALL=C
KIT=$(cd "$(dirname "$0")/.." && pwd -P)
BUNDLE=""
COLLECTION=""
OUT=""
while [ $# -gt 0 ]; do
    case $1 in
        --bundle) BUNDLE=$2; shift 2 ;;
        --collection) COLLECTION=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        *) echo "usage: sudo bash linux/run-linux.sh [--bundle PATH] [--collection TARBALL] [--out FILE]" >&2; exit 2 ;;
    esac
done
[ "$(uname -s)" = Linux ] || { echo "run this under Linux (the macOS part is macos/run-macos.sh)" >&2; exit 2; }
[ "$(id -u)" = 0 ] || { echo "run with sudo: the self-check and the upgrade check need root" >&2; exit 2; }

STAMP=$(date -u +%Y%m%dT%H%M%SZ)
[ -n "$OUT" ] || OUT="$PWD/lsid-tester-linux-$STAMP.log"
WORK=$(mktemp -d /var/tmp/lsid-tester.XXXXXX) || exit 3
chmod 700 "$WORK"
trap 'rm -rf "$WORK"' EXIT INT TERM
: > "$OUT"; chmod 600 "$OUT"
[ -n "${SUDO_USER:-}" ] && chown "$SUDO_USER" "$OUT" 2>/dev/null
SINCE=$(date '+%Y-%m-%d %H:%M:%S')

log() { printf '%s\n' "$*" >> "$OUT"; case $1 in RESULT*|\#*) printf '%s\n' "$*" ;; esac; }
res() { log "RESULT $1 $2 ${*:3}"; }
indent() { sed 's/^/  /' >> "$OUT"; }
py() { python3 -I -c "$@"; }

log "# LSID tester kit 20261113, Linux part"
log "DATA kit_sha256=$(cd "$KIT" && cat README.md linux/build-v2.sh linux/run-linux.sh macos/run-macos.sh macos/lsidprobe.c redact-check.sh | sha256sum | cut -c1-64)"
log "DATA utc=$STAMP"
. /etc/os-release 2>/dev/null
log "DATA os=${PRETTY_NAME:-?} kernel=$(uname -r)"
log "DATA dt.model=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null) dt.compatible=$(tr '\0' ' ' < /proc/device-tree/compatible 2>/dev/null)"
SPID=-
[ -r /proc/device-tree/chosen/asahi,speaker-id ] && SPID=$(od -An -tu4 --endian=big /proc/device-tree/chosen/asahi,speaker-id | tr -d ' ')
log "DATA chosen.speaker_id=$SPID selinux=$(getenforce 2>/dev/null || echo none)"
for p in speakerguardd-v2 aurora-speaker-upgrade pipewire; do
    v=$( (rpm -q "$p" 2>/dev/null || pacman -Q "$p" 2>/dev/null || echo "$p not-installed") | head -1)
    log "DATA pkg $v"
done

# ---- the compiler
CLI=""; REV="-"
if [ -r "$KIT/work/build.env" ]; then
    benv() { sed -n "s/^$1=//p" "$KIT/work/build.env" | head -1; }
    SRC=$(benv SRC); REV=$(benv REV); DIRTY=$(benv DIRTY); BIN=$(benv BIN)
    if [ -x "${BIN:-}" ]; then
        CLI=$BIN
        res tool PASS "aurora-speaker-tuning built from speakerguardd $REV (contains 4d55e1d5; $DIRTY modified tracked files) sha256 $(sha256sum "$CLI" | cut -c1-16)"
    fi
fi
if [ -z "$CLI" ]; then
    if command -v aurora-speaker-tuning >/dev/null 2>&1; then
        CLI=$(command -v aurora-speaker-tuning)
        res tool WARN "linux/build-v2.sh not run: using the packaged $CLI ($(rpm -qf "$CLI" 2>/dev/null || echo unknown package)); it may predate 4d55e1d5"
    else
        res tool FAIL "no aurora-speaker-tuning: run 'sh linux/build-v2.sh' as your user first"
    fi
fi
WIRING_PY=""
for w in "${SRC:-/nonexistent}/packaging/session/wiring.py" /usr/libexec/speakerguardd-v2/wiring.py; do
    [ -r "$w" ] && { WIRING_PY=$w; break; }
done

# ---- the bundle
SRCARGS=()
if [ -n "$BUNDLE" ]; then SRCARGS=("$BUNDLE"); BDESC="--bundle (given)"
elif [ -d /var/lib/aurora-speaker-upgrade/bundle ]; then SRCARGS=(/var/lib/aurora-speaker-upgrade/bundle); BDESC="aurora-speaker-upgrade's stored bundle"
else BDESC="system firmware (vendorfw, then the ESP archive)"; fi
log "DATA bundle_source=$BDESC"

LSID=unknown
if [ -n "$CLI" ]; then
    # identify: AID, macOS build, speaker ID, digest. unit_id (a serial hash) is left out.
    if "$CLI" identify --json "${SRCARGS[@]}" > "$WORK/id.json" 2> "$WORK/id.err"; then
        py 'import json,sys; d=json.load(open(sys.argv[1])); d.pop("unit_id",None); d.pop("source",None)
print(" ".join(f"{k}={d[k]}" for k in ("kind","aid","acoustic_id","macos_build","speaker_id","board","unit_bound","verified","files","digest","reader") if k in d))' "$WORK/id.json" > "$WORK/id.txt"
        res identify PASS "$(cat "$WORK/id.txt")"
    else
        res identify FAIL "$(tail -2 "$WORK/id.err" | tr '\n' ' ')"
    fi

    # validate: diagnostic codes (messages only for errors).
    "$CLI" validate --json "${SRCARGS[@]}" > "$WORK/val.json" 2> "$WORK/val.err"; VRC=$?
    VSUM=$(py 'import json,sys
try: d=json.load(open(sys.argv[1]))
except Exception as e:
    print("unparsable:", e); sys.exit()
it=d.get("items",[]); c={}
for x in it: c[x["severity"]]=c.get(x["severity"],0)+1
codes=" ".join(sorted({x["severity"][0]+":"+x["code"] for x in it}))
print("%d errors, %d warnings, %d notes; %s" % (c.get("error",0), c.get("warning",0), c.get("info",0), codes))
for x in it:
    if x["severity"]=="error": print("  error", x["code"], x["context"], x["message"][:300])' "$WORK/val.json")
    res validate "$([ $VRC = 0 ] && echo PASS || echo FAIL)" "$(head -1 <<< "$VSUM")"
    tail -n +2 <<< "$VSUM" | indent

    # compile for this Mac (board and unit from the running device tree).
    "$CLI" compile "${SRCARGS[@]}" -o "$WORK/params.blob" > "$WORK/c.out" 2> "$WORK/c.err"; CRC=$?
    CODES=$(grep -o -E '^(info|warning|error)\[[a-z0-9-]+\]' "$WORK/c.err" | sort -u | tr '\n' ' ')
    WROTE=$(grep -E '^wrote ' "$WORK/c.err" | sed "s#^wrote [^ ]* #blob #")
    if [ $CRC = 0 ] && [ -s "$WORK/params.blob" ]; then
        res compile PASS "$WROTE; sha256 $(sha256sum "$WORK/params.blob" | cut -c1-64)"
    else
        res compile FAIL "rc $CRC"
        grep -E '^error' "$WORK/c.err" | cut -c1-300 | indent
    fi
    log "DATA compile.diagnostics=$CODES"
    if grep -q -E '\[lsid-(disabled|legacy-opaque)\]' "$WORK/c.err"; then
        LSID=present
        if grep -q '\[lsm2-release-gated\]' "$WORK/c.err"; then
            res lsid PASS "LSID companion present; compiled MUTED (lsm2-release-gated), as v2 requires until LSID is at parity"
        else
            res lsid FAIL "LSID companion present but the compile did not report lsm2-release-gated"
        fi
    elif [ $CRC = 0 ]; then
        LSID=absent
        res lsid PASS "LSID absent: this tuning has no LSID companion (nothing to gate)"
    else
        res lsid SKIP "compile failed; LSID state unknown"
    fi
    # The blob decoder's companion/gate lines only: no other field, so no tuning value reaches the log.
    if [ -s "$WORK/params.blob" ]; then
        "$CLI" blob "$WORK/params.blob" 2>&1 | grep -i -E 'companion|gate|lsid|mute' | grep -v -E '[0-9]+\.[0-9]+' | cut -c1-200 | head -12 | indent
    fi

    # voicing-check: pass/fail per check and the dB numbers.
    # The packaged parity attestations, when installed (a tuning without one is refused: expected for unattested builds).
    ATT=(); [ -r /usr/share/speakerguardd/parity.json ] && ATT=(--attestations /usr/share/speakerguardd/parity.json)
    [ -n "${SRC:-}" ] && [ -r "$SRC/packaging/session/parity.json" ] && ATT=(--attestations "$SRC/packaging/session/parity.json")
    "$CLI" voicing-check "${ATT[@]}" "${SRCARGS[@]}" > "$WORK/vc.out" 2>&1; VCRC=$?
    case $VCRC in 0) V=PASS ;; 1) V=REFUSED ;; *) V=FAIL ;; esac
    res voicing_check "$V" "exit $VCRC${ATT:+ (attestations ${ATT[1]##*/})}"
    cut -c1-300 "$WORK/vc.out" | head -40 | indent

    # wiring: derived from the device tree and the tuning graph.
    if [ -n "$WIRING_PY" ]; then
        if "$CLI" dump --json "${SRCARGS[@]}" > "$WORK/dump.json" 2>/dev/null && \
           python3 -I "$WIRING_PY" describe --tuning-json "$WORK/dump.json" > "$WORK/wiring.json" 2> "$WORK/wiring.err"; then
            H=$(py 'import json,sys,hashlib; w=json.load(open(sys.argv[1]))["wiring"]; print(w.get("hash") or w.get("wiring_hash") or hashlib.sha256(json.dumps(w,sort_keys=True).encode()).hexdigest()[:16])' "$WORK/wiring.json" 2>/dev/null)
            res wiring PASS "derived ($WIRING_PY), wiring $H"
            py 'import json,sys; print(json.dumps(json.load(open(sys.argv[1])), sort_keys=True, indent=1))' "$WORK/wiring.json" | head -120 | indent
        else
            res wiring FAIL "$(tail -2 "$WORK/wiring.err" 2>/dev/null | tr '\n' ' ')"
        fi
    else
        res wiring SKIP "wiring.py not found (run linux/build-v2.sh, or install speakerguardd-v2)"
    fi
fi

# ---- aurora-speaker-upgrade --check (dry run; nothing stored or switched)
if command -v aurora-speaker-upgrade >/dev/null 2>&1; then
    UARGS=(--check --json "$WORK/upgrade.json")
    [ -n "$COLLECTION" ] && UARGS+=(--collection "$COLLECTION")
    timeout 600 aurora-speaker-upgrade "${UARGS[@]}" > "$WORK/up.out" 2>&1; URC=$?
    case $URC in 0) U=PASS ;; 1) U=REFUSED ;; 2) U="NOT-NOW" ;; *) U=FAIL ;; esac
    res upgrade_check "$U" "exit $URC"
    if [ -s "$WORK/upgrade.json" ]; then
        py 'import json,sys
def scrub(x):
  if isinstance(x,dict): return {k:scrub(v) for k,v in x.items() if k not in ("unit_id","serial","serial_number","store","scratch")}
  if isinstance(x,list): return [scrub(v) for v in x]
  return x
print(json.dumps(scrub(json.load(open(sys.argv[1]))), sort_keys=True, indent=1))' "$WORK/upgrade.json" | head -80 | indent
    fi
    grep -v -E '^\s*$' "$WORK/up.out" | cut -c1-300 | tail -25 | indent
else
    res upgrade_check SKIP "aurora-speaker-upgrade is not installed"
fi

# ---- speakerguardd-selfcheck verify on the installed blob (zero samples only)
SC=/usr/libexec/speakerguardd-v2/speakerguardd-selfcheck
if [ -x "$SC" ] && [ -r /etc/speakerguardd/speakerguardd.conf ]; then
    . /etc/speakerguardd/speakerguardd.conf
    if [ -n "${SGV2_PARAMS:-}" ] && [ -r "$SGV2_PARAMS" ]; then
        log "DATA installed_blob_sha256=$(sha256sum "$SGV2_PARAMS" | cut -c1-64)"
        # shellcheck disable=SC2086
        timeout 300 "$SC" verify --params "$SGV2_PARAMS" --json "$WORK/sc.json" -- ${SGV2_TUNING:-} > "$WORK/sc.out" 2>&1; SRC_RC=$?
        case $SRC_RC in 0) S=PASS ;; 1) S=REFUSED ;; 2) S="CANNOT-RUN" ;; *) S=FAIL ;; esac
        res selfcheck_verify "$S" "exit $SRC_RC"
        cut -c1-400 "$WORK/sc.out" | head -30 | indent
    else
        res selfcheck_verify SKIP "no SGV2_PARAMS blob in /etc/speakerguardd/speakerguardd.conf"
    fi
else
    res selfcheck_verify SKIP "speakerguardd v2 is not installed/provisioned on this Mac"
fi

# ---- runtime state
if systemctl cat speakerguardd-v2.service >/dev/null 2>&1; then
    log "DATA v2.supervisor=$(systemctl is-active speakerguardd-v2 2>&1) legacy=$(systemctl is-active speakerguardd speakersafetyd 2>&1 | tr '\n' '/')"
    journalctl -b -u speakerguardd-v2 -o cat --no-pager 2>/dev/null | grep -i -E 'lsid|gate|mute|grant|refus|fault|ready|fail-safe|unavailable' | tail -10 | cut -c1-300 | indent
fi
AVC=$(journalctl --since "$SINCE" --no-pager 2>/dev/null | grep -c 'avc:  denied')
log "DATA avc_denials_during_run=$AVC failed_units=$(systemctl --failed --no-legend 2>/dev/null | wc -l)"

FAILS=$(grep -c -E '^RESULT [^ ]+ FAIL' "$OUT")
res summary "$([ "$FAILS" = 0 ] && echo PASS || echo FAIL)" "$FAILS failing step(s); LSID $LSID"
echo
sh "$KIT/redact-check.sh" "$OUT"; RRC=$?
echo "log: $OUT"
[ "$FAILS" = 0 ] && [ $RRC = 0 ]
