#!/bin/sh
# SPDX-License-Identifier: MIT
# LSID tester kit: check a log before you send it.   sh redact-check.sh LOG
#
# Looks for anything that could identify you or this Mac: your user and full
# name, the host name, the serial number and its speakerguardd unit id (a hash
# of the serial), the platform UUID / machine-id, MAC and IP addresses, e-mail
# addresses, home-directory paths and UUIDs, plus binary data and long encoded
# blobs (a tuning file or recording would show up that way). It reads these
# values on this Mac and never prints them: a hit names the line and the kind.
# Run it on the Mac the log came from (as root on Linux, to read the serial).
# Exit 0: nothing found. 1: hits (edit the log, or ask us). 2: usage.
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export LC_ALL=C
LOG=${1:-}
[ -n "$LOG" ] && [ -f "$LOG" ] || { echo "usage: sh redact-check.sh LOG" >&2; exit 2; }

HITS=0
hit() { echo "redact-check: line $1: $2"; HITS=$((HITS + 1)); }

# Fixed strings to look for (case-insensitive), one "kind<TAB>value" per line.
SECRETS=$(mktemp "${TMPDIR:-/tmp}/lsid-redact.XXXXXX") || exit 3
trap 'rm -f "$SECRETS"' EXIT INT TERM
add() { [ -n "$2" ] && [ "${#2}" -ge 3 ] && printf '%s\t%s\n' "$1" "$2" >> "$SECRETS"; }

for u in "$(id -un 2>/dev/null)" "${SUDO_USER:-}" "${USER:-}"; do
    case "$u" in root|"") ;; *) add "user name" "$u" ;; esac
done
add "host name" "$(hostname 2>/dev/null)"
add "host name" "$(hostname -s 2>/dev/null)"
SERIAL=""
if [ "$(uname -s)" = Darwin ]; then
    add "computer name" "$(scutil --get ComputerName 2>/dev/null)"
    add "host name" "$(scutil --get LocalHostName 2>/dev/null)"
    add "full name" "$(id -F 2>/dev/null)"
    IOP=$(ioreg -rd1 -c IOPlatformExpertDevice 2>/dev/null)
    SERIAL=$(printf '%s\n' "$IOP" | sed -n 's/.*"IOPlatformSerialNumber" = "\(.*\)"/\1/p' | head -1)
    add "platform UUID" "$(printf '%s\n' "$IOP" | sed -n 's/.*"IOPlatformUUID" = "\(.*\)"/\1/p' | head -1)"
else
    for u in "${SUDO_USER:-}" "$(id -un 2>/dev/null)"; do
        [ -n "$u" ] && [ "$u" != root ] && add "full name" "$(getent passwd "$u" 2>/dev/null | cut -d: -f5 | cut -d, -f1)"
    done
    [ -r /proc/device-tree/serial-number ] && SERIAL=$(tr -d '\0' < /proc/device-tree/serial-number)
    [ -r /etc/machine-id ] && add "machine-id" "$(cat /etc/machine-id)"
fi
if [ -n "$SERIAL" ]; then
    add "serial number" "$SERIAL"
    if command -v sha256sum >/dev/null 2>&1; then H=$(printf 'aurora-speaker-unit-v1\000%s' "$SERIAL" | sha256sum | cut -c1-64)
    else H=$(printf 'aurora-speaker-unit-v1\000%s' "$SERIAL" | shasum -a 256 | cut -c1-64); fi
    add "unit id (serial hash)" "$H"
    add "unit id (serial hash)" "$(printf '%s' "$H" | cut -c1-16)"
else
    echo "redact-check: note: serial number not readable here (on Linux run with sudo); serial and unit id not checked"
fi

while IFS="$(printf '\t')" read -r kind value; do
    grep -n -i -F -- "$value" "$LOG" | cut -d: -f1 | while read -r n; do echo "redact-check: line $n: $kind"; done
done < "$SECRETS" > "$SECRETS.out"
if [ -s "$SECRETS.out" ]; then sort -u "$SECRETS.out"; HITS=$((HITS + $(sort -u "$SECRETS.out" | wc -l))); fi
rm -f "$SECRETS.out"

pat() {  # kind, extended regex
    grep -n -i -E -- "$2" "$LOG" | cut -d: -f1 | while read -r n; do echo "redact-check: line $n: $1"; done
}
PATS=$( {
    pat "MAC address" '([0-9a-f]{2}[:-]){5}[0-9a-f]{2}'
    pat "IPv4 address" '(^|[^0-9.])((25[0-5]|2[0-4][0-9]|1?[0-9]?[0-9])\.){3}(25[0-5]|2[0-4][0-9]|1?[0-9]?[0-9])([^0-9.]|$)'
    pat "e-mail address" '[a-z0-9._%+-]+@[a-z0-9.-]+\.[a-z]{2,}'
    pat "home path" '(/home/|/Users/)[^/ ]+'
    pat "UUID" '[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}'
    pat "long encoded blob" '[A-Za-z0-9+/=]{120,}'
} )
if [ -n "$PATS" ]; then
    # 127.0.0.1-style loopback and version strings like 1.6.10.1 are not addresses.
    PATS=$(printf '%s\n' "$PATS" | while read -r line; do
        n=$(echo "$line" | sed 's/^redact-check: line \([0-9]*\):.*/\1/')
        if echo "$line" | grep -q IPv4 && sed -n "${n}p" "$LOG" | grep -q -E '(^|[^0-9.])(127\.0\.0\.1|0\.0\.0\.0)([^0-9.]|$)'; then continue; fi
        echo "$line"
    done)
    [ -n "$PATS" ] && { printf '%s\n' "$PATS"; HITS=$((HITS + $(printf '%s\n' "$PATS" | wc -l))); }
fi

# Binary data or an oversized log means something other than text got in.
if [ "$(LC_ALL=C tr -d '\11\12\15\40-\176\200-\377' < "$LOG" | wc -c | tr -d ' ')" != 0 ]; then
    hit 0 "binary/control bytes in the log"
fi
SIZE=$(wc -c < "$LOG" | tr -d ' ')
[ "$SIZE" -le 2000000 ] || hit 0 "log is $SIZE bytes (over 2 MB): more than text results"

if [ "$HITS" = 0 ]; then
    echo "redact-check: PASS $LOG: no personal or identifying data found ($SIZE bytes)"
    exit 0
fi
echo "redact-check: FAIL $LOG: $HITS hit(s) above. Edit those lines out (or replace the value with <redacted>) and run this again."
exit 1
