#!/bin/bash
# Root on target after owner qualification. Transaction only; reboot separately.
set -euo pipefail
cd -- "$(dirname -- "$0")"
[[ $EUID == 0 ]]
python3 - <<'PY'
import json
from pathlib import Path
k=json.loads(Path('kit.json').read_text())
b=Path('/proc/device-tree/compatible').read_bytes().split(b'\0')[0].decode()
assert b == 'apple,'+k['board'], 'wrong board'
assert Path('/etc/os-release').read_text().split('VERSION_ID=')[1].splitlines()[0].strip('"') == str(k['fedora']), 'wrong Fedora version'
PY
sha256sum --check SHA256SUMS
# Fail before the transaction if the exact rollback pair has not been staged.
[[ $(ls rollback/speakerguardd-v2-1*.aarch64.rpm | wc -l) == 1 ]]
[[ $(ls rollback/speakerguardd-v2-selinux-*.noarch.rpm | wc -l) == 1 ]]
stamp=$(date -u +%Y%m%dT%H%M%SZ)
evidence=/var/lib/speakerguardd-plan-a/$stamp-install
mkdir -p "$evidence"
rpm -q speakerguardd-v2 speakerguardd-v2-selinux > "$evidence/before.txt"
# RPM scriptlets manage the supervisor; never disable protection or start a second daemon.
dnf install -y ./rpms/*.rpm 2>&1 | tee "$evidence/dnf.log"
rpm -q speakerguardd-v2 speakerguardd-v2-selinux > "$evidence/after.txt"
echo 'Installed. Keep playback muted. Reboot through the existing fleet18/19 one-shot with bootguard fallback, then run ./verify.sh as root.'
