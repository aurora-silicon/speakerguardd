#!/bin/bash
# Root on the same board. Restore exact fleet 0.13 packages; reboot separately.
set -euo pipefail
cd -- "$(dirname -- "$0")"
[[ $EUID == 0 ]]
python3 -I - <<'PY'
import json
from pathlib import Path
k=json.loads(Path('kit.json').read_text())
assert Path('/proc/device-tree/compatible').read_bytes().split(b'\0')[0].decode() == 'apple,'+k['board'], 'wrong board'
PY
sha256sum --check SHA256SUMS
rpms=(rollback/*.rpm)
[[ ${#rpms[@]} == 2 ]]
for p in "${rpms[@]}"; do
    [[ $(rpm -qp --qf '%{VERSION}-%{RELEASE}' "$p") == 1.0.0~alpha.1-0.13.20261107git6d13c704202e.fc* ]]
done
stamp=$(date -u +%Y%m%dT%H%M%SZ)
evidence=/var/lib/speakerguardd-plan-a/$stamp-rollback
mkdir -p "$evidence"
rpm -q speakerguardd-v2 speakerguardd-v2-selinux > "$evidence/before.txt"
dnf downgrade -y "${rpms[@]}" 2>&1 | tee "$evidence/dnf.log"
rpm -q speakerguardd-v2 speakerguardd-v2-selinux > "$evidence/after.txt"
echo 'Fleet 0.13 restored. Keep playback muted and reboot through the existing one-shot/fallback path; provisioning recompiles the interim blob. Then run ./verify.sh --interim.'
