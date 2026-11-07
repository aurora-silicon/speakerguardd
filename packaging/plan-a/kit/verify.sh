#!/bin/bash
# Root on the selected target. Silent, read-only; no tuning values in output.
set -euo pipefail
cd -- "$(dirname -- "$0")"
exec python3 - "$@" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import pwd
import shlex
import subprocess
import sys

interim = sys.argv[1:] == ['--interim']
assert not sys.argv[1:] or interim, 'usage: verify.sh [--interim]'
assert os.geteuid() == 0, 'run as root'
kit = json.loads(Path('kit.json').read_text())
board = Path('/proc/device-tree/compatible').read_bytes().split(b'\0')[0].decode().removeprefix('apple,')
assert board == kit['board'], 'wrong physical board'
def run(*args, **kw):
    return subprocess.run(args, check=True, text=True, capture_output=True, timeout=60, **kw).stdout

def kv(path):
    return dict(line.split('=', 1) for line in Path(path).read_text().splitlines() if '=' in line and not line.startswith('#'))

conf = {k: shlex.split(v)[0] for k, v in kv('/etc/speakerguardd/speakerguardd.conf').items()}
args = shlex.split(conf['SGV2_ARGS'])
card = args[args.index('--card') + 1]
params = args[args.index('--params') + 1]
assert run('systemctl', 'is-active', 'speakerguardd-v2.service').strip() == 'active'
for unit in ('speakerguardd.service', 'speakersafetyd.service'):
    assert subprocess.run(['systemctl', 'is-active', '--quiet', unit]).returncode != 0, 'two protection stacks'
grant = run('amixer', '-c', card, 'cget', "name=Speaker Protection")
assert ': values=1' in grant, 'protection not granted'
gate = json.loads(Path('/usr/share/speakerguardd/gate.json').read_text())
if interim:
    assert not gate.get('voicing_pre', {}).get(board), 'voicing still enabled'
else:
    assert gate['voicing_pre'] == {board: True}, 'gate must select exactly this board'
    for package in ('speakerguardd-v2', 'speakerguardd-v2-selinux'):
        assert run('rpm', '-q', '--qf', '%{VERSION}-%{RELEASE}', package).strip() == kit['evr'], 'wrong RPM snapshot'
    result = run('/usr/libexec/speakerguardd-v2/blob-info', '--summary', params, board)
    assert 'voicing=present' in result and 'voicing_plugin=ready' in result
    print(result.strip())
bind = kv('/var/lib/speakerguardd/provision.bind')
assert bind['voicing_pre'] == ('false' if interim else 'true'), 'provisioned mode mismatch'
check = kv('/var/lib/speakerguardd/selfcheck.bind')
assert check['result'] == 'pass', 'selfcheck not passed'
digest = hashlib.sha256(Path(params).read_bytes()).hexdigest()
assert bind['params_sha256'] == check['params_sha256'] == digest, 'checked blob mismatch'
if not interim:
    assert hashlib.sha256(Path('/run/speakerguardd/params.bin').read_bytes()).hexdigest() == digest, 'published blob mismatch'
user = pwd.getpwnam(conf['SGV2_USER'])
env = dict(os.environ, XDG_RUNTIME_DIR=f'/run/user/{user.pw_uid}', DBUS_SESSION_BUS_ADDRESS=f'unix:path=/run/user/{user.pw_uid}/bus')
dump = json.loads(run('setpriv', f'--reuid={user.pw_uid}', f'--regid={user.pw_gid}', '--init-groups', '--', 'pw-dump', env=env))
nodes = {o['id']: o.get('info', {}).get('props', {}) for o in dump if o['type'] == 'PipeWire:Interface:Node'}
by_name = {p.get('node.name'): i for i, p in nodes.items()}
raw = by_name['alsa_output.platform-sound.RawSpeakers']
assert nodes[raw].get('sgv2.stage') == 'protect', 'protect stage missing'
assert f'sgv2.protect.{board}.out' in by_name, 'protect output missing'
assert f'sgv2.speaker.{board}' in by_name and f'sgv2.sense.{board}' in by_name
links = [o['info'] for o in dump if o['type'] == 'PipeWire:Interface:Link']
if not interim:
    capture = by_name[f'sgv2.voicing.{board}']
    assert nodes[capture].get('sgv2.stage') == 'voicing-pre'
    output = by_name[f'sgv2.voicing.{board}.out']
    assert nodes[output].get('target.object') == 'alsa_output.platform-sound.RawSpeakers'
    matched = [l for l in links if int(l['output-node-id']) == output and int(l['input-node-id']) == raw]
    assert len(matched) == kit['channels'], 'incomplete voicing -> RawSpeakers links'
    assert all(l['state'] in ('active', 'paused') for l in matched), 'voicing link failure'
else:
    assert f'sgv2.voicing.{board}' not in by_name, 'generated voicing node remains'
assert run('getenforce').strip() == 'Enforcing'
audit = subprocess.run(['ausearch', '-m', 'AVC,USER_AVC,SELINUX_ERR', '-ts', 'boot'], text=True, capture_output=True, timeout=60)
assert audit.returncode in (0, 1), 'audit query failed'
assert 'type=' not in audit.stdout, 'AVC or SELinux error this boot'
assert audit.returncode == 0 or '<no matches>' in audit.stdout + audit.stderr, 'audit evidence unavailable'
print(f'PASS {board}: v2 active, granted, checked blob, protect graph, ' + ('interim' if interim else 'voicing-pre linked') + ', Enforcing, 0 AVC; no playback')
PY
