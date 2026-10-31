#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run host-only RC checks, retaining every exit status and failure.

Use an admitted Linux build job with Rust, C, Python 3 and LADSPA headers.
No daemon, service, device, playback or recording is started. Nonzero suite
results remain nonzero acceptance results; subsequent checks still execute.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)
OUT = ROOT / 'validation'
OUT.mkdir(exist_ok=True)
os.environ['CARGO_BUILD_JOBS'] = '8'
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
if hasattr(os, 'sched_getaffinity'):
    os.sched_setaffinity(0, sorted(os.sched_getaffinity(0))[:8])


def manifest():
    suffixes = {'.rs', '.c', '.h', '.conf', '.py', '.toml', '.lock', '.json', '.csv'}
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(ROOT.rglob('*')) if p.is_file()
            and not {'.git', 'target', 'validation', '__pycache__'} & set(p.relative_to(ROOT).parts)
            and (p.suffix in suffixes or p.name in {'Makefile', 'preserve-recordings', 'speakerguardd.service'})}


before = manifest()
(OUT / 'source-manifest.json').write_text(json.dumps(before, indent=2) + '\n')
results = []


def run(name, command, extra=None, timeout=7200):
    env = dict(os.environ, **(extra or {}))
    start = time.monotonic()
    with (OUT / (name + '.log')).open('w') as log:
        log.write('command: ' + repr(command) + '\n')
        log.flush()
        try:
            rc = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            log.write('\nTIMEOUT\n')
            rc = 124
        except OSError as exc:
            log.write(str(exc) + '\n')
            rc = 127
    results.append(dict(name=name, command=command, rc=rc,
                        duration_s=round(time.monotonic()-start, 2)))
    (OUT / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    print(name, rc, flush=True)
    return rc


cargo = ['cargo', 'test', '--locked', '--release', '-j8']
compiled = run('compile', cargo + ['--no-run']) == 0
if compiled:
    run('full-release', cargo + ['--', '--test-threads=4'])
if compiled and '--standard-only' not in sys.argv:
    run('continuous-fallback', cargo + ['harness::continuous::all_30144_continuous_cases_stay_within_limits',
        '--', '--ignored', '--exact', '--nocapture'], {'PAIR_METRICS': str(OUT/'continuous-fallback.csv')})
    run('continuous-reference', cargo + ['harness::continuous::all_30144_continuous_cases_stay_within_limits',
        '--', '--ignored', '--exact', '--nocapture'],
        {'PAIR_METRICS': str(OUT/'continuous-reference.csv'), 'SPEAKERGUARDD_GATE_MODEL': '1'})
    run('mainline-j700-5000', cargo + ['harness::mainline::tests::random_interleavings_keep_the_properties',
        '--', '--exact', '--nocapture'], {'SPEAKERGUARDD_CASES':'5000', 'SPEAKERGUARDD_SEED':'8400000001'})
    run('vi-long', cargo + ['harness::vi::tests::vi_long_programs_keep_the_properties',
        '--', '--exact', '--nocapture'], {'SPEAKERGUARDD_VI_LONG':'1'})
run('clippy', ['cargo', 'clippy', '--locked', '--all-targets', '-j8'])
run('release-build', ['cargo', 'build', '--locked', '--release', '-j8'])
run('retention', ['python3', 'tools/test-preserve-recordings.py'])
run('neo-acceptance-fixtures', ['python3', 'tests/test_neo_three_boots.py'])
run('transfer-contract', ['python3', 'tools/check-transfer-contract.py'])
run('xlim', ['make', '-j8', 'filter-test'])
after = manifest()
changed = sorted(k for k in before.keys() | after.keys() if before.get(k) != after.get(k))
(OUT / 'source-unchanged.json').write_text(json.dumps(dict(unchanged=not changed, changed=changed), indent=2)+'\n')
raise SystemExit(1 if changed or any(r['rc'] for r in results) else 0)
