#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Check Norm timing against retained AU metadata and uncompensated audio.

Run on the buildbox. No AU execution, tuning values or fitted alignment.
An optional older box renderer checks bit identity over complete 512-frame
calls (its legacy flush/crop is internal to that older binary). Short final
calls are reported separately: the new runner meters their actual length.
"""
import argparse
import hashlib
import json
import pathlib
import subprocess
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', type=pathlib.Path, required=True)
    ap.add_argument('--renderer', type=pathlib.Path, required=True)
    ap.add_argument('--baseline', type=pathlib.Path)
    ap.add_argument('--work', type=pathlib.Path, required=True)
    a = ap.parse_args()
    a.work.mkdir(parents=True, exist_ok=True)
    probes = json.loads((a.root / 'probes.json').read_text())['cases']
    rows = json.loads((a.root / 'inventory.json').read_text())['rows']
    records = []
    for row in rows:
        if row['type'] != 'ldnm':
            continue
        ready = a.root / 'mac' / row['id'] / 'ready.json'
        if not ready.exists():
            records.append({'id': row['id'], 'status': 'missing-au'})
            continue
        md = json.loads(ready.read_text())
        metas = list(md.get('meta', {}).values())
        rec = {'id': row['id'], 'aid': row['aid'], 'cases': 0, 'planned': sum(p['ch'] == row['ch'] for p in probes),
               'complete_call_mismatches': 0, 'short_final_call_differences': 0,
               'timing_misses': [], 'errors': [], 'cached_onset_aligned': None}
        if not metas:
            rec['status'] = 'missing-au-metadata'
            records.append(rec)
            continue
        meta = metas[0]
        fs = meta['sample_rate']
        delay = round(meta['latency_s'] * fs)
        rec.update(au_latency_frames=delay, au_tail_frames=round(meta['tail_s'] * fs))
        def render(binary, inp, out):
            cmd = [str(binary), '--strip', str(a.root / row['strip']),
                   '--only', row['box'], '--ch', str(row['ch']), str(inp), str(out)]
            return subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        inp = a.work / 'impulse.f32'
        out = a.work / 'new.f32'
        old = a.work / 'old.f32'
        pulse = np.zeros((2048, row['ch']), '<f4')
        pulse[17, 0] = 0.001
        pulse.tofile(inp)
        r = render(a.renderer, inp, out)
        if r.returncode:
            rec.update(status='v2-refused', error=r.stderr[-400:])
            records.append(rec)
            continue
        y = np.fromfile(out, '<f4').reshape(-1, row['ch'])
        support = np.flatnonzero(np.any(y != 0, axis=1)).tolist()
        rec['impulse_frames'] = support
        rec['impulse_matches_au_latency'] = support == [17 + delay]
        for p in probes:
            if p['ch'] != row['ch']:
                continue
            macfile = a.root / 'mac' / row['id'] / (p['name'] + '.f32')
            if not macfile.exists():
                continue
            inp = a.root / p['file']
            r = render(a.renderer, inp, out)
            if r.returncode:
                rec['errors'].append({'case': p['name'], 'error': r.stderr[-200:]})
                continue
            new = np.fromfile(out, '<f4').reshape(-1, row['ch'])
            mac = np.fromfile(macfile, '<f4').reshape(-1, row['ch'])
            if rec['cached_onset_aligned'] is None:
                native = np.flatnonzero(np.any(mac != 0, axis=1))
                current = np.flatnonzero(np.any(new != 0, axis=1))
                rec['cached_onset_case'] = p['name']
                rec['au_onset_frame'] = int(native[0]) if native.size else None
                rec['v2_onset_frame'] = int(current[0]) if current.size else None
                rec['cached_onset_aligned'] = bool(native.size and current.size and native[0] == current[0])
            # A broadband onset has a unique sample position. Correlation is
            # diagnostic only and never shifts either side of the comparison.
            if p['name'].startswith('pink'):
                n = len(mac) + len(new)
                fftn = 1 << (n - 1).bit_length()
                c = np.fft.irfft(np.fft.rfft(new.sum(axis=1), fftn) *
                                 np.conj(np.fft.rfft(mac.sum(axis=1), fftn)), fftn)
                lag = int(np.argmax(c))
                if lag > fftn // 2:
                    lag -= fftn
                if lag != 0:
                    rec['timing_misses'].append({'case': p['name'], 'lag': lag})
            if a.baseline:
                r = render(a.baseline, inp, old)
                if r.returncode:
                    rec['errors'].append({'case': p['name'], 'baseline_error': r.stderr[-200:]})
                    continue
                base = np.fromfile(old, '<f4').reshape(-1, row['ch'])
                full = len(new) // 512 * 512
                rec['complete_call_mismatches'] += int(new[:full].tobytes() != base[:full].tobytes())
                rec['short_final_call_differences'] += int(new[full:].tobytes() != base[full:].tobytes())
            rec['cases'] += 1
        rec['status'] = 'PASS' if (rec['impulse_matches_au_latency'] and rec['cached_onset_aligned'] and not rec['timing_misses']
                                         and not rec['complete_call_mismatches'] and not rec['errors']) else 'FAIL'
        if rec['status'] == 'PASS' and rec['cases'] != rec['planned']:
            rec['status'] = 'incomplete'
        records.append(rec)
        print(rec['id'], rec['status'], rec['cases'], 'latency', delay,
              'bit_misses', rec['complete_call_mismatches'], flush=True)
    result = {'renderer_sha256': hashlib.sha256(a.renderer.read_bytes()).hexdigest(),
              'baseline_sha256': hashlib.sha256(a.baseline.read_bytes()).hexdigest() if a.baseline else None,
              'latency_compensation_frames': 0, 'rows': records}
    (a.work / 'timing.json').write_text(json.dumps(result, indent=1))
    return int(any(r['status'] == 'FAIL' for r in records))


if __name__ == '__main__':
    raise SystemExit(main())
