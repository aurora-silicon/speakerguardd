#!/usr/bin/env python3
"""spp3 full-chain safety scan (lane BVB; orchestrator 2026-11-06 ~22:10).

For one Mac's bundle, every program is rendered through
  v2:    the full voicing chain (voicing-render, raw program -> driver lanes)
         -> the spp3 protection unit (sgv2-render, the bundle's own blob, open loop)
  macOS: the Mac's own chain (preprotect_host) -> the Mac's own SpeakerProtect
         (rig_host, open loop), rendered on the host by mac_refs.py,
and the protected driver outputs are compared per driver on six axes, each must be
<= macOS:
  velocity      peak cone velocity (Thiele-Small model of each driver on the output
                voltage), ratio <= 1.001
  displacement  peak cone displacement (same model), ratio <= 1.001
  power         peak 1 s mean drive power V^2/Re, ratio <= 1.001
  coil temp     peak of the unit's own open-loop coil temperature meter, <= +0.01 C
  steady        per stepped-sine segment, steady amplitude at the tone, v2 - macOS <= +0.05 dB
  transient     per stepped-sine segment, peak 50 ms window at the tone, <= +0.05 dB
(the last two: the steady/transient margin of AUD's verify_dyn, here on the whole chain).
Peaks are time-free; the per-segment levels are read after aligning each side by the
cross-correlation lag of its output (the t-4967 lesson: never compare unaligned
waveforms).

Runs on the arm64 farm (static aarch64 binaries). Usage:
  scan.py --mac MACDIR --progs DIR --bundle B.tar.gz --bins DIR --ts ts.json --name MAC --out DIR
Everything it writes is Apple-derived (macOS renders, tuning): OUT must be private.
"""
import argparse, json, math, os, subprocess, tarfile, sys
import numpy as np

SR = 48000
# output lane -> Thiele-Small channel key (AUD verify_dyn's mapping; lanes in spp3 part order)
TS_KEY = {'m2': ['Channel0', 'Channel1', 'Channel2', 'Channel3'],
          'm1pro': ['Channel0', 'Channel2', 'Channel1', 'Channel3', 'Channel4', 'Channel5'],
          'j616s': ['Channel0', 'Channel2', 'Channel1', 'Channel3', 'Channel4', 'Channel5']}
LIM = {'velocity': 1.001, 'displacement': 1.001, 'power': 1.001, 'coil_temp': 0.01,
       'steady': 0.05, 'transient': 0.05, 'tweeter_lf': 0.05}
# tweeter low-frequency gate (orchestrator 11-06 ~22:30, HARD): per tweeter, 1/3-octave band level
# 20-200 Hz of the protected output, v2 minus max(macOS, TW_LF_FLOOR_DBFS). The floor keeps
# float-noise ratios from counting; anything above it must not exceed macOS.
# Floor -90 dBFS (orchestrator decision 11-07, was -120): tweeter content below -90 dBFS is
# far below audibility and has no excursion or thermal relevance.
TW_LF_BANDS = [20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200]
TW_LF_FLOOR_DBFS = -90.0


def lf_bands(y):
    X = np.abs(np.fft.rfft(y)) ** 2 * 2 / (len(y) ** 2 * 0.5)  # band power re a full-scale sine
    f = np.fft.rfftfreq(len(y), 1 / SR)
    return np.array([10 * np.log10(X[(f >= c / 2 ** (1 / 6)) & (f < c * 2 ** (1 / 6))].sum() + 1e-30)
                     for c in TW_LF_BANDS])


def run(cmd, **kw):
    p = subprocess.run(cmd, capture_output=True, text=True, **kw)
    if p.returncode:
        raise SystemExit(f'failed: {cmd[0]} rc={p.returncode}: {p.stderr[-600:]}')
    return p


def bilinear_2nd(b, a):
    """s-domain (b2 s^2 + b1 s + b0)/(a2 s^2 + a1 s + a0) -> z-domain, bilinear at SR."""
    K = 2 * SR
    B = [b[0] * K * K + b[1] * K + b[2], 2 * (b[2] - b[0] * K * K), b[0] * K * K - b[1] * K + b[2]]
    A = [a[0] * K * K + a[1] * K + a[2], 2 * (a[2] - a[0] * K * K), a[0] * K * K - a[1] * K + a[2]]
    return np.array(B) / A[0], np.array(A) / A[0]


def lfilter(b, a, x):
    y = np.zeros_like(x); z1 = z2 = 0.0
    # direct form II transposed, vectorised in chunks would be faster; scipy if present
    try:
        from scipy.signal import lfilter as lf
        return lf(b, a, x)
    except ImportError:
        for i, v in enumerate(x):
            o = b[0] * v + z1; z1 = b[1] * v - a[1] * o + z2; z2 = b[2] * v - a[2] * o; y[i] = o
        return y


def mech(y, p):
    """Cone displacement (mm) and velocity (m/s) of one driver from its output samples."""
    Q = max(p['Q_d'], math.sqrt(p['kt'] * p['mt']) / (p['ct'] + p['phi'] ** 2 / p['Re']))
    c = math.sqrt(p['kt'] * p['mt']) / Q
    g = p['ag'] * p['phi'] / p['Re']            # force per output unit (N per FS)
    bx, ax = bilinear_2nd([0, 0, g], [p['mt'], c, p['kt']])      # x (m)
    bv, av = bilinear_2nd([0, g, 0], [p['mt'], c, p['kt']])      # v (m/s)
    return lfilter(bx, ax, y) * 1e3, lfilter(bv, av, y)


def power_1s(y, p):
    v = y * p['ag']; w = v * v / p['Re']
    c = np.cumsum(np.r_[0.0, w]); n = SR
    return float(np.max((c[n:] - c[:-n]) / n)) if len(w) > n else float(np.mean(w))


def comp(y, f):
    n = np.arange(len(y)); return 2 * abs(np.sum(y * np.exp(-2j * np.pi * f * n / SR))) / len(y)


def lag(a, b, maxlag=4000):
    """Lag (frames) of b behind a, by cross-correlation on the strongest lane."""
    k = int(np.argmax(np.sum(a * a, 0)))
    x, y = a[:SR * 20, k], b[:SR * 20, k]
    n = 1 << int(math.ceil(math.log2(len(x) + len(y))))
    r = np.fft.irfft(np.fft.rfft(y, n) * np.conj(np.fft.rfft(x, n)), n)
    r = np.r_[r[-maxlag:], r[:maxlag + 1]]
    return int(np.argmax(r)) - maxlag


def mac_temps(d, nd):
    import csv
    rows = list(csv.DictReader(open(os.path.join(d, 'run.state.csv'))))
    return [max(float(r[f's4.e{e}.p8:OL_temperature']) for r in rows) for e in range(nd)]


def v2_temps(trace, nd):
    t = [-1e9] * nd
    for l in open(trace):
        r = json.loads(l)
        for e, c in enumerate(r.get('channels', [])[:nd]):
            if c.get('ol_temp_c') is not None:
                t[e] = max(t[e], c['ol_temp_c'])
    return t


def axes(y, ts_keys, ts, temps):
    out = []
    for e, k in enumerate(ts_keys):
        p = ts[k]; x, v = mech(y[:, e], p)
        out.append({'velocity': float(np.max(np.abs(v))), 'displacement': float(np.max(np.abs(x))),
                    'power': power_1s(y[:, e], p), 'coil_temp': temps[e], 'name': p.get('name', k)})
    return out


def steps_levels(y, meta, sh):
    st, tr = [], []
    for s in meta['segments']:
        a = s['start'] + sh; n = s['len']
        seg = y[a:a + n]
        st.append([20 * math.log10(comp(seg[n // 2:, e], s['f']) + 1e-12) for e in range(y.shape[1])])
        w = max(int(0.05 * SR), int(4 * SR / s['f']))
        tr.append([max(20 * math.log10(comp(seg[i:i + w, e], s['f']) + 1e-12)
                       for i in range(0, max(1, n // 2 - w), w // 2)) for e in range(y.shape[1])])
    return np.array(st), np.array(tr)


def main():
    ap = argparse.ArgumentParser()
    for k in ('mac', 'progs', 'bundle', 'bins', 'ts', 'name', 'out'):
        ap.add_argument('--' + k, required=True)
    A = ap.parse_args()
    os.makedirs(A.out, exist_ok=True)
    ts = json.load(open(A.ts))[A.name]; keys = TS_KEY[A.name]; nd = len(keys)
    man = json.loads(tarfile.open(A.bundle).extractfile('audio-tuning/manifest.json').read())
    blob = os.path.join(A.out, 'unit.blob')
    # Same unit configuration on both sides: the macOS reference (mac_refs.py) runs
    # SpeakerProtect without the unit's speaker calibration (no 64009), so the v2 blob is
    # compiled without the unit binding too. A calibrated v2 unit adds its sense pilot
    # (a DC/sub-20 Hz term) on every lane, which the driver model reads as static cone
    # displacement; comparing that against an uncalibrated macOS run is a false alarm
    # (lane BVB, 2026-11-06 M2 tweeter finding, withdrawn).
    run([os.path.join(A.bins, 'aurora-speaker-tuning'), 'compile', '--board', man['board'],
         '--bundle', A.bundle, '-o', blob])
    info = os.path.join(A.out, 'chain-info.json')
    rows, informational = [], False
    for prog in sorted(f[:-4] for f in os.listdir(A.progs) if f.endswith('.f32')):
        md = os.path.join(A.mac, prog)
        if not os.path.exists(os.path.join(md, 'run.out.f32')):
            continue
        w = os.path.join(A.out, prog); os.makedirs(w, exist_ok=True)
        feeds = os.path.join(w, 'feeds.f32')
        run([os.path.join(A.bins, 'voicing-render'), '--bundle', A.bundle, '--in',
             os.path.join(A.progs, prog + '.f32'), '--out', feeds, '--info', info])
        run([os.path.join(A.bins, 'sgv2-render'), '--unit', 'spp3', '--blob', blob, '--case', f'{A.name}-{prog}',
             '--in', feeds, '--ch', str(nd), '--file-ch', str(nd), '--slices', '512',
             '--out', os.path.join(w, 'trace.jsonl'), '--audio-out', os.path.join(w, 'out.f32')])
        yv = np.fromfile(os.path.join(w, 'out.f32'), '<f4').reshape(-1, nd).astype(float)
        ym = np.fromfile(os.path.join(md, 'run.out.f32'), '<f4').reshape(-1, nd).astype(float)
        n = min(len(yv), len(ym)); yv, ym = yv[:n], ym[:n]
        sh = lag(ym, yv)
        av = axes(yv, keys, ts, v2_temps(os.path.join(w, 'trace.jsonl'), nd))
        am = axes(ym, keys, ts, mac_temps(md, nd))
        r = {'program': prog, 'lag_frames': sh, 'drivers': []}
        for e in range(nd):
            d = {'driver': am[e]['name']}
            for ax in ('velocity', 'displacement', 'power'):
                d[ax] = {'v2': av[e][ax], 'macos': am[e][ax], 'ratio': av[e][ax] / max(am[e][ax], 1e-30)}
            d['coil_temp'] = {'v2': av[e]['coil_temp'], 'macos': am[e]['coil_temp'],
                              'delta': av[e]['coil_temp'] - am[e]['coil_temp']}
            if 'T' in d['driver'] and 'W' not in d['driver']:
                bv, bm = lf_bands(yv[:, e]), lf_bands(ym[:, e])
                ex = bv - np.maximum(bm, TW_LF_FLOOR_DBFS)
                k = int(np.argmax(ex))
                d['tweeter_lf'] = float(ex[k]); d['tweeter_lf_at'] = TW_LF_BANDS[k]
                d['tweeter_lf_levels'] = {'v2': float(bv[k]), 'macos': float(bm[k])}
            r['drivers'].append(d)
        if prog == 'steps':
            meta = json.load(open(os.path.join(A.progs, 'steps.json')))
            sv, tv = steps_levels(yv, meta, max(sh, 0)); sm, tm = steps_levels(ym, meta, 0)
            ok = sm > -100
            for e in range(nd):
                r['drivers'][e]['steady'] = float(np.max((sv - sm)[:, e][ok[:, e]]))
                r['drivers'][e]['transient'] = float(np.max((tv - tm)[:, e][ok[:, e]]))
        rows.append(r)
        os.remove(feeds)
    stubs = json.load(open(info)).get('stubs', []) if os.path.exists(info) else []
    informational = bool(stubs)
    # table
    worst = {}
    for r in rows:
        for e, d in enumerate(r['drivers']):
            for ax in ('velocity', 'displacement', 'power'):
                worst.setdefault((d['driver'], ax), (-1e9, ''))
                if d[ax]['ratio'] > worst[(d['driver'], ax)][0]:
                    worst[(d['driver'], ax)] = (d[ax]['ratio'], r['program'])
            worst.setdefault((d['driver'], 'coil_temp'), (-1e9, ''))
            if d['coil_temp']['delta'] > worst[(d['driver'], 'coil_temp')][0]:
                worst[(d['driver'], 'coil_temp')] = (d['coil_temp']['delta'], r['program'])
            for ax in ('steady', 'transient'):
                if ax in d:
                    worst[(d['driver'], ax)] = (d[ax], r['program'])
            if 'tweeter_lf' in d and d['tweeter_lf'] > worst.get((d['driver'], 'tweeter_lf'), (-1e9, ''))[0]:
                worst[(d['driver'], 'tweeter_lf')] = (d['tweeter_lf'], f"{r['program']} @{d['tweeter_lf_at']} Hz")
    lines = [f'# spp3 full-chain safety scan: {A.name}', '',
             ('INFORMATIONAL ONLY: the chain still stubs ' + ', '.join(f'{a} ({b})' for a, b in stubs)
              if informational else 'Chain complete (no stubs).'), '',
             '| driver | axis | worst v2 vs macOS | limit | program | pass |', '|---|---|---|---|---|---|']
    allpass = True
    for (drv, ax), (v, prog) in sorted(worst.items()):
        ok = v <= LIM[ax]
        allpass &= ok
        unit = {'velocity': 'x', 'displacement': 'x', 'power': 'x', 'coil_temp': ' C', 'steady': ' dB', 'transient': ' dB', 'tweeter_lf': ' dB'}[ax]
        lines.append(f'| {drv} | {ax} | {v:.4f}{unit} | {LIM[ax]}{unit} | {prog} | {"PASS" if ok else "FAIL"} |')
    lines += ['', f'RESULT {A.name}: {"PASS" if allpass else "FAIL"}' + (' (informational: stubbed chain)' if informational else '')]
    open(os.path.join(A.out, 'report.md'), 'w').write('\n'.join(lines) + '\n')
    json.dump({'rows': rows, 'stubs': stubs, 'pass': allpass, 'informational': informational},
              open(os.path.join(A.out, 'scan.json'), 'w'), indent=1)
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
