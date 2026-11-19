#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Norm rig: the Mac's own loudness normalizer (aufx/ldnm) against v2's box.

  rig.py ref OUT          on a Mac: every case through the installed unit
                          (norm_host, offline), metrics to OUT/<aid>_<case>.npz
  rig.py cmp OUT [AIDS]   anywhere: every case through v2's box
                          (aurora-speaker-tuning example norm-render),
                          compared with OUT's references; gates applied

Cases (deterministic, regenerated on both sides): steady sines at the
1/3-octave centres 20 Hz-20 kHz, -60..0 dBFS, left/right/both (full grid on
the three reference Macs' tunings, a thinned grid on the other spp3
tunings); level steps, tone bursts, a random level staircase, pink noise,
music items, and the system-volume parameter at several volumes.

Metrics per case: the applied gain after every 512-frame call (the unit's
parameter 100 / the box's gain), per-100 ms RMS of each output channel,
and the RMS of the last 0.5 s. Gates: steady cases within 0.1 dB at the
end; every other case within 0.2 dB at every call and every 100 ms window;
never louder than macOS by more than 0.05 dB anywhere.

Paths (environment): NORM_TUNINGS (tunings root, default the host's),
NORM_HOST (norm_host binary), NORM_RENDER (norm-render binary),
NORM_MUSIC (a stereo float WAV, required for the music cases),
NORM_SOUNDS (optional second program, raw stereo f32).

Everything `ref` writes is Apple-derived: keep OUT private, never commit it.
"""
import json
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

import numpy as np

FS = 48000
TUNINGS = os.environ.get('NORM_TUNINGS', '/System/Library/Audio/Tunings')
SPP3 = (8, 9, 13, 14, 15, 20, 21, 24, 27, 28, 29, 30, 34, 35, 38, 39, 40, 41, 43)
MAIN = (13, 9, 35)
FREQS = [20 * 2 ** (k / 3) for k in range(31)]


def sine(f, db, sec):
    n = int(sec * FS)
    return 10 ** (db / 20) * np.sin(2 * np.pi * f * np.arange(n) / FS)


def seg(levels, f=1000.0):
    n = sum(int(s * FS) for _, s in levels)
    amp = np.empty(n)
    i = 0
    for db, s in levels:
        k = int(s * FS)
        amp[i:i + k] = 10 ** (db / 20)
        i += k
    return amp * np.sin(2 * np.pi * f * np.arange(n) / FS)


def pink(sec, seed, rms_db):
    rng = np.random.default_rng(seed)
    n = int(sec * FS)
    w = np.fft.rfft(rng.standard_normal((n, 2)), axis=0)
    f = np.fft.rfftfreq(n, 1 / FS)
    f[0] = f[1]
    p = np.fft.irfft(w / np.sqrt(f)[:, None], n, axis=0)
    return p / np.sqrt(np.mean(p ** 2)) * 10 ** (rms_db / 20)


def program(path, gain_db):
    raw = open(path, 'rb').read()
    if raw[:4] == b'RIFF':
        i = raw.find(b'data')
        n = int.from_bytes(raw[i + 4:i + 8], 'little')
        x = np.frombuffer(raw[i + 8:i + 8 + n], np.float32)
    else:
        x = np.frombuffer(raw, np.float32)
    return x.reshape(-1, 2) * 10 ** (gain_db / 20)


def stereo(x, mode):
    if x.ndim == 2:
        return x
    z = 0 * x
    return {'L': np.stack([x, z], 1), 'R': np.stack([z, x], 1), 'LR': np.stack([x, x], 1)}[mode]


def staircase(seed):
    rng = np.random.default_rng(seed)
    out, t = [], 0.0
    while t < 40:
        d = float(rng.choice([0.02, 0.05, 0.1, 0.25, 0.5, 1, 2, 4]))
        out.append((float(rng.uniform(-50, 0)), d))
        t += d
    return out


def cases(which='all'):
    out = []
    music = os.environ.get('NORM_MUSIC')
    sounds = os.environ.get('NORM_SOUNDS')
    if which in ('all', 'steady'):
        for aid in SPP3:
            freqs = FREQS if aid in MAIN else FREQS[::3]
            levels = range(-60, 1, 6) if aid in MAIN else range(-60, 1, 12)
            modes = ('L', 'R', 'LR') if aid in MAIN else ('L', 'LR')
            for f in freqs:
                for db in levels:
                    for m in modes:
                        out.append((f'steady_{f:.0f}_{db}_{m}', aid,
                                    lambda f=f, db=db, m=m: stereo(sine(f, db, 8), m), (), 'steady'))
    if which in ('all', 'dyn'):
        for aid in SPP3:
            dyn = [
                ('steps', lambda: stereo(seg([(-30, 8), (-10, 6), (-30, 8), (-20, 4), (-5, 3), (-15, 5), (-45, 3), (0, 2)]), 'L')),
                ('stepsLR', lambda: stereo(seg([(-30, 8), (-10, 6), (-30, 8), (-20, 4), (-5, 3), (-15, 5)]), 'LR')),
                ('bursts', lambda: stereo(seg([(-30, 8), (-18, 0.01), (-30, 2), (-10, 0.05), (-30, 2), (-24, 0.1), (-30, 3),
                                               (-12, 0.5), (-30, 4), (-6, 1.0), (-30, 6), (-80, 1.0), (-30, 4)]), 'L')),
                ('random', lambda: stereo(seg(staircase(7)), 'LR')),
                ('pink-30', lambda: pink(20, 1, -30)),
                ('pink-20', lambda: pink(20, 2, -20)),
                ('pink-10', lambda: pink(20, 3, -10)),
            ]
            if music:
                dyn += [('music0', lambda: program(music, 0)), ('music-12', lambda: program(music, -12))]
            if sounds:
                dyn += [('sounds', lambda: program(sounds, 0)), ('sounds-20', lambda: program(sounds, -20))]
            if aid not in MAIN:
                dyn = [d for d in dyn if d[0] in ('steps', 'bursts', 'pink-20', 'music0')]
            for name, fn in dyn:
                out.append((name, aid, fn, (), 'dyn'))
    if which in ('all', 'vol'):
        for aid in MAIN:
            for v in (-6, -20, -40):
                if music:
                    out.append((f'vol{v}_music0', aid, lambda: program(music, 0), ((1, v),), 'vol'))
                out.append((f'vol{v}_steady1k-12', aid, lambda: stereo(sine(1000, -12, 8), 'LR'), ((1, v),), 'vol'))
    return out


def metrics(y):
    n = len(y) // 4800 * 4800
    r = y[:n].reshape(-1, 4800, 2).astype(np.float64)
    rms = 10 * np.log10(np.mean(r ** 2, axis=1) + 1e-30)
    tail = 10 * np.log10(np.mean(y[-24000:].astype(np.float64) ** 2, axis=0) + 1e-30)
    return rms, tail


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(f'{cmd[0]}: {r.stderr.strip()[-300:]}')


def ref(out, which):
    host = os.environ['NORM_HOST']
    os.makedirs(out, exist_ok=True)

    def one(c):
        name, aid, fn, sets, _ = c
        dst = f'{out}/{aid}_{name}.npz'
        if os.path.exists(dst):
            return
        x = fn().astype(np.float32)
        with tempfile.TemporaryDirectory() as d:
            x.tofile(f'{d}/in')
            strip = f'{TUNINGS}/AID{aid}/DSP/Strips/builtin_speaker_out_general.austrip'
            cmd = [host, '--strip', strip, '--stage', 'Norm', '--channels', '2', '--in', f'{d}/in',
                   '--out', f'{d}/out', '--meta', f'{d}/meta', '--trace', f'{d}/tr', '--q', '512']
            for k, v in sets:
                cmd += ['--set', f'{k}={v}']
            run(cmd)
            y = np.fromfile(f'{d}/out', np.float32).reshape(-1, 2)
            tr = np.fromfile(f'{d}/tr', np.float32).reshape(-1, 2)[:, 0]
        rms, tail = metrics(y)
        np.savez_compressed(dst, trace=tr, rms=rms, tail=tail)

    # at most two renders at once: the reference host is a shared Mac
    with ThreadPoolExecutor(2) as ex:
        list(ex.map(one, cases(which)))


def gate(r):
    if r['kind'] == 'steady':
        return r['tail_abs'] <= 0.1 and r['trace_loud'] <= 0.05
    return r['trace_abs'] <= 0.2 and r['rms_abs'] <= 0.2 and max(r['trace_loud'], r['rms_loud']) <= 0.05


def cmp(out, which, aids):
    exe = os.environ['NORM_RENDER']

    def one(c):
        name, aid, fn, sets, kind = c
        src = f'{out}/{aid}_{name}.npz'
        if not os.path.exists(src) or (aids and aid not in aids):
            return None
        refd = np.load(src)
        x = fn().astype(np.float32)
        with tempfile.TemporaryDirectory() as d:
            x.tofile(f'{d}/in')
            cmd = [exe, '--root', TUNINGS, '--aid', str(aid), f'{d}/in', f'{d}/out', '--trace', f'{d}/tr']
            for k, v in sets:
                cmd += ['--set', f'{k}={v}']
            run(cmd)
            y = np.fromfile(f'{d}/out', np.float32).reshape(-1, 2)
            tr = np.fromfile(f'{d}/tr', np.float32)
        n = min(len(tr), len(refd['trace']))
        dt = tr[:n] - refd['trace'][:n]
        rms, tail = metrics(y)
        live = refd['rms'] > -90
        dr = (rms - refd['rms'])[live] if live.any() else np.zeros(1)
        tl = refd['tail'] > -90
        dtail = (tail - refd['tail'])[tl] if tl.any() else np.zeros(1)
        return dict(name=name, aid=aid, kind=kind, trace_abs=float(np.abs(dt).max()), trace_loud=float(dt.max()),
                    at=float(np.argmax(np.abs(dt)) * 512 / FS), rms_abs=float(np.abs(dr).max()),
                    rms_loud=float(dr.max()), tail_abs=float(np.abs(dtail).max()))

    with ThreadPoolExecutor(os.cpu_count() or 4) as ex:
        res = [r for r in ex.map(one, cases(which)) if r]
    json.dump(res, open(f'{out}/cmp_{which}.json', 'w'))
    by = {}
    for r in res:
        by.setdefault((r['aid'], r['kind']), []).append(r)
    ok = True
    for (aid, kind), rs in sorted(by.items()):
        bad = [r for r in rs if not gate(r)]
        ok &= not bad
        worst = max(rs, key=lambda r: r['tail_abs'] if kind == 'steady' else r['trace_abs'])
        print(f"AID{aid:<3} {kind:6} n={len(rs):4} fail={len(bad):4} tail={max(r['tail_abs'] for r in rs):.3f} "
              f"gain={max(r['trace_abs'] for r in rs):.3f} rms={max(r['rms_abs'] for r in rs):.3f} "
              f"louder={max(max(r['trace_loud'], r['rms_loud']) for r in rs):+.3f} worst={worst['name']}@{worst['at']:.2f}s")
    return ok


if __name__ == '__main__':
    if len(sys.argv) < 3 or sys.argv[1] not in ('ref', 'cmp'):
        sys.exit(__doc__)
    which = os.environ.get('NORM_CASES', 'all')
    if sys.argv[1] == 'ref':
        ref(sys.argv[2], which)
    else:
        aids = {int(a) for a in sys.argv[3].split(',')} if len(sys.argv) > 3 else set()
        sys.exit(0 if cmp(sys.argv[2], which, aids) else 1)
