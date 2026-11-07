#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Synthetic linked-band gain probes: capture with the offline AU host, compare on a build host."""
import argparse
import array
import itertools
import json
import math
from pathlib import Path
import plistlib
import subprocess


def controls(band, algorithm, quality, overshoot):
    crossovers = ((1000, 6000, 16000), (100, 6000, 16000),
                  (100, 600, 6000), (100, 300, 1000))[band]
    p = {0: 0, 1: 0, 2: 0, 3: 0, 62: 1, 63: 0, 64: quality,
         69: overshoot, 70: algorithm}
    for i, frequency in enumerate(crossovers):
        p[4 + 2 * i], p[5 + 2 * i] = frequency, 4
    for b in range(4):
        o = 10 + 13 * b
        p.update({o: 1, o + 1: 0, o + 2: int(b == band), o + 3: 0,
                  o + 4: 0, o + 5: 1, o + 6: 0, o + 7: -100,
                  o + 8: 1, o + 9: 0, o + 10: -100, o + 11: 1,
                  o + 12: 0, 65 + b: 0, 71 + 2 * b: 0, 72 + 2 * b: 0})
    return p


def words(p, solo):
    w = [p[4], p[6], p[8], p[5], p[7], p[9], p[1], p[2], p[3],
         p[62] / 1000, p[63], p[64], p[69], p[70], 0, -100, 0]
    for b in range(4):
        o = 10 + 13 * b
        w += [0 if b == solo else 2, p[o + 4], p[o + 5], p[o + 6],
              p[o + 7], p[o + 8], p[o + 9], p[o + 10], p[o + 11],
              p[o + 12], p[65 + b], 0, 0]
    return w + [0] * 132  # Four empty threshold-automation curves.


def capture(args):
    args.out.mkdir(parents=True, exist_ok=True)
    stub = args.out / 'synthetic.austrip'
    with stub.open('wb') as f:
        plistlib.dump({'strips': [{'effects': [{'displayname': 'Comp'}]}]}, f)
    for sr, b, a, q, ov in itertools.product(args.rates, args.bands, args.algorithms,
                                            args.qualities, args.overshoot):
        name = f's{sr}-b{b}-a{a}-q{q}-o{ov}'
        frequency = 375 if b == 0 else 3000
        inp = args.out / f'input-{sr}-b{b}.f32'
        if not inp.exists():
            x = array.array('f')
            for i in range(int(.4 * sr)):
                t = i / sr
                loud = .1 <= t < .16 or i == int(.22 * sr) or .28 <= t < .281
                amplitude = 10 ** ((-6 if loud else -80) / 20)
                x.extend((amplitude * math.sin(2 * math.pi * frequency * t + .37),
                          10 ** (-70 / 20) * math.sin(2 * math.pi * frequency * t + .73)))
            with inp.open('wb') as f:
                x.tofile(f)
        neutral = controls(b, a, q, ov)
        compressed = dict(neutral)
        o = 10 + 13 * b
        compressed.update({o + 4: -36, o + 5: 10, o + 6: -40})
        case = dict(name=name, rate=sr, band=b, frequency=frequency, algorithm=a,
                    quality=q, overshoot=ov, input=inp.name)
        for label, p in [('neutral', neutral), ('compressed', compressed)]:
            output = args.out / f'{name}-{label}-au.f32'
            parameter_file = args.out / f'{name}-{label}-words.json'
            parameter_file.write_text(json.dumps(words(p, b)))
            command = [str(args.box_host), '--strip', str(stub), '--stage', 'Comp',
                       '--subtype', 'mstk', '--factory', 'AUMeisterStueckFactory',
                       '--channels', '2', '--no-preset', '--sr', str(sr), '--q', '512',
                       '--in', str(inp), '--out', str(output), '--meta', str(output) + '.json']
            for key, value in sorted(p.items()):
                command += ['--set', f'{key}={value}']
            subprocess.run(command, check=True, capture_output=True, timeout=120)
            inventory = json.loads(Path(str(output) + '.json').read_text())['parameters']
            by_id = {item['id']: item for item in inventory}
            for key, value in p.items():
                info = by_id[key]
                if not info['min'] <= value <= info['max'] or info['value'] != value:
                    raise RuntimeError(f"parameter {key}: requested {value}, AU reports {info}")
        (args.out / f'{name}.json').write_text(json.dumps(case, indent=2) + '\n')
        print(name, flush=True)


def compare(args):
    import numpy as np
    args.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in sorted(args.captures.glob('s*-b*-a*-q*-o*.json')):
        case = json.loads(path.read_text())
        if 'input' not in case:
            continue
        name, sr = case['name'], case['rate']
        samples = {}
        for label in ('neutral', 'compressed'):
            output = args.out / f'{name}-{label}-model.f32'
            subprocess.run([str(args.renderer), '--params-words',
                            str(args.captures / f'{name}-{label}-words.json'),
                            '--fs', str(sr), '--ch', '2', str(args.captures / case['input']),
                            str(output)], check=True, capture_output=True)
            for engine, file in [('au', args.captures / f'{name}-{label}-au.f32'),
                                 ('model', output)]:
                samples[engine, label] = np.fromfile(file, '<f4').reshape(-1, 2)[:, 1].astype(float)
        row = dict(name=name, windows={})
        for event, time in [('attack', .1), ('release', .16), ('bump', .22), ('hold', .28)]:
            for ms in (2, 5, 10, 20, 50):
                sl = slice(int(time * sr), int(time * sr) + int(ms * sr / 1000))
                energy = {key: float(np.sum(value[sl] ** 2)) for key, value in samples.items()}
                gain_error = 10 * math.log10((energy['model', 'compressed'] / energy['model', 'neutral']) /
                                            (energy['au', 'compressed'] / energy['au', 'neutral']))
                output_error = 10 * math.log10(energy['model', 'compressed'] / energy['au', 'compressed'])
                row['windows'][f'{event}-{ms}ms'] = dict(gain_error_db=gain_error, output_error_db=output_error)
        # Effective complex output gain, not an assertion about internal gain nodes.
        width = max(1, round(sr / case['frequency']))
        oscillator = np.exp(-2j * np.pi * case['frequency'] * np.arange(len(samples['au', 'neutral'])) / sr)
        gains = {}
        for engine in ('au', 'model'):
            demod = [np.convolve(samples[engine, label] * oscillator,
                                 np.ones(width) / width, 'same') for label in ('neutral', 'compressed')]
            valid = np.abs(demod[0]) > 1e-12
            gains[engine] = np.divide(demod[1], demod[0], out=np.zeros_like(demod[0]), where=valid)
            gains[engine + '_valid'] = valid
        np.savez(args.out / f'{name}-gain.npz', **gains)
        rows.append(row)
    if not rows:
        raise SystemExit('no capture cases found')
    (args.out / 'summary.json').write_text(json.dumps(rows, indent=2) + '\n')
    print(f'Compared {len(rows)} cases; results in {args.out / "summary.json"}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    cap = sub.add_parser('capture', help='macOS only: public AU API, no audio device')
    cap.add_argument('--box-host', type=Path, required=True)
    cap.add_argument('--out', type=Path, required=True)
    cap.add_argument('--rates', nargs='+', type=int, choices=(44100, 48000, 96000), default=[44100, 48000, 96000])
    cap.add_argument('--bands', nargs='+', type=int, choices=range(4), default=list(range(4)))
    cap.add_argument('--algorithms', nargs='+', type=int, choices=range(4), default=[0, 2])
    cap.add_argument('--qualities', nargs='+', type=int, choices=range(3), default=[0, 1, 2])
    cap.add_argument('--overshoot', nargs='+', type=int, choices=(0, 1), default=[0, 1])
    cmp = sub.add_parser('compare', help='build host: render v2 and measure effective gain')
    cmp.add_argument('--captures', type=Path, required=True)
    cmp.add_argument('--renderer', type=Path, required=True)
    cmp.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    # Child tools may change their working directory; pass stable paths.
    for name in ('out', 'box_host', 'captures', 'renderer'):
        if hasattr(args, name):
            setattr(args, name, getattr(args, name).resolve())
    (capture if args.action == 'capture' else compare)(args)


if __name__ == '__main__':
    main()
