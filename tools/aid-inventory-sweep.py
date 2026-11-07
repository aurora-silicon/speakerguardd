#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""All speaker configurations, two compiler modes, two feature builds. Buildbox only.

Uses the inventory's temporary observation counters; never changes compiler laws.
Raw inputs/results and forced-selection fixtures are private. The report exports
only names, reason codes and counts. No per-board selection or tuning constants.
"""
import argparse
from collections import Counter, defaultdict
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import plistlib
import re
import shutil
import subprocess

SPEC = importlib.util.spec_from_file_location('inventory', Path(__file__).with_name('aid-inventory.py'))
inv = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(inv)
FEATURES = ('default', 'conservative')
MODES = ('default', 'voicing-pre')


def dump(path, data):
    path.write_text(json.dumps(data, sort_keys=True, indent=2) + '\n')


def configurations(root, aid):
    """Deduplicate identical hardware routes, not equal tuning bytes.

    Keep every distinct Output mapping, substitution, rate and speaker selector.
    All protected configurations are executed. Raw diagnostic routes without
    any processing box are recorded separately, outside the protection compiler.
    """
    aname = f'AID{aid}'
    shared = root / 'DeviceDSPDatabase.plist'
    dbpath = shared if shared.is_file() else root / aname / 'DSP/DSPDatabase.plist'
    db = plistlib.loads(dbpath.read_bytes())[aname]['Built-in Speaker']
    if not isinstance(db, dict) or not db:
        raise ValueError('empty speaker database')
    routes = {}
    excluded = []
    for name, config in sorted(db.items()):
        if not isinstance(config, dict):
            raise ValueError('invalid configuration')
        # Raw diagnostic routes have no processing boxes or protection unit.
        # Keep an explicit coverage record instead of forcing their diagnostic
        # strip onto a protected graph, or counting them as compiler failures.
        graphs = [h['Path'] for o in config.get('Output', [])
                  for h in o.get('Hardware', []) if h.get('Type') == 'DSPGraph']
        if len(graphs) == 1:
            text = (root / graphs[0]).read_text()
            statements = [line.split(';', 1)[0].strip() for line in text.splitlines()]
            if (any(line.startswith('wire ') for line in statements)
                    and not any(line.startswith('box ') for line in statements)
                    and all(not line or line.startswith(('graphName ', '[def ', 'in ', 'out ', 'wire ', 'param ')) for line in statements)):
                excluded.append({'configuration': name, 'graph': graphs[0],
                                 'reason': 'raw-route-without-protection'})
                continue
        # cdes labels choose a mode, but its audio behavior is the complete
        # Output mapping. Underlying selectors can change rate/vendor behavior.
        key = plistlib.dumps({k: config[k] for k in ('Output', 'UnderlyingDeviceProperties') if k in config},
                             sort_keys=True)
        if key not in routes:
            routes[key] = {'config': config, 'names': [], 'strips': set()}
        entry = routes[key]
        entry['names'].append(name)
        for output in config.get('Output', []):
            for h in output.get('Hardware', []):
                if h.get('Type') == 'AUStrip':
                    entry['strips'].add(h['Path'])
    # Include a default-selection control, measured directly from the original
    # root. This independently checks that forcing a route did not hide a bug.
    variants = [{'original': True, 'config': None, 'names': [], 'strips': set()}]
    variants += [routes[key] for key in sorted(routes)]
    referenced = {h['Path'] for c in db.values() for o in c.get('Output', [])
                  for h in o.get('Hardware', []) if h.get('Type') == 'AUStrip'}
    disk = {str(p.relative_to(root)) for p in (root / aname / 'DSP/Strips').glob('*speaker*.austrip')}
    # Some releases retain vendor strips the database no longer selects.
    # Exercise each against every distinct General hardware mapping rather
    # than silently omitting it or inventing a vendor/board selector. These
    # are explicit forced-strip tests, not claimed macOS configurations.
    generals = [v for v in variants[1:] if any('General' in n for n in v['names'])]
    if disk - referenced and not generals:
        raise ValueError('unreferenced speaker strips but no General hardware mapping')
    for strip in sorted(disk - referenced):
        for general in generals:
            config = copy.deepcopy(general['config'])
            replaced = 0
            for output in config.get('Output', []):
                for hardware in output.get('Hardware', []):
                    if hardware.get('Type') == 'AUStrip':
                        hardware['Path'] = strip
                        replaced += 1
            if replaced != 1:
                raise ValueError('General mapping has no unique strip')
            variants.append({'config': config, 'names': [], 'strips': {strip}, 'unreferenced_strip': True})
    return variants, len(db), len(referenced | disk), len(disk), excluded


def fixture(root, aid, variant, dest):
    """Force one database route without changing any graph/strip bytes.

    Private hard links preserve the immutable tuning files. Replace the copied
    database inode, never write through a link into the staged original tree.
    The sole config is named General and its vendor selector is made neutral;
    its complete hardware mapping, rate/substitutions and property strip survive.
    This tests each route's compiler support, not the runtime vendor selector.
    """
    if variant.get('original'):
        return root
    aname = f'AID{aid}'
    shutil.copytree(root / aname, dest / aname, copy_function=os.link)
    config = copy.deepcopy(variant['config'])
    config['UnderlyingDeviceProperties'] = [p for p in config.get('UnderlyingDeviceProperties', [])
                                             if p.get('Selector') != 'spid']
    dbpath = dest / aname / 'DSP/DSPDatabase.plist'
    dbpath.unlink(missing_ok=True)
    dbpath.write_bytes(plistlib.dumps({aname: {'Built-in Speaker': {'General': config}}}))
    dump(dest / 'selection-private.json', {'configurations': variant['names'],
                                          'strips': sorted(variant['strips'])})
    return dest


def why(message):
    # Public-source literal diagnostic phrases only. Unknown text stays private.
    more = [
        ('output saturation mode/flavor/version', 'LAW-cfrk-saturation-flavor'),
        ('the high-pass is on (not carried)', 'LAW-cfrk-highpass'),
        ('filter type 9 is not carried', 'LAW-vrq2-filter-type9'),
        ('active BuzzKill detector/envelope law', 'LAW-buzk-detector-envelope'),
        ('mono VirtBass width law', 'LAW-vben-mono-width'),
        ('side-chain low-pass is set', 'LAW-Norm-sidechain-LP'),
        ('algorithm version 1 is not carried', 'LAW-Norm-algorithm1'),
        ('reset mode 8 is not carried', 'LAW-Norm-reset8'),
        ('reserved parameter (parameter 33)', 'LAW-XTC-parameter33'),
        ('type 3 phase companion is not carried', 'LAW-tmst-type3-phase-companion'),
        ('builtin sum ', 'graph-sum-unimplemented'),
        ('builtin dbcontrol', 'dbcontrol-unimplemented'),
        ('"lsid"', 'lsid-unimplemented'),
        ('type 17', 'tmst-type17-unimplemented'),
        ('type=17', 'tmst-type17-unimplemented'),
        ('on none of the unit', 'graph-lane-unfed'),
        ('lattice', 'lsm2-mask-lattice-missing'),
        ('not a dictionary', 'plist-shape-refused'),
        ('but its preset is for', 'strip-preset-unit-mismatch'),
        ('malformed wireGraphParam', 'dspg-wire-parameter-syntax-refused'),
        ('No such file', 'source-file-missing'),
    ]
    return next((r for phrase, r in more if phrase in message), inv.reason(message))


def execute(cmd, stem):
    rc = inv.run(cmd, stem)
    return rc, stem.with_suffix('.stderr').read_text(errors='replace')


def cached_program(path, channels, cache):
    # The deterministic input depends only on channel count. All consumers
    # open it read-only, so private hard links avoid regenerating it hundreds
    # of times per promotion without changing a single input sample.
    if channels in cache:
        os.link(cache[channels], path)
    else:
        inv.program(path, channels)
        cache[channels] = path


def measure(root, aid, private, binaries, discovery_error=None, programs=None):
    private.mkdir(parents=True)
    if programs is None:
        programs = {}
    public = {}
    for feature in FEATURES:
        bins = binaries[feature]
        source = ['--tunings-root', str(root), '--aid', str(aid)]
        stem = private / (feature + '-probe')
        rc, err = execute([bins['probe'], *source], stem)
        raw = {}
        if rc == 0:
            try:
                raw = json.loads(stem.with_suffix('.stdout').read_text())
            except (ValueError, OSError):
                rc, err = 1, 'invalid probe JSON'
        unit = raw.get('unit')
        boxes = raw.get('boxes', [])
        units = sorted({b['subtype'] for b in boxes if b.get('protection')})
        if isinstance(unit, str):
            units = sorted(set(units + [unit.lower()]))
        channels = raw.get('protection_channels')
        if channels is None:
            channels = next((b.get('channels') for b in boxes if b.get('protection')), None)
        probe_error = err if rc else next((raw[k] for k in ('load_error', 'chain_error', 'runtime_error') if k in raw), None)
        observed = inv.public_probe(raw) if raw.get('boxes') else {}
        stubs = raw.get('stubs', [])
        # Stub box identities are public-safe; stub messages remain private.
        stub_boxes = sorted(b['name'] for b in raw.get('compiled_boxes', []) if b['kind'] == 'stub')
        stub_reasons = [{'box': name, 'unit': unit,
                         'reason': why(next((msg for box, msg in raw.get('gaps', []) if box == name), 'stub'))}
                        for name, unit in stubs]
        bounds = [{'box': b['name'], 'reason': law} for b in observed.get('boxes', [])
                  for law in b.get('exercised_bounds', [])]
        compiles = {}
        for mode in MODES:
            stem = private / (feature + '-' + mode)
            # Development provenance is mandatory. A generic development board
            # only provides the CLI binding; no board-dependent DSP values.
            cmd = [bins['cli'], 'compile', '--development', '--board', 'j473', *source,
                   '-o', str(stem) + '.blob']
            if mode == 'voicing-pre':
                cmd.append('--voicing-pre')
            crc, text = execute(cmd, stem)
            reasons = Counter(why(line) for line in text.splitlines() if line.startswith('error['))
            if crc and not reasons:
                reasons[why(text)] += 1
            if discovery_error:
                reasons[discovery_error] += 1
            summary = inv.diagnostic_summary(text, interim=mode == 'default')
            compiles[mode] = {'rc': crc, 'refused': bool(crc or discovery_error),
                              'reason_counts': dict(sorted(reasons.items())),
                              'todo_spec': sum(summary['todo_spec_diagnostic_counts'].values()),
                              'todo_spec_reason_counts': summary['todo_spec_law_counts']}
        # An independent renderer consistency check for every successfully
        # compiled chain. Failed probes preserve their load/chain reason.
        render = {'rc': None, 'reason': why(probe_error) if probe_error else None}
        if not probe_error:
            inp = private / (feature + '-program.f32')
            cached_program(inp, raw.get('plan', {}).get('program_channels', 2), programs)
            stem = private / (feature + '-render')
            rrc, text = execute([bins['renderer'], *source, '--in', inp, '--out', str(stem) + '.f32',
                                 '--info', str(stem) + '.json'], stem)
            render = {'rc': rrc, 'reason': why(text) if rrc else None}
            if rrc == 0:
                info = json.loads(Path(str(stem) + '.json').read_text())
                assert all(info[k] == raw.get(k) for k in ('plan', 'stubs', 'gaps', 'unwired'))
        public[feature] = {'units': units, 'channels': channels,
                           'boxes': [{'name': b['name'], 'unit': b['subtype'], 'channels': b.get('channels')}
                                     for b in boxes],
                           'probe_reason': why(probe_error) if probe_error else None,
                           'stubs': len(stubs), 'stub_boxes': stub_boxes, 'stub_reasons': stub_reasons,
                           'bounds_exercised': bounds, 'compiles': compiles, 'render': render}
    return public


def sweep(root, aids, private, binaries, programs):
    rows = []
    for aid in aids:
        ap = private / f'AID{aid}'
        ap.mkdir(parents=True)
        try:
            variants, configs, strips, disk_strips, excluded = configurations(root, aid)
        except (OSError, ValueError, KeyError, TypeError, plistlib.InvalidFileException) as exc:
            (ap / 'discovery-error-private.txt').write_text(str(exc))
            variants, configs, strips, disk_strips, excluded = [dict(original=True, discovery_error='speaker-database-parse-refused')], 0, 0, 0, []
        measured = []
        for i, variant in enumerate(variants):
            vp = ap / f'variant-{i}'
            vp.mkdir()
            selected = fixture(root, aid, variant, vp / 'private-fixture')
            features = measure(selected, aid, vp / 'execution', binaries, variant.get('discovery_error'), programs)
            measured.append({'variant': i, 'default_selection': i == 0,
                              'selection_kind': 'default-control' if i == 0 else
                              'unreferenced-strip' if variant.get('unreferenced_strip') else
                              'diagnostic' if variant.get('names') and all('Diagnostic' in n for n in variant['names']) else 'speaker',
                              'configuration_count': len(variant.get('names', [])),
                              'strip_count': len(variant.get('strips', [])), 'features': features})
        rows.append({'aid': aid, 'configuration_count': configs, 'speaker_strips': strips,
                     'disk_speaker_strips': disk_strips, 'excluded_routes': [{'reason': e['reason']} for e in excluded], 'variants': measured})
        print(f'AID{aid}: {len(measured)} selections executed', flush=True)
    return rows


def assemble(rows, revision):
    rollup = defaultdict(set)
    reason_counts = Counter()
    reason_categories = defaultdict(Counter)
    totals = Counter()
    for row in rows:
        counts = Counter()
        refusals = Counter()
        stub_counts, probe_counts, render_counts = Counter(), Counter(), Counter()
        units, channels = set(), set()
        for variant in row['variants']:
            for feature in variant['features'].values():
                units.update(feature['units'])
                if feature['channels'] is not None:
                    channels.add(feature['channels'])
                counts['stubs'] += feature['stubs']
                stub_counts.update(s['reason'] for s in feature['stub_reasons'])
                counts['bounds'] += len(feature['bounds_exercised'])
                for mode in feature['compiles'].values():
                    counts['attempts'] += 1
                    counts['refusals'] += mode['refused']
                    counts['todo_spec'] += mode['todo_spec']
                    refusals.update(mode['reason_counts'])
                # Count auxiliary full-chain failure observations separately
                # from CLI refusals so the same rejection is not counted twice.
                for key in ('probe_reason',):
                    if feature.get(key):
                        counts['probe_refusals'] += 1
                        probe_counts[feature[key]] += 1
                if feature['render']['rc']:
                    counts['render_refusals'] += 1
                    render_counts[feature['render']['reason']] += 1
        for category, counter in [('compile_errors', refusals), ('stubs', stub_counts),
                                  ('probe_refusals', probe_counts), ('render_refusals', render_counts)]:
            for reason, count in counter.items():
                reason_categories[reason][category] += count
        for reason, count in (refusals + stub_counts + probe_counts + render_counts).items():
            rollup[reason].add(row['aid'])
            reason_counts[reason] += count
        row.update(units=sorted(units), channels=sorted(channels), counts=dict(counts),
                   reason_counts=dict(sorted(refusals.items())),
                   stub_reason_counts=dict(sorted(stub_counts.items())),
                   probe_reason_counts=dict(sorted(probe_counts.items())))
        totals.update(counts)
    totals['aids'] = len(rows)
    totals['refused_aids'] = sum(bool(r['counts'].get('refusals')) for r in rows)
    totals['selections'] = sum(len(r['variants']) for r in rows)
    return {'schema_version': 1, 'source_revision': revision, 'tuning_release': '27.2-26B8125',
            'totals': dict(totals), 'aids': rows,
            'per_reason': {k: {'aids': sorted(v), 'count': reason_counts[k],
                              **dict(reason_categories[k])} for k, v in sorted(rollup.items())},
            'definitions': {
                'refusals': 'Refused CLI invocations or explicit discovery-refusal attempts, across selections x 2 builds x 2 modes. rc preserves actual CLI status. Reason counts are emitted errors; one invocation can emit multiple errors.',
                'selections': 'Original default selection plus every distinct protected database Output/UnderlyingDeviceProperties mapping; boxless raw routes are recorded in excluded_routes. Identical mappings are executed once; configuration_count retains multiplicity. Each disk speaker strip absent from the database is additionally tested against every distinct General hardware mapping; these forced-strip cases are not claimed macOS configurations.',
                'stubs': 'Actual full-chain compiler stubs, once per selection/build; no double-count per CLI mode. A failed chain emits no stubs and is a refusal, never proof of stub-free support.',
                'bounds': 'Distinct instrumented (box, reason) hits, once per selection/build during deterministic quiet/loud/release offline rendering.',
                'todo_spec': 'Emitted TODO(SPEC) diagnostics across the four CLI calls per selection. Source-only TODOs are excluded.',
                'fixture': 'Private forced-selection database only: sole config named General, speaker selector neutralized. Original hardware mappings, substitutions, graph/strip/property bytes unchanged. Default control uses the staged tree directly.',
                'metadata': 'If graph parsing fails, the observer may omit parameter/property declarations and links solely to recover box/unit/channel identities. The actual loader/compiler always receives the complete original graph and retains its refusal.',
            }, 'limitations': [
                'Compile support measurement, not macOS parity qualification or hardware audio playback.',
                '48 kHz, one deterministic short program. Protection DSP is compiled by the CLI but not rendered by the voicing-chain probe.',
                'Existing inventory counters cover selected bounds only; zero hits do not imply all protective laws are absent.',
                'Development provenance; generic CLI binding is not a per-device calibration or production provenance test.',
            ]}


def total_line(data):
    t = data['totals']
    return (f"TOTAL {data['source_revision']}: AIDs {t['aids']}, refusals {t.get('refusals', 0)} "
            f"({t['refused_aids']} AIDs), stubs {t.get('stubs', 0)}, bounds {t.get('bounds', 0)}, "
            f"TODO(SPEC) {t.get('todo_spec', 0)}; selections {t['selections']}, attempts {t.get('attempts', 0)}")


def markdown(data):
    lines = ['# AID compile/stub sweep', '', f"Source: `{data['source_revision']}`; 27.2 / 26B8125.", '',
             total_line(data), '', '| AID | Unit | Channels | Selections | Refusals (reason: count) | Stubs | Bounds exercised | TODO(SPEC) |',
             '|---|---|---|---|---|---|---|---|']
    for row in data['aids']:
        c = row['counts']
        reasons = '; '.join(f'{r}: {n}' for r, n in row['reason_counts'].items()) or 'none'
        lines.append(f"| {row['aid']} | {', '.join(row['units']) or 'unknown'} | {','.join(map(str, row['channels'])) or 'unknown'} | "
                     f"{len(row['variants'])} | {c.get('refusals',0)} ({reasons}) | {c.get('stubs',0)} | {c.get('bounds',0)} | {c.get('todo_spec',0)} |")
    lines += ['', '## Per-reason rollup', '',
              '| Reason | AIDs | CLI errors | Stubs | Probe refusals | Render refusals |', '|---|---|---|---|---|---|']
    for reason, r in data['per_reason'].items():
        lines.append(f"| {reason} | {', '.join(map(str,r['aids']))} | {r.get('compile_errors',0)} | "
                     f"{r.get('stubs',0)} | {r.get('probe_refusals',0)} | {r.get('render_refusals',0)} |")
    lines += ['', '## Stubbed boxes by AID', '', '| AID | Box | Unit | Reason |', '|---|---|---|---|']
    for row in data['aids']:
        details = {(stub['box'], stub['unit'], stub['reason'])
                   for variant in row['variants'] for feature in variant['features'].values()
                   for stub in feature['stub_reasons']}
        for box, unit, reason in sorted(details):
            lines.append(f"| {row['aid']} | {box} | {unit} | {reason} |")
    lines += ['', '## Counting and coverage', '']
    lines += [f'- {k}: {v}' for k, v in data['definitions'].items()]
    lines += ['', '## Limits', ''] + ['- ' + s for s in data['limitations']]
    return '\n'.join(lines) + '\n'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('tuning_root', type=Path)
    p.add_argument('--private-dir', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--source-revision', required=True)
    args = p.parse_args()
    root, private = args.tuning_root.resolve(), args.private_dir.resolve()
    if not {'private', 'tainted'}.intersection(private.parts) or private.exists():
        p.error('use a fresh directory beneath private/ or tainted/')
    private.mkdir(parents=True, mode=0o700)
    os.chmod(private, 0o700)
    aids = sorted(int(p.name[3:]) for p in root.iterdir() if re.fullmatch('AID[0-9]+', p.name) and p.is_dir())
    if not aids:
        p.error('no AIDs in tuning root')
    binaries = inv.build(private)
    programs = {}
    first = sweep(root, aids, private / 'sweep-1', binaries, programs)
    second = sweep(root, aids, private / 'sweep-2', binaries, programs)
    if first != second:
        dump(private / 'mismatch-first.json', first)
        dump(private / 'mismatch-second.json', second)
        raise RuntimeError('repeat sweeps differ; private evidence preserved')
    data = assemble(first, args.source_revision)
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    dump(out / 'aid-sweep.json', data)
    (out / 'aid-sweep.md').write_text(markdown(data))
    dump(out / 'verification.json', {'identical_sweeps': 2,
         'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.glob('aid-sweep.*')}})
    print(total_line(data), flush=True)


if __name__ == '__main__':
    main()
