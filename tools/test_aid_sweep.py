#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Synthetic coverage, privacy and failure-accounting checks. Buildbox only."""
import copy
import importlib.util
import json
from pathlib import Path
import plistlib
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('sweep', Path(__file__).with_name('aid-inventory-sweep.py'))
sweep = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sweep)


class SweepCoverage(unittest.TestCase):
    def test_program_cache_preserves_samples_and_channel_stride(self):
        with tempfile.TemporaryDirectory(dir=sweep.inv.REPO) as tmp:
            root = Path(tmp)
            cache = {}
            sweep.cached_program(root / 'stereo', 2, cache)
            sweep.cached_program(root / 'stereo-again', 2, cache)
            sweep.cached_program(root / 'six-channel', 6, cache)
            self.assertEqual((root / 'stereo').read_bytes(), (root / 'stereo-again').read_bytes())
            self.assertEqual((root / 'six-channel').stat().st_size, 3 * (root / 'stereo').stat().st_size)
            self.assertNotEqual((root / 'stereo').stat().st_ino, (root / 'six-channel').stat().st_ino)

    def test_new_graph_parameter_syntax_has_a_stable_public_reason(self):
        self.assertEqual(sweep.why('graph: malformed wireGraphParam PRIVATE_TUNING_BYTES'),
                         'dspg-wire-parameter-syntax-refused')

    def make_root(self, root):
        base = root / 'AID123/DSP'
        (base / 'Strips').mkdir(parents=True)
        (base / 'Graphs').mkdir()
        for f in ('builtin_speaker_out_general.austrip', 'builtin_speaker_out_vendor.austrip',
                  'builtin_speaker_out_diagnostic.austrip'):
            (base / 'Strips' / f).write_bytes(b'PRIVATE_TUNING_BYTES')
        (base / 'Graphs/g.dspg').write_bytes(b'PRIVATE_GRAPH_BYTES')
        def config(strip):
            return {'UnderlyingDeviceProperties': [{'Selector': 'spid', 'Value': 1234567}],
                    'Output': [{'Hardware': [{'Type': 'DSPGraph', 'Path': 'AID123/DSP/Graphs/g.dspg',
                                               'Substitutions': {'RATE': '48000'}},
                                              {'Type': 'AUStrip', 'Path': 'AID123/DSP/Strips/' + strip}]}]}
        general = config('builtin_speaker_out_general.austrip')
        db = {'General': general, 'Telephony': copy.deepcopy(general),
              'General Vendor': config('builtin_speaker_out_vendor.austrip'),
              'Diagnostic': config('builtin_speaker_out_diagnostic.austrip')}
        (base / 'DSPDatabase.plist').write_bytes(plistlib.dumps({'AID123': {'Built-in Speaker': db}}))
        return base

    def test_all_strips_dedup_and_fixture_integrity(self):
        with tempfile.TemporaryDirectory(dir=sweep.inv.REPO) as tmp:
            root = Path(tmp) / 'tainted'
            base = self.make_root(root)
            before = {p: p.read_bytes() for p in base.rglob('*') if p.is_file()}
            variants, configs, strips, disk = sweep.configurations(root, 123)
            self.assertEqual((len(variants), configs, strips, disk), (4, 4, 3, 3))
            self.assertEqual(sorted(len(v['names']) for v in variants[1:]), [1, 1, 2])
            dest = Path(tmp) / 'private-fixture'
            sweep.fixture(root, 123, variants[1], dest)
            self.assertEqual(before, {p: p.read_bytes() for p in before})
            self.assertEqual((dest / 'AID123/DSP/Graphs/g.dspg').read_bytes(), b'PRIVATE_GRAPH_BYTES')
            forced = plistlib.loads((dest / 'AID123/DSP/DSPDatabase.plist').read_bytes())
            config = forced['AID123']['Built-in Speaker']['General']
            self.assertFalse(config['UnderlyingDeviceProperties'])
            self.assertEqual(config['Output'], variants[1]['config']['Output'])

    def test_substitutions_cannot_be_deduplicated(self):
        with tempfile.TemporaryDirectory(dir=sweep.inv.REPO) as tmp:
            root = Path(tmp)
            base = self.make_root(root)
            path = base / 'DSPDatabase.plist'
            db = plistlib.loads(path.read_bytes())
            db['AID123']['Built-in Speaker']['Telephony']['Output'][0]['Hardware'][0]['Substitutions']['RATE'] = '96000'
            path.write_bytes(plistlib.dumps(db))
            self.assertEqual(len(sweep.configurations(root, 123)[0]), 5)

    def test_unreferenced_strip_is_actually_selected_for_every_general_route(self):
        with tempfile.TemporaryDirectory(dir=sweep.inv.REPO) as tmp:
            root = Path(tmp)
            base = self.make_root(root)
            (base / 'Strips/builtin_speaker_orphan.austrip').write_bytes(b'private')
            variants, configs, strips, disk = sweep.configurations(root, 123)
            self.assertEqual(disk, 4)
            self.assertEqual(strips, 4)
            orphans = [v for v in variants if v.get('unreferenced_strip')]
            self.assertEqual(len(orphans), 2)
            self.assertEqual(sum(len(v['names']) for v in variants), configs)
            for variant in orphans:
                hardware = variant['config']['Output'][0]['Hardware']
                self.assertTrue(next(h['Path'] for h in hardware if h['Type'] == 'AUStrip').endswith('speaker_orphan.austrip'))

    def test_probe_failure_still_executes_four_cli_attempts(self):
        commands = []
        def run(cmd, stem):
            commands.append(cmd)
            stem.with_suffix('.stdout').write_text('PRIVATE BAD JSON')
            stem.with_suffix('.stderr').write_text('error[load]: no protection unit PRIVATE_VALUE')
            return 1
        binaries = {f: {'probe': 'probe', 'cli': 'cli', 'renderer': 'renderer'} for f in sweep.FEATURES}
        with tempfile.TemporaryDirectory(dir=sweep.inv.REPO) as tmp, patch.object(sweep.inv, 'run', run):
            features = sweep.measure(Path(tmp), 123, Path(tmp) / 'run', binaries)
        self.assertEqual(sum(cmd[0] == 'cli' for cmd in commands), 4)
        row = {'aid': 123, 'variants': [{'features': features}]}
        data = sweep.assemble([row], 'public-revision')
        self.assertEqual(data['totals']['refusals'], 4)
        self.assertEqual(data['totals']['refused_aids'], 1)
        self.assertNotIn('PRIVATE', json.dumps(data))
        self.assertEqual(sweep.markdown(data), sweep.markdown(copy.deepcopy(data)))


if __name__ == '__main__':
    unittest.main()
