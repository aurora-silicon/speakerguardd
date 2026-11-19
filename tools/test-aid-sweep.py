#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Synthetic selection coverage checks; run on buildbox."""
import importlib.util
from pathlib import Path
import plistlib
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('sweep', Path(__file__).with_name('aid-inventory-sweep.py'))
sweep = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sweep)


class Coverage(unittest.TestCase):
    def test_raw_routes_are_excluded_without_reintroducing_their_strips(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as scratch:
            root = Path(scratch)
            dsp = root / 'AID1/DSP'
            (dsp / 'Graphs').mkdir(parents=True)
            (dsp / 'Strips').mkdir()
            (dsp / 'Graphs/protected.dspg').write_text('box P (aufx spp3) 1 1\n')
            (dsp / 'Graphs/raw.dspg').write_text('in source\nout sink\nwire source sink (48000 2)\n')
            for name in ('general', 'retained', 'diagnostic'):
                (dsp / f'Strips/builtin_speaker_{name}.austrip').touch()
            def route(graph, strip):
                return {'Output': [{'Hardware': [
                    {'Type': 'DSPGraph', 'Path': f'AID1/DSP/Graphs/{graph}.dspg'},
                    {'Type': 'AUStrip', 'Path': f'AID1/DSP/Strips/builtin_speaker_{strip}.austrip'}]}]}
            db = {'AID1': {'Built-in Speaker': {
                'General': route('protected', 'general'),
                'Voice': route('protected', 'general'),
                'Diagnostic': route('raw', 'diagnostic')}}}
            (dsp / 'DSPDatabase.plist').write_bytes(plistlib.dumps(db))
            variants, configs, _, disk, excluded = sweep.configurations(root, 1)
            self.assertEqual((configs, disk, len(variants), len(excluded)), (3, 3, 3, 1))
            self.assertTrue(variants[0]['original'])
            self.assertEqual(variants[1]['names'], ['General', 'Voice'])
            self.assertTrue(variants[2]['unreferenced_strip'])
            self.assertFalse(any('diagnostic' in strip for v in variants for strip in v['strips']))
            # An unsupported statement is measured, not hidden as a raw route.
            (dsp / 'Graphs/raw.dspg').write_text('in source\nout sink\nwire source sink (48000 2)\nunknown thing\n')
            self.assertEqual(sweep.configurations(root, 1)[-1], [])


if __name__ == '__main__':
    unittest.main()
