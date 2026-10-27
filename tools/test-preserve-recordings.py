#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Host-only retention regression. All files are synthetic, in owned scratch."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
import importlib.machinery
import importlib.util
from unittest import mock

HELPER = Path(__file__).with_name('preserve-recordings').resolve()

class Retention(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='sg-retention-')
        self.root = Path(self.tmp.name)
        self.path = self.root / 'recording with spaces'
    def tearDown(self):
        self.tmp.cleanup()
    def run_helper(self, budget=None):
        env = os.environ.copy()
        if budget is not None:
            env['SG_RECORD_ARCHIVE_MAX_BYTES'] = str(budget)
        return subprocess.run([str(HELPER), str(self.path)], capture_output=True, text=True, env=env)
    def test_multiple_restarts_prune_oldest_and_keep_newest(self):
        historical = self.root / (self.path.name + '.old-evidence')
        historical.write_bytes(b'old evidence')
        for run in range(4):
            for suffix in ['', '.previous', '.next']:
                Path(str(self.path) + suffix).write_bytes(f'{run}:{suffix}'.encode())
            result = self.run_helper(40)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stderr.count('recording evidence kept:'), 3)
        archives = list(self.root.glob(self.path.name + '.saved.*'))
        self.assertEqual(len(archives), 2)
        self.assertTrue(all((p.stat().st_mode & 0o777) == 0o700 for p in archives))
        self.assertEqual({f.read_bytes() for p in archives for f in p.iterdir()},
                         {f'{run}:{suffix}'.encode() for run in (2, 3) for suffix in ['', '.previous', '.next']})
        self.assertEqual(historical.read_bytes(), b'old evidence')
    def test_empty_and_missing_files(self):
        self.assertEqual(self.run_helper().returncode, 0)
        self.path.touch()
        self.assertEqual(self.run_helper().returncode, 0)
        archives = list(self.root.glob(self.path.name + '.saved.*'))
        self.assertEqual((archives[0] / self.path.name).read_bytes(), b'')
    def test_symlink_and_stale_lock_preserve_sources(self):
        target = self.root / 'unrelated'
        target.write_bytes(b'keep')
        self.path.symlink_to(target)
        self.assertNotEqual(self.run_helper().returncode, 0)
        self.assertTrue(self.path.is_symlink())
        self.path.unlink()
        self.path.write_bytes(b'evidence')
        Path(str(self.path) + '.preserve-lock').mkdir()
        self.assertNotEqual(self.run_helper().returncode, 0)
        self.assertEqual(self.path.read_bytes(), b'evidence')
        self.assertEqual(target.read_bytes(), b'keep')
    def test_budget_refusal_keeps_sources(self):
        self.path.write_bytes(b'newest evidence')
        result = self.run_helper(1)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.path.read_bytes(), b'newest evidence')
    def test_unrelated_saved_name_is_not_pruned(self):
        unrelated = self.root / (self.path.name + '.saved.manual-archive')
        unrelated.mkdir()
        (unrelated / 'private').write_bytes(b'do not delete')
        for i in range(3):
            self.path.write_bytes(f'run{i}'.encode())
            self.assertEqual(self.run_helper(5).returncode, 0)
        self.assertEqual((unrelated / 'private').read_bytes(), b'do not delete')
    def test_storage_failure_does_not_destroy_source(self):
        # A stale lock models an unavailable archive operation. The unit's
        # ignored ExecStartPre failure and create_new keep protection running.
        self.path.write_bytes(b'protected evidence')
        Path(str(self.path) + '.preserve-lock').mkdir()
        result = self.run_helper()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.path.read_bytes(), b'protected evidence')
    def test_partial_move_is_marked_incomplete_and_never_pruned(self):
        loader = importlib.machinery.SourceFileLoader('preserve_recordings_test', str(HELPER))
        spec = importlib.util.spec_from_loader(loader.name, loader)
        helper = importlib.util.module_from_spec(spec)
        loader.exec_module(helper)
        self.path.write_bytes(b'active evidence')
        previous = Path(str(self.path) + '.previous')
        previous.write_bytes(b'previous evidence')
        real_replace = os.replace
        calls = 0
        def fail_second(source, dest):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise OSError('injected storage failure')
            return real_replace(source, dest)
        with mock.patch.object(helper.os, 'replace', side_effect=fail_second), mock.patch.object(helper.sys, 'argv', ['preserve-recordings', str(self.path)]):
            self.assertNotEqual(helper.main(), 0)
        archive = next(self.root.glob(self.path.name + '.saved.*'))
        self.assertTrue((archive / '.incomplete').exists())
        self.assertEqual((archive / self.path.name).read_bytes(), b'active evidence')
        self.assertEqual(previous.read_bytes(), b'previous evidence')
    def test_inherited_pruner_loses_historical_files(self):
        # Same keep-two pipeline as the deployed override, in synthetic scratch.
        for i in range(5):
            f = self.root / f'sg.rec.{i}'
            f.write_bytes(str(i).encode())
            os.utime(f, (100+i, 100+i))
        subprocess.run(['sh', '-c', 'ls -1t ./sg.rec.* | tail -n +3 | xargs -r rm -f'], cwd=self.root, check=True)
        self.assertEqual(len(list(self.root.glob('sg.rec.*'))), 2)
        print('old keep-two policy: three synthetic evidence files deleted')

if __name__ == '__main__':
    unittest.main(verbosity=2)
