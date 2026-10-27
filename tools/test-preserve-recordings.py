#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Host-only retention regression. All files are synthetic, in owned scratch."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

HELPER = Path(__file__).with_name('preserve-recordings').resolve()

class Retention(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='sg-retention-')
        self.root = Path(self.tmp.name)
        self.path = self.root / 'recording with spaces'
    def tearDown(self):
        self.tmp.cleanup()
    def run_helper(self):
        return subprocess.run(['sh', str(HELPER), str(self.path)], capture_output=True, text=True)
    def test_multiple_restarts_keep_all_segments_and_unrelated_evidence(self):
        historical = self.root / (self.path.name + '.old-evidence')
        historical.write_bytes(b'old evidence')
        for run in range(4):
            for suffix in ['', '.previous', '.next']:
                Path(str(self.path) + suffix).write_bytes(f'{run}:{suffix}'.encode())
            result = self.run_helper()
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stderr.count('recording evidence kept:'), 3)
        archives = list(self.root.glob(self.path.name + '.saved.*'))
        self.assertEqual(len(archives), 4)
        self.assertTrue(all((p.stat().st_mode & 0o777) == 0o700 for p in archives))
        self.assertEqual({f.read_bytes() for p in archives for f in p.iterdir()},
                         {f'{run}:{suffix}'.encode() for run in range(4) for suffix in ['', '.previous', '.next']})
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
