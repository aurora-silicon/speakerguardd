# SPDX-License-Identifier: MIT
"""Whether the boot starts the v2 supervisor, as each distribution enables it."""
import pathlib
import subprocess
import sys
import unittest
from unittest import mock
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from aurora_speaker_upgrade import upgrade as up


def systemctl(is_enabled, show):
    """A stand-in for `systemctl`: is-enabled's exit status, show's properties."""
    def run(*args):
        if args[0] == "is-enabled":
            return subprocess.CompletedProcess(args, is_enabled, "", "")
        if args[0] == "show" and show is not None:
            return subprocess.CompletedProcess(args, 0, "".join(f"{k}={v}\n" for k, v in show.items()), "")
        return subprocess.CompletedProcess(args, 1, "", "")
    return run


class StartsAtBoot(unittest.TestCase):
    def check(self, is_enabled, show):
        with mock.patch.object(up, "systemctl", systemctl(is_enabled, show)):
            return up.starts_at_boot("speakerguardd-v2.service")

    def test_package_wants_link_counts(self):
        # Arch: the package's multi-user.target.wants link in /usr/lib.
        self.assertTrue(self.check(1, {"LoadState": "loaded", "UnitFileState": "disabled",
                                       "WantedBy": "multi-user.target", "RequiredBy": ""}))

    def test_enabled_counts(self):
        # Fedora presets, `systemctl enable`.
        self.assertTrue(self.check(0, None))

    def test_nothing_starts_it(self):
        self.assertFalse(self.check(1, {"LoadState": "loaded", "UnitFileState": "disabled",
                                        "WantedBy": "", "RequiredBy": ""}))

    def test_masked_never(self):
        for load, state in (("masked", "masked"), ("loaded", "masked-runtime")):
            self.assertFalse(self.check(1, {"LoadState": load, "UnitFileState": state,
                                            "WantedBy": "multi-user.target", "RequiredBy": ""}))

    def test_show_fails(self):
        self.assertFalse(self.check(1, None))


if __name__ == "__main__":
    unittest.main()
