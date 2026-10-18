# SPDX-License-Identifier: MIT
"""Acceptance checks must reject a recovered startup failure as well as an outage."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("reboot_check", Path(__file__).with_name("neo-three-boots.py"))
reboot_check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reboot_check)


def healthy():
    snap = dict(ActiveState="active", SubState="running", Result="success", NRestarts="0",
                MainPID="123", InvocationID="one", ExecMainStatus="0", ExecMainStartTimestampMonotonic="100")
    return dict(trial=1, boot_id="boot-1", kernel="7.2.10-test", expected_kernel="7.2.10-test",
                device_fingerprint="synthetic-device", observe_seconds=30,
                compatible=["apple,j700"], cards=["AppleJ700"], snapshots=[snap, copy.deepcopy(snap)],
                states=[dict(armed="1", age_seconds=1), dict(armed="1", age_seconds=1)],
                journal=[dict(_COMM="speakerguardd", _SYSTEMD_INVOCATION_ID="one",
                              MESSAGE="[ 8.9] I AppleJ700: volume lock taken")])


class AcceptanceTests(unittest.TestCase):
    def test_first_invocation_success(self):
        self.assertEqual(reboot_check.failures(healthy()), [])

    def test_recovered_initial_failure_is_rejected_even_if_restart_count_was_reset(self):
        record = healthy()
        record["journal"].insert(0, dict(_COMM="speakerguardd", _SYSTEMD_INVOCATION_ID="old",
                                       MESSAGE="[ 7.6] E no sound card with id AppleJ700"))
        self.assertTrue(reboot_check.failures(record))

    def test_restart_without_an_error_log_is_rejected(self):
        record = healthy()
        record["snapshots"][1]["NRestarts"] = "1"
        self.assertTrue(reboot_check.failures(record))

    def test_stale_or_unarmed_state_is_rejected(self):
        for state in [dict(armed="0", age_seconds=1), dict(armed="1", age_seconds=6)]:
            record = healthy()
            record["states"][1] = state
            self.assertTrue(reboot_check.failures(record))

    def test_incomplete_or_missing_journal_is_not_a_pass(self):
        for key in ["journal", "states", "snapshots"]:
            record = healthy()
            record[key] = []
            self.assertTrue(reboot_check.failures(record))

    def test_wrong_kernel_or_missing_card_is_rejected(self):
        for key, value in [("kernel", "7.1-test"), ("compatible", ["apple,j314"]), ("cards", [])]:
            record = healthy()
            record[key] = value
            self.assertTrue(reboot_check.failures(record))

    def test_three_distinct_boots_on_one_device_are_required(self):
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            records = []
            for i in range(1, 4):
                record = healthy()
                record.update(trial=i, boot_id=f"boot-{i}")
                records.append(record)

            def check():
                for i, record in enumerate(records, 1):
                    (directory / f"boot-{i}.json").write_text(json.dumps(record))
                return reboot_check.check(directory)

            self.assertEqual(check(), 0)
            records[2]["boot_id"] = records[1]["boot_id"]
            self.assertEqual(check(), 1)
            records[2]["boot_id"] = "boot-3"
            records[2]["device_fingerprint"] = "another-synthetic-device"
            self.assertEqual(check(), 1)


if __name__ == "__main__":
    unittest.main()
