#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Synthetic privacy and classification regressions; run on a build host."""
import importlib.util
from pathlib import Path
import json
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("inventory", Path(__file__).with_name("aid-inventory.py"))
inventory = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(inventory)


class PrivacyAndStatus(unittest.TestCase):
    def test_instrumentation_restores_sources_after_failure(self):
        paths = [inventory.REPO / p for p in [
            "crates/sgv2-core/src/lib.rs", "crates/sgv2-core/src/voicing/chain.rs",
            "crates/sgv2-core/src/voicing/cfreak.rs", "crates/sgv2-core/src/prestage.rs",
            "crates/sgv2-core/src/voicing/lowflow.rs", "crates/sgv2-core/src/voicing/mstk.rs"]]
        originals = {p: p.read_bytes() for p in paths}
        probe = inventory.REPO / "crates/aurora-speaker-tuning/examples/aid-inventory-probe.rs"
        with tempfile.TemporaryDirectory(dir=inventory.REPO) as scratch:
            with self.assertRaisesRegex(RuntimeError, "synthetic build failure"):
                with inventory.overlay(Path(scratch) / "private"):
                    self.assertTrue(probe.exists())
                    raise RuntimeError("synthetic build failure")
        self.assertFalse(probe.exists())
        self.assertEqual(originals, {p: p.read_bytes() for p in paths})

    def test_messages_and_graph_defaults_never_cross_public_boundary(self):
        secret = "PRIVATE_VALUE_MUST_NOT_ESCAPE"
        raw = {"unit": "spp3", "boxes": [
            {"name": "Comp", "subtype": "mstk", "bypass": False, "before_protection": True,
             "protection": False, "mstk": {"algorithm": 0, "quality": 1, "ovs": 0,
                                           "unapproved_field": secret}}],
            "plan": {"nodes": [{"name": "Comp"}], "params": [{"default": secret}]},
            "compiled_boxes": [{"name": "Comp", "kind": "carried", "configured_bounds": []}],
            "gaps": [["Comp", "saved controls are invalid " + secret]],
            "binding": {"board": secret, "unit_id": secret},
            "diagnostics": {"items": [{"code": "synthetic", "message": "TODO(SPEC) " + secret,
                                       "context": secret}]}}
        public = inventory.public_probe(raw)
        self.assertNotIn(secret, json.dumps(public))
        self.assertEqual(public["boxes"][0]["status"], "gap")
        self.assertIn("mstk-saved-controls-refused", public["boxes"][0]["laws"])
        self.assertEqual(public["todo_spec_diagnostic_counts"], {"synthetic": 1})

    def test_clean_info_is_not_an_exactness_certificate(self):
        raw = {"unit": "spp3", "boxes": [
            {"name": "Comp", "subtype": "mstk", "bypass": False,
             "before_protection": True, "protection": False}],
            "plan": {"nodes": [{"name": "Comp"}]},
            "compiled_boxes": [{"name": "Comp", "kind": "carried", "configured_bounds": []}]}
        b = inventory.public_probe(raw)["boxes"][0]
        self.assertEqual(b["status"], "gap")
        self.assertIn("mstk-exact-parity-unverified", b["laws"])

    def test_bound_configuration_does_not_fabricate_a_branch_hit(self):
        raw = {"unit": "spp3", "boxes": [
            {"name": "Limit", "subtype": "cfrk", "bypass": False,
             "before_protection": True, "protection": False}],
            "compiled_boxes": [{"name": "Limit", "kind": "cfrk",
                                "configured_bounds": ["cfrk-slow-release-TODO(SPEC)"]}],
            "trace": [{"name": "Limit", "mask": 0}]}
        b = inventory.public_probe(raw)["boxes"][0]
        self.assertTrue(b["configured_bounds"])
        self.assertFalse(b["exercised_bounds"])
        raw["trace"][0]["mask"] = 1
        self.assertEqual(inventory.public_probe(raw)["boxes"][0]["exercised_bounds"],
                         ["cfrk-slow-release-TODO(SPEC)"])

    def test_disabled_type15_is_not_running(self):
        raw = {"unit": "lsm2", "boxes": [
            {"name": "Tone2", "subtype": "tmst", "bypass": False,
             "before_protection": True, "protection": False,
             "tmst_slots": [{"type": 15, "enabled": False, "group_bypassed": False},
                            {"type": 18, "enabled": True, "group_bypassed": True}]}]}
        t = inventory.public_probe(raw)["boxes"][0]["tmst"]
        self.assertEqual(t["saved_type_counts"], {"15": 1, "18": 1})
        self.assertEqual(t["running_type_counts"], {})

    def test_exact_volume_automation_is_not_counted_as_a_bound(self):
        text = ("info[prestage-volume-automation] Tone: carried at its saved values; "
                "the voicing chain follows its volume automation\n"
                "info[prestage-volume-automation] Tone: the volume-automated shelf is carried as a flat PRIVATE\n"
                "warning[spp3-optional-absent] channel: PilotEnableThres is absent: TODO(SPEC) value when absent\n")
        summary = inventory.diagnostic_summary(text)
        self.assertEqual(summary["emitted_law_counts"], {"tmst-volume-automation-bound": 1})
        self.assertEqual(summary["todo_spec_law_counts"], {"spp3-optional-absent:PilotEnableThres": 1})
        self.assertNotIn("PRIVATE", json.dumps(summary))

    def test_unknown_boxes_survive_a_loader_refusal(self):
        raw = {"unit": "lsm2", "load_error": "box db_convert: builtin dbcontrol is not carried PRIVATE",
               "boxes": [{"name": name, "subtype": subtype, "bypass": False,
                          "channels": 6, "before_protection": False, "protection": False}
                         for name, subtype in [("SysID", "lsid"), ("db_convert", "dbcontrol")]]}
        public = inventory.public_probe(raw)
        self.assertEqual([b["status"] for b in public["boxes"]], ["unimplemented", "unimplemented"])
        self.assertEqual(public["load_refusal"], "dbcontrol-unimplemented")
        self.assertNotIn("PRIVATE", json.dumps(public))


if __name__ == "__main__":
    unittest.main()
