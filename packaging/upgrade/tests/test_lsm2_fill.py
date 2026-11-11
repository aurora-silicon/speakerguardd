# SPDX-License-Identifier: MIT
"""Short-preset routing, entirely synthetic and without hardware access."""
import contextlib
import io
import json
import os
import pathlib
import plistlib
import struct
import sys
import tempfile
import types
import unittest
from unittest import mock
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from aurora_speaker_upgrade import collect, upgrade as up


def old_bundle(short=True):
    # A generated 963-record subset, not any real Mac's omitted-ID list.
    ids = sorted(collect.lsm2_fill.base_ids())
    if short: ids = ids[58:]
    state = struct.pack(">III", 0, 0, len(ids)) + b"".join(struct.pack(">If", i, n/17) for n, i in enumerate(ids))
    strip = {"strips": [{"effects": [{"unit": {"subtype": collect.lsm2_fill.SUBTYPE},
                                      "aupreset": {"data": state}}]}]}
    return {collect.audio_tuning.MANIFEST: json.dumps({"unit_id": "synthetic", "aid": "AID901"}).encode(),
            "AID901/DSP/Strips/generated.austrip": plistlib.dumps(strip)}


class FillRouting(unittest.TestCase):
    def test_status_names_old_collection_and_route3_only_when_short(self):
        for short in (True, False):
            bundle = old_bundle(short)
            store = types.SimpleNamespace(state=lambda: {}, read_bundle=lambda: bundle, interface="unused")
            output = io.StringIO()
            with mock.patch.object(up, "Store", return_value=store), \
                 mock.patch.object(up, "interface_mode", return_value=("none", None)), \
                 mock.patch.object(up, "read_text", return_value=None), contextlib.redirect_stdout(output):
                self.assertEqual(up.cmd_status(types.SimpleNamespace(store="unused")), 0)
            result = json.loads(output.getvalue())
            if short:
                self.assertEqual(result["lsm2_fill"], "collected before the installer read the lsm2 fill")
                self.assertIn("route 3", result["lsm2_fill_next"])
            else:
                self.assertEqual(result["lsm2_fill"], "not required")

    def test_acquire_and_check_refuse_short_offline_bundle_with_route3_reason(self):
        with tempfile.TemporaryDirectory() as scratch:
            a = types.SimpleNamespace(route="1", chosen="unused", installer_dirs=[], scratch=scratch,
                                      collection=None, adt=None, json=None)
            identity = types.SimpleNamespace(unit="synthetic", board="synthetic", acoustic=901,
                                             matches_dt=lambda: True)
            with mock.patch.object(up, "speaker_id_source", return_value=(None, False, "none")), \
                 mock.patch.object(up, "route1", return_value=(old_bundle(), {"source": "synthetic"})), \
                 mock.patch.object(up.collect, "Identity", return_value=identity), \
                 mock.patch.object(up, "interface_mode", return_value=("none", None)), \
                 mock.patch.object(up, "say") as say:
                with self.assertRaises(up.Refused) as error:
                    up.acquire(identity, a)
                self.assertIn("collected before the installer read the lsm2 fill", str(error.exception))
                self.assertIn("route 3", str(error.exception))
                self.assertEqual(up.cmd_check(a), up.EXIT_REFUSED)
                self.assertTrue(any("route 3" in str(c) for c in say.call_args_list))
