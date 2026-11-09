#!/usr/bin/python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""speakerguardd-selfcheck verify: the proof aurora-speaker-upgrade asks for.

Synthetic only: no Apple data. The voicing step's tool output is faked; the
real tool and the macOS parity gate are covered by the crate's tests and by
tools/diffrig/aidgate/attest.py.
"""
import importlib.machinery
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
_loader = importlib.machinery.SourceFileLoader("sgsc_verify", os.path.join(HERE, "speakerguardd-selfcheck"))
_spec = importlib.util.spec_from_loader("sgsc_verify", _loader)
sgsc = importlib.util.module_from_spec(_spec)
_loader.exec_module(sgsc)

KERNEL = os.uname().release


def voicing(result="pass", steps=None, key="ab" * 32, aid=14):
    steps = steps if steps is not None else [
        {"name": "compiler-gaps", "result": "pass", "detail": "none"},
        {"name": "finite", "result": "pass", "detail": "every sample finite"},
        {"name": "macos-parity", "result": "pass", "detail": "within tolerance"},
    ]
    return json.dumps({"result": result, "key": key, "aid": aid, "law": "cd" * 32, "steps": steps})


# The j414s case, as the gate records it (numbers only).
J414S_LIKE = [
    {"name": "compiler-gaps", "result": "pass", "detail": "none"},
    {"name": "finite", "result": "pass", "detail": "every sample finite"},
    {"name": "macos-parity", "result": "refused",
     "detail": "macOS parity gate (macOS 26.6.2 25G83 Audio Units) refused this tuning: 10 cell(s) louder than macOS "
               "by more than 0.25 dB, worst +4.35 dB (lane 0, step_up_125)"},
]


class Verify(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = self.tmp.name
        self.state = os.path.join(self.d, "state")
        os.makedirs(self.state)
        self.params = os.path.join(self.d, "params.bin")
        with open(self.params, "wb") as f:
            f.write(b"SGV2PRM\0" + bytes(64))
        self.graph = os.path.join(self.d, "graph")
        os.makedirs(os.path.join(self.graph, "pipewire"))
        self.conf = os.path.join(self.d, "speakerguardd.conf")
        with open(self.conf, "w") as f:
            f.write("SGV2_MACHINE=j414\nSGV2_UID=4242\nSGV2_TUNING=--bundle /x\n")
        self.out = os.path.join(self.d, "verify.json")
        self.patches = [
            mock.patch.object(sgsc.wiring, "read_dt", lambda *a, **k: {"board": "j414s"}),
            mock.patch.object(sgsc, "CONF", self.conf),
            mock.patch.object(sgsc, "kernel_interface", lambda wait=5.0: (self.iface, "AppleJ414")),
        ]
        self.iface = "dual"
        for p in self.patches:
            p.start()

    def tearDown(self):
        for p in self.patches:
            p.stop()
        self.tmp.cleanup()

    def bind(self, result="pass"):
        rec = {"schema": 1, "version": sgsc.VERSION, "result": result, "board": "j414s", "kernel": KERNEL,
               "params_sha256": sgsc.sha256(self.params), "wiring": "w1", "card": "c", "amps": 6, "sense": 12,
               "family": "sn012776", "key": "j414", "method": "derived",
               "reasons": [] if result == "pass" else ["sense channel 3 stayed live"]}
        with open(os.path.join(self.state, "selfcheck.bind"), "w") as f:
            f.write(sgsc.bind_lines(rec))
        with open(os.path.join(self.state, "selfcheck.json"), "w") as f:
            json.dump({"record": rec}, f)

    def graph_conf(self, quantum=512, lock=True):
        with open(os.path.join(self.graph, "pipewire", "sgv2-j414.conf"), "w") as f:
            f.write(f"node.force-quantum = {quantum}\n" + ("node.lock-quantum = true\n" if lock else ""))

    def run_verify(self, tool_out, tool_rc=0):
        argv = ["speakerguardd-selfcheck", "verify", "--params", self.params, "--state", self.state,
                "--json", self.out, "--graph-dir", self.graph, "--tuning-cli", "/nonexistent/ast"]
        real_run = subprocess.run

        def fake_run(cmd, *a, **k):
            if cmd and cmd[0] == "/nonexistent/ast":
                self.tool_cmd = cmd
                return subprocess.CompletedProcess(cmd, tool_rc, tool_out, "")
            return real_run(cmd, *a, **k)

        with mock.patch.object(sys, "argv", argv), mock.patch.object(sgsc.subprocess, "run", fake_run), \
                mock.patch.object(sgsc.os.path, "exists",
                                  lambda p, _e=os.path.exists: False if str(p).startswith("/run/user/") else _e(p)):
            code = sgsc.main()
        with open(self.out) as f:
            return code, json.load(f)

    def test_pass(self):
        self.bind()
        self.graph_conf()
        code, out = self.run_verify(voicing())
        self.assertEqual((code, out["result"]), (0, "pass"), out)
        self.assertEqual([s["name"] for s in out["steps"]], ["kernel", "hardware", "voicing", "graph"])
        self.assertEqual(out["wiring_hash"], "w1")
        # the tuning source comes from the units' configuration
        self.assertEqual(self.tool_cmd[-2:], ["--bundle", "/x"])
        self.assertIn("no session PipeWire", out["steps"][-1]["detail"])

    def test_j414s_like_tuning_is_refused(self):
        self.bind()
        self.graph_conf()
        code, out = self.run_verify(voicing("refused", J414S_LIKE), tool_rc=1)
        self.assertEqual((code, out["result"]), (1, "refused"))
        self.assertIn("+4.35 dB", out["steps"][-1]["detail"])
        self.assertEqual(out["steps"][-1]["name"], "voicing")

    def test_compiler_gap_is_refused(self):
        self.bind()
        self.graph_conf()
        steps = [{"name": "compiler-gaps", "result": "refused", "detail": "Tone2: parity pending"}]
        code, out = self.run_verify(voicing("refused", steps), tool_rc=1)
        self.assertEqual(code, 1)
        self.assertIn("parity pending", out["steps"][-1]["detail"])

    def test_old_reader_without_voicing_check_refuses(self):
        self.bind()
        self.graph_conf()
        code, out = self.run_verify("", tool_rc=2)
        self.assertEqual((code, out["result"]), (3, "error"))

    def test_no_record_cannot_run(self):
        self.graph_conf()
        code, out = self.run_verify(voicing())
        self.assertEqual((code, out["result"]), (2, "cannot-run"))

    def test_failed_record_is_refused(self):
        self.bind("fail")
        self.graph_conf()
        code, out = self.run_verify(voicing())
        self.assertEqual(code, 1)
        self.assertEqual(out["steps"][-1]["name"], "hardware")

    def test_unlocked_quantum_is_refused(self):
        self.bind()
        self.graph_conf(lock=False)
        code, out = self.run_verify(voicing())
        self.assertEqual(code, 1)
        self.assertIn("512-frame quantum", out["steps"][-1]["detail"])

    def test_legacy_only_kernel_cannot_run(self):
        self.bind()
        self.graph_conf()
        self.iface = "legacy"
        code, out = self.run_verify(voicing())
        self.assertEqual((code, out["result"]), (2, "cannot-run"))
        self.assertEqual(out["steps"][0]["name"], "kernel")

    def test_no_graph_cannot_run(self):
        self.bind()
        code, out = self.run_verify(voicing())
        self.assertEqual(code, 2)


class Pilot(unittest.TestCase):
    """The live graph step against a fake pw-top."""

    def top(self, err_speaker, err_sense):
        return ("S   ID  QUANT   RATE    WAIT    BUSY   W/Q   B/Q  ERR FORMAT           NAME\n"
                f"R   60    512  48000  10.1us  20.2us  0.00  0.00  {err_speaker}   S24LE 6 48000 sgv2.speaker.j414\n"
                f"R   61    512  48000  10.1us  20.2us  0.00  0.00  {err_sense}   S16LE 12 48000 sgv2.sense.j414\n")

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        g = os.path.join(self.tmp.name, "pipewire")
        os.makedirs(g)
        with open(os.path.join(g, "sgv2-j414.conf"), "w") as f:
            f.write("node.force-quantum = 512\nnode.lock-quantum = true\n")
        self.conf = os.path.join(self.tmp.name, "c.conf")
        with open(self.conf, "w") as f:
            f.write(f"SGV2_MACHINE=j414\nSGV2_UID={os.getuid()}\n")
        self.args = mock.Mock(graph_dir=self.tmp.name)
        self.refused = 0

    def tearDown(self):
        self.tmp.cleanup()

    def live(self, tops):
        seq = iter(tops)
        pilots = []

        def fake_run(cmd, *a, **k):
            if "pw-top" in cmd:
                return subprocess.CompletedProcess(cmd, 0, next(seq), "")
            if "pw-cat" in cmd:
                pilots.append(k.get("input"))
                return subprocess.CompletedProcess(cmd, 0, b"", b"")
            raise AssertionError(cmd)

        with mock.patch.object(sgsc, "CONF", self.conf), mock.patch.object(sgsc.subprocess, "run", fake_run), \
                mock.patch.object(sgsc, "refused_starts", lambda: self.refused), \
                mock.patch.object(sgsc.time, "sleep", lambda s: None), \
                mock.patch.object(sgsc.os.path, "exists", lambda p: True):
            try:
                return sgsc.verify_graph(self.args), pilots
            except sgsc.wiring.Refused as e:
                return e, pilots

    def test_clean(self):
        r, pilots = self.live([self.top(0, 0), self.top(0, 0)])
        self.assertIn("no xrun", r)
        self.assertEqual(len(pilots), sgsc.PILOT_STARTS)
        # silent: every pilot sample is zero
        self.assertTrue(all(p and not any(p) for p in pilots))

    def test_xrun_at_stream_start_is_refused(self):
        r, _ = self.live([self.top(0, 0), self.top(3, 0)])
        self.assertIsInstance(r, sgsc.wiring.Refused)
        self.assertIn("sgv2.speaker.j414 +3", str(r))

    def test_refused_starts_before_the_grant_are_not_xruns(self):
        # the kernel refused one start before the grant: pw-top counts it once
        self.refused = 1
        r, pilots = self.live([self.top(1, 0), self.top(1, 0)])
        self.assertIn("no xrun", r)
        self.refused = 1
        r, _ = self.live([self.top(2, 0)])
        self.assertIsInstance(r, sgsc.wiring.Refused)

    def test_xruns_since_graph_start_are_refused(self):
        # DJKIT 11-09: ERR 44 on the M2's speaker node right after boot
        r, pilots = self.live([self.top(44, 0)])
        self.assertIsInstance(r, sgsc.wiring.Refused)
        self.assertIn("since it started", str(r))
        self.assertEqual(pilots, [])


class Packaged(unittest.TestCase):
    """The packaged attestations: generated by tools/diffrig/aidgate/attest.py,
    never by hand."""

    def test_consistent(self):
        with open(os.path.join(HERE, "parity.json")) as f:
            a = json.load(f)
        self.assertEqual(a["format"], 2)
        self.assertEqual(len(a["law"]), 64)
        tol = a["tolerance"]
        builds = [r["build"] for r in a["releases"]]
        self.assertTrue(builds, "at least one attested macOS release")
        self.assertEqual(len(builds), len(set(builds)), "one block per macOS release")
        for r in a["releases"]:
            self.assertRegex(r["build"], r"^[0-9]{2}[A-Z][0-9]+[a-z]?$")
            self.assertIn(r["build"], r["reference"], "the reference names its own build")
            keys = [e["key"] for e in r["entries"]]
            self.assertEqual(len(keys), len(set(keys)), "one entry per tuning key")
            for e in r["entries"]:
                self.assertIn(e["verdict"], ("pass", "refused"))
                self.assertEqual(len(e["key"]), 64)
                if e["verdict"] == "pass":
                    self.assertLessEqual(e["worst_louder_db"], tol["louder_db"], e)
                    self.assertGreaterEqual(e["worst_quieter_db"], -tol["quieter_db"], e)
                else:
                    self.assertTrue(e["reason"])


if __name__ == "__main__":
    unittest.main()
