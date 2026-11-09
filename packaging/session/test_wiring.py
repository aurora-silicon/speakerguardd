#!/usr/bin/env python3
"""Tests of the wiring derivation and the self-check's verdicts, on the
recorded device trees and synthetic sense captures.

    python3 -I packaging/session/test_wiring.py
"""
import copy
import importlib.machinery
import importlib.util
import json
import os
import sys
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen  # noqa: E402
import wiring  # noqa: E402

_loader = importlib.machinery.SourceFileLoader("sgsc", os.path.join(HERE, "speakerguardd-selfcheck"))
_spec = importlib.util.spec_from_loader("sgsc", _loader)
sgsc = importlib.util.module_from_spec(_spec)
_loader.exec_module(sgsc)

def fx(m):
    return copy.deepcopy(gen.fixture(m))


def remap(w, unit=None, swap_vi=False):
    """A wrong map for the checks to catch: the wiring with other unit
    channels, or V and I swapped on every amp."""
    out = copy.deepcopy(w)
    if unit is not None:
        out["unit_channels"] = list(unit)
        for a, u in zip(out["amps"], unit):
            a["unit"] = u
        for s in out["sense"]:
            s["port"] = 2 * out["amps"][s["amp"]]["unit"] + (0 if s["kind"] == "V" else 1)
    if swap_vi:
        for s in out["sense"]:
            s["port"] ^= 1
            s["kind"] = "I" if s["kind"] == "V" else "V"
    out["sense_ports"] = [s["port"] for s in out["sense"]]
    out["hash"] = wiring.wiring_hash(out)
    return out


class OwnerSenseWrites(unittest.TestCase):
    def test_switches_use_the_grant_holding_file(self):
        ctl = object.__new__(sgsc.Control)
        ctl.fd = 73
        with mock.patch.object(sgsc.fcntl, "ioctl") as ioctl:
            for value, expected in (("on", 1), ("off", 0)):
                sgsc.switch_set(ctl, "Left Tweeter VSENSE Switch", value)
                fd, op, buf = ioctl.call_args.args
                self.assertEqual(fd, 73)
                self.assertEqual(op, sgsc._ELEM_WRITE)
                self.assertEqual(bytes(buf[:sgsc._ID_SIZE]), ctl._id("Left Tweeter VSENSE Switch"))
                self.assertEqual(sgsc.struct.unpack_from("q", buf, 72)[0], expected)
            ctl.write(50)
            self.assertEqual(bytes(ioctl.call_args.args[2][:sgsc._ID_SIZE]), ctl._id("Speaker Protection"))

    def test_denied_switch_write_is_reported(self):
        ctl = mock.Mock()
        ctl.write.side_effect = PermissionError("not the owner")
        with self.assertRaisesRegex(wiring.Refused, "not the owner"):
            sgsc.switch_set(ctl, "Left Tweeter ISENSE Switch", "off")


class Derivation(unittest.TestCase):
    def test_every_fixture_derives(self):
        for m in gen.machines():
            f = fx(m)
            w = wiring.derive(f["hw"], f["tuning"])
            self.assertEqual(w["key"], m)
            self.assertEqual(sorted(w["unit_channels"]), list(range(len(w["amps"]))), m)
            self.assertEqual(sorted(w["sense_ports"]), list(range(len(w["sense"]))), m)

    def test_j316_physical_order_and_sense(self):
        w = wiring.derive(fx("j316")["hw"], fx("j316")["tuning"])
        self.assertEqual(
            [a["prefix"] for a in w["amps"]],
            ["Left Woofer 1", "Right Woofer 1", "Left Tweeter", "Right Tweeter", "Left Woofer 2", "Right Woofer 2"],
        )
        # B6: the tweeters are unit channels 4 and 5, the second woofers 2 and 3
        self.assertEqual(w["unit_channels"], [0, 1, 4, 5, 2, 3])
        # each amp is [I, V] on the sense PCM (imon slot below vmon)
        self.assertEqual([s["kind"] for s in w["sense"]], ["I", "V"] * 6)

    def test_a_new_board_derives_with_nothing_written_for_it(self):
        for m in ("j316", "j413"):
            f = fx(m)
            f["hw"]["board"] = "j999"
            f["hw"]["sound_compatible"] = ["apple,j999-macaudio", "apple,macaudio"]
            f["hw"]["model"] = "Test Mac J999"
            w = wiring.derive(f["hw"], f["tuning"])
            ref = wiring.derive(fx(m)["hw"], fx(m)["tuning"])
            self.assertEqual((w["unit_channels"], w["sense_ports"], w["hash"]), (ref["unit_channels"], ref["sense_ports"], ref["hash"]))
            c = wiring.describe(f["hw"], w)
            self.assertEqual((c["key"], c["card"], c["name"]), ("j999", "AppleJ999", "Test Mac J999"))

    def test_b6_bug_identity_order_is_caught(self):
        f = fx("j316")
        w = wiring.derive(f["hw"], f["tuning"])
        bad = remap(w, unit=[0, 1, 2, 3, 4, 5])
        errs = wiring.role_errors(bad, wiring.roles(f["tuning"])["channels"])
        self.assertEqual(len(errs), 4, errs)
        self.assertTrue(any("Left Tweeter" in e and "L woofer" in e for e in errs), errs)
        self.assertEqual(wiring.role_errors(w, wiring.roles(f["tuning"])["channels"]), [])

    def test_b6_bug_vi_order_is_only_seen_by_the_sense_check(self):
        f = fx("j413")
        w = wiring.derive(f["hw"], f["tuning"])
        bad = remap(w, swap_vi=True)
        self.assertNotEqual(bad["hash"], w["hash"])
        self.assertEqual(wiring.role_errors(bad, wiring.roles(f["tuning"])["channels"]), [])

    def test_refusals(self):
        f = fx("j413")
        hw, t = f["hw"], f["tuning"]

        def refused(hw=hw, t=t):
            with self.assertRaises(wiring.Refused) as cm:
                wiring.derive(hw, t)
            return str(cm.exception)

        def edit(fn):
            h = copy.deepcopy(hw)
            fn(h["links"][0]["codecs"])
            return h

        self.assertIn("no sound-name-prefix", refused(edit(lambda c: c[0].pop("prefix"))))
        self.assertIn("not an amp", refused(edit(lambda c: c[0].update(compatible=["vendor,amp"]))))
        self.assertIn("no ti,vmon-slot-no", refused(edit(lambda c: c[1].pop("vmon"))))
        self.assertIn("share a side", refused(edit(lambda c: c[2].update(prefix="Left Woofer"))))
        self.assertIn("names no side", refused(edit(lambda c: c[0].update(prefix="Woofer Left"))))
        self.assertIn("not 0..", refused(edit(lambda c: c[0].update(vmon=4))))
        self.assertIn("16-bit", refused(edit(lambda c: c[0].update(vmon=3))))
        h = copy.deepcopy(hw)
        h["links"][0]["cpus"] = 3
        self.assertIn("back-ends", refused(h))
        h = copy.deepcopy(hw)
        h["links"].append(copy.deepcopy(h["links"][0]))
        self.assertIn("2 speaker dai-links", refused(h))
        t2 = copy.deepcopy(t)
        t2["tuning"]["protection"]["lanes"].pop()
        self.assertIn("3 channels", refused(t=t2))
        t2 = copy.deepcopy(t)
        t2["tuning"]["protection"]["strip_properties"] = [{"number": 64005}]
        self.assertIn("64005", refused(t=t2))
        t2 = copy.deepcopy(t)
        t2["tuning"]["protection"]["lanes"][0]["hops"].append({"join": "XoverHF"})
        self.assertIn("both LF and HF", refused(t=t2))
        t2 = copy.deepcopy(t)
        for l in t2["tuning"]["protection"]["lanes"][:2]:
            l["hops"] = [h for h in l["hops"] if not h["join"].endswith("LF")]
        self.assertIn("some do not", refused(t=t2))
        t2 = copy.deepcopy(t)
        t2["tuning"]["protection"]["lanes"][0]["origin"]["channel"] = 2
        self.assertIn("not left", refused(t=t2))

    def test_full_range_two_channel(self):
        # a J700-like pair: full-range left and right, no sense
        hw = {
            "board": "j999",
            "sound_compatible": ["apple,j999-macaudio", "apple,macaudio"],
            "links": [
                {
                    "name": "Speakers",
                    "cpus": 1,
                    "codecs": [
                        {"node": "a", "prefix": "Right Speaker", "compatible": ["maxim,max98360a"]},
                        {"node": "b", "prefix": "Left Speaker", "compatible": ["maxim,max98360a"]},
                    ],
                }
            ],
        }
        lanes = [{"origin": {"node": "in", "port": 0, "channel": c}, "hops": [{"join": "j"}]} for c in (0, 1)]
        sel = {"sample_rate": 48000, "substitutions": {"fs": "48000"}}
        w = wiring.derive(hw, {"tuning": {"selection": sel, "protection": {"kind": "lsm2", "lanes": lanes}}})
        self.assertEqual(w["rate"], 48000)
        # one back-end: slots in dai-link order, so channel 0 is the right amp
        self.assertEqual([a["prefix"] for a in w["amps"]], ["Right Speaker", "Left Speaker"])
        self.assertEqual(w["unit_channels"], [1, 0])
        self.assertEqual(w["sense"], [])

    def test_lsid_companion_selects_vi_without_changing_plain_lsm2(self):
        f = fx("j316")
        t = f["tuning"]["tuning"]
        t["protection"]["kind"] = "lsm2"
        t["lsid"] = {
            "adaptation_enabled": False, "source_port": "synthetic-sense",
            "source_lanes": list(range(12)),
            "delay_frames": [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        }
        w = wiring.derive(f["hw"], f["tuning"])
        d = wiring.describe(f["hw"], w)
        self.assertEqual((d["plugin"], d["sense"]), ("6ch-vi", 12))
        self.assertEqual(w["unit_channels"], [0, 1, 4, 5, 2, 3])
        for key, value in [("adaptation_enabled", True), ("source_lanes", list(reversed(range(12)))), ("delay_frames", [0] * 12)]:
            bad = copy.deepcopy(f["tuning"])
            bad["tuning"]["lsid"][key] = value
            with self.assertRaises(wiring.Refused):
                wiring.derive(f["hw"], bad)
        no_companion = copy.deepcopy(w)
        no_companion.pop("sense_companion")
        with self.assertRaises(wiring.Refused):
            # No 6ch model-only variant exists: do not silently claim V/I.
            wiring.describe(f["hw"], no_companion)

    def test_the_rate_is_the_tunings(self):
        t = fx("j413")["tuning"]
        self.assertEqual(wiring.tuning_rate(t), 48000)
        for sel, ok in (
            ({"sample_rate": 96000}, 96000),
            ({"substitutions": {"fs": "44100"}}, 44100),
            ({"sample_rate": 96000, "substitutions": {"fs": "96000"}}, 96000),
            ({"sample_rate": 48000, "substitutions": {"fs": "96000"}}, None),
            ({}, None),
            ({"sample_rate": 0}, None),
            ({"substitutions": {"fs": "x"}}, None),
        ):
            t2 = copy.deepcopy(t)
            t2["tuning"]["selection"] = sel
            if ok is None:
                with self.assertRaises(wiring.Refused):
                    wiring.tuning_rate(t2)
            else:
                self.assertEqual(wiring.tuning_rate(t2), ok)
                hw = fx("j413")["hw"]
                self.assertEqual(wiring.describe(hw, wiring.derive(hw, t2))["rate"], ok)
        # a rate is not part of the wiring: the hash is the map's alone
        t2 = copy.deepcopy(t)
        t2["tuning"]["selection"] = {"sample_rate": 96000}
        hw = fx("j413")["hw"]
        self.assertEqual(wiring.derive(hw, t2)["hash"], wiring.derive(hw, t)["hash"])

    def test_macaudio_slot_rule(self):
        self.assertEqual(wiring.tdm_slots(6, 2), [0, 2, 4, 1, 3, 5])
        self.assertEqual(wiring.tdm_slots(4, 2), [0, 2, 1, 3])
        self.assertEqual(wiring.tdm_slots(4, 1), [0, 1, 2, 3])
        self.assertEqual(wiring.tdm_slots(1, 1), [0])

    def test_generated_files_are_current(self):
        def current(p, text):
            if not os.path.exists(p):
                return False
            with open(p) as f:
                return f.read() == text

        stale = [p for p, text in gen.outputs() if not current(p, text)]
        self.assertEqual(stale, [])


def capture(n, quiet=(), pattern=()):
    """Synthetic sense statistics: every channel live except `quiet` (zeros)
    and `pattern` (a fixed two-value idle square)."""
    ch = []
    for k in range(n):
        if k in quiet:
            ch.append({"distinct": 1, "rms": 0.0, "mean": 0.0})
        elif k in pattern:
            ch.append({"distinct": 2, "rms": 2.0, "mean": -2.0})
        else:
            ch.append({"distinct": 60, "rms": 8.0, "mean": -0.4})
    return {"frames": 19200, "ch": ch}


class SenseCheck(unittest.TestCase):
    """check_sense() against a fake card whose amps behave as the J413's
    do: [I, V] per amp, V switched off gives an idle pattern, I gives
    zeros."""

    def run_check(self, w, truth, also=None):
        switches = {}
        also = also or {}
        n = len(w["amps"])

        calls = self.calls = []

        class FakeStreams:
            def __init__(self, *a):
                self.fmt = "S24_LE"

            def start(self, idx):
                calls.append("start")

            def capture(self):
                calls.append("capture")
                quiet, pattern = [], []
                for (prefix, kind), on in switches.items():
                    if on == "off":
                        for ch in (truth[(prefix, kind)], also.get((prefix, kind))):
                            if ch is not None:
                                (pattern if kind == "V" else quiet).append(ch)
                return capture(2 * n, quiet, pattern)

            def stop(self):
                calls.append("stop")

        def get(card, name):
            prefix, kind = name.rsplit(" ", 2)[0], name.rsplit(" ", 2)[1][0]
            return switches.setdefault((prefix, kind), "on")

        def put(card, name, v):
            prefix, kind = name.rsplit(" ", 2)[0], name.rsplit(" ", 2)[1][0]
            switches[(prefix, kind)] = v

        saved = sgsc.Streams, sgsc.switch_get, sgsc.switch_set
        sgsc.Streams, sgsc.switch_get, sgsc.switch_set = FakeStreams, get, put
        try:
            errs, fmt = sgsc.check_sense(None, 1, "AppleJ413", (1, 2), w, {})
        finally:
            sgsc.Streams, sgsc.switch_get, sgsc.switch_set = saved
        self.assertTrue(all(v == "on" for v in switches.values()), "switches restored")
        return errs

    def setUp(self):
        f = fx("j413")
        self.w = wiring.derive(f["hw"], f["tuning"])
        # the J413's real layout: amp p has I on 2p and V on 2p + 1
        self.truth = {}
        for p, a in enumerate(self.w["amps"]):
            self.truth[(a["prefix"], "I")] = 2 * p
            self.truth[(a["prefix"], "V")] = 2 * p + 1

    def test_the_derived_map_passes(self):
        self.assertEqual(self.run_check(self.w, self.truth), [])

    def test_one_grant_for_the_whole_switch_test(self):
        # one stream (one grant) for the baseline and every switch, not a
        # grant per step: 1 + 2 x amps grants at first boot read like the
        # protection flapping
        self.assertEqual(self.run_check(self.w, self.truth), [])
        n = len(self.w["amps"])
        k, _ = sgsc.switch_codes(2 * n)
        self.assertEqual(k, 5)  # J413: 8 switches in 5 coded captures
        self.assertEqual(self.calls, ["start"] + ["capture"] * (1 + k) + ["stop"])

    def test_a_failed_step_still_ends_the_stream_and_the_grant(self):
        calls, switches = [], {}

        class Dying:
            def __init__(self, *a):
                self.fmt = "S24_LE"

            def start(self, idx):
                calls.append("start")

            def capture(self):
                calls.append("capture")
                if calls.count("capture") == 3:
                    raise sgsc.wiring.Refused("the zero-sample stream stopped during the switch test")
                return capture(2 * len(self_w["amps"]), [], [])

            def stop(self):
                calls.append("stop")

        self_w = self.w
        saved = sgsc.Streams, sgsc.switch_get, sgsc.switch_set
        sgsc.Streams = Dying
        sgsc.switch_get = lambda card, name: switches.setdefault(name, "on")
        sgsc.switch_set = lambda card, name, v: switches.__setitem__(name, v)
        try:
            with self.assertRaises(sgsc.wiring.Refused):
                sgsc.check_sense(None, 1, "AppleJ413", (1, 2), self.w, {})
        finally:
            sgsc.Streams, sgsc.switch_get, sgsc.switch_set = saved
        self.assertEqual(calls, ["start", "capture", "capture", "capture", "stop"])
        self.assertTrue(all(v == "on" for v in switches.values()), "switches restored")

    def test_a_vi_override_fails_on_every_channel(self):
        errs = self.run_check(remap(self.w, swap_vi=True), self.truth)
        self.assertEqual(len(errs), 8, errs)
        self.assertIn("Left Woofer VSENSE: the map has its V on channel 0, which went quiet in captures [0, 2], the switch's are "
                      "[0, 1] (channel 0 follows Left Woofer I)", errs)

    def test_a_dead_or_shared_channel_fails(self):
        truth = dict(self.truth)
        truth[("Right Tweeter", "I")] = truth[("Left Tweeter", "I")]
        errs = self.run_check(self.w, truth)
        self.assertTrue(any("Right Tweeter ISENSE" in e for e in errs), errs)

    def test_switch_codes(self):
        # No code contains another (two of k each), all distinct, and coding
        # only where it saves captures.
        for n in range(1, 25):
            k, codes = sgsc.switch_codes(n)
            if not k:
                self.assertLessEqual(n, 5)
                continue
            self.assertLess(k, n)
            self.assertEqual(len(set(codes)), n)
            self.assertTrue(all(len(c) == 2 and c <= set(range(k)) for c in codes))
        self.assertEqual(sgsc.switch_codes(12)[0], 6)  # J316s/J616s: 13 captures -> 7
        self.assertEqual(sgsc.switch_codes(8)[0], 5)   # J413: 9 -> 6
        self.assertEqual(sgsc.switch_codes(4), (0, []))

    def test_a_switch_that_also_quiets_another_channel_fails(self):
        # Left Tweeter's I switch also quiets Left Woofer's I channel: the
        # union of two codes is no single code.
        errs = self.run_check(self.w, self.truth, also={("Left Tweeter", "I"): self.truth[("Left Woofer", "I")]})
        self.assertTrue(any("Left Woofer ISENSE" in e and "no single switch" in e for e in errs), errs)
        self.assertEqual(len(errs), 1, errs)

    def test_a_switch_that_quiets_nothing_fails(self):
        truth = dict(self.truth)
        truth[("Right Woofer", "V")] = None
        errs = self.run_check(self.w, truth)
        self.assertTrue(any("Right Woofer VSENSE" in e and "follows no switch" in e for e in errs), errs)

    def test_j616s_aop_sense_check(self):
        f = fx("j616")
        w = wiring.derive(f["hw"], f["tuning"])
        truth = {}
        for p, a in enumerate(w["amps"]):
            truth[(a["prefix"], "I")] = 2 * p
            truth[(a["prefix"], "V")] = 2 * p + 1
        self.assertEqual(self.run_check(w, truth), [])
        # the template before 2c16d1c: [V, I]
        self.assertEqual(len(self.run_check(remap(w, swap_vi=True), truth)), 12)

    def test_pcm_status_reads_state_and_owner(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(os.path.join(d, "card1", "pcm1p", "sub0"))
            f = os.path.join(d, "card1", "pcm1p", "sub0", "status")
            saved = sgsc.open if hasattr(sgsc, "open") else None
            real_open = open

            def fake_open(path, *a, **k):
                return real_open(path.replace("/proc/asound", d), *a, **k)

            sgsc.open = fake_open
            try:
                with real_open(f, "w") as fh:
                    fh.write("state: RUNNING\nowner_pid   : 4242\ntrigger_time: 12.000000000\n")
                st = sgsc.pcm_status(1, 1, "p")
                self.assertEqual((st["state"], st["owner_pid"]), ("RUNNING", "4242"))
                self.assertEqual(sgsc.pcm_state(1, 1, "p"), "RUNNING")
                with real_open(f, "w") as fh:
                    fh.write("closed\n")
                self.assertEqual(sgsc.pcm_state(1, 1, "p"), "closed")
                self.assertEqual(sgsc.pcm_state(1, 2, "c"), "unknown")
            finally:
                if saved is None:
                    del sgsc.open
                else:
                    sgsc.open = saved

    def test_the_format_alsa_takes_is_the_transports(self):
        c = gen.machine("j616")[0]
        self.assertEqual(sgsc.format_errors(c, "S24_LE"), [])
        self.assertIn("transport gives S24_LE", sgsc.format_errors(c, "S32_LE")[0])
        self.assertTrue(sgsc.format_errors(c, None))
        self.assertTrue(sgsc.format_errors(None, "S24_LE"))

    def test_classifier_on_the_b6_captures(self):
        # live 49-82 distinct values, V off 2-3, I off 1 (B6, J316 and J413)
        self.assertEqual(sgsc.classify({"ch": [{"distinct": 49}, {"distinct": 2}, {"distinct": 3}, {"distinct": 1}]}),
                         ["live", "quiet", "quiet", "quiet"])
        self.assertEqual(sgsc.classify({"ch": [{"distinct": 9}]}), ["unclear"])



class RestoreCommand(unittest.TestCase):
    """Real command/report/binding path; only physical hardware is replaced.

    Force faults that a physical smoke cannot safely inject after all wiring
    phases passed. Cleanup must fail admission and still visit every switch.
    """

    def run_restore(self, fault):
        import tempfile
        import types
        from contextlib import ExitStack

        f = fx("j413")
        w = wiring.derive(f["hw"], f["tuning"])
        w["method"] = "derived"
        hw = dict(f["hw"], board="j413")
        desc = wiring.describe(hw, w)
        switches = {a["prefix"] + " " + kind + "SENSE Switch": "off"
                    for a in w["amps"] for kind in ("V", "I")}
        names = list(switches)
        state = {"stopped": False, "restored": [], "closed": False}
        truth = {a["prefix"] + " " + kind + "SENSE Switch": 2 * p + offset
                 for p, a in enumerate(w["amps"]) for kind, offset in (("I", 0), ("V", 1))}

        class Streams:
            def __init__(self, *args):
                self.fmt = "S24_LE"

            def start(self, idx):
                pass

            def capture(self):
                quiet = [truth[n] for n, value in switches.items() if value == "off"]
                return capture(2 * len(w["amps"]), quiet, [])

            def stop(self):
                state["stopped"] = True

        class Control:
            def __init__(self, idx):
                pass

            def close(self):
                state["closed"] = True

        def put(ctl, name, value):
            if state["stopped"]:
                state["restored"].append(name)
                if fault == "write" and name == names[0]:
                    raise wiring.Refused("injected restore write failure")
            switches[name] = value

        def get(card, name):
            if state["stopped"] and name == names[0]:
                if fault == "readback":
                    return "on"
                if fault == "unreadable":
                    return None
            return switches[name]

        with tempfile.TemporaryDirectory() as tmp, ExitStack() as stack:
            params = os.path.join(tmp, "params")
            with open(params, "wb") as stream:
                stream.write(b"synthetic test parameters")
            args = types.SimpleNamespace(state=tmp, params=params, force=True, defer=False,
                                         deferred=os.path.join(tmp, "selfcheck.deferred"))
            patches = {
                "lv2_shadows": lambda: [], "derive_live": lambda args: (hw, w, desc, []),
                "blob_rate": lambda path: w["rate"], "find_card": lambda hw: (1, desc["card"]),
                "pcm_ids": lambda idx: {("Secondary", "p"): 1, ("Speaker Sense", "c"): 2},
                "pcm_state": lambda *args: "closed", "check_registers": lambda w: ([], {}),
                "Control": Control, "Streams": Streams, "switch_get": get, "switch_set": put,
                "amixer": lambda *args: types.SimpleNamespace(returncode=1, stdout=""),
                "MARKER": os.path.join(tmp, "active"),
            }
            for name, value in patches.items():
                stack.enter_context(mock.patch.object(sgsc, name, value))
            status = sgsc.cmd_run(args)
            with open(os.path.join(tmp, "selfcheck.json")) as stream:
                report = json.load(stream)
            binding = sgsc.read_bind(os.path.join(tmp, "selfcheck.bind"))
            # Check cleanup independently before checking admission, so faults
            # cannot pass just by skipping the hardware portion of the command.
            self.assertTrue(state["stopped"])
            self.assertTrue(state["closed"])
            self.assertEqual(state["restored"], names)
            self.assertFalse(os.path.exists(sgsc.MARKER))
            self.assertEqual(len(report["runs"]), 1 + (sgsc.switch_codes(len(names))[0] or len(names)))
            if fault:
                self.assertEqual(status, 1, (fault, report))
                self.assertEqual(binding["result"], "fail")
                self.assertTrue(any("restoring " + names[0] in r for r in report["record"]["reasons"]))
            else:
                self.assertEqual(status, 0, report)
                self.assertEqual(binding["result"], "pass")

    def test_restore_failures_cannot_persist_a_pass_binding(self):
        for fault in ("write", "readback", "unreadable"):
            with self.subTest(fault=fault):
                self.run_restore(fault)

    def test_verified_cleanup_can_persist_a_pass_binding(self):
        self.run_restore(None)

def fake_dt(root, hw, jack_card=False):
    """A /proc/device-tree in miniature from a fixture's hardware
    description: the root compatible, a back-end node per link (one
    #sound-dai-cells, as the MCA and AOP have), every codec node with its
    properties and phandle, and the macaudio card(s)."""
    ph = [0x100]

    def node(path, props):
        os.makedirs(os.path.join(root, path), exist_ok=True)
        for k, v in props.items():
            with open(os.path.join(root, path, k), "wb") as f:
                f.write(v)
        ph[0] += 1
        with open(os.path.join(root, path, "phandle"), "wb") as f:
            f.write(ph[0].to_bytes(4, "big"))
        return ph[0]

    def strs(*xs):
        return b"".join(x.encode() + b"\0" for x in xs)

    def u32(*xs):
        return b"".join(x.to_bytes(4, "big") for x in xs)

    with open(os.path.join(root, "compatible"), "wb") as f:
        f.write(strs(*hw["compatible"]))
    bes = {}

    def backend(compat):
        key = tuple(compat)
        if key not in bes:
            bes[key] = node(f"soc/backend{len(bes)}", {"#sound-dai-cells": u32(1), "compatible": strs(*compat)})
        return bes[key]

    def card(name, links, model, compat):
        node(name, {"compatible": strs(*compat), "model": strs(model)})
        for i, l in enumerate(links):
            d = f"{name}/dai-link@{i}"
            node(d, {"link-name": strs(l["name"])})
            cpus = l.get("cpu_compatible") or [["apple,mca"]] * l["cpus"]
            node(d + "/cpu", {"sound-dai": u32(*[w for b, c in enumerate(cpus) for w in (backend(c), b)])})
            phs = []
            for c in l["codecs"]:
                props = {"compatible": strs(*c["compatible"]), "#sound-dai-cells": u32(0)}
                if "prefix" in c:
                    props["sound-name-prefix"] = strs(c["prefix"])
                for k in ("vmon", "imon"):
                    if k in c:
                        props[f"ti,{k}-slot-no"] = u32(c[k])
                phs.append(node(c["node"], props))
            node(d + "/codec", {"sound-dai": u32(*phs)})

    card("sound", hw["links"], hw["model"], hw.get("sound_compatible") or ["apple,test-macaudio", "apple,macaudio"])
    if jack_card:
        jack = {"name": "Headphone Jack", "cpus": 1, "codecs": [{"node": "soc/jack", "prefix": "Jack", "compatible": ["cirrus,cs42l84"]}]}
        card("sound-jack", [jack], hw["model"] + " Jack", ["apple,test-macaudio-jack", "apple,macaudio"])


class Atsp(unittest.TestCase):
    """The atsp Macs (SPEC-ATSP 7): every one derives from its device tree
    and tuning alone, with nothing written for the board."""

    BOARDS = {
        # key: (unit channels by physical channel, plugin, amp family)
        "j293": ([0, 1, 2, 3], "4ch", "tas2770"),
        "j493": ([0, 1, 2, 3], "4ch", "sn012776"),
        "j313": ([0, 1], "2ch", "tas2770"),
        "j274": ([0], "1ch", "tas2770"),
        "j375": ([0], "1ch", "sn012776"),
        "j473": ([0], "1ch", "sn012776"),
        "j474": ([0], "1ch", "sn012776"),
        "j475": ([0], "1ch", "sn012776"),
    }

    def test_every_atsp_board_derives(self):
        for m, (unit, plugin, family) in self.BOARDS.items():
            f = fx(m)
            w = wiring.derive(f["hw"], f["tuning"])
            c = wiring.describe(f["hw"], w)
            self.assertEqual((w["unit_kind"], w["unit_channels"], w["family"]), ("atsp", unit, family), m)
            # atsp takes no V/I: the graph carries none, the plugin has none
            self.assertEqual((c["plugin"], c["sense"]), (plugin, 0), m)
            # but the amps have sense, so the self-check can prove the map
            self.assertEqual(len(w["sense"]), 2 * len(w["amps"]), m)
            self.assertEqual(wiring.role_errors(w, wiring.roles(f["tuning"])["channels"]), [], m)

    def test_four_channel_order_is_upper_then_lower(self):
        # AID3/AID18: upper-left, upper-right, lower-left, lower-right
        for m in ("j293", "j493"):
            w = wiring.derive(fx(m)["hw"], fx(m)["tuning"])
            by_unit = {a["unit"]: a["prefix"] for a in w["amps"]}
            self.assertEqual([by_unit[u] for u in range(4)], ["Left Front", "Right Front", "Left Rear", "Right Rear"], m)

    def test_positional_prefixes(self):
        self.assertEqual(wiring.parse_prefix("Left Front"), ("L", "full", 1))
        self.assertEqual(wiring.parse_prefix("Right Rear"), ("R", "full", 2))
        self.assertEqual(wiring.parse_prefix("Left Upper"), ("L", "full", 1))
        self.assertEqual(wiring.parse_prefix("Right Lower"), ("R", "full", 2))
        self.assertEqual(wiring.parse_prefix("Left"), ("L", "full", 1))
        for bad in ("Front Left", "Front", "Left Middle", "Centre"):
            with self.assertRaises(wiring.Refused):
                wiring.parse_prefix(bad)

    def test_mono_needs_one_amp_and_one_lane(self):
        f = fx("j274")
        self.assertNotIn("prefix", f["hw"]["links"][0]["codecs"][0])
        w = wiring.derive(f["hw"], f["tuning"])
        self.assertEqual((w["amps"][0]["side"], w["amps"][0]["role"]), (None, "full"))
        # a stereo tuning on a mono desktop is refused, not guessed
        t = fx("j313")["tuning"]
        with self.assertRaises(wiring.Refused):
            wiring.derive(f["hw"], t)
        # two amps without prefixes are not a mono desktop
        h = copy.deepcopy(f["hw"])
        c = copy.deepcopy(h["links"][0]["codecs"][0])
        c.update(node="x", vmon=6, imon=4)
        h["links"][0]["codecs"].append(c)
        with self.assertRaises(wiring.Refused) as cm:
            wiring.derive(h, fx("j313")["tuning"])
        self.assertIn("no sound-name-prefix", str(cm.exception))

    def test_tas2770_register_check(self):
        w = wiring.derive(fx("j313")["hw"], fx("j313")["tuning"])
        for p, a in enumerate(w["amps"]):
            a["i2c"] = f"1-003{p}"
        # TAS2770: TDM_CFG_REG3 0x0D (RX slot), REG5 0x0F (VSNS), REG6 0x10 (ISNS)
        good = {"1-0030": {0x0D: 0x00, 0x0F: 0x42, 0x10: 0x40}, "1-0031": {0x0D: 0x11, 0x0F: 0x46, 0x10: 0x44}}
        saved = sgsc.regmap_values
        try:
            sgsc.regmap_values = lambda i2c, regs: {r: v for r, v in good[i2c].items() if r in regs}
            errs, seen = sgsc.check_registers(w)
            self.assertEqual(errs, [])
            self.assertEqual(len(seen), 2)
            bad = copy.deepcopy(good)
            bad["1-0031"][0x0D] = 0x00
            bad["1-0030"][0x10] = 0x00
            sgsc.regmap_values = lambda i2c, regs: {r: v for r, v in bad[i2c].items() if r in regs}
            errs, _ = sgsc.check_registers(w)
            self.assertEqual(len(errs), 2, errs)
            self.assertTrue(any("receives TDM slot 0" in e for e in errs), errs)
            self.assertTrue(any("I sense transmit is disabled" in e for e in errs), errs)
        finally:
            sgsc.regmap_values = saved

    def test_every_plugin_variant_is_in_the_lv2_bundle(self):
        ttl = open(os.path.join(HERE, "..", "..", "crates", "sgv2-lv2", "lv2", "sgv2.lv2", "manifest.ttl")).read()
        for v in wiring.PLUGINS.values():
            self.assertIn(f"<https://aurora-silicon.org/lv2/sgv2#{v}>", ttl, v)

    def test_integration_dtbs(self):
        """With SGV2_DTB_DIR naming a directory of the kernel's compiled
        device trees, every atsp board derives from its .dtb exactly as its
        fixture does."""
        d = os.environ.get("SGV2_DTB_DIR")
        if not d:
            self.skipTest("SGV2_DTB_DIR is not set")
        dtbs = {"j293": "t8103-j293", "j313": "t8103-j313", "j493": "t8112-j493", "j274": "t8103-j274",
                "j375": "t6001-j375c", "j473": "t8112-j473", "j474": "t6020-j474s", "j475": "t6021-j475c"}
        for m, name in dtbs.items():
            hw = wiring.read_dtb(os.path.join(d, name + ".dtb"))
            w = wiring.derive(hw, fx(m)["tuning"])
            self.assertEqual(w["hash"], wiring.derive(fx(m)["hw"], fx(m)["tuning"])["hash"], m)


class DeviceTree(unittest.TestCase):
    def test_read_dt_on_the_aop_and_max98360a_layouts(self):
        import tempfile

        for m, jack in (("j616", True), ("j700", False), ("j316", False)):
            want = fx(m)["hw"]
            with tempfile.TemporaryDirectory() as d:
                fake_dt(d, want, jack_card=jack)
                got = wiring.read_dt(d, sysfs=os.path.join(d, "nosysfs"))
            self.assertEqual(got["board"], want["board"], m)
            self.assertEqual(got["sound"], "sound", m)
            self.assertEqual((got["sound_compatible"], got["model"]), (want["sound_compatible"], want["model"]), m)
            strip = lambda links: [[{k: v for k, v in c.items() if k not in ("node", "i2c")} for c in l["codecs"]] + [l["name"], l["cpus"], l["cpu_compatible"]] for l in links]
            self.assertEqual(strip(got["links"]), strip(want["links"]), m)
            self.assertEqual(wiring.hardware(got), wiring.hardware({**want, "links": [
                {**l, "codecs": [{k: v for k, v in c.items() if k != "i2c"} for c in l["codecs"]]} for l in want["links"]]}), m)

    def test_two_speaker_cards_are_ambiguous(self):
        import tempfile

        hw = fx("j616")["hw"]
        with tempfile.TemporaryDirectory() as d:
            import shutil

            fake_dt(d, hw)
            shutil.copytree(os.path.join(d, "sound"), os.path.join(d, "sound2"))
            with self.assertRaises(wiring.Refused):
                wiring.read_dt(d, sysfs=os.path.join(d, "nosysfs"))


class AopAndJ700(unittest.TestCase):
    def test_j616s_derives_iv_and_the_old_hand_map_is_refused(self):
        f = fx("j616")
        w = wiring.derive(f["hw"], f["tuning"])
        self.assertEqual(w["unit_channels"], [0, 1, 2, 3, 4, 5])
        # the amps' own TX slots: ISNS below VSNS, passed through by the AOP ring
        self.assertEqual([s["kind"] for s in w["sense"]], ["I", "V"] * 6)
        self.assertEqual(w["sense_ports"], [1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10])

    def test_j700_stereo_pair_from_one_node(self):
        f = fx("j700")
        w = wiring.derive(f["hw"], f["tuning"])
        self.assertEqual([(a["prefix"], a["slot"], a["side"]) for a in w["amps"]], [("Speaker L", 0, "L"), ("Speaker R", 1, "R")])
        self.assertTrue(all(a["by_strap"] for a in w["amps"]))
        self.assertEqual((w["unit_channels"], w["sense"]), ([0, 1], []))
        self.assertEqual(len(wiring.role_errors(remap(w, unit=[1, 0]), wiring.roles(f["tuning"])["channels"])), 2)

    def test_j700_check_is_static(self):
        f = fx("j700")
        w = wiring.derive(f["hw"], f["tuning"])
        saved = sgsc.pcm_ids
        sgsc.pcm_ids = lambda idx: {("Primary", "p"): 0, ("Secondary", "p"): 1, ("Speaker Sense", "c"): 2}
        try:
            report = {}
            self.assertEqual(sgsc.static(0, w, report), [])
            self.assertIn("SD_MODE", report["static"])
            sgsc.pcm_ids = lambda idx: {("Primary", "p"): 0}
            with self.assertRaises(wiring.Refused):
                sgsc.static(0, w, {})
        finally:
            sgsc.pcm_ids = saved


class NoHandMap(unittest.TestCase):
    """There is no hand map to fall back on: a Mac whose wiring the
    derivation cannot read fails the self-check."""

    def test_an_old_tuning_reader_fails(self):
        import argparse

        refusal = wiring.Refused("aurora-speaker-tuning dump --json gave no JSON (exit 2): usage")
        saved = (wiring.read_dt, wiring.tuning_dump)
        wiring.read_dt = lambda *a, **k: fx("j316")["hw"]

        def no_dump(*a, **k):
            raise refusal

        wiring.tuning_dump = no_dump
        try:
            with self.assertRaises(wiring.Refused):
                sgsc.derive_live(argparse.Namespace(tuning=[], tuning_cli="aurora-speaker-tuning"))
        finally:
            wiring.read_dt, wiring.tuning_dump = saved

    def test_derive_live_describes_the_machine(self):
        import argparse

        saved = (wiring.read_dt, wiring.tuning_dump)
        wiring.read_dt = lambda *a, **k: fx("j616")["hw"]
        wiring.tuning_dump = lambda *a, **k: fx("j616")["tuning"]
        try:
            hw, w, c, errs = sgsc.derive_live(argparse.Namespace(tuning=[], tuning_cli="x"))
        finally:
            wiring.read_dt, wiring.tuning_dump = saved
        self.assertEqual((w["method"], errs), ("derived", []))
        self.assertEqual(c, gen.machine("j616")[0])

    def test_the_blob_header_names_its_rate(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "params.bin")
            head = b"SGV2PRM\0" + (1).to_bytes(4, "little") + b"SPP3" + bytes(8) + (96000).to_bytes(4, "little")
            with open(p, "wb") as f:
                f.write(head + bytes(20))
            self.assertEqual(sgsc.blob_rate(p), 96000)
            with open(p, "wb") as f:
                f.write(b"NOTABLOB" + bytes(40))
            with self.assertRaises(wiring.Refused):
                sgsc.blob_rate(p)

    def test_no_machine_table_is_shipped(self):
        self.assertFalse(os.path.exists(os.path.join(HERE, "machines.json")))
        self.assertFalse(hasattr(wiring, "apply_override"))


def fdt(tree):
    """A flattened device tree (version 17) from {name: value | subtree}."""
    import struct

    strings, names = bytearray(), {}
    out = bytearray()

    def pad():
        while len(out) % 4:
            out.append(0)

    def emit(name, node):
        out.extend(struct.pack(">I", 1))
        out.extend(name.encode() + b"\0")
        pad()
        for k, v in node.items():
            if isinstance(v, dict):
                continue
            if k not in names:
                names[k] = len(strings)
                strings.extend(k.encode() + b"\0")
            out.extend(struct.pack(">III", 3, len(v), names[k]))
            out.extend(v)
            pad()
        for k, v in node.items():
            if isinstance(v, dict):
                emit(k, v)
        out.extend(struct.pack(">I", 2))

    emit("", tree)
    out.extend(struct.pack(">I", 9))
    hdr = 40 + 16
    off_struct, off_strings = hdr, hdr + len(out)
    total = off_strings + len(strings)
    head = struct.pack(">IIIIIIIIII", 0xD00DFEED, total, off_struct, off_strings, 40, 17, 16, 0, len(strings), len(out))
    return head + bytes(16) + bytes(out) + bytes(strings)


def dir_tree(root):
    """A device-tree directory as {name: bytes | subtree}."""
    t = {}
    for e in sorted(os.listdir(root)):
        p = os.path.join(root, e)
        if os.path.isdir(p):
            t[e] = dir_tree(p)
        else:
            with open(p, "rb") as f:
                t[e] = f.read()
    return t


class Dtb(unittest.TestCase):
    def test_a_compiled_tree_reads_as_the_live_one(self):
        import tempfile

        for m in gen.machines():
            want = fx(m)["hw"]
            with tempfile.TemporaryDirectory() as d:
                fake_dt(d, want, jack_card=True)
                live = wiring.read_dt(d, sysfs=os.path.join(d, "nosysfs"))
                path = os.path.join(d, "t.dtb")
                blob = fdt(dir_tree(d))
                with open(path, "wb") as f:
                    f.write(blob)
                self.assertEqual(wiring.read_dtb(path), live, m)

    def test_not_a_dtb(self):
        import tempfile

        with tempfile.NamedTemporaryFile(suffix=".dtb") as f:
            f.write(b"\0" * 64)
            f.flush()
            with self.assertRaises(wiring.Refused):
                wiring.read_dtb(f.name)


class Describe(unittest.TestCase):
    """The rest of the machine description, from the device tree alone."""

    def hw(self, cpu=("apple,mca",), cpus=2, model="Test Mac J999", sound=("apple,j999-macaudio", "apple,macaudio")):
        h = fx("j413")["hw"]
        h.update(board="j999", model=model, sound_compatible=list(sound))
        link = [l for l in h["links"] if l["name"] in ("Speaker", "Speakers")][0]
        link["cpu_compatible"] = [list(cpu)] * cpus
        return h

    def describe(self, h):
        return wiring.describe(h, wiring.derive(h, fx("j413")["tuning"]))

    def test_per_transport_not_per_board(self):
        c = self.describe(self.hw())
        self.assertEqual(
            {k: c[k] for k in ("key", "card", "transport", "amps", "sense", "plugin", "speaker_format", "speaker_headroom", "sense_headroom", "sense_batch")},
            {"key": "j999", "card": "AppleJ999", "transport": "mca", "amps": 4, "sense": 8, "plugin": "4ch-vi",
             "speaker_format": "S24_32LE", "speaker_headroom": 0, "sense_headroom": 512, "sense_batch": True},
        )
        # the kernel binds the first compatible it knows
        c = self.describe(self.hw(cpu=("apple,t6041-aop-audio", "apple,t6030-aop-audio")))
        self.assertEqual((c["transport"], c["speaker_format"], c["sense_batch"], c["speaker_headroom"]), ("aop-spkr", "S24_32LE", False, 256))
        # the T6040 speaker service's ADMAC raises a period interrupt per period (SINKGAP 11-09): no timer scheduling
        self.assertTrue(c["speaker_irq"])
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pipewire", "sgv2-j616.conf")) as f:
            self.assertIn("api.alsa.disable-tsched = true", f.read())
        c = self.describe(self.hw(cpu=("vendor,new-dai", "apple,t8140-aop-audio")))
        self.assertEqual((c["transport"], c["speaker_format"]), ("aop-t8140", "S32LE"))

    def test_the_card_id_is_macaudios(self):
        self.assertEqual(wiring.card_id({"model": "Test Mac J999"}), "AppleJ999")
        self.assertEqual(wiring.card_id({"model": "A " + "x" * 20}), "Apple" + "x" * 10)
        with self.assertRaises(wiring.Refused):
            wiring.card_id({"model": None})

    def test_refusals(self):
        for h, why in (
            (self.hw(cpu=("vendor,new-dai",)), "not a transport"),
            (self.hw(sound=("apple,macaudio",)), "names no apple,<machine>-macaudio"),
            (self.hw(model=""), "no model"),
        ):
            with self.assertRaises(wiring.Refused) as cm:
                self.describe(h)
            self.assertIn(why, str(cm.exception))
        h = self.hw()
        link = [l for l in h["links"] if l["name"] in ("Speaker", "Speakers")][0]
        link["cpu_compatible"] = [["apple,mca"], ["apple,t6041-aop-audio"]]
        with self.assertRaises(wiring.Refused) as cm:
            self.describe(h)
        self.assertIn("mixed speaker transports", str(cm.exception))

    def test_every_fixture_matches_its_shipped_files(self):
        for m in gen.machines():
            c, w = gen.machine(m)
            self.assertEqual(c["key"], m)
            self.assertIn(f'hw:CARD={c["card"]},DEV=1', gen.pipewire_conf(m, c, w))


class Record(unittest.TestCase):
    def test_bind_lines_are_what_the_supervisor_parses(self):
        rec = {"schema": 1, "version": 1, "result": "fail", "board": "j316s", "kernel": "k", "params_sha256": "ab",
               "wiring": "0979c55ed5ddf377", "card": "AppleJ316", "amps": 6, "sense": 12, "family": "sn012776",
               "key": "j316", "method": "derived", "reasons": ["first\nline", "second"]}
        text = sgsc.bind_lines(rec)
        lines = text.splitlines()
        self.assertIn("reason=first line", lines)
        self.assertEqual(len([l for l in lines if l.startswith("reason=")]), 1)
        self.assertTrue(all("=" in l for l in lines))


class BoundRecord(unittest.TestCase):
    """A bound pass is reused only with its record (the graph is generated
    from it): a bind without one is checked again (11-07: a removed
    selfcheck.json with selfcheck.bind left crashed 'generate')."""

    def test_report_matches(self):
        import tempfile
        bind = {"result": "pass", "board": "j616s", "kernel": "7.3.0-aurora-fleet19", "params_sha256": "ab" * 32,
                "wiring": "164e7a0d23003bab"}
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "selfcheck.json")
            self.assertFalse(sgsc.report_matches(path, bind))
            rec = {k: v for k, v in bind.items()}
            with open(path, "w") as f:
                json.dump({"record": rec}, f)
            self.assertTrue(sgsc.report_matches(path, bind))
            rec["params_sha256"] = "cd" * 32
            with open(path, "w") as f:
                json.dump({"record": rec}, f)
            self.assertFalse(sgsc.report_matches(path, bind))
            with open(path, "w") as f:
                f.write("{")
            self.assertFalse(sgsc.report_matches(path, bind))


class RegmapRead(unittest.TestCase):
    """Only the wanted registers' lines are read from the regmap dump, at
    their offsets; a dump whose lines do not line up is read whole."""

    def dump(self, d, regs, width=2):
        os.makedirs(os.path.join(d, "3-0038"))
        with open(os.path.join(d, "3-0038", "registers"), "w") as f:
            for r, v in regs:
                f.write(f"{r:0{width}x}: {v:02x}\n")

    def test_targeted_and_fallback(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d, mock.patch.object(sgsc, "REGMAP", d):
            self.dump(d, [(r, (r * 7) & 0xFF) for r in range(0x40)])
            reads = []
            real = os.pread
            with mock.patch.object(sgsc.os, "pread", side_effect=lambda fd, n, off: (reads.append((n, off)), real(fd, n, off))[1]):
                self.assertEqual(sgsc.regmap_values("3-0038", {0x0C, 0x0E, 0x0F}), {0x0C: 0x54, 0x0E: 0x62, 0x0F: 0x69})
            self.assertEqual(reads, [(64, 0), (7, 0x0C * 7), (7, 0x0E * 7), (7, 0x0F * 7)])
        with tempfile.TemporaryDirectory() as d, mock.patch.object(sgsc, "REGMAP", d):
            # register 0x05 is not in the dump: the offsets shift, read it whole
            self.dump(d, [(r, r) for r in range(0x40) if r != 5], width=4)
            self.assertEqual(sgsc.regmap_values("3-0038", {0x0C, 0x0E, 0x0F}), {0x0C: 0x0C, 0x0E: 0x0E, 0x0F: 0x0F})
        with tempfile.TemporaryDirectory() as d, mock.patch.object(sgsc, "REGMAP", d):
            with self.assertRaises(sgsc.wiring.Refused):
                sgsc.regmap_values("3-0038", {0x0C})


class StreamStop(unittest.TestCase):
    """The zero-sample playback ends on SIGTERM; the sense capture, whose
    data is no longer needed and which can stall once the playback stopped,
    is killed at once (speakerguardd#102: 2 s per check otherwise)."""

    def test_capture_killed_playback_terminated(self):
        class Proc:
            def __init__(self, name):
                self.args, self.sig, self.stdout, self.stderr = [name], [], None, None

            def poll(self):
                return 0 if self.sig else None

            def terminate(self):
                self.sig.append("TERM")

            def kill(self):
                self.sig.append("KILL")

            def wait(self, timeout=None):
                return 0
        st = sgsc.Streams(None, "AppleJ413", 1, 2, 4, 48000)
        st.play, st.rec = Proc("aplay"), Proc("arecord")
        play, rec = st.play, st.rec
        st.stop()
        self.assertEqual(play.sig, ["TERM"])
        self.assertEqual(rec.sig, ["KILL"])


class Deferral(unittest.TestCase):
    """speakerguardd#102: before login only the binding and the cheap checks
    run (`run --defer`); a needed switch test is left to the after-login
    unit (`pending`, then `run`). Nothing passes, and no graph loads, until
    the switch test itself passed."""

    def setUp(self):
        import tempfile
        self.d = tempfile.mkdtemp(prefix="sgv2-defer-")
        self.state = os.path.join(self.d, "state")
        self.deferred = os.path.join(self.d, "run", "speakerguardd-graph", "selfcheck.deferred")
        self.params = os.path.join(self.d, "params.bin")
        with open(self.params, "wb") as f:
            f.write(b"SGV2PRM\0" + bytes(24))
        self.use("j413")
        self.shadows = []
        self.hw_errs = []
        self.calls = []

    def tearDown(self):
        import shutil
        shutil.rmtree(self.d)

    def use(self, m, static=False):
        f = fx(m)
        self.hw = f["hw"]
        self.w = wiring.derive(self.hw, f["tuning"])
        self.w["method"] = "derived-static" if static else "derived"
        self.desc = wiring.describe(self.hw, self.w)

    def run_sc(self, *argv, kernel="7.3.0-test"):
        def active(idx, card, w, report):
            self.calls.append("active")
            return list(self.hw_errs)

        def static(idx, w, report):
            self.calls.append("static")
            return []
        uname = os.uname()
        fake = mock.Mock(release=kernel, sysname=uname.sysname, nodename=uname.nodename)
        with mock.patch.object(sgsc, "lv2_shadows", return_value=self.shadows), \
                mock.patch.object(sgsc, "derive_live", return_value=(self.hw, self.w, self.desc, [])), \
                mock.patch.object(sgsc, "blob_rate", return_value=self.w["rate"]), \
                mock.patch.object(sgsc, "find_card", return_value=(0, self.desc["card"])), \
                mock.patch.object(sgsc, "active", side_effect=active), \
                mock.patch.object(sgsc, "static", side_effect=static), \
                mock.patch.object(sgsc.os, "uname", return_value=fake), \
                mock.patch.object(sys, "argv", ["speakerguardd-selfcheck", *argv, "--state", self.state,
                                                "--deferred", self.deferred, "--params", self.params]):
            return sgsc.main()

    def bind(self):
        return sgsc.read_bind(os.path.join(self.state, "selfcheck.bind"))

    def test_first_boot_defers_then_the_switch_test_passes_after_login(self):
        self.assertEqual(self.run_sc("run", "--defer"), 0)
        self.assertEqual(self.calls, [])
        self.assertTrue(os.path.exists(self.deferred))
        self.assertEqual(self.bind(), {}, "no record before the switch test")
        self.assertEqual(self.run_sc("pending"), 0)
        self.assertEqual(self.run_sc("run"), 0)
        self.assertEqual(self.calls, ["active"])
        self.assertEqual(self.bind()["result"], "pass")
        # pending until the unit's ExecStopPost, after the graph is generated
        self.assertTrue(os.path.exists(self.deferred))
        self.assertEqual(self.run_sc("pending", "--clear"), 0)
        self.assertFalse(os.path.exists(self.deferred))
        self.assertEqual(self.run_sc("pending"), 1)
        # the next boot, same kernel: the bound pass, nothing deferred
        self.assertEqual(self.run_sc("run", "--defer"), 0)
        self.assertEqual(self.calls, ["active"])
        self.assertEqual(self.run_sc("pending"), 1)
        # a new kernel: the binding is stale, deferred again, the old pass
        # does not cover it
        self.assertEqual(self.run_sc("run", "--defer", kernel="7.3.0-next"), 0)
        self.assertEqual(self.calls, ["active"])
        self.assertEqual(self.run_sc("pending"), 0)
        self.assertEqual(self.bind()["kernel"], "7.3.0-test")

    def test_a_failed_switch_test_after_login_fails_closed(self):
        self.assertEqual(self.run_sc("run", "--defer"), 0)
        self.hw_errs = ["AMP V SENSE off quietened sense channel(s) []"]
        self.assertEqual(self.run_sc("run"), 1)
        self.assertEqual(self.bind()["result"], "fail")
        self.assertEqual(self.run_sc("pending", "--clear"), 0)
        self.assertFalse(os.path.exists(self.deferred))
        # the next run before login does not reuse the failure: deferred again
        self.assertEqual(self.run_sc("run", "--defer"), 0)
        self.assertTrue(os.path.exists(self.deferred))
        self.assertEqual(self.bind()["result"], "fail")

    def test_failed_cheap_checks_fail_before_login_without_the_switch_test(self):
        self.shadows = [("/home/u/.lv2/sgv2.lv2", "u's ~/.lv2")]
        self.assertEqual(self.run_sc("run", "--defer"), 1)
        self.assertEqual(self.calls, [])
        self.assertEqual(self.bind()["result"], "fail")
        self.assertFalse(os.path.exists(self.deferred))
        self.assertEqual(self.run_sc("pending"), 1)

    def test_a_board_without_sense_is_never_deferred(self):
        self.use("j700", static=True)
        self.assertEqual(self.run_sc("run", "--defer"), 0)
        self.assertEqual(self.calls, ["static"])
        self.assertEqual(self.bind()["result"], "pass")
        self.assertFalse(os.path.exists(self.deferred))

    def test_without_defer_the_whole_check_runs(self):
        self.assertEqual(self.run_sc("run"), 0)
        self.assertEqual(self.calls, ["active"])
        self.assertEqual(self.bind()["result"], "pass")

    def test_runs_take_the_lock(self):
        import fcntl
        os.makedirs(self.state)
        fd = os.open(os.path.join(self.state, "selfcheck.lock"), os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            with mock.patch.object(sgsc.fcntl, "flock", side_effect=BlockingIOError) as fl:
                with self.assertRaises(BlockingIOError):
                    self.run_sc("run", "--defer")
                self.assertEqual(fl.call_args[0][1], fcntl.LOCK_EX)
        finally:
            os.close(fd)


class GatePath(unittest.TestCase):
    """gen.py finds gate.json where the distributions install it, not only
    beside itself (11-07: the packaged gen.py read /usr/libexec/.../gate.json
    and every Plan A provisioning failed closed)."""

    def test_falls_back_to_the_installed_gate(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            g = os.path.join(d, "gate.json")
            with open(g, "w") as f:
                json.dump({"voicing_pre": {"j413": True}}, f)
            with mock.patch.object(gen, "GATE_PATHS", (os.path.join(d, "missing.json"), g)):
                self.assertTrue(gen.voicing_enabled("j413"))
                self.assertFalse(gen.voicing_enabled("j316s"))
            with mock.patch.dict(os.environ, {"SGV2_GATE": g}), \
                    mock.patch.object(gen, "GATE_PATHS", (os.path.join(d, "missing.json"),)):
                self.assertTrue(gen.voicing_enabled("j413"))
            with mock.patch.object(gen, "GATE_PATHS", (os.path.join(d, "missing.json"),)):
                with self.assertRaises(FileNotFoundError):
                    gen.voicing_enabled("j413")


if __name__ == "__main__":
    unittest.main()
