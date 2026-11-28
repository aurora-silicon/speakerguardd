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
import re
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


def capture(n, quiet=(), pattern=(), late=False):
    """Synthetic sense statistics: every channel live except `quiet` (zeros)
    and `pattern` (a fixed two-value idle square). `late`: the words were
    sampled one bit clock late (J615 before its receiver delay): live idle
    noise then reads near full scale and a quiet channel takes the previous
    word's last bit as its sign bit."""
    ch = []
    for k in range(n):
        if k in quiet:
            ch.append({"distinct": 2, "rms": 16383.5, "mean": -16384.0} if late and k else
                      {"distinct": 1, "rms": 0.0, "mean": 0.0})
        elif k in pattern:
            ch.append({"distinct": 2, "rms": 2.0, "mean": -2.0})
        else:
            ch.append({"distinct": 9000, "rms": 23170.0, "mean": 1100.0} if late else
                      {"distinct": 60, "rms": 8.0, "mean": -0.4})
    return {"frames": 19200, "ch": ch}


class SenseCheck(unittest.TestCase):
    """check_sense() against a fake card whose amps behave as the J413's
    do: [I, V] per amp, V switched off gives an idle pattern, I gives
    zeros."""

    def run_check(self, w, truth, also=None, late=False):
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
                return capture(2 * n, quiet, pattern, late)

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

    def test_sense_sampled_a_bit_clock_late_fails(self):
        # J615 before its receiver delay: the switches still map one to one
        # (a switched-off word reads 0 or -32768), but idle noise is near
        # full scale, so the power the protection computes is meaningless.
        errs = self.run_check(self.w, self.truth, late=True)
        self.assertEqual(len(errs), 1, errs)
        self.assertIn("misframed", errs[0])
        self.assertIn("23170 counts RMS", errs[0])

    def test_idle_ceiling(self):
        self.assertEqual(sgsc.idle_too_loud(capture(4)), [])
        self.assertEqual(sgsc.idle_too_loud(capture(4, late=True)), [0, 1, 2, 3])
        self.assertEqual(sgsc.idle_too_loud({"ch": [{"rms": 2048.0}, {"rms": 2048.5}]}), [1])

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
            def __init__(self, idx, switch=False):
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

    def backend(compat, names=None):
        key = (tuple(compat), tuple(names) if names is not None else None)
        if key not in bes:
            props = {"#sound-dai-cells": u32(1), "compatible": strs(*compat)}
            if names is not None:
                props["dma-names"] = strs(*names)
            bes[key] = node(f"soc/backend{len(bes)}", props)
        return bes[key]

    def card(name, links, model, compat):
        node(name, {"compatible": strs(*compat), "model": strs(model)})
        for i, l in enumerate(links):
            d = f"{name}/dai-link@{i}"
            node(d, {"link-name": strs(l["name"])})
            cpus = l.get("cpu_compatible") or [["apple,mca"]] * l["cpus"]
            dmas = l.get("cpu_dma_names") or [None] * len(cpus)
            node(d + "/cpu", {"sound-dai": u32(*[w for b, (c, n) in enumerate(zip(cpus, dmas)) for w in (backend(c, n), b)])})
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

    def test_mono_amp_sense_switches_have_no_prefix(self):
        # The lone desktop amp has no sound-name-prefix, so ALSA names its
        # switches "VSENSE Switch"/"ISENSE Switch" (the J473's card); the
        # derivation's "Speaker" is a label, not part of the control name.
        for board in ("j274", "j473"):
            f = fx(board)
            w = wiring.derive(f["hw"], f["tuning"])
            self.assertEqual(w["amps"][0]["prefix"], "Speaker")
            self.assertEqual(w["amps"][0]["control_prefix"], "")
            asked = []

            def get(card, name):
                asked.append(name)
                return None

            saved = sgsc.switch_get
            sgsc.switch_get = get
            try:
                errs, fmt = sgsc.check_sense(None, 0, "Apple" + board.upper(), (1, 2), w, {})
            finally:
                sgsc.switch_get = saved
            self.assertEqual(asked, ["VSENSE Switch"], board)
            self.assertEqual(errs, ["no 'VSENSE Switch' control"])
        # an amp that names itself keeps its prefix in the control name
        f = fx("j413")
        w = wiring.derive(f["hw"], f["tuning"])
        self.assertNotIn("control_prefix", w["amps"][0])

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

    def test_j815_t8142_derives_vi_and_the_tweeters_in_the_middle(self):
        # the J815's own derivation, as its silent self-check proved it on the Mac
        f = fx("j815")
        w = wiring.derive(f["hw"], f["tuning"])
        self.assertEqual(
            [a["prefix"] for a in w["amps"]],
            ["Left Woofer 1", "Right Woofer 1", "Left Tweeter", "Right Tweeter", "Left Woofer 2", "Right Woofer 2"],
        )
        self.assertEqual(w["unit_channels"], [0, 1, 2, 3, 4, 5])
        # each amp is [V, I] on the sense PCM (vmon slot below imon)
        self.assertEqual([s["kind"] for s in w["sense"]], ["V", "I"] * 6)
        self.assertEqual(w["sense_ports"], list(range(12)))
        self.assertEqual(w["hash"], "725f518662e42388")
        c = gen.machine("j815")[0]
        self.assertEqual((c["card"], c["transport"], c["plugin"], c["speaker_irq"]), ("AppleJ815", "aop-dma", "6ch-vi", False))

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


def j773g(names=("spkr", "cout", "cin")):
    """The M4 Mac mini (J773g) as its device tree describes it: one
    SN012776 behind the T8132 AOP, whose speaker back-end names a TX channel
    and no sense (its ADT audio-speaker is TX only), and the J473's atsp
    mono tuning. names=None: a back-end without dma-names."""
    f = fx("j473")
    h = f["hw"]
    h.update(board="j773g", compatible=["apple,j773g", "apple,t8132", "apple,arm-platform"],
             sound_compatible=["apple,j773g-macaudio", "apple,macaudio"], model="Mac mini J773g")
    link = next(l for l in h["links"] if l["name"] == "Speaker")
    link["cpu_compatible"] = [["apple,t8132-aop-audio", "apple,t6041-aop-audio"]]
    link["cpu_dma_names"] = [list(names) if names is not None else None]
    return f


class NoSensePath(unittest.TestCase):
    """A sense-capable amp family on a speaker transport that carries no
    V/I sense (the J773g's AOP): proven like a family without sense, for
    one amp only."""

    def test_j773g_derives_without_sense(self):
        f = j773g()
        w = wiring.derive(f["hw"], f["tuning"])
        c = wiring.describe(f["hw"], w)
        self.assertTrue(w["no_sense_path"])
        self.assertEqual((w["family"], w["unit_kind"], w["unit_channels"], w["sense"]), ("sn012776", "atsp", [0], []))
        self.assertEqual((c["plugin"], c["sense"]), ("1ch", 0))
        self.assertEqual(sgsc.method_of(w), "derived-nosense")
        # the wiring the J773g's self-check recorded on the Mac
        self.assertEqual(w["hash"], "d1d62e8a0aaaef3d")

    def test_a_named_sense_channel_keeps_sense(self):
        for names in (("dma", "spkr", "sense", "cout", "cin"), ("spkr", "sense")):
            f = j773g(names)
            w = wiring.derive(f["hw"], f["tuning"])
            self.assertFalse(w["no_sense_path"], names)
            self.assertEqual(len(w["sense"]), 2, names)
            self.assertEqual(sgsc.method_of(w), "derived", names)
            self.assertNotEqual(w["hash"], "d1d62e8a0aaaef3d", names)

    def test_missing_or_partial_dma_names_keep_sense(self):
        # no dma-names, a record from before they were read, or a node that
        # does not name its speaker channel (the base T8132 node: "dma")
        for names in (None, ("dma",), ()):
            f = j773g(names)
            self.assertTrue(wiring.transport_senses(f["hw"]), names)
            w = wiring.derive(f["hw"], f["tuning"])
            self.assertEqual((w["no_sense_path"], len(w["sense"])), (False, 2), names)
        f = j773g()
        next(l for l in f["hw"]["links"] if l["name"] == "Speaker").pop("cpu_dma_names")
        self.assertTrue(wiring.transport_senses(f["hw"]))
        self.assertFalse(wiring.derive(f["hw"], f["tuning"])["no_sense_path"])

    def test_other_transports_are_unaffected(self):
        # a back-end whose transport has no sense channel name keeps its family's sense
        for m in ("j316", "j473", "j616", "j815"):
            f = fx(m)
            link = next(l for l in f["hw"]["links"] if l["name"] in ("Speaker", "Speakers"))
            link["cpu_dma_names"] = [["tx"]] * link["cpus"]
            if m == "j616":
                continue  # an aop-spkr back-end: see test_several_amps_without_sense_are_refused
            self.assertTrue(wiring.transport_senses(f["hw"]), m)
        # J616s keeps its hash with or without the recorded names
        f = fx("j616")
        before = wiring.derive(f["hw"], f["tuning"])["hash"]
        f["hw"]["links"][0]["cpu_dma_names"] = [["dma", "spkr", "sense", "cout", "cin"]]
        self.assertEqual(wiring.derive(f["hw"], f["tuning"])["hash"], before)
        self.assertEqual(before, "164e7a0d23003bab")

    def test_sense_units_are_refused(self):
        f = j773g()
        f["tuning"]["tuning"]["protection"]["kind"] = "spp3"
        with self.assertRaises(wiring.Refused) as cm:
            wiring.derive(f["hw"], f["tuning"])
        self.assertIn("carries none", str(cm.exception))
        f = j773g()
        t = f["tuning"]["tuning"]
        t["protection"]["kind"] = "lsm2"
        t["lsid"] = {"adaptation_enabled": False, "source_port": "synthetic-sense", "source_lanes": [0, 1],
                     "delay_frames": [0, 0]}
        with self.assertRaises(wiring.Refused) as cm:
            wiring.derive(f["hw"], f["tuning"])
        self.assertIn("V/I sense", str(cm.exception))

    def test_several_amps_without_sense_are_refused(self):
        # A multi-amp Mac whose device tree lost its sense channel (plain
        # lsm2, no LSID): its channel order would rest on the device tree
        # alone, so it fails rather than runs unproven.
        f = fx("j616")
        f["hw"]["links"][0]["cpu_dma_names"] = [["dma", "spkr", "cout", "cin"]]
        f["tuning"]["tuning"]["protection"]["kind"] = "lsm2"
        self.assertFalse(wiring.transport_senses(f["hw"]))
        with self.assertRaises(wiring.Refused) as cm:
            wiring.derive(f["hw"], f["tuning"])
        self.assertIn("one amp only", str(cm.exception))

    def test_aop_dma_sense_follows_the_named_channel(self):
        # t8152_aop_audio (T8142, T6050): the J815 names "sense" and keeps
        # its sense and its hash; without it, it is refused (its spp3 unit
        # reads sense, and six amps could not run without it anyway).
        f = fx("j815")
        link = next(l for l in f["hw"]["links"] if l["name"] == "Speaker")
        link["cpu_dma_names"] = [["cout", "cin", "spkr", "sense", "hpai"]]
        self.assertTrue(wiring.transport_senses(f["hw"]))
        self.assertEqual(wiring.derive(f["hw"], f["tuning"])["hash"], "725f518662e42388")
        link["cpu_dma_names"] = [["cout", "cin", "spkr"]]
        self.assertFalse(wiring.transport_senses(f["hw"]))
        with self.assertRaises(wiring.Refused) as cm:
            wiring.derive(f["hw"], f["tuning"])
        self.assertIn("carries none", str(cm.exception))
        f["tuning"]["tuning"]["protection"]["kind"] = "lsm2"
        with self.assertRaises(wiring.Refused) as cm:
            wiring.derive(f["hw"], f["tuning"])
        self.assertIn("one amp only", str(cm.exception))

    def test_t6050_one_amp_without_sense(self):
        # The M5 Mac Studio's shape: one SN012776 on the T6050 AOP whose
        # audio node names no "sense" (its ADT audio-speaker is TX only).
        f = j773g(("cout", "cin", "spkr"))
        link = next(l for l in f["hw"]["links"] if l["name"] == "Speaker")
        link["cpu_compatible"] = [["apple,t6050-aop-audio"]]
        self.assertFalse(wiring.transport_senses(f["hw"]))
        w = wiring.derive(f["hw"], f["tuning"])
        self.assertTrue(w["no_sense_path"])
        self.assertEqual((w["unit_channels"], w["sense"]), ([0], []))
        self.assertEqual(sgsc.method_of(w), "derived-nosense")
        link["cpu_dma_names"] = [["cout", "cin", "spkr", "sense"]]
        self.assertTrue(wiring.transport_senses(f["hw"]))
        self.assertFalse(wiring.derive(f["hw"], f["tuning"])["no_sense_path"])

    def test_static_report_names_the_no_sense_case(self):
        f = j773g()
        w = wiring.derive(f["hw"], f["tuning"])
        w["method"] = sgsc.method_of(w)
        report = {}
        with mock.patch.object(sgsc, "pcm_ids", return_value={("Secondary", "p"): 1}):
            self.assertEqual(sgsc.static(None, w, report), [])
        self.assertIn("no V/I sense", report["static"])
        self.assertNotIn("SD_MODE", report["static"])

    def test_read_dt_records_dma_names(self):
        import tempfile

        want = j773g()["hw"]
        with tempfile.TemporaryDirectory() as d:
            fake_dt(d, want, jack_card=False)
            got = wiring.read_dt(d, sysfs=os.path.join(d, "nosysfs"))
        link = next(l for l in got["links"] if l["name"] == "Speaker")
        self.assertEqual(link["cpu_dma_names"], [["spkr", "cout", "cin"]])
        self.assertFalse(wiring.transport_senses(got))
        self.assertEqual(wiring.derive(got, j773g()["tuning"])["hash"], "d1d62e8a0aaaef3d")


def t8152_six_amp(names=("hpai", "spkr", "sense", "cout", "cin")):
    """Six SN012776 amps behind the T8152 AOP back-end (J804 shape), using the
    generic J815 six-channel spp3 lane structure."""
    f = fx("j815")
    h = f["hw"]
    h.update(board="j804", compatible=["apple,j804", "apple,t8152", "apple,arm-platform"],
             sound_compatible=["apple,j804-macaudio", "apple,macaudio"], model="MacBook Pro J804")
    link = next(l for l in h["links"] if l["name"] == "Speaker")
    link["name"] = "Speakers"
    link["cpu_compatible"] = [["apple,t8152-aop-audio"]]
    link["cpu_dma_names"] = [list(names) if names is not None else None]
    return f


def t8152_one_amp(names=("cout", "cin", "spkr")):
    """One SN012776 behind the T8152 AOP back-end (J873g shape) with a mono
    atsp tuning."""
    f = fx("j473")
    h = f["hw"]
    h.update(board="j873g", compatible=["apple,j873g", "apple,t8152", "apple,arm-platform"],
             sound_compatible=["apple,j873-macaudio", "apple,macaudio"], model="Mac mini J873")
    link = next(l for l in h["links"] if l["name"] == "Speaker")
    link["cpu_compatible"] = [["apple,t8152-aop-audio"]]
    link["cpu_dma_names"] = [list(names) if names is not None else None]
    link["codecs"] = [{"node": "soc/i2c@33101c000/codec@38", "compatible": ["ti,sn012776", "ti,tas2764"],
                       "vmon": 0, "imon": 2, "i2c": "1-0038"}]
    return f


class T8152Transport(unittest.TestCase):
    """apple,t8152-aop-audio binds the same t8152_aop_audio driver (aop-dma)
    as T8142/T6050: sense follows the back-end node's dma-names, and PCM
    widths follow the link's amps and sense lanes."""

    def test_six_amps_with_sense_derive_6ch_vi(self):
        for names in (("hpai", "spkr", "sense", "cout", "cin"), None):
            f = t8152_six_amp(names)
            self.assertTrue(wiring.transport_senses(f["hw"]), names)
            w = wiring.derive(f["hw"], f["tuning"])
            c = wiring.describe(f["hw"], w)
            self.assertFalse(w["no_sense_path"], names)
            self.assertEqual(len(w["sense"]), 12, names)
            self.assertEqual(sgsc.method_of(w), "derived", names)
            self.assertEqual(
                (c["card"], c["transport"], c["plugin"], c["speaker_pcm"], c["sense_pcm"],
                 c["alsa_format"], c["speaker_format"], c["speaker_irq"]),
                ("AppleJ804", "aop-dma", "6ch-vi", 6, 12, "S24_LE", "S24_32LE", False),
                names,
            )

    def test_six_amps_without_sense_channel_are_refused(self):
        f = t8152_six_amp(("hpai", "spkr", "cout", "cin"))
        self.assertFalse(wiring.transport_senses(f["hw"]))
        with self.assertRaises(wiring.Refused) as cm:
            wiring.derive(f["hw"], f["tuning"])
        self.assertIn("carries none", str(cm.exception))
        f["tuning"]["tuning"]["protection"]["kind"] = "lsm2"
        with self.assertRaises(wiring.Refused) as cm:
            wiring.derive(f["hw"], f["tuning"])
        self.assertIn("one amp only", str(cm.exception))

    def test_one_amp_with_sense_fails_closed_when_card_has_no_sense_pcm(self):
        for names in (("cout", "cin", "spkr", "sense"), None):
            f = t8152_one_amp(names)
            self.assertTrue(wiring.transport_senses(f["hw"]), names)
            w = wiring.derive(f["hw"], f["tuning"])
            self.assertFalse(w["no_sense_path"], names)
            self.assertEqual(len(w["sense"]), 2, names)
            self.assertEqual(sgsc.method_of(w), "derived", names)
            with mock.patch.object(sgsc, "pcm_ids", return_value={("Secondary", "p"): 1}):
                with self.assertRaises(wiring.Refused) as cm:
                    sgsc.active(0, "AppleJ873", w, {})
                self.assertIn("sense PCM None", str(cm.exception))

    def test_one_amp_without_sense_channel_derives_nosense(self):
        for names in (("cout", "cin", "spkr"), ("spkr", "cout", "cin")):
            f = t8152_one_amp(names)
            self.assertFalse(wiring.transport_senses(f["hw"]), names)
            w = wiring.derive(f["hw"], f["tuning"])
            c = wiring.describe(f["hw"], w)
            self.assertTrue(w["no_sense_path"], names)
            self.assertEqual((w["family"], w["unit_kind"], w["unit_channels"], w["sense"]),
                             ("sn012776", "atsp", [0], []), names)
            self.assertEqual((c["transport"], c["plugin"], c["speaker_pcm"], c["sense_pcm"], c["alsa_format"]),
                             ("aop-dma", "1ch", 1, 0, "S24_LE"), names)
            self.assertFalse(c["speaker_irq"], names)
            self.assertEqual(sgsc.method_of(w), "derived-nosense", names)
            self.assertEqual(w["hash"], "d1d62e8a0aaaef3d", names)

    def test_t8142_keeps_its_sense_and_hash(self):
        f = fx("j815")
        self.assertTrue(wiring.transport_senses(f["hw"]))
        w = wiring.derive(f["hw"], f["tuning"])
        self.assertEqual((w["no_sense_path"], len(w["sense"]), w["hash"]), (False, 12, "725f518662e42388"))
        self.assertEqual(wiring.describe(f["hw"], w)["transport"], "aop-dma")


class RunLockedDispatch(unittest.TestCase):
    """run_locked() proves each method on its own branch, and fails closed on
    a method no branch names."""

    def run_with(self, w, defer=False):
        import tempfile
        import types

        f = j773g()
        desc = wiring.describe(f["hw"], w)
        calls = []

        def static(idx, w2, report):
            calls.append(("static", w2["method"]))
            report["static"] = "spy"
            return []

        def active(idx, card, w2, report, args=None):
            calls.append(("active", w2["method"]))
            return []

        with tempfile.TemporaryDirectory() as d:
            params = os.path.join(d, "params.bin")
            with open(params, "wb") as fh:
                fh.write(b"params")
            args = types.SimpleNamespace(state=d, params=params, force=False, defer=defer,
                                         deferred=os.path.join(d, "deferred"))
            with mock.patch.object(sgsc, "lv2_shadows", return_value=[]), \
                    mock.patch.object(sgsc, "derive_live", return_value=(f["hw"], w, desc, [])), \
                    mock.patch.object(sgsc, "blob_rate", return_value=w["rate"]), \
                    mock.patch.object(sgsc, "read_bind", return_value={}), \
                    mock.patch.object(sgsc, "find_card", return_value=(0, desc["card"])), \
                    mock.patch.object(sgsc, "set_deferred"), \
                    mock.patch.object(sgsc, "static", side_effect=static), \
                    mock.patch.object(sgsc, "active", side_effect=active), \
                    mock.patch.object(sgsc, "log"):
                rc = sgsc.run_locked(args)
            with open(os.path.join(d, "selfcheck.json")) as fh:
                rec = json.load(fh)["record"]
        return rc, rec, calls

    def derived_j773g(self):
        f = j773g()
        w = wiring.derive(f["hw"], f["tuning"])
        w["method"] = sgsc.method_of(w)
        return w

    def test_a_no_sense_mac_takes_the_static_branch(self):
        for defer in (False, True):
            rc, rec, calls = self.run_with(self.derived_j773g(), defer=defer)
            self.assertEqual(calls, [("static", "derived-nosense")], defer)
            self.assertEqual((rc, rec["result"], rec["method"], rec["sense"]), (0, "pass", "derived-nosense", 0), defer)

    def test_an_unknown_method_fails_closed(self):
        w = self.derived_j773g()
        w["method"] = "derived-unproven"
        rc, rec, calls = self.run_with(w)
        self.assertEqual(calls, [])
        self.assertEqual((rc, rec["result"]), (1, "fail"))
        self.assertTrue(any("derived-unproven" in r for r in rec["reasons"]), rec["reasons"])


class NoSenseUnits(unittest.TestCase):
    def test_only_the_units_that_run_without_sense(self):
        # an allowlist: atsp and plain lsm2 run on a transport without sense
        for kind in wiring.NO_SENSE_UNITS:
            f = j773g()
            f["tuning"]["tuning"]["protection"]["kind"] = kind
            self.assertTrue(wiring.derive(f["hw"], f["tuning"])["no_sense_path"], kind)
        # anything else is refused there, a unit added later included
        for kind in ("spp3", "lsid", "spp4"):
            f = j773g()
            f["tuning"]["tuning"]["protection"]["kind"] = kind
            with self.assertRaises(wiring.Refused, msg=kind) as cm:
                wiring.derive(f["hw"], f["tuning"])
            self.assertIn("carries none", str(cm.exception), kind)
        self.assertFalse(set(wiring.SENSE_UNITS) & set(wiring.NO_SENSE_UNITS))


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
        self.assertEqual(dict(c, aid=w["aid"]), gen.machine("j616")[0])

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
        # the T8140 speaker DMA raises a period interrupt per period (SINKGAP 11-09): no timer scheduling
        self.assertTrue(c["speaker_irq"])
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pipewire", "sgv2-j700.conf")) as f:
            self.assertIn("api.alsa.disable-tsched = true", f.read())

    def test_mca_without_generic_fallback(self):
        expected = self.describe(self.hw())
        for cpu in (("apple,t8103-mca",), ("apple,t6020-mca", "apple,t8103-mca")):
            with self.subTest(cpu=cpu):
                self.assertEqual(self.describe(self.hw(cpu=cpu)), expected)
        h = self.hw()
        link = [l for l in h["links"] if l["name"] in ("Speaker", "Speakers")][0]
        link["cpu_compatible"] = [["apple,t8103-mca"], ["apple,mca"]]
        self.assertEqual(self.describe(h), expected)

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


class Microphones(unittest.TestCase):
    """The built-in microphone graph's shipped files, from the fixtures'
    microphone nodes (wiring.read_mic_dt of each board's device tree)."""

    def with_mic(self):
        return [m for m in gen.machines() if gen.mic_machine(m) is not None]

    def test_shipped_files_are_the_fixtures_with_an_array(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(gen.__file__)))
        have = sorted(f[len("sgv2-mic-"):-len(".conf")] for f in os.listdir(os.path.join(pkg, "pipewire")) if f.startswith("sgv2-mic-"))
        rules = sorted(f[len("61-sgv2-mic-"):-len(".conf")] for f in os.listdir(os.path.join(pkg, "wireplumber")) if f.startswith("61-sgv2-mic-"))
        self.assertEqual(have, self.with_mic())
        self.assertEqual(rules, self.with_mic())
        self.assertTrue(have)
        # a desktop Mac has no array, and every fixture says which it is
        self.assertTrue(set(gen.machines()) - set(have))
        for m in gen.machines():
            self.assertIn("mic", gen.fixture(m), m)

    def test_the_graph_is_the_arrays(self):
        for m in self.with_mic():
            d = gen.mic_machine(m)
            fx = gen.fixture(m)["mic"]
            t = wiring.mic_transport(fx)
            self.assertIn(t["dma"], fx["dma_names"])
            self.assertEqual((d["key"], d["card"], d["dev"]), (m, f"Apple{fx['chassis']}{t['card']}", t["dev"]))
            pw, wp = gen.mic_pipewire_conf(d), gen.mic_wireplumber_conf(d)
            for text in (pw, wp):
                self.assertEqual(text.count("{"), text.count("}"))
                self.assertEqual(text.count("["), text.count("]"))
            # sgv2's own node on the PCM, by the name no other rule matches
            self.assertIn(f'api.alsa.path          = "hw:CARD={d["card"]},DEV={d["dev"]}"', pw)
            self.assertNotIn(f'"hw:{d["card"]},{d["dev"]}"', pw)
            self.assertIn(f'sgv2#mic-{d["channels"]}ch"', pw)
            self.assertEqual(pw.count('"mic:in_'), d["channels"])
            self.assertEqual(pw.count(f"audio.rate         = {d['rate']}"), 2)
            # exactly one source is called the built-in microphone, above the raw array
            self.assertEqual(pw.count('node.description   = "Built-in Microphone"'), 1)
            self.assertEqual(pw.count('media.class        = "Audio/Source"') + pw.count('media.class            = "Audio/Source"'), 2)
            self.assertIn(f"priority.session       = {gen.MIC_RAW_PRIORITY}", pw)
            self.assertIn(f"priority.session   = {gen.MIC_PRIORITY}", pw)
            self.assertLess(gen.MIC_RAW_PRIORITY, gen.MIC_PRIORITY)
            self.assertIn(f'target.object      = "{gen.mic_names(m)["raw"]}"', pw)
            # WirePlumber's own node on that PCM is the one switched off
            self.assertIn(f'{{ api.alsa.path = "hw:{d["card"]},{d["dev"]}" }}', wp)
            self.assertIn(f'{{ alsa.id = "{d["card"]}", alsa.device = {d["dev"]} }}', wp)
            self.assertIn("node.disabled = true", wp)
            self.assertNotIn("software-dsp", wp.split("monitor.alsa.rules")[1])

    def test_no_array_without_its_stream_or_its_card_name(self):
        fx = gen.fixture(self.with_mic()[0])["mic"]
        self.assertEqual(wiring.describe_mic(fx)["channels"], wiring.mic_transport(fx)["channels"])
        with self.assertRaises(wiring.Refused):
            wiring.describe_mic(dict(fx, chassis=None))
        with self.assertRaises(wiring.Refused):
            wiring.describe_mic(dict(fx, chassis="J3/16"))
        with self.assertRaises(wiring.Refused):
            wiring.describe_mic(dict(fx, compatible=["apple,other-audio"]))

    def test_the_speaker_wiring_does_not_read_the_microphone_node(self):
        for m in gen.machines():
            fx = gen.fixture(m)
            c, w = gen.machine(m)
            fx2 = dict(fx, mic=None)
            self.assertEqual(wiring.derive(fx2["hw"], fx2["tuning"])["hash"], w[2])


class PcmWidths(unittest.TestCase):
    """A transport whose PCMs are wider than the link's amps: the T6040
    speaker service has six playback and twelve sense channels on every
    Mac, and the MacBook Air M4 (T8132 AOP) puts four amps on it."""

    CPU = ["apple,t8132-aop-audio", "apple,t6041-aop-audio", "apple,t6030-aop-audio"]

    def hw(self):
        """Four SN012776 amps on one I2C bus behind one AOP back-end, as the
        device tree of the MacBook Air M4 describes them, on a made-up board."""
        amps = (("Left Woofer", 0x38), ("Right Woofer", 0x3B), ("Left Tweeter", 0x39), ("Right Tweeter", 0x3C))
        return {
            "board": "j998", "compatible": ["apple,j998", "apple,t8132", "apple,arm-platform"], "sound": "sound",
            "sound_compatible": ["apple,j998-macaudio", "apple,macaudio"], "model": "Test Mac J998",
            "links": [{"name": "Speakers", "cpus": 1, "cpu_compatible": [list(self.CPU)], "codecs": [
                {"node": f"soc/i2c@3ad01c000/codec@{addr:x}", "prefix": prefix, "compatible": ["ti,sn012776", "ti,tas2764"],
                 "vmon": 4 * k + 2, "imon": 4 * k}
                for k, (prefix, addr) in enumerate(amps)]}],
        }

    def machine(self, hw):
        w = wiring.derive(hw, fx("j413")["tuning"])
        return wiring.describe(hw, w), w

    def test_from_a_device_tree(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            fake_dt(d, self.hw(), jack_card=True)
            live = wiring.read_dt(d, sysfs=os.path.join(d, "nosysfs"))
            path = os.path.join(d, "t.dtb")
            with open(path, "wb") as f:
                f.write(fdt(dir_tree(d)))
            compiled = wiring.read_dtb(path)
        for hw in (live, compiled):
            c, w = self.machine(hw)
            self.assertEqual(
                {k: c[k] for k in ("card", "transport", "amps", "sense", "speaker_pcm", "sense_pcm", "plugin", "alsa_format")},
                {"card": "AppleJ998", "transport": "aop-spkr", "amps": 4, "sense": 8, "speaker_pcm": 6, "sense_pcm": 12,
                 "plugin": "4ch-vi", "alsa_format": "S24_LE"},
            )
            # the amps' lanes are the first ones of each PCM
            self.assertEqual([a["slot"] for a in w["amps"]], [0, 1, 2, 3])
            self.assertEqual([s["pcm"] for s in w["sense"]], list(range(8)))
            self.assertEqual([s["kind"] for s in w["sense"]], ["I", "V"] * 4)

    def test_the_specific_compatible_is_the_fallbacks_transport(self):
        want, _ = self.machine(self.hw())
        h = self.hw()
        h["links"][0]["cpu_compatible"] = [self.CPU[1:]]
        self.assertEqual(self.machine(h)[0], want)
        h["links"][0]["cpu_compatible"] = [self.CPU[:1]]
        self.assertEqual(self.machine(h)[0], want)

    def test_other_transports_are_as_wide_as_their_amps(self):
        for m in gen.machines():
            c, _ = gen.machine(m)
            f = fx(m)
            self.assertEqual((c["speaker_pcm"], c["sense_pcm"]),
                             (c["amps"], len(wiring.derive(f["hw"], f["tuning"])["sense"])), m)
        # the six-amp Macs fill the speaker service's PCMs
        c, _ = gen.machine("j616")
        self.assertEqual((c["transport"], c["speaker_pcm"], c["sense_pcm"]), ("aop-spkr", 6, 12))

    def test_a_unit_without_sense_ports_still_has_a_sense_pcm(self):
        hw = self.hw()
        w = wiring.derive(hw, fx("j413")["tuning"])
        w["unit_kind"], w["sense_companion"] = "lsm2", False
        c = wiring.describe(hw, w)
        # the graph has no sense node; the self-check still captures twelve channels
        self.assertEqual((c["sense"], c["plugin"], c["speaker_pcm"], c["sense_pcm"]), (0, "4ch", 6, 12))

    def test_more_amps_than_channels_is_refused(self):
        t = wiring.TRANSPORTS["apple,t6041-aop-audio"]
        self.assertEqual(wiring.pcm_widths(t, 6, 12), (6, 12))
        with self.assertRaises(wiring.Refused) as cm:
            wiring.pcm_widths(t, 8, 16)
        self.assertIn("8 amps on a 6-channel speaker PCM", str(cm.exception))
        with self.assertRaises(wiring.Refused) as cm:
            wiring.pcm_widths(dict(t, speaker_channels=8), 7, 14)
        self.assertIn("14 sense lanes on a 12-channel sense PCM", str(cm.exception))
        self.assertEqual(wiring.pcm_widths(wiring.TRANSPORTS["apple,mca"], 4, 8), (4, 8))
        self.assertEqual(wiring.pcm_widths(wiring.TRANSPORTS["apple,t8140-aop-audio"], 2, 0), (2, 0))

    def test_the_graph_opens_the_pcms_at_their_width(self):
        c, w = self.machine(self.hw())
        conf = gen.pipewire_conf("j998", c, (w["unit_channels"], w["sense_ports"], w["hash"]))

        def node(name):
            body = conf.split(f'"{name}"')[1]
            return tuple(re.search(rf"audio\.{k}\s*=\s*(.+)", body).group(1).strip() for k in ("channels", "position"))

        n = gen.names("j998")
        self.assertEqual(node(n["speaker"]), ("6", "[ AUX0 AUX1 AUX2 AUX3 AUX4 AUX5 ]"))
        self.assertEqual(node(n["sense"]), ("12", "[ " + " ".join(f"AUX{k}" for k in range(12)) + " ]"))
        # the protect stage itself keeps one port per amp and per V/I lane
        self.assertIn("audio.channels     = 12\n", conf)
        self.assertIn('inputs  = [ "sgv2:in_0" "sgv2:in_1" "sgv2:in_2" "sgv2:in_3" ' + " ".join(
            f'"sgv2:sense_{p}"' for p in w["sense_ports"]) + " ]", conf)

    def test_the_self_check_opens_the_pcms_at_their_width(self):
        c, _ = self.machine(self.hw())
        self.assertEqual(sgsc.pcm_widths(c), (6, 12))
        self.assertIsNone(sgsc.pcm_widths(None))
        st = sgsc.Streams(None, "AppleJ998", 1, 2, 4, 48000, sgsc.pcm_widths(c))
        self.assertEqual(st.frame_bytes, 24)
        with mock.patch.object(sgsc.subprocess, "Popen") as popen:
            st._play("S24_LE")
        argv = popen.call_args[0][0]
        self.assertEqual(argv[argv.index("-c") + 1], "6")
        # without a description (no transport): as wide as the amps
        st = sgsc.Streams(None, "AppleJ998", 1, 2, 4, 48000)
        self.assertEqual((st.spk_ch, st.sense_ch, st.frame_bytes), (4, 8, 16))

    def test_a_description_without_the_widths_generates_the_same_graph(self):
        # a passing self-check record written before the description carried
        # the widths is reused (same kernel and parameters): its Mac's PCMs
        # are as wide as its amps
        for m in gen.machines():
            c, w = gen.machine(m)
            old = {k: v for k, v in c.items() if k not in ("speaker_pcm", "sense_pcm")}
            self.assertEqual(gen.pipewire_conf(m, old, w), gen.pipewire_conf(m, c, w), m)

    def test_only_the_amps_lanes_are_judged(self):
        import struct

        frames = 480
        raw = b"".join(struct.pack("<12h", *([f % 97 + k for k in range(8)] + [0, 0, f, -f])) for f in range(frames))
        st = sgsc.amp_lanes(sgsc.stats(raw, 12), 4)
        self.assertEqual((st["frames"], len(st["ch"])), (frames, 8))
        self.assertEqual(sgsc.classify(st), ["live"] * 8)
        self.assertIsNone(sgsc.amp_lanes(sgsc.stats(b"", 12), 4))
        self.assertEqual(sgsc.amp_lanes(sgsc.stats(raw[: 16 * frames], 8), 4)["ch"], sgsc.stats(raw[: 16 * frames], 8)["ch"])


class AudioStack(unittest.TestCase):
    """wiring.audio_stack: the speaker card, the nodes its links point at and
    the loaded sound modules, nothing else."""

    @staticmethod
    def be(*w):
        import struct
        return struct.pack(">%dI" % len(w), *w)

    def tree(self, d):
        import struct
        root, mods = os.path.join(d, "dt"), os.path.join(d, "module")

        def node(path, props):
            p = os.path.join(root, path)
            os.makedirs(p, exist_ok=True)
            for k, v in props.items():
                with open(os.path.join(p, k), "wb") as f:
                    f.write(v)

        be = self.be
        node("", {"compatible": b"apple,j615\0"})
        node("soc/i2s@2a3400000", {"compatible": b"apple,t8122-mca\0apple,mca\0", "phandle": be(1),
                                   "#sound-dai-cells": be(1)})
        node("soc/i2c@0/codec@38", {"compatible": b"ti,sn012776\0", "phandle": be(2)})
        node("soc/gpu@0", {"compatible": b"apple,agx\0", "phandle": be(3)})
        node("sound", {"compatible": b"apple,macaudio\0"})
        node("sound/dai-link@0", {"link-name": b"Speakers\0"})
        node("sound/dai-link@0/cpu", {"sound-dai": be(1, 0)})
        node("sound/dai-link@0/codec", {"sound-dai": be(2)})
        for m, v in (("snd_soc_apple_mca", b"\xaa" * 20), ("snd_soc_macaudio", b"\xbb" * 20), ("nvme", b"\xcc" * 20)):
            os.makedirs(os.path.join(mods, m, "notes"))
            with open(os.path.join(mods, m, "notes", ".note.gnu.build-id"), "wb") as f:
                f.write(b"\x04\0\0\0\x14\0\0\0\x03\0\0\0GNU\0" + v)
        return root, mods

    def test_what_changes_the_fingerprint(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            root, mods = self.tree(d)
            be = self.be

            def fp():
                return wiring.audio_stack("sound", root, mods)
            base = fp()
            self.assertRegex(base, r"^[0-9a-f]{16}$")
            self.assertEqual(fp(), base)
            # unrelated: another node, another module
            with open(os.path.join(root, "soc/gpu@0/status"), "wb") as f:
                f.write(b"okay\0")
            with open(os.path.join(mods, "nvme", "notes", ".note.gnu.build-id"), "ab") as f:
                f.write(b"\xdd")
            self.assertEqual(fp(), base)
            # the serializer node a link points at (the J615's receiver delay)
            with open(os.path.join(root, "soc/i2s@2a3400000/apple,rx-delay-bclks"), "wb") as f:
                f.write(bytes.fromhex("00000001000000010000000000000000"))
            with_prop = fp()
            self.assertNotEqual(with_prop, base)
            # a rebuilt sound module under the same kernel release (its
            # build ID; srcversion where the module has one)
            with open(os.path.join(mods, "snd_soc_apple_mca", "notes", ".note.gnu.build-id"), "r+b") as f:
                f.seek(16)
                f.write(b"\xee" * 20)
            rebuilt_id = fp()
            self.assertNotEqual(rebuilt_id, with_prop)
            with open(os.path.join(mods, "snd_soc_apple_mca", "srcversion"), "w") as f:
                f.write("EEEE\n")
            self.assertNotEqual(fp(), rebuilt_id)
            rebuilt = fp()
            self.assertNotEqual(rebuilt, with_prop)
            # an amp node a link points at
            with open(os.path.join(root, "soc/i2c@0/codec@38/ti,imon-slot-no"), "wb") as f:
                f.write(be(4))
            self.assertNotEqual(fp(), rebuilt)
            self.assertEqual(wiring.audio_stack("nowhere", root, mods), "-")

    def test_the_bind_carries_it(self):
        rec = {"schema": 1, "version": 1, "result": "pass", "board": "j615", "kernel": "k", "params_sha256": "ab",
               "wiring": "a46e56034fa096f0", "audio": "0123456789abcdef", "card": "AppleJ615", "amps": 6, "sense": 12,
               "family": "sn012776", "key": "j615", "method": "derived", "reasons": []}
        self.assertIn("audio=0123456789abcdef", sgsc.bind_lines(rec).splitlines())
        del rec["audio"]
        self.assertIn("audio=-", sgsc.bind_lines(rec).splitlines())


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


    def test_a_record_carries_only_the_gate_cap_never_the_volume(self):
        # no cap is no line; a cap (0 dB included) is volume_cap_cdb; the
        # volume itself is the card's ceiling, so no record names it
        rec = {"schema": 1, "version": 3, "result": "pass", "board": "j873g", "kernel": "k", "params_sha256": "ab",
               "wiring": "d1d62e8a0aaaef3d", "card": "AppleJ873", "amps": 1, "sense": 0, "family": "sn012776",
               "key": "j873", "method": "derived-nosense", "reasons": []}
        for r in (rec, dict(rec, volume_cap_cdb=None)):
            self.assertFalse(any(l.startswith("volume_") for l in sgsc.bind_lines(r).splitlines()))
        for cap in (0, -1600):
            lines = sgsc.bind_lines(dict(rec, volume_cap_cdb=cap)).splitlines()
            self.assertEqual([l for l in lines if l.startswith("volume_")], [f"volume_cap_cdb={cap}"])


class OperatingVolumeCap(unittest.TestCase):
    """gate.json operating_volume_cap_db: a cap that can only lower the
    ceiling the card reports under the grant, never a volume of its own.
    No board, the J873g included, carries one: its -16 dB is the card's."""

    def test_shipped_gate_names_no_board(self):
        self.assertEqual(gen.load_gate()["operating_volume_cap_db"], {})
        self.assertNotIn("operating_volume_db", gen.load_gate())
        for board in ("j316s", "j413", "j616s", "j700", "j773g", "j813", "j815", "j873g"):
            self.assertIsNone(gen.operating_volume_cap_db(board), board)

    def test_malformed_values_refuse_the_board(self):
        for v in (1.0, 0.5, -40.5, True, "-16", [-16]):
            with self.assertRaises(wiring.Refused, msg=repr(v)):
                gen.operating_volume_cap_db("j873g", {"operating_volume_cap_db": {"j873g": v}})
        with self.assertRaises(wiring.Refused):
            gen.operating_volume_cap_db("j873g", {"operating_volume_cap_db": [-16]})
        self.assertIsNone(gen.operating_volume_cap_db("j873g", {}))
        self.assertIsNone(gen.operating_volume_cap_db("j873g", {"operating_volume_cap_db": {"j873g": None}}))
        self.assertEqual(gen.operating_volume_cap_db("j873g", {"operating_volume_cap_db": {"j873g": -40}}), -40.0)
        self.assertEqual(gen.operating_volume_cap_db("j873g", {"operating_volume_cap_db": {"j873g": 0}}), 0.0)

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


class SenseOpen(unittest.TestCase):
    def test_checks_applied_pcm_geometry_before_taking_the_grant(self):
        st = sgsc.Streams(None, "AppleK114c", 1, 2, 6, 48000)
        good = "format: S16_LE\nchannels: 12\nrate: 48000 (48000/1)\nperiod_size: 2730\n"
        for line, replacement in (
            ("S16_LE", "S32_LE"),
            ("channels: 12", "channels: 10"),
            ("48000 (48000/1)", "44100 (44100/1)"),
            ("period_size: 2730", "period_size: 0"),
            (good, "closed\n"),
        ):
            with self.subTest(replacement=replacement), \
                    mock.patch("builtins.open", mock.mock_open(read_data=good.replace(line, replacement))), \
                    mock.patch.object(sgsc.subprocess, "Popen"), \
                    mock.patch.object(sgsc.threading, "Thread"), \
                    mock.patch.object(st, "_owned", return_value=True), \
                    mock.patch.object(sgsc, "Lease") as lease:
                with self.assertRaisesRegex(wiring.Refused, "sense PCM hw_params"):
                    st.start(3)
                lease.assert_not_called()

        # The applied size need not equal the requested period or the graph
        # quantum: a valid 2730-frame capture can proceed to the silent grant.
        with mock.patch("builtins.open", mock.mock_open(read_data=good)) as proc, \
                mock.patch.object(sgsc.subprocess, "Popen"), \
                mock.patch.object(sgsc.threading, "Thread"), \
                mock.patch.object(st, "_owned", return_value=True), \
                mock.patch.object(sgsc, "Lease", side_effect=RuntimeError("ready for grant")):
            with self.assertRaisesRegex(RuntimeError, "ready for grant"):
                st.start(3)
            proc.assert_called_once_with("/proc/asound/card3/pcm2c/sub0/hw_params")


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

    def run_sc(self, *argv, kernel="7.3.0-test", audio="a1a1a1a1a1a1a1a1", card=None, live=True):
        def active(idx, card, w, report, args=None):
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
                mock.patch.object(sgsc, "audio_of", return_value=audio), \
                mock.patch.object(sgsc, "find_card", return_value=(0, card or self.desc["card"])), \
                mock.patch.object(sgsc.wiring, "live_card", return_value=(0, card or self.desc["card"]) if live else None), \
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

    def test_a_changed_audio_stack_is_checked_again(self):
        # J615: the same kernel release with the stock MCA module (sense one
        # bit clock late) and with the fixed one. A pass bound to one never
        # covers the other.
        self.assertEqual(self.run_sc("run"), 0)
        self.assertEqual(self.run_sc("pending", "--clear"), 0)
        self.assertEqual(self.bind()["audio"], "a1a1a1a1a1a1a1a1")
        self.assertEqual(self.run_sc("run", "--defer"), 0)
        self.assertEqual(self.calls, ["active"])
        self.assertEqual(self.run_sc("pending"), 1)
        self.assertEqual(self.run_sc("run", "--defer", audio="b2b2b2b2b2b2b2b2"), 0)
        self.assertEqual(self.run_sc("pending"), 0, "the switch test is due again")
        self.assertEqual(self.calls, ["active"])
        self.assertEqual(self.run_sc("run", audio="b2b2b2b2b2b2b2b2"), 0)
        self.assertEqual(self.calls, ["active", "active"])
        self.assertEqual(self.bind()["audio"], "b2b2b2b2b2b2b2b2")

    def test_a_changed_card_id_is_checked_again(self):
        # The graph opens hw:CARD=<id>: a pass bound to one card id is not
        # reused when the card bound to the sound node has another, and the
        # new record, its machine description and the generated graph all
        # carry the running card's id.
        import gen
        self.assertEqual(self.run_sc("run"), 0)
        self.assertEqual(self.run_sc("pending", "--clear"), 0)
        self.assertEqual(self.bind()["card"], self.desc["card"])
        self.assertEqual(self.run_sc("run", "--defer", card="AppleJ413Renamed"), 0)
        self.assertEqual(self.run_sc("pending"), 0, "the switch test is due again")
        self.assertEqual(self.calls, ["active"])
        self.assertEqual(self.run_sc("run", card="AppleJ413Renamed"), 0)
        self.assertEqual(self.calls, ["active", "active"])
        self.assertEqual(self.bind()["card"], "AppleJ413Renamed")
        with open(os.path.join(self.state, "selfcheck.json")) as f:
            report = json.load(f)
        self.assertEqual(report["record"]["card"], "AppleJ413Renamed")
        self.assertEqual(report["machine"]["card"], "AppleJ413Renamed")
        w = report["wiring"]
        conf = gen.pipewire_conf(w["key"], dict(report["machine"], board=report["record"]["board"]),
                                 (w["unit_channels"], w["sense_ports"], w["hash"]), "test", voicing_pre=False)
        self.assertIn("hw:CARD=AppleJ413Renamed", conf)
        self.assertNotIn(f"hw:CARD={self.desc['card']},", conf)
        # a card not bound yet (an AOP card after this unit) cannot
        # contradict the record: the pass is reused
        self.assertEqual(self.run_sc("pending", "--clear"), 0)
        self.assertEqual(self.run_sc("run", "--defer", live=False), 0)
        self.assertEqual(self.run_sc("pending"), 1)
        self.assertEqual(self.calls, ["active", "active"])

    def test_a_pass_without_an_audio_stack_is_not_reused(self):
        # a record from before the audio stack was bound
        self.assertEqual(self.run_sc("run"), 0)
        path = os.path.join(self.state, "selfcheck.bind")
        with open(path) as f:
            old = "".join(l for l in f if not l.startswith("audio="))
        with open(path, "w") as f:
            f.write(old)
        self.assertEqual(self.run_sc("run"), 0)
        self.assertEqual(self.calls, ["active", "active"])

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


class DumpRefusalReason(unittest.TestCase):
    """A strict dump that the reader refuses names the reason (the journal
    is all a user on a muted Mac sees)."""

    def run_dump(self, rc, stdout, stderr=""):
        r = mock.Mock(returncode=rc, stdout=stdout, stderr=stderr)
        with mock.patch.object(wiring.subprocess, "run", return_value=r):
            with self.assertRaises(wiring.Refused) as e:
                wiring.tuning_dump(["bundle"], strict=True)
        return str(e.exception)

    def test_load_error_diagnostics(self):
        out = json.dumps({"items": [
            {"severity": "warning", "code": "w", "context": "x", "message": "ignored"},
            {"severity": "error", "code": "load", "context": "DSP database",
             "message": "speaker ID unknown (no speaker_id in the bundle manifest and no /chosen/asahi,speaker-id)"},
        ]})
        m = self.run_dump(1, out)
        self.assertIn("(exit 1): load: speaker ID unknown (no speaker_id", m)
        self.assertNotIn("ignored", m)

    def test_errors_in_a_loaded_dump(self):
        out = json.dumps({"tuning": {}, "diagnostics": {"items": [
            {"severity": "error", "code": "spp3-channel-count", "context": "x", "message": "2 != 4"}]}})
        self.assertIn("spp3-channel-count: 2 != 4", self.run_dump(1, out))

    def test_no_json_falls_back_to_stderr(self):
        self.assertIn("(exit 1): boom", self.run_dump(1, "", "line\nboom\n"))


class Capabilities(unittest.TestCase):
    """The capability probe (wiring.probe): every value the description
    uses comes from the machine, says where from, and the board is an
    identity only."""

    PREFIXES = ("dt:", "alsa:", "driver:", "tuning:", "derived:")

    def test_every_capability_has_a_machine_source(self):
        for m in gen.machines():
            f = fx(m)
            w = wiring.derive(f["hw"], f["tuning"])
            rep = wiring.caps_report(wiring.probe(f["hw"], None, w))
            for k, v in rep.items():
                self.assertTrue(v["source"].startswith(self.PREFIXES), f"{m} {k}: {v['source']!r}")
            for k in ("transport", "amps", "family", "sense_path", "card", "speaker_format", "unit", "rate"):
                self.assertIn(k, rep, m)

    def test_the_board_selects_nothing(self):
        # the same hardware and tuning under another board name: every
        # capability but the identity is the same, and so is the description
        for m in gen.machines():
            f = fx(m)
            w = wiring.derive(f["hw"], f["tuning"])
            h = copy.deepcopy(f["hw"])
            h["board"] = "j999"
            h["compatible"] = ["apple,j999"] + h["compatible"][1:]
            a = wiring.caps_report(wiring.probe(f["hw"], None, w))
            b = wiring.caps_report(wiring.probe(h, None, w))
            a.pop("board"), b.pop("board")
            self.assertEqual(a, b, m)
            da, db = wiring.describe(f["hw"], w), wiring.describe(h, w)
            self.assertEqual(db.pop("board"), "j999")
            da.pop("board")
            self.assertEqual(da, db, m)

    def test_sense_path_from_the_back_ends_dma_names(self):
        f = j773g()
        c = wiring.probe(f["hw"])
        self.assertFalse(c["sense_path"].value)
        self.assertIn("'sense'", c["sense_path"].source)
        self.assertEqual(c["sense_slots"].value, 0)
        f = j773g(("spkr", "sense", "cout", "cin"))
        c = wiring.probe(f["hw"])
        self.assertTrue(c["sense_path"].value)
        self.assertEqual(c["sense_slots"].value, 2)
        # no dma-names: the family's sense stands, to be proven on the card
        c = wiring.probe(j773g(None)["hw"])
        self.assertTrue(c["sense_path"].value)
        self.assertIn("silent sense test", c["sense_path"].source)
        # dma-names that name neither channel: the same, and the source
        # does not claim the sense channel is named
        c = wiring.probe(j773g(("tx", "rx"))["hw"])
        self.assertTrue(c["sense_path"].value)
        self.assertIn("silent sense test", c["sense_path"].source)
        self.assertNotIn("(names", c["sense_path"].source)

    def test_driver_capabilities_by_family(self):
        self.assertEqual(wiring.probe(fx("j316")["hw"])["fault_status"].value, True)
        self.assertEqual(wiring.probe(fx("j293")["hw"])["fault_status"].value, False)
        j700 = wiring.probe(fx("j700")["hw"])
        self.assertEqual((j700["gain_control"].value, j700["fault_status"].value), (False, False))
        self.assertEqual(j700["sense_path"], wiring.Cap(False, "driver:max98360a returns no V/I sense"))

    def test_unknown_blocks_refuse(self):
        f = fx("j413")
        h = copy.deepcopy(f["hw"])
        next(l for l in h["links"] if l["name"] in ("Speaker", "Speakers"))["cpu_compatible"] = [["vendor,new-dai"]] * 2
        with self.assertRaises(wiring.Refused):
            wiring.probe(h)
        h = copy.deepcopy(f["hw"])
        for c in next(l for l in h["links"] if l["name"] in ("Speaker", "Speakers"))["codecs"]:
            c["compatible"] = ["vendor,new-amp"]
        with self.assertRaises(wiring.Refused):
            wiring.probe(h)

    def _sysfs(self, d, dt, card_of, idx=1, cid="AppleJ413Real"):
        sysfs, proc = os.path.join(d, "sys"), os.path.join(d, "proc")
        os.makedirs(os.path.join(sysfs, f"card{idx}", "device"))
        os.symlink(os.path.join(dt, card_of), os.path.join(sysfs, f"card{idx}", "device", "of_node"))
        open(os.path.join(sysfs, f"card{idx}", f"controlC{idx}"), "w").close()
        os.makedirs(os.path.join(proc, f"card{idx}"))
        with open(os.path.join(proc, f"card{idx}", "id"), "w") as fh:
            fh.write(cid + "\n")
        return sysfs, proc

    def test_the_running_cards_own_id(self):
        import tempfile

        f = fx("j413")
        with tempfile.TemporaryDirectory() as d:
            dt = os.path.join(d, "dt")
            os.makedirs(dt)
            fake_dt(dt, f["hw"], jack_card=True)
            hw = wiring.read_dt(dt, sysfs=os.path.join(d, "none"))
            # a card on another node (the jack card) is not the speaker card
            sysfs, proc = self._sysfs(d, dt, "sound-jack", idx=0, cid="AppleJack")
            self.assertIsNone(wiring.live_card(hw, dt, sysfs, proc))
            os.makedirs(os.path.join(sysfs, "card1", "device"))
            os.symlink(os.path.join(dt, "sound"), os.path.join(sysfs, "card1", "device", "of_node"))
            open(os.path.join(sysfs, "card1", "controlC1"), "w").close()
            os.makedirs(os.path.join(proc, "card1"))
            with open(os.path.join(proc, "card1", "id"), "w") as fh:
                fh.write("AppleJ413Real\n")
            live = wiring.live_card(hw, dt, sysfs, proc)
            self.assertEqual(live, (1, "AppleJ413Real"))
            w = wiring.derive(hw, f["tuning"])
            c = wiring.probe(hw, live, w)
            self.assertEqual(c["card"].value, "AppleJ413Real")
            self.assertTrue(c["card"].source.startswith("alsa:card1"))
            self.assertEqual(wiring.describe(hw, w, live)["card"], "AppleJ413Real")
            # offline, macaudio's name from the model
            self.assertEqual(wiring.describe(hw, w)["card"], wiring.card_id(hw))
            self.assertTrue(wiring.probe(hw)["card"].source.startswith("dt:"))
        self.assertIsNone(wiring.live_card(hw, "/nonexistent", "/nonexistent"))

    def test_a_card_still_registering_is_not_bound_yet(self):
        # snd_card_register() adds the card device (its id and of_node link)
        # first, the control device controlCN after the card's other
        # devices, and the card's proc entries last. Until both are there
        # the card is "not bound yet" (None): provisioning keeps waiting and
        # the self-check keeps its record. The sysfs id is read first; an id
        # that exists but cannot be read refuses with its path.
        import tempfile

        f = fx("j413")
        with tempfile.TemporaryDirectory() as d:
            dt = os.path.join(d, "dt")
            os.makedirs(dt)
            fake_dt(dt, f["hw"])
            hw = wiring.read_dt(dt, sysfs=os.path.join(d, "none"))
            sysfs, proc = self._sysfs(d, dt, "sound", idx=1)
            with open(os.path.join(sysfs, "card1", "id"), "w") as fh:
                fh.write("AppleJ413Sys\n")
            self.assertEqual(wiring.live_card(hw, dt, sysfs, proc), (1, "AppleJ413Sys"))
            self.assertTrue(wiring.card_registered(os.path.join(sysfs, "card1"), proc))
            # no proc entries yet (the id in sysfs is already there)
            os.rename(os.path.join(proc, "card1", "id"), os.path.join(d, "proc-id"))
            self.assertFalse(wiring.card_registered(os.path.join(sysfs, "card1"), proc))
            self.assertIsNone(wiring.live_card(hw, dt, sysfs, proc))
            # nor the control device: the Neo's window (speakerguardd#235)
            os.unlink(os.path.join(sysfs, "card1", "controlC1"))
            os.rename(os.path.join(d, "proc-id"), os.path.join(proc, "card1", "id"))
            self.assertFalse(wiring.card_registered(os.path.join(sysfs, "card1"), proc))
            self.assertIsNone(wiring.live_card(hw, dt, sysfs, proc))
            # a kernel without ALSA's proc interface: the control device decides
            self.assertFalse(wiring.card_registered(os.path.join(sysfs, "card1"), os.path.join(d, "no-proc")))
            open(os.path.join(sysfs, "card1", "controlC1"), "w").close()
            self.assertTrue(wiring.card_registered(os.path.join(sysfs, "card1"), os.path.join(d, "no-proc")))
            self.assertEqual(wiring.live_card(hw, dt, sysfs, proc), (1, "AppleJ413Sys"))
            os.unlink(os.path.join(sysfs, "card1", "id"))
            os.makedirs(os.path.join(sysfs, "card1", "id"))
            with self.assertRaises(wiring.Refused) as e:
                wiring.live_card(hw, dt, sysfs, proc)
            self.assertIn(os.path.join(sysfs, "card1", "id"), str(e.exception))

    def test_back_end_nodes_are_recorded_for_the_report(self):
        import tempfile

        f = fx("j316")
        with tempfile.TemporaryDirectory() as d:
            fake_dt(d, f["hw"])
            hw = wiring.read_dt(d, sysfs=os.path.join(d, "none"))
        link = next(l for l in hw["links"] if l["name"] in ("Speaker", "Speakers"))
        self.assertEqual(len(link["cpu_nodes"]), link["cpus"])
        self.assertIn(link["cpu_nodes"][0], wiring.probe(hw)["transport"].source)

    def test_caps_command(self):
        import contextlib
        import io
        import tempfile

        f = fx("j616")
        with tempfile.TemporaryDirectory() as d:
            hwp, tp = os.path.join(d, "hw.json"), os.path.join(d, "t.json")
            json.dump(f["hw"], open(hwp, "w"))
            json.dump(f["tuning"], open(tp, "w"))
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(wiring.main(["caps", "--hw", hwp]), 0)
            rep = json.loads(out.getvalue())
            self.assertEqual(rep["transport"]["value"], "aop-spkr")
            self.assertNotIn("unit", rep)
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(wiring.main(["caps", "--hw", hwp, "--tuning-json", tp]), 0)
            self.assertIn("unit", json.loads(out.getvalue()))
            # --live with no card bound to the node refuses (exit 2)
            with mock.patch.object(wiring, "live_card", return_value=None), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(wiring.main(["caps", "--hw", hwp, "--live"]), 2)


if __name__ == "__main__":
    unittest.main()
