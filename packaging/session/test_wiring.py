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

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen  # noqa: E402
import wiring  # noqa: E402

_loader = importlib.machinery.SourceFileLoader("sgsc", os.path.join(HERE, "speakerguardd-selfcheck"))
_spec = importlib.util.spec_from_loader("sgsc", _loader)
sgsc = importlib.util.module_from_spec(_spec)
_loader.exec_module(sgsc)

with open(os.path.join(HERE, "machines.json")) as _f:
    MACHINES = json.load(_f)


def fx(m):
    return copy.deepcopy(gen.fixture(m))


class Derivation(unittest.TestCase):
    def test_the_derived_maps_are_the_hand_maps(self):
        for m in ("j316", "j413"):
            f = fx(m)
            w = wiring.derive(f["hw"], f["tuning"])
            hand = gen.channel_map(m, MACHINES[m])
            self.assertEqual((w["unit_channels"], w["sense_ports"]), hand, m)
            eff, dis = wiring.apply_override(w, MACHINES[m])
            self.assertEqual(dis, [], m)
            self.assertEqual(eff["hash"], w["hash"], m)

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

    def test_no_machines_json_still_derives(self):
        # a new board: no entry, nothing hand-written
        for m in ("j316", "j413"):
            f = fx(m)
            f["hw"]["board"] = "j999"
            w = wiring.derive(f["hw"], f["tuning"])
            self.assertEqual(wiring.machine_entry({}, "j999"), (None, None))
            self.assertEqual((w["unit_channels"], w["sense_ports"]), gen.channel_map(m, MACHINES[m]))

    def test_b6_bug_identity_order_is_caught(self):
        f = fx("j316")
        w = wiring.derive(f["hw"], f["tuning"])
        bad = dict(MACHINES["j316"], unit_channels=[0, 1, 2, 3, 4, 5])
        eff, dis = wiring.apply_override(w, bad)
        self.assertTrue(any("unit_channels" in d for d in dis), dis)
        errs = wiring.role_errors(eff, wiring.roles(f["tuning"])["channels"])
        self.assertEqual(len(errs), 4, errs)
        self.assertTrue(any("Left Tweeter" in e and "L woofer" in e for e in errs), errs)
        with self.assertRaises(SystemExit):
            gen.wiring_for("j316", bad)

    def test_b6_bug_vi_order_is_caught(self):
        f = fx("j413")
        w = wiring.derive(f["hw"], f["tuning"])
        bad = dict(MACHINES["j413"], sense_pairs="vi")
        eff, dis = wiring.apply_override(w, bad)
        self.assertTrue(any("sense_ports" in d for d in dis), dis)
        # the roles are right: only the sense check sees this one
        self.assertEqual(wiring.role_errors(eff, wiring.roles(f["tuning"])["channels"]), [])
        with self.assertRaises(SystemExit):
            gen.wiring_for("j413", bad)

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
        w = wiring.derive(hw, {"tuning": {"protection": {"kind": "lsm2", "lanes": lanes}}})
        # one back-end: slots in dai-link order, so channel 0 is the right amp
        self.assertEqual([a["prefix"] for a in w["amps"]], ["Right Speaker", "Left Speaker"])
        self.assertEqual(w["unit_channels"], [1, 0])
        self.assertEqual(w["sense"], [])

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

    def run_check(self, w, truth):
        switches = {}
        n = len(w["amps"])

        class FakeStreams:
            def __init__(self, *a):
                self.fmt = "S24_LE"

            def run(self, idx):
                quiet, pattern = [], []
                for (prefix, kind), on in switches.items():
                    if on == "off":
                        ch = truth[(prefix, kind)]
                        (pattern if kind == "V" else quiet).append(ch)
                return capture(2 * n, quiet, pattern)

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

    def test_a_vi_override_fails_on_every_channel(self):
        eff, _ = wiring.apply_override(self.w, dict(MACHINES["j413"], sense_pairs="vi"))
        errs = self.run_check(eff, self.truth)
        self.assertEqual(len(errs), 8, errs)
        self.assertIn("Left Woofer VSENSE off quietened sense channel(s) [1]; the map has its V on channel 0", errs)

    def test_a_dead_or_shared_channel_fails(self):
        truth = dict(self.truth)
        truth[("Right Tweeter", "I")] = truth[("Left Tweeter", "I")]
        errs = self.run_check(self.w, truth)
        self.assertTrue(any("Right Tweeter ISENSE" in e for e in errs), errs)

    def test_j616s_aop_sense_check(self):
        f = fx("j616")
        w = wiring.derive(f["hw"], f["tuning"])
        truth = {}
        for p, a in enumerate(w["amps"]):
            truth[(a["prefix"], "I")] = 2 * p
            truth[(a["prefix"], "V")] = 2 * p + 1
        self.assertEqual(self.run_check(w, truth), [])
        old = {k: v for k, v in MACHINES["j616"].items() if k != "sense_pairs"}
        eff, _ = wiring.apply_override(w, old)
        self.assertEqual(len(self.run_check(eff, truth)), 12)

    def test_classifier_on_the_b6_captures(self):
        # live 49-82 distinct values, V off 2-3, I off 1 (B6, J316 and J413)
        self.assertEqual(sgsc.classify({"ch": [{"distinct": 49}, {"distinct": 2}, {"distinct": 3}, {"distinct": 1}]}),
                         ["live", "quiet", "quiet", "quiet"])
        self.assertEqual(sgsc.classify({"ch": [{"distinct": 9}]}), ["unclear"])


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
    be = node("soc/backend", {"#sound-dai-cells": u32(1)})

    def card(name, links, model):
        node(name, {"compatible": strs("apple,test-macaudio", "apple,macaudio"), "model": strs(model)})
        for i, l in enumerate(links):
            d = f"{name}/dai-link@{i}"
            node(d, {"link-name": strs(l["name"])})
            node(d + "/cpu", {"sound-dai": u32(*[w for b in range(l["cpus"]) for w in (be, b)])})
            phs = []
            for c in l["codecs"]:
                props = {"sound-name-prefix": strs(c["prefix"]), "compatible": strs(*c["compatible"]), "#sound-dai-cells": u32(0)}
                for k in ("vmon", "imon"):
                    if k in c:
                        props[f"ti,{k}-slot-no"] = u32(c[k])
                phs.append(node(c["node"], props))
            node(d + "/codec", {"sound-dai": u32(*phs)})

    card("sound", hw["links"], hw["model"])
    if jack_card:
        jack = {"name": "Headphone Jack", "cpus": 1, "codecs": [{"node": "soc/jack", "prefix": "Jack", "compatible": ["cirrus,cs42l84"]}]}
        card("sound-jack", [jack], hw["model"] + " Jack")


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
            strip = lambda links: [[{k: v for k, v in c.items() if k not in ("node", "i2c")} for c in l["codecs"]] + [l["name"], l["cpus"]] for l in links]
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
        self.assertEqual((w["unit_channels"], w["sense_ports"]), gen.channel_map("j616", MACHINES["j616"]))
        # the template before 2c16d1c: no sense_pairs, so [V, I]
        old = {k: v for k, v in MACHINES["j616"].items() if k != "sense_pairs"}
        _, dis = wiring.apply_override(w, old)
        self.assertTrue(any("sense_ports" in d for d in dis), dis)
        with self.assertRaises(SystemExit):
            gen.wiring_for("j616", old)

    def test_j700_stereo_pair_from_one_node(self):
        f = fx("j700")
        w = wiring.derive(f["hw"], f["tuning"])
        self.assertEqual([(a["prefix"], a["slot"], a["side"]) for a in w["amps"]], [("Speaker L", 0, "L"), ("Speaker R", 1, "R")])
        self.assertTrue(all(a["by_strap"] for a in w["amps"]))
        self.assertEqual((w["unit_channels"], w["sense"]), ([0, 1], []))
        bad = dict(MACHINES["j700"], unit_channels=[1, 0])
        eff, dis = wiring.apply_override(w, bad)
        self.assertTrue(dis)
        self.assertEqual(len(wiring.role_errors(eff, wiring.roles(f["tuning"])["channels"])), 2)

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


if __name__ == "__main__":
    unittest.main()
