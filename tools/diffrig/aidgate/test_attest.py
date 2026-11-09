#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""attest.py verdicts on recorded gate cases (numbers only: no Apple data).

Each case is a worst cell the 25G83 gate measured (gate2 / attest.py, 2026-11-09)
on a tuning v2 then rendered louder or quieter than macOS. All but the lab
boards must be refused until their law is fixed and the gate measures them
within tolerance again.
"""
import os, sys, unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.dont_write_bytecode = True
import attest  # noqa: E402

# (case, segment, lane, louder dB, quieter dB, expected verdict)
CASES = [
    ("AID14 j414s mstk onset", "step_up_125", 0, 4.352, -1.06, "refused"),
    ("AID8 mstk onset", "step_up_125", 0, 4.352, -1.06, "refused"),
    ("AID3 mstk onset", "step_up_125", 0, 4.678, -1.80, "refused"),
    ("AID18 mstk onset", "step_up_125", 2, 4.750, -1.79, "refused"),
    ("AID19 burst", "burst_1000", 1, 21.001, -2.84, "refused"),
    ("AID2/6/12 Tone2", "tone_160_-12", 0, 12.311, -54.45, "refused"),
    ("AID26 Buzz", "step_down_2000", 0, 11.811, -51.79, "refused"),
    ("AID49 Tone2", "tone_4000_0", 1, 3.228, -4.59, "refused"),
    ("AID4 Comp", "burst_1000", 0, 1.685, -44.98, "refused"),
    ("main-band cell quieter only", "tone_16000_0", 2, -0.089, -4.84, "refused"),
    ("just over the louder bound", "pink_-6", 0, 0.251, 0.0, "refused"),
    ("lab AID9", "step_down_2000", 0, 0.120, -0.377, "pass"),
    ("lab AID13", "tone_20_-12", 0, 0.106, -0.247, "pass"),
    ("lab AID35 spid 0", "tone_4000_0", 0, 0.061, -0.251, "pass"),
    ("lab AID44", "tone_16000_0", 0, 0.002, -0.024, "pass"),
]


class Verdicts(unittest.TestCase):
    def test_cases(self):
        for name, seg, lane, louder, quieter, want in CASES:
            # the worst cell among ordinary ones
            rows = [{"seg": "pink_-30", "lane": 0, "louder": 0.01, "quieter": -0.01},
                    {"seg": seg, "lane": lane, "louder": louder, "quieter": quieter}]
            v = attest.verdict(rows)
            self.assertEqual(v["verdict"], want, f"{name}: {v}")
            if want == "refused":
                self.assertIn(seg, v["reason"])

    def test_floor_residue(self):
        # (delta dB, macOS RMS) measurements on a lane whose own peak is 0.5 (about -6 dBFS)
        peak = 0.5
        deep = peak * 10 ** (-64 / 20)      # 64 dB under the lane's peak
        edge = peak * 10 ** (-55 / 20)      # exactly at the edge: still main band
        main = peak * 10 ** (-40 / 20)
        ok = {"seg": "pink_-30", "lane": 0, "louder": 0.01, "quieter": -0.01, "floor_quieter": None,
              "chaotic": None}
        # quieter than macOS deep under the lane's peak: reported, not refused
        v = attest.verdict([ok, attest.cell("tone_16000_0", 2, [(-4.84, deep)], peak)])
        self.assertEqual(v["verdict"], "pass", v)
        self.assertEqual(v["floor_residue_cells"], 1)
        self.assertEqual(v["worst_floor_residue_db"], -4.84)
        self.assertIn("not refused", v["reason"])
        # louder than macOS is refused at any level
        v = attest.verdict([ok, attest.cell("tone_16000_0", 2, [(0.30, deep)], peak)])
        self.assertEqual(v["verdict"], "refused", v)
        # the same quieter delta in the main band is refused
        for lv in (main, edge):
            v = attest.verdict([ok, attest.cell("tone_16000_0", 2, [(-4.84, lv)], peak)])
            self.assertEqual(v["verdict"], "refused", v)
        # a cell with a deep quieter window and a main-band louder window keeps the louder one
        c = attest.cell("burst_1000", 1, [(-3.0, deep), (0.4, main)], peak)
        self.assertEqual((c["louder"], c["quieter"], c["floor_quieter"]), (0.4, 0.4, -3.0))
        self.assertEqual(attest.verdict([ok, c])["verdict"], "refused")

    def test_arithmetic_chaotic_cells(self):
        # (delta dB, macOS RMS) on a lane whose own peak is -26 dBFS; synthetic numbers only
        peak = 0.05
        deep = 10 ** (-70 / 20)            # -70 dBFS, 44 dB under the peak (not floor residue)
        shallow = 10 ** (-50 / 20)         # -50 dBFS: above the -60 dBFS ceiling
        near = 10 ** (-61 / 20)            # under -60 dBFS but only 35 dB under the peak
        ok = {"seg": "pink_-30", "lane": 0, "louder": 0.01, "quieter": -0.01, "floor_quieter": None,
              "chaotic": None}
        # louder by 0.30 dB in a deep cell where macOS itself moves by 0.40 dB: held, passes
        c = attest.cell("tone_20_-12", 0, [(0.30, deep)], peak, 0.40)
        v = attest.verdict([ok, c])
        self.assertEqual(v["verdict"], "pass", v)
        self.assertEqual(v["chaotic_cells"], 1)
        self.assertIn("macOS's own", v["reason"])
        # outside macOS's own spread: refused, louder or quieter
        for db in (0.45, -1.50):
            v = attest.verdict([ok, attest.cell("tone_20_-12", 0, [(db, deep)], peak, 0.40)])
            self.assertEqual(v["verdict"], "refused", (db, v))
        # the ceiling: main-band measurements get at most a 0.25 dB allowance whatever the spread
        for lv in (shallow, near):
            v = attest.verdict([ok, attest.cell("tone_20_-12", 0, [(0.30, lv)], peak, 0.40)])
            self.assertEqual(v["verdict"], "refused", (lv, v))
        # main band: at most the 0.25 dB cap, even where macOS's own spread is larger
        self.assertEqual(attest.MAIN_SPREAD_CAP_DB, 0.25)
        v = attest.verdict([ok, attest.cell("tone_20_-12", 0, [(0.26, shallow)], peak, 2.0)])
        self.assertEqual(v["verdict"], "refused", v)
        v = attest.verdict([ok, attest.cell("tone_20_-12", 0, [(0.24, shallow)], peak, 2.0)])
        self.assertEqual(v["verdict"], "pass", v)
        # no measured spread: ordinary rule
        v = attest.verdict([ok, attest.cell("tone_20_-12", 0, [(0.30, deep)], peak, None)])
        self.assertEqual(v["verdict"], "refused", v)
        # a spread under the tolerance changes nothing
        v = attest.verdict([ok, attest.cell("tone_20_-12", 0, [(0.20, deep)], peak, 0.10)])
        self.assertEqual(v["verdict"], "pass", v)
        self.assertNotIn("chaotic_cells", v)

    def test_nothing_measured_is_refused(self):
        self.assertEqual(attest.verdict([])["verdict"], "refused")


class Releases(unittest.TestCase):
    """One block per macOS release; a release is never inherited from another."""

    def test_merge_keeps_other_releases_of_the_same_law(self):
        import json, tempfile
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "parity.json")
            r = lambda b: {"build": b, "reference": f"macOS {b} Audio Units", "rev": "x", "entries": []}
            json.dump(attest.merge(p, "L1", r("25G83")), open(p, "w"))
            doc = attest.merge(p, "L1", r("26A428"))
            self.assertEqual([x["build"] for x in doc["releases"]], ["25G83", "26A428"])
            json.dump(doc, open(p, "w"))
            # re-gating one release replaces only that block
            self.assertEqual([x["build"] for x in attest.merge(p, "L1", r("26A428"))["releases"]], ["25G83", "26A428"])
            # other rendering code: every older block is stale and dropped
            self.assertEqual([x["build"] for x in attest.merge(p, "L2", r("26A428"))["releases"]], ["26A428"])


if __name__ == "__main__":
    unittest.main()
