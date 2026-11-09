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
    ("AID5/7 Tone3 quieter only", "tone_16000_0", 2, -0.089, -4.84, "refused"),
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

    def test_nothing_measured_is_refused(self):
        self.assertEqual(attest.verdict([])["verdict"], "refused")


if __name__ == "__main__":
    unittest.main()
