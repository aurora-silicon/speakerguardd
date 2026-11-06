#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Per-box differential rig: one box of a Mac's speaker graph, macOS vs v2.

macOS side: the installed unit through `box_host` with the strip's saved
state. v2 side: `voicing-box-render` (aurora-speaker-tuning example), which
compiles the same saved state as provisioning does and runs v2's box.
Both render the same signals offline: steady sines on a third-octave grid
at several input levels and pink noise at several levels, identical on
every box channel. Per case and channel it reports the steady-state RMS of
each side (second half of the render) and v2 minus macOS in dB.

    box_rig.py --tunings DIR --aid N --box NAME --ch C --v2 PATH/voicing-box-render \
        --work DIR [--json OUT] [--levels 0,-6,-12,-24,-40] [--tol 0.05]

Exit 0 when v2 is at or below macOS + tol on every case and channel. Nothing
is played; outputs made with Apple tuning are Apple-derived (keep them out
of the repository; --work is deleted case by case).
"""

import argparse
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FS = 48000
FREQS = [25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250,
         1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000]
FACTORY = {"mstk": "AUMeisterStueckFactory", "tmst": "AUToneMeisterFactory", "vben": "AUVirtualBassFactory",
           "ldnm": "AULoudnessNormalizerFactory", "vrq2": "AUVariableEQV2Factory", "axtc": "AUXTCFactory",
           "cfrk": "AUControlFreakFactory", "buzk": "AUBuzzKillFactory", "cdly": "AUChannelSpecificDelayFactory",
           "volt": "AUVolumeTaperFactory", "flow": "AULowFlowFactory"}


def pink(n, seed=1):
    rng = np.random.default_rng(seed)
    f = np.fft.rfft(rng.standard_normal(n))
    k = np.arange(len(f), dtype=float)
    k[0] = 1.0
    p = np.fft.irfft(f / np.sqrt(k), n)
    return p / np.max(np.abs(p))


def rms_db(y):
    h = y[len(y) // 2:]
    return [float(20 * np.log10(np.sqrt(np.mean(h[:, c] ** 2)) + 1e-15)) for c in range(y.shape[1])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tunings", required=True)
    ap.add_argument("--aid", type=int, required=True)
    ap.add_argument("--box", required=True)
    ap.add_argument("--subtype", default="mstk")
    ap.add_argument("--slot", type=int, help="render only this tmst slot (all box gains and other slots off)")
    ap.add_argument("--ch", type=int, required=True)
    ap.add_argument("--v2", required=True)
    ap.add_argument("--work", required=True)
    ap.add_argument("--json")
    ap.add_argument("--strip", default="builtin_speaker_out_general.austrip")
    ap.add_argument("--levels", default="0,-6,-12,-24,-40")
    ap.add_argument("--seconds", type=float, default=1.0)
    ap.add_argument("--tol", type=float, default=0.05)
    a = ap.parse_args()

    os.makedirs(a.work, exist_ok=True)
    strip = os.path.join(a.tunings, f"AID{a.aid}", "DSP", "Strips", a.strip)
    n = int(FS * a.seconds)
    t = np.arange(n) / FS
    levels = [float(v) for v in a.levels.split(",")]
    cases = [(f"sine_{f:g}_{-l:g}", 10 ** (l / 20) * np.sin(2 * np.pi * f * t)) for f in FREQS for l in levels]
    pk = pink(n)
    cases += [(f"pink_{-l:g}", 10 ** (l / 20) * pk) for l in levels]

    rows, worst = [], -1e9
    fin, fmac, fv2 = (os.path.join(a.work, x) for x in ("in.f32", "mac.f32", "v2.f32"))
    for name, x in cases:
        np.repeat(x[:, None], a.ch, axis=1).astype("<f4").tofile(fin)
        mac_cmd = [os.path.join(HERE, "box_host"), "--strip", strip, "--stage", a.box,
                   "--subtype", a.subtype, "--factory", FACTORY[a.subtype], "--channels", str(a.ch),
                   "--in", fin, "--out", fmac, "--meta", os.path.join(a.work, "meta.json")]
        if a.slot is not None:
            for pid in (0, 1, 4, 11, 13):
                mac_cmd += ["--set", f"{pid}=0"]
            for slot in range(50):
                if slot != a.slot:
                    mac_cmd += ["--set", f"{14 + 10 * slot}=0"]
        r = subprocess.run(mac_cmd,
                           capture_output=True, text=True)
        if r.returncode:
            sys.exit(f"box_host: {r.stderr.strip()}")
        v2_cmd = [a.v2, "--tunings-root", a.tunings, "--aid", str(a.aid), "--box", a.box, "--ch", str(a.ch)]
        if a.slot is not None:
            v2_cmd += ["--only-slot", str(a.slot)]
        r = subprocess.run(v2_cmd + [fin, fv2], capture_output=True, text=True)
        if r.returncode:
            print(json.dumps({"aid": a.aid, "box": a.box, "refused": r.stderr.strip()}))
            return 1
        m = rms_db(np.fromfile(fmac, "<f4").reshape(-1, a.ch))
        v = rms_db(np.fromfile(fv2, "<f4").reshape(-1, a.ch))
        d = [vv - mm for vv, mm in zip(v, m)]
        worst = max(worst, max(d))
        rows.append({"case": name, "mac_db": m, "v2_db": v, "diff_db": d})
    for f in (fin, fmac, fv2):
        if os.path.exists(f):
            os.unlink(f)
    over = [r for r in rows if max(r["diff_db"]) > a.tol]
    summary = {"aid": a.aid, "box": a.box, "slot": a.slot, "cases": len(rows), "worst_over_db": worst,
               "min_diff_db": min(min(r["diff_db"]) for r in rows), "over": len(over),
               "worst_cases": sorted(rows, key=lambda r: -max(r["diff_db"]))[:5]}
    if a.json:
        json.dump({"summary": summary, "rows": rows}, open(a.json, "w"), indent=1)
    print(json.dumps({k: summary[k] for k in ("aid", "box", "cases", "worst_over_db", "min_diff_db", "over")}))
    for r in summary["worst_cases"][:3]:
        print("   ", r["case"], "mac", [round(x, 2) for x in r["mac_db"]], "v2-mac", [round(x, 2) for x in r["diff_db"]])
    return 0 if not over else 1


if __name__ == "__main__":
    sys.exit(main())
