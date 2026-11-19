#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Pre-stage rig: what v2's pre-stage carries of a Mac's speaker graph, vs macOS.

macOS side: graph_render.py up to the protection unit with only the filter
(`tmst`) and compressor (`mstk`) boxes running; every other Audio Unit box
is a wire. v2 side: the graph's routing alone (graph_render.py with every
Audio Unit box a wire) gives each unit lane its input; the compiled blob's
pre-stage (`prestage-render`) runs on it. Same signals both sides: steady
sines on a third-octave grid at several levels, identical on both program
channels, and pink noise. Per case and unit lane: steady RMS (second half)
and v2 minus macOS in dB.

    prestage_rig.py --tunings DIR --aid N --cli PATH/aurora-speaker-tuning \
        --render PATH/prestage-render --work DIR [--json OUT] [--levels 0,-12,-30]

Exit 0 when v2 is at or below macOS + tol everywhere. Offline only;
outputs made with Apple tuning are Apple-derived (keep them out of git).
"""

import argparse
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FS = 48000
FREQS = [25, 40, 63, 100, 160, 250, 400, 630, 1000, 1600, 2500, 4000, 6300, 10000, 16000]
CARRIED = {"tmst", "mstk"}


def boxes(tunings, aid):
    """Every Audio Unit box name and subtype in the AID's speaker graph."""
    sys.path.insert(0, HERE)
    import graph_render as gr
    root = os.path.join(tunings, f"AID{aid}", "DSP", "Graphs", "builtin_speaker_out.dspg")
    _, b, _, _, _ = gr.parse_dspg(open(root).read())
    return [(n, v["type"]) for n, v in b.items() if v.get("au")]


def render_graph(a, inp, out, bypass):
    cmd = [sys.executable, os.path.join(HERE, "graph_render.py"), "--tunings", a.tunings, "--aid", str(a.aid),
           "--in", inp, "--out", out, "--meta", out + ".json"]
    for b in bypass:
        cmd += ["--bypass", b]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        sys.exit(f"graph_render: {r.stderr.strip()}")
    return json.load(open(out + ".json"))["channels"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tunings", required=True)
    ap.add_argument("--aid", type=int, required=True)
    ap.add_argument("--cli", required=True)
    ap.add_argument("--render", required=True)
    ap.add_argument("--work", required=True)
    ap.add_argument("--json")
    ap.add_argument("--levels", default="0,-12,-30")
    ap.add_argument("--tol", type=float, default=0.05)
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=True)
    blob = os.path.join(a.work, "blob")
    r = subprocess.run([a.cli, "compile", "--development", "--board", "j473", "--tunings-root", a.tunings,
                        "--aid", str(a.aid), "-o", blob], capture_output=True, text=True)
    if r.returncode:
        print(json.dumps({"aid": a.aid, "refused": [l for l in r.stderr.splitlines() if l.startswith("error")]}))
        return 1
    au = boxes(a.tunings, a.aid)
    keep_off = [n for n, t in au if t not in CARRIED]
    all_off = [n for n, _ in au]

    n = FS
    t = np.arange(n) / FS
    rng = np.random.default_rng(1)
    f = np.fft.rfft(rng.standard_normal(n))
    k = np.arange(len(f), dtype=float)
    k[0] = 1
    pk = np.fft.irfft(f / np.sqrt(k), n)
    pk /= np.max(np.abs(pk))
    cases = [(f"sine_{fr:g}_{-l:g}", 10 ** (l / 20) * np.sin(2 * np.pi * fr * t))
             for fr in FREQS for l in [float(v) for v in a.levels.split(",")]]
    cases += [(f"pink_{-l:g}", 10 ** (l / 20) * pk) for l in [float(v) for v in a.levels.split(",")]]

    fin, fmac, flane, fv2 = (os.path.join(a.work, x) for x in ("in.f32", "mac.f32", "lane.f32", "v2.f32"))
    rows, worst = [], -1e9
    for name, x in cases:
        np.stack([x, x], 1).astype("<f4").tofile(fin)
        ch = render_graph(a, fin, fmac, keep_off)
        render_graph(a, fin, flane, all_off)
        r = subprocess.run([a.render, blob, flane, fv2], capture_output=True, text=True)
        if r.returncode:
            sys.exit(f"prestage-render: {r.stderr.strip()}")
        m = np.fromfile(fmac, "<f4").reshape(-1, ch).astype(float)[n // 2:]
        v = np.fromfile(fv2, "<f4").reshape(-1, ch).astype(float)[n // 2:]
        md = [float(20 * np.log10(np.sqrt(np.mean(m[:, c] ** 2)) + 1e-15)) for c in range(ch)]
        vd = [float(20 * np.log10(np.sqrt(np.mean(v[:, c] ** 2)) + 1e-15)) for c in range(ch)]
        # a lane macOS drives below -100 dB is not compared
        d = [vv - mm if mm > -100 else 0.0 for vv, mm in zip(vd, md)]
        worst = max(worst, max(d))
        rows.append({"case": name, "mac_db": md, "v2_db": vd, "diff_db": d})
    for p in (fin, fmac, flane, fv2, fmac + ".json", flane + ".json"):
        if os.path.exists(p):
            os.unlink(p)
    over = [r for r in rows if max(r["diff_db"]) > a.tol]
    s = {"aid": a.aid, "cases": len(rows), "worst_over_db": worst,
         "min_diff_db": min(min(r["diff_db"]) for r in rows), "over": len(over),
         "bypassed_both_sides": keep_off}
    if a.json:
        json.dump({"summary": s, "rows": rows}, open(a.json, "w"), indent=1)
    print(json.dumps({k: s[k] for k in ("aid", "cases", "worst_over_db", "min_diff_db", "over")}))
    for r in sorted(rows, key=lambda r: -max(r["diff_db"]))[:3]:
        print("   ", r["case"], "mac", [round(x, 2) for x in r["mac_db"]], "v2-mac", [round(x, 2) for x in r["diff_db"]])
    return 0 if not over else 1


if __name__ == "__main__":
    sys.exit(main())
