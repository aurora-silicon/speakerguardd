#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Offline per-box differential rig: installed Audio Unit versus v2.

The same interleaved f32 programs are rendered by ``box_host`` with the
tuning's saved state and by ``voicing-box-render``. Output derived from an
installed tuning belongs in an ignored work directory, never in the source
tree. Steady cases compare the second-half RMS; burst and level-step cases
compare every non-silent 10 ms window.
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
FACTORY = {
    "mstk": "AUMeisterStueckFactory",
    "tmst": "AUToneMeisterFactory",
    "vben": "AUVirtualBassFactory",
    "ldnm": "AULoudnessNormalizerFactory",
    "vrq2": "AUVariableEQV2Factory",
    "axtc": "AUXTCFactory",
    "cfrk": "AUControlFreakFactory",
    "buzk": "AUBuzzKillFactory",
    "cdly": "AUChannelSpecificDelayFactory",
    "volt": "AUVolumeTaperFactory",
    "flow": "AULowFlowFactory",
}


def pink(n, seed=1):
    rng = np.random.default_rng(seed)
    spectrum = np.fft.rfft(rng.standard_normal(n))
    bins = np.arange(len(spectrum), dtype=float)
    bins[0] = 1.0
    out = np.fft.irfft(spectrum / np.sqrt(bins), n)
    return out / np.max(np.abs(out))


def rms_db(y):
    steady = y[len(y) // 2:]
    return [float(20 * np.log10(np.sqrt(np.mean(steady[:, c] ** 2)) + 1e-15)) for c in range(y.shape[1])]


def window_metrics(mac, v2, frames=480):
    """Per-channel worst v2-minus-reference over non-overlapping windows."""
    out = []
    for channel in range(mac.shape[1]):
        best = (-1e9, 0.0, 0.0, 0)
        minimum = 1e9
        for start in range(0, len(mac) - frames + 1, frames):
            left = float(20 * np.log10(np.sqrt(np.mean(mac[start:start + frames, channel] ** 2)) + 1e-15))
            right = float(20 * np.log10(np.sqrt(np.mean(v2[start:start + frames, channel] ** 2)) + 1e-15))
            if left <= -100.0:
                continue
            diff = right - left
            minimum = min(minimum, diff)
            if diff > best[0]:
                best = (diff, left, right, start)
        out.append({"diff_db": best[0], "mac_db": best[1], "v2_db": best[2], "frame": best[3], "min_diff_db": minimum})
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tunings", required=True)
    parser.add_argument("--aid", type=int, required=True)
    parser.add_argument("--box", required=True)
    parser.add_argument("--subtype", default="mstk")
    parser.add_argument("--slot", type=int, help="render only this tmst slot (all box gains and other slots off)")
    parser.add_argument("--ch", type=int, required=True)
    parser.add_argument("--v2", required=True)
    parser.add_argument("--work", required=True)
    parser.add_argument("--json")
    parser.add_argument("--strip", default="builtin_speaker_out_general.austrip")
    parser.add_argument("--levels", default="0,-6,-12,-24,-40")
    parser.add_argument("--seconds", type=float, default=1.0)
    parser.add_argument("--tol", type=float, default=0.05)
    parser.add_argument("--dynamic", action="store_true", help="also run bursts and level steps; compare 10 ms windows")
    parser.add_argument("--dynamic-only", action="store_true", help="run only the burst and level-step cases")
    args = parser.parse_args()

    os.makedirs(args.work, exist_ok=True)
    strip = os.path.join(args.tunings, f"AID{args.aid}", "DSP", "Strips", args.strip)
    n = int(FS * args.seconds)
    time = np.arange(n) / FS
    levels = [float(value) for value in args.levels.split(",")]
    cases = []
    if not args.dynamic_only:
        cases = [("steady", f"sine_{freq:g}_{-level:g}", 10 ** (level / 20) * np.sin(2 * np.pi * freq * time))
                 for freq in FREQS for level in levels]
        noise = pink(n)
        cases += [("steady", f"pink_{-level:g}", 10 ** (level / 20) * noise) for level in levels]
    if args.dynamic or args.dynamic_only:
        for freq in (80, 500, 2500, 10000):
            carrier = np.sin(2 * np.pi * freq * time)
            burst = ((np.arange(n) // (FS // 10)) % 2 == 0).astype(float)
            cases.append(("transient", f"burst_{freq:g}", burst * carrier))
            step = np.empty(n)
            step_levels = (-40.0, -12.0, 0.0, -24.0)
            for index, level in enumerate(step_levels):
                begin = index * n // len(step_levels)
                end = (index + 1) * n // len(step_levels)
                step[begin:end] = 10 ** (level / 20)
            cases.append(("level_step", f"step_{freq:g}", step * carrier))

    rows = []
    paths = [os.path.join(args.work, name) for name in ("in.f32", "mac.f32", "v2.f32")]
    fin, fmac, fv2 = paths
    for kind, name, signal in cases:
        np.repeat(signal[:, None], args.ch, axis=1).astype("<f4").tofile(fin)
        mac_cmd = [os.path.join(HERE, "box_host"), "--strip", strip, "--stage", args.box,
                   "--subtype", args.subtype, "--factory", FACTORY[args.subtype], "--channels", str(args.ch),
                   "--in", fin, "--out", fmac, "--meta", os.path.join(args.work, "meta.json")]
        if args.slot is not None:
            for pid in (0, 1, 4, 11, 13):
                mac_cmd += ["--set", f"{pid}=0"]
            for slot in range(50):
                if slot != args.slot:
                    mac_cmd += ["--set", f"{14 + 10 * slot}=0"]
        result = subprocess.run(mac_cmd, capture_output=True, text=True)
        if result.returncode:
            sys.exit(f"box_host: {result.stderr.strip()}")
        v2_cmd = [args.v2, "--tunings-root", args.tunings, "--aid", str(args.aid), "--box", args.box,
                  "--ch", str(args.ch)]
        if args.slot is not None:
            v2_cmd += ["--only-slot", str(args.slot)]
        result = subprocess.run(v2_cmd + [fin, fv2], capture_output=True, text=True)
        if result.returncode:
            print(json.dumps({"aid": args.aid, "box": args.box, "refused": result.stderr.strip()}))
            return 1
        mac_audio = np.fromfile(fmac, "<f4").reshape(-1, args.ch).astype(float)
        v2_audio = np.fromfile(fv2, "<f4").reshape(-1, args.ch).astype(float)
        if kind == "steady":
            mac = rms_db(mac_audio)
            v2 = rms_db(v2_audio)
            rows.append({"kind": kind, "case": name, "mac_db": mac, "v2_db": v2,
                         "diff_db": [right - left for right, left in zip(v2, mac)]})
        else:
            windows = window_metrics(mac_audio, v2_audio)
            rows.append({"kind": kind, "case": name,
                         "mac_db": [row["mac_db"] for row in windows],
                         "v2_db": [row["v2_db"] for row in windows],
                         "diff_db": [row["diff_db"] for row in windows],
                         "min_diff_db": min(row["min_diff_db"] for row in windows),
                         "worst_frame": [row["frame"] for row in windows]})

    for path in paths:
        if os.path.exists(path):
            os.unlink(path)
    over = [row for row in rows if max(row["diff_db"]) > args.tol]
    summary = {
        "aid": args.aid,
        "box": args.box,
        "slot": args.slot,
        "cases": len(rows),
        "worst_over_db": max(max(row["diff_db"]) for row in rows),
        "min_diff_db": min(min(row["diff_db"]) for row in rows),
        "over": len(over),
        "worst_cases": sorted(rows, key=lambda row: -max(row["diff_db"]))[:5],
        "worst_by_kind_db": {
            kind: max(max(row["diff_db"]) for row in rows if row["kind"] == kind)
            for kind in sorted({row["kind"] for row in rows})
        },
    }
    if args.json:
        with open(args.json, "w", encoding="utf-8") as stream:
            json.dump({"summary": summary, "rows": rows}, stream, indent=1)
    print(json.dumps({key: summary[key] for key in ("aid", "box", "cases", "worst_over_db", "min_diff_db", "over")}))
    return 0 if not over else 1


if __name__ == "__main__":
    sys.exit(main())
