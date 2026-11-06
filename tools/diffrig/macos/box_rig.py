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
import array
import json
import math
import os
import subprocess
import sys

try:
    import numpy as np
except ModuleNotFoundError:
    np = None

HERE = os.path.dirname(os.path.abspath(__file__))
FS = 48000
FREQS = [25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250,
         1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000]
FACTORY = {"mstk": "AUMeisterStueckFactory", "tmst": "AUToneMeisterFactory", "vben": "AUVirtualBassFactory",
           "ldnm": "AULoudnessNormalizerFactory", "vrq2": "AUVariableEQV2Factory", "axtc": "AUXTCFactory",
           "cfrk": "AUControlFreakFactory", "buzk": "AUBuzzKillFactory", "cdly": "AUChannelSpecificDelayFactory",
           "volt": "AUVolumeTaperFactory", "flow": "AULowFlowFactory"}


def floats(path):
    values = array.array("f")
    with open(path, "rb") as stream:
        values.fromfile(stream, os.path.getsize(path) // values.itemsize)
    if sys.byteorder != "little":
        values.byteswap()
    return values


def split_v2(a):
    with open(os.path.join(a.reference_dir, "manifest.json")) as stream:
        manifest = json.load(stream)
    if (manifest["aid"], manifest["box"], manifest["slot"], manifest["channels"]) != \
            (a.aid, a.box, a.slot, a.ch):
        sys.exit("reference manifest does not match this rig invocation")
    rows, worst = [], -1e9
    os.makedirs(a.work, exist_ok=True)
    fv2 = os.path.join(a.work, "v2.f32")
    window_frames = max(1, int(a.window_ms * FS / 1000))
    for row in manifest["cases"]:
        name = row["name"]
        fin = os.path.join(a.reference_dir, row["input"])
        fmac = os.path.join(a.reference_dir, row["mac"])
        command = [a.v2, "--tunings-root", a.tunings, "--aid", str(a.aid), "--box", a.box,
                   "--ch", str(a.ch)]
        if a.slot is not None:
            command += ["--only-slot", str(a.slot)]
        result = subprocess.run(command + [fin, fv2], capture_output=True, text=True)
        if result.returncode:
            print(json.dumps({"aid": a.aid, "box": a.box, "refused": result.stderr.strip()}))
            return 1
        mac, v2 = floats(fmac), floats(fv2)
        frames = min(len(mac), len(v2)) // a.ch
        half = frames // 2
        m, v, window = [], [], []
        for channel in range(a.ch):
            def rms(values, begin, end):
                count = max(1, end - begin)
                return math.sqrt(sum(values[i * a.ch + channel] ** 2 for i in range(begin, end)) / count)
            mr, vr = rms(mac, half, frames), rms(v2, half, frames)
            m.append(20 * math.log10(mr + 1e-15))
            v.append(20 * math.log10(vr + 1e-15))
            wd = -1e9
            for at in range(0, frames - window_frames + 1, window_frames):
                mwr, vwr = rms(mac, at, at + window_frames), rms(v2, at, at + window_frames)
                if mwr >= 1e-4:
                    wd = max(wd, 20 * math.log10((vwr + 1e-15) / (mwr + 1e-15)))
            window.append(wd if name.startswith(("step_", "burst_")) else v[-1] - m[-1])
        diff = [vv - mm for vv, mm in zip(v, m)]
        worst = max(worst, max(diff), max(window))
        rows.append({"case": name, "mac_db": m, "v2_db": v, "diff_db": diff,
                     "window_diff_db": window, "latency_frames": 0})
    if os.path.exists(fv2):
        os.unlink(fv2)
    over = [row for row in rows if max(max(row["diff_db"]), max(row["window_diff_db"])) > a.tol]
    summary = {"aid": a.aid, "box": a.box, "slot": a.slot, "cases": len(rows), "worst_over_db": worst,
               "min_diff_db": min(min(min(row["diff_db"]), min(row["window_diff_db"])) for row in rows),
               "over": len(over),
               "worst_cases": sorted(rows, key=lambda row: -max(max(row["diff_db"]),
                                                                  max(row["window_diff_db"])))[:5]}
    if a.json:
        with open(a.json, "w") as stream:
            json.dump({"summary": summary, "rows": rows}, stream, indent=1)
    print(json.dumps({key: summary[key] for key in ("aid", "box", "cases", "worst_over_db", "min_diff_db", "over")}))
    return 0 if not over else 1


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


def window_diff_db(v2, mac, frames):
    worst = [-1e9] * mac.shape[1]
    for at in range(0, len(mac) - frames + 1, frames):
        for c in range(mac.shape[1]):
            mr = np.sqrt(np.mean(mac[at:at + frames, c] ** 2)) + 1e-15
            vr = np.sqrt(np.mean(v2[at:at + frames, c] ** 2)) + 1e-15
            if mr >= 1e-4:  # ignore inaudible filter tails below -80 dBFS
                worst[c] = max(worst[c], float(20 * np.log10(vr / mr)))
    return worst


def align_envelopes(v2, mac, hop=48, max_lag_ms=30):
    count = min(len(v2), len(mac)) // hop
    ve = np.mean(v2[:count * hop, 0].reshape(count, hop) ** 2, axis=1)
    me = np.mean(mac[:count * hop, 0].reshape(count, hop) ** 2, axis=1)
    best = (float("-inf"), 0)
    for lag in range(-max_lag_ms, max_lag_ms + 1):
        a, b = (me[lag:], ve[:len(ve) - lag]) if lag >= 0 else (me[:len(me) + lag], ve[-lag:])
        score = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-30))
        best = max(best, (score, lag))
    lag = best[1] * hop
    if lag > 0:
        return v2[:-lag], mac[lag:], lag
    if lag < 0:
        return v2[-lag:], mac[:lag], lag
    return v2, mac, 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tunings", required=True)
    ap.add_argument("--aid", type=int, required=True)
    ap.add_argument("--box", required=True)
    ap.add_argument("--subtype", default="mstk")
    ap.add_argument("--slot", type=int, help="render only this tmst slot (all box gains and other slots off)")
    ap.add_argument("--ch", type=int, required=True)
    ap.add_argument("--v2", required=True)
    ap.add_argument("--box-host", default=os.path.join(HERE, "box_host"))
    ap.add_argument("--work", required=True)
    ap.add_argument("--json")
    ap.add_argument("--reference-dir", help="persistent private inputs/macOS outputs for a split rig run")
    phase = ap.add_mutually_exclusive_group()
    phase.add_argument("--mac-only", action="store_true", help="render and retain only the installed-unit references")
    phase.add_argument("--v2-only", action="store_true", help="consume a prior --mac-only reference set")
    ap.add_argument("--strip", default="builtin_speaker_out_general.austrip")
    ap.add_argument("--levels", default="0,-6,-12,-24,-40")
    ap.add_argument("--frequencies", help="comma-separated sine frequencies (default: third-octave grid)")
    ap.add_argument("--dynamic-programs", action="store_true", help="add low/high steps, release steps and tone bursts")
    ap.add_argument("--window-ms", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=1.0)
    ap.add_argument("--tol", type=float, default=0.05)
    a = ap.parse_args()

    if (a.mac_only or a.v2_only) and not a.reference_dir:
        ap.error("--mac-only/--v2-only requires --reference-dir")
    if a.v2_only:
        return split_v2(a)
    if np is None:
        sys.exit("numpy is required to generate reference programs")

    os.makedirs(a.work, exist_ok=True)
    strip = os.path.join(a.tunings, f"AID{a.aid}", "DSP", "Strips", a.strip)
    n = int(FS * a.seconds)
    t = np.arange(n) / FS
    levels = [float(v) for v in a.levels.split(",")]
    frequencies = FREQS if a.frequencies is None else [float(v) for v in a.frequencies.split(",")]
    cases = [(f"sine_{f:g}_{-l:g}", 10 ** (l / 20) * np.sin(2 * np.pi * f * t)) for f in frequencies for l in levels]
    pk = pink(n)
    cases += [(f"pink_{-l:g}", 10 ** (l / 20) * pk) for l in levels]
    if a.dynamic_programs:
        half = n // 2
        for f in frequencies:
            carrier = np.sin(2 * np.pi * f * t)
            attack = np.r_[np.full(half, 0.01), np.ones(n - half)]
            release = np.r_[np.ones(half), np.full(n - half, 0.01)]
            burst = ((np.arange(n) // max(1, int(0.02 * FS))) % 4 == 0).astype(float)
            cases += [(f"step_up_{f:g}", attack * carrier), (f"step_down_{f:g}", release * carrier),
                      (f"burst_{f:g}", burst * carrier)]

    if a.reference_dir:
        os.makedirs(a.reference_dir, exist_ok=True)
    if a.mac_only:
        manifest = {"aid": a.aid, "box": a.box, "slot": a.slot, "channels": a.ch,
                    "window_ms": a.window_ms, "cases": []}
        for index, (name, x) in enumerate(cases):
            fin = os.path.join(a.reference_dir, f"{index:04d}-in.f32")
            fmac = os.path.join(a.reference_dir, f"{index:04d}-mac.f32")
            np.repeat(x[:, None], a.ch, axis=1).astype("<f4").tofile(fin)
            mac_cmd = [a.box_host, "--strip", strip, "--stage", a.box,
                       "--subtype", a.subtype, "--factory", FACTORY[a.subtype], "--channels", str(a.ch),
                       "--in", fin, "--out", fmac, "--meta", os.path.join(a.reference_dir, "meta.json")]
            if a.slot is not None:
                for pid in (0, 1, 4, 11, 13):
                    mac_cmd += ["--set", f"{pid}=0"]
                for slot in range(50):
                    if slot != a.slot:
                        mac_cmd += ["--set", f"{14 + 10 * slot}=0"]
            r = subprocess.run(mac_cmd, capture_output=True, text=True)
            if r.returncode:
                sys.exit(f"box_host: {r.stderr.strip()}")
            manifest["cases"].append({"name": name, "input": os.path.basename(fin),
                                      "mac": os.path.basename(fmac)})
        with open(os.path.join(a.reference_dir, "manifest.json"), "w") as stream:
            json.dump(manifest, stream, indent=1)
        print(json.dumps({"aid": a.aid, "box": a.box, "references": len(cases)}))
        return 0

    rows, worst = [], -1e9
    fin, fmac, fv2 = (os.path.join(a.work, x) for x in ("in.f32", "mac.f32", "v2.f32"))
    for name, x in cases:
        np.repeat(x[:, None], a.ch, axis=1).astype("<f4").tofile(fin)
        mac_cmd = [a.box_host, "--strip", strip, "--stage", a.box,
                   "--subtype", a.subtype, "--factory", FACTORY[a.subtype], "--channels", str(a.ch),
                   "--in", fin, "--out", fmac, "--meta", os.path.join(a.work, "meta.json")]
        if a.slot is not None:
            for pid in (0, 1, 4, 11, 13):
                mac_cmd += ["--set", f"{pid}=0"]
            for slot in range(50):
                if slot != a.slot:
                    mac_cmd += ["--set", f"{14 + 10 * slot}=0"]
        r = subprocess.run(mac_cmd, capture_output=True, text=True)
        if r.returncode:
            sys.exit(f"box_host: {r.stderr.strip()}")
        v2_cmd = [a.v2, "--tunings-root", a.tunings, "--aid", str(a.aid), "--box", a.box, "--ch", str(a.ch)]
        if a.slot is not None:
            v2_cmd += ["--only-slot", str(a.slot)]
        r = subprocess.run(v2_cmd + [fin, fv2], capture_output=True, text=True)
        if r.returncode:
            print(json.dumps({"aid": a.aid, "box": a.box, "refused": r.stderr.strip()}))
            return 1
        mac = np.fromfile(fmac, "<f4").reshape(-1, a.ch)
        v2 = np.fromfile(fv2, "<f4").reshape(-1, a.ch)
        latency = 0
        if name.startswith("burst_"):
            v2, mac, latency = align_envelopes(v2, mac)
        m = rms_db(mac)
        v = rms_db(v2)
        d = [vv - mm for vv, mm in zip(v, m)]
        window = (window_diff_db(v2, mac, max(1, int(a.window_ms * FS / 1000)))
                  if name.startswith(("step_", "burst_")) else d.copy())
        worst = max(worst, max(d), max(window))
        rows.append({"case": name, "mac_db": m, "v2_db": v, "diff_db": d, "window_diff_db": window,
                     "latency_frames": latency})
    for f in (fin, fmac, fv2):
        if os.path.exists(f):
            os.unlink(f)
    over = [r for r in rows if max(max(r["diff_db"]), max(r["window_diff_db"])) > a.tol]
    summary = {"aid": a.aid, "box": a.box, "slot": a.slot, "cases": len(rows), "worst_over_db": worst,
               "min_diff_db": min(min(min(r["diff_db"]), min(r["window_diff_db"])) for r in rows), "over": len(over),
               "worst_cases": sorted(rows, key=lambda r: -max(max(r["diff_db"]), max(r["window_diff_db"])))[:5]}
    if a.json:
        json.dump({"summary": summary, "rows": rows}, open(a.json, "w"), indent=1)
    print(json.dumps({k: summary[k] for k in ("aid", "box", "cases", "worst_over_db", "min_diff_db", "over")}))
    for r in summary["worst_cases"][:3]:
        print("   ", r["case"], "mac", [round(x, 2) for x in r["mac_db"]], "v2-mac", [round(x, 2) for x in r["diff_db"]],
              "window", [round(x, 2) for x in r["window_diff_db"]])
    return 0 if not over else 1


if __name__ == "__main__":
    sys.exit(main())
