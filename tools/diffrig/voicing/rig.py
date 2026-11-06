#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Voicing rig: the Mac's whole speaker chain on macOS against v2's.

On a Mac (offline, no audio device): for every tuning given, render one
program through

  * macOS: the graph's boxes in the plan's order, each through the Mac's
    own installed Audio Unit (box_host, with the strip's state, the
    property strip and the graph parameters at the volume under test),
    the graph's builtins (non-finite guard, gain) here, the plumbing from
    the plan's sources;
  * v2: the voicing chain compiled from the same tuning
    (`aurora-speaker-tuning` example `voicing-render`),

and compare per unit lane (driver) and per band: steady sines at the
1/3-octave centres 20 Hz to 20 kHz, at several program levels, at several
system volumes. A band where v2 is louder than macOS by more than the
tolerance is flagged. Results that depend on a box v2 still stubs are
marked with the stub list.

Both sides share one topology (the plan the tuning crate traces).
`--check-plan` checks it against macOS's own boxes: a tone pair on one
program channel at a time, and every lane must answer the program channel
the plan traces it to.

Everything this writes from a Mac's tuning is Apple-derived: write it to
a private directory (the default output is under the scratch directory
given), never into the repository.

Volume: `--volumes` takes system volume scalars. Each maps to the graph's
`mvsc` (the scalar) and `mvol` (dB) through `--law`: a JSON file
{"points": [[scalar, mvol_db], ...]} (linear interpolation), or the
built-in placeholder `linear-db:RANGE` (mvol = RANGE * (scalar - 1) dB).
The placeholder is a test-point generator only, not a claim about any
Mac's curve; the report states which law was used.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor

import numpy as np

FS = 48000
HERE = os.path.dirname(os.path.abspath(__file__))


def third_octaves(lo=20.0, hi=20000.0):
    out, k = [], -17
    while True:
        f = 1000.0 * 2 ** (k / 3)
        if f > hi * 1.01:
            break
        if f >= lo * 0.95:  # the 20 Hz band centre is 19.7 Hz
            out.append(round(f, 1))
        k += 1
    return out


def program(bands, levels, seg_s, gap_s):
    seg, gap = int(seg_s * FS), int(gap_s * FS)
    t = np.arange(seg) / FS
    ramp = min(seg // 10, int(0.01 * FS))
    win = np.ones(seg)
    win[:ramp] = np.linspace(0, 1, ramp)
    win[-ramp:] = np.linspace(1, 0, ramp)
    parts, index = [], []
    pos = 0
    for L in levels:
        for f in bands:
            x = 10 ** (L / 20) * np.sin(2 * np.pi * f * t) * win
            parts.append(np.stack([x, x], 1))
            parts.append(np.zeros((gap, 2)))
            index.append((L, f, pos, pos + seg))
            pos += seg + gap
    return np.concatenate(parts).astype(np.float32), index


def tone_db(y, f, a, b):
    """Level (dBFS peak) of the component at f over the settled part."""
    a = a + (b - a) * 2 // 5
    b = b - (b - a) // 10
    seg = y[a:b].astype(np.float64)
    t = np.arange(a, b) / FS
    c, s = np.cos(2 * np.pi * f * t), np.sin(2 * np.pi * f * t)
    amp = 2 * math.hypot(seg @ c, seg @ s) / len(seg)
    return 20 * math.log10(max(amp, 1e-12))


def law_from(spec):
    if spec.startswith("linear-db:"):
        r = float(spec.split(":", 1)[1])
        return (lambda s: r * (s - 1.0)), spec
    pts = sorted(json.load(open(spec))["points"])
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (lambda s: float(np.interp(s, xs, ys))), spec


# ---------------------------------------------------------------- macOS side

def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(f"{' '.join(cmd[:4])}...: {r.stderr.strip()[-400:]}")
    return r


def macos_chain(plan, strip, propstrip, x, mvsc, mvol, work, host, taps=None):
    """Renders program x (frames x program channels) through the Mac's own
    boxes in plan order. Returns (lanes, node outputs if taps)."""
    nodes = plan["nodes"]
    outs = []
    values = {"mvsc": mvsc, "mvol": mvol, "mmut": 0.0, "lbal": 0.0, "rbal": 0.0}
    for i, n in enumerate(nodes):
        src = []
        for s in n["inputs"]:
            src.append(x[:, s["channel"]] if s["from"] == "program" else outs[s["node"]][:, s["channel"]])
        u = np.stack(src, 1).astype(np.float32)
        links = [l for l in plan["params"] if l["node"] == i]
        kind = n["kind"]["kind"]
        if kind == "non_finite":
            y = np.nan_to_num(u, nan=0.0, posinf=0.0, neginf=0.0)
        elif kind == "gain":
            g, mute = 0.0, False
            for l in links:
                v = values.get(l["param"], l.get("default") or 0.0)
                if l["id"] == 0:
                    g = v
                elif l["id"] == 1:
                    mute = v != 0.0
            y = np.zeros_like(u) if mute else u * np.float32(10 ** (g / 20))
        else:
            fin, fout, meta = (os.path.join(work, f"{i}.{e}") for e in ("in.f32", "out.f32", "json"))
            u.tofile(fin)
            cmd = [host, "--strip", strip, "--box", n["name"], "--channels", str(u.shape[1]),
                   "--in", fin, "--out", fout, "--meta", meta]
            if propstrip:
                cmd += ["--propstrip", propstrip]
            if n["bypass"]:
                cmd += ["--bypass"]
            for l in links:
                v = values.get(l["param"], l.get("default") or 0.0)
                rest = l.get("rest") or []
                scope = {"global": 0, "input": 1, "output": 2}.get(rest[0], rest[0]) if rest else 0
                elem = rest[1] if len(rest) > 1 else 0
                cmd += ["--set", f"{l['id']}/{scope}/{elem}={v}"]
            run(cmd)
            y = np.fromfile(fout, np.float32).reshape(-1, u.shape[1])
            for f in (fin, fout):
                os.remove(f)
        outs.append(y)
    lanes = []
    for s in plan["outputs"]:
        lanes.append(x[:, s["channel"]] if s["from"] == "program" else outs[s["node"]][:, s["channel"]])
    return np.stack(lanes, 1), (outs if taps else None)


# ---------------------------------------------------------------- v2 side

def v2_chain(render, src_args, x, mvsc, mvol, work):
    fin, fout = os.path.join(work, "v2in.f32"), os.path.join(work, "v2out.f32")
    x.astype(np.float32).tofile(fin)
    run([render, *src_args, "--in", fin, "--out", fout, "--mvsc", str(mvsc), "--mvol", str(mvol)])
    y = np.fromfile(fout, np.float32)
    os.remove(fin)
    os.remove(fout)
    return y


def info(render, src_args, work):
    p = os.path.join(work, "info.json")
    r = subprocess.run([render, *src_args, "--info", p], capture_output=True, text=True)
    if r.returncode:
        return None, r.stderr.strip()[-300:]
    return json.load(open(p)), None


# ---------------------------------------------------------------- one tuning

def one(job):
    name, src_args, strip, propstrip, a = job
    work = tempfile.mkdtemp(prefix=f"vrig-{name}-", dir=a["tmp"])
    try:
        inf, err = info(a["render"], src_args, work)
        if inf is None:
            return {"tuning": name, "error": f"v2 compile refused: {err}"}
        plan = inf["plan"]
        extra = []
        if a.get("box"):
            plan, extra = one_box(plan, a["box"])
            if plan is None:
                return {"tuning": name, "skipped": f"no box {a['box']}"}
        bands = third_octaves()
        x, index = program(bands, a["levels"], a["seg"], a["gap"])
        if x.shape[1] != plan["program_channels"]:
            x = np.tile(x[:, :1], (1, plan["program_channels"]))
        law, _ = law_from(a["law"])
        lanes_n = len(plan["outputs"])
        res = {"tuning": name, "stubs": inf["stubs"], "gaps": inf.get("gaps", []), "unwired": inf["unwired"],
               "lanes": lanes_n,
               "volumes": []}
        if a["check_plan"] and not a.get("box"):
            res["plan_check"] = check_plan(plan, strip, propstrip, work, a["host"])
        for s in a["volumes"]:
            mvol = law(s)
            mac, _ = macos_chain(plan, strip, propstrip, x, s, mvol, work, a["host"])
            v2 = v2_chain(a["render"], src_args + extra, x, s, mvol, work).reshape(-1, lanes_n)
            rows = []
            for (L, f, i0, i1) in index:
                for ln in range(lanes_n):
                    m, v = tone_db(mac[:, ln], f, i0, i1), tone_db(v2[:, ln], f, i0, i1)
                    rows.append({"level": L, "hz": f, "lane": ln, "macos_db": round(m, 3),
                                 "v2_db": round(v, 3), "excess_db": round(v - m, 3)})
            res["volumes"].append({"scalar": s, "mvol_db": round(mvol, 3), "rows": rows,
                                   "macos_peak": float(np.max(np.abs(mac))),
                                   "v2_peak": float(np.max(np.abs(v2)))})
        return res
    except Exception as e:  # report, keep the farm going
        return {"tuning": name, "error": repr(e)}
    finally:
        for f in os.listdir(work):
            os.remove(os.path.join(work, f))
        os.rmdir(work)


def one_box(plan, name):
    """A plan of one box alone: its channels are the program, its output
    the lanes (the per-box gate; v2 runs it with `--only`)."""
    idx = next((i for i, n in enumerate(plan["nodes"]) if n["name"] == name), None)
    if idx is None:
        return None, []
    n = dict(plan["nodes"][idx])
    w = len(n["inputs"])
    n["inputs"] = [{"from": "program", "channel": c} for c in range(w)]
    links = [dict(l, node=0) for l in plan["params"] if l["node"] == idx]
    one = {"program_channels": w, "nodes": [n], "params": links,
           "outputs": [{"from": "node", "node": 0, "channel": c} for c in range(w)]}
    return one, ["--only", name]


def check_plan(plan, strip, propstrip, work, host):
    """A tone pair (100 Hz for the woofers, 4 kHz for the tweeters) on one
    program channel at a time, through macOS's own boxes: every lane must
    answer the program channel the plan traces it to more than the other
    one (XTC mixes the sides, so the margin asked is 1 dB)."""
    pc = plan["program_channels"]
    n = FS // 2
    t = np.arange(n) / FS
    lv = []
    for ch in range(pc):
        x = np.zeros((n, pc), np.float32)
        x[:, ch] = 0.05 * (np.sin(2 * np.pi * 100 * t) + np.sin(2 * np.pi * 4000 * t))
        lanes, _ = macos_chain(plan, strip, propstrip, x, 1.0, 0.0, work, host)
        lv.append([max(tone_db(lanes[:, ln], 100, 0, n), tone_db(lanes[:, ln], 4000, 0, n))
                   for ln in range(lanes.shape[1])])
    lv = np.array(lv)  # program channel x lane
    nodes = plan["nodes"]

    def origin(s):
        # follow channel 'place' back to the program (a mixing box keeps it)
        while s["from"] != "program":
            s = nodes[s["node"]]["inputs"][s["channel"]]
        return s["channel"]

    want = [origin(s) for s in plan["outputs"]]
    ok = pc != 2 or all(lv[want[ln], ln] >= lv[1 - want[ln], ln] + 1.0 for ln in range(len(want)))
    return {"ok": bool(ok), "want": want, "levels_db": np.round(lv, 1).tolist()}


# ---------------------------------------------------------------- report

def summarise(results, tol, law_name, out_md):
    lines = ["# Voicing rig: v2 chain vs macOS chain, per driver and band", "",
             f"Law: {law_name}. Tolerance: v2 may not exceed macOS by more than {tol} dB.",
             "Excess = v2 - macOS (dB) at the tone frequency; worst per volume/level shown.", ""]
    flagged = 0
    for r in results:
        lines.append(f"## {r['tuning']}")
        if "skipped" in r:
            lines.append(r["skipped"])
            lines.append("")
            continue
        if "error" in r:
            lines.append(f"ERROR: {r['error']}")
            flagged += 1
            lines.append("")
            continue
        if r.get("plan_check"):
            lines.append(f"plan check: {'OK' if r['plan_check']['ok'] else 'FAIL'} {r['plan_check']['want']}")
        lines.append("stubs: " + (", ".join(f"{a}({b})" for a, b in r["stubs"]) or "none"))
        if r.get("gaps"):
            lines.append("gaps: " + "; ".join(f"{a}: {b}" for a, b in r["gaps"]))
        if r["unwired"]:
            lines.append("graph params not driven by v2: " + "; ".join(r["unwired"]))
        lines.append("")
        lines.append("| volume | mvol dB | level dBFS | worst excess dB (lane @ Hz) | bands above | lanes above "
                     "| worst abs diff dB |")
        lines.append("|---|---|---|---|---|---|---|")
        for v in r["volumes"]:
            for L in sorted({x["level"] for x in v["rows"]}):
                rows = [x for x in v["rows"] if x["level"] == L and x["macos_db"] > -120]
                w = max(rows, key=lambda x: x["excess_db"])
                above = [x for x in rows if x["excess_db"] > tol]
                flagged += len(above)
                inb = [x for x in rows if x["macos_db"] > L - 60]
                ad = max((abs(x["excess_db"]) for x in inb), default=0.0)
                lines.append(f"| {v['scalar']} | {v['mvol_db']} | {L} | {w['excess_db']:+.2f} "
                             f"(L{w['lane']} @ {w['hz']}) | {len({x['hz'] for x in above})} "
                             f"| {sorted({x['lane'] for x in above})} | {ad:.3f} |")
        lines.append("")
    lines.insert(4, f"FLAGGED (v2 above macOS beyond tolerance, or errors): {flagged}")
    lines.insert(5, "")
    open(out_md, "w").write("\n".join(lines) + "\n")
    return flagged


def self_test():
    b = third_octaves()
    assert len(b) == 31 and abs(b[0] - 19.7) < 0.1, b[:2]
    assert abs(b[-1] - 20158.7) < 1
    x, index = program([100.0, 1000.0], [-20.0, 0.0], 0.2, 0.05)
    assert x.shape[1] == 2 and len(index) == 4
    for (L, f, a, b2) in index:
        assert abs(tone_db(x[:, 0], f, a, b2) - L) < 0.05, (L, f)
    law, _ = law_from("linear-db:60")
    assert law(1.0) == 0.0 and abs(law(0.5) + 30) < 1e-9
    # the report flags v2 above macOS beyond tolerance, and errors
    rows = [{"level": -6, "hz": 100.0, "lane": 0, "macos_db": -10.0, "v2_db": -9.5, "excess_db": 0.5},
            {"level": -6, "hz": 200.0, "lane": 1, "macos_db": -10.0, "v2_db": -12.0, "excess_db": -2.0}]
    res = [{"tuning": "T", "stubs": [["VarEQ", "vrq2"]], "gaps": [], "unwired": [], "lanes": 2,
            "volumes": [{"scalar": 1.0, "mvol_db": 0.0, "rows": rows}]},
           {"tuning": "U", "error": "refused"}]
    with tempfile.TemporaryDirectory() as d:
        n = summarise(res, 0.1, "linear-db:60", os.path.join(d, "r.md"))
        txt = open(os.path.join(d, "r.md")).read()
    assert n == 2 and "+0.50 (L0 @ 100.0)" in txt and "ERROR: refused" in txt, txt
    print("rig self-test ok")


def main():
    if sys.argv[1:] == ["--self-test"]:
        return self_test()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tunings-root", default="/System/Library/Audio/Tunings")
    ap.add_argument("--aids", default="", help="comma list; default: every spp3 AID the root has")
    ap.add_argument("--bundle", action="append", default=[], help="NAME=PATH of a collected bundle (repeat)")
    ap.add_argument("--host", default=None, help="box_host binary (built from box_host.m)")
    ap.add_argument("--render", required=True, help="voicing-render example binary")
    ap.add_argument("--volumes", default="1,0.75,0.5,0.25,0.1")
    ap.add_argument("--law", default="linear-db:63.5")
    ap.add_argument("--levels", default="-20,-6,0")
    ap.add_argument("--seg", type=float, default=0.5)
    ap.add_argument("--gap", type=float, default=0.1)
    ap.add_argument("--tol", type=float, default=0.1)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--check-plan", action="store_true")
    ap.add_argument("--box", default=None, help="per-box gate: this box alone, same input, both sides")
    ap.add_argument("--out", required=True, help="private output directory (Apple-derived results)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    host = a.host or os.path.join(a.out, "box_host")
    if not a.host:
        run(["clang", "-fobjc-arc", "-O2", "-framework", "AudioToolbox", "-framework", "Foundation",
             os.path.join(HERE, "box_host.m"), "-o", host])
    tmp = os.path.join(a.out, "tmp")
    os.makedirs(tmp, exist_ok=True)
    common = {"host": host, "render": a.render, "volumes": [float(v) for v in a.volumes.split(",")],
              "law": a.law, "levels": [float(v) for v in a.levels.split(",")], "seg": a.seg, "gap": a.gap,
              "check_plan": a.check_plan, "tmp": tmp, "box": a.box}
    jobs = []
    aids = [x for x in a.aids.split(",") if x]
    if not aids and not a.bundle:
        aids = [d[3:] for d in sorted(os.listdir(a.tunings_root), key=lambda d: int(d[3:]) if d[3:].isdigit() else 0)
                if d.startswith("AID") and spp3(os.path.join(a.tunings_root, d))]
    for aid in aids:
        d = os.path.join(a.tunings_root, f"AID{aid}", "DSP", "Strips")
        jobs.append((f"AID{aid}", ["--tunings-root", a.tunings_root, "--aid", aid],
                     os.path.join(d, "builtin_speaker_out_general.austrip"),
                     os.path.join(d, "builtin_speaker_out_general.propstrip"), common))
    for b in a.bundle:
        name, path = b.split("=", 1)
        strip, prop = bundle_strips(path, a.out)
        jobs.append((name, ["--bundle", path], strip, prop, common))
    with ProcessPoolExecutor(a.jobs) as ex:
        results = list(ex.map(one, jobs))
    json.dump(results, open(os.path.join(a.out, "rig.json"), "w"))
    flagged = summarise(results, a.tol, a.law, os.path.join(a.out, "report.md"))
    print(open(os.path.join(a.out, "report.md")).read())
    sys.exit(1 if flagged else 0)


def spp3(aid_dir):
    s = os.path.join(aid_dir, "DSP", "Strips")
    return os.path.isdir(s) and any(f.endswith("-aufx-spp3-appl.plist") for f in os.listdir(s))


def bundle_strips(path, out):
    """The general speaker strips in a collected bundle (audio-tuning/AIDn).
    The property strip names its files by their path on the collecting
    Mac; a copy pointing into the bundle is written to `out`."""
    import plistlib
    for root, _, files in os.walk(path):
        if "builtin_speaker_out_general.austrip" in files:
            p = os.path.join(root, "builtin_speaker_out_general.propstrip")
            if not os.path.exists(p):
                return os.path.join(root, "builtin_speaker_out_general.austrip"), None
            ps = plistlib.load(open(p, "rb"))
            tuning_root = os.path.dirname(os.path.dirname(os.path.dirname(root)))
            for b in ps.get("Boxes", []):
                for prop in b.get("Properties", []):
                    v = prop.get("Path")
                    if v and "/Audio/Tunings/" in v:
                        local = os.path.join(tuning_root, v.split("/Audio/Tunings/", 1)[1])
                        if not os.path.exists(local):
                            raise SystemExit(f"{path}: the property strip names {v}, not in the bundle")
                        prop["Path"] = local
            q = os.path.join(out, f"{abs(hash(path))}.propstrip")
            plistlib.dump(ps, open(q, "wb"))
            return os.path.join(root, "builtin_speaker_out_general.austrip"), q
    raise SystemExit(f"{path}: no speaker strip")


if __name__ == "__main__":
    main()
