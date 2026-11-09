#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Volume-move gate: how a slider step reaches each unit lane, macOS vs v2, per tuning.

A volume move reaches the speaker graph while it plays, and macOS ramps it: the gain box over
20 ms, ToneMeister's automation over 40 ms starting a slice or two later. The parity gate
proper runs at fixed full volume and cannot see that. This gate plays two programs (100 Hz +
700 Hz, and 6 kHz) at slider S0, then steps to S1 at frame AT. For each side it takes the step
response (the step render minus a render held at S0) over the 100 ms after the step, and
compares v2's with macOS's after one best-fit gain, which leaves only shape and timing. The
number per lane is the residual in dB re macOS's step response: about -45 dB on the woofers
for a step applied like macOS, and -15 to -3 dB for one applied at once.

    volmove.py plan  --variants V.json --bin BIN --out PLAN.json            (buildbox: slider -> mvol)
    volmove.py refs  --plan PLAN.json --gr gr.py --box-host BOXHOST --tunings DIR --out REFDIR   (macOS host)
    volmove.py gate  --plan PLAN.json --refs REFDIR --bin BIN --out RESULT.json       (buildbox)

A lane within LANE_DB of the tuning's loudest lane fails when its residual is above LIMIT_DB.
Outputs made from Apple tuning are Apple-derived: REFDIR is private; RESULT.json holds keys,
verdicts and numbers only. Report only for now: verify does not read it.
"""
import argparse, json, math, os, subprocess, sys
from array import array

FS = 48000
S0, S1, AT = 0.15, 0.16, 48128
N = 4800
LIMIT_DB = -20.0
LANE_DB = 20.0
PROGRAMS = {"lo": (100.0, 700.0), "hi": (6000.0,)}
SECONDS = 2


def program(path, tones):
    a = array("f")
    for i in range(SECONDS * FS):
        v = sum(0.25 * math.sin(2 * math.pi * f * i / FS) for f in tones)
        a.append(v); a.append(v)
    with open(path, "wb") as f:
        a.tofile(f)


def mvol_of(binv, root, aid, s, env):
    z = "/tmp/volmove-z.f32"
    open(z, "wb").write(bytes(8 * 512))
    r = subprocess.run([os.path.join(binv, "voicing-render"), "--tunings-root", root, "--aid", str(aid), "--in", z,
                        "--out", "/dev/null", "--mvsc", str(s), "--mvol", "auto"], capture_output=True, text=True, env=env, timeout=120)
    for line in r.stderr.splitlines():
        if line.startswith("mvol auto:") and "->" in line:
            return float(line.split("->")[1].split()[0])
    return None


def plan(a):
    vs = json.load(open(a.variants))
    env = {**os.environ, "SGV2_MACOS_BUILD": vs["build"]}
    out = []
    for v in vs["variants"]:
        if not v.get("key") or v.get("speaker_id") is not None:
            continue
        m0, m1 = mvol_of(a.bin, v["tree"], v["aid"], S0, env), mvol_of(a.bin, v["tree"], v["aid"], S1, env)
        if m0 is not None and m1 is not None:
            out.append({**v, "mvol": [m0, m1]})
    json.dump({"build": vs["build"], "law": vs["law"], "s": [S0, S1], "at": AT, "tunings": out}, open(a.out, "w"), indent=1)
    print(f"{len(out)} tunings planned")
    return 0


def refs(a):
    p = json.load(open(a.plan)); os.makedirs(a.out, mode=0o700, exist_ok=True)
    for name, tones in PROGRAMS.items():
        program(os.path.join(a.out, f"prog-{name}.f32"), tones)
    for t in p["tunings"]:
        for name in PROGRAMS:
            for kind in ("hold", "step"):
                mv = ["--param", f"mvsc={S0}", "--param", f"mvol={t['mvol'][0]}"]
                if kind == "step":
                    mv += ["--param-at", f"{AT}:mvsc={S1}", "--param-at", f"{AT}:mvol={t['mvol'][1]}"]
                base = os.path.join(a.out, f"AID{t['aid']}-{name}-{kind}")
                cmd = ["nice", "-n", "15", sys.executable, "-I", a.gr, "--tunings", a.tunings, "--aid", str(t["aid"]),
                       "--strip", t["strip"]] + (["--propstrip", t["propstrip"]] if t.get("propstrip") else []) + \
                      ["--in", os.path.join(a.out, f"prog-{name}.f32"), "--out", base + ".f32", "--meta", base + ".json",
                       "--box-host", a.box_host] + mv
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
                if r.returncode:
                    print(f"AID{t['aid']} {name} {kind} rc={r.returncode}", flush=True)
        print(f"AID{t['aid']} done", flush=True)
    return 0


def residuals(ms, mh, vs, vh, ch):
    import numpy as np
    out = []
    for c in range(ch):
        dm = ms[AT:AT + N, c] - mh[AT:AT + N, c]; dv = vs[AT:AT + N, c] - vh[AT:AT + N, c]
        em = float(dm @ dm)
        if em < 1e-24:
            out.append((None, -400.0)); continue
        g = float(dm @ dv) / (float(dv @ dv) + 1e-30)
        e = dm - g * dv
        out.append((10 * math.log10(float(e @ e) / em + 1e-30), 10 * math.log10(em / N + 1e-30)))
    return out


def gate(a):
    import numpy as np
    p = json.load(open(a.plan)); env = {**os.environ, "SGV2_MACOS_BUILD": p["build"]}
    for name, tones in PROGRAMS.items():
        program(f"/tmp/volmove-{name}.f32", tones)
    res = []
    for t in p["tunings"]:
        e = {"key": t["key"], "aid": t["aid"], "verdict": "refused", "reason": "", "lanes": {}}
        worst, at = -400.0, ""
        ok = True
        for name in PROGRAMS:
            base = os.path.join(a.refs, f"AID{t['aid']}-{name}")
            if not (os.path.exists(base + "-step.f32") and os.path.exists(base + "-hold.f32")):
                ok = False; e["reason"] = "no macOS reference"; break
            ch = json.load(open(base + "-step.json"))["channels"]
            rd = lambda f: np.fromfile(f, "<f4").astype(float).reshape(-1, ch)
            ms, mh = rd(base + "-step.f32"), rd(base + "-hold.f32")
            v = {}
            for kind in ("hold", "step"):
                o = f"/tmp/volmove-v2-{kind}.f32"
                cmd = [os.path.join(a.bin, "voicing-render"), "--tunings-root", t["tree"], "--aid", str(t["aid"]),
                       "--in", f"/tmp/volmove-{name}.f32", "--out", o, "--mvsc", str(S0), "--mvol", "auto"]
                if kind == "step":
                    cmd += ["--mvsc-ramp", f"{S1}:{S1 - S0}:{AT}"]
                if subprocess.run(cmd, capture_output=True, env=env).returncode:
                    ok = False; e["reason"] = "v2 does not render"; break
                v[kind] = rd(o); os.unlink(o)
            if not ok:
                break
            r = residuals(ms, mh, v["step"], v["hold"], ch)
            top = max(lv for _, lv in r)
            e["lanes"][name] = [None if x is None else round(x, 1) for x, _ in r]
            for c, (x, lv) in enumerate(r):
                if x is not None and lv >= top - LANE_DB and x > worst:
                    worst, at = x, f"{name} lane {c}"
        if ok:
            e["worst_db"] = round(worst, 1)
            e["verdict"] = "pass" if worst <= LIMIT_DB else "refused"
            e["reason"] = f"step response off macOS's by {worst:.1f} dB ({at})"
        res.append(e)
        print(f"AID{t['aid']:<3d} {e['verdict']:8s} {e['reason']}", flush=True)
    json.dump({"build": p["build"], "law": p["law"], "limit_db": LIMIT_DB, "s": p["s"], "entries": res}, open(a.out, "w"), indent=1)
    print(f"{sum(x['verdict'] == 'pass' for x in res)}/{len(res)} pass")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    x = sub.add_parser("plan"); [x.add_argument(f"--{k}", required=True) for k in ("variants", "bin", "out")]
    x = sub.add_parser("refs"); [x.add_argument(f"--{k}", required=True) for k in ("plan", "gr", "box-host", "tunings", "out")]
    x = sub.add_parser("gate"); [x.add_argument(f"--{k}", required=True) for k in ("plan", "refs", "bin", "out")]
    a = ap.parse_args()
    return {"plan": plan, "refs": refs, "gate": gate}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
