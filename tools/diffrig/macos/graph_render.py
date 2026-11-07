#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Offline macOS reference render of a Mac's whole speaker graph.

Reads the graph (`.dspg`) and strip (`.austrip`) of one acoustic ID from a
tuning folder at run time, walks the graph's wires in order, and renders
every Audio Unit box through the Mac's own installed unit (`box_host`, one
process per box over the whole signal). The graph's built-in boxes are done
here: `nfnp` (non-finite samples to 0), `split`/`chsp` (one channel per
output port), `join`/`chjn` (ports concatenated), `sum`/`summ` (ports
added), `dbgn` (dB gain from its graph parameters; 0 dB, unmuted at the
default full volume).

Graph-level parameters default to full system volume (`mvsc` 1, `mvol`
0 dB, everything else 0) and can be overridden with `--param NAME=VALUE`.
By default rendering stops before the protection unit (`atsp`, `spp3`,
`lsm2`): the output is the unit's input, one channel per unit channel,
which is what the voicing chain plus pre-stage must reproduce.

Nothing is played and no audio device is opened. Outputs made with Apple
tuning are Apple-derived data: keep them out of this repository.

    graph_render.py --tunings /System/Library/Audio/Tunings --aid 12 \
        --in stereo.f32 --out unit_in.f32 [--strip NAME] [--graph NAME] \
        [--param mvsc=1] [--through-protection] [--q 512] [--keep DIR]
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

FACTORY = {
    "tmst": "AUToneMeisterFactory",
    "mstk": "AUMeisterStueckFactory",
    "vben": "AUVirtualBassFactory",
    "ldnm": "AULoudnessNormalizerFactory",
    "vrq2": "AUVariableEQV2Factory",
    "axtc": "AUXTCFactory",
    "cfrk": "AUControlFreakFactory",
    "buzk": "AUBuzzKillFactory",
    "cdly": "AUChannelSpecificDelayFactory",
    "volt": "AUVolumeTaperFactory",
    "flow": "AULowFlowFactory",
    "atsp": "AUThermalSpeakerProtectionFactory",
    "spp3": "AUSpeakerProtectionV3Factory",
    "lsm2": "AULoudspeakerManagerV2Factory",
}
PROTECTION = {"atsp", "spp3", "lsm2"}
# Graph parameters at full system volume.
FULL_VOLUME = {"mvsc": 1.0, "mvol": 0.0}


def strip_comment(line):
    return line.split(";", 1)[0].strip()


def parse_dspg(text):
    defs, boxes, wires, params, gparam = {}, {}, [], {}, []
    for raw in text.splitlines():
        line = strip_comment(raw)
        if not line:
            continue
        m = re.match(r"\[def\s+(\S+)\s+(\S+)\]", line)
        if m:
            defs[m.group(1)] = m.group(2)
            continue
        tok = line.split()
        if tok[0] in ("in", "out") and len(tok) == 2:
            boxes[tok[1]] = {"name": tok[1], "type": tok[0], "nin": 1, "nout": 1}
        elif tok[0] == "box":
            m = re.match(r"box\s+(\S+)\s+(\([^)]*\)|\S+)\s+(\d+)\s+(\d+)", line)
            if not m:
                raise SystemExit(f"bad box line: {line}")
            name, kind, nin, nout = m.groups()
            k = kind.strip("()").split()
            boxes[name] = {
                "name": name,
                "type": k[1] if len(k) > 1 else k[0],
                "au": len(k) > 1,
                "nin": int(nin),
                "nout": int(nout),
            }
        elif tok[0] == "wire":
            m = re.match(r"wire\s+(\(\S+\s+\d+\)|\S+)\s+(\(\S+\s+\d+\)|\S+)\s+\((.*)\)", line)
            if not m:
                raise SystemExit(f"bad wire line: {line}")

            def port(s):
                s = s.strip("()").split()
                return s[0], int(s[1]) if len(s) > 1 else 0

            src, dst, fmt = m.groups()
            ch = fmt.split()[-1]
            ch = defs.get(ch.strip("[]"), ch.strip("[]"))
            wires.append({"src": port(src), "dst": port(dst), "ch": int(ch)})
        elif tok[0] == "param":
            params[tok[1]] = float(tok[2])
        elif tok[0] == "wireGraphParam":
            # (Box id) or (Box id scope element); only global scope 0 element 0
            m = re.match(r"wireGraphParam\s+(\S+)\s+\((\S+)\s+(\d+)(?:\s+(\d+)\s+(\d+))?\)", line)
            if not m:
                raise SystemExit(f"bad wireGraphParam line: {line}")
            if (m.group(4) or "0") != "0" or (m.group(5) or "0") != "0":
                raise SystemExit(f"graph parameter on a non-global scope/element: {line}")
            gparam.append((m.group(1), m.group(2), int(m.group(3))))
    return defs, boxes, wires, params, gparam


def render_au(box, x, strip, args, sets, work):
    sub = box["type"]
    if sub not in FACTORY:
        raise SystemExit(f"no factory known for aufx/{sub} ({box['name']})")
    n = x.shape[1]
    fi, fo, fm = (os.path.join(work, f"{box['name']}.{e}") for e in ("in.f32", "out.f32", "json"))
    x.astype("<f4").tofile(fi)
    cmd = [os.path.join(HERE, "box_host"), "--strip", strip, "--stage", box["name"],
           "--subtype", sub, "--factory", FACTORY[sub], "--channels", str(n),
           "--in", fi, "--out", fo, "--meta", fm, "--q", str(args.q), "--sr", str(args.sr)]
    for pid, val in sets:
        cmd += ["--set", f"{pid}={val}"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(f"box_host {box['name']}: {r.stderr.strip()}")
    y = np.fromfile(fo, dtype="<f4").reshape(-1, n)
    meta = json.load(open(fm))
    if not args.keep:
        for f in (fi, fo):
            os.unlink(f)
    return y, meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tunings", required=True)
    ap.add_argument("--aid", required=True, type=int)
    ap.add_argument("--graph", default="builtin_speaker_out.dspg")
    ap.add_argument("--strip", default="builtin_speaker_out_general.austrip")
    ap.add_argument("--in", dest="inp", required=True, help="interleaved f32, graph input channels")
    ap.add_argument("--out", required=True)
    ap.add_argument("--meta")
    ap.add_argument("--param", action="append", default=[])
    ap.add_argument("--through-protection", action="store_true")
    ap.add_argument("--bypass", action="append", default=[], help="treat this box as a wire (diagnostics)")
    ap.add_argument("--q", type=int, default=512)
    ap.add_argument("--sr", type=float, default=48000.0)
    ap.add_argument("--keep")
    args = ap.parse_args()

    root = os.path.join(args.tunings, f"AID{args.aid}", "DSP")
    defs, boxes, wires, params, gparam = parse_dspg(open(os.path.join(root, "Graphs", args.graph)).read())
    strip = os.path.join(root, "Strips", args.strip)
    params.update(FULL_VOLUME)
    for kv in args.param:
        k, v = kv.split("=")
        params[k] = float(v)
    box_sets = {}
    for gname, bname, pid in gparam:
        box_sets.setdefault(bname, []).append((pid, params.get(gname, 0.0)))

    nin = int(defs.get("numChansIn", 2))
    x = np.fromfile(args.inp, dtype="<f4").reshape(-1, nin)
    frames = x.shape[0]
    work = args.keep or tempfile.mkdtemp(prefix="graph_render.")
    os.makedirs(work, exist_ok=True)

    outputs = {("in", 0): x}  # (box, port) -> frames x ch
    done, metas, stop_box = {"in"}, [], None
    incoming = {}
    for w in wires:
        incoming.setdefault(w["dst"][0], []).append(w)

    def ready(name):
        return all(w["src"][0] in done for w in incoming.get(name, []))

    pending = [b for b in boxes if b != "in"]
    while pending:
        progressed = False
        for name in list(pending):
            if not ready(name):
                continue
            box = boxes[name]
            ins = sorted(incoming.get(name, []), key=lambda w: w["dst"][1])
            ports = []
            for w in ins:
                y = outputs[w["src"]]
                if y.shape[1] != w["ch"]:
                    raise SystemExit(f"wire {w['src']}->{w['dst']}: {y.shape[1]} ch, declared {w['ch']}")
                ports.append(y)
            t = box["type"]
            if not ports:
                y = None
            elif box.get("au") and t in PROTECTION and not args.through_protection:
                stop_box = name
                y = np.concatenate(ports, axis=1)
                outputs[("__unit_in__", 0)] = y
                y = np.zeros_like(y)
            elif box.get("au") and name in args.bypass:
                y = np.concatenate(ports, axis=1)
            elif box.get("au"):
                y, meta = render_au(box, np.concatenate(ports, axis=1), strip, args, box_sets.get(name, []), work)
                metas.append({"box": name, "subtype": t, "latency_s": meta["latency_s"], "tail_s": meta["tail_s"]})
            elif t == "nfnp":
                y = np.nan_to_num(ports[0], nan=0.0, posinf=0.0, neginf=0.0)
            elif t in ("split", "chsp"):
                y = ports[0]
                for i in range(y.shape[1]):
                    outputs[(name, i)] = y[:, i:i + 1]
            elif t in ("join", "chjn"):
                y = np.concatenate(ports, axis=1)
            elif t in ("sum", "summ"):
                y = np.sum(ports, axis=0)
            elif t == "dbgn":
                s = dict(box_sets.get(name, []))
                g = 0.0 if s.get(1, 0.0) else 10 ** (s.get(0, 0.0) / 20)
                y = ports[0] * g
            elif t in ("in", "out"):
                y = ports[0]
            else:
                raise SystemExit(f"built-in box type {t} ({name}) is not handled")
            if y is not None and t not in ("split", "chsp"):
                outputs[(name, 0)] = y
            done.add(name)
            pending.remove(name)
            progressed = True
        if not progressed:
            raise SystemExit(f"graph does not resolve: {pending}")

    if args.through_protection:
        y = outputs[("out", 0)]
    else:
        if stop_box is None:
            raise SystemExit("no protection unit in the graph")
        y = outputs[("__unit_in__", 0)]
    y.astype("<f4").tofile(args.out)
    if args.meta:
        json.dump({"aid": args.aid, "frames": frames, "channels": y.shape[1], "stop": stop_box,
                   "params": params, "boxes": metas}, open(args.meta, "w"), indent=1)
    if not args.keep:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
