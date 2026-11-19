#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Whole-graph macOS reference render for ANY acoustic ID (stdlib only).

A port of tools/diffrig/macos/graph_render.py that needs no numpy and covers
every speaker graph in the 25G83 and 27.2 trees:
- `[if COND {A}{B}]` blocks (the bring-up branch is never taken: DEVICE_BRING_UP=0);
- `parameter_add` boxes (output 0 = sum of the inputs' graph parameters);
- `wireParam (SRC id ...) (DST id ...)`: the source box's settled parameter value
  (box_host on 2 s of silence) is set on the destination before it initialises;
- `reblocker` / `srlconverter` are wires; `delay` delays by its parameter;
- boxes without a fed input (telemetry, SysID) are skipped;
- rendering stops at the protection unit (atsp/spp3/lsm2): the output is the
  unit's input, one channel per unit channel.

Every Audio Unit box runs through the installed unit via box_host over the whole
signal. No playback, no audio device. Outputs made from Apple tuning are
Apple-derived: keep them in a private directory.

    gr.py --tunings DIR --aid N --in stereo.f32 --out unit_in.f32 --box-host BIN
          [--strip NAME] [--propstrip NAME] [--meta FILE] [--keep DIR] [--param NAME=VALUE ...]
          [--param-at FRAME:NAME=VALUE ...]

`--param-at` moves a graph parameter at run time (a host volume slider): every Audio Unit box
it is wired to gets the change before the host slice that starts at FRAME (box_host --set-at),
as macOS's graph does. The graph's own `dbgn` gain box ramps each change linearly in
amplitude over 20 ms, as macOS's does.

`--strip`/`--propstrip` name the speaker configuration's strip and property
strip (in AID<n>/DSP/Strips), as the DSP database selects them for a speaker
ID. The property strip's properties are set on each box as macOS sets them;
its `Path` values under /System/Library/Audio/Tunings are read from --tunings.
"""
import argparse, json, os, plistlib, re, shutil, struct, subprocess, sys, tempfile
from array import array

FACTORY = {
    "tmst": "AUToneMeisterFactory", "mstk": "AUMeisterStueckFactory",
    "vben": "AUVirtualBassFactory", "ldnm": "AULoudnessNormalizerFactory",
    "vrq2": "AUVariableEQV2Factory", "axtc": "AUXTCFactory",
    "cfrk": "AUControlFreakFactory", "buzk": "AUBuzzKillFactory",
    "cdly": "AUChannelSpecificDelayFactory", "volt": "AUVolumeTaperFactory",
    "flow": "AULowFlowFactory", "pchr": "AUMatrixPatcherFactory",
    "atsp": "AUThermalSpeakerProtectionFactory", "spp3": "AUSpeakerProtectionV3Factory",
    "lsm2": "AULoudspeakerManagerV2Factory", "lsid": "AULoudspeakerSystemIDFactory",
}
PROTECTION = {"atsp", "spp3", "lsm2"}
FULL_VOLUME = {"mvsc": 1.0, "mvol": 0.0}
WIRE_BUILTINS = {"reblocker", "srlconverter", "in", "out"}


def preprocess(text, defs_true=()):
    """Resolve [if ...]{A}{B}] blocks: the condition is false (no bring-up)."""
    out, stack = [], []  # stack of (taking_then?, in_else?)
    for raw in text.splitlines():
        line = raw.split(";", 1)[0].strip()
        if not line:
            continue
        active = all(take for take in stack)
        if line.startswith("[if"):
            cond = any(d in line for d in defs_true)
            stack.append(cond)
            continue
        if line.startswith("}{}]"):
            stack.pop()
            continue
        if line.startswith("}{"):
            stack[-1] = not stack[-1]
            continue
        if line.startswith("}]"):
            stack.pop()
            rest = line[2:].strip()
            if rest and all(take for take in stack):
                out.append(rest)
            continue
        if active:
            out.append(line)
    return out


def parse(lines):
    defs, boxes, wires, params, gparam, pwires = {}, {}, [], {}, [], []
    def sym(s):
        s = s.strip()
        for _ in range(8):
            if s.startswith("[") and s.endswith("]"):
                s = defs.get(s[1:-1], s[1:-1]).strip()
            else:
                break
        return s
    for line in lines:
        m = re.match(r"\[def\s+(\S+)\s+(\S+)\]", line)
        if m:
            defs[m.group(1)] = m.group(2)
            continue
        tok = line.split()
        if tok[0] in ("in", "out") and len(tok) == 2:
            boxes[tok[1]] = {"name": tok[1], "type": tok[0], "au": False}
        elif tok[0] == "box":
            m = re.match(r"box\s+(\S+)\s+(\([^)]*\)|\S+)\s+(\S+)\s+(\S+)", line)
            if not m:
                raise SystemExit(f"bad box line: {line}")
            name, kind = m.group(1), m.group(2)
            k = kind.strip("()").split()
            boxes[name] = {"name": name, "type": k[1] if len(k) > 1 else k[0], "au": len(k) > 1}
        elif tok[0] == "wire":
            m = re.match(r"wire\s+(\(\S+\s+\d+\)|\S+)\s+(\(\S+\s+\d+\)|\S+)\s*(.*)$", line)
            if not m:
                raise SystemExit(f"bad wire line: {line}")
            def port(s):
                s = s.strip("()").split()
                return s[0], int(s[1]) if len(s) > 1 else 0
            src, dst, fmt = m.groups()
            fmt = fmt.strip()
            if fmt.startswith("("):
                ch = fmt.strip("()").split()[-1]
                ch = int(sym(ch))
            else:
                ch = None  # named format (telemetry); never on the program path
            wires.append({"src": port(src), "dst": port(dst), "ch": ch})
        elif tok[0] == "param":
            try:
                params[sym(tok[1])] = float(tok[2])
            except ValueError:
                pass
        elif tok[0] == "wireGraphParam":
            m = re.match(r"wireGraphParam\s+(\S+)\s+\((\S+)\s+(\S+)(.*)\)", line)
            if not m:
                raise SystemExit(f"bad wireGraphParam line: {line}")
            rest = m.group(4).split()
            gparam.append((sym(m.group(1)), m.group(2), sym(m.group(3)), rest))
        elif tok[0] == "wireParam":
            m = re.match(r"wireParam\s+\((\S+)\s+(\S+)([^)]*)\)\s+\((\S+)\s+(\S+)([^)]*)\)", line)
            if not m:
                raise SystemExit(f"bad wireParam line: {line}")
            pwires.append({"src": m.group(1), "src_id": m.group(2), "src_rest": m.group(3).split(),
                           "dst": m.group(4), "dst_id": int(m.group(5)), "dst_rest": m.group(6).split()})
    return defs, boxes, wires, params, gparam, pwires


def strip_bypassed(strip_path):
    """Effects the strip marks bypassed (macOS runs them as the unit's bypass: identity)."""
    strip = plistlib.load(open(strip_path, "rb"))
    return {e.get("displayname") for st in strip["strips"] for e in st["effects"] if e.get("bypass") in (1, True)}


def preset_params(strip_path, name):
    """Global (scope 0, element 0) saved parameters of a strip effect's aupreset data blob."""
    strip = plistlib.load(open(strip_path, "rb"))
    for st in strip["strips"]:
        for e in st["effects"]:
            if e.get("displayname") == name:
                blob = e["aupreset"]["data"]
                out, at = {}, 0
                while at + 12 <= len(blob):
                    scope, element, n = struct.unpack_from(">III", blob, at)
                    at += 12
                    for _ in range(n):
                        pid, v = struct.unpack_from(">If", blob, at)
                        at += 8
                        if scope == 0 and element == 0:
                            out[pid] = v
                return out
    raise SystemExit(f"no preset for {name}")


def read_f32(path, n):
    a = array("f")
    with open(path, "rb") as f:
        a.frombytes(f.read())
    return [a[c::n] for c in range(n)]


def write_f32(path, chans):
    n, frames = len(chans), len(chans[0])
    a = array("f", bytes(4 * n * frames))
    for c, x in enumerate(chans):
        a[c::n] = x
    with open(path, "wb") as f:
        a.tofile(f)


class Renderer:
    def __init__(self, args, strip, work):
        self.args, self.strip, self.work = args, strip, work
        self.propstrip = None

    def box_host(self, name, sub, chans, sets, tag="run", set_ats=()):
        n = len(chans)
        fi, fo, fm = (os.path.join(self.work, f"{name}.{tag}.{e}") for e in ("in.f32", "out.f32", "json"))
        write_f32(fi, chans)
        cmd = ["nice", "-n", "15", self.args.box_host, "--strip", self.strip, "--stage", name,
               "--subtype", sub, "--factory", FACTORY[sub], "--channels", str(n),
               "--in", fi, "--out", fo, "--meta", fm, "--q", str(self.args.q), "--sr", str(self.args.sr)]
        if self.propstrip:
            cmd += ["--propstrip", self.propstrip]
        for pid, val in sets:
            cmd += ["--set", f"{pid}={val!r}"]
        for frame, pid, val in set_ats:
            cmd += ["--set-at", f"{frame}:{pid}={val!r}"]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        if r.returncode:
            raise SystemExit(f"box_host {name}: rc={r.returncode} {r.stderr.strip()[-400:]}")
        y = read_f32(fo, n)
        meta = json.load(open(fm))
        os.unlink(fi)
        if not self.args.keep:
            os.unlink(fo)
        return y, meta


def remap_propstrip(path, tunings, work):
    """The property strip with its Path values read from `tunings` (they name
    /System/Library/Audio/Tunings/...), written to `work`; the protection
    unit's own properties stay (gr.py never runs it)."""
    ps = plistlib.load(open(path, "rb"))
    sysroot = "/System/Library/Audio/Tunings/"
    for box in ps.get("Boxes", []):
        for p in box.get("Properties", []):
            v = p.get("Path")
            if isinstance(v, str) and v.startswith(sysroot):
                local = os.path.join(tunings, v[len(sysroot):])
                # a path the tree lacks stays: box_host fails on it only if
                # a box it renders needs it
                if os.path.exists(local):
                    p["Path"] = local
    out = os.path.join(work, "propstrip.plist")
    with open(out, "wb") as f:
        plistlib.dump(ps, f, fmt=plistlib.FMT_XML)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tunings", required=True)
    ap.add_argument("--aid", required=True, type=int)
    ap.add_argument("--graph", default="builtin_speaker_out.dspg")
    ap.add_argument("--strip", default="builtin_speaker_out_general.austrip")
    ap.add_argument("--propstrip")
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--meta")
    ap.add_argument("--box-host", required=True)
    ap.add_argument("--param", action="append", default=[])
    ap.add_argument("--param-at", action="append", default=[], help="FRAME:NAME=VALUE, a run-time graph parameter move")
    ap.add_argument("--q", type=int, default=512)
    ap.add_argument("--sr", type=float, default=48000.0)
    ap.add_argument("--keep")
    ap.add_argument("--parse-only", action="store_true")
    args = ap.parse_args()

    root = os.path.join(args.tunings, f"AID{args.aid}", "DSP")
    lines = preprocess(open(os.path.join(root, "Graphs", args.graph)).read())
    defs, boxes, wires, params, gparam, pwires = parse(lines)
    strip = os.path.join(root, "Strips", args.strip)
    params.update(FULL_VOLUME)
    for kv in args.param:
        k, v = kv.split("=")
        params[k] = float(v)
    if args.parse_only:
        au = sorted({b["type"] for b in boxes.values() if b.get("au")})
        bi = sorted({b["type"] for b in boxes.values() if not b.get("au")})
        print(json.dumps({"aid": args.aid, "au": au, "builtin": bi, "param_wires": len(pwires),
                          "unhandled": [t for t in au if t not in FACTORY]}))
        return 0
    work = args.keep or tempfile.mkdtemp(prefix="gr.")
    os.makedirs(work, exist_ok=True)
    R = Renderer(args, strip, work)
    if args.propstrip:
        R.propstrip = remap_propstrip(os.path.join(root, "Strips", args.propstrip), args.tunings, work)
    bypassed = strip_bypassed(strip)

    moves = []
    for kv in args.param_at:
        f, rest = kv.split(":", 1)
        k, v = rest.split("=")
        moves.append((int(f), k, float(v)))
    box_moves = {}
    for gname, bname, pid, rest in gparam:
        if bname in boxes and boxes[bname]["type"] != "parameter_add":
            for f, k, v in moves:
                if k == gname:
                    try:
                        box_moves.setdefault(bname, []).append((f, int(pid), v))
                    except ValueError:
                        pass
    box_sets, padd_in = {}, {}
    for gname, bname, pid, rest in gparam:
        if bname not in boxes:
            continue
        if boxes[bname]["type"] == "parameter_add":
            padd_in.setdefault(bname, []).append(params.get(gname, 0.0))
            continue
        try:
            box_sets.setdefault(bname, []).append((int(pid), params.get(gname, 0.0)))
        except ValueError:
            pass  # symbolic meter ids on the protection/SysID boxes
    notes = []
    # Box-to-box parameter wires: settle the source, then set the destination.
    for pw in pwires:
        src, dst = pw["src"], pw["dst"]
        if src not in boxes or dst not in boxes:
            continue
        if boxes[src]["type"] == "parameter_add":
            value = sum(padd_in.get(src, []))
        elif boxes[src].get("au"):
            ch = int(defs.get("numChansIn", 2))
            for w in wires:
                if w["dst"][0] == src and w["ch"]:
                    ch = w["ch"]
            silence = [array("f", bytes(4 * int(2 * args.sr))) for _ in range(ch)]
            _, meta = R.box_host(src, boxes[src]["type"], silence, box_sets.get(src, []), tag="settle")
            vals = {p["id"]: p["value"] for p in meta["parameters"]}
            value = vals.get(int(pw["src_id"]))
            if value is None:
                raise SystemExit(f"wireParam source {src} {pw['src_id']} has no settled value")
        else:
            continue
        box_sets.setdefault(dst, []).append((pw["dst_id"], value))
        notes.append({"wireParam": f"{src}.{pw['src_id']}->{dst}.{pw['dst_id']}", "value": value})

    nin = int(defs.get("numChansIn", 2))
    x = read_f32(args.inp, nin)
    frames = len(x[0])
    outputs = {("in", 0): x}
    done, metas, stop_box, unit_in = {"in"}, [], None, None
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
            box, t = boxes[name], boxes[name]["type"]
            ins = sorted(incoming.get(name, []), key=lambda w: w["dst"][1])
            ports = [outputs.get(w["src"]) for w in ins]
            fed = bool(ports) and all(p is not None for p in ports)
            for w, p in zip(ins, ports):
                if p is not None and w["ch"] is not None and len(p) != w["ch"]:
                    raise SystemExit(f"wire {w['src']}->{w['dst']}: {len(p)} ch, declared {w['ch']}")
            y = None
            if box.get("au") and t in PROTECTION and stop_box is None and any(p is not None for p in ports):
                # the program reaches the unit's audio port(s); telemetry stays unfed (None)
                stop_box = name
                unit_in = [c for p in ports if p is not None for c in p]
            elif not fed or stop_box is not None:
                y = None
            elif box.get("au") and name in bypassed:
                y = [c for p in ports for c in p]
                notes.append({"strip_bypass": name})
            elif box.get("au") and t == "pchr":
                # No installed factory on 25G83 (row 29, t-5535): model the saved 2x2 matrix
                # (params 0..3, row-major), exactly as v2 compiles it. Marked in the meta.
                m = preset_params(strip, name)
                x0, x1 = [c for p in ports for c in p]
                y = [array("f", (m[0] * a + m[1] * b for a, b in zip(x0, x1))),
                     array("f", (m[2] * a + m[3] * b for a, b in zip(x0, x1)))]
                notes.append({"modelled_box": name, "law": "pchr 2x2 matrix (no installed AU)"})
            elif box.get("au"):
                y, meta = R.box_host(name, t, [c for p in ports for c in p], box_sets.get(name, []),
                                     set_ats=box_moves.get(name, ()))
                metas.append({"box": name, "subtype": t, "latency_s": meta["latency_s"],
                              "tail_s": meta["tail_s"], "sets": box_sets.get(name, [])})
            elif t == "nfnp" or t in WIRE_BUILTINS:
                y = [c for p in ports for c in p]
            elif t in ("split", "chsp"):
                chans = ports[0]
                for i, c in enumerate(chans):
                    outputs[(name, i)] = [c]
            elif t in ("join", "chjn"):
                y = [c for p in ports for c in p]
            elif t in ("sum", "summ"):
                n = len(ports[0])
                y = []
                for c in range(n):
                    acc = array("f", ports[0][c])
                    for p in ports[1:]:
                        acc = array("f", (a + b for a, b in zip(acc, p[c])))
                    y.append(acc)
            elif t == "dbgn":
                s = dict(box_sets.get(name, []))
                g = 0.0 if s.get(1, 0.0) else 10 ** (s.get(0, 0.0) / 20)
                mv = sorted(box_moves.get(name, []))
                if not mv:
                    y = [c if g == 1.0 else array("f", (v * g for v in c)) for p in ports for c in p]
                else:
                    # run-time moves (macOS's graph gain box): each change ramps linearly in
                    # amplitude over floor(0.020 * fs) samples, the current value on the first
                    # sample; a change during a ramp starts its own ramp after it
                    nramp = max(1, int(0.020 * int(args.sr)))
                    y = []
                    for p in ports:
                        for c in p:
                            st = dict(s); cur = g; tgt = g; inc = 0.0; left = 0
                            pend = sorted(mv); out = array("f")
                            for n, xv in enumerate(c):
                                while pend and pend[0][0] <= n:
                                    _, pid, v = pend.pop(0); st[pid] = v
                                desired = 0.0 if st.get(1, 0.0) else 10 ** (st.get(0, 0.0) / 20)
                                if left == 0 and desired != tgt:
                                    tgt = desired; left = nramp; inc = (tgt - cur) / left
                                out.append(xv * cur)
                                if left:
                                    left -= 1
                                    cur = tgt if left == 0 else cur + inc
                            y.append(out)
                    notes.append({"dbgn_moves": name, "law": "linear amplitude ramp, 20 ms"})
            elif t == "delay":
                d = int(dict(box_sets.get(name, [])).get(0, 0))
                y = [array("f", bytes(4 * d)) + c[: len(c) - d] for p in ports for c in p]
            elif t in ("dead", "parameter_add", "dbcontrol"):
                y = None
            else:
                raise SystemExit(f"built-in box type {t} ({name}) is not handled")
            if y is not None and t not in ("split", "chsp"):
                outputs[(name, 0)] = y
            done.add(name)
            pending.remove(name)
            progressed = True
        if not progressed:
            raise SystemExit(f"graph does not resolve: {pending}")
    if unit_in is None:
        raise SystemExit("no protection unit reached")
    write_f32(args.out, unit_in)
    if args.meta:
        json.dump({"aid": args.aid, "frames": frames, "channels": len(unit_in), "stop": stop_box,
                   "params": params, "boxes": metas, "param_wires": notes}, open(args.meta, "w"), indent=1)
    if not args.keep:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
