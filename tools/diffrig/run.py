#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run the Linux side of the differential rig against a set of macOS reference cases.

For every case directory under MACOS_DIR (as written by macos/capture.py: run.meta.json,
trace.jsonl, run.out.f32 and, for V/I units, run.tele.f32), render the same program
signal, and the same recorded sense, through sgv2-render. Then compare the traces
and audio with compare.py.

  run.py MACOS_DIR SIGNALS_DIR OUT_DIR [--render PATH] [--blob UNIT=FILE ...]
         [--slices LIST] [--only SUBSTRING ...] [--plots]

Writes OUT_DIR/<case>/{trace.jsonl,out.f32,report.json[,plots/]} and OUT_DIR/summary.md.
Exit status 0 only if every case passes.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import compare  # noqa: E402


def main(argv):
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    macos, signals, out = argv[:3]
    rest = argv[3:]
    render = os.path.join(HERE, "..", "..", "target", "release", "sgv2-render")
    blobs, only, slices, plots = {}, [], None, False
    i = 0
    while i < len(rest):
        a = rest[i]
        if a == "--render":
            render = rest[i + 1]; i += 2
        elif a == "--blob":
            u, f = rest[i + 1].split("=", 1); blobs[u] = f; i += 2
        elif a == "--slices":
            slices = rest[i + 1]; i += 2
        elif a == "--only":
            only.append(rest[i + 1]); i += 2
        elif a == "--plots":
            plots = True; i += 1
        else:
            print(f"unknown argument {a}", file=sys.stderr)
            return 2
    os.makedirs(out, exist_ok=True)
    rows, all_ok = [], True
    for case in sorted(os.listdir(macos)):
        d = os.path.join(macos, case)
        meta_path = os.path.join(d, "run.meta.json")
        if not os.path.isfile(meta_path) or (only and not any(s in case for s in only)):
            continue
        meta = json.load(open(meta_path))
        c = meta["case"]
        unit, ch = meta["unit"], meta["channels"]
        o = os.path.join(out, case)
        os.makedirs(o, exist_ok=True)
        args = [render, "--unit", unit, "--case", case,
                "--in", os.path.join(signals, f"{c['signal']}.{ch}ch.f32"), "--ch", str(ch),
                "--slices", slices or str(meta["quantum"]),
                "--out", os.path.join(o, "trace.jsonl"), "--audio-out", os.path.join(o, "out.f32")]
        tele = os.path.join(d, "run.tele.f32")
        if meta.get("tele_bus") and os.path.isfile(tele):
            args += ["--sense", tele, "--sense-ch", "12"]
        if unit in blobs:
            args += ["--blob", blobs[unit]]
        p = subprocess.run(args, capture_output=True, text=True)
        if p.returncode:
            rows.append((case, False, f"render failed: {p.stderr.strip()[:200]}"))
            all_ok = False
            continue
        ref, got = compare.load(os.path.join(d, "trace.jsonl")), compare.load(os.path.join(o, "trace.jsonl"))
        ok, rep = compare.compare(ref, got)
        lat = lambda t: next(iter(t.values())).get("latency", 0) if t else 0  # noqa: E731
        aok, arep = compare.compare_audio(os.path.join(d, "run.out.f32"), os.path.join(o, "out.f32"), ch,
                                          lat(ref), lat(got))
        rep["audio"] = arep
        ok = ok and aok
        rep["pass"] = ok
        with open(os.path.join(o, "report.json"), "w") as fd:
            json.dump(compare.jsonable(rep), fd, indent=1, sort_keys=True)
        if plots:
            compare.plot(ref, got, os.path.join(o, "plots"))
        worst = ", ".join(f"{k} {v['error']:.3g}" for k, v in sorted(rep["worst"].items()))
        rows.append((case, ok, f"audio {arep.get('max_abs_error', float('nan')):.3g}; {worst}"))
        all_ok &= ok
    with open(os.path.join(out, "summary.md"), "w") as fd:
        fd.write("| case | pass | worst error per field |\n|---|---|---|\n")
        for case, ok, w in rows:
            fd.write(f"| {case} | {'yes' if ok else 'no'} | {w} |\n")
    n_ok = sum(1 for r in rows if r[1])
    print(f"{n_ok}/{len(rows)} cases pass; summary in {os.path.join(out, 'summary.md')}")
    return 0 if all_ok and rows else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
