#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run the Linux side of the differential rig against a set of macOS reference cases.

For every case directory under MACOS_DIR (as written by macos/capture.py: run.meta.json,
trace.jsonl, run.out.f32 and, for V/I units, run.tele.f32), render the same program
signal, and the same recorded sense, through sgv2-render. Then compare the traces
and audio with compare.py.

  run.py MACOS_DIR SIGNALS_DIR OUT_DIR [--render PATH] [--blob KEY=FILE ...]
         [--cal DIR] [--slices LIST] [--only SUBSTRING ...] [--keep-audio] [--plots]
  run.py --self-test

The run-time and case properties each reference was rendered with are listed,
with their SHA-256, in its run.meta.json (cfprops). The same fixtures, from
--cal DIR (default $SGV2_RIG_DIR/cal, else MACOS_DIR/../cal), are handed to
sgv2-render as --cfprop ID=FILE; a missing or changed fixture, or a reference
that does not record its properties, fails the case.

--blob KEY=FILE: KEY is the unit (lsm2) or the unit and the case's AID
(atsp-aid3); the AID-specific key wins.

AU parameter overrides a case was captured with (run.meta.json case.params
before initialisation, case.runtime_params after it, case.param_schedule at
frames) are handed to sgv2-render. A signal missing at the stream width is
read from its 2-channel file, stream channel c taking file channel c mod 2,
as the macOS host does.

Writes OUT_DIR/<case>/{trace.jsonl,report.json[,out.f32][,plots/]} and
OUT_DIR/summary.md (worst error per field, `!` outside tolerance, audio
max/RMS). Rendered audio is deleted after the comparison unless --keep-audio.
Exit status 0 only if every case passes.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import compare  # noqa: E402


def sha256(path):
    import hashlib

    h = hashlib.sha256()
    with open(path, "rb") as fd:
        for b in iter(lambda: fd.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def render_inputs(meta, cal):
    """sgv2-render arguments for the reference's resolved properties, or (None, why)."""
    if "cfprops" not in meta:
        return None, "the reference does not record its run-time properties (capture predates them); recapture"
    args = []
    for pr in meta["cfprops"]:
        path = os.path.join(cal, pr["fixture"])
        if not os.path.isfile(path):
            return None, f"fixture {pr['fixture']} for property {pr['id']} is not in {cal}"
        if sha256(path) != pr["sha256"]:
            return None, f"fixture {pr['fixture']} differs from the one the reference was rendered with"
        args += ["--cfprop", f"{pr['id']}={path}"]
    return args, None


def check_rendered_inputs(got, meta):
    """Every sgv2 slice must name the properties it was rendered with, in order."""
    want = [pr["id"] for pr in meta["cfprops"]]
    bad = next((k for k, r in sorted(got.items()) if r.get("cfprops") != want), None)
    if bad is not None:
        return f"sgv2 slice {bad} was rendered with properties {got[bad].get('cfprops')}, not {want}"
    return None


def fieldok(rep, name):
    return name not in rep.get("failed_fields", [])


def self_test():
    import tempfile

    d = tempfile.mkdtemp()
    fx = os.path.join(d, "fx.json")
    with open(fx, "w") as f:
        f.write("{}")
    meta = {"cfprops": [{"id": 64009, "fixture": "fx.json", "sha256": sha256(fx)}]}
    args, why = render_inputs(meta, d)
    assert args == ["--cfprop", f"64009={fx}"] and why is None, (args, why)
    assert render_inputs({}, d)[0] is None  # old capture: no recorded list
    assert render_inputs({"cfprops": []}, d) == ([], None)
    assert render_inputs(meta, os.path.join(d, "nowhere"))[0] is None
    with open(fx, "w") as f:
        f.write("{\"changed\": 1}")
    assert "differs" in render_inputs(meta, d)[1]
    got = {("c", 512): {"cfprops": [64009]}, ("c", 1024): {"cfprops": [64009]}}
    assert check_rendered_inputs(got, meta) is None
    got[("c", 1024)] = {}
    assert check_rendered_inputs(got, meta)
    assert check_rendered_inputs({("c", 512): {"cfprops": []}}, {"cfprops": []}) is None
    print("self-test OK")
    return 0


def main(argv):
    if argv == ["--self-test"]:
        return self_test()
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    macos, signals, out = argv[:3]
    rest = argv[3:]
    render = os.path.join(HERE, "..", "..", "target", "release", "sgv2-render")
    rig = os.environ.get("SGV2_RIG_DIR")
    cal = os.path.join(rig, "cal") if rig else os.path.join(macos, "..", "cal")
    blobs, only, slices, plots, keep = {}, [], None, False, False
    i = 0
    while i < len(rest):
        a = rest[i]
        if a == "--render":
            render = rest[i + 1]; i += 2
        elif a == "--blob":
            u, f = rest[i + 1].split("=", 1); blobs[u] = f; i += 2
        elif a == "--cal":
            cal = rest[i + 1]; i += 2
        elif a == "--slices":
            slices = rest[i + 1]; i += 2
        elif a == "--only":
            only.append(rest[i + 1]); i += 2
        elif a == "--keep-audio":
            keep = True; i += 1
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
        # The rig host reads a file of file_ch channels, stream channel c taking
        # file channel c mod file_ch.
        fch = meta.get("file_channels", c.get("file_ch", ch))
        if not os.path.exists(os.path.join(signals, f"{c['signal']}.{fch}ch.f32")):
            fch = 2
        args = [render, "--unit", unit, "--case", case,
                "--in", os.path.join(signals, f"{c['signal']}.{fch}ch.f32"), "--ch", str(ch), "--file-ch", str(fch),
                "--slices", slices or str(meta["quantum"]),
                "--out", os.path.join(o, "trace.jsonl"), "--audio-out", os.path.join(o, "out.f32")]
        tele = os.path.join(d, "run.tele.f32")
        if meta.get("tele_bus") and not os.path.isfile(tele) and os.path.isfile(tele + ".gz"):
            # compact captures keep the recorded sense gzipped; the renderer reads raw Float32
            import gzip
            import shutil
            tele_raw = os.path.join(o, "sense.f32")
            with gzip.open(tele + ".gz", "rb") as fi, open(tele_raw, "wb") as fo:
                shutil.copyfileobj(fi, fo)
            tele = tele_raw
        if meta.get("tele_bus") and os.path.isfile(tele):
            args += ["--sense", tele, "--sense-ch", "12"]
        aid = meta.get("aid", c.get("aid"))
        blob = blobs.get(f"{unit}-aid{aid}", blobs.get(unit))
        if blob:
            args += ["--blob", blob]
        if unit == "atsp" and meta.get("sr"):
            args += ["--rate", str(meta["sr"])]
        if c.get("limit_frames"):
            args += ["--limit-frames", str(c["limit_frames"])]
        for pp in c.get("preset_params", []):
            args += ["--preset-param", pp]
        for pp in c.get("params", []):
            args += ["--param", pp]
        for pp in c.get("params_at", []):
            args += ["--param-at", pp]
        if c.get("events"):
            args += ["--events", ",".join(c["events"])]
        props, why = render_inputs(meta, cal)
        if why:
            rows.append((case, False, "-", why))
            all_ok = False
            continue
        args += props
        p = subprocess.run(args, capture_output=True, text=True)
        if p.returncode:
            rows.append((case, False, "-", f"render failed: {p.stderr.strip()[:200]}"))
            all_ok = False
            continue
        ref, got = compare.load(os.path.join(d, "trace.jsonl")), compare.load(os.path.join(o, "trace.jsonl"))
        ok, rep = compare.compare(ref, got)
        why = check_rendered_inputs(got, meta)
        if why:
            ok = False
            rep["inputs"] = why
        lat = lambda t: next(iter(t.values())).get("latency", 0) if t else 0  # noqa: E731
        aok, arep = compare.compare_audio(os.path.join(d, "run.out.f32"), os.path.join(o, "out.f32"), ch,
                                          lat(ref), lat(got))
        if not keep:
            os.remove(os.path.join(o, "out.f32"))
            if os.path.exists(os.path.join(o, "sense.f32")):
                os.remove(os.path.join(o, "sense.f32"))
        rep["audio"] = arep
        ok = ok and aok
        rep["pass"] = ok
        with open(os.path.join(o, "report.json"), "w") as fd:
            json.dump(compare.jsonable(rep), fd, indent=1, sort_keys=True)
        if plots:
            compare.plot(ref, got, os.path.join(o, "plots"))
        worst = ", ".join(f"{k} {v['error']:.3g}" + ("" if fieldok(rep, k) else "!")
                          for k, v in sorted(rep["worst"].items()))
        missing = ", ".join(k for k, v in sorted(rep["fields"].items()) if v["missing"])
        skipped = ", ".join(sorted(rep["not_compared"]))
        note = "".join([f"; absent on sgv2: {missing}" if missing else "",
                        f"; not compared: {skipped}" if skipped else "",
                        f"; {rep['inputs']}" if rep.get("inputs") else ""])
        rows.append((case, ok, f"{arep.get('max_abs_error', float('nan')):.3g} / {arep.get('rms_error', float('nan')):.3g} "
                     f"(within FS {arep.get('max_abs_error_within_fs', float('nan')):.3g})",
                     f"{worst}{note}"))
        all_ok &= ok
    with open(os.path.join(out, "summary.md"), "w") as fd:
        fd.write("Units: dB for gains, degC for temperatures, relative for powers and mechanics, "
                 "linear for audio. `!` marks a field outside its tolerance.\n\n")
        fd.write("| case | pass | audio max / rms | worst error per field |\n|---|---|---|---|\n")
        for case, ok, a, w in rows:
            fd.write(f"| {case} | {'yes' if ok else 'no'} | {a} | {w} |\n")
    n_ok = sum(1 for r in rows if r[1])
    print(f"{n_ok}/{len(rows)} cases pass; summary in {os.path.join(out, 'summary.md')}")
    return 0 if all_ok and rows else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
