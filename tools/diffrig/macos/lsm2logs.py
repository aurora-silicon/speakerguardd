#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Black-box decoder for the lsm2 debug logs the rig captures (LSMv2EnableLogging).

File layout: little-endian float32. Word 0 is the system (channel) count N. Then
records, each `n` (frames covered) followed by a payload whose size depends on the
log, N and n (the lsm2 behaviour specification, native logging contract).
"""
import argparse
import glob
import hashlib
import json
import os
from pathlib import Path
import sys
import numpy as np

# payload size as a function of (nch, n)
LAYOUT = {
    "ControllerThermalLog": lambda c, n: c * 5,
    "ControllerPowerLog": lambda c, n: c * 6 * n,
    "ControllerSignalLog": lambda c, n: c * 5 * n,
    "ControllerFrameLog": lambda c, n: 5 * c + 5,  # axis0..2, vel limit, final per system; 4 power limits; global
    "LsmFrameLog": lambda c, n: c,
    "LsmSignalLog": lambda c, n: c * n,
}

# per channel in ControllerThermalLog
THERMAL_FIELDS = ["theta_vc", "theta_mag", "ambient_c", "t_vc_c", "t_control_c"]
SIGNAL_LAYOUTS = ("output", "output-telemetry")


def _count(value, context):
    if not np.isfinite(value) or value <= 0 or value != int(value):
        raise ValueError(f"{context}: expected a positive integer, got {value}")
    return int(value)


def parse(path, name, *, signal_layout="output", strict=False):
    """Decode records; strict mode rejects unfinished logs instead of trimming them.

    Signal layout is supplied by the caller, never inferred from sample values.
    Payload NaNs/infinities are retained as evidence of native behavior.
    """
    if signal_layout not in SIGNAL_LAYOUTS:
        raise ValueError(f"unknown signal layout: {signal_layout}")
    if strict and os.path.getsize(path) % 4:
        raise ValueError(f"{name}: trailing partial float32 word")
    a = np.fromfile(path, "<f4")
    if not a.size:
        if strict:
            raise ValueError(f"{name}: missing channel count")
        return 0, []
    nch = _count(a[0], f"{name} channel count")
    if strict and a.size == 1:
        raise ValueError(f"{name}: no records")
    f = LAYOUT[name]
    factor = 2 if name == "LsmSignalLog" and signal_layout == "output-telemetry" else 1
    i, out, frames = 1, [], 0
    while i < a.size:
        n = _count(a[i], f"{name} record length at word {i}")
        sz = factor * f(nch, n)
        if i + 1 + sz > a.size:
            if strict:
                raise ValueError(f"{name}: incomplete record at word {i}: expected {sz} payload words")
            break  # trailing partial record
        # NumPy frame offsets use int64. Thermal/frame records can describe a
        # huge frame count with a tiny payload, so file size is not a bound.
        if n > np.iinfo(np.int64).max - frames:
            raise ValueError(f"{name}: cumulative frame count exceeds int64 at word {i}")
        out.append((n, a[i + 1:i + 1 + sz]))
        frames += n
        i += 1 + sz
    return nch, out


def load_dir(d, *, signal_layout="output", strict=False):
    res = {}
    for name in LAYOUT:
        g = sorted(glob.glob(os.path.join(d, f"*_{name}.bin")))
        if not g:
            continue
        if strict and len(g) != 1:
            raise ValueError(f"{name}: ambiguous input ({len(g)} files)")
        nch, recs = parse(g[0], name, signal_layout=signal_layout, strict=strict)
        frames = np.cumsum([0] + [n for n, _ in recs])[:-1]
        res[name] = {"nch": nch, "frame": frames, "n": np.array([n for n, _ in recs]),
                     "data": [p for _, p in recs], "path": g[0]}
    res["overwrites"] = len(glob.glob(os.path.join(d, "*OVERWRITE*")))
    return res


def thermal_array(res):
    t = res["ControllerThermalLog"]
    return t["frame"], np.stack(t["data"]).reshape(len(t["data"]), t["nch"], 5)


def signal_arrays(res, signal_layout):
    """Return frames-by-channels arrays from channel-major native records."""
    if signal_layout not in SIGNAL_LAYOUTS:
        raise ValueError(f"unknown signal layout: {signal_layout}")
    t = res.get("LsmSignalLog")
    if t is None or not t["data"]:
        raise ValueError("LsmSignalLog: no complete signal records to export")
    lanes = 2 if signal_layout == "output-telemetry" else 1
    blocks = [p.reshape(t["nch"], lanes, int(n)) for n, p in zip(t["n"], t["data"])]
    arrays = {"output": np.concatenate([b[:, 0, :].T for b in blocks]),
              "record_start": t["frame"]}
    if lanes == 2:
        arrays["telemetry"] = np.concatenate([b[:, 1, :].T for b in blocks])
    return arrays


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("directory", type=Path)
    ap.add_argument("--signal-layout", choices=SIGNAL_LAYOUTS, default="output",
                    help="explicit LsmSignalLog payload layout; never auto-detected")
    ap.add_argument("--strict", action="store_true",
                    help="reject partial words/records, empty logs and ambiguous inputs")
    ap.add_argument("--summary", type=Path, help="write input hashes and record counts as JSON")
    ap.add_argument("--signals", type=Path, help="export decoded signal arrays as NPZ")
    args = ap.parse_args()
    try:
        r = load_dir(args.directory, signal_layout=args.signal_layout, strict=args.strict)
        logs = {k: v for k, v in r.items() if k != "overwrites"}
        if not logs:
            raise ValueError("no recognized log files")
        arrays = signal_arrays(r, args.signal_layout) if args.signals else None
        inputs = {Path(v["path"]).resolve() for v in logs.values()}
        outputs = [p.resolve() for p in (args.summary, args.signals) if p is not None]
        if len(set(outputs)) != len(outputs) or inputs.intersection(outputs):
            raise ValueError("artifact paths must be distinct from each other and the input logs")
        report = {"signal_layout": args.signal_layout, "strict": args.strict,
                  "overwrites": r["overwrites"], "numpy": np.__version__,
                  "decoder_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  "logs": {}}
        for name, v in logs.items():
            report["logs"][name] = {
                "file": Path(v["path"]).name,
                "sha256": hashlib.sha256(Path(v["path"]).read_bytes()).hexdigest(),
                "channels": v["nch"], "records": len(v["data"]),
                "frames": int(v["n"].sum()),
                "nonfinite_payload_words": sum(int((~np.isfinite(p)).sum()) for p in v["data"]),
            }
        # Finish parsing/validation before creating any success artifacts.
        if args.signals:
            with args.signals.open("wb") as f:
                np.savez(f, **arrays)
        if args.summary:
            args.summary.write_text(json.dumps(report, indent=2) + "\n")
    except (OSError, ValueError) as exc:
        ap.exit(1, f"lsm2logs: {exc}\n")
    for k, v in r.items():
        if k == "overwrites":
            print("overwrites", v)
            continue
        ns = np.unique(v["n"])
        print(k, "records", len(v["data"]), "n values", ns, "frames covered", int(v["n"].sum()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
