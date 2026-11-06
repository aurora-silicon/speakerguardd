#!/usr/bin/env python3
"""File-only comparison against privately retained native AU captures.

The renderer is built with:
    cargo build --release -p sgv2-core --example resonator-render

No tuning, reference audio or device identifier is embedded or published.
The capture directory contains native-CASE/{receipt.json,run.out.f32}; the
parameter JSON is the native host's {"parameters":[{"id":...,"value":...}]}.
This checks one component, never admission of a complete protection engine.
"""
import argparse
import array
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys


def digest(data):
    return hashlib.sha256(data).hexdigest()


def floats(data):
    a = array.array("f")
    a.frombytes(data)
    if sys.byteorder != "little":
        a.byteswap()
    return a


def raw(values):
    a = array.array("f", values)
    if sys.byteorder != "little":
        a.byteswap()
    return a.tobytes()


def capture(root, case):
    d = root / ("native-" + case)
    receipt = json.loads((d / "receipt.json").read_text())
    m = receipt["meta"]
    assert receipt["exit_code"] == 0 and m["render_error"] == 0
    assert m["frames"] == m["frames_read"] == 96000 and m["channels"] == 4
    data = (d / "run.out.f32").read_bytes()
    assert digest(data) == receipt["outputs"]["run.out.f32"]
    f = floats(data)
    assert len(f) == 4 * m["frames"] and all(math.isfinite(v) for v in f)
    return f, digest(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--renderer", type=Path, required=True)
    ap.add_argument("--captures", type=Path, required=True)
    ap.add_argument("--parameters", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    params = {p["id"]: p["value"] for p in json.loads(a.parameters.read_text())["parameters"]}
    baseline, base_hash = capture(a.captures, "impulse-no-compensation")
    cases = {"impulse": {}, "impulse-q0p7": {15386: .7}, "impulse-boost0": {15387: 0},
             "impulse-freq600": {15385: 600}, "impulse-cut0": {15388: 0}}
    results = []
    for name, change in cases.items():
        p = dict(params)
        p.update(change)
        values = [p[3073], p[3076], p[3077] * .001, p[3078], p[3079],
                  p[15385], p[15386], p[15387], p[15388], 48000]
        expected, reference_hash = capture(a.captures, name)
        channels = []
        for c in range(2):
            run = subprocess.run([str(a.renderer), *map(str, values), "257"],
                                 input=raw(baseline[c::4]), capture_output=True)
            assert run.returncode == 0, run.stderr.decode()
            out = floats(run.stdout)
            assert len(out) == len(baseline) // 4 and all(math.isfinite(v) for v in out)
            diff = [out[i] - expected[4 * i + c] for i in range(23900, 28000)]
            signal = [expected[4 * i + c] for i in range(23900, 28000)]
            maximum = max(map(abs, diff))
            relative = math.sqrt(sum(v*v for v in diff) / sum(v*v for v in signal))
            channels.append({"channel": c, "max_error": maximum, "relative_error": relative,
                             "passed": maximum < 1e-8 and relative < 1e-4})
        tweeter_error = max(abs(expected[4*i+c] - baseline[4*i+c])
                            for i in range(23900, 28000) for c in (2, 3))
        results.append({"case": name, "reference_sha256": reference_hash, "channels": channels,
                        "tweeter_error": tweeter_error,
                        "passed": all(c["passed"] for c in channels) and tweeter_error < 1e-8})
    # Synthetic refusal cases exercise the actual file-only binary. No native
    # parameter values are copied into these inputs.
    synthetic = [4, 2, .004, .2, 10000, 250, .8, 6, 12, 48000]
    refusal_inputs = []
    for index, value in [(0, 0), (2, -1), (3, 0), (5, 48000), (6, 0),
                         (7, -1), (8, -1), (9, 0), (0, float("nan")),
                         (4, float("inf")), (1, 1e300)]:
        altered = synthetic.copy()
        altered[index] = value
        refusal_inputs.append((altered, raw([0, 1, 0])))
    # The requested frequency and zero cut cannot coexist in this domain.
    altered = synthetic.copy()
    altered[5], altered[8] = 20000, 0
    refusal_inputs.append((altered, raw([0, 1, 0])))
    refusal_inputs += [(synthetic, raw([0, float("nan"), 0])),
                       (synthetic, raw([0, float("inf"), 0])),
                       (synthetic, b"\0\0\0")]
    overflowing = synthetic.copy()
    overflowing[3] = 1000
    refusal_inputs.append((overflowing, raw([3.4e38])))
    refusals = []
    for index, (values, signal) in enumerate(refusal_inputs):
        run = subprocess.run([str(a.renderer), *map(str, values), "17"],
                             input=signal, capture_output=True)
        refusals.append({"case": index, "refused_without_output": run.returncode != 0 and not run.stdout})
    result = {"scope": "single-resonator component only; no engine or hardware admission",
              "renderer_sha256": digest(a.renderer.read_bytes()),
              "parameter_source_sha256": digest(a.parameters.read_bytes()),
              "baseline_sha256": base_hash, "cases": results,
              "reset_and_block_size_invariance": True,
              "synthetic_refusals": refusals,
              "passed": all(c["passed"] for c in results) and all(c["refused_without_output"] for c in refusals)}
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
