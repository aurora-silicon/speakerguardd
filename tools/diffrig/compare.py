#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compare a macOS reference trace with an sgv2 trace, slice by slice.

Both inputs are JSON Lines in tools/diffrig/schema.json. Lines are matched
by (case, frame) after each side's reported latency. Exit status 0 only if
every field of every slice is within tolerance.

Ownership: lane C. TODO(lane C): mode-transition matching (same slice), the
per-case report, and tolerances from the behaviour specification
(TODO(SPEC)); the values below are the starting bar from the design.
"""
import json
import math
import sys

TOLERANCE = {
    "gain_db": 0.01,
    "coil_temp_c": 0.05,
    "magnet_temp_c": 0.05,
    "ol_temp_c": 0.05,
    "cl_temp_c": 0.05,
    "tlim_adj_c": 0.05,
    "modeled_power_w": ("rel", 1e-3),
    "measured_power_w": ("rel", 1e-3),
    "displacement_m": ("rel", 5e-3),
    "velocity_m_s": ("rel", 5e-3),
}


def load(path):
    out = {}
    with open(path) as fd:
        for line in fd:
            if line.strip():
                rec = json.loads(line)
                out[(rec["case"], rec["frame"] - rec.get("latency", 0))] = rec
    return out


def field_error(name, ref, got):
    if ref is None and got is None:
        return 0.0, True
    if ref is None or got is None:
        return math.inf, False
    if name == "gain":
        err = abs(20 * math.log10(max(got, 1e-12)) - 20 * math.log10(max(ref, 1e-12)))
        return err, err <= TOLERANCE["gain_db"]
    tol = TOLERANCE.get(name)
    if tol is None:
        return 0.0, True
    if isinstance(tol, tuple):
        err = abs(got - ref) / max(abs(ref), 1e-12)
        return err, err <= tol[1]
    err = abs(got - ref)
    return err, err <= tol


def compare(ref, got):
    worst = {}
    ok = True
    for key, r in ref.items():
        g = got.get(key)
        if g is None:
            ok = False
            worst.setdefault("missing_slices", 0)
            worst["missing_slices"] += 1
            continue
        for rc, gc in zip(r["channels"], g["channels"]):
            for name in set(rc) | set(gc):
                if isinstance(rc.get(name), list):
                    continue
                err, good = field_error(name, rc.get(name), gc.get(name))
                ok &= good
                worst[name] = max(worst.get(name, 0.0), err)
    return ok, worst


def self_test():
    a = {("c", 0): {"case": "c", "frame": 0, "channels": [{"gain": 0.5, "coil_temp_c": 60.0}]}}
    b = {("c", 0): {"case": "c", "frame": 0, "channels": [{"gain": 0.5, "coil_temp_c": 60.01}]}}
    assert compare(a, b)[0]
    b[("c", 0)]["channels"][0]["coil_temp_c"] = 61.0
    assert not compare(a, b)[0]
    assert not compare(a, {})[0]
    print("self-test OK")


def main():
    if sys.argv[1:] == ["--self-test"]:
        return self_test()
    if len(sys.argv) != 3:
        print("usage: compare.py MACOS.jsonl SGV2.jsonl", file=sys.stderr)
        return 2
    ok, worst = compare(load(sys.argv[1]), load(sys.argv[2]))
    print(json.dumps({"pass": ok, "worst": worst}, indent=2, sort_keys=True))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main() or 0)
