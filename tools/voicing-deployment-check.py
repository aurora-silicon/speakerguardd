#!/usr/bin/env python3
"""Compare the reference example with the deployed LV2 ABI offline.

Fixtures are generated from public synthetic bundles by voicing_deploy.rs:
SGV2_EQ_DIR=DIR cargo test -p aurora-speaker-tuning --test voicing_deploy
No user bundle, hardware or Apple tuning is needed for this check.
"""
import argparse
import array
import json
import math
import os
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fixtures", required=True)
    ap.add_argument("--render", required=True)
    ap.add_argument("--plugin", required=True)
    args = ap.parse_args()
    rows = []
    for board in ("j616s", "j316s", "j413"):
        root = os.path.join(args.fixtures, board)
        for scalar, db in ((0.25, -36), (0.5, -18), (0.75, -6), (1.0, 0)):
            src = os.path.join(root, "program.f32")
            ref, live = [os.path.join(root, f"{scalar}-{kind}.f32") for kind in ("reference", "plugin")]
            subprocess.run([args.render, "--bundle", os.path.join(root, "bundle"), "--in", src, "--out", ref,
                            "--mvsc", str(scalar), "--mvol", str(db)], check=True)
            subprocess.run([args.plugin, "--params", os.path.join(root, "params.bin"), "--board", board,
                            "--in", src, "--out", live, "--mvsc", str(scalar), "--mvol", str(db)], check=True)
            data = []
            for path in (ref, live):
                with open(path, "rb") as f:
                    a = array.array("f")
                    a.frombytes(f.read())
                if sys.byteorder != "little":
                    a.byteswap()
                if not all(math.isfinite(v) for v in a):
                    raise RuntimeError(f"nonfinite output: {path}")
                data.append(a)
            if len(data[0]) != len(data[1]) or not data[0]:
                raise RuntimeError("output length mismatch or empty output")
            error = max(abs(a-b) for a, b in zip(*data))
            row = {"board": board, "scalar": scalar, "db": db, "samples": len(data[0]),
                   "max_abs_error": error, "bit_identical": data[0].tobytes() == data[1].tobytes()}
            rows.append(row)
            if error > 1e-6:
                raise RuntimeError(row)
    report = {"fixture_source": "public synthetic bundles; board geometry only", "cases": rows}
    with open(os.path.join(args.fixtures, "equivalence.json"), "w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
