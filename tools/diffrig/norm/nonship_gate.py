#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Gate the non-ship Norm algorithm-1 and mono long-path states.

Reference audio, strips and results stay in the caller's private buildbox
directories. This driver contains no tuning values or reference audio.
"""

import argparse
import importlib.util
import json
import pathlib


def link(target: pathlib.Path, source: pathlib.Path) -> None:
    if target.is_symlink():
        target.unlink()
    if not target.exists():
        target.symlink_to(source, target_is_directory=source.is_dir())


def run_rows(boxrig, root: pathlib.Path, rows: list[dict], tag: str) -> dict:
    results = []
    for row in rows:
        row = dict(row)
        row["v2_rc"] = 0
        result = boxrig.render_row(root, row, tag, True)
        print(row["id"], result["cases"], result["over"], result["worst_db"], flush=True)
        results.append(result)
    return {
        "states": len(results),
        "cases": sum(row["cases"] for row in results),
        "over": sum(row["over"] for row in results),
        "unmeasured": sum(row["planned"] - row["cases"] for row in results),
        "worst_db": max(row["worst_db"] for row in results),
        "failing_states": [
            {key: row[key] for key in ("id", "aid", "over", "worst_db", "worst_case", "signature")}
            for row in results
            if row["over"]
        ],
    }


def prepare(root: pathlib.Path, base: pathlib.Path, renderer: pathlib.Path, mac: pathlib.Path) -> None:
    (root / "bin").mkdir(parents=True, exist_ok=True)
    for name in ("strips", "inputs", "probes.json", ".venv", "boxrig.py"):
        link(root / name, base / name)
    link(root / "mac", mac)
    link(root / "bin/voicing-box-render", renderer)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--boxrig-root", type=pathlib.Path, required=True)
    parser.add_argument("--t5513-root", type=pathlib.Path, required=True)
    parser.add_argument("--renderer", type=pathlib.Path, required=True)
    parser.add_argument("--work", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()

    spec = importlib.util.spec_from_file_location("boxrig", args.boxrig_root / "boxrig.py")
    boxrig = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(boxrig)

    full = json.loads((args.t5513_root / "full-inventory.json").read_text())
    algorithm1_rows = [row for row in full if row["type"] == "ldnm" and row["aid"] in (3, 4, 5, 7, 18)]
    algorithm1_root = args.work / "algorithm1"
    prepare(algorithm1_root, args.boxrig_root, args.renderer, args.t5513_root / "full-mac")

    base = json.loads((args.boxrig_root / "inventory.json").read_text())["rows"]
    row28_rows = [row for row in base if row["type"] == "ldnm" and row["aid"] in (10, 25, 32, 33, 47)]
    row28_root = args.work / "row28"
    prepare(row28_root, args.boxrig_root, args.renderer, args.boxrig_root / "mac")

    result = {
        "algorithm1": run_rows(boxrig, algorithm1_root, algorithm1_rows, "algorithm1"),
        "row28": run_rows(boxrig, row28_root, row28_rows, "row28"),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=1))
    print(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
