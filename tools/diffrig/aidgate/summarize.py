#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Per-AID table from gate.py results + the aidsweep JSON (numbers only, no tuning values).

    summarize.py GATE_OUT_DIR SWEEP_JSON [SWEEP_JSON_25G83] > table.md
"""
import json, os, sys


def main():
    gdir, sweeps = sys.argv[1], sys.argv[2:]
    sw = {}
    for p in sweeps:
        for a in json.load(open(p))["aids"]:
            sw.setdefault(a["aid"], a)
    rows, tot = [], {"PASS": 0}
    for f in sorted(os.listdir(gdir), key=lambda x: int(x[3:-5]) if x.startswith("AID") and x.endswith(".json") else 999):
        if not (f.startswith("AID") and f.endswith(".json")):
            continue
        g = json.load(open(os.path.join(gdir, f)))
        aid = g["aid"]
        s = sw.get(aid, {})
        c = s.get("counts", {})
        laws = ",".join(sorted(s.get("stub_reason_counts", {}))) or "-"
        w = g.get("worst") or {}
        st = g["status"]
        tot[st] = tot.get(st, 0) + 1
        over_boxes = [b["box"] for b in g.get("boxes", []) if b.get("over") or b.get("status")]
        rows.append(
            f"| {aid} | {'/'.join(s.get('units', [])) or '?'} | {c.get('refusals', '?')} | {c.get('stubs', '?')} | "
            f"{c.get('bounds', '?')} | {c.get('todo_spec', '?')} | **{st}** | {g.get('measured', '-')} | {g.get('over', '-')} | "
            f"{g.get('louder_over', '-')} | {w.get('db', float('nan')):+.3f} {w.get('seg', '')} | {g.get('lag_frames', '-')} | "
            f"{g.get('first_diverging_box') or '-'} | {','.join(over_boxes[:6]) or '-'} | {laws} |")
    print("| AID | unit | refusals | stubs | bounds | TODO(SPEC) | gate | measured | over | louder | worst dB (seg) | lag | first box over | boxes over | open law stubs |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    print("\n".join(rows))
    print()
    print("GATE TOTAL: " + ", ".join(f"{k} {v}" for k, v in sorted(tot.items())) + f" of {len(rows)} AIDs")


if __name__ == "__main__":
    main()
