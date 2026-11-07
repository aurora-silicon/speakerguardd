#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Assemble public-safe measured reports and a structural catalogue.

This is report generation only: it does not execute a compiler or infer an
unobserved DSP law. Missing execution remains explicit in the output.
"""
import argparse
from collections import defaultdict
import hashlib
import importlib.util
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalogue", required=True, type=Path)
    parser.add_argument("--measured", action="append", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("inventory", Path(__file__).with_name("aid-inventory.py"))
    inventory = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(inventory)
    catalogue = json.loads(args.catalogue.read_text())
    reports = [json.loads(path.read_text()) for path in args.measured]
    revisions = {r["source_revision"] for r in reports}
    if len(revisions) != 1:
        raise RuntimeError("Measured reports use different core revisions")
    rows = {}
    for report in reports:
        for row in report["aids"]:
            if row["aid"] is None:
                continue
            key = row["aid"], row["source"]
            if key in rows and rows[key] != row:
                raise RuntimeError("Conflicting measured records for one AID/source")
            rows[key] = row
    measured = sorted(rows.values(), key=lambda r: (r["aid"], r["source"]))
    data = inventory.report(measured, revisions.pop())
    membership = catalogue["membership"]
    reference = set(membership["25G83"])
    newer_only = set(membership["only_in_27_2"])
    executed = {r["aid"] for r in measured if r["source"] == "host"}
    newer_executed = {r["aid"] for r in measured if r["source"].startswith("27.")}
    missing_reference = sorted(reference - executed)
    missing_newer = sorted(newer_only - newer_executed)
    structural = {r["aid"]: r for r in catalogue["reference_structural_declarations"]}
    structural.update({r["aid"]: r for r in catalogue["newer_structural_declarations"]})
    controls = {r["aid"]: r for r in catalogue.get("saved_control_metadata", [])}
    unexecuted = []
    for aid in sorted(set(missing_reference) | set(missing_newer)):
        boxes = []
        for box in structural.get(aid, {}).get("boxes", []):
            unknown = box["kind"] in {"lsid", "dbcontrol"}
            boxes.append({"name": box["name"], "subtype": box["kind"],
                          "status": "unimplemented" if unknown else "unexecuted",
                          "evidence": "static declaration; no compiler/render outcome"})
        board, model = inventory.KNOWN_BOARDS.get(aid, (None, None))
        kinds = {box["subtype"] for box in boxes}
        unexecuted.append({"aid": aid, "only_in_27_2": aid in newer_only,
                           "board": board, "model": model,
                           "declared_units": sorted(kinds & {"spp3", "atsp", "lsm2", "lsid"}),
                           "declared_presence": {name: kind in kinds for name, kind in
                               [("LowFlow", "flow"), ("Norm", "ldnm"), ("VirtBass", "vben"), ("XTC", "axtc")]},
                           "boxes": boxes, "saved_control_metadata": controls.get(aid),
                           "execution_status": "not_run"})
    rollup = defaultdict(set, {law: set(aids) for law, aids in data["per_law"].items()})
    for row in catalogue.get("known_unimplemented", []):
        rollup[row["kind"] + "-unimplemented"].add(row["aid"])
    for row in unexecuted:
        for box in (row.get("saved_control_metadata") or {}).get("boxes", []):
            types = box.get("tmst", {}).get("state_and_group_enabled_type_counts", {})
            for kind in ("15", "18"):
                if types.get(kind, 0):
                    rollup["tmst-type" + kind + "-coverage-unverified"].add(row["aid"])
    data.update({"schema_version": 2, "status": "partial" if unexecuted else "completed",
                 "reference_membership": membership, "unexecuted_aids": unexecuted,
                 "coverage": {"reference_aids": len(reference), "measured_reference_aids": len(reference & executed),
                              "missing_reference_aids": missing_reference,
                              "newer_only_aids": sorted(newer_only), "missing_newer_aids": missing_newer},
                 "per_law": {law: sorted(aids) for law, aids in sorted(rollup.items())},
                 "input_report_sha256": {p.name + ":" + str(i): hashlib.sha256(p.read_bytes()).hexdigest()
                                         for i, p in enumerate(args.measured)},
                 "catalogue_sha256": hashlib.sha256(args.catalogue.read_bytes()).hexdigest(),
                 "assembler_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
    data["limitations"] += [
        "Measured counts cover only executed reference AIDs; source availability is not an AID refusal.",
        "Unexecuted rows retain structural facts and explicit implementation gaps; no renderer execution is inferred.",
    ]
    counts = data["counts"]
    lines = ["# AID exact-parity inventory", "", "Status: **" + data["status"] + "**. 25G83 remains the parity reference.", "",
             f"Measured reference coverage: {len(reference & executed)}/{len(reference)} AIDs. " +
             "Exclusive measured counts: " + ", ".join(f"{k}={v}" for k, v in counts.items()) + ".", "",
             "Unexecuted reference AIDs: " + ", ".join(map(str, missing_reference)) + ". " +
             "Newer-only AIDs awaiting execution: " + ", ".join(map(str, missing_newer)) + ".", "",
             "The unexecuted rows below are structural observations, not compiler refusals or parity certificates.", "",
             "| AID | Known representative board / model | 25G83 parity reference | 27.2 presence | Only in 27.2 |",
             "|---|---|---|---|---|"]
    hosts = {r["aid"]: r for r in measured if r["source"] == "host"}
    newer_members = set(membership.get("27.2-26B8125", [])) | set(membership.get("27.2-26B123", []))
    for aid in sorted(reference | newer_only):
        state = hosts[aid]["category"] if aid in hosts else "not run" if aid in reference else "absent"
        board, model = inventory.KNOWN_BOARDS.get(aid, (None, None))
        identity = (board + " / " + model) if board and model else "unknown"
        lines.append(f'| {aid} | {identity} | {state} | {"present" if aid in newer_members else "absent"} | {"yes" if aid in newer_only else "no"} |')
    lines += ["", "## Unexecuted structural inventory", "",
              "| AID | Box | Type | Law/coverage status |", "|---|---|---|---|"]
    for row in unexecuted:
        for box in row["boxes"]:
            lines.append(f'| {row["aid"]} | {box["name"]} | {box["subtype"]} | {box["status"]}; declaration only |')
    lines += ["", "### Unexecuted saved controls", "",
              "These are saved general-strip enums and state/group checks. They do not establish renderer execution.", "",
              "| AID | Box | Saved mstk algorithm/quality/ovs | tmst state/group-enabled types |",
              "|---|---|---|---|"]
    for row in unexecuted:
        for box in (row.get("saved_control_metadata") or {}).get("boxes", []):
            control = box.get("saved_mstk")
            saved = "/".join(str(control[k]) for k in ("algorithm", "quality", "ovs")) if control else ""
            types = box.get("tmst", {}).get("state_and_group_enabled_type_counts", {})
            lines.append(f'| {row["aid"]} | {box["name"]} | {saved} | ' + json.dumps(types, sort_keys=True) + " |")
    lines += ["", "## Measured inventory and complete per-law rollup", "",
              inventory.markdown(data).replace("# AID exact-parity inventory", "### Measured report", 1)]
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "aid-inventory.json").write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    (args.out_dir / "aid-inventory.md").write_text("\n".join(lines))
    print(json.dumps({"status": data["status"], "coverage": data["coverage"], "measured_counts": counts}, sort_keys=True))


if __name__ == "__main__":
    main()
