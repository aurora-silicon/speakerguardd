#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Check exported continuous hot-start results, including both Neo speakers.

Usage: python3 tests/check-noevents.py EVIDENCE_DIRECTORY
The directory must contain logs/peaks.json from the gain-trace analysis.
The in-process regression is full_scale_after_hot_start_without_control_events.
"""

import json
from pathlib import Path
import sys


def check(directory):
    rows = [
        row for row in json.loads((directory / "logs/peaks.json").read_text())
        if row["tree"] == "interim-neo" and row["mode"] == "neo_no_events"
    ]
    assert len(rows) == 2 and {row["speaker"] for row in rows} == {"Left", "Right"}, rows
    for row in rows:
        print(f"Neo without events, {row['speaker']}: {row['peak']:.9f} C against {row['limit']:.1f} C")
    assert all(row["limit"] == 135.0 and row["peak"] <= row["limit"] + 1e-6 for row in rows), (
        "minimum-volume startup protection was skipped without control events"
    )


if __name__ == "__main__":
    check(Path(sys.argv[1]))
