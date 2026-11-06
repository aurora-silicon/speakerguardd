#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Command-level synthetic log replay; retain a reproducible result artifact."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

import numpy as np

HERE = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True, type=Path)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    cases = []
    output = np.arange(15, dtype=np.float32).reshape(5, 3) - 20
    telemetry = np.arange(15, dtype=np.float32).reshape(5, 3) + 100

    def encoded(paired):
        words = [3.0]
        for begin, end in ((0, 2), (2, 5)):
            words.append(float(end - begin))
            for c in range(3):
                words.extend(output[begin:end, c])
                if paired:
                    words.extend(telemetry[begin:end, c])
        return np.array(words, dtype="<f4").tobytes()

    def run(name, payload, layout="output", strict=True, expect=True,
            duplicate=False, thermal=False, missing=False,
            log_name="LsmSignalLog", export_signals=True):
        directory = args.out / name
        directory.mkdir()
        log = directory / f"synthetic_{log_name}.bin"
        if not missing:
            log.write_bytes(payload)
        if duplicate:
            (directory / "other_LsmSignalLog.bin").write_bytes(payload)
        if thermal:
            (directory / "synthetic_ControllerThermalLog.bin").write_bytes(
                np.array([3, 2, *range(15)], dtype="<f4").tobytes())
        summary, signals = directory / "summary.json", directory / "signals.npz"
        command = [sys.executable, "-I", str(HERE / "lsm2logs.py"), str(directory),
                   "--signal-layout", layout, "--summary", str(summary)]
        if export_signals:
            command.extend(["--signals", str(signals)])
        if strict:
            command.append("--strict")
        (directory / "command.json").write_text(json.dumps(command, indent=2) + "\n")
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        except (subprocess.TimeoutExpired, OSError) as exc:
            for stream in ("stdout", "stderr"):
                value = getattr(exc, stream, None) or ""
                if isinstance(value, bytes):
                    value = value.decode(errors="replace")
                (directory / f"{stream}.txt").write_text(value)
            cases.append({"case": name, "exit": None, "expected_success": expect,
                          "input_sha256": hashlib.sha256(payload).hexdigest(),
                          "passed": False, "error": str(exc)})
            return
        (directory / "stdout.txt").write_text(result.stdout)
        (directory / "stderr.txt").write_text(result.stderr)
        observation = {"case": name, "exit": result.returncode, "expected_success": expect,
                       "input_sha256": hashlib.sha256(payload).hexdigest(), "passed": False}
        try:
            if not expect:
                assert result.returncode != 0, "malformed input accepted"
                assert not summary.exists() and not signals.exists(), "failed parse emitted artifacts"
            else:
                assert result.returncode == 0, result.stderr
                report = json.loads(summary.read_text())
                observation["summary"] = report
                meta = report["logs"]["LsmSignalLog"]
                assert meta["channels"] == 3 and meta["records"] == 2 and meta["frames"] == 5
                assert meta["sha256"] == hashlib.sha256(payload).hexdigest()
                assert report["signal_layout"] == layout and report["strict"] == strict
                with np.load(signals, allow_pickle=False) as data:
                    np.testing.assert_array_equal(data["output"], output)
                    np.testing.assert_array_equal(data["record_start"], [0, 2])
                    if layout == "output-telemetry":
                        np.testing.assert_array_equal(data["telemetry"], telemetry)
                    else:
                        assert "telemetry" not in data.files
                if thermal:
                    assert report["logs"]["ControllerThermalLog"]["frames"] == 2
            observation["passed"] = True
        except (AssertionError, KeyError, OSError, ValueError) as exc:
            observation["error"] = str(exc)
        cases.append(observation)

    run("paired", encoded(True), "output-telemetry", thermal=True)
    run("legacy", encoded(False))
    run("wrong-layout", encoded(True), expect=False)
    run("truncated-float", encoded(True)[:-1], "output-telemetry", expect=False)
    run("truncated-record", encoded(True)[:-4], "output-telemetry", expect=False)
    run("empty", b"", expect=False)
    run("header-only", struct.pack("<f", 3), expect=False)
    for name, count in (("zero-channels", 0), ("fractional-channels", 1.5), ("nan-channels", float("nan"))):
        run(name, struct.pack("<f", count) + encoded(False)[4:], expect=False)
    for name, count in (("zero-frames", 0), ("fractional-frames", 1.5), ("nan-frames", float("nan"))):
        run(name, encoded(False)[:4] + struct.pack("<f", count) + encoded(False)[8:], expect=False)
    run("ambiguous-files", encoded(False), duplicate=True, expect=False)
    run("missing-signal", b"", missing=True, expect=False)
    run("legacy-partial-tail", encoded(False) + struct.pack("<ff", 4, 0), strict=False)
    run("frame-count-overflow", struct.pack("<7f", 1, 2**62, 0, 2**62, 0, 2**62, 0),
        log_name="LsmFrameLog", export_signals=False, expect=False)
    run("record-count-overflow", struct.pack("<3f", 1, 2**63, 0),
        log_name="LsmFrameLog", export_signals=False, expect=False)
    # Nonfinite samples are evidence, not framing errors. Preserve them exactly.
    output[0, 0] = float("nan")
    telemetry[1, 1] = float("inf")
    run("nonfinite-payload", encoded(True), "output-telemetry")
    report = {"passed": all(c["passed"] for c in cases), "cases": cases,
              "parser_sha256": hashlib.sha256((HERE / "lsm2logs.py").read_bytes()).hexdigest(),
              "python": sys.version, "numpy": np.__version__, "native_inputs": False}
    (args.out / "RESULT.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": report["passed"], "cases": len(cases),
                      "failed": [c["case"] for c in cases if not c["passed"]]}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
