#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Read-only evidence collection after each of three separately approved reboots.

This script never reboots, installs, changes a service, opens ALSA, or plays audio.
Run only on the controller-approved Neo, after each reboot performed by its agent.
Keep the output private: boot journals and package paths are local test evidence.
"""

import argparse
import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

UNIT = "speakerguardd.service"
PROPERTIES = (
    "ActiveState", "SubState", "Result", "NRestarts", "MainPID", "InvocationID",
    "ExecMainStatus", "ExecMainStartTimestampMonotonic", "FragmentPath", "DropInPaths",
)


def run(*command):
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=20, check=False)
    if result.returncode:
        raise RuntimeError(f"{command[0]} failed ({result.returncode}): {result.stderr.strip()}")
    return result.stdout


def snapshot():
    text = run("systemctl", "show", UNIT, "--no-pager",
               "--property=" + ",".join(PROPERTIES))
    return dict(line.split("=", 1) for line in text.splitlines() if "=" in line)


def read_state():
    text = Path("/run/speakerguardd/state").read_text()
    fields = dict(line.split(" ", 1) for line in text.splitlines()
                  if line.startswith(("armed ", "boottime_ns ")))
    return {"text": text, "age_seconds": time.clock_gettime(time.CLOCK_BOOTTIME)
            - int(fields["boottime_ns"]) / 1e9, "armed": fields.get("armed")}


def failures(record):
    errors = list(record.get("collection_errors", []))
    if record["kernel"] != record["expected_kernel"]:
        errors.append("unexpected kernel release")
    if "apple,j700" not in record["compatible"]:
        errors.append("not a J700")
    if "AppleJ700" not in record["cards"]:
        errors.append("AppleJ700 is absent")
    snapshots = record["snapshots"]
    if len(snapshots) != 2:
        errors.append("two service snapshots are required")
    for snap in snapshots:
        if (snap.get("ActiveState"), snap.get("SubState"), snap.get("Result")) != ("active", "running", "success"):
            errors.append("service is not healthy")
        if snap.get("NRestarts") != "0" or snap.get("ExecMainStatus") != "0":
            errors.append("service restarted or failed")
        if not snap.get("MainPID", "").isdigit() or snap.get("MainPID") == "0":
            errors.append("no running daemon PID")
        if not snap.get("InvocationID"):
            errors.append("missing invocation identity")
    if len(snapshots) == 2:
        for key in ("MainPID", "InvocationID", "ExecMainStartTimestampMonotonic"):
            if snapshots[0].get(key) != snapshots[1].get(key):
                errors.append("service changed during observation")
    states = record["states"]
    if len(states) != 2 or any(s["armed"] != "1" or not 0 <= s["age_seconds"] <= 5 for s in states):
        errors.append("missing, stale, or unarmed model state")
    journal = record["journal"]
    daemon = [j for j in journal if j.get("_COMM") == "speakerguardd"]
    messages = [str(j.get("MESSAGE", "")) for j in journal]
    invocations = {j["_SYSTEMD_INVOCATION_ID"] for j in daemon if j.get("_SYSTEMD_INVOCATION_ID")}
    current = snapshots[-1].get("InvocationID") if snapshots else None
    if not current or invocations != {current}:
        errors.append("journal does not show exactly the current first invocation")
    if sum("volume lock taken" in m for m in messages) != 1:
        errors.append("expected exactly one initial volume-lock acquisition")
    bad = ("no sound card", "no Apple sound card", "Failed to start", "Failed with result",
           "Scheduled restart job", "Start request repeated too quickly")
    if any(any(word in m for word in bad) or re.search(r"\]\s+E\s", m) for m in messages):
        errors.append("boot journal contains a startup failure, restart, or daemon error")
    return sorted(set(errors))


def collect(args):
    # Compare locally without storing either machine id in the evidence.
    expected = args.expected_machine_id_file.read_text().strip()
    actual = Path("/etc/machine-id").read_text().strip()
    if not re.fullmatch(r"[0-9a-f]{32}", expected) or not hmac.compare_digest(expected, actual):
        raise RuntimeError("controller-approved machine identity does not match")
    compatible = Path("/sys/firmware/devicetree/base/compatible").read_bytes().decode().strip("\0").split("\0")
    if "apple,j700" not in compatible:
        raise RuntimeError("collection is restricted to J700")
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    kernel = os.uname().release
    if kernel != args.expected_kernel:
        raise RuntimeError("kernel release does not match the controller-approved test kernel")
    if time.clock_gettime(time.CLOCK_BOOTTIME) < 60:
        raise RuntimeError("collect at least 60 seconds after boot")
    args.output.mkdir(mode=0o700, parents=True, exist_ok=True)
    if args.output.stat().st_mode & 0o077:
        raise RuntimeError("evidence directory must have mode 0700")
    destination = args.output / f"boot-{args.trial}.json"
    if destination.exists():
        raise RuntimeError("trial evidence already exists; preserve it and use a new run directory")
    for path in args.output.glob("boot-*.json"):
        if json.loads(path.read_text())["boot_id"] == boot_id:
            raise RuntimeError("this boot has already been collected")
    record = dict(trial=args.trial, boot_id=boot_id, kernel=kernel,
                  device_fingerprint=hashlib.sha256(b"speakerguardd-reboot-test\0" + actual.encode()).hexdigest(),
                  expected_kernel=args.expected_kernel, compatible=compatible,
                  cards=[p.read_text().strip() for p in Path("/sys/class/sound").glob("card*/id")],
                  snapshots=[], states=[], journal=[], collection_errors=[],
                  observe_seconds=args.observe_seconds)
    try:
        record["snapshots"].append(snapshot())
        record["states"].append(read_state())
        time.sleep(args.observe_seconds)
        record["snapshots"].append(snapshot())
        record["states"].append(read_state())
        # Service-only journal: no general host, user, network or account inventory.
        text = run("journalctl", "--boot=0", "--unit=" + UNIT, "--output=json", "--no-pager")
        allowed = ("MESSAGE", "__MONOTONIC_TIMESTAMP", "_COMM", "_SYSTEMD_INVOCATION_ID")
        record["journal"] = [{k: j[k] for k in allowed if k in j}
                             for j in map(json.loads, text.splitlines())]
        paths = ("/usr/bin/speakerguardd", "/usr/lib/systemd/system/speakerguardd.service",
                 "/usr/lib/udev/rules.d/95-speakerguardd.rules")
        effective = record["snapshots"][-1].get("FragmentPath")
        if effective and effective not in paths:
            paths += (effective,)
        if shutil.which("rpm"):
            command = ("rpm", "-qf", *paths)
        elif shutil.which("pacman"):
            command = ("pacman", "-Qo", *paths)
        else:
            raise RuntimeError("no supported package database")
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, timeout=20, check=False)
        record["package_owners"] = dict(command=command, status=result.returncode,
                                       stdout=result.stdout, stderr=result.stderr)
        if result.returncode:
            record["collection_errors"].append("a binary, rule or effective unit is not package-owned; review package_owners")
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        record["collection_errors"].append(str(error))
    record["failures"] = failures(record)
    # Exclusive creation preserves failures and prevents replacing earlier trials.
    fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as output:
        json.dump(record, output, indent=2)
        output.write("\n")
    print(f"trial {args.trial}: {'FAIL: ' + '; '.join(record['failures']) if record['failures'] else 'PASS'}")
    print(f"Private evidence: {destination}")
    return int(bool(record["failures"]))


def check(directory):
    records = [json.loads((directory / f"boot-{i}.json").read_text()) for i in range(1, 4)]
    errors = []
    if len({r["boot_id"] for r in records}) != 3:
        errors.append("three distinct boots are required")
    if len({r["kernel"] for r in records}) != 1:
        errors.append("all three boots must use the same kernel")
    if len({r.get("device_fingerprint") for r in records}) != 1 or not records[0].get("device_fingerprint"):
        errors.append("all three boots must use the same approved device")
    for i, record in enumerate(records, 1):
        if record["trial"] != i or record.get("observe_seconds", 0) < 10:
            errors.append(f"trial {i}: invalid trial or observation interval")
        errors.extend(f"trial {i}: {error}" for error in failures(record))
    print("FAIL: " + "; ".join(errors) if errors else "PASS: three distinct boots, first invocation healthy, no startup failure or restart")
    return int(bool(errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    item = sub.add_parser("collect")
    item.add_argument("--trial", type=int, choices=(1, 2, 3), required=True)
    item.add_argument("--expected-machine-id-file", type=Path, required=True)
    item.add_argument("--expected-kernel", required=True)
    item.add_argument("--output", type=Path, required=True)
    item.add_argument("--observe-seconds", type=int, default=30)
    item = sub.add_parser("check")
    item.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "collect" and not 10 <= args.observe_seconds <= 60:
        parser.error("observation must be between 10 and 60 seconds")
    try:
        return collect(args) if args.command == "collect" else check(args.output)
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.TimeoutExpired) as error:
        parser.exit(1, f"FAIL: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
