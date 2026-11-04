#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Exit-status tests for rig_host (macOS only, after `make`).

Renders a short synthetic signal offline through the installed spp3 unit (no audio
device is opened) into a temporary directory that is deleted afterwards, and checks
that argument bounds, refused or unreadable CF properties, render errors and short
renders give a nonzero exit, that capture.check_host rejects the failed meta, and
that prefshim redirects whole path components only.

  test_rig_host.py [RIG_HOST]     (default: ./rig_host next to this file)
"""
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import capture  # noqa: E402


def main(argv):
    host = argv[0] if argv else os.path.join(HERE, "rig_host")
    if sys.platform != "darwin" or not os.path.exists(host):
        print("skip: needs macOS and a built rig_host")
        return 0
    strip = capture.STRIP["spp3"]
    if not os.path.exists(strip):
        print(f"skip: {strip} is not on this Mac")
        return 0
    d = tempfile.mkdtemp()
    try:
        return run(host, strip, d)
    finally:
        shutil.rmtree(d)


def run(host, strip, d):
    nch, frames = 6, 2048
    sig = os.path.join(d, "in.f32")
    with open(sig, "wb") as f:
        f.write(struct.pack(f"<{nch * frames}f", *[0.01 * ((i // nch) % 7 - 3) for i in range(nch * frames)]))
    good = os.path.join(d, "good.json")
    with open(good, "w") as f:
        json.dump({"a": 1}, f)
    bad = os.path.join(d, "bad.json")
    with open(bad, "w") as f:
        f.write("not json")
    base = [host, "--unit", "spp3", "--strip", strip, "--in", sig, "--ch", str(nch), "--q", "512"]

    def rc(*extra, out="run"):
        p = subprocess.run(base + ["--out", os.path.join(d, out)] + list(extra), capture_output=True, text=True)
        return p.returncode, p.stderr

    def meta(out="run"):
        return json.load(open(os.path.join(d, out + ".meta.json")))

    failures = []

    def expect(name, got, want):
        if got != want:
            failures.append(f"{name}: exit {got}, want {want}")

    r, err = rc()
    expect("plain render", r, 0)
    if r == 0:
        m = meta()
        capture.check_host("plain", m, frames)
        if m.get("frames_read") != frames:
            failures.append(f"frames_read {m.get('frames_read')}")
    # a render error exits nonzero after writing the meta
    r, _ = rc("--test-fail-at", "1024", out="fail")
    expect("render error", r, 3)
    try:
        m = meta("fail")
    except (OSError, ValueError):
        m = None
        failures.append("render error: no meta written")
    if m is not None:
        if m.get("render_error") == 0 or m.get("frames") != 1024 or m.get("frames_read") != 1536:
            failures.append(f"render error meta {m}")
        try:
            capture.check_host("fail", m, frames)
            failures.append("check_host accepted a failed render")
        except RuntimeError:
            pass
    # CF properties: refused or unparsable is fatal
    expect("refused cfprop", rc("--cfprop", f"4000000000={good}")[0], 1)
    expect("cfprop not JSON", rc("--cfprop", f"64009={bad}")[0], 1)
    expect("cfprop missing file", rc("--cfprop", f"64009={d}/absent.json")[0], 1)
    # argument bounds
    expect("--tele-ch 17", rc("--tele-ch", "17")[0], 2)
    expect("--tele-ch 0", rc("--tele-ch", "0")[0], 2)
    expect("cfprop without =", rc("--cfprop", "64009")[0], 2)
    expect("17 cfprops", rc(*sum((["--cfprop", f"64009={good}"] for _ in range(17)), []))[0], 2)
    expect("33 props", rc(*sum((["--prop", "1=1"] for _ in range(33)), []))[0], 2)
    expect("bad prop", rc("--prop", "x")[0], 2)
    expect("9 plistprops", rc(*sum((["--plistprop", f"1={good}"] for _ in range(9)), []))[0], 2)
    # prefshim redirects whole path components only
    shim = os.path.join(os.path.dirname(host), "prefshim.dylib")
    if os.path.exists(shim):
        for sub in ("from", "fromsibling", "to"):
            os.makedirs(os.path.join(d, sub))
        env = dict(os.environ, DYLD_INSERT_LIBRARIES=shim, RIGREDIR_FROM=os.path.join(d, "from"),
                   RIGREDIR_TO=os.path.join(d, "to"))
        for out, landed in (("from/x", "to/x"), ("fromsibling/x", "fromsibling/x")):
            p = subprocess.run(base + ["--out", os.path.join(d, out)], env=env, capture_output=True, text=True)
            if p.returncode or not os.path.exists(os.path.join(d, landed + ".meta.json")):
                failures.append(f"prefshim: {out} did not land at {landed}: {p.stderr[-300:]}")
    else:
        failures.append("prefshim.dylib is not built")
    for f in failures:
        print("FAIL", f)
    print("rig_host tests:", "FAILED" if failures else "OK")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
