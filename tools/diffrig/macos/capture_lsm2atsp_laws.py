#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Designed-experiment captures for the lsm2 and atsp behavioural laws (speakerguardd v2
rig, macOS side). Black box only: public AU calls (presets, parameters, CF properties),
the rendered audio, the public meters and lsm2's own debug logs (LSMv2EnableLogging).

A case is a dict:
  name     lsm2-law-* or atsp-law-*
  unit     "lsm2" (AID44 strip, propstrip 6632 + 64004, run-time 64008) or "atsp"
  aid      atsp AID (default 3)
  gen      callable(n_ch) -> float32 array (frames, ch); written to SIG at run time
  params   ["ID=VALUE" ...] global-scope parameters, set before and after Initialize
  cfprops  extra ["ID=fixture.json" ...] (cal/); lsm2 also gets cal/runtime-lsm2.json
           unless runtime=False
  q, sr, events, logmask (lsm2: LSMv2EnableLogging value; default all logs)

Each case directory under OUT holds run.out.f32.gz, run.state.csv.gz, trace.jsonl.gz,
run.meta.json and, for lsm2, lsm2.npz with the decoded controller logs:
  frame[k,16], power[k,97], thermal[k,11] and (when logged) signal[k,2561].
Everything is Apple-derived output: the directory is mode 700 and never committed.

  capture_lsm2atsp_laws.py OUT CASES.py [case-substring ...]
CASES.py defines cases() -> list of case dicts (kept with the analysis, not here).
"""
import gzip, importlib.util, json, os, shutil, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import capture  # noqa: E402
import capture_edge as ce  # noqa: E402
import lsm2logs  # noqa: E402

RIG, SIG, BIN = capture.RIG, capture.SIG, capture.BIN
LOGS_ALL = "1"


def _npz_logs(d, keep_signal=True):
    lsm = lsm2logs.load_dir(os.path.join(d, "logs"))
    out = {}
    for key, name in (("frame", "ControllerFrameLog"), ("power", "ControllerPowerLog"),
                      ("thermal", "ControllerThermalLog"), ("signal", "ControllerSignalLog"),
                      ("lsmframe", "LsmFrameLog")):
        if key == "signal" and not keep_signal:
            continue
        if name in lsm and lsm[name]["data"]:
            recs = lsm[name]["data"]
            w = recs[0].size
            a = np.stack([r for r in recs if r.size == w]).astype(np.float32)
            if key == "signal" and keep_signal is not True:
                a = a[:int(keep_signal)]  # keep_signal=N keeps the first N controller frames
            out[key] = a
            out[key + "_n"] = np.asarray(lsm[name]["n"], np.int32)
    np.savez_compressed(os.path.join(d, "lsm2.npz"), **out)
    return lsm["overwrites"]


def run_case(c, outroot, force=False):
    unit = c["unit"]
    aid = c.get("aid", 44 if unit == "lsm2" else 3)
    nch = 2 if unit == "lsm2" else ce.atsp_channels(aid)
    q = c.get("q", 512)
    d = os.path.abspath(os.path.join(outroot, c["name"]))
    sig = os.path.join(SIG, f"{c['name']}.{nch}ch.f32")
    x = np.ascontiguousarray(c["gen"](nch), dtype=np.float32)
    assert x.ndim == 2 and x.shape[1] == nch, x.shape
    x.tofile(sig)
    os.chmod(sig, 0o600)
    env = dict(os.environ, DYLD_INSERT_LIBRARIES=os.path.join(BIN, "prefshim.dylib"),
               RIGREDIR_TO=os.path.join(d, "logs"))
    args = [os.path.join(BIN, "rig_host"), "--unit", unit, "--strip", ce.strip_path(aid),
            "--in", sig, "--ch", str(nch), "--q", str(q), "--out", os.path.join(d, "run")]
    if c.get("sr"):
        args += ["--sr", str(c["sr"])]
    cc = dict(c, signal=c["name"])
    cc.pop("gen")
    runtime_file, cfprops = capture.resolve_cfprops(cc, use_runtime=(unit == "lsm2"))
    if unit == "lsm2":
        props, paths = capture.propstrip_props("lsm2")
        for pr in props:
            args += ["--prop", pr]
        for pr in paths:
            args += ["--plistprop", pr]
        if c.get("logs", True):
            env.update(RIGREDIR_FROM="/Library/Preferences/Audio/lsm2",
                       RIGPREF_LSMv2EnableLogging=str(c.get("logmask", LOGS_ALL)))
    for pr in cfprops:
        args += ["--cfprop", f"{pr['id']}={pr['path']}"]
    for p in c.get("params", []):
        args += ["--param", p if p.count(".") >= 2 and "=" in p and p.split("=")[0].count(".") == 2 else f"0.0.{p}"]
    for ev in c.get("events", []):
        args += ["--event", ev]
    key = capture.inputs_key(args + [json.dumps(cc, sort_keys=True, default=str)], sig, runtime_file, cfprops)
    if not force and capture.cached(d, key):
        return d
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(os.path.join(d, "logs"), mode=0o700)
    os.chmod(d, 0o700)
    pace = float(c.get("pace", 40.0)) if unit == "lsm2" and c.get("logs", True) and q >= 64 else 0.0
    t0 = time.time()
    while True:
        rc, out, err, pid = capture.run_host(args + (["--pace", str(pace)] if pace else []), env, d)
        open(os.path.join(d, "host.log"), "w").write(out + err)
        if rc:
            raise RuntimeError(f"{c['name']}: rig_host rc={rc}: {err[-400:]}")
        logs = os.listdir(os.path.join(d, "logs"))
        if not any("OVERWRITE" in f for f in logs):
            break
        if not pace or pace <= 2:
            raise RuntimeError(f"{c['name']}: lsm2 log overwrite at pace {pace}")
        pace /= 2.5
        for f in logs:
            os.unlink(os.path.join(d, "logs", f))
    meta = json.load(open(os.path.join(d, "run.meta.json")))
    capture.check_host(c["name"], meta, len(x))
    meta.update(case=cc, aid=aid, host_pid=pid, pace=pace, wall_s=round(time.time() - t0, 2),
                runtime_file=runtime_file, cfprops=cfprops)
    json.dump(meta, open(os.path.join(d, "run.meta.json"), "w"), indent=1, default=str)
    # schema trace (the shared writers), then compact storage
    old_sig = capture.SIG
    if unit == "lsm2":
        capture.write_trace(d, cc, meta)
        _npz_logs(d, c.get("keep_signal", True))
    else:
        ce.write_trace(d, dict(cc, aid=aid), meta)
    capture.SIG = old_sig
    shutil.rmtree(os.path.join(d, "logs"))
    for f in ("run.out.f32", "run.state.csv", "trace.jsonl", "run.tele.f32"):
        ce.gz(os.path.join(d, f))
    # the key is written last: an interrupted case is never taken as cached
    meta["inputs_sha256"] = key
    json.dump(meta, open(os.path.join(d, "run.meta.json"), "w"), indent=1, default=str)
    for root, dirs, files in os.walk(d):
        for f in files:
            os.chmod(os.path.join(root, f), 0o600)
    return d


def load(d, nch=None):
    """(input, output, meta, logs) of a captured case; logs is the lsm2.npz dict or {}."""
    meta = json.load(open(os.path.join(d, "run.meta.json")))
    nch = nch or meta["channels"]
    y = ce.f32(os.path.join(d, "run.out.f32"), nch)
    x = np.fromfile(os.path.join(SIG, f"{meta['case']['name']}.{nch}ch.f32"), np.float32).reshape(-1, nch)
    z = os.path.join(d, "lsm2.npz")
    logs = dict(np.load(z)) if os.path.exists(z) else {}
    return x, y, meta, logs


if __name__ == "__main__":
    out, casefile = sys.argv[1], sys.argv[2]
    spec = importlib.util.spec_from_file_location("lawcases", casefile)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    sel = sys.argv[3:] or None
    for c in mod.cases():
        if sel and not any(s in c["name"] for s in sel):
            continue
        dd = run_case(c, out, force=bool(sel))
        m = json.load(open(os.path.join(dd, "run.meta.json")))
        print(f"{c['name']}: frames={m['frames']} wall={m['wall_s']}s pace={m.get('pace')} "
              f"err={m['render_error']}", flush=True)
