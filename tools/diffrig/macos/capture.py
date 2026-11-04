#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""macOS side of the speakerguardd v2 differential rig: run the case list through the
installed protection units offline (rig_host) and write schema traces.

Each case writes, under <out>/<case>/ (mode 700, Apple-derived data):
  run.out.f32 run.state.csv run.meta.json [run.tele.f32]  host outputs
  logs/                                                     the unit's own debug logs
  trace.jsonl                                               tools/diffrig/schema.json lines
Nothing here touches the user's defaults (prefshim answers lookups in-process) or any
audio device.

run.meta.json also records the resolved CF properties (cfprops: id, fixture, sha256,
source runtime|case), the run-time file used (runtime_file) and inputs_sha256, the hash
of the host arguments, input signal and fixtures. A cached trace is reused only when
inputs_sha256 matches. lsm2 cases need cal/runtime-<unit>.json (RUNTIME_REQUIRED)
unless they set runtime=False. rig_host's render_error and frame count are checked
before a trace is written. mode.closed_loop is the per-channel list of the schema:
from the unit's log when the calibration object is set, all open loop otherwise.

  capture.py [OUT [case-substring ...]] | --retrace DIR ... | --self-test
"""
import csv, hashlib, json, math, os, re, shutil, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
# The rig's data directory: generated signals in signals/, plant files in cal/, traces
# out. Apple-derived output stays there, never in the repository.
RIG = os.environ.get("SGV2_RIG_DIR", os.path.dirname(HERE))
SIG = os.path.join(RIG, "signals")
BIN = os.environ.get("SGV2_RIG_BIN", HERE)
TUN = "/System/Library/Audio/Tunings"
STRIP = {"spp3": f"{TUN}/AID35/DSP/Strips/builtin_speaker_out_general.austrip",
         "lsm2": f"{TUN}/AID44/DSP/Strips/builtin_speaker_out_general.austrip"}
SIDECAR = {"spp3": f"{TUN}/AID35/DSP/Strips/aid35-aufx-spp3-appl.plist"}
CH = {"spp3": 6, "lsm2": 2}
sys.path.insert(0, HERE)
# Units whose default cases need the live graph's run-time properties
# (cal/runtime-<unit>.json). A case of such a unit fails when the file is missing,
# unless it opts out with runtime=False (the lsm2-noruntime-* cases).
RUNTIME_REQUIRED = {"lsm2"}
# The spp3 calibration object (SpTS). Without it spp3 ignores telemetry and runs
# its open-loop kernel on every channel.
SPTS_PROP = 64009


AID = {"spp3": 35, "lsm2": 44}
SUBTYPE = {"spp3": b"spp3", "lsm2": b"lsm2"}


def propstrip_props(unit):
    """The unit's macOS configuration: the property-strip box of the effect that holds
    the unit in the built-in speaker strip, read from this Mac's tuning at run time."""
    import plistlib
    base = f"{TUN}/AID{AID[unit]}/DSP/Strips/builtin_speaker_out_general"
    strip = plistlib.load(open(base + ".austrip", "rb"))
    sub = int.from_bytes(SUBTYPE[unit], "big")
    names = [e.get("displayname") for s in strip["strips"] for e in s["effects"]
             if e.get("unit", {}).get("subtype") == sub]
    try:
        boxes = plistlib.load(open(base + ".propstrip", "rb"))["Boxes"]
    except FileNotFoundError:
        return [], []
    props, paths = [], []
    for box in boxes:
        if box.get("Name") not in names:
            continue
        for p in box.get("Properties", []):
            if "UInt32" in p:
                props.append(f"{p['Number']}={p['UInt32']}")
            elif "Path" in p and p["Number"] != 64003:  # 64003 is passed as --sidecar
                paths.append(f"{p['Number']}={p['Path']}")
    return props, paths


def cases():
    C = []
    sigs = ["silence_10s", "impulse", "pink_steps", "sweep_log", "tone_bursts",
            "thermal_ramp_1k", "thermal_ramp_200", "tokyo_30s"]
    for s in sigs:
        C.append(dict(name=f"spp3-{s}", unit="spp3", signal=s))
        C.append(dict(name=f"lsm2-{s}", unit="lsm2", signal=s))
    # not the macOS configuration: 64007 left at its default (state starts near the limit)
    for s in ["impulse", "pink_steps", "thermal_ramp_1k"]:
        C.append(dict(name=f"spp3-hotstart-{s}", unit="spp3", signal=s,
                      props=[p for p in propstrip_props("spp3")[0] if not p.startswith("64007=")]))
    # V/I from the rig's own speaker plant (closed loop needs an SpTS calibration fixture)
    for s in ["pink_steps", "thermal_ramp_1k"]:
        C.append(dict(name=f"spp3-plant-{s}", unit="spp3", signal=s, plant="plant-aid35-like.json"))
    C.append(dict(name="spp3-plant-deadI-pink_steps", unit="spp3", signal="pink_steps", plant="plant-deadI.json"))
    # Synthetic closed-loop captures: the calibration and V/I telemetry are both
    # produced by the rig, while the discovered default props mirror the propstrip.
    for s in ["pink_steps", "thermal_ramp_1k"]:
        C.append(dict(name=f"spp3-cl-{s}", unit="spp3", signal=s,
                      plant="plant-aid35-like.json",
                      cfprops=["64009=spts-synth-nominal.json"]))
    # Short calibration acceptance matrix. These record spp3's per-channel
    # closed/open-loop decision rather than exercising the thermal trajectory.
    for suffix, fixture in [
            ("nominal", "spts-synth-nominal.json"),
            ("reb-out-of-range", "spts-synth-reb-out-of-range.json"),
            ("caltemp-out-of-range", "spts-synth-caltemp-out-of-range.json"),
            ("wrong-source-ids", "spts-synth-wrong-source-ids.json"),
            ("five-entries", "spts-synth-five-entries.json")]:
        C.append(dict(name=f"spp3-cal-{suffix}", unit="spp3", signal="impulse",
                      cfprops=[f"64009={fixture}"]))
    # failure cases
    C.append(dict(name="spp3-nosidecar-pink_steps", unit="spp3", signal="pink_steps", sidecar=False))
    # host slice size: both units' output depends on it; the macOS graphs use 512
    for s in ["pink_steps", "tokyo_30s"]:  # without the runtime properties (preset limits only)
        C.append(dict(name=f"lsm2-noruntime-{s}", unit="lsm2", signal=s, runtime=False))
    for q in (1, 7, 160, 4096):
        C.append(dict(name=f"lsm2-q{q}-tone_bursts", unit="lsm2", signal="tone_bursts", q=q, logs=q >= 64))
    for q in (256, 1024):
        C.append(dict(name=f"spp3-q{q}-tone_bursts", unit="spp3", signal="tone_bursts", q=q))
    return C


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fd:
        for b in iter(lambda: fd.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def resolve_cfprops(c, rig=None, use_runtime=True):
    """The case's CF properties, resolved: the run-time list from cal/runtime-<unit>.json
    (when the case uses it) followed by the case's own. Returns (runtime_file, props):
    runtime_file is {"path", "sha256"} or None; each prop is
    {"id", "fixture", "path", "sha256", "source"}. Raises when a required runtime
    file or any fixture is missing."""
    rig = rig or RIG
    unit = c["unit"]
    runtime = os.path.join(rig, "cal", f"runtime-{unit}.json")
    want = use_runtime and c.get("runtime", True)
    rt, rtf = [], None
    if want and os.path.exists(runtime):
        rt = json.load(open(runtime))
        if not (isinstance(rt, list) and all(isinstance(x, str) and "=" in x for x in rt)):
            raise ValueError(f"{runtime}: expected a list of \"ID=fixture.json\" strings")
        rtf = {"path": runtime, "sha256": sha256(runtime)}
    elif want and unit in RUNTIME_REQUIRED:
        raise FileNotFoundError(f"{c['name']}: {runtime} is required for {unit} cases "
                                f"(without it the case would silently match its noruntime variant)")
    out = []
    for source, lst in (("runtime", rt), ("case", c.get("cfprops", []))):
        for pr in lst:
            prop, fixture = pr.split("=", 1)
            path = os.path.join(rig, "cal", fixture)
            if not os.path.isfile(path):
                raise FileNotFoundError(f"{c['name']}: fixture {path} for property {prop} is missing")
            out.append({"id": int(prop), "fixture": fixture, "path": path, "sha256": sha256(path),
                        "source": source})
    return rtf, out


def inputs_key(args, signal, runtime_file, cfprops):
    """Hash of everything that determines a reference: the host arguments, the
    input signal and every fixture. A cached trace is reused only when it matches."""
    h = hashlib.sha256()
    h.update(json.dumps({"args": args, "signal": sha256(signal) if os.path.exists(signal) else None,
                         "runtime_file": runtime_file, "cfprops": cfprops}, sort_keys=True).encode())
    return h.hexdigest()


def cached(d, key):
    """True when d holds a finished trace made from exactly these inputs."""
    try:
        meta = json.load(open(os.path.join(d, "run.meta.json")))
    except (OSError, ValueError):
        return False
    return meta.get("inputs_sha256") == key and (os.path.exists(os.path.join(d, "trace.jsonl")) or
                                                 os.path.exists(os.path.join(d, "trace.jsonl.gz")))


def check_host(name, meta, expect_frames):
    """rig_host exits nonzero on these too; check the meta as well, so a truncated
    reference is never written."""
    if meta.get("render_error"):
        raise RuntimeError(f"{name}: render error {meta['render_error']} after {meta.get('frames')} frames")
    if meta.get("frames") != expect_frames:
        raise RuntimeError(f"{name}: rendered {meta.get('frames')} frames of {expect_frames}")


def run_host(args, env, cwd):
    """Run rig_host; return (returncode, stdout, stderr, pid). The pid selects this run's
    own unit log messages: `log stream` sees every rig_host on the Mac (other lanes and
    earlier paced retries of the same case)."""
    p = subprocess.Popen(args, env=env, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out, err = p.communicate()
    return p.returncode, out, err, p.pid


_DECISION = re.compile(r"(?:rig_host\[(\d+)[:\]].*?)?Ch (\d+), Using (closed loop parameters|open loop kernel)")


def parse_decisions(text, pid=None):
    """spp3's per-channel closed/open-loop decisions from its log messages, in log order.
    pid: keep only that process's messages (None keeps every message). Each channel's
    decisions are kept as a sequence (one per Initialize: a reset or a new instance makes
    a new one), so a channel that goes closed, open, closed in one run is recorded, not
    rejected."""
    seq = {}
    for line in text.splitlines():
        m = _DECISION.search(line)
        if not m or (pid is not None and (m.group(1) is None or int(m.group(1)) != pid)):
            continue
        seq.setdefault(int(m.group(2)), []).append(m.group(3).startswith("closed"))
    return {"closed_loop_channels": sorted(c for c, v in seq.items() if any(v)),
            "open_loop_channels": sorted(c for c, v in seq.items() if not all(v)),
            "closed_loop_sequence": {str(c): v for c, v in sorted(seq.items())}}


def closed_loop_mode(unit, meta, nchan):
    """mode.closed_loop for the trace: a per-channel list (schema), or None when the
    unit makes no such decision. spp3 without its calibration object runs open loop on
    every channel; with it, every channel's decision must be in the unit's log."""
    if unit != "spp3":
        return None
    if "cfprops" in meta:
        ids = [p["id"] for p in meta["cfprops"]]
    else:  # captures made before the resolved list was recorded
        ids = [int(p.split("=", 1)[0]) for p in meta.get("case", {}).get("cfprops", [])]
    if SPTS_PROP not in ids and not meta.get("spts"):
        return [False] * nchan
    seq = meta.get("closed_loop_sequence")
    if seq is not None:  # captures that record the ordered decisions
        missing = [ch for ch in range(nchan) if not seq.get(str(ch))]
        if missing:
            raise ValueError(f"closed-loop decision not recorded for channels {missing}")
        # A channel whose decision changed during the run has no single per-slice value
        # without the frame of each re-decision: the trace records null (not compared) and
        # the meta keeps the sequence.
        if any(len(set(seq[str(ch)])) > 1 for ch in range(nchan)):
            return None
        return [seq[str(ch)][0] for ch in range(nchan)]
    cl, ol = set(meta.get("closed_loop_channels", [])), set(meta.get("open_loop_channels", []))
    both, neither = cl & ol, set(range(nchan)) - cl - ol
    if both or neither:
        raise ValueError(f"closed-loop decision not recorded unambiguously: channels in both "
                         f"{sorted(both)}, in neither {sorted(neither)}")
    return [ch in cl for ch in range(nchan)]


def run_case(c, outroot, force=False):
    if os.geteuid() == 0:
        raise RuntimeError("refusing to run as root: a missed log redirect would write system directories")
    d = os.path.abspath(os.path.join(outroot, c["name"]))
    unit, q = c["unit"], c.get("q", 512)
    env = dict(os.environ, DYLD_INSERT_LIBRARIES=os.path.join(BIN, "prefshim.dylib"),
               RIGREDIR_TO=os.path.join(d, "logs"))
    args = [os.path.join(BIN, "rig_host"), "--unit", unit, "--strip", STRIP[unit],
            "--in", os.path.join(SIG, f"{c['signal']}.{CH[unit]}ch.f32"), "--ch", str(CH[unit]),
            "--q", str(q), "--out", os.path.join(d, "run")]
    if unit == "spp3":
        if c.get("sidecar", True):
            args += ["--sidecar", SIDECAR[unit]]
    props, paths = propstrip_props(unit)
    for pr in c.get("props", props):
        args += ["--prop", pr]
    # Properties the live graph sets at run time (not tuning), as observed on the Mac,
    # from the rig data dir: cal/runtime-<unit>.json = ["ID=fixture.json", ...].
    runtime_file, cfprops = resolve_cfprops(c)
    for pr in cfprops:
        args += ["--cfprop", f"{pr['id']}={pr['path']}"]
    for pr in paths:
        args += ["--plistprop", pr]
    if unit == "spp3":
        if c.get("plant"):
            args += ["--plant", os.path.join(RIG, "cal", c["plant"])]
            env.update(RIGREDIR_FROM="/Library/Preferences/Audio/CLSPLogs", RIGPREF_clsp_logging_level="11")
    if unit == "lsm2" and c.get("logs", True):
        # small host slices outrun the unit's log writer at any pace: audio and meters only
        env.update(RIGREDIR_FROM="/Library/Preferences/Audio/lsm2", RIGPREF_LSMv2EnableLogging="1")
    signal = os.path.join(SIG, f"{c['signal']}.{CH[unit]}ch.f32")
    key = inputs_key(args, signal, runtime_file, cfprops)
    if not force and cached(d, key):
        return d
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(os.path.join(d, "logs"), mode=0o700)
    os.chmod(d, 0o700)
    expect_frames = os.path.getsize(signal) // (4 * CH[unit])
    t0 = time.time()
    pace = 50.0 if q >= 64 else 0.0
    # Capture only spp3's own calibration decision messages.  Starting the
    # stream before rig_host avoids relying on the process after it exits.
    logp = logf = None
    if any(p["id"] == SPTS_PROP for p in cfprops):
        predicate = ('process == "rig_host" AND '
                     '(eventMessage CONTAINS "Using closed loop parameters" OR '
                     'eventMessage CONTAINS "Using open loop kernel" OR '
                     'eventMessage CONTAINS "Failed to parse speaker calibration data" OR '
                     'eventMessage CONTAINS "Closed loop checks")')
        logf = open(os.path.join(d, "au.log"), "w")
        logp = subprocess.Popen(["/usr/bin/log", "stream", "--style", "compact", "--level", "debug",
                                 "--predicate", predicate], stdout=logf, stderr=subprocess.STDOUT)
        time.sleep(0.75)
    try:
        while True:
            a2 = args + (["--pace", str(pace)] if unit == "lsm2" and c.get("logs", True) else [])
            rc, out, err, host_pid = run_host(a2, env, d)
            open(os.path.join(d, "host.log"), "w").write(out + err)
            if rc:
                raise RuntimeError(f"{c['name']}: rig_host rc={rc}: {err[-400:]}")
            logs = os.listdir(os.path.join(d, "logs"))
            if not any("OVERWRITE" in f for f in logs):
                break
            # the unit's async log writer dropped records: slow down and redo
            if pace == 0.0:
                pace = 8.0
            elif pace <= 2:
                raise RuntimeError(f"{c['name']}: log overwrite even at pace {pace}")
            else:
                pace /= 2.5
            for f in logs:
                os.unlink(os.path.join(d, "logs", f))
    finally:
        if logp:
            time.sleep(0.25)
            logp.terminate()
            try:
                logp.wait(timeout=2)
            except subprocess.TimeoutExpired:
                logp.kill()
                logp.wait()
            logf.close()
    meta = json.load(open(os.path.join(d, "run.meta.json")))
    check_host(c["name"], meta, expect_frames)
    decisions = {"closed_loop_channels": [], "open_loop_channels": []}
    aulog = os.path.join(d, "au.log")
    if os.path.exists(aulog):
        decisions = parse_decisions(open(aulog).read(), host_pid)
    meta.update(case=c, host_pid=host_pid, wall_s=round(time.time() - t0, 2), runtime_file=runtime_file, cfprops=cfprops,
                inputs_sha256=key, **decisions)
    json.dump(meta, open(os.path.join(d, "run.meta.json"), "w"), indent=1)
    write_trace(d, c, meta)
    return d


def fnum(x):
    x = float(x)
    return None if not math.isfinite(x) else x


def write_trace(d, c, meta):
    import numpy as np
    import lsm2logs

    unit, q = c["unit"], meta["quantum"]
    rows = list(csv.DictReader(open(os.path.join(d, "run.state.csv"))))
    lat = int(round(meta["latency_s"] * meta["sr"]))
    lsm = lsm2logs.load_dir(os.path.join(d, "logs")) if unit == "lsm2" else None
    if lsm is not None and "ControllerThermalLog" in lsm:
        cf = lsm["ControllerFrameLog"]
        cframe_end = np.cumsum(cf["n"])
        th = np.stack(lsm["ControllerThermalLog"]["data"]).reshape(-1, lsm["ControllerThermalLog"]["nch"], 5)
        ctl = np.stack(cf["data"])
    # applied broadband gain measured from the audio itself (aligned by the reported latency)
    nchan = meta["channels"]
    x = np.fromfile(os.path.join(SIG, f"{c['signal']}.{nchan}ch.f32"), np.float32).reshape(-1, nchan)
    y = np.fromfile(os.path.join(d, "run.out.f32"), np.float32).reshape(-1, nchan)
    x = np.concatenate([np.zeros((lat, nchan), np.float32), x])[:len(y)]

    def audio_gain(a, b):
        ea, eb = float(np.dot(a, a)), float(np.dot(b, b))
        return None if ea < len(a) * 1e-12 else math.sqrt(eb / ea)
    closed_loop = closed_loop_mode(unit, meta, nchan)
    recs = []
    with open(os.path.join(d, "trace.jsonl"), "w") as fo:
        for r in rows:
            end = int(r["frame"])
            n = min(q, end) if end % q == 0 else end % q
            rec = dict(schema=1, side="macos", unit=unit, case=c["name"], frame=end - n, frames=n,
                       latency=lat, mode={"failsafe": None})
            chans = []
            if unit == "spp3":
                rec["global"] = {"modeled_power_w": fnum(r["s0.e0.p15:modeled_speaker_power"]),
                                 "measured_power_w": fnum(r["s0.e0.p16:measured_speaker_power"]),
                                 "thermal_gain_db": fnum(r["s0.e0.p12:thermal_gain"])}
                cl = []
                for e in range(6):
                    g = [fnum(r[f"s4.e{e}.p{p}:{nm}"]) for p, nm in
                         ((64, "thermal_gain"), (66, "power_control_gain"), (65, "displacement_gain"))]
                    gd = min(x for x in g if x is not None)
                    chans.append({"gain": 10 ** (gd / 20), "ol_temp_c": fnum(r[f"s4.e{e}.p8:OL_temperature"]),
                                  "cl_temp_c": fnum(r[f"s4.e{e}.p7:temperature"]),
                                  "axis_gain_db": {"thermal": g[0], "power": g[1], "displacement": g[2]},
                                  "displacement_rel_max": fnum(r[f"s4.e{e}.p40:Max_relative_displacement"])})
                rec["mode"]["closed_loop"] = closed_loop
            else:
                rec["global"] = {"modeled_amp_power_w": fnum(r["s0.e0.p1073741824:ModeledAmplifierPower"]),
                                 "amp_input_power_per_speaker_w": fnum(r["s0.e0.p1073741826:AmplifierInputPowerPerSpeaker"]),
                                 "health_bits": fnum(r["s0.e0.p1073741825:LoudspeakerHealthBitField"])}
                k = None
                if lsm is not None and "ControllerThermalLog" in lsm:
                    k = int(np.searchsorted(cframe_end, end, side="right")) - 1
                for ch in range(2):
                    s = {}
                    if k is not None and k >= 0:
                        # thermal record (SPEC-LSM2 §10): theta_vc, theta_mag, ambient, Tvc, Tcontrol
                        t = th[k, ch]
                        f = ctl[k]  # axis0[2] axis1[2] axis2[2] vel_limit[2] power_limit[4] global final[2]
                        s.update(coil_temp_c=float(t[3]), magnet_temp_c=float(t[2] + t[1]),
                                 control_temp_c=float(t[4]), gain=float(f[13 + ch]),
                                 velocity_limit_m_s=float(f[6 + ch]),
                                 raw={"axis0": float(f[ch]), "axis1": float(f[2 + ch]), "axis2": float(f[4 + ch])})
                    chans.append(s)
                if k is not None and k >= 0:
                    rec["global"]["power_hard_limit_w"] = [float(x) for x in ctl[k][8:12]]
                    rec["global"]["global_gain"] = float(ctl[k][12])
            ag = [audio_gain(x[end - n:end, ch], y[end - n:end, ch]) for ch in range(nchan)]
            for ch, s in enumerate(chans):
                s["audio_gain"] = ag[ch]
            rec["mode"]["failsafe"] = bool(all(g is not None and abs(g - 0.1) < 1e-4 for g in ag)) if any(
                g is not None for g in ag) else None
            rec["channels"] = chans
            recs.append(rec)
        # A silent slice shows nothing in the audio: it takes the fail-safe state of the
        # nearest audible slice (the fail-safe is a configuration state, not a level one).
        last = None
        for r in recs:  # forward fill, then fill the leading silent slices backwards
            if r["mode"]["failsafe"] is None:
                r["mode"]["failsafe"] = last
            last = r["mode"]["failsafe"]
        first = next((r["mode"]["failsafe"] for r in recs if r["mode"]["failsafe"] is not None), None)
        for r in recs:
            if r["mode"]["failsafe"] is None:
                r["mode"]["failsafe"] = first
            fo.write(json.dumps(r, separators=(",", ":")) + "\n")


def self_test():
    """Checks of the case plumbing that need no Mac, no numpy and no Apple data."""
    import tempfile

    rig = tempfile.mkdtemp()
    os.makedirs(os.path.join(rig, "cal"))

    def put(name, text):
        with open(os.path.join(rig, "cal", name), "w") as f:
            f.write(text)

    def raises(exc, f, *a, **k):
        try:
            f(*a, **k)
        except exc:
            return True
        raise AssertionError(f"{f.__name__}{a} did not raise {exc.__name__}")

    lsm = dict(name="lsm2-x", unit="lsm2", signal="s")
    # A required run-time file that is missing fails the case; opting out does not.
    raises(FileNotFoundError, resolve_cfprops, lsm, rig)
    assert resolve_cfprops(dict(lsm, runtime=False), rig) == (None, [])
    assert resolve_cfprops(dict(name="spp3-x", unit="spp3", signal="s"), rig) == (None, [])
    put("rt.json", '{"a": 1}')
    put("runtime-lsm2.json", '["1234=rt.json"]')
    rtf, props = resolve_cfprops(lsm, rig)
    assert rtf["sha256"] == sha256(os.path.join(rig, "cal", "runtime-lsm2.json"))
    assert [(p["id"], p["source"]) for p in props] == [(1234, "runtime")], props
    assert props[0]["sha256"] == sha256(os.path.join(rig, "cal", "rt.json"))
    assert resolve_cfprops(dict(lsm, runtime=False), rig) == (None, [])
    # case fixtures follow the run-time list; a missing fixture fails
    put("spts.json", "{}")
    _, props = resolve_cfprops(dict(lsm, cfprops=["64009=spts.json"]), rig)
    assert [p["id"] for p in props] == [1234, 64009]
    raises(FileNotFoundError, resolve_cfprops, dict(lsm, cfprops=["64009=absent.json"]), rig)
    put("runtime-lsm2.json", '{"1234": "rt.json"}')
    raises(ValueError, resolve_cfprops, lsm, rig)
    put("runtime-lsm2.json", '["1234=rt.json"]')
    # The cache key follows the fixture contents, not only the names.
    sig = os.path.join(rig, "sig.f32")
    with open(sig, "wb") as f:
        f.write(b"\0" * 16)
    k1 = inputs_key(["a"], sig, *resolve_cfprops(lsm, rig))
    put("rt.json", '{"a": 2}')
    k2 = inputs_key(["a"], sig, *resolve_cfprops(lsm, rig))
    assert k1 != k2
    d = os.path.join(rig, "case")
    os.makedirs(d)
    with open(os.path.join(d, "run.meta.json"), "w") as f:
        json.dump({"inputs_sha256": k1}, f)
    open(os.path.join(d, "trace.jsonl"), "w").close()
    assert cached(d, k1) and not cached(d, k2)
    # Host results: a render error or a short render is fatal.
    check_host("c", {"render_error": 0, "frames": 1000}, 1000)
    raises(RuntimeError, check_host, "c", {"render_error": -50, "frames": 512}, 1000)
    raises(RuntimeError, check_host, "c", {"render_error": 0, "frames": 512}, 1000)
    raises(RuntimeError, check_host, "c", {"render_error": 0}, 1000)
    # Closed-loop decisions: a per-channel list.
    text = "\n".join(["x Ch 0, Using closed loop parameters", "x Ch 1, Using open loop kernel"])
    dec = parse_decisions(text)
    assert dec["closed_loop_channels"] == [0] and dec["open_loop_channels"] == [1], dec
    spts = [{"id": SPTS_PROP}]
    # only the named process; a channel that goes closed, open, closed is a sequence
    text2 = "\n".join(["a rig_host[7:1a] [x] Ch 0, Using closed loop parameters r",
                        "a rig_host[9:1b] [x] Ch 0, Using open loop kernel",
                        "a rig_host[7:1a] [x] Ch 1, Using closed loop parameters r",
                        "a rig_host[7:1a] [x] Ch 0, Using open loop kernel",
                        "a rig_host[7:1a] [x] Ch 1, Using closed loop parameters r",
                        "a rig_host[7:1a] [x] Ch 0, Using closed loop parameters r"])
    d7 = parse_decisions(text2, 7)
    assert d7["closed_loop_sequence"] == {"0": [True, False, True], "1": [True, True]}, d7
    assert parse_decisions(text2, 9)["closed_loop_sequence"] == {"0": [False]}
    assert closed_loop_mode("spp3", dict(cfprops=spts, **d7), 2) is None
    assert closed_loop_mode("spp3", dict(cfprops=spts, closed_loop_sequence={"0": [True], "1": [False]}), 2) == [True, False]
    raises(ValueError, closed_loop_mode, "spp3", dict(cfprops=spts, closed_loop_sequence={"0": [True]}), 2)
    spts = [{"id": SPTS_PROP}]
    assert closed_loop_mode("spp3", dict(cfprops=spts, **dec), 2) == [True, False]
    raises(ValueError, closed_loop_mode, "spp3", dict(cfprops=spts, **dec), 3)  # channel 2 undecided
    raises(ValueError, closed_loop_mode, "spp3", dict(cfprops=spts, closed_loop_channels=[0, 1],
                                                       open_loop_channels=[1]), 2)
    assert closed_loop_mode("spp3", {"cfprops": []}, 3) == [False, False, False]
    assert closed_loop_mode("spp3", {"case": {"cfprops": ["64009=x"]}, **dec}, 2) == [True, False]
    assert closed_loop_mode("lsm2", {"cfprops": []}, 2) is None
    print("self-test OK")
    return 0


if __name__ == "__main__":
    if sys.argv[1:2] == ["--self-test"]:
        sys.exit(self_test())
    if sys.argv[1:2] == ["--retrace"]:
        for d in sys.argv[2:]:
            m = json.load(open(os.path.join(d, "run.meta.json")))
            write_trace(os.path.abspath(d), m["case"], m)
            print("retraced", d)
        sys.exit(0)
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RIG, "traces")
    sel = sys.argv[2:] or None
    for c in cases():
        if sel and not any(s in c["name"] for s in sel):
            continue
        d = run_case(c, out, force=bool(sel))
        m = json.load(open(os.path.join(d, "run.meta.json")))
        print(f"{c['name']}: frames={m['frames']} wall={m['wall_s']}s err={m['render_error']}", flush=True)
