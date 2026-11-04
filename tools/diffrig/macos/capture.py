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
"""
import csv, json, math, os, re, shutil, subprocess, sys, time
import numpy as np

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
import lsm2logs  # noqa: E402


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
    # host slice size: lsm2 output depends on it (the graph uses 512); spp3 is invariant
    for s in ["pink_steps", "tokyo_30s"]:  # without the runtime properties (preset limits only)
        C.append(dict(name=f"lsm2-noruntime-{s}", unit="lsm2", signal=s, runtime=False))
    for q in (1, 7, 160, 4096):
        C.append(dict(name=f"lsm2-q{q}-tone_bursts", unit="lsm2", signal="tone_bursts", q=q, logs=q >= 64))
    for q in (256, 1024):
        C.append(dict(name=f"spp3-q{q}-tone_bursts", unit="spp3", signal="tone_bursts", q=q))
    return C


def run_case(c, outroot, force=False):
    d = os.path.abspath(os.path.join(outroot, c["name"]))
    if os.path.exists(os.path.join(d, "trace.jsonl")) and not force:
        return d
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(os.path.join(d, "logs"), mode=0o700)
    os.chmod(d, 0o700)
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
    runtime = os.path.join(RIG, "cal", f"runtime-{unit}.json")
    rt = json.load(open(runtime)) if os.path.exists(runtime) and c.get("runtime", True) else []
    for pr in rt + c.get("cfprops", []):
        prop, fixture = pr.split("=", 1)
        args += ["--cfprop", f"{prop}={os.path.join(RIG, 'cal', fixture)}"]
    for pr in paths:
        args += ["--plistprop", pr]
    if unit == "spp3":
        if c.get("plant"):
            args += ["--plant", os.path.join(RIG, "cal", c["plant"])]
            env.update(RIGREDIR_FROM="/Library/Preferences/Audio/CLSPLogs", RIGPREF_clsp_logging_level="11")
    if unit == "lsm2" and c.get("logs", True):
        # small host slices outrun the unit's log writer at any pace: audio and meters only
        env.update(RIGREDIR_FROM="/Library/Preferences/Audio/lsm2", RIGPREF_LSMv2EnableLogging="1")
    t0 = time.time()
    pace = 50.0 if q >= 64 else 0.0
    # Capture only spp3's own calibration decision messages.  Starting the
    # stream before rig_host avoids relying on the process after it exits.
    logp = logf = None
    if c.get("cfprops"):
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
            p = subprocess.run(a2, env=env, capture_output=True, text=True, cwd=d)
            open(os.path.join(d, "host.log"), "w").write(p.stdout + p.stderr)
            if p.returncode:
                raise RuntimeError(f"{c['name']}: rig_host rc={p.returncode}: {p.stderr[-400:]}")
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
    decisions = {"closed_loop_channels": [], "open_loop_channels": []}
    aulog = os.path.join(d, "au.log")
    if os.path.exists(aulog):
        text = open(aulog).read()
        decisions["closed_loop_channels"] = sorted({int(x) for x in re.findall(
            r"Ch (\d+), Using closed loop parameters", text)})
        decisions["open_loop_channels"] = sorted({int(x) for x in re.findall(
            r"Ch (\d+), Using open loop kernel", text)})
    meta.update(case=c, wall_s=round(time.time() - t0, 2), **decisions)
    json.dump(meta, open(os.path.join(d, "run.meta.json"), "w"), indent=1)
    write_trace(d, c, meta)
    return d


def fnum(x):
    x = float(x)
    return None if not math.isfinite(x) else x


def write_trace(d, c, meta):
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
                decided = meta.get("closed_loop_channels", []) or meta.get("open_loop_channels", [])
                rec["mode"]["closed_loop"] = (meta.get("closed_loop_channels") == list(range(nchan))
                                                if decided else None)
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


if __name__ == "__main__":
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
