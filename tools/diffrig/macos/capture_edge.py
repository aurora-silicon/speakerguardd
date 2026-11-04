#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Edge-case captures for the speakerguardd v2 differential rig (macOS side, offline).

Three families, each run through rig_host exactly like capture.py:

  atsp-aid<N>-<signal>, atsp-aid<N>-q<Q>-tone_bursts
      audio-only atsp on that AID's own strip preset; no meters exist, so the trace
      carries the measured audio gain and the per-sample gain extremes of each slice.
  spp3-cl-edge-*
      closed loop on the synthetic SpTS fixture and the rig's speaker plant, with a
      time schedule of sense faults (dead I or V, implausible V/I, all-zero telemetry,
      fixed sensed coil temperature, V/I power steps) and pilot-gate level stairs.
      The unit's closed-loop controller logs are decoded into clsp.npz.
  spp3-rampout-aid<N>-step_1k
      open loop on each AID with a fractional amplifier output resistance (Rampout),
      under a 20 s step and a 20 s release; plus a zero-Rampout control.

Outputs are compact: run.out.f32, run.tele.f32, run.state.csv and trace.jsonl are
gzipped, and the large regenerable V/I logs are not written (logging level 10:
coil-temperature and controller records only).

  capture_edge.py OUT [case-substring ...]      (SGV2_RIG_DIR, SGV2_RIG_BIN as capture.py)
"""
import csv, glob, gzip, json, math, os, plistlib, shutil, struct, subprocess, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import capture  # noqa: E402

RIG, SIG, BIN, TUN = capture.RIG, capture.SIG, capture.BIN, capture.TUN
SR = 48000
# Debug-log bit mask of the spp3 closed-loop kernel: 2 coil temperature, 8 controller records.
CLSP_LEVEL = "10"


def strip_path(aid):
    return f"{TUN}/AID{aid}/DSP/Strips/builtin_speaker_out_general.austrip"


def sidecar_path(aid):
    return f"{TUN}/AID{aid}/DSP/Strips/aid{aid}-aufx-spp3-appl.plist"


def propstrip(unit, aid):
    """The propstrip box properties for the unit on this AID (capture.propstrip_props, any AID)."""
    old = capture.AID.get(unit)
    capture.AID[unit] = aid
    try:
        return capture.propstrip_props(unit)
    finally:
        if old is None:
            capture.AID.pop(unit)
        else:
            capture.AID[unit] = old


def atsp_preset(aid):
    """(parameter-ID -> value) of the atsp effect in the AID's strip preset: AUBase SaveState
    data, a 12-byte header whose last word is the record count, then big-endian (ID, Float32)."""
    s = plistlib.load(open(strip_path(aid), "rb"))
    sub = int.from_bytes(b"atsp", "big")
    for st in s["strips"]:
        for e in st["effects"]:
            if e.get("unit", {}).get("subtype") == sub:
                d = e["aupreset"]["data"]
                n = struct.unpack(">I", d[8:12])[0]
                return dict(struct.unpack(">If", d[12 + 8 * i:20 + 8 * i]) for i in range(n))
    raise KeyError(f"AID{aid}: no atsp in the speaker strip")


def atsp_channels(aid):
    p = atsp_preset(aid)
    mask = 0
    for t in range(3):
        mask |= int(p.get(1 + 13 * t + 12, 0))
    if mask in (0, 0xFFFF):
        raise ValueError(f"AID{aid}: channel count not determined by the slot masks ({mask:#x})")
    return mask.bit_length()


# ---- the rig speaker plant (our own values) and sense schedules ----
PLANT = json.load(open(os.path.join(RIG, "cal", "plant-aid35-like.json"))) if os.path.exists(
    os.path.join(RIG, "cal", "plant-aid35-like.json")) else None


def plant(schedule=None, **kw):
    p = {k: v for k, v in PLANT.items() if k != "comment"}
    p.update(kw)
    if schedule:
        p["schedule"] = schedule
    return p


def re_temp(re_ohm, ch=0):
    """Plant coil temperature at which the sensed resistance (V/I) equals re_ohm."""
    re20 = PLANT["re20"][ch]
    return 20 + (re_ohm / re20 - 1) / PLANT["alpha"]


def cases():
    C = []
    sigs = ["silence_10s", "impulse", "pink_steps", "sweep_log", "tone_bursts",
            "thermal_ramp_1k", "thermal_ramp_200", "tokyo_30s"]
    # atsp: J293 (AID3), J493 (AID18), J313 (AID4), and two different two-type profiles
    for aid in (3, 18, 4, 5, 19):
        for s in sigs:
            C.append(dict(name=f"atsp-aid{aid}-{s}", unit="atsp", aid=aid, signal=s))
        C.append(dict(name=f"atsp-aid{aid}-thermal_ramp_1k_probe", unit="atsp", aid=aid,
                      signal="thermal_ramp_1k_probe"))
        for q in (1, 7, 160, 4096):
            C.append(dict(name=f"atsp-aid{aid}-q{q}-tone_bursts", unit="atsp", aid=aid,
                          signal="tone_bursts", q=q))
    # spp3 closed-loop edges (AID35, synthetic SpTS, rig plant)
    cl = dict(unit="spp3", aid=35, cfprops=["64009=spts-synth-nominal.json"], clsp=True)
    ramp = "thermal_ramp_1k"  # 120 s at -3 dBFS (deep in limiting from ~1 s) then 60 s release
    C.append(dict(cl, name="spp3-cl-edge-deadI-midlimit", signal=ramp,
                  plant=plant([{"t": 40, "i_gain": 0}])))
    for gap in (1, 5, 9.5, 10.5, 30):
        C.append(dict(cl, name=f"spp3-cl-edge-deadI-recover-{gap:g}s", signal=ramp,
                      plant=plant([{"t": 40, "i_gain": 0}, {"t": 40 + gap, "i_gain": 1}])))
    C.append(dict(cl, name="spp3-cl-edge-deadV-recover-30s", signal=ramp,
                  plant=plant([{"t": 40, "v_gain": 0}, {"t": 70, "v_gain": 1}])))
    C.append(dict(cl, name="spp3-cl-edge-deadVI-recover-30s", signal=ramp,
                  plant=plant([{"t": 40, "v_gain": 0, "i_gain": 0}, {"t": 70, "v_gain": 1, "i_gain": 1}])))
    # implausible V/I: the sensed resistance doubled or halved by the current scale
    C.append(dict(cl, name="spp3-cl-edge-reb-high-recover", signal=ramp,
                  plant=plant([{"t": 40, "i_gain": 0.5}, {"t": 70, "i_gain": 1}])))
    C.append(dict(cl, name="spp3-cl-edge-reb-low-recover", signal=ramp,
                  plant=plant([{"t": 40, "i_gain": 2}, {"t": 70, "i_gain": 1}])))
    C.append(dict(cl, name="spp3-cl-edge-zero-tele", signal=ramp, plant=None))
    # Fixed sensed coil temperature (resistance) staircases under a moderate drive
    # (T_OL stays near 60-70 C): the closed-loop limit, gain and pilot level against T_CL.
    tone = "tone_1k_m20_90s"
    stairs = [60, 85, 95, 99, 101, 110, 130, 85, 60]
    C.append(dict(cl, name="spp3-cl-edge-hotsense-stairs", signal=tone,
                  plant=plant([{"t": 10 * i, "fixed_temp": T} for i, T in enumerate(stairs)])))
    # The resistance acceptance range at and around its ends (woofers: re20 3.3 ohm)
    rng = [5.4, 5.49, 5.5, 5.51, 5.6, 5.5, 3.2, 3.11, 3.1, 3.09, 3.0, 3.3]
    C.append(dict(cl, name="spp3-cl-edge-rebrange-stairs", signal=tone,
                  plant=plant([{"t": 7.5 * i, "fixed_temp": [re_temp(r, c) for c in range(6)]}
                               for i, r in enumerate(rng)])))
    # V/I power gates: V and I scaled together (resistance unchanged), 2 s per step
    ks = [round(10 ** (-0.1 * i), 6) for i in range(30)] + [1.0]
    C.append(dict(cl, name="spp3-cl-edge-vipower-stairs", signal=tone,
                  plant=plant([{"t": 2 * i + 4, "v_gain": k, "i_gain": k} for i, k in enumerate(ks)])))
    # the same, fine: 0.005 steps of the common V/I scale across the gate found above
    kf = [round(0.64 - 0.005 * i, 3) for i in range(31)]
    C.append(dict(cl, name="spp3-cl-edge-vipower-fine", signal=tone,
                  plant=plant([{"t": 1.5 * i + 4, "v_gain": k, "i_gain": k} for i, k in enumerate(kf)]
                              + [{"t": 1.5 * len(kf) + 4, "v_gain": 1, "i_gain": 1}])))
    # one sense line at a time (the other at unity): I scaled at a low coil resistance, V
    # scaled at a high one, so the apparent resistance stays inside the acceptance range
    ka = [round(0.70 - 0.005 * i, 3) for i in range(25)]
    C.append(dict(cl, name="spp3-cl-edge-ipower-fine", signal=tone,
                  plant=plant([{"t": 0, "fixed_temp": [re_temp(3.15, c) for c in range(6)]}] +
                              [{"t": 1.5 * i + 4, "i_gain": k} for i, k in enumerate(ka)])))
    C.append(dict(cl, name="spp3-cl-edge-vpower-fine", signal=tone,
                  plant=plant([{"t": 0, "fixed_temp": [re_temp(5.45, c) for c in range(6)]}] +
                              [{"t": 1.5 * i + 4, "v_gain": k} for i, k in enumerate(ka)])))
    # Pilot content gate near PilotEnableThres (0.03): program level stairs, cool plant
    C.append(dict(cl, name="spp3-cl-edge-pilot-stairs", signal="pilot_stairs", plant=plant()))
    # Pilot high/low thresholds (rising and falling coil temperature with program present)
    C.append(dict(cl, name="spp3-cl-edge-pilot-hyst", signal="pilot_hyst", plant=plant()))
    # Pilot content gate at equality: DC program stepped one Float32 ULP at a time across the
    # level whose modeled drive power equals PilotEnableThres (made from this Mac's tuning)
    C.append(dict(cl, name="spp3-cl-edge-pilot-dc-ulp", signal="rt-pilot_dc_ulp", plant=plant()))
    # Host-side events mid-limit: AudioUnitReset, wall-clock idle, and a sample-time jump
    C.append(dict(cl, name="spp3-cl-edge-reset-midlimit", signal=ramp, plant=plant(),
                  events=[f"{40 * SR}:reset"]))
    for sl in (5, 15):
        C.append(dict(cl, name=f"spp3-cl-edge-sleep-{sl}s-midlimit", signal=ramp, plant=plant(),
                      events=[f"{40 * SR}:sleep={sl}"]))
        C.append(dict(cl, name=f"spp3-cl-edge-tsjump-{sl}s-midlimit", signal=ramp, plant=plant(),
                      events=[f"{40 * SR}:tsjump={sl * SR}"]))
    C.append(dict(name="spp3-ol-edge-reset-midlimit", unit="spp3", aid=35, signal=ramp,
                  events=[f"{40 * SR}:reset"]))
    # spp3 fractional Rampout, open loop, each AID's own propstrip configuration
    for aid in (8, 14, 24, 28, 30, 34, 39, 40):
        C.append(dict(name=f"spp3-rampout-aid{aid}-step_1k", unit="spp3", aid=aid, signal="step_1k"))
    C.append(dict(name="spp3-rampout-aid24-zero-step_1k", unit="spp3", aid=24, signal="step_1k",
                  sidecar_patch={"Rampout": 0}))
    return C


# ---- I/O helpers ----
def f32(path, ch):
    if not os.path.exists(path) and os.path.exists(path + ".gz"):
        with gzip.open(path + ".gz", "rb") as f:
            return np.frombuffer(f.read(), np.float32).reshape(-1, ch)
    return np.fromfile(path, np.float32).reshape(-1, ch)


def gz(path):
    if os.path.exists(path):
        subprocess.run(["gzip", "-6", "-f", path], check=True)


def decode_clsp(logdir, nch):
    """spp3 closed-loop logs: T_Coil_chN (10 Float32 per 512-frame block) and the
    'Reb_Data' controller stream (14 Float32 per 256-frame thermal update). A rebuilt
    instance (rig_host newinst events) opens new log files: they are joined in name order."""
    out = {}
    for key, width in (("T_Coil", 10), ("Reb_Data", 14)):
        rows = []
        for c in range(nch):
            fs = sorted(glob.glob(os.path.join(logdir, f"*_{key}_ch{c}_*")),
                        key=lambda f: (os.path.basename(f)[:17], os.path.getmtime(f)))
            a = np.concatenate([np.fromfile(f, np.float32) for f in fs]) if fs else np.zeros(0, np.float32)
            rows.append(a[: a.size // width * width].reshape(-1, width))
        n = min(len(r) for r in rows)
        out[key] = (np.stack([r[:n] for r in rows]), [len(r) for r in rows])
    return out


def patch_sidecar(pl, patch):
    """Apply a sidecar patch. Either {key: value} for every channel's OL_thermal (the original
    form), or a dict with any of: "global": {key: value} (top level), and "<Section>" or
    "Channel<N>.<Section>" keys ({key: value}) for every channel or one channel; Section is
    OL_thermal, CL or OL_disp. A value of None deletes the key."""
    def put(d, kv):
        for k, v in kv.items():
            if v is None:
                d.pop(k, None)
            else:
                d[k] = v
    sections = ("OL_thermal", "CL", "OL_disp")
    if not any(k == "global" or k.split(".")[-1] in sections for k in patch):
        patch = {"OL_thermal": patch}
    for k, kv in patch.items():
        if k == "global":
            put(pl, kv)
            continue
        if "." in k:
            chn, sec = k.split(".", 1)
            put(pl["ChannelSpecificParams"][chn][sec], kv)
        else:
            for ch in pl["ChannelSpecificParams"].values():
                put(ch[k], kv)
    return pl


def runtime_signal(name, nch):
    """Signals that depend on this Mac's tuning, written to SIG at run time (never committed)."""
    path = os.path.join(SIG, f"{name}.{nch}ch.f32")
    if name == "rt-pilot_dc_ulp":
        pl = plistlib.load(open(sidecar_path(35), "rb"))["ChannelSpecificParams"]
        segs = []
        for ch in ("Channel0", "Channel4"):  # woofer, then tweeter family
            p = pl[ch]
            ag, reb = float(p["OL_disp"]["ag"]), float(p["OL_thermal"]["Reb_ref"])
            rs, ro = float(p["OL_thermal"]["Rshunt"]), float(p["OL_thermal"].get("Rampout", 0))
            thr = float(p["CL"]["PilotEnableThres"])
            x0 = np.float32(math.sqrt(thr * (reb + rs + ro) ** 2 / reb) / ag)
            xs = [x0]
            for _ in range(48):
                xs.insert(0, np.nextafter(xs[0], np.float32(0)))
                xs.append(np.nextafter(xs[-1], np.float32(1)))
            segs += [np.zeros(SR // 2, np.float32)] + [np.full(1024, v, np.float32) for v in xs]
        x = np.concatenate(segs + [np.zeros(SR // 2, np.float32)])
        np.ascontiguousarray(np.repeat(x[:, None], nch, axis=1)).tofile(path)
        os.chmod(path, 0o600)
    return path


def run_case(c, outroot, force=False):
    if os.geteuid() == 0:
        raise RuntimeError("refusing to run as root: a missed log redirect would write system directories")
    d = os.path.abspath(os.path.join(outroot, c["name"]))
    unit, aid, q = c["unit"], c["aid"], c.get("q", 512)
    nch = atsp_channels(aid) if unit == "atsp" else 6
    env = dict(os.environ, DYLD_INSERT_LIBRARIES=os.path.join(BIN, "prefshim.dylib"),
               RIGREDIR_TO=os.path.join(d, "logs"))
    if c["signal"].startswith("rt-"):
        runtime_signal(c["signal"], nch)
    elif c.get("gen"):
        c["gen"](os.path.join(SIG, f"{c['signal']}.{nch}ch.f32"), nch)
    # The edge list sets no run-time graph properties: only the case's own fixtures.
    runtime_file, cfprops = capture.resolve_cfprops(c, use_runtime=False)
    args = [os.path.join(BIN, "rig_host"), "--unit", unit, "--strip", strip_path(aid),
            "--in", os.path.join(SIG, f"{c['signal']}.{nch}ch.f32"), "--ch", str(nch),
            "--q", str(q), "--out", os.path.join(d, "run")]
    if c.get("sr"):
        args += ["--sr", str(c["sr"])]
    for ev in c.get("events", []):
        args += ["--event", ev]
    if c.get("dump_params"):
        args += ["--dump-params"]
    expect_frames = os.path.getsize(os.path.join(SIG, f"{c['signal']}.{nch}ch.f32")) // (4 * nch)
    if unit == "spp3":
        sc = sidecar_path(aid)
        if c.get("sidecar_patch"):
            patched_sidecar = patch_sidecar(plistlib.load(open(sc, "rb")), c["sidecar_patch"])
            sc = os.path.join(d, "sidecar.plist")
        args += ["--sidecar", sc]
        props, paths = propstrip(unit, aid)
        # "props": {ID: value or None}: replace or drop propstrip box properties
        for pid, val in c.get("props", {}).items():
            props = [p for p in props if p.split("=")[0] != str(pid)]
            if val is not None:
                props.append(f"{pid}={val}")
        for pr in props:
            args += ["--prop", pr]
        for pr in c.get("params", []):
            args += ["--param", pr]
        for pr in paths:
            args += ["--plistprop", pr]
        for pr in cfprops:
            args += ["--cfprop", f"{pr['id']}={pr['path']}"]
        if c.get("plant"):
            args += ["--plant", os.path.join(d, "plant.json")]
        if c.get("clsp"):
            env.update(RIGREDIR_FROM="/Library/Preferences/Audio/CLSPLogs", RIGPREF_clsp_logging_level=str(c.get("clsp_level", CLSP_LEVEL)))
    signal = os.path.join(SIG, f"{c['signal']}.{nch}ch.f32")
    key = capture.inputs_key(args + [json.dumps({k: v for k, v in c.items() if k != "gen"}, sort_keys=True)],
                             signal, runtime_file, cfprops)
    if not force and capture.cached(d, key):
        return d
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(os.path.join(d, "logs"), mode=0o700)
    os.chmod(d, 0o700)
    if unit == "spp3" and c.get("sidecar_patch"):
        plistlib.dump(patched_sidecar, open(sc, "wb"))
        os.chmod(sc, 0o600)
    if unit == "spp3" and c.get("plant"):
        json.dump(c["plant"], open(os.path.join(d, "plant.json"), "w"), indent=1)
    t0 = time.time()
    pace = c.get("pace", 25.0) if c.get("clsp") else 0.0
    logp = logf = None
    if any(p["id"] == capture.SPTS_PROP for p in cfprops):
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
            a2 = args + (["--pace", str(pace)] if pace else [])
            p = subprocess.run(a2, env=env, capture_output=True, text=True, cwd=d)
            open(os.path.join(d, "host.log"), "w").write(p.stdout + p.stderr)
            if p.returncode:
                raise RuntimeError(f"{c['name']}: rig_host rc={p.returncode}: {p.stderr[-400:]}")
            if not c.get("clsp"):
                break
            # every block's controller record must be present, or the timeline has holes
            cl = decode_clsp(os.path.join(d, "logs"), nch)
            # the thermal cadence is 256 frames at 44.1 and 48 kHz and 512 at 96 kHz (observed)
            per = 256 * max(1, round(c.get("sr", SR) / SR))
            want = (int(expect_frames // (2 * per)), int(expect_frames // per))
            got = (cl["T_Coil"][1], cl["Reb_Data"][1])
            if all(n >= want[0] for n in got[0]) and all(n >= want[1] for n in got[1]):
                break
            # An instance rebuild (uninit/reinit/newinst events) can drop or restart a log file:
            # accept 80% after the first pace halving (records lost there are never recovered by a
            # slower pace); the public meters and audio stay complete, and clsp_records shows the gaps.
            rebuild = any(e.split(":", 1)[1].split("=")[0] in ("uninit", "reinit", "newinst", "newinst_ci")
                          for e in c.get("events", []))
            if rebuild and pace < 25 and all(n >= 0.8 * want[1] for n in got[1]):
                break
            if pace <= 3:
                # under host load the unit's log writer drops records at any pace: retry a few
                # times at the lowest pace before giving up
                tries = c.setdefault("_low_pace_tries", 0) + 1
                c["_low_pace_tries"] = tries
                if tries > 4:
                    raise RuntimeError(f"{c['name']}: log records lost even at pace {pace}: {got} want {want}")
            else:
                pace /= 2
            for f in os.listdir(os.path.join(d, "logs")):
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
    capture.check_host(c["name"], meta, expect_frames)
    decisions = {"closed_loop_channels": [], "open_loop_channels": []}
    if logf:
        decisions = capture.parse_decisions(open(os.path.join(d, "au.log")).read())
    c.pop("_low_pace_tries", None)
    meta.update(case={k: v for k, v in c.items() if k != "gen"}, aid=aid, wall_s=round(time.time() - t0, 2), pace=pace, runtime_file=runtime_file,
                cfprops=cfprops, inputs_sha256=key, **decisions)
    if c.get("clsp"):
        cl = decode_clsp(os.path.join(d, "logs"), nch)
        np.savez_compressed(os.path.join(d, "clsp.npz"), t_coil=cl["T_Coil"][0], ctl=cl["Reb_Data"][0])
        meta["clsp_records"] = {"t_coil": cl["T_Coil"][1], "ctl": cl["Reb_Data"][1]}
        if not c.get("keep_logs"):
            shutil.rmtree(os.path.join(d, "logs"))
            os.makedirs(os.path.join(d, "logs"), mode=0o700)
    json.dump(meta, open(os.path.join(d, "run.meta.json"), "w"), indent=1)
    write_trace(d, c, meta)
    for f in ("run.out.f32", "run.tele.f32", "run.state.csv", "trace.jsonl"):
        gz(os.path.join(d, f))
    return d


def spp3_limits(aid, c):
    pl = plistlib.load(open(sidecar_path(aid), "rb"))
    chans = pl["ChannelSpecificParams"]
    return [float(chans[f"Channel{i}"]["OL_thermal"]["TemperatureLimit"]) for i in range(len(chans))]


def write_trace(d, c, meta):
    """Schema lines. spp3 reuses capture.write_trace and adds the decoded controller state;
    atsp has no meters: the audio gives the gain."""
    unit, nch, q = c["unit"], meta["channels"], meta["quantum"]
    if unit == "spp3":
        capture.write_trace(d, c, meta)
        npz = os.path.join(d, "clsp.npz")
        if not os.path.exists(npz):
            return
        z = np.load(npz)
        tc, ctl = z["t_coil"], z["ctl"]
        tlim = spp3_limits(c["aid"], c)
        lines = [json.loads(x) for x in open(os.path.join(d, "trace.jsonl"))]
        for r in lines:
            end = r["frame"] + r["frames"]
            k, kb = end // 256 - 1, end // 512 - 1
            for ch, s in enumerate(r["channels"]):
                if 0 <= k < ctl.shape[1]:
                    v = [float(x) for x in ctl[ch, k]]
                    s["tlim_adj_c"] = v[3] - tlim[ch]
                    s["raw"] = {"ctl": v}
                if 0 <= kb < tc.shape[1]:
                    s.setdefault("raw", {})["t_coil"] = [float(x) for x in tc[ch, kb]]
        with open(os.path.join(d, "trace.jsonl"), "w") as fo:
            for r in lines:
                fo.write(json.dumps(r, separators=(",", ":")) + "\n")
        return
    x = np.fromfile(os.path.join(SIG, f"{c['signal']}.{nch}ch.f32"), np.float32).reshape(-1, nch)
    y = f32(os.path.join(d, "run.out.f32"), nch)
    lat = int(round(meta["latency_s"] * meta["sr"]))
    x = np.concatenate([np.zeros((lat, nch), np.float32), x])[:len(y)]
    ends = [int(r["frame"]) for r in csv.DictReader(open(os.path.join(d, "run.state.csv")))]
    with open(os.path.join(d, "trace.jsonl"), "w") as fo:
        prev = 0
        for end in ends:
            a, b = x[prev:end].astype(np.float64), y[prev:end].astype(np.float64)
            chans = []
            for ch in range(nch):
                ea, eb = float(np.dot(a[:, ch], a[:, ch])), float(np.dot(b[:, ch], b[:, ch]))
                s = {"audio_gain": None if ea < (end - prev) * 1e-12 else math.sqrt(eb / ea)}
                m = np.abs(a[:, ch]) > 1e-6
                if m.any():
                    g = b[m, ch] / a[m, ch]
                    s["raw"] = {"sample_gain_min": float(g.min()), "sample_gain_max": float(g.max()),
                                "sample_gain_last": float(g[-1])}
                chans.append(s)
            rec = dict(schema=1, side="macos", unit=unit, case=c["name"], frame=prev, frames=end - prev,
                       latency=lat, mode={"failsafe": None}, channels=chans, **{"global": {}})
            fo.write(json.dumps(rec, separators=(",", ":")) + "\n")
            prev = end


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RIG, "traces")
    sel = sys.argv[2:] or None
    for c in cases():
        if sel and not any(s in c["name"] for s in sel):
            continue
        dd = run_case(c, out, force=bool(sel))
        m = json.load(open(os.path.join(dd, "run.meta.json")))
        print(f"{c['name']}: frames={m['frames']} wall={m['wall_s']}s pace={m.get('pace')} "
              f"err={m['render_error']} cl={m.get('closed_loop_channels')} rec={m.get('clsp_records')}", flush=True)
