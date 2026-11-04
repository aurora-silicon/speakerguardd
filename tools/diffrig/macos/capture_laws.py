#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Designed-experiment captures for the spp3 behavioural laws (speakerguardd v2 rig, macOS side).

Each family isolates one law of the unit and holds some cases out for confirmation. Every
case runs through rig_host exactly like capture_edge.py (same compact outputs, CLSP logs
decoded into clsp.npz), on the AID35 strip and propstrip, with:
  - signals made at run time from a small segment description (law-*), written to SIG and
    never committed (they are regenerable);
  - synthetic SpTS calibration records written to cal/spts-law-*.json (our own values);
  - sidecar patches (capture_edge.patch_sidecar), parameter sets (--param) and host events.

Families (case-name prefix spp3-law-):
  pdrv-*      drive power against level, frequency and pilot on/off
  pcoil-*     coil power: program x pilot cross term (pilot level, program phase and
              frequency at and off the pilot bin)
  tcl-*       resistance-to-temperature law: R_cal, T_cal, a_t_20C/a_t_35C, Rshunt/Rampout
  env-*       pilot envelope: on and off steps at different times, decay-time variants
  hold-*      feed-forward candidate hold: arm and release timing against tunings
  noff-*      ThermalFFSpeedupFactor 0 hard path
  restore-*   SafeTlim across reset, uninitialize/initialize, ClassInfo and new instances

  capture_laws.py OUT [case-substring ...]      (SGV2_RIG_DIR, SGV2_RIG_BIN as capture.py)
"""
import json, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import capture  # noqa: E402
import capture_edge as ce  # noqa: E402

SR = 48000
RIG = capture.RIG
NCH = 6


# ---- signals: a list of segments, each {"dur": s, ...}; per-channel values may be lists ----
def _per_ch(v, n=NCH):
    return list(v) if isinstance(v, (list, tuple)) else [v] * n


def render_segments(segs, nch=NCH):
    """Segment kinds: zero; dc (a); sine (f, a, ph radians at the segment start unless
    "abs": true, which references the phase to frame 0); sum of sines ("parts": [...])."""
    out = []
    t0 = 0
    for s in segs:
        n = int(round(s["dur"] * SR))
        x = np.zeros((n, nch), np.float64)
        parts = s.get("parts", [s])
        for p in parts:
            kind = p.get("kind", s.get("kind", "zero"))
            if kind == "zero":
                continue
            a = np.array(_per_ch(p.get("a", 0.0), nch))
            if kind == "dc":
                x += a[None, :]
            elif kind == "sine":
                f = np.array(_per_ch(p["f"], nch))
                ph = np.array(_per_ch(p.get("ph", 0.0), nch))
                k = np.arange(n)[:, None] + (t0 if p.get("abs") else 0)
                x += a[None, :] * np.sin(2 * np.pi * f[None, :] * k / SR + ph[None, :])
            else:
                raise ValueError(kind)
        out.append(x.astype(np.float32))
        t0 += n
    return np.concatenate(out)


def gen(segs):
    def g(path, nch):
        x = render_segments(segs, nch)
        np.ascontiguousarray(x).tofile(path)
        os.chmod(path, 0o600)
    return g


# ---- synthetic SpTS fixtures (our own values) ----
def spts(re, t, tag):
    re, t = _per_ch(re), _per_ch(t)
    recs = [{"thiele_small_bi": 0, "thiele_small_fs": 0, "thiele_small_kms": 0, "thiele_small_le": 0,
             "thiele_small_mms": 0, "thiele_small_qts": 0, "thiele_small_rms": 0,
             "thiele_small_data_source_id": f"fcc:spA{i}", "thiele_small_re": float(re[i]),
             "thiele_small_t": float(t[i])} for i in range(NCH)]
    name = f"spts-law-{tag}.json"
    path = os.path.join(RIG, "cal", name)
    if not os.path.exists(path) or json.load(open(path)) != recs:
        json.dump(recs, open(path, "w"), indent=1)
        os.chmod(path, 0o600)
    return f"64009={name}"


def plant(**kw):
    return ce.plant(**kw) if kw.get("schedule") is None else ce.plant(kw.pop("schedule"), **kw)


def case(name, segs, cl=True, spts_fx=None, **kw):
    c = dict(name=f"spp3-law-{name}", unit="spp3", aid=35, signal=f"law-{name}", gen=gen(segs))
    if cl:
        c.update(cfprops=[spts_fx or "64009=spts-synth-nominal.json"], clsp=True)
        c.setdefault("plant", ce.plant())
    elif kw.get("clsp"):
        c["clsp"] = True
    c.update(kw)
    return c


def tone(dur, a, f=1000.0, ph=0.0):
    return {"dur": dur, "kind": "sine", "f": f, "a": a, "ph": ph, "abs": True}


def quiet(dur):
    return {"dur": dur}


def stairs(levels, dur=1.0, f=1000.0):
    return [tone(dur, a, f) for a in levels]


def mix(seed=1, n=30):
    """Non-periodic program: two incommensurate tones whose levels change every 0.1 s."""
    rng = np.random.default_rng(seed)
    return [{"dur": 0.1, "parts": [tone(0, 0.05 + 0.1 * rng.random(), 1000 + 37 * rng.random()),
                                   tone(0, 0.03 * rng.random(), 317.3)]} for _ in range(n)]


PILOT_OFF = [f"4.{ch}.21=0" for ch in range(NCH)]  # part parameter 21: pilot tone enabled
FS48 = 48000


def cases():
    C = []
    # ---- 1/2: drive power and coil power ----
    lv = [0.02, 0.05, 0.1, 0.2, 0.5, 0.9]
    prog = mix() + stairs(lv) + [quiet(0.5)]
    C.append(case("pdrv-cl", prog))
    C.append(case("pdrv-ol", prog, cl=False, clsp=True))
    C.append(case("pdrv-cl-pilotoff", prog, params=PILOT_OFF))
    C.append(case("pdrv-ol-pilotoff", prog, cl=False, clsp=True, params=PILOT_OFF))
    fb = 15 * FS48 / 16384  # the pilot generator's frequency (Goertzel bin)
    bin_ph = [tone(1.5, 0.05, fb, ph) for ph in (0, np.pi / 2, np.pi, 1.5 * np.pi)]
    C.append(case("pcoil-bin-phase", bin_ph + [tone(1.5, 0.05, 43.0664), tone(1.5, 0.05, 1000)],
                  sidecar_patch={"CL": {"PilotAmplLo_dB": -20}}))
    for lo in (-16, -26):
        C.append(case(f"pcoil-pilot{lo}", [tone(3, 0.05, fb), tone(3, 0.08, 1000)],
                      sidecar_patch={"CL": {"PilotAmplLo_dB": lo}}))
    # ---- 3: resistance-to-temperature law ----
    temps = [30, 45, 60, 75, 90, 105]
    stair_plant = ce.plant([{"t": 2 * i, "fixed_temp": T} for i, T in enumerate(temps)])
    tcl = {
        "nominal": {},
        "rcal4": dict(spts_fx=spts(4.0, 25, "rcal4")),
        "tcal40": dict(spts_fx=spts([3.3] * 4 + [3.75] * 2, 40, "tcal40")),
        "tcal15": dict(spts_fx=spts([3.3] * 4 + [3.75] * 2, 15, "tcal15")),
        "rshunt03": dict(sidecar_patch={"OL_thermal": {"Rshunt": 0.3}}),
        "rampout02": dict(sidecar_patch={"OL_thermal": {"Rampout": 0.2}}),
        "alpha": dict(sidecar_patch={"CL": {"a_t_20C": 0.004, "a_t_35C": 0.0039}}),
        "rebref2": dict(sidecar_patch={"OL_thermal": {"Reb_ref": 2.0}}),
        "rdcscale": dict(sidecar_patch={"CL": {"RdcScale": 1.1}}),
        # held out: every input moved at once, per channel
        "heldout": dict(spts_fx=spts([4.5, 3.4, 4.5, 3.4, 4.2, 3.6], [35, 22, 35, 22, 47, 12], "heldout"),
                        sidecar_patch={"OL_thermal": {"Rshunt": 0.15, "Rampout": 0.05},
                                       "CL": {"a_t_20C": 0.0039, "a_t_35C": 0.0036, "RdcScale": 0.95},
                                       "Channel4.OL_thermal": {"Rshunt": 0.21},
                                       "Channel4.CL": {"a_t_20C": 0.0041, "a_t_35C": 0.0035}}),
    }
    for k, v in tcl.items():
        C.append(case(f"tcl-{k}", [tone(12, 0.1)], plant=stair_plant, dump_params=True, **v))
    # ---- 4: pilot envelope ----
    steps = [tone(1.0, 0.05), quiet(0.5), tone(1.0, 0.05), quiet(0.5)]
    env = {"a": (0.03, 0.03), "b": (0.01, 0.06), "c": (0.06, 0.01)}
    for k, (t1, t2) in env.items():
        C.append(case(f"env-{k}", steps, sidecar_patch={"CL": {"PilotAmplLo_dB": -10, "PilotDecayTime": t1,
                                                               "PilotDecayTimeStage2": t2}}))
    # held out: shipped levels with on/off steps inside a thermal update; hi/lo switching
    C.append(case("env-shipped", [tone(1.0, 0.05), quiet(0.5), tone(0.75, 0.05), quiet(0.25 + 128 / FS48),
                                  tone(1.0 - 128 / FS48, 0.05), quiet(0.5)]))
    hl = [tone(1.5, 0.2), tone(3, 0.05), quiet(0.5)]
    C.append(case("env-hilo1", hl, sidecar_patch={"CL": {"PilotUpperThres": 54, "PilotLowerThres": 53.6,
                                                          "PilotAmplHi_dB": -10.4, "PilotAmplLo_dB": -12}}))
    C.append(case("env-hilo2", hl, sidecar_patch={"CL": {"PilotUpperThres": 54, "PilotLowerThres": 53.6,
                                                          "PilotAmplHi_dB": -20, "PilotAmplLo_dB": -26,
                                                          "PilotDecayTime": 0.045, "PilotDecayTimeStage2": 0.02}}))
    # ---- 5: feed-forward candidate release hold ----
    hp = [tone(8, 0.7), tone(4, 0.35), quiet(4)]
    hold = {
        "base": {}, "rel10": dict(sidecar_patch={"global": {"Thermal_GainReleaseTime_s": 10}}),
        "att2": dict(sidecar_patch={"global": {"Thermal_GainAttackTime_s": 2}}),
        "la1": dict(sidecar_patch={"global": {"Thermal_LookaheadDelay_ms": 1}}),
        "tauvc3": dict(sidecar_patch={"OL_thermal": {"tau_Tvc": 3.0}}),
        "ff05": dict(sidecar_patch={"OL_thermal": {"ThermalFFSpeedupFactor": 0.5}}),
        "pwin2": dict(params=["0.0.14=2"]),
        "misc": dict(sidecar_patch={"global": {"Thermal_SafeTlimTimeMin": 5, "Disp_PeakDecayTime_s": 0.1,
                                               "Disp_PeakAttackTime_s": 0.001, "Disp_FBIntegrationTime_s": 0.001,
                                               "Thermal_LookaheadDelay_ms": 0},
                                    "OL_thermal": {"HardTempLimitHeadroom": 5, "tau_Tmg": 50.0, "T_sett_vc": 20.0},
                                    "CL": {"TlimErrDecayTime": 10, "PilotDecayTime": 0.06}}),
        "sr96": dict(sr=96000), "sr441": dict(sr=44100), "q128": dict(q=128),
    }
    for k, v in hold.items():
        C.append(case(f"hold-{k}", hp, cl=False, clsp=True, **v))
    C.append(case("hold-cl", hp))
    C.append(case("hold-cl-sense", hp, sidecar_patch={"CL": {"TempSenseSmoothingTau": 2.5,
                                                              "TempSenseWindowTime": 0.09}}))
    # held out: a short dip (hold runs its full length) and a louder return (hold cancelled)
    C.append(case("hold-dip", [tone(8, 0.7), tone(0.3, 0.2), tone(4, 0.7), quiet(0.5), tone(3, 0.7)],
                  cl=False, clsp=True))
    C.append(case("hold-louder", [tone(8, 0.4), quiet(0.2), tone(4, 0.99)], cl=False, clsp=True))
    # ---- 6a: no feed-forward node ----
    nf = [tone(20, 0.7), tone(10, 0.35), quiet(5)]
    noff = {"OL_thermal": {"ThermalFFSpeedupFactor": 0}}
    C.append(case("noff", nf, cl=False, clsp=True, sidecar_patch=noff))
    C.append(case("noff-cl", nf, sidecar_patch=noff))
    C.append(case("noff-h0", [quiet(1), tone(5, 0.7), tone(3, 0.35), quiet(2)], cl=False, clsp=True,
                  sidecar_patch={"OL_thermal": {"ThermalFFSpeedupFactor": 0, "HardTempLimitHeadroom": 0}}))
    C.append(case("noff-ch4", [quiet(1), tone(5, 0.7), tone(3, 0.35), quiet(2)], cl=False, clsp=True,
                  sidecar_patch={"Channel4.OL_thermal": {"ThermalFFSpeedupFactor": 0}}))
    # ---- 6b: SafeTlim and thermal state across host events (TimeMin shortened to 1.2 s) ----
    st = {"global": {"Thermal_SafeTlimTimeMin": 0.02}}
    ev = 6 * FS48
    rs = {"none": [], "reset": ["reset"], "classinfo": ["classinfo"], "uninit": ["uninit"],
          "uninit0": ["uninit=0"], "reinit": ["reinit"], "newinst": ["newinst"], "newinst0": ["newinst=0"],
          "newinst1": ["newinst=1"], "newinstci0": ["newinst_ci=0"], "newinstci": ["newinst_ci"]}
    # The unit's log writer drops its unflushed records when the instance is reset or rebuilt:
    # 2 s of host idle before the event lets it drain (idle alone is a no-op, FINDINGS-safetlim).
    for k, e in rs.items():
        C.append(case(f"restore-{k}", [tone(12, 0.7)], cl=False, clsp=True, sidecar_patch=st,
                      events=[f"{ev}:sleep=2"] + [f"{ev}:{x}" for x in e]))
    for k in ("reset", "newinst0"):
        C.append(case(f"restore-cl-{k}", [tone(12, 0.7)], sidecar_patch=st,
                      events=[f"{ev}:sleep=2", f"{ev}:{rs[k][0]}"]))
    # ---- input clamp: SPP3 clamps its input to +-1.5 before the drive power and the output ----
    C.append(case("clip", [tone(1, 1.4), tone(1, 2.0), tone(1, 4.0), {"dur": 0.5, "kind": "dc", "a": 1.7},
                           {"dur": 0.5, "kind": "dc", "a": -3.0}], cl=False, clsp=True))
    # ---- fuzz finding 15: constant drive with fast taus / fast release (does macOS limit-cycle?) ----
    f15 = {"b": ({"Thermal_GainAttackTime_s": 1.87, "Thermal_GainReleaseTime_s": 0.045},
                 {"tau_Tvc": 0.021, "tau_Tmg": 0.022, "TemperatureLimit": 75, "HardTempLimitHeadroom": 10.5}, 0.7, 1000.0),
           "b-tau01": ({"Thermal_GainAttackTime_s": 1.87, "Thermal_GainReleaseTime_s": 0.045},
                       {"tau_Tvc": 0.1, "tau_Tmg": 0.105, "TemperatureLimit": 75, "HardTempLimitHeadroom": 10.5}, 0.7, 1000.0),
           "a": ({"Thermal_GainAttackTime_s": 2.0, "Thermal_GainReleaseTime_s": 5.0},
                 {"tau_Tvc": 0.5, "tau_Tmg": 5.0, "TemperatureLimit": 90, "HardTempLimitHeadroom": 11.9}, 1.122032, 1012.14)}
    for k, (g, ot, a, f) in f15.items():
        C.append(case(f"f15-{k}", [tone(30, a, f)], cl=False, clsp=True, dump_params=True,
                      sidecar_patch={"global": g, "OL_thermal": ot}))
    return C


def main(argv):
    out = argv[0] if argv else os.path.join(RIG, "traces")
    sel = argv[1:] or None
    mode = os.environ.get("LAWS_MODE", "")  # "ol" or "cl": only open- or closed-loop cases
    for c in cases():
        if sel and not any(s in c["name"] for s in sel):
            continue
        if mode and (mode == "cl") != bool(c.get("cfprops")):
            continue
        try:
            d = ce.run_case(c, out, force=bool(sel))
        except RuntimeError as e:  # one lossy case must not stop the batch
            print(f"{c['name']}: FAILED {e}", flush=True)
            continue
        m = json.load(open(os.path.join(d, "run.meta.json")))
        print(f"{c['name']}: frames={m['frames']} wall={m['wall_s']}s pace={m.get('pace')} "
              f"err={m['render_error']} cl={m.get('closed_loop_channels')} rec={m.get('clsp_records')}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
