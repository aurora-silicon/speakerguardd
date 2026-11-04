#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compare a macOS reference trace with an sgv2 trace, slice by slice.

Both inputs are JSON Lines in tools/diffrig/schema.json. A reference line is
compared with the sgv2 line whose slice ends on the same input frame
(frame + frames), so the two sides may slice the program differently: the
state after the same input is compared. Every field the reference carries
must have a tolerance and be present on the sgv2 side, unless it is in the
explicit not-compared tables, which the report lists with their reasons. A
field present on one side and null on the other is a failure. The report
gives per-field compare and missing counts; a case with no reference slices
or no compared values fails.

Mode flags (fail-safe, closed loop per channel) must agree in every slice
where the reference observed them, so every mode transition lands in the
same slice. In a reference fail-safe slice only the audio-measured gain is
compared, because the macOS unit's meters do not show the fail-safe
attenuation; a fail-safe run must compare at least one non-null audio gain.

Optional audio comparison: --audio REF.f32 GOT.f32 --ch N compares rendered
float32 audio sample by sample after aligning each side by its reported
latency.

Usage:
  compare.py MACOS.jsonl SGV2.jsonl [--audio REF.f32 GOT.f32 --ch N]
             [--report OUT.json] [--plot DIR]
  compare.py --self-test

Exit status 0 only if everything is within tolerance.

Tolerances are the acceptance bar of the behaviour specification
(differential-test plan): 0.01 dB gain, 0.02 degC state temperature, 1e-5
linear audio below limiting. Power and mechanical states are relative.
lsm2 reports its axis gains as a linear list in the schema order (sample
peak, displacement, velocity, linked power, temperature); each element is
compared to 0.01 dB.
TODO(SPEC): tolerances for lsm2 mask and velocity state once its plan fixes them.

The audio comparison uses numpy when it is installed (the pure-Python loop
takes minutes on the 180 s cases).
"""
import json
import os
import math
import sys

TOLERANCE = {
    "gain": ("db", 0.01),
    "audio_gain": ("db", 0.01),
    "coil_temp_c": ("abs", 0.02),
    "magnet_temp_c": ("abs", 0.02),
    "control_temp_c": ("abs", 0.02),
    "ol_temp_c": ("abs", 0.02),
    "cl_temp_c": ("abs", 0.02),
    "tlim_adj_c": ("abs", 0.02),
    "modeled_power_w": ("rel", 1e-3),
    "measured_power_w": ("rel", 1e-3),
    "displacement_m": ("rel", 5e-3),
    "velocity_m_s": ("rel", 5e-3),
}
GLOBAL_TOLERANCE = {
    "modeled_power_w": ("rel", 1e-3),
    "measured_power_w": ("rel", 1e-3),
    "thermal_gain_db": ("abs", 0.01),
    "modeled_amp_power_w": ("rel", 1e-3),
    "amp_input_power_per_speaker_w": ("rel", 1e-3),
    "power_hard_limit_w": ("rel", 1e-3),  # per element
    "power_gain": ("db", 0.01),
}
# spp3 reports its per-axis gains in dB by name; sgv2 reports them linear in a
# fixed order (schema: sample peak, displacement, velocity, power, temperature).
AXIS_INDEX = {"peak": 0, "displacement": 1, "velocity": 2, "power": 3, "thermal": 4}
AXIS_TOLERANCE = ("abs", 0.01)  # dB
AXIS_NAMES = ("peak", "displacement", "velocity", "power", "temperature")
# Reference fields that are deliberately not compared, with the reason. Every
# other field the reference carries must have a tolerance above and must be
# present on the sgv2 side; a field in neither table fails the case.
NOT_COMPARED = {
    "raw": "unit-private state not named by the specification",
    "displacement_rel_max": "spp3 reports displacement relative to its own limit; sgv2 reports metres "
                            "(displacement_m) and the limit is not in the trace",
    "velocity_limit_m_s": "TODO(SPEC): the lsm2 plan fixes no tolerance for the velocity limit yet",
    "mask_db": "TODO(SPEC): the lsm2 plan fixes no tolerance for the mask yet",
    "band_gain": "TODO(SPEC): the lsm2 plan fixes no tolerance for band gains yet",
}
GLOBAL_NOT_COMPARED = {
    "health_bits": "the macOS health bit layout is not mapped to sgv2 Health bits by the specification",
    "global_gain": "TODO(SPEC): the scale (linear or dB) of the lsm2 controller global gain is not fixed",
}
AUDIO_TOLERANCE = 1e-5
# Relative errors use max(|ref|, floor) so values near zero do not give
# meaningless ratios.
REL_FLOOR = {"modeled_power_w": 1e-3, "measured_power_w": 1e-3, "displacement_m": 1e-6, "velocity_m_s": 1e-4,
             "modeled_amp_power_w": 1e-3, "amp_input_power_per_speaker_w": 1e-3, "power_hard_limit_w": 1e-3}


def _open(path, mode="r"):
    """Open path, or its gzipped form (path.gz) when only that exists."""
    import gzip

    if not os.path.exists(path) and os.path.exists(path + ".gz"):
        return gzip.open(path + ".gz", mode + "t" if mode == "r" else mode)
    return open(path, mode)


def load(path):
    out = {}
    with _open(path) as fd:
        for n, line in enumerate(fd, 1):
            if not line.strip():
                continue
            rec = json.loads(line)
            for k in ("schema", "side", "unit", "case", "frame", "frames", "channels"):
                if k not in rec:
                    raise ValueError(f"{path}:{n}: missing {k}")
            mode = rec.get("mode")
            if mode is not None:
                if not isinstance(mode, dict):
                    raise ValueError(f"{path}:{n}: mode is not an object")
                cl = mode.get("closed_loop")
                if cl is not None and not (isinstance(cl, list) and all(isinstance(x, bool) for x in cl)):
                    raise ValueError(f"{path}:{n}: mode.closed_loop must be a list of booleans or null, not {cl!r}")
                fs = mode.get("failsafe")
                if fs is not None and not isinstance(fs, bool):
                    raise ValueError(f"{path}:{n}: mode.failsafe must be a boolean or null, not {fs!r}")
            key = (rec["case"], rec["frame"] + rec["frames"])
            if key in out:
                raise ValueError(f"{path}:{n}: duplicate slice: case {key[0]} ending on frame {key[1]}")
            out[key] = rec
    return out


def error(name, kind_tol, ref, got):
    """Return (error, ok). None on both sides is equal; on one side it fails.
    A non-finite error (NaN on either side) fails."""
    if ref is None and got is None:
        return 0.0, True
    if ref is None or got is None:
        return math.inf, False
    kind, tol = kind_tol
    if kind == "db":
        if ref <= 0 or got <= 0:
            err = 0.0 if ref == got else math.inf
        else:
            err = abs(20 * math.log10(got / ref))
    elif kind == "rel":
        err = abs(got - ref) / max(abs(ref), REL_FLOOR.get(name, 1e-12))
    else:
        err = abs(got - ref)
    if not math.isfinite(err):
        return math.inf, False
    return err, err <= tol


class Worst:
    def __init__(self):
        self.fields = {}
        self.counts = {}
        self.fail = []
        self.failed = set()

    def count(self, name, what):
        c = self.counts.setdefault(name, {"compared": 0, "missing": 0})
        c[what] += 1

    def add(self, name, err, ok, where):
        self.count(name, "compared")
        w = self.fields.get(name)
        if w is None or err > w["error"]:
            self.fields[name] = {"error": err, "at": where}
        if not ok:
            self.failure(name, err, where)

    def failure(self, name, err, where, why=None):
        self.failed.add(name)
        if len(self.fail) < 50:
            f = {"field": name, "error": err, "at": where}
            if why:
                f["why"] = why
            self.fail.append(f)


def compare(ref, got):
    """Compare two indexed traces. Returns (ok, report).

    Every field the reference carries is compared and must be present on the
    sgv2 side, unless it is in NOT_COMPARED / GLOBAL_NOT_COMPARED, which the
    report lists with the reason and the number of slices it was skipped in.
    A case fails when it has no reference slices, compares no values, or (in
    a fail-safe run) compares no non-null audio gain."""
    worst = Worst()
    cases = {}
    not_compared = {}
    missing = 0

    def skip(name, reason):
        n = not_compared.setdefault(name, {"reason": reason, "slices": 0})
        n["slices"] += 1

    for key in sorted(ref):
        r = ref[key]
        case, end = key
        c = cases.setdefault(case, {"ref_slices": 0, "slices": 0, "values": 0, "failsafe_slices": 0,
                                    "failsafe_audio_values": 0, "pass": True})
        c["ref_slices"] += 1
        g = got.get(key)
        if g is None:
            missing += 1
            c["pass"] = False
            continue
        c["slices"] += 1
        where = {"case": case, "end_frame": end}
        ok_slice = True

        def field(name, tol, rv, gv, present, at):
            nonlocal ok_slice
            if not present:
                worst.count(name, "missing")
                worst.failure(name, math.inf, at, "absent on the sgv2 side")
                ok_slice = False
                return
            err, ok = error(name.split(".")[-1], tol, rv, gv)
            worst.add(name, err, ok, at)
            c["values"] += 1
            ok_slice &= ok

        rm, gm = r.get("mode") or {}, g.get("mode") or {}
        # Mode flags. The reference's flag is compared whenever it was observed;
        # an unobserved one is reported as not compared.
        rfs = rm.get("failsafe")
        if rfs is None:
            skip("mode.failsafe", "not observed in the reference (audio silent throughout)")
        else:
            gfs = gm.get("failsafe")
            ok = isinstance(gfs, bool) and gfs == rfs
            worst.add("mode.failsafe", 0.0 if ok else math.inf, ok, where)
            c["values"] += 1
            ok_slice &= ok
        rcl = rm.get("closed_loop")
        if rcl is None:
            skip("mode.closed_loop", "the reference did not record a closed-loop decision")
        else:
            gcl = gm.get("closed_loop")
            ok = isinstance(gcl, list) and len(gcl) >= len(rcl) and gcl[:len(rcl)] == rcl
            worst.add("mode.closed_loop", 0.0 if ok else math.inf, ok, where)
            c["values"] += 1
            ok_slice &= ok

        rg, gg = r.get("global") or {}, g.get("global")
        for name in sorted(rg):
            rv = rg[name]
            if name in GLOBAL_NOT_COMPARED:
                skip("global." + name, GLOBAL_NOT_COMPARED[name])
                continue
            tol = GLOBAL_TOLERANCE.get(name)
            if tol is None:
                worst.failure("global." + name, math.inf, where, "no tolerance for this reference field")
                ok_slice = False
                continue
            present = isinstance(gg, dict) and name in gg
            gv = gg.get(name) if present else None
            if isinstance(rv, list):
                if present and not (isinstance(gv, list) and len(gv) == len(rv)):
                    worst.add("global." + name, math.inf, False, where)
                    ok_slice = False
                    continue
                for i, x in enumerate(rv):
                    field("global." + name, tol, x, gv[i] if present else None, present, dict(where, index=i))
            else:
                field("global." + name, tol, rv, gv, present, where)

        if len(g["channels"]) < len(r["channels"]):
            worst.failure("channels", math.inf, where, "fewer channels on the sgv2 side")
            ok_slice = False
        # In the macOS fail-safe the published meters keep their last model values
        # (gain 0 dB); only the audio shows the attenuation. Compare the audio there.
        failsafe = bool(rfs)
        if failsafe:
            c["failsafe_slices"] += 1
        for ch, rc in enumerate(r["channels"]):
            gc = g["channels"][ch] if ch < len(g["channels"]) else {}
            at = dict(where, channel=ch)
            for name in sorted(rc):
                rv = rc[name]
                if failsafe and name != "audio_gain":
                    skip(name, "reference slice is in the fail-safe; only audio_gain is compared there")
                    continue
                if name in NOT_COMPARED:
                    skip(name, NOT_COMPARED[name])
                    continue
                if name == "axis_gain_db":
                    gax = gc.get("axis_gain")
                    for axis in sorted(rv or {}):
                        idx = AXIS_INDEX.get(axis)
                        nm = f"axis_gain_db.{axis}"
                        if idx is None:
                            worst.failure(nm, math.inf, at, "no sgv2 axis for this reference axis")
                            ok_slice = False
                            continue
                        present = isinstance(gax, list) and idx < len(gax)
                        lin = gax[idx] if present else None
                        gdb = None if lin is None else (20 * math.log10(lin) if lin > 0 else -math.inf)
                        field(nm, AXIS_TOLERANCE, rv[axis], gdb, present, at)
                    continue
                if name == "axis_gain" and isinstance(rv, list):
                    gax = gc.get("axis_gain")
                    present = isinstance(gax, list) and len(gax) >= len(rv)
                    for j, x in enumerate(rv):
                        field(f"axis_gain.{AXIS_NAMES[j]}", ("db", 0.01), x, gax[j] if present else None, present, at)
                    continue
                tol = TOLERANCE.get(name)
                if tol is None:
                    worst.failure(name, math.inf, at, "no tolerance for this reference field")
                    ok_slice = False
                    continue
                field(name, tol, rv, gc.get(name), name in gc, at)
                if failsafe and name == "audio_gain" and rv is not None and gc.get(name) is not None:
                    c["failsafe_audio_values"] += 1
        c["pass"] &= ok_slice
    for case in sorted({k[0] for k in got} - set(cases)):
        cases[case] = {"ref_slices": 0, "slices": 0, "values": 0, "pass": False,
                       "why": "no reference slices for this case"}
    for c in cases.values():
        if c["ref_slices"] and not c["values"]:
            c["pass"], c["why"] = False, "no values compared"
        elif c.get("failsafe_slices") and not c["failsafe_audio_values"]:
            c["pass"], c["why"] = False, "fail-safe run with no compared audio gain"
    ok = bool(cases) and missing == 0 and not worst.fail and all(c["pass"] for c in cases.values())
    return ok, {"pass": ok, "missing_slices": missing, "cases": cases, "worst": worst.fields,
                "fields": worst.counts, "not_compared": not_compared, "failures": worst.fail,
                "failed_fields": sorted(worst.failed)}


def read_f32(path):
    with _open(path, "rb") as fd:
        data = fd.read()
    try:
        import numpy as np

        return np.frombuffer(data, "<f4")
    except ImportError:
        import array

        a = array.array("f")
        a.frombytes(data)
        if sys.byteorder != "little":
            a.byteswap()
        return a


def compare_audio(ref_path, got_path, channels, ref_latency=0, got_latency=0):
    a, b = read_f32(ref_path), read_f32(got_path)
    if len(a) % channels or len(b) % channels:
        return False, {"pass": False, "error": "file length is not a whole number of frames"}
    na, nb = len(a) // channels, len(b) // channels
    n = min(na - ref_latency, nb - got_latency)
    # Any non-finite sample on either side fails, wherever it is (also outside
    # the aligned span), and NaN differences never compare as small.
    nonfinite = {}
    worst, at, rms = 0.0, None, math.nan
    if hasattr(a, "reshape"):
        import numpy as np

        for side, x in (("reference", a), ("sgv2", b)):
            bad = np.flatnonzero(~np.isfinite(x))
            if bad.size:
                nonfinite[side] = {"frame": int(bad[0]) // channels, "channel": int(bad[0]) % channels}
        if n > 0:
            d = np.abs(a.reshape(na, channels)[ref_latency:ref_latency + n].astype(np.float64)
                       - b.reshape(nb, channels)[got_latency:got_latency + n])
            d[~np.isfinite(d)] = np.inf
            i = int(np.argmax(d))
            worst, at = float(d.flat[i]), {"frame": i // channels, "channel": i % channels}
            rms = float(np.sqrt(np.mean(d * d)))
    else:
        for side, x in (("reference", a), ("sgv2", b)):
            bad = next((i for i, v in enumerate(x) if not math.isfinite(v)), None)
            if bad is not None:
                nonfinite[side] = {"frame": bad // channels, "channel": bad % channels}
        for i in range(max(n, 0)):
            for c in range(channels):
                d = abs(a[(i + ref_latency) * channels + c] - b[(i + got_latency) * channels + c])
                if not d <= worst:
                    worst, at = (d if math.isfinite(d) else math.inf), {"frame": i, "channel": c}
    ok = (n > 0 and not nonfinite and math.isfinite(worst) and worst <= AUDIO_TOLERANCE
          and na - ref_latency == nb - got_latency)
    rep = {"pass": ok, "frames": n, "max_abs_error": worst, "rms_error": rms, "at": at,
           "tolerance": AUDIO_TOLERANCE}
    if nonfinite:
        rep["non_finite"] = nonfinite
    return ok, rep


def plot(ref, got, outdir):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("plot: matplotlib is not available", file=sys.stderr)
        return
    import os

    os.makedirs(outdir, exist_ok=True)
    sr = 48000.0
    for case in sorted({k[0] for k in ref}):
        keys = sorted(k for k in ref if k[0] == case)
        fields = [f for f in TOLERANCE if any(f in c for c in ref[keys[0]]["channels"])]
        if not fields:
            continue
        fig, axes = plt.subplots(len(fields), 1, figsize=(10, 2.4 * len(fields)), sharex=True, squeeze=False)
        t = [k[1] / sr for k in keys]

        def val(rec, ch, f):
            v = rec["channels"][ch].get(f) if rec and ch < len(rec["channels"]) else None
            if v is None:
                return math.nan
            return 20 * math.log10(v) if "gain" in f and v > 0 else v

        for ax, f in zip(axes[:, 0], fields):
            for ch in range(len(ref[keys[0]]["channels"])):
                ax.plot(t, [val(ref[k], ch, f) for k in keys], lw=1.2, label=f"macOS ch{ch}")
                ax.plot(t, [val(got.get(k), ch, f) for k in keys], lw=0.8, ls="--", label=f"sgv2 ch{ch}")
            ax.set_ylabel(f + (" (dB)" if "gain" in f else ""))
        axes[0, 0].legend(fontsize=6, ncol=6)
        axes[-1, 0].set_xlabel("input time (s)")
        fig.suptitle(case)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, f"{case}.png"), dpi=110)
        plt.close(fig)


def self_test():
    def rec(side, frame, frames, gain, temp, failsafe=False, cl=None):
        return {"schema": 1, "side": side, "unit": "spp3", "case": "c", "frame": frame, "frames": frames,
                "mode": {"failsafe": failsafe, "closed_loop": cl},
                "global": {"modeled_power_w": 1.0},
                "channels": [{"gain": gain, "coil_temp_c": temp, "ol_temp_c": None}]}

    def idx(*rs):
        return {(r["case"], r["frame"] + r["frames"]): r for r in rs}

    ref = idx(rec("macos", 0, 512, 0.5, 60.0), rec("macos", 512, 512, 0.5, 60.1))
    # Same state, different slicing: only slices ending on common frames compare.
    got = idx(rec("sgv2", 0, 256, 0.9, 10.0), rec("sgv2", 256, 256, 0.5, 60.005),
              rec("sgv2", 512, 512, 0.5005, 60.11))
    assert compare(ref, got)[0], compare(ref, got)
    bad = dict(got)
    bad[("c", 1024)] = rec("sgv2", 512, 512, 0.5, 60.2)
    assert not compare(ref, bad)[0]
    bad[("c", 1024)] = rec("sgv2", 512, 512, 0.52, 60.1)  # 0.34 dB
    assert not compare(ref, bad)[0]
    bad[("c", 1024)] = rec("sgv2", 512, 512, 0.5, 60.1, failsafe=True)
    assert not compare(ref, bad)[0]
    assert not compare(idx(rec("macos", 0, 512, 0.5, 60.0, cl=[True])),
                       idx(rec("sgv2", 0, 512, 0.5, 60.0, cl=[False])))[0]
    assert not compare(ref, {})[0]
    fs_ref = idx(rec("macos", 0, 512, 1.0, 50.0, failsafe=True))
    fs_ref[("c", 512)]["channels"][0]["audio_gain"] = 0.1
    fs_got = idx(rec("sgv2", 0, 512, 0.1, float("nan"), failsafe=True))
    fs_got[("c", 512)]["channels"][0]["audio_gain"] = 0.1
    assert compare(fs_ref, fs_got)[0]
    fs_got[("c", 512)]["channels"][0]["audio_gain"] = 1.0
    assert not compare(fs_ref, fs_got)[0]
    # null on one side only fails; null on both sides is equal
    g3 = idx(rec("sgv2", 0, 512, 0.5, 60.0), rec("sgv2", 512, 512, 0.5, 60.1))
    assert compare(ref, g3)[0]
    g3[("c", 1024)]["channels"][0]["ol_temp_c"] = 50.0
    assert not compare(ref, g3)[0]

    # Closed-loop flags are always compared when the reference carries them.
    assert not compare(idx(rec("macos", 0, 512, 0.5, 60.0, cl=[True, False])),
                       idx(rec("sgv2", 0, 512, 0.5, 60.0, cl=[True])))[0]
    assert not compare(idx(rec("macos", 0, 512, 0.5, 60.0, cl=[True])),
                       idx(rec("sgv2", 0, 512, 0.5, 60.0, cl=None)))[0]
    ok, rep = compare(idx(rec("macos", 0, 512, 0.5, 60.0, cl=[True])),
                      idx(rec("sgv2", 0, 512, 0.5, 60.0, cl=[True, False])))
    assert ok and rep["fields"]["mode.closed_loop"]["compared"] == 1, rep
    # Missing fields and empty references fail; nothing passes vacuously.
    assert not compare({}, {})[0]
    assert not compare({}, got)[0]
    empty = idx(rec("sgv2", 0, 512, 0.5, 60.0))
    empty[("c", 512)]["channels"] = [{}]
    assert not compare(idx(rec("macos", 0, 512, 0.5, 60.0)), empty)[0]
    noglobal = idx(rec("sgv2", 0, 512, 0.5, 60.0))
    del noglobal[("c", 512)]["global"]
    ok, rep = compare(idx(rec("macos", 0, 512, 0.5, 60.0)), noglobal)
    assert not ok and rep["fields"]["global.modeled_power_w"]["missing"] == 1, rep
    r1 = idx(rec("macos", 0, 512, 0.5, 60.0))
    r1[("c", 512)]["channels"][0]["control_temp_c"] = 40.0
    assert not compare(r1, idx(rec("sgv2", 0, 512, 0.5, 60.0)))[0]
    r1[("c", 512)]["channels"][0] = {"brand_new_field": 1.0}
    ok, rep = compare(r1, idx(rec("sgv2", 0, 512, 0.5, 60.0)))
    assert not ok and rep["failures"][0]["why"] == "no tolerance for this reference field", rep
    # A case on the sgv2 side with no reference slices fails.
    other = idx(rec("sgv2", 0, 512, 0.5, 60.0))
    other[("c", 512)]["case"] = "d"
    other = {("d", 512): other[("c", 512)]}
    other.update(got)
    assert not compare(ref, other)[0]
    # A reference with only not-compared fields compares no values and fails.
    bare = idx(rec("macos", 0, 512, 0.5, 60.0))
    bare[("c", 512)].update(mode={"failsafe": None, "closed_loop": None}, channels=[{"raw": {"a": 1.0}}])
    del bare[("c", 512)]["global"]
    ok, rep = compare(bare, idx(rec("sgv2", 0, 512, 0.5, 60.0)))
    assert not ok and rep["cases"]["c"]["why"] == "no values compared", rep
    assert rep["not_compared"]["raw"]["slices"] == 1 and rep["not_compared"]["mode.failsafe"]["slices"] == 1
    # Not-compared fields are reported with their reason; the rest still compare.
    r2 = idx(rec("macos", 0, 512, 0.5, 60.0))
    r2[("c", 512)]["channels"][0]["displacement_rel_max"] = 0.3
    r2[("c", 512)]["global"]["health_bits"] = 0
    ok, rep = compare(r2, idx(rec("sgv2", 0, 512, 0.5, 60.0)))
    assert ok and set(rep["not_compared"]) >= {"displacement_rel_max", "global.health_bits"}, rep
    assert rep["fields"]["gain"]["compared"] == 1 and rep["cases"]["c"]["values"] >= 4, rep
    # spp3 axis gains (dB by name) against sgv2's linear axis array.
    r3 = idx(rec("macos", 0, 512, 0.5, 60.0))
    r3[("c", 512)]["channels"][0]["axis_gain_db"] = {"thermal": -6.0, "power": 0.0, "displacement": None}
    g3x = idx(rec("sgv2", 0, 512, 0.5, 60.0))
    g3x[("c", 512)]["channels"][0]["axis_gain"] = [1.0, None, 1.0, 1.0, 10 ** (-6.0 / 20)]
    assert compare(r3, g3x)[0], compare(r3, g3x)
    g3x[("c", 512)]["channels"][0]["axis_gain"][4] = 10 ** (-6.5 / 20)
    assert not compare(r3, g3x)[0]
    del g3x[("c", 512)]["channels"][0]["axis_gain"]
    assert not compare(r3, g3x)[0]
    # power_hard_limit_w is compared per element.
    r4 = idx(rec("macos", 0, 512, 0.5, 60.0))
    r4[("c", 512)]["global"]["power_hard_limit_w"] = [1.0, 2.0, 3.0, 4.0]
    g4 = idx(rec("sgv2", 0, 512, 0.5, 60.0))
    g4[("c", 512)]["global"]["power_hard_limit_w"] = [1.0, 2.0, 3.0, 4.0]
    assert compare(r4, g4)[0]
    g4[("c", 512)]["global"]["power_hard_limit_w"] = [1.0, 2.0, 3.1, 4.0]
    assert not compare(r4, g4)[0]
    # A fail-safe run must compare at least one non-null audio gain.
    fs_r = idx(rec("macos", 0, 512, 1.0, 50.0, failsafe=True))
    fs_r[("c", 512)]["channels"][0]["audio_gain"] = None
    fs_g = idx(rec("sgv2", 0, 512, 0.1, 50.0, failsafe=True))
    fs_g[("c", 512)]["channels"][0]["audio_gain"] = None
    ok, rep = compare(fs_r, fs_g)
    assert not ok and rep["cases"]["c"]["why"] == "fail-safe run with no compared audio gain", rep

    import os
    import struct
    import tempfile

    d = tempfile.mkdtemp()
    # load(): duplicate keys and malformed mode flags are rejected.
    pj = os.path.join(d, "t.jsonl")
    line = json.dumps(rec("macos", 0, 512, 0.5, 60.0))
    with open(pj, "w") as f:
        f.write(line + "\n" + line + "\n")
    try:
        load(pj)
        raise AssertionError("duplicate slice accepted")
    except ValueError:
        pass
    for bad_cl in (True, "yes", [1]):
        with open(pj, "w") as f:
            f.write(json.dumps(rec("macos", 0, 512, 0.5, 60.0, cl=bad_cl)) + "\n")
        try:
            load(pj)
            raise AssertionError(f"closed_loop {bad_cl!r} accepted")
        except ValueError:
            pass
    with open(pj, "w") as f:
        f.write(json.dumps(rec("macos", 0, 512, 0.5, 60.0, cl=[True, False])) + "\n")
    assert load(pj)[("c", 512)]["mode"]["closed_loop"] == [True, False]
    pa, pb = os.path.join(d, "a.f32"), os.path.join(d, "b.f32")
    sig = [0.1 * math.sin(i / 7) for i in range(400)]
    with open(pa, "wb") as f:
        f.write(struct.pack("<400f", *sig))
    with open(pb, "wb") as f:
        f.write(struct.pack("<403f", 0, 0, 0, *[x + 2e-6 for x in sig]))
    assert not compare_audio(pa, pb, 2, 0, 2)[0]  # not a whole number of frames
    with open(pb, "wb") as f:
        f.write(struct.pack("<404f", 0, 0, 0, 0, *[x + 2e-6 for x in sig]))
    assert compare_audio(pa, pb, 2, 0, 2)[0]
    with open(pb, "wb") as f:
        f.write(struct.pack("<404f", 0, 0, 0, 0, *[x + 2e-4 for x in sig]))
    assert not compare_audio(pa, pb, 2, 0, 2)[0]
    # Non-finite audio fails on either side, even when the other side is finite.
    with open(pa, "wb") as f:
        f.write(struct.pack("<4f", 0.1, 0.2, 0.3, 0.4))
    with open(pb, "wb") as f:
        f.write(struct.pack("<4f", *[math.nan] * 4))
    ok, rep = compare_audio(pa, pb, 1)
    assert not ok and "sgv2" in rep["non_finite"], rep
    assert not compare_audio(pb, pa, 1)[0]
    with open(pb, "wb") as f:
        f.write(struct.pack("<4f", 0.1, math.inf, 0.3, 0.4))
    assert not compare_audio(pa, pb, 1)[0]
    with open(pb, "wb") as f:
        f.write(struct.pack("<5f", math.nan, 0.1, 0.2, 0.3, 0.4))  # outside the aligned span
    assert not compare_audio(pa, pb, 1, 0, 1)[0]
    with open(pb, "wb") as f:
        f.write(struct.pack("<5f", 0.0, 0.1, 0.2, 0.3, 0.4))
    assert compare_audio(pa, pb, 1, 0, 1)[0]
    print("self-test OK")
    return 0


def jsonable(x):
    if isinstance(x, float) and not math.isfinite(x):
        return None
    if isinstance(x, dict):
        return {k: jsonable(v) for k, v in x.items()}
    if isinstance(x, list):
        return [jsonable(v) for v in x]
    return x


def main(argv):
    if argv == ["--self-test"]:
        return self_test()
    args = list(argv)
    opts = {}
    for flag, n in (("--audio", 2), ("--ch", 1), ("--report", 1), ("--plot", 1)):
        if flag in args:
            i = args.index(flag)
            opts[flag] = args[i + 1:i + 1 + n]
            del args[i:i + 1 + n]
    if len(args) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    ref, got = load(args[0]), load(args[1])
    ok, report = compare(ref, got)
    if "--audio" in opts:
        def latency(t):
            return next(iter(t.values())).get("latency", 0) if t else 0

        aok, areport = compare_audio(*opts["--audio"], int(opts.get("--ch", ["1"])[0]), latency(ref), latency(got))
        report["audio"] = areport
        ok = ok and aok
        report["pass"] = ok
    if "--plot" in opts:
        plot(ref, got, opts["--plot"][0])
    text = json.dumps(jsonable(report), indent=2, sort_keys=True)
    if "--report" in opts:
        with open(opts["--report"][0], "w") as fd:
            fd.write(text + "\n")
    print(text)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]) or 0)
