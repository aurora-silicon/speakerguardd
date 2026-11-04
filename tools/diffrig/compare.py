#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compare a macOS reference trace with an sgv2 trace, slice by slice.

Both inputs are JSON Lines in tools/diffrig/schema.json. A reference line is
compared with the sgv2 line whose slice ends on the same input frame
(frame + frames), so the two sides may slice the program differently: the
state after the same input is compared. Fields absent on either side are
skipped; a field present on one side and null on the other is a failure.

Mode flags (fail-safe, closed loop per channel) must agree in every compared
slice, so every mode transition lands in the same slice. In a reference
fail-safe slice only the audio-measured gain is compared, because the macOS
unit's meters do not show the fail-safe attenuation.

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
TODO(SPEC): tolerances for lsm2 mask and velocity state once its plan fixes them.
"""
import json
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
}
AUDIO_TOLERANCE = 1e-5
# Relative errors use max(|ref|, floor) so values near zero do not give
# meaningless ratios.
REL_FLOOR = {"modeled_power_w": 1e-3, "measured_power_w": 1e-3, "displacement_m": 1e-6, "velocity_m_s": 1e-4}


def load(path):
    out = {}
    with open(path) as fd:
        for n, line in enumerate(fd, 1):
            if not line.strip():
                continue
            rec = json.loads(line)
            for k in ("schema", "side", "unit", "case", "frame", "frames", "channels"):
                if k not in rec:
                    raise ValueError(f"{path}:{n}: missing {k}")
            out[(rec["case"], rec["frame"] + rec["frames"])] = rec
    return out


def error(name, kind_tol, ref, got):
    """Return (error, ok). None on both sides is equal; on one side it fails."""
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
    return err, err <= tol


class Worst:
    def __init__(self):
        self.fields = {}
        self.fail = []

    def add(self, name, err, ok, where):
        w = self.fields.get(name)
        if w is None or err > w["error"]:
            self.fields[name] = {"error": err, "at": where}
        if not ok and len(self.fail) < 50:
            self.fail.append({"field": name, "error": err, "at": where})


def compare(ref, got):
    worst = Worst()
    cases = {}
    missing = 0
    for key in sorted(ref):
        r = ref[key]
        case, end = key
        c = cases.setdefault(case, {"slices": 0, "pass": True})
        g = got.get(key)
        if g is None:
            missing += 1
            c["pass"] = False
            continue
        c["slices"] += 1
        where = {"case": case, "end_frame": end}
        ok_slice = True
        rm, gm = r.get("mode") or {}, g.get("mode") or {}
        if rm.get("failsafe") is not None and bool(rm.get("failsafe")) != bool(gm.get("failsafe")):
            worst.add("mode.failsafe", math.inf, False, where)
            ok_slice = False
        rcl, gcl = rm.get("closed_loop"), gm.get("closed_loop")
        if isinstance(rcl, list) and rcl != (gcl or [])[: len(rcl)]:
            worst.add("mode.closed_loop", math.inf, False, where)
            ok_slice = False
        for name, tol in GLOBAL_TOLERANCE.items():
            rv = (r.get("global") or {}).get(name)
            gv = (g.get("global") or {}).get(name, "absent")
            if rv is None or gv == "absent":
                continue
            err, ok = error(name, tol, rv, gv)
            worst.add("global." + name, err, ok, where)
            ok_slice &= ok
        if len(g["channels"]) < len(r["channels"]):
            worst.add("channels", math.inf, False, where)
            ok_slice = False
        # In the macOS fail-safe the published meters keep their last model values
        # (gain 0 dB); only the audio shows the attenuation. Compare the audio there.
        fields = ("audio_gain",) if rm.get("failsafe") else TOLERANCE
        for ch, (rc, gc) in enumerate(zip(r["channels"], g["channels"])):
            for name in fields:
                tol = TOLERANCE[name]
                if name not in rc or name not in gc:
                    continue
                err, ok = error(name, tol, rc[name], gc[name])
                worst.add(name, err, ok, dict(where, channel=ch))
                ok_slice &= ok
        c["pass"] &= ok_slice
    ok = missing == 0 and not worst.fail and all(c["pass"] for c in cases.values())
    return ok, {"pass": ok, "missing_slices": missing, "cases": cases, "worst": worst.fields, "failures": worst.fail}


def read_f32(path):
    import array

    a = array.array("f")
    with open(path, "rb") as fd:
        a.frombytes(fd.read())
    if sys.byteorder != "little":
        a.byteswap()
    return a


def compare_audio(ref_path, got_path, channels, ref_latency=0, got_latency=0):
    a, b = read_f32(ref_path), read_f32(got_path)
    if len(a) % channels or len(b) % channels:
        return False, {"pass": False, "error": "file length is not a whole number of frames"}
    na, nb = len(a) // channels, len(b) // channels
    n = min(na - ref_latency, nb - got_latency)
    worst, at = 0.0, None
    for i in range(max(n, 0)):
        for c in range(channels):
            d = abs(a[(i + ref_latency) * channels + c] - b[(i + got_latency) * channels + c])
            if d > worst:
                worst, at = d, {"frame": i, "channel": c}
    ok = n > 0 and worst <= AUDIO_TOLERANCE and na - ref_latency == nb - got_latency
    return ok, {"pass": ok, "frames": n, "max_abs_error": worst, "at": at, "tolerance": AUDIO_TOLERANCE}


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

    import os
    import struct
    import tempfile

    d = tempfile.mkdtemp()
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
