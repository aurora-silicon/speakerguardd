#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Checks the spp3 behavioural laws against the spp3-law-* captures (and, where named, the
older golden traces as held-out data). Prints one line per case and law; exit status 1 if any
check fails. Reads private rig data; writes nothing.

  validate_laws.py RIG_TRACES_DIR [law ...]      (laws: pdrv pcoil tcl env hold noff restore)
"""
import glob, gzip, json, math, os, plistlib, re, sys
import numpy as np

SR = 48000
TUN = "/System/Library/Audio/Tunings/AID35/DSP/Strips/aid35-aufx-spp3-appl.plist"
f32 = np.float32
FAIL = []


def say(ok, name, law, msg):
    print(f"{'PASS' if ok else 'FAIL'} {law:7s} {name:40s} {msg}")
    if not ok:
        FAIL.append((name, law))


def arr(path, n):
    if os.path.exists(path + ".gz"):
        return np.frombuffer(gzip.open(path + ".gz").read(), np.float32).reshape(-1, n)
    return np.fromfile(path, np.float32).reshape(-1, n)


class Case:
    def __init__(self, d, rig):
        self.d, self.name = d, os.path.basename(d)
        self.meta = json.load(open(os.path.join(d, "run.meta.json")))
        c = self.meta["case"]
        self.x = np.fromfile(os.path.join(rig, "signals", f"{c['signal']}.6ch.f32"), np.float32).reshape(-1, 6)
        self.y = arr(os.path.join(d, "run.out.f32"), 6)
        z = np.load(os.path.join(d, "clsp.npz"))
        self.ctl, self.tc = z["ctl"], z["t_coil"]
        sc = os.path.join(d, "sidecar.plist")
        self.sc = plistlib.load(open(sc if os.path.exists(sc) else TUN, "rb"))
        self.params = {}
        hl = os.path.join(d, "host.log")
        for line in open(hl) if os.path.exists(hl) else []:
            m = re.match(r"params\[init\] (\d+)\.(\d+)\.(\d+) .* = (\S+)", line)
            if m:
                self.params[(int(m[1]), int(m[2]), int(m[3]))] = float(m[4])

    def ch(self, i):
        return self.sc["ChannelSpecificParams"][f"Channel{i}"]

    def power(self, sig, i):
        p = self.ch(i)
        ag, reb = p["OL_disp"]["ag"], p["OL_thermal"]["Reb_ref"]
        r = reb + p["OL_thermal"]["Rshunt"] + p["OL_thermal"].get("Rampout", 0)
        cur = ag * sig.astype(np.float64) / r
        n = min(len(cur) // 256, self.ctl.shape[1])
        return (cur[: n * 256] ** 2 * reb).reshape(n, 256).mean(1), n


def law_power(c, which):
    worst = 0.0
    for i in range(6):
        sig = c.x[:, i] if which == "pdrv" else c.y[:, i]
        P, n = c.power(sig, i)
        col = 8 if which == "pdrv" else 7
        m = P > 1e-6
        e = np.abs(c.ctl[i, :n, col][m] / P[m] - 1)
        z = np.abs(c.ctl[i, :n, col][~m]).max() if (~m).any() else 0.0
        worst = max(worst, e.max(), z)
    say(worst < 1e-6, c.name, which, f"max relative error {worst:.2e} over {c.ctl.shape[1]} updates x 6 ch")


def law_pilotoff(c, root, rig):
    # Part parameter 21 (pilot tone enabled) set to 0 with AudioUnitSetParameter, before and
    # after Initialize, has no effect: the output is identical to the same case without it.
    ref = Case(os.path.join(root, c.name.replace("-pilotoff", "")), rig)
    same = c.y.shape == ref.y.shape and bool((c.y == ref.y).all())
    say(same, c.name, "pcoil", "parameter 21 = 0 ignored: output bit-identical to " + ref.name)
    law_power(c, "pcoil")


def inv_tempco(a20, a35, tcal):
    a20, a35, tcal = (float(f32(v)) for v in (a20, a35, tcal))
    return f32(1.0 / (a20 + (tcal - 20.0) * (a35 - a20) / 15.0))


def fma32(a, b, c):
    return (np.float64(a) * np.float64(b) + np.float64(c)).astype(f32)


def law_tcl(c, rig):
    fx = [p for p in c.meta["cfprops"] if p["id"] == 64009][0]
    recs = json.load(open(os.path.join(rig, "cal", fx["fixture"])))
    nok = ntot = inv_ok = 0
    for i in range(6):
        cl = c.ch(i)["CL"]
        rcal, tcal = f32(recs[i]["thiele_small_re"]), f32(recs[i]["thiele_small_t"])
        inv = inv_tempco(cl["a_t_20C"], cl["a_t_35C"], tcal)
        inv_ok += inv == f32(c.params[(4, i, 26)])
        rsh = f32(c.ch(i)["OL_thermal"]["Rshunt"])
        T, Rw = c.tc[i, :, 5], c.tc[i, :, 6].astype(f32)
        m = np.r_[False, np.diff(T) != 0]
        pred = fma32(inv, ((Rw[m] - rsh) / (rcal - rsh) - f32(1)), tcal)
        nok += int((pred == T[m]).sum())
        ntot += int(m.sum())
    say(inv_ok == 6 and nok == ntot and ntot > 0, c.name, "tcl",
        f"invTempCo {inv_ok}/6 bit-exact; T_CL {nok}/{ntot} windows bit-exact")


def k99(t, rate):
    return 1 - math.exp(math.log(0.01) / (t * rate))


def pilot_model(targets, t1, t2, nsamp):
    kb, ks = f32(k99(t1, SR / 256)), f32(k99(t2 / 2, SR))
    s1 = a = b = f32(0)
    e = np.zeros(len(targets) * 256, np.float32)
    i = 0
    for k in range(len(targets)):
        for _ in range(256):
            a = f32(a + ks * f32(s1 - a))
            b = f32(b + ks * f32(a - b))
            e[i] = b
            i += 1
        s1 = f32(s1 + kb * f32(f32(targets[k]) - s1))
    n = np.arange(len(e))
    return (e.astype(np.float64) * np.sin(2 * np.pi * 15 * n / 16384))[:nsamp]


def law_env(c):
    cl = c.ch(0)["CL"]
    hi = min(10 ** (cl["PilotAmplHi_dB"] / 20), 0.3)
    head = f32(1) - f32(hi)
    worst = 0.0
    for i in range(6):
        pp = pilot_model(c.ctl[i, :, 13].astype(np.float64), cl["PilotDecayTime"], cl["PilotDecayTimeStage2"], len(c.y))
        n = min(len(pp), len(c.y), len(c.x))
        err = c.y[:n, i] - ((head * c.x[:n, i]).astype(np.float64) + pp[:n])
        worst = max(worst, np.abs(err).max())
    say(worst < 1e-7, c.name, "env", f"output model max |error| {worst:.2e} (peak output {np.abs(c.y).max():.2f})")


def hold_sim(ctl, nhold, amb):
    g = np.sqrt(np.clip((ctl[:, 3] - amb - ctl[:, 6]) / (ctl[:, 4] + 1), 1e-4, 1)).astype(np.float64)
    sel = ctl[:, 2].astype(np.float64)
    u = np.zeros(len(g))
    u[0] = g[0]
    state, cnt, held = "attack", 0, 0.0
    for k in range(1, len(g)):
        gp = sel[k - 1]
        if cnt > 0:
            if g[k] >= gp:
                u[k] = held
                cnt -= 1
                if cnt == 0:
                    state = "release"
                continue
            cnt = 0
        if g[k] >= gp:
            if state == "attack":
                held, cnt, state = u[k - 1], nhold - 1, "hold"
                u[k] = held
            else:
                u[k] = g[k]
        else:
            state, u[k] = "attack", g[k]
    return u, g, sel


def law_hold(c):
    sr = c.meta.get("sr", SR)
    upd = 256 * max(1, round(sr / SR))
    nhold = math.ceil(sr / upd * 1.0)
    amb = c.sc["Thermal_AmbientTemp"]
    bad = ties = 0
    runs = []
    for i in range(6):
        u, g, sel = hold_sim(c.ctl[i], nhold, amb)
        e = np.abs(u - c.ctl[i, :, 0]) > 2e-6
        # a near-tie (fresh candidate within 1e-7 of the previous selected gain) is decided by
        # float rounding the logs do not carry
        tie = e & (np.abs(g - np.r_[1.0, sel[:-1]]) < 1e-7)
        bad += int((e & ~tie).sum())
        ties += int(tie.sum())
        held = np.r_[False, c.ctl[i, 1:, 0] == c.ctl[i, :-1, 0]] & (np.abs(g - c.ctl[i, :, 0]) > 1e-7)
        k = 0
        while k < len(held):
            if held[k]:
                j = k
                while j < len(held) and held[j]:
                    j += 1
                if j - k > 2:
                    runs.append(j - k)
                k = j
            else:
                k += 1
    say(bad == 0, c.name, "hold", f"N={nhold}; sim mismatches {bad} (+{ties} ties) of {c.ctl.size // 14}; "
        f"hold lengths {sorted(set(runs))}")


def law_noff(c):
    amb = c.sc["Thermal_AmbientTemp"]
    dt = 256 / SR
    worst, nan_ok, chans = 0.0, True, 0
    for i in range(6):
        ot = c.ch(i)["OL_thermal"]
        if ot["ThermalFFSpeedupFactor"] != 0:
            continue
        chans += 1
        nan_ok &= bool(np.isnan(c.ctl[i, :, 4]).all())
        p0 = (ot["TemperatureLimit"] - amb) / (ot["T_sett_vc"] + ot["T_sett_mg"])
        P = c.ctl[i, :, 7].astype(np.float64)
        for key, ts, tau, col in (("vc", ot["T_sett_vc"], ot["tau_Tvc"], 5), ("mg", ot["T_sett_mg"], ot["tau_Tmg"], 6)):
            pred = (1 - dt / tau) * p0 * ts + 0.5 * ts * dt / tau * (P[1:] + P[:-1])
            worst = max(worst, np.abs(pred - c.ctl[i, 1:, col]).max())
    say(nan_ok and worst < 2e-3 and chans, c.name, "noff",
        f"{chans} ch with FF=0: FF node NaN={nan_ok}; per-update hot re-seed max error {worst:.1e} K")


# expected outcome per restore case: (base limit after the event, thermal state after it)
RESTORE = {"none": (100, "continue"), "reset": (100, "continue"), "classinfo": (100, "continue"),
           "uninit": (140, "cold"), "reinit": (140, "cold"), "newinst": (140, "cold"), "newinstci": (140, "cold"),
           "uninit0": (140, "restore"), "newinst0": (140, "restore"), "newinstci0": (140, "restore"),
           "newinst1": (140, "hot"), "cl-reset": (100, "continue"), "cl-newinst0": (140, "restore")}


def law_restore(c, key):
    want_lim, want_state = RESTORE[key]
    K = 6 * SR // 256
    ok, notes = True, []
    rec = c.meta.get("clsp_records", {}).get("ctl", [c.ctl.shape[1]] * 6)
    for i in range(6):
        if rec[i] < 12 * SR // 256:
            notes.append(f"ch{i} records {rec[i]} (skipped)")
            continue
        ctl = c.ctl[i]
        ot = c.ch(i)["OL_thermal"]
        amb = c.sc["Thermal_AmbientTemp"]
        before, after = ctl[K - 1], ctl[K]
        # open loop: the raw limit (ctl[10]) is the base limit; closed loop: the CL estimator
        # restarts, so TlimAdj is 0 at once and the effective limit (ctl[3]) is the base
        lim = ctl[K, 3] if c.meta["case"].get("cfprops") else ctl[K + 1, 10]
        lim_ok = abs(lim - want_lim) < 0.01
        dt = 256 / SR
        a = 1 - dt / ot["tau_Tvc"]
        b = 0.5 * ot["T_sett_vc"] * dt / ot["tau_Tvc"]
        P = ctl[:, 7].astype(np.float64)
        if want_state == "continue":
            pred = a * before[5] + b * (P[K] + P[K - 1])
        elif want_state == "restore":
            pred = a * before[5] + b * P[K]  # previous-power memory restarts at 0
        elif want_state == "cold":
            pred = b * P[K]
        else:  # hot seed
            p0 = (ot["TemperatureLimit"] - amb) / (ot["T_sett_vc"] + ot["T_sett_mg"])
            pred = a * p0 * ot["T_sett_vc"] + b * P[K]
        st_ok = abs(pred - after[5]) < 2e-3 * max(1.0, abs(after[5]))
        ok &= bool(lim_ok and st_ok)
        if not (lim_ok and st_ok):
            notes.append(f"ch{i} lim {lim:.2f} vc {after[5]:.4f} pred {pred:.4f}")
    say(ok, c.name, "restore", f"expect base {want_lim} and thermal state '{want_state}' "
        + ("; ".join(notes) if notes else "(all channels)"))


def main(argv):
    root = argv[0]
    rig = os.path.dirname(os.path.abspath(root))
    laws = set(argv[1:]) or {"pdrv", "pcoil", "tcl", "env", "hold", "noff", "restore"}
    for d in sorted(glob.glob(os.path.join(root, "spp3-law-*"))):
        name = os.path.basename(d)[len("spp3-law-"):]
        if not os.path.exists(os.path.join(d, "clsp.npz")) or not os.path.exists(os.path.join(d, "trace.jsonl.gz")):
            continue  # incomplete capture
        c = Case(d, rig)
        if name.startswith("pdrv") and laws & {"pdrv", "pcoil"}:
            law_power(c, "pdrv")
            law_pilotoff(c, root, rig) if "pilotoff" in name else law_power(c, "pcoil")
        elif name.startswith("pcoil") and "pcoil" in laws:
            law_power(c, "pcoil")
            law_power(c, "pdrv")
        elif name.startswith("tcl") and "tcl" in laws:
            law_tcl(c, rig)
        elif name.startswith("env") and "env" in laws:
            law_env(c)
        elif name.startswith("hold") and "hold" in laws:
            law_hold(c)
        elif name.startswith("noff") and "noff" in laws:
            law_noff(c)
        elif name.startswith("restore") and "restore" in laws:
            law_restore(c, name[len("restore-"):])
    # held out: the earlier goldens (designed before these laws)
    if "hold" in laws or "pcoil" in laws:
        for d in sorted(glob.glob(os.path.join(root, "spp3-cl-*"))):
            if os.path.exists(os.path.join(d, "clsp.npz")):
                c = Case(d, rig)
                if "hold" in laws:
                    law_hold(c)
                if "pcoil" in laws and os.path.exists(os.path.join(d, "run.out.f32.gz")):
                    law_power(c, "pcoil")
    print(f"{len(FAIL)} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
