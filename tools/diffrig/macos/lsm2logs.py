#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Black-box decoder for the lsm2 debug logs the rig captures (LSMv2EnableLogging).

File layout: little-endian float32. Word 0 is the system (channel) count N. Then
records, each `n` (frames covered) followed by a payload whose size depends on the
log, N and n (the lsm2 behaviour specification, native logging contract).
"""
import glob, os, sys
import numpy as np

# payload size as a function of (nch, n)
LAYOUT = {
    "ControllerThermalLog": lambda c, n: c * 5,
    "ControllerPowerLog": lambda c, n: c * 6 * n,
    "ControllerSignalLog": lambda c, n: c * 5 * n,
    "ControllerFrameLog": lambda c, n: 5 * c + 5,  # axis0..2, vel limit, final per system; 4 power limits; global
    "LsmFrameLog": lambda c, n: c,
    "LsmSignalLog": lambda c, n: c * n,
}

# per channel in ControllerThermalLog
THERMAL_FIELDS = ["theta_vc", "theta_mag", "ambient_c", "t_vc_c", "t_control_c"]


def parse(path, name):
    a = np.fromfile(path, "<f4")
    if a.size < 2:
        return int(a[0]) if a.size else 0, []
    nch = int(a[0])
    f = LAYOUT[name]
    i, out = 1, []
    while i < a.size:
        n = a[i]
        if not np.isfinite(n) or n <= 0 or n != int(n):
            raise ValueError(f"{name}: bad record length {n} at word {i}")
        n = int(n)
        sz = f(nch, n)
        if i + 1 + sz > a.size:
            break  # trailing partial record
        out.append((n, a[i + 1:i + 1 + sz]))
        i += 1 + sz
    return nch, out


def load_dir(d):
    res = {}
    for name in LAYOUT:
        g = glob.glob(os.path.join(d, f"*_{name}.bin"))
        if not g:
            continue
        nch, recs = parse(g[0], name)
        frames = np.cumsum([0] + [n for n, _ in recs])[:-1]
        res[name] = {"nch": nch, "frame": frames, "n": np.array([n for n, _ in recs]),
                     "data": [p for _, p in recs]}
    res["overwrites"] = len(glob.glob(os.path.join(d, "*OVERWRITE*")))
    return res


def thermal_array(res):
    t = res["ControllerThermalLog"]
    return t["frame"], np.stack(t["data"]).reshape(len(t["data"]), t["nch"], 5)


if __name__ == "__main__":
    r = load_dir(sys.argv[1])
    for k, v in r.items():
        if k == "overwrites":
            print("overwrites", v)
            continue
        ns = np.unique(v["n"])
        print(k, "records", len(v["data"]), "n values", ns, "frames covered", int(v["n"].sum()))
