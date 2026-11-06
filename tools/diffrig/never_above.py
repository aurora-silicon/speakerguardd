#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Never-above-macOS gate: per-driver drive, excursion, velocity and temperature.

The differential rig (compare.py) checks that a unit matches macOS slice by
slice. This gate asks a narrower question about the whole speaker path, the
voicing chain included: for every tuning, case, volume and driver, is the
signal v2 sends to the amplifier ever harder on the driver than the signal
macOS sends? It is independent of both renderers: it reads the per-driver
amplifier-input audio of each side and applies its own driver models, so a
bug shared by a v2 engine and its own telemetry cannot hide here.

Metrics per driver (both sides, same model):
  band_peak_db  per third-octave band 20 Hz-20 kHz, the loudest short-time
                frame (Hann, about 1/3 s)
  band_rms_db   per band, over the whole case
  peak_db       sample peak
  win_rms_db    loudest sliding RMS over 10 ms, 100 ms, 1 s and 10 s
  excursion_m   peak cone displacement, linear moving-coil model
  velocity_m_s  peak cone velocity, same model
  coil_temp_c   peak voice-coil temperature, two-node thermal model on the
  magnet_temp_c coil power (current through Reb + Rshunt + Rampout)

Rule: v2 <= macOS + tolerance on every metric, every band, every driver. A
band where v2 is below the measurement floor is not compared (macOS may be
far below it too), nor a driver whose whole v2 signal is below it. Anything missing fails: a tuning without a required case
or volume, a driver without a model, a file of the wrong size, a non-finite
sample.

Manifest (JSON; Apple-derived paths stay outside this repository):
  {
    "required": {"cases": ["sine_125_0", ...], "volumes": [1.0, 0.5]},
    "tolerance": {"db": 0.05, "rel": 1e-3, "temp_c": 0.02, "floor_db": -150},
    "tunings": [{
      "name": "j413",
      "fs": 48000,
      "drivers": ["woofer_l", "woofer_r", "tweeter_l", "tweeter_r"],
      "models": "models.json",
      "cases": [{"case": "sine_125_0", "volume": 1.0,
                 "macos": "mac/sine_125_0_v1.f32", "v2": "v2/sine_125_0_v1.f32"}]
    }]
  }
Each audio file is interleaved little-endian float32, one channel per driver
in the "drivers" order, in full-scale units at the amplifier input. Relative
paths resolve against the manifest's directory.

Driver models (models.json, one object per driver name; every key required),
named as the tuning sidecar names them:
  ag                   amplifier output, volts peak at digital full scale
  Reb_ref              voice-coil DC resistance, ohm
  Rshunt, Rampout      current-sense shunt and amplifier output resistance, ohm
  phi, mt, kt, ct      force factor T*m, moving mass kg, stiffness N/m,
                       mechanical damping N*s/m
  T_sett_vc, tau_Tvc   coil thermal resistance K/W (above the magnet), time s
  T_sett_mg, tau_Tmg   magnet thermal resistance K/W, time s
  Thermal_AmbientTemp  degC
`never_above.py models SIDECAR.plist OUT.json` writes them from the sidecar,
driver "chN" = ChannelSpecificParams/ChannelN = unit channel N (the unit
indexes its channel tuning by part element, not by SpeakerName). The gate
refuses a model with a missing or non-finite key rather than guess one.

Usage:
  never_above.py gate MANIFEST [--report OUT.json]
  never_above.py signals OUT_DIR [--fs 48000]   write the required case set
  never_above.py models SIDECAR.plist OUT.json  per-driver models from a tuning
  never_above.py --self-test
Exit status 0 only if every check passes.
"""
import cmath
import json
import math
import os
import struct
import sys

try:
    import numpy as np
except ImportError:  # CI's self-test runs without numpy
    np = None

DEFAULT_TOL = {"db": 0.05, "rel": 1e-3, "temp_c": 0.02, "floor_db": -150.0}
WINDOWS_S = (0.01, 0.1, 1.0, 10.0)
# The tuning sidecar's own field names (open-loop displacement and thermal
# dictionaries of ChannelSpecificParams/ChannelN, ambient from the globals).
MODEL_KEYS = ("ag", "Reb_ref", "Rshunt", "Rampout", "phi", "mt", "kt", "ct",
              "T_sett_vc", "tau_Tvc", "T_sett_mg", "tau_Tmg", "Thermal_AmbientTemp")
THERMAL_STEP_S = 0.001

# Required case set. Levels in dBFS at the program input; the voicing chain
# under test runs at each required volume.
SINE_HZ = (20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400,
           500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000,
           6300, 8000, 10000, 12500, 16000, 20000)
SINE_DBFS = (0, -12)
PHASE_HZ_MAX = 500  # low tones also run with the channels in anti-phase
BURST_HZ = (63, 125, 250, 1000)
# Two tones: full-scale sub-bass under a tone near woofer resonance. A
# multiband compressor's detector placed after the driver filters no longer
# sees the sub-bass, so single tones cannot show a chain-order difference.
DUO_SUB_HZ = (25, 31.5, 40)
DUO_TONE_HZ = (100, 125, 160, 200, 250)
DEFAULT_VOLUMES = (1.0, 0.5, 0.25)


def band_edges():
    centres = [1000.0 * 2 ** (k / 3) for k in range(-17, 14)]
    return [(c, c * 2 ** (-1 / 6), c * 2 ** (1 / 6)) for c in centres]


BANDS = band_edges()


# ---------------------------------------------------------------- FFT helpers

def _fft_py(x):
    n = len(x)
    a = [complex(v) for v in x]
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j |= bit
        if i < j:
            a[i], a[j] = a[j], a[i]
    size = 2
    while size <= n:
        w = cmath.exp(-2j * math.pi / size)
        for s in range(0, n, size):
            wk = 1.0
            for k in range(size // 2):
                u, t = a[s + k], a[s + k + size // 2] * wk
                a[s + k], a[s + k + size // 2] = u + t, u - t
                wk *= w
        size *= 2
    return a


def rfft(x, n):
    """Spectrum bins 0..n/2 of x zero-padded to n (a power of two)."""
    if np is not None:
        return list(np.fft.rfft(np.asarray(x, dtype=np.float64), n))
    return _fft_py(list(x) + [0.0] * (n - len(x)))[: n // 2 + 1]


def irfft(spec, n):
    if np is not None:
        return list(np.fft.irfft(np.asarray(spec), n))
    full = list(spec) + [s.conjugate() for s in reversed(spec[1:-1])]
    out = _fft_py([s.conjugate() for s in full])
    return [v.real / n for v in out]


def pow2(n):
    p = 1
    while p < n:
        p *= 2
    return p


def db(power):
    return 10 * math.log10(power) if power > 0 else -math.inf


# ------------------------------------------------------------------ metrics

def band_powers(spec, n, fs):
    out = []
    for _, lo, hi in BANDS:
        k0, k1 = int(math.ceil(lo * n / fs)), int(math.floor(hi * n / fs))
        k1 = min(k1, len(spec) - 1)
        p = sum(abs(spec[k]) ** 2 for k in range(max(k0, 1), k1 + 1)) if k1 >= k0 else 0.0
        out.append(2 * p / (n * n))
    return out


def stft_band_peak(x, fs):
    n = pow2(int(fs / 3))
    hop = n // 4
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / n) for i in range(n)]
    norm = sum(w * w for w in win) / n
    best = [0.0] * len(BANDS)
    start = 0
    while True:
        seg = x[start:start + n]
        if not seg:
            break
        spec = rfft([s * w for s, w in zip(seg, win)], n)
        for b, p in enumerate(band_powers(spec, n, fs)):
            best[b] = max(best[b], p / norm)
        if start + n >= len(x):
            break
        start += hop
    return best


def window_rms(x, fs):
    out = {}
    sq = [v * v for v in x]
    for w in WINDOWS_S:
        m = max(1, int(round(w * fs)))
        if m > len(sq):
            out[str(w)] = None
            continue
        acc = sum(sq[:m])
        best = acc
        for i in range(m, len(sq)):
            acc += sq[i] - sq[i - m]
            best = max(best, acc)
        out[str(w)] = db(best / m)
    return out


def mechanics(x, fs, m):
    """Peak excursion and velocity of the linear moving-coil model."""
    rtot = m["Reb_ref"] + m["Rshunt"] + m["Rampout"]
    tail = int(fs * 8 * m["mt"] / max(m["ct"] + m["phi"] ** 2 / rtot, 1e-12)) + 1
    n = pow2(len(x) + min(tail, 4 * len(x) + fs))
    if np is not None:
        sp = np.fft.rfft(np.asarray(x, dtype=np.float64) * m["ag"], n)
        w = 2 * np.pi * np.arange(len(sp)) * fs / n
        h = (m["phi"] / rtot) / ((m["kt"] - m["mt"] * w * w) + 1j * w * (m["ct"] + m["phi"] ** 2 / rtot))
        return (float(np.abs(np.fft.irfft(sp * h, n)).max()), float(np.abs(np.fft.irfft(sp * h * 1j * w, n)).max()))
    spec = rfft([v * m["ag"] for v in x], n)
    xs, vs = [], []
    for k, s in enumerate(spec):
        w = 2 * math.pi * k * fs / n
        h = (m["phi"] / rtot) / complex(m["kt"] - m["mt"] * w * w, w * (m["ct"] + m["phi"] ** 2 / rtot))
        xs.append(s * h)
        vs.append(s * h * 1j * w)
    return (max(abs(v) for v in irfft(xs, n)), max(abs(v) for v in irfft(vs, n)))


def thermal(x, fs, m):
    """Peak coil and magnet temperature: coil power through two first-order nodes."""
    rtot = m["Reb_ref"] + m["Rshunt"] + m["Rampout"]
    scale = m["ag"] ** 2 * m["Reb_ref"] / rtot ** 2
    step = max(1, int(round(THERMAL_STEP_S * fs)))
    dt = step / fs
    avc = math.exp(-dt / m["tau_Tvc"])
    amg = math.exp(-dt / m["tau_Tmg"])
    tvc = tmg = 0.0
    pk_vc = pk_mg = m["Thermal_AmbientTemp"]
    if np is not None:
        a = np.asarray(x, dtype=np.float64) ** 2
        pad = (-len(a)) % step
        cnt = np.full(len(a) // step + (1 if pad else 0), float(step))
        if pad:
            cnt[-1] = step - pad
        powers = (np.concatenate((a, np.zeros(pad))).reshape(-1, step).sum(axis=1) / cnt * scale).tolist()
    else:
        powers = [scale * sum(v * v for v in x[i:i + step]) / len(x[i:i + step]) for i in range(0, len(x), step)]
    for p in powers:
        tvc = avc * tvc + (1 - avc) * m["T_sett_vc"] * p
        tmg = amg * tmg + (1 - amg) * m["T_sett_mg"] * p
        pk_mg = max(pk_mg, m["Thermal_AmbientTemp"] + tmg)
        pk_vc = max(pk_vc, m["Thermal_AmbientTemp"] + tmg + tvc)
    return pk_vc, pk_mg


def _driver_metrics_np(x, fs, model):
    x = np.asarray(x, dtype=np.float64)
    n = pow2(len(x))
    spec = np.fft.rfft(x, n)
    freqs = np.arange(len(spec)) * fs / n
    pw = np.abs(spec) ** 2
    rms = []
    for _, lo, hi in BANDS:
        sel = (freqs >= lo) & (freqs <= hi) & (freqs > 0)
        rms.append(db(float(2 * pw[sel].sum() / (n * n))))
    m = pow2(int(fs / 3))
    hop = m // 4
    win = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(m) / m)
    norm = float((win ** 2).sum() / m)
    starts = list(range(0, max(len(x) - m, 0) + 1, hop))
    if starts[-1] + m < len(x):
        starts.append(starts[-1] + hop)
    fpk = np.zeros(len(BANDS))
    ff = np.arange(m // 2 + 1) * fs / m
    masks = [(ff >= lo) & (ff <= hi) & (ff > 0) for _, lo, hi in BANDS]
    for i in range(0, len(starts), 64):
        seg = np.zeros((len(starts[i:i + 64]), m))
        for j, st in enumerate(starts[i:i + 64]):
            chunk = x[st:st + m]
            seg[j, :len(chunk)] = chunk
        p = np.abs(np.fft.rfft(seg * win, m, axis=1)) ** 2
        for b, mk in enumerate(masks):
            fpk[b] = max(fpk[b], float((2 * p[:, mk].sum(axis=1) / (m * m)).max()) / norm)
    c = np.concatenate(([0.0], np.cumsum(x * x)))
    wr = {}
    for w in WINDOWS_S:
        k = max(1, int(round(w * fs)))
        wr[str(w)] = db(float((c[k:] - c[:-k]).max() / k)) if k <= len(x) else None
    out = {"band_rms_db": rms, "band_peak_db": [db(float(p)) for p in fpk],
           "peak_db": db(float((x * x).max())), "win_rms_db": wr}
    if model is not None:
        out["excursion_m"], out["velocity_m_s"] = mechanics(x, fs, model)
        out["coil_temp_c"], out["magnet_temp_c"] = thermal(x, fs, model)
    return out


def driver_metrics(x, fs, model):
    if np is not None:
        return _driver_metrics_np(x, fs, model)
    n = pow2(len(x))
    out = {
        "band_rms_db": [db(p) for p in band_powers(rfft(x, n), n, fs)],
        "band_peak_db": [db(p) for p in stft_band_peak(x, fs)],
        "peak_db": db(max((v * v for v in x), default=0.0)),
        "win_rms_db": window_rms(x, fs),
    }
    if model is not None:
        out["excursion_m"], out["velocity_m_s"] = mechanics(x, fs, model)
        out["coil_temp_c"], out["magnet_temp_c"] = thermal(x, fs, model)
    return out


# --------------------------------------------------------------------- gate

def check_model(name, m):
    if not isinstance(m, dict):
        return f"driver {name}: no model"
    bad = [k for k in MODEL_KEYS if not isinstance(m.get(k), (int, float)) or not math.isfinite(m[k])]
    if bad:
        return f"driver {name}: model missing or non-finite {', '.join(bad)}"
    if min(m[k] for k in ("Reb_ref", "mt", "kt", "tau_Tvc", "tau_Tmg")) <= 0:
        return f"driver {name}: model has a non-positive Reb_ref/mt/kt/tau"
    return None


def read_f32(path, channels):
    raw = open(path, "rb").read()
    if len(raw) % (4 * channels):
        raise ValueError(f"{path}: size is not a whole number of {channels}-channel frames")
    if np is not None:
        a = np.frombuffer(raw, dtype="<f4").astype(np.float64)
        if not np.isfinite(a).all():
            raise ValueError(f"{path}: non-finite sample")
        return [a[c::channels] for c in range(channels)]
    flat = struct.unpack(f"<{len(raw) // 4}f", raw)
    if not all(math.isfinite(v) for v in flat):
        raise ValueError(f"{path}: non-finite sample")
    return [list(flat[c::channels]) for c in range(channels)]


def compare(mac, v2, tol):
    """List of (metric, macOS, v2, excess) where v2 is above macOS."""
    over = []
    if v2["peak_db"] < tol["floor_db"]:
        return over

    def lvl(name, a, b):
        if b is None or b == -math.inf or b < tol["floor_db"]:
            return
        if a is None or b > a + tol["db"]:
            over.append((name, a, b, b - (a if a is not None and a != -math.inf else tol["floor_db"])))

    for i, (c, _, _) in enumerate(BANDS):
        lvl(f"band_rms_db@{c:.0f}", mac["band_rms_db"][i], v2["band_rms_db"][i])
        lvl(f"band_peak_db@{c:.0f}", mac["band_peak_db"][i], v2["band_peak_db"][i])
    lvl("peak_db", mac["peak_db"], v2["peak_db"])
    for w, b in v2["win_rms_db"].items():
        lvl(f"win_rms_db@{w}s", mac["win_rms_db"].get(w), b)
    for k in ("excursion_m", "velocity_m_s"):
        if k in v2 and v2[k] > mac[k] * (1 + tol["rel"]) + 1e-15:
            over.append((k, mac[k], v2[k], v2[k] / mac[k] if mac[k] else math.inf))
    for k in ("coil_temp_c", "magnet_temp_c"):
        if k in v2 and v2[k] > mac[k] + tol["temp_c"]:
            over.append((k, mac[k], v2[k], v2[k] - mac[k]))
    return over


def gate(manifest_path, report_path=None, log=print):
    man = json.load(open(manifest_path))
    base = os.path.dirname(os.path.abspath(manifest_path))
    tol = dict(DEFAULT_TOL, **man.get("tolerance", {}))
    req = man.get("required", {})
    req_cases = req.get("cases") or default_cases()
    req_vols = req.get("volumes") or list(DEFAULT_VOLUMES)
    failures, report = [], {"tolerance": tol, "tunings": []}
    if not man.get("tunings"):
        failures.append("manifest names no tunings")
    for t in man.get("tunings", []):
        name, fs, drivers = t.get("name", "?"), t.get("fs"), t.get("drivers") or []
        tr = {"name": name, "cases": [], "worst": {}}
        report["tunings"].append(tr)
        if not isinstance(fs, int) or fs <= 0 or not drivers:
            failures.append(f"{name}: needs fs and drivers")
            continue
        models = t.get("models")
        if isinstance(models, str):
            models = json.load(open(os.path.join(base, models)))
        models = models or {}
        for d in drivers:
            err = check_model(d, models.get(d))
            if err:
                failures.append(f"{name}: {err}")
        have = {(c.get("case"), float(c.get("volume", -1))) for c in t.get("cases", [])}
        for c in req_cases:
            for v in req_vols:
                if (c, float(v)) not in have:
                    failures.append(f"{name}: missing case {c} at volume {v}")
        for c in t.get("cases", []):
            label = f"{name}/{c.get('case')}@{c.get('volume')}"
            try:
                mac = read_f32(os.path.join(base, c["macos"]), len(drivers))
                got = read_f32(os.path.join(base, c["v2"]), len(drivers))
            except (OSError, KeyError, ValueError) as e:
                failures.append(f"{label}: {e}")
                continue
            if len(mac[0]) == 0 or len(got[0]) == 0 or len(mac[0]) != len(got[0]):
                failures.append(f"{label}: empty render or the two sides differ in length")
                continue
            cr = {"case": c.get("case"), "volume": c.get("volume"), "over": {}}
            for d, a, b in zip(drivers, mac, got):
                model = models.get(d) if check_model(d, models.get(d)) is None else None
                over = compare(driver_metrics(a, fs, model), driver_metrics(b, fs, model), tol)
                if over:
                    cr["over"][d] = [list(o) for o in over]
                    worst = max(over, key=lambda o: o[3])
                    failures.append(f"{label} {d}: {len(over)} metric(s) above macOS, worst {worst[0]} "
                                    f"macOS {worst[1]:.6g} v2 {worst[2]:.6g}")
                    for o in over:
                        k = f"{d}:{o[0].split('@')[0]}"
                        if k not in tr["worst"] or o[3] > tr["worst"][k][3]:
                            tr["worst"][k] = [label] + list(o[1:])
            tr["cases"].append(cr)
    report["pass"] = not failures
    report["failures"] = failures
    if report_path:
        with open(report_path, "w") as f:
            json.dump(report, f, indent=1, default=str)
    for msg in failures[:200]:
        log("FAIL " + msg)
    if len(failures) > 200:
        log(f"... and {len(failures) - 200} more")
    log(("PASS" if not failures else f"FAIL ({len(failures)})") + f": {manifest_path}")
    return not failures


# ------------------------------------------------------------------ signals

def default_cases():
    cases = []
    for f in SINE_HZ:
        for lv in SINE_DBFS:
            cases.append(f"sine_{f:g}_{-lv}")
            if f <= PHASE_HZ_MAX:
                cases.append(f"sineanti_{f:g}_{-lv}")
    cases += [f"burst_{f:g}" for f in BURST_HZ]
    cases += [f"duo_{a:g}_{b:g}" for a in DUO_SUB_HZ for b in DUO_TONE_HZ]
    cases += ["square_100_1", "sweep_log_0", "pink_6", "pink_long_0"]
    return cases


def make_signal(case, fs, rng_seed=20261106):
    """Stereo program for a required case, as two lists."""
    kind, rest = case.split("_", 1)
    n2 = int(1.5 * fs)
    if kind in ("sine", "sineanti"):
        f, lv = rest.split("_")
        a = 10 ** (-float(lv) / 20)
        l = [a * math.sin(2 * math.pi * float(f) * i / fs) for i in range(n2)]
        return l, ([-v for v in l] if kind == "sineanti" else list(l))
    if kind == "duo":
        fa, fb = (float(v) for v in rest.split("_"))
        a, b = 10 ** (-1 / 20) / 1.5, 10 ** (-1 / 20) * 0.5 / 1.5
        l = [a * math.sin(2 * math.pi * fa * i / fs) + b * math.sin(2 * math.pi * fb * i / fs) for i in range(2 * fs)]
        return l, list(l)
    if kind == "burst":
        f = float(rest)
        on, period = int(0.1 * fs), int(0.5 * fs)
        l = [math.sin(2 * math.pi * f * i / fs) if i % period < on else 0.0 for i in range(3 * fs)]
        return l, list(l)
    if kind == "square":
        a = 10 ** (-1 / 20)
        l = [a if math.sin(2 * math.pi * 100 * i / fs) >= 0 else -a for i in range(n2)]
        return l, list(l)
    if kind == "sweep":
        n = 10 * fs
        k = math.log(20000 / 20)
        l = [math.sin(2 * math.pi * 20 * n / fs / k * (math.exp(k * i / n) - 1)) for i in range(n)]
        return l, list(l)
    if kind == "pink":
        n = (60 if rest.startswith("long") else 10) * fs
        lv = float(rest.split("_")[-1])
        if np is None:
            raise RuntimeError("pink cases need numpy")
        rng = np.random.default_rng(rng_seed)
        w = np.fft.rfft(rng.standard_normal(n))
        k = np.arange(len(w), dtype=float)
        k[0] = 1
        w /= np.sqrt(k)
        w[0] = 0
        x = np.fft.irfft(w, n)
        x *= 10 ** (-lv / 20) / np.max(np.abs(x))
        r = np.roll(x, fs // 7)
        return list(x), list(r)
    raise ValueError(f"unknown case {case}")


def write_signals(out, fs):
    os.makedirs(out, exist_ok=True)
    index = {}
    for c in default_cases():
        l, r = make_signal(c, fs)
        p = os.path.join(out, f"{c}.2ch.f32")
        with open(p, "wb") as f:
            for a, b in zip(l, r):
                f.write(struct.pack("<2f", a, b))
        index[c] = os.path.basename(p)
    with open(os.path.join(out, "cases.json"), "w") as f:
        json.dump({"fs": fs, "volumes": list(DEFAULT_VOLUMES), "cases": index}, f, indent=1)
    print(f"{len(index)} cases in {out}")


# ---------------------------------------------------------------- self-test

def _synthetic_model():
    # Synthetic driver, not any Mac's: round numbers of a small woofer.
    return {"ag": 10.0, "Reb_ref": 4.0, "Rshunt": 0.1, "Rampout": 0.1, "phi": 3.0, "mt": 0.003,
            "kt": 2000.0, "ct": 0.5, "T_sett_vc": 20.0, "tau_Tvc": 0.5, "T_sett_mg": 10.0,
            "tau_Tmg": 5.0, "Thermal_AmbientTemp": 25.0}


def _find(d, key):
    """The value of key anywhere under d (dicts and lists), or None."""
    if isinstance(d, dict):
        if key in d:
            return d[key]
        vals = d.values()
    elif isinstance(d, list):
        vals = d
    else:
        return None
    for v in vals:
        r = _find(v, key)
        if r is not None:
            return r
    return None


def sidecar_models(plist):
    """Per-unit-channel driver models from a parsed sidecar; raises on any gap."""
    chans = _find(plist, "ChannelSpecificParams")
    if not isinstance(chans, dict) or not chans:
        raise ValueError("sidecar has no ChannelSpecificParams")
    glob = {k: v for k, v in plist.items() if k != "ChannelSpecificParams"}
    out = {}
    for name in sorted(chans, key=lambda k: int(k[7:]) if k.startswith("Channel") and k[7:].isdigit() else -1):
        if not (name.startswith("Channel") and name[7:].isdigit()):
            raise ValueError(f"unexpected channel entry {name}")
        m = {}
        for k in MODEL_KEYS:
            v = _find(chans[name], k)
            if v is None:
                v = _find(glob, k)
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise ValueError(f"{name}: no numeric {k}")
            m[k] = float(v)
        out[f"ch{name[7:]}"] = m
    return out


def self_test():
    import tempfile
    fs = 4000
    ok = True

    def tone(f, a, n):
        return [a * math.sin(2 * math.pi * f * i / fs) for i in range(n)]

    def write(path, chans):
        with open(path, "wb") as fh:
            for fr in zip(*chans):
                fh.write(struct.pack(f"<{len(fr)}f", *fr))

    def run(tag, mac_chs, v2_chs, expect, models=None, cases=None, mutate=None):
        nonlocal ok
        with tempfile.TemporaryDirectory() as d:
            write(os.path.join(d, "m.f32"), mac_chs)
            write(os.path.join(d, "g.f32"), v2_chs)
            man = {"required": {"cases": cases or ["c"], "volumes": [1.0]},
                   "tunings": [{"name": "syn", "fs": fs, "drivers": ["w", "t"],
                                "models": models if models is not None else
                                {"w": _synthetic_model(), "t": _synthetic_model()},
                                "cases": [{"case": "c", "volume": 1.0, "macos": "m.f32", "v2": "g.f32"}]}]}
            if mutate:
                mutate(man, d)
            p = os.path.join(d, "man.json")
            json.dump(man, open(p, "w"))
            msgs = []
            got = gate(p, log=msgs.append)
            if got != expect:
                ok = False
                print(f"self-test {tag}: expected {'pass' if expect else 'fail'}\n  " + "\n  ".join(msgs))
            else:
                print(f"self-test {tag}: ok")

    n = 2048
    w, t = tone(80, 0.5, n), tone(1000, 0.3, n)
    run("identical", [w, t], [w, t], True)
    run("quieter", [w, t], [[v * 0.8 for v in w], [v * 0.9 for v in t]], True)
    run("louder by 0.1 dB", [w, t], [[v * 10 ** (0.1 / 20) for v in w], t], False)
    run("one driver louder", [w, t], [w, [v * 1.05 for v in t]], False)
    lf = [a + b for a, b in zip(tone(80, 0.45, n), tone(31, 0.05, n))]
    run("same peak, more low end", [w, t], [lf, t], False)
    run("energy where macOS has none", [w, [0.0] * n], [w, tone(40, 1e-4, n)], False)
    run("v2 below the floor", [w, [0.0] * n], [w, tone(40, 1e-9, n)], True)
    run("missing model", [w, t], [w, t], False, models={"w": _synthetic_model()})
    bad = dict(_synthetic_model())
    del bad["T_sett_vc"]
    run("model key missing", [w, t], [w, t], False, models={"w": _synthetic_model(), "t": bad})
    run("missing case", [w, t], [w, t], False, cases=["c", "d"])

    def nan(man, d):
        write(os.path.join(d, "g.f32"), [[math.nan] + w[1:], t])
    run("non-finite", [w, t], [w, t], False, mutate=nan)

    def short(man, d):
        with open(os.path.join(d, "g.f32"), "ab") as fh:
            fh.write(b"\0\0\0\0")
    run("torn file", [w, t], [w, t], False, mutate=short)
    hot = [v * (1.0 if i < n // 2 else 1.02) for i, v in enumerate(w)]
    run("hotter late", [w, t], [hot, t], False)

    syn = {"Thermal_AmbientTemp": 25.0, "ChannelSpecificParams": {
        "Channel1": {"OL_disp": {k: _synthetic_model()[k] for k in ("ag", "phi", "mt", "kt", "ct")},
                     "OL_thermal": {k: _synthetic_model()[k] for k in
                                    ("Reb_ref", "Rshunt", "Rampout", "T_sett_vc", "tau_Tvc", "T_sett_mg", "tau_Tmg")}},
        "Channel0": {"OL_disp": {}, "OL_thermal": {}}}}
    try:
        sidecar_models(syn)
        ok = False
        print("self-test models: accepted a channel with no fields")
    except ValueError:
        syn["ChannelSpecificParams"]["Channel0"] = syn["ChannelSpecificParams"]["Channel1"]
        got = sidecar_models(syn)
        if list(got) != ["ch0", "ch1"] or got["ch1"] != _synthetic_model():
            ok = False
            print(f"self-test models: wrong extraction {got}")
        else:
            print("self-test models: ok")
    m = _synthetic_model()
    x0, v0 = mechanics(tone(10, 1.0, 4 * fs), fs, m)
    static = m["ag"] * m["phi"] / ((m["Reb_ref"] + m["Rshunt"] + m["Rampout"]) * m["kt"])
    if not 0.9 < x0 / static < 1.15:
        ok = False
        print(f"self-test mechanics: below resonance x {x0:.4g} vs static {static:.4g}")
    else:
        print("self-test mechanics: ok")
    cases = default_cases()
    if len(cases) != len(set(cases)) or len(cases) < 60:
        ok = False
        print("self-test cases: duplicate or short required case set")
    print("self-test:", "PASS" if ok else "FAIL")
    return ok


def main(argv):
    if argv[1:] == ["--self-test"]:
        return 0 if self_test() else 1
    if len(argv) >= 3 and argv[1] == "gate":
        rep = argv[argv.index("--report") + 1] if "--report" in argv else None
        return 0 if gate(argv[2], rep) else 1
    if len(argv) == 4 and argv[1] == "models":
        import plistlib
        with open(argv[2], "rb") as f:
            models = sidecar_models(plistlib.load(f))
        with open(argv[3], "w") as f:
            json.dump(models, f, indent=1)
        print(f"{len(models)} driver models in {argv[3]}")
        return 0
    if len(argv) >= 3 and argv[1] == "signals":
        fs = int(argv[argv.index("--fs") + 1]) if "--fs" in argv else 48000
        write_signals(argv[2], fs)
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
