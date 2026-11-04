#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Deterministic golden-vector input signals for the speakerguardd v2 differential rig.

Our own generated signals only (seeded). Each signal is written as interleaved float32 raw
(<name>.<ch>ch.f32) for the offline hosts, plus a manifest with sha256 and description.
Levels are digital dBFS at the protection unit's input; the offline route needs no speakers.
"""
import hashlib, json, os, sys
import numpy as np

SR = 48000
SEED = 20261104


def pink(n, rng):
    # Voss-McCartney-free: FFT-shaped 1/f noise, normalised to unit RMS
    w = rng.standard_normal(n)
    f = np.fft.rfft(w)
    k = np.arange(len(f)); k[0] = 1
    f /= np.sqrt(k)
    f[0] = 0
    x = np.fft.irfft(f, n)
    return x / np.sqrt(np.mean(x ** 2))


def db(x):
    return 10 ** (x / 20)


def fade(x, ms=5):
    n = int(SR * ms / 1000)
    w = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, n))
    x[:n] *= w; x[-n:] *= w[::-1]
    return x


def tone(f, sec, lvl_db):
    t = np.arange(int(SR * sec)) / SR
    return fade(np.sqrt(2) * db(lvl_db) * np.sin(2 * np.pi * f * t))  # lvl = RMS dBFS


def sweep(f0, f1, sec, lvl_db):
    t = np.arange(int(SR * sec)) / SR
    k = np.log(f1 / f0)
    ph = 2 * np.pi * f0 * sec / k * (np.exp(t / sec * k) - 1)
    return fade(np.sqrt(2) * db(lvl_db) * np.sin(ph))


def sil(sec):
    return np.zeros(int(SR * sec))


def build(rng):
    S = {}
    S["silence_10s"] = (sil(10), "digital silence, 10 s (idle state, pilot behaviour)")
    steps = [sil(2)]
    for l in (-40, -30, -20, -12, -6):
        steps += [fade(db(l) * pink(SR * 6, rng)), sil(2)]
    S["pink_steps"] = (np.concatenate(steps), "pink noise 6 s steps at -40/-30/-20/-12/-6 dBFS RMS, 2 s gaps")
    S["sweep_log"] = (np.concatenate([sil(1), sweep(20, 20000, 10, -12), sil(1)]),
                      "log sweep 20 Hz-20 kHz, 10 s, -12 dBFS RMS")
    bursts = [sil(1)]
    for f in (100, 1000, 4000):
        for l in (-20, -6, 0):
            n = int(SR * 0.2)
            bursts += [fade(np.sqrt(2) * db(l) * np.sin(2 * np.pi * f * np.arange(n) / SR)).clip(-1, 1), sil(0.8)]
    S["tone_bursts"] = (np.concatenate(bursts), "200 ms bursts at 100/1k/4k Hz, -20/-6/0 dBFS RMS (0 dBFS clipped to FS), 800 ms gaps")
    # thermal ramp: sustained tone, offline only (no speakers), long enough to reach limits
    S["thermal_ramp_1k"] = (np.concatenate([tone(1000, 120, -3), sil(60)]),
                            "1 kHz sine -3 dBFS RMS for 120 s then 60 s silence (offline thermal ramp and release)")
    S["thermal_ramp_200"] = (np.concatenate([tone(200, 120, -3), sil(60)]),
                             "200 Hz sine -3 dBFS RMS for 120 s then 60 s silence")
    S["impulse"] = (np.concatenate([[0.5], np.zeros(SR - 1)]), "single 0.5 impulse then 1 s silence (latency)")
    return S


def main(outdir, tokyo=None):
    os.makedirs(outdir, exist_ok=True)
    rng = np.random.default_rng(SEED)
    S = build(rng)
    if tokyo:
        import scipy.io.wavfile as wf
        r, x = wf.read(tokyo)
        assert r == SR, r
        x = x.astype(np.float64)
        if x.dtype.kind == "i" or np.abs(x).max() > 2: x /= 32768.0
        S["tokyo_30s"] = (x, f"external 30 s program clip {os.path.basename(tokyo)} (third-party audio: never commit)")
    man = {"sr": SR, "seed": SEED, "signals": {}}
    for name, (x, desc) in S.items():
        x = np.asarray(x, dtype=np.float32)
        stereo = x.ndim == 2
        for ch in (2, 6):
            if stereo:
                y = x[:, :2] if ch == 2 else x[:, [0, 1, 0, 1, 0, 1]]
            else:
                y = np.repeat(x[:, None], ch, axis=1)
            y = np.ascontiguousarray(y.astype(np.float32))
            fn = f"{name}.{ch}ch.f32"
            y.tofile(os.path.join(outdir, fn))
            man["signals"].setdefault(name, {"desc": desc, "frames": int(y.shape[0]), "files": {}})
            man["signals"][name]["files"][fn] = hashlib.sha256(y.tobytes()).hexdigest()
    json.dump(man, open(os.path.join(outdir, "manifest.json"), "w"), indent=1)
    print(json.dumps({k: v["frames"] for k, v in man["signals"].items()}))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
