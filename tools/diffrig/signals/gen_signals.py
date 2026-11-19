#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Deterministic golden-vector input signals for the speakerguardd v2 differential rig.

Our own generated signals only (seeded). Each signal is written as interleaved float32 raw
(<name>.<ch>ch.f32) for the offline hosts, plus a manifest with sha256 and description.
Levels are digital dBFS at the protection unit's input; the offline route needs no speakers.
Layouts: 2, 4 and 6 channels; a stereo source maps L R L R ... (the 4-channel layout is the
duplicated-stereo amplifier order of the four-speaker atsp machines).

  gen_signals.py OUT [program.wav]   all signals
  gen_signals.py OUT --edge          only the edge-case signals, merged into the manifest
  gen_signals.py OUT --derive=4      add a channel layout to every signal from its 2-ch file
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


def build_edge():
    """Closed-loop edge, pilot-gate and series-resistance cases (no random numbers)."""
    S = {}
    S["step_1k"] = (np.concatenate([tone(1000, 20, -6), sil(20)]),
                    "1 kHz sine -6 dBFS RMS for 20 s then 20 s silence (step and release)")
    S["tone_1k_m20_90s"] = (tone(1000, 90, -20),
                            "1 kHz sine -20 dBFS RMS for 90 s (steady moderate drive under sense steps)")
    # 1 kHz staircase around a 0.03 level gate, 2 s per level, continuous phase, up then down.
    # Levels are peak amplitudes: 0.03 x the factors, then the same factors x sqrt(2)
    # (the gate read as RMS).
    f = [0.5, 0.9, 0.99, 0.999, 1.0, 1.001, 1.01, 1.1, 2.0]
    lv = [0.03 * k for k in f] + [0.03 * np.sqrt(2) * k for k in f]
    lv = lv + lv[::-1]
    n = 2 * SR
    t = np.arange(n * len(lv)) / SR
    env = np.repeat(np.asarray(lv), n)
    S["pilot_stairs"] = (fade(env * np.sin(2 * np.pi * 1000 * t)),
                         "1 kHz staircase, 2 s per peak level 0.03 x {0.5..2} then x sqrt(2) x {0.5..2}, up then down")
    # level steps with continuous phase: hot (-3 dBFS), then moderate (-20 dBFS) so a
    # protected coil cools through 90 and 80 C with program present, then hot again
    segs = [(10, -3), (30, -20), (5, -3), (15, -20)]
    t = np.arange(int(SR * sum(d for d, _ in segs))) / SR
    env = np.concatenate([np.full(int(SR * d), np.sqrt(2) * db(l)) for d, l in segs])
    # the thermal ramp with a -40 dBFS probe instead of silence, so an audio-only unit's
    # release is observable in the output
    t = np.arange(SR * 180) / SR
    env = np.where(t < 120, np.sqrt(2) * db(-3), np.sqrt(2) * db(-40))
    S["thermal_ramp_1k_probe"] = (fade(env * np.sin(2 * np.pi * 1000 * t)),
                                  "1 kHz sine -3 dBFS RMS for 120 s then -40 dBFS RMS probe for 60 s (release visible)")
    S["pilot_hyst"] = (fade(env * np.sin(2 * np.pi * 1000 * t)),
                       "1 kHz, continuous phase: 10 s -3, 30 s -20, 5 s -3, 15 s -20 dBFS RMS")
    return S


def channel_map(x, ch):
    """Interleaved ch-channel copy: mono repeats; stereo maps L/R alternately (L R L R ...)."""
    if x.ndim == 2:
        return x[:, [i % 2 for i in range(ch)]]
    return np.repeat(x[:, None], ch, axis=1)


def write(outdir, S, chans=(2, 4, 6)):
    mp = os.path.join(outdir, "manifest.json")
    man = json.load(open(mp)) if os.path.exists(mp) else {"sr": SR, "seed": SEED, "signals": {}}
    for name, (x, desc) in S.items():
        x = np.asarray(x, dtype=np.float32)
        ent = man["signals"].setdefault(name, {"desc": desc, "frames": int(x.shape[0]), "files": {}})
        ent["desc"] = desc
        for ch in chans:
            y = np.ascontiguousarray(channel_map(x, ch).astype(np.float32))
            fn = f"{name}.{ch}ch.f32"
            y.tofile(os.path.join(outdir, fn))
            ent["files"][fn] = hashlib.sha256(y.tobytes()).hexdigest()
    json.dump(man, open(mp, "w"), indent=1)


def derive(outdir, ch):
    """Write <name>.<ch>ch.f32 for every signal from its 2-channel file. Exact: every layout
    is a channel map of the 2-channel data."""
    mp = os.path.join(outdir, "manifest.json")
    man = json.load(open(mp))
    for name, ent in man["signals"].items():
        x2 = np.fromfile(os.path.join(outdir, f"{name}.2ch.f32"), np.float32).reshape(-1, 2)
        y = np.ascontiguousarray(x2[:, [i % 2 for i in range(ch)]])
        fn = f"{name}.{ch}ch.f32"
        y.tofile(os.path.join(outdir, fn))
        ent["files"][fn] = hashlib.sha256(y.tobytes()).hexdigest()
    json.dump(man, open(mp, "w"), indent=1)


def main(outdir, tokyo=None):
    os.makedirs(outdir, exist_ok=True)
    if tokyo == "--edge":  # the edge-case signals only, merged into the manifest
        write(outdir, build_edge())
        return
    if tokyo and tokyo.startswith("--derive="):  # e.g. --derive=4
        derive(outdir, int(tokyo.split("=", 1)[1]))
        return
    rng = np.random.default_rng(SEED)
    S = build(rng)
    S.update(build_edge())
    if tokyo:
        import scipy.io.wavfile as wf
        r, x = wf.read(tokyo)
        assert r == SR, r
        x = x.astype(np.float64)
        if x.dtype.kind == "i" or np.abs(x).max() > 2: x /= 32768.0
        S["tokyo_30s"] = (x, f"external 30 s program clip {os.path.basename(tokyo)} (third-party audio: never commit)")
    mp = os.path.join(outdir, "manifest.json")
    if os.path.exists(mp):
        os.unlink(mp)
    write(outdir, S)
    print(json.dumps({k: len(v[0]) for k, v in S.items()}))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
