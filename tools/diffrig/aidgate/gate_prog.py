#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Deterministic stereo probe program for the per-AID full-chain gate.

Writes PROG.f32 (interleaved stereo float32, 48 kHz) and PROG.json (segments).
No tuning data. Segments: stepped sines at two levels, pink at two levels,
level steps and bursts, single-channel pink and decorrelated stereo pink.
"""
import json, sys
import numpy as np

FS = 48000
def main():
    out = sys.argv[1]
    rng = np.random.default_rng(20261107)
    def pink(n):
        s = np.fft.rfft(rng.standard_normal(n)); k = np.arange(len(s), dtype=float); k[0] = 1
        p = np.fft.irfft(s / np.sqrt(k), n); return p / np.max(np.abs(p))
    segs, parts, at = [], [], 0
    def add(name, x, kind):
        nonlocal at
        x = np.asarray(x, dtype=np.float64)
        if x.ndim == 1: x = np.stack([x, x], axis=1)
        segs.append({"name": name, "start": at, "frames": len(x), "kind": kind}); parts.append(x); at += len(x)
    add("silence_head", np.zeros(2 * FS), "silence")  # VolumeTaper settles
    t = np.arange(int(0.35 * FS)) / FS
    for f in [20, 31.5, 50, 63, 80, 100, 125, 160, 250, 400, 630, 1000, 1600, 2500, 4000, 6300, 8000, 10000, 12500, 16000, 20000]:
        add(f"tone_{f:g}_-12", np.sin(2 * np.pi * f * t) * 10 ** (-12 / 20), "steady")
    for f in [40, 100, 250, 630, 1600, 4000, 10000, 16000]:
        add(f"tone_{f:g}_0", np.sin(2 * np.pi * f * t), "steady")
    n = int(1.5 * FS)
    add("pink_-6", pink(n) * 10 ** (-6 / 20), "steady")
    add("pink_-30", pink(n) * 10 ** (-30 / 20), "steady")
    tt = np.arange(n) / FS
    add("step_up_125", np.sin(2 * np.pi * 125 * tt) * np.where(tt < 0.5, 0.01, 1.0), "dynamic")
    add("burst_1000", np.sin(2 * np.pi * 1000 * tt) * np.where((np.arange(n) // 480) % 10 == 0, 1.0, 0.0), "dynamic")
    add("step_down_2000", np.sin(2 * np.pi * 2000 * tt) * np.where(tt < 0.5, 1.0, 0.01), "dynamic")
    m = FS; p = pink(m) * 10 ** (-12 / 20)
    add("left_only_pink_-12", np.stack([p, np.zeros(m)], axis=1), "steady")
    add("right_only_pink_-12", np.stack([np.zeros(m), p], axis=1), "steady")
    add("stereo_pink_-12", np.stack([pink(n), pink(n)], axis=1) * 10 ** (-12 / 20), "steady")
    add("silence_tail", np.zeros(FS), "silence")
    x = np.concatenate(parts).astype("<f4")
    x.tofile(out + ".f32")
    json.dump({"fs": FS, "channels": 2, "frames": at, "segments": segs}, open(out + ".json", "w"), indent=1)
    print(out, at / FS, "s", len(segs), "segments")

if __name__ == "__main__":
    main()
