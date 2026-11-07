# BuzzKill version-zero tracker

BuzzKill is a linked multi-line suppressor. It is not a bank of static equalizers. The ten saved Kill points shape a permitted line level; the moving sections are created from the analysed program.

At 48 kHz the analyser low-pass resamples the linked, already-suppressed program to 12 kHz with a 97-tap, 4:1 low-pass stage. The resampler has a pass-band gain of two; this is part of the detector's amplitude domain and must not be normalized away. It retains 512 analysis samples and makes one decision every 128 host frames. Before the transform it applies the symmetric window

```text
w[n] = (1 - cos(2*pi*n/511)) / (2*511),  n = 0..511.
```

The real-input transform uses its doubled positive-frequency convention. With the direct complex sum `X[k]` used below, the line amplitude is therefore `A[k] = sqrt(2) * abs(X[k])` (the doubled transform magnitude divided by `sqrt(2)` for RMS). The transform keeps the fixed window normalization while the initial record fills; the partial window produces the neighbouring onset candidates. It evaluates bins from 50 Hz through the selected maximum (2.5, 3, 4, 6, or 20 kHz). For each bin whose transformed amplitude `A[k]` exceeds the shaped threshold `T[k]`, it emits a candidate with multiplier

```text
m[k] = clamp(T[k] / A[k], 1e-6, 1).
```

Candidate frequency is not the centre of the FFT bin. Let `X[k]` be the complex transform of the windowed signal and `D[k]` the transform of its windowed first difference. With `q = abs(D[k]) / (abs(X[k]) + 1e-20)`, the refined frequency is

```text
f[k] = (fs_analysis / (2*pi)) * q^2 / sqrt(2 * (1 - cos(q))).
```

Neighbouring over-threshold bins are independent candidates. This is why one line can install several sections at onset; as the 512-sample record fills and the sections feed back into the detector, only the stable line remains.

There are 50 tracks. A candidate matches the first unused track satisfying

```text
abs(f_candidate - f_track) / f_candidate < 0.03526492.
```

Every decision first releases each track towards unity,

```text
g_release = g + (1 - exp(-128 / (0.050 * fs))) * (1 - g),
```

then a match applies `g = clamp(g_release * m, 1e-6, 1)`. A new line replaces the closest track whose gain is above `0.82` and starts at `m`. Each track drives one narrow peaking cut with centre `f`, centre gain `g`, and 6 Hz bandwidth. The same sections filter the detector input and the look-ahead audio. The saved look-ahead is an exact transport delay (8 or 15 ms in the supported states).

Reset clears the delay, transformed-spectrum history, decision phase, tracks, and every section history. Host call boundaries do not reset or advance the decision phase specially.

The compiler reads the strip at runtime. It does not contain per-Mac thresholds, Kill points, or look-ahead values.
