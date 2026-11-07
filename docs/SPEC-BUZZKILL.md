# BuzzKill version-zero tracker

BuzzKill is a linked multi-line suppressor. It is not a bank of static equalizers. The ten saved Kill points shape a permitted line level; the moving sections are created from the analysed program.

At 48 kHz the analyser low-pass resamples the linked program to 12 kHz with a 97-tap, 4:1 low-pass stage. The resampler has a pass-band gain of two; this is part of the detector's amplitude domain and must not be normalized away. The moving rejection bank then filters that 12 kHz signal before it enters the retained 512-sample analysis history. Audio has a separate rejection bank at the host rate. The analyser makes one decision every 128 analysis frames, which advances 512 host frames at the supported 4:1 rate. Before the transform it applies the symmetric window

```text
w[n] = (1 - cos(2*pi*n/511)) / (2*511),  n = 0..511.
```

The enabled Kill points shape the threshold with ordinary parametric peaks. Their saved quality is an analog-decay convention: the digital peak uses `alpha = sin(w) * sinh(0.5 / quality)`. The real-input transform and its first-difference transform run in Float32 and use the doubled positive-frequency peak convention. With the direct complex sum `X[k]` used below, the compared line amplitude is `A[k] = 2 * abs(X[k])`; there is no RMS conversion. The transformed program uses integer cells, while the shaped threshold cell `T[k]` is sampled at `(k + 0.5) * fs_analysis / 512`. The transform keeps the fixed window normalization while the initial record fills, placing received samples in the tail of the zero-filled retained record. These partial windows produce the neighbouring onset candidates. Only the refined frequency is range-checked, strictly above 50 Hz and below the selected maximum (2.5, 3, 4, 6, or 20 kHz). For each bin whose transformed amplitude `A[k]` exceeds `T[k]`, it emits a candidate with multiplier

```text
m[k] = clamp(T[k] / A[k], 1e-6, 1).
```

Before track matching, a refined candidate is discarded when it lies inside a candidate already accepted by this decision: strictly above `0.9659363 * f_previous` and strictly below `1.035265 * f_previous`. This suppresses duplicate refined cells without changing the moving-track match rule below.

Candidate frequency is not the centre of the FFT bin. Let `X[k]` be the complex transform of the windowed signal and `D[k]` the transform of its windowed first difference. With `q = abs(D[k]) / (abs(X[k]) + 1e-40)`, the refined frequency is

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
g_release = g + (1 - exp(-128 / (0.050 * fs_host))) * (1 - g),
```

then a match applies `g = clamp(g_release * m, 1e-6, 1)`. A new line replaces the closest track whose gain is above `0.82` and starts at `m`. The tracked value `g` is the dry remainder of a fixed, unity-centre band-pass rejection component: each bank outputs `x - (1 - g) * bandpass(x)`. The band-pass quality is the constant `Q = 6`; it is not a constant 6 Hz bandwidth. Thus its bandwidth follows `f / 6`. A fully released track is bypassed and does not advance its filter state. Track gains, section state, and resampler accumulation use the unit's Float32 signal domain.

The saved look-ahead is an inclusive span (8 or 15 ms in the supported states). The transport ring retains one fewer prior frame than the rounded saved count, so at 48 kHz an 8 ms setting reports 383 retained frames and maps input frame 1 to output frame 384.

Rendering is decision-blocked: the analyser consumes up to the next 128-analysis-frame (512-host-frame) boundary, updates the tracks, and then applies those updated tracks to that same transported block. Applying the decision to the following block adds an incorrect attack delay.

Reset clears the delay, transformed-spectrum history, decision phase, tracks, and every section history. Host call boundaries do not reset or advance the decision phase specially.

The compiler reads the strip at runtime. It does not contain per-Mac thresholds, Kill points, or look-ahead values.
