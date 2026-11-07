# LSID numerical and handoff addendum

Version: 1.0, 2026-11-07

This addendum defines speakerguardd's clean-room LSID estimator law. It extends
the saved-state, graph, and safety contract in `LSID.md`. It contains no vendor
code, binary data, preset values, response arrays, or native payload bytes.

The law is independently specified and synthetically testable. It does not
claim sample-exact parity with an unavailable native LSID host. Consequently
the existing companion release gate remains mandatory until the offline
differential plan passes. Implementing this law cannot unmute a companion
bundle.

## 1. Input, alignment, and windows

The estimator consumes six voltage/current pairs. Pair `p` uses lanes `2p` and
`2p+1`, scaled by the selected bank's voltage and current full-scale values.
Lane 1 is delayed by exactly one input frame; its uninitialised first value is
discarded. A non-finite lane, wrong source identity, wrong lane count, or short
buffer invalidates the entire quantum and discards every partial window.

Pairs select banks `[0,1,0,1,2,2]`. Samples belonging to the same bank are
averaged before analysis. The desired V/I rate is implemented by a bounded
phase accumulator; a missing or out-of-range desired rate selects the stream
rate. Each bank owns one preallocated power-of-two window of 16 through 65,536
samples. Saved integral FFT values below 16 encode the base-2 exponent; values
of 16 or more encode the literal length. Other encodings are refused. No
allocation, lock, system call, or logging occurs while observing samples.

## 2. Spectrum, validity, and resistance

A complete window receives a Hann taper and an in-place radix-2 FFT. For bins
with finite non-zero voltage and current, complex cross energy is

```text
C = sum(V[k] * conjugate(I[k]))
coherence = |C|^2 / (sum(|V[k]|^2) * sum(|I[k]|^2))
```

The result is clamped to `[0,1]` and must meet the bank's Rdc coherence
threshold. The low-frequency resistance observation is the real
least-squares projection

```text
R = sum(real(V[k] * conjugate(I[k]))) / sum(|I[k]|^2)
```

over bins below the LR2 fit-band lower edge, or one sixteenth of the estimator
rate when that edge is absent. `R` must be finite and inside the saved Rdc
bounds.

For a window duration `dt` and smoothing constant `tau`, every smoother uses
`a = 1 - exp(-dt/tau)`; non-positive `tau` means an immediate update. The fast
lane moves directly by `a_fast`. The slow lane moves only when the fast/slow
percentage difference meets `PercentSlowLaneThreshold`, then uses `a_slow`.
When yoyo smoothing is disabled the slow lane updates every accepted window;
when enabled, the percentage threshold gates that update.

## 3. Electrical and mechanical fit

The fitted electrical impedance is

```text
Z(f) = Rdc + j*w*Le + (j*w*Le2 * RLe2)/(RLe2 + j*w*Le2)
       + sum_r Bl_r^2 / (Rms_r + j*(w*Mms_r - Kms_r/w))
```

where `w = 2*pi*f` and stored gram values are converted to kilograms. Disabled
LR2 or resonance branches retain their defaults. Enabled adaptive parameters
are fitted only inside their saved bounds. A bounded coordinate descent uses
the saved iteration count (clamped to 1 through 24), halves its multiplicative
step each round, and minimises normalised complex RMS impedance error over the
saved frequency band and stride. Per-branch adaptation flags control `Bl` and
`Mms`; `Rms` and `Kms` remain bounded fit coordinates for an enabled branch.
The saved smoothing time is applied after fitting. A finite positive saved
error threshold rejects a worse fit.

Creep/secondary mechanical branches, frequency/Q-bound mode, and percent-error
mode are not part of this law. If any is enabled, the whole window is rejected
and no partial model generation is published. Their saved values remain
preserved for a later qualified extension.

The accepted confidence is `coherence * clamp(1 - fit_error, 0, 1)`. The
calibration tuple is valid only when calibration temperature and resistance
lie inside their respective saved bounds and the temperature coefficient is
positive. Closed-loop temperature is then

```text
Tvc = Tcal + (Rdc/Rcal - 1)/alpha
```

and is published as valid only when thermal estimation is enabled.

## 4. Cadence, stale state, and measured power

A bank cannot accept two windows closer than its impedance-update period. A
first accepted window enters `TRACKING`; complete rejected windows remain
`ACQUIRING` and preserve the last accepted model. Total measured power is the
non-negative sum of finite `V*I` across physical pairs, averaged over input
frames. It is unavailable after invalid sensing and is never substituted for
LSM2's modeled protection power.

Each bank counts input frames since acceptance. After the saved hold time, its
private fast and slow estimates exponentially approach the static default with
the saved decay constant and the estimator enters `DECAYING`. A stale or
decaying value is not published to LSM2: invalid sensing therefore cannot
relax the last accepted or static protection model. Reset discards partial
windows, alignment history, measured power, and pending transactions and
reseeds all private state.

## 5. Property 6630 semantic payload and atomicity

The clean internal property-6630 payload is one immutable value containing:

- a strictly increasing 64-bit generation;
- all three complete bank models (fast/slow Rdc, LR2, three resonance models,
  calibration and thermal validity, temperature, and confidence);
- a three-bit accepted-bank mask; and
- optional finite non-negative total measured power.

This is a semantic representation, not a claim about Apple's private binary
serialization. The single-owner mailbox validates a complete generation,
stages it without changing the controller-visible value, and swaps the whole
value only at an LSM2 controller boundary. Reset discards a staged generation.
No individual coefficient setter exists, so LSM2 cannot observe mixed
generations.

## 6. Six-lane flow expansion

The two flow records are side records. Expansion is fixed by output lane and
role:

| LSM2 lane | Side | Role |
|---:|---:|---|
| 0 | 0 | woofer |
| 1 | 1 | woofer |
| 2 | 0 | woofer |
| 3 | 1 | woofer |
| 4 | 0 | tweeter |
| 5 | 1 | tweeter |

This mapping prevents the two records from being mistaken for channels 0 and
1. Response arrays and role-specific gains remain inside the property-6632
dictionary until their numerical application is independently qualified.

## 7. Safety and qualification boundary

The normal host path lacks an authenticated sense-source identity, so it
cannot accept a numerical window. Even an offline authenticated estimator only
produces a candidate: all LSID companion bundles retain a non-zero release
gate and the governing LSM2 unit produces fail-safe muted output. The solver
has no feature flag or parameter that removes this gate.

Exact native update timing, private property-6630 byte layout, response/gain
application from property 6632, and any newly enabled appended LSM2 feature
remain differential-host gates. Unknown enabled fields are refused; disabled
unknown fields are preserved bit-for-bit.
