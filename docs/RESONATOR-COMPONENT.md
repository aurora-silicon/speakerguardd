# Single-resonator compensation component

This component implements the finite-damping, single-resonator transfer
observed through the native AudioUnit interface. LSM2 uses it only when the
controller's `CompensateResonators` switch and the type-A acceleration
manager's `UseCompensation` switch are both enabled.

Given the runtime model's resistance R, force factor B, moving mass M,
mechanical damping D and stiffness K, let:

- `w0 = sqrt(K/M)` and `d0 = (D + B²/R)/M`;
- `w1 = max(2π desired_hz, w0 × 10^(-max_boost_db/40))`;
- `d1 = min(w1/desired_q, sqrt((d0 × 10^(max_cut_db/20))²
  - ((w1²-w0²)/w0)²))`.

The transfer is `(s²+d0 s+w0²)/(s²+d1 s+w1²)`, transformed with the plain
bilinear substitution `s = 2 rate (1-z^-1)/(1+z^-1)`, without prewarping.
The first clamp bounds DC boost; the second bounds attenuation at the source
resonance. Nonpositive or nonfinite physical quantities, negative limits,
resonances outside Nyquist, a nonpositive radicand, or a nonfinite/unstable
discrete filter are refused. There is no fallback to an uncompensated signal.
Processing has no allocation and retains history across blocks. Nonfinite
input/output clears the current output block and latches a fault until reset.

## LSM2 integration

The compensator is the first internal LSM2 program stage. It runs at the
render sample rate on each channel that selects speaker type A, before the
analysis bank, sample/displacement/velocity allocation, power and temperature
models, gain followers, clipping and output monitor. Consequently the complete
nonlinear controller sees and limits the compensated program. It is not a
post-limiter EQ and adds no separate declared latency. Type-B and type-C
channels remain unchanged.

The source model is the acceleration manager's live `Rdc`, `Bl`, `Mms`, `Rms`
and `Kms` fields. These reads are direct: the manager's generic selectors do
not redirect compensation to the impedance/family row. The desired frequency,
quality, maximum DC boost and maximum resonance cut come from that same
type-A acceleration block. The shipped path uses the first resonator only; an
enabled second resonator or another optional acceleration branch is refused
rather than dropped.

Saved model fields are sufficient to construct the filter. LSID sensing is
not required for compensation to operate. When an authenticated LSID model
generation is accepted, LSM2 replaces the five source-model fields atomically
at a controller boundary and recomputes the coefficients. Input/output filter
history is retained across that coefficient change. `AudioUnitReset` clears
the filter history and failure latch; it does not discard the accepted model.
An invalid model/update faults the unit instead of bypassing compensation.

The equations were derived independently from controlled AudioUnit impulse
observations. No Apple code, coefficients, tuning or recordings are included.
The file-only `resonator-render` example and `verify-resonator.py` compare a
fresh Rust binary against privately supplied native captures. The verifier
checks five small-signal parameter/clamp cases, unchanged uncompensated
channels, reset/block-size invariance and malformed-input refusal, and emits
a JSON artifact with binary/input hashes and numerical errors. The integration
rig additionally disables only native compensation, places this clean-room
filter on either side of that native nonlinear controller, and requires the
before-controller result to pass the absolute 0.05 dB gate for both the quiet
and limiting programs. Other LSM2/LSID release gates remain independent.
