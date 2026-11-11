# Single-resonator compensation component

This component implements the finite-damping, single-resonator transfer
observed through the native AudioUnit interface. For an LSID companion that
requests this supported single-resonator form, LSM2 applies it to the woofer
lanes before every limiter model and gain decision. Tweeter lanes are
unchanged. A host reset clears its history and failure latch. Adaptive model
updates and the other J813 model features remain outside this component's
scope, and the compiler and fleet gate continue to refuse or mute unsupported
protection configurations.

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

The equations and placement were derived independently from controlled,
offline AudioUnit observations. No Apple code, coefficients, tuning or
recordings are included.
The file-only `resonator-render` example and `verify-resonator.py` compare a
fresh Rust binary against privately supplied native captures. The verifier
checks five small-signal parameter/clamp cases, unchanged tweeter channels,
reset/block-size invariance and malformed-input refusal, and emits
a JSON artifact with binary/input hashes and numerical errors. This evidence
does not qualify a complete LSM2/LSID engine or a speaker for audible use.
