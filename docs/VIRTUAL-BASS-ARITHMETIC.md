# VirtualBass filter arithmetic

This is the clean-room behavioural specification for the arithmetic and call
geometry of the VirtualBass (`vben`) filters. It records externally observable
facts needed for an independent implementation. It contains no vendor code,
binary-derived addresses, or machine tuning values.

## Coefficients and arithmetic

Every filter in the signal path uses the platform's single-precision
multichannel biquad API. Filter designs may use double-precision intermediate
math, but each normalized SOS is stored as five Float32 values before the
platform setup is made. Audio buffers, filter state, prepared impulse terms,
and the filter output are Float32. There is no double-precision filter object
in this box.

The public API has two observable execution paths:

- A setup with at least two channels runs fused transposed direct form II,
  section by section and frame by frame. Dividing the same signal into
  different calls does not change this path's arithmetic.
- A one-channel setup runs each section over the complete call in 8-frame
  groups, then a 4-frame group when possible, then individual frames. A call
  boundary restarts that grouping schedule, so the call length is part of the
  numerical result.

The independent implementation in `sgv2_core::vdsp` preserves the coefficient
rounding, Float32 fused operations, section order, state publication, and
8/4/1 call schedule.

## Filter objects

VirtualBass constructs and calls the following independent setups. “Half” is
one Butterworth half of a Linkwitz-Riley edge; duplicated halves remain
adjacent and are not interleaved.

| setup | ordered sections | setup channels | frames in one render call |
|---|---|---:|---:|
| full-rate high path | crossover high-pass half twice, upper-edge all-pass, lower-edge all-pass | stream channel count | render frames |
| full-rate mid low path | crossover low-pass half twice | 1 | render frames |
| selected-rate blend phase | upper-edge all-pass, lower-edge all-pass | 1 | selected frames |
| selected-rate harmonic first stage | lower high-pass half twice, first upper low-pass half | 1 | selected frames |
| selected-rate harmonic second stage | second upper low-pass half | 1 | selected frames |
| full-rate reconstruction | saved reconstruction SOS in their design order | 1 | render frames after zero insertion |

At a 48 kHz stream rate the selected branch runs at 8 kHz. Its factor-six
phase is carried across render calls. Starting from phase zero, consecutive
512-frame calls therefore select 86, 85, and 85 frames, then repeat. The
reconstruction filter still receives all 512 full-rate frames in each call.
A one-frame render gives the selected-rate setups one frame only when the
carried phase selects that frame; reconstruction always receives the one
full-rate frame.

## Routing

The full-rate high path filters every input channel independently. For stereo,
both `HP(M)` and `HP(S)` survive. The mono input is formed separately from the
channel mean, passed through the low and harmonic branches, reconstructed,
and added equally to each filtered output channel. Describing the high path as
side-only is therefore incomplete.

## Render-quantum dependence

The full-rate high path is quantum-independent for stereo because its setup
has two channels. Every mono setup is quantum-dependent in principle. The
full-rate mono crossover low pass is the material source of the observed
512-frame-versus-one-frame low-frequency harmonic difference; cancellation
when the filtered high and reconstructed mono paths are added makes its small
Float32 difference visible at the output. The selected-rate and reconstruction
objects retain their actual call lengths as specified above.

The selected-rate phase is not restarted at a render boundary. The observed
difference is filter grouping arithmetic, not a resampling-grid reset.

## Silence and repeatability

The filter path adds no dither, DC offset, random signal, or denormal guard.
Exact-zero renders remain exact Float32 zero, and identical input, state,
parameters, and render geometry produce identical output. Very low residual
lines are deterministic rounding and cancellation products.

## Verification boundary

Behaviour was established with synthetic calls to the public platform API and
offline black-box AudioUnit renders. Published tests use only synthetic
coefficients and signals. Per-machine parameters and private verification
traces are not part of this repository.
