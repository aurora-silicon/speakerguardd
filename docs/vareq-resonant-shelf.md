# Resonant low shelves and type transitions

Filter type 9 is the resonant low shelf described by [Apple's AudioUnit EQ documentation](https://developer.apple.com/documentation/avfaudio/avaudiouniteqfiltertype/resonantlowshelf).
It uses the existing cookbook low-shelf equations with the octave-bandwidth
alpha already used by the peaking filter. Ordinary shelves retain their
fixed Butterworth damping.

Between enabled ordinary and resonant low shelves, interpolate the numerator
coefficients linearly. Interpolate each conjugate pole pair's radius and angle
linearly, then reconstruct the denominator. Interpolating the filter type,
its ordinary parameters, normalized denominator coefficients, or the two
outputs does not reproduce this transition. No measured coefficient is
embedded in the implementation.

Mixed transitions currently require nonzero endpoint gains and conjugate
poles at the compile rate. Other type changes and real-pole or zero-gain mixed
endpoints remain refusals. The compiler, artifact loader and chain constructor
check the carried domain. Type-9 arithmetic overflow and unstable endpoint
designs are refused. The existing serialized words are unchanged.

The isolated file renderer now passes explicit parameter-5 overrides into the
runtime volume input. It retains full-volume operation when no override is
provided.

Qualification: 688 cases on previously refused saved states, 78 synthetic
static controls, 144 synthetic/native volume points, and 140 broader controls
all pass the absolute 0.05 dB gate. The worst targeted error is 0.000003 dB.
The broader controls vary gain sign, corner, bandwidth and transition direction.
This is an amplitude qualification, not sample identity.
