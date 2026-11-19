# VirtualBass mono and selected-rate law

This document specifies the clean-room `vben` path carried by `sgv2-core`.
It contains algorithm behaviour only. Device tuning is supplied at runtime;
no device values or implementation code from another system are included.
The provenance and verification rules in [PROVENANCE.md](PROVENANCE.md)
apply.

## Width and full-rate split

The box accepts one or two channels. Let `M` be the arithmetic mean of the
input channels after input gain. For mono, `M` is the sole input; therefore a
mono render and an equal-channel stereo render share exactly the same common
branch.

At crossover `fc`, the direct path is the Linkwitz–Riley high-pass plus the
phase companions for both band edges. The common branch starts with the
matching Linkwitz–Riley low-pass. Real saved orders are rounded to the nearest
integer (halves upward) and then raised to even. Orders 2 through 20 are
carried. A maximum-order direct cascade needs all ten Linkwitz–Riley sections
and both five-section phase companions; no companions may be truncated.

The saved low-end blend is limited to the public 0 dB maximum before conversion
to linear amplitude. This applies uniformly to mono and stereo states.

## Selected-rate branch

The selected rate is 8 kHz, requiring the stream rate to be an integer
multiple. The stream-rate crossover low-pass feeds phase-0 decimation
directly; there is no additional pre-decimation elliptic stage. The band-edge
all-pass and Linkwitz–Riley sections, level control, nonlinear device and
harmonic compressor run at the selected rate. Phase-0 zero insertion and the
seventh-order elliptic reconstruction low-pass return the common branch to the
stream rate before it is added to every delayed direct channel. A matched
pre-decimation elliptic trial was rejected because it changed steady LF gain
by 0.24–0.79 dB across the stereo states.

With level control enabled and an odd lower band-edge half-order, that edge's
one selected-rate frame of delay sits before the nonlinear device. The device
uses the previous `lo` sample and its previous input-follower value; output
normalization restores with the current input follower and current band-output
follower. The blend branch is held by the same frame, and the corresponding
post-branch delay is removed, so reported and total latency do not change.

## Level-control followers

Input and band-output followers have the same lifecycle and 0.9 restart
threshold. Their retained envelope and smoothing stores initialize to `1e-3`
on construction and explicit reset. Both state updates use Float32 fused
multiply-add association:

```text
e = fma(a, e, (1-a) * abs(x))
s = fma(b, s, (1-b) * e)
```

The attack/release/hold choice precedes these updates. Normalization applies
the published floor and clip only at the input and output divisions; the
floor is not the follower seed. Follower coefficients and retained follower
state are Float32. The filter cascades retain the common stable implementation
used by the qualified stereo path.

There is no AID-specific branch, fitted gain, fitted timing offset or
protective hold margin in this law.

## Acceptance

Synthetic tests require mono output to equal both channels of an equal-stereo
render through the complete order-20 path. Production qualification requires
the installed-unit box rig to have no mono steady or transient window over
0.05 dB, the `LAW-vben-mono-width` refusal to disappear from the all-AID
sweep, and ship-AID blobs to remain byte-identical.
