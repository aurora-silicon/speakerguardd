# Static low-pass phase compensation

A type-3 ToneMeister slot with a nonzero phase mask changes both its filtered
channels and its companion channels. For a carried even order `N`, filtered
channels run two identical Butterworth low-pass cascades of order `N/2`.
Companion channels run the corresponding all-pass sections: each denominator
is paired with its reversed numerator. An odd half-order adds one frame of
delay to both paths. The filtered path is 6.0206 dB down at its corner; the
companion has unit magnitude and the same phase.

Channel masks and bypassed groups retain their normal meaning. This carries
static even orders from 2 through 20. Odd orders and moving phase compensation
remain explicit refusals. Other filter types retain their existing rules.
The sections use the existing artifact format and allocation-free runner.

Offline installed-unit qualification covers 192 full four-channel cases,
including isolated channels, steady tones, pink noise, steps and bursts:
zero above an absolute 0.05 dB RMS difference, worst 0.002239 dB. Independent
synthetic order/mask checks verify the magnitude and phase relationships.
This amplitude gate does not assert sample identity.
