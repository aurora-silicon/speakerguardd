# Saved XTC filter compatibility

An explicitly supplied `Setup_0/W` can store its 512-bin real/imaginary vectors
as raw-DEFLATE Float32 bytes or as numeric arrays. The compression flag and
storage must agree, every vector must have exactly 512 finite Float32 values,
and malformed flags or vectors are refused. No saved spectrum is embedded in
source or tests.

Saved-filter flags 33 and 49 accept boolean values in this fixed-setup path.
Both live parameter changes and changes before ClassInfo restoration preserve
the measured output with a supplied filter dictionary. This does not carry a
generated-filter mode or additional orientations.

The requested group delay is a position on the finite FIR's tap grid. Convert
the saved percentage to a Float32 fraction, multiply it by the tap count,
round to the nearest tap, and cap it at the integer center tap. Using the
untrimmed duration in samples changes the startup/restored filter for some
fractional lengths. The settled cached matrix and its construction defaults
retain their existing rules.

At the carried system volume of 0 dB, a saved upper taper endpoint below 0 dB
still selects its upper wet/dry value. Coincident or reversed taper points are
accepted only when both wet/dry values agree.

The three newly supported states pass all 546 isolated steady/transient cases,
with zero above an absolute 0.05 dB and worst 0.010494 dB. Synthetic tests cover
storage equivalence, reserved-flag rejection and tap-grid boundary controls.
The existing fixed-setup, rate, orientation, mute, swap and optimizer guards
remain in force.
