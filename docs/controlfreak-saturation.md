# ControlFreak saturation path

This is the implementation contract for the optional output shaper in the
`cfrk` voicing box.  It contains equations and state rules only; tuning values
remain runtime inputs.

For each render piece of at most 512 base-rate frames, derive one linked drive
scalar `k` from the unmodified input block.  Process every channel in this
order:

```text
base-rate DF1 de-esser cut
  -> multiply by k
  -> 2x, 49-tap interpolation FIR (kernel 2h)
  -> memoryless sine or cubic shaper
  -> 2x, 49-tap decimation FIR (kernel h)
  -> multiply by 1/k
  -> base-rate DF1 inverse de-esser
```

The new `k` applies at base frame zero of the piece.  There is no within-piece
interpolation.  The interpolation delay retains samples scaled by the drive
active when they entered.  The reciprocal follows that delayed carrier in a
continuous 24-frame history, including across short host pieces.  Both FIR
delays and both independent DF1 states continue without scaling, exchange or
coefficient changes.  The FIR pair adds 24 base frames of latency.

The de-esser recurrence is single precision:

```text
y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
                   - a1*y[n-1] - a2*y[n-2]
```

For saved centre `f0`, quality `Q`, strength `S`, and base rate `fs`, set
`G = 10^(-S/20)`, `w = 2*pi*f0/fs`, `alpha = sin(w)/(2Q)`.  The cut has raw
numerator `[1+alpha, -2*cos(w), 1-alpha]` and denominator
`[1+alpha/G, -2*cos(w), 1-alpha/G]`; normalise by the denominator's first
coefficient.  The inverse swaps those raw arrays and owns separate state.
Apple documents its multichannel single-precision biquad as direct-form I:
<https://developer.apple.com/library/archive/documentation/Performance/Conceptual/vDSP_Programming_Guide/UsingBiquadraticFilterFunctions/UsingBiquadraticFilterFunctions.html>.

## FIR construction

Generate a symmetric, max-normalised, 100-point Dolph-Chebyshev window at
90 dB using the public analytic definition in SciPy's `chebwin`, then round it
to `f32`.  SciPy's implementation and license are:

- <https://github.com/scipy/scipy/blob/main/scipy/signal/windows/_windows.py>
- <https://github.com/scipy/scipy/blob/main/LICENSE.txt> (BSD-3-Clause)

Resample those 100 control points to 49 with clamped cubic B-spline weights.
For tap `j`, `u = f32(99*j/48)`, `r = u - floor(u)`, and

```text
w[j] = sum(C[clamp(floor(u)+m, 0, 99)] * B3(r-m), m=-1..2)

B3(x) = 2/3 - x^2 + |x|^3/2,  |x| < 1
      = (2-|x|)^3/6,           1 <= |x| < 2
      = 0,                     otherwise
```

For mode `m`, use `fc = min(19 kHz - m*1 kHz, fs/2)` below 88.2 kHz and
`fc = fs/4` at 88.2 kHz and above.  Clamp `r = 2*fc/fs` to `[0.5, 1]`, set
`a = r/2`, and for `j=0..48` form

```text
q[j] = f32(w[j] * sinc_pi(a*(j-24)))
scale = f32(1 / sum(double(q[j])))
h[j] = f32(q[j] * scale)
```

The interpolator uses `f32(2*h)` and the decimator uses `h`; their samples,
accumulators, memoryless shaper and persistent polyphase histories are
single-precision and are not reset at drive-block boundaries.  Both saturation
flavours share this path.
