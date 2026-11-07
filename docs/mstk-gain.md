# Multiband real gain synthesis

The real-waveform processing paths retain the interpolated detector gain `g`
and its 50 Hz bilinear low-pass companion `l`. They use the bounded weight

```
w = clamp(1 - (g / l)^2, 0, 1)
effective_gain = w * g + (1 - w) * l
```

The zero-denominator convention is the same as for the analytic path. At a
constant gain the two terms coincide. During recovery the weight clamps to
zero; reduction blends the direct and low-pass terms. The real path does not
advance the analytic gain-correction allpasses. Band bypass and mute retain
their direct gains, and saturation stays ahead of the real gain multiplier.

## Synthetic measurement rig

`tools/diffrig/mstk-gain.py` captures a stereo signal through the offline
Audio Unit host: one channel drives the linked detector, and the other
contains a quiet, continuous carrier. Only synthetic controls are supplied;
the generated strip is a stage-name stub and no saved tuning is loaded.
Every supplied control is checked against the AU's parameter inventory.
The default matrix covers four solo bands, algorithms 0 and 2, qualities
0 through 2, overshoot compensation off/on, and 44.1/48/96 kHz.

On a macOS reference host with `box_host` already built:

```
python3 tools/diffrig/mstk-gain.py capture --box-host /path/to/box_host --out /path/to/captures
```

On a build host with NumPy and `voicing-box-render`:

```
python3 tools/diffrig/mstk-gain.py compare --captures /path/to/captures \
  --renderer /path/to/voicing-box-render --out /path/to/comparison
```

The JSON summary reports 2/5/10/20/50 ms windows after attack, release,
a one-sample bump, and a short hold. It records both output error and gain
error normalized by each engine's neutral render. NPZ files retain each
engine's effective complex output gain and its validity mask. These are
output measurements; they do not uniquely identify internal detector nodes
or floating-point evaluation order. A successful render is not a parity
verdict.
