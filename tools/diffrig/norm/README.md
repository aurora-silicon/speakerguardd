# Norm rig

The per-box differential test for the voicing chain's loudness normalizer
(`aufx/ldnm`, `sgv2_core::voicing::norm`): the Mac's own installed unit,
rendered offline on macOS with each tuning's saved state, against v2's box
compiled from the same tuning. See `rig.py` for the cases, metrics and gates.

```sh
# on a Mac (offline; no audio device is opened)
clang -O2 -fobjc-arc -framework AudioToolbox -framework Foundation -o norm_host norm_host.m
NORM_HOST=$PWD/norm_host NORM_MUSIC=program.wav python3 rig.py ref "$PRIVATE/norm-rig"

# anywhere with the references and the tunings
cargo build --release -p aurora-speaker-tuning --example norm-render
NORM_RENDER=target/release/examples/norm-render NORM_TUNINGS=... NORM_MUSIC=program.wav \
  python3 rig.py cmp "$PRIVATE/norm-rig" [13,9,35]
```

References are Apple-derived data: keep them in a private directory.
