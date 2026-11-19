# Voicing rig

The acceptance test for the voicing chain (`sgv2_core::voicing::chain`):
the Mac's whole speaker chain, program to protection unit, rendered on
macOS through the Mac's own boxes and through v2's chain compiled from the
same tuning, compared per unit lane (driver) and per 1/3-octave band from
20 Hz to 20 kHz, at several program levels and system volumes. A band
where v2 is louder than macOS by more than the tolerance (0.1 dB) is
flagged; so is a tuning v2 refuses.

```
tuning ──► aurora-speaker-tuning voicing::topology::plan ──► one plan (boxes, per-channel sources, graph-param links)
               │                                                   │
               ├──► voicing::chain::compile ──► voicing-render ────┤  v2 lanes
               │                                                   │
               └──► strip + propstrip ──► box_host (per box) ──────┘  macOS lanes  ──► rig.py: per lane, per band, per volume
```

- `box_host.m`: renders one box of the graph through the installed Audio
  Unit: the strip's saved state (class info), the strip's bypass, the
  property strip's properties (a `Path` property is passed as the file's
  contents; the unit refuses a URL or a string), graph parameters with
  `--set`. Public AudioToolbox API only, offline, no audio device.
- `rig.py`: builds the macOS chain from the plan (builtins here: non-finite
  guard, gain/balance/mute), runs v2's `voicing-render`, compares.
  `--check-plan` checks the plan's fan-out against macOS's own boxes.
  `--law` maps a system volume scalar to the graph's `mvsc`/`mvol`; the
  default `linear-db:63.5` is a placeholder test-point generator, not any
  Mac's curve (the volume-law work replaces it with a per-Mac JSON).
- The chain's boxes still stubbed, and the boxes carried only in part
  (`gaps`: a filter box's volume automation, a state the module refuses),
  are listed per tuning; a Mac with either stays on the interim voicing.

Run (on a Mac):

```
cargo build --release -p aurora-speaker-tuning --example voicing-render
python3 tools/diffrig/voicing/rig.py --render target/release/examples/voicing-render \
    --out PRIVATE_DIR [--aids 13,35] [--bundle m2=PATH] [--volumes 1,0.5] [--check-plan]
```

Everything it writes from a Mac's tuning is Apple-derived: `--out` must be a
private directory, never this repository.
