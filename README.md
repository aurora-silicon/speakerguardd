# speakerguardd v2

Speaker protection for Apple Silicon Macs that works the way macOS's does: the same protection unit at the same place in the speaker graph, with the amplifier's voltage and current sense as its second input where the Mac has it, driven by that Mac's own tuning and factory calibration.

- **spp3** for the V/I MacBooks (14" and 16" MacBook Pro, MacBook Air M2/M3, 15" Air M4/M5);
- **lsm2** for the MacBook Neo and the 13" MacBook Air M4/M5;
- **atsp** for the 13" MacBook Pro M1/M2 and the MacBook Air M1, later.

The acceptance bar is 1:1 parity: on identical input, the unit's internal state (power, coil and magnet temperatures, closed-loop correction, every constraint and the final gain) matches macOS's own, quantum by quantum, including the failure cases.

**Status:** under development on this branch. Not for use. The v0.1 daemon is archived on `legacy/v0.1` (tags `v0.1.12`, `v0.1.13-rc.2`) and stays deployed on each machine until v2 qualifies there.

## Layout

| Path | What |
|---|---|
| `crates/aurora-speaker-tuning` | Reader and validator for the Mac's tuning bundle that the Aurora installer stores with the vendor firmware |
| `crates/sgv2-core` | The protection framework: one `ProtectionUnit` trait for spp3, lsm2 and atsp; real-time safe |
| `crates/sgv2-lv2` | LV2 host for the core, loaded by the PipeWire filter-chain |
| `crates/sgv2-supervisor` | `speakerguardd`: path integrity and the kernel interlock |
| `crates/sgv2-render` | Offline renderer for the differential rig |
| `tools/diffrig` | Differential rig against macOS: trace schema, comparator, macOS reference host |
| `packaging` | systemd, SELinux, PipeWire and WirePlumber |
| `docs` | [Design](docs/DESIGN.md), [conventions](docs/CONVENTIONS.md), [provenance](docs/PROVENANCE.md) |

## Build

```
cargo test --workspace
cargo build --release --target aarch64-unknown-linux-musl   # static, for the Macs
```

## Credits

The tuning reader and the installer's collection of the tuning bundle are based on aurora-audio-dsp by saku (MIT). See `crates/aurora-speaker-tuning/LICENSE-aurora-audio-dsp`.

## License

MIT. See [LICENSE](LICENSE).
