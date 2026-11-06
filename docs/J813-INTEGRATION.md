# J813 integration prerequisites

J813 uses the T8142 configuration of Linux's `t8152_aop_audio` driver.
Its speaker PCM carries four S24_LE samples in 32-bit containers, with
eight S16_LE voltage/current channels on the sense PCM. The ADMAC driver
reports burst-granularity residue; this is not the T6041 AOP firmware ring.
The transport description therefore uses non-batched sense and no additional
speaker headroom. The live self-check must still confirm the PCM format and
channel wiring before a graph can be used.

The transport entry is specific to `apple,t8142-aop-audio`. It does not
admit the `apple,t8152-aop-audio` fallback by itself or qualify J873.

## What is and is not qualified

The offline wiring derivation was checked against a retained J813 boot
device tree and that machine's own tuning bundle. It preserves the physical
order left woofer, right woofer, left tweeter, right tweeter, with a voltage
and current pair for each. The existing wiring, provisioning and session
checks also pass. These results establish transport recognition and wiring
derivation, not protected playback or PipeWire timing on hardware.

J813's native tuning selects LSM2 and requests mechanisms that the live
engine cannot yet represent. The compiler must continue refusing that
configuration; see [the compensation component](RESONATOR-COMPONENT.md) for
its independently checked scope. LSM2 also remains outside
`units_at_parity`, and J813 is not added to the qualified-board list.

The existing LSM2 graph takes program samples only. Detecting physical V/I
channels during the self-check does not establish that the protection engine
consumes them or implements closed-loop temperature estimation.

Linux [#922](https://github.com/aurora-silicon/linux/pull/922) supplies the
`Speaker Protection` ownership interface required by v2. A legacy
`Speaker Volume Unlock` kernel is not interchangeable with it. Kernel
ownership tests and a successful silent wiring check are necessary but do
not establish LSM2 parity or authorize an unqualified tuning.

## Reproduce the offline wiring check

Use the machine's own collected bundle and retained boot DTB. Keep the
outputs private: the dump contains native tuning.

```sh
cargo build --release -p aurora-speaker-tuning
target/release/aurora-speaker-tuning dump --json "$BUNDLE" > tuning.json
python3 -I packaging/session/wiring.py describe \
    --dtb "$DTB" --tuning-json tuning.json > wiring.json
sha256sum "$DTB" tuning.json wiring.json > wiring.sha256
```

The expected description has card `AppleJ813`, four program channels,
48 kHz, format `S24_LE`, and the `4ch` LSM2 plugin. Its zero sense-input
count describes the current engine contract, not the absence of physical
V/I feedback. Retain the input hashes, source commit, complete output and
compiler refusals when reporting a result.

## Attribution

Thomas Glanzmann (`sithglan`) authored the J813 Linux audio support.
Ryan Murray and the Aurora contributors authored the existing v2 wiring,
self-check and kernel ownership design. Acelogic performed the J813
integration checks and added the T8142 transport description. The tuning
reader retains its existing aurora-audio-dsp and saku attribution.
