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
derivation. The subsequent physical silent check passed all nine phases
(baseline and eight individual sense-switch checks) with the existing
classification thresholds and S24_LE playback. This does not establish
protected playback or PipeWire timing.

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

## Keep sensing aligned during the silent check

Arm one continuous sense capture before starting the zero-sample speaker
stream, and keep it open across every switch step and playback format
fallback. On J813, reopening capture while the speaker serializer is
running can start at a different channel slot. The original per-step
capture check failed on hardware; the continuous capture passed without
reordering samples or relaxing the live/quiet thresholds.

The self-check bounds its capture buffer and refuses EOF, lost ownership,
lease expiry, overwritten windows or missing data. Cleanup stops both
owned streams, releases the lease and restores the sense switches. Every
restored switch is read back; a refused write or mismatched/unreadable value
fails qualification while cleanup still attempts the remaining switches. This
qualifies the capture-first lifecycle only. The kernel's late-capture
behavior remains unqualified; a graph must not infer that arbitrary
startup or restart ordering is safe from this check.

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
integration checks and added the T8142 transport description and continuous
capture lifecycle fix. The tuning
reader retains its existing aurora-audio-dsp and saku attribution.
