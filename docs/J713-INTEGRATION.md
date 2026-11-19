# J713 integration prerequisites

J713 (MacBook Air 13-inch, M4) has four SN012776 amps behind the T8132 AOP.
Its audio node is `apple,t8132-aop-audio` with `apple,t6041-aop-audio` as a
fallback, and Linux binds the T6040 speaker service driver
(`sound/soc/apple/aop_spkr.rs`), the one the six-amp MacBook Pros use. The
transport description is therefore the existing `aop-spkr` one: S24_LE
playback in 32-bit containers, S16_LE sense from the AOP ring, non-batched
sense and 256 frames of speaker headroom.

## PCM widths

The speaker service gives its PCMs one width on every Mac: six playback
channels and twelve sense channels. On the six-amp Macs that is one channel
and one current/voltage pair per amp. J713 has four amps on it, so both PCMs
are wider than the amps:

| PCM | Width | The amps' lanes |
|---|---|---|
| Speaker (`hw:AppleJ713,1`) | 6 channels | 0 to 3: left woofer, right woofer, left tweeter, right tweeter |
| Sense (`hw:AppleJ713,2`) | 12 channels | 0 to 7: current then voltage for each amp, in the same order |

Neither PCM opens at four or eight channels. The transport table carries the
two widths, `wiring.describe` reports them as `speaker_pcm` and `sense_pcm`,
and the graph's speaker sink and sense source open at those widths. The
protect stage keeps one port per amp and per sense lane, and the session
links them by name, so the extra channels stay unlinked and silent. The
self-check plays zeros and captures at the PCMs' widths and judges the amps'
lanes only.

The amps' lanes come first because the derivation already requires it: the
amps' TDM slots must be 0 to n-1 and their sense slots 0 to 2n-1. A transport
whose fixed width is smaller than the link's amps is refused.

## What is and is not qualified

The offline wiring derivation was checked against a J713's boot device tree
and that machine's own tuning bundle. The silent self-check then passed on
the hardware with the existing classification thresholds and S24_LE
playback: the baseline capture and the five coded sense-switch captures, at
six playback and twelve sense channels. These results establish transport
recognition, the PCM widths and the wiring derivation. They do not establish
protected playback or PipeWire timing.

J713's native tuning selects LSM2 with an LSID companion and requests
mechanisms the live engine cannot yet represent. The compiler must continue
refusing that configuration, LSM2 with that companion remains outside
`units_at_parity`, and J713 is not added to the qualified-board list. No
graph file is shipped for it.

v2 needs the kernel's `Speaker Protection` ownership interface (Linux
[#922](https://github.com/aurora-silicon/linux/pull/922)). A kernel with only
`Speaker Volume Unlock` is not interchangeable with it.

## Reproduce the offline wiring check

Use the machine's own collected bundle and its boot DTB. Keep the outputs
private: the dump contains native tuning.

```sh
cargo build --release -p aurora-speaker-tuning
target/release/aurora-speaker-tuning dump --json "$BUNDLE" > tuning.json
python3 -I packaging/session/wiring.py describe \
    --dtb "$DTB" --tuning-json tuning.json > wiring.json
sha256sum "$DTB" tuning.json wiring.json > wiring.sha256
```

The expected description has card `AppleJ713`, transport `aop-spkr`, four
amps, 48 kHz, format `S24_LE`, `speaker_pcm` 6 and `sense_pcm` 12.

## Attribution

Ryan Murray and the Aurora contributors authored the v2 wiring, self-check
and kernel ownership design. Erik Bourget performed the J713 integration
checks and added the PCM widths.
