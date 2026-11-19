# Per-AID full-chain parity gate

Gives every acoustic ID (AID) a pass/fail number against the installed macOS Audio Units, not only
"compiles". It runs from each AID's own tuning folder, with no per-board code, so an AID nobody has
plugged in is gated exactly like the test Macs.

Two sides, one program:

- `gate_prog.py OUT`: deterministic stereo program (24.15 s, 39 segments): 2 s settle, stepped sines at
  -12 and 0 dBFS, pink at -6 and -30 dBFS, a level step up, a burst and a step down, left-only and
  right-only pink, decorrelated stereo pink. No tuning data.
- macOS side, on a Mac under a host-heavy slot: `host_refs.sh SLOT AID...` runs `gr.py` for each AID.
  `gr.py` renders that AID's whole `builtin_speaker_out` graph up to the protection unit's input through
  the installed units (`box_host`, one process per box, nice 15). It handles `[if]` blocks (no bring-up),
  `parameter_add`, box-to-box `wireParam` (the source box's settled value on 2 s of silence),
  reblockers and format converters as wires, `delay`, and unfed telemetry and SysID paths. It keeps each
  AU box's output for attribution, copies the results to the buildbox, and deletes the local audio.
  numpy is not needed.
- v2 side, on the buildbox: `gate.py` renders the same program through `voicing-render`, compiled from
  the same tuning at full volume, and taps every AU box that v2's plan also has.

The gate compares each segment on each unit lane: the RMS of the second half, plus 10 ms windows on
dynamic segments where either side is above -80 dBFS. A lane more than 60 dB under the segment's loudest
lane on both sides (a crossover stop band) is numerical residue and is not measured. Any value outside 0.05 dB in either direction
fails. Floor residue (below) is reported separately and does not count over the limit. Both sides are compared at their original sample positions with zero latency compensation.
Norm meters each render call before applying gain to that same call; only its intrinsic look-ahead
remains. The raw cross-correlation lag is reported for diagnosis and never shifts the comparison. An AID passes only if its chain compiles with no stubs, the lane counts match and no
measurement is over the limit. Each AU box's cumulative output is compared the same way, and the first
box over the limit is named.

`summarize.py GATE_OUT SWEEP_JSON...` joins the gate results with the all-AID compile sweep
(`aidsweep.sh`: refusals, stubs, bounds, TODO(SPEC)) into one table per AID.

Limits: the reference is the installed host build's units (25G83). States from the 27.2 tunings that
run through those units are compatibility measurements, not native 27.2 qualification. The gate checks
amplitude only, not sample identity. It runs at 48 kHz and full volume; `attest.py` also gates every tuning
at lower volume sliders ("Volume sliders" below). The protection unit is
outside this gate; it has its own rigs. Every output made from Apple tuning is Apple-derived: keep it
in private directories and publish numbers only.

## macOS parity attestations (`attest.py`): what `speakerguardd-selfcheck verify` reads

The gate above answers "how close is v2 to macOS" per AID. `attest.py` turns the same measurement
into the file a Mac checks before v2 may take its speakers (`packaging/session/parity.json`, read by
`aurora-speaker-tuning voicing-check`, `crates/aurora-speaker-tuning/src/voicing/parity.rs`).

- It covers every tuning the release can select: every AID of the Tunings folder and every speaker
  configuration (speaker ID) of its DSP database, deduplicated by tuning key (a SHA-256 of what the
  voicing compiler reads: chain, saved states, unit dictionaries, plan, volume range; no per-unit or
  per-collection data). It never lists boards and is never edited by hand: it records whether this
  code renders each tuning like macOS.
- The macOS side applies the configuration's strip and property strip (`gr.py --strip --propstrip`,
  `box_host --propstrip`), as macOS does.
- Verdict per tuning: refused when the reference failed, v2 does not compile it, the compiler reports
  a stub, an unwired parameter or a gap, the lanes differ, any sample is non-finite, or any measured
  cell is louder than macOS by more than 0.25 dB or quieter by more than 1.0 dB. Floor residue
  (below) is reported in the entry (`floor_residue_cells`, `worst_floor_residue_db`) and never refuses.
- Each file is bound to `law`, a hash of the rendering code (`crates/aurora-speaker-tuning/build.rs`).
  A build whose law differs refuses every Mac until the file is regenerated: any change to the voicing
  compiler or engine needs a new run before it can ship.
- The file holds keys, verdicts, reasons and summary numbers only. Every render and the per-cell rows
  (`--details`) stay in private, tainted directories.

- The file holds one block per macOS release (`releases[].build`, format 2). `verify` takes the
  block of the tuning's law build (`Tuning::law_build`, `crates/aurora-speaker-tuning/src/reference.rs`):
  - v2 implements the laws of the reference release, 25G83. A tuning whose speaker files for its
    AID (speaker strips and property strips, protection sidecars, speaker graphs,
    `Config/AID<n>.plist`; not mic, ADM or DSP-database files) are byte-identical to 25G83's is
    compiled with 25G83's laws whatever build the Mac runs, so v2 renders exactly what the 25G83
    block measured for that key, and the 25G83 block decides. `verify` says so in its detail
    ("tuning from macOS 26A434: its speaker files are byte-identical to 25G83's ...").
  - Any other tuning keeps the build its bundle came from. A bundle from a build with no block is
    refused ("macOS 26A428 has no parity attestation"), even when another release's block holds
    the same tuning key: that release's Audio Units, not this build's, were the reference.
  New tunings on new macOS builds therefore get coverage by a run of the steps below on that
  build, never by inheritance from an older one. The reference mapping is a choice of target (v2
  renders like 25G83), not a claim about the newer release's units; a run on the newer build
  measures that. Every 25G83 AID was byte-identical in 26A428, 26B123, 26B8125, 25C56 and 25G72
  when the mapping went in, and keys that pass in several blocks do so within a few hundredths of
  a dB of each other.

### Floor residue

Both `gate.py` and `attest.py` apply one per-measurement rule, the same for every tuning (it names
no board and holds no tuning data):

- a measurement whose macOS level is more than 55 dB (`FLOOR_RESIDUE_DB`) under its lane's own peak
  (the loudest level that lane reaches anywhere in the program, macOS side), where v2 is **quieter**
  than macOS, is *floor residue*: reported and counted, not a failure or a refusal;
- a measurement where v2 is **louder** than macOS fails at every level, floor or not;
- every main-band measurement (55 dB or less under the lane's peak) is gated as before.

Why: the macOS voicing units run their filters in Float32 arithmetic. Low-corner, high-order
sections at 48 kHz have a large low-frequency noise gain, so a full-scale tone leaves a
deterministic residue around -90 dBFS that a double-precision path does not have (measured
2026-11-09 on the installed 25G83 VirtualBass unit: a 4 kHz or 16 kHz tone at 0.7 peak leaves
-89..-91 dBFS below 2 kHz, linear in level and independent of the harmonic and blend gains;
v2: -143..-174 dBFS). In the woofer lanes of a high tone that residue is the whole cell, so v2
reads several dB "quieter" there while the program itself matches. Such a cell is inaudible
(more than 55 dB under the lane's own program) and on the safe side: v2 sends less to that
driver, never more. Louder cells stay failures at any level because extra energy is what
protection must never pass. The rule does not replace exact parity: matching the units'
Float32 arithmetic remains the goal, and the gate still prints every floor-residue cell.

### Arithmetic-chaotic cells

Some cells measure macOS's own Float32 arithmetic rather than the program: the woofer lanes of a
high tone, the residue a 20 Hz high pass leaves of a 20 Hz tone, a crossover's stop band right
after a loud step. There the units' low-corner, high-order Float32 filters amplify any one-bit
difference in their input into a different rounding pattern, so macOS does not even agree with
itself. Measured 2026-11-09 on 25G83: VirtualBass fed v2's Norm output, which differs from
macOS's by 3.6e-7, lands 72 dB under its output instead of 103 dB, and the analytical crossover's
stop band after the 2 kHz step-down misses an exact convolution with its own impulse response by
+-0.03 dB.

So the gate measures that spread on macOS itself:

1. `attest.py spread` compares every reference with renders of the same tuning from four or more
   (eight on 25G83 and 25C56)
   copies of the program in which every nonzero sample moved by one Float32 ULP (seeded; zeros
   stay zero). A cell's spread is the largest |dB| of any of its measurements over those renders.
2. A measurement is held to that spread instead of the ordinary tolerance (louder included) only
   if the spread is larger, its macOS level is below -60 dBFS absolute and it sits more than
   40 dB under its lane's own peak. A main-band measurement keeps the ordinary tolerance whatever
   the spread, except for the capped allowance below.
3. A main-band measurement passes when v2's deviation is within macOS's own spread for that cell
   **and** within 0.25 dB (`MAIN_SPREAD_CAP_DB`), louder or quieter. Above 0.25 dB it fails as
   before, whatever the spread. Measured on 25G83, macOS moves by 0.07-0.14 dB in some main-band
   cells (AID9 step-down tail 0.14, AID36 31.5 Hz 0.12, AID10 20 Hz 0.07): at that size macOS does
   not agree with itself, so a 0.05 dB demand there tests rounding luck, not parity. The cap keeps
   every main-band difference within the attestation's own louder bound.
4. `spread-<build>.json` (this directory, one per macOS release that has a host for the perturbed
   renders: 25G83 and 25C56 today; 26A428 needs a Mac on that build and is gated by the ordinary
   rule until it has one) holds tuning keys, cell names and spread values only: no
   Apple data. `parity.json` records the spread file's SHA-256 and the rule's limits in that release's block (`releases[].spread`); a release without a spread file is gated by the ordinary rule. Regenerate
   both for every macOS release, from the same references (`attest_refs.sh` once per perturbed
   program: `GATE`-side `perturb.py`).

`gate.py --spread spread-25G83.json` applies the same rule per AID at 0.05 dB; it prints the held
cells (`chaotic_held`) separately and counts a measurement outside macOS's spread as over the gate.
Exact parity stays the goal: the floor and spread rules only stop the gate from demanding more
agreement than macOS has with itself.

### Volume sliders

The speaker graphs feed the system volume to volume-dependent stages: Tone automation (`tmst`
SystemVolume), VarEQ, LowFlow, the VolumeTaper and its Comp relay, and the graph's gain box. A
mapping error there shows only below full scale, so `attest.py` gates every tuning at sliders
0.7, 0.46 and 0.25 (`SLIDERS`) besides full volume:

- `attest.py variants` records, per tuning, the `(mvsc, mvol)` pair for each slider: `mvsc` is
  the user scalar and `mvol` the dB the chain's volume law gives it (the HAL law over the Mac's
  own range, `voicing-render --mvol auto`). That is the pair macOS's HAL hands the graph.
- `attest_refs.sh` with `SLIDER=S` renders the macOS side at that pair (`gr.py --param mvsc=..
  --param mvol=..`), one reference directory per slider.
- `attest.py gate --slider S=DIR` renders v2 at the same pair and compares *settled* band
  levels: every steady segment (the stepped third-octave sines and pink), per lane, second-half
  RMS from 10 Hz up (`SETTLED_HP_HZ`), with the same tolerance (louder 0.25 dB, quieter 1.0 dB),
  stop band, floor-residue and arithmetic-chaotic rules as full volume. Each entry gets
  `sliders[]` (verdict and worst numbers per slider); a refusal at any slider refuses the
  tuning. The release block lists the sliders it was gated at (`releases[].sliders`), and
  `verify` requires a passing verdict at each (`parity.rs`).
- Why from 10 Hz: the program's lowest tone is 20 Hz, so the cut removes no program content,
  only subsonic drift. Below about 10 Hz the units' Float32 residue after a loud high tone sits
  in the woofer lanes, and Tone automation lifts it at low volume. Measured 2026-11-12 on 25G83,
  AID14 at slider 0.7, woofer lanes of `tone_16000_0`: broadband -76.4 dBFS on macOS and
  -70.1 dBFS on v2, 65 dB under the tweeter lane, almost all of it in the two lowest FFT bins.
  macOS does not reproduce that cell itself: under +-1 ULP program perturbations it moved by up
  to 1.4 dB over 4 renders, 4.0 dB over 8, 6.7 dB over 16 and 32, and v2's sign flips between
  sliders. From 10 Hz up the same cell is -81.8 dBFS on both sides (-0.05 dB). Tone automation's
  steady response at 5-40 Hz matches macOS within 0.07 dB at sliders 1.0, 0.7 and 0.46.
- Spread at a slider (`attest.py spread --slider S --perturbed ...`, `gate --slider-spread
  S=FILE`): macOS's own +-1 ULP spread measured on the slider's references, settled the same
  way, down to the absolute floor rather than the stop band (the stop band moves with volume, so
  a cell can sit just under it in every macOS render and just over it in v2's). No release needs
  one today. If a tuning is refused at a slider on deep cells only, render 16 or more perturbed
  programs at that slider and gate again with the spread.
- A release without a host for slider references (26A428 needs a Mac on 27.0) can only be
  written with `--full-volume-only`. `verify` accepts such a block on its full-volume verdicts
  and says "full volume only" in its detail; `test_attest.py` lists the builds allowed to be in
  that state. Regenerate the block with sliders as soon as a host on that build exists.

### When it runs

Run all three steps, in full, whenever:
1. **a macOS release ships** (every point release and every device-forked build: their Tunings may
   add AIDs, speaker configurations or new states). Update the host Mac to it first, because the
   reference is the installed Audio Units. Extract the release's Tunings folder from its IPSW
   (`ipsw extract --remote --dmg fs`, or SGUP2's `tools/aea_ro.py` + `apfs_ro.py` ranged reads) for
   AIDs the host's own tree lacks;
2. **the rendering code changes** (the law changes; the macOS renders from step 2 can be reused while
   the release is unchanged);
3. **the gate program or the tolerance changes.**

Each run regenerates the spread file too (step 3 in `attest.py`), before the gate step.

```text
# buildbox
attest.py variants --bin BIN --build BUILD --tunings NEW_TREE --tunings-fallback HOST_TREE --out variants.json
# host Mac on that release, under a host-heavy slot (about 2 s per tuning and slider);
# one SCRATCH per concurrent run (renders are named by tuning)
BOXHOST=.../box_host SCRATCH=... PROG=prog.f32 attest_refs.sh SLOT variants.json
for S in 0.7 0.46 0.25; do SLIDER=$S DEST=...REFS-$S BOXHOST=... SCRATCH=...-$S PROG=prog.f32 attest_refs.sh SLOT variants.json; done
# buildbox (about 2 min for 63 tunings with 12 jobs)
attest.py gate --variants variants.json --mac REFS --bin BIN --prog prog --reference "macOS <ver> <build> Audio Units" \
    --slider 0.7=REFS-0.7 --slider 0.46=REFS-0.46 --slider 0.25=REFS-0.25 [--slider-spread S=spread-<build>-sS.json] \
    --out packaging/session/parity.json --details PRIVATE_DIR
```

A tuning refused at a slider on deep cells only gets 16 or more perturbed renders at that slider (`perturb.py`
seeds, `SLIDER=S PROG=prog-pN.f32 attest_refs.sh`), then `attest.py spread --slider S --variants V --mac REFS-S
--perturbed P1 ... P16 --out spread-<build>-sS.json` and the gate again with `--slider-spread`.

Commit the new `parity.json` in the same PR as the code change it attests. `test_verify.py` checks
that the file is consistent with its own tolerance, and `cargo test` (`crates/aurora-speaker-tuning/tests/parity_law.rs`)
fails while the file's `law` differs from the code's.

### Builds without a Mac (`ipsw_units.py`)

A release can be attested without booting a Mac on it when its units load on the reference
host. `ipsw_units.py BUILD BOARD DIR --ipsw URL` reads the build's AudioDSP.component and Tunings
from its restore image by HTTP ranges. After an ad-hoc re-sign of the private copy, `attest_refs.sh`
runs with `AUDIODSP_PATH=DIR/AudioDSP-signed`, `TUNINGS_NEW=TUNINGS_HOST=DIR/Tunings`, and
`attest.py variants/gate --build BUILD` writes that build's block. The reference string says
so ("its AudioDSP.component ... rendered on a <host build> host"): the system frameworks the
units link against are the host's.

Coverage on 2026-11-09 (host 25G83): 25C56 (26.2) loads and is attested this way.
26A428 (27.0) does not load ("built for macOS 27.0 which is newer than running OS"), so it was
gated on a J616s booted into 26A428. Newer builds than the host need such a Mac. A build
whose AudioDSP and Tunings are byte-identical to an attested build's renders the same.

`attest-build.sh BUILD BOARD IPSW_URL` runs those steps for one build: fetch (a Linux Mac with liblzfse +
python3-cryptography), carry-over check against `units.json` (AudioDSP hashes of attested builds, hashes only),
host render with that build's units, gate and merge. Per-release trigger: run it for every new macOS build in
Apple's feed (point releases, betas users install, device-forked builds) and after every law change (all blocks,
because a block is kept only with the law it was gated at). A build whose units do not load on the host stops with
exit 3; that build needs a reference host on it (today the J616s on 26A428). First end-to-end run: 25G72 (26.6) on
2026-11-09, fetched on an M2, rendered on the 25G83 host, 39/56 pass.
