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
amplitude only, not sample identity. It runs at 48 kHz and full volume only. The protection unit is
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
   copies of the program in which every nonzero sample moved by one Float32 ULP (seeded; zeros
   stay zero). A cell's spread is the largest |dB| of any of its measurements over those renders.
2. A measurement is held to that spread instead of the ordinary tolerance (louder included) only
   if the spread is larger, its macOS level is below -60 dBFS absolute and it sits more than
   40 dB under its lane's own peak. A main-band measurement keeps the ordinary tolerance whatever
   the spread, so a main-band cell louder than macOS still fails.
3. `spread-25G83.json` (this directory) holds tuning keys, cell names and spread values only: no
   Apple data. `parity.json` records the spread file's SHA-256 next to the tolerance. Regenerate
   both for every macOS release, from the same references (`attest_refs.sh` once per perturbed
   program: `GATE`-side `perturb.py`).

`gate.py --spread spread-25G83.json` applies the same rule per AID at 0.05 dB; it prints the held
cells (`chaotic_held`) separately and counts a measurement outside macOS's spread as over the gate.
Exact parity stays the goal: the floor and spread rules only stop the gate from demanding more
agreement than macOS has with itself.

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
attest.py variants --bin BIN --tunings NEW_TREE --tunings-fallback HOST_TREE --out variants.json
# host Mac on that release, under a host-heavy slot (about 2 s per tuning)
BOXHOST=.../box_host SCRATCH=... PROG=prog.f32 attest_refs.sh SLOT variants.json
# buildbox (about 1 min for 63 tunings with 12 jobs)
attest.py gate --variants variants.json --mac REFS --bin BIN --prog prog --reference "macOS <ver> <build> Audio Units" \
    --out packaging/session/parity.json --details PRIVATE_DIR
```

Commit the new `parity.json` in the same PR as the code change it attests. `test_verify.py` checks
that the file is consistent with its own tolerance.
