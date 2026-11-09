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
fails. Both sides are compared at their original sample positions with zero latency compensation.
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
  cell is louder than macOS by more than 0.25 dB or quieter by more than 1.0 dB.
- Each file is bound to `law`, a hash of the rendering code (`crates/aurora-speaker-tuning/build.rs`).
  A build whose law differs refuses every Mac until the file is regenerated: any change to the voicing
  compiler or engine needs a new run before it can ship.
- The file holds keys, verdicts, reasons and summary numbers only. Every render and the per-cell rows
  (`--details`) stay in private, tainted directories.

- The file holds one block per macOS release (`releases[].build`, format 2). `verify` takes the
  block whose build is the one the Mac's tuning bundle came from. A bundle from a build with no
  block is refused ("macOS 26A428 has no parity attestation"), even when another release's block
  holds the same tuning key: that release's Audio Units, not this build's, were the reference.
  New macOS builds therefore get coverage by a run of the steps below on that build, never by
  inheritance from an older one.

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

```text
# buildbox
attest.py variants --bin BIN --build BUILD --tunings NEW_TREE --tunings-fallback HOST_TREE --out variants.json
# host Mac on that release, under a host-heavy slot (about 2 s per tuning)
BOXHOST=.../box_host SCRATCH=... PROG=prog.f32 attest_refs.sh SLOT variants.json
# buildbox (about 1 min for 63 tunings with 12 jobs)
attest.py gate --variants variants.json --mac REFS --bin BIN --prog prog --reference "macOS <ver> <build> Audio Units" \
    --out packaging/session/parity.json --details PRIVATE_DIR
```

Commit the new `parity.json` in the same PR as the code change it attests. `test_verify.py` checks
that the file is consistent with its own tolerance.
