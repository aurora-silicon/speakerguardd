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
dynamic segments where either side is above -80 dBFS. Any value outside 0.05 dB in either direction
fails. One integer latency per AID is estimated by cross-correlation on the pink segment and applied;
a non-zero lag fails the AID as a timing gap. An AID passes only if its chain compiles with no stubs,
the lane counts match, the lag is 0 and no measurement is over the limit. Each AU box's cumulative
output is compared the same way, and the first box over the limit is named.

`summarize.py GATE_OUT SWEEP_JSON...` joins the gate results with the all-AID compile sweep
(`aidsweep.sh`: refusals, stubs, bounds, TODO(SPEC)) into one table per AID.

Limits: the reference is the installed host build's units (25G83). States from the 27.2 tunings that
run through those units are compatibility measurements, not native 27.2 qualification. The gate checks
amplitude only, not sample identity. It runs at 48 kHz and full volume only. The protection unit is
outside this gate; it has its own rigs. Every output made from Apple tuning is Apple-derived: keep it
in private directories and publish numbers only.
