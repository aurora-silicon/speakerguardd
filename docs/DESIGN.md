# Design

## Principles

1. **Do what macOS does.** The protection unit, its position in the speaker graph, its inputs, its parameters and its failure behaviour all follow macOS. Behaviour comes from the behaviour specification. Parameters come from the user's own tuning bundle at runtime. Gaps are marked `TODO(SPEC)` and are closed from the specification or from macOS reference traces, never by guessing. No invented margins, heuristics or extra stages.
2. **Two first-class units from the start.** spp3 and lsm2 share one framework (`sgv2-core::ProtectionUnit`), with atsp following.
3. **Parity is the acceptance test.** Each unit's per-quantum state must match macOS's on identical input, within stated tolerances, including the failure and edge cases (`tools/diffrig`).
4. **Fast and real-time safe.** No allocation, locks or syscalls in the audio path. NEON kernels, with scalar references that must match them. Only the unit's own algorithmic latency is added. Sense is ingested synchronously, at the smallest period the hardware qualifies.

## Graph

```
apps → voicing-pre filter-chain (user volume, the stages before the unit)
     → protect filter-chain: sgv2 LV2 (all channels, one instance) → the stages after the unit
     → speaker PCM
sense PCM (same card, same clock, same cycle) → protect filter-chain sense ports
```

- The unit sits where the Mac's own speaker graph places it, and the stages around it come from the same bundle.
- The core is host-agnostic. The LV2 shim is the host for the PipeWire filter-chain, which every supported distribution ships.

## Sense

| Machines | Unit | Sense |
|---|---|---|
| V/I MacBooks | spp3 | voltage and current, for closed-loop temperature |
| MacBook Air M4/M5 13" | lsm2 | voltage and current, for system identification and closed-loop temperature |
| MacBook Neo | lsm2 | none: a static model, which is its normal mode |
| MacBook Pro 13" M1/M2, MacBook Air M1 | atsp | none |

Unusable sense makes the affected channel run open loop, as on macOS. Invalid parameters or a channel-count mismatch give the fail-safe attenuation.

## Supervisor and kernel

`speakerguardd` does no signal processing. It:
- loads and verifies the bundle, and publishes a compiled parameter blob that the plugin loads;
- holds the kernel's speaker interlock, with the amplifier at the operating point the tuning assumes.

It renews the interlock only while these all hold:
- the plugin's heartbeat is fresh, and the plugin runs the published parameters;
- the graph's links are exactly the configured set;
- the sense carrier is at unity;
- the speaker PCM belongs to the process hosting the plugin.

Any failure releases the interlock at once, and the kernel's own safe level applies. That safe level is the hardware floor whenever no protection holds the amplifiers. There is no chaining to other daemons.

## Binding a blob to its machine

The compiler (`aurora-speaker-tuning compile`) refuses, with no blob, any tuning the engine must not run: an error finding from the reader, an unverified bundle (unless `--development`), a channel count other than the board's amp count, or parameters the engine would answer with the fail-safe.

The blob header (`sgv2_core::blob`, version 1, 48 bytes) carries, in what were reserved bytes:

| Offset | Size | Field |
|---|---|---|
| 36 | 8 | the board: the first device-tree root `compatible` without `apple,`, ASCII, NUL-padded |
| 44 | 4 | the operating point: output peak voltage at digital full scale, mV |

The supervisor (`sgv2-supervisor`, `main.rs`) refuses a blob that names another board or no operating point, and grants only while every amp reads back that operating point. The compiler takes the board from the device tree, or from `--board` when compiling for another Mac.

The operating point comes from the tuning, in peak volts at a normalized sample of 1 (Daybreak t-4662): spp3 `ag`, lsm2 `VoltsFullScale`, and atsp `Amplifier sensitivity`. The header holds one value, channel 0's. The payload keeps each channel's own value, and all of them must lie within 0.25 dB of the header's, the supervisor's tolerance. The J700's 7.30 V and 7.33 V are 0.036 dB apart and pass. The machine description (`aurora-speaker-tuning::machine`) is a cross-check. It holds the amp gain the kernel fixes, whose dBV is rms-referenced, so 18.5 dBV is 11.899 V peak against AID35's `ag` of 11.89 V. A disagreement of more than 0.25 dB is reported as a warning. If the supervisor lane settles on per-channel operating points in the header, the compiler will emit them.

The speaker-assembly ID (`spid`) selects the vendor strip. The reader takes it from, in order: an explicit override; the installer manifest's `speaker_id` (read on macOS from the IORegistry `speakerID`); the device tree's `/chosen/asahi,speaker-id`. A manifest value that disagrees with the device tree's is an error. m1n1 does not export `asahi,speaker-id` yet: it would read the strap GPIOs that the Apple device tree names (`function-speaker-id0/1`; AP GPIO 52/53 on the J616s), validate the result against `valid-speaker-ids`, and write it to the FDT. Linux never requests those GPIOs itself, since claiming a line is not a passive read.

Not in the header yet, pending agreement with the supervisor lane:
- the acoustic-id;
- the speaker ID.

No byte of the version-1 header is free for them. Proposal for a header version 2:
- at offset 48, the acoustic-id (u32);
- at 52, the speaker ID (u32);
- at 56, flags (u32), bit 0 meaning the speaker ID is present.

With these, the supervisor can compare the acoustic-id with the installed bundle's manifest, and the speaker ID with the device tree export once it exists. Until then, the compiler checks the acoustic-id against an independently known value when the caller has one (`Machine::acoustic_id`). On Linux no such value exists, so the binding rests on the board and the speaker ID used for selection.

## Rollout

speakerguardd replaces what a machine runs today only once that machine passes the differential qualification:
- parity with macOS;
- fault injection;
- zero SELinux denials in enforcing mode;
- a clean boot log;
- loudness and clarity measured against macOS.

Until then that machine keeps what it runs today.

## Versions and the switch to main

speakerguardd starts again at 1.0.0: `1.0.0-alpha.N` while the units are built, then `-beta.N` while machines qualify, then 1.0.0. The 0.1.x daemon is a different program, archived on `legacy/v0.1` with tags `v0.1.12` and `v0.1.13-rc.2`.

When `v2` is ready, it becomes `main` by an ordinary merge that keeps history and replaces the contents. There is no force push:

```
git switch main
git merge --no-ff -s ours v2 -m "Merge v2: replace 0.1.x with speakerguardd 1.0"  # record v2 as merged, keep main's tree for a moment
git read-tree -u --reset v2                                                       # make the tree exactly v2's
git commit --amend --no-edit                                                       # the merge commit now carries v2's tree
git push origin main                                                               # fast-forward: main's old history stays reachable
```

The result is one merge commit whose first parent is the old `main` and second parent is `v2`, and whose tree is exactly `v2`'s. That commit lands as a pull request from a `v2-switch-main` branch, so it is reviewed like any other change, and the `main` push is a fast-forward. Afterwards `v2` is retired, and work continues on `main`.
