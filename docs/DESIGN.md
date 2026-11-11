# Design

## Principles

1. **Do what macOS does.** The protection unit, its position in the speaker graph, its inputs, its parameters and its failure behaviour all follow macOS. Behaviour comes from the behaviour specification. Parameters come from the user's own tuning bundle at runtime. Gaps are marked `TODO(SPEC)` and are closed from the specification or from macOS reference traces, never by guessing. No invented margins, heuristics or extra stages.
2. **Two first-class units from the start.** spp3 and lsm2 share one framework (`sgv2-core::ProtectionUnit`), with atsp following.
3. **Parity is the acceptance test.** Each unit's per-quantum state must match macOS's on identical input, within stated tolerances, including the failure and edge cases (`tools/diffrig`).
4. **Fast and real-time safe.** No allocation, locks or syscalls in the audio path. NEON kernels, with scalar references that must match them. Only the unit's own algorithmic latency is added. Sense is ingested synchronously, at the smallest period the hardware qualifies.

## Graph

```
apps → voicing-pre filter-chain (user volume, the stages before the unit)
     → protect filter-chain: sgv2 LV2 (all channels, one instance):
         pre-stage (identity by default; tuning-derived only on request) → unit
     → the stages after the unit → speaker PCM
sense PCM (same card, same clock, same cycle) → protect filter-chain sense ports
```

- **Generated voicing mode (default off).** `gate.json`'s per-board
  `voicing_pre` flag replaces the interim convolver with a stereo-to-lanes
  LV2 host of `sgv2_core::voicing::Chain`. The full spec (including box
  parameter words, automation and relays) is a CRC-checked versioned section
  appended after the length-delimited pre-stage in the unit blob. It binds
  to the exact preceding unit blob. The supervisor publishes both together;
  the protect plugin's pre-stage is identity and declares the crossover
  supplied by the full chain. Both plugins refuse invalid artifacts, with
  silence from voicing. See `packaging/pipewire/README.md` for activation.
- The unit sits where the Mac's own speaker graph places it, and the stages around it come from the same bundle.
- **Pre-stage.** The compiler defaults to identity for every protection unit: it protects the signal supplied by the external voicing graph. `--prestage tuning` explicitly requests the interim tuning-derived pre-stage; its crossover/compressor admission rules apply only in that mode. `--voicing-pre` compiles the full voicing artifact and keeps the protection pre-stage identity. Changing from the former tuning default to identity changes the pre-stage bytes, while the protection payload stays identical; equivalent tuning and voicing modes retain their existing encoding. In explicit tuning mode, spp3 limits coil temperature; how hard each cone is driven at low frequency is set by the filtering macOS's graph applies ahead of the unit: the program's high-pass and, per speaker, the crossover (tweeter high-pass, woofer low-pass), shelves, gains and cuts. The compiler reads those filter boxes along each unit channel's lane (`aurora-speaker-tuning::prestage`) into second-order sections that the plugin runs right before the unit (`sgv2-core::prestage`). A filter it cannot carry exactly is left out only when it is a boost; otherwise the compile is refused, as is a filter box without a preset, a saved-state field that is missing, or a box ahead of the unit the compiler does not account for. The filters' response is scaled to never exceed unity. The supervisor declares it to the kernel ("Speaker Protection Crossover"), and only then does the kernel open the tweeters' amp high-pass to the Mac's own corner.
- **Dynamics (explicit tuning pre-stage).** The graph's multiband compressor boxes ahead of the unit (`aufx/mstk`) are part of the pre-stage too, each compiled by the voicing box (`voicing::mstk`) from its saved state and run at its own place in the lane: the filters and gains before it run ahead of it, the rest after, so each detector sees what macOS's sees. Its detector is linked across every channel, its band tree is the unit's (measured: split at the middle crossover first, Linkwitz-Riley crossovers of the order control rounded up to even), and its output carries the unit's own gain staging (input and output gain, band input gains, makeups). The gain computer is interim (peak power, hard knee, instant attack): the unit's own is an RMS detector with a soft, ratio-dependent knee and a soft range, which the interim law does not bound from above in every band (up to 2 dB louder at full scale on some states), so the exact law is required before a board's voicing counts as matching macOS. A compressor feature the box does not carry refuses the compile.
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

It renews the interlock only while a client stream plays into the protect stage (the session helper reports it) and these all hold:
- the plugin's heartbeat is fresh, and the plugin runs the published parameters;
- the graph's links are exactly the configured set;
- the sense carrier is at unity;
- the speaker PCM belongs to the process hosting the plugin.

Any failure releases the interlock at once, and the kernel's own safe level applies. With nothing playing the helper unlinks and suspends the amps and the sense (both PCMs close) and the supervisor gives the interlock back, so an idle Mac holds its speaker path at the floor with its PCMs and DMA stopped, as macOS powers it down. The protect stage keeps running on PipeWire's dummy driver, so its heartbeat, checks and thermal model carry on, and the next stream is granted within one tick of the helper's report. That safe level is the hardware floor whenever no protection holds the amplifiers. There is no chaining to other daemons.

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

With these, the supervisor can compare the acoustic-id with the installed bundle's manifest, and the speaker ID with the device tree export once it exists. Until then, the compiler checks the acoustic-id against an independently known value when the caller has one (`Machine::acoustic_id`). On Linux no such value exists, so the binding rests on the board, the unit and the speaker ID used for selection.

### The bundle belongs to one Mac

SpTS is one unit's factory calibration. A bundle copied from another Mac of the same model would pass every check above, and its resistances would seed the closed loop: about 8 K of temperature error per 3 % of R, enough on a board that relaxes (J616s) to raise the limit above what this unit's coils allow. So the installer records where it collected the bundle, and the compiler checks it against the running Mac:

- `board` in the manifest: the ADT root `target-type`, lowercased (`J616s` → `j616s`), the name the device tree's first compatible carries.
- `unit_id` in the manifest: SHA-256 over the tag `aurora-speaker-unit-v1\0` and the root `serial-number` without trailing NULs. macOS has it in the `IODeviceTree` root (32 bytes, NUL-padded); aura copies the same ADT property into the device-tree root (one NUL), where Linux reads it from `/proc/device-tree/serial-number`. Every aura and m1n1 release exports it, so no boot-chain change is needed. Only the hash leaves the collector; the serial number never reaches the bundle, a blob, a diagnostic or the provisioning record.

| bundle | this Mac | compile |
|---|---|---|
| another `board` | any | refused: no blob, no grant, speakers at the kernel floor |
| another `unit_id` | a readable `serial-number` | refused |
| the same `unit_id` | the same | SpTS in use (closed loop as qualified) |
| no `unit_id` (collected before the installer recorded it) | any | open loop at the base limits, warning `unit-unbound` |
| a `unit_id` | no or a malformed `serial-number` | open loop, warning `unit-unverifiable` |

Open loop is the most protective case, not a guess: the closed loop can only raise the limit (`clamp(T_OL - T_CL, 0, cap)`), so without trusted calibration each channel holds its base limit, as M2 and M1 Pro already do. A unit without SpTS (lsm2, atsp, desktops) has nothing per unit, and a bundle without identity compiles for it as before. Provisioning binds the blob to the unit too (`provision.bind` key `unit`, the same hash), so a disk moved to another Mac recompiles and is refused there.

To re-collect, run the installer's firmware step on this Mac's macOS, or `tools/speaker-tuning/collect-macos.sh` there and `install-linux.sh` on Linux (aurora-installer), then restart `speakerguardd-provision.service`. The manifest is integrity metadata, not a signature: the binding stops the wrong Mac's bundle, not a forged one.

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
