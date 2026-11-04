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

## Rollout

v2 replaces the previous daemon on a machine only once that machine passes the differential qualification:
- parity with macOS;
- fault injection;
- zero SELinux denials in enforcing mode;
- a clean boot log;
- loudness and clarity measured against macOS.

Until then that machine keeps what it runs today.
