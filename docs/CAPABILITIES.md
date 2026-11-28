# Capabilities, not models

speakerguardd decides what a Mac's speakers can do from values the Mac
itself provides: its device tree, its running sound card, the kernel
drivers its device tree binds, and its own tuning bundle. It never chooses
a value because of a board, model or SoC name. Where a value is missing or
cannot be proven, the speakers stay muted, with the reason, exactly as
before.

This note lists every place the protect stack depends on the machine, where
each value comes from today, and what is still to move. The probe that
reads them is `wiring.probe()` (`packaging/session/wiring.py`, section
"capabilities"); `wiring.py caps [--live]` prints it with the source of
every value, and the self-check stores the same report in
`/var/lib/speakerguardd/selfcheck.json` (`caps`).

## Sources

| Prefix | Source | Examples |
|---|---|---|
| `dt:` | the device tree, as macaudio reads it | speaker dai-link, amp nodes and their `sound-name-prefix`, sense slots, back-end `dma-names` |
| `alsa:` | the running card | the card bound to the device tree's sound node; control dB scales (supervisor) |
| `tuning:` | the Mac's own tuning bundle (reader's graph trace) | protection unit, channel roles, sample rate, speaker identity (AID) |
| `driver:` | a constant of the kernel driver a compatible binds | sample formats, ring cadence; keyed by the **block's** compatible (see "Driver constants" below) |
| `derived:` | computed from the above | PCM widths, plugin variant, channel and sense maps |

The board name is still read, as an identity: the self-check record, the
parameter blob and the persisted thermal state are bound to the Mac they
were made on, so one Mac's proof never admits another. It selects no value.

## Inventory

| What | Used by | Source today | Status |
|---|---|---|---|
| Amp count, order, side/role/ordinal | wiring map, graph | `dt:` speaker link codecs, `sound-name-prefix`, TDM slots | capability |
| Amp family (V/I sense, gain control, fault status, volume control name) | wiring, supervisor `Amp` | `driver:` amp compatible (`AMPS`, `FAMILY`, `machine.rs`) | capability (per block) |
| Speaker transport (MCA, AOP variants) | graph format and buffering | `driver:` back-end compatible (`TRANSPORTS`) | capability (per block); a new block needs an entry, never a board entry |
| Sense present on this Mac | wiring method, unit admission | `dt:` back-end `dma-names` names the driver's sense channel (or not) | capability |
| Sense slots and their order | sense map | `dt:` `ti,vmon-slot-no` / `ti,imon-slot-no` | capability, proven silently by the self-check |
| ALSA card id | graph (`hw:CARD=`), supervisor, provisioning | `alsa:` the card bound to the sound node (**this change**; the self-check and provisioning use the same lookup, and a reused pass needs the same id; the gate's interface read still finds the card by its speaker link, the same card today); offline `dt:` model | capability |
| Machine key (`apple,<key>-macaudio`) | names the voicing graph and the generated file | `dt:` sound node compatible | naming only |
| Protection unit, roles, rate | wiring, blob | `tuning:` | capability |
| Units that may run without sense | wiring | `tuning:` unit kind (spp3 needs sense) | capability |
| Volume step and dB level | supervisor operating point | `alsa:` control TLV | capability |
| Applied sense geometry (format, channels, period) | self-check | `alsa:` applied hw params | capability |
| Speaker ID (`spid`, the straps' vendor ID) | vendor chain selection in the tuning reader | bundle manifest `speaker_id` and `dt:` `/chosen/asahi,speaker-id` (the loader, from the ADT straps): either alone is used, and the two must agree | capability where the loader exports it (below) |
| Playback ceiling under the lease | supervisor grant check | `alsa:` each playback volume control's current maximum from control info, converted to dB using its TLV | capability |
| Bundle speaker identity (`aid`) | voicing trim keys (below) | `tuning:` the bundle's `aid` (`AID<n>`), carried by the record and checked against the provisioned bundle at generation; no loader exports it | tuning |
| Interim voicing trims and low-pass | voicing graph | `gate.json` tables keyed by the bundle's speaker identity (`aid`) | tuning (below) |
| Voicing deployment overrides | offline fixtures' recorded mode | `gate.json` `voicing_pre` keyed by board | policy, to re-key with the fixtures (below) |
| High-pass in front of protection for a speaker without a model | protect graph | not on main | read from the Mac's own tuning bundle; absent = refused (below) |
| Board identity | record, blob, thermal state | `dt:` root compatible | identity binding, stays |

## Failing closed

Each capability either has a source or refuses:

- An unknown amp or back-end compatible, mixed families or transports, a
  missing prefix or sense slot: refused, muted.
- No sense on the transport: only a unit that runs without sense, on one
  amp, is admitted; the self-check proves the static method.
- Sense the device tree does not rule out: the self-check's silent sense
  test must see it on the card, or the record fails.
- No ALSA card bound to the sound node: the self-check fails.
- Under a grant, the supervisor latches if any playback volume control
  lacks a usable ceiling, the controls disagree on the ceiling, the
  ceiling fails the admission bounds, or any current level differs from
  the reported ceiling.
- A required high-pass ahead of protection needs its own admission check.
  The live ceiling check only proves the volume. The tuning reader and
  session graph need to prove the filter is present and applied before a
  route that requires it can be admitted.

The only defaults are the safe ones. One missing value still admits a Mac
today, and it is tracked:

- **Speaker ID unknown on a database with one chain per vendor.** When the
  tuning database has per-vendor configurations and neither the bundle
  manifest nor the loader names this Mac's speaker ID,
  `aurora-speaker-tuning` (`select.rs`, `select-speaker-id-unknown`) warns
  and compiles the vendor-neutral chain. That chain may carry protection
  parameters meant for another vendor's assembly. The fix makes this a
  refusal (no blob, muted); it changes the tuning reader, so it lands as its
  own change with a re-run of the macOS parity gate and a fleet row showing
  no Mac takes this path today.

## Providers first, fallbacks second

Where a capability must first be exposed by the kernel, the device tree or
the loader, that provider lands first. speakerguardd reads it when present
and keeps today's path as the fallback when it is absent; the fallback goes
once a release carries the provider. The fallback is never more permissive
than the provider, and for a value that protects the speaker the only
fallback is refusal.

1. **Playback ceiling under the lease.** macaudio publishes the effective
   playback maximum in ALSA control info. The supervisor converts that
   maximum and the current setting to dB with the same control TLV. Every
   playback control must report the same ceiling and be at that level
   while the grant is held. The supervisor's current admission band is
   −40 to 0 dB. An optional policy cap from `gate.json`
   (`operating_volume_cap_db`) can reject a higher card-reported ceiling.
   Missing control metadata causes a latch even when a policy cap exists.
2. **Speaker identity.** Linux has no passive reader of the speaker-ID
   straps. Provider: the loader reads the straps the ADT names
   (`find-speaker-id`, `function-speaker-id0/1`, `valid-speaker-ids`) and
   exports the ID as `/chosen/asahi,speaker-id`; aura does this
   (`kboot_audio.c`). Reader: `aurora-speaker-tuning`
   (`tuning.rs`, `resolve_speaker_id`) takes an explicit override first,
   then the manifest's `speaker_id` and the device tree's export: either
   alone is used, and when both are present they must agree or the bundle
   is refused as not this Mac's. This is the strap `spid`, which chooses
   the vendor chain; it is not the bundle's `aid` the voicing tables key
   on, which no loader exports.
3. **Sense presence on a new transport.** Provider: the back-end node's
   `dma-names` (sense channel named or not), already used for the T6041
   and T8132 AOP. A new driver entry names its `speaker_dma` and
   `sense_dma`.

## Voicing trims are tuning; protection values come from the machine

Two kinds of per-model data, with opposite rules:

- **Voicing trims** compensate for an open voicing law. They only
  attenuate, they never admit a Mac, and a missing entry means "no trim",
  which is harmless. They belong to the speaker, not the board, so they
  stay as tuning keyed by what the bundle names (its speaker identity),
  never as a code gate.
  - `gate.json` `voicing_pre_bound_db` and `voicing_pre_lowpass` are keyed
    by the bundle's speaker identity (`AID<n>`, the `aid` the wiring
    carries from the tuning). The one entry with an effect is `AID13`, the
    J413's speakers (woofers -0.043 dB); the 0 dB entries were dropped. A
    key that is not a speaker identity refuses the table, and an `aid`
    that is missing or not of the form `AID<n>` refuses the voicing, so no
    Mac loses its bound without a word.
  - The -0.043 dB value is ours, not Apple's: the bound `bound.py` computes
    from our full-chain safety scan of v2's voicing against the macOS
    chain's own render (VirtBass/Tone onset precision puts v2 a few
    hundredths of a dB over on woofer bursts). It is a voicing trim, not a
    protection level: each value is checked to lie in [-12, 0] dB and it
    sits before protection, which is unchanged by it.
- `voicing_pre` overrides equal the runtime default (generated voicing) on
  every board they name; they keep the offline fixtures' recorded mode and
  are re-keyed with the fixtures.
- **Protection values** (a volume ceiling under the lease, a high-pass in
  front of protection for a speaker without a model, any excursion,
  thermal or gain limit) protect the hardware. They come from the machine
  only: the kernel/driver (a control's range, a read-only control) or the
  Mac's own tuning bundle. They are never a table of ours, keyed by board
  or by speaker identity, and the keyed-table helper the voicing trims use
  is not reused for them. Absent from the machine = refused: no grant,
  muted.

## Driver constants

`driver:` values (amp `FAMILY` gain control and fault status, `TRANSPORTS`
formats, headroom and IRQ cadence) are userspace copies of kernel
constants, keyed by the block's compatible: no board gate. They are meant
as the offline value only; on a running Mac the live reading should win
and a mismatch refuse. Today that holds for the sample format: the
self-check reads the applied hw params and fails on a mismatch. Gain
control and fault status are not yet compared with the card's own control
list (the supervisor uses the copy); reading them from the card, and
refusing when the copy disagrees, is an open item.

## Open changes and the capability layer

| Change | Its machine-specific part | Maps onto |
|---|---|---|
| T8152 transport without sense, playback ceiling and pre-protection voicing (J873g) | `apple,t8152-aop-audio` entry and back-end DMA names; playback ceiling from the ALSA control range and TLV; compiled `AID48` `Atsp` + `--voicing-pre` chain | On main: transport discovery (`#272`), live ceiling check (`#222`), J873g fixture and 420 s `AID48` playback ladder (`#285`), and macOS `26B123`/`26B131` voicing parity attestation (`#286`, `compiler-gaps: none`, `sgv2#voicing-1ch` ahead of `sgv2#1ch`). Whether the compiled voicing chain counts as the required high-pass ahead of protection (CAPABILITIES lines 51/69) is an open maintainer decision; until it is made, a standalone tuning-backed high-pass and session-graph proof remain open. |
| T8142/T6050 transport without a sense channel | `speaker_dma`/`sense_dma` on the driver entry | as is: per block, `sense_path` |
| K114c sense on the v2 path | applied sense geometry | as is: `alsa:` applied hw params |
| Fail closed on an unproven method; units without sense by name | method dispatch, `NO_SENSE_UNITS` | as is: `tuning:` unit kind |
| Volume step from the control's TLV | operating-point tolerance | as is: `alsa:` TLV |
| Supervisor start on a late speaker card | udev trigger on the card | as is: the card's own appearance |
| Startup compile of verified bundles | bundle, unit, board binding | as is: `tuning:` plus identity binding |

## Proof

`wiring.probe()` gives the same hardware map and a byte-identical machine
description as before on every fleet device tree with a speaker link, for
every recorded tuning that fits it, and refuses exactly the trees that were
refused before; the generated PipeWire files are unchanged
(`gen.py --check`). On a running Mac, `wiring.py caps --live` reads only the
device tree and sysfs: it opens no PCM and writes no control.
