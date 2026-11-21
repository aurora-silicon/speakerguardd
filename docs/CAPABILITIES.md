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
| `driver:` | a constant of the kernel driver a compatible binds | sample formats, ring cadence; keyed by the **block's** compatible |
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
| ALSA card id | graph (`hw:CARD=`), supervisor | `alsa:` the card bound to the sound node (**this change**); offline `dt:` model | capability |
| Machine key (`apple,<key>-macaudio`) | names the voicing graph and the generated file | `dt:` sound node compatible | naming only |
| Protection unit, roles, rate | wiring, blob | `tuning:` | capability |
| Units that may run without sense | wiring | `tuning:` unit kind (spp3 needs sense) | capability |
| Volume step and dB level | supervisor operating point | `alsa:` control TLV | capability |
| Applied sense geometry (format, channels, period) | self-check | `alsa:` applied hw params | capability |
| Speaker identity (`spid`) | tuning selection | `dt:` `/chosen/asahi,speaker-id` (the loader, from the ADT straps); bundle manifest without it | capability where the loader exports it (below) |
| Fixed volume ceiling under the lease | supervisor grant check | not read: the supervisor expects 0 dB and latches on anything else | provider gap (below) |
| Interim voicing trims and low-pass | voicing graph | `gate.json` tables keyed by the bundle's speaker identity (`aid`) | tuning (below) |
| Voicing deployment overrides | offline fixtures' recorded mode | `gate.json` `voicing_pre` keyed by board | policy, to re-key with the fixtures (below) |
| High-pass in front of protection for a speaker without a model | protect graph | not on main | tuning, keyed by the bundle (below) |
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
- A grant at any volume other than the one expected: the supervisor latches.

The only defaults are the safe ones. No missing value admits a Mac.

## Providers first, fallbacks second

Where a capability must first be exposed by the kernel, the device tree or
the loader, that provider lands first. speakerguardd reads it when present
and keeps today's path as the fallback when it is absent; the fallback goes
once a release carries the provider. The fallback is never more permissive
than the provider.

1. **Fixed ceiling under the lease.** The kernel knows the level its lease
   lifts the speaker to on a machine with a fixed ceiling. Provider: the
   kernel reports it to userspace (the leased volume control's own range,
   or a read-only control). Reader: the self-check records it and the
   supervisor grants only at exactly that level. Fallback until then: a
   `gate.json` value, which only lowers the volume and is cross-checked
   against the card; with neither, 0 dB is expected and a ceiling latches.
2. **Speaker identity.** Linux has no passive reader of the speaker-ID
   straps. Provider: the loader reads the straps the ADT names
   (`find-speaker-id`, `function-speaker-id0/1`, `valid-speaker-ids`) and
   exports the ID as `/chosen/asahi,speaker-id`; aura does this
   (`kboot_audio.c`), and the J616s and J713 boot with it. Reader:
   `aurora-speaker-tuning` reads it when present. Fallback, for a boot
   chain without the export: the bundle manifest's `speaker_id`.
3. **Sense presence on a new transport.** Provider: the back-end node's
   `dma-names` (sense channel named or not), already used for the T6041
   and T8132 AOP. A new driver entry names its `speaker_dma` and
   `sense_dma`.

## Per-model data is tuning

Values that compensate for an open voicing law, or protect a speaker that
has no model, belong to the speaker, not the board. They stay as tuning
keyed by what the bundle names (its speaker identity), never as a code
gate, and never as a value that admits a Mac on its own:

- `gate.json` `voicing_pre_bound_db` and `voicing_pre_lowpass` are keyed
  by the bundle's speaker identity (`AID<n>`, the `aid` the wiring carries
  from the tuning). The one entry with an effect is `AID13`, the J413's
  speakers (woofers -0.043 dB); the 0 dB entries were dropped. A key that
  is not a speaker identity refuses the table, and a description without
  an `aid` cannot generate the voicing, so no Mac loses its bound without
  a word.
- `voicing_pre` overrides equal the runtime default (generated voicing) on
  every board they name; they keep the offline fixtures' recorded mode and
  are re-keyed with the fixtures.
- A protect-stage high-pass for a speaker without a model is tuning, keyed
  the same way.

## Open changes and the capability layer

| Change | Its machine-specific part | Maps onto |
|---|---|---|
| T8152 transport without sense, fixed ceiling, high-pass (J873g) | `apple,t8152-aop-audio` entry; `operating_volume_db`; `protect_highpass` keyed by board | transport entry as is (per block); ceiling via provider 1 with the table as fallback; high-pass as tuning keyed by the bundle |
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
