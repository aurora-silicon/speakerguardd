# New platform readiness: speakers on day one

This page lists what a new Mac needs before speakerguardd v2 can drive its
speakers. For each item it says whether the item is automatic or still needs
a code change. The aim is that a new Mac's speakers work with no
hand-written template and with nobody at the machine.

## Summary

| Item | Automatic? | What a new Mac needs |
|---|---|---|
| Protection unit | spp3 only | spp3 is at parity. lsm2 and atsp are pending. |
| Kernel driver | no | A `macaudio` match entry and a device tree for the board. |
| Tuning | yes | The installer collects the Mac's tuning bundle. |
| Parameter blob | partly | The compiler has a table of boards, so a new board needs an entry. |
| Channel map and V/I order | yes | Derived from the device tree and the tuning, then checked by the self-check. |
| PipeWire and WirePlumber files | yes | Generated from the self-check's record. |
| Supervisor | yes, for SN012776 amps | Takes its machine description from the self-check's record. |
| LV2 plugin variant | 2, 4 or 6 amps | Any other amp count needs a new variant. |
| Voicing (the stages before protection) | no | Still the interim asahi-audio graph. A new board has none yet. |

## 1. The protection unit

The tuning names its protection unit, and the reader (`aurora-speaker-tuning`)
records which one it is.

- **spp3** is at parity. It covers the V/I Macs with TAS2764/SN012776 amps:
  the J3xx, J4xx and J6xx families.
- **lsm2** is pending. It covers the J700 Neo (no V/I) and J713/J813 (V/I).
- **atsp** is pending. It covers the audio-only Macs.

Until a Mac's unit is at parity, its speakers stay at the kernel's floor.

## 2. The kernel driver and the device tree

The following needs a code change for each new Mac.

- `sound/soc/apple/macaudio.c` needs an `apple,<board>-macaudio` match entry
  with the board's platform configuration: the amp type, the amp gain, the
  safe level and the speaker front-end channels. Without it, macaudio refuses
  to enable the speakers ("can't assure safety on this model").
- The board's device tree must describe the speakers, as the J316 and J413
  trees already do:
  - a `Speaker`/`Speakers` dai-link under the `apple,macaudio` node, with the
    amps in `codec` `sound-dai` and the back-ends in `cpu` `sound-dai`;
  - a `sound-name-prefix` on each amp that names its side, role and ordinal:
    `Left Woofer 1`, `Right Tweeter`, `Left Speaker`;
  - `ti,vmon-slot-no` and `ti,imon-slot-no` on each V/I amp.

The following works with no code change.

- The amp driver: `tas2764` covers SN012776 and TAS2764. It already exposes
  the VSENSE/ISENSE switches and regmap in debugfs.
- The v2 ownership interface ("Speaker Protection"), the floor and the
  quiet refusals. All of these are generic across macaudio boards.

The AOP speaker path (J614s, J616s, J713) and the J700's MAX98360A path are
not derived yet. Section 5 says why.

## 3. Tuning collection

This is automatic. On the user's own macOS, the installer collects the
Mac's `AID<n>` folder and its per-unit speaker calibration, and stores them
with the vendor firmware. The project never ships them.

The reader then finds the protection unit and, for each unit channel:

- the side it serves: a lane starts on the graph's left or right input;
- its role: woofer or tweeter, from the crossover branch it passes (a join
  whose name ends in `LF` or `HF`). With no crossover branch, every channel
  is full-range.

**Code change needed:** the blob compiler (`aurora-speaker-tuning compile`)
still looks the board up in its own table (`machine.rs`) to get the amp count
and gain step. A new board needs an entry there, until the compiler takes
that description from the self-check's record instead.

## 4. The self-check (`speakerguardd-selfcheck`)

This is automatic. It runs as `speakerguardd-selfcheck.service` before the
supervisor, while the kernel holds the speakers at the floor.

1. **Derive.** It works out the channel map (the unit channel for each
   physical speaker channel) and the sense map (which amp's V or I is on
   each sense PCM channel). It uses only:
   - the device tree: macaudio's TDM slot rule, each amp's prefix, and its
     `vmon`/`imon` slots;
   - the tuning's graph, as in section 3.

   If anything is missing, unknown or ambiguous, the derivation refuses and
   says why. Examples: an amp with no prefix, two amps with the same side,
   role and ordinal, a slot that leaves gaps, or a tuning that sets
   `SpeakerInputDataSourceOrder`. A `machines.json` entry is only an
   optional override, and any disagreement with the derivation fails the
   check.
2. **Check the map against the tuning.** Each physical channel's amp, by its
   device-tree side and role, must be the speaker its unit channel models.
3. **Check the map against the hardware, silently.** Every sample of the
   stream is zero, so nothing can be heard.
   - Each amp's registers, read from regmap in debugfs, must show its RX slot
     as its speaker PCM channel and its V and I TX slots as the device tree
     gives them.
   - With a zero-sample stream running, the check turns off each amp's
     VSENSE and then its ISENSE. Each time, exactly the one sense channel the
     map assigns must go quiet (zeros) or fall to its idle pattern, and every
     other channel must stay live.
   - The kernel admits a stream only under a grant. So the check takes
     "Speaker Protection" itself, holds a 50 ms lease only while its own
     stream runs, keeps both PCMs to itself, and tells the session helper
     not to link.
4. **Bind.** The result goes to `/var/lib/speakerguardd/selfcheck.bind`, with
   a readable `selfcheck.json` beside it. The record names:
   - the board;
   - the kernel release;
   - the parameter blob's SHA-256;
   - the wiring hash.

   The supervisor refuses to run without a passing record for exactly this
   board, kernel and blob. It grants only while the session helper reports
   that the protect stage was built with that wiring (`sgv2.wiring`). After a
   kernel or tuning change the record no longer matches, and the next start
   runs the check again. A failure logs every reason and leaves the floor in
   place.

For a board with no `machines.json` entry:

- `speakerguardd-selfcheck generate` writes the PipeWire and WirePlumber
  files from the record;
- the session helper reads the card and channel counts from the record;
- the supervisor builds its machine description from the record. This works
  for SN012776 amps only, which have an operating-point check.

The check takes about 13 s on the J316 (13 captures of 0.4 s) and about 8 s
on the J413. A boot that has a valid record only checks the binding.

## 5. What is not derived yet

- **J614s, J616s, J713 (the AOP speaker path) and J700 (MAX98360A).** These
  boards keep their `machines.json` hand maps, and the record marks them
  `method=hand` (unproven), as they were before the self-check.
  - The J616s's device-tree layout and AOP sense ring have not been run
    through the derivation.
  - The J700 has no V/I. Its silent check would be a test on the
    SpeakerTap, which does not exist yet.
- **The voicing graph.** The WirePlumber file still points at the interim
  asahi-audio graph for the machine. A new board has no such graph, so its
  protect stage runs, but nothing feeds it until the voicing is generated
  from the tuning bundle (lane B).
- **Other amp families.** TAS2770 (M1) has no register check and no
  operating-point check in the supervisor.

## 6. Checklist for a new Mac

1. Kernel: add the `macaudio` match entry and the board's device tree with
   the amps (section 2).
2. Compiler: add the board to `aurora-speaker-tuning`'s machine table, until
   it reads the description from the record.
3. Confirm the unit is spp3. If it is lsm2 or atsp, wait for that unit's
   parity.
4. Confirm the amp count has an LV2 variant (2, 4 or 6).
5. Everything else is automatic: the installer collects the tuning, the
   self-check derives and proves the wiring at first boot, and the graph is
   generated from the self-check's record.
