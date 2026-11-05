# New platform readiness: speakers on day one

This page lists what a new Mac needs before speakerguardd v2 can drive its
speakers. For each item it says whether the item is automatic or still needs
a code change. The aim is that a new Mac's speakers work with no
hand-written template and with nobody at the machine.

## Summary

| Item | Automatic? | What a new Mac needs |
|---|---|---|
| Protection unit | spp3 and atsp | spp3 and atsp are at parity. lsm2 is pending. |
| Kernel driver | no | A `macaudio` match entry and a device tree for the board. |
| Tuning | yes | The installer collects the Mac's tuning bundle. |
| Parameter blob | yes | Compiled and bound at boot by `speakerguardd-provision`. The compiler has no table of boards. |
| Configuration and seat user | yes | Written at boot by `speakerguardd-provision`; the seat user is resolved by name. |
| Fleet gate | data | `gate.json` (policy only: `units_at_parity`, and per unit the `qualified` boards); the unit is the one the Mac's own tuning names; the kernel's interface picks v2 or the older daemons. |
| Channel map and V/I order | yes | Derived from the device tree and the tuning, then checked by the self-check (MCA and AOP paths; the J700 is derived but cannot be proven silently). |
| PipeWire and WirePlumber files | yes | Generated from the self-check's record. |
| Machine description | yes | Derived from the device tree (`wiring.describe`): the machine key, card id, model, amp and sense counts, plugin variant, speaker format and buffering (per transport driver); the self-check confirms the card id and the format against ALSA. There is no `machines.json`. |
| Supervisor | yes, for SN012776 and MAX98360A amps | Takes its machine description from the self-check's record; there is no table of boards. |
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

The same holds on the AOP speaker path (J616s, J713): the firmware runs the
serializer, but macaudio still gives each amp its slot, the amps sit on I2C
with their own V/I slots, and the AOP sense ring passes the amps' TX slots
through unchanged. The J700's MAX98360A pair is one device-tree node with
no side: it is derived as left on slot 0 and right on slot 1, the frame
order each amp's SD_MODE strap selects from.

## 3. Tuning collection

This is automatic. On the user's own macOS, the installer collects the
Mac's `AID<n>` folder and its per-unit speaker calibration, and stores them
with the vendor firmware. The project never ships them.

The reader then finds the protection unit and, for each unit channel:

- the side it serves: a lane starts on the graph's left or right input;
- its role: woofer or tweeter, from the crossover branch it passes (a join
  whose name ends in `LF` or `HF`). With no crossover branch, every channel
  is full-range.

The blob compiler (`aurora-speaker-tuning compile`) needs nothing about the
board but its name, from the device tree. The amp count and the amp gain
are checked on the running Mac: the self-check refuses a tuning whose unit
channels are not the device tree's amps, and the supervisor checks the
blob's channels against the self-check's record and its operating point
against the amps' "Amp Gain Volume" readback.

## 4. Provisioning (`speakerguardd-provision`)

This is automatic. `speakerguardd-provision.service` runs at boot, before the
self-check and the supervisor, which both require it.

1. **Compile.** It finds the installer's bundle (the vendor firmware tree,
   then the ESP's `vendorfw/firmware.tar`) and compiles it into
   `/var/lib/speakerguardd/params.bin` with `aurora-speaker-tuning compile`,
   for this Mac's device tree.
2. **Bind.** `provision.bind` records what the blob was compiled from: the
   board, the kernel release, the bundle's digest
   (`aurora-speaker-tuning identify`: the manifest's files, sizes and hashes,
   wherever the bundle is stored), the macOS build it was collected from,
   the device tree's speaker ID and the compiler. The next boot compiles
   again only if one of those changed, or the blob on disk is not the one
   bound.
3. **Seat user.** It resolves the seat user by name, and the uid from the
   user database, never a fixed uid (on a Mac with several users, uid 1000
   can be someone else). In order: `SGV2_SEAT_USER` in
   `/etc/speakerguardd/provision.conf`; the user of seat0's active session;
   the last such user; the display manager's automatic login; the only
   regular user. With several users and none of those, there is no seat
   user until the first login on seat0: `speakerguardd-seat.path` follows
   seat0 and restarts the supervisor, which provisions again, whenever a
   different regular user becomes active there.
4. **Configure.** It writes `/etc/speakerguardd/speakerguardd.conf` (the
   machine, the seat user's name and uid, the supervisor's arguments with
   the speaker card and its PCM, and the self-check's). The file is
   generated; overrides go in `provision.conf`.
5. **Graph.** After the self-check passes, `speakerguardd-provision activate`
   generates the machine's PipeWire and WirePlumber files from the record
   into `/run/speakerguardd-graph`. The user unit `speakerguardd-graph.service`,
   ordered before PipeWire, links them into the seat user's
   `pipewire.conf.d` and `wireplumber.conf.d`, and removes them for every
   other user. If the seat user's PipeWire was already running, it is
   restarted on the new graph. The links lead into `/run`, so after a boot
   without v2 they lead nowhere, and PipeWire skips them.

Any failure fails closed: the blob, the configuration and the graph are
removed, the self-check and the supervisor do not start, the kernel holds
the speakers at its floor, and the journal lists every reason. For tests,
`--root DIR` (or `SGV2_PROVISION_ROOT`) moves every file it writes under a
test root, and `--bundle` names another tuning source.

## 5. The fleet gate (`speakerguardd-gate`)

Distributions install v2 on every Mac beside the older daemons
(speakerguardd 0.1 and speakersafetyd). The gate is the `ExecCondition=` of
both, so exactly one stack runs on a board:

- v2 runs when the protection unit the Mac's own tuning names is at parity
  (`gate.json` `units_at_parity`), the board passed an audible run (that
  unit's `qualified` list in `gate.json`, while `require_qualified_board` is
  true), and the kernel has the v2 interface ("Speaker Protection");
- the older daemons run when v2 does not and the kernel has no v2
  interface.

The kernel is the switch: a fleet kernel without the v2 interface keeps
every Mac on the older daemons, the kernel with it moves every gated Mac at
once, and booting the older kernel moves it back. atsp joined
`units_at_parity` when all 117 rig cases matched macOS bit for bit. The
fleet cutover adds lsm2 once it reaches parity and sets
`require_qualified_board` to false: every board with a parity unit. The data
is speakerguardd's own; every distribution ships it unchanged.
`speakerguardd-gate explain` prints the decision.

## 6. The self-check (`speakerguardd-selfcheck`)

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
   `SpeakerInputDataSourceOrder`. There is no hand map to fall back on:
   a refusal fails the check.

   The rest of the machine description comes from the same device tree
   (`wiring.describe`): the machine key (`apple,<key>-macaudio`), the ALSA
   card id macaudio derives from the model ("Apple" and its last word), the
   amp and sense counts, the LV2 variant, and from the speaker link's CPU
   DAI driver (MCA, the T6040 AOP speaker service, the T8140 AOP) the
   speaker format and the PipeWire buffering. The check fails if ALSA's card
   id or the format the speaker PCM takes disagree.
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
     "Speaker Protection" itself: one grant for the whole switch test,
     renewed as a 50 ms lease only while its own stream owns the speaker
     PCM. The kernel logs one grant and one release for the check. It keeps
     both PCMs to itself and tells the session helper not to link.
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

On every Mac, known or new:

- `speakerguardd-selfcheck generate` writes the PipeWire and WirePlumber
  files from the record;
- the session helper reads the card and channel counts from the record;
- the supervisor builds its machine description from the record: SN012776
  amps (proven, `method=derived`, checked against their gain readback) and
  MAX98360A pairs (`method=derived-static`; their gain is a strap no
  software can read, so the blob's channels must share the tuning's own
  full scale).

The check takes about 13 s on the J316 (13 captures of 0.4 s) and about 8 s
on the J413. A boot that has a valid record only checks the binding.

## 7. What the check can and cannot prove per path

- **MCA Macs (J316, J413) and the AOP path (J616s; J713 by the same
  driver):** derived and proven: regmap slots and the switch test
  (`method=derived`). On the J616s the derivation gives `[I, V]` per amp
  from the device tree alone; the template before 2c16d1c (`vi` by default)
  now fails the check. Its silent switch run on the hardware is still to
  do.
- **J700 (MAX98360A, no V/I):** derived, not proven (`method=derived-static`).
  It has no amp registers and no sense, and each amp's side is its SD_MODE
  strap, which no software can read. A zero-sample stream shows nothing.
  The SpeakerTap returns the words on the wire, so a marker on it could
  prove the PCM-to-frame order (not the strap), but only with non-zero
  samples. That is not done without an explicit OK.
- **The voicing graph.** The WirePlumber file still points at the interim
  asahi-audio graph for the machine. A new board has no such graph, so its
  protect stage runs, but nothing feeds it until the voicing is generated
  from the tuning bundle (lane B).
- **Other amp families.** TAS2770 (M1) has no register check and no
  operating-point check in the supervisor.
- **Not derivable from the device tree.** The amp gain is macaudio's
  per-configuration `amp_gain`, not a device-tree property: offline nothing
  can check it, and the supervisor checks it on the card. A new transport
  driver (a CPU DAI compatible not in `wiring.py` TRANSPORTS) is refused
  until its formats and ring behaviour are added there.

## 8. Checklist for a new Mac

1. Kernel: add the `macaudio` match entry and the board's device tree with
   the amps (section 2).
2. Confirm the unit is spp3. If it is lsm2 or atsp, wait for that unit's
   parity.
3. Confirm the amp count has an LV2 variant (2, 4 or 6), and the speaker
   link's CPU DAI driver is in `wiring.py` TRANSPORTS.
4. After its audible run, add the board to its unit's `qualified` list in
   `gate.json`; once `require_qualified_board` is false, nothing.
6. Everything else is automatic: the installer collects the tuning,
   provisioning compiles and binds it and configures the units, the
   self-check derives and proves the wiring at first boot, and the graph is
   generated from the self-check's record.
