# aurora-speaker-upgrade

Puts an existing install's speakers on speakerguardd v2 with the Mac's own
tuning. speakerguardd v2 is the only speaker protection: this tool never
starts, enables or hands the speakers to an older daemon (speakersafetyd,
speakerguardd 0.1), and never writes the kernel's interface back to the
volume lock ("Speaker Volume Unlock"). Where v2 does not run, no speaker
daemon runs and the kernel's own limit holds the speakers.

```sh
sudo aurora-speaker-upgrade            # collect, compile, self-check, switch (idempotent)
sudo aurora-speaker-upgrade status
sudo aurora-speaker-upgrade --check [--json F]   # dry run: collect, compile, self-check in /run; nothing stored or switched
```

On a dual kernel `--check` lends the speakers to the silent switch test
for a few seconds; an older speaker daemon still running there is stopped
and stays stopped.

`--revert` and `--purge` are gone: there is no older protection to go back
to. Both are refused (exit 1) with that reason and change nothing, so an old
script fails loudly instead of silently doing something else. To stop v2 by
hand, stop its unit (`systemctl stop speakerguardd`, `speakerguardd-v2` in
the distributions): the kernel's own limit then holds the speakers. The next
boot starts it again wherever the gate qualifies the board.

## What it does

1. **Tuning bundle.** The bundle is the installer's own. The installer's
   collector (`asahi_firmware.audio_tuning`, vendored unchanged, see
   `aurora_speaker_upgrade/asahi_firmware/__init__.py`) builds it from one
   source only: Apple's restore image (IPSW) of the pinned macOS release.
   The pin, `share/restore-images.json`, names the release (macOS 27.2, the
   speaker safety stack's, for every Mac whatever macOS it runs) and its
   restore images in order; the first one whose BuildManifest carries this
   Mac's board is used, and its build must be a pinned one. `aea_ro` reads
   it by HTTP ranges, about 26 MB of a 19 GB image, and decrypts it with the
   key Apple publishes for it (a port of the installer's AEA range path,
   macos-firmware-extract@71b413af; `tests/` checks it against that
   reference's vectors). The reader (`ipsw`, `aea_ro`, `apfs_ro`,
   `remotezip`) is the installer's `asahi_firmware`, vendored unchanged, so
   the installer's Linux tools and this tool read restore images with one
   implementation. `apfs_ro` reads its System volume: the Tunings
   tree and the AudioDSP component (lsm2 lattice, BuzzKill quadrature
   matrix). `--ipsw URL` reads the same pinned image from a mirror.
   There is no other source: not the installed macOS, not an older bundle,
   not another build. If no pinned image carries the board, or it cannot be
   read, the tool refuses and the speakers stay as they are.

   Only this unit's own values come from the Mac, all in `unit.py`: the ADT
   the boot chain exposes as the `adt` MTD partition (acoustic-id,
   speaker-thiele-small, target-type, the serial number, of which only its
   hash is kept), the speaker ID from the boot chain
   (`/chosen/asahi,speaker-id`) or from this unit's installer bundle, and
   the lsm2 parameter readback described next. If the tuning selects by
   speaker ID and neither source has it, the tool refuses. Nothing is
   guessed.

   Short lsm2 presets need the parameter readback (`lsm2-param-fill.bin`,
   manifest `lsm2_param_fill`) that the installer takes by hosting the
   AudioUnit on this Mac's macOS; a restore image cannot run it. It is
   taken from this unit's installer bundle only when it was read from
   exactly the restore image's files: the same AID, build and unit, the
   same configuration digest over the strips, propstrips and property
   lists, and the same AudioDSP component (SHA-256). Otherwise the tool
   refuses. `status` reports a missing readback. Full presets need none.
   A bundle that carries a record speakerguardd v2 does not read yet
   (`UNREAD_RECORDS` in `collect.py`; none today) is stored without that
   record and its file, and the tool says so: the tuning reader refuses any
   manifest key it does not know.
2. **Storage.** The bundle goes to `/var/lib/aurora-speaker-upgrade/bundle`
   (0700, root). It is never packaged or sent anywhere.
   `/etc/speakerguardd/provision.conf` points provisioning at it
   (`SGV2_BUNDLE`).
3. **Compile and self-check.** speakerguardd v2 compiles the bundle. Its
   self-check (`speakerguardd-selfcheck verify`) then proves the result
   silently. Exit 0 lets the switch go ahead, 1 refuses it, 2 means retry
   later, and 3 is treated as a refusal.
4. **Switch, only on a pass.** On kernels with the interlock, the tool
   writes "Speaker Protection" to the kernel's "Speaker Protection
   Interface" control. Its setting lives in
   `/var/lib/aurora-speaker-upgrade/interface`, and the v2 supervisor's
   own `ExecStartPre=speakerguardd-gate interface` applies it before every
   start (a refused switch fails the start). On kernels with only the v2 interface
   (fleet kernels), the switch only restarts the v2 supervisor. On kernels
   without the v2 interface, the tool refuses the switch. Any older
   speaker daemon still running on a dual kernel is stopped before the
   switch and never started again.
5. **A switch that does not go through** (a refusal, or v2 not taking the
   speakers within 30 s) removes the setting and provisioning's pointer to
   the bundle and stops v2; the seat user's session comes back on the
   distribution's graph. No speaker daemon takes over: the kernel returns
   to its volume lock by itself 30 s after the last lease closes, at its
   own limit. On a fleet kernel v2 restarts as it ran before (the
   installer's bundle, where that qualifies), or nothing runs.

Exit codes: 0 done (or nothing to do), 1 refused (nothing switched; no older
daemon starts), 2 not now (retry), 3 internal error (nothing switched).

The boot service runs after `multi-user.target` and any scheduled Aurora
Stage 2 commit. A slow restore image fetch can then finish independently
of the Stage 2 rollback deadline.

At boot (`aurora-speaker-upgrade.service`, `run --at-boot`), a refusal is
recorded in `/var/lib/aurora-speaker-upgrade/refused.json` and not tried
again until the kernel, the live speaker wiring or sound modules,
speakerguardd v2's provisioning or self-check, the
parity attestation or this tool changes; the unit counts a refusal as an
outcome, not a failure. `run` by hand always tries again.

A boot run waits up to 120 s for the speaker card, because the card's
driver can bind after `sound.target` (on a J413 the card registered a
second after the run). It waits only while the live device tree describes
a speaker card. If the tree has none (no enabled `apple,macaudio` node
with a Speaker link), the run stops at once with "not now" and says that
this kernel's device tree lacks the speaker amp nodes. The speakers stay
at the kernel's own limit, and the next boot on a kernel and boot update
that carries the nodes tries again. A boot run also keeps starting new
attempts at the pinned restore image every 5 s for 180 s (a retry-start
budget; an attempt started in it runs under the reader's own HTTP
timeouts): on a J413 `network-online.target` was reached before DNS
answered. Only "cannot be read now" is tried again; any answer from the
image is final. After the fetch the run checks that the kernel still
serves the same speaker card (number, ALSA id, interface) before it
changes provisioning or the protection switch; a reprobed or renumbered
card is "not now", with nothing changed. A run by hand
does not wait for either.

The boot unit also moves a speaker only once protected playback on it is on
record. speakerguardd v2's gate data (`packaging/session/gate.json`,
installed as `/usr/share/speakerguardd/gate.json`) lists in `playback_rows`,
by the tuning's speaker identity (`AID<n>`), the rows where sound was played
through v2's protection on that speaker and measured with V/I sense and the
Mac's own microphone. A speaker counts once its rows cover every item of
`playback_covers` (a loud step and a thermal ramp). Without that,
`run --at-boot` leaves the speakers as they are (what the installer set
up, or the kernel's own limit), collects nothing and records no refusal; a
silent self-check pass alone is not playback evidence. `run` by hand is not
limited by the record (v2's provisioning still refuses a protection type
the gate data holds for playback rows, LSID's static release), and `status`
shows `boot_migration`. A new playback row makes the boot unit try a
refused Mac again.

Requirements: python3, python3-cryptography, liblzfse, alsa-utils,
systemd, network access to Apple's CDN and key service, and speakerguardd
v2 (the same release) for the switch.
