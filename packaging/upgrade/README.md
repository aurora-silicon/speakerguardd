# aurora-speaker-upgrade

Moves an existing install's speakers from the legacy protection
(speakersafetyd, speakerguardd 0.1) to speakerguardd v2 with the Mac's own
tuning, and back. It ships **disabled**. Turning it on for users waits for
the maintainers.

```sh
sudo aurora-speaker-upgrade            # collect, compile, self-check, switch (idempotent)
sudo aurora-speaker-upgrade status
sudo aurora-speaker-upgrade --check [--json F]   # dry run: collect, compile, self-check in /run; nothing stored or switched
sudo aurora-speaker-upgrade --revert   # back to the legacy protection (--purge drops the bundle)
```

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
   with only the legacy interface, the tool refuses the switch. If v2 does
   not take the speakers within 30 s, the tool goes back to the legacy
   protection. The legacy stack always stays installed.

Exit codes: 0 done (or nothing to do), 1 refused (the legacy protection
keeps the speakers), 2 not now (retry), 3 internal error (nothing switched).

At boot (`aurora-speaker-upgrade.service`, `run --at-boot`), a refusal is
recorded in `/var/lib/aurora-speaker-upgrade/refused.json` and not tried
again until the kernel, speakerguardd v2's provisioning or self-check, the
parity attestation or this tool changes; the unit counts a refusal as an
outcome, not a failure. `run` by hand always tries again.

A boot run waits up to 120 s for the speaker card, because the card's
driver can bind after `sound.target` (on a J413 the card registered a
second after the run). It waits only while the live device tree describes
a speaker card. If the tree has none (no enabled `apple,macaudio` node
with a Speaker link), the run stops at once with "not now" and says that
this kernel's device tree lacks the speaker amp nodes. The speakers stay
at the kernel's own limit, and the next boot on a kernel and boot update
that carries the nodes tries again. A boot run also keeps trying to reach
the pinned restore image for up to 180 s, every 5 s: on a J413
`network-online.target` was reached before DNS answered. Only "cannot be
read now" is tried again; any answer from the image is final. A run by hand
does not wait for either.

The boot unit also moves a speaker only once protected playback on it is on
record. speakerguardd v2's gate data (`packaging/session/gate.json`,
installed as `/usr/share/speakerguardd/gate.json`) lists in `playback_rows`,
by the tuning's speaker identity (`AID<n>`), the rows where sound was played
through v2's protection on that speaker and measured with V/I sense and the
Mac's own microphone. A speaker counts once its rows cover every item of
`playback_covers` (a loud step and a thermal ramp). Without that,
`run --at-boot` leaves the speakers as they are (the legacy protection, or
what the installer set up), collects nothing and records no refusal; a
silent self-check pass alone is not playback evidence. `run` by hand is not
limited by the record (v2's provisioning still refuses a protection type
the gate data holds for playback rows, LSID's static release), and `status`
shows `boot_migration`. A new playback row makes the boot unit try a
refused Mac again.

Requirements: python3, python3-cryptography, liblzfse, alsa-utils,
systemd, network access to Apple's CDN and key service, and speakerguardd
v2 (the same release) for the switch.
