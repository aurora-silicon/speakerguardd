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
aurora-speaker-upgrade guide           # the macOS route, when Linux cannot read the tuning
```

## What it does

1. **Tuning bundle.** The bundle is the installer's own. The installer's
   collector (`asahi_firmware.audio_tuning`, vendored unchanged, see
   `aurora_speaker_upgrade/asahi_firmware/__init__.py`) builds it from the
   first source that works:
   0. an installer bundle already on the Mac, bound to this unit
      (`unit_id`);
   1. the installed macOS's System volume. `apfs_ro` reads it from Linux,
      read only, straight from the block device. The volume is not encrypted,
      whether FileVault is on or off; only Data is.
   2. Apple's restore image (IPSW) of that macOS build. `aea_ro` reads it by
      HTTP ranges, about 20 MB of a 19 GB image, and decrypts it with the key
      Apple publishes for it. `aea_ro` is a port of the installer's AEA range
      path (its pinned reference, macos-firmware-extract@71b413af).
      `tests/` checks it against that reference's vectors. The build comes
      from the Preboot volume, which is also not encrypted.
   3. a collection made on the Mac's own macOS with the installer's
      `tools/speaker-tuning/collect-macos.sh` (`--collection`).

   The ADT the boot chain exposes as the `adt` MTD partition answers what
   the collector asks macOS's IORegistry for: acoustic-id,
   speaker-thiele-small, target-type and the serial number (only its hash
   is kept). The speaker ID comes from the boot chain
   (`/chosen/asahi,speaker-id`) or from this unit's installer bundle. If the
   tuning selects by speaker ID and neither source has it, the tool refuses
   and points to route 3. Nothing is guessed.
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
   `/var/lib/aurora-speaker-upgrade/interface`, and the
   `50-aurora-speaker-upgrade.conf` drop-in applies it before the v2
   supervisor starts at every boot. On kernels with only the v2 interface
   (fleet kernels), the switch only restarts the v2 supervisor. On kernels
   with only the legacy interface, the tool refuses the switch. If v2 does
   not take the speakers within 30 s, the tool goes back to the legacy
   protection. The legacy stack always stays installed.

Exit codes: 0 done (or nothing to do), 1 refused (the legacy protection
keeps the speakers), 2 not now (retry), 3 internal error (nothing switched).

Requirements: python3, python3-cryptography, liblzfse, util-linux
(lsblk), alsa-utils, systemd, and speakerguardd v2 for the switch.
