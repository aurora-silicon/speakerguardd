# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""The only values this tool reads from the Mac itself.

Everything per model (the Tunings tree, the DSP database, the AudioDSP
component's lsm2 lattice and BuzzKill matrix) comes from Apple's restore
image (ipsw.py), never from an installed macOS. What is read here differs
from one unit of the same model to the next, so no restore image can carry
it. The allow-list, and why each item is per unit:

- ADT `acoustic-id` (audio node): which tuning this unit's speakers take.
  Units of one board can carry different speaker assemblies.
- ADT `speaker-thiele-small` (audio node): this unit's factory-measured
  speaker parameters (calibration).
- ADT `target-type` and `serial-number` (root): the board name the restore
  image is searched for, and the unit identity a bundle is bound to (only
  the serial number's hash is kept, see audio_tuning.unit_id).
- `/chosen/asahi,speaker-id` (the boot chain reads the speaker strap): which
  vendor's speakers this unit carries, for tunings that select by it.
- The speaker ID and the lsm2 parameter readback recorded in this unit's
  own installer bundle (installer_speaker_id, installer_fill): the speaker
  ID is this unit's strap as macOS reported it; the readback was taken on
  this unit and is used only when it was read from the exact files and
  AudioDSP component of the restore image (collect.attach_fill checks both),
  so it never carries another build's tuning.

Nothing else is read from the Mac.
"""
import json
import os
import struct

from . import adt_ro
from .asahi_firmware import audio_tuning, lsm2_fill

CHOSEN_SPID = "/proc/device-tree/chosen/asahi,speaker-id"
DT_SERIAL = "/proc/device-tree/serial-number"
INSTALLER_BUNDLES = ("/lib/firmware/vendor/apple/audio-tuning",
                     "/run/aurora-firmware/vendorfw/apple/audio-tuning")
# The HAL's invalid strap value. aura (aura#121) publishes it in
# /chosen/asahi,speaker-id when the strap read is invalid, as macOS's
# IORegistry reports it; the installer keeps it as reported in a bundle. It
# is never a vendor index: it counts as no speaker ID.
INVALID_SPEAKER_ID = 0xFFFF


class UnitError(Exception):
    pass


class Identity:
    """What macOS's IORegistry would say about this unit, from the ADT."""

    def __init__(self, adt_path=None):
        try:
            root = adt_ro.load(adt_path)
        except (OSError, adt_ro.AdtError) as e:
            raise UnitError(f"cannot read the ADT: {e}") from None
        self.target_type = root["props"].get("target-type")
        self.serial = root["props"].get("serial-number")
        audio = [n for _, n in adt_ro.find(root, "audio")
                 if isinstance(n["props"].get("acoustic-id"), bytes)]
        if len(audio) != 1:
            raise UnitError(f"the ADT has {len(audio)} audio nodes with an acoustic-id")
        self.acoustic_raw = audio[0]["props"]["acoustic-id"]
        self.spts = audio[0]["props"].get("speaker-thiele-small")
        self.acoustic = audio_tuning.acoustic_id(self.acoustic_raw)
        if self.acoustic is None:
            raise UnitError("malformed acoustic-id in the ADT")
        self.board = audio_tuning.board_name(self.target_type)
        self.unit = audio_tuning.unit_id(self.serial)

    def matches_dt(self, dt_serial=DT_SERIAL):
        """The Linux device tree names the same unit (aura copies it)."""
        try:
            with open(dt_serial, "rb") as f:
                return audio_tuning.unit_id(f.read()) == self.unit
        except OSError:
            return False


def valid_speaker_id(value):
    return type(value) is int and 0 <= value <= 0xFFFFFFFF and value != INVALID_SPEAKER_ID


def chosen_speaker_id(path=CHOSEN_SPID):
    """The speaker ID the boot chain read from the strap, or None."""
    try:
        with open(path, "rb") as f:
            data = f.read(16)
    except OSError:
        return None
    if len(data) != 4:
        return None
    v = struct.unpack(">I", data)[0]
    return v if valid_speaker_id(v) else None


def _own_manifests(identity, dirs):
    for d in dirs:
        try:
            with open(os.path.join(d, audio_tuning.MANIFEST)) as f:
                m = json.load(f)
        except (OSError, ValueError):
            continue
        if isinstance(m, dict) and identity.unit and m.get("unit_id") == identity.unit:
            yield d, m


def installer_speaker_id(identity, dirs=INSTALLER_BUNDLES):
    """(speaker ID, directory) recorded in this unit's installer bundle, or
    (None, None)."""
    for d, m in _own_manifests(identity, dirs):
        if valid_speaker_id(m.get("speaker_id")):
            return m["speaker_id"], d
    return None, None


def speaker_id(identity, chosen=CHOSEN_SPID, dirs=INSTALLER_BUNDLES):
    """(speaker ID or None, known?, where from)."""
    v = chosen_speaker_id(chosen)
    if v is not None:
        return v, True, "the boot chain (/chosen/asahi,speaker-id)"
    v, d = installer_speaker_id(identity, dirs)
    if v is not None:
        return v, True, f"this unit's installer bundle ({d})"
    return None, False, "none"


def installer_fill_build(identity, aid, dirs=INSTALLER_BUNDLES):
    """The macOS build recorded in this unit's installer lsm2 readback for
    this AID, or None. Used only to prefer that build when it is already
    in the pinned restore-image catalog."""
    if not getattr(identity, "unit", None):
        return None
    for _, m in _own_manifests(identity, dirs):
        fill = m.get("lsm2_param_fill")
        if m.get("aid") == aid and isinstance(fill, dict) and isinstance(fill.get("macos_build"), str):
            return fill["macos_build"]
    return None


def installer_fill(identity, aid, dirs=INSTALLER_BUNDLES):
    """[(directory, lsm2 readback bytes)] from this unit's installer bundles
    for this AID; collect.attach_fill decides whether one fits."""
    out = []
    for d, m in _own_manifests(identity, dirs):
        if m.get("aid") != aid or not isinstance(m.get("lsm2_param_fill"), dict):
            continue
        try:
            with open(os.path.join(d, lsm2_fill.NAME), "rb") as f:
                data = f.read(lsm2_fill.MAX_BYTES + 1)
        except OSError:
            continue
        if 0 < len(data) <= lsm2_fill.MAX_BYTES:
            out.append((d, data))
    return out
