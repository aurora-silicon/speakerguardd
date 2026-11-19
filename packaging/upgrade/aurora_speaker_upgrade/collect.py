# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""This Mac's speaker tuning bundle, collected from Linux.

The bundle is the installer's own: the installer's collector
(asahi_firmware.audio_tuning, vendored unchanged) builds it. This module
only gives that collector what it reads on macOS:

- the Tunings tree and the AudioDSP component, staged read-only from a
  macOS System volume: the installed one (route 1, apfs_ro on the APFS
  container) or the restore image's (route 2, apfs_ro over aea_ro);
- the answers of `ioreg` and `sysctl`, from the ADT the boot chain exposes
  (acoustic-id, speaker-thiele-small, target-type, serial-number), the
  System volume's SystemVersion.plist (the build), and the speaker ID from
  the boot chain (/chosen/asahi,speaker-id) or from this unit's own
  installer bundle. Nothing is guessed: no source, no speaker ID.
"""
import json
import logging
import os
import plistlib
import shutil
import stat
import subprocess

from . import adt_ro, apfs_ro
from .asahi_firmware import audio_tuning, lsm2_lattice, lsm2_fill

log = logging.getLogger("aurora-speaker-upgrade")

TUNINGS = audio_tuning.TUNINGS_ROOT.lstrip("/")
SYSTEM_VERSION = "System/Library/CoreServices/SystemVersion.plist"
CHOSEN_SPID = "/proc/device-tree/chosen/asahi,speaker-id"
DT_SERIAL = "/proc/device-tree/serial-number"
APFS_GUID = "7c3457ef-0000-11aa-aa11-00306543ecac"
STAGE_BUDGET = audio_tuning.MAX_BYTES + lsm2_lattice.MAX_COMPONENT_BYTES


class CollectError(Exception):
    pass


class SpeakerIdUnknown(CollectError):
    """The AID selects by speaker ID and no trusted source gives it."""


# --- the Mac's identity ---------------------------------------------------

class Identity:
    """What macOS's IORegistry would say, from the ADT."""

    def __init__(self, adt_path=None):
        try:
            root = adt_ro.load(adt_path)
        except (OSError, adt_ro.AdtError) as e:
            raise CollectError(f"cannot read the ADT: {e}") from None
        self.target_type = root["props"].get("target-type")
        self.serial = root["props"].get("serial-number")
        audio = [n for _, n in adt_ro.find(root, "audio")
                 if isinstance(n["props"].get("acoustic-id"), bytes)]
        if len(audio) != 1:
            raise CollectError(f"the ADT has {len(audio)} audio nodes with an acoustic-id")
        self.acoustic_raw = audio[0]["props"]["acoustic-id"]
        self.spts = audio[0]["props"].get("speaker-thiele-small")
        self.acoustic = audio_tuning.acoustic_id(self.acoustic_raw)
        if self.acoustic is None:
            raise CollectError("malformed acoustic-id in the ADT")
        self.board = audio_tuning.board_name(self.target_type)
        self.unit = audio_tuning.unit_id(self.serial)

    def matches_dt(self, dt_serial=DT_SERIAL):
        """The Linux device tree names the same unit (aura copies it)."""
        try:
            with open(dt_serial, "rb") as f:
                return audio_tuning.unit_id(f.read()) == self.unit
        except OSError:
            return False


# The HAL's invalid strap value. aura (aura#121) publishes it in
# /chosen/asahi,speaker-id when the strap read is invalid, as macOS's
# IORegistry reports it; the installer keeps it as reported in a bundle. It
# is never a vendor index: it counts as no speaker ID.
INVALID_SPEAKER_ID = 0xFFFF


def valid_speaker_id(value):
    return type(value) is int and 0 <= value <= 0xFFFFFFFF and value != INVALID_SPEAKER_ID


def chosen_speaker_id(path=CHOSEN_SPID):
    """/chosen/asahi,speaker-id (one big-endian u32 cell), or None (absent,
    malformed, or the invalid strap value 65535)."""
    try:
        with open(path, "rb") as f:
            raw = f.read(16)
    except OSError:
        return None
    if len(raw) != 4:
        return None
    value = int.from_bytes(raw, "big")
    return value if valid_speaker_id(value) else None


def selects_by_speaker_id(database_entry):
    """Whether this AID's database entry selects anything by `spid`."""
    def walk(n):
        if isinstance(n, dict):
            if n.get("Selector") == "spid":
                return True
            return any(walk(v) for v in n.values())
        if isinstance(n, list):
            return any(walk(v) for v in n)
        return False
    return walk(plistlib.loads(database_entry))


# --- route 0: an installer bundle already on this Mac ----------------------

INSTALLER_BUNDLES = ("/lib/firmware/vendor/apple/audio-tuning",
                     "/run/aurora-firmware/vendorfw/apple/audio-tuning")


def installer_bundle(identity, dirs=INSTALLER_BUNDLES):
    """(directory, bundle dict) of a verified installer bundle bound to this
    unit, or (None, reason)."""
    reasons = []
    for d in dirs:
        if not os.path.isfile(os.path.join(d, audio_tuning.MANIFEST)):
            continue
        c = audio_tuning.AudioTuningFWCollection.__new__(audio_tuning.AudioTuningFWCollection)
        c.fwfiles, c.bundle = [], {}
        try:
            c.update(d)
        except Exception as e:
            reasons.append(f"{d}: {e}")
            continue
        if not c.bundle:
            reasons.append(f"{d}: empty")
            continue
        m = json.loads(c.bundle[audio_tuning.MANIFEST])
        if m.get("unit_id") is None or m.get("unit_id") != identity.unit:
            reasons.append(f"{d}: collected on {'an unnamed unit' if m.get('unit_id') is None else 'another unit'}")
            continue
        if m.get("board") != identity.board or m.get("acoustic_id") != identity.acoustic:
            reasons.append(f"{d}: board or acoustic-id differ from this Mac's")
            continue
        if m.get("lsm2_mask_lattice") is None:
            reasons.append(f"{d}: collected before the installer read the lsm2 lattice")
            continue
        if lsm2_fill.needs_fill(c.bundle) and m.get("lsm2_param_fill") is None:
            reasons.append(f"{d}: collected before the installer read the lsm2 fill; "
                           "route 3: aurora-speaker-upgrade guide")
            continue
        if not valid_speaker_id(m.get("speaker_id")) \
                and selects_by_speaker_id(c.bundle[audio_tuning.DATABASE]):
            # Its tuning selects by speaker ID, and the bundle has none (or the
            # invalid strap value): the other routes, with the boot chain's ID.
            reasons.append(f"{d}: no valid speaker ID, and AID{identity.acoustic} selects by it")
            continue
        return d, c.bundle
    return None, "; ".join(reasons) or "no installer bundle on this Mac"


# --- System volumes ----------------------------------------------------------

def apfs_partitions(run=subprocess.run):
    p = run(["lsblk", "-J", "-p", "-o", "PATH,PARTTYPE"], capture_output=True, text=True)
    if p.returncode != 0:
        return []
    out = []

    def walk(devs):
        for d in devs:
            if (d.get("parttype") or "").lower() == APFS_GUID:
                out.append(d["path"])
            walk(d.get("children") or [])
    walk(json.loads(p.stdout).get("blockdevices", []))
    return out


def system_volumes(acoustic, devices=None):
    """[(build, device, volume)] of every readable macOS System volume that
    has this Mac's AID<n> tuning folder."""
    found = []
    for dev in devices if devices is not None else apfs_partitions():
        try:
            c = apfs_ro.Container(dev)
            vols = [v for v in c.volumes() if v.role == apfs_ro.ROLE_SYSTEM and not v.encrypted]
        except (OSError, apfs_ro.ApfsError) as e:
            log.info(f"{dev}: not readable as APFS ({e})")
            continue
        for v in vols:
            try:
                v.resolve(f"{TUNINGS}/AID{acoustic}")
                build = plistlib.loads(v.read_file(v.resolve(SYSTEM_VERSION)[0]))["ProductBuildVersion"]
            except (apfs_ro.ApfsError, KeyError, ValueError) as e:
                log.info(f"{dev} {v.name}: no tuning for AID{acoustic} ({e})")
                continue
            found.append((build, dev, v))
    return found


def build_key(build):
    """25C56 -> (25, 'C', 56, ''): newer macOS builds sort later."""
    import re
    m = re.match(r"\A(\d+)([A-Z])(\d+)([a-z]?)\Z", build or "")
    return (int(m[1]), m[2], int(m[3]), m[4]) if m else (0, "", 0, build or "")


# --- staging ------------------------------------------------------------------

class Stager:
    """Copies paths out of a System volume into a private directory, keeping
    links inside it as links (the collector resolves them as on macOS)."""

    def __init__(self, vol, dest):
        self.vol, self.dest, self.total = vol, dest, 0

    def _target(self, rel):
        full = os.path.join(self.dest, rel)
        os.makedirs(os.path.dirname(full), mode=0o700, exist_ok=True)
        return full

    def file(self, rel, missing_ok=True):
        try:
            ino, kind = self.vol.resolve(rel)
        except apfs_ro.ApfsError:
            if missing_ok:
                return False
            raise
        self._node(ino, kind, rel)
        return True

    def tree(self, rel):
        ino, kind = self.vol.resolve(rel)
        self._node(ino, kind, rel)

    def _node(self, ino, kind, rel):
        full = self._target(rel)
        if kind == 4:
            os.makedirs(full, mode=0o700, exist_ok=True)
            for name, (cino, ck) in sorted(self.vol.children(ino).items()):
                if "/" in name or name in (".", "..") or "\0" in name:
                    continue
                self._node(cino, ck, f"{rel}/{name}")
        elif kind == 8:
            data = self.vol.read_file(ino)
            self.total += len(data)
            if self.total > STAGE_BUDGET:
                raise CollectError("the staged tuning is over budget")
            fd = os.open(full, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, "wb") as f:
                f.write(data)
        elif kind == 10:
            os.symlink(self.vol.readlink(ino), full)


def stage(vol, acoustic, dest):
    """Everything the installer's collector reads, from one System volume."""
    s = Stager(vol, dest)
    t = TUNINGS
    s.tree(f"{t}/AID{acoustic}")
    s.file(f"{t}/{audio_tuning.DATABASE}")
    s.file(f"{t}/{audio_tuning.config_name(acoustic)}")
    s.file(f"{t}/{audio_tuning.LOUDNESS}")
    s.file(SYSTEM_VERSION, missing_ok=False)
    s.file(lsm2_lattice.AUDIODSP, missing_ok=False)
    # The shared files the speaker modes reference (Common/), as the
    # collector will select them from the database entry.
    entry = database_entry(dest, acoustic)
    if entry is not None:
        for path in sorted(audio_tuning.speaker_references(entry, acoustic)[1]):
            s.file(f"{t}/{path}")
    return s.total


def database_entry(root, acoustic):
    tun = os.path.join(root, TUNINGS)
    own = os.path.join(tun, audio_tuning.aid_database(acoustic))
    shared = os.path.join(tun, audio_tuning.DATABASE)
    for p in (own, shared):
        if os.path.lexists(p):
            data = audio_tuning.read_bounded(p, audio_tuning.MAX_DATABASE_BYTES)
            if p == own and audio_tuning.only_entry(data, acoustic):
                return data
            e = audio_tuning.database_entry(data, acoustic)
            if e is not None:
                return e
    return None


# --- the collector, fed from Linux --------------------------------------------

class FakeRun:
    """Answers the collector's three `ioreg` queries and its `sysctl`, from
    the ADT, the staged SystemVersion.plist and the speaker ID."""

    def __init__(self, identity, build, speaker_id):
        self.identity, self.build, self.speaker_id = identity, build, speaker_id

    def __call__(self, argv, capture_output=True, **kw):
        out = None
        if argv[:1] == ["ioreg"] and "-k" in argv and argv[argv.index("-k") + 1] == "acoustic-id":
            node = {"acoustic-id": self.identity.acoustic_raw}
            if self.identity.spts is not None:
                node["speaker-thiele-small"] = self.identity.spts
            out = plistlib.dumps([node], fmt=plistlib.FMT_XML)
        elif argv[:1] == ["ioreg"] and "device-tree" in argv:
            node = {}
            if self.identity.target_type is not None:
                node["target-type"] = self.identity.target_type
            if self.identity.serial is not None:
                node["serial-number"] = self.identity.serial
            out = plistlib.dumps([node], fmt=plistlib.FMT_XML)
        elif argv[:1] == ["ioreg"] and audio_tuning.SPEAKER_SERVICE in argv:
            svc = [] if self.speaker_id is None else [{"device UID": "Speaker", "speakerID": self.speaker_id}]
            out = plistlib.dumps(svc, fmt=plistlib.FMT_XML) if svc else b""
        elif argv[:3] == ["sysctl", "-n", "kern.osversion"]:
            out = self.build.encode() + b"\n"
        if out is None:
            return subprocess.CompletedProcess(argv, 1, b"", b"not answered on Linux")
        return subprocess.CompletedProcess(argv, 0, out, b"")


def collect(staged, identity, speaker_id, speaker_id_known):
    """The installer's bundle (relative path -> bytes) from a staged System
    volume. Refuses an AID that selects by speaker ID without a known one."""
    acoustic = identity.acoustic
    entry = database_entry(staged, acoustic)
    if entry is None:
        raise CollectError(f"the DSP database has no entry for AID{acoustic}")
    if not speaker_id_known and selects_by_speaker_id(entry):
        raise SpeakerIdUnknown(f"AID{acoustic} selects its speaker tuning by speaker ID, and neither "
                               "the boot chain nor an installer bundle of this unit gives it")
    with open(os.path.join(staged, SYSTEM_VERSION), "rb") as f:
        build = plistlib.load(f)["ProductBuildVersion"]
    run = FakeRun(identity, build, speaker_id)
    c = audio_tuning.AudioTuningFWCollection.__new__(audio_tuning.AudioTuningFWCollection)
    c.fwfiles, c.bundle = [], {}
    # load() directly: the constructor turns every failure into an empty
    # collection, and the caller needs the reason.
    c.load(os.path.join(staged, TUNINGS), run, os.path.join(staged, lsm2_lattice.AUDIODSP))
    if not c.bundle:
        raise CollectError("the installer's collector produced no bundle")
    return c.bundle, build


def write_bundle(bundle, dest):
    """Writes a bundle tree (0700 directories, 0600 files) at dest, which
    must not exist."""
    os.makedirs(dest, mode=0o700)
    for rel, data in sorted(bundle.items()):
        if not audio_tuning.admitted(rel):
            raise CollectError(f"inadmissible bundle path {rel!r}")
        full = os.path.join(dest, rel)
        os.makedirs(os.path.dirname(full), mode=0o700, exist_ok=True)
        fd = os.open(full, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, "wb") as f:
            f.write(data)


def compare(a, b):
    """[differences] between two bundles (relative path -> bytes)."""
    diffs = [f"only in the first: {p}" for p in sorted(set(a) - set(b))]
    diffs += [f"only in the second: {p}" for p in sorted(set(b) - set(a))]
    diffs += [f"differs: {p}" for p in sorted(set(a) & set(b)) if a[p] != b[p]]
    return diffs


def rmtree(path):
    if os.path.lexists(path):
        shutil.rmtree(path)


ROLE_PREBOOT = 0x10
# Where Preboot records each macOS's build: the restore record of the
# installed system, then its cryptex.
PREBOOT_VERSIONS = ("restore/SystemVersion.plist", "cryptex1/current/SystemVersion.plist")


def preboot_build(dev):
    """The installed macOS build as its Preboot volume records it (the
    volume is never encrypted): <group>/restore/SystemVersion.plist; the
    newest when several macOS share the container."""
    try:
        c = apfs_ro.Container(dev)
        vols = [v for v in c.volumes() if v.role == ROLE_PREBOOT and not v.encrypted]
    except (OSError, apfs_ro.ApfsError):
        return None
    builds = []
    for v in vols:
        try:
            groups = v.children(v.resolve("/")[0])
        except apfs_ro.ApfsError:
            continue
        for name, (ino, kind) in groups.items():
            if kind != 4:
                continue
            for rec in PREBOOT_VERSIONS:
                try:
                    data = v.read_file(v.resolve(f"{name}/{rec}")[0])
                    builds.append(plistlib.loads(data)["ProductBuildVersion"])
                    break
                except (apfs_ro.ApfsError, KeyError, ValueError):
                    continue
    return max(builds, key=build_key) if builds else None
