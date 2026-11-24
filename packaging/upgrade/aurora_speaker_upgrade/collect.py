# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""This Mac's speaker tuning bundle, built on Linux from the pinned restore image.

The bundle is the installer's own: the installer's collector
(asahi_firmware.audio_tuning, vendored unchanged) builds it. This module
only gives that collector what it reads on macOS:

- the Tunings tree and the AudioDSP component, staged read-only from the
  System volume of the pinned macOS restore image (ipsw.py: apfs_ro over
  aea_ro, by HTTP ranges). Never from an installed macOS;
- the answers of `ioreg` and `sysctl`: this unit's own values from unit.py
  (acoustic-id, speaker-thiele-small, target-type, serial-number, speaker
  ID) and the restore image's SystemVersion.plist (the build). Nothing is
  guessed: no source, no speaker ID.
"""
import json
import logging
import os
import plistlib
import shutil
import subprocess

from . import apfs_ro, unit
from .asahi_firmware import audio_tuning, lsm2_lattice, lsm2_fill

log = logging.getLogger("aurora-speaker-upgrade")

TUNINGS = audio_tuning.TUNINGS_ROOT.lstrip("/")
SYSTEM_VERSION = "System/Library/CoreServices/SystemVersion.plist"
CHOSEN_SPID = unit.CHOSEN_SPID
INSTALLER_BUNDLES = unit.INSTALLER_BUNDLES
STAGE_BUDGET = audio_tuning.MAX_BYTES + lsm2_lattice.MAX_COMPONENT_BYTES


# One error for both: a unit value the ADT cannot give refuses like a
# tuning the restore image cannot give.
CollectError = unit.UnitError


class SpeakerIdUnknown(CollectError):
    """The AID selects by speaker ID and no trusted source gives it."""


# --- the Mac's identity ---------------------------------------------------

Identity = unit.Identity
valid_speaker_id = unit.valid_speaker_id
chosen_speaker_id = unit.chosen_speaker_id
INVALID_SPEAKER_ID = unit.INVALID_SPEAKER_ID


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
    # The BuzzKill detector's quadrature matrix too: speakerguardd v2 reads
    # it, and a tuning with a Buzz box does not compile without it. The
    # collector reads it from the staged AudioDSP component; a component
    # without it gives a bundle without the record (fine for a graph with
    # no Buzz box; the compile refuses one that has it).
    c.collect_buzzkill = True
    # load() directly: the constructor turns every failure into an empty
    # collection, and the caller needs the reason.
    c.load(os.path.join(staged, TUNINGS), run, os.path.join(staged, lsm2_lattice.AUDIODSP))
    if not c.bundle:
        raise CollectError("the installer's collector produced no bundle")
    return c.bundle, build


# The BuzzKill unit (`aufx/buzk`), as the tuning's strips and graphs name an
# Audio Unit: a dictionary whose `type` and `subtype` are FourCCs, either as
# integers or as four-character strings.
BUZZ_UNIT = ("aufx", "buzk")


def _fourcc(v):
    if isinstance(v, int) and 0 <= v <= 0xFFFFFFFF:
        return v.to_bytes(4, "big").decode("latin-1")
    return v if isinstance(v, str) else None


def has_buzz_unit(bundle):
    """Whether any property list of the bundle names a BuzzKill unit: its
    graph then needs the BuzzKill quadrature matrix to compile."""
    def walk(n):
        if isinstance(n, dict):
            if (_fourcc(n.get("type")), _fourcc(n.get("subtype"))) == BUZZ_UNIT:
                return True
            return any(walk(v) for v in n.values())
        if isinstance(n, list):
            return any(walk(v) for v in n)
        return False
    for rel, data in bundle.items():
        if rel == audio_tuning.MANIFEST or not data.startswith((b"bplist", b"<?xml", b"<plist")):
            continue
        try:
            doc = plistlib.loads(data)
        except Exception:
            continue
        if walk(doc):
            return True
    return False


# Manifest records a newer installer writes that this speakerguardd does
# not read yet, and the file each one names. Its tuning reader parses a
# schema-2 manifest strictly: one key it does not know refuses the whole
# bundle ("unknown field"), and the speakers stay at the kernel's floor.
# Add an entry here when the installer's collector starts writing a record
# before speakerguardd reads it; remove it once the reader does.
# `buzzkill_quadrature` was the first (aurora-installer#119); speakerguardd
# reads it since #244, so it is kept.
UNREAD_RECORDS = {}


def for_this_reader(bundle):
    """(bundle, [record names dropped]): the bundle without the records this
    speakerguardd does not read (UNREAD_RECORDS) and their files, every
    other file and manifest entry unchanged. The manifest is written again
    the way the installer's collector writes it (sorted keys, two-space
    indent), so the result is the bundle the collector writes without
    those records. A bundle without any is returned as is."""
    m = json.loads(bundle[audio_tuning.MANIFEST])
    dropped = [k for k in UNREAD_RECORDS if k in m]
    if not dropped:
        return bundle, []
    out = dict(bundle)
    for k in dropped:
        rec = m.pop(k)
        if not isinstance(rec, dict) or rec.get("path") != UNREAD_RECORDS[k]:
            raise CollectError(f"malformed {k} record in the bundle's manifest")
        out.pop(UNREAD_RECORDS[k], None)
    out[audio_tuning.MANIFEST] = json.dumps(m, indent=2, sort_keys=True).encode("utf-8")
    return out, dropped


def component_sha256(staged):
    """SHA-256 of the staged AudioDSP component (the restore image's)."""
    import hashlib
    with open(os.path.join(staged, lsm2_lattice.AUDIODSP), "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def attach_fill(bundle, fill, component):
    """The bundle with this unit's lsm2 parameter readback, or a reason it
    does not fit (CollectError).

    The readback runs Apple's AudioUnit, which only a running macOS can do,
    so it cannot come from the restore image. It is taken only when it was
    read from exactly this bundle: the same AID, build and unit, a
    configuration digest over byte-identical strips, propstrips and property
    lists, and the same AudioDSP component (SHA-256) as the restore image's.
    The tuning itself always stays the restore image's."""
    m = json.loads(bundle[audio_tuning.MANIFEST])
    aid, build, own = m.get("aid"), m.get("macos_build"), m.get("unit_id")
    try:
        rec = audio_tuning.fill_record(fill, aid, build, own)
        lsm2_fill.validate_configuration(fill, bundle, aid)
        comps = {c["component_sha256"] for c in json.loads(fill)["presets"]}
    except (ValueError, KeyError, TypeError) as e:
        raise CollectError(f"lsm2 readback does not fit the restore image's {aid}: {e}") from None
    if comps != {component}:
        raise CollectError(f"lsm2 readback was taken from another AudioDSP component than macOS {build}'s")
    out = dict(bundle)
    out[lsm2_fill.NAME] = fill
    m["lsm2_param_fill"] = rec
    out[audio_tuning.MANIFEST] = audio_tuning.encode_manifest(m)
    return out


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
