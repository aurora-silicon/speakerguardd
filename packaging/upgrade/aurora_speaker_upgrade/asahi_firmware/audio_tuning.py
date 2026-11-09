# SPDX-License-Identifier: MIT
#
# Based on aurora-audio-dsp by saku (MIT): the collection of the machine's
# own AID<n> tuning folder through the device tree's `acoustic-id`, and of the
# per-unit `speaker-thiele-small` calibration, follows saku's
# tools/macos-collector/collect-audio-tuning.sh.
#
# Copyright (c) 2026 saku
# Copyright (c) 2026 The Aurora Silicon Contributors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
"""This Mac's own speaker tuning, for the Linux speaker protection.

The device tree's audio node names the tuning folder (`acoustic-id` 35
selects /System/Library/Audio/Tunings/AID35) and, on Macs with speaker
calibration, carries the per-unit `speaker-thiele-small` record. Both are
copied unchanged, read-only, into the per-machine vendor firmware under
`apple/audio-tuning/`, with a manifest of sizes and SHA-256 hashes, and into
the raw fallback archive under `audio-tuning/`. Nothing is shipped: the data
comes from the user's own system volume and stays on the user's own ESP.

The bundle also carries the lsm2 masker's bucket lattice, read from this
Mac's own AudioDSP component (see lsm2_lattice). A collection without it is
no bundle.

The manifest names the Mac the bundle was collected on: its board (the
device tree root's `target-type`, `J616s` -> `j616s`) and its unit, as the
domain-separated SHA-256 of the root `serial-number` (unit_id). The SpTS
record is that unit's factory calibration; Linux reads the same property from
its device tree (aura copies it from the ADT) and refuses a bundle collected
on another unit. No serial number is stored.

Collection never stops an installation; Linux falls back to its family
profiles without it.
"""
import hashlib, json, logging, os, plistlib, re, shutil, stat, subprocess

from . import lsm2_lattice
from .core import FWFile

log = logging.getLogger("asahi_firmware.audio_tuning")

TUNINGS_ROOT = "/System/Library/Audio/Tunings"
PACKAGE_PREFIX = "apple/audio-tuning/"
FALLBACK_DIR = "audio-tuning"
MANIFEST = "manifest.json"
SPTS = "speaker-thiele-small.bin"
DATABASE = "DeviceDSPDatabase.plist"
# The lsm2 masker's bucket lattice (lsm2_lattice), as its 800 bytes.
LATTICE = "lsm2-mask-lattice.bin"
# Where it is read: the AudioDSP component of the running macOS.
AUDIODSP_PATH = "/" + lsm2_lattice.AUDIODSP


# macOS 27 keeps each Mac's entry in its own folder instead of the shared
# database, with the same shape: one top-level key, AID<n>.
AID_DATABASE = "DSP/DSPDatabase.plist"
# Shared files the speaker graph may reference outside AID<n>/.
SHARED_ROOTS = ("Common",)
SPEAKER_DEVICE = "Built-in Speaker"
# The loudness manager's DRC applies to the AIDs it lists.
LOUDNESS = "DRC/loudness-manager.plist"


def config_name(acoustic):
    return f"Config/AID{acoustic}.plist"


def aid_database(acoustic):
    return f"AID{acoustic}/{AID_DATABASE}"


def _paths(node):
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "Path" and isinstance(value, str):
                yield value
            else:
                yield from _paths(value)
    elif isinstance(node, list):
        for value in node:
            yield from _paths(value)


def only_entry(data, acoustic):
    """Whether a database holds exactly one key, AID<n>, with a dictionary."""
    root = plistlib.loads(data)
    key = f"AID{acoustic}"
    return isinstance(root, dict) and list(root) == [key] and isinstance(root[key], dict)


def speaker_references(entry, acoustic):
    """(paths under AID<n>/ relative to it, shared paths) that the database
    entry's built-in speaker modes reference. Paths are relative to the
    Tunings root. Anything outside AID<n>/ and Common/ is refused."""
    root = plistlib.loads(entry)
    aid = f"AID{acoustic}"
    speaker = root.get(aid, {}).get(SPEAKER_DEVICE) if isinstance(root.get(aid), dict) else None
    own, shared = set(), set()
    for path in _paths(speaker or {}):
        if not admitted(path):
            raise ValueError(f"inadmissible speaker graph reference {path!r}")
        if path.startswith(aid + "/"):
            own.add(path[len(aid) + 1:])
        elif path.split("/")[0] in SHARED_ROOTS and "/" in path:
            shared.add(path)
        else:
            raise ValueError(f"speaker graph references a file outside {aid}/ and Common/: {path}")
    return own, shared


def loudness_lists(data, acoustic):
    """Whether the loudness manager plist lists AID<n>."""
    root = plistlib.loads(data)
    listed = root.get("Supported_Hardware") if isinstance(root, dict) else None
    if not isinstance(listed, list) or not all(isinstance(x, str) for x in listed):
        raise ValueError("malformed loudness manager")
    return f"AID{acoustic}" in listed


def database_entry(data, acoustic):
    """This Mac's entry of the DSP database, as a plist with that one key."""
    root = plistlib.loads(data)
    if not isinstance(root, dict):
        raise ValueError("DSP database is not a dictionary")
    key = f"AID{acoustic}"
    if key not in root:
        return None
    return plistlib.dumps({key: root[key]}, fmt=plistlib.FMT_XML)


SCHEMA = 2
MAX_FILES = 1024
# The folder and the selection data together, as in the native installer.
MAX_BYTES = 64 << 20
# Per-file bounds of the native installer: the DSP database is read with the
# folder's bound, the volume configuration and the manifest with 1 MiB.
MAX_DATABASE_BYTES = 64 << 20
MAX_CONFIG_BYTES = 1 << 20
MAX_MANIFEST_BYTES = 1 << 20
MAX_SPTS_BYTES = 4096
BASED_ON = "aurora-audio-dsp by saku (MIT)"


def read_bounded(path, maximum):
    """A regular file's bytes, at most `maximum` of them; links are refused."""
    fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(fd, "rb") as f:
        if not stat.S_ISREG(os.fstat(f.fileno()).st_mode):
            raise ValueError(f"not a regular file: {path}")
        data = f.read(maximum + 1)
    if len(data) > maximum:
        raise ValueError(f"over budget: {path}")
    return data


def _raise(error):
    raise error


def acoustic_id(data):
    """The device tree stores `acoustic-id` as one little-endian 32-bit word."""
    if not isinstance(data, bytes) or len(data) != 4:
        return None
    value = int.from_bytes(data, "little")
    return value if 0 < value < 100000 else None


def admitted(relative):
    parts = relative.split("/")
    if not relative or relative.startswith("/") or "\\" in relative or "\0" in relative:
        return False
    if any(p in ("", ".", "..") for p in parts):
        return False
    return not any(p.startswith("._") or p == ".DS_Store" for p in parts)


def probe(run=subprocess.run):
    """(acoustic-id bytes, speaker-thiele-small bytes or None), or None."""
    p = run(["ioreg", "-a", "-r", "-p", "IODeviceTree", "-n", "audio", "-k", "acoustic-id"],
            capture_output=True)
    if p.returncode != 0 or not p.stdout.strip():
        return None
    found = []
    for node in plistlib.loads(p.stdout):
        if isinstance(node.get("acoustic-id"), bytes):
            spts = node.get("speaker-thiele-small")
            found.append((node["acoustic-id"], spts if isinstance(spts, bytes) else None))
    if len(found) != 1:
        if found:
            log.warning("Several audio nodes carry acoustic-id; speaker tuning not collected")
        return None
    return found[0]


# The unit ID: SHA-256 over this tag and the root serial-number without its
# NUL padding. The same rule as aurora-speaker-tuning's machine::unit_id.
UNIT_ID_DOMAIN = b"aurora-speaker-unit-v1\0"
UNIT_ID = re.compile(r"\A[0-9a-f]{64}\Z")
# A board as the Linux device tree's first compatible names it (j616s).
BOARD = re.compile(r"\A[a-z0-9]{1,8}\Z")


def unit_id(raw):
    """The unit ID of a raw `serial-number` property, or None unless it is
    1 to 64 printable, non-space ASCII bytes after its NUL padding."""
    if not isinstance(raw, bytes):
        return None
    serial = raw.rstrip(b"\0")
    if not 0 < len(serial) <= 64 or not all(0x21 <= b <= 0x7E for b in serial):
        return None
    return hashlib.sha256(UNIT_ID_DOMAIN + serial).hexdigest()


def board_name(raw):
    """The board of a `target-type` property (J616s, or J616sAP): j616s."""
    if isinstance(raw, bytes):
        raw = raw.rstrip(b"\0").decode("ascii", "replace")
    if not isinstance(raw, str):
        return None
    raw = raw[:-2] if raw.endswith("AP") else raw
    board = raw.lower()
    return board if BOARD.match(board) else None


def probe_identity(run=subprocess.run):
    """(board, unit ID) of this Mac from the device tree root, read only;
    either is None when unknown. Never fails the collection: a bundle
    without them still installs, and Linux then runs its speakers without
    the per-unit calibration until it is collected again."""
    try:
        p = run(["ioreg", "-a", "-r", "-d", "1", "-p", "IODeviceTree", "-n", "device-tree"],
                capture_output=True)
        if p.returncode != 0 or not p.stdout.strip():
            return None, None
        roots = [n for n in plistlib.loads(p.stdout) if isinstance(n, dict)]
        if len(roots) != 1:
            return None, None
        return board_name(roots[0].get("target-type")), unit_id(roots[0].get("serial-number"))
    except Exception as e:
        log.info(f"Mac identity unknown: {e}")
        return None, None


SPEAKER_SERVICE = "AppleEmbeddedAudioDevice"
MAX_SPEAKER_ID = 0xFFFFFFFF


def speaker_id(services):
    """The `speakerID` of the one AppleEmbeddedAudioDevice whose `device UID`
    is `Speaker`: the HAL's `spid`, which selects among speaker vendor
    variants. None (unknown) for no such service, several, or a value that
    is not an integer from 0 to 2^32-1. 65535 (an invalid strap) is kept
    as reported."""
    if not isinstance(services, list):
        return None
    speakers = [s for s in services
                if isinstance(s, dict) and s.get("device UID") == "Speaker"]
    if len(speakers) != 1:
        return None
    value = speakers[0].get("speakerID")
    if type(value) is not int or not 0 <= value <= MAX_SPEAKER_ID:
        return None
    return value


def probe_speaker_id(run=subprocess.run):
    """Read-only `speakerID` from the IORegistry service plane, or None.
    Never fails the collection."""
    try:
        p = run(["ioreg", "-a", "-r", "-c", SPEAKER_SERVICE, "-k", "speakerID"],
                capture_output=True)
        if p.returncode != 0 or not p.stdout.strip():
            return None
        return speaker_id(plistlib.loads(p.stdout))
    except Exception as e:
        log.info(f"Speaker ID unknown: {e}")
        return None


def read_shared(tunings_root, path):
    """A shared file the speaker graph references, a regular file reached
    without links inside its Tunings subdirectory."""
    root = os.path.realpath(tunings_root)
    full = os.path.join(root, path)
    if os.path.realpath(full) != full:
        raise ValueError(f"shared tuning file is reached through a link: {path}")
    return read_bounded(full, MAX_BYTES)


def read_folder(path):
    """Regular files of a tuning folder by relative path, and the number of
    admitted entries found; links within the folder are kept as copies of
    their targets.

    Anything that would leave the set short fails the whole read: a
    directory that cannot be listed, a link to a directory, a link that does
    not resolve to a regular file inside the folder, or the budget."""
    files = {}
    total = 0
    count = 0
    root = os.path.realpath(path)
    for parent, dirs, names in os.walk(root, onerror=_raise, followlinks=False):
        for name in sorted(dirs):
            rel = os.path.relpath(os.path.join(parent, name), root)
            if admitted(rel) and os.path.islink(os.path.join(parent, name)):
                raise ValueError(f"tuning folder holds a directory link: {rel}")
        for name in sorted(names):
            full = os.path.join(parent, name)
            rel = os.path.relpath(full, root)
            if not admitted(rel):
                continue
            count += 1
            real = os.path.realpath(full)
            if os.path.commonpath([real, root]) != root or not os.path.isfile(real):
                raise ValueError(f"tuning entry is not a regular file in its folder: {rel}")
            if len(files) >= MAX_FILES:
                raise ValueError("tuning folder over budget")
            data = read_bounded(real, MAX_BYTES - total)
            total += len(data)
            files[rel] = data
    return files, count


def record(path, data):
    return {"path": path, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def lattice_record(data):
    rec = record(LATTICE, data)
    rec.update({"version": lsm2_lattice.VERSION, "format": lsm2_lattice.FORMAT,
                "entries": lsm2_lattice.ENTRIES})
    return rec


def manifest(aid, acoustic, files, spts, build, selection=None, speaker=None,
             shared=None, loudness=None, source=DATABASE, lattice=None, board=None, unit=None):
    selection = selection or {}
    shared = shared or {}
    return {
        # The Mac the bundle was collected on: its board and the unit ID of
        # its serial number (never the serial). null when unknown.
        "board": board,
        "unit_id": unit,
        # The HAL's `spid` (IORegistry `speakerID`), or null when unknown.
        "speaker_id": speaker,
        "schema": SCHEMA,
        "kind": "aurora-audio-tuning",
        # Entries of the source AID<n> folder (regular files and links to
        # them, sidecars excluded); equals the AID<n>/ records in `files`.
        "source_file_count": len(files),
        # Only a full set is ever written: every folder entry and both
        # selection files. Readers refuse anything else.
        "complete": True,
        "acoustic_id": acoustic,
        "aid": aid,
        "macos_build": build,
        "speaker_thiele_small": None if spts is None else {
            "path": SPTS, "bytes": len(spts), "sha256": hashlib.sha256(spts).hexdigest()},
        "device_dsp_database": DATABASE if DATABASE in selection else None,
        # Where the entry was read: the shared DeviceDSPDatabase.plist (up to
        # macOS 26) or AID<n>/DSP/DSPDatabase.plist (macOS 27). Optional;
        # missing means the shared database.
        "device_dsp_database_source": source,
        # Files outside AID<n>/ that the speaker modes reference (Common/).
        # Optional, kept apart from `files` so schema 2 readers still accept
        # the bundle.
        "shared_files": [record(path, shared[path]) for path in sorted(shared)],
        # The loudness manager, only when it lists this AID. Optional.
        "loudness_manager": None if loudness is None else record(LOUDNESS, loudness),
        # The lsm2 masker's bucket lattice from this Mac's AudioDSP: 100
        # little-endian u64 exclusive bucket ends (version 1). Optional for
        # readers: bundles collected before it have none.
        "lsm2_mask_lattice": None if lattice is None else lattice_record(lattice),
        "config": config_name(acoustic) if config_name(acoustic) in selection else None,
        "files": [{"path": f"{aid}/{rel}", "bytes": len(files[rel]),
                   "sha256": hashlib.sha256(files[rel]).hexdigest()}
                  for rel in sorted(files)]
                 + [{"path": name, "bytes": len(selection[name]),
                     "sha256": hashlib.sha256(selection[name]).hexdigest()}
                    for name in sorted(selection)],
        "based_on": BASED_ON,
    }


def tree(acoustic, files, spts, build, selection=None, source_count=None, speaker=None,
         shared=None, loudness=None, source=DATABASE, lattice=None, board=None, unit=None):
    """Relative path -> bytes for one bundle: AID<n>/..., the selection
    data, the shared files the speaker graph references, the loudness
    manager, the record, the manifest. Only a complete set makes a bundle."""
    selection = selection or {}
    shared = shared or {}
    for name in selection:
        if name not in (DATABASE, config_name(acoustic)):
            raise ValueError(f"unexpected selection file {name!r}")
    for name in (DATABASE, config_name(acoustic)):
        if not selection.get(name):
            raise ValueError(f"incomplete speaker tuning: no {name}")
    if not files or len(files) > MAX_FILES:
        raise ValueError("empty or oversized tuning folder")
    if source_count is not None and source_count != len(files):
        raise ValueError("tuning folder read short")
    if source not in (DATABASE, aid_database(acoustic)):
        raise ValueError(f"unexpected DSP database source {source!r}")
    for path in shared:
        if not admitted(path) or path.split("/")[0] not in SHARED_ROOTS or "/" not in path:
            raise ValueError(f"inadmissible shared tuning path {path!r}")
    if len(files) + len(shared) > MAX_FILES:
        raise ValueError("tuning folder over budget")
    if loudness is not None and not 0 < len(loudness) <= MAX_CONFIG_BYTES:
        raise ValueError("implausible loudness manager")
    if sum(map(len, files.values())) + sum(map(len, selection.values())) \
            + sum(map(len, shared.values())) + len(loudness or b"") > MAX_BYTES:
        raise ValueError("tuning folder over budget")
    if spts is not None and not 0 < len(spts) <= MAX_SPTS_BYTES:
        raise ValueError("implausible speaker-thiele-small")
    if speaker is not None and (type(speaker) is not int or not 0 <= speaker <= MAX_SPEAKER_ID):
        raise ValueError("implausible speaker ID")
    if not lsm2_lattice.valid(lattice):
        raise ValueError("incomplete speaker tuning: no valid lsm2 masker lattice")
    if board is not None and not (isinstance(board, str) and BOARD.match(board)):
        raise ValueError("implausible board")
    if unit is not None and not (isinstance(unit, str) and UNIT_ID.match(unit)):
        raise ValueError("implausible unit ID")
    aid = f"AID{acoustic}"
    out = {}
    for rel, data in files.items():
        if not admitted(rel):
            raise ValueError(f"inadmissible tuning path {rel!r}")
        out[f"{aid}/{rel}"] = data
    out.update(selection)
    out.update(shared)
    if loudness is not None:
        out[LOUDNESS] = loudness
    check_references(out, acoustic)
    if spts is not None:
        out[SPTS] = spts
    out[LATTICE] = bytes(lattice)
    out[MANIFEST] = json.dumps(manifest(aid, acoustic, files, spts, build, selection, speaker,
                                        shared, loudness, source, bytes(lattice), board, unit),
                               indent=2, sort_keys=True).encode("utf-8")
    return out


def check_references(bundle, acoustic):
    """Every file the speaker modes reference is in the bundle, and the
    shared files are exactly those referenced."""
    own, shared = speaker_references(bundle[DATABASE], acoustic)
    aid = f"AID{acoustic}"
    missing = sorted([f"{aid}/{p}" for p in own if f"{aid}/{p}" not in bundle]
                     + [p for p in shared if p not in bundle])
    if missing:
        raise ValueError(f"speaker graph references missing files: {', '.join(missing[:4])}")
    held = {p for p in bundle if p.split("/")[0] in SHARED_ROOTS}
    if held != shared:
        raise ValueError("shared tuning files differ from the speaker graph's references")


def os_build(run=subprocess.run):
    p = run(["sysctl", "-n", "kern.osversion"], capture_output=True)
    value = p.stdout.decode("ascii", "replace").strip() if p.returncode == 0 else ""
    return value if value.isalnum() and len(value) <= 16 else "unknown"


class AudioTuningFWCollection(object):
    """Live collection on macOS (path None), or re-collection from an
    extracted fallback archive (path = the directory holding audio-tuning/)."""

    def __init__(self, path=None, tunings_root=TUNINGS_ROOT, run=subprocess.run, audiodsp=None):
        self.fwfiles = []
        self.bundle = {}
        try:
            if path is None:
                self.load(tunings_root, run, AUDIODSP_PATH if audiodsp is None else audiodsp)
            else:
                self.update(os.path.join(path, FALLBACK_DIR))
        except lsm2_lattice.LatticeError as e:
            log.error(f"Speaker tuning not collected: the lsm2 masker lattice: {e}")
            self.fwfiles = []
            self.bundle = {}
        except Exception as e:
            log.warning(f"Speaker tuning not collected: {e}")
            self.fwfiles = []
            self.bundle = {}

    def files(self):
        return self.fwfiles

    def _publish(self, bundle):
        self.bundle = bundle
        self.fwfiles = [(PACKAGE_PREFIX + rel, FWFile(PACKAGE_PREFIX + rel, data))
                        for rel, data in sorted(bundle.items())]

    def load(self, tunings_root, run, audiodsp=AUDIODSP_PATH):
        props = probe(run)
        if props is None:
            log.info("No device-tree acoustic-id; no speaker tuning to collect")
            return
        acoustic = acoustic_id(props[0])
        if acoustic is None:
            raise ValueError("malformed acoustic-id")
        folder = os.path.join(tunings_root, f"AID{acoustic}")
        if not os.path.isdir(folder):
            log.info(f"No tuning folder {folder}")
            return
        files, count = read_folder(folder)
        selection = {}
        source = DATABASE
        db_path = os.path.join(tunings_root, DATABASE)
        if os.path.lexists(os.path.join(folder, AID_DATABASE)):
            # macOS 27: the per-machine entry in the folder wins.
            source = aid_database(acoustic)
            db_path = os.path.join(tunings_root, source)
        if os.path.lexists(db_path):
            data = read_bounded(db_path, MAX_DATABASE_BYTES)
            if source != DATABASE and only_entry(data, acoustic):
                # Already this Mac's entry alone: kept byte for byte.
                entry = data
            else:
                entry = database_entry(data, acoustic)
            if entry is not None:
                selection[DATABASE] = entry
        cfg_path = os.path.join(tunings_root, config_name(acoustic))
        if os.path.lexists(cfg_path):
            selection[config_name(acoustic)] = read_bounded(cfg_path, MAX_CONFIG_BYTES)
        shared = {}
        if DATABASE in selection:
            for path in sorted(speaker_references(selection[DATABASE], acoustic)[1]):
                shared[path] = read_shared(tunings_root, path)
        loudness = None
        lm_path = os.path.join(tunings_root, LOUDNESS)
        if os.path.lexists(lm_path):
            data = read_bounded(lm_path, MAX_CONFIG_BYTES)
            if loudness_lists(data, acoustic):
                loudness = data
        speaker = probe_speaker_id(run)
        board, unit = probe_identity(run)
        if unit is None and props[1] is not None:
            log.warning("This Mac's serial-number is unreadable: the speaker tuning is collected "
                        "without its unit, and Linux will not use its speaker calibration")
        # Exactly one valid lattice, or no bundle: never a guessed table.
        lattice = lsm2_lattice.read(audiodsp)
        self._publish(tree(acoustic, files, props[1], os_build(run), selection, count, speaker,
                           shared, loudness, source, lattice, board, unit))
        log.info(f"  Collected speaker tuning AID{acoustic} ({len(files)} files"
                 + (f", {len(shared)} shared" if shared else "")
                 + (", loudness manager" if loudness is not None else "")
                 + ", lsm2 masker lattice)")

    def update(self, path):
        """Re-verify an extracted fallback bundle against its manifest.
        Anything but a complete, self-consistent set is refused."""
        meta_path = os.path.join(path, MANIFEST)
        if not os.path.lexists(meta_path):
            return
        raw = read_bounded(meta_path, MAX_MANIFEST_BYTES)
        meta = json.loads(raw)
        if not isinstance(meta, dict) or meta.get("schema") != SCHEMA \
                or meta.get("kind") != "aurora-audio-tuning":
            raise ValueError("unknown speaker tuning manifest")
        acoustic = meta.get("acoustic_id")
        if type(acoustic) is not int or not 0 < acoustic < 100000:
            raise ValueError("malformed acoustic_id")
        aid = f"AID{acoustic}"
        selection = (DATABASE, config_name(acoustic))
        if meta.get("aid") != aid:
            raise ValueError("manifest aid does not match acoustic_id")
        speaker = meta.get("speaker_id")
        if speaker is not None and (type(speaker) is not int
                                    or not 0 <= speaker <= MAX_SPEAKER_ID):
            raise ValueError("malformed speaker_id")
        board = meta.get("board")
        if board is not None and not (isinstance(board, str) and BOARD.match(board)):
            raise ValueError("malformed board")
        unit = meta.get("unit_id")
        if unit is not None and not (isinstance(unit, str) and UNIT_ID.match(unit)):
            raise ValueError("malformed unit_id")
        if meta.get("complete") is not True:
            raise ValueError("incomplete speaker tuning bundle")
        if meta.get("device_dsp_database") != DATABASE \
                or meta.get("config") != config_name(acoustic):
            raise ValueError("speaker tuning bundle lacks its selection data")
        records = meta.get("files")
        if not isinstance(records, list) or len(records) > MAX_FILES + len(selection):
            raise ValueError("malformed speaker tuning file list")
        folder = 0
        paths = set()
        for rec in records:
            rel = rec.get("path") if isinstance(rec, dict) else None
            if not isinstance(rel, str) or not admitted(rel) or rel in paths:
                raise ValueError(f"inadmissible tuning path {rel!r}")
            if rel.startswith(aid + "/"):
                folder += 1
            elif rel not in selection:
                raise ValueError(f"tuning path outside the bundle layout {rel!r}")
            paths.add(rel)
        count = meta.get("source_file_count")
        if type(count) is not int or count <= 0 or count != folder:
            raise ValueError("speaker tuning file count does not match its entries")
        if not all(name in paths for name in selection):
            raise ValueError("speaker tuning bundle lacks its selection data")
        source = meta.get("device_dsp_database_source", DATABASE)
        if source not in (None, DATABASE, aid_database(acoustic)):
            raise ValueError("unknown DSP database source")
        if source == aid_database(acoustic) and source not in paths:
            raise ValueError("the DSP database source is not in the bundle")
        shared = meta.get("shared_files") or []
        if not isinstance(shared, list) or len(shared) + folder > MAX_FILES:
            raise ValueError("malformed shared tuning file list")
        for rec in shared:
            rel = rec.get("path") if isinstance(rec, dict) else None
            if not isinstance(rel, str) or not admitted(rel) or rel in paths \
                    or rel.split("/")[0] not in SHARED_ROOTS or "/" not in rel:
                raise ValueError(f"inadmissible shared tuning path {rel!r}")
            paths.add(rel)
        records = records + shared
        loudness = meta.get("loudness_manager")
        if loudness is not None:
            if not isinstance(loudness, dict) or loudness.get("path") != LOUDNESS:
                raise ValueError("malformed loudness manager record")
            records = records + [loudness]
        spts = meta.get("speaker_thiele_small")
        if spts is not None:
            if not isinstance(spts, dict) or spts.get("path") != SPTS:
                raise ValueError("malformed speaker-thiele-small record")
            records = records + [spts]
        lattice = meta.get("lsm2_mask_lattice")
        if lattice is not None:
            if not isinstance(lattice, dict) or lattice.get("path") != LATTICE \
                    or lattice.get("version") != lsm2_lattice.VERSION \
                    or lattice.get("format") != lsm2_lattice.FORMAT \
                    or lattice.get("entries") != lsm2_lattice.ENTRIES \
                    or lattice.get("bytes") != lsm2_lattice.SIZE:
                raise ValueError("malformed lsm2 masker lattice record")
            records = records + [lattice]
        else:
            log.warning("This speaker tuning bundle has no lsm2 masker lattice (collected "
                        "before the installer read it); collect it again on macOS")
        if unit is None and spts is not None:
            log.warning("This speaker tuning bundle does not name the Mac it was collected on "
                        "(collected before the installer recorded it): Linux runs the speakers "
                        "without its calibration (open loop); collect it again on macOS")
        root = os.path.realpath(path)
        bundle = {}
        total = 0
        for rec in records:
            rel = rec["path"]
            size = rec.get("bytes")
            limit = MAX_SPTS_BYTES if rel == SPTS else lsm2_lattice.SIZE if rel == LATTICE else \
                min(MAX_CONFIG_BYTES, MAX_BYTES - total) if rel == LOUDNESS else MAX_BYTES - total
            if type(size) is not int or not 0 <= size <= limit:
                raise ValueError(f"tuning file over budget: {rel}")
            full = os.path.join(root, rel)
            if os.path.realpath(full) != full:
                raise ValueError(f"tuning file is reached through a link: {rel}")
            data = read_bounded(full, size)
            if len(data) != size or hashlib.sha256(data).hexdigest() != rec.get("sha256"):
                raise ValueError(f"tuning file changed: {rel}")
            if rel not in (SPTS, LATTICE):
                total += size
            bundle[rel] = data
        if LATTICE in bundle and not lsm2_lattice.valid(bundle[LATTICE]):
            raise ValueError("the lsm2 masker lattice is not valid")
        if LOUDNESS in bundle and not loudness_lists(bundle[LOUDNESS], acoustic):
            raise ValueError("the loudness manager does not list this AID")
        check_references(bundle, acoustic)
        bundle[MANIFEST] = raw
        self._publish(bundle)

    def write_fallback(self, directory):
        """Write audio-tuning/ for the raw fallback archive under directory.
        A failure leaves no partial audio-tuning/ behind and never stops the
        installation; the result says whether the bundle was written."""
        if not self.bundle:
            return False
        target = os.path.join(directory, FALLBACK_DIR)
        try:
            for rel, data in self.bundle.items():
                dest = os.path.join(target, rel)
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                with open(dest, "wb") as fd:
                    fd.write(data)
        except Exception as e:
            log.warning(f"Speaker tuning left out of the fallback archive: {e}")
            shutil.rmtree(target, ignore_errors=True)
            return False
        return True
