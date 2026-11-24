# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""The one source of per-model firmware a Linux tool takes from Apple:
the macOS restore image (IPSW) for this Mac's board and a firmware
version, read by HTTP ranges without downloading it.

Per-model, per-version data (radio firmware, speaker tuning, and the like)
comes only from here, never from an installed macOS. Only per-unit values
(calibration, keys, identity) are read from the Mac, by each tool's own
allow-listed reader.

A version names the restore images to look in: the packaged catalog
(restore-images.json, the stub firmware the installer admits) and Apple's
IPSW feed. The first image whose BuildManifest carries the board is used;
aea_ro decodes its root file system on demand and apfs_ro reads its
System volume. An image whose BuildManifest names another version, a
board no image carries, or an image that cannot be read is an error:
callers refuse, they never look elsewhere.

    ipsw.py find BOARD VERSION      which restore image a Mac would use
"""
import json
import logging
import os
import plistlib
import sys
import urllib.request

from . import aea_ro, apfs_ro, remotezip, udif_ro

log = logging.getLogger("aurora-firmware")

CATALOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "restore-images.json")
MESU = "https://mesu.apple.com/assets/macos/com_apple_macOSIPSW/com_apple_macOSIPSW.xml"
MAX_FEED = 8 << 20
MAX_MANIFEST = 64 << 20


class IpswError(Exception):
    pass


class IpswUnavailable(IpswError):
    """The restore image or Apple's key service cannot be reached now."""


def catalog(path=None):
    """[(version, build, url)] from the packaged catalog."""
    path = path or CATALOG
    try:
        with open(path) as f:
            doc = json.load(f)
    except (OSError, ValueError) as e:
        raise IpswError(f"{path}: {e}") from None
    out = []
    for i in doc.get("restore_images", []):
        if not (isinstance(i, dict) and all(isinstance(i.get(k), str) for k in ("version", "build", "url"))
                and i["url"].startswith("https://")):
            raise IpswError(f"{path}: malformed restore image entry")
        out.append((i["version"], i["build"], i["url"]))
    return out


def feed(fetch=None):
    """[(version, build, url)] from Apple's IPSW feed; empty when it cannot
    be read (the catalog still answers)."""
    try:
        req = urllib.request.Request(MESU, headers={"User-Agent": aea_ro.UA})
        with (fetch or urllib.request.urlopen)(req, timeout=30) as r:
            data = r.read(MAX_FEED + 1)
        if len(data) > MAX_FEED:
            raise IpswError("Apple's IPSW feed is over budget")
        doc = plistlib.loads(data)
    except Exception as e:
        log.info(f"Apple's IPSW feed is unavailable: {e}")
        return []
    out = []
    for group in doc.get("MobileDeviceSoftwareVersionsByVersion", {}).values():
        for builds in (group.get("MobileDeviceSoftwareVersions") or {}).values():
            for build, v in (builds or {}).items():
                r = (v or {}).get("Restore") or {}
                url, version = r.get("FirmwareURL"), r.get("ProductVersion")
                if isinstance(url, str) and url.startswith("https://") and isinstance(version, str) \
                        and build != "Unknown" and url.endswith(".ipsw"):
                    out.append((version, build, url))
    return out


def images_for_version(version, entries=None, fetch=None):
    """[(build, url)] of the restore images of one macOS version, without
    repeats: the packaged catalog's, and Apple's feed only for a version
    the catalog does not list (the feed is not fetched otherwise)."""
    if entries is None:
        entries = catalog()
        if not any(v == version for v, _, _ in entries):
            entries = feed(fetch)
    seen, out = set(), []
    for v, b, u in entries:
        if v == version and b not in seen:
            seen.add(b)
            out.append((b, u))
    return out


def os_member(manifest, board):
    """The root file system member a BuildManifest names for a board, or
    None when the image does not carry it."""
    paths = {bi["Manifest"]["OS"]["Info"]["Path"] for bi in manifest.get("BuildIdentities", [])
             if bi.get("Info", {}).get("DeviceClass") == f"{board}ap" and "OS" in bi.get("Manifest", {})}
    if len(paths) > 1:
        raise IpswError(f"the restore image names {len(paths)} root file systems for {board}")
    return paths.pop() if paths else None


def _manifest(z):
    m = z.members.get("BuildManifest.plist")
    if m is None or m.usize > MAX_MANIFEST:
        raise IpswError("the restore image has no usable BuildManifest")
    return plistlib.loads(m.read(MAX_MANIFEST))


def find(board, images, version=None, open_zip=remotezip.RemoteZip):
    """(build, url, member) of the first image in images ([(build, url)])
    that carries the board. Each image must be the build (and version, when
    given) it is listed as."""
    if not images:
        raise IpswError(f"no restore image of macOS {version} is known")
    seen = []
    for build, url in images:
        try:
            bm = _manifest(open_zip(url))
        except (remotezip.RemoteZipUnavailable, OSError) as e:
            raise IpswUnavailable(f"restore image {build}: {e}") from None
        except (IpswError, remotezip.RemoteZipError, ValueError) as e:
            raise IpswError(f"restore image {build}: {e}") from None
        got, gotv = bm.get("ProductBuildVersion"), bm.get("ProductVersion")
        if got != build or (version is not None and gotv != version):
            raise IpswError(f"the restore image listed as {build} is macOS {gotv} ({got})")
        member = os_member(bm, board)
        if member is not None:
            return build, url, member
        seen.append(build)
    raise IpswError(f"no restore image of macOS {version or ''} ({', '.join(seen)}) carries board {board}")


def open_system(board, images, version=None, open_zip=remotezip.RemoteZip):
    """(System volume, range fetcher, build, url) of the restore image for a
    board (j413) among images."""
    build, url, member = find(board, images, version, open_zip)
    try:
        return _open(board, build, url, member, open_zip)
    except (aea_ro.AeaUnavailable, remotezip.RemoteZipUnavailable, OSError) as e:
        raise IpswUnavailable(f"restore image {build}: {e}") from None
    except (aea_ro.AeaError, apfs_ro.ApfsError, udif_ro.UdifError, remotezip.RemoteZipError) as e:
        raise IpswError(f"restore image {build}: {e}") from None


def _open(board, build, url, member, open_zip):
    if member.endswith(".aea"):
        src, rng, _ = aea_ro.open_member(url, member)
    else:
        # Before Apple Encrypted Archives: a stored UDIF disk image.
        mem = open_zip(url).members.get(member)
        if mem is None or mem.method != 0:
            raise IpswError(f"{member}: not a stored member")
        rng = aea_ro.Ranges(url)
        raw = type("_M", (), {"pread": staticmethod(lambda n, off: mem.pread(n, off))})
        try:
            src = udif_ro.Partition(raw, mem.usize)
        except udif_ro.UdifError as e:
            raise IpswError(f"{member}: {e}") from None
    vols = [v for v in apfs_ro.Container(src).volumes() if v.role == apfs_ro.ROLE_SYSTEM]
    if len(vols) != 1:
        raise IpswError("the root file system has no single System volume")
    vol = vols[0]
    try:
        sv = plistlib.loads(vol.read_file(vol.resolve("System/Library/CoreServices/SystemVersion.plist")[0]))
    except (apfs_ro.ApfsError, ValueError) as e:
        raise IpswError(f"the restore image's System volume has no SystemVersion.plist: {e}") from None
    if sv.get("ProductBuildVersion") != build:
        raise IpswError(f"the restore image's System volume is macOS {sv.get('ProductBuildVersion')}, not {build}")
    return vol, rng, build, url


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 3 or argv[0] != "find":
        print(__doc__.strip().splitlines()[-1].strip(), file=sys.stderr)
        return 64
    board, version = argv[1], argv[2]
    try:
        build, url, member = find(board, images_for_version(version), version)
    except IpswError as e:
        print(f"ipsw: {e}", file=sys.stderr)
        return 1
    print(f"{build}\t{url}\t{member}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
