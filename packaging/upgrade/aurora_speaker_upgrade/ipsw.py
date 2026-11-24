# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""The one source of this Mac's per-model speaker tuning: Apple's macOS
restore image (IPSW) of the pinned release, read by HTTP ranges without
downloading it.

The pin (share/restore-images.json) names the release (macOS 27.2, the
speaker safety stack's) and its restore images in order. The first one
whose BuildManifest carries this Mac's board gives the root file system;
aea_ro decodes it on demand and apfs_ro reads its System volume. The image
must be exactly a pinned build. There is no other source: not the installed
macOS, not an older bundle, not another build. If no pinned image carries
the board, or one cannot be read, the tool refuses and the speakers stay
as they are.
"""
import json
import logging
import os
import plistlib

from . import aea_ro, apfs_ro, remotezip

log = logging.getLogger("aurora-speaker-upgrade")

CATALOG = "/usr/share/aurora-speaker-upgrade/restore-images.json"
MAX_MANIFEST = 64 << 20


class IpswError(Exception):
    pass


def pin(paths=None):
    """(release, [(build, url)]) from the packaged pin (or, uninstalled, the
    one beside this package)."""
    if paths is None:
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        paths = (CATALOG, os.path.join(here, "share", "restore-images.json"))
    for path in paths:
        try:
            with open(path) as f:
                doc = json.load(f)
        except (OSError, ValueError):
            continue
        images = doc.get("restore_images")
        release = doc.get("macos")
        if not isinstance(release, str) or not isinstance(images, list) or not images:
            raise IpswError(f"{path}: malformed restore image pin")
        out = []
        for i in images:
            if not (isinstance(i, dict) and isinstance(i.get("build"), str)
                    and isinstance(i.get("url"), str) and i["url"].startswith("https://")):
                raise IpswError(f"{path}: malformed restore image pin entry")
            out.append((i["build"], i["url"]))
        return release, out
    raise IpswError("no restore image pin is installed (restore-images.json)")


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


def find(board, images=None, url=None, open_zip=remotezip.RemoteZip):
    """(build, url, BuildManifest, member) of the pinned image for a board.
    url (--ipsw, e.g. a local mirror) replaces the addresses, never the pin:
    its build must still be a pinned one."""
    release, pinned = pin() if images is None else images
    builds = [b for b, _ in pinned]
    candidates = [(None, url)] if url else pinned
    reasons = []
    for build, u in candidates:
        try:
            bm = _manifest(open_zip(u))
        except (IpswError, remotezip.RemoteZipError, OSError, ValueError) as e:
            raise IpswError(f"macOS {release} restore image {build or u}: {e}") from None
        got = bm.get("ProductBuildVersion")
        if got not in builds or (build is not None and got != build):
            raise IpswError(f"the restore image is macOS {got}, not a pinned macOS {release} build "
                            f"({', '.join(builds)})")
        member = os_member(bm, board)
        if member is not None:
            return got, u, bm, member
        reasons.append(got)
    raise IpswError(f"no pinned macOS {release} restore image ({', '.join(reasons)}) carries board {board}")


def open_system(board, images=None, url=None):
    """(System volume, range fetcher, build) of the pinned restore image for
    a board (j413)."""
    build, url, bm, member = find(board, images, url)
    if member.endswith(".aea"):
        a, rng, _ = aea_ro.open_member(url, member)
        src = a
    else:
        mem = remotezip.RemoteZip(url).members.get(member)
        if mem is None or mem.method != 0:
            raise IpswError(f"{member}: not a stored member")
        rng = aea_ro.Ranges(url)
        src = type("_M", (), {"pread": staticmethod(lambda n, off: mem.pread(n, off))})
    vols = [v for v in apfs_ro.Container(src).volumes() if v.role == apfs_ro.ROLE_SYSTEM]
    if len(vols) != 1:
        raise IpswError("the root file system has no single System volume")
    return vols[0], rng, build, url
