# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""Route 2: this Mac's macOS build, read from Apple's restore image (IPSW)
by HTTP ranges, without downloading it.

The restore image of a build is found in Apple's IPSW feed (mesu) or in the
packaged catalog of restore images (public CDN addresses only, no Apple
data). Its BuildManifest names the root file system member for this board;
aea_ro decodes it on demand and apfs_ro reads its System volume.
"""
import json
import logging
import os
import plistlib
import urllib.request

from . import aea_ro, apfs_ro, remotezip

log = logging.getLogger("aurora-speaker-upgrade")

MESU = "https://mesu.apple.com/assets/macos/com_apple_macOSIPSW/com_apple_macOSIPSW.xml"
CATALOG = "/usr/share/aurora-speaker-upgrade/restore-images.json"
MAX_FEED = 8 << 20
MAX_MANIFEST = 64 << 20


class IpswError(Exception):
    pass


def feed_urls(fetch=None):
    """{build: url} from Apple's IPSW feed."""
    try:
        req = urllib.request.Request(MESU, headers={"User-Agent": aea_ro.UA})
        with (fetch or urllib.request.urlopen)(req, timeout=30) as r:
            data = r.read(MAX_FEED + 1)
        if len(data) > MAX_FEED:
            raise IpswError("Apple's IPSW feed is over budget")
        feed = plistlib.loads(data)
    except Exception as e:
        log.info(f"Apple's IPSW feed is unavailable: {e}")
        return {}
    out = {}
    for group in feed.get("MobileDeviceSoftwareVersionsByVersion", {}).values():
        for builds in (group.get("MobileDeviceSoftwareVersions") or {}).values():
            for build, v in (builds or {}).items():
                url = ((v or {}).get("Restore") or {}).get("FirmwareURL")
                if isinstance(url, str) and build != "Unknown":
                    out.setdefault(build, url)
    return out


def catalog_urls(paths=(CATALOG, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                                "share", "restore-images.json"))):
    """{build: url} from the packaged catalog (or, uninstalled, the one
    beside this package)."""
    for path in paths:
        try:
            with open(path) as f:
                return {k: v for k, v in json.load(f).get("restore_images", {}).items() if isinstance(v, str)}
        except (OSError, ValueError):
            continue
    return {}


def find_url(build, explicit=None):
    if explicit:
        return explicit
    url = catalog_urls().get(build) or feed_urls().get(build)
    if not url:
        raise IpswError(f"no restore image of macOS {build} is known (pass --ipsw URL)")
    return url


def open_system(url, board, build=None):
    """(System volume, range fetcher) of the restore image at url, for a
    board (j413) and, when given, exactly that build."""
    z = remotezip.RemoteZip(url)
    m = z.members.get("BuildManifest.plist")
    if m is None or m.usize > MAX_MANIFEST:
        raise IpswError("the restore image has no usable BuildManifest")
    bm = plistlib.loads(m.read(MAX_MANIFEST))
    if build and bm.get("ProductBuildVersion") != build:
        raise IpswError(f"the restore image is macOS {bm.get('ProductBuildVersion')}, not {build}")
    paths = {bi["Manifest"]["OS"]["Info"]["Path"] for bi in bm.get("BuildIdentities", [])
             if bi.get("Info", {}).get("DeviceClass") == f"{board}ap" and "OS" in bi.get("Manifest", {})}
    if len(paths) != 1:
        raise IpswError(f"the restore image names {len(paths)} root file systems for {board}")
    member = paths.pop()
    if member.endswith(".aea"):
        a, rng, _ = aea_ro.open_member(url, member)
        src = a
    else:
        mem = z.members.get(member)
        if mem is None or mem.method != 0:
            raise IpswError(f"{member}: not a stored member")
        rng = aea_ro.Ranges(url)
        src = type("_M", (), {"pread": staticmethod(lambda n, off: mem.pread(n, off))})
    vols = [v for v in apfs_ro.Container(src).volumes() if v.role == apfs_ro.ROLE_SYSTEM]
    if len(vols) != 1:
        raise IpswError("the root file system has no single System volume")
    return vols[0], rng, bm.get("ProductBuildVersion")
