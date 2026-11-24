# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""The one source of this Mac's per-model speaker tuning: Apple's macOS
restore image (IPSW) of the pinned release, read by HTTP ranges without
downloading it.

The pin (share/restore-images.json) names the release (macOS 27.2, the
speaker safety stack's) and its restore images in order. The first one
whose BuildManifest carries this Mac's board gives the System volume. The
reading itself is the installer's restore image reader
(asahi_firmware.ipsw with aea_ro, apfs_ro and remotezip, vendored
unchanged), the same code the installer's Linux tools use. On top of it
this module adds only what the speaker tuning needs: the pin, and that the
root file system is an Apple Encrypted Archive, whose every byte is
authenticated against Apple's key, so a mirror (--ipsw) cannot serve an
altered volume under a pinned build. There is no other source: not the
installed macOS, not an older bundle, not another build.
"""
import json
import os
import plistlib

from .asahi_firmware import aea_ro, apfs_ro, remotezip
from .asahi_firmware import ipsw as reader

CATALOG = "/usr/share/aurora-speaker-upgrade/restore-images.json"
IpswError = reader.IpswError
IpswUnavailable = reader.IpswUnavailable


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


def mirror_build(url, open_zip=remotezip.RemoteZip):
    """The build a mirror (--ipsw) serves, from its BuildManifest."""
    try:
        m = open_zip(url).members.get("BuildManifest.plist")
        if m is None or m.usize > reader.MAX_MANIFEST:
            raise IpswError("the restore image has no usable BuildManifest")
        return plistlib.loads(m.read(reader.MAX_MANIFEST)).get("ProductBuildVersion")
    except (remotezip.RemoteZipUnavailable, OSError) as e:
        raise IpswUnavailable(f"restore image {url}: {e}") from None
    except (remotezip.RemoteZipError, ValueError) as e:
        raise IpswError(f"restore image {url}: {e}") from None


def find(board, images=None, url=None, open_zip=remotezip.RemoteZip):
    """(build, url, member) of the pinned image for a board. url (--ipsw,
    e.g. a mirror) replaces the addresses, never the pin: its build must
    still be a pinned one. Only an Apple Encrypted Archive is accepted."""
    release, pinned = pin() if images is None else images
    builds = [b for b, _ in pinned]
    if url:
        got = mirror_build(url, open_zip)
        if got not in builds:
            raise IpswError(f"the restore image is macOS {got}, not a pinned macOS {release} build "
                            f"({', '.join(builds)})")
        pinned = [(got, url)]
    build, u, member = reader.find(board, pinned, release, open_zip)
    if not member.endswith(".aea"):
        raise IpswError(f"the macOS {build} root file system {member} is not an Apple Encrypted Archive")
    return build, u, member


def open_system(board, images=None, url=None, open_zip=remotezip.RemoteZip):
    """(System volume, range fetcher, build, url) of the pinned restore image
    for a board (j413). The shared reader finds the board's image and opens
    exactly the member it checked (require_aea): one BuildManifest fetch
    decides both, so nothing but an Apple Encrypted Archive is ever opened."""
    release, pinned = pin() if images is None else images
    builds = [b for b, _ in pinned]
    if url:
        got = mirror_build(url, open_zip)
        if got not in builds:
            raise IpswError(f"the restore image is macOS {got}, not a pinned macOS {release} build "
                            f"({', '.join(builds)})")
        pinned = [(got, url)]
    return reader.open_system(board, pinned, release, open_zip, require_aea=True)
