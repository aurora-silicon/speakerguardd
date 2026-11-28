#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""A macOS build's speaker units and Tunings, from its restore image, without booting a Mac.

    ipsw_units.py BUILD BOARD OUTDIR [--ipsw URL] [--no-tunings]

Reads the build's restore image by HTTP ranges (packaging/upgrade's aea_ro + apfs_ro; needs
liblzfse or libcompression, and python3-cryptography). Writes OUTDIR/AudioDSP (the
AudioDSP.component binary that implements the speaker graph's Audio Units) and
OUTDIR/Tunings, and prints their hashes. Both are Apple binaries and data: OUTDIR must be
a private, tainted directory.

Use (README, "Builds without a Mac"): if the binary loads on the reference host's macOS
(box_host with AUDIODSP_PATH, after an ad-hoc re-sign of the private copy: `codesign -f -s -`),
attest_refs.sh renders that build's Tunings through that build's units on the host, and
attest.py gates them as that build's block. A binary built for a newer macOS than the host
does not load: those builds need a host on that macOS.
"""
import argparse, hashlib, json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
UPGRADE = os.path.join(HERE, "..", "..", "..", "packaging", "upgrade")
sys.path.insert(0, UPGRADE)
from aurora_speaker_upgrade import ipsw  # noqa: E402
from aurora_speaker_upgrade.asahi_firmware import apfs_ro  # noqa: E402
# This tree's pin, not an installed (possibly older) /usr/share copy
PIN = os.path.join(UPGRADE, "share", "restore-images.json")

UNIT = "/System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("build"); ap.add_argument("board"); ap.add_argument("out")
    ap.add_argument("--ipsw"); ap.add_argument("--no-tunings", action="store_true")
    a = ap.parse_args()
    release, entries = ipsw._pin_entries([PIN])
    pinned = {b: u for b, u, _ in entries}
    url = a.ipsw or pinned.get(a.build)
    if not url:
        sys.exit(f"macOS {a.build} is not in {PIN} (pinned: {', '.join(pinned)}); pass --ipsw URL")
    vol = ipsw.open_system(a.board, images=(release, entries), url=url)[0]
    got = a.build
    os.makedirs(a.out, mode=0o700, exist_ok=True)
    ino, _ = vol.resolve(UNIT)
    b = vol.read_file(ino)
    open(os.path.join(a.out, "AudioDSP"), "wb").write(b)
    res = {"build": got, "AudioDSP": {"sha256": hashlib.sha256(b).hexdigest(), "size": len(b)}}
    if not a.no_tunings:
        res["tunings_files"] = apfs_ro.copy_tree(vol, "/System/Library/Audio/Tunings", os.path.join(a.out, "Tunings"))
    print(json.dumps(res))
    return 0


if __name__ == "__main__":
    sys.exit(main())
