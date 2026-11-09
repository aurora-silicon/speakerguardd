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
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "packaging", "upgrade"))
from aurora_speaker_upgrade import apfs_ro, ipsw  # noqa: E402

UNIT = "/System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("build"); ap.add_argument("board"); ap.add_argument("out")
    ap.add_argument("--ipsw"); ap.add_argument("--no-tunings", action="store_true")
    a = ap.parse_args()
    vol, _, got = ipsw.open_system(ipsw.find_url(a.build, a.ipsw), a.board, a.build)
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
