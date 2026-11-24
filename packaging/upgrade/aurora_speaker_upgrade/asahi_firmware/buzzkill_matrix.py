# SPDX-License-Identifier: MIT
"""The BuzzKill detector's quadrature filter matrix, read from this Mac's own macOS.

The BuzzKill unit's detector passes its 12 kHz analysis stream through a
two-output matrix of second-order sections before the transform. The
coefficients are a table inside macOS's AudioDSP component, not something a
formula reproduces, so the installer reads them from the user's own system
volume, as it does the speaker tuning and the lsm2 lattice, and never ships
them.

The table is found by its structure alone, with no offset, hash, symbol or
value: in every Mach-O slice of the component, at naturally four-byte-aligned
addresses of the file-backed `__const` sections of `__TEXT`, `__DATA_CONST`
and `__DATA`, 80 bytes of little-endian binary32 words holding four
second-order sections `b0, b1, b2, a1, a2` (transfer function
`(b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)`), stored output-major
(output 0 section 0, output 0 section 1, output 1 section 0, output 1
section 1), such that

- every word of the four sections and of the four sections that follow is
  finite, and every one of those eight denominators is strictly stable
  (`|a2| < 1`, `1 + a1 + a2 > 0`, `1 - a1 + a2 > 0`);
- section 1: `b2 == 1`, `b0 == a2`, `b1 != a1`;
- section 2: `b2 == 1`, `b0 == a2`, `b1 == a1` (an exact allpass);
- section 3: `b2 == 1`, `b1 == a1`, `b0 != a2`;
- section 4: `b2 != 1`, `b0 != a2`, `b1 != a1`;
- the four sections that follow are exact allpasses (`b2 == 1`, `b0 == a2`,
  `b1 == a1`). They only guard the match and are not extracted.

Exactly one such table must exist in each slice, and every slice must hold the
same bytes. Anything else (no table, two tables, a malformed slice, slices that
disagree) is an error: there is no fallback table.
"""
import math
import struct

from .lsm2_lattice import (AUDIODSP, LC_SEGMENT_64, MACHO64_MAGIC, MAX_COMMANDS,
                           MAX_COMPONENT_BYTES, SECTION_TYPE, ZEROFILL_TYPES, slices)

SECTIONS = 4
WORDS = 5
SIZE = SECTIONS * WORDS * 4
GUARD = SECTIONS * WORDS * 4
ONE = struct.pack("<f", 1.0)

# The bundle field: 20 little-endian binary32 words, four output-major SOS.
VERSION = 1
FORMAT = "f32le-sos4-output-major"

SEGMENTS = (b"__TEXT", b"__DATA_CONST", b"__DATA")


class MatrixError(ValueError):
    """The component does not yield exactly one valid BuzzKill matrix."""


def _within(total, offset, size, what):
    if offset < 0 or size < 0 or offset > total or size > total - offset:
        raise MatrixError(f"{what} lies outside the file")


def _sections(blob):
    words = struct.unpack(f"<{len(blob) // 4}f", blob)
    return [words[i:i + WORDS] for i in range(0, len(words), WORDS)]


def _stable(s):
    b0, b1, b2, a1, a2 = s
    return (all(math.isfinite(v) for v in s) and abs(a2) < 1.0
            and 1.0 + a1 + a2 > 0.0 and 1.0 - a1 + a2 > 0.0)


def _pattern(s, unit, b0a2, b1a1):
    b0, b1, b2, a1, a2 = s
    return (b2 == 1.0) == unit and (b0 == a2) == b0a2 and (b1 == a1) == b1a1


def valid(blob, guard=None):
    """Whether `blob` (80 bytes) is a matrix by the rule above; with `guard`
    (the next 80 bytes of the component) the allpass guard is checked too."""
    if not isinstance(blob, (bytes, bytearray)) or len(blob) != SIZE:
        return False
    sos = _sections(bytes(blob))
    if not all(_stable(s) for s in sos):
        return False
    if not (_pattern(sos[0], True, True, False) and _pattern(sos[1], True, True, True)
            and _pattern(sos[2], True, False, True) and _pattern(sos[3], False, False, False)):
        return False
    if guard is None:
        return True
    if len(guard) != GUARD:
        return False
    return all(_stable(s) and _pattern(s, True, True, True) for s in _sections(bytes(guard)))


def sections(blob):
    """The four sections `(b0, b1, b2, a1, a2)` of a valid matrix, output-major."""
    if not valid(blob):
        raise MatrixError("not a valid BuzzKill quadrature matrix")
    return _sections(bytes(blob))


def const_sections(view):
    """(file offset, size, address) of every file-backed `__const` section of
    `__TEXT`, `__DATA_CONST` and `__DATA` in a 64-bit Mach-O slice."""
    endian = MACHO64_MAGIC.get(bytes(view[:4]))
    if endian is None:
        raise MatrixError("a slice is not a 64-bit Mach-O image")
    _within(len(view), 0, 32, "the Mach-O header")
    ncmds, sizeofcmds = struct.unpack_from(endian + "II", view, 16)
    if ncmds > MAX_COMMANDS:
        raise MatrixError(f"implausible load command count {ncmds}")
    _within(len(view), 32, sizeofcmds, "the load commands")
    pos, end = 32, 32 + sizeofcmds
    found = []
    for _ in range(ncmds):
        _within(end, pos, 8, "a load command")
        cmd, cmdsize = struct.unpack_from(endian + "II", view, pos)
        if cmdsize < 8 or cmdsize > end - pos:
            raise MatrixError("a load command has an invalid size")
        if cmd == LC_SEGMENT_64:
            if cmdsize < 72:
                raise MatrixError("a short segment command")
            nsects = struct.unpack_from(endian + "I", view, pos + 64)[0]
            if 72 + nsects * 80 > cmdsize:
                raise MatrixError("sections overrun their segment command")
            for k in range(nsects):
                base = pos + 72 + k * 80
                sect, seg = struct.unpack_from(endian + "16s16s", view, base)
                if sect.split(b"\0", 1)[0] != b"__const" or seg.split(b"\0", 1)[0] not in SEGMENTS:
                    continue
                addr, size, offset = struct.unpack_from(endian + "QQI", view, base + 32)
                flags = struct.unpack_from(endian + "I", view, base + 64)[0]
                if flags & SECTION_TYPE in ZEROFILL_TYPES:
                    continue
                _within(len(view), offset, size, "a __const section")
                found.append((offset, size, addr))
        pos += cmdsize
    if pos != end:
        raise MatrixError("the load commands do not fill their declared size")
    return found


def candidates(view):
    """Offsets in a slice of every matrix (with its guard) at a naturally
    aligned address of a `__const` section."""
    out = []
    for offset, size, addr in const_sections(view):
        stop = offset + size
        # Section 1's `b2` is a unit word two words into the table.
        at = view.find(ONE, offset + 8, stop)
        while at >= 0:
            start = at - 8
            if (addr + start - offset) % 4 == 0 and start + SIZE + GUARD <= stop \
                    and valid(view[start:start + SIZE], view[start + SIZE:start + SIZE + GUARD]):
                out.append(start)
            at = view.find(ONE, at + 1, stop)
    return out


def extract(data):
    """The matrix bytes of a component image, or MatrixError."""
    data = bytes(data)
    tables = []
    for name, offset, size in slices(data):
        view = bytes(data[offset:offset + size])
        found = candidates(view)
        if len(found) != 1:
            raise MatrixError(f"slice {name}: expected one BuzzKill quadrature matrix, found {len(found)}")
        tables.append(view[found[0]:found[0] + SIZE])
    if any(t != tables[0] for t in tables[1:]):
        raise MatrixError("the component's slices hold different BuzzKill quadrature matrices")
    return tables[0]


def read(path):
    """The matrix of the component at `path`: a regular file, not a link."""
    import os
    import stat
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    except OSError as e:
        raise MatrixError(f"cannot open {path}: {e.strerror}") from e
    with os.fdopen(fd, "rb") as f:
        if not stat.S_ISREG(os.fstat(f.fileno()).st_mode):
            raise MatrixError(f"{path} is not a regular file")
        data = f.read(MAX_COMPONENT_BYTES + 1)
    if len(data) > MAX_COMPONENT_BYTES:
        raise MatrixError(f"{path} is over budget")
    return extract(data)


def main(argv=None):
    """Checks a component and reports where the matrix is, never its values."""
    import argparse
    import hashlib
    import os
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument("component", nargs="?", default=os.path.join("/", AUDIODSP))
    parser.add_argument("--out", help="write the 80 bytes here (a private path)")
    args = parser.parse_args(argv)
    try:
        with open(args.component, "rb") as f:
            data = f.read(MAX_COMPONENT_BYTES + 1)
        if len(data) > MAX_COMPONENT_BYTES:
            raise MatrixError("over budget")
        parts = slices(data)
        for name, offset, size in parts:
            found = candidates(data[offset:offset + size])
            print(f"slice {name} at {offset:#x}: {len(found)} candidate(s)")
        table = extract(data)
    except (OSError, ValueError) as e:
        print(f"FAIL {e}")
        return 1
    if args.out:
        fd = os.open(args.out, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "wb") as f:
            f.write(table)
    print(f"PASS {len(parts)} slice(s), one identical matrix, sha256 {hashlib.sha256(table).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
