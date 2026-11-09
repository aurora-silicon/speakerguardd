# SPDX-License-Identifier: MIT
"""The lsm2 masker's bucket lattice, read from this Mac's own macOS.

The lsm2 speaker protection unit reduces a 640-point spectrum to 100
buckets. The bucket edges are a table inside macOS's AudioDSP component,
not something a formula reproduces, so the installer reads it from the
user's own system volume, as it does the speaker tuning, and never ships it.

The table is found by its structure alone, with no offset, hash or symbol:
in every Mach-O slice of the component, at naturally aligned positions of
the `__TEXT,__const` section, 100 little-endian 64-bit exclusive bucket
ends that

- start with the 64 single-bin buckets (ends 1 to 64),
- increase strictly,
- stay within the one-sided 640-point spectrum (1 to 321), and
- end at 321, its exclusive upper edge.

Exactly one such table must exist in each slice, and every slice must hold
the same bytes. Anything else (no table, two tables, a malformed slice,
slices that disagree) is an error: there is no fallback table.
"""
import struct

# The component, relative to the root of a macOS system volume.
AUDIODSP = "System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP"
# The component is about 17 MB on macOS 26 and 27.
MAX_COMPONENT_BYTES = 256 << 20

ENTRIES = 100
SINGLETONS = 64
POINTS = 640
FINAL_END = POINTS // 2 + 1
SIZE = ENTRIES * 8

# The bundle field: `ENTRIES` little-endian u64 exclusive bucket ends.
VERSION = 1
FORMAT = "u64le-exclusive-ends"

FAT_MAGIC = {
    b"\xca\xfe\xba\xbe": (">", False),
    b"\xbe\xba\xfe\xca": ("<", False),
    b"\xca\xfe\xba\xbf": (">", True),
    b"\xbf\xba\xfe\xca": ("<", True),
}
MACHO64_MAGIC = {b"\xcf\xfa\xed\xfe": "<", b"\xfe\xed\xfa\xcf": ">"}
LC_SEGMENT_64 = 0x19
SECTION_TYPE = 0xFF
ZEROFILL_TYPES = (0x1, 0xC, 0x12)
MAX_SLICES = 64
MAX_COMMANDS = 65536


class LatticeError(ValueError):
    """The component does not yield exactly one valid lattice."""


def valid(blob):
    """Whether `blob` is a lattice: the rule above, on its own bytes."""
    if not isinstance(blob, (bytes, bytearray)) or len(blob) != SIZE:
        return False
    ends = struct.unpack(f"<{ENTRIES}Q", blob)
    return (ends[:SINGLETONS] == tuple(range(1, SINGLETONS + 1))
            and all(a < b for a, b in zip(ends, ends[1:]))
            and all(1 <= e <= FINAL_END for e in ends)
            and ends[-1] == FINAL_END)


def ends(blob):
    """The 100 exclusive bucket ends of a valid lattice."""
    if not valid(blob):
        raise LatticeError("not a valid lsm2 masker lattice")
    return list(struct.unpack(f"<{ENTRIES}Q", blob))


def _within(total, offset, size, what):
    if offset < 0 or size < 0 or offset > total or size > total - offset:
        raise LatticeError(f"{what} lies outside the file")


def slices(data):
    """(name, offset, size) of each architecture slice; one for a thin file."""
    if len(data) < 8:
        raise LatticeError("the component is too short to be a Mach-O file")
    fat = FAT_MAGIC.get(bytes(data[:4]))
    if fat is None:
        return [("thin", 0, len(data))]
    endian, wide = fat
    count = struct.unpack_from(endian + "I", data, 4)[0]
    if not 0 < count <= MAX_SLICES:
        raise LatticeError(f"implausible slice count {count}")
    entry = endian + ("iiQQII" if wide else "iiIII")
    step = struct.calcsize(entry)
    _within(len(data), 8, count * step, "the slice table")
    out = []
    for i in range(count):
        cputype, subtype, offset, size = struct.unpack_from(entry, data, 8 + i * step)[:4]
        _within(len(data), offset, size, f"slice {i}")
        for _, o, s in out:
            if offset < o + s and o < offset + size:
                raise LatticeError("overlapping slices")
        out.append((f"cpu {cputype & 0xFFFFFFFF:#x}/{subtype & 0xFFFFFF}", offset, size))
    return out


def const_section(view):
    """(file offset, size, address) of the one `__TEXT,__const` section of a
    64-bit Mach-O slice."""
    endian = MACHO64_MAGIC.get(bytes(view[:4]))
    if endian is None:
        raise LatticeError("a slice is not a 64-bit Mach-O image")
    _within(len(view), 0, 32, "the Mach-O header")
    ncmds, sizeofcmds = struct.unpack_from(endian + "II", view, 16)
    if ncmds > MAX_COMMANDS:
        raise LatticeError(f"implausible load command count {ncmds}")
    _within(len(view), 32, sizeofcmds, "the load commands")
    pos, end = 32, 32 + sizeofcmds
    found = []
    for _ in range(ncmds):
        _within(end, pos, 8, "a load command")
        cmd, cmdsize = struct.unpack_from(endian + "II", view, pos)
        if cmdsize < 8 or cmdsize > end - pos:
            raise LatticeError("a load command has an invalid size")
        if cmd == LC_SEGMENT_64:
            if cmdsize < 72:
                raise LatticeError("a short segment command")
            nsects = struct.unpack_from(endian + "I", view, pos + 64)[0]
            if 72 + nsects * 80 > cmdsize:
                raise LatticeError("sections overrun their segment command")
            for k in range(nsects):
                base = pos + 72 + k * 80
                sect, seg = struct.unpack_from(endian + "16s16s", view, base)
                if (sect.split(b"\0", 1)[0], seg.split(b"\0", 1)[0]) != (b"__const", b"__TEXT"):
                    continue
                addr, size, offset = struct.unpack_from(endian + "QQI", view, base + 32)
                flags = struct.unpack_from(endian + "I", view, base + 64)[0]
                if flags & SECTION_TYPE in ZEROFILL_TYPES:
                    raise LatticeError("__TEXT,__const has no file contents")
                _within(len(view), offset, size, "__TEXT,__const")
                found.append((offset, size, addr))
        pos += cmdsize
    if pos != end:
        raise LatticeError("the load commands do not fill their declared size")
    if len(found) != 1:
        raise LatticeError(f"expected one __TEXT,__const section, found {len(found)}")
    return found[0]


def candidates(view):
    """Offsets in a slice of every valid lattice at a naturally aligned
    address of its `__TEXT,__const`."""
    offset, size, addr = const_section(view)
    prefix = struct.pack(f"<{SINGLETONS}Q", *range(1, SINGLETONS + 1))
    stop = offset + size
    out = []
    at = view.find(prefix, offset, stop)
    while at >= 0:
        if (addr + at - offset) % 8 == 0 and at + SIZE <= stop and valid(view[at:at + SIZE]):
            out.append(at)
        at = view.find(prefix, at + 1, stop)
    return out


def extract(data):
    """The lattice bytes of a component image, or LatticeError."""
    data = bytes(data)
    tables = []
    for name, offset, size in slices(data):
        view = bytes(data[offset:offset + size])
        found = candidates(view)
        if len(found) != 1:
            raise LatticeError(f"slice {name}: expected one lsm2 masker lattice in "
                               f"__TEXT,__const, found {len(found)}")
        tables.append(view[found[0]:found[0] + SIZE])
    if any(t != tables[0] for t in tables[1:]):
        raise LatticeError("the component's slices hold different lsm2 masker lattices")
    return tables[0]


def read(path):
    """The lattice of the component at `path`: a regular file, not a link."""
    import os
    import stat
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    except OSError as e:
        raise LatticeError(f"cannot open {path}: {e.strerror}") from e
    with os.fdopen(fd, "rb") as f:
        if not stat.S_ISREG(os.fstat(f.fileno()).st_mode):
            raise LatticeError(f"{path} is not a regular file")
        data = f.read(MAX_COMPONENT_BYTES + 1)
    if len(data) > MAX_COMPONENT_BYTES:
        raise LatticeError(f"{path} is over budget")
    return extract(data)


def main(argv=None):
    """Checks a component and reports where the lattice is, never its values."""
    import argparse
    import hashlib
    import os
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument("component", nargs="?", default=os.path.join("/", AUDIODSP))
    args = parser.parse_args(argv)
    try:
        with open(args.component, "rb") as f:
            data = f.read(MAX_COMPONENT_BYTES + 1)
        if len(data) > MAX_COMPONENT_BYTES:
            raise LatticeError("over budget")
        parts = slices(data)
        for name, offset, size in parts:
            found = candidates(data[offset:offset + size])
            print(f"slice {name} at {offset:#x}: {len(found)} candidate(s)")
        table = extract(data)
    except (OSError, LatticeError) as e:
        print(f"FAIL {e}")
        return 1
    print(f"PASS {len(parts)} slice(s), one identical lattice, sha256 "
          f"{hashlib.sha256(table).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
