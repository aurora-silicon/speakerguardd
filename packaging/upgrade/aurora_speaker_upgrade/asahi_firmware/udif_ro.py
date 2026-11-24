# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""udif_ro: random, read-only access to the APFS partition inside a UDIF
disk image (.dmg), the root file system of restore images that predate
Apple Encrypted Archives (macOS 13 and earlier), over any source with
pread(n, offset) (a stored zip member read by HTTP ranges).

Written from the public description of the format: a 512-byte `koly`
trailer names an XML property list whose `blkx` entries each describe one
partition as `mish` block tables of chunks (zero fill, raw, zlib, LZFSE).
Only the chunks a read touches are fetched and decoded, a few at a time.
Every count, size and offset is checked before use; anything else is
UdifError.
"""
import plistlib
import struct
import zlib

from . import apfs_ro

SECTOR = 512
KOLY = 512
MAX_XML = 16 << 20
MAX_CHUNKS = 1 << 20
MAX_CHUNK_OUT = 64 << 20
ZERO, RAW, IGNORE, ZLIB, LZFSE, COMMENT, END = 0, 1, 2, 0x80000005, 0x80000007, 0x7FFFFFFE, 0xFFFFFFFF
CACHE = 8


class UdifError(Exception):
    pass


class Partition:
    """One partition of the image (default: the Apple_APFS one), as a
    pread source in partition-relative bytes."""

    def __init__(self, src, size, name="Apple_APFS"):
        self.src = src
        if size < KOLY:
            raise UdifError("too small for a UDIF image")
        k = src.pread(KOLY, size - KOLY)
        if len(k) != KOLY or k[:4] != b"koly":
            raise UdifError("no UDIF trailer")
        data_off, data_len = struct.unpack_from(">QQ", k, 24)
        xml_off, xml_len = struct.unpack_from(">QQ", k, 216)
        if not (0 < xml_len <= MAX_XML and xml_off + xml_len <= size and data_off + data_len <= size):
            raise UdifError("UDIF trailer out of bounds")
        # Read outside the parse: a transfer that fails here is the
        # source's error (unavailable, retried later), not a bad image.
        xml = src.pread(xml_len, xml_off)
        if len(xml) != xml_len:
            raise UdifError("short read of the UDIF property list")
        try:
            doc = plistlib.loads(xml)
            blkx = doc["resource-fork"]["blkx"]
        except Exception as e:
            raise UdifError(f"UDIF property list: {e}") from None
        hits = [b for b in blkx if name in str(b.get("Name", ""))]
        if len(hits) != 1:
            raise UdifError(f"{len(hits)} {name} partitions in the image")
        mish = hits[0].get("Data")
        if not isinstance(mish, bytes) or len(mish) < 204 or mish[:4] != b"mish":
            raise UdifError("malformed block table")
        first, count, base = struct.unpack_from(">QQQ", mish, 8)
        n = struct.unpack_from(">I", mish, 200)[0]
        if n > MAX_CHUNKS or len(mish) < 204 + 40 * n:
            raise UdifError("block table out of bounds")
        self.chunks = []
        end = 0
        for i in range(n):
            kind, _, sec, cnt, coff, clen = struct.unpack_from(">IIQQQQ", mish, 204 + 40 * i)
            if kind in (COMMENT, END):
                continue
            if sec != end or cnt == 0:
                raise UdifError("block table is not contiguous")
            if kind not in (ZERO, RAW, IGNORE, ZLIB, LZFSE):
                raise UdifError(f"chunk type {kind:#x} not supported")
            # Zero-fill, ignored and raw chunks are read in place; a
            # compressed chunk is decoded whole, so its size is capped.
            if kind in (ZLIB, LZFSE) and cnt * SECTOR > MAX_CHUNK_OUT:
                raise UdifError("chunk over the decode budget")
            if kind in (RAW, ZLIB, LZFSE) and data_off + base + coff + clen > data_off + data_len:
                raise UdifError("chunk outside the data fork")
            self.chunks.append((sec, cnt, kind, data_off + base + coff, clen))
            end = sec + cnt
        if end != count:
            raise UdifError("block table does not cover the partition")
        self.size = count * SECTOR
        self.starts = [c[0] for c in self.chunks]
        self._cache = {}

    def _chunk(self, i):
        if i in self._cache:
            return self._cache[i]
        sec, cnt, kind, off, clen = self.chunks[i]
        out = cnt * SECTOR
        raw = self.src.pread(clen, off)
        if len(raw) != clen:
            raise UdifError("short read")
        if kind == ZLIB:
            data = zlib.decompressobj().decompress(raw, out + 1)
        else:
            try:
                data = apfs_ro.lzfse_decode(raw, out)
            except apfs_ro.ApfsError as e:
                raise UdifError(f"chunk {i}: {e}") from None
        if len(data) != out:
            raise UdifError(f"chunk {i}: {len(data)} bytes decoded, {out} expected")
        if len(self._cache) >= CACHE:
            self._cache.pop(next(iter(self._cache)))
        self._cache[i] = data
        return data

    def pread(self, n, off):
        import bisect
        if off < 0 or n < 0 or off + n > self.size:
            raise UdifError("read outside the partition")
        out = bytearray()
        while n:
            i = bisect.bisect_right(self.starts, off // SECTOR) - 1
            sec, cnt, kind = self.chunks[i][:3]
            o = off - sec * SECTOR
            take = min(n, cnt * SECTOR - o)
            if kind in (ZERO, IGNORE):
                out += bytes(take)
            elif kind == RAW:
                b = self.src.pread(take, self.chunks[i][3] + o)
                if len(b) != take:
                    raise UdifError("short read")
                out += b
            else:
                out += self._chunk(i)[o:o + take]
            off, n = off + take, n - take
        return bytes(out)
