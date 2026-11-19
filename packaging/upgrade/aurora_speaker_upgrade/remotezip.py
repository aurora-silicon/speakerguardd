#!/usr/bin/python3 -I
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""remotezip: random access to the members of a zip on an HTTPS server
(Apple's IPSW CDN) through HTTP range requests, so a reader can take a few
megabytes out of a 19 GB IPSW without downloading it.

    remotezip.py URL list
"""
import struct
import sys
import urllib.request
import zlib

UA = "aurora-speaker-upgrade/1"


class RemoteZipError(Exception):
    pass


class RemoteFile:
    """A read-only, range-fetched file with a small block cache."""

    BLOCK = 1 << 20

    def __init__(self, url, timeout=60):
        self.url, self.timeout = url, timeout
        req = urllib.request.Request(url, method="HEAD", headers={"User-Agent": UA})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            self.size = int(r.headers["Content-Length"])
            if r.headers.get("Accept-Ranges", "") != "bytes":
                raise RemoteZipError("the server does not serve byte ranges")
        self.cache = {}
        self.fetched = 0

    def _block(self, i):
        if i not in self.cache:
            lo = i * self.BLOCK
            hi = min(self.size, lo + self.BLOCK) - 1
            req = urllib.request.Request(self.url, headers={"User-Agent": UA, "Range": f"bytes={lo}-{hi}"})
            for attempt in range(4):
                try:
                    with urllib.request.urlopen(req, timeout=self.timeout) as r:
                        data = r.read()
                    break
                except OSError:
                    if attempt == 3:
                        raise
            if len(data) != hi - lo + 1:
                raise RemoteZipError("short range response")
            self.fetched += len(data)
            if len(self.cache) > 512:
                self.cache.clear()
            self.cache[i] = data
        return self.cache[i]

    def pread(self, n, off):
        if off < 0 or off + n > self.size:
            raise RemoteZipError("read past the end")
        out = bytearray()
        while n:
            i, o = divmod(off, self.BLOCK)
            b = self._block(i)[o:o + n]
            out += b
            off += len(b)
            n -= len(b)
        return bytes(out)


class Member:
    def __init__(self, z, name, method, csize, usize, local_off):
        self.z, self.name, self.method = z, name, method
        self.csize, self.usize, self.local_off = csize, usize, local_off
        self._data_off = None

    @property
    def data_off(self):
        if self._data_off is None:
            h = self.z.f.pread(30, self.local_off)
            if h[:4] != b"PK\x03\x04":
                raise RemoteZipError(f"{self.name}: no local header")
            nl, xl = struct.unpack_from("<HH", h, 26)
            self._data_off = self.local_off + 30 + nl + xl
        return self._data_off

    def pread(self, n, off):
        """Random access; stored members only."""
        if self.method != 0:
            raise RemoteZipError(f"{self.name}: compressed member, no random access")
        if off + n > self.usize:
            raise RemoteZipError("read past the member's end")
        return self.z.f.pread(n, self.data_off + off)

    def read(self, limit=64 << 20):
        if self.usize > limit:
            raise RemoteZipError(f"{self.name}: {self.usize} bytes is over the limit")
        raw = self.z.f.pread(self.csize, self.data_off)
        if self.method == 0:
            return raw
        if self.method == 8:
            return zlib.decompress(raw, -15)
        raise RemoteZipError(f"{self.name}: compression method {self.method}")


class RemoteZip:
    def __init__(self, url):
        self.f = RemoteFile(url)
        tail_n = min(self.f.size, 1 << 16)
        tail = self.f.pread(tail_n, self.f.size - tail_n)
        e = tail.rfind(b"PK\x05\x06")
        if e < 0:
            raise RemoteZipError("no end of central directory")
        cd_n, cd_size, cd_off = struct.unpack_from("<HII", tail, e + 10)
        if cd_off == 0xFFFFFFFF or cd_n == 0xFFFF:
            loc = tail.rfind(b"PK\x06\x07")
            z64_off = struct.unpack_from("<Q", tail, loc + 8)[0]
            z = self.f.pread(56, z64_off)
            cd_n, cd_size, cd_off = struct.unpack_from("<QQQ", z, 32)
        cd = self.f.pread(cd_size, cd_off)
        self.members = {}
        p = 0
        for _ in range(cd_n):
            if cd[p:p + 4] != b"PK\x01\x02":
                raise RemoteZipError("bad central directory")
            method = struct.unpack_from("<H", cd, p + 10)[0]
            csize, usize = struct.unpack_from("<II", cd, p + 20)
            nl, xl, cl = struct.unpack_from("<HHH", cd, p + 28)
            off = struct.unpack_from("<I", cd, p + 42)[0]
            name = cd[p + 46:p + 46 + nl].decode("utf-8", "replace")
            x = cd[p + 46 + nl:p + 46 + nl + xl]
            q = 0
            while q + 4 <= len(x):
                hid, hl = struct.unpack_from("<HH", x, q)
                if hid == 1:
                    vals = list(struct.unpack_from(f"<{hl // 8}Q", x, q + 4))
                    if usize == 0xFFFFFFFF:
                        usize = vals.pop(0)
                    if csize == 0xFFFFFFFF:
                        csize = vals.pop(0)
                    if off == 0xFFFFFFFF:
                        off = vals.pop(0)
                q += 4 + hl
            self.members[name] = Member(self, name, method, csize, usize, off)
            p += 46 + nl + xl + cl


if __name__ == "__main__":
    z = RemoteZip(sys.argv[1])
    for m in sorted(z.members.values(), key=lambda m: -m.usize)[:40]:
        print(f"{m.usize:>14} m{m.method} {m.name}")
    print("fetched", z.f.fetched, "bytes")
