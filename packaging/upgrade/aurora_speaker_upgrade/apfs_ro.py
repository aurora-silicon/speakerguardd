#!/usr/bin/python3 -I
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""apfs_ro: a minimal read-only APFS reader for the macOS system volume.

Reads the sealed macOS System volume of an Apple silicon Mac from Linux,
straight from the block device, opened O_RDONLY. It never writes, never
mounts and needs no kernel module or FUSE. Written from Apple's public
"Apple File System Reference".

Scope, on purpose small: the newest checkpoint, the container object map,
volumes that are not encrypted (APFS_FS_UNENCRYPTED: the System, Preboot,
Recovery and Update volumes; the Data volume is encrypted by the Secure
Enclave and is refused), the file-system tree (hashed, header-less sealed
nodes included), the sealed volume's file-extent tree, directory
records, inodes, extended attributes and decmpfs compression (zlib, LZVN,
LZFSE, uncompressed; in the attribute or the resource fork). LZVN is
decoded here; LZFSE frames go through liblzfse.

    apfs_ro.py volumes DEV
    apfs_ro.py ls  DEV PATH [--volume NAME|--role System]
    apfs_ro.py cat DEV PATH [--volume ...]   > file
    apfs_ro.py copy DEV PATH DEST [--volume ...]   (a directory tree, or a file)
"""
import argparse
import ctypes
import ctypes.util
import os
import stat
import struct
import sys
import zlib

ROLE_SYSTEM, ROLE_DATA = 1, 0x40
ROLES = {0: "none", 1: "System", 2: "User", 4: "Recovery", 8: "VM", 0x10: "Preboot",
         0x20: "Installer", 0x40: "Data", 0x80: "Baseband", 0xC0: "Update", 0x100: "xART",
         0x140: "Hardware", 0x180: "Backup", 0x200: "Enterprise", 0x280: "Prelogin"}
FS_UNENCRYPTED = 0x1
BTNODE_ROOT, BTNODE_LEAF, BTNODE_FIXED, BTNODE_HASHED, BTNODE_NOHEADER = 1, 2, 4, 8, 0x10
BTREE_PHYSICAL = 0x10
OMAP_VAL_DELETED, OMAP_VAL_ENCRYPTED, OMAP_VAL_NOHEADER = 1, 4, 8
T_INODE, T_XATTR, T_EXTENT, T_DREC = 3, 4, 8, 9
INO_EXT_TYPE_DSTREAM = 8
XATTR_DATA_STREAM, XATTR_DATA_EMBEDDED = 1, 2
ROOT_DIR_INO = 2
DECMPFS_MAGIC = 0x636D7066  # 'fpmc'
MAX_FILE = 256 << 20


class ApfsError(Exception):
    pass


def fletcher64(block):
    s1 = s2 = 0
    for (w,) in struct.iter_unpack("<I", block[8:]):
        s1 = (s1 + w) % 0xFFFFFFFF
        s2 = (s2 + s1) % 0xFFFFFFFF
    c1 = 0xFFFFFFFF - ((s1 + s2) % 0xFFFFFFFF)
    c2 = 0xFFFFFFFF - ((s1 + c1) % 0xFFFFFFFF)
    return (c2 << 32) | c1


_lzfse = None


def lzfse():
    global _lzfse
    if _lzfse is None:
        # AURORA_LZFSE_LIB: distributions without a library search path (NixOS).
        name = os.environ.get("AURORA_LZFSE_LIB") or ctypes.util.find_library("lzfse") or "liblzfse.so.1"
        try:
            lib = ctypes.CDLL(name)
        except OSError:
            lib = ctypes.CDLL("liblzfse.so")
        lib.lzfse_decode_buffer.restype = ctypes.c_size_t
        lib.lzfse_decode_buffer.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_char_p,
                                            ctypes.c_size_t, ctypes.c_void_p]
        lib.lzfse_decode_scratch_size.restype = ctypes.c_size_t
        _lzfse = lib
    return _lzfse


def lzfse_decode(data, size):
    if sys.platform == "darwin":  # development hosts: the system's libcompression
        lib = ctypes.CDLL("/usr/lib/libcompression.dylib")
        lib.compression_decode_buffer.restype = ctypes.c_size_t
        out = ctypes.create_string_buffer(size + 1)
        n = lib.compression_decode_buffer(out, ctypes.c_size_t(size + 1), data, ctypes.c_size_t(len(data)),
                                          None, ctypes.c_int(0x801))
        if n != size:
            raise ApfsError(f"lzfse: {n} bytes decoded, {size} expected")
        return out.raw[:size]
    lib = lzfse()
    out = ctypes.create_string_buffer(size + 1)
    scratch = ctypes.create_string_buffer(lib.lzfse_decode_scratch_size())
    n = lib.lzfse_decode_buffer(out, size + 1, data, len(data), scratch)
    if n != size:
        raise ApfsError(f"lzfse: {n} bytes decoded, {size} expected")
    return out.raw[:size]


def lzvn_decode(src, size):
    """A raw LZVN stream (decmpfs types 7 and 8) to `size` bytes."""
    out = bytearray()
    i, d, n = 0, 0, len(src)

    def need(k):
        if i + k > n:
            raise ApfsError("lzvn: truncated stream")

    while True:
        need(1)
        op = src[i]
        L = M = 0
        if op == 0x06:
            break
        if op in (0x0E, 0x16):
            i += 1
            continue
        if op >= 0xF0:  # match with the previous distance
            if op == 0xF0:
                need(2)
                M, i = src[i + 1] + 16, i + 2
            else:
                M, i = op & 0xF, i + 1
        elif op >= 0xE0:  # literals
            if op == 0xE0:
                need(2)
                L, i = src[i + 1] + 16, i + 2
            else:
                L, i = op & 0xF, i + 1
        elif 0xA0 <= op < 0xC0:  # medium distance
            need(3)
            b1, b2 = src[i + 1], src[i + 2]
            L = (op >> 3) & 3
            M = (((op & 7) << 2) | (b1 & 3)) + 3
            d = (b1 >> 2) | (b2 << 6)
            i += 3
        elif (0x70 <= op < 0x80) or op in (0x1E, 0x26, 0x2E, 0x36, 0x3E):
            raise ApfsError(f"lzvn: undefined opcode {op:#x}")
        else:
            L = (op >> 6) & 3
            M = ((op >> 3) & 7) + 3
            low = op & 7
            if low == 7:  # large distance
                need(3)
                d = src[i + 1] | (src[i + 2] << 8)
                i += 3
            elif low == 6:  # previous distance
                i += 1
            else:  # small distance
                need(2)
                d = (low << 8) | src[i + 1]
                i += 2
        if L:
            need(L)
            out += src[i:i + L]
            i += L
        if M:
            if d == 0 or d > len(out):
                raise ApfsError("lzvn: match before the start of the output")
            start = len(out) - d
            for j in range(M):
                out.append(out[start + j])
        if len(out) > size:
            raise ApfsError("lzvn: output over the expected size")
    if len(out) != size:
        raise ApfsError(f"lzvn: {len(out)} bytes decoded, {size} expected")
    return bytes(out)


class Device:
    """A block device or image, opened O_RDONLY, or any object with
    pread(n, offset) (aea_ro.Archive: an IPSW's decoded root image)."""

    def __init__(self, path):
        if hasattr(path, "pread"):
            self.src = path
        else:
            fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC)
            self.src = type("_Fd", (), {"pread": staticmethod(lambda n, off: os.pread(fd, n, off))})
        self.bs = 4096

    def read(self, paddr, n=1):
        b = self.src.pread(self.bs * n, paddr * self.bs)
        if len(b) != self.bs * n:
            raise ApfsError(f"short read at block {paddr:#x}")
        return b

    def obj(self, paddr, noheader=False):
        b = self.read(paddr)
        if not noheader and struct.unpack_from("<Q", b, 0)[0] != fletcher64(b):
            raise ApfsError(f"object at block {paddr:#x}: bad checksum")
        return b


class Node:
    """One b-tree node: (key bytes, value bytes) pairs."""

    def __init__(self, b, bs):
        self.flags, self.level, self.nkeys = struct.unpack_from("<HHI", b, 32)
        toff, tlen = struct.unpack_from("<HH", b, 40)
        if self.flags & ~0x1F or self.level > 16 or self.nkeys > bs // 4 or 56 + toff + tlen > bs:
            raise ApfsError("not a b-tree node (encrypted or damaged)")
        self.b = b
        kstart = 56 + toff + tlen
        vend = bs - (40 if self.flags & BTNODE_ROOT else 0)
        self.info = b[bs - 40:] if self.flags & BTNODE_ROOT else None
        self.kv = []
        for i in range(self.nkeys):
            if self.flags & BTNODE_FIXED:
                k, v = struct.unpack_from("<HH", b, 56 + toff + 4 * i)
                kl, vl = 16, 16 if self.level == 0 else 8
            else:
                k, kl, v, vl = struct.unpack_from("<HHHH", b, 56 + toff + 8 * i)
            if v == 0xFFFF:
                continue
            self.kv.append((b[kstart + k:kstart + k + kl], b[vend - v:vend - v + vl]))


class Omap:
    def __init__(self, dev, paddr):
        om = dev.obj(paddr)
        self.dev = dev
        self.tree = struct.unpack_from("<Q", om, 48)[0]

    def lookup(self, oid, xid):
        paddr = self.tree
        for _ in range(16):
            n = Node(self.dev.obj(paddr), self.dev.bs)
            best = None
            for k, v in n.kv:
                ko, kx = struct.unpack_from("<QQ", k)
                if (ko, kx) <= (oid, xid):
                    best = (ko, v)
                else:
                    break
            if best is None:
                break
            if n.level == 0:
                if best[0] != oid:
                    break
                flags, size, p = struct.unpack("<IIQ", best[1])
                if flags & OMAP_VAL_DELETED:
                    break
                return p, flags
            paddr = struct.unpack_from("<Q", best[1])[0]
        raise ApfsError(f"object {oid:#x} not in the object map")


def jkey(k):
    v = struct.unpack_from("<Q", k)[0]
    return v & 0x0FFFFFFFFFFFFFFF, v >> 60


class Volume:
    def __init__(self, c, index, sb, paddr):
        self.c, self.dev, self.index = c, c.dev, index
        self.xid = struct.unpack_from("<Q", sb, 16)[0]
        self.omap_oid, self.root_oid = struct.unpack_from("<QQ", sb, 128)
        self.fs_flags = struct.unpack_from("<Q", sb, 264)[0]
        self.incompat = struct.unpack_from("<Q", sb, 56)[0]
        self.name = sb[704:960].split(b"\0", 1)[0].decode("utf-8", "replace")
        self.role = struct.unpack_from("<H", sb, 964)[0]
        self.fext_oid = struct.unpack_from("<Q", sb, 1032)[0]
        self.uuid = sb[240:256]
        self.files = struct.unpack_from("<Q", sb, 184)[0]
        self._omap = None

    @property
    def encrypted(self):
        return not self.fs_flags & FS_UNENCRYPTED

    def omap(self):
        if self._omap is None:
            self._omap = Omap(self.dev, self.omap_oid)
        return self._omap

    def node(self, oid, physical):
        if physical:
            return Node(self._read_phys(oid), self.dev.bs)
        p, flags = self.omap().lookup(oid, self.xid)
        if flags & OMAP_VAL_ENCRYPTED:
            raise ApfsError(f"volume '{self.name}' is encrypted (Secure Enclave key); not readable here")
        return Node(self.dev.obj(p, noheader=bool(flags & OMAP_VAL_NOHEADER)), self.dev.bs)

    def _read_phys(self, paddr):
        b = self.dev.read(paddr)
        if any(b[:8]) and struct.unpack_from("<Q", b, 0)[0] != fletcher64(b):
            raise ApfsError(f"node at block {paddr:#x}: bad checksum")
        return b

    def records(self, oid, typ):
        """Every fs-tree record (key, value) of object `oid`, type `typ`."""
        if self.encrypted:
            raise ApfsError(f"volume '{self.name}' is encrypted (Secure Enclave key); not readable here")
        root = self.node(self.root_oid, False)
        physical = bool(struct.unpack_from("<I", root.info, 0)[0] & BTREE_PHYSICAL) if root.info else False
        want = (oid, typ)
        out = []

        def walk(n, depth):
            if depth > 16:
                raise ApfsError("fs tree too deep")
            keys = [jkey(k) for k, _ in n.kv]
            for i, (k, v) in enumerate(n.kv):
                if n.level == 0:
                    if keys[i] == want:
                        out.append((k, v))
                    continue
                nxt = keys[i + 1] if i + 1 < len(keys) else None
                if keys[i] <= want and (nxt is None or nxt >= want):
                    child = struct.unpack_from("<Q", v)[0]
                    if relative:
                        child += self.root_oid
                    cn = self.node(child, physical)
                    if cn.kv and jkey(cn.kv[0][0]) < keys[i]:
                        raise ApfsError(f"fs tree child {child:#x} does not follow its index key")
                    walk(cn, depth + 1)

        # The sealed system volume's header-less tree stores each child as
        # an offset from the root's virtual object ID (seen on macOS 26:
        # child = root_tree_oid + value; every child's first key checks).
        relative = bool(root.flags & BTNODE_NOHEADER) and not physical
        walk(root, 0)
        return out

    # --------------------------------------------------------- objects

    def children(self, ino):
        out = {}
        for k, v in self.records(ino, T_DREC):
            if self.hashed_names:
                nl = struct.unpack_from("<I", k, 8)[0] & 0x3FF
                name = k[12:12 + nl]
            else:
                nl = struct.unpack_from("<H", k, 8)[0]
                name = k[10:10 + nl]
            fid, _, flags = struct.unpack_from("<QQH", v)
            out[name.rstrip(b"\0").decode("utf-8", "surrogateescape")] = (fid, flags & 0xF)
        return out

    @property
    def hashed_names(self):
        return bool(self.incompat & (0x1 | 0x8))  # case- or normalization-insensitive

    def inode(self, ino):
        recs = self.records(ino, T_INODE)
        if not recs:
            raise ApfsError(f"inode {ino} not found")
        v = recs[0][1]
        private_id = struct.unpack_from("<Q", v, 8)[0]
        bsd_flags = struct.unpack_from("<I", v, 68)[0]
        mode = struct.unpack_from("<H", v, 80)[0]
        size = None
        if len(v) > 92:
            nx, used = struct.unpack_from("<HH", v, 92)
            off = 96 + 4 * nx
            for i in range(nx):
                t, f, s = struct.unpack_from("<BBH", v, 96 + 4 * i)
                if t == INO_EXT_TYPE_DSTREAM:
                    size = struct.unpack_from("<Q", v, off)[0]
                off += (s + 7) & ~7
        return {"private_id": private_id, "mode": mode, "size": size or 0,
                "compressed": bool(bsd_flags & 0x20)}

    def xattrs(self, ino):
        out = {}
        for k, v in self.records(ino, T_XATTR):
            nl = struct.unpack_from("<H", k, 8)[0]
            name = k[10:10 + nl].rstrip(b"\0").decode("utf-8", "replace")
            flags, dl = struct.unpack_from("<HH", v)
            out[name] = (flags, v[4:4 + dl])
        return out

    def extents(self, private_id):
        """[(logical, length, physical block)] of a data stream."""
        exts = []
        if self.fext_oid:
            # Sealed volume: a physical, fixed-size file-extent tree.
            def walk(paddr, depth):
                n = Node(self._read_phys(paddr), self.dev.bs)
                keys = [struct.unpack_from("<Q", k)[0] for k, _ in n.kv]
                for i, (k, v) in enumerate(n.kv):
                    if n.level == 0:
                        if keys[i] == private_id:
                            la = struct.unpack_from("<Q", k, 8)[0]
                            lf, pb = struct.unpack_from("<QQ", v)
                            exts.append((la, lf & 0x00FFFFFFFFFFFFFF, pb))
                        continue
                    nxt = keys[i + 1] if i + 1 < len(keys) else None
                    if keys[i] <= private_id and (nxt is None or nxt >= private_id):
                        walk(struct.unpack_from("<Q", v)[0], depth + 1)
            walk(self.fext_oid, 0)
        else:
            for k, v in self.records(private_id, T_EXTENT):
                la = struct.unpack_from("<Q", k, 8)[0]
                lf, pb, crypto = struct.unpack_from("<QQQ", v)
                exts.append((la, lf & 0x00FFFFFFFFFFFFFF, pb))
        return sorted(exts)

    def stream(self, private_id, size):
        if size > MAX_FILE:
            raise ApfsError(f"stream of {size} bytes is over the limit")
        buf = bytearray(size)
        for la, ln, pb in self.extents(private_id):
            if la >= size:
                continue
            n = min(ln, size - la)
            if pb == 0:
                continue  # sparse
            blocks = (n + self.dev.bs - 1) // self.dev.bs
            buf[la:la + n] = self.dev.read(pb, blocks)[:n]
        return bytes(buf)

    def read_file(self, ino):
        info = self.inode(ino)
        if not stat.S_ISREG(info["mode"]):
            raise ApfsError(f"inode {ino} is not a regular file")
        xa = self.xattrs(ino) if info["compressed"] else {}
        if "com.apple.decmpfs" in xa:
            return self._decmpfs(ino, xa)
        return self.stream(info["private_id"], info["size"])

    def _xattr_data(self, flags, data):
        if flags & XATTR_DATA_EMBEDDED:
            return data
        oid = struct.unpack_from("<Q", data)[0]
        size = struct.unpack_from("<Q", data, 8)[0]
        return self.stream(oid, size)

    def _decmpfs(self, ino, xa):
        flags, hdr = xa["com.apple.decmpfs"]
        hdr = self._xattr_data(flags, hdr)
        magic, ctype, usize = struct.unpack_from("<IIQ", hdr)
        if magic != DECMPFS_MAGIC or usize > MAX_FILE:
            raise ApfsError(f"inode {ino}: bad decmpfs header")
        body = hdr[16:]
        if ctype in (1, 9):
            return body[:usize]
        if ctype == 3:
            return body[1:1 + usize] if body[:1] == b"\xff" else zlib.decompress(body)[:usize]
        if ctype == 7:
            return body[1:1 + usize] if body[:1] == b"\x06" else lzvn_decode(body, usize)
        if ctype == 11:
            return body[1:1 + usize] if body[:1] == b"\xff" else lzfse_decode(body, usize)
        if ctype not in (4, 8, 12):
            raise ApfsError(f"inode {ino}: decmpfs type {ctype} not supported")
        if "com.apple.ResourceFork" not in xa:
            raise ApfsError(f"inode {ino}: compressed data in a missing resource fork")
        rsrc = self._xattr_data(*xa["com.apple.ResourceFork"])
        out = bytearray()
        chunk = 0x10000
        if ctype == 4:
            base = struct.unpack_from(">I", rsrc, 0)[0] + 4
            count = struct.unpack_from("<I", rsrc, base)[0]
            for i in range(count):
                off, ln = struct.unpack_from("<II", rsrc, base + 4 + 8 * i)
                c = rsrc[base + off:base + off + ln]
                want = min(chunk, usize - len(out))
                out += c[1:1 + want] if c[:1] == b"\xff" else zlib.decompress(c)
        else:
            first = struct.unpack_from("<I", rsrc, 0)[0]
            offs = [struct.unpack_from("<I", rsrc, 4 * i)[0] for i in range(first // 4)]
            for a, b in zip(offs, offs[1:]):
                c = rsrc[a:b]
                want = min(chunk, usize - len(out))
                raw = b"\x06" if ctype == 8 else b"\xff"
                if c[:1] == raw:
                    out += c[1:1 + want]
                elif ctype == 8:
                    out += lzvn_decode(c, want)
                else:
                    out += lzfse_decode(c, want)
        if len(out) != usize:
            raise ApfsError(f"inode {ino}: {len(out)} bytes decompressed, {usize} expected")
        return bytes(out)

    def readlink(self, ino):
        """A symbolic link's target (the com.apple.fs.symlink attribute)."""
        xa = self.xattrs(ino)
        if "com.apple.fs.symlink" not in xa:
            raise ApfsError(f"inode {ino}: not a symbolic link")
        return self._xattr_data(*xa["com.apple.fs.symlink"]).split(b"\0", 1)[0].decode("utf-8", "surrogateescape")

    def resolve(self, path):
        ino, kind = ROOT_DIR_INO, 4
        for part in [p for p in path.split("/") if p]:
            kids = self.children(ino)
            hit = kids.get(part)
            if hit is None and self.incompat & 0x1:
                low = {k.casefold(): v for k, v in kids.items()}
                hit = low.get(part.casefold())
            if hit is None:
                raise ApfsError(f"{path}: {part} not found")
            ino, kind = hit
        return ino, kind


class Container:
    def __init__(self, path):
        self.dev = Device(path)
        b0 = self.dev.read(0)
        if b0[32:36] != b"NXSB":
            raise ApfsError("not an APFS container")
        self.dev.bs = struct.unpack_from("<I", b0, 36)[0]
        b0 = self.dev.obj(0)
        best, bx = b0, struct.unpack_from("<Q", b0, 16)[0]
        desc_blocks = struct.unpack_from("<I", b0, 104)[0]
        desc_base = struct.unpack_from("<Q", b0, 112)[0]
        if not desc_blocks & 0x80000000:
            for i in range(desc_blocks & 0x7FFFFFFF):
                try:
                    b = self.dev.obj(desc_base + i)
                except ApfsError:
                    continue
                x = struct.unpack_from("<Q", b, 16)[0]
                if b[32:36] == b"NXSB" and x > bx:
                    best, bx = b, x
        self.nx, self.xid = best, bx
        self.omap = Omap(self.dev, struct.unpack_from("<Q", best, 160)[0])

    def volumes(self):
        out = []
        maxfs = min(struct.unpack_from("<I", self.nx, 180)[0], 100)
        for i in range(maxfs):
            oid = struct.unpack_from("<Q", self.nx, 184 + 8 * i)[0]
            if not oid:
                continue
            p, _ = self.omap.lookup(oid, self.xid)
            sb = self.dev.obj(p)
            if sb[32:36] != b"APSB":
                raise ApfsError(f"volume {i}: no superblock")
            out.append(Volume(self, i, sb, p))
        return out

    def volume(self, name=None, role=None):
        for v in self.volumes():
            if (name and v.name == name) or (role is not None and v.role == role):
                return v
        raise ApfsError(f"no volume {name or ROLES.get(role, role)}")


def copy_tree(vol, path, dest, limit=MAX_FILE):
    """Copies a file or a directory tree out of the volume. Returns the
    number of files. Symlinks and special files are skipped."""
    ino, kind = vol.resolve(path)
    count = 0

    def rec(ino, kind, dst):
        nonlocal count
        if kind == 4:  # DT_DIR
            os.makedirs(dst, mode=0o700, exist_ok=True)
            for name, (cino, ck) in sorted(vol.children(ino).items()):
                if "/" in name or name in (".", ".."):
                    continue
                rec(cino, ck, os.path.join(dst, name))
        elif kind == 8:  # DT_REG
            data = vol.read_file(ino)
            fd = os.open(dst, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, "wb") as f:
                f.write(data)
            count += 1

    rec(ino, kind, dest)
    return count


def main(argv=None):
    ap = argparse.ArgumentParser(prog="apfs_ro")
    ap.add_argument("cmd", choices=["volumes", "ls", "cat", "copy"])
    ap.add_argument("dev")
    ap.add_argument("path", nargs="?", default="/")
    ap.add_argument("dest", nargs="?")
    ap.add_argument("--volume")
    ap.add_argument("--role", default="System")
    a = ap.parse_args(argv)
    c = Container(a.dev)
    if a.cmd == "volumes":
        for v in c.volumes():
            print(f"{v.index}\t{ROLES.get(v.role, hex(v.role))}\t{'encrypted' if v.encrypted else 'plain'}\t{v.name}")
        return 0
    role = {n: r for r, n in ROLES.items()}[a.role]
    vol = c.volume(name=a.volume) if a.volume else c.volume(role=role)
    if a.cmd == "ls":
        ino, kind = vol.resolve(a.path)
        for name, (cino, ck) in sorted(vol.children(ino).items()):
            print(f"{'d' if ck == 4 else 'l' if ck == 10 else '-'} {cino}\t{name}")
    elif a.cmd == "cat":
        ino, _ = vol.resolve(a.path)
        sys.stdout.buffer.write(vol.read_file(ino))
    else:
        print(copy_tree(vol, a.path, a.dest), "files")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ApfsError as e:
        print(f"apfs_ro: {e}", file=sys.stderr)
        sys.exit(1)
