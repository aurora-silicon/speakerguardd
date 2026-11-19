#!/usr/bin/python3 -I
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""adt_ro: read the Apple device tree (ADT) that the boot chain hands to
Linux, read only.

The boot chain exposes the ADT iBoot gave it as the MTD partition "adt"
(/proc/mtd); this reads it through the read-only node (/dev/mtdNro) or a
file, and returns nodes as {"name", "props", "children"}.

    adt_ro.py [FILE] find NODE-NAME [PROP]
"""
import os
import struct
import sys

MAX_ADT = 8 << 20


class AdtError(Exception):
    pass


def mtd_path(name="adt", proc="/proc/mtd"):
    try:
        with open(proc) as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 4 and parts[3].strip('"') == name:
                    return f"/dev/{parts[0].rstrip(':')}ro"
    except OSError:
        pass
    return None


def load(path=None):
    path = path or mtd_path()
    if not path:
        raise AdtError("the boot chain exposes no ADT (no 'adt' MTD partition)")
    with open(path, "rb") as f:
        data = f.read(MAX_ADT)
    node, end = parse(data, 0, 0)
    return node


def parse(data, off, depth):
    if depth > 32 or off + 8 > len(data):
        raise AdtError("malformed ADT")
    nprops, nchildren = struct.unpack_from("<II", data, off)
    if nprops > 4096 or nchildren > 4096:
        raise AdtError("malformed ADT node")
    off += 8
    props = {}
    for _ in range(nprops):
        if off + 36 > len(data):
            raise AdtError("truncated ADT property")
        name = data[off:off + 32].split(b"\0", 1)[0].decode("ascii", "replace")
        size = struct.unpack_from("<I", data, off + 32)[0] & 0x7FFFFFFF
        off += 36
        if off + size > len(data):
            raise AdtError("truncated ADT property value")
        props[name] = data[off:off + size]
        off += (size + 3) & ~3
    children = []
    for _ in range(nchildren):
        child, off = parse(data, off, depth + 1)
        children.append(child)
    name = props.get("name", b"").split(b"\0", 1)[0].decode("ascii", "replace")
    return {"name": name, "props": props, "children": children}, off


def walk(node, path=""):
    p = f"{path}/{node['name']}" if path else node["name"]
    yield p, node
    for c in node["children"]:
        yield from walk(c, p)


def find(root, name):
    return [(p, n) for p, n in walk(root) if n["name"] == name]


if __name__ == "__main__":
    args = sys.argv[1:]
    src = args.pop(0) if args and os.path.exists(args[0]) else None
    root = load(src)
    if args[:1] == ["find"]:
        for p, n in find(root, args[1]):
            if len(args) > 2:
                v = n["props"].get(args[2])
                print(p, args[2], None if v is None else v.hex())
            else:
                print(p, sorted(n["props"]))
