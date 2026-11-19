#!/usr/bin/python3 -I
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Eryk Wieliczko (macos-firmware-extract, the AEA range path)
# Copyright (c) 2026 The Aurora Silicon Contributors
"""aea_ro: random, read-only access to the decoded disk image inside an Apple
Encrypted Archive (AEA) member of a restore image (IPSW), by HTTP range
requests, without downloading or decrypting the whole archive.

A Python port of the authenticated range path of the aurora-installer's
restore image helper (macOS/RestoreImage, the Swift port), whose pinned
reference is Eryk Wieliczko's macos-firmware-extract at
71b413af680edec1808a709cb7addfbde5c21cfc (ranges.go, main.go: metadata,
keyURL, fetchKey, unwrap, deriveRange, rangeMAC, rangeCTR, decodeRanges,
boundedLZFSE). Where the reference decodes the segments a recipe names, this
walks the archive's cluster headers itself and decodes any segment on
demand, so a reader (apfs_ro) can open the APFS image inside.

Every byte is authenticated before use exactly as in the reference: the
root header and each cluster header by HMAC-SHA256 chained from the
archive's root, each segment by its HMAC, and each decoded segment by its
SHA-256. The archive key comes only from Apple's FCS key service over HTTPS
(the two hosts the reference admits), unwrapped with HPKE (RFC 9180, base
mode, DHKEM(P-256, HKDF-SHA256), HKDF-SHA256, AES-256-GCM). Nothing about
the key (its URL, name or bytes) is ever printed or logged.
"""
import base64
import hashlib
import hmac
import json
import os
import re
import struct
import sys
import urllib.parse
import urllib.request

SEGMENT = 1 << 20
CLUSTER_WIDTH = 256
CLUSTER_HEADER = CLUSTER_WIDTH * 72 + 32
MAX_METADATA = 1 << 20
UA = "aurora-speaker-upgrade/1"
KEY_HOSTS = ("wkms-public.apple.com", "fcs-keys-pub-prod.cdn-apple.com")
KEY_PATH = re.compile(r"\A/fcs-keys/[A-Za-z0-9_-]{43}=\Z")


class AeaError(Exception):
    pass


# --- HTTP ranges -------------------------------------------------------------

class Ranges:
    """Exact byte ranges of one HTTPS object (the reference's fetchRange)."""

    def __init__(self, url, timeout=60, opener=None):
        u = urllib.parse.urlsplit(url)
        if u.scheme != "https" or not (u.hostname or "").endswith(".cdn-apple.com"):
            raise AeaError("the restore image must come from Apple's CDN over HTTPS")
        self.url, self.timeout = url, timeout
        self.open = opener or urllib.request.urlopen
        req = urllib.request.Request(url, method="HEAD", headers={"User-Agent": UA})
        with self.open(req, timeout=timeout) as r:
            self.size = int(r.headers["Content-Length"])
        self.fetched = 0

    def get(self, off, n):
        if n <= 0 or off < 0 or off + n > self.size:
            raise AeaError("range outside the restore image")
        lo, hi = off, off + n - 1
        last = None
        for _ in range(3):
            req = urllib.request.Request(self.url, headers={
                "User-Agent": UA, "Range": f"bytes={lo}-{hi}", "Accept-Encoding": "identity"})
            try:
                with self.open(req, timeout=self.timeout) as r:
                    if r.status != 206 or r.headers.get("Content-Range") != f"bytes {lo}-{hi}/{self.size}" \
                            or r.headers.get("Content-Encoding", "identity") != "identity":
                        raise AeaError("Apple did not return the exact requested range")
                    data = r.read(n + 1)
                if len(data) != n:
                    raise AeaError("short range response")
                self.fetched += n
                return data
            except (OSError, AeaError) as e:
                last = e
        raise AeaError(f"range fetch failed: {last}")


class Slice:
    """A window [base, base+size) of another range source."""

    def __init__(self, src, base, size):
        self.src, self.base, self.size = src, base, size

    def get(self, off, n):
        if off < 0 or off + n > self.size:
            raise AeaError("read past the member")
        return self.src.get(self.base + off, n)


# --- primitives (reference: deriveRange, rangeMAC, rangeCTR) ------------------

def hkdf_extract(salt, ikm):
    return hmac.new(salt or b"\0" * 32, ikm, hashlib.sha256).digest()


def hkdf_expand(prk, info, n):
    out, t, i = b"", b"", 1
    while len(out) < n:
        t = hmac.new(prk, t + info + bytes([i]), hashlib.sha256).digest()
        out += t
        i += 1
    return out[:n]


def derive(key, salt, info, n):
    return hkdf_expand(hkdf_extract(salt, key), info, n)


def indexed(name, index):
    return name + struct.pack("<I", index & 0xFFFFFFFF)


def check_mac(material, data, salt, expected):
    m = hmac.new(material[:32], salt + data + struct.pack("<Q", len(salt)), hashlib.sha256).digest()
    if not hmac.compare_digest(m, expected):
        raise AeaError("AEA range authentication failed")


def ctr(material, data):
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
    d = Cipher(algorithms.AES(material[32:64]), modes.CTR(material[64:80])).decryptor()
    return d.update(data) + d.finalize()


# --- metadata and key (reference: metadata, keyURL, fetchKey, unwrap) ---------

def metadata(prefix):
    if len(prefix) < 12 or prefix[:4] != b"AEA1" or struct.unpack_from("<I", prefix, 4)[0] != 1:
        raise AeaError("unsupported AEA header/profile")
    length = struct.unpack_from("<I", prefix, 8)[0]
    if not 0 < length <= MAX_METADATA or len(prefix) < 12 + length:
        raise AeaError("invalid AEA metadata length")
    data = prefix[12:12 + length]
    out = {}
    while data:
        if len(data) < 4:
            raise AeaError("truncated AEA metadata entry")
        size = struct.unpack_from("<I", data)[0]
        if size < 6 or size > len(data):
            raise AeaError("invalid AEA metadata entry size")
        key, sep, value = data[4:size].partition(b"\0")
        if not sep or not key or key.decode("latin-1") in out:
            raise AeaError("invalid AEA metadata key")
        out[key.decode("latin-1")] = value
        data = data[size:]
    return out


def key_url(raw):
    u = urllib.parse.urlsplit(raw)
    if u.scheme != "https" or u.hostname not in KEY_HOSTS or u.username or u.password \
            or u.port or u.query or u.fragment or not KEY_PATH.match(u.path):
        raise AeaError("AEA public key URL is outside Apple's FCS service")
    return raw


class _NoForeignRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if urllib.parse.urlsplit(newurl).path != urllib.parse.urlsplit(req.full_url).path:
            raise AeaError("invalid FCS key redirect")
        key_url(newurl)
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def fetch_key(raw, opener=None):
    url = key_url(raw)
    op = opener or urllib.request.build_opener(_NoForeignRedirect()).open
    try:
        with op(urllib.request.Request(url, headers={"User-Agent": UA}), timeout=30) as r:
            if r.status != 200:
                raise AeaError(f"Apple FCS key service returned HTTP {r.status}")
            data = r.read(16385)
    except OSError as e:
        raise AeaError(f"Apple FCS key service unreachable: {type(e).__name__}") from None
    if len(data) > 16384:
        raise AeaError("invalid Apple FCS response size")
    return data


def _labeled_extract(suite, salt, label, ikm):
    return hkdf_extract(salt, b"HPKE-v1" + suite + label + ikm)


def _labeled_expand(suite, prk, label, info, n):
    return hkdf_expand(prk, struct.pack(">H", n) + b"HPKE-v1" + suite + label + info, n)


def unwrap(md, release_key_pem):
    """The archive key: HPKE base-mode open of the FCS response's wrapped key
    with the release key from Apple's FCS service (reference: unwrap)."""
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM
    m = re.fullmatch(rb"\s*-----BEGIN PRIVATE KEY-----\s*([A-Za-z0-9+/=\s]+?)-----END PRIVATE KEY-----\s*",
                     release_key_pem)
    if not m:
        raise AeaError("invalid FCS release key")
    sk = serialization.load_der_private_key(base64.b64decode(b"".join(m.group(1).split())), None)
    if not isinstance(sk, ec.EllipticCurvePrivateKey) or sk.curve.name != "secp256r1":
        raise AeaError("FCS key must be P-256")
    try:
        resp = json.loads(md["com.apple.wkms.fcs-response"])
        enc = base64.b64decode(resp["enc-request"], validate=True)
        wrapped = base64.b64decode(resp["wrapped-key"], validate=True)
    except (KeyError, ValueError, TypeError):
        raise AeaError("invalid FCS response") from None
    if len(enc) != 65 or len(wrapped) != 48:
        raise AeaError("invalid FCS encapsulation")
    pke = ec.EllipticCurvePublicKey.from_encoded_point(ec.SECP256R1(), enc)
    dh = sk.exchange(ec.ECDH(), pke)
    pkr = sk.public_key().public_bytes(serialization.Encoding.X962,
                                       serialization.PublicFormat.UncompressedPoint)
    kem = b"KEM" + struct.pack(">H", 0x0010)
    eae = _labeled_extract(kem, b"", b"eae_prk", dh)
    shared = _labeled_expand(kem, eae, b"shared_secret", enc + pkr, 32)
    suite = b"HPKE" + struct.pack(">HHH", 0x0010, 0x0001, 0x0002)
    ctx = b"\0" + _labeled_extract(suite, b"", b"psk_id_hash", b"") \
        + _labeled_extract(suite, b"", b"info_hash", b"")
    secret = _labeled_extract(suite, shared, b"secret", b"")
    key = _labeled_expand(suite, secret, b"key", ctx, 32)
    nonce = _labeled_expand(suite, secret, b"base_nonce", ctx, 12)
    try:
        out = AESGCM(key).decrypt(nonce, wrapped, None)
    except Exception:
        raise AeaError("FCS key authentication failed") from None
    if len(out) != 32:
        raise AeaError("FCS key authentication failed")
    return out


# --- LZFSE (reference: boundedLZFSE) ------------------------------------------

def bounded_lzfse(data, maximum):
    total = expansion = 0
    off = 0
    while True:
        if off + 4 > len(data):
            raise AeaError("truncated LZFSE stream")
        magic = data[off:off + 4]
        if magic == b"bvx$":
            if off + 4 != len(data) or total > maximum:
                raise AeaError("invalid LZFSE termination")
            out = lzfse_decode(data, total)
            if len(out) != total:
                raise AeaError("LZFSE output length mismatch")
            return out
        if off + 8 > len(data):
            raise AeaError("truncated LZFSE header")
        raw = struct.unpack_from("<I", data, off + 4)[0]
        total += raw
        if raw > maximum or total > maximum:
            raise AeaError("LZFSE output budget exceeded")
        matches = 0
        if magic == b"bvx-":
            header, payload = 8, raw
        elif magic == b"bvxn":
            if off + 12 > len(data):
                raise AeaError("truncated LZVN header")
            header, payload = 12, struct.unpack_from("<I", data, off + 8)[0]
        elif magic == b"bvx1":
            if off + 772 > len(data):
                raise AeaError("truncated LZFSE V1 header")
            if struct.unpack_from("<I", data, off + 12)[0] > SEGMENT + 4:
                raise AeaError("LZFSE literal budget exceeded")
            matches = struct.unpack_from("<I", data, off + 16)[0]
            header = 772
            payload = struct.unpack_from("<I", data, off + 20)[0] + struct.unpack_from("<I", data, off + 24)[0]
        elif magic == b"bvx2":
            if off + 32 > len(data):
                raise AeaError("truncated LZFSE V2 header")
            v0, v1 = struct.unpack_from("<QQ", data, off + 8)
            matches = (v0 >> 40) & 0xFFFFF
            header = struct.unpack_from("<I", data, off + 24)[0]
            payload = ((v0 >> 20) & 0xFFFFF) + ((v1 >> 40) & 0xFFFFF)
            if header < 32:
                raise AeaError("invalid LZFSE V2 header length")
        else:
            raise AeaError("unsupported LZFSE block")
        expansion += raw + matches * (315 + 2359)
        if expansion > 512 << 20:
            raise AeaError("LZFSE match expansion budget exceeded")
        if header + payload > len(data) - off:
            raise AeaError("LZFSE block exceeds input")
        off += header + payload


def have_lzfse():
    """Whether an LZFSE decoder is available (liblzfse, or libcompression
    on macOS development hosts)."""
    import ctypes
    import ctypes.util
    if sys.platform == "darwin":
        return True
    name = os.environ.get("AURORA_LZFSE_LIB") or ctypes.util.find_library("lzfse") or "liblzfse.so.1"
    try:
        ctypes.CDLL(name)
        return True
    except OSError:
        return False


def lzfse_decode(data, size):
    import ctypes
    import ctypes.util
    out = ctypes.create_string_buffer(size + 1)
    if sys.platform == "darwin":
        lib = ctypes.CDLL("/usr/lib/libcompression.dylib")
        lib.compression_decode_buffer.restype = ctypes.c_size_t
        lib.compression_decode_buffer.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_char_p,
                                                  ctypes.c_size_t, ctypes.c_void_p, ctypes.c_int]
        n = lib.compression_decode_buffer(out, size + 1, data, len(data), None, 0x801)
    else:
        # AURORA_LZFSE_LIB: distributions without a library search path (NixOS).
        name = os.environ.get("AURORA_LZFSE_LIB") or ctypes.util.find_library("lzfse") or "liblzfse.so.1"
        try:
            lib = ctypes.CDLL(name)
        except OSError:
            raise AeaError("Apple's restore image is LZFSE-compressed and liblzfse is not installed "
                           "(install the lzfse package)") from None
        lib.lzfse_decode_buffer.restype = ctypes.c_size_t
        lib.lzfse_decode_buffer.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_char_p,
                                            ctypes.c_size_t, ctypes.c_void_p]
        lib.lzfse_decode_scratch_size.restype = ctypes.c_size_t
        scratch = ctypes.create_string_buffer(lib.lzfse_decode_scratch_size())
        n = lib.lzfse_decode_buffer(out, size + 1, data, len(data), scratch)
    return out.raw[:n]


# --- the archive ---------------------------------------------------------------

class Archive:
    """The decoded image of one AEA member, read by segment on demand.
    `src` has .get(off, n) and .size (a Slice of the IPSW)."""

    def __init__(self, src, key_fetch=fetch_key, cache=64, archive_key=None):
        self.src = src
        head = src.get(0, 12)
        if head[:4] != b"AEA1" or struct.unpack_from("<I", head, 4)[0] != 1:
            raise AeaError("unsupported AEA range profile")
        auth_len = struct.unpack_from("<I", head, 8)[0]
        if auth_len > MAX_METADATA:
            raise AeaError("invalid AEA prefix length")
        self.prefix_size = 12 + auth_len + 32 + 112
        prefix = src.get(0, self.prefix_size)
        md = metadata(prefix)
        if archive_key is not None:  # tests: the synthetic vectors' key
            key = archive_key
        elif "com.apple.wkms.fcs-key-url" not in md:
            raise AeaError("Missing FCS key URL.")
        else:
            key = unwrap(md, key_fetch(md["com.apple.wkms.fcs-key-url"].decode("ascii", "replace")))
        auth, salt = prefix[12:12 + auth_len], prefix[12 + auth_len:44 + auth_len]
        self.main = derive(key, salt, b"AEA_AMK" + prefix[4:8], 32)
        root = prefix[44 + auth_len:]
        material = derive(self.main, b"", b"AEA_RHEK", 80)
        check_mac(material, root[32:80], root[80:] + auth, root[:32])
        plain = ctr(material, root[32:80])
        self.size, encoded = struct.unpack_from("<QQ", plain, 0)
        seg, width = struct.unpack_from("<II", plain, 16)
        if encoded != src.size or seg != SEGMENT or width != CLUSTER_WIDTH \
                or plain[24:25] != b"e" or plain[25] != 2:
            raise AeaError("AEA image parameters are not the restore image profile")
        self.nsegments = (self.size + SEGMENT - 1) // SEGMENT
        self.clusters = []           # [(offset, [(seg_off, size, plain, sha, mac)])]
        self.next_mac = root[80:112]
        self.next_off = self.prefix_size
        self.cache, self.cache_max = {}, cache

    def _cluster(self, c):
        while len(self.clusters) <= c:
            k = len(self.clusters)
            off = self.next_off
            data = self.src.get(off, CLUSTER_HEADER)
            ck = derive(self.main, b"", indexed(b"AEA_CK", k), 32)
            material = derive(ck, b"", b"AEA_CHEK", 80)
            split = CLUSTER_WIDTH * 40
            check_mac(material, data[:split], data[split:], self.next_mac)
            headers = ctr(material, data[:split])
            pos = off + CLUSTER_HEADER
            segs = []
            for i in range(CLUSTER_WIDTH):
                sid = k * CLUSTER_WIDTH + i
                psize, size = struct.unpack_from("<II", headers, i * 40)
                want = min(SEGMENT, max(0, self.size - sid * SEGMENT))
                if psize != want or size > psize or (psize > 0 and size == 0) or pos > self.src.size - size:
                    raise AeaError("invalid AEA segment bounds")
                start = split + 32 + i * 32
                segs.append((pos, size, psize, headers[i * 40 + 8:i * 40 + 40], data[start:start + 32], ck))
                pos += size
            self.clusters.append((off, segs))
            self.next_mac = data[split:split + 32]
            self.next_off = pos
        return self.clusters[c][1]

    def segment(self, sid):
        if sid in self.cache:
            return self.cache[sid]
        if not 0 <= sid < self.nsegments:
            raise AeaError("segment outside the image")
        pos, size, psize, sha, mac, ck = self._cluster(sid // CLUSTER_WIDTH)[sid % CLUSTER_WIDTH]
        material = derive(ck, b"", indexed(b"AEA_SK", sid % CLUSTER_WIDTH), 80)
        data = self.src.get(pos, size)
        check_mac(material, data, b"", mac)
        plain = ctr(material, data)
        if size != psize:
            plain = bounded_lzfse(plain, psize)
        if len(plain) != psize or hashlib.sha256(plain).digest() != sha:
            raise AeaError("AEA plaintext segment SHA-256 mismatch")
        if len(self.cache) >= self.cache_max:
            self.cache.pop(next(iter(self.cache)))
        self.cache[sid] = plain
        return plain

    def pread(self, n, off):
        """Decoded image bytes [off, off+n)."""
        if off < 0 or off + n > self.size:
            raise AeaError("read past the decoded image")
        out = bytearray()
        while n:
            sid, o = divmod(off, SEGMENT)
            b = self.segment(sid)[o:o + n]
            out += b
            off += len(b)
            n -= len(b)
        return bytes(out)


def open_member(ipsw_url, member, key_fetch=fetch_key):
    """(Archive, Ranges) for one stored .aea member of a remote IPSW."""
    try:
        from . import remotezip
    except ImportError:
        import remotezip
    z = remotezip.RemoteZip(ipsw_url)
    m = z.members.get(member)
    if m is None or m.method != 0:
        raise AeaError(f"{member}: no stored member of that name")
    rng = Ranges(ipsw_url)
    return Archive(Slice(rng, m.data_off, m.usize), key_fetch), rng, z


if __name__ == "__main__":
    a, rng, _ = open_member(sys.argv[1], sys.argv[2])
    print("decoded", a.size, "segments", a.nsegments)
    b = a.pread(4096, int(sys.argv[3]) if len(sys.argv) > 3 else 0)
    print(b[:64].hex())
    print("fetched", rng.fetched)
