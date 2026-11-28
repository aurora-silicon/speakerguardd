# SPDX-License-Identifier: MIT
"""apfs_ro: decmpfs type 9 (uncompressed in the attribute) is a 0xcc marker
byte followed by exactly the file. Synthetic data only."""
import pathlib
import struct
import sys
import types
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from aurora_speaker_upgrade.asahi_firmware import aea_ro, apfs_ro  # noqa: E402


class Type9(unittest.TestCase):
    def decode(self, body, usize):
        hdr = struct.pack("<IIQ", apfs_ro.DECMPFS_MAGIC, 9, usize) + body
        fake = types.SimpleNamespace(_xattr_data=lambda flags, data: data)
        return apfs_ro.Volume._decmpfs(fake, 7, {"com.apple.decmpfs": (2, hdr)})

    def test_a_marker_then_the_file(self):
        self.assertEqual(self.decode(b"\xcc<plist/>", 8), b"<plist/>")

    def test_no_marker_or_another_size_is_an_error(self):
        for body, usize in ((b"<plist/>", 8), (b"\xcc<plist/>", 9), (b"\xcc<plist/>x", 8)):
            with self.assertRaises(apfs_ro.ApfsError):
                self.decode(body, usize)


class AeaRangeTransfer(unittest.TestCase):
    """A range body cut short (http.client.IncompleteRead, not an OSError)
    is retried and then reported as unavailability, never as an answer."""

    def test_incomplete_read_is_unavailable(self):
        import http.client
        import io as _io
        calls = []

        class Resp(_io.BytesIO):
            status = 206
            headers = {}

        def opener(req, timeout=None):
            calls.append(req.get_method())
            if req.get_method() == "HEAD":
                r = Resp(b"")
                r.headers = {"Content-Length": "4096"}
                return r
            raise http.client.IncompleteRead(b"x", 10)
        r = aea_ro.Ranges("https://updates.cdn-apple.com/x.ipsw", opener=opener)
        with self.assertRaises(aea_ro.AeaUnavailable):
            r.get(0, 11)
        self.assertEqual(calls, ["HEAD", "GET", "GET", "GET"])


if __name__ == "__main__":
    unittest.main()
