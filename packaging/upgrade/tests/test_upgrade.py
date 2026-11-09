# SPDX-License-Identifier: MIT
import base64, hashlib, json, os, plistlib, struct, sys, unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from aurora_speaker_upgrade import aea_ro, collect  # noqa: E402


def fixture(name):
    with open(os.path.join(HERE, "fixtures", name)) as f:
        return json.load(f)


def b64(s):
    return base64.b64decode(s)


class Bytes:
    def __init__(self, data):
        self.data, self.size = data, len(data)

    def get(self, off, n):
        assert 0 <= off and off + n <= self.size
        return self.data[off:off + n]


class GoVectors(unittest.TestCase):
    """The primitives against the pinned Go reference's vectors (the same the
    installer's Swift port is checked against)."""

    def setUp(self):
        self.v = fixture("go-vectors.json")

    def test_unwrap(self):
        v = self.v
        md = {"com.apple.wkms.fcs-response": json.dumps({"enc-request": v["enc"], "wrapped-key": v["wrapped"]}).encode()}
        self.assertEqual(aea_ro.unwrap(md, v["pem"].encode()), b64(v["key"]))
        w = bytearray(b64(v["wrapped"]))
        w[0] ^= 1
        md = {"com.apple.wkms.fcs-response": json.dumps({"enc-request": v["enc"],
                                                          "wrapped-key": base64.b64encode(bytes(w)).decode()}).encode()}
        with self.assertRaises(aea_ro.AeaError):
            aea_ro.unwrap(md, v["pem"].encode())

    def test_ctr_hkdf_mac(self):
        v = self.v
        m, c, p = b64(v["material"]), b64(v["ciphertext"]), b64(v["plain"])
        self.assertEqual(aea_ro.ctr(m, c), p)
        self.assertEqual(aea_ro.derive(b64(v["key"]), b"salt", b"AEA_RHEK", 80), b64(v["hkdf"]))
        mac = bytes.fromhex(v["mac"])
        aea_ro.check_mac(m, c, b"metadata", mac)
        with self.assertRaises(aea_ro.AeaError):
            aea_ro.check_mac(m, p, b"metadata", mac)
        with self.assertRaises(aea_ro.AeaError):
            aea_ro.check_mac(m, c, b"", mac)

    def test_key_url(self):
        ok = "https://wkms-public.apple.com/fcs-keys/" + "A" * 43 + "="
        self.assertEqual(aea_ro.key_url(ok), ok)
        for bad in (ok.replace("https", "http"), ok.replace("wkms-public.apple.com", "example.com"),
                    ok + "?x=1", "https://wkms-public.apple.com/other/" + "A" * 43 + "="):
            with self.assertRaises(aea_ro.AeaError):
                aea_ro.key_url(bad)


class GoArchive(unittest.TestCase):
    def setUp(self):
        f = fixture("go-aea.json")
        recs = sorted(f["records"], key=lambda r: r["record"]["offset"])
        self.data = b"".join(b64(r["data"]) for r in recs)
        self.f = f

    @unittest.skipUnless(aea_ro.have_lzfse(), "no liblzfse (its segment is LZFSE)")
    def test_decode(self):
        a = aea_ro.Archive(Bytes(self.data), archive_key=b64(self.f["key"]))
        self.assertEqual(a.size, len(b64(self.f["plain"])))
        self.assertEqual(a.pread(a.size, 0), b64(self.f["plain"]))
        self.assertEqual(a.pread(10, 100), b64(self.f["plain"])[100:110])

    def test_every_byte_is_authenticated(self):
        # Authentication fails before any decompression, so this needs no
        # LZFSE decoder.
        for off in (len(self.data) - 1, 200, 5000, len(self.data) - 50):
            d = bytearray(self.data)
            d[off] ^= 1
            with self.assertRaises(aea_ro.AeaError):
                a = aea_ro.Archive(Bytes(bytes(d)), archive_key=b64(self.f["key"]))
                a.pread(a.size, 0)


class SpeakerId(unittest.TestCase):
    def test_selects(self):
        per_vendor = plistlib.dumps({"AID9": {"Built-in Speaker": [{"Selector": "spid", "Value": 3}]}})
        neutral = plistlib.dumps({"AID13": {"Built-in Speaker": [{"Selector": "hwid", "Value": 1}]}})
        self.assertTrue(collect.selects_by_speaker_id(per_vendor))
        self.assertFalse(collect.selects_by_speaker_id(neutral))

    def test_chosen(self):
        p = os.path.join(HERE, "spid.bin")
        try:
            with open(p, "wb") as f:
                f.write(struct.pack(">I", 3))
            self.assertEqual(collect.chosen_speaker_id(p), 3)
            with open(p, "wb") as f:
                f.write(b"\0\0")
            self.assertIsNone(collect.chosen_speaker_id(p))
        finally:
            os.unlink(p)
        self.assertIsNone(collect.chosen_speaker_id(p))

    def test_invalid_strap_is_no_speaker_id(self):
        # aura#121 publishes 65535 for an invalid strap read (macOS semantics):
        # exactly like a missing ID, never a vendor index.
        p = os.path.join(HERE, "spid-invalid.bin")
        try:
            with open(p, "wb") as f:
                f.write(struct.pack(">I", 0xFFFF))
            self.assertIsNone(collect.chosen_speaker_id(p))
        finally:
            os.unlink(p)
        self.assertFalse(collect.valid_speaker_id(65535))
        self.assertFalse(collect.valid_speaker_id(None))
        self.assertTrue(collect.valid_speaker_id(0))
        self.assertTrue(collect.valid_speaker_id(3))

    def test_invalid_strap_falls_back_then_refuses(self):
        import tempfile
        from aurora_speaker_upgrade import upgrade
        from aurora_speaker_upgrade.asahi_firmware import audio_tuning as at

        class I:
            unit = at.unit_id(b"S\0")
        with tempfile.TemporaryDirectory() as t:
            chosen = os.path.join(t, "spid")
            with open(chosen, "wb") as f:
                f.write(struct.pack(">I", 0xFFFF))
            b1, b2 = os.path.join(t, "b1"), os.path.join(t, "b2")
            for d, spid in ((b1, 0xFFFF), (b2, 3)):
                os.makedirs(d)
                with open(os.path.join(d, "manifest.json"), "w") as f:
                    json.dump({"unit_id": I.unit, "speaker_id": spid}, f)
            # chosen 65535 -> this unit's bundle (a bundle's 65535 is skipped too)
            self.assertEqual(upgrade.speaker_id_source(I(), chosen, [b1, b2])[:2], (3, True))
            # nothing valid anywhere -> unknown: the AID that selects by spid refuses
            self.assertEqual(upgrade.speaker_id_source(I(), chosen, [b1])[:2], (None, False))
        per_vendor = plistlib.dumps({"AID9": {"Built-in Speaker": [{"Selector": "spid", "Value": 3}]}})
        with tempfile.TemporaryDirectory() as t:
            tun = os.path.join(t, collect.TUNINGS)
            os.makedirs(tun)
            with open(os.path.join(tun, "DeviceDSPDatabase.plist"), "wb") as f:
                f.write(per_vendor)

            class J:
                acoustic = 9
            with self.assertRaises(collect.SpeakerIdUnknown):
                collect.collect(t, J(), None, False)

    def test_fake_ioreg(self):
        class I:
            acoustic_raw, spts, target_type, serial = (9).to_bytes(4, "little"), b"x", b"J316sAP\0", b"S\0"
        from aurora_speaker_upgrade.asahi_firmware import audio_tuning as at
        run = collect.FakeRun(I(), "25C56", 0)
        self.assertEqual(at.probe(run), (I.acoustic_raw, b"x"))
        self.assertEqual(at.probe_speaker_id(run), 0)
        self.assertEqual(at.probe_identity(run), ("j316s", at.unit_id(b"S\0")))
        self.assertEqual(at.os_build(run), "25C56")
        self.assertIsNone(at.probe_speaker_id(collect.FakeRun(I(), "25C56", None)))


class DualKernelRun(unittest.TestCase):
    """cmd_run on a dual kernel (linux 98cb6a86+): legacy stops, the tuning
    compiles, the interface goes to the lease right before the self-check
    (which makes this kernel's hardware record with --switch-interface, then
    verifies), and only then the setting is written and v2 restarted."""

    def test_order_and_setting(self):
        import tempfile, types
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        calls = []
        with tempfile.TemporaryDirectory() as d:
            store = os.path.join(d, "store")
            conf = os.path.join(d, "speakerguardd.conf")
            with open(conf, "w") as f:
                f.write("SGV2_TUNING=--bundle /b\n")

            class Ident:
                unit, board, acoustic = "u" * 64, "j316s", 9

                def __init__(self, adt):
                    pass

                def matches_dt(self):
                    return True

            def run(argv, **kw):
                calls.append(("run", tuple(argv)))
                if argv[-1] == "--help":
                    return types.SimpleNamespace(returncode=0, stdout="run verify --switch-interface", stderr="")
                return types.SimpleNamespace(returncode=0, stdout="", stderr="")
            a = types.SimpleNamespace(store=store, adt=None, no_switch=False, force=False, scratch=os.path.join(d, "s"))
            with mock.patch.object(up.collect, "Identity", Ident), \
                    mock.patch.object(up, "interface_mode", return_value=("dual", "1")), \
                    mock.patch.object(up, "acquire", return_value=({}, {"route": 1, "aid": "AID9"})), \
                    mock.patch.object(up, "digest", return_value="dg"), \
                    mock.patch.object(up.Store, "put_bundle", lambda self, b: None), \
                    mock.patch.object(up.Store, "read_bundle", lambda self: None), \
                    mock.patch.object(up, "PROVISION_CONF", os.path.join(d, "provision.conf")), \
                    mock.patch.object(up, "V2_CONF", conf), \
                    mock.patch.object(up, "tool", lambda name: "/x/" + name), \
                    mock.patch.object(up, "run", run), \
                    mock.patch.object(up, "legacy_stop", lambda: calls.append(("legacy_stop",))), \
                    mock.patch.object(up, "interface_set", lambda card, v: calls.append(("iface", v))), \
                    mock.patch.object(up, "systemctl", lambda *x: calls.append(("systemctl",) + x) or
                                      types.SimpleNamespace(returncode=0, stdout="", stderr="")), \
                    mock.patch.object(up, "v2_unit", return_value="speakerguardd-v2.service"), \
                    mock.patch.object(up, "wait_v2_granted", return_value=True):
                self.assertEqual(up.cmd_run(a), up.EXIT_OK)
                with open(os.path.join(store, "interface")) as f:
                    self.assertEqual(f.read(), "v2\n")
        names = [c[0] if c[0] != "run" else os.path.basename(c[1][0]) + " " + c[1][1] for c in calls]
        i = names.index
        self.assertLess(i("legacy_stop"), i("speakerguardd-provision run"))
        self.assertLess(i("speakerguardd-provision run"), i("iface"))
        self.assertLess(i("iface"), i("speakerguardd-selfcheck run"))
        self.assertLess(i("speakerguardd-selfcheck run"), i("speakerguardd-selfcheck verify"))
        sc = [c[1] for c in calls if c[0] == "run" and c[1][1:2] == ("run",) and "selfcheck" in c[1][0]][0]
        self.assertIn("--switch-interface", sc)
        self.assertEqual(sc[-2:], ("--bundle", "/b"))
        self.assertIn(("iface", "Speaker Protection"), calls)


if __name__ == "__main__":
    unittest.main()
