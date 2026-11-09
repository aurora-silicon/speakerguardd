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
        # the graph is generated from the new record before verify (whose graph step needs it)
        self.assertLess(i("speakerguardd-selfcheck run"), i("speakerguardd-provision activate"))
        self.assertLess(i("speakerguardd-provision activate"), i("speakerguardd-selfcheck verify"))
        sc = [c[1] for c in calls if c[0] == "run" and c[1][1:2] == ("run",) and "selfcheck" in c[1][0]][0]
        self.assertIn("--switch-interface", sc)
        self.assertEqual(sc[-2:], ("--bundle", "/b"))
        self.assertIn(("iface", "Speaker Protection"), calls)


class BootRefusalRecord(unittest.TestCase):
    """The boot unit does not redo a refusal at every boot: same unit, kernel
    and speakerguardd -> skipped (exit 0); a by-hand run, or any change of
    the key, tries again; a success clears the record."""

    def setUp(self):
        import tempfile, types
        from aurora_speaker_upgrade import upgrade as up
        self.up, self.dir = up, tempfile.mkdtemp()
        self.calls, self.result = 0, "refuse"
        self.saved = (up._cmd_run, collect.Identity, up.refusal_key, up.Store.ensure)
        test = self

        def fake_run(a):
            test.calls += 1
            if test.result == "refuse":
                raise up.Refused("the speaker self-check refused this Mac (exit 1)")
            return up.EXIT_OK
        up._cmd_run = fake_run
        collect.Identity = lambda adt=None: types.SimpleNamespace(unit="u1")
        self.key = "k1"
        up.refusal_key = lambda identity: test.key
        up.Store.ensure = lambda self: os.makedirs(self.root, mode=0o700, exist_ok=True)
        self.a = lambda boot: types.SimpleNamespace(store=self.dir, adt=None, at_boot=boot)

    def tearDown(self):
        import shutil
        up = self.up
        up._cmd_run, collect.Identity, up.refusal_key, up.Store.ensure = self.saved
        shutil.rmtree(self.dir)

    def test_refusal_is_recorded_and_skipped_at_boot(self):
        up = self.up
        with self.assertRaises(up.Refused):
            up.cmd_run(self.a(True))
        rec = json.load(open(os.path.join(self.dir, "refused.json")))
        self.assertEqual(rec["key"], "k1")
        self.assertIn("self-check refused", rec["reason"])
        self.assertEqual(oct(os.stat(os.path.join(self.dir, "refused.json")).st_mode & 0o777), "0o600")
        self.assertEqual(up.cmd_run(self.a(True)), up.EXIT_OK)
        self.assertEqual(self.calls, 1)

    def test_by_hand_and_new_key_try_again(self):
        up = self.up
        with self.assertRaises(up.Refused):
            up.cmd_run(self.a(True))
        with self.assertRaises(up.Refused):
            up.cmd_run(self.a(False))
        self.assertEqual(self.calls, 2)
        self.key = "k2"
        self.result = "pass"
        self.assertEqual(up.cmd_run(self.a(True)), up.EXIT_OK)
        self.assertEqual(self.calls, 3)
        self.assertFalse(os.path.exists(os.path.join(self.dir, "refused.json")))

    def test_key_covers_kernel_and_tools(self):
        import types
        up = self.up
        k = self.saved[2](types.SimpleNamespace(unit="u1"))
        self.assertRegex(k, r"^[0-9a-f]{64}$")
        self.assertNotEqual(k, self.saved[2](types.SimpleNamespace(unit="u2")))


class V2Umask(unittest.TestCase):
    def test_v2_programs_run_with_the_units_umask(self):
        """B4: v2's provisioning and self-check run as their units would
        (umask 022), not with this tool's 077."""
        import subprocess, tempfile
        from aurora_speaker_upgrade import upgrade as up
        old = os.umask(0o077)
        try:
            with tempfile.TemporaryDirectory() as d:
                r = up.run_v2(["sh", "-c", "umask"])
                self.assertEqual(r.stdout.strip(), "0022")
                self.assertEqual(up.run(["sh", "-c", "umask"]).stdout.strip(), "0077")
        finally:
            os.umask(old)


class StorePermissions(unittest.TestCase):
    """The store holds this Mac's extracted Apple tuning: root only (0700
    directories, 0600 files) after a run, whatever umask v2's own programs
    get (B4 gave them 022)."""

    def test_store_is_root_only_after_a_run(self):
        import stat, tempfile
        from aurora_speaker_upgrade import upgrade as up
        old = os.umask(0o077)  # what main() sets
        try:
            with tempfile.TemporaryDirectory() as d:
                st = up.Store(os.path.join(d, "store"))
                st.ensure()
                st.put_bundle({"manifest.json": b"{}", "tuning/AID13/a.aupreset": b"x" * 16})
                up.write_file(st.interface, "v2\n", 0o600)
                st.save({"switched": True})
                up.run_v2(["true"])  # v2 subprocesses get 022, the store does not change
                for root, dirs, files in os.walk(st.root):
                    self.assertEqual(stat.S_IMODE(os.stat(root).st_mode), 0o700, root)
                    for f in files:
                        self.assertEqual(stat.S_IMODE(os.stat(os.path.join(root, f)).st_mode), 0o600, f)
        finally:
            os.umask(old)


class LegacyStart(unittest.TestCase):
    """RC8 E2E B2: a revert starts the older daemon the packaged udev rules
    want for the sense PCM, never every installed one (speakerguardd 0.1
    on an M2 exits 78/CONFIG)."""

    def test_only_the_wanted_unit(self):
        import types
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        props = {"/dev/snd/pcmC0D2c": "DEVNAME=/dev/snd/pcmC0D2c\nSYSTEMD_WANTS=speakersafetyd.service\n",
                 "/dev/snd/pcmC1D0c": "DEVNAME=/dev/snd/pcmC1D0c\n"}
        calls = []

        def run(argv, **kw):
            out = props.get(argv[-1], "") if argv[:2] == ["udevadm", "info"] else ""
            return types.SimpleNamespace(returncode=0, stdout=out, stderr="")
        with mock.patch.object(up, "run", run), mock.patch.object(up.glob, "glob", lambda p: sorted(props)), \
                mock.patch.object(up, "v2_unit", return_value="speakerguardd-v2.service"), \
                mock.patch.object(up, "systemctl", lambda *a: calls.append(a) or types.SimpleNamespace(returncode=0)):
            self.assertEqual(up.legacy_wanted(), ["speakersafetyd.service"])
            up.legacy_start()
        started = [a[-1] for a in calls if a[0] == "start"]
        self.assertEqual(started, ["speakersafetyd.service"])


if __name__ == "__main__":
    unittest.main()


class PreviewWarning(unittest.TestCase):
    """Ryan 11-09: every switch is preceded by the preview warning; a
    non-interactive run needs --i-understand-preview, else exit 2 with
    nothing done; --check, status and --revert never ask."""

    def setUp(self):
        import io, types
        from aurora_speaker_upgrade import upgrade as up
        self.up, self.types, self.io = up, types, io
        self.saved = (up.cmd_run, up.cmd_check, up.cmd_status, up.cmd_revert, up.locked, os.geteuid, sys.stdin)
        self.ran = []
        for name in ("run", "check", "status", "revert"):
            setattr(up, f"cmd_{name}", (lambda n: lambda a: self.ran.append(n) or up.EXIT_OK)(name))
        import contextlib
        up.locked = contextlib.nullcontext
        os.geteuid = lambda: 0
        sys.stdin = io.StringIO("")          # not a TTY

    def tearDown(self):
        up = self.up
        up.cmd_run, up.cmd_check, up.cmd_status, up.cmd_revert, up.locked, os.geteuid, sys.stdin = self.saved

    def main(self, *argv):
        import contextlib
        err = self.io.StringIO()
        with contextlib.redirect_stderr(err):
            rc = self.up.main(list(argv) + ["--scratch", "/nonexistent/sgb6-test-scratch"])
        return rc, err.getvalue()

    def test_non_interactive_run_refuses_without_the_flag(self):
        rc, err = self.main("run")
        self.assertEqual(rc, self.up.EXIT_RETRY)
        self.assertEqual(self.ran, [])
        self.assertIn("PREVIEW", err)
        self.assertIn("does not fully match macOS", err)
        self.assertIn("supported", err)
        self.assertIn("aurora-speaker-upgrade --revert", err)
        self.assertIn("--i-understand-preview", err)
        rc, _ = self.main("run", "--at-boot")
        self.assertEqual((rc, self.ran), (self.up.EXIT_RETRY, []))

    def test_flag_runs_and_still_warns(self):
        rc, err = self.main("run", "--i-understand-preview")
        self.assertEqual((rc, self.ran), (self.up.EXIT_OK, ["run"]))
        self.assertIn("PREVIEW", err)

    def test_interactive_asks(self):
        up = self.up
        a = self.types.SimpleNamespace(i_understand_preview=False)
        import contextlib
        with contextlib.redirect_stderr(self.io.StringIO()):
            self.assertTrue(up.preview_ack(a, ask=lambda q: "y", interactive=True))
            self.assertTrue(up.preview_ack(a, ask=lambda q: "YES ", interactive=True))
            for no in ("", "n", "no", "maybe"):
                with self.assertRaises(up.Retry):
                    up.preview_ack(a, ask=lambda q: no, interactive=True)

            def eof(q):
                raise EOFError
            with self.assertRaises(up.Retry):
                up.preview_ack(a, ask=eof, interactive=True)

    def test_check_status_revert_never_ask(self):
        for argv, name in ((["--check"], "check"), (["status"], "status"), (["--revert"], "revert"),
                           (["run", "--no-switch"], "run")):
            self.ran.clear()
            rc, err = self.main(*argv)
            self.assertEqual((rc, self.ran), (self.up.EXIT_OK, [name]), argv)
            self.assertNotIn("PREVIEW", err, argv)
