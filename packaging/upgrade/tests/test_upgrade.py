# SPDX-License-Identifier: MIT
import base64, contextlib, hashlib, json, os, plistlib, struct, sys, unittest

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


class UnknownSpeakerIdGoesToRoute3(unittest.TestCase):
    """An AID that selects its tuning by speaker ID, with no speaker ID from
    the boot chain: routes 1 and 2 read the same tuning and cannot select
    it, but the macOS collection (route 3) records the ID and must still be
    tried."""

    def test_route3_after_unknown_speaker_id(self):
        import tempfile, types
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        from aurora_speaker_upgrade.asahi_firmware import audio_tuning as at
        tried = []
        unit = "u" * 64
        bundle = {at.MANIFEST: json.dumps({"unit_id": unit, "aid": "AID40", "macos_build": "26A428",
                                           "speaker_id": 3}).encode()}

        def r0(identity, a):
            tried.append("0")
            raise collect.CollectError("no installer bundle on this Mac")

        def r1(identity, a, spid, known, scratch):
            tried.append("1")
            raise collect.SpeakerIdUnknown("AID40 selects its speaker tuning by speaker ID")

        def r2(identity, a, spid, known, scratch):
            tried.append("2")
            return bundle, {"route": "2", "source": "ipsw"}

        def r3(identity, a):
            tried.append("3")
            return bundle, {"route": "3", "source": "collection"}

        with tempfile.TemporaryDirectory() as d:
            a = types.SimpleNamespace(route="auto", chosen=None, installer_dirs=[], scratch=d, collection="c.tar.gz")
            ident = types.SimpleNamespace(unit=unit)
            with mock.patch.object(up, "speaker_id_source", return_value=(None, False, None)), \
                    mock.patch.object(up, "route0", r0), mock.patch.object(up, "route1", r1), \
                    mock.patch.object(up, "route2", r2), mock.patch.object(up, "route3", r3):
                b, prov = up.acquire(ident, a)
        self.assertEqual(tried, ["0", "1", "3"])
        self.assertEqual(prov["route"], "3")
        self.assertEqual(prov["speaker_id"], 3)


class DualKernelRun(unittest.TestCase):
    """cmd_run on a dual kernel (linux 98cb6a86+): legacy stops, the tuning
    compiles, the interface goes to the lease right before the self-check
    (which makes this kernel's hardware record with --switch-interface, then
    verifies), and only then the setting is written and v2 restarted."""

    def test_order_and_setting(self):
        calls, rc, iface = self.dual_run()
        self.assertEqual(rc, "ok")
        self.assertEqual(iface, "v2\n")
        names = self.names(calls)
        i = names.index
        # the session stops before the lease and starts on the new graph before verify
        self.assertLess(i("speakerguardd-provision run"), i("session_stop"))
        self.assertLess(i("session_stop"), i("iface"))
        self.assertLess(i("speakerguardd-provision activate"), i("session_start"))
        self.assertLess(i("session_start"), i("speakerguardd-selfcheck verify"))
        self.assertEqual(names.count("session_stop"), 1)
        self.assertEqual(names.count("session_start"), 1)

    def test_rollback_starts_the_session_on_the_legacy_graph(self):
        calls, rc, iface = self.dual_run(verify_rc=1)
        self.assertEqual(rc, "refused")
        self.assertIsNone(iface)
        names = self.names(calls)
        i = names.index
        last = len(names) - 1 - names[::-1].index("session_start")
        self.assertLess(i("session_stop"), i("iface"))
        legacy = calls.index(("iface", "Speaker Volume Unlock"))
        self.assertLess(legacy, last)
        self.assertLess(names.index("speakerguardd-provision deactivate"), last)
        self.assertLess(last, i("legacy_start"))
        self.assertNotIn(("systemctl", "restart", "speakerguardd-v2.service"), calls)

    def test_session_that_will_not_stop_refuses_before_the_switch(self):
        calls, rc, iface = self.dual_run(session_stop_ok=False)
        self.assertEqual(rc, "refused")
        names = self.names(calls)
        self.assertNotIn(("iface", "Speaker Protection"), calls)
        self.assertNotIn("speakerguardd-selfcheck run", names)
        self.assertIn("legacy_start", names)
        self.assertIn("session_start", names)

    def test_session_stop_timeout_is_a_refusal(self):
        import subprocess
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up

        def run(argv, **kw):
            if argv[-1] == "--help":
                import types
                return types.SimpleNamespace(returncode=0, stdout="session-stop session-start", stderr="")
            raise subprocess.TimeoutExpired(argv, kw.get("timeout"))
        with mock.patch.object(up, "tool", lambda n: "/x/" + n), mock.patch.object(up, "run", run), \
                mock.patch.object(up, "v2_unit", return_value="speakerguardd-v2.service"), \
                mock.patch.object(up, "say", lambda m: None):
            with self.assertRaises(up.Refused):
                up.session_stop()
            up.session_start()  # never raises
        self.assertGreater(up.SESSION_STOP_LIMIT_S, 15 + 10 + 15)

    def names(self, calls):
        return [c[0] if c[0] != "run" else os.path.basename(c[1][0]) + " " + c[1][1] for c in calls]

    def dual_run(self, verify_rc=0, session_stop_ok=True):
        import tempfile, types
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        calls = []
        rc = iface = None
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
                if argv[-1] == "--help":
                    return types.SimpleNamespace(returncode=0, stdout="run verify --switch-interface deactivate", stderr="")
                calls.append(("run", tuple(argv)))
                code = verify_rc if argv[1:2] == ["verify"] else 0
                return types.SimpleNamespace(returncode=code, stdout="", stderr="")

            def stop():
                calls.append(("session_stop",))
                if not session_stop_ok:
                    raise up.Refused("the seat user's audio session did not stop")
            a = types.SimpleNamespace(store=store, adt=None, no_switch=False, force=False, scratch=os.path.join(d, "s"))
            with contextlib.ExitStack() as es:
                es.enter_context(mock.patch.object(up.collect, "Identity", Ident))
                es.enter_context(mock.patch.object(up, "interface_mode", return_value=("dual", "1")))
                es.enter_context(mock.patch.object(up, "acquire", return_value=({}, {"route": 1, "aid": "AID9"})))
                es.enter_context(mock.patch.object(up, "digest", return_value="dg"))
                es.enter_context(mock.patch.object(up.Store, "put_bundle", lambda self, b: None))
                es.enter_context(mock.patch.object(up.Store, "read_bundle", lambda self: None))
                es.enter_context(mock.patch.object(up, "PROVISION_CONF", os.path.join(d, "provision.conf")))
                es.enter_context(mock.patch.object(up, "V2_CONF", conf))
                es.enter_context(mock.patch.object(up, "tool", lambda name: "/x/" + name))
                es.enter_context(mock.patch.object(up, "run", run))
                es.enter_context(mock.patch.object(up, "legacy_stop", lambda: calls.append(("legacy_stop",))))
                es.enter_context(mock.patch.object(up, "legacy_start", lambda: calls.append(("legacy_start",))))
                es.enter_context(mock.patch.object(up, "pcm_settle", lambda card: True))
                es.enter_context(mock.patch.object(up, "session_stop", stop))
                es.enter_context(mock.patch.object(up, "session_start", lambda: calls.append(("session_start",))))
                es.enter_context(mock.patch.object(up, "interface_set", lambda card, v: calls.append(("iface", v))))
                es.enter_context(mock.patch.object(up, "systemctl", lambda *x: calls.append(("systemctl",) + x) or types.SimpleNamespace(returncode=0, stdout="", stderr="")))
                es.enter_context(mock.patch.object(up, "v2_unit", return_value="speakerguardd-v2.service"))
                es.enter_context(mock.patch.object(up, "say", lambda m: None))
                es.enter_context(mock.patch.object(up, "wait_v2_granted", return_value=True))
                try:
                    rc = "ok" if up.cmd_run(a) == up.EXIT_OK else "other"
                except up.Refused:
                    rc = "refused"
                iface = up.read_text(os.path.join(store, "interface"))
        return calls, rc, iface

    def test_order_before_the_session_hooks(self):
        calls, rc, _ = self.dual_run()
        names = self.names(calls)
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


class RevertSettle(unittest.TestCase):
    """A same-boot revert on a dual kernel restarts the seat user's
    PipeWire, waits until the speaker PCMs keep their state, and only then
    starts the older daemon, so the two never configure the I2S at once."""

    def test_order(self):
        import types
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        order = []
        store = types.SimpleNamespace(interface="/nonexistent/sgb6-interface", save=lambda st: None)
        with mock.patch.object(up, "systemctl", lambda *a: order.append(("systemctl",) + a) or types.SimpleNamespace(returncode=0)), \
                mock.patch.object(up, "interface_set", lambda c, v: order.append(("interface", v))), \
                mock.patch.object(up, "graph_deactivate", lambda: order.append(("graph_deactivate",))), \
                mock.patch.object(up, "session_stop", lambda: order.append(("session_stop",))), \
                mock.patch.object(up, "session_start", lambda: order.append(("session_start",))), \
                mock.patch.object(up, "pcm_settle", lambda card: order.append(("settle", card)) or True), \
                mock.patch.object(up, "legacy_start", lambda: order.append(("legacy_start",))), \
                mock.patch.object(up, "v2_unit", return_value="speakerguardd-v2.service"), \
                mock.patch.object(up, "PROVISION_CONF", "/nonexistent/sgb6-provision.conf"):
            up.back_to_legacy(store, {}, "dual", "AppleJ316")
        names = [o[0] for o in order]
        self.assertEqual(names[0], "session_stop")
        self.assertEqual(order[1], ("systemctl", "stop", "speakerguardd-v2.service"))
        self.assertEqual(names[-5:], ["interface", "graph_deactivate", "session_start", "settle", "legacy_start"])
        self.assertIn(("settle", "AppleJ316"), order)

    def test_settle_waits_for_a_steady_state(self):
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        now = [0.0]
        seq = iter(["closed"] * 3 + ["SETUP", "PREPARED"] + ["RUNNING"] * 100)
        with mock.patch.object(up, "pcm_states", lambda card: {"p1": next(seq)}), \
                mock.patch.object(up.time, "monotonic", lambda: now[0]), \
                mock.patch.object(up.time, "sleep", lambda t: now.__setitem__(0, now[0] + t)):
            self.assertTrue(up.pcm_settle("AppleJ316", wait=10, quiet=1.5))
            self.assertGreaterEqual(now[0], 0.4 + 1.5 - 1e-9)
        flip = iter(["closed", "RUNNING"] * 200)
        now[0] = 0.0
        with mock.patch.object(up, "pcm_states", lambda card: {"p1": next(flip)}), \
                mock.patch.object(up.time, "monotonic", lambda: now[0]), \
                mock.patch.object(up.time, "sleep", lambda t: now.__setitem__(0, now[0] + t)):
            self.assertFalse(up.pcm_settle("AppleJ316", wait=2, quiet=1.5))

    def test_pcm_states_reads_proc(self):
        import tempfile
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(f"{d}/pcm1p/sub0")
            open(f"{d}/pcm1p/sub0/status", "w").write("state: RUNNING\nowner_pid   : 42\n")
            os.makedirs(f"{d}/pcm0p/sub0")
            open(f"{d}/pcm0p/sub0/status", "w").write("closed\n")
            os.makedirs(f"{d}/pcm2c/sub0")
            open(f"{d}/pcm2c/sub0/status", "w").write("state: SETUP\n")
            real = up.glob.glob
            with mock.patch.object(up.glob, "glob", lambda p: real(p.replace("/proc/asound/AppleJ316", d))):
                self.assertEqual(sorted(up.pcm_states("AppleJ316").values()), ["RUNNING", "SETUP", "closed"])


class CheckKeepsTheInterface(unittest.TestCase):
    """--check is a dry run: on a Mac whose speakers v2 already owns it never
    lends them to the switch test (that left the kernel on the legacy
    interface, lease 0 and the speakers at the floor until a reboot), and
    otherwise the switch test hands back the interface it found."""

    def amixer(self, value):
        import types
        out = ("numid=9,iface=MIXER,name='Speaker Protection Interface'\n"
               "  ; type=ENUMERATED,access=rw------,values=1,items=2\n"
               "  ; Item #0 'Speaker Volume Unlock'\n  ; Item #1 'Speaker Protection'\n"
               f"  : values={value}\n")
        return types.SimpleNamespace(returncode=0, stdout=out, stderr="")

    def test_interface_get(self):
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        for v, want in (("1", up.IF_V2), ("0", up.IF_LEGACY)):
            with mock.patch.object(up, "run", lambda argv, **kw: self.amixer(v)):
                self.assertEqual(up.interface_get("AppleJ616"), want)

    def test_switch_test_hands_back_what_it_found(self):
        import types
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        sets = []
        for found in (up.IF_V2, up.IF_LEGACY):
            sets.clear()
            with mock.patch.object(up, "interface_get", return_value=found), \
                    mock.patch.object(up, "interface_set", lambda c, v: sets.append(v)), \
                    mock.patch.object(up, "legacy_units", return_value=[]), \
                    mock.patch.object(up, "v2_tuning_args", return_value=[]), \
                    mock.patch.object(up, "run", lambda argv, **kw: types.SimpleNamespace(returncode=2, stdout="", stderr="pcm1p stayed in use for 15 s")):
                res, _ = up.check_hardware("/x/selfcheck", "AppleJ616", "/p", "/s", "/c")
            self.assertEqual(res, "cannot-run")
            self.assertEqual(sets, [found])

    def test_check_does_not_take_the_speakers_from_v2(self):
        import types, tempfile
        from unittest import mock
        from aurora_speaker_upgrade import upgrade as up
        ident = types.SimpleNamespace(board="j616s", acoustic=35, matches_dt=lambda: True)
        called = []
        with tempfile.TemporaryDirectory() as d:
            a = types.SimpleNamespace(adt=None, json=None, scratch=d)
            def run(argv, **kw):
                if argv[1:2] == ["--help"]:
                    return types.SimpleNamespace(returncode=0, stdout="run --switch-interface verify --conf", stderr="")
                if argv[1:2] == ["run"]:
                    os.makedirs(os.path.join(argv[-1], up.PARAMS.lstrip("/")).rsplit("/", 1)[0], exist_ok=True)
                    open(os.path.join(argv[-1], up.PARAMS.lstrip("/")), "wb").write(b"blob")
                return types.SimpleNamespace(returncode=0, stdout="", stderr="")
            with mock.patch.object(up.collect, "Identity", return_value=ident), \
                    mock.patch.object(up, "interface_mode", return_value=("dual", "AppleJ616")), \
                    mock.patch.object(up, "acquire", return_value=({}, {"route": 0, "source": "test"})), \
                    mock.patch.object(up, "digest", return_value="d"), \
                    mock.patch.object(up.collect, "write_bundle", lambda b, p: os.makedirs(p, exist_ok=True)), \
                    mock.patch.object(up, "tool", lambda n: "/x/" + n), \
                    mock.patch.object(up, "run", run), \
                    mock.patch.object(up, "interface_get", return_value=up.IF_V2), \
                    mock.patch.object(up, "systemctl", lambda *x: types.SimpleNamespace(returncode=0, stdout="", stderr="")), \
                    mock.patch.object(up, "check_hardware", lambda *x: called.append(x) or ("pass", "")), \
                    mock.patch.object(up, "say", lambda m: None):
                rc = up.cmd_check(a)
        self.assertEqual(rc, up.EXIT_RETRY)
        self.assertEqual(called, [])


class RunRechecksTheLiveState(unittest.TestCase):
    """'run' on a Mac switched on record says "nothing to do" only while v2
    really holds the speakers in this boot (or at boot, where v2's own unit
    starts it); otherwise it switches again instead of leaving the speakers
    at the floor."""

    def setUp(self):
        import types
        from aurora_speaker_upgrade import upgrade as up
        self.up, self.types = up, types

    def run_with(self, at_boot, owns):
        import json, contextlib, io
        from unittest import mock
        up, types = self.up, self.types
        manifest = {"unit_id": "U1", "aid": "AID9", "macos_build": "25C56"}
        store = types.SimpleNamespace(state=lambda: {"switched": True, "route": 0}, interface="/x/interface",
                                      read_bundle=lambda: {up.audio_tuning.MANIFEST: json.dumps(manifest)})
        acquired = []
        with mock.patch.object(up, "Store", return_value=store), \
                mock.patch.object(up.collect, "Identity", return_value=types.SimpleNamespace(unit="U1", matches_dt=lambda: True)), \
                mock.patch.object(up, "interface_mode", return_value=("dual", "AppleJ316")), \
                mock.patch.object(up, "read_text", return_value="v2\n"), \
                mock.patch.object(up, "v2_owns_speakers", return_value=owns), \
                mock.patch.object(up, "acquire", side_effect=lambda *x: acquired.append(1) or (_ for _ in ()).throw(up.Retry("stop here"))):
            a = types.SimpleNamespace(store="/x", adt=None, at_boot=at_boot)
            with contextlib.redirect_stderr(io.StringIO()):
                try:
                    rc = up._cmd_run(a)
                except up.Retry:
                    rc = "switching"
        return rc, acquired

    def test_nothing_to_do_while_v2_holds_them(self):
        self.assertEqual(self.run_with(False, True), (self.up.EXIT_OK, []))

    def test_switches_again_when_v2_does_not_hold_them(self):
        self.assertEqual(self.run_with(False, False), ("switching", [1]))

    def test_at_boot_trusts_the_record(self):
        self.assertEqual(self.run_with(True, False), (self.up.EXIT_OK, []))


if __name__ == "__main__":
    unittest.main()


class NoConfirmation(unittest.TestCase):
    """A run switches without a prompt or a flag, from a terminal or not (the
    boot unit); --i-understand-preview is still accepted and changes nothing."""

    def setUp(self):
        import io
        from aurora_speaker_upgrade import upgrade as up
        self.up, self.io = up, io
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

    def test_non_interactive_run_switches(self):
        for argv in (["run"], ["run", "--at-boot"], ["run", "--i-understand-preview"],
                     ["run", "--at-boot", "--i-understand-preview"]):
            self.ran.clear()
            rc, err = self.main(*argv)
            self.assertEqual((rc, self.ran), (self.up.EXIT_OK, ["run"]), argv)
            self.assertNotIn("PREVIEW", err.upper(), argv)

    def test_other_commands_unchanged(self):
        for argv, name in ((["--check"], "check"), (["status"], "status"), (["--revert"], "revert"),
                           (["run", "--no-switch"], "run")):
            self.ran.clear()
            rc, _ = self.main(*argv)
            self.assertEqual((rc, self.ran), (self.up.EXIT_OK, [name]), argv)
