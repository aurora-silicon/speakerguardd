# SPDX-License-Identifier: MIT
"""The pinned restore image is the only source of per-model tuning; only
unit.py reads the Mac. Synthetic data only, no network, no hardware."""
import contextlib
import http.client
import io
import json
import os
import pathlib
import plistlib
import struct
import sys
import tempfile
import types
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from aurora_speaker_upgrade import collect, ipsw, unit, upgrade as up  # noqa: E402
from aurora_speaker_upgrade.asahi_firmware import audio_tuning as at, lsm2_fill  # noqa: E402

PIN = ("27.2", [("26B123", "https://example.invalid/a.ipsw"), ("26B8125", "https://example.invalid/b.ipsw")])


def manifest(build, boards, version="27.2", ext=".dmg.aea"):
    return plistlib.dumps({"ProductBuildVersion": build, "ProductVersion": version, "BuildIdentities": [
        {"Info": {"DeviceClass": f"{b}ap"}, "Manifest": {"OS": {"Info": {"Path": f"{build}-{b}{ext}"}}}}
        for b in boards]})


def zips(table):
    """open_zip stand-in: url -> a zip with that BuildManifest."""
    def open_zip(url):
        data = table[url]
        member = types.SimpleNamespace(usize=len(data), read=lambda n: data)
        return types.SimpleNamespace(members={"BuildManifest.plist": member})
    return open_zip


class Pin(unittest.TestCase):
    def test_the_shipped_pin(self):
        release, images = ipsw.pin(paths=[str(pathlib.Path(__file__).resolve().parents[1]
                                              / "share" / "restore-images.json")])
        self.assertEqual(release, "27.2")
        self.assertEqual([b for b, _ in images], ["26B123", "26B8125"])
        self.assertTrue(all(u.startswith("https://updates.cdn-apple.com/") for _, u in images))

    def test_malformed_or_missing_pin_refuses(self):
        with tempfile.TemporaryDirectory() as t:
            p = os.path.join(t, "p.json")
            with self.assertRaises(ipsw.IpswError):
                ipsw.pin(paths=[p])
            for doc in ({"macos": "27.2", "restore_images": []},
                        {"macos": "27.2", "restore_images": [{"build": "26B123", "url": "http://x"}]},
                        {"restore_images": [{"build": "26B123", "url": "https://x"}]}):
                with open(p, "w") as f:
                    json.dump(doc, f)
                with self.assertRaises(ipsw.IpswError):
                    ipsw.pin(paths=[p])

    def test_first_pinned_image_carrying_the_board(self):
        oz = zips({"https://example.invalid/a.ipsw": manifest("26B123", ["j473", "j413"]),
                   "https://example.invalid/b.ipsw": manifest("26B8125", ["k114c"])})
        self.assertEqual(ipsw.find("j473", PIN, open_zip=oz)[:2], ("26B123", "https://example.invalid/a.ipsw"))
        self.assertEqual(ipsw.find("k114c", PIN, open_zip=oz)[0], "26B8125")
        with self.assertRaises(ipsw.IpswError) as e:
            ipsw.find("j999", PIN, open_zip=oz)
        self.assertIn("carries board j999", str(e.exception))

    def test_an_image_that_is_not_its_pinned_build_refuses(self):
        oz = zips({"https://example.invalid/a.ipsw": manifest("26B131", ["j473"]),
                   "https://example.invalid/b.ipsw": manifest("26B8125", ["j473"])})
        with self.assertRaises(ipsw.IpswError):
            ipsw.find("j473", PIN, open_zip=oz)

    def test_a_mirror_must_serve_a_pinned_build(self):
        oz = zips({"https://mirror.invalid/x.ipsw": manifest("25G83", ["j473"]),
                   "https://mirror.invalid/y.ipsw": manifest("26B123", ["j473"])})
        with self.assertRaises(ipsw.IpswError):
            ipsw.find("j473", PIN, url="https://mirror.invalid/x.ipsw", open_zip=oz)
        self.assertEqual(ipsw.find("j473", PIN, url="https://mirror.invalid/y.ipsw", open_zip=oz)[0], "26B123")

    def test_an_unreachable_image_is_unavailable_and_no_other_source_is_tried(self):
        tried = []

        def oz(url):
            tried.append(url)
            raise OSError("offline")
        with self.assertRaises(ipsw.IpswUnavailable):
            ipsw.find("j473", PIN, open_zip=oz)
        self.assertEqual(tried, ["https://example.invalid/a.ipsw"])

    def test_only_an_apple_encrypted_archive_is_read(self):
        oz = zips({"https://m.invalid/x.ipsw": manifest("26B123", ["j473"], ext=".dmg")})
        with mock.patch.object(ipsw.aea_ro, "open_member") as om, \
                mock.patch.object(ipsw.reader, "_open") as opened:
            with self.assertRaisesRegex(ipsw.IpswError, "not an Apple Encrypted Archive"):
                ipsw.open_system("j473", PIN, url="https://m.invalid/x.ipsw", open_zip=oz)
        om.assert_not_called()
        opened.assert_not_called()

    def test_a_manifest_that_changes_after_the_check_opens_nothing_else(self):
        """A mirror serving an AEA manifest first and a plain one later: only
        the checked .aea member is ever opened."""
        served = [manifest("26B123", ["j473"]), manifest("26B123", ["j473"]),
                  manifest("26B123", ["j473"], ext=".dmg")]
        fetches = []

        def oz(url):
            data = served[min(len(fetches), len(served) - 1)]
            fetches.append(url)
            return types.SimpleNamespace(members={"BuildManifest.plist": types.SimpleNamespace(
                usize=len(data), read=lambda n: data)})
        opened = []
        with mock.patch.object(ipsw.reader, "_open",
                               side_effect=lambda *a: opened.append(a[3]) or ("vol", None, a[1], a[2])):
            ipsw.open_system("j473", PIN, url="https://m.invalid/x.ipsw", open_zip=oz)
            fetches.clear()
            ipsw.open_system("j473", PIN, open_zip=oz)
        self.assertTrue(opened)
        self.assertTrue(all(m.endswith(".aea") for m in opened), opened)

    def test_the_reader_is_the_installers(self):
        from aurora_speaker_upgrade import asahi_firmware
        self.assertIs(ipsw.reader, asahi_firmware.ipsw)
        self.assertIs(ipsw.IpswUnavailable, asahi_firmware.ipsw.IpswUnavailable)
        here = pathlib.Path(__file__).resolve().parents[1] / "aurora_speaker_upgrade"
        for name in ("aea_ro.py", "apfs_ro.py", "remotezip.py"):
            self.assertFalse((here / name).exists(), f"a second copy of {name}")


class OfflineAtBoot(unittest.TestCase):
    """No network at boot is not an answer: no refusal record, so a later
    boot (with Wi-Fi up) tries again. The image's own answers still refuse
    and are recorded."""

    def run_boot(self):
        with tempfile.TemporaryDirectory() as d:
            ident = types.SimpleNamespace(unit="u" * 64, board="j473", acoustic=12, matches_dt=lambda: True)
            a = types.SimpleNamespace(store=d, adt=None, at_boot=True, chosen=None, installer_dirs=[],
                                      scratch=os.path.join(d, "scratch"), ipsw=None, no_switch=False)
            with mock.patch.object(up.collect, "Identity", return_value=ident), \
                    mock.patch.object(up, "interface_mode", return_value=("v2-only", "card")), \
                    mock.patch.object(up, "playback_proof", return_value=True), \
                    mock.patch.object(up, "speaker_id_source", return_value=(None, False, "none")), \
                    mock.patch.object(up.Store, "ensure", lambda self: os.makedirs(self.root, exist_ok=True)), \
                    mock.patch.object(up, "say"):
                try:
                    up.cmd_run(a)
                    outcome = "ok"
                except up.Retry:
                    outcome = "retry"
                except up.Refused:
                    outcome = "refused"
            return outcome, os.path.exists(os.path.join(d, "refused.json"))

    def test_unreachable_is_retry_without_a_record(self):
        with mock.patch.object(ipsw, "open_system", side_effect=ipsw.IpswUnavailable("no route to host")):
            self.assertEqual(self.run_boot(), ("retry", False))

    def test_an_answer_of_the_image_is_recorded(self):
        with mock.patch.object(ipsw, "open_system", side_effect=ipsw.IpswError("no pinned image carries board j473")):
            self.assertEqual(self.run_boot(), ("refused", True))

    def test_incomplete_zip_range_retries_without_a_record(self):
        def open_url(request, **kwargs):
            if request.get_method() == "HEAD":
                response = io.BytesIO()
                response.headers = {"Content-Length": "400", "Accept-Ranges": "bytes"}
                return response
            if reported_by_http:
                raise http.client.IncompleteRead(bytes(399), 1)
            return io.BytesIO(bytes(399))

        for reported_by_http in (False, True):
            with self.subTest(reported_by_http=reported_by_http), \
                    mock.patch.object(ipsw, "pin", return_value=PIN), \
                    mock.patch.object(ipsw.remotezip.urllib.request, "urlopen", side_effect=open_url) as fetch:
                self.assertEqual(self.run_boot(), ("retry", False))
                self.assertEqual(fetch.call_count, 5)  # One HEAD and four range attempts.

    def test_a_drop_while_staging_is_retry(self):
        vol = object()
        with tempfile.TemporaryDirectory() as d:
            a = types.SimpleNamespace(ipsw=None)
            ident = types.SimpleNamespace(board="j473", acoustic=12)
            rng = types.SimpleNamespace(fetched=0)
            with mock.patch.object(up.ipsw, "open_system", return_value=(vol, rng, "26B123", "https://x/y.ipsw")), \
                    mock.patch.object(up.collect, "stage", side_effect=up.aea_ro.AeaUnavailable("range fetch failed")), \
                    mock.patch.object(up, "say"):
                with self.assertRaises(up.Retry):
                    up.from_ipsw(ident, a, None, False, d)


class NoMachineRoutes(unittest.TestCase):
    def test_the_installed_macos_readers_are_gone(self):
        for name in ("system_volumes", "apfs_partitions", "preboot_build", "installer_bundle", "build_key"):
            self.assertFalse(hasattr(collect, name), name)
        for name in ("route0", "route1", "route2", "route3", "build_hint", "GUIDE"):
            self.assertFalse(hasattr(up, name), name)

    def test_the_old_route_options_are_refused(self):
        for argv in (["--route", "1"], ["--collection", "x.tar.gz"], ["--macos-build", "26B131"],
                     ["--device", "/dev/nvme0n1p2"], ["guide"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                up.main(argv)

    def test_an_ipsw_failure_refuses(self):
        with tempfile.TemporaryDirectory() as d:
            a = types.SimpleNamespace(chosen=None, installer_dirs=[], scratch=d, ipsw=None)
            ident = types.SimpleNamespace(unit="u" * 64, board="j473", acoustic=12)
            with mock.patch.object(up, "speaker_id_source", return_value=(None, False, "none")), \
                    mock.patch.object(up.ipsw, "open_system", side_effect=ipsw.IpswError("offline")), \
                    mock.patch.object(up, "say"):
                with self.assertRaises(up.Refused) as e:
                    up.acquire(ident, a)
        self.assertIn("offline", str(e.exception))


def short_bundle(unit_id="u" * 64, build="26B123"):
    ids = sorted(lsm2_fill.base_ids())[58:]
    state = struct.pack(">III", 0, 0, len(ids)) + b"".join(struct.pack(">If", i, n / 17) for n, i in enumerate(ids))
    strip = {"strips": [{"effects": [{"unit": {"subtype": lsm2_fill.SUBTYPE}, "aupreset": {"data": state}}]}]}
    b = {"AID901/DSP/Strips/generated.austrip": plistlib.dumps(strip)}
    b[at.MANIFEST] = at.encode_manifest({"aid": "AID901", "macos_build": build, "unit_id": unit_id})
    return b


def fill_for(bundle, component, build="26B123", unit_id="u" * 64):
    files = {k: v for k, v in bundle.items() if k != at.MANIFEST}
    missing = sorted(lsm2_fill.base_ids())[:58]

    def readback(path, effect, files):
        return {"unit_version": 1, "component_sha256": component,
                "published": [{"id": i, "name": "", "bits": 0x3f800000} for i in sorted(lsm2_fill.base_ids())]}
    return lsm2_fill.collect(files, "AID901", build, unit_id, readback=readback)


class ReadbackBinding(unittest.TestCase):
    C = "c" * 64

    def test_a_readback_of_the_same_files_and_component_is_attached(self):
        b = short_bundle()
        out = collect.attach_fill(b, fill_for(b, self.C), self.C)
        m = json.loads(out[at.MANIFEST])
        self.assertEqual(m["lsm2_param_fill"]["path"], lsm2_fill.NAME)
        self.assertIn(lsm2_fill.NAME, out)
        self.assertEqual(out["AID901/DSP/Strips/generated.austrip"], b["AID901/DSP/Strips/generated.austrip"])

    def test_another_component_refuses(self):
        b = short_bundle()
        with self.assertRaises(collect.CollectError):
            collect.attach_fill(b, fill_for(b, "d" * 64), self.C)

    def test_other_files_refuse(self):
        b = short_bundle()
        fill = fill_for(b, self.C)
        changed = dict(b)
        changed["AID901/DSP/Strips/other.austrip"] = b"x"
        with self.assertRaises(collect.CollectError):
            collect.attach_fill(changed, fill, self.C)

    def test_another_build_or_unit_refuses(self):
        b = short_bundle()
        with self.assertRaises(collect.CollectError):
            collect.attach_fill(b, fill_for(short_bundle(build="26B131"), self.C, build="26B131"), self.C)
        with self.assertRaises(collect.CollectError):
            collect.attach_fill(b, fill_for(b, self.C, unit_id="v" * 64), self.C)

    def test_acquire_takes_this_units_installer_readback(self):
        b = short_bundle()
        fill = fill_for(b, self.C)
        with tempfile.TemporaryDirectory() as d:
            inst = os.path.join(d, "inst")
            os.makedirs(inst)
            with open(os.path.join(inst, at.MANIFEST), "w") as f:
                json.dump({"unit_id": "u" * 64, "aid": "AID901", "lsm2_param_fill": {"path": lsm2_fill.NAME}}, f)
            with open(os.path.join(inst, lsm2_fill.NAME), "wb") as f:
                f.write(fill)
            a = types.SimpleNamespace(chosen=None, installer_dirs=[inst], scratch=os.path.join(d, "s"), ipsw=None)
            ident = types.SimpleNamespace(unit="u" * 64, board="j713", acoustic=901)
            with mock.patch.object(up, "speaker_id_source", return_value=(None, False, "none")), \
                    mock.patch.object(up, "from_ipsw", return_value=(b, {"source": "ipsw"}, self.C)), \
                    mock.patch.object(up, "say"):
                out, prov = up.acquire(ident, a)
                self.assertEqual(prov["lsm2_fill_from"], inst)
                self.assertIn(lsm2_fill.NAME, out)
                # another unit's installer bundle is never read
                ident.unit = "w" * 64
                with self.assertRaises(up.Refused):
                    up.acquire(ident, a)


class UnitAllowList(unittest.TestCase):
    def test_speaker_id_sources(self):
        ident = types.SimpleNamespace(unit="u" * 64)
        with tempfile.TemporaryDirectory() as t:
            chosen = os.path.join(t, "spid")
            with open(chosen, "wb") as f:
                f.write(struct.pack(">I", 2))
            self.assertEqual(unit.speaker_id(ident, chosen, [])[:2], (2, True))
            b = os.path.join(t, "b")
            os.makedirs(b)
            with open(os.path.join(b, at.MANIFEST), "w") as f:
                json.dump({"unit_id": "u" * 64, "speaker_id": 5}, f)
            self.assertEqual(unit.speaker_id(ident, os.path.join(t, "none"), [b])[:2], (5, True))
            ident.unit = "x" * 64
            self.assertEqual(unit.speaker_id(ident, os.path.join(t, "none"), [b])[:2], (None, False))

    def test_only_unit_py_opens_machine_paths(self):
        here = pathlib.Path(__file__).resolve().parents[1] / "aurora_speaker_upgrade"
        for p in here.glob("*.py"):
            # unit.py is the allow-list; adt_ro reads the ADT for it.
            if p.name in ("unit.py", "adt_ro.py"):
                continue
            text = p.read_text()
            for needle in ("/proc/device-tree", "/lib/firmware/vendor", "lsblk", "preboot_build",
                           "/dev/nvme", "system_volumes("):
                self.assertFalse(needle in text, f"{p.name} reads {needle}")


if __name__ == "__main__":
    unittest.main()
