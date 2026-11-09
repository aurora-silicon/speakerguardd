#!/usr/bin/env python3
"""Tests of speakerguardd-provision and speakerguardd-gate, on a test root
with a stand-in tuning reader, device tree and speaker card.

    python3 -I packaging/session/test_provision.py
"""
import contextlib
import glob
import hashlib
import importlib.machinery
import importlib.util
import io
import json
import os
import pwd
import shutil
import sys
import tempfile
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name, file):
    loader = importlib.machinery.SourceFileLoader(name, os.path.join(HERE, file))
    spec = importlib.util.spec_from_loader(name, loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


prov = load("provision", "speakerguardd-provision")
gate = load("gate", "speakerguardd-gate")

# A stand-in for aurora-speaker-tuning: `identify` and `compile`, steered by
# a JSON file beside it.
FAKE_CLI = r'''#!/usr/bin/env python3
import hashlib, json, os, sys
ctl = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "ctl.json")))
cmd = sys.argv[1]
log = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calls")
with open(log, "a") as f:
    f.write(" ".join(sys.argv[1:]) + "\n")
if cmd == "identify":
    if ctl.get("no_bundle"):
        print(json.dumps({"error": "no speaker tuning bundle found"})); sys.exit(1)
    print(json.dumps({"source": "/lib/firmware/vendor/apple/audio-tuning", "verified": True, "aid": "AID9",
                      "macos_build": ctl.get("build", "25G83"), "digest": ctl.get("digest", "d1"), "reader": "1.0.0",
                      "kind": ctl.get("kind", "Installer"), "board": ctl.get("board", "j316s"),
                      "unit_bound": ctl.get("unit_bound", True),
                      "unit_id": ctl.get("unit_id", hashlib.sha256(b"aurora-speaker-unit-v1\0SYNTHETIC0A").hexdigest())}))
elif cmd == "compile":
    if ctl.get("compile_fails"):
        print("error[spp3-channel-count] NumChannels 6 but the graph feeds the unit 4 channels", file=sys.stderr)
        sys.exit(1)
    out = sys.argv[sys.argv.index("-o") + 1]
    open(out, "wb").write(("blob " + ctl.get("digest", "d1")).encode())
    print("wrote blob", file=sys.stderr)
    for w in ctl.get("warnings", []):
        print(w, file=sys.stderr)
elif cmd == "dump":
    t = {"protection": {"kind": ctl.get("unit", "spp3")}, "refusals": ctl.get("refusals", [])}
    if ctl.get("lsid"):
        t["lsid"] = {}
    print(json.dumps({"tuning": t}))
    sys.exit(ctl.get("dump_rc", 0))
'''

# Parity belongs to the build, never a per-board list.
GATE = {"units_at_parity": ["spp3"], "denied_boards": [], "board_overrides": {}}


def write_dt(root, board):
    """A synthetic device tree: the root compatible and one macaudio card
    with a speaker link (one back-end, one amp), whose machine key is the
    board without its s/c suffix and whose model ends in that key."""
    key = board.rstrip("sc")

    def node(path, props, phandle=None):
        d = os.path.join(root, path)
        os.makedirs(d, exist_ok=True)
        for k, v in props.items():
            with open(os.path.join(d, k), "wb") as f:
                f.write(v)
        if phandle:
            with open(os.path.join(d, "phandle"), "wb") as f:
                f.write(phandle.to_bytes(4, "big"))

    for d in ("sound", "soc"):
        shutil.rmtree(os.path.join(root, d), ignore_errors=True)
    node(".", {"compatible": f"apple,{board}\0apple,t6000\0".encode(), "serial-number": b"SYNTHETIC0A\0"})
    node("soc/be", {"compatible": b"apple,mca\0", "#sound-dai-cells": (1).to_bytes(4, "big")}, 0x10)
    node("soc/amp", {"compatible": b"ti,sn012776\0", "sound-name-prefix": b"Speaker\0"}, 0x11)
    node("sound", {"compatible": f"apple,{key}-macaudio\0apple,macaudio\0".encode(), "model": f"Test Mac {key.upper()}\0".encode()})
    node("sound/dai-link@0", {"link-name": b"Speakers\0"})
    node("sound/dai-link@0/cpu", {"sound-dai": (0x10).to_bytes(4, "big") + (0).to_bytes(4, "big")})
    node("sound/dai-link@0/codec", {"sound-dai": (0x11).to_bytes(4, "big")})

ME = pwd.getpwuid(os.getuid())


class Base(unittest.TestCase):
    def setUp(self):
        self.t = tempfile.mkdtemp(prefix="sgv2-provision-")
        self.root = os.path.join(self.t, "root")
        self.dt = os.path.join(self.t, "dt")
        os.makedirs(os.path.join(self.dt, "chosen"))
        self.board("j316s")
        bindir = os.path.join(self.t, "bin")
        os.makedirs(bindir)
        self.cli = os.path.join(bindir, "aurora-speaker-tuning")
        with open(self.cli, "w") as f:
            # Run by this test's own interpreter: a package build's sandbox
            # (Nix) has no /usr/bin/env.
            f.write(FAKE_CLI.replace("#!/usr/bin/env python3", "#!" + sys.executable, 1))
        os.chmod(self.cli, 0o755)
        self.ctl({})
        self.p = prov.Paths(self.root)
        self.saved = {}
        # The live machine's card, seat and users are stand-ins.
        self.patch(prov, "find_card", lambda want, wait: (0, want))
        self.patch(prov, "speaker_pcm", lambda idx: 1)
        self.patch(prov, "seat0_user", lambda rng: None)
        self.patch(prov, "autologin_user", lambda etc: None)
        self.patch(prov, "regular_users", lambda rng: [ME.pw_name])
        self.patch(prov, "regular", lambda pw, rng: True)

    def tearDown(self):
        for (mod, name), v in self.saved.items():
            setattr(mod, name, v)
        shutil.rmtree(self.t)

    def patch(self, mod, name, value):
        self.saved.setdefault((mod, name), getattr(mod, name))
        setattr(mod, name, value)

    def board(self, b):
        write_dt(self.dt, b)

    def ctl(self, d):
        with open(os.path.join(os.path.dirname(self.cli), "ctl.json"), "w") as f:
            json.dump(d, f)

    def calls(self):
        try:
            with open(os.path.join(os.path.dirname(self.cli), "calls")) as f:
                return [l.split()[0] for l in f]
        except OSError:
            return []

    def args(self, cmd="run", **kw):
        a = dict(cmd=cmd, bundle=None, force=False, unit="speakerguardd.service", graph_unit="speakerguardd-graph.service",
                 switch_test_wait=0.3, root=self.root,
                 tuning_cli=self.cli, dt=self.dt, etc=os.path.join(self.t, "etc"), card_wait=0)
        a.update(kw)
        return types.SimpleNamespace(**a)

    def run_(self, **kw):
        return prov.cmd_run(self.args(**kw), self.p)

    def record(self):
        with open(self.p.report) as f:
            return json.load(f)


class Provision(Base):
    def test_voicing_gate_switch_recompiles_and_records_the_mode(self):
        import gen
        self.patch(gen, "voicing_enabled", lambda board: False)
        self.assertEqual(self.run_(), 0)
        self.assertEqual(prov.read_kv(self.p.bind)["voicing_pre"], "false")
        self.patch(gen, "voicing_enabled", lambda board: board == "j316s")
        self.assertEqual(self.run_(), 0)
        self.assertTrue(self.record()["compiled"])
        self.assertEqual(prov.read_kv(self.p.bind)["voicing_pre"], "true")
        with open(os.path.join(os.path.dirname(self.cli), "calls")) as f:
            self.assertIn("--voicing-pre", f.read())
        self.assertEqual(self.run_(), 0)
        self.assertFalse(self.record()["compiled"])
        self.patch(gen, "voicing_enabled", lambda board: False)
        self.assertEqual(self.run_(), 0)
        self.assertTrue(self.record()["compiled"])
        self.assertEqual(prov.read_kv(self.p.bind)["voicing_pre"], "false")


    def test_compiles_binds_and_configures(self):
        self.assertEqual(self.run_(), 0)
        self.assertEqual(self.calls(), ["identify", "dump", "compile"])
        conf = prov.read_kv(self.p.conf)
        self.assertEqual(conf["SGV2_MACHINE"], "j316")
        self.assertEqual(conf["SGV2_USER"], ME.pw_name)
        self.assertEqual(conf["SGV2_UID"], str(ME.pw_uid))
        self.assertEqual(conf["SGV2_ARGS"], f"--card AppleJ316 --pcm 1 --uid {ME.pw_uid} --params {self.p.params}")
        self.assertEqual(conf["SGV2_PARAMS"], self.p.params)
        self.assertEqual(conf["SGV2_TUNING"], "")
        bind = prov.read_kv(self.p.bind)
        self.assertEqual(bind["result"], "pass")
        self.assertEqual(bind["board"], "j316s")
        self.assertEqual(bind["kernel"], os.uname().release)
        self.assertEqual(bind["bundle_digest"], "d1")
        self.assertEqual(bind["macos_build"], "25G83")
        self.assertEqual(bind["params_sha256"], prov.sha256_file(self.p.params))
        self.assertTrue(prov.is_generated(self.p.conf))

    def test_recompiles_only_on_a_change(self):
        self.assertEqual(self.run_(), 0)
        self.assertEqual(self.run_(), 0)
        self.assertEqual(self.calls().count("compile"), 1)
        # Another bundle, another macOS build, another kernel, a changed blob.
        for change in (lambda: self.ctl({"digest": "d2"}), lambda: self.ctl({"digest": "d2", "build": "25H1"}),
                       lambda: self.rebind(kernel="7.0.0-other"), lambda: self.rebind(speaker_id="00000001"),
                       lambda: self.rebind(compiler="0.9:feedface"), lambda: self.rebind(unit="0" * 64),
                       self.tamper):
            n = self.calls().count("compile")
            change()
            self.assertEqual(self.run_(), 0)
            self.assertEqual(self.calls().count("compile"), n + 1)
            self.assertEqual(prov.read_kv(self.p.bind)["params_sha256"], prov.sha256_file(self.p.params))
        n = self.calls().count("compile")
        self.assertEqual(self.run_(force=True), 0)
        self.assertEqual(self.calls().count("compile"), n + 1)

    def tamper(self):
        with open(self.p.params, "ab") as f:
            f.write(b"x")

    def rebind(self, **kw):
        b = prov.read_kv(self.p.bind)
        b.update(kw)
        prov.write_atomic(self.p.bind, "".join(f"{k}={v}\n" for k, v in b.items()))

    def test_binds_the_unit_not_the_serial(self):
        self.assertEqual(self.run_(), 0)
        want = hashlib.sha256(b"aurora-speaker-unit-v1\0SYNTHETIC0A").hexdigest()
        self.assertEqual(prov.read_kv(self.p.bind)["unit"], want)
        for path in (self.p.bind, self.p.report, self.p.conf):
            with open(path, "rb") as f:
                self.assertNotIn(b"SYNTHETIC0A", f.read())
        # Another Mac's extraction and a missing/malformed local identity
        # must never reuse a blob or compile in open-loop fallback mode.
        with open(os.path.join(self.dt, "serial-number"), "wb") as f:
            f.write(b"SYNTHETIC0B\0")
        self.assertEqual(self.run_(), 1)
        self.assert_closed("provenance")
        os.unlink(os.path.join(self.dt, "serial-number"))
        self.assertEqual(self.run_(), 1)
        self.assert_closed("provenance")
        with open(os.path.join(self.dt, "serial-number"), "wb") as f:
            f.write(b"NOT ONE\0")
        self.assertEqual(prov.unit_id(self.dt), "malformed")
        self.assertEqual(self.run_(), 1)

    def test_logs_the_compilers_warnings(self):
        warn = "warning[unit-unbound] compile: the tuning bundle does not name the Mac it was collected on"
        self.ctl({"warnings": ["note[x] compile: quiet", warn]})
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.assertEqual(self.run_(), 0)
        self.assertIn(warn, err.getvalue())
        self.assertNotIn("quiet", err.getvalue())

    def test_another_board_recompiles(self):
        self.assertEqual(self.run_(), 0)
        self.board("j413")
        self.ctl({"board": "j413"})
        self.assertEqual(self.run_(), 0)
        self.assertEqual(self.calls().count("compile"), 2)
        self.assertEqual(prov.read_kv(self.p.conf)["SGV2_MACHINE"], "j413")

    def assert_closed(self, why):
        self.assertFalse(os.path.exists(self.p.params))
        self.assertFalse(os.path.exists(self.p.conf))
        rec = self.record()
        self.assertEqual(rec["result"], "fail")
        self.assertTrue(any(why in r for r in rec["reasons"]), rec["reasons"])
        self.assertEqual(prov.read_kv(self.p.bind)["result"], "fail")

    def test_fails_closed_when_the_bundle_does_not_compile(self):
        self.assertEqual(self.run_(), 0)
        self.ctl({"digest": "d3", "compile_fails": True})
        self.assertEqual(self.run_(), 1)
        self.assert_closed("does not compile")
        self.assertTrue(any("spp3-channel-count" in r for r in self.record()["reasons"]))

    def test_fails_closed_without_a_bundle(self):
        self.assertEqual(self.run_(), 0)
        self.ctl({"no_bundle": True})
        self.assertEqual(self.run_(), 1)
        self.assert_closed("no usable speaker tuning bundle")

    def test_fails_closed_without_a_seat_user(self):
        self.patch(prov, "regular_users", lambda rng: ["alice", "bob"])
        self.assertEqual(self.run_(), 1)
        self.assert_closed("no seat user: 2 regular users (alice, bob)")

    def test_fails_closed_without_a_card(self):
        def nocard(want, wait):
            raise prov.Failed("no speaker card (apple,macaudio) after 0 s")
        self.patch(prov, "find_card", nocard)
        self.assertEqual(self.run_(), 1)
        self.assert_closed("no speaker card")

    def test_a_hand_written_conf_is_kept_aside(self):
        os.makedirs(self.p.etc)
        with open(self.p.conf, "w") as f:
            f.write('SGV2_ARGS="--pcm 1 --uid 1000 --params /x"\n')
        self.assertEqual(self.run_(), 0)
        self.assertTrue(prov.is_generated(self.p.conf))
        with open(self.p.conf + ".hand") as f:
            self.assertIn("--uid 1000", f.read())

    def test_explicit_bundle(self):
        b = os.path.join(self.t, "bundle")
        os.makedirs(b)
        self.assertEqual(self.run_(bundle=b), 0)
        self.assertEqual(prov.read_kv(self.p.conf)["SGV2_TUNING"], b)
        self.assertEqual(self.run_(bundle=os.path.join(self.t, "absent")), 1)
        self.assertEqual(self.run_(bundle=os.path.join(self.t, "a b")), 1)


class Seat(Base):
    def test_order(self):
        # Stand-in users, so the test means the same as root (a package
        # build) and as anyone else: root is never a seat user.
        users = {"alice": 1000, "bob": 1001, "root": 0}
        self.patch(prov, "by_name", lambda n: pwd.struct_passwd((n, "x", users[n], users[n], "", "/", "/bin/sh"))
                   if n in users else None)
        p, ov = self.p, {}
        os.makedirs(p.state)
        # The only regular user.
        self.patch(prov, "regular_users", lambda rng: ["alice"])
        self.assertEqual(prov.resolve_seat_user(p, ov, (1000, 60000), self.t), ("alice", "only regular user"))
        # Several: none, until something names one.
        self.patch(prov, "regular_users", lambda rng: ["alice", "bob"])
        with self.assertRaises(prov.Failed):
            prov.resolve_seat_user(p, ov, (1000, 60000), self.t)
        self.patch(prov, "autologin_user", lambda etc: "root")
        with self.assertRaises(prov.Failed):
            prov.resolve_seat_user(p, ov, (1000, 60000), self.t)
        self.patch(prov, "autologin_user", lambda etc: "bob")
        self.assertEqual(prov.resolve_seat_user(p, ov, (1000, 60000), self.t), ("bob", "automatic login"))
        prov.write_atomic(p.seat_last, "alice\n")
        self.assertEqual(prov.resolve_seat_user(p, ov, (1000, 60000), self.t), ("alice", "last seat0 user"))
        self.patch(prov, "seat0_user", lambda rng: "bob")
        self.assertEqual(prov.resolve_seat_user(p, ov, (1000, 60000), self.t), ("bob", "seat0"))
        self.assertEqual(prov.resolve_seat_user(p, {"SGV2_SEAT_USER": "alice"}, (1000, 60000), self.t), ("alice", "provision.conf"))
        for bad in ("no-such-user-sgv2", "root"):
            with self.assertRaises(prov.Failed):
                prov.resolve_seat_user(p, {"SGV2_SEAT_USER": bad}, (1000, 60000), self.t)

    def test_uid_is_resolved_by_name_never_fixed(self):
        # The name is what is kept; the uid comes from the user database.
        self.patch(prov, "seat0_user", lambda rng: ME.pw_name)
        self.assertEqual(self.run_(), 0)
        conf = prov.read_kv(self.p.conf)
        self.assertEqual(conf["SGV2_UID"], str(pwd.getpwnam(ME.pw_name).pw_uid))
        with open(self.p.seat_last) as f:
            self.assertEqual(f.read().strip(), ME.pw_name)

    def test_autologin_files(self):
        etc = os.path.join(self.t, "etc")
        os.makedirs(os.path.join(etc, "gdm"))
        prov.autologin_user = self.saved.get((prov, "autologin_user"), prov.autologin_user)
        with open(os.path.join(etc, "gdm", "custom.conf"), "w") as f:
            f.write("[daemon]\nAutomaticLoginEnable=True\nAutomaticLogin=ryan\n")
        self.assertEqual(prov.autologin_user(etc), "ryan")
        with open(os.path.join(etc, "gdm", "custom.conf"), "w") as f:
            f.write("[daemon]\nAutomaticLoginEnable=False\nAutomaticLogin=ryan\n")
        self.assertIsNone(prov.autologin_user(etc))
        os.makedirs(os.path.join(etc, "sddm.conf.d"))
        with open(os.path.join(etc, "sddm.conf.d", "kde_settings.conf"), "w") as f:
            f.write("[Autologin]\nSession=plasma\nUser=chloe\n")
        self.assertEqual(prov.autologin_user(etc), "chloe")

    def test_uid_range(self):
        f = os.path.join(self.t, "login.defs")
        with open(f, "w") as fh:
            fh.write("# x\nUID_MIN                  1000\nUID_MAX 59999\n")
        self.assertEqual(prov.uid_range(f), (1000, 59999))
        self.assertEqual(prov.uid_range(os.path.join(self.t, "absent")), (1000, 60000))


class Graph(Base):
    def test_checked_voicing_blob_is_published_before_graph_start(self):
        self.provisioned()
        digest = prov.sha256_file(self.p.params)
        prov.write_atomic(self.p.bind, f"voicing_pre=true\nparams_sha256={digest}\n")
        prov.write_atomic(os.path.join(self.p.state, "selfcheck.bind"), f"result=pass\nparams_sha256={digest}\n")
        prov.publish_voicing_parameters(self.p)
        with open(self.p.published, "rb") as f:
            self.assertEqual(f.read(), b"blob d1")
        with open(self.p.params, "ab") as f:
            f.write(b"changed")
        with self.assertRaises(prov.Failed):
            prov.publish_voicing_parameters(self.p)
        with open(self.p.published, "rb") as f:
            self.assertEqual(f.read(), b"blob d1")


    def test_rollback_publishes_checked_interim_parameters_before_restarting_graph(self):
        self.provisioned()
        digest = prov.sha256_file(self.p.params)
        prov.write_atomic(self.p.bind, f"voicing_pre=false\nparams_sha256={digest}\n")
        prov.write_atomic(os.path.join(self.p.state, "selfcheck.bind"), f"result=pass\nparams_sha256={digest}\n")
        prov.publish_voicing_parameters(self.p, previous_voicing=True)
        with open(self.p.published, "rb") as f:
            self.assertEqual(f.read(), b"blob d1")

    def test_voicing_only_parameter_update_changes_graph_digest(self):
        self.provisioned()
        prov.write_atomic(self.p.bind, "voicing_pre=true\nparams_sha256=aaa\n")
        first = prov.graph_digest(self.p)
        prov.write_atomic(self.p.bind, "voicing_pre=true\nparams_sha256=bbb\n")
        self.assertNotEqual(first, prov.graph_digest(self.p))


    def provisioned(self):
        self.assertEqual(self.run_(), 0)
        for d, n in (("pipewire", "sgv2-j316.conf"), ("wireplumber", "60-sgv2-j316.conf")):
            os.makedirs(os.path.join(self.p.graph, d), exist_ok=True)
            with open(os.path.join(self.p.graph, d, n), "w") as f:
                f.write("# graph\n")
        prov.write_atomic(self.p.graph_state, "result=pass\n")
        home = os.path.join(self.t, "home")
        os.environ["XDG_CONFIG_HOME"] = home
        return home

    def tearDown(self):
        os.environ.pop("XDG_CONFIG_HOME", None)
        super().tearDown()

    def test_links_for_the_seat_user_only(self):
        home = self.provisioned()
        self.assertEqual(prov.cmd_graph_user(self.args("graph-user"), self.p), 0)
        pw = os.path.join(home, "pipewire", "pipewire.conf.d", "sgv2-j316.conf")
        wp = os.path.join(home, "wireplumber", "wireplumber.conf.d", "60-sgv2-j316.conf")
        self.assertEqual(os.readlink(pw), os.path.join(self.p.graph, "pipewire", "sgv2-j316.conf"))
        self.assertTrue(os.path.islink(wp))
        self.assertEqual(prov.cmd_is_seat(self.args("is-seat"), self.p), 0)
        # Another seat user: the links go, and the helper does not start.
        conf = open(self.p.conf).read().replace(f"SGV2_UID={ME.pw_uid}", f"SGV2_UID={ME.pw_uid + 1}")
        prov.write_atomic(self.p.conf, conf)
        self.assertEqual(prov.cmd_graph_user(self.args("graph-user"), self.p), 0)
        self.assertFalse(os.path.lexists(pw) or os.path.lexists(wp))
        self.assertEqual(prov.cmd_is_seat(self.args("is-seat"), self.p), 1)

    def _cards(self, idx, card, name, pcms):
        a = self.p.proc_asound
        os.makedirs(a, exist_ok=True)
        with open(os.path.join(a, "cards"), "a") as f:
            f.write(f"{idx:2d} [{card:<15s}]: macaudio - {name}\n                      {name}\n")
        for d in pcms:
            os.makedirs(os.path.join(a, f"card{idx}", d), exist_ok=True)

    def test_card_layout(self):
        self._cards(0, "AppleJ616HPAI", "MacBook Pro J616 HPAI", ["pcm0c"])
        self._cards(1, "AppleJ616", "MacBook Pro J616", ["pcm1p", "pcm2c"])
        self._cards(2, "AppleJ413", "MacBook Air J413", ["pcm0p", "pcm0c", "pcm1p", "pcm2c"])
        a = self.p.proc_asound
        self.assertEqual(prov.card_layout(1, a), ("MacBook Pro J616", True))
        self.assertEqual(prov.card_layout(2, a), ("MacBook Air J413", False))
        self.assertEqual(prov.card_layout(7, a), (None, False))

    def test_raw_rules_name_the_card_and_mark_ucm(self):
        """The speaker card's device loses its pro-audio profile; a card of
        speakers and sense alone is hidden from clients too; the UCM marker
        is written beside the rules."""
        import gen
        self._cards(0, "AppleJ316", "MacBook Pro J316", ["pcm0p", "pcm0c", "pcm1p", "pcm2c"])
        self.provisioned()
        text = open(os.path.join(self.p.raw, gen.RAW_RULES)).read()
        self.assertIn('api.alsa.card.name = "MacBook Pro J316"', text)
        self.assertIn("api.acp.disable-pro-audio = true", text)
        self.assertNotIn("asahi.hide-device", text)
        self.assertEqual(open(os.path.join(self.p.graph, gen.UCM_MARKER)).read(), "AppleJ316\n")
        only = gen.wireplumber_raw_conf("AppleJ616", "MacBook Pro J616", True)
        self.assertIn("asahi.hide-device = true", only)
        self.assertIn("node.disabled = true", only)
        self.assertNotIn("api.alsa.card.name", gen.wireplumber_raw_conf("AppleJ616"))

    def test_raw_rules_for_every_user_from_the_start(self):
        """Provisioning writes the raw-PCM rules; every user's session links
        them, before the self-check has passed and when not the seat user."""
        home = self.provisioned()
        raw = os.path.join(self.p.raw, "59-sgv2-raw.conf")
        self.assertTrue(os.path.exists(raw))
        text = open(raw).read()
        self.assertIn("node.disabled = true", text)
        self.assertNotIn("software-dsp", text)
        wpd = os.path.join(home, "wireplumber", "wireplumber.conf.d")
        # the self-check has not passed yet: the raw rules, not the graph
        prov.write_atomic(self.p.graph_state, "result=fail\n")
        self.assertEqual(prov.cmd_graph_user(self.args("graph-user"), self.p), 0)
        self.assertEqual(os.readlink(os.path.join(wpd, "59-sgv2-raw.conf")), raw)
        self.assertFalse(os.path.lexists(os.path.join(wpd, "60-sgv2-j316.conf")))
        # another user (a greeter): the raw rules too
        conf = open(self.p.conf).read().replace(f"SGV2_UID={ME.pw_uid}", f"SGV2_UID={ME.pw_uid + 1}")
        prov.write_atomic(self.p.conf, conf)
        prov.write_atomic(self.p.graph_state, "result=pass\n")
        self.assertEqual(prov.cmd_graph_user(self.args("graph-user"), self.p), 0)
        self.assertTrue(os.path.islink(os.path.join(wpd, "59-sgv2-raw.conf")))
        self.assertFalse(os.path.lexists(os.path.join(wpd, "60-sgv2-j316.conf")))

    def test_no_graph_in_this_boot(self):
        home = self.provisioned()
        prov.cmd_graph_user(self.args("graph-user"), self.p)
        # A boot without v2: the configuration stays in /etc, /run is empty.
        shutil.rmtree(self.p.graph)
        self.assertEqual(prov.cmd_is_seat(self.args("is-seat"), self.p), 1)
        self.assertEqual(prov.cmd_graph_user(self.args("graph-user"), self.p), 0)
        self.assertEqual(os.listdir(os.path.join(home, "pipewire", "pipewire.conf.d")), [])

    def test_a_foreign_file_is_left_alone(self):
        home = self.provisioned()
        d = os.path.join(home, "pipewire", "pipewire.conf.d")
        os.makedirs(d)
        with open(os.path.join(d, "sgv2-j316.conf"), "w") as f:
            f.write("# the user's own\n")
        with open(os.path.join(d, "10-mine.conf"), "w") as f:
            f.write("# the user's own\n")
        os.symlink("/usr/share/x.conf", os.path.join(d, "20-link.conf"))
        prov.cmd_graph_user(self.args("graph-user"), self.p)
        self.assertFalse(os.path.islink(os.path.join(d, "sgv2-j316.conf")))
        self.assertTrue(os.path.islink(os.path.join(d, "20-link.conf")))
        self.assertTrue(os.path.exists(os.path.join(d, "10-mine.conf")))

    def test_activate_while_the_switch_test_is_deferred(self):
        """Before login with the switch test deferred: no graph (an earlier
        one is removed), the seat user's session gets the raw-PCM rules
        alone, the session helper does not start, and nothing fails."""
        home = self.provisioned()
        prov.write_atomic(self.p.deferred, "kernel=k\n")
        self.assertEqual(prov.cmd_activate(self.args("activate"), self.p), 0)
        self.assertEqual(prov.read_kv(self.p.graph_state)["result"], "pending")
        self.assertEqual(os.listdir(os.path.join(self.p.graph, "pipewire")), [])
        self.assertEqual(os.listdir(os.path.join(self.p.graph, "wireplumber")), [])
        # The switch test passed after login: its activate generates the
        # graph while the test is still marked pending (until ExecStopPost).
        digest = prov.sha256_file(self.p.params)
        prov.write_atomic(os.path.join(self.p.state, "selfcheck.bind"),
                          f"result=pass\nkernel={os.uname().release}\nparams_sha256={digest}\n")
        from unittest import mock
        done = types.SimpleNamespace(returncode=0, stderr="", stdout="")
        with mock.patch.object(prov.subprocess, "run", return_value=done) as run:
            self.assertEqual(prov.cmd_activate(self.args("activate"), self.p), 0)
        self.assertIn("generate", run.call_args_list[0][0][0])
        self.assertEqual(prov.read_kv(self.p.graph_state)["result"], "pass")
        # A failed test: no pass, the graph stays down; the session starts
        # with the raw rules alone, and the helper does not start.
        prov.write_atomic(os.path.join(self.p.state, "selfcheck.bind"), "result=fail\n")
        self.assertEqual(prov.cmd_activate(self.args("activate"), self.p), 0)
        self.assertEqual(prov.read_kv(self.p.graph_state)["result"], "pending")
        os.unlink(self.p.deferred)
        self.assertEqual(prov.cmd_graph_user(self.args("graph-user"), self.p), 0)
        wpd = os.path.join(home, "wireplumber", "wireplumber.conf.d")
        self.assertEqual(os.listdir(wpd), ["59-sgv2-raw.conf"])
        self.assertEqual(prov.cmd_is_seat(self.args("is-seat"), self.p), 1)

    def test_sessions_wait_for_a_pending_switch_test(self):
        """No session's PipeWire (and so WirePlumber, whose device probe opens
        the raw PCMs) starts while the switch test is pending; it starts with
        the graph once the test passed."""
        import threading
        home = self.provisioned()
        prov.write_atomic(self.p.deferred, "kernel=k\n")
        prov.write_atomic(self.p.graph_state, "result=pending\n")
        done = []

        def finish():
            time.sleep(0.4)
            prov.write_atomic(self.p.graph_state, "result=pass\n")
            os.unlink(self.p.deferred)
            done.append(time.monotonic())
        th = threading.Thread(target=finish)
        th.start()
        t = time.monotonic()
        self.assertEqual(prov.cmd_graph_user(self.args("graph-user", switch_test_wait=10), self.p), 0)
        th.join()
        self.assertGreaterEqual(time.monotonic(), done[0])
        self.assertGreater(time.monotonic() - t, 0.35)
        self.assertTrue(os.path.islink(os.path.join(home, "pipewire", "pipewire.conf.d", "sgv2-j316.conf")))
        # bounded: a test that never ends does not keep PipeWire down forever
        prov.write_atomic(self.p.deferred, "kernel=k\n")
        t = time.monotonic()
        self.assertEqual(prov.cmd_graph_user(self.args("graph-user", switch_test_wait=0.3), self.p), 0)
        self.assertLess(time.monotonic() - t, 2)

    def test_a_failed_run_clears_the_graph(self):
        self.provisioned()
        self.ctl({"digest": "d9", "compile_fails": True})
        self.assertEqual(self.run_(), 1)
        self.assertEqual(prov.read_kv(self.p.graph_state)["result"], "fail")
        self.assertEqual(os.listdir(os.path.join(self.p.graph, "pipewire")), [])



class Sleep(Base):
    """The sleep hold: set before sleep, cleared after the resume."""

    def test_hold_set_and_cleared(self):
        self.assertEqual(prov.cmd_sleep(self.args("sleep", sleep_wait=0), self.p), 0)
        self.assertTrue(os.path.exists(self.p.hold))
        self.assertTrue(self.p.hold.startswith(self.root))
        self.assertEqual(prov.cmd_wake(self.args("wake"), self.p), 0)
        self.assertFalse(os.path.exists(self.p.hold))
        # A resume without a hold (a sleep v2 did not see) is fine too
        self.assertEqual(prov.cmd_wake(self.args("wake"), self.p), 0)

    def test_session_helper_honours_the_hold(self):
        import importlib.machinery
        import importlib.util
        loader = importlib.machinery.SourceFileLoader("sgv2_session", os.path.join(HERE, "speakerguardd-session"))
        spec = importlib.util.spec_from_loader("sgv2_session", loader)
        ses = importlib.util.module_from_spec(spec)
        loader.exec_module(ses)
        self.assertEqual(ses.SLEEP_HOLD, prov.Paths().hold)

    def test_provision_unit_name(self):
        self.assertEqual(prov.provision_unit("speakerguardd.service"), "speakerguardd-provision.service")
        self.assertEqual(prov.provision_unit("speakerguardd-v2.service"), "speakerguardd-v2-provision.service")

    def test_helper_amps_needs_a_fresh_report(self):
        self.assertIsNone(prov.helper_amps({}, 0))
        self.assertIsNone(prov.helper_amps({"SGV2_UID": "999999999"}, 0))

    def _pcm(self, card, name, first):
        d = os.path.join(self.p.proc_asound, card, name, "sub0")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "status"), "w") as f:
            f.write(first + "\n")
        return os.path.join(d, "status")

    def test_open_pcms_are_the_graph_ones(self):
        gd = os.path.join(self.p.graph, "pipewire")
        os.makedirs(gd, exist_ok=True)
        with open(os.path.join(gd, "sgv2-j413.conf"), "w") as f:
            f.write('api.alsa.path          = "hw:CARD=AppleJ413,DEV=1"\n'
                    'api.alsa.path          = "hw:CARD=AppleJ413,DEV=2"\n')
        spk = self._pcm("AppleJ413", "pcm1p", "state: RUNNING")
        sense = self._pcm("AppleJ413", "pcm2c", "state: SETUP")
        # the headphone PCM is not the graph's: never waited for
        self._pcm("AppleJ413", "pcm0p", "state: RUNNING")
        self.assertEqual(prov.graph_pcms(self.p), [spk, sense])
        self.assertEqual(prov.open_pcms(self.p), [spk, sense])
        self._pcm("AppleJ413", "pcm1p", "closed")
        self.assertEqual(prov.open_pcms(self.p), [sense])
        self._pcm("AppleJ413", "pcm2c", "closed")
        self.assertEqual(prov.open_pcms(self.p), [])

    def test_open_pcms_without_a_graph(self):
        self.assertEqual(prov.open_pcms(self.p), [])

    def test_helper_suspends_under_the_hold_only_when_unlinked(self):
        import importlib.machinery
        import importlib.util
        loader = importlib.machinery.SourceFileLoader("sgv2_session", os.path.join(HERE, "speakerguardd-session"))
        spec = importlib.util.spec_from_loader("sgv2_session", loader)
        ses = importlib.util.module_from_spec(spec)
        loader.exec_module(ses)

        class G:
            def __init__(self, states, links):
                self.states, self._links = states, links
                self.objs = {}

            def node_id(self, name):
                return {"sgv2.speaker.j413": 10, "sgv2.sense.j413": 20}.get(name)

            def node_state(self, nid):
                return self.states.get(nid)

            def of_type(self, t):
                return [{"id": 11, "info": {"props": {"node.id": 10}}},
                        {"id": 21, "info": {"props": {"node.id": 20}}}] if t == "Port" else []

            def links(self):
                return self._links

            def ports(self, nid, d):
                return {}

        s = ses.Session("j413", {"card": "AppleJ413", "amps": 4, "sense": 8})
        spawned = []
        s._spawn = lambda key, argv: spawned.append(argv)
        os.makedirs(self.p.sleep, exist_ok=True)
        hold = self.p.hold
        ses.SLEEP_HOLD = hold
        # no hold: a withdrawal unlinks, never suspends
        s.reconcile(G({10: "idle", 20: "running"}, []), False)
        self.assertEqual(spawned, [])
        open(hold, "w").close()
        # still linked: unlink first
        s.reconcile(G({10: "running", 20: "running"}, [(99, 5, 11)]), False)
        self.assertEqual(spawned, [["pw-link", "-d", "99"]])
        spawned.clear()
        s.reconcile(G({10: "idle", 20: "running"}, []), False)
        self.assertEqual(spawned, [["pw-cli", "send-command", "10", "Suspend", "{}"], ["pw-cli", "send-command", "20", "Suspend", "{}"]])
        spawned.clear()
        s.last_suspend = -10.0
        s.reconcile(G({10: "suspended", 20: "suspended"}, []), False)
        self.assertEqual(spawned, [])


class UserManager(unittest.TestCase):
    """refresh_session reaches the seat user's own manager on the user's bus,
    as that user, never through machined (which fails from a system unit)."""

    def test_reaches_the_user_bus_as_the_user(self):
        me = pwd.getpwuid(os.getuid())
        rt = tempfile.mkdtemp()
        calls = []
        real_run, real_exists = prov.subprocess.run, prov.os.path.exists
        try:
            prov.subprocess.run = lambda cmd, **kw: calls.append((cmd, kw)) or prov.subprocess.CompletedProcess(cmd, 0, "", "")
            prov.os.path.exists = lambda p: p == f"/run/user/{me.pw_uid}/bus" or real_exists(p)
            r = prov.user_systemctl(me.pw_name, "show", "pipewire.service")
        finally:
            prov.subprocess.run, prov.os.path.exists = real_run, real_exists
            shutil.rmtree(rt)
        self.assertEqual(r.returncode, 0)
        cmd, kw = calls[0]
        self.assertNotIn("--machine", " ".join(cmd))
        self.assertEqual(cmd[:2], ["setpriv", f"--reuid={me.pw_uid}"])
        self.assertEqual(cmd[-4:], ["systemctl", "--user", "show", "pipewire.service"])
        self.assertEqual(kw["env"]["XDG_RUNTIME_DIR"], f"/run/user/{me.pw_uid}")
        self.assertEqual(kw["env"]["DBUS_SESSION_BUS_ADDRESS"], f"unix:path=/run/user/{me.pw_uid}/bus")

    def test_no_user_bus_is_no_session(self):
        me = pwd.getpwuid(os.getuid())
        real_exists = prov.os.path.exists
        try:
            prov.os.path.exists = lambda p: False if p.startswith("/run/user/") else real_exists(p)
            r = prov.user_systemctl(me.pw_name, "show", "pipewire.service")
        finally:
            prov.os.path.exists = real_exists
        self.assertNotEqual(r.returncode, 0)


class LV2Shadow(unittest.TestCase):
    """The self-check fails on any sgv2 plugin a session could load instead
    of the packaged one."""

    def setUp(self):
        self.sc = load("selfcheck", "speakerguardd-selfcheck")
        self.root = tempfile.mkdtemp()
        self.home = os.path.join(self.root, "home", "u")
        self.etc = os.path.join(self.root, "etc")
        os.makedirs(self.home); os.makedirs(self.etc)

    def tearDown(self):
        shutil.rmtree(self.root)

    def bundle(self, d, uri="https://aurora-silicon.org/lv2/sgv2#4ch-vi"):
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "manifest.ttl"), "w") as f:
            f.write(f"<{uri}> a lv2:Plugin .\n")

    def scan(self, environ=None):
        return self.sc.lv2_shadows(users=[("u", self.home)], etc=self.etc, environ=environ or {})

    def test_clean(self):
        self.bundle(os.path.join(self.home, ".lv2", "other.lv2"), "urn:other#x")
        self.assertEqual(self.scan(), [])

    def test_user_lv2(self):
        self.bundle(os.path.join(self.home, ".lv2", "sgv2.lv2"))
        found = self.scan()
        self.assertEqual(len(found), 1)
        self.assertIn("~/.lv2", found[0][1])

    def test_renamed_bundle_is_found_by_uri(self):
        self.bundle(os.path.join(self.home, ".lv2", "old-copy.lv2"))
        self.assertEqual(len(self.scan()), 1)

    def test_lv2_path_overrides(self):
        d = os.path.join(self.root, "opt", "lv2")
        self.bundle(os.path.join(d, "sgv2.lv2"))
        os.makedirs(os.path.join(self.home, ".config", "environment.d"))
        with open(os.path.join(self.home, ".config", "environment.d", "50-lv2.conf"), "w") as f:
            f.write(f"LV2_PATH={d}:/usr/lib64/lv2\n")
        self.assertEqual([w for _, w in self.scan()], [f"LV2_PATH in {self.home}/.config/environment.d/50-lv2.conf"])
        with open(os.path.join(self.etc, "environment"), "w") as f:
            f.write(f'export LV2_PATH="{d}"\n')
        self.assertEqual(len(self.scan()), 1)
        self.assertEqual(len(self.scan(environ={"LV2_PATH": d})), 1)

    def test_system_dirs_are_the_packaged_ones(self):
        self.assertEqual(self.scan(environ={"LV2_PATH": "/usr/lib64/lv2:/usr/lib/lv2"}), [])

class Umask(Base):
    def test_graph_dirs_are_readable_whatever_the_callers_umask(self):
        """RC8 E2E B4: aurora-speaker-upgrade runs with umask 077; the graph
        directories it made were 0700 and the seat user's session could not
        read them, so v2 never granted in the same boot."""
        import stat
        shutil.rmtree(self.p.graph, ignore_errors=True)
        old = os.umask(0o077)
        try:
            prov.write_raw_rules(self.p, "AppleJ316")
        finally:
            os.umask(old)
        for d in (self.p.graph, self.p.raw):
            self.assertEqual(stat.S_IMODE(os.stat(d).st_mode), 0o755, d)


    def test_the_blob_from_the_macs_own_tuning_stays_root_only(self):
        """The graph directories open up for the session; the compiled blob
        at rest does not, under the units' umask or the upgrade tool's."""
        import stat
        for mask in (0o022, 0o077):
            old = os.umask(mask)
            try:
                self.assertEqual(self.run_(force=True), 0)
            finally:
                os.umask(old)
            self.assertEqual(stat.S_IMODE(os.stat(self.p.params).st_mode), 0o600, oct(mask))


class Deactivate(Base):
    def test_revert_removes_the_v2_graph(self):
        os.makedirs(os.path.join(self.p.graph, "pipewire"), exist_ok=True)
        f = os.path.join(self.p.graph, "pipewire", "sgv2-j316.conf")
        prov.write_atomic(f, "context.modules = []\n")
        prov.write_atomic(self.p.graph_state, "result=pass\nuser=ryan\n")
        prov.write_raw_rules(self.p, "AppleJ316")
        raw = glob.glob(os.path.join(self.p.raw, "*.conf"))
        self.assertTrue(raw)
        a = types.SimpleNamespace(graph_unit="speakerguardd-graph.service")
        self.assertEqual(prov.cmd_deactivate(a, self.p), 0)
        self.assertFalse(os.path.exists(f))
        self.assertFalse(any(os.path.exists(r) for r in raw))
        self.assertFalse(os.path.exists(os.path.join(self.p.graph, "speakerguardd-owns-speakers")))
        self.assertEqual(prov.read_kv(self.p.graph_state)["result"], "legacy")


class Gate(Base):
    def decide(self, iface="v2", gate_data=None, cmd="v2"):
        g = os.path.join(self.t, "gate.json")
        with open(g, "w") as f:
            json.dump(GATE if gate_data is None else gate_data, f)
        self.patch(gate, "kernel_interface", lambda wait: (iface, "AppleX") if iface else (None, None))
        self.patch(gate, "PROVISION_CONF", os.path.join(self.t, "provision.conf"))
        # the gate runs as root on a Mac (ExecCondition=+...); no waits here
        self.patch(gate, "privileged", lambda: True)
        self.patch(gate, "STABLE_WAIT_S", 0)
        a = types.SimpleNamespace(gate=g, dt=self.dt, tuning_cli=self.cli, wait=0, state=self.p.state, cmd=cmd,
                                  setting=os.path.join(self.t, "upgrade-interface"))
        v2, legacy, why = gate.decide(a)
        self.why = why
        self.assertFalse(v2 and legacy, why)
        return v2, legacy

    def checked(self, result="pass", **kw):
        sc = load("sc", "speakerguardd-selfcheck")
        rec = dict(result=result, board=prov.board_of(self.dt), kernel=os.uname().release,
                   params_sha256=prov.sha256_file(self.p.params), wiring="synthetic-wiring")
        rec.update(kw)
        prov.write_atomic(os.path.join(self.p.state, "selfcheck.bind"), "".join(f"{k}={v}\n" for k, v in rec.items()))
        prov.write_atomic(self.p.selfcheck_json, json.dumps({"record": rec}))
        self.assertTrue(sc.report_matches(self.p.selfcheck_json, {k: str(v) for k, v in rec.items()}) if result == "pass" else True)

    def test_first_boot_prepares_but_only_a_current_check_admits_v2(self):
        self.board("j999")
        self.ctl({"board": "j999"})
        self.assertEqual(self.decide(cmd="prepare"), (True, False))
        self.assertEqual(self.decide(), (False, False))
        self.assertEqual(self.run_(), 0)
        with open(os.path.join(os.path.dirname(self.cli), "calls")) as f:
            self.assertIn("--voicing-pre", f.read())
        self.assertEqual(self.decide(), (False, False))
        self.checked()
        self.assertEqual(self.decide(), (True, False))
        self.assertEqual(self.decide("legacy"), (False, True))
        self.assertEqual(self.decide(None), (False, True))
        self.checked(result="fail")
        self.assertEqual(self.decide(), (False, False))
        self.checked(kernel="another-kernel")
        self.assertEqual(self.decide(), (False, False))
        self.checked(board="another-board")
        self.assertEqual(self.decide(), (False, False))
        self.checked()
        with open(self.p.params, "ab") as f:
            f.write(b"new-blob")
        self.assertEqual(self.decide(), (False, False))

    def test_kernel_interface_reads_the_selector(self):
        self.patch(gate, "speaker_card", lambda: "AppleX")
        for controls, want in (({"Speaker Protection Interface", "Speaker Protection"}, "dual"),
                               ({"Speaker Protection"}, "v2"), (set(), "legacy")):
            self.patch(gate, "has_control", lambda card, name, c=controls: name in c)
            self.assertEqual(gate.kernel_interface(0), (want, "AppleX"))
        self.patch(gate, "has_control", lambda card, name: None)
        self.assertEqual(gate.kernel_interface(0), (None, "AppleX"))

    def test_dual_kernel_runs_v2_only_after_the_upgrade_switched(self):
        # linux#1279: "Speaker Protection Interface" serves both interfaces,
        # the volume lock by default; v2 needs the upgrade's setting too
        self.board("j999")
        self.ctl({"board": "j999"})
        self.assertEqual(self.run_(), 0)
        self.checked()
        setting = os.path.join(self.t, "upgrade-interface")
        self.assertEqual(self.decide("dual"), (False, True))
        with open(setting, "w") as f:
            f.write("legacy\n")
        self.assertEqual(self.decide("dual"), (False, True))
        with open(setting, "w") as f:
            f.write("v2\n")
        self.assertEqual(self.decide("dual"), (True, False))
        # a failing proof on a dual kernel keeps the older daemons, never the floor
        self.checked(result="fail")
        self.assertEqual(self.decide("dual"), (False, True))
        # the v2-only kernel is unchanged
        self.assertEqual(self.decide("v2"), (False, False))
        os.unlink(setting)

    def test_dual_kernel_switched_mac_waits_for_its_switch_test(self):
        """A switched Mac on a kernel its switch test has not run on: neither
        daemon (no older daemon holding the volume lock the test takes); a
        failed test sends it back to the older daemons."""
        setting = os.path.join(self.t, "upgrade-interface")
        self.board("j999")
        self.ctl({"board": "j999"})
        self.assertEqual(self.run_(), 0)
        with open(setting, "w") as f:
            f.write("v2\n")
        self.checked(kernel="another-kernel")
        self.assertEqual(self.decide("dual"), (False, False))
        self.assertEqual(self.decide("dual", cmd="prepare"), (True, False))
        self.checked(result="fail")
        self.assertEqual(self.decide("dual"), (False, True))
        self.checked()
        self.assertEqual(self.decide("dual"), (True, False))
        os.unlink(setting)
        self.checked(kernel="another-kernel")
        self.assertEqual(self.decide("dual"), (False, True))

    def test_dual_kernel_refused_board_keeps_the_older_daemons(self):
        with open(os.path.join(self.t, "upgrade-interface"), "w") as f:
            f.write("v2\n")
        self.board("j999")
        self.ctl({"board": "j999", "refusals": [{"code": "synthetic-refusal"}]})
        self.assertEqual(self.run_(), 1)
        self.assertEqual(self.decide("dual"), (False, True))
        self.assertEqual(self.decide("v2"), (False, False))

    def test_switched_mac_never_gets_legacy_on_a_transient_answer(self):
        """RC8 E2E B1: as speakersafetyd's DynamicUser the gate could not read
        the root-only store and started legacy on a switched Mac. A switched
        Mac on a dual kernel starts the older daemons only on a stable,
        definite refusal; an unreadable setting starts nothing."""
        setting = os.path.join(self.t, "upgrade-interface")
        self.board("j999")
        self.ctl({"board": "j999"})
        self.assertEqual(self.run_(), 0)
        self.checked()
        with open(setting, "w") as f:
            f.write("v2\n")
        self.assertEqual(self.decide("dual", cmd="legacy"), (True, False))
        calls = []
        real = gate.tuning_units

        def flaky(cli, conf=None):
            calls.append(1)
            if len(calls) == 1:
                raise PermissionError(13, "Permission denied", "/var/lib/aurora-speaker-upgrade/bundle")
            return real(cli, conf)
        self.patch(gate, "tuning_units", flaky)
        self.assertEqual(self.decide("dual", cmd="legacy"), (True, False))
        self.assertEqual(len(calls), 2, "asked again")

        def broken(cli, conf=None):
            raise PermissionError(13, "Permission denied", "/var/lib/aurora-speaker-upgrade/bundle")
        self.patch(gate, "tuning_units", broken)
        self.assertEqual(self.decide("dual", cmd="legacy"), (False, False))
        self.assertIn("not starting the older daemons", self.why)
        # not root: an unreadable store is not an answer
        self.patch(gate, "tuning_units", real)
        self.patch(gate, "privileged", lambda: False)
        os.chmod(setting, 0)
        try:
            if os.geteuid() != 0:
                self.assertEqual(gate.upgrade_setting(setting), "unreadable")
        finally:
            os.chmod(setting, 0o600)

    def test_interface_switches_only_dual_and_upgraded(self):
        sets = []
        state = {"iface": "dual"}
        setting = os.path.join(self.t, "upgrade-interface")
        self.patch(gate, "kernel_interface", lambda wait: (state["iface"], "AppleX"))
        self.patch(gate.wiring, "interface_lease", lambda card: sets.append(card) or True)
        a = types.SimpleNamespace(wait=0, setting=setting)
        self.assertEqual(gate.interface(a), 0)
        self.assertEqual(sets, [])
        with open(setting, "w") as f:
            f.write("v2\n")
        self.assertEqual(gate.interface(a), 0)
        self.assertEqual(sets, ["AppleX"])
        state["iface"] = "v2"
        self.assertEqual(gate.interface(a), 0)
        self.assertEqual(sets, ["AppleX"])
        state["iface"] = "dual"

        def refuse(card):
            raise gate.wiring.Refused("-EIO")
        self.patch(gate.wiring, "interface_lease", refuse)
        self.assertEqual(gate.interface(a), 1)

    def test_unlisted_board_silent_check_and_full_voicing_generation(self):
        import gen
        import wiring
        from unittest import mock
        sc = load("sc", "speakerguardd-selfcheck")
        self.board("j999")
        self.ctl({"board": "j999", "unit": "lsm2"})
        self.assertEqual(self.run_(), 0)
        fx = gen.fixture("j700")
        hw = fx["hw"]
        hw.update(board="j999", compatible=["apple,j999", "apple,t8140"],
                  sound_compatible=["apple,j999-macaudio", "apple,macaudio"], model="Synthetic Mac J999")
        w = wiring.derive(hw, fx["tuning"])
        w["method"] = "derived-static"
        desc = wiring.describe(hw, w)
        args = types.SimpleNamespace(state=self.p.state, params=self.p.params, force=False, defer=True, deferred=self.p.deferred,
                                     pipewire_dir=os.path.join(self.t, "pw"), wireplumber_dir=os.path.join(self.t, "wp"))
        with mock.patch.object(sc, "derive_live", return_value=(hw, w, desc, [])), \
                mock.patch.object(sc, "lv2_shadows", return_value=[]), \
                mock.patch.object(sc, "blob_rate", return_value=w["rate"]), \
                mock.patch.object(sc, "find_card", return_value=(0, desc["card"])), \
                mock.patch.object(sc, "pcm_ids", return_value={("Secondary", "p"): 1}) as pcm:
            self.assertEqual(sc.cmd_run(args), 0)
            self.assertEqual(pcm.call_count, 1)
            self.assertEqual(sc.cmd_run(args), 0)  # exact binding: no new probe
            self.assertEqual(pcm.call_count, 1)
            self.assertEqual(self.decide(gate_data=gen.load_gate()), (True, False))
            self.assertEqual(sc.cmd_generate(args), 0)
            with open(os.path.join(args.pipewire_dir, "sgv2-j999.conf")) as f:
                self.assertIn("sgv2#voicing", f.read())
            with open(os.path.join(args.wireplumber_dir, "60-sgv2-j999.conf")) as f:
                self.assertIn("node.software-dsp.rules = []", f.read())
            # Kernel and blob changes re-run the silent check. Until it
            # passes again, the stale graph record cannot admit the board.
            self.checked(kernel="old-kernel")
            self.assertEqual(self.decide(), (False, False))
            self.assertEqual(sc.cmd_run(args), 0)
            self.assertEqual(pcm.call_count, 2)
            with open(self.p.params, "ab") as f:
                f.write(b"changed-blob")
            self.assertEqual(self.decide(), (False, False))
            with self.assertRaises(wiring.Refused):
                sc.cmd_generate(args)
            self.assertEqual(sc.cmd_run(args), 0)
            self.assertEqual(pcm.call_count, 3)
            # A self-check alone cannot approve an uncompiled replacement.
            self.assertEqual(self.decide(), (False, False))

    def test_refusal_or_compiler_failure_stays_muted(self):
        self.board("j999")
        for change in ({"refusals": [{"code": "synthetic-refusal"}]}, {"dump_rc": 1}, {"compile_fails": True}):
            with self.subTest(change=change):
                self.ctl({"board": "j999", **change})
                self.assertEqual(self.run_(), 1)
                self.assertEqual(self.decide(), (False, False))
        self.ctl({"board": "j999"})
        self.assertEqual(self.run_(), 0)
        self.checked()
        self.assertEqual(self.decide(), (True, False))
        self.ctl({"board": "j999", "refusals": [{"code": "new-refusal"}]})
        self.assertEqual(self.decide(), (False, False))

    def test_foreign_and_unbound_provenance_stays_muted(self):
        self.board("j999")
        for change in ({"board": "j998"}, {"unit_id": "0" * 64}, {"unit_bound": False}, {"kind": "Collector"}, {"build": None}):
            with self.subTest(change=change):
                self.ctl({"board": "j999", **change})
                self.assertEqual(self.run_(), 1)
                self.assertEqual(self.decide(), (False, False))

    def test_deny_wins_and_allow_cannot_bypass_proofs_or_parity(self):
        self.assertEqual(self.run_(), 0)
        allow = dict(GATE, board_overrides={"j316s": "allow"})
        self.assertEqual(self.decide(gate_data=allow), (False, False))
        self.checked()
        self.assertEqual(self.decide(gate_data=allow), (True, False))
        self.assertEqual(self.decide(gate_data=dict(allow, denied_boards=["j316s"])), (False, False))
        self.assertEqual(self.decide(gate_data=dict(GATE, board_overrides={"j316s": "deny"})), (False, False))
        self.assertEqual(self.decide(gate_data=dict(allow, units_at_parity=[])), (False, False))
        # Old policy switches and the old hand-written allow file do not
        # bypass qualification even if a distribution accidentally keeps them.
        self.checked(result="fail")
        old = dict(GATE, require_qualified_board=False, qualified={"spp3": ["j316s"]})
        self.assertEqual(self.decide(gate_data=old), (False, False))

    def test_corrupt_or_disagreeing_qualification_records_stay_muted(self):
        self.assertEqual(self.run_(), 0)
        self.checked()
        self.assertEqual(self.decide(), (True, False))
        allow = dict(GATE, board_overrides={"j316s": "allow"})
        for change in ({"provenance": "foreign"}, {"compile_refusals": 1}, {"version": 1}):
            with self.subTest(change=change):
                self.assertEqual(self.run_(), 0)
                rec = self.record()
                rec.update(change)
                prov.write_atomic(self.p.report, json.dumps(rec))
                bind = prov.read_kv(self.p.bind)
                bind.update({k: str(v) for k, v in change.items()})
                prov.write_atomic(self.p.bind, "".join(f"{k}={v}\n" for k, v in bind.items()))
                self.assertEqual(self.decide(gate_data=allow), (False, False))
        self.assertEqual(self.run_(), 0)
        prov.write_atomic(self.p.selfcheck_json, "{}")
        self.assertEqual(self.decide(), (False, False))
        self.checked()
        os.unlink(self.p.bind)
        self.assertEqual(self.decide(), (False, False))

    def test_lsid_requires_its_own_build_parity(self):
        policy = dict(GATE, units_at_parity=["spp3", "atsp", "lsm2"])
        for board in ("k114", "k116"):
            self.board(board)
            self.ctl({"board": board, "unit": "lsm2", "lsid": True})
            self.assertEqual(self.decide(gate_data=policy, cmd="prepare"), (False, False))
            self.assertEqual(self.decide(gate_data=policy), (False, False))
        # An allow for a board can never promote LSID itself.
        policy["board_overrides"] = {"k116": "allow"}
        self.assertEqual(self.decide(gate_data=policy, cmd="prepare"), (False, False))

    def test_four_reference_boards_qualify_without_a_board_list(self):
        import gen
        for board, kind in (("j316s", "spp3"), ("j413", "spp3"), ("j616s", "spp3"), ("j700", "lsm2")):
            with self.subTest(board=board):
                self.board(board)
                self.ctl({"board": board, "unit": kind})
                self.assertEqual(self.run_(), 0)
                self.checked()
                self.assertEqual(self.decide(gate_data=gen.load_gate()), (True, False))

    def test_the_bundle_provisioning_compiles_names_the_units(self):
        with open(os.path.join(self.t, "provision.conf"), "w") as f:
            f.write("SGV2_BUNDLE='/var/lib/speakerguardd-tuning/audio-tuning'\n")
        self.decide(cmd="prepare")
        with open(os.path.join(os.path.dirname(self.cli), "calls")) as f:
            self.assertEqual(f.read().splitlines()[-1], "dump --json /var/lib/speakerguardd-tuning/audio-tuning")

    def test_shipped_data(self):
        with open(os.path.join(HERE, "gate.json")) as f:
            g = json.load(f)
        self.assertEqual(g["units_at_parity"], ["spp3", "lsm2", "atsp"])
        self.assertNotIn("qualified", g)
        self.assertNotIn("require_qualified_board", g)
        self.assertEqual(g["denied_boards"], [])
        self.assertEqual(g["board_overrides"], {})
        self.assertEqual(g["voicing_pre"], {"j316s": True, "j413": True, "j616s": True, "j700": True})
        self.assertFalse(os.path.exists(os.path.join(HERE, "machines.json")))


JACK = b"apple,j616-macaudio-jack\0apple,macaudio\0"


class SpeakerCard(unittest.TestCase):
    """The gate's speaker card is the macaudio card with the speaker link,
    whichever card number the jack-only card took."""

    def card(self, sysfs, n, ident, compat, link):
        node = os.path.join(sysfs, f"card{n}", "device", "of_node")
        os.makedirs(os.path.join(node, "dai-link@0"))
        with open(os.path.join(sysfs, f"card{n}", "id"), "w") as f:
            f.write(ident + "\n")
        with open(os.path.join(node, "compatible"), "wb") as f:
            f.write(compat)
        with open(os.path.join(node, "dai-link@0", "link-name"), "wb") as f:
            f.write(link + b"\0")

    def test_the_jack_card_first_is_skipped(self):
        with tempfile.TemporaryDirectory() as sysfs:
            self.card(sysfs, 0, "AppleJ616HPAI", b"apple,aop-audio\0", b"Speakers")
            self.card(sysfs, 1, "AppleJack", JACK, b"Headphone Jack")
            self.card(sysfs, 2, "AppleJ616", b"apple,j616-macaudio\0apple,macaudio\0", b"Speakers")
            self.assertEqual(gate.speaker_card(sysfs), "AppleJ616")

    def test_the_provisioner_skips_the_jack_card(self):
        with tempfile.TemporaryDirectory() as sysfs:
            self.card(sysfs, 1, "AppleJack", JACK, b"Headphone Jack")
            self.card(sysfs, 2, "AppleJ616", b"apple,j616-macaudio\0apple,macaudio\0", b"Speakers")
            self.assertEqual(prov.find_card("AppleJ616", 0, sysfs), (2, "AppleJ616"))
            with self.assertRaises(prov.Failed):
                prov.find_card("AppleJ614", 0, sysfs)

    def test_no_speaker_link_is_no_speaker_card(self):
        with tempfile.TemporaryDirectory() as sysfs:
            self.card(sysfs, 1, "AppleJack", JACK, b"Headphone Jack")
            self.assertIsNone(gate.speaker_card(sysfs))


class Units(unittest.TestCase):
    """The units' order: provisioning, then the self-check, then the supervisor."""

    def unit(self, name):
        with open(os.path.join(HERE, "..", "systemd", name)) as f:
            return f.read().splitlines()

    def test_order_and_gate(self):
        sup, chk, prv = (self.unit(n) for n in ("speakerguardd.service", "speakerguardd-selfcheck.service",
                                                 "speakerguardd-provision.service"))
        req = lambda u: " ".join(l.split("=", 1)[1] for l in u if l.startswith("Requires="))  # noqa: E731
        aft = lambda u: " ".join(l.split("=", 1)[1] for l in u if l.startswith("After="))  # noqa: E731
        self.assertIn("speakerguardd-provision.service", req(sup))
        self.assertIn("speakerguardd-selfcheck.service", req(sup))
        self.assertIn("speakerguardd-provision.service", aft(sup))
        self.assertIn("speakerguardd-provision.service", req(chk))
        self.assertIn("speakerguardd-provision.service", aft(chk))
        self.assertIn("Before=speakerguardd-selfcheck.service speakerguardd.service", prv)
        # The binding (and the graph it reuses) is in /run before any login,
        # so on a boot without a kernel or tuning change the seat user's
        # PipeWire never starts without it.
        bef = lambda u: " ".join(l.split("=", 1)[1] for l in u if l.startswith("Before="))  # noqa: E731
        self.assertIn("systemd-user-sessions.service", bef(chk))
        self.assertIn("systemd-user-sessions.service", aft(sup))
        self.assertIn("ExecStart=/usr/libexec/speakerguardd/speakerguardd-selfcheck run --defer --params ${SGV2_PARAMS} -- $SGV2_TUNING", chk)
        # The switch test it defers runs after login and before the
        # supervisor, which requires it; it runs only when deferred.
        swt = self.unit("speakerguardd-switchtest.service")
        self.assertIn("speakerguardd-switchtest.service", req(sup))
        self.assertIn("speakerguardd-switchtest.service", aft(sup))
        self.assertIn("systemd-user-sessions.service", aft(swt))
        self.assertIn("speakerguardd-selfcheck.service", aft(swt))
        self.assertIn("speakerguardd-selfcheck.service", req(swt))
        self.assertIn("speakerguardd.service", bef(swt))
        self.assertNotIn("systemd-user-sessions.service", bef(swt))
        self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-selfcheck pending", swt)
        self.assertIn("ExecStart=/usr/libexec/speakerguardd/speakerguardd-selfcheck run --params ${SGV2_PARAMS} -- $SGV2_TUNING", swt)
        self.assertIn("ExecStartPost=/usr/libexec/speakerguardd/speakerguardd-provision activate --graph-unit speakerguardd-graph.service", swt)
        self.assertIn("ExecStopPost=/usr/libexec/speakerguardd/speakerguardd-selfcheck pending --clear", swt)
        # The sessions' PipeWire waits for a pending switch test, beyond its
        # timeout.
        self.assertIn("TimeoutStartSec=180", self.unit("speakerguardd-graph.service"))
        self.assertIn("RequiredBy=speakerguardd.service", swt)
        # Provisioning, which writes the raw-PCM rules every session starts
        # with, is before the self-check and so before every login.
        self.assertIn("speakerguardd-provision.service", aft(chk))
        slp = self.unit("speakerguardd-sleep.service")
        self.assertIn("Before=sleep.target", slp)
        self.assertIn("StopWhenUnneeded=yes", slp)
        self.assertIn("WantedBy=sleep.target", slp)
        self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-gate v2", slp)
        self.assertIn("ExecStart=/usr/libexec/speakerguardd/speakerguardd-provision sleep --unit speakerguardd.service", slp)
        self.assertIn("ExecStop=/usr/libexec/speakerguardd/speakerguardd-provision wake --unit speakerguardd.service", slp)
        self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-gate v2", sup)
        for u in (chk, prv, self.unit("speakerguardd-seat.service")):
            self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-gate prepare", u)
        self.assertIn("ExecStartPost=/usr/libexec/speakerguardd/speakerguardd-provision activate --graph-unit speakerguardd-graph.service", chk)
        ses = self.unit("speakerguardd-session.service")
        self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-provision is-seat", ses)
        # never in root's user manager, where PipeWire itself does not run
        self.assertIn("ConditionUser=!root", ses)
        # Every program a unit runs is in one directory, so a package moves
        # them all with one rewrite.
        for name in ("speakerguardd-provision.service", "speakerguardd-selfcheck.service", "speakerguardd-switchtest.service",
                     "speakerguardd-session.service", "speakerguardd-seat.service", "speakerguardd-graph.service",
                     "speakerguardd-sleep.service"):
            for l in self.unit(name):
                if l.startswith(("ExecStart=", "ExecStartPost=", "ExecStop=", "ExecStopPost=", "ExecCondition=")):
                    self.assertTrue(l.split("=", 1)[1].startswith("/usr/libexec/speakerguardd/"), l)
        self.assertIn("Before=pipewire.service wireplumber.service", self.unit("speakerguardd-graph.service"))
        self.assertIn("ExecCondition=+/usr/libexec/speakerguardd/speakerguardd-gate legacy", self.unit("legacy-gate.conf"))
        for u in (sup, chk, prv, swt):
            self.assertFalse([l for l in u if l.startswith(("OnFailure=", "OnSuccess="))])
        # A unit the gate skips must not fail first on the configuration that
        # only provisioning writes (systemd loads it before ExecCondition=).
        for u in (sup, chk, ses, swt):
            self.assertIn("EnvironmentFile=-/etc/speakerguardd/speakerguardd.conf", u)


if __name__ == "__main__":
    unittest.main()
