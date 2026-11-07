#!/usr/bin/env python3
"""Tests of speakerguardd-provision and speakerguardd-gate, on a test root
with a stand-in tuning reader, device tree and speaker card.

    python3 -I packaging/session/test_provision.py
"""
import contextlib
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
import json, os, sys
ctl = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "ctl.json")))
cmd = sys.argv[1]
log = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calls")
with open(log, "a") as f:
    f.write(" ".join(sys.argv[1:]) + "\n")
if cmd == "identify":
    if ctl.get("no_bundle"):
        print(json.dumps({"error": "no speaker tuning bundle found"})); sys.exit(1)
    print(json.dumps({"source": "/lib/firmware/vendor/apple/audio-tuning", "verified": True, "aid": "AID9",
                      "macos_build": ctl.get("build", "25G83"), "digest": ctl.get("digest", "d1"), "reader": "1.0.0"}))
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
    print(json.dumps({"tuning": {"protection": {"kind": ctl.get("unit", "spp3")}}}))
'''

# The test gate: only j316s has qualified.
GATE = {"units_at_parity": ["spp3"], "require_qualified_board": True, "qualified": {"spp3": ["j316s"]}}


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
    node(".", {"compatible": f"apple,{board}\0apple,t6000\0".encode()})
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
        a = dict(cmd=cmd, bundle=None, force=False, unit="speakerguardd.service", graph_unit="speakerguardd-graph.service", root=self.root,
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
        self.assertEqual(self.calls(), ["identify", "compile"])
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
        # No serial-number: "-". A synthetic one: its domain-separated hash,
        # and another unit recompiles (where the compiler refuses the bundle).
        self.assertEqual(self.run_(), 0)
        self.assertEqual(prov.read_kv(self.p.bind)["unit"], "-")
        with open(os.path.join(self.dt, "serial-number"), "wb") as f:
            f.write(b"SYNTHETIC0A\0")
        self.assertEqual(self.run_(), 0)
        want = hashlib.sha256(b"aurora-speaker-unit-v1\0SYNTHETIC0A").hexdigest()
        self.assertEqual(prov.read_kv(self.p.bind)["unit"], want)
        self.assertEqual(self.calls().count("compile"), 2)
        for path in (self.p.bind, self.p.report, self.p.conf):
            with open(path, "rb") as f:
                self.assertNotIn(b"SYNTHETIC0A", f.read())
        with open(os.path.join(self.dt, "serial-number"), "wb") as f:
            f.write(b"SYNTHETIC0B\0")
        self.assertEqual(self.run_(), 0)
        self.assertEqual(self.calls().count("compile"), 3)
        with open(os.path.join(self.dt, "serial-number"), "wb") as f:
            f.write(b"NOT ONE\0")
        self.assertEqual(prov.unit_id(self.dt), "malformed")

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

class Gate(Base):
    def decide(self, iface="v2", gate_data=None, allow=False):
        g = os.path.join(self.t, "gate.json")
        with open(g, "w") as f:
            json.dump(gate_data or GATE, f)
        self.patch(gate, "kernel_interface", lambda wait: (iface, "AppleX") if iface else (None, None))
        self.patch(gate, "ALLOW", os.path.join(self.t, "allow"))
        self.patch(gate, "PROVISION_CONF", os.path.join(self.t, "provision.conf"))
        if allow:
            with open(gate.ALLOW, "w") as f:
                f.write("j413\n")
        a = types.SimpleNamespace(gate=g, dt=self.dt, tuning_cli=self.cli, wait=0)
        v2, legacy, why = gate.decide(a)
        self.assertFalse(v2 and legacy, why)
        return v2, legacy

    def test_matrix(self):
        self.board("j316s")
        self.assertEqual(self.decide("v2"), (True, False))
        self.assertEqual(self.decide("legacy"), (False, True))
        self.assertEqual(self.decide(None), (False, True))
        # Not qualified: the older daemons, or on a v2 kernel the floor.
        self.board("j413")
        self.assertEqual(self.decide("legacy"), (False, True))
        self.assertEqual(self.decide("v2"), (False, False))
        self.assertEqual(self.decide("v2", allow=True), (True, False))
        # The unit is the one this Mac's tuning names: lsm2 is not at parity,
        # even for a board the spp3 list names.
        self.board("j700")
        self.ctl({"unit": "lsm2"})
        self.assertEqual(self.decide("legacy"), (False, True))
        self.assertEqual(self.decide("v2"), (False, False))
        self.board("j316s")
        self.assertEqual(self.decide("v2"), (False, False))
        # Option A: every unit at parity, no per-board run required.
        a = {"units_at_parity": ["spp3", "lsm2", "atsp"], "require_qualified_board": False}
        self.assertEqual(self.decide("v2", a), (True, False))
        self.board("j413")
        self.ctl({"unit": "spp3"})
        self.assertEqual(self.decide("v2", a), (True, False))
        # A new board: nothing about it is written anywhere.
        self.board("j999")
        self.assertEqual(self.decide("v2", a), (True, False))
        self.assertEqual(self.decide("v2"), (False, False))
        # No tuning unit: never v2.
        self.ctl({"no_bundle": True, "unit": None})
        self.assertEqual(self.decide("v2", a), (False, False))

    def test_the_bundle_provisioning_compiles_names_the_unit(self):
        # SGV2_BUNDLE in provisioning's configuration: the gate reads the
        # same bundle provisioning compiles, not only the installer's copy
        self.board("j700")
        self.ctl({"unit": "lsm2"})
        with open(os.path.join(self.t, "provision.conf"), "w") as f:
            f.write("SGV2_BUNDLE='/var/lib/speakerguardd-tuning/audio-tuning'\n")
        self.decide("v2")
        with open(os.path.join(os.path.dirname(self.cli), "calls")) as f:
            calls = f.read().splitlines()
        self.assertEqual(calls[-1], "dump --json /var/lib/speakerguardd-tuning/audio-tuning")
        os.remove(os.path.join(self.t, "provision.conf"))
        self.decide("v2")
        with open(os.path.join(os.path.dirname(self.cli), "calls")) as f:
            self.assertEqual(f.read().splitlines()[-1], "dump --json")

    def test_shipped_data(self):
        with open(os.path.join(HERE, "gate.json")) as f:
            g = json.load(f)
        # fleet cutover: only the qualified boards of a parity unit run v2
        self.assertEqual(g["units_at_parity"], ["spp3", "lsm2", "atsp"])
        self.assertTrue(g["require_qualified_board"])
        self.assertEqual(sorted(g["qualified"]), ["atsp", "lsm2", "spp3"])
        # the qualified lists stay as the record of audible runs
        qualified = sorted(b for u in g["units_at_parity"] for b in g["qualified"].get(u, []))
        self.assertEqual(
            qualified,
            sorted(["j316s", "j413", "j616s", "j700", "j274", "j293", "j313", "j375c", "j375d", "j473", "j474s", "j475c", "j475d", "j493"]),
        )
        # policy only: no hardware in the gate
        self.assertEqual(sorted(k for k in g if not k.startswith("_")), ["qualified", "require_qualified_board", "units_at_parity", "voicing_pre", "voicing_pre_bound_db", "voicing_pre_lowpass"])
        self.assertEqual(g["voicing_pre"], {"j316s": True, "j413": True, "j616s": True, "j700": True})
        self.assertFalse(os.path.exists(os.path.join(HERE, "machines.json")))


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
        # The graph is in /run before any login, so the seat user's PipeWire
        # never starts without it (a first-boot amp fault otherwise).
        bef = lambda u: " ".join(l.split("=", 1)[1] for l in u if l.startswith("Before="))  # noqa: E731
        self.assertIn("systemd-user-sessions.service", bef(chk))
        self.assertIn("systemd-user-sessions.service", aft(sup))
        slp = self.unit("speakerguardd-sleep.service")
        self.assertIn("Before=sleep.target", slp)
        self.assertIn("StopWhenUnneeded=yes", slp)
        self.assertIn("WantedBy=sleep.target", slp)
        self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-gate v2", slp)
        self.assertIn("ExecStart=/usr/libexec/speakerguardd/speakerguardd-provision sleep --unit speakerguardd.service", slp)
        self.assertIn("ExecStop=/usr/libexec/speakerguardd/speakerguardd-provision wake --unit speakerguardd.service", slp)
        for u in (sup, chk, prv, self.unit("speakerguardd-seat.service")):
            self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-gate v2", u)
        self.assertIn("ExecStartPost=/usr/libexec/speakerguardd/speakerguardd-provision activate --graph-unit speakerguardd-graph.service", chk)
        ses = self.unit("speakerguardd-session.service")
        self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-provision is-seat", ses)
        # never in root's user manager, where PipeWire itself does not run
        self.assertIn("ConditionUser=!root", ses)
        # Every program a unit runs is in one directory, so a package moves
        # them all with one rewrite.
        for name in ("speakerguardd-provision.service", "speakerguardd-selfcheck.service", "speakerguardd-session.service",
                     "speakerguardd-seat.service", "speakerguardd-graph.service", "speakerguardd-sleep.service"):
            for l in self.unit(name):
                if l.startswith(("ExecStart=", "ExecStartPost=", "ExecStop=", "ExecCondition=")):
                    self.assertTrue(l.split("=", 1)[1].startswith("/usr/libexec/speakerguardd/"), l)
        self.assertIn("Before=pipewire.service wireplumber.service", self.unit("speakerguardd-graph.service"))
        self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-gate legacy", self.unit("legacy-gate.conf"))
        for u in (sup, chk, prv):
            self.assertFalse([l for l in u if l.startswith(("OnFailure=", "OnSuccess="))])
        # A unit the gate skips must not fail first on the configuration that
        # only provisioning writes (systemd loads it before ExecCondition=).
        for u in (sup, chk, ses):
            self.assertIn("EnvironmentFile=-/etc/speakerguardd/speakerguardd.conf", u)


if __name__ == "__main__":
    unittest.main()
