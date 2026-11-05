#!/usr/bin/env python3
"""Tests of speakerguardd-provision and speakerguardd-gate, on a test root
with a stand-in tuning reader, device tree and speaker card.

    python3 -I packaging/session/test_provision.py
"""
import importlib.machinery
import importlib.util
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
elif cmd == "dump":
    print(json.dumps({"tuning": {"protection": {"kind": ctl.get("unit", "spp3")}}}))
'''

MACHINES = {
    "j316": {"unit": "spp3", "qualified": ["j316s"], "boards": ["j316s", "j316c"], "card": "AppleJ316"},
    "j413": {"unit": "spp3", "qualified": [], "boards": ["j413"], "card": "AppleJ413"},
    "j700": {"unit": "lsm2", "qualified": [], "boards": ["j700"], "card": "AppleJ700"},
}

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
            f.write(FAKE_CLI)
        os.chmod(self.cli, 0o755)
        self.ctl({})
        self.machines = os.path.join(self.t, "machines.json")
        with open(self.machines, "w") as f:
            json.dump(MACHINES, f)
        self.p = prov.Paths(self.root)
        self.saved = {}
        # The live machine's card, seat and users are stand-ins.
        self.patch(prov, "find_card", lambda entry, wait: (0, entry["card"] if entry else "AppleX"))
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
        with open(os.path.join(self.dt, "compatible"), "wb") as f:
            f.write(f"apple,{b}\0apple,t6000\0".encode())

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
                 machines=self.machines, tuning_cli=self.cli, dt=self.dt, etc=os.path.join(self.t, "etc"), card_wait=0)
        a.update(kw)
        return types.SimpleNamespace(**a)

    def run_(self, **kw):
        return prov.cmd_run(self.args(**kw), self.p)

    def record(self):
        with open(self.p.report) as f:
            return json.load(f)


class Provision(Base):
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
                       lambda: self.rebind(compiler="0.9:feedface"), self.tamper):
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
        def nocard(entry, wait):
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
        p, ov = self.p, {}
        os.makedirs(p.state)
        # The only regular user.
        self.assertEqual(prov.resolve_seat_user(p, ov, (1000, 60000), self.t), (ME.pw_name, "only regular user"))
        # Several: none, until something names one.
        self.patch(prov, "regular_users", lambda rng: [ME.pw_name, "zz-other"])
        with self.assertRaises(prov.Failed):
            prov.resolve_seat_user(p, ov, (1000, 60000), self.t)
        self.patch(prov, "autologin_user", lambda etc: ME.pw_name)
        self.assertEqual(prov.resolve_seat_user(p, ov, (1000, 60000), self.t)[1], "automatic login")
        prov.write_atomic(p.seat_last, ME.pw_name + "\n")
        self.assertEqual(prov.resolve_seat_user(p, ov, (1000, 60000), self.t)[1], "last seat0 user")
        self.patch(prov, "seat0_user", lambda rng: ME.pw_name)
        self.assertEqual(prov.resolve_seat_user(p, ov, (1000, 60000), self.t)[1], "seat0")
        self.assertEqual(prov.resolve_seat_user(p, {"SGV2_SEAT_USER": ME.pw_name}, (1000, 60000), self.t)[1], "provision.conf")
        with self.assertRaises(prov.Failed):
            prov.resolve_seat_user(p, {"SGV2_SEAT_USER": "no-such-user-sgv2"}, (1000, 60000), self.t)

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


class Gate(Base):
    def decide(self, iface="v2", gate_data=None, allow=False):
        g = os.path.join(self.t, "gate.json")
        with open(g, "w") as f:
            json.dump(gate_data or {"units_at_parity": ["spp3"], "require_qualified_board": True}, f)
        self.patch(gate, "kernel_interface", lambda entry, wait: (iface, "AppleX") if iface else (None, None))
        self.patch(gate, "ALLOW", os.path.join(self.t, "allow"))
        if allow:
            with open(gate.ALLOW, "w") as f:
                f.write("j413\n")
        a = types.SimpleNamespace(gate=g, machines=self.machines, dt=self.dt, tuning_cli=self.cli, wait=0)
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
        # lsm2 is not at parity.
        self.board("j700")
        self.assertEqual(self.decide("legacy"), (False, True))
        self.assertEqual(self.decide("v2"), (False, False))
        # Option A: every unit at parity, no per-board run required.
        a = {"units_at_parity": ["spp3", "lsm2", "atsp"], "require_qualified_board": False}
        self.assertEqual(self.decide("v2", a), (True, False))
        self.board("j413")
        self.assertEqual(self.decide("v2", a), (True, False))
        # A board with no description: its tuning names the unit.
        self.board("j999")
        self.ctl({"unit": "spp3"})
        self.assertEqual(self.decide("v2", a), (True, False))
        self.ctl({"unit": "lsm2"})
        self.assertEqual(self.decide("v2"), (False, False))

    def test_shipped_data(self):
        with open(os.path.join(HERE, "gate.json")) as f:
            g = json.load(f)
        with open(os.path.join(HERE, "machines.json")) as f:
            m = json.load(f)
        self.assertEqual(g["units_at_parity"], ["spp3"])
        self.assertTrue(g["require_qualified_board"])
        for k, e in m.items():
            self.assertIn(e["unit"], ("spp3", "lsm2", "atsp"), k)
            self.assertTrue(set(e["qualified"]) <= set(e["boards"]), k)
        qualified = sorted(b for e in m.values() if e["unit"] in g["units_at_parity"] for b in e["qualified"])
        self.assertEqual(qualified, ["j316s", "j616s"])


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
        for u in (sup, chk, prv, self.unit("speakerguardd-seat.service")):
            self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-gate v2", u)
        self.assertIn("ExecStartPost=/usr/libexec/speakerguardd/speakerguardd-provision activate --graph-unit speakerguardd-graph.service", chk)
        ses = self.unit("speakerguardd-session.service")
        self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-provision is-seat", ses)
        # Every program a unit runs is in one directory, so a package moves
        # them all with one rewrite.
        for name in ("speakerguardd-provision.service", "speakerguardd-selfcheck.service", "speakerguardd-session.service",
                     "speakerguardd-seat.service", "speakerguardd-graph.service"):
            for l in self.unit(name):
                if l.startswith(("ExecStart=", "ExecStartPost=", "ExecCondition=")):
                    self.assertTrue(l.split("=", 1)[1].startswith("/usr/libexec/speakerguardd/"), l)
        self.assertIn("Before=pipewire.service wireplumber.service", self.unit("speakerguardd-graph.service"))
        self.assertIn("ExecCondition=/usr/libexec/speakerguardd/speakerguardd-gate legacy", self.unit("legacy-gate.conf"))
        for u in (sup, chk, prv):
            self.assertFalse([l for l in u if l.startswith(("OnFailure=", "OnSuccess="))])


if __name__ == "__main__":
    unittest.main()
