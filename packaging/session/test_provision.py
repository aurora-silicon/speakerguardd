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
    print(json.dumps({"source": "/lib/firmware/vendor/apple/audio-tuning", "verified": True, "aid": ctl.get("aid", "AID9"),
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
        self.patch(prov, "find_card", lambda hw, want, wait, **kw: (0, want))
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


    def test_the_running_cards_own_id_names_the_card_for_both_writers(self):
        # The card bound to the sound node has an id other than the one
        # macaudio derives from the model: provisioning passes with the
        # running card's id, and the self-check's lookup (and so its
        # generated graph) names the same card.
        import gen
        import wiring
        sc = load("sc", "speakerguardd-selfcheck")
        sysfs, proc = os.path.join(self.t, "sys"), os.path.join(self.t, "proc")
        for n, of, ident in ((1, os.path.join(self.t, "jack"), "AppleJack"), (2, os.path.join(self.dt, "sound"), "AppleJ316Live")):
            os.makedirs(of, exist_ok=True)
            os.makedirs(os.path.join(sysfs, f"card{n}", "device"))
            os.symlink(of, os.path.join(sysfs, f"card{n}", "device", "of_node"))
            open(os.path.join(sysfs, f"card{n}", f"controlC{n}"), "w").close()
            os.makedirs(os.path.join(proc, f"card{n}"))
            with open(os.path.join(proc, f"card{n}", "id"), "w") as f:
                f.write(ident + "\n")
        self.patch(prov, "find_card", self.saved.get((prov, "find_card"), prov.find_card))
        real = wiring.live_card
        self.patch(wiring, "live_card", lambda hw, dt="/proc/device-tree", *a, **kw: real(hw, dt, sysfs, proc))
        self.assertEqual(self.run_(), 0)
        conf = prov.read_kv(self.p.conf)
        self.assertTrue(conf["SGV2_ARGS"].startswith("--card AppleJ316Live --pcm 1 "), conf["SGV2_ARGS"])
        hw = wiring.read_dt(self.dt)
        self.assertNotEqual(wiring.card_id(hw), "AppleJ316Live")
        live = sc.find_card(hw, self.dt)
        self.assertEqual(live, (2, "AppleJ316Live"))
        fx = gen.fixture("j316")
        w = wiring.derive(fx["hw"], fx["tuning"])
        text = gen.pipewire_conf(w["key"], dict(wiring.describe(fx["hw"], w, live), board="j316s"),
                                 (w["unit_channels"], w["sense_ports"], w["hash"]), "test", voicing_pre=False)
        self.assertIn("hw:CARD=AppleJ316Live,", text)

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
        # The unit ID is root-only at rest, on a pass and on a failure alike.
        for path in (self.p.bind, self.p.report):
            self.assertEqual(os.stat(path).st_mode & 0o777, 0o600, path)
        # Another Mac's extraction and a missing/malformed local identity
        # must never reuse a blob or compile in open-loop fallback mode.
        with open(os.path.join(self.dt, "serial-number"), "wb") as f:
            f.write(b"SYNTHETIC0B\0")
        self.assertEqual(self.run_(), 1)
        self.assert_closed("provenance")
        for path in (self.p.bind, self.p.report):
            self.assertEqual(os.stat(path).st_mode & 0o777, 0o600, path)
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
        def nocard(hw, want, wait, **kw):
            raise prov.Failed("no ALSA card bound to the device tree's sound node after 0 s")
        self.patch(prov, "find_card", nocard)
        self.assertEqual(self.run_(), 1)
        self.assert_closed("no ALSA card bound")

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

    def seat_change(self, is_enabled, show):
        """seat0 changes to a regular user nobody provisioned (the first
        login on a fresh install). `is_enabled` is `systemctl is-enabled`'s
        exit status, `show` the unit's `systemctl show` properties. The
        systemctl calls made, in order."""
        calls = []

        def run(cmd, **kw):
            calls.append(cmd)
            if cmd[1] == "is-enabled":
                return prov.subprocess.CompletedProcess(cmd, is_enabled)
            if cmd[1] == "show":
                out = "".join(f"{k}={v}\n" for k, v in show.items())
                return prov.subprocess.CompletedProcess(cmd, 0, out, "")
            return prov.subprocess.CompletedProcess(cmd, 0)
        self.patch(prov, "seat0_user", lambda rng: "alice")
        self.patch(prov.subprocess, "run", run)
        os.makedirs(self.p.state, exist_ok=True)
        self.assertEqual(prov.cmd_seat(self.args("seat"), self.p), 0)
        return calls

    def restarted(self, calls):
        return ["systemctl", "restart", "--no-block", "speakerguardd.service"] in calls

    def test_seat_restarts_a_unit_the_package_wants_without_enabling_it(self):
        # Arch (alarm-aurora): pacman applies no presets, so the package ships
        # its multi-user.target.wants link in /usr/lib. `is-enabled` says
        # "disabled", but multi-user.target starts the unit every boot: the
        # first login must provision, not wait for the next boot.
        calls = self.seat_change(1, {"LoadState": "loaded", "UnitFileState": "disabled",
                                     "WantedBy": "multi-user.target", "RequiredBy": ""})
        self.assertTrue(self.restarted(calls))

    def test_seat_restarts_an_enabled_unit(self):
        # Fedora (presets) and `systemctl enable`: the link is in /etc.
        calls = self.seat_change(0, {})
        self.assertTrue(self.restarted(calls))

    def test_seat_leaves_a_unit_nothing_starts(self):
        # Disabled by the administrator where the link was in /etc: nothing
        # pulls the unit in, so nothing is provisioned.
        calls = self.seat_change(1, {"LoadState": "loaded", "UnitFileState": "disabled",
                                     "WantedBy": "", "RequiredBy": ""})
        self.assertFalse(self.restarted(calls))

    def test_seat_leaves_a_masked_unit(self):
        # Masked over the package's own link (the only way to turn it off on
        # Arch): the target still lists it, but it never starts.
        for load, state in (("masked", "masked"), ("loaded", "masked-runtime")):
            calls = self.seat_change(1, {"LoadState": load, "UnitFileState": state,
                                         "WantedBy": "multi-user.target", "RequiredBy": ""})
            self.assertFalse(self.restarted(calls))

    def test_seat_leaves_a_unit_systemctl_cannot_show(self):
        calls = []

        def run(cmd, **kw):
            calls.append(cmd)
            return prov.subprocess.CompletedProcess(cmd, 1 if cmd[1] in ("is-enabled", "show") else 0, "", "")
        self.patch(prov, "seat0_user", lambda rng: "alice")
        self.patch(prov.subprocess, "run", run)
        os.makedirs(self.p.state, exist_ok=True)
        self.assertEqual(prov.cmd_seat(self.args("seat"), self.p), 0)
        self.assertFalse(self.restarted(calls))

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

    def test_the_seat_user_without_v2_this_boot_is_told_so(self):
        """A boot without v2 (no /run graph) or before the self-check passed:
        the seat user's session is told v2 is not provisioned in this boot,
        never that it is not the seat user."""
        self.provisioned()

        def said(cmd):
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                getattr(prov, cmd)(self.args("graph-user"), self.p)
            return err.getvalue()

        shutil.rmtree(self.p.graph)
        out = said("cmd_graph_user")
        self.assertNotIn("is not the seat user", out)
        self.assertIn(f"uid {ME.pw_uid} is the seat user, but speakerguardd v2 is not provisioned in this boot", out)
        os.makedirs(self.p.graph)
        prov.write_atomic(self.p.graph_state, "result=pending\n")
        out = said("cmd_graph_user")
        self.assertNotIn("is not the seat user", out)
        self.assertIn("graph is not ready in this boot (result=pending)", out)
        out = said("cmd_is_seat")
        self.assertIn("not provisioned in this boot", out)
        # another user: told it is not the seat user, by name
        conf = open(self.p.conf).read().replace(f"SGV2_UID={ME.pw_uid}", f"SGV2_UID={ME.pw_uid + 1}")
        prov.write_atomic(self.p.conf, conf)
        for cmd in ("cmd_graph_user", "cmd_is_seat"):
            self.assertIn(f"uid {ME.pw_uid} is not the seat user ({ME.pw_name})", said(cmd))
        # nothing provisioned at all
        prov.write_atomic(self.p.conf, "# empty\n")
        self.assertIn("speakerguardd v2 is not provisioned (no seat user)", said("cmd_graph_user"))

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
        # The passing activate ends the pending state itself.
        self.assertFalse(os.path.exists(self.p.deferred))
        # A failed test: no pass, the graph stays down; the session starts
        # with the raw rules alone, and the helper does not start.
        prov.write_atomic(self.p.deferred, "kernel=k\n")
        prov.write_atomic(os.path.join(self.p.state, "selfcheck.bind"), "result=fail\n")
        self.assertEqual(prov.cmd_activate(self.args("activate"), self.p), 0)
        self.assertEqual(prov.read_kv(self.p.graph_state)["result"], "pending")
        os.unlink(self.p.deferred)
        self.assertEqual(prov.cmd_graph_user(self.args("graph-user"), self.p), 0)
        wpd = os.path.join(home, "wireplumber", "wireplumber.conf.d")
        self.assertEqual(os.listdir(wpd), ["59-sgv2-raw.conf"])
        self.assertEqual(prov.cmd_is_seat(self.args("is-seat"), self.p), 1)

    def test_switch_test_activate_does_not_wait_on_the_session(self):
        """The switch test passed while the seat user's PipeWire runs (a
        switch in a live session): activate starts the user's graph unit and
        waits for it. That unit waits while the switch test is pending, so
        activate must end the pending state first, or the two wait on each
        other until the user manager's timeout and the switch test fails."""
        home = self.provisioned()
        prov.write_atomic(self.p.deferred, "kernel=k\n")
        digest = prov.sha256_file(self.p.params)
        prov.write_atomic(os.path.join(self.p.state, "selfcheck.bind"),
                          f"result=pass\nkernel={os.uname().release}\nparams_sha256={digest}\n")
        started = []

        def refresh(p, user, changed_us, unit):
            # systemctl --user start <unit>: runs graph-user to completion
            t = time.monotonic()
            self.assertEqual(prov.cmd_graph_user(self.args("graph-user", switch_test_wait=5), self.p), 0)
            started.append(time.monotonic() - t)
        self.patch(prov, "refresh_session", refresh)
        self.p.root = ""  # the paths stay under the test root; take the live-session branch
        from unittest import mock
        done = types.SimpleNamespace(returncode=0, stderr="", stdout="")
        with mock.patch.object(prov.subprocess, "run", return_value=done):
            self.assertEqual(prov.cmd_activate(self.args("activate"), self.p), 0)
        self.assertEqual(len(started), 1)
        self.assertLess(started[0], 1.0)
        self.assertTrue(os.path.islink(os.path.join(home, "pipewire", "pipewire.conf.d", "sgv2-j316.conf")))

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
        # unlinks as pw-link -d (no persistent connection here)
        s.cli = None
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



class FakeUserManager:
    """A stand-in for the seat user's systemd --user (user_systemctl), on a
    fake clock: units stop at once, or (hung) ignore SIGTERM and stay
    "deactivating" until SIGKILL, as WirePlumber did on a swapping j414s."""

    def __init__(self, active, hung=(), unkillable=(), session=True):
        self.now = 0.0
        self.state = {u: ("active" if u in active else "inactive") for u in prov.SESSION_UNITS}
        self.hung, self.unkillable, self.session = set(hung), set(unkillable), session
        self.calls = []
        self.killed = []

    def monotonic(self):
        return self.now

    def sleep(self, t):
        self.now += t

    def __call__(self, user, *a, timeout=60):
        self.calls.append((self.now, a))
        self.now += 0.05
        if not self.session:
            return types.SimpleNamespace(returncode=1, stdout="", stderr="no user bus")
        if a[0] == "show":
            # PipeWire's stop waits for WirePlumber's (WirePlumber is ordered after it)
            for u in ("pipewire.service", "pipewire.socket"):
                if self.state.get(u) == "stopping" and self.state["wireplumber.service"] in ("inactive", "failed"):
                    self.state[u] = "inactive"
            units = [x for x in a[1:] if not x.startswith("--")]
            shown = lambda u: "active" if self.state[u] == "stopping" else self.state[u]  # noqa: E731
            out = "\n\n".join(f"Id={u}\nLoadState=loaded\nActiveState={shown(u)}" for u in units if u in self.state)
            return types.SimpleNamespace(returncode=0, stdout=out + "\n", stderr="")
        units = [x for x in a[1:] if not x.startswith("--")]
        if a[0] == "stop":
            for u in units:
                if u in self.hung:
                    self.state[u] = "deactivating"
                elif u in ("pipewire.service", "pipewire.socket") and "wireplumber.service" in self.hung:
                    self.state[u] = "stopping"  # queued behind WirePlumber, shown as active
                else:
                    self.state[u] = "inactive"
        elif a[0] == "kill":
            assert "--signal=SIGKILL" in a
            self.killed += units
            for u in units:
                if u not in self.unkillable:
                    self.state[u] = "failed"
        elif a[0] == "reset-failed":
            for u in units:
                if self.state.get(u) == "failed":
                    self.state[u] = "inactive"
        elif a[0] == "start":
            for u in units:
                if u in self.state:
                    self.state[u] = "active"
        return types.SimpleNamespace(returncode=0, stdout="", stderr="")

    def verbs(self):
        return [c[1][0] for c in self.calls if c[1][0] != "show"]


class SessionStopStart(unittest.TestCase):
    """aurora-speaker-upgrade stops the seat user's audio session before the
    kernel's speaker interface switches and starts it after: bounded, with
    SIGKILL for a unit still stopping after the grace."""

    ALL = prov.SESSION_UNITS

    def setUp(self):
        self.t = tempfile.mkdtemp()
        self.p = prov.Paths(self.t)

    def tearDown(self):
        shutil.rmtree(self.t)

    @contextlib.contextmanager
    def manager(self, m):
        from unittest import mock
        with mock.patch.object(prov, "user_systemctl", m), mock.patch.object(prov.time, "monotonic", m.monotonic), \
                mock.patch.object(prov.time, "sleep", m.sleep), mock.patch.object(prov, "log", lambda *a: None):
            yield m

    def test_normal_stop_then_start(self):
        m = FakeUserManager(active=self.ALL)
        with self.manager(m):
            out = prov.session_stop(self.p, "u")
            self.assertEqual(out["stopped"], list(self.ALL))
            self.assertEqual((out["killed"], out["stuck"]), ([], []))
            self.assertLess(out["seconds"], 2)
            self.assertTrue(all(m.state[u] == "inactive" for u in self.ALL))
            self.assertEqual(prov.read_session_record(self.p), ("u", list(self.ALL)))
            self.assertNotIn("kill", m.verbs())
            self.assertEqual(prov.session_start(self.p, "speakerguardd-graph.service"), [])
        self.assertTrue(all(m.state[u] == "active" for u in self.ALL))
        self.assertFalse(os.path.exists(self.p.session_stopped))
        starts = [c[1] for c in m.calls if c[1][0] == "start"]
        # the graph unit links the graph before PipeWire reads its configuration
        self.assertEqual(starts[0], ("start", "speakerguardd-graph.service"))
        self.assertEqual(starts[1][:2], ("start", "--no-block"))
        self.assertNotIn("try-restart", m.verbs())

    def test_hung_wireplumber_is_killed_after_the_grace(self):
        m = FakeUserManager(active=self.ALL, hung={"wireplumber.service"})
        with self.manager(m):
            out = prov.session_stop(self.p, "u", grace=15, kill_wait=10)
        # only the unit that ignored SIGTERM; PipeWire, queued behind it, stops by itself
        self.assertEqual(out["killed"], ["wireplumber.service"])
        self.assertEqual(m.killed, ["wireplumber.service"])
        self.assertEqual(out["stuck"], [])
        self.assertTrue(all(m.state[u] == "inactive" for u in self.ALL))
        kill_at = [c[0] for c in m.calls if c[1][0] == "kill"]
        self.assertEqual(len(kill_at), 1)
        self.assertGreaterEqual(kill_at[0], 15)
        self.assertLess(kill_at[0], 17)
        self.assertLess(out["seconds"], 15 + 10 + prov.CALL_SLACK_S)
        self.assertEqual(m.state["wireplumber.service"], "inactive")  # reset-failed after the kill
        self.assertIn("reset-failed", m.verbs())

    def test_unkillable_is_bounded_and_fails(self):
        m = FakeUserManager(active=self.ALL, hung={"wireplumber.service"}, unkillable={"wireplumber.service"})
        with self.manager(m):
            out = prov.session_stop(self.p, "u", grace=15, kill_wait=10)
            self.assertEqual(out["stuck"], ["wireplumber.service"])
            self.assertLessEqual(m.now, 15 + 10 + prov.CALL_SLACK_S + 1)
            a = types.SimpleNamespace(user="u", stop_grace=15, kill_wait=10, etc="/etc")
            self.assertEqual(prov.cmd_session_stop(a, self.p), 1)

    def test_only_what_ran_starts_again(self):
        m = FakeUserManager(active={"pipewire.socket", "pipewire.service", "wireplumber.service"})
        with self.manager(m):
            prov.session_stop(self.p, "u")
            self.assertEqual(prov.read_session_record(self.p)[1], ["pipewire.socket", "wireplumber.service", "pipewire.service"])
            prov.session_start(self.p, "g.service")
        self.assertEqual(m.state["pipewire-pulse.service"], "inactive")
        self.assertEqual(m.state["pipewire-pulse.socket"], "inactive")
        self.assertEqual(m.state["wireplumber.service"], "active")

    def test_a_second_stop_keeps_the_first_record(self):
        # the rollback stops again (nothing runs any more) and must still
        # start what ran before the first stop
        m = FakeUserManager(active=self.ALL)
        with self.manager(m):
            prov.session_stop(self.p, "u")
            out = prov.session_stop(self.p, "u")
            self.assertEqual(out["stopped"], [])
            self.assertEqual(prov.read_session_record(self.p), ("u", list(self.ALL)))
            prov.session_start(self.p, "g.service")
        self.assertTrue(all(m.state[u] == "active" for u in self.ALL))

    def test_no_session(self):
        m = FakeUserManager(active=self.ALL, session=False)
        with self.manager(m):
            self.assertIsNone(prov.session_stop(self.p, "u"))
            self.assertEqual(prov.session_start(self.p, "g.service"), [])
        self.assertFalse(os.path.exists(self.p.session_stopped))
        self.assertEqual(m.verbs(), [])

    def test_start_without_a_record_does_nothing(self):
        m = FakeUserManager(active=())
        with self.manager(m):
            self.assertEqual(prov.session_start(self.p, "g.service"), [])
        self.assertEqual(m.calls, [])

    def test_refresh_restarts_bounded_never_try_restart(self):
        m = FakeUserManager(active=self.ALL, hung={"wireplumber.service"})
        real = m.__call__

        def mgr(user, *a, timeout=60):
            if a[0] == "show" and "--property=ActiveEnterTimestampMonotonic" in a:
                m.calls.append((m.now, a))
                return types.SimpleNamespace(returncode=0, stdout="ActiveState=active\nActiveEnterTimestampMonotonic=5\n", stderr="")
            return real(user, *a, timeout=timeout)
        mgr.monotonic, mgr.sleep = m.monotonic, m.sleep
        with self.manager(mgr):
            prov.refresh_session(self.p, "u", 10, "g.service")
        self.assertNotIn("try-restart", m.verbs())
        self.assertIn("kill", m.verbs())
        self.assertTrue(all(m.state[u] == "active" for u in self.ALL))
        self.assertLess(m.now, 15 + 10 + prov.CALL_SLACK_S + prov.START_WAIT_S)

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


    def test_an_unreadable_config_dir_is_repaired(self):
        """J815: /etc/speakerguardd left 0700 by aurora-speaker-upgrade's
        umask 077; systemd only warns about it. The seat user's session read
        no speakerguardd.conf ("none provisioned") and linked no graph."""
        import stat
        os.makedirs(self.p.etc, exist_ok=True)
        os.chmod(self.p.etc, 0o700)
        self.assertEqual(self.run_(force=True), 0)
        self.assertEqual(stat.S_IMODE(os.stat(self.p.etc).st_mode), 0o755)
        self.assertEqual(stat.S_IMODE(os.stat(self.p.conf).st_mode), 0o644)

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
    def decide(self, iface="v2", gate_data=None, cmd="v2", card=None):
        g = os.path.join(self.t, "gate.json")
        with open(g, "w") as f:
            json.dump(GATE if gate_data is None else gate_data, f)
        self.patch(gate, "kernel_interface", lambda wait, **kw: (iface, "AppleX") if iface else (None, card))
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

    def test_unreadable_controls_on_a_registered_card_start_neither(self):
        # the speaker card is there but its controls could not be read in
        # the wait: no answer, so neither stack starts (never the older
        # daemons beside v2); no speaker card at all keeps today's answer
        self.board("j999")
        self.ctl({"board": "j999"})
        for cmd in ("v2", "legacy", "prepare"):
            self.assertEqual(self.decide(None, cmd=cmd, card="AppleX"), (False, False), cmd)
            self.assertIn("could not be read", self.why)
        self.assertEqual(self.decide(None), (False, True))

    def test_has_control_tells_a_missing_control_from_a_failed_open(self):
        # only "the card opened and has no such control" is a "no": amixer
        # failing to find or open the card (still registering, gone, no
        # access) says nothing about the kernel's controls
        out = {}
        self.patch(gate.subprocess, "run", lambda cmd, **kw: types.SimpleNamespace(**out))
        self.patch(gate, "log", lambda *a: None)
        for rc, err, want in (
                (0, "", True),
                (1, "amixer: Cannot find the given element from control sysdefault:1\n\n", False),
                (1, "Invalid card number 'AppleJ700'.\n", None),
                (1, "amixer: Control sysdefault:1 open error: No such device\n\n", None),
                (1, "amixer: Control sysdefault:1 open error: Permission denied\n\n", None)):
            out.update(returncode=rc, stdout="", stderr=err)
            self.assertIs(gate.has_control("AppleJ700", "Speaker Protection"), want, err)

    def test_kernel_interface_asks_again_within_the_wait(self):
        # the card was found but amixer could not read it yet: asked again
        # until the wait ends, then the real answer counts
        self.patch(gate, "speaker_card", lambda sysfs=None: "AppleX")
        answers = [None, None, True]
        self.patch(gate, "has_control", lambda card, name: answers.pop(0) if answers else True)
        self.patch(gate.time, "sleep", lambda s: None)
        self.assertEqual(gate.kernel_interface(10), ("dual", "AppleX"))
        self.assertEqual(answers, [])
        answers[:] = [False, None, False, False]
        self.assertEqual(gate.kernel_interface(10), ("legacy", "AppleX"))

    def test_kernel_interface_reads_the_selector(self):
        self.patch(gate, "speaker_card", lambda sysfs=None: "AppleX")
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
        self.patch(gate, "kernel_interface", lambda wait, **kw: (state["iface"], "AppleX"))
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
        self.ctl({"board": "j999", "unit": "lsm2", "aid": "AID44"})
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
            # The speaker tables are read by the speaker identity the record's
            # wiring carries (the bundle's AID): a 4-lane AID13 bound on this
            # 2-amp Mac refuses its voicing, and a record without one cannot
            # generate the voicing at all.
            record = os.path.join(self.p.state, "selfcheck.json")
            with open(record) as f:
                saved = f.read()
            with open(self.p.report) as f:
                prov_saved = f.read()
            for aid in ("AID13", None):
                report = json.loads(saved)
                report["wiring"]["aid"] = aid
                with open(record, "w") as f:
                    json.dump(report, f)
                # the provisioned bundle names the same identity, so the
                # speaker table itself decides
                with open(self.p.report, "w") as f:
                    json.dump(dict(json.loads(prov_saved), aid=aid), f)
                with self.assertRaises(ValueError, msg=aid):
                    sc.cmd_generate(args)
            with open(self.p.report, "w") as f:
                f.write(prov_saved)
            # The record's identity must be the provisioned bundle's: one
            # that differs (a record and a bundle that drifted apart) refuses
            # generation, before any speaker table is read.
            report = json.loads(saved)
            report["wiring"]["aid"] = "AID9"
            with open(record, "w") as f:
                json.dump(report, f)
            with self.assertRaises(SystemExit) as e:
                sc.cmd_generate(args)
            self.assertIn("provisioned bundle 'AID44'", str(e.exception))
            with open(record, "w") as f:
                f.write(saved)
            self.assertEqual(sc.cmd_generate(args), 0)
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

    def test_lsid_qualifies_only_when_the_compiler_releases_it(self):
        policy = dict(GATE, units_at_parity=["spp3", "atsp", "lsm2", "lsid"])
        gated = "warning[lsm2-release-gated] compile: unqualified companion/model/flow laws: output MUTED"
        released = "warning[lsid-static-release] compile: the LSID estimator is not run"
        for board, warning, ok in (("k116", gated, False), ("j713", released, True)):
            with self.subTest(board=board):
                self.board(board)
                self.ctl({"board": board, "unit": "lsm2", "lsid": True, "warnings": [warning]})
                self.assertEqual(self.run_(), 0 if ok else 1)
                if ok:
                    self.checked()
                self.assertEqual(self.decide(gate_data=policy), (ok, False))

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
        self.assertEqual(g["units_at_parity"], ["spp3", "lsm2", "atsp", "lsid"])
        self.assertNotIn("qualified", g)
        self.assertNotIn("require_qualified_board", g)
        self.assertEqual(g["denied_boards"], [])
        self.assertEqual(g["board_overrides"], {})
        self.assertEqual(g["voicing_pre"], {"j316s": True, "j413": True, "j616s": True, "j700": True})
        self.assertFalse(os.path.exists(os.path.join(HERE, "machines.json")))


JACK = b"apple,j616-macaudio-jack\0apple,macaudio\0"


class SpeakerRouting(Base):
    def setUp(self):
        super().setUp()
        self.route = "Primary"
        self.writes = []
        self.reads = 0
        self.write_error = False
        self.ignore_write = False
        run = prov.subprocess.run

        def amixer(argv, **kw):
            if argv[0] != "amixer":
                return run(argv, **kw)
            self.assertEqual(argv[:3], ["amixer", "-c", "AppleJ316"])
            self.assertEqual(argv[4], "name=Speaker Playback Mux")
            if argv[3] == "cset":
                self.assertEqual(argv[5:], ["Secondary"])
                self.writes.append(argv[5])
                if self.write_error:
                    return types.SimpleNamespace(returncode=1, stdout="", stderr="Device or resource busy")
                if not self.ignore_write:
                    self.route = "Secondary"
            else:
                self.assertEqual(argv[3], "cget")
                self.reads += 1
            value = 1 if self.route == "Secondary" else 0
            return types.SimpleNamespace(returncode=0, stderr="", stdout=(
                "; type=ENUMERATED,access=rw------,values=1,items=2\n"
                "; Item #0 'Primary'\n; Item #1 'Secondary'\n"
                f": values={value}\n"))

        self.patch(prov.subprocess, "run", amixer)

    def live(self):
        # Keep every path under the test root; exercise the hardware branch
        # with only the mixer boundary replaced.
        self.p.root = ""
        return self.run_()

    def test_fresh_card_routes_pcm1_before_the_graph_can_start(self):
        self.assertEqual(self.live(), 0)
        self.assertEqual(self.route, "Secondary")
        self.assertEqual(self.writes, ["Secondary"])
        self.assertEqual(self.reads, 2)

    def test_existing_secondary_route_is_read_without_a_write(self):
        self.route = "Secondary"
        self.assertEqual(self.live(), 0)
        self.assertEqual(self.writes, [])
        self.assertEqual(self.reads, 1)

    def test_busy_route_fails_closed(self):
        self.write_error = True
        self.assertEqual(self.live(), 1)
        self.assertFalse(os.path.exists(self.p.conf))
        self.assertFalse(os.path.exists(self.p.params))
        self.assertEqual(self.record()["result"], "fail")

    def test_unchanged_readback_fails_closed(self):
        self.ignore_write = True
        self.assertEqual(self.live(), 1)
        self.assertFalse(os.path.exists(self.p.conf))
        self.assertFalse(os.path.exists(self.p.params))

    def test_test_root_never_touches_the_live_mixer(self):
        self.assertEqual(self.run_(), 0)
        self.assertEqual((self.reads, self.writes), (0, []))


def gate_proc(case):
    """A private /proc/asound for the gate's card lookup in `case`."""
    proc = tempfile.mkdtemp()
    case.addCleanup(shutil.rmtree, proc, True)
    orig = gate.PROC
    gate.PROC = proc
    case.addCleanup(setattr, gate, "PROC", orig)
    return proc


def alsa_card(sysfs, n, ident, compat, link, registered=True):
    """Card n in a test /sys/class/sound (its device tree node behind
    device/of_node), registered in full by ALSA (controlCN, the proc id in
    gate.PROC) unless `registered` is False: then it is mid-registration,
    with only its device, id and of_node link."""
    node = os.path.join(sysfs, f"card{n}", "device", "of_node")
    os.makedirs(os.path.join(node, "dai-link@0"))
    with open(os.path.join(sysfs, f"card{n}", "id"), "w") as f:
        f.write(ident + "\n")
    with open(os.path.join(node, "compatible"), "wb") as f:
        f.write(compat)
    with open(os.path.join(node, "dai-link@0", "link-name"), "wb") as f:
        f.write(link + b"\0")
    if registered:
        register(sysfs, n, ident)


def register(sysfs, n, ident):
    """ALSA finishes registering card n: controlCN, then its proc entries."""
    open(os.path.join(sysfs, f"card{n}", f"controlC{n}"), "w").close()
    os.makedirs(os.path.join(gate.PROC, f"card{n}"), exist_ok=True)
    with open(os.path.join(gate.PROC, f"card{n}", "id"), "w") as f:
        f.write(ident + "\n")


class SpeakerCard(unittest.TestCase):
    """The gate's speaker card is the macaudio card with the speaker link,
    whichever card number the jack-only card took."""

    def setUp(self):
        gate_proc(self)

    def card(self, sysfs, n, ident, compat, link, registered=True):
        alsa_card(sysfs, n, ident, compat, link, registered)

    def test_the_jack_card_first_is_skipped(self):
        with tempfile.TemporaryDirectory() as sysfs:
            self.card(sysfs, 0, "AppleJ616HPAI", b"apple,aop-audio\0", b"Speakers")
            self.card(sysfs, 1, "AppleJack", JACK, b"Headphone Jack")
            self.card(sysfs, 2, "AppleJ616", b"apple,j616-macaudio\0apple,macaudio\0", b"Speakers")
            self.assertEqual(gate.speaker_card(sysfs), "AppleJ616")

    def test_the_provisioner_skips_the_jack_card(self):
        with tempfile.TemporaryDirectory() as d:
            sysfs, proc, dt = os.path.join(d, "sys"), os.path.join(d, "proc"), os.path.join(d, "dt")
            for n, node, ident in ((1, "sound-jack", "AppleJack"), (2, "sound", "AppleJ616")):
                os.makedirs(os.path.join(dt, node))
                os.makedirs(os.path.join(sysfs, f"card{n}", "device"))
                os.symlink(os.path.join(dt, node), os.path.join(sysfs, f"card{n}", "device", "of_node"))
                open(os.path.join(sysfs, f"card{n}", f"controlC{n}"), "w").close()
                os.makedirs(os.path.join(proc, f"card{n}"))
                with open(os.path.join(proc, f"card{n}", "id"), "w") as f:
                    f.write(ident + "\n")
            hw = {"sound": "sound"}
            self.assertEqual(prov.find_card(hw, "AppleJ616", 0, dt, sysfs, proc), (2, "AppleJ616"))
            # another model-derived name is logged, not refused: the id is the card's own
            self.assertEqual(prov.find_card(hw, "AppleJ614", 0, dt, sysfs, proc), (2, "AppleJ616"))
            with self.assertRaises(prov.Failed):
                prov.find_card({"sound": "sound-missing"}, "AppleJ616", 0, dt, sysfs, proc)
            # a card still registering (of_node link, no id yet) is waited
            # for, not refused: the wait runs out with "no card bound"
            os.unlink(os.path.join(proc, "card2", "id"))
            with self.assertRaises(prov.Failed) as e:
                prov.find_card(hw, "AppleJ616", 0, dt, sysfs, proc)
            self.assertIn("no ALSA card bound", str(e.exception))
            # the same before its control device: the bounded wait covers it
            with open(os.path.join(proc, "card2", "id"), "w") as f:
                f.write("AppleJ616\n")
            os.unlink(os.path.join(sysfs, "card2", "controlC2"))
            with self.assertRaises(prov.Failed) as e:
                prov.find_card(hw, "AppleJ616", 0, dt, sysfs, proc)
            self.assertIn("no ALSA card bound", str(e.exception))
            import threading
            t = threading.Timer(0.6, lambda: open(os.path.join(sysfs, "card2", "controlC2"), "w").close())
            t.start()
            self.addCleanup(t.cancel)
            self.assertEqual(prov.find_card(hw, "AppleJ616", 10, dt, sysfs, proc), (2, "AppleJ616"))

    def test_no_speaker_link_is_no_speaker_card(self):
        with tempfile.TemporaryDirectory() as sysfs:
            self.card(sysfs, 1, "AppleJack", JACK, b"Headphone Jack")
            self.assertIsNone(gate.speaker_card(sysfs))


class CardOrder(unittest.TestCase):
    """The gate reads the interface from the speaker card whatever order the
    cards register in: the jack-only card first, the speaker card first, or
    the speaker card seconds after the jack card (7.3.0-33 on a J616s: the
    jack card was card1, the speaker card card2 2 s later)."""

    SPEAKER = b"apple,j616-macaudio\0apple,macaudio\0"

    def setUp(self):
        self.sysfs = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.sysfs, True)
        gate_proc(self)
        self.asked = []
        # the jack card has no speaker controls; the speaker card has both
        controls = {"AppleJ616": {"Speaker Protection Interface", "Speaker Protection"}}
        orig = gate.has_control
        gate.has_control = lambda card, name: self.asked.append(card) or name in controls.get(card, set())
        self.addCleanup(setattr, gate, "has_control", orig)
        orig_log = gate.log
        gate.log = lambda *a: None
        self.addCleanup(setattr, gate, "log", orig_log)

    def card(self, n, ident, compat, link, registered=True):
        alsa_card(self.sysfs, n, ident, compat, link, registered)

    def jack(self, n):
        self.card(n, "AppleJack", JACK, b"Headphone Jack")

    def speaker(self, n, registered=True):
        self.card(n, "AppleJ616", self.SPEAKER, b"Speakers", registered)

    def test_jack_first(self):
        self.card(0, "AppleJ616HPAI", b"apple,aop-audio\0", b"Speakers")
        self.jack(1)
        self.speaker(2)
        self.assertEqual(gate.kernel_interface(0, self.sysfs), ("dual", "AppleJ616"))
        self.assertEqual(set(self.asked), {"AppleJ616"})

    def test_speaker_first(self):
        self.speaker(1)
        self.jack(2)
        self.assertEqual(gate.kernel_interface(0, self.sysfs), ("dual", "AppleJ616"))
        self.assertEqual(set(self.asked), {"AppleJ616"})

    def test_speaker_late(self):
        import threading
        self.jack(1)
        t = threading.Timer(0.6, self.speaker, (2,))
        t.start()
        self.addCleanup(t.cancel)
        t0 = time.monotonic()
        self.assertEqual(gate.kernel_interface(10, self.sysfs), ("dual", "AppleJ616"))
        self.assertGreaterEqual(time.monotonic() - t0, 0.5)
        self.assertEqual(set(self.asked), {"AppleJ616"})

    def test_a_card_still_registering_is_waited_for(self):
        # the Neo race (speakerguardd#235): the card's directory, id and
        # of_node link exist before ALSA registered its control device and
        # proc entries. amixer cannot open it then; the gate used to take
        # that for "no Speaker Protection control" = a definite legacy
        # answer. A card counts only once it is registered in full.
        def amixer(card, name):
            self.asked.append(card)
            if not os.path.exists(os.path.join(self.sysfs, "card1", "controlC1")):
                return False  # the old reading of a failed open
            return name in {"Speaker Protection Interface", "Speaker Protection"}
        gate.has_control = amixer
        self.speaker(1, registered=False)
        self.assertEqual(gate.kernel_interface(0, self.sysfs), (None, None))
        self.assertEqual(self.asked, [])
        # controlC1 is there, the proc entries are not yet: still registering
        open(os.path.join(self.sysfs, "card1", "controlC1"), "w").close()
        self.assertEqual(gate.kernel_interface(0, self.sysfs), (None, None))
        self.assertEqual(self.asked, [])
        os.unlink(os.path.join(self.sysfs, "card1", "controlC1"))
        import threading
        t = threading.Timer(0.6, register, (self.sysfs, 1, "AppleJ616"))
        t.start()
        self.addCleanup(t.cancel)
        t0 = time.monotonic()
        self.assertEqual(gate.kernel_interface(10, self.sysfs), ("dual", "AppleJ616"))
        self.assertGreaterEqual(time.monotonic() - t0, 0.5)

    def test_without_alsa_proc_the_control_device_decides(self):
        # a kernel without ALSA's proc interface: controlCN alone
        gate.PROC = os.path.join(self.sysfs, "no-proc")
        self.speaker(1, registered=False)
        self.assertEqual(gate.kernel_interface(0, self.sysfs), (None, None))
        open(os.path.join(self.sysfs, "card1", "controlC1"), "w").close()
        self.assertEqual(gate.kernel_interface(0, self.sysfs), ("dual", "AppleJ616"))

    def test_jack_only_is_no_speaker_card(self):
        # the bounded wait ends without a speaker card: the caller keeps
        # today's answer (the older daemons decide), and the jack card's
        # missing controls never count as a kernel without the v2 interface
        self.jack(1)
        self.assertEqual(gate.kernel_interface(0.3, self.sysfs), (None, None))
        self.assertEqual(self.asked, [])

    def test_legacy_kernel_answer_comes_from_the_speaker_card(self):
        self.jack(1)
        self.card(2, "AppleJ413", b"apple,j413-macaudio\0apple,macaudio\0", b"Speaker")
        self.assertEqual(gate.kernel_interface(0, self.sysfs), ("legacy", "AppleJ413"))
        self.assertEqual(set(self.asked), {"AppleJ413"})

    def dt(self, nodes):
        """A live device tree with the given root nodes:
        {name: (compatible, link names, status or None)}."""
        root = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, root, True)
        with open(os.path.join(root, "compatible"), "wb") as f:
            f.write(b"apple,j999\0apple,arm-platform\0")
        for name, (compat, links, status) in nodes.items():
            n = os.path.join(root, name)
            os.makedirs(n)
            with open(os.path.join(n, "compatible"), "wb") as f:
                f.write(compat)
            if status is not None:
                with open(os.path.join(n, "status"), "wb") as f:
                    f.write(status + b"\0")
            for i, link in enumerate(links):
                os.makedirs(os.path.join(n, f"dai-link@{i}"))
                with open(os.path.join(n, f"dai-link@{i}", "link-name"), "wb") as f:
                    f.write(link + b"\0")
        return root

    def timed(self, wait, dt):
        t0 = time.monotonic()
        r = gate.kernel_interface(wait, self.sysfs, dt=dt)
        return r, time.monotonic() - t0

    def test_no_sound_node_answers_at_once(self):
        # speakerguardd#183: a tree that describes no speaker card cannot
        # get one, so the gate does not wait out --wait for it
        for nodes in ({},
                      {"sound": (JACK, [b"Headphone Jack"], None)},
                      {"sound": (self.SPEAKER, [b"Speakers"], b"disabled")},
                      {"i2c@0": (b"apple,i2c\0", [], None)}):
            r, took = self.timed(10, self.dt(nodes))
            self.assertEqual(r, (None, None), nodes)
            self.assertLess(took, 1.0, nodes)
        self.assertEqual(self.asked, [])

    def test_a_described_speaker_card_is_still_waited_for(self):
        import threading
        dt = self.dt({"sound": (self.SPEAKER, [b"Speakers"], b"okay"),
                      "sound-jack": (JACK, [b"Headphone Jack"], None)})
        self.assertIs(gate.dt_speaker_node(dt), True)
        self.jack(1)
        t = threading.Timer(0.6, self.speaker, (2,))
        t.start()
        self.addCleanup(t.cancel)
        r, took = self.timed(10, dt)
        self.assertEqual(r, ("dual", "AppleJ616"))
        self.assertGreaterEqual(took, 0.5)
        # and with no card at all it waits the full bound, as before
        shutil.rmtree(self.sysfs)
        os.makedirs(self.sysfs)
        r, took = self.timed(0.6, dt)
        self.assertEqual(r, (None, None))
        self.assertGreaterEqual(took, 0.6)

    def test_an_unreadable_tree_keeps_the_wait(self):
        missing = os.path.join(self.sysfs, "no-device-tree")
        self.assertIsNone(gate.dt_speaker_node(missing))
        r, took = self.timed(0.6, missing)
        self.assertEqual(r, (None, None))
        self.assertGreaterEqual(took, 0.6)

    def test_the_card_is_read_before_the_tree(self):
        # a registered speaker card answers whatever the tree scan says
        self.speaker(1)
        self.assertEqual(gate.kernel_interface(0, self.sysfs, dt=self.dt({})), ("dual", "AppleJ616"))


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
