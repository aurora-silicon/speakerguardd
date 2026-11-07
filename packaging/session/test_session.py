#!/usr/bin/env python3
"""Tests of speakerguardd-session's link and carrier checks on synthetic graphs.

    python3 -I packaging/session/test_session.py
"""
import importlib.machinery
import importlib.util
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
_loader = importlib.machinery.SourceFileLoader("sgs", os.path.join(HERE, "speakerguardd-session"))
_spec = importlib.util.spec_from_loader("sgs", _loader)
sgs = importlib.util.module_from_spec(_spec)
_loader.exec_module(sgs)

sys.path.insert(0, HERE)
import gen  # noqa: E402

# Every machine with a fixture, as the derivation describes it.
MACHINES = {m: gen.machine(m)[0] for m in gen.machines()}


class FakeGraph(sgs.Graph):
    """A Graph without pw-dump: objects are added by the test."""

    def __init__(self):
        self.objs = {}
        self.next = 100

    def node(self, name, props=None):
        nid = self.next
        self.next += 1
        info = {"props": {"node.name": name}}
        if props is not None:
            info["params"] = {"Props": [props]}
        self.objs[nid] = {"id": nid, "type": "PipeWire:Interface:Node", "info": info}
        return nid

    def port(self, nid, direction, channel):
        pid = self.next
        self.next += 1
        self.objs[pid] = {
            "id": pid,
            "type": "PipeWire:Interface:Port",
            "info": {"props": {"node.id": nid, "port.direction": direction, "audio.channel": channel}},
        }
        return pid

    def link(self, out_p, in_p):
        lid = self.next
        self.next += 1
        self.objs[lid] = {
            "id": lid,
            "type": "PipeWire:Interface:Link",
            "info": {"output-port-id": out_p, "input-port-id": in_p},
        }
        return lid


UNITY = {"volume": 1.0, "mute": False, "channelVolumes": [1.0] * 18}


def build(m="j316", carrier=UNITY, linked=True):
    c = MACHINES[m]
    amps, sense = c["amps"], c["sense"]
    g = FakeGraph()
    voicing = g.node(f"effect_output.{m}-convolver")
    protect = g.node(sgs.PROTECT_INPUT, carrier)
    pout = g.node(f"sgv2.protect.{m}.out")
    spk = g.node(f"sgv2.speaker.{m}")
    sn = g.node(f"sgv2.sense.{m}", UNITY) if sense else None
    p = {
        "v": [g.port(voicing, "out", f"AUX{i}") for i in range(amps)],
        "pin": [g.port(protect, "in", f"AUX{i}") for i in range(amps + sense)],
        "pout": [g.port(pout, "out", f"AUX{i}") for i in range(amps)],
        "spk": [g.port(spk, "in", f"AUX{i}") for i in range(amps)],
        "sense": [g.port(sn, "out", f"AUX{i}") for i in range(sense)] if sense else [],
    }
    for i in range(amps):
        g.link(p["v"][i], p["pin"][i])
    if linked:
        for i in range(amps):
            g.link(p["pout"][i], p["spk"][i])
        for i in range(sense):
            g.link(p["sense"][i], p["pin"][amps + i])
    return g, p, sgs.Session(m, c)


class Checks(unittest.TestCase):
    def test_the_configured_graph_is_clean_on_every_machine(self):
        for m in MACHINES:
            g, _, s = build(m)
            self.assertEqual(s.check(g), (True, True), m)

    def test_missing_links_are_silence_not_a_fault(self):
        g, _, s = build(linked=False)
        self.assertEqual(s.check(g), (True, True))

    def test_a_foreign_link_into_the_amps_is_a_fault(self):
        g, p, s = build()
        g.link(p["v"][0], p["spk"][0])
        self.assertFalse(s.check(g)[0])

    def test_crossed_amp_channels_are_a_fault(self):
        g, p, s = build(linked=False)
        g.link(p["pout"][0], p["spk"][1])
        self.assertFalse(s.check(g)[0])

    def test_crossed_or_foreign_sense_is_a_fault(self):
        g, p, s = build(linked=False)
        g.link(p["sense"][0], p["pin"][7])
        self.assertFalse(s.check(g)[0])
        g, p, s = build(linked=False)
        g.link(p["v"][0], p["pin"][6])
        self.assertFalse(s.check(g)[0])

    def test_the_sense_feeding_anything_else_is_a_fault(self):
        g, p, s = build()
        other = g.node("some.recorder")
        g.link(p["sense"][0], g.port(other, "in", "AUX0"))
        self.assertFalse(s.check(g)[0])

    def test_a_second_voicing_into_the_program_is_protected_and_allowed(self):
        g, p, s = build()
        ab = g.node("effect_output.some-ab-sink")
        g.link(g.port(ab, "out", "AUX0"), p["pin"][0])
        self.assertTrue(s.check(g)[0])

    def test_a_crossed_program_link_or_the_sense_on_the_program_is_a_fault(self):
        g, p, s = build()
        ab = g.node("effect_output.some-ab-sink")
        g.link(g.port(ab, "out", "AUX1"), p["pin"][0])
        self.assertFalse(s.check(g)[0])
        g, p, s = build()
        g.link(p["sense"][0], p["pin"][0])
        self.assertFalse(s.check(g)[0])

    def test_the_carrier_must_be_unity_and_unmuted(self):
        for bad in (
            dict(UNITY, mute=True),
            dict(UNITY, volume=0.5),
            dict(UNITY, channelVolumes=[1.0] * 17 + [0.9]),
            {},
        ):
            g, _, s = build(carrier=bad)
            self.assertEqual(s.check(g), (True, False), bad)

    def test_a_missing_stage_fails_closed(self):
        g, _, s = build()
        del g.objs[g.node_id(f"sgv2.speaker.j316")]
        self.assertEqual(s.check(g), (False, False))

    def test_wanted_links_are_channel_for_channel(self):
        g, p, s = build(linked=False)
        want = s.wanted(g)
        amps = MACHINES["j316"]["amps"]
        self.assertEqual(want[:amps], list(zip(p["pout"], p["spk"])))
        self.assertEqual(want[amps:], list(zip(p["sense"], p["pin"][amps:])))

    def test_the_neo_has_no_sense_links(self):
        g, p, s = build("j700", linked=False)
        self.assertEqual(s.wanted(g), list(zip(p["pout"], p["spk"])))


class Stop(unittest.TestCase):
    def test_stopping_unlinks_the_amps_and_the_sense_within_its_bound(self):
        import time

        g, p, s = build("j616")
        c = MACHINES["j616"]
        ran = []

        class Slow:
            def __init__(self, argv):
                ran.append(argv)
                self.t = time.monotonic()
                self.killed = False

            def poll(self):
                return -9 if self.killed else None

            def wait(self, timeout):
                if timeout < 5:
                    raise sgs.subprocess.TimeoutExpired("pw-link", timeout)

            def kill(self):
                self.killed = True

        orig = sgs.subprocess.Popen
        sgs.subprocess.Popen = lambda argv, **kw: Slow(argv)
        try:
            t0 = time.monotonic()
            s.unlink_all(g, 0.2)
            took = time.monotonic() - t0
        finally:
            sgs.subprocess.Popen = orig
        # every amp link and every sense link, nothing else (the voicing stays)
        self.assertEqual(len(ran), c["amps"] + c["sense"], ran)
        self.assertTrue(all(a[:2] == ["pw-link", "-d"] for a in ran), ran)
        # bounded: a pw-link that does not finish is killed, not waited for
        self.assertLess(took, 1.0)
        self.assertEqual(s.pending, {})


class Generated(unittest.TestCase):
    def test_voicing_gate_is_per_physical_board_and_default_off(self):
        self.assertFalse(gen.voicing_enabled("j616s", {}))
        self.assertTrue(gen.voicing_enabled("j616s", {"voicing_pre": {"j616s": True}}))
        self.assertFalse(gen.voicing_enabled("j616c", {"voicing_pre": {"j616s": True}}))
        for value in (False, "true", 1, None):
            self.assertFalse(gen.voicing_enabled("j616s", {"voicing_pre": {"j616s": value}}))
        # Shipped gate (Ryan 2026-11-07, Plan A tonight): the four qualified test
        # boards together, every other board keeps the interim graph.
        on = sorted(gen.fixture(m)["hw"]["board"] for m in gen.machines() if gen.voicing_enabled(gen.fixture(m)["hw"]["board"]))
        self.assertEqual(on, ["j316s", "j413", "j616s", "j700"])

    def test_voicing_bound_is_validated_and_never_positive(self):
        self.assertIsNone(gen.voicing_bound("j616s", {}))
        self.assertEqual(gen.voicing_bound("j616s", {"voicing_pre_bound_db": {"j616s": [0, -1.5]}}), [0.0, -1.5])
        for bad in ([0.1], ["-1"], [True], -1.0, [-13.0], [float("nan")]):
            with self.assertRaises(ValueError):
                gen.voicing_bound("j616s", {"voicing_pre_bound_db": {"j616s": bad}})
        for m in gen.machines():
            c, w = gen.machine(m)
            b = gen.voicing_bound(c.get("board", m))
            if gen.voicing_enabled(c.get("board", m)) and b is not None:
                self.assertEqual(len(b), c["amps"])

    def test_both_modes_target_the_same_protect_input_and_only_generated_mode_owns_volume(self):
        for m in gen.machines():
            c, w = gen.machine(m)
            interim = gen.pipewire_conf(m, c, w, voicing_pre=False)
            generated = gen.pipewire_conf(m, c, w, voicing_pre=True)
            self.assertNotIn('sgv2.stage = "voicing-pre"', interim)
            self.assertEqual(generated.count("context.modules = ["), 1)
            self.assertIn(f'target.object = "{gen.PROTECT_INPUT}"', generated)
            self.assertIn('audio.position = [ FL FR ]', generated)
            self.assertIn('capture.volumes = [ { control = "voicing:volume"', generated)
            self.assertIn('scale = cubic', generated)
            b = gen.voicing_bound(c.get("board", m)) or []
            ports = " ".join(f'"b{lane}:Out"' if lane < len(b) and b[lane] < 0 else f'"voicing:out_{lane}"' for lane in w[0])
            self.assertIn(f'outputs = [ {ports} ]', generated)
            wp_old = gen.wireplumber_conf(m, c, voicing_pre=False)
            wp_new = gen.wireplumber_conf(m, c, voicing_pre=True)
            self.assertIn(f'/usr/share/asahi-audio/{m}/graph.json', wp_old)
            self.assertNotIn("create-filter", wp_new)
            self.assertNotIn("graph.json", wp_new)
            self.assertIn("node.disabled = true", wp_new)


    def test_generated_files_are_current(self):
        import subprocess
        import sys

        r = subprocess.run([sys.executable, "-I", os.path.join(HERE, "gen.py"), "--check"], capture_output=True)
        self.assertEqual(r.returncode, 0, r.stdout.decode())


class Report(unittest.TestCase):
    def setUp(self):
        if not os.path.exists("/proc/self/stat"):
            # not Linux: name a writer without /proc
            sgs._WRITER = (os.getpid(), 4242)

    def test_each_report_names_its_live_writer(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "session")
            self.assertTrue(sgs.write_report(path, True, True))
            fields = dict(w.split("=") for w in open(path).read().split())
            self.assertEqual(int(fields["pid"]), os.getpid())
            if os.path.exists("/proc/self/stat"):
                self.assertEqual(int(fields["start"]), sgs.own_start())
            self.assertEqual(sorted(fields), ["carrier", "links", "mono_ns", "pid", "start"])

    def test_the_report_is_renamed_into_place_whole_and_group_readable(self):
        import stat
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "session")
            # a reader holding the old report keeps it whole
            self.assertTrue(sgs.write_report(path, True, True))
            old = open(path, "rb")
            self.assertTrue(sgs.write_report(path, False, True))
            first = old.read().decode()
            self.assertTrue(first.startswith("links=1 carrier=1 mono_ns=") and first.endswith("\n"))
            now = open(path).read()
            self.assertTrue(now.startswith("links=0 carrier=1 mono_ns=") and now.endswith("\n"), now)
            self.assertEqual(stat.S_IMODE(os.stat(path).st_mode), 0o640)
            self.assertEqual(sorted(os.listdir(d)), ["session"])
            # the stamp is CLOCK_MONOTONIC, the supervisor's clock
            import time

            t = int(now.split("mono_ns=")[1].split()[0])
            self.assertLessEqual(abs(time.monotonic_ns() - t), 1_000_000_000)

    def test_an_unwritable_directory_reports_failure(self):
        self.assertFalse(sgs.write_report("/nonexistent-dir/session", True, True))


if __name__ == "__main__":
    unittest.main()
