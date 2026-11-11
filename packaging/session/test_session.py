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
        self._memo = {}

    @property
    def gen(self):
        # tests edit objs directly: every read is a new generation
        self._g = getattr(self, "_g", 0) + 1
        return self._g

    def node(self, name, props=None, state=None, group=None):
        nid = self.next
        self.next += 1
        info = {"props": {"node.name": name}}
        if group is not None:
            info["props"]["node.link-group"] = group
        if state is not None:
            info["state"] = state
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


class AmpsUnity(unittest.TestCase):
    def _run(self, props):
        g, _, s = build("j316")
        spk = g.node_id("sgv2.speaker.j316")
        g.objs[spk]["info"].setdefault("params", {})["Props"] = [props] if props is not None else []
        ran = []

        class P:
            def __init__(self, argv):
                ran.append(argv)

            def poll(self):
                return 0

        orig = sgs.subprocess.Popen
        sgs.subprocess.Popen = lambda argv, **kw: P(argv)
        try:
            started = s.pin_amps_unity(g)
            again = s.pin_amps_unity(g)
        finally:
            sgs.subprocess.Popen = orig
        return started, again, ran, spk

    def test_a_volume_on_the_amps_sink_is_put_back_to_unity(self):
        started, again, ran, spk = self._run({"volume": 1.0, "mute": False, "channelVolumes": [0.027] * 6})
        self.assertTrue(started)
        self.assertFalse(again)  # at most once a second
        self.assertEqual(len(ran), 1)
        self.assertEqual(ran[0][:4], ["pw-cli", "set-param", str(spk), "Props"])
        self.assertIn("channelVolumes: [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ]", ran[0][4])
        self.assertIn("mute: false", ran[0][4])

    def test_unity_or_unknown_amps_volume_is_left_alone(self):
        for props in ({"volume": 1.0, "mute": False, "channelVolumes": [1.0] * 6}, {}, None):
            started, _, ran, _ = self._run(props)
            self.assertFalse(started, props)
            self.assertEqual(ran, [], props)


class VoicingVolume(unittest.TestCase):
    """The voicing sink's volume: restore, first start, store, control re-sync."""

    def setUp(self):
        import tempfile

        self.tmp = tempfile.TemporaryDirectory()
        self.old_env = os.environ.get("XDG_STATE_HOME")
        os.environ["XDG_STATE_HOME"] = self.tmp.name
        self.ran = []
        ran = self.ran

        class P:
            def __init__(self, argv):
                ran.append(argv)

            def poll(self):
                return 0

        self.orig = sgs.subprocess.Popen
        sgs.subprocess.Popen = lambda argv, **kw: P(argv)

    def tearDown(self):
        sgs.subprocess.Popen = self.orig
        if self.old_env is None:
            os.environ.pop("XDG_STATE_HOME", None)
        else:
            os.environ["XDG_STATE_HOME"] = self.old_env
        self.tmp.cleanup()

    def sink(self, g, vols, mute=False, control=None):
        nid = g.node_id("sgv2.voicing.j413")
        if nid is None:
            nid = g.node("sgv2.voicing.j413")
        self.set(g, nid, vols, mute, control)
        return nid

    @staticmethod
    def set(g, nid, vols, mute=False, control=None):
        props = [{"volume": 1.0, "mute": mute, "channelVolumes": list(vols), "softVolumes": [1.0] * len(vols)}]
        if control is not None:
            props.append({"params": ["voicing:volume", control, "voicing:volume_db", 1.0]})
        g.objs[nid]["info"]["params"] = {"Props": props}

    def session(self):
        g, _, _ = build("j413")
        return g, sgs.Session("j413", MACHINES["j413"])

    def test_first_start_with_nothing_stored_is_30_percent_not_100(self):
        g, s = self.session()
        nid = self.sink(g, [1.0, 1.0], control=1.0)
        self.assertEqual(s.keep_voicing_volume(g, 0.0), "restore")
        self.assertEqual(self.ran[-1][:4], ["pw-cli", "set-param", str(nid), "Props"])
        self.assertIn("channelVolumes: [ 0.027, 0.027 ]", self.ran[-1][4])
        self.assertIn("mute: false", self.ran[-1][4])
        # not stored until the sink has taken it
        self.assertFalse(os.path.exists(s.volume_file))
        self.set(g, nid, [0.027, 0.027], control=0.3)
        self.assertIsNone(s.keep_voicing_volume(g, 0.1))
        with open(s.volume_file) as f:
            self.assertEqual(sgs.json.load(f), {"channelVolumes": [0.027, 0.027], "mute": False})

    def test_a_stored_volume_is_restored_on_every_new_sink(self):
        g, s = self.session()
        os.makedirs(os.path.dirname(s.volume_file))
        with open(s.volume_file, "w") as f:
            sgs.json.dump({"channelVolumes": [0.008, 0.008], "mute": True}, f)
        nid = self.sink(g, [1.0, 1.0], control=1.0)
        self.assertEqual(s.keep_voicing_volume(g, 0.0), "restore")
        self.assertIn("mute: true, channelVolumes: [ 0.008, 0.008 ]", self.ran[-1][4])
        # retried if the sink has not taken it, not before the retry time
        self.assertIsNone(s.keep_voicing_volume(g, 1.0))
        self.assertEqual(s.keep_voicing_volume(g, 2.5), "restore")
        # a new sink (graph restart) is restored again
        del g.objs[nid]
        g.next += 1
        nid2 = self.sink(g, [1.0, 1.0], control=1.0)
        self.assertNotEqual(nid, nid2)
        self.assertEqual(s.keep_voicing_volume(g, 3.0), "restore")
        self.assertEqual(self.ran[-1][2], str(nid2))

    def test_a_bad_record_is_ignored(self):
        g, s = self.session()
        os.makedirs(os.path.dirname(s.volume_file))
        for rec in ({"channelVolumes": [2.0, 2.0]}, {"channelVolumes": [0.1]}, {"channelVolumes": [0.1, 0.1], "mute": 1}, []):
            with open(s.volume_file, "w") as f:
                sgs.json.dump(rec, f)
            self.assertIsNone(s.load_volume(2), rec)

    def test_user_changes_are_stored(self):
        g, s = self.session()
        nid = self.sink(g, [1.0, 1.0], control=1.0)
        s.keep_voicing_volume(g, 0.0)
        self.set(g, nid, [0.027, 0.027], control=0.3)
        s.keep_voicing_volume(g, 0.1)
        self.set(g, nid, [0.125, 0.125], mute=True, control=0.0)
        self.assertIsNone(s.keep_voicing_volume(g, 0.2))
        with open(s.volume_file) as f:
            self.assertEqual(sgs.json.load(f), {"channelVolumes": [0.125, 0.125], "mute": True})

    def test_a_control_apart_from_the_volume_gets_a_real_change(self):
        g, s = self.session()
        nid = self.sink(g, [0.008, 0.008], control=0.2)
        s.vs_id, s.vs_settled = nid, True
        self.assertIsNone(s.keep_voicing_volume(g, 0.0))  # 0.008 ** (1/3) = 0.2: in step
        # the incident: control at its default under a lower volume
        self.set(g, nid, [0.008, 0.008], control=1.0)
        self.assertEqual(s.keep_voicing_volume(g, 1.0), "resync")
        argv = self.ran[-1]
        self.assertEqual(argv[:2], ["sh", "-c"])
        self.assertEqual(argv[3], str(nid))
        self.assertIn("0.0079992", argv[4])  # the nudge goes down, never up
        self.assertIn("channelVolumes: [ 0.008, 0.008 ]", argv[5])
        # at most once a second
        self.assertIsNone(s.keep_voicing_volume(g, 1.5))
        # muted, or volume 0: the control must read 0
        self.set(g, nid, [0.3, 0.3], mute=True, control=0.0)
        self.assertIsNone(s.keep_voicing_volume(g, 3.0))
        self.set(g, nid, [0.0, 0.0], control=1.0)
        self.assertEqual(s.keep_voicing_volume(g, 5.0), "resync")
        self.assertIn("1e-09", self.ran[-1][4])

    def test_no_generated_sink_or_no_volume_is_left_alone(self):
        g, s = self.session()
        self.assertIsNone(s.keep_voicing_volume(g, 0.0))
        g.node("sgv2.voicing.j413")
        self.assertIsNone(s.keep_voicing_volume(g, 0.0))
        self.assertEqual(self.ran, [])


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


class Idle(unittest.TestCase):
    """The speakers are wanted only while a client stream runs into the
    protect stage; otherwise the amps and the sense are unlinked and
    suspended, and the report asks for no grant."""

    def voiced(self, m="j316", state="running"):
        """The configured graph with the generated voicing (a filter-chain
        pair: its sink and its output share a link group) and one client
        playing into the voicing's sink."""
        g, p, s = build(m, linked=True)
        amps = MACHINES[m]["amps"]
        # the build's plain voicing out is replaced by the pair
        for lid, o, i in g.links():
            if o in p["v"]:
                del g.objs[lid]
        vin = g.node(f"sgv2.voicing.{m}", state="running", group="filter-chain-1-32")
        vout = g.node(f"sgv2.voicing.{m}.out", state="running", group="filter-chain-1-32")
        vin_p = [g.port(vin, "in", ch) for ch in ("FL", "FR")]
        vout_p = [g.port(vout, "out", f"AUX{i}") for i in range(amps)]
        for i in range(amps):
            g.link(vout_p[i], p["pin"][i])
        app = g.node("Firefox", state=state)
        app_p = [g.port(app, "out", ch) for ch in ("FL", "FR")]
        for a, b in zip(app_p, vin_p):
            g.link(a, b)
        return g, p, s, app, vin

    def test_a_client_through_the_voicing_is_found_and_plays_while_running(self):
        g, _, s, app, vin = self.voiced()
        self.assertEqual(s.clients(g), {app})
        self.assertTrue(s.playing(g))
        for st in ("idle", "suspended", "paused", None):
            g.objs[app]["info"]["state"] = st
            self.assertFalse(s.playing(g), st)

    def test_the_stage_itself_the_sense_and_the_amps_are_never_clients(self):
        # no client at all: the voicing pair and the sense run, nothing plays
        g, _, s, app, _ = self.voiced()
        for lid, o, i in g.links():
            if (g.objs.get(o) or {}).get("info", {}).get("props", {}).get("node.id") == app:
                del g.objs[lid]
        self.assertEqual(s.clients(g), set())
        self.assertFalse(s.playing(g))

    def test_a_client_straight_into_the_protect_stage_counts(self):
        g, p, s = build(linked=True)
        ab = g.node("effect_output.some-ab-sink", state="running")
        g.link(g.port(ab, "out", "AUX0"), p["pin"][0])
        self.assertIn(ab, s.clients(g))
        self.assertTrue(s.playing(g))

    def test_a_recorder_of_the_voicing_is_not_a_client(self):
        g, _, s, app, vin = self.voiced(state="idle")
        rec = g.node("recorder", state="running")
        g.link(g.port(vin, "out", "FL"), g.port(rec, "in", "FL"))
        self.assertNotIn(rec, s.clients(g))
        self.assertFalse(s.playing(g))

    def test_the_speakers_stay_wanted_for_the_hold_after_the_last_stream(self):
        g, _, s, app, _ = self.voiced()
        self.assertTrue(s.want(g, 100.0))
        g.objs[app]["info"]["state"] = "idle"
        self.assertTrue(s.want(g, 100.0 + sgs.IDLE_HOLD_S - 0.01))
        self.assertFalse(s.want(g, 100.0 + sgs.IDLE_HOLD_S))
        # a fresh helper with nothing playing never wants them
        g2, _, s2, app2, _ = self.voiced(state="idle")
        self.assertFalse(s2.want(g2, 1.0))

    def test_the_first_link_after_idle_is_not_rate_limited(self):
        g, _, s, app, _ = self.voiced(state="idle")
        s.want(g, 1.0)
        s.last_try = 0.9
        g.objs[app]["info"]["state"] = "running"
        self.assertTrue(s.want(g, 1.1))
        self.assertLess(s.last_try, 0.0)

    def test_play_ends_only_once_the_pcms_are_suspended_or_after_the_bound(self):
        g, _, s, app, _ = self.voiced()
        spk, sense = g.node_id("sgv2.speaker.j316"), g.node_id("sgv2.sense.j316")
        for n in (spk, sense):
            g.objs[n]["info"]["state"] = "running"
        s.want(g, 10.0)
        g.objs[app]["info"]["state"] = "idle"
        t = 10.0 + sgs.IDLE_HOLD_S
        self.assertFalse(s.want(g, t))
        # still linked: the grant stays while the helper takes them down
        self.assertTrue(s.play_report(g, False, t))
        for lid, o, i in g.links():
            if (g.objs.get(i) or {}).get("info", {}).get("props", {}).get("node.id") == spk:
                del g.objs[lid]
        self.assertTrue(s.play_report(g, False, t + 0.1))
        for n in (spk, sense):
            g.objs[n]["info"]["state"] = "suspended"
        self.assertFalse(s.play_report(g, False, t + 0.2))
        # a node that never suspends does not hold the grant past the bound
        g.objs[spk]["info"]["state"] = "idle"
        self.assertTrue(s.play_report(g, False, t + 0.3))
        self.assertFalse(s.play_report(g, False, t + sgs.TEARDOWN_S))
        self.assertTrue(s.play_report(g, True, t + sgs.TEARDOWN_S))
        # a helper that starts with nothing playing asks for no grant at all
        g, _, s, _, _ = self.voiced(state="idle")
        for n in (g.node_id("sgv2.speaker.j316"), g.node_id("sgv2.sense.j316")):
            g.objs[n]["info"]["state"] = "idle"
        self.assertFalse(s.want(g, 1.0))
        self.assertFalse(s.play_report(g, False, 1.0))

    def _reconcile(self, g, s, granted, play):
        ran = []

        class P:
            def __init__(self, argv):
                ran.append(argv)

            def poll(self):
                return 0

        orig = sgs.subprocess.Popen
        sgs.subprocess.Popen = lambda argv, **kw: P(argv)
        try:
            s.reconcile(g, granted, play)
        finally:
            sgs.subprocess.Popen = orig
        return ran

    def test_granted_but_idle_unlinks_the_amps_and_the_sense_then_suspends_them(self):
        c = MACHINES["j616"]
        g, p, s = build("j616", linked=True)
        ran = self._reconcile(g, s, True, False)
        self.assertEqual(len(ran), c["amps"] + c["sense"], ran)
        self.assertTrue(all(a[:2] == ["pw-link", "-d"] for a in ran), ran)
        # once nothing is linked: both nodes get the Suspend command
        g, p, s = build("j616", linked=False)
        for n in ("sgv2.speaker.j616", "sgv2.sense.j616"):
            g.objs[g.node_id(n)]["info"]["state"] = "idle"
        ran = self._reconcile(g, s, True, False)
        self.assertEqual(sorted(a[2] for a in ran),
                         sorted(str(g.node_id(n)) for n in ("sgv2.speaker.j616", "sgv2.sense.j616")))
        self.assertTrue(all(a[:2] == ["pw-cli", "send-command"] and a[3] == "Suspend" for a in ran), ran)

    def test_granted_and_playing_links_and_playing_alone_never_does(self):
        c = MACHINES["j616"]
        g, p, s = build("j616", linked=False)
        ran = self._reconcile(g, s, True, True)
        self.assertEqual(len(ran), c["amps"] + c["sense"], ran)
        self.assertTrue(all(a[0] == "pw-link" and a[1] != "-d" for a in ran), ran)
        g, p, s = build("j616", linked=False)
        ran = self._reconcile(g, s, False, True)
        self.assertEqual(ran, [])

    def test_the_report_carries_play(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "session")
            self.assertTrue(sgs.write_report(path, True, True, "0123456789abcdef", False, False))
            with open(path) as f:
                line = f.read()
            self.assertTrue(line.endswith(" amps=0 play=0\n"), line)


class Memo(unittest.TestCase):
    def test_graph_derivations_are_computed_once_per_change(self):
        g = sgs.Graph.__new__(sgs.Graph)
        g.objs, g.gen, g._memo = {}, 0, {}
        calls = []
        f = lambda: calls.append(1) or len(calls)
        self.assertEqual(g.memo("k", f), 1)
        self.assertEqual(g.memo("k", f), 1)
        g.gen += 1
        self.assertEqual(g.memo("k", f), 2)


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

    def test_voicing_lowpass_is_interim_validated_and_only_on_the_j700(self):
        self.assertIsNone(gen.voicing_lowpass("j700", {}))
        self.assertEqual(gen.voicing_lowpass("j700", {"voicing_pre_lowpass": {"j700": {"hz": 17400, "order": 8}}}), (17400.0, 8))
        for bad in ({"hz": 17400}, {"hz": 17400, "order": 7}, {"hz": 17400, "order": 8.0}, {"hz": "17400", "order": 8},
                    {"hz": True, "order": 8}, {"hz": 500, "order": 8}, {"hz": 30000, "order": 8}, {"hz": float("nan"), "order": 8},
                    {"hz": 17400, "order": 8, "gain": 0}, [17400, 8], 17400):
            with self.assertRaises(ValueError):
                gen.voicing_lowpass("j700", {"voicing_pre_lowpass": {"j700": bad}})
        # Butterworth pole-pair Qs (order 2: 1/sqrt(2); order 8: the four classic values)
        self.assertAlmostEqual(gen.butterworth_q(2)[0], 2 ** -0.5, places=12)
        self.assertEqual([round(q, 4) for q in gen.butterworth_q(8)], [2.5629, 0.9, 0.6013, 0.5098])
        # Shipped gate: no board needs the interim band limit since the AID44 ControlFreak saturation
        # law is exact (2026-11-08); the mechanism stays for a future open law.
        on = sorted(b for b in (gen.fixture(m)["hw"]["board"] for m in gen.machines()) if gen.voicing_lowpass(b) is not None)
        self.assertEqual(sorted(set(on)), [])
        for m in gen.machines():
            c, w = gen.machine(m)
            conf = gen.pipewire_conf(m, c, w, voicing_pre=True)
            lp = gen.voicing_lowpass(c.get("board", m))
            if lp is None:
                self.assertNotIn("bq_lowpass", conf)
                self.assertIn('inputs = [ "voicing:in_0" "voicing:in_1" ]', conf)
                continue
            hz, order = lp
            self.assertEqual(conf.count("label = bq_lowpass"), order)
            self.assertEqual(conf.count(f'"Freq" = {hz:.1f}'), order)
            self.assertIn('inputs = [ "lp0s0:In" "lp1s0:In" ]', conf)
            for ch in (0, 1):
                self.assertIn(f'{{ output = "lp{ch}s{order // 2 - 1}:Out" input = "voicing:in_{ch}" }}', conf)
        # A corner at or above Nyquist refuses the board.
        m = next(m for m in gen.machines() if gen.fixture(m)["hw"]["board"] == "j700")
        c, w = gen.machine(m)
        orig = gen.voicing_lowpass
        try:
            gen.voicing_lowpass = lambda board, gate=None: (24000.0, 8)
            with self.assertRaises(ValueError):
                gen.voicing_conf(m, c, w)
        finally:
            gen.voicing_lowpass = orig

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


    def test_the_protect_input_is_internal(self):
        # A desktop that lists it, or resolves its volume keys through the
        # voicing to it, would change its volume and drop the carrier.
        for m in gen.machines():
            c, w = gen.machine(m)
            for pre in (False, True):
                conf = gen.pipewire_conf(m, c, w, voicing_pre=pre)
                block = conf.split(f'node.name          = "{gen.PROTECT_INPUT}"', 1)[1].split("}", 1)[0]
                self.assertIn('media.class        = "Audio/Sink/Internal"', block)

    def test_the_amps_and_sense_are_one_internal_link_group(self):
        # wpctl and other session tools list a node with a link group as part
        # of a graph, not as a device sink or source; the group is the same
        # set as node.group, so the drivers do not change.
        for m in gen.machines():
            c, w = gen.machine(m)
            n = gen.names(m)
            for pre in (False, True):
                conf = gen.pipewire_conf(m, c, w, voicing_pre=pre)
                for node in ("speaker", "sense") if c["sense"] else ("speaker",):
                    block = conf.split(f'node.name              = "{n[node]}"', 1)[1].split("}", 1)[0]
                    self.assertIn(f'node.link-group        = "{n["group"]}"', block)
                    self.assertIn(f'node.group             = "{n["group"]}"', block)
                    self.assertIn('/Internal"', block)

    def test_every_sink_of_the_graph_holds_the_pcm_rate(self):
        # The amps PCM runs at audio.rate with resampling off. A start after
        # a grant, a wake or a boot takes the highest rate of a driver's
        # followers, and while the amps wait for their grant the voicing
        # sink runs on another driver at a playing client's rate: a 44.1 kHz
        # client then wedged the path (the amps can never start). The amps
        # sink, the protect stage and the voicing sink hold the PCM rate.
        for m in gen.machines():
            c, w = gen.machine(m)
            for voicing_pre in (False, True):
                text = gen.pipewire_conf(m, c, w, voicing_pre=voicing_pre)
                sink = text.split("factory.name           = api.alsa.pcm.sink", 1)[1].split("}", 1)[0]
                self.assertIn(f"audio.rate             = {c['rate']}", sink)
                self.assertIn(f"node.force-rate        = {c['rate']}", sink)
                protect = text.split(f'node.name          = "{gen.names(m)["protect"]}"', 1)[1].split("}", 1)[0]
                self.assertIn(f"node.force-rate    = {c['rate']}", protect)
                if voicing_pre:
                    voicing = text.split(f'node.name = "sgv2.voicing.{m}"', 1)[1].split("}", 1)[0]
                    self.assertIn(f"node.force-rate = {c['rate']}", voicing)

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
