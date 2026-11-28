"""A boot run waits for the speaker card the device tree describes (SGV2-FLEET 11-28: on a fresh J413 boot the
card registered 1 s after the run, which then waited a whole boot)."""
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from aurora_speaker_upgrade import upgrade as up  # noqa: E402


def tree(d, nodes):
    """nodes: {name: (compatible, status or None, [link-names])}"""
    for name, (compat, status, links) in nodes.items():
        n = os.path.join(d, name)
        os.makedirs(n)
        with open(os.path.join(n, "compatible"), "wb") as f:
            f.write(compat + b"\0")
        if status is not None:
            with open(os.path.join(n, "status"), "wb") as f:
                f.write(status + b"\0")
        for i, link in enumerate(links):
            os.makedirs(os.path.join(n, f"dai-link@{i}"))
            with open(os.path.join(n, f"dai-link@{i}", "link-name"), "wb") as f:
                f.write(link + b"\0")


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


class DtSpeakerCard(unittest.TestCase):
    def check(self, nodes, want):
        with tempfile.TemporaryDirectory() as d:
            tree(d, nodes)
            self.assertIs(up.unit.dt_speaker_card(d), want)

    def test_speaker_link(self):
        self.check({"sound": (b"apple,j413-macaudio\0apple,macaudio", None, [b"Speaker", b"Headphone Jack"])}, True)

    def test_speakers_plural(self):
        self.check({"sound": (b"apple,macaudio", b"okay", [b"Speakers"])}, True)

    def test_jack_only_card(self):
        self.check({"sound": (b"apple,macaudio", None, [b"Headphone Jack"])}, False)

    def test_disabled(self):
        self.check({"sound": (b"apple,macaudio", b"disabled", [b"Speaker"])}, False)

    def test_no_macaudio(self):
        self.check({"cpus": (b"apple,cpus", None, [])}, False)

    def test_unreadable(self):
        self.assertIsNone(up.unit.dt_speaker_card("/nonexistent/dt"))


class WaitInterfaceMode(unittest.TestCase):
    def run_wait(self, modes, dt_card, wait):
        c = Clock()
        seq = iter(modes)
        with mock.patch.object(up, "interface_mode", side_effect=lambda: next(seq)), \
                mock.patch.object(up.unit, "dt_speaker_card", return_value=dt_card):
            try:
                return up.wait_interface_mode(wait, sleep=c.sleep, clock=c), c.t
            except up.Retry as e:
                return str(e), c.t

    def test_card_registers_late(self):
        got, t = self.run_wait([("none", None)] * 2 + [("dual", "1")], True, 120)
        self.assertEqual(got, ("dual", "1"))
        self.assertEqual(t, 1.0)

    def test_card_present_no_wait(self):
        got, t = self.run_wait([("v2-only", "0")], True, 120)
        self.assertEqual((got, t), (("v2-only", "0"), 0.0))

    def test_gives_up_after_wait(self):
        got, t = self.run_wait([("none", None)] * 1000, True, 120)
        self.assertIn("no speaker card yet (waited 120 s)", got)
        self.assertGreaterEqual(t, 120)

    def test_dt_without_speaker_card_retries_at_once_with_reason(self):
        got, t = self.run_wait([("none", None)] * 10, False, 120)
        self.assertIn("lacks the speaker amp nodes", got)
        self.assertEqual(t, 0.0)

    def test_unreadable_dt_still_waits(self):
        got, t = self.run_wait([("none", None)] * 3 + [("dual", "2")], None, 120)
        self.assertEqual(got, ("dual", "2"))

    def test_by_hand_run_does_not_wait(self):
        got, t = self.run_wait([("none", None)] * 10, True, 0)
        self.assertIn("no speaker card yet", got)
        self.assertEqual(t, 0.0)


class RunUsesWait(unittest.TestCase):
    def test_at_boot_waits_by_hand_does_not(self):
        seen = []

        class Ident:
            def __init__(self, adt):
                pass

            def matches_dt(self):
                return True

        def fake_wait(w, **kw):
            seen.append(w)
            raise up.Retry("stop here")
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(up.collect, "Identity", Ident), \
                mock.patch.object(up, "wait_interface_mode", fake_wait):
            for at_boot in (True, False):
                a = mock.Mock(store=d, adt=None, at_boot=at_boot)
                with self.assertRaises(up.Retry):
                    up._cmd_run(a)
        self.assertEqual(seen, [up.CARD_WAIT_S, 0])


if __name__ == "__main__":
    unittest.main()


class AcquireWaiting(unittest.TestCase):
    """A boot run keeps trying to reach the pinned restore image (on a fresh J413 boot DNS answered only after
    network-online.target)."""

    def go(self, results, wait):
        c = Clock()
        seq = iter(results)

        def fake(identity, a):
            r = next(seq)
            if isinstance(r, Exception):
                raise r
            return r
        with mock.patch.object(up, "acquire", fake):
            try:
                return up.acquire_waiting(None, None, wait, sleep=c.sleep, clock=c), c.t
            except up.Retry as e:
                return e, c.t

    def test_reaches_after_dns(self):
        got, t = self.go([up.Unreachable("dns")] * 3 + [("b", {"p": 1})], 180)
        self.assertEqual((got, t), (("b", {"p": 1}), 15))

    def test_gives_up(self):
        got, t = self.go([up.Unreachable("dns")] * 100, 180)
        self.assertIsInstance(got, up.Unreachable)
        self.assertLessEqual(t, 180)

    def test_other_retry_not_waited(self):
        got, t = self.go([up.Retry("self-check cannot run now")], 180)
        self.assertNotIsInstance(got, up.Unreachable)
        self.assertEqual(t, 0)

    def test_refusal_passes_through(self):
        with mock.patch.object(up, "acquire", side_effect=up.Refused("no tuning")):
            with self.assertRaises(up.Refused):
                up.acquire_waiting(None, None, 180, sleep=lambda s: None)

    def test_by_hand_no_wait(self):
        got, t = self.go([up.Unreachable("dns"), ("b", {})], 0)
        self.assertIsInstance(got, up.Unreachable)
        self.assertEqual(t, 0)

    def test_unreachable_is_a_retry(self):
        self.assertTrue(issubclass(up.Unreachable, up.Retry))


class SameSpeakerCard(unittest.TestCase):
    """After the (boot-time, possibly minutes-long) fetch the run checks it still talks to the same speaker card
    before it touches provisioning or the protection switch (review of #299)."""

    def check(self, now, ids, want_retry):
        with mock.patch.object(up, "interface_mode", return_value=now), \
                mock.patch.object(up, "card_identity", side_effect=lambda c: ids.get(c)):
            if want_retry:
                with self.assertRaises(up.Retry) as cm:
                    up.same_speaker_card("dual", "1", "AppleJ413")
                self.assertIn("changed during the fetch", str(cm.exception))
            else:
                up.same_speaker_card("dual", "1", "AppleJ413")

    def test_unchanged(self):
        self.check(("dual", "1"), {"1": "AppleJ413"}, False)

    def test_card_disappeared(self):
        self.check(("none", None), {}, True)

    def test_card_renumbered(self):
        self.check(("dual", "2"), {"2": "AppleJ413"}, True)

    def test_number_reused_by_another_card(self):
        self.check(("dual", "1"), {"1": "HDMI"}, True)

    def test_mode_changed(self):
        self.check(("v2-only", "1"), {"1": "AppleJ413"}, True)

    def test_card_identity_reads_proc(self):
        self.assertIsNone(up.card_identity("99999"))


class RunRevalidatesBeforeStoring(unittest.TestCase):
    def test_changed_card_stops_before_store(self):
        stored = []

        class Ident:
            unit, board, acoustic = "u" * 64, "j413", 13

            def __init__(self, adt):
                pass

            def matches_dt(self):
                return True
        modes = iter([("dual", "1"), ("none", None)])
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(up.collect, "Identity", Ident), \
                mock.patch.object(up, "interface_mode", side_effect=lambda: next(modes)), \
                mock.patch.object(up, "card_identity", return_value="AppleJ413"), \
                mock.patch.object(up, "playback_proof", return_value=True), \
                mock.patch.object(up, "acquire", return_value=({}, {"source": "x"})), \
                mock.patch.object(up.Store, "put_bundle", lambda self, b: stored.append(b)), \
                mock.patch.object(up, "say"):
            a = mock.Mock(store=d, adt=None, at_boot=False)
            with self.assertRaises(up.Retry):
                up._cmd_run(a)
        self.assertEqual(stored, [])


class Boundaries(unittest.TestCase):
    """Second review of #299: unreadable/empty card ids, a late wake past the budget, local staging failures."""

    def test_unreadable_ids_both_times_refuse_to_proceed(self):
        for ids in ({}, {"1": ""}):
            with self.subTest(ids=ids), mock.patch.object(up, "interface_mode", return_value=("dual", "1")), \
                    mock.patch.object(up, "card_identity", side_effect=lambda c: ids.get(c)):
                with self.assertRaises(up.Retry):
                    up.same_speaker_card("dual", "1", ids.get("1"))

    def test_run_needs_a_readable_id_before_fetching(self):
        fetched = []

        class Ident:
            unit, board, acoustic = "u" * 64, "j413", 13

            def __init__(self, adt):
                pass

            def matches_dt(self):
                return True
        for bad in (None, ""):
            with self.subTest(id=bad), tempfile.TemporaryDirectory() as d, \
                    mock.patch.object(up.collect, "Identity", Ident), \
                    mock.patch.object(up, "interface_mode", return_value=("dual", "1")), \
                    mock.patch.object(up, "card_identity", return_value=bad), \
                    mock.patch.object(up, "playback_proof", return_value=True), \
                    mock.patch.object(up, "acquire", side_effect=lambda i, a: fetched.append(1)), \
                    mock.patch.object(up, "say"):
                with self.assertRaises(up.Retry) as cm:
                    up._cmd_run(mock.Mock(store=d, adt=None, at_boot=False))
                self.assertIn("ALSA id", str(cm.exception))
        self.assertEqual(fetched, [])

    def test_late_wake_starts_no_attempt_after_budget(self):
        c = Clock()
        calls = []

        def fake(identity, a):
            calls.append(c.t)
            raise up.Unreachable("dns")

        def late_sleep(s):
            c.t += 400  # the machine slept through the budget
        with mock.patch.object(up, "acquire", fake):
            with self.assertRaises(up.Unreachable):
                up.acquire_waiting(None, None, 180, sleep=late_sleep, clock=c)
        self.assertEqual(calls, [0.0])

    def test_local_enospc_is_not_retried_as_unreachable(self):
        import errno as _errno
        with tempfile.TemporaryDirectory() as d:
            a = mock.Mock(ipsw=None)
            ident = mock.Mock(board="j413", acoustic=13)
            rng = mock.Mock(fetched=0)
            with mock.patch.object(up.ipsw, "open_system", return_value=(object(), rng, "26B123", "https://x/y.ipsw")), \
                    mock.patch.object(up.ipsw, "pin", side_effect=up.ipsw.IpswError("no pin")), \
                    mock.patch.object(up.unit, "installer_fill_build", return_value=None), \
                    mock.patch.object(up.collect, "stage", side_effect=OSError(_errno.ENOSPC, "No space left on device")), \
                    mock.patch.object(up, "say"):
                with self.assertRaises(up.Retry) as cm:
                    up.from_ipsw(ident, a, None, False, d)
        self.assertNotIsInstance(cm.exception, up.Unreachable)
        self.assertIn("on this Mac", str(cm.exception))

    def test_network_oserror_stays_unreachable(self):
        import errno as _errno
        for e in (OSError(_errno.ECONNRESET, "reset"), TimeoutError("timed out"),
                  up.urllib.error.URLError("Temporary failure in name resolution")):
            with self.subTest(e=e):
                self.assertFalse(up.local_failure(e) if isinstance(e, OSError) else False)

    def test_acquire_waiting_does_not_repeat_local_failure(self):
        calls = []

        def fake(identity, a):
            calls.append(1)
            raise up.Retry("staging the restore image's files failed on this Mac: ENOSPC")
        with mock.patch.object(up, "acquire", fake):
            with self.assertRaises(up.Retry):
                up.acquire_waiting(None, None, 180, sleep=lambda s: None)
        self.assertEqual(calls, [1])
