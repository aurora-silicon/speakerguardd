# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""aurora-speaker-upgrade: move this Mac's speakers from the legacy
protection (speakersafetyd, speakerguardd 0.1) to speakerguardd v2, with
this Mac's own tuning, and back.

    aurora-speaker-upgrade [run] [--ipsw URL] [--no-switch]
    aurora-speaker-upgrade --revert [--purge]
    aurora-speaker-upgrade --check [--json FILE]                (dry run: /run only, nothing stored or switched)
    aurora-speaker-upgrade status
    aurora-speaker-upgrade collect --out DIR                   (no switch; for checks)
    aurora-speaker-upgrade interface-apply                     (what speakerguardd-gate interface does; kept for scripts)

1. The tuning bundle, the installer's own, built from one source only:
   Apple's restore image of the pinned macOS release (27.2, the speaker
   safety stack's), read by HTTP ranges (ipsw.py). Never from an installed
   macOS, an older bundle or another build; no restore image, no bundle.
   Only this unit's own values come from the Mac (unit.py: the ADT, the
   boot chain's speaker ID, and the speaker ID and lsm2 readback of this
   unit's installer bundle when they fit). Nothing is guessed.
2. It is stored root-only in /var/lib/aurora-speaker-upgrade (0700), never
   packaged or sent anywhere.
3. speakerguardd v2 compiles it and its self-check proves it, silently.
4. Only on a pass, the kernel's speaker protection interface switches to v2.
   The legacy stack stays installed; any failure leaves (or puts) it in
   charge, and `--revert` goes back to it.
"""
import argparse
import contextlib
import fcntl
import glob
import hashlib
import json
import logging
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time

from . import collect, ipsw, unit
from .asahi_firmware import aea_ro, apfs_ro
from .asahi_firmware import audio_tuning

log = logging.getLogger("aurora-speaker-upgrade")

STORE = "/var/lib/aurora-speaker-upgrade"
LOCK = "/run/aurora-speaker-upgrade.lock"
PROVISION_CONF = "/etc/speakerguardd/provision.conf"
PARAMS = "/var/lib/speakerguardd/params.bin"
V2_CONF = "/etc/speakerguardd/speakerguardd.conf"
STATE_V2 = "/var/lib/speakerguardd"
LIBEXEC = ("/usr/libexec/speakerguardd-v2", "/usr/libexec/speakerguardd")
INTERFACE_CTL = "Speaker Protection Interface"
IF_V2, IF_LEGACY = "Speaker Protection", "Speaker Volume Unlock"
V2_UNITS = ("speakerguardd-v2.service", "speakerguardd.service")
GRANT_WAIT_S = 30
# After a revert restarts the seat user's PipeWire, the older daemon starts
# once the speaker card's PCMs have kept their state this long
# (at most PCM_SETTLE_S).
PCM_SETTLE_S = 10
PCM_QUIET_S = 1.5
# The seat user's audio session stops before the kernel's speaker interface
# switches and starts after it (speakerguardd-provision session-stop,
# session-start). A session left running across the switch keeps starting
# the speaker PCM the new interface refuses (EPERM): PipeWire raises a node
# error for each refused start, and on a loaded host WirePlumber then
# outlived its 90 s stop timeout, longer than the old 60 s restart bound.
# Provisioning's stop is bounded (15 s grace, SIGKILL, 10 s, 15 s slack);
# these limits are above its worst case.
SESSION_STOP_LIMIT_S = 15 + 10 + 15 + 30
# the graph unit (TimeoutStartSec=180) and the units' start wait (30 s)
SESSION_START_LIMIT_S = 180 + 30 + 30
PARITY = ("/usr/share/speakerguardd/parity.json",)
# speakerguardd v2's gate data: its playback_rows record the protected
# playback rows, by speaker (the tuning's AID): sound played through v2's
# protection on that speaker, measured with V/I sense and the Mac's own
# microphone. The boot unit moves a speaker to v2 only when its rows cover
# every item of the record's playback_covers; a silent self-check alone is not
# playback evidence. `run` by hand is not limited by it.
GATE = "/usr/share/speakerguardd/gate.json"
MARK = "# aurora-speaker-upgrade: the bundle collected for this Mac"

EXIT_OK, EXIT_REFUSED, EXIT_RETRY, EXIT_ERROR = 0, 1, 2, 3


class Refused(Exception):
    """Stay on (or go back to) the legacy stack."""


class Retry(Exception):
    """Cannot run now; nothing changed."""


class Unreachable(Retry):
    """The pinned restore image could not be read now (no network yet, DNS,
    a timeout, a server error): a boot run waits a while for it."""


def say(msg):
    print(f"aurora-speaker-upgrade: {msg}", file=sys.stderr, flush=True)


# --- the store -------------------------------------------------------------

class Store:
    def __init__(self, root=STORE):
        self.root = root
        self.bundle = os.path.join(root, "bundle")
        self.state_path = os.path.join(root, "state.json")
        self.interface = os.path.join(root, "interface")

    def ensure(self):
        os.makedirs(self.root, mode=0o700, exist_ok=True)
        os.chmod(self.root, 0o700)
        st = os.lstat(self.root)
        if st.st_uid != 0 and os.geteuid() == 0:
            raise Refused(f"{self.root} is not owned by root")

    def state(self):
        try:
            with open(self.state_path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def save(self, state):
        tmp = self.state_path + ".new"
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, "w") as f:
            json.dump(state, f, indent=2, sort_keys=True)
            f.write("\n")
        os.rename(tmp, self.state_path)

    def put_bundle(self, bundle):
        new = self.bundle + ".new"
        collect.rmtree(new)
        collect.write_bundle(bundle, new)
        old = self.bundle + ".old"
        collect.rmtree(old)
        if os.path.lexists(self.bundle):
            os.rename(self.bundle, old)
        os.rename(new, self.bundle)
        collect.rmtree(old)

    def read_bundle(self):
        if not os.path.isfile(os.path.join(self.bundle, audio_tuning.MANIFEST)):
            return None
        c = audio_tuning.AudioTuningFWCollection.__new__(audio_tuning.AudioTuningFWCollection)
        c.fwfiles, c.bundle = [], {}
        try:
            c.update(self.bundle)
        except Exception:
            return None
        return c.bundle or None


def digest(bundle):
    import hashlib
    return hashlib.sha256(bundle[audio_tuning.MANIFEST]).hexdigest()


# --- acquisition -------------------------------------------------------------

def speaker_id_source(identity, chosen=collect.CHOSEN_SPID, dirs=collect.INSTALLER_BUNDLES):
    """(speaker ID or None, known?, where from): unit.speaker_id."""
    return unit.speaker_id(identity, chosen, dirs)


def from_ipsw(identity, a, spid, known, scratch):
    """(bundle, provenance, AudioDSP SHA-256) from the pinned restore image."""
    # Reaching Apple is not an answer: no network (a laptop whose Wi-Fi
    # connects only after login), a timeout or a server error is Retry, which
    # leaves no refusal record, so the next boot tries again.
    try:
        pinned_builds = [b for b, _ in ipsw.pin()[1]]
    except ipsw.IpswError:
        pinned_builds = None
    preferred = unit.installer_fill_build(identity, f"AID{identity.acoustic}",
                                          getattr(a, "installer_dirs", collect.INSTALLER_BUNDLES),
                                          pinned_builds=pinned_builds)
    try:
        kw = {"preferred_build": preferred} if preferred else {}
        vol, rng, build, url = ipsw.open_system(identity.board, url=a.ipsw, **kw)
    except ipsw.IpswUnavailable as e:
        raise Unreachable(f"the pinned restore image cannot be read now: {e}") from None
    except ipsw.IpswError as e:
        raise collect.CollectError(str(e)) from None
    say(f"macOS {build} restore image ({url.rsplit('/', 1)[-1]}), by ranges")
    st = tempfile.mkdtemp(prefix="stage-", dir=scratch)
    try:
        collect.stage(vol, identity.acoustic, st)
    except (aea_ro.AeaUnavailable, OSError) as e:
        raise Unreachable(f"the pinned restore image cannot be read now: {e}") from None
    except (aea_ro.AeaError, apfs_ro.ApfsError) as e:
        raise collect.CollectError(f"macOS {build} restore image: {e}") from None
    b, sysbuild = collect.collect(st, identity, spid, known)
    if sysbuild != build:
        raise collect.CollectError(f"the restore image's System volume is macOS {sysbuild}, not {build}")
    say(f"{rng.fetched >> 20} MiB fetched from Apple")
    return b, {"source": url.rsplit("/", 1)[-1], "macos_build": sysbuild}, collect.component_sha256(st)


def with_fill(identity, b, component, dirs):
    """b with this unit's lsm2 readback when its short presets need one
    (collect.attach_fill decides whether it fits); refuses without one."""
    m = json.loads(b[audio_tuning.MANIFEST])
    if not collect.lsm2_fill.needs_fill(b) or m.get("lsm2_param_fill") is not None:
        return b, None
    reasons = []
    for d, fill in unit.installer_fill(identity, m.get("aid"), dirs):
        try:
            return collect.attach_fill(b, fill, component), d
        except collect.CollectError as e:
            reasons.append(f"{d}: {e}")
    raise Refused(f"{m.get('aid')} has short lsm2 presets, which need the parameter readback the installer "
                  f"takes on this Mac's macOS {m.get('macos_build')} from the same files; "
                  + ("; ".join(reasons) if reasons else "this unit's installer bundle has none")
                  + ". The speakers stay as they are")


def acquire_waiting(identity, a, wait, sleep=time.sleep, clock=time.monotonic):
    """acquire(), starting a new attempt every 5 s while the pinned restore
    image cannot be reached, as long as the attempt starts within `wait`
    seconds (the last attempt itself is bounded by the reader's HTTP
    timeouts, not by `wait`). Anything else (a refusal, a
    retry for another reason) is passed on at once."""
    deadline = clock() + wait
    while True:
        try:
            return acquire(identity, a)
        except Unreachable:
            if clock() + 5 > deadline:
                raise
            sleep(5)


def acquire(identity, a):
    """(bundle, provenance) from the pinned restore image, or Refused."""
    spid, known, spid_from = speaker_id_source(identity, a.chosen, a.installer_dirs)
    os.makedirs(a.scratch, mode=0o700, exist_ok=True)
    scratch = tempfile.mkdtemp(prefix="scratch-", dir=a.scratch)
    try:
        try:
            b, prov, component = from_ipsw(identity, a, spid, known, scratch)
        except collect.CollectError as e:
            raise Refused(f"no speaker tuning for this Mac: {e}") from None
    finally:
        collect.rmtree(scratch)
    try:
        b, dropped = collect.for_this_reader(b)
    except collect.CollectError as e:
        raise Refused(str(e)) from None
    if dropped:
        say(f"the bundle carries {', '.join(dropped)}, which this speakerguardd does not read yet; "
            "it is stored without it")
    b, fill_from = with_fill(identity, b, component, a.installer_dirs)
    m = json.loads(b[audio_tuning.MANIFEST])
    if m.get("unit_id") != identity.unit:
        raise Refused("the bundle does not name this unit")
    prov.update(speaker_id=m.get("speaker_id"), speaker_id_from=spid_from, aid=m.get("aid"),
                macos_build=m.get("macos_build"), lsm2_fill_from=fill_from)
    say(f"tuning {m.get('aid')} from macOS {m.get('macos_build')} ({prov['source']})")
    return b, prov


# --- speakerguardd v2 --------------------------------------------------------

def tool(name):
    for d in LIBEXEC:
        p = os.path.join(d, name)
        if os.access(p, os.X_OK):
            return p
    return None


def run(argv, **kw):
    log.info("+ " + " ".join(argv))
    return subprocess.run(argv, capture_output=True, text=True, **kw)


# This tool runs with umask 077 (its store is root only). speakerguardd v2's
# own programs are written for their units' umask 022: run under 077 in the
# same boot as a switch, provisioning made /run/speakerguardd-graph and its
# subdirectories 0700, so the seat user's session could not read the graph
# state or the generated configuration, the session helper was skipped, and
# the supervisor never granted until a reboot (RC8 E2E B4, M1 Pro + M2).
V2_UMASK = 0o022


def run_v2(argv, **kw):
    return run(argv, umask=V2_UMASK, **kw)


def provision_conf_with(text, path):
    lines = [l for l in (text or "").splitlines() if not l.startswith("SGV2_BUNDLE=") and l != MARK]
    return "\n".join(lines + [MARK, f"SGV2_BUNDLE={path}"]) + "\n"


def make_parents(path, mode=0o755):
    """The directories above a file this tool writes outside its store, at
    `mode` whatever this tool's umask 077. /etc/speakerguardd made here (the
    switch writes provision.conf before v2 ever provisioned) came out 0700,
    and v2's speakerguardd.conf in it was then unreadable for the seat user's
    session: no speaker graph, no sound (J815)."""
    d = os.path.dirname(path)
    missing = []
    while d and not os.path.isdir(d):
        missing.append(d)
        d = os.path.dirname(d)
    for d in reversed(missing):
        os.makedirs(d, exist_ok=True)
        os.chmod(d, mode)


def write_file(path, text, mode=0o644):
    make_parents(path)
    tmp = path + ".aurora-speaker-upgrade.new"
    with open(tmp, "w") as f:
        f.write(text)
    os.chmod(tmp, mode)
    os.rename(tmp, path)


def read_text(path):
    try:
        with open(path) as f:
            return f.read()
    except FileNotFoundError:
        return None


def gate_paths():
    """The installed gate data, then the source tree's (tests, a checkout)."""
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return (GATE, os.path.join(here, "session", "gate.json"))


def playback_proof(acoustic, paths=None):
    """The recorded protected playback rows for speaker AID<acoustic>, or []
    unless those with evidence together cover the record's playback_covers
    (speakerguardd's gen.playback_rows). A record that cannot be read
    proves nothing."""
    for p in paths or gate_paths():
        try:
            with open(p) as f:
                gate = json.load(f)
        except FileNotFoundError:
            continue
        except (OSError, ValueError):
            return []
        if not isinstance(gate, dict):
            return []
        covers, record = gate.get("playback_covers"), gate.get("playback_rows")
        if not isinstance(covers, list) or not covers or not isinstance(record, dict):
            return []
        rows = record.get(f"AID{acoustic}")
        if not all(isinstance(c, str) for c in covers):
            return []
        rows = [r for r in rows if isinstance(r, dict) and isinstance(r.get("evidence"), str) and r["evidence"].strip()
                and isinstance(r.get("covers"), list) and all(isinstance(c, str) for c in r["covers"])] \
            if isinstance(rows, list) else []
        return rows if {c for r in rows for c in r["covers"]} >= set(covers) else []
    return []


def provision(store, a):
    p = tool("speakerguardd-provision")
    if not p:
        raise Refused("speakerguardd v2 is not installed (no speakerguardd-provision)")
    r = run_v2([p, "run"] + (["--force"] if a.force else []))
    sys.stderr.write(r.stderr[-2000:])
    if r.returncode != 0:
        raise Refused(f"speakerguardd v2 could not compile this Mac's tuning (exit {r.returncode})")


def v2_tuning_args(conf=None):
    """The tuning source the units pass the self-check (SGV2_TUNING)."""
    with contextlib.suppress(OSError):
        with open(conf or V2_CONF) as f:
            for line in f:
                if line.startswith("SGV2_TUNING="):
                    return shlex.split(line.split("=", 1)[1])
    return []


def selfcheck(a, mode=None):
    """SGFIX's verify: 0 pass, 1 refused, 2 cannot run, 3 error. Until
    `verify` exists: a passing record for this kernel and blob, else a full
    `run`. On a dual kernel the hardware record of this kernel is made
    first (`run --switch-interface`: reused at once when it is bound
    already), with the speakers on the v2 lease, so verify has it."""
    p = tool("speakerguardd-selfcheck")
    if not p:
        raise Refused("speakerguardd v2 is not installed (no speakerguardd-selfcheck)")
    js = os.path.join(a.scratch, "selfcheck.json")
    os.makedirs(a.scratch, mode=0o700, exist_ok=True)
    h = run([p, "--help"])
    if mode == "dual" and "--switch-interface" in h.stdout + h.stderr:
        r = run_v2([p, "run", "--switch-interface", "--params", PARAMS, "--", *v2_tuning_args()])
        sys.stderr.write(r.stderr[-3000:])
        if r.returncode != 0:
            raise Refused(f"the silent hardware self-check failed on this kernel (exit {r.returncode}): "
                          "the legacy protection keeps the speakers")
        # The boot did not prepare v2 on a Mac that was not switched yet (the
        # gate kept the older daemons), and a revert removed the graph:
        # generate it from the record just made and bring the session onto
        # it (as speakerguardd-selfcheck.service's ExecStartPost does at
        # boot), so verify's graph step sees it. The session stopped before
        # the switch (session_stop) starts on it.
        graph_activate()
        session_start()
    if re.search(r"\bverify\b", h.stdout + h.stderr):
        r = run_v2([p, "verify", "--params", PARAMS, "--json", js])
        code = r.returncode
    else:
        r = run_v2([p, "check", "--params", PARAMS])
        code = r.returncode
        if code != 0:
            r = run_v2([p, "run", "--params", PARAMS])
            code = 0 if r.returncode == 0 else 1
    sys.stderr.write(r.stderr[-3000:])
    if code == 0:
        return
    if code == 2:
        raise Retry("the self-check cannot run now (a session holds the speakers, or no card): try again later")
    raise Refused(f"the speaker self-check refused this Mac (exit {code}): the legacy protection keeps the speakers")


# --- the kernel interlock (SGUP-KI) -------------------------------------------

def _has(card, name):
    # cget, not `amixer controls`: inactive controls are not listed there.
    return run(["amixer", "-c", card, "cget", f"name={name}"]).returncode == 0


def interface_mode():
    """('dual' (the kernel has the interlock switch) | 'v2-only' (a fleet
    kernel: only the v2 interface) | 'legacy-only' | 'none', card)."""
    cards = sorted(c[4:] for c in (os.listdir("/proc/asound") if os.path.isdir("/proc/asound") else [])
                   if re.fullmatch(r"card\d+", c))
    for card in cards:
        if _has(card, INTERFACE_CTL):
            return "dual", card
    for card in cards:
        if _has(card, IF_V2):
            return "v2-only", card
    for card in cards:
        if _has(card, IF_LEGACY):
            return "legacy-only", card
    return "none", None


# How long a boot run waits for the speaker card. The unit is ordered after
# sound.target, but the card's driver can bind a second after that (J413: the
# card registered 1 s after the run on a fresh boot), and a run that finds no
# card waits for the next boot.
CARD_WAIT_S = 120
# How long a boot run keeps starting new attempts at the pinned restore
# image (a retry-start budget: an attempt started inside it runs to the end
# under the reader's own HTTP timeouts). On a fresh J413 boot
# network-online.target was reached before DNS answered, and the run gave up
# until the next boot.
NET_WAIT_S = 180



def card_identity(card):
    """The card's ALSA id (/proc/asound/cardN/id, e.g. AppleJ413), or None
    when card N is gone. With the number it tells a reprobed or renumbered
    card from the one a run started on."""
    try:
        with open(f"/proc/asound/card{card}/id") as f:
            return f.read().strip()
    except OSError:
        return None


def same_speaker_card(mode, card, ident):
    """Retry unless the kernel still serves `mode` on the same card number
    with the same ALSA id. Called after the restore image fetch, which can
    take minutes at boot, before anything changes provisioning or the
    protection switch."""
    now_mode, now_card = interface_mode()
    if (now_mode, now_card) != (mode, card) or card_identity(card) != ident:
        raise Retry(f"the speaker card changed during the fetch (was {mode} on card {card} {ident}, now "
                    f"{now_mode} on card {now_card} {card_identity(now_card) if now_card else None}): "
                    "nothing changed, try again")


def wait_interface_mode(wait, dt=None, sleep=time.sleep, clock=time.monotonic):
    """interface_mode(), waiting up to `wait` seconds for the speaker card
    while the device tree describes one. A tree with no speaker card is a
    retry at once with the reason: this kernel's device tree lacks the
    speaker nodes, so nothing can register until a kernel/DTB update that
    has them."""
    deadline = clock() + wait
    while True:
        mode, card = interface_mode()
        if mode != "none":
            return mode, card
        if unit.dt_speaker_card(dt) is False:
            raise Retry("the device tree describes no speaker card (no enabled apple,macaudio node with a "
                        "Speaker link): this kernel's device tree lacks the speaker amp nodes; the speakers "
                        "stay at the kernel's own limit until a kernel and boot update that carries them")
        if clock() >= deadline:
            raise Retry(f"no speaker card yet (waited {wait:.0f} s)")
        sleep(0.5)


def interface_get(card):
    """The kernel's current speaker protection interface (IF_V2, IF_LEGACY)
    on a dual kernel, or None when it cannot be read."""
    r = run(["amixer", "-c", card, "cget", f"name={INTERFACE_CTL}"])
    items = dict(re.findall(r"Item #(\d+) '([^']*)'", r.stdout))
    m = re.search(r"(?m)^\s*: values=(\d+)", r.stdout)
    return items.get(m.group(1)) if r.returncode == 0 and m else None


def interface_set(card, value):
    r = run(["amixer", "-c", card, "-q", "cset", f"name={INTERFACE_CTL}", value])
    if r.returncode != 0:
        raise Refused(f"the kernel refused the interface switch to {value!r}: {r.stderr.strip()}")


def systemctl(*args):
    return run(["systemctl", *args])


def starts_at_boot(unit):
    """Whether the boot starts `unit`: it is enabled (a link in /etc, from
    `systemctl enable` or a preset), or a target pulls it in through a link the
    package ships in /usr/lib (Arch: pacman applies no presets, and
    `systemctl is-enabled` says "disabled" there). Never a masked unit. The
    same test as speakerguardd-provision's."""
    if systemctl("is-enabled", "-q", unit).returncode == 0:
        return True
    r = systemctl("show", "--property=LoadState", "--property=UnitFileState",
                  "--property=WantedBy", "--property=RequiredBy", unit)
    if r.returncode != 0:
        return False
    props = dict(line.split("=", 1) for line in r.stdout.splitlines() if "=" in line)
    if props.get("LoadState") != "loaded" or props.get("UnitFileState", "").startswith("masked"):
        return False
    return bool(props.get("WantedBy", "").split() or props.get("RequiredBy", "").split())


def v2_unit():
    """The v2 supervisor's unit: the distributions name it
    speakerguardd-v2.service beside speakerguardd 0.1; upstream names it
    speakerguardd.service."""
    for u in V2_UNITS:
        r = systemctl("show", "-p", "ExecStart", "--value", u)
        if r.returncode == 0 and "speakerguardd-v2" in r.stdout:
            return u
    return V2_UNITS[0]


def legacy_units():
    v2 = v2_unit()
    return [u for u in ("speakersafetyd.service", "speakerguardd.service") if u != v2]


def graph_unit():
    return "speakerguardd-v2-graph.service" if v2_unit() == "speakerguardd-v2.service" else "speakerguardd-graph.service"


def graph_activate():
    p = tool("speakerguardd-provision")
    if not p:
        return
    r = run_v2([p, "activate", "--graph-unit", graph_unit()])
    sys.stderr.write(r.stderr[-1000:])
    if r.returncode != 0:
        raise Refused(f"speakerguardd v2 could not generate this Mac's speaker graph (exit {r.returncode})")


def graph_deactivate():
    """v2's session graph goes too, so the seat user's running session plays
    through the distribution's speaker graph again in this boot."""
    p = tool("speakerguardd-provision")
    if not p or "deactivate" not in run([p, "--help"]).stdout:
        return
    r = run_v2([p, "deactivate", "--graph-unit", graph_unit()])
    sys.stderr.write(r.stderr[-1000:])


def _session(verb, limit):
    p = tool("speakerguardd-provision")
    if not p or verb not in run([p, "--help"]).stdout:
        return None
    argv = [p, verb] + (["--graph-unit", graph_unit()] if verb == "session-start" else [])
    try:
        r = run_v2(argv, timeout=limit)
    except subprocess.TimeoutExpired:
        say(f"speakerguardd-provision {verb} did not finish within {limit} s")
        return False
    sys.stderr.write(r.stderr[-2000:])
    return r.returncode == 0


def session_stop():
    """The seat user's PipeWire, WirePlumber and pipewire-pulse stop before
    the speaker interface switches. A unit that will not stop even after
    SIGKILL refuses the switch."""
    if _session("session-stop", SESSION_STOP_LIMIT_S) is False:
        raise Refused("the seat user's audio session did not stop: the legacy protection keeps the speakers")


def session_start():
    """The session session_stop stopped starts again (on whatever graph is
    in place now). Never fails: a session that does not come back only
    means no sound until the user logs in again."""
    _session("session-start", SESSION_START_LIMIT_S)


def legacy_wanted():
    """The older daemons the packaged udev rules start for this Mac's
    speaker sense PCMs (SYSTEMD_WANTS), exactly as a boot would: speakersafetyd
    on most Macs, speakerguardd 0.1 where its own rules ask for it (the
    J700). Never a unit nothing asks for."""
    want = []
    for dev in sorted(glob.glob("/dev/snd/pcmC*D*c")):
        r = run(["udevadm", "info", "--query=property", dev])
        for line in r.stdout.splitlines():
            if line.startswith("SYSTEMD_WANTS="):
                for u in line.split("=", 1)[1].split():
                    u = u if u.endswith(".service") else u + ".service"
                    if u in legacy_units() and u not in want:
                        want.append(u)
    return want


def pcm_states(card):
    """The state of each substream of the speaker card, playback and
    capture (the sense PCM), from /proc/asound (closed, SETUP, PREPARED,
    RUNNING, ...)."""
    out = {}
    for f in sorted(glob.glob(f"/proc/asound/{card}/pcm*[pc]/sub*/status")):
        t = read_text(f) or ""
        m = re.search(r"(?m)^state:\s*(\S+)", t)
        out[f] = m.group(1) if m else t.strip()
    return out


def pcm_settle(card, wait=PCM_SETTLE_S, quiet=PCM_QUIET_S):
    """Waits until the speaker card's PCMs keep their state for
    `quiet` seconds, at most `wait`. False on a timeout; the caller goes on."""
    end = time.monotonic() + wait
    last, since = None, time.monotonic()
    while True:
        cur, now = pcm_states(card), time.monotonic()
        if cur != last:
            last, since = cur, now
        elif now - since >= quiet:
            return True
        if now >= end:
            return False
        time.sleep(0.1)


def legacy_start():
    """Puts the legacy daemon back, the one the packaged udev rules start
    for the speaker sense PCM; nothing else (on a Mac that is not a J700,
    speakerguardd 0.1 is installed but not wanted, and fails 78/CONFIG)."""
    run(["udevadm", "trigger", "--subsystem-match=sound", "--action=change"])
    run(["udevadm", "settle", "--timeout=10"])
    units = legacy_wanted()
    if not units and systemctl("cat", "speakersafetyd.service").returncode == 0:
        # rules without SYSTEMD_WANTS on the PCM: the distributions' default
        units = ["speakersafetyd.service"]
    for u in units:
        systemctl("reset-failed", u)
        systemctl("start", "--no-block", u)


def legacy_stop():
    for u in legacy_units():
        systemctl("stop", u)


def wait_v2_granted(since, wait=GRANT_WAIT_S):
    end = time.monotonic() + wait
    while time.monotonic() < end:
        if systemctl("is-active", "-q", v2_unit()).returncode == 0:
            k = run(["journalctl", "-k", "--since", since, "--no-pager", "-o", "cat"])
            u = run(["journalctl", "-u", v2_unit(), "--since", since, "--no-pager", "-o", "cat"])
            # with nothing playing the supervisor only reports that it
            # would grant ("protection ready"): it grants while a stream plays
            if re.search(r"protection granted|protection ready|granted", k.stdout + u.stdout, re.I):
                return True
        time.sleep(0.5)
    return False


# --- commands ----------------------------------------------------------------

@contextlib.contextmanager
def locked():
    fd = os.open(LOCK, os.O_RDWR | os.O_CREAT, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise Retry("another aurora-speaker-upgrade is running") from None
    try:
        yield
    finally:
        os.close(fd)


def cmd_check(a):
    """A dry run: collect, compile and self-check this Mac's tuning in the
    /run scratch only. Nothing is stored and no configuration is written.
    On a dual kernel, where no hardware record of this kernel exists before
    a switch, the silent switch test runs into the private root: the older
    daemon is paused and the interface lent to the test for a few seconds,
    then both are put back. When speakerguardd v2 already owns the speakers
    in this boot, the test does not run and the check cannot run (2): a dry
    run never takes the speakers from v2. Exit codes are the self-check's: 0 pass,
    1 refused, 2 cannot run, 3 error."""
    out = {"result": "error", "kernel": os.uname().release, "steps": []}

    def step(name, result, detail=""):
        out["steps"].append({"name": name, "result": result, "detail": detail})
        say(f"check: {name}: {result}{': ' + detail if detail else ''}")

    def finish(result, code, reason=""):
        out["result"], out["reason"] = result, reason
        if a.json:
            with open(a.json, "w") as f:
                json.dump(out, f, indent=2, sort_keys=True)
                f.write("\n")
        say(f"check: {result.upper()}" + (f": {reason}" if reason else ""))
        return code

    try:
        identity = collect.Identity(a.adt)
        out.update(board=identity.board, acoustic_id=identity.acoustic, unit_bound=identity.matches_dt())
        if not out["unit_bound"]:
            step("identity", "refused", "the Linux device tree and the ADT name different units")
            return finish("refused", EXIT_REFUSED, "the device tree and the ADT name different units")
        step("identity", "pass", f"{identity.board}, AID{identity.acoustic}")
        mode, card = interface_mode()
        out["kernel_interface"] = mode
        try:
            b, prov = acquire(identity, a)
        except Refused as e:
            step("acquire", "refused", str(e))
            return finish("refused", EXIT_REFUSED, str(e).splitlines()[0])
        out.update({k: prov.get(k) for k in ("source", "aid", "macos_build", "speaker_id", "speaker_id_from",
                                              "lsm2_fill_from")},
                   bundle_digest=digest(b))
        step("acquire", "pass", prov["source"])
        os.makedirs(a.scratch, mode=0o700, exist_ok=True)
        work = tempfile.mkdtemp(prefix="check-", dir=a.scratch)
        bdir = os.path.join(work, "bundle")
        collect.write_bundle(b, bdir)
        prov_tool, sc_tool = tool("speakerguardd-provision"), tool("speakerguardd-selfcheck")
        if not prov_tool or not sc_tool:
            step("compile", "cannot-run", "speakerguardd v2 is not installed")
            return finish("cannot-run", EXIT_RETRY, "speakerguardd v2 is not installed: only the bundle was checked")
        root = os.path.join(work, "root")
        r = run([prov_tool, "run", "--force", "--bundle", bdir, "--root", root])
        params = os.path.join(root, PARAMS.lstrip("/"))
        if r.returncode != 0 or not os.path.isfile(params):
            step("compile", "refused", r.stderr.strip()[-800:])
            return finish("refused", EXIT_REFUSED, "speakerguardd v2 could not compile this Mac's tuning")
        import hashlib
        with open(params, "rb") as f:
            out["params_sha256"] = hashlib.sha256(f.read()).hexdigest()
        step("compile", "pass", f"blob {out['params_sha256'][:12]}")
        state = os.path.join(root, STATE_V2.lstrip("/"))
        conf = os.path.join(root, V2_CONF.lstrip("/"))
        js = os.path.join(work, "selfcheck.json")
        h = run([sc_tool, "--help"])
        if mode == "dual" and interface_get(card) == IF_V2 \
                and systemctl("is-active", "-q", v2_unit()).returncode == 0:
            # speakerguardd v2 already owns the speakers in this boot: the
            # switch test would need them, and a dry run does not take them.
            step("hardware switch test", "cannot-run", "speakerguardd v2 owns the speakers in this boot")
            return finish("cannot-run", EXIT_RETRY, "speakerguardd v2 owns the speakers in this boot; "
                          "a dry run does not take them (aurora-speaker-upgrade status shows the live setup)")
        if mode == "dual" and "--switch-interface" in h.stdout + h.stderr:
            # No hardware record of this kernel exists before a switch: make
            # one in the private root, the way 'run' does. The silent switch
            # test needs the speakers for a few seconds: the older daemon is
            # paused and the interface lent to the test, then both are put
            # back; nothing is stored outside /run.
            res, detail = check_hardware(sc_tool, card, params, state, conf)
            step("hardware switch test", res, detail)
            if res != "pass":
                code = EXIT_RETRY if res == "cannot-run" else EXIT_REFUSED
                return finish(res, code, detail.splitlines()[0] if detail else "the silent switch test did not pass")
        graph = os.path.join(work, "graph")
        if "--conf" in h.stdout + h.stderr:
            run([sc_tool, "generate", "--state", state, "--pipewire-dir", os.path.join(graph, "pipewire"),
                 "--wireplumber-dir", os.path.join(graph, "wireplumber")])
        if re.search(r"\bverify\b", h.stdout + h.stderr):
            extra = ["--conf", conf, "--graph-dir", graph] if "--conf" in h.stdout + h.stderr else []
            r = run([sc_tool, "verify", "--params", params, "--state", state, "--json", js, *extra])
            code = r.returncode if r.returncode in (0, 1, 2, 3) else EXIT_ERROR
            with contextlib.suppress(OSError, ValueError):
                with open(js) as f:
                    out["selfcheck"] = json.load(f)
            res = {0: "pass", 1: "refused", 2: "cannot-run", 3: "error"}[code]
            step("selfcheck verify", res, r.stderr.strip()[-800:])
            return finish(res, code, "" if code == 0 else "the speaker self-check did not pass")
        # Until SGFIX's verify lands: a passing hardware record of this
        # kernel for exactly this blob (the live one) counts; anything else
        # needs the hardware switch test, which a dry run may not take.
        r = run([sc_tool, "check", "--params", params])
        if r.returncode == 0:
            step("selfcheck", "pass", "a passing self-check record binds this kernel to this exact blob")
            return finish("pass", EXIT_OK)
        step("selfcheck", "cannot-run", "no record for this blob; the silent switch test needs "
             "speakerguardd-selfcheck verify (SGFIX) or the speakers")
        return finish("cannot-run", EXIT_RETRY, "compiled; the hardware self-check could not run in a dry run")
    except collect.CollectError as e:
        step("acquire", "refused", str(e))
        return finish("refused", EXIT_REFUSED, str(e))
    except Exception as e:
        return finish("error", EXIT_ERROR, f"{type(e).__name__}: {e}")


def check_hardware(sc_tool, card, params, state, conf):
    """--check on a dual kernel: the silent switch test into the private
    state, with the speakers lent to it and given back. ('pass'|'refused'|
    'cannot-run', detail)."""
    before = interface_get(card) or IF_LEGACY
    paused = [u for u in legacy_units() if systemctl("is-active", "-q", u).returncode == 0]
    for u in paused:
        systemctl("stop", u)
    stopped = False
    try:
        session_stop()
        stopped = True
        r = run([sc_tool, "run", "--switch-interface", "--force", "--params", params, "--state", state,
                 "--", *v2_tuning_args(conf)])
    finally:
        # Back to the interface it found, not always the legacy one.
        with contextlib.suppress(Refused):
            interface_set(card, before)
        if stopped:
            session_start()
        for u in paused:
            systemctl("reset-failed", u)
            systemctl("start", "--no-block", u)
    if r.returncode == 0:
        return "pass", "silent switch test passed on this kernel (private record)"
    tail = r.stderr.strip()[-800:]
    if re.search(r"stayed in use|owned by another process|no ALSA card", tail):
        return "cannot-run", tail
    return "refused", tail


def cmd_collect(a):
    identity = collect.Identity(a.adt)
    b, prov = acquire(identity, a)
    collect.write_bundle(b, a.out)
    with open(os.path.join(a.out, "..", os.path.basename(a.out) + ".provenance.json"), "w") as f:
        json.dump(prov, f, indent=2, sort_keys=True)
    say(f"bundle written to {a.out}")
    return EXIT_OK


def _cmd_run(a):
    store = Store(a.store)
    identity = collect.Identity(a.adt)
    if not identity.matches_dt():
        raise Refused("the Linux device tree and the ADT name different units")
    mode, card = wait_interface_mode(CARD_WAIT_S if getattr(a, "at_boot", False) else 0)
    state = store.state()
    have = store.read_bundle()
    m = json.loads(have[audio_tuning.MANIFEST]) if have else {}
    switched_here = bool(state.get("switched") and have and m.get("unit_id") == identity.unit)
    if switched_here:
        if (mode != "dual" or read_text(store.interface) == "v2\n") \
                and (getattr(a, "at_boot", False) or v2_owns_speakers(mode, card)):
            say(f"nothing to do: speakerguardd v2 owns the speakers with {m.get('aid')} from macOS "
                f"{m.get('macos_build')} ({state.get('source')})")
            return EXIT_OK
        if not getattr(a, "at_boot", False):
            # Switched on record, but v2 does not hold the speakers in this
            # boot (for example speakerguardd-v2 was reinstalled or updated
            # since the boot, so its provisioning is gone until it runs
            # again): switch again instead of leaving them at the floor.
            say("switched on record, but speakerguardd v2 does not hold the speakers in this boot: switching again")
    if getattr(a, "at_boot", False) and not switched_here and not playback_proof(identity.acoustic):
        # The boot unit moves a speaker only once protected playback on it
        # is on record; until then the speakers stay as they are (the legacy
        # protection, or what the installer set up) and nothing is collected.
        say(f"AID{identity.acoustic}: no protected playback row (loud step, thermal ramp) is on record "
            "for this speaker yet, so the boot unit leaves the speakers as they are "
            "('aurora-speaker-upgrade run' switches by hand)")
        return EXIT_OK
    ident = card_identity(card)
    b, prov = acquire_waiting(identity, a, NET_WAIT_S if getattr(a, "at_boot", False) else 0)
    same_speaker_card(mode, card, ident)
    store.ensure()
    if not have or digest(have) != digest(b):
        store.put_bundle(b)
        say(f"stored in {store.bundle} (root only)")
    if "provision_conf_before" not in state:
        state["provision_conf_before"] = read_text(PROVISION_CONF)
    state.update(prov, digest=digest(b), collected=time.strftime("%Y-%m-%dT%H:%M:%S%z"), switched=False)
    store.save(state)
    if a.no_switch:
        say("collected; --no-switch: the legacy protection keeps the speakers")
        return EXIT_OK
    if mode == "legacy-only":
        raise Refused("this kernel has only the legacy speaker interface, so no switch is possible: the "
                      "legacy protection keeps the speakers (the bundle is stored; run again on a newer kernel)")
    write_file(PROVISION_CONF, provision_conf_with(read_text(PROVISION_CONF), store.bundle))
    since = time.strftime("%Y-%m-%d %H:%M:%S")
    try:
        if mode == "dual":
            legacy_stop()
        provision(store, a)
        if mode == "dual":
            # No session may run across the switch: it would keep starting
            # the speaker PCM the lease refuses. It starts again on v2's
            # graph once the self-check made it (selfcheck), or on the
            # distribution's on a rollback (back_to_legacy).
            session_stop()
            # The self-check takes a v2 lease itself: v2 must own the
            # interface while it runs; the kernel falls back to legacy (from
            # the floor) 30 s after the last owner closes, and speakerguardd's
            # ExecStartPre switches it again when the supervisor starts.
            interface_set(card, IF_V2)
        selfcheck(a, mode)
    except (Refused, Retry):
        back_to_legacy(store, state, mode, card)
        raise
    write_file(store.interface, "v2\n", 0o600)
    state["switched"] = True
    store.save(state)
    systemctl("daemon-reload")
    systemctl("reset-failed", v2_unit())
    r = systemctl("restart", v2_unit())
    if r.returncode != 0 or not wait_v2_granted(since):
        back_to_legacy(store, state, mode, card)
        raise Refused("speakerguardd v2 did not take the speakers within "
                      f"{GRANT_WAIT_S} s: back on the legacy protection")
    say(f"done: speakerguardd v2 owns the speakers ({prov.get('aid')}, macOS {prov.get('macos_build')}, "
        f"{prov.get('source')}); the legacy stack stays installed (aurora-speaker-upgrade --revert)")
    return EXIT_OK


def refusal_key(identity):
    """What a refusal depends on: this unit, the running kernel, and the code
    and data that judged it (this tool, v2's provisioning and self-check, the
    parity attestation, the gate data with its playback rows). A new kernel,
    a v2 update, a new attestation or a new playback row makes the boot unit
    try again; nothing else does."""
    h = hashlib.sha256()
    h.update(f"{identity.unit}\0{os.uname().release}\0".encode())
    for p in [os.path.abspath(__file__), tool("speakerguardd-provision"), tool("speakerguardd-selfcheck"), *PARITY, GATE,
              ipsw.CATALOG]:
        try:
            with open(p or "", "rb") as f:
                h.update(hashlib.sha256(f.read()).digest())
        except OSError:
            h.update(b"-")
    return h.hexdigest()


def cmd_run(a):
    """run, plus the boot unit's refusal record: a Mac the self-check (or the
    collection) refused is not tried again at every boot, which would stop
    and restart the legacy protection each time for the same answer (a new
    pin of restore images counts as new data). The record lives in the
    store and is keyed by refusal_key(); `run` by hand
    always tries again."""
    store = Store(a.store)
    identity = collect.Identity(a.adt)
    record = os.path.join(store.root, "refused.json")
    key = refusal_key(identity)
    if getattr(a, "at_boot", False):
        try:
            with open(record) as f:
                old = json.load(f)
        except (OSError, ValueError):
            old = {}
        if old.get("key") == key:
            say(f"refused before for this kernel and this speakerguardd ({old.get('when')}): {old.get('reason')}; "
                "the legacy protection keeps the speakers (run 'aurora-speaker-upgrade run' to try again)")
            return EXIT_OK
    try:
        code = _cmd_run(a)
    except (Refused, collect.CollectError) as e:
        with contextlib.suppress(Exception):
            store.ensure()
            fd = os.open(record, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, "w") as f:
                json.dump({"key": key, "reason": str(e).splitlines()[0],
                           "when": time.strftime("%Y-%m-%dT%H:%M:%S%z")}, f, indent=2, sort_keys=True)
                f.write("\n")
        raise
    if code == EXIT_OK:
        with contextlib.suppress(FileNotFoundError):
            os.unlink(record)
    return code


def v2_owns_speakers(mode, card):
    """In this boot: the v2 supervisor runs and, on a dual kernel, the
    kernel's interface is v2's (the speakers are not left at the floor)."""
    if systemctl("is-active", "-q", v2_unit()).returncode != 0:
        return False
    return mode != "dual" or interface_get(card) == IF_V2


def back_to_legacy(store, state, mode, card):
    with contextlib.suppress(FileNotFoundError):
        os.unlink(store.interface)
    before = state.get("provision_conf_before")
    if before is None:
        with contextlib.suppress(FileNotFoundError):
            os.unlink(PROVISION_CONF)
    elif "provision_conf_before" in state:
        write_file(PROVISION_CONF, before)
    state["switched"] = False
    store.save(state)
    if mode == "dual":
        # The session first (it would keep starting the speaker PCM once v2
        # releases it), then v2, then the interface; the session starts
        # again on the distribution's graph. After a session_stop on the
        # way up this stops nothing and starts what ran before.
        with contextlib.suppress(Refused):
            session_stop()
        systemctl("stop", v2_unit())
        with contextlib.suppress(Refused):
            interface_set(card, IF_LEGACY)
        graph_deactivate()
        session_start()
        # speakersafetyd reads the sense PCM's rate range and then opens it
        # at the lowest rate; when the restarted session's WirePlumber opened
        # the speaker PCM at 48 kHz in between, the open failed on the I2S
        # rate symmetry (EINVAL) and the daemon panicked and restarted. Wait
        # for the session to finish opening the speaker first; its ALSA
        # monitor can also hold the sense PCM for a moment while it starts
        # (EBUSY), so every PCM of the card counts. (Starting the
        # daemon before the restart does not work: v2's graph still holds
        # the sense PCM, EBUSY.)
        if card:
            pcm_settle(card)
        legacy_start()
    else:
        # A fleet kernel (v2 interface only): the stack that ran before
        # (speakerguardd v2 with the installer's bundle, or nothing) again.
        if starts_at_boot(v2_unit()):
            systemctl("restart", v2_unit())


def cmd_revert(a):
    store = Store(a.store)
    state = store.state()
    if not state and not os.path.exists(store.root):
        say("nothing to revert: this Mac was never upgraded")
        return EXIT_OK
    mode, card = interface_mode()
    back_to_legacy(store, state, mode, card)
    if a.purge:
        collect.rmtree(store.root)
        say(f"reverted; {store.root} removed")
    else:
        state.pop("provision_conf_before", None)
        store.save(state)
        say("reverted: the legacy protection owns the speakers (the collected bundle is kept; --purge removes it)")
    return EXIT_OK


def cmd_status(a):
    store = Store(a.store)
    st = store.state()
    mode, card = interface_mode()
    b = store.read_bundle()
    m = json.loads(b[audio_tuning.MANIFEST]) if b else {}
    fill_needed = bool(b) and collect.lsm2_fill.needs_fill(b) and m.get("lsm2_param_fill") is None
    print(json.dumps({"lsm2_fill": "needed, not present" if fill_needed
                      else "present" if m.get("lsm2_param_fill") is not None else "not required",
                      "lsm2_fill_next": "the installer's readback on this Mac's macOS, from the pinned build"
                      if fill_needed else None,
                      "kernel_interface": mode, "switched": bool(st.get("switched")),
                      "setting": (read_text(store.interface) or "").strip() or None,
                      "source": st.get("source"), "aid": m.get("aid"), "macos_build": m.get("macos_build"),
                      "speaker_id": m.get("speaker_id"), "bundle_digest": st.get("digest"),
                      "boot_migration": boot_migration(a)}, indent=2))
    return EXIT_OK


def boot_migration(a):
    """status: whether the boot unit would move this speaker (its playback
    rows are on record) or leaves it to a run by hand."""
    try:
        acoustic = collect.Identity(getattr(a, "adt", None)).acoustic
    except collect.CollectError:
        return None
    return "playback rows on record" if playback_proof(acoustic) else "by hand only (no playback rows on record)"


def cmd_interface_apply(a):
    """Puts the kernel on the v2 interface when this Mac was upgraded; a
    no-op on kernels without the switch. The supervisor unit does the same
    itself (ExecStartPre=speakerguardd-gate interface); this command stays
    for scripts that call it."""
    store = Store(a.store)
    mode, card = interface_mode()
    if mode != "dual":
        return EXIT_OK
    if read_text(store.interface) == "v2\n":
        interface_set(card, IF_V2)
        say("kernel speaker interface: speakerguardd v2")
    return EXIT_OK


# Accepted and ignored: the boot unit drop-ins and scripts that passed it
# for the earlier confirmation step keep working.
ACK_FLAG = "--i-understand-preview"


def main(argv=None):
    ap = argparse.ArgumentParser(prog="aurora-speaker-upgrade", description=__doc__.split("\n\n")[0])
    ap.add_argument("cmd", nargs="?", default="run",
                    choices=["run", "check", "collect", "status", "interface-apply", "revert"])
    ap.add_argument("--revert", action="store_true", help="go back to the legacy protection")
    ap.add_argument("--check", action="store_true",
                    help="dry run: collect, compile and self-check in /run only; nothing stored or switched")
    ap.add_argument("--json", help="--check: write the result here")
    ap.add_argument("--purge", action="store_true", help="with --revert: remove the stored bundle")
    ap.add_argument("--ipsw", help="read the pinned restore image from this address (a mirror); "
                    "its build must still be a pinned one")
    ap.add_argument("--no-switch", action="store_true", help="collect and store only")
    ap.add_argument("--at-boot", action="store_true",
                    help="run: skip a Mac refused before for the same kernel and speakerguardd (the boot unit)")
    ap.add_argument("--force", action="store_true", help="recompile even if bound")
    ap.add_argument("--out", help="collect: the directory to write the bundle to (must not exist)")
    ap.add_argument("--store", default=STORE)
    ap.add_argument("--adt", help="the ADT (default: the boot chain's 'adt' MTD partition)")
    ap.add_argument("--installer-dir", action="append", dest="installer_dirs")
    ap.add_argument("--chosen", default=collect.CHOSEN_SPID)
    ap.add_argument("--scratch", default=None)
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument(ACK_FLAG, dest="i_understand_preview", action="store_true",
                    help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO if a.verbose else logging.WARNING,
                        format="aurora-speaker-upgrade: %(message)s")
    if a.revert:
        a.cmd = "revert"
    if a.check:
        a.cmd = "check"
    a.installer_dirs = a.installer_dirs or list(collect.INSTALLER_BUNDLES)
    # Staging (about 25 MB) in a root-only directory on /run, never in the store.
    a.scratch = a.scratch or "/run/aurora-speaker-upgrade.scratch"
    if os.geteuid() != 0:
        say("run me as root")
        return EXIT_RETRY
    if a.cmd == "collect" and not a.out:
        ap.error("collect needs --out")
    os.umask(0o077)
    cmds = {"run": cmd_run, "check": cmd_check, "collect": cmd_collect, "status": cmd_status,
            "interface-apply": cmd_interface_apply, "revert": cmd_revert}
    try:
        if a.cmd in ("status", "interface-apply"):
            # Never behind the lock: status only reads, and interface-apply
            # only repeats the gate's switch; neither waits for a run.
            return cmds[a.cmd](a)
        with locked():
            return cmds[a.cmd](a)
    except Refused as e:
        say(f"REFUSED: {e}")
        return EXIT_REFUSED
    except Retry as e:
        say(f"not now: {e}")
        return EXIT_RETRY
    except collect.CollectError as e:
        say(f"REFUSED: {e}")
        return EXIT_REFUSED
    except Exception as e:
        say(f"internal error ({type(e).__name__}): {e}; nothing switched")
        return EXIT_ERROR
    finally:
        if a.cmd in ("run", "collect", "check"):
            collect.rmtree(a.scratch)
