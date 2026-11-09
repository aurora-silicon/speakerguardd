# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The Aurora Silicon Contributors
"""aurora-speaker-upgrade: move this Mac's speakers from the legacy
protection (speakersafetyd, speakerguardd 0.1) to speakerguardd v2, with
this Mac's own tuning, and back.

    aurora-speaker-upgrade [run] [--i-understand-preview] [--route auto|0|1|2|3] [--ipsw URL]
                                 [--macos-build B] [--collection TARBALL] [--no-switch]
    aurora-speaker-upgrade --revert [--purge]
    aurora-speaker-upgrade --check [--json FILE]                (dry run: /run only, nothing stored or switched)
    aurora-speaker-upgrade status
    aurora-speaker-upgrade collect --out DIR [--route ...]     (no switch; for checks)
    aurora-speaker-upgrade interface-apply                     (speakerguardd.service ExecStartPre)

1. The tuning bundle, the installer's own, first found of:
   0. an installer bundle already on this Mac, bound to this unit;
   1. the installed macOS System volume (read only from Linux; it is not
      encrypted, FileVault or not) with the ADT the boot chain exposes;
   2. Apple's restore image of that macOS build, by HTTP ranges;
   3. a collection made on this Mac's macOS by the installer
      (`--collection`, see `aurora-speaker-upgrade guide`).
   The speaker ID comes from the boot chain (/chosen/asahi,speaker-id) or
   from this unit's installer bundle; a Mac whose tuning selects by speaker
   ID without either needs route 3. Nothing is guessed.
2. It is stored root-only in /var/lib/aurora-speaker-upgrade (0700), never
   packaged or sent anywhere.
3. speakerguardd v2 compiles it and its self-check proves it, silently.
4. Only on a pass, the kernel's speaker protection interface switches to v2.
   The legacy stack stays installed; any failure leaves (or puts) it in
   charge, and `--revert` goes back to it.

speakerguardd v2 is a preview: it does not fully match macOS yet, and the
legacy protection is the supported default. A run that can switch says so
first; interactively it asks, otherwise it needs --i-understand-preview
(without it: exit 2, nothing changed).
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
import tarfile
import tempfile
import time

from . import collect, ipsw
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
PARITY = ("/usr/share/speakerguardd/parity.json",)
MARK = "# aurora-speaker-upgrade: the bundle collected for this Mac"

EXIT_OK, EXIT_REFUSED, EXIT_RETRY, EXIT_ERROR = 0, 1, 2, 3


class Refused(Exception):
    """Stay on (or go back to) the legacy stack."""


class Retry(Exception):
    """Cannot run now; nothing changed."""


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
    """(speaker ID or None, known?, where from)."""
    v = collect.chosen_speaker_id(chosen)
    if v is not None:
        return v, True, "the boot chain (/chosen/asahi,speaker-id)"
    for d in dirs:
        try:
            with open(os.path.join(d, audio_tuning.MANIFEST)) as f:
                m = json.load(f)
        except (OSError, ValueError):
            continue
        if m.get("unit_id") == identity.unit and identity.unit and collect.valid_speaker_id(m.get("speaker_id")):
            return m["speaker_id"], True, f"this unit's installer bundle ({d})"
    return None, False, "none"


def route0(identity, a):
    d, b = collect.installer_bundle(identity, a.installer_dirs)
    if d is None:
        raise collect.CollectError(b)
    return b, {"route": 0, "source": d}


def route1(identity, a, spid, known, scratch):
    vols = collect.system_volumes(identity.acoustic, a.devices)
    if not vols:
        raise collect.CollectError("no readable macOS System volume with this Mac's tuning")
    builds = sorted({b for b, _, _ in vols}, key=collect.build_key)
    build, dev, vol = max(vols, key=lambda x: collect.build_key(x[0]))
    if len(builds) > 1:
        say(f"several macOS installs ({', '.join(builds)}): using the newest, {build} on {dev}")
    st = tempfile.mkdtemp(prefix="stage-", dir=scratch)
    collect.stage(vol, identity.acoustic, st)
    b, got = collect.collect(st, identity, spid, known)
    return b, {"route": 1, "source": f"{dev} ({vol.name})", "macos_build": got}


def build_hint(identity, a):
    """The macOS build to fetch: --macos-build, else the installed macOS's
    Preboot record, else this unit's installer bundle."""
    if a.macos_build:
        return a.macos_build, "--macos-build"
    for dev in a.devices if a.devices is not None else collect.apfs_partitions():
        b = collect.preboot_build(dev)
        if b:
            return b, f"the Preboot volume of {dev}"
    for d in a.installer_dirs:
        try:
            with open(os.path.join(d, audio_tuning.MANIFEST)) as f:
                m = json.load(f)
            if m.get("unit_id") == identity.unit and m.get("macos_build"):
                return m["macos_build"], f"this unit's installer bundle ({d})"
        except (OSError, ValueError):
            pass
    raise collect.CollectError("the installed macOS build is unknown (pass --macos-build)")


def route2(identity, a, spid, known, scratch):
    build, why = build_hint(identity, a)
    url = ipsw.find_url(build, a.ipsw)
    say(f"route 2: macOS {build} ({why}) from Apple's restore image, by ranges")
    vol, rng, got = ipsw.open_system(url, identity.board, None if a.ipsw else build)
    st = tempfile.mkdtemp(prefix="stage-", dir=scratch)
    collect.stage(vol, identity.acoustic, st)
    b, sysbuild = collect.collect(st, identity, spid, known)
    say(f"route 2: {rng.fetched >> 20} MiB fetched from Apple")
    return b, {"route": 2, "source": url.rsplit("/", 1)[-1], "macos_build": sysbuild}


def route3(identity, a):
    if not a.collection:
        raise collect.CollectError("no macOS collection given (see `aurora-speaker-upgrade guide`)")
    with tempfile.TemporaryDirectory(dir=a.scratch) as t:
        with tarfile.open(a.collection) as tf:
            for m in tf.getmembers():
                if not (m.isfile() or m.isdir()) or m.name.startswith("/") or ".." in m.name.split("/"):
                    raise collect.CollectError(f"unexpected member in the collection: {m.name!r}")
            tf.extractall(t, filter="data")
        roots = [os.path.dirname(os.path.join(dp, audio_tuning.FALLBACK_DIR))
                 for dp, dn, _ in os.walk(t) if audio_tuning.FALLBACK_DIR in dn]
        if len(roots) != 1:
            raise collect.CollectError("the collection holds no single audio-tuning/ bundle")
        c = audio_tuning.AudioTuningFWCollection.__new__(audio_tuning.AudioTuningFWCollection)
        c.fwfiles, c.bundle = [], {}
        c.update(os.path.join(roots[0], audio_tuning.FALLBACK_DIR))
    m = json.loads(c.bundle[audio_tuning.MANIFEST])
    if m.get("unit_id") != identity.unit or m.get("board") != identity.board:
        raise collect.CollectError("the collection was made on another Mac")
    return c.bundle, {"route": 3, "source": os.path.basename(a.collection)}


def acquire(identity, a):
    """(bundle, provenance). Tries the routes in order; refuses when none
    gives a bundle."""
    spid, known, spid_from = speaker_id_source(identity, a.chosen, a.installer_dirs)
    routes = [a.route] if a.route != "auto" else ["0", "1", "2", "3"]
    reasons = []
    os.makedirs(a.scratch, mode=0o700, exist_ok=True)
    scratch = tempfile.mkdtemp(prefix="scratch-", dir=a.scratch)
    try:
        for r in routes:
            try:
                if r == "0":
                    b, prov = route0(identity, a)
                elif r == "1":
                    b, prov = route1(identity, a, spid, known, scratch)
                elif r == "2":
                    b, prov = route2(identity, a, spid, known, scratch)
                else:
                    b, prov = route3(identity, a)
            except collect.SpeakerIdUnknown as e:
                reasons.append(f"route {r}: {e}")
                break  # every other route reads the same tuning: go to route 3
            except Exception as e:
                reasons.append(f"route {r}: {e}")
                continue
            m = json.loads(b[audio_tuning.MANIFEST])
            if m.get("unit_id") != identity.unit:
                reasons.append(f"route {r}: the bundle does not name this unit")
                continue
            prov.update(speaker_id=m.get("speaker_id"), speaker_id_from=spid_from if r in "12" else "the bundle",
                        aid=m.get("aid"), macos_build=m.get("macos_build"))
            say(f"tuning {m.get('aid')} from macOS {m.get('macos_build')} via route {r} ({prov['source']})")
            return b, prov
    finally:
        collect.rmtree(scratch)
    if a.route in ("auto", "3") and a.collection is None:
        reasons.append("route 3: boot macOS once and collect (aurora-speaker-upgrade guide)")
    raise Refused("no speaker tuning for this Mac:\n  " + "\n  ".join(reasons))


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


def write_file(path, text, mode=0o644):
    os.makedirs(os.path.dirname(path), exist_ok=True)
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
        # boot), so verify's graph step sees it.
        graph_activate()
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
            if re.search(r"protection granted|granted", k.stdout + u.stdout, re.I):
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
        out.update({k: prov.get(k) for k in ("route", "source", "aid", "macos_build", "speaker_id", "speaker_id_from")},
                   bundle_digest=digest(b))
        step("acquire", "pass", f"route {prov['route']} ({prov['source']})")
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
    try:
        r = run([sc_tool, "run", "--switch-interface", "--force", "--params", params, "--state", state,
                 "--", *v2_tuning_args(conf)])
    finally:
        # Back to the interface it found, not always the legacy one.
        with contextlib.suppress(Refused):
            interface_set(card, before)
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
    mode, card = interface_mode()
    if mode == "none":
        raise Retry("no speaker card yet")
    state = store.state()
    have = store.read_bundle()
    m = json.loads(have[audio_tuning.MANIFEST]) if have else {}
    if state.get("switched") and have and m.get("unit_id") == identity.unit:
        if mode != "dual" or read_text(store.interface) == "v2\n":
            say(f"nothing to do: speakerguardd v2 owns the speakers with {m.get('aid')} from macOS "
                f"{m.get('macos_build')} (route {state.get('route')})")
            return EXIT_OK
    b, prov = acquire(identity, a)
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
        f"route {prov['route']}); the legacy stack stays installed (aurora-speaker-upgrade --revert)")
    return EXIT_OK


def refusal_key(identity):
    """What a refusal depends on: this unit, the running kernel, and the code
    and data that judged it (this tool, v2's provisioning and self-check, the
    parity attestation). A new kernel, a v2 update or a new attestation makes
    the boot unit try again; nothing else does."""
    h = hashlib.sha256()
    h.update(f"{identity.unit}\0{os.uname().release}\0".encode())
    for p in [os.path.abspath(__file__), tool("speakerguardd-provision"), tool("speakerguardd-selfcheck"), *PARITY]:
        try:
            with open(p or "", "rb") as f:
                h.update(hashlib.sha256(f.read()).digest())
        except OSError:
            h.update(b"-")
    return h.hexdigest()


def cmd_run(a):
    """run, plus the boot unit's refusal record: a Mac the self-check (or the
    collection) refused is not tried again at every boot, which would stop
    and restart the legacy protection each time for the same answer. The
    record lives in the store and is keyed by refusal_key(); `run` by hand
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
        systemctl("stop", v2_unit())
        with contextlib.suppress(Refused):
            interface_set(card, IF_LEGACY)
        graph_deactivate()
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
        systemctl("restart", v2_unit()) if systemctl("is-enabled", "-q", v2_unit()).returncode == 0 else None


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
    print(json.dumps({"kernel_interface": mode, "switched": bool(st.get("switched")),
                      "setting": (read_text(store.interface) or "").strip() or None,
                      "route": st.get("route"), "aid": m.get("aid"), "macos_build": m.get("macos_build"),
                      "speaker_id": m.get("speaker_id"), "bundle_digest": st.get("digest")}, indent=2))
    return EXIT_OK


def cmd_interface_apply(a):
    """speakerguardd.service ExecStartPre: puts the kernel on the v2
    interface when this Mac was upgraded; never fails the unit on kernels
    without the switch."""
    store = Store(a.store)
    mode, card = interface_mode()
    if mode != "dual":
        return EXIT_OK
    if read_text(store.interface) == "v2\n":
        interface_set(card, IF_V2)
        say("kernel speaker interface: speakerguardd v2")
    return EXIT_OK


# v2 ships opt-in while it is a preview. The same text is in
# the package descriptions and the distribution docs; keep them in step.
PREVIEW_WARNING = """speakerguardd v2 is a PREVIEW.
  - It does not fully match macOS yet.
  - The legacy protection (speakersafetyd, speakerguardd 0.1) is the supported
    default, and it stays installed.
  - To go back at any time: sudo aurora-speaker-upgrade --revert"""
ACK_FLAG = "--i-understand-preview"


def preview_ack(a, ask=None, interactive=None):
    """Before a run that can switch the speakers to v2: say plainly that v2 is
    a preview. An interactive run asks; a non-interactive one (the boot unit,
    a script) needs --i-understand-preview, and without it is "not now"
    (EXIT_RETRY): nothing is collected, stored or switched."""
    print(PREVIEW_WARNING, file=sys.stderr, flush=True)
    if a.i_understand_preview:
        return True
    if interactive is None:
        interactive = sys.stdin.isatty() and sys.stderr.isatty()
    if not interactive:
        raise Retry(f"speakerguardd v2 is a preview: a non-interactive run needs {ACK_FLAG} "
                    "(the legacy protection keeps the speakers)")
    ask = ask or (lambda q: input(q))
    try:
        answer = ask("Switch this Mac's speakers to the speakerguardd v2 preview? [y/N] ")
    except EOFError:
        answer = ""
    if answer.strip().lower() not in ("y", "yes"):
        raise Retry("not confirmed (the legacy protection keeps the speakers)")
    return True


GUIDE = """Route 3: collect this Mac's speaker tuning on its own macOS, once.
Needed only when Linux cannot read it (no readable macOS, no restore image of
its build) or when the tuning selects by speaker ID and the boot chain does
not report it.
  1. Boot macOS (hold the power button, choose Macintosh HD) and turn the
     volume to 0.
  2. In Terminal, from a checkout of github.com/aurora-silicon/aurora-installer:
       tools/speaker-tuning/collect-macos.sh
     It reads only, needs no sudo, and writes
     ~/aurora-speaker-tuning-<mac>-AID<n>.tar.gz.
  3. Copy that file to Linux (a USB stick or scp), boot Linux, then:
       sudo aurora-speaker-upgrade --collection ~/aurora-speaker-tuning-<mac>-AID<n>.tar.gz
"""


def main(argv=None):
    ap = argparse.ArgumentParser(prog="aurora-speaker-upgrade", description=__doc__.split("\n\n")[0])
    ap.add_argument("cmd", nargs="?", default="run",
                    choices=["run", "check", "collect", "status", "guide", "interface-apply", "revert"])
    ap.add_argument("--revert", action="store_true", help="go back to the legacy protection")
    ap.add_argument("--check", action="store_true",
                    help="dry run: collect, compile and self-check in /run only; nothing stored or switched")
    ap.add_argument("--json", help="--check: write the result here")
    ap.add_argument("--purge", action="store_true", help="with --revert: remove the stored bundle")
    ap.add_argument("--route", default="auto", choices=["auto", "0", "1", "2", "3"])
    ap.add_argument("--ipsw", help="route 2: this restore image (an Apple CDN URL)")
    ap.add_argument("--macos-build", help="route 2: this macOS build")
    ap.add_argument("--collection", help="route 3: the installer's macOS collection tarball")
    ap.add_argument("--no-switch", action="store_true", help="collect and store only")
    ap.add_argument("--at-boot", action="store_true",
                    help="run: skip a Mac refused before for the same kernel and speakerguardd (the boot unit)")
    ap.add_argument("--force", action="store_true", help="recompile even if bound")
    ap.add_argument("--out", help="collect: the directory to write the bundle to (must not exist)")
    ap.add_argument("--store", default=STORE)
    ap.add_argument("--adt", help="the ADT (default: the boot chain's 'adt' MTD partition)")
    ap.add_argument("--device", action="append", dest="devices", help="APFS container(s) to read (default: all)")
    ap.add_argument("--installer-dir", action="append", dest="installer_dirs")
    ap.add_argument("--chosen", default=collect.CHOSEN_SPID)
    ap.add_argument("--scratch", default=None)
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument(ACK_FLAG, dest="i_understand_preview", action="store_true",
                    help="run: accept that speakerguardd v2 is a preview (needed when not interactive)")
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
    if a.cmd == "guide":
        print(GUIDE)
        return EXIT_OK
    if os.geteuid() != 0:
        say("run me as root")
        return EXIT_RETRY
    if a.cmd == "collect" and not a.out:
        ap.error("collect needs --out")
    if a.cmd == "run" and not a.no_switch:
        try:
            preview_ack(a)
        except Retry as e:
            say(f"not now: {e}")
            return EXIT_RETRY
    os.umask(0o077)
    cmds = {"run": cmd_run, "check": cmd_check, "collect": cmd_collect, "status": cmd_status,
            "interface-apply": cmd_interface_apply, "revert": cmd_revert}
    try:
        if a.cmd in ("status", "interface-apply"):
            # Never behind the lock: run restarts speakerguardd, whose
            # ExecStartPre is interface-apply.
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
