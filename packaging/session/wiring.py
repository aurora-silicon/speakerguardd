#!/usr/bin/python3 -I
"""The speaker wiring of a Mac, derived from its device tree and its tuning.

    wiring.py {derive|describe} [--dt ROOT | --dtb FILE.dtb | --hw HW.json]
                                (--tuning-json DUMP.json | -- TUNING-CLI-ARGS...) [--key KEY]
    wiring.py hardware [--dt ROOT | --dtb FILE.dtb | --hw HW.json]
    wiring.py caps [--dt ROOT | --dtb FILE.dtb | --hw HW.json] [--live]
                   [--tuning-json DUMP.json | -- TUNING-CLI-ARGS...]

The protect stage needs two maps (packaging/session/gen.py):

- **unit_channels**: the unit channel of each physical speaker channel. The
  speaker PCM runs in the card's physical order; the unit takes its program
  in the tuning's own channel order.
- **sense_ports**: the unit's sense port of each sense PCM channel. The
  unit's sense bus is [V0, I0, V1, I1, ...] in unit channel order.

Both come from two independent sources, and nothing is written by hand.

**Hardware (the device tree, as macaudio reads it).**

- The card's "Speaker"/"Speakers" dai-link lists the amps (`codec`
  `sound-dai`, in order) and the back-ends (`cpu` `sound-dai`). macaudio
  gives the k-th amp of back-end b the k-th TDM slot of that back-end's mask:
  with two back-ends, the even slots on the first and the odd slots on the
  second; with one, every slot in order (macaudio_be_assign_tdm). Speaker
  PCM channel c is TDM slot c (the front-end's mask is every slot).
- Each amp's `sound-name-prefix` ("Left Woofer 2", "Right Tweeter") gives
  its side, role and ordinal.
- A lone speaker codec with no side (one node for a stereo pair, the
  J700's MAX98360As) is two amps, left on slot 0 and right on slot 1: each
  amp takes its half of the frame by its SD_MODE strap, which no software
  can observe, so that half of the map rests on the frame convention.
- Each V/I amp's `ti,vmon-slot-no` / `ti,imon-slot-no` are its sense TX
  slots. The TAS2764/SN012776 and TAS2770 TX slots are 8 bits wide and the
  sense PCM's channels are 16-bit slots, so sense PCM channel = slot / 2.

**Tuning (the reader's graph trace, `aurora-speaker-tuning dump --json`).**

- Each unit channel's lane starts at a channel of the graph's stereo input:
  channel 0 is left, 1 is right.
- Its role is the crossover branch it passes: a join whose name ends in
  "LF" (woofer) or "HF" (tweeter). A graph with no such joins is
  full-range on every channel.
- Within one side and role, unit channels take the amps' ordinals in order
  (the first left woofer lane is "Left Woofer 1").

Anything missing, unknown or ambiguous is refused with a message saying
what, never guessed. The self-check (speakerguardd-selfcheck) then proves
the hardware half on the running Mac, silently.

The rest of the machine description comes from the same device tree
(`describe()`): the machine key (the card's "apple,<key>-macaudio"
compatible, which also names its voicing graph), the ALSA card id (macaudio
names it "Apple" and the model's last word), the model, and the transport
(the speaker link's CPU DAI driver: its sample formats and how its rings
advance). Nothing per board is written by hand; the self-check confirms the
card id and the speaker format against ALSA on the running Mac.
"""
import hashlib
import json
import os
import re
import stat
import subprocess
import sys
import tempfile
from collections import namedtuple

SCHEMA = 1


class Refused(Exception):
    """The wiring cannot be derived, or is ambiguous."""


# Amp families macaudio drives, by device-tree compatible: whether each amp
# returns V/I sense, and the width of its sense TX slots in bits.
AMPS = {
    "ti,sn012776": ("sn012776", True, 8),
    "ti,tas2764": ("sn012776", True, 8),
    "ti,tas2770": ("tas2770", True, 8),
    "maxim,max98360a": ("max98360a", False, 0),
}
# The sense PCM's sample width (macaudio: 16-bit sense slots, S16_LE).
SENSE_BITS = 16

# The speaker link's CPU DAI drivers, by device-tree compatible (the kernel
# binds the node's first compatible it knows, so the first match in the
# node's list wins). Per driver, not per board:
#
# - formats: the speaker PCM's sample formats (the driver's hw constraints;
#   the speaker format is the first of FORMAT_ORDER it takes, the order the
#   self-check tries on the running card);
# - sense_batch: whether the sense ring's pointer advances by periods
#   (PipeWire's batch handling fits it) or in small chunks (the AOP sense
#   ring's 120 frames, where batch handling makes the follower resync);
# - speaker_headroom: frames of extra queue on the speaker sink (the AOP
#   speaker service's DMA cadence underran without 256);
# - speaker_irq: the speaker sink wakes on the PCM's period interrupt
#   (api.alsa.disable-tsched), i.e. on the audio clock, not on a system timer.
#   PipeWire's timer scheduling runs on CLOCK_MONOTONIC, which NTP slews (up
#   to 8.3% with chrony's default maxslewrate); during a slew the sense
#   follower, which may not resample, drifts against it and resyncs every
#   ~10 quanta, and the speaker driver xruns: 42 xruns at every M2 boot when
#   chrony corrected the boot clock (0.37 s, 15.5-20.3 s after boot), 34
#   on a forced 0.4 s slew; 0 of either with period interrupts (SGCHK
#   2026-11-09). MCA raises a period interrupt per period. The T6040 AOP
#   speaker service's playback is a cyclic ADMAC channel with a callback per
#   period: on the J616s the channel's line ran 494/s with the 400/s sense
#   ring, i.e. 94/s of playback against 93.75/s for 512-frame periods, and
#   with period interrupts a forced 0.4 s slew re-prepared the speaker PCM
#   0 times (timer: 14 SETUP/XRUN restarts and 27 sense resyncs, SINKGAP
#   2026-11-09). The T8140 (MacBook Neo) speaker DMA raises one interrupt
#   per period too (94/s for 512-frame periods on its own line), and its
#   timer-scheduled sink stopped and restarted 12 times under a forced slew
#   1.5 min after boot (0 with period interrupts; SINKGAP 2026-11-09). The
#   T8142 transport's cadence is unproven here and keeps the timer.
TRANSPORTS = {
    # sound/soc/apple/mca.c: S16_LE, S24_LE, S32_LE
    "apple,mca": {"transport": "mca", "formats": ["S16_LE", "S24_LE", "S32_LE"], "sense_batch": True, "speaker_headroom": 0,
                  "speaker_irq": True},
    # sound/soc/apple/aop_spkr.rs (T6040 speaker service): S24_LE playback, S16_LE sense.
    # Its PCMs have one width on every Mac, whatever the number of amps: six
    # playback channels and twelve sense channels (see PCM widths below).
    # Its sense is the back-end's "sense" DMA channel: a Mac whose AOP
    # carries no V/I sense (the J773g's ADT audio-speaker has a TX channel
    # only) names none, and its amps' sense is not read (transport_senses).
    "apple,t6041-aop-audio": {"transport": "aop-spkr", "formats": ["S24_LE"], "sense_batch": False, "speaker_headroom": 256,
                              "speaker_irq": True, "speaker_channels": 6, "sense_channels": 12,
                              "speaker_dma": "spkr", "sense_dma": "sense"},
    # sound/soc/apple/aop_t8140.rs: S32_LE playback, no sense
    "apple,t8140-aop-audio": {"transport": "aop-t8140", "formats": ["S32_LE"], "sense_batch": True, "speaker_headroom": 0,
                              "speaker_irq": True},
    # t8152_aop_audio.rs, T8142/T6050/T8152 configurations: ADMAC burst-residue
    # DMA, S24_LE playback and S16_LE sense, not the T6041 firmware ring above.
    # As there, the sense is the audio node's "sense" DMA channel: a Mac
    # whose AOP speaker stream carries none (e.g. the M5 Mac Studio's ADT
    # audio-speaker has one TX slot and no RX slot) names none (transport_senses).
    "apple,t8142-aop-audio": {"transport": "aop-dma", "formats": ["S24_LE"], "sense_batch": False, "speaker_headroom": 0,
                              "speaker_irq": False, "speaker_dma": "spkr", "sense_dma": "sense"},
}
# The T6050 (J714s) and T8152 (J804, J873g) bind the same t8152_aop_audio.rs
# driver: PCM widths follow the link's amps and sense path, and the self-check
# refuses a sensed wiring on a card with no sense PCM.
TRANSPORTS["apple,t6050-aop-audio"] = TRANSPORTS["apple,t8142-aop-audio"]
TRANSPORTS["apple,t8152-aop-audio"] = TRANSPORTS["apple,t8142-aop-audio"]
# Newer device trees omit the generic MCA fallback. Both compatibles bind
# mca.c; share the entry so back-ends using either name are the same driver.
TRANSPORTS["apple,t8103-mca"] = TRANSPORTS["apple,mca"]
# The T8132 AOP (MacBook Air M4) runs the same speaker service and binds the
# same driver; its node lists the T6041 compatible as a fallback.
TRANSPORTS["apple,t8132-aop-audio"] = TRANSPORTS["apple,t6041-aop-audio"]
# PCM widths. A transport without "speaker_channels"/"sense_channels" offers
# PCMs exactly as wide as the link's amps (one playback channel and one V/I
# pair each). A transport with them offers that width only, and the amps'
# lanes are the first ones: the amps' TDM slots and sense slots are checked to
# be 0..n-1 and 0..2n-1 (speaker_amps), and the rest carry nothing of theirs.

# The self-check's probe order on the speaker PCM.
FORMAT_ORDER = ["S24_LE", "S32_LE", "S16_LE"]
# ALSA format -> PipeWire audio.format (the 24-bit samples sit in 32-bit words).
PW_FORMAT = {"S24_LE": "S24_32LE", "S32_LE": "S32LE", "S16_LE": "S16LE"}
# Frames of headroom on a V/I sense follower: a graph cycle may run this late
# before PipeWire resyncs the sense against the program.
SENSE_HEADROOM = 512
# The sgv2 LV2 bundle's variants (crates/sgv2-lv2 VARIANTS):
# (amps, sense channels) -> URI fragment.
PLUGINS = {(1, 0): "1ch", (2, 0): "2ch", (2, 4): "2ch-vi", (4, 0): "4ch", (4, 8): "4ch-vi", (6, 12): "6ch-vi"}
# The protection units that model from the amps' V/I sense.
SENSE_UNITS = ("spp3",)
# The protection units that run without it (atsp open loop, lsm2 from its
# model alone): the only ones a transport that carries no sense admits. An
# allowlist, so that a unit added later is refused there until it is named.
NO_SENSE_UNITS = ("atsp", "lsm2")

_PREFIX = re.compile(r"^(?:(Left|Right)\s+)?(Woofer|Tweeter|Speaker|Full[- ]Range)(?:\s+(\d+))?$")
_ROLES = {"woofer": "woofer", "tweeter": "tweeter", "speaker": "full", "full-range": "full", "full range": "full"}
# Full-range speakers named by side and position ("Left Front", "Right
# Rear": the J293 and J493), or by side alone ("Left": the J313). The
# position is the speaker's ordinal on its side: the front (upper) one
# first, the rear (lower) one second, the order of the AID3/AID18 graphs
# (upper-left, upper-right, lower-left, lower-right).
_POSITIONAL = re.compile(r"^(Left|Right)(?:\s+(Front|Upper|Rear|Lower))?$")
_POSITIONS = {None: 1, "Front": 1, "Upper": 1, "Rear": 2, "Lower": 2}


def parse_prefix(prefix):
    """(side, role, ordinal) from a sound-name-prefix; side None = centre."""
    m = _PREFIX.match(prefix.strip())
    if m:
        side = {"Left": "L", "Right": "R", None: None}[m.group(1)]
        role = _ROLES[m.group(2).lower()]
        return side, role, int(m.group(3) or 1)
    m = _POSITIONAL.match(prefix.strip())
    if m:
        return {"Left": "L", "Right": "R"}[m.group(1)], "full", _POSITIONS[m.group(2)]
    raise Refused(f"amp prefix {prefix!r} names no side and role this derivation knows")


# ---------------------------------------------------------------- hardware


def _u32s(raw):
    return [int.from_bytes(raw[i : i + 4], "big") for i in range(0, len(raw) - len(raw) % 4, 4)]


def _str(raw):
    return raw.split(b"\0")[0].decode(errors="replace")


def _strs(raw):
    return [s.decode(errors="replace") for s in raw.split(b"\0") if s]


def _read(path):
    with open(path, "rb") as f:
        return f.read()


def read_user_file(path, cap, follow=False):
    """A file in a directory a user owns, read as root with the supervisor's
    rules for the helper report (read_report): no final link followed
    (unless `follow`), a regular file only, opened without blocking, at most
    `cap` bytes. None for anything else (missing, a link, a FIFO, a device,
    a directory, longer than `cap`): the caller treats it as not readable.
    The type is checked on an O_PATH reference before anything is opened
    for reading, so no device or FIFO is ever opened; the read reopens that
    same inode, so a link swapped in between changes nothing."""
    flags = os.O_PATH | os.O_CLOEXEC
    if not follow:
        flags |= os.O_NOFOLLOW
    try:
        ref = os.open(path, flags)
    except OSError:
        return None
    try:
        if not stat.S_ISREG(os.fstat(ref).st_mode):
            return None
        fd = os.open(f"/proc/self/fd/{ref}", os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
    except OSError:
        return None
    finally:
        os.close(ref)
    try:
        data = b""
        while len(data) <= cap:
            chunk = os.read(fd, cap + 1 - len(data))
            if not chunk:
                break
            data += chunk
    except OSError:
        return None
    finally:
        os.close(fd)
    if len(data) > cap:
        return None
    return data.decode(errors="replace")


def _phandles(root):
    """{phandle: node path} over the whole tree."""
    out = {}
    for d, _dirs, files in os.walk(root):
        for name in ("phandle", "linux,phandle"):
            if name in files:
                try:
                    out[_u32s(_read(os.path.join(d, name)))[0]] = d
                except (OSError, IndexError):
                    pass
    return out


def _cells(node):
    try:
        return _u32s(_read(os.path.join(node, "#sound-dai-cells")))[0]
    except (OSError, IndexError):
        return 0


def _dai_list(prop, phandles):
    """The nodes a sound-dai list points at, honouring #sound-dai-cells."""
    words = _u32s(prop)
    out, i = [], 0
    while i < len(words):
        node = phandles.get(words[i])
        if node is None:
            raise Refused(f"sound-dai points at unknown phandle {words[i]:#x}")
        out.append(node)
        i += 1 + _cells(node)
    return out


def _i2c_name(node, sysfs="/sys/bus/i2c/devices"):
    """The I2C device (e.g. "1-0038") whose of_node is `node`, if bound."""
    try:
        want = os.path.realpath(node)
        for d in os.listdir(sysfs):
            of = os.path.join(sysfs, d, "of_node")
            if os.path.exists(of) and os.path.realpath(of) == want:
                return d
    except OSError:
        pass
    return None


def _links(snd, root, phandles, sysfs):
    links = []
    for d in sorted(os.listdir(snd)):
        p = os.path.join(snd, d)
        if not os.path.isdir(p) or not os.path.exists(os.path.join(p, "link-name")):
            continue
        status = os.path.join(p, "status")
        if os.path.exists(status) and _str(_read(status)) not in ("okay", "ok"):
            continue
        cpu = _dai_list(_read(os.path.join(p, "cpu", "sound-dai")), phandles)
        codecs = []
        for node in _dai_list(_read(os.path.join(p, "codec", "sound-dai")), phandles):
            c = {"node": os.path.relpath(node, root)}
            for key, prop, conv in (
                ("prefix", "sound-name-prefix", _str),
                ("compatible", "compatible", _strs),
                ("vmon", "ti,vmon-slot-no", lambda r: _u32s(r)[0]),
                ("imon", "ti,imon-slot-no", lambda r: _u32s(r)[0]),
            ):
                f = os.path.join(node, prop)
                if os.path.exists(f):
                    c[key] = conv(_read(f))
            i2c = _i2c_name(node, sysfs)
            if i2c:
                c["i2c"] = i2c
            codecs.append(c)
        cpu_compat, cpu_dmas = [], []
        cpu_nodes = [os.path.relpath(node, root) for node in cpu]
        for node in cpu:
            f = os.path.join(node, "compatible")
            cpu_compat.append(_strs(_read(f)) if os.path.exists(f) else [])
            f = os.path.join(node, "dma-names")
            cpu_dmas.append(_strs(_read(f)) if os.path.exists(f) else None)
        links.append(
            {"name": _str(_read(os.path.join(p, "link-name"))), "cpus": len(cpu), "cpu_compatible": cpu_compat,
             "cpu_dma_names": cpu_dmas, "cpu_nodes": cpu_nodes, "codecs": codecs}
        )
    return links


def audio_stack(sound, root="/proc/device-tree", modules="/sys/module"):
    """A fingerprint (16 hex digits) of what carries the speaker and sense
    words on this boot, beyond the kernel release: the speaker card's node
    `sound` (relative to `root`, with everything under it), the nodes its
    links' `sound-dai` lists point at (the serializer and amp nodes, their
    own properties), and the build ID of every loaded sound module (and of
    the AOP core). A
    rebuilt module or a changed device tree under the same release (the
    J615's MCA receiver moved one bit clock by a module and a DT property)
    changes it. "-" when `sound` is not in the tree."""
    root = os.path.realpath(root)
    snd = os.path.join(root, sound)
    if not sound or not os.path.isdir(snd):
        return "-"
    h = hashlib.sha256()

    def props(node, recursive):
        for d, dirs, files in os.walk(node):
            dirs.sort()
            for name in sorted(files):
                f = os.path.join(d, name)
                h.update(os.path.relpath(f, root).encode() + b"\0")
                try:
                    h.update(_read(f))
                except OSError:
                    h.update(b"\xff")
                h.update(b"\0")
            if not recursive:
                dirs[:] = []

    props(snd, True)
    phandles = _phandles(root)
    nodes = set()
    for d in sorted(os.listdir(snd)):
        for side in ("cpu", "codec"):
            f = os.path.join(snd, d, side, "sound-dai")
            if os.path.exists(f):
                try:
                    nodes.update(_dai_list(_read(f), phandles))
                except Refused:
                    h.update(f"unresolved {d}/{side}".encode())
    for node in sorted(nodes):
        props(node, False)
    try:
        names = sorted(os.listdir(modules))
    except OSError:
        names = []
    for m in names:
        # the sound modules, and the AOP core the AOP Macs' speaker and
        # sense rings run through (not its sensor clients)
        if not (m.startswith("snd") or m == "aop"):
            continue
        h.update(f"module {m}\0".encode())
        # the loaded binary: its GNU build ID note (srcversion exists only
        # for modules with a MODULE_VERSION or CONFIG_MODULE_SRCVERSION_ALL)
        for f in ("notes/.note.gnu.build-id", "srcversion"):
            try:
                h.update(_read(os.path.join(modules, m, f)))
            except OSError:
                h.update(b"-")
            h.update(b"\0")
    return h.hexdigest()[:16]


def has_speaker_link(node):
    """Whether a macaudio device tree node has the link macaudio drives
    speakers on ("Speaker" or "Speakers"); the J614s/J616s/J713 jack-only
    card has none."""
    for c in os.listdir(node):
        try:
            with open(os.path.join(node, c, "link-name"), "rb") as f:
                if f.read().rstrip(b"\0") in (b"Speaker", b"Speakers"):
                    return True
        except OSError:
            continue
    return False


def card_registered(card_dir, proc="/proc/asound"):
    """Whether ALSA has finished registering the card at `card_dir`
    (/sys/class/sound/cardN). snd_card_register() adds the card device
    first (the sysfs directory with its id and of_node link), then the
    card's devices with the control device controlCN last, then publishes
    the card for lookup by number and id, and adds the card's proc entries
    (/proc/asound/cardN) at the very end. In between the card can be found
    but not opened: amixer then fails as if the card had no such control,
    and the proc PCM list is missing. Registered = controlCN in sysfs and,
    where ALSA's proc interface exists (`proc`), /proc/asound/cardN/id."""
    m = re.fullmatch(r"card(\d+)", os.path.basename(os.path.normpath(card_dir)))
    if not m or not os.path.exists(os.path.join(card_dir, f"controlC{m.group(1)}")):
        return False
    return not os.path.isdir(proc) or os.path.exists(os.path.join(proc, f"card{m.group(1)}", "id"))


def read_dt(root="/proc/device-tree", sysfs="/sys/bus/i2c/devices"):
    """The hardware description of the running Mac, from its device tree:
    the apple,macaudio card that has the speaker link (the J614s/J616s/J713
    also have a jack-only macaudio card)."""
    root = os.path.realpath(root)
    compatible = _strs(_read(os.path.join(root, "compatible")))
    phandles = _phandles(root)
    found = []
    for d in sorted(os.listdir(root)):
        p = os.path.join(root, d)
        if not (os.path.isdir(p) and os.path.exists(os.path.join(p, "compatible"))):
            continue
        if "apple,macaudio" not in _strs(_read(os.path.join(p, "compatible"))):
            continue
        status = os.path.join(p, "status")
        if os.path.exists(status) and _str(_read(status)) not in ("okay", "ok"):
            continue
        links = _links(p, root, phandles, sysfs)
        if any(l["name"] in ("Speaker", "Speakers") for l in links):
            found.append((p, links))
    if len(found) != 1:
        raise Refused(f"{len(found)} apple,macaudio cards with a speaker link in the device tree (want 1)")
    snd, links = found[0]
    model = os.path.join(snd, "model")
    return {
        "board": board_of(compatible),
        "compatible": compatible,
        "sound": os.path.relpath(snd, root),
        "sound_compatible": _strs(_read(os.path.join(snd, "compatible"))),
        "model": _str(_read(model)) if os.path.exists(model) else None,
        "links": links,
    }


def board_of(compatible):
    first = compatible[0] if compatible else ""
    if not first.startswith("apple,") or first == "apple,":
        raise Refused(f"root compatible {first!r} is not an Apple board")
    return first[len("apple,") :]


def _fdt_to_dir(blob, out):
    """Writes a flattened device tree (a .dtb) out as /proc/device-tree
    does: a directory per node, a file per property."""
    import struct

    magic, _size, off_struct, off_strings = struct.unpack(">IIII", blob[:16])
    if magic != 0xD00DFEED:
        raise Refused("not a flattened device tree (bad magic)")
    i, path = off_struct, [out]
    while True:
        (tok,) = struct.unpack(">I", blob[i : i + 4])
        i += 4
        if tok == 1:  # FDT_BEGIN_NODE
            end = blob.index(b"\0", i)
            name = blob[i:end].decode()
            i = (end + 4) & ~3
            path.append(os.path.join(path[-1], name) if name else path[-1])
            os.makedirs(path[-1], exist_ok=True)
        elif tok == 2:  # FDT_END_NODE
            path.pop()
        elif tok == 3:  # FDT_PROP
            ln, nameoff = struct.unpack(">II", blob[i : i + 8])
            i += 8
            name = blob[off_strings + nameoff : blob.index(b"\0", off_strings + nameoff)].decode()
            with open(os.path.join(path[-1], name), "wb") as f:
                f.write(blob[i : i + ln])
            i = (i + ln + 3) & ~3
        elif tok == 4:  # FDT_NOP
            continue
        elif tok == 9:  # FDT_END
            return
        else:
            raise Refused(f"device tree token {tok:#x} at {i - 4}")


def read_dtb(path):
    """read_dt() on a compiled device tree (the kernel build's .dtb): the
    same description, without the I2C bindings only a running Mac has."""
    with tempfile.TemporaryDirectory() as d:
        root = os.path.join(d, "dt")
        os.makedirs(root)
        _fdt_to_dir(_read(path), root)
        return read_dt(root, sysfs=os.path.join(d, "no-sysfs"))


_KEY = re.compile(r"^apple,([a-z0-9]+)-macaudio$")


def machine_key(hw):
    """The machine key ("j316" for the J316s and J316c): the speaker card's
    own "apple,<key>-macaudio" compatible, the configuration macaudio runs
    and the name of the machine's voicing graph."""
    for c in hw.get("sound_compatible") or []:
        m = _KEY.match(c)
        if m:
            return m.group(1)
    raise Refused(f"the speaker card's compatible {hw.get('sound_compatible')} names no apple,<machine>-macaudio")


def card_id(hw):
    """The ALSA card id macaudio gives the card: "Apple" and the last word
    of the device tree's model, in the 15 characters ALSA keeps."""
    model = (hw.get("model") or "").strip()
    if not model:
        raise Refused("the speaker card has no model: macaudio names no card id")
    return ("Apple" + model.split()[-1])[:15]


def transport(hw):
    """The speaker link's transport (TRANSPORTS), from its CPU DAI nodes'
    compatibles. Every back-end must be the same driver."""
    link = _speaker_link(hw)
    found = []
    for compat in link.get("cpu_compatible") or []:
        t = next((TRANSPORTS[c] for c in compat if c in TRANSPORTS), None)
        if t is None:
            raise Refused(f"speaker back-end {', '.join(compat) or '(no compatible)'} is not a transport this derivation knows")
        found.append(t)
    if not found:
        raise Refused("the speaker dai-link names no CPU DAI compatible")
    if any(t is not found[0] for t in found):
        raise Refused(f"mixed speaker transports: {sorted({t['transport'] for t in found})}")
    return dict(found[0])


def speaker_format(t):
    """The speaker PCM's ALSA format: the first of FORMAT_ORDER the
    transport takes."""
    fmt = next((f for f in FORMAT_ORDER if f in t["formats"]), None)
    if fmt is None:
        raise Refused(f"transport {t['transport']} takes none of {FORMAT_ORDER}")
    return fmt


def pcm_widths(t, amps, sense):
    """The channel counts the speaker and sense PCMs open with, for `amps`
    amps and `sense` sense lanes on transport `t` (PCM widths above)."""
    speaker_pcm = t.get("speaker_channels", amps)
    sense_pcm = t.get("sense_channels", sense) if sense else 0
    if speaker_pcm < amps:
        raise Refused(f"{amps} amps on a {speaker_pcm}-channel speaker PCM ({t['transport']})")
    if sense_pcm < sense:
        raise Refused(f"{sense} sense lanes on a {sense_pcm}-channel sense PCM ({t['transport']})")
    return speaker_pcm, sense_pcm


def _speaker_link(hw):
    spk = [l for l in hw["links"] if l["name"] in ("Speaker", "Speakers")]
    if len(spk) != 1:
        raise Refused(f"{len(spk)} speaker dai-links in the device tree (want 1)")
    return spk[0]


def tdm_slots(n_codecs, n_bes):
    """macaudio's TDM slot of each speaker amp, in dai-link order."""
    if n_bes not in (1, 2):
        raise Refused(f"{n_bes} speaker back-ends: macaudio's slot assignment covers one or two")
    if n_codecs % n_bes:
        raise Refused(f"{n_codecs} amps do not split over {n_bes} back-ends")
    nch = max(n_codecs, 2)
    left = [s for s in range(nch) if s % 2 == 0]
    right = [s for s in range(nch) if s % 2 == 1]
    per = n_codecs // n_bes
    slots = []
    for b in range(n_bes):
        mask = (right if b else left) if n_bes == 2 else sorted(left + right)
        if per > len(mask):
            raise Refused(f"{per} amps on a back-end with {len(mask)} slots")
        slots += mask[:per]
    return slots


def transport_senses(hw):
    """Whether the speaker link's transport carries the amps' V/I sense on
    this Mac. A transport whose sense is a named DMA channel ("sense_dma")
    carries none when its back-end nodes name their speaker channel
    ("speaker_dma") and not that one. A back-end with no dma-names, one
    that does not name its speaker channel either (a node not yet
    described for this Mac), or a device-tree read from before they were
    recorded, keeps the sense the amps' family offers. (transport() refuses
    unknown or mixed back-ends; this only reads the ones it knows.)"""
    link = _speaker_link(hw)
    dmas = link.get("cpu_dma_names")
    if not dmas or any(d is None for d in dmas):
        return True
    for compat, names in zip(link.get("cpu_compatible") or [], dmas):
        t = next((TRANSPORTS[c] for c in compat if c in TRANSPORTS), None)
        if t and t.get("sense_dma") and t.get("speaker_dma") in names and t["sense_dma"] not in names:
            return False
    return True


def hardware(hw):
    """The speaker amps in physical (speaker PCM) order, each with its
    side, role, ordinal, slot and sense channels."""
    link = _speaker_link(hw)
    codecs = link["codecs"]
    if not codecs:
        raise Refused("the speaker dai-link lists no amps")
    slots = tdm_slots(len(codecs), link["cpus"])
    # An amp family's V/I sense counts only where the transport carries it:
    # without, the amps are wired without it, and the record says why
    # ("no_sense_path", method "derived-nosense").
    sensed = transport_senses(hw)
    amps, families = [], set()
    # A lone amp with no sound-name-prefix is the one full-range speaker of
    # a desktop Mac (J274, J375, J473, J474, J475: macaudio's mono speaker
    # configuration). Any other amp must name itself.
    mono = len(codecs) == 1 and "prefix" not in codecs[0]
    for c, slot in zip(codecs, slots):
        if mono:
            c = dict(c, prefix="Speaker")
        if "prefix" not in c:
            raise Refused(f"amp {c['node']} has no sound-name-prefix")
        fam = next((AMPS[x] for x in c.get("compatible", []) if x in AMPS), None)
        if fam is None:
            raise Refused(f"amp {c['prefix']} ({', '.join(c.get('compatible', []))}) is not an amp this derivation knows")
        families.add(fam)
        side, role, ordinal = parse_prefix(c["prefix"])
        a = {"prefix": c["prefix"], "family": fam[0], "slot": slot, "side": side, "role": role, "ordinal": ordinal}
        if mono:
            # "Speaker" only labels the lone amp here: with no
            # sound-name-prefix its ALSA controls carry no prefix at all
            # ("VSENSE Switch", "Speaker Volume").
            a["control_prefix"] = ""
        if c.get("i2c"):
            a["i2c"] = c["i2c"]
        if fam[1] and not sensed:
            a["no_sense_path"] = True
        elif fam[1]:
            for k in ("vmon", "imon"):
                if k not in c:
                    raise Refused(f"amp {c['prefix']} has V/I sense but no ti,{k}-slot-no")
                bits = c[k] * fam[2]
                if bits % SENSE_BITS:
                    raise Refused(f"amp {c['prefix']} {k} slot {c[k]} is not on a {SENSE_BITS}-bit sense channel")
                a[k[0] + "_ch"] = bits // SENSE_BITS
        amps.append(a)
    if len(families) != 1:
        raise Refused(f"mixed amp families on the speaker link: {sorted(f[0] for f in families)}")
    if len(amps) == 1 and amps[0]["side"] is None and not mono:
        # One device-tree node for a stereo pair (the J700's MAX98360As):
        # macaudio gives a lone speaker codec two channels, and each amp
        # takes its frame half by its SD_MODE strap, left on slot 0 and
        # right on slot 1 (the I2S/DSP_A frame order). Nothing in software
        # can observe the strap: this half of the map is the frame
        # convention, and the record says so.
        if next(iter(families))[1]:
            raise Refused(f"amp {amps[0]['prefix']} has V/I sense but names no side")
        one = amps[0]
        amps = [
            dict(one, prefix=f"{one['prefix']} {side}", side=side, slot=slot, by_strap=True)
            for slot, side in ((0, "L"), (1, "R"))
        ]
    order = sorted(range(len(amps)), key=lambda i: amps[i]["slot"])
    amps = [amps[i] for i in order]
    if [a["slot"] for a in amps] != list(range(len(amps))):
        raise Refused(f"the amps' TDM slots {[a['slot'] for a in amps]} leave gaps in the speaker PCM")
    keys = [(a["side"], a["role"], a["ordinal"]) for a in amps]
    if len(set(keys)) != len(keys):
        raise Refused(f"two amps share a side, role and ordinal: {[a['prefix'] for a in amps]}")
    if next(iter(families))[1] and sensed:
        chans = sorted(ch for a in amps for ch in (a["v_ch"], a["i_ch"]))
        if chans != list(range(2 * len(amps))):
            raise Refused(f"the sense slots give channels {chans}, not 0..{2 * len(amps) - 1} once each")
    return amps


# ------------------------------------------------------------- capabilities
#
# speakerguardd selects nothing by board, model or SoC. Every fact the
# protect stage, the self-check and the supervisor depend on is a
# *capability*, read from a source the machine itself provides, and each one
# carries that source (`Cap.source`) so a report says where it came from:
#
# - ``dt:``    the device tree, as macaudio reads it: the speaker dai-link,
#              its back-end (CPU DAI) and amp (codec) nodes, their
#              compatibles, `dma-names`, `sound-name-prefix` and sense slots;
# - ``alsa:``  the running card: the card bound to the device tree's sound
#              node, and (supervisor side) its controls' own dB scales;
# - ``tuning:`` the Mac's own tuning bundle, through the reader's graph trace:
#              the protection unit, its channel roles, its sample rate, the
#              speaker identity (AID) it names;
# - ``driver:`` a constant of the kernel driver a compatible binds (sample
#              formats, ring cadence). Keyed by the block's compatible,
#              never by the board that carries it;
# - ``derived:`` computed from the capabilities above (the PCM widths, the
#              plugin variant), never from an identity.
#
# A capability no source establishes is a refusal (Refused): the
# speakers stay muted exactly as before, with the reason. There is no
# default that admits a Mac; the only defaults are ones that keep a Mac
# safe (no sense proven: the unit must run without sense; no fixed ceiling
# named: the supervisor latches on any other grant volume).
#
# The board itself is still read (``dt:/compatible``) but only as an
# identity: the self-check record and the parameter blob are bound to the Mac
# they were made on, so one Mac's proof never admits another. It is never
# used to choose a value.
#
# See docs/CAPABILITIES.md for the inventory, the providers and the plan.

# A capability: its value and where it came from.
Cap = namedtuple("Cap", "value source")

# Everything an amp family's driver offers userspace, by the family the
# amp's compatible names (AMPS). The supervisor checks each of these
# against the card's controls before a grant (crates/sgv2-supervisor
# machine.rs Amp): these are the driver's, not the board's.
FAMILY = {
    "sn012776": {"gain_control": True, "fault_status": True},
    "tas2770": {"gain_control": True, "fault_status": False},
    "max98360a": {"gain_control": False, "fault_status": False},
}


def _src_cpu(hw):
    link = _speaker_link(hw)
    nodes = link.get("cpu_nodes")
    where = ", ".join(nodes) if nodes else f"link {link['name']!r} cpu"
    return f"dt:{where}/compatible"


def cap_transport(hw):
    """The speaker back-end's driver and what it offers (TRANSPORTS)."""
    t = transport(hw)
    return Cap(t, "driver:" + t["transport"] + " (" + _src_cpu(hw) + ")")


def sense_path(hw):
    """Whether this Mac returns its amps' V/I sense: the amp family must
    offer it and the speaker transport carry it (transport_senses), and
    from which value.

    - An amp family without sense (AMPS): none, by the family's driver.
    - The back-end names the driver's sense DMA channel: it does.
    - It names its speaker channel and not the sense one: it does not.
    - It names neither, or no dma-names at all: the amp family's sense
      stands, and the self-check's silent sense test must then see it on
      the card; a transport that sends none fails that test (muted).
    """
    link = _speaker_link(hw)
    fams = {AMPS[x] for c in link["codecs"] for x in c.get("compatible", []) if x in AMPS}
    if fams and not any(f[1] for f in fams):
        return Cap(False, f"driver:{', '.join(sorted(f[0] for f in fams))} returns no V/I sense")
    dmas = link.get("cpu_dma_names")
    t = transport(hw)
    carries = transport_senses(hw)
    if not dmas or any(d is None for d in dmas) or not t.get("sense_dma"):
        return Cap(carries, "dt:no sense dma-names on the back-end; proven by the self-check's silent sense test")
    names = sorted({n for d in dmas for n in d})
    if t["sense_dma"] in names:
        return Cap(carries, f"dt:back-end dma-names {names} (names {t['sense_dma']!r})")
    if not carries:
        return Cap(carries, f"dt:back-end dma-names {names} (names {t.get('speaker_dma')!r}, no {t['sense_dma']!r})")
    # dma-names that name neither channel describe nothing about sense
    return Cap(carries, f"dt:back-end dma-names {names} name neither {t.get('speaker_dma')!r} nor {t['sense_dma']!r}; "
                        "proven by the self-check's silent sense test")


def cap_amps(hw):
    """The amps in speaker PCM order (hardware), and the family's
    driver capabilities."""
    a = hardware(hw)
    fam = a[0]["family"]
    if fam not in FAMILY:
        raise Refused(f"amp family {fam} has no driver capabilities this probe knows")
    sensed = sum(1 for x in a if "v_ch" in x)
    return {
        "amps": Cap(len(a), "dt:speaker dai-link codec sound-dai"),
        "family": Cap(fam, "dt:amp compatible"),
        "roles": Cap([[x["side"], x["role"], x["ordinal"]] for x in a], "dt:amp sound-name-prefix"),
        "gain_control": Cap(FAMILY[fam]["gain_control"], f"driver:{fam}"),
        "fault_status": Cap(FAMILY[fam]["fault_status"], f"driver:{fam}"),
        "sense_slots": Cap(2 * sensed, "dt:ti,vmon-slot-no / ti,imon-slot-no" if sensed else "dt:no sense slots"),
        "_amps": a,
    }


def live_card(hw, dt="/proc/device-tree", sysfs="/sys/class/sound", proc="/proc/asound"):
    """(index, id) of the ALSA card whose device is the device tree's sound
    node: the running card's own id. None when no card is bound to it, or
    when the bound card is still registering (card_registered): the caller
    waits, or keeps its record."""
    want = os.path.realpath(os.path.join(dt, hw["sound"]))
    try:
        names = sorted(os.listdir(sysfs))
    except OSError:
        return None
    for d in names:
        if not re.fullmatch(r"card\d+", d):
            continue
        of = os.path.join(sysfs, d, "device", "of_node")
        if os.path.exists(of) and os.path.realpath(of) == want:
            if not card_registered(os.path.join(sysfs, d), proc):
                return None
            idx = int(d[4:])
            # The id from sysfs: the attribute exists as soon as the card
            # device does. snd_card_register() adds the device (and its
            # of_node link) before the card's proc entries, so a card that is
            # still registering may have no /proc id yet (the proc copy is the
            # fallback). Not there yet = not bound yet (None: the caller waits
            # or keeps the record); any other error refuses.
            for path in (os.path.join(sysfs, d, "id"), os.path.join(proc, f"card{idx}", "id")):
                try:
                    with open(path) as f:
                        cid = f.read().strip()
                except FileNotFoundError:
                    continue
                except OSError as e:
                    raise Refused(f"the speaker card {d} is bound to {hw['sound']}, but its id cannot be read ({path}: "
                                  f"{e.strerror or e})") from None
                if cid:
                    return idx, cid
            return None
    return None


def cap_card(hw, live=None):
    """The speaker card's ALSA id. On a running Mac (`live`, from
    live_card()) it is the card's own; offline (fixtures, a .dtb) it is the
    name macaudio gives it from the device tree's model, which the
    self-check then confirms on the card."""
    if live is not None:
        return Cap(live[1], f"alsa:card{live[0]} (of_node {hw['sound']})")
    return Cap(card_id(hw), f"dt:/{hw['sound']}/model (macaudio's naming; confirmed on the card by the self-check)")


def probe(hw, live=None, w=None):
    """Every capability of the speaker path, each with its source: the
    hardware's, and with a wiring `w` (derive) the tuning's too. Refuses
    (Refused) on anything no source establishes."""
    t = cap_transport(hw)
    a = cap_amps(hw)
    s = sense_path(hw)
    fmt = speaker_format(t.value)
    caps = {
        "board": Cap(hw["board"], "dt:/compatible (identity binding only)"),
        "machine_key": Cap(machine_key(hw), f"dt:/{hw['sound']}/compatible"),
        "model": Cap(hw.get("model"), f"dt:/{hw['sound']}/model"),
        "card": cap_card(hw, live),
        "transport": Cap(t.value["transport"], t.source),
        "speaker_format": Cap(fmt, f"driver:{t.value['transport']} formats {t.value['formats']}"),
        "speaker_irq": Cap(t.value["speaker_irq"], f"driver:{t.value['transport']}"),
        "speaker_headroom": Cap(t.value["speaker_headroom"], f"driver:{t.value['transport']}"),
        "sense_batch": Cap(t.value["sense_batch"], f"driver:{t.value['transport']}"),
        "sense_path": s,
    }
    caps.update({k: v for k, v in a.items() if not k.startswith("_")})
    if w is not None:
        caps.update(
            unit=Cap(w["unit_kind"], "tuning:the protection unit the graph trace names"),
            aid=Cap(w["aid"], "tuning:the bundle's speaker identity"),
            rate=Cap(w["rate"], "tuning:the graph's sample rate"),
            unit_channels=Cap(w["unit_channels"], "derived:dt roles matched to the tuning's channel roles"),
            sense_ports=Cap(w["sense_ports"], "derived:dt sense slots matched to the unit's channels"),
        )
    caps["_transport"] = t.value
    caps["_amps"] = a["_amps"]
    return caps


def describe(hw, w, live=None):
    """ the machine description the protect stage's graph
    is generated from, built from the capabilities (probe) and the wiring
    derived from the device tree and the tuning. Byte for byte what the
    description was before the probe layer, except that on a running Mac
    the card id is the card's own (`live`)."""
    c = probe(hw, live, w)
    t = c["_transport"]
    n = len(w["amps"])
    if n != c["amps"].value:
        raise Refused(f"the wiring has {n} amps, the device tree {c['amps'].value}")
    # LSID is a side-chain inside the existing LSM2 governor. Plain LSM2
    # bundles still have no sense ports; only an explicit companion requests
    # the V/I variant. The self-check proves the sense map either way.
    takes_sense = w.get("unit_kind") in (None, *SENSE_UNITS) or w.get("sense_companion", False)
    s = len(w["sense"]) if takes_sense else 0
    plugin = PLUGINS.get((n, s))
    if plugin is None:
        raise Refused(f"no sgv2 plugin variant for {n} amps and {s} sense channels")
    speaker_pcm, sense_pcm = pcm_widths(t, n, len(w["sense"]))
    fmt = c["speaker_format"].value
    return {
        "key": c["machine_key"].value,
        "board": c["board"].value,
        "card": c["card"].value,
        "name": c["model"].value or c["card"].value,
        "transport": c["transport"].value,
        "amps": n,
        "sense": s,
        # the PCMs' own widths: the amps' lanes come first (PCM widths in py)
        "speaker_pcm": speaker_pcm,
        "sense_pcm": sense_pcm,
        "rate": c["rate"].value,
        "plugin": plugin,
        "alsa_format": fmt,
        "speaker_format": PW_FORMAT[fmt],
        "speaker_headroom": c["speaker_headroom"].value,
        "sense_headroom": SENSE_HEADROOM if s else 0,
        "sense_batch": c["sense_batch"].value,
        "speaker_irq": c["speaker_irq"].value,
    }


def caps_report(caps):
    """The capabilities as JSON-able {name: {"value", "source"}}."""
    return {k: {"value": v.value, "source": v.source} for k, v in caps.items() if not k.startswith("_")}


# ------------------------------------------------------------------ tuning


# Qualification records contain hashes/structural facts only, never tuning values.
QUALIFICATION_VERSION = 2
PROTECTION_UNITS = {"spp3", "atsp", "lsm2", "lsid"}


def unit_id(dt="/proc/device-tree"):
    """The collector/compiler's domain-separated unit identity, never the serial."""
    try:
        with open(os.path.join(dt, "serial-number"), "rb") as f:
            serial = f.read().rstrip(b"\0")
    except OSError:
        return "-"
    if not 0 < len(serial) <= 64 or not all(0x21 <= b <= 0x7E for b in serial):
        return "malformed"
    return hashlib.sha256(b"aurora-speaker-unit-v1\0" + serial).hexdigest()


def protection_units(dump):
    """All protection types on the graph, including the LSID companion.

    The production reader admits exactly one primary protection box. Its
    refusals and error diagnostics must be empty; wiring alone is not proof
    that a graph can compile.
    """
    t = dump.get("tuning", dump)
    if t.get("refusals") or any(d.get("severity") == "error" for d in dump.get("diagnostics", {}).get("items", [])):
        raise Refused("the tuning graph has refusals")
    primary = (t.get("protection") or {}).get("kind")
    if primary not in PROTECTION_UNITS - {"lsid"}:
        raise Refused("the tuning names no supported primary protection unit")
    return sorted({primary} | ({"lsid"} if t.get("lsid") is not None else set()))


def read_binding(path):
    with open(path) as f:
        return dict(line.strip().split("=", 1) for line in f if "=" in line)


# ------------------------------------------------------- kernel interface

# A dual kernel (linux integration/aurora 98cb6a86 and later) serves both
# speaker protection interfaces on one card. "Speaker Volume Unlock" (the
# older daemons: speakersafetyd, speakerguardd 0.1) is in charge at every
# card bind. A root write of "Speaker Protection Interface" = "Speaker
# Protection" hands the speakers to the v2 lease, through the floor; the
# kernel hands them back to the volume lock (at its floor) after 30 s with
# no owner of "Speaker Protection". The switch is state for the boot, never
# saved by alsactl: v2 writes it again at every start, and only on a Mac
# that aurora-speaker-upgrade switched (its setting file, root only).
INTERFACE_CONTROL = "Speaker Protection Interface"
IF_LEASE = "Speaker Protection"
IF_UNLOCK = "Speaker Volume Unlock"
UPGRADE_SETTING = "/var/lib/aurora-speaker-upgrade/interface"


def upgraded(path=None):
    """True when aurora-speaker-upgrade switched this Mac to v2 (its
    persistent choice). Only that tool writes the file."""
    try:
        with open(path or UPGRADE_SETTING) as f:
            return f.read() == "v2\n"
    except OSError:
        return False


def _amixer(card, *args):
    return subprocess.run(["amixer", "-c", str(card), *args], capture_output=True, text=True, timeout=10)


def parse_enum(text):
    """The current item's name from `amixer cget` of an enumerated control."""
    items = dict(re.findall(r"; Item #(\d+) '([^']*)'", text))
    m = re.search(r": values=(\d+)", text)
    return items.get(m.group(1)) if m else None


def interface_get(card):
    """The card's "Speaker Protection Interface" item, or None when the
    kernel has no such control (not a dual kernel)."""
    r = _amixer(card, "cget", f"name={INTERFACE_CONTROL}")
    if r.returncode != 0:
        return None
    cur = parse_enum(r.stdout)
    if cur is None:
        raise Refused(f"cannot read {INTERFACE_CONTROL!r} on card {card}: {r.stdout.strip()[-200:]}")
    return cur


def interface_lease(card):
    """Puts a dual kernel's speakers on the v2 lease (root). Returns True if
    it switched, False if the lease already was the interface."""
    if interface_get(card) == IF_LEASE:
        return False
    r = _amixer(card, "-q", "cset", f"name={INTERFACE_CONTROL}", IF_LEASE)
    if r.returncode != 0:
        raise Refused(f"the kernel refused {INTERFACE_CONTROL!r} = {IF_LEASE!r}: {r.stderr.strip()[-200:]}")
    if interface_get(card) != IF_LEASE:
        raise Refused(f"{INTERFACE_CONTROL!r} did not switch to {IF_LEASE!r}")
    return True


def qualification(state, board, kernel, params=None, unit=None, units=None):
    """Require matching successful compile/provenance and silent-check records.

    Only root-owned provisioning writes these records. A legacy record lacks
    this version and therefore triggers fresh provisioning, never admission.
    Returns the compiled protection types, for the build's parity policy.
    """
    try:
        with open(os.path.join(state, "provision.json")) as f:
            p = json.load(f)
        pb = read_binding(os.path.join(state, "provision.bind"))
        with open(params or os.path.join(state, "params.bin"), "rb") as f:
            digest = hashlib.sha256(f.read()).hexdigest()
        want = {"version": QUALIFICATION_VERSION, "result": "pass", "board": board,
                "kernel": kernel, "params_sha256": digest, "compile_refusals": 0,
                "provenance": "own-macos"}
        if any(type(p.get(k)) is not type(v) or p[k] != v for k, v in want.items()):
            raise Refused("no current successful compilation with own-Mac provenance")
        keys = (*want, "unit", "bundle_digest", "macos_build", "protection_units", "voicing_pre")
        if any(pb.get(k) != str(p.get(k)) for k in keys):
            raise Refused("the provisioning report and binding disagree")
        if not re.fullmatch(r"[0-9a-f]{64}", p.get("unit", "")) or (unit is not None and p["unit"] != unit):
            raise Refused("the tuning provenance is not bound to this Mac")
        compiled = p.get("protection_units", "").split(",")
        if not compiled or any(u not in PROTECTION_UNITS for u in compiled) or (units is not None and compiled != units):
            raise Refused("the compiled protection types disagree with the tuning")
        b = read_binding(os.path.join(state, "selfcheck.bind"))
        with open(os.path.join(state, "selfcheck.json")) as f:
            checked = json.load(f)["record"]
        want = {"result": "pass", "board": board, "kernel": kernel, "params_sha256": digest}
        if any(b.get(k) != v or checked.get(k) != v for k, v in want.items()):
            raise Refused("no passing self-check for this board, kernel and blob")
        if not b.get("wiring") or b["wiring"] == "-" or checked.get("wiring") != b["wiring"]:
            raise Refused("the self-check wiring records disagree")
        return compiled
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
        raise Refused(f"qualification records unavailable or malformed: {e}") from e


def roles(dump):
    """Each unit channel's (side, role), from the reader's JSON dump."""
    t = dump.get("tuning", dump)
    p = t.get("protection")
    if not p:
        raise Refused("the tuning has no protection unit")
    for sp in p.get("strip_properties") or []:
        if sp.get("number") == 64005:
            raise Refused("the strip sets SpeakerInputDataSourceOrder (64005): not yet derivable")
    lanes = p.get("lanes") or []
    if not lanes:
        raise Refused("the tuning's graph feeds the unit no lanes")
    origins = {(l["origin"]["node"], l["origin"]["port"]) for l in lanes}
    if len(origins) != 1:
        raise Refused(f"the unit's lanes start at {len(origins)} graph inputs, not one stereo input")
    out = []
    for k, l in enumerate(lanes):
        ch = l["origin"]["channel"]
        if ch not in (0, 1):
            raise Refused(f"unit channel {k} starts at input channel {ch}, not left (0) or right (1)")
        tags = set()
        for h in l.get("hops") or []:
            j = h.get("join", "")
            if j.upper().endswith("LF"):
                tags.add("woofer")
            elif j.upper().endswith("HF"):
                tags.add("tweeter")
        if len(tags) > 1:
            raise Refused(f"unit channel {k} passes both LF and HF branches")
        # a unit with one lane protects the one speaker of a mono Mac
        out.append((None if len(lanes) == 1 else "LR"[ch], tags.pop() if tags else "full"))
    rs = {r for _, r in out}
    if "full" in rs and len(rs) > 1:
        raise Refused("some unit channels pass a crossover branch and some do not")
    result = {"kind": p.get("kind"), "aid": t.get("aid"), "channels": out}
    companion = t.get("lsid")
    if companion is not None:
        if (p.get("kind") != "lsm2" or not isinstance(companion, dict)
                or companion.get("adaptation_enabled") is not False
                or not isinstance(companion.get("source_port"), str)
                or not companion["source_port"]
                or companion.get("source_lanes") != list(range(2 * len(lanes)))):
            raise Refused("invalid or enabled LSID companion metadata")
        if len(lanes) == 6 and companion.get("delay_frames") != [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]:
            raise Refused("LSID needs lane 1 alone aligned by one frame")
        result["sense_companion"] = True
    return result


def tuning_rate(dump):
    """The speaker graph's sample rate: the tuning's own, as the reader
    selected it (the DSP database's rate and the graph's `fs`), which is
    also the rate the parameter blob is compiled for. Refused when the
    tuning states none, or two that disagree."""
    t = dump.get("tuning", dump)
    sel = t.get("selection") or {}
    found = set()
    for v in (sel.get("sample_rate"), (sel.get("substitutions") or {}).get("fs")):
        if v is None:
            continue
        try:
            r = int(str(v))
        except ValueError:
            raise Refused(f"the tuning's sample rate {v!r} is not a number")
        if r <= 0:
            raise Refused(f"the tuning's sample rate is {r}")
        found.add(r)
    if not found:
        raise Refused("the tuning states no sample rate (no DSP database selection)")
    if len(found) > 1:
        raise Refused(f"the tuning's sample rates disagree: {sorted(found)}")
    return found.pop()


# ------------------------------------------------------------------ wiring


def derive(hw, dump, key=None):
    """The full wiring: amps in physical order with their unit channel, and
    the sense map."""
    amps = hardware(hw)
    rl = roles(dump)
    no_sense_path = any(a.get("no_sense_path") for a in amps)
    if no_sense_path and rl["kind"] not in NO_SENSE_UNITS:
        raise Refused(
            f"the tuning's {rl['kind']} protection is not one that runs without V/I sense "
            f"({', '.join(NO_SENSE_UNITS)}), and this Mac's speaker transport carries none"
        )
    # Without sense, nothing checks which amp plays which unit channel: one
    # amp has no order to get wrong, several would rest on the device tree
    # alone. A sense-capable Mac whose device tree lost its sense channel
    # must fail here, not run unproven.
    if no_sense_path and len(amps) != 1:
        raise Refused(
            f"this Mac's speaker transport carries no V/I sense, which is admitted for one amp only; "
            f"the order of its {len(amps)} amps would rest on the device tree alone"
        )
    units = rl["channels"]
    if len(units) != len(amps):
        raise Refused(f"the tuning's unit has {len(units)} channels, the board {len(amps)} amps")
    seen, want = {}, {}
    for u, (side, role) in enumerate(units):
        seen[(side, role)] = seen.get((side, role), 0) + 1
        want[(side, role, seen[(side, role)])] = u
    for a in amps:
        k = (a["side"], a["role"], a["ordinal"])
        if k not in want:
            raise Refused(
                f"amp {a['prefix']} ({a['side']} {a['role']} {a['ordinal']}) matches no unit channel; the tuning has "
                + ", ".join(f"{s} {r}" for s, r in units)
            )
        a["unit"] = want.pop(k)
    if want:
        raise Refused(f"unit channels {sorted(want.values())} match no amp")
    sense = []
    if "v_ch" in amps[0]:
        by_ch = {}
        for p, a in enumerate(amps):
            by_ch[a["v_ch"]] = (p, "V")
            by_ch[a["i_ch"]] = (p, "I")
        for k in range(2 * len(amps)):
            p, kind = by_ch[k]
            sense.append({"pcm": k, "amp": p, "kind": kind, "port": 2 * amps[p]["unit"] + (0 if kind == "V" else 1)})
    w = {
        "schema": SCHEMA,
        "board": hw["board"],
        "key": key or machine_key(hw),
        "model": hw.get("model"),
        "family": amps[0]["family"],
        "aid": rl["aid"],
        "unit_kind": rl["kind"],
        "rate": tuning_rate(dump),
        "amps": amps,
        "unit_channels": [a["unit"] for a in amps],
        "sense": sense,
        "sense_ports": [s["port"] for s in sense],
        # A sense-capable amp family on a transport that carries no sense
        # (transport_senses): proven like a family without sense, and the
        # record says why (method "derived-nosense").
        "no_sense_path": no_sense_path,
    }
    if rl.get("sense_companion"):
        if len(sense) != 2 * len(amps):
            raise Refused("LSID companion requires a complete V/I sense bus")
        w["sense_companion"] = True
    w["hash"] = wiring_hash(w)
    return w


def wiring_hash(w):
    """16 hex digits over the map the graph and the self-check rely on (the
    board is bound separately, so the J316s and J316c share it)."""
    core = {
        "unit_channels": w["unit_channels"],
        "sense_ports": w["sense_ports"],
        "amps": [[a["prefix"], a["slot"], a.get("v_ch"), a.get("i_ch")] for a in w["amps"]],
    }
    return hashlib.sha256(json.dumps(core, sort_keys=True, separators=(",", ":")).encode()).hexdigest()[:16]


def role_errors(w, units):
    """Every physical channel whose amp is not the speaker its unit channel
    models (side and role), for any map, derived or not."""
    errs = []
    for p, a in enumerate(w["amps"]):
        side, role = units[a["unit"]]
        if (a["side"], a["role"]) != (side, role):
            errs.append(
                f"physical channel {p} ({a['prefix']}) feeds unit channel {a['unit']}, which the tuning models as {side} {role}"
            )
    return errs


def dump_errors(r):
    """The reader's error diagnostics ("code: message"), so a refusal says
    why in the journal: from the JSON (a failed load prints the bare
    diagnostics, a load with errors the tuning and its diagnostics), else
    the last stderr line."""
    try:
        v = json.loads(r.stdout)
        items = (v.get("diagnostics", v) if isinstance(v, dict) else {}).get("items", [])
        errs = [f"{d.get('code')}: {d.get('message')}" for d in items if d.get("severity") == "error"]
    except (ValueError, AttributeError):
        errs = []
    if not errs and r.stderr.strip():
        errs = [r.stderr.strip().splitlines()[-1]]
    return (": " + "; ".join(errs))[:600] if errs else ""


def tuning_dump(cli_args, cli="aurora-speaker-tuning", strict=False):
    """Run the tuning reader's JSON dump; its exit code reports refusals,
    which offline derivation does not need. Qualification uses strict=True
    to require success and an empty refusal/error set."""
    r = subprocess.run([cli, "dump", "--json", *cli_args], capture_output=True, text=True, timeout=60)
    if strict and r.returncode != 0:
        raise Refused(f"{cli} dump refused the tuning (exit {r.returncode}){dump_errors(r)}")
    try:
        result = json.loads(r.stdout)
        if strict:
            protection_units(result)
        return result
    except ValueError:
        raise Refused(f"{cli} dump --json gave no JSON (exit {r.returncode}): {r.stderr.strip()[-300:]}")


# ---------------------------------------------------------------- microphones
#
# The built-in microphone array (the "hpai" stream of the AOP audio
# service). What the kernel's drivers fix per audio block, keyed by the
# block's device-tree compatible like TRANSPORTS:
# - dma: the dma-names entry the driver takes the array's stream from; a Mac
#   whose audio node lists none has no array (Mac mini, Mac Studio);
# - card: the suffix of the card the driver registers the array on
#   ("Apple<apple,chassis-name><suffix>");
# - dev: the array's capture PCM on that card (its PCM id is "hpai");
# - channels, rate: the PCM's only layout.
MIC_PCM_ID = "hpai"
MIC_TRANSPORTS = {
    # sound/soc/apple/aop_audio.rs (the PDM microphones, T8103 to T6041)
    "apple,aop-audio": {"dma": "dma", "card": "HPAI", "dev": 0, "channels": 3, "rate": 48000},
    "apple,t6030-aop-audio": {"dma": "dma", "card": "HPAI", "dev": 0, "channels": 3, "rate": 48000},
    "apple,t6041-aop-audio": {"dma": "dma", "card": "HPAI", "dev": 0, "channels": 3, "rate": 48000},
    # sound/soc/apple/aop_t8140.rs: PCM 0 of its card is the low-power microphone
    "apple,t8140-aop-audio": {"dma": "hpai", "card": "AOP", "dev": 1, "channels": 2, "rate": 48000},
    # sound/soc/apple/t8152_aop_audio.rs (T8142, T6050, T8152)
    "apple,t8152-aop-audio": {"dma": "hpai", "card": "HPAI", "dev": 0, "channels": 3, "rate": 48000},
}
MIC_TRANSPORTS["apple,t8142-aop-audio"] = MIC_TRANSPORTS["apple,t8152-aop-audio"]
MIC_TRANSPORTS["apple,t6050-aop-audio"] = MIC_TRANSPORTS["apple,t8152-aop-audio"]
# The sgv2_mic plugin's variants (crates/sgv2-lv2/src/mic.rs VARIANTS).
MIC_PLUGIN_MAX = 4


def _okay(node, root):
    """Whether a device-tree node and every node above it is enabled."""
    node, root = os.path.realpath(node), os.path.realpath(root)
    while True:
        status = os.path.join(node, "status")
        if os.path.exists(status) and _str(_read(status)) not in ("okay", "ok"):
            return False
        if node == root or os.path.dirname(node) == node:
            return True
        node = os.path.dirname(node)


def read_mic_dt(root="/proc/device-tree"):
    """The built-in microphone array of the running Mac, from its device
    tree, or None when it has none: the enabled AOP audio node (a node named
    "audio" whose compatible MIC_TRANSPORTS knows) that lists the array's
    stream in dma-names. {node, compatible, dma_names, chassis}."""
    root = os.path.realpath(root)
    found = []
    for d, dirs, files in os.walk(root):
        dirs.sort()
        if os.path.basename(d) != "audio" or "compatible" not in files:
            continue
        compatible = _strs(_read(os.path.join(d, "compatible")))
        if not any(c in MIC_TRANSPORTS for c in compatible) or not _okay(d, root):
            continue
        names = _strs(_read(os.path.join(d, "dma-names"))) if "dma-names" in files else []
        chassis = _str(_read(os.path.join(d, "apple,chassis-name"))) if "apple,chassis-name" in files else None
        found.append({"node": os.path.relpath(d, root), "compatible": compatible, "dma_names": names, "chassis": chassis})
    found = [m for m in found if mic_transport(m)["dma"] in m["dma_names"]]
    if not found:
        return None
    if len(found) != 1:
        raise Refused(f"{len(found)} AOP audio nodes with a microphone array in the device tree (want 1)")
    return found[0]


def read_mic_dtb(path):
    """read_mic_dt() on a compiled device tree (.dtb)."""
    with tempfile.TemporaryDirectory() as d:
        root = os.path.join(d, "dt")
        os.makedirs(root)
        _fdt_to_dir(_read(path), root)
        return read_mic_dt(root)


def mic_transport(mic):
    for c in mic["compatible"]:
        if c in MIC_TRANSPORTS:
            return MIC_TRANSPORTS[c]
    raise Refused(f"no microphone transport is known for {mic['compatible']}")


def describe_mic(mic):
    """The array's description, from the device tree's node alone (the
    driver's rules): {key, card, dev, channels, rate}. The key is the
    chassis the card is named for, as the speaker graph's machine key."""
    t = mic_transport(mic)
    chassis = mic.get("chassis")
    if not chassis or not re.fullmatch(r"[A-Za-z0-9]+", chassis):
        raise Refused(f"the device tree's {mic['node']} node names no chassis (apple,chassis-name {chassis!r})")
    if not 1 <= t["channels"] <= MIC_PLUGIN_MAX:
        raise Refused(f"no microphone plugin takes {t['channels']} channels")
    return {"key": chassis.lower(), "card": f"Apple{chassis}{t['card']}", "dev": t["dev"],
            "channels": t["channels"], "rate": t["rate"]}


def live_mic_pcm(desc, proc="/proc/asound"):
    """The array's capture PCM device on the running card `desc` names: the
    card's capture PCM whose id is MIC_PCM_ID (as the speaker PCM is found
    by its id), else the driver's fixed one if the card has it. None
    without the card or the PCM."""
    card = os.path.join(proc, desc["card"])
    try:
        names = sorted(os.listdir(card))
    except OSError:
        return None
    fixed = None
    for d in names:
        m = re.fullmatch(r"pcm(\d+)c", d)
        if not m:
            continue
        try:
            with open(os.path.join(card, d, "info")) as f:
                info = dict(l.split(": ", 1) for l in f.read().splitlines() if ": " in l)
        except OSError:
            continue
        if re.sub(r"\s*\(\*\)$", "", info.get("id", "").strip()) == MIC_PCM_ID:
            return int(m.group(1))
        if int(m.group(1)) == desc["dev"]:
            fixed = desc["dev"]
    return fixed


# The microphone blob's header (crates/sgv2-core/src/mic/artifact.rs), as
# far as provisioning binds it. MIC_BLOB_VERSION is the artifact's VERSION:
# the plugin refuses any other (silence), so a blob of another version is
# never kept (test_provision checks the two constants agree).
MIC_BLOB_MAGIC = b"SGV2MIC\0"
MIC_BLOB_VERSION = 2
MIC_BLOB_HEADER = 76
MIC_BLOB_MAX = 4 * 1024 * 1024
MIC_FLAG_PARTIAL = 1


def mic_blob_header(data):
    """{version, partial, rate, mics, latency, machine, unit_bound} of a
    microphone blob whose framing holds (magic, lengths, CRC-32). Refuses
    anything else. The version is reported, not judged."""
    import zlib

    if len(data) < MIC_BLOB_HEADER + 4 or len(data) > MIC_BLOB_MAX or data[:8] != MIC_BLOB_MAGIC:
        raise Refused("not a microphone blob (length or magic)")
    u32 = lambda at: int.from_bytes(data[at : at + 4], "little")  # noqa: E731
    if MIC_BLOB_HEADER + u32(72) + 4 != len(data):
        raise Refused("the microphone blob's payload length does not match its size")
    if zlib.crc32(data[:-4]) != u32(len(data) - 4):
        raise Refused("the microphone blob's checksum does not match")
    machine = data[28:36].split(b"\0", 1)[0]
    try:
        machine = machine.decode("ascii")
    except UnicodeDecodeError:
        raise Refused("the microphone blob names no board") from None
    return {"version": u32(8), "partial": bool(u32(12) & MIC_FLAG_PARTIAL), "rate": u32(16), "mics": u32(20),
            "latency": u32(24), "machine": machine, "unit_bound": any(data[40:72])}


def mic_blob_errors(h, board, desc):
    """Why the plugin's variant for `desc` on `board` would refuse (or must
    not run) a blob with header `h`: [] when it is this Mac's, complete, and
    of the version this package's plugin takes."""
    errs = []
    if h["version"] != MIC_BLOB_VERSION:
        errs.append(f"blob version {h['version']}, the plugin takes {MIC_BLOB_VERSION}")
    if h["partial"]:
        errs.append("a partial design (stages carried as identity)")
    if h["machine"] != board:
        errs.append(f"compiled for {h['machine'] or 'no board'}, this Mac is {board}")
    if h["rate"] != desc["rate"]:
        errs.append(f"compiled for {h['rate']} Hz, the microphone PCM runs at {desc['rate']} Hz")
    if h["mics"] != desc["channels"]:
        errs.append(f"compiled for {h['mics']} microphones, the microphone PCM has {desc['channels']}")
    return errs


MIC_PLUGIN_URI = "https://aurora-silicon.org/lv2/sgv2#mic-{}ch"
# Where the packaged bundle's library is, under a system LV2 directory.
MIC_PLUGIN_LIB = os.path.join("sgv2.lv2", "libsgv2_lv2.so")


def mic_plugin_health(lib, blob, mics, rate, frames=512):
    """Loads the packaged plugin's microphone variant for `mics` channels as
    a host does, on blob `blob`, runs one block of silence and returns its
    (health, latency) outputs (health 0: it runs the blob; 1: it refused it
    and is silent).
    The plugin reads the board from this Mac's device tree. Refuses when
    the library or the variant cannot be loaded. Run in a process of its
    own (speakerguardd-selfcheck mic): it loads native code."""
    import ctypes

    class Descriptor(ctypes.Structure):
        _fields_ = [("uri", ctypes.c_char_p),
                    ("instantiate", ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double, ctypes.c_char_p, ctypes.c_void_p)),
                    ("connect_port", ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p)),
                    ("activate", ctypes.CFUNCTYPE(None, ctypes.c_void_p)),
                    ("run", ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_uint32)),
                    ("deactivate", ctypes.CFUNCTYPE(None, ctypes.c_void_p)),
                    ("cleanup", ctypes.CFUNCTYPE(None, ctypes.c_void_p)),
                    ("extension_data", ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_char_p))]

    try:
        so = ctypes.CDLL(lib)
        entry = so.lv2_descriptor
    except (OSError, AttributeError) as e:
        raise Refused(f"the plugin library {lib} cannot be loaded ({e})") from None
    entry.restype = ctypes.POINTER(Descriptor)
    entry.argtypes = [ctypes.c_uint32]
    uri = MIC_PLUGIN_URI.format(mics).encode()
    desc = None
    for i in range(256):
        d = entry(i)
        if not d:
            break
        if d.contents.uri == uri:
            desc = d
            break
    if desc is None:
        raise Refused(f"{lib} has no {uri.decode()} plugin")
    old = os.environ.get("SGV2_MIC_PARAMS")
    os.environ["SGV2_MIC_PARAMS"] = blob
    try:
        h = desc.contents.instantiate(desc, float(rate), os.path.dirname(lib).encode(), None)
    finally:
        if old is None:
            del os.environ["SGV2_MIC_PARAMS"]
        else:
            os.environ["SGV2_MIC_PARAMS"] = old
    if not h:
        raise Refused(f"the {uri.decode()} plugin did not instantiate")
    try:
        # ports: the microphones, the output, latency, health
        audio = [(ctypes.c_float * frames)() for _ in range(mics + 1)]
        latency, health = ctypes.c_float(-1.0), ctypes.c_float(-1.0)
        for i, buf in enumerate(audio):
            desc.contents.connect_port(h, i, ctypes.cast(buf, ctypes.c_void_p))
        desc.contents.connect_port(h, mics + 1, ctypes.cast(ctypes.byref(latency), ctypes.c_void_p))
        desc.contents.connect_port(h, mics + 2, ctypes.cast(ctypes.byref(health), ctypes.c_void_p))
        if desc.contents.activate:
            desc.contents.activate(h)
        desc.contents.run(h, frames)
        return int(health.value), int(latency.value)
    finally:
        desc.contents.cleanup(h)


def mic_plugin_lib(dirs):
    """The packaged plugin library, in the first of the system LV2
    directories `dirs` that has it."""
    for d in dirs:
        lib = os.path.join(d, MIC_PLUGIN_LIB)
        if os.path.isfile(lib):
            return lib
    raise Refused("the packaged sgv2 LV2 plugin is in none of " + ", ".join(dirs))


def main(argv):
    import argparse

    ap = argparse.ArgumentParser(prog="wiring.py")
    ap.add_argument("cmd", choices=["derive", "hardware", "describe", "caps"])
    ap.add_argument("--live", action="store_true", help="caps: read the running card too (read-only)")
    ap.add_argument("--dt", default="/proc/device-tree")
    ap.add_argument("--dtb", help="a compiled device tree (.dtb) instead of the live one")
    ap.add_argument("--hw", help="a hardware description (JSON) instead of the live device tree")
    ap.add_argument("--tuning-json")
    ap.add_argument("--tuning-cli", default="aurora-speaker-tuning")
    ap.add_argument("--key")
    ap.add_argument("rest", nargs="*", help="after --: the tuning reader's source arguments")
    a = ap.parse_args(argv)
    try:
        hw = json.load(open(a.hw)) if a.hw else read_dtb(a.dtb) if a.dtb else read_dt(a.dt)
        if a.cmd == "hardware":
            print(json.dumps({"hw": hw, "amps": hardware(hw)}, indent=1))
            return 0
        if a.cmd == "caps":
            live = live_card(hw, a.dt) if a.live else None
            if a.live and live is None:
                raise Refused(f"no ALSA card is bound to the device tree's {hw['sound']} node")
            w = None
            if a.tuning_json or a.rest:
                w = derive(hw, json.load(open(a.tuning_json)) if a.tuning_json else tuning_dump(a.rest, a.tuning_cli), a.key)
            print(json.dumps(caps_report(probe(hw, live, w)), indent=1))
            return 0
        dump = json.load(open(a.tuning_json)) if a.tuning_json else tuning_dump(a.rest, a.tuning_cli)
        w = derive(hw, dump, a.key)
        if a.cmd == "describe":
            w = {"wiring": w, "machine": describe(hw, w)}
        print(json.dumps(w, indent=1))
        return 0
    except Refused as e:
        print(f"wiring: refused: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
