#!/usr/bin/python3 -I
"""The speaker wiring of a Mac, derived from its device tree and its tuning.

    wiring.py {derive|describe} [--dt ROOT | --dtb FILE.dtb | --hw HW.json]
                                (--tuning-json DUMP.json | -- TUNING-CLI-ARGS...) [--key KEY]
    wiring.py hardware [--dt ROOT | --dtb FILE.dtb | --hw HW.json]

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
import subprocess
import sys
import tempfile

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
#   speaker service's DMA cadence underran without 256).
TRANSPORTS = {
    # sound/soc/apple/mca.c: S16_LE, S24_LE, S32_LE
    "apple,mca": {"transport": "mca", "formats": ["S16_LE", "S24_LE", "S32_LE"], "sense_batch": True, "speaker_headroom": 0},
    # sound/soc/apple/aop_spkr.rs (T6040 speaker service): S24_LE playback, S16_LE sense
    "apple,t6041-aop-audio": {"transport": "aop-spkr", "formats": ["S24_LE"], "sense_batch": False, "speaker_headroom": 256},
    # sound/soc/apple/aop_t8140.rs: S32_LE playback, no sense
    "apple,t8140-aop-audio": {"transport": "aop-t8140", "formats": ["S32_LE"], "sense_batch": True, "speaker_headroom": 0},
    # t8152_aop_audio.rs, T8142 configuration: ADMAC burst-residue DMA,
    # S24_LE playback and S16_LE sense, not the T6041 firmware ring above.
    "apple,t8142-aop-audio": {"transport": "aop-dma", "formats": ["S24_LE"], "sense_batch": False, "speaker_headroom": 0},
}
# Newer device trees omit the generic MCA fallback. Both compatibles bind
# mca.c; share the entry so back-ends using either name are the same driver.
TRANSPORTS["apple,t8103-mca"] = TRANSPORTS["apple,mca"]
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
        cpu_compat = []
        for node in cpu:
            f = os.path.join(node, "compatible")
            cpu_compat.append(_strs(_read(f)) if os.path.exists(f) else [])
        links.append(
            {"name": _str(_read(os.path.join(p, "link-name"))), "cpus": len(cpu), "cpu_compatible": cpu_compat, "codecs": codecs}
        )
    return links


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


def describe(hw, w):
    """Everything the protect stage's graph needs that is not the channel
    map: derived from the device tree (and the wiring derived from it),
    and the tuning's sample rate.
    The self-check checks the card id and the format against ALSA."""
    t = transport(hw)
    # LSID is a side-chain inside the existing LSM2 governor. Plain LSM2
    # bundles still have no sense ports; only an explicit companion requests
    # the V/I variant. The self-check proves the sense map either way.
    n = len(w["amps"])
    takes_sense = w.get("unit_kind") in (None, *SENSE_UNITS) or w.get("sense_companion", False)
    s = len(w["sense"]) if takes_sense else 0
    plugin = PLUGINS.get((n, s))
    if plugin is None:
        raise Refused(f"no sgv2 plugin variant for {n} amps and {s} sense channels")
    fmt = speaker_format(t)
    return {
        "key": machine_key(hw),
        "board": hw["board"],
        "card": card_id(hw),
        "name": hw.get("model") or card_id(hw),
        "transport": t["transport"],
        "amps": n,
        "sense": s,
        "rate": w["rate"],
        "plugin": plugin,
        "alsa_format": fmt,
        "speaker_format": PW_FORMAT[fmt],
        "speaker_headroom": t["speaker_headroom"],
        "sense_headroom": SENSE_HEADROOM if s else 0,
        "sense_batch": t["sense_batch"],
    }


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


def hardware(hw):
    """The speaker amps in physical (speaker PCM) order, each with its
    side, role, ordinal, slot and sense channels."""
    link = _speaker_link(hw)
    codecs = link["codecs"]
    if not codecs:
        raise Refused("the speaker dai-link lists no amps")
    slots = tdm_slots(len(codecs), link["cpus"])
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
        if c.get("i2c"):
            a["i2c"] = c["i2c"]
        if fam[1]:
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
    if next(iter(families))[1]:
        chans = sorted(ch for a in amps for ch in (a["v_ch"], a["i_ch"]))
        if chans != list(range(2 * len(amps))):
            raise Refused(f"the sense slots give channels {chans}, not 0..{2 * len(amps) - 1} once each")
    return amps


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


def tuning_dump(cli_args, cli="aurora-speaker-tuning", strict=False):
    """Run the tuning reader's JSON dump; its exit code reports refusals,
    which offline derivation does not need. Qualification uses strict=True
    to require success and an empty refusal/error set."""
    r = subprocess.run([cli, "dump", "--json", *cli_args], capture_output=True, text=True, timeout=60)
    if strict and r.returncode != 0:
        raise Refused(f"{cli} dump refused the tuning (exit {r.returncode})")
    try:
        result = json.loads(r.stdout)
        if strict:
            protection_units(result)
        return result
    except ValueError:
        raise Refused(f"{cli} dump --json gave no JSON (exit {r.returncode}): {r.stderr.strip()[-300:]}")


def main(argv):
    import argparse

    ap = argparse.ArgumentParser(prog="wiring.py")
    ap.add_argument("cmd", choices=["derive", "hardware", "describe"])
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
