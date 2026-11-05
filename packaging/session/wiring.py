#!/usr/bin/python3 -I
"""The speaker wiring of a Mac, derived from its device tree and its tuning.

    wiring.py derive [--dt ROOT | --hw HW.json] (--tuning-json DUMP.json | -- TUNING-CLI-ARGS...)
                     [--machines machines.json] [--key KEY]

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

machines.json is an optional override: an entry states a whole hand map
(unit_channels, default the identity; sense_pairs "vi" | "iv", default
"vi"), and `disagreements()` lists every place it and the derivation
differ. An entry with `"wiring": "derive"` states no map. Only an entry with
`"wiring": "hand-by-design"` (a board whose wiring cannot be derived by
design) lets the self-check run on its hand map when the derivation
refuses; any other board fails instead.
"""
import hashlib
import json
import os
import re
import subprocess
import sys

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

_PREFIX = re.compile(r"^(?:(Left|Right)\s+)?(Woofer|Tweeter|Speaker|Full[- ]Range)(?:\s+(\d+))?$")
_ROLES = {"woofer": "woofer", "tweeter": "tweeter", "speaker": "full", "full-range": "full", "full range": "full"}


def parse_prefix(prefix):
    """(side, role, ordinal) from a sound-name-prefix; side None = centre."""
    m = _PREFIX.match(prefix.strip())
    if not m:
        raise Refused(f"amp prefix {prefix!r} names no side and role this derivation knows")
    side = {"Left": "L", "Right": "R", None: None}[m.group(1)]
    role = _ROLES[m.group(2).lower()]
    return side, role, int(m.group(3) or 1)


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
        links.append({"name": _str(_read(os.path.join(p, "link-name"))), "cpus": len(cpu), "codecs": codecs})
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
        "model": _str(_read(model)) if os.path.exists(model) else None,
        "links": links,
    }


def board_of(compatible):
    first = compatible[0] if compatible else ""
    if not first.startswith("apple,") or first == "apple,":
        raise Refused(f"root compatible {first!r} is not an Apple board")
    return first[len("apple,") :]


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
    spk = [l for l in hw["links"] if l["name"] in ("Speaker", "Speakers")]
    if len(spk) != 1:
        raise Refused(f"{len(spk)} speaker dai-links in the device tree (want 1)")
    link = spk[0]
    codecs = link["codecs"]
    if not codecs:
        raise Refused("the speaker dai-link lists no amps")
    slots = tdm_slots(len(codecs), link["cpus"])
    amps, families = [], set()
    for c, slot in zip(codecs, slots):
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
    if len(amps) == 1 and amps[0]["side"] is None:
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
        out.append(("LR"[ch], tags.pop() if tags else "full"))
    rs = {r for _, r in out}
    if "full" in rs and len(rs) > 1:
        raise Refused("some unit channels pass a crossover branch and some do not")
    return {"kind": p.get("kind"), "aid": t.get("aid"), "channels": out}


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
        "key": key or hw["board"],
        "model": hw.get("model"),
        "family": amps[0]["family"],
        "aid": rl["aid"],
        "unit_kind": rl["kind"],
        "amps": amps,
        "unit_channels": [a["unit"] for a in amps],
        "sense": sense,
        "sense_ports": [s["port"] for s in sense],
    }
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


def sense_ports_from_pairs(unit, pairs, amps):
    """The legacy machines.json form: the sense PCM carries the amps in
    physical order, two channels each, V first ("vi") or I first ("iv")."""
    if pairs not in ("vi", "iv"):
        raise Refused(f"sense_pairs {pairs!r} is neither 'vi' nor 'iv'")
    ports = []
    for k in range(2 * amps):
        is_v = (k % 2 == 0) == (pairs == "vi")
        ports.append(2 * unit[k // 2] + (0 if is_v else 1))
    return ports


def apply_override(w, entry):
    """The wiring with a machines.json entry's hand map applied, and the
    list of every place the hand map and the derivation disagree.

    An entry states a whole map, with the defaults gen.py has always used:
    unit_channels the identity, sense_pairs "vi". (The J616s template said
    nothing, so it said [V, I], and that was wrong.) An entry with
    `"wiring": "derive"` states none, and the derivation stands alone."""
    if entry.get("wiring") == "derive":
        out = json.loads(json.dumps(w))
        out["override"] = []
        return out, []
    out = json.loads(json.dumps(w))
    n = len(w["amps"])
    unit = entry.get("unit_channels", list(range(n)))
    if sorted(unit) != list(range(n)):
        raise Refused(f"override unit_channels {unit} is not a permutation of {n} channels")
    out["unit_channels"] = list(unit)
    for a, u in zip(out["amps"], unit):
        a["unit"] = u
    ports = sense_ports_from_pairs(out["unit_channels"], entry.get("sense_pairs", "vi"), n) if w["sense"] else []
    out["sense_ports"] = ports
    for s, port in zip(out["sense"], ports):
        s["port"] = port
        # what the hand map claims each sense channel carries
        s["amp"] = out["unit_channels"].index(port // 2)
        s["kind"] = "V" if port % 2 == 0 else "I"
    out["override"] = sorted(k for k in ("unit_channels", "sense_pairs") if k in entry) or ["defaults"]
    out["hash"] = wiring_hash(out)
    return out, disagreements(w, out)


def disagreements(derived, effective):
    out = []
    if derived["unit_channels"] != effective["unit_channels"]:
        out.append(
            "unit_channels: the hand map says %s, the device tree and tuning say %s"
            % (effective["unit_channels"], derived["unit_channels"])
        )
    if derived["sense_ports"] != effective["sense_ports"]:
        out.append(
            "sense_ports: the hand map says %s, the device tree says %s"
            % (effective["sense_ports"], derived["sense_ports"])
        )
    return out


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


def machine_entry(machines, board):
    """The machines.json key and entry for a board ("j316s" -> "j316")."""
    for k, c in machines.items():
        if board in c.get("boards", [k]) or board == k:
            return k, c
    return None, None


def tuning_dump(cli_args, cli="aurora-speaker-tuning"):
    """Run the tuning reader's JSON dump; its exit code reports refusals,
    which the derivation does not need, so the JSON is what counts."""
    r = subprocess.run([cli, "dump", "--json", *cli_args], capture_output=True, text=True, timeout=60)
    try:
        return json.loads(r.stdout)
    except ValueError:
        raise Refused(f"{cli} dump --json gave no JSON (exit {r.returncode}): {r.stderr.strip()[-300:]}")


def main(argv):
    import argparse

    ap = argparse.ArgumentParser(prog="wiring.py")
    ap.add_argument("cmd", choices=["derive", "hardware"])
    ap.add_argument("--dt", default="/proc/device-tree")
    ap.add_argument("--hw", help="a hardware description (JSON) instead of the live device tree")
    ap.add_argument("--tuning-json")
    ap.add_argument("--tuning-cli", default="aurora-speaker-tuning")
    ap.add_argument("--machines")
    ap.add_argument("--key")
    ap.add_argument("rest", nargs="*", help="after --: the tuning reader's source arguments")
    a = ap.parse_args(argv)
    try:
        hw = json.load(open(a.hw)) if a.hw else read_dt(a.dt)
        if a.cmd == "hardware":
            print(json.dumps({"hw": hw, "amps": hardware(hw)}, indent=1))
            return 0
        dump = json.load(open(a.tuning_json)) if a.tuning_json else tuning_dump(a.rest, a.tuning_cli)
        w = derive(hw, dump, a.key)
        if a.machines:
            k, entry = machine_entry(json.load(open(a.machines)), hw["board"])
            if entry:
                w["key"] = k
                w, dis = apply_override(w, entry)
                w["disagreements"] = dis
        print(json.dumps(w, indent=1))
        return 1 if w.get("disagreements") else 0
    except Refused as e:
        print(f"wiring: refused: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
