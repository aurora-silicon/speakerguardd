#!/usr/bin/env python3
"""Generate the per-machine PipeWire and WirePlumber files of the v2 speaker graph.

    python3 -I packaging/session/gen.py            # rewrite the files
    python3 -I packaging/session/gen.py --check    # fail if any file is stale

The channel map of each machine is derived (wiring.py) from its device tree
and its tuning's graph, recorded in fixtures/<machine>.json. A machines.json
entry states a hand map (unit_channels, default the identity; sense_pairs,
default "vi"; none with "wiring": "derive"), and the derivation must agree,
or nothing is generated. A machine with no fixture uses its hand map. On a
Mac with no entry at all,
speakerguardd-selfcheck derives the map live and generates the files.

The graph, per machine (machines.json):

    voicing sink (the user's speakers) ──► protect stage ──► speaker amps (ALSA)
                                               ▲
                                  sense (ALSA) ┘

- The voicing stage is the machine's voicing graph, loaded by WirePlumber's
  software-dsp in front of the protect stage. Its output stream targets the
  node named alsa_output.platform-sound.RawSpeakers, which on v2 is the
  protect stage's input.
- The protect stage is a filter-chain around one sgv2 LV2 instance: the
  program on AUX0.. and the amps' V/I sense after it.
- The speaker amps and the sense are ALSA nodes of their own on the speaker
  card's PCMs 1 and 2 (hw:CARD=<card>,DEV=<n>, a name no aurora-audio rule
  matches), in one node.group, so the amps' sink drives the sense at the
  same 512-frame quantum. Their class is internal: no client or policy
  links to them.
- WirePlumber's own node for the speaker PCM is disabled, so aurora-audio's
  rules for it never create a second path to the amps.
- speakerguardd-session makes the links to the amps and the sense only while
  the kernel grants protection, and reports the link set and the carrier to
  the supervisor.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wiring  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
QUANTUM = 512
RATE = 48000
PROTECT_INPUT = "alsa_output.platform-sound.RawSpeakers"


def names(m):
    return {
        "protect": PROTECT_INPUT,
        "protect_out": f"sgv2.protect.{m}.out",
        "speaker": f"sgv2.speaker.{m}",
        "sense": f"sgv2.sense.{m}",
        "group": f"sgv2.speakers.{m}",
    }


def aux(n, base=0):
    return "[ " + " ".join(f"AUX{base + i}" for i in range(n)) + " ]"


# The plugin variants the LV2 bundle provides (crates/sgv2-lv2 VARIANTS):
# (amps, sense channels) -> URI fragment.
PLUGINS = {(2, 0): "2ch", (2, 4): "2ch-vi", (4, 0): "4ch", (4, 8): "4ch-vi", (6, 12): "6ch-vi"}


def channel_map(m, c):
    """The hand-written map of a machines.json entry (no derivation).

    - unit_channels: the unit channel of each physical speaker channel
      (default: the same order);
    - sense_pairs: the order of each amp's two channels on the sense PCM,
      "vi" (default) or "iv". The sense PCM carries the amps in the
      physical speaker order.

    Returns the unit channel of each physical channel, and the unit's sense
    port for each sense PCM channel.
    """
    amps, sense = c["amps"], c["sense"]
    unit = c.get("unit_channels", list(range(amps)))
    if sorted(unit) != list(range(amps)):
        raise SystemExit(f"{m}: unit_channels {unit} is not a permutation of {amps} channels")
    if sense not in (0, 2 * amps):
        raise SystemExit(f"{m}: {sense} sense channels for {amps} amps")
    try:
        ports = wiring.sense_ports_from_pairs(unit, c.get("sense_pairs", "vi"), amps) if sense else []
    except wiring.Refused as e:
        raise SystemExit(f"{m}: {e}")
    return unit, ports


def fixture(m):
    """The machine's recorded device tree and tuning trace, if any
    (fixtures/<m>.json: public device-tree facts and the lane structure of
    the tuning's graph, no tuning values)."""
    path = os.path.join(HERE, "fixtures", f"{m}.json")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def wiring_for(m, c):
    """The protect stage's map for a machines.json entry: derived from the
    machine's fixture where there is one (the entry's hand map, if any, must
    agree), else the entry's hand map.

    Returns (unit_channels, sense_ports, wiring hash, source)."""
    fx = fixture(m)
    if fx is None:
        unit, ports = channel_map(m, c)
        h = wiring.wiring_hash(
            {"unit_channels": unit, "sense_ports": ports, "amps": [{"prefix": f"ch{i}", "slot": i} for i in range(c["amps"])]}
        )
        return unit, ports, h, "hand"
    try:
        w = wiring.derive(fx["hw"], fx["tuning"], key=m)
        eff, dis = wiring.apply_override(w, c)
    except wiring.Refused as e:
        raise SystemExit(f"{m}: {e}")
    if dis:
        raise SystemExit(f"{m}: machines.json disagrees with the derived wiring: " + "; ".join(dis))
    if (len(eff["amps"]), len(eff["sense"])) != (c["amps"], c["sense"]):
        raise SystemExit(f"{m}: the derivation has {len(eff['amps'])} amps and {len(eff['sense'])} sense channels")
    return eff["unit_channels"], eff["sense_ports"], eff["hash"], "derived"


def entry_from_wiring(w, card, speaker_format, base=None):
    """A machines.json-style entry for a board with no entry: everything
    the graph needs from the derived wiring and the running card."""
    n, s = len(w["amps"]), len(w["sense"])
    plugin = PLUGINS.get((n, s))
    if plugin is None:
        raise SystemExit(f"{w['board']}: no sgv2 plugin variant for {n} amps and {s} sense channels")
    c = {
        "card": card,
        "name": w.get("model") or card,
        "amps": n,
        "speaker_format": speaker_format,
        "sense": s,
        "plugin": plugin,
        "sense_headroom": 512,
    }
    c.update(base or {})
    return c


def pipewire_conf(m, c, w=None):
    """The PipeWire file. `w` is (unit_channels, sense_ports, hash), from
    wiring_for() when not given."""
    n = names(m)
    amps, sense = c["amps"], c["sense"]
    uri = f"https://aurora-silicon.org/lv2/sgv2#{c['plugin']}"
    # filter-chain feeds capture channel i to inputs[i] and playback channel
    # j from outputs[j]: the lists put each physical channel on its unit port.
    unit, sense_ports, whash = (w or wiring_for(m, c))[:3]
    inputs = [f'"sgv2:in_{unit[p]}"' for p in range(amps)] + [f'"sgv2:sense_{s}"' for s in sense_ports]
    outputs = [f'"sgv2:out_{unit[p]}"' for p in range(amps)]
    sense_obj = ""
    # A sense PCM whose pointer advances in small chunks (the AOP sense ring:
    # 120 frames) is not period-batched: PipeWire's batch handling would
    # halve its period to 256 and add 256 frames of headroom, and its
    # follower then resyncs (drops frames) on ordinary chunk jitter.
    sense_batch = "" if c.get("sense_batch", True) else "\n            api.alsa.disable-batch = true"
    # The sense is a follower without resampling: PipeWire resyncs it (drops
    # or skips frames, shifting it against the program) when its delay is off
    # target by more than max((quantum + headroom) / 2, 256) frames, so the
    # headroom sets how late a graph cycle may run before the sense shifts.
    if sense:
        sense_obj = f"""
    {{   factory = adapter
        args = {{
            factory.name           = api.alsa.pcm.source
            node.name              = "{n['sense']}"
            node.description       = "{c['name']} speaker sense (protection only)"
            media.class            = "Audio/Source/Internal"
            api.alsa.path          = "hw:CARD={c['card']},DEV=2"
            audio.format           = "S16LE"
            audio.rate             = {RATE}
            audio.channels         = {sense}
            audio.position         = {aux(sense)}
            api.alsa.period-size   = {QUANTUM}
            api.alsa.headroom      = {c.get('sense_headroom', 0)}{sense_batch}
            node.group             = "{n['group']}"
            node.passive           = true
            priority.driver        = 0
            priority.session       = 0
            node.autoconnect       = false
            node.pause-on-idle     = false
            resample.disable       = true
            channelmix.disable     = true
            monitor.channel-volumes = false
        }}
    }}"""
    return f"""# Generated by packaging/session/gen.py from machines.json ({m}); do not edit.
# speakerguardd v2: the protect stage, the speaker amps and the sense of the {c['name']}.

context.objects = [
    {{   factory = adapter
        args = {{
            factory.name           = api.alsa.pcm.sink
            node.name              = "{n['speaker']}"
            node.description       = "{c['name']} speaker amps (protected path only)"
            media.class            = "Audio/Sink/Internal"
            api.alsa.path          = "hw:CARD={c['card']},DEV=1"
            audio.format           = "{c['speaker_format']}"
            audio.rate             = {RATE}
            audio.channels         = {amps}
            audio.position         = {aux(amps)}
            api.alsa.period-size   = {QUANTUM}
            api.alsa.headroom      = {c.get('speaker_headroom', 0)}
            node.group             = "{n['group']}"
            node.force-quantum     = {QUANTUM}
            node.lock-quantum      = true
            priority.driver        = 2000
            priority.session       = 0
            node.autoconnect       = false
            # Idle (unlinked) means paused: the kernel refuses every start
            # without a grant, and a driver kept running here retried its
            # start every quantum after each withdrawal.
            node.pause-on-idle     = true
            resample.disable       = true
            channelmix.disable     = true
            monitor.channel-volumes = false
        }}
    }}{sense_obj}
]

context.modules = [
    {{   name = libpipewire-module-filter-chain
        flags = [ nofail ]
        args = {{
            node.description = "{c['name']} speaker protection"
            media.name       = "{c['name']} speaker protection"
            filter.graph = {{
                nodes = [
                    {{ type = lv2 name = sgv2 plugin = "{uri}" }}
                ]
                inputs  = [ {" ".join(inputs)} ]
                outputs = [ {" ".join(outputs)} ]
            }}
            capture.props = {{
                node.name          = "{n['protect']}"
                node.description   = "{c['name']} speaker protection input (internal)"
                media.class        = "Audio/Sink"
                audio.rate         = {RATE}
                audio.channels     = {amps + sense}
                audio.position     = {aux(amps + sense)}
                priority.session   = 0
                priority.driver    = 0
                node.lock-quantum  = true
                node.pause-on-idle = false
                # Runs from the start, on any driver: the plugin is
                # instantiated and beating before the grant, which waits for
                # it, and before the amps are linked.
                node.always-process = true
                stream.dont-remix  = true
                channelmix.disable = true
                resample.disable   = true
                sgv2.stage         = "protect"
                sgv2.machine       = "{m}"
                # The wiring this stage was built with; the session helper
                # reports it, and the supervisor grants only for the wiring
                # the self-check proved.
                sgv2.wiring        = "{whash}"
            }}
            playback.props = {{
                node.name          = "{n['protect_out']}"
                node.description   = "{c['name']} speaker protection output"
                audio.rate         = {RATE}
                audio.channels     = {amps}
                audio.position     = {aux(amps)}
                node.autoconnect   = false
                node.dont-reconnect = true
                node.passive       = true
                node.pause-on-idle = false
                node.always-process = true
                stream.dont-remix  = true
                channelmix.disable = true
                resample.disable   = true
            }}
        }}
    }}
]
"""


def wireplumber_conf(m, c):
    card = c["card"]
    return f"""# Generated by packaging/session/gen.py from machines.json ({m}); do not edit.
# speakerguardd v2 on the {c['name']}: the only path to the speaker amps is the protect stage.

monitor.alsa.rules = [
    # WirePlumber's own nodes for the speaker PCM and the sense PCM (UCM and
    # pro-audio profiles alike): never created, so no rule (aurora-audio's
    # software-dsp, volume locks) can make a second path to the amps. These
    # rules see only the ALSA monitor's nodes, never sgv2's own.
    {{
        matches = [
            {{ api.alsa.path = "hw:{card},1" }}
            {{ api.alsa.path = "hw:{card},2" }}
            {{ alsa.id = "{card}", alsa.device = 1 }}
            {{ alsa.id = "{card}", alsa.device = 2 }}
        ]
        actions = {{
            update-props = {{
                node.disabled = true
            }}
        }}
    }}
]

node.software-dsp.rules = [
    # The machine's voicing in front of the protect stage. Not hidden: the
    # session helper must see the protect stage's ports.
    {{
        matches = [
            {{ node.name = "{PROTECT_INPUT}", sgv2.stage = "protect", sgv2.machine = "{m}" }}
        ]
        actions = {{
            create-filter = {{
                filter-path = "/usr/share/asahi-audio/{m}/graph.json"
            }}
        }}
    }}
]
"""


def outputs():
    with open(os.path.join(HERE, "machines.json")) as f:
        machines = json.load(f)
    for m, c in sorted(machines.items()):
        yield os.path.join(PKG, "pipewire", f"sgv2-{m}.conf"), pipewire_conf(m, c)
        yield os.path.join(PKG, "wireplumber", f"60-sgv2-{m}.conf"), wireplumber_conf(m, c)


def main():
    check = "--check" in sys.argv[1:]
    stale = 0
    for path, text in outputs():
        old = open(path).read() if os.path.exists(path) else None
        if old == text:
            continue
        if check:
            print(f"stale: {os.path.relpath(path, PKG)}")
            stale += 1
        else:
            with open(path, "w") as f:
                f.write(text)
            print(f"wrote {os.path.relpath(path, PKG)}")
    return 1 if stale else 0


if __name__ == "__main__":
    sys.exit(main())
