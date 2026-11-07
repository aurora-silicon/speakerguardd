#!/usr/bin/env python3
"""Generate the per-machine PipeWire and WirePlumber files of the v2 speaker graph.

    python3 -I packaging/session/gen.py            # rewrite the files
    python3 -I packaging/session/gen.py --check    # fail if any file is stale

Every value comes from the machine's own device tree and tuning, recorded in
fixtures/<machine>.json (public device-tree facts and the lane structure of
the tuning's graph, no tuning values): the channel map (wiring.derive) and
the rest of the description (wiring.describe: the card, the model, the amp
and sense counts, the plugin variant, the speaker format and the
transport's buffering). Nothing is written by hand per board. On a Mac with
no fixture, speakerguardd-selfcheck derives the same from the live device
tree, checks it against ALSA, and generates the files.

The graph, per machine:

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
PROTECT_INPUT = "alsa_output.platform-sound.RawSpeakers"


def voicing_enabled(board, gate=None):
    """Per-board deployment gate. Missing keys/boards keep the interim graph.
    Values must be literal JSON booleans; strings never enable a board.
    """
    if gate is None:
        with open(os.path.join(HERE, "gate.json")) as f:
            gate = json.load(f)
    return gate.get("voicing_pre", {}).get(board) is True


def voicing_bound(board, gate=None):
    """Interim never-louder bound for a board's generated voicing: one gain
    in dB (<= 0) per unit lane, applied after the voicing plugin, from
    gate.json voicing_pre_bound_db. It exists only while a voicing law is
    still open (it keeps every lane at or below macOS on the full-chain
    safety scan) and goes when the law is exact. Anything malformed, any
    positive value or a wrong lane count refuses the board's voicing.
    """
    if gate is None:
        with open(os.path.join(HERE, "gate.json")) as f:
            gate = json.load(f)
    b = gate.get("voicing_pre_bound_db", {}).get(board)
    if b is None:
        return None
    if not isinstance(b, list) or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and -12.0 <= x <= 0.0 for x in b):
        raise ValueError(f"voicing_pre_bound_db for {board}: want a list of dB values in [-12, 0]")
    return [float(x) for x in b]


def voicing_conf(m, c, w):
    """One stereo chain, in unit-lane order, mapped to physical AUX ports
    with the same map as the protection input. Volume is handled by the
    chain control, so PipeWire must not attenuate the audio a second time.
    """
    amps = c["amps"]
    unit = w[0]
    bound = voicing_bound(c.get("board", m))
    if bound is not None and len(bound) != amps:
        raise ValueError(f"voicing_pre_bound_db for {c.get('board', m)}: {len(bound)} lanes, the board has {amps}")
    trims = {k: bound[k] for k in range(amps) if bound and bound[k] < 0.0}
    outputs = " ".join(f'"b{unit[p]}:Out"' if unit[p] in trims else f'"voicing:out_{unit[p]}"' for p in range(amps))
    extra_nodes = "".join(
        f' {{ type = builtin name = b{k} label = linear control = {{ "Mult" = {10.0 ** (t / 20.0):.9f} "Add" = 0.0 }} }}'
        for k, t in sorted(trims.items())
    )
    links = "".join(f' {{ output = "voicing:out_{k}" input = "b{k}:In" }}' for k in sorted(trims))
    links_line = f"\n               links = [{links} ]" if trims else ""
    return f"""
    {{ name = libpipewire-module-filter-chain
       flags = [ nofail ]
       args = {{
           node.description = "{c['name']} speakers"
           media.name = "{c['name']} speakers"
           filter.graph = {{
               nodes = [ {{ type = lv2 name = voicing plugin = "https://aurora-silicon.org/lv2/sgv2#voicing-{amps}ch" }}{extra_nodes} ]{links_line}
               inputs = [ "voicing:in_0" "voicing:in_1" ]
               outputs = [ {outputs} ]
               capture.volumes = [ {{ control = "voicing:volume" min = 0.0 max = 1.0 scale = cubic }} ]
           }}
           capture.props = {{
               node.name = "sgv2.voicing.{m}"
               node.description = "{c['name']} speakers"
               media.class = "Audio/Sink"
               audio.rate = {c['rate']}
               audio.channels = 2
               audio.position = [ FL FR ]
               priority.session = 1000
               sgv2.stage = "voicing-pre"
               sgv2.machine = "{m}"
           }}
           playback.props = {{
               node.name = "sgv2.voicing.{m}.out"
               target.object = "{PROTECT_INPUT}"
               audio.rate = {c['rate']}
               audio.channels = {amps}
               audio.position = {aux(amps)}
               node.passive = true
               stream.dont-remix = true
               channelmix.disable = true
               resample.disable = true
           }}
       }}
    }}
"""


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


def fixture(m):
    """The machine's recorded device tree and tuning trace, if any."""
    path = os.path.join(HERE, "fixtures", f"{m}.json")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def machines():
    """The machines with a fixture, by key."""
    d = os.path.join(HERE, "fixtures")
    return sorted(f[: -len(".json")] for f in os.listdir(d) if f.endswith(".json"))


def machine(m):
    """(description, (unit_channels, sense_ports, wiring hash)) of a
    machine, derived from its fixture."""
    fx = fixture(m)
    if fx is None:
        raise SystemExit(f"{m}: no fixture")
    try:
        w = wiring.derive(fx["hw"], fx["tuning"])
        c = wiring.describe(fx["hw"], w)
    except wiring.Refused as e:
        raise SystemExit(f"{m}: {e}")
    if c["key"] != m:
        raise SystemExit(f"fixtures/{m}.json: its device tree names machine {c['key']}")
    return c, (w["unit_channels"], w["sense_ports"], w["hash"])


def pipewire_conf(m, c, w, source=None, voicing_pre=None):
    """The PipeWire file for description `c` (wiring.describe) and map `w`
    (unit_channels, sense_ports, wiring hash)."""
    enabled = voicing_enabled(c.get("board", m)) if voicing_pre is None else voicing_pre
    n = names(m)
    amps, sense = c["amps"], c["sense"]
    uri = f"https://aurora-silicon.org/lv2/sgv2#{c['plugin']}"
    # filter-chain feeds capture channel i to inputs[i] and playback channel
    # j from outputs[j]: the lists put each physical channel on its unit port.
    unit, sense_ports, whash = w[:3]
    # the graph carries sense only to a unit that takes it (wiring.describe)
    sense_ports = sense_ports if sense else []
    inputs = [f'"sgv2:in_{unit[p]}"' for p in range(amps)] + [f'"sgv2:sense_{s}"' for s in sense_ports]
    outputs = [f'"sgv2:out_{unit[p]}"' for p in range(amps)]
    sense_obj = ""
    # A sense PCM whose pointer advances in small chunks (the AOP sense ring:
    # 120 frames) is not period-batched: PipeWire's batch handling would
    # halve its period to 256 and add 256 frames of headroom, and its
    # follower then resyncs (drops frames) on ordinary chunk jitter.
    sense_batch = "" if c["sense_batch"] else "\n            api.alsa.disable-batch = true"
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
            audio.rate             = {c['rate']}
            audio.channels         = {sense}
            audio.position         = {aux(sense)}
            api.alsa.period-size   = {QUANTUM}
            api.alsa.headroom      = {c['sense_headroom']}{sense_batch}
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
    text = f"""# Generated by packaging/session/gen.py from {source or f'fixtures/{m}.json'}; do not edit.
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
            audio.rate             = {c['rate']}
            audio.channels         = {amps}
            audio.position         = {aux(amps)}
            api.alsa.period-size   = {QUANTUM}
            api.alsa.headroom      = {c['speaker_headroom']}
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
                audio.rate         = {c['rate']}
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
                audio.rate         = {c['rate']}
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
    return text.rsplit("]", 1)[0] + voicing_conf(m, c, w) + "]\n" if enabled else text


RAW_RULES = "59-sgv2-raw.conf"


def wireplumber_raw_conf(card):
    """Only the rules that keep WirePlumber off the raw speaker and sense
    PCMs (devices 1 and 2 of the speaker card): written by provisioning on
    every v2 boot and linked for every user from the start of the session,
    before the self-check has generated the graph. Without them a session
    that starts during the self-check probes the raw PCMs (open, hw_params,
    prepare, close on the shared speaker port), which latched 'bad TDM
    clock' on an amp of the MacBook Air M2 (KFIX 11-07)."""
    return f"""# Generated by speakerguardd-provision; do not edit.
# speakerguardd v2 owns the speaker card {card}: no WirePlumber node for its raw speaker or sense PCM.

monitor.alsa.rules = [
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
"""


def wireplumber_conf(m, c, source=None, voicing_pre=None):
    enabled = voicing_enabled(c.get("board", m)) if voicing_pre is None else voicing_pre
    card = c["card"]
    text = f"""# Generated by packaging/session/gen.py from {source or f'fixtures/{m}.json'}; do not edit.
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
    if enabled:
        # Do not create the interim software-dsp filter as well.
        text = text[:text.index("node.software-dsp.rules = [")] + "node.software-dsp.rules = []\n"
    return text


def outputs():
    for m in machines():
        c, w = machine(m)
        yield os.path.join(PKG, "pipewire", f"sgv2-{m}.conf"), pipewire_conf(m, c, w)
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
