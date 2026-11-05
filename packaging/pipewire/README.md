# PipeWire templates

Generated per machine by `packaging/session/gen.py`. Do not edit `sgv2-<machine>.conf` by hand. Every value is derived (`packaging/session/wiring.py`) from the machine's device tree and its tuning's graph, recorded in `packaging/session/fixtures/`: the channel map and V/I order (`derive`), and the card, model, amp and sense counts, plugin variant, speaker format and buffering (`describe`, the last two per transport driver). Nothing is written by hand per board. On any Mac, `speakerguardd-selfcheck generate` writes these files from its record, derived from the live device tree and checked against ALSA. The protect stage carries the map's hash as `sgv2.wiring`, which the supervisor requires to equal the self-check's (`docs/NEW-PLATFORM.md`).

Each file holds:
- the speaker amps' ALSA sink (`hw:CARD=<card>,DEV=1`);
- the sense source (`DEV=2`), in one `node.group`, at a 512-frame quantum;
- the protect filter-chain around one `sgv2` LV2 instance. Its input is named `alsa_output.platform-sound.RawSpeakers`, the name the machine's voicing graph targets.

See `gen.py` for the whole graph and `../session/speakerguardd-session` for the links and the report.

TODO(lane B): replace the interim voicing (the aurora-audio graph of the machine) with the generated voicing-pre.
