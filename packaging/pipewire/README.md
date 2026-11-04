# PipeWire templates

Generated per machine by `packaging/session/gen.py`. Do not edit `sgv2-<machine>.conf` by hand. Each machine's channel map and V/I order are derived (`packaging/session/wiring.py`) from its device tree and its tuning's graph, recorded in `packaging/session/fixtures/`; `machines.json` holds the rest of the entry, and its `unit_channels`/`sense_pairs`, where present, must agree with the derivation. On a Mac with no entry, `speakerguardd-selfcheck generate` writes these files from its record. The protect stage carries the map's hash as `sgv2.wiring`, which the supervisor requires to equal the self-check's (`docs/NEW-PLATFORM.md`).

Each file holds:
- the speaker amps' ALSA sink (`hw:CARD=<card>,DEV=1`);
- the sense source (`DEV=2`), in one `node.group`, at a 512-frame quantum;
- the protect filter-chain around one `sgv2` LV2 instance. Its input is named `alsa_output.platform-sound.RawSpeakers`, the name the machine's voicing graph targets.

See `gen.py` for the whole graph and `../session/speakerguardd-session` for the links and the report.

TODO(lane B): replace the interim voicing (the aurora-audio graph of the machine) with the generated voicing-pre.
