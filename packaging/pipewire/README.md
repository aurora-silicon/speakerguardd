# PipeWire templates

Generated per machine by `packaging/session/gen.py` from `packaging/session/machines.json`. Do not edit `sgv2-<machine>.conf` by hand.

Each file holds:
- the speaker amps' ALSA sink (`hw:CARD=<card>,DEV=1`);
- the sense source (`DEV=2`), in one `node.group`, at a 512-frame quantum;
- the protect filter-chain around one `sgv2` LV2 instance. Its input is named `alsa_output.platform-sound.RawSpeakers`, the name the machine's voicing graph targets.

See `gen.py` for the whole graph and `../session/speakerguardd-session` for the links and the report.

TODO(lane B): replace the interim voicing (the aurora-audio graph of the machine) with the generated voicing-pre.
