# WirePlumber rules

Generated per machine by `packaging/session/gen.py`. Each `60-sgv2-<machine>.conf`:
- disables WirePlumber's own nodes for the speaker and sense PCMs, so no other rule can make a second path to the amps;
- puts the machine's voicing in front of the protect stage, without hiding it, so the session helper can see the stage's ports.

Each `61-sgv2-mic-<machine>.conf` (machines with a microphone array) disables WirePlumber's own node for the array's PCM, to which asahi-audio attaches its microphone filter. `speakerguardd-provision` writes it at runtime, together with `../pipewire/sgv2-mic-<machine>.conf`, only while this Mac's microphone blob is valid; see `../pipewire/README.md`.
