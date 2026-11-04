# WirePlumber rules

Generated per machine by `packaging/session/gen.py`. Each `60-sgv2-<machine>.conf`:
- disables WirePlumber's own nodes for the speaker and sense PCMs, so no other rule can make a second path to the amps;
- puts the machine's voicing in front of the protect stage, without hiding it, so the session helper can see the stage's ports.
