# PipeWire and WirePlumber templates

Ownership: lane B. The graph generator emits, per machine, from that Mac's own tuning bundle:

- `voicing-pre` filter-chain: the user-facing virtual sink and the stages Apple's graph runs before the protection unit;
- `protect` filter-chain: an internal capture stream with N program ports and S sense ports (`node.autoconnect=false`, `stream.dont-remix=true`, `channelmix.disable=true`), the `sgv2` LV2 node with all channels in one instance, then the stages Apple's graph runs after the unit;
- a WirePlumber component that puts the sense source in the speaker sink's driver group, makes the exact links, and reports the link set and carrier volume to the supervisor.

TODO(lane B): templates and generator.
