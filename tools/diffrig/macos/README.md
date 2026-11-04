# macOS reference host

Ownership: lane C.

`aurender` (Swift, built on the user's own Mac) instantiates the system's own protection unit (`aufx` with `spp3`, `lsm2` or `atsp`, by `appl`) through the public AudioComponent API, applies that Mac's own preset, renders the rig's signals in manual rendering mode (with a telemetry bus for V/I units), and records the unit's published state every slice in `../schema.json`.

Traces made with Apple tuning are Apple-derived data. They never enter this repository; only synthetic-parameter results are committed.

TODO(lane C): the host, the case list, and the signal generator.
