# PipeWire templates

Generated per machine by `packaging/session/gen.py`. Do not edit `sgv2-<machine>.conf` by hand. Every value is derived (`packaging/session/wiring.py`) from the machine's device tree and its tuning's graph, recorded in `packaging/session/fixtures/`: the channel map and V/I order (`derive`), and the card, model, amp and sense counts, plugin variant, speaker format and buffering (`describe`, the last two per transport driver). Nothing is written by hand per board. On any Mac, `speakerguardd-selfcheck generate` writes these files from its record, derived from the live device tree and checked against ALSA. The protect stage carries the map's hash as `sgv2.wiring`, which the supervisor requires to equal the self-check's (`docs/NEW-PLATFORM.md`).

Each file holds:
- the speaker amps' ALSA sink (`hw:CARD=<card>,DEV=1`);
- the sense source (`DEV=2`), in one `node.group`, at a 512-frame quantum;
- the protect filter-chain around one `sgv2` LV2 instance. Its input is named `alsa_output.platform-sound.RawSpeakers`, the name the machine's voicing graph targets.

See `gen.py` for the whole graph and `../session/speakerguardd-session` for the links and the report.

At runtime, new boards automatically compile their own generated voicing.
`packaging/session/gate.json`'s `voicing_pre` map holds explicit deployment
overrides: literal `true` selects the generated stereo `voicing-pre` chain,
and `false` selects the interim software-dsp convolver. Offline fixture
artifacts retain their recorded mode. Qualification still requires a
refusal-free compile, build-wide protection parity, own-Mac extraction
provenance and a current silent self-check; an override cannot bypass it.

Provisioning passes `--voicing-pre` to `aurora-speaker-tuning compile` in this
mode. It compiles the **same spec as `voicing-render`**, with every box's
parameter words, Tone automation, graph links and box relays. The ordinary
board/unit/provenance checks still apply. Unimplemented boxes and unwired
graph inputs refuse compilation; partial-law findings remain warnings for
the lane's separate qualification decision. The appended versioned artifact
records the LowFlow build flavor (native/conservative), has a CRC over its
identity and contents, and is bound to the complete unit
blob, including board, rate, geometry and operating point. No bundle values
are installed in the package or committed to source.

The existing supervisor publishes one `params.bin` atomically. After the
self-check passes, activation also publishes those exact checked bytes
before restarting PipeWire; this avoids the service ordering race where
plugins could instantiate before the supervisor starts. Both plugins
load that file during instantiate. The voicing plugin additionally checks
its live device-tree board and host rate/geometry. Missing, corrupt or
mismatched voicing produces silence. The protect plugin checks the same
artifact and runs an identity pre-stage in this mode, so crossover and
dynamics are applied once. The generated lane order uses `wiring.py`'s map
on both sides of `RawSpeakers`; the sense inputs and protection ownership
path are unchanged.

The visible stereo sink handles user volume through `capture.volumes` and
an LV2 control port, as documented in [PipeWire's filter-chain volume
controls](https://docs.pipewire.org/page_module_filter_chain.html). Cubic
mapping sends the slider scalar to `VolumeScalar`; the plugin derives
`VolumeDb = 60*log10(scalar)` for the graph's dB consumers. `VolumeTaper`
applies the bundle's own curve and publishes its gain to Comp automation
through the saved relay. Offline hosts can supply an explicit nonpositive
`volume_db` control to match `voicing-render --mvsc S --mvol DB` exactly.
Zero/non-finite scalar mutes residual history as well as the input. Balance
controls are not exposed; the chain uses its centred defaults.

After an approved per-board gate change, rerun provisioning and the
self-check/activation sequence, then restart the graph's PipeWire session so
both plugins instantiate from the newly published blob. Mode changes
invalidate the provisioning cache. A voicing-only parameter change also
invalidates the graph digest and the protect engine's advertised identity;
the persisted thermal state remains bound to the unchanged unit payload. Generation refuses a gate/compiled-mode
mismatch. For rollback, set that board's flag to `false` and repeat the same
sequence; provisioning recompiles the interim pre-stage and restores the
software-dsp convolver. No target was changed by the implementation tests.

The LV2 binary remains `libsgv2_lv2.so` in `sgv2.lv2`; the generated Turtle
files advertise additional `#voicing-{1,2,4,6}ch` descriptors. Install the
binary and both Turtle files together. The DSP core and artifact decoder
remain `no_std`; the existing LV2 host shim uses `std` for non-RT loading.
All chain storage is allocated before `run()`, which chunks host callbacks
into at most 512 frames without allocation, locks, file I/O or logging.
