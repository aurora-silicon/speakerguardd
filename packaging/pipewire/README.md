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
`VolumeDb` for the graph's dB consumers with the artifact's volume law: the
macOS HAL law, `min + (max - min) * sqrt(scalar)` over the speaker volume
range the Mac's own tuning bundle states (`Config/AID<n>.plist`), so slider
x% gives the level macOS gives at x%. A bundle without that range keeps
`60*log10(scalar)`. `VolumeTaper`
applies the bundle's own curve and publishes its gain to Comp automation
through the saved relay. Offline hosts can supply an explicit nonpositive
`volume_db` control to match `voicing-render --mvsc S --mvol DB` exactly
(`--mvol auto` applies the artifact's law).
WirePlumber's record for this sink depends on which key it picks, and a
first start has none, so the sink would come up at 100 %.
`speakerguardd-session` stores the sink's channel volumes and mute in
`$XDG_STATE_HOME/speakerguardd/voicing-<machine>.json`. A new sink that
WirePlumber restored keeps that volume; one still at the default after a
second gets the stored volume, or 30 % on the cubic slider (channel volume
0.027) with nothing stored. Note: the `voicing:volume` value a Props dump
lists is a snapshot PipeWire does not refresh on every volume change; the
plugin's control itself follows each change (filter-graph's info log shows
each write).
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

## Built-in microphone

`sgv2-mic-<machine>.conf` (and `../wireplumber/61-sgv2-mic-<machine>.conf`)
are generated by `gen.py` for every fixture whose device tree has a
microphone array (the fixture's `mic`: the AOP audio node that lists the
array's stream; `wiring.read_mic_dt`, `wiring.describe_mic`). Each holds:
- the array's capture PCM as an ALSA source of sgv2's own
  (`hw:CARD=<card>,DEV=<n>`, "Microphone Array (raw channels)", priority
  100): listed for applications that want the channels, never the default
  while another source exists;
- a capture filter-chain around the `#mic-<n>ch` plugin
  (`crates/sgv2-lv2/src/mic.rs`) that publishes the processed channel as the
  source "Built-in Microphone" (priority 2005, the priority asahi-audio gives
  its own microphone filter, so its headset rules still sit above it).

They are never installed statically. `speakerguardd-provision run` writes
them into `/run/speakerguardd-graph/mic/` from the live device tree and
card, after the speaker steps and apart from them, only when all of this
holds, and removes them (and the blob) otherwise:
- `aurora-speaker-tuning mic-compile` compiled this Mac's own bundle
  **without** `--allow-partial` into `/var/lib/speakerguardd/mic.bin`
  (bound in `mic.bind` to the board, the unit, the bundle's digest, the
  compiler and the blob version; any change compiles again, so a blob of
  an older version is never kept);
- the blob's header names this board, the array's rate and microphone
  count, the version the plugin takes, and no partial flag;
- the packaged plugin, loaded as a host loads it, runs the blob with
  health 0 (`speakerguardd-selfcheck mic`).

The blob is then published as `/run/speakerguardd/mic.bin`, where the plugin
reads it (else `$SGV2_MIC_PARAMS`). The seat user's
`speakerguardd-graph.service` links both files or neither.

**With asahi-audio.** asahi-audio attaches its microphone filter to
WirePlumber's own node for the array (`node.software-dsp.rules` matching
`api.alsa.path = "hw:<card>,<n>"`, `hide-parent`). The WirePlumber file
disables that node, so that filter is not created while sgv2's graph is
linked, and sgv2's own ALSA node (`hw:CARD=...`) matches none of its rules.
Without sgv2's files (no array, a refused or partial design, a blob the
plugin does not run, another user's session, a boot without provisioning)
nothing of this is linked and asahi-audio's microphone filter is as it was.
The states:

| state | "Built-in Microphone" | raw array |
|---|---|---|
| blob valid, files linked | sgv2's processed source (default input) | sgv2's ALSA source, priority 100 |
| mic refused / absent / not the seat user | asahi-audio's filter, as before | hidden behind it, as before |
| files linked but the filter-chain or plugin fails to load | none (the module is `nofail`) | sgv2's ALSA source becomes the default input |

`speakerguardd-selfcheck mic` (also reported by `verify`, beside the
speakers' result and never part of it) gives the verdict: `pass`, `absent`
or `refused` with the reasons. `/var/lib/speakerguardd/mic.json` is
provisioning's record.

`examples/sgv2-mic-capture.conf.example` remains a hand-editable example of
the same filter-chain for development. The
artifact is versioned and the plugin refuses another version (a
version-1 blob from the first mic release gives silence and health 1 with
the current plugin), which is why provisioning binds the version and
compiles again when it changes.
