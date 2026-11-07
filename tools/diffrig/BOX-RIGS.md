# Split voicing box rig

`voicing-box-render --strip FILE --only NAME --ch N IN.f32 OUT.f32` compiles
one saved box through its production module and runs it independently. The
strip and channel width must be from the same graph. It supports cfrk, vben,
axtc, vrq2, ldnm, pchr and volt in addition to the existing mstk/tmst paths.
It preserves compiler refusals. `--set ID=VALUE` changes saved parameters
before compilation. A separate strip avoids unrelated whole-graph failures.

`boxrig.py generate --root PRIVATE_DIR` generates deterministic probes from
`inventory.json` (rows: id, type, box, ch, strip, v2_rc). Inventory and strips
are private inputs; the rig embeds no machine tuning. `boxrig_host.py` in
`macos/` is a thin stdlib adapter for installed `box_host`; hold a host-heavy
slot and run nice 15. Signal generation, v2 rendering and comparisons run on
the buildbox. The host adapter transfers each completed state before deleting
its bulky local audio. All resource paths and destinations are arguments.

`boxrig.py worker --root PRIVATE_DIR` consumes complete host state directories;
`boxrig.py run --root PRIVATE_DIR --box TYPE --tag rerun` rerenders a box
against its retained AU references. The caller provides an approved buildbox
slot. Raw data and parameter metadata never belong in public git. Save source,
binary, preset and input hashes with the private inventory.

A case is the worst absolute RMS difference across its channels, steady
second half and 10 ms transient windows. The 0.05 dB gate is two-sided.
Transient windows with both outputs below -80 dBFS are ignored; the steady
numerical-silence floor is -240 dBFS. Refusals and missing output are separate
from measured failures. Norm meters and applies gain inside each render call; its intrinsic
look-ahead is retained. Neither renderer flushes or crops a transport call.
There is no latency compensation, fitted alignment or fitted gain.
Amplitude parity alone does not establish phase or sample identity.

Controls, run on buildbox:

```sh
python3 tools/diffrig/test_boxrig_renderer.py --renderer PATH/voicing-box-render --work PRIVATE_SCRATCH
python3 tools/diffrig/test_boxrig_metric.py
```
