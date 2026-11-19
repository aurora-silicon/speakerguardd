# LSID companion bundles

LSID is a sensing companion inside the existing LSM2 protection unit. It is
never another protection unit, program effect, process, or service. Its measured
power is separate from LSM2's modeled power.

The reader validates the complete 603-record ID geometry, including all three
banks and opaque general/auxiliary fields. The six-channel LSM2 schema is the
1,326-record append-only extension of the 1,021-record schema. Records bind by
ID, retain Float32 bits, and reject duplicates, unknown IDs and non-finite
values. Legacy companion schemas remain opaque and release-gated; they are not
interpreted as the modern schema.

Six audio lanes carry three independent assignments: tuning bank
`[0,1,0,1,2,2]`, speaker family `[A,A,A,A,B,B]`, and controller system
`[1,1,1,1,2,2]`. The program graph supplies the permutation from duplicated
stereo lanes, `[0,3,1,4,2,5]`. These are logical system labels; raw `SystemNumber` selectors are preserved
without assuming their label/base encoding. The two flow dictionary records are retained as
side records, with explicit woofer/tweeter roles on every output lane. Their
unqualified response/gain law is not applied as if they were two audio channels.

The companion payload retains the complete saved states and flow dictionary,
property 64004, an explicit voicing qualification gate, and the twelve-lane source identity/order/alignment. Only current
lane 1 has a one-frame delay. This delay belongs to sensing and does not change
program latency. Adaptation must not infer source identity from channel count.
The current host I/O interface does not authenticate that identity.

The estimator exposes seeded, acquiring, tracking, decaying and reset states,
per-bank acquisition/validity state, bounded fast/slow resistance, LR2 and
three-resonance fits, measured power, calibration validity, and immutable
complete model generations. `SPEC-LSID-NUMERICS.md` defines the independent
clean-room law. The handoff stages a complete property-6630 semantic payload
before committing it at a controller boundary. Reset discards partial
acquisition and pending generations. Invalid, dead, stale or mismatched
telemetry cannot replace the last accepted model, produce valid measured
power, or raise the output.

The native private property serialization, legacy compensated models,
appended enabled-feature semantics and response/gain application from the flow
dictionary remain unqualified. Adaptation cannot be enabled, including with
all Cargo features: the normal host path also lacks authenticated source
identity. Numerical candidates can therefore be exercised only offline and
cannot remove the mandatory release gate. Stale private state decays toward
the static seed but is not published, so decay cannot relax protection. There
is no estimator persistence.

A companion bundle therefore carries mandatory audible-release gates. The
single governing LSM2 unit retains/runs its representable static/open-loop model
and advances companion state, but **mutes output** until those gates are closed.
An unrepresentable static model remains stored with the same mute gate.
A refused interim pre-stage becomes identity only under this mandatory mute
gate. If full voicing cannot be encoded, the compiler reports the unqualified
boxes/parameters and emits the muted bundle with identity pre-stage and **no
voicing artifact**, even when full voicing was requested. The ordinary artifact
admission checks are retained. Neither fallback can enable a board or
unprotected playback. Compile
diagnostics say `lsid-disabled` and `lsm2-release-gated`; runtime health reports
the fail-safe and unavailable parameters. This is compile/load support, not
board enablement or numerical parity. There is no switch to remove these gates.
Extracting ordinary `Params` from a companion blob is refused; hosts load the
whole bundle with `lsm2::Unit::from_blob` so they cannot accidentally lose the
gate. Existing bundles without a companion keep their original payload and
behavior.

An orphan `dbcontrol` is accepted only with zero ports, the expected control-unit
identity and saved-state geometry, and no audio, graph parameter, graph property,
box parameter, box property or property-strip connectivity. It is diagnosed and
eliminated from execution. Any connected instance is refused.

All repository tests generate synthetic records and graphs. Reference tuning
validation is private and offline; it must not add presets, compiled reference
blobs or tuning logs to the repository. No speaker playback is needed.
