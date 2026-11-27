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

## The static class

One kind of companion is released: one whose estimator is never run, beside a
complete LSM2 model. The single LSM2 unit protects on that static model, the
unit's own mode when it holds no valid estimate: open-loop temperature from
the type's thermal model and ambient, no closed-loop temperature, no measured
power. The companion's records are carried opaque and never interpreted. The
compiler reports `lsid-static-release`. Two layouts are in the class:

- A companion that is not the six-lane schema has no twelve-lane topology, so
  the unit never builds an estimator. Its model is the complete 1,021-record
  schema (a short preset completed by the owner-local fill counts). Today this
  is the four-lane LSM2 tuning of the MacBook Air 13-inch M4 and M5 (J713
  AID37, J813 AID42).
- A six-lane companion is released when its model is the complete 1,326-record
  extended schema and none of the 305 appended fields asks for a mechanism the
  static model leaves out (`lsm2::companion::six_lane_static_refusals`). The
  compiler drops its twelve-lane topology, so no estimator is built and no
  sense source can be bound to it. Today this is the six-lane tuning of the
  14-inch and 16-inch M5 MacBook Pros (K114 AID49, K116 AID50). A six-lane
  companion with any refused field keeps its gates and stays muted; the
  compiler names the fields (`lsid-six-lane-unqualified`).

The released model must be representable exactly as without a companion; the
compiler refuses it otherwise, and the Mac stays at the kernel floor. Only the
two mechanisms the companion owns are left to it, unrun: system identification
and closed-loop temperature. Three differences follow from what is run:

- The mid/high rail, battery and soft-knee amplifier models act only on the
  power axis. With `ControlPower` off they change no gain (the engine's own
  rule), so they are accepted then and refused otherwise.
- A lane may use tuning bank 1 or 2. Without the estimator the bank only
  selects the pilot block whose amplitude the peak axis reserves.
- The current model has no inductive branch. Where leaving it out could lower
  the modeled power, the current model is scaled until it cannot; see below.

### The six-lane appended fields

These are run, or change nothing with the rest of the model as admitted:

- `TemperatureResponseRateFactor` (controller field 27) is run. It is an
  additive floor on the temperature candidate's allocation weight, which
  moves the split of the allowed rise from the present voice-coil/magnet
  proportions toward the steady-state thermal-resistance proportions:
  `w = clamp(q^2 + factor, 0, 1)`, where `q^2` alone is the base schema's
  weight. It applies with open-loop temperature too.
- The displacement and velocity release times belong to adaptive release and
  change nothing while it is off.
- The per-system power limits change no gain with no power groups and the
  power axis off.
- Slow and temperature gains link across all six lanes, never per controller
  system; thermal states stay per lane.
- The system-ID FFT size, the Rdc tolerance, the sensitivity pairing and the
  parameters of a disabled mechanism belong to the unrun estimator or to that
  mechanism.

Refused when set: sensitivity control, MIMO compensation, the high-impedance
watermark, adaptive release, power groups, a nonzero reserved pilot-tone
channel selector, other than one speaker per thermal type, and the per-system
power limits with the power axis controlled.

### The inductive branch

The admittance model of these tunings enables an inductive (LR2) branch,
`Z = jwLe + (R2 jwL2)/(R2 + jwL2)` in series with `Rdc` and the first
resonator, and the native unit's thermal power uses it. The current model has
no such branch. The power path integrates `u * i`, so for any signal the
modeled power stays at or above the full model's if `k Re Y` without the
branch is never below `Re Y` with it. The compiler finds the smallest such
`k >= 1` from DC to Nyquist on the channel's own resolved model (refined
around the grid's maximum, rounded up to binary32) and the unit multiplies
the current model by it (`ChannelParams::admittance_bound`), so every thermal
and power estimate errs high. Where leaving the branch out only lowers `Re Y`,
`k` is exactly 1 and the current model runs as it stands, bit for bit (the
J713 and J813 tunings). A branch that is not passive (a negative or non-finite
field) is refused. On the K114 and K116 tunings `k` is a few percent above 1,
from a band just above the woofers' and tweeters' resonance where the branch's
inductance cancels part of the motional reactance.

The specification does not name the branch's three fields. We read them as R2,
Le and L2 because that order fits our own V/I sense measurements of a J713's
four speakers (swept sine, current and voltage aligned per lane): on the
woofers its impedance error is about half that of the order with the
inductances swapped and a fifth of the model with no branch. On those
measurements the static model, which leaves the branch out, predicts 3 to 10 %
more electrical power over the swept band than the speakers drew.

### Sense alignment

The released class does not use sense for protection. The self-check still
proves the V/I wiring silently, and the measured lane alignment is recorded
for when adaptation is qualified: on our J713 the left woofer's current
arrives one frame after its voltage on the Linux sense PCM, while the
tuning's own source description has no delay on any of the eight lanes.

## Gated companions

Every other companion carries mandatory audible-release gates. The
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
board enablement or numerical parity. There is no switch to remove these gates,
and provisioning refuses a blob the compiler reported as `lsm2-release-gated`,
so such a board never admits a supervisor.
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
