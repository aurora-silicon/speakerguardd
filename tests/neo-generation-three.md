# Generation 3 compatibility fixture

`neo-generation-three.state` is synthetic state emitted by the unmodified
`state::encode` from public PR #20 at
`1de4cde4dcfcc09a1e7ba2430e8901d0fbd3ab53`. It contains no device data.

The generator parses the packaged J700 profile, creates two speakers at
50 C, sets their coil to `125.12345649 + index`, magnet to
`95.12345649 + index`, peak to `131.12345649 + index`, reduction to 6 dB,
and hard latch to false. It then encodes a released, unarmed clean stop
with both timestamps at 100 seconds. Histories and sensed voltage are zero.

The reader regression checks the old writer's rounded values, both speaker
identities and clean-stop marker, and keeps the rewritten generation at 3.
Generation 2 remains refused. Missing engine records cannot initialize an
engine; that separate fail-closed rule remains covered in `state.rs`.
