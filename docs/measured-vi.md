# Measured V/I mode

`measured_vi = true` selects the 2.0.1 speakersafetyd model and ramp, rather
than a predictive governor with a measured-power guard. It requires
`governor = asahi`, `strict_admission = true`, S16 V/I on every output,
no guard/shadow, no `z_min` override, and zero power budgets. Config/CLI
combinations that could re-enable those paths are refused. `Guard::start`
refuses this mode; the live entry point constructs only `ViGuard` and `Model`.
No legacy or unified model, budget history, impedance fallback, predictive
onset reservation or warm-state restore is constructed. Every process start
assumes both thermal nodes at `t_limit + t_headroom`.

The active model uses `tr_*`, `tau_*`, `t_limit`, `t_headroom`, `z_nominal`,
V/I channels/scales, ambient, window, hysteresis and the actual configured amp
gain. `z_nominal` enters only the 2.0.1 ramp's minimum-gain formula; it is not
an impedance validity gate. `r_ff`, `release_s`, and `vi_*` duplicates remain
source metadata. No feed-forward thermal transfer is used. On J613, the ramp
ends at 140 C and the absolute fault boundary is 150 C. The discrete sample
math deliberately retains the original f32 coefficients and power conversion.
The source is MIT speakersafetyd 2.0.1
`63a92658a4daec170c6472e38162374206e593fb`, `src/types.rs`; the retained MIT
notice applies. Tests pin the independent scalar oracle to those equations.
The active tweeter values are Asahi's 200 K/W and 1.85 s.

Every speaker validates on a copy before any state or output is committed.
A complete, stable-rate period is mandatory; two consecutive complete periods
are required before the first unlock or a re-admission. Invalid/short capture,
ESTRPIPE, lost position, rate changes and overdue feedback revoke the lease,
close capture, freeze unmeasured cooling, and raise BOTH nodes to the hard
limit (or keep a hotter known state). There is no speculative V-squared heat
charge. Measured samples cool this initial hot state; this is deliberately
more conservative than 2.0.1's restart initialization and idle cooling.

## Paired kernel protocol

Measured mode requires the additive mixer control `Speaker Protection Epoch`.
The daemon locks it and `Speaker Volume Unlock` on the same ALSA control file.
It reads a positive epoch before opening fresh sense capture, then writes that
same epoch only after complete periods have validated. Under
`volume_lock_mutex`, the kernel checks the epoch and grants the existing
250 ms lease in the same critical section as its PM notifier. Suspend prepare
increments the epoch, expires the lease and caps speakers; PM post retains the
cap. Therefore even a sub-millisecond sleep invalidates the token. If suspend
lands inside the final heartbeat, either the grant precedes PM's cap, or the
write is rejected with ESTALE. An old token never grants after resume. Epoch
wrap exhausts strict admission instead of reusing tokens. Old magic heartbeats
remain supported for existing daemons, including rollback speakersafetyd.

This is a new paired ABI, not functionality supplied by the three older PM/
lock-failure backports alone. The J613 package must be tried only inside the
matching kit boot. Kernel module compile and host tests are not physical audio
or suspend qualification. The profile remains `qualified = false`; an explicit
owner-attended trial is separate from qualification or fleet activation.

Offline checks: `cargo test --locked vi_only`,
`cargo test --locked profile_tests`, `python3 tests/j613_sources.py`.
