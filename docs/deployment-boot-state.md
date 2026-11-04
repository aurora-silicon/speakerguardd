# RC persistence resolution

The candidate reconciles main `028702964884` (#17, #20 and #29) with
RC `5d8076e23479` (#27 and #28), using the RC evidence implementation.
Runtime state remains generation 3: #20 clean-stop snapshots remain
readable, generation 2 is refused, and missing engine models start latched.

The default `boot_resume = evidence` applies only when `one_model(cfg)` is
true: no V/I sense, the legacy governor and no guard (currently J700).
V/I and multiple-model profiles start latched, even when their configuration
requests evidence, and do not write evidence checkpoints. A past owner
need not have bounded every model's coil; this retains review fix C2.

Before unlocking, the daemon durably writes the boot id to
`/run/speakerguardd.unlocked`, outside the unit's RuntimeDirectory, and
invalidates the clean runtime state. With persistence configured a failed
runtime-state write keeps the card locked; removing the state is not a
successful invalidation. A missing state plus a same-boot or unreadable
marker starts latched. These are review fix C1. The marker is never removed
by the daemon. Keep it outside RuntimeDirectory when packaging.

A same-boot record, unknown boot identity, foreign protection flag, or set
or unreadable kernel protection override also refuses the evidence start.
Chassis measurements can only raise the starting bound. RTC errors never
supply cooling credit; the accepted clock and cross-OS residuals are
specified in `src/boot.rs`.

The independent opt-in `boot_resume = idle` retains #11's RTC, complete
boot-history and durable owner-invalidation contract. #24's assumed silent
cross-OS interval and marker-only ownership proof remain excluded. The
reviewed #28 evidence checkpoints supersede the earlier deferral of active
checkpoints for J700; they do not enable #24's restore policy on V/I models.

Replace pre-marker binaries across a reboot: mixed versions within one
boot cannot establish a protected history. One source/version covers all
profiles, while qualification refusal, governor/guard choices, kernel
lease, queue/floor policy and ownership rules remain effective. A package
update does not qualify a profile or authorize a protection handover.

The two ordinary-listening assertions still fail under the linear transfer
contract. Their budgets and the thermal limits remain required. See
[the necessary-dose bound](playback-energy-gate.md#j616-coverage-and-reference-envelope-feasibility).
Independent review of the reconciled candidate and successful release checks
are required before deployment. No machine deployment is performed here.
