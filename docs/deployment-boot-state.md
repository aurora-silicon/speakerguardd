# RC persistence resolution

PR #24 was proposed on the older deployed #8 source. The RC retains the
stricter #11/#15/#20 persistence and startup rules. Missing, damaged, active
or old-generation runtime state starts latched with independent hot coil
and magnet bounds. A foreign protection owner also forces that path.

The RC does not credit the assumed 30-second silent cross-OS transition in
#24, does not accept its short clean-reboot marker as an ownership proof,
and does not replace the full-dose onset reserve with a one-second bound.
Its 60 dB profile changes and deployed fault-detector base do not replace
the later profile policy or recurrent-line repairs.

Clean same-boot state remains available. Cross-boot idle restore stays
opt-in under #11's independent RTC, complete boot-history and durable
owner-invalidation contract. All packaged profiles default to latched.
Periodic active durable checkpoint resume from #24 is deferred: the RC
retains no new unproven cooling or ownership assumption. No suspend hook
or device-specific packaging override is added by this merge.

This is a deliberate safety conflict resolution, not a claim that #24's
instant-boot or periodic-checkpoint behavior shipped. An independent
review must assess the final RC before any release or deployment.
