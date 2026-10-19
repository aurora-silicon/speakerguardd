# Three-reboot startup check (Neo agent)

This is a proposed hardware procedure, not authorization to run it. The
operator must approve the exact device, kernel, package installation and
each reboot through the controller's session, source, resource, ownership
and policy gates. No hardware action is performed by the collector. Do not
use this procedure to bypass those gates or to SSH directly to a device.

Before the run, stage the reviewed patched binary and unit as a package.
Preserve the previous package, unit, overrides and their hashes in the
controller's private evidence area for rollback. Preserve PR #2's protection
changes and `Restart=on-failure`. Do not alter kernel lock controls, delete
the model state, disable the daemon, or use an `ExecStartPre` workaround.
If an overriding `/etc/systemd/system/speakerguardd.service` is present,
review it: replacing only the vendor unit may not change the effective unit.
The patched binary defaults to the same 30-second wait with older units.

Have the controller-approved Neo agent provision a private file containing
the expected `/etc/machine-id` from its trusted device inventory. Do not
derive that expected identity from the device during the test. Do not put
its contents in source or a public report. Obtain the exact expected kernel
release independently from the approved 7.2.10 / unified build manifest.

Perform three controller-approved reboots with that same kernel and
package. After **each** reboot, wait at least 60 seconds, keep the machine
idle without audio playback, and run the collector through the authorized
agent. Use a new, private output directory for each three-boot run:

```sh
# Replace these placeholders with the controller-approved paths/release.
# Run once after each reboot, setting N to 1, then 2, then 3.
python3 tests/neo-three-boots.py collect --trial N \
  --expected-machine-id-file /protected/controller/neo-machine-id \
  --expected-kernel EXACT_APPROVED_KERNEL_RELEASE \
  --output /protected/controller/speakerguardd-boot-run

# After the third reboot:
python3 tests/neo-three-boots.py check \
  --output /protected/controller/speakerguardd-boot-run
```

Run with sufficient privilege to read the system service journal and
`/run/speakerguardd/state`. The collector checks the device identity and
J700 compatible, enforces three distinct boot IDs on one device and one
kernel, refuses to overwrite evidence, and observes each service for 30
seconds. Collection failures are failures, not passes. Reusing a boot or
an existing trial file is rejected. Keep any failed run intact and use a
new directory if the operator authorizes another run.

Pass criteria for every boot: AppleJ700 exists; the service remains active
in its first invocation, `NRestarts=0`, exactly one initial volume-lock
acquisition is logged, no missing-card error/start failure/restart appears
in the boot's service journal, and two model snapshots are fresh and armed.
A recovered initial failure still fails the check. Package ownership of
the binary, unit and rule is recorded with `rpm -qf` (Fedora) or `pacman -Qo`
(Arch). An unowned file is a collection failure and needs packaging review.

This quiet check establishes startup behavior and the daemon's reported
lock state. It does not electrically prove the kernel interlock, inject a
driver delay, or replace PR #2's separately authorized fail-safe drills.
Do not start playback to complete this check. Do not reset failure counters
or manually restart the service during a trial.

On failure, retain all evidence and stop the qualification. If the operator
approves rollback, reinstall the preserved known-good package and effective
unit/overrides through the controller; reload systemd as part of that
approved package action and qualify the restored boot separately. Do not
work around a failure by disabling restart or releasing the speaker lock.
Only publish a redacted result (kernel/package version, three pass/fail
outcomes and startup timings). Journals, boot IDs, device fingerprints and
local package paths stay in the controller's private evidence area.
