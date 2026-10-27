# Boot-state deployment note — review build only

This branch is based directly on the deployed `2f7d16db0313` pr8 source.
It keeps that commit's V/I low-line and power-factor fault thresholds. The
RPM does not enable speakerguardd or select a speaker owner. Fedora 45 is
for the J616s; Fedora 44 is for the M1 Pro (J316) and M2 (J413), subject
to each device's existing qualification and owner policy.

## Safety and timing

The daemon atomically saves the model every 9 s, on suspend, and after a
clean stop to `/var/lib/speakerguardd/state`. It records UTC realtime,
boot ID, machine ID, configuration fingerprint and an RTC reading when
available. The last successful snapshot may age at most 10 s while the
daemon renews the volume lease; at that age it relinquishes the lock.
The kernel's 250 ms lease, 500 ms queue and 250 ms margin fit within the
11 s full-gain unclean-playback charge. Same-boot restore credits only
`min(positive realtime, CLOCK_BOOTTIME)` and advances the coupled thermal
plant with exact exponentials. A backwards wall clock gets zero credit.

The clean-stop `last-shutdown` marker is diagnostic. Neither it nor a
paired realtime/RTC reading proves that macOS did not play in between.
Cross-boot checkpoints therefore receive no cooling credit. A clean
checkpoint matching the last Linux shutdown marker can resume if the
positive wall-clock gap is under 90 s; it receives zero cooling credit.
That operational fast-restart signal cannot exclude an intervening OS.
All other cross-boot starts initialise each V/I model from the hotter of
its feed-forward and measured-power
plants after a 30 s silent transition from 150 °C at both nodes, plus 1 °C.
On the J616s woofer this is 139.4 °C coil and 136.9 °C magnet. Missing or corrupt
state uses the same envelope.

The governor bounds an unknown full-scale square wave over the next second
against every plant's 140 °C working ceiling before the volume lock can
be taken. The V/I profiles allow up to 60 dB governed attenuation, so
the card can unlock at reduced level and release gain as the model cools.
A hard-limit or invalid-model error still hands the card to the kernel.
These results depend on the stated cross-OS envelope; M1 Pro and M2
physical temperatures and device audio timing remain hardware gates.

## Host tests and artifacts

The Linux host suite exercises cold handover, warm same-boot checkpoint
restore, backwards clock handling and 120 s full-scale pink and max-power
sweep bounds on J316, J413 and J616. The actual waveforms remain below a
full-scale square wave at every sample; the governing model charges that
square-wave upper bound. The fake-card handover takes 0.171 s on all three
profiles. Under the continuous full-scale square-wave charge, attenuation
at 0, 1, 5, 30, 60 and 120 s is:

| Profile | 0 s | 1 s | 5 s | 30 s | 60 s | 120 s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| J316 / M1 Pro | 23.3 | 24.7 | 24.2 | 19.9 | 17.9 | 16.8 dB |
| J413 / M2 | 18.9 | 21.1 | 22.1 | 21.4 | 21.2 | 21.2 dB |
| J616s | 23.3 | 24.7 | 24.2 | 19.9 | 17.9 | 16.8 dB |

The first seconds may need more attenuation as continuous maximum power
warms the coil; under silence the model cools and releases gain. Record
the final CI run, fc44/fc45 RPM identities, source stamp and SHA-256
checksums here before a package trial.

The earlier fc45 artifact
`speakerguardd-0.1.10-13.pr8.6fb46182694f.fc45.aarch64.rpm` with SHA-256
`45b5791547a57c92c37805e679939fd7feedc207c140ce205fb2387646ca7838`
is from the superseded base and is not interchangeable with this build.

## Orchestrator deployment and rollback gates

1. Preserve the installed RPM, source stamp, checkpoint, local drop-ins,
   interim helpers, recorder files and journals. Record the effective unit,
   owner policy, profile, governor and all drop-ins on each device. Keep
   private device records out of the public PR and source.
2. Confirm the package commit and both checksums, then compare the
   safety-sensitive source, effective unit and profile with the installed
   `0.1.10-10.pr8.2f7d16db0313` package. Check that no local `-m 7`
   override defeats the packaged 60 dB handover range.
3. This RPM packages PR #23's recording preservation helper at
   `/usr/libexec/speakerguardd/preserve-recordings`. Replace only a local
   destructive `ExecStartPre` pruning line with that helper after retaining
   the old override and recordings. Keep every session, source, resource
   and owner gate; see `docs/recording-retention.md`. Remove the interim
   state save/restore drop-in and scripts only after checking the packaged
   persistence path. Do not stack both restore mechanisms.
4. The orchestrator separately verifies J616s cold and warm boots, owner,
   lock handover timing, interlock and rollback, then applicable gates on
   qualified M1 Pro and M2 devices. This task does no hardware playback,
   reboot, installation or override migration.

Rollback restores the prior RPM and effective unit/drop-ins under the
existing owner policy. Retain checkpoints, recordings and journals as
evidence.
