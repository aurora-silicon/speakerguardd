# Keep recording evidence across restarts

The repository service does not delete recordings. A local recording override
may contain an `ExecStartPre` command that renames the active recording and
then pipes `ls -1t FILE.*` through `tail -n +3 | xargs -r rm -f`. **That command
permanently deletes all but two matching files at every restart.** Its broad
glob can also select rolling segments and unrelated evidence. Removal is not
reported by the daemon. Do not use that retention command to preserve evidence.

The installed `/usr/libexec/speakerguardd/preserve-recordings` helper instead moves
the stopped owner's `FILE`, `FILE.previous` and `FILE.next`, including empty
files, into a new private `FILE.saved.XXXXXXXXXX` directory. It logs every move,
never replaces an archive, never prunes, and refuses symlinks, non-regular
sources, or an existing preservation lock. A partial failure leaves the saved
files and remaining inputs for inspection. Files outside these three explicit
names are untouched. A stale lock must be investigated by the operator, not
silently removed. The helper requires standard Linux coreutils and `/bin/sh`.

For an existing local recording override, **edit only the destructive
`ExecStartPre` line in its defining file**, replacing that entire line with:

```ini
ExecStartPre=/usr/libexec/speakerguardd/preserve-recordings /var/tmp/sg.rec
```

Use the actual absolute recording path. Retain every other session, ownership,
identity, resource and admission check, the governing command, the independent
guard, and any fallback service policy. Do not append this line after the old
pruner, reset the whole `ExecStartPre` list, or prefix this command with `-`:
a preservation failure must prevent the following daemon from truncating the
recording. The existing kernel safe-level behavior applies until protection
owns and renews the volume lock. An operator must review the effective unit
and authorize any reload/restart; this documentation does not change a running
system. Copying this repository's service over a local unit is not migration.

The helper must run only while that recording's writer is stopped. Its lock
serializes helper invocations, not arbitrary external writers. It does not
flush a live daemon, monitor service ownership or archive an ongoing stream.
For manual daemon invocations, preserve existing files before `--record FILE`,
which otherwise retains its existing create/truncate behavior.

Saved directories are **outside** `--record-max-mb` and have no automatic
retention limit. Monitor free space and explicitly archive or remove selected
saved evidence under the operator's retention policy. Disk exhaustion can stop
recording with a warning while protection continues. A rolling recorder may
intentionally replace its `.previous` segment during one run; archival here
preserves what remains at restart, not segments already evicted from the ring.
Earlier `FILE.*` archives are kept as they are, without a glob-based cleanup.
