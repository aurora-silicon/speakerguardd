# Keep recent recording evidence across restarts

The repository service does not delete recordings. A deployed local override may have an `ExecStartPre` command that prunes older `sg.rec.*` files on every restart. Replace only that destructive line in its defining override with the installed helper, after reviewing the effective unit:

```ini
ExecStartPre=-/usr/lib/speakerguardd/preserve-recordings /var/tmp/sg.rec
```

Use the actual absolute recording path. The leading `-` is required: an archive failure must never stop the governing service. Preserve all other ownership, session, protection and service checks. The daemon opens its recording target with `create_new` and refuses existing active or sidecar files, warning and continuing protection without recording. Thus a failed helper cannot cause the following daemon to truncate evidence or pair a stale `.previous` with a new run. The kernel safe level applies until the daemon owns and renews the existing lock.

The helper moves only the stopped owner's `FILE`, `FILE.previous` and `FILE.next` into a private archive. It requires Python 3 and refuses symlinks, non-regular sources and stale locks. A partial move leaves an `.incomplete` marker and is never pruned automatically. The archive budget is **4 GiB by default**, configurable with `SG_RECORD_ARCHIVE_MAX_BYTES` in the unit environment. Incomplete archive bytes count against the budget. Before moving current evidence, the helper removes the oldest complete archives until there is room. It marks an archive before deletion, so a failed deletion leaves any remaining evidence marked incomplete. If the incoming evidence cannot fit alongside incomplete archives, it remains in place and recording is disabled on restart. Removal and every preservation failure are logged loudly. Unrelated files are untouched. This policy intentionally discards the oldest complete archived evidence; copy it elsewhere before the budget is reached if it must be retained longer.

The helper runs only after the recorder has stopped. An operator must review and authorize any effective-unit migration, reload or restart. This repository change does not modify a running unit.
