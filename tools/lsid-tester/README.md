# LSID tester kit (speakerguardd v2), 2026-11-13

This kit is for owners of Macs whose speaker tuning has an LSID stage. LSID is
an estimator inside the lsm2 speaker protection. You'll find it on the 13-inch
MacBook Air with M4 (J713, AID37) or M5 (J813, AID42), and the newest 14-inch
and 16-inch MacBook Pro (K114, AID49 and K116, AID50). None of the Aurora lab
Macs has it, so speakerguardd v2 keeps those speakers muted and we can't
qualify LSID ourselves. One run on your Mac tells us what we need.

**It is silent.** Nothing plays through the speakers at any step.
- Under Linux, the only streams are the self-check's, and every sample in them
  is zero. v2 keeps LSID speakers muted the whole time anyway.
- Under macOS, the script sets the built-in speakers to volume 0 and mute,
  reads both back, and only then plays digital silence (all-zero samples) for
  30 seconds. It re-checks volume and mute every 200 ms and stops at once if
  either changes or headphones get plugged in. Afterwards it puts your volume
  and mute back as they were (add `--leave-muted` to keep them at 0/muted).

**What you send back:** the two text logs (one per OS), nothing else. They
hold pass/fail lines, numbers, hashes, error text and the exact OS build. No
audio, recordings, tuning files or personal data. Your tuning bundle stays on
your Mac. Each script ends by running `redact-check.sh` on its log, which
looks for your user name, host name, serial number and similar. Read the logs
before sending them; edit out anything you don't want to share.

It takes about 10 minutes (plus a one-time Rust build). **Saying no is fine**,
and so is running only one half.

## What it uses

- Linux: speakerguardd v2 at commit `4d55e1d5` or newer (built from source by
  `linux/build-v2.sh`), and, if they're installed, the packaged
  `aurora-speaker-upgrade` and `speakerguardd-selfcheck`.
- macOS: our own reader, `macos/lsidprobe.c` (about 600 lines of C you can
  read first). It uses only public, documented interfaces: CoreAudio device
  properties and one output callback that writes zeros; AudioToolbox
  component lookup and parameter lists; IOKit registry reads (what `ioreg`
  shows); and `log show`. It makes no private calls, doesn't load or inspect
  any Apple binary, and records nothing.

## Linux (about 6 minutes)

Boot Linux on the Mac. From the kit directory:

```sh
sh linux/build-v2.sh                    # as your user: clones speakerguardd v2, builds aurora-speaker-tuning (needs git + cargo)
sudo bash linux/run-linux.sh            # writes ./lsid-tester-linux-<time>.log
```

Options for `run-linux.sh`:
- `--bundle PATH` uses your tuning bundle (the directory or `.tar.gz` from
  `collect-macos.sh`). Without it the script uses the bundle
  `aurora-speaker-upgrade` stored, then the system firmware.
- `--collection TARBALL` passes the same tarball to
  `aurora-speaker-upgrade --check`.

`build-v2.sh --src DIR` uses a checkout you already have, and so does running
the kit from `tools/lsid-tester` inside a speakerguardd checkout; nothing in
it is changed.

What `run-linux.sh` runs, in order (the commands it runs, `CLI` being the
`aurora-speaker-tuning` you built):

| Step | Command | Changes anything? |
|---|---|---|
| identify | `CLI identify --json [BUNDLE]` (the unit id is not logged) | no |
| validate | `CLI validate --json [BUNDLE]` | no |
| compile | `CLI compile [BUNDLE] -o <private temp file>` for this Mac (board and unit from the device tree); the log gets the size, header line, diagnostic codes and sha256 | no (temp file deleted) |
| LSID | whether the compile reported `lsid-disabled` and `lsm2-release-gated` (LSID present, compiled muted) | no |
| voicing-check | `CLI voicing-check [--attestations parity.json] [BUNDLE]`: offline render checks, no audio device | no |
| wiring | `wiring.py describe` from the device tree and the tuning graph | no |
| upgrade check | `aurora-speaker-upgrade --check --json F` (dry run: collect, compile, self-check in /run; nothing stored or switched), if installed | no |
| self-check | `speakerguardd-selfcheck verify --params <installed blob>`: zero samples only, if v2 is installed | no |
| runtime | `systemctl is-active`, v2's journal lines about LSID/gate/mute/grant, SELinux denials during the run | no |

The script needs root for the last three. Its scratch files live in a
private directory under `/var/tmp` that is deleted on exit.

Expected on an LSID Mac: compile PASS with `lsid-disabled` and
`lsm2-release-gated` (muted), and `RESULT lsid PASS ... compiled MUTED`.
`voicing-check` may say REFUSED for `macos-parity` when no attestation
matches your macOS build or this compiler. That's expected and not a failure.
`upgrade --check` says CANNOT-RUN when v2 already drives the speakers in this
boot. That's expected too.

## macOS (about 2 minutes)

Boot the Mac's own macOS and log in. From the kit directory, as your user (no sudo):

```sh
sh macos/run-macos.sh                   # writes ./lsid-tester-macos-<build>-<time>.log
```

It compiles `macos/lsidprobe.c` with the Command Line Tools
(`xcode-select --install` if you don't have them), or, with `--prebuilt`,
runs `macos/lsidprobe-arm64`, built from the same source (sha256 in
`macos/SHA256SUMS`; shipped with kit copies only, not in the repository). Then it:

1. reads the AID from the device tree (`acoustic-id`), the speaker ID, and
   whether the per-unit calibration property exists (its length only);
2. checks whether this macOS build registers standalone `lsid`, `lsm2`,
   `spp3` or `atsp` Audio Units (`AudioComponentFindNext`); for any that are
   registered, it logs the parameter count and a hash of the parameter-ID list;
3. snapshots the audio-related IORegistry state (scalars only; data blobs as
   lengths; serial/UUID/address-like keys skipped);
4. sets the built-in speakers to volume 0 and mute, reads both back, and plays
   30 s of digital silence while the watchdog re-checks them. At 2 s, halfway
   and near the end it snapshots the registry again and logs what changed;
5. hashes this Mac's tuning folder (`/System/Library/Audio/Tunings/AID<n>`),
   giving a file count and one tree hash. The files themselves are not copied;
6. reads the unified log of the audio daemons for the stream window and keeps
   only lines that mention the estimator, protection or thermal state;
7. restores volume and mute, then runs `redact-check.sh` on the log.

The stream is digital silence. macOS applies volume and mute above the
speaker DSP, so the estimator sees no excitation. It should therefore stay in
its seeded/acquiring state: no new model, no measured power. That no-excitation
behaviour is the first reference v2 needs. We don't ask for any audible
signal.

## Checking and sending

```sh
sh redact-check.sh lsid-tester-linux-*.log     # on Linux: sudo sh redact-check.sh ... (to read the serial)
sh redact-check.sh lsid-tester-macos-*.log
```

PASS means none of the checked kinds of data were found. A hit names the line
and the kind of data, never the value. Then send the logs back where you were
asked, or attach them to a speakerguardd GitHub issue.

## Files

- `linux/build-v2.sh`, `linux/run-linux.sh`: the Linux part.
- `macos/run-macos.sh`, `macos/lsidprobe.c`, `macos/lsidprobe-arm64`, `macos/SHA256SUMS`: the macOS part.
- `redact-check.sh`: the log check (POSIX sh, both OSes).
