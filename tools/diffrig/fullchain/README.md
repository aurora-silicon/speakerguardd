# spp3 full-chain safety scan

Raw program -> v2 voicing chain (`voicing-render`) -> v2 spp3 unit (`sgv2-render`, the
Mac's own bundle blob) against the Mac's own chain (`preprotect_host`) -> the Mac's own
SpeakerProtect (`rig_host`), both offline and open loop. Per driver, each axis must be at
or below macOS: peak cone velocity and displacement (Thiele-Small model per driver), peak
1 s drive power, peak coil temperature (the unit's own meter), stepped-sine steady and
transient margin. Outputs are aligned by cross-correlation before per-segment levels are
read.

- `progs.py OUT`: the programs (stepped sines, bursts, pink, music).
- `mac_refs.py PROGS OUT`: macOS side, on a Mac only (run it under the host-heavy slot).
- `a64build.sh NAME REF [MERGE...]`: static aarch64 builds of the three tools (buildbox).
- `scan.py`: one Mac's scan on a farm node; writes `report.md` (pass/fail per driver/axis)
  and `scan.json`. A chain that still stubs boxes is reported as informational only.
- `farmscan.sh NAME REF [MERGE...]`: the whole run on the arm64 farm (M1 Pro node).

Everything these write from a Mac's tuning or macOS renders is Apple-derived: write it to
a private directory, never into the repository.
