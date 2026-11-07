#!/usr/bin/env python3
# Per-lane, per-segment diff of one box (or the unit input) for one AID: diag.py AID BOX|unit [segfilter]
import json, os, subprocess, sys
import numpy as np
aid, box = int(sys.argv[1]), sys.argv[2]; flt = sys.argv[3] if len(sys.argv) > 3 else ""
B = "/srv/aurora/fast/o/swsweep"; md = f"{B}/tainted/mac/AID{aid}"; w = f"{B}/tainted/diag-AID{aid}"; os.makedirs(w, exist_ok=True)
st = json.load(open(f"{md}/status.json")); gm = json.load(open(f"{md}/meta.json"))
tree = "/srv/aurora/fast/o/spp3v/tainted/tunings-26B8125" if st["tree"].startswith("macos-27") else f"{B}/tainted/tunings-25G83"
binv = os.environ.get("BIN", f"{B}/bin/d21d8bc")
cmd = [f"{binv}/voicing-render", "--tunings-root", tree, "--aid", str(aid), "--in", f"{B}/prog.f32", "--out", f"{w}/v2.f32", "--mvsc", "1", "--mvol", "0"]
if box != "unit": cmd += ["--tap", f"{box}={w}/{box}.f32"]
subprocess.run(cmd, check=True, capture_output=True)
if box == "unit":
    m = np.fromfile(f"{md}/unit_in.f32", "<f4"); v = np.fromfile(f"{w}/v2.f32", "<f4"); ch = gm["channels"]
else:
    m = np.fromfile(f"{md}/boxes/{box}.run.out.f32", "<f4"); v = np.fromfile(f"{w}/{box}.f32", "<f4")
    ch = m.size // (json.load(open(f"{B}/prog.json"))["frames"])
m = m.reshape(-1, ch).astype(float); v = v.reshape(-1, ch).astype(float)
lag = int(os.environ.get("LAG", "0"))
if lag: v = np.concatenate([v[lag:], np.zeros((lag, ch))])
for sg in json.load(open(f"{B}/prog.json"))["segments"]:
    if flt not in sg["name"]: continue
    s, n = sg["start"], sg["frames"]; a = m[s + n // 2:s + n]; b = v[s + n // 2:s + n]
    ra = np.sqrt((a * a).mean(0)); rb = np.sqrt((b * b).mean(0))
    d = 20 * np.log10(np.maximum(rb, 1e-15) / np.maximum(ra, 1e-15))
    print(f"{sg['name']:22s}", " ".join(f"{20*np.log10(max(x,1e-15)):7.1f}/{y:+7.3f}" for x, y in zip(ra, d)))
    if os.environ.get("WIN") and sg["kind"] == "dynamic":
        for at in range(0, n - 479, 480):
            a = m[s + at:s + at + 480]; b = v[s + at:s + at + 480]
            ra = np.sqrt((a * a).mean(0)); rb = np.sqrt((b * b).mean(0))
            if max(ra.max(), rb.max()) < 1e-4: continue
            d = 20 * np.log10(np.maximum(rb, 1e-15) / np.maximum(ra, 1e-15))
            if np.abs(d).max() > 0.05:
                print(f"   win {at/48000:6.3f}s", " ".join(f"{20*np.log10(max(x,1e-15)):7.1f}/{20*np.log10(max(y,1e-15)):7.1f}" for x, y in zip(ra, rb)))
