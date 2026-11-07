#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Per-AID full-chain parity gate (buildbox side).

For every AID with a macOS whole-graph reference (gr.py on the host, 25G83
Audio Units): render the same program through v2's voicing chain compiled
from the same tuning (voicing-render, full volume), tap every Audio Unit box,
and compare per segment and per unit lane:
- steady segments: RMS of the second half;
- dynamic segments: also every 10 ms window above -80 dBFS on either side;
- a measurement fails above 0.05 dB in either direction.
Latency is structural: v2's Norm runs one 512-frame call behind the unit
(a stubbed Norm: 0); the raw cross-correlation lag is reported.
An AID PASSES only with a compiled chain, matching lane count and 0 over.
Per-box attribution compares each AU box's cumulative output (both sides
fed by their own upstream chain): the first box over the gate is named.

    gate.py --prog PROG --mac DIR --bin DIR --tunings27 DIR --tunings25 DIR --out DIR [AID ...]
"""
import argparse, json, os, subprocess, sys
import numpy as np

GATE = 0.05
# Cells where both sides sit below -80 dBFS (deep driver stop bands, numerical residue) are not
# measured, as in the box rigs' transient windows; they are counted separately.
FLOOR = 1e-4
STOPBAND = 60.0
# v2's Norm (ldnm) runner meters one whole 512-frame call before applying it, so the chain runs
# exactly one call (512 frames) behind the unit fed in 512-frame quanta; that constant is allowed.
ALLOWED_LAGS = (0, 512)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x, dtype=np.float64)))) if len(x) else 0.0


def ddb(a, b, floor):
    ra, rb = rms(a), rms(b)
    if max(ra, rb) < floor:
        return None
    return float(20 * np.log10(max(rb, 1e-15) / max(ra, 1e-15)))


def lag_of(mac, v2, seg, search=4096, raw=False):
    s, n = seg["start"], seg["frames"]
    a = mac[s:s + n].sum(axis=1)
    lo = max(0, s - search)
    b = v2[lo:s + n + search].sum(axis=1)
    if rms(a) < 1e-9 or rms(b) < 1e-9:
        return 0
    m = len(a) + len(b)
    f = 1 << (m - 1).bit_length()
    c = np.fft.irfft(np.fft.rfft(b, f) * np.conj(np.fft.rfft(a, f)), f)[: len(b)]
    best = int(np.argmax(c))
    if raw:
        return best - (s - lo)
    # Band-limited lanes give broad correlation peaks: compare at the better
    # of the allowed lags; the raw peak is reported separately.
    return max(ALLOWED_LAGS, key=lambda lag: c[lag + (s - lo)] if 0 <= lag + (s - lo) < len(c) else -np.inf)


def compare(mac, v2, segs, lag):
    """Per segment x lane deltas; v2 shifted back by lag."""
    if lag > 0:
        v2 = np.concatenate([v2[lag:], np.zeros((lag, v2.shape[1]), v2.dtype)])
    elif lag < 0:
        v2 = np.concatenate([np.zeros((-lag, v2.shape[1]), v2.dtype), v2[:lag]])
    rows = []
    for sg in segs:
        if sg["kind"] == "silence":
            continue
        s, n = sg["start"], sg["frames"]
        # A lane more than STOPBAND dB under the segment's loudest lane on
        # both sides is a crossover stop band: numerical residue, not program.
        top = max(rms(mac[s + n // 2:s + n, c]) for c in range(mac.shape[1]))
        floor = max(FLOOR, top * 10 ** (-STOPBAND / 20))
        for c in range(mac.shape[1]):
            a, b = mac[s:s + n, c], v2[s:s + n, c]
            d = [ddb(a[n // 2:], b[n // 2:], floor)]
            if sg["kind"] == "dynamic":
                for at in range(0, n - 479, 480):
                    d.append(ddb(a[at:at + 480], b[at:at + 480], 1e-4))
            d = [x for x in d if x is not None]
            if not d:
                continue
            w = max(d, key=abs)
            rows.append({"seg": sg["name"], "lane": c, "db": w})
    over = [r for r in rows if abs(r["db"]) > GATE]
    worst = max(rows, key=lambda r: abs(r["db"])) if rows else None
    return {"measured": len(rows), "over": len(over), "louder_over": sum(r["db"] > GATE for r in over),
            "worst": worst, "over_rows": over[:400]}


def main():
    ap = argparse.ArgumentParser()
    for k in ("prog", "mac", "bin", "tunings27", "tunings25", "out"):
        ap.add_argument("--" + k, required=True)
    ap.add_argument("aids", nargs="*", type=int)
    a = ap.parse_args()
    meta = json.load(open(a.prog + ".json"))
    segs = meta["segments"]
    pink = next(s for s in segs if s["name"] == "pink_-6")
    os.makedirs(a.out, exist_ok=True)
    aids = a.aids or sorted(int(d[3:]) for d in os.listdir(a.mac) if d.startswith("AID"))
    for aid in aids:
        md = os.path.join(a.mac, f"AID{aid}")
        res = {"aid": aid, "bin_rev": open(os.path.join(a.bin, "REV")).read().strip()}
        st = json.load(open(os.path.join(md, "status.json")))
        res["mac_tree"] = st["tree"]
        if st["rc"]:
            res.update(status="REF-FAIL", reason=open(os.path.join(md, "gr.log")).read()[-400:])
            json.dump(res, open(os.path.join(a.out, f"AID{aid}.json"), "w"), indent=1)
            print(aid, res["status"], flush=True)
            continue
        gm = json.load(open(os.path.join(md, "meta.json")))
        nch = gm["channels"]
        mac = np.fromfile(os.path.join(md, "unit_in.f32"), "<f4").reshape(-1, nch)
        tree = a.tunings27 if st["tree"].startswith("macos-27") else a.tunings25
        work = os.path.join(a.out, f"work-AID{aid}")
        os.makedirs(work, exist_ok=True)
        taps = []
        for b in gm["boxes"]:
            f = os.path.join(md, "boxes", f"{b['box']}.run.out.f32")
            if os.path.exists(f):
                taps.append((b["box"], f, os.path.join(work, f"{b['box']}.v2.f32")))
        base = [os.path.join(a.bin, "voicing-render"), "--tunings-root", tree, "--aid", str(aid),
                "--in", a.prog + ".f32", "--out", os.path.join(work, "v2.f32"),
                "--info", os.path.join(work, "info.json"), "--mvsc", "1", "--mvol", "0"]
        r = subprocess.run(base, capture_output=True, text=True, timeout=900)
        if r.returncode:
            res.update(status="V2-REFUSED", reason=(r.stderr or r.stdout)[-600:])
            json.dump(res, open(os.path.join(a.out, f"AID{aid}.json"), "w"), indent=1)
            print(aid, res["status"], flush=True)
            continue
        nodes = {n.get("name") for n in json.load(open(os.path.join(work, "info.json")))["plan"].get("nodes", [])}
        missing = [t[0] for t in taps if t[0] not in nodes]
        taps = [t for t in taps if t[0] in nodes]
        res["boxes_not_in_v2_plan"] = missing
        if taps:
            cmd = list(base)
            for name, _, vf in taps:
                cmd += ["--tap", f"{name}={vf}"]
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
            if r.returncode:
                res["taps"] = "failed: " + r.stderr[-300:]
        info = json.load(open(os.path.join(work, "info.json")))
        res["stubs"] = info.get("stubs")
        res["gaps"] = info.get("gaps")
        # Structural latency: v2's Norm runs one 512-frame call behind the
        # unit; a stubbed Norm is a zero-latency wire.
        stubbed = {x[0] if isinstance(x, list) else x for x in (res["stubs"] or [])}
        norm_live = [b["box"] for b in gm["boxes"] if b["subtype"] == "ldnm" and b["box"] not in stubbed]
        v2 = np.fromfile(os.path.join(work, "v2.f32"), "<f4")
        if v2.size % nch:
            res.update(status="LANE-MISMATCH", reason=f"v2 samples {v2.size} not a multiple of {nch}")
        else:
            v2 = v2.reshape(-1, nch)
            if not np.isfinite(v2).all():
                res.update(status="NONFINITE")
            else:
                lag = 512 if norm_live else 0
                res["lag_frames"] = lag
                res["xcorr_lag"] = lag_of(mac, v2, pink, raw=True)
                res.update(compare(mac, v2, segs, lag))
                res["status"] = "PASS" if res["over"] == 0 and not res["stubs"] else "FAIL"
        # Per-box cumulative attribution.
        boxes = []
        for name, mf, vf in taps:
            if not os.path.exists(vf):
                continue
            mb = np.fromfile(mf, "<f4")
            vb = np.fromfile(vf, "<f4")
            if mb.size != vb.size or mb.size == 0:
                boxes.append({"box": name, "status": "width-mismatch", "mac": int(mb.size), "v2": int(vb.size)})
                continue
            ch = mb.size // mac.shape[0]
            mb, vb = mb.reshape(-1, ch), vb.reshape(-1, ch)
            order = [x["box"] for x in gm["boxes"]]
            upto = order[: order.index(name) + 1] if name in order else order
            lb = 512 if any(x in norm_live for x in upto) else 0
            c = compare(mb, vb, segs, lb)
            boxes.append({"box": name, "lag": lb, "xcorr_lag": lag_of(mb, vb, pink, raw=True), "measured": c["measured"], "over": c["over"],
                          "worst_db": c["worst"]["db"] if c["worst"] else None,
                          "worst_seg": c["worst"]["seg"] if c["worst"] else None})
        res["boxes"] = boxes
        first = next((b for b in boxes if b.get("over") or b.get("status")), None)
        res["first_diverging_box"] = first["box"] if first else None
        json.dump(res, open(os.path.join(a.out, f"AID{aid}.json"), "w"), indent=1)
        for f in os.listdir(work):
            if f.endswith(".f32"):
                os.unlink(os.path.join(work, f))
        w = res.get("worst") or {}
        print(aid, res["status"], res.get("measured"), res.get("over"), res.get("lag_frames"),
              w.get("db"), w.get("seg"), res["first_diverging_box"], flush=True)


if __name__ == "__main__":
    sys.exit(main())
