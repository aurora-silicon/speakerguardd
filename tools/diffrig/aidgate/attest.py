#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""macOS parity attestations for `speakerguardd-selfcheck verify` (all AIDs, automatic).

Every tuning a macOS release can select is rendered through that release's own Audio Units
and through v2's voicing chain; the result, one entry per tuning key, is the attestation
file v2 reads on a Mac before it may take the speakers (crates/aurora-speaker-tuning
src/voicing/parity.rs). It covers every AID and every speaker configuration of the
release's Tunings folder, with no list of boards: what it records is whether this code
renders that tuning like macOS, never which Macs are allowed.

Three steps, re-run in full for every macOS release and for every change to the rendering
code (`law`, see parity.rs; a stale file refuses every Mac until it is regenerated):

  1. variants (buildbox): every AID x every speaker configuration in the tree, deduplicated
     by tuning key (two speaker IDs whose strips are identical share one entry):
       attest.py variants --bin BIN --build BUILD --tunings TREE [--tunings-fallback TREE] --out variants.json
  2. macOS references (the host Mac running that release, under a host-heavy slot):
       attest_refs.sh SLOTDIR variants.json         (gr.py per variant, strip + property strip)
     and once per volume slider below full scale (SLIDERS), into its own directory:
       SLIDER=0.46 DEST=REFS-0.46 attest_refs.sh SLOTDIR variants.json
  3. macOS spread (host + buildbox): the same references rendered again from the program
     with every nonzero sample moved by one Float32 ULP (``perturb``, at least four seeds), then
       attest.py spread --variants variants.json --mac REFDIR --perturbed P1 P2 P3 P4 \
             --prog PROG --reference 25G83 --out spread-25G83.json
  4. gate (buildbox): render each variant through v2 and compare with its reference, at full
     volume and at every slider (settled band levels, the same tolerance):
       attest.py gate --variants variants.json --mac REFDIR --bin BIN --prog PROG \
             --reference "macOS 27.0 26A428 Audio Units" [--spread spread-26A428.json] \
             --slider 0.7=REFS-0.7 --slider 0.46=REFS-0.46 --slider 0.25=REFS-0.25 \
             [--slider-spread 0.7=spread-26A428-s0.7.json] --out parity.json [--details DIR]
     A release with no host for slider references needs --full-volume-only (README).

BUILD is the macOS build whose Tunings and installed Audio Units the run uses (26A428). The
gate writes one release block for it into parity.json and keeps the blocks of other releases
already there (same law only). `verify` looks up the block of the tuning's law build
(crates/aurora-speaker-tuning src/reference.rs). v2 implements the laws of the reference
release, 25G83: a tuning whose speaker files for its AID (speaker strips and property strips,
protection sidecars, speaker graphs, Config/AID<n>.plist) are byte-identical to 25G83's is
compiled with 25G83's laws on any macOS build, so v2 renders exactly what the 25G83 block
measured for its key, and that block decides. Any other tuning keeps the build its bundle came
from, and a Mac whose bundle comes from a build without a block is refused by `verify`: such a
release needs its own run, never an assumption that an older release's Audio Units render it
the same. The reference mapping makes v2 render like 25G83, not like the newer release; how
close a newer release's own units come is what that release's block (when one is run) shows.

A tuning is refused when: the macOS reference failed or is missing; v2's chain does not
compile, has a stub, an unwired graph parameter or a compiler gap; the lane counts
differ; any sample is non-finite; or any measured cell (second-half RMS of a steady
segment, or a 10 ms window of a dynamic one, both sides above the crossover stop band) is
louder than macOS by more than LOUDER_DB or quieter by more than QUIETER_DB. A measurement
more than FLOOR_RESIDUE_DB under its lane's own peak where v2 is quieter than macOS is floor
residue (README: Float32 arithmetic residue of the macOS units): it is reported and counted,
never refused. Louder is refused at any level. A deep, low-level measurement where macOS's
own +-1 LSB spread (``spread``, a fourth step) exceeds the tolerance is held to that spread
instead (README, "Arithmetic-chaotic cells"). The file
holds keys, verdicts, reasons and worst numbers only: no Apple data and no audio.
--details DIR (a private, tainted directory) keeps the per-cell rows.

BIN holds voicing-render, aurora-speaker-tuning and REV (the commit they were built from).
"""
import argparse, json, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

LOUDER_DB = 0.25
QUIETER_DB = 1.0
FLOOR = 1e-4
STOPBAND = 60.0
# A measurement this far (dB) under its lane's own peak (the loudest macOS level that lane
# reaches anywhere in the program) where v2 is QUIETER than macOS is floor residue: reported,
# not refused. Louder than macOS is refused at every level. See README.md, "Floor residue".
FLOOR_RESIDUE_DB = 55.0
# Arithmetic-chaotic cells (README.md): macOS's own run-to-run spread under a +-1 LSB program
# perturbation, measured per cell, becomes the cell's tolerance (louder included) when it exceeds
# the ordinary one, but only for measurements below CHAOTIC_MAX_DBFS absolute and more than
# CHAOTIC_UNDER_PEAK_DB under the lane's own peak. Main-band measurements keep the ordinary
# tolerance whatever the spread.
CHAOTIC_MAX_DBFS = -60.0
CHAOTIC_UNDER_PEAK_DB = 40.0
# A main-band measurement may also differ by macOS's own spread for that cell, but never by more
# than MAIN_SPREAD_CAP_DB (louder or quieter); beyond that the ordinary tolerance decides.
MAIN_SPREAD_CAP_DB = 0.25
SPREAD_FORMAT = 1
FORMAT = 2
# Volume slider positions (the user scalar, graph `mvsc`) every tuning is gated at besides full
# volume. Each is rendered on both sides with the same (mvsc, mvol) pair: mvol is the dB the
# chain's volume law (the HAL law over the Mac's own range, crates/sgv2-core voicing/volume.rs)
# gives for that scalar, the value macOS's HAL hands the graph.
SLIDERS = (0.7, 0.46, 0.25)
# Settled band levels at a slider are measured from this frequency up (the program's lowest tone
# is 20 Hz). Sample rate of the gate program.
SETTLED_HP_HZ = 10.0
FS = 48000


def tool(bin_dir, name):
    return os.path.join(bin_dir, name)


BUILD_ENV = {}


def run_json(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=900, env={**os.environ, **BUILD_ENV})
    try:
        return r.returncode, json.loads(r.stdout)
    except ValueError:
        return r.returncode, {"error": (r.stderr or r.stdout)[-600:]}


def aids_of(tree):
    return sorted(int(d[3:]) for d in os.listdir(tree) if d.startswith("AID") and d[3:].isdigit())


def variants(a):
    """Every (AID, speaker configuration) of the tree, one per tuning key."""
    trees = [a.tunings] + ([a.tunings_fallback] if a.tunings_fallback else [])
    aids = sorted(set().union(*(aids_of(t) for t in trees)))
    out, law = [], None
    for aid in aids:
        tree = next(t for t in trees if os.path.isdir(os.path.join(t, f"AID{aid}")))
        base = ["--tunings-root", tree, "--aid", str(aid)]
        rc, dump = run_json([tool(a.bin, "aurora-speaker-tuning"), "dump", "--json"] + base)
        sel = (dump.get("tuning") or {}).get("selection") or {}
        # the speaker IDs the database's general configurations name, and the default
        spids = [None] + sorted({c.get("speaker_id") for c in sel.get("all", [])
                                 if c.get("cdes_kind") == 0 and c.get("speaker_id") is not None})
        seen = {}
        for spid in spids:
            extra = [] if spid is None else ["--speaker-id", str(spid)]
            rc, k = run_json([tool(a.bin, "aurora-speaker-tuning"), "voicing-check", "--key-only"] + base + extra)
            if rc or "key" not in k:
                out.append({"aid": aid, "speaker_id": spid, "tree": tree, "key": None,
                            "error": k.get("error", "no key")})
                continue
            law = k["law"]
            if k["key"] in seen:
                seen[k["key"]]["also_speaker_ids"].append(spid)
                continue
            rc, d = run_json([tool(a.bin, "aurora-speaker-tuning"), "dump", "--json"] + base + extra)
            s = (d.get("tuning") or {}).get("selection") or {}
            v = {"aid": aid, "speaker_id": spid, "also_speaker_ids": [], "tree": tree, "key": k["key"],
                 "strip": os.path.basename(s.get("strip") or ""),
                 "propstrip": os.path.basename(s.get("propstrip") or "") or None,
                 "graph": os.path.basename(s.get("graph") or ""),
                 "volume": volume_points(a.bin, base + extra, os.path.dirname(os.path.abspath(a.out)))}
            seen[k["key"]] = v
            out.append(v)
        print(f"AID{aid}: {len(seen)} tuning(s)", flush=True)
    json.dump({"law": law, "build": a.build, "variants": out}, open(a.out, "w"), indent=1)
    return 0


def volume_points(bin_dir, args, work):
    """[[scalar, mvol dB], ...] for SLIDERS: the dB the chain's volume law gives each scalar
    (voicing-render --mvol auto on a 512-frame silent program)."""
    os.makedirs(work, exist_ok=True)
    z = os.path.join(work, "silence-512.f32")
    if not os.path.exists(z):
        with open(z, "wb") as f:
            f.write(bytes(8 * 512))
    out = []
    for sl in SLIDERS:
        r = subprocess.run([tool(bin_dir, "voicing-render")] + args + ["--in", z, "--out", os.devnull,
                           "--mvsc", repr(sl), "--mvol", "auto"], capture_output=True, text=True, timeout=300,
                           env={**os.environ, **BUILD_ENV})
        db = None
        for line in r.stderr.splitlines():
            if line.startswith("mvol auto:") and "->" in line:
                db = float(line.split("->")[1].split()[0])
        if r.returncode or db is None:
            return None
        out.append([sl, db])
    return out


def ref_name(v):
    return f"AID{v['aid']}" + ("" if v["speaker_id"] is None else f"-spid{v['speaker_id']}")


def rms_of(np):
    def rms(x):
        return float(np.sqrt(np.mean(np.square(x, dtype=np.float64)))) if len(x) else 0.0
    return rms


def measure(mac, v2, segs, np, stopband=True):
    """Every measurement of the gate program, per segment x lane: (seg, lane, [(delta dB,
    macOS RMS)]) with the lanes' own peaks. `stopband=False` measures crossover stop bands too
    (down to FLOOR), for macOS's own spread at a slider."""
    rms = rms_of(np)

    def ddb(x, y, floor):
        rx, ry = rms(x), rms(y)
        if max(rx, ry) < floor:
            return None
        return float(20 * np.log10(max(ry, 1e-15) / max(rx, 1e-15))), rx

    peak = lane_peaks(mac, segs, rms)
    out = []
    for sg in segs:
        if sg["kind"] == "silence":
            continue
        s, n = sg["start"], sg["frames"]
        top = max(rms(mac[s + n // 2:s + n, c]) for c in range(mac.shape[1]))
        floor = max(FLOOR, top * 10 ** (-STOPBAND / 20)) if stopband else FLOOR
        for c in range(mac.shape[1]):
            x, y = mac[s:s + n, c], v2[s:s + n, c]
            d = [ddb(x[n // 2:], y[n // 2:], floor)]
            if sg["kind"] == "dynamic":
                for at in range(0, n - 479, 480):
                    d.append(ddb(x[at:at + 480], y[at:at + 480], max(FLOOR, floor)))
            d = [v for v in d if v is not None]
            if d:
                out.append((sg["name"], c, d))
    return out, peak


def compare(mac, v2, segs, np, spread=None):
    """Per segment x lane: the largest louder and quieter deltas of the main measurements,
    the largest quieter floor-residue delta, and the arithmetic-chaotic measurements with
    their spread (dB; None when there is none). `spread`: {(seg, lane): dB}."""
    cells, peak = measure(mac, v2, segs, np)
    spread = spread or {}
    return [cell(seg, c, d, peak[c], spread.get((seg, c))) for seg, c, d in cells]


def lane_peaks(mac, segs, rms):
    """Each lane's own peak: its loudest macOS level (RMS of a steady second half or of a
    10 ms window of a dynamic segment) anywhere in the program."""
    peak = [0.0] * mac.shape[1]
    for sg in segs:
        if sg["kind"] == "silence":
            continue
        s, n = sg["start"], sg["frames"]
        for c in range(mac.shape[1]):
            lv = [rms(mac[s + n // 2:s + n, c])]
            if sg["kind"] == "dynamic":
                lv += [rms(mac[s + at:s + at + 480, c]) for at in range(0, n - 479, 480)]
            peak[c] = max(peak[c], *lv)
    return peak


def settled_view(x, segs, np):
    """`x` with the second half of every steady segment high-passed at SETTLED_HP_HZ (FFT, each
    lane): the settled band levels the slider gate compares. The program's lowest tone is 20 Hz,
    so the cut removes only subsonic drift, never program content."""
    y = np.array(x, dtype=np.float64, copy=True)
    for sg in segs:
        if sg["kind"] != "steady":
            continue
        a, b = sg["start"] + sg["frames"] // 2, sg["start"] + sg["frames"]
        f = np.fft.rfftfreq(b - a, 1 / FS)
        X = np.fft.rfft(y[a:b], axis=0)
        X[f < SETTLED_HP_HZ] = 0
        y[a:b] = np.fft.irfft(X, b - a, axis=0)
    return y


def settled(mac, v2, segs, np, spread=None):
    """Settled band levels at a volume slider: per steady segment (stepped third-octave sines,
    pink) x lane, the second-half RMS on both sides from SETTLED_HP_HZ up (`settled_view`).
    Below that sits only subsonic drift: the units' Float32 residue, which Tone automation
    lifts at low volume and macOS does not reproduce itself (README). The stop-band floor,
    the floor-residue rule and the arithmetic-chaotic rule apply as at full volume, against the
    lane's own settled peak; `spread` is macOS's own +-1 LSB spread measured at this slider
    (`spread --slider`), {(seg, lane): dB}."""
    steady = [sg for sg in segs if sg["kind"] == "steady"]
    cells, peak = measure(settled_view(mac, steady, np), settled_view(v2, steady, np), steady, np)
    spread = spread or {}
    return [cell(seg, c, d, peak[c], spread.get((seg, c))) for seg, c, d in cells]


def lane_worst(rows):
    """Per lane: [largest louder, largest quieter] dB over its main-band cells."""
    out = {}
    for r in rows:
        lo, qu = out.get(r["lane"], (0.0, 0.0))
        out[r["lane"]] = (max(lo, r["louder"]), min(qu, r["quieter"]))
    return [[round(out[c][0], 3), round(out[c][1], 3)] for c in sorted(out)]


def is_floor_residue(db, mac_level, lane_peak):
    """v2 quieter than macOS in a measurement more than FLOOR_RESIDUE_DB under its lane's peak."""
    return db < 0 and mac_level < lane_peak * 10 ** (-FLOOR_RESIDUE_DB / 20)


def is_chaotic(mac_level, lane_peak, spread):
    """A deep, low-level measurement with a measured macOS spread."""
    return (spread is not None and mac_level < 10 ** (CHAOTIC_MAX_DBFS / 20)
            and mac_level < lane_peak * 10 ** (-CHAOTIC_UNDER_PEAK_DB / 20))


def cell(seg, lane, meas, lane_peak, spread=None):
    """One measured cell from its (delta dB, macOS RMS) measurements."""
    res = [db for db, lv in meas if is_floor_residue(db, lv, lane_peak)]
    rest = [(db, lv) for db, lv in meas if not is_floor_residue(db, lv, lane_peak)]
    ch = [db for db, lv in rest if is_chaotic(lv, lane_peak, spread)]
    main = [db for db, lv in rest if not is_chaotic(lv, lane_peak, spread)]
    return {"seg": seg, "lane": lane, "louder": max(main) if main else 0.0,
            "quieter": min(main) if main else 0.0, "floor_quieter": min(res) if res else None,
            "chaotic": [max(ch), min(ch), spread] if ch else None, "spread": spread}


def verdict(rows):
    """The attestation verdict for measured cells ({"seg", "lane", "louder", "quieter",
    optional "floor_quieter"} in dB): refused on any cell louder than macOS by more than
    LOUDER_DB or quieter (outside the floor residue) by more than QUIETER_DB. Floor-residue
    cells are counted and reported, never refused."""
    if not rows:
        return {"verdict": "refused", "reason": "nothing measured", "measured": 0}
    def allow(r, tol):
        # main band: macOS's own spread, capped
        sp = r.get("spread")
        return tol if sp is None else max(tol, min(sp, MAIN_SPREAD_CAP_DB))
    lo = max(rows, key=lambda r: r["louder"])
    qu = min(rows, key=lambda r: r["quieter"])
    fl = [r for r in rows if r.get("floor_quieter") is not None and r["floor_quieter"] < -QUIETER_DB]
    why = []
    if any(r["louder"] > allow(r, LOUDER_DB) for r in rows):
        n = sum(r["louder"] > allow(r, LOUDER_DB) for r in rows)
        why.append(f"{n} cell(s) louder than macOS by more than {LOUDER_DB} dB, worst {lo['louder']:+.2f} dB "
                   f"(lane {lo['lane']}, {lo['seg']})")
    if any(r["quieter"] < -allow(r, QUIETER_DB) for r in rows):
        n = sum(r["quieter"] < -allow(r, QUIETER_DB) for r in rows)
        why.append(f"{n} cell(s) quieter than macOS by more than {QUIETER_DB} dB, worst {qu['quieter']:+.2f} dB "
                   f"(lane {qu['lane']}, {qu['seg']})")
    out = {"measured": len(rows), "worst_louder_db": round(lo["louder"], 3),
           "worst_quieter_db": round(qu["quieter"], 3), "verdict": "refused" if why else "pass",
           "reason": "; ".join(why) if why else f"within +{LOUDER_DB}/-{QUIETER_DB} dB of macOS on {len(rows)} cells"}
    ch = [r for r in rows if r.get("chaotic")]
    held = [r for r in ch if r["chaotic"][0] > LOUDER_DB or r["chaotic"][1] < -QUIETER_DB]
    bad = [r for r in ch if r["chaotic"][0] > max(LOUDER_DB, r["chaotic"][2])
           or r["chaotic"][1] < -max(QUIETER_DB, r["chaotic"][2])]
    if bad:
        w = max(bad, key=lambda r: max(abs(r["chaotic"][0]), abs(r["chaotic"][1])))
        why.append(f"{len(bad)} arithmetic-chaotic cell(s) outside macOS's own spread, worst "
                   f"{w['chaotic'][0]:+.2f}/{w['chaotic'][1]:+.2f} dB against {w['chaotic'][2]:.2f} dB "
                   f"(lane {w['lane']}, {w['seg']})")
        out["verdict"] = "refused"
        out["reason"] = "; ".join(why)
    if held:
        w = max(held, key=lambda r: max(abs(r["chaotic"][0]), abs(r["chaotic"][1])))
        out["chaotic_cells"] = len(held)
        out["worst_chaotic_db"] = round(max(w["chaotic"][:2], key=abs), 3)
        out["reason"] += (f"; {len(held)} arithmetic-chaotic cell(s) beyond the ordinary tolerance, worst "
                          f"{out['worst_chaotic_db']:+.2f} dB within macOS's own {w['chaotic'][2]:.2f} dB spread "
                          f"(lane {w['lane']}, {w['seg']})" if not bad else "")
    if fl:
        w = min(fl, key=lambda r: r["floor_quieter"])
        out["floor_residue_cells"] = len(fl)
        out["worst_floor_residue_db"] = round(w["floor_quieter"], 3)
        out["reason"] += (f"; {len(fl)} floor-residue cell(s) quieter than macOS by more than {QUIETER_DB} dB "
                          f"more than {FLOOR_RESIDUE_DB:g} dB under their lane's peak, worst {w['floor_quieter']:+.2f} dB "
                          f"(lane {w['lane']}, {w['seg']}): reported, not refused")
    return out


def gate_one(a, v, segs, np):
    e = {"key": v.get("key"), "aid": v["aid"], "speaker_id": v["speaker_id"],
         "also_speaker_ids": v.get("also_speaker_ids", []), "verdict": "refused"}
    if not v.get("key"):
        e["reason"] = f"no tuning key: {v.get('error', '')[:200]}"
        return e, None
    md = os.path.join(a.mac, ref_name(v))
    try:
        st = json.load(open(os.path.join(md, "status.json")))
    except OSError:
        e["reason"] = "no macOS reference render"
        return e, None
    if st.get("rc"):
        e["reason"] = f"the macOS reference render failed (rc {st['rc']})"
        return e, None
    extra = [] if v["speaker_id"] is None else ["--speaker-id", str(v["speaker_id"])]
    root = ["--tunings-root", v["tree"], "--aid", str(v["aid"])]
    rc, chk = run_json([tool(a.bin, "aurora-speaker-tuning"), "voicing-check", "--json"] + root + extra)
    if chk.get("key") != v["key"]:
        e["reason"] = "the tuning key changed between steps"
        return e, None
    bad = [s for s in chk.get("steps", []) if s["result"] == "refused" and s["name"] != "macos-parity"]
    if bad:
        e["reason"] = "; ".join(f"{s['name']}: {s['detail'][:240]}" for s in bad)
        return e, None
    gm = json.load(open(os.path.join(md, "meta.json")))
    nch = gm["channels"]
    mac = np.fromfile(os.path.join(md, "unit_in.f32"), "<f4").reshape(-1, nch)
    def render(mvsc, mvol, mac):
        """v2's lanes at (mvsc, mvol), or the refusal reason."""
        out = os.path.join(a.work, ref_name(v) + f".{mvsc!r}.v2.f32")
        r = subprocess.run([tool(a.bin, "voicing-render")] + root + extra +
                           ["--in", a.prog + ".f32", "--out", out, "--mvsc", repr(mvsc), "--mvol", repr(mvol)],
                           capture_output=True, text=True, timeout=900, env={**os.environ, **BUILD_ENV})
        if r.returncode:
            return None, f"v2 does not render the chain: {(r.stderr or r.stdout)[-240:]}"
        y = np.fromfile(out, "<f4")
        os.unlink(out)
        if y.size != mac.size:
            return None, f"lane count or length differs from macOS ({y.size} vs {mac.size} samples, {nch} lanes)"
        y = y.reshape(-1, nch)
        if not np.isfinite(y).all():
            return None, "non-finite output"
        return y, None

    v2, why = render(1.0, 0.0, mac)
    if why:
        e["reason"] = why
        return e, None
    rows = compare(mac, v2, segs, np, a.spread_cells.get(v["key"]))
    e.update(verdict(rows))
    details = {"full": rows, "sliders": {}}
    if a.slider_refs:
        e["sliders"] = []
        points = {sl: db for sl, db in (v.get("volume") or [])}
        for sl, refdir in a.slider_refs:
            s = {"slider": sl, "verdict": "refused"}
            e["sliders"].append(s)
            if sl not in points:
                s["reason"] = "no volume law point for this slider (attest.py variants)"
                continue
            s["mvol_db"] = round(points[sl], 4)
            sd = os.path.join(refdir, ref_name(v))
            try:
                sst = json.load(open(os.path.join(sd, "status.json")))
                sm = json.load(open(os.path.join(sd, "meta.json")))
            except OSError:
                s["reason"] = "no macOS reference render at this slider"
                continue
            if sst.get("rc"):
                s["reason"] = f"the macOS reference render failed (rc {sst['rc']})"
                continue
            if abs(sm["params"].get("mvsc", 1.0) - sl) > 1e-9 or abs(sm["params"].get("mvol", 0.0) - points[sl]) > 1e-6:
                s["reason"] = "the macOS reference was rendered at another volume"
                continue
            ms = np.fromfile(os.path.join(sd, "unit_in.f32"), "<f4").reshape(-1, sm["channels"])
            ys, why = render(sl, points[sl], ms)
            if why:
                s["reason"] = why
                continue
            srows = settled(ms, ys, segs, np, a.slider_spread.get(sl, {}).get(v["key"]))
            s.update(verdict(srows))
            details["sliders"][repr(sl)] = {"mvol_db": points[sl], "rows": srows, "lanes": lane_worst(srows)}
        bad = [s for s in e["sliders"] if s["verdict"] != "pass"]
        if bad and e["verdict"] == "pass":
            e["verdict"] = "refused"
            e["reason"] = "; ".join(f"slider {s['slider']:g}: {s['reason']}" for s in bad)
        elif bad:
            e["reason"] += "; " + "; ".join(f"slider {s['slider']:g}: {s['reason']}" for s in bad)
    return e, details


def load_spread(path):
    """{tuning key: {(seg, lane): dB}} from a spread file (empty without one)."""
    if not path:
        return {}
    doc = json.load(open(path))
    if doc.get("format") != SPREAD_FORMAT:
        raise SystemExit(f"{path}: spread format {doc.get('format')}")
    return {e["key"]: {(c[0], c[1]): c[2] for c in e["cells"]} for e in doc["entries"]}


def spread(a):
    """macOS's own per-cell spread: each reference against its +-1 LSB perturbed renders."""
    import numpy as np
    vs = json.load(open(a.variants))
    segs = json.load(open(a.prog + ".json"))["segments"]
    entries = []
    for v in vs["variants"]:
        if not v.get("key"):
            continue
        name = ref_name(v)
        try:
            gm = json.load(open(os.path.join(a.mac, name, "meta.json")))
            base = np.fromfile(os.path.join(a.mac, name, "unit_in.f32"), "<f4").reshape(-1, gm["channels"])
            runs = [np.fromfile(os.path.join(p, name, "unit_in.f32"), "<f4").reshape(-1, gm["channels"])
                    for p in a.perturbed]
        except (OSError, ValueError) as err:
            print(f"{name:16s} no spread ({err})", flush=True)
            continue
        worst = {}
        for r in runs:
            if a.slider is None:
                cells, _ = measure(base, r, segs, np)
            else:
                # At a slider the crossover stop band moves with the volume: a cell can sit just
                # under it in every macOS render and just over it in v2's. Measure macOS's own
                # spread down to the absolute floor, so such a cell gets its spread recorded.
                steady = [sg for sg in segs if sg["kind"] == "steady"]
                cells, _ = measure(settled_view(base, steady, np), settled_view(r, steady, np), steady, np,
                                   stopband=False)
            for seg, c, d in cells:
                m = max(abs(db) for db, _ in d)
                worst[(seg, c)] = max(worst.get((seg, c), 0.0), m)
        # record only what can matter: above the smallest gate tolerance (0.05 dB), rounded up
        keep = sorted(([k[0], k[1], float(np.ceil(m * 1000) / 1000)] for k, m in worst.items() if m > 0.05),
                      key=lambda c: (c[0], c[1]))
        entries.append({"key": v["key"], "name": name, "cells": keep})
        print(f"{name:16s} {len(keep):3d} cell(s) with spread > 0.05 dB, max "
              f"{max([c[2] for c in keep], default=0.0):.3f}", flush=True)
    doc = {"format": SPREAD_FORMAT, "reference": a.reference, "perturbations": len(a.perturbed),
           **({} if a.slider is None else {"slider": a.slider, "measurements": "settled: steady segments, second half"}),
           "method": "every nonzero program sample moved by one Float32 ULP, seeded; per cell the "
                     "largest |dB| of any measurement over all perturbed renders",
           "entries": entries}
    json.dump(doc, open(a.out, "w"), indent=1)
    return 0


def spread_id(path):
    """The spread rule a release block was gated with: the spread file's SHA-256 and the
    rule's limits (None without a spread file: the ordinary rule)."""
    if not path:
        return None
    import hashlib
    return {"sha256": hashlib.sha256(open(path, "rb").read()).hexdigest(),
            "chaotic_max_dbfs": CHAOTIC_MAX_DBFS, "chaotic_under_peak_db": CHAOTIC_UNDER_PEAK_DB,
            "main_spread_cap_db": MAIN_SPREAD_CAP_DB}


def gate(a):
    import numpy as np
    a.spread_cells = load_spread(a.spread)
    a.slider_refs = []
    for x in a.slider or []:
        sl, d = x.split("=", 1)
        a.slider_refs.append((float(sl), d))
    a.slider_spread, slider_spread_ids = {}, {}
    for x in a.slider_spread_file or []:
        sl, f = x.split("=", 1)
        doc = json.load(open(f))
        if doc.get("slider") != float(sl):
            raise SystemExit(f"{f}: spread measured at slider {doc.get('slider')}, not {sl}")
        a.slider_spread[float(sl)] = load_spread(f)
        slider_spread_ids[repr(float(sl))] = spread_id(f)
    if sorted(sl for sl, _ in a.slider_refs) != sorted(SLIDERS) and not a.full_volume_only:
        raise SystemExit(f"gate needs macOS references at every slider {SLIDERS} (--slider S=DIR), "
                         "or --full-volume-only with the reason in --reference")
    vs = json.load(open(a.variants))
    segs = json.load(open(a.prog + ".json"))["segments"]
    os.makedirs(a.work, exist_ok=True)
    if a.details:
        os.makedirs(a.details, mode=0o700, exist_ok=True)
    with ThreadPoolExecutor(a.jobs) as ex:
        res = list(ex.map(lambda v: gate_one(a, v, segs, np), vs["variants"]))
    entries = []
    for (e, rows), v in zip(res, vs["variants"]):
        entries.append(e)
        if a.details and rows is not None:
            json.dump(rows["full"], open(os.path.join(a.details, ref_name(v) + ".json"), "w"))
            if rows["sliders"]:
                json.dump(rows["sliders"], open(os.path.join(a.details, ref_name(v) + ".sliders.json"), "w"))
        print(f"{ref_name(v):16s} {e['verdict']:8s} {e['reason'][:160]}", flush=True)
        for sv in e.get("sliders", []):
            print(f"{'':16s}   slider {sv['slider']:<4g} {sv['verdict']:8s} louder {sv.get('worst_louder_db', float('nan')):+.3f}"
                  f" quieter {sv.get('worst_quieter_db', float('nan')):+.3f}  {sv.get('reason', '')[:120]}", flush=True)
    release = {
        "build": vs["build"],
        "reference": a.reference,
        "rev": open(os.path.join(a.bin, "REV")).read().strip(),
        "spread": spread_id(a.spread),
        "sliders": sorted((sl for sl, _ in a.slider_refs), reverse=True),
        "slider_spread": slider_spread_ids,
        "entries": sorted(entries, key=lambda e: (e["aid"], -1 if e["speaker_id"] is None else e["speaker_id"])),
    }
    doc = merge(a.out, vs["law"], release)
    json.dump(doc, open(a.out, "w"), indent=1)
    p = sum(e["verdict"] == "pass" for e in entries)
    print(f"{p}/{len(entries)} tunings pass", flush=True)
    return 0


def merge(path, law, release):
    """The attestation file with `release` added or replaced; other releases are kept only
    when they were gated with the same rendering code (law)."""
    doc = {"format": FORMAT, "law": law,
           "tolerance": {"louder_db": LOUDER_DB, "quieter_db": QUIETER_DB, "floor_residue_db": FLOOR_RESIDUE_DB},
           "releases": []}
    if os.path.exists(path):
        old = json.load(open(path))
        if old.get("format") == FORMAT and old.get("law") == law and old.get("tolerance") == doc["tolerance"]:
            doc["releases"] = [r for r in old["releases"] if r["build"] != release["build"]]
        else:
            dropped = [r["build"] for r in old.get("releases", [])] or ["(format 1 file)"]
            print(f"dropping releases gated with other code or tolerance: {', '.join(dropped)}", flush=True)
    doc["releases"] = sorted(doc["releases"] + [release], key=lambda r: r["build"])
    return doc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("variants")
    v.add_argument("--bin", required=True)
    v.add_argument("--tunings", required=True)
    v.add_argument("--tunings-fallback")
    v.add_argument("--build", required=True, help="the macOS build the Tunings come from (26A428)")
    v.add_argument("--out", required=True)
    g = sub.add_parser("gate")
    for k in ("variants", "mac", "bin", "prog", "reference", "out"):
        g.add_argument("--" + k, required=True)
    g.add_argument("--details")
    g.add_argument("--spread")
    g.add_argument("--slider", action="append", help="S=DIR: macOS references rendered at slider S (attest_refs.sh SLIDER=S)")
    g.add_argument("--slider-spread", dest="slider_spread_file", action="append",
                   help="S=FILE: macOS's own spread at slider S (attest.py spread --slider S)")
    g.add_argument("--full-volume-only", action="store_true",
                   help="write a block without slider verdicts (a release with no host for slider references)")
    g.add_argument("--work", default="/tmp/attest-work")
    g.add_argument("--jobs", type=int, default=8)
    sp = sub.add_parser("spread")
    for k in ("variants", "mac", "prog", "reference", "out"):
        sp.add_argument("--" + k, required=True)
    sp.add_argument("--perturbed", nargs="+", required=True)
    sp.add_argument("--slider", type=float, help="the references are at this volume slider: settled cells only")
    a = ap.parse_args()
    if a.cmd == "spread" and len(a.perturbed) < 4:
        raise SystemExit("spread needs at least four perturbed reference sets")
    if a.cmd == "variants":
        BUILD_ENV["SGV2_MACOS_BUILD"] = a.build
    else:
        BUILD_ENV["SGV2_MACOS_BUILD"] = json.load(open(a.variants))["build"]
    return {"variants": variants, "gate": gate, "spread": spread}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
