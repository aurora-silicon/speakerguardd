#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Inventory a private tuning root in place. Run ONLY on an approved build host.

python3 tools/aid-inventory.py TUNING_ROOT --private-dir /.../private/run \
    --out-dir /.../public --source-revision COMMIT [--expected-aids 40]

Never copies inputs. All raw output, info JSON, blobs and renders stay private.
The output uses an allowlist: discrete structural enums, box names, counts,
diagnostic codes and public-source reason labels. It never exports diagnostic
messages, graph parameter defaults, coefficients, unit IDs or bundle manifests.
Builds default and lowflow-conservative tools, runs both compile modes and the
renderer for each AID, then repeats the complete sweep to verify reproducibility.

Temporary, exact-match source overlays count known law paths in the
actual full-chain render. They change no signal calculations and are restored
even on build failure. Use a dedicated checkout: do not run concurrent builds
in it. Instrumentation sources and build failures are preserved privately.
"""
import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess

REPO = Path(__file__).resolve().parents[1]
KNOWN_BOARDS = {9: ("j316s", "MacBook Pro (16-inch, M1 Pro, 2021)"),
                13: ("j413", "MacBook Air (13-inch, M2, 2022)"),
                35: ("j616s", "MacBook Pro (16-inch, M4 Pro)"),
                44: ("j700", "MacBook Neo"),
                47: ("j775c", "Mac Studio (M5 Max / M5 Ultra)"),
                48: ("j873s", "Mac mini (M5 Pro / M6)"),
                49: ("k114s", "K114 MacBook Pro (M5 Pro / M5 Max)"),
                50: ("k116s", "K116 MacBook Pro (M5 Pro / M5 Max)")}
TRACE_LAWS = {1: "cfrk-slow-release-TODO(SPEC)", 2: "tmst-type18-TODO(SPEC)",
              4: "lowflow-conservative", 8: "cfrk-true-peak-scale", 16: "tmst-type15-release-gap"}
LAW_SOURCES = {
    "cfrk-slow-release-TODO(SPEC)": "crates/sgv2-core/src/voicing/cfreak.rs:TAU_SLOW_S",
    "cfrk-ceiling-trim": "crates/sgv2-core/src/voicing/cfreak.rs:CEILING_TRIM_DB",
    "cfrk-true-peak-scale": "crates/sgv2-core/src/voicing/cfreak.rs:TRUE_PEAK_SCALE_DB",
    "gain-positive-clamp": "crates/sgv2-core/src/voicing/chain.rs:Run::set",
    "tmst-type18-TODO(SPEC)": "crates/sgv2-core/src/prestage.rs:process_conservative_high_shelf",
    "tmst-type15-release-gap": "crates/sgv2-core/src/prestage.rs:DynamicRun::process",
    "lowflow-conservative": "crates/sgv2-core/src/voicing/lowflow.rs:CONSERVATIVE_GUARD_DB",
    "cfrk-frame-rounding": "crates/aurora-speaker-tuning/src/voicing/cfreak.rs:compile",
    "interim-filter-unity-bound": "crates/aurora-speaker-tuning/src/prestage.rs:derive_lanes",
    "interim-voicing-boxes-unwired": "crates/aurora-speaker-tuning/src/prestage.rs:derive_lanes",
    "atsp-interim-protection-only": "crates/aurora-speaker-tuning/src/compile.rs:compile_with",
    "lsm2-pressure-masking-unwired": "crates/aurora-speaker-tuning/src/compile.rs:pressure_response",
    "lsm2-pressure-response-gap": "crates/aurora-speaker-tuning/src/compile.rs:pressure_response",
    "spp3-closed-loop-sense-unwired": "crates/aurora-speaker-tuning/src/compile.rs:compile_spp3",
}
PUBLIC_OPTIONAL_FIELDS = set(re.findall(r'\w+:\s*"(\w+)",\s*Opt\w+',
    (REPO / "crates/aurora-speaker-tuning/src/spp3.rs").read_text()))
# Match only literal phrases in public compiler/chain diagnostics. No inferred
# law based on tuning magnitudes or fitted thresholds is permitted here.
REASONS = [
    ("builtin dbcontrol", "dbcontrol-unimplemented"),
    ("lsid", "lsid-unimplemented"),
    ("no protection unit", "protection-unit-unimplemented"),
    ("more than one protection unit", "multiple-protection-units-refused"),
    ("two compressor boxes", "mstk-interim-multiple-boxes-refused"),
    ("only fourth order", "mstk-interim-crossover-order-refused"),
    ("release is switched off", "mstk-interim-release-off-refused"),
    ("per-band input gain", "mstk-interim-band-input-gain-refused"),
    ("saturation (parameter", "mstk-interim-saturation-refused"),
    ("is soloed", "mstk-interim-solo-refused"),
    ("upward or expanding branch", "mstk-interim-upward-expander-refused"),
    ("unimplemented boxes or graph parameters", "chain-stub-or-unwired-refused"),
    ("CompensateResonators", "spp3-CompensateResonators-refused"),
    ("UseFastPeakSampleControl", "spp3-UseFastPeakSampleControl-refused"),
    ("saturation flavor", "mstk-saturation-flavor-refused"),
    ("saved controls are invalid", "mstk-saved-controls-refused"),
    ("exact wire representation", "mstk-wire-representation-refused"),
    ("lowflow-conservative build", "lowflow-exact-detector-gap"),
    ("active-floor and release-tracking parity", "tmst-type15-release-gap"),
    ("least-passing setting", "tmst-volume-automation-bound"),
    ("not wired to the system volume", "volt-volume-unwired"),
    ("stubs", "chain-stubs-refused"),
    ("stub", "chain-stubs-refused"),
    ("voicing gap", "chain-gap-refused"),
    ("unwired", "chain-parameter-unwired"),
    ("bank other than", "spp3-bank-TODO(SPEC)"),
    ("no saved state", "saved-state-missing"),
    ("no preset", "preset-missing"),
    ("global saved state", "global-state-missing"),
    ("not carried", "box-state-not-carried"),
    ("channels select different amplifier models", "amplifier-model-mismatch"),
    ("mid or high rail", "amplifier-rail-refused"),
    ("model without resonator", "speaker-model-resonator-missing"),
    ("model enables", "speaker-model-feature-refused"),
    ("does not have", "required-field-missing"),
    ("required field", "required-field-missing"),
]

def reason(message):
    return next((label for phrase, label in REASONS if phrase in message), "unclassified-refusal")

def todo_law(code, message):
    field = re.search(r'\b(\w+) is absent: TODO\(SPEC\)', message)
    # Only identifiers also present in the public schema can become law labels.
    return code + (":" + field[1] if field and field[1] in PUBLIC_OPTIONAL_FIELDS else "")

def run(cmd, stem, cwd=REPO):
    with stem.with_suffix(".stdout").open("wb") as out, stem.with_suffix(".stderr").open("wb") as err:
        p = subprocess.run(list(map(str, cmd)), cwd=cwd, stdout=out, stderr=err, check=False)
    return p.returncode

TRACE_MODULE = '''
pub mod aid_inventory_trace {
    use std::sync::atomic::{AtomicUsize, AtomicU64, Ordering};
    static NODE: AtomicUsize = AtomicUsize::new(0);
    static HITS: [AtomicU64; 256] = [const { AtomicU64::new(0) }; 256];
    pub fn select(n: usize) { assert!(n < HITS.len()); NODE.store(n, Ordering::Relaxed); }
    pub fn hit(bit: u64) { HITS[NODE.load(Ordering::Relaxed)].fetch_or(bit, Ordering::Relaxed); }
    pub fn reset() { for h in &HITS { h.store(0, Ordering::Relaxed); } }
    pub fn get(n: usize) -> u64 { HITS[n].load(Ordering::Relaxed) }
}
'''

@contextmanager
def overlay(private):
    patches = {
        "crates/sgv2-core/src/lib.rs": [("extern crate alloc;", "extern crate alloc;\n" + TRACE_MODULE)],
        "crates/sgv2-core/src/voicing/chain.rs": [
            ("            self.runs[n].process(x, y, frames);", "            crate::aid_inventory_trace::select(n);\n            self.runs[n].process(x, y, frames);")],
        "crates/sgv2-core/src/voicing/cfreak.rs": [
            ("            self.slow += self.a_slow * (u - self.slow);", "            if u > self.slow { crate::aid_inventory_trace::hit(1); }\n            self.slow += self.a_slow * (u - self.slow);"),
            ("                if self.true_peak {", "                if self.true_peak {\n                    crate::aid_inventory_trace::hit(8);")],
        "crates/sgv2-core/src/prestage.rs": [
            ("            self.process_conservative_high_shelf(at, bufs, frames, n);", "            crate::aid_inventory_trace::hit(2);\n            self.process_conservative_high_shelf(at, bufs, frames, n);"),
            ("        if at.params.kind == DynamicKind::HighShelf && !at.params.smooth {", "        if at.params.kind == DynamicKind::Notch { crate::aid_inventory_trace::hit(16); }\n        if at.params.kind == DynamicKind::HighShelf && !at.params.smooth {")],
        "crates/sgv2-core/src/voicing/lowflow.rs": [
            ("                    state.conservative_floor\n", "                    crate::aid_inventory_trace::hit(4);\n                    state.conservative_floor\n")],
    }
    originals = {}
    probe = REPO / "crates/aurora-speaker-tuning/examples/aid-inventory-probe.rs"
    if probe.exists():
        raise RuntimeError("temporary probe path already exists")
    try:
        for rel, replacements in patches.items():
            path = REPO / rel
            original = path.read_text()
            text = original
            for before, after in replacements:
                if text.count(before) != 1:
                    raise RuntimeError("instrumentation anchor changed: " + rel)
                text = text.replace(before, after)
            originals[path] = original
            path.write_text(text)
            evidence = private / "instrumentation" / rel
            evidence.parent.mkdir(parents=True, exist_ok=True)
            evidence.write_text(text)
        shutil.copyfile(REPO / "tools/aid-inventory-probe.rs", probe)
        yield
    finally:
        for path, original in originals.items():
            path.write_text(original)
        if probe.exists():
            probe.unlink()

def build(private):
    binaries = {}
    with overlay(private):
        for feature in ("default", "conservative"):
            target = private / ("target-" + feature)
            cmd = ["cargo", "build", "--release", "-j8", "--target-dir", target,
                   "-p", "aurora-speaker-tuning", "--bin", "aurora-speaker-tuning",
                   "--example", "voicing-render", "--example", "aid-inventory-probe"]
            if feature == "conservative":
                cmd += ["--features", "sgv2-core/lowflow-conservative"]
            if run(cmd, private / ("build-" + feature)):
                raise RuntimeError("build failed; preserved privately: " + feature)
            binaries[feature] = {"cli": target / "release/aurora-speaker-tuning",
                                 "renderer": target / "release/examples/voicing-render",
                                 "probe": target / "release/examples/aid-inventory-probe"}
    return binaries

def diagnostic_summary(text, interim=False):
    codes = Counter(re.findall(r"(?:info|warning|error)\[([a-z0-9-]+)\]", text))
    todos = Counter()
    todo_laws = Counter()
    errors = []
    paths = Counter()
    for line in text.splitlines():
        code = re.match(r"(?:info|warning|error)\[([a-z0-9-]+)\]", line)
        if code and "TODO(SPEC)" in line:
            todos[code[1]] += 1
            todo_laws[todo_law(code[1], line)] += 1
        if line.startswith("error["):
            errors.append(reason(line))
        if code:
            law = {"prestage-scaled": "interim-filter-unity-bound",
                   "prestage-not-carried": "interim-voicing-boxes-unwired",
                   "prestage-atsp-protection-only": "atsp-interim-protection-only",
                   "cfrk-fractional-frames": "cfrk-frame-rounding",
                   "lsm2-6632-absent": "lsm2-pressure-masking-unwired",
                   "lsm2-6632-pressure": "lsm2-pressure-response-gap",
                   "compile-no-sense": "spp3-closed-loop-sense-unwired"}.get(code[1])
            if interim and code[1] == "prestage-group-bypass":
                law = "tmst-interim-group-cut-bound"
            if code[1] == "prestage-volume-automation" and "the voicing chain follows its volume automation" not in line:
                law = "tmst-volume-automation-bound"
            if law:
                paths[law] += 1
    return {"diagnostic_counts": dict(sorted(codes.items())),
            "todo_spec_diagnostic_counts": dict(sorted(todos.items())),
            "todo_spec_law_counts": dict(sorted(todo_laws.items())),
            "emitted_law_counts": dict(sorted(paths.items())),
            "refusal_reasons": sorted(set(errors))}

def program(path, channels=2):
    seed = 1
    with path.open("wb") as f:
        for block in range(96):
            for _ in range(512):
                seed = (seed * 1664525 + 1013904223) & 0xffffffff
                amplitude = 32.0 if 16 <= block < 32 else 0.1 if block < 48 else 0.0
                # Round at the same f32 operations as the Rust probe.
                f32 = lambda v: struct.unpack("<f", struct.pack("<f", v))[0]
                sample = f32(f32((seed >> 8) / 8388608.0 - 1.0) * f32(amplitude))
                f.write(struct.pack("<" + "f" * channels,
                                    *[sample if ch % 2 == 0 else f32(-sample * 0.75) for ch in range(channels)]))

def public_probe(raw):
    if "load_error" in raw and not raw.get("boxes"):
        return {"unit": None, "boxes": [], "load_refusal": reason(raw["load_error"])}
    compiled = {b["name"]: b for b in raw.get("compiled_boxes", [])}
    gaps = defaultdict(list)
    for name, message in raw.get("gaps", []):
        gaps[name].append(reason(message))
    hits = {b["name"]: b["mask"] for b in raw.get("trace", [])}
    unwired_boxes = {s.split(" -> ", 1)[1].rsplit(" ", 1)[0]
                     for s in raw.get("unwired", []) if " -> " in s}
    planned = {n["name"] for n in raw.get("plan", {}).get("nodes", [])}
    boxes = []
    for b in raw["boxes"]:
        name, subtype = b["name"], b["subtype"]
        c = compiled.get(name, {})
        bounds = c.get("configured_bounds", [])
        exercised = [law for bit, law in TRACE_LAWS.items() if hits.get(name, 0) & bit]
        laws = list(gaps[name]) + bounds
        if subtype in {"lsid", "dbcontrol"}:
            status = "unimplemented"
            laws.append(subtype + "-unimplemented")
        elif c.get("kind") == "stub":
            status = "stub"
            laws.append(subtype + "-stub")
        elif name in unwired_boxes:
            status = "unwired"
            laws.append("graph-parameter-unwired")
        elif c.get("kind") == "bypass":
            status = "exact"
        elif gaps[name]:
            status = "gap"
        elif bounds:
            status = "protective bound active"
        elif c.get("kind") == "nonfinite":
            status = "exact"
        elif b["protection"]:
            status = "gap"
            laws.append(subtype + "-protection-parity-unverified")
        elif name in planned or c:
            status = "gap"
            laws.append(subtype + "-exact-parity-unverified")
        else:
            # Structural graph plumbing is not a DSP exactness claim.
            status = "exact" if subtype in {"chsp", "chjn", "rebl", "srlc", "nfnp", "summ", "reblocker", "srlconverter"} else "unwired"
            if status == "unwired":
                laws.append(subtype + "-outside-voicing-plan")
        out = {"name": name, "subtype": subtype, "status": status,
               "channels": b.get("channels"),
               "compiled_kind": c.get("kind"),
               "in_voicing_plan": name in planned, "bypassed": b["bypass"],
               "laws": sorted(set(laws)), "configured_bounds": sorted(bounds),
               "exercised_bounds": sorted(law for law in exercised if law != "tmst-type15-release-gap"),
               "exercised_law_paths": sorted(exercised)}
        if "compiled_dynamic_type_counts" in c:
            out["compiled_dynamic_type_counts"] = c["compiled_dynamic_type_counts"]
        if status == "unimplemented":
            out["refusal_text"] = ("builtin dbcontrol on the unit's lanes is not carried" if subtype == "dbcontrol"
                                   else "lsid has no carried compiler implementation")
            out["refusal_evidence"] = "compiler capability; actual compile refusals are recorded separately"
        if "mstk" in b:
            out["mstk"] = {key: b["mstk"][key] for key in
                           ("algorithm", "quality", "ovs", "enabled_saturation_flavors") if key in b["mstk"]}
            if c and c.get("kind") not in ("bypass", "stub"):
                m = out["mstk"]
                out["laws"].append(f'mstk-alg{m["algorithm"]}-q{m["quality"]}-parity-unverified')
                if m["ovs"]:
                    out["laws"].append("mstk-ovs-parity-unverified")
                for flavor in sorted(set(m.get("enabled_saturation_flavors", []))):
                    out["laws"].append(f"mstk-saturation-flavor{flavor}-parity-unverified")
        if "tmst_slots" in b:
            slots = b["tmst_slots"]
            out["tmst"] = {
                "saved_type_counts": dict(sorted(Counter(str(s["type"]) for s in slots).items())),
                "enabled_type_counts": dict(sorted(Counter(str(s["type"]) for s in slots if s["enabled"]).items())),
                "running_type_counts": dict(sorted(Counter(str(s["type"]) for s in slots
                    if s["enabled"] and not s["group_bypassed"] and not b["bypass"]).items())),
            }
        boxes.append(out)
    diag = raw.get("diagnostics", {}).get("items", [])
    return {"unit": raw.get("unit"), "boxes": boxes,
            "load_refusal": reason(raw["load_error"]) if "load_error" in raw else None,
            "box_status_counts": dict(sorted(Counter(b["status"] for b in boxes).items())),
            "chain_refusal": reason(raw["chain_error"]) if "chain_error" in raw else None,
            "unwired_parameter_count": len(raw.get("unwired", [])),
            "diagnostic_counts": dict(sorted(Counter(d["code"] for d in diag).items())),
            "todo_spec_diagnostic_counts": dict(sorted(Counter(d["code"] for d in diag if "TODO(SPEC)" in d["message"]).items())),
            "todo_spec_law_counts": dict(sorted(Counter(todo_law(d["code"], d["message"]) for d in diag
                                                       if "TODO(SPEC)" in d["message"]).items()))}

def sweep(root, aids, private, binaries, bundles, source_tag="host"):
    rows = []
    sources = [(aid, source_tag, ["--tunings-root", str(root), "--aid", str(aid)]) for aid in aids]
    sources += [(None, tag, ["--bundle", str(path)]) for tag, path in bundles]
    for aid, tag, source in sources:
        d = private / (f"AID{aid}" if aid is not None else "bundle-" + tag)
        d.mkdir(parents=True)
        # Probe first so missing topology cannot hide the compiler outcomes.
        variants = {}
        for feature, bins in binaries.items():
            stem = d / (feature + "-probe")
            rc = run([bins["probe"], *source], stem)
            if rc:
                raise RuntimeError(f"probe failed for {tag}/AID{aid}; private evidence preserved")
            raw = json.loads(stem.with_suffix(".stdout").read_text())
            public = public_probe(raw)
            if aid is None:
                aid = raw.get("aid")
            compiles = {}
            binding = raw.get("binding", {})
            board = binding.get("board") or KNOWN_BOARDS.get(aid, ("j473", "unknown"))[0]
            for mode in ("default", "voicing-pre"):
                stem = d / (feature + "-" + mode)
                cmd = [bins["cli"], "compile", "--development", "--board", board, *source,
                       "-o", str(stem) + ".blob"]
                if binding.get("unit_id"):
                    cmd += ["--unit-id", binding["unit_id"]]
                if mode == "voicing-pre":
                    cmd.append("--voicing-pre")
                rc = run(cmd, stem)
                text = stem.with_suffix(".stderr").read_text()
                compiles[mode] = {"rc": rc, "status": "accepted" if rc == 0 else "refused", **diagnostic_summary(text, interim=mode == "default")}
            # --info includes Apple graph defaults: retain it privately only.
            inp = d / (feature + "-program.f32")
            program(inp, raw.get("plan", {}).get("program_channels", 2))
            stem = d / (feature + "-render")
            rc = run([bins["renderer"], *source, "--in", inp, "--out", str(stem) + ".f32",
                      "--info", str(stem) + ".json"], stem)
            info = Path(str(stem) + ".json")
            if info.exists():
                j = json.loads(info.read_text())
                # Independent renderer/probe consistency check, no graph defaults exported.
                assert j["stubs"] == raw.get("stubs", [])
                assert j["gaps"] == raw.get("gaps", [])
                assert j["unwired"] == raw.get("unwired", [])
                assert j["plan"] == raw.get("plan")
            public["compiles"] = compiles
            public["render"] = {"rc": rc, "info_written": info.exists(),
                                "refusal_reason": reason(stem.with_suffix(".stderr").read_text()) if rc else None}
            variants[feature] = public
        subtypes = {b["subtype"] for b in variants["default"]["boxes"]}
        refused = any(c["rc"] for v in variants.values() for c in v["compiles"].values())
        # These are exclusive inventory categories, never ship-gate results.
        category = "refused" if refused else "bounded" if any(b["configured_bounds"] for v in variants.values() for b in v["boxes"]) else "gap"
        if all(b["status"] == "exact" for v in variants.values() for b in v["boxes"]) and not refused and subtypes:
            category = "exact"
        board, model = KNOWN_BOARDS.get(aid, (None, None))
        rows.append({"aid": aid, "source": tag, "board": board, "model": model,
                     "category": category, "presence": {name: subtype in subtypes for name, subtype in
                        [("LowFlow", "flow"), ("Norm", "ldnm"), ("VirtBass", "vben"), ("XTC", "axtc")]},
                     "variants": variants})
    return rows

def report(rows, revision):
    baseline = {r["aid"] for r in rows if r["source"] == "host"}
    newer_tags = sorted({r["source"] for r in rows if r["source"].startswith("27.")})
    for row in rows:
        row["tuning_release"] = "25G83" if row["source"] == "host" else row["source"]
        row["only_in_27_2"] = row["source"] in newer_tags and row["aid"] not in baseline
        row["parity_reference"] = "25G83"
    rollup = defaultdict(set)
    for row in rows:
        for variant in row["variants"].values():
            for key in ("chain_refusal", "load_refusal"):
                if variant.get(key):
                    rollup[variant[key]].add(row["aid"])
            if variant["render"].get("refusal_reason"):
                rollup[variant["render"]["refusal_reason"]].add(row["aid"])
            for law in variant.get("todo_spec_law_counts", {}):
                rollup["TODO(SPEC):" + law].add(row["aid"])
            for box in variant["boxes"]:
                for law in box["laws"]:
                    rollup[law].add(row["aid"])
            for mode in variant["compiles"].values():
                for law in mode["emitted_law_counts"]:
                    rollup[law].add(row["aid"])
                for law in mode["refusal_reasons"]:
                    rollup[law].add(row["aid"])
                for law in mode["todo_spec_law_counts"]:
                    rollup["TODO(SPEC):" + law].add(row["aid"])
    counts = Counter(r["category"] for r in rows if r["source"] == "host")
    compile_counts = {feature: {mode: dict(Counter(r["variants"][feature]["compiles"][mode]["status"]
                        for r in rows if r["source"] == "host"))
                        for mode in ("default", "voicing-pre")} for feature in ("default", "conservative")}
    return {"schema_version": 1, "source_revision": revision,
            "tool_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                            (REPO / "tools/aid-inventory.py", REPO / "tools/aid-inventory-probe.rs")},
            "scope": "default speaker selection for every AID directory; development provenance; 48 kHz; compile acceptance is not exact parity",
            "privacy": "Only structural enum IDs, names, status labels and counts; no tuning magnitudes or private identities",
            "counts": {k: counts[k] for k in ("exact", "bounded", "refused", "gap")},
            "category_definition": "refused: at least one requested compile mode refuses; bounded: all compile modes accept and a bound is configured; gap: accepted with unresolved parity and no configured bound; exact: only exact boxes and every compile accepted",
            "compile_counts": compile_counts,
            "newer_root_counts": {tag: dict(sorted(Counter(r["category"] for r in rows if r["source"] == tag).items())) for tag in newer_tags},
            "only_in_27_2_aids": sorted({r["aid"] for r in rows if r["only_in_27_2"]}),
            "aids": rows, "per_law": {k: sorted(v) for k, v in sorted(rollup.items())},
            "law_sources": LAW_SOURCES,
            "limitations": [
                "Exact is reserved for bypass/nonfinite/plumbing; active DSP without a parity certificate remains a gap.",
                "Bounded means a protective bound is configured; exercised_bounds records actual instrumented branch hits independently.",
                "The short synthetic render does not qualify all histories, volumes or sample rates and does not render the protection unit.",
                "TODO(SPEC) counts are emitted diagnostics; source-only TODOs are not falsely reported as exercised.",
                "Default host selection only; explicit optional bundles provide actual rig selections. Vendor configurations are not exhaustively swept.",
                "Temporary counters cover CFreak nontrivial slow release and true-peak scaling, type-15 execution, type-18 conservative dispatch and active conservative LowFlow; other configured bounds have no hit counter.",
            ]}

def markdown(data):
    lines = ["# AID exact-parity inventory", "", "Source: `" + data["source_revision"] + "`. " + data["scope"] + ".", "",
             "Counts (exclusive, host AIDs): " + ", ".join(f"{k}={v}" for k, v in data["counts"].items()) + ".", "",
             data["category_definition"] + ".", "",
             "Compile acceptance by build/mode: `" + json.dumps(data["compile_counts"], sort_keys=True) + "`.", "",
             "Newer-root categories: `" + json.dumps(data["newer_root_counts"], sort_keys=True) + "`. Only in 27.2: " +
             ", ".join(map(str, data["only_in_27_2_aids"])) + ".", "",
             "Accepted blobs and empty chain gaps do not certify macOS parity. Every bound remains an open item. "
             "Algorithm/quality/ovs and tmst types below are discrete structural enums; no tuning magnitudes are included.", "",
             "## Release comparison", "", "25G83 remains the parity reference. Newer tuning availability is not a new parity reference.", "",
             "| AID | 25G83 parity reference | 27.2 inventory | Only in 27.2 |",
             "|---|---|---|---|"]
    reference = {r["aid"]: r for r in data["aids"] if r["source"] == "host"}
    newer = defaultdict(list)
    for r in data["aids"]:
        if r["source"].startswith("27."):
            newer[r["aid"]].append(r)
    for aid in sorted(set(reference) | set(newer)):
        lines.append(f'| {aid} | {reference[aid]["category"] if aid in reference else "absent"} | ' +
                     "; ".join(r["source"] + ": " + r["category"] for r in newer[aid]) +
                     f' | {"yes" if aid not in reference else "no"} |')
    lines += ["", "## Compile and presence matrix", "",
              "| AID | Source / known board | Unit | Category | Default / pre | Conservative default / pre | LowFlow / Norm / VirtBass / XTC |",
              "|---|---|---|---|---|---|---|"]
    for r in data["aids"]:
        variants = r["variants"]
        cell = lambda v: " / ".join("OK" if v["compiles"][m]["rc"] == 0 else "refused" for m in ("default", "voicing-pre"))
        identity = r["board"] or "unknown"
        if r["model"]:
            identity += " (" + r["model"] + ")"
        lines.append(f'| {r["aid"]} | {r["source"]} / {identity} | {variants["default"]["unit"]} | {r["category"]} | {cell(variants["default"])} | {cell(variants["conservative"])} | ' + " / ".join("yes" if b else "no" for b in r["presence"].values()) + " |")
    lines += ["", "## Boxes and emitted findings per AID", ""]
    for r in data["aids"]:
        lines += [f'### AID{r["aid"]} ({r["source"]})', "", "| Box | Type | Default law status | Conservative law status | Discrete controls / active tmst types | Open laws |", "|---|---|---|---|---|---|"]
        cons = {b["name"]: b for b in r["variants"]["conservative"]["boxes"]}
        for b in r["variants"]["default"]["boxes"]:
            c = cons[b["name"]]
            controls = ""
            if "mstk" in b:
                controls = "alg/q/ovs=" + "/".join(str(b["mstk"][k]) for k in ("algorithm", "quality", "ovs"))
                controls += "; saturation flavors=" + ",".join(map(str, b["mstk"].get("enabled_saturation_flavors", [])))
            if "tmst" in b:
                controls = "running types " + ",".join(b["tmst"]["running_type_counts"]) + "; saved 15/18=" + "/".join(str(b["tmst"]["saved_type_counts"].get(k, 0)) for k in ("15", "18"))
            laws = "; ".join(sorted(set(b["laws"] + c["laws"])))
            if b.get("refusal_text"):
                laws += "; " + b["refusal_text"]
            lines.append(f'| {b["name"]} | {b["subtype"]} | {b["status"]} | {c["status"]} | {controls} | {laws} |')
        lines.append("")
        for feature, v in r["variants"].items():
            for mode, c in v["compiles"].items():
                lines.append(f'- {feature}/{mode}: {c["status"]}; refusals: ' + (", ".join(c["refusal_reasons"]) or "none") + "; TODO(SPEC) law counts: " + json.dumps(c["todo_spec_law_counts"], sort_keys=True) + "; emitted bound/gap paths: " + json.dumps(c["emitted_law_counts"], sort_keys=True) + ".")
            exercised = {b["name"]: b["exercised_law_paths"] for b in v["boxes"] if b["exercised_law_paths"]}
            lines.append(f'- {feature} render rc={v["render"]["rc"]}; unwired parameter count={v.get("unwired_parameter_count",0)}; exercised law paths: ' + json.dumps(exercised, sort_keys=True) + ".")
        lines.append("")
    lines += ["## Per-law rollup", "", "| Law / open item | AIDs needing it | Count |", "|---|---|---|"]
    for law, aids in data["per_law"].items():
        lines.append(f'| {law} | ' + ", ".join(map(str, aids)) + f' | {len(aids)} |')
    lines += ["", "## Evidence limits", ""] + ["- " + s for s in data["limitations"]]
    return "\n".join(lines) + "\n"

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("tuning_root", type=Path)
    parser.add_argument("--private-dir", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--source-revision", required=True)
    parser.add_argument("--expected-aids", type=int, default=40)
    parser.add_argument("--bundle", action="append", default=[], metavar="TAG=PATH")
    parser.add_argument("--newer-root", action="append", default=[], metavar="27.2-LABEL=PATH",
                        help="Additional tuning root; 25G83 remains the parity reference")
    args = parser.parse_args()
    private, out = args.private_dir.resolve(), args.out_dir.resolve()
    if not {"private", "tainted"}.intersection(private.parts):
        parser.error("--private-dir must be under a private/ or tainted/ directory")
    if private.exists():
        parser.error("use a new private directory so failures and earlier runs are preserved")
    private.mkdir(parents=True, mode=0o700)
    os.chmod(private, 0o700)
    root = args.tuning_root.resolve()
    aids = sorted(int(p.name[3:]) for p in root.iterdir() if re.fullmatch(r"AID[0-9]+", p.name) and p.is_dir())
    if len(aids) != args.expected_aids:
        parser.error(f"expected {args.expected_aids} AIDs, found {len(aids)}; verify root")
    bundles = [(tag, Path(path)) for tag, path in (s.split("=", 1) for s in args.bundle)]
    if any(not re.fullmatch(r"[a-z0-9-]+", tag) for tag, _ in bundles):
        parser.error("bundle tags must contain only lowercase letters, digits and hyphens")
    binaries = build(private)
    first = sweep(root, aids, private / "sweep-1", binaries, bundles)
    second = sweep(root, aids, private / "sweep-2", binaries, bundles)
    if first != second:
        raise RuntimeError("sweep mismatch; both private runs preserved")
    for label, path in (s.split("=", 1) for s in args.newer_root):
        if not re.fullmatch(r"27\.[a-zA-Z0-9.-]+", label):
            parser.error("newer-root labels must start with 27. and contain only letters, digits, dots or hyphens")
        newer_root = Path(path).resolve()
        newer_aids = sorted(int(p.name[3:]) for p in newer_root.iterdir()
                            if re.fullmatch(r"AID[0-9]+", p.name) and p.is_dir())
        newer_first = sweep(newer_root, newer_aids, private / label / "sweep-1", binaries, [], source_tag=label)
        newer_second = sweep(newer_root, newer_aids, private / label / "sweep-2", binaries, [], source_tag=label)
        if newer_first != newer_second:
            raise RuntimeError("newer-root sweep mismatch; private evidence preserved")
        first.extend(newer_first)
    data = report(first, args.source_revision)
    out.mkdir(parents=True, exist_ok=True)
    (out / "aid-inventory.json").write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    (out / "aid-inventory.md").write_text(markdown(data))
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.glob("aid-inventory.*")}
    (out / "verification.json").write_text(json.dumps({"identical_sweeps": 2, "hashes": hashes}, indent=2) + "\n")
    print(json.dumps({"counts": data["counts"], "identical_sweeps": 2, "hashes": hashes}, sort_keys=True))

if __name__ == "__main__":
    main()
