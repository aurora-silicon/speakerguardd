// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Computes `SGV2_VOICING_LAW`: a SHA-256 over the source of every file
//! that can change how the voicing chain renders (the voicing compiler, the
//! tuning reader, and the engine's voicing and filter code). The macOS parity
//! attestations (`voicing::parity`) are bound to it, so a change to any of
//! those files makes them stale until the gate runs again.
//!
//! The protection units' own code (spp3, lsm2, atsp, sense, LSID) is left
//! out: it has its own rigs and does not render the voicing.

use sha2::{Digest, Sha256};
use std::path::{Path, PathBuf};

/// Directories hashed, relative to the workspace's `crates/`.
const ROOTS: &[&str] = &["aurora-speaker-tuning/src", "sgv2-core/src", "sgv2-lv2/src/voicing.rs"];
/// Files and directories (relative to `crates/`) that never render the voicing.
const SKIP: &[&str] = &[
    "aurora-speaker-tuning/src/bin",
    "aurora-speaker-tuning/src/spp3.rs",
    "aurora-speaker-tuning/src/spts.rs",
    "aurora-speaker-tuning/src/lsid.rs",
    "aurora-speaker-tuning/src/lattice.rs",
    "aurora-speaker-tuning/src/machine.rs",
    "aurora-speaker-tuning/src/units.rs",
    "aurora-speaker-tuning/src/voicing/parity.rs",
    "sgv2-core/src/spp3.rs",
    "sgv2-core/src/lsm2.rs",
    "sgv2-core/src/lsm2",
    "sgv2-core/src/atsp.rs",
    "sgv2-core/src/lsid.rs",
    "sgv2-core/src/sense.rs",
    "sgv2-core/src/telemetry.rs",
    "sgv2-core/src/failsafe.rs",
    "sgv2-core/src/ring.rs",
    "sgv2-core/src/machine.rs",
    "sgv2-core/src/placeholder.rs",
];

fn walk(base: &Path, rel: &str, out: &mut Vec<(String, PathBuf)>) {
    if SKIP.contains(&rel) {
        return;
    }
    let p = base.join(rel);
    if p.is_dir() {
        let mut names: Vec<String> = std::fs::read_dir(&p)
            .unwrap_or_else(|e| panic!("{}: {e}", p.display()))
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        names.sort();
        for n in names {
            walk(base, &format!("{rel}/{n}"), out);
        }
    } else if rel.ends_with(".rs") {
        out.push((rel.to_string(), p));
    }
}

fn main() {
    let crates = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap().to_path_buf();
    let mut files = Vec::new();
    for r in ROOTS {
        println!("cargo:rerun-if-changed={}", crates.join(r).display());
        if crates.join(r).exists() {
            walk(&crates, r, &mut files);
        }
    }
    let mut h = Sha256::new();
    h.update(b"sgv2 voicing law v1\n");
    for (rel, p) in &files {
        let data = std::fs::read(p).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
        h.update(rel.as_bytes());
        h.update([0]);
        h.update((data.len() as u64).to_le_bytes());
        h.update(&data);
    }
    // Without the sibling crates (an unusual packaging), the law is a hash
    // of nothing: no attestation matches and the check refuses.
    let law: String = h.finalize().iter().map(|b| format!("{b:02x}")).collect();
    println!("cargo:rustc-env=SGV2_VOICING_LAW={law}");
}
