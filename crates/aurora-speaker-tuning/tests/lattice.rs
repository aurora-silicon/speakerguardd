// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The lsm2 masker lattice the installer provisions, on synthetic bundles:
//! read, re-validated with the installer's rule, absent with a warning.

mod common;

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::manifest::sha256_hex;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::Diagnostics;
use common::*;
use std::collections::BTreeMap;

fn lsm2_bundle(lattice: Option<Vec<u8>>) -> BTreeMap<String, Vec<u8>> {
    let mut files = spp3_files(4);
    files.insert(LSM2_METADATA.into(), lsm2_metadata_bytes(1.75));
    files.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph_text("lsm2").into_bytes());
    files.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"), strip("lsm2", 4));
    if let Some(l) = lattice {
        files.insert(LATTICE.into(), l);
    }
    files
}

fn open(files: &BTreeMap<String, Vec<u8>>, manifest: &str, name: &str) -> aurora_speaker_tuning::Result<Bundle> {
    let dir = scratch(name);
    write_dir(&dir, files, manifest);
    Bundle::open(&dir)
}

fn load(b: &Bundle) -> (Tuning, Diagnostics) {
    let mut d = Diagnostics::default();
    let t = Tuning::load(b, &Options::default(), &mut d).unwrap();
    (t, d)
}

fn has(d: &Diagnostics, code: &str) -> bool {
    d.items.iter().any(|x| x.code == code)
}

#[test]
fn a_provisioned_lattice_is_read_and_validated() {
    let ends = synthetic_lattice_ends();
    let files = lsm2_bundle(Some(lattice_bytes(&ends)));
    let b = open(&files, &manifest_json(&files, "25G83"), "lat-ok").unwrap();
    assert_eq!(b.manifest.mask_lattice.as_ref().unwrap().path, LATTICE);
    let (t, d) = load(&b);
    assert!(!d.has_errors(), "{:?}", d.items);
    assert!(!has(&d, "mask-lattice-absent"));
    let got = t.mask_lattice.expect("lattice");
    assert_eq!(got.iter().map(|&v| v as u64).collect::<Vec<_>>(), ends);
    assert!(t.mask_lattice_present);
    // The lattice is part of the bundle's identity.
    let without = lsm2_bundle(None);
    let other = open(&without, &manifest_json(&without, "25G83"), "lat-ok-id").unwrap();
    assert_ne!(b.digest(), other.digest());
}

#[test]
fn an_absent_lattice_warns_and_gives_no_lattice() {
    let files = lsm2_bundle(None);
    let b = open(&files, &manifest_json(&files, "25G83"), "lat-absent").unwrap();
    let (t, d) = load(&b);
    assert!(!d.has_errors(), "{:?}", d.items);
    assert!(has(&d, "mask-lattice-absent"));
    assert!(t.mask_lattice.is_none() && !t.mask_lattice_present);
    // Only lsm2 uses it: an spp3 bundle without one says nothing.
    let files = spp3_files(4);
    let b = open(&files, &manifest_json(&files, "25G83"), "lat-absent-spp3").unwrap();
    let (_, d) = load(&b);
    assert!(!has(&d, "mask-lattice-absent"));
}

#[test]
fn an_invalid_lattice_is_a_refusal() {
    let ends = synthetic_lattice_ends();
    let bad = |edit: &dyn Fn(&mut Vec<u64>)| {
        let mut e = ends.clone();
        edit(&mut e);
        lattice_bytes(&e)
    };
    for (name, bytes) in [
        ("final edge", bad(&|e| e[99] = 320)),
        ("not increasing", bad(&|e| e[80] = e[79])),
        ("singleton prefix", bad(&|e| e[3] = 5)),
        ("out of range", bad(&|e| e[99] = 400)),
    ] {
        // Hashed as the installer would: the manifest record matches.
        let files = lsm2_bundle(Some(bytes));
        let b = open(&files, &manifest_json(&files, "25G83"), &format!("lat-bad-{}", name.replace(' ', "-"))).unwrap();
        let (t, d) = load(&b);
        assert!(has(&d, "mask-lattice-invalid"), "{name}");
        assert!(!t.refusals.is_empty(), "{name}");
        assert!(t.mask_lattice.is_none(), "{name}");
    }
}

#[test]
fn a_malformed_or_changed_record_refuses_the_bundle() {
    let lat = lattice_bytes(&synthetic_lattice_ends());
    let files = lsm2_bundle(Some(lat.clone()));
    let good = manifest_json(&files, "25G83");
    for (name, from, to) in [
        ("version", r#""version":1"#, r#""version":2"#),
        ("format", r#""format":"u64le-exclusive-ends""#, r#""format":"u32""#),
        ("entries", r#""entries":100"#, r#""entries":99"#),
        ("path", r#""path":"lsm2-mask-lattice.bin""#, r#""path":"AID901/lattice.bin""#),
        ("bytes", r#""bytes":800,"sha256""#, r#""bytes":792,"sha256""#),
        ("unknown key", r#""entries":100"#, r#""entries":100,"x":1"#),
    ] {
        let m = good.replace(from, to);
        assert_ne!(m, good, "{name}");
        assert!(open(&files, &m, &format!("lat-rec-{}", name.replace(' ', "-"))).is_err(), "{name}");
    }
    // The bytes changed after collection: the hash refuses them.
    let mut changed = files.clone();
    let mut l = lat;
    l[600] ^= 1;
    changed.insert(LATTICE.into(), l);
    assert!(open(&changed, &good, "lat-changed").is_err());
    // Listed but missing.
    let mut missing = files.clone();
    missing.remove(LATTICE);
    assert!(open(&missing, &good, "lat-missing").is_err());
    // The record names the synthetic table's own hash.
    assert!(good.contains(&sha256_hex(&files[LATTICE])));
}
