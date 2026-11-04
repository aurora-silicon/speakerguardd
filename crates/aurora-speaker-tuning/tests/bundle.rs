// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! End to end: a synthetic bundle on disk and as an archive, through
//! verification, selection, the graph, the strip and the sidecar.

mod common;

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::tuning::{Options, Tuning, UnitKind};
use aurora_speaker_tuning::{Diagnostics, Error};
use common::*;
use std::io::Write;

fn load(b: &Bundle, spid: Option<i128>) -> (Result<Tuning, Error>, Diagnostics) {
    let mut d = Diagnostics::default();
    let t = Tuning::load(b, &Options { speaker_id: spid }, &mut d);
    (t, d)
}

#[test]
fn directory_bundle_end_to_end() {
    let files = spp3_files(4);
    let dir = scratch("e2e-dir");
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    let b = Bundle::open(&dir).unwrap();
    assert!(b.verified);
    let (t, d) = load(&b, None);
    let t = t.unwrap();
    assert!(!d.has_errors(), "{:?}", d.items);
    // Only the speaker-ID warning: the build is a validated one.
    assert!(d.items.iter().any(|x| x.code == "select-speaker-id-unknown"));
    assert!(!d.items.iter().any(|x| x.code.starts_with("macos-build")));

    assert_eq!(t.aid, AID);
    assert_eq!(t.selection.configuration.as_deref(), Some("48k - General"));
    assert_eq!(t.selection.substitutions.get("fs").map(String::as_str), Some("48000"));
    let p = &t.protection;
    assert_eq!(p.kind, UnitKind::Spp3);
    assert_eq!(p.box_name, "Prot");
    assert_eq!(t.chain[p.position].name, "Prot");
    assert_eq!(t.chain.last().unwrap().name, "Fade");
    assert!(t.chain[..p.position].iter().all(|s| s.before_protection));
    assert_eq!(p.block_size, Some(256.0));
    assert_eq!(p.lanes.len(), 4);
    let tm = p.telemetry.as_ref().unwrap();
    assert_eq!((tm.port.as_str(), tm.channels, tm.interleaved), ("sense", Some(8), true));
    assert_eq!(tm.path, vec!["Conv".to_string()]);
    let sc = p.spp3.as_ref().unwrap();
    assert_eq!(sc.num_channels, 4);
    assert_eq!(p.property_files.len(), 1);
    let spts = t.spts.as_ref().unwrap();
    assert_eq!(spts.records.len(), 4);
    assert_eq!(spts.records[2].r_raw, 3002);
    // The Eq stage carries its saved state; the split builtin has none.
    let eq = t.chain.iter().find(|s| s.name == "Eq").unwrap();
    assert_eq!(eq.state.as_ref().unwrap().global().unwrap().get(2), Some(900.0));
    assert!(t.chain.iter().find(|s| s.name == "Spl").unwrap().state.as_ref().unwrap().sections.is_empty());
    // The dump serializes.
    assert!(serde_json::to_string(&t).unwrap().contains("\"kind\":\"spp3\""));
}

#[test]
fn tar_gz_bundle_with_top_directory() {
    let files = spp3_files(2);
    let manifest = manifest_json(&files, "26A1");
    let dir = scratch("e2e-tar");
    let path = dir.join("bundle.tar.gz");
    {
        let f = std::fs::File::create(&path).unwrap();
        let gz = flate2::write::GzEncoder::new(f, flate2::Compression::fast());
        let mut ar = tar::Builder::new(gz);
        let mut add = |name: &str, data: &[u8]| {
            let mut h = tar::Header::new_gnu();
            h.set_size(data.len() as u64);
            h.set_mode(0o644);
            h.set_cksum();
            ar.append_data(&mut h, format!("top/{name}"), data).unwrap();
        };
        for (p, d) in &files {
            add(p, d);
        }
        add("manifest.json", manifest.as_bytes());
        add("stray.txt", b"not listed");
        ar.into_inner().unwrap().finish().unwrap().flush().unwrap();
    }
    let b = Bundle::open(&path).unwrap();
    assert_eq!(b.unlisted, vec!["stray.txt".to_string()]);
    let (t, d) = load(&b, None);
    assert!(t.is_ok(), "{:?}", t.err());
    assert!(d.items.iter().any(|x| x.code == "macos-build-unvalidated"));
    assert!(d.items.iter().any(|x| x.code == "bundle-unlisted"));
}

#[test]
fn integrity_failures() {
    let files = spp3_files(2);
    let manifest = manifest_json(&files, "25G83");
    // A changed byte.
    let dir = scratch("e2e-tamper");
    let mut tampered = files.clone();
    tampered.get_mut(&format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg")).unwrap()[0] ^= 1;
    write_dir(&dir, &tampered, &manifest);
    assert!(matches!(Bundle::open(&dir), Err(Error::Integrity { .. })));
    // A missing file.
    let dir = scratch("e2e-missing");
    let mut missing = files.clone();
    missing.remove("speaker-thiele-small.bin");
    write_dir(&dir, &missing, &manifest);
    assert!(Bundle::open(&dir).is_err());
}

#[test]
fn selection_by_speaker_id() {
    let mut files = spp3_files(2);
    let alt = files[&format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip")].clone();
    files.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general_7.austrip"), alt);
    files.insert(
        "DeviceDSPDatabase.plist".into(),
        database(&[(65535, "builtin_speaker_out_general.austrip"), (7, "builtin_speaker_out_general_7.austrip")]),
    );
    let dir = scratch("e2e-spid");
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    let b = Bundle::open(&dir).unwrap();
    let (t, d) = load(&b, Some(7));
    let t = t.unwrap();
    assert!(t.selection.strip.ends_with("_7.austrip"));
    assert!(!d.items.iter().any(|x| x.code == "select-speaker-id-unknown"));
    assert_eq!(t.selection.all.len(), 3);
    let (t, _) = load(&b, None);
    assert!(t.unwrap().selection.strip.ends_with("general.austrip"));
    // An ID the database does not know is an error, not a fallback.
    assert!(load(&b, Some(3)).0.is_err());
}

#[test]
fn no_database_falls_back_with_warning() {
    let mut files = spp3_files(2);
    files.remove("DeviceDSPDatabase.plist");
    let dir = scratch("e2e-nodb");
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    let (t, d) = load(&Bundle::open(&dir).unwrap(), None);
    let t = t.unwrap();
    assert!(t.selection.configuration.is_none());
    assert!(d.items.iter().any(|x| x.code == "select-no-database"));
    // fs is unresolved without the substitution.
    assert!(d.items.iter().any(|x| x.code == "dspg-unresolved"));
}

#[test]
fn channel_count_mismatch_is_an_error() {
    let mut files = spp3_files(4);
    // A two-channel sidecar for a four-channel unit.
    let sc = format!("{AID}/DSP/Strips/{}-aufx-spp3-appl.plist", AID.to_lowercase());
    files.insert(sc, aurora_speaker_tuning::plist::binary::write(&sidecar_value(2)));
    let dir = scratch("e2e-count");
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    let (t, d) = load(&Bundle::open(&dir).unwrap(), None);
    assert!(t.is_ok());
    assert!(d.items.iter().any(|x| x.code == "spp3-channel-count"));
    assert!(d.items.iter().any(|x| x.code == "spp3-preset-channels"));
    assert!(d.has_errors());
}

#[test]
fn other_unit_kinds_expose_raw_state() {
    let mut files = spp3_files(4);
    for unit in ["lsm2", "atsp"] {
        files.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph_text(unit).into_bytes());
        files.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"), strip(unit, 4));
        let dir = scratch(&format!("e2e-{unit}"));
        write_dir(&dir, &files, &manifest_json(&files, "25G83"));
        let (t, d) = load(&Bundle::open(&dir).unwrap(), None);
        let t = t.unwrap();
        assert!(t.protection.spp3.is_none());
        assert_eq!(t.protection.state.elements(4).len(), 4);
        assert!(d.items.iter().any(|x| x.code == "params-raw-ids"));
    }
}

#[test]
fn units_table_covers_every_sidecar_key() {
    let t = aurora_speaker_tuning::units::table();
    for k in ["Reb_ref", "T_sett_vc", "VMONScale", "Thermal_GainAttackTime_s", "ag", "SpeakerGroup"] {
        assert!(t.iter().any(|r| r.field == k), "{k}");
    }
    assert!(t.iter().filter(|r| r.unit.contains("TODO(SPEC)")).count() > 0);
}
