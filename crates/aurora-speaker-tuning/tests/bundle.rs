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
    let t = Tuning::load(b, &Options { speaker_id: spid, ..Options::default() }, &mut d);
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

/// A bundle with a vendor-neutral and a spid-7 configuration, whose
/// manifest records `manifest_spid`.
fn spid_bundle(name: &str, manifest_spid: Option<u32>) -> Bundle {
    let mut files = spp3_files(2);
    let alt = files[&format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip")].clone();
    files.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general_7.austrip"), alt);
    files.insert(
        "DeviceDSPDatabase.plist".into(),
        database(&[(65535, "builtin_speaker_out_general.austrip"), (7, "builtin_speaker_out_general_7.austrip")]),
    );
    let dir = scratch(name);
    write_dir(&dir, &files, &manifest_json_with(&files, "25G83", manifest_spid));
    Bundle::open(&dir).unwrap()
}

fn load_with(b: &Bundle, opts: Options) -> Result<Tuning, Error> {
    Tuning::load(b, &opts, &mut Diagnostics::default())
}

#[test]
fn speaker_id_comes_from_the_manifest_then_the_device_tree() {
    // The installer's speaker ID selects the vendor strip.
    let b = spid_bundle("e2e-spid-manifest", Some(7));
    assert_eq!(b.manifest.speaker_id, Some(7));
    let t = load_with(&b, Options::default()).unwrap();
    assert!(t.selection.strip.ends_with("_7.austrip"));
    assert_eq!((t.speaker_id, t.speaker_id_source), (Some(7), Some("manifest")));
    // 65535 (invalid strap) takes the general route.
    let t = load_with(&spid_bundle("e2e-spid-65535", Some(65535)), Options::default()).unwrap();
    assert!(t.selection.strip.ends_with("general.austrip"));
    // An ID the database lacks is an error, not another vendor's strip.
    assert!(load_with(&spid_bundle("e2e-spid-3", Some(3)), Options::default()).is_err());
    // Without a manifest value, the device tree's export.
    let b = spid_bundle("e2e-spid-dt", None);
    let t = load_with(&b, Options { device_tree_speaker_id: Some(7), ..Options::default() }).unwrap();
    assert!(t.selection.strip.ends_with("_7.austrip"));
    assert_eq!(t.speaker_id_source, Some("device-tree"));
    // Neither: the vendor-neutral fallback, with a warning.
    let mut d = Diagnostics::default();
    let t = Tuning::load(&b, &Options::default(), &mut d).unwrap();
    assert!(t.selection.strip.ends_with("general.austrip"));
    assert!(d.items.iter().any(|x| x.code == "select-speaker-id-unknown"));
    assert_eq!(t.speaker_id, None);
    // The manifest and the device tree disagree: not this Mac's bundle.
    let b = spid_bundle("e2e-spid-clash", Some(7));
    assert!(load_with(&b, Options { device_tree_speaker_id: Some(3), ..Options::default() }).is_err());
    // An explicit override wins.
    let t = load_with(&b, Options { speaker_id: Some(65535), device_tree_speaker_id: Some(3) }).unwrap();
    assert_eq!(t.speaker_id_source, Some("override"));
}

#[test]
fn no_database_falls_back_with_warning() {
    let mut files = spp3_files(2);
    files.remove("DeviceDSPDatabase.plist");
    files.remove(&format!("Config/{AID}.plist"));
    let dir = scratch("e2e-nodb");
    // The installer never writes a set without its database (schema 2
    // refuses it); a development collector bundle can lack one.
    write_dir(&dir, &files, "{}");
    std::fs::remove_file(dir.join("manifest.json")).unwrap();
    std::fs::write(dir.join("metadata.json"), collector_json(&files)).unwrap();
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

/// The digest provisioning binds a blob to: the same for the same bundle
/// wherever and however it is stored, different for any change it states.
#[test]
fn digest_is_the_bundle_identity() {
    let files = spp3_files(2);
    let manifest = manifest_json(&files, "25G83");
    let a = scratch("digest-a");
    write_dir(&a, &files, &manifest);
    let b = scratch("digest-b");
    write_dir(&b, &files, &manifest);
    let da = Bundle::open(&a).unwrap().digest();
    assert_eq!(da.len(), 64);
    assert_eq!(da, Bundle::open(&b).unwrap().digest());
    // As an archive, under a top directory.
    let path = scratch("digest-tar").join("bundle.tar");
    {
        let mut ar = tar::Builder::new(std::fs::File::create(&path).unwrap());
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
        ar.into_inner().unwrap().flush().unwrap();
    }
    assert_eq!(da, Bundle::open(&path).unwrap().digest());
    // Another macOS build, speaker ID or file content is another bundle.
    let c = scratch("digest-build");
    write_dir(&c, &files, &manifest_json(&files, "25G84"));
    assert_ne!(da, Bundle::open(&c).unwrap().digest());
    let c = scratch("digest-spid");
    write_dir(&c, &files, &manifest_json_with(&files, "25G83", Some(7)));
    assert_ne!(da, Bundle::open(&c).unwrap().digest());
    let mut changed = files.clone();
    changed.get_mut(&format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg")).unwrap().push(b'\n');
    let c = scratch("digest-file");
    write_dir(&c, &changed, &manifest_json(&changed, "25G83"));
    assert_ne!(da, Bundle::open(&c).unwrap().digest());
}

/// `identify --json` on a bundle, and on a missing one.
#[test]
fn identify_cli() {
    let files = spp3_files(2);
    let dir = scratch("identify-cli");
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    let exe = env!("CARGO_BIN_EXE_aurora-speaker-tuning");
    let out = std::process::Command::new(exe).args(["identify", "--json"]).arg(&dir).output().unwrap();
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    let v: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(v["digest"].as_str().unwrap(), Bundle::open(&dir).unwrap().digest());
    assert_eq!(v["macos_build"], "25G83");
    assert_eq!(v["aid"], AID);
    assert_eq!(v["verified"], true);
    let out = std::process::Command::new(exe).args(["identify", "--json"]).arg(dir.join("absent")).output().unwrap();
    assert_eq!(out.status.code(), Some(1));
    let v: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert!(v["error"].as_str().is_some());
}
