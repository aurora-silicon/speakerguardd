// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Offline only: exercise the production startup module, inject all host
//! dependencies, and open only generated synthetic files. Never invoke the
//! daemon or use the live device tree, kernel controls, engine pages or ALSA.
#[path = "../../../aurora-speaker-tuning/tests/common/mod.rs"]
mod common;
use crate::startup;

use super::Environment;
use aurora_speaker_tuning::{
    aupreset::{ParamSection, ParamState},
    bundle::Bundle,
    compile::{self, PreStageMode, Provenance},
    machine::{self, Machine},
    manifest::sha256_hex,
    plist::{binary, Value},
    tuning::{Options, Tuning},
    Diagnostics,
};
use common::*;
use sgv2_supervisor::selfcheck::{self, Bind};
use std::path::{Path, PathBuf};

struct Host {
    root: PathBuf,
    bind: Bind,
    events: Vec<&'static str>,
    published: Option<Vec<u8>>,
    unverified: bool,
    kernel_fails: bool,
    publish_fails: bool,
    bind_fails: bool,
}

impl Environment for Host {
    fn identity(&mut self) -> Result<(Machine, Option<u32>), String> {
        self.events.push("identity");
        let dt = self.root.join("dt");
        Ok((
            Machine::from_device_tree(&dt).map_err(|e| e.to_string())?,
            machine::speaker_id(&dt).map_err(|e| e.to_string())?,
        ))
    }
    fn bundle(&mut self, path: Option<&Path>) -> aurora_speaker_tuning::Result<Bundle> {
        self.events.push(if path.is_some() { "explicit" } else { "discover" });
        // A controlled discovery result, never Bundle::discover() on the host.
        let mut bundle = Bundle::open(path.unwrap_or(&self.root.join("bundle")))?;
        if self.unverified {
            bundle.verified = false;
        }
        Ok(bundle)
    }
    fn selfcheck(&mut self) -> Result<Bind, String> {
        self.events.push("selfcheck");
        if self.bind_fails {
            Err("synthetic missing self-check record".into())
        } else {
            Ok(self.bind.clone())
        }
    }
    fn kernel_release(&mut self) -> Result<String, String> {
        self.events.push("kernel");
        if self.kernel_fails {
            Err("synthetic uname failure".into())
        } else {
            Ok("synthetic-kernel".into())
        }
    }
    fn publish(&mut self, blob: &[u8]) -> Result<(), String> {
        self.events.push("publish");
        if self.publish_fails {
            return Err("synthetic publication failure".into());
        }
        startup::publish_atomic(&self.root.join("run"), &self.root.join("run/user"), blob)
            .map_err(|e| e.to_string())?;
        self.published = Some(blob.to_vec());
        Ok(())
    }
}

fn files(unit: &str) -> std::collections::BTreeMap<String, Vec<u8>> {
    let mut files = spp3_files(4);
    let graph = format!("in in\nin sense\nout out\nbox Split split 1 2\nbox Join join 4 1\nbox Prot (aufx {unit}) 2 1\nwire in Split ([fs] 2)\nwire (Split 0) (Join 0) ([fs] 1)\nwire (Split 1) (Join 1) ([fs] 1)\nwire (Split 0) (Join 2) ([fs] 1)\nwire (Split 1) (Join 3) ([fs] 1)\nwire Join (Prot 0) ([fs] 4 256)\nwire sense (Prot 1) ([fs] 8i 256)\nwire Prot out ([fs] 4)\n");
    files.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph.into_bytes());
    let state = if unit == "atsp" {
        let mut params = vec![(0, 0.0), (40, 1.0)];
        for slot in 0..3 {
            for (k, v) in [
                11.89,
                8.5,
                35.0,
                2.0,
                45.0,
                70.0,
                40.0,
                125.0,
                700.0,
                18000.0,
                9.0,
                1.0,
                if slot == 0 { 15.0 } else { 0.0 },
            ]
            .into_iter()
            .enumerate()
            {
                params.push((1 + 13 * slot + k as u32, v));
            }
        }
        ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] }
    } else if unit == "lsm2" {
        lsm2_state(|_| {})
    } else {
        spp3_state(4)
    };
    files.insert(
        format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"),
        binary::write(&dict(vec![(
            "strips",
            Value::Array(vec![dict(vec![(
                "effects",
                Value::Array(vec![effect("Prot", unit, preset(unit, "synthetic protection", &state))]),
            )])]),
        )])),
    );
    if unit != "spp3" {
        files.insert(
            format!("{AID}/DSP/Strips/builtin_speaker_out_general.propstrip"),
            binary::write(&dict(vec![("Boxes", Value::Array(vec![]))])),
        );
    }
    files
}

impl Host {
    fn new(name: &str, unit: &str, voicing: bool) -> Self {
        let root = scratch(&format!("supervisor-startup-{name}"));
        std::fs::create_dir_all(root.join("dt/chosen")).unwrap();
        std::fs::write(root.join("dt/compatible"), b"apple,j413\0apple,synthetic\0").unwrap();
        std::fs::write(root.join("dt/serial-number"), b"SYNTHETIC0A\0").unwrap();
        let files = files(unit);
        write_dir(
            &root.join("bundle"),
            &files,
            &manifest_json_ident(&files, "25G83", None, Some("j413"), Some(&test_unit())),
        );
        let mut host = Host {
            root, bind: selfcheck::parse(&format!("schema=1\nresult=pass\nboard=j413\nkernel=synthetic-kernel\nparams_sha256={}\nwiring=0123456789abcdef\ncard=SyntheticCard\namps=4\nsense=8\nfamily=sn012776\nkey=synthetic\nmethod=derived\n", "0".repeat(64))).unwrap(),
            events: vec![], published: None, unverified: false, kernel_fails: false, publish_fails: false, bind_fails: false,
        };
        host.bind.params_sha256 = sha256_hex(&host.reference(voicing));
        host
    }

    // The exact existing compile CLI calls, independently of startup::start.
    fn reference(&self, voicing: bool) -> Vec<u8> {
        let bundle = Bundle::open(&self.root.join("bundle")).unwrap();
        let dt = self.root.join("dt");
        let tuning = Tuning::load(
            &bundle,
            &Options { speaker_id: None, device_tree_speaker_id: machine::speaker_id(&dt).unwrap() },
            &mut Diagnostics::default(),
        )
        .unwrap();
        let machine = Machine::from_device_tree(&dt).unwrap();
        let mut diags = Diagnostics::default();
        if voicing {
            compile::compile_voicing_with(&tuning, &machine, Provenance::Verified, &mut diags)
        } else {
            compile::compile_opts(&tuning, &machine, Provenance::Verified, PreStageMode::default(), &mut diags)
        }
        .unwrap()
        .blob
    }

    fn refused(&mut self, path: Option<&Path>, voicing: bool, why: &str) {
        let result = startup::start(self, path, voicing);
        let error = result.err().expect("must refuse before publication");
        let message = match error {
            startup::Error::Config(m) | startup::Error::Unavailable(m) => m,
        };
        assert!(message.contains(why), "expected {why:?}, got {message:?}");
        assert!(!self.events.contains(&"publish"), "refusal published parameters");
        assert!(self.published.is_none());
        assert!(!self.root.join("run/params.bin").exists());
        println!("REFUSED {why}: no publication; no device implementation available");
    }

    fn manifest(&mut self, edit: impl FnOnce(&mut serde_json::Value)) {
        let path = self.root.join("bundle/manifest.json");
        let mut json = serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        edit(&mut json);
        std::fs::write(path, serde_json::to_vec(&json).unwrap()).unwrap();
    }
}

#[test]
fn production_startup_matches_existing_compiler_for_protection_and_voicing() {
    for unit in ["spp3", "atsp", "lsm2"] {
        for voicing in [false, true] {
            for explicit in [false, true] {
                let mut host = Host::new(&format!("equivalence-{unit}-{voicing}-{explicit}"), unit, voicing);
                let reference = host.reference(voicing);
                // Leave only generated data in Cargo's own scratch for an
                // independent existing-CLI byte comparison.
                std::fs::write(host.root.join("synthetic-unit-id"), test_unit()).unwrap();
                let path = explicit.then(|| host.root.join("bundle"));
                let ready = startup::start(&mut host, path.as_deref(), voicing).unwrap();
                assert_eq!(host.published.as_deref(), Some(reference.as_slice()));
                assert_eq!(sha256_hex(host.published.as_ref().unwrap()), host.bind.params_sha256);
                assert_eq!(
                    host.events,
                    ["identity", "selfcheck", if explicit { "explicit" } else { "discover" }, "kernel", "publish"]
                );
                assert_eq!(ready.board, "j413");
                assert_eq!(ready.machine.amps, 4);
                assert_eq!(ready.bind, host.bind);
                assert_eq!(ready.header.channels, 4);
                assert_eq!(ready.full_scales, [11890; 4]);
                assert_ne!(ready.engine_crc, 0);
                assert_eq!(ready.crossover, voicing);
                assert_eq!(sgv2_core::blob::voicing(&reference).unwrap().is_some(), voicing);
                println!(
                    "BYTE_EQ {unit} voicing={voicing} explicit={explicit}: {} bytes SHA256 {}",
                    reference.len(),
                    sha256_hex(&reference)
                );
            }
        }
    }
}

#[test]
fn speaker_id_uses_manifest_or_device_tree_and_refuses_conflict() {
    for manifest in [false, true] {
        let mut host = Host::new(&format!("speaker-match-{manifest}"), "spp3", false);
        std::fs::write(host.root.join("dt/chosen/asahi,speaker-id"), 3u32.to_be_bytes()).unwrap();
        if manifest {
            host.manifest(|m| m["speaker_id"] = 3.into());
        }
        host.bind.params_sha256 = sha256_hex(&host.reference(false));
        startup::start(&mut host, None, false).unwrap();
    }
    let mut host = Host::new("speaker-conflict", "spp3", false);
    host.manifest(|m| m["speaker_id"] = 2.into());
    std::fs::write(host.root.join("dt/chosen/asahi,speaker-id"), 3u32.to_be_bytes()).unwrap();
    host.refused(None, false, "speaker");
}

#[test]
fn malformed_speaker_id_and_unknown_board_refuse() {
    let mut host = Host::new("speaker-malformed", "spp3", false);
    std::fs::write(host.root.join("dt/chosen/asahi,speaker-id"), [3, 0]).unwrap();
    host.refused(None, false, "not 4");
    std::fs::remove_file(host.root.join("dt/chosen/asahi,speaker-id")).unwrap();
    std::fs::write(host.root.join("dt/compatible"), b"vendor,unknown\0").unwrap();
    host.refused(None, false, "not apple");
}

#[test]
fn missing_bundle_does_not_fall_back_to_a_stale_blob() {
    for explicit in [false, true] {
        let mut host = Host::new(&format!("missing-{explicit}"), "spp3", false);
        std::fs::write(host.root.join("params.bin"), host.reference(false)).unwrap();
        std::fs::remove_dir_all(host.root.join("bundle")).unwrap();
        let path = explicit.then(|| host.root.join("bundle"));
        host.refused(path.as_deref(), false, "No such file");
    }
}

#[test]
fn invalid_manifest_and_corrupt_digest_refuse() {
    let mut host = Host::new("invalid-manifest", "spp3", false);
    std::fs::write(host.root.join("bundle/manifest.json"), b"{}").unwrap();
    host.refused(None, false, "manifest");
    let mut host = Host::new("bad-digest", "spp3", false);
    host.manifest(|m| m["files"][0]["sha256"] = "0".repeat(64).into());
    host.refused(None, false, "sha256");
}

#[test]
fn unverified_bundle_is_refused_even_if_its_bytes_match_the_selfcheck() {
    let mut host = Host::new("unverified", "spp3", false);
    host.unverified = true;
    host.refused(None, false, "not verified");
}

#[test]
fn wrong_board_and_wrong_unit_refuse() {
    let mut host = Host::new("wrong-board", "spp3", false);
    host.manifest(|m| m["board"] = "j999".into());
    host.refused(None, false, "collected on a j999");
    let mut host = Host::new("wrong-unit", "spp3", false);
    host.manifest(|m| m["unit_id"] = other_unit().into());
    host.refused(None, false, "another Mac");
}

#[test]
fn unsupported_mechanism_and_invalid_parameters_refuse() {
    let mut host = Host::new("unsupported", "spp3", false);
    let mut files = files("spp3");
    let sc = format!("{AID}/DSP/Strips/{}-aufx-spp3-appl.plist", AID.to_lowercase());
    // Unknown required source-order mechanism: the established compiler must
    // refuse it, never silently drop it to get startup through.
    files.insert(
        format!("{AID}/DSP/Strips/builtin_speaker_out_general.propstrip"),
        propstrip_extra(
            &format!("{}-aufx-spp3-appl.plist", AID.to_lowercase()),
            Some(2),
            vec![dict(vec![("Number", i(64005)), ("Data", Value::Data(vec![0, 1, 2, 3]))])],
        ),
    );
    write_dir(&host.root.join("bundle"), &files, &manifest_json(&files, "25G83"));
    host.refused(None, false, "64005");
    let mut host = Host::new("bad-parameters", "spp3", false);
    files.insert(sc, binary::write(&dict(vec![])));
    write_dir(&host.root.join("bundle"), &files, &manifest_json(&files, "25G83"));
    host.refused(None, false, "required field Version");
}

#[test]
fn selfcheck_kernel_hash_board_result_and_geometry_must_match() {
    for (case, why) in [(0, "kernel"), (1, "parameters"), (2, "self-check ran on"), (3, "failed"), (4, "channels")] {
        let mut host = Host::new(&format!("binding-{case}"), "spp3", false);
        match case {
            0 => host.bind.kernel = "other-kernel".into(),
            1 => host.bind.params_sha256 = "0".repeat(64),
            2 => host.bind.board = "j999".into(),
            3 => host.bind.result = "fail".into(),
            _ => {
                host.bind.amps = 2;
                host.bind.sense = 4;
            }
        }
        host.refused(None, false, why);
    }
}

#[test]
fn voicing_mode_change_needs_its_own_exact_selfcheck() {
    let mut host = Host::new("voicing-hash", "spp3", false);
    host.refused(None, true, "parameters");
}

#[test]
fn kernel_or_publication_failure_never_returns_prepared() {
    let mut host = Host::new("kernel-failure", "spp3", false);
    host.kernel_fails = true;
    host.refused(None, false, "uname failure");
    let mut host = Host::new("publication-failure", "spp3", false);
    host.publish_fails = true;
    assert!(matches!(startup::start(&mut host, None, false), Err(startup::Error::Unavailable(_))));
    assert!(host.published.is_none());
}

#[test]
fn cli_has_no_legacy_or_identity_or_development_bypass() {
    let parse = |suffix: &str| {
        startup::Options::parse(format!("--pcm 1 --uid 1000 {suffix}").split_whitespace().map(str::to_owned))
    };
    let normal = parse("").unwrap();
    assert_eq!((normal.pcm, normal.uid), (1, 1000));
    assert_eq!(normal.bundle, None);
    assert!(!normal.voicing_pre);
    let full =
        parse("--bundle /synthetic/source.tar --voicing-pre --selfcheck /synthetic/bind --card SyntheticCard").unwrap();
    assert!(full.voicing_pre);
    assert_eq!(full.bundle.as_deref(), Some(Path::new("/synthetic/source.tar")));
    for option in [
        "--params stale.bin",
        "--development",
        "--allow-unknown",
        "--board j413",
        "--unit-id x",
        "--speaker-id 2",
        "--root /synthetic",
        "--prestage tuning",
        "--bundle",
        "--uid 123",
        "--voicing-pre --voicing-pre",
    ] {
        assert!(parse(option).is_err(), "accepted {option}");
    }
    assert!(startup::USAGE.contains("--bundle"));
    assert!(!startup::USAGE.contains("--params"));
}

#[test]
fn missing_selfcheck_and_invalid_machine_description_refuse() {
    let mut host = Host::new("missing-selfcheck", "spp3", false);
    host.bind_fails = true;
    host.refused(None, false, "missing self-check");
    let mut host = Host::new("unsupported-amp", "spp3", false);
    host.bind.family = "synthetic-unsupported".into();
    host.refused(None, false, "amp family");
}

#[test]
fn explicit_bad_source_never_falls_back_to_discovery() {
    let mut host = Host::new("no-source-fallback", "spp3", false);
    let bad = host.root.join("bad-source");
    std::fs::create_dir(&bad).unwrap();
    std::fs::write(bad.join("manifest.json"), b"{}").unwrap();
    host.refused(Some(&bad), false, "manifest");
    assert_eq!(host.events, ["identity", "selfcheck", "explicit"]);
}

#[test]
fn unknown_unit_preserves_compiler_open_loop_and_exact_hash_binding() {
    for malformed in [false, true] {
        let mut host = Host::new(&format!("unknown-unit-{malformed}"), "spp3", false);
        let trusted_hash = host.bind.params_sha256.clone();
        if malformed {
            std::fs::write(host.root.join("dt/serial-number"), b"INVALID SERIAL").unwrap();
        } else {
            std::fs::remove_file(host.root.join("dt/serial-number")).unwrap();
        }
        let reference = host.reference(false);
        let params = sgv2_core::spp3::Params::from_blob(&reference).unwrap();
        assert!(params.ch[..4].iter().all(|c| c.cal_present == 0));
        assert_ne!(sha256_hex(&reference), trusted_hash);
        host.refused(None, false, "parameters");
        host.bind.params_sha256 = sha256_hex(&reference);
        startup::start(&mut host, None, false).unwrap();
        assert_eq!(host.published.as_deref(), Some(reference.as_slice()));
    }
}

#[test]
fn speaker_vendor_selection_requires_a_known_supported_id_when_no_neutral_exists() {
    for (name, spid, succeeds) in [
        ("manifest", Some(2), true),
        ("device-tree", None, true),
        ("unknown", None, false),
        ("unsupported", Some(9), false),
    ] {
        let mut host = Host::new(&format!("vendor-{name}"), "spp3", false);
        if name == "device-tree" {
            std::fs::write(host.root.join("dt/chosen/asahi,speaker-id"), 3u32.to_be_bytes()).unwrap();
        }
        let mut files = files("spp3");
        files.insert(
            "DeviceDSPDatabase.plist".into(),
            database(&[(2, "builtin_speaker_out_general.austrip"), (3, "builtin_speaker_out_general.austrip")]),
        );
        write_dir(&host.root.join("bundle"), &files, &manifest_json_with(&files, "25G83", spid));
        if succeeds {
            host.bind.params_sha256 = sha256_hex(&host.reference(false));
            startup::start(&mut host, None, false).unwrap();
        } else {
            host.refused(None, false, "configuration");
        }
    }
}

#[test]
fn publication_replaces_the_complete_blob_and_leaves_no_temporary_file() {
    let mut host = Host::new("atomic-publication", "spp3", false);
    std::fs::create_dir(host.root.join("run")).unwrap();
    std::fs::write(host.root.join("run/params.bin"), b"stale").unwrap();
    let expected = host.reference(false);
    startup::start(&mut host, None, false).unwrap();
    assert_eq!(std::fs::read(host.root.join("run/params.bin")).unwrap(), expected);
    assert!(!host.root.join("run/.params.bin.new").exists());
}
