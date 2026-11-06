// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//! Command-level checks for missing-only native LSM2 selector supplements.
//! All values and identities are synthetic. See docs/LSM2-NATIVE-STATE.md.
mod common;
use aurora_speaker_tuning::manifest::sha256_hex;
use aurora_speaker_tuning::plist::{binary, Value};
use common::*;
use sgv2_core::lsm2::ids as id;
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    process::Command,
};

const SNAP: &str = "lsm2-native-state.plist";
fn key(c: u32) -> u32 {
    id::channel(c, id::CH_POWER_AMP_MODEL)
}
fn fingerprint(files: &BTreeMap<String, Vec<u8>>) -> String {
    let rows: Vec<_> = files.iter().map(|(p, b)| (p, b.len() as u64, sha256_hex(b))).collect();
    let mut bytes = b"aurora-lsm2-native-source-v1\0".to_vec();
    bytes.extend(serde_json::to_vec(&rows).unwrap());
    sha256_hex(&bytes)
}
fn set(value: &mut Value, name: &str, replacement: Value) {
    let Value::Dict(d) = value else { panic!("not a dictionary") };
    *d.get_mut(name).unwrap() = replacement;
}
fn identity(build: &str, board: &str, unit: &str) -> Value {
    dict(vec![("macos_build", s(build)), ("board", s(board)), ("unit_id", s(unit))])
}
fn fixture(root: &Path, name: &str) -> PathBuf {
    let missing = !matches!(name, "baseline" | "changed-present");
    let mut files = lsm2_files(|params| {
        if missing {
            for c in 0..8 {
                params.remove(&key(c));
            }
        }
        if name == "partial-present" {
            params.insert(key(0), 0.0);
        }
        if name == "unsupported-dsp" {
            params.insert(id::controller(id::C_COMPENSATE_RESONATORS), 1.0);
        }
    });
    if name == "wrong-protection" {
        files = spp3_files(4);
    }
    let mut canonical = lsm2_state(|params| {
        if name == "changed-flag" {
            params.insert(id::ENABLE_CLOSED_LOOP_TEMPERATURE, 1.0);
        }
        if name == "changed-present" {
            params.insert(key(0), 1.0);
        }
        if name == "fractional" {
            params.insert(key(0), 0.5);
        }
        if name == "missing-canonical" {
            params.remove(&key(0));
        }
        if name == "unsupported-dsp" {
            params.insert(id::controller(id::C_COMPENSATE_RESONATORS), 1.0);
        }
        params.insert(9_000_000, 123.0); // An unrelated native default must not be imported.
    });
    if name == "duplicate-parameter" {
        canonical.sections[0].params.push((key(0), 0.0));
    }
    let mut native = binary::write(&preset("lsm2", "native_fixture", &canonical));
    if name == "wrong-component" {
        native = binary::write(&preset("spp3", "native_fixture", &canonical));
    }
    let mut queries = Vec::new();
    for stage in ["before-init", "after-init", "after-render"] {
        for c in 0..4 {
            if name == "missing-query" && stage == "after-init" && c == 0 {
                continue;
            }
            queries.push(dict(vec![
                ("stage", s(stage)),
                ("id", i(key(if name == "inactive-query" && c == 0 { 4 } else { c }) as i128)),
                ("parameter_info_status", i(0)),
                ("get_parameter_status", i(if name == "failed-query" && c == 0 { -50 } else { 0 })),
                ("value", r(if name == "query-mismatch" && c == 0 { 2.0 } else { 0.0 })),
            ]));
        }
    }
    let mut snap = dict(vec![
        ("schema", i(1)),
        ("channels", i(4)),
        ("sample_rate", i(48000)),
        ("method", s("AudioUnitGetProperty-ClassInfo")),
        ("identity_before", identity("25G83", "j413", &test_unit())),
        ("identity_after", identity("25G83", "j413", &test_unit())),
        ("source_fingerprint", s(&fingerprint(&files))),
        ("runtime_overrides", Value::Array(vec![])),
        ("canonical_sha256", s(&sha256_hex(&native))),
        ("canonical_state", Value::Data(native)),
        ("parameter_queries", Value::Array(queries)),
    ]);
    match name {
        "wrong-build" => {
            for stage in ["identity_before", "identity_after"] {
                set(&mut snap, stage, identity("other", "j413", &test_unit()));
            }
        }
        "wrong-board" => {
            for stage in ["identity_before", "identity_after"] {
                set(&mut snap, stage, identity("25G83", "j999", &test_unit()));
            }
        }
        "wrong-unit" => {
            for stage in ["identity_before", "identity_after"] {
                set(&mut snap, stage, identity("25G83", "j413", &other_unit()));
            }
        }
        "identity-changed" => set(&mut snap, "identity_after", identity("25G83", "j413", &other_unit())),
        "wrong-source" => set(&mut snap, "source_fingerprint", s(&"0".repeat(64))),
        "wrong-canonical-hash" => set(&mut snap, "canonical_sha256", s(&"0".repeat(64))),
        "runtime-override" => set(&mut snap, "runtime_overrides", Value::Array(vec![s("test override")])),
        "wrong-channels" => set(&mut snap, "channels", i(2)),
        "wrong-rate" => set(&mut snap, "sample_rate", i(44100)),
        "empty-build" => {
            for stage in ["identity_before", "identity_after"] {
                set(&mut snap, stage, identity("  ", "j413", &test_unit()));
            }
        }
        _ => {}
    }
    if name == "changed-source-record" {
        files
            .get_mut(&format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"))
            .unwrap()
            .extend_from_slice(b"\n; changed source\n");
    }
    let manifest_text = manifest_json_ident(&files, "25G83", None, Some("j413"), Some(&test_unit()));
    let mut manifest: serde_json::Value = serde_json::from_str(&manifest_text).unwrap();
    if name == "empty-build" {
        manifest["macos_build"] = serde_json::json!("  ");
    }
    if name != "baseline" && name != "absent" {
        let bytes = binary::write(&snap);
        manifest["lsm2_native_state"] =
            serde_json::json!({"path":SNAP,"bytes":bytes.len(),"sha256":sha256_hex(&bytes)});
        files.insert(SNAP.into(), bytes);
    }
    if name == "record-hash" {
        manifest["lsm2_native_state"]["sha256"] = serde_json::json!("0".repeat(64));
    }
    let dir = root.join(name).join("bundle");
    std::fs::create_dir_all(&dir).unwrap();
    write_dir(&dir, &files, &serde_json::to_string(&manifest).unwrap());
    dir
}
#[test]
fn native_selector_cli_contract() {
    let root = std::env::var_os("AURORA_TUNING_E2E_OUT").map(PathBuf::from).unwrap_or_else(|| {
        Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!(
            "native-lsm2-e2e-{}-{}",
            std::process::id(),
            std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos()
        ))
    });
    std::fs::create_dir(&root).unwrap();
    eprintln!("native LSM2 CLI artifacts: {}", root.display());
    let mut receipts = Vec::new();
    let mut baseline = Vec::new();
    let cases = [
        "baseline",
        "absent",
        "valid",
        "partial-present",
        "changed-flag",
        "changed-present",
        "wrong-build",
        "wrong-board",
        "wrong-unit",
        "identity-changed",
        "wrong-source",
        "wrong-canonical-hash",
        "record-hash",
        "failed-query",
        "missing-query",
        "query-mismatch",
        "missing-canonical",
        "fractional",
        "duplicate-parameter",
        "wrong-component",
        "runtime-override",
        "unknown-runtime-unit",
        "foreign-runtime-unit",
        "unsupported-dsp",
        "wrong-protection",
        "changed-source-record",
        "wrong-channels",
        "wrong-rate",
        "empty-build",
        "inactive-query",
    ];
    let mut failures = Vec::new();
    for name in cases {
        let bundle = fixture(&root, name);
        let output_path = root.join(name).join("compiled.bin");
        let mut args = vec!["compile".to_owned(), "--board".into(), "j413".into()];
        if name != "unknown-runtime-unit" {
            args.extend(["--unit-id".into(), if name == "foreign-runtime-unit" { other_unit() } else { test_unit() }]);
        }
        args.extend(["--bundle".into(), bundle.display().to_string(), "-o".into(), output_path.display().to_string()]);
        let dir = root.join(name);
        let binary = env!("CARGO_BIN_EXE_aurora-speaker-tuning");
        let command: Vec<_> = std::iter::once(binary).chain(args.iter().map(String::as_str)).collect();
        std::fs::write(dir.join("command.json"), serde_json::to_vec_pretty(&command).unwrap()).unwrap();
        let result = match Command::new(binary).args(&args).output() {
            Ok(result) => result,
            Err(error) => {
                failures.push(format!("{name}: could not execute CLI: {error}"));
                receipts.push(serde_json::json!({"case":name,"execution_error":error.to_string()}));
                continue;
            }
        };
        std::fs::write(dir.join("stdout.txt"), &result.stdout).unwrap();
        std::fs::write(dir.join("stderr.txt"), &result.stderr).unwrap();
        let expect_ok = matches!(name, "baseline" | "valid" | "partial-present");
        let actual_ok = result.status.success();
        if actual_ok != expect_ok {
            failures.push(format!("{name}: exit {:?}", result.status.code()));
        }
        let bytes = std::fs::read(&output_path).ok();
        if name == "baseline" {
            baseline = bytes.clone().unwrap_or_default();
        }
        if expect_ok && name != "baseline" && bytes.as_deref() != Some(&baseline) {
            failures.push(format!("{name}: blob changed"));
        }
        if !expect_ok && output_path.exists() {
            failures.push(format!("{name}: refusal wrote a blob"));
        }
        if name == "unsupported-dsp" && !String::from_utf8_lossy(&result.stderr).contains("CompensateResonators") {
            failures.push("unsupported DSP flag did not remain refused".into());
        }
        if name == "valid" {
            let command = [binary, "dump", "--json", bundle.to_str().unwrap()];
            std::fs::write(dir.join("dump-command.json"), serde_json::to_vec_pretty(&command).unwrap()).unwrap();
            match Command::new(binary).args(&command[1..]).output() {
                Ok(dumped) => {
                    std::fs::write(dir.join("dump.json"), &dumped.stdout).unwrap();
                    std::fs::write(dir.join("dump-stderr.txt"), &dumped.stderr).unwrap();
                    let parsed = serde_json::from_slice::<serde_json::Value>(&dumped.stdout).ok();
                    let params = parsed
                        .as_ref()
                        .and_then(|j| j["tuning"]["protection"]["state"]["sections"][0]["params"].as_array());
                    if let Some(params) = params {
                        if params
                            .iter()
                            .any(|p| p[0].as_u64() == Some(9_000_000) || p[0].as_u64() == Some(key(4) as u64))
                        {
                            failures.push("inactive or unrelated canonical default was imported".into());
                        }
                    } else {
                        failures.push("CLI dump did not contain a parameter array".into());
                    }
                    if !dumped.status.success()
                        || parsed.as_ref().is_none_or(|j| j["tuning"]["native_lsm2_selectors"] != 4)
                    {
                        failures.push("failed dump or wrong native-selector count".into());
                    }
                }
                Err(error) => failures.push(format!("could not execute CLI dump: {error}")),
            }
        }
        receipts.push(serde_json::json!({"case":name,"expected_success":expect_ok,"exit":result.status.code(),
            "blob_sha256":bytes.as_deref().map(sha256_hex),"stdout_sha256":sha256_hex(&result.stdout),"stderr_sha256":sha256_hex(&result.stderr)}));
    }
    std::fs::write(root.join("RESULT.json"), serde_json::to_vec_pretty(&serde_json::json!({
        "scope":"synthetic CLI contract; no hardware qualification", "passed":failures.is_empty(), "failures":failures,"cases":receipts})).unwrap()).unwrap();
    assert!(failures.is_empty(), "{}", failures.join("; "));
}
