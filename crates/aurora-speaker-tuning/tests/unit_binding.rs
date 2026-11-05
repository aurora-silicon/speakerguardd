// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! A tuning bundle is bound to the Mac it was collected on: its board and
//! its unit (the unit ID of the root `serial-number`).  Synthetic bundles
//! and device trees only; every identity here is made up.

mod common;

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::compile;
use aurora_speaker_tuning::machine::{self, Machine, Unit};
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::Diagnostics;
use common::*;
use std::collections::BTreeMap;

fn load(files: &BTreeMap<String, Vec<u8>>, manifest: &str, name: &str) -> (Bundle, Tuning) {
    let dir = scratch(name);
    write_dir(&dir, files, manifest);
    let b = Bundle::open(&dir).unwrap();
    let mut d = Diagnostics::default();
    let t = Tuning::load(&b, &Options::default(), &mut d).unwrap();
    assert!(!d.has_errors(), "{:?}", d.items);
    (b, t)
}

fn bundle(board: Option<&str>, unit: Option<&str>, name: &str) -> Tuning {
    let f = spp3_files(4);
    load(&f, &manifest_json_ident(&f, "25G83", None, board, unit), name).1
}

/// Compiles; `(calibrated channels, warning codes)`.
fn run(t: &Tuning, m: &Machine) -> aurora_speaker_tuning::Result<(usize, Vec<&'static str>)> {
    let mut d = Diagnostics::default();
    let c = compile::compile(t, m, &mut d)?;
    let p = sgv2_core::spp3::Params::from_blob(&c.blob).unwrap();
    let cal = p.ch[..4].iter().filter(|c| c.cal_present == 1).count();
    assert_eq!(c.calibration, cal == 4);
    let warns = d.items.iter().filter(|x| x.severity == aurora_speaker_tuning::Severity::Warning).map(|x| x.code);
    Ok((cal, warns.collect()))
}

fn this_mac() -> Machine {
    Machine::with_unit("j413", &test_unit()).unwrap()
}

#[test]
fn the_collecting_unit_uses_its_calibration() {
    let t = bundle(Some("j413"), Some(&test_unit()), "ub-match");
    let (cal, warns) = run(&t, &this_mac()).unwrap();
    assert_eq!(cal, 4);
    assert!(!warns.contains(&"unit-unbound") && !warns.contains(&"unit-unverifiable"), "{warns:?}");
}

#[test]
fn another_unit_is_refused() {
    // The I1 case: another unit of the same model, whose SpTS would pass
    // every other check.  No blob: no grant, the speakers stay muted.
    let t = bundle(Some("j413"), Some(&other_unit()), "ub-other");
    let e = run(&t, &this_mac()).unwrap_err().to_string();
    assert!(e.contains("collected on another Mac"), "{e}");
    assert!(e.contains("collect the speaker tuning again"), "{e}");
    // The refusal names no identity.
    assert!(!e.contains(&other_unit()) && !e.contains(&test_unit()), "{e}");
    // Without a board in the manifest too.
    let t = bundle(None, Some(&other_unit()), "ub-other-noboard");
    assert!(run(&t, &this_mac()).is_err());
}

#[test]
fn another_board_is_refused() {
    let t = bundle(Some("j616s"), Some(&test_unit()), "ub-board");
    let e = run(&t, &this_mac()).unwrap_err().to_string();
    assert!(e.contains("collected on a j616s, this Mac is a j413"), "{e}");
}

#[test]
fn a_bundle_without_identity_runs_open_loop() {
    // Collected before the installer recorded the unit: the per-unit
    // calibration is not trusted, every channel holds its base limit.
    for (board, name) in [(None, "ub-legacy"), (Some("j413"), "ub-legacy-board")] {
        let t = bundle(board, None, name);
        let (cal, warns) = run(&t, &this_mac()).unwrap();
        assert_eq!(cal, 0, "{name}");
        assert!(warns.contains(&"unit-unbound"), "{warns:?}");
    }
    // Everything else in the blob is the same as the bound bundle's: only
    // the calibration differs.
    let bound = bundle(None, Some(&test_unit()), "ub-cmp-bound");
    let legacy = bundle(None, None, "ub-cmp-legacy");
    let pb = sgv2_core::spp3::Params::from_blob(
        &compile::compile(&bound, &this_mac(), &mut Diagnostics::default()).unwrap().blob,
    )
    .unwrap();
    let pl = sgv2_core::spp3::Params::from_blob(
        &compile::compile(&legacy, &this_mac(), &mut Diagnostics::default()).unwrap().blob,
    )
    .unwrap();
    for c in 0..4 {
        assert_eq!(pb.ch[c].temperature_limit, pl.ch[c].temperature_limit);
        assert_eq!((pl.ch[c].cal_present, pl.ch[c].cal_r, pl.ch[c].cal_t), (0, 0.0, 0.0));
    }
}

#[test]
fn an_unknown_running_unit_runs_open_loop() {
    // A bound bundle, but this Mac's unit is not known (no or a malformed
    // device-tree serial-number, or an offline compile without --unit-id).
    let t = bundle(Some("j413"), Some(&test_unit()), "ub-unknown");
    for unit in [Unit::Unknown, Unit::Malformed] {
        let m = Machine { unit: unit.clone(), ..Machine::new("j413").unwrap() };
        let (cal, warns) = run(&t, &m).unwrap();
        assert_eq!(cal, 0, "{unit:?}");
        assert!(warns.contains(&"unit-unverifiable"), "{warns:?}");
    }
}

#[test]
fn no_calibration_no_warning() {
    // A Mac without SpTS (desktops, the lsm2 and atsp units) has nothing
    // per unit, so a bundle without identity compiles as before, quietly.
    let mut f = spp3_files(4);
    f.remove("speaker-thiele-small.bin");
    let (_, t) = load(&f, &manifest_json_ident(&f, "25G83", None, None, None), "ub-nocal");
    let (cal, warns) = run(&t, &this_mac()).unwrap();
    assert_eq!(cal, 0);
    assert!(!warns.contains(&"unit-unbound"), "{warns:?}");
}

#[test]
fn the_running_unit_comes_from_the_device_tree() {
    // A fake device tree, as aura writes it: one NUL after the serial.
    let dt = scratch("ub-dt");
    std::fs::write(dt.join("compatible"), b"apple,j413\0apple,t8112\0apple,arm-platform\0").unwrap();
    std::fs::write(dt.join(machine::SERIAL_NUMBER), b"SYNTHETIC0A\0").unwrap();
    let m = Machine::from_device_tree(&dt).unwrap();
    assert_eq!(m.unit, Unit::Known(test_unit()));
    let bound = bundle(Some("j413"), Some(&test_unit()), "ub-dt-bound");
    assert_eq!(run(&bound, &m).unwrap().0, 4);
    // The same tree on another unit refuses the bundle.
    std::fs::write(dt.join(machine::SERIAL_NUMBER), b"SYNTHETIC0B\0").unwrap();
    assert!(run(&bound, &Machine::from_device_tree(&dt).unwrap()).is_err());
    // No serial-number: open loop, never a match.
    std::fs::remove_file(dt.join(machine::SERIAL_NUMBER)).unwrap();
    assert_eq!(run(&bound, &Machine::from_device_tree(&dt).unwrap()).unwrap().0, 0);
}

#[test]
fn the_identity_is_part_of_the_digest() {
    let f = spp3_files(4);
    let digest = |board: Option<&str>, unit: Option<&str>, name: &str| {
        load(&f, &manifest_json_ident(&f, "25G83", None, board, unit), name).0.digest()
    };
    let legacy = digest(None, None, "ud-legacy");
    let a = digest(None, Some(&test_unit()), "ud-a");
    let b = digest(None, Some(&other_unit()), "ud-b");
    let ab = digest(Some("j413"), Some(&test_unit()), "ud-ab");
    assert!(legacy != a && a != b && a != ab && legacy != ab);
}
