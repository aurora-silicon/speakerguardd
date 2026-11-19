// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//! The LSID sense binding on the host side: the supervisor's word on the
//! engine page, and the plugin binding the unit's companion only while that
//! word is fresh and names the blob it runs. All values are synthetic.

use core::sync::atomic::Ordering;

use sgv2_core::{
    blob::{self, Pod},
    lsid::{self, Preset, Topology},
    lsm2::{
        self,
        companion::{self, Companion},
    },
    spp3, Io, ProtectionUnit, UnitKind,
};
use sgv2_lv2::page::{EnginePage, SENSE_BIND_STALE_NS, SENSE_BOUND, SENSE_NO_COMPANION, SENSE_UNBOUND};
use sgv2_lv2::{load_unit, lv2_descriptor, AnyUnit, Role, VARIANTS};

/// The synthetic preset and companion of sgv2-core's LSID tests: six
/// channels, the twelve-lane source "synthetic-sense", release-gated.
fn preset() -> Preset {
    let mut records: Vec<_> = lsid::schema_ids().into_iter().map(|id| (id, 0.0)).collect();
    for (id, v) in &mut records {
        if *id == 1024 {
            *v = 0.5;
        }
        for bank in 0..3 {
            let off = bank * 7168;
            let set = [
                (2048, 1.0),
                (2049, 1.0),
                (2054, 32.0),
                (3075, 3.0 + bank as f32),
                (3073, 1.0),
                (3074, 0.5),
                (3076, 1.0),
                (3077, 12.0),
                (5125, 1.0),
                (5126, 20_000.0),
                (5137, 1.0),
            ];
            for (base, value) in set {
                if *id == base + off {
                    *v = value;
                }
            }
        }
    }
    Preset::decode(&records).unwrap()
}

fn companion() -> Companion {
    let mut records: Vec<_> = companion::schema_ids().into_iter().map(|id| (id, 0.0)).collect();
    for (id, v) in &mut records {
        if *id == lsm2::ids::controller(9) {
            *v = 2.0;
        }
        if *id == lsm2::ids::controller(lsm2::ids::C_LINK_SLOW) {
            *v = 1.0;
        }
        for ch in 0..6 {
            if *id == lsm2::ids::CH_TUNING + ch {
                *v = lsid::BANK_ASSIGNMENT[ch as usize] as f32;
            }
            if *id == lsm2::ids::CH_SPEAKER_TYPE + ch {
                *v = lsid::FAMILY_ASSIGNMENT[ch as usize] as f32;
            }
            if *id == lsm2::ids::channel(ch, 30) {
                *v = lsid::SYSTEM_ASSIGNMENT[ch as usize] as f32;
            }
        }
    }
    Companion {
        gates: companion::UNQUALIFIED_FLOW,
        channels: 6,
        lsm2_records: records,
        lsid_records: preset().records(),
        topology: Some(Topology::new("synthetic-sense").unwrap()),
        flow: vec![0, 1, 2, 3],
        property_64004: 1,
        flow_roles: Companion::six_flow_roles(),
        systems: Companion::six_systems(),
    }
}

fn lsid_bundle() -> Vec<u8> {
    let params = lsm2::Params {
        channels: 6,
        sample_rate: 48000,
        enable_mitigation: 1,
        control_temperature: 1,
        ..Default::default()
    };
    let mut payload = params.as_bytes().to_vec();
    payload.extend_from_slice(&companion().encode().unwrap());
    blob::build(UnitKind::Lsm2, 6, 12, 48000, &payload)
}

/// Six-ohm resistive V/I that the synthetic preset accepts (lane 1 leads by
/// the graph's one-frame alignment), as twelve lanes.
fn resistive_sense(frames: usize) -> Vec<Vec<f32>> {
    let current: Vec<f32> = (0..frames)
        .map(|n| {
            let phase = 2.0 * core::f32::consts::PI * (n % 64) as f32 / 64.0;
            (3.0 * phase).sin() + 0.4 * (11.0 * phase).sin()
        })
        .collect();
    let voltage: Vec<f32> = current.iter().map(|x| x * 6.0).collect();
    let leading: Vec<f32> = (0..frames).map(|n| current[(n + 1) % 64]).collect();
    let mut lanes = vec![voltage.clone(), leading];
    for _ in 0..5 {
        lanes.push(voltage.clone());
        lanes.push(current.clone());
    }
    lanes
}

fn tmp(name: &str) -> String {
    let d = std::env::temp_dir().join(format!("sgv2-sense-bind-{}-{name}", std::process::id()));
    let _ = std::fs::create_dir_all(&d);
    d.join("engine").to_string_lossy().into_owned()
}

#[test]
fn the_word_binds_only_its_own_blob_while_fresh() {
    let path = tmp("word");
    let page = EnginePage::create(&path).unwrap();
    // a supervisor that never wrote it (or an older one) binds nothing
    assert!(!page.sense_bind(0x1234));
    page.set_sense_bind(Some(0x1234));
    assert!(page.sense_bind(0x1234));
    assert!(!page.sense_bind(0x4321), "another blob");
    assert!(!page.sense_bind(0), "no blob");
    page.set_sense_bind(None);
    assert!(!page.sense_bind(0x1234), "withdrawn");
    // a word the supervisor stopped refreshing
    page.set_sense_bind(Some(0x1234));
    let h = page.header();
    let at = h.sense_bind_ns.load(Ordering::Relaxed);
    h.sense_bind_ns.store(at.saturating_sub(SENSE_BIND_STALE_NS + 1).max(1), Ordering::Release);
    assert!(!page.sense_bind(0x1234), "stale");
    let _ = std::fs::remove_file(&path);
}

#[test]
fn only_an_lsid_companion_binds_and_its_gates_still_mute() {
    let (mut unit, err, crc) = load_unit(Some(&lsid_bundle()), 48_000.0, 6, 12, Role::Protect);
    assert!(err.is_none() && crc != 0 && matches!(unit, AnyUnit::Lsm2(_)));
    assert_eq!(unit.follow_sense_bind(false), SENSE_UNBOUND);
    assert_eq!(unit.follow_sense_bind(true), SENSE_BOUND);
    assert_eq!(unit.follow_sense_bind(true), SENSE_BOUND, "already bound");
    let AnyUnit::Lsm2(u) = &mut unit else { unreachable!() };
    assert!(u.sense_source_bound());
    // bound: the estimator takes the window, the release gate mutes
    let sense = resistive_sense(65);
    let lanes: Vec<&[f32]> = sense.iter().map(|l| l.as_slice()).collect();
    let input = [0.25f32; 65];
    let mut storage = [[9.0f32; 65]; 6];
    let mut out: Vec<_> = storage.iter_mut().map(|a| a.as_mut_slice()).collect();
    assert!(u.process(Io { frames: 65, input: &[&input[..]; 6], sense: &lanes, output: &mut out }).is_failsafe());
    assert_eq!(u.model_generation().unwrap().generation(), 2);
    assert!(storage.iter().flatten().all(|&x| x == 0.0), "the release gate still mutes");
    assert_ne!(u.companion().unwrap().gates, 0);
    assert_eq!(unit.follow_sense_bind(false), SENSE_UNBOUND);
    let AnyUnit::Lsm2(u) = &unit else { unreachable!() };
    assert!(!u.sense_source_bound());

    // lsm2 without a companion, and spp3: nothing to bind, nothing changes
    let mut unit = AnyUnit::Lsm2(Box::new(lsm2::Unit::new(lsm2::Params {
        channels: 6,
        sample_rate: 48000,
        ..Default::default()
    })));
    assert_eq!(unit.follow_sense_bind(true), SENSE_NO_COMPANION);
    let p = spp3::Params { channels: 2, sample_rate: 48000, ..Default::default() };
    let (mut unit, _, _) =
        load_unit(Some(&blob::build(UnitKind::Spp3, 2, 4, 48000, p.as_bytes())), 48_000.0, 2, 4, Role::Protect);
    assert_eq!(unit.follow_sense_bind(true), SENSE_NO_COMPANION);
    assert_eq!(unit.follow_sense_bind(false), SENSE_UNBOUND);
}

/// Through the C ABI: `run` binds while the attached page carries the
/// supervisor's fresh word for the blob it runs, and drops the binding when
/// the word is withdrawn or names another blob. The output stays muted.
#[test]
fn run_follows_the_supervisor_word_through_the_c_abi() {
    let dir = std::env::temp_dir().join(format!("sgv2-sense-bind-{}-abi", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let blob_path = dir.join("params.bin");
    std::fs::write(&blob_path, lsid_bundle()).unwrap();
    let page_path = dir.join("engine").to_string_lossy().into_owned();
    let page = EnginePage::create(&page_path).unwrap();
    std::env::set_var("SGV2_PARAMS", &blob_path);
    std::env::set_var("SGV2_ENGINE_PAGE", &page_path);
    let index = VARIANTS.iter().position(|v| v.1 == 6 && v.2 == 12 && v.3 == Role::Protect).unwrap();
    let d = lv2_descriptor(index as u32);
    const N: usize = 512;
    let sense = resistive_sense(N);
    let mut input = [[0.25f32; N]; 6];
    let mut output = [[9.0f32; N]; 6];
    let mut controls = [0.0f32; 3];
    let h = page.header();
    unsafe {
        let inst = ((*d).instantiate)(d, 48_000.0, core::ptr::null(), core::ptr::null());
        assert!(!inst.is_null());
        for c in 0..6 {
            ((*d).connect_port)(inst, c as u32, input[c].as_mut_ptr().cast());
            ((*d).connect_port)(inst, (6 + c) as u32, output[c].as_mut_ptr().cast());
        }
        for (k, lane) in sense.iter().enumerate() {
            ((*d).connect_port)(inst, (12 + k) as u32, lane.as_ptr().cast_mut().cast());
        }
        for (k, c) in controls.iter_mut().enumerate() {
            ((*d).connect_port)(inst, (24 + k) as u32, (c as *mut f32).cast());
        }
        ((*d).activate.unwrap())(inst);
        let crc = h.params_crc.load(Ordering::Relaxed);
        assert_ne!(crc, 0);
        let run = || {
            ((*d).run)(inst, N as u32);
            h.sense_bound.load(Ordering::Acquire)
        };
        assert_eq!(run(), SENSE_UNBOUND, "no word, no binding");
        page.set_sense_bind(Some(crc ^ 1));
        assert_eq!(run(), SENSE_UNBOUND, "a word for another blob");
        page.set_sense_bind(Some(crc));
        assert_eq!(run(), SENSE_BOUND);
        assert_eq!(run(), SENSE_BOUND);
        page.set_sense_bind(None);
        assert_eq!(run(), SENSE_UNBOUND, "withdrawn");
        page.set_sense_bind(Some(crc));
        assert_eq!(run(), SENSE_BOUND);
        // the supervisor stopped writing: the word goes stale
        let at = h.sense_bind_ns.load(Ordering::Relaxed);
        h.sense_bind_ns.store(at.saturating_sub(SENSE_BIND_STALE_NS + 1).max(1), Ordering::Release);
        assert_eq!(run(), SENSE_UNBOUND, "stale");
        assert!(output.iter().flatten().all(|&x| x == 0.0), "a release-gated unit stays muted");
        ((*d).cleanup)(inst);
    }
    let _ = std::fs::remove_dir_all(&dir);
}
