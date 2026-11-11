// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//! All values are synthetic. No solver or native-component parity is claimed.
use sgv2_core::{
    blob::{self, Pod},
    lsid::{self, Error, Estimator, Handoff, ModelGeneration, Preset, State, Topology},
    lsm2::{
        self,
        companion::{self, Companion},
    },
    Io, ProtectionUnit, UnitKind, MAX_CHANNELS,
};

fn preset() -> Preset {
    let mut records: Vec<_> = lsid::schema_ids().into_iter().map(|id| (id, 0.0)).collect();
    for (id, v) in &mut records {
        if *id == 1024 {
            *v = 0.5;
        }
        for bank in 0..3 {
            let off = bank * 7168;
            if *id == 2048 + off || *id == 2049 + off {
                *v = 1.0;
            }
            if *id == 2054 + off {
                *v = 32.0;
            }
            if *id == 3075 + off {
                *v = 3.0 + bank as f32;
            }
            if *id == 3073 + off {
                *v = 1.0;
            }
            if *id == 3074 + off {
                *v = 0.5;
            }
            if *id == 3076 + off {
                *v = 1.0;
            }
            if *id == 3077 + off {
                *v = 12.0;
            }
            if *id == 5125 + off {
                *v = 1.0;
            }
            if *id == 5126 + off {
                *v = 20_000.0;
            }
            if *id == 5137 + off {
                *v = 1.0;
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
        calibration_present: 0,
        calibration_temperature: [0.0; MAX_CHANNELS],
    }
}
fn bundle() -> Vec<u8> {
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
#[test]
fn exact_id_geometry_and_unknown_float_bits_round_trip() {
    let original = preset();
    assert_eq!(original.records().len(), 603);
    let mut records = original.records();
    // Opaque auxiliary field: signed zero must survive storage.
    records.iter_mut().find(|r| r.0 == 23559).unwrap().1 = -0.0;
    records.iter_mut().find(|r| r.0 == 26624).unwrap().1 = -1.0;
    records.reverse();
    let parsed = Preset::decode(&records).unwrap();
    assert_eq!(parsed.selectors[0], -1.0);
    assert_eq!(parsed.banks[0].auxiliary[7].to_bits(), (-0.0f32).to_bits());
    let reparsed = Preset::decode(&parsed.records()).unwrap();
    assert_eq!(reparsed.banks[0].auxiliary[7].to_bits(), (-0.0f32).to_bits());
    for bad in [0, 1, 602, 604] {
        let mut r = original.records();
        r.resize(bad, (42, 1.0));
        assert!(Preset::decode(&r).is_err());
    }
    let mut r = original.records();
    r[3].0 = 999;
    assert_eq!(Preset::decode(&r), Err(Error::Schema));
    let mut r = original.records();
    r[3].0 = r[2].0;
    assert_eq!(Preset::decode(&r), Err(Error::Schema));
    let mut r = original.records();
    r[3].1 = f32::NAN;
    assert_eq!(Preset::decode(&r), Err(Error::NonFinite));
}
#[test]
fn extended_schema_is_append_only_and_id_strict() {
    assert_eq!(companion::base_ids().len(), 1021);
    assert_eq!(companion::schema_ids().len(), 1326);
    let mut c = companion();
    c.lsm2_records.reverse();
    c.lsm2_records.iter_mut().find(|r| r.0 == 3124).unwrap().1 = -0.0;
    let decoded = Companion::decode(&c.encode().unwrap()).unwrap();
    assert_eq!(decoded.lsm2_records.iter().find(|r| r.0 == 3124).unwrap().1.to_bits(), (-0.0f32).to_bits());
    for mutation in 0..4 {
        let mut c = c.clone();
        match mutation {
            0 => {
                c.lsm2_records.pop();
            }
            1 => {
                c.lsm2_records[0].0 = 123456;
            }
            2 => {
                c.lsm2_records[0].1 = f32::INFINITY;
            }
            _ => {
                c.lsm2_records.push((123456, 0.0));
            }
        }
        assert!(c.encode().is_err());
    }
    let encoded = c.encode().unwrap();
    for n in 0..encoded.len() {
        assert!(Companion::decode(&encoded[..n]).is_err());
    }
}
#[test]
fn six_independent_axes_and_two_flow_side_roles() {
    assert_eq!(lsid::PERMUTATION.map(|i| [10, 11, 12, 20, 21, 22][i]), [10, 20, 11, 21, 12, 22]);
    assert_eq!(lsid::BANK_ASSIGNMENT, [0, 1, 0, 1, 2, 2]);
    assert_eq!(lsid::FAMILY_ASSIGNMENT, [0, 0, 0, 0, 1, 1]);
    assert_eq!(lsid::SYSTEM_ASSIGNMENT, [1, 1, 1, 1, 2, 2]);
    let c = companion();
    assert_eq!(c.flow_roles, [[0, 0], [1, 0], [0, 0], [1, 0], [0, 1], [1, 1]]);
    assert_eq!(c.systems, [1, 1, 1, 1, 2, 2, 0, 0]);
    for channel in 0..8 {
        let mut bad = c.clone();
        bad.systems[channel] = 7;
        assert!(bad.encode().is_err());
    }
    for id in [lsm2::ids::CH_TUNING + 2, lsm2::ids::CH_SPEAKER_TYPE + 5, lsm2::ids::controller(9)] {
        let mut bad = c.clone();
        bad.lsm2_records.iter_mut().find(|r| r.0 == id).unwrap().1 = 7.0;
        assert!(bad.encode().is_err());
    }
}
#[test]
fn invalid_dead_mismatched_sensing_and_stale_state_keep_static_models() {
    let p = preset();
    let mut e = Estimator::new(p, Topology::new("synthetic-sense").unwrap(), 48000).unwrap();
    assert_eq!(e.state(), State::Seeded);
    assert!(!e.adaptation_enabled());
    assert_eq!(e.enable_adaptation(), Err(Error::Unsupported));
    let original = e.model().clone();
    let one = [1.0; 16];
    let zero = [0.0; 16];
    let nan = [f32::NAN; 16];
    let good = [&one[..]; 12];
    e.observe("synthetic-sense", &good, 1); // initialize the one-frame lane delay
    e.observe("synthetic-sense", &good, 3);
    assert!(e.sensing_valid());
    assert_eq!(e.partial_frames(), 4);
    e.reset();
    assert_eq!(e.state(), State::Reset);
    assert_eq!(e.partial_frames(), 0);
    for source in ["wrong-source", "synthetic-sense"] {
        for bad in [&zero[..], &nan[..], &one[..0]] {
            let mut lanes = good;
            lanes[7] = bad;
            e.observe(source, &lanes, 16);
            assert!(!e.sensing_valid());
            assert_eq!(e.model(), &original);
            assert_eq!(e.measured_power(), None);
        }
    }
    e.observe("synthetic-sense", &[], 48000);
    e.mark_stale();
    assert_eq!(e.state(), State::Decaying);
    assert_eq!(e.model(), &original);
    assert!(e.banks().iter().all(|b| b.partial_frames == 0 && !b.valid && b.fast_rdc == b.slow_rdc));
}
#[test]
fn immutable_complete_generations_commit_only_at_boundaries() {
    let p = preset();
    let seed = ModelGeneration::seeded(1, &p).unwrap();
    let mut h = Handoff::new(seed.clone());
    h.stage(ModelGeneration::seeded(2, &p).unwrap()).unwrap();
    assert_eq!(h.current(), &seed);
    assert!(h.stage(seed).is_err());
    h.commit();
    assert_eq!(h.current().generation(), 2);
    let mut changed = p.clone();
    changed.banks[1].rdc[3] = 9.0;
    assert!(h.stage(ModelGeneration::seeded(3, &changed).unwrap()).is_err());
    assert_eq!(h.current().banks()[1].fast_rdc, 4.0);
    let mut unit = lsm2::Unit::from_blob(&bundle()).unwrap();
    let input = [1.0; 256];
    let mut storage = [[9.0; 256]; 6];
    let mut render = |u: &mut lsm2::Unit, n| {
        let mut out: Vec<_> = storage.iter_mut().map(|a| a.as_mut_slice()).collect();
        assert!(u.process(Io { frames: n, input: &[&input[..]; 6], sense: &[], output: &mut out }).is_failsafe());
    };
    render(&mut unit, 1);
    unit.stage_model(ModelGeneration::seeded(2, &p).unwrap()).unwrap();
    render(&mut unit, 254);
    assert_eq!(unit.model_generation().unwrap().generation(), 1);
    render(&mut unit, 1);
    assert_eq!(unit.model_generation().unwrap().generation(), 2);
    unit.stage_model(ModelGeneration::seeded(3, &p).unwrap()).unwrap();
    unit.reset();
    render(&mut unit, 256);
    assert_eq!(unit.model_generation().unwrap().generation(), 2);
}
#[test]
fn single_lsm2_governor_cannot_release_unqualified_models() {
    let blob = bundle();
    // Extracting Params alone cannot lose the mandatory release gate.
    assert!(lsm2::Params::from_blob(&blob).is_err());
    let mut u = lsm2::Unit::from_blob(&blob).unwrap();
    assert_eq!(u.kind(), UnitKind::Lsm2);
    assert!(u.estimator().is_some());
    assert!(!u.estimator().unwrap().adaptation_enabled());
    for sample in [0.0, 0.25, 2.0, f32::NAN, f32::INFINITY] {
        let a = [sample; 257];
        let sense = [f32::NAN; 257];
        let mut storage = [[9.0; 257]; 6];
        let mut out: Vec<_> = storage.iter_mut().map(|a| a.as_mut_slice()).collect();
        let health = u.process(Io { frames: 257, input: &[&a[..]; 6], sense: &[&sense[..]; 12], output: &mut out });
        assert!(health.is_failsafe());
        assert!(storage.iter().flatten().all(|&x| x == 0.0));
        assert!(!u.estimator().unwrap().sensing_valid());
    }
    let before = u.model_generation().unwrap().clone();
    let mut changed = preset();
    changed.banks[0].rdc[3] = 2.0;
    assert!(u.stage_model(ModelGeneration::seeded(2, &changed).unwrap()).is_err());
    assert_eq!(u.model_generation(), Some(&before));
    u.set_runtime_budgets([100.0; 4]);
    assert!(u.companion().unwrap().gates != 0);
}

#[test]
fn synthetic_resistive_windows_update_fast_slow_rdc_and_power() {
    let mut p = preset();
    for bank in &mut p.banks {
        bank.impedance[3] = 0.0;
        bank.rdc[0] = 0.0;
        bank.rdc[1] = 1.0;
        bank.rdc[2] = 0.95;
        bank.rdc[4] = 1.0;
        bank.rdc[5] = 12.0;
        bank.rdc[9] = 0.0;
        bank.rdc[10] = 0.0;
        bank.rdc[13] = 0.0;
        bank.lr2[5] = 1.0;
        bank.lr2[6] = 20_000.0;
        bank.lr2[17] = 1.0;
        bank.lr2[21] = 0.0;
    }
    let mut e = Estimator::new(p, Topology::new("synthetic-sense").unwrap(), 48_000).unwrap();
    let mut current = [0.0f32; 65];
    for (n, x) in current.iter_mut().enumerate() {
        let phase = 2.0 * core::f32::consts::PI * n as f32 / 64.0;
        *x = libm::sinf(3.0 * phase) + 0.4 * libm::sinf(11.0 * phase);
    }
    let mut voltage = current;
    voltage.iter_mut().for_each(|x| *x *= 6.0);
    // The graph delays lane 1 by one frame before fitting. Its raw current
    // must therefore lead voltage by one frame for this resistive plant.
    let leading_current: [f32; 65] = core::array::from_fn(|n| current[(n + 1) % 64]);
    let sense = [
        &voltage[..],
        &leading_current[..],
        &voltage[..],
        &current[..],
        &voltage[..],
        &current[..],
        &voltage[..],
        &current[..],
        &voltage[..],
        &current[..],
        &voltage[..],
        &current[..],
    ];
    e.observe("synthetic-sense", &sense, 65);
    assert_eq!(e.state(), State::Tracking);
    assert_eq!(e.model().generation(), 2);
    assert_eq!(e.model().accepted_bank_mask(), 0b111);
    for (index, bank) in e.model().banks().iter().enumerate() {
        assert!((bank.fast_rdc - 6.0).abs() < 0.05, "bank {index}: {}", bank.fast_rdc);
        assert!((bank.slow_rdc - 6.0).abs() < 0.05, "bank {index}: {}", bank.slow_rdc);
        assert!(bank.confidence > 0.9, "bank {index}: {}", bank.confidence);
    }
    assert!(e.measured_power().is_some_and(|p| p > 0.0));
    assert!(e.model().measured_power_w().is_some_and(|p| p > 0.0));
    assert!(!e.adaptation_enabled(), "release gate remains closed");
    let accepted = e.model().clone();
    e.observe("synthetic-sense", &[], 24_001);
    assert_eq!(e.state(), State::Decaying);
    assert_eq!(e.model(), &accepted, "stale private decay is never published");
    for (index, bank) in e.banks().iter().enumerate() {
        assert_eq!(bank.fast_rdc, 3.0 + index as f32);
        assert!(!bank.valid);
    }

    let mut unit = lsm2::Unit::from_blob(&bundle()).unwrap();
    unit.observe_estimator("synthetic-sense", &sense, 65);
    assert_eq!(unit.model_generation().unwrap().generation(), 1, "staged only");
    let input = [0.0; 256];
    let mut storage = [[9.0; 256]; 6];
    let mut out: Vec<_> = storage.iter_mut().map(|a| a.as_mut_slice()).collect();
    assert!(unit.process(Io { frames: 256, input: &[&input[..]; 6], sense: &[], output: &mut out }).is_failsafe());
    assert_eq!(unit.model_generation().unwrap().generation(), 2);
    assert!(storage.iter().flatten().all(|&x| x == 0.0));
}
