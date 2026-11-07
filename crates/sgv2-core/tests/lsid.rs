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
    Io, ProtectionUnit, UnitKind,
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
    assert_eq!(e.partial_frames(), 3);
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
    changed.banks[1].rdc[3] = 99.0;
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
    changed.banks[0].rdc[3] = 1.0;
    assert!(u.stage_model(ModelGeneration::seeded(2, &changed).unwrap()).is_err());
    assert_eq!(u.model_generation(), Some(&before));
    u.set_runtime_budgets([100.0; 4]);
    assert!(u.companion().unwrap().gates != 0);
}
