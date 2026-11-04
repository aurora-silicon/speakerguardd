// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The blob compiler, on synthetic bundles: every payload field traced to
//! its tuning source, decoded back with the engine's own decoder.

mod common;

use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::compile::{compile, lsm2_ids as id};
use aurora_speaker_tuning::tuning::{Options, Tuning, UnitKind};
use aurora_speaker_tuning::Diagnostics;
use common::*;
use std::collections::BTreeMap;

fn tuning(files: &BTreeMap<String, Vec<u8>>, name: &str) -> Tuning {
    tuning_allowing(files, name, &[])
}

fn tuning_allowing(files: &BTreeMap<String, Vec<u8>>, name: &str, _expected: &[&str]) -> Tuning {
    let dir = scratch(name);
    write_dir(&dir, files, &manifest_json(files, "25G83"));
    let b = Bundle::open(&dir).unwrap();
    let mut d = Diagnostics::default();
    let t = Tuning::load(&b, &Options { speaker_id: None }, &mut d).unwrap();
    assert!(!d.has_errors(), "{:?}", d.items);
    t
}

#[test]
fn spp3_fields_come_from_the_sidecar_and_state() {
    let t = tuning(&spp3_files(4), "c-spp3");
    let mut d = Diagnostics::default();
    let c = compile(&t, &mut d).unwrap();
    assert!(!d.has_errors(), "{:?}", d.items);
    assert_eq!((c.unit, c.channels, c.sense_channels, c.sample_rate), (UnitKind::Spp3, 4, 8, 48000));
    let p = sgv2_core::spp3::Params::from_blob(&c.blob).unwrap();
    assert_eq!(p.block_frames, 256);
    assert_eq!(p.thermal_ambient_temp, 41.0);
    assert_eq!(p.thermal_gain_attack_time_s, 3.5);
    assert_eq!(p.thermal_lookahead_delay_ms, 1.5);
    assert_eq!(p.thermal_protection_enabled, 1);
    for ch in 0..4 {
        let x = &p.ch[ch];
        assert_eq!(x.source, ch as u32);
        assert_eq!(x.group, (ch / 2) as u32);
        assert_eq!((x.sense_i, x.sense_v), (2 * ch as u32 + 1, 2 * ch as u32));
        assert_eq!((x.t_sett_vc, x.t_sett_mg, x.tau_tvc, x.tau_tmg), (17.0, 19.0, 2.5, 90.0));
        assert_eq!((x.reb_ref, x.rshunt, x.ag), (3.3, 0.05, 9.5));
        assert_eq!((x.temperature_limit, x.hard_temp_limit_headroom), (121.0, 7.0));
        assert_eq!((x.imon_scale, x.vmon_scale), (2.0, 10.0));
        assert_eq!(x.cl_thermal_feedback_enabled, 1);
        assert_eq!((x.initial_theta_vc, x.initial_theta_mg), (0.0, 0.0));
        // spA<ch> carries raw R 3000+ch (Q6.10) and raw T 3100 (Q9.7).
        assert_eq!(x.cal_present, 1);
        assert_eq!((x.cal_r, x.cal_t), ((3000 + ch) as f32 / 1024.0, 3100.0 / 128.0));
        assert_eq!((x.temp_sense_window_time_s, x.tlim_adj_cap, x.a_t_20c), (0.2, 40.0, 0.0039));
    }
}

#[test]
fn spp3_calibration_needs_a_valid_matching_record() {
    // A bad checksum: every channel open loop.
    let mut f = spp3_files(4);
    let spts = f.get_mut("speaker-thiele-small.bin").unwrap();
    let n = spts.len();
    spts[n - 1] ^= 1;
    let t = tuning_allowing(&f, "c-cal-bad", &["spts-checksum"]);
    let p = sgv2_core::spp3::Params::from_blob(&compile(&t, &mut Diagnostics::default()).unwrap().blob).unwrap();
    assert!(p.ch[..4].iter().all(|c| c.cal_present == 0));
    // A record only for channel 2: the others open loop.
    let mut f = spp3_files(4);
    let id = aurora_speaker_tuning::fourcc::FourCc::from_str_exact("spA2").unwrap();
    f.insert("speaker-thiele-small.bin".into(), aurora_speaker_tuning::spts::encode(&[(id, 3500, 3000)], 0));
    let t = tuning_allowing(&f, "c-cal-one", &["spts-count"]);
    let p = sgv2_core::spp3::Params::from_blob(&compile(&t, &mut Diagnostics::default()).unwrap().blob).unwrap();
    let present: Vec<u32> = p.ch[..4].iter().map(|c| c.cal_present).collect();
    assert_eq!(present, vec![0, 0, 1, 0]);
}

#[test]
fn spp3_start_state_must_be_the_specified_one() {
    for state in [None, Some(0), Some(1)] {
        let mut f = spp3_files(4);
        let sc = format!("{}-aufx-spp3-appl.plist", AID.to_lowercase());
        f.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general.propstrip"), propstrip_with(&sc, state));
        let t = tuning(&f, &format!("c-start-{state:?}"));
        assert!(compile(&t, &mut Diagnostics::default()).is_err(), "{state:?}");
    }
}

/// A synthetic lsm2 saved state: one active type-A model, two channels.
fn lsm2_state(tweak: impl Fn(&mut BTreeMap<u32, f32>)) -> ParamState {
    let mut m: BTreeMap<u32, f32> = BTreeMap::new();
    m.insert(id::ENABLE_CLOSED_LOOP_TEMPERATURE, 0.0);
    m.insert(id::BYPASS_SYS_ID, 1.0);
    for c in 0..8 {
        m.insert(id::CH_TUNING + c, 0.0);
        m.insert(id::CH_SPEAKER_TYPE + c, 0.0);
    }
    for mgr in 0..5 {
        for ty in 0..3 {
            for k in 0..37 {
                m.insert(id::model(mgr, ty, k), 0.0);
            }
            let set = |m: &mut BTreeMap<u32, f32>, k, v| {
                m.insert(id::model(mgr, ty, k), v);
            };
            set(&mut m, id::M_RDC, 5.0 + mgr as f32);
            set(&mut m, id::M_ENABLE_RES1, 1.0);
            set(&mut m, id::M_BL, 0.9);
            set(&mut m, id::M_MMS_G, 0.3);
            set(&mut m, id::M_RMS, 0.2);
            set(&mut m, id::M_KMS, 1500.0 + 10.0 * mgr as f32);
        }
    }
    for ty in 0..3 {
        for (k, v) in [(id::T_RTH_VC, 30.0), (id::T_TAU_VC, 3.0), (id::T_RTH_MG, 20.0), (id::T_TAU_MG, 80.0)] {
            m.insert(id::thermal(ty, k), v);
        }
        m.insert(id::thermal(ty, id::T_AMBIENT), 45.0);
        m.insert(id::thermal(ty, id::T_ENABLE), 1.0);
    }
    for k in 0..23 {
        m.insert(id::controller(k), 1.0);
    }
    for (k, v) in [
        (id::C_ATTACK, 4.0),
        (id::C_FAST_RELEASE, 150.0),
        (id::C_SLOW_RELEASE, 15000.0),
        (id::C_FILTERBANK_LATENCY, 2.0),
        (id::C_LOOKAHEAD, 0.5),
        (id::C_UNLINK_TEMPERATURE, 0.0),
        (id::C_COMPENSATE_RESONATORS, 0.0),
        (id::C_USE_FAST_PEAK, 0.0),
    ] {
        m.insert(id::controller(k), v);
    }
    for k in 0..37 {
        m.insert(id::amp0(k), 0.0);
    }
    for j in 0..4 {
        m.insert(id::amp0(id::A_TIME_SCALE + j), [0.01, 0.2, 2.0, 30.0][j as usize]);
        m.insert(id::amp0(id::A_HARD_LIMIT + j), [50.0, 8.0, 3.0, 2.0][j as usize]);
        m.insert(id::amp0(id::A_SOFT_LIMIT + j), 100.0);
    }
    m.insert(id::amp0(id::A_LOW_IDLE), 0.02);
    m.insert(id::amp0(id::A_LOW_RATIO), 1.2);
    m.insert(id::amp0(id::A_MAX_ATTENUATION), 4.0);
    for c in 0..8 {
        for k in 0..33 {
            m.insert(id::channel(c, k), 1.0);
        }
        m.insert(id::channel(c, id::CH_VOLTS_FS), 6.0 + c as f32);
        m.insert(id::channel(c, id::CH_PARASITIC), 0.2);
        m.insert(id::channel(c, id::CH_DISPLACEMENT_LIMIT), 0.0005);
        m.insert(id::channel(c, id::CH_TEMP_HARD), 130.0);
        m.insert(id::channel(c, id::CH_TEMP_SOFT), 115.0);
        m.insert(id::channel(c, id::CH_VELOCITY_LIMIT0), 0.5);
        m.insert(id::channel(c, id::CH_MAX_VELOCITY_ATTEN), 18.0);
        m.insert(id::channel(c, id::CH_MAX_TEMP_ATTEN), 10.0);
        m.insert(id::channel(c, id::CH_TEMP_HEADROOM), 4.0);
        m.insert(id::channel(c, id::CH_POWER_AMP_MODEL), 0.0);
    }
    tweak(&mut m);
    ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: m.into_iter().collect() }] }
}

fn lsm2_files(tweak: impl Fn(&mut BTreeMap<u32, f32>)) -> BTreeMap<String, Vec<u8>> {
    let mut f = spp3_files(4);
    f.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph_text("lsm2").into_bytes());
    f.insert(
        format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"),
        strip_with_state("lsm2", &lsm2_state(tweak)),
    );
    f
}

#[test]
fn lsm2_fields_follow_the_id_rule() {
    let t = tuning(&lsm2_files(|_| {}), "c-lsm2");
    let c = compile(&t, &mut Diagnostics::default()).unwrap();
    let p = sgv2_core::lsm2::Params::from_blob(&c.blob).unwrap();
    assert_eq!((p.channels, p.sample_rate, p.model_only), (4, 48000, 1));
    assert_eq!((p.attack_time_ms, p.fast_release_time_ms, p.slow_release_time_ms), (4.0, 150.0, 15000.0));
    assert_eq!((p.filterbank_latency_ms, p.speaker_response_lookahead_ms), (2.0, 0.5));
    assert_eq!(p.power_hard_limit_w, [50.0, 8.0, 3.0, 2.0]);
    assert_eq!((p.low_rail_idle_power_w, p.low_rail_power_ratio, p.other_power_features), (0.02, 1.2, 0));
    let x = &p.ch[1];
    assert_eq!((x.source, x.volts_full_scale, x.parasitic_resistance), (1, 7.0, 0.2));
    // Admittance is manager 1, displacement manager 2 (distinct synthetic Rdc/Kms).
    assert_eq!((x.rdc, x.kms, x.disp_kms), (6.0, 1510.0, 1520.0));
    assert!((x.mms_kg - 0.0003).abs() < 1e-9);
    assert_eq!((x.rth_vc, x.tau_mag, x.ambient), (30.0, 80.0, 45.0));
    assert_eq!((x.temperature_soft_limit, x.temperature_hard_limit, x.temperature_headroom), (115.0, 130.0, 4.0));
}

#[test]
fn lsm2_refuses_what_the_payload_cannot_carry() {
    for (name, k, v) in [
        ("res2", id::model(id::MANAGER_ADMITTANCE, 0, id::M_ENABLE_RES2), 1.0),
        ("lr2", id::model(id::MANAGER_DISPLACEMENT, 0, id::M_ENABLE_LR2), 1.0),
        ("fastpeak", id::controller(id::C_USE_FAST_PEAK), 1.0),
        ("tuning", id::CH_TUNING + 1, 1.0),
        ("type", id::CH_SPEAKER_TYPE, 3.0),
    ] {
        let t = tuning(
            &lsm2_files(|m| {
                m.insert(k, v);
            }),
            &format!("c-lsm2-{name}"),
        );
        assert!(compile(&t, &mut Diagnostics::default()).is_err(), "{name}");
    }
}

#[test]
fn atsp_slots_from_the_id_map() {
    let mut params: Vec<(u32, f32)> = vec![(0, 0.0), (40, 1.0)];
    for s in 0..3u32 {
        let b = 1 + 13 * s;
        let mask = if s == 0 { 15.0 } else { 0.0 };
        let vals = [10.0, 8.5, 35.0, 2.0, 45.0, 70.0, 40.0, 125.0, 700.0, 18000.0, 9.0, 1.0, mask];
        for (k, v) in vals.iter().enumerate() {
            params.push((b + k as u32, *v));
        }
    }
    let mut f = spp3_files(4);
    f.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph_text("atsp").into_bytes());
    let st = ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] };
    f.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"), strip_with_state("atsp", &st));
    let t = tuning(&f, "c-atsp");
    let c = compile(&t, &mut Diagnostics::default()).unwrap();
    let p = sgv2_core::atsp::Params::from_blob(&c.blob).unwrap();
    assert_eq!((p.channels, p.block_frames, p.bypass, p.gain_link_all), (4, 256, 0, 1));
    let s = &p.slot[0];
    assert_eq!((s.channel_mask, s.amplifier_sensitivity, s.rdc, s.rth_vc, s.tau_mg), (15, 10.0, 8.5, 35.0, 70.0));
    assert_eq!((s.temperature_limit, s.attack_ms, s.release_ms, s.hard_limit_headroom), (125.0, 700.0, 18000.0, 9.0));
    assert_eq!(p.slot[1].channel_mask, 0);
}
