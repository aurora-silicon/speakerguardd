// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The blob compiler, on synthetic bundles: every payload field traced to
//! its tuning source, decoded back with the engine's own decoder.

mod common;

use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::compile::{self as comp, lsm2_ids as id, Provenance};
use aurora_speaker_tuning::machine::Machine;
use aurora_speaker_tuning::plist::Value;
use aurora_speaker_tuning::tuning::{Options, Tuning, UnitKind};
use aurora_speaker_tuning::Diagnostics;
use common::*;
use std::collections::BTreeMap;

/// The synthetic bundles have four channels: a four-amp board.
fn j413() -> Machine {
    Machine::new("j413").unwrap()
}

fn compile(t: &Tuning, d: &mut Diagnostics) -> aurora_speaker_tuning::Result<comp::Compiled> {
    comp::compile(t, &j413(), d)
}

fn tuning(files: &BTreeMap<String, Vec<u8>>, name: &str) -> Tuning {
    tuning_allowing(files, name, &[])
}

fn tuning_allowing(files: &BTreeMap<String, Vec<u8>>, name: &str, _expected: &[&str]) -> Tuning {
    let dir = scratch(name);
    write_dir(&dir, files, &manifest_json(files, "25G83"));
    let b = Bundle::open(&dir).unwrap();
    let mut d = Diagnostics::default();
    let t = Tuning::load(&b, &Options::default(), &mut d).unwrap();
    assert!(!d.has_errors(), "{:?}", d.items);
    t
}

/// Loads a bundle that may carry errors (the compiler must refuse it).
fn tuning_any(files: &BTreeMap<String, Vec<u8>>, name: &str) -> Tuning {
    let dir = scratch(name);
    write_dir(&dir, files, &manifest_json(files, "25G83"));
    let b = Bundle::open(&dir).unwrap();
    Tuning::load(&b, &Options::default(), &mut Diagnostics::default()).unwrap()
}

fn sidecar_name() -> String {
    format!("{}-aufx-spp3-appl.plist", AID.to_lowercase())
}

fn propstrip_path() -> String {
    format!("{AID}/DSP/Strips/builtin_speaker_out_general.propstrip")
}

#[test]
fn spp3_fields_come_from_the_sidecar_and_state() {
    let t = tuning(&spp3_files(4), "c-spp3");
    let mut d = Diagnostics::default();
    let c = compile(&t, &mut d).unwrap();
    assert!(!d.has_errors(), "{:?}", d.items);
    assert_eq!((c.unit, c.channels, c.sense_channels, c.sample_rate), (UnitKind::Spp3, 4, 8, 48000));
    // The header names the board and the machine's operating point.
    let (h, _) = sgv2_core::blob::Header::parse(&c.blob).unwrap();
    assert_eq!((h.machine(), h.full_scale_mv), (Some("j413"), 8414));
    assert_eq!((c.board.as_str(), c.full_scale_mv), ("j413", 8414));
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
fn spp3_start_state_follows_64007_per_aid() {
    // 2: cold.  Unset (AID8/9/14/15), 0, 1 and other values: the hot
    // fallback, the steady-state split of TemperatureLimit over ambient.
    // Fixture: ambient 41, limit 121, T_sett 17/19 K/W: P0 = 80/36 W.
    let p0 = 80.0f64 / 36.0;
    for (state, hot) in [(Some(2), false), (None, true), (Some(0), true), (Some(1), true), (Some(7), true)] {
        let mut f = spp3_files(4);
        f.insert(propstrip_path(), propstrip_with(&sidecar_name(), state));
        let t = tuning(&f, &format!("c-start-{state:?}"));
        let c = compile(&t, &mut Diagnostics::default()).unwrap();
        let p = sgv2_core::spp3::Params::from_blob(&c.blob).unwrap();
        for x in &p.ch[..4] {
            let (vc, mg) = if hot { ((p0 * 17.0) as f32, (p0 * 19.0) as f32) } else { (0.0, 0.0) };
            assert_eq!((x.initial_theta_vc, x.initial_theta_mg), (vc, mg), "{state:?}");
            // T_OL starts at the limit when hot.
            if hot {
                assert!((41.0 + x.initial_theta_vc + x.initial_theta_mg - 121.0).abs() < 1e-4);
            }
        }
    }
}

#[test]
fn spp3_hot_seed_matches_the_spec_family_values() {
    // SPEC-SPP3 section 10: AID9-family woofer, ambient 50, limit 140,
    // Rvc/Rmg 26.06/25.55 K/W: 45.444681 K coil, 44.555319 K magnet.
    let p0 = (140.0f64 - 50.0) / (26.06 + 25.55);
    assert!((p0 * 26.06 - 45.444681).abs() < 1e-5);
    assert!((p0 * 25.55 - 44.555319).abs() < 1e-5);
}

#[test]
fn spp3_source_order_binds_sense_and_calibration() {
    // 64005 reverses the four sources: channel c takes spA<3-c>.
    let order = Value::Array(["spA3", "spA2", "spA1", "spA0"].iter().map(|x| s(x)).collect());
    let mut f = spp3_files(4);
    f.insert(
        propstrip_path(),
        propstrip_extra(&sidecar_name(), Some(2), vec![dict(vec![("Number", i(64005)), ("Array", order)])]),
    );
    let t = tuning(&f, "c-order");
    let p = sgv2_core::spp3::Params::from_blob(&compile(&t, &mut Diagnostics::default()).unwrap().blob).unwrap();
    for c in 0..4u32 {
        let k = 3 - c;
        let x = &p.ch[c as usize];
        assert_eq!((x.sense_i, x.sense_v), (2 * k + 1, 2 * k), "channel {c}");
        assert_eq!(x.cal_r, (3000 + k) as f32 / 1024.0, "channel {c}");
    }
    // Integer FourCCs are the same order.
    let ints = Value::Array(
        ["spA3", "spA2", "spA1", "spA0"]
            .iter()
            .map(|x| i(u32::from_be_bytes(x.as_bytes().try_into().unwrap()).into()))
            .collect(),
    );
    let mut f = spp3_files(4);
    f.insert(
        propstrip_path(),
        propstrip_extra(&sidecar_name(), Some(2), vec![dict(vec![("Number", i(64005)), ("Array", ints)])]),
    );
    let q = sgv2_core::spp3::Params::from_blob(
        &compile(&tuning(&f, "c-order-int"), &mut Diagnostics::default()).unwrap().blob,
    )
    .unwrap();
    assert_eq!(q.ch[0].sense_v, 6);
    // Malformed orders are refused, never guessed.
    for (name, v) in [
        ("short", Value::Array(vec![s("spA0"), s("spA1")])),
        ("dup", Value::Array(vec![s("spA0"), s("spA0"), s("spA1"), s("spA2")])),
        ("range", Value::Array(vec![s("spA0"), s("spA1"), s("spA2"), s("spA9")])),
        ("name", Value::Array(vec![s("spA0"), s("spA1"), s("spA2"), s("left")])),
        ("scalar", i(3)),
    ] {
        let key = if matches!(v, Value::Integer(_)) { "UInt32" } else { "Array" };
        let mut f = spp3_files(4);
        f.insert(
            propstrip_path(),
            propstrip_extra(&sidecar_name(), Some(2), vec![dict(vec![("Number", i(64005)), (key, v)])]),
        );
        let t = tuning(&f, &format!("c-order-{name}"));
        assert!(compile(&t, &mut Diagnostics::default()).is_err(), "{name}");
    }
}

#[test]
fn spp3_records_must_belong_to_channels() {
    // A record for a source no channel takes (spA7 on a four-channel unit).
    let mut f = spp3_files(4);
    let recs: Vec<_> = [0u32, 1, 2, 7]
        .iter()
        .map(|k| (aurora_speaker_tuning::fourcc::FourCc::from_str_exact(&format!("spA{k}")).unwrap(), 3000, 3100))
        .collect();
    f.insert("speaker-thiele-small.bin".into(), aurora_speaker_tuning::spts::encode(&recs, 0));
    let t = tuning_allowing(&f, "c-cal-stray", &[]);
    assert!(compile(&t, &mut Diagnostics::default()).is_err());
}

#[test]
fn compile_refuses_what_the_engine_must_not_run() {
    // A channel-count mismatch (SPEC-SPP3: a hard 20 dB fail-safe) loads
    // with an error finding; the library refuses to compile it.
    let mut f = spp3_files(4);
    f.insert(
        format!("{AID}/DSP/Strips/{}", sidecar_name()),
        aurora_speaker_tuning::plist::binary::write(&sidecar_value(2)),
    );
    let t = tuning_any(&f, "c-refuse-count");
    assert!(!t.refusals.is_empty());
    let e = compile(&t, &mut Diagnostics::default()).unwrap_err().to_string();
    assert!(e.contains("must not run"), "{e}");
    // A bypassed protection unit.
    let mut t = tuning(&spp3_files(4), "c-refuse-bypass");
    t.refusals.push(aurora_speaker_tuning::Diagnostic {
        severity: aurora_speaker_tuning::Severity::Error,
        code: "protection-bypassed",
        context: "strip".into(),
        message: "the strip bypasses Prot".into(),
    });
    assert!(compile(&t, &mut Diagnostics::default()).is_err());
    // An unverified bundle, unless development is asked for.
    let mut t = tuning(&spp3_files(4), "c-refuse-unverified");
    t.verified = false;
    assert!(compile(&t, &mut Diagnostics::default()).is_err());
    assert!(comp::compile_with(&t, &j413(), Provenance::AllowUnverified, &mut Diagnostics::default()).is_ok());
}

#[test]
fn compile_binds_to_the_machine() {
    let t = tuning(&spp3_files(4), "c-machine");
    // Another board's amp count, an unknown board, a board too long.
    assert!(comp::compile(&t, &Machine::new("j616s").unwrap(), &mut Diagnostics::default()).is_err());
    assert!(comp::compile(&t, &Machine::new("j274").unwrap(), &mut Diagnostics::default()).is_err());
    assert!(Machine::new("j616s-too-long").is_err());
    // A known acoustic-id must match the bundle's.
    let mut m = j413();
    m.acoustic_id = Some(ACOUSTIC_ID + 1);
    assert!(comp::compile(&t, &m, &mut Diagnostics::default()).is_err());
    m.acoustic_id = Some(ACOUSTIC_ID);
    assert!(comp::compile(&t, &m, &mut Diagnostics::default()).is_ok());
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
        // 2 * 160 + 32 = 352 frames at 48 kHz (the engine refuses < 256)
        (id::C_FILTERBANK_LATENCY, 10.0 / 3.0),
        (id::C_LOOKAHEAD, 2.0 / 3.0),
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
        m.insert(id::channel(c, id::CH_VOLTS_FS), 8.414);
        m.insert(id::channel(c, id::CH_PARASITIC), 0.2);
        m.insert(id::channel(c, id::CH_DISPLACEMENT_LIMIT), 0.0005);
        m.insert(id::channel(c, id::CH_TEMP_HARD), 130.0);
        m.insert(id::channel(c, id::CH_TEMP_SOFT), 115.0);
        m.insert(id::channel(c, id::CH_VELOCITY_LIMIT0), 0.5);
        m.insert(id::channel(c, id::CH_MAX_VELOCITY_ATTEN), 18.0);
        m.insert(id::channel(c, id::CH_MAX_TEMP_ATTEN), 10.0);
        m.insert(id::channel(c, id::CH_TEMP_HEADROOM), 4.0);
        m.insert(id::channel(c, id::CH_POWER_AMP_MODEL), 0.0);
        m.insert(id::channel(c, id::CH_MASK_BAND_MIN), 8000.0);
        m.insert(id::channel(c, id::CH_MASK_BAND_MAX), 12000.0);
        m.insert(id::channel(c, id::CH_MASK_ATTACK), 0.01);
        m.insert(id::channel(c, id::CH_MASK_RELEASE), 0.1);
        m.insert(id::channel(c, id::CH_MASK_ATTACK_FAST_MS), 5.0);
        m.insert(id::channel(c, id::CH_MASK_ATTACK_RANGE), 70.0);
        m.insert(id::channel(c, id::CH_BANDWIDTH), 0.0);
        for (j, v) in [0.5, 0.6, 0.9].into_iter().enumerate() {
            m.insert(id::channel(c, id::CH_VELOCITY_LIMIT0 + j as u32), v);
        }
        for (j, v) in [10.0, 20.0, 30.0, 40.0].into_iter().enumerate() {
            m.insert(id::channel(c, id::CH_MASK_ENERGY0 + j as u32), v);
        }
        for (j, v) in [60.0, 300.0, 650.0, 1600.0, 5000.0].into_iter().enumerate() {
            m.insert(id::channel(c, id::CH_BAND_CENTER0 + j as u32), v);
        }
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
    // The operating point is the tuning's VoltsFullScale.
    assert_eq!(c.full_scale_mv, 8414);
    let p = sgv2_core::lsm2::Params::from_blob(&c.blob).unwrap();
    assert_eq!((p.channels, p.sample_rate, p.model_only), (4, 48000, 1));
    assert_eq!((p.attack_time_ms, p.fast_release_time_ms, p.slow_release_time_ms), (4.0, 150.0, 15000.0));
    assert_eq!((p.filterbank_latency_ms, p.speaker_response_lookahead_ms), (10.0 / 3.0, 2.0 / 3.0));
    assert_eq!(p.power_hard_limit_w, [50.0, 8.0, 3.0, 2.0]);
    assert_eq!((p.low_rail_idle_power_w, p.low_rail_power_ratio, p.other_power_features), (0.02, 1.2, 0));
    let x = &p.ch[1];
    assert_eq!((x.source, x.volts_full_scale, x.parasitic_resistance), (1, 8.414, 0.2));
    // Admittance is manager 1, displacement manager 2 (distinct synthetic Rdc/Kms).
    assert_eq!((x.rdc, x.kms, x.disp_kms), (6.0, 1510.0, 1520.0));
    assert!((x.mms_kg - 0.0003).abs() < 1e-9);
    assert_eq!((x.rth_vc, x.tau_mag, x.ambient), (30.0, 80.0, 45.0));
    assert_eq!((x.temperature_soft_limit, x.temperature_hard_limit, x.temperature_headroom), (115.0, 130.0, 4.0));
    // velocity manager (3) and the masking/filterbank fields
    assert_eq!((x.vel_rdc, x.vel_kms, x.disp_rdc), (8.0, 1530.0, 7.0));
    assert_eq!((x.velocity_limit, x.mask_band_hz), ([0.5, 0.6, 0.9], [8000.0, 12000.0]));
    assert!((x.mask_attack_fast_s - 0.005).abs() < 1e-9);
    assert_eq!(x.band_center_hz, [60.0, 300.0, 650.0, 1600.0, 5000.0]);
    // no property 6632 in the synthetic bundle: no masking path
    assert_eq!(x.pressure_taps, 0);
}

#[test]
fn lsm2_refuses_what_the_payload_cannot_carry() {
    for (name, k, v) in [
        ("res2", id::model(id::MANAGER_ADMITTANCE, 0, id::M_ENABLE_RES2), 1.0),
        ("lr2", id::model(id::MANAGER_DISPLACEMENT, 0, id::M_ENABLE_LR2), 1.0),
        ("fastpeak", id::controller(id::C_USE_FAST_PEAK), 1.0),
        ("tuning", id::CH_TUNING + 1, 1.0),
        ("type", id::CH_SPEAKER_TYPE, 3.0),
        // Channels at different operating points.
        ("vfs", id::channel(2, id::CH_VOLTS_FS), 6.0),
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
