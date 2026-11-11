// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The blob compiler, on synthetic bundles: every payload field traced to
//! its tuning source, decoded back with the engine's own decoder.

mod common;

use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::compile::{self as comp, lsm2_ids as id, Provenance};
use aurora_speaker_tuning::fourcc::FourCc;
use aurora_speaker_tuning::machine::Machine;
use aurora_speaker_tuning::plist::Value;
use aurora_speaker_tuning::spts::{self, SizeForm};
use aurora_speaker_tuning::tuning::{Options, Tuning, UnitKind};
use aurora_speaker_tuning::Diagnostics;
use common::*;
use std::collections::BTreeMap;

/// The synthetic bundles have four channels: a four-amp board, the unit
/// they were collected on.
fn j413() -> Machine {
    Machine::with_unit("j413", &test_unit()).unwrap()
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
    assert_eq!((h.machine(), h.full_scale_mv), (Some("j413"), 11890));
    assert_eq!((c.board.as_str(), c.full_scale_mv), ("j413", 11890));
    let p = sgv2_core::spp3::Params::from_blob(&c.blob).unwrap();
    assert_eq!(p.block_frames, 256);
    assert_eq!(p.thermal_ambient_temp, 41.0);
    assert_eq!(p.thermal_gain_attack_time_s, 3.5);
    assert_eq!(p.thermal_lookahead_delay_ms, 1.5);
    assert_eq!(p.thermal_protection_enabled, 1);
    // The fixture strip sets 64007 = 2: the cold start.
    assert_eq!(p.thermal_state_override, 2);
    for ch in 0..4 {
        let x = &p.ch[ch];
        assert_eq!(x.source, ch as u32);
        assert_eq!(x.group, (ch / 2) as u32);
        assert_eq!((x.sense_i, x.sense_v), (2 * ch as u32 + 1, 2 * ch as u32));
        assert_eq!((x.t_sett_vc, x.t_sett_mg, x.tau_tvc, x.tau_tmg), (17.0, 19.0, 2.5, 90.0));
        assert_eq!((x.reb_ref, x.rshunt, x.ag), (3.3, 0.05, 11.89));
        assert_eq!((x.temperature_limit, x.hard_temp_limit_headroom), (121.0, 7.0));
        assert_eq!((x.imon_scale, x.vmon_scale), (2.0, 10.0));
        assert_eq!(x.cl_thermal_feedback_enabled, 1);

        // Record ch carries raw R 3000+ch (x0.001 ohm) and raw T 3100 (x0.01 C).
        assert_eq!(x.cal_present, 1);
        assert_eq!((x.cal_r, x.cal_t), ((3000 + ch) as f32 * 0.001, 3100.0 * 0.01));
        assert_eq!((x.temp_sense_window_time_s, x.tlim_adj_cap, x.a_t_20c), (0.2, 40.0, 0.0039));
    }
}

#[test]
fn spp3_calibration_needs_a_valid_record_per_channel() {
    // A bad checksum: every channel open loop.
    let mut f = spp3_files(4);
    let spts = f.get_mut("speaker-thiele-small.bin").unwrap();
    let n = spts.len();
    spts[n - 1] ^= 1;
    let t = tuning_allowing(&f, "c-cal-bad", &["spts-checksum"]);
    let p = sgv2_core::spp3::Params::from_blob(&compile(&t, &mut Diagnostics::default()).unwrap().blob).unwrap();
    assert!(p.ch[..4].iter().all(|c| c.cal_present == 0));
    // Records bind by position, so a valid file with fewer (or more) records
    // than channels is refused: there is no way to tell which is whose.
    for k in [1usize, 3, 5] {
        let mut f = spp3_files(4);
        let recs: Vec<_> = (0..k).map(|i| (fcc(&format!("spA{i}")), 3500, 3000)).collect();
        f.insert("speaker-thiele-small.bin".into(), spts::encode(&recs, 0));
        let t = tuning_allowing(&f, &format!("c-cal-count-{k}"), &["spts-count"]);
        assert!(compile(&t, &mut Diagnostics::default()).is_err(), "{k} records");
    }
}

#[test]
fn spp3_start_state_follows_64007_per_aid() {
    // The raw 64007 value reaches the engine; unset (AID8/9/14/15) is 0 and
    // is never synthesised as 2.  The engine computes the hot fallback.
    for (state, want) in [(Some(2), 2u32), (None, 0), (Some(0), 0), (Some(1), 1), (Some(7), 7)] {
        let mut f = spp3_files(4);
        f.insert(propstrip_path(), propstrip_with(&sidecar_name(), state));
        let t = tuning(&f, &format!("c-start-{state:?}"));
        let c = compile(&t, &mut Diagnostics::default()).unwrap();
        let p = sgv2_core::spp3::Params::from_blob(&c.blob).unwrap();
        assert_eq!(p.thermal_state_override, want, "{state:?}");
    }
}

fn fcc(s: &str) -> FourCc {
    FourCc::from_str_exact(s).unwrap()
}

/// Compiles `spp3_files(4)` with `recs` as the record file in `form`.
fn compile_records(form: SizeForm, recs: &[(FourCc, u16, u16)], name: &str) -> sgv2_core::spp3::Params {
    let mut f = spp3_files(4);
    f.insert("speaker-thiele-small.bin".into(), spts::encode_as(form, recs));
    let t = tuning(&f, name);
    sgv2_core::spp3::Params::from_blob(&compile(&t, &mut Diagnostics::default()).unwrap().blob).unwrap()
}

#[test]
fn spp3_records_bind_by_position_not_by_id() {
    // Shaped like the M2: four records spA0 spA1 spA4 spA5 in the
    // total-length header.  All four channels take their record in stored
    // order; nothing is left for absent spA2/spA3.
    let raw = |i: usize| (3000 + 100 * i as u16, 3000 + i as u16);
    let sparse: Vec<_> =
        ["spA0", "spA1", "spA4", "spA5"].iter().enumerate().map(|(i, s)| (fcc(s), raw(i).0, raw(i).1)).collect();
    let p = compile_records(SizeForm::TotalLength, &sparse, "c-pos-sparse");
    for c in 0..4 {
        let x = &p.ch[c];
        assert_eq!(x.cal_present, 1, "channel {c}");
        assert_eq!((x.cal_r, x.cal_t), (f32::from(raw(c).0) * 0.001, f32::from(raw(c).1) * 0.01), "channel {c}");
        // The sense pair follows the channel, never the record's ID.
        assert_eq!((x.sense_i, x.sense_v), (2 * c as u32 + 1, 2 * c as u32), "channel {c}");
    }
    // The stride header with the same records binds the same.
    assert_eq!(compile_records(SizeForm::RecordStride, &sparse, "c-pos-stride").ch, p.ch);
    // Reversed IDs with the same values: the same binding (no sort by ID).
    let mut rev = sparse.clone();
    let ids: Vec<FourCc> = rev.iter().rev().map(|r| r.0).collect();
    for (r, id) in rev.iter_mut().zip(ids) {
        r.0 = id;
    }
    assert_eq!(compile_records(SizeForm::TotalLength, &rev, "c-pos-rev").ch, p.ch);
    // IDs outside spA0..spA3, or not spA at all: still positional.
    let odd: Vec<_> = sparse.iter().enumerate().map(|(i, r)| (FourCc(0x1000 + i as u32), r.1, r.2)).collect();
    assert_eq!(compile_records(SizeForm::RecordStride, &odd, "c-pos-odd").ch, p.ch);
}

#[test]
fn spp3_telemetry_order_is_refused() {
    // 64005 orders the telemetry streams; its encoding is not specified, so
    // any strip that sets it is refused, never applied to calibration.
    for (name, key, v) in [
        ("spa", "Array", Value::Array(["spA3", "spA2", "spA1", "spA0"].iter().map(|x| s(x)).collect())),
        ("spvi", "Array", Value::Array(["spV0", "spI0", "spV1", "spI1"].iter().map(|x| s(x)).collect())),
        ("scalar", "UInt32", i(3)),
    ] {
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
    // No table of boards: any board that fits the header compiles, and the
    // header names it (the amp count is the running Mac's to check).
    for b in ["t4", "t999x"] {
        let c = comp::compile(&t, &Machine::new(b).unwrap(), &mut Diagnostics::default()).unwrap();
        assert_eq!(c.board, b);
        assert_eq!(sgv2_core::blob::Header::parse(&c.blob).unwrap().0.machine(), Some(b));
    }
    // The board is all that differs.
    let a = comp::compile(&t, &Machine::new("t4").unwrap(), &mut Diagnostics::default()).unwrap();
    let b = comp::compile(&t, &Machine::new("t999x").unwrap(), &mut Diagnostics::default()).unwrap();
    assert_eq!(a.blob.len(), b.blob.len());
    assert!(a.blob[sgv2_core::blob::HEADER_LEN..] == b.blob[sgv2_core::blob::HEADER_LEN..]);
    assert!(Machine::new("t4-too-long-board").is_err());
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
    // type-A PilotToneAmplitude_dB: the peak axis reserves it
    m.insert(1024 + 2, -40.0);
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
        m.insert(id::channel(c, id::CH_VOLTS_FS), 11.89);
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
    // The operating point is the tuning's VoltsFullScale (peak).
    assert_eq!(c.full_scale_mv, 11890);
    let p = sgv2_core::lsm2::Params::from_blob(&c.blob).unwrap();
    assert_eq!((p.channels, p.sample_rate, p.model_only), (4, 48000, 1));
    assert_eq!((p.attack_time_ms, p.fast_release_time_ms, p.slow_release_time_ms), (4.0, 150.0, 15000.0));
    // SPEC-LSM2 AID44 values: 2 x 160 + 32 = 352 frames.
    assert_eq!((p.filterbank_latency_ms, p.speaker_response_lookahead_ms), (10.0 / 3.0, 2.0 / 3.0));
    assert_eq!(p.latency_frames(), 352);
    assert_eq!(p.power_hard_limit_w, [50.0, 8.0, 3.0, 2.0]);
    assert_eq!((p.low_rail_idle_power_w, p.low_rail_power_ratio, p.other_power_features), (0.02, 1.2, 0));
    let x = &p.ch[1];
    assert_eq!((x.source, x.volts_full_scale, x.parasitic_resistance), (1, 11.89, 0.2));
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
    assert!((x.pilot_reserve - 0.01).abs() < 1e-7);
    // no property 6632 in the synthetic bundle: no masking path
    assert_eq!(x.pressure_taps, 0);
}

#[test]
fn lsm2_masker_lattice_comes_from_the_bundle() {
    // Provisioned: the validated table reaches the engine unchanged.
    let ends = synthetic_lattice_ends();
    let mut files = lsm2_files(|_| {});
    files.insert(LATTICE.into(), lattice_bytes(&ends));
    let t = tuning(&files, "c-lsm2-lattice");
    let c = compile(&t, &mut Diagnostics::default()).unwrap();
    let p = sgv2_core::lsm2::Params::from_blob(&c.blob).unwrap();
    assert_eq!(p.mask_lattice_len, 100);
    assert_eq!(p.mask_lattice.iter().map(|&v| v as u64).collect::<Vec<_>>(), ends);
    // Absent (a bundle collected before the installer read it): no
    // lattice, so no masking credit, and the load warned.
    let dir = scratch("c-lsm2-no-lattice");
    let files = lsm2_files(|_| {});
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    let mut d = Diagnostics::default();
    let t = Tuning::load(&Bundle::open(&dir).unwrap(), &Options::default(), &mut d).unwrap();
    assert!(d.items.iter().any(|x| x.code == "mask-lattice-absent"));
    let c = compile(&t, &mut Diagnostics::default()).unwrap();
    let p = sgv2_core::lsm2::Params::from_blob(&c.blob).unwrap();
    assert_eq!((p.mask_lattice_len, p.mask_lattice), (0, [0; 100]));
    // A listed table that fails the rule is refused before any blob.
    let mut bad = ends.clone();
    bad[99] = 320;
    let mut files = lsm2_files(|_| {});
    files.insert(LATTICE.into(), lattice_bytes(&bad));
    let t = tuning_any(&files, "c-lsm2-bad-lattice");
    assert!(compile(&t, &mut Diagnostics::default()).is_err());
}

#[test]
fn buzzkill_matrix_comes_from_the_bundle() {
    // Provisioned: the validated matrix reaches the tuning unchanged.
    let m = synthetic_buzzkill();
    let mut files = lsm2_files(|_| {});
    files.insert(BUZZKILL.into(), buzzkill_bytes(&m));
    let t = tuning(&files, "c-buzzkill");
    assert_eq!(t.buzzkill, Some(m));
    // Absent: none, and no error (only a Buzz box would need it).
    let t = tuning(&lsm2_files(|_| {}), "c-no-buzzkill");
    assert_eq!(t.buzzkill, None);
    // A listed matrix that fails the structural rule is an error.
    let mut bad = m;
    bad[1][0] = 0.5;
    let mut files = lsm2_files(|_| {});
    files.insert(BUZZKILL.into(), buzzkill_bytes(&bad));
    let dir = scratch("c-bad-buzzkill");
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    let mut d = Diagnostics::default();
    let t = Tuning::load(&Bundle::open(&dir).unwrap(), &Options::default(), &mut d).unwrap();
    assert!(d.items.iter().any(|x| x.code == "buzzkill-matrix-invalid"));
    assert_eq!(t.buzzkill, None);
}

#[test]
fn lsm2_channels_keep_their_own_full_scale_within_one_operating_point() {
    // As on the J700 (7.30/7.33 V, 0.036 dB): the header takes channel 0's,
    // the payload keeps each channel's own.
    let t = tuning(
        &lsm2_files(|m| {
            m.insert(id::channel(1, id::CH_VOLTS_FS), 11.89 * 7.33 / 7.30);
        }),
        "c-lsm2-vfs-close",
    );
    let c = compile(&t, &mut Diagnostics::default()).unwrap();
    assert_eq!(c.full_scale_mv, 11890);
    let p = sgv2_core::lsm2::Params::from_blob(&c.blob).unwrap();
    assert!((p.ch[1].volts_full_scale - 11.89 * 7.33 / 7.30).abs() < 1e-5);
    // Past the supervisor's 0.25 dB: refused.
    let t = tuning(
        &lsm2_files(|m| {
            m.insert(id::channel(3, id::CH_VOLTS_FS), 11.89 * 1.04);
        }),
        "c-lsm2-vfs-far",
    );
    assert!(compile(&t, &mut Diagnostics::default()).is_err());
}

#[test]
fn spp3_operating_point_is_the_peak_ag() {
    // The operating point is the tuning's own ag; the supervisor checks it
    // against the amps' gain on the running card.
    let mut f = spp3_files(4);
    let mut v = sidecar_value(4);
    if let Value::Dict(d) = &mut v {
        if let Some(Value::Dict(chs)) = d.get_mut("ChannelSpecificParams") {
            for c in 0..4 {
                if let Some(Value::Dict(ch)) = chs.get_mut(&format!("Channel{c}")) {
                    if let Some(Value::Dict(od)) = ch.get_mut("OL_disp") {
                        *od.get_mut("ag").unwrap() = Value::Real(9.5);
                    }
                }
            }
        }
    }
    f.insert(format!("{AID}/DSP/Strips/{}", sidecar_name()), aurora_speaker_tuning::plist::binary::write(&v));
    let t = tuning(&f, "c-spp3-ag");
    let mut d = Diagnostics::default();
    let c = compile(&t, &mut d).unwrap();
    assert_eq!(c.full_scale_mv, 9500);
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
        let vals = [11.89, 8.5, 35.0, 2.0, 45.0, 70.0, 40.0, 125.0, 700.0, 18000.0, 9.0, 1.0, mask];
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
    assert_eq!((s.channel_mask, s.amplifier_sensitivity, s.rdc, s.rth_vc, s.tau_mg), (15, 11.89, 8.5, 35.0, 70.0));
    assert_eq!((s.temperature_limit, s.attack_ms, s.release_ms, s.hard_limit_headroom), (125.0, 700.0, 18000.0, 9.0));
    assert_eq!(p.slot[1].channel_mask, 0);
}

/// atsp parameter 0 bypasses the unit (a bit-exact copy, no protection):
/// refused like spp3's disabled thermal protection.
#[test]
fn a_bypassed_atsp_preset_is_refused() {
    let mut params: Vec<(u32, f32)> = vec![(0, 1.0), (40, 0.0)];
    for s in 0..3u32 {
        let b = 1 + 13 * s;
        let mask = if s == 0 { 15.0 } else { 0.0 };
        let vals = [11.89, 8.5, 35.0, 2.0, 45.0, 70.0, 40.0, 125.0, 700.0, 18000.0, 9.0, 1.0, mask];
        for (k, v) in vals.iter().enumerate() {
            params.push((b + k as u32, *v));
        }
    }
    let mut f = spp3_files(4);
    f.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph_text("atsp").into_bytes());
    let st = ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] };
    f.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"), strip_with_state("atsp", &st));
    let t = tuning(&f, "c-atsp-bypass");
    let e = compile(&t, &mut Diagnostics::default()).unwrap_err();
    assert!(format!("{e}").contains("bypasses atsp"), "{e}");
}

/// Plan B (2026-11-07): the shipped compile runs the unit pass-through, an
/// identity pre-stage with no crossover declared, for spp3 as for atsp; the
/// tuning's own pre-stage only when asked for (rigs, the full macOS chain).
#[test]
fn the_shipped_prestage_is_pass_through() {
    let t = tuning(&spp3_files(4), "c-prestage-pass-through");
    let mut d = Diagnostics::default();
    let c = compile(&t, &mut d).unwrap();
    let pre = sgv2_core::blob::prestage(&c.blob, c.sample_rate as f64).unwrap();
    assert!(!pre.from_tuning(), "no crossover declared");
    assert_eq!(pre.box_count, 0, "identity: nothing ahead of the unit");
    assert_eq!(pre, sgv2_core::prestage::Params::identity(c.channels as usize));
    assert!(d.items.iter().any(|x| x.code == "prestage-protection-only"), "the compile says so");
    // The same as asking for it, and the unit itself is the same either way
    let explicit = comp::compile_opts(
        &t,
        &j413(),
        Provenance::Verified,
        comp::PreStageMode::Identity,
        &mut Diagnostics::default(),
    )
    .unwrap();
    assert_eq!(c.blob, explicit.blob);
    // The tuning's pre-stage only on request, exactly what derive gives
    let derived = aurora_speaker_tuning::prestage::derive(
        &t,
        c.sample_rate as f64,
        c.channels as usize,
        &mut Diagnostics::default(),
    );
    let tuned =
        comp::compile_opts(&t, &j413(), Provenance::Verified, comp::PreStageMode::Tuning, &mut Diagnostics::default());
    match (derived, tuned) {
        (Ok(want), Ok(got)) => {
            let p = sgv2_core::blob::prestage(&got.blob, got.sample_rate as f64).unwrap();
            assert_eq!(p.from_tuning(), want.from_tuning());
            assert_eq!(p.box_count, want.box_count);
        }
        (Err(_), Err(_)) => {}
        (a, b) => panic!("derive {:?} vs compile {:?}", a.is_ok(), b.is_ok()),
    }
}
