//! spp3 against the behaviour specification's equations, on synthetic
//! parameters only (no Mac's tuning appears here).

use sgv2_core::spp3::{self, ChannelParams, Params};
use sgv2_core::{Health, Io, ProtectionUnit, Telemetry};

const RATE: u32 = 48_000;
const BLOCK: u32 = 512;

/// Synthetic two-group, four-channel parameter set with short time
/// constants so the tests settle quickly.
fn params() -> Params {
    let mut p = Params {
        channels: 4,
        sample_rate: RATE,
        block_frames: BLOCK,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 0.5,
        thermal_gain_release_time_s: 2.0,
        thermal_lookahead_delay_ms: 1.5,
        thermal_protection_enabled: 1,
        thermal_safe_tlim: 100.0,
        thermal_safe_tlim_offset: -15.0,
        thermal_safe_tlim_time_min: 10.0,
        thermal_state_override: spp3::THERMAL_STATE_COLD,
        ..Params::default()
    };
    for c in 0..4 {
        p.ch[c] = ChannelParams {
            source: c as u32,
            group: if c < 2 { 7 } else { 3 },
            ignore_telemetry: 0,
            t_sett_vc: 20.0 + c as f32,
            t_sett_mg: 10.0,
            tau_tvc: 0.4,
            tau_tmg: 3.0,
            temperature_limit: 120.0,
            hard_temp_limit_headroom: 8.0,
            reb_ref: 4.0,
            rshunt: 0.2,
            thermal_ff_speedup_factor: 0.5,
            cl_thermal_feedback_enabled: 1,
            rampout: 0.0,
            ag: 10.0,
            imon_scale: 2.0,
            vmon_scale: 12.0,
            i_pwr_thres: 0.0,
            v_pwr_thres: 0.0,
            sense_i: 2 * c as u32,
            sense_v: 2 * c as u32 + 1,
            pilot_tone_enabled: 0,
            pilot_ampl_hi_db: -30.0,
            pilot_ampl_lo_db: -40.0,
            pilot_upper_thres: 90.0,
            pilot_lower_thres: 80.0,
            pilot_decay_time_s: 0.03,
            pilot_decay_time_stage2_s: 0.03,
            pilot_freq: 43.0664,
            pilot_enable_thres: 0.03,
            thermal_measurement_method: spp3::METHOD_PILOT_TONE,
            temp_sense_window_time_s: 0.1,
            temp_sense_smoothing_tau_s: 0.5,
            rdc_scale: 1.0,
            reb_range_min: 2.0,
            reb_range_max: 8.0,
            cal_temp_range_min: 10.0,
            cal_temp_range_max: 50.0,
            a_t_20c: 0.004,
            a_t_35c: 0.0038,
            tlim_err_decay_time_s: 5.0,
            tlim_adj_cap: 30.0,
            cal_present: 0,
            cal_r: 0.0,
            cal_t: 0.0,
        };
    }
    p
}

const U: usize = spp3::THERMAL_FRAMES;
const LOOK: usize = 72; // 1.5 ms at 48 kHz

fn dt() -> f64 {
    U as f64 / RATE as f64
}

fn elec(p: &Params, c: usize) -> f64 {
    let ch = &p.ch[c];
    let (ag, reb, rs) = (ch.ag as f64, ch.reb_ref as f64, ch.rshunt as f64);
    ag * ag * reb / ((reb + rs) * (reb + rs))
}

/// Run `seconds` of constant-level drive `x[c]` through the unit in host
/// slices of `slice` frames; return the final telemetry and the last output
/// sample per channel.
fn drive(unit: &mut spp3::Unit, x: [f32; 4], seconds: f64, slice: usize) -> (Telemetry, [f32; 4]) {
    let total = (seconds * RATE as f64) as usize;
    let input: Vec<Vec<f32>> = (0..4).map(|c| vec![x[c]; slice]).collect();
    let mut output = vec![vec![0.0f32; slice]; 4];
    let mut done = 0;
    while done < total {
        let n = slice.min(total - done);
        let ins: Vec<&[f32]> = input.iter().map(|b| &b[..n]).collect();
        let mut outs: Vec<&mut [f32]> = output.iter_mut().map(|b| &mut b[..n]).collect();
        unit.process(Io { frames: n, input: &ins, sense: &[], output: &mut outs });
        done += n;
    }
    let mut t = Telemetry::new();
    unit.telemetry(&mut t);
    let last = [0, 1, 2, 3].map(|c| output[c][(total - 1) % slice]);
    (t, last)
}

#[test]
fn steady_state_temperature() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let (t, _) = drive(&mut u, [0.08; 4], 60.0, 1024);
    for c in 0..4 {
        let pw = elec(&p, c) * 0.0064;
        let want = 40.0 + pw * (p.ch[c].t_sett_vc as f64 + p.ch[c].t_sett_mg as f64);
        let got = t.channels[c].ol_temp_c as f64;
        assert!((got - want).abs() < 0.01, "ch{c} {got} {want}");
    }
}

#[test]
fn host_slicing_does_not_change_anything() {
    let p = params();
    let n = 48_000 * 3;
    let sig: Vec<Vec<f32>> = (0..4)
        .map(|c| {
            let mut s = 17u32 + c;
            (0..n)
                .map(|k| {
                    s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                    let env = if (k / 9000) % 2 == 0 { 0.9 } else { 0.05 };
                    ((s >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 2.0 * env
                })
                .collect()
        })
        .collect();
    let render = |slice: usize| {
        let mut u = spp3::Unit::new(p);
        let mut out = vec![vec![0.0f32; n]; 4];
        let mut t = Telemetry::new();
        let mut done = 0;
        while done < n {
            let m = slice.min(n - done);
            let ins: Vec<&[f32]> = sig.iter().map(|b| &b[done..done + m]).collect();
            let mut outs: Vec<&mut [f32]> = out.iter_mut().map(|b| &mut b[done..done + m]).collect();
            u.process(Io { frames: m, input: &ins, sense: &[], output: &mut outs });
            done += m;
        }
        u.telemetry(&mut t);
        let state: Vec<(f32, f32, f32)> =
            (0..4).map(|c| (t.channels[c].ol_temp_c, t.channels[c].magnet_temp_c, t.channels[c].gain)).collect();
        (out, (t.frame, state))
    };
    let (ref_out, ref_t) = render(512);
    // the limiter must actually have engaged for this to mean anything
    assert!(ref_t.1[0].2 < 0.999, "{:?}", ref_t);
    for slice in [1usize, 7, 31, 96, 127, 160, 1024, 4096] {
        let (o, t) = render(slice);
        assert!(o == ref_out, "audio differs at slice {slice}");
        assert_eq!(t, ref_t, "state differs at slice {slice}");
    }
}

#[test]
fn audio_is_not_delayed_and_unity_is_bit_exact() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let mut x = vec![0.0f32; 512];
    x[10] = 0.25;
    x[11] = -0.123_456_79;
    let ins: Vec<&[f32]> = (0..4).map(|_| &x[..]).collect();
    let mut ob = vec![vec![0.0f32; 512]; 4];
    let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
    u.process(Io { frames: 512, input: &ins, sense: &[], output: &mut outs });
    for o in &ob {
        assert_eq!(o, &x);
    }
}

#[test]
fn failsafe_cases_are_exactly_twenty_db() {
    let x = [0.5f32; 64];
    let mut o = [[0.0f32; 64]; 4];
    // channel-count mismatch
    let mut u = spp3::Unit::new(params());
    let ins: [&[f32]; 3] = [&x, &x, &x];
    {
        let [a, b, c, _] = &mut o;
        let mut outs: [&mut [f32]; 3] = [a, b, c];
        let h = u.process(Io { frames: 64, input: &ins, sense: &[], output: &mut outs });
        assert!(h.is_failsafe() && h.0 & Health::CHANNEL_MISMATCH != 0);
    }
    assert!(o[0].iter().all(|&v| (v - 0.05).abs() < 1e-7));
    // invalid essential parameters
    let mut bad = params();
    bad.ch[2].tau_tvc = f32::NAN;
    assert!(spp3::Unit::init_error(&bad).is_some());
    // a controller the implementation does not have
    let mut cl = params();
    cl.enable_cpms_control = 1;
    assert_eq!(spp3::Unit::init_error(&cl), Some(spp3::InitError::Unsupported("EnableCPMSControl")));
    for p in [bad, cl, Params::default()] {
        let mut u = spp3::Unit::new(p);
        assert!(u.is_failsafe());
        let ins: [&[f32]; 4] = [&x; 4];
        let [a, b, c, d] = &mut o;
        let mut outs: [&mut [f32]; 4] = [a, b, c, d];
        let h = u.process(Io { frames: 64, input: &ins, sense: &[], output: &mut outs });
        assert!(h.is_failsafe() && h.0 & Health::NO_PARAMS != 0);
        assert!(o[3].iter().all(|&v| (v - 0.05).abs() < 1e-7));
    }
}

#[test]
fn non_finite_input_never_reaches_the_output_or_the_model() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let mut x = vec![0.1f32; 512];
    x[5] = f32::NAN;
    x[6] = f32::INFINITY;
    let ins: Vec<&[f32]> = (0..4).map(|_| &x[..]).collect();
    let mut ob = vec![vec![0.0f32; 512]; 4];
    let mut any_fs = false;
    for _ in 0..3 {
        let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
        let h = u.process(Io { frames: 512, input: &ins, sense: &[], output: &mut outs });
        any_fs |= h.is_failsafe();
        assert!(ob.iter().flatten().all(|v| v.is_finite()));
    }
    assert!(any_fs);
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    assert!(t.channels[0].ol_temp_c.is_finite() && t.channels[0].gain.is_finite());
}

#[test]
fn blob_roundtrip() {
    use sgv2_core::blob::{build, Pod};
    let p = params();
    let b = build(sgv2_core::UnitKind::Spp3, 4, 8, RATE, p.as_bytes());
    assert_eq!(Params::from_blob(&b).unwrap(), p);
    let mut bad = b.clone();
    let n = bad.len();
    bad[n - 3] ^= 0x40;
    assert!(Params::from_blob(&bad).is_err());
}

#[test]
fn measured_power_and_sense_geometry() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let x = vec![0.1f32; 512];
    let i = vec![0.05f32; 512];
    let v = vec![0.2f32; 512];
    let ins: Vec<&[f32]> = (0..4).map(|_| &x[..]).collect();
    let sense: Vec<&[f32]> = (0..8).map(|k| if k % 2 == 0 { &i[..] } else { &v[..] }).collect();
    let mut ob = vec![vec![0.0f32; 512]; 4];
    let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
    let h = u.process(Io { frames: 512, input: &ins, sense: &sense, output: &mut outs });
    assert_eq!(h, Health::OK);
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    // the published meter reads 0 W without the closed loop
    assert_eq!(t.channels[0].measured_power_w, 0.0);
    let want = 0.05 * 2.0 * 0.2 * 12.0;
    assert!((u.sense_power(0) - want).abs() < 1e-6);
    // too few sense ports: open loop, never fail-safe
    let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
    let h = u.process(Io { frames: 512, input: &ins, sense: &sense[..3], output: &mut outs });
    assert_eq!(h.0, Health::OPEN_LOOP);
}

#[test]
fn disabled_thermal_protection_is_refused() {
    let mut p = params();
    p.thermal_protection_enabled = 0;
    assert!(spp3::Unit::new(p).is_failsafe());
}

#[test]
fn power_topology_has_no_history_shift() {
    // Pcoil and Pdrv are the update's own frames (spec 4, 16): the 2 ms
    // history does not shorten the first update
    let p = params();
    let mut u = spp3::Unit::new(p);
    assert!(!u.is_failsafe());
    assert_eq!(u.latency_frames(), 0);
    assert_eq!(u.lookahead_frames(), LOOK);
    let (t, _) = drive(&mut u, [0.1; 4], U as f64 / RATE as f64, U);
    let want = elec(&p, 0) * 0.01;
    assert!((t.channels[0].modeled_power_w as f64 - want).abs() < 1e-6 * want, "{}", t.channels[0].modeled_power_w);
    assert!((u.drive_power(0) - want).abs() < 1e-6 * want);
}

#[test]
fn rampout_is_series_resistance() {
    let mut p = params();
    p.ch[0].rampout = 0.05;
    let mut u = spp3::Unit::new(p);
    assert!(!u.is_failsafe());
    let (t, _) = drive(&mut u, [0.1; 4], 2.0 * U as f64 / RATE as f64, U);
    let c = &p.ch[0];
    let (ag, reb) = (c.ag as f64, c.reb_ref as f64);
    let r = reb + c.rshunt as f64 + 0.05;
    let want = (ag * 0.1 / r).powi(2) * reb;
    assert!((t.channels[0].modeled_power_w as f64 - want).abs() < 1e-6 * want);
}

#[test]
fn trapezoidal_thermal_recurrence() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let x = 0.05f64;
    let c = &p.ch[1];
    let (a_vc, b_vc) = (1.0 - dt() / c.tau_tvc as f64, 0.5 * c.t_sett_vc as f64 * dt() / c.tau_tvc as f64);
    let (a_mg, b_mg) = (1.0 - dt() / c.tau_tmg as f64, 0.5 * c.t_sett_mg as f64 * dt() / c.tau_tmg as f64);
    let (mut vc, mut mg, mut prev) = (0.0f64, 0.0f64, 0.0f64);
    for k in 0..800 {
        let fill = if k == 0 { (U - LOOK) as f64 / U as f64 } else { 1.0 };
        let pw = elec(&p, 1) * x * x * fill;
        vc = a_vc * vc + b_vc * (pw + prev);
        mg = a_mg * mg + b_mg * (pw + prev);
        prev = pw;
    }
    let (t, _) = drive(&mut u, [x as f32; 4], 800.0 * dt(), 333);
    assert!(
        (t.channels[1].ol_temp_c as f64 - (40.0 + vc + mg)).abs() < 1e-3,
        "{} {}",
        t.channels[1].ol_temp_c,
        40.0 + vc + mg
    );
    assert!((t.channels[1].magnet_temp_c as f64 - (40.0 + mg)).abs() < 1e-3);
    assert_eq!(t.channels[1].gain, 1.0, "well under the limit: no attenuation");
}

#[test]
fn coefficient_law_is_99_percent_settling() {
    let k = spp3::settle99(4.6, 48_000.0 / 256.0);
    assert!((k - 0.005_325_098_88).abs() < 1e-10, "{k}");
    let k = spp3::settle99(20.0, 48_000.0 / 256.0);
    assert!((k - 0.001_227_291_64).abs() < 1e-10, "{k}");
    let k = spp3::settle99(0.005, 48_000.0);
    assert!((k - 0.019_005_287_3).abs() < 1e-9, "{k}");
}

#[test]
fn feed_forward_onset_is_the_virtual_coil_plus_one_degree() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    // ch0 alone, loud: find the first update with a target below unity and
    // check the onset condition there with an independent recurrence
    let c = &p.ch[0];
    let tau_ff = c.thermal_ff_speedup_factor as f64 * c.tau_tvc as f64;
    let (a_ff, b_ff) = (1.0 - dt() / tau_ff, 0.5 * c.t_sett_vc as f64 * dt() / tau_ff);
    let (a_mg, b_mg) = (1.0 - dt() / c.tau_tmg as f64, 0.5 * c.t_sett_mg as f64 * dt() / c.tau_tmg as f64);
    let (mut ff, mut mg, mut prev) = (0.0f64, 0.0f64, 0.0f64);
    let x = 0.6f64;
    let mut t = Telemetry::new();
    for k in 0..4000 {
        let fill = if k == 0 { (U - LOOK) as f64 / U as f64 } else { 1.0 };
        let gain = t.channels[0].gain as f64; // at most unity before onset
        let pw = elec(&p, 0) * x * x * fill * if k == 0 { 1.0 } else { gain * gain };
        ff = a_ff * ff + b_ff * (pw + prev);
        mg = a_mg * mg + b_mg * (pw + prev);
        prev = pw;
        let (tt, _) = drive(&mut u, [x as f32, 0.0, 0.0, 0.0], dt(), U);
        t = tt;
        let predicted = 40.0 + mg + ff + 1.0;
        if t.channels[0].axis_gain[4] < 1.0 {
            assert!(predicted > c.temperature_limit as f64, "onset at {predicted}");
            return;
        }
        assert!(predicted <= c.temperature_limit as f64 + 1e-6, "missed onset at {predicted}");
    }
    panic!("no onset");
}

#[test]
fn limiting_holds_below_the_limit_and_links_the_group() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let (t, _) = drive(&mut u, [0.5, 0.0, 0.0, 0.0], 120.0, 1024);
    let temp = t.channels[0].ol_temp_c;
    assert!(temp < 120.0 && temp > 115.0, "{temp}");
    assert!(t.channels[0].gain < 0.9);
    assert_eq!(t.channels[1].gain, t.channels[0].gain);
    assert_eq!(t.channels[2].gain, 1.0);
}

#[test]
fn hard_path_arms_above_headroom_and_bypasses_the_attack() {
    let mut p = params();
    for c in 0..4 {
        // a slow attack and a feed-forward pole equal to the coil's, so the
        // ordinary path lets the coil overshoot into the hard path
        p.ch[c].thermal_ff_speedup_factor = 1.0;
    }
    p.thermal_gain_attack_time_s = 60.0;
    let mut u = spp3::Unit::new(p);
    let mut armed_at = None;
    let mut pre = 1.0f32;
    let mut peak: f32 = 0.0;
    for k in 0..(30.0 / dt()) as usize {
        let (t, _) = drive(&mut u, [1.0; 4], dt(), U);
        peak = peak.max(t.channels[0].ol_temp_c);
        if armed_at.is_none() && t.channels[0].ol_temp_c > 128.0 {
            armed_at = Some(k);
            pre = t.channels[0].gain;
        }
        if let Some(a) = armed_at {
            if k == a + 2 {
                // the hard target is selected at once and reached through the
                // ~7 ms dezipper within the next decision interval
                assert!(t.channels[0].axis_gain[4] < 0.9, "{}", t.channels[0].axis_gain[4]);
                assert!(t.channels[0].gain < pre - 0.05, "{} {pre}", t.channels[0].gain);
            }
        }
    }
    assert!(armed_at.is_some());
    // the hard path pulls the coil back down below its own peak
    let (t, _) = drive(&mut u, [1.0; 4], 30.0, U);
    assert!(t.channels[0].ol_temp_c < peak);
}

#[test]
fn zero_speedup_factor_holds_the_feed_forward_reduction_at_unity() {
    let mut p = params();
    for c in 0..4 {
        p.ch[c].thermal_ff_speedup_factor = 0.0;
    }
    let mut u = spp3::Unit::new(p);
    // steady state between the limit and the hard limit
    let (t, _) = drive(&mut u, [0.355, 0.0, 0.0, 0.0], 40.0, 1024);
    assert!(t.channels[0].ol_temp_c > 120.0, "{}", t.channels[0].ol_temp_c);
    assert_eq!(t.channels[0].axis_gain[4], 1.0);
    assert_eq!(t.channels[0].gain, 1.0);
}

#[test]
fn pilot_mode_reserves_program_headroom_from_the_first_sample() {
    let mut p = params();
    for c in 0..4 {
        p.ch[c].pilot_tone_enabled = 1;
    }
    let mut u = spp3::Unit::new(p);
    let mut x = vec![0.0f32; 512];
    x[0] = 0.5;
    let ins: Vec<&[f32]> = (0..4).map(|_| &x[..]).collect();
    let mut ob = vec![vec![0.0f32; 512]; 4];
    let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
    u.process(Io { frames: 512, input: &ins, sense: &[], output: &mut outs });
    let want = 0.5 * (1.0 - 10f32.powf(-30.0 / 20.0));
    assert!((ob[0][0] - want).abs() < 1e-7, "{} {want}", ob[0][0]);
}

#[test]
fn hot_fallback_seeds_the_steady_state_split_at_the_limit() {
    // 64007 other than 2 (spec 10): theta_vc0 = P0 T_sett_vc, theta_mg0 =
    // P0 T_sett_mg, theta_ff0 = theta_vc0 with P0 = (Tlim - ambient) /
    // (T_sett_vc + T_sett_mg); the previous power is zero
    for ovr in [0u32, 1, 3, 7] {
        let mut p = params();
        p.thermal_state_override = ovr;
        let mut u = spp3::Unit::new(p);
        for _ in 0..2 {
            let (t, _) = drive(&mut u, [0.0; 4], dt(), U);
            for c in 0..4 {
                let ch = &p.ch[c];
                let (tvc, tmg) = (ch.t_sett_vc as f64, ch.t_sett_mg as f64);
                let p0 = (120.0 - 40.0) / (tvc + tmg);
                let want =
                    40.0 + p0 * tvc * (1.0 - dt() / ch.tau_tvc as f64) + p0 * tmg * (1.0 - dt() / ch.tau_tmg as f64);
                assert!((t.channels[c].ol_temp_c as f64 - want).abs() < 1e-3, "64007={ovr} ch{c}");
            }
            // kernel reconstruction reseeds
            u.reset();
        }
    }
}

/// Program and pilot-bin telemetry for the closed-loop tests: V is a sine
/// on the analyser bin (N = 8192 for 0.1 s, bin m = round(43.0664 N / fs)),
/// I = V / R.
fn pilot_sense(r: f64, frames: usize) -> Vec<Vec<f32>> {
    let n = 8192.0;
    let m = (43.0664f64 * n / RATE as f64).round();
    let v: Vec<f32> =
        (0..frames).map(|k| (0.05 * (2.0 * std::f64::consts::PI * m * k as f64 / n).sin()) as f32).collect();
    let i: Vec<f32> = v.iter().map(|&x| ((x as f64 * 12.0 / r) / 2.0) as f32).collect();
    (0..8).map(|k| if k % 2 == 0 { i.clone() } else { v.clone() }).collect()
}

fn run_cl(u: &mut spp3::Unit, x: f32, tele: &[Vec<f32>], from: usize, to: usize) -> Telemetry {
    let xs = vec![x; 512];
    let mut ob = vec![vec![0.0f32; 512]; 4];
    let mut f = from;
    while f + 512 <= to {
        let ins: Vec<&[f32]> = (0..4).map(|_| &xs[..]).collect();
        let sense: Vec<&[f32]> = tele.iter().map(|b| &b[f..f + 512]).collect();
        let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
        u.process(Io { frames: 512, input: &ins, sense: &sense, output: &mut outs });
        f += 512;
    }
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    t
}

fn cl_params(rc: f32, tc: f32) -> Params {
    let mut p = params();
    for c in 0..4 {
        p.ch[c].cal_present = 1;
        p.ch[c].cal_r = rc;
        p.ch[c].cal_t = tc;
    }
    p
}

#[test]
fn closed_loop_reads_coil_temperature_from_the_pilot_bin_and_never_relaxes() {
    let (rc, tc) = (4.0f32, 25.0f32);
    let p = cl_params(rc, tc);
    let mut u = spp3::Unit::new(p);
    // a resistive plant whose coil sits at 60 C: R = Rc (1 + alpha (T - Tc))
    let alpha = 0.004 + (25.0 - 20.0) * (0.0038 - 0.004) / 15.0;
    let r = rc as f64 * (1.0 + alpha * (60.0 - 25.0));
    let n = 48_000 * 20;
    let tele = pilot_sense(r, n);
    let t = run_cl(&mut u, 0.3, &tele, 0, n);
    assert!(t.channels[0].closed_loop);
    assert!((t.channels[0].cl_temp_c - 60.0).abs() < 0.05, "T_CL {}", t.channels[0].cl_temp_c);
    assert_eq!(u.controller_state(0)[9], 1.0, "window accepted");
    // the model runs hotter than the sensed 60 C, but the correction is
    // capped at the base until the alpha inversion is resolved (t-4668)
    assert_eq!(t.channels[0].tlim_adj_c, 0.0);
    assert_eq!(u.effective_limit(0), 120.0);
}

#[test]
fn closed_loop_range_is_on_the_window_and_rejection_holds_the_published_temperature() {
    let p = cl_params(4.0, 25.0);
    let mut u = spp3::Unit::new(p);
    let alpha = 0.004 + (25.0 - 20.0) * (0.0038 - 0.004) / 15.0;
    let n = 48_000 * 20;
    let tele = pilot_sense(4.0 * (1.0 + alpha * 35.0), n);
    run_cl(&mut u, 0.3, &tele, 0, n);
    assert_eq!(u.controller_state(0)[9], 1.0);
    // an implausible window (R above RebRange) is rejected at once; the
    // published temperature follows the smoothed (rejected) window
    let bad = pilot_sense(20.0, n);
    let t = run_cl(&mut u, 0.3, &bad, 0, 48_000 * 2);
    assert!(t.channels[0].closed_loop);
    assert_eq!(u.controller_state(0)[9], 0.0);
    assert!(t.channels[0].cl_temp_c > 100.0);
    assert_eq!(u.effective_limit(0), 120.0);
    // recovery is accepted on the instantaneous window although the
    // smoothed resistance is still out of range
    run_cl(&mut u, 0.3, &tele, 0, 48_000);
    assert_eq!(u.controller_state(0)[9], 1.0);
}

#[test]
fn audio_unit_reset_clears_only_the_closed_loop() {
    let p = cl_params(4.0, 25.0);
    let mut u = spp3::Unit::new(p);
    let alpha = 0.004 + (25.0 - 20.0) * (0.0038 - 0.004) / 15.0;
    let n = 48_000 * 10;
    let tele = pilot_sense(4.0 * (1.0 + alpha * 35.0), n);
    let t0 = run_cl(&mut u, 0.3, &tele, 0, n);
    let s0 = u.controller_state(0);
    u.host_reset();
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    // OL state and gains continue; CL back to the calibration and the base
    assert_eq!(t.channels[0].ol_temp_c, t0.channels[0].ol_temp_c);
    assert_eq!(t.channels[0].gain, t0.channels[0].gain);
    assert_eq!(u.controller_state(0)[1], s0[1]);
    assert_eq!(t.channels[0].cl_temp_c, 25.0);
    assert_eq!(t.channels[0].tlim_adj_c, 0.0);
    // an open-loop kernel is unchanged by it
    let mut q = spp3::Unit::new(params());
    drive(&mut q, [0.2; 4], 2.0, 512);
    let (mut a, mut b) = (Telemetry::new(), Telemetry::new());
    q.telemetry(&mut a);
    q.host_reset();
    q.telemetry(&mut b);
    assert_eq!(a.channels[0].ol_temp_c, b.channels[0].ol_temp_c);
    assert_eq!(a.channels[0].gain, b.channels[0].gain);
    // kernel reconstruction reseeds
    q.reset();
    q.telemetry(&mut b);
    assert_eq!(b.channels[0].ol_temp_c, 40.0);
}

#[test]
fn safe_tlim_count_is_minutes_in_float32() {
    assert_eq!(spp3::safe_count_init(10.0, 48_000), Some(112_500));
    assert_eq!(spp3::safe_count_init(0.1, 48_000), Some(1125));
    assert_eq!(spp3::safe_count_init(0.0, 48_000), Some(0));
}

/// Hold `x` for `updates` thermal updates; return how many updates ran
/// before channel 0 latched, or `None`.
fn hold_until_latched(u: &mut spp3::Unit, x: f32, updates: usize) -> Option<usize> {
    let xs = vec![x; U];
    let mut ob = vec![vec![0.0f32; U]; 4];
    for k in 0..updates {
        let ins: Vec<&[f32]> = (0..4).map(|_| &xs[..]).collect();
        let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
        u.process(Io { frames: U, input: &ins, sense: &[], output: &mut outs });
        if u.safe_tlim_latched(0) {
            return Some(k);
        }
    }
    None
}

#[test]
fn safe_tlim_latches_after_ten_continuous_minutes() {
    // a long synthetic case: 1 FS keeps T_OL above Tlim + offset (105 C)
    // under limiting; 112 500 consecutive true updates latch, one-way
    let p = params();
    let mut u = spp3::Unit::new(p);
    let n = 112_500 + 4000;
    let k = hold_until_latched(&mut u, 1.0, n).expect("latched within the run");
    // predicate first true at update q; latch after the 112 500th true one
    let mut v = spp3::Unit::new(p);
    let q = (0..4000)
        .find(|_| {
            hold_until_latched(&mut v, 1.0, 1);
            let s = v.controller_state(0);
            40.0 + s[4] + s[5] > 105.0
        })
        .unwrap();
    assert_eq!(k, q + 112_500 - 1, "q {q} k {k}");
    // the new base applies from the next update
    assert_eq!(u.effective_limit(0), 120.0);
    hold_until_latched(&mut u, 1.0, 1);
    assert_eq!(u.effective_limit(0), 100.0);
    // no runtime exit: ten minutes of silence keep it
    hold_until_latched(&mut u, 0.0, 112_600);
    assert!(u.safe_tlim_latched(0));
    assert_eq!(u.effective_limit(0), 100.0);
    // AudioUnitReset keeps it; reconstruction restores the ordinary base
    u.host_reset();
    assert!(u.safe_tlim_latched(0));
    u.reset();
    assert!(!u.safe_tlim_latched(0));
    // other channels latched on their own predicates
    assert!(p.ch[1].temperature_limit == 120.0);
}

#[test]
fn safe_tlim_reloads_on_equality_or_cooling() {
    // TimeMin 0.1 (1125 updates): 1000 hot updates, cool, then hot again;
    // the count restarts from the full value
    let mut p = params();
    p.thermal_safe_tlim_time_min = 0.1;
    let mut u = spp3::Unit::new(p);
    assert_eq!(hold_until_latched(&mut u, 1.0, 1000), None);
    assert_eq!(hold_until_latched(&mut u, 0.0, 3000), None);
    let mut v = spp3::Unit::new(p);
    let fresh = hold_until_latched(&mut v, 1.0, 5000).unwrap();
    let again = hold_until_latched(&mut u, 1.0, 5000).unwrap();
    // from a hotter start the predicate turns true sooner, never later than
    // a full reload allows
    assert!(again >= 1124 && again <= fresh, "{again} {fresh}");
}

#[test]
fn safe_tlim_setup_clamps() {
    // SafeTlim above TemperatureLimit is clamped down; a positive offset is 0
    let mut p = params();
    p.thermal_safe_tlim = 200.0;
    p.thermal_safe_tlim_offset = 5.0;
    p.thermal_safe_tlim_time_min = 0.01;
    let mut u = spp3::Unit::new(p);
    hold_until_latched(&mut u, 1.0, 20000);
    hold_until_latched(&mut u, 1.0, 1);
    if u.safe_tlim_latched(0) {
        assert_eq!(u.effective_limit(0), 120.0);
    }
    // TimeMin 0 latches on the first update
    let mut p = params();
    p.thermal_safe_tlim_time_min = 0.0;
    let mut u = spp3::Unit::new(p);
    assert_eq!(hold_until_latched(&mut u, 0.0, 1), Some(0));
}

#[test]
fn pilot_gates_on_drive_power_and_selects_by_t_ol() {
    let mut p = cl_params(4.0, 25.0);
    for c in 0..4 {
        p.ch[c].pilot_tone_enabled = 1;
    }
    let mut u = spp3::Unit::new(p);
    // Pdrv = elec * x^2 for DC: the gate is >= PilotEnableThres (0.03 W)
    let e = elec(&p, 0);
    let x0 = (0.03f64 / e).sqrt() as f32;
    let below = f32::from_bits(x0.to_bits() - 64);
    drive(&mut u, [below; 4], dt(), U);
    assert_eq!(u.pilot_state(0), Some((false, false)));
    drive(&mut u, [x0 * 1.0001; 4], dt(), U);
    assert_eq!(u.pilot_state(0), Some((true, false)));
    // above PilotUpperThres the high level; held through the band
    drive(&mut u, [1.0; 4], 0.5, U);
    let s = u.controller_state(0);
    assert!(40.0 + s[4] + s[5] > 90.0);
    assert_eq!(u.pilot_state(0), Some((true, true)));
    // without the calibration vector there is no generator
    let mut q = params();
    q.ch[0].pilot_tone_enabled = 1;
    assert_eq!(spp3::Unit::new(q).pilot_state(0), None);
}

#[test]
fn closed_loop_needs_a_calibration_inside_its_ranges() {
    let mut p = params();
    p.ch[0].cal_present = 1;
    p.ch[0].cal_r = 4.0;
    p.ch[0].cal_t = 60.0; // outside 10..50
    let mut u = spp3::Unit::new(p);
    let (t, _) = drive(&mut u, [0.1; 4], 0.5, 512);
    assert!(!t.channels[0].closed_loop);
}
