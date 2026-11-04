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
        thermal_safe_tlim: 0.0,
        thermal_safe_tlim_offset: 0.0,
        thermal_safe_tlim_time_min: 0.0,
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
            initial_theta_vc: 0.0,
            initial_theta_mg: 0.0,
            pilot_tone_enabled: 0,
            pilot_ampl_hi_db: -30.0,
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
fn power_topology_and_the_model_history_fill() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    assert!(!u.is_failsafe());
    assert_eq!(u.latency_frames(), 0);
    assert_eq!(u.lookahead_frames(), LOOK);
    let (t, _) = drive(&mut u, [0.1; 4], U as f64 / RATE as f64, U);
    // the first update sees the 72 empty history frames
    let want = elec(&p, 0) * 0.01 * (U - LOOK) as f64 / U as f64;
    assert!((t.channels[0].modeled_power_w as f64 - want).abs() < 1e-6 * want, "{}", t.channels[0].modeled_power_w);
    let (t, _) = drive(&mut u, [0.1; 4], U as f64 / RATE as f64, U);
    let want = elec(&p, 0) * 0.01;
    assert!((t.channels[0].modeled_power_w as f64 - want).abs() < 1e-6 * want);
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
fn start_state_is_a_parameter() {
    let mut p = params();
    p.ch[0].initial_theta_vc = 70.0;
    p.ch[0].initial_theta_mg = 30.0;
    let mut u = spp3::Unit::new(p);
    let (t, _) = drive(&mut u, [0.0; 4], dt(), U);
    let c = &p.ch[0];
    let want = 40.0 + 70.0 * (1.0 - dt() / c.tau_tvc as f64) + 30.0 * (1.0 - dt() / c.tau_tmg as f64);
    assert!((t.channels[0].ol_temp_c as f64 - want).abs() < 1e-3);
    u.reset();
    let (t, _) = drive(&mut u, [0.0; 4], dt(), U);
    assert!((t.channels[0].ol_temp_c as f64 - want).abs() < 1e-3);
}
