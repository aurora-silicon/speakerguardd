//! lsm2 against the behaviour specification's equations, on synthetic
//! parameters only (no Mac's tuning appears here).

use sgv2_core::lsm2::{ChannelParams, Params, Unit};
use sgv2_core::{Io, ProtectionUnit, Telemetry};

const RATE: u32 = 48_000;

fn params() -> Params {
    let mut p = Params {
        channels: 2,
        sample_rate: RATE,
        enable_mitigation: 1,
        link_slow_gains: 1,
        model_only: 1,
        attack_time_ms: 2.0,
        fast_release_time_ms: 150.0,
        slow_release_time_ms: 10_000.0,
        filterbank_latency_ms: 2.5,
        speaker_response_lookahead_ms: 1.0,
        power_time_scale_s: [0.002, 0.2, 2.0, 30.0],
        power_hard_limit_w: [50.0, 20.0, 2.0, 1.5],
        power_soft_limit_pct: [100.0; 4],
        max_power_attenuation_db: 4.0,
        low_rail_idle_power_w: 0.0,
        low_rail_power_ratio: 1.0,
        ..Params::default()
    };
    for c in 0..2 {
        p.ch[c] = ChannelParams {
            source: c as u32,
            volts_full_scale: 6.0,
            parasitic_resistance: 0.2,
            rdc: 4.0,
            bl: 0.8,
            mms_kg: 0.0002,
            rms: 0.2,
            kms: 1500.0,
            disp_bl: 0.8,
            disp_mms_kg: 0.00022,
            disp_rms: 0.25,
            disp_kms: 1600.0,
            rth_vc: 30.0,
            tau_vc: 2.0,
            rth_mag: 20.0,
            tau_mag: 60.0,
            ambient: 40.0,
            amplitude_limit: 1.0,
            displacement_limit: 0.0005,
            velocity_limit0: 0.5,
            max_velocity_attenuation_db: 18.0,
            temperature_soft_limit: 110.0,
            temperature_hard_limit: 125.0,
            temperature_headroom: 4.0,
            max_temperature_attenuation_db: 10.0,
        };
    }
    p
}

fn run(u: &mut Unit, x: &[Vec<f32>], slice: usize) -> Vec<Vec<f32>> {
    let n = x[0].len();
    let mut out = vec![vec![0.0f32; n]; x.len()];
    let mut f = 0;
    while f < n {
        let m = slice.min(n - f);
        let ins: Vec<&[f32]> = x.iter().map(|b| &b[f..f + m]).collect();
        let mut outs: Vec<&mut [f32]> = out.iter_mut().map(|b| &mut b[f..f + m]).collect();
        u.process(Io { frames: m, input: &ins, sense: &[], output: &mut outs });
        f += m;
    }
    out
}

fn sine(freq: f64, amp: f32, secs: f64) -> Vec<f32> {
    let n = (secs * RATE as f64) as usize;
    (0..n).map(|k| amp * (2.0 * std::f64::consts::PI * freq * k as f64 / RATE as f64).sin() as f32).collect()
}

#[test]
fn latency_is_two_filterbank_stages_plus_lookahead_for_any_call_size() {
    let mut p = params();
    p.enable_mitigation = 0;
    for slice in [1usize, 7, 31, 32, 33, 96, 127, 160, 256, 512, 4096] {
        let mut u = Unit::new(p);
        assert_eq!(u.latency_frames(), 2 * 120 + 48);
        let mut x = vec![0.0f32; 2000];
        x[0] = 0.5;
        let out = run(&mut u, &[x.clone(), x], slice);
        let first = out[0].iter().position(|&v| v != 0.0).unwrap();
        assert_eq!(first, 288, "slice {slice}");
        assert_eq!(out[0][288], 0.5);
    }
}

#[test]
fn disabled_axes_and_quiet_audio_are_unity() {
    let p = params();
    let mut u = Unit::new(p);
    let x = sine(1000.0, 0.01, 2.0);
    let out = run(&mut u, &[x.clone(), x.clone()], 512);
    for k in 288..x.len() {
        assert!((out[0][k] - x[k - 288]).abs() < 1e-7);
    }
}

#[test]
fn displacement_candidate_matches_the_transfer_function() {
    let mut p = params();
    p.control_displacement = 1;
    let mut u = Unit::new(p);
    let f = 80.0;
    let x = sine(f, 1.0, 1.0);
    run(&mut u, &[x.clone(), x], 256);
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    // |X/U| at f from the continuous model
    let c = &p.ch[0];
    let w = 2.0 * std::f64::consts::PI * f;
    let r = (c.rdc + c.parasitic_resistance) as f64;
    let (bl, m, rm, k) = (c.disp_bl as f64, c.disp_mms_kg as f64, c.disp_rms as f64, c.disp_kms as f64);
    let re = r * (k - m * w * w);
    let im = (r * rm + bl * bl) * w;
    let xpk = bl / (re * re + im * im).sqrt() * c.volts_full_scale as f64;
    let want = (c.displacement_limit as f64 / xpk).min(1.0);
    let got = t.channels[0].axis_gain[1] as f64;
    assert!((got - want).abs() < 2e-3 * want, "{got} {want}");
    assert!(t.channels[0].gain <= got as f32 + 1e-3);
}

#[test]
fn power_candidate_is_capped_and_linked() {
    let mut p = params();
    p.control_power = 1;
    p.power_hard_limit_w[2] = 0.5;
    let mut u = Unit::new(p);
    let loud = sine(1000.0, 1.0, 6.0);
    let quiet = vec![0.0f32; loud.len()];
    run(&mut u, &[loud, quiet], 512);
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    let floor = 10f32.powf(-4.0 / 20.0);
    assert!((t.channels[0].axis_gain[3] - floor).abs() < 1e-4, "{}", t.channels[0].axis_gain[3]);
    // the quiet channel takes the linked slow gain
    assert!((t.channels[1].gain - t.channels[0].gain).abs() < 1e-6);
}

#[test]
fn temperature_stand_in_takes_the_cap_at_the_soft_limit() {
    let mut p = params();
    p.control_temperature = 1;
    let mut u = Unit::new(p);
    let x = sine(150.0, 1.0, 40.0);
    run(&mut u, &[x.clone(), x], 1024);
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    assert!(t.channels[0].coil_temp_c + 4.0 >= 110.0, "{}", t.channels[0].coil_temp_c);
    assert!((t.channels[0].axis_gain[4] - 10f32.powf(-10.0 / 20.0)).abs() < 1e-6);
}

#[test]
fn refused_configurations_are_the_failsafe() {
    let mut p = params();
    p.model_only = 0;
    assert!(Unit::new(p).is_failsafe());
    let mut p = params();
    p.power_soft_limit_pct[2] = 90.0;
    assert!(Unit::new(p).is_failsafe());
    assert!(Unit::new(Params::default()).is_failsafe());
}

#[test]
fn slicing_does_not_change_the_audio() {
    let mut p = params();
    p.control_sample_peak = 1;
    p.control_displacement = 1;
    p.control_velocity = 1;
    p.control_power = 1;
    p.control_temperature = 1;
    let x = sine(200.0, 1.2, 3.0);
    let y = sine(3000.0, 0.7, 3.0);
    let r = run(&mut Unit::new(p), &[x.clone(), y.clone()], 256);
    for slice in [1usize, 7, 160, 1000, 4096] {
        assert!(run(&mut Unit::new(p), &[x.clone(), y.clone()], slice) == r, "slice {slice}");
    }
}
