//! lsm2 against the behaviour specification's equations, on synthetic
//! parameters only (no Mac's tuning appears here).

use sgv2_core::lsm2::{self, ChannelParams, Params, Unit, BANDS};
use sgv2_core::{dsp, HandOff, Io, ProtectionUnit, Telemetry};

const RATE: u32 = 48_000;
/// 2 * 120 + 48 frames: the synthetic filterbank stage and look-ahead.
const LATENCY: usize = 288;

/// Every axis on, as the Neo ships; tests switch off what they isolate.
fn params() -> Params {
    let mut p = Params {
        channels: 2,
        sample_rate: RATE,
        enable_mitigation: 1,
        control_sample_peak: 1,
        control_displacement: 1,
        control_velocity: 1,
        control_power: 1,
        control_temperature: 1,
        link_slow_gains: 1,
        model_only: 1,
        attack_time_ms: 2.0,
        fast_release_time_ms: 150.0,
        slow_release_time_ms: 10_000.0,
        low_band_release_time_ms: 150.0,
        filterbank_latency_ms: 2.5,
        speaker_response_lookahead_ms: 1.0,
        power_time_scale_s: [0.002, 0.2, 2.0, 30.0],
        power_hard_limit_w: [50.0, 20.0, 2.0, 1.5],
        power_soft_limit_pct: [100.0; 4],
        max_power_attenuation_db: 4.0,
        low_rail_idle_power_w: 0.01,
        low_rail_power_ratio: 1.1,
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
            disp_rdc: 4.0,
            disp_bl: 0.8,
            disp_mms_kg: 0.00022,
            disp_rms: 0.25,
            disp_kms: 1600.0,
            vel_rdc: 4.0,
            vel_bl: 0.8,
            vel_mms_kg: 0.0002,
            vel_rms: 0.2,
            vel_kms: 1500.0,
            rth_vc: 30.0,
            tau_vc: 2.0,
            rth_mag: 20.0,
            tau_mag: 60.0,
            ambient: 40.0,
            amplitude_limit: 1.0,
            displacement_limit: 0.0005,
            velocity_limit: [0.5, 0.6, 0.9],
            mask_energy_db: [-40.0, -30.0, -20.0, -10.0],
            mask_band_hz: [8000.0, 12000.0],
            mask_attack_s: 0.01,
            mask_release_s: 0.1,
            mask_attack_fast_s: 0.005,
            mask_attack_range_db: 70.0,
            max_velocity_attenuation_db: 18.0,
            velocity_ignore_low_band: 1,
            velocity_ignore_high_band: 1,
            temperature_soft_limit: 110.0,
            temperature_hard_limit: 125.0,
            temperature_headroom: 4.0,
            max_temperature_attenuation_db: 10.0,
            band_center_hz: [60.0, 300.0, 650.0, 1600.0, 5000.0],
            ..ChannelParams::default()
        };
    }
    p
}

fn only(axes: [bool; 5]) -> Params {
    let mut p = params();
    let [a, b, c, d, e] = axes.map(u32::from);
    (p.control_sample_peak, p.control_displacement, p.control_velocity, p.control_power, p.control_temperature) =
        (a, b, c, d, e);
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

fn tel(u: &Unit) -> Telemetry {
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    t
}

fn sine(freq: f64, amp: f32, secs: f64) -> Vec<f32> {
    let n = (secs * RATE as f64) as usize;
    (0..n).map(|k| amp * (2.0 * std::f64::consts::PI * freq * k as f64 / RATE as f64).sin() as f32).collect()
}

#[test]
fn latency_is_two_filterbank_stages_plus_lookahead_for_any_call_size() {
    for slice in [1usize, 7, 31, 32, 33, 96, 127, 160, 256, 512, 4096] {
        let mut u = Unit::new(params());
        assert_eq!(u.latency_frames() as usize, LATENCY);
        let mut x = vec![0.0f32; 2000];
        x[0] = 0.5;
        let out = run(&mut u, &[x.clone(), x], slice);
        let first = out[0].iter().position(|&v| v != 0.0).unwrap();
        assert_eq!(first, LATENCY, "slice {slice}");
        assert_eq!(out[0][LATENCY], 0.5);
    }
}

#[test]
fn quiet_audio_is_the_input_delayed_bit_for_bit() {
    // spec 6.1: every gain at unity bypasses the bank
    let mut u = Unit::new(params());
    let x = sine(1000.0, 0.01, 2.0);
    let y = sine(70.0, 0.02, 2.0);
    let out = run(&mut u, &[x.clone(), y.clone()], 512);
    for k in LATENCY..x.len() {
        assert_eq!(out[0][k], x[k - LATENCY]);
        assert_eq!(out[1][k], y[k - LATENCY]);
    }
}

#[test]
fn filterbank_is_complementary_and_symmetric() {
    let c = params().ch[0].band_center_hz;
    for bw in 0..3 {
        let h = lsm2::filterbank(RATE as f64, &c, bw, 160);
        assert!(h.iter().all(|b| b.len() == 319));
        for j in 0..319 {
            let sum: f64 = (0..BANDS).map(|k| h[k][j] as f64).sum();
            let want = if j == 159 { 1.0 } else { 0.0 };
            assert!((sum - want).abs() < 1e-6, "bw {bw} tap {j}: {sum}");
            for b in &h {
                assert!((b[j] - b[318 - j]).abs() < 1e-7, "symmetric");
            }
        }
        // band 0 passes DC, band 4 blocks it
        let dc: Vec<f64> = h.iter().map(|b| b.iter().map(|&v| v as f64).sum()).collect();
        assert!(dc[0] > 0.99 && dc[4].abs() < 1e-6, "{dc:?}");
    }
}

#[test]
fn displacement_candidate_matches_the_transfer_function() {
    let p = only([false, true, false, false, true]);
    let mut u = Unit::new(p);
    let f = 80.0;
    run(&mut u, &[sine(f, 1.0, 1.0), sine(f, 1.0, 1.0)], 256);
    // |X/U| at f from the continuous model, R = Rdc (module TODO(SPEC))
    let c = &p.ch[0];
    let w = 2.0 * std::f64::consts::PI * f;
    let r = c.disp_rdc as f64;
    let (bl, m, rm, k) = (c.disp_bl as f64, c.disp_mms_kg as f64, c.disp_rms as f64, c.disp_kms as f64);
    let re = r * (k - m * w * w);
    let im = (r * rm + bl * bl) * w;
    let xpk = bl / (re * re + im * im).sqrt() * c.volts_full_scale as f64;
    let want = (c.displacement_limit as f64 / xpk).min(1.0);
    let t = tel(&u);
    let got = t.channels[0].axis_gain[1] as f64;
    assert!((got - want).abs() < 2e-3 * want, "{got} {want}");
    // displacement acts on every band
    let bg = t.channels[0].band_gain;
    assert!(bg.iter().all(|&g| (g as f64 - want).abs() < 2e-3), "{bg:?}");
}

#[test]
fn velocity_spares_the_ignored_outer_bands() {
    let p = only([false, false, true, false, true]);
    let mut u = Unit::new(p);
    run(&mut u, &[sine(1000.0, 1.0, 0.5), sine(1000.0, 1.0, 0.5)], 512);
    let t = tel(&u).channels[0];
    assert!(t.axis_gain[2] < 0.9, "{:?}", t.axis_gain);
    assert_eq!((t.band_gain[0], t.band_gain[4]), (1.0, 1.0));
    assert!(
        t.band_gain[1..4].iter().all(|&g| (g - t.axis_gain[2]).abs() < 1e-2),
        "{:?} {:?}",
        t.band_gain,
        t.axis_gain
    );
}

#[test]
fn power_is_capped_and_linked() {
    let mut p = only([false, false, false, true, false]);
    p.power_hard_limit_w[2] = 0.5;
    let mut u = Unit::new(p);
    let loud = sine(1000.0, 1.0, 6.0);
    let quiet = vec![0.0f32; loud.len()];
    run(&mut u, &[loud, quiet], 512);
    let t = tel(&u);
    let floor = 10f32.powf(-4.0 / 20.0);
    assert!((t.channels[0].axis_gain[3] - floor).abs() < 1e-4, "{}", t.channels[0].axis_gain[3]);
    assert!((t.global.power_gain - floor).abs() < 1e-4);
    // the quiet channel takes the linked slow gain
    assert!((t.channels[1].gain - t.channels[0].gain).abs() < 1e-6);
}

#[test]
fn power_averages_are_exponential_from_idle() {
    // spec 6.1 anchor: silence, idle 0.018 W, one 32-frame substep
    let mut p = only([false, false, false, true, false]);
    p.low_rail_idle_power_w = 0.018;
    p.power_time_scale_s = [0.001, 0.1, 1.0, 60.0];
    let mut u = Unit::new(p);
    run(&mut u, &[vec![0.0; 32], vec![0.0; 32]], 32);
    let want = [0.0087584909, 0.00011960088, 0.0000119960005, 0.00000019999888];
    let mut rec = HandOff::EMPTY;
    assert!(u.save_state(&mut rec));
    for (j, w) in want.iter().enumerate() {
        assert!((rec.node[0][2 + j] - w).abs() <= 1e-6 * w, "{j}: {}", rec.node[0][2 + j]);
    }
}

#[test]
fn temperature_knee_is_linear_in_db() {
    let g = |t| lsm2::temperature_gain(t, 5.0, 55.0, 70.0, 12.0);
    assert_eq!(g(40.0), 1.0);
    // 69 C + 5 headroom is above the 70 C hard limit: the 12 dB cap
    assert!((20.0 * g(69.0).log10() + 12.0).abs() < 1e-9);
    assert!((20.0 * g(57.5).log10() + 6.0).abs() < 1e-9);
    // a step when the limits are equal, the cap when they are reversed
    assert_eq!(lsm2::temperature_gain(49.0, 5.0, 55.0, 55.0, 12.0), 1.0);
    assert!(lsm2::temperature_gain(50.0, 5.0, 55.0, 55.0, 12.0) < 0.26);
    assert!(lsm2::temperature_gain(0.0, 5.0, 70.0, 55.0, 12.0) < 0.26);
    assert!(lsm2::temperature_gain(f64::NAN, 5.0, 55.0, 70.0, 12.0) < 0.26);
}

#[test]
fn mask_curve_is_piecewise() {
    let k = [17.4817867, 29.8765697, 42.0877037, 45.0321045];
    let v = [0.600095034, 0.702066243, 1.09745789];
    assert_eq!(lsm2::mask_curve(0.0, &k, &v), v[0]);
    assert!((lsm2::mask_curve((k[0] + k[1]) / 2.0, &k, &v) - (v[0] + v[1]) / 2.0).abs() < 1e-12);
    assert_eq!(lsm2::mask_curve(35.0, &k, &v), v[1]);
    assert_eq!(lsm2::mask_curve(100.0, &k, &v), v[2]);
}

#[test]
fn temperature_heats_and_takes_the_knee() {
    let p = only([false, false, false, false, true]);
    let mut u = Unit::new(p);
    let x = sine(150.0, 1.0, 40.0);
    run(&mut u, &[x.clone(), x], 1024);
    let t = tel(&u).channels[0];
    assert!(t.coil_temp_c + 4.0 >= 110.0, "{}", t.coil_temp_c);
    let want = lsm2::temperature_gain(t.coil_temp_c as f64, 4.0, 110.0, 125.0, 10.0) as f32;
    assert!((t.axis_gain[4] - want).abs() < 2e-3, "{} {want}", t.axis_gain[4]);
    assert!(t.gain < 1.0);
}

#[test]
fn refused_configurations_are_the_failsafe() {
    let mut p = params();
    p.model_only = 0;
    assert!(Unit::new(p).is_failsafe());
    let mut p = params();
    p.power_soft_limit_pct[2] = 90.0;
    assert!(Unit::new(p).is_failsafe());
    let mut p = params();
    p.ch[1].band_center_hz[2] = 100.0;
    assert!(Unit::new(p).is_failsafe());
    let mut p = params();
    p.ch[0].pressure_taps = 7;
    assert!(Unit::new(p).is_failsafe());
    let mut p = params();
    p.ch[0].velocity_limit = [f32::NAN; 3];
    assert!(Unit::new(p).is_failsafe());
    assert!(Unit::new(Params::default()).is_failsafe());
}

#[test]
fn slicing_does_not_change_the_audio() {
    let x = sine(200.0, 1.2, 3.0);
    let y = sine(3000.0, 0.7, 3.0);
    let r = run(&mut Unit::new(params()), &[x.clone(), y.clone()], 256);
    for slice in [1usize, 7, 160, 1000, 4096] {
        assert!(run(&mut Unit::new(params()), &[x.clone(), y.clone()], slice) == r, "slice {slice}");
    }
}

#[test]
fn runtime_budgets_replace_the_preset_limits() {
    let mut p = only([false, false, false, true, false]);
    p.runtime_budget_present = 1;
    p.runtime_budget_w = [50.0, 0.4, 50.0, 50.0];
    let mut u = Unit::new(p);
    let loud = sine(1000.0, 1.0, 3.0);
    run(&mut u, &[loud.clone(), loud], 512);
    let t = tel(&u);
    assert_eq!(t.global.power_hard_limit_w, [50.0, 0.4, 50.0, 50.0]);
    let floor = 10f32.powf(-4.0 / 20.0);
    // the 0.4 W runtime 200 ms budget binds at the cap
    assert!((t.channels[0].axis_gain[3] - floor).abs() < 1e-4, "{}", t.channels[0].axis_gain[3]);
}

#[test]
fn masking_raises_the_velocity_limit() {
    let mut p = only([false, false, true, false, true]);
    let mut fir = [0.0f32; lsm2::PRESSURE_TAPS];
    fir[0] = 1.0;
    for c in 0..2 {
        p.ch[c].pressure_taps = lsm2::PRESSURE_TAPS as u32;
        p.ch[c].pressure_fir = fir;
    }
    let quiet: Vec<f32> = sine(1000.0, 0.5, 0.5);
    let masked: Vec<f32> = quiet.iter().zip(sine(10_000.0, 0.3, 0.5)).map(|(a, b)| a + b).collect();
    let mut u = Unit::new(p);
    run(&mut u, &[quiet.clone(), quiet], 512);
    let unmasked = tel(&u).channels[0].velocity_limit_m_s;
    let mut v = Unit::new(p);
    run(&mut v, &[masked.clone(), masked], 512);
    let t = tel(&v).channels[0];
    assert_eq!(unmasked, 0.5);
    assert!(t.velocity_limit_m_s > 0.6 && t.mask_db.is_finite(), "{t:?}");
}

#[test]
fn neon_dot_matches_the_scalar_reference() {
    // the documented bound: 2e-6 of the sum of |products|
    let mut s = 1u32;
    let mut r = || {
        s = s.wrapping_mul(1664525).wrapping_add(1013904223);
        (s >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0
    };
    for n in [1usize, 15, 16, 17, 239, 319, 1024] {
        let a: Vec<f32> = (0..n).map(|_| r()).collect();
        let b: Vec<f32> = (0..n).map(|_| r()).collect();
        let abs: f64 = a.iter().zip(&b).map(|(x, y)| (x * y).abs() as f64).sum();
        let d = (dsp::dot_f32(&a, &b) - dsp::dot_f32_scalar(&a, &b)).abs() as f64;
        assert!(d <= 2e-6 * abs.max(1e-30), "{n}: {d} of {abs}");
    }
}

#[test]
fn a_bad_hand_off_record_leaves_the_reset_state() {
    let mut u = Unit::new(params());
    let x = sine(150.0, 1.0, 2.0);
    run(&mut u, &[x.clone(), x], 512);
    let mut rec = HandOff::EMPTY;
    assert!(u.save_state(&mut rec));
    rec.node[0][0] = f64::NAN;
    let mut v = Unit::new(params());
    assert!(!v.restore_state(&rec, 0.0));
    let q = sine(1000.0, 0.01, 0.1);
    let o = run(&mut v, &[q.clone(), q.clone()], 512);
    assert_eq!(o[0][LATENCY..], q[..q.len() - LATENCY]);
}

#[test]
fn band_gains_never_raise_any_frequency() {
    // zero-phase, nonnegative, complementary bands: |sum g_k H_k(f)| <= 1
    // on the design grid; the 319-tap truncation ripples between grid
    // points, bounded here at +0.17 dB (measured +0.14)
    let c = params().ch[0].band_center_hz;
    let h = lsm2::filterbank(RATE as f64, &c, 0, 160);
    let gains = [[1.0, 0.3, 0.3, 0.3, 1.0], [0.1, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 0.5, 1.0, 0.2]];
    for g in gains {
        for f in (10..24_000).step_by(37) {
            let w = 2.0 * std::f64::consts::PI * f as f64 / RATE as f64;
            let (mut re, mut im) = (0.0, 0.0);
            for k in 0..BANDS {
                for (j, &t) in h[k].iter().enumerate() {
                    let ph = w * (j as f64 - 159.0);
                    re += g[k] * t as f64 * ph.cos();
                    im -= g[k] * t as f64 * ph.sin();
                }
            }
            assert!((re * re + im * im).sqrt() <= 1.02, "{g:?} {f} Hz: {}", (re * re + im * im).sqrt());
        }
    }
}

#[test]
fn controller_cadence_is_a_time() {
    // 25G83 rig: 256 frames at 48 kHz, 512 at 96 kHz (5.333 ms)
    assert_eq!(lsm2::controller_frames(48_000), 256);
    assert_eq!(lsm2::controller_frames(96_000), 512);
    assert_eq!(lsm2::controller_frames(44_100), 232);
}
