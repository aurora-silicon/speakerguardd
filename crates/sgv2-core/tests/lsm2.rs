//! lsm2 against the behaviour specification's equations, on synthetic
//! parameters only (no Mac's tuning appears here).

use sgv2_core::lsm2::{self, ChannelParams, Params, Unit, BANDS};
use sgv2_core::{dsp, HandOff, Io, ProtectionUnit, Telemetry};

const RATE: u32 = 48_000;
/// 2 * 96 + 64 frames (the synthetic filterbank stage and look-ahead) plus
/// the controller frame the unit buffers.
const LATENCY: usize = 2 * 96 + 64 + 256;

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
        filterbank_latency_ms: 2.0,
        speaker_response_lookahead_ms: 4.0 / 3.0,
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
            mask_energy_db: [0.0, 10.0, 20.0, 30.0],
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
fn velocity_holds_the_full_band_limit_on_every_band() {
    // the velocity candidate acts on every band, as the displacement one:
    // water filling over the non-ignored bands left the full-band velocity
    // above the limit (25G83 thermal_ramp_200: 1.19x where macOS holds 0.97x)
    let p = only([false, false, true, false, true]);
    let mut u = Unit::new(p);
    run(&mut u, &[sine(1000.0, 1.0, 0.5), sine(1000.0, 1.0, 0.5)], 512);
    let t = tel(&u).channels[0];
    assert!(t.axis_gain[2] < 0.9, "{:?}", t.axis_gain);
    for g in t.band_gain {
        // the 0.99 law: at or just below the candidate
        assert!(g <= t.axis_gain[2] + 1e-6 && g >= 0.985 * t.axis_gain[2], "{:?} {:?}", t.band_gain, t.axis_gain);
    }
}

#[test]
fn power_is_one_gain_on_the_total_budget_without_a_cap() {
    // rig law 7: g = sqrt((B - N idle) / sum(Phat - idle)), B the total
    // budget, Phat the unattenuated average; MaxPowerAttenuation (4 dB
    // here) does not bound it
    let mut p = only([false, false, false, true, false]);
    p.control_temperature = 1;
    p.power_hard_limit_w[2] = 0.3;
    p.ch[0].temperature_soft_limit = 1000.0;
    p.ch[1].temperature_soft_limit = 1000.0;
    p.ch[0].temperature_hard_limit = 1001.0;
    p.ch[1].temperature_hard_limit = 1001.0;
    let loud = sine(1000.0, 1.0, 6.0);
    let quiet = vec![0.0f32; loud.len()];
    // the unattenuated 1 s average, from a unit with the power axis off
    let mut q = p;
    q.control_power = 0;
    let mut r = Unit::new(q);
    run(&mut r, &[loud.clone(), quiet.clone()], 512);
    let mut rec = HandOff::EMPTY;
    assert!(r.save_state(&mut rec));
    let idle = p.low_rail_idle_power_w as f64;
    let demand = (rec.node[0][4] - idle) + (rec.node[1][4] - idle);
    let want = ((0.3 - 2.0 * idle) / demand).sqrt();
    let mut u = Unit::new(p);
    run(&mut u, &[loud, quiet], 512);
    let t = tel(&u);
    let got = t.channels[0].axis_gain[3] as f64;
    assert!(want < 10f64.powf(-4.0 / 20.0), "the case must go past the old cap: {want}");
    assert!((got - want).abs() < 2e-3 * want, "{got} {want}");
    assert!((t.global.power_gain as f64 - got).abs() < 1e-6);
    // the quiet channel takes the same gain
    assert!((t.channels[1].gain - t.channels[0].gain).abs() < 1e-6);
}

#[test]
fn power_averages_are_exponential_from_idle() {
    // spec 6.1: silence, idle 0.018 W; one controller frame is eight 32-frame
    // substeps of each matched one-pole, starting from 0 W. The first silent
    // substep's values are the spec's anchors.
    let mut p = only([false, false, false, true, false]);
    p.low_rail_idle_power_w = 0.018;
    p.power_time_scale_s = [0.001, 0.1, 1.0, 60.0];
    let mut u = Unit::new(p);
    run(&mut u, &[vec![0.0; 256], vec![0.0; 256]], 256);
    let first = [0.0087584909, 0.00011960088, 0.0000119960005, 0.00000019999888];
    let mut rec = HandOff::EMPTY;
    assert!(u.save_state(&mut rec));
    for (j, (&tau, w1)) in p.power_time_scale_s.iter().zip(first).enumerate() {
        let b = (-32.0 / (48_000.0 * tau as f64)).exp();
        assert!(((1.0 - b) * 0.018 - w1).abs() <= 1e-6 * w1, "{j}");
        let want = 0.018 * (1.0 - b.powi(8));
        assert!((rec.node[0][2 + j] - want).abs() <= 1e-6 * want, "{j}: {} {want}", rec.node[0][2 + j]);
    }
}

#[test]
fn temperature_candidate_engages_only_above_the_soft_limit_and_releases_in_silence() {
    // spec 15.5: the axis's own predictor; strictly above the soft limit the
    // candidate regulates toward the hard limit, in silence it releases
    let p = only([false, false, false, false, true]);
    let mut u = Unit::new(p);
    let x = sine(150.0, 0.5, 2.0);
    run(&mut u, &[x.clone(), x], 512);
    let t = tel(&u).channels[0];
    assert!(t.coil_temp_c < 110.0, "{}", t.coil_temp_c);
    assert_eq!(t.axis_gain[4], 1.0);
    let x = sine(150.0, 1.0, 40.0);
    run(&mut u, &[x.clone(), x], 512);
    let hot = tel(&u).channels[0];
    assert!(hot.axis_gain[4] < 1.0 && hot.gain < 1.0, "{hot:?}");
    let z = vec![0.0f32; 48_000 * 20];
    run(&mut u, &[z.clone(), z], 512);
    let cool = tel(&u).channels[0];
    assert!(cool.axis_gain[4] > hot.axis_gain[4], "{} {}", cool.axis_gain[4], hot.axis_gain[4]);
}

#[test]
fn mask_curve_is_piecewise() {
    // synthetic knots and plateaus
    let k = [10.0, 20.0, 30.0, 40.0];
    let v = [0.5, 0.6, 0.9];
    assert_eq!(lsm2::mask_curve(0.0, &k, &v), v[0]);
    assert!((lsm2::mask_curve((k[0] + k[1]) / 2.0, &k, &v) - (v[0] + v[1]) / 2.0).abs() < 1e-12);
    assert_eq!(lsm2::mask_curve(25.0, &k, &v), v[1]);
    assert_eq!(lsm2::mask_curve(100.0, &k, &v), v[2]);
}

#[test]
fn temperature_holds_the_coil_below_the_hard_ceiling() {
    // the slow gain's ceiling: the monitor never passes hard + headroom
    let p = only([false, false, false, false, true]);
    let mut u = Unit::new(p);
    let x = sine(150.0, 1.0, 60.0);
    let mut peak = 0.0f32;
    for chunk in x.chunks(48_000) {
        let c = chunk.to_vec();
        run(&mut u, &[c.clone(), c], 512);
        peak = peak.max(tel(&u).channels[0].coil_temp_c);
    }
    assert!(peak <= 125.0 + 4.0 + 0.05, "{peak}");
    assert!(tel(&u).channels[0].gain < 1.0);
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
    // the 0.4 W runtime 200 ms budget binds (the preset's would not)
    assert!(t.channels[0].axis_gain[3] < 0.5, "{}", t.channels[0].axis_gain[3]);
}

#[test]
fn masking_raises_the_velocity_limit() {
    let p = with_identity_pressure(only([false, false, true, false, true]));
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
    // points, bounded here at +0.26 dB (measured +0.20 with the rig-law bank)
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
            assert!((re * re + im * im).sqrt() <= 1.03, "{g:?} {f} Hz: {}", (re * re + im * im).sqrt());
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

#[test]
fn peak_reserves_the_pilot_amplitude() {
    // rig law 6: (AmplitudeLimit - pilot) / peak; -36 dB on a 1.0012 sine
    let mut p = only([true, false, false, false, true]);
    for c in 0..2 {
        p.ch[c].pilot_reserve = 10f32.powf(-36.0 / 20.0);
    }
    let mut u = Unit::new(p);
    let x: Vec<f32> = sine(1000.0, 1.0012, 0.2);
    run(&mut u, &[x.clone(), x], 512);
    let g = tel(&u).channels[0].axis_gain[0];
    let pk = sine(1000.0, 1.0012, 0.2).iter().fold(0f32, |m, v| m.max(v.abs()));
    let want = (1.0 - 10f32.powf(-36.0 / 20.0)) / pk;
    assert!((g - want).abs() < 1e-6, "{g} {want}");
}

#[test]
fn filterbank_matches_the_rig_construction_anchors() {
    // rig law 10 on the spec's AID44 left centres: centre tap 0.00625 and
    // -3/-6/-12 dB at about 108/150/204 Hz for band 0 (150 Hz grid)
    let c = [62.23778f32, 292.82724, 649.9541, 1634.0066, 4953.1763];
    let h = lsm2::filterbank(RATE as f64, &c, 0, 160);
    assert!((h[0][159] - 0.00625).abs() < 1e-6, "{}", h[0][159]);
    let mag = |f: f64| {
        let w = 2.0 * std::f64::consts::PI * f / RATE as f64;
        let (re, im) = h[0].iter().enumerate().fold((0.0, 0.0), |(r, i), (j, &t)| {
            let ph = w * (j as f64 - 159.0);
            (r + t as f64 * ph.cos(), i - t as f64 * ph.sin())
        });
        20.0 * (re * re + im * im).sqrt().log10()
    };
    for (f, db) in [(107.98, -3.0), (149.88, -6.0), (203.54, -12.0)] {
        assert!((mag(f) - db).abs() < 0.05, "{f} Hz: {} dB", mag(f));
    }
}

/// A synthetic masker lattice (not the macOS one): 64 single bins, then
/// widening buckets to bin 321.
fn synthetic_lattice() -> [u32; 100] {
    let mut l = [0u32; 100];
    for (i, b) in l.iter_mut().enumerate() {
        *b = if i < 64 { i as u32 + 1 } else { 64 + ((i as u32 - 63) * 257).div_ceil(36) };
    }
    l[99] = 321;
    l
}

fn with_identity_pressure(mut p: Params) -> Params {
    for c in 0..2 {
        p.ch[c].pressure_taps = lsm2::PRESSURE_TAPS as u32;
        p.ch[c].pressure_fir = [0.0; lsm2::PRESSURE_TAPS];
        p.ch[c].pressure_fir[0] = 1.0;
    }
    p.mask_lattice_len = 100;
    p.mask_lattice = synthetic_lattice();
    p
}

#[test]
fn without_an_extracted_lattice_there_is_no_masking_credit() {
    // the fallback is at least as protective as any lattice: the limit stays
    // at its lowest anchor however loud the mask band is
    let mut p = with_identity_pressure(only([false, false, true, false, true]));
    p.mask_lattice_len = 0;
    let mut u = Unit::new(p);
    let x: Vec<f32> = sine(1000.0, 0.5, 0.5).iter().zip(sine(10_000.0, 0.3, 0.5)).map(|(a, b)| a + b).collect();
    run(&mut u, &[x.clone(), x], 512);
    let t = tel(&u).channels;
    assert_eq!(t[0].velocity_limit_m_s, 0.5);
}

#[test]
fn a_tone_below_the_mask_band_does_not_raise_the_velocity_limit() {
    // residuals 3: only the mask band's spectral median masks; a loud 1 kHz
    // tone leaves the limit at its unmasked anchor, a 10 kHz masker lifts it
    let p = with_identity_pressure(only([false, false, true, false, true]));
    let mut u = Unit::new(p);
    run(&mut u, &[sine(1000.0, 1.0, 0.5), sine(1000.0, 1.0, 0.5)], 512);
    let t = tel(&u).channels;
    assert_eq!(t[0].velocity_limit_m_s, 0.5, "{}", t[0].mask_db);
    let mut u = Unit::new(p);
    let x: Vec<f32> = sine(1000.0, 0.5, 0.5).iter().zip(sine(10_000.0, 0.3, 0.5)).map(|(a, b)| a + b).collect();
    run(&mut u, &[x.clone(), x], 512);
    let t = tel(&u).channels;
    assert!(t[0].velocity_limit_m_s > 0.6, "{} {}", t[0].velocity_limit_m_s, t[0].mask_db);
}

#[test]
fn the_velocity_limit_is_one_for_the_pair() {
    // residuals 3: both channels' curves take the largest level of the pair
    let p = with_identity_pressure(only([false, false, true, false, true]));
    let mut u = Unit::new(p);
    let loud: Vec<f32> = sine(1000.0, 0.5, 0.5).iter().zip(sine(10_000.0, 0.3, 0.5)).map(|(a, b)| a + b).collect();
    run(&mut u, &[loud, sine(1000.0, 0.5, 0.5)], 512);
    let t = tel(&u).channels;
    assert_eq!(t[0].velocity_limit_m_s, t[1].velocity_limit_m_s);
    assert!(t[0].velocity_limit_m_s > 0.6, "{}", t[0].velocity_limit_m_s);
}

#[test]
fn the_power_candidate_is_input_referred() {
    // rig law 4: the velocity axis cutting the audio does not lower the power
    // the power axis sees
    let mut a = only([false, false, false, true, true]);
    a.power_hard_limit_w = [50.0, 0.5, 50.0, 50.0];
    let mut b = a;
    b.control_velocity = 1;
    let x = sine(1000.0, 1.0, 0.5);
    let mut ua = Unit::new(a);
    let mut ub = Unit::new(b);
    run(&mut ua, &[x.clone(), x.clone()], 512);
    run(&mut ub, &[x.clone(), x], 512);
    let (ta, tb) = (tel(&ua).channels[0], tel(&ub).channels[0]);
    assert!(tb.axis_gain[2] < 0.9 && ta.axis_gain[3] < 1.0, "{:?} {:?}", ta.axis_gain, tb.axis_gain);
    assert_eq!(ta.axis_gain[3], tb.axis_gain[3]);
}

#[test]
fn generic_flags_switch_a_model_to_the_family_row_at_run_time() {
    // spec 15.1: the flag selects the embedded family row; the block's own
    // field is kept for when the flag is cleared again
    let mut p = params();
    for c in 0..2 {
        p.ch[c].family = [3.0, 0.5, 0.0001, 0.1, 1000.0];
        p.ch[c].explicit = [
            [p.ch[c].rdc, p.ch[c].bl, p.ch[c].mms_kg, p.ch[c].rms, p.ch[c].kms],
            [p.ch[c].disp_rdc, p.ch[c].disp_bl, p.ch[c].disp_mms_kg, p.ch[c].disp_rms, p.ch[c].disp_kms],
            [p.ch[c].vel_rdc, p.ch[c].vel_bl, p.ch[c].vel_mms_kg, p.ch[c].vel_rms, p.ch[c].vel_kms],
        ];
    }
    let vel = |m: u32, ty: u32, k: u32| 3072 + 1024 * (3 * m + ty) + k;
    assert!(p.set(vel(3, 0, 0), 1.0));
    assert_eq!(p.ch[0].vel_rdc, 3.0);
    assert_eq!(p.ch[1].vel_bl, 0.8, "the resonator flag is separate");
    assert!(p.set(vel(3, 0, 1), 6.0), "an own-field write while generic");
    assert_eq!(p.ch[0].vel_rdc, 3.0);
    assert!(p.set(vel(3, 0, 0), 0.0));
    assert_eq!(p.ch[0].vel_rdc, 6.0);
    // the impedance block is accepted and inert
    assert!(p.set(vel(0, 0, 1), 9.0));
    assert_eq!(p.ch[0].family[0], 3.0);
}
