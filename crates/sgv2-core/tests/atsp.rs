//! atsp against the behaviour specification's equations, on synthetic
//! parameters only.

use sgv2_core::atsp::{Params, Slot, Unit};
use sgv2_core::{Io, ProtectionUnit, Telemetry, FAILSAFE_GAIN};

const RATE: u32 = 48_000;

fn params() -> Params {
    let s = Slot {
        channel_mask: 0b1111,
        gain_link: 1,
        amplifier_sensitivity: 10.0,
        rdc: 8.0,
        rth_vc: 30.0,
        tau_vc: 1.0,
        rth_mg: 40.0,
        tau_mg: 20.0,
        ambient: 40.0,
        temperature_limit: 110.0,
        hard_limit_headroom: 8.0,
        attack_ms: 500.0,
        release_ms: 5000.0,
    };
    Params {
        channels: 4,
        sample_rate: RATE,
        block_frames: 512,
        bypass: 0,
        gain_link_all: 0,
        slot: [s, Slot::default(), Slot::default()],
    }
}

fn drive(u: &mut Unit, x: [f32; 4], secs: f64, slice: usize) -> Telemetry {
    let total = (secs * RATE as f64) as usize;
    let ins: Vec<Vec<f32>> = (0..4).map(|c| vec![x[c]; slice]).collect();
    let mut outs = vec![vec![0.0f32; slice]; 4];
    let mut f = 0;
    while f < total {
        let m = slice.min(total - f);
        let i: Vec<&[f32]> = ins.iter().map(|b| &b[..m]).collect();
        let mut o: Vec<&mut [f32]> = outs.iter_mut().map(|b| &mut b[..m]).collect();
        u.process(Io { frames: m, input: &i, sense: &[], output: &mut o });
        f += m;
    }
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    t
}

#[test]
fn steady_temperature_and_power() {
    let mut u = Unit::new(params());
    assert!(!u.is_failsafe());
    let t = drive(&mut u, [0.1; 4], 200.0, 1024);
    // the post-gain predictor takes the unit's own output, which carries the
    // dezipper's fixed point (the reference's +0.052 dB trim)
    let trim = sgv2_core::atsp::NOMINAL_TRIM as f64;
    let p = 100.0 * 0.01 / 8.0 * trim * trim;
    assert!((t.channels[0].modeled_power_w as f64 - p).abs() < 1e-5, "{}", t.channels[0].modeled_power_w);
    let want = 40.0 + p * 70.0;
    assert!((t.channels[0].coil_temp_c as f64 - want).abs() < 0.05, "{}", t.channels[0].coil_temp_c);
}

fn settle(rate: f64, ms: f64) -> f64 {
    1.0 - (0.01f64.ln() / (ms * 1e-3 * rate)).exp()
}

#[test]
fn ceiling_holds_the_limit_and_links_the_slot() {
    let mut u = Unit::new(params());
    let t = drive(&mut u, [0.4, 0.0, 0.0, 0.0], 300.0, 512);
    // SPEC-ATSP 10 at equilibrium: the input-referred coil node is
    // Rvc Pd, the post-gain magnet node Rmg Pd trim^2 g^2, and the request
    // g^2 = ((Tlim - Tamb) - theta_mg_post) / (theta_vc_in + 1) is the target
    // (the post-gain power is the unit's own output's, as it reports it)
    let p_drive: f64 = 100.0 * 0.16 / 8.0;
    let p_post = t.channels[0].modeled_power_w as f64;
    let g2 = (70.0 - 40.0 * p_post) / (30.0 * p_drive + 1.0);
    assert!((t.channels[0].gain as f64 - g2.sqrt()).abs() < 1e-4, "{} {}", t.channels[0].gain, g2.sqrt());
    // the post-gain coil settles just under the limit
    let coil = 40.0 + 70.0 * p_post;
    assert!((t.channels[0].coil_temp_c as f64 - coil).abs() < 0.05, "{} {coil}", t.channels[0].coil_temp_c);
    assert!(t.channels[0].coil_temp_c < 110.0);
    assert_eq!(t.channels[3].gain, t.channels[0].gain, "slot link");
}

#[test]
fn no_forecast_before_the_crossing() {
    // steady state far above the limit, but the coil is still below it
    let mut p = params();
    p.slot[0].tau_vc = 10_000.0;
    p.slot[0].tau_mg = 10_000.0;
    let mut u = Unit::new(p);
    let t = drive(&mut u, [1.0; 4], 5.0, 512);
    assert!(t.channels[0].coil_temp_c < 110.0);
    assert_eq!(t.channels[0].gain, 1.0);
}

#[test]
fn panic_above_limit_plus_headroom() {
    let mut p = params();
    p.slot[0].attack_ms = 30_000.0;
    let mut u = Unit::new(p);
    let (mut low, mut peak) = (1.0f32, 0.0f32);
    for _ in 0..4000 {
        let t = drive(&mut u, [1.0; 4], 0.01, 512);
        peak = peak.max(t.channels[0].coil_temp_c);
        low = low.min(t.channels[0].gain);
    }
    // far slower than the drive heats the coil, the attack alone would let
    // it run away: the panic takes the target to 20 dB down at once, and
    // holds the coil at the hard limit (one window of overshoot)
    assert!(low <= FAILSAFE_GAIN * 1.01, "{low}");
    assert!(peak < 118.5, "{peak}");
}

#[test]
fn settling_coefficients() {
    let a = sgv2_core::atsp::settle_coeff(48_000.0, 800.0);
    assert!((a - settle(48_000.0, 800.0)).abs() < 1e-15);
    // 99 % of a step after the settling time
    let n = (0.8 * 48_000.0) as i32;
    assert!(((1.0 - a).powi(n) - 0.01).abs() < 1e-6);
}

#[test]
fn routing_masks() {
    let mut p = params();
    p.slot[0].channel_mask = 0;
    assert_eq!(p.route()[..4], [0, 0, 0, 0], "a zero A mask keeps the A fallback");
    p.slot[1] = p.slot[0];
    p.slot[1].channel_mask = 0b0110;
    p.slot[0].channel_mask = 0b0010;
    assert_eq!(p.route()[..4], [0, 0, 1, 0], "A wins the overlap, unclaimed go to A");
    p.slot[0].channel_mask = 0xffff;
    p.channels = 1;
    assert!(p.validate().is_ok(), "a wide mask is valid");
    assert!(!Unit::new(p).is_failsafe());
}

#[test]
fn bypass_copies_and_freezes() {
    let mut u = Unit::new(params());
    drive(&mut u, [0.4; 4], 20.0, 512);
    let before = {
        let mut t = Telemetry::new();
        u.telemetry(&mut t);
        t
    };
    assert!(u.set_parameter(0, 1.0));
    let x = [0.123_456_7f32, -0.5, 0.25, 1.5];
    let ins: Vec<Vec<f32>> = (0..4).map(|c| vec![x[c]; 512]).collect();
    let mut outs = vec![vec![0.0f32; 512]; 4];
    for _ in 0..200 {
        let i: Vec<&[f32]> = ins.iter().map(|b| &b[..]).collect();
        let mut o: Vec<&mut [f32]> = outs.iter_mut().map(|b| &mut b[..]).collect();
        u.process(Io { frames: 512, input: &i, sense: &[], output: &mut o });
    }
    // a bit-exact copy of in-range program, held to full scale above it
    let want: Vec<Vec<f32>> = ins.iter().map(|b| b.iter().map(|v| v.clamp(-1.0, 1.0)).collect()).collect();
    assert!(outs == want, "bit-exact copy");
    assert!(u.set_parameter(0, 0.0));
    let after = {
        let mut t = Telemetry::new();
        u.telemetry(&mut t);
        t
    };
    assert_eq!(after.channels[0].coil_temp_c, before.channels[0].coil_temp_c);
    assert_eq!(after.channels[0].gain, before.channels[0].gain);
}

#[test]
fn startup_mute_then_unity() {
    let mut u = Unit::new(params());
    let n = 48_000 / 4;
    let ins: Vec<Vec<f32>> = (0..4).map(|_| vec![0.01f32; n]).collect();
    let mut outs = vec![vec![0.0f32; n]; 4];
    let i: Vec<&[f32]> = ins.iter().map(|b| &b[..]).collect();
    let mut o: Vec<&mut [f32]> = outs.iter_mut().map(|b| &mut b[..]).collect();
    u.process(Io { frames: n, input: &i, sense: &[], output: &mut o });
    assert!(outs[0][..240].iter().all(|&y| y == 0.0));
    assert!(outs[0][241] > 0.0);
    // the dezipper passes as the reference's: up to its 1.0283 overshoot,
    // settling at its +0.052 dB fixed point
    let peak = sgv2_core::atsp::DEZIPPER_PEAK;
    assert!(outs[0].iter().all(|&y| y <= 0.01 * peak));
    assert!(outs[0].iter().any(|&y| y > 0.0102));
    assert!((outs[0][n - 1] / 0.01 - sgv2_core::atsp::NOMINAL_TRIM).abs() < 1e-5, "{}", outs[0][n - 1] / 0.01);
}

#[test]
fn refusals() {
    assert!(Unit::new(Params::default()).is_failsafe());
    let mut p = params();
    p.slot[0].rdc = 0.0;
    assert!(Unit::new(p).is_failsafe());
}

#[test]
fn slicing_invariance() {
    let p = params();
    let render = |slice| {
        let mut u = Unit::new(p);
        let t = drive(&mut u, [0.5, 0.3, 0.2, 0.0], 4.0, slice);
        (t.channels[0].gain, t.channels[0].coil_temp_c)
    };
    // SPEC-ATSP 10: one trajectory per staging family; calls shorter than
    // a window, and whole multiples of the two-window staging buffer
    let r = render(2);
    for s in [1usize, 3, 7, 160, 255] {
        assert_eq!(render(s), r, "slice {s}");
    }
    let r = render(512);
    for s in [1024usize, 2048, 4096] {
        assert_eq!(render(s), r, "slice {s}");
    }
}

#[test]
fn dezipper_fixed_point_is_the_reference_trim() {
    // rig: the dezipper's Float32 fixed point is 1.0060077 at 48 kHz (the
    // "+0.052 dB trim"); the engine caps the applied gain at unity, so read
    // the filter itself through a unit whose output we divide out
    for (rate, want) in [(48_000u32, 1.006_007_7f32), (44_100, 0.999_897_8), (96_000, 1.042_091_8)] {
        let mut p = params();
        p.sample_rate = rate;
        let mut u = Unit::new(p);
        let n = rate as usize / 4;
        let ins: Vec<Vec<f32>> = (0..4).map(|_| vec![0.01f32; n]).collect();
        let mut outs = vec![vec![0.0f32; n]; 4];
        for _ in 0..12 {
            let i: Vec<&[f32]> = ins.iter().map(|b| &b[..]).collect();
            let mut o: Vec<&mut [f32]> = outs.iter_mut().map(|b| &mut b[..]).collect();
            u.process(Io { frames: n, input: &i, sense: &[], output: &mut o });
        }
        assert!((u.dezipper_gain() - want).abs() < 2e-7, "{rate}: {} vs {want}", u.dezipper_gain());
    }
}
