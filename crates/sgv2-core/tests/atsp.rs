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
    let p = 100.0 * 0.01 / 8.0;
    assert!((t.channels[0].modeled_power_w as f64 - p).abs() < 1e-6);
    let want = 40.0 + p * 70.0;
    assert!((t.channels[0].coil_temp_c as f64 - want).abs() < 0.05, "{}", t.channels[0].coil_temp_c);
}

#[test]
fn ceiling_holds_the_limit_and_links_the_slot() {
    let mut u = Unit::new(params());
    let t = drive(&mut u, [0.4, 0.0, 0.0, 0.0], 300.0, 512);
    let p_hold = 70.0 / 70.0;
    let p_drive: f64 = 100.0 * 0.16 / 8.0;
    let want = (p_hold / p_drive).sqrt();
    assert!((t.channels[0].gain as f64 - want).abs() < 3e-3, "{} {want}", t.channels[0].gain);
    assert!((t.channels[0].coil_temp_c - 110.0).abs() < 0.5);
    assert_eq!(t.channels[3].gain, t.channels[0].gain);
}

#[test]
fn panic_above_limit_plus_headroom() {
    let mut p = params();
    p.slot[0].attack_ms = 30_000.0;
    let mut u = Unit::new(p);
    let mut seen = false;
    for _ in 0..400 {
        let t = drive(&mut u, [1.0; 4], 0.1, 512);
        if t.channels[0].coil_temp_c > 118.0 {
            seen = true;
            assert!(t.channels[0].gain <= FAILSAFE_GAIN + 1e-6);
        }
    }
    assert!(seen);
}

#[test]
fn refusals() {
    let mut p = params();
    p.bypass = 1;
    assert!(Unit::new(p).is_failsafe());
    let mut p = params();
    p.slot[0].channel_mask = 0b0111;
    assert!(Unit::new(p).is_failsafe(), "channel 3 unprotected");
    assert!(Unit::new(Params::default()).is_failsafe());
}

#[test]
fn slicing_invariance() {
    let p = params();
    let render = |slice| {
        let mut u = Unit::new(p);
        let t = drive(&mut u, [0.5, 0.3, 0.2, 0.0], 20.0, slice);
        (t.channels[0].gain, t.channels[0].coil_temp_c)
    };
    let r = render(512);
    for s in [1usize, 7, 160, 1000, 4096] {
        assert_eq!(render(s), r, "slice {s}");
    }
}

#[test]
fn host_block_mode_updates_per_call() {
    let mut p = params();
    p.block_frames = 0;
    let mut u = Unit::new(p);
    assert!(!u.is_failsafe());
    let t = drive(&mut u, [0.1; 4], 200.0, 480);
    let pw = 100.0 * 0.01 / 8.0;
    assert!((t.channels[0].modeled_power_w as f64 - pw).abs() < 1e-6);
    assert!((t.channels[0].coil_temp_c as f64 - (40.0 + pw * 70.0)).abs() < 0.05);
}
