//! Safety review of PR #31. Finding 1 (critical): huge finite samples must not overflow the model meters,
//! unlock full scale or mute a unit.
//! Synthetic parameters only; the test failed on 5764482.

mod review_common;

use review_common::*;
use sgv2_core::{atsp, lsm2, spp3};

/// Finding 1 (critical): a block of huge finite samples overflowed the f32
/// square sums, the thermal state went inf/NaN and spp3 then passed full
/// scale at unity gain.
#[test]
fn spp3_huge_finite_samples_never_unlock_full_scale() {
    // mean output over the last 10 s of 30 s at full scale
    let tail = |u: &mut spp3::Unit| {
        let (mut sum, mut n) = (0.0f64, 0);
        for k in 0..secs(30.0) {
            let o = blk(u, 1.0);
            if k >= secs(20.0) {
                sum += o as f64;
                n += 1;
            }
        }
        sum / n as f64
    };
    for ff in [0.5f32, 0.0] {
        let mut p = spp3_params();
        for ch in &mut p.ch[..4] {
            ch.thermal_ff_speedup_factor = ff;
        }
        let want = tail(&mut spp3::Unit::new(p));
        let mut u = spp3::Unit::new(p);
        for x in [1e20f32, 3.0e38, -1e30, 0.0] {
            let o = blk_s(&mut u, x, 512, None, 0);
            assert!(o.iter().all(|v| v.is_finite() && v.abs() <= 1.0), "ff={ff} x={x}");
        }
        let got = tail(&mut u);
        let t = tel(&u);
        let c = &t.channels[0];
        assert!(c.ol_temp_c.is_finite() && c.magnet_temp_c.is_finite() && c.gain.is_finite(), "ff={ff} {c:?}");
        assert!(got <= want * 1.02 + 1e-3, "ff={ff}: {got} after the burst, {want} fresh");
        // (without feed-forward the temperature bound is finding 2's test)
        assert!(ff == 0.0 || c.ol_temp_c < 130.0, "ff={ff}: T_OL {}", c.ol_temp_c);
    }
}

/// Finding 1 for atsp: the same overflow muted the unit for good.
#[test]
fn atsp_huge_finite_samples_neither_unlock_nor_mute() {
    let mut fresh = atsp::Unit::new(atsp_params());
    let mut want = 0.0;
    for _ in 0..secs(20.0) {
        want = blk(&mut fresh, 0.1);
    }
    let mut u = atsp::Unit::new(atsp_params());
    for x in [1e20f32, 3.0e38, 0.0] {
        let o = blk_s(&mut u, x, 512, None, 0);
        assert!(o.iter().all(|v| v.is_finite() && v.abs() <= 1.0), "x={x}");
    }
    let mut got = 0.0;
    for _ in 0..secs(20.0) {
        got = blk(&mut u, 0.1);
    }
    let t = tel(&u);
    assert!(t.channels[0].coil_temp_c.is_finite() && t.channels[0].gain.is_finite());
    assert!((got - want).abs() < 1e-3, "{got} after the burst, {want} fresh");
}

/// Finding 1 for lsm2: the burst is metered at the 16 FS bound, so it
/// heats the model as 512 frames at 16 FS would and limits; it must never
/// make the unit louder, poison the state, or leave the model hot for
/// longer than that heat takes to clear.
#[test]
fn lsm2_huge_finite_samples_do_not_poison_the_model() {
    let x: Vec<f32> = (0..RATE as usize * 5)
        .map(|k| 0.3 * (2.0 * std::f64::consts::PI * 150.0 * k as f64 / RATE as f64).sin() as f32)
        .collect();
    let peak = |o: &[f32]| o.iter().skip(o.len() - 4800).fold(0.0f32, |m, v| m.max(v.abs()));
    let run = |burst: Option<f32>, rest_s: usize| {
        let mut u = lsm2::Unit::new(lsm2_params());
        let b = vec![burst.unwrap_or(0.0); 512];
        let (o, _) = lsm2_go(&mut u, &b);
        assert!(o.iter().all(|v| v.is_finite() && v.abs() <= 1.0));
        for _ in 0..rest_s {
            lsm2_go(&mut u, &vec![0.0; RATE as usize]);
        }
        let (o, _) = lsm2_go(&mut u, &x);
        let t = tel(&u);
        assert!(t.channels[0].coil_temp_c.is_finite() && t.channels[0].gain.is_finite(), "{:?}", t.channels[0]);
        (peak(&o), t.channels[0].coil_temp_c)
    };
    let (want, t_want) = run(None, 0);
    // at once: never louder than a fresh unit, and no hotter than the same
    // burst at the meter bound
    let (got, t_got) = run(Some(1e20), 0);
    let (_, t_bound) = run(Some(16.0), 0);
    assert!(got <= want * 1.001, "{got} after the burst, {want} fresh");
    assert!((t_got - t_bound).abs() < 1e-3, "{t_got} after 1e20, {t_bound} after 16 FS");
    // and the heat clears like any other: after a minute of silence the
    // unit is back to the fresh behaviour
    let (want, t_want_rest) = run(None, 60);
    let (got, t_got) = run(Some(1e20), 60);
    assert!((got - want).abs() < 0.02 * want, "{got} a minute after the burst, {want} fresh");
    // (the 60 s magnet still holds a few kelvin of the burst)
    assert!(t_got < t_want_rest + 5.0 && t_want.is_finite(), "{t_got} a minute after the burst, {t_want_rest} fresh");
}

/// The meters follow the reference and see a slightly over-range program
/// unclamped (a tuning's EQ can peak just over full scale), up to the
/// 16 FS bound; only the output is held to full scale.
#[test]
fn over_range_program_is_metered_unclamped_up_to_sixteen_fs() {
    let temp = |x: f32| {
        let mut u = spp3::Unit::new(spp3_params());
        for _ in 0..secs(0.5) {
            let o = blk_s(&mut u, x, 512, None, 0);
            assert!(o.iter().all(|v| v.abs() <= 1.0));
        }
        tel(&u).channels[0].ol_temp_c
    };
    let (t1, t12, t16, t100) = (temp(1.0), temp(1.2), temp(16.0), temp(100.0));
    assert!(t12 > t1 + 1.0, "1.2 FS metered as {t12} C, 1 FS as {t1} C");
    assert!(t16 > t12 && t100 == t16, "16 FS {t16} C, 100 FS {t100} C");
    // lsm2's monitor is output-referred (rig law 4) and the output is held
    // to full scale, so the input-referred candidates carry the over-range
    // program: the sample-peak candidate sees 1.2 FS as 1.2, and 100 FS as
    // the meter's 16 FS
    let peak = |x: f32| {
        let mut u = lsm2::Unit::new(lsm2_params());
        lsm2_go(&mut u, &vec![x; 4800]);
        tel(&u).channels[0].axis_gain[0]
    };
    assert!(peak(1.2) < peak(1.0) && peak(100.0) == peak(16.0));
}
