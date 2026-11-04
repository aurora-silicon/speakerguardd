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
    for ff in [0.5f32, 0.0] {
        let mut p = spp3_params();
        for c in 0..4 {
            p.ch[c].thermal_ff_speedup_factor = ff;
        }
        let mut fresh = spp3::Unit::new(p);
        let mut want = 0.0;
        for _ in 0..secs(30.0) {
            want = blk(&mut fresh, 1.0);
        }
        let mut u = spp3::Unit::new(p);
        for x in [1e20f32, 3.0e38, -1e30, 0.0] {
            let o = blk_s(&mut u, x, 512, None, 0);
            assert!(o.iter().all(|v| v.is_finite() && v.abs() <= 1.0), "ff={ff} x={x}");
        }
        let mut got = 0.0;
        for _ in 0..secs(30.0) {
            got = blk(&mut u, 1.0);
        }
        let t = tel(&u);
        let c = &t.channels[0];
        assert!(c.ol_temp_c.is_finite() && c.magnet_temp_c.is_finite() && c.gain.is_finite(), "ff={ff} {c:?}");
        assert!(got <= want + 1e-3, "ff={ff}: {got} after the burst, {want} fresh");
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

/// Finding 1 for lsm2: the burst must not leave the model hot for minutes.
#[test]
fn lsm2_huge_finite_samples_do_not_poison_the_model() {
    let x: Vec<f32> = (0..RATE as usize * 5)
        .map(|k| 0.3 * (2.0 * std::f64::consts::PI * 150.0 * k as f64 / RATE as f64).sin() as f32)
        .collect();
    let run = |burst: bool| {
        let mut u = lsm2::Unit::new(lsm2_params());
        if burst {
            let b = vec![1e20f32; 512];
            let (o, _) = lsm2_go(&mut u, &b);
            assert!(o.iter().all(|v| v.is_finite() && v.abs() <= 1.0));
        } else {
            lsm2_go(&mut u, &[0.0; 512]);
        }
        let (o, _) = lsm2_go(&mut u, &x);
        let t = tel(&u);
        assert!(t.channels[0].coil_temp_c.is_finite(), "{:?}", t.channels[0]);
        (o.iter().skip(o.len() - 4800).fold(0.0f32, |m, v| m.max(v.abs())), t.channels[0].coil_temp_c)
    };
    let (want, t_want) = run(false);
    let (got, t_got) = run(true);
    assert!((got - want).abs() < 0.02 * want, "{got} after the burst, {want} fresh");
    assert!(t_got < t_want + 5.0, "{t_got} after the burst, {t_want} fresh");
}
