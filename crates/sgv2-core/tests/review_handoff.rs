//! Safety review of PR #31, finding 5: a restarted stream must not give a
//! hot speaker full power back. The thermal state is handed to the next
//! stream or instance, cooled for the gap. Synthetic parameters only.

mod review_common;

use review_common::*;
use sgv2_core::{atsp, lsm2, spp3, HandOff, ProtectionUnit, UnitKind};

/// Heat `u` at level `x` for `s` seconds; return its output and telemetry.
fn heat(u: &mut dyn ProtectionUnit, x: f32, s: f64) -> (f32, sgv2_core::Telemetry) {
    let mut o = 0.0;
    for _ in 0..secs(s) {
        o = blk(u, x);
    }
    (o, tel(u))
}

#[test]
fn spp3_a_restarted_stream_keeps_the_heat() {
    let p = spp3_params();
    let mut u = spp3::Unit::new(p);
    let (hot_out, hot) = heat(&mut u, 1.0, 30.0);
    assert!(hot_out < 0.5 && hot.channels[0].ol_temp_c > 110.0, "{hot_out} {:?}", hot.channels[0]);
    let mut rec = HandOff::EMPTY;
    assert!(u.save_state(&mut rec));

    // what LV2 activate did on 5764482: reset alone, a cold model
    u.reset();
    let cold = blk(&mut u, 1.0);
    assert_eq!(cold, 1.0);

    // the same stream restarted at once, and a new instance
    u.reset();
    for v in [&mut u, &mut spp3::Unit::new(p)] {
        assert!(v.restore_state(&rec, 0.0));
        let t = tel(v);
        assert!((t.channels[0].ol_temp_c - hot.channels[0].ol_temp_c).abs() < 1e-3);
        let o = blk(v, 1.0);
        assert!(o <= hot_out + 0.02, "restarted at {o}, was {hot_out}");
    }

    // after a gap every node has cooled by exp(-gap/tau) and the gain has
    // released toward unity, but no further
    let gap = 1.0;
    let mut v = spp3::Unit::new(p);
    assert!(v.restore_state(&rec, gap));
    let t = tel(&v);
    let c = &p.ch[0];
    let rise = (hot.channels[0].ol_temp_c - hot.channels[0].magnet_temp_c) as f64;
    let mag = (hot.channels[0].magnet_temp_c - 40.0) as f64;
    let want = 40.0 + rise * (-gap / c.tau_tvc as f64).exp() + mag * (-gap / c.tau_tmg as f64).exp();
    assert!((t.channels[0].ol_temp_c as f64 - want).abs() < 1e-3, "{} {want}", t.channels[0].ol_temp_c);
    assert!(t.channels[0].gain > hot.channels[0].gain && t.channels[0].gain < 1.0);
}

#[test]
fn spp3_refuses_a_record_that_does_not_fit() {
    let p = spp3_params();
    let mut u = spp3::Unit::new(p);
    heat(&mut u, 1.0, 10.0);
    let mut rec = HandOff::EMPTY;
    assert!(u.save_state(&mut rec));
    let bad: [fn(&mut HandOff); 7] = [
        |r| r.kind = UnitKind::Atsp as u32,
        |r| r.channels = 2,
        |r| r.sample_rate = 44_100,
        |r| r.node[1][0] = f64::NAN,
        |r| r.node[3][1] = -1.0,
        |r| r.gain[0] = f64::INFINITY,
        |r| r.gain[2] = 0.0,
    ];
    for f in bad {
        let mut r = rec;
        f(&mut r);
        let mut v = spp3::Unit::new(p);
        assert!(!v.restore_state(&r, 0.0));
        assert_eq!(tel(&v).channels[0].ol_temp_c, 40.0);
    }
    // a non-finite or negative gap is no cooling, never a warmer start
    for gap in [f64::NAN, -5.0, f64::NEG_INFINITY] {
        let mut v = spp3::Unit::new(p);
        assert!(v.restore_state(&rec, gap));
        assert!((tel(&v).channels[0].ol_temp_c - tel(&u).channels[0].ol_temp_c).abs() < 1e-3);
    }
    // a fail-safe unit has nothing to hand off and takes nothing
    let mut f = spp3::Unit::new(spp3::Params::default());
    let mut out = HandOff::EMPTY;
    assert!(!f.save_state(&mut out) && !f.restore_state(&rec, 0.0));
}

#[test]
fn atsp_a_restarted_stream_keeps_the_heat() {
    let p = atsp_params();
    let mut u = atsp::Unit::new(p);
    let (hot_out, hot) = heat(&mut u, 1.0, 30.0);
    assert!(hot_out < 0.5, "{hot_out}");
    let mut rec = HandOff::EMPTY;
    assert!(u.save_state(&mut rec));
    u.reset();
    assert_eq!(blk(&mut u, 1.0), 1.0);
    let mut v = atsp::Unit::new(p);
    assert!(v.restore_state(&rec, 0.0));
    assert!((tel(&v).channels[0].coil_temp_c - hot.channels[0].coil_temp_c).abs() < 1e-3);
    let o = blk(&mut v, 1.0);
    assert!(o <= hot_out + 0.02, "restarted at {o}, was {hot_out}");
    // cooling follows the two-node model: the magnet holds the coil up
    let mut w = atsp::Unit::new(p);
    assert!(w.restore_state(&rec, 5.0));
    let t = tel(&w).channels[0];
    assert!(t.coil_temp_c < hot.channels[0].coil_temp_c && t.coil_temp_c >= t.magnet_temp_c - 1e-3);
    assert!(t.magnet_temp_c > 40.0 + 0.7 * (hot.channels[0].magnet_temp_c - 40.0));
}

#[test]
fn lsm2_a_restarted_stream_keeps_the_heat() {
    let p = lsm2_params();
    let tone: Vec<f32> = (0..RATE as usize * 20)
        .map(|k| (2.0 * std::f64::consts::PI * 150.0 * k as f64 / RATE as f64).sin() as f32)
        .collect();
    let mut u = lsm2::Unit::new(p);
    lsm2_go(&mut u, &tone);
    let hot = tel(&u).channels[0];
    assert!(hot.coil_temp_c > 100.0 && hot.gain < 0.9, "{hot:?}");
    let mut rec = HandOff::EMPTY;
    assert!(u.save_state(&mut rec));
    let mut v = lsm2::Unit::new(p);
    assert!(v.restore_state(&rec, 0.0));
    let t = tel(&v).channels[0];
    assert!((t.coil_temp_c - hot.coil_temp_c).abs() < 1e-3 && (t.gain - hot.gain).abs() < 1e-3);
    // a cold unit would pass the next second of tone at unity after its
    // latency; the restored one keeps attenuating
    let (o, _) = lsm2_go(&mut v, &tone[..RATE as usize]);
    let peak = o[1000..].iter().fold(0.0f32, |m, v| m.max(v.abs()));
    assert!(peak < 0.9, "{peak}");
}
