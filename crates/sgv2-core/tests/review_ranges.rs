//! Safety review of PR #31. Findings 6, 9 and 10: spp3 blob ranges are validated at load, so
//! `process` can never panic or diverge.
//! Synthetic parameters only; the test failed on 5764482.

mod review_common;

use review_common::*;
use sgv2_core::spp3;

/// Finding 6: a negative or non-finite TlimAdjCap was accepted and the
/// first accepted window panicked in `clamp` on the RT thread.
#[test]
fn spp3_bad_tlim_adj_cap_is_refused_and_never_panics() {
    for cap in [-1.0f32, f32::NAN, f32::INFINITY] {
        let mut u = spp3::Unit::new(spp3_cl_params(cap));
        assert!(u.is_failsafe(), "cap {cap} accepted");
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            for _ in 0..secs(1.0) {
                blk_s(&mut u, 0.05, 512, Some((0.3, 0.2)), 512);
            }
        }));
        assert!(r.is_ok(), "cap {cap} panicked");
    }
    // the other closed-loop fields are ranges too
    let bad: [fn(&mut spp3::ChannelParams); 6] = [
        |c| c.temp_sense_window_time_s = f32::NAN,
        |c| c.tlim_err_decay_time_s = 0.0,
        |c| c.reb_range_min = 9.0,
        |c| c.cal_temp_range_max = f32::NAN,
        |c| c.rdc_scale = -1.0,
        |c| c.a_t_20c = f32::INFINITY,
    ];
    for f in bad {
        let mut p = spp3_cl_params(30.0);
        f(&mut p.ch[2]);
        assert!(spp3::Unit::new(p).is_failsafe());
    }
}

/// Finding 9: a start state below ambient allowed more drive than a cold
/// start. The hot fallback (64007 other than 2) seeds the steady-state split
/// at the ordinary limit, so a limit below ambient is refused.
#[test]
fn spp3_a_hot_start_below_ambient_is_refused() {
    let mut p = spp3_params();
    p.thermal_state_override = 0;
    assert!(!spp3::Unit::new(p).is_failsafe());
    p.ch[0].temperature_limit = 30.0; // ambient 40
    assert!(spp3::Unit::new(p).is_failsafe());
    // the cold start never seeds below ambient
    p.thermal_state_override = spp3::THERMAL_STATE_COLD;
    assert!(!spp3::Unit::new(p).is_failsafe());
}

/// Finding 10: time constants below one thermal update made the forward
/// recurrence diverge; fuzz finding 15: coil poles of 16-21 ms (3-4 updates)
/// cycle through the hard path above the hard limit under constant drive,
/// as the reference itself overshoots there. Below the reference's
/// documented 0.1 s minimum the set is refused.
#[test]
fn spp3_time_constants_below_the_documented_minimum_are_refused() {
    let bad: [fn(&mut spp3::ChannelParams); 6] = [
        |c| c.tau_tvc = 0.001,
        |c| c.tau_tmg = 0.004,
        |c| c.thermal_ff_speedup_factor = 0.001,
        // the finding-15 sets: 21 / 22 ms and 16 ms coils
        |c| (c.tau_tvc, c.tau_tmg) = (0.021, 0.022),
        |c| c.tau_tvc = 0.016,
        // a 0.4 s coil with a 0.2 speed-up is an 80 ms feed-forward pole
        |c| c.thermal_ff_speedup_factor = 0.2,
    ];
    for f in bad {
        let mut p = spp3_params();
        f(&mut p.ch[1]);
        assert!(spp3::Unit::new(p).is_failsafe());
    }
    // exactly the minimum is accepted, also as the feed-forward pole
    let mut p = spp3_params();
    for c in 0..4 {
        p.ch[c].tau_tvc = spp3::TAU_MIN_S;
        p.ch[c].tau_tmg = spp3::TAU_MIN_S;
        p.ch[c].thermal_ff_speedup_factor = 1.0;
    }
    assert!(!spp3::Unit::new(p).is_failsafe());
}
