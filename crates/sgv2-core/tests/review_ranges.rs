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

/// Finding 9: a negative start state allowed more drive than a cold start.
#[test]
fn spp3_negative_start_state_is_refused() {
    let mut p = spp3_params();
    p.ch[0].initial_theta_vc = -1000.0;
    assert!(spp3::Unit::new(p).is_failsafe());
    let mut p = spp3_params();
    p.ch[3].initial_theta_mg = -0.5;
    assert!(spp3::Unit::new(p).is_failsafe());
}

/// Finding 10: time constants below one thermal update made the forward
/// recurrence diverge.
#[test]
fn spp3_time_constants_below_one_update_are_refused() {
    let dt = spp3::THERMAL_FRAMES as f32 / RATE as f32;
    let bad: [fn(&mut spp3::ChannelParams); 3] =
        [|c| c.tau_tvc = 0.001, |c| c.tau_tmg = 0.004, |c| c.thermal_ff_speedup_factor = 0.001];
    for f in bad {
        let mut p = spp3_params();
        f(&mut p.ch[1]);
        assert!(spp3::Unit::new(p).is_failsafe());
    }
    // exactly one update is the shortest usable pole
    let mut p = spp3_params();
    for c in 0..4 {
        p.ch[c].tau_tvc = dt;
        p.ch[c].thermal_ff_speedup_factor = 1.0;
    }
    assert!(!spp3::Unit::new(p).is_failsafe());
}
