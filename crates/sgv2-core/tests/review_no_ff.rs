//! Safety review of PR #31. Finding 2 (critical): without the feed-forward filter the hard path holds
//! the limit, and a non-finite speed-up factor is refused.
//! Synthetic parameters only; the test failed on 5764482.

mod review_common;

use review_common::*;
use sgv2_core::spp3;

/// Finding 2 (critical): with the feed-forward filter disabled the hard
/// path settled about 70 C above the limit.
#[test]
fn spp3_without_feed_forward_the_hard_path_holds_the_limit() {
    for ff in [0.0f32, -1.0] {
        let mut p = spp3_params();
        for c in 0..4 {
            p.ch[c].thermal_ff_speedup_factor = ff;
        }
        let hard_on = p.ch[0].temperature_limit + p.ch[0].hard_temp_limit_headroom;
        let mut u = spp3::Unit::new(p);
        assert!(!u.is_failsafe());
        let mut peak = 0.0f32;
        let mut late_sum = 0.0f64;
        let mut late_n = 0;
        for k in 0..secs(120.0) {
            blk(&mut u, 1.0);
            let t = tel(&u).channels[0].ol_temp_c;
            peak = peak.max(t);
            if k > secs(30.0) {
                late_sum += t as f64;
                late_n += 1;
            }
        }
        // the hard path is the only thermal control here: the coil never
        // runs past its arming point by more than one decision interval
        assert!(peak < hard_on + 2.0, "ff={ff}: peak {peak}");
        assert!(late_sum / (late_n as f64) < hard_on as f64, "ff={ff}: mean {}", late_sum / late_n as f64);
    }
}

#[test]
fn spp3_non_finite_speedup_factor_is_refused() {
    for ff in [f32::NAN, f32::INFINITY] {
        let mut p = spp3_params();
        p.ch[1].thermal_ff_speedup_factor = ff;
        assert!(spp3::Unit::new(p).is_failsafe(), "{ff}");
    }
}
