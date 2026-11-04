//! Safety review of PR #31, findings 7 and 8: lsm2 refuses to run
//! unprotected or with a latency below its controller cadence.
//! Synthetic parameters only; the test failed on 5764482.

mod review_common;

use review_common::*;
use sgv2_core::{lsm2, ProtectionUnit, FAILSAFE_GAIN};

/// Finding 7: lsm2 accepted mitigation off, and every thermal axis off,
/// and then passed audio at unity reporting OK.
#[test]
fn lsm2_refuses_to_run_unprotected() {
    let mut p = lsm2_params();
    p.enable_mitigation = 0;
    assert!(lsm2::Unit::new(p).is_failsafe());
    let mut p = lsm2_params();
    p.control_sample_peak = 0;
    p.control_displacement = 0;
    p.control_velocity = 0;
    p.control_power = 0;
    p.control_temperature = 0;
    assert!(lsm2::Unit::new(p).is_failsafe());
    // excursion axes alone do not protect the coil
    let mut p = lsm2_params();
    p.control_power = 0;
    p.control_temperature = 0;
    assert!(lsm2::Unit::new(p).is_failsafe());
    let mut u = lsm2::Unit::new(p);
    let (o, h) = lsm2_go(&mut u, &[1.0; 512]);
    assert!(h.is_failsafe() && o.iter().all(|v| (v - FAILSAFE_GAIN).abs() < 1e-7));
}

/// Finding 8: a latency below the 256-frame controller cadence let a
/// transient leave before the controller saw it.
#[test]
fn lsm2_refuses_latency_below_the_controller_cadence() {
    for (fb, look) in [(0.0f32, 0.0f32), (2.0, 1.0)] {
        let mut p = lsm2_params();
        p.filterbank_latency_ms = fb;
        p.speaker_response_lookahead_ms = look;
        assert!(lsm2::Unit::new(p).is_failsafe(), "{fb} {look}");
    }
    let mut p = lsm2_params();
    p.filterbank_latency_ms = 2.0;
    p.speaker_response_lookahead_ms = 1.334;
    assert!(!lsm2::Unit::new(p).is_failsafe());
    assert!(lsm2::Unit::new(p).latency_frames() >= lsm2::CONTROLLER_FRAMES as u32);
}
