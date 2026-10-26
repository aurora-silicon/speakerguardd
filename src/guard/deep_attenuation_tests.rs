// SPDX-License-Identifier: MIT
//! Normal attenuation crosses the former 7 dB handoff without losing audio.
use super::fake::FakeHw;
use super::tests::{run_until, vi_state};
use super::*;

#[test]
fn a_7_9_db_request_keeps_the_lock_and_the_same_continuous_gain() {
    let mut cfg = Config::parse(include_str!("../../conf/apple/j616.conf")).unwrap();
    cfg.allow_unqualified = true;
    // Match the deployed command line without changing any profile.
    cfg.globals.governor = Governor::Unified;
    cfg.globals.guard = Some(Governor::Asahi);
    cfg.allow_unified = true;
    let mut hw = FakeHw::per_speaker(&cfg);
    hw.level = Box::new(|_| 0.0);
    let path = vi_state(&cfg, 55.0, 52.0);
    let mut g = Guard::start(&mut hw, &cfg, 7.0, Some(path.clone())).unwrap();
    run_until(&mut g, &mut hw, Duration::from_secs(2)).unwrap();
    assert!(g.armed());
    let relinquished = hw.relinquished;
    let lapses = hw.lapses.len();
    let writes = hw.writes.len();
    for a in g.engine.as_mut().unwrap().asahi.as_mut().unwrap() { a.gain = -6.0; }
    g.decide(&mut hw, false).unwrap();
    let mut previous = hw.per.clone();
    // Cross 7 dB both ways repeatedly, then use much deeper valid targets.
    // Setting the model's output isolates the handoff from the ramp itself.
    for request in [6.0, 6.5, 7.0, 7.5, 7.9, 8.0, 8.5, 8.0, 7.9, 7.0, 6.5,
                    7.0, 7.9, 8.5, 9.0, 9.5, 10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0,
                    15.5, 16.0, 16.5, 17.0, 17.5, 18.0] {
        for a in g.engine.as_mut().unwrap().asahi.as_mut().unwrap() {
            a.gain = -request;
        }
        assert!((g.governing_state().0 - request).abs() < 1e-9);
        let (_, expected, hard) = g.governing_state();
        assert!(!hard);
        g.decide(&mut hw, false).unwrap();
        assert!(g.armed() && hw.unlocked_at(hw.t), "lost audio at {request} dB");
        assert_eq!(hw.relinquished, relinquished);
        assert_eq!(hw.lapses.len(), lapses);
        for ((&applied, &want), &old) in hw.per.iter().zip(&expected).zip(&previous) {
            assert_eq!(applied, want, "the governor ramp changed at {request} dB");
            assert!((applied - old).abs() <= 1.0, "gain jumped from {old} to {applied}");
            assert!(applied < hw.safe, "wrote the kernel safe level");
        }
        previous = hw.per.clone();
    }
    assert!(hw.writes.len() > writes);
    let _ = std::fs::remove_file(path);
}

#[test]
fn the_hardware_range_is_inclusive_and_has_recovery_hysteresis() {
    let cfg = tests::j413();
    let mut hw = FakeHw::per_speaker(&cfg);
    let path = vi_state(&cfg, 55.0, 52.0);
    let mut g = Guard::start(&mut hw, &cfg, 7.0, Some(path.clone())).unwrap();
    for request in [7.9, 20.0, 99.5, 100.0, 100.5] {
        assert!(g.within_range(&hw, &[request]));
    }
    assert!(!g.within_range(&hw, &[100.5001]));
    for request in [100.5, 99.5, 98.0, 100.0, 97.5001] {
        assert!(!g.within_range(&hw, &[request]), "oscillated at {request} dB");
    }
    assert!(g.within_range(&hw, &[97.5]));
    // A card with a smaller range must use its own range, not 201 steps.
    hw.reduction_limit = 12.0;
    assert!(g.within_range(&hw, &[12.0]));
    assert!(!g.within_range(&hw, &[12.01]));
    assert!(!g.within_range(&hw, &[10.0]));
    assert!(g.within_range(&hw, &[9.0]));
    assert!(!g.within_range(&hw, &[f64::NAN]));
    let _ = std::fs::remove_file(path);
}

#[test]
fn a_failed_deep_write_still_hands_the_card_back() {
    let cfg = tests::j413();
    let mut hw = FakeHw::per_speaker(&cfg);
    hw.level = Box::new(|_| 0.0);
    let path = vi_state(&cfg, 55.0, 52.0);
    let mut g = Guard::start(&mut hw, &cfg, 7.0, Some(path.clone())).unwrap();
    run_until(&mut g, &mut hw, Duration::from_secs(2)).unwrap();
    assert!(g.armed());
    for a in g.engine.as_mut().unwrap().asahi.as_mut().unwrap() { a.gain = -7.9; }
    hw.target_fail = Box::new(|_| true);
    g.decide(&mut hw, false).unwrap();
    assert!(!g.armed());
    assert!(hw.relinquished > 0);
    assert!((0..hw.per.len()).all(|k| hw.effective_of(k) >= hw.safe));
    let _ = std::fs::remove_file(path);
}
