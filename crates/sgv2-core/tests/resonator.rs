// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The standalone resonator component's fail-closed contract
//! (`docs/RESONATOR-COMPONENT.md`): parameter domain checks, and a latched,
//! zeroed block on any non-finite input or state until an explicit reset.
//!
//! These tests live outside `src/` so the voicing law (a hash over
//! `sgv2-core/src`, see `aurora-speaker-tuning/build.rs`) is unchanged.
use sgv2_core::resonator::{Error, Filter, Params};

/// A synthetic model and request (not taken from any machine's tuning),
/// the same values `tests/lsm2.rs` uses for its compensated channel.
fn synthetic() -> Params {
    Params {
        rdc: 4.0,
        bl: 0.8,
        mass_kg: 0.0002,
        damping: 0.2,
        stiffness: 1500.0,
        desired_hz: 250.0,
        desired_q: 1.0,
        max_boost_db: 2.0,
        max_cut_db: 6.0,
        rate: 48_000.0,
    }
}

fn signal(n: usize) -> Vec<f32> {
    (0..n).map(|k| 0.3 * libm::sinf(k as f32 * 0.05) + 0.1 * libm::sinf(k as f32 * 0.31)).collect()
}

#[test]
fn a_valid_model_filters_and_reset_replays_bit_exact_at_any_block_size() {
    let mut f = Filter::new(synthetic()).unwrap();
    let x = signal(4000);
    let mut a = x.clone();
    for block in a.chunks_mut(512) {
        f.process(block).unwrap();
    }
    assert!(a.iter().zip(&x).any(|(y, x)| y != x), "the filter must change the signal");
    assert!(a.iter().all(|v| v.is_finite()));
    f.reset();
    let mut b = x.clone();
    for block in b.chunks_mut(37) {
        f.process(block).unwrap();
    }
    assert!(a.iter().zip(&b).all(|(p, q)| p.to_bits() == q.to_bits()));
    // an empty block is fine and leaves the state alone
    f.process(&mut []).unwrap();
}

#[test]
fn every_parameter_outside_its_domain_is_refused() {
    type Set = fn(&mut Params, f64);
    let positive: [Set; 7] = [
        |p, v| p.rdc = v,
        |p, v| p.mass_kg = v,
        |p, v| p.damping = v,
        |p, v| p.stiffness = v,
        |p, v| p.desired_hz = v,
        |p, v| p.desired_q = v,
        |p, v| p.rate = v,
    ];
    for (i, set) in positive.iter().enumerate() {
        for v in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut p = synthetic();
            set(&mut p, v);
            assert_eq!(Filter::new(p).err(), Some(Error::InvalidParameters), "positive field {i} = {v}");
        }
    }
    let nonnegative: [Set; 3] = [|p, v| p.bl = v, |p, v| p.max_boost_db = v, |p, v| p.max_cut_db = v];
    for (i, set) in nonnegative.iter().enumerate() {
        for v in [-1e-9, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut p = synthetic();
            set(&mut p, v);
            assert_eq!(Filter::new(p).err(), Some(Error::InvalidParameters), "nonnegative field {i} = {v}");
        }
        // zero is in the domain of a nonnegative field
        let mut p = synthetic();
        set(&mut p, 0.0);
        assert!(Filter::new(p).is_ok(), "nonnegative field {i} = 0");
    }
}

#[test]
fn a_request_outside_the_validated_domain_is_refused() {
    // the requested resonance at or above Nyquist
    let mut p = synthetic();
    p.desired_hz = 30_000.0;
    assert_eq!(Filter::new(p).err(), Some(Error::OutsideValidatedDomain));
    // the physical resonance itself above Nyquist
    let mut p = synthetic();
    p.stiffness = 1e12;
    assert_eq!(Filter::new(p).err(), Some(Error::OutsideValidatedDomain));
    // a shift the allowed cut cannot reach (no real damping solves it)
    let mut p = synthetic();
    p.desired_hz = 2000.0;
    p.max_cut_db = 0.0;
    assert_eq!(Filter::new(p).err(), Some(Error::OutsideValidatedDomain));
}

#[test]
fn nonfinite_input_zeroes_the_whole_block_and_latches_until_reset() {
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut f = Filter::new(synthetic()).unwrap();
        let mut block = [0.5f32, -0.25, bad, 0.75];
        assert_eq!(f.process(&mut block), Err(Error::NonfiniteInput));
        assert_eq!(block, [0.0; 4]);
        // closed: a finite block is zeroed too, with no bypass
        let mut next = [0.5f32; 8];
        assert_eq!(f.process(&mut next), Err(Error::Faulted));
        assert_eq!(next, [0.0; 8]);
        assert_eq!(f.process(&mut next), Err(Error::Faulted));
        // reset reopens it with clean history: it matches a fresh filter
        f.reset();
        let mut a = signal(300);
        let mut b = a.clone();
        f.process(&mut a).unwrap();
        Filter::new(synthetic()).unwrap().process(&mut b).unwrap();
        assert_eq!(a, b);
    }
}

#[test]
fn an_output_that_leaves_f32_range_zeroes_the_block_and_latches() {
    // a full-range DC input: the synthetic model's DC gain is above one,
    // so the output overflows f32 within the block
    let mut f = Filter::new(synthetic()).unwrap();
    let mut block = vec![f32::MAX; 2048];
    assert_eq!(f.process(&mut block), Err(Error::NonfiniteState));
    assert!(block.iter().all(|v| *v == 0.0), "the samples already written are zeroed too");
    let mut next = [0.1f32; 4];
    assert_eq!(f.process(&mut next), Err(Error::Faulted));
    assert_eq!(next, [0.0; 4]);
}
