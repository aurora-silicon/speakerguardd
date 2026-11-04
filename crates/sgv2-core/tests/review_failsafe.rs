//! Safety review of PR #31. Finding 4: the fail-safe output is finite and bounded for any input.
//! Synthetic parameters only; the test failed on 5764482.

use sgv2_core::{atsp, lsm2, spp3, FailSafe, Health, Io, ProtectionUnit, UnitKind, FAILSAFE_GAIN};

/// Finding 4: the fail-safe unit turned +Inf into +Inf (full-scale DC).
#[test]
fn failsafe_output_is_finite_and_bounded() {
    let x = [f32::INFINITY, f32::NEG_INFINITY, f32::NAN, 1e20, -1e30, 0.5, -1.0, 1e-40, 0.0];
    let want = [0.0f32, 0.0, 0.0, 0.1, -0.1, 0.05, -0.1, 1e-40 * 0.1, 0.0];
    let check = |u: &mut dyn ProtectionUnit, n: usize| {
        let ins: Vec<&[f32]> = (0..n).map(|_| &x[..]).collect();
        let mut ob = vec![[9.0f32; 9]; n];
        let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
        let h = u.process(Io { frames: 9, input: &ins, sense: &[], output: &mut outs });
        assert!(h.is_failsafe());
        for o in &ob {
            for k in 0..9 {
                assert!(o[k].is_finite() && o[k].abs() <= FAILSAFE_GAIN, "{:?}", o);
                assert!((o[k] - want[k]).abs() <= 1e-7, "{k}: {} {}", o[k], want[k]);
            }
        }
    };
    check(&mut FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS), 2);
    check(&mut spp3::Unit::new(spp3::Params::default()), 4);
    check(&mut atsp::Unit::new(atsp::Params::default()), 4);
    check(&mut lsm2::Unit::new(lsm2::Params::default()), 2);
}
