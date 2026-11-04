//! Safety review of PR #31. No input sequence (random, NaN, Inf, denormal, 1e30, f32::MAX) ever
//! produces an output above the input times the allowed gain, in any unit (lsm2, a five-band
//! limiter: above full scale; its per-frequency bound is in tests/lsm2.rs).
//! Synthetic parameters only; the test failed on 5764482.

mod review_common;

use review_common::*;
use sgv2_core::{atsp, lsm2, spp3, FailSafe, Health, UnitKind};

#[test]
fn no_input_ever_leaves_above_the_input_times_the_allowed_gain() {
    for seed in [1u64, 0x9e37_79b9_7f4a_7c15, 42] {
        never_above_input(&mut spp3::Unit::new(spp3_params()), 4, 0, seed, 400);
        never_above_input_sensed(&mut spp3::Unit::new(spp3_cl_params(30.0)), 4, 8, 0, seed, 400);
        let mut p = spp3_params();
        for c in 0..4 {
            p.ch[c].thermal_ff_speedup_factor = 0.0;
        }
        never_above_input(&mut spp3::Unit::new(p), 4, 0, seed, 400);
        never_above_input(&mut atsp::Unit::new(atsp_params()), 4, 0, seed, 400);
        let mut p = atsp_params();
        p.block_frames = 0;
        never_above_input(&mut atsp::Unit::new(p), 4, 0, seed, 400);
        // lsm2 splits the program into five bands: see never_above_full_scale
        never_above_full_scale(&mut lsm2::Unit::new(lsm2_params()), 2, seed, 400);
        never_above_input(&mut FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS), 4, 0, seed, 100);
    }
}
