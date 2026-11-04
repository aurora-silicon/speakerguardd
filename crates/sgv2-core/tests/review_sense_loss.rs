//! Safety review of PR #31. Finding 3: losing the sense bus returns the corrected limit to Tlim.
//! Synthetic parameters only; the test failed on 5764482.

mod review_common;

use review_common::*;
use sgv2_core::spp3;

/// Finding 3: when the sense bus goes away the last cool T_CL was kept and
/// the limit stayed relaxed by TlimAdjCap.
#[test]
fn spp3_lost_sense_returns_the_limit_to_tlim() {
    // R = 4 ohm = the calibration, so T_CL = 25 C (a cool coil)
    let s = Some((0.3f32, 0.2f32));
    let mut ol = spp3::Unit::new(spp3_params());
    for _ in 0..secs(2.0) {
        blk_s(&mut ol, 0.05, 512, s, 512);
    }
    for _ in 0..secs(60.0) {
        blk(&mut ol, 1.0);
    }
    let t_ol = tel(&ol).channels[0].ol_temp_c;
    // the bus missing, then present but shorter than the call
    for short in [0usize, 100] {
        let mut u = spp3::Unit::new(spp3_cl_params(30.0));
        for _ in 0..secs(2.0) {
            blk_s(&mut u, 0.05, 512, s, 512);
        }
        let t = tel(&u);
        assert!(t.channels[0].closed_loop && (t.channels[0].cl_temp_c - 25.0).abs() < 1.0, "{:?}", t.channels[0]);
        for _ in 0..secs(60.0) {
            if short == 0 {
                blk(&mut u, 1.0);
            } else {
                blk_s(&mut u, 1.0, 512, s, short);
            }
        }
        let t = tel(&u).channels[0];
        assert!(t.tlim_adj_c < 0.5, "short={short}: TlimAdj {}", t.tlim_adj_c);
        assert!(t.ol_temp_c < t_ol + 1.0, "short={short}: {} against open loop {t_ol}", t.ol_temp_c);
    }
}
