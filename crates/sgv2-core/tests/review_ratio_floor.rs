//! spp3's candidate ratio floor is 1e-4 (spec 6: a 0.01 linear-gain floor,
//! -40 dB). The engine had 0.01, a 0.1 gain floor, so a drive needing more
//! than 20 dB of reduction settled above the limit without ever arming the
//! hard path. Found while fixing the PR #31 review; failed on 5764482.

mod review_common;

use review_common::*;
use sgv2_core::spp3;

#[test]
fn spp3_reduces_below_twenty_db_when_the_limit_needs_it() {
    let mut p = spp3_params();
    for ch in &mut p.ch[..4] {
        // 2 K of headroom over ambient: holding it at full scale takes
        // about -24 dB, inside the -40 dB floor, below the hard path
        ch.temperature_limit = 42.0;
    }
    let mut u = spp3::Unit::new(p);
    let mut out = 1.0;
    for _ in 0..secs(60.0) {
        out = blk(&mut u, 1.0);
    }
    let t = tel(&u).channels[0];
    assert!(t.ol_temp_c < 42.0 + 0.2, "T_OL {} against a 42 C limit (out {out})", t.ol_temp_c);
    assert!(out < 0.1 && out > 0.01, "{out}");
}
