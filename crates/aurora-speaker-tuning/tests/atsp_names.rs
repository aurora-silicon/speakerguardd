// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The atsp name map covers exactly the 41 IDs the engine accepts (spec 2).

use aurora_speaker_tuning::names::{atsp, ATSP_GLOBAL, SCOPE_GLOBAL, SCOPE_PART};
use sgv2_core::atsp::{ids, Params};

#[test]
fn atsp_map_is_the_engine_map() {
    let got: Vec<u32> = ATSP_GLOBAL.iter().map(|p| p.id).collect();
    assert_eq!(got, (0..=40).collect::<Vec<u32>>());
    let mut p = Params::default();
    for n in ATSP_GLOBAL {
        assert!(p.set(n.id, 1.0), "engine refuses named ID {}", n.id);
        assert_eq!(atsp(SCOPE_GLOBAL, n.id).map(|x| x.id), Some(n.id));
    }
    assert!(!p.set(41, 1.0));
    assert!(atsp(SCOPE_GLOBAL, 41).is_none() && atsp(SCOPE_PART, 0).is_none());
    assert_eq!(atsp(SCOPE_GLOBAL, ids::GAIN_LINK_ALL).unwrap().name, "gain link all");
    for t in 0..3u32 {
        let first = 1 + ids::SLOT_STRIDE * t;
        assert!(atsp(SCOPE_GLOBAL, first).unwrap().name.ends_with("amplifier sensitivity"));
        assert!(atsp(SCOPE_GLOBAL, first + 12).unwrap().name.ends_with("audio channel assignment"));
    }
}
