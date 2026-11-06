// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The per-group peak limiter box (`aufx/cfrk`, "ControlFreak"), compiled
//! to `sgv2_core::voicing::cfreak::Params` from its saved state.
//!
//! Global parameters, as the unit reads them (an ID absent from the saved
//! state takes the unit's published default, given in brackets):
//!
//! | ID | meaning | carried |
//! |---:|---|---|
//! | 0 | bypass [0] | 1: the box is left out |
//! | 1, 10, 17 | saturation oversampling, flavour, amount [0] | only matter with 16 |
//! | 2, 11 | input gain (dB) [0], its stage enabled [1] | only 0 dB or disabled |
//! | 3 | threshold (dB) [0] | yes |
//! | 4 | MinFastGain (dB) [3] | yes |
//! | 5 | fast gain release (ms) [10]: a hold | yes |
//! | 6 | true-peak detection [0] | yes |
//! | 7 | look-ahead (ms) [0.5] | yes |
//! | 8, 15 | output gain (dB) [0], its stage enabled [1] | only 0 dB or disabled |
//! | 9 | parameter ramp time [0] | not needed: the state is fixed |
//! | 12, 13, 21, 22 | high-pass enable [0], corner, order, placement | refused when enabled |
//! | 14 | limiter enabled [1] | refused when off |
//! | 16 | output saturation [0] | refused when on |
//! | 18-20 | de-esser frequency, strength, quality | no effect while saturation is off |
//! | 23 | algorithm version [0] | only 0 |
//!
//! The group-scope sections the strips also save do not change the unit's
//! output; the global values are used.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::tuning::Stage;
use sgv2_core::voicing::cfreak::Params;

/// The limiter unit's subtype.
pub const CFRK: FourCc = FourCc::from_bytes(*b"cfrk");

fn default(id: u32) -> Option<f64> {
    Some(match id {
        0..=3 | 6 | 8 | 9 | 10 | 12 | 16 | 17 | 22 | 23 => 0.0,
        4 => 3.0,
        5 => 10.0,
        7 => 0.5,
        11 | 14 | 15 => 1.0,
        _ => return None,
    })
}

pub fn compile(stage: &Stage, fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    let ctx = format!("limiter box {}", stage.name);
    let bad = |m: String| Error::format(&ctx, m);
    if stage.unit.map(|u| u.subtype) != Some(CFRK) {
        return Err(bad(format!("is {}, not a cfrk unit", stage.kind)));
    }
    if stage.bypass {
        return Ok(None);
    }
    let g =
        stage.state.as_ref().and_then(|s| s.global()).ok_or_else(|| bad("the strip gives it no saved state".into()))?;
    let get = |id: u32| -> Result<f64> {
        match g.get(id) {
            Some(v) if v.is_finite() => Ok(f64::from(v)),
            Some(_) => Err(bad(format!("saved parameter {id} is not finite"))),
            None => default(id).ok_or_else(|| bad(format!("saved parameter {id} is missing"))),
        }
    };
    if get(0)? != 0.0 {
        return Ok(None);
    }
    if get(14)? == 0.0 {
        return Err(bad("the limiter is switched off (not carried)".into()));
    }
    if get(16)? != 0.0 {
        return Err(bad("output saturation is on (not carried)".into()));
    }
    if get(12)? != 0.0 {
        return Err(bad("the high-pass is on (not carried)".into()));
    }
    if get(23)? != 0.0 {
        return Err(bad(format!("algorithm version {} (only 0 is carried)", get(23)?)));
    }
    if get(11)? != 0.0 && get(2)? != 0.0 {
        return Err(bad(format!("input gain {} dB (not carried)", get(2)?)));
    }
    if get(15)? != 0.0 && get(8)? != 0.0 {
        return Err(bad(format!("output gain {} dB (not carried)", get(8)?)));
    }
    let words = [get(7)? / 1000.0, get(5)? / 1000.0, get(3)?, get(4)?, get(6)?];
    let p = Params::from_words(&words).ok_or_else(|| bad(format!("values out of range: {words:?}")))?;
    let frames = |s: f64| s * fs;
    if (frames(p.lookahead_s) - frames(p.lookahead_s).round()).abs() > 1e-6
        || (frames(p.hold_s) - frames(p.hold_s).round()).abs() > 1e-6
    {
        diags.warn(
            "cfrk-fractional-frames",
            &ctx,
            format!("look-ahead or hold is not a whole number of frames at {fs} Hz; rounded"),
        );
    }
    Ok(Some(p))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState, UnitId};

    fn stage(params: &[(u32, f32)]) -> Stage {
        Stage {
            name: "Limit".into(),
            kind: "aufx/cfrk/appl".into(),
            unit: Some(UnitId {
                kind: FourCc::from_bytes(*b"aufx"),
                subtype: CFRK,
                manufacturer: FourCc::from_bytes(*b"appl"),
            }),
            inputs: 1,
            outputs: 1,
            channels: Some(2),
            bypass: false,
            preset_name: None,
            state: Some(ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: params.to_vec() }] }),
            preset_extra_keys: Vec::new(),
            before_protection: true,
        }
    }

    fn compile_of(params: &[(u32, f32)]) -> Result<Option<Params>> {
        compile(&stage(params), 48_000.0, &mut Diagnostics::default())
    }

    #[test]
    fn reads_the_limiter_and_defaults() {
        let p = compile_of(&[(3, -1.5), (4, 12.0), (5, 2.0), (6, 1.0), (7, 0.25)]).unwrap().unwrap();
        assert_eq!(
            p,
            Params { lookahead_s: 0.00025, hold_s: 0.002, threshold_db: -1.5, min_fast_gain_db: 12.0, true_peak: true }
        );
        let d = compile_of(&[]).unwrap().unwrap();
        assert_eq!(
            (d.lookahead_s, d.hold_s, d.threshold_db, d.min_fast_gain_db, d.true_peak),
            (0.0005, 0.01, 0.0, 3.0, false)
        );
    }

    #[test]
    fn bypass_and_refusals() {
        assert_eq!(compile_of(&[(0, 1.0)]).unwrap(), None);
        let mut st = stage(&[]);
        st.bypass = true;
        assert_eq!(compile(&st, 48_000.0, &mut Diagnostics::default()).unwrap(), None);
        for refused in [
            [(14, 0.0), (0, 0.0)],
            [(16, 1.0), (0, 0.0)],
            [(12, 1.0), (0, 0.0)],
            [(23, 1.0), (0, 0.0)],
            [(2, 3.0), (11, 1.0)],
            [(8, -2.0), (15, 1.0)],
            [(6, 0.5), (0, 0.0)],
            [(7, 5.0), (0, 0.0)],
        ] {
            assert!(compile_of(&refused).is_err(), "{refused:?}");
        }
        assert!(compile_of(&[(2, 3.0), (11, 0.0), (8, -2.0), (15, 0.0)]).unwrap().is_some());
        assert!(compile_of(&[(99, 1.0)]).unwrap().is_some());
        let mut st = stage(&[]);
        st.state = None;
        assert!(compile(&st, 48_000.0, &mut Diagnostics::default()).is_err());
    }
}
