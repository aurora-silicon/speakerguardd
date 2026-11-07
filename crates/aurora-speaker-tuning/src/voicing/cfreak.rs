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
//! | 1, 10, 17 | saturation oversampling, flavour, amount [0] | modes 1-7 / sine / dormant |
//! | 2, 11 | input gain (dB) [0], its stage enabled [1] | yes, or unity when disabled |
//! | 3 | threshold (dB) [0] | yes |
//! | 4 | MinFastGain (dB) [3] | yes |
//! | 5 | fast gain release (ms) [10]: a hold | yes |
//! | 6 | true-peak detection [0] | yes |
//! | 7 | look-ahead (ms) [0.5] | yes |
//! | 8, 15 | output gain (dB) [0], its stage enabled [1] | yes, or unity when disabled |
//! | 9 | parameter ramp time [0] | not needed: the state is fixed |
//! | 12, 13, 21, 22 | high-pass enable [0], corner, order, placement | refused when enabled |
//! | 14 | limiter enabled [1] | refused when off |
//! | 16 | output saturation [0] | off, or the sine flavour |
//! | 18-20 | de-esser frequency, strength (dB), Q | yes, with output saturation |
//! | 21 | high-pass order | required with output saturation |
//! | 23 | algorithm version [0] | 0 without saturation, 1 with the carried subset |
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
    let saturation = get(16)? != 0.0;
    if get(12)? != 0.0 {
        return Err(bad("the high-pass is on (not carried)".into()));
    }
    let (saturation_mode, saturation_flavor, saturation_mix) = if saturation {
        let mode = get(1)?;
        if !(1.0..=7.0).contains(&mode) || mode != mode.trunc() || get(10)? != 2.0 || get(23)? != 1.0 {
            return Err(bad(format!(
                "output saturation mode/flavor/version {}/{}/{} (modes 1-7, sine flavor 2, version 1 are carried)",
                mode,
                get(10)?,
                get(23)?
            )));
        }
        let amount = get(17)?;
        if !(0.0..=100.0).contains(&amount) {
            return Err(bad(format!("output saturation amount {amount}%")));
        }
        // The de-esser (18-20) shapes the saturation drive; 21 belongs to
        // the version-1 state. Require complete finite saved values rather
        // than silently defaulting an unknown state.
        for id in [18, 19, 20, 21] {
            get(id)?;
        }
        (mode, 2.0, amount / 100.0)
    } else {
        if get(23)? != 0.0 {
            return Err(bad(format!("algorithm version {} (only 0 without saturation is carried)", get(23)?)));
        }
        (0.0, 0.0, 0.0)
    };
    let words = [
        get(7)? / 1000.0,
        get(5)? / 1000.0,
        get(3)?,
        get(4)?,
        get(6)?,
        saturation_mode,
        saturation_flavor,
        saturation_mix,
        if get(11)? != 0.0 { get(2)? } else { 0.0 },
        if get(15)? != 0.0 { get(8)? } else { 0.0 },
        if saturation { get(18)? } else { 0.0 },
        if saturation { get(19)? } else { 0.0 },
        if saturation { get(20)? } else { 0.0 },
    ];
    let words = if saturation { &words[..] } else { &words[..Params::PLAIN_WORDS] };
    let p = Params::from_words(words).ok_or_else(|| bad(format!("values out of range: {words:?}")))?;
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
            preset_extra: Default::default(),
            strip_properties: Vec::new(),
            property_files: Vec::new(),
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
            Params {
                lookahead_s: 0.00025,
                hold_s: 0.002,
                threshold_db: -1.5,
                min_fast_gain_db: 12.0,
                true_peak: true,
                saturation_mode: 0,
                saturation_flavor: 0,
                saturation_mix: 0.0,
                deesser_hz: 0.0,
                deesser_db: 0.0,
                deesser_q: 0.0,
                input_gain_db: 0.0,
                output_gain_db: 0.0,
            }
        );
        let d = compile_of(&[]).unwrap().unwrap();
        assert_eq!(
            (d.lookahead_s, d.hold_s, d.threshold_db, d.min_fast_gain_db, d.true_peak),
            (0.0005, 0.01, 0.0, 3.0, false)
        );
    }

    #[test]
    fn carries_enabled_gain_stages_and_ignores_disabled_values() {
        let enabled = compile_of(&[(2, 3.0), (8, -2.0)]).unwrap().unwrap();
        assert_eq!((enabled.input_gain_db, enabled.output_gain_db), (3.0, -2.0));
        let disabled = compile_of(&[(2, 3.0), (11, 0.0), (8, -2.0), (15, 0.0)]).unwrap().unwrap();
        assert_eq!((disabled.input_gain_db, disabled.output_gain_db), (0.0, 0.0));
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
            [(6, 0.5), (0, 0.0)],
            [(7, 5.0), (0, 0.0)],
        ] {
            assert!(compile_of(&refused).is_err(), "{refused:?}");
        }
        assert!(compile_of(&[(2, 3.0), (11, 0.0), (8, -2.0), (15, 0.0)]).unwrap().is_some());
        let saturated = compile_of(&[
            (1, 3.0),
            (10, 2.0),
            (16, 1.0),
            (17, 30.0),
            (18, 10_000.0),
            (19, 10.0),
            (20, 1.0),
            (21, 2.0),
            (23, 1.0),
        ])
        .unwrap()
        .unwrap();
        assert_eq!((saturated.saturation_mode, saturated.saturation_flavor, saturated.saturation_mix), (3, 2, 0.3));
        assert_eq!((saturated.deesser_hz, saturated.deesser_db, saturated.deesser_q), (10_000.0, 10.0, 1.0));
        for refused in [[(1, 0.0)], [(1, 8.0)], [(10, 1.0)], [(23, 0.0)]] {
            let mut st = vec![
                (1, 3.0),
                (10, 2.0),
                (16, 1.0),
                (17, 30.0),
                (18, 10_000.0),
                (19, 10.0),
                (20, 1.0),
                (21, 2.0),
                (23, 1.0),
            ];
            st.retain(|(id, _)| *id != refused[0].0);
            st.push(refused[0]);
            assert!(compile_of(&st).is_err(), "{refused:?}");
        }
        assert!(compile_of(&[(99, 1.0)]).unwrap().is_some());
        let mut st = stage(&[]);
        st.state = None;
        assert!(compile(&st, 48_000.0, &mut Diagnostics::default()).is_err());
    }
}
