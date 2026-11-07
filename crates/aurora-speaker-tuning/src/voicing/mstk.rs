// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compile the public saved state of `aufx/mstk/appl`.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::plist::Value;
use crate::tuning::Stage;
use sgv2_core::voicing::mstk::{
    AutomationPoint, Band, BandMode, Params, ThresholdAutomation, BANDS, MAX_AUTOMATION_POINTS,
};

pub const MSTK: FourCc = FourCc::from_bytes(*b"mstk");

fn bad(stage: &Stage, message: impl Into<String>) -> Error {
    Error::format("voicing/mstk", format!("compressor box {}: {}", stage.name, message.into()))
}

fn flag(stage: &Stage, id: u32, value: f64) -> Result<bool> {
    if value == 0.0 {
        Ok(false)
    } else if value == 1.0 {
        Ok(true)
    } else {
        Err(bad(stage, format!("parameter {id} is {value}, not a switch")))
    }
}

fn threshold_automation(stage: &Stage, input_min: f64, input_max: f64) -> Result<[ThresholdAutomation; BANDS]> {
    let mut out = [ThresholdAutomation::default(); BANDS];
    let Some(value) = stage.preset_extra.get("automation") else { return Ok(out) };
    let inputs = value.as_dict().ok_or_else(|| bad(stage, "automation is not a dictionary"))?;
    if inputs.len() != 1 || inputs.get("0").is_none() {
        return Err(bad(stage, "automation must contain only input 0"));
    }
    let targets =
        inputs.get("0").and_then(Value::as_dict).ok_or_else(|| bad(stage, "automation input 0 is not a dictionary"))?;
    for (target, value) in targets.iter() {
        let target =
            target.parse::<u32>().map_err(|_| bad(stage, format!("automation target {target:?} is not numeric")))?;
        let Some(band) = (0..BANDS).find(|band| target == 4 + 25 * *band as u32) else {
            return Err(bad(stage, format!("automation target {target} is not a downward-threshold parameter")));
        };
        let points =
            value.as_array().ok_or_else(|| bad(stage, format!("automation target {target} is not an array")))?;
        if !(2..=MAX_AUTOMATION_POINTS).contains(&points.len()) {
            return Err(bad(
                stage,
                format!("automation target {target} has {} points, expected 2..={MAX_AUTOMATION_POINTS}", points.len()),
            ));
        }
        let mut curve = ThresholdAutomation { count: points.len(), ..ThresholdAutomation::default() };
        for (i, point) in points.iter().enumerate() {
            let pair = point
                .as_array()
                .filter(|pair| pair.len() == 2)
                .ok_or_else(|| bad(stage, format!("automation target {target} point {i} is not a pair")))?;
            let input = pair[0]
                .as_number()
                .filter(|v| v.is_finite() && (input_min..=input_max).contains(v))
                .ok_or_else(|| bad(stage, format!("automation target {target} point {i} has invalid input")))?;
            let threshold = pair[1]
                .as_number()
                .filter(|v| v.is_finite() && (-120.0..=0.0).contains(v))
                .ok_or_else(|| bad(stage, format!("automation target {target} point {i} has invalid threshold")))?;
            if i != 0 && input <= curve.points[i - 1].input {
                return Err(bad(stage, format!("automation target {target} inputs are not strictly increasing")));
            }
            curve.points[i] = AutomationPoint { input, value: threshold };
        }
        out[band] = curve;
    }
    Ok(out)
}

/// Compile one graph stage.  Part-scope sections are the unit's identical
/// element defaults; a non-identical part section is refused rather than
/// mistaken for an active volume/channel curve.
pub fn compile(stage: &Stage, fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    if stage.unit.map(|u| u.subtype) != Some(MSTK) {
        return Err(bad(stage, "is not aufx/mstk"));
    }
    if stage.bypass {
        return Ok(None);
    }
    let state = stage.state.as_ref().ok_or_else(|| bad(stage, "has no saved state"))?;
    let g = state.global().ok_or_else(|| bad(stage, "has no global saved state"))?;
    let req = |id: u32| -> Result<f64> {
        g.get(id)
            .map(f64::from)
            .filter(|v| v.is_finite())
            .ok_or_else(|| bad(stage, format!("parameter {id} is missing")))
    };
    let opt = |id: u32, default: f64| g.get(id).map(f64::from).unwrap_or(default);
    if flag(stage, 0, req(0)?)? {
        return Ok(None);
    }

    let parts = state.elements(4);
    if let Some(first) = parts.first() {
        if parts.iter().any(|p| p.params != first.params) {
            return Err(bad(stage, "part-scope parameter sections differ between elements"));
        }
        diags.info(
            "mstk-part-defaults",
            stage.name.clone(),
            "identical part-scope element defaults are not an active per-channel curve",
        );
    }

    let mut solo = [false; BANDS];
    for (b, s) in solo.iter_mut().enumerate() {
        *s = flag(stage, 12 + 13 * b as u32, req(12 + 13 * b as u32)?)?;
    }
    let any_solo = solo.iter().any(|v| *v);
    let mut bands = [Band::default(); BANDS];
    for (b, out) in bands.iter_mut().enumerate() {
        let o = 10 + 13 * b as u32;
        let enabled = flag(stage, o, req(o)?)?;
        let bypass = flag(stage, o + 1, req(o + 1)?)?;
        let mute = flag(stage, o + 3, req(o + 3)?)?;
        let sat_on = flag(stage, 71 + 2 * b as u32, opt(71 + 2 * b as u32, 0.0))?;
        let flavor = opt(72 + 2 * b as u32, 0.0);
        if flavor.fract() != 0.0 || !(0.0..=4.0).contains(&flavor) {
            return Err(bad(stage, format!("band {b} saturation flavor {flavor}")));
        }
        let mode = if mute || (any_solo && !solo[b]) {
            BandMode::Mute
        } else if !enabled || bypass {
            BandMode::Pass
        } else {
            BandMode::Active
        };
        *out = Band {
            mode,
            down_threshold_db: req(o + 4)?,
            down_ratio: req(o + 5)?,
            down_range_db: req(o + 6)?,
            up_threshold_db: req(o + 7)?,
            up_ratio: req(o + 8)?,
            up_range_db: req(o + 9)?,
            exp_threshold_db: req(o + 10)?,
            exp_ratio: req(o + 11)?,
            makeup_db: req(o + 12)?,
            input_gain_db: opt(65 + b as u32, 0.0),
            saturation: sat_on.then_some(flavor as u32),
        };
    }
    let integer = |id: u32, default: f64, hi: u32| -> Result<u32> {
        let v = opt(id, default);
        if v.fract() == 0.0 && (0.0..=hi as f64).contains(&v) {
            Ok(v as u32)
        } else {
            Err(bad(stage, format!("parameter {id} is {v}, expected 0..{hi}")))
        }
    };
    let automation_input = opt(79, 0.0);
    let automation_min = opt(80, -100.0);
    let automation_max = opt(81, 0.0);
    let p = Params {
        crossover_hz: [req(4)?, req(6)?, req(8)?],
        crossover_order: [req(5)?, req(7)?, req(9)?],
        input_gain_db: req(1)?,
        output_gain_db: req(2)?,
        bandwidth_compensation: flag(stage, 3, req(3)?)?,
        release_s: req(62)? / 1000.0,
        release_on: flag(stage, 63, req(63)?)?,
        render_quality: integer(64, 0.0, 2)?,
        overshoot_compensation: flag(stage, 69, opt(69, 0.0))?,
        algorithm: integer(70, 0.0, 3)?,
        automation_input,
        automation_min,
        automation_max,
        threshold_automation: threshold_automation(stage, automation_min, automation_max)?,
        band: bands,
    };
    let mut words = [0.0; Params::WORDS];
    p.to_words(&mut words);
    let decoded =
        Params::from_words(&words).ok_or_else(|| bad(stage, format!("saved controls are invalid at {fs} Hz")))?;
    if p != decoded {
        return Err(bad(stage, "saved controls do not have an exact wire representation"));
    }
    Ok(Some(p))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState, UnitId};
    use crate::plist::{Dict, Value};

    fn stage(extra: &[(u32, f32)]) -> Stage {
        let mut params = vec![
            (0, 0.0),
            (1, -12.0),
            (2, 14.0),
            (3, 1.0),
            (4, 200.0),
            (5, 4.0),
            (6, 1500.0),
            (7, 4.0),
            (8, 8000.0),
            (9, 4.0),
        ];
        for b in 0..4u32 {
            let o = 10 + 13 * b;
            params.extend_from_slice(&[
                (o, 1.0),
                (o + 1, 0.0),
                (o + 2, 0.0),
                (o + 3, 0.0),
                (o + 4, -30.0),
                (o + 5, 4.0),
                (o + 6, -20.0),
                (o + 7, -50.0),
                (o + 8, 1.0),
                (o + 9, 10.0),
                (o + 10, -90.0),
                (o + 11, 1.0),
                (o + 12, 0.0),
            ]);
        }
        params.extend_from_slice(&[(62, 50.0), (63, 1.0), (64, 1.0)]);
        for &(id, value) in extra {
            if let Some(p) = params.iter_mut().find(|p| p.0 == id) {
                p.1 = value
            } else {
                params.push((id, value))
            }
        }
        Stage {
            name: "synthetic compressor".into(),
            kind: "aufx/mstk/appl".into(),
            unit: Some(UnitId {
                kind: FourCc::from_bytes(*b"aufx"),
                subtype: MSTK,
                manufacturer: FourCc::from_bytes(*b"appl"),
            }),
            inputs: 1,
            outputs: 1,
            channels: Some(2),
            bypass: false,
            preset_name: None,
            state: Some(ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] }),
            preset_extra_keys: vec![],
            preset_extra: Default::default(),
            strip_properties: Vec::new(),
            property_files: Vec::new(),
            before_protection: true,
        }
    }

    #[test]
    fn compiles_every_branch_and_extended_control() {
        let s = stage(&[
            (18, 2.0),
            (21, 1.5),
            (65, -3.0),
            (69, 1.0),
            (70, 3.0),
            (71, 1.0),
            (72, 4.0),
            (79, -16.0),
            (80, -100.0),
            (81, 0.0),
        ]);
        let p = compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        assert_eq!((p.band[0].up_ratio, p.band[0].exp_ratio, p.band[0].input_gain_db), (2.0, 1.5, -3.0));
        assert_eq!(p.band[0].saturation, Some(4));
        assert!(p.overshoot_compensation);
    }

    #[test]
    fn compiles_generic_threshold_automation_curves() {
        let mut s = stage(&[(79, -16.0), (80, -100.0), (81, 0.0)]);
        let pair = |input: f64, threshold: f64| Value::Array(vec![Value::Real(input), Value::Real(threshold)]);
        let mut targets = Dict::new();
        assert!(targets
            .insert_unique("4".into(), Value::Array(vec![pair(-30.0, -10.0), pair(-10.0, -20.0), pair(0.0, -24.0)]),));
        let mut inputs = Dict::new();
        assert!(inputs.insert_unique("0".into(), Value::Dict(targets)));
        assert!(s.preset_extra.insert_unique("automation".into(), Value::Dict(inputs)));

        let p = compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        assert_eq!(p.threshold_automation[0].count, 3);
        assert_eq!(p.threshold_automation[0].points[1], AutomationPoint { input: -10.0, value: -20.0 });
        assert!(p.threshold_automation[1..].iter().all(|curve| curve.count == 0));
    }

    #[test]
    fn bypass_solo_and_bad_part_scope() {
        let mut s = stage(&[(12, 1.0)]);
        let p = compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        assert_eq!(p.band[1].mode, BandMode::Mute);
        s.state.as_mut().unwrap().global().unwrap();
        s.bypass = true;
        assert!(compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap().is_none());
    }

    #[test]
    fn accepts_fractional_and_odd_orders() {
        let s = stage(&[(5, 3.0), (7, 4.5)]);
        let p = compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        assert_eq!(p.crossover_order, [3.0, 4.5, 4.0]);
    }
}
