// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compile AUVolumeTaper's custom, non-ramping saved-state subset.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::tuning::Stage;
use sgv2_core::voicing::volt::{Params, CURVE_POINTS, OFFSET_POINTS};
use sgv2_core::MAX_CHANNELS;

pub const VOLT: FourCc = FourCc::from_bytes(*b"volt");
const AUFX: FourCc = FourCc::from_bytes(*b"aufx");
const APPL: FourCc = FourCc::from_bytes(*b"appl");

fn bad(stage: &Stage, message: impl Into<String>) -> Error {
    Error::format(&stage.name, message)
}

fn global(stage: &Stage) -> Result<&crate::aupreset::ParamSection> {
    let state = stage.state.as_ref().ok_or_else(|| Error::missing(&stage.name, "saved state"))?;
    if state.sections.len() != 1 {
        return Err(bad(
            stage,
            format!("saved state has {} sections, expected one global section", state.sections.len()),
        ));
    }
    state.global().ok_or_else(|| Error::missing(&stage.name, "global parameter section"))
}

fn parameter(section: &crate::aupreset::ParamSection, stage: &Stage, id: u32) -> Result<f64> {
    section.get(id).map(f64::from).ok_or_else(|| Error::missing(&stage.name, &format!("parameter {id}")))
}

fn in_range(section: &crate::aupreset::ParamSection, stage: &Stage, id: u32, lo: f64, hi: f64) -> Result<f64> {
    let value = parameter(section, stage, id)?;
    if (lo..=hi).contains(&value) {
        Ok(value)
    } else {
        Err(Error::range(&stage.name, &format!("parameter {id}"), format!("is {value}, expected {lo}..={hi}")))
    }
}

/// Compiles the exact custom-curve subset used by the speaker graphs. The
/// graph-owned slider (ID 4) is supplied at run time through `Run::set_volume`.
pub fn compile(stage: &Stage, _fs: f64, _diags: &mut Diagnostics) -> Result<Option<Params>> {
    if stage.bypass {
        return Ok(None);
    }
    let unit = stage.unit.ok_or_else(|| Error::missing(&stage.name, "Audio Unit identity"))?;
    if unit.kind != AUFX || unit.subtype != VOLT || unit.manufacturer != APPL {
        return Err(bad(stage, format!("unit is {unit}, expected aufx/volt/appl")));
    }
    if !matches!(stage.channels, Some(channels) if (1..=MAX_CHANNELS as u32).contains(&channels)) {
        return Err(bad(stage, format!("box has {:?} channels, expected 1..={MAX_CHANNELS}", stage.channels)));
    }
    let section = global(stage)?;
    let mut expected: Vec<u32> = (0..=7).collect();
    expected.extend(1024..1024 + CURVE_POINTS as u32);
    expected.extend(2048..=2050);
    expected.extend(3072..3072 + OFFSET_POINTS as u32);
    let mut actual: Vec<u32> = section.params.iter().map(|(id, _)| *id).collect();
    actual.sort_unstable();
    if actual != expected {
        return Err(bad(stage, format!("parameter IDs are {actual:?}, expected the complete custom-curve state")));
    }

    if parameter(section, stage, 0)? != 0.0 {
        return Err(bad(stage, "Mute (parameter 0) is active"));
    }
    if parameter(section, stage, 1)? != 0.0 {
        return Err(bad(stage, "Ramp (parameter 1) is active; the runner carries only the non-ramping state"));
    }
    in_range(section, stage, 2, 0.01, 60_000.0)?;
    in_range(section, stage, 3, 0.01, 60_000.0)?;
    let saved_scalar = in_range(section, stage, 4, 0.0, 1.0)?;
    let pre_gain_db = in_range(section, stage, 5, -120.0, 20.0)?;
    let auto_mute_db = in_range(section, stage, 6, -240.0, 0.0)?;
    match parameter(section, stage, 7)? {
        0.0 | 1.0 => {}
        value => return Err(bad(stage, format!("SlewRateUnit (parameter 7) is {value}, expected 0 or 1"))),
    }
    if parameter(section, stage, 2048)? != 0.0 {
        return Err(bad(stage, "CurveType (parameter 2048) is not the custom 17-point curve"));
    }
    // Parameters 2049/2050 select the analytic laws and are dormant for
    // CurveType 0. Still require values admitted by the unit's API.
    in_range(section, stage, 2049, -100.0, 100.0)?;
    in_range(section, stage, 2050, -100.0, 100.0)?;

    let mut curve_db = [0.0; CURVE_POINTS];
    for (i, value) in curve_db.iter_mut().enumerate() {
        *value = in_range(section, stage, 1024 + i as u32, -120.0, 20.0)?;
    }
    let mut offset_db = [0.0; OFFSET_POINTS];
    for (i, value) in offset_db.iter_mut().enumerate() {
        *value = in_range(section, stage, 3072 + i as u32, -120.0, 20.0)?;
    }
    // AutoMuteThreshold is dormant only when even the conservative lower
    // bound of the carried curve stays above it.
    let lower_bound = pre_gain_db
        + curve_db.iter().copied().fold(f64::INFINITY, f64::min)
        + offset_db.iter().copied().fold(f64::INFINITY, f64::min);
    if auto_mute_db >= lower_bound {
        return Err(bad(
            stage,
            format!("AutoMuteThreshold {auto_mute_db} dB can engage (carried curve lower bound {lower_bound} dB)"),
        ));
    }
    Ok(Some(Params { saved_scalar, pre_gain_db, curve_db, offset_db }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState, UnitId};

    fn stage() -> Stage {
        let mut params = vec![(0, 0.0), (1, 0.0), (2, 1.0), (3, 1.0), (4, 0.375), (5, 0.0), (6, -99.0), (7, 0.0)];
        for i in 0..CURVE_POINTS {
            params.push((1024 + i as u32, -60.0 + 60.0 * i as f32 / 16.0));
        }
        params.extend([(2048, 0.0), (2049, 40.0), (2050, -40.0)]);
        for i in 0..OFFSET_POINTS {
            params.push((3072 + i as u32, 0.0));
        }
        Stage {
            name: "VT".into(),
            kind: "aufx/volt/appl".into(),
            unit: Some(UnitId { kind: AUFX, subtype: VOLT, manufacturer: APPL }),
            inputs: 1,
            outputs: 1,
            channels: Some(2),
            bypass: false,
            preset_name: Some("synthetic".into()),
            state: Some(ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] }),
            preset_extra_keys: Vec::new(),
            before_protection: true,
        }
    }

    #[test]
    fn compiles_complete_custom_state() {
        let p = compile(&stage(), 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        assert_eq!((p.saved_scalar, p.curve_db[0], p.curve_db[16]), (0.375, -60.0, 0.0));
        let mut mono = stage();
        mono.channels = Some(1);
        assert!(compile(&mono, 48_000.0, &mut Diagnostics::default()).unwrap().is_some());
    }

    #[test]
    fn refuses_ramping_unknown_ids_and_live_auto_mute() {
        let mut s = stage();
        s.state.as_mut().unwrap().sections[0].params.iter_mut().find(|p| p.0 == 1).unwrap().1 = 1.0;
        assert!(compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap_err().to_string().contains("Ramp"));
        let mut s = stage();
        s.state.as_mut().unwrap().sections[0].params.push((999, 0.0));
        assert!(compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap_err().to_string().contains("parameter IDs"));
        let mut s = stage();
        s.state.as_mut().unwrap().sections[0].params.iter_mut().find(|p| p.0 == 6).unwrap().1 = -50.0;
        assert!(compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap_err().to_string().contains("AutoMute"));
    }
}
