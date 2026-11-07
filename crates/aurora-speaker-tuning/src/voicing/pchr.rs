// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compile the exact two-channel AUMidSide/channel-matrix saved state.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::tuning::Stage;
use sgv2_core::voicing::pchr::Params;
use sgv2_core::MAX_CHANNELS;

pub const PCHR: FourCc = FourCc::from_bytes(*b"pchr");
const AUFX: FourCc = FourCc::from_bytes(*b"aufx");
const APPL: FourCc = FourCc::from_bytes(*b"appl");

fn bad(stage: &Stage, message: impl Into<String>) -> Error {
    Error::format(&stage.name, message)
}

pub fn compile(stage: &Stage, _fs: f64, _diags: &mut Diagnostics) -> Result<Option<Params>> {
    if stage.bypass {
        return Ok(None);
    }
    let unit = stage.unit.ok_or_else(|| Error::missing(&stage.name, "Audio Unit identity"))?;
    if unit.kind != AUFX || unit.subtype != PCHR || unit.manufacturer != APPL {
        return Err(bad(stage, format!("unit is {unit}, expected aufx/pchr/appl")));
    }
    if stage.channels != Some(2) {
        return Err(bad(stage, format!("box has {:?} channels, expected stereo", stage.channels)));
    }
    let state = stage.state.as_ref().ok_or_else(|| Error::missing(&stage.name, "saved state"))?;
    if state.sections.len() != 1 {
        return Err(bad(
            stage,
            format!("saved state has {} sections, expected one global section", state.sections.len()),
        ));
    }
    let global = state.global().ok_or_else(|| Error::missing(&stage.name, "global parameter section"))?;
    let mut actual: Vec<u32> = global.params.iter().map(|(id, _)| *id).collect();
    actual.sort_unstable();
    if actual != (0..4).collect::<Vec<_>>() {
        return Err(bad(stage, format!("parameter IDs are {actual:?}, expected [0, 1, 2, 3]")));
    }
    let mut matrix = [[0.0; MAX_CHANNELS]; MAX_CHANNELS];
    for i in 0..4 {
        let value = f64::from(global.get(i as u32).unwrap());
        if !(-1.0..=1.0).contains(&value) {
            return Err(Error::range(&stage.name, &format!("parameter {i}"), format!("is {value}, expected -1..=1")));
        }
        matrix[i / 2][i % 2] = value;
    }
    Ok(Some(Params { channels: 2.0, matrix }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState, UnitId};

    fn stage() -> Stage {
        Stage {
            name: "toMS".into(),
            kind: "aufx/pchr/appl".into(),
            unit: Some(UnitId { kind: AUFX, subtype: PCHR, manufacturer: APPL }),
            inputs: 1,
            outputs: 1,
            channels: Some(2),
            bypass: false,
            preset_name: Some("synthetic".into()),
            state: Some(ParamState {
                sections: vec![ParamSection {
                    scope: 0,
                    element: 0,
                    params: vec![(0, 1.0), (1, 1.0), (2, 1.0), (3, -1.0)],
                }],
            }),
            preset_extra_keys: Vec::new(),
            preset_extra: Default::default(),
            strip_properties: Vec::new(),
            property_files: Vec::new(),
            before_protection: true,
        }
    }

    #[test]
    fn compiles_matrix_and_refuses_unrepresented_state() {
        let p = compile(&stage(), 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        assert_eq!([p.matrix[0][0], p.matrix[0][1], p.matrix[1][0], p.matrix[1][1]], [1.0, 1.0, 1.0, -1.0]);
        let mut s = stage();
        s.state.as_mut().unwrap().sections[0].params.push((4, 0.0));
        assert!(compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap_err().to_string().contains("parameter IDs"));
        let mut s = stage();
        s.channels = Some(1);
        assert!(compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap_err().to_string().contains("stereo"));
    }
}
