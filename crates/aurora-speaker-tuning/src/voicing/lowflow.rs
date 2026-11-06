// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compile AULowFlow (`aufx/flow/appl`) only when mitigation is disabled.

use crate::error::{Diagnostics, Error, Result};
use crate::tuning::Stage;
use sgv2_core::voicing::lowflow::Params;

const RATE: f64 = 48_000.0;

fn parameter(stage: &Stage, id: u32) -> Result<f64> {
    let global = stage
        .state
        .as_ref()
        .and_then(|state| state.global())
        .ok_or_else(|| Error::missing(&stage.name, "global parameter section"))?;
    global.get(id).map(f64::from).ok_or_else(|| Error::missing(&stage.name, &format!("parameter {id}")))
}

fn boolean(stage: &Stage, id: u32) -> Result<bool> {
    match parameter(stage, id)? {
        0.0 => Ok(false),
        1.0 => Ok(true),
        value => Err(Error::range(&stage.name, &format!("parameter {id}"), format!("is {value}, expected 0 or 1"))),
    }
}

pub fn compile(stage: &Stage, fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    if stage.bypass || boolean(stage, 0)? {
        diags.info("lowflow-bypassed", &stage.name, "Audio Unit bypass is identity; box omitted");
        return Ok(None);
    }
    if fs != RATE {
        return Err(Error::range(&stage.name, "sample rate", format!("is {fs}, only {RATE} Hz is specified")));
    }
    let bottom_enabled = boolean(stage, 8)?;
    let bottom_mute = boolean(stage, 9)?;
    let top_enabled = boolean(stage, 32)?;
    let top_mute = boolean(stage, 33)?;
    if bottom_mute || top_mute {
        return Err(Error::version(&stage.name, "LowFlow mute behavior is not carried"));
    }
    if bottom_enabled || top_enabled {
        return Err(Error::version(
            &stage.name,
            "active property-6620 velocity/masking law is not exact; refusing instead of approximating",
        ));
    }
    diags.info("lowflow-delay-only", &stage.name, "both mitigation paths disabled; compiling fixed 256-frame delay");
    Ok(Some(Params))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState};

    fn stage(bypass: bool, bottom: f32, top: f32) -> Stage {
        Stage {
            name: "LowFlow".into(),
            kind: "aufx/flow/appl".into(),
            unit: None,
            inputs: 1,
            outputs: 1,
            channels: Some(4),
            bypass,
            preset_name: Some("synthetic".into()),
            state: Some(ParamState {
                sections: vec![ParamSection {
                    scope: 0,
                    element: 0,
                    params: vec![(0, 0.0), (8, bottom), (9, 0.0), (32, top), (33, 0.0)],
                }],
            }),
            preset_extra_keys: Vec::new(),
            before_protection: true,
        }
    }

    #[test]
    fn only_bypass_or_disabled_mitigation_compiles() {
        assert_eq!(compile(&stage(true, 1.0, 1.0), RATE, &mut Diagnostics::default()).unwrap(), None);
        assert_eq!(compile(&stage(false, 0.0, 0.0), RATE, &mut Diagnostics::default()).unwrap(), Some(Params));
        assert!(compile(&stage(false, 1.0, 1.0), RATE, &mut Diagnostics::default()).is_err());
    }
}
