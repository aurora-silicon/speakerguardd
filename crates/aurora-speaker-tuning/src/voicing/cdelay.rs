// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compile ChannelSpecificDelay (`aufx/cdly/appl`).

use crate::error::{Diagnostics, Error, Result};
use crate::tuning::Stage;
use sgv2_core::voicing::cdelay::{Params, MAX_DELAY_FRAMES};
use sgv2_core::MAX_CHANNELS;

const RATE: f64 = 48_000.0;
const PARAM_SLOTS: usize = 8;
const PART_SLOTS: usize = 32;

fn bad(stage: &Stage, message: impl Into<String>) -> Error {
    Error::format(&stage.name, message)
}

/// Compiles the public millisecond controls to the whole-frame delays the
/// installed unit renders.  Only the 48 kHz profile used by the speaker
/// graphs is admitted.
pub fn compile(stage: &Stage, fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    if stage.bypass {
        diags.info("cdelay-bypassed", &stage.name, "strip bypass is sample-exact identity; box omitted");
        return Ok(None);
    }
    if fs != RATE {
        return Err(Error::range(&stage.name, "sample rate", format!("is {fs}, only {RATE} Hz is specified")));
    }
    let channels = stage.channels.ok_or_else(|| Error::missing(&stage.name, "channel count"))? as usize;
    if !(1..=MAX_CHANNELS).contains(&channels) {
        return Err(Error::range(&stage.name, "channel count", format!("is {channels}, expected 1..={MAX_CHANNELS}")));
    }
    let state = stage.state.as_ref().ok_or_else(|| Error::missing(&stage.name, "saved state"))?;
    let global = state.global().ok_or_else(|| Error::missing(&stage.name, "global parameter section"))?;
    if global.params.len() != PARAM_SLOTS
        || global.params.iter().enumerate().any(|(id, (actual, _))| *actual != id as u32)
    {
        return Err(bad(stage, "expected exactly global delay parameters 0 through 7"));
    }
    let mut delay_frames = [0; MAX_CHANNELS];
    for (slot, (_, value)) in global.params.iter().enumerate() {
        let ms = f64::from(*value);
        if !ms.is_finite() || !(0.0..=100.0).contains(&ms) {
            return Err(Error::range(&stage.name, &format!("delay parameter {slot}"), format!("is {ms} ms")));
        }
        if slot >= channels {
            if ms != 0.0 {
                return Err(bad(stage, format!("unused delay parameter {slot} is nonzero")));
            }
            continue;
        }
        let frames = (ms * RATE / 1000.0).round() as usize;
        if frames > MAX_DELAY_FRAMES {
            return Err(Error::range(&stage.name, &format!("delay parameter {slot}"), "exceeds the frame bound"));
        }
        delay_frames[slot] = frames;
    }
    let parts = state.elements(4);
    if parts.len() != PART_SLOTS
        || parts
            .iter()
            .enumerate()
            .any(|(element, section)| section.element != element as u32 || section.params.as_slice() != [(0, 0.0)])
        || state.sections.len() != PART_SLOTS + 1
    {
        return Err(bad(stage, "unexpected part-scope state; expected 32 zero-valued compatibility slots"));
    }
    Ok(Some(Params { channels, delay_frames }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState, UnitId};
    use crate::fourcc::FourCc;

    fn stage(bypass: bool, values: [f32; PARAM_SLOTS]) -> Stage {
        let mut sections = vec![ParamSection {
            scope: 0,
            element: 0,
            params: values.into_iter().enumerate().map(|(id, value)| (id as u32, value)).collect(),
        }];
        sections.extend((0..PART_SLOTS).map(|element| ParamSection {
            scope: 4,
            element: element as u32,
            params: vec![(0, 0.0)],
        }));
        Stage {
            name: "Delay".into(),
            kind: "aufx/cdly/appl".into(),
            unit: Some(UnitId {
                kind: FourCc::from_bytes(*b"aufx"),
                subtype: FourCc::from_bytes(*b"cdly"),
                manufacturer: FourCc::from_bytes(*b"appl"),
            }),
            inputs: 1,
            outputs: 1,
            channels: Some(2),
            bypass,
            preset_name: Some("synthetic".into()),
            state: Some(ParamState { sections }),
            preset_extra_keys: Vec::new(),
            before_protection: true,
        }
    }

    #[test]
    fn compiles_milliseconds_with_native_rounding() {
        let p = compile(&stage(false, [1.0, 4.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), RATE, &mut Diagnostics::default())
            .unwrap()
            .unwrap();
        assert_eq!(p.channels, 2);
        assert_eq!(&p.delay_frames[..2], &[48, 206]);
    }

    #[test]
    fn bypass_omits_box_and_unknown_profiles_refuse() {
        let values = [4.3, 4.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert_eq!(compile(&stage(true, values), RATE, &mut Diagnostics::default()).unwrap(), None);
        assert!(compile(&stage(false, values), 96_000.0, &mut Diagnostics::default()).is_err());
        let mut bad = values;
        bad[2] = 1.0;
        assert!(compile(&stage(false, bad), RATE, &mut Diagnostics::default()).is_err());
    }
}
