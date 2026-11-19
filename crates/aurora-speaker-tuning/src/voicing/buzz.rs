// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compile BuzzKill (`aufx/buzk/appl`).

use crate::error::{Diagnostics, Error, Result};
use crate::tuning::Stage;
use sgv2_core::voicing::buzz::{Band, Params, BANDS};

const BUZK: crate::fourcc::FourCc = crate::fourcc::FourCc::from_bytes(*b"buzk");

fn default(id: u32) -> Option<f64> {
    const FREQUENCY: [f64; BANDS] = [20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0, 10000.0];
    Some(match id {
        0 | 1 | 4 | 8 | 12 | 16 | 20 | 24 | 28 | 32 | 36 | 40 | 44 => 0.0,
        2 => 15.0,
        3 => -20.0,
        id if (4..44).contains(&id) => {
            let band = ((id - 4) / 4) as usize;
            match (id - 4) % 4 {
                1 => FREQUENCY[band],
                2 => 0.0,
                3 => 1.0,
                _ => 0.0,
            }
        }
        _ => return None,
    })
}

pub fn compile(stage: &Stage, fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    let context = format!("BuzzKill box {}", stage.name);
    let bad = |message: String| Error::format(&context, message);
    if stage.unit.map(|unit| unit.subtype) != Some(BUZK) {
        return Err(bad(format!("is {}, not a buzk unit", stage.kind)));
    }
    if stage.bypass {
        diags.info("buzz-bypassed", &stage.name, "strip bypass is exact identity; box omitted");
        return Ok(None);
    }
    if !(fs.is_finite() && fs > 0.0) {
        return Err(bad(format!("sample rate {fs}")));
    }
    let global =
        stage.state.as_ref().and_then(|state| state.global()).ok_or_else(|| bad("has no global saved state".into()))?;
    let get = |id: u32| -> Result<f64> {
        match global.get(id) {
            Some(value) if value.is_finite() => Ok(f64::from(value)),
            Some(_) => Err(bad(format!("parameter {id} is not finite"))),
            None => default(id).ok_or_else(|| bad(format!("parameter {id} is not carried"))),
        }
    };
    if let Some(&(id, _)) = global.params.iter().find(|(id, _)| *id > 44) {
        return Err(bad(format!("parameter {id} is not carried")));
    }
    let integer = |id: u32, max: u32| -> Result<u32> {
        let value = get(id)?;
        if value >= 0.0 && value <= f64::from(max) && value.trunc() == value {
            Ok(value as u32)
        } else {
            Err(bad(format!("parameter {id} is {value}, expected an integer 0..={max}")))
        }
    };
    if get(0)? != 0.0 {
        return Ok(None);
    }
    let mut params = Params {
        operation_frequency: integer(1, 4)?,
        lookahead_s: get(2)? / 1000.0,
        threshold_db: get(3)?,
        algorithm: integer(44, 3)?,
        bands: [Band::default(); BANDS],
    };
    for (index, band) in params.bands.iter_mut().enumerate() {
        let id = 4 + index as u32 * 4;
        let enabled = get(id)?;
        if enabled != 0.0 && enabled != 1.0 {
            return Err(bad(format!("Kill parameter {id} is {enabled}")));
        }
        *band =
            Band { enabled: enabled != 0.0, frequency_hz: get(id + 1)?, gain_db: get(id + 2)?, quality: get(id + 3)? };
    }
    let mut words = [0.0; Params::WORDS];
    params.to_words(&mut words);
    let params =
        Params::from_words(&words).ok_or_else(|| bad("saved parameters are outside the carried domain".into()))?;
    if params.algorithm != 0 {
        return Err(Error::version(&stage.name, format!("BuzzKill algorithm {} is not carried", params.algorithm)));
    }
    Ok(Some(params))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fourcc::FourCc;
    fn stage(bypass: bool) -> Stage {
        Stage {
            name: "Buzz".into(),
            kind: "aufx/buzk/appl".into(),
            unit: Some(crate::aupreset::UnitId {
                kind: FourCc::from_bytes(*b"aufx"),
                subtype: FourCc::from_bytes(*b"buzk"),
                manufacturer: FourCc::from_bytes(*b"appl"),
            }),
            inputs: 1,
            outputs: 1,
            channels: Some(2),
            bypass,
            preset_name: Some("synthetic".into()),
            state: Some(crate::aupreset::ParamState {
                sections: vec![crate::aupreset::ParamSection {
                    scope: 0,
                    element: 0,
                    params: vec![(2, 8.0), (3, -12.0), (12, 1.0), (13, 160.0), (14, -10.0), (15, 2.0)],
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
    fn compiles_active_version_zero_and_bypass() {
        let mut d = Diagnostics::default();
        assert_eq!(compile(&stage(true), 48_000.0, &mut d).unwrap(), None);
        let p = compile(&stage(false), 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        assert_eq!(p.lookahead_s, 0.008);
        assert_eq!(p.threshold_db, -12.0);
        assert!(p.bands[2].enabled);
    }
}
