// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compile AULowFlow from its saved state and property-6620 sidecar.

use crate::error::{Diagnostics, Error, Result};
use crate::plist::{Dict, Value};
use crate::tuning::Stage;
use sgv2_core::voicing::lowflow::{Model, Params, MAX_MODELS, RESPONSE_TAPS};

pub const PROPERTY_SIDECAR: u32 = 6620;
const RESPONSE_BYTES: usize = RESPONSE_TAPS * 4;

fn bad(context: &str, message: impl Into<String>) -> Error {
    Error::format(context, message)
}

fn number(d: &Dict, key: &str, context: &str) -> Result<f64> {
    let v = d.get(key).ok_or_else(|| Error::missing(context, key))?;
    let n = v.as_number().ok_or_else(|| Error::Type {
        context: context.into(),
        field: key.into(),
        expected: "integer or real",
        found: v.kind(),
    })?;
    if !n.is_finite() {
        return Err(Error::range(context, key, "is not finite"));
    }
    Ok(n)
}

fn response(d: &Dict, key: &str, context: &str) -> Result<[f64; RESPONSE_TAPS]> {
    let v = d.get(key).ok_or_else(|| Error::missing(context, key))?;
    match v {
        Value::Data(bytes) if bytes.len() == RESPONSE_BYTES => {
            let mut out = [0.0; RESPONSE_TAPS];
            for (dst, chunk) in out.iter_mut().zip(bytes.chunks_exact(4)) {
                let value = f32::from_le_bytes(chunk.try_into().unwrap());
                if !value.is_finite() {
                    return Err(Error::range(context, key, "contains a non-finite float32 tap"));
                }
                *dst = f64::from(value);
            }
            Ok(out)
        }
        Value::Data(bytes) => Err(bad(context, format!("{key} has {} bytes, expected {RESPONSE_BYTES}", bytes.len()))),
        Value::Array(values) if values.len() == RESPONSE_TAPS => {
            let mut out = [0.0; RESPONSE_TAPS];
            for (index, (dst, value)) in out.iter_mut().zip(values).enumerate() {
                let number = value.as_number().ok_or_else(|| Error::Type {
                    context: context.into(),
                    field: format!("{key}[{index}]"),
                    expected: "integer or real",
                    found: value.kind(),
                })?;
                if !number.is_finite() {
                    return Err(Error::range(context, &format!("{key}[{index}]"), "is not finite"));
                }
                *dst = number;
            }
            Ok(out)
        }
        Value::Array(values) => Err(bad(context, format!("{key} has {} taps, expected {RESPONSE_TAPS}", values.len()))),
        other => Err(Error::Type {
            context: context.into(),
            field: key.into(),
            expected: "data or numeric array",
            found: other.kind(),
        }),
    }
}

fn parameter(stage: &Stage, id: u32) -> Result<f64> {
    let context = format!("{} saved state", stage.name);
    let global = stage
        .state
        .as_ref()
        .and_then(|s| s.global())
        .ok_or_else(|| Error::missing(&context, "global parameter section"))?;
    global.get(id).map(f64::from).ok_or_else(|| Error::missing(&context, &format!("parameter {id}")))
}

fn boolean(stage: &Stage, id: u32) -> Result<bool> {
    match parameter(stage, id)? {
        0.0 => Ok(false),
        1.0 => Ok(true),
        v => Err(bad(&stage.name, format!("parameter {id} is {v}, expected 0 or 1"))),
    }
}

fn in_range(stage: &Stage, id: u32, lo: f64, hi: f64) -> Result<f64> {
    let v = parameter(stage, id)?;
    if v.is_finite() && (lo..=hi).contains(&v) {
        Ok(v)
    } else {
        Err(Error::range(&stage.name, &format!("parameter {id}"), format!("is {v}, expected {lo}..={hi}")))
    }
}

fn model(channel: &Dict, stage: &Stage, index: usize) -> Result<Model> {
    let context = format!("{} property {PROPERTY_SIDECAR} ChannelData[{index}]", stage.name);
    let ts = channel
        .get("ThieleSmall")
        .and_then(Value::as_dict)
        .ok_or_else(|| Error::missing(&context, "ThieleSmall dictionary"))?;
    let pressure_response = response(channel, "PressureResponse", &context)?;
    let tweeter_response = response(channel, "TweeterResponse", &context)?;
    let tweeter_gain = number(channel, "TweeterGain", &context)?;
    if tweeter_gain <= 0.0 {
        return Err(Error::range(&context, "TweeterGain", "must be positive"));
    }
    let offset = if index == 0 { 0 } else { 24 };
    Ok(Model {
        bl: number(ts, "Bl", &context)?,
        kms: number(ts, "Kms", &context)?,
        kms_front_port: number(ts, "KmsFrontPort", &context)?,
        mms: number(ts, "Mms", &context)?,
        mms_front_port: number(ts, "MmsFrontPort", &context)?,
        reb: number(ts, "Reb", &context)?,
        rms: number(ts, "Rms", &context)?,
        rms_front_port: number(ts, "RmsFrontPort", &context)?,
        analog_gain: number(channel, "AnalogGain", &context)?,
        tweeter_gain,
        mask_band_min_hz: in_range(stage, 10 + offset, 0.0, 24_000.0)?,
        mask_band_max_hz: in_range(stage, 11 + offset, 0.0, 24_000.0)?,
        min_velocity: in_range(stage, 12 + offset, 0.001, 2.0)?,
        min_mask_energy_db: in_range(stage, 13 + offset, -100.0, 120.0)?,
        mid_velocity: in_range(stage, 14 + offset, 0.001, 2.0)?,
        mid_mask_energy_db: in_range(stage, 15 + offset, -100.0, 120.0)?,
        high_velocity: in_range(stage, 16 + offset, 0.001, 2.0)?,
        high_mask_energy_db: in_range(stage, 17 + offset, -100.0, 120.0)?,
        max_mask_energy_db: in_range(stage, 18 + offset, -100.0, 120.0)?,
        mask_attack_s: in_range(stage, 19 + offset, 0.0, 5.0)?,
        mask_release_s: in_range(stage, 20 + offset, 0.0, 5.0)?,
        max_attenuation_db: in_range(stage, 21 + offset, 0.0, 24.0)?,
        release_s: in_range(stage, 22 + offset, 0.0, 5.0)?,
        min_frequency_hz: in_range(stage, 23 + offset, 0.0, 19_950.0)?,
        max_frequency_hz: in_range(stage, 24 + offset, 50.0, 96_000.0)?,
        mid_attenuation_db: in_range(stage, 25 + offset, 0.0, 24.0)?,
        mid_level_db: in_range(stage, 26 + offset, -100.0, -0.01)?,
        enabled: boolean(stage, 8 + offset)?,
        mute: boolean(stage, 9 + offset)?,
        pressure_response,
        tweeter_response,
    })
}

fn conservative_model(stage: &Stage, index: usize) -> Result<Model> {
    let offset = if index == 0 { 0 } else { 24 };
    let mut pressure_response = [0.0; RESPONSE_TAPS];
    pressure_response[0] = 1.0;
    Ok(Model {
        // The guarded fallback never evaluates the acoustic or velocity
        // detector, but serialized models retain finite, valid placeholders.
        bl: 1.0,
        kms: 1.0,
        kms_front_port: 1.0,
        mms: 1.0,
        mms_front_port: 1.0,
        reb: 1.0,
        rms: 1.0,
        rms_front_port: 1.0,
        analog_gain: 1.0,
        tweeter_gain: 1.0,
        mask_band_min_hz: in_range(stage, 10 + offset, 0.0, 24_000.0)?,
        mask_band_max_hz: in_range(stage, 11 + offset, 0.0, 24_000.0)?,
        min_velocity: in_range(stage, 12 + offset, 0.001, 2.0)?,
        min_mask_energy_db: in_range(stage, 13 + offset, -100.0, 120.0)?,
        mid_velocity: in_range(stage, 14 + offset, 0.001, 2.0)?,
        mid_mask_energy_db: in_range(stage, 15 + offset, -100.0, 120.0)?,
        high_velocity: in_range(stage, 16 + offset, 0.001, 2.0)?,
        high_mask_energy_db: in_range(stage, 17 + offset, -100.0, 120.0)?,
        max_mask_energy_db: in_range(stage, 18 + offset, -100.0, 120.0)?,
        mask_attack_s: in_range(stage, 19 + offset, 0.0, 5.0)?,
        mask_release_s: in_range(stage, 20 + offset, 0.0, 5.0)?,
        max_attenuation_db: in_range(stage, 21 + offset, 0.0, 24.0)?,
        release_s: in_range(stage, 22 + offset, 0.0, 5.0)?,
        min_frequency_hz: in_range(stage, 23 + offset, 0.0, 19_950.0)?,
        max_frequency_hz: in_range(stage, 24 + offset, 50.0, 96_000.0)?,
        mid_attenuation_db: in_range(stage, 25 + offset, 0.0, 24.0)?,
        mid_level_db: in_range(stage, 26 + offset, -100.0, -0.01)?,
        enabled: boolean(stage, 8 + offset)?,
        mute: boolean(stage, 9 + offset)?,
        pressure_response,
        tweeter_response: pressure_response,
    })
}

fn conservative_params(stage: &Stage, diags: &mut Diagnostics) -> Result<Option<Params>> {
    let channels = stage.channels.unwrap_or(0) as usize / 2;
    if channels == 0 || channels > MAX_MODELS || stage.channels != Some((2 * channels) as u32) {
        return Err(Error::range(
            &stage.name,
            "channels",
            format!("is {:?}, expected an even width for 1..={MAX_MODELS} acoustic channels", stage.channels),
        ));
    }
    let mut models = [Model::default(); MAX_MODELS];
    for (index, model) in models[..channels].iter_mut().enumerate() {
        *model = conservative_model(stage, index)?;
    }
    let oversampling = in_range(stage, 5, 0.0, 5.0)?.max(3.0) as usize;
    diags.warn(
        "lowflow-conservative-fallback",
        &stage.name,
        "property 6620 is incomplete; selecting the guarded never-louder fallback",
    );
    Ok(Some(Params {
        model_count: channels,
        link_channel_energies: boolean(stage, 1)?,
        latency_frames: (oversampling - 1) * 128,
        conservative_fallback: true,
        models,
    }))
}

/// Compiles the saved controls and sidecar. Graph parameter 4 remains a
/// run-time input and is supplied through `Run::set_volume`.
pub fn compile(stage: &Stage, _fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    if stage.bypass || boolean(stage, 0)? {
        return Ok(None);
    }
    let files: Vec<_> = stage.property_files.iter().filter(|p| p.number == PROPERTY_SIDECAR).collect();
    let property = match files.as_slice() {
        [one] => *one,
        [] => return conservative_params(stage, diags),
        _ => return Err(bad(&stage.name, format!("property {PROPERTY_SIDECAR} appears more than once"))),
    };
    let root = property.value.as_dict().ok_or_else(|| bad(&property.path, "sidecar root is not a dictionary"))?;
    let channels = match root.get("ChannelData").and_then(Value::as_array) {
        Some(channels) => channels,
        None => return conservative_params(stage, diags),
    };
    if channels.is_empty() || channels.len() > MAX_MODELS {
        return Err(Error::range(
            &property.path,
            "ChannelData",
            format!("has {} entries, expected 1..={MAX_MODELS}", channels.len()),
        ));
    }
    if stage.channels != Some((2 * channels.len()) as u32) {
        return Err(bad(
            &stage.name,
            format!("sidecar has {} acoustic channels but the box width is {:?}", channels.len(), stage.channels),
        ));
    }
    let mut models = [Model::default(); MAX_MODELS];
    for (i, value) in channels.iter().enumerate() {
        let d = value.as_dict().ok_or_else(|| bad(&property.path, format!("ChannelData[{i}] is not a dictionary")))?;
        models[i] = match model(d, stage, i) {
            Ok(model) => model,
            Err(Error::Missing { .. }) => return conservative_params(stage, diags),
            Err(error) => return Err(error),
        };
    }
    diags.info("lowflow-sidecar", &stage.name, "property 6620 loaded with velocity models and acoustic responses");
    let oversampling = in_range(stage, 5, 0.0, 5.0)?.max(3.0) as usize;
    Ok(Some(Params {
        model_count: channels.len(),
        link_channel_energies: boolean(stage, 1)?,
        latency_frames: (oversampling - 1) * 128,
        conservative_fallback: false,
        models,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState};
    use crate::tuning::PropertyFile;

    fn stage_without_sidecar() -> Stage {
        let mut params = vec![(0, 0.0), (1, 1.0), (4, 0.0), (5, 3.0)];
        for offset in [0, 24] {
            params.extend([
                (8 + offset, 1.0),
                (9 + offset, 0.0),
                (10 + offset, 4_000.0),
                (11 + offset, 8_000.0),
                (12 + offset, 0.04),
                (13 + offset, -10.0),
                (14 + offset, 0.08),
                (15 + offset, 10.0),
                (16 + offset, 0.16),
                (17 + offset, 30.0),
                (18 + offset, 40.0),
                (19 + offset, 0.0),
                (20 + offset, 0.05),
                (21 + offset, 12.0),
                (22 + offset, 0.05),
                (23 + offset, 300.0),
                (24 + offset, 2_500.0),
                (25 + offset, 0.0),
                (26 + offset, -30.0),
            ]);
        }
        Stage {
            name: "LowFlow".into(),
            kind: "aufx/flow/appl".into(),
            unit: None,
            inputs: 1,
            outputs: 1,
            channels: Some(4),
            bypass: false,
            preset_name: None,
            state: Some(ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] }),
            preset_extra_keys: Vec::new(),
            preset_extra: Default::default(),
            strip_properties: Vec::new(),
            property_files: Vec::new(),
            before_protection: true,
        }
    }

    #[test]
    fn missing_sidecar_selects_conservative_fallback() {
        let mut diagnostics = Diagnostics::default();
        let params = compile(&stage_without_sidecar(), 48_000.0, &mut diagnostics).unwrap().unwrap();
        assert!(params.conservative_fallback);
        assert_eq!(params.model_count, 2);
        assert!(diagnostics.items.iter().any(|item| item.code == "lowflow-conservative-fallback"));
    }

    #[test]
    fn missing_sidecar_field_selects_conservative_fallback() {
        let mut stage = stage_without_sidecar();
        stage.property_files.push(PropertyFile {
            number: PROPERTY_SIDECAR,
            path: "synthetic.plist".into(),
            keys: Vec::new(),
            value: Value::Dict(Dict::new()),
        });
        let mut diagnostics = Diagnostics::default();
        let params = compile(&stage, 48_000.0, &mut diagnostics).unwrap().unwrap();
        assert!(params.conservative_fallback);
        assert!(diagnostics.items.iter().any(|item| item.code == "lowflow-conservative-fallback"));
    }
}
