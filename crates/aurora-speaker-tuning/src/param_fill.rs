// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Owner-local lsm2 parameter readback. Values are private inputs, never defaults.
//! Published parameters are retained for provenance; only `fill_ids` may supply
//! missing saved records. ID sets, not record positions, define completeness:
//! a preset is short when it lacks any base ID, whatever its record count.
//! A short preset may also carry saved IDs from the append map
//! ([`sgv2_core::lsm2::companion::extension_ids`]) that the unit publishes;
//! any other saved ID is unknown and refused.
use crate::{
    aupreset::ParamSection,
    error::{Error, Result},
    manifest::sha256_hex,
};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const NAME: &str = "lsm2-param-fill.bin";
/// 2: captures name the component (SHA-256 of its executable) and may report
/// unit version 0 (the unit reports none); see the installer's lsm2_fill.py.
pub const VERSION: u32 = 2;
pub const FORMAT: &str = "json-id-name-f32bits-v1";
pub const MAX_BYTES: usize = 4 << 20;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Field {
    version: u32,
    format: String,
    aid: String,
    macos_build: String,
    unit_id: String,
    configuration_sha256: String,
    presets: Vec<Capture>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Capture {
    strip: String,
    preset_sha256: String,
    /// 0 when the unit reports no version; `component_sha256` and the
    /// macOS build then identify the unit code.
    unit_version: u32,
    component_sha256: String,
    published: Vec<Parameter>,
    fill_ids: Vec<u32>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Parameter {
    id: u32,
    name: String,
    bits: u32,
}

fn bad(message: &str) -> Error {
    Error::format(NAME, message)
}

fn is_sha256(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

/// Base IDs the saved preset does not carry; non-empty means short.
pub fn missing_base(saved: &ParamSection) -> BTreeSet<u32> {
    let carried: BTreeSet<_> = saved.params.iter().map(|p| p.0).collect();
    sgv2_core::lsm2::companion::base_ids().into_iter().filter(|id| !carried.contains(id)).collect()
}

/// Saved IDs outside the base schema.
fn extra_saved(saved: &ParamSection) -> BTreeSet<u32> {
    let base: BTreeSet<_> = sgv2_core::lsm2::companion::base_ids().into_iter().collect();
    saved.params.iter().map(|p| p.0).filter(|id| !base.contains(id)).collect()
}

/// Hash the exact AID/Common source files that may configure the unit.
/// The same framing is used by the installer (no tuning values in source).
pub fn configuration_digest(files: &BTreeMap<String, Vec<u8>>, aid: &str) -> String {
    let mut hash = Sha256::new();
    hash.update(b"aurora-lsm2-configuration-v1\0");
    for (path, data) in files {
        if path.starts_with(&format!("{aid}/")) || path.starts_with("Common/") {
            hash.update(path.as_bytes());
            hash.update([0]);
            hash.update((data.len() as u64).to_le_bytes());
            hash.update(data);
        }
    }
    format!("{:x}", hash.finalize())
}

/// Validate provenance and select the capture for this exact strip/preset.
/// Error messages deliberately never include names or values from the field.
pub fn read(
    bytes: &[u8],
    bundle: &crate::bundle::Bundle,
    strip: &str,
    saved: &ParamSection,
) -> Result<Option<Vec<(u32, f32)>>> {
    let aid = bundle.manifest.aid.as_str();
    let build = bundle.manifest.macos_build.as_deref();
    let unit = bundle.manifest.unit_id.as_deref();
    let unit_versions = &bundle.manifest.param_fill_unit_versions;
    if bytes.is_empty() || bytes.len() > MAX_BYTES {
        return Err(bad("fill field over budget or empty"));
    }
    let field: Field = serde_json::from_slice(bytes).map_err(|_| bad("malformed fill field"))?;
    if field.version != VERSION
        || field.format != FORMAT
        || field.aid != aid
        || Some(field.macos_build.as_str()) != build
        || Some(field.unit_id.as_str()) != unit
        || !crate::machine::valid_unit_id(&field.unit_id)
    {
        return Err(bad("fill identity differs from its bundle"));
    }
    if field.configuration_sha256 != configuration_digest(&bundle.files, aid) {
        return Err(bad("fill source configuration changed"));
    }
    if field.presets.is_empty() || field.presets.len() > 64 {
        return Err(bad("invalid capture count"));
    }
    let mut versions = BTreeSet::new();
    let mut identities = BTreeSet::new();
    let mut captures = BTreeSet::new();
    let short = !missing_base(saved).is_empty();
    let state = crate::aupreset::ParamState { sections: vec![saved.clone()] }.encode();
    let digest = sha256_hex(&state);
    let mut selected = None;
    for capture in field.presets {
        if !is_sha256(&capture.component_sha256)
            || !captures.insert((capture.strip.clone(), capture.preset_sha256.clone()))
        {
            return Err(bad("duplicate or invalid capture"));
        }
        versions.insert(capture.unit_version);
        identities.insert((capture.unit_version, capture.component_sha256.clone()));
        if capture.published.is_empty() || capture.published.len() > 16384 {
            return Err(bad("invalid published parameter count"));
        }
        // Names are provenance only (may be empty); only fill values are
        // used, so only they must be finite.
        let mut published = BTreeMap::new();
        for p in capture.published {
            if p.name.len() > 1024 || published.insert(p.id, f32::from_bits(p.bits)).is_some() {
                return Err(bad("duplicate or invalid published parameter"));
            }
        }
        let mut fill = BTreeMap::new();
        for id in capture.fill_ids {
            let value = *published.get(&id).ok_or_else(|| bad("fill ID not published by the unit"))?;
            if !value.is_finite() {
                return Err(bad("fill value is not finite"));
            }
            if fill.insert(id, value).is_some() {
                return Err(bad("duplicate fill ID"));
            }
        }
        if capture.strip == strip && capture.preset_sha256 == digest {
            // Validate even an empty full-preset dry-run capture, but do not
            // alter the saved record order or any compiled byte for it.
            if short {
                if extra_saved(saved).iter().any(|id| !published.contains_key(id)) {
                    return Err(bad("saved ID outside the base schema is not published by the unit"));
                }
                merge(saved, &fill.iter().map(|(&id, &v)| (id, v)).collect::<Vec<_>>())?;
            } else if !fill.is_empty() {
                return Err(bad("full presets cannot have fill records"));
            }
            selected = Some(fill.into_iter().collect());
        }
    }
    if identities.len() != 1 {
        return Err(bad("captures name different units"));
    }
    if !versions.iter().copied().eq(unit_versions.iter().copied()) {
        return Err(bad("unit versions differ from the manifest"));
    }
    if short && selected.is_none() {
        return Err(bad("no capture for the selected strip and preset"));
    }
    Ok(selected)
}

/// The fill is exactly base_ids minus the preset's IDs. Never override a
/// carried record, accept a renumbered short layout, or fill an extra ID.
/// Saved append-map IDs ([`read`] has checked the unit publishes them) are
/// left out of the completed 1,021-ID section: the base model does not
/// interpret them, and a partial append map is not the extended schema.
/// Any other saved ID outside the base schema is refused.
pub fn merge(saved: &ParamSection, fill: &[(u32, f32)]) -> Result<ParamSection> {
    use sgv2_core::lsm2::companion::{base_ids, extension_ids};
    let expected: BTreeSet<_> = base_ids().into_iter().collect();
    let appended: BTreeSet<_> = extension_ids().into_iter().collect();
    let carried: BTreeSet<_> = saved.params.iter().map(|p| p.0).collect();
    if carried.len() != saved.params.len()
        || !carried.iter().all(|id| expected.contains(id) || appended.contains(id))
        || saved.params.iter().any(|p| !p.1.is_finite())
    {
        return Err(bad("preset carries IDs outside the known lsm2 schema"));
    }
    let missing: BTreeSet<_> = expected.difference(&carried).copied().collect();
    let supplied: BTreeSet<_> = fill.iter().map(|p| p.0).collect();
    if supplied != missing || supplied.len() != fill.len() || fill.iter().any(|p| !p.1.is_finite()) {
        return Err(bad("fill must contain exactly the missing saved IDs"));
    }
    let mut merged = saved.clone();
    merged.params.retain(|p| expected.contains(&p.0));
    merged.params.extend_from_slice(fill);
    sgv2_core::lsm2::companion::validate_records(&merged.params, false)
        .map_err(|_| bad("filled preset does not match the 1,021-ID layout"))?;
    Ok(merged)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> (ParamSection, Vec<(u32, f32)>) {
        let records: Vec<_> = sgv2_core::lsm2::companion::base_ids()
            .into_iter()
            .enumerate()
            .map(|(i, id)| (id, (i + 1) as f32 / 17.0))
            .collect();
        let saved = ParamSection { scope: 0, element: 0, params: records[58..].to_vec() };
        (saved, records[..58].to_vec())
    }
    #[test]
    fn exact_missing_ids_independent_of_order() {
        let (saved, mut fill) = fixture();
        fill.reverse();
        let merged = merge(&saved, &fill).unwrap();
        assert_eq!(&merged.params[..saved.params.len()], &saved.params);
        assert_eq!(merged.params.len(), 1021);
    }
    #[test]
    fn missing_extra_mismatched_duplicate_nonfinite_and_override_refused() {
        let (saved, fill) = fixture();
        let mut bad_cases = vec![fill[..57].to_vec()];
        let mut extra = fill.clone();
        extra.push((u32::MAX, 1.0));
        bad_cases.push(extra);
        let mut mismatch = fill.clone();
        mismatch[0].0 = u32::MAX;
        bad_cases.push(mismatch);
        let mut duplicate = fill.clone();
        duplicate[0].0 = duplicate[1].0;
        bad_cases.push(duplicate);
        let mut nonfinite = fill.clone();
        nonfinite[0].1 = f32::NAN;
        bad_cases.push(nonfinite);
        let mut override_saved = fill.clone();
        override_saved.push(saved.params[0]);
        bad_cases.push(override_saved);
        for bad_fill in bad_cases {
            assert!(merge(&saved, &bad_fill).is_err());
        }
        let mut renumbered = saved.clone();
        renumbered.params[0].0 = u32::MAX;
        assert!(merge(&renumbered, &fill).is_err());
    }
    #[test]
    fn append_id_is_kept_out_of_the_base_section_and_unknown_ids_refused() {
        let (saved, fill) = fixture();
        let append = sgv2_core::lsm2::companion::extension::PILOT_TUNING_RESERVED;
        let mut with_append = saved.clone();
        with_append.params.insert(3, (append, 0.5));
        assert_eq!(missing_base(&with_append).len(), 58);
        let merged = merge(&with_append, &fill).unwrap();
        assert_eq!(merged, merge(&saved, &fill).unwrap());
        let mut unknown = saved.clone();
        unknown.params.push((append + 1, 0.5));
        assert!(merge(&unknown, &fill).is_err());
        // A full-count preset that lacks a base ID is still short.
        let mut swapped = merged.clone();
        let dropped = swapped.params.remove(0).0;
        swapped.params.push((append, 0.5));
        assert_eq!(swapped.params.len(), 1021);
        assert_eq!(missing_base(&swapped).into_iter().collect::<Vec<_>>(), vec![dropped]);
    }
}
