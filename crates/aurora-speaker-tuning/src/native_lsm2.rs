// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//! Missing-only amplifier selectors from a manifest-bound native snapshot.
//!
//! These checks establish receipt self-consistency, not capture authenticity.
//! See `docs/LSM2-NATIVE-STATE.md`. No other native defaults are imported.

use crate::aupreset::{self, AuPreset, ParamState};
use crate::bundle::Bundle;
use crate::error::{Error, Result};
use crate::manifest::{self, Manifest};
use crate::plist::{self, Dict, Value};
use std::collections::{BTreeMap, BTreeSet};

pub const NAME: &str = "lsm2-native-state.plist";
pub const MAX_BYTES: usize = 16 * 1024 * 1024;
const AMP0: u32 = 1_058_848;
const STRIDE: u32 = 1024;
const STAGES: [&str; 3] = ["before-init", "after-init", "after-render"];

fn bad(message: impl Into<String>) -> Error {
    Error::format(NAME, message)
}
fn require(condition: bool, message: &str) -> Result<()> {
    if condition {
        Ok(())
    } else {
        Err(bad(message))
    }
}
fn keys(d: &Dict, allowed: &[&str]) -> Result<()> {
    require(
        d.len() == allowed.len() && d.keys().all(|key| allowed.contains(&key)),
        "unexpected or missing snapshot fields",
    )
}
fn field<'a>(d: &'a Dict, key: &str) -> Result<&'a Value> {
    d.get(key).ok_or_else(|| bad(format!("missing {key}")))
}
fn dictionary<'a>(d: &'a Dict, key: &str) -> Result<&'a Dict> {
    field(d, key)?.as_dict().ok_or_else(|| bad(format!("{key} is not a dictionary")))
}
fn string<'a>(d: &'a Dict, key: &str) -> Result<&'a str> {
    match field(d, key)? {
        Value::String(s) => Ok(s),
        _ => Err(bad(format!("{key} is not a string"))),
    }
}
fn integer(d: &Dict, key: &str) -> Result<i128> {
    match field(d, key)? {
        Value::Integer(i) => Ok(*i),
        _ => Err(bad(format!("{key} is not an integer"))),
    }
}
fn array<'a>(d: &'a Dict, key: &str) -> Result<&'a [Value]> {
    match field(d, key)? {
        Value::Array(a) => Ok(a),
        _ => Err(bad(format!("{key} is not an array"))),
    }
}
fn number(d: &Dict, key: &str) -> Result<f64> {
    let n = match field(d, key)? {
        Value::Real(n) => *n,
        Value::Integer(n) => *n as f64,
        _ => return Err(bad(format!("{key} is not a number"))),
    };
    if n.is_finite() {
        Ok(n)
    } else {
        Err(bad(format!("{key} is not finite")))
    }
}

/// Hash every source record, excluding the native snapshot itself. The fixed
/// domain and sorted compact JSON tuples make the representation unambiguous.
/// Identity/build fields are checked separately against the same manifest.
pub fn source_fingerprint(m: &Manifest) -> Result<String> {
    let mut seen = BTreeSet::new();
    let mut rows = Vec::new();
    for r in m.files.iter().chain(m.shared.iter()).chain(m.speaker_thiele_small.iter()).chain(m.mask_lattice.iter()) {
        require(r.path.is_ascii() && manifest::admitted(&r.path), "source path is not canonical ASCII")?;
        require(r.path != NAME && seen.insert(r.path.as_str()), "duplicate or ambiguous source path")?;
        require(crate::machine::valid_unit_id(&r.sha256), "source hash is not canonical SHA-256")?;
        rows.push((r.path.as_str(), r.bytes, r.sha256.as_str()));
    }
    rows.sort_unstable_by_key(|r| r.0);
    let mut bytes = b"aurora-lsm2-native-source-v1\0".to_vec();
    bytes.extend(serde_json::to_vec(&rows).map_err(|e| bad(format!("source fingerprint: {e}")))?);
    Ok(manifest::sha256_hex(&bytes))
}

fn identity(d: &Dict, m: &Manifest) -> Result<()> {
    keys(d, &["macos_build", "board", "unit_id"])?;
    let build = m.macos_build.as_deref().ok_or_else(|| bad("source build is unknown"))?;
    let board = m.board.as_deref().ok_or_else(|| bad("source board is unknown"))?;
    let unit = m.unit_id.as_deref().ok_or_else(|| bad("source unit is unknown"))?;
    require(!build.trim().is_empty() && !board.trim().is_empty(), "source build or board is empty")?;
    require(string(d, "macos_build")? == build, "native build differs from source bundle")?;
    require(string(d, "board")? == board, "native board differs from source bundle")?;
    require(
        crate::machine::valid_unit_id(unit) && string(d, "unit_id")? == unit,
        "native unit differs from source bundle",
    )
}

/// Validate a declared snapshot and copy only missing active-channel amp IDs.
/// The caller must retain the returned count to enforce runtime unit binding.
pub fn supplement(b: &Bundle, original: &AuPreset, channels: usize) -> Result<(ParamState, usize, u32)> {
    require(b.verified, "native selectors require a verified bundle")?;
    require((1..=sgv2_core::MAX_CHANNELS).contains(&channels), "invalid active channel count")?;
    let record = b.manifest.lsm2_native_state.as_ref().ok_or_else(|| bad("no declared native state"))?;
    let bytes = b.file(&record.path).ok_or_else(|| bad("declared native state is missing"))?;
    require(bytes.len() <= MAX_BYTES, "native state is too large")?;
    record.verify(bytes)?;
    let (value, _) = plist::parse(bytes).map_err(|e| Error::plist(NAME, e))?;
    let d = value.as_dict().ok_or_else(|| bad("native state root is not a dictionary"))?;
    keys(
        d,
        &[
            "schema",
            "method",
            "identity_before",
            "identity_after",
            "source_fingerprint",
            "runtime_overrides",
            "canonical_state",
            "canonical_sha256",
            "parameter_queries",
            "channels",
            "sample_rate",
        ],
    )?;
    require(integer(d, "schema")? == 1, "unsupported native state schema")?;
    require(integer(d, "channels")? == channels as i128, "native channel count differs from graph")?;
    let sample_rate =
        u32::try_from(integer(d, "sample_rate")?).map_err(|_| bad("native sample rate is out of range"))?;
    require(sample_rate != 0, "native sample rate is zero")?;
    require(string(d, "method")? == "AudioUnitGetProperty-ClassInfo", "unsupported native capture method")?;
    require(array(d, "runtime_overrides")?.is_empty(), "native capture used runtime overrides")?;
    let before = dictionary(d, "identity_before")?;
    let after = dictionary(d, "identity_after")?;
    identity(before, &b.manifest)?;
    identity(after, &b.manifest)?;
    require(
        string(d, "source_fingerprint")? == source_fingerprint(&b.manifest)?,
        "native source fingerprint mismatch",
    )?;
    let native_bytes = match field(d, "canonical_state")? {
        Value::Data(bytes) => bytes,
        _ => return Err(bad("canonical_state is not plist bytes")),
    };
    require(string(d, "canonical_sha256")? == manifest::sha256_hex(native_bytes), "canonical state digest mismatch")?;
    let canonical = aupreset::parse_file(native_bytes, NAME)?;
    require(
        original.version.is_some() && original.unit == canonical.unit && original.version == canonical.version,
        "native component or preset version differs",
    )?;
    require(
        canonical.unit.kind.bytes() == *b"aufx"
            && canonical.unit.subtype.bytes() == *b"lsm2"
            && canonical.unit.manufacturer.bytes() == *b"appl",
        "canonical state is not Apple LSM2",
    )?;
    require(original.has_state && canonical.has_state, "parameter state is missing")?;
    for section in &original.state.sections {
        let native = canonical
            .state
            .section(section.scope, section.element)
            .ok_or_else(|| bad("canonical state removed an original section"))?;
        for &(id, value) in &section.params {
            require(
                native.get(id).is_some_and(|n| n.to_bits() == value.to_bits()),
                "canonical state changed or removed an original parameter",
            )?;
        }
    }
    let rows = array(d, "parameter_queries")?;
    require(rows.len() == 3 * channels, "native parameter query count differs from active channels")?;
    let mut queries = BTreeMap::new();
    for row in rows {
        let row = row.as_dict().ok_or_else(|| bad("parameter query is not a dictionary"))?;
        keys(row, &["stage", "id", "parameter_info_status", "get_parameter_status", "value"])?;
        let stage = string(row, "stage")?;
        require(STAGES.contains(&stage), "unknown native query stage")?;
        let id = u32::try_from(integer(row, "id")?).map_err(|_| bad("native query ID is out of range"))?;
        require(
            id >= AMP0 && (id - AMP0) % STRIDE == 0 && ((id - AMP0) / STRIDE) < channels as u32,
            "query is not an active amplifier selector",
        )?;
        require(
            integer(row, "parameter_info_status")? == 0 && integer(row, "get_parameter_status")? == 0,
            "native parameter query failed",
        )?;
        number(row, "value")?;
        require(queries.insert((stage, id), row).is_none(), "duplicate native parameter query")?;
    }
    let native = canonical.state.global().ok_or_else(|| bad("canonical global state is missing"))?;
    let mut result = original.state.clone();
    let global = result
        .sections
        .iter_mut()
        .find(|s| s.scope == 0 && s.element == 0)
        .ok_or_else(|| bad("original global state is missing"))?;
    let mut added = 0;
    for channel in 0..channels {
        let id = AMP0 + channel as u32 * STRIDE;
        let value = native.get(id).ok_or_else(|| bad("required native amplifier selector is missing"))?;
        require(matches!(value, 0.0 | 1.0 | 2.0), "native amplifier selector is not 0, 1, or 2")?;
        for stage in STAGES {
            let row = queries.get(&(stage, id)).ok_or_else(|| bad("required native parameter query is missing"))?;
            require(number(row, "value")? == f64::from(value), "native query differs from canonical selector")?;
        }
        if global.get(id).is_none() {
            global.params.push((id, value));
            added += 1;
        }
    }
    Ok((result, added, sample_rate))
}
