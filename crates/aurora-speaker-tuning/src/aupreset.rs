//! AudioUnit presets (`ClassInfo` dictionaries) and their parameter blobs.
//!
//! A preset is a dictionary with `type`, `subtype`, `manufacturer` (FourCCs
//! stored as integers), `version`, an optional `name`, and `data`.  The `data`
//! blob is the AudioUnit SDK's saved parameter state: a sequence of sections,
//! each
//!
//! ```text
//! u32 scope, u32 element, u32 count, count x (u32 parameter ID, f32 value)
//! ```
//!
//! all big-endian.  This is the public AudioUnit SDK layout (`AUBase` /
//! `AUScope` / `AUElement` state saving); the scopes are the public
//! `kAudioUnitScope_*` constants (0 global, 1 input, 2 output, 3 group... see
//! [`scope_name`]).  Every shipped speaker-strip blob decodes to exactly
//! its length with this layout, including the per-element sections that
//! earlier notes called "nested content" (spp3 keeps one part-scope element
//! per channel).
//!
//! Other keys a preset may carry (`DefaultAudioChannelLayout`, `classname`,
//! `auversion`, unit-specific dictionaries such as XTC's custom filter
//! dictionary) are kept verbatim in [`AuPreset::extra`].

use crate::error::{Error, Result};
use crate::fourcc::FourCc;
use crate::plist::{Dict, Value};
use serde::Serialize;

/// Public `AudioUnitScope` names.
pub fn scope_name(scope: u32) -> &'static str {
    match scope {
        0 => "global",
        1 => "input",
        2 => "output",
        3 => "group",
        4 => "part",
        5 => "note",
        6 => "layer",
        7 => "layer-item",
        _ => "unknown",
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ParamSection {
    pub scope: u32,
    pub element: u32,
    /// (parameter ID, value) in stored order.  IDs are unique per section.
    pub params: Vec<(u32, f32)>,
}

impl ParamSection {
    pub fn get(&self, id: u32) -> Option<f32> {
        self.params.iter().find(|(i, _)| *i == id).map(|(_, v)| *v)
    }
}

/// A decoded parameter blob.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct ParamState {
    pub sections: Vec<ParamSection>,
}

impl ParamState {
    pub fn section(&self, scope: u32, element: u32) -> Option<&ParamSection> {
        self.sections.iter().find(|s| s.scope == scope && s.element == element)
    }

    pub fn global(&self) -> Option<&ParamSection> {
        self.section(0, 0)
    }

    /// Elements of one scope, in element order.
    pub fn elements(&self, scope: u32) -> Vec<&ParamSection> {
        let mut v: Vec<_> = self.sections.iter().filter(|s| s.scope == scope).collect();
        v.sort_by_key(|s| s.element);
        v
    }

    /// Decodes a blob strictly: the sections must cover it exactly, no
    /// (scope, element) may repeat, no ID may repeat within a section, and
    /// every value must be finite.
    pub fn decode(blob: &[u8], context: &str) -> Result<Self> {
        let bad = |at: usize, why: String| Error::format(context, format!("parameter blob at byte {at}: {why}"));
        let u32_at = |at: usize| u32::from_be_bytes(blob[at..at + 4].try_into().unwrap());
        let mut sections: Vec<ParamSection> = Vec::new();
        let mut at = 0usize;
        while at < blob.len() {
            if blob.len() - at < 12 {
                return Err(bad(at, format!("{} trailing bytes, too short for a section header", blob.len() - at)));
            }
            let (scope, element, count) = (u32_at(at), u32_at(at + 4), u32_at(at + 8) as usize);
            let body = at + 12;
            let len = count.checked_mul(8).ok_or_else(|| bad(at, "count overflows".into()))?;
            if blob.len() - body < len {
                return Err(bad(
                    at,
                    format!("section ({scope}, {element}) declares {count} parameters, blob too short"),
                ));
            }
            if sections.iter().any(|s| s.scope == scope && s.element == element) {
                return Err(bad(at, format!("section ({scope}, {element}) repeats")));
            }
            let mut params = Vec::with_capacity(count);
            for k in 0..count {
                let p = body + 8 * k;
                let id = u32_at(p);
                let v = f32::from_bits(u32_at(p + 4));
                if !v.is_finite() {
                    return Err(bad(p, format!("parameter {id} in ({scope}, {element}) is not finite")));
                }
                if params.iter().any(|(i, _)| *i == id) {
                    return Err(bad(p, format!("parameter {id} repeats in ({scope}, {element})")));
                }
                params.push((id, v));
            }
            sections.push(ParamSection { scope, element, params });
            at = body + len;
        }
        Ok(ParamState { sections })
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::new();
        for s in &self.sections {
            out.extend_from_slice(&s.scope.to_be_bytes());
            out.extend_from_slice(&s.element.to_be_bytes());
            out.extend_from_slice(&(s.params.len() as u32).to_be_bytes());
            for (id, v) in &s.params {
                out.extend_from_slice(&id.to_be_bytes());
                out.extend_from_slice(&v.to_bits().to_be_bytes());
            }
        }
        out
    }
}

/// The identity of an AudioUnit: (type, subtype, manufacturer).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize)]
pub struct UnitId {
    #[serde(rename = "type")]
    pub kind: FourCc,
    pub subtype: FourCc,
    pub manufacturer: FourCc,
}

impl std::fmt::Display for UnitId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}/{}", self.kind, self.subtype, self.manufacturer)
    }
}

pub(crate) fn fourcc_field(d: &Dict, key: &str, context: &str) -> Result<FourCc> {
    let v = d.get(key).ok_or_else(|| Error::missing(context, key))?;
    match v {
        Value::Integer(i) if (0..=u32::MAX as i128).contains(i) => Ok(FourCc(*i as u32)),
        Value::String(s) => {
            FourCc::from_str_exact(s).ok_or_else(|| Error::format(context, format!("{key} {s:?} is not a FourCC")))
        }
        other => Err(Error::format(context, format!("{key} is a {}, not a FourCC", other.kind()))),
    }
}

pub(crate) fn unit_id(d: &Dict, context: &str) -> Result<UnitId> {
    Ok(UnitId {
        kind: fourcc_field(d, "type", context)?,
        subtype: fourcc_field(d, "subtype", context)?,
        manufacturer: fourcc_field(d, "manufacturer", context)?,
    })
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct AuPreset {
    pub unit: UnitId,
    pub name: Option<String>,
    pub version: Option<i128>,
    /// False when the preset has no `data` (graph builtins such as channel
    /// splitters carry only a `classname`).
    pub has_state: bool,
    pub state: ParamState,
    /// Keys other than type/subtype/manufacturer/name/version/data, verbatim.
    #[serde(skip)]
    pub extra: Dict,
}

impl AuPreset {
    pub fn from_dict(d: &Dict, context: &str) -> Result<Self> {
        let unit = unit_id(d, context)?;
        let name = match d.get("name") {
            None => None,
            Some(Value::String(s)) => Some(s.clone()),
            Some(o) => return Err(Error::format(context, format!("name is a {}", o.kind()))),
        };
        let version = match d.get("version") {
            None => None,
            Some(Value::Integer(i)) => Some(*i),
            Some(o) => return Err(Error::format(context, format!("version is a {}", o.kind()))),
        };
        let (has_state, state) = match d.get("data") {
            Some(Value::Data(b)) => (true, ParamState::decode(b, context)?),
            Some(o) => return Err(Error::format(context, format!("data is a {}", o.kind()))),
            None => (false, ParamState::default()),
        };
        let mut extra = Dict::new();
        for (k, v) in d.iter() {
            if !matches!(k, "type" | "subtype" | "manufacturer" | "name" | "version" | "data") {
                extra.insert_unique(k.to_string(), v.clone());
            }
        }
        Ok(AuPreset { unit, name, version, has_state, state, extra })
    }

    pub fn extra_keys(&self) -> Vec<String> {
        self.extra.keys().map(str::to_string).collect()
    }
}

/// Parses a standalone `.aupreset` file.
pub fn parse_file(bytes: &[u8], context: &str) -> Result<AuPreset> {
    let (v, _) = crate::plist::parse(bytes).map_err(|e| Error::plist(context, e))?;
    let d = v.as_dict().ok_or_else(|| Error::format(context, "preset root is not a dictionary"))?;
    AuPreset::from_dict(d, context)
}
