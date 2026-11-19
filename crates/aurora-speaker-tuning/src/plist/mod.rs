//! Property lists: one value model, two decoders.
//!
//! * [`binary`]: the complete `bplist00` format (every object type, every
//!   integer width, extended counts, shared references, cycle and bounds
//!   checks).
//! * [`xml`]: the XML 1.0 `plist` DTD, with its own small tokenizer (no
//!   external XML dependency), entity and character references, CDATA,
//!   comments and processing instructions.
//!
//! [`parse`] picks the decoder from the leading bytes.  Every failure is a
//! [`PlistError`] that names the byte offset (binary) or line (XML).

pub mod binary;
pub mod xml;

use std::fmt;

/// One decoded property-list value.
///
/// Dictionaries keep their on-disk key order (binary plists are not sorted),
/// and duplicate keys are rejected by both decoders.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Bool(bool),
    /// Every plist integer fits: binary plists store up to 128-bit integers
    /// (unsigned 64-bit values are written as 16-byte integers).
    Integer(i128),
    Real(f64),
    String(String),
    Data(Vec<u8>),
    /// Seconds since 2001-01-01T00:00:00Z (the Core Foundation epoch).
    Date(f64),
    Array(Vec<Value>),
    Dict(Dict),
    /// Keyed-archiver object reference (binary only).
    Uid(u64),
    /// `bplist00` null object (marker 0x00); not representable in XML.
    Null,
}

/// An ordered dictionary with unique keys.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Dict {
    entries: Vec<(String, Value)>,
}

impl Dict {
    pub fn new() -> Self {
        Self::default()
    }

    /// Inserts a key; returns false (and leaves the dictionary unchanged)
    /// if the key is already present.
    pub fn insert_unique(&mut self, key: String, value: Value) -> bool {
        if self.get(&key).is_some() {
            return false;
        }
        self.entries.push((key, value));
        true
    }

    pub fn get(&self, key: &str) -> Option<&Value> {
        self.entries.iter().find(|(k, _)| k == key).map(|(_, v)| v)
    }

    pub fn get_mut(&mut self, key: &str) -> Option<&mut Value> {
        self.entries.iter_mut().find(|(k, _)| k == key).map(|(_, v)| v)
    }

    /// Removes a key, returning its value.
    pub fn remove(&mut self, key: &str) -> Option<Value> {
        let i = self.entries.iter().position(|(k, _)| k == key)?;
        Some(self.entries.remove(i).1)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&str, &Value)> {
        self.entries.iter().map(|(k, v)| (k.as_str(), v))
    }

    pub fn keys(&self) -> impl Iterator<Item = &str> {
        self.entries.iter().map(|(k, _)| k.as_str())
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

impl Value {
    pub fn kind(&self) -> &'static str {
        match self {
            Value::Bool(_) => "bool",
            Value::Integer(_) => "integer",
            Value::Real(_) => "real",
            Value::String(_) => "string",
            Value::Data(_) => "data",
            Value::Date(_) => "date",
            Value::Array(_) => "array",
            Value::Dict(_) => "dict",
            Value::Uid(_) => "uid",
            Value::Null => "null",
        }
    }

    pub fn as_dict(&self) -> Option<&Dict> {
        match self {
            Value::Dict(d) => Some(d),
            _ => None,
        }
    }

    pub fn as_array(&self) -> Option<&[Value]> {
        match self {
            Value::Array(a) => Some(a),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::String(s) => Some(s),
            _ => None,
        }
    }

    pub fn as_data(&self) -> Option<&[u8]> {
        match self {
            Value::Data(d) => Some(d),
            _ => None,
        }
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Value::Bool(b) => Some(*b),
            _ => None,
        }
    }

    pub fn as_integer(&self) -> Option<i128> {
        match self {
            Value::Integer(i) => Some(*i),
            _ => None,
        }
    }

    /// A number written either as `<integer>` or `<real>`.  Apple's tuning
    /// files use both spellings for the same field (for example
    /// `CalTempRangeMin` is `<integer>10</integer>` on one channel and
    /// `<real>10</real>` on the next), so numeric fields accept both.
    pub fn as_number(&self) -> Option<f64> {
        match self {
            Value::Integer(i) => Some(*i as f64),
            Value::Real(r) => Some(*r),
            _ => None,
        }
    }
}

/// Which encoding a document used.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub enum Format {
    Binary,
    Xml,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlistError {
    pub format: Format,
    /// Byte offset (binary) or 1-based line (XML) where the problem was found.
    pub at: usize,
    pub message: String,
}

impl fmt::Display for PlistError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.format {
            Format::Binary => write!(f, "bplist00 at byte {}: {}", self.at, self.message),
            Format::Xml => write!(f, "XML plist at line {}: {}", self.at, self.message),
        }
    }
}

impl std::error::Error for PlistError {}

/// Decodes a property list, choosing the format from its first bytes.
pub fn parse(bytes: &[u8]) -> Result<(Value, Format), PlistError> {
    if bytes.starts_with(b"bplist") {
        binary::parse(bytes).map(|v| (v, Format::Binary))
    } else {
        xml::parse(bytes).map(|v| (v, Format::Xml))
    }
}

/// True if the bytes look like a property list of either format.
pub fn sniff(bytes: &[u8]) -> Option<Format> {
    if bytes.starts_with(b"bplist00") {
        return Some(Format::Binary);
    }
    let start = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF][..]).unwrap_or(bytes);
    let head = &start[..start.len().min(512)];
    let text = String::from_utf8_lossy(head);
    let t = text.trim_start();
    if t.starts_with("<?xml") || t.starts_with("<!DOCTYPE plist") || t.starts_with("<plist") {
        Some(Format::Xml)
    } else {
        None
    }
}
