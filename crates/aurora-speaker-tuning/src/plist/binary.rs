//! `bplist00` decoder.
//!
//! Layout: the 8-byte magic `bplist00`, the object area, an offset table, and
//! a 32-byte trailer (`[6]` sort version, `[6]` offset width, `[7]` reference
//! width, then three big-endian u64: object count, top object, offset-table
//! offset).  Each object starts with a marker byte whose high nibble is the
//! type and low nibble a size or count; a low nibble of 0xF means the count
//! follows as an integer object.
//!
//! The decoder checks every offset and length against the buffer, rejects
//! reference cycles, bounds the nesting depth, and bounds the total number of
//! decoded nodes so a small file with many shared references cannot expand
//! without limit.

use super::{Dict, Format, PlistError, Value};

const MAX_DEPTH: usize = 256;
const MAX_NODES: usize = 1 << 22;

fn err(at: usize, message: impl Into<String>) -> PlistError {
    PlistError { format: Format::Binary, at, message: message.into() }
}

struct Reader<'a> {
    buf: &'a [u8],
    offsets: Vec<usize>,
    ref_size: usize,
    /// Objects currently being decoded (cycle detection).
    active: Vec<bool>,
    nodes: usize,
    object_end: usize,
}

fn be_uint(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0u64, |acc, b| (acc << 8) | u64::from(*b))
}

pub fn parse(buf: &[u8]) -> Result<Value, PlistError> {
    if buf.len() < 8 + 1 + 32 {
        return Err(err(0, "too short for a bplist00 document"));
    }
    if &buf[..8] != b"bplist00" {
        let version = String::from_utf8_lossy(&buf[6..8]).into_owned();
        return Err(err(0, format!("unsupported binary plist version {version:?} (only 00)")));
    }
    let t = buf.len() - 32;
    let trailer = &buf[t..];
    let offset_size = trailer[6] as usize;
    let ref_size = trailer[7] as usize;
    let num_objects = be_uint(&trailer[8..16]);
    let top = be_uint(&trailer[16..24]);
    let table = be_uint(&trailer[24..32]);
    if !(1..=8).contains(&offset_size) {
        return Err(err(t + 6, format!("offset width {offset_size} not in 1..=8")));
    }
    if !(1..=8).contains(&ref_size) {
        return Err(err(t + 7, format!("reference width {ref_size} not in 1..=8")));
    }
    if num_objects == 0 {
        return Err(err(t + 8, "no objects"));
    }
    if top >= num_objects {
        return Err(err(t + 16, format!("top object {top} >= object count {num_objects}")));
    }
    // A reference must be able to name every object.
    if ref_size < 8 && num_objects > (1u64 << (8 * ref_size as u32)) {
        return Err(err(t + 7, "reference width too small for the object count"));
    }
    let table = usize::try_from(table).map_err(|_| err(t + 24, "offset table offset overflows"))?;
    let n = usize::try_from(num_objects).map_err(|_| err(t + 8, "object count overflows"))?;
    let table_len = n.checked_mul(offset_size).ok_or_else(|| err(t + 8, "offset table size overflows"))?;
    if table < 9 || table.checked_add(table_len).is_none_or(|end| end > t) {
        return Err(err(t + 24, "offset table lies outside the document"));
    }
    let mut offsets = Vec::with_capacity(n);
    for i in 0..n {
        let at = table + i * offset_size;
        let off = be_uint(&buf[at..at + offset_size]) as usize;
        if off < 8 || off >= table {
            return Err(err(at, format!("object {i} offset {off} outside the object area")));
        }
        offsets.push(off);
    }
    let mut r = Reader { buf, offsets, ref_size, active: vec![false; n], nodes: 0, object_end: table };
    r.object(top as usize, 0)
}

impl<'a> Reader<'a> {
    fn take(&self, at: usize, len: usize) -> Result<&'a [u8], PlistError> {
        match at.checked_add(len) {
            Some(end) if end <= self.object_end => Ok(&self.buf[at..end]),
            _ => Err(err(at, format!("object of {len} bytes runs past the object area"))),
        }
    }

    /// Reads the count for a marker whose low nibble is `low`; returns
    /// (count, offset of the payload).
    fn count(&self, at: usize, low: u8) -> Result<(usize, usize), PlistError> {
        if low != 0x0F {
            return Ok((low as usize, at + 1));
        }
        let m = *self.take(at + 1, 1)?.first().unwrap();
        if m >> 4 != 0x1 {
            return Err(err(at + 1, format!("extended count marker 0x{m:02x} is not an integer")));
        }
        let width = 1usize << (m & 0x0F);
        if width > 8 {
            return Err(err(at + 1, "extended count wider than 8 bytes"));
        }
        let raw = be_uint(self.take(at + 2, width)?);
        let count = usize::try_from(raw).map_err(|_| err(at + 2, "count overflows"))?;
        Ok((count, at + 2 + width))
    }

    fn reference(&self, at: usize) -> Result<usize, PlistError> {
        let raw = be_uint(self.take(at, self.ref_size)?);
        let idx = usize::try_from(raw).map_err(|_| err(at, "reference overflows"))?;
        if idx >= self.offsets.len() {
            return Err(err(at, format!("reference {idx} >= object count {}", self.offsets.len())));
        }
        Ok(idx)
    }

    fn refs(&self, at: usize, count: usize) -> Result<Vec<usize>, PlistError> {
        let len = count.checked_mul(self.ref_size).ok_or_else(|| err(at, "reference list size overflows"))?;
        self.take(at, len)?;
        (0..count).map(|i| self.reference(at + i * self.ref_size)).collect()
    }

    fn object(&mut self, idx: usize, depth: usize) -> Result<Value, PlistError> {
        let at = self.offsets[idx];
        if depth > MAX_DEPTH {
            return Err(err(at, format!("nesting deeper than {MAX_DEPTH}")));
        }
        if self.active[idx] {
            return Err(err(at, format!("object {idx} contains itself (reference cycle)")));
        }
        self.nodes += 1;
        if self.nodes > MAX_NODES {
            return Err(err(at, "too many decoded objects (shared-reference expansion)"));
        }
        let marker = self.take(at, 1)?[0];
        let (high, low) = (marker >> 4, marker & 0x0F);
        let value = match high {
            0x0 => match low {
                0x0 => Value::Null,
                0x8 => Value::Bool(false),
                0x9 => Value::Bool(true),
                _ => return Err(err(at, format!("marker 0x{marker:02x} is not an object"))),
            },
            0x1 => {
                if low > 4 {
                    return Err(err(at, format!("integer of 2^{low} bytes")));
                }
                let width = 1usize << low;
                let b = self.take(at + 1, width)?;
                Value::Integer(match width {
                    // 1, 2 and 4-byte integers are unsigned.
                    1 | 2 | 4 => be_uint(b) as i128,
                    8 => be_uint(b) as i64 as i128,
                    16 => i128::from_be_bytes(b.try_into().unwrap()),
                    _ => unreachable!(),
                })
            }
            0x2 => match low {
                2 => Value::Real(f32::from_be_bytes(self.take(at + 1, 4)?.try_into().unwrap()) as f64),
                3 => Value::Real(f64::from_be_bytes(self.take(at + 1, 8)?.try_into().unwrap())),
                _ => return Err(err(at, format!("real of 2^{low} bytes"))),
            },
            0x3 => {
                if low != 3 {
                    return Err(err(at, format!("date marker 0x{marker:02x}")));
                }
                Value::Date(f64::from_be_bytes(self.take(at + 1, 8)?.try_into().unwrap()))
            }
            0x4 => {
                let (n, p) = self.count(at, low)?;
                Value::Data(self.take(p, n)?.to_vec())
            }
            0x5 => {
                let (n, p) = self.count(at, low)?;
                let b = self.take(p, n)?;
                if let Some(pos) = b.iter().position(|c| *c >= 0x80) {
                    return Err(err(p + pos, "non-ASCII byte in an ASCII string"));
                }
                Value::String(String::from_utf8(b.to_vec()).unwrap())
            }
            0x6 => {
                let (n, p) = self.count(at, low)?;
                let len = n.checked_mul(2).ok_or_else(|| err(at, "UTF-16 length overflows"))?;
                let b = self.take(p, len)?;
                let units: Vec<u16> = b.chunks_exact(2).map(|c| u16::from_be_bytes([c[0], c[1]])).collect();
                Value::String(String::from_utf16(&units).map_err(|_| err(p, "invalid UTF-16 string"))?)
            }
            0x8 => {
                let b = self.take(at + 1, low as usize + 1)?;
                if b.len() > 8 {
                    return Err(err(at, "UID wider than 8 bytes"));
                }
                Value::Uid(be_uint(b))
            }
            0xA | 0xC => {
                // 0xC (set) has no plist-level equivalent in XML; it decodes
                // to an array, which is how Foundation exposes it too.
                let (n, p) = self.count(at, low)?;
                let refs = self.refs(p, n)?;
                self.active[idx] = true;
                let mut items = Vec::with_capacity(n.min(4096));
                for r in refs {
                    items.push(self.object(r, depth + 1)?);
                }
                self.active[idx] = false;
                Value::Array(items)
            }
            0xD => {
                let (n, p) = self.count(at, low)?;
                let keys = self.refs(p, n)?;
                let vals = self.refs(p + n * self.ref_size, n)?;
                self.active[idx] = true;
                let mut dict = Dict::new();
                for (k, v) in keys.into_iter().zip(vals) {
                    let key = match self.object(k, depth + 1)? {
                        Value::String(s) => s,
                        other => return Err(err(self.offsets[k], format!("dictionary key is a {}", other.kind()))),
                    };
                    let value = self.object(v, depth + 1)?;
                    if !dict.insert_unique(key.clone(), value) {
                        return Err(err(at, format!("duplicate dictionary key {key:?}")));
                    }
                }
                self.active[idx] = false;
                Value::Dict(dict)
            }
            _ => return Err(err(at, format!("unknown object marker 0x{marker:02x}"))),
        };
        Ok(value)
    }
}

/// A minimal `bplist00` writer, used by the synthetic fixtures and tests.
///
/// It emits each value as its own object (no sharing), integers in the
/// narrowest legal width, and reals as f64.
pub fn write(value: &Value) -> Vec<u8> {
    struct W {
        objects: Vec<Vec<u8>>,
    }
    fn count_marker(out: &mut Vec<u8>, high: u8, n: usize) {
        if n < 15 {
            out.push((high << 4) | n as u8);
        } else {
            out.push((high << 4) | 0x0F);
            int_body(out, n as i128);
        }
    }
    fn int_body(out: &mut Vec<u8>, v: i128) {
        if (0..=0xFF).contains(&v) {
            out.push(0x10);
            out.push(v as u8);
        } else if (0..=0xFFFF).contains(&v) {
            out.push(0x11);
            out.extend_from_slice(&(v as u16).to_be_bytes());
        } else if (0..=0xFFFF_FFFF).contains(&v) {
            out.push(0x12);
            out.extend_from_slice(&(v as u32).to_be_bytes());
        } else if v >= i64::MIN as i128 && v <= i64::MAX as i128 {
            out.push(0x13);
            out.extend_from_slice(&(v as i64).to_be_bytes());
        } else {
            out.push(0x14);
            out.extend_from_slice(&v.to_be_bytes());
        }
    }
    impl W {
        fn add(&mut self, v: &Value) -> usize {
            let idx = self.objects.len();
            self.objects.push(Vec::new());
            let mut out = Vec::new();
            match v {
                Value::Null => out.push(0x00),
                Value::Bool(b) => out.push(if *b { 0x09 } else { 0x08 }),
                Value::Integer(i) => int_body(&mut out, *i),
                Value::Real(r) => {
                    out.push(0x23);
                    out.extend_from_slice(&r.to_be_bytes());
                }
                Value::Date(d) => {
                    out.push(0x33);
                    out.extend_from_slice(&d.to_be_bytes());
                }
                Value::Data(d) => {
                    count_marker(&mut out, 0x4, d.len());
                    out.extend_from_slice(d);
                }
                Value::String(s) => {
                    if s.is_ascii() {
                        count_marker(&mut out, 0x5, s.len());
                        out.extend_from_slice(s.as_bytes());
                    } else {
                        let units: Vec<u16> = s.encode_utf16().collect();
                        count_marker(&mut out, 0x6, units.len());
                        for u in units {
                            out.extend_from_slice(&u.to_be_bytes());
                        }
                    }
                }
                Value::Uid(u) => {
                    out.push(0x87);
                    out.extend_from_slice(&u.to_be_bytes());
                }
                Value::Array(items) => {
                    let refs: Vec<usize> = items.iter().map(|i| self.add(i)).collect();
                    count_marker(&mut out, 0xA, refs.len());
                    for r in refs {
                        out.extend_from_slice(&(r as u32).to_be_bytes());
                    }
                }
                Value::Dict(d) => {
                    let mut krefs = Vec::new();
                    let mut vrefs = Vec::new();
                    for (k, val) in d.iter() {
                        krefs.push(self.add(&Value::String(k.to_string())));
                        vrefs.push(self.add(val));
                    }
                    count_marker(&mut out, 0xD, krefs.len());
                    for r in krefs.into_iter().chain(vrefs) {
                        out.extend_from_slice(&(r as u32).to_be_bytes());
                    }
                }
            }
            self.objects[idx] = out;
            idx
        }
    }
    let mut w = W { objects: Vec::new() };
    let top = w.add(value);
    let mut buf = b"bplist00".to_vec();
    let mut offsets = Vec::new();
    for o in &w.objects {
        offsets.push(buf.len() as u64);
        buf.extend_from_slice(o);
    }
    let table = buf.len() as u64;
    for o in &offsets {
        buf.extend_from_slice(&o.to_be_bytes());
    }
    buf.extend_from_slice(&[0, 0, 0, 0, 0, 0, 8, 4]);
    buf.extend_from_slice(&(w.objects.len() as u64).to_be_bytes());
    buf.extend_from_slice(&(top as u64).to_be_bytes());
    buf.extend_from_slice(&table.to_be_bytes());
    buf
}
