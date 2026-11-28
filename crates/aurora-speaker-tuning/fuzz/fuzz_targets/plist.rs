//! Both property-list decoders on any bytes. Nothing may panic, overflow
//! the stack or run away; a document the binary decoder accepts writes back
//! (`binary::write`) to one that decodes to the same value, and so does
//! one written as XML, up to what XML cannot carry (dates, UIDs and the
//! null become reals, integers and empty strings).
#![no_main]

use aurora_speaker_tuning::plist::{self, binary, xml, Value};
use libfuzzer_sys::fuzz_target;

/// The value as an XML document can hold it, with reals compared by bits.
fn as_xml(v: &Value) -> Value {
    match v {
        Value::Date(d) => Value::Real(*d),
        Value::Uid(u) => Value::Integer(i128::from(*u)),
        Value::Null => Value::String(String::new()),
        Value::Array(a) => Value::Array(a.iter().map(as_xml).collect()),
        Value::Dict(d) => {
            let mut out = plist::Dict::new();
            for (k, x) in d.iter() {
                out.insert_unique(k.to_string(), as_xml(x));
            }
            Value::Dict(out)
        }
        o => o.clone(),
    }
}

/// Equality with NaN equal to itself (a real survives by its bits).
fn same(a: &Value, b: &Value) -> bool {
    match (a, b) {
        (Value::Real(x), Value::Real(y)) | (Value::Date(x), Value::Date(y)) => {
            x.to_bits() == y.to_bits() || (x.is_nan() && y.is_nan())
        }
        (Value::Array(x), Value::Array(y)) => x.len() == y.len() && x.iter().zip(y).all(|(p, q)| same(p, q)),
        (Value::Dict(x), Value::Dict(y)) => {
            x.len() == y.len() && x.iter().zip(y.iter()).all(|((k, p), (l, q))| k == l && same(p, q))
        }
        _ => a == b,
    }
}

/// XML text carries no NUL and no other C0 control but tab, LF and CR,
/// and a CR does not survive line-end normalisation.
fn xml_safe(v: &Value) -> bool {
    let ok = |s: &str| s.chars().all(|c| c == '\t' || c == '\n' || c >= ' ');
    match v {
        Value::String(s) => ok(s),
        Value::Real(r) | Value::Date(r) => r.is_finite(),
        Value::Array(a) => a.iter().all(xml_safe),
        Value::Dict(d) => d.iter().all(|(k, x)| ok(k) && xml_safe(x)),
        _ => true,
    }
}

fuzz_target!(|data: &[u8]| {
    let _ = plist::sniff(data);
    let b = binary::parse(data);
    let x = xml::parse(data);
    for v in [b.ok(), x.ok()].into_iter().flatten() {
        let again = binary::parse(&binary::write(&v)).expect("binary::write output does not decode");
        assert!(same(&v, &again), "binary round trip:\n{v:?}\n{again:?}");
        if xml_safe(&v) {
            let text = xml::write(&v);
            let back = xml::parse(text.as_bytes()).expect("xml::write output does not decode");
            assert!(same(&as_xml(&v), &back), "XML round trip:\n{v:?}\n{back:?}");
        }
    }
});
