// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Property-list decoders: every bplist00 object type and width, XML
//! syntax, and rejection of malformed input (never a panic).

use aurora_speaker_tuning::plist::{self, binary, xml, Dict, Format, Value};

fn sample() -> Value {
    let mut inner = Dict::new();
    inner.insert_unique("x".into(), Value::Real(-0.25));
    inner.insert_unique("long key with more than fifteen chars".into(), Value::Bool(true));
    let mut d = Dict::new();
    d.insert_unique("bools".into(), Value::Array(vec![Value::Bool(true), Value::Bool(false)]));
    d.insert_unique(
        "ints".into(),
        Value::Array(vec![
            Value::Integer(0),
            Value::Integer(255),
            Value::Integer(256),
            Value::Integer(70_000),
            Value::Integer(5_000_000_000),
            Value::Integer(-3),
            Value::Integer(i64::MIN as i128),
            Value::Integer(u64::MAX as i128),
        ]),
    );
    d.insert_unique("real".into(), Value::Real(43.0625));
    d.insert_unique("ascii".into(), Value::String("plain".into()));
    d.insert_unique("unicode".into(), Value::String("Température ✓".into()));
    d.insert_unique("data".into(), Value::Data((0u8..=40).collect()));
    d.insert_unique("empty".into(), Value::Array(vec![]));
    d.insert_unique("many".into(), Value::Array((0..40).map(Value::Integer).collect()));
    d.insert_unique("nested".into(), Value::Dict(inner));
    Value::Dict(d)
}

#[test]
fn binary_roundtrip_all_types() {
    let v = sample();
    let b = binary::write(&v);
    let (back, f) = plist::parse(&b).unwrap();
    assert_eq!(f, Format::Binary);
    assert_eq!(back, v);
}

#[test]
fn xml_roundtrip_all_types() {
    let v = sample();
    let x = xml::write(&v);
    let (back, f) = plist::parse(x.as_bytes()).unwrap();
    assert_eq!(f, Format::Xml);
    assert_eq!(back, v);
}

/// Hand-assembled bplist with features the writer does not emit: f32 reals,
/// a date, a UID, a set, 1-byte references with shared objects, and a
/// 2-byte offset table.
#[test]
fn binary_hand_assembled() {
    let mut b = b"bplist00".to_vec();
    let mut offs = Vec::new();
    let mut obj = |b: &mut Vec<u8>, bytes: &[u8]| {
        offs.push(b.len() as u16);
        b.extend_from_slice(bytes);
    };
    // 0: dict {k0: 2, k1: 3, k2: 4, k3: 5, k4: 2 (shared)}
    obj(&mut b, &[0xD5, 1, 6, 7, 8, 9, 2, 3, 4, 5, 2]);
    // 1: "k0"
    obj(&mut b, &[0x52, b'k', b'0']);
    // 2: f32 1.5
    obj(&mut b, &[0x22, 0x3F, 0xC0, 0x00, 0x00]);
    // 3: date 1.0
    let mut d = vec![0x33];
    d.extend_from_slice(&1.0f64.to_be_bytes());
    obj(&mut b, &d);
    // 4: UID 7
    obj(&mut b, &[0x80, 7]);
    // 5: set {2}
    obj(&mut b, &[0xC1, 2]);
    // 6..9: keys
    obj(&mut b, &[0x52, b'k', b'1']);
    obj(&mut b, &[0x52, b'k', b'2']);
    obj(&mut b, &[0x52, b'k', b'3']);
    obj(&mut b, &[0x52, b'k', b'4']);
    let table = b.len() as u64;
    for o in &offs {
        b.extend_from_slice(&o.to_be_bytes());
    }
    b.extend_from_slice(&[0, 0, 0, 0, 0, 0, 2, 1]);
    b.extend_from_slice(&(offs.len() as u64).to_be_bytes());
    b.extend_from_slice(&0u64.to_be_bytes());
    b.extend_from_slice(&table.to_be_bytes());
    let v = binary::parse(&b).unwrap();
    let d = v.as_dict().unwrap();
    assert_eq!(d.get("k0"), Some(&Value::Real(1.5)));
    assert_eq!(d.get("k1"), Some(&Value::Date(1.0)));
    assert_eq!(d.get("k2"), Some(&Value::Uid(7)));
    assert_eq!(d.get("k3"), Some(&Value::Array(vec![Value::Real(1.5)])));
    assert_eq!(d.get("k4"), Some(&Value::Real(1.5)));
}

fn minimal(objects: &[&[u8]], top: u64) -> Vec<u8> {
    let mut b = b"bplist00".to_vec();
    let mut offs = Vec::new();
    for o in objects {
        offs.push(b.len() as u8);
        b.extend_from_slice(o);
    }
    let table = b.len() as u64;
    b.extend_from_slice(&offs);
    b.extend_from_slice(&[0, 0, 0, 0, 0, 0, 1, 1]);
    b.extend_from_slice(&(objects.len() as u64).to_be_bytes());
    b.extend_from_slice(&top.to_be_bytes());
    b.extend_from_slice(&table.to_be_bytes());
    b
}

#[test]
fn binary_rejections() {
    // Self-containing array.
    assert!(binary::parse(&minimal(&[&[0xA1, 0]], 0)).unwrap_err().message.contains("cycle"));
    // Duplicate key.
    assert!(binary::parse(&minimal(&[&[0xD2, 1, 1, 2, 2], &[0x51, b'a'], &[0x09]], 0))
        .unwrap_err()
        .message
        .contains("duplicate"));
    // Non-string key.
    assert!(binary::parse(&minimal(&[&[0xD1, 1, 1], &[0x10, 1]], 0)).is_err());
    // Reference out of range.
    assert!(binary::parse(&minimal(&[&[0xA1, 9]], 0)).is_err());
    // Truncated string.
    assert!(binary::parse(&minimal(&[&[0x55, b'a']], 0)).is_err());
    // Non-ASCII in an ASCII string.
    assert!(binary::parse(&minimal(&[&[0x51, 0xC3]], 0)).is_err());
    // Unknown marker.
    assert!(binary::parse(&minimal(&[&[0x70]], 0)).is_err());
    // Top object out of range.
    assert!(binary::parse(&minimal(&[&[0x09]], 3)).is_err());
    // Wrong version.
    let mut v = minimal(&[&[0x09]], 0);
    v[7] = b'1';
    assert!(binary::parse(&v).is_err());
    // Too short.
    assert!(binary::parse(b"bplist00").is_err());
}

/// Deterministic mutation fuzzing: no input may panic the decoders.
#[test]
fn mutations_never_panic() {
    let base = binary::write(&sample());
    let xbase = xml::write(&sample()).into_bytes();
    let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut next = || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x
    };
    for round in 0..4000 {
        for src in [&base, &xbase] {
            let mut m = src.clone();
            let edits = 1 + (next() % 6) as usize;
            for _ in 0..edits {
                let at = (next() as usize) % m.len();
                match next() % 3 {
                    0 => m[at] = next() as u8,
                    1 => m.truncate(at.max(1)),
                    _ => m.insert(at, next() as u8),
                }
            }
            let _ = plist::parse(&m);
        }
        if round == 0 {
            assert!(plist::parse(&base).is_ok());
        }
    }
}

#[test]
fn xml_syntax() {
    let doc = r#"<?xml version="1.0" encoding="UTF-8"?>
<!-- leading comment -->
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>a&amp;b</key><string>x &lt; y &#x41;&#66; <![CDATA[<raw>]]></string>
  <key>n</key><integer> -12 </integer>
  <key>h</key><integer>0x1F</integer>
  <key>r</key><real>1e-5</real>
  <key>t</key><true/>
  <key>f</key><false></false>
  <key>d</key><data>
     AAEC
     Aw==
  </data>
  <key>when</key><date>2001-01-02T00:00:00Z</date>
  <key>e</key><array/>
  <key>ed</key><dict/>
  <key>es</key><string/>
</dict>
</plist>
"#;
    let v = xml::parse(doc.as_bytes()).unwrap();
    let d = v.as_dict().unwrap();
    assert_eq!(d.get("a&b").unwrap().as_str(), Some("x < y AB <raw>"));
    assert_eq!(d.get("n"), Some(&Value::Integer(-12)));
    assert_eq!(d.get("h"), Some(&Value::Integer(31)));
    assert_eq!(d.get("r").unwrap().as_number(), Some(1e-5));
    assert_eq!(d.get("t"), Some(&Value::Bool(true)));
    assert_eq!(d.get("f"), Some(&Value::Bool(false)));
    assert_eq!(d.get("d"), Some(&Value::Data(vec![0, 1, 2, 3])));
    assert_eq!(d.get("when"), Some(&Value::Date(86400.0)));
    assert_eq!(d.get("es"), Some(&Value::String(String::new())));
}

#[test]
fn xml_rejections() {
    for bad in [
        "<plist><dict><key>a</key></dict></plist>",
        "<plist><dict><key>a</key><true/><key>a</key><true/></dict></plist>",
        "<plist><integer>1.5</integer></plist>",
        "<plist><real>abc</real></plist>",
        "<plist><string>&bogus;</string></plist>",
        "<plist><data>A=AA</data></plist>",
        "<plist><foo/></plist>",
        "<plist><true/><true/></plist>",
        "<plist><string>a</plist>",
        "<!DOCTYPE plist [ <!ENTITY x 'y'> ]><plist><true/></plist>",
        "<plist><true/></plist>trailing",
        "<plist><dict><string>a</string></dict></plist>",
    ] {
        assert!(xml::parse(bad.as_bytes()).is_err(), "{bad}");
    }
}
