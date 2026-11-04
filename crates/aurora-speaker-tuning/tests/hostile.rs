// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Hostile inputs from the PR 32 review: each one used to overflow the
//! stack, run for minutes, or allocate hundreds of megabytes.  Each must
//! now fail fast with an error.  A stack overflow aborts this test binary,
//! so a regression cannot pass silently.

use aurora_speaker_tuning::dspg::{self, analysis, Endpoint};
use aurora_speaker_tuning::plist::{self, Value};
use std::collections::BTreeMap;
use std::time::{Duration, Instant};

fn none() -> BTreeMap<String, String> {
    BTreeMap::new()
}

/// Runs `f`, which must finish within `limit`.
fn quick<T>(limit: Duration, f: impl FnOnce() -> T) -> T {
    let t = Instant::now();
    let r = f();
    assert!(t.elapsed() < limit, "took {:?}", t.elapsed());
    r
}

#[test]
fn deeply_nested_brackets_are_refused() {
    let s = "[".repeat(200_000);
    let e = quick(Duration::from_secs(5), || dspg::parse(&s, &none(), "g")).unwrap_err();
    assert!(e.to_string().contains("nested deeper"), "{e}");
    let s = "{".repeat(200_000);
    assert!(dspg::parse(&s, &none(), "g").is_err());
}

#[test]
fn deeply_nested_parentheses_are_refused() {
    let s = format!("box {}", "(".repeat(200_000));
    let e = quick(Duration::from_secs(5), || dspg::parse(&s, &none(), "g")).unwrap_err();
    assert!(e.to_string().contains("nested deeper"), "{e}");
}

#[test]
fn exponential_macro_expansion_is_capped() {
    // 457 bytes that expand to 10^8 tokens.
    let mut s = String::from("[def a0 x x x x x x x x x x]\n");
    for i in 1..8 {
        let p = i - 1;
        s.push_str(&format!("[def a{i}{}]\n", format!(" [a{p}]").repeat(10)));
    }
    s.push_str("graphName [a7]\n");
    assert!(s.len() < 500);
    let e = quick(Duration::from_secs(10), || dspg::parse(&s, &none(), "g")).unwrap_err();
    let msg = e.to_string();
    assert!(msg.contains("exceeds"), "{msg}");
    assert!(msg.len() < 400, "error message embeds the expansion: {} bytes", msg.len());
}

#[test]
fn self_recursive_macro_fails_fast() {
    let s = "[def a [a] [a]]\ngraphName [a]\n";
    assert!(quick(Duration::from_secs(5), || dspg::parse(s, &none(), "g")).is_err());
}

#[test]
fn long_statement_is_quoted_truncated() {
    let s = format!("graphName {}\n", "x ".repeat(100_000));
    let e = dspg::parse(&s, &none(), "g").unwrap_err().to_string();
    assert!(e.len() < 400, "{} bytes", e.len());
}

#[test]
fn huge_channel_count_is_refused() {
    let s = "in i0\nout o0\nbox P (aufx spp3) 1 1\nwire i0 P (48000 4000000000)\nwire P o0 (48000 4000000000)\n";
    let g = dspg::parse(s, &none(), "g").unwrap();
    let e = quick(Duration::from_secs(5), || analysis::lanes_into(&g, &Endpoint { node: "P".into(), port: 0 }))
        .unwrap_err();
    assert!(e.to_string().contains("at most"), "{e}");
}

fn chain(n: usize) -> String {
    let mut s = String::from("in i0\nout o0\n");
    for k in 0..n {
        s.push_str(&format!("box b{k} gain 1 1\n"));
    }
    s.push_str("wire i0 b0 (48000 2)\n");
    for k in 1..n {
        s.push_str(&format!("wire b{} b{k} (48000 2)\n", k - 1));
    }
    s.push_str(&format!("wire b{} o0 (48000 2)\n", n - 1));
    s
}

#[test]
fn long_box_chain_is_refused_not_overflowed() {
    // 10k boxes: over the box cap.
    assert!(dspg::parse(&chain(10_000), &none(), "g").is_err());
    // Under the box cap but past the tracer's depth: an error, no overflow.
    let g = dspg::parse(&chain(1000), &none(), "g").unwrap();
    let e = analysis::lanes_into(&g, &Endpoint { node: "b999".into(), port: 0 }).unwrap_err();
    assert!(e.to_string().contains("longer than"), "{e}");
    // A realistic chain still traces.
    let g = dspg::parse(&chain(64), &none(), "g").unwrap();
    assert_eq!(analysis::lanes_into(&g, &Endpoint { node: "b63".into(), port: 0 }).unwrap().len(), 2);
}

/// obj0: a `data_len`-byte blob; obj1: `fan1` references to obj0; obj2:
/// `fan2` references to obj1 (the top object).
fn bplist_amplify(data_len: usize, fan1: usize, fan2: usize) -> Vec<u8> {
    let mut buf = b"bplist00".to_vec();
    let mut offs = vec![buf.len() as u64];
    buf.push(0x4F);
    buf.push(0x12);
    buf.extend_from_slice(&(data_len as u32).to_be_bytes());
    buf.extend(std::iter::repeat_n(0xAA, data_len));
    offs.push(buf.len() as u64);
    buf.push(0xAF);
    buf.push(0x11);
    buf.extend_from_slice(&(fan1 as u16).to_be_bytes());
    for _ in 0..fan1 {
        buf.extend_from_slice(&0u16.to_be_bytes());
    }
    offs.push(buf.len() as u64);
    buf.push(0xAF);
    buf.push(0x11);
    buf.extend_from_slice(&(fan2 as u16).to_be_bytes());
    for _ in 0..fan2 {
        buf.extend_from_slice(&1u16.to_be_bytes());
    }
    let table = buf.len() as u64;
    for o in &offs {
        buf.extend_from_slice(&o.to_be_bytes());
    }
    buf.extend_from_slice(&[0, 0, 0, 0, 0, 0, 8, 2]);
    buf.extend_from_slice(&3u64.to_be_bytes());
    buf.extend_from_slice(&2u64.to_be_bytes());
    buf.extend_from_slice(&table.to_be_bytes());
    buf
}

#[test]
fn shared_reference_amplification_is_capped() {
    // 65,870 bytes that decoded to 268 MB.
    let f = bplist_amplify(64 << 10, 64, 64);
    assert!(f.len() < 70_000);
    let e = quick(Duration::from_secs(5), || plist::parse(&f)).unwrap_err();
    assert!(e.to_string().contains("shared-reference"), "{e}");
    // Modest sharing still decodes.
    let f = bplist_amplify(16, 4, 4);
    let (v, _) = plist::parse(&f).unwrap();
    let Value::Array(a) = v else { panic!() };
    assert_eq!(a.len(), 4);
}

#[test]
fn object_count_beyond_the_object_area_is_refused() {
    let mut f = bplist_amplify(16, 2, 2);
    let n = f.len();
    // Claim 2^40 objects.
    f[n - 24..n - 16].copy_from_slice(&(1u64 << 40).to_be_bytes());
    assert!(plist::parse(&f).is_err());
}

#[test]
fn xml_integer_sign_inside_hex_is_refused() {
    for body in ["-0x-80000000000000000000000000000000", "0x-5", "0x+5", "-0x80000000000000000000000000000000"] {
        let x = format!("<?xml version=\"1.0\"?><plist version=\"1.0\"><integer>{body}</integer></plist>");
        assert!(plist::parse(x.as_bytes()).is_err(), "{body}");
    }
    let x = "<?xml version=\"1.0\"?><plist version=\"1.0\"><integer>-0x10</integer></plist>";
    assert_eq!(plist::parse(x.as_bytes()).unwrap().0, Value::Integer(-16));
}
