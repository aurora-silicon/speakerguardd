// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Saved AU state, strips, the graph language and its analysis, and the
//! spp3 sidecar, on synthetic inputs.

mod common;

use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::dspg::{self, analysis, BoxKind, Endpoint};
use aurora_speaker_tuning::plist::{binary, Value};
use aurora_speaker_tuning::spp3::{self, Sidecar};
use aurora_speaker_tuning::strip::{AuStrip, PropStrip, PropValue};
use aurora_speaker_tuning::{Diagnostics, Error, Severity};
use common::*;
use std::collections::BTreeMap;

#[test]
fn param_state_roundtrip_and_rejections() {
    let st = spp3_state(3);
    let blob = st.encode();
    assert_eq!(blob.len(), 4 * 12 + (3 + 3 * 3) * 8);
    let back = ParamState::decode(&blob, "t").unwrap();
    assert_eq!(back, st);
    assert_eq!(back.elements(4).len(), 3);
    assert_eq!(back.global().unwrap().get(3), Some(11.0));

    let mut trailing = blob.clone();
    trailing.extend_from_slice(&[0, 0, 0]);
    assert!(ParamState::decode(&trailing, "t").is_err());
    let mut short = blob.clone();
    short.truncate(blob.len() - 4);
    assert!(ParamState::decode(&short, "t").is_err());
    let rep = ParamState {
        sections: vec![
            ParamSection { scope: 0, element: 0, params: vec![(1, 1.0)] },
            ParamSection { scope: 0, element: 0, params: vec![(2, 1.0)] },
        ],
    };
    assert!(ParamState::decode(&rep.encode(), "t").is_err());
    let dupid = ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: vec![(1, 1.0), (1, 2.0)] }] };
    assert!(ParamState::decode(&dupid.encode(), "t").is_err());
    let nan = ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: vec![(1, f32::NAN)] }] };
    assert!(ParamState::decode(&nan.encode(), "t").is_err());
    assert_eq!(ParamState::decode(&[], "t").unwrap().sections.len(), 0);
}

#[test]
fn austrip_and_propstrip() {
    let s = AuStrip::parse(&strip("spp3", 4), "strip").unwrap();
    let p = s.effect("Prot").unwrap();
    assert_eq!(p.unit.subtype.to_string(), "spp3");
    assert!(p.preset.has_state);
    assert_eq!(p.preset.state.elements(4).len(), 4);
    let spl = s.effect("Spl").unwrap();
    assert!(!spl.preset.has_state);
    assert_eq!(spl.preset.extra_keys(), vec!["classname".to_string()]);

    let ps = PropStrip::parse(&propstrip("x.plist"), "ps").unwrap();
    let b = ps.get("Prot").unwrap();
    assert_eq!(b.property(64004), Some(&PropValue::UInt32(1)));
    assert!(matches!(b.property(64003), Some(PropValue::Path(p)) if p.ends_with("/x.plist")));

    // An effect whose preset is for another unit is rejected.
    let bad = binary::write(&dict(vec![(
        "strips",
        Value::Array(vec![dict(vec![(
            "effects",
            Value::Array(vec![effect("Prot", "spp3", preset("lsm2", "x", &spp3_state(1)))]),
        )])]),
    )]));
    assert!(AuStrip::parse(&bad, "bad").is_err());
}

fn subst() -> BTreeMap<String, String> {
    [("fs".to_string(), "48000".to_string())].into()
}

#[test]
fn dspg_parse_preprocess_and_lanes() {
    let g = dspg::parse(&graph_text("spp3"), &subst(), "g").unwrap();
    assert_eq!(g.name.as_deref(), Some("test_speaker_out"));
    assert!(g.unresolved.is_empty());
    // The [if] chose the else-branch: Prot is the AudioUnit.
    let prot = g.find_box("Prot").unwrap();
    assert!(matches!(prot.kind, BoxKind::AudioUnit { .. }));
    assert_eq!(prot.inputs, 2);
    assert_eq!(g.macros.get("blk").map(String::as_str), Some("256"));

    let order = analysis::processing_order(&g).unwrap();
    let pos = |n: &str| order.order.iter().position(|x| x == n).unwrap();
    assert!(pos("Perm") < pos("Prot") && pos("Prot") < pos("Fade") && pos("Conv") < pos("Prot"));

    let lanes = analysis::lanes_into(&g, &Endpoint { node: "Prot".into(), port: 0 }).unwrap();
    let got: Vec<(u32, u32)> = lanes.iter().map(|l| (l.origin.channel, l.hops[0].input)).collect();
    // [L via Up 0, R via Up 2, L via Up 1, R via Up 3]
    assert_eq!(got, vec![(0, 0), (1, 2), (0, 1), (1, 3)]);
    assert_eq!(lanes[1].hops.last().unwrap().join, "Perm");
    assert_eq!(g.param_links[0].target.id.value(), Some(15.0));
    assert_eq!(g.property_links[0].target.id.value(), Some(64005.0));
}

#[test]
fn dspg_if_true_branch_and_unresolved() {
    let alt = graph_text("spp3").replace("[def USE_ALT 0]", "[def USE_ALT 1]");
    // The true branch declares Prot as a 1-in builtin, so the sense wire into
    // (Prot 1) no longer names a port: rejected, not silently dropped.
    assert!(dspg::parse(&alt, &subst(), "g").is_err());
    // Without the fs substitution the graph still parses, fs stays symbolic.
    let g = dspg::parse(&graph_text("spp3"), &BTreeMap::new(), "g").unwrap();
    assert_eq!(g.unresolved, vec!["fs".to_string()]);
    // A condition on an unresolved macro is an error.
    assert!(dspg::parse("[if [== [nope] 1] { in a }]", &BTreeMap::new(), "g").is_err());
}

#[test]
fn dspg_rejections() {
    let s = subst();
    for bad in [
        "frobnicate x",
        "box A split 1",
        "box A split 1 2\nbox A join 2 1",
        "in a\nout b\nwire a c ([fs] 1)",
        "in a\nout b\nbox A dbgn 1 1\nwire a (A 1) ([fs] 1)",
        "in a\nout b\nwire a b ([fs] 1)\nwire a b ([fs] 1)",
        "in a\nout b\nwire a b undefinedFormat",
        "[def]",
        "box A (aufx toolong) 1 1",
        "graphName \"unterminated",
        "box A dbgn 1 1 )",
        "[unknown op here]",
    ] {
        assert!(dspg::parse(bad, &s, "g").is_err(), "{bad}");
    }
}

#[test]
fn dspg_split_width_errors() {
    let s = subst();
    let g = dspg::parse(
        "in in\nout out\nbox S split 1 2\nbox J join 2 1\nwire in S ([fs] 3)\nwire (S 0) (J 0) ([fs] 1)\nwire (S 1) (J 1) ([fs] 1)\nwire J out ([fs] 2)",
        &s,
        "g",
    )
    .unwrap();
    // The split covers 2 of 3 channels: refused.
    assert!(analysis::lanes_into(&g, &Endpoint { node: "out".into(), port: 0 }).is_err());
    let cyc = dspg::parse("box A dbgn 1 1\nbox B dbgn 1 1\nwire A B ([fs] 1)\nwire B A ([fs] 1)", &s, "g").unwrap();
    assert!(analysis::processing_order(&cyc).is_err());
}

#[test]
fn sidecar_parses_and_validates() {
    let mut d = Diagnostics::default();
    let s = Sidecar::parse(&binary::write(&sidecar_value(4)), "sc", &mut d).unwrap();
    assert_eq!(s.num_channels, 4);
    assert_eq!(s.channels[0].ol_thermal.t_sett_vc, 17.0);
    assert_eq!(s.channels[0].ol_thermal.temperature_limit, 121.0);
    assert_eq!(s.channels[3].cl.as_ref().unwrap().cal_temp_range_min, 15.0);
    assert_eq!(s.groups(), vec![(0, vec![0, 1]), (1, vec![2, 3])]);
    assert_eq!(s.globals.value("Thermal_AmbientTemp"), Some(41.0));
    assert!(d.items.is_empty(), "{:?}", d.items);
    // The same document as XML.
    let x = aurora_speaker_tuning::plist::xml::write(&sidecar_value(4));
    assert_eq!(Sidecar::parse(x.as_bytes(), "sc", &mut d).unwrap(), s);
}

fn at<'a>(v: &'a mut Value, path: &[&str]) -> &'a mut aurora_speaker_tuning::plist::Dict {
    let mut cur = v;
    for p in path {
        let Value::Dict(d) = cur else { panic!("{p}") };
        cur = d.get_mut(p).expect("path");
    }
    let Value::Dict(d) = cur else { panic!() };
    d
}

fn without(v: &Value, path: &[&str], key: &str) -> Value {
    let mut v = v.clone();
    at(&mut v, path).remove(key).expect("key");
    v
}

fn with(v: &Value, path: &[&str], key: &str, val: Value) -> Value {
    let mut v = v.clone();
    let d = at(&mut v, path);
    d.remove(key);
    d.insert_unique(key.to_string(), val);
    v
}

#[test]
fn sidecar_rejections_and_warnings() {
    let base = sidecar_value(2);
    let parse = |v: &Value| {
        let mut d = Diagnostics::default();
        let r = Sidecar::parse(&binary::write(v), "sc", &mut d);
        (r, d)
    };
    let ch = ["ChannelSpecificParams", "Channel1"];
    let th = ["ChannelSpecificParams", "Channel1", "OL_thermal"];
    // Missing essential fields.
    assert!(matches!(parse(&without(&base, &th, "tau_Tvc")).0, Err(Error::Missing { .. })));
    assert!(matches!(parse(&without(&base, &[], "Thermal_AmbientTemp")).0, Err(Error::Missing { .. })));
    // Wrong types and ranges.
    assert!(matches!(parse(&with(&base, &th, "Reb_ref", s("3"))).0, Err(Error::Type { .. })));
    assert!(matches!(parse(&with(&base, &th, "tau_Tvc", r(0.0))).0, Err(Error::Range { .. })));
    assert!(matches!(parse(&with(&base, &th, "TemperatureLimit", r(30.0))).0, Err(Error::Range { .. })));
    assert!(parse(&with(&base, &[], "Version", i(10))).0.is_err());
    assert!(parse(&with(&base, &[], "NumChannels", i(3))).0.is_err());
    assert!(parse(&with(&base, &ch, "SpeakerGroup", i(-1))).0.is_err());
    // Unknown keys warn, optional absences warn, the rest stays valid.
    let (ok, d) = parse(&with(&base, &th, "NewField", r(1.0)));
    assert!(ok.is_ok());
    assert!(d.items.iter().any(|x| x.code == "spp3-unknown-key" && x.severity == Severity::Warning));
    let cl = ["ChannelSpecificParams", "Channel1", "CL"];
    let (ok, d) = parse(&without(&base, &cl, "PilotEnableThres"));
    assert!(ok.unwrap().channels[1].cl.as_ref().unwrap().pilot_enable_thres.is_none());
    assert!(d.items.iter().any(|x| x.code == "spp3-optional-absent"));
    // A channel without CL is open loop only.
    let (ok, _) = parse(&without(&base, &ch, "CL"));
    assert!(ok.unwrap().channels[1].cl.is_none());
}

#[test]
fn sidecar_cross_check_against_saved_state() {
    let mut d = Diagnostics::default();
    let s = Sidecar::parse(&binary::write(&sidecar_value(2)), "sc", &mut d).unwrap();
    // spp3_state carries ambient 41, attack 3.5, release 11, limit 121, Rvc 17, Reb 3.3: all agree.
    spp3::cross_check(&s, &spp3_state(2), "p", &mut d);
    assert!(!d.items.iter().any(|x| x.code == "spp3-preset-differs"), "{:?}", d.items);
    let mut st = spp3_state(2);
    st.sections[1].params[1].1 = 99.0;
    spp3::cross_check(&s, &st, "p", &mut d);
    assert!(d.items.iter().any(|x| x.code == "spp3-preset-differs"));
    let mut d2 = Diagnostics::default();
    spp3::cross_check(&s, &spp3_state(3), "p", &mut d2);
    assert!(d2.has_errors());
}
