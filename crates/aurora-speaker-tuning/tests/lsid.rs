// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//! Synthetic topology and control-only graph cases.
use aurora_speaker_tuning::{
    aupreset::{AuPreset, ParamSection, ParamState, UnitId},
    dspg::{self, analysis, BoxKind, Endpoint},
    fourcc::FourCc,
    lsid,
    plist::Dict,
    strip::{AuStrip, Effect, PropBox, PropStrip, PropValue, Property},
    Diagnostics,
};
use std::collections::BTreeMap;
mod common;
fn fcc(s: &str) -> FourCc {
    FourCc::from_str_exact(s).unwrap()
}
fn effect(name: &str, kind: &str, subtype: &str, manufacturer: &str, state: ParamState) -> Effect {
    let unit = UnitId { kind: fcc(kind), subtype: fcc(subtype), manufacturer: fcc(manufacturer) };
    Effect {
        box_name: name.into(),
        bypass: false,
        unit,
        preset: AuPreset { unit, name: None, version: None, has_state: true, state, extra: Dict::new() },
    }
}
fn strip() -> AuStrip {
    let records = sgv2_core::lsid::schema_ids()
        .into_iter()
        .map(|id| {
            // Valid synthetic acquisition geometry is required even while the
            // mandatory companion mute keeps adaptation unqualified.
            let value = match id {
                2048 | 2049 | 9216 | 9217 | 16384 | 16385 => 1.0,
                2054 | 9222 | 16390 => 32.0,
                3075 | 10243 | 17411 => 3.0,
                _ => 0.0,
            };
            (id, value)
        })
        .collect();
    AuStrip {
        strips: vec![vec![effect(
            "Estimator",
            "aufx",
            "lsid",
            "appl",
            ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: records }] },
        )]],
    }
}
fn graph_text() -> String {
    let mut text = String::from("in program\nin sense\nout out\nbox S split 1 2\nbox D join 6 1\nbox P split 1 6\nbox J join 6 1\nbox LSM (aufx lsm2) 1 1\nbox Convert srlconverter 1 1\nbox TS split 1 12\nbox TJ join 12 1\nbox Align delay 1 1\nbox Estimator (aufx lsid) 1 1\nbox End dead 1 0\nwire program S (48000 2)\n");
    for i in 0..6 {
        text.push_str(&format!("wire (S {}) (D {i}) (48000 1)\n", usize::from(i >= 3)));
    }
    text.push_str("wire D P (48000 6)\n");
    for (dst, src) in sgv2_core::lsid::PERMUTATION.iter().enumerate() {
        text.push_str(&format!("wire (P {src}) (J {dst}) (48000 1)\n"));
    }
    text.push_str(
        "wire J LSM (48000 6)\nwire LSM out (48000 6)\nwire sense Convert (48000 12i)\nwire Convert TS (48000 12)\n",
    );
    for lane in 0..12 {
        if lane == 1 {
            text.push_str("wire (TS 1) Align (48000 1)\nwire Align (TJ 1) (48000 1)\n");
        } else {
            text.push_str(&format!("wire (TS {lane}) (TJ {lane}) (48000 1)\n"));
        }
    }
    text.push_str("wire TJ Estimator (48000 12)\nwire Estimator End (48000 12)\nparam alignment 1 in\nwireGraphParam alignment (Align 0 0 0)\nwireProperty (Estimator 6630 0 0) (LSM 6630 0 0) true\nparam measured 0 out\nwireGraphParam measured (Estimator 1073741824 global 0) false\nparam constant 1 const\n");
    text
}
fn graph() -> dspg::Graph {
    dspg::parse(&graph_text(), &BTreeMap::new(), "synthetic").unwrap()
}
#[test]
fn compiled_companion_records_source_pairing_and_one_frame_alignment() {
    let g = graph();
    let c = lsid::load(&g, &strip(), "LSM", 6, &mut Diagnostics::default()).unwrap().unwrap();
    assert_eq!(c.source_port, "sense");
    assert_eq!(c.source_lanes, (0..12).collect::<Vec<_>>());
    assert_eq!(c.delay_frames, [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert!(!c.adaptation_enabled);
    assert!(c.preset.is_some());
    let lanes = analysis::lanes_into(&g, &Endpoint { node: "LSM".into(), port: 0 }).unwrap();
    assert_eq!(lanes.iter().map(|l| l.origin.channel).collect::<Vec<_>>(), [0, 1, 0, 1, 0, 1]);
    // Trace the actual duplication lane at the permutation join, not a
    // standalone constant: each physical channel must enter its intended slot.
    let duplicate_lanes: Vec<_> =
        lanes.iter().map(|l| l.hops.iter().find(|h| h.join == "D").unwrap().channel).collect();
    assert_eq!(duplicate_lanes, [0, 3, 1, 4, 2, 5]);
    assert!(g.params.iter().find(|p| p.name == "constant").unwrap().constant);
    assert_eq!(g.param_links.iter().find(|l| l.name == "measured").unwrap().flag.as_deref(), Some("false"));
}
#[test]
fn malformed_603_geometry_and_graph_contract_refuse() {
    for case in 0..7 {
        let mut g = graph();
        let mut s = strip();
        match case {
            0 => {
                s.strips[0][0].preset.state.sections[0].params.pop();
            }
            1 => {
                s.strips[0][0].preset.state.sections[0].params[0].1 = f32::NAN;
            }
            2 => {
                g.params.iter_mut().find(|p| p.name == "alignment").unwrap().default = 2.0;
            }
            3 => {
                g.box_property_links.clear();
            }
            4 => {
                g.boxes.iter_mut().find(|b| b.name == "End").unwrap().kind = BoxKind::Builtin("dbgn".into());
            }
            5 => {
                g.wires.iter_mut().find(|w| w.src.node == "TS" && w.src.port == 1).unwrap().src.port = 2;
            }
            _ => {
                s.strips[0][0].unit.manufacturer = fcc("test");
            }
        }
        assert!(lsid::load(&g, &s, "LSM", 6, &mut Diagnostics::default()).is_err(), "case {case}");
    }
}
fn control_strip() -> AuStrip {
    AuStrip {
        strips: vec![vec![effect(
            "Control",
            "cntl",
            "dbct",
            "abox",
            ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: vec![(0, 0.0), (1, -9.0)] }] },
        )]],
    }
}
#[test]
fn dbcontrol_orphan_proof_includes_every_connection_kind() {
    let base = "box Control dbcontrol 0 0\nbox Other parameter_add 0 0\n";
    let parse = |tail: &str| dspg::parse(&format!("{base}{tail}"), &BTreeMap::new(), "synthetic").unwrap();
    let graph = parse("");
    let strip = control_strip();
    assert_eq!(lsid::eliminate_dbcontrol(&graph, &strip, None).unwrap(), ["Control"]);
    for tail in [
        "param p 0\nwireGraphParam p (Control 0)\n",
        "property p in\nwireGraphProperty p (Control 0)\n",
        "wireParam (Control 0) (Other 0)\n",
        "wireParam (Other 0) (Control 0)\n",
        "wireProperty (Control 0) (Other 0)\n",
        "wireProperty (Other 0) (Control 0)\n",
    ] {
        assert!(lsid::eliminate_dbcontrol(&parse(tail), &strip, None).is_err());
    }
    let props = PropStrip {
        boxes: vec![PropBox {
            name: "Control".into(),
            properties: vec![Property { number: 1, scope: 0, element: 0, value: PropValue::UInt32(0) }],
        }],
    };
    assert!(lsid::eliminate_dbcontrol(&graph, &strip, Some(&props)).is_err());
    let mut ports = graph.clone();
    ports.boxes[0].inputs = 1;
    assert!(lsid::eliminate_dbcontrol(&ports, &strip, None).is_err());
    assert!(
        dspg::parse(&format!("{base}in audio\nwire audio Control (48000 1)\n"), &BTreeMap::new(), "synthetic").is_err()
    );
}

#[test]
fn scoped_property_id_spelling_retains_scope_and_element() {
    use aurora_speaker_tuning::plist::{binary, Value};
    fn dict(entries: Vec<(&str, Value)>) -> Value {
        let mut d = Dict::new();
        for (k, v) in entries {
            d.insert_unique(k.into(), v);
        }
        Value::Dict(d)
    }
    let input = dict(vec![(
        "Boxes",
        Value::Array(vec![dict(vec![
            ("Name", Value::String("Align".into())),
            (
                "Properties",
                Value::Array(vec![dict(vec![
                    ("PropertyID", Value::Real(100.0)),
                    ("Scope", Value::Integer(2)),
                    ("Element", Value::Integer(3)),
                    ("UInt32", Value::Integer(1)),
                ])]),
            ),
        ])]),
    )]);
    let parsed = PropStrip::parse(&binary::write(&input), "synthetic").unwrap();
    let p = &parsed.boxes[0].properties[0];
    assert_eq!((p.number, p.scope, p.element), (100, 2, 3));
    assert!(parsed.boxes[0].property(100).is_none());
}

/// A synthetic six-lane LSM2 + LSID bundle (generated values only), its
/// flow dictionary and the tuning loaded from it. `representable` fills a
/// model the engine can run; otherwise every record but the geometry is 0.
fn six_lane_tuning(
    name: &str,
    representable: bool,
    tweak: impl Fn(&mut BTreeMap<u32, f32>),
) -> (aurora_speaker_tuning::tuning::Tuning, aurora_speaker_tuning::plist::Value) {
    use aurora_speaker_tuning::{
        bundle::Bundle,
        plist::{binary, Value},
        tuning::{Options, Tuning},
    };
    use common::{dict, effect as effect_value, i, preset, s, AID};
    use sgv2_core::lsm2;
    let mut values: BTreeMap<_, _> = lsm2::companion::schema_ids().into_iter().map(|id| (id, 0.0)).collect();
    if representable {
        common::lsm2_model(&mut values);
        // the six-lane banks' pilot blocks, which the peak axis reserves
        for bank in 1..3 {
            values.insert(lsm2::ids::PILOT[bank] + lsm2::ids::P_AMPLITUDE_DB, -40.0);
        }
    }
    values.insert(lsm2::ids::controller(9), 2.0);
    values.insert(lsm2::ids::controller(lsm2::ids::C_LINK_SLOW), 1.0);
    values.insert(lsm2::ids::controller(lsm2::ids::C_ENABLE_MITIGATION), 1.0);
    values.insert(lsm2::ids::controller(lsm2::ids::C_CONTROL_TEMPERATURE), 1.0);
    for ty in 0..3 {
        values.insert(lsm2::ids::thermal(ty, lsm2::ids::T_ENABLE), 1.0);
    }
    for ch in 0..6u32 {
        values.insert(lsm2::ids::channel(ch, lsm2::ids::CH_VOLTS_FS), 12.0);
        values.insert(lsm2::ids::channel(ch, 30), sgv2_core::lsid::SYSTEM_ASSIGNMENT[ch as usize] as f32);
        values.insert(lsm2::ids::CH_TUNING + ch, sgv2_core::lsid::BANK_ASSIGNMENT[ch as usize] as f32);
        values.insert(lsm2::ids::CH_SPEAKER_TYPE + ch, sgv2_core::lsid::FAMILY_ASSIGNMENT[ch as usize] as f32);
    }
    tweak(&mut values);
    let state =
        ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: values.into_iter().collect() }] };
    let lsid_state = strip().strips[0][0].preset.state.clone();
    let effects = Value::Array(vec![
        effect_value("LSM", "lsm2", preset("lsm2", "synthetic controller", &state)),
        effect_value("Estimator", "lsid", preset("lsid", "synthetic estimator", &lsid_state)),
    ]);
    let mut files = common::spp3_files(6);
    files.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph_text().into_bytes());
    files.insert(
        format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"),
        binary::write(&dict(vec![("strips", Value::Array(vec![dict(vec![("effects", effects)])]))])),
    );
    let flow_name = "synthetic-flow.plist";
    let flow = dict(vec![(
        "ChannelData",
        Value::Array(vec![dict(vec![("SyntheticSideMarker", i(11))]), dict(vec![("SyntheticSideMarker", i(22))])]),
    )]);
    files.insert(format!("{AID}/DSP/Strips/{flow_name}"), binary::write(&flow));
    let properties = Value::Array(vec![
        dict(vec![("Number", i(64004)), ("UInt32", i(1))]),
        dict(vec![
            ("Number", i(6632)),
            ("Path", s(&format!("/System/Library/Audio/Tunings/{AID}/DSP/Strips/{flow_name}"))),
        ]),
    ]);
    files.insert(
        format!("{AID}/DSP/Strips/builtin_speaker_out_general.propstrip"),
        binary::write(&dict(vec![(
            "Boxes",
            Value::Array(vec![dict(vec![("Name", s("LSM")), ("Properties", properties)])]),
        )])),
    );
    let dir = common::scratch(name);
    common::write_dir(&dir, &files, &common::manifest_json(&files, "25G83"));
    let bundle = Bundle::open(&dir).unwrap();
    let mut diagnostics = Diagnostics::default();
    let tuning = Tuning::load(&bundle, &Options::default(), &mut diagnostics).unwrap();
    assert!(!diagnostics.has_errors(), "{:?}", diagnostics.items);
    (tuning, flow)
}

#[test]
fn complete_synthetic_six_channel_bundle_compiles_in_both_modes() {
    use aurora_speaker_tuning::{compile, machine::Machine, plist::binary};
    use sgv2_core::{lsm2, Io, ProtectionUnit};
    // an appended field with no qualified law keeps the companion muted
    let (tuning, flow) = six_lane_tuning("six-lsid-end-to-end", false, |v| {
        v.insert(lsm2::ids::controller(lsm2::companion::extension::CONTROLLER_ADAPTIVE_RELEASE_ENABLE), 1.0);
    });
    let mut diagnostics = Diagnostics::default();
    let machine = Machine::new("j473").unwrap();
    for voicing in [false, true] {
        let compiled = if voicing {
            compile::compile_voicing_with(&tuning, &machine, compile::Provenance::Verified, &mut diagnostics)
        } else {
            compile::compile(&tuning, &machine, &mut diagnostics)
        }
        .unwrap();
        assert_eq!((compiled.channels, compiled.sense_channels), (6, 12));
        assert_eq!(sgv2_core::blob::voicing(&compiled.blob).unwrap().is_some(), voicing);
        let mut unit = lsm2::Unit::from_blob(&compiled.blob).unwrap();
        let companion = unit.companion().unwrap();
        assert_eq!(companion.property_64004, 1);
        assert_eq!(binary::parse(&companion.flow).unwrap(), flow);
        assert_eq!(companion.flow_roles, lsm2::companion::Companion::six_flow_roles());
        let mut storage = [[1.0; 32]; 6];
        let mut out: Vec<_> = storage.iter_mut().map(|s| s.as_mut_slice()).collect();
        assert!(unit
            .process(Io { frames: 32, input: &[&[0.5; 32][..]; 6], sense: &[], output: &mut out })
            .is_failsafe());
        assert!(storage.iter().flatten().all(|x| *x == 0.0));
    }
    // A schema-complete preset with invalid seed/acquisition values must not
    // become loadable just because its companion already has a mute gate.
    for bank in 0..3 {
        for (offset, value) in [(2048, 0.0), (2049, -1.0), (2054, 0.0), (2054, 32.5), (3075, 0.0)] {
            let mut invalid = tuning.clone();
            let records = &mut invalid.lsid.as_mut().unwrap().state.sections[0].params;
            records.iter_mut().find(|r| r.0 == offset + bank * 7168).unwrap().1 = value;
            for voicing in [false, true] {
                let result = if voicing {
                    compile::compile_voicing_with(&invalid, &machine, compile::Provenance::Verified, &mut diagnostics)
                } else {
                    compile::compile(&invalid, &machine, &mut diagnostics)
                };
                assert!(result.is_err(), "bank {bank}, record {offset}, value {value}, voicing {voicing}");
            }
        }
    }
    // A rejected voicing effect may produce a quarantined bundle only when
    // the already compiled governor proves it has the mandatory mute gate.
    let mut unqualified = tuning.clone();
    let mut stage = unqualified.chain.iter().find(|s| s.name == "Estimator").unwrap().clone();
    stage.name = "UnknownVoice".into();
    stage.kind = "aufx/test/appl".into();
    stage.unit.as_mut().unwrap().subtype = fcc("test");
    stage.channels = Some(2);
    stage.before_protection = true;
    unqualified.chain.insert(0, stage);
    unqualified.protection.position += 1;
    for lane in &mut unqualified.protection.lanes {
        lane.through.insert(0, analysis::Pass { node: "UnknownVoice".into(), channel: lane.origin.channel });
    }
    for voicing in [false, true] {
        let compiled = if voicing {
            compile::compile_voicing_with(&unqualified, &machine, compile::Provenance::Verified, &mut diagnostics)
        } else {
            compile::compile(&unqualified, &machine, &mut diagnostics)
        }
        .unwrap();
        assert!(sgv2_core::blob::voicing(&compiled.blob).unwrap().is_none());
        let unit = lsm2::Unit::from_blob(&compiled.blob).unwrap();
        assert!(unit.is_failsafe());
        assert!(unit.companion().unwrap().gates & lsm2::companion::UNQUALIFIED_VOICING != 0);
    }
    assert!(diagnostics.items.iter().any(|d| d.code == "lsid-six-lane-unqualified"));
    assert!(diagnostics.items.iter().any(|d| d.code == "lsm2-release-gated"));
}

fn compile_six(t: &aurora_speaker_tuning::tuning::Tuning) -> (sgv2_core::lsm2::Unit, Diagnostics) {
    let mut d = Diagnostics::default();
    let machine = aurora_speaker_tuning::machine::Machine::new("j473").unwrap();
    let c = aurora_speaker_tuning::compile::compile(t, &machine, &mut d).unwrap_or_else(|e| panic!("{e}"));
    (sgv2_core::lsm2::Unit::from_blob(&c.blob).unwrap(), d)
}

#[test]
fn six_lane_companion_with_inert_appended_fields_plays_on_its_static_model() {
    use lsm2::companion::extension as x;
    use sgv2_core::{lsm2, Io, ProtectionUnit};
    let (tuning, flow) = six_lane_tuning("six-lsid-static", true, |_| {});
    let (mut unit, d) = compile_six(&tuning);
    assert!(d.items.iter().any(|i| i.code == "lsid-static-release"), "{:?}", d.items);
    assert!(!d.items.iter().any(|i| i.code == "lsm2-release-gated"));
    let companion = unit.companion().unwrap().clone();
    assert_eq!(companion.gates, 0);
    assert!(companion.is_static());
    assert!(companion.topology.is_none(), "the estimator's topology is dropped");
    assert!(unit.estimator().is_none(), "no estimator is built");
    assert_eq!(aurora_speaker_tuning::plist::binary::parse(&companion.flow).unwrap(), flow);
    assert_eq!(unit.params().model_only, 1);
    let frames = unit.params().latency_frames() + 4096;
    let input: Vec<f32> = (0..frames).map(|i| 0.1 * (i as f32 * 0.05).sin()).collect();
    let mut storage = vec![vec![0.0f32; frames]; 6];
    let mut output: Vec<_> = storage.iter_mut().map(|s| s.as_mut_slice()).collect();
    let h = unit.process(Io { input: &[&input[..]; 6], output: &mut output, sense: &[], frames });
    assert!(!h.is_failsafe(), "{h:?}");
    assert!(storage.iter().all(|lane| lane.iter().any(|&v| v != 0.0)), "every lane plays");

    // The displacement and velocity release times belong to adaptive release,
    // which is off: they change nothing, whichever axes are controlled. The
    // temperature response-rate factor is run by the temperature candidate.
    let (tuning, _) = six_lane_tuning("six-lsid-static-release-times", true, |v| {
        v.insert(lsm2::ids::controller(x::CONTROLLER_DISPLACEMENT_RELEASE), 50.0);
        v.insert(lsm2::ids::controller(x::CONTROLLER_VELOCITY_RELEASE), 500.0);
        v.insert(lsm2::ids::controller(x::CONTROLLER_TEMPERATURE_RATE_FACTOR), 0.125);
    });
    let (unit, _) = compile_six(&tuning);
    assert_eq!(unit.companion().unwrap().gates, 0);
    assert_eq!(unit.params().temperature_rate_factor, 0.125);
    // Parameters of a disabled mechanism and the estimator's own fields are
    // carried, not interpreted.
    let (tuning, _) = six_lane_tuning("six-lsid-static-inert", true, |v| {
        for ty in 0..3 {
            v.insert(lsm2::ids::model(1, ty, x::M_SENSITIVITY_MAX_GAIN_DB), 3.0);
            v.insert(lsm2::ids::model(1, ty, x::M_MIMO_TARGET_Q), 0.7);
            v.insert(lsm2::ids::thermal(ty, x::T_NUM_PARALLEL_SPEAKERS), 1.0);
            v.insert(lsm2::ids::thermal(ty, x::T_RDC_TOLERANCE_PERCENT), 10.0);
        }
        v.insert(x::GENERAL_SYS_ID_FFT_SIZE, 2048.0);
        for c in 0..8 {
            v.insert(24576 + c, -1.0);
        }
        v.insert(25601, -100.0);
    });
    assert_eq!(compile_six(&tuning).0.companion().unwrap().gates, 0);
}

#[test]
fn six_lane_appended_mechanisms_without_a_law_keep_the_companion_muted() {
    use sgv2_core::lsm2::{companion::extension as x, ids};
    let cases: Vec<(&str, Vec<(u32, f32)>)> = vec![
        ("sensitivity control", vec![(ids::model(2, 1, x::M_SENSITIVITY_ENABLE), 1.0)]),
        ("MIMO compensation", vec![(ids::model(1, 0, x::M_MIMO_COMPENSATION_ENABLE), 1.0)]),
        ("high-impedance watermark", vec![(26624, 1.0)]),
        ("adaptive release", vec![(ids::controller(x::CONTROLLER_ADAPTIVE_RELEASE_ENABLE), 1.0)]),
        ("power groups", vec![(ids::controller(x::CONTROLLER_NUM_POWER_GROUPS), 2.0)]),
        ("power groups", vec![(ids::channel(5, x::CHANNEL_POWER_GROUP_NUMBER), 1.0)]),
        ("reserved pilot-tone channel selector", vec![(x::PILOT_TUNING_RESERVED, 1.0)]),
        ("parallel speakers per thermal type", vec![(ids::thermal(1, x::T_NUM_PARALLEL_SPEAKERS), 2.0)]),
        ("per-system power limits", vec![(1068032 + 4, 10.0)]),
    ];
    for (i, (why, records)) in cases.into_iter().enumerate() {
        let (tuning, _) = six_lane_tuning(&format!("six-lsid-refuse-{i}"), true, |v| {
            for &(id, value) in &records {
                v.insert(id, value);
            }
        });
        let (unit, d) = compile_six(&tuning);
        assert!(unit.is_failsafe(), "{why}");
        assert_ne!(unit.companion().unwrap().gates, 0, "{why}");
        assert!(unit.companion().unwrap().topology.is_some(), "{why}: a gated companion keeps its topology");
        let note = d.items.iter().find(|i| i.code == "lsid-six-lane-unqualified").expect(why);
        assert!(note.message.contains(why), "{why}: {}", note.message);
        assert!(d.items.iter().any(|i| i.code == "lsm2-release-gated"), "{why}");
    }
    // The engine refuses a zero-gate six-lane companion with an enabled
    // appended mechanism, whatever wrote it.
    let (tuning, _) = six_lane_tuning("six-lsid-engine-refuses", true, |_| {});
    let (unit, _) = compile_six(&tuning);
    let mut forged = unit.companion().unwrap().clone();
    forged.lsm2_records.iter_mut().find(|r| r.0 == ids::controller(x::CONTROLLER_ADAPTIVE_RELEASE_ENABLE)).unwrap().1 =
        1.0;
    assert!(!forged.is_static());
    assert!(forged.encode().is_err());
}
