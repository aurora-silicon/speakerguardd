// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The voicing chain's plan and compile on the synthetic graph: stereo
//! program, a non-finite guard and an effect on it, duplicated to four
//! lanes and permuted to [L, R, L, R] before the unit.

mod common;

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::voicing::{chain, topology};
use aurora_speaker_tuning::Diagnostics;
use common::*;
use sgv2_core::voicing::chain::{Chain, Kind, Source};

fn tuning(tag: &str) -> Tuning {
    let files = spp3_files(4);
    // one directory per test: the tests run in parallel in one process
    let dir = scratch(&format!("voicing-chain-{tag}"));
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    let b = Bundle::open(&dir).unwrap();
    let mut d = Diagnostics::default();
    Tuning::load(&b, &Options::default(), &mut d).unwrap()
}

#[test]
fn plan_traces_the_fanout() {
    let t = tuning("t1");
    let p = topology::plan(&t).unwrap();
    assert_eq!(p.program_channels, 2);
    let names: Vec<&str> = p.nodes.iter().map(|n| n.name.as_str()).collect();
    assert_eq!(names, ["Guard", "Eq"]);
    assert!(p.nodes.iter().all(|n| n.role == topology::Role::Program));
    assert_eq!(p.nodes[0].kind, topology::NodeKind::NonFinite);
    assert_eq!(p.nodes[1].kind, topology::NodeKind::Unit { subtype: "vrq2".into() });
    let e = |c| topology::Source::Node { node: 1, channel: c };
    assert_eq!(p.outputs, [e(0), e(1), e(0), e(1)]);
    assert_eq!(p.fanout_at(), 2);
}

#[test]
fn compile_stubs_unknown_boxes_and_runs() {
    let t = tuning("t2");
    let mut d = Diagnostics::default();
    let c = chain::compile(&t, 48_000.0, &mut d).unwrap();
    assert_eq!(c.stubs, [("Eq".to_string(), "vrq2".to_string())]);
    assert!(matches!(c.spec.nodes[1].kind, Kind::Stub(_)));
    assert_eq!(c.spec.outputs[2], Source::Node(1, 0));
    let mut ch = Chain::new(c.spec, 48_000.0, 16).unwrap();
    let (l, r) = ([0.25f32; 16], [-0.5f32; 16]);
    let mut o = [[0f32; 16]; 4];
    let mut it = o.iter_mut();
    let mut lanes: [&mut [f32]; 4] = std::array::from_fn(|_| &mut it.next().unwrap()[..]);
    ch.process(&[&l, &r], &mut lanes, 16);
    assert_eq!([lanes[0][3], lanes[1][3], lanes[2][3], lanes[3][3]], [0.25, -0.5, 0.25, -0.5]);
}

/// The stereo program summed to one channel (a one-speaker graph) and
/// duplicated to the four lanes.
fn summed_tuning(tag: &str) -> Tuning {
    let mut files = spp3_files(4);
    let key = files.keys().find(|k| k.ends_with(".dspg")).unwrap().clone();
    let g = String::from_utf8(files[&key].clone()).unwrap();
    let g = g
        .replace("box Spl split 1 2\n", "box Spl split 1 2\nbox Mix sum 2 1\n")
        .replace(
            "wire Guard Eq ([fs] [chIn])\nwire Eq Spl ([fs] [chIn])\n",
            "wire Guard Spl ([fs] [chIn])\nwire (Spl 0) (Mix 0) ([fs] 1)\nwire (Spl 1) (Mix 1) ([fs] 1)\nwire Mix Eq ([fs] 1)\n",
        )
        .replace("wire (Spl 0) (Up 0) ([fs] 1)\n", "wire Eq (Up 0) ([fs] 1)\n")
        .replace("wire (Spl 0) (Up 1) ([fs] 1)\n", "wire Eq (Up 1) ([fs] 1)\n")
        .replace("wire (Spl 1) (Up 2) ([fs] 1)\n", "wire Eq (Up 2) ([fs] 1)\n")
        .replace("wire (Spl 1) (Up 3) ([fs] 1)\n", "wire Eq (Up 3) ([fs] 1)\n");
    assert!(g.contains("wire Mix Eq") && g.contains("wire Eq (Up 3)"));
    files.insert(key, g.into_bytes());
    let dir = scratch(&format!("voicing-chain-{tag}"));
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    let b = Bundle::open(&dir).unwrap();
    let mut d = Diagnostics::default();
    Tuning::load(&b, &Options::default(), &mut d).unwrap()
}

#[test]
fn plan_carries_a_sum_of_both_program_channels() {
    let t = summed_tuning("sum");
    let p = topology::plan(&t).unwrap();
    let names: Vec<&str> = p.nodes.iter().map(|n| n.name.as_str()).collect();
    assert_eq!(names, ["Guard", "Mix", "Eq"]);
    assert_eq!(p.nodes[1].kind, topology::NodeKind::Sum { inputs: 2 });
    let guard = |c| topology::Source::Node { node: 0, channel: c };
    assert_eq!(p.nodes[0].inputs, [topology::Source::Program { channel: 0 }, topology::Source::Program { channel: 1 }]);
    assert_eq!(p.nodes[1].inputs, [guard(0), guard(1)]);
    assert_eq!(p.nodes[1].out_width(), 1);
    assert_eq!(p.nodes[2].inputs, [topology::Source::Node { node: 1, channel: 0 }]);
    assert_eq!(p.outputs, [topology::Source::Node { node: 2, channel: 0 }; 4]);

    let mut d = Diagnostics::default();
    let c = chain::compile(&t, 48_000.0, &mut d).unwrap();
    assert_eq!(c.spec.nodes[1].kind, Kind::Sum(1));
    let mut run = Chain::new(c.spec, 48_000.0, 16).unwrap();
    let l = [0.25f32; 16];
    let r = [0.5f32; 16];
    let mut o = [[0.0f32; 16]; 4];
    let mut it = o.iter_mut();
    let mut lanes: [&mut [f32]; 4] = core::array::from_fn(|_| &mut it.next().unwrap()[..]);
    run.process(&[&l, &r], &mut lanes, 16);
    assert!(lanes.iter().all(|lane| lane[9] == 0.75));
}

#[test]
fn zero_port_control_sum_selects_tone_groups_at_runtime() {
    use aurora_speaker_tuning::{
        aupreset::{ParamSection, ParamState},
        dspg,
        fourcc::FourCc,
    };
    let mut t = tuning("control-sum");
    let control_graph = dspg::parse(
        "box Selector parameter_add 0 0\nbox Eq (aufx tmst) 1 1\nparam one 1 const\nparam lids 1\nwireGraphParam one (Selector 0 input 0)\nwireGraphParam lids (Selector 0 input 1)\nwireParam (Selector 0 output 0) (Eq 1)\n",
        &Default::default(), "synthetic controls").unwrap();
    t.graph.boxes.retain(|b| b.name != "Eq");
    t.graph.boxes.extend(control_graph.boxes);
    t.graph.params.extend(control_graph.params);
    t.graph.param_links.extend(control_graph.param_links);
    t.graph.box_param_links.extend(control_graph.box_param_links);
    let stage = t.chain.iter_mut().find(|s| s.name == "Eq").unwrap();
    stage.unit.as_mut().unwrap().subtype = FourCc::from_bytes(*b"tmst");
    let mut params: std::collections::BTreeMap<u32, f32> = (0..514).map(|id| (id, 0.0)).collect();
    params.extend([(5, 12.0), (14, 3.0), (15, 9.0), (16, -6.0), (17, 500.0), (18, 1.0), (19, 2.0), (21, 3.0)]);
    stage.state = Some(ParamState {
        sections: vec![ParamSection { scope: 0, element: 0, params: params.into_iter().collect() }],
    });
    let c = chain::compile(&t, 48_000.0, &mut Diagnostics::default()).unwrap();
    assert!(c.stubs.is_empty() && c.unwired.is_empty(), "{:?}", c.gaps);
    assert_eq!(c.plan.nodes[0].name, "Selector");
    let base = aurora_speaker_tuning::compile::compile(
        &t,
        &aurora_speaker_tuning::machine::Machine::with_unit("j413", &test_unit()).unwrap(),
        &mut Diagnostics::default(),
    )
    .unwrap();
    let bytes = sgv2_core::voicing::artifact::encode(&c.spec, &base.blob).unwrap();
    assert_eq!(sgv2_core::voicing::artifact::decode(&bytes, &base.blob).unwrap(), c.spec);
    let mut run = Chain::new(c.spec, 48_000.0, 16).unwrap();
    let input = [1.0; 16];
    let mut output = [[0.0; 16]; 4];
    for (lid, expected) in [(1.0, 10f32.powf(-6.0 / 20.0)), (0.0, 1.0), (1.0, 10f32.powf(-6.0 / 20.0))] {
        run.set_param(sgv2_core::voicing::chain::GraphParam::Lid, lid);
        let mut refs: Vec<&mut [f32]> = output.iter_mut().map(|c| &mut c[..]).collect();
        run.process(&[&input, &input], &mut refs, 16);
        assert!(output.iter().flatten().all(|v| (*v - expected).abs() < 1e-6));
    }
    // A control input with no supported runtime source is not silently lost.
    t.graph.params.iter_mut().find(|p| p.name == "one").unwrap().constant = false;
    assert!(chain::compile(&t, 48_000.0, &mut Diagnostics::default()).is_err());
}

/// A level-dependent notch (tmst slot type 15) compiles as a carried box,
/// not a compiler gap, and acts on level: a loud tone at its frequency is
/// cut by its ratio law up to its range, a quiet one passes, and a loud tone well away
/// from it passes. Synthetic values only.
#[test]
fn level_dependent_notch_is_carried_and_follows_level() {
    use aurora_speaker_tuning::{
        aupreset::{ParamSection, ParamState},
        dspg,
        fourcc::FourCc,
    };
    let mut t = tuning("notch");
    let g = dspg::parse("box Eq (aufx tmst) 1 1\n", &Default::default(), "synthetic notch").unwrap();
    let at = t.graph.boxes.iter().position(|b| b.name == "Eq").unwrap();
    t.graph.boxes[at] = g.boxes.into_iter().next().unwrap();
    let stage = t.chain.iter_mut().find(|s| s.name == "Eq").unwrap();
    stage.unit.as_mut().unwrap().subtype = FourCc::from_bytes(*b"tmst");
    let mut params: std::collections::BTreeMap<u32, f32> = (0..640).map(|id| (id, 0.0)).collect();
    // global: shelf order 4, algorithm version 1; slot 0: running, type 15,
    // range -12 dB at 1 kHz, quality 2, ratio 4, threshold -40 dB, both
    // channels; first level-dependent slot's attack 1 ms, release 50 ms
    params.extend([(5, 4.0), (12, 1.0)]);
    params.extend([(14, 3.0), (15, 15.0), (16, -12.0), (17, 1000.0), (18, 2.0), (19, 4.0), (20, -40.0), (21, 3.0)]);
    params.extend([(616, 1.0), (617, 50.0)]);
    stage.state = Some(ParamState {
        sections: vec![ParamSection { scope: 0, element: 0, params: params.into_iter().collect() }],
    });
    let c = chain::compile(&t, 48_000.0, &mut Diagnostics::default()).unwrap();
    assert!(c.stubs.is_empty() && c.unwired.is_empty() && c.gaps.is_empty(), "{:?} {:?}", c.stubs, c.gaps);
    match &c.spec.nodes[1].kind {
        Kind::Tone(tone) => assert_eq!(tone.dynamic.len(), 1),
        k => panic!("Eq compiled as {k:?}"),
    }
    let level = |f: f64, db: f64| -> f64 {
        let mut run = Chain::new(c.spec.clone(), 48_000.0, 512).unwrap();
        let a = 10f64.powf(db / 20.0);
        let n = 48_000;
        let x: Vec<f32> =
            (0..n).map(|i| (a * (2.0 * std::f64::consts::PI * f * i as f64 / 48_000.0).sin()) as f32).collect();
        let mut out = vec![vec![0f32; n]; 4];
        for at in (0..n).step_by(512) {
            let m = 512.min(n - at);
            let mut refs: Vec<&mut [f32]> = out.iter_mut().map(|o| &mut o[at..at + m]).collect();
            run.process(&[&x[at..at + m], &x[at..at + m]], &mut refs, m);
        }
        let tail = &out[0][n / 2..];
        let rms = (tail.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / tail.len() as f64).sqrt();
        20.0 * (rms / (a / 2f64.sqrt())).log10()
    };
    let loud = level(1000.0, -6.0);
    let mid = level(1000.0, -30.0);
    let quiet = level(1000.0, -80.0);
    let away = level(100.0, -6.0);
    // 34 dB over the threshold: the cut reaches the range
    assert!((-12.5..=-11.0).contains(&loud), "loud tone at the notch: {loud} dB");
    // 10 dB over at ratio 4: about 7.5 dB of cut
    assert!((-8.5..=-6.5).contains(&mid), "tone 10 dB over the threshold: {mid} dB");
    assert!(quiet.abs() < 0.25, "quiet tone at the notch: {quiet} dB");
    assert!(away.abs() < 0.5, "loud tone an octave and more away: {away} dB");
    assert!(loud < quiet - 6.0);
}
