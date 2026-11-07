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
