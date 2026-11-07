// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Exercise the isolated renderer's compiler with made-up tuning values.
mod common;
#[allow(dead_code)]
#[path = "../examples/voicing-box-render.rs"]
mod renderer;

use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::plist::{binary, Value};
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::{voicing, Diagnostics};
use common::*;
use sgv2_core::voicing::chain::{Chain, GraphParam, Kind};

fn fixture(tag: &str) -> Tuning {
    let mut files = spp3_files(4);
    let graph = graph_text("spp3").replace("aufx vrq2", "aufx tmst") + "param mvsc 1\nwireGraphParam mvsc (Eq 3 0 0)\n";
    files.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph.into_bytes());
    let mut p: std::collections::BTreeMap<u32, f32> = (0..616).map(|id| (id, 0.0)).collect();
    // Gain plus a peak: both gain and frequency move with parameter 3.
    for (id, value) in [
        (5, 10.0),
        (14, 3.0),
        (15, 9.0),
        (16, -8.0),
        (21, 3.0),
        (24, 3.0),
        (25, 1.0),
        (26, -4.0),
        (27, 600.0),
        (28, 1.0),
        (29, 2.0),
        (31, 3.0),
        (516, 0.0),
        (517, 1.0),
        (518, 0.0),
        (519, 1.0),
        (520, 6.0),
        (526, 1.0),
        (527, 1.0),
        (528, 0.0),
        (529, 1.0),
        (530, 4.0),
        (531, 1200.0),
    ] {
        p.insert(id, value);
    }
    let state = ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: p.into_iter().collect() }] };
    files.insert(
        format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"),
        binary::write(&dict(vec![(
            "strips",
            Value::Array(vec![dict(vec![(
                "effects",
                Value::Array(vec![
                    effect("Eq", "tmst", preset("tmst", "synthetic automated tone", &state)),
                    effect("Prot", "spp3", preset("spp3", "synthetic protection", &spp3_state(4))),
                ]),
            )])]),
        )])),
    );
    let dir = scratch(tag);
    write_dir(&dir, &files, &manifest_json(&files, "25G83"));
    Tuning::load(&Bundle::open(&dir).unwrap(), &Options::default(), &mut Diagnostics::default()).unwrap()
}

#[test]
fn explicit_tone_volume_matches_production_at_zero_middle_and_full_volume() {
    let t = fixture("isolated-tone-wired");
    let stage = t.chain.iter().find(|s| s.name == "Eq").unwrap();
    let mut d = Diagnostics::default();
    let compiled = voicing::chain::compile(&t, 48_000.0, &mut d).unwrap();
    assert!(compiled.stubs.is_empty() && compiled.gaps.is_empty());
    let eq = compiled.plan.node("Eq").unwrap();
    let input: Vec<f32> = (0..1025).map(|i| ((i as f64 * 0.13).sin() * 0.25) as f32).collect();
    let mut levels = Vec::new();
    for volume in [0.0, 0.5, 1.0] {
        let spec = renderer::tone_spec(stage, 2, 48_000.0, Some(volume), &mut d).unwrap();
        assert_eq!(spec.nodes[0].kind, compiled.spec.nodes[eq].kind);
        let mut isolated = Chain::new(spec, 48_000.0, 512).unwrap();
        let mut production = Chain::new(compiled.spec.clone(), 48_000.0, 512).unwrap();
        isolated.set_param(GraphParam::VolumeScalar, volume);
        production.set_param(GraphParam::VolumeScalar, volume);
        let mut energy = 0.0f64;
        for chunk in input.chunks(512) {
            let n = chunk.len();
            let mut expected = vec![vec![0.0; n]; 4];
            let mut actual = vec![vec![0.0; n]; 2];
            production.process(&[chunk, chunk], &mut expected.iter_mut().map(Vec::as_mut_slice).collect::<Vec<_>>(), n);
            isolated.process(&[chunk, chunk], &mut actual.iter_mut().map(Vec::as_mut_slice).collect::<Vec<_>>(), n);
            assert_eq!(actual, expected[..2]);
            energy += actual[0].iter().map(|x| f64::from(*x).powi(2)).sum::<f64>();
        }
        levels.push(energy);
    }
    assert!(levels[0] < levels[1] && levels[1] < levels[2], "automation must change the render: {levels:?}");
    // An absent override keeps the unwired rig's least-passing carry.
    let spec = renderer::tone_spec(stage, 2, 48_000.0, None, &mut d).unwrap();
    assert!(spec.links.is_empty());
    let Kind::Tone(tone) = &spec.nodes[0].kind else { panic!("expected Tone") };
    assert!(tone.automation.is_empty());
    assert_ne!(spec.nodes[0].kind, compiled.spec.nodes[eq].kind);
}

#[test]
fn explicit_volume_refuses_partially_carried_automation() {
    let t = fixture("isolated-tone-partial");
    let mut stage = t.chain.iter().find(|s| s.name == "Eq").unwrap().clone();
    let params = &mut stage.state.as_mut().unwrap().sections[0].params;
    // Two moving entries on the same slot have no exact compiled law.
    for id in [526, 531] {
        params.iter_mut().find(|(i, _)| *i == id).unwrap().1 = 0.0;
    }
    assert!(renderer::tone_spec(&stage, 2, 48_000.0, Some(1.0), &mut Diagnostics::default()).is_err());
    assert!(renderer::tone_spec(&stage, 2, 48_000.0, None, &mut Diagnostics::default()).is_ok());
}
