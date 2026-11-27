// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The microphone chain compiler on synthetic bundles. Every value here is
//! made up: the structure mirrors a tuning bundle's microphone files, the
//! numbers describe no real Mac.

mod common;

use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::machine::Machine;
use aurora_speaker_tuning::mic::{self, Status};
use aurora_speaker_tuning::plist::{binary, Value};
use aurora_speaker_tuning::Diagnostics;
use common::{dict, fcc, i, r, s, ACOUSTIC_ID, AID};
use sgv2_core::mic::{artifact, Engine, Stage};
use std::collections::BTreeMap;

const HOP: usize = 240;
const BINS: usize = HOP + 1;

fn state(params: &[(u32, f32)]) -> ParamState {
    ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: params.to_vec() }] }
}

fn unit_effect(name: &str, sub: &str, st: &ParamState, bypass: bool, builtin: Option<&str>) -> Value {
    let mut p = vec![
        ("data", Value::Data(st.encode())),
        ("manufacturer", i(fcc("appl"))),
        ("subtype", i(fcc(sub))),
        ("type", i(fcc("aufx"))),
        ("version", i(0)),
    ];
    if let Some(c) = builtin {
        p.push(("classname", s(c)));
    }
    dict(vec![
        ("aupreset", dict(p)),
        ("bypass", i(bypass as i128)),
        ("displayname", s(name)),
        ("unit", dict(vec![("manufacturer", i(fcc("appl"))), ("subtype", i(fcc(sub))), ("type", i(fcc("aufx")))])),
    ])
}

/// A filter box state: every global and slot present (zeros), one peak.
fn tone_state(gain_db: f32, f0: f32, q: f32) -> ParamState {
    let mut full: BTreeMap<u32, f32> = (0..14 + 10 * 50).map(|id| (id, 0.0)).collect();
    full.insert(5, 10.0);
    full.insert(12, 1.0);
    for (k, v) in [(0u32, 3.0f32), (1, 1.0), (2, gain_db), (3, f0), (4, q), (5, 2.0), (7, 1.0)] {
        full.insert(14 + k, v);
    }
    ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: full.into_iter().collect() }] }
}

fn be(v: &[f32]) -> Value {
    Value::Data(v.iter().flat_map(|x| x.to_be_bytes()).collect())
}

/// A beam file: one beam, `mics` microphones, real weights `w[m]` on
/// every bin.
fn beam_file(mics: usize, w: &[f32]) -> Vec<u8> {
    beam_file_n(mics, w, BINS)
}

fn beam_file_n(mics: usize, w: &[f32], bins: usize) -> Vec<u8> {
    let mut re = Vec::new();
    let mut im = Vec::new();
    for (m, wm) in w.iter().enumerate().take(mics) {
        re.push((format!("Mic{m}"), be(&vec![*wm; bins])));
        im.push((format!("Mic{m}"), be(&vec![0.0; bins])));
    }
    let as_dict = |v: Vec<(String, Value)>| {
        let mut d = aurora_speaker_tuning::plist::Dict::new();
        for (k, x) in v {
            d.insert_unique(k, x);
        }
        Value::Dict(d)
    };
    let beam0 = dict(vec![
        ("Azimuth", r(0.0)),
        ("Description", s("Beam0")),
        ("Distance", r(1.0)),
        ("EQSelection", be(&vec![1.0; bins])),
        ("Elevation", r(90.0)),
        ("ImagCoefficients", as_dict(im)),
        ("NonZeroMics", be(&vec![1.0; mics])),
        ("RealCoefficients", as_dict(re)),
    ]);
    binary::write(&dict(vec![
        ("Beam0", beam0),
        ("SampleRate", i(48_000)),
        ("nBeams", i(1)),
        ("nCoefficients", i(bins as i128)),
        ("nMicsUsed", i(mics as i128)),
    ]))
}

/// A suppressor profile file: `mics` profiles of `BINS` powers.
fn sens_file(mics: usize, power: f32, block: f64) -> Vec<u8> {
    let mut kv = vec![("BlockSize", r(block)), ("NumberOfChannels", i(mics as i128)), ("SampleRate", r(48_000.0))];
    let names: Vec<String> = (0..mics).map(|m| format!("NoiseProfile{m}")).collect();
    for n in &names {
        kv.push((n.as_str(), be(&vec![power; block as usize + 1])));
    }
    binary::write(&dict(kv))
}

/// A suppressor preset: every global parameter, made-up values.
fn sens_state(mode: f32) -> ParamState {
    state(&[
        (0, 0.0),
        (1, mode),
        (2, 0.0),
        (3, 8.0),
        (4, 4.0),
        (5, 0.0),
        (6, -10.0),
        (7, 1.2),
        (8, 0.0),
        (9, 1.0),
        (10, 0.3),
        (11, 8.0),
        (12, 4.0),
        (13, 0.0),
    ])
}

/// A beamformer preset in the fixed-beam mode (made-up blend controls).
fn beam_state(fixed: bool) -> ParamState {
    let sel = if fixed { 1.0 } else { 0.0 };
    state(&[
        (0, 0.0),
        (1, 1.0 - sel),
        (2, sel),
        (3, 0.0),
        (4, 0.0),
        (5, 0.0),
        (21, 0.0),
        (22, 0.0),
        (23, 0.0),
        (31, -10.0),
        (32, 900.0),
        (37, 150.0),
        (42, 0.0),
    ])
}

fn graph(extra_box: Option<&str>) -> String {
    let (decl, path) = match extra_box {
        Some(kind) => {
            (format!("box Odd {kind} 1 1\n"), "wire S Odd ([fs] 1 [blockSize])\nwire Odd Eq ([fs] 1 [blockSize])\n")
        }
        None => (String::new(), "wire S Eq ([fs] 1 [blockSize])\n"),
    };
    format!(
        r#"graphName "test_mic"
; synthetic microphone graph
[def blockSize {HOP}]
[def numChansIn 2]
in in
out out
box Pre gain 1 1
box Vol dbgn 1 1
box A fft 1 1
box S fft 1 1
box Rb reblocker 1 1
box Dc (aufx dccb appl) 1 1
box Beam (aufx bmfd appl) 2 4
box D1 dead 1 0
box D2 dead 1 0
box D3 dead 1 0
box Ns (aufx ocns appl) 1 1
box Sens (aufx sens appl) 1 1
box Eq (aufx tmst appl) 1 1
{decl}wire in Dc ([fs] [numChansIn])
wire Dc Rb ([fs] [numChansIn])
wire Rb Sens ([fs] [numChansIn] [blockSize])
wire Sens Pre ([fs] [numChansIn] [blockSize])
wire Pre A ([fs] [numChansIn] [blockSize])
wire A (Beam 0) (freq [fs] [numChansIn] [blockSize])
wire A (Beam 1) (freq [fs] [numChansIn] [blockSize])
wire (Beam 0) Ns (freq [fs] 1 [blockSize])
wire (Beam 1) D1 (freq [fs] 1 [blockSize])
wire (Beam 2) D2 (freq [fs] 1 [blockSize])
wire (Beam 3) D3 (freq [fs] 1 [blockSize])
wire Ns S (freq [fs] 1 [blockSize])
{path}wire Eq Vol ([fs] 1 [blockSize])
wire Vol out ([fs] 1 [blockSize])
param mvol 0
wireGraphParam mvol (Vol 0)
"#
    )
}

fn database() -> Vec<u8> {
    let hw = |g: &str| {
        Value::Array(vec![dict(vec![(
            "Hardware",
            Value::Array(vec![
                dict(vec![
                    ("Path", s(&format!("{AID}/DSP/Graphs/{g}.dspg"))),
                    ("Substitutions", dict(vec![("fs", s("48000"))])),
                    ("Type", s("DSPGraph")),
                ]),
                dict(vec![("Path", s(&format!("{AID}/DSP/Strips/{g}.austrip"))), ("Type", s("AUStrip"))]),
                dict(vec![("Path", s(&format!("{AID}/DSP/Strips/{g}.propstrip"))), ("Type", s("PropertyStrip"))]),
            ]),
        )])])
    };
    let conf = |kind: i128, g: &str| {
        dict(vec![
            (
                "DSPDeviceProperties",
                Value::Array(vec![dict(vec![("Selector", s("cdes")), ("Value", dict(vec![("kind", i(kind))]))])]),
            ),
            ("Input", hw(g)),
        ])
    };
    binary::write(&dict(vec![(
        AID,
        dict(vec![(
            "Built-in Microphone",
            dict(vec![
                ("48k - General", conf(0, "mic_general")),
                ("48k - Movie", conf(0, "mic_general")),
                ("48k - Voice", conf(1, "mic_voice")),
            ]),
        )]),
    )]))
}

struct Fixture {
    graph: String,
    effects: Vec<Value>,
    beam: Vec<u8>,
    sens: Vec<u8>,
    /// Extra property strip boxes and files.
    props: Vec<(String, i128, String)>,
    files: Vec<(String, Vec<u8>)>,
}

impl Fixture {
    fn new() -> Fixture {
        Fixture {
            graph: graph(None),
            effects: vec![
                unit_effect("Dc", "dccb", &state(&[(0, 25.0), (1, 0.0), (2, 0.0)]), false, None),
                unit_effect("Sens", "sens", &sens_state(6.0), false, None),
                unit_effect("Pre", "gain", &state(&[(0, 2.0), (1, 0.0)]), false, Some("gain")),
                unit_effect("Beam", "bmfd", &beam_state(true), false, None),
                unit_effect("Ns", "ocns", &state(&[(0, -6.0)]), true, None),
                unit_effect("Eq", "tmst", &tone_state(-3.0, 1000.0, 1.0), false, None),
                unit_effect("Vol", "dbgn", &state(&[(0, 0.0), (1, 0.0)]), false, Some("dbgain")),
            ],
            beam: beam_file(2, &[0.5, 0.5]),
            // no noise: the suppressor passes everything
            sens: sens_file(2, 0.0, HOP as f64),
            props: Vec::new(),
            files: Vec::new(),
        }
    }

    fn files(&self) -> BTreeMap<String, Vec<u8>> {
        let mut f = BTreeMap::new();
        f.insert("DeviceDSPDatabase.plist".to_string(), database());
        f.insert(format!("{AID}/DSP/Graphs/mic_general.dspg"), self.graph.clone().into_bytes());
        f.insert(
            format!("{AID}/DSP/Strips/mic_general.austrip"),
            binary::write(&dict(vec![(
                "strips",
                Value::Array(vec![dict(vec![("effects", Value::Array(self.effects.clone()))])]),
            )])),
        );
        let prop = |name: &str, number: i128, file: &str| {
            dict(vec![
                ("Name", s(name)),
                (
                    "Properties",
                    Value::Array(vec![dict(vec![
                        ("Element", i(0)),
                        ("Number", i(number)),
                        ("Path", s(&format!("/System/Library/Audio/Tunings/{AID}/DSP/Strips/{file}"))),
                        ("Scope", i(0)),
                    ])]),
                ),
            ])
        };
        f.insert(
            format!("{AID}/DSP/Strips/mic_general.propstrip"),
            binary::write(&dict(vec![(
                "Boxes",
                Value::Array(
                    [prop("Beam", 4630, "mic_beam.plist"), prop("Sens", 1000, "mic_sens.plist")]
                        .into_iter()
                        .chain(self.props.iter().map(|(b, n, f)| prop(b, *n, f)))
                        .collect(),
                ),
            )])),
        );
        f.insert(format!("{AID}/DSP/Strips/mic_beam.plist"), self.beam.clone());
        f.insert(format!("{AID}/DSP/Strips/mic_sens.plist"), self.sens.clone());
        for (n, d) in &self.files {
            f.insert(format!("{AID}/DSP/Strips/{n}"), d.clone());
        }
        f
    }

    /// Read in place (unverified), as a development tree.
    fn tree(&self, name: &str) -> Bundle {
        let root = common::scratch(name);
        for (p, d) in self.files() {
            let full = root.join(&p);
            std::fs::create_dir_all(full.parent().unwrap()).unwrap();
            std::fs::write(full, d).unwrap();
        }
        Bundle::from_tunings_root(&root, ACOUSTIC_ID).unwrap()
    }

    /// An installer bundle collected on `board` / `unit`.
    fn installed(&self, name: &str, board: &str, unit: &str) -> Bundle {
        let dir = common::scratch(name);
        let mut files = self.files();
        files.insert(format!("Config/{AID}.plist"), binary::write(&common::device_config(-40.0)));
        let m = common::manifest_json_ident(&files, "27A000", None, Some(board), Some(unit));
        common::write_dir(&dir, &files, &m);
        Bundle::from_dir(&dir).unwrap()
    }
}

fn partial() -> mic::Options {
    mic::Options { allow_partial: true, ..Default::default() }
}

#[test]
fn compiles_the_synthetic_chain_with_every_stage_reported() {
    let b = Fixture::new().tree("mic-compile");
    let mut d = Diagnostics::default();
    // every stage is carried: the default compile takes it
    let c = mic::compile(&b, &mic::Options::default(), &mut d).unwrap();
    assert!(c.report.refusals.is_empty(), "{:?}", c.report.refusals);
    let st: Vec<(&str, &Status)> = c.report.stages.iter().map(|s| (s.name.as_str(), &s.status)).collect();
    assert_eq!(
        st,
        vec![
            ("Dc", &Status::Approximate),
            ("Rb", &Status::Structural),
            ("Sens", &Status::Approximate),
            ("Pre", &Status::Exact),
            ("A", &Status::Exact),
            ("Beam", &Status::Exact),
            ("Ns", &Status::Bypassed),
            ("S", &Status::Exact),
            ("Eq", &Status::Exact),
            ("Vol", &Status::Exact),
        ]
    );
    assert_eq!(c.report.configuration.as_deref(), Some("48k - General"));
    assert!(!c.report.partial);
    assert!(!c.refused());
    let p = c.params.unwrap();
    assert_eq!((p.mics, p.stft.hop, p.stft.size, p.beam.bins), (2, HOP, 2 * HOP, BINS));
    assert!(!p.partial);
    assert_eq!(p.stft.window, sgv2_core::mic::Window::Quadrant);
    // the suppressor: the global parameters, a hop of latency
    let Stage::Sens(sp) = &p.pre[1] else { panic!("{:?}", p.pre[1]) };
    assert_eq!((sp.mics, sp.hop, sp.min_len, sp.aggressive), (2, HOP, 4, false));
    assert_eq!((sp.signal_ms, sp.gain_ms, sp.floor_db, sp.multiplier), (8.0, 4.0, -10.0, 1.2f32 as f64));
    assert_eq!(c.report.latency_frames, 2 * HOP - 1 + HOP);
    // the gain builtin's parameter 0 is a linear factor: 2.0 = +6.02 dB
    match p.pre[2] {
        Stage::Gain { db } => assert!((db - 20.0 * 2f64.log10()).abs() < 1e-9),
        ref o => panic!("{o:?}"),
    }
    // every approximate stage on the report carries what it still depends on
    assert!(c.report.stages.iter().find(|s| s.name == "Sens").unwrap().todo.iter().any(|t| t.contains("artifact")));
    assert!(c.report.stages.iter().filter(|s| s.status == Status::Exact).all(|s| s.todo.is_empty()));
}

#[test]
fn the_compiled_chain_renders_the_stated_laws() {
    let b = Fixture::new().tree("mic-render");
    let p = mic::compile(&b, &mic::Options::default(), &mut Diagnostics::default()).unwrap().params.unwrap();
    let mut e = Engine::new(p).unwrap();
    // the same 1 kHz tone on both microphones: the beam averages them,
    // the gain doubles, the peak cuts 3 dB at its centre, the DC blocker
    // passes it
    let n = 48_000;
    let x: Vec<f32> =
        (0..n).map(|k| 0.1 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / 48_000.0).sin() as f32).collect();
    let mut y = vec![0.0f32; n];
    for at in (0..n).step_by(128) {
        e.process(&[&x[at..at + 128], &x[at..at + 128]], &mut y[at..at + 128], 128);
    }
    // amplitude from the rms over whole periods (48 frames each)
    let tail = &y[24_000..24_000 + 48 * 400];
    let amp = (tail.iter().map(|v| f64::from(*v).powi(2)).sum::<f64>() / tail.len() as f64).sqrt() * 2f64.sqrt();
    let fs = 48_000.0;
    let eq = sgv2_core::voicing::tone::peak(-3.0, 1000.0, 1.0, fs).magnitude(1000.0, fs);
    let dc = sgv2_core::voicing::tone::butterworth(true, 2, 25.0, fs).sections[0].magnitude(1000.0, fs);
    let want = 0.1 * 2.0 * eq * dc;
    assert!((amp / want - 1.0).abs() < 1e-3, "amplitude {amp} want {want}");
    assert!((eq - 10f64.powf(-3.0 / 20.0)).abs() < 0.01);
}

#[test]
fn default_compile_refuses_uncharacterised_stages() {
    // the noise suppressor not bypassed: uncharacterised
    let mut f = Fixture::new();
    f.effects[4] = unit_effect("Ns", "ocns", &state(&[(0, -6.0)]), false, None);
    let b = f.tree("mic-refuse-default");
    let c = mic::compile(&b, &mic::Options::default(), &mut Diagnostics::default()).unwrap();
    assert!(c.refused());
    assert!(c.report.refusals.iter().any(|r| r.contains("Ns")));
    // a partial compile carries it as identity and flags the chain
    let c = mic::compile(&b, &partial(), &mut Diagnostics::default()).unwrap();
    assert!(!c.refused(), "{:?}", c.report.refusals);
    assert!(c.report.partial && c.params.unwrap().partial);
    let (blob, _) = mic::compile_blob(
        &b,
        &Machine::new("j999").unwrap(),
        &mic::Options::default(),
        true,
        &mut Diagnostics::default(),
    )
    .unwrap();
    assert!(blob.is_none());
}

#[test]
fn an_unknown_box_refuses_even_a_partial_compile() {
    let mut f = Fixture::new();
    f.graph = graph(Some("(aufx zzzz appl)"));
    let b = f.tree("mic-refuse-unknown");
    let c = mic::compile(&b, &partial(), &mut Diagnostics::default()).unwrap();
    assert!(c.refused());
    assert!(c.report.stages.iter().any(|s| s.name == "Odd" && s.status == Status::Refused));
}

#[test]
fn a_bypassed_unknown_box_is_honoured() {
    let mut f = Fixture::new();
    f.graph = graph(Some("(aufx zzzz appl)"));
    f.effects.push(unit_effect("Odd", "zzzz", &state(&[(0, 1.0)]), true, None));
    let b = f.tree("mic-bypassed");
    let c = mic::compile(&b, &partial(), &mut Diagnostics::default()).unwrap();
    assert!(!c.refused(), "{:?}", c.report.refusals);
    assert!(c.report.stages.iter().any(|s| s.name == "Odd" && s.status == Status::Bypassed));
}

#[test]
fn a_beam_file_for_other_microphones_refuses() {
    let mut f = Fixture::new();
    f.beam = beam_file(3, &[0.3, 0.3, 0.3]);
    let b = f.tree("mic-beam-mismatch");
    let c = mic::compile(&b, &partial(), &mut Diagnostics::default()).unwrap();
    assert!(c.refused());
    assert!(c.report.refusals.iter().any(|r| r.contains("microphones")));
}

#[test]
fn the_blob_is_bound_to_the_board_and_unit() {
    let f = Fixture::new();
    let unit = common::test_unit();
    let b = f.installed("mic-bind", "j999", &unit);
    let m = Machine::with_unit("j999", &unit).unwrap();
    let (blob, report) = mic::compile_blob(&b, &m, &partial(), false, &mut Diagnostics::default()).unwrap();
    let blob = blob.expect("a blob");
    let (h, p) = artifact::decode(&blob).unwrap();
    assert_eq!(h.binding.machine(), Some("j999"));
    assert_eq!(h.binding.acoustic_id, ACOUSTIC_ID);
    assert!(h.binding.unit_bound());
    assert_eq!(h.flags & artifact::FLAG_PARTIAL, 0);
    assert_eq!(h.version, artifact::VERSION);
    assert_eq!(h.latency as usize, report.latency_frames);
    assert_eq!(p.mics, 2);
    // another board, another unit: no blob
    assert!(
        mic::compile_blob(&b, &Machine::new("j998").unwrap(), &partial(), false, &mut Diagnostics::default()).is_err()
    );
    let other = Machine::with_unit("j999", &common::other_unit()).unwrap();
    assert!(mic::compile_blob(&b, &other, &partial(), false, &mut Diagnostics::default()).is_err());
    // an unknown unit compiles, unbound
    let (blob, _) =
        mic::compile_blob(&b, &Machine::new("j999").unwrap(), &partial(), false, &mut Diagnostics::default()).unwrap();
    assert!(!artifact::decode(&blob.unwrap()).unwrap().0.binding.unit_bound());
    // an unverified tree only in development
    let t = f.tree("mic-bind-tree");
    assert!(mic::compile_blob(&t, &m, &partial(), false, &mut Diagnostics::default()).is_err());
}

#[test]
fn nested_graphs_flatten_with_their_parameter_logic() {
    // three channels: a filter box on all, then channel 2 replaced by the
    // sum of channel 2 and a gain-scaled copy of channel 0; the gain comes
    // from parameter logic driven by the parent's parameter
    let text = r#"graphName Nest
[def n 3]
box Eq (aufx tmst appl) 1 1
box Sp split 1 3
box Jn join 3 1
box Up dbgn 1 1
box Mix asum 2 1
box Neg parameter_logical_not 0 0
box Scale parameter_multiply 0 0
in in
out out
wire in Eq ([sampleRate] [n] [blockSize])
wire Eq Sp ([sampleRate] [n] [blockSize])
wire (Sp 0) (Jn 0) ([sampleRate] 1 [blockSize])
wire (Sp 0) Up ([sampleRate] 1 [blockSize])
wire (Sp 1) (Jn 1) ([sampleRate] 1 [blockSize])
wire (Sp 2) (Mix 0) ([sampleRate] 1 [blockSize])
wire Up (Mix 1) ([sampleRate] 1 [blockSize])
wire Mix (Jn 2) ([sampleRate] 1 [blockSize])
wire Jn out ([sampleRate] [n] [blockSize])
param flag 0 in
param step -12 in
wireGraphParam flag (Neg 0 input 0)
wireGraphParam step (Scale 0 input 1)
wireParam (Neg 0 output 0) (Scale 0 input 0) true
wireParam (Scale 0 output 0) (Up 0) true
"#;
    let (text, _) = mic::prepare_graph(text);
    let g = aurora_speaker_tuning::dspg::parse(&text, &BTreeMap::new(), "nest").unwrap();
    assert_eq!(g.name.as_deref(), Some("Nest"));
    let strip_v = dict(vec![(
        "strips",
        Value::Array(vec![dict(vec![(
            "effects",
            Value::Array(vec![
                unit_effect("Eq", "tmst", &tone_state(-6.0, 2000.0, 2.0), false, None),
                unit_effect("Up", "dbgn", &state(&[(0, 0.0), (1, 0.0)]), false, Some("dbgain")),
            ]),
        )])]),
    )]);
    let strip = aurora_speaker_tuning::strip::AuStrip::parse(&binary::write(&strip_v), "nest strip").unwrap();
    let mut notes = Vec::new();
    let mut dropped = Vec::new();
    let parent: BTreeMap<String, f64> = [("flag".to_string(), 0.0)].into();
    let nest = mic::Nest {
        graph: &g,
        strip: &strip,
        props: None,
        parent_params: &parent,
        width: 3,
        fs: 48_000.0,
        dev_drop_tone_types: 0,
    };
    let spec = mic::nested(&nest, &mut notes, &mut dropped).unwrap();
    use sgv2_core::voicing::chain::{Chain, Kind, Source};
    assert_eq!(spec.program_channels, 3);
    assert_eq!(spec.nodes.len(), 3, "{:?}", spec.nodes.iter().map(|n| &n.kind).collect::<Vec<_>>());
    assert!(matches!(spec.nodes[0].kind, Kind::Tone(_)));
    // not(0) * -12 = -12 dB on the copy of channel 0
    let Kind::Tone(t) = &spec.nodes[1].kind else { panic!() };
    assert!((t.channels[0].gain - 10f64.powf(-12.0 / 20.0)).abs() < 1e-12);
    assert_eq!(spec.nodes[1].inputs, vec![Source::Node(0, 0)]);
    assert!(matches!(spec.nodes[2].kind, Kind::Sum(1)));
    assert_eq!(spec.outputs, vec![Source::Node(0, 0), Source::Node(0, 1), Source::Node(2, 0)]);
    // and it runs
    let mut c = Chain::new(spec, 48_000.0, 64).unwrap();
    let x = [vec![1.0f32; 64], vec![0.0; 64], vec![0.0; 64]];
    let mut y = [vec![0.0f32; 64], vec![0.0; 64], vec![0.0; 64]];
    let ins: Vec<&[f32]> = x.iter().map(|v| v.as_slice()).collect();
    let mut outs: Vec<&mut [f32]> = y.iter_mut().map(|v| v.as_mut_slice()).collect();
    c.process(&ins, &mut outs, 64);
    assert!(y[1].iter().all(|v| *v == 0.0));
    assert!((y[2][63] / y[0][63] - 10f32.powf(-12.0 / 20.0)).abs() < 1e-4);
}

#[test]
fn named_parameter_links_and_unquoted_names_are_prepared_for_the_reader() {
    let text = "graphName plain\nin in\nout out\nbox N (aufx dspg appl) 1 1\nwire in N (48000 1)\nwire N out (48000 1)\nparam gmco 0 in\nwireGraphParam gmco (N gmco) ; by name\n";
    let (prepared, links) = mic::prepare_graph(text);
    assert_eq!(links, vec![mic::NamedLink { param: "gmco".into(), node: "N".into(), code: "gmco".into() }]);
    let g = aurora_speaker_tuning::dspg::parse(&prepared, &BTreeMap::new(), "t").unwrap();
    assert_eq!(g.name.as_deref(), Some("plain"));
    assert!(g.param_links.is_empty());
    // numeric links are left to the reader
    let (prepared, links) = mic::prepare_graph(&text.replace("(N gmco)", "(N 3)"));
    assert!(links.is_empty());
    assert_eq!(aurora_speaker_tuning::dspg::parse(&prepared, &BTreeMap::new(), "t").unwrap().param_links.len(), 1);
}

#[test]
fn notch_slots_are_carried_ahead_of_the_filter_box() {
    let mut f = Fixture::new();
    let mut st = tone_state(-3.0, 1000.0, 1.0);
    // slot 1: a running notch at 9 kHz, quality 4, on channel 0
    let g = &mut st.sections[0].params;
    for (k, v) in [(0u32, 3.0f32), (1, 6.0), (2, 0.0), (3, 9000.0), (4, 4.0), (5, 2.0), (7, 1.0)] {
        let id = 24 + k;
        g.retain(|(i, _)| *i != id);
        g.push((id, v));
    }
    f.effects[5] = unit_effect("Eq", "tmst", &st, false, None);
    let b = f.tree("mic-notch");
    let c = mic::compile(&b, &partial(), &mut Diagnostics::default()).unwrap();
    assert!(!c.refused(), "{:?}", c.report.refusals);
    let p = c.params.unwrap();
    let Stage::Graph(spec) = &p.post[0] else { panic!("{:?}", p.post) };
    use sgv2_core::voicing::chain::Kind;
    assert_eq!(spec.nodes.len(), 2);
    let Kind::Tone(n) = &spec.nodes[0].kind else { panic!() };
    assert_eq!(n.channels[0].sections(), &[mic::notch(9000.0, 4.0, 48_000.0)]);
    let s0 = mic::notch(9000.0, 4.0, 48_000.0);
    assert!(s0.magnitude(9000.0, 48_000.0) < 1e-9 && (s0.magnitude(100.0, 48_000.0) - 1.0).abs() < 1e-4);
    // a notch with a gain is not characterised
    let g = &mut st.sections[0].params;
    g.retain(|(i, _)| *i != 26);
    g.push((26, 2.0));
    f.effects[5] = unit_effect("Eq", "tmst", &st, false, None);
    let c = mic::compile(&f.tree("mic-notch-gain"), &partial(), &mut Diagnostics::default()).unwrap();
    assert!(c.refused());
}

#[test]
fn suppressor_modes_and_profiles_that_do_not_fit_refuse() {
    // another processing mode
    let mut f = Fixture::new();
    f.effects[1] = unit_effect("Sens", "sens", &sens_state(3.0), false, None);
    let c = mic::compile(&f.tree("mic-sens-mode"), &partial(), &mut Diagnostics::default()).unwrap();
    assert!(c.refused());
    assert!(c.report.refusals.iter().any(|r| r.contains("processing mode")), "{:?}", c.report.refusals);
    // a profile for another block size or microphone count
    for (name, file) in [("mic-sens-block", sens_file(2, 0.0, 480.0)), ("mic-sens-mics", sens_file(3, 0.0, HOP as f64))]
    {
        let mut f = Fixture::new();
        f.sens = file;
        let c = mic::compile(&f.tree(name), &partial(), &mut Diagnostics::default()).unwrap();
        assert!(c.refused(), "{name}");
        assert!(c.report.stages.iter().any(|s| s.name == "Sens" && s.status == Status::Refused));
    }
}

#[test]
fn a_beamformer_outside_the_fixed_beam_mode_refuses() {
    let mut f = Fixture::new();
    f.effects[3] = unit_effect("Beam", "bmfd", &beam_state(false), false, None);
    let c = mic::compile(&f.tree("mic-beam-mode"), &partial(), &mut Diagnostics::default()).unwrap();
    assert!(c.refused());
    assert!(c.report.refusals.iter().any(|r| r.contains("fixed-beam mode")), "{:?}", c.report.refusals);
}

/// The fixture's graph with a second beamformer (a delay-and-sum on the
/// analysis) and a low-frequency blend unit after the main beam.
fn blend_graph() -> String {
    graph(None)
        .replace(
            "box Beam (aufx bmfd appl) 2 4\n",
            "box Beam (aufx bmfd appl) 2 4\nbox Das (aufx bmfd appl) 1 1\nbox Lw ( aufx lwbo appl ) 3 1\n",
        )
        .replace(
            "wire (Beam 0) Ns (freq [fs] 1 [blockSize])\n",
            "wire A Das (freq [fs] [numChansIn] [blockSize])\nwire (Beam 0) (Lw 0) (freq [fs] 1 [blockSize])\n\
             wire Das (Lw 1) (freq [fs] 1 [blockSize])\nwire Das (Lw 2) (freq [fs] 1 [blockSize])\n\
             wire Lw Ns (freq [fs] 1 [blockSize])\n",
        )
}

fn lwbo_state(reference: f32) -> ParamState {
    state(&[
        (0, reference),
        (1, 3000.0),
        (2, 0.0),
        (3, 0.0),
        (5, -90.0),
        (6, 0.0),
        (7, -110.0),
        (8, 0.0),
        (9, 0.0),
        (10, 0.0),
        (11, 0.0),
        (12, 0.0),
        (13, 0.0),
        (14, -180.0),
        (15, 0.0),
        (16, 0.0),
        (17, 0.0),
    ])
}

#[test]
fn a_low_frequency_blend_after_two_beamformers_compiles_and_runs() {
    let mut f = Fixture::new();
    f.graph = blend_graph();
    f.effects.push(unit_effect("Das", "bmfd", &beam_state(true), false, None));
    f.effects.push(unit_effect("Lw", "lwbo", &lwbo_state(1.0), false, None));
    f.props.push(("Das".into(), 4630, "mic_das.plist".into()));
    // the reference: microphone 0 alone; the main beam: twice the mean
    f.files.push(("mic_das.plist".into(), beam_file(2, &[1.0, 0.0])));
    f.beam = beam_file(2, &[1.0, 1.0]);
    let b = f.tree("mic-lwbo");
    // not yet measured against the unit: refused by default, partial on request
    let d = mic::compile(&b, &mic::Options::default(), &mut Diagnostics::default()).unwrap();
    assert!(d.refused() && d.report.refusals.iter().any(|r| r.contains("aufx/lwbo")), "{:?}", d.report.refusals);
    let opts = partial();
    let c = mic::compile(&b, &opts, &mut Diagnostics::default()).unwrap();
    assert!(!c.refused(), "{:?}", c.report.refusals);
    assert!(c.report.partial);
    let st: Vec<(&str, &Status)> = c.report.stages.iter().map(|s| (s.name.as_str(), &s.status)).collect();
    assert!(st.contains(&("Das", &Status::Exact)) && st.contains(&("Lw", &Status::Uncharacterised)), "{st:?}");
    let p = c.params.unwrap();
    assert_eq!(p.refs.len(), 1);
    assert!(matches!(p.freq[0], sgv2_core::mic::FreqStage::LowBand { compare: 0, blend: 0, .. }));
    // it encodes, decodes and runs: under 3 kHz the beam (2 x) is limited
    // to the reference (1 x); far over it, it passes
    let unit = common::test_unit();
    let blob = mic::compile_blob(
        &f.installed("mic-lwbo-blob", "j999", &unit),
        &Machine::with_unit("j999", &unit).unwrap(),
        &opts,
        false,
        &mut Diagnostics::default(),
    )
    .unwrap()
    .0
    .expect("a blob");
    let (_, q) = artifact::decode(&blob).unwrap();
    assert_eq!(q, p);
    let amp = |hz: f64| {
        let mut e = Engine::new(q.clone()).unwrap();
        let n = 24_000;
        let x: Vec<f32> =
            (0..n).map(|k| 0.1 * (2.0 * std::f64::consts::PI * hz * k as f64 / 48_000.0).sin() as f32).collect();
        let mut y = vec![0.0f32; n];
        for at in (0..n).step_by(240) {
            e.process(&[&x[at..at + 240], &x[at..at + 240]], &mut y[at..at + 240], 240);
        }
        let tail = &y[12_000..];
        (tail.iter().map(|v| f64::from(*v).powi(2)).sum::<f64>() / tail.len() as f64).sqrt() * 2f64.sqrt()
    };
    let fs = 48_000.0;
    let eq = |hz: f64| sgv2_core::voicing::tone::peak(-3.0, 1000.0, 1.0, fs).magnitude(hz, fs);
    let gain = 2.0; // the Pre gain box
    let low = amp(500.0) / (0.1 * gain * eq(500.0));
    let high = amp(8000.0) / (0.1 * gain * eq(8000.0));
    assert!((low - 1.0).abs() < 0.02, "500 Hz: {low}");
    assert!((high - 2.0).abs() < 0.04, "8 kHz: {high}");
    // another reference channel is not characterised
    let mut f2 = Fixture::new();
    f2.graph = blend_graph();
    f2.effects.push(unit_effect("Das", "bmfd", &beam_state(true), false, None));
    f2.effects.push(unit_effect("Lw", "lwbo", &lwbo_state(2.0), false, None));
    f2.props.push(("Das".into(), 4630, "mic_das.plist".into()));
    f2.files.push(("mic_das.plist".into(), beam_file(2, &[1.0, 0.0])));
    let c = mic::compile(&f2.tree("mic-lwbo-ref"), &partial(), &mut Diagnostics::default()).unwrap();
    assert!(c.refused());
}

#[test]
fn the_phase_companion_is_a_butterworth_allpass() {
    let fs = 48_000.0;
    // order-6 slot -> Butterworth-3 all-pass: one first- and one second-order section
    let ap = mic::butterworth_allpass(3, 3000.0, fs);
    assert_eq!(ap.len(), 2);
    for hz in [20.0, 300.0, 3000.0, 9000.0, 20_000.0] {
        let m: f64 = ap.iter().map(|s| s.magnitude(hz, fs)).product();
        assert!((m - 1.0).abs() < 1e-9, "{hz} Hz: {m}");
    }
    // order 2 (Butterworth-1 all-pass is first order); Butterworth-2 = the
    // textbook second-order all-pass at Q = 1/sqrt(2)
    let b2 = mic::butterworth_allpass(2, 1000.0, fs);
    assert_eq!(b2.len(), 1);
    let w0 = 2.0 * std::f64::consts::PI * 1000.0 / fs;
    let alpha = w0.sin() / (2.0 * std::f64::consts::FRAC_1_SQRT_2);
    let a0 = 1.0 + alpha;
    let s = b2[0];
    assert!((s.a1 - (-2.0 * w0.cos() / a0)).abs() < 1e-9 && (s.a2 - (1.0 - alpha) / a0).abs() < 1e-9, "{s:?}");
    assert!((s.b0 - s.a2).abs() < 1e-12 && (s.b1 - s.a1).abs() < 1e-12 && (s.b2 - 1.0).abs() < 1e-12);
}

/// The base graph with the suppressor and pre-gain moved after the analysis
/// (`freq_sens`) and/or a trackpad block after the synthesis (`trackpad`).
fn neo_graph(freq_sens: bool, trackpad: bool) -> String {
    // the trackpad block is characterised at block 480 only
    let mut g = graph(None).replace(&format!("[def blockSize {HOP}]"), "[def blockSize 480]");
    if freq_sens {
        let time = "wire Rb Sens ([fs] [numChansIn] [blockSize])\nwire Sens Pre ([fs] [numChansIn] [blockSize])\n\
                    wire Pre A ([fs] [numChansIn] [blockSize])\nwire A (Beam 0) (freq [fs] [numChansIn] [blockSize])\n\
                    wire A (Beam 1) (freq [fs] [numChansIn] [blockSize])\n";
        assert!(g.contains(time));
        g = g.replace(
            time,
            "wire Rb A ([fs] [numChansIn] [blockSize])\nwire A Sens (freq [fs] [numChansIn] [blockSize])\n\
             wire Sens Pre (freq [fs] [numChansIn] [blockSize])\nwire Pre (Beam 0) (freq [fs] [numChansIn] [blockSize])\n\
             wire Pre (Beam 1) (freq [fs] [numChansIn] [blockSize])\n",
        );
    }
    if trackpad {
        let path = "wire S Eq ([fs] 1 [blockSize])\n";
        assert!(g.contains(path));
        g = g.replace(
            path,
            "box Tp (aufx nnet appl) 1 1\nbox Td delay 1 1\nbox Tn adif 2 1\nbox Ta dbgn 1 1\nbox Tm asum 2 1\n\
             wire S Tp ([fs] 1 [blockSize])\nwire S Td ([fs] 1 [blockSize])\nwire Td (Tn 0) ([fs] 1 [blockSize])\n\
             wire Tp (Tn 1) ([fs] 1 [blockSize])\nwire Tn Ta ([fs] 1 [blockSize])\nwire Tp (Tm 0) ([fs] 1 [blockSize])\n\
             wire Ta (Tm 1) ([fs] 1 [blockSize])\nwire Tm Eq ([fs] 1 [blockSize])\nwireParam (Tp 14 0 0) (Td 0 0 0)\n",
        );
    }
    g
}

fn neo_fixture(freq_sens: bool, trackpad: bool, unit_bypassed: bool) -> Fixture {
    let mut f = Fixture::new();
    f.graph = neo_graph(freq_sens, trackpad);
    f.sens = sens_file(2, 0.0, 480.0);
    f.beam = beam_file_n(2, &[0.5, 0.5], 481);
    if trackpad {
        f.effects.push(unit_effect("Tp", "nnet", &state(&[(0, 0.0)]), unit_bypassed, None));
    }
    f
}

#[test]
fn a_frequency_placed_suppressor_with_a_bypassed_trackpad_block_is_the_time_domain_chain() {
    let base = mic::compile(
        &neo_fixture(false, false, true).tree("mic-neo-base"),
        &mic::Options::default(),
        &mut Diagnostics::default(),
    )
    .unwrap();
    assert!(!base.refused(), "{:?}", base.report.refusals);
    let c = mic::compile(
        &neo_fixture(true, true, true).tree("mic-neo"),
        &mic::Options::default(),
        &mut Diagnostics::default(),
    )
    .unwrap();
    assert!(!c.refused(), "{:?}", c.report.refusals);
    let st: Vec<(&str, &Status)> = c.report.stages.iter().map(|s| (s.name.as_str(), &s.status)).collect();
    for n in ["Tp", "Td", "Tn", "Ta", "Tm"] {
        assert!(st.contains(&(n, &Status::Exact)), "{st:?}");
    }
    // the suppressor's extra block is the trackpad block's delay: the same
    // engine as the time-domain graph without the block
    assert_eq!(c.params.unwrap(), base.params.unwrap());
}

#[test]
fn a_trackpad_block_without_a_frequency_placed_suppressor_refuses() {
    let c = mic::compile(
        &neo_fixture(false, true, true).tree("mic-neo-tp-alone"),
        &mic::Options::default(),
        &mut Diagnostics::default(),
    )
    .unwrap();
    assert!(c.refused() && c.report.refusals.iter().any(|r| r.contains("trackpad block")), "{:?}", c.report.refusals);
}

#[test]
fn a_frequency_placed_suppressor_without_a_trackpad_block_refuses() {
    let c = mic::compile(
        &neo_fixture(true, false, true).tree("mic-neo-sens-alone"),
        &mic::Options::default(),
        &mut Diagnostics::default(),
    )
    .unwrap();
    assert!(c.refused() && c.report.refusals.iter().any(|r| r.contains("one block later")), "{:?}", c.report.refusals);
}

#[test]
fn a_trackpad_block_with_a_live_neural_unit_refuses() {
    let c = mic::compile(
        &neo_fixture(true, true, false).tree("mic-neo-tp-live"),
        &mic::Options::default(),
        &mut Diagnostics::default(),
    )
    .unwrap();
    assert!(
        c.refused() && c.report.refusals.iter().any(|r| r.contains("only a bypassed neural unit")),
        "{:?}",
        c.report.refusals
    );
}
