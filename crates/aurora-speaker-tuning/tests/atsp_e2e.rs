//! atsp end to end on synthetic bundles shaped like each shipped atsp
//! profile family (SPEC-ATSP 7): the reader and compiler, the blob the
//! sgv2 LV2 plugin loads for the board's variant, and the unit's
//! protection on that plugin's unit. Every value here is this test's own.

mod common;

use std::collections::BTreeMap;

use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::compile;
use aurora_speaker_tuning::machine::Machine;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::Diagnostics;
use common::*;
use sgv2_core::{Io, Telemetry};
use sgv2_lv2::{load_unit, LoadError, Role};

/// A graph that feeds the protection unit `n` channels straight from its
/// input (the one- and two-channel shapes); four channels use the common
/// duplicated-stereo graph (the four-speaker shape).
fn graph(n: usize) -> String {
    if n == 4 {
        return graph_text("atsp");
    }
    format!(
        r#"graphName "test_speaker_out"
[def ch {n}]
[def blk 256]
in in
in sense
out out
box Guard nfnp 1 1
box Prot (aufx atsp) 2 1
box Conv srlconverter 1 1
box Fade dbgn 1 1
format senseFmt ([fs] 8i [blk])
wire in Guard ([fs] [ch])
wire Guard (Prot 0) ([fs] [ch] [blk])
wire sense Conv senseFmt
wire Conv (Prot 1) senseFmt
wire Prot Fade ([fs] [ch])
wire Fade out ([fs] [ch])
"#
    )
}

/// One slot's thirteen values in AU parameter order (spec 2): sensitivity,
/// Rdc, Rvc, tau_vc, Rmg, tau_mg, ambient, limit, attack, release,
/// headroom, link, mask.
fn slot(mask: f32, link: f32) -> [f32; 13] {
    [10.0, 6.0, 30.0, 1.5, 40.0, 30.0, 45.0, 115.0, 500.0, 10_000.0, 8.0, link, mask]
}

struct Shape {
    name: &'static str,
    board: &'static str,
    channels: usize,
    slots: [[f32; 13]; 3],
    link_all: f32,
}

fn shapes() -> Vec<Shape> {
    let off = [8.0, 8.0, 50.0, 2.0, 20.0, 30.0, 50.0, 100.0, 5000.0, 30000.0, 10.0, 0.0, 0.0];
    vec![
        // four speakers, one model on all four, linked (the J293/J493 shape)
        Shape {
            name: "four",
            board: "j293",
            channels: 4,
            slots: [slot(15.0, 1.0), slot(0.0, 0.0), off],
            link_all: 1.0,
        },
        // two speakers, one slot each (the J313 shape)
        Shape { name: "two", board: "j313", channels: 2, slots: [slot(1.0, 1.0), slot(2.0, 1.0), off], link_all: 1.0 },
        // one speaker, a mask wider than the stream (the desktop shape)
        Shape { name: "one", board: "j274", channels: 1, slots: [slot(65535.0, 0.0), off, off], link_all: 0.0 },
    ]
}

fn bundle(s: &Shape) -> BTreeMap<String, Vec<u8>> {
    let mut params: Vec<(u32, f32)> = vec![(0, 0.0), (40, s.link_all)];
    for (k, vals) in s.slots.iter().enumerate() {
        for (j, v) in vals.iter().enumerate() {
            params.push((1 + 13 * k as u32 + j as u32, *v));
        }
    }
    let mut f = spp3_files(s.channels);
    f.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph(s.channels).into_bytes());
    let st = ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] };
    f.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"), strip_with_state("atsp", &st));
    f
}

fn load(files: &BTreeMap<String, Vec<u8>>, name: &str) -> Tuning {
    let dir = scratch(name);
    write_dir(&dir, files, &manifest_json(files, "25G83"));
    let b = Bundle::open(&dir).unwrap();
    let mut d = Diagnostics::default();
    let t = Tuning::load(&b, &Options::default(), &mut d).unwrap();
    assert!(!d.has_errors(), "{:?}", d.items);
    t
}

#[test]
fn every_shape_compiles_loads_in_its_plugin_and_protects() {
    for s in shapes() {
        let t = load(&bundle(&s), &format!("atsp-e2e-{}", s.name));
        let c = compile::compile(&t, &Machine::new(s.board).unwrap(), &mut Diagnostics::default())
            .unwrap_or_else(|e| panic!("{}: {e:?}", s.name));
        assert_eq!((c.channels as usize, c.sense_channels), (s.channels, 0), "{}", s.name);
        assert_eq!(c.board, s.board);
        // atsp runs protection-only: an identity pre-stage that declares no
        // crossover to the kernel
        let pre = sgv2_core::blob::prestage(&c.blob, 48_000.0).unwrap();
        assert!(!pre.from_tuning() && pre.box_count == 0, "{}", s.name);
        assert!(pre.channel[..s.channels].iter().all(|ch| ch.count == 0 && ch.gain == 1.0), "{}", s.name);

        // the board's plugin variant carries no sense (wiring.describe):
        // the plugin takes the blob; a variant with sense refuses it
        let (_, err, _) = load_unit(Some(&c.blob), 48_000.0, s.channels, 2 * s.channels, Role::Protect);
        assert_eq!(err, Some(LoadError::Geometry), "{}", s.name);
        let (mut unit, err, crc) = load_unit(Some(&c.blob), 48_000.0, s.channels, 0, Role::Protect);
        assert_eq!(err, None, "{}", s.name);
        assert_ne!(crc, 0);

        // a hot program: the unit limits, holds the coil at its hard limit,
        // and its output never exceeds
        // the program (beyond the dezipper's startup overshoot)
        let u = unit.as_dyn();
        let x = 0.9f32;
        let ins: Vec<Vec<f32>> = (0..s.channels).map(|_| vec![x; 512]).collect();
        let mut outs = vec![vec![0.0f32; 512]; s.channels];
        let mut t = Telemetry::new();
        let (mut peak, mut last) = (0.0f32, 1.0f32);
        for _ in 0..(60 * 48_000 / 512) {
            let i: Vec<&[f32]> = ins.iter().map(|b| &b[..]).collect();
            let mut o: Vec<&mut [f32]> = outs.iter_mut().map(|b| &mut b[..]).collect();
            let h = u.process(Io { frames: 512, input: &i, sense: &[], output: &mut o });
            assert_eq!(h.0 & sgv2_core::Health::FAILSAFE, 0, "{}", s.name);
            assert!(outs.iter().flatten().all(|y| y.is_finite() && y.abs() <= x * sgv2_core::atsp::DEZIPPER_PEAK));
            u.telemetry(&mut t);
            for ch in &t.channels[..s.channels] {
                peak = peak.max(ch.coil_temp_c);
            }
            last = outs[0][511] / x;
        }
        // the panic acts at the first 5 ms tick that sees the post-gain coil
        // over the hard limit (115 + 8), so it can pass it by what full
        // drive adds over one tick, one post-gain window and the dezipper's
        // glide (about 25 ms at most Rvc P / tau_vc = 30 * 13.5 / 1.5 =
        // 270 C/s)
        assert!(peak > 115.0 && peak < 115.0 + 8.0 + 0.025 * 270.0, "{}: peak coil {peak}", s.name);
        assert!(last < 0.9, "{}: gain {last} after a minute of a hot program", s.name);
        // linked channels share one gain
        if s.channels > 1 {
            let g: Vec<f32> = (0..s.channels).map(|c| outs[c][511]).collect();
            assert!(g.iter().all(|v| *v == g[0]), "{}: {g:?}", s.name);
        }
    }
}
