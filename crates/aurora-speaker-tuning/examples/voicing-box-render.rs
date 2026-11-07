// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Render one voicing box of a Mac's speaker graph through v2's own
//! implementation, offline: the Linux side of the per-box rig.
//!
//! ```text
//! voicing-box-render --tunings-root DIR --aid N --box NAME --ch C IN.f32 OUT.f32
//! ```
//!
//! The box's saved state is compiled from the tuning exactly as the
//! provisioning compile does (`aurora_speaker_tuning::voicing`); a refusal
//! is printed and the exit status is 1. IN and OUT are interleaved
//! little-endian f32 with C channels, rendered in 512-frame slices. A box
//! the saved state bypasses is copied through. For the differential rig;
//! never part of the plugin.

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::dspg::analysis::{Lane, Origin, Pass};
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::{prestage, voicing, Diagnostics};
use sgv2_core::voicing::chain::{Chain, Kind, NodeSpec, Source, Spec};
use std::path::PathBuf;
use std::process::exit;

fn die(msg: impl std::fmt::Display) -> ! {
    eprintln!("voicing-box-render: {msg}");
    exit(1)
}

fn main() {
    let mut root = None;
    let (mut aid, mut name, mut ch, mut only_slot) = (None, None, None, None);
    let mut voicing_chain = false;
    let mut sample_rate = None;
    let mut files = Vec::new();
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--tunings-root" => root = it.next().map(PathBuf::from),
            "--aid" => aid = it.next().and_then(|v| v.parse::<u32>().ok()),
            "--box" => name = it.next(),
            "--ch" => ch = it.next().and_then(|v| v.parse::<usize>().ok()),
            "--only-slot" => only_slot = it.next().and_then(|v| v.parse::<u32>().ok()),
            "--voicing-chain" => voicing_chain = true,
            "--sample-rate" => sample_rate = it.next().and_then(|v| v.parse::<i128>().ok()),
            _ => files.push(a),
        }
    }
    let (Some(root), Some(aid), Some(name), Some(ch), [inp, out]) = (root, aid, name, ch, files.as_slice()) else {
        eprintln!(
            "usage: voicing-box-render --tunings-root DIR --aid N --box NAME --ch C [--voicing-chain] [--sample-rate HZ] IN.f32 OUT.f32"
        );
        exit(2)
    };
    let mut diags = Diagnostics::default();
    let bundle = Bundle::from_tunings_root(&root, aid).unwrap_or_else(|e| die(format!("{e:?}")));
    let t = Tuning::load(&bundle, &Options::default(), &mut diags).unwrap_or_else(|e| die(format!("{e:?}")));
    let fs = sample_rate.or(t.selection.sample_rate).unwrap_or(48_000) as f64;
    let mut stage = t.chain.iter().find(|s| s.name == name).cloned().unwrap_or_else(|| die(format!("no box {name}")));
    if let Some(slot) = only_slot {
        let global = stage
            .state
            .as_mut()
            .and_then(|s| s.sections.iter_mut().find(|s| s.scope == 0 && s.element == 0))
            .unwrap_or_else(|| die(format!("box {name} has no global saved state")));
        let mut set = |id: u32, value: f32| {
            if let Some((_, v)) = global.params.iter_mut().find(|(i, _)| *i == id) {
                *v = value;
            } else {
                global.params.push((id, value));
            }
        };
        for id in [0, 1, 4, 11, 13] {
            set(id, 0.0);
        }
        for i in 0..prestage::SLOTS {
            if i != slot {
                set(14 + 10 * i, 0.0);
            }
        }
    }
    let stage = &stage;

    let raw = std::fs::read(inp).unwrap_or_else(|e| die(e));
    let mut x: Vec<f32> = raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    let frames = x.len() / ch;
    let mut planar: Vec<Vec<f32>> = (0..ch).map(|c| (0..frames).map(|i| x[i * ch + c]).collect()).collect();

    let subtype = stage.unit.map(|u| u.subtype.to_string()).unwrap_or_default();
    match subtype.as_str() {
        "mstk" => {
            let p = voicing::mstk::compile(stage, fs, &mut diags).unwrap_or_else(|e| die(format!("{e:?}")));
            if let Some(p) = p {
                let mut run = sgv2_core::voicing::mstk::Run::new(&p, fs, ch);
                let mut at = 0;
                while at < frames {
                    let k = (frames - at).min(512);
                    let mut bufs: Vec<&mut [f32]> = planar.iter_mut().map(|v| &mut v[at..at + k]).collect();
                    run.process(&mut bufs, k);
                    at += k;
                }
            }
        }
        "tmst" => {
            if voicing_chain {
                let (tone, partial) =
                    prestage::voicing_tone(stage, ch, false, fs, &mut diags).unwrap_or_else(|e| die(format!("{e:?}")));
                if !partial.is_empty() {
                    die(format!("volume automation entries carried only in part: {partial:?}"));
                }
                let spec = Spec {
                    program_channels: ch,
                    nodes: vec![NodeSpec { kind: Kind::Tone(tone), inputs: (0..ch).map(Source::Program).collect() }],
                    outputs: (0..ch).map(|channel| Source::Node(0, channel)).collect(),
                    links: Vec::new(),
                };
                let mut run = Chain::new(spec, fs, 512).unwrap_or_else(|| die("invalid isolated Tone chain"));
                let mut rendered = vec![vec![0.0f32; frames]; ch];
                let mut at = 0;
                while at < frames {
                    let k = (frames - at).min(512);
                    let input: Vec<&[f32]> = planar.iter().map(|v| &v[at..at + k]).collect();
                    let mut output: Vec<&mut [f32]> = rendered.iter_mut().map(|v| &mut v[at..at + k]).collect();
                    run.process(&input, &mut output, k);
                    at += k;
                }
                planar = rendered;
                for i in 0..frames {
                    for c in 0..ch {
                        x[i * ch + c] = planar[c][i];
                    }
                }
                let bytes: Vec<u8> = x.iter().flat_map(|v| v.to_le_bytes()).collect();
                std::fs::write(out, bytes).unwrap_or_else(|e| die(e));
                return;
            }
            let lanes: Vec<Lane> = (0..ch)
                .map(|c| Lane {
                    origin: Origin { node: "in".into(), port: 0, channel: c as u32 },
                    hops: Vec::new(),
                    through: vec![Pass { node: stage.name.clone(), channel: c as u32 }],
                })
                .collect();
            let mut p = prestage::derive_lanes(std::slice::from_ref(stage), &lanes, fs, ch, &mut diags)
                .unwrap_or_else(|e| die(format!("{e:?}")));
            // The production pre-stage applies one final whole-lane unity
            // envelope after every box. Slot-isolation rigs compare the
            // slot's sections themselves, before that envelope; restore a
            // target boost that derive_lanes deliberately scaled away.
            if let Some(slot) = only_slot {
                let global = stage.state.as_ref().and_then(|s| s.global());
                let kind = global.and_then(|g| g.get(14 + 10 * slot + 1)).unwrap_or(0.0) as u32;
                let gain = f64::from(global.and_then(|g| g.get(14 + 10 * slot + 2)).unwrap_or(0.0));
                let raw_gain = match kind {
                    0 | 7 => Some(10f64.powf(-gain / 20.0)),
                    2 => Some(1.0),
                    _ => None,
                };
                if let Some(raw_gain) = raw_gain {
                    for channel in &mut p.channel[..ch] {
                        channel.gain = raw_gain;
                    }
                }
            }
            let mut run = sgv2_core::prestage::PreStage::with_rate(p, fs);
            let mut at = 0;
            while at < frames {
                let k = (frames - at).min(512);
                let mut bufs: Vec<&mut [f32]> = planar.iter_mut().map(|v| &mut v[at..at + k]).collect();
                run.process_block(&mut bufs, k);
                at += k;
            }
        }
        other => die(format!("box {name} is aufx/{other}: no v2 implementation in this renderer yet")),
    }

    for i in 0..frames {
        for c in 0..ch {
            x[i * ch + c] = planar[c][i];
        }
    }
    let bytes: Vec<u8> = x.iter().flat_map(|v| v.to_le_bytes()).collect();
    std::fs::write(out, bytes).unwrap_or_else(|e| die(e));
}
