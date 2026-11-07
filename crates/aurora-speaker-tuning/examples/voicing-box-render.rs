// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Render one voicing box of a Mac's speaker graph through v2's own
//! implementation, offline: the Linux side of the per-box rig.
//!
//! ```text
//! voicing-box-render --tunings-root DIR --aid N --box NAME --ch C IN.f32 OUT.f32
//! voicing-box-render --params-words FILE [--fs HZ] --ch C IN.f32 OUT.f32
//! ```
//!
//! The box's saved state is compiled from the tuning exactly as the
//! provisioning compile does (`aurora_speaker_tuning::voicing`); a refusal
//! is printed and the exit status is 1. IN and OUT are interleaved
//! little-endian f32 with C channels, rendered in 512-frame slices. A box
//! the saved state bypasses is copied through. For the differential rig;
//! never part of the plugin.
//!
//! For a full ToneMeister box, `--voicing-chain --tone-volume V` drives
//! parameter 3 and compiles its slot automation. Supply the value at that
//! box input, including any upstream relay's effect; this isolated render
//! does not run the graph's other boxes. Without `--tone-volume`, it keeps
//! the unwired pre-stage's least-passing settings.

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::dspg::analysis::{Lane, Origin, Pass};
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::{prestage, voicing, Diagnostics};
use sgv2_core::voicing::chain::{Chain, GraphParam, Kind, Link, NodeSpec, Source, Spec};
use std::path::PathBuf;
use std::process::exit;

// The rig's explicit parameter-3 input must enable automation at compile
// time, just as a volume wire does in the full-chain compiler.
pub(crate) fn tone_spec(
    stage: &aurora_speaker_tuning::tuning::Stage,
    ch: usize,
    fs: f64,
    volume: Option<f64>,
    diags: &mut Diagnostics,
) -> aurora_speaker_tuning::Result<Spec> {
    let (tone, partial) = prestage::voicing_tone(stage, ch, volume.is_some(), fs, diags)?;
    if !partial.is_empty() {
        return Err(aurora_speaker_tuning::Error::format(
            "isolated Tone chain",
            format!("volume automation entries carried only in part: {partial:?}"),
        ));
    }
    Ok(Spec {
        program_channels: ch,
        nodes: vec![NodeSpec { kind: Kind::Tone(tone), inputs: (0..ch).map(Source::Program).collect() }],
        outputs: (0..ch).map(|channel| Source::Node(0, channel)).collect(),
        links: volume.map(|_| vec![Link { param: GraphParam::VolumeScalar, node: 0, id: 3 }]).unwrap_or_default(),
        relays: Vec::new(),
    })
}

fn die(msg: impl std::fmt::Display) -> ! {
    eprintln!("voicing-box-render: {msg}");
    exit(1)
}

fn render_mstk_words(path: PathBuf, fs: f64, ch: usize, inp: &str, out: &str) {
    if !fs.is_finite() || fs <= 1.0 {
        die("--params-words needs a valid --fs")
    }
    let raw = std::fs::read(path).unwrap_or_else(|e| die(e));
    let words: Vec<f64> = serde_json::from_slice(&raw).unwrap_or_else(|e| die(e));
    let p = sgv2_core::voicing::mstk::Params::from_words(&words).unwrap_or_else(|| die("invalid mstk parameter words"));
    let raw = std::fs::read(inp).unwrap_or_else(|e| die(e));
    let mut x: Vec<f32> = raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    let frames = x.len() / ch;
    let mut planar: Vec<Vec<f32>> = (0..ch).map(|c| (0..frames).map(|i| x[i * ch + c]).collect()).collect();
    let mut run = sgv2_core::voicing::mstk::Run::new(&p, fs, ch);
    let mut at = 0;
    while at < frames {
        let k = (frames - at).min(512);
        let mut bufs: Vec<&mut [f32]> = planar.iter_mut().map(|v| &mut v[at..at + k]).collect();
        run.process(&mut bufs, k);
        at += k;
    }
    for i in 0..frames {
        for c in 0..ch {
            x[i * ch + c] = planar[c][i];
        }
    }
    let bytes: Vec<u8> = x.iter().flat_map(|v| v.to_le_bytes()).collect();
    std::fs::write(out, bytes).unwrap_or_else(|e| die(e));
}

fn main() {
    let (mut root, mut params_words) = (None, None);
    let mut fs = 48_000.0;
    let (mut aid, mut name, mut ch, mut only_slot) = (None, None, None, None);
    let mut voicing_chain = false;
    let mut sample_rate = None;
    let mut automation_input = None;
    let mut tone_volume = None;
    let mut own_timing = false;
    let mut files = Vec::new();
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--tunings-root" => root = it.next().map(PathBuf::from),
            "--params-words" => params_words = it.next().map(PathBuf::from),
            "--fs" => fs = it.next().and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0),
            "--aid" => aid = it.next().and_then(|v| v.parse::<u32>().ok()),
            "--box" => name = it.next(),
            "--ch" => ch = it.next().and_then(|v| v.parse::<usize>().ok()),
            "--only-slot" => only_slot = it.next().and_then(|v| v.parse::<u32>().ok()),
            "--slot-own-timing" => own_timing = true,
            "--voicing-chain" => voicing_chain = true,
            "--sample-rate" => sample_rate = it.next().and_then(|v| v.parse::<i128>().ok()),
            "--automation-input" => automation_input = it.next().and_then(|v| v.parse::<f64>().ok()),
            // ToneMeister's volume-automation input (its parameter 3), as
            // the installed unit's saved state or a rig sets it.
            "--tone-volume" => tone_volume = it.next().and_then(|v| v.parse::<f64>().ok()),
            _ => files.push(a),
        }
    }
    let (Some(ch), [inp, out]) = (ch, files.as_slice()) else {
        eprintln!(
            "usage: voicing-box-render (--tunings-root DIR --aid N --box NAME [--voicing-chain] [--tone-volume V] [--sample-rate HZ] [--automation-input V] \
             | --params-words FILE [--fs HZ]) --ch C IN.f32 OUT.f32"
        );
        exit(2)
    };
    if let Some(path) = params_words {
        if root.is_some() || aid.is_some() || name.is_some() {
            die("--params-words is exclusive with tuning selection")
        }
        render_mstk_words(path, fs, ch, inp, out);
        return;
    }
    let (Some(root), Some(aid), Some(name)) = (root, aid, name) else {
        die("tuning selection needs --tunings-root, --aid and --box")
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
        // The unit stores attack/release/smoothing triplets per
        // level-dependent slot in use, so an isolated slot reads the first
        // triplet. With --slot-own-timing the slot keeps its own (a rig must
        // then set the installed unit's first triplet the same way).
        let saved = |id: u32| global.params.iter().find(|(i, _)| *i == id).map(|(_, v)| *v).unwrap_or(0.0);
        let in_use = (0..slot)
            .filter(|&j| {
                let kind = saved(14 + 10 * j + 1);
                saved(14 + 10 * j) != 0.0 && (kind == 15.0 || kind == 18.0)
            })
            .count() as u32;
        let triplet = [0, 1, 2].map(|k| saved(616 + 3 * in_use + k));
        let mut set = |id: u32, value: f32| {
            if let Some((_, v)) = global.params.iter_mut().find(|(i, _)| *i == id) {
                *v = value;
            } else {
                global.params.push((id, value));
            }
        };
        if own_timing {
            for (k, v) in triplet.into_iter().enumerate() {
                set(616 + k as u32, v);
            }
        }
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
                if let Some(v) = automation_input {
                    // The value a graph relays into the box's automation
                    // input (AU parameter 79), e.g. a VolumeTaper output.
                    run.set_automation_input(v);
                }
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
                let spec = tone_spec(stage, ch, fs, tone_volume, &mut diags).unwrap_or_else(|e| die(format!("{e:?}")));
                let mut run = Chain::new(spec, fs, 512).unwrap_or_else(|| die("invalid isolated Tone chain"));
                if let Some(v) = tone_volume {
                    run.set_param(GraphParam::VolumeScalar, v);
                }
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
