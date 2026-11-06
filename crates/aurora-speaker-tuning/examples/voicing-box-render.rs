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
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::{voicing, Diagnostics};
use std::path::PathBuf;
use std::process::exit;

fn die(msg: impl std::fmt::Display) -> ! {
    eprintln!("voicing-box-render: {msg}");
    exit(1)
}

fn main() {
    let mut root = None;
    let (mut aid, mut name, mut ch) = (None, None, None);
    let mut files = Vec::new();
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--tunings-root" => root = it.next().map(PathBuf::from),
            "--aid" => aid = it.next().and_then(|v| v.parse::<u32>().ok()),
            "--box" => name = it.next(),
            "--ch" => ch = it.next().and_then(|v| v.parse::<usize>().ok()),
            _ => files.push(a),
        }
    }
    let (Some(root), Some(aid), Some(name), Some(ch), [inp, out]) = (root, aid, name, ch, files.as_slice()) else {
        eprintln!("usage: voicing-box-render --tunings-root DIR --aid N --box NAME --ch C IN.f32 OUT.f32");
        exit(2)
    };
    let mut diags = Diagnostics::default();
    let bundle = Bundle::from_tunings_root(&root, aid).unwrap_or_else(|e| die(format!("{e:?}")));
    let t = Tuning::load(&bundle, &Options::default(), &mut diags).unwrap_or_else(|e| die(format!("{e:?}")));
    let fs = t.selection.sample_rate.unwrap_or(48_000) as f64;
    let stage = t.chain.iter().find(|s| s.name == name).unwrap_or_else(|| die(format!("no box {name}")));

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
