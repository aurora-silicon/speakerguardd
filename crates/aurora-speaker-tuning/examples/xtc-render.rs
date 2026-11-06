//! Offline differential-rig renderer for the compiled XTC box.
//!
//! Input and output are headerless, stereo-interleaved little-endian f32.

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::voicing::xtc::{self, AXTC};
use aurora_speaker_tuning::Diagnostics;
use sgv2_core::voicing::xtc::Run;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let root = args.next().ok_or("tunings root")?;
    let aid: u32 = args.next().ok_or("acoustic ID")?.parse()?;
    let input = args.next().ok_or("input path")?;
    let output = args.next().ok_or("output path")?;
    if args.next().is_some() {
        return Err("usage: xtc-render TUNINGS_ROOT AID INPUT.f32 OUTPUT.f32".into());
    }
    let bundle = Bundle::from_tunings_root(Path::new(&root), aid)?;
    let mut diags = Diagnostics::default();
    let tuning = Tuning::load(&bundle, &Options::default(), &mut diags)?;
    let stage = tuning
        .chain
        .iter()
        .find(|s| s.unit.is_some_and(|u| u.subtype == AXTC))
        .ok_or("speaker chain has no XTC stage")?;
    let params = xtc::compile(stage, 48_000.0, &mut diags)?.ok_or("XTC is direct-only or bypassed")?;
    for d in &diags.items {
        eprintln!("{d}");
    }

    let bytes = std::fs::read(input)?;
    if bytes.len() % 8 != 0 {
        return Err("input is not whole stereo f32 frames".into());
    }
    let mut left = Vec::with_capacity(bytes.len() / 8);
    let mut right = Vec::with_capacity(bytes.len() / 8);
    for frame in bytes.chunks_exact(8) {
        left.push(f32::from_le_bytes(frame[..4].try_into()?));
        right.push(f32::from_le_bytes(frame[4..].try_into()?));
    }
    let mut run = Run::new(&params, 48_000.0, 2);
    let frames = left.len();
    run.process(&mut [&mut left, &mut right], frames);
    let mut out = Vec::with_capacity(bytes.len());
    for i in 0..frames {
        out.extend_from_slice(&left[i].to_le_bytes());
        out.extend_from_slice(&right[i].to_le_bytes());
    }
    std::fs::write(output, out)?;
    eprintln!("frames={frames} taps={} latency={}", params.count, run.latency());
    Ok(())
}
