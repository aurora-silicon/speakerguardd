//! Offline differential-rig renderer for the compiled Norm box.
//!
//! `norm-render (--root TUNINGS --aid N | --bundle PATH) IN.f32 OUT.f32 [--q FRAMES] [--trace TRACE.f32] [--set ID=VALUE]...`
//!
//! Input and output are headerless, stereo-interleaved little-endian f32.
//! The input is fed in calls of `--q` frames (default 512); `--trace` writes
//! the applied gain (dB) after each call, like the unit's momentary-gain
//! parameter. `--set ID=VALUE` overrides one saved parameter first.

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::voicing::norm::{self, LDNM};
use aurora_speaker_tuning::Diagnostics;
use sgv2_core::voicing::norm::Run;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let (mut root, mut aid, mut bundle_path, mut q, mut trace) = (None, None, None, 512usize, None);
    let mut pos = Vec::new();
    let mut sets: Vec<(u32, f32)> = Vec::new();
    while let Some(a) = args.next() {
        match a.as_str() {
            "--root" => root = args.next(),
            "--aid" => aid = Some(args.next().ok_or("--aid N")?.parse::<u32>()?),
            "--bundle" => bundle_path = args.next(),
            "--q" => q = args.next().ok_or("--q N")?.parse()?,
            "--trace" => trace = args.next(),
            "--set" => {
                let kv = args.next().ok_or("--set ID=VALUE")?;
                let (k, v) = kv.split_once('=').ok_or("--set ID=VALUE")?;
                sets.push((k.parse()?, v.parse()?));
            }
            _ => pos.push(a),
        }
    }
    let [input, output] = <[String; 2]>::try_from(pos).map_err(|_| "need IN.f32 OUT.f32")?;
    let bundle = match (root, aid, bundle_path) {
        (Some(r), Some(a), None) => Bundle::from_tunings_root(Path::new(&r), a)?,
        (None, None, Some(b)) => Bundle::open(Path::new(&b))?,
        _ => return Err("give --root and --aid, or --bundle".into()),
    };
    let mut diags = Diagnostics::default();
    let tuning = Tuning::load(&bundle, &Options::default(), &mut diags)?;
    let mut stage = tuning
        .chain
        .iter()
        .find(|s| s.unit.is_some_and(|u| u.subtype == LDNM))
        .ok_or("speaker chain has no Norm stage")?
        .clone();
    if let Some(sec) = stage.state.as_mut().and_then(|s| s.sections.iter_mut().find(|x| x.scope == 0)) {
        for (id, v) in &sets {
            sec.params.retain(|(i, _)| i != id);
            sec.params.push((*id, *v));
        }
    }
    let params = norm::compile(&stage, 48_000.0, &mut diags)?.ok_or("Norm is bypassed")?;
    if std::env::var_os("NORM_RENDER_DIAGS").is_some() {
        for d in &diags.items {
            eprintln!("{d}");
        }
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
    let mut tr = Vec::new();
    let mut at = 0;
    while at < frames {
        let n = q.min(frames - at);
        run.process(&mut [&mut left[at..at + n], &mut right[at..at + n]], n);
        tr.extend_from_slice(&(run.gain_db() as f32).to_le_bytes());
        at += n;
    }
    let mut out = Vec::with_capacity(bytes.len());
    for i in 0..frames {
        out.extend_from_slice(&left[i].to_le_bytes());
        out.extend_from_slice(&right[i].to_le_bytes());
    }
    std::fs::write(output, out)?;
    if let Some(t) = trace {
        std::fs::write(t, tr)?;
    }
    Ok(())
}
