//! Offline differential-rig renderer for the compiled VarEQ box.
//!
//! Input and output are headerless, interleaved little-endian f32. The
//! volume scalar is set before the first frame; `--at FRAME:VOLUME` changes
//! it at the start of the `--q`-frame slice holding FRAME, as a host that
//! sets the unit's parameter between render calls does.

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::voicing::vareq::{self, VRQ2};
use aurora_speaker_tuning::Diagnostics;
use sgv2_core::voicing::vareq::Run;
use std::path::Path;

const USAGE: &str =
    "usage: vareq-render TUNINGS_ROOT AID FS CHANNELS VOLUME IN.f32 OUT.f32 [--q N] [--at FRAME:VOLUME]...";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 7 {
        return Err(USAGE.into());
    }
    let (root, aid, fs, channels, volume) = (
        &args[0],
        args[1].parse::<u32>()?,
        args[2].parse::<f64>()?,
        args[3].parse::<usize>()?,
        args[4].parse::<f64>()?,
    );
    let (input, output) = (&args[5], &args[6]);
    let mut quantum = 512usize;
    let mut changes: Vec<(usize, f64)> = Vec::new();
    let mut rest = args[7..].iter();
    while let Some(a) = rest.next() {
        let v = rest.next().ok_or(USAGE)?;
        match a.as_str() {
            "--q" => quantum = v.parse()?,
            "--at" => {
                let (f, x) = v.split_once(':').ok_or(USAGE)?;
                changes.push((f.parse()?, x.parse()?));
            }
            _ => return Err(USAGE.into()),
        }
    }
    if quantum == 0 || channels == 0 {
        return Err(USAGE.into());
    }

    let bundle = Bundle::from_tunings_root(Path::new(root), aid)?;
    let mut diags = Diagnostics::default();
    let tuning = Tuning::load(&bundle, &Options::default(), &mut diags)?;
    let stage = tuning
        .chain
        .iter()
        .find(|s| s.unit.is_some_and(|u| u.subtype == VRQ2))
        .ok_or("speaker chain has no VarEQ stage")?;
    let params = vareq::compile(stage, fs, &mut diags)?.ok_or("VarEQ is bypassed")?;

    let bytes = std::fs::read(input)?;
    if bytes.len() % (4 * channels) != 0 {
        return Err("input is not whole f32 frames".into());
    }
    let frames = bytes.len() / (4 * channels);
    let mut planes = vec![Vec::with_capacity(frames); channels];
    for (i, s) in bytes.chunks_exact(4).enumerate() {
        planes[i % channels].push(f32::from_le_bytes(s.try_into()?));
    }
    let mut run = Run::new(&params, fs, channels);
    run.set_volume_scalar(volume);
    let mut at = 0;
    while at < frames {
        let n = quantum.min(frames - at);
        for &(f, v) in &changes {
            if f >= at && f < at + n {
                run.set_volume_scalar(v);
            }
        }
        let mut slices: Vec<&mut [f32]> = planes.iter_mut().map(|p| &mut p[at..at + n]).collect();
        run.process(&mut slices, n);
        at += n;
    }
    let mut out = Vec::with_capacity(bytes.len());
    for i in 0..frames {
        for p in &planes {
            out.extend_from_slice(&p[i].to_le_bytes());
        }
    }
    std::fs::write(output, out)?;
    Ok(())
}
