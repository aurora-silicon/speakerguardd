//! Offline render of the virtual bass box for the differential rig.
//!
//! `vbass_render (--tunings-root DIR --aid N | --bundle DIR) [--set ID=VALUE..] < in.f32 > out.f32`
//!
//! Input and output are interleaved stereo f32 at 48 kHz. `--set` overrides
//! one saved parameter before compiling, as the rig does on macOS.
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::error::Diagnostics;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::voicing::vbass;
use sgv2_core::voicing::vbass::Run;
use std::io::{Read, Write};
use std::path::PathBuf;

fn main() {
    let mut it = std::env::args().skip(1);
    let (mut root, mut aid, mut dir, mut sets) = (None, None, None, Vec::new());
    while let Some(a) = it.next() {
        let v = it.next().expect("value");
        match a.as_str() {
            "--tunings-root" => root = Some(PathBuf::from(v)),
            "--aid" => aid = Some(v.parse::<u32>().expect("aid")),
            "--bundle" => dir = Some(PathBuf::from(v)),
            "--set" => {
                let (k, x) = v.split_once('=').expect("ID=VALUE");
                sets.push((k.parse::<u32>().expect("id"), x.parse::<f32>().expect("value")));
            }
            _ => panic!("bad arg {a}"),
        }
    }
    let bundle = match (root, aid, dir) {
        (Some(r), Some(a), None) => Bundle::from_tunings_root(&r, a),
        (None, None, Some(d)) if d.is_file() => Bundle::from_tar(&d, None),
        (None, None, Some(d)) => Bundle::from_dir(&d),
        _ => panic!("give --tunings-root and --aid, or --bundle"),
    }
    .expect("bundle");
    let mut diags = Diagnostics::default();
    let mut t = Tuning::load(&bundle, &Options::default(), &mut diags).expect("tuning");
    let stage = t.chain.iter_mut().find(|s| s.kind == "aufx/vben/appl").expect("no vben box");
    if let Some(g) = stage.state.as_mut().and_then(|s| s.sections.iter_mut().find(|x| x.scope == 0 && x.element == 0)) {
        for (k, x) in &sets {
            match g.params.iter_mut().find(|(id, _)| id == k) {
                Some(slot) => slot.1 = *x,
                None => g.params.push((*k, *x)),
            }
        }
    }
    let fs = 48_000.0;
    let p = vbass::compile(stage, fs, &mut diags).expect("compile").expect("bypassed");
    let mut run = Run::new(&p, fs, 2);
    let mut raw = Vec::new();
    std::io::stdin().read_to_end(&mut raw).expect("read");
    let x: Vec<f32> = raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    let n = x.len() / 2;
    let mut l: Vec<f32> = (0..n).map(|i| x[2 * i]).collect();
    let mut r: Vec<f32> = (0..n).map(|i| x[2 * i + 1]).collect();
    for start in (0..n).step_by(512) {
        let end = (start + 512).min(n);
        let mut bufs = [&mut l[start..end], &mut r[start..end]];
        run.process(&mut bufs, end - start);
    }
    let mut out = Vec::with_capacity(n * 8);
    for i in 0..n {
        out.extend_from_slice(&l[i].to_le_bytes());
        out.extend_from_slice(&r[i].to_le_bytes());
    }
    std::io::stdout().write_all(&out).expect("write");
}
