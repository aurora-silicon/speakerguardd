//! Offline render of the virtual bass box for the differential rig.
//!
//! `vbass_render (--tunings-root DIR --aid N | --bundle DIR | --strip FILE) [--set ID=VALUE..] < in.f32 > out.f32`
//!
//! Input and output are interleaved stereo f32 at 48 kHz. `--set` overrides
//! one saved parameter before compiling, as the rig does on macOS.
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::error::Diagnostics;
use aurora_speaker_tuning::strip::AuStrip;
use aurora_speaker_tuning::tuning::{Options, Stage, Tuning};
use aurora_speaker_tuning::voicing::vbass;
use sgv2_core::voicing::vbass::Run;
use std::io::{Read, Write};
use std::path::PathBuf;

fn main() {
    let mut it = std::env::args().skip(1);
    let (mut root, mut aid, mut dir, mut strip, mut sets) = (None, None, None, None, Vec::new());
    while let Some(a) = it.next() {
        let v = it.next().expect("value");
        match a.as_str() {
            "--tunings-root" => root = Some(PathBuf::from(v)),
            "--aid" => aid = Some(v.parse::<u32>().expect("aid")),
            "--bundle" => dir = Some(PathBuf::from(v)),
            "--strip" => strip = Some(PathBuf::from(v)),
            "--set" => {
                let (k, x) = v.split_once('=').expect("ID=VALUE");
                sets.push((k.parse::<u32>().expect("id"), x.parse::<f32>().expect("value")));
            }
            _ => panic!("bad arg {a}"),
        }
    }
    let mut diags = Diagnostics::default();
    let mut stage = match (root, aid, dir, strip) {
        (Some(r), Some(a), None, None) => {
            let bundle = Bundle::from_tunings_root(&r, a).expect("bundle");
            let t = Tuning::load(&bundle, &Options::default(), &mut diags).expect("tuning");
            t.chain.into_iter().find(|s| s.kind == "aufx/vben/appl").expect("no vben box")
        }
        (None, None, Some(d), None) => {
            let bundle = if d.is_file() { Bundle::from_tar(&d, None) } else { Bundle::from_dir(&d) }.expect("bundle");
            let t = Tuning::load(&bundle, &Options::default(), &mut diags).expect("tuning");
            t.chain.into_iter().find(|s| s.kind == "aufx/vben/appl").expect("no vben box")
        }
        (None, None, None, Some(path)) => {
            let bytes = std::fs::read(&path).expect("read strip");
            let strip = AuStrip::parse(&bytes, &path.display().to_string()).expect("strip");
            let effect = strip.effects().find(|e| e.unit.subtype.bytes() == *b"vben").expect("no vben box");
            Stage {
                name: effect.box_name.clone(),
                kind: effect.unit.to_string(),
                unit: Some(effect.unit),
                inputs: 1,
                outputs: 1,
                channels: Some(2),
                bypass: effect.bypass,
                preset_name: effect.preset.name.clone(),
                state: Some(effect.preset.state.clone()),
                preset_extra_keys: effect.preset.extra_keys(),
                preset_extra: effect.preset.extra.clone(),
                strip_properties: Vec::new(),
                property_files: Vec::new(),
                before_protection: true,
            }
        }
        _ => panic!("give --tunings-root and --aid, --bundle, or --strip"),
    };
    if let Some(g) = stage.state.as_mut().and_then(|s| s.sections.iter_mut().find(|x| x.scope == 0 && x.element == 0)) {
        for (k, x) in &sets {
            match g.params.iter_mut().find(|(id, _)| id == k) {
                Some(slot) => slot.1 = *x,
                None => g.params.push((*k, *x)),
            }
        }
    }
    let fs = 48_000.0;
    let p = vbass::compile(&stage, fs, &mut diags).expect("compile").expect("bypassed");
    let mut raw = Vec::new();
    std::io::stdin().read_to_end(&mut raw).expect("read");
    let x: Vec<f32> = raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    let n = x.len() / 2;
    let mut run = Run::new(&p, fs, 2, 512);
    if std::env::var_os("VBASS_COEFFICIENTS").is_some() {
        let mut words = [[0u32; 5]; 20];
        let sections = run.crossover_lowpass_bits(&mut words);
        for section in &words[..sections] {
            eprintln!("{:08x} {:08x} {:08x} {:08x} {:08x}", section[0], section[1], section[2], section[3], section[4]);
        }
    }
    let mut l: Vec<f32> = (0..n).map(|i| x[2 * i]).collect();
    let mut r: Vec<f32> = (0..n).map(|i| x[2 * i + 1]).collect();
    // VBASS_TIES=FILE: per 512-frame block, the closest level-control
    // hold-restart test to macOS's threshold ("seconds margin").
    let mut ties = std::env::var("VBASS_TIES").ok().map(|f| std::fs::File::create(f).expect("ties"));
    for start in (0..n).step_by(512) {
        let end = (start + 512).min(n);
        let mut bufs = [&mut l[start..end], &mut r[start..end]];
        run.process(&mut bufs, end - start);
        let m = run.take_tie_margin();
        if let Some(f) = ties.as_mut() {
            if m < 0.01 {
                writeln!(f, "{:.6} {:.3e}", start as f64 / fs, m).expect("ties");
            }
        }
    }
    let mut out = Vec::with_capacity(n * 8);
    for i in 0..n {
        out.extend_from_slice(&l[i].to_le_bytes());
        out.extend_from_slice(&r[i].to_le_bytes());
    }
    std::io::stdout().write_all(&out).expect("write");
}
