// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Offline check of the speaker volume law a bundle compiles to.
//!
//! `hal-volume-check (--bundle PATH | --tunings-root DIR --aid N) [--measured FILE] [--tolerance DB]`
//!
//! Loads the tuning, compiles its voicing chain and prints the volume law the
//! artifact will carry.  With `--measured`, a file of `scalar dB` lines read
//! from macOS (the HAL's own scalar -> dB, e.g. a VolumeScalar set and
//! VolumeDecibels read back), it compares the law at every point and exits
//! non-zero if any point misses by more than the tolerance (default 0.01 dB).

use std::path::Path;

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::error::Diagnostics;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::voicing::chain::compile;
use sgv2_core::voicing::volume::Curve;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let (mut bundle, mut root, mut aid, mut measured, mut tol) = (None, None, None, Vec::new(), 0.01f64);
    while let Some(a) = args.next() {
        let mut v = || args.next().ok_or_else(|| format!("{a} needs a value"));
        match a.as_str() {
            "--bundle" => bundle = Some(v()?),
            "--tunings-root" => root = Some(v()?),
            "--aid" => aid = Some(v()?.trim_start_matches("AID").parse::<u32>()?),
            "--measured" => measured.push(v()?),
            "--tolerance" => tol = v()?.parse()?,
            _ => return Err(format!("unknown argument {a}").into()),
        }
    }
    let b = match (bundle, root, aid) {
        (Some(p), None, None) => {
            let p = Path::new(&p);
            if p.is_dir() {
                Bundle::from_dir(p)?
            } else {
                Bundle::from_tar(p, None)?
            }
        }
        (None, Some(r), Some(a)) => Bundle::from_tunings_root(Path::new(&r), a)?,
        _ => return Err("give --bundle PATH, or --tunings-root DIR --aid N".into()),
    };
    let mut d = Diagnostics::default();
    let t = Tuning::load(&b, &Options::default(), &mut d)?;
    let curve = match compile(&t, 48_000.0, &mut d) {
        Ok(c) => c.spec.volume,
        Err(e) => {
            eprintln!("note: voicing chain does not compile ({e}); using the tuning's range directly");
            t.volume_range.as_ref().and_then(|r| r.curve()).unwrap_or_default()
        }
    };
    let source = t.volume_range.as_ref().map_or("none".to_string(), |r| r.source.clone());
    match curve {
        Curve::Cubic => println!("{}: law cubic (60*log10 s), range source {source}", t.aid),
        Curve::Hal { .. } => println!("{}: law HAL 2-over-1 over the bundle range, source {source}", t.aid),
    }
    for item in d.items.iter().filter(|i| i.code.starts_with("hal-volume")) {
        println!("  diag {:?} {} {}: {}", item.severity, item.code, item.context, item.message);
    }
    let mut fail = false;
    for m in &measured {
        let text = std::fs::read_to_string(m)?;
        let (mut worst, mut worst_cubic, mut n) = (0.0f64, 0.0f64, 0);
        for line in text.lines().filter(|l| !l.trim().is_empty() && !l.starts_with('#')) {
            let mut f = line.split_whitespace().map(str::parse::<f64>);
            let (Some(Ok(s)), Some(Ok(db))) = (f.next(), f.next()) else {
                return Err(format!("{m}: bad line {line:?}").into());
            };
            if s <= 0.0 {
                continue; // macOS mutes at 0; the plugin silences it too
            }
            let e = curve.db(s) - db;
            let ec = Curve::Cubic.db(s) - db;
            println!("  {m}: s {s:.4} macOS {db:9.4} law {:9.4} err {e:+.5} | cubic err {ec:+.3}", curve.db(s));
            worst = worst.max(e.abs());
            worst_cubic = worst_cubic.max(ec.abs());
            n += 1;
        }
        let ok = worst <= tol;
        fail |= !ok;
        println!(
            "{} {m}: {n} points, max |law - macOS| {worst:.5} dB (tolerance {tol}), cubic would be {worst_cubic:.3} dB",
            if ok { "PASS" } else { "FAIL" }
        );
    }
    if fail {
        std::process::exit(1);
    }
    Ok(())
}
