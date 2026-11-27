// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Offline parity of the microphone chain against renders of the same
//! input by macOS's own graph engine.
//!
//! ```text
//! mic-parity chain --tunings-root DIR --aid N --stim IN.f32 --oracle REF.f32
//!            [--out OUT.f32] [--window sine|vorbis] [--half-offset] [--frame-phase P]
//!            [--beam-eq] [--drop-tone-types 6,20]
//! mic-parity probe --kind id|delay|bin48 --stim IN.f32 --oracle REF.f32 [transform options]
//! ```
//!
//! `chain` compiles the AID's microphone chain (`aurora_speaker_tuning::mic`)
//! and renders `IN.f32` (interleaved float32, one channel per microphone);
//! `probe` renders a one-microphone transform with a synthetic beam (unit,
//! a 100-sample delay, or bin 48 alone). Both print one JSON object:
//! latency-aligned difference SNR, output levels, and for `chain` the
//! 1/3-octave magnitude deviation. `--drop-tone-types` switches off filter
//! slots of the listed types (development analysis: such a chain is refused
//! for a blob). No input or reference data is part of this repository.

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::mic;
use aurora_speaker_tuning::Diagnostics;
use sgv2_core::mic::fft::{Complex, Fft};
use sgv2_core::mic::{Beam, Engine, Params, Stft, Window};
use std::path::PathBuf;

/// Widest latency difference searched, frames.
const MAX_LAG: i64 = 1200;

fn usage() -> ! {
    eprintln!(
        "usage: mic-parity chain --tunings-root DIR --aid N --stim IN.f32 --oracle REF.f32 [--out OUT.f32] [opts]\n       \
         mic-parity probe --kind id|delay|bin48 --stim IN.f32 --oracle REF.f32 [opts]\n       \
         opts: --window sine|vorbis --half-offset --frame-phase P --beam-eq --drop-tone-types A,B"
    );
    std::process::exit(2)
}

fn read_f32(p: &PathBuf) -> Vec<f32> {
    let b = std::fs::read(p).unwrap_or_else(|e| {
        eprintln!("{}: {e}", p.display());
        std::process::exit(1)
    });
    b.chunks_exact(4).map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()
}

fn render(e: &mut Engine, x: &[Vec<f32>]) -> Vec<f32> {
    let n = x[0].len();
    let mut y = vec![0.0f32; n];
    let mut at = 0;
    while at < n {
        let len = 256.min(n - at);
        let ins: Vec<&[f32]> = x.iter().map(|c| &c[at..at + len]).collect();
        e.process(&ins, &mut y[at..at + len], len);
        at += len;
    }
    y
}

fn rms_db(x: &[f64]) -> f64 {
    let p = x.iter().map(|v| v * v).sum::<f64>() / x.len().max(1) as f64;
    10.0 * p.max(1e-30).log10()
}

/// The lag (ours later by `lag`, negative: earlier) in `-max..=max` with
/// the largest absolute correlation, over the oracle's most energetic
/// window of at most 96000 frames.
fn align(ours: &[f32], oracle: &[f32], max: i64) -> i64 {
    let n = ours.len().min(oracle.len()) as i64;
    let win = 96_000.min(n - 2 * max).max(1);
    // the window start with the most oracle energy, on a coarse grid
    let mut start = max;
    let mut best_e = -1.0;
    let mut s = max;
    while s + win <= n - max {
        let e: f64 = (s..s + win).step_by(16).map(|i| f64::from(oracle[i as usize]).powi(2)).sum();
        if e > best_e {
            best_e = e;
            start = s;
        }
        s += win / 4 + 1;
    }
    let mut best = (f64::MIN, 0i64);
    for lag in -max..=max {
        let mut acc = 0.0f64;
        for i in start..start + win {
            acc += f64::from(oracle[i as usize]) * f64::from(ours[(i + lag) as usize]);
        }
        if acc.abs() > best.0 {
            best = (acc.abs(), lag);
        }
    }
    best.1
}

/// 1/3-octave band powers (Hann-windowed 4096-point frames, 50% overlap).
fn third_octaves(x: &[f64], fs: f64) -> Vec<(f64, f64)> {
    const N: usize = 4096;
    let mut fft = Fft::new(N).unwrap();
    let w: Vec<f64> = (0..N).map(|i| 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / N as f64).cos()).collect();
    let mut psd = vec![0.0f64; N / 2 + 1];
    let mut buf = vec![Complex::ZERO; N];
    let mut at = 0;
    while at + N <= x.len().max(N) {
        for (i, b) in buf.iter_mut().enumerate() {
            *b = Complex::new(x.get(at + i).copied().unwrap_or(0.0) * w[i], 0.0);
        }
        fft.forward(&mut buf);
        for (k, p) in psd.iter_mut().enumerate() {
            *p += buf[k].re * buf[k].re + buf[k].im * buf[k].im;
        }
        at += N / 2;
    }
    let mut out = Vec::new();
    let mut fc = 50.0f64;
    while fc <= 16_000.0 {
        let (lo, hi) = (fc / 2f64.powf(1.0 / 6.0), fc * 2f64.powf(1.0 / 6.0));
        let p: f64 = psd
            .iter()
            .enumerate()
            .filter(|(k, _)| {
                let f = *k as f64 * fs / N as f64;
                f >= lo && f < hi
            })
            .map(|(_, v)| v)
            .sum();
        out.push((fc, p));
        fc *= 2f64.powf(1.0 / 3.0);
    }
    out
}

struct Opt {
    cmd: String,
    root: Option<PathBuf>,
    aid: Option<u32>,
    stim: Option<PathBuf>,
    oracle: Option<PathBuf>,
    out: Option<PathBuf>,
    kind: String,
    mopts: mic::Options,
}

fn parse() -> Opt {
    let mut it = std::env::args().skip(1);
    let cmd = it.next().unwrap_or_else(|| usage());
    let mut o = Opt {
        cmd,
        root: None,
        aid: None,
        stim: None,
        oracle: None,
        out: None,
        kind: "id".into(),
        mopts: mic::Options { allow_partial: true, ..Default::default() },
    };
    while let Some(a) = it.next() {
        let mut v = || it.next().unwrap_or_else(|| usage());
        match a.as_str() {
            "--tunings-root" => o.root = Some(v().into()),
            "--aid" => o.aid = v().trim_start_matches("AID").parse().ok(),
            "--stim" => o.stim = Some(v().into()),
            "--oracle" => o.oracle = Some(v().into()),
            "--out" => o.out = Some(v().into()),
            "--kind" => o.kind = v(),
            "--window" => {
                o.mopts.window = match v().as_str() {
                    "sine" => Window::Sine,
                    "vorbis" => Window::Vorbis,
                    _ => usage(),
                }
            }
            "--half-offset" => o.mopts.half_offset = true,
            "--frame-phase" => o.mopts.frame_phase = v().parse().unwrap_or_else(|_| usage()),
            "--beam-eq" => o.mopts.beam_eq = true,
            "--drop-tone-types" => {
                for t in v().split(',') {
                    let t: u32 = t.parse().unwrap_or_else(|_| usage());
                    o.mopts.dev_drop_tone_types |= 1 << t;
                }
            }
            _ => usage(),
        }
    }
    o
}

fn probe_params(kind: &str, m: &mic::Options) -> Params {
    let (hop, bins) = (480usize, 481usize);
    let weights = (0..bins)
        .map(|k| match kind {
            "id" => [1.0, 0.0],
            "delay" => {
                let a = -2.0 * std::f64::consts::PI * k as f64 * 100.0 / 960.0;
                [a.cos(), a.sin()]
            }
            "bin48" => [if k == 48 { 1.0 } else { 0.0 }, 0.0],
            _ => usage(),
        })
        .collect();
    Params {
        sample_rate: 48_000,
        mics: 1,
        pre: Vec::new(),
        stft: Stft { hop, size: 2 * hop, window: m.window, half_offset: m.half_offset, frame_phase: m.frame_phase },
        beam: Beam { mics: 1, bins, weights, eq: None },
        post: Vec::new(),
        partial: false,
    }
}

fn main() {
    let o = parse();
    let (stim, oracle) = (o.stim.clone().unwrap_or_else(|| usage()), o.oracle.clone().unwrap_or_else(|| usage()));
    let mut report = serde_json::Map::new();
    let params = match o.cmd.as_str() {
        "chain" => {
            let b =
                Bundle::from_tunings_root(&o.root.clone().unwrap_or_else(|| usage()), o.aid.unwrap_or_else(|| usage()))
                    .unwrap_or_else(|e| {
                        eprintln!("{e}");
                        std::process::exit(1)
                    });
            let mut d = Diagnostics::default();
            let c = mic::compile(&b, &o.mopts, &mut d).unwrap_or_else(|e| {
                eprintln!("{e}");
                std::process::exit(1)
            });
            report.insert("refusals".into(), serde_json::json!(c.report.refusals));
            report.insert(
                "stages".into(),
                serde_json::json!(c
                    .report
                    .stages
                    .iter()
                    .map(|s| format!("{}:{:?}", s.name, s.status))
                    .collect::<Vec<_>>()),
            );
            c.params.unwrap_or_else(|| {
                eprintln!("no chain: {:?}", c.report.refusals);
                std::process::exit(1)
            })
        }
        "probe" => probe_params(&o.kind, &o.mopts),
        // the chain's post-synthesis stages alone, on a mono input
        "post" => {
            let b =
                Bundle::from_tunings_root(&o.root.clone().unwrap_or_else(|| usage()), o.aid.unwrap_or_else(|| usage()))
                    .unwrap_or_else(|e| {
                        eprintln!("{e}");
                        std::process::exit(1)
                    });
            let mut d = Diagnostics::default();
            let c = mic::compile(&b, &o.mopts, &mut d).unwrap();
            let mut p = probe_params("id", &o.mopts);
            p.post = c.params.expect("no chain").post;
            let x = read_f32(&stim);
            let mut e = Engine::new(p).unwrap();
            let mut y = render(&mut e, &[x]);
            y.drain(..959);
            if let Some(out) = &o.out {
                let bytes: Vec<u8> = y.iter().flat_map(|v| v.to_le_bytes()).collect();
                std::fs::write(out, bytes).unwrap();
            }
            return;
        }
        _ => usage(),
    };
    let mics = params.mics;
    let raw = read_f32(&stim);
    let frames = raw.len() / mics;
    let x: Vec<Vec<f32>> = (0..mics).map(|m| (0..frames).map(|i| raw[i * mics + m]).collect()).collect();
    let mut e = Engine::new(params).unwrap_or_else(|| {
        eprintln!("the engine refuses the parameters");
        std::process::exit(1)
    });
    let y = render(&mut e, &x);
    if let Some(p) = &o.out {
        let bytes: Vec<u8> = y.iter().flat_map(|v| v.to_le_bytes()).collect();
        std::fs::write(p, bytes).unwrap();
    }
    let r = read_f32(&oracle);
    let n = y.len().min(r.len());
    let lag = align(&y[..n], &r[..n], MAX_LAG);
    // ours shifted by the lag, both over the common span
    let skip = MAX_LAG as usize;
    let idx: Vec<usize> = (skip..n - skip).collect();
    let ours: Vec<f64> = idx.iter().map(|&i| f64::from(y[(i as i64 + lag) as usize])).collect();
    let refr: Vec<f64> = idx.iter().map(|&i| f64::from(r[i])).collect();
    let diff: Vec<f64> = ours.iter().zip(&refr).map(|(a, b)| a - b).collect();
    let (lo, lr, ld) = (rms_db(&ours), rms_db(&refr), rms_db(&diff));
    let peak_ref = refr.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let peak_err = diff.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    // the best single gain, to separate level from shape
    let g =
        ours.iter().zip(&refr).map(|(a, b)| a * b).sum::<f64>() / ours.iter().map(|a| a * a).sum::<f64>().max(1e-30);
    let diff_g: Vec<f64> = ours.iter().zip(&refr).map(|(a, b)| g * a - b).collect();
    report.insert("lag".into(), serde_json::json!(lag));
    report.insert("level_ours_dbfs".into(), serde_json::json!(lo));
    report.insert("level_oracle_dbfs".into(), serde_json::json!(lr));
    report.insert("level_diff_db".into(), serde_json::json!(lo - lr));
    report.insert("diff_snr_db".into(), serde_json::json!(lr - ld));
    report.insert("diff_snr_gain_fit_db".into(), serde_json::json!(lr - rms_db(&diff_g)));
    report.insert("gain_fit_db".into(), serde_json::json!(20.0 * g.abs().max(1e-30).log10()));
    report.insert("max_abs_err_rel".into(), serde_json::json!(peak_err / peak_ref.max(1e-30)));
    if o.cmd == "chain" {
        let (bo, br) = (third_octaves(&ours, 48_000.0), third_octaves(&refr, 48_000.0));
        let bands: Vec<(f64, f64)> = bo
            .iter()
            .zip(&br)
            .filter(|(a, b)| a.1 > 0.0 && b.1 > 0.0)
            .map(|(a, b)| ((a.0 * 10.0).round() / 10.0, 10.0 * (a.1 / b.1).log10()))
            .collect();
        let worst = bands.iter().fold(0.0f64, |m, b| m.max(b.1.abs()));
        let mean_abs = bands.iter().map(|b| b.1.abs()).sum::<f64>() / bands.len().max(1) as f64;
        report.insert("third_octave_max_abs_dev_db".into(), serde_json::json!(worst));
        report.insert("third_octave_mean_abs_dev_db".into(), serde_json::json!(mean_abs));
        report.insert("third_octave_dev_db".into(), serde_json::json!(bands));
    }
    println!("{}", serde_json::Value::Object(report));
}
