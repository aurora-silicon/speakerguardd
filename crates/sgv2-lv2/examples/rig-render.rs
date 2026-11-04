//! Render a unit offline over an interleaved f32 program file and write one
//! trace line per slice in the rig's schema (tools/diffrig/schema.json).
//! Usage: `rig-render <blob> <program.f32> <channels> <case> [slice] [tele.f32 tele_channels]`.

use std::io::Write;

use sgv2_core::{Io, ProtectionUnit, Telemetry};

fn read_f32(path: &str) -> Vec<f32> {
    let b = std::fs::read(path).unwrap();
    b.chunks_exact(4).map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()
}

fn num(v: f32) -> String {
    if v.is_finite() {
        format!("{v}")
    } else {
        "null".into()
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let blob = std::fs::read(&a[1]).unwrap();
    let prog = read_f32(&a[2]);
    let n: usize = a[3].parse().unwrap();
    let case = &a[4];
    let slice: usize = a.get(5).and_then(|v| v.parse().ok()).unwrap_or(512);
    let tele = a.get(6).map(|p| read_f32(p));
    let tn: usize = a.get(7).and_then(|v| v.parse().ok()).unwrap_or(0);
    let p = sgv2_core::spp3::Params::from_blob(&blob).unwrap_or_else(|e| panic!("{e:?}"));
    let mut u = sgv2_core::spp3::Unit::new(p);
    let frames = prog.len() / n;
    let mut ins = vec![vec![0.0f32; slice]; n];
    let mut outs = vec![vec![0.0f32; slice]; n];
    let mut sense = vec![vec![0.0f32; slice]; tn];
    let mut t = Telemetry::new();
    let stdout = std::io::stdout();
    let mut w = std::io::BufWriter::new(stdout.lock());
    let mut f = 0;
    while f < frames {
        let m = slice.min(frames - f);
        for k in 0..m {
            for c in 0..n {
                ins[c][k] = prog[(f + k) * n + c];
            }
            if let Some(tv) = &tele {
                for c in 0..tn {
                    sense[c][k] = tv[(f + k) * tn + c];
                }
            }
        }
        let i: Vec<&[f32]> = ins.iter().map(|b| &b[..m]).collect();
        let s: Vec<&[f32]> = sense.iter().map(|b| &b[..m]).collect();
        let mut o: Vec<&mut [f32]> = outs.iter_mut().map(|b| &mut b[..m]).collect();
        let h = u.process(Io { frames: m, input: &i, sense: &s, output: &mut o });
        u.telemetry(&mut t);
        let chans: Vec<String> = (0..n)
            .map(|c| {
                let s = &t.channels[c];
                // audio gain over the slice, from the samples themselves
                let (num_, den) = (0..m).fold((0.0f64, 0.0f64), |(a, b), k| {
                    (a + (outs[c][k] as f64) * (ins[c][k] as f64), b + (ins[c][k] as f64).powi(2))
                });
                let ag = if den > 0.0 { (num_ / den) as f32 } else { f32::NAN };
                format!(
                    "{{\"gain\":{},\"audio_gain\":{},\"ol_temp_c\":{},\"coil_temp_c\":{},\"magnet_temp_c\":{},\"modeled_power_w\":{},\"cl_temp_c\":{}}}",
                    num(s.gain), num(ag), num(s.ol_temp_c), num(s.coil_temp_c), num(s.magnet_temp_c), num(s.modeled_power_w), num(s.cl_temp_c)
                )
            })
            .collect();
        writeln!(
            w,
            "{{\"schema\":1,\"side\":\"sgv2\",\"unit\":\"spp3\",\"case\":\"{case}\",\"frame\":{f},\"frames\":{m},\"latency\":{},\"mode\":{{\"failsafe\":{}}},\"channels\":[{}]}}",
            u.latency_frames(),
            h.is_failsafe(),
            chans.join(",")
        )
        .unwrap();
        f += m;
    }
}
