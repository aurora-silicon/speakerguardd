//! Rig diagnostics: the per-update controller state of spp3 channels, for
//! comparison with the reference's `Reb_Data` records (`clsp.npz` `ctl`).
//!
//! `spp3-dump BLOB PROGRAM.f32 CH[,CH..] [SENSE.f32 [RESET_FRAME,..]]` renders in
//! 256-frame calls (one thermal update each) and writes to stdout, per
//! update and per requested channel in order, the 11 f64 values of
//! `spp3::Unit::controller_state`.
use std::io::Write;

use sgv2_core::blob::Header;
use sgv2_core::{Io, ProtectionUnit};

fn rd(p: &str) -> Vec<f32> {
    std::fs::read(p).unwrap().chunks_exact(4).map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let blob = std::fs::read(&a[1]).unwrap();
    let (h, _) = Header::parse(&blob).unwrap();
    let p = sgv2_core::spp3::Params::from_blob(&blob).unwrap_or_else(|e| panic!("{e:?}"));
    let n = p.channels as usize;
    let sn = h.sense_channels as usize;
    let prog = rd(&a[2]);
    let chs: Vec<usize> = a[3].split(',').map(|x| x.parse().unwrap()).collect();
    let sense = a.get(4).map(|s| rd(s));
    let resets: Vec<usize> = a
        .get(5)
        .map(|s| s.split(',').filter(|x| !x.is_empty()).map(|x| x.parse().unwrap()).collect())
        .unwrap_or_default();
    let mut u = sgv2_core::spp3::Unit::new(p);
    let frames = prog.len() / n;
    let q = sgv2_core::spp3::THERMAL_FRAMES;
    let mut ins = vec![vec![0f32; q]; n];
    let mut ss = vec![vec![0f32; q]; sn];
    let mut outs = vec![vec![0f32; q]; n];
    let mut w = std::io::BufWriter::new(std::io::stdout().lock());
    let mut f = 0;
    while f + q <= frames {
        if resets.contains(&f) {
            u.host_reset();
        }
        for k in 0..q {
            for (c, b) in ins.iter_mut().enumerate() {
                b[k] = prog[(f + k) * n + c];
            }
            if let Some(s) = &sense {
                for (c, b) in ss.iter_mut().enumerate() {
                    b[k] = s.get((f + k) * sn + c).copied().unwrap_or(0.0);
                }
            }
        }
        let i: Vec<&[f32]> = ins.iter().map(|b| &b[..]).collect();
        let s: Vec<&[f32]> = if sense.is_some() { ss.iter().map(|b| &b[..]).collect() } else { vec![] };
        let mut o: Vec<&mut [f32]> = outs.iter_mut().map(|b| &mut b[..]).collect();
        u.process(Io { frames: q, input: &i, sense: &s, output: &mut o });
        for &c in &chs {
            for v in u.controller_state(c) {
                w.write_all(&v.to_le_bytes()).unwrap();
            }
        }
        f += q;
    }
}
