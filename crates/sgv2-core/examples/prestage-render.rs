//! Render a parameter blob's pre-stage (filters and dynamics) offline.
//!
//!     prestage-render BLOB IN.f32 OUT.f32
//!
//! IN: interleaved f32 with the blob's channel count; OUT: the same layout.
//! For the differential rig; never part of the plugin.
use sgv2_core::prestage::PreStage;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() != 4 {
        eprintln!("usage: prestage-render BLOB IN.f32 OUT.f32");
        std::process::exit(2);
    }
    let blob = std::fs::read(&a[1]).expect("blob");
    let (h, _) = sgv2_core::blob::Header::parse(&blob).expect("header");
    let fs = h.sample_rate as f64;
    let p = sgv2_core::blob::prestage(&blob, fs).expect("pre-stage");
    let n = p.channels;
    let raw = std::fs::read(&a[2]).expect("input");
    let x: Vec<f32> = raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    let frames = x.len() / n;
    let mut pre = PreStage::with_rate(p, fs);
    let mut out = vec![0.0f32; x.len()];
    const Q: usize = 512;
    let mut outs = vec![vec![0.0f32; Q]; n];
    let mut at = 0;
    while at < frames {
        let k = (frames - at).min(Q);
        for c in 0..n {
            for i in 0..k {
                outs[c][i] = x[(at + i) * n + c];
            }
        }
        let mut bufs: Vec<&mut [f32]> = outs.iter_mut().map(|o| &mut o[..k]).collect();
        pre.process_block(&mut bufs, k);
        for c in 0..n {
            for i in 0..k {
                out[(at + i) * n + c] = outs[c][i];
            }
        }
        at += k;
    }
    let bytes: Vec<u8> = out.iter().flat_map(|v| v.to_le_bytes()).collect();
    std::fs::write(&a[3], bytes).expect("output");
}
