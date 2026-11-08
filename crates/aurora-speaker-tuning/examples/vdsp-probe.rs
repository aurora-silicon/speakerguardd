//! Differential tool: run a cascade through sgv2-core's model of the
//! platform's Float32 multichannel biquad, for comparison with the platform.
//!
//! usage: vdsp-probe COEFS.txt IN.f32 OUT.f32 CHANNELS CALLLEN...
//! COEFS.txt: one "b0 b1 b2 a1 a2" line per section (same on every channel);
//! IN/OUT interleaved Float32; call lengths repeat until the input ends.
use sgv2_core::vdsp::Cascade;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 6 {
        eprintln!("usage: vdsp-probe COEFS.txt IN.f32 OUT.f32 CHANNELS CALLLEN...");
        std::process::exit(2);
    }
    let coefs: Vec<[f64; 5]> = std::fs::read_to_string(&a[1])
        .expect("coefs")
        .lines()
        .filter_map(|l| {
            let v: Vec<f64> = l.split_whitespace().filter_map(|t| t.parse().ok()).collect();
            (v.len() == 5).then(|| [v[0], v[1], v[2], v[3], v[4]])
        })
        .collect();
    let ch: usize = a[4].parse().expect("channels");
    let calls: Vec<usize> = a[5..].iter().map(|s| s.parse().expect("call")).collect();
    let raw = std::fs::read(&a[2]).expect("input");
    let x: Vec<f32> = raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    let n = x.len() / ch;
    let mut casc = vec![Cascade::new(&coefs, ch > 1); ch];
    let mut out = vec![0.0f32; n * ch];
    let (mut pos, mut ci) = (0, 0);
    while pos < n {
        let len = calls[ci % calls.len()].min(n - pos);
        ci += 1;
        for (c, cs) in casc.iter_mut().enumerate() {
            let mut buf: Vec<f32> = (0..len).map(|i| x[(pos + i) * ch + c]).collect();
            cs.run(&mut buf);
            for (i, v) in buf.iter().enumerate() {
                out[(pos + i) * ch + c] = *v;
            }
        }
        pos += len;
    }
    let bytes: Vec<u8> = out.iter().flat_map(|v| v.to_le_bytes()).collect();
    std::fs::write(&a[3], bytes).expect("output");
}
