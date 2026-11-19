//! Dump the spp3 controller state after every thermal update, in the
//! reference's `Reb_Data` field order, for a differential against the
//! macOS unit's bit-8 debug log.
//!
//! `spp3-state-dump BLOB FEED.f32 CHANNELS OUT.f32 [SLICE] [AUDIO.f32]`
//!
//! OUT gets, per update and channel, 11 Float64 values
//! ([`sgv2_core::spp3::Unit::controller_state`]). Both inputs carry Mac
//! tuning: keep them out of git.
use sgv2_core::spp3::{Params, Unit};
use sgv2_core::{Io, ProtectionUnit};
use std::io::Write;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 5 {
        eprintln!("usage: spp3-state-dump BLOB FEED.f32 CHANNELS OUT.f64 [SLICE] [AUDIO.f32]");
        std::process::exit(2);
    }
    let p = Params::from_blob(&std::fs::read(&a[1]).expect("blob")).expect("params");
    let nch: usize = a[3].parse().expect("channels");
    for c in 0..nch {
        let q = &p.ch[c];
        eprintln!("ch{c} ag {:e} reb {:e} rshunt {:e} rampout {:e} tsvc {:e} tsmg {:e} tauvc {:e} taumg {:e} ff {:e} tlim {:e} amb {:e}", q.ag, q.reb_ref, q.rshunt, q.rampout, q.t_sett_vc, q.t_sett_mg, q.tau_tvc, q.tau_tmg, q.thermal_ff_speedup_factor, q.temperature_limit, p.thermal_ambient_temp);
    }
    let slice: usize = a.get(5).map_or(256, |s| s.parse().expect("slice"));
    let raw = std::fs::read(&a[2]).expect("feed");
    let x: Vec<f32> = raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    let frames = x.len() / nch;
    let prog: Vec<Vec<f32>> = (0..nch).map(|c| (0..frames).map(|i| x[i * nch + c]).collect()).collect();
    let mut u = Unit::new(p);
    let mut st = std::io::BufWriter::new(std::fs::File::create(&a[4]).expect("out"));
    let mut audio = vec![0f32; frames * nch];
    let mut buf = vec![vec![0f32; slice]; nch];
    let mut pos = 0;
    while pos < frames {
        let n = slice.min(frames - pos);
        let ins: Vec<&[f32]> = prog.iter().map(|c| &c[pos..pos + n]).collect();
        {
            let mut outs: Vec<&mut [f32]> = buf.iter_mut().map(|c| &mut c[..n]).collect();
            u.process(Io { frames: n, input: &ins, sense: &[], output: &mut outs });
        }
        for c in 0..nch {
            for i in 0..n {
                audio[(pos + i) * nch + c] = buf[c][i];
            }
            for v in u.controller_state(c) {
                st.write_all(&v.to_le_bytes()).unwrap();
            }
        }
        pos += n;
    }
    if let Some(path) = a.get(6) {
        let mut f = std::io::BufWriter::new(std::fs::File::create(path).expect("audio"));
        for v in audio {
            f.write_all(&v.to_le_bytes()).unwrap();
        }
    }
}
