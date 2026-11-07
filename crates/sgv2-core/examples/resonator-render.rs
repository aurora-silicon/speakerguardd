//! File-only renderer for the standalone resonator component. It never opens
//! an audio device and is not a protection controller.
use sgv2_core::resonator::{Filter, Params};
use std::io::{Read, Write};

fn run() -> Result<(), String> {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.len() != 11 {
        return Err("usage: resonator-render Rdc Bl MassKg Damping Stiffness Hz Q MaxBoostDb MaxCutDb Rate ChunkFrames; mono f32le stdin/stdout".into());
    }
    let v: Vec<f64> =
        a[..10].iter().map(|s| s.parse().map_err(|_| "invalid number".to_owned())).collect::<Result<_, _>>()?;
    let chunk: usize = a[10].parse().map_err(|_| "invalid chunk size")?;
    if chunk == 0 {
        return Err("zero chunk size".into());
    }
    let params = Params {
        rdc: v[0],
        bl: v[1],
        mass_kg: v[2],
        damping: v[3],
        stiffness: v[4],
        desired_hz: v[5],
        desired_q: v[6],
        max_boost_db: v[7],
        max_cut_db: v[8],
        rate: v[9],
    };
    let mut filter = Filter::new(params).map_err(|e| format!("refused: {e:?}"))?;
    let mut bytes = Vec::new();
    std::io::stdin().read_to_end(&mut bytes).map_err(|e| e.to_string())?;
    if bytes.len() % 4 != 0 {
        return Err("truncated f32le input".into());
    }
    let original: Vec<f32> = bytes.chunks_exact(4).map(|b| f32::from_le_bytes(b.try_into().unwrap())).collect();
    let mut samples = original.clone();
    for block in samples.chunks_mut(chunk) {
        filter.process(block).map_err(|e| format!("refused: {e:?}"))?;
    }
    // Reset is part of the file-only check: the same stream must reproduce
    // exactly even if the caller chooses a different block size.
    filter.reset();
    let mut replay = original;
    for block in replay.chunks_mut(1 + chunk % 113) {
        filter.process(block).map_err(|e| format!("refused after reset: {e:?}"))?;
    }
    if samples.iter().zip(&replay).any(|(a, b)| a.to_bits() != b.to_bits()) {
        return Err("reset or block-size invariance failed".into());
    }
    let mut invalid = [0.0, f32::NAN, 1.0];
    if filter.process(&mut invalid).is_ok() || invalid.iter().any(|v| *v != 0.0) {
        return Err("nonfinite input did not fail closed".into());
    }
    let mut latched = [1.0];
    if filter.process(&mut latched).is_ok() || latched[0] != 0.0 {
        return Err("fault did not remain latched".into());
    }
    filter.reset();
    let mut quiet = [0.0; 17];
    if filter.process(&mut quiet).is_err() || quiet.iter().any(|v| *v != 0.0) {
        return Err("fault reset did not restore quiet operation".into());
    }
    let output: Vec<u8> = samples.iter().flat_map(|v| v.to_le_bytes()).collect();
    std::io::stdout().write_all(&output).map_err(|e| e.to_string())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}
