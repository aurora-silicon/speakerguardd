//! `sgv2-bench`: wall-clock cost of each unit's `process`, per quantum and
//! per channel per quantum, at the quantum sizes PipeWire runs.
//!
//! Parameters are synthetic (no Mac's tuning): the cost of spp3 does not
//! depend on the values, only on the channel count and the code path, and
//! the drive is chosen so the limiter is engaged (the slowest path).
//!
//! Usage: `sgv2-bench [seconds-per-case] [--json]`.

use std::time::Instant;

use sgv2_core::{placeholder, spp3, FailSafe, Health, Io, ProtectionUnit, UnitKind};

const RATE: u32 = 48_000;

fn spp3_params(n: usize) -> spp3::Params {
    let mut p = spp3::Params {
        channels: n as u32,
        sample_rate: RATE,
        block_frames: 512,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 1.0,
        thermal_gain_release_time_s: 5.0,
        thermal_lookahead_delay_ms: 2.0,
        thermal_protection_enabled: 1,
        ..spp3::Params::default()
    };
    for c in 0..n {
        p.ch[c] = spp3::ChannelParams {
            source: c as u32,
            group: (c / 4) as u32,
            t_sett_vc: 20.0,
            t_sett_mg: 20.0,
            tau_tvc: 1.0,
            tau_tmg: 50.0,
            temperature_limit: 100.0,
            hard_temp_limit_headroom: 5.0,
            reb_ref: 4.0,
            rshunt: 0.1,
            ag: 10.0,
            thermal_ff_speedup_factor: 0.5,
            imon_scale: 1.0,
            vmon_scale: 10.0,
            sense_i: 2 * c as u32,
            sense_v: 2 * c as u32 + 1,
            ..Default::default()
        };
    }
    p
}

fn noise(n: usize, seed: u32, level: f32) -> Vec<f32> {
    let mut s = seed;
    (0..n)
        .map(|_| {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            ((s >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0) * level
        })
        .collect()
}

struct Result {
    unit: &'static str,
    channels: usize,
    sense: usize,
    quantum: usize,
    calls: usize,
    p50_ns: f64,
    p99_ns: f64,
    max_ns: f64,
    mean_ns: f64,
}

fn bench(name: &'static str, unit: &mut dyn ProtectionUnit, n: usize, s: usize, q: usize, seconds: f64) -> Result {
    let len = q * 64;
    let input: Vec<Vec<f32>> = (0..n).map(|c| noise(len, 11 + c as u32, 0.9)).collect();
    let sense: Vec<Vec<f32>> = (0..s).map(|k| noise(len, 101 + k as u32, 0.3)).collect();
    let mut output = vec![vec![0.0f32; q]; n];
    let calls = ((seconds * RATE as f64) / q as f64) as usize;
    let mut t = Vec::with_capacity(calls);
    // warm-up: one second of audio, so the limiter is engaged
    for k in 0..(RATE as usize / q) {
        let off = (k % 64) * q;
        let ins: Vec<&[f32]> = input.iter().map(|b| &b[off..off + q]).collect();
        let sns: Vec<&[f32]> = sense.iter().map(|b| &b[off..off + q]).collect();
        let mut outs: Vec<&mut [f32]> = output.iter_mut().map(|b| &mut b[..]).collect();
        unit.process(Io { frames: q, input: &ins, sense: &sns, output: &mut outs });
    }
    for k in 0..calls {
        let off = (k % 64) * q;
        let ins: Vec<&[f32]> = input.iter().map(|b| &b[off..off + q]).collect();
        let sns: Vec<&[f32]> = sense.iter().map(|b| &b[off..off + q]).collect();
        let mut outs: Vec<&mut [f32]> = output.iter_mut().map(|b| &mut b[..]).collect();
        let t0 = Instant::now();
        let h: Health = unit.process(Io { frames: q, input: &ins, sense: &sns, output: &mut outs });
        let dt = t0.elapsed().as_nanos() as f64;
        std::hint::black_box(h);
        std::hint::black_box(&output);
        t.push(dt);
    }
    let mean = t.iter().sum::<f64>() / t.len() as f64;
    t.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let pct = |p: f64| t[((t.len() as f64 - 1.0) * p) as usize];
    Result {
        unit: name,
        channels: n,
        sense: s,
        quantum: q,
        calls,
        p50_ns: pct(0.5),
        p99_ns: pct(0.99),
        max_ns: *t.last().unwrap(),
        mean_ns: mean,
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let seconds: f64 = args.iter().skip(1).find_map(|a| a.parse().ok()).unwrap_or(30.0);
    let json = args.iter().any(|a| a == "--json");
    let mut results = Vec::new();
    for q in [256usize, 512, 1024] {
        let mut u = spp3::Unit::new(spp3_params(6));
        assert!(!u.is_failsafe());
        results.push(bench("spp3", &mut u, 6, 12, q, seconds));
        let mut u = spp3::Unit::new(spp3_params(6));
        results.push(bench("spp3-no-sense", &mut u, 6, 0, q, seconds));
        let mut u = spp3::Unit::new(spp3_params(2));
        results.push(bench("spp3", &mut u, 2, 4, q, seconds));
        let mut u = placeholder::Unit::new(placeholder::Params { channels: 6, ..Default::default() });
        results.push(bench("placeholder", &mut u, 6, 12, q, seconds));
        let mut u = FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS);
        results.push(bench("failsafe", &mut u, 6, 0, q, seconds));
    }
    if json {
        for r in &results {
            println!(
                "{{\"unit\":\"{}\",\"channels\":{},\"sense\":{},\"quantum\":{},\"calls\":{},\"mean_ns\":{:.0},\"p50_ns\":{:.0},\"p99_ns\":{:.0},\"max_ns\":{:.0},\"per_channel_p50_ns\":{:.0},\"cpu_pct_of_realtime\":{:.4}}}",
                r.unit, r.channels, r.sense, r.quantum, r.calls, r.mean_ns, r.p50_ns, r.p99_ns, r.max_ns,
                r.p50_ns / r.channels as f64,
                100.0 * r.mean_ns / (r.quantum as f64 / RATE as f64 * 1e9)
            );
        }
        return;
    }
    println!(
        "{:<14} {:>3} {:>5} {:>7} {:>9} {:>9} {:>9} {:>9} {:>11} {:>9}",
        "unit", "ch", "sense", "quantum", "mean ns", "p50 ns", "p99 ns", "max ns", "p50 ns/ch", "% RT"
    );
    for r in &results {
        println!(
            "{:<14} {:>3} {:>5} {:>7} {:>9.0} {:>9.0} {:>9.0} {:>9.0} {:>11.0} {:>9.4}",
            r.unit,
            r.channels,
            r.sense,
            r.quantum,
            r.mean_ns,
            r.p50_ns,
            r.p99_ns,
            r.max_ns,
            r.p50_ns / r.channels as f64,
            100.0 * r.mean_ns / (r.quantum as f64 / RATE as f64 * 1e9)
        );
    }
}
