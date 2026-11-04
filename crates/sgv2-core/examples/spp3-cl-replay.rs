//! Replay logged engine telemetry through the spp3 closed-loop estimator,
//! limit smoother and plausibility guard, with the relaxation on or off.
//!
//! `spp3-cl-replay BLOB TELEMETRY.csv on|off [CHANNELS]`
//!
//! The CSV is the engine-watch log (one row per 512-frame block): `frame`,
//! then per channel `pmod<c>` (drive, W), `tol<c>`, `tcl<c>`, `tadj<c>`,
//! `vms<c>`/`ims<c>` (the scaled pilot-band powers of the last Goertzel
//! period). A new period is a change of (vms, ims). Prints, per channel,
//! the replayed TlimAdj and T_CL, the guard verdict, and the agreement
//! with the logged T_CL. Both files carry Mac tuning: keep them out of git.
use sgv2_core::spp3::{thermal_frames, ClReplay, Params};
use std::collections::HashMap;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 4 {
        eprintln!("usage: spp3-cl-replay BLOB TELEMETRY.csv on|off [CH,..]");
        std::process::exit(2);
    }
    let blob = std::fs::read(&a[1]).expect("blob");
    let p = Params::from_blob(&blob).expect("params");
    let relax = a[3] == "on";
    let text = std::fs::read_to_string(&a[2]).expect("csv");
    let mut lines = text.lines();
    let head: HashMap<&str, usize> = lines.next().unwrap().split(',').enumerate().map(|(i, h)| (h, i)).collect();
    let rows: Vec<Vec<f64>> =
        lines.map(|l| l.split(',').map(|x| x.parse::<f64>().unwrap_or(f64::NAN)).collect()).collect();
    let chans: Vec<usize> = match a.get(4) {
        Some(s) => s.split(',').map(|x| x.parse().unwrap()).collect(),
        None => (0..p.channels as usize).collect(),
    };
    let upd = thermal_frames(p.sample_rate) as f64;
    let fi = head["frame"];
    let t0 = rows[0][fi];
    for &c in &chans {
        let ch = &p.ch[c];
        println!("ch{c}: TemperatureLimit {} C, TlimAdjCap {} K", ch.temperature_limit, ch.tlim_adj_cap);
    }
    println!("channel windows accepted maxTlimAdj meanTlimAdj(driven>0) trip_s maxTlimAdj_after_trip |T_CL-logged|max");
    for &c in &chans {
        let col = |n: &str| head[format!("{n}{c}").as_str()];
        let (pm, to, tc, vm, im) = (col("pmod"), col("tol"), col("tcl"), col("vms"), col("ims"));
        let Some(mut r) = ClReplay::new(&p, c, relax) else {
            println!("{c} no closed loop");
            continue;
        };
        let (mut prev, mut nwin, mut nacc) = ((f64::NAN, f64::NAN), 0, 0);
        let (mut max_adj, mut sum_adj, mut n_adj) = (0.0f64, 0.0, 0usize);
        let (mut trip, mut after, mut dmax) = (None::<f64>, 0.0f64, 0.0f64);
        let mut last_frame = t0;
        for row in rows.iter().filter(|r| r.len() == head.len()) {
            let (v, i) = (row[vm], row[im]);
            if v.is_finite() && i.is_finite() && (v, i) != prev {
                prev = (v, i);
                nwin += 1;
                if r.window(v as f32, i as f32) {
                    nacc += 1;
                }
            }
            let n = (((row[fi] - last_frame) / upd).round().max(1.0)) as usize;
            last_frame = row[fi];
            let mut out = (0.0, 0.0, false);
            for _ in 0..n {
                out = r.update(row[to], row[pm]);
            }
            let (adj, tcl, bad) = out;
            if (tcl - row[tc]).is_finite() {
                dmax = dmax.max((tcl - row[tc]).abs());
            }
            max_adj = max_adj.max(adj);
            if adj > 0.0 {
                sum_adj += adj;
                n_adj += 1;
            }
            if bad && trip.is_none() {
                trip = Some((row[fi] - t0) / p.sample_rate as f64);
            }
            if trip.is_some() {
                after = after.max(adj);
            }
        }
        let mean = if n_adj > 0 { sum_adj / n_adj as f64 } else { 0.0 };
        let trip = trip.map_or("-".into(), |t| format!("{t:.2}"));
        println!("{c} {nwin} {nacc} {max_adj:.2} {mean:.2} {trip} {after:.2} {dmax:.4}");
    }
}
