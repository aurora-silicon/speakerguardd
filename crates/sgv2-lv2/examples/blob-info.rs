//! Check a parameter blob the way the plugin loads it, and print what it
//! holds (for bring-up; the output is that Mac's tuning, so keep it local).
//! Usage: `blob-info <blob> [channels] [sense]`.

use sgv2_core::blob::Header;
use sgv2_core::{spp3, Io};
use sgv2_lv2::{load_unit, AnyUnit, Role};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = a.get(1).expect("usage: blob-info <blob> [channels] [sense]");
    let n: usize = a.get(2).and_then(|v| v.parse().ok()).unwrap_or(6);
    let s: usize = a.get(3).and_then(|v| v.parse().ok()).unwrap_or(2 * n);
    let bytes = std::fs::read(path).unwrap();
    let (h, _) = Header::parse(&bytes).expect("header");
    println!("header: {h:?}");
    let (mut unit, err, crc) = load_unit(Some(&bytes), 48_000.0, n, s, Role::Protect);
    println!("load: err={err:?} crc={crc:08x} spp3={}", matches!(unit, AnyUnit::Spp3(_)));
    if let Ok(p) = spp3::Params::from_blob(&bytes) {
        println!(
            "global: block={} ambient={} attack={} release={} look={}ms",
            p.block_frames,
            p.thermal_ambient_temp,
            p.thermal_gain_attack_time_s,
            p.thermal_gain_release_time_s,
            p.thermal_lookahead_delay_ms
        );
        for (i, c) in p.ch[..p.channels as usize].iter().enumerate() {
            println!(
                "ch{i}: src={} grp={} ign={} Tlim={} reb={} rsh={} ag={} imon={} vmon={} ipt={} vpt={} sI={} sV={} \
                 clfb={} pilot={} reb_rng={}..{} cal_rng={}..{} a20={} a35={} cal={} R={} T={}",
                c.source,
                c.group,
                c.ignore_telemetry,
                c.temperature_limit,
                c.reb_ref,
                c.rshunt,
                c.ag,
                c.imon_scale,
                c.vmon_scale,
                c.i_pwr_thres,
                c.v_pwr_thres,
                c.sense_i,
                c.sense_v,
                c.cl_thermal_feedback_enabled,
                c.pilot_tone_enabled,
                c.reb_range_min,
                c.reb_range_max,
                c.cal_temp_range_min,
                c.cal_temp_range_max,
                c.a_t_20c,
                c.a_t_35c,
                c.cal_present,
                c.cal_r,
                c.cal_t
            );
        }
    }
    // one second of digital silence with silent sense: the output must be
    // exact zeros and the unit healthy
    let frames = 512;
    let zin = vec![0.0f32; frames];
    let input: Vec<&[f32]> = (0..n).map(|_| zin.as_slice()).collect();
    let sense: Vec<&[f32]> = (0..s).map(|_| zin.as_slice()).collect();
    let mut outs: Vec<Vec<f32>> = (0..n).map(|_| vec![1.0f32; frames]).collect();
    let mut health = 0u32;
    for _ in 0..94 {
        let mut o: Vec<&mut [f32]> = outs.iter_mut().map(|v| v.as_mut_slice()).collect();
        let u = unit.as_dyn();
        health |= u.process(Io { frames, input: &input, sense: &sense, output: &mut o }).0;
    }
    let peak = outs.iter().flatten().fold(0.0f32, |m, v| m.max(v.abs()));
    println!("silence: health_or={health:#x} out_peak={peak} latency={}", unit.unit().latency_frames());
}
