//! Write a parameter blob with SYNTHETIC spp3 values (no Mac's tuning), for
//! exercising the plugin in a graph that plays digital silence only.
//! Usage: `test-blob <path> [channels] [sense]`.

use sgv2_core::blob::{build, Pod};
use sgv2_core::{spp3, UnitKind};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = a.get(1).expect("usage: test-blob <path> [channels] [sense]");
    let n: usize = a.get(2).and_then(|v| v.parse().ok()).unwrap_or(6);
    let s: usize = a.get(3).and_then(|v| v.parse().ok()).unwrap_or(2 * n);
    let mut p = spp3::Params {
        channels: n as u32,
        sample_rate: 48_000,
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
            group: 0,
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
            vmon_scale: 1.0,
            sense_i: (2 * c) as u32 % s.max(1) as u32,
            sense_v: (2 * c + 1) as u32 % s.max(1) as u32,
            ..Default::default()
        };
    }
    assert!(spp3::Unit::init_error(&p).is_none());
    std::fs::write(path, build(UnitKind::Spp3, n as u32, s as u32, 48_000, p.as_bytes())).unwrap();
    println!("wrote synthetic spp3 blob ({n} ch, {s} sense) to {path}");
}
