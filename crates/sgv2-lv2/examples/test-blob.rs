//! Write a parameter blob with SYNTHETIC spp3 values (no Mac's tuning), for
//! exercising the plugin in a graph that plays digital silence only.
//! Usage: `test-blob <path> <channels> <sense> <machine|-> <full_scale_mv>`.
//!
//! The operating point has no default: pass the one the supervisor will
//! read back on the card under test (the peak volts at digital full scale
//! of its amps' "Amp Gain Volume" TLV, `sgv2_supervisor::machine::
//! full_scale_mv`), or the blob is refused there. It is written to every
//! channel's `ag` and to the header. The machine "-" is this Mac's board
//! (the device tree's first root compatible, without "apple,"); the
//! supervisor refuses a blob for another board or operating point.

use sgv2_core::blob::{build_for, machine_field, Pod};
use sgv2_core::{spp3, UnitKind};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    const USAGE: &str = "usage: test-blob <path> <channels> <sense> <machine|-> <full_scale_mv>";
    let path = a.get(1).expect(USAGE);
    let n: usize = a.get(2).and_then(|v| v.parse().ok()).expect(USAGE);
    let s: usize = a.get(3).and_then(|v| v.parse().ok()).expect(USAGE);
    let board = a.get(4).filter(|b| b.as_str() != "-").cloned().unwrap_or_else(|| {
        let dt = std::fs::read("/proc/device-tree/compatible").unwrap_or_default();
        let first = dt.split(|&b| b == 0).next().unwrap_or_default();
        String::from_utf8_lossy(first).strip_prefix("apple,").unwrap_or_default().to_owned()
    });
    let machine = machine_field(&board).unwrap_or_else(|| panic!("no usable machine name {board:?}: pass one"));
    let full_scale_mv: u32 = a.get(5).and_then(|v| v.parse().ok()).filter(|&v| v > 0).expect(USAGE);
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
    for (c, ch) in p.ch[..n].iter_mut().enumerate() {
        *ch = spp3::ChannelParams {
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
            ag: full_scale_mv as f32 / 1000.0,
            thermal_ff_speedup_factor: 0.5,
            imon_scale: 1.0,
            vmon_scale: 1.0,
            sense_i: (2 * c) as u32 % s.max(1) as u32,
            sense_v: (2 * c + 1) as u32 % s.max(1) as u32,
            ..Default::default()
        };
    }
    assert!(spp3::Unit::init_error(&p).is_none());
    let blob = build_for(UnitKind::Spp3, n as u32, s as u32, 48_000, machine, full_scale_mv, p.as_bytes());
    std::fs::write(path, blob).unwrap();
    println!("wrote synthetic spp3 blob ({n} ch, {s} sense) for {board} at {full_scale_mv} mV to {path}");
}
