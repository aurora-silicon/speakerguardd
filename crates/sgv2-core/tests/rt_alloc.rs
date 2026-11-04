//! Real-time safety: `process` must never allocate. Every unit is run under
//! a counting global allocator across the slice sizes the rig uses.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use sgv2_core::{atsp, lsm2, placeholder, spp3, Io, ProtectionUnit, Telemetry};

struct Counting;
static ALLOCS: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn run(unit: &mut dyn ProtectionUnit, channels: usize, sense: usize) {
    let input = vec![vec![0.5f32; 4096]; channels];
    let sense_buf = vec![vec![0.0f32; 4096]; sense];
    let mut output = vec![vec![0.0f32; 4096]; channels];
    let mut telemetry = Telemetry::new();
    for frames in [1usize, 7, 31, 96, 127, 160, 512, 1024, 4096] {
        let ins: Vec<&[f32]> = input.iter().map(|b| &b[..frames]).collect();
        let sns: Vec<&[f32]> = sense_buf.iter().map(|b| &b[..frames]).collect();
        let mut outs: Vec<&mut [f32]> = output.iter_mut().map(|b| &mut b[..frames]).collect();
        let before = ALLOCS.load(Ordering::SeqCst);
        unit.process(Io { frames, input: &ins, sense: &sns, output: &mut outs });
        unit.telemetry(&mut telemetry);
        assert_eq!(ALLOCS.load(Ordering::SeqCst), before, "{:?} allocated at {frames} frames", unit.kind());
    }
}

/// A valid synthetic spp3 set (not any Mac's values).
fn spp3_synthetic() -> spp3::Params {
    let mut p = spp3::Params {
        channels: 6,
        sample_rate: 48_000,
        block_frames: 512,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 1.0,
        thermal_gain_release_time_s: 5.0,
        thermal_lookahead_delay_ms: 1.0,
        thermal_protection_enabled: 1,
        ..spp3::Params::default()
    };
    for c in 0..6 {
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
            imon_scale: 1.0,
            vmon_scale: 10.0,
            sense_i: 2 * c as u32,
            sense_v: 2 * c as u32 + 1,
            thermal_ff_speedup_factor: 0.5,
            ..Default::default()
        };
    }
    p
}

#[test]
fn no_unit_allocates_in_process() {
    run(&mut spp3::Unit::new(spp3::Params::default()), 6, 12);
    let mut real = spp3::Unit::new(spp3_synthetic());
    assert!(!real.is_failsafe());
    run(&mut real, 6, 12);
    run(&mut real, 6, 0);
    run(&mut real, 5, 12);
    run(&mut placeholder::Unit::new(placeholder::Params { channels: 6, ..Default::default() }), 6, 12);
    run(&mut lsm2::Unit::new(lsm2::Params::default()), 2, 0);
    run(&mut atsp::Unit::new(atsp::Params::default()), 4, 0);
}
