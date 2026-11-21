//! Real-time safety: `process` must never allocate. Every unit is run under
//! a counting global allocator across the slice sizes the rig uses.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

mod review_common;

use sgv2_core::{atsp, lsm2, placeholder, spp3, Io, ProtectionUnit, Telemetry};

struct Counting;

thread_local! {
    /// Allocations made by this thread only, so the test harness's other
    /// threads (and any test running in parallel) cannot pollute the count.
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

fn count() {
    // `try_with`: an allocation during thread teardown is not ours to count
    let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
}

fn allocs() -> usize {
    ALLOCS.with(|c| c.get())
}

// SAFETY: forwards to the system allocator; the counter is a const-
// initialised thread-local `Cell`, which never allocates.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count();
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count();
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        count();
        unsafe { System.realloc(ptr, layout, new_size) }
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
        let before = allocs();
        unit.process(Io { frames, input: &ins, sense: &sns, output: &mut outs });
        unit.telemetry(&mut telemetry);
        assert_eq!(allocs(), before, "{:?} allocated at {frames} frames", unit.kind());
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
    for (c, ch) in p.ch[..6].iter_mut().enumerate() {
        *ch = spp3::ChannelParams {
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
    let mut compensated = review_common::lsm2_params();
    compensated.ch[0].compensation = [1.0, 250.0, 1.0, 2.0, 6.0, 4.0, 0.8, 0.0002, 0.2, 1500.0];
    let mut compensated = lsm2::Unit::new(compensated);
    assert!(!compensated.is_failsafe());
    run(&mut compensated, 2, 0);
    run(&mut atsp::Unit::new(atsp::Params::default()), 4, 0);
    let slot = atsp::Slot {
        channel_mask: 0b1111,
        gain_link: 1,
        amplifier_sensitivity: 10.0,
        rdc: 8.0,
        rth_vc: 30.0,
        tau_vc: 1.0,
        rth_mg: 40.0,
        tau_mg: 20.0,
        ambient: 40.0,
        temperature_limit: 110.0,
        hard_limit_headroom: 8.0,
        attack_ms: 500.0,
        release_ms: 5000.0,
    };
    let mut a = atsp::Unit::new(atsp::Params {
        channels: 4,
        sample_rate: 48_000,
        block_frames: 512,
        slot: [slot, Default::default(), Default::default()],
        ..Default::default()
    });
    assert!(!a.is_failsafe());
    run(&mut a, 4, 0);
}

#[test]
fn xtc_startup_and_reset_do_not_allocate() {
    use sgv2_core::voicing::xtc::{Params, Run};
    let params = Params::identity().with_startup(Params::identity(), Params::identity(), 16).unwrap();
    let mut run = Run::new(&params, 48_000.0, 2);
    for frames in [1, 7, 31, 512, 1024] {
        let (mut left, mut right) = (vec![0.25; frames], vec![0.5; frames]);
        let before = allocs();
        run.process(&mut [&mut left, &mut right], frames);
        assert_eq!(allocs(), before, "XTC allocated during its startup queue");
    }
    let before = allocs();
    run.reset();
    assert_eq!(allocs(), before, "XTC allocated while selecting the restored filter");
}
