//! Real-time safety: `process` must never allocate. Every unit is run under
//! a counting global allocator across the slice sizes the rig uses.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use sgv2_core::{atsp, lsm2, spp3, Io, ProtectionUnit, Telemetry};

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

#[test]
fn no_unit_allocates_in_process() {
    run(&mut spp3::Unit::new(spp3::Params::default()), 6, 12);
    run(&mut lsm2::Unit::new(lsm2::Params::default()), 2, 0);
    run(&mut atsp::Unit::new(atsp::Params::default()), 4, 0);
}
