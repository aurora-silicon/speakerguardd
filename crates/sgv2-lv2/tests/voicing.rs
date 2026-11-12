//! The public board fixtures supply lane geometry only. All processing
//! parameters are synthetic. Compare the C ABI callback with the exact
//! Chain hosting sequence used by voicing-render, including control changes.
#[path = "../../sgv2-core/tests/voicing_fixture/mod.rs"]
mod fixture;
use sgv2_core::voicing::chain::{Chain, GraphParam};
use sgv2_core::{blob, MAX_CHANNELS};
use sgv2_lv2::voicing::{self, BLOCK};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

struct Counting;
thread_local! { static ALLOCS: Cell<usize> = const { Cell::new(0) }; }
fn count() {
    let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
}
fn allocs() -> usize {
    ALLOCS.with(|c| c.get())
}
// SAFETY: every allocator method forwards the original pointer/layout to System;
// the const-initialized thread-local counter never allocates.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        count();
        // SAFETY: forward the unchanged allocation layout to System.
        unsafe { System.alloc(l) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        count();
        // SAFETY: forward the unchanged allocation layout to System.
        unsafe { System.alloc_zeroed(l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        count();
        // SAFETY: p/l came from System; the caller supplies the new size.
        unsafe { System.realloc(p, l, n) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        // SAFETY: p/l are forwarded to their original allocator unchanged.
        unsafe { System.dealloc(p, l) }
    }
}
#[global_allocator]
static ALLOC: Counting = Counting;

fn check(board: &str, lanes: usize) {
    let spec = fixture::spec(lanes);
    let blob = blob::with_voicing(&fixture::unit(board, lanes), &spec).unwrap();
    let mut reference = Chain::new(spec, 48_000.0, BLOCK).unwrap();
    let vi = voicing::VARIANTS.iter().position(|v| v.1 == lanes).unwrap();
    // SAFETY: the offline host owns the live handle and all connected buffers;
    // descriptor storage is static and every callback length fits those buffers.
    let d = unsafe { &*voicing::descriptor(vi) };
    let h = voicing::instantiate_offline(Some(&blob), board, 48_000.0, lanes);
    assert!(!h.is_null());
    let mut input = [[0.0; 8193]; 2];
    for (channel, buffer) in input.iter_mut().enumerate() {
        for (i, sample) in buffer.iter_mut().enumerate() {
            *sample = if channel == 0 {
                ((i as f64 * 0.071).sin() * 0.3) as f32
            } else {
                ((i as f64 * 0.133).cos() * 0.2) as f32
            };
        }
    }
    let mut output = [[0.0f32; 8193]; MAX_CHANNELS];
    let mut expected = [[0.0f32; 8193]; MAX_CHANNELS];
    let mut scalar = 1.0f32;
    let mut db = 0.0f32;
    let mut health = -1.0f32;
    // SAFETY: the offline host owns the live handle and all connected buffers;
    // descriptor storage is static and every callback length fits those buffers.
    unsafe {
        for (c, p) in input.iter_mut().enumerate() {
            (d.connect_port)(h, c as u32, p.as_mut_ptr().cast());
        }
        for (c, p) in output[..lanes].iter_mut().enumerate() {
            (d.connect_port)(h, (c + 2) as u32, p.as_mut_ptr().cast());
        }
        (d.connect_port)(h, (lanes + 2) as u32, (&mut scalar as *mut f32).cast());
        (d.connect_port)(h, (lanes + 3) as u32, (&mut db as *mut f32).cast());
        (d.connect_port)(h, (lanes + 4) as u32, (&mut health as *mut f32).cast());
        (d.activate.unwrap())(h);
    }
    let mut max_error = 0.0f32;
    for (frames, s) in [(0, 1.0), (1, 0.25), (7, 0.5), (127, 0.75), (512, 1.0), (513, 0.5), (4096, 0.25), (8193, 1.0)] {
        scalar = s;
        db = (60.0 * f64::from(s).log10()) as f32;
        reference.set_param(GraphParam::VolumeScalar, scalar as f64);
        reference.set_param(GraphParam::VolumeDb, db as f64);
        for at in (0..frames).step_by(BLOCK) {
            let len = BLOCK.min(frames - at);
            let mut it = expected.iter_mut();
            let mut out: [&mut [f32]; MAX_CHANNELS] = core::array::from_fn(|_| &mut it.next().unwrap()[at..at + len]);
            reference.process(&[&input[0][at..at + len], &input[1][at..at + len]], &mut out[..lanes], len);
        }
        let before = allocs();
        // SAFETY: the offline host owns the live handle and all connected buffers;
        // descriptor storage is static and every callback length fits those buffers.
        unsafe {
            (d.run)(h, frames as u32);
        }
        assert_eq!(allocs(), before, "allocation in run at {frames}");
        assert_eq!(health, 0.0);
        for (a, b) in output[..lanes].iter().flatten().zip(expected[..lanes].iter().flatten()) {
            max_error = max_error.max((a - b).abs());
        }
        // Compare only the current callback region (longer calls update it).
        for (out, reference) in output[..lanes].iter().zip(&expected[..lanes]) {
            for (a, b) in out[..frames].iter().zip(&reference[..frames]) {
                assert!((a - b).abs() <= 1e-6);
            }
        }
    }
    assert_eq!(max_error, 0.0);
    println!("EQUIVALENCE {board}: lanes={lanes} max_abs_error={max_error:.9} RT_allocations=0");
    // Zero/NaN volume silences filter history as well as the current program.
    scalar = f32::NAN;
    assert!(scalar.is_nan());
    // SAFETY: the offline host owns the live handle and all connected buffers;
    // descriptor storage is static and every callback length fits those buffers.
    unsafe {
        (d.run)(h, 8193);
    }
    assert!(output[..lanes].iter().flatten().all(|v| *v == 0.0));
    // SAFETY: the offline host owns the live handle and all connected buffers;
    // descriptor storage is static and every callback length fits those buffers.
    unsafe {
        (d.cleanup)(h);
    }
}

#[test]
fn j616s_m1pro_m2_public_fixture_geometry_matches_renderer_and_allocates_nothing() {
    // A larger stack accommodates the existing fixed-array box constructors
    // in debug builds; no thread or stack creation is done in run().
    std::thread::Builder::new()
        .stack_size(16 * 1024 * 1024)
        .spawn(|| {
            for (name, board) in [("j616", "j616s"), ("j316", "j316s"), ("j413", "j413")] {
                let path = format!("{}/../../packaging/session/fixtures/{name}.json", env!("CARGO_MANIFEST_DIR"));
                let text = std::fs::read_to_string(path).unwrap();
                // Geometry is independently covered by test_wiring.py; no board
                // coefficients or private bundle values enter this test.
                assert!(text.contains(board));
                check(board, if name == "j413" { 4 } else { 6 });
            }
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn absent_mismatched_or_corrupt_artifacts_silence_every_output() {
    let good = blob::with_voicing(&fixture::unit("j413", 4), &fixture::spec(4)).unwrap();
    let mut corrupt = good.clone();
    let last = corrupt.len() - 1;
    corrupt[last] ^= 1;
    for (bytes, board, rate, n) in [
        (None, "j413", 48000.0, 4),
        (Some(&good[..]), "j316s", 48000.0, 4),
        (Some(&good[..]), "j413", 44100.0, 4),
        (Some(&good[..]), "j413", 48000.0, 6),
        (Some(&corrupt[..]), "j413", 48000.0, 4),
    ] {
        let mut engine = voicing::Voicing::load(bytes, board, rate, n);
        assert!(!engine.ready());
        let mut o = [[1.0; 17]; MAX_CHANNELS];
        let mut it = o.iter_mut();
        let mut out: [&mut [f32]; MAX_CHANNELS] = core::array::from_fn(|_| &mut it.next().unwrap()[..]);
        engine.process(&[&[0.5; 17], &[0.25; 17]], &mut out[..n], 17, 1.0, 0.0);
        assert!(out[..n].iter().all(|b| b.iter().all(|v| *v == 0.0)));
    }
}

#[test]
fn null_ports_mute_connected_outputs() {
    // SAFETY: the offline host owns the live handle and all connected buffers;
    // descriptor storage is static and every callback length fits those buffers.
    let d = unsafe { &*voicing::descriptor(2) };
    let good = blob::with_voicing(&fixture::unit("j413", 4), &fixture::spec(4)).unwrap();
    for missing in 0..7 {
        let h = voicing::instantiate_offline(Some(&good), "j413", 48000.0, 4);
        let mut buffers = [[0.5f32; 19]; 6];
        let mut volume = 1.0f32;
        // SAFETY: the offline host owns the live handle and all connected buffers;
        // descriptor storage is static and every callback length fits those buffers.
        unsafe {
            for (c, b) in buffers.iter_mut().enumerate() {
                if c != missing {
                    (d.connect_port)(h, c as u32, b.as_mut_ptr().cast());
                }
            }
            if missing != 6 {
                (d.connect_port)(h, 6, (&mut volume as *mut f32).cast());
            }
            (d.run)(h, 19);
            (d.cleanup)(h);
        }
        for (c, b) in buffers.iter().enumerate().skip(2) {
            if c != missing {
                assert!(b.iter().all(|v| *v == 0.0));
            }
        }
    }
}

#[test]
fn the_artifact_volume_law_drives_mvol_and_the_port_still_overrides() {
    use sgv2_core::voicing::volume::Curve;
    let board = "j616s";
    let lanes = 6;
    // Synthetic range: the law, not any Mac's value.
    let mut spec = fixture::spec(lanes);
    spec.volume = Curve::hal(-40.0, 0.0).unwrap();
    let blob = blob::with_voicing(&fixture::unit(board, lanes), &spec).unwrap();
    let engine = voicing::Voicing::load(Some(&blob), board, 48_000.0, lanes);
    assert!(engine.ready());
    assert_eq!(engine.curve(), Curve::Hal { min_db: -40.0, max_db: 0.0 });
    assert_eq!(engine.volume_db(0.25, 1.0), -20.0);
    assert_eq!(engine.volume_db(1.0, 1.0), 0.0);
    assert_eq!(engine.volume_db(f64::NAN, 1.0), -40.0);
    assert_eq!(engine.volume_db(0.25, -7.0), -7.0);
    // A bundle without a range keeps the cubic taper exactly.
    let cubic = blob::with_voicing(&fixture::unit(board, lanes), &fixture::spec(lanes)).unwrap();
    let engine = voicing::Voicing::load(Some(&cubic), board, 48_000.0, lanes);
    assert_eq!(engine.curve(), Curve::Cubic);
    assert_eq!(engine.volume_db(0.5, 1.0), 60.0 * 0.5f64.log10());
    assert_eq!(engine.volume_db(0.0, 1.0), -200.0);
    // No artifact: silent anyway, and the law falls back to cubic.
    assert_eq!(voicing::Voicing::load(None, board, 48_000.0, lanes).curve(), Curve::Cubic);
}

/// Connects a 4-lane instance to `buf` (two inputs then four outputs).
/// SAFETY: the caller keeps `buf`, `scalar` and the handle alive while it runs.
unsafe fn connect4(d: &sgv2_lv2::Lv2Descriptor, h: *mut core::ffi::c_void, buf: &mut [Vec<f32>; 6], scalar: &mut f32) {
    // SAFETY: forwarded from the caller.
    unsafe {
        for (c, b) in buf.iter_mut().enumerate() {
            (d.connect_port)(h, c as u32, b.as_mut_ptr().cast());
        }
        (d.connect_port)(h, 6, (scalar as *mut f32).cast());
        (d.activate.unwrap())(h);
    }
}

#[test]
fn long_silence_stops_the_chain_and_a_new_program_renders_as_on_a_fresh_stream() {
    std::thread::Builder::new()
        .stack_size(16 * 1024 * 1024)
        .spawn(|| {
            const Q: usize = 1024;
            // SAFETY: static descriptor storage.
            let d = unsafe { &*voicing::descriptor(2) };
            let good = blob::with_voicing(&fixture::unit("j413", 4), &fixture::spec(4)).unwrap();
            let a = voicing::instantiate_offline(Some(&good), "j413", 48000.0, 4);
            let b = voicing::instantiate_offline(Some(&good), "j413", 48000.0, 4);
            let mut ba: [Vec<f32>; 6] = core::array::from_fn(|_| vec![0.0; Q]);
            let mut bb: [Vec<f32>; 6] = core::array::from_fn(|_| vec![0.0; Q]);
            let (mut sa, mut sb) = (1.0f32, 1.0f32);
            let tone = |k: usize, c: usize| (((k as f64) * if c == 0 { 0.071 } else { 0.133 }).sin() * 0.3) as f32;
            let program = |b: &mut [Vec<f32>; 6], q: usize| {
                for (c, ch) in b[..2].iter_mut().enumerate() {
                    for (k, s) in ch.iter_mut().enumerate() {
                        *s = tone(q * Q + k, c);
                    }
                }
            };
            // SAFETY: buffers, scalars and handles outlive every callback below.
            unsafe {
                connect4(d, a, &mut ba, &mut sa);
                connect4(d, b, &mut bb, &mut sb);
                // A plays, then goes quiet for longer than IDLE_AFTER.
                for q in 0..8 {
                    program(&mut ba, q);
                    (d.run)(a, Q as u32);
                }
                let quiet = (voicing::IDLE_AFTER as usize).div_ceil(Q) + 4;
                for _ in 0..quiet {
                    ba[0].fill(0.0);
                    ba[1].fill(0.0);
                    let before = allocs();
                    (d.run)(a, Q as u32);
                    assert_eq!(allocs(), before);
                }
                assert!(ba[2..].iter().flatten().all(|v| *v == 0.0), "idle output is exact silence");
                // A new program: A must render exactly as the fresh instance B.
                for q in 0..6 {
                    program(&mut ba, q);
                    program(&mut bb, q);
                    (d.run)(a, Q as u32);
                    (d.run)(b, Q as u32);
                    assert_eq!(&ba[2..], &bb[2..], "quantum {q}");
                }
                assert!(ba[2..].iter().flatten().any(|v| *v != 0.0));
                (d.cleanup)(a);
                (d.cleanup)(b);
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
