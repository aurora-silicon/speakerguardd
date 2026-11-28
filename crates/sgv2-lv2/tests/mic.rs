// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The microphone plugin through its C ABI, on a synthetic blob: it runs
//! exactly the engine the blob describes, in place or not, without
//! allocating in `run`, and gives silence and health 1 for a blob made for
//! another board, rate or microphone count.

use sgv2_core::blob::machine_field;
use sgv2_core::mic::{artifact, Beam, Engine, Params, Stage, Stft, Window};
use sgv2_lv2::{lv2_descriptor, mic, Lv2Descriptor, VARIANTS};
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
// SAFETY: every method forwards the unchanged pointer/layout to System; the
// const-initialised thread-local counter never allocates.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        count();
        // SAFETY: forwarded unchanged.
        unsafe { System.alloc(l) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        count();
        // SAFETY: forwarded unchanged.
        unsafe { System.alloc_zeroed(l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        count();
        // SAFETY: p/l came from System.
        unsafe { System.realloc(p, l, n) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        // SAFETY: forwarded unchanged.
        unsafe { System.dealloc(p, l) }
    }
}
#[global_allocator]
static GLOBAL: Counting = Counting;

/// A one-node filter chain: a synthetic peak.
fn tone() -> sgv2_core::voicing::chain::Spec {
    use sgv2_core::prestage::Channel;
    use sgv2_core::voicing::chain::{Kind, NodeSpec, Source, Spec, Tone};
    let mut ch = Channel::IDENTITY;
    ch.count = 1;
    ch.sections[0] = sgv2_core::voicing::tone::peak(-4.0, 2500.0, 1.5, 48_000.0);
    Spec {
        program_channels: 1,
        nodes: vec![NodeSpec {
            kind: Kind::Tone(Tone {
                channels: vec![ch],
                automation: vec![],
                dynamic: vec![],
                dynamic_automation: vec![],
            }),
            inputs: vec![Source::Program(0)],
        }],
        outputs: vec![Source::Node(0, 0)],
        links: vec![],
        relays: vec![],
        volume: Default::default(),
    }
}

fn params() -> Params {
    let (hop, bins, mics) = (480, 481, 3);
    let weights = (0..mics * bins)
        .map(|j| {
            let (m, k) = (j / bins, j % bins);
            let a = -0.01 * (k * (m + 1)) as f64;
            [0.3 * a.cos(), 0.3 * a.sin()]
        })
        .collect();
    Params {
        sample_rate: 48_000,
        mics,
        pre: vec![Stage::DcBlock { cutoff_hz: 40.0, ramp_ms: 5.0, gain_db: 0.0 }, Stage::Gain { db: 9.0 }],
        stft: Stft { hop, size: 2 * hop, window: Window::Quadrant, half_offset: false, frame_phase: 0 },
        beam: Beam { mics, bins, weights, eq: None },
        refs: Vec::new(),
        freq: Vec::new(),
        post: vec![Stage::Gain { db: -2.0 }, Stage::Graph(tone())],
        partial: false,
    }
}

fn blob(board: &str) -> Vec<u8> {
    let b = artifact::Binding { machine: machine_field(board).unwrap(), acoustic_id: 1, unit: [0; artifact::UNIT_LEN] };
    artifact::encode(&params(), &b).unwrap()
}

fn mic_descriptor(mics: usize) -> &'static Lv2Descriptor {
    let base = VARIANTS.len() + sgv2_lv2::voicing::VARIANTS.len();
    let k = mic::VARIANTS.iter().position(|v| v.1 == mics).unwrap();
    // SAFETY: the entry point returns a pointer to a static descriptor.
    unsafe { &*lv2_descriptor((base + k) as u32) }
}

/// Runs the plugin over `x` in host blocks of `block`, returning the output,
/// the last latency and health.
fn run(
    handle: *mut core::ffi::c_void,
    d: &Lv2Descriptor,
    x: &[Vec<f32>],
    block: usize,
    in_place: bool,
) -> (Vec<f32>, f32, f32) {
    let n = x.len();
    let total = x[0].len();
    let mut bufs: Vec<Vec<f32>> = x.to_vec();
    let mut out = vec![0.0f32; total];
    let (mut latency, mut health) = (-1.0f32, -1.0f32);
    unsafe {
        (d.activate.unwrap())(handle);
        let mut at = 0;
        while at < total {
            let len = block.min(total - at);
            for (c, b) in bufs.iter_mut().enumerate() {
                (d.connect_port)(handle, c as u32, b[at..].as_mut_ptr().cast());
            }
            let o = if in_place { bufs[0][at..].as_mut_ptr() } else { out[at..].as_mut_ptr() };
            (d.connect_port)(handle, n as u32, o.cast());
            (d.connect_port)(handle, n as u32 + 1, (&mut latency as *mut f32).cast());
            (d.connect_port)(handle, n as u32 + 2, (&mut health as *mut f32).cast());
            let before = allocs();
            (d.run)(handle, len as u32);
            assert_eq!(allocs(), before, "run allocated");
            if in_place {
                out[at..at + len].copy_from_slice(&bufs[0][at..at + len]);
            }
            at += len;
        }
    }
    (out, latency, health)
}

fn input() -> Vec<Vec<f32>> {
    (0..3).map(|c| (0..12_000).map(|i| (0.2 * ((i * (c + 3)) as f64 * 0.013).sin()) as f32 + 0.05).collect()).collect()
}

#[test]
fn runs_the_engine_the_blob_describes() {
    let x = input();
    let mut e = Engine::new(params()).unwrap();
    let mut want = vec![0.0f32; x[0].len()];
    let ins: Vec<&[f32]> = x.iter().map(|v| v.as_slice()).collect();
    e.process(&ins, &mut want, x[0].len());
    let d = mic_descriptor(3);
    let b = blob("j000");
    for (block, in_place) in [(128usize, false), (77, true), (1024, false)] {
        let h = mic::instantiate_offline(Some(&b), "j000", 48_000.0, 3);
        assert!(!h.is_null());
        let (got, latency, health) = run(h, d, &x, block, in_place);
        assert_eq!(health, 0.0);
        assert_eq!(latency, 959.0);
        assert_eq!(got, want, "block {block} in place {in_place}");
        unsafe { (d.cleanup)(h) };
    }
}

#[test]
fn another_board_rate_or_geometry_is_silence_with_health_set() {
    let x = input();
    let b = blob("j000");
    for (bytes, board, rate, mics) in [
        (Some(b.clone()), "j001", 48_000.0, 3usize),
        (Some(b.clone()), "j000", 44_100.0, 3),
        (Some(b.clone()), "j000", 48_000.0, 2),
        (None, "j000", 48_000.0, 3),
        (Some(b[..b.len() - 1].to_vec()), "j000", 48_000.0, 3),
    ] {
        let h = mic::instantiate_offline(bytes.as_deref(), board, rate, mics);
        let d = mic_descriptor(mics);
        let (got, _, health) = run(h, d, &x[..mics], 256, false);
        assert_eq!(health, 1.0);
        assert!(got.iter().all(|v| *v == 0.0));
        unsafe { (d.cleanup)(h) };
    }
    assert!(mic::instantiate_offline(Some(&b), "j000", 48_000.0, 5).is_null());
}
