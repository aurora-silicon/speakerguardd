// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The built-in microphone chain (`sgv2_core::mic`): N microphone inputs,
//! one processed output. Port order: N audio inputs, one audio output, a
//! latency control output (`lv2:reportsLatency`), a health control output
//! (0: running the blob; 1: no valid blob or unconnected ports, output
//! silent).
//!
//! The blob is `$SGV2_MIC_PARAMS`, else `/run/speakerguardd/mic.bin`
//! (`aurora-speaker-tuning mic-compile`). `instantiate` (not real time)
//! reads it, and refuses (silence, health 1) one that names another board,
//! another rate or another microphone count than the variant's. `run` is
//! real time: no allocation, locks or system calls.
use crate::Lv2Descriptor;
use core::ffi::{c_char, c_void};
use sgv2_core::mic::{artifact, Engine, MAX_MICS};

pub const VARIANTS: &[(&str, usize)] = &[
    ("https://aurora-silicon.org/lv2/sgv2#mic-1ch\0", 1),
    ("https://aurora-silicon.org/lv2/sgv2#mic-2ch\0", 2),
    ("https://aurora-silicon.org/lv2/sgv2#mic-3ch\0", 3),
    ("https://aurora-silicon.org/lv2/sgv2#mic-4ch\0", 4),
];
pub const DEFAULT_PATH: &str = "/run/speakerguardd/mic.bin";

/// The loaded chain, or nothing (silence).
pub struct Mic {
    engine: Option<Engine>,
}

impl Mic {
    /// Non-RT: the blob must decode and name this board, rate and
    /// microphone count.
    pub fn load(bytes: Option<&[u8]>, board: &str, rate: f64, mics: usize) -> Mic {
        let engine = (|| {
            let (h, p) = artifact::decode(bytes?).ok()?;
            if h.binding.machine() != Some(board) || f64::from(h.sample_rate) != rate || h.mics as usize != mics {
                return None;
            }
            Engine::new(p)
        })();
        Mic { engine }
    }
    pub fn ready(&self) -> bool {
        self.engine.is_some()
    }
    pub fn latency(&self) -> usize {
        self.engine.as_ref().map_or(0, Engine::latency)
    }
    pub fn reset(&mut self) {
        if let Some(e) = &mut self.engine {
            e.reset();
        }
    }
    /// RT.
    pub fn process(&mut self, inputs: &[&[f32]], output: &mut [f32], frames: usize) {
        let frames = frames.min(output.len());
        match &mut self.engine {
            Some(e) => e.process(inputs, output, frames),
            None => output[..frames].fill(0.0),
        }
    }
}

/// Frames per copy block in `run`.
pub const BLOCK: usize = sgv2_core::mic::BLOCK;

struct Instance {
    mic: Mic,
    mics: usize,
    ports: [*mut f32; MAX_MICS + 3],
    input: [[f32; BLOCK]; MAX_MICS],
    output: [f32; BLOCK],
}

fn live_board() -> Option<String> {
    let b = std::fs::read("/proc/device-tree/compatible").ok()?;
    let first = b.split(|&x| x == 0).next()?;
    std::str::from_utf8(first).ok()?.strip_prefix("apple,").map(str::to_owned)
}

unsafe extern "C" fn instantiate(
    d: *const Lv2Descriptor,
    rate: f64,
    _: *const c_char,
    _: *const *const c_void,
) -> *mut c_void {
    let Some(i) = DESCRIPTORS.iter().position(|v| core::ptr::eq(v, d)) else { return core::ptr::null_mut() };
    let path = std::env::var("SGV2_MIC_PARAMS").unwrap_or_else(|_| DEFAULT_PATH.into());
    let bytes = std::fs::read(path).ok();
    instantiate_offline(bytes.as_deref(), &live_board().unwrap_or_default(), rate, VARIANTS[i].1)
}

/// The same ABI instance for an offline host with an explicit board.
pub fn instantiate_offline(bytes: Option<&[u8]>, board: &str, rate: f64, mics: usize) -> *mut c_void {
    if !VARIANTS.iter().any(|v| v.1 == mics) {
        return core::ptr::null_mut();
    }
    Box::into_raw(Box::new(Instance {
        mic: Mic::load(bytes, board, rate, mics),
        mics,
        ports: [core::ptr::null_mut(); MAX_MICS + 3],
        input: [[0.0; BLOCK]; MAX_MICS],
        output: [0.0; BLOCK],
    }))
    .cast()
}

unsafe extern "C" fn connect_port(h: *mut c_void, port: u32, data: *mut c_void) {
    // SAFETY: LV2 passes the live handle returned by instantiate, or null.
    let Some(i) = (unsafe { h.cast::<Instance>().as_mut() }) else { return };
    if let Some(p) = i.ports.get_mut(port as usize) {
        *p = data.cast();
    }
}

unsafe extern "C" fn activate(h: *mut c_void) {
    // SAFETY: LV2 serializes activate/run for this live instance handle.
    if let Some(i) = unsafe { h.cast::<Instance>().as_mut() } {
        i.mic.reset();
    }
}

unsafe extern "C" fn run(h: *mut c_void, frames: u32) {
    // SAFETY: LV2 passes the live handle returned by instantiate, or null.
    let Some(i) = (unsafe { h.cast::<Instance>().as_mut() }) else { return };
    let n = i.mics;
    let frames = frames as usize;
    let complete = i.ports[..n + 1].iter().all(|p| !p.is_null());
    // SAFETY: a connected control output points to one writable f32.
    unsafe {
        if !i.ports[n + 1].is_null() {
            *i.ports[n + 1] = i.mic.latency() as f32;
        }
        if !i.ports[n + 2].is_null() {
            *i.ports[n + 2] = if i.mic.ready() && complete { 0.0 } else { 1.0 };
        }
    }
    if !complete {
        if !i.ports[n].is_null() {
            // SAFETY: a connected audio port covers the host callback length.
            unsafe { core::slice::from_raw_parts_mut(i.ports[n], frames) }.fill(0.0);
        }
        return;
    }
    // Copy each block out of the host buffers before writing the output:
    // LV2 lets a host connect the output to an input buffer (in place).
    for at in (0..frames).step_by(BLOCK) {
        let len = BLOCK.min(frames - at);
        for (c, dst) in i.input.iter_mut().take(n).enumerate() {
            // SAFETY: a connected input covers `frames` samples, at+len <= frames.
            let src = unsafe { core::slice::from_raw_parts(i.ports[c].add(at), len) };
            dst[..len].copy_from_slice(src);
        }
        let (input, output, mic) = (&i.input, &mut i.output, &mut i.mic);
        let ins: [&[f32]; MAX_MICS] = core::array::from_fn(|c| if c < n { &input[c][..len] } else { &[][..] });
        mic.process(&ins[..n], &mut output[..len], len);
        // SAFETY: the output is connected and covers `frames` samples; only
        // private scratch is borrowed while writing.
        let dst = unsafe { core::slice::from_raw_parts_mut(i.ports[n].add(at), len) };
        dst.copy_from_slice(&i.output[..len]);
    }
}

unsafe extern "C" fn cleanup(h: *mut c_void) {
    // SAFETY: LV2 returns the unique Box pointer once, after its final run.
    if !h.is_null() {
        drop(unsafe { Box::from_raw(h.cast::<Instance>()) });
    }
}

unsafe extern "C" fn extension_data(_: *const c_char) -> *const c_void {
    core::ptr::null()
}

macro_rules! descriptor {
    ($i:expr) => {
        Lv2Descriptor {
            uri: VARIANTS[$i].0.as_ptr().cast(),
            instantiate,
            connect_port,
            activate: Some(activate),
            run,
            deactivate: None,
            cleanup,
            extension_data,
        }
    };
}

static DESCRIPTORS: [Lv2Descriptor; 4] = [descriptor!(0), descriptor!(1), descriptor!(2), descriptor!(3)];
const _: () = assert!(DESCRIPTORS.len() == VARIANTS.len());

pub fn descriptor(index: usize) -> *const Lv2Descriptor {
    DESCRIPTORS.get(index).map_or(core::ptr::null(), |d| d as *const _)
}
