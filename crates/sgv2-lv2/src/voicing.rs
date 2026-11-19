// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Stereo program -> protection lanes. Port order is two audio inputs,
//! N audio outputs, scalar volume input, optional dB volume input, health
//! output. The scalar is the PipeWire slider position (`scale = cubic`).
//! Its dB comes from the artifact's volume law (`volume::Curve`): the Mac's
//! own HAL law when its bundle states the range, else 60*log10(scalar).
//! `volume_db` <= 0 overrides either.
//! The scalar reaches VolumeTaper, variable EQ and Tone automation; dB
//! reaches the graph's dB gain/LowFlow links. Chain applies its own relays.
//! Missing/bad artifacts or ports produce silence. All I/O and allocation
//! happens in instantiate; run/activate use only preallocated storage.
use crate::Lv2Descriptor;
use core::ffi::{c_char, c_void};
use sgv2_core::voicing::chain::{Chain, GraphParam};
use sgv2_core::voicing::volume::Curve;
use sgv2_core::{blob, MAX_CHANNELS};

pub const BLOCK: usize = 512;
/// Exact digital silence, in frames, after which the instance stops running
/// the chain (5 s at 48 kHz: longer than any box's tail).
pub const IDLE_AFTER: u64 = 240_000;
/// The last processed block must also be this quiet (about -120 dBFS)
/// before the chain stops, so stopping never cuts an audible tail.
pub const IDLE_PEAK: f32 = 1.0e-6;
pub const VARIANTS: &[(&str, usize)] = &[
    ("https://aurora-silicon.org/lv2/sgv2#voicing-1ch\0", 1),
    ("https://aurora-silicon.org/lv2/sgv2#voicing-2ch\0", 2),
    ("https://aurora-silicon.org/lv2/sgv2#voicing-4ch\0", 4),
    ("https://aurora-silicon.org/lv2/sgv2#voicing-6ch\0", 6),
];

pub struct Voicing {
    chain: Option<Chain>,
    /// The artifact's scalar-to-dB law; cubic when no chain loaded.
    curve: Curve,
    scalar: f64,
    db: f64,
}
impl Voicing {
    /// Non-RT: require the running board, rate and plugin geometry to match.
    pub fn load(bytes: Option<&[u8]>, board: &str, rate: f64, lanes: usize) -> Self {
        let loaded = (|| {
            let bytes = bytes?;
            let (h, _) = blob::Header::parse(bytes).ok()?;
            if h.machine() != Some(board) || h.sample_rate as f64 != rate || h.channels as usize != lanes {
                return None;
            }
            let spec = blob::voicing(bytes).ok()??;
            let curve = spec.volume;
            Some((Chain::new(spec, rate, BLOCK)?, curve))
        })();
        let (chain, curve) = match loaded {
            Some((c, v)) => (Some(c), v),
            None => (None, Curve::Cubic),
        };
        Self { chain, curve, scalar: f64::NAN, db: f64::NAN }
    }
    /// The loaded volume law.
    pub fn curve(&self) -> Curve {
        self.curve
    }
    /// RT: the dB the graph's `mvol` links receive for `scalar`, unless a
    /// connected `volume_db` port (<= 0) overrides it.
    #[inline]
    pub fn volume_db(&self, scalar: f64, override_db: f64) -> f64 {
        if override_db.is_finite() && override_db <= 0.0 {
            override_db
        } else {
            self.curve.db(scalar)
        }
    }
    pub fn ready(&self) -> bool {
        self.chain.is_some()
    }
    pub fn reset(&mut self) {
        if let Some(c) = &mut self.chain {
            c.reset();
        }
    }
    /// RT: control values are checked before reaching any box. Zero volume
    /// also silences residual history; the chain continues to advance.
    pub fn process(&mut self, input: &[&[f32]; 2], output: &mut [&mut [f32]], frames: usize, scalar: f64, db: f64) {
        let scalar = if scalar.is_finite() { scalar.clamp(0.0, 1.0) } else { 0.0 };
        let db = if db.is_finite() { db.min(0.0) } else { -200.0 };
        if let Some(c) = &mut self.chain {
            if scalar != self.scalar {
                c.set_param(GraphParam::VolumeScalar, scalar);
                self.scalar = scalar;
            }
            if db != self.db {
                c.set_param(GraphParam::VolumeDb, db);
                self.db = db;
            }
            c.process(input, output, frames);
        }
        if !self.ready() || scalar == 0.0 {
            for o in output {
                o[..frames].fill(0.0);
            }
        }
    }
}

struct Instance {
    engine: Voicing,
    lanes: usize,
    ports: [*mut f32; MAX_CHANNELS + 5],
    input: [[f32; BLOCK]; 2],
    output: [[f32; BLOCK]; MAX_CHANNELS],
    /// Frames of exact zero input in a row.
    silent: u64,
    /// The chain is reset and not running: the input has been exactly zero
    /// for IDLE_AFTER frames and the output had decayed below IDLE_PEAK.
    /// The first nonzero sample runs it again from the state a fresh
    /// activate gives, so a resumed program renders as on a new stream.
    idle: bool,
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
    let lanes = VARIANTS[i].1;
    let path = std::env::var("SGV2_PARAMS").unwrap_or_else(|_| "/run/speakerguardd/params.bin".into());
    let bytes = std::fs::read(path).ok();
    let board = live_board().unwrap_or_default();
    instantiate_offline(bytes.as_deref(), &board, rate, lanes)
}

/// Construct the same ABI instance for an offline host with an explicit
/// board identity. The live LV2 entry point always reads the device tree.
pub fn instantiate_offline(bytes: Option<&[u8]>, board: &str, rate: f64, lanes: usize) -> *mut c_void {
    if !VARIANTS.iter().any(|v| v.1 == lanes) {
        return core::ptr::null_mut();
    }
    Box::into_raw(Box::new(Instance {
        engine: Voicing::load(bytes, board, rate, lanes),
        lanes,
        ports: [core::ptr::null_mut(); MAX_CHANNELS + 5],
        input: [[0.0; BLOCK]; 2],
        output: [[0.0; BLOCK]; MAX_CHANNELS],
        silent: 0,
        idle: false,
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
        i.engine.reset();
        i.silent = 0;
        i.idle = false;
    }
}
unsafe extern "C" fn run(h: *mut c_void, frames: u32) {
    // SAFETY: LV2 passes the live handle returned by instantiate, or null.
    let Some(i) = (unsafe { h.cast::<Instance>().as_mut() }) else { return };
    let n = i.lanes;
    let complete = i.ports[..n + 3].iter().all(|p| !p.is_null());
    // SAFETY: a connected control output points to one writable f32.
    if !i.ports[n + 4].is_null() {
        unsafe { *i.ports[n + 4] = if i.engine.ready() && complete { 0.0 } else { 1.0 } };
    }
    if !complete {
        for &p in &i.ports[2..2 + n] {
            // SAFETY: a connected audio port covers the host callback length.
            if !p.is_null() {
                unsafe { core::slice::from_raw_parts_mut(p, frames as usize) }.fill(0.0);
            }
        }
        return;
    }
    // SAFETY: complete checked every input/control pointer; controls hold one f32.
    let scalar = unsafe { *i.ports[n + 2] } as f64;
    // SAFETY: the optional connected control holds one readable f32.
    let override_db = if i.ports[n + 3].is_null() { 1.0 } else { (unsafe { *i.ports[n + 3] }) as f64 };
    let db = i.engine.volume_db(scalar, override_db);
    for at in (0..frames as usize).step_by(BLOCK) {
        let len = BLOCK.min(frames as usize - at);
        for (c, dst) in i.input.iter_mut().enumerate() {
            // Copy before creating output slices: LV2 permits in-place ports.
            // SAFETY: the host provides frames samples and at+len <= frames.
            let src = unsafe { core::slice::from_raw_parts(i.ports[c].add(at), len) };
            for (d, s) in dst.iter_mut().zip(src) {
                *d = if s.is_finite() { *s } else { 0.0 };
            }
        }
        let quiet = i.input.iter().all(|c| c[..len].iter().all(|&x| x == 0.0));
        i.silent = if quiet { i.silent.saturating_add(len as u64) } else { 0 };
        if !quiet {
            i.idle = false;
        }
        if i.idle {
            for o in &mut i.output[..n] {
                o[..len].fill(0.0);
            }
        } else {
            let mut it = i.output.iter_mut();
            let mut out: [&mut [f32]; MAX_CHANNELS] = core::array::from_fn(|_| &mut it.next().unwrap()[..len]);
            i.engine.process(&[&i.input[0][..len], &i.input[1][..len]], &mut out[..n], len, scalar, db);
            // Nothing to play: stop running the chain (the protect stage
            // downstream keeps running, and PipeWire keeps scheduling this
            // node because of it, so without this the whole voicing runs on
            // silence forever).
            if i.silent >= IDLE_AFTER && i.output[..n].iter().all(|c| c[..len].iter().all(|x| x.abs() < IDLE_PEAK)) {
                i.engine.reset();
                i.idle = true;
                for o in &mut i.output[..n] {
                    o[..len].fill(0.0);
                }
            }
        }
        for (c, src) in i.output[..n].iter().enumerate() {
            // SAFETY: the output is connected and covers frames samples;
            // only the private scratch is borrowed while writing, allowing in-place I/O.
            let dst = unsafe { core::slice::from_raw_parts_mut(i.ports[c + 2].add(at), len) };
            for (d, s) in dst.iter_mut().zip(src) {
                *d = if s.is_finite() { *s } else { 0.0 };
            }
        }
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
pub fn descriptor(index: usize) -> *const Lv2Descriptor {
    DESCRIPTORS.get(index).map_or(core::ptr::null(), |d| d as *const _)
}
