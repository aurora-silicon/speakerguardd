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
use core::ffi::{c_char, c_void, CStr};
use sgv2_core::voicing::buzz::{BufferMode, HOST_HOP};
use sgv2_core::voicing::chain::{Chain, GraphParam};
use sgv2_core::voicing::volume::Curve;
use sgv2_core::{blob, MAX_CHANNELS};

pub const BLOCK: usize = 512;
const FEATURE_OPTIONS: &[u8] = b"http://lv2plug.in/ns/ext/options#options\0";
const FEATURE_URID_MAP: &[u8] = b"http://lv2plug.in/ns/ext/urid#map\0";
#[cfg(test)]
const FEATURE_FIXED_BLOCK: &[u8] = b"http://lv2plug.in/ns/ext/buf-size#fixedBlockLength\0";
#[cfg(test)]
const FEATURE_POWER_OF_TWO_BLOCK: &[u8] = b"http://lv2plug.in/ns/ext/buf-size#powerOf2BlockLength\0";
#[cfg(test)]
const FEATURE_BOUNDED_BLOCK: &[u8] = b"http://lv2plug.in/ns/ext/buf-size#boundedBlockLength\0";
#[cfg(test)]
const FEATURE_LOG: &[u8] = b"http://lv2plug.in/ns/ext/log#log\0";
#[cfg(test)]
const FEATURE_URID_UNMAP: &[u8] = b"http://lv2plug.in/ns/ext/urid#unmap\0";
const OPTION_MIN_BLOCK: &[u8] = b"http://lv2plug.in/ns/ext/buf-size#minBlockLength\0";
const OPTION_MAX_BLOCK: &[u8] = b"http://lv2plug.in/ns/ext/buf-size#maxBlockLength\0";
#[cfg(test)]
const OPTION_NOMINAL_BLOCK: &[u8] = b"http://lv2plug.in/ns/ext/buf-size#nominalBlockLength\0";
#[cfg(test)]
const OPTION_SEQUENCE_SIZE: &[u8] = b"http://lv2plug.in/ns/ext/buf-size#sequenceSize\0";
#[cfg(test)]
const OPTION_SAMPLE_RATE: &[u8] = b"http://lv2plug.in/ns/ext/parameters#sampleRate\0";
const ATOM_INT: &[u8] = b"http://lv2plug.in/ns/ext/atom#Int\0";
#[cfg(test)]
const ATOM_FLOAT: &[u8] = b"http://lv2plug.in/ns/ext/atom#Float\0";
pub const VARIANTS: &[(&str, usize, BufferMode)] = &[
    ("https://aurora-silicon.org/lv2/sgv2#voicing-1ch\0", 1, BufferMode::Direct),
    ("https://aurora-silicon.org/lv2/sgv2#voicing-2ch\0", 2, BufferMode::Direct),
    ("https://aurora-silicon.org/lv2/sgv2#voicing-4ch\0", 4, BufferMode::Direct),
    ("https://aurora-silicon.org/lv2/sgv2#voicing-6ch\0", 6, BufferMode::Direct),
    // Appended so every deployed descriptor index above stays unchanged.
    ("https://aurora-silicon.org/lv2/sgv2#voicing-1ch-staged\0", 1, BufferMode::Staged),
    ("https://aurora-silicon.org/lv2/sgv2#voicing-2ch-staged\0", 2, BufferMode::Staged),
    ("https://aurora-silicon.org/lv2/sgv2#voicing-4ch-staged\0", 4, BufferMode::Staged),
    ("https://aurora-silicon.org/lv2/sgv2#voicing-6ch-staged\0", 6, BufferMode::Staged),
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
        Self::load_with_buffer_mode(bytes, board, rate, lanes, BufferMode::Direct)
    }

    pub fn load_with_buffer_mode(
        bytes: Option<&[u8]>,
        board: &str,
        rate: f64,
        lanes: usize,
        buffer_mode: BufferMode,
    ) -> Self {
        let loaded = (|| {
            let bytes = bytes?;
            let (h, _) = blob::Header::parse(bytes).ok()?;
            if h.machine() != Some(board) || h.sample_rate as f64 != rate || h.channels as usize != lanes {
                return None;
            }
            let spec = blob::voicing(bytes).ok()??;
            let curve = spec.volume;
            Some((Chain::with_buzz_buffer_mode(spec, rate, BLOCK, buffer_mode)?, curve))
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
    pub fn latency(&self) -> usize {
        self.chain.as_ref().map_or(0, Chain::latency)
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
}

#[repr(C)]
struct Lv2Feature {
    uri: *const c_char,
    data: *mut c_void,
}

#[repr(C)]
struct UridMap {
    handle: *mut c_void,
    map: unsafe extern "C" fn(*mut c_void, *const c_char) -> u32,
}

#[repr(C)]
struct Lv2Option {
    context: u32,
    subject: u32,
    key: u32,
    size: u32,
    value_type: u32,
    value: *const c_void,
}

unsafe fn feature_is(feature: &Lv2Feature, uri: &[u8]) -> bool {
    !feature.uri.is_null() && unsafe { CStr::from_ptr(feature.uri) }.to_bytes_with_nul() == uri
}

/// Choose once at instantiation. The deployed URIs preserve the historical
/// direct path unless the host proves it cannot provide a complete decision
/// hop. A broad capability range or nominal length is not such proof: in
/// particular PipeWire advertises a broad capability range even when the
/// graph quantum is locked to one decision hop. The staged URIs are always
/// staged.
unsafe fn buffer_mode(requested: BufferMode, features: *const *const c_void) -> BufferMode {
    if requested == BufferMode::Staged {
        return BufferMode::Staged;
    }
    if features.is_null() {
        return BufferMode::Direct;
    }
    let (mut options, mut map) = (core::ptr::null(), core::ptr::null());
    let mut at = 0;
    loop {
        // SAFETY: LV2 supplies a null-terminated feature-pointer array.
        let feature = unsafe { *features.add(at) }.cast::<Lv2Feature>();
        if feature.is_null() {
            break;
        }
        // SAFETY: each non-null entry points to an LV2_Feature for instantiate.
        let feature = unsafe { &*feature };
        if unsafe { feature_is(feature, FEATURE_OPTIONS) } {
            options = feature.data.cast::<Lv2Option>();
        } else if unsafe { feature_is(feature, FEATURE_URID_MAP) } {
            map = feature.data.cast::<UridMap>();
        }
        at += 1;
    }
    if options.is_null() || map.is_null() {
        return BufferMode::Direct;
    }
    // SAFETY: the map feature data has the LV2_URID_Map ABI.
    let map = unsafe { &*map };
    let urid = |uri: &[u8]| unsafe { (map.map)(map.handle, uri.as_ptr().cast()) };
    let [min_key, max_key, int_type] = [urid(OPTION_MIN_BLOCK), urid(OPTION_MAX_BLOCK), urid(ATOM_INT)];
    if [min_key, max_key, int_type].contains(&0) {
        return BufferMode::Direct;
    }
    let (mut min, mut max) = (None, None);
    let mut at = 0;
    loop {
        // SAFETY: options points to the host's zero-terminated LV2 options array.
        let option = unsafe { &*options.add(at) };
        if option.key == 0 && option.value.is_null() {
            break;
        }
        if option.context == 0 && option.size == 4 && option.value_type == int_type && !option.value.is_null() {
            // SAFETY: size/type identify one atom:Int body; unaligned hosts are tolerated.
            let value = unsafe { core::ptr::read_unaligned(option.value.cast::<i32>()) };
            let value = usize::try_from(value).ok().filter(|value| *value != 0);
            match option.key {
                key if key == min_key => min = value,
                key if key == max_key => max = value,
                _ => {}
            }
        }
        at += 1;
    }
    let maximum_too_short = max.is_some_and(|frames| frames < HOST_HOP);
    let fixed_misaligned = matches!((min, max), (Some(a), Some(b)) if a == b && a % HOST_HOP != 0);
    if maximum_too_short || fixed_misaligned {
        BufferMode::Staged
    } else {
        BufferMode::Direct
    }
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
    features: *const *const c_void,
) -> *mut c_void {
    let Some(i) = DESCRIPTORS.iter().position(|v| core::ptr::eq(v, d)) else { return core::ptr::null_mut() };
    let (_, lanes, requested_mode) = VARIANTS[i];
    let path = std::env::var("SGV2_PARAMS").unwrap_or_else(|_| "/run/speakerguardd/params.bin".into());
    let bytes = std::fs::read(path).ok();
    let board = live_board().unwrap_or_default();
    // SAFETY: LV2 owns the feature array for the duration of instantiate.
    let mode = unsafe { buffer_mode(requested_mode, features) };
    instantiate_offline_with_mode(bytes.as_deref(), &board, rate, lanes, mode)
}

/// Construct the same ABI instance for an offline host with an explicit
/// board identity. The live LV2 entry point always reads the device tree.
pub fn instantiate_offline(bytes: Option<&[u8]>, board: &str, rate: f64, lanes: usize) -> *mut c_void {
    instantiate_offline_with_mode(bytes, board, rate, lanes, BufferMode::Direct)
}

pub fn instantiate_offline_with_mode(
    bytes: Option<&[u8]>,
    board: &str,
    rate: f64,
    lanes: usize,
    mode: BufferMode,
) -> *mut c_void {
    if !VARIANTS.iter().any(|v| v.1 == lanes) {
        return core::ptr::null_mut();
    }
    Box::into_raw(Box::new(Instance {
        engine: Voicing::load_with_buffer_mode(bytes, board, rate, lanes, mode),
        lanes,
        ports: [core::ptr::null_mut(); MAX_CHANNELS + 5],
        input: [[0.0; BLOCK]; 2],
        output: [[0.0; BLOCK]; MAX_CHANNELS],
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
        let mut it = i.output.iter_mut();
        let mut out: [&mut [f32]; MAX_CHANNELS] = core::array::from_fn(|_| &mut it.next().unwrap()[..len]);
        i.engine.process(&[&i.input[0][..len], &i.input[1][..len]], &mut out[..n], len, scalar, db);
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
static DESCRIPTORS: [Lv2Descriptor; 8] = [
    descriptor!(0),
    descriptor!(1),
    descriptor!(2),
    descriptor!(3),
    descriptor!(4),
    descriptor!(5),
    descriptor!(6),
    descriptor!(7),
];
const _: () = assert!(DESCRIPTORS.len() == VARIANTS.len());
pub fn descriptor(index: usize) -> *const Lv2Descriptor {
    DESCRIPTORS.get(index).map_or(core::ptr::null(), |d| d as *const _)
}

#[cfg(test)]
mod tests {
    use super::*;

    const URID_MIN_BLOCK: u32 = 1;
    const URID_MAX_BLOCK: u32 = 2;
    const URID_NOMINAL_BLOCK: u32 = 3;
    const URID_ATOM_INT: u32 = 4;
    const URID_SEQUENCE_SIZE: u32 = 5;
    const URID_SAMPLE_RATE: u32 = 6;
    const URID_ATOM_FLOAT: u32 = 7;
    const PIPEWIRE_MIN_BLOCK: i32 = 1;
    const PIPEWIRE_MAX_BLOCK: i32 = 8192;
    const PIPEWIRE_NOMINAL_BLOCK: i32 = 1024;
    const PIPEWIRE_SEQUENCE_SIZE: i32 = 32768;
    const PIPEWIRE_SAMPLE_RATE: f32 = 48_000.0;

    unsafe extern "C" fn map_uri(_: *mut c_void, uri: *const c_char) -> u32 {
        // SAFETY: the parser calls the mock with one of its static URI strings.
        match unsafe { CStr::from_ptr(uri) }.to_bytes_with_nul() {
            OPTION_MIN_BLOCK => URID_MIN_BLOCK,
            OPTION_MAX_BLOCK => URID_MAX_BLOCK,
            OPTION_NOMINAL_BLOCK => URID_NOMINAL_BLOCK,
            ATOM_INT => URID_ATOM_INT,
            OPTION_SEQUENCE_SIZE => URID_SEQUENCE_SIZE,
            OPTION_SAMPLE_RATE => URID_SAMPLE_RATE,
            ATOM_FLOAT => URID_ATOM_FLOAT,
            _ => 0,
        }
    }

    fn declared_mode(
        requested: BufferMode,
        fixed: bool,
        power_of_two: bool,
        bounded: bool,
        min: Option<i32>,
        max: Option<i32>,
        nominal: Option<i32>,
    ) -> BufferMode {
        let mut options = Vec::new();
        for (key, value) in
            [(URID_MIN_BLOCK, min.as_ref()), (URID_MAX_BLOCK, max.as_ref()), (URID_NOMINAL_BLOCK, nominal.as_ref())]
        {
            if let Some(value) = value {
                options.push(Lv2Option {
                    context: 0,
                    subject: 0,
                    key,
                    size: 4,
                    value_type: URID_ATOM_INT,
                    value: (value as *const i32).cast(),
                });
            }
        }
        options.push(Lv2Option { context: 0, subject: 0, key: 0, size: 0, value_type: 0, value: core::ptr::null() });
        let mut map = UridMap { handle: core::ptr::null_mut(), map: map_uri };
        let mut features = vec![
            Lv2Feature { uri: FEATURE_URID_MAP.as_ptr().cast(), data: (&mut map as *mut UridMap).cast() },
            Lv2Feature { uri: FEATURE_OPTIONS.as_ptr().cast(), data: options.as_mut_ptr().cast() },
        ];
        if fixed {
            features.push(Lv2Feature { uri: FEATURE_FIXED_BLOCK.as_ptr().cast(), data: core::ptr::null_mut() });
        }
        if power_of_two {
            features.push(Lv2Feature { uri: FEATURE_POWER_OF_TWO_BLOCK.as_ptr().cast(), data: core::ptr::null_mut() });
        }
        if bounded {
            features.push(Lv2Feature { uri: FEATURE_BOUNDED_BLOCK.as_ptr().cast(), data: core::ptr::null_mut() });
        }
        let mut pointers: Vec<*const c_void> =
            features.iter().map(|feature| (feature as *const Lv2Feature).cast()).collect();
        pointers.push(core::ptr::null());
        // SAFETY: all mock feature and option storage remains live for this call.
        unsafe { buffer_mode(requested, pointers.as_ptr()) }
    }

    fn pipewire_1_6_7_mode(requested: BufferMode) -> BufferMode {
        let mut options = [
            Lv2Option {
                context: 0,
                subject: 0,
                key: URID_MIN_BLOCK,
                size: size_of::<i32>() as u32,
                value_type: URID_ATOM_INT,
                value: (&PIPEWIRE_MIN_BLOCK as *const i32).cast(),
            },
            Lv2Option {
                context: 0,
                subject: 0,
                key: URID_MAX_BLOCK,
                size: size_of::<i32>() as u32,
                value_type: URID_ATOM_INT,
                value: (&PIPEWIRE_MAX_BLOCK as *const i32).cast(),
            },
            Lv2Option {
                context: 0,
                subject: 0,
                key: URID_SEQUENCE_SIZE,
                size: size_of::<i32>() as u32,
                value_type: URID_ATOM_INT,
                value: (&PIPEWIRE_SEQUENCE_SIZE as *const i32).cast(),
            },
            Lv2Option {
                context: 0,
                subject: 0,
                key: URID_NOMINAL_BLOCK,
                size: size_of::<i32>() as u32,
                value_type: URID_ATOM_INT,
                value: (&PIPEWIRE_NOMINAL_BLOCK as *const i32).cast(),
            },
            Lv2Option {
                context: 0,
                subject: 0,
                key: URID_SAMPLE_RATE,
                size: size_of::<f32>() as u32,
                value_type: URID_ATOM_FLOAT,
                value: (&PIPEWIRE_SAMPLE_RATE as *const f32).cast(),
            },
            Lv2Option { context: 0, subject: 0, key: 0, size: 0, value_type: 0, value: core::ptr::null() },
        ];
        let mut map = UridMap { handle: core::ptr::null_mut(), map: map_uri };
        let features = [
            Lv2Feature { uri: FEATURE_LOG.as_ptr().cast(), data: core::ptr::null_mut() },
            Lv2Feature { uri: FEATURE_URID_MAP.as_ptr().cast(), data: (&mut map as *mut UridMap).cast() },
            Lv2Feature { uri: FEATURE_URID_UNMAP.as_ptr().cast(), data: core::ptr::null_mut() },
            Lv2Feature { uri: FEATURE_FIXED_BLOCK.as_ptr().cast(), data: core::ptr::null_mut() },
            Lv2Feature { uri: FEATURE_POWER_OF_TWO_BLOCK.as_ptr().cast(), data: core::ptr::null_mut() },
            Lv2Feature { uri: FEATURE_BOUNDED_BLOCK.as_ptr().cast(), data: core::ptr::null_mut() },
            Lv2Feature { uri: FEATURE_OPTIONS.as_ptr().cast(), data: options.as_mut_ptr().cast() },
        ];
        let mut pointers: Vec<*const c_void> =
            features.iter().map(|feature| (feature as *const Lv2Feature).cast()).collect();
        pointers.push(core::ptr::null());
        // SAFETY: this is PipeWire 1.6.7's feature and option layout, with all
        // pointed-to mock storage live for the call.
        unsafe { buffer_mode(requested, pointers.as_ptr()) }
    }

    #[test]
    fn pipewire_1_6_7_features_keep_shipped_uris_direct() {
        assert_eq!(pipewire_1_6_7_mode(BufferMode::Direct), BufferMode::Direct);
    }

    #[test]
    fn only_explicit_staging_or_proven_misalignment_selects_staged() {
        assert_eq!(pipewire_1_6_7_mode(BufferMode::Staged), BufferMode::Staged);
        let aligned = HOST_HOP as i32;
        let aligned_double = (2 * HOST_HOP) as i32;
        let misaligned = (HOST_HOP + 1) as i32;
        let too_short = (HOST_HOP - 1) as i32;
        assert_eq!(
            declared_mode(BufferMode::Direct, true, false, false, Some(aligned), Some(aligned), Some(aligned)),
            BufferMode::Direct
        );
        assert_eq!(
            declared_mode(
                BufferMode::Direct,
                true,
                false,
                false,
                Some(aligned_double),
                Some(aligned_double),
                Some(aligned_double)
            ),
            BufferMode::Direct
        );
        assert_eq!(
            declared_mode(BufferMode::Direct, true, false, false, Some(misaligned), Some(misaligned), Some(misaligned)),
            BufferMode::Staged
        );
        assert_eq!(
            declared_mode(BufferMode::Direct, false, false, true, Some(1), Some(too_short), Some(too_short)),
            BufferMode::Staged
        );
        assert_eq!(
            declared_mode(BufferMode::Direct, true, false, false, Some(aligned), Some(aligned_double), Some(aligned)),
            BufferMode::Direct
        );
        assert_eq!(
            declared_mode(BufferMode::Direct, false, true, false, None, None, Some(aligned)),
            BufferMode::Direct
        );
        assert_eq!(
            declared_mode(BufferMode::Direct, false, false, false, None, None, Some(misaligned)),
            BufferMode::Direct
        );
    }
}
