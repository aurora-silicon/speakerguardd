//! LV2 C ABI shim around `sgv2-core`, written by hand (no LV2 crate).
//!
//! One plugin instance carries every speaker channel of the Mac, because the
//! macOS units link gains across channels and groups. Port layout for a
//! plugin variant with N channels and S sense channels:
//!
//! | ports | role |
//! |---|---|
//! | 0 .. N | audio in (program) |
//! | N .. 2N | audio out |
//! | 2N .. 2N+S | audio in (sense, in the order the parameters name) |
//! | 2N+S | control out: latency (frames), `lv2:reportsLatency` |
//! | 2N+S+1 | control out: health bits |
//!
//! Ownership: lane B (in-graph engine). TODO(lane B):
//! - load the parameter blob in `instantiate` (path from the supervisor)
//!   and pick the unit by its kind; map the heartbeat page;
//! - variants for every Mac's channel and sense counts;
//! - heartbeat and telemetry writes from `run` without syscalls.
//!
//! Until then every variant runs the fail-safe unit.

use core::ffi::{c_char, c_void};
use sgv2_core::{FailSafe, Health, Io, ProtectionUnit, UnitKind};

#[repr(C)]
pub struct Lv2Descriptor {
    pub uri: *const c_char,
    pub instantiate:
        unsafe extern "C" fn(*const Lv2Descriptor, f64, *const c_char, *const *const c_void) -> *mut c_void,
    pub connect_port: unsafe extern "C" fn(*mut c_void, u32, *mut c_void),
    pub activate: Option<unsafe extern "C" fn(*mut c_void)>,
    pub run: unsafe extern "C" fn(*mut c_void, u32),
    pub deactivate: Option<unsafe extern "C" fn(*mut c_void)>,
    pub cleanup: unsafe extern "C" fn(*mut c_void),
    pub extension_data: unsafe extern "C" fn(*const c_char) -> *const c_void,
}

// SAFETY: the descriptor holds only static data and function pointers.
unsafe impl Sync for Lv2Descriptor {}

/// Plugin variants: (URI, channels, sense channels).
pub const VARIANTS: &[(&str, usize, usize)] = &[
    ("https://aurora-silicon.org/lv2/sgv2#2ch\0", 2, 0),
    ("https://aurora-silicon.org/lv2/sgv2#2ch-vi\0", 2, 4),
    ("https://aurora-silicon.org/lv2/sgv2#4ch-vi\0", 4, 8),
    ("https://aurora-silicon.org/lv2/sgv2#6ch-vi\0", 6, 12),
];

const MAX_PORTS: usize = 3 * sgv2_core::MAX_CHANNELS + 2;

struct Instance {
    channels: usize,
    sense: usize,
    ports: [*mut f32; MAX_PORTS],
    unit: FailSafe,
}

unsafe extern "C" fn instantiate(
    descriptor: *const Lv2Descriptor,
    _rate: f64,
    _bundle: *const c_char,
    _features: *const *const c_void,
) -> *mut c_void {
    let index = DESCRIPTORS.iter().position(|d| core::ptr::eq(d, descriptor));
    let Some((_, channels, sense)) = index.map(|i| VARIANTS[i]) else {
        return core::ptr::null_mut();
    };
    let instance = Box::new(Instance {
        channels,
        sense,
        ports: [core::ptr::null_mut(); MAX_PORTS],
        // TODO(lane B): the unit from the parameter blob.
        unit: FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS),
    });
    Box::into_raw(instance).cast()
}

unsafe extern "C" fn connect_port(handle: *mut c_void, port: u32, data: *mut c_void) {
    // SAFETY: the host passes the handle `instantiate` returned.
    let instance = unsafe { &mut *handle.cast::<Instance>() };
    if let Some(slot) = instance.ports.get_mut(port as usize) {
        *slot = data.cast();
    }
}

unsafe extern "C" fn run(handle: *mut c_void, frames: u32) {
    // SAFETY: the host passes the handle `instantiate` returned and has
    // connected every port to a buffer of at least `frames` samples.
    let instance = unsafe { &mut *handle.cast::<Instance>() };
    let (n, s, frames) = (instance.channels, instance.sense, frames as usize);
    let empty: &[f32] = &[];
    let mut input = [empty; sgv2_core::MAX_CHANNELS];
    let mut sense = [empty; 2 * sgv2_core::MAX_CHANNELS];
    let mut out_store: [&mut [f32]; sgv2_core::MAX_CHANNELS] = Default::default();
    for c in 0..n {
        let (i, o) = (instance.ports[c], instance.ports[n + c]);
        if i.is_null() || o.is_null() {
            return;
        }
        // SAFETY: connected audio ports hold `frames` samples.
        unsafe {
            input[c] = core::slice::from_raw_parts(i, frames);
            out_store[c] = core::slice::from_raw_parts_mut(o, frames);
        }
    }
    for (k, slot) in sense.iter_mut().enumerate().take(s) {
        let p = instance.ports[2 * n + k];
        if !p.is_null() {
            // SAFETY: as above.
            *slot = unsafe { core::slice::from_raw_parts(p, frames) };
        }
    }
    let latency = instance.unit.latency_frames() as f32;
    let health =
        instance.unit.process(Io { frames, input: &input[..n], sense: &sense[..s], output: &mut out_store[..n] });
    for (port, value) in [(2 * n + s, latency), (2 * n + s + 1, health.0 as f32)] {
        let p = instance.ports[port];
        if !p.is_null() {
            // SAFETY: control ports point at one f32.
            unsafe { *p = value };
        }
    }
}

unsafe extern "C" fn cleanup(handle: *mut c_void) {
    // SAFETY: the handle came from `Box::into_raw` in `instantiate`.
    drop(unsafe { Box::from_raw(handle.cast::<Instance>()) });
}

unsafe extern "C" fn extension_data(_uri: *const c_char) -> *const c_void {
    core::ptr::null()
}

macro_rules! descriptor {
    ($i:expr) => {
        Lv2Descriptor {
            uri: VARIANTS[$i].0.as_ptr().cast(),
            instantiate,
            connect_port,
            activate: None,
            run,
            deactivate: None,
            cleanup,
            extension_data,
        }
    };
}

static DESCRIPTORS: [Lv2Descriptor; 4] = [descriptor!(0), descriptor!(1), descriptor!(2), descriptor!(3)];

/// The LV2 entry point.
#[no_mangle]
pub extern "C" fn lv2_descriptor(index: u32) -> *const Lv2Descriptor {
    DESCRIPTORS.get(index as usize).map_or(core::ptr::null(), |d| d as *const _)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runs_the_failsafe_through_the_c_abi() {
        let d = lv2_descriptor(0);
        assert!(!d.is_null());
        assert!(lv2_descriptor(VARIANTS.len() as u32).is_null());
        unsafe {
            let h = ((*d).instantiate)(d, 48_000.0, core::ptr::null(), core::ptr::null());
            let mut l = [1.0f32; 4];
            let mut r = [-1.0f32; 4];
            let mut ol = [0.0f32; 4];
            let mut or = [0.0f32; 4];
            let mut latency = -1.0f32;
            let mut health = 0.0f32;
            for (p, buf) in [(0, l.as_mut_ptr()), (1, r.as_mut_ptr()), (2, ol.as_mut_ptr()), (3, or.as_mut_ptr())] {
                ((*d).connect_port)(h, p, buf.cast());
            }
            ((*d).connect_port)(h, 4, (&mut latency as *mut f32).cast());
            ((*d).connect_port)(h, 5, (&mut health as *mut f32).cast());
            ((*d).run)(h, 4);
            assert!((ol[0] - 0.1).abs() < 1e-7 && (or[3] + 0.1).abs() < 1e-7);
            assert_eq!(latency, 0.0);
            assert_eq!(health as u32 & Health::FAILSAFE, Health::FAILSAFE);
            ((*d).cleanup)(h);
        }
    }
}
