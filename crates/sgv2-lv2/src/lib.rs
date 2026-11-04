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
//! | 2N+S+2 | control out: accepted program-to-sense delay, frames (-1: none) |
//!
//! `instantiate` (not real time) reads the parameter blob, builds the unit
//! named by its kind, and maps the supervisor's engine page. `run` (real
//! time) only calls `process`, copies the unit's telemetry into the shared
//! ring when a unit block completes, and stores the heartbeat with release
//! ordering: no allocation, locks or system calls (the monotonic clock is
//! read through the vDSO).
//!
//! Paths:
//! - parameter blob: `$SGV2_PARAMS`, else `/run/speakerguardd/params.bin`
//!   (the supervisor points it at the current `params-<sha256>.bin`);
//! - engine page: `$SGV2_ENGINE_PAGE`, else `/run/speakerguardd/<uid>/engine`
//!   (created and sized by the supervisor; see [`page`]). Without a page the
//!   plugin still protects, and the supervisor, seeing no heartbeat, never
//!   takes ownership of the amps.
//!
//! A blob that is missing, corrupt, for another unit kind, or for another
//! channel/sense geometry or rate gives the fail-safe unit (20 dB), as macOS
//! does without its essential parameters.

pub mod page;
pub mod ttl;

use core::ffi::{c_char, c_void};
use sgv2_core::{lsm2, placeholder, spp3, FailSafe, Health, Io, ProtectionUnit, Telemetry, UnitKind, MAX_CHANNELS};

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

/// What a variant runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// The protection unit named by the parameter blob.
    Protect,
    /// The test placeholder (fail-safe-level gain), never selected by a
    /// Mac's tuning; for graph and ingestion tests only.
    Placeholder,
}

/// Plugin variants: (URI, channels, sense channels, role).
pub const VARIANTS: &[(&str, usize, usize, Role)] = &[
    ("https://aurora-silicon.org/lv2/sgv2#2ch\0", 2, 0, Role::Protect),
    ("https://aurora-silicon.org/lv2/sgv2#2ch-vi\0", 2, 4, Role::Protect),
    ("https://aurora-silicon.org/lv2/sgv2#4ch\0", 4, 0, Role::Protect),
    ("https://aurora-silicon.org/lv2/sgv2#4ch-vi\0", 4, 8, Role::Protect),
    ("https://aurora-silicon.org/lv2/sgv2#6ch-vi\0", 6, 12, Role::Protect),
    ("https://aurora-silicon.org/lv2/sgv2#6ch-vi-placeholder\0", 6, 12, Role::Placeholder),
];

const MAX_SENSE: usize = 2 * MAX_CHANNELS;
const MAX_PORTS: usize = 2 * MAX_CHANNELS + MAX_SENSE + 3;

pub enum AnyUnit {
    Spp3(Box<spp3::Unit>),
    Lsm2(Box<lsm2::Unit>),
    Placeholder(placeholder::Unit),
    FailSafe(FailSafe),
}

impl AnyUnit {
    pub fn as_dyn(&mut self) -> &mut dyn ProtectionUnit {
        match self {
            AnyUnit::Spp3(u) => u.as_mut(),
            AnyUnit::Lsm2(u) => u.as_mut(),
            AnyUnit::Placeholder(u) => u,
            AnyUnit::FailSafe(u) => u,
        }
    }
    pub fn unit(&self) -> &dyn ProtectionUnit {
        match self {
            AnyUnit::Spp3(u) => u.as_ref(),
            AnyUnit::Lsm2(u) => u.as_ref(),
            AnyUnit::Placeholder(u) => u,
            AnyUnit::FailSafe(u) => u,
        }
    }
    pub fn sense_delay(&self) -> Option<u32> {
        match self {
            AnyUnit::Spp3(u) => u.sense_delay(),
            _ => None,
        }
    }
}

/// Why `load_unit` fell back to the fail-safe.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LoadError {
    Read,
    Blob,
    Geometry,
    Rate,
    Refused,
    NotImplemented,
}

/// Build the unit for a variant from blob bytes (or the placeholder), and
/// the blob's payload CRC as its identity. Not real time.
pub fn load_unit(
    blob: Option<&[u8]>,
    rate: f64,
    channels: usize,
    sense: usize,
    role: Role,
) -> (AnyUnit, Option<LoadError>, u32) {
    if role == Role::Placeholder {
        let p = placeholder::Params { channels: channels as u32, ..Default::default() };
        return (AnyUnit::Placeholder(placeholder::Unit::new(p)), None, 0);
    }
    let fail = |e| (AnyUnit::FailSafe(FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS)), Some(e), 0);
    let Some(bytes) = blob else { return fail(LoadError::Read) };
    let Ok((h, _)) = sgv2_core::blob::Header::parse(bytes) else { return fail(LoadError::Blob) };
    if h.channels as usize != channels || h.sense_channels as usize != sense {
        return fail(LoadError::Geometry);
    }
    if (h.sample_rate as f64 - rate).abs() > 0.5 {
        // A rate change re-initialises from parameters made for that rate
        // (no in-place migration); a blob for another rate is unusable.
        return fail(LoadError::Rate);
    }
    if h.unit == UnitKind::Spp3 as u32 {
        match spp3::Params::from_blob(bytes) {
            Ok(p) if spp3::Unit::init_error(&p).is_none() => {
                (AnyUnit::Spp3(Box::new(spp3::Unit::new(p))), None, h.payload_crc)
            }
            Ok(_) => fail(LoadError::Refused),
            Err(_) => fail(LoadError::Blob),
        }
    } else if h.unit == UnitKind::Lsm2 as u32 {
        match lsm2::Params::from_blob(bytes) {
            Ok(p) if lsm2::Unit::init_error(&p).is_none() => {
                (AnyUnit::Lsm2(Box::new(lsm2::Unit::new(p))), None, h.payload_crc)
            }
            Ok(_) => fail(LoadError::Refused),
            Err(_) => fail(LoadError::Blob),
        }
    } else {
        // TODO(lane B): atsp payload.
        fail(LoadError::NotImplemented)
    }
}

struct Instance {
    channels: usize,
    sense: usize,
    ports: [*mut f32; MAX_PORTS],
    unit: AnyUnit,
    page: Option<page::EnginePage>,
    telemetry: Telemetry,
    last_frame: u64,
    frames: u64,
}

fn env_or(var: &str, default: String) -> String {
    std::env::var(var).unwrap_or(default)
}

unsafe extern "C" fn instantiate(
    descriptor: *const Lv2Descriptor,
    rate: f64,
    _bundle: *const c_char,
    _features: *const *const c_void,
) -> *mut c_void {
    let index = DESCRIPTORS.iter().position(|d| core::ptr::eq(d, descriptor));
    let Some((_, channels, sense, role)) = index.map(|i| VARIANTS[i]) else {
        return core::ptr::null_mut();
    };
    // SAFETY: getuid has no preconditions.
    let uid = unsafe { libc::getuid() };
    let blob_path = env_or("SGV2_PARAMS", "/run/speakerguardd/params.bin".into());
    let page_path = env_or("SGV2_ENGINE_PAGE", format!("/run/speakerguardd/{uid}/engine"));
    let blob = std::fs::read(&blob_path).ok();
    let (unit, _err, params_crc) = load_unit(blob.as_deref(), rate, channels, sense, role);
    let page = page::EnginePage::open(&page_path).ok();
    if let Some(p) = &page {
        p.start(params_crc, unit.unit().latency_frames(), channels as u32, sense as u32);
    }
    let instance = Box::new(Instance {
        channels,
        sense,
        ports: [core::ptr::null_mut(); MAX_PORTS],
        unit,
        page,
        telemetry: Telemetry::new(),
        last_frame: u64::MAX,
        frames: 0,
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

unsafe extern "C" fn activate(handle: *mut c_void) {
    // SAFETY: the host passes the handle `instantiate` returned.
    let instance = unsafe { &mut *handle.cast::<Instance>() };
    instance.unit.as_dyn().reset();
    instance.last_frame = u64::MAX;
    instance.frames = 0;
}

unsafe extern "C" fn run(handle: *mut c_void, frames: u32) {
    // SAFETY: the host passes the handle `instantiate` returned and has
    // connected every port to a buffer of at least `frames` samples.
    let instance = unsafe { &mut *handle.cast::<Instance>() };
    let (n, s, frames) = (instance.channels, instance.sense, frames as usize);
    let empty: &[f32] = &[];
    let mut input = [empty; MAX_CHANNELS];
    let mut sense = [empty; MAX_SENSE];
    let mut out_store: [&mut [f32]; MAX_CHANNELS] = Default::default();
    for c in 0..n {
        let (i, o) = (instance.ports[c], instance.ports[n + c]);
        if i.is_null() || o.is_null() {
            return;
        }
    }
    // The core needs distinct input and output buffers. PipeWire's
    // filter-graph gives every port its own buffer; if a host ever connects
    // a channel in place, every output is muted for that call rather than
    // run on aliased memory.
    let aliased = (0..n).any(|c| (0..n).any(|k| core::ptr::eq(instance.ports[c], instance.ports[n + k])));
    if aliased {
        for c in 0..n {
            // SAFETY: connected audio ports hold `frames` samples.
            unsafe { core::slice::from_raw_parts_mut(instance.ports[n + c], frames) }.fill(0.0);
        }
        return;
    }
    for c in 0..n {
        // SAFETY: connected audio ports hold `frames` samples and are
        // distinct buffers (checked above).
        unsafe {
            input[c] = core::slice::from_raw_parts(instance.ports[c], frames);
            out_store[c] = core::slice::from_raw_parts_mut(instance.ports[n + c], frames);
        }
    }
    let mut sense_n = 0;
    for (k, slot) in sense.iter_mut().enumerate().take(s) {
        let p = instance.ports[2 * n + k];
        if p.is_null() {
            break;
        }
        // SAFETY: as above.
        *slot = unsafe { core::slice::from_raw_parts(p, frames) };
        sense_n = k + 1;
    }
    // An incomplete sense bus is no bus: the unit runs open loop.
    let sense_slice = if sense_n == s { &sense[..s] } else { &sense[..0] };
    let unit = instance.unit.as_dyn();
    let latency = unit.latency_frames() as f32;
    let health = unit.process(Io { frames, input: &input[..n], sense: sense_slice, output: &mut out_store[..n] });
    instance.frames += frames as u64;
    let delay = instance.unit.sense_delay().map_or(-1, |d| d as i32);
    if let Some(page) = &instance.page {
        instance.unit.unit().telemetry(&mut instance.telemetry);
        if instance.telemetry.frame != instance.last_frame {
            instance.last_frame = instance.telemetry.frame;
            page.push(&instance.telemetry);
        }
        page.beat(instance.frames, health, delay);
    }
    for (port, value) in [(2 * n + s, latency), (2 * n + s + 1, health.0 as f32), (2 * n + s + 2, delay as f32)] {
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
            activate: Some(activate),
            run,
            deactivate: None,
            cleanup,
            extension_data,
        }
    };
}

static DESCRIPTORS: [Lv2Descriptor; 6] =
    [descriptor!(0), descriptor!(1), descriptor!(2), descriptor!(3), descriptor!(4), descriptor!(5)];

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
        std::env::set_var("SGV2_PARAMS", "/nonexistent/sgv2-test");
        std::env::set_var("SGV2_ENGINE_PAGE", "/nonexistent/sgv2-test");
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
            let mut delay = 0.0f32;
            for (p, buf) in [(0, l.as_mut_ptr()), (1, r.as_mut_ptr()), (2, ol.as_mut_ptr()), (3, or.as_mut_ptr())] {
                ((*d).connect_port)(h, p, buf.cast());
            }
            ((*d).connect_port)(h, 4, (&mut latency as *mut f32).cast());
            ((*d).connect_port)(h, 5, (&mut health as *mut f32).cast());
            ((*d).connect_port)(h, 6, (&mut delay as *mut f32).cast());
            ((*d).activate.unwrap())(h);
            ((*d).run)(h, 4);
            assert!((ol[0] - 0.1).abs() < 1e-7 && (or[3] + 0.1).abs() < 1e-7);
            assert_eq!(latency, 0.0);
            assert_eq!(delay, -1.0);
            assert_eq!(health as u32 & Health::FAILSAFE, Health::FAILSAFE);
            ((*d).cleanup)(h);
        }
    }

    #[test]
    fn wrong_geometry_rate_or_parameters_is_the_failsafe() {
        use sgv2_core::blob::Pod;
        let p = sgv2_core::spp3::Params { channels: 2, sample_rate: 48_000, ..Default::default() };
        let b = sgv2_core::blob::build(UnitKind::Spp3, 2, 4, 48_000, p.as_bytes());
        let (_, e, _) = load_unit(Some(&b), 48_000.0, 6, 12, Role::Protect);
        assert_eq!(e, Some(LoadError::Geometry));
        let (_, e, _) = load_unit(Some(&b), 44_100.0, 2, 4, Role::Protect);
        assert_eq!(e, Some(LoadError::Rate));
        // default params are structurally invalid
        let (u, e, _) = load_unit(Some(&b), 48_000.0, 2, 4, Role::Protect);
        assert!(e.is_some());
        assert!(matches!(u, AnyUnit::FailSafe(_)));
        let (_, e, _) = load_unit(None, 48_000.0, 2, 4, Role::Protect);
        assert_eq!(e, Some(LoadError::Read));
    }

    #[test]
    fn checked_in_ttl_matches_the_variants() {
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/lv2/sgv2.lv2/");
        let (manifest, plugin) = ttl::generate();
        let hint = "regenerate with: cargo run -p sgv2-lv2 --example gen-ttl";
        assert_eq!(std::fs::read_to_string(format!("{dir}manifest.ttl")).unwrap(), manifest, "{hint}");
        assert_eq!(std::fs::read_to_string(format!("{dir}sgv2.ttl")).unwrap(), plugin, "{hint}");
    }
}
