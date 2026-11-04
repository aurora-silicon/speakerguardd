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
//!
//! `activate` clears the audio histories but not the thermal model: `run`
//! publishes the unit's thermal state after every call and `activate`
//! restores the latest record that fits, cooled for the time since (see
//! [`handoff`]). A pause, a suspend or a graph rebuild in the same process
//! therefore never gives a hot speaker full power back.

pub mod handoff;
pub mod page;
pub mod ttl;

use core::ffi::{c_char, c_void};
use sgv2_core::{
    atsp, lsm2, placeholder, spp3, FailSafe, HandOff, Health, Io, ProtectionUnit, Telemetry, UnitKind, MAX_CHANNELS,
};

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
    Atsp(Box<atsp::Unit>),
    Placeholder(placeholder::Unit),
    FailSafe(FailSafe),
}

impl AnyUnit {
    pub fn as_dyn(&mut self) -> &mut dyn ProtectionUnit {
        match self {
            AnyUnit::Spp3(u) => u.as_mut(),
            AnyUnit::Lsm2(u) => u.as_mut(),
            AnyUnit::Atsp(u) => u.as_mut(),
            AnyUnit::Placeholder(u) => u,
            AnyUnit::FailSafe(u) => u,
        }
    }
    pub fn unit(&self) -> &dyn ProtectionUnit {
        match self {
            AnyUnit::Spp3(u) => u.as_ref(),
            AnyUnit::Lsm2(u) => u.as_ref(),
            AnyUnit::Atsp(u) => u.as_ref(),
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
    } else if h.unit == UnitKind::Atsp as u32 {
        match atsp::Params::from_blob(bytes) {
            Ok(p) if atsp::Unit::init_error(&p).is_none() => {
                (AnyUnit::Atsp(Box::new(atsp::Unit::new(p))), None, h.payload_crc)
            }
            Ok(_) => fail(LoadError::Refused),
            Err(_) => fail(LoadError::Blob),
        }
    } else {
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
    /// Scratch for the thermal hand-off record `run` publishes.
    handoff: HandOff,
    /// The blob payload CRC the unit runs (0: none).
    params_crc: u32,
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
    handoff::init();
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
        handoff: HandOff::EMPTY,
        params_crc,
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
    let provided = instance.page.as_ref().and_then(|p| p.provided_for(instance.params_crc));
    start_stream(instance.unit.as_dyn(), &handoff::SLOT, provided);
    instance.last_frame = u64::MAX;
    instance.frames = 0;
}

/// The oldest supervisor-provided record a new process restores; older,
/// the speakers have long cooled and the parameters' start state applies.
pub const PROVIDED_MAX_AGE_S: f64 = 600.0;

/// A stream starts: audio histories cleared, the thermal model restored
/// from the latest hand-off record that fits this unit (cooled for its age
/// by the unit's own decay), else the parameters' start state. The
/// process's own record comes first; a new process (its slot empty) takes
/// the record the supervisor `provided` across the restart, if it is no
/// older than [`PROVIDED_MAX_AGE_S`]. Not real time.
pub fn start_stream(unit: &mut dyn ProtectionUnit, slot: &handoff::Slot, provided: Option<(HandOff, f64)>) -> bool {
    unit.reset();
    match slot.latest() {
        Some((record, age_s)) => unit.restore_state(&record, age_s),
        None => match provided {
            Some((record, age_s)) if age_s.is_finite() && age_s <= PROVIDED_MAX_AGE_S => {
                unit.restore_state(&record, age_s)
            }
            _ => false,
        },
    }
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
    // A missing program port: every connected output gets the 20 dB
    // fail-safe of its own input (silence without one), never a stale
    // buffer, and the call still reports and beats.
    if (0..2 * n).any(|p| instance.ports[p].is_null()) {
        for c in 0..n {
            let (i, o) = (instance.ports[c], instance.ports[n + c]);
            if o.is_null() {
                continue;
            }
            // Raw pointers only: `i` may be `o` itself (in place, each
            // sample read before it is written); another channel's input
            // aliasing this output is muted instead.
            let cross = (0..n).any(|k| k != c && core::ptr::eq(instance.ports[k], o));
            for k in 0..frames {
                // SAFETY: connected audio ports hold `frames` samples.
                unsafe {
                    let y = if i.is_null() || cross {
                        0.0
                    } else {
                        sgv2_core::dsp::bound1(*i.add(k)) * sgv2_core::FAILSAFE_GAIN
                    };
                    *o.add(k) = y;
                }
            }
        }
        finish_call(instance, frames, Health(Health::FAILSAFE | Health::CHANNEL_MISMATCH));
        return;
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
        finish_call(instance, frames, Health(Health::FAILSAFE | Health::CHANNEL_MISMATCH));
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
    if instance.unit.unit().save_state(&mut instance.handoff) {
        handoff::SLOT.publish(&instance.handoff);
        // for a later process, through the supervisor; never a record the
        // unit does not trust (save_state is false then) nor one from a
        // faulted or fail-safe call
        if let Some(page) = &instance.page {
            // MODEL_RANGE still protects: kept
            let untrusted = Health::FAILSAFE | Health::NO_PARAMS | Health::CHANNEL_MISMATCH | Health::MODEL_FAULT;
            if health.0 & untrusted == 0 {
                page.publish_saved(&instance.handoff);
            }
        }
    }
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

/// A call the unit did not run: the frame count, the heartbeat and the
/// control ports still move, so the supervisor sees the fault.
fn finish_call(instance: &mut Instance, frames: usize, health: Health) {
    let (n, s) = (instance.channels, instance.sense);
    instance.frames += frames as u64;
    if let Some(page) = &instance.page {
        page.beat(instance.frames, health, -1);
    }
    let latency = instance.unit.unit().latency_frames() as f32;
    for (port, value) in [(2 * n + s, latency), (2 * n + s + 1, health.0 as f32), (2 * n + s + 2, -1.0)] {
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

    /// The C ABI reads its paths from the environment: tests that set it
    /// take turns.
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn runs_the_failsafe_through_the_c_abi() {
        let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
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

    /// Review finding 5 on PR #31: `activate` reset the thermal model, so a
    /// pause or graph rebuild gave a hot speaker full power back.
    #[test]
    fn activate_keeps_the_thermal_state_across_streams_and_instances() {
        use sgv2_core::blob::Pod;
        let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut p = spp3::Params {
            channels: 2,
            sample_rate: 48_000,
            block_frames: 512,
            thermal_ambient_temp: 40.0,
            thermal_gain_attack_time_s: 0.5,
            thermal_gain_release_time_s: 2.0,
            thermal_lookahead_delay_ms: 1.0,
            thermal_protection_enabled: 1,
            ..Default::default()
        };
        for c in 0..2 {
            p.ch[c] = spp3::ChannelParams {
                source: c as u32,
                t_sett_vc: 20.0,
                t_sett_mg: 10.0,
                tau_tvc: 0.4,
                tau_tmg: 3.0,
                temperature_limit: 120.0,
                hard_temp_limit_headroom: 8.0,
                reb_ref: 4.0,
                rshunt: 0.2,
                ag: 10.0,
                thermal_ff_speedup_factor: 0.5,
                ..Default::default()
            };
        }
        let blob = sgv2_core::blob::build(UnitKind::Spp3, 2, 0, 48_000, p.as_bytes());
        let path = std::env::temp_dir().join(format!("sgv2-handoff-test-{}.bin", std::process::id()));
        std::fs::write(&path, blob).unwrap();
        std::env::set_var("SGV2_PARAMS", &path);
        std::env::set_var("SGV2_ENGINE_PAGE", "/nonexistent/sgv2-test");
        let d = lv2_descriptor(0);
        let l = [1.0f32; 512];
        let r = [1.0f32; 512];
        let mut ol = [0.0f32; 512];
        let mut or = [0.0f32; 512];
        let mut ctl = [0.0f32; 3];
        let bufs = [l.as_ptr() as *mut f32, r.as_ptr() as *mut f32, ol.as_mut_ptr(), or.as_mut_ptr()];
        let ctl_p = ctl.as_mut_ptr();
        let last = bufs[2];
        unsafe {
            let open = || {
                let h = ((*d).instantiate)(d, 48_000.0, core::ptr::null(), core::ptr::null());
                for (k, buf) in bufs.into_iter().enumerate() {
                    ((*d).connect_port)(h, k as u32, buf.cast());
                }
                for k in 0..3 {
                    ((*d).connect_port)(h, 4 + k as u32, ctl_p.add(k).cast());
                }
                h
            };
            let out = || *last.add(511);
            let h = open();
            ((*d).activate.unwrap())(h);
            for _ in 0..(30 * 48_000 / 512) {
                ((*d).run)(h, 512);
            }
            let hot = out();
            assert!(hot < 0.5, "{hot}");
            // pause and resume the same instance
            ((*d).activate.unwrap())(h);
            ((*d).run)(h, 512);
            assert!(out() <= hot + 0.02, "resumed at {}, was {hot}", out());
            ((*d).cleanup)(h);
            // a new instance in the same process starts from that state too
            let h = open();
            ((*d).activate.unwrap())(h);
            ((*d).run)(h, 512);
            assert!(out() <= hot + 0.02, "new instance at {}, was {hot}", out());
            ((*d).cleanup)(h);
        }
        let _ = std::fs::remove_file(&path);
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

    /// The J616s incident: a PipeWire restart mid-playback started the model
    /// cold while the coils were hot. Across processes the record goes
    /// through the page: `saved` (the plugin), then `provided` (the
    /// supervisor), then a new process's `start_stream` with its own slot
    /// empty, aged by the gap.
    #[test]
    fn a_new_process_restores_the_hot_state_the_supervisor_provided() {
        use sgv2_core::blob::Pod;
        let mut p = spp3::Params {
            channels: 2,
            sample_rate: 48_000,
            block_frames: 512,
            thermal_ambient_temp: 40.0,
            thermal_gain_attack_time_s: 0.5,
            thermal_gain_release_time_s: 2.0,
            thermal_lookahead_delay_ms: 1.0,
            thermal_protection_enabled: 1,
            ..Default::default()
        };
        for c in 0..2 {
            p.ch[c] = spp3::ChannelParams {
                source: c as u32,
                t_sett_vc: 20.0,
                t_sett_mg: 10.0,
                tau_tvc: 0.4,
                tau_tmg: 3.0,
                temperature_limit: 120.0,
                hard_temp_limit_headroom: 8.0,
                reb_ref: 4.0,
                rshunt: 0.2,
                ag: 10.0,
                thermal_ff_speedup_factor: 0.5,
                ..Default::default()
            };
        }
        let blob = sgv2_core::blob::build(UnitKind::Spp3, 2, 0, 48_000, p.as_bytes());
        let one = [1.0f32; 512];
        let run = |u: &mut AnyUnit| -> (f32, Health) {
            let mut a = [0.0f32; 512];
            let mut b = [0.0f32; 512];
            let mut out: [&mut [f32]; 2] = [&mut a, &mut b];
            let h = u.as_dyn().process(Io { frames: 512, input: &[&one, &one], sense: &[], output: &mut out });
            (a[511], h)
        };
        let path = std::env::temp_dir().join(format!("sgv2-xproc-page-{}", std::process::id()));
        let path = path.to_str().unwrap().to_string();
        let page = page::EnginePage::create(&path).unwrap();
        // process A heats the speakers and publishes its record
        let (mut a, _, crc) = load_unit(Some(&blob), 48_000.0, 2, 0, Role::Protect);
        assert_ne!(crc, 0);
        a.as_dyn().reset();
        let mut hot = 1.0;
        for _ in 0..(30 * 48_000 / 512) {
            hot = run(&mut a).0;
        }
        assert!(hot < 0.5, "{hot}");
        let mut rec = HandOff::EMPTY;
        assert!(a.unit().save_state(&mut rec));
        page.publish_saved(&rec);
        // the supervisor reads it and provides it for the blob
        let (got, stamp) = page.header().saved.read().unwrap();
        assert_eq!(got, rec);
        page.provide(&got, stamp, crc);
        // process B: its own slot is empty, so it restores what was provided
        let empty = handoff::Slot::new();
        let (mut b, _, crc_b) = load_unit(Some(&blob), 48_000.0, 2, 0, Role::Protect);
        assert!(start_stream(b.as_dyn(), &empty, page.provided_for(crc_b)));
        let restored = run(&mut b).0;
        assert!(restored <= hot + 0.02, "restored at {restored}, was {hot}");
        // a cold start, for comparison, plays louder
        let (mut cold, _, _) = load_unit(Some(&blob), 48_000.0, 2, 0, Role::Protect);
        assert!(!start_stream(cold.as_dyn(), &empty, None));
        let cold_out = run(&mut cold).0;
        assert!(cold_out > restored + 0.1, "cold {cold_out}, restored {restored}");
        // another blob gets nothing; a record past ten minutes is not restored
        assert!(page.provided_for(crc ^ 1).is_none());
        let (mut c, _, _) = load_unit(Some(&blob), 48_000.0, 2, 0, Role::Protect);
        assert!(!start_stream(c.as_dyn(), &empty, Some((got, PROVIDED_MAX_AGE_S + 1.0))));
        // the record is aged by the unit's own decay: an older record
        // restores cooler (this unit's time constants are seconds)
        let (mut d, _, _) = load_unit(Some(&blob), 48_000.0, 2, 0, Role::Protect);
        assert!(start_stream(d.as_dyn(), &empty, Some((got, 20.0))));
        let aged = run(&mut d).0;
        assert!(aged >= restored - 1e-6, "aged {aged}, restored {restored}");
        let _ = std::fs::remove_file(&path);
    }
}
