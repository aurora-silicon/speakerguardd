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
//!   takes ownership of the amps. A watcher thread re-attaches when the file
//!   is replaced or appears later ([`attach`]), so a restarted supervisor
//!   sees the heartbeat without a PipeWire restart.
//!
//! A blob that is missing, corrupt, for another unit kind, or for another
//! channel/sense geometry or rate gives the fail-safe unit (20 dB), as macOS
//! does without its essential parameters. So does a blob whose pre-stage
//! ([`sgv2_core::prestage`]: the Mac's own filtering ahead of the unit,
//! which `run` applies to the program before the unit sees it) is unstable
//! or would add drive.
//!
//! `activate` clears the audio histories but not the thermal model: `run`
//! publishes the unit's thermal state after every call and `activate`
//! restores the latest record that fits, cooled for the time since (see
//! [`handoff`]). A pause, a suspend or a graph rebuild in the same process
//! therefore never gives a hot speaker full power back.

pub mod attach;
pub mod gate;
pub mod handoff;
pub mod page;
pub mod preroll;
pub mod ttl;
pub mod voicing;

use core::ffi::{c_char, c_void};
use sgv2_core::prestage::PreStage;
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
    // appended: the descriptor indexes of the variants before it stay
    ("https://aurora-silicon.org/lv2/sgv2#1ch\0", 1, 0, Role::Protect),
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
/// the blob's deployment CRC as its identity. Not real time.
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
    let Ok(engine_crc) = sgv2_core::blob::engine_crc(bytes) else { return fail(LoadError::Blob) };
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
                (AnyUnit::Spp3(Box::new(spp3::Unit::for_machine(p, h.machine()))), None, engine_crc)
            }
            Ok(_) => fail(LoadError::Refused),
            Err(_) => fail(LoadError::Blob),
        }
    } else if h.unit == UnitKind::Lsm2 as u32 {
        match lsm2::Unit::from_blob(bytes) {
            Ok(unit) if unit.companion().is_some() || !unit.is_failsafe() => {
                (AnyUnit::Lsm2(Box::new(unit)), None, engine_crc)
            }
            Ok(_) => fail(LoadError::Refused),
            Err(_) => fail(LoadError::Blob),
        }
    } else if h.unit == UnitKind::Atsp as u32 {
        match atsp::Params::from_blob(bytes) {
            Ok(p) if atsp::Unit::init_error(&p).is_none() => {
                (AnyUnit::Atsp(Box::new(atsp::Unit::new(p))), None, engine_crc)
            }
            Ok(_) => fail(LoadError::Refused),
            Err(_) => fail(LoadError::Blob),
        }
    } else {
        fail(LoadError::NotImplemented)
    }
}

/// The pre-stage for a unit `load_unit` built from `blob`: the blob's own,
/// checked; `None` when it fails its checks (the caller then runs the
/// fail-safe). A unit not built from a blob (the fail-safe, the
/// placeholder) runs without one. Not real time.
pub fn load_prestage(blob: Option<&[u8]>, rate: f64, channels: usize, unit: &AnyUnit) -> Option<PreStage> {
    let from_blob = matches!(unit, AnyUnit::Spp3(_) | AnyUnit::Lsm2(_) | AnyUnit::Atsp(_));
    if !from_blob {
        return Some(PreStage::new(sgv2_core::prestage::Params::identity(channels)));
    }
    sgv2_core::blob::voicing(blob?).ok()?;
    let p = sgv2_core::blob::prestage(blob?, rate).ok()?;
    (p.channels == channels).then(|| PreStage::with_rate(p, rate))
}

/// Frames `run` filters at once; a longer host call is run in chunks.
pub const SCRATCH_FRAMES: usize = 4096;

struct Instance {
    channels: usize,
    sense: usize,
    ports: [*mut f32; MAX_PORTS],
    unit: AnyUnit,
    /// The supervisor's engine page, re-attached when the file is replaced.
    attach: attach::Attach,
    telemetry: Telemetry,
    last_frame: u64,
    frames: u64,
    /// Scratch for the thermal hand-off record `run` publishes.
    handoff: HandOff,
    /// The blob deployment CRC the unit runs (0: none).
    params_crc: u32,
    /// The Mac's filtering ahead of the unit, and its output.
    pre: Box<PreStage>,
    scratch: Box<[[f32; SCRATCH_FRAMES]; MAX_CHANNELS]>,
    /// Private copies of the sense ports and the unit's output: LV2 lets a
    /// host connect ports in place (an output may share any input's buffer),
    /// so every input is copied before any output is written.
    sense_scratch: Box<[[f32; SCRATCH_FRAMES]; MAX_SENSE]>,
    out_scratch: Box<[[f32; SCRATCH_FRAMES]; MAX_CHANNELS]>,
    /// Follows the kernel's speaker grant (a watcher thread).
    gate: gate::Gate,
    /// The gain the grant puts on the program.
    ramp: gate::Ramp,
    /// The first frames of a wake, held until the amps play.
    preroll: Box<preroll::PreRoll>,
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
    let (mut unit, _err, mut params_crc) = load_unit(blob.as_deref(), rate, channels, sense, role);
    let pre = match load_prestage(blob.as_deref(), rate, channels, &unit) {
        Some(p) => p,
        None => {
            unit = AnyUnit::FailSafe(FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS));
            params_crc = 0;
            PreStage::new(sgv2_core::prestage::Params::identity(channels))
        }
    };
    let attach = attach::Attach::new(
        page_path,
        attach::Header {
            params_crc,
            latency: unit.unit().latency_frames(),
            channels: channels as u32,
            sense: sense as u32,
        },
    );
    let instance = Box::new(Instance {
        channels,
        sense,
        ports: [core::ptr::null_mut(); MAX_PORTS],
        unit,
        attach,
        telemetry: Telemetry::new(),
        last_frame: u64::MAX,
        frames: 0,
        handoff: HandOff::EMPTY,
        params_crc,
        pre: Box::new(pre),
        scratch: Box::new([[0.0; SCRATCH_FRAMES]; MAX_CHANNELS]),
        sense_scratch: Box::new([[0.0; SCRATCH_FRAMES]; MAX_SENSE]),
        out_scratch: Box::new([[0.0; SCRATCH_FRAMES]; MAX_CHANNELS]),
        gate: gate::Gate::watch(),
        ramp: gate::Ramp::new(rate),
        preroll: Box::new(preroll::PreRoll::new(channels, rate)),
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
    let provided = instance.attach.enter().and_then(|p| p.provided_for(instance.params_crc));
    instance.attach.leave();
    start_stream(instance.unit.as_dyn(), &handoff::SLOT, provided);
    instance.pre.reset();
    instance.preroll.reset();
    // histories cleared under a program that may be mid-waveform: ramp in
    instance.ramp.restart();
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
    let attach: *const attach::Attach = &instance.attach;
    // SAFETY: `attach` lives as long as the instance and is not touched
    // by `run_call`; the page stays mapped until `leave`.
    let page = unsafe { (*attach).enter() };
    run_call(instance, page, frames);
    // SAFETY: as above.
    unsafe { (*attach).leave() };
}

fn run_call(instance: &mut Instance, page: Option<&page::EnginePage>, frames: u32) {
    let frames = frames as usize;
    if frames == 0 {
        return run_chunk(instance, page, 0, 0);
    }
    let mut off = 0;
    while off < frames {
        let k = (frames - off).min(SCRATCH_FRAMES);
        run_chunk(instance, page, off, k);
        off += k;
    }
}

/// `ports[p]`, `off` samples in (null stays null).
fn at(ports: &[*mut f32; MAX_PORTS], p: usize, off: usize) -> *mut f32 {
    let q = ports[p];
    if q.is_null() {
        q
    } else {
        // SAFETY: connected audio ports hold the call's frames; `off` is
        // inside the call.
        unsafe { q.add(off) }
    }
}

fn run_chunk(instance: &mut Instance, page: Option<&page::EnginePage>, off: usize, frames: usize) {
    let (n, s) = (instance.channels, instance.sense);
    let empty: &[f32] = &[];
    let mut sense = [empty; MAX_SENSE];
    // A missing program port: every connected output gets the 20 dB
    // fail-safe of its own input (silence without one), never a stale
    // buffer, and the call still reports and beats.
    if (0..2 * n).any(|p| instance.ports[p].is_null()) {
        for c in 0..n {
            let (i, o) = (at(&instance.ports, c, off), at(&instance.ports, n + c, off));
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
        finish_call(instance, page, frames, Health(Health::FAILSAFE | Health::CHANNEL_MISMATCH));
        return;
    }
    // LV2 lets a host connect ports in place: an output may share its own
    // input's buffer, another channel's, or a sense port's. Every program
    // and sense input is copied into private storage first; the unit then
    // writes private storage, and only that is copied to the output ports.
    for c in 0..n {
        // SAFETY: connected audio ports hold `frames` samples (checked
        // above: no program port is null). Read only, before any write.
        let src = unsafe { core::slice::from_raw_parts(at(&instance.ports, c, off), frames) };
        instance.scratch[c][..frames].copy_from_slice(src);
    }
    let mut sense_n = 0;
    for k in 0..s {
        let p = at(&instance.ports, 2 * n + k, off);
        if p.is_null() {
            break;
        }
        // SAFETY: as above; copied before any output port is written.
        let src = unsafe { core::slice::from_raw_parts(p, frames) };
        instance.sense_scratch[k][..frames].copy_from_slice(src);
        sense_n = k + 1;
    }
    // The Mac's own filtering ahead of the unit (the pre-stage): the unit
    // protects what the speakers will actually be driven with.
    let mut filtered = [empty; MAX_CHANNELS];
    {
        let (pre, scratch) = (&mut instance.pre, &mut instance.scratch);
        // filters, gains and the graph's compressor boxes in graph order
        // (each box linked across every channel)
        let mut bufs: [&mut [f32]; MAX_CHANNELS] = Default::default();
        for (b, s) in bufs.iter_mut().zip(scratch.iter_mut()).take(n) {
            *b = &mut s[..frames];
        }
        // a wake's first frames, held while the amps are not playing yet
        // (see preroll), ahead of everything that models what they play
        let amps = page.map_or(preroll::Amps::Unknown, |p| p.amps());
        instance.preroll.process(&mut bufs[..n], frames, amps);
        // the kernel's grant: muted while withdrawn, ramped in after a
        // grant (see gate), ahead of every filter so none sees a step
        instance.ramp.follow(instance.gate.shared().load());
        instance.ramp.apply(&mut bufs[..n], frames);
        pre.process_block(&mut bufs[..n], frames);
    }
    for (f, buf) in filtered.iter_mut().zip(instance.scratch.iter()).take(n) {
        *f = &buf[..frames];
    }
    for (slot, src) in sense.iter_mut().zip(instance.sense_scratch.iter()).take(sense_n) {
        *slot = &src[..frames];
    }
    let mut out_store: [&mut [f32]; MAX_CHANNELS] = Default::default();
    for (slot, dst) in out_store.iter_mut().zip(instance.out_scratch.iter_mut()).take(n) {
        *slot = &mut dst[..frames];
    }
    // An incomplete sense bus is no bus: the unit runs open loop.
    let sense_slice = if sense_n == s { &sense[..s] } else { &sense[..0] };
    let unit = instance.unit.as_dyn();
    let latency = unit.latency_frames() as f32;
    let health = unit.process(Io { frames, input: &filtered[..n], sense: sense_slice, output: &mut out_store[..n] });
    for c in 0..n {
        // SAFETY: connected output ports hold `frames` samples; the source
        // is private storage. A raw copy creates no reference to host
        // memory, so two outputs sharing one buffer are not two `&mut`.
        unsafe {
            core::ptr::copy_nonoverlapping(instance.out_scratch[c].as_ptr(), at(&instance.ports, n + c, off), frames)
        };
    }
    instance.frames += frames as u64;
    if instance.unit.unit().save_state(&mut instance.handoff) {
        handoff::SLOT.publish(&instance.handoff);
        // for a later process, through the supervisor; never a record the
        // unit does not trust (save_state is false then) nor one from a
        // faulted or fail-safe call
        if let Some(page) = page {
            // MODEL_RANGE still protects: kept
            let untrusted = Health::FAILSAFE | Health::NO_PARAMS | Health::CHANNEL_MISMATCH | Health::MODEL_FAULT;
            if health.0 & untrusted == 0 {
                page.publish_saved(&instance.handoff);
            }
        }
    }
    let delay = instance.unit.sense_delay().map_or(-1, |d| d as i32);
    if let Some(page) = page {
        instance.unit.unit().telemetry(&mut instance.telemetry);
        if instance.telemetry.frame != instance.last_frame {
            instance.last_frame = instance.telemetry.frame;
            page.push(&instance.telemetry);
        }
        let w = &instance.preroll;
        page.publish_wakes(w.wakes, w.last_wake, w.overflow);
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
fn finish_call(instance: &mut Instance, page: Option<&page::EnginePage>, frames: usize, health: Health) {
    let (n, s) = (instance.channels, instance.sense);
    instance.frames += frames as u64;
    if let Some(page) = page {
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

static DESCRIPTORS: [Lv2Descriptor; 7] =
    [descriptor!(0), descriptor!(1), descriptor!(2), descriptor!(3), descriptor!(4), descriptor!(5), descriptor!(6)];
const _: () = assert!(DESCRIPTORS.len() == VARIANTS.len());

/// The LV2 entry point.
#[no_mangle]
pub extern "C" fn lv2_descriptor(index: u32) -> *const Lv2Descriptor {
    DESCRIPTORS
        .get(index as usize)
        .map_or_else(|| voicing::descriptor(index as usize - DESCRIPTORS.len()), |d| d as *const _)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The C ABI reads its paths from the environment: tests that set it
    /// take turns.
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// The plugin builds spp3 for the board its blob names, so only a board
    /// proven for the relaxed closed loop may relax; a blob for any other
    /// board, or naming none, runs capped.
    #[test]
    fn the_blob_board_decides_the_closed_loop_relaxation() {
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
        for (c, ch) in p.ch[..2].iter_mut().enumerate() {
            *ch = spp3::ChannelParams {
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
        let relaxes = |blob: &[u8]| match load_unit(Some(blob), 48_000.0, 2, 0, Role::Protect) {
            (AnyUnit::Spp3(u), None, _) => u.closed_loop_may_relax(),
            _ => panic!("the spp3 blob did not load"),
        };
        let for_board = |b: &str| {
            sgv2_core::blob::build_for(
                UnitKind::Spp3,
                2,
                0,
                48_000,
                sgv2_core::blob::machine_field(b).unwrap(),
                0,
                p.as_bytes(),
            )
        };
        assert!(relaxes(&for_board("j616s")));
        for b in ["j316s", "j413", "j614s", "j700"] {
            assert!(!relaxes(&for_board(b)), "{b}");
        }
        assert!(!relaxes(&sgv2_core::blob::build(UnitKind::Spp3, 2, 0, 48_000, p.as_bytes())));
    }

    #[test]
    fn runs_the_failsafe_through_the_c_abi() {
        let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        std::env::set_var("SGV2_PARAMS", "/nonexistent/sgv2-test");
        std::env::set_var("SGV2_ENGINE_PAGE", "/nonexistent/sgv2-test");
        let d = lv2_descriptor(0);
        assert!(!d.is_null());
        assert!(!lv2_descriptor(VARIANTS.len() as u32).is_null());
        assert!(lv2_descriptor((VARIANTS.len() + voicing::VARIANTS.len()) as u32).is_null());
        unsafe {
            let h = ((*d).instantiate)(d, 48_000.0, core::ptr::null(), core::ptr::null());
            // past the ramp in that follows activate (gate::RAMP_IN_MS)
            const N: usize = 1024;
            let mut l = [1.0f32; N];
            let mut r = [-1.0f32; N];
            let mut ol = [0.0f32; N];
            let mut or = [0.0f32; N];
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
            ((*d).run)(h, N as u32);
            // the stream starts from silence, then the fail-safe's 20 dB
            assert!(ol[0].abs() < 1e-5 && ol[..480].windows(2).all(|w| w[1] >= w[0]));
            assert!((ol[N - 1] - 0.1).abs() < 1e-7 && (or[N - 1] + 0.1).abs() < 1e-7);
            assert_eq!(latency, 0.0);
            assert_eq!(delay, -1.0);
            assert_eq!(health as u32 & Health::FAILSAFE, Health::FAILSAFE);
            ((*d).cleanup)(h);
        }
    }

    /// A wake from the idle power-down: while the supervisor says the amps
    /// are not playing, the program is kept and the unit gets silence; once
    /// they play, the kept frames come out first, from the stream's first
    /// frame, with no ramp cutting into them.
    #[test]
    fn a_wake_plays_the_first_frames_once_the_amps_play() {
        let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = std::env::temp_dir().join(format!("sgv2-wake-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("engine");
        let sup = page::EnginePage::create(path.to_str().unwrap()).unwrap();
        sup.set_amps(false);
        std::env::set_var("SGV2_PARAMS", "/nonexistent/sgv2-test");
        std::env::set_var("SGV2_ENGINE_PAGE", &path);
        let d = lv2_descriptor(0);
        unsafe {
            let h = ((*d).instantiate)(d, 48_000.0, core::ptr::null(), core::ptr::null());
            const N: usize = 512;
            let mut l = [0.0f32; N];
            let mut r = [0.0f32; N];
            let mut ol = [0.0f32; N];
            let mut or = [0.0f32; N];
            let mut ctl = [0.0f32; 3];
            for (p, buf) in [(0, l.as_mut_ptr()), (1, r.as_mut_ptr()), (2, ol.as_mut_ptr()), (3, or.as_mut_ptr())] {
                ((*d).connect_port)(h, p, buf.cast());
            }
            for (k, c) in ctl.iter_mut().enumerate() {
                ((*d).connect_port)(h, 4 + k as u32, (c as *mut f32).cast());
            }
            ((*d).activate.unwrap())(h);
            // idle, then the stream starts at frame 700 with 1, 2, 3, ... (scaled)
            let prog = |i: usize| if i < 700 { 0.0 } else { (i - 699) as f32 * 1e-4 };
            let mut out = Vec::new();
            let mut i = 0;
            for call in 0..8 {
                if call == 4 {
                    sup.set_amps(true);
                }
                for k in 0..N {
                    l[k] = prog(i + k);
                    r[k] = -prog(i + k);
                }
                ((*d).run)(h, N as u32);
                out.extend_from_slice(&ol);
                assert!(ol.iter().zip(&or).all(|(a, b)| *a == -*b));
                i += N;
            }
            // silence while held, then the program from its first frame (the
            // fail-safe's 20 dB on it), in order
            assert!(out[..4 * N].iter().all(|&x| x == 0.0));
            for (k, &y) in out[4 * N..].iter().enumerate() {
                let want = sgv2_core::dsp::bound1((k + 1) as f32 * 1e-4) * sgv2_core::FAILSAFE_GAIN;
                assert!((y - want).abs() < 1e-9, "frame {k}: {y} != {want}");
            }
            let hd = sup.header();
            assert_eq!(hd.wakes.load(core::sync::atomic::Ordering::Acquire), 1);
            assert_eq!(hd.wake_frames.load(core::sync::atomic::Ordering::Relaxed), (4 * N - 700) as u32);
            assert_eq!(hd.wake_overflow.load(core::sync::atomic::Ordering::Relaxed), 0);
            ((*d).cleanup)(h);
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Review finding 5 on PR #31: `activate` reset the thermal model, so a
    /// pause or graph rebuild gave a hot speaker full power back.
    #[test]
    fn activate_keeps_the_thermal_state_across_streams_and_instances() {
        use sgv2_core::blob::Pod;
        let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let _clock = handoff::test_clock::Clock::at(1_000_000_000);
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
        for (c, ch) in p.ch[..2].iter_mut().enumerate() {
            *ch = spp3::ChannelParams {
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
            // No elapsed time at the hand-off, regardless of host load.
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

    /// The pre-stage runs ahead of the unit: a high-pass on channel 0 takes
    /// a low tone away there and leaves channel 1 alone; a pre-stage that
    /// would add drive is the fail-safe.
    #[test]
    fn the_prestage_filters_the_program_ahead_of_the_unit() {
        use sgv2_core::blob::Pod;
        use sgv2_core::prestage::{Params as Pre, Section, FLAG_FROM_TUNING};
        let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut p = spp3::Params {
            channels: 2,
            sample_rate: 48_000,
            block_frames: 512,
            thermal_ambient_temp: 25.0,
            thermal_gain_attack_time_s: 0.5,
            thermal_gain_release_time_s: 2.0,
            thermal_lookahead_delay_ms: 1.0,
            thermal_protection_enabled: 1,
            ..Default::default()
        };
        for (c, ch) in p.ch[..2].iter_mut().enumerate() {
            *ch = spp3::ChannelParams {
                source: c as u32,
                t_sett_vc: 20.0,
                t_sett_mg: 10.0,
                tau_tvc: 0.4,
                tau_tmg: 3.0,
                temperature_limit: 120.0,
                hard_temp_limit_headroom: 8.0,
                reb_ref: 4.0,
                rshunt: 0.2,
                ag: 1.0,
                thermal_ff_speedup_factor: 0.5,
                ..Default::default()
            };
        }
        // fourth-order Butterworth high-pass at 1 kHz (two sections)
        let fs = 48_000.0f64;
        let k = (core::f64::consts::PI * 1000.0 / fs).tan();
        let mut pre = Pre::identity(2);
        pre.flags = FLAG_FROM_TUNING;
        for (i, q2) in [0.765_366_864_730_18f64, 1.847_759_065_022_57].into_iter().enumerate() {
            let n = 1.0 + q2 * k + k * k;
            pre.channel[0].sections[i] = Section {
                b0: 1.0 / n,
                b1: -2.0 / n,
                b2: 1.0 / n,
                a1: 2.0 * (k * k - 1.0) / n,
                a2: (1.0 - q2 * k + k * k) / n,
            };
        }
        pre.channel[0].count = 2;
        let blob = sgv2_core::blob::build(UnitKind::Spp3, 2, 0, 48_000, p.as_bytes());
        let blob = sgv2_core::blob::with_prestage(&blob, &pre).unwrap();
        let (u, e, _) = load_unit(Some(&blob), 48_000.0, 2, 0, Role::Protect);
        assert_eq!(e, None);
        assert!(load_prestage(Some(&blob), 48_000.0, 2, &u).unwrap().params().from_tuning());
        // a gain over unity is refused, and the plugin runs the fail-safe
        let mut loud = pre.clone();
        loud.channel[1].gain = 2.0;
        let bad = sgv2_core::blob::with_prestage(&blob, &loud).unwrap();
        let (u_bad, e, _) = load_unit(Some(&bad), 48_000.0, 2, 0, Role::Protect);
        assert_eq!(e, None);
        assert!(load_prestage(Some(&bad), 48_000.0, 2, &u_bad).is_none());

        let path = std::env::temp_dir().join(format!("sgv2-prestage-test-{}.bin", std::process::id()));
        let run = |blob: &[u8]| -> (f32, f32, u32) {
            std::fs::write(&path, blob).unwrap();
            std::env::set_var("SGV2_PARAMS", &path);
            std::env::set_var("SGV2_ENGINE_PAGE", "/nonexistent/sgv2-test");
            let d = lv2_descriptor(0);
            let n = 48_000usize;
            let tone: std::vec::Vec<f32> =
                (0..n).map(|i| (0.25 * (2.0 * core::f64::consts::PI * 100.0 * i as f64 / fs).sin()) as f32).collect();
            let mut ol = std::vec![0.0f32; n];
            let mut or = std::vec![0.0f32; n];
            let mut ctl = [0.0f32; 3];
            unsafe {
                let h = ((*d).instantiate)(d, 48_000.0, core::ptr::null(), core::ptr::null());
                ((*d).connect_port)(h, 0, tone.as_ptr() as *mut c_void);
                ((*d).connect_port)(h, 1, tone.as_ptr() as *mut c_void);
                ((*d).connect_port)(h, 2, ol.as_mut_ptr().cast());
                ((*d).connect_port)(h, 3, or.as_mut_ptr().cast());
                for k in 0..3 {
                    ((*d).connect_port)(h, 4 + k as u32, ctl.as_mut_ptr().add(k).cast());
                }
                ((*d).activate.unwrap())(h);
                // one long host call: the plugin runs it in scratch-sized chunks
                ((*d).run)(h, n as u32);
                ((*d).cleanup)(h);
            }
            // the 100 Hz component (spp3 may add its pilot tone)
            let amp = |v: &[f32]| {
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for (i, x) in v[n / 2..].iter().enumerate() {
                    let w = 2.0 * core::f64::consts::PI * 100.0 * (i + n / 2) as f64 / fs;
                    re += *x as f64 * w.cos();
                    im += *x as f64 * w.sin();
                }
                (2.0 * (re * re + im * im).sqrt() / (n / 2) as f64) as f32
            };
            (amp(&ol), amp(&or), ctl[1] as u32)
        };
        let (l, r, health) = run(&blob);
        assert_eq!(health & Health::FAILSAFE, 0);
        // the unit treats both channels alike; the pre-stage takes channel
        // 0 down by 80 dB (a decade under its fourth-order corner)
        assert!(r > 1e-3, "{r}");
        assert!(l < r * 1.2e-4 && l > r * 0.8e-4, "{l} vs {r}");
        let (_, r, health) = run(&bad);
        assert_eq!(health & Health::FAILSAFE, Health::FAILSAFE);
        assert!(r <= 0.025 + 1e-6, "{r}");
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
        let clock = handoff::test_clock::Clock::at(1_000_000_000);
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
        for (c, ch) in p.ch[..2].iter_mut().enumerate() {
            *ch = spp3::ChannelParams {
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
        assert_eq!(stamp, 1_000_000_000);
        page.provide(&got, stamp, crc);
        // process B: its own slot is empty, so it restores what was provided
        let empty = handoff::Slot::new();
        let (mut b, _, crc_b) = load_unit(Some(&blob), 48_000.0, 2, 0, Role::Protect);
        assert_eq!(page.provided_for(crc_b), Some((got, 0.0)));
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
        // the record is aged by the unit's own decay: an older record
        // restores cooler (this unit's time constants are seconds)
        clock.advance(20_000_000_000);
        assert_eq!(page.provided_for(crc), Some((got, 20.0)));
        let (mut d, _, _) = load_unit(Some(&blob), 48_000.0, 2, 0, Role::Protect);
        assert!(start_stream(d.as_dyn(), &empty, page.provided_for(crc)));
        let aged = run(&mut d).0;
        assert!(aged > restored + 0.1, "aged {aged}, restored {restored}");
        // Test the inclusive age limit through the page's timestamp path.
        clock.advance(580_000_000_000);
        let (mut c, _, _) = load_unit(Some(&blob), 48_000.0, 2, 0, Role::Protect);
        assert!(start_stream(c.as_dyn(), &empty, page.provided_for(crc)));
        clock.advance(1_000_000_000);
        assert!(!start_stream(c.as_dyn(), &empty, page.provided_for(crc)));
        let _ = std::fs::remove_file(&path);
    }
}
