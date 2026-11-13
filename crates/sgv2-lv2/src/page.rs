//! The engine page: one shared file mapping per plugin instance, created by
//! the supervisor and mapped by the plugin in `instantiate()`.
//!
//! ```text
//! offset 0     PageHeader (one 4 KiB page; written by the plugin only)
//! offset 4096  Ring<Telemetry, RING> (one record per completed unit block)
//! ```
//!
//! The plugin writes everything but `provided`. The supervisor reads the
//! header's `seq`/`frames`/`mono_ns` to judge freshness and drains the ring
//! at its own pace; a slow reader loses the oldest records and sees the gap
//! in the sequence numbers, the RT thread never waits.
//!
//! Thermal state across processes (layout 3): `run` publishes the unit's
//! hand-off record into `saved` (CLOCK_BOOTTIME stamps). The supervisor
//! keeps the last good one under /run and writes it into `provided`, with
//! the blob CRC it belongs to, so a new PipeWire process starts from the
//! speakers' real temperature instead of cold.

use core::sync::atomic::{AtomicI32, AtomicU32, AtomicU64, Ordering};
use std::fs::OpenOptions;
use std::os::fd::AsRawFd;

use sgv2_core::ring::Ring;
use sgv2_core::{HandOff, Health, Telemetry};

use crate::handoff::{boottime_ns, Slot};

pub const MAGIC: u64 = u64::from_le_bytes(*b"SGV2ENG1");
pub const LAYOUT: u32 = 3;
/// Telemetry records kept: 256 unit blocks, 2.7 s at 512 frames / 48 kHz.
pub const RING: usize = 256;
pub const HEADER_BYTES: usize = 4096;
pub const PAGE_BYTES: usize = HEADER_BYTES + core::mem::size_of::<Ring<Telemetry, RING>>();

#[repr(C)]
pub struct PageHeader {
    pub magic: AtomicU64,
    pub layout: AtomicU32,
    /// CRC-32 of the parameter blob payload the unit runs (0: none).
    pub params_crc: AtomicU32,
    pub latency: AtomicU32,
    pub channels: AtomicU32,
    pub sense: AtomicU32,
    pub pid: AtomicU32,
    /// Incremented once per `run` call, after the frames below.
    pub seq: AtomicU64,
    pub frames: AtomicU64,
    /// CLOCK_MONOTONIC of the last `run`, ns.
    pub mono_ns: AtomicU64,
    pub health: AtomicU32,
    pub sense_delay: AtomicI32,
    /// Most restrictive gain of the active channels in the last record, as
    /// f32 bits; NaN when an active channel's gain is non-finite (a fault).
    pub min_gain_bits: AtomicU32,
    /// The unit's latest hand-off record, written by the plugin.
    pub saved: Slot,
    /// The record for a new process to restore, written by the supervisor.
    pub provided: Slot,
    /// The blob payload CRC `provided` was saved under (0: none).
    pub provided_crc: AtomicU32,
    /// Whether the amps play, written by the supervisor every tick:
    /// [`AMPS_HELD`] (not granted, or their PCM not started yet) or
    /// [`AMPS_PLAYING`]; 0 from a supervisor that does not say. The plugin
    /// holds the first frames of a wake while it reads held ([`crate::preroll`]).
    pub amps: AtomicU32,
    /// CLOCK_MONOTONIC of the supervisor's last `amps` write, ns: an older
    /// word than [`AMPS_STALE_NS`] says nothing.
    pub amps_ns: AtomicU64,
    /// Frames the last wake kept, written by the plugin when the amps start
    /// playing, then `wakes` bumped (release).
    pub wake_frames: AtomicU32,
    pub wakes: AtomicU32,
    /// Frames a wake could not keep (beyond its capacity), since the
    /// plugin started.
    pub wake_overflow: AtomicU64,
    /// The sense binding, written by the supervisor every tick: the blob
    /// payload CRC whose sense input it has proven (the self-check's wiring,
    /// reported linked and at unity by the session helper for the engine
    /// running that blob), or 0 for none. See [`EnginePage::sense_bind`].
    pub sense_bind_crc: AtomicU32,
    /// CLOCK_MONOTONIC of the supervisor's last `sense_bind_crc` write, ns:
    /// an older word than [`SENSE_BIND_STALE_NS`] binds nothing.
    pub sense_bind_ns: AtomicU64,
    /// What the plugin did with the binding ([`SENSE_UNBOUND`] ..
    /// [`SENSE_REFUSED`]); 0 from a plugin that does not say.
    pub sense_bound: AtomicU32,
}

pub const AMPS_HELD: u32 = 1;
pub const AMPS_PLAYING: u32 = 2;
/// The supervisor ticks every 10 ms; a word older than this is no word.
pub const AMPS_STALE_NS: u64 = 500_000_000;
/// A sense binding older than this binds nothing (the supervisor writes it
/// every 10 ms tick).
pub const SENSE_BIND_STALE_NS: u64 = 500_000_000;
/// [`PageHeader::sense_bound`]: no binding offered, or it was withdrawn.
pub const SENSE_UNBOUND: u32 = 1;
/// The unit's LSID companion feeds its estimator from the bound source.
pub const SENSE_BOUND: u32 = 2;
/// Offered, but the unit has no LSID companion (nothing to bind).
pub const SENSE_NO_COMPANION: u32 = 3;
/// Offered, and the unit refused it (its companion has no estimator or
/// names no source).
pub const SENSE_REFUSED: u32 = 4;

const _: () = assert!(core::mem::size_of::<PageHeader>() <= HEADER_BYTES);

pub struct EnginePage {
    base: *mut u8,
    len: usize,
}

// SAFETY: the mapping is process-shared memory accessed only through
// atomics and the seqlocked ring.
unsafe impl Send for EnginePage {}

fn mono_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: valid out-pointer; CLOCK_MONOTONIC is served by the vDSO on
    // aarch64 and x86_64 Linux (no system call).
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

impl EnginePage {
    /// Map an existing page file of at least `PAGE_BYTES`. The plugin never
    /// creates or grows it: the supervisor owns the file and its label.
    pub fn open(path: &str) -> std::io::Result<Self> {
        let f = OpenOptions::new().read(true).write(true).open(path)?;
        let len = f.metadata()?.len() as usize;
        if len < PAGE_BYTES {
            return Err(std::io::Error::other("engine page too small"));
        }
        Self::map(f.as_raw_fd(), PAGE_BYTES)
    }

    /// Create (or truncate) a page file and map it: the supervisor's side,
    /// and tests.
    pub fn create(path: &str) -> std::io::Result<Self> {
        let f = OpenOptions::new().read(true).write(true).create(true).truncate(true).open(path)?;
        f.set_len(PAGE_BYTES as u64)?;
        Self::map(f.as_raw_fd(), PAGE_BYTES)
    }

    fn map(fd: i32, len: usize) -> std::io::Result<Self> {
        // SAFETY: mapping a file we hold open, shared, read/write.
        let p = unsafe {
            libc::mmap(core::ptr::null_mut(), len, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, fd, 0)
        };
        if p == libc::MAP_FAILED {
            return Err(std::io::Error::last_os_error());
        }
        // Keep the page resident: the RT thread must not take a fault on it.
        // SAFETY: the range was just mapped.
        unsafe { libc::mlock(p, len) };
        Ok(EnginePage { base: p.cast(), len })
    }

    pub fn header(&self) -> &PageHeader {
        // SAFETY: the mapping is at least HEADER_BYTES, page-aligned, and
        // PageHeader is all atomics (any bit pattern is valid).
        unsafe { &*self.base.cast::<PageHeader>() }
    }

    pub fn ring(&self) -> &Ring<Telemetry, RING> {
        // SAFETY: the ring follows the header page; zero-filled memory is an
        // empty ring; the type is repr(C) and page-aligned here.
        unsafe { &*self.base.add(HEADER_BYTES).cast::<Ring<Telemetry, RING>>() }
    }

    /// Called from `instantiate`.
    pub fn start(&self, params_crc: u32, latency: u32, channels: u32, sense: u32) {
        let h = self.header();
        h.layout.store(LAYOUT, Ordering::Relaxed);
        h.params_crc.store(params_crc, Ordering::Relaxed);
        h.latency.store(latency, Ordering::Relaxed);
        h.channels.store(channels, Ordering::Relaxed);
        h.sense.store(sense, Ordering::Relaxed);
        h.pid.store(std::process::id(), Ordering::Relaxed);
        h.magic.store(MAGIC, Ordering::Release);
    }

    /// Real time: one telemetry record.
    #[inline]
    pub fn push(&self, t: &Telemetry) {
        self.ring().push(t);
        let g = min_gain(t, self.header().channels.load(Ordering::Relaxed) as usize);
        self.header().min_gain_bits.store(g.to_bits(), Ordering::Relaxed);
    }

    /// Real time: the unit's hand-off record, for a later process.
    #[inline]
    pub fn publish_saved(&self, h: &HandOff) {
        self.header().saved.publish_at(h, boottime_ns());
    }

    /// Not real time: the supervisor's record for this blob, and its age in
    /// seconds, if it has one.
    pub fn provided_for(&self, params_crc: u32) -> Option<(HandOff, f64)> {
        let h = self.header();
        let crc = h.provided_crc.load(Ordering::Acquire);
        if params_crc == 0 || crc != params_crc {
            return None;
        }
        h.provided.latest_at(boottime_ns())
    }

    /// The supervisor's side: offer `rec` (stamped `t_boottime_ns`) for
    /// blob `params_crc`.
    pub fn provide(&self, rec: &HandOff, t_boottime_ns: u64, params_crc: u32) {
        let h = self.header();
        h.provided_crc.store(0, Ordering::Release);
        h.provided.publish_at(rec, t_boottime_ns);
        h.provided_crc.store(params_crc, Ordering::Release);
    }

    /// The supervisor's side: whether the amps play.
    pub fn set_amps(&self, playing: bool) {
        let h = self.header();
        h.amps.store(if playing { AMPS_PLAYING } else { AMPS_HELD }, Ordering::Relaxed);
        h.amps_ns.store(mono_ns(), Ordering::Release);
    }

    /// Real time: the supervisor's word on the amps, if fresh.
    #[inline]
    pub fn amps(&self) -> crate::preroll::Amps {
        use crate::preroll::Amps;
        let h = self.header();
        let at = h.amps_ns.load(Ordering::Acquire);
        let word = h.amps.load(Ordering::Relaxed);
        if at == 0 || mono_ns().saturating_sub(at) > AMPS_STALE_NS {
            return Amps::Unknown;
        }
        match word {
            AMPS_HELD => Amps::Held,
            AMPS_PLAYING => Amps::Playing,
            _ => Amps::Unknown,
        }
    }

    /// The supervisor's side: offer the sense binding for blob `params_crc`
    /// (`None`: withdraw it).
    pub fn set_sense_bind(&self, params_crc: Option<u32>) {
        let h = self.header();
        h.sense_bind_crc.store(params_crc.unwrap_or(0), Ordering::Relaxed);
        h.sense_bind_ns.store(mono_ns(), Ordering::Release);
    }

    /// Real time: whether the supervisor binds the sense input of blob
    /// `params_crc` now: a fresh word naming exactly that blob.
    #[inline]
    pub fn sense_bind(&self, params_crc: u32) -> bool {
        let h = self.header();
        let at = h.sense_bind_ns.load(Ordering::Acquire);
        let crc = h.sense_bind_crc.load(Ordering::Relaxed);
        params_crc != 0 && crc == params_crc && at != 0 && mono_ns().saturating_sub(at) <= SENSE_BIND_STALE_NS
    }

    /// Real time: what the plugin did with the binding.
    #[inline]
    pub fn report_sense_bound(&self, state: u32) {
        let h = self.header();
        if h.sense_bound.load(Ordering::Relaxed) != state {
            h.sense_bound.store(state, Ordering::Release);
        }
    }

    /// Real time: what the wakes kept.
    #[inline]
    pub fn publish_wakes(&self, wakes: u32, last: u32, overflow: u64) {
        let h = self.header();
        h.wake_overflow.store(overflow, Ordering::Relaxed);
        if h.wakes.load(Ordering::Relaxed) != wakes {
            h.wake_frames.store(last, Ordering::Relaxed);
            h.wakes.store(wakes, Ordering::Release);
        }
    }

    /// Real time: the heartbeat, last.
    #[inline]
    pub fn beat(&self, frames: u64, health: Health, sense_delay: i32) {
        let h = self.header();
        h.frames.store(frames, Ordering::Relaxed);
        h.health.store(health.0, Ordering::Relaxed);
        h.sense_delay.store(sense_delay, Ordering::Relaxed);
        h.mono_ns.store(mono_ns(), Ordering::Relaxed);
        h.seq.fetch_add(1, Ordering::Release);
    }
}

/// The most restrictive gain of the `channels` active channels. A
/// non-finite gain on an active channel is a fault and is reported as NaN,
/// never dropped (absent channels beyond `channels` are NaN by design).
/// With no channel count (0) only finite gains count, as before.
pub fn min_gain(t: &Telemetry, channels: usize) -> f32 {
    if channels == 0 {
        return t.channels.iter().map(|c| c.gain).filter(|g| g.is_finite()).fold(1.0f32, f32::min);
    }
    let mut m = 1.0f32;
    for c in &t.channels[..channels.min(t.channels.len())] {
        if !c.gain.is_finite() {
            return f32::NAN;
        }
        m = m.min(c.gain);
    }
    m
}

impl Drop for EnginePage {
    fn drop(&mut self) {
        // SAFETY: unmapping our own mapping.
        unsafe { libc::munmap(self.base.cast(), self.len) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sgv2_core::ring::Reader;

    #[test]
    fn writer_and_reader_share_the_page() {
        let dir = std::env::temp_dir().join(format!("sgv2-page-{}", std::process::id()));
        let path = dir.to_str().unwrap().to_string();
        let w = EnginePage::create(&path).unwrap();
        let r = EnginePage::open(&path).unwrap();
        w.start(0xabcd, 96, 6, 12);
        let mut t = Telemetry::new();
        for f in 0..300u64 {
            t.frame = f * 512;
            t.channels[0].gain = 1.0 - f as f32 / 1000.0;
            w.push(&t);
            w.beat((f + 1) * 512, Health::OK, 1536);
        }
        let h = r.header();
        assert_eq!(h.magic.load(Ordering::Acquire), MAGIC);
        assert_eq!(h.params_crc.load(Ordering::Relaxed), 0xabcd);
        assert_eq!(h.seq.load(Ordering::Acquire), 300);
        assert_eq!(h.sense_delay.load(Ordering::Relaxed), 1536);
        let mut rd = Reader::new();
        let first = rd.poll(r.ring()).unwrap();
        assert!(first.frame >= (300 - RING as u64) * 512);
        assert!(rd.lost > 0);
        let _ = std::fs::remove_file(&path);
    }
}
