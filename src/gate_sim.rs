// SPDX-License-Identifier: MIT
//! Host execution of the pinned, dual-licensed Linux ledger and ADMAC payload
//! helper. The library has no devices or driver registration. Reference plant
//! certificates here never enable a production negotiation path.

use crate::{
    config::Config,
    playback::{Snapshot, BYTES},
};
use std::{
    ffi::{c_void, CStr, CString},
    io,
    marker::PhantomData,
    ptr::NonNull,
    rc::Rc,
    sync::OnceLock,
};

struct Api {
    new: unsafe extern "C" fn(
        u32,
        u32,
        u32,
        u32,
        u32,
        u32,
        u32,
        u32,
        *const libc::c_char,
    ) -> *mut c_void,
    free: unsafe extern "C" fn(*mut c_void),
    owner: unsafe extern "C" fn(*mut c_void, libc::c_int, u64) -> libc::c_int,
    snapshot: unsafe extern "C" fn(*mut c_void, u64, *mut c_void) -> libc::c_int,
    write: unsafe extern "C" fn(*mut c_void, *const c_void, u64) -> libc::c_int,
    ping: unsafe extern "C" fn(*mut c_void, u64) -> libc::c_int,
    unlocked: unsafe extern "C" fn(*mut c_void, u64) -> libc::c_int,
    submit: unsafe extern "C" fn(*mut c_void, *const i32, *mut i32, u64, *mut u64) -> libc::c_int,
    retire: unsafe extern "C" fn(*mut c_void, u64, u64) -> libc::c_int,
    stop: unsafe extern "C" fn(*mut c_void, u64) -> libc::c_int,
    start: unsafe extern "C" fn(*mut c_void, u64) -> libc::c_int,
}

fn api() -> &'static Api {
    static API: OnceLock<Api> = OnceLock::new();
    API.get_or_init(|| {
        // Build under the same admitted test job, once per process. Normal
        // production builds do not require Python, a C compiler or this library.
        let source = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/kernel");
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let out =
            std::env::temp_dir().join(format!("speakerguardd-gate-{}-{nonce}", std::process::id()));
        std::fs::create_dir(&out).unwrap();
        let built = std::process::Command::new("python3")
            .arg(source.join("build-host.py"))
            .arg("--vendor")
            .arg(&source)
            .arg("--out")
            .arg(&out)
            .status()
            .unwrap();
        assert!(
            built.success(),
            "the pinned kernel model must build for every test run"
        );
        let path = CString::new(out.join("libapple-speaker-energy.so").to_str().unwrap()).unwrap();
        // SAFETY: only a cfg(test), build-owned library is loaded. Its symbols
        // have these fixed C signatures. Retain its handle for the test process.
        unsafe {
            let handle = libc::dlopen(path.as_ptr(), libc::RTLD_NOW | libc::RTLD_LOCAL);
            assert!(
                !handle.is_null(),
                "kernel test library: {}",
                CStr::from_ptr(libc::dlerror()).to_string_lossy()
            );
            macro_rules! symbol {
                ($name:literal,$ty:ty) => {{
                    let p = libc::dlsym(handle, concat!($name, "\0").as_ptr().cast());
                    assert!(!p.is_null(), "missing kernel model symbol {}", $name);
                    std::mem::transmute::<*mut c_void, $ty>(p)
                }};
            }
            std::fs::remove_dir_all(&out).unwrap();
            Api {
                new: symbol!(
                    "ag_new",
                    unsafe extern "C" fn(
                        u32,
                        u32,
                        u32,
                        u32,
                        u32,
                        u32,
                        u32,
                        u32,
                        *const libc::c_char,
                    ) -> *mut c_void
                ),
                free: symbol!("ag_free", unsafe extern "C" fn(*mut c_void)),
                owner: symbol!(
                    "ag_owner",
                    unsafe extern "C" fn(*mut c_void, libc::c_int, u64) -> libc::c_int
                ),
                snapshot: symbol!(
                    "ag_snapshot",
                    unsafe extern "C" fn(*mut c_void, u64, *mut c_void) -> libc::c_int
                ),
                write: symbol!(
                    "ag_write",
                    unsafe extern "C" fn(*mut c_void, *const c_void, u64) -> libc::c_int
                ),
                ping: symbol!(
                    "ag_ping",
                    unsafe extern "C" fn(*mut c_void, u64) -> libc::c_int
                ),
                unlocked: symbol!(
                    "ag_unlocked",
                    unsafe extern "C" fn(*mut c_void, u64) -> libc::c_int
                ),
                submit: symbol!(
                    "ag_submit",
                    unsafe extern "C" fn(
                        *mut c_void,
                        *const i32,
                        *mut i32,
                        u64,
                        *mut u64,
                    ) -> libc::c_int
                ),
                retire: symbol!(
                    "ag_retire",
                    unsafe extern "C" fn(*mut c_void, u64, u64) -> libc::c_int
                ),
                stop: symbol!(
                    "ag_stop",
                    unsafe extern "C" fn(*mut c_void, u64) -> libc::c_int
                ),
                start: symbol!(
                    "ag_start",
                    unsafe extern "C" fn(*mut c_void, u64) -> libc::c_int
                ),
            }
        }
    })
}

fn checked(code: libc::c_int) -> io::Result<()> {
    if code == 0 {
        Ok(())
    } else {
        Err(io::Error::from_raw_os_error(-code))
    }
}

pub struct KernelGate {
    pointer: NonNull<c_void>,
    pub frames: usize,
    pub channels: usize,
    pub format: u32,
    model_of_slot: Vec<usize>,
    _local: PhantomData<Rc<()>>,
}

impl KernelGate {
    pub fn new(cfg: &Config, rate: u32, frames: usize) -> Self {
        Self::with_tail(cfg, rate, frames, 0)
    }
    pub fn with_tail(cfg: &Config, rate: u32, frames: usize, tail_us: u32) -> Self {
        let card = CString::new(
            cfg.playback_cards
                .first()
                .expect("fixture card identity")
                .as_str(),
        )
        .unwrap();
        let transfer = if cfg.has_vi() { 2 } else { 1 };
        let safe = if cfg.has_vi() {
            100000
        } else {
            (cfg.globals.safe_reduction * 1000.0).round() as u32
        };
        let format = if cfg.playback_cards.iter().any(|s| s == "AppleJ616") {
            6
        } else {
            10
        };
        // SAFETY: bounded numeric dimensions and a valid NUL-terminated card id.
        let p = unsafe {
            (api().new)(
                rate,
                cfg.speakers.len() as u32,
                frames as u32,
                format,
                transfer,
                (cfg.globals.amp_gain_dbv * 1000.0).round() as u32,
                safe,
                tail_us,
                card.as_ptr(),
            )
        };
        Self {
            pointer: NonNull::new(p).expect("construct kernel test gate"),
            frames,
            channels: cfg.speakers.len(),
            format,
            model_of_slot: (0..cfg.speakers.len())
                .map(|slot| {
                    cfg.playback_slots
                        .iter()
                        .position(|s| *s == slot)
                        .expect("complete physical map")
                })
                .collect(),
            _local: PhantomData,
        }
    }
    pub fn owner(&mut self, owner: bool, now: u64) -> io::Result<()> {
        // SAFETY: this object owns its opaque C allocation for every call.
        checked(unsafe { (api().owner)(self.pointer.as_ptr(), owner.into(), now) })
    }
    pub fn status(&mut self, now: u64) -> io::Result<Snapshot> {
        let mut wire = [0u8; BYTES];
        // SAFETY: output is the full 512-byte syscall layout.
        checked(unsafe { (api().snapshot)(self.pointer.as_ptr(), now, wire.as_mut_ptr().cast()) })?;
        Snapshot::decode(wire)
    }
    pub fn write(&mut self, wire: &[u8; BYTES], now: u64) -> io::Result<()> {
        // SAFETY: the initialized input is exactly the C syscall layout.
        checked(unsafe { (api().write)(self.pointer.as_ptr(), wire.as_ptr().cast(), now) })
    }
    pub fn ping(&mut self, now: u64) -> io::Result<()> {
        // SAFETY: live opaque object.
        checked(unsafe { (api().ping)(self.pointer.as_ptr(), now) })
    }
    pub fn unlocked(&self, now: u64) -> bool {
        // SAFETY: C keeps its mutable implementation behind its own locks.
        unsafe { (api().unlocked)(self.pointer.as_ptr(), now) != 0 }
    }
    pub fn submit(&mut self, input: &[i32], now: u64) -> io::Result<(u64, Vec<i32>)> {
        assert_eq!(input.len(), self.frames * self.channels);
        let mut output = vec![0i32; input.len()];
        let mut ticket = 0;
        // SAFETY: both arrays have the exact dimensions fixed at construction.
        checked(unsafe {
            (api().submit)(
                self.pointer.as_ptr(),
                input.as_ptr(),
                output.as_mut_ptr(),
                now,
                &mut ticket,
            )
        })?;
        Ok((ticket, output))
    }
    pub fn retire(&mut self, ticket: u64, now: u64) -> io::Result<()> {
        // SAFETY: live object; C checks single-use FIFO ticket ownership.
        checked(unsafe { (api().retire)(self.pointer.as_ptr(), ticket, now) })
    }
    pub fn stop(&mut self, now: u64) -> io::Result<()> {
        // SAFETY: no hardware exists; this proves the host queue discarded.
        checked(unsafe { (api().stop)(self.pointer.as_ptr(), now) })
    }
    pub fn start(&mut self, now: u64) -> io::Result<()> {
        // SAFETY: C quiesces before assigning the new mapping epoch.
        checked(unsafe { (api().start)(self.pointer.as_ptr(), now) })
    }
    pub fn normalized_input(&self, x: i32) -> i32 {
        match self.format {
            2 => (x / 65536) * 65536,
            6 => (x / 256) * 256,
            _ => x,
        }
    }
}

impl Drop for KernelGate {
    fn drop(&mut self) {
        // SAFETY: no cloned owner or asynchronous host callback exists.
        unsafe {
            (api().free)(self.pointer.as_ptr());
        }
    }
}

#[test]
fn actual_kernel_payload_and_grant_abi_round_trip() {
    let cfg = Config::parse(include_str!("../conf/apple/j413.conf")).unwrap();
    let mut gate = KernelGate::new(&cfg, 48000, 32);
    gate.owner(true, 1_000_000_000).unwrap();
    gate.start(1_000_000_000).unwrap();
    let input = vec![21474836; 32 * cfg.speakers.len()];
    let (ticket, output) = gate.submit(&input, 1_000_000_000).unwrap();
    assert_eq!(input, output);
    assert!(!gate.unlocked(1_000_000_000));
    let s = gate.status(1_000_000_000).unwrap();
    assert!(s.qualified(&cfg));
    assert!(s.unbudgeted[0] > 0);
    assert_eq!(s.submitted[0], 0);
    let cap: Vec<u64> = (0..s.speakers).map(|k| s.unbudgeted[k] * 2).collect();
    let packet = s
        .grant(&cap, &vec![crate::playback::UNITY; s.speakers])
        .unwrap();
    gate.write(&packet, 1_000_000_000).unwrap();
    assert!(gate.unlocked(1_000_000_000));
    gate.retire(ticket, 1_001_000_000).unwrap();
    let s = gate.status(1_001_000_000).unwrap();
    assert!(s.completed[0] > 0);
    assert_eq!(s.refunded[0], 0);
    assert_eq!(
        gate.write(&packet, 1_001_000_000)
            .unwrap_err()
            .raw_os_error(),
        Some(libc::ESTALE)
    );
    assert!(!gate.unlocked(1_001_000_000));
}

#[test]
fn actual_kernel_expiry_and_owner_death_keep_the_safe_level() {
    let cfg = Config::parse(include_str!("../conf/apple/j700.conf")).unwrap();
    let mut gate = KernelGate::new(&cfg, 48000, 32);
    gate.owner(true, 1).unwrap();
    let s = gate.status(1).unwrap();
    let cap = vec![1 << 40; s.speakers];
    gate.write(
        &s.grant(&cap, &vec![crate::playback::UNITY; s.speakers])
            .unwrap(),
        1,
    )
    .unwrap();
    assert!(gate.unlocked(2));
    assert!(!gate.unlocked(250_000_002));
    gate.owner(false, 250_000_002).unwrap();
    let s = gate.status(250_000_002).unwrap();
    assert_eq!(
        gate.write(
            &s.grant(&cap, &vec![crate::playback::UNITY; s.speakers])
                .unwrap(),
            250_000_002
        )
        .unwrap_err()
        .raw_os_error(),
        Some(libc::EPERM)
    );
}

/// Immutable, five-descriptor playback queue. Programme access is restricted
/// to this hardware-side renderer; the daemon sees only the ABI receipt.
/// Sense/oracle reads use the saved output and never spend the grant again.
pub struct DescriptorQueue {
    pub gate: KernelGate,
    rate: u32,
    boot_ns: u64,
    next: u64,
    playing: bool,
    pub debug_case: u64,
    rows: std::collections::VecDeque<Rendered>,
    active: std::collections::VecDeque<(u64, u64)>,
}
struct Rendered {
    start: u64,
    end: u64,
    words: Vec<i32>,
    original: Vec<i32>,
}
impl DescriptorQueue {
    pub fn new(cfg: &Config, rate: u32, frames: usize, boot_ns: u64) -> Self {
        let mut gate = KernelGate::new(cfg, rate, frames);
        gate.start(boot_ns).unwrap();
        Self {
            gate,
            rate,
            boot_ns,
            next: 0,
            playing: true,
            debug_case: 0,
            rows: Default::default(),
            active: Default::default(),
        }
    }
    pub fn ns(&self, seconds: f64) -> u64 {
        self.boot_ns + (seconds * 1e9).round() as u64
    }
    fn first_frame(seconds: f64, rate: u32) -> u64 {
        let rate = f64::from(rate);
        let mut n = (seconds * rate).ceil() as u64;
        // Match the plant's sample-time comparison exactly; ceil of a
        // rounded (n/rate)*rate can otherwise invent a one-frame hole.
        while n > 0 && (n - 1) as f64 / rate >= seconds {
            n -= 1;
        }
        while (n as f64 / rate) < seconds {
            n += 1;
        }
        n
    }
    fn frame_at(seconds: f64, rate: u32) -> u64 {
        let rate = f64::from(rate);
        let mut n = (seconds * rate).floor() as u64;
        while (n + 1) as f64 / rate <= seconds {
            n += 1;
        }
        while n > 0 && n as f64 / rate > seconds {
            n -= 1;
        }
        n
    }
    /// A PCM close proves discard; a new prepare starts a distinct epoch.
    /// Retired discarded copies remain conservatively billed as possibly played.
    pub fn set_playing(&mut self, playing: bool, seconds: f64) -> bool {
        if self.playing == playing {
            return false;
        }
        let now = self.ns(seconds);
        if playing {
            self.gate.start(now).unwrap();
            self.next = Self::first_frame(seconds, self.rate);
        } else {
            self.gate.stop(now).unwrap();
            let end = Self::first_frame(seconds, self.rate);
            for r in &mut self.rows {
                r.end = r.end.min(end);
            }
            self.active.clear();
        }
        self.playing = playing;
        true
    }
    /// Render at descriptor-publication boundaries, including during a
    /// userspace stall. Retire each descriptor exactly once in FIFO order.
    pub fn advance(&mut self, seconds: f64, mut programme: impl FnMut(u64, usize) -> f64) {
        if !self.playing {
            return;
        }
        let target = Self::frame_at(seconds, self.rate);
        let mut at = self
            .active
            .front()
            .map_or(self.next, |r| r.0.saturating_sub(self.gate.frames as u64));
        loop {
            if at > target {
                break;
            }
            let ns = self.boot_ns + at * 1_000_000_000 / u64::from(self.rate);
            while self.active.front().is_some_and(|r| r.0 <= at) {
                let (_, ticket) = self.active.pop_front().unwrap();
                self.gate.retire(ticket, ns).unwrap();
            }
            while self.active.len() < 5 {
                let mut input = Vec::with_capacity(self.gate.frames * self.gate.channels);
                for n in self.next..self.next + self.gate.frames as u64 {
                    for k in 0..self.gate.channels {
                        let x = programme(n, self.gate.model_of_slot[k]);
                        assert!(
                            x.is_finite() && x.abs() <= 1.0 + 1e-12,
                            "invalid queued PCM {x}"
                        );
                        input.push((x.clamp(-1.0, 1.0) * 2147483647.0) as i32);
                    }
                }
                let (ticket, output) = self.gate.submit(&input, ns).unwrap();
                self.rows.push_back(Rendered {
                    start: self.next,
                    end: self.next + self.gate.frames as u64,
                    words: output,
                    original: input
                        .into_iter()
                        .map(|x| self.gate.normalized_input(x))
                        .collect(),
                });
                self.next += self.gate.frames as u64;
                self.active.push_back((self.next, ticket));
            }
            let end = self.active.front().unwrap().0;
            if end > target {
                break;
            }
            at = end;
        }
    }
    /// Digital gain actually applied to a sample, prior to the separate
    /// amplifier volume. Quantization is included in the ratio.
    pub fn ratio(&self, n: u64, k: usize) -> f64 {
        let r = self
            .rows
            .iter()
            .rev()
            .find(|r| r.start <= n && n < r.end)
            .unwrap_or_else(||panic!("oracle cannot read unrendered playback: case={} frame={n} model={k} next={} playing={} rows={} first={:?} last={:?}",
                self.debug_case,self.next,self.playing,self.rows.len(),self.rows.front().map(|r|r.start),self.rows.back().map(|r|r.start)));
        let physical = self
            .gate
            .model_of_slot
            .iter()
            .position(|m| *m == k)
            .unwrap();
        let i = (n - r.start) as usize * self.gate.channels + physical;
        if r.original[i] == 0 {
            assert_eq!(r.words[i], 0);
            1.0 // no additional attenuation of an already-zero PCM word
        } else {
            f64::from(r.words[i]) / f64::from(r.original[i])
        }
    }
    pub fn sample(&self, n: u64, k: usize) -> i32 {
        let r = self
            .rows
            .iter()
            .rev()
            .find(|r| r.start <= n && n < r.end)
            .expect("sense cannot read unrendered playback");
        let physical = self
            .gate
            .model_of_slot
            .iter()
            .position(|m| *m == k)
            .unwrap();
        r.words[(n - r.start) as usize * self.gate.channels + physical]
    }
    pub fn emitted_frames(&self, seconds: f64) -> u64 {
        Self::frame_at(seconds, self.rate)
    }

    /// RMS-equivalent attenuation of the actual native queued words. This
    /// compares the same PCM format on both sides, not integer conversion
    /// against an unrepresentable float. Every requested frame must exist.
    pub fn rms_reduction(&self, from: u64, to: u64) -> Vec<f64> {
        assert!(to > from);
        let channels = self.gate.channels;
        let mut input = vec![0.0; channels];
        let mut output = vec![0.0; channels];
        let mut covered = 0;
        let mut identical = true;
        for r in &self.rows {
            let start = from.max(r.start);
            let end = to.min(r.end);
            if end <= start {
                continue;
            }
            covered += end - start;
            let lo = (start - r.start) as usize * channels;
            let hi = (end - r.start) as usize * channels;
            identical &= r.original[lo..hi] == r.words[lo..hi];
            for (a, b) in r.original[lo..hi]
                .chunks_exact(channels)
                .zip(r.words[lo..hi].chunks_exact(channels))
            {
                for k in 0..channels {
                    input[k] += f64::from(a[k]).powi(2);
                    output[k] += f64::from(b[k]).powi(2);
                }
            }
        }
        assert_eq!(
            covered,
            to - from,
            "listening metric must cover actual queued frames exactly"
        );
        if identical {
            return vec![0.0; channels];
        }
        (0..channels)
            .map(|model| {
                let k = self
                    .gate
                    .model_of_slot
                    .iter()
                    .position(|m| *m == model)
                    .unwrap();
                if input[k] == 0.0 {
                    assert_eq!(output[k], 0.0);
                    0.0
                } else if output[k] == 0.0 {
                    f64::INFINITY
                } else {
                    10.0 * (input[k] / output[k]).log10()
                }
            })
            .collect()
    }

    pub fn prune_before(&mut self, n: u64) {
        while self
            .rows
            .front()
            .is_some_and(|r| r.start + (self.gate.frames as u64) < n)
        {
            self.rows.pop_front();
        }
    }
}

#[test]
fn descriptor_queue_renders_future_pcm_once_and_keeps_stall_debt() {
    let cfg = Config::parse(include_str!("../conf/apple/j413.conf")).unwrap();
    let mut q = DescriptorQueue::new(&cfg, 48000, 64, 1_000_000_000);
    q.gate.owner(true, q.ns(0.0)).unwrap();
    let status = q.gate.status(q.ns(0.0)).unwrap();
    q.gate
        .write(
            &status
                .grant(
                    &vec![1 << 40; status.speakers],
                    &vec![crate::playback::UNITY; status.speakers],
                )
                .unwrap(),
            q.ns(0.0),
        )
        .unwrap();
    let calls = std::cell::Cell::new(0);
    q.advance(0.0, |_, _| {
        calls.set(calls.get() + 1);
        0.1
    });
    assert_eq!(calls.get(), 5 * 64 * cfg.speakers.len());
    let s = q.gate.status(q.ns(0.0)).unwrap();
    assert_eq!(s.pending, 5);
    assert!(s.submitted[0] > s.completed[0]);
    let before = calls.get();
    let ratio = q.ratio(16, 0);
    assert_eq!(ratio, q.ratio(16, 0));
    assert_eq!(before, calls.get());
    // The kernel continues filling during an absent daemon. It enforces
    // the expired grant itself, without a userspace callback.
    q.advance(0.30, |_, _| 0.1);
    assert!(!q.gate.unlocked(q.ns(0.30)));
    // After expiry raw caching may resume only under the hardware safe cap.
    assert!(q.ratio(14200, 0) <= 1.0);
    let s = q.gate.status(q.ns(0.30)).unwrap();
    assert!(s.completed[0] > 0);
    assert_eq!(s.pending, 5);
    assert!(s.submitted[0] >= s.completed[0]);
}

#[test]
fn an_unaffordable_safe_prefix_drains_before_any_unlock() {
    let cfg = Config::parse(include_str!("../conf/apple/j413.conf")).unwrap();
    let mut gate = KernelGate::new(&cfg, 48000, 64);
    gate.owner(true, 1).unwrap();
    let full = vec![i32::MAX; gate.frames * gate.channels];
    let (raw_ticket, raw) = gate.submit(&full, 1).unwrap();
    assert_eq!(raw, full);
    let s = gate.status(1).unwrap();
    let low = vec![1; s.speakers];
    assert_eq!(
        gate.write(
            &s.grant(&low, &vec![crate::playback::UNITY; s.speakers])
                .unwrap(),
            1
        )
        .unwrap_err()
        .raw_os_error(),
        Some(libc::EAGAIN)
    );
    assert!(!gate.unlocked(1));
    let (protected_ticket, protected) = gate.submit(&full, 2).unwrap();
    assert!(protected.iter().all(|x| x.abs() <= 21474));
    gate.retire(raw_ticket, 3).unwrap();
    let s = gate.status(3).unwrap();
    assert_eq!(s.unbudgeted[0], 0);
    gate.write(
        &s.grant(
            &vec![1 << 35; s.speakers],
            &vec![crate::playback::UNITY; s.speakers],
        )
        .unwrap(),
        3,
    )
    .unwrap();
    assert!(gate.unlocked(3));
    gate.retire(protected_ticket, 4).unwrap();
}

#[test]
fn a_restart_never_forgives_possibly_played_payloads() {
    let cfg = Config::parse(include_str!("../conf/apple/j616.conf")).unwrap();
    let mut gate = KernelGate::new(&cfg, 48000, 64);
    gate.owner(true, 1).unwrap();
    let s = gate.status(1).unwrap();
    gate.write(
        &s.grant(
            &vec![1 << 40; s.speakers],
            &vec![crate::playback::UNITY; s.speakers],
        )
        .unwrap(),
        1,
    )
    .unwrap();
    gate.submit(&vec![i32::MAX; 64 * gate.channels], 2).unwrap();
    let before = gate.status(2).unwrap();
    gate.owner(false, 3).unwrap();
    gate.stop(4).unwrap();
    gate.start(5).unwrap();
    gate.owner(true, 6).unwrap();
    let after = gate.status(6).unwrap();
    assert!(after.epoch > before.epoch);
    assert_eq!(after.submitted, before.submitted);
    assert_eq!(after.completed, before.submitted);
    assert_eq!(after.refunded, before.refunded);
    assert!(!gate.unlocked(6));
}

#[test]
fn pcm_close_and_restart_revoke_and_replace_overlapping_future_copies() {
    let cfg = Config::parse(include_str!("../conf/apple/j413.conf")).unwrap();
    let mut q = DescriptorQueue::new(&cfg, 48000, 4096, 1_000_000_000);
    q.gate.owner(true, q.ns(0.0)).unwrap();
    let s = q.gate.status(q.ns(0.0)).unwrap();
    let old = s
        .grant(
            &vec![1 << 44; s.speakers],
            &vec![crate::playback::UNITY; s.speakers],
        )
        .unwrap();
    q.gate.write(&old, q.ns(0.0)).unwrap();
    q.advance(0.0, |_, _| 0.8);
    let before = q.gate.status(q.ns(0.01)).unwrap();
    q.set_playing(false, 0.01);
    let stopped = q.gate.status(q.ns(0.01)).unwrap();
    assert_eq!(stopped.completed, before.submitted);
    assert_eq!(stopped.pending, 0);
    assert!(!q.gate.unlocked(q.ns(0.01)));
    q.set_playing(true, 0.02);
    q.advance(0.02, |_, _| 0.2);
    let after = q.gate.status(q.ns(0.02)).unwrap();
    assert!(after.epoch > before.epoch);
    assert_eq!(after.submitted, before.submitted);
    assert!(q.sample(1000, 0) < (2147483647.0 * 0.201) as i32);
    assert!(q.sample(1000, 0) > (2147483647.0 * 0.199) as i32);
    assert_eq!(
        q.gate.write(&old, q.ns(0.02)).unwrap_err().raw_os_error(),
        Some(libc::ESTALE)
    );
    assert!(!q.gate.unlocked(q.ns(0.02)));
}

#[test]
fn pcm_epochs_start_on_the_same_grid_as_the_sample_oracle() {
    for rate in [44100, 48000, 96000, 192000] {
        for n in (0..rate as u64).step_by(37) {
            let t = n as f64 / f64::from(rate);
            assert_eq!(DescriptorQueue::first_frame(t, rate), n);
            assert_eq!(DescriptorQueue::frame_at(t, rate), n);
            let after = t + 0.25 / f64::from(rate);
            assert_eq!(DescriptorQueue::first_frame(after, rate), n + 1);
            assert_eq!(DescriptorQueue::frame_at(after, rate), n);
        }
    }
}

#[test]
fn a_unity_j616_grant_adds_no_attenuation_to_its_native_pcm_words() {
    let cfg = Config::parse(include_str!("../conf/apple/j616.conf")).unwrap();
    let mut q = DescriptorQueue::new(&cfg, 48000, 64, 1_000_000_000);
    q.gate.owner(true, q.ns(0.0)).unwrap();
    let s = q.gate.status(q.ns(0.0)).unwrap();
    q.gate
        .write(
            &s.grant(
                &vec![1 << 40; s.speakers],
                &vec![crate::playback::UNITY; s.speakers],
            )
            .unwrap(),
            q.ns(0.0),
        )
        .unwrap();
    q.advance(0.0, |n, k| 0.031 + (n + 4 * k as u64) as f64 * 0.00000137);
    for n in 0..64 {
        for k in 0..cfg.speakers.len() {
            assert_eq!(q.ratio(n, k), 1.0);
        }
    }
}

#[test]
fn listening_meter_includes_the_kernel_clamp_and_safe_floor() {
    let cfg = Config::parse(include_str!("../conf/apple/j700.conf")).unwrap();
    let mut q = DescriptorQueue::new(&cfg, 48000, 32, 1_000_000_000);
    q.gate.owner(true, q.ns(0.0)).unwrap();
    let s = q.gate.status(q.ns(0.0)).unwrap();
    q.gate
        .write(
            &s.grant(
                &vec![16 * (1 << (crate::playback::UNIT_SHIFT - 2)); s.speakers],
                &vec![crate::playback::UNITY; s.speakers],
            )
            .unwrap(),
            q.ns(0.0),
        )
        .unwrap();
    q.advance(0.0, |_, _| 0.5);
    let expected = 10.0 * (2.0f64 / 1.01).log10();
    for db in q.rms_reduction(0, 32) {
        assert!((db - expected).abs() < 1e-6, "{db} vs {expected}");
    }
    assert!(q.rms_reduction(16, 32).iter().all(|db| *db > 19.999));
}
