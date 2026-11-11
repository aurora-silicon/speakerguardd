//! The kernel's speaker grant, followed by the plugin, and the ramp it puts
//! on the program around it.
//!
//! The kernel withdraws protection by ramping the amps down and stopping the
//! speaker stream; it grants it again (the supervisor came back, a lease
//! lapse ended, a resume) with the amps at their fixed operating gain, and
//! the audio server restarts the stream. The program has gone on meanwhile:
//! without a ramp, the first frames the server writes after its silence
//! prefill start in the middle of a waveform, at the operating gain, and
//! click. The amps cannot ramp in (their gain is fixed while granted: the
//! supervisor checks it), and the kernel never sees those frames (the ring
//! is mapped). This plugin is the last thing to touch the program, so the
//! ramp in is here, the mirror of the kernel's ramp down:
//!
//! - while the kernel does not grant, the program into the unit is muted
//!   (after a short ramp out, if a call was running);
//! - after a grant, it stays muted for [`HOLD_MS`], long enough for the
//!   audio server to restart the stream (it retries within a period), and
//!   then ramps in over [`RAMP_IN_MS`]. Whatever the server buffered before
//!   the restart is silence, and the program starts from a smooth ramp.
//! - unless the program has been silent (digital zero, at least
//!   [`crate::preroll::SILENT_RUN`] frames up to the grant): then nothing was
//!   cut and nothing is mid-waveform, and the program passes from the grant
//!   on. That is the wake from the idle power-down, where the first frames
//!   of the stream that woke the speakers are held ahead of this ramp
//!   ([`crate::preroll`]) and must not be muted.
//!
//! A watcher thread (not real time) follows the card's "Speaker Protection"
//! control through its change events (the kernel notifies every grant and
//! every withdrawal, not the renewals) and publishes it in atomics; the
//! real-time thread only loads them. Without the control (another card, a
//! test host, a kernel without it, the watcher failing), nothing is muted:
//! the program passes as before. The ramp only ever attenuates, so it can
//! never drive the speakers harder than the program would.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;

/// Silence held after a grant before the program ramps in, ms.
pub const HOLD_MS: f64 = 50.0;
/// The ramp in after the hold, ms.
pub const RAMP_IN_MS: f64 = 10.0;
/// The ramp out when protection goes while a call runs, ms.
pub const RAMP_OUT_MS: f64 = 2.0;

/// The grant as the watcher last saw it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Grant {
    /// The watcher follows the control; false: no gate.
    pub known: bool,
    pub granted: bool,
    /// Bumped on every change the kernel notified, so that a withdrawal and
    /// a grant between two calls are not lost.
    pub epoch: u32,
}

#[derive(Default)]
pub struct Shared {
    known: AtomicBool,
    granted: AtomicBool,
    epoch: AtomicU32,
    stop: AtomicBool,
}

impl Shared {
    /// Real time.
    #[inline]
    pub fn load(&self) -> Grant {
        let known = self.known.load(Ordering::Acquire);
        Grant { known, granted: self.granted.load(Ordering::Relaxed), epoch: self.epoch.load(Ordering::Relaxed) }
    }

    /// The control read `granted`; `changed`: after a change event.
    pub fn set(&self, granted: bool, changed: bool) {
        self.granted.store(granted, Ordering::Relaxed);
        if changed {
            self.epoch.fetch_add(1, Ordering::Relaxed);
        }
        self.known.store(true, Ordering::Release);
    }

    /// The control is gone (card removed, watcher stopped): no gate.
    pub fn lost(&self) {
        self.known.store(false, Ordering::Release);
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    /// Gain 1: the program passes untouched.
    Open,
    /// Ramping out to the mute.
    Out,
    Muted,
    /// Muted for this many more frames after a grant.
    Hold(u32),
    /// Ramping in to `Open`.
    In,
}

/// The real-time side: the gain on the program, frame by frame.
pub struct Ramp {
    phase: Phase,
    /// Linear position of the ramp, 0..=1 (shaped by [`smooth`]).
    x: f32,
    step_in: f32,
    step_out: f32,
    hold: u32,
    epoch: Option<u32>,
    /// Digital-zero frames in a row at the ramp's input, up to the last
    /// call (saturating).
    zero_run: u32,
}

/// The ramp's shape: smoothstep, flat at both ends.
#[inline]
fn smooth(x: f32) -> f32 {
    x * x * (3.0 - 2.0 * x)
}

impl Ramp {
    pub fn new(rate: f64) -> Self {
        let frames = |ms: f64| ((rate * ms / 1000.0).round() as u32).max(1);
        Ramp {
            phase: Phase::Open,
            x: 1.0,
            step_in: 1.0 / frames(RAMP_IN_MS) as f32,
            step_out: 1.0 / frames(RAMP_OUT_MS) as f32,
            hold: frames(HOLD_MS),
            epoch: None,
            zero_run: 0,
        }
    }

    /// A stream starts (LV2 activate: the histories are cleared): ramp the
    /// program in, unless the grant keeps it muted anyway.
    pub fn restart(&mut self) {
        if matches!(self.phase, Phase::Open | Phase::In | Phase::Out) {
            self.x = 0.0;
            self.phase = Phase::In;
        }
    }

    /// Whether the program passes untouched.
    pub fn open(&self) -> bool {
        self.phase == Phase::Open
    }

    /// At the start of a call: follow the grant.
    pub fn follow(&mut self, g: Grant) {
        if !g.known {
            // no gate: back to the program, smoothly if it was muted
            self.epoch = None;
            if !matches!(self.phase, Phase::Open | Phase::In) {
                self.phase = Phase::In;
            }
            return;
        }
        let changed = self.epoch.is_some_and(|e| e != g.epoch);
        let first = self.epoch.is_none();
        self.epoch = Some(g.epoch);
        if !g.granted {
            self.phase = match self.phase {
                Phase::Muted | Phase::Hold(_) => Phase::Muted,
                // a running call: ramp out (the kernel has ramped the amps
                // down already; this keeps the next restart's buffer silent)
                _ if first => {
                    self.x = 0.0;
                    Phase::Muted
                }
                _ => Phase::Out,
            };
            return;
        }
        // granted
        let regrant = changed || matches!(self.phase, Phase::Muted | Phase::Out);
        if regrant && self.zero_run >= crate::preroll::SILENT_RUN && self.phase != Phase::Out {
            // granted while the program is silent (a wake): nothing was cut,
            // so nothing restarts mid-waveform; the program passes at once
            self.x = 1.0;
            self.phase = Phase::Open;
        } else if changed {
            // a grant since the last call (perhaps after a withdrawal this
            // plugin never ran through): the stream restarts from silence
            self.x = 0.0;
            self.phase = Phase::Hold(self.hold);
        } else if matches!(self.phase, Phase::Muted | Phase::Out) {
            // granted with no change seen: the watcher's first reading
            self.x = 0.0;
            self.phase = Phase::Hold(self.hold);
        }
    }

    /// Follow the silence at the ramp's input: trailing digital-zero frames.
    fn note_silence(&mut self, bufs: &[&mut [f32]], frames: usize) {
        let zero = |i: usize| bufs.iter().all(|b| b[i] == 0.0);
        let trailing = (0..frames).rev().take_while(|&i| zero(i)).count();
        self.zero_run = if trailing == frames { self.zero_run.saturating_add(frames as u32) } else { trailing as u32 };
    }

    /// The gain for the next frame.
    #[inline]
    fn next(&mut self) -> f32 {
        match self.phase {
            Phase::Open => 1.0,
            Phase::Muted => 0.0,
            Phase::Hold(n) => {
                self.phase = if n <= 1 { Phase::In } else { Phase::Hold(n - 1) };
                0.0
            }
            Phase::In => {
                self.x += self.step_in;
                if self.x >= 1.0 {
                    self.x = 1.0;
                    self.phase = Phase::Open;
                }
                smooth(self.x)
            }
            Phase::Out => {
                self.x -= self.step_out;
                if self.x <= 0.0 {
                    self.x = 0.0;
                    self.phase = Phase::Muted;
                }
                smooth(self.x)
            }
        }
    }

    /// Real time: apply the gain to `frames` frames of every channel.
    pub fn apply(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        self.note_silence(bufs, frames);
        if self.phase == Phase::Open {
            return;
        }
        if self.phase == Phase::Muted {
            for b in bufs.iter_mut() {
                b[..frames].fill(0.0);
            }
            return;
        }
        for i in 0..frames {
            let g = self.next();
            for b in bufs.iter_mut() {
                b[i] *= g;
            }
        }
    }
}

/// The watcher: follows the control until dropped.
pub struct Gate {
    shared: Arc<Shared>,
    watcher: Option<JoinHandle<()>>,
    /// Wakes the watcher's poll on drop (-1: none).
    wake: i32,
}

impl Gate {
    /// A gate nobody moves: the program passes (tests, other hosts).
    pub fn none() -> Self {
        Gate { shared: Arc::new(Shared::default()), watcher: None, wake: -1 }
    }

    /// Follow the speaker card's grant. `SGV2_GRANT_GATE=0` turns it off.
    pub fn watch() -> Self {
        if std::env::var("SGV2_GRANT_GATE").is_ok_and(|v| v == "0") {
            return Self::none();
        }
        #[cfg(target_os = "linux")]
        {
            // SAFETY: eventfd has no memory preconditions.
            let wake = unsafe { libc::eventfd(0, libc::EFD_CLOEXEC | libc::EFD_NONBLOCK) };
            if wake < 0 {
                return Self::none();
            }
            let shared = Arc::new(Shared::default());
            let s = shared.clone();
            let watcher =
                std::thread::Builder::new().name("sgv2-grant-watch".into()).spawn(move || ctl::watch(&s, wake)).ok();
            Gate { shared, watcher, wake }
        }
        #[cfg(not(target_os = "linux"))]
        Self::none()
    }

    pub fn shared(&self) -> &Arc<Shared> {
        &self.shared
    }
}

impl Drop for Gate {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        if self.wake >= 0 {
            let one: u64 = 1;
            // SAFETY: a valid eventfd and an 8-byte buffer.
            unsafe { libc::write(self.wake, (&one as *const u64).cast(), 8) };
        }
        if let Some(w) = self.watcher.take() {
            let _ = w.join();
        }
        if self.wake >= 0 {
            // SAFETY: our own fd, closed once.
            unsafe { libc::close(self.wake) };
        }
    }
}

/// The control device, read-only: find the card with "Speaker Protection",
/// read it, and follow its change events.
#[cfg(target_os = "linux")]
mod ctl {
    use super::*;

    pub(super) const PROTECTION_CONTROL: &str = "Speaker Protection";
    const IFACE_MIXER: i32 = 2;
    const ID_NAME_LEN: usize = 44;
    /// `SNDRV_CTL_EVENT_MASK_VALUE`, and the whole mask on removal.
    pub(super) const MASK_VALUE: u32 = 1;
    pub(super) const MASK_REMOVE: u32 = !0;
    /// `struct snd_ctl_event`: type, then the element's mask and id.
    pub(super) const EVENT_BYTES: usize = 72;

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub(super) struct ElemId {
        pub numid: u32,
        pub iface: i32,
        pub device: u32,
        pub subdevice: u32,
        pub name: [u8; ID_NAME_LEN],
        pub index: u32,
    }

    #[repr(C)]
    pub(super) struct ElemValue {
        pub id: ElemId,
        pub indirect: u32,
        pub value: [i64; 128],
        pub reserved: [u8; 128],
    }

    #[repr(C)]
    pub(super) struct ElemInfo {
        pub id: ElemId,
        pub kind: i32,
        pub access: u32,
        pub count: u32,
        pub owner: i32,
        pub value: [u64; 16],
        pub reserved: [u8; 64],
    }

    const fn ioc(dir: u64, nr: u64, size: usize) -> u64 {
        (dir << 30) | ((size as u64) << 16) | (0x55 << 8) | nr
    }
    const RW: u64 = 3;
    pub(super) const ELEM_INFO: u64 = ioc(RW, 0x11, core::mem::size_of::<ElemInfo>());
    pub(super) const ELEM_READ: u64 = ioc(RW, 0x12, core::mem::size_of::<ElemValue>());
    pub(super) const SUBSCRIBE_EVENTS: u64 = ioc(RW, 0x16, core::mem::size_of::<i32>());

    pub(super) fn elem_id(name: &str) -> ElemId {
        let mut id = ElemId { numid: 0, iface: IFACE_MIXER, device: 0, subdevice: 0, name: [0; ID_NAME_LEN], index: 0 };
        let n = name.len().min(ID_NAME_LEN - 1);
        id.name[..n].copy_from_slice(&name.as_bytes()[..n]);
        id
    }

    /// Each event in `buf` (whole `snd_ctl_event`s): whether one is a value
    /// change of `numid`, and whether one removes it.
    pub(super) fn scan_events(buf: &[u8], numid: u32) -> (bool, bool) {
        let (mut changed, mut removed) = (false, false);
        for ev in buf.chunks_exact(EVENT_BYTES) {
            let kind = i32::from_ne_bytes(ev[0..4].try_into().unwrap_or_default());
            let mask = u32::from_ne_bytes(ev[4..8].try_into().unwrap_or_default());
            let id = u32::from_ne_bytes(ev[8..12].try_into().unwrap_or_default());
            // SNDRV_CTL_EVENT_ELEM
            if kind != 0 || id != numid {
                continue;
            }
            if mask == MASK_REMOVE {
                removed = true;
            } else if mask & MASK_VALUE != 0 {
                changed = true;
            }
        }
        (changed, removed)
    }

    struct Card {
        fd: i32,
        id: ElemId,
    }

    impl Drop for Card {
        fn drop(&mut self) {
            // SAFETY: our own fd, closed once.
            unsafe { libc::close(self.fd) };
        }
    }

    impl Card {
        fn find() -> Option<Card> {
            (0..32).find_map(|n| Card::open(&format!("/dev/snd/controlC{n}\0")))
        }

        fn open(path: &str) -> Option<Card> {
            // SAFETY: a NUL-terminated path.
            let fd = unsafe { libc::open(path.as_ptr().cast(), libc::O_RDONLY | libc::O_CLOEXEC | libc::O_NONBLOCK) };
            if fd < 0 {
                return None;
            }
            let mut c = Card { fd, id: elem_id(PROTECTION_CONTROL) };
            let mut info = Box::new(ElemInfo {
                id: c.id,
                kind: 0,
                access: 0,
                count: 0,
                owner: 0,
                value: [0; 16],
                reserved: [0; 64],
            });
            // SAFETY: a valid fd and a live, correctly sized ElemInfo.
            if unsafe { libc::ioctl(fd, ELEM_INFO as _, &mut *info as *mut ElemInfo) } < 0 || info.id.numid == 0 {
                return None;
            }
            c.id = info.id;
            let on: i32 = 1;
            // SAFETY: a valid fd and a live i32.
            if unsafe { libc::ioctl(fd, SUBSCRIBE_EVENTS as _, &on as *const i32) } < 0 {
                return None;
            }
            Some(c)
        }

        fn granted(&self) -> Option<bool> {
            let mut v = Box::new(ElemValue { id: self.id, indirect: 0, value: [0; 128], reserved: [0; 128] });
            // SAFETY: a valid fd and a live, correctly sized ElemValue.
            if unsafe { libc::ioctl(self.fd, ELEM_READ as _, &mut *v as *mut ElemValue) } < 0 {
                return None;
            }
            Some(v.value[0] == 1)
        }
    }

    /// Wait for `fd` (or the wake fd); false: stop.
    fn wait(fd: i32, wake: i32, ms: i32, s: &Shared) -> bool {
        let mut p = [
            libc::pollfd { fd: wake, events: libc::POLLIN, revents: 0 },
            libc::pollfd { fd, events: libc::POLLIN, revents: 0 },
        ];
        let n = if fd < 0 { 1 } else { 2 };
        // SAFETY: `p` holds `n` live pollfds.
        unsafe { libc::poll(p.as_mut_ptr(), n, ms) };
        !s.stop.load(Ordering::Acquire) && p[0].revents == 0
    }

    pub(super) fn watch(s: &Shared, wake: i32) {
        while !s.stop.load(Ordering::Acquire) {
            let Some(card) = Card::find() else {
                s.lost();
                if !wait(-1, wake, 1000, s) {
                    break;
                }
                continue;
            };
            match card.granted() {
                Some(g) => s.set(g, false),
                None => {
                    s.lost();
                    if !wait(-1, wake, 1000, s) {
                        break;
                    }
                    continue;
                }
            }
            let mut buf = [0u8; EVENT_BYTES * 16];
            loop {
                if !wait(card.fd, wake, -1, s) {
                    s.lost();
                    return;
                }
                let (mut changed, mut removed) = (false, false);
                loop {
                    // SAFETY: a valid fd and a live buffer of its length.
                    let n = unsafe { libc::read(card.fd, buf.as_mut_ptr().cast(), buf.len()) };
                    if n <= 0 {
                        // EAGAIN: drained; an error or end: the card went
                        removed |= n == 0 || std::io::Error::last_os_error().raw_os_error() != Some(libc::EAGAIN);
                        break;
                    }
                    let (c, r) = scan_events(&buf[..n as usize], card.id.numid);
                    changed |= c;
                    removed |= r;
                }
                if removed {
                    s.lost();
                    break;
                }
                if changed {
                    match card.granted() {
                        Some(g) => s.set(g, true),
                        None => {
                            s.lost();
                            break;
                        }
                    }
                }
            }
        }
        s.lost();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48000.0;

    fn frames(ms: f64) -> usize {
        (RATE * ms / 1000.0).round() as usize
    }

    fn run(r: &mut Ramp, g: Grant, n: usize) -> Vec<f32> {
        r.follow(g);
        let mut a = vec![1.0f32; n];
        let mut b = vec![-0.5f32; n];
        let mut bufs: [&mut [f32]; 2] = [&mut a, &mut b];
        r.apply(&mut bufs, n);
        // both channels see the same gain
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(*y, -0.5 * x);
        }
        a
    }

    const GRANTED: Grant = Grant { known: true, granted: true, epoch: 0 };

    #[test]
    fn a_steady_grant_passes_the_program_untouched() {
        let mut r = Ramp::new(RATE);
        for _ in 0..10 {
            assert!(run(&mut r, GRANTED, 512).iter().all(|&g| g == 1.0));
        }
        assert!(r.open());
    }

    #[test]
    fn no_gate_passes_the_program() {
        let mut r = Ramp::new(RATE);
        let none = Grant { known: false, granted: false, epoch: 0 };
        assert!(run(&mut r, none, 512).iter().all(|&g| g == 1.0));
        assert_eq!(Gate::none().shared().load(), Grant { known: false, granted: false, epoch: 0 });
    }

    /// A withdrawal ramps out within RAMP_OUT_MS and mutes; the grant after
    /// it holds silence for HOLD_MS and ramps in smoothly over RAMP_IN_MS.
    #[test]
    fn a_regrant_holds_then_ramps_in_without_a_step() {
        let mut r = Ramp::new(RATE);
        run(&mut r, GRANTED, 512);
        let off = Grant { known: true, granted: false, epoch: 1 };
        let g = run(&mut r, off, 512);
        let out = frames(RAMP_OUT_MS);
        assert!(g[..out].windows(2).all(|w| w[1] <= w[0]));
        assert!(g[out..].iter().all(|&x| x == 0.0));
        // the stream is stopped; the graph may still run: silence
        assert!(run(&mut r, off, 2048).iter().all(|&x| x == 0.0));
        let on = Grant { known: true, granted: true, epoch: 2 };
        let mut all = Vec::new();
        for _ in 0..10 {
            all.extend(run(&mut r, on, 512));
        }
        let hold = frames(HOLD_MS);
        assert!(all[..hold].iter().all(|&x| x == 0.0));
        let ramp = frames(RAMP_IN_MS);
        let rise = &all[hold..hold + ramp];
        assert!(rise.windows(2).all(|w| w[1] >= w[0]));
        // smooth: no step bigger than a linear ramp's 1.5x
        let max_step = rise.windows(2).map(|w| w[1] - w[0]).fold(0.0f32, f32::max);
        assert!(max_step <= 1.5 / ramp as f32 + 1e-6, "{max_step}");
        assert!(all[hold + ramp..].iter().all(|&x| x == 1.0));
        assert!(r.open());
    }

    /// A withdrawal and a grant between two calls (the plugin never ran
    /// muted): the epoch tells, and the program still restarts from silence.
    #[test]
    fn an_unseen_withdrawal_still_ramps_in() {
        let mut r = Ramp::new(RATE);
        run(&mut r, GRANTED, 512);
        let g = run(&mut r, Grant { known: true, granted: true, epoch: 2 }, 512);
        assert!(g.iter().all(|&x| x == 0.0));
        assert!(!r.open());
    }

    /// Started while not granted (boot): silent, then hold and ramp at the
    /// first grant.
    #[test]
    fn the_first_grant_after_start_ramps_in() {
        let mut r = Ramp::new(RATE);
        let off = Grant { known: true, granted: false, epoch: 0 };
        assert!(run(&mut r, off, 512).iter().all(|&x| x == 0.0));
        let on = Grant { known: true, granted: true, epoch: 1 };
        let g = run(&mut r, on, 512);
        assert!(g.iter().all(|&x| x == 0.0));
        let mut n = 512;
        while !r.open() {
            run(&mut r, on, 512);
            n += 512;
        }
        assert!(n >= frames(HOLD_MS + RAMP_IN_MS));
    }

    fn run_with(r: &mut Ramp, g: Grant, input: &[f32]) -> Vec<f32> {
        r.follow(g);
        let mut a = input.to_vec();
        let mut b: Vec<f32> = input.iter().map(|x| -x).collect();
        let n = a.len();
        let mut bufs: [&mut [f32]; 2] = [&mut a, &mut b];
        r.apply(&mut bufs, n);
        a
    }

    /// A grant while the program is silent (the wake from idle): no hold,
    /// no ramp; the first program frames after it pass untouched.
    #[test]
    fn a_grant_over_silence_passes_the_program_at_once() {
        let mut r = Ramp::new(RATE);
        let off = Grant { known: true, granted: false, epoch: 0 };
        let on = Grant { known: true, granted: true, epoch: 1 };
        // muted while idle, the input silent
        assert!(run_with(&mut r, off, &[0.0; 512]).iter().all(|&x| x == 0.0));
        // granted: still silent, then the program starts
        run_with(&mut r, on, &[0.0; 512]);
        assert!(r.open());
        let prog: Vec<f32> = (0..512).map(|i| 0.5 + i as f32 * 1e-3).collect();
        assert_eq!(run_with(&mut r, on, &prog), prog);
        // a grant over a silence shorter than SILENT_RUN still holds
        let mut r = Ramp::new(RATE);
        let mut tail = vec![0.7f32; 512];
        tail[500..].fill(0.0);
        run_with(&mut r, off, &tail);
        run_with(&mut r, on, &[0.7; 512]);
        assert!(!r.open());
    }

    /// The watcher losing the control never leaves the program muted.
    #[test]
    fn a_lost_gate_ramps_back_in() {
        let mut r = Ramp::new(RATE);
        run(&mut r, Grant { known: true, granted: false, epoch: 0 }, 512);
        let none = Grant { known: false, granted: false, epoch: 0 };
        let g = run(&mut r, none, frames(RAMP_IN_MS) + 64);
        assert_eq!(g[0], smooth(1.0 / frames(RAMP_IN_MS) as f32));
        assert_eq!(*g.last().unwrap_or(&0.0), 1.0);
    }

    /// A stream start (activate) ramps in; a muted or held gate stays so.
    #[test]
    fn a_restart_ramps_in() {
        let mut r = Ramp::new(RATE);
        run(&mut r, GRANTED, 512);
        r.restart();
        let g = run(&mut r, GRANTED, 1024);
        let ramp = frames(RAMP_IN_MS);
        assert!(g[0] < 0.001 && g[..ramp].windows(2).all(|w| w[1] >= w[0]));
        assert!(g[ramp..].iter().all(|&x| x == 1.0));
        let off = Grant { known: true, granted: false, epoch: 1 };
        run(&mut r, off, 512);
        r.restart();
        assert!(run(&mut r, off, 512).iter().all(|&x| x == 0.0));
    }

    #[test]
    fn shared_state_bumps_the_epoch_on_changes_only() {
        let s = Shared::default();
        s.set(true, false);
        assert_eq!(s.load(), Grant { known: true, granted: true, epoch: 0 });
        s.set(false, true);
        s.set(true, true);
        assert_eq!(s.load(), Grant { known: true, granted: true, epoch: 2 });
        s.lost();
        assert!(!s.load().known);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn control_events_are_parsed_by_numid() {
        use super::ctl::*;
        let ev = |kind: i32, mask: u32, numid: u32| {
            let mut e = [0u8; EVENT_BYTES];
            e[0..4].copy_from_slice(&kind.to_ne_bytes());
            e[4..8].copy_from_slice(&mask.to_ne_bytes());
            e[8..12].copy_from_slice(&numid.to_ne_bytes());
            e
        };
        let mut buf = Vec::new();
        buf.extend(ev(0, MASK_VALUE, 7));
        assert_eq!(scan_events(&buf, 7), (true, false));
        assert_eq!(scan_events(&buf, 8), (false, false));
        buf.extend(ev(0, MASK_REMOVE, 7));
        assert_eq!(scan_events(&buf, 7), (true, true));
        // an info-only change of the control is no value change
        assert_eq!(scan_events(&ev(0, 2, 7), 7), (false, false));
        assert_eq!(scan_events(&ev(1, MASK_VALUE, 7), 7), (false, false));
    }

    #[cfg(all(target_os = "linux", target_pointer_width = "64"))]
    #[test]
    fn ioctl_numbers_match_the_kernel_uapi() {
        use super::ctl::*;
        assert_eq!(core::mem::size_of::<ElemId>(), 64);
        assert_eq!(core::mem::size_of::<ElemValue>(), 1224);
        assert_eq!(core::mem::size_of::<ElemInfo>(), 272);
        assert_eq!(ELEM_INFO, 0xc110_5511);
        assert_eq!(ELEM_READ, 0xc4c8_5512);
        assert_eq!(SUBSCRIBE_EVENTS, 0xc004_5516);
        let id = elem_id(PROTECTION_CONTROL);
        assert_eq!(&id.name[..PROTECTION_CONTROL.len()], PROTECTION_CONTROL.as_bytes());
        assert_eq!(id.name[PROTECTION_CONTROL.len()], 0);
    }
}
