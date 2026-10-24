// SPDX-License-Identifier: MIT
//! The protection loop, behind a small hardware interface so that its
//! timing rules can be exercised without a sound card: the card's volume
//! lock, its speaker sample-rate control and the sense capture.
//!
//! The rules it keeps:
//!
//! * The card's volume lock is only renewed while every safety-critical
//!   control operation of the pass succeeded: a failed sample-rate read or
//!   volume write hands the speakers to the card's safe level at once.
//! * Every moment of playback is accounted, by measured sense frames or at
//!   the worst case the card's gain lets through: a period covers the time
//!   it was captured over (its backlog in the capture buffer taken into
//!   account), and time no period covers is charged before the next one
//!   is.  A stream's edges too: the sample-rate control's change events
//!   wake the loop when a stream opens or closes, the window from that
//!   wake-up to the first captured frame is charged at the worst case, and
//!   at close the frames after the last whole period are read and measured
//!   and the rest of the window is charged.  Without change events the
//!   opening window runs from the last time the speakers were seen closed.
//! * The lock is not taken before the model allows it and the daemon can
//!   see: no unlock at start, and while the speakers play not before
//!   REARM_PERIODS complete periods.
//! * A model that may have reached a coil's limit inside a period latches a
//!   hard fault; the card stays with the kernel until the coil has cooled.
//! * The state a restart resumes from is written before the lock is taken
//!   and at least every SAVE_MAX_AGE while it is held.

use std::error::Error;
use std::io;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use crate::config::{Config, Globals};
use crate::model::{self, SpeakerState, SUB_BLOCKS};
use crate::state;
use crate::{debug, info, warn};

/// The kernel locks the volume again 250 ms after the last write; write well
/// inside that while the speakers play.
pub const PING_INTERVAL: Duration = Duration::from_millis(100);
/// Cadence of the per-speaker debug line.
const LOG_INTERVAL: Duration = Duration::from_secs(1);
/// Cadence of the model state file.
pub const SAVE_INTERVAL: Duration = Duration::from_millis(500);
/// The lock is not held on a state file older than this.
pub const SAVE_MAX_AGE: Duration = Duration::from_secs(1);
/// Consecutive failures of one control operation (about a minute of
/// retries) before the daemon gives up and exits.  Every failure already
/// hands the speakers to the card's safe level; exiting only stops the
/// retries.
const MAX_CONTROL_FAILURES: u32 = 600;
/// Sense data missing for this long means the stream is gone, not late.
pub const STALL_TIMEOUT: Duration = Duration::from_millis(500);

/// What the loop needs from the machine.
pub trait Hw {
    fn now(&self) -> Instant;
    /// Time since boot, suspend included (CLOCK_BOOTTIME): the model state
    /// saved by an earlier run is charged with the time between the runs.
    fn boottime(&self) -> Duration;
    fn sleep(&mut self, d: Duration);
    /// Lock every volume control and the unlock control.  No ping: the
    /// card's volume stays limited until the daemon decides to lift it.
    fn take_lock(&mut self) -> io::Result<()>;
    /// Renew the card's volume lock ("Speaker Volume Unlock").
    fn ping(&mut self) -> io::Result<()>;
    /// Write the gain reduction (dB below the control's maximum).
    fn set_reduction(&mut self, db: f64) -> io::Result<()>;
    /// Turn every volume control to its minimum, which the card accepts
    /// while the volume is locked: done before each unlock, so that the
    /// control is known to work and the unlock never lets an older, louder
    /// value through.
    fn set_min(&mut self) -> io::Result<()>;
    /// Give the card's lock up and take it again: the card locks the
    /// volume at once and drops what was left of its lease.
    fn relinquish(&mut self) -> io::Result<()>;
    /// The speakers' sample rate, 0 while they are closed.
    fn sample_rate(&mut self) -> io::Result<i64>;
    fn open_sense(&mut self, rate: u32) -> io::Result<()>;
    fn close_sense(&mut self);
    /// One sense period: `Some(true)` complete, `Some(false)` short or
    /// overrun, `None` nothing within one poll (at most 50 ms).
    fn read_period(&mut self, words: &mut [i32]) -> io::Result<Option<bool>>;
    /// Frames the capture holds beyond the period just read.
    fn queued(&mut self) -> io::Result<u64>;
    /// Whatever the capture holds now, up to a period, without waiting
    /// (frames; 0 when there are none or the stream cannot give them).
    fn read_tail(&mut self, words: &mut [i32]) -> io::Result<usize>;
    /// Wait up to `timeout` for a change of the speakers' sample rate.
    /// True if one woke the wait.
    fn wait_rate_change(&mut self, timeout: Duration) -> bool;
    /// Whether the sample rate changed since the last call (consumes it).
    fn rate_events(&mut self) -> bool;
}

/// Errors a PCM or control returns while the playback side is being
/// reconfigured underneath us (the sense front-end follows the speaker
/// back-end, which PipeWire opens, suspends and reopens at will).
pub fn transient(e: &io::Error) -> bool {
    matches!(
        e.raw_os_error(),
        Some(libc::EINVAL | libc::EBADFD | libc::EPIPE | libc::ESTRPIPE | libc::ENODEV
            | libc::EIO | libc::ETIMEDOUT | libc::EAGAIN | libc::EINTR | libc::ENXIO)
    )
}

/// How long a sense capture has gone without data.
#[derive(Default)]
pub struct StallClock(Option<Instant>);

impl StallClock {
    /// A period arrived.
    pub fn data(&mut self) {
        self.0 = None;
    }

    /// Nothing arrived by `now`.  True once the stream has been silent for
    /// STALL_TIMEOUT, counted from the first empty wait.
    pub fn silent(&mut self, now: Instant) -> bool {
        let since = *self.0.get_or_insert(now);
        now.duration_since(since) >= STALL_TIMEOUT
    }
}

/// How late the next sense period may be before the time since the model's
/// account ends is charged at the worst case.  Only when to charge: once
/// it is, all of that time is.
pub const BLIND_SLACK: Duration = PING_INTERVAL;
/// Without complete sense data for this long (after the period that should
/// have arrived) the daemon stops renewing the card's volume lock and the
/// kernel drops the speakers to its safe level within 250 ms.
pub const BLIND_LIMIT: Duration = STALL_TIMEOUT;
/// The kernel's lock timeout (macaudio SPEAKER_LOCK_TIMEOUT).
pub const KERNEL_LOCK_TIMEOUT: Duration = Duration::from_millis(250);
/// The longest a run's card may have played at its gain after the last
/// state it wrote: the state is refreshed every SAVE_INTERVAL and the lock
/// dropped once it is SAVE_MAX_AGE old, the kernel takes the volume back
/// 250 ms after the last ping, and a pass of the loop may be one poll late.
pub const RESTORE_TAIL: Duration = Duration::from_millis(1500);
/// Consecutive complete periods needed before the lock is taken back.
pub const REARM_PERIODS: u32 = 2;
/// Hysteresis between handing the card to the kernel at --max-reduction and
/// taking it back.
pub const REARM_MARGIN_DB: f64 = 3.0;
/// How far apart two sense periods' account may be before the time between
/// them counts as uncovered: the capture pointer's granularity and the time
/// between reading it and the clock.  Up to this much is covered by the
/// next period's own power instead (at most 2.3 % of a 4096-frame period).
pub const PERIOD_JITTER: Duration = Duration::from_millis(2);
/// The reduction the volume controls' minimum stands for (dB).
pub const MUTE_DB: f64 = 120.0;

pub struct Guard {
    globals: Globals,
    pub speakers: Vec<SpeakerState>,
    max_reduction: f64,
    /// True while the daemon renews the card's volume lock.  False hands the
    /// speakers to the kernel's safe level; the daemon keeps modelling and
    /// takes the lock back when it can see and the speaker allows.
    armed: bool,
    /// Ever armed in this run (for the log).
    held: bool,
    /// The open sense capture's stall clock; it lives and dies with the
    /// capture: one left over from an earlier stream would call the next
    /// stream stalled at its first empty wait, before its first period.
    stall: Option<StallClock>,
    /// Seconds per sense period while the speakers play, None while closed.
    dt: Option<f64>,
    /// While the speakers may be playing: the model has accounted for the
    /// time up to here, with measured or worst-case power.  None while they
    /// are known to be closed.
    charged_to: Option<Instant>,
    /// When the last complete sense period was read (the stream's first
    /// sighting until one is).
    seen_to: Option<Instant>,
    /// When the speakers were last seen closed.
    last_closed: Option<Instant>,
    /// When a sample-rate change last woke the idle wait.
    event_wake: Option<Instant>,
    /// The model's idle cooling has run up to here.
    idle_to: Option<Instant>,
    /// The speakers' rate while they play (0 before the first stream).
    rate: u32,
    /// The last sample-rate read failed: playback state unknown.
    unknown: bool,
    /// Complete periods in a row.
    fresh: u32,
    words: Vec<i32>,
    last_ping: Instant,
    last_log: Instant,
    /// The reduction the card's volume controls hold, as far as the daemon
    /// knows (dB; infinite = unknown, taken as full gain).
    written: f64,
    applied: f64,
    /// What the controls held and whether the lock was renewed, since the
    /// model's account ends: (from, written, armed), oldest first, the first
    /// in effect at the account's end.  Time is charged at what the card
    /// held while it passed, not at what it holds when the charge is made
    /// (a hand-off writes the safe level before the time played at the old
    /// gain is charged).
    held_gains: Vec<(Instant, f64, bool)>,
    /// What is left of the kernel's lease since the last ping, in playing
    /// time: macaudio pauses it while the speakers are idle, so idle time
    /// never runs it out (released only by playing, or by the owner going).
    lease_left: Duration,
    /// The last complete period and anything after it came from the same
    /// capture without a loss: the next frames follow on directly.
    contiguous: bool,
    /// When the model was last advanced while the speakers were closed
    /// (start, then every idle step).
    accounted_idle: Instant,
    rate_failures: u32,
    /// Failures of the volume writes: to the minimum before an unlock, and
    /// to the governed reduction.  Each clears only on its own success.
    min_failures: u32,
    target_failures: u32,
    state_path: Option<PathBuf>,
    last_save: Option<Instant>,
    last_save_try: Option<Instant>,
    save_failed: bool,
    stopping: bool,
}

impl Guard {
    /// Restore the model from the last run (if any) and lock the card's
    /// controls.  The volume lock is not lifted here: the first passes of
    /// the loop take it once the model allows and the daemon can see.
    pub fn start<H: Hw>(hw: &mut H, cfg: &Config, max_reduction: f64, state_path: Option<PathBuf>) -> Result<Guard, Box<dyn Error>> {
        if !max_reduction.is_finite() || max_reduction <= 0.0 {
            return Err("maximum reduction must be finite and positive".into());
        }
        let globals = cfg.globals.clone();
        let mut speakers: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), globals.t_ambient)).collect();
        // nothing about the speakers' past is known: at the coil's limit,
        // latched, until the model has cooled under the working ceiling
        let hot = |speakers: &mut Vec<SpeakerState>| {
            for s in speakers.iter_mut() {
                s.assume_latched(globals.t_ambient);
            }
        };
        match &state_path {
            None => {
                warn!("no model state file: assuming hot speakers (latched)");
                hot(&mut speakers);
            }
            Some(path) => match std::fs::read_to_string(path) {
                Ok(text) => match state::decode(&text) {
                    Some(saved) => {
                        let now = hw.boottime();
                        state::restore(&mut speakers, &saved, now, RESTORE_TAIL, &globals);
                        info!(
                            "model restored from {} (covering up to {:.1} s ago{}{})",
                            path.display(),
                            now.saturating_sub(saved.covered).as_secs_f64(),
                            if saved.armed { ", its run may have held the card" } else { "" },
                            if speakers.iter().any(|s| s.hard) { ", latched" } else { "" }
                        );
                    }
                    None => {
                        warn!("{}: unreadable model state; assuming hot speakers (latched)", path.display());
                        hot(&mut speakers);
                    }
                },
                Err(e) if e.kind() == io::ErrorKind::NotFound => {
                    // the absence of this daemon's state proves nothing
                    // about the speakers: an older daemon with another state
                    // path, a failed write, a warm reboot
                    info!("{}: no model state; assuming hot speakers (latched)", path.display());
                    hot(&mut speakers);
                }
                Err(e) => {
                    warn!("{}: {e}; assuming hot speakers (latched)", path.display());
                    hot(&mut speakers);
                }
            },
        }
        for s in speakers.iter_mut() {
            s.govern(&globals);
            if !s.finite() {
                return Err("nonfinite speaker model state".into());
            }
        }
        hw.take_lock()?;
        let now = hw.now();
        Ok(Guard {
            words: vec![0i32; globals.period * globals.channels],
            globals,
            speakers,
            max_reduction,
            armed: false,
            held: false,
            stall: None,
            dt: None,
            charged_to: None,
            seen_to: None,
            last_closed: None,
            event_wake: None,
            idle_to: None,
            rate: 0,
            unknown: false,
            fresh: 0,
            // an earlier run may have pinged just before: its lock runs out
            // KERNEL_LOCK_TIMEOUT from now at the latest
            last_ping: now,
            last_log: now,
            written: f64::INFINITY,
            applied: 0.0,
            held_gains: vec![(now, f64::INFINITY, false)],
            // a new owner has no lease until it pings (macaudio clears it)
            lease_left: Duration::ZERO,
            contiguous: false,
            accounted_idle: now,
            rate_failures: 0,
            min_failures: 0,
            target_failures: 0,
            state_path,
            last_save: None,
            last_save_try: None,
            save_failed: false,
            stopping: false,
        })
    }

    #[cfg(test)]
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Stop renewing the lock for good (SIGTERM): the controls go to the
    /// card's safe level; the loop keeps modelling until `released` and the
    /// caller then saves and exits (closing the control file ends the
    /// lease in any case).
    pub fn stop<H: Hw>(&mut self, hw: &mut H) {
        if self.armed {
            info!("stopping: leaving the speakers at the card's safe level");
        }
        self.stopping = true;
        if let Err(e) = self.hand_off(hw) {
            warn!("{e}");
        }
    }

    /// The card's speakers are at most at its safe level: the controls are
    /// there, or no lease is left.
    pub fn released<H: Hw>(&self, _hw: &H) -> bool {
        !self.armed && (self.lease_left.is_zero() || self.written_db() >= self.globals.safe_reduction)
    }

    /// The reduction the controls hold, unknown taken as full gain.
    fn written_db(&self) -> f64 {
        if self.written.is_finite() { self.written } else { 0.0 }
    }

    /// The controls or the lock changed at `at`.
    fn note_held(&mut self, at: Instant) {
        let same = self.held_gains.last().is_some_and(|&(_, w, a)| (w == self.written || (w.is_nan() && self.written.is_nan())) && a == self.armed);
        if !same {
            self.held_gains.push((at, self.written, self.armed));
        }
    }

    /// Forget what the controls held before `to` (the account's end),
    /// keeping the entry in effect there.
    fn prune_held(&mut self, to: Instant) {
        let keep = self.held_gains.iter().rposition(|&(t, _, _)| t <= to).unwrap_or(0);
        self.held_gains.drain(..keep);
    }

    /// Over [from, to): the least and the most reduction the controls held
    /// (unknown counts as full gain), and whether the lock was renewed at
    /// any time in it.
    fn held_over(&self, from: Instant, to: Instant) -> (f64, f64, bool) {
        let (mut least, mut most, mut armed) = (f64::INFINITY, 0.0f64, false);
        for (i, &(t, w, a)) in self.held_gains.iter().enumerate() {
            let ends_after = self.held_gains.get(i + 1).is_none_or(|next| next.0 > from);
            if !(ends_after && (t < to || i == 0)) {
                continue;
            }
            let v = if w.is_finite() { w } else { 0.0 };
            least = least.min(v);
            most = most.max(v);
            armed |= a;
        }
        if least.is_infinite() {
            return (self.written_db(), self.written_db(), self.armed);
        }
        (least, most, armed)
    }

    /// Stop renewing, and bring the speakers to the card's safe level at
    /// once, whatever lease the kernel has left (macaudio pauses it while
    /// the speakers are idle, so it can outlast any wait): the controls are
    /// turned down to the safe level, a value the card takes with or
    /// without the lock; if that write fails, the card's lock is given up
    /// and taken again, which locks the volume and clears the lease.  If
    /// both fail the daemon exits, which does the same.
    fn hand_off<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        self.armed = false;
        self.note_held(hw.now());
        if self.written_db() >= self.globals.safe_reduction || self.lease_left.is_zero() {
            return Ok(());
        }
        let safe = self.globals.safe_reduction;
        match hw.set_reduction(safe) {
            Ok(()) => {
                self.written = safe;
                self.note_held(hw.now());
                return Ok(());
            }
            Err(e) => warn!("volume control at hand-off: {e}; giving the lock up instead"),
        }
        match hw.relinquish() {
            Ok(()) => {
                self.lease_left = Duration::ZERO;
                Ok(())
            }
            Err(e) => Err(format!("giving up the volume lock: {e}").into()),
        }
    }

    fn disarm<H: Hw>(&mut self, hw: &mut H, why: &str) -> Result<(), Box<dyn Error>> {
        if self.armed {
            warn!("{why}; leaving the speakers at the card's safe level");
        }
        self.hand_off(hw)
    }

    /// Pings the volume lock.  A ping that answers ETIMEDOUT is the card's
    /// one-shot notice that the lock had lapsed (it does not re-arm the
    /// timer), so ping once more; otherwise every volume write until the
    /// next ping fails with EINVAL against the safe limit.
    fn ping_lock<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        match hw.ping() {
            Ok(()) => {}
            Err(e) if e.raw_os_error() == Some(libc::ETIMEDOUT) => {
                if self.armed {
                    warn!("the card locked the volume while we were away; taking it back");
                }
                hw.ping().map_err(|e| format!("lock ping: {e}"))?;
            }
            Err(e) => return Err(format!("lock ping: {e}").into()),
        }
        self.last_ping = hw.now();
        self.lease_left = KERNEL_LOCK_TIMEOUT;
        Ok(())
    }

    /// The reductions the card applies over `d` seconds of playing time
    /// from now on in the account, as (seconds, dB) pieces, using up the
    /// kernel's lease: the controls' value while the lock is held or the
    /// lease lasts, at least the safe level after.
    fn playing_reductions(&mut self, d: f64, written: f64, armed: bool) -> Vec<(f64, f64)> {
        if armed || written >= self.globals.safe_reduction {
            return vec![(d, written)];
        }
        let first = d.min(self.lease_left.as_secs_f64());
        self.lease_left = self.lease_left.saturating_sub(Duration::from_secs_f64(first));
        let mut out = Vec::with_capacity(2);
        if first > 0.0 {
            out.push((first, written));
        }
        if d > first {
            out.push((d - first, self.globals.safe_reduction));
        }
        out
    }

    /// The reduction the card applied while measured frames played, for
    /// the budgets' power before its gain: the larger one it could have
    /// been (more power before the gain is the safer guess there).  The
    /// frames are playing time, so they use up the lease too.
    fn measured_reduction(&mut self, d: f64, (least, most, armed): (f64, f64, bool)) -> f64 {
        let pieces = self.playing_reductions(d, least, armed);
        if armed {
            most
        } else {
            pieces.iter().map(|p| p.1).fold(most, f64::max)
        }
    }

    fn check_model(&self) -> Result<(), Box<dyn Error>> {
        if self.speakers.iter().all(|s| s.finite()) {
            Ok(())
        } else {
            Err("nonfinite speaker model state".into())
        }
    }

    /// Charge [from, to) as playing at the most power the card's gain lets
    /// through, the gain dropping where the kernel's lease runs out.
    fn charge_worst(&mut self, from: Instant, to: Instant) -> Result<(), Box<dyn Error>> {
        if to <= from {
            return Ok(());
        }
        // at the least reduction the controls held over the span
        let (least, _, armed) = self.held_over(from, to);
        for (d, reduction) in self.playing_reductions((to - from).as_secs_f64(), least, armed || self.armed) {
            let globals = &self.globals;
            for s in self.speakers.iter_mut() {
                let p = s.worst_case_power(reduction);
                s.step_at(p, d, globals.t_ambient, reduction);
                s.govern(globals);
            }
        }
        self.prune_held(to);
        self.check_model()
    }

    /// `frames` sense frames in `self.words` whose last one was captured at
    /// `end` (the clock's estimate: the read time less the backlog).
    ///
    /// Frames that follow the last ones in the same capture without a loss
    /// cover the time right after them, whatever the clock's estimate says
    /// (it carries the pointer's granularity and the read's own timing).
    /// Where the estimate runs ahead of the frames by more than
    /// PERIOD_JITTER, the capture has fallen behind real time and the
    /// difference is charged at the worst case.  Frames after a loss or a
    /// new capture are placed by the estimate, and all of the time before
    /// them since the account ended is charged at the worst case.
    fn account_frames(&mut self, end: Instant, frames: usize, contiguous: bool) -> Result<(), Box<dyn Error>> {
        if frames == 0 || self.rate == 0 {
            return Ok(());
        }
        let dt = frames as f64 / self.rate as f64;
        let span_d = Duration::from_secs_f64(dt);
        let (lo, behind) = match self.charged_to {
            Some(c) if contiguous => (c, end.checked_sub(span_d).filter(|&s| s > c + PERIOD_JITTER)),
            Some(c) => {
                let start = end.checked_sub(span_d).unwrap_or(end);
                if c < start {
                    self.charge_worst(c, start)?;
                }
                (start.max(c), None)
            }
            None => (end.checked_sub(span_d).unwrap_or(end), None),
        };
        if let (Some(start), true) = (behind, contiguous) {
            // the capture fell behind: the frames cover their own length,
            // the time between is charged
            self.charge_worst(lo, start)?;
        }
        let lo = behind.unwrap_or(lo);
        // a part of the frames' span already charged at the worst case is not
        // charged again
        let span = if contiguous { dt } else { end.saturating_duration_since(lo).as_secs_f64().min(dt) };
        let held = self.held_over(lo, end.max(lo + span_d));
        let applied = self.measured_reduction(span, (held.0, held.1, held.2 || self.armed));
        let globals = self.globals.clone();
        let words = &self.words[..frames * globals.channels];
        let blocks = (SUB_BLOCKS * frames).div_ceil(globals.period).max(1);
        for s in self.speakers.iter_mut() {
            let (powers, v) = s.block_powers(words, globals.channels, blocks);
            if !v.is_finite() || powers.iter().any(|p| !p.is_finite() || *p < 0.0) {
                return Err("invalid speaker sense voltage or power".into());
            }
            s.v_rms = v;
            if span > 0.0 {
                s.step_period(&powers, span, globals.t_ambient, applied);
                s.govern(&globals);
            }
        }
        self.check_model()?;
        // contiguous frames end where they end, but never far ahead of the
        // clock (a capture running fast must not cover the future)
        let covered = if contiguous { (lo + span_d).min(end + PERIOD_JITTER) } else { lo.max(end) };
        let covered = covered.max(self.charged_to.unwrap_or(covered));
        self.charged_to = Some(covered);
        self.prune_held(covered);
        Ok(())
    }

    /// A closing stream: measure what the capture holds after the last
    /// whole period, then charge the rest of the window up to now.
    fn account_close<H: Hw>(&mut self, hw: &mut H) -> Result<usize, Box<dyn Error>> {
        let mut measured = 0;
        let Some(_) = self.charged_to else { return Ok(0) };
        if self.stall.is_some() && !self.unknown {
            loop {
                let mut words = std::mem::take(&mut self.words);
                let read = hw.read_tail(&mut words);
                self.words = words;
                let frames = match read {
                    Ok(f) => f,
                    Err(e) => {
                        warn!("sense capture at close: {e}");
                        0
                    }
                };
                if frames == 0 {
                    break;
                }
                let end = hw.now().checked_sub(Duration::from_secs_f64(hw.queued().unwrap_or(0) as f64 / self.rate.max(1) as f64)).unwrap_or(hw.now());
                let contiguous = self.contiguous;
                self.account_frames(end, frames, contiguous)?;
                measured += frames;
                if frames < self.globals.period || measured > 8 * self.globals.period {
                    break;
                }
            }
        }
        let now = hw.now();
        if let Some(charged) = self.charged_to {
            self.charge_worst(charged, now)?;
        }
        self.charged_to = None;
        self.contiguous = false;
        Ok(measured)
    }

    /// The idle wait: until the next ping is due or the sample rate changes.
    fn idle_wait<H: Hw>(&mut self, hw: &mut H) {
        if hw.wait_rate_change(PING_INTERVAL) {
            self.event_wake = Some(hw.now());
        }
    }

    /// One pass of the loop: idle cooling, or one sense period.
    pub fn tick<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        let globals = self.globals.clone();
        // a change event since the last pass: read before the rate, so that
        // a change after the read is kept for the next pass
        let changed = hw.rate_events();
        let woke = self.event_wake.take();
        // playback state: the card reports the speakers' sample rate, 0 when closed
        let rate = match hw.sample_rate() {
            Ok(v) => {
                self.rate_failures = 0;
                self.unknown = false;
                u32::try_from(v).map_err(|_| "invalid negative or excessive speaker sample rate")?
            }
            Err(e) if transient(&e) && self.rate_failures < MAX_CONTROL_FAILURES => {
                // Playback state unknown: the speakers may be playing, from
                // the last moment they were seen closed if they were.
                self.rate_failures += 1;
                warn!("sample-rate control: {e}; retrying ({})", self.rate_failures);
                self.unknown = true;
                self.fresh = 0;
                if self.charged_to.is_none() {
                    self.charged_to = Some(self.last_closed.unwrap_or_else(|| hw.now()));
                }
                self.idle_to = None;
                self.contiguous = false;
                self.disarm(hw, "the speaker sample-rate control failed")?;
                self.maintain(hw)?;
                hw.sleep(PING_INTERVAL);
                return Ok(());
            }
            Err(e) => return Err(format!("sample-rate control: {e}").into()),
        };
        if rate == 0 {
            if self.charged_to.is_none() && changed {
                // a stream opened and closed again between two looks: the
                // time since the wake-up (or the last look) is unmeasured
                if let Some(from) = woke.or(self.last_closed) {
                    self.charged_to = Some(from);
                }
            }
            // the stream's last frames, and the rest of its closing window
            let tail = self.account_close(hw)?;
            let now_c = hw.now();
            self.prune_held(now_c);
            self.dt = None;
            self.seen_to = None;
            self.fresh = 0;
            if self.stall.take().is_some() {
                hw.close_sense();
                info!("speakers closed ({tail} sense frames after the last whole period)");
            }
            // cool down with no power for the time the speakers were closed
            let now = hw.now();
            let idle = self.idle_to.map_or(0.0, |t| now.saturating_duration_since(t).as_secs_f64());
            for s in self.speakers.iter_mut() {
                s.step(0.0, idle, globals.t_ambient);
                s.govern(&globals);
            }
            self.idle_to = Some(now);
            self.accounted_idle = now;
            self.check_model()?;
            self.last_closed = Some(now);
            self.maintain(hw)?;
            self.idle_wait(hw);
            return Ok(());
        }
        let dt = globals.period as f64 / rate as f64;
        if !dt.is_finite() || dt <= 0.0 {
            return Err("invalid sense period".into());
        }
        self.dt = Some(dt);
        self.rate = rate;
        self.idle_to = None;
        if self.charged_to.is_none() {
            // a stream seen for the first time is accounted from the change
            // event that announced it, or from the last time the speakers
            // were seen closed; the window up to its first frame is charged
            let now = hw.now();
            let from = if changed { woke.or(self.last_closed) } else { self.last_closed };
            self.charged_to = Some(from.map_or(now, |f| f.min(now)));
            self.seen_to = Some(now);
        }
        if self.seen_to.is_none() {
            self.seen_to = self.charged_to;
        }
        if self.stall.is_none() {
            // Opening the sense PCM blocks while the playback side is being
            // set up (about 100 ms on the J700): renew a lock that is still
            // held first, so the open cannot run its lease out.  The time is
            // charged as unmeasured all the same, and an open that blocks
            // past the lease lets it lapse.
            if self.armed && hw.now() < self.last_ping + KERNEL_LOCK_TIMEOUT {
                self.ping_lock(hw)?;
            }
            self.contiguous = false;
            match hw.open_sense(rate) {
                Ok(()) => {
                    info!("speakers open at {rate} Hz, sense capture running");
                    self.stall = Some(StallClock::default());
                }
                Err(e) => {
                    warn!("sense PCM: {e}; retrying");
                    self.maintain(hw)?;
                    hw.sleep(PING_INTERVAL);
                    return Ok(());
                }
            }
        }
        // The sense stream dies with the speaker back-end whenever playback is
        // torn down and rebuilt (EINVAL/EBADFD/EPIPE...).  That is not our
        // failure: drop the capture and reopen it.  Until complete periods
        // flow again the model charges the time at the worst case, and a gap
        // longer than BLIND_LIMIT hands the speakers to the kernel's safe
        // level until they do.
        let mut words = std::mem::take(&mut self.words);
        let read = hw.read_period(&mut words);
        self.words = words;
        match read {
            Ok(Some(true)) => {
                let queued = match hw.queued() {
                    Ok(q) => q,
                    Err(e) => {
                        // without its position the period cannot be placed
                        warn!("sense capture position: {e}; period discarded");
                        self.fresh = 0;
                        self.contiguous = false;
                        return self.maintain(hw);
                    }
                };
                self.stall.as_mut().unwrap().data();
                let now = hw.now();
                let end = now.checked_sub(Duration::from_secs_f64(queued as f64 / rate as f64)).unwrap_or(now);
                let contiguous = self.contiguous;
                self.account_frames(end, globals.period, contiguous)?;
                self.contiguous = true;
                self.seen_to = Some(now);
                self.fresh = self.fresh.saturating_add(1);
            }
            Ok(Some(false)) => {
                // its frames are lost: the next complete period charges them
                warn!("incomplete sense period or overrun, step skipped");
                self.fresh = 0;
                self.contiguous = false;
            }
            Ok(None) => {
                // No sense data within one poll; only a stream silent for a
                // whole STALL_TIMEOUT is dropped and reopened.
                if self.stall.as_mut().unwrap().silent(hw.now()) {
                    warn!("sense stream stalled for {} ms; reopening", STALL_TIMEOUT.as_millis());
                    self.stall = None;
                    self.fresh = 0;
                    self.contiguous = false;
                    hw.close_sense();
                }
            }
            Err(e) if transient(&e) => {
                warn!("sense capture: {e}; reopening");
                self.stall = None;
                self.fresh = 0;
                self.contiguous = false;
                hw.close_sense();
                self.maintain(hw)?;
                hw.sleep(PING_INTERVAL);
                return Ok(());
            }
            Err(e) => return Err(format!("sense capture: {e}").into()),
        }
        self.maintain(hw)
    }

    /// How long the next sense period is overdue (zero while none is due).
    fn overdue(&self, now: Instant) -> Duration {
        match (self.dt, self.seen_to) {
            (Some(dt), Some(seen)) => now.saturating_duration_since(seen + Duration::from_secs_f64(dt)),
            _ => Duration::ZERO,
        }
    }

    /// Charge the time since the model's account ends once nothing can
    /// cover it any more: the playback state is unknown, or the next period
    /// is overdue past the slack.  Returns how long the period is overdue.
    fn charge_blind<H: Hw>(&mut self, hw: &H) -> Result<Duration, Box<dyn Error>> {
        let now = hw.now();
        let overdue = self.overdue(now);
        if let Some(charged) = self.charged_to {
            if (self.unknown || overdue > BLIND_SLACK) && now > charged {
                self.charge_worst(charged, now)?;
                self.charged_to = Some(now);
            }
        }
        Ok(overdue)
    }

    /// Write the model state for the next run (atomically).  True on success.
    fn save_now<H: Hw>(&mut self, hw: &H, held: bool) -> bool {
        let Some(path) = &self.state_path else { return true };
        let now = hw.now();
        self.last_save_try = Some(now);
        // the model holds the speakers' power up to here, not up to now
        let covered = self.charged_to.unwrap_or(self.accounted_idle).min(now);
        let boot = hw.boottime();
        let text = state::encode(boot, boot.saturating_sub(now - covered), &self.speakers, held);
        let tmp = path.with_extension("tmp");
        let result = std::fs::write(&tmp, text).and_then(|()| std::fs::rename(&tmp, path));
        match result {
            Ok(()) => {
                self.save_failed = false;
                self.last_save = Some(now);
                true
            }
            Err(e) => {
                if !self.save_failed {
                    warn!("{}: {e}; the model state is not saved", path.display());
                    self.save_failed = true;
                }
                false
            }
        }
    }

    /// Whether the card may be playing above its safe level at this run's
    /// gain: the lock is held, or some of its lease is left while the
    /// controls are above the safe level.
    fn may_hold<H: Hw>(&self, _hw: &H) -> bool {
        !self.released(_hw)
    }

    /// How far the model's account trails the clock.
    fn account_lag(&self, now: Instant) -> Duration {
        now.saturating_duration_since(self.charged_to.unwrap_or(self.accounted_idle))
    }

    /// The model is current and its state saved recently enough to lift
    /// or renew the lock now (checked again after anything that may block).
    fn fresh_enough(&self, now: Instant) -> bool {
        let saved = self.state_path.is_none() || self.last_save.is_some_and(|t| now.duration_since(t) <= SAVE_MAX_AGE);
        let allowed = Duration::from_secs_f64(self.dt.unwrap_or(0.0)) + BLIND_SLACK;
        saved && self.account_lag(now) <= allowed
    }

    /// Charge unmeasured playback time, decide whether the daemon holds the
    /// card, and if it does write the model's reduction and renew the lock.
    fn maintain<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        let blind = self.charge_blind(hw)?;
        let now = hw.now();
        let reduction = model::reduction_db(&self.speakers);
        if !reduction.is_finite() {
            return Err("nonfinite speaker reduction".into());
        }
        let hard = self.speakers.iter().any(|s| s.hard);
        if self.armed && now >= self.last_ping + KERNEL_LOCK_TIMEOUT {
            // the daemon was late and the card may have taken its volume
            // back: take it again the safe way (controls to their minimum,
            // then unlock)
            self.disarm(hw, "the card's lock lapsed while the daemon was away")?;
        }
        let stale = self.state_path.is_some() && self.last_save.is_none_or(|t| now.duration_since(t) > SAVE_MAX_AGE);
        if self.armed {
            let why = if blind > BLIND_LIMIT {
                Some(format!("no complete sense data for {} ms", blind.as_millis()))
            } else if hard {
                Some("a coil may have reached its limit".to_string())
            } else if reduction >= self.max_reduction {
                Some(format!("the model asks for {reduction:.1} dB"))
            } else if stale {
                Some("the model state could not be saved".to_string())
            } else {
                None
            };
            if let Some(why) = why {
                self.disarm(hw, &why)?;
            }
        } else if !self.stopping && !self.unknown && !hard {
            let seeing = self.dt.is_none() || (self.fresh >= REARM_PERIODS && blind <= BLIND_SLACK);
            // the state a restart would resume from says the card is held
            // before it is
            if seeing && reduction < self.max_reduction - REARM_MARGIN_DB && self.save_now(hw, true) {
                match hw.set_min() {
                    Ok(()) => {
                        self.written = MUTE_DB;
                        self.note_held(hw.now());
                        self.min_failures = 0;
                        // the minimum write may have blocked: the saved state
                        // and the model must still be current at the unlock
                        if self.fresh_enough(hw.now()) {
                            self.ping_lock(hw)?;
                            info!("taking the volume lock{} at {reduction:.1} dB", if self.held { " back" } else { "" });
                            self.armed = true;
                            self.note_held(hw.now());
                            self.held = true;
                            self.applied = reduction;
                        } else {
                            debug!("not taking the lock: the model or its saved state fell behind");
                        }
                    }
                    Err(e) if transient(&e) && self.min_failures < MAX_CONTROL_FAILURES => {
                        self.min_failures += 1;
                        warn!("volume control: {e}; not taking the lock ({})", self.min_failures);
                    }
                    Err(e) => return Err(format!("volume control: {e}").into()),
                }
            }
        }
        if self.armed {
            if (reduction - self.applied).abs() >= 0.5 {
                info!("gain reduction {reduction:.1} dB");
                self.applied = reduction;
            }
            match hw.set_reduction(reduction) {
                Ok(()) => {
                    self.written = reduction;
                    self.note_held(hw.now());
                    self.target_failures = 0;
                }
                Err(e) if transient(&e) && self.target_failures < MAX_CONTROL_FAILURES => {
                    self.target_failures += 1;
                    warn!("volume control: {e} ({})", self.target_failures);
                    // some controls may hold the new value and some the old:
                    // the card applies at least the smaller reduction
                    if self.written.is_finite() {
                        self.written = self.written.min(reduction);
                    }
                    self.note_held(hw.now());
                    self.disarm(hw, "the volume control failed")?;
                }
                Err(e) => return Err(format!("volume control: {e}").into()),
            }
            if self.armed && hw.now().duration_since(self.last_ping) >= PING_INTERVAL {
                // the write may have blocked: renew only on a current state
                if self.fresh_enough(hw.now()) {
                    self.ping_lock(hw)?;
                } else {
                    self.disarm(hw, "the model or its saved state fell behind")?;
                }
            }
        }
        // a cleared fault only takes effect from the next pass
        for s in self.speakers.iter_mut() {
            if s.clear_hard_if_cool() {
                info!("{}: coil back under its working limit", s.spec.name);
            }
        }
        if hw.now().duration_since(self.last_log) >= LOG_INTERVAL {
            for s in &self.speakers {
                debug!(
                    "{}: {:.3} Vrms {:.1} mW coil {:.1} C magnet {:.1} C reduction {:.1} dB{}",
                    s.spec.name,
                    s.v_rms,
                    s.power * 1000.0,
                    s.t_coil,
                    s.t_magnet,
                    s.reduction_db,
                    if self.armed { "" } else { " (card at its safe level)" }
                );
            }
            self.last_log = hw.now();
        }
        if self.last_save_try.is_none_or(|t| hw.now().duration_since(t) >= SAVE_INTERVAL) {
            let held = self.may_hold(hw);
            self.save_now(hw, held);
        }
        Ok(())
    }

    /// Write the model state for the next run (best effort).
    pub fn save<H: Hw>(&mut self, hw: &H) {
        let held = self.may_hold(hw);
        self.save_now(hw, held);
    }
}

#[cfg(test)]
pub mod fake {
    //! A simulated card: a virtual clock, the macaudio volume-lock rules
    //! (the lock lapses 250 ms after the last ping; the next ping answers
    //! ETIMEDOUT once; volume writes above the safe level fail while
    //! lapsed) and a sense stream that produces one period every
    //! period/rate seconds, carrying the program after the card's gain.
    //! With `dma` set the sense stream is instead the simulated kernel PCM
    //! of alsa::sim, read through the daemon's own PeriodReader.
    use super::*;
    use crate::alsa::sim::SimPcm;
    use crate::alsa::PeriodReader;
    use std::rc::Rc;

    pub const LOCK_TIMEOUT: Duration = Duration::from_millis(250);
    /// How long after a sample-rate change its event wakes the daemon.
    pub const EVENT_LATENCY: Duration = Duration::from_micros(200);
    /// The kernel's safe level without a daemon (macaudio safe_vol).
    pub const SAFE_DB: f64 = 20.0;

    pub struct FakeHw {
        pub t0: Instant,
        pub t: Duration,
        pub period: Duration,
        pub channels: usize,
        /// Speakers open (rate 48000) at time t?
        pub playing: Box<dyn Fn(Duration) -> bool>,
        /// Sense data flowing at time t?
        pub sensing: Box<dyn Fn(Duration) -> bool>,
        /// Program level at time t, as a fraction of full scale (square wave).
        pub level: Box<dyn Fn(Duration) -> f64>,
        pub sense_open: bool,
        next_period: Duration,
        pub opens: u32,
        pub pings: Vec<Duration>,
        last_ok_ping: Option<Duration>,
        notice_pending: bool,
        /// The kernel's lease, in playing time (macaudio pauses it while
        /// the speakers are idle), and where it was last brought up to date.
        lease_left: Duration,
        lease_t: Duration,
        /// Writes of the governed reduction (not the minimum) fail (EIO).
        pub target_fail: Box<dyn Fn(Duration) -> bool>,
        /// How long a minimum write blocks.
        pub min_delay: Duration,
        pub relinquished: u32,
        pub lapses: Vec<Duration>,
        /// Reduction (dB) last accepted by the card.
        pub reduction: f64,
        pub writes: Vec<(Duration, f64)>,
        /// CLOCK_BOOTTIME at t = 0.
        pub boot: Duration,
        /// Extra time a sample-rate read takes at time t (a descheduled
        /// daemon, a slow ioctl).
        pub control_delay: Box<dyn Fn(Duration) -> Duration>,
        /// Sense through the simulated kernel PCM, whose DMA delivers this
        /// fraction of its rate at time t (it runs only while that is > 0).
        pub dma: Option<Rc<dyn Fn(Duration) -> f64>>,
        /// The simulated DMA pointer's granularity in frames.
        pub granule: usize,
        period_frames: usize,
        pcm: Option<PeriodReader<SimPcm>>,
        /// When complete and incomplete sense periods reached the daemon.
        pub complete: Vec<Duration>,
        pub incomplete: Vec<Duration>,
        /// The sample-rate control fails (EIO) at time t.
        pub rate_fail: Box<dyn Fn(Duration) -> bool>,
        /// Volume writes fail (EIO) at time t, the card keeping its value.
        pub volume_fail: Box<dyn Fn(Duration) -> bool>,
        /// A late read finds only the newest period: the frames before it
        /// are lost, not queued (no overrun is reported).
        pub lossy: bool,
        /// Where the period last returned ended.
        last_end: Duration,
        pub locked: bool,
        /// The card sends sample-rate change events (else the daemon polls).
        pub events: bool,
        /// How long opening the sense PCM blocks.
        pub open_delay: Duration,
        /// Where rate_events last looked.
        events_seen_to: Duration,
    }

    impl FakeHw {
        pub fn new(channels: usize, period_frames: usize) -> FakeHw {
            FakeHw {
                t0: Instant::now(),
                t: Duration::ZERO,
                period: Duration::from_secs_f64(period_frames as f64 / 48000.0),
                channels,
                playing: Box::new(|_| true),
                sensing: Box::new(|_| true),
                level: Box::new(|_| 0.1),
                sense_open: false,
                next_period: Duration::ZERO,
                opens: 0,
                pings: Vec::new(),
                last_ok_ping: None,
                notice_pending: false,
                lease_left: Duration::ZERO,
                lease_t: Duration::ZERO,
                target_fail: Box::new(|_| false),
                min_delay: Duration::ZERO,
                relinquished: 0,
                lapses: Vec::new(),
                reduction: SAFE_DB,
                writes: Vec::new(),
                boot: Duration::from_secs(1000),
                control_delay: Box::new(|_| Duration::ZERO),
                dma: None,
                granule: 32,
                period_frames,
                pcm: None,
                complete: Vec::new(),
                incomplete: Vec::new(),
                rate_fail: Box::new(|_| false),
                volume_fail: Box::new(|_| false),
                lossy: false,
                last_end: Duration::ZERO,
                locked: false,
                events: true,
                events_seen_to: Duration::ZERO,
                open_delay: Duration::ZERO,
            }
        }

        /// The first time in (from, to] the speakers open or close.
        pub fn next_change(&self, from: Duration, to: Duration) -> Option<Duration> {
            let was = (self.playing)(from);
            let step = Duration::from_micros(250);
            let mut t = from + step;
            while t <= to {
                if (self.playing)(t) != was {
                    return Some(t);
                }
                t += step;
            }
            None
        }

        /// The first successful lock renewal.
        pub fn first_ping(&self) -> Option<Duration> {
            self.pings.first().copied()
        }

        /// Is the card's volume lock lifted (now; `_t` is the current time)?
        pub fn unlocked_at(&self, _t: Duration) -> bool {
            self.last_ok_ping.is_some() && !self.lease_left.is_zero() && !self.notice_pending
        }

        /// The gain the card applies: the daemon's reduction while the lock
        /// is held, never less than the safe level otherwise.
        pub fn effective_reduction(&self) -> f64 {
            if self.unlocked_at(self.t) {
                self.reduction
            } else {
                self.reduction.max(SAFE_DB)
            }
        }

        /// Run the lease down by the playing time since it was last looked
        /// at; at zero the card locks the volume (macaudio's timeout work).
        fn lapse_check(&mut self) {
            let step = Duration::from_micros(500);
            let mut a = self.lease_t;
            while a < self.t && !self.lease_left.is_zero() {
                let b = (a + step).min(self.t);
                if (self.playing)(a) {
                    self.lease_left = self.lease_left.saturating_sub(b - a);
                    if self.lease_left.is_zero() && self.last_ok_ping.is_some() && !self.notice_pending {
                        self.lapses.push(b);
                        self.notice_pending = true;
                    }
                }
                a = b;
            }
            self.lease_t = self.t;
        }

        /// Largest gap between consecutive successful lock renewals in [from, to).
        pub fn max_ping_gap(&self, from: Duration, to: Duration) -> Duration {
            let mut last = from;
            let mut gap = Duration::ZERO;
            for &p in self.pings.iter().filter(|&&p| p >= from && p < to) {
                gap = gap.max(p - last);
                last = p;
            }
            gap.max(to.saturating_sub(last))
        }
    }

    impl Hw for FakeHw {
        fn now(&self) -> Instant {
            self.t0 + self.t
        }
        fn boottime(&self) -> Duration {
            self.boot + self.t
        }
        fn sleep(&mut self, d: Duration) {
            self.t += d;
            self.lapse_check();
        }
        fn take_lock(&mut self) -> io::Result<()> {
            self.locked = true;
            Ok(())
        }
        fn ping(&mut self) -> io::Result<()> {
            self.lapse_check();
            if self.notice_pending {
                // the one-shot notice; the timer is not re-armed
                self.notice_pending = false;
                self.last_ok_ping = None;
                return Err(io::Error::from_raw_os_error(libc::ETIMEDOUT));
            }
            self.last_ok_ping = Some(self.t);
            self.lease_left = LOCK_TIMEOUT;
            self.pings.push(self.t);
            Ok(())
        }
        fn set_min(&mut self) -> io::Result<()> {
            self.t += self.min_delay;
            self.lapse_check();
            if (self.volume_fail)(self.t) {
                return Err(io::Error::from_raw_os_error(libc::EIO));
            }
            self.reduction = MUTE_DB;
            Ok(())
        }
        fn relinquish(&mut self) -> io::Result<()> {
            self.lapse_check();
            // macaudio_slk_unlock then _lock: locked, no lease, no notice
            self.lease_left = Duration::ZERO;
            self.last_ok_ping = None;
            self.notice_pending = false;
            self.relinquished += 1;
            Ok(())
        }
        fn set_reduction(&mut self, db: f64) -> io::Result<()> {
            self.lapse_check();
            if (self.volume_fail)(self.t) || (self.target_fail)(self.t) {
                return Err(io::Error::from_raw_os_error(libc::EIO));
            }
            if !self.unlocked_at(self.t) && db < SAFE_DB {
                return Err(io::Error::from_raw_os_error(libc::EINVAL));
            }
            self.reduction = db;
            self.writes.push((self.t, db));
            Ok(())
        }
        fn sample_rate(&mut self) -> io::Result<i64> {
            self.t += (self.control_delay)(self.t);
            self.lapse_check();
            if (self.rate_fail)(self.t) {
                return Err(io::Error::from_raw_os_error(libc::EIO));
            }
            Ok(if (self.playing)(self.t) { 48000 } else { 0 })
        }
        fn queued(&mut self) -> io::Result<u64> {
            if let Some(reader) = self.pcm.as_mut() {
                return reader.queued();
            }
            Ok((self.t.saturating_sub(self.last_end).as_secs_f64() * 48000.0) as u64)
        }
        fn read_tail(&mut self, words: &mut [i32]) -> io::Result<usize> {
            if !self.sense_open {
                return Ok(0);
            }
            let gain = 10f64.powf(-self.effective_reduction() / 20.0);
            let a = ((self.level)(self.t) * gain).clamp(0.0, 1.0);
            let word = (a * 2147483647.0) as i32;
            if let Some(reader) = self.pcm.as_mut() {
                let t = self.t;
                reader.io().advance_to(t);
                reader.io().word = word;
                return reader.read_tail(words);
            }
            // the frames since the last whole period, up to where the
            // stream stopped (or now)
            let start = self.next_period.saturating_sub(self.period);
            let stop = self.next_change(start, self.t).unwrap_or(self.t).min(self.t);
            if !(self.sensing)(start) {
                return Ok(0);
            }
            let k = ((stop.saturating_sub(start).as_secs_f64() * 48000.0) as usize).min(self.period_frames - 1);
            for (i, w) in words[..k * self.channels].iter_mut().enumerate() {
                let frame = i / self.channels;
                *w = if frame % 2 == 0 { word } else { -word };
            }
            self.last_end = self.t;
            Ok(k)
        }
        fn wait_rate_change(&mut self, timeout: Duration) -> bool {
            if self.events {
                if let Some(c) = self.next_change(self.t, self.t + timeout) {
                    self.t = c + EVENT_LATENCY;
                    self.lapse_check();
                    return true;
                }
            }
            self.t += timeout;
            self.lapse_check();
            false
        }
        fn rate_events(&mut self) -> bool {
            let changed = self.events && self.next_change(self.events_seen_to, self.t).is_some();
            self.events_seen_to = self.t;
            changed
        }
        fn open_sense(&mut self, _rate: u32) -> io::Result<()> {
            self.t += self.open_delay;
            self.lapse_check();
            self.sense_open = true;
            self.opens += 1;
            self.next_period = self.t + self.period;
            if let Some(flow) = &self.dma {
                let pcm = SimPcm::new(48000, self.channels, self.period_frames, self.granule, flow.clone(), self.t);
                self.pcm = Some(PeriodReader::new(pcm, self.channels, self.period_frames));
            }
            Ok(())
        }
        fn close_sense(&mut self) {
            self.sense_open = false;
            self.pcm = None;
        }
        fn read_period(&mut self, words: &mut [i32]) -> io::Result<Option<bool>> {
            assert!(self.sense_open, "read on a closed capture");
            // a sample-rate change ends the wait early, as the card's event does
            let change = if self.events { self.next_change(self.t, self.t + Duration::from_millis(50)) } else { None };
            if self.pcm.is_some() {
                if let Some(c) = change {
                    let ms = (c.saturating_sub(self.t).as_micros().div_ceil(1000)).max(1) as libc::c_int;
                    let gain = 10f64.powf(-self.effective_reduction() / 20.0);
                    let a = ((self.level)(self.t) * gain).clamp(0.0, 1.0);
                    let t = self.t;
                    let reader = self.pcm.as_mut().unwrap();
                    reader.io().advance_to(t);
                    reader.io().word = (a * 2147483647.0) as i32;
                    let read = reader.read_period_within(words, ms);
                    self.t = reader.io().t.max(if read.as_ref().is_ok_and(|r| r.is_none()) { c + EVENT_LATENCY } else { Duration::ZERO });
                    self.lapse_check();
                    match read {
                        Ok(Some(true)) => self.complete.push(self.t),
                        Ok(Some(false)) => self.incomplete.push(self.t),
                        _ => {}
                    }
                    return read;
                }
            }
            if self.pcm.is_none() {
                if let Some(c) = change {
                    if c < self.next_period || !(self.sensing)(self.t) {
                        self.t = c + EVENT_LATENCY;
                        self.lapse_check();
                        return Ok(None);
                    }
                }
            }
            if self.pcm.is_some() {
                let gain = 10f64.powf(-self.effective_reduction() / 20.0);
                let a = ((self.level)(self.t) * gain).clamp(0.0, 1.0);
                let t = self.t;
                let reader = self.pcm.as_mut().unwrap();
                reader.io().advance_to(t);
                reader.io().word = (a * 2147483647.0) as i32;
                let read = reader.read_period(words);
                self.t = reader.io().t;
                self.lapse_check();
                match read {
                    Ok(Some(true)) => self.complete.push(self.t),
                    Ok(Some(false)) => self.incomplete.push(self.t),
                    _ => {}
                }
                return read;
            }
            let deadline = self.t + Duration::from_millis(50);
            if !(self.sensing)(self.t) {
                // the stream is stuck: nothing will come, periods are lost
                self.t = deadline;
                self.next_period = self.t + self.period;
                self.lapse_check();
                return Ok(None);
            }
            if self.next_period > deadline {
                self.t = deadline;
                self.lapse_check();
                return Ok(None);
            }
            if self.lossy && self.t > self.next_period + self.period {
                // everything but the period that just completed is gone
                self.next_period = self.t;
            }
            self.t = self.t.max(self.next_period);
            self.last_end = self.next_period;
            self.next_period += self.period;
            self.lapse_check();
            let gain = 10f64.powf(-self.effective_reduction() / 20.0);
            let a = ((self.level)(self.t) * gain).clamp(0.0, 1.0);
            let word = (a * 2147483647.0) as i32;
            for (i, w) in words.iter_mut().enumerate() {
                let frame = i / self.channels;
                *w = if frame % 2 == 0 { word } else { -word };
            }
            Ok(Some(true))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fake::*;
    use super::*;

    const POLL: Duration = Duration::from_millis(50);
    const J700: &str = include_str!("../conf/apple/j700.conf");

    #[test]
    fn a_silent_stream_stalls_after_the_timeout() {
        let t0 = Instant::now();
        let mut clock = StallClock::default();
        let mut t = t0;
        while t < t0 + STALL_TIMEOUT {
            assert!(!clock.silent(t), "stalled after {:?}", t - t0);
            t += POLL;
        }
        assert!(clock.silent(t0 + STALL_TIMEOUT));
    }

    #[test]
    fn data_restarts_the_clock() {
        let t0 = Instant::now();
        let mut clock = StallClock::default();
        assert!(!clock.silent(t0));
        assert!(!clock.silent(t0 + STALL_TIMEOUT - POLL));
        clock.data();
        let t1 = t0 + STALL_TIMEOUT + POLL;
        assert!(!clock.silent(t1));
        assert!(!clock.silent(t1 + STALL_TIMEOUT - POLL));
        assert!(clock.silent(t1 + STALL_TIMEOUT));
    }

    /// The sequence the J700 showed: a stream closed during an empty wait,
    /// and the next one opened minutes later.  Its first empty wait comes
    /// 50 ms after the open, before the first period (85 ms at 48 kHz).
    #[test]
    fn a_new_capture_gets_its_whole_timeout() {
        let t0 = Instant::now();
        let mut old = StallClock::default();
        assert!(!old.silent(t0));

        let open = t0 + Duration::from_secs(360);
        let mut clock = StallClock::default();
        assert!(!clock.silent(open + POLL));
        assert!(!clock.silent(open + STALL_TIMEOUT));
        assert!(clock.silent(open + POLL + STALL_TIMEOUT));
    }

    pub fn j700() -> Config {
        Config::parse(J700).unwrap()
    }

    /// A state file path of its own, not there yet.
    pub fn missing_state() -> Option<PathBuf> {
        use std::sync::atomic::{AtomicU32, Ordering};
        static N: AtomicU32 = AtomicU32::new(0);
        let dir = std::env::temp_dir().join(format!("speakerguardd-test-{}-{}", std::process::id(), N.fetch_add(1, Ordering::Relaxed)));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        Some(dir.join("state"))
    }

    /// A state file of its own saying the speakers are cold and the card
    /// released, current at the fake's start (boot time 1000 s).
    pub fn tmp_state() -> Option<PathBuf> {
        let path = missing_state().unwrap();
        let cfg = j700();
        let cold: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), cfg.globals.t_ambient)).collect();
        let boot = Duration::from_secs(1000);
        std::fs::write(&path, state::encode(boot, boot, &cold, false)).unwrap();
        Some(path)
    }

    /// Run the loop until `until`, returning the first error.
    pub fn run_until(g: &mut Guard, hw: &mut FakeHw, until: Duration) -> Result<(), String> {
        while hw.t < until {
            g.tick(hw).map_err(|e| format!("at {:?}: {e}", hw.t))?;
        }
        Ok(())
    }

    #[test]
    fn healthy_playback_keeps_the_lock_and_full_volume() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, Duration::from_secs(10)).unwrap();
        assert!(hw.lapses.is_empty(), "{:?}", hw.lapses);
        assert!(hw.max_ping_gap(Duration::ZERO, hw.t) < LOCK_TIMEOUT);
        assert_eq!(hw.reduction, 0.0);
        assert_eq!(hw.opens, 1);
    }

    #[test]
    fn idle_keeps_the_lock() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.playing = Box::new(|t| t < Duration::from_secs(2));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, Duration::from_secs(5)).unwrap();
        assert!(hw.lapses.is_empty(), "{:?}", hw.lapses);
        assert!(!hw.sense_open);
    }

    // ---- regressions: speakerguardd PR #2 (sense gaps were fatal) ----

    fn secs(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    /// A 300 ms gap in the sense stream (stream start latency, an xrun, a
    /// PipeWire reconfiguration) must neither end the daemon nor drop the
    /// card to its safe level: the gap is shorter than the stall timeout and
    /// is covered by the model's worst-case accounting.
    #[test]
    fn a_300ms_sense_gap_does_not_kill_the_daemon() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.sensing = Box::new(|t| !(secs(2.0)..secs(2.3)).contains(&t));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(8.0)).expect("the daemon must survive a short sense gap");
        assert!(hw.lapses.is_empty(), "the card clamped at {:?}", hw.lapses);
        assert!(hw.max_ping_gap(Duration::ZERO, hw.t) < LOCK_TIMEOUT);
        assert_eq!(hw.reduction, 0.0, "full volume is restored after the gap");
    }

    /// A sense outage longer than the stall timeout hands the card to the
    /// kernel's safe level (the lock lapses), the capture is reopened, and the
    /// lock is taken back once complete periods flow again, all without
    /// exiting: the PR #2 head exited at 250 ms and, with Restart=no, left
    /// the speakers 20 dB down until the next boot.
    #[test]
    fn a_long_sense_outage_clamps_through_the_kernel_then_recovers() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.sensing = Box::new(|t| !(secs(2.0)..secs(4.0)).contains(&t));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(9.0)).expect("a sense outage must not end the daemon");
        assert_eq!(hw.lapses.len(), 1, "{:?}", hw.lapses);
        let lapse = hw.lapses[0];
        assert!(lapse > secs(2.0) && lapse < secs(3.0), "clamped at {lapse:?}");
        assert!(
            !hw.pings.iter().any(|&p| p > lapse && p < secs(4.0)),
            "the lock stays lapsed while the daemon is blind"
        );
        assert!(hw.opens >= 2, "the stalled capture was reopened");
        assert!(hw.unlocked_at(hw.t), "the lock is taken back after the outage");
        assert_eq!(hw.reduction, 0.0);
    }

    /// When the model asks for more than --max-reduction the card is handed
    /// to the kernel's safe level, but the daemon keeps modelling (the sense
    /// tap shows the clamped wire) and takes the lock back once the speaker
    /// has cooled, instead of exiting for good.
    #[test]
    fn the_reduction_limit_clamps_instead_of_exiting_and_recovers() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 1.0); // a full-scale square wave: 12.7 W
        hw.playing = Box::new(|t| t < secs(8.0));
        let mut g = Guard::start(&mut hw, &cfg, 6.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(60.0)).expect("the reduction limit must not end the daemon");
        assert!(!hw.lapses.is_empty(), "the limit handed the card to the kernel");
        assert!(hw.unlocked_at(hw.t), "the lock is taken back once cool");
        assert!(hw.reduction < 6.0 - 3.0 + 1e-9, "{}", hw.reduction);
    }

    /// One complete period after an outage is not enough to take the lock
    /// back: a stream that delivers a period and dies again stays clamped.
    #[test]
    fn a_single_period_after_an_outage_does_not_take_the_lock_back() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        // one period's worth of data in the middle of a long outage
        hw.sensing = Box::new(|t| t < secs(2.0) || (secs(3.4)..secs(3.48)).contains(&t) || t >= secs(6.0));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(5.9)).unwrap();
        assert_eq!(hw.lapses.len(), 1, "{:?}", hw.lapses);
        assert!(!g.armed(), "a lone period re-armed the lock");
        assert!(!hw.pings.iter().any(|&p| p > hw.lapses[0] && p < secs(5.9)));
        run_until(&mut g, &mut hw, secs(8.0)).unwrap();
        assert!(g.armed() && hw.unlocked_at(hw.t));
    }

    /// Time the daemon could not measure is charged at the worst case: the
    /// same program heats the model more with a gap in the sense data than
    /// without one.
    #[test]
    fn blind_time_is_charged_at_the_worst_case_power() {
        let cfg = j700();
        let coil_after = |gap: bool| {
            let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
            hw.level = Box::new(|_| 0.2);
            if gap {
                hw.sensing = Box::new(|t| !(secs(1.0)..secs(1.4)).contains(&t));
            }
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            run_until(&mut g, &mut hw, secs(1.6)).unwrap();
            g.speakers[0].t_coil
        };
        let (measured, blind) = (coil_after(false), coil_after(true));
        // 0.2 FS is 0.51 W; a 400 ms gap at 0 dB is charged as 12.7 W
        assert!(blind > measured + 20.0, "blind {blind:.1} C vs measured {measured:.1} C");
    }

    /// A late daemon (a slow control read, a descheduled process) lets the
    /// card's lock lapse; the next ping's ETIMEDOUT notice is answered with a
    /// second ping and the lock is taken back without exiting.
    #[test]
    fn a_lapse_from_a_late_daemon_is_taken_back() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.control_delay = Box::new(|t| if (secs(3.0)..secs(3.1)).contains(&t) { Duration::from_millis(400) } else { Duration::ZERO });
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(6.0)).expect("a late daemon must recover the lock");
        assert_eq!(hw.lapses.len(), 1, "{:?}", hw.lapses);
        assert!(g.armed() && hw.unlocked_at(hw.t));
        assert_eq!(hw.reduction, 0.0);
    }

    /// "Peak volume": with a cool speaker and a loud but sustainable program
    /// (0.3 of full scale, 1.1 W into each coil) the daemon leaves the card
    /// at the top of its range, the amplifier's full scale, for the whole
    /// burst; the model only takes gain once the power budget or the coil
    /// demands it.
    #[test]
    fn a_cool_speaker_gets_the_full_scale() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.3);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(10.0)).unwrap();
        assert!(hw.writes.iter().all(|&(_, db)| db == 0.0), "{:?}", hw.writes.iter().find(|w| w.1 != 0.0));
        assert!(hw.lapses.is_empty());
    }

    /// The power budgets must hold the mean power the speakers actually get
    /// at or under the budget.  A budget reduction computed from the power
    /// after the card's gain only asks for the excess left over the
    /// reduction already applied, so in steady state it settles at about
    /// half of the dB the budget needs (6.3 W against 2.8154 W: 1.75 dB
    /// instead of 3.5 dB, 4.2 W through the speakers instead of 2.8 W).
    /// The thermal model is made unreachable here so only the budgets act.
    #[test]
    fn the_power_budgets_hold_the_mean_power_at_the_budget() {
        let base = J700.replace("t_limit = 135.0", "t_limit = 2000.0");
        for (one_s, sixty_s, watts, settle) in [
            (true, false, 3.5, 30.0),
            (true, false, 4.5, 30.0),
            (true, false, 6.3, 30.0),
            (false, true, 2.6, 240.0),
            (false, true, 3.5, 240.0),
            (false, true, 6.3, 240.0),
        ] {
            let mut text = base.clone();
            if !one_s {
                text = text.replace("p_limit_1s = 2.8154", "p_limit_1s = 0");
            }
            if !sixty_s {
                text = text.replace("p_limit_60s = 2.5", "p_limit_60s = 0");
            }
            let cfg = Config::parse(&text).unwrap();
            let limit = if one_s { 2.8154 } else { 2.5 };
            // a square wave at `level` of full scale: (level * vs_scale)^2 / z
            // watts before the card's gain (the left speaker: 7.3 V, 4.2 ohm)
            let level = (watts * 4.2f64).sqrt() / 7.3;
            let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
            hw.level = Box::new(move |_| level);
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            run_until(&mut g, &mut hw, secs(settle)).unwrap();
            let want = 10.0 * (watts / limit).log10();
            // the reduction over the last 10 s, and the power it lets through
            let tail: Vec<f64> = hw.writes.iter().filter(|w| w.0 >= secs(settle - 10.0)).map(|w| w.1).collect();
            let mean_db = tail.iter().sum::<f64>() / tail.len() as f64;
            let through = watts * 10f64.powf(-mean_db / 10.0);
            assert!(
                (mean_db - want).abs() < 0.3 && through <= limit * 1.08,
                "{} s budget {limit} W at {watts} W: {mean_db:.2} dB (want {want:.2} dB), {through:.2} W through",
                if one_s { 1 } else { 60 }
            );
        }
    }

    /// A restarted daemon must not start cold: with the state file of a hot
    /// run it governs from that run's heat, not from full volume.
    #[test]
    fn a_restart_starts_from_the_saved_model() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.6); // 4.6 W: budget and coil both govern
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path.clone())).unwrap();
        run_until(&mut g, &mut hw, secs(20.0)).unwrap();
        let before = model::reduction_db(&g.speakers);
        let coil = g.speakers[0].t_coil;
        assert!(before > 3.0, "{before}");
        g.save(&hw);
        drop(g);
        // the next run, 0.5 s later (systemd's restart delay), after a
        // crash: the old run held the card, so the gap is charged at full gain
        let mut hw2 = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw2.boot = hw.boot + hw.t + Duration::from_millis(500);
        hw2.level = Box::new(|_| 0.6);
        let mut g2 = Guard::start(&mut hw2, &cfg, 20.0, Some(path.clone())).unwrap();
        assert!(g2.speakers[0].t_coil >= coil, "{} vs {coil}", g2.speakers[0].t_coil);
        assert!(hw2.pings.is_empty(), "no unlock at start");
        run_until(&mut g2, &mut hw2, secs(1.0)).unwrap();
        // the first gain it allows is not full volume
        if let Some(&(_, after)) = hw2.writes.first() {
            assert!(after > 0.5, "restarted at {after:.1} dB after {before:.1} dB");
        }
        drop(g2);
        // whereas the first run of a boot starts from the card's safe level
        let mut cold = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut c = Guard::start(&mut cold, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut c, &mut cold, secs(0.5)).unwrap();
        assert_eq!(cold.writes.first().map(|w| w.1), Some(0.0));
        // an unreadable file starts hot, not cold
        std::fs::write(&path, "garbage").unwrap();
        let mut hw3 = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g3 = Guard::start(&mut hw3, &cfg, 20.0, Some(path.clone())).unwrap();
        run_until(&mut g3, &mut hw3, secs(6.0)).unwrap();
        // (hot: 12 dB, released by at most 0.5 dB per period before it arms)
        assert!(hw3.writes.first().map(|w| w.1).unwrap() >= cfg.globals.t_reduction_max - 2.0);
    }

    // ---- the J700 sense PCM: a DMA-precise pointer behind a non-blocking read ----

    fn dma_hw(cfg: &Config, flow: impl Fn(Duration) -> f64 + 'static) -> FakeHw {
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.dma = Some(std::rc::Rc::new(flow));
        hw
    }

    /// Continuous playback through a capture whose pointer moves in 32-frame
    /// steps: every period arrives whole and the lock is never handed over.
    /// The PR #2 head read before it waited, got a short transfer every
    /// 50 ms and handed the card to the kernel 515 ms into the first sound.
    #[test]
    fn a_fine_grained_sense_pointer_gives_complete_periods() {
        let cfg = j700();
        let mut hw = dma_hw(&cfg, |_| 1.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(10.0)).unwrap();
        assert!(hw.incomplete.is_empty(), "{} incomplete, first at {:?}", hw.incomplete.len(), hw.incomplete.first());
        assert!(hw.complete.len() >= 115, "{} periods", hw.complete.len());
        assert!(hw.lapses.is_empty(), "{:?}", hw.lapses);
        assert_eq!(hw.opens, 1);
        assert!(g.armed());
        assert_eq!(hw.reduction, 0.0);
    }

    /// The 2026-10-19 sequence: volume keys pressed in quick succession play
    /// a short feedback sound per step, so the speakers open and close
    /// faster than a sense period, then music plays.  Complete sense data
    /// and the daemon's own gain control must follow within a bounded time.
    #[test]
    fn a_burst_of_short_streams_resumes_complete_sense_data() {
        let cfg = j700();
        // 20 sounds of 60 ms, 100 ms apart, then 10 s of playback
        let playing = |t: Duration| {
            let t = t.as_secs_f64();
            if (1.0..4.2).contains(&t) {
                (t - 1.0) % 0.16 < 0.06
            } else {
                (4.2..14.2).contains(&t)
            }
        };
        let mut hw = dma_hw(&cfg, move |t| if playing(t) { 1.0 } else { 0.0 });
        hw.playing = Box::new(playing);
        hw.level = Box::new(|_| 0.1);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(14.2)).unwrap();
        assert!(hw.opens >= 20, "{} opens", hw.opens);
        let first = hw.complete.iter().copied().find(|&t| t >= secs(4.2)).expect("no complete period after the burst");
        assert!(first < secs(4.2 + 0.2), "first complete period at {first:?}");
        let after: Vec<_> = hw.incomplete.iter().filter(|&&t| t >= secs(4.2)).collect();
        assert!(after.is_empty(), "incomplete periods after the burst: {after:?}");
        let n = hw.complete.iter().filter(|&&t| t >= secs(4.2)).count();
        assert!(n >= 115, "{n} complete periods in 10 s");
        assert!(hw.lapses.is_empty(), "the card was clamped at {:?}", hw.lapses);
        assert!(g.armed() && hw.unlocked_at(hw.t));
        assert!(hw.max_ping_gap(secs(4.2), hw.t) < LOCK_TIMEOUT);
        assert_eq!(hw.reduction, 0.0, "the daemon governs, not the safe level");
    }

    /// The safety rule still holds on the real read path: a DMA that stops
    /// while the speakers play hands the card to the kernel within the blind
    /// limit and the lock stays lapsed until complete periods return.
    #[test]
    fn a_stopped_dma_still_hands_the_card_to_the_kernel() {
        let cfg = j700();
        let mut hw = dma_hw(&cfg, |t| if (secs(3.0)..secs(5.0)).contains(&t) { 0.0 } else { 1.0 });
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(8.0)).unwrap();
        assert_eq!(hw.lapses.len(), 1, "{:?}", hw.lapses);
        let lapse = hw.lapses[0];
        assert!(lapse > secs(3.0) && lapse < secs(3.0) + BLIND_LIMIT + LOCK_TIMEOUT + secs(0.2), "clamped at {lapse:?}");
        assert!(!hw.pings.iter().any(|&p| p > lapse && p < secs(5.0)), "the lock was renewed while blind");
        assert!(g.armed() && hw.unlocked_at(hw.t), "taken back after the outage");
    }

    /// A stream that trickles (a tenth of its rate) is not sense data: it
    /// never completes a period inside the stall timeout, so the card stays
    /// with the kernel until the stream is whole again.
    #[test]
    fn a_trickling_dma_never_renews_the_lock() {
        let cfg = j700();
        let mut hw = dma_hw(&cfg, |t| if (secs(2.0)..secs(8.0)).contains(&t) { 0.1 } else { 1.0 });
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(8.0)).unwrap();
        assert_eq!(hw.lapses.len(), 1, "{:?}", hw.lapses);
        let lapse = hw.lapses[0];
        // the period in flight when the trickle began still completes (its
        // frames are all real); the blind limit runs from that one
        let last = hw.complete.iter().copied().filter(|&t| t < lapse).last().unwrap();
        assert!(last < secs(2.0) + STALL_TIMEOUT, "a trickle completed a period at {last:?}");
        assert!(lapse < last + secs(0.0853) + BLIND_LIMIT + LOCK_TIMEOUT + PING_INTERVAL, "clamped at {lapse:?}, last period {last:?}");
        assert!(!g.armed());
        assert!(!hw.pings.iter().any(|&p| p > lapse && p < secs(8.0)), "the lock was renewed on a trickle");
        run_until(&mut g, &mut hw, secs(10.0)).unwrap();
        assert!(g.armed() && hw.unlocked_at(hw.t));
    }

    // ---- 0.1.3: the speakerguardd#6 review's witnesses that 0.1.2 shared ----

    /// Review F1, volume route: a volume control that fails must not leave
    /// the lock renewed at the older, louder gain.  0.1.2 counted the
    /// failure, kept pinging, and a good sample-rate read reset the count.
    #[test]
    fn a_failing_volume_control_hands_the_card_to_the_kernel() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.6); // 4.6 W: the budget asks for a reduction
        hw.volume_fail = Box::new(|t| (secs(2.0)..secs(8.0)).contains(&t));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(10.0)).unwrap();
        let renewed: Vec<_> = hw.pings.iter().filter(|&&p| p >= secs(2.0) && p < secs(8.0)).collect();
        assert!(renewed.is_empty(), "the lock was renewed while the volume control failed: {renewed:?}");
        // the hand-off's safe-level write fails too: the lock is given up
        // (macaudio locks the volume at once), or else it lapses
        let lapse = hw.lapses.iter().copied().find(|&l| l >= secs(2.0));
        assert!(hw.relinquished >= 1 || lapse.is_some_and(|l| l < secs(2.0) + LOCK_TIMEOUT), "neither given up nor lapsed: {lapse:?}");
        assert!(g.armed() && hw.unlocked_at(hw.t), "taken back once the control works");
        assert!(hw.reduction > 0.0, "the budget's reduction is applied after recovery");
    }

    /// Review F1, sample-rate route: with the rate control failing from
    /// the start the playback state is unknown.  0.1.2 kept unlocking (100
    /// pings in 10 s at full volume) and charged nothing.
    #[test]
    fn a_failing_rate_control_never_unlocks_and_charges_the_time() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.playing = Box::new(|_| false);
        hw.rate_fail = Box::new(|t| t < secs(10.0));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        let coil0 = g.speakers[0].t_coil;
        run_until(&mut g, &mut hw, secs(10.0)).unwrap();
        assert!(hw.pings.is_empty(), "unlocked {} times with the playback state unknown", hw.pings.len());
        assert!(!g.armed() && hw.opens == 0);
        // charged as playing at the card's safe level: 0.127 W heats the coil
        assert!(g.speakers[0].t_coil > coil0 + 1.0, "{} vs {coil0}", g.speakers[0].t_coil);
        // the control returns: closed is verified and the card is taken
        run_until(&mut g, &mut hw, secs(11.0)).unwrap();
        assert!(g.armed() && hw.unlocked_at(hw.t));
    }

    /// The same once the daemon held the card: the first failing read
    /// stops the renewal.
    #[test]
    fn a_rate_control_failing_mid_stream_stops_the_renewal() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.rate_fail = Box::new(|t| (secs(3.0)..secs(3.3)).contains(&t));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(6.0)).unwrap();
        assert!(!hw.pings.iter().any(|&p| (secs(3.0)..secs(3.3)).contains(&p)), "renewed while the rate control failed");
        assert_eq!(hw.lapses.len(), 1, "{:?}", hw.lapses);
        assert!(g.armed() && hw.unlocked_at(hw.t));
    }

    /// Review F2: a hot or damaged state must not unlock the card before
    /// the daemon can see.  0.1.2 pinged in Guard::start, before any
    /// sense period, and also when the start already needed a hand-off.
    #[test]
    fn a_hot_start_while_playing_waits_for_fresh_periods() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        std::fs::write(&path, "damaged").unwrap();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path)).unwrap();
        assert!(hw.pings.is_empty(), "unlocked at start");
        run_until(&mut g, &mut hw, secs(6.0)).unwrap();
        let first = hw.first_ping().expect("never armed");
        let second_period = secs(2.0 * cfg.globals.period as f64 / 48000.0);
        assert!(first + Duration::from_millis(1) >= second_period, "unlocked at {first:?}, before two complete periods");
        assert!(hw.writes.first().is_some_and(|w| w.1 >= cfg.globals.t_reduction_max - 2.0), "{:?}", hw.writes.first());
    }

    #[test]
    fn a_start_that_needs_the_kernel_never_unlocks() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        std::fs::write(&path, "damaged").unwrap(); // hot: 12 dB
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.playing = Box::new(|_| false);
        let mut g = Guard::start(&mut hw, &cfg, 10.0, Some(path)).unwrap();
        run_until(&mut g, &mut hw, secs(0.5)).unwrap();
        assert!(hw.pings.is_empty(), "unlocked a start that needs a hand-off");
        // it cools and takes the card once it may
        run_until(&mut g, &mut hw, secs(30.0)).unwrap();
        let first = hw.first_ping().expect("never armed");
        assert!(first > secs(0.5));
        assert!(hw.writes.iter().all(|w| w.1 < 10.0 - REARM_MARGIN_DB + 1e-9), "{:?}", hw.writes.first());
    }

    /// The unlock never lets an older value through: the controls are at
    /// their minimum when the lock is lifted.
    #[test]
    fn the_unlock_happens_on_muted_controls() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.reduction = 0.0; // an earlier run left full volume in the controls
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        while hw.pings.is_empty() {
            g.tick(&mut hw).unwrap();
            if hw.pings.is_empty() {
                assert!(hw.reduction >= MUTE_DB || !hw.unlocked_at(hw.t));
            }
        }
        assert_eq!(hw.writes.first().map(|w| w.1), Some(0.0));
    }

    /// Review F4's pattern in the legacy model: a period's energy at its
    /// start heats the coil above where its end leaves it.  0.1.2 stepped
    /// the period's mean once, saw only the end, and had no hard limit.
    #[test]
    fn a_peak_inside_a_period_latches_the_hard_limit() {
        let cfg = j700();
        let mut s = SpeakerState::new(cfg.speakers[0].clone(), cfg.globals.t_ambient);
        s.resume(132.3, 100.0, 0.0, 0.0);
        let mut powers = vec![0.0; SUB_BLOCKS];
        for p in powers.iter_mut().take(SUB_BLOCKS / 4) {
            *p = s.worst_case_power(0.0);
        }
        s.step_period(&powers, 4096.0 / 48000.0, cfg.globals.t_ambient, 0.0);
        assert!(s.t_coil < s.spec.t_limit, "end {:.2} C", s.t_coil);
        assert!(s.t_peak >= s.spec.t_limit, "peak {:.2} C", s.t_peak);
        assert!(s.hard);
        // the mean over the whole period would not have reached the limit
        let mut m = SpeakerState::new(cfg.speakers[0].clone(), cfg.globals.t_ambient);
        m.resume(132.3, 100.0, 0.0, 0.0);
        let mean = powers.iter().sum::<f64>() / SUB_BLOCKS as f64;
        m.step(mean, 4096.0 / 48000.0, cfg.globals.t_ambient);
        assert!(m.t_coil < m.spec.t_limit && m.t_peak < m.spec.t_limit, "{} {}", m.t_coil, m.t_peak);
        // the latch holds until the daemon clears it under the ceiling
        assert!(!s.clear_hard_if_cool());
        s.t_coil = 110.0;
        s.t_peak = 110.0;
        assert!(s.clear_hard_if_cool() && !s.hard);
    }

    /// The hard limit hands the card to the kernel whatever --max-reduction
    /// says.  A warm coil (95 C, still at full gain) loses 600 ms of sense
    /// data at full gain in one piece (a descheduled daemon, a lossy
    /// capture): the worst-case charge crosses 135 C.  0.1.2 handed off
    /// only on the reduction (21 dB at 135 C, under -m 40) and had no
    /// limit on the coil itself.
    #[test]
    fn the_coil_limit_hands_off_whatever_the_reduction_limit() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        let mut warm: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), cfg.globals.t_ambient)).collect();
        for s in warm.iter_mut() {
            s.resume(95.0, 70.0, 0.0, 0.0);
        }
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        std::fs::write(&path, state::encode(hw.boot, hw.boot, &warm, false)).unwrap();
        hw.lossy = true;
        hw.control_delay = Box::new(|t| if (secs(1.0)..secs(1.05)).contains(&t) { Duration::from_millis(600) } else { Duration::ZERO });
        let mut g = Guard::start(&mut hw, &cfg, 40.0, Some(path)).unwrap();
        let mut tripped = None;
        while hw.t < secs(4.0) {
            g.tick(&mut hw).unwrap();
            if g.speakers.iter().any(|s| s.hard) && tripped.is_none() {
                tripped = Some(hw.t);
            }
        }
        let at = tripped.expect("the model never reached the coil limit");
        assert!(at < secs(1.8), "tripped at {at:?}");
        assert!(!hw.pings.iter().any(|&p| p > at && p < at + secs(1.0)), "renewed after the coil limit at {at:?}");
    }

    /// Review F6: a complete period after lost time must not erase it.
    /// The daemon is descheduled for 240 ms and the capture keeps only the
    /// newest period.  0.1.2 reset its clock on that period and charged
    /// nothing; a queued backlog (not lossy) is measured, not charged.
    #[test]
    fn a_period_after_lost_time_does_not_erase_it() {
        let cfg = j700();
        let coil_after = |delay: bool, lossy: bool| {
            let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
            hw.level = Box::new(|_| 0.1);
            hw.lossy = lossy;
            if delay {
                hw.control_delay = Box::new(|t| if (secs(2.0)..secs(2.05)).contains(&t) { Duration::from_millis(240) } else { Duration::ZERO });
            }
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            run_until(&mut g, &mut hw, secs(2.6)).unwrap();
            g.speakers[0].t_coil
        };
        let (clean, queued, lost) = (coil_after(false, false), coil_after(true, false), coil_after(true, true));
        assert!((queued - clean).abs() < 0.5, "a queued backlog was charged: {queued:.2} vs {clean:.2} C");
        // about 155 ms at 12.7 W: some 18 C on a 50 C coil
        assert!(lost > clean + 12.0, "lost time was not charged: {lost:.2} vs {clean:.2} C");
    }

    /// Review F6, repeated: losses each shorter than a period plus the
    /// slack were never charged by 0.1.2.
    #[test]
    fn repeated_short_losses_are_all_charged() {
        let cfg = j700();
        let coil_after = |hiccups: bool| {
            let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
            hw.level = Box::new(|_| 0.1);
            hw.lossy = true;
            if hiccups {
                hw.control_delay = Box::new(|t| if t.as_secs_f64() % 1.0 < 0.05 && t > secs(1.0) { Duration::from_millis(150) } else { Duration::ZERO });
            }
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            run_until(&mut g, &mut hw, secs(10.0)).unwrap();
            g.speakers[0].t_coil
        };
        let (clean, lossy) = (coil_after(false), coil_after(true));
        assert!(lossy > clean + 20.0, "{lossy:.2} vs {clean:.2} C");
    }

    /// Review F5: the lock is not held without a state a restart can
    /// resume from.  0.1.2 held it whether or not the file could be
    /// written, so a crash restarted from an old snapshot or from cold.
    #[test]
    fn the_lock_is_not_taken_without_a_saved_state() {
        let cfg = j700();
        let dir = tmp_state().unwrap().parent().unwrap().to_path_buf();
        let path = dir.join("missing-dir").join("state");
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path)).unwrap();
        run_until(&mut g, &mut hw, secs(3.0)).unwrap();
        assert!(hw.pings.is_empty(), "unlocked without a saved state");
    }

    #[test]
    fn the_lock_is_released_when_saving_stops_working() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        let dir = path.parent().unwrap().to_path_buf();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path)).unwrap();
        run_until(&mut g, &mut hw, secs(2.0)).unwrap();
        assert!(g.armed());
        std::fs::remove_dir_all(&dir).unwrap();
        run_until(&mut g, &mut hw, secs(2.0) + SAVE_MAX_AGE + secs(0.3)).unwrap();
        assert!(!g.armed(), "held the card with a stale state file");
        let last = hw.pings.last().copied().unwrap();
        assert!(last < secs(2.0) + SAVE_MAX_AGE + PING_INTERVAL, "renewed at {last:?}");
    }

    /// Review R8: a missing state of this daemon proves nothing about the
    /// speakers (an older daemon with another state path, a failed write,
    /// a warm reboot).  0.1.3 started at the safe level's 58 C and
    /// unlocked at 0 dB after two quiet periods while a speaker coming from
    /// a 119 C session still needed 10.8 dB; 0.1.2 started at 50 C.
    #[test]
    fn a_missing_state_starts_latched_at_the_limit() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, missing_state()).unwrap();
        assert!(g.speakers.iter().all(|s| s.hard && s.t_coil >= s.spec.t_limit), "{}", g.speakers[0].t_coil);
        // the speaker the reviewer had in mind, cooled the same way
        let mut hot = SpeakerState::new(cfg.speakers[0].clone(), cfg.globals.t_ambient);
        hot.resume(119.0, 95.0, 0.0, 0.0);
        while hw.pings.is_empty() && hw.t < secs(30.0) {
            let before = hw.t;
            g.tick(&mut hw).unwrap();
            let d = (hw.t - before).as_secs_f64();
            hot.step(0.0, d, cfg.globals.t_ambient);
            hot.govern(&cfg.globals);
        }
        let first = hw.writes.first().expect("never armed").1;
        assert!(first >= hot.reduction_db - 1e-9, "unlocked at {first:.2} dB while a 119 C speaker needed {:.2} dB", hot.reduction_db);
        assert!(g.speakers[0].t_coil < g.speakers[0].spec.t_limit - g.speakers[0].spec.t_headroom);
    }

    /// A clean stop (SIGTERM) hands the card back before the final state is
    /// written, so the next run need not charge a powered tail.
    #[test]
    fn a_clean_stop_releases_the_card_then_saves() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.6);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path.clone())).unwrap();
        run_until(&mut g, &mut hw, secs(10.0)).unwrap();
        g.stop(&mut hw);
        let stop = hw.t;
        while !g.released(&hw) {
            g.tick(&mut hw).unwrap();
        }
        g.save(&hw);
        assert!(!hw.pings.iter().any(|&p| p > stop), "renewed after the stop");
        let saved = state::decode(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert!(!saved.armed, "the final state says the card may be held");
        let coil = g.speakers[0].t_coil;
        let mut hw2 = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw2.boot = hw.boot + hw.t + Duration::from_millis(500);
        let g2 = Guard::start(&mut hw2, &cfg, 20.0, Some(path)).unwrap();
        // 0.5 s at the safe level: cooler, by what the coil sheds in that time
        assert!(g2.speakers[0].t_coil < coil && g2.speakers[0].t_coil > coil - 10.0, "{} vs {coil}", g2.speakers[0].t_coil);
    }

    // ---- 0.1.4: a stream's edges ----

    /// Streams shorter than a sense period (UI sounds) never completed one,
    /// so 0.1.3 measured none of them: sixty 40 ms streams 10 ms apart at
    /// half of full scale left the model where it was.  Their frames are
    /// now read at close and the windows around them charged, so they heat
    /// the model as much as a continuous stream of the same mean power.
    #[test]
    fn back_to_back_short_streams_heat_the_model() {
        let cfg = j700();
        fn bursts(t: Duration) -> bool {
            let s = t.as_secs_f64();
            (1.0..4.0).contains(&s) && (s - 1.0) % 0.05 < 0.04
        }
        fn continuous(t: Duration) -> bool {
            (1.0..4.0).contains(&t.as_secs_f64())
        }
        let rise = |playing: fn(Duration) -> bool, level: f64| {
            let mut hw = dma_hw(&cfg, move |t| if playing(t) { 1.0 } else { 0.0 });
            hw.playing = Box::new(playing);
            hw.level = Box::new(move |_| level);
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            run_until(&mut g, &mut hw, secs(0.9)).unwrap();
            let before = g.speakers[0].t_coil;
            run_until(&mut g, &mut hw, secs(4.1)).unwrap();
            (g.speakers[0].t_coil - before, hw.opens)
        };
        let (b, opens) = rise(bursts, 0.5);
        let (c, _) = rise(continuous, 0.5 * 0.8f64.sqrt());
        assert!(opens >= 55, "{opens} streams");
        assert!(c > 10.0, "the continuous stream heated the coil by only {c:.2} K");
        assert!(b >= 0.8 * c, "sixty 40 ms streams heated the coil by {b:.2} K, a continuous stream of the same mean power by {c:.2} K");
    }

    /// With change events the window before a stream's first frame is the
    /// event's latency and the capture's start, not an idle poll: a quiet
    /// stream opened in the middle of an idle wait is not charged 100 ms at
    /// the worst case.
    #[test]
    fn a_change_event_starts_the_account_at_the_stream() {
        let cfg = j700();
        let coil = |events: bool| {
            let mut hw = dma_hw(&cfg, |t| if t > secs(1.05) { 1.0 } else { 0.0 });
            hw.playing = Box::new(|t| t > secs(1.05));
            hw.level = Box::new(|_| 0.01);
            hw.events = events;
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            run_until(&mut g, &mut hw, secs(1.5)).unwrap();
            g.speakers[0].t_coil
        };
        let (with, without) = (coil(true), coil(false));
        assert!(without > with + 1.0, "the polled opening window was not charged: {without:.2} vs {with:.2} C");
        let floor = 50.0 + 0.127 * 64.0;
        assert!(with < floor + 1.0, "an evented quiet stream heated the model to {with:.2} C");
    }

    /// Opening the J700's sense PCM blocks for about 100 ms while the
    /// playback is being set up, and a stream seen at its change event is
    /// opened right then: 0.1.4 let the lock lapse at the start of a sound
    /// after idle (the last idle ping up to 100 ms before, the open, then a
    /// 50 ms wait), so its first ~125 ms played at the safe level.
    #[test]
    fn a_slow_sense_open_does_not_lapse_the_lock() {
        let cfg = j700();
        for start in [1.03, 1.06, 1.09] {
            let mut hw = dma_hw(&cfg, move |t| if t.as_secs_f64() > start { 1.0 } else { 0.0 });
            hw.playing = Box::new(move |t| t.as_secs_f64() > start);
            hw.open_delay = Duration::from_millis(150);
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            run_until(&mut g, &mut hw, secs(3.0)).unwrap();
            assert!(hw.lapses.is_empty(), "stream at {start} s: the lock lapsed at {:?}", hw.lapses);
            assert!(g.armed());
        }
    }

    // ---- 0.1.6: the PR #7 review's witnesses ----

    /// Review R3: macaudio pauses the lease while the speakers are idle,
    /// so idle wall time does not bring the card to its safe level.  With
    /// the rate control failing while idle (and the hand-off write failing
    /// too), 0.1.5 assumed the safe level 250 ms after the last ping, while
    /// the card kept 0 dB into the next playback.  The model must stay at
    /// least as hot as one charged with what the card really applied.
    #[test]
    fn the_lease_is_counted_in_playing_time() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.playing = Box::new(|t| t >= secs(10.0));
        hw.level = Box::new(|_| 1.0);
        hw.rate_fail = Box::new(|t| (secs(1.0)..secs(10.3)).contains(&t));
        hw.volume_fail = Box::new(|t| (secs(1.0)..secs(10.3)).contains(&t));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        let mut real = g.speakers[0].clone();
        let mut at = Duration::ZERO;
        while hw.t < secs(10.3) {
            g.tick(&mut hw).unwrap();
            while at < hw.t {
                let d = Duration::from_micros(500);
                let p = if (hw.playing)(at) { real.worst_case_power(hw.effective_reduction()) } else { 0.0 };
                real.step(p, d.as_secs_f64(), cfg.globals.t_ambient);
                at += d;
            }
        }
        assert!(g.speakers[0].t_coil >= real.t_coil - 0.05, "model {:.2} C, charged with the card's real gain {:.2} C", g.speakers[0].t_coil, real.t_coil);
    }

    /// On a hand-off the controls are turned down to the safe level at
    /// once, so a paused lease cannot carry the old gain into playback.
    #[test]
    fn a_hand_off_turns_the_controls_down() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.playing = Box::new(|t| t >= secs(5.0));
        hw.rate_fail = Box::new(|t| (secs(1.0)..secs(5.5)).contains(&t));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(5.2)).unwrap();
        assert!(!g.armed());
        assert!(hw.effective_reduction() >= SAFE_DB, "the card plays at {} dB after the hand-off", hw.effective_reduction());
    }

    /// Review R6: up to 2 ms before each period was covered by the
    /// period's own power, so 1000 losses of 1.5 ms followed by silent
    /// periods were 1.5 s of energy taken as zero.  Frames after a loss
    /// leave all of the gap charged; contiguous frames run the account on
    /// by their own length and the clock's lead is charged once it shows.
    #[test]
    fn small_gaps_are_charged_not_filled_with_the_next_period() {
        let cfg = j700();
        let charged = |contiguous: bool| {
            let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
            hw.playing = Box::new(|_| true);
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            g.rate = 48000;
            g.armed = true;
            g.written = 0.0;
            g.held_gains = vec![(hw.now(), 0.0, true)];
            let dt = Duration::from_secs_f64(4096.0 / 48000.0);
            let t0 = hw.now();
            g.charged_to = Some(t0);
            for w in g.words.iter_mut() {
                *w = 0;
            }
            let mut end = t0;
            for _ in 0..1000 {
                end += Duration::from_micros(1500) + dt;
                g.account_frames(end, 4096, contiguous).unwrap();
            }
            g.speakers[0].t_coil
        };
        let mut want = SpeakerState::new(cfg.speakers[0].clone(), cfg.globals.t_ambient);
        // 1000 x 1.5 ms at full gain, spread over the 87 s the periods take
        for _ in 0..1000 {
            want.step(want.worst_case_power(0.0), 0.0015, cfg.globals.t_ambient);
            want.step(0.0, 4096.0 / 48000.0, cfg.globals.t_ambient);
        }
        let (lost, drift) = (charged(false), charged(true));
        assert!(lost >= want.t_coil - 0.05, "after losses {lost:.3} C, charged {:.3} C", want.t_coil);
        // the clock running ahead of contiguous frames is charged too, a
        // jitter's worth late at most
        assert!(drift >= want.t_coil - 0.5, "after a lead {drift:.3} C, charged {:.3} C", want.t_coil);
        assert!(lost > cfg.globals.t_ambient + 5.0);
    }

    /// Review R7: a working minimum write cleared the target write's
    /// failure count, so 700 failed target writes renewed 700 times without
    /// ever giving up.  Each operation now keeps its own count.
    #[test]
    fn target_write_failures_are_counted_on_their_own() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.playing = Box::new(|_| false);
        hw.target_fail = Box::new(|_| true);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        let err = run_until(&mut g, &mut hw, secs(200.0)).expect_err("700 failed target writes and still running");
        assert!(err.contains("volume control"), "{err}");
        assert!(hw.pings.len() <= MAX_CONTROL_FAILURES as usize + 1, "{} pings", hw.pings.len());
        assert!(hw.reduction >= MUTE_DB, "the controls stayed at their minimum");
    }

    /// Review R2: a minimum write that blocks for 2 s must not be followed
    /// by an unlock on the now stale snapshot and model.  0.1.5 pinged right
    /// after it, with a snapshot two seconds old.
    #[test]
    fn no_unlock_after_a_blocking_minimum_write() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.playing = Box::new(|_| false);
        hw.min_delay = Duration::from_secs(2);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path.clone())).unwrap();
        let mut pinged_stale = false;
        while hw.t < secs(10.0) {
            let before = hw.pings.len();
            g.tick(&mut hw).unwrap();
            if hw.pings.len() > before {
                let saved = state::decode(&std::fs::read_to_string(&path).unwrap()).unwrap();
                let age = (hw.boot + hw.t).saturating_sub(saved.boottime);
                // the ping is made before the end-of-pass save: look at the
                // save that preceded it
                pinged_stale |= g.last_save.is_some_and(|s| hw.now().duration_since(s) > SAVE_MAX_AGE) && age > SAVE_MAX_AGE;
            }
        }
        assert!(!pinged_stale, "unlocked on a snapshot older than {SAVE_MAX_AGE:?}");
        assert!(hw.pings.is_empty(), "every unlock followed a 2 s minimum write");
    }

    /// Review R2: the saved state names what the model covers.  During a
    /// stream it is the end of the last accounted frames, not the write
    /// time, so a restart charges the time in between.
    #[test]
    fn the_saved_state_names_its_coverage() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path.clone())).unwrap();
        run_until(&mut g, &mut hw, secs(2.0)).unwrap();
        // a pass that ends between periods: the account trails the clock
        let charged = g.charged_to.unwrap();
        g.save(&hw);
        let saved = state::decode(&std::fs::read_to_string(&path).unwrap()).unwrap();
        let lag = saved.boottime.saturating_sub(saved.covered);
        assert_eq!(lag, hw.now().saturating_duration_since(charged).min(lag), "coverage {:?} behind the write", lag);
        assert!(saved.covered <= saved.boottime);
    }

    /// Review R4 at the loop: a run latched at its limit and saved while
    /// still above the working ceiling does not rearm after a restart
    /// before the coil is under it.  0.1.5 rearmed at 127 C.
    #[test]
    fn a_latched_state_is_honoured_after_a_restart() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        let mut sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), cfg.globals.t_ambient)).collect();
        for s in sp.iter_mut() {
            s.resume(131.94, 100.0, 0.0, 0.0);
            s.hard = true;
            s.t_peak = 135.256;
        }
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.0);
        std::fs::write(&path, state::encode(hw.boot, hw.boot, &sp, false)).unwrap();
        hw.boot += Duration::from_millis(50);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path)).unwrap();
        let ceiling = cfg.speakers[0].t_limit - cfg.speakers[0].t_headroom;
        while hw.pings.is_empty() && hw.t < secs(30.0) {
            g.tick(&mut hw).unwrap();
        }
        assert!(!hw.pings.is_empty(), "never rearmed");
        assert!(g.speakers[0].t_coil < ceiling, "rearmed at {:.2} C", g.speakers[0].t_coil);
    }

    /// Found porting PR #6 (the rework, review F1): time charged after the
    /// card's gain changed is charged at what the card held while it
    /// passed.  0.1.6 charged it at the gain of the moment of the charge:
    /// the hand-off after a failed rate read wrote the safe level first,
    /// and the time since the last period, played at 0 dB, was then charged
    /// at 20 dB (100x less power).
    #[test]
    fn time_before_a_hand_off_is_charged_at_the_gain_it_played_at() {
        let cfg = j700();
        let ta = cfg.globals.t_ambient;
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        g.rate = 48000;
        g.armed = true;
        g.written = 0.0;
        g.lease_left = KERNEL_LOCK_TIMEOUT;
        let t0 = hw.now();
        g.held_gains = vec![(t0, 0.0, true)];
        g.charged_to = Some(t0);
        // 150 ms of playback at 0 dB not yet accounted, then a hand-off
        hw.t += Duration::from_millis(150);
        g.disarm(&mut hw, "test").unwrap();
        assert_eq!(g.written, SAFE_DB, "the hand-off did not write the safe level");
        let before = g.speakers[0].clone();
        let now = hw.now();
        g.charge_worst(t0, now).unwrap();
        let mut want = before.clone();
        want.step_at(want.worst_case_power(0.0), 0.15, ta, 0.0);
        assert!(g.speakers[0].t_coil >= want.t_coil - 1e-9, "charged {:.3} C, at the gain it played at {:.3} C", g.speakers[0].t_coil, want.t_coil);
        // and time after the hand-off at the safe level
        let t1 = hw.now();
        hw.t += Duration::from_millis(100);
        let before = g.speakers[0].clone();
        g.charge_worst(t1, hw.now()).unwrap();
        let mut want = before.clone();
        want.step_at(want.worst_case_power(SAFE_DB), 0.1, ta, SAFE_DB);
        assert!((g.speakers[0].t_coil - want.t_coil).abs() < 1e-9);
    }

    /// The same through the loop: a rate read fails right after a period
    /// at full gain; the hand-off writes the safe level, and the time since
    /// the period is still charged at 0 dB.
    #[test]
    fn a_rate_failure_charges_the_time_before_it_at_the_old_gain() {
        let cfg = j700();
        let coil = |fail: bool| {
            let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
            hw.level = Box::new(|_| 0.0);
            if fail {
                hw.rate_fail = Box::new(|t| (secs(2.0)..secs(2.05)).contains(&t));
            }
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            run_until(&mut g, &mut hw, secs(2.0)).unwrap();
            // the failing pass charges the time since the account's end, all
            // of which played at 0 dB before the hand-off
            let pending = hw.now().saturating_duration_since(g.charged_to.unwrap()).as_secs_f64();
            let start = g.speakers[0].clone();
            g.tick(&mut hw).unwrap();
            (g.speakers[0].t_coil, start, pending)
        };
        let (failed, start, pending) = coil(true);
        assert!(pending > 0.0);
        let mut want = start.clone();
        want.step_at(want.worst_case_power(0.0), pending, cfg.globals.t_ambient, 0.0);
        assert!(failed >= want.t_coil - 1e-6, "{failed:.3} C, the pending {pending:.3} s at 0 dB gives {:.3} C", want.t_coil);
    }
}
