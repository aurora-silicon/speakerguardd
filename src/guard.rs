// SPDX-License-Identifier: MIT
//! The protection loop, behind a small hardware interface so that its
//! timing rules can be exercised without a sound card: the card's volume
//! lock, its speaker sample-rate control and the sense capture.
//!
//! The rules it keeps (0.1.8: simple over precise):
//!
//! * One account.  The thermal model has accounted wall time up to
//!   `account_to`, and every step advances it by exactly the time stepped:
//!   thermal time moves once per wall interval, never twice.
//! * Measured or charged.  Time is either covered by measured sense frames
//!   (placed by the read time less the capture's backlog), or charged as a
//!   full-scale signal at the least reduction the controls held at any time
//!   since the last verified boundary, widened by how long the card keeps
//!   playing samples converted at an earlier gain.  No partial credits, no
//!   tolerances, no means, no assumption about when inside an uncertain
//!   interval anything played.  Frames over time already charged add
//!   nothing.
//! * Verified boundaries.  The end of measured frames, or two looks at
//!   closed speakers with no sample-rate change event between them, and
//!   nothing else.  Without the card's change events nothing between two
//!   looks is known, and the daemon never takes the lock.
//! * The lock.  Not taken before the model allows it and the daemon can see:
//!   no unlock at start, and while the speakers play not before
//!   REARM_PERIODS complete periods.  Controls to their minimum before each
//!   unlock.  Every hand-off gives the card's lock up (macaudio locks the
//!   volume and stops streams converted at the old gain); if that fails the
//!   daemon exits, which does the same.
//! * A model that may have reached a coil's limit inside a period latches a
//!   hard fault; the card stays with the kernel until the coil has cooled.
//! * Restarts.  Only a state written by a clean stop, after the lock was
//!   given up, its converted samples drained and every moment accounted,
//!   is resumed.  Any other state, or none, starts at the coil's limit,
//!   latched.
//!
//! The cost, accepted: every stream after idle is charged at full scale
//! from the last look at closed speakers (up to one idle poll) to its first
//! frames, and the end of every stream from its last frames to the look
//! that sees it closed; a sense outage is charged at full scale at the
//! least gain since the last verified boundary until periods return, and a
//! restart without a clean stop starts latched (about 4 s with the
//! speakers at the card's safe level on the J700).

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
/// How often closed speakers are looked at: a stream after idle is charged
/// at full scale from the last look to its first frames, so the look is
/// kept short.
pub const IDLE_POLL: Duration = Duration::from_millis(25);
/// Cadence of the per-speaker debug line.
const LOG_INTERVAL: Duration = Duration::from_secs(1);
/// Cadence of the state file (a status readout; only a clean stop's state
/// is resumed by a restart).
pub const SAVE_INTERVAL: Duration = Duration::from_millis(500);
/// Consecutive failures of one control operation (about a minute of
/// retries) before the daemon gives up and exits.  Every failure already
/// hands the speakers to the card's safe level; exiting only stops the
/// retries.
const MAX_CONTROL_FAILURES: u32 = 600;
/// Sense data missing for this long means the stream is gone, not late.
pub const STALL_TIMEOUT: Duration = Duration::from_millis(500);
/// How long the card can keep playing samples converted at an earlier gain
/// after a gain change: the J700 converts on copy into a playback queue of
/// up to 16384 frames, 341 ms at 48 kHz.  With margin.
pub const QUEUE_BOUND: Duration = Duration::from_millis(500);
/// After the lock is given up, the time until samples converted at the old
/// gain are certainly gone (macaudio's stop work, then the queue).
pub const DRAIN: Duration = QUEUE_BOUND;

/// What the loop needs from the machine.
pub trait Hw {
    fn now(&self) -> Instant;
    /// Time since boot, suspend included (CLOCK_BOOTTIME).
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
    /// while the volume is locked: done before each unlock.
    fn set_min(&mut self) -> io::Result<()>;
    /// Give the card's lock up and take it again: the card locks the
    /// volume at once, drops what was left of its lease and stops the
    /// streams converted at the old gain.
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
    /// Whether the sample rate changed since the last call (consumes it;
    /// an error counts as a change).
    fn rate_events(&mut self) -> bool;
    /// The card's change events are being delivered.
    fn events_available(&self) -> bool;
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

/// How late the next sense period may be before the time since the
/// account's end is charged (only when to charge: once it is, all of it is).
pub const BLIND_SLACK: Duration = PING_INTERVAL;
/// Without complete sense data for this long (after the period that should
/// have arrived) the daemon hands the speakers to the kernel's safe level.
pub const BLIND_LIMIT: Duration = STALL_TIMEOUT;
/// The kernel's lock timeout (macaudio SPEAKER_LOCK_TIMEOUT), in playing
/// time.
pub const KERNEL_LOCK_TIMEOUT: Duration = Duration::from_millis(250);
/// Consecutive complete periods needed before the lock is taken back.
pub const REARM_PERIODS: u32 = 2;
/// Hysteresis between handing the card to the kernel at --max-reduction and
/// taking it back.
pub const REARM_MARGIN_DB: f64 = 3.0;
/// The reduction the volume controls' minimum stands for (dB).
pub const MUTE_DB: f64 = 120.0;

pub struct Guard {
    globals: Globals,
    pub speakers: Vec<SpeakerState>,
    max_reduction: f64,
    /// True while the daemon renews the card's volume lock.
    armed: bool,
    /// Ever armed in this run (for the log).
    held: bool,
    /// The open sense capture's stall clock; it lives and dies with the
    /// capture.
    stall: Option<StallClock>,
    /// Seconds per sense period while the speakers play, None while closed.
    dt: Option<f64>,
    /// When the last complete sense period was read (the stream's first
    /// sighting until one is).
    seen_to: Option<Instant>,
    /// The speakers' rate while they play (0 before the first stream).
    rate: u32,
    /// The thermal model has accounted wall time up to here.
    account_to: Instant,
    /// A stream is open: seen with a rate, not yet seen closed.
    playing: bool,
    /// The last look saw the speakers closed.
    closed_seen: bool,
    /// The last sample-rate read failed: playback state unknown.
    unknown: bool,
    /// The card's change events are delivered (else the lock is never taken).
    events: bool,
    /// Complete periods in a row.
    fresh: u32,
    words: Vec<i32>,
    last_ping: Instant,
    last_log: Instant,
    /// When the governed reduction was last written.
    last_write: Instant,
    /// The reduction the card's volume controls hold, as far as the daemon
    /// knows (dB; infinite = unknown, taken as full gain).
    written: f64,
    applied: f64,
    /// What the controls held: (from, written), oldest first, back to
    /// QUEUE_BOUND before the last verified boundary.
    gains: Vec<(Instant, f64)>,
    /// The last verified boundary.
    verified_at: Instant,
    rate_failures: u32,
    /// Failures of the volume writes: to the minimum before an unlock, and
    /// to the governed reduction.  Each clears only on its own success.
    min_failures: u32,
    target_failures: u32,
    state_path: Option<PathBuf>,
    last_save_try: Option<Instant>,
    save_failed: bool,
    stopping: bool,
    /// When the lock was given up for the stop.
    relinquished_at: Option<Instant>,
}

impl Guard {
    /// Restore the model from a clean stop (if there is one) and lock the
    /// card's controls.  The volume lock is not lifted here.
    pub fn start<H: Hw>(hw: &mut H, cfg: &Config, max_reduction: f64, state_path: Option<PathBuf>) -> Result<Guard, Box<dyn Error>> {
        if !max_reduction.is_finite() || max_reduction <= 0.0 {
            return Err("maximum reduction must be finite and positive".into());
        }
        let globals = cfg.globals.clone();
        let mut speakers: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), globals.t_ambient)).collect();
        let latched = |speakers: &mut Vec<SpeakerState>| {
            for s in speakers.iter_mut() {
                s.assume_latched(globals.t_ambient);
            }
        };
        match &state_path {
            None => {
                warn!("no model state file: assuming hot speakers (latched)");
                latched(&mut speakers);
            }
            Some(path) => match std::fs::read_to_string(path) {
                Ok(text) => match state::decode(&text) {
                    Some(saved) => {
                        if state::restore(&mut speakers, &saved, hw.boottime(), &globals) {
                            info!("model restored from {} (a clean stop {:.1} s ago)", path.display(), hw.boottime().saturating_sub(saved.boottime).as_secs_f64());
                        } else {
                            info!("{}: not from a clean stop; assuming hot speakers (latched)", path.display());
                        }
                    }
                    None => {
                        warn!("{}: unreadable model state; assuming hot speakers (latched)", path.display());
                        latched(&mut speakers);
                    }
                },
                Err(e) if e.kind() == io::ErrorKind::NotFound => {
                    info!("{}: no model state; assuming hot speakers (latched)", path.display());
                    latched(&mut speakers);
                }
                Err(e) => {
                    warn!("{}: {e}; assuming hot speakers (latched)", path.display());
                    latched(&mut speakers);
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
        let events = hw.events_available();
        if !events {
            warn!("no control events from the card: streams between two looks cannot be accounted; leaving the speakers at the card's safe level");
        }
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
            seen_to: None,
            rate: 0,
            account_to: now,
            playing: false,
            closed_seen: false,
            unknown: false,
            events,
            fresh: 0,
            last_ping: now,
            last_log: now,
            last_write: now,
            written: f64::INFINITY,
            applied: 0.0,
            gains: vec![(now, f64::INFINITY)],
            verified_at: now,
            rate_failures: 0,
            min_failures: 0,
            target_failures: 0,
            state_path,
            last_save_try: None,
            save_failed: false,
            stopping: false,
            relinquished_at: None,
        })
    }

    #[cfg(test)]
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Stop renewing the lock for good (SIGTERM): give the card's lock up.
    /// The loop keeps accounting until `released`, then the caller writes
    /// the final state (`final_save`) and exits.  An error here leaves the
    /// state as it is (not a clean stop) and the caller exits at once.
    pub fn stop<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        if self.armed {
            info!("stopping: leaving the speakers at the card's safe level");
        }
        self.stopping = true;
        self.armed = false;
        hw.relinquish().map_err(|e| format!("giving up the volume lock: {e}"))?;
        self.relinquished_at = Some(hw.now());
        Ok(())
    }

    /// The lock was given up for the stop and the samples converted at the
    /// old gain have drained.
    pub fn released<H: Hw>(&self, hw: &H) -> bool {
        !self.armed && self.relinquished_at.is_some_and(|t| hw.now() >= t + DRAIN)
    }

    /// The state a restart may resume: everything up to now accounted, the
    /// lock given up and drained.
    pub fn final_save<H: Hw>(&mut self, hw: &H) -> Result<(), Box<dyn Error>> {
        if !self.released(hw) {
            return Err("final save before the card was released".into());
        }
        self.charge(hw.now())?;
        if self.save_now(hw, true) {
            Ok(())
        } else {
            Err("the final model state could not be saved".into())
        }
    }

    /// The reduction the controls hold, unknown taken as full gain.
    fn written_db(&self) -> f64 {
        if self.written.is_finite() { self.written } else { 0.0 }
    }

    /// The controls' value changed at `at`.
    fn note_written(&mut self, at: Instant) {
        if self.gains.last().is_none_or(|&(_, w)| w != self.written) {
            self.gains.push((at, self.written));
        }
    }

    /// A verified boundary at `at`: only the gains held from QUEUE_BOUND
    /// before it on can still be playing.
    fn verify(&mut self, at: Instant) {
        self.verified_at = at;
        let from = at.checked_sub(QUEUE_BOUND).unwrap_or(at);
        let keep = self.gains.iter().rposition(|&(t, _)| t <= from).unwrap_or(0);
        self.gains.drain(..keep);
    }

    /// The least and the most reduction the controls held since QUEUE_BOUND
    /// before the last verified boundary (unknown counts as full gain).
    fn held(&self) -> (f64, f64) {
        let mut least = f64::INFINITY;
        let mut most = 0.0f64;
        for &(_, w) in &self.gains {
            let v = if w.is_finite() { w } else { 0.0 };
            least = least.min(v);
            most = most.max(v);
        }
        let now = self.written_db();
        (least.min(now), most.max(now))
    }

    /// The most reduction the controls held for any real time (1 ms or
    /// more, or held now) over [from, to): what measured frames played at,
    /// for the budgets' power before the gain.  The minimum written just
    /// before each unlock lasts no time and is no gain anything played at.
    fn most_during(&self, from: Instant, to: Instant) -> f64 {
        let mut most: Option<f64> = None;
        for (i, &(t, w)) in self.gains.iter().enumerate() {
            let next = self.gains.get(i + 1).map(|n| n.0);
            let lasted = next.is_none_or(|n| n.saturating_duration_since(t) >= Duration::from_millis(1));
            let overlaps = next.is_none_or(|n| n > from) && (t < to || i == 0);
            if lasted && overlaps {
                let v = if w.is_finite() { w } else { 0.0 };
                most = Some(most.map_or(v, |m: f64| m.max(v)));
            }
        }
        most.unwrap_or_else(|| self.written_db())
    }

    /// Give the card's lock up: macaudio locks the volume at once and stops
    /// what was converted at the old gain.  The account keeps the old gain
    /// until the next verified boundary.  A failure is fatal: the daemon
    /// exits, which gives the lock up too.
    fn hand_off<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        self.armed = false;
        hw.relinquish().map_err(|e| format!("giving up the volume lock: {e}").into())
    }

    fn disarm<H: Hw>(&mut self, hw: &mut H, why: &str) -> Result<(), Box<dyn Error>> {
        if self.armed {
            warn!("{why}; leaving the speakers at the card's safe level");
            self.hand_off(hw)?;
        }
        Ok(())
    }

    /// Pings the volume lock.  A ping that answers ETIMEDOUT is the card's
    /// one-shot notice that the lock had lapsed, so ping once more.
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
        Ok(())
    }

    fn check_model(&self) -> Result<(), Box<dyn Error>> {
        if self.speakers.iter().all(|s| s.finite()) {
            Ok(())
        } else {
            Err("nonfinite speaker model state".into())
        }
    }

    /// Charge the time from the account's end to `to` as a full-scale
    /// signal at the least reduction held since the last verified boundary.
    fn charge(&mut self, to: Instant) -> Result<(), Box<dyn Error>> {
        if to <= self.account_to {
            return Ok(());
        }
        let d = (to - self.account_to).as_secs_f64();
        let (least, _) = self.held();
        let globals = &self.globals;
        for s in self.speakers.iter_mut() {
            let p = s.worst_case_power(least);
            s.step_at(p, d, globals.t_ambient, least);
            s.govern(globals);
        }
        self.account_to = to;
        self.check_model()
    }

    /// Verified closed speakers from the account's end to `to`: no power.
    fn cool(&mut self, to: Instant) -> Result<(), Box<dyn Error>> {
        if to > self.account_to {
            let d = (to - self.account_to).as_secs_f64();
            let globals = &self.globals;
            for s in self.speakers.iter_mut() {
                s.step(0.0, d, globals.t_ambient);
                s.govern(globals);
            }
            self.account_to = to;
        }
        self.verify(to);
        self.check_model()
    }

    /// `frames` sense frames in `self.words` whose last one was captured at
    /// `end` (the read time less the capture's backlog).  Time before them
    /// since the account's end is charged first; frames over time already
    /// accounted add nothing; frames partly over it count only for the rest,
    /// at the most power of the blocks that cover the rest.
    fn measure(&mut self, end: Instant, frames: usize) -> Result<(), Box<dyn Error>> {
        if frames == 0 || self.rate == 0 || end <= self.account_to {
            return Ok(());
        }
        let dt = frames as f64 / self.rate as f64;
        let start = end.checked_sub(Duration::from_secs_f64(dt)).unwrap_or(end);
        if start > self.account_to {
            self.charge(start)?;
        }
        let globals = self.globals.clone();
        let words = &self.words[..frames * globals.channels];
        let blocks = (SUB_BLOCKS * frames).div_ceil(globals.period).max(1);
        let span = (end - self.account_to).as_secs_f64().min(dt);
        let whole = span >= dt;
        // the budgets' power before the gain: the most reduction it played
        // at (more power before the gain is the safer guess there)
        let most = self.most_during(start, end);
        for s in self.speakers.iter_mut() {
            let (powers, v) = s.block_powers(words, globals.channels, blocks);
            if !v.is_finite() || powers.iter().any(|p| !p.is_finite() || *p < 0.0) {
                return Err("invalid speaker sense voltage or power".into());
            }
            s.v_rms = v;
            if whole {
                s.step_period(&powers, dt, globals.t_ambient, most);
            } else {
                // the blocks covering the last `span` of the frames
                let n = powers.len();
                let k = ((span / dt) * n as f64).ceil().clamp(1.0, n as f64) as usize;
                let p = powers[n - k..].iter().cloned().fold(0.0, f64::max);
                s.step_at(p, span, globals.t_ambient, most);
            }
            s.govern(&globals);
        }
        self.account_to = end;
        self.verify(end);
        self.check_model()
    }

    /// A closing stream: measure what the capture holds after the last
    /// whole period, then charge the rest up to now.
    fn account_close<H: Hw>(&mut self, hw: &mut H) -> Result<usize, Box<dyn Error>> {
        let mut measured = 0;
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
                // without the capture's position the frames cannot be placed
                let Ok(q) = hw.queued() else { break };
                let now = hw.now();
                let end = now.checked_sub(Duration::from_secs_f64(q as f64 / self.rate.max(1) as f64)).unwrap_or(now);
                self.measure(end, frames)?;
                measured += frames;
                if frames < self.globals.period || measured > 8 * self.globals.period {
                    break;
                }
            }
        }
        let now = hw.now();
        self.charge(now)?;
        Ok(measured)
    }

    /// One pass of the loop: idle accounting, or one sense period.
    pub fn tick<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        let globals = self.globals.clone();
        // a change event since the last pass (read before the rate, so that
        // a change after the read is kept for the next pass)
        let changed = hw.rate_events();
        let rate = match hw.sample_rate() {
            Ok(v) => {
                self.rate_failures = 0;
                self.unknown = false;
                u32::try_from(v).map_err(|_| "invalid negative or excessive speaker sample rate")?
            }
            Err(e) if transient(&e) && self.rate_failures < MAX_CONTROL_FAILURES => {
                // Playback state unknown: everything since the account's end
                // is charged (maintain), and the card is handed off.
                self.rate_failures += 1;
                warn!("sample-rate control: {e}; retrying ({})", self.rate_failures);
                self.unknown = true;
                self.closed_seen = false;
                self.fresh = 0;
                self.disarm(hw, "the speaker sample-rate control failed")?;
                self.maintain(hw)?;
                hw.sleep(PING_INTERVAL);
                return Ok(());
            }
            Err(e) => return Err(format!("sample-rate control: {e}").into()),
        };
        if rate == 0 {
            let now = hw.now();
            if self.playing {
                // the stream's last frames, then the rest up to this look
                let tail = self.account_close(hw)?;
                self.playing = false;
                if self.stall.take().is_some() {
                    hw.close_sense();
                    info!("speakers closed ({tail} sense frames after the last whole period)");
                }
            } else if self.events && self.closed_seen && !changed {
                // closed at both looks, nothing changed between: no power
                self.cool(now)?;
            } else {
                // a change between two looks at closed speakers (a stream
                // that came and went), or nothing known about the time
                self.charge(now)?;
            }
            self.dt = None;
            self.seen_to = None;
            self.fresh = 0;
            self.closed_seen = true;
            self.maintain(hw)?;
            hw.wait_rate_change(IDLE_POLL);
            return Ok(());
        }
        let dt = globals.period as f64 / rate as f64;
        if !dt.is_finite() || dt <= 0.0 {
            return Err("invalid sense period".into());
        }
        self.dt = Some(dt);
        self.rate = rate;
        if !self.playing {
            // a stream seen for the first time: it may have played since
            // the last look at closed speakers, all of which is charged
            let now = hw.now();
            self.charge(now)?;
            self.playing = true;
            self.closed_seen = false;
            self.seen_to = Some(now);
        }
        if self.seen_to.is_none() {
            self.seen_to = Some(self.account_to);
        }
        if self.stall.is_none() {
            // Opening the sense PCM blocks while the playback side is being
            // set up (about 100 ms on the J700): renew a lock that is still
            // held first, so the open cannot run its lease out.  The time is
            // charged all the same.
            if self.armed && hw.now() < self.last_ping + KERNEL_LOCK_TIMEOUT {
                self.ping_lock(hw)?;
            }
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
                        return self.maintain(hw);
                    }
                };
                self.stall.as_mut().unwrap().data();
                let now = hw.now();
                let end = now.checked_sub(Duration::from_secs_f64(queued as f64 / rate as f64)).unwrap_or(now);
                self.measure(end, globals.period)?;
                self.seen_to = Some(now);
                self.fresh = self.fresh.saturating_add(1);
            }
            Ok(Some(false)) => {
                warn!("incomplete sense period or overrun, step skipped");
                self.fresh = 0;
            }
            Ok(None) => {
                if self.stall.as_mut().unwrap().silent(hw.now()) {
                    warn!("sense stream stalled for {} ms; reopening", STALL_TIMEOUT.as_millis());
                    self.stall = None;
                    self.fresh = 0;
                    hw.close_sense();
                }
            }
            Err(e) if transient(&e) => {
                warn!("sense capture: {e}; reopening");
                self.stall = None;
                self.fresh = 0;
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

    /// Write the state file (atomically): a status readout, or with
    /// `released` the state a restart may resume.
    fn save_now<H: Hw>(&mut self, hw: &H, released: bool) -> bool {
        let Some(path) = &self.state_path else { return true };
        let now = hw.now();
        self.last_save_try = Some(now);
        let boot = hw.boottime();
        let covered = boot.saturating_sub(now.saturating_duration_since(self.account_to));
        let text = state::encode(boot, covered, &self.speakers, self.armed, released);
        let tmp = path.with_extension("tmp");
        let result = std::fs::write(&tmp, text).and_then(|()| std::fs::rename(&tmp, path));
        match result {
            Ok(()) => {
                self.save_failed = false;
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

    /// The model is current enough to lift or renew the lock now (checked
    /// again after anything that may block).
    fn fresh_enough(&self, now: Instant) -> bool {
        let allowed = Duration::from_secs_f64(self.dt.unwrap_or(0.0)) + BLIND_SLACK;
        now.saturating_duration_since(self.account_to) <= allowed
    }

    /// Charge unmeasured playback time, decide whether the daemon holds the
    /// card, and if it does write the model's reduction and renew the lock.
    fn maintain<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        let now = hw.now();
        let blind = self.overdue(now);
        if self.unknown || (self.playing && blind > BLIND_SLACK) {
            // nothing can cover the time since the account's end any more
            self.charge(now)?;
        }
        let reduction = model::reduction_db(&self.speakers);
        if !reduction.is_finite() {
            return Err("nonfinite speaker reduction".into());
        }
        let hard = self.speakers.iter().any(|s| s.hard);
        if self.armed && now >= self.last_ping + KERNEL_LOCK_TIMEOUT {
            // the daemon was late and the card may have taken its volume
            // back: give it up, then take it again the safe way
            self.disarm(hw, "the card's lock lapsed while the daemon was away")?;
        }
        if self.armed {
            let why = if blind > BLIND_LIMIT {
                Some(format!("no complete sense data for {} ms", blind.as_millis()))
            } else if hard {
                Some("a coil may have reached its limit".to_string())
            } else if reduction >= self.max_reduction {
                Some(format!("the model asks for {reduction:.1} dB"))
            } else {
                None
            };
            if let Some(why) = why {
                self.disarm(hw, &why)?;
            }
        } else if !self.stopping && !self.unknown && !hard && self.events {
            let seeing = !self.playing || (self.fresh >= REARM_PERIODS && blind <= BLIND_SLACK);
            if seeing && reduction < self.max_reduction - REARM_MARGIN_DB {
                match hw.set_min() {
                    Ok(()) => {
                        self.written = MUTE_DB;
                        self.note_written(hw.now());
                        self.min_failures = 0;
                        // the minimum write may have blocked: the model must
                        // still be current at the unlock
                        if self.fresh_enough(hw.now()) {
                            self.ping_lock(hw)?;
                            info!("taking the volume lock{} at {reduction:.1} dB", if self.held { " back" } else { "" });
                            self.armed = true;
                            self.held = true;
                            self.applied = reduction;
                        } else {
                            debug!("not taking the lock: the model fell behind");
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
            let due = reduction != self.written || hw.now().duration_since(self.last_write) >= PING_INTERVAL;
            match if due { hw.set_reduction(reduction) } else { Ok(()) } {
                Ok(()) if !due => {}
                Ok(()) => {
                    self.written = reduction;
                    self.last_write = hw.now();
                    self.note_written(hw.now());
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
                    self.note_written(hw.now());
                    self.disarm(hw, "the volume control failed")?;
                }
                Err(e) => return Err(format!("volume control: {e}").into()),
            }
            if self.armed && hw.now().duration_since(self.last_ping) >= PING_INTERVAL {
                // the write may have blocked: renew only on a current model
                if self.fresh_enough(hw.now()) {
                    self.ping_lock(hw)?;
                } else {
                    self.disarm(hw, "the model fell behind")?;
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
            self.save_now(hw, false);
        }
        Ok(())
    }

    /// Write the state file as a status readout (a restart does not
    /// resume it).
    pub fn save<H: Hw>(&mut self, hw: &H) {
        self.save_now(hw, false);
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
            if self.unlocked_at(self.t) {
                self.lapses.push(self.t);
            }
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
        fn events_available(&self) -> bool {
            self.events
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
        std::fs::write(&path, state::encode(boot, boot, &cold, false, true)).unwrap();
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
        run_until(&mut g, &mut hw, secs(4.0)).expect("a sense outage must not end the daemon");
        assert_eq!(hw.lapses.len(), 1, "{:?}", hw.lapses);
        let lapse = hw.lapses[0];
        assert!(lapse > secs(2.0) && lapse < secs(3.0), "clamped at {lapse:?}");
        assert!(
            !hw.pings.iter().any(|&p| p > lapse && p < secs(4.0)),
            "the lock stays lapsed while the daemon is blind"
        );
        assert!(hw.opens >= 2, "the stalled capture was reopened");
        // the 2 s it could not see were charged at full scale (0.1.8's
        // rule): the lock comes back once that has cooled
        run_until(&mut g, &mut hw, secs(60.0)).unwrap();
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
        run_until(&mut g, &mut hw, secs(60.0)).unwrap();
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
        // each sound's edges are charged at full scale (0.1.8's rule): a
        // burst of them may hand the card to the kernel for a while, but
        // the daemon governs again during the music
        let last_lapse = hw.lapses.last().copied().unwrap_or(Duration::ZERO);
        assert!(last_lapse < secs(4.2) + secs(5.0), "still clamped at {last_lapse:?}");
        assert!(g.armed() && hw.unlocked_at(hw.t));
        assert!(hw.max_ping_gap(secs(9.2), hw.t) < LOCK_TIMEOUT);
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
        run_until(&mut g, &mut hw, secs(60.0)).unwrap();
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
        run_until(&mut g, &mut hw, secs(90.0)).unwrap();
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
        // the control returns: the 10 s it could not see were charged at
        // full scale (0.1.8's rule), so the card is taken once that has
        // cooled
        run_until(&mut g, &mut hw, secs(60.0)).unwrap();
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
        std::fs::write(&path, state::encode(hw.boot, hw.boot, &warm, false, true)).unwrap();
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
        g.stop(&mut hw).unwrap();
        let stop = hw.t;
        while !g.released(&hw) {
            g.tick(&mut hw).unwrap();
        }
        g.final_save(&hw).unwrap();
        assert!(!hw.pings.iter().any(|&p| p > stop), "renewed after the stop");
        let saved = state::decode(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert!(!saved.armed && saved.released && saved.covered == saved.boottime, "the final state is not a clean stop");
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
    /// periods were 1.5 s of energy taken as zero.  Every gap before
    /// measured frames is charged, whatever its size.
    #[test]
    fn small_gaps_are_charged_not_filled_with_the_next_period() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        g.rate = 48000;
        g.written = 0.0;
        let t0 = hw.now();
        g.gains = vec![(t0, 0.0)];
        g.verified_at = t0;
        g.account_to = t0;
        for w in g.words.iter_mut() {
            *w = 0;
        }
        let dt = Duration::from_secs_f64(4096.0 / 48000.0);
        let mut end = t0;
        for _ in 0..1000 {
            end += Duration::from_micros(1500) + dt;
            g.measure(end, 4096).unwrap();
        }
        let mut want = SpeakerState::new(cfg.speakers[0].clone(), cfg.globals.t_ambient);
        for _ in 0..1000 {
            want.step(want.worst_case_power(0.0), 0.0015, cfg.globals.t_ambient);
            want.step(0.0, 4096.0 / 48000.0, cfg.globals.t_ambient);
        }
        let got = g.speakers[0].t_coil;
        assert!(got >= want.t_coil - 0.05, "after losses {got:.3} C, charged {:.3} C", want.t_coil);
        assert!(got > cfg.globals.t_ambient + 5.0);
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
    /// by an unlock on a model two seconds behind.
    #[test]
    fn no_unlock_after_a_blocking_minimum_write() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.playing = Box::new(|_| false);
        hw.min_delay = Duration::from_secs(2);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(10.0)).unwrap();
        assert!(hw.pings.is_empty(), "every unlock followed a 2 s minimum write");
    }

    /// The state file names what the model covers: during a stream, the
    /// end of the last accounted frames, not the write time.
    #[test]
    fn the_saved_state_names_its_coverage() {
        let cfg = j700();
        let path = tmp_state().unwrap();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path.clone())).unwrap();
        run_until(&mut g, &mut hw, secs(2.0)).unwrap();
        let accounted = g.account_to;
        g.save(&hw);
        let saved = state::decode(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(saved.boottime.saturating_sub(saved.covered), hw.now().saturating_duration_since(accounted));
        assert!(!saved.released, "a running daemon's state is never a release");
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
        std::fs::write(&path, state::encode(hw.boot, hw.boot, &sp, false, true)).unwrap();
        hw.boot += Duration::from_millis(50);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path)).unwrap();
        let ceiling = cfg.speakers[0].t_limit - cfg.speakers[0].t_headroom;
        while hw.pings.is_empty() && hw.t < secs(30.0) {
            g.tick(&mut hw).unwrap();
        }
        assert!(!hw.pings.is_empty(), "never rearmed");
        assert!(g.speakers[0].t_coil < ceiling, "rearmed at {:.2} C", g.speakers[0].t_coil);
    }

    /// Found porting PR #6 (and the PR #7 r3 review's T2): time is charged
    /// at the least reduction held since the last verified boundary, so a
    /// hand-off never lowers the charge for time before or after it.  0.1.6
    /// charged 150 ms played at 0 dB at the 20 dB its hand-off had just
    /// written.
    #[test]
    fn time_around_a_hand_off_is_charged_at_the_gain_it_may_have_played_at() {
        let cfg = j700();
        let ta = cfg.globals.t_ambient;
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        g.rate = 48000;
        g.armed = true;
        g.written = 0.0;
        let t0 = hw.now();
        g.gains = vec![(t0, 0.0)];
        g.verified_at = t0;
        g.account_to = t0;
        hw.t += Duration::from_millis(150);
        g.disarm(&mut hw, "test").unwrap();
        assert_eq!(hw.relinquished, 1, "the hand-off gave the lock up");
        let before = g.speakers[0].clone();
        g.charge(hw.now()).unwrap();
        let mut want = before.clone();
        want.step_at(want.worst_case_power(0.0), 0.15, ta, 0.0);
        assert!(g.speakers[0].t_coil >= want.t_coil - 1e-9, "charged {:.3} C, at the gain it played at {:.3} C", g.speakers[0].t_coil, want.t_coil);
        // after it too, until a verified boundary
        hw.t += Duration::from_millis(100);
        let before = g.speakers[0].clone();
        g.charge(hw.now()).unwrap();
        let mut want = before.clone();
        want.step_at(want.worst_case_power(0.0), 0.1, ta, 0.0);
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
            let pending = hw.now().saturating_duration_since(g.account_to).as_secs_f64();
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

    // ---- 0.1.8: the PR #7 third review's witnesses (r3), as safe outcomes ----

    /// A control-event wait that resumes only at its timeout (a daemon
    /// scheduled late): the event is still queued, but its wake comes late.
    struct DelayedWait {
        hw: FakeHw,
    }
    impl Hw for DelayedWait {
        fn now(&self) -> Instant { self.hw.now() }
        fn boottime(&self) -> Duration { self.hw.boottime() }
        fn sleep(&mut self, d: Duration) { self.hw.sleep(d) }
        fn take_lock(&mut self) -> io::Result<()> { self.hw.take_lock() }
        fn ping(&mut self) -> io::Result<()> { self.hw.ping() }
        fn set_min(&mut self) -> io::Result<()> { self.hw.set_min() }
        fn set_reduction(&mut self, d: f64) -> io::Result<()> { self.hw.set_reduction(d) }
        fn relinquish(&mut self) -> io::Result<()> { self.hw.relinquish() }
        fn sample_rate(&mut self) -> io::Result<i64> { self.hw.sample_rate() }
        fn open_sense(&mut self, r: u32) -> io::Result<()> { self.hw.open_sense(r) }
        fn close_sense(&mut self) { self.hw.close_sense() }
        fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> { self.hw.read_period(w) }
        fn queued(&mut self) -> io::Result<u64> { self.hw.queued() }
        fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> { self.hw.read_tail(w) }
        fn rate_events(&mut self) -> bool { self.hw.rate_events() }
        fn events_available(&self) -> bool { self.hw.events_available() }
        fn wait_rate_change(&mut self, d: Duration) -> bool {
            let before = self.hw.t;
            self.hw.sleep(d);
            self.hw.next_change(before, self.hw.t).is_some()
        }
    }

    /// The model against one charged with what the card really let through:
    /// full scale wherever the stream played, at the card's real gain.
    fn charged_reference(reference: &mut SpeakerState, cfg: &Config, from: Duration, to: Duration, playing: &dyn Fn(Duration) -> bool, gain_db: f64) {
        let mut at = from;
        while at < to {
            let d = Duration::from_micros(500).min(to - at);
            let p = if playing(at) { reference.worst_case_power(gain_db) } else { 0.0 };
            reference.step(p, d.as_secs_f64(), cfg.globals.t_ambient);
            at += d;
        }
        reference.govern(&cfg.globals);
    }

    /// Review r3 T1: a stream that opens and closes while the daemon's
    /// event wake is late was erased whole: 0.1.6 took the late wake as the
    /// stream's start and charged nothing (0 opens, 30 pings, 0 dB, 50 C
    /// against a reference that latched at 826 ms).  Now everything since
    /// the last look at closed speakers is charged when an event is seen.
    #[test]
    fn a_late_event_wake_charges_the_whole_interval_since_the_last_look() {
        let cfg = j700();
        let playing = |t: Duration| (10..90).contains(&(t.as_millis() % 100));
        let mut inner = FakeHw::new(2, 4096);
        inner.playing = Box::new(playing);
        inner.level = Box::new(|_| 1.0); // full scale, as the reference assumes
        let mut hw = DelayedWait { hw: inner };
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        let mut reference = g.speakers[0].clone();
        // the reference runs to the model's own account (which trails the
        // clock by up to a period plus the slack), at the gain the card had
        let t0 = hw.hw.t0;
        let mut ref_to = g.account_to - t0;
        for _ in 0..120 {
            let gain = hw.hw.effective_reduction();
            g.tick(&mut hw).unwrap();
            let to = g.account_to - t0;
            charged_reference(&mut reference, &cfg, ref_to, to, &playing, gain);
            ref_to = to;
            assert!(g.speakers[0].t_coil >= reference.t_coil - 0.5, "at {:?}: model {:.2} C, charged reference {:.2} C", to, g.speakers[0].t_coil, reference.t_coil);
        }
    }

    /// Review r3 T1, without events: nothing between two looks is known, so
    /// the lock is never taken.  0.1.6 polled and renewed at 0 dB.
    #[test]
    fn without_control_events_the_lock_is_never_taken() {
        let cfg = j700();
        let mut hw = FakeHw::new(2, 4096);
        hw.events = false;
        hw.playing = Box::new(|t| (10..90).contains(&(t.as_millis() % 100)));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(3.0)).unwrap();
        assert!(hw.pings.is_empty() && !g.armed(), "{} pings", hw.pings.len());
        assert!(hw.effective_reduction() >= SAFE_DB);
    }

    /// Review r3 T2: a clean stop with time not yet accounted (the sense
    /// stalled) charged it as safe-level history on restore (100.0 C where
    /// 107.2 C was needed, then 0 dB after two quiet periods).  The stop now
    /// gives the lock up, waits out the drain and accounts everything before
    /// the final write.
    #[test]
    fn a_clean_stop_accounts_everything_before_its_final_state() {
        let cfg = j700();
        let mut hw = FakeHw::new(2, 4096);
        hw.level = Box::new(|_| 0.0);
        let path = tmp_state().unwrap();
        let warm: Vec<_> = cfg.speakers.iter().map(|s| { let mut x = SpeakerState::new(s.clone(), 50.0); x.resume(100.0, 100.0, 0.0, 0.0); x }).collect();
        std::fs::write(&path, state::encode(hw.boot, hw.boot, &warm, false, true)).unwrap();
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path.clone())).unwrap();
        run_until(&mut g, &mut hw, secs(1.0)).unwrap();
        hw.sensing = Box::new(|_| false);
        g.tick(&mut hw).unwrap();
        let lag = hw.now() - g.account_to;
        assert!(lag >= secs(0.049));
        let mut reference = g.speakers[0].clone();
        reference.step_at(reference.worst_case_power(0.0), lag.as_secs_f64(), cfg.globals.t_ambient, 0.0);
        g.stop(&mut hw).unwrap();
        while !g.released(&hw) {
            g.tick(&mut hw).unwrap();
        }
        g.final_save(&hw).unwrap();
        let saved = state::decode(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert!(saved.released && saved.covered == saved.boottime);
        let mut restored = g.speakers.clone();
        assert!(state::restore(&mut restored, &saved, hw.boottime(), &cfg.globals));
        // the drain after the stop is charged too, so the restored coil is at
        // least the reference charged only with the pending interval, cooled
        // over the same time at most
        assert!(restored[0].t_coil >= reference.t_coil - 2.0, "restored {:.3} C, the pending interval alone needs {:.3} C", restored[0].t_coil, reference.t_coil);
        assert!(g.speakers[0].t_coil >= reference.t_coil - 2.0);
    }

    /// Review r3 T4: a held snapshot is not resumed: whatever the kernel's
    /// paused lease let play before the daemon died is not bounded by
    /// anything the snapshot can say.  0.1.6 restored 87.4 C where late
    /// playback could have left 112.2 C, and rearmed at 0 dB.
    #[test]
    fn a_held_snapshot_restarts_latched() {
        let cfg = j700();
        let warm: Vec<_> = cfg.speakers.iter().map(|s| { let mut x = SpeakerState::new(s.clone(), 50.0); x.resume(100.0, 100.0, 0.0, 0.0); x }).collect();
        let path = tmp_state().unwrap();
        std::fs::write(&path, state::encode(secs(1000.0), secs(1000.0), &warm, true, false)).unwrap();
        let mut hw = FakeHw::new(2, 4096);
        hw.boot = secs(1060.0);
        hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path)).unwrap();
        assert!(g.speakers.iter().all(|s| s.hard && s.t_coil >= s.spec.t_limit));
        let mut reference = warm[0].clone();
        reference.step(0.0, 59.76, 50.0);
        reference.step_at(reference.worst_case_power(0.0), 0.24, 50.0, 0.0);
        while hw.pings.is_empty() {
            g.tick(&mut hw).unwrap();
        }
        reference.step(0.0, hw.t.as_secs_f64(), 50.0);
        reference.govern(&cfg.globals);
        assert!(hw.reduction >= reference.reduction_db - 1e-6, "rearmed at {} dB, the late playback needs {} dB", hw.reduction, reference.reduction_db);
    }

    /// Review r3 T6: frames over time already charged at the worst case
    /// must not advance thermal time again.  0.1.6 integrated them in full
    /// and cooled a latched 123 C coil to 119.8 C (clearing the latch) where
    /// the true elapsed time left 121.0 C.
    #[test]
    fn frames_over_charged_time_add_nothing() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        g.rate = 48000;
        g.written = 20.0;
        let t0 = hw.now();
        g.gains = vec![(t0, 20.0)];
        g.verified_at = t0;
        g.account_to = t0;
        for s in &mut g.speakers {
            s.resume(123.0, 100.0, 0.0, 0.0);
            s.hard = true;
        }
        hw.sleep(secs(0.3));
        g.charge(hw.now()).unwrap();
        let after_charge = g.speakers[0].t_coil;
        for w in g.words.iter_mut() {
            *w = 0;
        }
        // three quiet periods whose wall spans were all charged already
        for i in 1..=3 {
            g.measure(t0 + secs(i as f64 * 4096.0 / 48000.0), 4096).unwrap();
        }
        assert_eq!(g.speakers[0].t_coil, after_charge, "already charged time was integrated again");
        assert!(!g.speakers[0].clear_hard_if_cool());
    }

    /// The J700's copy-time gain (an independent abstraction of the public
    /// driver): frames converted before a gain change keep their old gain
    /// for up to 341 ms, unless the card's lock is given up.
    struct CopyQueue {
        hw: FakeHw,
        until: Duration,
        old_db: f64,
        fail_relinquish: bool,
    }
    impl CopyQueue {
        fn output_db(&self) -> f64 {
            if self.hw.t < self.until && self.hw.unlocked_at(self.hw.t) { self.old_db } else { self.hw.effective_reduction() }
        }
    }
    impl Hw for CopyQueue {
        fn now(&self) -> Instant { self.hw.now() }
        fn boottime(&self) -> Duration { self.hw.boottime() }
        fn sleep(&mut self, d: Duration) { self.hw.sleep(d) }
        fn take_lock(&mut self) -> io::Result<()> { self.hw.take_lock() }
        fn ping(&mut self) -> io::Result<()> { self.hw.ping() }
        fn set_min(&mut self) -> io::Result<()> { self.hw.set_min() }
        fn set_reduction(&mut self, d: f64) -> io::Result<()> { self.hw.set_reduction(d) }
        fn relinquish(&mut self) -> io::Result<()> {
            if self.fail_relinquish {
                return Err(io::Error::from_raw_os_error(libc::EIO));
            }
            self.until = self.hw.t;
            self.hw.relinquish()
        }
        fn sample_rate(&mut self) -> io::Result<i64> { self.hw.sample_rate() }
        fn open_sense(&mut self, r: u32) -> io::Result<()> { self.hw.open_sense(r) }
        fn close_sense(&mut self) { self.hw.close_sense() }
        fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> { self.hw.read_period(w) }
        fn queued(&mut self) -> io::Result<u64> { self.hw.queued() }
        fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> { self.hw.read_tail(w) }
        fn rate_events(&mut self) -> bool { self.hw.rate_events() }
        fn events_available(&self) -> bool { self.hw.events_available() }
        fn wait_rate_change(&mut self, d: Duration) -> bool { self.hw.wait_rate_change(d) }
    }

    /// Review r3 T3: a hand-off by a safe-level write left samples converted
    /// at 0 dB playing, while 0.1.6 charged 20 dB at once (50.3 C against
    /// 78.4 C).  A hand-off now gives the lock up (macaudio stops the old
    /// streams), and the account keeps the old gain until a verified
    /// boundary.
    #[test]
    fn a_hand_off_gives_the_lock_up_and_charges_the_old_gain() {
        let cfg = j700();
        let mut hw = CopyQueue { hw: FakeHw::new(2, 4096), until: Duration::ZERO, old_db: 0.0, fail_relinquish: false };
        hw.hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        while hw.hw.t < secs(1.0) {
            g.tick(&mut hw).unwrap();
        }
        let mut reference = g.speakers[0].clone();
        hw.until = hw.hw.t + secs(0.2);
        g.disarm(&mut hw, "synthetic hand-off").unwrap();
        assert_eq!(hw.hw.relinquished, 1);
        assert!(hw.output_db() >= SAFE_DB, "the old gain still plays after the hand-off");
        hw.hw.rate_fail = Box::new(|_| true);
        let from = g.account_to;
        for _ in 0..3 {
            g.tick(&mut hw).unwrap();
        }
        reference.step(reference.worst_case_power(0.0), (g.account_to - from).as_secs_f64(), 50.0);
        assert!(g.speakers[0].t_coil >= reference.t_coil - 0.5, "model {:.2} C, the old gain needs {:.2} C", g.speakers[0].t_coil, reference.t_coil);
    }

    /// Review r3 T8: a stop whose hand-off fails is an error the caller
    /// exits on (closing the control file gives the lock up); 0.1.6 logged
    /// it and kept ticking unreleased.
    #[test]
    fn a_failed_stop_is_an_error() {
        let cfg = j700();
        let mut hw = CopyQueue { hw: FakeHw::new(2, 4096), until: Duration::ZERO, old_db: 0.0, fail_relinquish: true };
        hw.hw.playing = Box::new(|_| false);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        g.tick(&mut hw).unwrap();
        assert!(g.armed());
        assert!(g.stop(&mut hw).is_err(), "a stop that could not give the lock up returned success");
        assert!(!g.released(&hw));
    }

    /// Review r3 T6 through the loop: a close seen late, with its queued
    /// frames read after the time was charged, does not clear a latch the
    /// true elapsed time still holds.
    #[test]
    fn a_late_close_with_queued_frames_does_not_cool_twice() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(1.0)).unwrap();
        for s in &mut g.speakers {
            s.resume(123.0, 100.0, 0.0, 0.0);
            s.hard = true;
        }
        let start = hw.t;
        let mut reference = g.speakers[0].clone();
        // the daemon is away 300 ms; the stream closed meanwhile
        let close = start + secs(0.3);
        hw.playing = Box::new(move |t| t < close);
        hw.sleep(secs(0.3));
        while hw.t < start + secs(3.0) {
            g.tick(&mut hw).unwrap();
            if g.armed() {
                break;
            }
        }
        reference.step(0.0, (hw.t - start).as_secs_f64(), 50.0);
        assert!(g.speakers[0].t_coil >= reference.t_coil - 1e-6, "model {:.3} C below the true elapsed time's {:.3} C", g.speakers[0].t_coil, reference.t_coil);
        assert!(!(g.armed() && reference.t_coil > 120.0), "rearmed while the true coil is still over the ceiling");
    }
}
