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
//!   playing samples converted at an earlier gain.  While the card holds
//!   its lock (taken at start or given up, and not renewed since), once
//!   what it converted before has drained, that reduction is at least the
//!   card's safe level.  No partial credits, no tolerances, no means, no
//!   assumption about when inside an uncertain interval anything played.
//!   Frames over time already charged add nothing.
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
//!   latched, and so does every state while another protection daemon
//!   (speakersafetyd) has run this boot: its flag file says it may have
//!   driven the speakers since.
//! * The engine's models (engine.rs: the asahi governor on the machines with
//!   V/I sense, the unified one when it governs or the shadow log runs) get
//!   exactly this account: the same measured frames over the same time, the
//!   same charges at each speaker's least reduction since the last verified
//!   boundary, the same cooling.  Whichever model governs, its reductions
//!   are written per speaker and its hard latch hands the card to the
//!   kernel.  Under the asahi guard (the unified governor only) each
//!   speaker gets the larger of the two governors' reductions, and either
//!   one's latch hands the card over.
//! * Sense data that is not a measurement (a mean V*I below -0.01 W, or
//!   nonfinite) is uncovered time: charged at full scale, never measured.
//!   The card is handed off at once and REARM_PERIODS good periods in a row
//!   are needed to take it back.
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

use crate::config::{Config, Globals, Governor};
use crate::engine::Engine;
use crate::model::{self, SpeakerState, SUB_BLOCKS};
use crate::record::{self, Recorder};
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
/// A change event can reach the daemon this long after the change: a look
/// at closed speakers vouches only for the time up to this before it (the
/// rest is the next look's to vouch for or charge).
pub const EVENT_SLACK: Duration = Duration::from_millis(5);
/// How much longer than the queue the controls' history is kept before the
/// last verified boundary: frames measured partly over the account reach
/// back up to one frame before it.
const GAIN_RETAIN: Duration = Duration::from_secs(1);

/// What each speaker's control held from `at` on: `lo` and `hi` the least
/// and the most (dB; infinite = unknown), `cands` the values it may hold
/// between them, each up to a step wide (empty: anything from `lo` to
/// `hi`), `kernel` whether the card's safe level may have applied instead.
#[derive(Debug, Clone, PartialEq)]
pub struct Gain {
    pub at: Instant,
    pub lo: Vec<f64>,
    pub hi: Vec<f64>,
    pub kernel: bool,
    pub cands: Vec<Vec<(f64, f64)>>,
}

/// A control holds at most this many candidate values; past it the two
/// closest merge into the span between them, which holds both, so no value
/// is lost.  A merged span wider than PREDICT_SPREAD_DB leaves its frames
/// out of the unified regulator's prediction (the budgets still read them
/// at the most).  The guard does not reach the cap today: every failed
/// write hands the card off, and every retake starts from the minimum, so
/// a set holds at most two values.  A change that lets failures pile up in
/// one armed stretch should raise the cap or keep a merged span's frames
/// (review r4 addendum 3 of speakerguardd#8).
const CANDIDATES_MAX: usize = 4;

/// Add the range `r` to a control's candidate values, merged with any it
/// overlaps.
fn add_candidate(set: &mut Vec<(f64, f64)>, r: (f64, f64)) {
    set.push(r);
    set.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut out: Vec<(f64, f64)> = Vec::with_capacity(set.len());
    for &(a, b) in set.iter() {
        match out.last_mut() {
            Some(last) if a <= last.1 => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    while out.len() > CANDIDATES_MAX {
        let i = (0..out.len() - 1).min_by(|&i, &j| (out[i + 1].0 - out[i].1).total_cmp(&(out[j + 1].0 - out[j].1))).unwrap();
        out[i].1 = out[i].1.max(out[i + 1].1);
        out.remove(i + 1);
    }
    *set = out;
}

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
    /// Write the gain reduction (dB below the control's maximum).  Returns
    /// the reduction the card applies: the control's steps round it up
    /// (review r5 V6).
    fn set_reduction(&mut self, db: f64) -> io::Result<f64>;
    /// Per-speaker reductions, for cards with a volume control per speaker;
    /// a card with one shared control gets the largest.  A failure may
    /// leave some controls at the new value and some at the old one.
    /// Returns what each speaker's control applies.
    fn set_reductions(&mut self, db: &[f64]) -> io::Result<Vec<f64>> {
        let applied = self.set_reduction(db.iter().cloned().fold(0.0, f64::max))?;
        Ok(vec![applied; db.len()])
    }
    /// The controls' step (dB): a request is applied rounded up to it.
    fn step_db(&self) -> f64 {
        0.0
    }
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

#[cfg(test)]
thread_local! {
    /// Tests: every state file write and removal fails.
    pub static STATE_IO_FAILS: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

fn state_io_fails() -> io::Result<()> {
    #[cfg(test)]
    if STATE_IO_FAILS.with(|f| f.get()) {
        return Err(io::Error::from_raw_os_error(libc::EROFS));
    }
    Ok(())
}

/// Write the state file atomically (a temporary file renamed over it).
fn state_write(path: &std::path::Path, text: &str) -> io::Result<()> {
    state_io_fails()?;
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, text).and_then(|()| std::fs::rename(&tmp, path))
}

fn state_remove(path: &std::path::Path) -> io::Result<()> {
    state_io_fails()?;
    std::fs::remove_file(path)
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
/// Where speakersafetyd notes that it has run since boot (its "warm boot"
/// flag: /run/speakersafetyd/speakersafetyd.flag from 2.0, written into its
/// runtime directory, /run/speakersafetyd.flag before; the ref Macs run
/// 1.0.2 and 2.0.1).  Only its presence counts; its absence proves nothing,
/// as 2.0's runtime directory goes when the service stops.
pub const FOREIGN_FLAGS: &[&str] = &["/run/speakersafetyd/speakersafetyd.flag", "/run/speakersafetyd.flag"];
/// The shadow log is rotated (to `<file>.1`) past this size.
const SHADOW_MAX_BYTES: u64 = 16 << 20;

/// How a span of the account was accounted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Accounted {
    /// Covered by measured sense frames.
    Measured,
    /// Not covered: charged as a full-scale signal.
    Charged,
    /// Verified closed speakers.
    Cooled,
}

/// The decisions a replay compares with the live run's, and its account.
#[derive(Debug, Clone, PartialEq)]
pub enum Trace {
    /// The lock taken (true) or the card handed to the kernel, and why.
    Permit(bool, String),
    /// The governing model's hard latch set or cleared.
    Latch(bool),
    /// A span of the account (boot times) and how it was accounted.
    Account(Duration, Duration, Accounted),
}

/// Record at the boot time now, read only while a recording runs (a clock
/// read is a call that can block or be descheduled like any other).
fn rec_now<H: Hw>(rec: &mut Option<Recorder>, hw: &H, f: impl FnOnce(&mut Recorder, u64) -> io::Result<()>) {
    if rec.is_some() {
        rec_do(rec, hw.boottime(), f);
    }
}

/// One record, or the recording stops (a write failure, its size limit).
/// Recording never decides anything: a blocked write only delays the loop,
/// which the account charges like any other lateness.
fn rec_do(rec: &mut Option<Recorder>, t: Duration, f: impl FnOnce(&mut Recorder, u64) -> io::Result<()>) {
    let Some(r) = rec.as_mut() else { return };
    let ns = t.as_nanos() as u64;
    match f(r, ns) {
        Ok(()) if r.full() => {
            let _ = r.end(ns);
            info!("recording reached its size limit; stopped");
            *rec = None;
        }
        Ok(()) => {}
        Err(e) => {
            warn!("recording: {e}; stopped");
            *rec = None;
        }
    }
}

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
    /// A change event was seen after the last look's sample-rate read: the
    /// time after that look is not known to be quiet.
    event_pending: bool,
    /// This run has invalidated the clean stop it may have resumed (the
    /// state file says released 0, or there is none): needed before any
    /// unlock, so that a later crash cannot resume an older clean stop.
    invalidated: bool,
    invalidate_failed: bool,
    /// The least reduction the card can have converted samples at while
    /// the controls' value was unknown (dB).
    unknown_floor: f64,
    /// Complete periods in a row.
    fresh: u32,
    words: Vec<i32>,
    last_ping: Instant,
    last_log: Instant,
    /// When the governed reduction was last written.
    last_write: Instant,
    /// The reduction each speaker's volume control holds, as far as the
    /// daemon knows (dB; infinite = unknown, taken as full gain): what the
    /// card applies, its steps' rounding included; the least of it after a
    /// failed write.
    written: Vec<f64>,
    /// The most each control may hold (after a failed write, the new value
    /// rounded up by a step, or the old one).
    written_hi: Vec<f64>,
    /// The values each control may hold: one after a write that worked,
    /// the old ones and the new one (up to a step above it) after one that
    /// failed, since each control's write is atomic (review r4 CC1 of
    /// speakerguardd#8).  Empty while unknown.
    written_set: Vec<Vec<(f64, f64)>>,
    /// The reductions last asked for.
    requested: Vec<f64>,
    applied: f64,
    /// What the controls held, oldest first, back to QUEUE_BOUND +
    /// GAIN_RETAIN before the last verified boundary.  The card applies its
    /// safe level whenever the daemon does not hold a live lease.
    gains: Vec<Gain>,
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
    /// Since when the card has certainly held its lock (taken at start, or
    /// given up and taken again) with no renewal since: from then on it
    /// applies at least its safe level to whatever it converts.  None from
    /// the first renewal on.
    locked_at: Option<Instant>,
    /// The engine's models: the asahi ones on machines with V/I sense, the
    /// unified ones when they govern or the shadow log runs.  None on the
    /// J700's default (legacy, no shadow).
    pub engine: Option<Engine>,
    /// The governor whose reductions are written.
    pub governing: Governor,
    shadow_path: Option<PathBuf>,
    shadow_failed: bool,
    /// The last sense data was not a measurement (one warning per run).
    bad_data: bool,
    /// Bad sense data was seen and REARM_PERIODS valid complete periods in
    /// a row have not followed yet: the lock is not taken, idle or not
    /// (review P8-2).  Carried by a clean stop's state.
    recovering: bool,
    /// Valid complete periods in a row since the last bad data.
    good_since_bad: u32,
    /// What each control held over the last measured frames (for the
    /// recording's rebuilt program).
    last_applied: Vec<f64>,
    /// The recording (--record), and the replay's trace.
    pub rec: Option<Recorder>,
    pub trace: Option<Vec<(Duration, Trace)>>,
    /// Boot time at the guard's `Instant` zero (for the trace).
    boot_at: Option<(Instant, Duration)>,
    /// The last sample-rate read recorded (rate or -errno).
    rec_rate: Option<i64>,
    /// The governing latch as last recorded.
    last_hard: bool,
    /// When the lock was last taken (None while it is not held).
    armed_at: Option<Instant>,
}

impl Guard {
    /// Restore the model from a clean stop (if there is one) and lock the
    /// card's controls.  The volume lock is not lifted here.
    /// speakersafetyd's flag files are consulted (FOREIGN_FLAGS).
    pub fn start<H: Hw>(hw: &mut H, cfg: &Config, max_reduction: f64, state_path: Option<PathBuf>) -> Result<Guard, Box<dyn Error>> {
        // tests run on machines that may have speakersafetyd: they name
        // their flag files themselves
        let foreign: Vec<PathBuf> = if cfg!(test) { Vec::new() } else { FOREIGN_FLAGS.iter().map(PathBuf::from).collect() };
        Self::start_with(hw, cfg, max_reduction, state_path, &foreign)
    }

    /// `start`, with the flag files of other protection daemons to consult.
    pub fn start_with<H: Hw>(hw: &mut H, cfg: &Config, max_reduction: f64, state_path: Option<PathBuf>, foreign: &[PathBuf]) -> Result<Guard, Box<dyn Error>> {
        if !max_reduction.is_finite() || max_reduction <= 0.0 {
            return Err("maximum reduction must be finite and positive".into());
        }
        // the unified governor's opt-in, an unqualified machine's, and the
        // asahi governor's sense, before the card is touched
        cfg.check_governor()?;
        let globals = cfg.globals.clone();
        let governing = globals.governor;
        let mut engine = Engine::new(cfg, governing == Governor::Unified || globals.shadow);
        if governing != Governor::Legacy && !engine.as_ref().is_some_and(|e| e.has(governing)) {
            return Err(format!("governor {} has no models on this machine", governing.name()).into());
        }
        if let Some(guard) = globals.guard {
            if !engine.as_ref().is_some_and(|e| e.has(guard)) {
                return Err(format!("guard {} has no models on this machine", guard.name()).into());
            }
        }
        let mut speakers: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), globals.t_ambient)).collect();
        let latched = |speakers: &mut Vec<SpeakerState>, engine: &mut Option<Engine>| {
            for s in speakers.iter_mut() {
                s.assume_latched(globals.t_ambient);
            }
            if let Some(e) = engine.as_mut() {
                e.assume_latched();
            }
        };
        // what the card applied before this run: after a clean stop the
        // last owner gave the lock up and drained before writing it, so the
        // card has applied at least its own safe level since; otherwise a
        // dead owner's gain may still be playing
        // The card first: its lock is what makes this run the only owner,
        // so the state is read and consumed under it (review r5 V2).
        hw.take_lock()?;
        let events = hw.events_available();
        if !events {
            warn!("no control events from the card: streams between two looks cannot be accounted; leaving the speakers at the card's safe level");
        }
        // The controls' value before this run is unknown, and the budgets
        // must count samples converted at it as loud as they can have been:
        // with the controls at their minimum from here on that lasts the
        // copy queue's half second, not until the first unlock (on a
        // machine whose budgets need more than --max-reduction for a
        // full-scale signal, never).  The minimum is the one value the
        // card accepts while locked; the speakers are silent until the
        // first unlock.  Without events there is no unlock: they stay at
        // the card's safe level.
        let min_now = events && match hw.set_min() {
            Ok(()) => true,
            Err(e) => {
                warn!("volume control: {e}; the controls' value stays unknown");
                false
            }
        };
        let mut resumed = false;
        let mut recovering = false;
        let mut restored_at = None;
        match &state_path {
            None => {
                warn!("no model state file: assuming hot speakers (latched)");
                latched(&mut speakers, &mut engine);
            }
            Some(path) => match std::fs::read_to_string(path) {
                Ok(text) => match state::decode(&text) {
                    Some(saved) => {
                        let at = hw.boottime();
                        if let Some(e) = engine.as_mut() {
                            state::restore_engine(e, &saved, at, &globals);
                        }
                        if state::restore(&mut speakers, &saved, at, &globals) {
                            resumed = true;
                            restored_at = Some(at);
                            recovering = saved.recovering;
                            info!("model restored from {} (a clean stop {:.1} s ago)", path.display(), at.saturating_sub(saved.boottime).as_secs_f64());
                        } else {
                            info!("{}: not from a clean stop; assuming hot speakers (latched)", path.display());
                        }
                    }
                    None => {
                        warn!("{}: unreadable model state; assuming hot speakers (latched)", path.display());
                        latched(&mut speakers, &mut engine);
                    }
                },
                Err(e) if e.kind() == io::ErrorKind::NotFound => {
                    info!("{}: no model state; assuming hot speakers (latched)", path.display());
                    latched(&mut speakers, &mut engine);
                }
                Err(e) => {
                    warn!("{}: {e}; assuming hot speakers (latched)", path.display());
                    latched(&mut speakers, &mut engine);
                }
            },
        }
        // another protection daemon ran this boot: it may have driven the
        // speakers after this daemon's last state (or before its first)
        for p in foreign {
            let present = match std::fs::symlink_metadata(p) {
                Ok(_) => true,
                Err(e) => e.kind() != io::ErrorKind::NotFound,
            };
            if present {
                info!("{}: speakersafetyd has run since boot; assuming hot speakers (latched)", p.display());
                latched(&mut speakers, &mut engine);
                resumed = false;
                break;
            }
        }
        for s in speakers.iter_mut() {
            s.govern(&globals);
            if !s.finite() {
                return Err("nonfinite speaker model state".into());
            }
        }
        // the governor's output before anything may unlock (review F2)
        if let Some(e) = engine.as_mut() {
            e.govern(0.0, false);
            if !e.finite() {
                return Err("nonfinite speaker model state".into());
            }
        }
        if governing != Governor::Legacy {
            let guard = globals.guard.map_or(String::new(), |g| format!(" under the {} guard", g.name()));
            info!("governor {}{guard}{}", governing.name(), if globals.shadow { ", the others in shadow" } else { "" });
        }
        let shadow_path = if globals.shadow { state_path.as_ref().map(|p| p.with_file_name("shadow.jsonl")) } else { None };
        let n = speakers.len();
        let now = hw.now();
        if let Some(at) = restored_at {
            // The restore covers up to its boot time, the account starts at
            // this monotonic time: whatever lay between them is charged at a
            // full-scale signal at full gain, as long as the two boot-time
            // reads around it lay apart (as in the final save).
            let pairing = hw.boottime().saturating_sub(at).as_secs_f64();
            if pairing > 0.0 {
                for s in speakers.iter_mut() {
                    let full = s.worst_case_power(0.0);
                    s.step_pre(full, full, pairing, globals.t_ambient);
                    s.govern(&globals);
                }
                if let Some(e) = engine.as_mut() {
                    e.worst(pairing, &vec![0.0; n]);
                }
            }
        }
        let unknown_floor = if resumed { globals.safe_reduction } else { 0.0 };
        let mut g = Guard {
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
            event_pending: false,
            invalidated: false,
            invalidate_failed: false,
            unknown_floor,
            fresh: 0,
            last_ping: now,
            last_log: now,
            last_write: now,
            written: vec![if min_now { MUTE_DB } else { f64::INFINITY }; n],
            written_hi: vec![if min_now { MUTE_DB } else { f64::INFINITY }; n],
            written_set: vec![if min_now { vec![(MUTE_DB, MUTE_DB)] } else { Vec::new() }; n],
            requested: vec![f64::NAN; n],
            applied: 0.0,
            gains: {
                let unknown = Gain { at: now, lo: vec![f64::INFINITY; n], hi: vec![f64::INFINITY; n], kernel: true, cands: vec![Vec::new(); n] };
                if min_now { vec![unknown, Gain { at: now, lo: vec![MUTE_DB; n], hi: vec![MUTE_DB; n], kernel: true, cands: vec![vec![(MUTE_DB, MUTE_DB)]; n] }] } else { vec![unknown] }
            },
            verified_at: now,
            rate_failures: 0,
            min_failures: 0,
            target_failures: 0,
            state_path,
            last_save_try: None,
            save_failed: false,
            stopping: false,
            relinquished_at: None,
            // the lock taken above: macaudio clears any lease with it
            locked_at: Some(now),
            engine,
            governing,
            shadow_path,
            shadow_failed: false,
            bad_data: false,
            recovering,
            good_since_bad: 0,
            last_applied: vec![0.0; n],
            rec: None,
            trace: None,
            boot_at: None,
            rec_rate: None,
            last_hard: false,
            armed_at: None,
        };
        // a clean stop is resumed once: gone before this run can unlock
        g.invalidated = g.invalidate(hw);
        Ok(g)
    }

    /// Replace the state file with this run's status (released 0), or
    /// remove it.  True once no clean stop is left for a later start.
    fn invalidate<H: Hw>(&mut self, hw: &H) -> bool {
        let Some(path) = self.state_path.clone() else { return true };
        if self.save_now(hw) {
            return true;
        }
        match state_remove(&path) {
            Ok(()) => true,
            Err(e) if e.kind() == io::ErrorKind::NotFound => true,
            Err(e) => {
                if !self.invalidate_failed {
                    warn!("{}: {e}; an earlier clean stop could not be invalidated, so the lock is not taken", path.display());
                    self.invalidate_failed = true;
                }
                false
            }
        }
    }

    /// For the recording: the most reduction each control may have applied
    /// over [from, to).  Unless the lock was held all through it the card
    /// may have been at its safe level, the larger reduction (the louder
    /// program rebuilt from the sensed words, the safer guess there).
    fn applied_over(&self, from: Instant, most: &[f64]) -> Vec<f64> {
        let held_through = self.armed && self.armed_at.is_some_and(|t| t <= from);
        let safe = self.globals.safe_reduction;
        most.iter().map(|&m| if held_through { m } else { m.max(safe) }).collect()
    }

    /// Start recording: the header and the model this guard starts from
    /// (what a replay starts from: written as a clean stop's state at this
    /// instant, which is how the replay resumes it unchanged).
    pub fn attach_recorder<H: Hw>(&mut self, hw: &H, rec: Recorder) {
        let boot = hw.boottime();
        let text = state::encode_with(boot, boot, &self.speakers, self.engine.as_ref(), false, true);
        let mut slot = Some(rec);
        let (gov, guard, max) = (self.governing, self.globals.guard, self.max_reduction);
        rec_do(&mut slot, boot, |r, t| r.start(t, gov, guard, max, &text));
        self.rec = slot;
        self.last_hard = self.governing_state().2;
        // the account's spans are recorded in boot time
        if self.boot_at.is_none() {
            self.boot_at = Some((hw.now(), boot));
        }
    }

    /// Keep a trace of decisions and of the account (the replay).
    pub fn start_trace<H: Hw>(&mut self, hw: &H) {
        self.trace = Some(Vec::new());
        self.boot_at = Some((hw.now(), hw.boottime()));
        self.last_hard = self.governing_state().2;
    }

    /// Whether the daemon renews the card's lock (for the replay's log).
    pub fn armed_now(&self) -> bool {
        self.armed
    }

    /// The governor whose reductions the governing one's are raised to.
    #[cfg(test)]
    pub fn guard(&self) -> Option<Governor> {
        self.globals.guard
    }

    /// A guard `Instant` as boot time (the trace's clock).
    fn boot_of(&self, t: Instant) -> Duration {
        match self.boot_at {
            Some((i, b)) if t >= i => b + (t - i),
            Some((i, b)) => b.saturating_sub(i - t),
            None => Duration::ZERO,
        }
    }

    /// A span of the account, for the trace and the recording.
    fn note_account(&mut self, from: Instant, to: Instant, how: Accounted) {
        if to > from && (self.trace.is_some() || self.rec.is_some()) {
            let (a, b) = (self.boot_of(from), self.boot_of(to));
            if let Some(tr) = self.trace.as_mut() {
                tr.push((b, Trace::Account(a, b, how)));
            }
            rec_do(&mut self.rec, b, |r, ns| r.account(ns, a.as_nanos() as u64, how));
        }
    }

    /// The lock was taken or given up.
    fn note_permit<H: Hw>(&mut self, hw: &H, armed: bool, why: &str) {
        if self.trace.is_none() && self.rec.is_none() {
            return;
        }
        let t = hw.boottime();
        if let Some(tr) = self.trace.as_mut() {
            tr.push((t, Trace::Permit(armed, why.to_string())));
        }
        rec_do(&mut self.rec, t, |r, ns| r.permit(ns, armed, why));
    }

    #[cfg(test)]
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Up to where the model has accounted (tests).
    #[cfg(test)]
    pub fn account_to(&self) -> Instant {
        self.account_to
    }

    /// Stop renewing the lock for good (SIGTERM): give the card's lock up.
    /// The loop keeps accounting until `released`, then the caller writes
    /// the final state (`final_save`) and exits.  An error here leaves the
    /// state as it is (not a clean stop) and the caller exits at once.
    pub fn stop<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        if self.armed {
            info!("stopping: leaving the speakers at the card's safe level");
            self.note_permit(hw, false, "stopping");
        }
        self.stopping = true;
        self.armed = false;
        self.armed_at = None;
        self.note_written(hw.now());
        hw.relinquish().map_err(|e| format!("giving up the volume lock: {e}"))?;
        self.relinquished_at = Some(hw.now());
        self.locked_at = self.relinquished_at;
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
        // One epoch for the model and the file (review r5 V1).  The
        // monotonic clock is read between two reads of the boot time (the
        // closest of a few tries) and the model charged up to it as usual.
        // The file names the later boot time; the time between the reads is
        // then charged once more at a full-scale signal at full gain, which
        // heats more than anything the card can have done in it.
        let (mut boot, mut now, mut after) = (hw.boottime(), hw.now(), hw.boottime());
        for _ in 0..3 {
            if after.saturating_sub(boot) <= Duration::from_millis(1) {
                break;
            }
            (boot, now, after) = (hw.boottime(), hw.now(), hw.boottime());
        }
        self.charge(now)?;
        let pairing = after.saturating_sub(boot).as_secs_f64();
        if pairing > 0.0 {
            let ta = self.globals.t_ambient;
            for s in self.speakers.iter_mut() {
                let full = s.worst_case_power(0.0);
                s.step_pre(full, full, pairing, ta);
                s.govern(&self.globals);
            }
            if let Some(e) = self.engine.as_mut() {
                e.worst(pairing, &vec![0.0; self.written.len()]);
            }
            self.check_model()?;
        }
        if self.write_state(hw, after, after, true) {
            Ok(())
        } else {
            Err("the final model state could not be saved".into())
        }
    }

    /// The reduction each control holds, unknown taken as full gain.
    fn written_db(&self) -> Vec<f64> {
        self.written.iter().map(|&w| if w.is_finite() { w } else { 0.0 }).collect()
    }

    /// A write of `wanted` failed: some controls may hold it and some the
    /// old value, so each holds at least the smaller reduction (review F1
    /// of speakerguardd#6) and at most the larger, rounded up by up to a
    /// step the card may have taken; an unknown control stays unknown.
    /// Each control's write is atomic, so it holds one of those values, not
    /// anything between them (review r4 CC1 of speakerguardd#8).
    fn write_failed(&mut self, wanted: &[f64], step: f64) {
        for (((w, h), set), &r) in self.written.iter_mut().zip(self.written_hi.iter_mut()).zip(self.written_set.iter_mut()).zip(wanted) {
            if w.is_finite() {
                *w = w.min(r);
            }
            if h.is_finite() {
                *h = h.max(r + step);
            }
            if w.is_finite() && h.is_finite() {
                add_candidate(set, (r, r + step));
            } else {
                set.clear();
            }
        }
        self.requested = vec![f64::NAN; wanted.len()];
    }

    /// Each control's candidate values for the history, or none (the span
    /// from the least to the most) where they do not account for exactly
    /// that span.
    fn candidates(&self) -> Vec<Vec<(f64, f64)>> {
        self.written_set
            .iter()
            .zip(self.written.iter().zip(&self.written_hi))
            .map(|(set, (&lo, &hi))| {
                let (a, b) = set.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), r| (a.min(r.0), b.max(r.1)));
                if !set.is_empty() && a == lo && b == hi { set.clone() } else { Vec::new() }
            })
            .collect()
    }

    /// The governing reductions: card-wide (dB), per speaker, and whether
    /// the governing model latched its hard limit.
    fn governing_state(&self) -> (f64, Vec<f64>, bool) {
        let n = self.speakers.len();
        match (self.governing, self.engine.as_ref()) {
            (Governor::Legacy, _) | (_, None) => {
                let r = model::reduction_db(&self.speakers);
                (r, vec![r; n], self.speakers.iter().any(|s| s.hard))
            }
            (gov, Some(e)) => {
                // with the asahi guard, each speaker's larger reduction of
                // the two governors and either one's latch
                let (per, hard) = e.governed(gov);
                let r = if per.iter().all(|r| r.is_finite()) { per.iter().cloned().fold(0.0, f64::max) } else { f64::NAN };
                (r, per, hard)
            }
        }
    }

    /// The controls' values, or whether the daemon holds the lease, changed
    /// at `at`.
    fn note_written(&mut self, at: Instant) {
        let g = Gain { at, lo: self.written.clone(), hi: self.written_hi.clone(), kernel: !self.armed, cands: self.candidates() };
        if self.gains.last().is_none_or(|e| e.lo != g.lo || e.hi != g.hi || e.kernel != g.kernel || e.cands != g.cands) {
            self.gains.push(g);
        }
    }

    /// The card may have held its own safe level since `from` (a lease
    /// that may have run out): the values held since then may have
    /// converted samples at it.
    fn mark_kernel_since(&mut self, from: Instant) {
        if let Some(i) = self.gains.iter().rposition(|e| e.at <= from) {
            if !self.gains[i].kernel && self.gains[i].at < from {
                let e = Gain { at: from, kernel: true, ..self.gains[i].clone() };
                self.gains.insert(i + 1, e);
            }
        }
        for e in self.gains.iter_mut().filter(|e| e.at >= from) {
            e.kernel = true;
        }
    }

    /// A verified boundary at `at`: only the gains held from QUEUE_BOUND
    /// before it on can still be playing (kept GAIN_RETAIN longer).
    fn verify(&mut self, at: Instant) {
        self.verified_at = at;
        let Some(from) = at.checked_sub(QUEUE_BOUND + GAIN_RETAIN) else { return };
        let keep = self.gains.iter().rposition(|e| e.at <= from).unwrap_or(0);
        self.gains.drain(..keep);
    }

    /// The entries of the controls' history that held at some time in
    /// [from, to].
    fn held_over(&self, from: Instant, to: Instant) -> impl Iterator<Item = &Gain> + '_ {
        self.gains.iter().enumerate().filter_map(move |(i, e)| {
            let next = self.gains.get(i + 1).map(|n| n.at);
            let overlaps = next.is_none_or(|n| n > from) && (e.at <= to || i == 0);
            overlaps.then_some(e)
        })
    }

    /// The least reduction each speaker's control may have applied to
    /// samples playing after `from` (unknown counts as full gain): the
    /// least since QUEUE_BOUND before it.  When the card has held its lock
    /// since before that (a hand-off that has drained), each control's
    /// least is its value or the card's safe level, whichever is more
    /// (review X1 of speakerguardd#8).
    fn least_since(&self, from: Instant) -> Vec<f64> {
        let converted = from.checked_sub(QUEUE_BOUND);
        let from = converted.unwrap_or(from);
        let far = from + Duration::from_secs(1 << 20);
        let mut least = self.written_db();
        for e in self.held_over(from, far) {
            for (l, &x) in least.iter_mut().zip(&e.lo) {
                *l = l.min(if x.is_finite() { x } else { 0.0 });
            }
        }
        if self.locked_at.is_some_and(|t| converted.is_some_and(|c| c >= t)) {
            let safe = self.globals.safe_reduction;
            for l in least.iter_mut() {
                *l = l.max(safe);
            }
        }
        least
    }

    /// The reductions (ranges, dB) the card may have converted each
    /// speaker's samples at over [from, to]: what its control may have held
    /// (after a failed write anything from the old value to the new one
    /// rounded up), the card's safe level over it while the daemon may not
    /// have held a live lease, anything (from `unknown_floor`) for an
    /// unknown value.
    fn possible_reductions(&self, from: Instant, to: Instant) -> Vec<Vec<(f64, f64)>> {
        let safe = self.globals.safe_reduction;
        let mut out = vec![Vec::new(); self.written.len()];
        for e in self.held_over(from, to) {
            for (k, o) in out.iter_mut().enumerate() {
                let (lo, hi) = (e.lo[k], e.hi[k]);
                if lo.is_finite() && hi.is_finite() {
                    // a failed write's old and new values, or the span
                    let span = [(lo, hi)];
                    let values = match e.cands.get(k) {
                        Some(c) if !c.is_empty() => &c[..],
                        _ => &span[..],
                    };
                    for &(a, b) in values {
                        o.push((a, b));
                        if e.kernel && safe > a {
                            o.push((safe, safe.max(b)));
                        }
                    }
                } else {
                    o.push((self.unknown_floor, MUTE_DB));
                }
            }
        }
        out
    }

    /// The least and the most known reduction each control held since
    /// QUEUE_BOUND before the last verified boundary (unknown counts as full
    /// gain), for the recording's rebuilt program.
    fn held(&self) -> (Vec<f64>, Vec<f64>) {
        let least = self.least_since(self.verified_at);
        let from = self.verified_at.checked_sub(QUEUE_BOUND).unwrap_or(self.verified_at);
        let mut most = self.written_db();
        for e in self.held_over(from, from + Duration::from_secs(1 << 20)) {
            for (m, &x) in most.iter_mut().zip(&e.hi) {
                *m = m.max(if x.is_finite() && x < MUTE_DB { x } else { 0.0 });
            }
        }
        (least, most)
    }

    /// Give the card's lock up: macaudio locks the volume at once and stops
    /// what was converted at the old gain.  The account keeps the old gain
    /// until the next verified boundary.  A failure is fatal: the daemon
    /// exits, which gives the lock up too.
    fn hand_off<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        // the lease may have run out before this, whatever the reason now
        let lapse = self.last_ping + KERNEL_LOCK_TIMEOUT;
        if hw.now() > lapse {
            self.mark_kernel_since(lapse);
        }
        self.armed = false;
        self.armed_at = None;
        self.note_written(hw.now());
        hw.relinquish().map_err(|e| format!("giving up the volume lock: {e}"))?;
        self.locked_at = Some(hw.now());
        Ok(())
    }

    fn disarm<H: Hw>(&mut self, hw: &mut H, why: &str) -> Result<(), Box<dyn Error>> {
        if self.armed {
            warn!("{why}; leaving the speakers at the card's safe level");
            self.note_permit(hw, false, why);
            self.hand_off(hw)?;
        }
        Ok(())
    }

    /// Pings the volume lock.  A ping that answers ETIMEDOUT is the card's
    /// one-shot notice that the lock had lapsed, so ping once more.
    fn ping_lock<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        // whatever the ping answers, the card may be unlocked from here on
        self.locked_at = None;
        match hw.ping() {
            Ok(()) => {}
            Err(e) if e.raw_os_error() == Some(libc::ETIMEDOUT) => {
                if self.armed {
                    warn!("the card locked the volume while we were away; taking it back");
                }
                // the card held its safe level from the lapse (not before
                // the lease's length after the last ping) until now
                self.mark_kernel_since(self.last_ping + KERNEL_LOCK_TIMEOUT);
                hw.ping().map_err(|e| format!("lock ping: {e}"))?;
            }
            Err(e) => return Err(format!("lock ping: {e}").into()),
        }
        self.last_ping = hw.now();
        Ok(())
    }

    fn check_model(&self) -> Result<(), Box<dyn Error>> {
        if self.speakers.iter().all(|s| s.finite()) && self.engine.as_ref().is_none_or(Engine::finite) {
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
        let least = self.least_since(self.verified_at);
        let globals = &self.globals;
        for (s, &r) in self.speakers.iter_mut().zip(&least) {
            let p = s.worst_case_power(r);
            s.step_at(p, d, globals.t_ambient, r);
            s.govern(globals);
        }
        if let Some(e) = self.engine.as_mut() {
            e.worst(d, &least);
        }
        self.note_account(self.account_to, to, Accounted::Charged);
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
            if let Some(e) = self.engine.as_mut() {
                e.idle(d);
            }
            self.note_account(self.account_to, to, Accounted::Cooled);
            self.account_to = to;
        }
        self.verify(to);
        self.check_model()
    }

    /// `frames` sense frames in `self.words` whose last one was captured at
    /// `end` (the read time less the capture's backlog).  Time before them
    /// since the account's end is charged first.  Of the frames, those over
    /// time not yet accounted are integrated as a period is, every model
    /// the same way; the oldest of them only for its part over that time,
    /// frames wholly over it not at all (review P8-3).  The loop places
    /// every period with `place`; this is for the tests.
    #[cfg(test)]
    fn measure(&mut self, end: Instant, frames: usize) -> Result<(), Box<dyn Error>> {
        self.measure_shifted(end, frames, 0)
    }

    /// The same for frames whose end is only known to lie between `end`
    /// and `shift` frames after it.  Only `frames - shift` of them then
    /// certainly played up to `end`, and which ones is not known: each
    /// model takes the most energetic run of that many (the legacy thermal
    /// model the most delivered energy, its budgets the most before the
    /// gain; the engine's models likewise), and the time before the run is
    /// charged.
    fn measure_shifted(&mut self, end: Instant, frames: usize, shift: usize) -> Result<(), Box<dyn Error>> {
        if frames == 0 || self.rate == 0 || end <= self.account_to {
            return Ok(());
        }
        let shift = shift.min(frames - 1);
        let certain = frames - shift;
        let rate = self.rate as f64;
        let dt = certain as f64 / rate;
        let start = end.checked_sub(Duration::from_secs_f64(dt)).unwrap_or(end);
        if start > self.account_to {
            self.charge(start)?;
        }
        // A lease that may have run out before these frames played: the
        // card may have converted them at its safe level (review r5 V4),
        // whatever the next decision makes of it, and whether or not the
        // lock has been given up since.
        let lapse = self.last_ping + KERNEL_LOCK_TIMEOUT;
        if end > lapse {
            self.mark_kernel_since(lapse);
        }
        let globals = self.globals.clone();
        let ch = globals.channels;
        // the time not yet accounted, in frames: the newest `k` of the run,
        // the first of them for `first` of a frame
        let new = ((end - self.account_to).as_secs_f64() * rate).min(certain as f64);
        let k = (new.ceil() as usize).clamp(1, certain);
        let first = (new - (k - 1) as f64).clamp(f64::MIN_POSITIVE, 1.0);
        let all = &self.words[..frames * ch];
        // The budgets' power before the gain: these frames played up to
        // `end`, converted up to QUEUE_BOUND before they played, each at any
        // reduction the card may have applied then (the most that can have
        // delivered the frame is the safer guess).
        let played = end.checked_sub(Duration::from_secs_f64(frames as f64 / rate)).unwrap_or(end);
        let converted = played.checked_sub(QUEUE_BOUND).unwrap_or(played);
        let possible = self.possible_reductions(converted, end);
        let parts: &[(usize, usize, f64)] = if first < 1.0 { &[(0, 1, first), (1, k, 1.0)] } else { &[(0, k, 1.0)] };
        // the models take the blocks as equal shares of the run's time, so
        // each block's sum is spread over an equal share: the run's energy
        // is exact however its frames split
        let means = |x: &[f64], blocks: usize| -> Vec<f64> {
            let n = x.len();
            (0..blocks).map(|b| {
                let (lo, hi) = (n * b / blocks, n * (b + 1) / blocks);
                x[lo..hi].iter().sum::<f64>() / (n as f64 / blocks as f64)
            }).collect()
        };
        for (s, pos) in self.speakers.iter_mut().zip(&possible) {
            let (p, q) = s.frame_powers(all, ch, pos);
            if p.iter().any(|x| !x.is_finite()) || q.iter().any(|x| !x.is_finite() || *x < 0.0) {
                return Err("invalid speaker sense voltage or power".into());
            }
            let run = |o: usize| (o + certain - k, o + certain);
            let best = |x: &[f64]| (0..=shift).max_by(|&a, &b| {
                let (sa, sb) = (run(a), run(b));
                x[sa.0..sa.1].iter().sum::<f64>().total_cmp(&x[sb.0..sb.1].iter().sum::<f64>())
            }).unwrap_or(0);
            let (hp, hq) = (run(best(&p)), run(best(&q)));
            let (heat, budget) = (&p[hp.0..hp.1], &q[hq.0..hq.1]);
            let v = (heat.iter().sum::<f64>() / k as f64 * s.spec.r_ff).sqrt();
            let mut peak = f64::NEG_INFINITY;
            for &(lo, hi, w) in parts {
                if hi <= lo {
                    continue;
                }
                let n = hi - lo;
                let blocks = (SUB_BLOCKS * n).div_ceil(globals.period).clamp(1, n);
                let powers = means(&heat[lo..hi], blocks);
                let pre = means(&budget[lo..hi], blocks);
                let secs = if n == 1 { w / rate } else { n as f64 / rate };
                s.step_period_pre(&powers, &pre, secs, globals.t_ambient);
                peak = peak.max(s.t_peak);
            }
            s.t_peak = peak;
            s.v_rms = v;
            s.govern(&globals);
        }
        if let Some(e) = self.engine.as_mut() {
            // the same frames over the same time: the sensed power is what
            // the card's gain let through
            e.measured(all, frames, certain, k, shift, 1.0 / rate, first, &possible);
            for (k, lost) in std::mem::take(&mut e.lost_changed) {
                let name = &self.speakers[k].spec.name;
                if lost {
                    warn!("{name}: the current sense reads far too little for the voltage; its measured power is bounded by V^2 over the speaker's smallest impedance until it reads again");
                } else {
                    info!("{name}: the current sense reads again");
                }
            }
        }
        // for the recording: the most known reduction over them
        self.last_applied = possible.iter().map(|p| p.iter().map(|r| if r.1 >= MUTE_DB { 0.0 } else { r.1 }).fold(0.0, f64::max)).collect();
        self.note_account(self.account_to, end, Accounted::Measured);
        self.account_to = end;
        self.verify(end);
        self.check_model()
    }

    /// Measure `frames` frames whose position (`queued` frames behind) was
    /// read between `before` and `after`: their end lies between the two
    /// less the backlog.  The time they may not cover at either end is
    /// charged (a pause around the position read never lets them stand for
    /// time they did not see).
    fn place(&mut self, before: Instant, after: Instant, queued: u64, frames: usize) -> Result<(), Box<dyn Error>> {
        let rate = self.rate.max(1) as f64;
        // a whole-frame position: the frames end up to a frame before the
        // backlog says, and anywhere the two clock reads leave open
        let early = before.checked_sub(Duration::from_secs_f64((queued + 1) as f64 / rate)).unwrap_or(before);
        let late = after.checked_sub(Duration::from_secs_f64(queued as f64 / rate)).unwrap_or(after);
        let shift = (late.saturating_duration_since(early).as_secs_f64() * rate).ceil() as usize;
        self.measure_shifted(early, frames, shift)
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
                let before = hw.now();
                let queued = hw.queued();
                let after = hw.now();
                let ch = self.globals.channels;
                let q_rec = *queued.as_ref().unwrap_or(&0);
                rec_now(&mut self.rec, hw, |r, t| r.tail(t, q_rec, frames, &self.words[..frames * ch]));
                if let Err(why) = self.check_sense(frames) {
                    // not a measurement: the rest up to now is charged
                    self.bad_sense(hw, &why)?;
                    break;
                }
                // without the capture's position the frames cannot be placed
                let Ok(q) = queued else { break };
                self.place(before, after, q, frames)?;
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

    /// Whether `frames` sense frames in `self.words` are a measurement: on
    /// the machines with V/I sense, a mean V*I below -0.01 W, or a
    /// nonfinite one, is bad sense data (review F3 of speakerguardd#6).
    fn check_sense(&mut self, frames: usize) -> Result<(), String> {
        let Some(e) = self.engine.as_mut() else { return Ok(()) };
        let words = &self.words[..frames * self.globals.channels];
        e.check(words, self.rate as f64)
    }

    /// Bad sense data: the card goes to the kernel at once, and the lock
    /// is only taken back after REARM_PERIODS complete periods that are
    /// measurements again.  Its time stays uncovered, charged at full
    /// scale like any other.
    fn bad_sense<H: Hw>(&mut self, hw: &mut H, why: &str) -> Result<(), Box<dyn Error>> {
        if !self.bad_data || self.armed {
            warn!("{why}: the sense data is not a measurement");
        }
        self.bad_data = true;
        self.recovering = true;
        self.fresh = 0;
        self.good_since_bad = 0;
        self.disarm(hw, "the sense data is not a measurement")
    }

    /// One pass of the loop: idle accounting, or one sense period.
    pub fn tick<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        let globals = self.globals.clone();
        // before the rate read: a stream it shows was there by then, and the
        // time without sense data is counted from here, however long the
        // daemon is held up after the read
        let look = hw.now();
        // a change event since the last pass (read before the rate, so that
        // a change after the read is kept for the next pass)
        let before = hw.rate_events();
        let changed = std::mem::take(&mut self.event_pending) | before;
        let read = hw.sample_rate();
        let seen = match &read {
            Ok(v) => *v,
            Err(e) => -(e.raw_os_error().unwrap_or(libc::EIO) as i64).max(1),
        };
        if self.rec.is_some() && (self.rec_rate != Some(seen) || seen < 0) {
            self.rec_rate = Some(seen);
            rec_now(&mut self.rec, hw, |r, t| r.rate(t, seen));
        }
        let rate = match read {
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
                self.good_since_bad = 0;
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
            } else {
                // Events again, after the read: one that came while the read
                // took its time (a stream inside it) makes the interval
                // unknown, and one after it the next.  Both are kept for
                // the next pass.
                let late = hw.rate_events();
                self.event_pending = late;
                if self.events && self.closed_seen && !changed && !late {
                    // closed at both looks, nothing changed from the last
                    // look's events to now: no power, up to where an event
                    // in flight could still say otherwise
                    if let Some(to) = now.checked_sub(EVENT_SLACK) {
                        self.cool(to)?;
                    }
                } else {
                    // a change between two looks at closed speakers (a
                    // stream that came and went), or nothing known
                    let from = self.account_to;
                    self.charge(now)?;
                    if self.rec.is_some() {
                        // what decided it, for a replay (review P8-8): its
                        // own events cannot say
                        let (a, b) = (self.boot_of(from), self.boot_of(now));
                        rec_do(&mut self.rec, b, |r, ns| r.look(ns, a.as_nanos() as u64, changed, late));
                    }
                }
            }
            self.dt = None;
            self.seen_to = None;
            self.fresh = 0;
            self.good_since_bad = 0;
            self.closed_seen = true;
            self.maintain(hw)?;
            if hw.wait_rate_change(IDLE_POLL) {
                rec_now(&mut self.rec, hw, |r, t| r.wake(t));
            }
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
            self.seen_to = Some(look);
        }
        if self.seen_to.is_none() {
            self.seen_to = Some(self.account_to);
        }
        if self.stall.is_none() {
            // Opening the sense PCM blocks while the playback side is being
            // set up (about 100 ms on the J700): renew a lock that is still
            // held first, so the open cannot run its lease out, through the
            // same decision as every renewal (a fault the charge above just
            // found hands the card off instead).  The time is charged all
            // the same.
            if self.armed {
                self.decide(hw, true)?;
            }
            let opened = hw.open_sense(rate);
            let errno = opened.as_ref().err().map_or(0, |e| e.raw_os_error().unwrap_or(libc::EIO));
            rec_now(&mut self.rec, hw, |r, t| r.open(t, errno));
            match opened {
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
                let before = hw.now();
                let queued = match hw.queued() {
                    Ok(q) => q,
                    Err(e) => {
                        // without its position the period cannot be placed
                        warn!("sense capture position: {e}; period discarded");
                        let errno = e.raw_os_error().unwrap_or(libc::EIO);
                        rec_now(&mut self.rec, hw, |r, t| r.loss(t, record::LOSS_POSITION, errno));
                        self.fresh = 0;
                        self.good_since_bad = 0;
                        return self.maintain(hw);
                    }
                };
                let after = hw.now();
                self.stall.as_mut().unwrap().data();
                // the latest the frames can end (bad data is charged to it)
                let end = after.checked_sub(Duration::from_secs_f64(queued as f64 / rate as f64)).unwrap_or(after);
                let read_at = if self.rec.is_some() { hw.boottime() } else { Duration::ZERO };
                let start = end.checked_sub(Duration::from_secs_f64(dt)).unwrap_or(end);
                if let Err(why) = self.check_sense(globals.period) {
                    if self.rec.is_some() {
                        // what the card may have applied over it
                        let (_, most) = self.held();
                        let applied = self.applied_over(start, &most);
                        rec_do(&mut self.rec, read_at, |r, t| r.period(t, queued, rate, false, &applied, &self.words));
                    }
                    // uncovered time: charged at full scale up to the
                    // period's end, never measured; the period does not
                    // count as seen
                    self.charge(end)?;
                    self.bad_sense(hw, &why)?;
                    return self.maintain(hw);
                }
                self.bad_data = false;
                self.good_since_bad = self.good_since_bad.saturating_add(1);
                if self.good_since_bad >= REARM_PERIODS {
                    self.recovering = false;
                }
                self.place(before, after, queued, globals.period)?;
                if self.rec.is_some() {
                    let applied = self.applied_over(start, &self.last_applied);
                    rec_do(&mut self.rec, read_at, |r, t| r.period(t, queued, rate, true, &applied, &self.words));
                }
                // when data last arrived (a backlog being read is data)
                self.seen_to = Some(before);
                self.fresh = self.fresh.saturating_add(1);
            }
            Ok(Some(false)) => {
                warn!("incomplete sense period or overrun, step skipped");
                rec_now(&mut self.rec, hw, |r, t| r.loss(t, record::LOSS_SHORT, 0));
                self.fresh = 0;
                self.good_since_bad = 0;
            }
            Ok(None) => {
                if self.stall.as_mut().unwrap().silent(hw.now()) {
                    warn!("sense stream stalled for {} ms; reopening", STALL_TIMEOUT.as_millis());
                    rec_now(&mut self.rec, hw, |r, t| r.loss(t, record::LOSS_STALL, 0));
                    self.stall = None;
                    self.fresh = 0;
                    self.good_since_bad = 0;
                    hw.close_sense();
                }
            }
            Err(e) if transient(&e) => {
                warn!("sense capture: {e}; reopening");
                let errno = e.raw_os_error().unwrap_or(libc::EIO);
                rec_now(&mut self.rec, hw, |r, t| r.loss(t, record::LOSS_CAPTURE, errno));
                self.stall = None;
                self.fresh = 0;
                self.good_since_bad = 0;
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

    /// Write the state file as a status readout (never resumed).
    fn save_now<H: Hw>(&mut self, hw: &H) -> bool {
        let now = hw.now();
        let boot = hw.boottime();
        let covered = boot.saturating_sub(now.saturating_duration_since(self.account_to));
        self.write_state(hw, boot, covered, false)
    }

    /// Write the state file (atomically): a status readout, or with
    /// `released` the state a restart may resume, its model covering up to
    /// `covered` (boot time).
    fn write_state<H: Hw>(&mut self, hw: &H, boot: Duration, covered: Duration, released: bool) -> bool {
        let Some(path) = &self.state_path else { return true };
        self.last_save_try = Some(hw.now());
        let text = state::encode_full(boot, covered, &self.speakers, self.engine.as_ref(), self.armed, released, self.recovering);
        match state_write(path, &text) {
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
        now.saturating_duration_since(self.account_to) <= allowed && !(self.playing && self.overdue(now) > BLIND_LIMIT)
    }

    /// Charge unmeasured playback time, decide whether the daemon holds the
    /// card, and if it does write the model's reduction and renew the lock.
    fn maintain<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        self.decide(hw, false)
    }

    /// Every renewal of the lock goes through here: the account brought up
    /// to date, then the faults, the reduction limit and the model's
    /// freshness decide whether the daemon keeps the card.  With `renew_now`
    /// a lock that is kept is renewed whatever its age (before something
    /// that blocks), and a cleared fault waits for the next pass.
    fn decide<H: Hw>(&mut self, hw: &mut H, renew_now: bool) -> Result<(), Box<dyn Error>> {
        let now = hw.now();
        let blind = self.overdue(now);
        if self.unknown || (self.playing && blind > BLIND_SLACK) {
            // nothing can cover the time since the account's end any more
            self.charge(now)?;
        }
        let (reduction, per, hard) = self.governing_state();
        if !reduction.is_finite() {
            return Err("nonfinite speaker reduction".into());
        }
        if hard != self.last_hard {
            self.last_hard = hard;
            if self.trace.is_some() || self.rec.is_some() {
                let t = hw.boottime();
                if let Some(tr) = self.trace.as_mut() {
                    tr.push((t, Trace::Latch(hard)));
                }
                rec_do(&mut self.rec, t, |r, ns| r.latch(ns, hard));
            }
        }
        if self.armed && now >= self.last_ping + KERNEL_LOCK_TIMEOUT {
            // the daemon was late and the card may have taken its volume
            // back: give it up, then take it again the safe way
            self.mark_kernel_since(self.last_ping + KERNEL_LOCK_TIMEOUT);
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
        } else if !self.stopping && !self.unknown && !hard && self.events && !renew_now && !self.recovering {
            // after bad sense data, REARM_PERIODS valid periods in a row
            // first, idle or not (review P8-2)
            let seeing = !self.playing || (self.fresh >= REARM_PERIODS && blind <= BLIND_SLACK);
            let retry = self.last_save_try.is_none_or(|t| hw.now().duration_since(t) >= SAVE_INTERVAL);
            if seeing && reduction < self.max_reduction - REARM_MARGIN_DB && !self.invalidated && retry {
                // the clean stop this run may have resumed goes first
                self.invalidated = self.invalidate(hw);
            }
            if seeing && reduction < self.max_reduction - REARM_MARGIN_DB && self.invalidated {
                match hw.set_min() {
                    Ok(()) => {
                        let n = self.written.len();
                        self.written = vec![MUTE_DB; n];
                        self.written_hi = vec![MUTE_DB; n];
                        self.written_set = vec![vec![(MUTE_DB, MUTE_DB)]; n];
                        self.requested = vec![f64::NAN; n];
                        self.note_written(hw.now());
                        self.min_failures = 0;
                        // the minimum write may have blocked: the model must
                        // still be current at the unlock
                        if self.fresh_enough(hw.now()) {
                            self.ping_lock(hw)?;
                            if self.fresh_enough(hw.now()) {
                                info!("taking the volume lock{} at {reduction:.1} dB", if self.held { " back" } else { "" });
                                self.armed = true;
                                self.armed_at = Some(hw.now());
                                self.note_written(hw.now());
                                self.note_permit(hw, true, "lock taken");
                                self.held = true;
                                self.applied = reduction;
                            } else {
                                // the ping itself blocked: give it up again
                                debug!("not taking the lock: the ping blocked");
                                hw.relinquish().map_err(|e| format!("giving up the volume lock: {e}"))?;
                                self.locked_at = Some(hw.now());
                            }
                        } else {
                            debug!("not taking the lock: the model fell behind");
                        }
                    }
                    Err(e) if transient(&e) && self.min_failures < MAX_CONTROL_FAILURES => {
                        self.min_failures += 1;
                        warn!("volume control: {e}; not taking the lock ({})", self.min_failures);
                        rec_now(&mut self.rec, hw, |r, t| r.control_failure(t, record::FAIL_MIN));
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
            let due = per != self.requested || hw.now().duration_since(self.last_write) >= PING_INTERVAL;
            // the legacy governor writes one value to every control; the
            // engine's write per speaker (a card with one control gets the
            // largest)
            let n = per.len();
            let write = |hw: &mut H| if self.governing == Governor::Legacy { hw.set_reduction(reduction).map(|a| vec![a; n]) } else { hw.set_reductions(&per) };
            match if due { write(hw).map(Some) } else { Ok(None) } {
                Ok(None) => {}
                Ok(Some(applied)) => {
                    // what the cards apply, never less than asked for
                    self.written = per.iter().zip(applied.iter().chain(std::iter::repeat(&f64::NAN))).map(|(&r, &a)| if a.is_finite() { a.max(r) } else { r }).collect();
                    self.written_hi = self.written.clone();
                    self.written_set = self.written.iter().map(|&w| vec![(w, w)]).collect();
                    self.requested = per.clone();
                    self.last_write = hw.now();
                    self.note_written(hw.now());
                    self.target_failures = 0;
                }
                Err(e) if transient(&e) && self.target_failures < MAX_CONTROL_FAILURES => {
                    self.target_failures += 1;
                    warn!("volume control: {e} ({})", self.target_failures);
                    // some controls may hold the new value and some the old:
                    // each applies at least the smaller reduction, at most
                    // the larger rounded up
                    let step = hw.step_db();
                    self.write_failed(&per, step);
                    self.note_written(hw.now());
                    rec_now(&mut self.rec, hw, |r, t| r.control_failure(t, record::FAIL_TARGET));
                    self.disarm(hw, "the volume control failed")?;
                }
                Err(e) => return Err(format!("volume control: {e}").into()),
            }
            if self.armed && (renew_now || hw.now().duration_since(self.last_ping) >= PING_INTERVAL) {
                // the write may have blocked: renew only on a current model
                if self.fresh_enough(hw.now()) {
                    self.ping_lock(hw)?;
                    // a ping that blocked (or found the lock lapsed and took
                    // it again) must not carry a model that fell behind in it
                    if !self.fresh_enough(hw.now()) {
                        self.disarm(hw, "the renewal blocked")?;
                    }
                } else {
                    self.disarm(hw, "the model fell behind")?;
                }
            }
        }
        if renew_now {
            return Ok(());
        }
        // a cleared fault only takes effect from the next pass
        for s in self.speakers.iter_mut() {
            if s.clear_hard_if_cool() {
                info!("{}: coil back under its working limit", s.spec.name);
            }
        }
        for name in self.engine.as_mut().map(Engine::clear_hard_if_cool).unwrap_or_default() {
            info!("{name}: back under its working limit");
        }
        if hw.now().duration_since(self.last_log) >= LOG_INTERVAL {
            for (k, s) in self.speakers.iter().enumerate() {
                let engine = match &self.engine {
                    Some(e) => {
                        let (a, ff, vi) = e.temps(k);
                        format!(" | asahi {a:.1} C, unified {ff:.1}/{vi:.1} C, {} {:.1} dB", self.governing.name(), per.get(k).copied().unwrap_or(f64::NAN))
                    }
                    None => String::new(),
                };
                debug!(
                    "{}: {:.3} Vrms {:.1} mW coil {:.1} C magnet {:.1} C reduction {:.1} dB{}{engine}",
                    s.spec.name,
                    s.v_rms,
                    s.power * 1000.0,
                    s.t_coil,
                    s.t_magnet,
                    s.reduction_db,
                    if self.armed { "" } else { " (card at its safe level)" }
                );
            }
            self.shadow(hw);
            self.last_log = hw.now();
        }
        if self.last_save_try.is_none_or(|t| hw.now().duration_since(t) >= SAVE_INTERVAL) {
            self.save_now(hw);
        }
        Ok(())
    }

    /// Write the state file as a status readout (a restart does not
    /// resume it).
    pub fn save<H: Hw>(&mut self, hw: &H) {
        self.save_now(hw);
    }

    /// Append one line to the shadow log (best effort; rotated by size).
    fn shadow<H: Hw>(&mut self, hw: &H) {
        let (Some(path), Some(e)) = (self.shadow_path.clone(), self.engine.as_ref()) else { return };
        let legacy: Vec<(f64, f64)> = self.speakers.iter().map(|s| (s.t_coil, s.reduction_db)).collect();
        let line = e.shadow_line(hw.boottime().as_secs_f64(), self.governing, self.armed, &self.written, &legacy);
        if std::fs::metadata(&path).is_ok_and(|m| m.len() > SHADOW_MAX_BYTES) {
            let _ = std::fs::rename(&path, path.with_extension("jsonl.1"));
        }
        use std::io::Write as _;
        let result = std::fs::OpenOptions::new().create(true).append(true).open(&path).and_then(|mut f| f.write_all(line.as_bytes()));
        match result {
            Ok(()) => self.shadow_failed = false,
            Err(e) if !self.shadow_failed => {
                warn!("{}: {e}; no shadow log", path.display());
                self.shadow_failed = true;
            }
            Err(_) => {}
        }
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
        /// The card rounds reductions up to 0.5 dB steps, as the J700 does.
        pub steps: bool,
        pub events: bool,
        /// How long opening the sense PCM blocks.
        pub open_delay: Duration,
        /// Where rate_events last looked.
        events_seen_to: Duration,
        /// The kernel's safe level (SAFE_DB, the J700's; 99.5 on the V/I
        /// machines).
        pub safe: f64,
        /// Per-speaker controls: what each last accepted (empty: one
        /// shared control, `reduction`).
        pub per: Vec<f64>,
        /// The speaker each sense channel belongs to (empty: every channel
        /// carries the shared control's gain).
        pub chan_speaker: Vec<Option<usize>>,
        /// A per-speaker write fails (EIO) at time t after this many
        /// speakers took the new value.
        pub partial_fail: Box<dyn Fn(Duration) -> Option<usize>>,
        /// Rewrites each complete period's words (the n-th period, from 0).
        pub tweak: Option<Box<dyn FnMut(u32, &mut [i32])>>,
        pub delivered: u32,
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
                steps: false,
                events: true,
                events_seen_to: Duration::ZERO,
                open_delay: Duration::ZERO,
                safe: SAFE_DB,
                per: Vec::new(),
                chan_speaker: Vec::new(),
                partial_fail: Box::new(|_| None),
                tweak: None,
                delivered: 0,
            }
        }

        /// A card with a volume control per speaker and the sense channels
        /// of `cfg` (V and I of each speaker), at the configuration's safe
        /// level.
        pub fn per_speaker(cfg: &Config) -> FakeHw {
            let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
            hw.safe = cfg.globals.safe_reduction;
            hw.reduction = hw.safe;
            hw.per = vec![hw.safe; cfg.speakers.len()];
            hw.chan_speaker = vec![None; cfg.globals.channels];
            for (k, s) in cfg.speakers.iter().enumerate() {
                hw.chan_speaker[s.vs_chan] = Some(k);
                if let Some(ic) = s.is_chan {
                    hw.chan_speaker[ic] = Some(k);
                }
            }
            hw
        }

        /// The gain the card applies to speaker `k`.
        pub fn effective_of(&self, k: usize) -> f64 {
            let r = self.per.get(k).copied().unwrap_or(self.reduction);
            if self.unlocked_at(self.t) {
                r
            } else {
                r.max(self.safe)
            }
        }

        /// The gain on sense channel `c`.
        fn chan_reduction(&self, c: usize) -> f64 {
            match self.chan_speaker.get(c).copied().flatten() {
                Some(k) if !self.per.is_empty() => self.effective_of(k),
                _ => self.effective_reduction(),
            }
        }

        /// `frames` frames of the program at time t through the card's
        /// gains: a square wave, sign alternating frame to frame.
        fn fill(&self, words: &mut [i32], frames: usize) {
            let level = (self.level)(self.t);
            let per: Vec<i32> = (0..self.channels)
                .map(|c| ((level * 10f64.powf(-self.chan_reduction(c) / 20.0)).clamp(0.0, 1.0) * 2147483647.0) as i32)
                .collect();
            for (i, w) in words[..frames * self.channels].iter_mut().enumerate() {
                let word = per[i % self.channels];
                *w = if (i / self.channels) % 2 == 0 { word } else { -word };
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
                self.reduction.max(self.safe)
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
            for p in self.per.iter_mut() {
                *p = MUTE_DB;
            }
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
        fn step_db(&self) -> f64 {
            if self.steps { 0.5 } else { 0.0 }
        }
        fn set_reduction(&mut self, db: f64) -> io::Result<f64> {
            self.lapse_check();
            if (self.volume_fail)(self.t) || (self.target_fail)(self.t) {
                return Err(io::Error::from_raw_os_error(libc::EIO));
            }
            if !self.unlocked_at(self.t) && db < self.safe {
                return Err(io::Error::from_raw_os_error(libc::EINVAL));
            }
            let db = if self.steps { (db * 2.0 - 1e-9).ceil() / 2.0 } else { db };
            self.reduction = db;
            for p in self.per.iter_mut() {
                *p = db;
            }
            self.writes.push((self.t, db));
            Ok(db)
        }
        fn set_reductions(&mut self, db: &[f64]) -> io::Result<Vec<f64>> {
            if self.per.is_empty() {
                let a = self.set_reduction(db.iter().cloned().fold(0.0, f64::max))?;
                return Ok(vec![a; db.len()]);
            }
            self.lapse_check();
            if (self.volume_fail)(self.t) || (self.target_fail)(self.t) {
                return Err(io::Error::from_raw_os_error(libc::EIO));
            }
            if !self.unlocked_at(self.t) && db.iter().any(|&d| d < self.safe) {
                return Err(io::Error::from_raw_os_error(libc::EINVAL));
            }
            let upto = (self.partial_fail)(self.t);
            for (k, (p, &d)) in self.per.iter_mut().zip(db).enumerate() {
                if upto.is_some_and(|n| k >= n) {
                    return Err(io::Error::from_raw_os_error(libc::EIO));
                }
                *p = d;
            }
            if upto.is_some() {
                return Err(io::Error::from_raw_os_error(libc::EIO));
            }
            self.reduction = db.iter().cloned().fold(0.0, f64::max);
            self.writes.push((self.t, self.reduction));
            Ok(db.to_vec())
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
            if self.pcm.is_some() {
                let gain = 10f64.powf(-self.effective_reduction() / 20.0);
                let a = ((self.level)(self.t) * gain).clamp(0.0, 1.0);
                let t = self.t;
                let reader = self.pcm.as_mut().unwrap();
                reader.io().advance_to(t);
                reader.io().word = (a * 2147483647.0) as i32;
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
            self.fill(words, k);
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
            self.fill(words, self.period_frames);
            if let Some(tweak) = self.tweak.as_mut() {
                tweak(self.delivered, words);
            }
            self.delivered += 1;
            Ok(Some(true))
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
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

    /// The time without sense data counts from before the rate read that
    /// shows a stream.  A read held up 0.12 s (the stream may have played
    /// all through it) leaves that much less of the blind limit, not more:
    /// counted from after the read, the lease could be renewed 0.12 s
    /// longer blind.
    #[test]
    fn blindness_counts_from_before_the_rate_read() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.0);
        hw.playing = Box::new(|t| t >= secs(2.0));
        hw.sensing = Box::new(|t| t < secs(2.0));
        hw.control_delay = Box::new(|t| if (secs(1.97)..secs(2.0)).contains(&t) { Duration::from_millis(120) } else { Duration::ZERO });
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(1.97)).unwrap();
        assert!(g.armed() && !g.playing);
        let look = hw.now();
        g.tick(&mut hw).unwrap();
        assert!(g.playing && hw.t >= secs(2.09), "the slow read did not show the stream");
        let seen = g.seen_to.unwrap();
        assert!(seen <= look, "blindness counted from {:?} after the look", seen.saturating_duration_since(look));
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
        run_until(&mut g3, &mut hw3, secs(12.0)).unwrap();
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
        run_until(&mut g, &mut hw, secs(12.0)).unwrap();
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
        // after the start: the samples playing at it may have been
        // converted at the card's safe level, which the budgets charge
        hw.control_delay = Box::new(|t| if (secs(2.5)..secs(2.55)).contains(&t) { Duration::from_millis(600) } else { Duration::ZERO });
        let mut g = Guard::start(&mut hw, &cfg, 40.0, Some(path)).unwrap();
        let mut tripped = None;
        while hw.t < secs(5.5) {
            g.tick(&mut hw).unwrap();
            if g.speakers.iter().any(|s| s.hard) && tripped.is_none() {
                tripped = Some(hw.t);
            }
        }
        let at = tripped.expect("the model never reached the coil limit");
        assert!(at < secs(3.3), "tripped at {at:?}");
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
        g.written = vec![0.0; 2];
        g.written_hi = g.written.clone();
        let t0 = hw.now();
        g.gains = vec![Gain { at: t0, lo: vec![0.0; 2], hi: vec![0.0; 2], kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
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
        g.written = vec![0.0; 2];
        g.written_hi = g.written.clone();
        let t0 = hw.now();
        g.gains = vec![Gain { at: t0, lo: vec![0.0; 2], hi: vec![0.0; 2], kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
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
        fn set_reduction(&mut self, d: f64) -> io::Result<f64> { self.hw.set_reduction(d) }
        fn set_reductions(&mut self, d: &[f64]) -> io::Result<Vec<f64>> { self.hw.set_reductions(d) }
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
        g.written = vec![20.0; 2];
        g.written_hi = g.written.clone();
        let t0 = hw.now();
        g.gains = vec![Gain { at: t0, lo: vec![20.0; 2], hi: vec![20.0; 2], kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
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
        fn set_reduction(&mut self, d: f64) -> io::Result<f64> { self.hw.set_reduction(d) }
        fn set_reductions(&mut self, d: &[f64]) -> io::Result<Vec<f64>> { self.hw.set_reductions(d) }
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

    // ---- the engine on the guard: speakerguardd#6's review witnesses, and
    // the r3 rules on the V/I path ----

    const J413: &str = include_str!("../conf/apple/j413.conf");
    const J316: &str = include_str!("../conf/apple/j316.conf");
    const J616: &str = include_str!("../conf/apple/j616.conf");

    /// The J413's configuration, allowed to govern for these offline tests
    /// (it is not qualified on the machine).
    pub fn j413() -> Config {
        let mut c = Config::parse(J413).unwrap();
        c.allow_unqualified = true;
        c
    }

    /// A clean stop's state of its own for `cfg`: every model's coil at
    /// `tc` and magnet at `tm`, written at the fake's start (boot 1000 s).
    pub fn vi_state(cfg: &Config, tc: f64, tm: f64) -> PathBuf {
        let path = missing_state().unwrap();
        let g = &cfg.globals;
        let mut sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        for s in sp.iter_mut() {
            s.resume(tc, tm, 0.0, 0.0);
        }
        let mut e = Engine::new(cfg, true).unwrap();
        for a in e.asahi.iter_mut().flatten() {
            a.th.t_coil = tc;
            a.th.t_magnet = tm;
            a.coil_peak = tc;
            a.magnet_peak = tm;
            a.coil_hyst = tc;
            a.magnet_hyst = tm;
        }
        for u in e.unified.iter_mut().flatten() {
            u.ff.t_coil = tc;
            u.ff.t_magnet = tm;
            if let Some(v) = u.vi.as_mut() {
                v.t_coil = tc;
                v.t_magnet = tm;
            }
            u.peak = tc;
        }
        let boot = Duration::from_secs(1000);
        std::fs::write(&path, state::encode_with(boot, boot, &sp, Some(&e), false, true)).unwrap();
        path
    }

    fn asahi(g: &Guard) -> &Vec<crate::engine::AsahiSpeaker> {
        g.engine.as_ref().unwrap().asahi.as_ref().unwrap()
    }

    fn idx(cfg: &Config, name: &str) -> usize {
        cfg.speakers.iter().position(|s| s.name == name).unwrap()
    }

    /// The guard never believes a control holds more reduction than it does.
    fn belief_holds(g: &Guard, hw: &FakeHw) {
        for (k, (&w, &p)) in g.written.iter().zip(&hw.per).enumerate() {
            assert!(!w.is_finite() || w <= p + 1e-9, "speaker {k}: believed {w} dB, the control holds {p} dB at {:?}", hw.t);
        }
    }

    /// The worst case of speaker `k`'s measured-power models at `db`.
    fn vi_worst(cfg: &Config, k: usize, db: f64) -> f64 {
        let v = crate::engine::v_worst(&cfg.speakers[k], &cfg.globals) * 10f64.powf(-db / 20.0);
        v * v / cfg.speakers[k].z_nominal.min(cfg.speakers[k].r_ff)
    }

    /// The V/I path end to end: a quiet program, per-speaker controls.
    #[test]
    fn vi_healthy_playback_keeps_the_lock_at_full_scale() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.03);
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
        assert_eq!(g.governing, Governor::Asahi);
        run_until(&mut g, &mut hw, secs(10.0)).unwrap();
        assert!(hw.lapses.is_empty(), "{:?}", hw.lapses);
        assert!(hw.max_ping_gap(secs(0.3), hw.t) < LOCK_TIMEOUT);
        assert!(g.armed() && hw.per.iter().all(|&p| p == 0.0), "{:?}", hw.per);
        assert_eq!(hw.opens, 1);
        assert!(!g.engine.as_ref().unwrap().hard(Governor::Asahi));
    }

    /// Review F1, volume route, with the asahi governor: 160 failed writes
    /// left 0 dB of 6.2 dB applied and the lock renewed 54 times.  The first
    /// failure gives the lock up, nothing renews while the control fails,
    /// and the guard never believes more reduction than a control holds.
    #[test]
    fn vi_a_failing_volume_control_hands_the_card_to_the_kernel() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.06);
        hw.volume_fail = Box::new(|t| (secs(2.0)..secs(5.0)).contains(&t));
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 122.0, 95.0))).unwrap();
        while hw.t < secs(30.0) {
            g.tick(&mut hw).unwrap();
            belief_holds(&g, &hw);
            if (secs(2.1)..secs(5.0)).contains(&hw.t) {
                assert!(!g.armed());
                assert!((0..cfg.speakers.len()).all(|k| hw.effective_of(k) >= hw.safe), "the card plays above its safe level at {:?}", hw.t);
            }
        }
        let renewed: Vec<_> = hw.pings.iter().filter(|&&p| p >= secs(2.0) && p < secs(5.0)).collect();
        assert!(renewed.is_empty(), "renewed while the volume control failed: {renewed:?}");
        assert!(hw.relinquished >= 1, "the hand-off did not give the lock up");
        assert!(g.armed() && hw.unlocked_at(hw.t), "taken back once the control works");
        let want = g.engine.as_ref().unwrap().reductions(Governor::Asahi);
        assert!(hw.per.iter().zip(&want).all(|(p, w)| (p - w).abs() < 0.5), "the governor's reductions are written after recovery: {:?} vs {want:?}", hw.per);
    }

    /// Review F1: a failed per-speaker write leaves each control at the old
    /// value or the new one, so each counts at the smaller reduction, and
    /// an unknown control stays unknown (full gain).  Charges use each
    /// control's own least.
    #[test]
    fn a_failed_per_speaker_write_keeps_the_smaller_reduction_of_each_control() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 60.0, 55.0))).unwrap();
        g.written = vec![10.0; 4];
        g.written_hi = g.written.clone();
        g.write_failed(&[2.0, 2.0, 6.0, 6.0], 0.0);
        assert_eq!(g.written, vec![2.0, 2.0, 6.0, 6.0]);
        g.write_failed(&[8.0; 4], 0.0);
        assert_eq!(g.written, vec![2.0, 2.0, 6.0, 6.0]);
        let t0 = hw.now();
        g.gains = vec![Gain { at: t0, lo: g.written.clone(), hi: g.written.clone(), kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
        g.verified_at = t0;
        g.account_to = t0;
        let before = asahi(&g).clone();
        g.charge(t0 + Duration::from_millis(50)).unwrap();
        for (k, s) in before.iter().enumerate() {
            let mut want = s.clone();
            want.th.hold(vi_worst(&cfg, k, g.written[k]), 0.05, cfg.globals.t_ambient);
            assert!((asahi(&g)[k].th.t_coil - want.th.t_coil).abs() < 1e-9, "speaker {k}");
        }
        g.written = vec![f64::INFINITY; 4];
        g.written_hi = g.written.clone();
        g.write_failed(&[8.0; 4], 0.0);
        assert!(g.written.iter().all(|w| w.is_infinite()), "an unknown control became a known one");
        // in the loop: a partial write hands the card over at once, and the
        // belief never runs ahead of a control
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.06);
        hw.partial_fail = Box::new(|t| if (secs(2.0)..secs(3.0)).contains(&t) { Some(2) } else { None });
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 122.0, 95.0))).unwrap();
        let mut handed = false;
        while hw.t < secs(4.0) {
            g.tick(&mut hw).unwrap();
            belief_holds(&g, &hw);
            handed |= hw.t >= secs(2.0) && hw.t < secs(3.0) && !g.armed();
        }
        assert!(handed, "a partial write did not hand the card over");
    }

    /// Review F1, sample-rate route, on a V/I machine: unknown playback
    /// state from the start never unlocks and never opens capture blind;
    /// the time is charged at full scale (at the unknown controls' full
    /// gain), so the lock comes back only once that has cooled.
    #[test]
    fn vi_a_failing_rate_control_never_unlocks() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.rate_fail = Box::new(|t| t < secs(2.0));
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 60.0, 55.0))).unwrap();
        run_until(&mut g, &mut hw, secs(2.0)).unwrap();
        assert!(hw.pings.is_empty(), "unlocked {} times with the playback state unknown", hw.pings.len());
        assert!(!g.armed() && hw.opens == 0);
        assert!(asahi(&g).iter().all(|a| a.hard), "two unknown seconds at full scale did not latch the models");
        run_until(&mut g, &mut hw, secs(600.0)).unwrap();
        assert!(g.armed() && hw.unlocked_at(hw.t), "never taken back");
    }

    /// Review F2: a valid 139 C state and a damaged file both started
    /// armed at 0 dB.  The governor's output is computed in start; the
    /// 139 C start needs the kernel and does not unlock until the ramp
    /// allows; the damaged one starts latched at the limit.
    #[test]
    fn vi_a_hot_or_damaged_start_waits_for_the_ramp() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.playing = Box::new(|_| false);
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 139.0, 105.0))).unwrap();
        let demand = |g: &Guard| g.engine.as_ref().unwrap().reductions(Governor::Asahi).into_iter().fold(0.0, f64::max);
        assert!(demand(&g) >= cfg.globals.max_reduction, "the ramp was not computed at start: {:.2} dB", demand(&g));
        assert!(hw.pings.is_empty(), "unlocked at start");
        let mut last = demand(&g);
        while hw.pings.is_empty() && hw.t < secs(60.0) {
            g.tick(&mut hw).unwrap();
            last = demand(&g);
        }
        assert!(!hw.pings.is_empty(), "never armed");
        assert!(last < cfg.globals.max_reduction - REARM_MARGIN_DB, "unlocked with the ramp at {last:.2} dB");
        assert!(hw.t > secs(1.0), "unlocked after {:?}", hw.t);
        let path = vi_state(&cfg, 60.0, 55.0);
        std::fs::write(&path, "damaged").unwrap();
        let mut hw = FakeHw::per_speaker(&cfg);
        let g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(path)).unwrap();
        assert!(asahi(&g).iter().all(|a| a.hard && a.th.t_coil >= 150.0));
        assert!(hw.pings.is_empty());
        assert!(demand(&g) >= cfg.globals.max_reduction);
    }

    /// Review F3: half the periods with negative V*I kept the card armed at
    /// 0 dB, and their time was neither measured nor charged (the tweeter
    /// model cooled from 119 to 104.6 C).  A bad period is uncovered time:
    /// charged at full scale at the least gain held, never measured, and
    /// the card is handed off at once.  At the J413 tweeter's sensitivity
    /// that charge latches the model.
    #[test]
    fn vi_bad_sense_data_hands_off_at_once_and_is_charged() {
        let cfg = j413();
        let ch = cfg.globals.channels;
        let negate = |cfg: &Config| {
            let ics: Vec<usize> = cfg.speakers.iter().map(|s| s.is_chan.unwrap()).collect();
            move |w: &mut [i32]| {
                for f in w.chunks_exact_mut(ch) {
                    for &ic in &ics {
                        f[ic] = -f[ic];
                    }
                }
            }
        };
        let run = |bad: bool| {
            let mut hw = FakeHw::per_speaker(&cfg);
            hw.level = Box::new(|_| 0.04);
            let flip = negate(&cfg);
            hw.tweak = Some(Box::new(move |n, w| {
                if bad && n >= 20 && n % 2 == 1 {
                    flip(w);
                }
            }));
            let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
            let (mut seen, mut latched) = (0, false);
            while hw.t < secs(9.0) {
                let before = hw.delivered;
                let loud = (0..cfg.speakers.len()).any(|k| hw.effective_of(k) < hw.safe);
                g.tick(&mut hw).unwrap();
                latched |= asahi(&g).iter().any(|a| a.hard);
                if bad && loud && hw.delivered > before && before >= 20 && before % 2 == 1 {
                    seen += 1;
                    assert!(!g.armed(), "armed after bad sense data at {:?}", hw.t);
                    assert!((0..cfg.speakers.len()).all(|k| hw.effective_of(k) >= hw.safe), "the card plays above its safe level at {:?}", hw.t);
                }
            }
            (g, hw, seen, latched)
        };
        let (good, ghw, _, good_latched) = run(false);
        let (bad, bhw, seen, bad_latched) = run(true);
        assert!(ghw.delivered >= 100 && bhw.delivered >= 100);
        assert!(!good_latched && good.armed());
        assert!(seen >= 1, "no bad period arrived at full level");
        assert!(bad_latched, "bad periods charged at less than the gain the card held");
        let k = idx(&cfg, "Left Woofer");
        assert!(asahi(&bad)[k].th.t_coil > asahi(&good)[k].th.t_coil + 1.0, "{:.2} vs {:.2} C", asahi(&bad)[k].th.t_coil, asahi(&good)[k].th.t_coil);
        // a single bad period: the lock comes back only after two good ones
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.02);
        let flip = negate(&cfg);
        hw.tweak = Some(Box::new(move |n, w| {
            if n == 20 {
                flip(w);
            }
        }));
        let mut g = Guard::start(&mut hw, &cfg, 40.0, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
        let mut bad_at = None;
        while hw.t < secs(20.0) {
            g.tick(&mut hw).unwrap();
            if bad_at.is_none() && hw.delivered == 21 {
                bad_at = Some(hw.t);
                assert!(!g.armed());
            }
        }
        let bad_at = bad_at.unwrap();
        let back = hw.pings.iter().copied().find(|&p| p > bad_at).expect("never taken back");
        assert!(back + Duration::from_millis(1) >= bad_at + secs(2.0 * 4096.0 / 48000.0), "taken back at {back:?}, bad period at {bad_at:?}");
    }

    fn word(x: f64, fs: f64) -> i32 {
        (((x / fs * 32768.0).round().clamp(-32768.0, 32767.0)) as i16 as i32) << 16
    }

    /// Review F4 at the loop: the +25 W / -20 W period ends at 122 C after
    /// a 175 C peak, and the guard stayed armed at 2.1 dB.  The latch is set
    /// inside the period and the card goes to the kernel before any renewal.
    #[test]
    fn vi_an_intra_period_peak_hands_the_card_to_the_kernel() {
        let cfg = j413();
        let k = idx(&cfg, "Left Tweeter");
        let (vc, ic) = (cfg.speakers[k].vs_chan, cfg.speakers[k].is_chan.unwrap());
        let ch = cfg.globals.channels;
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.02);
        hw.tweak = Some(Box::new(move |n, w| {
            if n == 30 {
                for (i, f) in w.chunks_exact_mut(ch).enumerate() {
                    let p = if i < 1920 { 25.0 } else { -20.0 };
                    f[vc] = word(7.0, 14.0);
                    f[ic] = word(p / 7.0, 3.75);
                }
            }
        }));
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 119.0, 50.0 + 69.0 * 200.0 / 304.9))).unwrap();
        let mut at = None;
        while hw.t < secs(4.0) {
            let pings = hw.pings.len();
            g.tick(&mut hw).unwrap();
            if asahi(&g).iter().any(|a| a.hard) {
                assert_eq!(hw.pings.len(), pings, "renewed with a latched model at {:?}", hw.t);
            }
            if at.is_none() && hw.delivered == 31 {
                at = Some(hw.t);
                assert!(asahi(&g)[k].hard, "the crossing did not latch");
                assert!(!g.armed(), "armed after the model may have reached the hard limit");
                assert!(asahi(&g)[k].th.t_coil >= 150.0, "the period's last samples cooled the model back");
            }
        }
        let at = at.unwrap();
        assert!(!hw.pings.iter().any(|&p| p >= at && p < at + secs(1.0)), "renewed after the latch");
    }

    /// Review F5 and R8 on a V/I machine: a missing state started the asahi
    /// models at speakersafetyd's 119 C cold boot.  It starts latched at the
    /// limit, and so does a clean stop of a run without the engine.
    #[test]
    fn vi_a_missing_state_starts_latched() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        let g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, missing_state()).unwrap();
        assert!(asahi(&g).iter().all(|a| a.hard && a.th.t_coil >= 150.0), "{:?}", asahi(&g)[0].th);
        let path = tmp_state().unwrap();
        let mut sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), cfg.globals.t_ambient)).collect();
        for s in sp.iter_mut() {
            s.resume(60.0, 55.0, 0.0, 0.0);
        }
        std::fs::write(&path, state::encode(hw.boot, hw.boot, &sp, false, true)).unwrap();
        let mut hw = FakeHw::per_speaker(&cfg);
        let g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(path)).unwrap();
        assert!(asahi(&g).iter().all(|a| a.hard));
        assert!(g.speakers.iter().all(|s| !s.hard && s.t_coil < 61.0));
    }

    /// Review F5 and r3 T4 on the engine: a held snapshot is not resumed.
    /// 0.1.6 (and PR #6) cooled the time after it as silence; whatever the
    /// paused lease let play is not bounded by it.
    #[test]
    fn vi_a_held_snapshot_restarts_latched() {
        let cfg = j413();
        let path = vi_state(&cfg, 119.0, 95.0);
        let text = std::fs::read_to_string(&path).unwrap().replace("armed 0\nreleased 1", "armed 1\nreleased 0");
        std::fs::write(&path, text).unwrap();
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.boot += secs(1.0);
        let g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(path)).unwrap();
        assert!(asahi(&g).iter().all(|a| a.hard && a.th.t_coil >= 150.0));
    }

    /// Review F5: speakersafetyd's own warm-boot flag says it ran this boot
    /// and may have driven the speakers after this daemon's last state.  A
    /// clean stop's state of this daemon does not outweigh it.
    #[test]
    fn speakersafetyds_flag_starts_every_model_latched() {
        let cfg = j413();
        let path = vi_state(&cfg, 55.0, 52.0);
        let flag = path.with_file_name("speakersafetyd.flag");
        let mut hw = FakeHw::per_speaker(&cfg);
        let g = Guard::start_with(&mut hw, &cfg, cfg.globals.max_reduction, Some(path.clone()), std::slice::from_ref(&flag)).unwrap();
        assert!(asahi(&g).iter().all(|a| !a.hard), "latched without the flag");
        std::fs::write(&flag, "started").unwrap();
        let mut hw = FakeHw::per_speaker(&cfg);
        let g = Guard::start_with(&mut hw, &cfg, cfg.globals.max_reduction, Some(path), &[flag]).unwrap();
        assert!(asahi(&g).iter().all(|a| a.hard && a.th.t_coil >= 150.0));
        assert!(g.speakers.iter().all(|s| s.hard));
    }

    /// Review F6 on the engine: a complete period after lost time erased it
    /// (the tweeter cooled from 120.4 to 119.3 C).  The loss is charged,
    /// which on the J413's tweeter at full gain latches it; a queued
    /// backlog is measured, not charged.
    #[test]
    fn vi_a_period_after_lost_time_does_not_erase_it() {
        let cfg = j413();
        let k = idx(&cfg, "Left Woofer");
        let run = |delay: bool, lossy: bool| {
            let mut hw = FakeHw::per_speaker(&cfg);
            hw.level = Box::new(|_| 0.03);
            hw.lossy = lossy;
            if delay {
                hw.control_delay = Box::new(|t| if (secs(2.0)..secs(2.05)).contains(&t) { Duration::from_millis(240) } else { Duration::ZERO });
            }
            let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
            run_until(&mut g, &mut hw, secs(2.6)).unwrap();
            (asahi(&g)[k].th.t_coil, asahi(&g).iter().any(|a| a.hard))
        };
        let ((clean, _), (queued, queued_hard), (lost, lost_hard)) = (run(false, false), run(true, false), run(true, true));
        assert!((queued - clean).abs() < 0.5 && !queued_hard, "a queued backlog was charged: {queued:.2} vs {clean:.2} C");
        assert!(lost > clean + 5.0, "lost time was not charged: {lost:.2} vs {clean:.2} C");
        assert!(lost_hard, "155 ms at full gain did not latch the tweeter's model");
    }

    /// Review R6 on the engine: 1.5 ms losses before silent periods are
    /// charged, not filled with the next period's power.
    #[test]
    fn vi_small_gaps_are_charged_on_the_engine_too() {
        let cfg = j413();
        let k = idx(&cfg, "Left Woofer");
        let mut hw = FakeHw::per_speaker(&cfg);
        let mut g = Guard::start(&mut hw, &cfg, 40.0, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
        g.rate = 48000;
        g.written = vec![0.0; 4];
        g.written_hi = g.written.clone();
        let t0 = hw.now();
        g.gains = vec![Gain { at: t0, lo: vec![0.0; 4], hi: vec![0.0; 4], kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
        g.verified_at = t0;
        g.account_to = t0;
        for w in g.words.iter_mut() {
            *w = 0;
        }
        let mut reference = Engine::new(&cfg, false).unwrap();
        reference.asahi.as_mut().unwrap()[k] = asahi(&g)[k].clone();
        let zeros = vec![0i32; cfg.globals.channels * 4096];
        let dt = Duration::from_secs_f64(4096.0 / 48000.0);
        let mut end = t0;
        for _ in 0..200 {
            end += Duration::from_micros(1500) + dt;
            g.measure(end, 4096).unwrap();
            reference.worst(0.0015, &[0.0; 4]);
            reference.measured(&zeros, 4096, 4096, 4096, 0, dt.as_secs_f64() / 4096.0, 1.0, &vec![vec![(0.0, 0.0)]; 4]);
        }
        let want = reference.asahi.as_ref().unwrap()[k].th.t_coil;
        let got = asahi(&g)[k].th.t_coil;
        // the same charges and frames (to the nanoseconds of the clock)
        assert!(got >= want - 1e-3, "after losses {got:.4} C, charged {want:.4} C");
        assert!(got > 55.0 + 1.0, "{got}");
    }

    /// Review F8: the unqualified machines (the J616, and the J316 and J413
    /// until they are qualified on the device) are not governed, and their
    /// card is not touched, without --allow-unqualified; udev starts the
    /// daemon on none of them.
    #[test]
    fn an_unqualified_machine_is_not_governed() {
        for text in [J616, J316, J413] {
            let mut cfg = Config::parse(text).unwrap();
            let mut hw = FakeHw::per_speaker(&cfg);
            assert!(Guard::start(&mut hw, &cfg, 7.0, tmp_state()).is_err());
            assert!(!hw.locked, "the interlock was taken");
            cfg.allow_unqualified = true;
            assert!(Guard::start(&mut hw, &cfg, 7.0, tmp_state()).is_ok());
        }
        let rules = include_str!("../95-speakerguardd.rules");
        let starts: Vec<&str> = rules.lines().filter(|l| l.contains("SYSTEMD_WANTS")).collect();
        for m in ["J616", "J316", "J413"] {
            assert!(starts.iter().all(|l| !l.contains(m)), "udev starts the daemon on the {m}: {starts:?}");
        }
        assert!(starts.iter().any(|l| l.contains("AppleJ700")));
    }

    /// The unified governor is shadow-only: it does not govern without its
    /// opt-in, and as a shadow it changes nothing the legacy governor does.
    #[test]
    fn the_unified_governor_is_shadow_only() {
        let mut cfg = j700();
        cfg.globals.governor = Governor::Unified;
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        assert!(Guard::start(&mut hw, &cfg, 20.0, tmp_state()).is_err());
        cfg.allow_unified = true;
        let g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        assert_eq!(g.governing, Governor::Unified);
        let writes = |shadow: bool| {
            let mut cfg = j700();
            cfg.globals.shadow = shadow;
            let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
            hw.level = Box::new(|_| 0.6);
            let path = tmp_state();
            let mut g = Guard::start(&mut hw, &cfg, 20.0, path.clone()).unwrap();
            run_until(&mut g, &mut hw, secs(15.0)).unwrap();
            let log = path.unwrap().with_file_name("shadow.jsonl");
            (hw.writes.clone(), hw.pings.clone(), std::fs::read_to_string(log).map(|t| t.lines().count()).unwrap_or(0))
        };
        let ((w0, p0, l0), (w1, p1, l1)) = (writes(false), writes(true));
        assert_eq!(w0, w1, "the shadow changed the legacy governor's writes");
        assert_eq!(p0, p1);
        assert_eq!(l0, 0);
        assert!(l1 >= 10, "{l1} shadow lines");
    }

    /// Group linking and per-speaker writes: a hot tweeter and cool woofers
    /// on the J316 get their own group's reduction, the same on both sides.
    #[test]
    fn per_speaker_reductions_are_linked_by_group() {
        let mut cfg = Config::parse(J316).unwrap();
        cfg.allow_unqualified = true;
        let path = vi_state(&cfg, 55.0, 52.0);
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.playing = Box::new(|_| false);
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(path)).unwrap();
        run_until(&mut g, &mut hw, secs(1.0)).unwrap();
        assert!(g.armed());
        let (lt, rt, lw) = (idx(&cfg, "Left Tweeter"), idx(&cfg, "Right Tweeter"), idx(&cfg, "Left Woofer 1"));
        {
            let a = &mut g.engine.as_mut().unwrap().asahi.as_mut().unwrap()[lt];
            a.th.t_coil = 124.0;
            a.th.t_magnet = 123.0;
            a.coil_peak = 124.0;
            a.magnet_peak = 123.0;
        }
        run_until(&mut g, &mut hw, secs(1.3)).unwrap();
        assert!(g.armed());
        assert!(hw.per[lt] > 0.5 && hw.per[lt] == hw.per[rt], "{:?}", hw.per);
        assert_eq!(hw.per[lw], 0.0, "{:?}", hw.per);
    }

    /// The J316 under the unified governor, with the asahi guard or (with
    /// the experimental flag) without it, at the given program level.
    fn unified_j316(guard: bool, level: f64) -> (Config, FakeHw) {
        let mut cfg = Config::parse(J316).unwrap();
        cfg.allow_unqualified = true;
        cfg.globals.governor = Governor::Unified;
        if guard {
            cfg.globals.guard = Some(Governor::Asahi);
        } else {
            cfg.allow_unified = true;
        }
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(move |_| level);
        (cfg, hw)
    }

    /// Review r4 of speakerguardd#8, the asahi guard: with the unified
    /// governor writing the card, every period each speaker's control
    /// holds the larger of the two governors' reductions, group-linked:
    /// the asahi ramp's on hot tweeters, the unified regulator's on woofers
    /// whose feed-forward model it predicts past its target.  The asahi
    /// model's latch alone hands the card to the kernel.  Without the guard
    /// the tweeters get only unified's.
    #[test]
    fn the_asahi_guard_gives_each_speaker_the_larger_reduction() {
        let start = |guard: bool| {
            let (cfg, mut hw) = unified_j316(guard, 0.1);
            let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
            run_until(&mut g, &mut hw, secs(1.0)).unwrap();
            assert!(g.armed());
            let tweeters: Vec<usize> = (0..cfg.speakers.len()).filter(|&k| cfg.speakers[k].name.contains("Tweeter")).collect();
            let e = g.engine.as_mut().unwrap();
            for (k, s) in cfg.speakers.iter().enumerate() {
                if tweeters.contains(&k) {
                    let a = &mut e.asahi.as_mut().unwrap()[k];
                    (a.th.t_coil, a.th.t_magnet, a.coil_peak, a.magnet_peak) = (126.0, 120.0, 126.0, 120.0);
                } else {
                    let u = &mut e.unified.as_mut().unwrap()[k];
                    (u.ff.t_coil, u.ff.t_magnet) = (139.0, 135.0);
                }
                let _ = s;
            }
            (cfg, hw, g, tweeters)
        };
        let (cfg, mut hw, mut g, tweeters) = start(true);
        let mut both = false;
        while hw.t < secs(3.0) {
            g.tick(&mut hw).unwrap();
            assert!(g.armed(), "handed off at {:?}", hw.t);
            let e = g.engine.as_ref().unwrap();
            let (u, a) = (e.reductions(Governor::Unified), e.reductions(Governor::Asahi));
            for (k, s) in cfg.speakers.iter().enumerate() {
                assert_eq!(hw.per[k], u[k].max(a[k]), "{} at {:?}: the card {} dB, unified {} dB, asahi {} dB", s.name, hw.t, hw.per[k], u[k], a[k]);
            }
            let woofers = (0..u.len()).filter(|k| !tweeters.contains(k));
            both |= tweeters.iter().all(|&k| a[k] > u[k] + 1.0) && woofers.clone().all(|k| u[k] > a[k] + 1.0);
        }
        assert!(both, "the two governors never each bound a group of their own");
        // without the guard the tweeters get unified's
        let (_, mut hw2, mut g2, tweeters2) = start(false);
        run_until(&mut g2, &mut hw2, secs(1.5)).unwrap();
        let e = g2.engine.as_ref().unwrap();
        let (u, a) = (e.reductions(Governor::Unified), e.reductions(Governor::Asahi));
        assert!(tweeters2.iter().all(|&k| hw2.per[k] == u[k] && a[k] > u[k] + 1.0), "card {:?}, unified {u:?}, asahi {a:?}", hw2.per);
        // the asahi model's latch alone hands the guarded card over, with
        // both governors asking for less than the hand-over
        g.engine.as_mut().unwrap().asahi.as_mut().unwrap()[tweeters[0]].hard = true;
        let e = g.engine.as_ref().unwrap();
        assert!(!e.hard(Governor::Unified));
        assert!(e.governed(Governor::Unified).0.iter().all(|&r| r < cfg.globals.max_reduction - 1.0));
        g.tick(&mut hw).unwrap();
        assert!(!g.armed(), "the asahi latch did not hand the card over");
    }

    /// Review r4 of speakerguardd#8: under the asahi guard the unified
    /// regulator holds the feed-forward model only, the measured-power side
    /// being the guard's.  A hot measured-power model with a cool
    /// feed-forward one asks nothing of the guarded regulator (the card
    /// still gets the guard's ramp, and the model's latch), where alone
    /// (the experimental flag) the regulator tightens on it.
    #[test]
    fn under_the_guard_the_regulator_holds_the_feed_forward_model() {
        let demand = |guard: bool| {
            let (cfg, mut hw) = unified_j316(guard, 0.1);
            let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
            run_until(&mut g, &mut hw, secs(1.0)).unwrap();
            for u in g.engine.as_mut().unwrap().unified.as_mut().unwrap() {
                let v = u.vi.as_mut().unwrap();
                (v.t_coil, v.t_magnet) = (132.0, 128.0);
            }
            run_until(&mut g, &mut hw, secs(1.2)).unwrap();
            assert!(g.armed());
            let e = g.engine.as_ref().unwrap();
            let tweeters: Vec<usize> = (0..cfg.speakers.len()).filter(|&k| cfg.speakers[k].name.contains("Tweeter")).collect();
            (e.reductions(Governor::Unified), e.reductions(Governor::Asahi), hw.per.clone(), tweeters)
        };
        let (alone, _, card, tweeters) = demand(false);
        assert!(tweeters.iter().all(|&k| alone[k] > 1.0), "the regulator alone did not tighten on its measured-power model: {alone:?}");
        assert_eq!(card, alone);
        let (guarded, asahi, card, _) = demand(true);
        assert!(guarded.iter().all(|&r| r == 0.0), "the guarded regulator tightened on its measured-power model: {guarded:?}");
        assert_eq!(card, asahi, "the card under the guard is not the guard's ramp");
    }

    /// Review r4 CC1 (addendum 2) of speakerguardd#8: each control's write
    /// is atomic, so a write that fails part way leaves each control at its
    /// old value or the new one, never between them.  The first write after
    /// an unlock, from the unlock's minimum to 0 dB, read as the span from 0
    /// dB to mute, left a full-scale onset under it out of the regulator's
    /// prediction (an onset inside a partial write failure crossed 150 C
    /// 425 times in 7,020 on a dead line, against 173 with the span read
    /// at its most); read at its most, it held an audible program as if it
    /// were full scale (CC2).  As the two values, the onset is the program
    /// at 0 dB and is attacked, and a moderate program is what it is.
    #[test]
    fn a_failed_first_write_after_an_unlock_leaves_either_value() {
        let run = |amp: f64| {
            let (cfg, mut hw) = unified_j316(true, 0.1);
            let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
            let n = cfg.speakers.len();
            // the unlock's minimum, held
            g.armed = true;
            g.written = vec![MUTE_DB; n];
            g.written_hi = g.written.clone();
            g.written_set = vec![vec![(MUTE_DB, MUTE_DB)]; n];
            let t0 = hw.now();
            g.gains = vec![Gain { at: t0, lo: g.written.clone(), hi: g.written.clone(), kernel: false, cands: g.candidates() }];
            // the first write, 0 dB, fails part way
            g.write_failed(&vec![0.0; n], 0.5);
            g.note_written(t0 + secs(0.01));
            assert_eq!((g.written[0], g.written_hi[0]), (0.0, MUTE_DB));
            let possible = g.possible_reductions(t0, t0 + secs(0.2));
            for p in &possible {
                assert!(p.contains(&(0.0, 0.5)) && p.contains(&(MUTE_DB, MUTE_DB)) && p.iter().all(|r| r.1 - r.0 <= 0.5), "{p:?}");
            }
            // a period of a sine of `amp` volts peak at 1 kHz on every speaker
            let (ch, period) = (cfg.globals.channels, cfg.globals.period);
            let mut w = vec![0i32; ch * period];
            for f in 0..period {
                let v = amp * (2.0 * std::f64::consts::PI * 1000.0 * f as f64 / 48000.0).sin();
                for sp in &cfg.speakers {
                    w[f * ch + sp.vs_chan] = ((v / sp.vs_scale * 32767.0).round() as i32) << 16;
                    w[f * ch + sp.is_chan.unwrap()] = ((v / 4.0 / sp.is_scale * 32767.0).round() as i32) << 16;
                }
            }
            let e = g.engine.as_mut().unwrap();
            e.govern(0.0, false);
            for _ in 0..2 {
                e.measured(&w, period, period, period, 0, 1.0 / 48000.0, 1.0, &possible);
            }
            e.reductions(Governor::Unified)
        };
        let cfg = Config::parse(J316).unwrap();
        // the amplifier's loudest sine: attacked at once
        let full = run(2f64.sqrt() * 10f64.powf(cfg.globals.amp_gain_dbv / 20.0));
        assert!(full.iter().all(|&r| r > 3.0), "a full-scale onset under the failed write: {full:?}");
        // 2.5 V peak, about 1 W into 3 ohm: nothing to hold
        let moderate = run(2.5);
        assert!(moderate.iter().all(|&r| r == 0.0), "a moderate program under the failed write: {moderate:?}");
    }

    /// Review r4 of speakerguardd#8: a guarded start that did not follow a
    /// clean stop, with a stream already playing (a crash restart), takes
    /// the lock back once the latched start has cooled.  The first half
    /// second may have been converted at any value, and read as full scale
    /// it would keep the regulator over the hand-over for as long as the
    /// stream plays.  On these cards it cannot be: the owner's death drops
    /// its lock and the amplifiers apply the card's safe level as they
    /// play, so those frames are 100 dB down and never the prediction.  A
    /// regression test, which holds with or without
    /// frames_at_an_ambiguous_gain_are_not_the_prediction's rule.
    #[test]
    fn a_guarded_start_into_a_playing_stream_takes_the_lock_back() {
        let (cfg, mut hw) = unified_j316(true, 0.1);
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, missing_state()).unwrap();
        assert!(g.engine.as_ref().unwrap().governed(Governor::Unified).1, "not a latched start");
        run_until(&mut g, &mut hw, secs(150.0)).unwrap();
        assert!(g.armed(), "still at the card's safe level: unified {:?}, asahi {:?}", g.engine.as_ref().unwrap().reductions(Governor::Unified), g.engine.as_ref().unwrap().reductions(Governor::Asahi));
    }

    /// Review r4 BB1 of speakerguardd#8: a current line that reads nothing
    /// while its voltage carries a program (ISENSE off, a dead line) is
    /// lost, and the measured power is bounded by V^2 over the speaker's
    /// smallest impedance, the most a live line could read.  Under the
    /// guard and under the asahi governor alone the card is held as a live
    /// line at that impedance holds it.  Before, the asahi models saw no
    /// power: the asahi governor alone played on at 0 dB while the
    /// feed-forward model passed its hard limit, and the guard added
    /// nothing.
    #[test]
    fn a_dead_current_line_is_governed_as_the_most_current_there_can_be() {
        let run = |guard: bool, dead: bool, level: f64| {
            let (mut cfg, mut hw) = unified_j316(guard, level);
            if !guard {
                cfg.globals.governor = Governor::Asahi;
                cfg.globals.shadow = true;
            }
            let lines: Vec<(usize, usize, f64)> = cfg.speakers.iter().map(|s| (s.vs_chan, s.is_chan.unwrap(), s.vs_scale / s.is_scale / s.z_min())).collect();
            let ch = cfg.globals.channels;
            hw.tweak = Some(Box::new(move |_, w: &mut [i32]| {
                for f in w.chunks_exact_mut(ch) {
                    for &(vc, ic, k) in &lines {
                        f[ic] = if dead { 0 } else { (f[vc] as f64 * k).round().clamp(i32::MIN as f64, i32::MAX as f64) as i32 };
                    }
                }
            }));
            let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
            let (mut cards, mut hottest) = (Vec::new(), 0.0f64);
            while hw.t < secs(120.0) {
                g.tick(&mut hw).unwrap();
                let e = g.engine.as_ref().unwrap();
                cards.push(if g.armed() { hw.per.clone() } else { vec![f64::INFINITY; cfg.speakers.len()] });
                hottest = (0..cfg.speakers.len()).map(|k| e.temps(k).1).fold(hottest, f64::max);
            }
            let lost = g.engine.as_ref().unwrap().lost.clone();
            (cards, hottest, lost, g)
        };
        for guard in [true, false] {
            // loud enough for the asahi ramp to ask 6.5 dB, not the 7 that
            // hands the card off
            let (dead, hot_dead, lost_dead, g) = run(guard, true, 0.25);
            let (live, hot_live, lost_live, _) = run(guard, false, 0.25);
            assert!(lost_dead.iter().all(|&l| l) && lost_live.iter().all(|&l| !l), "{lost_dead:?} / {lost_live:?}");
            assert!(g.armed() && dead.iter().skip(10).all(|c| c.iter().all(|x| x.is_finite())), "guard {guard}: the dead line handed the card off");
            assert_eq!(dead.len(), live.len());
            let worst = dead.iter().zip(&live).flat_map(|(d, l)| d.iter().zip(l).map(|(d, l)| l - d)).fold(0.0f64, f64::max);
            assert!(worst <= 0.01, "guard {guard}: the dead line was held up to {worst} dB less than a live line at the smallest impedance");
            assert!(live.last().unwrap().iter().any(|&p| p > 1.0), "guard {guard}: the live line at the smallest impedance asked for nothing: {:?}", live.last());
            assert!((hot_dead - hot_live).abs() < 0.5, "guard {guard}: {hot_dead} against {hot_live} C");
        }
    }

    /// r3 T2 and T3 on the engine: the hand-off gives the lock up and writes
    /// nothing, and time around it is charged at each control's least
    /// reduction since the last verified boundary: woofers at 0 dB and
    /// tweeters at 6 dB stay so, before the hand-off and after it.
    #[test]
    fn vi_time_around_a_hand_off_is_charged_per_speaker_at_the_least_held() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        let mut g = Guard::start(&mut hw, &cfg, 40.0, Some(vi_state(&cfg, 60.0, 55.0))).unwrap();
        let (lt, lw) = (idx(&cfg, "Left Tweeter"), idx(&cfg, "Left Woofer"));
        let mut per = vec![0.0; 4];
        per[lt] = 6.0;
        per[idx(&cfg, "Right Tweeter")] = 6.0;
        g.rate = 48000;
        g.armed = true;
        g.written = per.clone();
        g.written_hi = g.written.clone();
        let t0 = hw.now();
        g.gains = vec![Gain { at: t0, lo: per.clone(), hi: per.clone(), kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
        g.verified_at = t0;
        g.account_to = t0;
        let writes = hw.writes.len();
        hw.t += Duration::from_millis(150);
        g.disarm(&mut hw, "test").unwrap();
        assert_eq!(hw.relinquished, 1, "the hand-off gave the lock up");
        assert_eq!(hw.writes.len(), writes, "the hand-off wrote the controls");
        for d in [0.15, 0.1] {
            let before = asahi(&g).clone();
            hw.t += Duration::from_millis(if d == 0.1 { 100 } else { 0 });
            g.charge(hw.now()).unwrap();
            for k in [lt, lw] {
                let mut want = before[k].clone();
                want.th.hold(vi_worst(&cfg, k, per[k]), d, cfg.globals.t_ambient);
                assert!((asahi(&g)[k].th.t_coil - want.th.t_coil).abs() < 1e-9, "speaker {k} over {d} s");
            }
        }
    }

    /// r3 T1 on the engine: a stream that opens and closes while the event
    /// wake is late is charged whole, for the measured-power models too.
    #[test]
    fn vi_a_late_event_wake_charges_the_whole_interval() {
        let cfg = j413();
        let k = idx(&cfg, "Left Woofer");
        let playing = |t: Duration| (10..90).contains(&(t.as_millis() % 100));
        let mut inner = FakeHw::per_speaker(&cfg);
        inner.playing = Box::new(playing);
        inner.level = Box::new(|_| 0.85);
        let mut hw = DelayedWait { hw: inner };
        let mut g = Guard::start(&mut hw, &cfg, 40.0, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
        let mut reference = asahi(&g)[k].clone();
        let t0 = hw.hw.t0;
        let mut ref_to = g.account_to - t0;
        for _ in 0..120 {
            let gain = hw.hw.effective_of(k);
            g.tick(&mut hw).unwrap();
            let to = g.account_to - t0;
            let mut at = ref_to;
            while at < to {
                let d = Duration::from_micros(500).min(to - at);
                let p = if playing(at) { vi_worst(&cfg, k, gain) } else { 0.0 };
                reference.th.hold(p, d.as_secs_f64(), cfg.globals.t_ambient);
                at += d;
            }
            ref_to = to;
            assert!(asahi(&g)[k].th.t_coil >= reference.th.t_coil - 0.5, "at {to:?}: model {:.2} C, charged reference {:.2} C", asahi(&g)[k].th.t_coil, reference.th.t_coil);
        }
    }

    /// r3 T1 on a V/I machine without events: never armed.
    #[test]
    fn vi_without_control_events_the_lock_is_never_taken() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.events = false;
        hw.playing = Box::new(|t| (10..90).contains(&(t.as_millis() % 100)));
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 55.0, 52.0))).unwrap();
        run_until(&mut g, &mut hw, secs(3.0)).unwrap();
        assert!(hw.pings.is_empty() && !g.armed(), "{} pings", hw.pings.len());
    }

    /// r3 T2 on the engine: a clean stop with pending time accounts it for
    /// the engine's models too before its final state, and that state
    /// resumes them.
    #[test]
    fn vi_a_clean_stop_accounts_the_engine_before_its_final_state() {
        let cfg = j413();
        let k = idx(&cfg, "Left Woofer");
        let path = vi_state(&cfg, 90.0, 80.0);
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 40.0, Some(path.clone())).unwrap();
        run_until(&mut g, &mut hw, secs(1.0)).unwrap();
        hw.sensing = Box::new(|_| false);
        g.tick(&mut hw).unwrap();
        let lag = hw.now() - g.account_to;
        assert!(lag >= secs(0.049));
        let mut reference = asahi(&g)[k].clone();
        reference.th.hold(vi_worst(&cfg, k, 0.0), lag.as_secs_f64(), cfg.globals.t_ambient);
        g.stop(&mut hw).unwrap();
        while !g.released(&hw) {
            g.tick(&mut hw).unwrap();
        }
        g.final_save(&hw).unwrap();
        let saved = state::decode(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert!(saved.proven() && saved.asahi.len() == 4);
        let mut e = Engine::new(&cfg, false).unwrap();
        assert!(state::restore_engine(&mut e, &saved, hw.boottime(), &cfg.globals));
        let restored = e.asahi.as_ref().unwrap()[k].th.t_coil;
        assert!(restored >= reference.th.t_coil - 2.0, "restored {restored:.3} C, the pending interval alone needs {:.3} C", reference.th.t_coil);
    }

    /// r3 T6 on the engine: frames over time already charged add nothing,
    /// and frames partly over it count only for the rest, at the most power
    /// of the blocks that cover it.
    #[test]
    fn vi_frames_over_charged_time_add_nothing_to_the_engine() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        let mut g = Guard::start(&mut hw, &cfg, 40.0, Some(vi_state(&cfg, 123.0, 100.0))).unwrap();
        g.rate = 48000;
        g.written = vec![99.5; 4];
        g.written_hi = g.written.clone();
        let t0 = hw.now();
        g.gains = vec![Gain { at: t0, lo: vec![99.5; 4], hi: vec![99.5; 4], kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
        g.verified_at = t0;
        g.account_to = t0;
        hw.sleep(secs(0.3));
        g.charge(hw.now()).unwrap();
        let after = asahi(&g).clone();
        for w in g.words.iter_mut() {
            *w = 0;
        }
        for i in 1..=3 {
            g.measure(t0 + secs(i as f64 * 4096.0 / 48000.0), 4096).unwrap();
        }
        for (k, a) in after.iter().enumerate() {
            assert_eq!(asahi(&g)[k].th, a.th, "already charged time was integrated again (speaker {k})");
        }
        // partly over: the first half quiet and the second loud, the last
        // quarter not yet accounted: those frames, sample by sample (the
        // oldest of them for its new part only)
        let k = idx(&cfg, "Left Woofer");
        let (vc, ic) = (cfg.speakers[k].vs_chan, cfg.speakers[k].is_chan.unwrap());
        let ch = cfg.globals.channels;
        for (i, f) in g.words.chunks_exact_mut(ch).enumerate() {
            let on = i >= 2048;
            f[vc] = if on { word(4.0, 14.0) } else { 0 };
            f[ic] = if on { word(1.0, 3.75) } else { 0 };
        }
        let quarter = secs(4096.0 / 48000.0 / 4.0);
        let start = g.account_to;
        let before = asahi(&g)[k].clone();
        g.measure(start + quarter, 4096).unwrap();
        let mut want = before.th.clone();
        let p = (word(4.0, 14.0) as f64 / 2147483648.0 * 14.0) * (word(1.0, 3.75) as f64 / 2147483648.0 * 3.75) * crate::engine::ASAHI_MARGIN;
        let new = quarter.as_secs_f64() * 48000.0;
        let n = new.ceil() as usize;
        for i in 0..n {
            let h = if i == 0 { (new - (n - 1) as f64) / 48000.0 } else { 1.0 / 48000.0 };
            let (ac, am) = (h / (want.tau_coil + h), h / (want.tau_magnet + h));
            let (tc, tm) = (want.t_magnet + p * want.tr_coil, cfg.globals.t_ambient + p * want.tr_magnet);
            want.t_coil = tc * ac + want.t_coil * (1.0 - ac);
            want.t_magnet = tm * am + want.t_magnet * (1.0 - am);
        }
        assert!((asahi(&g)[k].th.t_coil - want.t_coil).abs() < 1e-9, "{} vs {}", asahi(&g)[k].th.t_coil, want.t_coil);
        assert_eq!(g.account_to, start + quarter);
    }

    /// Review R4 on the engine: an asahi latch saved with the coil still
    /// over the working ceiling holds after a restart until it is under it.
    #[test]
    fn vi_a_latched_state_is_honoured_after_a_restart() {
        let cfg = j413();
        let path = missing_state().unwrap();
        let g0 = &cfg.globals;
        let mut sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g0.t_ambient)).collect();
        for s in sp.iter_mut() {
            s.resume(60.0, 55.0, 0.0, 0.0);
        }
        let mut e = Engine::new(&cfg, false).unwrap();
        for a in e.asahi.as_mut().unwrap() {
            a.th.t_coil = 60.0;
            a.th.t_magnet = 55.0;
        }
        let k = idx(&cfg, "Left Tweeter");
        {
            let a = &mut e.asahi.as_mut().unwrap()[k];
            a.th.t_coil = 145.0;
            a.th.t_magnet = 100.0;
            a.coil_peak = 152.0;
            a.coil_hyst = 150.0;
            a.hard = true;
        }
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.playing = Box::new(|_| false);
        std::fs::write(&path, state::encode_with(hw.boot, hw.boot, &sp, Some(&e), false, true)).unwrap();
        hw.boot += Duration::from_millis(50);
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(path)).unwrap();
        assert!(asahi(&g)[k].hard, "the latch was lost");
        while hw.pings.is_empty() && hw.t < secs(30.0) {
            g.tick(&mut hw).unwrap();
        }
        assert!(!hw.pings.is_empty(), "never rearmed");
        assert!(asahi(&g)[k].th.t_coil < 140.0, "rearmed at {:.2} C", asahi(&g)[k].th.t_coil);
    }

    // ---- 0.1.9: the PR #7 fourth review's witnesses (r4), as safe outcomes ----

    /// A sample-rate read the daemon is descheduled in: its change event
    /// arrives after the drain that came before the read, and the wait
    /// that follows sees it at once.
    struct ReadRace {
        hw: FakeHw,
        pending: bool,
        slow: bool,
        enable: bool,
    }
    impl Hw for ReadRace {
        fn now(&self) -> Instant { self.hw.now() }
        fn boottime(&self) -> Duration { self.hw.boottime() }
        fn sleep(&mut self, d: Duration) { self.hw.sleep(d) }
        fn take_lock(&mut self) -> io::Result<()> { self.hw.take_lock() }
        fn ping(&mut self) -> io::Result<()> { self.hw.ping() }
        fn set_min(&mut self) -> io::Result<()> { self.hw.set_min() }
        fn set_reduction(&mut self, d: f64) -> io::Result<f64> { self.hw.set_reduction(d) }
        fn set_reductions(&mut self, d: &[f64]) -> io::Result<Vec<f64>> { self.hw.set_reductions(d) }
        fn relinquish(&mut self) -> io::Result<()> { self.hw.relinquish() }
        fn sample_rate(&mut self) -> io::Result<i64> {
            if self.enable {
                if self.slow {
                    // a stream plays inside the read; both ends see closed
                    // speakers and the event missed the drain before it
                    self.hw.sleep(secs(0.1));
                    self.pending = true;
                }
                self.slow = !self.slow;
            }
            Ok(0)
        }
        fn open_sense(&mut self, r: u32) -> io::Result<()> { self.hw.open_sense(r) }
        fn close_sense(&mut self) { self.hw.close_sense() }
        fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> { self.hw.read_period(w) }
        fn queued(&mut self) -> io::Result<u64> { self.hw.queued() }
        fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> { self.hw.read_tail(w) }
        fn wait_rate_change(&mut self, d: Duration) -> bool {
            if self.pending { true } else { self.hw.sleep(d); false }
        }
        fn rate_events(&mut self) -> bool { std::mem::take(&mut self.pending) }
        fn events_available(&self) -> bool { true }
    }

    /// Review r4 U1: a stream inside a delayed sample-rate read was cooled
    /// away: 0.1.8 stayed armed at 0 dB with no capture opened (88 C
    /// against a reference latched at 302 C).  Events are drained again
    /// after the read; one there makes the interval, and the next, charged.
    #[test]
    fn a_stream_inside_a_rate_read_is_charged() {
        let cfg = j700();
        let mut hw = ReadRace { hw: FakeHw::new(2, 4096), pending: false, slow: true, enable: false };
        hw.hw.playing = Box::new(|_| false);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, missing_state()).unwrap();
        // from the real latched start, cooled by verified quiet looks
        for _ in 0..10000 {
            g.tick(&mut hw).unwrap();
            if g.armed() && hw.hw.reduction == 0.0 {
                break;
            }
        }
        assert!(g.armed() && hw.hw.reduction == 0.0);
        let begin = hw.hw.t;
        hw.hw.playing = Box::new(move |t| t >= begin && (10..90).contains(&((t - begin).as_millis() % 125)));
        hw.enable = true;
        let mut reference = g.speakers[0].clone();
        let mut ref_to = g.account_to - hw.hw.t0;
        for _ in 0..60 {
            g.tick(&mut hw).unwrap();
            let to = g.account_to - hw.hw.t0;
            while ref_to < to {
                let d = secs(0.0005).min(to - ref_to);
                let p = if (hw.hw.playing)(ref_to) { reference.worst_case_power(0.0) } else { 0.0 };
                reference.step(p, d.as_secs_f64(), 50.0);
                ref_to += d;
            }
            assert!(g.speakers[0].t_coil >= reference.t_coil - 0.5, "at {:?}: model {:.2} C, the streams inside the reads {:.2} C", hw.hw.t - begin, g.speakers[0].t_coil, reference.t_coil);
            assert!(!(reference.hard && g.armed()), "armed while the streams latched the reference");
        }
        assert!(!g.armed() || hw.hw.reduction > 0.0, "still at full gain after {:?} of hidden streams", hw.hw.t - begin);
    }

    /// The other order: an event that comes after the read (a stream that
    /// opens just after it) is kept for the next look, so a stream that
    /// comes and goes before that look is charged too.
    #[test]
    fn an_event_after_the_rate_read_is_kept_for_the_next_look() {
        let cfg = j700();
        let mut hw = FakeHw::new(2, 4096);
        hw.level = Box::new(|_| 1.0);
        hw.playing = Box::new(|_| false);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(1.0)).unwrap();
        // the next look's events are drained with one pending from after
        // this one's read: it must not be cooled over
        g.event_pending = true;
        let before = g.speakers[0].clone();
        let from = g.account_to;
        g.tick(&mut hw).unwrap();
        let mut want = before.clone();
        want.step_at(want.worst_case_power(0.0), (g.account_to - from).as_secs_f64(), 50.0, 0.0);
        assert!(g.account_to > from && g.speakers[0].t_coil >= want.t_coil - 1e-9, "a pending event was cooled over");
    }

    /// Review r4 U2: a run that could not rewrite the state file left an
    /// earlier clean stop in place, and the run after its crash resumed it
    /// (0.5 dB at 100.8 C where the held run left 116.5 C needing 10.4 dB).
    /// A run now invalidates the clean stop before it can unlock: here the
    /// rewrite fails, so the file is removed, and the next start is latched.
    #[test]
    fn a_held_run_consumes_the_clean_stop_it_resumed() {
        let cfg = j700();
        let p = missing_state().unwrap();
        let warm: Vec<_> = cfg.speakers.iter().map(|s| { let mut x = SpeakerState::new(s.clone(), 50.0); x.resume(100.0, 100.0, 0.0, 0.0); x }).collect();
        std::fs::write(&p, state::encode(secs(1000.0), secs(1000.0), &warm, false, true)).unwrap();
        // only the atomic rewrite fails: the readable clean file would stay
        std::fs::create_dir(p.with_extension("tmp")).unwrap();
        let mut hw = FakeHw::new(2, 4096);
        hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p.clone())).unwrap();
        assert!(!g.speakers[0].hard, "the clean stop was resumed");
        let left = std::fs::read_to_string(&p).ok().and_then(|t| state::decode(&t));
        assert!(left.is_none_or(|s| !s.released), "a clean stop is still there for the next start");
        while !g.armed() {
            g.tick(&mut hw).unwrap();
        }
        hw.level = Box::new(|_| 1.0);
        while hw.t < secs(0.6) {
            g.tick(&mut hw).unwrap();
        }
        let live = g.speakers[0].clone();
        let reboot = hw.boottime();
        drop(g); // died: no final state
        let mut next = FakeHw::new(2, 4096);
        next.boot = reboot;
        next.level = Box::new(|_| 0.0);
        let mut resumed = Guard::start(&mut next, &cfg, 20.0, Some(p)).unwrap();
        assert!(resumed.speakers.iter().all(|s| s.hard && s.t_coil >= s.spec.t_limit), "the next start resumed an old clean stop");
        while !resumed.armed() {
            resumed.tick(&mut next).unwrap();
        }
        let mut reference = live.clone();
        reference.step(0.0, next.t.as_secs_f64(), 50.0);
        reference.govern(&cfg.globals);
        assert!(next.reduction >= reference.reduction_db - 1e-6, "rearmed at {} dB, the held run needs {:.2} dB", next.reduction, reference.reduction_db);
    }

    /// When the clean stop can neither be rewritten nor removed, the run
    /// never unlocks (a later crash would resume it), and unlocks once it
    /// can.
    #[test]
    fn a_clean_stop_that_cannot_be_invalidated_keeps_the_card_locked() {
        let cfg = j700();
        let p = tmp_state().unwrap();
        let mut hw = FakeHw::new(2, 4096);
        hw.playing = Box::new(|_| false);
        STATE_IO_FAILS.with(|f| f.set(true));
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p.clone())).unwrap();
        run_until(&mut g, &mut hw, secs(3.0)).unwrap();
        assert!(hw.pings.is_empty() && !g.armed(), "unlocked with the clean stop still in place");
        assert!(state::decode(&std::fs::read_to_string(&p).unwrap()).unwrap().released);
        STATE_IO_FAILS.with(|f| f.set(false));
        run_until(&mut g, &mut hw, secs(4.0)).unwrap();
        assert!(g.armed());
        assert!(!state::decode(&std::fs::read_to_string(&p).unwrap()).unwrap().released);
    }

    /// Review r4 U4: frames converted at 20 dB (the control has moved to
    /// 0 dB since, within the copy queue) were undone at 0 dB: 0.03 W
    /// before the gain where 3.2 W came, 0 dB where 0.6 dB was needed.
    #[test]
    fn measured_budgets_undo_the_gain_the_frames_may_have_been_converted_at() {
        let cfg = j700();
        let mut hw = FakeHw::new(2, 4096);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        let t0 = hw.now();
        g.rate = 48000;
        g.written = vec![0.0; 2];
        g.written_hi = g.written.clone();
        g.gains = vec![Gain { at: t0 - secs(0.1), lo: vec![20.0; 2], hi: vec![20.0; 2], kernel: false, cands: Vec::new() }, Gain { at: t0, lo: vec![0.0; 2], hi: vec![0.0; 2], kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
        g.verified_at = t0;
        g.account_to = t0;
        for w in &mut g.words {
            *w = (2147483647.0 * 0.1) as i32;
        }
        let mut reference = g.speakers[0].clone();
        let (powers, _) = reference.block_powers(&g.words, 2, SUB_BLOCKS);
        let dt = 4096.0 / 48000.0;
        for i in 1..=3 {
            g.measure(t0 + secs(i as f64 * dt), 4096).unwrap();
            reference.step_period(&powers, dt, 50.0, 20.0);
            reference.govern(&cfg.globals);
            // (the fake clock's nanoseconds round each period down by a hair)
            assert!(g.speakers[0].mean_power(1.0) >= reference.mean_power(1.0) * (1.0 - 1e-7), "{} W before the gain, {} W came", g.speakers[0].mean_power(1.0), reference.mean_power(1.0));
            assert!(g.speakers[0].reduction_db >= reference.reduction_db - 1e-6);
        }
        assert!(reference.reduction_db > 0.5);
    }

    /// The same for the card's own safe level: frames played after a
    /// hand-off may have been converted at it, whatever the controls say.
    #[test]
    fn frames_after_a_hand_off_count_at_the_cards_safe_level() {
        let cfg = j700();
        let mut hw = FakeHw::new(2, 4096);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        let t0 = hw.now();
        g.rate = 48000;
        g.written = vec![0.0; 2];
        g.written_hi = g.written.clone();
        g.gains = vec![Gain { at: t0, lo: vec![0.0; 2], hi: vec![0.0; 2], kernel: true, cands: Vec::new() }];
        g.verified_at = t0;
        g.account_to = t0;
        for w in &mut g.words {
            *w = (2147483647.0 * 0.1) as i32;
        }
        let mut reference = g.speakers[0].clone();
        let (powers, _) = reference.block_powers(&g.words, 2, SUB_BLOCKS);
        let dt = 4096.0 / 48000.0;
        for i in 1..=3 {
            g.measure(t0 + secs(i as f64 * dt), 4096).unwrap();
            reference.step_period(&powers, dt, 50.0, cfg.globals.safe_reduction);
        }
        assert!(g.speakers[0].mean_power(1.0) >= reference.mean_power(1.0) - 1e-9);
    }

    /// The bound is not a blanket one: a block too loud to have come
    /// through the minimum written before an unlock (or any reduction the
    /// card did not apply long) is undone at a reduction that can have
    /// delivered it, and the budgets of a steady stream at full gain after
    /// an unlock stay where its real power puts them.
    #[test]
    fn the_unlock_minimum_does_not_inflate_a_loud_stream() {
        let cfg = j700();
        let mut hw = FakeHw::new(2, 4096);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        let t0 = hw.now();
        g.rate = 48000;
        g.written = vec![0.0; 2];
        g.written_hi = g.written.clone();
        g.gains = vec![Gain { at: t0 - secs(0.2), lo: vec![MUTE_DB; 2], hi: vec![MUTE_DB; 2], kernel: true, cands: Vec::new() }, Gain { at: t0 - secs(0.2), lo: vec![MUTE_DB; 2], hi: vec![MUTE_DB; 2], kernel: false, cands: Vec::new() }, Gain { at: t0 - secs(0.2), lo: vec![0.0; 2], hi: vec![0.0; 2], kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
        g.verified_at = t0;
        g.account_to = t0;
        for w in &mut g.words {
            *w = (2147483647.0 * 0.3) as i32;
        }
        let dt = 4096.0 / 48000.0;
        for i in 1..=3 {
            g.measure(t0 + secs(i as f64 * dt), 4096).unwrap();
        }
        let p = g.speakers[0].power;
        let want = 3.0 * dt * p;
        assert!((g.speakers[0].mean_power(1.0) - want).abs() < want * 1e-6, "{} W, the stream's own {want} W", g.speakers[0].mean_power(1.0));
    }

    /// Review r4 U5: the renewal before the blocking sense open ran after
    /// the open's own charge had latched the hard limit, and renewed it
    /// (one ping at 220 ms, relinquished only at 470 ms).  It now goes
    /// through the same decision as every renewal.
    #[test]
    fn the_renewal_before_a_sense_open_obeys_a_fault_the_charge_just_found() {
        let cfg = j700();
        let mut hw = FakeHw::new(2, 4096);
        hw.playing = Box::new(|_| false);
        hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        g.tick(&mut hw).unwrap();
        assert!(g.armed());
        for s in &mut g.speakers {
            s.resume(110.0, 100.0, 0.0, 0.0);
            s.govern(&cfg.globals);
            assert!(!s.hard && s.reduction_db < 17.0);
        }
        hw.set_reduction(6.0).unwrap();
        g.written = vec![6.0; 2];
        g.written_hi = g.written.clone();
        g.note_written(hw.now());
        hw.sleep(secs(0.195));
        let before = hw.pings.len();
        hw.playing = Box::new(|_| true);
        hw.open_delay = secs(0.2);
        g.tick(&mut hw).unwrap();
        assert!(g.speakers[0].hard, "the open's charge did not latch");
        assert_eq!(hw.pings.len(), before, "renewed after the account latched the hard limit");
        assert!(hw.relinquished > 0 && !g.armed());
    }

    /// RealHw's clock moves between any two reads.
    struct TickingNow {
        hw: FakeHw,
        calls: std::cell::Cell<u64>,
    }
    impl Hw for TickingNow {
        fn now(&self) -> Instant {
            let n = self.calls.get() + 1;
            self.calls.set(n);
            self.hw.now() + Duration::from_micros(n)
        }
        fn boottime(&self) -> Duration { self.hw.boottime() + Duration::from_micros(self.calls.get()) }
        fn sleep(&mut self, d: Duration) { self.hw.sleep(d) }
        fn take_lock(&mut self) -> io::Result<()> { self.hw.take_lock() }
        fn ping(&mut self) -> io::Result<()> { self.hw.ping() }
        fn set_min(&mut self) -> io::Result<()> { self.hw.set_min() }
        fn set_reduction(&mut self, d: f64) -> io::Result<f64> { self.hw.set_reduction(d) }
        fn set_reductions(&mut self, d: &[f64]) -> io::Result<Vec<f64>> { self.hw.set_reductions(d) }
        fn relinquish(&mut self) -> io::Result<()> { self.hw.relinquish() }
        fn sample_rate(&mut self) -> io::Result<i64> { self.hw.sample_rate() }
        fn open_sense(&mut self, r: u32) -> io::Result<()> { self.hw.open_sense(r) }
        fn close_sense(&mut self) { self.hw.close_sense() }
        fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> { self.hw.read_period(w) }
        fn queued(&mut self) -> io::Result<u64> { self.hw.queued() }
        fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> { self.hw.read_tail(w) }
        fn wait_rate_change(&mut self, d: Duration) -> bool { self.hw.wait_rate_change(d) }
        fn rate_events(&mut self) -> bool { self.hw.rate_events() }
        fn events_available(&self) -> bool { true }
    }

    /// Review r4 U6: with a clock that moves, the final state named a
    /// coverage 1 us short of its write and was never resumed.  One epoch
    /// now names both; then (U2) the run that resumes it consumes it.
    #[test]
    fn a_clean_stop_on_a_moving_clock_is_resumed_once() {
        let cfg = j700();
        let p = tmp_state().unwrap();
        let mut hw = TickingNow { hw: FakeHw::new(2, 4096), calls: std::cell::Cell::new(0) };
        hw.hw.playing = Box::new(|_| false);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p.clone())).unwrap();
        for _ in 0..20 {
            g.tick(&mut hw).unwrap();
        }
        g.stop(&mut hw).unwrap();
        while !g.released(&hw) {
            g.tick(&mut hw).unwrap();
        }
        g.final_save(&hw).unwrap();
        let saved = state::decode(&std::fs::read_to_string(&p).unwrap()).unwrap();
        assert!(saved.released && saved.covered == saved.boottime, "coverage lag {:?}", saved.boottime - saved.covered);
        let mut hw2 = TickingNow { hw: FakeHw::new(2, 4096), calls: std::cell::Cell::new(0) };
        hw2.hw.boot = hw.boottime() + secs(0.5);
        let g2 = Guard::start(&mut hw2, &cfg, 20.0, Some(p.clone())).unwrap();
        assert!(!g2.speakers[0].hard, "a clean stop was not resumed");
        drop(g2);
        let mut hw3 = FakeHw::new(2, 4096);
        hw3.boot = hw2.boottime() + secs(0.5);
        let g3 = Guard::start(&mut hw3, &cfg, 20.0, Some(p)).unwrap();
        assert!(g3.speakers[0].hard, "the clean stop was resumed twice");
    }

    // ---- the PR #8 review's V/I witnesses (P8-2, P8-3), as safe outcomes ----

    /// A card whose closing tail is scripted.
    struct TailHw {
        hw: FakeHw,
        tail: Option<Vec<i32>>,
    }
    impl Hw for TailHw {
        fn now(&self) -> Instant { self.hw.now() }
        fn boottime(&self) -> Duration { self.hw.boottime() }
        fn sleep(&mut self, d: Duration) { self.hw.sleep(d) }
        fn take_lock(&mut self) -> io::Result<()> { self.hw.take_lock() }
        fn ping(&mut self) -> io::Result<()> { self.hw.ping() }
        fn set_min(&mut self) -> io::Result<()> { self.hw.set_min() }
        fn set_reduction(&mut self, d: f64) -> io::Result<f64> { self.hw.set_reduction(d) }
        fn set_reductions(&mut self, d: &[f64]) -> io::Result<Vec<f64>> { self.hw.set_reductions(d) }
        fn relinquish(&mut self) -> io::Result<()> { self.hw.relinquish() }
        fn sample_rate(&mut self) -> io::Result<i64> { self.hw.sample_rate() }
        fn open_sense(&mut self, r: u32) -> io::Result<()> { self.hw.open_sense(r) }
        fn close_sense(&mut self) { self.hw.close_sense() }
        fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> { self.hw.read_period(w) }
        fn queued(&mut self) -> io::Result<u64> { Ok(0) }
        fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> {
            match self.tail.take() {
                Some(t) => {
                    w[..t.len()].copy_from_slice(&t);
                    Ok(t.len() / self.hw.channels)
                }
                None => Ok(0),
            }
        }
        fn wait_rate_change(&mut self, d: Duration) -> bool { self.hw.wait_rate_change(d) }
        fn rate_events(&mut self) -> bool { self.hw.rate_events() }
        fn events_available(&self) -> bool { true }
    }

    /// Review P8-2: a closing tail with negative V*I handed the card off,
    /// then the idle look in the same pass took it back with no good period
    /// since.  After bad sense data REARM_PERIODS valid periods come first,
    /// idle or not.
    #[test]
    fn bad_sense_data_at_close_is_not_followed_by_an_idle_rearm() {
        let c = j413();
        let mut hw = TailHw { hw: FakeHw::per_speaker(&c), tail: None };
        hw.hw.level = Box::new(|_| 0.001);
        let mut g = Guard::start(&mut hw, &c, 7.0, Some(vi_state(&c, 55.0, 52.0))).unwrap();
        for _ in 0..20 {
            g.tick(&mut hw).unwrap();
            if g.armed() {
                break;
            }
        }
        assert!(g.armed());
        let mut tail = vec![0; 48 * c.globals.channels];
        let s = &c.speakers[0];
        for f in tail.chunks_exact_mut(c.globals.channels) {
            f[s.vs_chan] = word(1.0, 14.0);
            f[s.is_chan.unwrap()] = word(-0.1, 3.75);
        }
        hw.tail = Some(tail);
        hw.hw.playing = Box::new(|_| false);
        hw.sleep(secs(0.001));
        let pings = hw.hw.pings.len();
        g.tick(&mut hw).unwrap();
        assert!(g.bad_data && g.recovering, "the bad tail was not seen");
        assert!(!g.armed() && hw.hw.pings.len() == pings, "rearmed in the pass that handed off for bad data");
        // idle afterwards: still no lock without good periods
        for _ in 0..40 {
            g.tick(&mut hw).unwrap();
        }
        assert!(!g.armed(), "an idle look took the lock back after bad data");
    }

    /// Review P8-3: a whole period, contiguous with the last, whose
    /// Duration rounding made it look partial, was integrated as its
    /// blocks' means (a +/- 25 W pair in one block cancelled): no latch at
    /// 149 C where the samples cross 150 C, 0 dB at 119 C where 2.1 dB is
    /// needed.  Every frame over new time is integrated sample by sample.
    #[test]
    fn a_whole_period_is_integrated_sample_by_sample_whatever_the_rounding() {
        let c = j413();
        let k = idx(&c, "Left Tweeter");
        let s = c.speakers[k].clone();
        let dt = 4096.0 / 48000.0;
        for tc in [149.0, 119.0] {
            let p = vi_state(&c, tc, 100.0);
            let saved = state::decode(&std::fs::read_to_string(&p).unwrap()).unwrap();
            let mut hw = FakeHw::per_speaker(&c);
            let mut g = Guard::start(&mut hw, &c, 40.0, Some(p)).unwrap();
            g.rate = 48000;
            for (i, f) in g.words.chunks_exact_mut(c.globals.channels).enumerate() {
                if i >= 3840 {
                    f[s.vs_chan] = word(7.0, 14.0);
                    f[s.is_chan.unwrap()] = word(if i < 3968 { 25.0 / 7.0 } else { -25.0 / 7.0 }, 3.75);
                }
            }
            g.check_sense(4096).unwrap();
            let mut reference = Engine::new(&c, false).unwrap();
            state::restore_engine(&mut reference, &saved, hw.boottime(), &c.globals);
            reference.check(&g.words, 48000.0).unwrap();
            reference.measured(&g.words, 4096, 4096, 4096, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
            let end = hw.now() + secs(dt);
            g.measure(end, 4096).unwrap();
            let (got, want) = (&asahi(&g)[k], &reference.asahi.as_ref().unwrap()[k]);
            assert_eq!(got.hard, want.hard, "{tc} C: the latch");
            assert!(got.coil_peak >= want.coil_peak - 1e-6, "{tc} C: peak {} vs samplewise {}", got.coil_peak, want.coil_peak);
            assert!(got.gain <= want.gain + 1e-9, "{tc} C: {} dB vs samplewise {} dB", -got.gain, -want.gain);
            // and against the review's own sample-by-sample integration
            if tc > 140.0 {
                assert!(got.hard, "149 C: the samples cross 150 C");
            } else {
                assert!(-got.gain >= 2.0, "119 C: {} dB where the samples need 2.07 dB", -got.gain);
            }
        }
    }

    /// The same where only the last 256 frames are new: they are integrated
    /// sample by sample from where the account stood (0.2's rest path held
    /// their blocks' means: no latch at 148 C where they cross 150 C).
    #[test]
    fn a_partly_accounted_period_integrates_its_new_frames_sample_by_sample() {
        let c = j413();
        let k = idx(&c, "Left Tweeter");
        let s = c.speakers[k].clone();
        let p = vi_state(&c, 148.0, 100.0);
        let saved = state::decode(&std::fs::read_to_string(&p).unwrap()).unwrap();
        let mut hw = FakeHw::per_speaker(&c);
        let mut g = Guard::start(&mut hw, &c, 40.0, Some(p)).unwrap();
        g.rate = 48000;
        for (i, f) in g.words.chunks_exact_mut(c.globals.channels).enumerate() {
            if i >= 3840 {
                f[s.vs_chan] = word(7.0, 14.0);
                f[s.is_chan.unwrap()] = word(if i < 3968 { 25.0 / 7.0 } else { -25.0 / 7.0 }, 3.75);
            }
        }
        g.check_sense(4096).unwrap();
        let t0 = hw.now();
        let end = t0 + Duration::from_nanos((256.0e9 / 48000.0) as u64 + 1);
        g.account_to = t0;
        let mut reference = Engine::new(&c, false).unwrap();
        state::restore_engine(&mut reference, &saved, hw.boottime(), &c.globals);
        let ch = c.globals.channels;
        let tail = &g.words[(4096 - 256) * ch..4096 * ch];
        reference.measured(tail, 256, 256, 256, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
        g.measure(end, 4096).unwrap();
        let (got, want) = (&asahi(&g)[k], &reference.asahi.as_ref().unwrap()[k]);
        assert!(want.hard, "the samples cross the limit");
        assert!(got.hard, "the new frames' crossing was not seen");
    }

    // ---- 0.1.10: the PR #7 fifth review's witnesses (r5), as safe outcomes ----

    /// A cold clean stop's state at the fake's start (boot time 1000 s).
    fn cold_file(cfg: &Config) -> PathBuf {
        let p = missing_state().unwrap();
        let sp: Vec<_> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 50.0)).collect();
        std::fs::write(&p, state::encode(secs(1000.0), secs(1000.0), &sp, false, true)).unwrap();
        p
    }

    /// Review r5 V3: the gap after a clean stop may have carried a
    /// full-scale signal through the card's safe level: its budgets count
    /// it at full scale before the gain (0.1.9 counted the 20 dB lower
    /// delivered power, 0 dB where 6.5 dB was needed).
    #[test]
    fn a_restart_gap_is_full_scale_before_the_gain() {
        let cfg = j700();
        let ta = cfg.globals.t_ambient;
        let cold: Vec<_> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), ta)).collect();
        let saved = state::decode(&state::encode(secs(1000.0), secs(1000.0), &cold, false, true)).unwrap();
        for elapsed in [1.0, 60.0, 3601.0] {
            let mut resumed = cold.clone();
            assert!(state::restore(&mut resumed, &saved, secs(1000.0 + elapsed), &cfg.globals));
            let mut reference = cold[0].clone();
            let full = reference.worst_case_power(0.0);
            let safe = reference.worst_case_power(cfg.globals.safe_reduction);
            reference.step_pre(safe, full, f64::min(elapsed, 60.0), ta);
            reference.govern(&cfg.globals);
            resumed[0].govern(&cfg.globals);
            for w in [1.0, 60.0] {
                assert!(resumed[0].mean_power(w) >= reference.mean_power(w) - 1e-6, "{elapsed} s gap: {w} s mean {} vs {} W", resumed[0].mean_power(w), reference.mean_power(w));
            }
            assert!(resumed[0].reduction_db >= reference.reduction_db - 1e-6, "{elapsed} s gap: {} vs {} dB", resumed[0].reduction_db, reference.reduction_db);
        }
        // and through a start: the first unlock after a minute's gap
        let p = cold_file(&cfg);
        let mut hw = FakeHw::new(2, 4096);
        hw.boot = secs(1060.0);
        hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p)).unwrap();
        for _ in 0..40 {
            if g.armed() {
                break;
            }
            g.tick(&mut hw).unwrap();
        }
        let mut want = cold[0].clone();
        want.step_pre(want.worst_case_power(20.0), want.worst_case_power(0.0), 60.0, ta);
        want.step(0.0, hw.t.as_secs_f64(), ta);
        want.govern(&cfg.globals);
        assert!(want.reduction_db > 7.0);
        assert!(!g.armed() || hw.reduction >= want.reduction_db - 1e-6, "unlocked at {} dB, the gap needs {} dB", hw.reduction, want.reduction_db);
    }

    /// Review r5 V5: one block can hold frames converted at different
    /// gains (a quiet half at 20 dB, a loud half at 0 dB, both full scale
    /// before the gain); 0.1.9 inverted the block's mean at one gain.
    #[test]
    fn a_block_of_mixed_gains_is_inverted_frame_by_frame() {
        let cfg = j700();
        let p = cold_file(&cfg);
        let mut hw = FakeHw::new(2, 4096);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p)).unwrap();
        let t0 = hw.now();
        g.rate = 48000;
        g.written = vec![0.0; g.speakers.len()];
        g.written_hi = vec![0.0; g.speakers.len()];
        g.gains = vec![Gain { at: t0 - secs(0.2), lo: vec![20.0; g.speakers.len()], hi: vec![20.0; g.speakers.len()], kernel: false, cands: Vec::new() }, Gain { at: t0, lo: vec![0.0; g.speakers.len()], hi: vec![0.0; g.speakers.len()], kernel: false, cands: Vec::new() }];
        // a lease held by hand: the card applies the controls
        g.locked_at = None;
        g.account_to = t0;
        g.verified_at = t0;
        let full = g.speakers[0].worst_case_power(0.0);
        let frames = 3840;
        let dt = frames as f64 / 48000.0;
        for s in &mut g.speakers {
            s.step_pre(2.65 / 100.0, 2.65, 1.0 - dt, 50.0);
        }
        let mut reference = g.speakers[0].clone();
        for f in 0..frames {
            let a = if f < 128 { 0.1 } else if f < 256 { 1.0 } else { 0.6 };
            let w = (a * 2147483647.0) as i32;
            g.words[f * 2] = w;
            g.words[f * 2 + 1] = w;
        }
        let (powers, _) = reference.block_powers(&g.words[..frames * 2], 2, 15);
        let mut pre = powers.clone();
        pre[0] = full;
        reference.step_period_pre(&powers, &pre, dt, 50.0);
        reference.govern(&cfg.globals);
        g.measure(t0 + secs(dt), frames).unwrap();
        assert!(g.speakers[0].mean_power(1.0) >= reference.mean_power(1.0) * (1.0 - 1e-6), "{} vs {} W", g.speakers[0].mean_power(1.0), reference.mean_power(1.0));
        assert!(g.speakers[0].reduction_db >= reference.reduction_db - 1e-6);
    }

    /// Review r5 V6: the card rounds a reduction up to its 0.5 dB steps and
    /// the frames are converted at that; 0.1.9 undid the request (5.5 dB
    /// applied where the input's budget needed 5.6, so 6 rounded).  The
    /// history holds what the card applies.
    #[test]
    fn the_history_holds_the_reduction_the_card_applies() {
        let cfg = j700();
        let p = cold_file(&cfg);
        let mut hw = FakeHw::new(2, 4096);
        hw.steps = true;
        hw.level = Box::new(|_| 0.9);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p)).unwrap();
        let required = 10.0 * (g.speakers[0].worst_case_power(0.0) * 0.81 / cfg.speakers[0].p_limit_1s).log10();
        let mut lowest = f64::INFINITY;
        while hw.t < secs(1.6) {
            g.tick(&mut hw).unwrap();
            if hw.t >= secs(1.2) && g.armed() {
                lowest = lowest.min(hw.reduction);
            }
        }
        assert!(lowest >= required, "applied {lowest} dB, the input's budget needs {required} dB");
        assert!(g.gains.iter().all(|e| e.lo.iter().all(|&lo| !lo.is_finite() || lo >= MUTE_DB || (lo * 2.0).fract() == 0.0)), "a requested value in the history: {:?}", g.gains);
    }

    /// The clock reads of the final save, with a pause between the boot
    /// time and the monotonic time.
    struct EpochGap {
        hw: FakeHw,
        extra: std::cell::Cell<Duration>,
        inject: std::cell::Cell<bool>,
        pending: std::cell::Cell<bool>,
        /// Every boot-time read is followed by the pause.
        always: bool,
    }
    impl Hw for EpochGap {
        fn now(&self) -> Instant {
            if self.pending.replace(false) {
                self.extra.set(self.extra.get() + secs(2.0));
            }
            self.hw.now() + self.extra.get()
        }
        fn boottime(&self) -> Duration {
            let b = self.hw.boottime() + self.extra.get();
            if self.inject.replace(self.always) {
                self.pending.set(true);
            }
            b
        }
        fn sleep(&mut self, d: Duration) { self.hw.sleep(d) }
        fn take_lock(&mut self) -> io::Result<()> { self.hw.take_lock() }
        fn ping(&mut self) -> io::Result<()> { self.hw.ping() }
        fn set_min(&mut self) -> io::Result<()> { self.hw.set_min() }
        fn set_reduction(&mut self, d: f64) -> io::Result<f64> { self.hw.set_reduction(d) }
        fn set_reductions(&mut self, d: &[f64]) -> io::Result<Vec<f64>> { self.hw.set_reductions(d) }
        fn relinquish(&mut self) -> io::Result<()> { self.hw.relinquish() }
        fn sample_rate(&mut self) -> io::Result<i64> { self.hw.sample_rate() }
        fn open_sense(&mut self, r: u32) -> io::Result<()> { self.hw.open_sense(r) }
        fn close_sense(&mut self) { self.hw.close_sense() }
        fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> { self.hw.read_period(w) }
        fn queued(&mut self) -> io::Result<u64> { self.hw.queued() }
        fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> { self.hw.read_tail(w) }
        fn wait_rate_change(&mut self, d: Duration) -> bool { self.hw.wait_rate_change(d) }
        fn rate_events(&mut self) -> bool { self.hw.rate_events() }
        fn events_available(&self) -> bool { true }
    }

    /// Review r5 V1: a two-second pause between the final save's clock
    /// reads left a model cooled to the later time under the earlier epoch,
    /// and the restore cooled the pause again (the latch cleared at 117.6 C
    /// where the true elapsed time left 121.5 C).  The reads are retried,
    /// whatever pause is left between them is charged once more at full
    /// scale and full gain, and the file names the later epoch: the
    /// restored model is never cooler than the speaker, pause or not.
    #[test]
    fn a_pause_inside_the_final_save_never_cools_the_restored_model() {
        let cfg = j700();
        for always in [false, true] {
            let p = cold_file(&cfg);
            let mut hw = EpochGap { hw: FakeHw::new(2, 4096), extra: Default::default(), inject: Default::default(), pending: Default::default(), always };
            hw.hw.playing = Box::new(|_| false);
            let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p.clone())).unwrap();
            for s in &mut g.speakers {
                s.resume(130.0, 110.0, 0.0, 0.0);
                s.hard = true;
                s.govern(&cfg.globals);
            }
            g.written = vec![20.0; g.speakers.len()];
            g.written_hi = vec![20.0; g.speakers.len()];
            g.gains = vec![Gain { at: hw.now() - secs(2.0), lo: vec![20.0; g.speakers.len()], hi: vec![20.0; g.speakers.len()], kernel: true, cands: Vec::new() }];
            g.verified_at = hw.now();
            g.account_to = hw.now();
            g.stop(&mut hw).unwrap();
            hw.sleep(DRAIN);
            let before = g.speakers[0].clone();
            let from = g.account_to;
            hw.inject.set(true);
            g.final_save(&hw).unwrap();
            hw.inject.set(false);
            hw.always = false;
            let saved = state::decode(&std::fs::read_to_string(p).unwrap()).unwrap();
            let mut resumed = g.speakers.clone();
            assert!(state::restore(&mut resumed, &saved, hw.boottime(), &cfg.globals));
            // the true elapsed time, at the most the card can have let through
            let mut truth = before.clone();
            truth.step_at(truth.worst_case_power(20.0), (hw.now() - from).as_secs_f64(), 50.0, 20.0);
            assert!(resumed[0].t_coil >= truth.t_coil - 1e-6, "pause at every read {always}: restored {} C, the true elapsed time {} C", resumed[0].t_coil, truth.t_coil);
            for _ in 0..400 {
                let p = truth.worst_case_power(cfg.globals.safe_reduction);
                truth.step_at(p, 0.025, 50.0, cfg.globals.safe_reduction);
                resumed[0].step_at(p, 0.025, 50.0, cfg.globals.safe_reduction);
                assert!(!(resumed[0].clear_hard_if_cool() && !truth.clear_hard_if_cool()), "the restored latch cleared before the true one could");
            }
        }
    }


    /// Review r5 V4: a lease that ran out while the daemon was away left
    /// the period after it inverted at the old gain, before the hand-off
    /// marked the card's safe level (4.0 W where 5.1 W came).  The safe
    /// level is a candidate as soon as the lease may have run out.
    #[test]
    fn frames_after_a_possible_lapse_count_at_the_safe_level() {
        let cfg = j700();
        let p = cold_file(&cfg);
        let mut hw = FakeHw::new(2, 4096);
        hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p)).unwrap();
        for _ in 0..20 {
            g.tick(&mut hw).unwrap();
        }
        assert!(g.armed());
        let mut want = g.speakers[0].clone();
        let from = g.account_to;
        hw.level = Box::new(|_| 1.0);
        hw.lossy = true;
        hw.sleep(secs(0.4));
        assert_eq!(hw.effective_reduction(), cfg.globals.safe_reduction);
        g.tick(&mut hw).unwrap();
        let dt = 4096.0 / 48000.0;
        let span = (g.account_to - from).as_secs_f64();
        want.step_at(want.worst_case_power(0.0), span - dt, 50.0, 0.0);
        want.step_at(want.worst_case_power(20.0), dt, 50.0, 20.0);
        want.govern(&cfg.globals);
        assert!(g.speakers[0].mean_power(1.0) >= want.mean_power(1.0) - 1e-3, "{} vs {} W", g.speakers[0].mean_power(1.0), want.mean_power(1.0));
        assert!(g.speakers[0].reduction_db >= want.reduction_db - 1e-3);
    }

    /// Another owner runs between the start and the lock: it plays, writes
    /// its status (not a clean stop) and dies.
    struct OwnershipRace {
        hw: FakeHw,
        p: PathBuf,
        intervening: Vec<SpeakerState>,
    }
    impl Hw for OwnershipRace {
        fn now(&self) -> Instant { self.hw.now() }
        fn boottime(&self) -> Duration { self.hw.boottime() }
        fn sleep(&mut self, d: Duration) { self.hw.sleep(d) }
        fn take_lock(&mut self) -> io::Result<()> {
            self.hw.sleep(secs(0.12));
            let boot = self.hw.boottime();
            std::fs::write(&self.p, state::encode(boot, boot, &self.intervening, true, false))?;
            self.hw.take_lock()
        }
        fn ping(&mut self) -> io::Result<()> { self.hw.ping() }
        fn set_min(&mut self) -> io::Result<()> { self.hw.set_min() }
        fn set_reduction(&mut self, d: f64) -> io::Result<f64> { self.hw.set_reduction(d) }
        fn set_reductions(&mut self, d: &[f64]) -> io::Result<Vec<f64>> { self.hw.set_reductions(d) }
        fn relinquish(&mut self) -> io::Result<()> { self.hw.relinquish() }
        fn sample_rate(&mut self) -> io::Result<i64> { self.hw.sample_rate() }
        fn open_sense(&mut self, r: u32) -> io::Result<()> { self.hw.open_sense(r) }
        fn close_sense(&mut self) { self.hw.close_sense() }
        fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> { self.hw.read_period(w) }
        fn queued(&mut self) -> io::Result<u64> { self.hw.queued() }
        fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> { self.hw.read_tail(w) }
        fn wait_rate_change(&mut self, d: Duration) -> bool { self.hw.wait_rate_change(d) }
        fn rate_events(&mut self) -> bool { self.hw.rate_events() }
        fn events_available(&self) -> bool { true }
    }

    /// Review r5 V2: the state was read before the lock was taken, so a
    /// clean stop another owner had since used and replaced was resumed
    /// (0 dB at 100 C where that owner's run needed 10.3 dB).  The state
    /// is read under the lock.
    #[test]
    fn the_state_is_read_under_the_cards_lock() {
        let cfg = j700();
        let p = missing_state().unwrap();
        let warm: Vec<_> = cfg.speakers.iter().map(|s| { let mut x = SpeakerState::new(s.clone(), 50.0); x.resume(100.0, 100.0, 0.0, 0.0); x }).collect();
        std::fs::write(&p, state::encode(secs(1000.0), secs(1000.0), &warm, false, true)).unwrap();
        let mut intervening = warm.clone();
        for s in &mut intervening {
            s.step_at(s.worst_case_power(0.0), 0.12, 50.0, 0.0);
            s.govern(&cfg.globals);
        }
        let mut hw = OwnershipRace { hw: FakeHw::new(2, 4096), p: p.clone(), intervening };
        hw.hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p)).unwrap();
        assert!(g.speakers.iter().all(|s| s.hard), "resumed a state read before the lock");
        let begin = hw.hw.t;
        while !g.armed() && hw.hw.t < secs(60.0) {
            g.tick(&mut hw).unwrap();
        }
        let mut reference = hw.intervening[0].clone();
        reference.step(0.0, (hw.hw.t - begin).as_secs_f64(), 50.0);
        reference.govern(&cfg.globals);
        assert!(hw.hw.reduction >= reference.reduction_db - 1e-6, "unlocked at {} dB, the other owner's run needs {} dB", hw.hw.reduction, reference.reduction_db);
    }

    #[test]
    fn a_start_during_playback_costs_little() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.3);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        run_until(&mut g, &mut hw, secs(10.0)).unwrap();
        assert!(hw.writes.iter().all(|&(t, db)| db <= 1.0 && (t < secs(2.5) || db == 0.0)), "{:?}", hw.writes.iter().filter(|w| w.1 != 0.0).collect::<Vec<_>>());
        assert!(hw.lapses.is_empty());
    }

    // ---- review r2 of speakerguardd#8 ----

    /// Seeded sense noise for the V/I witnesses: `n` LSB either way on every
    /// S16 word, on top of a per-channel offset of up to `dc` LSB, as the
    /// M1 Pro's sense lines carry it.
    pub fn sense_noise(channels: usize, n: i32, dc: i32, seed: u64) -> impl FnMut(&mut [i32]) {
        let mut x = seed | 1;
        let mut next = move || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x
        };
        let offsets: Vec<i32> = (0..channels).map(|_| if dc > 0 { (next() % (2 * dc as u64 + 1)) as i32 - dc } else { 0 }).collect();
        move |w: &mut [i32]| {
            for (i, v) in w.iter_mut().enumerate() {
                let r = if n > 0 { (next() % (2 * n as u64 + 1)) as i32 - n } else { 0 };
                *v = (r + offsets[i % channels]).clamp(-32768, 32767) << 16;
            }
        }
    }

    /// Review X1: after a hand-off the controls keep the governed value,
    /// but the card holds its lock and so its safe level.  Once what was
    /// converted before the hand-off has drained, uncovered time is charged
    /// at each control's value or the safe level, whichever is more, until
    /// the next renewal.  Before the drain it is charged at the controls'.
    #[test]
    fn vi_a_drained_hand_off_charges_at_the_safe_level() {
        let cfg = j413();
        let safe = cfg.globals.safe_reduction;
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.02);
        let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 60.0, 55.0))).unwrap();
        run_until(&mut g, &mut hw, secs(2.0)).unwrap();
        assert!(g.armed() && hw.per.iter().all(|&p| p == 0.0), "{:?}", hw.per);
        g.disarm(&mut hw, "a test hand-off").unwrap();
        let handed = hw.t;
        assert!(g.written.iter().all(|&w| w == 0.0), "the controls keep the governed value");
        assert!(g.least_since(g.verified_at).iter().all(|&r| r == 0.0), "charged at the safe level before the drain");
        // the card stays with the kernel (the ramp is not the point here)
        g.max_reduction = 0.1;
        while hw.t < handed + DRAIN + secs(0.3) {
            g.tick(&mut hw).unwrap();
            assert!(!g.armed());
        }
        assert!(g.verified_at >= g.locked_at.unwrap() + DRAIN, "no verified boundary after the drain");
        assert_eq!(g.least_since(g.verified_at), vec![safe; cfg.speakers.len()]);
        // a charge heats every model at the safe level's worst case
        let ta = cfg.globals.t_ambient;
        let before: Vec<_> = asahi(&g).iter().map(|a| a.th.clone()).collect();
        let to = g.account_to + secs(0.2);
        g.charge(to).unwrap();
        for (k, (a, b)) in asahi(&g).iter().zip(before).enumerate() {
            let mut want = b.clone();
            want.hold(vi_worst(&cfg, k, safe), 0.2, ta);
            assert!((a.th.t_coil - want.t_coil).abs() < 1e-9, "{}: charged to {} C, the safe level gives {} C", cfg.speakers[k].name, a.th.t_coil, want.t_coil);
        }
        // a renewal ends it
        g.max_reduction = cfg.globals.max_reduction;
        while !g.armed() && hw.t < secs(30.0) {
            g.tick(&mut hw).unwrap();
        }
        assert!(g.armed() && g.locked_at.is_none());
        assert!(g.least_since(g.verified_at).iter().all(|&r| r < safe));
    }

    /// Review X1: with the governed 0 dB still on the controls, a sense
    /// outage latched the asahi tweeter, and the stream then played on at
    /// the safe level.  Each period's sliver of uncovered time was charged
    /// at 0 dB, which put the model back over the limit, and the next
    /// negative V*I sample held it there: with +-1 LSB of noise the latch
    /// never cleared and the lock was never taken back.  Now the latch
    /// clears once the model is under the working limit, and the lock
    /// comes back, whatever the noise.
    #[test]
    fn vi_a_latch_after_a_hand_off_clears_through_sense_noise() {
        let cfg = j413();
        let k = idx(&cfg, "Left Tweeter");
        let ceiling = cfg.speakers[k].t_limit - cfg.speakers[k].t_headroom;
        for (n, dc) in [(0, 0), (1, 0), (3, 2), (9, 10)] {
            let mut hw = FakeHw::per_speaker(&cfg);
            hw.level = Box::new(|_| 0.05);
            let noisy = std::rc::Rc::new(std::cell::Cell::new(false));
            let on = noisy.clone();
            let mut noise = sense_noise(cfg.globals.channels, n, dc, 0x9e37_79b9 + n as u64);
            hw.tweak = Some(Box::new(move |_, w| {
                if on.get() {
                    noise(w);
                }
            }));
            let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(vi_state(&cfg, 110.0, 100.0))).unwrap();
            while !g.armed() && hw.t < secs(5.0) {
                g.tick(&mut hw).unwrap();
            }
            assert!(g.armed() && hw.per.iter().all(|&p| p == 0.0), "not governing at 0 dB: {:?}", hw.per);
            hw.sensing = Box::new(|_| false);
            while !asahi(&g)[k].hard && hw.t < secs(30.0) {
                g.tick(&mut hw).unwrap();
            }
            assert!(asahi(&g)[k].hard && asahi(&g)[k].th.t_coil >= 150.0, "the outage did not latch the tweeter");
            let latched = hw.t;
            hw.sensing = Box::new(|_| true);
            noisy.set(true);
            let (mut cleared, mut rearmed) = (None, None);
            while hw.t < latched + secs(60.0) && rearmed.is_none() {
                g.tick(&mut hw).unwrap();
                let a = &asahi(&g)[k];
                if cleared.is_none() && !g.engine.as_ref().unwrap().hard(Governor::Asahi) {
                    cleared = Some(hw.t - latched);
                    let hottest = a.th.t_coil.max(a.th.t_magnet).max(a.coil_peak).max(a.magnet_peak);
                    assert!(hottest < ceiling, "+-{n} LSB: cleared at {hottest} C");
                }
                if g.armed() {
                    rearmed = Some(hw.t - latched);
                }
            }
            let cleared = cleared.unwrap_or_else(|| panic!("+-{n} LSB: the latch never cleared (tweeter {:.3} C)", asahi(&g)[k].th.t_coil));
            let rearmed = rearmed.unwrap_or_else(|| panic!("+-{n} LSB: the lock was never taken back"));
            assert!(cleared < secs(15.0) && rearmed < secs(30.0), "+-{n} LSB: cleared after {cleared:?}, taken back after {rearmed:?}");
        }
    }

    /// Review X3: with nothing known (no state, as after every boot) the
    /// V/I machines sat at the card's safe level for 163 s (J413) and
    /// 127 s (J316) while the drain's magnet cooled.  The latched start now
    /// holds the drain from coil and magnet at the limit
    /// (engine::a_latched_start_bounds_every_past_it_stands_for), and the
    /// lock comes back once the ramp allows, sooner.
    #[test]
    fn vi_a_start_with_nothing_known_takes_the_lock_back_sooner() {
        let mut j316 = Config::parse(include_str!("../conf/apple/j316.conf")).unwrap();
        j316.allow_unqualified = true;
        for (cfg, within) in [(j413(), 70.0), (j316, 95.0)] {
            let mut hw = FakeHw::per_speaker(&cfg);
            hw.level = Box::new(|_| 0.0);
            let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, missing_state()).unwrap();
            assert!(asahi(&g).iter().all(|a| a.hard && a.th.t_coil >= 150.0));
            while !g.armed() && hw.t < secs(300.0) {
                g.tick(&mut hw).unwrap();
            }
            assert!(g.armed(), "{}: never taken back", cfg.speakers[0].name);
            eprintln!("X3 {} speakers: the lock taken back after {:.1} s", cfg.speakers.len(), hw.t.as_secs_f64());
            assert!(hw.t < secs(within), "{}: taken back after {:?}", cfg.speakers[0].name, hw.t);
            let demand = g.engine.as_ref().unwrap().reductions(Governor::Asahi).into_iter().fold(0.0, f64::max);
            assert!(demand < cfg.globals.max_reduction - REARM_MARGIN_DB);
        }
    }
}
