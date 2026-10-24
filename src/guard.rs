// SPDX-License-Identifier: MIT
//! The protection loop, behind a small hardware interface so that its
//! timing rules can be exercised without a sound card: the card's volume
//! lock, its speaker sample-rate control and the sense capture.

use std::error::Error;
use std::io;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use crate::config::{Config, Globals, Governor};
use crate::engine::{self, Engine, Start};
use crate::model::{self, SpeakerState};
use crate::state;
use crate::{debug, info, warn};

/// The kernel locks the volume again 250 ms after the last write; write well
/// inside that while the speakers play.
pub const PING_INTERVAL: Duration = Duration::from_millis(100);
/// Cadence of the per-speaker debug line.
const LOG_INTERVAL: Duration = Duration::from_secs(1);
/// Consecutive control failures tolerated before the daemon gives up.  While it
/// stays alive the volume lock stays held; exiting hands the card to its own
/// safe level, which the listener hears as a one-second dip.
const MAX_CONTROL_FAILURES: u32 = 50;
/// Sense data missing for this long means the stream is gone, not late.
pub const STALL_TIMEOUT: Duration = Duration::from_millis(500);

/// What the loop needs from the machine.
pub trait Hw {
    fn now(&self) -> Instant;
    /// Time since boot, suspend included (CLOCK_BOOTTIME): the model state
    /// saved by an earlier run is cooled by the time between the runs.
    fn boottime(&self) -> Duration;
    fn sleep(&mut self, d: Duration);
    /// Lock every volume control and the unlock control, then ping once.
    fn take_lock(&mut self) -> io::Result<()>;
    /// Renew the card's volume lock ("Speaker Volume Unlock").
    fn ping(&mut self) -> io::Result<()>;
    /// Write the gain reduction (dB below the control's maximum).
    fn set_reduction(&mut self, db: f64) -> io::Result<()>;
    /// Per-speaker reductions, for cards with a volume control per speaker;
    /// a card with one shared control gets the largest.
    fn set_reductions(&mut self, db: &[f64]) -> io::Result<()> {
        self.set_reduction(db.iter().cloned().fold(0.0, f64::max))
    }
    /// The speakers' sample rate, 0 while they are closed.
    fn sample_rate(&mut self) -> io::Result<i64>;
    fn open_sense(&mut self, rate: u32) -> io::Result<()>;
    fn close_sense(&mut self);
    /// One sense period: `Some(true)` complete, `Some(false)` short or
    /// overrun, `None` nothing within one poll (at most 50 ms).
    fn read_period(&mut self, words: &mut [i32]) -> io::Result<Option<bool>>;
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

/// How late a sense period may be before the time it should have covered is
/// charged to the model at the worst-case power.
pub const BLIND_SLACK: Duration = PING_INTERVAL;
/// Without complete sense data for this long (after the period that should
/// have arrived) the daemon stops renewing the card's volume lock and the
/// kernel drops the speakers to its safe level within 250 ms.
pub const BLIND_LIMIT: Duration = STALL_TIMEOUT;
/// The kernel's lock timeout (macaudio SPEAKER_LOCK_TIMEOUT).
const KERNEL_LOCK_TIMEOUT: Duration = Duration::from_millis(250);
/// Consecutive complete periods needed before the lock is taken back.
pub const REARM_PERIODS: u32 = 2;
/// Hysteresis between handing the card to the kernel at --max-reduction and
/// taking it back.
pub const REARM_MARGIN_DB: f64 = 3.0;

pub struct Guard {
    globals: Globals,
    pub speakers: Vec<SpeakerState>,
    max_reduction: f64,
    /// True while the daemon renews the card's volume lock.  False hands the
    /// speakers to the kernel's safe level; the daemon keeps modelling and
    /// takes the lock back when it can see and the speaker allows.
    armed: bool,
    /// The open sense capture's stall clock; it lives and dies with the
    /// capture: one left over from an earlier stream would call the next
    /// stream stalled at its first empty wait, before its first period.
    stall: Option<StallClock>,
    /// Seconds per sense period while the speakers play, None while closed.
    dt: Option<f64>,
    /// While playing: when the last complete sense period arrived (the
    /// stream's start until the first one does).
    last_valid: Option<Instant>,
    /// While playing: the model has accounted for the time up to here,
    /// with measured or worst-case power.
    charged_to: Option<Instant>,
    /// Complete periods in a row.
    fresh: u32,
    words: Vec<i32>,
    last_ping: Instant,
    last_log: Instant,
    /// The reduction the card last accepted (dB).
    written: f64,
    applied: f64,
    control_failures: u32,
    state_path: Option<PathBuf>,
    save_failed: bool,
    /// The governors besides legacy (always running), and which one writes.
    pub engine: Engine,
    pub governing: Governor,
    /// Per-speaker reductions the card last accepted (dB).
    written_per: Vec<f64>,
    shadow_path: Option<PathBuf>,
    shadow_failed: bool,
    /// Raw sense periods kept for --replay.
    pub record: Option<crate::record::Recorder>,
}

/// The shadow log is rotated (to `<file>.1`) past this size.
const SHADOW_MAX_BYTES: u64 = 16 << 20;

impl Guard {
    /// Restore the model from the last run (if any), take the card's volume
    /// lock and start at the volume the model allows.
    pub fn start<H: Hw>(hw: &mut H, cfg: &Config, max_reduction: f64, state_path: Option<PathBuf>) -> Result<Guard, Box<dyn Error>> {
        if !max_reduction.is_finite() || max_reduction <= 0.0 {
            return Err("maximum reduction must be finite and positive".into());
        }
        let globals = cfg.globals.clone();
        let mut speakers: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), globals.t_ambient)).collect();
        if let Some(path) = &state_path {
            match std::fs::read_to_string(path) {
                Ok(text) => match state::decode(&text) {
                    Some(saved) => {
                        let elapsed = hw.boottime().saturating_sub(saved.boottime);
                        state::restore(&mut speakers, &saved, elapsed, &globals);
                        info!("model restored from {} ({:.1} s old)", path.display(), elapsed.as_secs_f64());
                    }
                    None => {
                        warn!("{}: unreadable model state; assuming hot speakers", path.display());
                        for s in speakers.iter_mut() {
                            s.assume_hot(globals.t_ambient);
                        }
                    }
                },
                Err(e) if e.kind() == io::ErrorKind::NotFound => {}
                Err(e) => {
                    warn!("{}: {e}; assuming hot speakers", path.display());
                    for s in speakers.iter_mut() {
                        s.assume_hot(globals.t_ambient);
                    }
                }
            }
        }
        for s in speakers.iter_mut() {
            s.govern(&globals);
            if !s.finite() {
                return Err("nonfinite speaker model state".into());
            }
        }
        let engine = Self::start_engine(hw, cfg, state_path.as_ref());
        if !engine.finite() {
            return Err("nonfinite speaker model state".into());
        }
        let shadow_path = if globals.shadow { state_path.as_ref().map(|p| p.with_file_name("shadow.jsonl")) } else { None };
        let n = cfg.speakers.len();
        let now = hw.now();
        let mut g = Guard {
            words: vec![0i32; globals.period * globals.channels],
            globals,
            speakers,
            max_reduction,
            armed: false,
            stall: None,
            dt: None,
            last_valid: None,
            charged_to: None,
            fresh: 0,
            last_ping: now,
            last_log: now,
            written: f64::INFINITY,
            applied: 0.0,
            control_failures: 0,
            state_path,
            save_failed: false,
            engine,
            governing: cfg.globals.governor,
            written_per: vec![f64::INFINITY; n],
            shadow_path,
            shadow_failed: false,
            record: None,
        };
        info!("governor {}{}", g.governing.name(), if cfg.globals.shadow { ", the others in shadow" } else { "" });
        hw.take_lock()?;
        g.last_ping = hw.now();
        let (reduction, per) = g.governing_reduction();
        let fault = g.fault();
        if reduction < max_reduction && fault.is_none() {
            g.write(hw, reduction, &per)?;
            g.applied = reduction;
            g.armed = true;
        } else {
            warn!("{}; leaving the speakers at the card's safe level", fault.unwrap_or_else(|| format!("the model asks for {reduction:.1} dB")));
        }
        Ok(g)
    }

    /// The engine's models: from the saved engine state, cooled by the time
    /// since; cold when there is no state at all (first start since boot);
    /// hot when the state is unreadable or only the legacy part exists.
    fn start_engine<H: Hw>(hw: &H, cfg: &Config, state_path: Option<&PathBuf>) -> Engine {
        let Some(path) = state_path else { return Engine::new(cfg, Start::Cold) };
        let epath = path.with_extension("engine");
        match std::fs::read_to_string(&epath) {
            Ok(text) => match engine::decode(&text) {
                Some(saved) => {
                    let mut e = Engine::new(cfg, Start::Hot);
                    e.restore(&saved);
                    let elapsed = hw.boottime().saturating_sub(saved.boottime).as_secs_f64();
                    let mut left = elapsed.min(3600.0);
                    while left > 0.0 {
                        let dt = if elapsed - left < 60.0 { 0.1f64.min(left) } else { 1f64.min(left) };
                        e.cool(dt);
                        left -= dt;
                    }
                    if elapsed > 3600.0 {
                        e.cool(elapsed - 3600.0);
                    }
                    e
                }
                None => {
                    warn!("{}: unreadable engine state; assuming hot speakers", epath.display());
                    Engine::new(cfg, Start::Hot)
                }
            },
            Err(e) if e.kind() == io::ErrorKind::NotFound && !path.exists() => Engine::new(cfg, Start::Cold),
            Err(_) => Engine::new(cfg, Start::Hot),
        }
    }

    /// The governing reduction: card-wide (dB) and per speaker.
    fn governing_reduction(&self) -> (f64, Vec<f64>) {
        match self.governing {
            Governor::Legacy => {
                let r = model::reduction_db(&self.speakers);
                (r, vec![r; self.speakers.len()])
            }
            gov => {
                let per = self.engine.reductions(gov);
                (per.iter().cloned().fold(0.0, f64::max), per)
            }
        }
    }

    /// A governing model past its hard limit.
    fn fault(&self) -> Option<String> {
        self.engine.hard_fault(self.governing)
    }

    fn write<H: Hw>(&mut self, hw: &mut H, reduction: f64, per: &[f64]) -> io::Result<()> {
        if self.governing == Governor::Legacy {
            hw.set_reduction(reduction)?;
        } else {
            hw.set_reductions(per)?;
        }
        self.written = reduction;
        self.written_per = per.to_vec();
        Ok(())
    }

    /// What the card applied to each speaker since `from` (dB).
    fn card_reductions(&self, from: Instant) -> Vec<f64> {
        let lapsed = !(self.armed || from < self.last_ping + KERNEL_LOCK_TIMEOUT);
        self.written_per
            .iter()
            .map(|&w| {
                let w = if w.is_finite() { w } else { 0.0 };
                if lapsed { w.max(self.globals.safe_reduction) } else { w }
            })
            .collect()
    }

    #[cfg(test)]
    pub fn armed(&self) -> bool {
        self.armed
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
        Ok(())
    }

    /// The reduction the card applies over time starting at `from`: ours
    /// while the lock holds, at least the safe level once it has lapsed.
    fn card_reduction(&self, from: Instant) -> f64 {
        let written = if self.written.is_finite() { self.written } else { 0.0 };
        if self.armed || from < self.last_ping + KERNEL_LOCK_TIMEOUT {
            written
        } else {
            written.max(self.globals.safe_reduction)
        }
    }

    fn check_model(&self) -> Result<(), Box<dyn Error>> {
        if self.speakers.iter().all(|s| s.finite()) && self.engine.finite() {
            Ok(())
        } else {
            Err("nonfinite speaker model state".into())
        }
    }

    /// One pass of the loop: idle cooling, or one sense period.
    pub fn tick<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        let globals = self.globals.clone();
        // playback state: the card reports the speakers' sample rate, 0 when closed
        let rate = match hw.sample_rate() {
            Ok(v) => {
                self.control_failures = 0;
                u32::try_from(v).map_err(|_| "invalid negative or excessive speaker sample rate")?
            }
            Err(e) if transient(&e) && self.control_failures < MAX_CONTROL_FAILURES => {
                // Playback state unknown: a running stream's time stays
                // unmeasured and is charged as such.
                self.control_failures += 1;
                warn!("sample-rate control: {e}; retrying ({})", self.control_failures);
                self.maintain(hw)?;
                hw.sleep(PING_INTERVAL);
                return Ok(());
            }
            Err(e) => return Err(format!("sample-rate control: {e}").into()),
        };
        if rate == 0 {
            self.dt = None;
            self.last_valid = None;
            self.charged_to = None;
            self.fresh = 0;
            if self.stall.take().is_some() {
                hw.close_sense();
                info!("speakers closed");
            }
            // cool down with no power
            for s in self.speakers.iter_mut() {
                s.step(0.0, PING_INTERVAL.as_secs_f64(), globals.t_ambient);
                s.govern(&globals);
            }
            self.engine.idle(PING_INTERVAL.as_secs_f64());
            self.check_model()?;
            self.maintain(hw)?;
            hw.sleep(PING_INTERVAL);
            return Ok(());
        }
        let dt = globals.period as f64 / rate as f64;
        if !dt.is_finite() || dt <= 0.0 {
            return Err("invalid sense period".into());
        }
        self.dt = Some(dt);
        if self.last_valid.is_none() {
            let now = hw.now();
            self.last_valid = Some(now);
            self.charged_to = Some(now);
        }
        if self.stall.is_none() {
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
                self.stall.as_mut().unwrap().data();
                let features = self.engine.extractor.extract(&self.engine.cfg, &self.words, rate as f64);
                if let Some((name, p)) = self.engine.cfg.speakers.iter().zip(&features).find_map(|(s, f)| f.vi.filter(|&p| !(p >= -0.01)).map(|p| (s.name.clone(), p))) {
                    // speakersafetyd's sanity check: negative real power is
                    // bad sense data, not a measurement
                    warn!("{name}: negative or invalid sensed power ({p:.3} W); period not counted");
                    self.fresh = 0;
                    return self.maintain(hw);
                }
                // what the card applied while this period played: card-wide
                // (legacy) and per speaker (the engine)
                let since = self.last_valid.unwrap_or_else(|| hw.now());
                let applied_card = self.card_reduction(since);
                let applied = self.card_reductions(since);
                for s in self.speakers.iter_mut() {
                    let v = s.v_rms_of(&self.words, globals.channels);
                    s.v_rms = v;
                    let power = v * v / s.spec.z_nominal;
                    if !v.is_finite() || !power.is_finite() || power < 0.0 {
                        return Err("invalid speaker sense voltage or power".into());
                    }
                    s.step_at(power, dt, globals.t_ambient, applied_card);
                    s.govern(&globals);
                }
                self.engine.period(&features, &applied, dt, model::reduction_db(&self.speakers));
                if let Some(rec) = self.record.as_mut() {
                    if let Err(e) = rec.write(rate, hw.boottime(), &applied, &self.words) {
                        warn!("recording: {e}; stopped");
                        self.record = None;
                    } else if rec.full() {
                        info!("recording reached its size limit; stopped");
                        self.record = None;
                    }
                }
                self.check_model()?;
                // only a complete period whose model state is valid renews
                let now = hw.now();
                self.last_valid = Some(now);
                self.charged_to = Some(self.charged_to.map_or(now, |c| c.max(now)));
                self.fresh = self.fresh.saturating_add(1);
            }
            Ok(Some(false)) => {
                warn!("incomplete sense period or overrun, step skipped");
                self.fresh = 0;
            }
            Ok(None) => {
                // No sense data within one poll; only a stream silent for a
                // whole STALL_TIMEOUT is dropped and reopened.
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

    /// Charge unmeasured playback time, decide whether the daemon holds the
    /// card, and if it does write the model's reduction and renew the lock.
    fn maintain<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        let now = hw.now();
        let mut blind = Duration::ZERO;
        if let (Some(dt), Some(last_valid)) = (self.dt, self.last_valid) {
            let expected = last_valid + Duration::from_secs_f64(dt);
            blind = now.saturating_duration_since(expected);
            // Time nobody measured, beyond the lateness tolerated, is charged
            // at the most power the card's current gain lets through.
            let from = (expected + BLIND_SLACK).max(self.charged_to.unwrap_or(last_valid));
            if now > from {
                let reduction = self.card_reduction(from);
                let globals = &self.globals;
                for s in self.speakers.iter_mut() {
                    let p = s.worst_case_power(reduction);
                    s.step_at(p, (now - from).as_secs_f64(), globals.t_ambient, reduction);
                    s.govern(globals);
                }
                self.engine.blind((now - from).as_secs_f64());
                self.check_model()?;
                self.charged_to = Some(now);
            }
        }
        let (reduction, per) = self.governing_reduction();
        if !reduction.is_finite() {
            return Err("nonfinite speaker reduction".into());
        }
        let fault = self.fault();
        if self.armed {
            let why = if blind > BLIND_LIMIT {
                Some(format!("no complete sense data for {} ms", blind.as_millis()))
            } else if reduction >= self.max_reduction {
                Some(format!("the model asks for {reduction:.1} dB"))
            } else {
                fault.clone()
            };
            if let Some(why) = why {
                warn!("{why}; leaving the speakers at the card's safe level");
                self.armed = false;
            }
        } else {
            let seeing = self.dt.is_none() || (self.fresh >= REARM_PERIODS && blind <= BLIND_SLACK);
            if seeing && reduction < self.max_reduction - REARM_MARGIN_DB && fault.is_none() {
                self.ping_lock(hw)?;
                info!("taking the volume lock back at {reduction:.1} dB");
                self.armed = true;
                self.applied = reduction;
            }
        }
        if self.armed {
            if (reduction - self.applied).abs() >= 0.5 {
                info!("gain reduction {reduction:.1} dB");
                self.applied = reduction;
            }
            match self.write(hw, reduction, &per) {
                Ok(()) => {
                    self.control_failures = 0;
                }
                Err(e) if transient(&e) && self.control_failures < MAX_CONTROL_FAILURES => {
                    self.control_failures += 1;
                    warn!("volume control: {e}; retrying ({})", self.control_failures);
                }
                Err(e) => return Err(format!("volume control: {e}").into()),
            }
            if hw.now().duration_since(self.last_ping) >= PING_INTERVAL {
                self.ping_lock(hw)?;
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
            self.save(hw);
            self.shadow(hw);
            self.last_log = hw.now();
        }
        Ok(())
    }

    /// Append one line to the shadow log (best effort; rotated by size).
    fn shadow<H: Hw>(&mut self, hw: &H) {
        let Some(path) = self.shadow_path.clone() else { return };
        let legacy: Vec<(f64, f64)> = self.speakers.iter().map(|s| (s.t_coil, s.reduction_db)).collect();
        let applied = self.card_reductions(hw.now());
        let line = self.engine.shadow_line(hw.boottime().as_secs_f64(), self.governing, self.armed, &applied, &legacy, model::reduction_db(&self.speakers));
        if std::fs::metadata(&path).map_or(false, |m| m.len() > SHADOW_MAX_BYTES) {
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

    /// Write the model state for the next run (atomically; best effort).
    pub fn save<H: Hw>(&mut self, hw: &H) {
        let Some(path) = &self.state_path else { return };
        let text = state::encode(hw.boottime(), &self.speakers, self.armed);
        let tmp = path.with_extension("tmp");
        let etext = self.engine.encode(hw.boottime());
        let etmp = path.with_extension("engine.tmp");
        let epath = path.with_extension("engine");
        let result = std::fs::write(&tmp, text)
            .and_then(|()| std::fs::rename(&tmp, path))
            .and_then(|()| std::fs::write(&etmp, etext))
            .and_then(|()| std::fs::rename(&etmp, &epath));
        match result {
            Ok(()) => self.save_failed = false,
            Err(e) if !self.save_failed => {
                warn!("{}: {e}; the model state is not saved", path.display());
                self.save_failed = true;
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
            }
        }

        /// Is the card's volume lock lifted at time t?
        pub fn unlocked_at(&self, t: Duration) -> bool {
            self.last_ok_ping.is_some_and(|p| t <= p + LOCK_TIMEOUT) && !self.notice_pending
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

        fn lapse_check(&mut self) {
            if let Some(p) = self.last_ok_ping {
                if self.t > p + LOCK_TIMEOUT && !self.notice_pending {
                    self.lapses.push(p + LOCK_TIMEOUT);
                    self.notice_pending = true;
                }
            }
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
            self.ping()
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
            self.pings.push(self.t);
            Ok(())
        }
        fn set_reduction(&mut self, db: f64) -> io::Result<()> {
            self.lapse_check();
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
            Ok(if (self.playing)(self.t) { 48000 } else { 0 })
        }
        fn open_sense(&mut self, _rate: u32) -> io::Result<()> {
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
            self.t = self.t.max(self.next_period);
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 6.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
            let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
            let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
    /// run it starts at that run's reduction, not at full volume.
    #[test]
    fn a_restart_starts_from_the_saved_model() {
        let cfg = j700();
        let dir = std::env::temp_dir().join(format!("speakerguardd-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("state");
        let _ = std::fs::remove_file(&path);
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.6); // 4.6 W: budget and coil both govern
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path.clone())).unwrap();
        run_until(&mut g, &mut hw, secs(20.0)).unwrap();
        let before = model::reduction_db(&g.speakers);
        let coil = g.speakers[0].t_coil;
        assert!(before > 3.0, "{before}");
        g.save(&hw);
        drop(g);
        // the next run, 0.5 s later (systemd's restart delay)
        let mut hw2 = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw2.boot = hw.boot + hw.t + Duration::from_millis(500);
        let g2 = Guard::start(&mut hw2, &cfg, 20.0, Some(path.clone())).unwrap();
        let after = hw2.writes.first().map(|w| w.1).unwrap();
        // the coil kept its heat (0.5 s of cooling) and the minute's energy
        // still counts, so the new run does not start at full volume
        assert!(g2.speakers[0].t_coil < coil && g2.speakers[0].t_coil > coil - 10.0, "{} vs {coil}", g2.speakers[0].t_coil);
        assert!(after > 0.5, "restarted at {after:.1} dB after {before:.1} dB");
        drop(g2);
        // whereas a cold start would have
        let mut cold = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let _c = Guard::start(&mut cold, &cfg, 20.0, None).unwrap();
        assert_eq!(cold.writes.first().map(|w| w.1), Some(0.0));
        // an unreadable file starts hot, not cold
        std::fs::write(&path, "garbage").unwrap();
        let mut hw3 = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let _g3 = Guard::start(&mut hw3, &cfg, 20.0, Some(path.clone())).unwrap();
        assert!(hw3.writes.first().map(|w| w.1).unwrap() >= cfg.globals.t_reduction_max - 1e-9);
        std::fs::remove_dir_all(&dir).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0, None).unwrap();
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
}
