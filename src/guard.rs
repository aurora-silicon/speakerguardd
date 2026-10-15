// SPDX-License-Identifier: MIT
//! The protection loop, behind a small hardware interface so that its
//! timing rules can be exercised without a sound card: the card's volume
//! lock, its speaker sample-rate control and the sense capture.

use std::error::Error;
use std::io;
use std::time::{Duration, Instant};

use crate::config::{Config, Globals};
use crate::health::{SenseExpired, SenseHealth};
use crate::model::{self, SpeakerState};
use crate::{debug, error, info, warn};

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
    fn sleep(&mut self, d: Duration);
    /// Lock every volume control and the unlock control, then ping once.
    fn take_lock(&mut self) -> io::Result<()>;
    /// Renew the card's volume lock ("Speaker Volume Unlock").
    fn ping(&mut self) -> io::Result<()>;
    /// Write the gain reduction (dB below the control's maximum).
    fn set_reduction(&mut self, db: f64) -> io::Result<()>;
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

pub struct Guard {
    globals: Globals,
    pub speakers: Vec<SpeakerState>,
    max_reduction: f64,
    health: SenseHealth,
    /// The open sense capture's stall clock; it lives and dies with the
    /// capture: one left over from an earlier stream would call the next
    /// stream stalled at its first empty wait, before its first period.
    stall: Option<StallClock>,
    words: Vec<i32>,
    last_ping: Instant,
    last_log: Instant,
    applied: f64,
    control_failures: u32,
}

impl Guard {
    /// Take the card's volume lock and start at full volume.
    pub fn start<H: Hw>(hw: &mut H, cfg: &Config, max_reduction: f64) -> Result<Guard, Box<dyn Error>> {
        if !max_reduction.is_finite() || max_reduction <= 0.0 {
            return Err("maximum reduction must be finite and positive".into());
        }
        let globals = cfg.globals.clone();
        let speakers = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), globals.t_ambient)).collect();
        let now = hw.now();
        let mut g = Guard {
            words: vec![0i32; globals.period * globals.channels],
            globals,
            speakers,
            max_reduction,
            health: SenseHealth::default(),
            stall: None,
            last_ping: now,
            last_log: now,
            applied: 0.0,
            control_failures: 0,
        };
        g.health.active(now);
        g.checked(hw, |hw| hw.take_lock())??;
        g.checked(hw, |hw| hw.set_reduction(0.0))??;
        g.last_ping = hw.now();
        Ok(g)
    }

    /// Check the sensing budget before and after potentially blocking I/O.
    fn checked<H: Hw, T>(&self, hw: &mut H, op: impl FnOnce(&mut H) -> T) -> Result<T, SenseExpired> {
        self.health.require(hw.now())?;
        let value = op(hw);
        self.health.require(hw.now())?;
        Ok(value)
    }

    /// Pings the volume lock.  A ping that answers ETIMEDOUT is the card's
    /// one-shot notice that the lock had lapsed (it does not re-arm the timer),
    /// so ping once more only while the sensing budget remains healthy;
    /// otherwise every volume write until the next ping fails with EINVAL
    /// against the safe limit.
    fn ping_lock<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        match self.checked(hw, |hw| hw.ping())? {
            Ok(()) => {}
            Err(e) if e.raw_os_error() == Some(libc::ETIMEDOUT) => {
                warn!("the card locked the volume while we were away; taking it back");
                self.checked(hw, |hw| hw.ping())?.map_err(|e| format!("lock ping: {e}"))?;
            }
            Err(e) => return Err(format!("lock ping: {e}").into()),
        }
        self.last_ping = hw.now();
        Ok(())
    }

    /// One pass of the loop: idle cooling, or one sense period.
    pub fn tick<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        let globals = self.globals.clone();
        // Sample-rate I/O makes playback state unknown until it returns.
        self.health.active(hw.now());
        self.health.require(hw.now())?;
        // playback state: the card reports the speakers' sample rate, 0 when closed
        let rate = match self.checked(hw, |hw| hw.sample_rate())? {
            Ok(v) => {
                self.control_failures = 0;
                u32::try_from(v).map_err(|_| "invalid negative or excessive speaker sample rate")?
            }
            Err(e) if transient(&e) && self.control_failures < MAX_CONTROL_FAILURES => {
                self.health.active(hw.now());
                self.control_failures += 1;
                warn!("sample-rate control: {e}; retrying ({})", self.control_failures);
                self.health.require(hw.now())?;
                self.checked(hw, |hw| hw.ping())??;
                hw.sleep(PING_INTERVAL);
                return Ok(());
            }
            Err(e) => return Err(format!("sample-rate control: {e}").into()),
        };
        if rate == 0 {
            self.health.idle();
            if self.stall.take().is_some() {
                hw.close_sense();
                info!("speakers closed");
            }
            // cool down with no power, keep the lock alive
            for s in self.speakers.iter_mut() {
                s.step(0.0, PING_INTERVAL.as_secs_f64(), globals.t_ambient);
                if !s.finite() {
                    return Err("nonfinite speaker model state".into());
                }
                s.govern(&globals);
                if !s.finite() {
                    return Err("nonfinite speaker governor".into());
                }
            }
            let reduction = model::reduction_db(&self.speakers);
            if let Err(e) = self.checked(hw, |hw| hw.set_reduction(reduction))? {
                if !transient(&e) || self.control_failures >= MAX_CONTROL_FAILURES {
                    return Err(format!("volume control: {e}").into());
                }
                self.control_failures += 1;
                warn!("volume control: {e}; retrying ({})", self.control_failures);
            }
            self.health.require(hw.now())?;
            self.ping_lock(hw)?;
            hw.sleep(PING_INTERVAL);
            return Ok(());
        }
        self.health.active(hw.now());
        if self.stall.is_none() {
            match self.checked(hw, |hw| hw.open_sense(rate))? {
                Ok(()) => {
                    info!("speakers open at {rate} Hz, sense capture running");
                    self.stall = Some(StallClock::default());
                }
                Err(e) => {
                    warn!("sense PCM: {e}; retrying");
                    self.health.require(hw.now())?;
                    self.checked(hw, |hw| hw.ping())??;
                    hw.sleep(PING_INTERVAL);
                    return Ok(());
                }
            }
        }
        let dt = globals.period as f64 / rate as f64;
        // The sense stream dies with the speaker back-end whenever playback is
        // torn down and rebuilt (EINVAL/EBADFD/EPIPE...).  That is not our
        // failure: drop the capture, keep the lock, and reopen once the card
        // reports a rate again.  Exiting here is what the listener heard as a
        // one-second dip to the safe level at high volume.
        let mut words = std::mem::take(&mut self.words);
        let read = self.checked(hw, |hw| hw.read_period(&mut words));
        self.words = words;
        let valid = match read? {
            Ok(Some(v)) => {
                self.stall.as_mut().unwrap().data();
                v
            }
            Ok(None) => {
                // No sense data within one ping interval.  Keep the lock alive
                // first; only a stream silent for a whole STALL_TIMEOUT is
                // dropped and reopened.
                let stalled = self.stall.as_mut().unwrap().silent(hw.now());
                self.health.require(hw.now())?;
                self.ping_lock(hw)?;
                if stalled {
                    warn!("sense stream stalled for {} ms; reopening", STALL_TIMEOUT.as_millis());
                    self.stall = None;
                    hw.close_sense();
                }
                return Ok(());
            }
            Err(e) if transient(&e) => {
                warn!("sense capture: {e}; reopening");
                self.stall = None;
                hw.close_sense();
                self.health.require(hw.now())?;
                self.ping_lock(hw)?;
                hw.sleep(PING_INTERVAL);
                return Ok(());
            }
            Err(e) => return Err(format!("sense capture: {e}").into()),
        };
        if !valid {
            warn!("incomplete sense period or overrun, step skipped");
        } else {
            for s in self.speakers.iter_mut() {
                let v = s.v_rms_of(&self.words, globals.channels);
                s.v_rms = v;
                let power = v * v / s.spec.z_nominal;
                if !v.is_finite() || !power.is_finite() || power < 0.0 || !dt.is_finite() || dt <= 0.0 {
                    return Err("invalid speaker sense voltage, power or period".into());
                }
                s.step(power, dt, globals.t_ambient);
                if !s.finite() {
                    return Err("nonfinite speaker model state".into());
                }
                s.govern(&globals);
                if !s.finite() {
                    return Err("nonfinite speaker governor".into());
                }
            }
        }
        // Renew only after complete frames and all derived model state passed.
        if valid {
            self.health.valid(hw.now())?;
        }
        let reduction = model::reduction_db(&self.speakers);
        if !reduction.is_finite() {
            return Err("nonfinite speaker reduction".into());
        }
        if reduction >= self.max_reduction {
            error!("gain reduction {reduction:.1} dB reached the limit; leaving the card locked");
            return Err("maximum reduction reached".into());
        }
        if (reduction - self.applied).abs() >= 0.5 {
            info!("gain reduction {reduction:.1} dB");
            self.applied = reduction;
        }
        match self.checked(hw, |hw| hw.set_reduction(reduction))? {
            Ok(()) => self.control_failures = 0,
            Err(e) if transient(&e) && self.control_failures < MAX_CONTROL_FAILURES => {
                self.control_failures += 1;
                warn!("volume control: {e}; retrying ({})", self.control_failures);
            }
            Err(e) => return Err(format!("volume control: {e}").into()),
        }
        if hw.now().duration_since(self.last_ping) >= PING_INTERVAL {
            self.health.require(hw.now())?;
            self.ping_lock(hw)?;
        }
        if hw.now().duration_since(self.last_log) >= LOG_INTERVAL {
            for s in &self.speakers {
                debug!(
                    "{}: {:.3} Vrms {:.1} mW coil {:.1} C magnet {:.1} C reduction {:.1} dB",
                    s.spec.name,
                    s.v_rms,
                    s.power * 1000.0,
                    s.t_coil,
                    s.t_magnet,
                    s.reduction_db
                );
            }
            self.last_log = hw.now();
        }
        Ok(())
    }
}

#[cfg(test)]
pub mod fake {
    //! A simulated card: a virtual clock, the macaudio volume-lock rules
    //! (the lock lapses 250 ms after the last ping; the next ping answers
    //! ETIMEDOUT once; volume writes above the safe level fail while
    //! lapsed) and a sense stream that produces one period every
    //! period/rate seconds, carrying the program after the card's gain.
    use super::*;

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
            Ok(if (self.playing)(self.t) { 48000 } else { 0 })
        }
        fn open_sense(&mut self, _rate: u32) -> io::Result<()> {
            self.sense_open = true;
            self.opens += 1;
            self.next_period = self.t + self.period;
            Ok(())
        }
        fn close_sense(&mut self) {
            self.sense_open = false;
        }
        fn read_period(&mut self, words: &mut [i32]) -> io::Result<Option<bool>> {
            assert!(self.sense_open, "read on a closed capture");
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0).unwrap();
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
        let mut g = Guard::start(&mut hw, &cfg, 20.0).unwrap();
        run_until(&mut g, &mut hw, Duration::from_secs(5)).unwrap();
        assert!(hw.lapses.is_empty(), "{:?}", hw.lapses);
        assert!(!hw.sense_open);
    }
}
