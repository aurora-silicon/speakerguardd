// SPDX-License-Identifier: MIT
//! speakerguardd - feed-forward speaker protection for amplifiers without
//! sense lines.
//!
//! The card's speaker sense PCM carries the words the amplifier receives.
//! Each period they are turned into the power in every voice coil, run
//! through a two-stage thermal model, and the coil temperature governs the
//! card's speaker volume control.  The card only unlocks that control while
//! this daemon holds the lock on it and keeps writing the unlock control
//! (the same interlock the other Macs' speakersafetyd uses).  Whenever the
//! daemon cannot vouch for the speakers -- no complete sense data for longer
//! than the stall timeout, or a model asking for more than
//! `--max-reduction` -- it stops renewing the lock, the card falls back to
//! its safe volume, and the daemon takes the lock back once it can.

mod alsa;
mod config;
mod guard;
#[cfg(test)]
mod harness;
mod log;
mod model;
mod startup;
mod state;
mod stats;

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use alsa::{Capture, Ctl, ElemId};
use config::{Config, Globals};
use guard::{Guard, Hw};
use log::Level;

/// The value the card's "Speaker Volume Unlock" control expects: the kernel
/// compares the written long with (s32)0xdec1be15.
const UNLOCK_MAGIC: i64 = 0xdec1be15u32 as i32 as i64;
/// Volume control steps per dB (t8140-aop-audio: 0.5 dB steps).
const STEPS_PER_DB: f64 = 2.0;

/// The version and the source it was built from (the packaging sets
/// SPEAKERGUARDD_COMMIT at build time), for `--version` and the journal.
fn version() -> String {
    format!("speakerguardd {} ({})", env!("CARGO_PKG_VERSION"), option_env!("SPEAKERGUARDD_COMMIT").unwrap_or("commit unknown"))
}

struct Args {
    config_path: PathBuf,
    max_reduction: f64,
    card: Option<String>,
    card_wait: Duration,
    state: Option<PathBuf>,
    verbosity: Level,
}

fn usage() -> ! {
    eprintln!(
        "Usage: speakerguardd [OPTIONS]\n\n\
         Options:\n  \
         -c, --config-path <DIR>         Directory holding <vendor>/<model>.conf\n  \
         -C, --card <ID>                 ALSA card id (default: the macaudio card)\n  \
         --wait-for-card <SECONDS>       Wait for card discovery before failing (default 30; 0 disables waiting)\n  \
         -m, --max-reduction <DB>        Gain reduction beyond which the card's safe level takes over (default 20)\n  \
         -s, --state <FILE>              Model state kept across restarts (default /run/speakerguardd/state)\n  \
         -v, --verbose                   More logging (repeatable)\n  \
         -q, --quiet                     Less logging\n  \
         -V, --version                   The version and source commit\n  \
         -h, --help                      This text"
    );
    std::process::exit(2)
}

fn parse_args() -> Args {
    let mut args = Args { config_path: PathBuf::from("/usr/share/speakerguardd"), max_reduction: 20.0, card: None, card_wait: Duration::from_secs(30), state: Some(PathBuf::from("/run/speakerguardd/state")), verbosity: Level::Info };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "-c" | "--config-path" => args.config_path = PathBuf::from(it.next().unwrap_or_else(|| usage())),
            "-C" | "--card" => args.card = Some(it.next().unwrap_or_else(|| usage())),
            "--wait-for-card" => args.card_wait = Duration::from_secs(it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| usage())),
            "-s" | "--state" => {
                let v = it.next().unwrap_or_else(|| usage());
                args.state = if v.is_empty() { None } else { Some(PathBuf::from(v)) };
            }
            "-m" | "--max-reduction" => {
                args.max_reduction = it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| usage());
            }
            "-v" | "--verbose" => args.verbosity = Level::Debug,
            "-q" | "--quiet" => args.verbosity = Level::Warn,
            "-V" | "--version" => {
                println!("{}", version());
                std::process::exit(0)
            }
            _ => usage(),
        }
    }
    args
}

/// The card id: Apple's ADT-derived "AppleJ<model>" as macaudio sets it.
/// Without `-C`, the card the macaudio driver registered.
fn find_card(requested: Option<&str>) -> std::io::Result<(Ctl, String)> {
    if let Some(id) = requested {
        return Ok((Ctl::open_by_id(id)?, id.to_string()));
    }
    let cards = std::fs::read_to_string("/proc/asound/cards")?;
    match startup::macaudio_card(&cards) {
        Some(id) => Ok((Ctl::open_by_id(&id)?, id)),
        None => Err(std::io::Error::new(std::io::ErrorKind::NotFound, "no macaudio sound card")),
    }
}

struct Card {
    ctl: Ctl,
    volume: Vec<ElemId>,
    /// The volume controls' maximum with the lock lifted, read after the
    /// first successful ping.
    volume_max: Option<i64>,
    volume_min: i64,
    /// The most they accept while the volume is locked (macaudio limits
    /// the range to the card's safe level), read when the lock is taken.
    locked_max: Option<i64>,
    unlock: ElemId,
    sample_rate: ElemId,
    /// Subscribed to the card's change events: the sample-rate control's
    /// changes wake the loop instead of waiting for its next poll.
    events: bool,
    /// A sample-rate change event arrived since the loop last asked.
    rate_event: bool,
}

impl Card {
    fn open(id: Option<&str>, cfg: &Config) -> Result<(Card, String), Box<dyn std::error::Error>> {
        let (ctl, name) = find_card(id)?;
        let volume = ctl.find(&format!("* {}", cfg.controls.volume))?;
        if volume.is_empty() {
            return Err(format!("card {name} has no {:?} control", cfg.controls.volume).into());
        }
        let unlock = ctl.find_one(&cfg.controls.unlock)?;
        let sample_rate = ctl.find_one(&cfg.controls.sample_rate)?;
        let events = match ctl.subscribe() {
            Ok(()) => true,
            Err(e) => {
                warn!("control events: {e}; polling the sample rate instead");
                false
            }
        };
        Ok((Card { ctl, volume, volume_max: None, volume_min: 0, locked_max: None, unlock, sample_rate, events, rate_event: false }, name))
    }

    /// Take the interlock: every volume control first, then the unlock
    /// control.  The first ping, which is when the card lifts the volume
    /// limit, is the protection loop's to make.
    fn take_lock(&mut self) -> std::io::Result<()> {
        for id in &self.volume {
            self.ctl.lock(id)?;
        }
        self.ctl.lock(&self.unlock)?;
        let (min, locked) = self.ctl.int_range(&self.volume[0])?;
        self.volume_min = min;
        self.locked_max = Some(locked);
        Ok(())
    }

    /// Give the unlock control's lock up and take it again: macaudio locks
    /// the volume at once and clears the lease (a new owner starts with
    /// none), whatever was left of it.
    fn relinquish(&mut self) -> std::io::Result<()> {
        self.ctl.unlock(&self.unlock)?;
        self.ctl.lock(&self.unlock)
    }

    /// Every volume control to the card's safe value: the most the card
    /// accepts while the volume is locked (macaudio limits the controls'
    /// range to it there), or, once the range with the lock lifted is
    /// known, that maximum less `safe_db`, whichever is lower.  The minimum
    /// where that write fails (the card always accepts it).  The reduction
    /// now applied (dB), from the value written once the maximum is known;
    /// before that, the card's own safe level, taken to be `safe_db` until
    /// the first ping checks it (review r7 Y5).
    fn write_safe(&self, safe_db: f64) -> std::io::Result<f64> {
        let (_, limit) = self.ctl.int_range(&self.volume[0])?;
        let value = match self.volume_max {
            Some(max) => limit.min(max - (safe_db * STEPS_PER_DB - 1e-9).ceil() as i64),
            None => limit,
        }
        .max(self.volume_min);
        match self.volume.iter().try_for_each(|id| self.ctl.write_int(id, value)) {
            Ok(()) => Ok(match self.volume_max {
                Some(max) => (max - value) as f64 / STEPS_PER_DB,
                None => safe_db,
            }),
            Err(e) => {
                warn!("volume control: {e} at the safe value; writing the minimum");
                self.write_min()?;
                Ok(guard::MUTE_DB)
            }
        }
    }

    /// Every volume control to its minimum: within the card's limit
    /// whether or not the volume is locked.
    fn write_min(&self) -> std::io::Result<()> {
        for id in &self.volume {
            self.ctl.write_int(id, self.volume_min)?;
        }
        Ok(())
    }

    /// Renew the lock; the control's range is read after the first ping
    /// lifted the limit.
    fn ping(&mut self) -> std::io::Result<()> {
        self.ctl.write_int(&self.unlock, UNLOCK_MAGIC)?;
        if self.volume_max.is_none() {
            let (_, max) = self.ctl.int_range(&self.volume[0])?;
            self.volume_max = Some(max);
        }
        Ok(())
    }

    /// Drain the queued events; true while a sample-rate change is pending.
    fn drain(&mut self) -> std::io::Result<bool> {
        if self.events && self.ctl.events(self.sample_rate.numid)? {
            self.rate_event = true;
        }
        Ok(self.rate_event)
    }

    /// Returns the reduction the controls now apply (dB).
    fn write_reduction(&self, db: f64) -> std::io::Result<f64> {
        // without the range nothing can be written (a failed write: the
        // loop leaves the card at its safe level)
        let max = self.volume_max.ok_or_else(|| std::io::Error::from_raw_os_error(libc::EAGAIN))?;
        // round the reduction up to the control's step: the card never
        // applies less than the model asked for
        let value = (max as f64 - (db * STEPS_PER_DB - 1e-9).ceil()).max(self.volume_min as f64) as i64;
        for id in &self.volume {
            self.ctl.write_int(id, value)?;
        }
        Ok((max - value) as f64 / STEPS_PER_DB)
    }
}

/// The card and its sense capture.
struct RealHw {
    card: Card,
    globals: Globals,
    capture: Option<Capture>,
    /// How long its calls hold the daemon up (summarised to the journal).
    stats: stats::Stats,
}

impl RealHw {
    /// Time one call into `kind`'s counter.
    fn timed<T>(&mut self, kind: stats::Call, f: impl FnOnce(&mut Self) -> T) -> T {
        let t = Instant::now();
        let r = f(self);
        self.stats.call(kind, t.elapsed());
        r
    }
}

impl Hw for RealHw {
    fn now(&self) -> Instant {
        Instant::now()
    }
    fn boottime(&self) -> Duration {
        let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: a valid, writable timespec.
        unsafe { libc::clock_gettime(libc::CLOCK_BOOTTIME, &mut ts) };
        Duration::new(ts.tv_sec as u64, ts.tv_nsec as u32)
    }
    fn sleep(&mut self, d: Duration) {
        thread::sleep(d)
    }
    fn take_lock(&mut self) -> std::io::Result<()> {
        self.card.take_lock()
    }
    fn ping(&mut self) -> std::io::Result<()> {
        let r = self.timed(stats::Call::Ping, |h| h.card.ping());
        if r.is_ok() {
            self.stats.renewed(Instant::now());
        }
        r
    }
    fn set_reduction(&mut self, db: f64) -> std::io::Result<f64> {
        self.timed(stats::Call::Write, |h| h.card.write_reduction(db))
    }
    fn step_db(&self) -> f64 {
        1.0 / STEPS_PER_DB
    }
    fn set_safe(&mut self) -> std::io::Result<f64> {
        let safe = self.globals.safe_reduction;
        self.timed(stats::Call::Write, |h| h.card.write_safe(safe))
    }
    fn card_safe_db(&self) -> Option<f64> {
        let (max, locked) = (self.card.volume_max?, self.card.locked_max?);
        Some((max - locked) as f64 / STEPS_PER_DB)
    }
    fn relinquish(&mut self) -> std::io::Result<()> {
        self.stats.released();
        self.timed(stats::Call::Relinquish, |h| h.card.relinquish())
    }
    fn sample_rate(&mut self) -> std::io::Result<i64> {
        self.timed(stats::Call::Rate, |h| h.card.ctl.read_int(&h.card.sample_rate))
    }
    fn open_sense(&mut self, rate: u32) -> std::io::Result<()> {
        let g = self.globals.clone();
        let c = self.timed(stats::Call::Open, |h| Capture::open(h.card.ctl.card, g.sense_pcm, g.channels, rate, g.period))?;
        self.capture = Some(c);
        Ok(())
    }
    fn close_sense(&mut self) {
        self.capture = None;
    }
    /// One sense period, or None once a sample-rate change arrives (the
    /// loop then reads the rate at once) or after one poll.
    fn read_period(&mut self, words: &mut [i32]) -> std::io::Result<Option<bool>> {
        let Some(c) = self.capture.as_mut() else {
            return Err(std::io::Error::from_raw_os_error(libc::EBADFD));
        };
        if !self.card.events {
            return c.read_period(words);
        }
        c.ensure_started()?;
        let deadline = Instant::now() + Duration::from_millis(alsa::PERIOD_WAIT_MS as u64);
        loop {
            if self.card.drain()? {
                return Ok(None);
            }
            let left = deadline.saturating_duration_since(Instant::now());
            let ms = (left.as_micros().div_ceil(1000)) as libc::c_int;
            let (pcm, ctl) = alsa::poll2(c.fd(), Some(self.card.ctl.fd()), ms)?;
            if pcm {
                return c.read_ready(words);
            }
            if !ctl || Instant::now() >= deadline {
                return Ok(None);
            }
        }
    }
    fn read_tail(&mut self, words: &mut [i32]) -> std::io::Result<usize> {
        match self.capture.as_mut() {
            Some(c) => c.read_tail(words),
            None => Ok(0),
        }
    }
    fn wait_rate_change(&mut self, timeout: Duration) -> bool {
        if !self.card.events {
            thread::sleep(timeout);
            return false;
        }
        let deadline = Instant::now() + timeout;
        loop {
            match self.card.drain() {
                Ok(true) => return true,
                Ok(false) => {}
                Err(_) => {
                    // the events are only a wake-up: fall back to the poll
                    thread::sleep(deadline.saturating_duration_since(Instant::now()));
                    return false;
                }
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return false;
            }
            let ms = (left.as_micros().div_ceil(1000)) as libc::c_int;
            if alsa::poll2(self.card.ctl.fd(), None, ms).is_err() {
                thread::sleep(deadline.saturating_duration_since(Instant::now()));
                return false;
            }
        }
    }
    fn rate_events(&mut self) -> bool {
        let pending = self.timed(stats::Call::Events, |h| h.card.drain().unwrap_or(true));
        self.card.rate_event = false;
        pending
    }
    fn events_available(&self) -> bool {
        self.card.events
    }
    fn queued(&mut self) -> std::io::Result<u64> {
        self.timed(stats::Call::Position, |h| match h.capture.as_mut() {
            Some(c) => c.queued(),
            None => Err(std::io::Error::from_raw_os_error(libc::EBADFD)),
        })
    }
}

/// Set by SIGTERM/SIGINT: hand the card back and exit.
static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_stop(_: libc::c_int) {
    STOP.store(true, Ordering::Relaxed);
}

fn catch_stop_signals() {
    for sig in [libc::SIGTERM, libc::SIGINT] {
        // SAFETY: the handler only stores to an atomic.
        unsafe { libc::signal(sig, on_stop as *const () as libc::sighandler_t) };
    }
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    info!("{}", version());
    // Time the daemon is held up is bounded only by the kernel's lease: no
    // page of it may have to come back from storage first.  The unit gives
    // it a real-time priority below PipeWire's and the locked-memory limit.
    // SAFETY: a plain system call on this process.
    if unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) } != 0 {
        warn!("mlockall: {}; page faults can hold the daemon up", std::io::Error::last_os_error());
    }
    // the model file is named after the card: AppleJ700 -> apple/j700.conf
    // Discovery only: no controls are locked or written until it succeeds,
    // the configuration is loaded and Guard::start takes the interlock.
    let mut waiting = false;
    let (ctl, name) = startup::wait_for_card(args.card_wait, || {
        let result = find_card(args.card.as_deref());
        if !waiting && args.card_wait > Duration::ZERO
            && result.as_ref().is_err_and(|e| e.kind() == std::io::ErrorKind::NotFound)
        {
            info!("waiting up to {} s for sound card {}", args.card_wait.as_secs(), args.card.as_deref().unwrap_or("(macaudio)"));
            waiting = true;
        }
        result
    })?;
    drop(ctl);
    let model = name.strip_prefix("Apple").unwrap_or(&name).to_lowercase();
    let path = args.config_path.join("apple").join(format!("{model}.conf"));
    let cfg = Config::load(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    info!("{}: {} speakers, sense PCM {}, period {} frames", path.display(), cfg.speakers.len(), cfg.globals.sense_pcm, cfg.globals.period);
    let (card, card_name) = Card::open(Some(&name), &cfg)?;
    let mut hw = RealHw { card, globals: cfg.globals.clone(), capture: None, stats: stats::Stats::new(Instant::now()) };
    catch_stop_signals();
    let mut guard = Guard::start(&mut hw, &cfg, args.max_reduction, args.state.clone())?;
    info!("{card_name}: {} volume control(s) and the unlock control locked", hw.card.volume.len());
    let mut stopping = false;
    loop {
        if STOP.load(Ordering::Relaxed) && !stopping {
            if let Err(e) = guard.stop(&mut hw) {
                // not a clean stop: the state stays a status readout and the
                // exit gives the lock up
                guard.save(&hw);
                return Err(e);
            }
            stopping = true;
        }
        if stopping && guard.released(&hw) {
            // the lock given up and drained, every moment accounted
            guard.final_save(&hw)?;
            info!("stopped");
            return Ok(());
        }
        if let Err(e) = guard.tick(&mut hw) {
            // the next run starts from where this model was
            guard.save(&hw);
            return Err(e);
        }
        let now = Instant::now();
        if hw.stats.due(now) {
            info!("{}", hw.stats.summary(now));
        }
    }
}

fn main() -> ExitCode {
    let args = parse_args();
    log::set_verbosity(args.verbosity);
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            error!("{e}");
            ExitCode::FAILURE
        }
    }
}
