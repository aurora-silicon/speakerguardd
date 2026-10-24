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
mod log;
mod model;
mod startup;
mod state;

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
    unlock: ElemId,
    sample_rate: ElemId,
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
        Ok((Card { ctl, volume, volume_max: None, volume_min: 0, unlock, sample_rate }, name))
    }

    /// Take the interlock: every volume control first, then the unlock
    /// control.  The first ping, which is when the card lifts the volume
    /// limit, is the protection loop's to make.
    fn take_lock(&mut self) -> std::io::Result<()> {
        for id in &self.volume {
            self.ctl.lock(id)?;
        }
        self.ctl.lock(&self.unlock)?;
        self.volume_min = self.ctl.int_range(&self.volume[0])?.0;
        Ok(())
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

    fn write_reduction(&self, db: f64) -> std::io::Result<()> {
        // without the range nothing can be written (a failed write: the
        // loop leaves the card at its safe level)
        let max = self.volume_max.ok_or_else(|| std::io::Error::from_raw_os_error(libc::EAGAIN))?;
        // round the reduction up to the control's step: the card never
        // applies less than the model asked for
        let value = (max as f64 - (db * STEPS_PER_DB - 1e-9).ceil()).max(self.volume_min as f64) as i64;
        for id in &self.volume {
            self.ctl.write_int(id, value)?;
        }
        Ok(())
    }
}

/// The card and its sense capture.
struct RealHw {
    card: Card,
    globals: Globals,
    capture: Option<Capture>,
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
        self.card.ping()
    }
    fn set_reduction(&mut self, db: f64) -> std::io::Result<()> {
        self.card.write_reduction(db)
    }
    fn set_min(&mut self) -> std::io::Result<()> {
        self.card.write_min()
    }
    fn sample_rate(&mut self) -> std::io::Result<i64> {
        self.card.ctl.read_int(&self.card.sample_rate)
    }
    fn open_sense(&mut self, rate: u32) -> std::io::Result<()> {
        let g = &self.globals;
        self.capture = Some(Capture::open(self.card.ctl.card, g.sense_pcm, g.channels, rate, g.period)?);
        Ok(())
    }
    fn close_sense(&mut self) {
        self.capture = None;
    }
    fn read_period(&mut self, words: &mut [i32]) -> std::io::Result<Option<bool>> {
        match self.capture.as_mut() {
            Some(c) => c.read_period(words),
            None => Err(std::io::Error::from_raw_os_error(libc::EBADFD)),
        }
    }
    fn queued(&mut self) -> std::io::Result<u64> {
        match self.capture.as_mut() {
            Some(c) => c.queued(),
            None => Err(std::io::Error::from_raw_os_error(libc::EBADFD)),
        }
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
    let mut hw = RealHw { card, globals: cfg.globals.clone(), capture: None };
    catch_stop_signals();
    let mut guard = Guard::start(&mut hw, &cfg, args.max_reduction, args.state.clone())?;
    info!("{card_name}: {} volume control(s) and the unlock control locked", hw.card.volume.len());
    let mut stopping = false;
    loop {
        if STOP.load(Ordering::Relaxed) && !stopping {
            guard.stop();
            stopping = true;
        }
        if stopping && guard.released(&hw) {
            // the card has its volume back: the state says so
            guard.save(&hw);
            info!("stopped");
            return Ok(());
        }
        if let Err(e) = guard.tick(&mut hw) {
            // the next run starts from where this model was
            guard.save(&hw);
            return Err(e);
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
