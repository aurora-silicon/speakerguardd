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
mod engine;
mod guard;
#[cfg(test)]
mod harness;
mod log;
mod model;
mod record;
mod startup;
mod state;

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use alsa::{Capture, Ctl, ElemId};
use config::{Config, Globals, Governor, SenseFormat};
use guard::{Guard, Hw};
use log::Level;

/// The value the card's "Speaker Volume Unlock" control expects: the kernel
/// compares the written long with (s32)0xdec1be15.
const UNLOCK_MAGIC: i64 = 0xdec1be15u32 as i32 as i64;
/// Volume control steps per dB (t8140-aop-audio: 0.5 dB steps).
const STEPS_PER_DB: f64 = 2.0;
/// The exit status of a configuration the daemon refuses to govern (an
/// unqualified machine, a missing opt-in, an unreadable file), before the
/// card is touched.  A restart cannot change it, and the unit does not
/// restart on it (RestartPreventExitStatus), so a drop-in's OnFailure=
/// hands the speakers back to the protection it replaced at once.
const EXIT_REFUSED: u8 = 78;

/// A refusal to govern (EXIT_REFUSED).
#[derive(Debug)]
struct Refused(String);

impl std::fmt::Display for Refused {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Refused {}

fn exit_status(e: &(dyn std::error::Error + 'static)) -> u8 {
    if e.is::<Refused>() { EXIT_REFUSED } else { 1 }
}

struct Args {
    config_path: PathBuf,
    max_reduction: Option<f64>,
    card: Option<String>,
    card_wait: Duration,
    state: Option<PathBuf>,
    verbosity: Level,
    shadow: bool,
    governor: Option<Governor>,
    /// Some(None): --guard none.
    guard: Option<Option<Governor>>,
    horizon: Option<f64>,
    ctrl_headroom: Option<f64>,
    unified_caps: bool,
    experimental_unified: bool,
    allow_unqualified: bool,
    record: Option<PathBuf>,
    record_max_mb: u64,
    replay: Option<PathBuf>,
    model: Option<String>,
    gain_db: f64,
    csv: Option<PathBuf>,
}

fn usage() -> ! {
    eprintln!(
        "Usage: speakerguardd [OPTIONS]\n\n\
         Options:\n  \
         -c, --config-path <DIR>         Directory holding <vendor>/<model>.conf\n  \
         -C, --card <ID>                 ALSA card id (default: the macaudio card)\n  \
         --wait-for-card <SECONDS>       Wait for card discovery before failing (default 30; 0 disables waiting)\n  \
         -m, --max-reduction <DB>        Gain reduction beyond which the card's safe level takes over (default: the config's, else 20)\n  \
         -s, --state <FILE>              Model state kept across restarts (default /run/speakerguardd/state)\n  \
         --governor <NAME>               legacy, asahi or unified (default: the config's, else legacy)\n  \
         --guard <NAME>                  asahi: the unified governor's card also gets the asahi governor's reductions (default: the config's, else none)\n  \
         --shadow                        Run every governor and log them to shadow.jsonl next to the state\n  \
         --horizon <S>                   The unified governor's prediction horizon (default: the config's)\n  \
         --ctrl-headroom <K>             The unified governor's margin under the working limit (default: the config's)\n  \
         --unified-caps                  macOS-style caps on the unified governor's thermal and power axes\n  \
         --experimental-unified          Let the unified governor write the card without the guard\n  \
         --allow-unqualified             Govern a machine whose configuration says qualified = false\n  \
         --record <FILE>                 Record what the guard sees and decides, for --replay\n  \
         --record-max-mb <MB>            Stop recording past this size (default 512)\n  \
         --replay <FILE>                 Run the guard on a recording offline (no card); exit 3 if it cannot certify it\n  \
         --model <NAME>                  With --replay: the config (apple/<NAME>.conf)\n  \
         --gain-db <DB>                  With --replay: play the recording this much louder\n  \
         --csv <FILE>                    With --replay: one line per period\n  \
         -v, --verbose                   More logging (repeatable)\n  \
         -q, --quiet                     Less logging\n  \
         -h, --help                      This text"
    );
    std::process::exit(2)
}

fn parse_args() -> Args {
    let mut args = Args {
        config_path: PathBuf::from("/usr/share/speakerguardd"),
        max_reduction: None,
        card: None,
        card_wait: Duration::from_secs(30),
        state: Some(PathBuf::from("/run/speakerguardd/state")),
        verbosity: Level::Info,
        shadow: false,
        governor: None,
        guard: None,
        horizon: None,
        ctrl_headroom: None,
        unified_caps: false,
        experimental_unified: false,
        allow_unqualified: false,
        record: None,
        record_max_mb: 512,
        replay: None,
        model: None,
        gain_db: 0.0,
        csv: None,
    };
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
                args.max_reduction = Some(it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| usage()));
            }
            "--shadow" => args.shadow = true,
            "--governor" => args.governor = Some(it.next().as_deref().and_then(Governor::parse).unwrap_or_else(|| usage())),
            "--guard" => {
                args.guard = Some(match it.next().as_deref() {
                    Some("asahi") => Some(Governor::Asahi),
                    Some("none") => None,
                    _ => usage(),
                })
            }
            "--unified-caps" => args.unified_caps = true,
            "--experimental-unified" => args.experimental_unified = true,
            "--allow-unqualified" => args.allow_unqualified = true,
            "--record" => args.record = Some(PathBuf::from(it.next().unwrap_or_else(|| usage()))),
            "--record-max-mb" => args.record_max_mb = it.next().and_then(|v| v.parse().ok()).filter(|&m: &u64| m > 0).unwrap_or_else(|| usage()),
            "--replay" => args.replay = Some(PathBuf::from(it.next().unwrap_or_else(|| usage()))),
            "--model" => args.model = Some(it.next().unwrap_or_else(|| usage())),
            "--gain-db" => args.gain_db = it.next().and_then(|v| v.parse().ok()).filter(|g: &f64| g.is_finite()).unwrap_or_else(|| usage()),
            "--csv" => args.csv = Some(PathBuf::from(it.next().unwrap_or_else(|| usage()))),
            "--ctrl-headroom" => args.ctrl_headroom = Some(it.next().and_then(|v| v.parse().ok()).filter(|h: &f64| h.is_finite() && *h >= 0.0).unwrap_or_else(|| usage())),
            "--horizon" => args.horizon = Some(it.next().and_then(|v| v.parse().ok()).filter(|h: &f64| h.is_finite() && *h > 0.0).unwrap_or_else(|| usage())),
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
    /// One volume control per speaker ("<speaker> <volume>"), in speaker
    /// order, on cards that have them (V/I machines).
    per_speaker: Option<Vec<ElemId>>,
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
        let named: Vec<Option<ElemId>> = cfg.speakers.iter().map(|s| ctl.find_one(&format!("{} {}", s.name, cfg.controls.volume)).ok()).collect();
        // every per-speaker control is one of the locked volume controls
        let per_speaker = if named.iter().all(|n| n.is_some_and(|id| volume.iter().any(|v| v.numid == id.numid))) { Some(named.into_iter().flatten().collect()) } else { None };
        // V/I machines: sense on, amplifier at the gain the model assumes
        // (speakersafetyd does the same; the kernel clamps the range).  None
        // of these lifts the volume lock.
        for s in &cfg.speakers {
            for sw in [&cfg.controls.vsense, &cfg.controls.isense].into_iter().flatten() {
                let id = ctl.find_one(&format!("{} {}", s.name, sw))?;
                ctl.lock(&id)?;
                ctl.write_int(&id, 1)?;
                if ctl.read_int(&id)? != 1 {
                    return Err(format!("{} {sw}: did not turn on", s.name).into());
                }
            }
            if let Some(gain) = &cfg.controls.amp_gain {
                let id = ctl.find_one(&format!("{} {}", s.name, gain))?;
                ctl.lock(&id)?;
                let (_, top) = ctl.int_range(&id)?;
                // the profile's own index where it names one: a control
                // that goes further would play louder than every worst
                // case assumes; one that stops short plays quieter
                let max = cfg.globals.amp_gain_index.map_or(top, |want| want.min(top));
                ctl.write_int(&id, max)?;
                if ctl.read_int(&id)? != max {
                    return Err(format!("{} {gain}: did not take {max}", s.name).into());
                }
                if max < top {
                    info!("{}: {gain} at {max} of {top} (the profile's amp_gain_index), {:.1} dBV assumed", s.name, cfg.globals.amp_gain_dbv);
                } else {
                    info!("{}: {gain} at its maximum ({max}), {:.1} dBV assumed", s.name, cfg.globals.amp_gain_dbv);
                }
            }
        }
        let events = match ctl.subscribe() {
            Ok(()) => true,
            Err(e) => {
                warn!("control events: {e}; polling the sample rate instead");
                false
            }
        };
        Ok((Card { ctl, volume, volume_max: None, volume_min: 0, per_speaker, unlock, sample_rate, events, rate_event: false }, name))
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

    /// Give the unlock control's lock up and take it again: macaudio locks
    /// the volume at once and clears the lease (a new owner starts with
    /// none), whatever was left of it.
    fn relinquish(&mut self) -> std::io::Result<()> {
        self.ctl.unlock(&self.unlock)?;
        self.ctl.lock(&self.unlock)
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

    /// The control value for a reduction, rounded up to the control's
    /// step: the card never applies less than the model asked for.
    fn value_of(&self, db: f64) -> std::io::Result<i64> {
        // without the range nothing can be written (a failed write: the
        // loop leaves the card at its safe level)
        let max = self.volume_max.ok_or_else(|| std::io::Error::from_raw_os_error(libc::EAGAIN))?;
        if !db.is_finite() {
            return Err(std::io::Error::from_raw_os_error(libc::EINVAL));
        }
        Ok((max as f64 - (db * STEPS_PER_DB - 1e-9).ceil()).max(self.volume_min as f64) as i64)
    }

    /// The reduction a control value applies (dB).
    fn db_of(&self, value: i64) -> f64 {
        (self.volume_max.unwrap_or(value) - value) as f64 / STEPS_PER_DB
    }

    /// Returns the reduction the controls now apply (dB).
    fn write_reduction(&self, db: f64) -> std::io::Result<f64> {
        let value = self.value_of(db)?;
        for id in &self.volume {
            self.ctl.write_int(id, value)?;
        }
        Ok(self.db_of(value))
    }

    /// One reduction per speaker on cards with a control each, the largest
    /// on the others.  Returns what each speaker's control now applies.
    fn write_reductions(&self, db: &[f64]) -> std::io::Result<Vec<f64>> {
        match &self.per_speaker {
            Some(ids) if ids.len() == db.len() => {
                let values = db.iter().map(|&d| self.value_of(d)).collect::<std::io::Result<Vec<i64>>>()?;
                for (id, &v) in ids.iter().zip(&values) {
                    self.ctl.write_int(id, v)?;
                }
                Ok(values.iter().map(|&v| self.db_of(v)).collect())
            }
            _ => {
                let applied = self.write_reduction(db.iter().cloned().fold(0.0, f64::max))?;
                Ok(vec![applied; db.len()])
            }
        }
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
    fn set_reduction(&mut self, db: f64) -> std::io::Result<f64> {
        self.card.write_reduction(db)
    }
    fn set_reductions(&mut self, db: &[f64]) -> std::io::Result<Vec<f64>> {
        self.card.write_reductions(db)
    }
    fn step_db(&self) -> f64 {
        1.0 / STEPS_PER_DB
    }
    fn set_min(&mut self) -> std::io::Result<()> {
        self.card.write_min()
    }
    fn relinquish(&mut self) -> std::io::Result<()> {
        self.card.relinquish()
    }
    fn sample_rate(&mut self) -> std::io::Result<i64> {
        self.card.ctl.read_int(&self.card.sample_rate)
    }
    fn open_sense(&mut self, rate: u32) -> std::io::Result<()> {
        let g = &self.globals;
        self.capture = Some(Capture::open(self.card.ctl.card, g.sense_pcm, g.channels, rate, g.period, g.sense_format == SenseFormat::S16)?);
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
        let pending = self.card.drain().unwrap_or(true);
        self.card.rate_event = false;
        pending
    }
    fn events_available(&self) -> bool {
        self.card.events
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
    // before any control of the card is touched
    let cfg = load_config(&path, args)?;
    info!("{}: {} speakers{}, sense PCM {}, period {} frames", path.display(), cfg.speakers.len(), if cfg.has_vi() { " with V/I sense" } else { "" }, cfg.globals.sense_pcm, cfg.globals.period);
    let (card, card_name) = Card::open(Some(&name), &cfg)?;
    let mut hw = RealHw { card, globals: cfg.globals.clone(), capture: None };
    catch_stop_signals();
    let max_reduction = args.max_reduction.unwrap_or(cfg.globals.max_reduction);
    let mut guard = Guard::start(&mut hw, &cfg, max_reduction, args.state.clone())?;
    if let Some(path) = &args.record {
        let max = args.record_max_mb.saturating_mul(1 << 20);
        guard.attach_recorder(&hw, record::Recorder::create(path, &cfg, hw.card.events, max)?);
        info!("recording to {}", path.display());
    }
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
    }
}

/// The machine's configuration with the command line's choices over it,
/// or the refusal to govern it.
fn load_config(path: &std::path::Path, args: &Args) -> Result<Config, Refused> {
    let mut cfg = Config::load(path).map_err(|e| Refused(format!("{}: {e}", path.display())))?;
    override_config(&mut cfg, args).map_err(|e| Refused(format!("{}: {e}", path.display())))?;
    Ok(cfg)
}

/// Command-line governor and shadow choices over the config's, and the
/// check that the result may write the card.
fn override_config(cfg: &mut Config, args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(g) = args.governor {
        cfg.globals.governor = g;
    }
    if let Some(g) = args.guard {
        cfg.globals.guard = g;
    }
    cfg.globals.shadow |= args.shadow;
    if let Some(h) = args.horizon {
        cfg.globals.horizon_s = h;
    }
    if let Some(h) = args.ctrl_headroom {
        cfg.globals.t_ctrl_headroom = h;
    }
    cfg.globals.unified_caps |= args.unified_caps;
    cfg.allow_unified = args.experimental_unified;
    cfg.allow_unqualified = args.allow_unqualified;
    if cfg.allow_unqualified && !cfg.globals.qualified {
        warn!("governing an unqualified configuration (--allow-unqualified)");
    }
    cfg.check_governor()?;
    Ok(())
}

/// --replay: the guard on a recording, offline.  Exit 3 when the replay
/// cannot certify the recording.
fn replay(args: &Args, path: &std::path::Path) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let model = args.model.as_deref().ok_or("--replay needs --model")?;
    let conf = args.config_path.join("apple").join(format!("{model}.conf"));
    let mut cfg = Config::load(&conf).map_err(|e| format!("{}: {e}", conf.display()))?;
    cfg.globals.shadow |= args.shadow;
    if let Some(h) = args.horizon {
        cfg.globals.horizon_s = h;
    }
    if let Some(h) = args.ctrl_headroom {
        cfg.globals.t_ctrl_headroom = h;
    }
    cfg.globals.unified_caps |= args.unified_caps;
    let out = record::replay(path, &cfg, &record::Replay { gain_db: args.gain_db, governor: args.governor, guard: args.guard, max_reduction: args.max_reduction, csv: args.csv.clone() })?;
    print!("{}", out.text);
    Ok(if out.certified { ExitCode::SUCCESS } else { ExitCode::from(3) })
}

fn main() -> ExitCode {
    let args = parse_args();
    log::set_verbosity(args.verbosity);
    if let Some(path) = &args.replay {
        return match replay(&args, path) {
            Ok(code) => code,
            Err(e) => {
                error!("{e}");
                ExitCode::FAILURE
            }
        };
    }
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            error!("{e}");
            ExitCode::from(exit_status(e.as_ref()))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(conf: &std::path::Path) -> Args {
        Args {
            config_path: conf.to_path_buf(),
            max_reduction: None,
            card: None,
            card_wait: Duration::ZERO,
            state: None,
            verbosity: Level::Info,
            shadow: false,
            governor: None,
            guard: None,
            horizon: None,
            ctrl_headroom: None,
            unified_caps: false,
            experimental_unified: false,
            allow_unqualified: false,
            record: None,
            record_max_mb: 0,
            replay: None,
            model: None,
            gain_db: 0.0,
            csv: None,
        }
    }

    /// Review X4 of speakerguardd#8: the unit conflicted with
    /// speakersafetyd, so starting it on an M1 Pro, M2 or J616 stopped
    /// speakersafetyd, then refused the unqualified configuration and
    /// exited, and after five tries neither daemon ran.  The packaged unit
    /// never stops another protection daemon, and a refusal exits with a
    /// status the unit does not restart on.
    #[test]
    fn a_refusal_to_govern_leaves_the_other_protection_running() {
        let unit = include_str!("../speakerguardd.service");
        let directive = |name: &str| -> Vec<String> {
            unit.lines().map(str::trim).filter(|l| l.starts_with(&format!("{name}="))).map(|l| l[name.len() + 1..].to_string()).collect()
        };
        assert!(directive("Conflicts").iter().all(|v| !v.contains("speakersafetyd")), "the unit stops speakersafetyd: {:?}", directive("Conflicts"));
        assert!(directive("RestartPreventExitStatus").iter().any(|v| v.split_whitespace().any(|x| x == EXIT_REFUSED.to_string())), "a refusal is restarted");
        // the configurations as built in, in a directory of their own (review
        // Z5 of speakerguardd#8: read from the source tree at run time, a
        // missing file passed the refusal half for the wrong reason)
        let conf = std::env::temp_dir().join(format!("speakerguardd-x4-{}", std::process::id()));
        std::fs::create_dir_all(conf.join("apple")).unwrap();
        for (m, text) in [("j316", include_str!("../conf/apple/j316.conf")), ("j413", include_str!("../conf/apple/j413.conf")), ("j616", include_str!("../conf/apple/j616.conf")), ("j700", include_str!("../conf/apple/j700.conf"))] {
            std::fs::write(conf.join("apple").join(format!("{m}.conf")), text).unwrap();
        }
        for m in ["j316", "j413", "j616"] {
            let path = conf.join("apple").join(format!("{m}.conf"));
            let e = load_config(&path, &args(&conf)).expect_err("an unqualified machine was governed");
            assert_eq!(exit_status(&e), EXIT_REFUSED, "{m}: {e}");
        }
        let missing = conf.join("apple").join("j000.conf");
        assert_eq!(exit_status(&load_config(&missing, &args(&conf)).unwrap_err()), EXIT_REFUSED);
        assert!(load_config(&conf.join("apple").join("j700.conf"), &args(&conf)).is_ok());
        let other: Box<dyn std::error::Error> = "sense capture: EIO".into();
        assert_eq!(exit_status(other.as_ref()), 1);
        let _ = std::fs::remove_dir_all(&conf);
    }
}
