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
mod log;
mod model;
mod record;
mod startup;
mod state;

use std::path::PathBuf;
use std::process::ExitCode;
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

struct Args {
    config_path: PathBuf,
    max_reduction: f64,
    card: Option<String>,
    card_wait: Duration,
    state: Option<PathBuf>,
    verbosity: Level,
    shadow: bool,
    governor: Option<Governor>,
    record: Option<PathBuf>,
    record_max_mb: u64,
    replay: Option<PathBuf>,
    model: Option<String>,
    gain_db: f64,
    csv: Option<PathBuf>,
    horizon: Option<f64>,
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
         --governor <NAME>               legacy, asahi or unified (default: the config's, else legacy)\n  \
         --shadow                        Run the other governors too and log them to shadow.jsonl next to the state\n  \
         --record <FILE>                 Keep the raw sense periods (and the applied gains) for --replay\n  \
         --record-max-mb <MB>            Stop recording past this size (default 512)\n  \
         --replay <FILE>                 Run every governor on a recording offline (no card), print a summary\n  \
         --model <NAME>                  With --replay: the config (apple/<NAME>.conf)\n  \
         --gain-db <DB>                  With --replay: play the recording this much louder\n  \
         --csv <FILE>                    With --replay: one line per period\n  \
         --horizon <S>                   The unified governor's prediction horizon (default: the config's)\n  \
         -v, --verbose                   More logging (repeatable)\n  \
         -q, --quiet                     Less logging\n  \
         -h, --help                      This text"
    );
    std::process::exit(2)
}

fn parse_args() -> Args {
    let mut args = Args {
        config_path: PathBuf::from("/usr/share/speakerguardd"),
        max_reduction: 20.0,
        card: None,
        card_wait: Duration::from_secs(30),
        state: Some(PathBuf::from("/run/speakerguardd/state")),
        verbosity: Level::Info,
        shadow: false,
        governor: None,
        record: None,
        record_max_mb: 512,
        replay: None,
        model: None,
        gain_db: 0.0,
        csv: None,
        horizon: None,
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
                args.max_reduction = it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| usage());
            }
            "--shadow" => args.shadow = true,
            "--governor" => {
                args.governor = Some(match it.next().as_deref() {
                    Some("legacy") => Governor::Legacy,
                    Some("asahi") => Governor::Asahi,
                    Some("unified") => Governor::Unified,
                    _ => usage(),
                })
            }
            "--record" => args.record = Some(PathBuf::from(it.next().unwrap_or_else(|| usage()))),
            "--record-max-mb" => args.record_max_mb = it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| usage()),
            "--replay" => args.replay = Some(PathBuf::from(it.next().unwrap_or_else(|| usage()))),
            "--model" => args.model = Some(it.next().unwrap_or_else(|| usage())),
            "--gain-db" => args.gain_db = it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| usage()),
            "--csv" => args.csv = Some(PathBuf::from(it.next().unwrap_or_else(|| usage()))),
            "--horizon" => args.horizon = Some(it.next().and_then(|v| v.parse().ok()).filter(|h: &f64| *h > 0.0).unwrap_or_else(|| usage())),
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
    volume_max: i64,
    /// One volume control per speaker ("<speaker> <volume>"), in speaker
    /// order, on cards that have them (V/I machines).
    per_speaker: Option<Vec<ElemId>>,
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
        let named: Vec<Option<ElemId>> = cfg
            .speakers
            .iter()
            .map(|s| ctl.find_one(&format!("{} {}", s.name, cfg.controls.volume)).ok())
            .collect();
        let per_speaker = if named.iter().all(Option::is_some) { Some(named.into_iter().flatten().collect()) } else { None };
        // V/I machines: sense on, amplifier at the gain the model assumes
        // (speakersafetyd does the same; the kernel clamps the range)
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
                let (_, max) = ctl.int_range(&id)?;
                ctl.write_int(&id, max)?;
                info!("{}: {gain} at its maximum ({max}), {:.1} dBV assumed", s.name, cfg.globals.amp_gain_dbv);
            }
        }
        Ok((Card { ctl, volume, volume_max: 0, per_speaker, unlock, sample_rate }, name))
    }

    /// Take the interlock: every volume control first, then the unlock
    /// control, then the first ping -- which is when the card lifts the
    /// volume limit, so the control's range is read after it.
    fn take_lock(&mut self) -> std::io::Result<()> {
        for id in &self.volume {
            self.ctl.lock(id)?;
        }
        self.ctl.lock(&self.unlock)?;
        self.ping()?;
        let (_, max) = self.ctl.int_range(&self.volume[0])?;
        self.volume_max = max;
        Ok(())
    }

    fn ping(&self) -> std::io::Result<()> {
        self.ctl.write_int(&self.unlock, UNLOCK_MAGIC)
    }

    /// The control value for a reduction, rounded up to the control's
    /// step: the card never applies less than the model asked for.
    fn value_of(&self, db: f64) -> i64 {
        (self.volume_max as f64 - (db * STEPS_PER_DB - 1e-9).ceil()).max(0.0) as i64
    }

    fn write_reduction(&self, db: f64) -> std::io::Result<()> {
        let value = self.value_of(db);
        for id in &self.volume {
            self.ctl.write_int(id, value)?;
        }
        Ok(())
    }

    fn write_reductions(&self, db: &[f64]) -> std::io::Result<()> {
        match &self.per_speaker {
            Some(ids) => {
                for (id, &d) in ids.iter().zip(db) {
                    self.ctl.write_int(id, self.value_of(d))?;
                }
                Ok(())
            }
            None => self.write_reduction(db.iter().cloned().fold(0.0, f64::max)),
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
    fn set_reduction(&mut self, db: f64) -> std::io::Result<()> {
        self.card.write_reduction(db)
    }
    fn set_reductions(&mut self, db: &[f64]) -> std::io::Result<()> {
        self.card.write_reductions(db)
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
    fn read_period(&mut self, words: &mut [i32]) -> std::io::Result<Option<bool>> {
        match self.capture.as_mut() {
            Some(c) => c.read_period(words),
            None => Err(std::io::Error::from_raw_os_error(libc::EBADFD)),
        }
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
    let mut cfg = Config::load(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    override_config(&mut cfg, args)?;
    info!("{}: {} speakers, sense PCM {}, period {} frames", path.display(), cfg.speakers.len(), cfg.globals.sense_pcm, cfg.globals.period);
    let (card, card_name) = Card::open(Some(&name), &cfg)?;
    let mut hw = RealHw { card, globals: cfg.globals.clone(), capture: None };
    let mut guard = Guard::start(&mut hw, &cfg, args.max_reduction, args.state.clone())?;
    if let Some(path) = &args.record {
        guard.record = Some(record::Recorder::create(path, &cfg, args.record_max_mb << 20)?);
        info!("recording sense periods to {}", path.display());
    }
    info!("{card_name}: volume lock taken, {} volume control(s), max {}", hw.card.volume.len(), hw.card.volume_max);
    loop {
        if let Err(e) = guard.tick(&mut hw) {
            // the next run starts from where this model was
            guard.save(&hw);
            return Err(e);
        }
    }
}

/// Command-line governor and shadow choices over the config's.
fn override_config(cfg: &mut Config, args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(g) = args.governor {
        cfg.globals.governor = g;
    }
    cfg.globals.shadow |= args.shadow;
    if let Some(h) = args.horizon {
        cfg.globals.horizon_s = h;
    }
    cfg.check_governor()?;
    Ok(())
}

fn main() -> ExitCode {
    let args = parse_args();
    log::set_verbosity(args.verbosity);
    if let Some(path) = &args.replay {
        let result = (|| -> Result<(), Box<dyn std::error::Error>> {
            let model = args.model.as_deref().ok_or("--replay needs --model")?;
            let conf = args.config_path.join("apple").join(format!("{model}.conf"));
            let mut cfg = Config::load(&conf).map_err(|e| format!("{}: {e}", conf.display()))?;
            override_config(&mut cfg, &args)?;
            let summary = record::replay(path, &cfg, args.gain_db, args.csv.as_deref())?;
            print!("{summary}");
            Ok(())
        })();
        return match result {
            Ok(()) => ExitCode::SUCCESS,
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
            ExitCode::FAILURE
        }
    }
}
