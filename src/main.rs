// SPDX-License-Identifier: MIT
//! speakerguardd - feed-forward speaker protection for amplifiers without
//! sense lines.
//!
//! The card's speaker sense PCM carries the words the amplifier receives.
//! Each period they are turned into the power in every voice coil, run
//! through a two-stage thermal model, and the coil temperature governs the
//! card's speaker volume control.  The card only unlocks that control while
//! this daemon holds the lock on it and keeps writing the unlock control
//! (the same interlock the other Macs' speakersafetyd uses); if the model
//! ever demands more reduction than `--max-reduction`, the daemon exits and
//! the card falls back to its locked, safe volume.

mod alsa;
mod config;
mod model;

use std::path::PathBuf;
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use alsa::{Capture, Ctl, ElemId};
use config::Config;
use model::SpeakerState;

/// The value the card's "Speaker Volume Unlock" control expects: the kernel
/// compares the written long with (s32)0xdec1be15.
const UNLOCK_MAGIC: i64 = 0xdec1be15u32 as i32 as i64;
/// The kernel locks the volume again 250 ms after the last write; write well
/// inside that while the speakers play.
/// Cadence of the per-speaker debug line.
const LOG_INTERVAL: Duration = Duration::from_secs(1);
const PING_INTERVAL: Duration = Duration::from_millis(100);
/// Volume control steps per dB (t8140-aop-audio: 0.5 dB steps).
const STEPS_PER_DB: f64 = 2.0;

#[derive(Clone, Copy, PartialEq, PartialOrd)]
enum Level {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
}

static mut VERBOSITY: Level = Level::Info;

/// Log lines carry the kernel's monotonic clock so they line up with dmesg.
fn log(level: Level, msg: std::fmt::Arguments) {
    // SAFETY: VERBOSITY is written once, before any other thread exists.
    if level <= unsafe { VERBOSITY } {
        let tag = match level {
            Level::Error => "E",
            Level::Warn => "W",
            Level::Info => "I",
            Level::Debug => "D",
        };
        let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: a valid, writable timespec.
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        eprintln!("[{:5}.{:06}] {tag} {msg}", ts.tv_sec, ts.tv_nsec / 1000);
    }
}

macro_rules! error { ($($t:tt)*) => { log(Level::Error, format_args!($($t)*)) } }
macro_rules! warn { ($($t:tt)*) => { log(Level::Warn, format_args!($($t)*)) } }
macro_rules! info { ($($t:tt)*) => { log(Level::Info, format_args!($($t)*)) } }
macro_rules! debug { ($($t:tt)*) => { log(Level::Debug, format_args!($($t)*)) } }

struct Args {
    config_path: PathBuf,
    max_reduction: f64,
    card: Option<String>,
    verbosity: Level,
}

fn usage() -> ! {
    eprintln!(
        "Usage: speakerguardd [OPTIONS]\n\n\
         Options:\n  \
         -c, --config-path <DIR>         Directory holding <vendor>/<model>.conf\n  \
         -C, --card <ID>                 ALSA card id (default: the first Apple* card)\n  \
         -m, --max-reduction <DB>        Gain reduction beyond which the daemon exits (default 20)\n  \
         -v, --verbose                   More logging (repeatable)\n  \
         -q, --quiet                     Less logging\n  \
         -h, --help                      This text"
    );
    std::process::exit(2)
}

fn parse_args() -> Args {
    let mut args = Args { config_path: PathBuf::from("/usr/share/speakerguardd"), max_reduction: 20.0, card: None, verbosity: Level::Info };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "-c" | "--config-path" => args.config_path = PathBuf::from(it.next().unwrap_or_else(|| usage())),
            "-C" | "--card" => args.card = Some(it.next().unwrap_or_else(|| usage())),
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
fn find_card(requested: Option<&str>) -> std::io::Result<(Ctl, String)> {
    if let Some(id) = requested {
        return Ok((Ctl::open_by_id(id)?, id.to_string()));
    }
    for entry in std::fs::read_dir("/proc/asound")? {
        let entry = entry?;
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.starts_with("AppleJ") {
            return Ok((Ctl::open_by_id(&name)?, name));
        }
    }
    Err(std::io::Error::new(std::io::ErrorKind::NotFound, "no Apple sound card"))
}

struct Card {
    ctl: Ctl,
    volume: Vec<ElemId>,
    volume_max: i64,
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
        Ok((Card { ctl, volume, volume_max: 0, unlock, sample_rate }, name))
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

    fn set_reduction(&self, db: f64) -> std::io::Result<()> {
        let value = (self.volume_max as f64 - db * STEPS_PER_DB).round().max(0.0) as i64;
        for id in &self.volume {
            self.ctl.write_int(id, value)?;
        }
        Ok(())
    }
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let (card, card_name) = {
        // the model file is named after the card: AppleJ700 -> apple/j700.conf
        let (ctl, name) = find_card(args.card.as_deref())?;
        drop(ctl);
        let model = name.strip_prefix("Apple").unwrap_or(&name).to_lowercase();
        let path = args.config_path.join("apple").join(format!("{model}.conf"));
        let cfg = Config::load(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        info!("{}: {} speakers, sense PCM {}, period {} frames", path.display(), cfg.speakers.len(), cfg.globals.sense_pcm, cfg.globals.period);
        let (card, name) = Card::open(Some(&name), &cfg)?;
        ((card, cfg), name)
    };
    let (mut card, cfg) = card;
    let globals = cfg.globals.clone();
    let mut speakers: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), globals.t_ambient)).collect();

    card.take_lock()?;
    card.set_reduction(0.0)?;
    info!("{card_name}: volume lock taken, {} volume control(s), max {}", card.volume.len(), card.volume_max);

    let mut capture: Option<Capture> = None;
    let mut words = vec![0i32; globals.period * globals.channels];
    let mut last_ping = Instant::now();
    let mut last_log = Instant::now();
    let mut applied = 0.0f64;
    loop {
        // playback state: the card reports the speakers' sample rate, 0 when closed
        let rate = card.ctl.read_int(&card.sample_rate)? as u32;
        if rate == 0 {
            if capture.take().is_some() {
                info!("speakers closed");
            }
            // cool down with no power, keep the lock alive
            for s in speakers.iter_mut() {
                s.step(0.0, PING_INTERVAL.as_secs_f64(), globals.t_ambient);
                s.govern(&globals);
            }
            card.set_reduction(model::reduction_db(&speakers))?;
            card.ping()?;
            last_ping = Instant::now();
            thread::sleep(PING_INTERVAL);
            continue;
        }
        if capture.is_none() {
            match Capture::open(card.ctl.card, globals.sense_pcm, globals.channels, rate, globals.period) {
                Ok(c) => {
                    info!("speakers open at {rate} Hz, sense capture running");
                    capture = Some(c);
                }
                Err(e) => {
                    warn!("sense PCM: {e}; retrying");
                    card.ping()?;
                    thread::sleep(PING_INTERVAL);
                    continue;
                }
            }
        }
        let cap = capture.as_mut().unwrap();
        let dt = globals.period as f64 / rate as f64;
        let valid = cap.read_period(&mut words)?;
        if !valid {
            warn!("sense overrun, step skipped");
        } else {
            for s in speakers.iter_mut() {
                let v = s.v_rms_of(&words, globals.channels);
                s.v_rms = v;
                let power = v * v / s.spec.z_nominal;
                s.step(power, dt, globals.t_ambient);
                s.govern(&globals);
            }
        }
        let reduction = model::reduction_db(&speakers);
        if reduction >= args.max_reduction {
            error!("gain reduction {reduction:.1} dB reached the limit; leaving the card locked");
            return Err("maximum reduction reached".into());
        }
        if (reduction - applied).abs() >= 0.5 {
            info!("gain reduction {reduction:.1} dB");
            applied = reduction;
        }
        card.set_reduction(reduction)?;
        if last_ping.elapsed() >= PING_INTERVAL {
            match card.ping() {
                Ok(()) => {}
                Err(e) if e.raw_os_error() == Some(libc::ETIMEDOUT) => {
                    warn!("the card locked the volume while we were away; continuing");
                }
                Err(e) => return Err(e.into()),
            }
            last_ping = Instant::now();
        }
        if last_log.elapsed() >= LOG_INTERVAL {
            for s in &speakers {
                debug!("{}: {:.3} Vrms {:.1} mW coil {:.1} C magnet {:.1} C reduction {:.1} dB", s.spec.name, s.v_rms, s.power * 1000.0, s.t_coil, s.t_magnet, s.reduction_db);
            }
            last_log = Instant::now();
        }
    }
}

fn main() -> ExitCode {
    let args = parse_args();
    // SAFETY: single-threaded at this point.
    unsafe { VERBOSITY = args.verbosity };
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            error!("{e}");
            ExitCode::FAILURE
        }
    }
}
