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
//! reaches `--max-reduction`, the daemon releases its lease while cooling.
//! Missing observations or invalid state cannot renew the lease.

mod alsa;
mod config;
mod model;

use std::path::PathBuf;
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use alsa::{Capture, Ctl, ElemId, SENSE_DEADLINE};
use config::Config;
use model::SpeakerState;

/// The value the card's "Speaker Volume Unlock" control expects: the kernel
/// compares the written long with (s32)0xdec1be15.
const UNLOCK_MAGIC: i64 = 0xdec1be15u32 as i32 as i64;
/// Cadence of the per-speaker debug line.
const LOG_INTERVAL: Duration = Duration::from_secs(1);
const PING_INTERVAL: Duration = Duration::from_millis(100);
/// Volume control steps per dB (t8140-aop-audio: 0.5 dB steps).
const STEPS_PER_DB: f64 = 2.0;
/// The J700 kernel's fallback attenuation when no lease is held.
const FALLBACK_REDUCTION_DB: f64 = 20.0;

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
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        // SAFETY: a valid, writable timespec.
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        eprintln!("[{:5}.{:06}] {tag} {msg}", ts.tv_sec, ts.tv_nsec / 1000);
    }
}

macro_rules! error { ($($t:tt)*) => { log(Level::Error, format_args!($($t)*)) } }
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
         -m, --max-reduction <DB>        Reduction that releases the lease (0 < dB <= 20)\n  \
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
        verbosity: Level::Info,
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "-c" | "--config-path" => {
                args.config_path = PathBuf::from(it.next().unwrap_or_else(|| usage()))
            }
            "-C" | "--card" => args.card = Some(it.next().unwrap_or_else(|| usage())),
            "-m" | "--max-reduction" => {
                args.max_reduction = it
                    .next()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or_else(|| usage());
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
    Err(std::io::Error::new(
        std::io::ErrorKind::NotFound,
        "no Apple sound card",
    ))
}

struct Card {
    ctl: Ctl,
    volume: Vec<ElemId>,
    volume_max: i64,
    unlock: ElemId,
    sample_rate: ElemId,
    locked: bool,
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
        Ok((
            Card {
                ctl,
                volume,
                volume_max: 0,
                unlock,
                sample_rate,
                locked: false,
            },
            name,
        ))
    }

    /// Take the interlock: every volume control first, then the unlock
    /// control, then the first ping -- which is when the card lifts the
    /// volume limit, so the control's range is read after it.
    fn take_lock(&mut self) -> std::io::Result<()> {
        if self.locked {
            return Ok(());
        }
        for id in &self.volume {
            self.ctl.lock(id)?;
        }
        self.ctl.lock(&self.unlock)?;
        self.ping()?;
        let (_, max) = self.ctl.int_range(&self.volume[0])?;
        self.volume_max = max;
        self.locked = true;
        Ok(())
    }

    fn release_lock(&mut self) -> std::io::Result<()> {
        if self.locked {
            // Revoke protection before releasing the child controls.
            self.ctl.unlock(&self.unlock)?;
            for id in &self.volume {
                self.ctl.unlock(id)?;
            }
            self.locked = false;
        }
        Ok(())
    }

    fn ping(&self) -> std::io::Result<()> {
        self.ctl.write_int(&self.unlock, UNLOCK_MAGIC)
    }

    fn set_reduction(&self, db: f64) -> std::io::Result<()> {
        if !db.is_finite() || db < 0.0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "invalid model reduction",
            ));
        }
        let value = (self.volume_max as f64 - db * STEPS_PER_DB)
            .floor()
            .max(0.0) as i64;
        for id in &self.volume {
            self.ctl.write_int(id, value)?;
        }
        Ok(())
    }
}

/// A lost lease means protection was interrupted. Do not silently re-arm it.
fn ping_lock(card: &Card, last_ping: &mut Instant) -> Result<(), Box<dyn std::error::Error>> {
    card.ping().map_err(|e| format!("lock ping: {e}"))?;
    *last_ping = Instant::now();
    Ok(())
}

fn may_grant_lease(
    reduction: f64,
    limit: f64,
    observation_age: Duration,
) -> Result<bool, &'static str> {
    if !reduction.is_finite() || reduction < 0.0 {
        return Err("invalid model reduction");
    }
    if observation_age >= SENSE_DEADLINE {
        return Err("speaker observation expired before lease renewal");
    }
    Ok(reduction < limit)
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let (card, _card_name) = {
        // the model file is named after the card: AppleJ700 -> apple/j700.conf
        let (ctl, name) = find_card(args.card.as_deref())?;
        drop(ctl);
        let model = name.strip_prefix("Apple").unwrap_or(&name).to_lowercase();
        let path = args.config_path.join("apple").join(format!("{model}.conf"));
        let cfg = Config::load(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        info!(
            "{}: {} speakers, sense PCM {}, period {} frames",
            path.display(),
            cfg.speakers.len(),
            cfg.globals.sense_pcm,
            cfg.globals.period
        );
        let (card, name) = Card::open(Some(&name), &cfg)?;
        ((card, cfg), name)
    };
    let (mut card, cfg) = card;
    let globals = cfg.globals.clone();
    if !args.max_reduction.is_finite()
        || args.max_reduction <= 0.0
        || args.max_reduction > FALLBACK_REDUCTION_DB
    {
        return Err("max reduction must be finite and within (0, 20] dB".into());
    }
    // A process restart says nothing about the physical speaker temperature.
    // Start at the configured hard limit, and retain the fallback until the
    // conservative model cools enough to permit less attenuation.
    let mut speakers: Vec<SpeakerState> = cfg
        .speakers
        .iter()
        .map(|s| SpeakerState::new(s.clone(), s.t_limit))
        .collect();
    let mut capture: Option<Capture> = None;
    let mut capture_rate = 0;
    let mut words = vec![0i32; globals.period * globals.channels];
    let mut last_ping = Instant::now();
    let mut last_log = Instant::now();
    let mut last_model = Instant::now();
    let mut last_sense: Option<Instant> = None;
    let mut applied = FALLBACK_REDUCTION_DB;
    loop {
        let rate_value = card.ctl.read_int(&card.sample_rate)?;
        if !(0..=192000).contains(&rate_value) {
            return Err("invalid speaker sample rate".into());
        }
        let rate = rate_value as u32;
        if rate == 0 {
            card.release_lock()?;
            capture = None;
            last_sense = None;
            applied = FALLBACK_REDUCTION_DB;
            let now = Instant::now();
            let elapsed = now.duration_since(last_model).as_secs_f64();
            for s in &mut speakers {
                s.step(0.0, elapsed, globals.t_ambient);
                s.govern(&globals);
            }
            last_model = now;
            thread::sleep(PING_INTERVAL);
            continue;
        }
        let dt = globals.period as f64 / rate as f64;
        if dt <= 0.0 || dt > 0.15 {
            return Err("sense period must fit inside the protection deadline".into());
        }
        if capture.is_none() {
            // Failure leaves the kernel interlock closed; never ping while
            // waiting for a missing, busy or broken observation stream.
            capture = Some(Capture::open(
                card.ctl.card,
                globals.sense_pcm,
                globals.channels,
                rate,
                globals.period,
            )?);
            capture_rate = rate;
        } else if capture_rate != rate {
            return Err("speaker rate changed without closing sense capture".into());
        }
        capture.as_mut().unwrap().read_period(&mut words)?;
        let now = Instant::now();
        if last_sense.is_some_and(|previous| now.duration_since(previous) >= SENSE_DEADLINE) {
            return Err("speaker observation interval exceeded the protection deadline".into());
        }
        last_sense = Some(now);
        last_model = now;
        for s in &mut speakers {
            let v = s.v_rms_of(&words, globals.channels);
            let power = v * v / s.spec.z_nominal;
            if !power.is_finite() {
                return Err("nonfinite speaker power estimate".into());
            }
            s.v_rms = v;
            s.step(power, dt, globals.t_ambient);
            // The shared control applies the strongest channel's reduction.
            s.reduction_db = applied;
            s.govern(&globals);
            if !s.t_coil.is_finite() || !s.t_magnet.is_finite() || !s.reduction_db.is_finite() {
                return Err("nonfinite speaker model state".into());
            }
        }
        let reduction = model::reduction_db(&speakers);
        if !may_grant_lease(reduction, args.max_reduction, now.elapsed())? {
            card.release_lock()?;
            applied = FALLBACK_REDUCTION_DB;
        } else {
            // Only a complete, timely, valid observation may grant/renew the
            // lease. No I/O error or overrun path reaches this point.
            card.take_lock()?;
            card.set_reduction(reduction)?;
            applied = reduction;
            ping_lock(&card, &mut last_ping)?;
        }
        if last_log.elapsed() >= LOG_INTERVAL {
            for s in &speakers {
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

#[cfg(test)]
mod safety_tests {
    use super::*;

    #[test]
    fn lease_requires_finite_model_and_fresh_observation() {
        assert_eq!(
            may_grant_lease(2.0, 20.0, Duration::from_millis(85)),
            Ok(true)
        );
        assert_eq!(may_grant_lease(20.0, 20.0, Duration::ZERO), Ok(false));
        for reduction in [f64::NAN, f64::INFINITY, -1.0] {
            assert!(may_grant_lease(reduction, 20.0, Duration::ZERO).is_err());
        }
        assert!(may_grant_lease(0.0, 20.0, SENSE_DEADLINE).is_err());
    }

    #[test]
    fn restart_does_not_assume_a_cold_speaker() {
        let cfg = Config::parse(include_str!("../conf/apple/j700.conf")).unwrap();
        let spec = cfg.speakers[0].clone();
        let mut state = SpeakerState::new(spec.clone(), spec.t_limit);
        let reduction = state.govern(&cfg.globals);
        assert!(reduction >= FALLBACK_REDUCTION_DB);
        assert_eq!(may_grant_lease(reduction, 20.0, Duration::ZERO), Ok(false));
    }
}
