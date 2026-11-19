// SPDX-License-Identifier: MIT
//! The per-machine configuration: an INI file with a `[Globals]` section, a
//! `[Controls]` section naming the card's controls, and one `[Speaker/<name>]`
//! section per driver, keyed like speakersafetyd's so the numbers read the
//! same way (thermal resistances in K/W, time constants in seconds,
//! temperatures in °C, impedance in Ω).

use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::path::Path;

#[derive(Debug)]
pub struct ConfigError(pub String);

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ConfigError {}

#[derive(Debug, Clone, PartialEq)]
pub struct Globals {
    /// The card's speaker sense PCM device number.
    pub sense_pcm: u32,
    /// Ambient temperature assumed at start (°C).
    pub t_ambient: f64,
    /// Gain is restored only once the coil cooled this far below the point
    /// where reduction began (K).
    pub t_hysteresis: f64,
    /// Reduction starts this far below the coil limit (K).
    pub t_window: f64,
    /// Channels of the sense PCM.
    pub channels: usize,
    /// Frames per model step.
    pub period: usize,
    /// Reduction the thermal governor reaches at the coil's working limit
    /// (dB); more than this means the model, not the music, is wrong.
    pub t_reduction_max: f64,
    /// The card's own level without a daemon, dB below the volume
    /// control's maximum (macaudio's safe_vol): what the speakers get once
    /// the lock lapses.
    pub safe_reduction: f64,
    /// The most frames the speaker playback path can hold already converted
    /// through the volume control (the speaker front-end's buffer limit):
    /// what the card converted can play at most this long after it, at the
    /// stream's rate.  Required: it bounds every gain change and handoff.
    pub copy_queue_frames: u32,
    /// How long samples converted at an earlier gain can keep playing after
    /// a gain change, and after the lock is given up until they are
    /// certainly gone (the card's stop work, then the queue), with margin
    /// (s).  Required, and at least the queue at `default_rate`.
    pub drain: f64,
    /// The rate the queue and the period are timed at before the first
    /// stream shows its own (Hz).  Required.
    pub default_rate: u32,
}

impl Globals {
    /// `drain` as a duration (whole milliseconds, as the guard counts it).
    pub fn drain_duration(&self) -> std::time::Duration {
        std::time::Duration::from_millis((self.drain * 1000.0) as u64)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Controls {
    pub volume: String,
    pub unlock: String,
    pub sample_rate: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Speaker {
    pub name: String,
    pub group: u32,
    /// Thermal resistance voice coil -> magnet (K/W) and its time constant (s).
    pub tr_coil: f64,
    pub tau_coil: f64,
    /// Thermal resistance magnet -> ambient (K/W) and its time constant (s).
    pub tr_magnet: f64,
    pub tau_magnet: f64,
    /// Maximum coil temperature (°C) and the margin kept below it.
    pub t_limit: f64,
    pub t_headroom: f64,
    /// Nominal coil impedance (Ω) the power is computed against.
    pub z_nominal: f64,
    /// Volts at the amplifier output for a full-scale sense word.
    pub vs_scale: f64,
    /// Sense PCM channel carrying this speaker.
    pub vs_chan: usize,
    /// Power budget: the mean power over the last second and the last
    /// minute may not exceed these (W); 0 disables a budget.
    pub p_limit_1s: f64,
    pub p_limit_60s: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    pub globals: Globals,
    pub controls: Controls,
    pub speakers: Vec<Speaker>,
}

type Sections = BTreeMap<String, BTreeMap<String, String>>;

fn parse_ini(text: &str) -> Result<Sections, ConfigError> {
    let mut sections: Sections = BTreeMap::new();
    let mut current: Option<String> = None;
    for (n, raw) in text.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with(';') {
            continue;
        }
        if let Some(name) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            current = Some(name.trim().to_string());
            sections.entry(name.trim().to_string()).or_default();
            continue;
        }
        let (key, value) = line
            .split_once('=')
            .ok_or_else(|| ConfigError(format!("line {}: expected key = value", n + 1)))?;
        let section = current
            .as_ref()
            .ok_or_else(|| ConfigError(format!("line {}: key outside a section", n + 1)))?;
        sections
            .get_mut(section)
            .unwrap()
            .insert(key.trim().to_string(), value.trim().to_string());
    }
    Ok(sections)
}

fn get<'a>(section: &'a BTreeMap<String, String>, name: &str, key: &str) -> Result<&'a str, ConfigError> {
    section
        .get(key)
        .map(String::as_str)
        .ok_or_else(|| ConfigError(format!("[{name}] is missing {key}")))
}

/// A numeric key that may be absent (0 when it is).
fn optional(values: &BTreeMap<String, String>, section: &str, key: &str) -> Result<f64, ConfigError> {
    match values.get(key) {
        None => Ok(0.0),
        Some(_) => num(values, section, key),
    }
}

fn num<T: std::str::FromStr>(section: &BTreeMap<String, String>, name: &str, key: &str) -> Result<T, ConfigError> {
    get(section, name, key)?
        .parse()
        .map_err(|_| ConfigError(format!("[{name}] {key}: not a number")))
}

impl Config {
    pub fn parse(text: &str) -> Result<Config, ConfigError> {
        let sections = parse_ini(text)?;
        let g = sections.get("Globals").ok_or_else(|| ConfigError("missing [Globals]".into()))?;
        let c = sections.get("Controls").ok_or_else(|| ConfigError("missing [Controls]".into()))?;
        let globals = Globals {
            sense_pcm: num(g, "Globals", "sense_pcm")?,
            t_ambient: num(g, "Globals", "t_ambient")?,
            t_hysteresis: num(g, "Globals", "t_hysteresis")?,
            t_window: num(g, "Globals", "t_window")?,
            channels: num(g, "Globals", "channels")?,
            period: num(g, "Globals", "period")?,
            t_reduction_max: match g.get("t_reduction_max") {
                None => 12.0,
                Some(_) => num(g, "Globals", "t_reduction_max")?,
            },
            safe_reduction: match g.get("safe_reduction") {
                None => 20.0,
                Some(_) => num(g, "Globals", "safe_reduction")?,
            },
            // the queue timing has no default: it is the card's, and a
            // profile without it is refused
            copy_queue_frames: num(g, "Globals", "copy_queue_frames")?,
            drain: num(g, "Globals", "drain")?,
            default_rate: num(g, "Globals", "default_rate")?,
        };
        for (key, value) in [
            ("t_ambient", globals.t_ambient),
            ("t_hysteresis", globals.t_hysteresis),
            ("t_window", globals.t_window),
            ("t_reduction_max", globals.t_reduction_max),
            ("safe_reduction", globals.safe_reduction),
            ("drain", globals.drain),
        ] {
            if !value.is_finite() {
                return Err(ConfigError(format!("[Globals] {key} must be finite")));
            }
        }
        if globals.t_window <= 0.0 || globals.t_hysteresis < 0.0 {
            return Err(ConfigError(
                "[Globals] window must be positive and hysteresis nonnegative".into(),
            ));
        }
        if globals.period.checked_mul(globals.channels).is_none() {
            return Err(ConfigError(
                "[Globals] period/channel count overflow".into(),
            ));
        }
        let controls = Controls {
            volume: get(c, "Controls", "volume")?.to_string(),
            unlock: get(c, "Controls", "unlock")?.to_string(),
            sample_rate: get(c, "Controls", "sample_rate")?.to_string(),
        };
        let mut speakers = Vec::new();
        for (section, values) in &sections {
            let Some(name) = section.strip_prefix("Speaker/") else { continue };
            let s = |key| num::<f64>(values, section, key);
            let speaker = Speaker {
                name: name.to_string(),
                group: num(values, section, "group")?,
                tr_coil: s("tr_coil")?,
                tau_coil: s("tau_coil")?,
                tr_magnet: s("tr_magnet")?,
                tau_magnet: s("tau_magnet")?,
                t_limit: s("t_limit")?,
                t_headroom: s("t_headroom")?,
                z_nominal: s("z_nominal")?,
                vs_scale: s("vs_scale")?,
                vs_chan: num(values, section, "vs_chan")?,
                p_limit_1s: optional(values, section, "p_limit_1s")?,
                p_limit_60s: optional(values, section, "p_limit_60s")?,
            };
            if speaker.vs_chan >= globals.channels {
                return Err(ConfigError(format!("[{section}] vs_chan {} beyond {} channels", speaker.vs_chan, globals.channels)));
            }
            for (key, value) in [
                ("tr_coil", speaker.tr_coil),
                ("tau_coil", speaker.tau_coil),
                ("tr_magnet", speaker.tr_magnet),
                ("tau_magnet", speaker.tau_magnet),
                ("z_nominal", speaker.z_nominal),
                ("vs_scale", speaker.vs_scale),
            ] {
                if !value.is_finite() || !(value > 0.0) {
                    return Err(ConfigError(format!("[{section}] {key} must be positive")));
                }
            }
            for (key, value) in [
                ("t_limit", speaker.t_limit),
                ("t_headroom", speaker.t_headroom),
                ("p_limit_1s", speaker.p_limit_1s),
                ("p_limit_60s", speaker.p_limit_60s),
            ] {
                if !value.is_finite() {
                    return Err(ConfigError(format!("[{section}] {key} must be finite")));
                }
            }
            if speaker.t_headroom < 0.0 {
                return Err(ConfigError(format!(
                    "[{section}] headroom cannot be negative"
                )));
            }
            if !(speaker.t_limit - speaker.t_headroom).is_finite()
                || !(globals.t_ambient + globals.t_window).is_finite()
                || speaker.t_limit - speaker.t_headroom <= globals.t_ambient + globals.t_window {
                return Err(ConfigError(format!("[{section}] no room between ambient and the limit")));
            }
            if speaker.p_limit_1s < 0.0 || speaker.p_limit_60s < 0.0 {
                return Err(ConfigError(format!("[{section}] power limits cannot be negative")));
            }
            speakers.push(speaker);
        }
        if speakers.is_empty() {
            return Err(ConfigError("no [Speaker/...] section".into()));
        }
        if globals.period == 0 || globals.channels == 0 {
            return Err(ConfigError("[Globals] period and channels must be positive".into()));
        }
        if !(globals.t_reduction_max > 0.0) {
            return Err(ConfigError("[Globals] t_reduction_max must be positive".into()));
        }
        if !(globals.safe_reduction > 0.0) {
            return Err(ConfigError("[Globals] safe_reduction must be positive".into()));
        }
        if globals.copy_queue_frames == 0 || globals.default_rate == 0 {
            return Err(ConfigError("[Globals] copy_queue_frames and default_rate must be positive".into()));
        }
        // the drain covers at least the whole queue at the default rate
        let queue_s = f64::from(globals.copy_queue_frames) / f64::from(globals.default_rate);
        if !(globals.drain >= queue_s) {
            return Err(ConfigError(format!(
                "[Globals] drain {} s is shorter than the copy queue ({} frames at {} Hz, {queue_s:.3} s)",
                globals.drain, globals.copy_queue_frames, globals.default_rate
            )));
        }
        Ok(Config { globals, controls, speakers })
    }

    pub fn load(path: &Path) -> Result<Config, Box<dyn std::error::Error>> {
        let text = fs::read_to_string(path)?;
        Ok(Config::parse(&text)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const J700: &str = include_str!("../conf/apple/j700.conf");

    #[test]
    fn parses_j700() {
        let c = Config::parse(J700).unwrap();
        assert_eq!(c.globals.sense_pcm, 2);
        assert_eq!(c.globals.period, 4096);
        assert_eq!(c.controls.volume, "Speaker Playback Volume");
        assert_eq!(c.speakers.len(), 2);
        assert_eq!(c.speakers[0].name, "Left");
        assert_eq!(c.speakers[1].vs_chan, 1);
        assert!((c.speakers[0].z_nominal - 4.2).abs() < 1e-9);
        assert!((c.speakers[1].vs_scale - 7.33).abs() < 1e-9);
        assert!((c.speakers[0].p_limit_1s - 2.8154).abs() < 1e-9);
        assert!((c.speakers[0].p_limit_60s - 2.5).abs() < 1e-9);
        assert!((c.globals.t_reduction_max - 12.0).abs() < 1e-9);
    }

    /// The J700's queue timing is the daemon's former constants exactly
    /// (COPY_QUEUE_FRAMES 16384, QUEUE_BOUND = DRAIN = 500 ms from
    /// DRAIN_S 0.5, DEFAULT_RATE 48000), so nothing changes there.
    #[test]
    fn j700_queue_timing_is_the_former_constants() {
        let g = Config::parse(J700).unwrap().globals;
        assert_eq!(g.copy_queue_frames, 16384);
        assert_eq!(g.drain.to_bits(), 0.5f64.to_bits());
        assert_eq!(g.default_rate, 48000);
        assert_eq!(g.drain_duration(), std::time::Duration::from_millis((0.5f64 * 1000.0) as u64));
        assert_eq!(g.drain_duration(), std::time::Duration::from_millis(500));
    }

    /// The queue timing is the card's: a profile without any one of the
    /// keys is refused, and so is one whose drain is shorter than its queue.
    #[test]
    fn queue_timing_is_required() {
        for line in ["copy_queue_frames = 16384\n", "drain = 0.5\n", "default_rate = 48000\n"] {
            assert!(J700.contains(line), "{line}");
            let key = line.split(' ').next().unwrap();
            let err = Config::parse(&J700.replace(line, "")).unwrap_err().to_string();
            assert_eq!(err, format!("[Globals] is missing {key}"));
        }
        for (from, to) in [
            ("copy_queue_frames = 16384", "copy_queue_frames = 0"),
            ("copy_queue_frames = 16384", "copy_queue_frames = -1"),
            ("default_rate = 48000", "default_rate = 0"),
            ("drain = 0.5", "drain = nan"),
            ("drain = 0.5", "drain = inf"),
            ("drain = 0.5", "drain = -0.5"),
            // 16384 frames at 48 kHz are 0.341 s
            ("drain = 0.5", "drain = 0.34"),
            // a deeper queue than the drain covers
            ("copy_queue_frames = 16384", "copy_queue_frames = 32768"),
            ("default_rate = 48000", "default_rate = 22050"),
        ] {
            assert!(Config::parse(&J700.replace(from, to)).is_err(), "{to}");
        }
        assert!(Config::parse(&J700.replace("drain = 0.5", "drain = 0.35")).is_ok());
    }

    #[test]
    fn power_budget_is_optional() {
        let none = J700.replace("p_limit_1s = 2.8154\n", "").replace("p_limit_60s = 2.5\n", "");
        let c = Config::parse(&none).unwrap();
        assert_eq!(c.speakers[0].p_limit_1s, 0.0);
        assert!(Config::parse(&J700.replace("p_limit_60s = 2.5", "p_limit_60s = -1")).is_err());
    }

    #[test]
    fn rejects_bad_files() {
        assert!(Config::parse("").is_err());
        assert!(Config::parse("[Globals]\nsense_pcm = x\n").is_err());
        let broken = J700.replace("z_nominal = 4.2", "z_nominal = 0");
        assert!(Config::parse(&broken).is_err());
        let hot = J700.replace("t_limit = 135.0", "t_limit = 40.0");
        assert!(Config::parse(&hot).is_err());
    }
    #[test]
    fn rejects_nonfinite_and_invalid_scalar_domains() {
        for key in [
            "t_ambient",
            "t_hysteresis",
            "t_window",
            "t_reduction_max",
            "safe_reduction",
            "tr_coil",
            "tau_coil",
            "tr_magnet",
            "tau_magnet",
            "t_limit",
            "t_headroom",
            "z_nominal",
            "vs_scale",
            "p_limit_1s",
            "p_limit_60s",
        ] {
            for value in ["NaN", "inf", "-inf"] {
                let text = J700
                    .lines()
                    .map(|line| {
                        if line.split('=').next().unwrap().trim() == key {
                            format!("{key} = {value}")
                        } else {
                            line.to_string()
                        }
                    })
                    .collect::<Vec<_>>()
                    .join("\n");
                assert!(Config::parse(&text).is_err(), "{key}={value}");
            }
        }
        for (key, value) in [
            ("t_window", "0"),
            ("t_hysteresis", "-1"),
            ("t_headroom", "-1"),
        ] {
            let text = J700
                .lines()
                .map(|line| {
                    if line.split('=').next().unwrap().trim() == key {
                        format!("{key} = {value}")
                    } else {
                        line.to_string()
                    }
                })
                .collect::<Vec<_>>()
                .join("\n");
            assert!(Config::parse(&text).is_err());
        }
    }
}
