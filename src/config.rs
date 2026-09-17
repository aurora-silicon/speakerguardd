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
        };
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
                if !(value > 0.0) {
                    return Err(ConfigError(format!("[{section}] {key} must be positive")));
                }
            }
            if speaker.t_limit - speaker.t_headroom <= globals.t_ambient + globals.t_window {
                return Err(ConfigError(format!("[{section}] no room between ambient and the limit")));
            }
            speakers.push(speaker);
        }
        if speakers.is_empty() {
            return Err(ConfigError("no [Speaker/...] section".into()));
        }
        if globals.period == 0 || globals.channels == 0 {
            return Err(ConfigError("[Globals] period and channels must be positive".into()));
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
        assert!((c.speakers[0].z_nominal - 4.43).abs() < 1e-9);
    }

    #[test]
    fn rejects_bad_files() {
        assert!(Config::parse("").is_err());
        assert!(Config::parse("[Globals]\nsense_pcm = x\n").is_err());
        let broken = J700.replace("z_nominal = 4.43", "z_nominal = 0");
        assert!(Config::parse(&broken).is_err());
        let hot = J700.replace("t_limit = 110.0", "t_limit = 40.0");
        assert!(Config::parse(&hot).is_err());
    }
}
