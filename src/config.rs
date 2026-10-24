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

/// Which governor writes the card; the others run in shadow (see engine.rs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Governor {
    /// speakerguardd's own window/hysteresis governor on the feed-forward model.
    Legacy,
    /// Asahi speakersafetyd's behaviour: measured V*I power, a linear ramp to
    /// its min_gain across t_window, peak-hold hysteresis.
    Asahi,
    /// The model-predictive regulator on the feed-forward model (corrected
    /// by measured power where V/I sense exists), after macOS's design.
    Unified,
}

impl Governor {
    pub fn name(self) -> &'static str {
        match self {
            Governor::Legacy => "legacy",
            Governor::Asahi => "asahi",
            Governor::Unified => "unified",
        }
    }
}

/// Sample format of the sense PCM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SenseFormat {
    /// One S32 word per speaker: what the amplifier receives (J700).
    S32,
    /// Interleaved S16 voltage and current sense words (TAS2764/SN012776).
    S16,
}

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
    /// The governor that writes the card (default legacy).
    pub governor: Governor,
    /// Compute and log every other governor too (observe only).
    pub shadow: bool,
    pub sense_format: SenseFormat,
    /// The amplifiers' output level at full scale (dBV, a 0 dBFS sine), on
    /// machines whose amp gain control the daemon sets to its maximum: the
    /// worst-case power and speakersafetyd's min_gain follow from it.  0 when
    /// the sense words are the amplifier's own full scale (J700).
    pub amp_gain_dbv: f64,
    /// Unified governor: prediction horizon (s) and release time constant (s).
    pub horizon_s: f64,
    pub release_s: f64,
    /// Unified governor: the regulator holds the coil this far below the
    /// working limit (K).
    pub t_ctrl_headroom: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Controls {
    pub volume: String,
    pub unlock: String,
    pub sample_rate: String,
    /// Per-speaker controls named "<speaker> <name>" (V/I machines): the
    /// sense switches turned on, and the amp gain set to its maximum.
    pub vsense: Option<String>,
    pub isense: Option<String>,
    pub amp_gain: Option<String>,
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
    /// Current sense channel and amps at full scale (V/I machines).
    pub is_chan: Option<usize>,
    pub is_scale: f64,
    /// DC resistance the feed-forward power is computed against (Ω):
    /// macOS's Reb_ref / RdcOhms.  Defaults to z_nominal.
    pub r_ff: f64,
    /// Thermal constants for the model driven by measured V*I power (the
    /// values validated that way, speakersafetyd's); default: the above.
    pub vi_tr_coil: f64,
    pub vi_tau_coil: f64,
    pub vi_tr_magnet: f64,
    pub vi_tau_magnet: f64,
    /// Coil resistance temperature coefficient at 20 °C (1/K), 0 if unknown.
    pub a_t_20c: f64,
}

impl Speaker {
    /// RMS volts of the most the amplifier can put out at the card's full
    /// gain: a full-scale square wave.
    pub fn v_worst(&self, globals: &Globals) -> f64 {
        if globals.amp_gain_dbv > 0.0 {
            // a 0 dBFS sine is amp_gain_dbv; the square has sqrt(2) more
            2f64.sqrt() * 10f64.powf(globals.amp_gain_dbv / 20.0)
        } else {
            self.vs_scale
        }
    }
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

fn boolean(values: &BTreeMap<String, String>, section: &str, key: &str) -> Result<bool, ConfigError> {
    match values.get(key).map(|v| v.to_ascii_lowercase()) {
        None => Ok(false),
        Some(v) if ["1", "true", "yes", "on"].contains(&v.as_str()) => Ok(true),
        Some(v) if ["0", "false", "no", "off"].contains(&v.as_str()) => Ok(false),
        Some(_) => Err(ConfigError(format!("[{section}] {key}: not a boolean"))),
    }
}

/// A numeric key that may be absent (`default` when it is).
fn or(values: &BTreeMap<String, String>, section: &str, key: &str, default: f64) -> Result<f64, ConfigError> {
    match values.get(key) {
        None => Ok(default),
        Some(_) => num(values, section, key),
    }
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
            governor: match g.get("governor").map(String::as_str) {
                None | Some("legacy") => Governor::Legacy,
                Some("asahi") => Governor::Asahi,
                Some("unified") => Governor::Unified,
                Some(v) => return Err(ConfigError(format!("[Globals] governor {v:?}: legacy, asahi or unified"))),
            },
            shadow: boolean(g, "Globals", "shadow")?,
            sense_format: match g.get("sense_format").map(String::as_str) {
                None | Some("s32") => SenseFormat::S32,
                Some("s16") => SenseFormat::S16,
                Some(v) => return Err(ConfigError(format!("[Globals] sense_format {v:?}: s32 or s16"))),
            },
            amp_gain_dbv: or(g, "Globals", "amp_gain_dbv", 0.0)?,
            horizon_s: or(g, "Globals", "horizon_s", 4.6)?,
            release_s: or(g, "Globals", "release_s", 20.0)?,
            t_ctrl_headroom: or(g, "Globals", "t_ctrl_headroom", 0.0)?,
        };
        for (key, value) in [
            ("amp_gain_dbv", globals.amp_gain_dbv),
            ("horizon_s", globals.horizon_s),
            ("release_s", globals.release_s),
            ("t_ctrl_headroom", globals.t_ctrl_headroom),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(ConfigError(format!("[Globals] {key} must be finite and nonnegative")));
            }
        }
        if !(globals.horizon_s > 0.0) || !(globals.release_s > 0.0) {
            return Err(ConfigError("[Globals] horizon_s and release_s must be positive".into()));
        }
        for (key, value) in [
            ("t_ambient", globals.t_ambient),
            ("t_hysteresis", globals.t_hysteresis),
            ("t_window", globals.t_window),
            ("t_reduction_max", globals.t_reduction_max),
            ("safe_reduction", globals.safe_reduction),
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
            vsense: c.get("vsense").cloned(),
            isense: c.get("isense").cloned(),
            amp_gain: c.get("amp_gain").cloned(),
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
                is_chan: match values.get("is_chan") {
                    None => None,
                    Some(_) => Some(num(values, section, "is_chan")?),
                },
                is_scale: optional(values, section, "is_scale")?,
                r_ff: 0.0,
                vi_tr_coil: 0.0,
                vi_tau_coil: 0.0,
                vi_tr_magnet: 0.0,
                vi_tau_magnet: 0.0,
                a_t_20c: optional(values, section, "a_t_20c")?,
            };
            let mut speaker = speaker;
            speaker.r_ff = or(values, section, "r_ff", speaker.z_nominal)?;
            speaker.vi_tr_coil = or(values, section, "vi_tr_coil", speaker.tr_coil)?;
            speaker.vi_tau_coil = or(values, section, "vi_tau_coil", speaker.tau_coil)?;
            speaker.vi_tr_magnet = or(values, section, "vi_tr_magnet", speaker.tr_magnet)?;
            speaker.vi_tau_magnet = or(values, section, "vi_tau_magnet", speaker.tau_magnet)?;
            if let Some(ch) = speaker.is_chan {
                if ch >= globals.channels {
                    return Err(ConfigError(format!("[{section}] is_chan {ch} beyond {} channels", globals.channels)));
                }
                if !(speaker.is_scale.is_finite() && speaker.is_scale > 0.0) {
                    return Err(ConfigError(format!("[{section}] is_chan needs a positive is_scale")));
                }
            }
            for (key, value) in [
                ("r_ff", speaker.r_ff),
                ("vi_tr_coil", speaker.vi_tr_coil),
                ("vi_tau_coil", speaker.vi_tau_coil),
                ("vi_tr_magnet", speaker.vi_tr_magnet),
                ("vi_tau_magnet", speaker.vi_tau_magnet),
            ] {
                if !value.is_finite() || !(value > 0.0) {
                    return Err(ConfigError(format!("[{section}] {key} must be positive")));
                }
            }
            if !speaker.a_t_20c.is_finite() || speaker.a_t_20c < 0.0 {
                return Err(ConfigError(format!("[{section}] a_t_20c must be finite and nonnegative")));
            }
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
        let cfg = Config { globals, controls, speakers };
        cfg.check_governor()?;
        Ok(cfg)
    }

    /// The asahi governor needs current sense and the amplifier's level.
    pub fn check_governor(&self) -> Result<(), ConfigError> {
        if self.globals.governor == Governor::Asahi {
            if self.speakers.iter().any(|s| s.is_chan.is_none()) {
                return Err(ConfigError("governor asahi needs current sense (is_chan) on every speaker".into()));
            }
            if !(self.globals.amp_gain_dbv > 0.0) {
                return Err(ConfigError("governor asahi needs [Globals] amp_gain_dbv".into()));
            }
        }
        Ok(())
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
