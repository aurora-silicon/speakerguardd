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

/// Which governor writes the card; the others, when they run, only
/// compute and log (see engine.rs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Governor {
    /// speakerguardd's own window/hysteresis governor on the feed-forward
    /// model (model.rs).
    Legacy,
    /// Asahi Linux speakersafetyd's behaviour: measured V*I power, a linear
    /// ramp to its min_gain across t_window, peak-hold hysteresis.
    Asahi,
    /// The look-ahead regulator on the feed-forward model, tightened by the
    /// measured-power model where V/I sense exists.  It writes the card
    /// under the asahi guard (`guard = asahi`), or alone only with
    /// --experimental-unified.
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

    pub fn parse(name: &str) -> Option<Governor> {
        match name {
            "legacy" => Some(Governor::Legacy),
            "asahi" => Some(Governor::Asahi),
            "unified" => Some(Governor::Unified),
            _ => None,
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
    /// Run every governor the machine supports and log them once a second
    /// (observe only).
    pub shadow: bool,
    pub sense_format: SenseFormat,
    /// The amplifiers' output level at full scale (dBV, a 0 dBFS sine), on
    /// machines whose amp gain control the daemon sets to its maximum: the
    /// worst-case power and speakersafetyd's min_gain follow from it.  0 when
    /// the sense words are the amplifier's own full scale (J700).
    pub amp_gain_dbv: f64,
    /// The amp gain control value `amp_gain_dbv` stands for: when set, the
    /// daemon writes it (or the control's maximum, if that is lower)
    /// instead of the maximum, so a control with a wider range than the
    /// profile assumed does not play louder than every worst case.
    pub amp_gain_index: Option<i64>,
    /// Unified governor: prediction horizon (s) and release time constant (s).
    pub horizon_s: f64,
    pub release_s: f64,
    /// Unified governor: the regulator holds the coil this far below the
    /// working limit (K).
    pub t_ctrl_headroom: f64,
    /// Unified governor, macOS-style axis caps (off by default): the thermal
    /// reduction stops at t_reduction_max and the power-budget reduction at
    /// p_reduction_max (dB); past them the coil may run up to its hard
    /// limit, where the daemon hands the speakers to the kernel.
    pub unified_caps: bool,
    pub p_reduction_max: f64,
    /// Gain reduction beyond which the card's safe level takes over (dB);
    /// `-m` overrides it.  speakersafetyd's machines use 7.
    pub max_reduction: f64,
    /// Whether this machine's sense mapping and thermal constants have been
    /// qualified on the machine itself.  An unqualified configuration is
    /// for offline work: the daemon refuses to govern with it unless the
    /// command line says --allow-unqualified.
    pub qualified: bool,
    /// A second governor whose reductions the card also gets: with the
    /// unified governor, `asahi` gives each speaker the larger of the two
    /// governors' reductions, and either one's hard latch hands the card
    /// to the kernel.  Needs V/I sense on every speaker.
    pub guard: Option<Governor>,
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
    /// Thermal constants for the models driven by measured V*I power (the
    /// values validated that way, speakersafetyd's); default: the above.
    pub vi_tr_coil: f64,
    pub vi_tau_coil: f64,
    pub vi_tr_magnet: f64,
    pub vi_tau_magnet: f64,
    /// Coil resistance temperature coefficient at 20 °C (1/K), 0 if unknown.
    pub a_t_20c: f64,
    /// The smallest impedance the speaker can present (Ω), when known
    /// better than the default, the smaller of z_nominal and r_ff: a unit's
    /// own DC resistance (review r4 addendum 4 of speakerguardd#8).  Never
    /// above the default.
    pub z_min_override: Option<f64>,
}

impl Speaker {
    /// The smallest impedance the measured-power bound may assume (Ω): the
    /// DC resistance where it is known and lower than the nominal one, as
    /// the most real power a voltage can drive is V^2 over the coil's
    /// resistance.
    pub fn z_min(&self) -> f64 {
        self.z_min_override.unwrap_or(self.z_nominal.min(self.r_ff))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    pub globals: Globals,
    pub controls: Controls,
    pub speakers: Vec<Speaker>,
    /// Set only by the command line (--experimental-unified, or a replay):
    /// the unified governor may write the card.
    pub allow_unified: bool,
    /// Set only by the command line (--allow-unqualified, or a replay): an
    /// unqualified configuration may govern.
    pub allow_unqualified: bool,
}

type Sections = BTreeMap<String, BTreeMap<String, String>>;

#[cfg(test)]
#[path = "profile_tests.rs"]
mod profile_tests;

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

fn boolean(values: &BTreeMap<String, String>, section: &str, key: &str, default: bool) -> Result<bool, ConfigError> {
    match values.get(key).map(|v| v.to_ascii_lowercase()) {
        None => Ok(default),
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
                None => Governor::Legacy,
                Some(v) => Governor::parse(v).ok_or_else(|| ConfigError(format!("[Globals] governor {v:?}: legacy, asahi or unified")))?,
            },
            shadow: boolean(g, "Globals", "shadow", false)?,
            sense_format: match g.get("sense_format").map(String::as_str) {
                None | Some("s32") => SenseFormat::S32,
                Some("s16") => SenseFormat::S16,
                Some(v) => return Err(ConfigError(format!("[Globals] sense_format {v:?}: s32 or s16"))),
            },
            amp_gain_dbv: or(g, "Globals", "amp_gain_dbv", 0.0)?,
            amp_gain_index: match g.get("amp_gain_index") {
                None => None,
                Some(v) => Some(v.parse::<i64>().ok().filter(|i| *i >= 0).ok_or_else(|| ConfigError(format!("[Globals] amp_gain_index {v:?}: a control value, 0 or more")))?),
            },
            horizon_s: or(g, "Globals", "horizon_s", 4.6)?,
            release_s: or(g, "Globals", "release_s", 20.0)?,
            t_ctrl_headroom: or(g, "Globals", "t_ctrl_headroom", 0.0)?,
            unified_caps: boolean(g, "Globals", "unified_caps", false)?,
            p_reduction_max: or(g, "Globals", "p_reduction_max", 0.0)?,
            max_reduction: or(g, "Globals", "max_reduction", 20.0)?,
            qualified: boolean(g, "Globals", "qualified", true)?,
            guard: match g.get("guard").map(String::as_str) {
                None | Some("none") => None,
                Some("asahi") => Some(Governor::Asahi),
                Some(v) => return Err(ConfigError(format!("[Globals] guard {v:?}: none or asahi"))),
            },
        };
        if !(globals.max_reduction.is_finite() && globals.max_reduction > 0.0) {
            return Err(ConfigError("[Globals] max_reduction must be finite and positive".into()));
        }
        for (key, value) in [
            ("amp_gain_dbv", globals.amp_gain_dbv),
            ("horizon_s", globals.horizon_s),
            ("release_s", globals.release_s),
            ("t_ctrl_headroom", globals.t_ctrl_headroom),
            ("p_reduction_max", globals.p_reduction_max),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(ConfigError(format!("[Globals] {key} must be finite and nonnegative")));
            }
        }
        if !(globals.horizon_s > 0.0) || !(globals.release_s > 0.0) {
            return Err(ConfigError("[Globals] horizon_s and release_s must be positive".into()));
        }
        // the amplifier level enters the worst case as 10^(dBV/20): keep
        // it physical (a laptop amplifier is well under 100 V)
        if globals.amp_gain_dbv > 40.0 {
            return Err(ConfigError("[Globals] amp_gain_dbv beyond 40 dBV".into()));
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
        if globals.amp_gain_index.is_some() && controls.amp_gain.is_none() {
            return Err(ConfigError("[Globals] amp_gain_index needs [Controls] amp_gain".into()));
        }
        let mut speakers = Vec::new();
        for (section, values) in &sections {
            let Some(name) = section.strip_prefix("Speaker/") else { continue };
            let s = |key| num::<f64>(values, section, key);
            let mut speaker = Speaker {
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
                z_min_override: match values.get("z_min") {
                    None => None,
                    Some(_) => Some(num::<f64>(values, section, "z_min")?),
                },
            };
            speaker.r_ff = or(values, section, "r_ff", speaker.z_nominal)?;
            speaker.vi_tr_coil = or(values, section, "vi_tr_coil", speaker.tr_coil)?;
            speaker.vi_tau_coil = or(values, section, "vi_tau_coil", speaker.tau_coil)?;
            speaker.vi_tr_magnet = or(values, section, "vi_tr_magnet", speaker.tr_magnet)?;
            speaker.vi_tau_magnet = or(values, section, "vi_tau_magnet", speaker.tau_magnet)?;
            if let Some(ch) = speaker.is_chan {
                if ch >= globals.channels {
                    return Err(ConfigError(format!("[{section}] is_chan {ch} beyond {} channels", globals.channels)));
                }
                if ch == speaker.vs_chan {
                    return Err(ConfigError(format!("[{section}] is_chan and vs_chan are the same channel")));
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
            if let Some(z) = speaker.z_min_override {
                if !(z.is_finite() && z > 0.0 && z <= speaker.z_nominal.min(speaker.r_ff)) {
                    return Err(ConfigError(format!("[{section}] z_min must be positive and no more than z_nominal and r_ff")));
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
        let cfg = Config { globals, controls, speakers, allow_unified: false, allow_unqualified: false };
        cfg.check_hardware()?;
        Ok(cfg)
    }

    /// Every speaker has current sense: the measured-power models run.
    pub fn has_vi(&self) -> bool {
        self.speakers.iter().all(|s| s.is_chan.is_some()) && self.globals.amp_gain_dbv > 0.0
    }

    /// What the file itself must hold for its governor and guard: the
    /// asahi governor, and the asahi guard, need current sense on every
    /// speaker and the amplifier's level; the guard guards the unified
    /// governor only.
    fn check_hardware(&self) -> Result<(), ConfigError> {
        let g = &self.globals;
        for (asahi, what) in [(g.governor == Governor::Asahi, "governor asahi"), (g.guard == Some(Governor::Asahi), "guard asahi")] {
            if !asahi {
                continue;
            }
            if self.speakers.iter().any(|s| s.is_chan.is_none()) {
                return Err(ConfigError(format!("{what} needs current sense (is_chan) on every speaker")));
            }
            if !(g.amp_gain_dbv > 0.0) {
                return Err(ConfigError(format!("{what} needs [Globals] amp_gain_dbv")));
            }
        }
        if g.guard.is_some() && g.governor != Governor::Unified {
            return Err(ConfigError(format!("guard asahi guards the unified governor, not {}", g.governor.name())));
        }
        Ok(())
    }

    /// Whether this configuration may write the card, after the command
    /// line's choices: the asahi governor needs V/I sense, the unified one
    /// the asahi guard or --experimental-unified, and an unqualified
    /// machine --allow-unqualified.  Guard::start checks it again.
    pub fn check_governor(&self) -> Result<(), ConfigError> {
        self.check_hardware()?;
        if self.globals.governor == Governor::Unified && self.globals.guard.is_none() && !self.allow_unified {
            return Err(ConfigError("governor unified writes the card under guard asahi (or alone with --experimental-unified)".into()));
        }
        if !self.globals.qualified && !self.allow_unqualified {
            return Err(ConfigError("this machine's configuration is not qualified (qualified = false): the daemon does not govern it without --allow-unqualified".into()));
        }
        Ok(())
    }

    /// Tests: `text` with speakersafetyd's governor alone, the V/I files'
    /// default before the feed-forward model became primary (the tests
    /// written against it keep their meaning).
    #[cfg(test)]
    pub fn asahi_alone(text: &str) -> String {
        let s = text.replace("governor = unified\nguard = asahi\n", "governor = asahi\n");
        assert!(s != text || !text.contains("guard = asahi"), "the file's governor lines changed");
        s
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

    const J316: &str = include_str!("../conf/apple/j316.conf");
    const J413: &str = include_str!("../conf/apple/j413.conf");
    const J616: &str = include_str!("../conf/apple/j616.conf");

    fn idx(c: &Config, name: &str) -> usize {
        c.speakers.iter().position(|s| s.name == name).unwrap()
    }

    /// A speaker's own smallest impedance (a unit's DC resistance, review
    /// r4 addendum 4): taken when no larger than the default, refused above.
    #[test]
    fn z_min_can_only_be_lowered() {
        let c = Config::parse(J413).unwrap();
        assert!((c.speakers[idx(&c, "Left Tweeter")].z_min() - 3.72).abs() < 1e-9);
        let text = J413.replacen("[Speaker/Left Tweeter]\n", "[Speaker/Left Tweeter]\nz_min = 3.55\n", 1);
        let c = Config::parse(&text).unwrap();
        assert!((c.speakers[idx(&c, "Left Tweeter")].z_min() - 3.55).abs() < 1e-9);
        assert!((c.speakers[idx(&c, "Right Tweeter")].z_min() - 3.72).abs() < 1e-9);
        let text = J413.replacen("[Speaker/Left Tweeter]\n", "[Speaker/Left Tweeter]\nz_min = 3.9\n", 1);
        assert!(Config::parse(&text).is_err());
    }

    /// amp_gain_index: the control value amp_gain_dbv stands for, a
    /// nonnegative integer, only with an amp gain control to write.
    #[test]
    fn amp_gain_index_is_a_control_value() {
        assert_eq!(Config::parse(J316).unwrap().globals.amp_gain_index, None);
        let with = J316.replace("amp_gain_dbv = 18.5", "amp_gain_dbv = 18.5\namp_gain_index = 15");
        assert_eq!(Config::parse(&with).unwrap().globals.amp_gain_index, Some(15));
        assert!(Config::parse(&with.replace("amp_gain_index = 15", "amp_gain_index = -1")).is_err());
        assert!(Config::parse(&with.replace("amp_gain_index = 15", "amp_gain_index = 1.5")).is_err());
        assert!(Config::parse(&J700.replace("[Globals]", "[Globals]\namp_gain_index = 15")).is_err());
    }

    #[test]
    fn parses_the_vi_machines() {
        for text in [J316, J413, J616] {
            let c = Config::parse(text).unwrap();
            assert!(c.has_vi());
            // feed-forward primary, the V/I model guarding it
            assert_eq!(c.globals.governor, Governor::Unified);
            assert_eq!(c.globals.guard, Some(Governor::Asahi));
            assert_eq!(c.globals.sense_format, SenseFormat::S16);
            assert_eq!(c.globals.max_reduction, 7.0);
            assert_eq!(c.controls.isense.as_deref(), Some("ISENSE Switch"));
        }
        let c = Config::parse(J316).unwrap();
        assert_eq!(c.speakers[idx(&c, "Left Tweeter")].is_chan, Some(4));
        assert!((c.speakers[idx(&c, "Left Woofer 1")].r_ff - 2.7).abs() < 1e-9);
        assert!((c.speakers[idx(&c, "Left Woofer 1")].z_min() - 2.7).abs() < 1e-9);
        assert!((c.speakers[idx(&c, "Left Tweeter")].r_ff - 3.2).abs() < 1e-9);
        // the J616's sense order follows its slots: LW1 RW1 LW2 RW2 LT RT
        let j616 = Config::parse(J616).unwrap();
        let ch = |n: &str| {
            let s = &j616.speakers[idx(&j616, n)];
            (s.is_chan.unwrap(), s.vs_chan)
        };
        assert_eq!(ch("Left Woofer 1"), (0, 1));
        assert_eq!(ch("Right Woofer 2"), (6, 7));
        assert_eq!(ch("Left Tweeter"), (8, 9));
        assert_eq!(ch("Right Tweeter"), (10, 11));
        let j700 = Config::parse(J700).unwrap();
        assert!(!j700.has_vi());
        assert_eq!(j700.globals.governor, Governor::Legacy);
        assert_eq!(j700.speakers[0].r_ff, j700.speakers[0].z_nominal);
        assert!(j700.globals.qualified);
    }

    /// Review F8 (speakerguardd#6): the J616 profile shipped governing
    /// with nothing qualifying its sense map or thermal transfer.  It is
    /// marked unqualified, and an unqualified file does not govern without
    /// --allow-unqualified.  The J316 and J413 are not qualified on the
    /// machines yet either; the J700 is.
    #[test]
    fn an_unqualified_machine_does_not_govern_without_the_flag() {
        for text in [J616, J316, J413] {
            let mut c = Config::parse(text).unwrap();
            assert!(!c.globals.qualified);
            assert!(c.check_governor().is_err());
            c.allow_unqualified = true;
            assert!(c.check_governor().is_ok());
        }
        assert!(Config::parse(J700).unwrap().check_governor().is_ok());
        assert!(Config::parse(&J700.replace("[Globals]", "[Globals]\nqualified = maybe")).is_err());
    }

    /// The unified governor writes the card under the asahi guard, which
    /// needs V/I sense on every speaker and guards nothing else; alone it
    /// still needs the experimental flag.
    #[test]
    fn the_asahi_guard_lets_the_unified_governor_write_the_card() {
        let guarded = J316.to_string();
        let mut c = Config::parse(&guarded).unwrap();
        assert_eq!(c.globals.guard, Some(Governor::Asahi));
        c.allow_unqualified = true;
        assert!(c.check_governor().is_ok(), "{:?}", c.check_governor());
        c.globals.guard = None;
        assert!(c.check_governor().is_err(), "unified governed alone without the flag");
        // the guard guards the unified governor only, and needs V/I sense
        assert!(Config::parse(&J316.replace("governor = unified", "governor = asahi")).is_err());
        assert!(Config::parse(&J700.replace("[Globals]", "[Globals]\ngovernor = unified\nguard = asahi")).is_err());
        assert!(Config::parse(&guarded.replace("amp_gain_dbv = 18.5", "amp_gain_dbv = 0")).is_err());
        assert!(Config::parse(&guarded.replace("guard = asahi", "guard = legacy")).is_err());
        assert_eq!(Config::parse(&guarded.replace("guard = asahi", "guard = none")).unwrap().globals.guard, None);
        // the command line's choice is checked the same way
        let mut c = Config::parse(J316).unwrap();
        c.allow_unqualified = true;
        c.globals.governor = Governor::Asahi;
        assert!(c.check_governor().is_err(), "guarded the asahi governor");
        c.globals.governor = Governor::Unified;
        assert!(c.check_governor().is_ok());
    }

    /// The unified governor is shadow-only: a file naming it parses, so the
    /// command line's opt-in can apply, and only that opt-in lets it govern.
    #[test]
    fn the_unified_governor_needs_the_experimental_flag() {
        let mut c = Config::parse(&J700.replace("[Globals]", "[Globals]\ngovernor = unified")).unwrap();
        assert!(c.check_governor().is_err());
        c.allow_unified = true;
        assert!(c.check_governor().is_ok());
        // asahi needs current sense and the amplifier's level
        assert!(Config::parse(&J700.replace("[Globals]", "[Globals]\ngovernor = asahi")).is_err());
        assert!(Config::parse(&J316.replace("amp_gain_dbv = 18.5", "amp_gain_dbv = 0")).is_err());
        assert!(Config::parse(&J316.replace("governor = unified\nguard = asahi", "governor = legacy")).is_ok());
        assert!(Config::parse(&J700.replace("[Globals]", "[Globals]\ngovernor = loud")).is_err());
        // a current channel on the voltage channel is a broken map
        assert!(Config::parse(&J413.replacen("is_chan = 0", "is_chan = 1", 1)).is_err());
        assert!(Config::parse(&J413.replacen("is_scale = 3.75", "is_scale = 0", 1)).is_err());
        assert!(Config::parse(&J413.replacen("vi_tau_coil = 4.7", "vi_tau_coil = NaN", 1)).is_err());
        assert!(Config::parse(&J413.replace("amp_gain_dbv = 18.5", "amp_gain_dbv = 400")).is_err());
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
