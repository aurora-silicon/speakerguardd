// SPDX-License-Identifier: MIT
//! The model's state across restarts.  A daemon that exits leaves the
//! speakers at the card's safe level; the one systemd starts next must not
//! begin with a cold coil, or a crash would reset the protection.  The state
//! (temperatures and recent power per speaker) is written once a second to
//! a file under /run, stamped with CLOCK_BOOTTIME, and cooled on load by
//! the time that has passed since.  Each speaker line also carries the last
//! sensed RMS voltage and the governor's reduction, and an `armed` line says
//! whether the daemon holds the card: the file doubles as a status readout
//! (`cat /run/speakerguardd/state`) for measurements on the machine.

use std::time::Duration;

use crate::config::Globals;
use crate::model::SpeakerState;

const MAGIC: &str = "speakerguardd-state 1";

#[derive(Debug, Clone, PartialEq)]
pub struct SavedSpeaker {
    pub name: String,
    pub t_coil: f64,
    pub t_magnet: f64,
    pub mean_1s: f64,
    pub mean_60s: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Saved {
    pub boottime: Duration,
    pub speakers: Vec<SavedSpeaker>,
}

pub fn encode(boottime: Duration, speakers: &[SpeakerState], armed: bool) -> String {
    let mut out = format!("{MAGIC}\nboottime_ns {}\narmed {}\n", boottime.as_nanos(), armed as u8);
    out += "# speaker name t_coil_C t_magnet_C mean_1s_W mean_60s_W v_rms_V reduction_dB\n";
    for s in speakers {
        out += &format!(
            "speaker {} {:.6} {:.6} {:.6} {:.6} {:.6} {:.3}\n",
            s.spec.name,
            s.t_coil,
            s.t_magnet,
            s.mean_power(1.0),
            s.mean_power(60.0),
            s.v_rms,
            s.reduction_db
        );
    }
    out
}

/// None for anything but a complete, finite, physically possible state.
pub fn decode(text: &str) -> Option<Saved> {
    let mut lines = text.lines();
    if lines.next()? != MAGIC {
        return None;
    }
    let ns: u64 = lines.next()?.strip_prefix("boottime_ns ")?.parse().ok()?;
    let mut speakers = Vec::new();
    for line in lines {
        if line.starts_with('#') || line.starts_with("armed ") {
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() != 8 || f[0] != "speaker" || f[6..].iter().any(|x| !x.parse::<f64>().is_ok_and(f64::is_finite)) {
            return None;
        }
        let v: Vec<f64> = f[2..6].iter().map(|x| x.parse().ok()).collect::<Option<_>>()?;
        if v.iter().any(|x| !x.is_finite()) || v[2] < 0.0 || v[3] < 0.0 || v[0] < -50.0 || v[1] < -50.0 {
            return None;
        }
        speakers.push(SavedSpeaker { name: f[1].to_string(), t_coil: v[0], t_magnet: v[1], mean_1s: v[2], mean_60s: v[3] });
    }
    if speakers.is_empty() {
        return None;
    }
    Some(Saved { boottime: Duration::from_nanos(ns), speakers })
}

/// Continue every speaker from the saved state, cooled for `elapsed` with
/// no power.  A speaker the file does not name starts hot.
pub fn restore(speakers: &mut [SpeakerState], saved: &Saved, elapsed: Duration, globals: &Globals) {
    for s in speakers.iter_mut() {
        match saved.speakers.iter().find(|x| x.name == s.spec.name) {
            Some(x) => s.resume(x.t_coil, x.t_magnet, x.mean_1s, x.mean_60s),
            None => s.assume_hot(globals.t_ambient),
        }
        // cool in 0.1 s steps for the first minute (the coil's time
        // constant is seconds), then in one step per second
        let mut left = elapsed.as_secs_f64();
        while left > 0.0 {
            let step: f64 = if elapsed.as_secs_f64() - left < 60.0 { 0.1 } else { 1.0 };
            let dt = step.min(left);
            s.step(0.0, dt, globals.t_ambient);
            left -= dt;
            if elapsed.as_secs_f64() - left > 3600.0 {
                // an hour without power: at ambient for any real speaker
                s.step(0.0, left, globals.t_ambient);
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;

    const J700: &str = include_str!("../conf/apple/j700.conf");

    fn hot() -> (Config, Vec<SpeakerState>) {
        let cfg = Config::parse(J700).unwrap();
        let mut sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 50.0)).collect();
        for s in sp.iter_mut() {
            for _ in 0..300 {
                s.step(1.5, 0.1, 50.0); // 30 s at 1.5 W
            }
        }
        (cfg, sp)
    }

    #[test]
    fn round_trip() {
        let (_, sp) = hot();
        let text = encode(Duration::from_secs(42), &sp, true);
        let saved = decode(&text).unwrap();
        assert_eq!(saved.boottime, Duration::from_secs(42));
        assert_eq!(saved.speakers.len(), 2);
        assert!((saved.speakers[0].t_coil - sp[0].t_coil).abs() < 1e-5);
        assert!((saved.speakers[1].mean_60s - sp[1].mean_power(60.0)).abs() < 1e-5);
    }

    #[test]
    fn rejects_damaged_files() {
        let (_, sp) = hot();
        let good = encode(Duration::from_secs(1), &sp, true);
        assert!(decode("").is_none());
        assert!(decode(&good.replace("speakerguardd-state 1", "speakerguardd-state 2")).is_none());
        assert!(decode(&good[..good.len() - 10]).is_none());
        let nan = good.replacen(&format!("{:.6}", sp[0].t_coil), "NaN", 1);
        assert!(decode(&nan).is_none());
        assert!(decode(&format!("{MAGIC}\nboottime_ns 5\n")).is_none());
    }

    /// A restarted daemon continues hot: right away it asks for the same
    /// reduction the old one had, and only time without power cools it.
    #[test]
    fn a_restart_resumes_a_hot_model_and_cools_it_by_the_gap() {
        let (cfg, sp) = hot();
        let g = &cfg.globals;
        let saved = decode(&encode(Duration::from_secs(100), &sp, true)).unwrap();
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut fresh, &saved, Duration::ZERO, g);
        assert!((fresh[0].t_coil - sp[0].t_coil).abs() < 1e-5);
        assert!(fresh[0].t_coil > 90.0, "{}", fresh[0].t_coil);
        assert!((fresh[0].mean_power(60.0) - sp[0].mean_power(60.0)).abs() < 1e-3);
        let mut later: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut later, &saved, Duration::from_secs(10), g);
        assert!(later[0].t_coil < fresh[0].t_coil - 10.0, "{} vs {}", later[0].t_coil, fresh[0].t_coil);
        assert!(later[0].t_coil > g.t_ambient);
        let mut cold: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut cold, &saved, Duration::from_secs(7200), g);
        assert!((cold[0].t_coil - g.t_ambient).abs() < 0.5, "{}", cold[0].t_coil);
    }

    #[test]
    fn an_unknown_speaker_starts_hot() {
        let (cfg, sp) = hot();
        let mut saved = decode(&encode(Duration::ZERO, &sp, false)).unwrap();
        saved.speakers.retain(|s| s.name == "Left");
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 50.0)).collect();
        restore(&mut fresh, &saved, Duration::ZERO, &cfg.globals);
        assert_eq!(fresh[1].t_coil, 120.0);
    }
}
