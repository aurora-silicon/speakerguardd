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
//!
//! The time since the snapshot is not assumed silent.  A run that may have
//! held the card (`armed 1`) may have played at full gain after it, up to
//! the restore tail the daemon bounds by refusing to hold the card on a
//! stale snapshot; the rest of the gap, and all of it after a run that had
//! released the card, is charged at the worst case the card's own safe
//! level lets through.

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
    /// Whether the run that wrote it may have held the card's volume lock
    /// (a file without the line counts as one that may have).
    pub armed: bool,
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
    let mut armed = true;
    for line in lines {
        if line.starts_with('#') {
            continue;
        }
        if let Some(v) = line.strip_prefix("armed ") {
            armed = match v {
                "0" => false,
                "1" => true,
                _ => return None,
            };
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
    Some(Saved { boottime: Duration::from_nanos(ns), armed, speakers })
}

/// Continue every speaker from the saved state.  The `elapsed` time since
/// it was written is charged: its first `tail` at the worst case at full
/// gain if that run may have held the card, the rest at the worst case at
/// the card's safe level.  A speaker the file does not name starts hot.
pub fn restore(speakers: &mut [SpeakerState], saved: &Saved, elapsed: Duration, tail: Duration, globals: &Globals) {
    for s in speakers.iter_mut() {
        match saved.speakers.iter().find(|x| x.name == s.spec.name) {
            Some(x) => s.resume(x.t_coil, x.t_magnet, x.mean_1s, x.mean_60s),
            None => s.assume_hot(globals.t_ambient),
        }
        let mut left = elapsed.as_secs_f64();
        if saved.armed {
            let t = left.min(tail.as_secs_f64());
            if t > 0.0 {
                s.step_at(s.worst_case_power(0.0), t, globals.t_ambient, 0.0);
                left -= t;
            }
        }
        // the safe level's worst case, as it reached the speaker (the
        // budgets' "before the card's gain" history is about this run's
        // gain, not the kernel's)
        let p = s.worst_case_power(globals.safe_reduction);
        if left > 3600.0 {
            // an hour at it is its steady state for any real speaker
            s.settle_at(p, globals.t_ambient);
        } else {
            while left > 0.0 {
                let dt = left.min(1.0);
                s.step_at(p, dt, globals.t_ambient, 0.0);
                left -= dt;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;

    const J700: &str = include_str!("../conf/apple/j700.conf");
    const TAIL: Duration = Duration::from_millis(1500);

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
        restore(&mut fresh, &saved, Duration::ZERO, TAIL, g);
        assert!((fresh[0].t_coil - sp[0].t_coil).abs() < 1e-5);
        assert!(fresh[0].t_coil > 90.0, "{}", fresh[0].t_coil);
        assert!((fresh[0].mean_power(60.0) - sp[0].mean_power(60.0)).abs() < 1e-3);
        // a run that had released the card: the gap cools at the safe level
        let released = decode(&encode(Duration::from_secs(100), &sp, false)).unwrap();
        let mut later: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut later, &released, Duration::from_secs(10), TAIL, g);
        assert!(later[0].t_coil < fresh[0].t_coil - 10.0, "{} vs {}", later[0].t_coil, fresh[0].t_coil);
        assert!(later[0].t_coil > g.t_ambient);
        // ...and never below the steady state of the safe level's worst case
        let floor = {
            let s = &later[0];
            let p = s.worst_case_power(g.safe_reduction);
            g.t_ambient + p * (s.spec.tr_coil + s.spec.tr_magnet)
        };
        let mut cold: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut cold, &released, Duration::from_secs(7200), TAIL, g);
        assert!((cold[0].t_coil - floor).abs() < 0.5, "{} vs {floor}", cold[0].t_coil);
    }

    /// Review F5 (speakerguardd#6, also in 0.1.2): the time after a
    /// snapshot of a run that held the card is not silence.  A 119 C coil
    /// saved one second before the restart restored to 108.8 C in 0.1.2;
    /// it must not come back colder than it was.
    #[test]
    fn the_time_after_a_snapshot_of_a_held_card_is_not_cooled() {
        let (cfg, mut sp) = hot();
        let g = &cfg.globals;
        for s in sp.iter_mut() {
            s.resume(119.0, 95.0, 0.0, 0.0);
        }
        let saved = decode(&encode(Duration::from_secs(100), &sp, true)).unwrap();
        assert!(saved.armed);
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut fresh, &saved, Duration::from_secs(1), TAIL, g);
        assert!(fresh[0].t_coil >= 119.0, "restored to {:.3} C", fresh[0].t_coil);
        // a file without the armed line (0.1.2 wrote one; damaged ones may not) counts as held
        let text = encode(Duration::from_secs(100), &sp, false).replace("armed 0\n", "");
        assert!(decode(&text).unwrap().armed);
    }

    #[test]
    fn an_unknown_speaker_starts_hot() {
        let (cfg, sp) = hot();
        let mut saved = decode(&encode(Duration::ZERO, &sp, false)).unwrap();
        saved.speakers.retain(|s| s.name == "Left");
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 50.0)).collect();
        restore(&mut fresh, &saved, Duration::ZERO, TAIL, &cfg.globals);
        assert_eq!(fresh[1].t_coil, 120.0);
    }
}
