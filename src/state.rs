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

const MAGIC: &str = "speakerguardd-state 2";
/// The format 0.1.2 to 0.1.5 wrote: no coverage epoch, no latch.
const MAGIC_V1: &str = "speakerguardd-state 1";

#[derive(Debug, Clone, PartialEq)]
pub struct SavedSpeaker {
    pub name: String,
    pub t_coil: f64,
    pub t_magnet: f64,
    pub mean_1s: f64,
    pub mean_60s: f64,
    /// The governor's reduction being released (dB).
    pub reduction_db: f64,
    /// The latched hard fault and the peak bound (None in a v1 file).
    pub hard: Option<bool>,
    pub t_peak: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Saved {
    /// When the file was written.
    pub boottime: Duration,
    /// Up to when the model it holds has accounted the speakers' power:
    /// time after this is unknown to it, whatever the file's write time.
    pub covered: Duration,
    /// Whether the run that wrote it may have held the card's volume lock
    /// (a file without the line counts as one that may have).
    pub armed: bool,
    pub speakers: Vec<SavedSpeaker>,
}

pub fn encode(boottime: Duration, covered: Duration, speakers: &[SpeakerState], armed: bool) -> String {
    let mut out = format!(
        "{MAGIC}\nboottime_ns {}\ncovered_ns {}\narmed {}\n",
        boottime.as_nanos(),
        covered.min(boottime).as_nanos(),
        armed as u8
    );
    out += "# speaker name t_coil_C t_magnet_C mean_1s_W mean_60s_W v_rms_V reduction_dB hard t_peak_C\n";
    for s in speakers {
        out += &format!(
            "speaker {} {:.6} {:.6} {:.6} {:.6} {:.6} {:.3} {} {:.6}\n",
            s.spec.name,
            s.t_coil,
            s.t_magnet,
            s.mean_power(1.0),
            s.mean_power(60.0),
            s.v_rms,
            s.reduction_db,
            s.hard as u8,
            s.t_peak.max(s.t_coil)
        );
    }
    out += "end\n";
    out
}

/// None for anything but a complete, finite, physically possible state.
pub fn decode(text: &str) -> Option<Saved> {
    let mut lines = text.lines();
    let v2 = match lines.next()? {
        MAGIC => true,
        MAGIC_V1 => false,
        _ => return None,
    };
    let ns: u64 = lines.next()?.strip_prefix("boottime_ns ")?.parse().ok()?;
    let mut covered = None;
    let mut speakers = Vec::new();
    let mut armed = true;
    let mut ended = false;
    for line in lines {
        if ended {
            return None;
        }
        if line.starts_with('#') {
            continue;
        }
        if line == "end" {
            ended = true;
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
        if let Some(v) = line.strip_prefix("covered_ns ") {
            covered = Some(v.parse::<u64>().ok()?);
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        let fields = if v2 { 10 } else { 8 };
        if f.len() != fields || f[0] != "speaker" {
            return None;
        }
        let v: Vec<f64> = f[2..8].iter().map(|x| x.parse().ok()).collect::<Option<_>>()?;
        if v.iter().any(|x| !x.is_finite()) || v[2] < 0.0 || v[3] < 0.0 || v[0] < -50.0 || v[1] < -50.0 || v[5] < 0.0 {
            return None;
        }
        let (hard, t_peak) = if v2 {
            let hard = match f[8] {
                "0" => false,
                "1" => true,
                _ => return None,
            };
            let peak: f64 = f[9].parse().ok().filter(|p: &f64| p.is_finite())?;
            (Some(hard), Some(peak))
        } else {
            (None, None)
        };
        speakers.push(SavedSpeaker { name: f[1].to_string(), t_coil: v[0], t_magnet: v[1], mean_1s: v[2], mean_60s: v[3], reduction_db: v[5], hard, t_peak });
    }
    // a v2 file is whole only with its end line
    if speakers.is_empty() || (v2 && !ended) {
        return None;
    }
    // a v1 file says nothing about coverage: its write time is all there is
    let covered = Duration::from_nanos(covered.unwrap_or(ns).min(ns));
    if v2 && covered > Duration::from_nanos(ns) {
        return None;
    }
    Some(Saved { boottime: Duration::from_nanos(ns), covered, armed, speakers })
}

/// Continue every speaker from the saved state, and charge the time since
/// the model's coverage ended (`now` is the time since boot): while the
/// run that wrote it may have held the card, that is everything up to the
/// file's write time and the `tail` after it, at full gain; the rest, and
/// all of it after a run that had released the card, at the worst case the
/// card's safe level lets through.  A speaker the file does not name starts
/// latched at its limit.
pub fn restore(speakers: &mut [SpeakerState], saved: &Saved, now: Duration, tail: Duration, globals: &Globals) {
    let elapsed = now.saturating_sub(saved.covered).as_secs_f64();
    let held = saved.boottime.saturating_sub(saved.covered).as_secs_f64() + tail.as_secs_f64();
    for s in speakers.iter_mut() {
        match saved.speakers.iter().find(|x| x.name == s.spec.name) {
            Some(x) => {
                s.resume(x.t_coil, x.t_magnet, x.mean_1s, x.mean_60s);
                s.resume_governor(x.hard, x.t_peak.unwrap_or(x.t_coil), x.reduction_db);
            }
            None => {
                s.assume_latched(globals.t_ambient);
                continue;
            }
        }
        let mut left = elapsed;
        if saved.armed {
            let t = left.min(held);
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
            let hard = s.hard;
            s.settle_at(p, globals.t_ambient);
            s.hard = hard;
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
        let text = encode(Duration::from_secs(42), Duration::from_secs(42), &sp, true);
        let saved = decode(&text).unwrap();
        assert_eq!(saved.boottime, Duration::from_secs(42));
        assert_eq!(saved.speakers.len(), 2);
        assert!((saved.speakers[0].t_coil - sp[0].t_coil).abs() < 1e-5);
        assert!((saved.speakers[1].mean_60s - sp[1].mean_power(60.0)).abs() < 1e-5);
    }

    #[test]
    fn rejects_damaged_files() {
        let (_, sp) = hot();
        let good = encode(Duration::from_secs(1), Duration::from_secs(1), &sp, true);
        assert!(decode("").is_none());
        assert!(decode(&good.replace("speakerguardd-state 2", "speakerguardd-state 3")).is_none());
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
        let saved = decode(&encode(Duration::from_secs(100), Duration::from_secs(100), &sp, true)).unwrap();
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut fresh, &saved, Duration::from_secs(100), TAIL, g);
        assert!((fresh[0].t_coil - sp[0].t_coil).abs() < 1e-5);
        assert!(fresh[0].t_coil > 90.0, "{}", fresh[0].t_coil);
        assert!((fresh[0].mean_power(60.0) - sp[0].mean_power(60.0)).abs() < 1e-3);
        // a run that had released the card: the gap cools at the safe level
        let released = decode(&encode(Duration::from_secs(100), Duration::from_secs(100), &sp, false)).unwrap();
        let mut later: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut later, &released, Duration::from_secs(110), TAIL, g);
        assert!(later[0].t_coil < fresh[0].t_coil - 10.0, "{} vs {}", later[0].t_coil, fresh[0].t_coil);
        assert!(later[0].t_coil > g.t_ambient);
        // ...and never below the steady state of the safe level's worst case
        let floor = {
            let s = &later[0];
            let p = s.worst_case_power(g.safe_reduction);
            g.t_ambient + p * (s.spec.tr_coil + s.spec.tr_magnet)
        };
        let mut cold: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut cold, &released, Duration::from_secs(7300), TAIL, g);
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
        let saved = decode(&encode(Duration::from_secs(100), Duration::from_secs(100), &sp, true)).unwrap();
        assert!(saved.armed);
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut fresh, &saved, Duration::from_secs(101), TAIL, g);
        assert!(fresh[0].t_coil >= 119.0, "restored to {:.3} C", fresh[0].t_coil);
        // a file without the armed line (0.1.2 wrote one; damaged ones may not) counts as held
        let text = encode(Duration::from_secs(100), Duration::from_secs(100), &sp, false).replace("armed 0\n", "");
        assert!(decode(&text).unwrap().armed);
    }

    #[test]
    fn an_unknown_speaker_starts_latched() {
        let (cfg, sp) = hot();
        let mut saved = decode(&encode(Duration::ZERO, Duration::ZERO, &sp, false)).unwrap();
        saved.speakers.retain(|s| s.name == "Left");
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 50.0)).collect();
        restore(&mut fresh, &saved, Duration::ZERO, TAIL, &cfg.globals);
        assert_eq!(fresh[1].t_coil, 135.0);
        assert!(fresh[1].hard, "an unknown speaker starts latched");
    }

    /// A v1 file (0.1.2 to 0.1.5) still restores; its coverage is its write
    /// time, and a coil not under the working ceiling comes back latched.
    #[test]
    fn a_v1_file_restores_conservatively() {
        let (cfg, mut sp) = hot();
        let g = &cfg.globals;
        for s in sp.iter_mut() {
            s.resume(131.9, 100.0, 0.0, 0.0);
        }
        let v1 = encode(Duration::from_secs(100), Duration::from_secs(100), &sp, false)
            .replace("speakerguardd-state 2", "speakerguardd-state 1")
            .lines()
            .filter(|l| !l.starts_with("covered_ns"))
            .map(|l| if l.starts_with("speaker ") { l.split_whitespace().take(8).collect::<Vec<_>>().join(" ") } else { l.to_string() })
            .collect::<Vec<_>>()
            .join("\n");
        let saved = decode(&v1).expect("v1 decodes");
        assert_eq!(saved.covered, saved.boottime);
        assert_eq!(saved.speakers[0].hard, None);
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut fresh, &saved, Duration::from_secs(100), TAIL, g);
        assert!(fresh[0].hard, "131.9 C without a latch record must come back latched");
    }

    /// Review R4: the latch and its peak survive a restart.  0.1.3 wrote
    /// neither, and a run latched at a 135.3 C peak, saved at 131.9 C,
    /// rearmed after a restart at 127.0 C (16.6 dB).
    #[test]
    fn the_hard_latch_survives_a_restart() {
        let (cfg, mut sp) = hot();
        let g = &cfg.globals;
        for s in sp.iter_mut() {
            s.resume(131.9, 100.0, 0.0, 0.0);
            s.hard = true;
            s.t_peak = 135.256;
        }
        let saved = decode(&encode(Duration::from_secs(100), Duration::from_secs(100), &sp, false)).unwrap();
        assert_eq!(saved.speakers[0].hard, Some(true));
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut fresh, &saved, Duration::from_millis(100_050), TAIL, g);
        assert!(fresh[0].hard && fresh[0].t_peak >= 131.9);
        assert!(!fresh[0].clear_hard_if_cool(), "cleared above the ceiling");
    }

    /// Review R2: the time between the model's coverage and the file's
    /// write is charged on restore.  0.1.3 stamped a snapshot as current
    /// over 50 ms of unaccounted playback and restored it as is.
    #[test]
    fn the_time_before_the_write_is_charged_from_the_coverage_epoch() {
        let (cfg, mut sp) = hot();
        let g = &cfg.globals;
        for s in sp.iter_mut() {
            s.resume(56.829, 52.0, 0.0, 0.0);
        }
        let saved = decode(&encode(Duration::from_millis(100_050), Duration::from_millis(100_000), &sp, true)).unwrap();
        assert_eq!(saved.covered, Duration::from_millis(100_000));
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        // a restart with no time since the write: the 50 ms before it still counts
        restore(&mut fresh, &saved, Duration::from_millis(100_050), TAIL, g);
        let mut want = fresh[0].clone();
        want.resume(56.829, 52.0, 0.0, 0.0);
        want.step_at(want.worst_case_power(0.0), 0.05, g.t_ambient, 0.0);
        assert!(fresh[0].t_coil >= want.t_coil - 1e-9, "{} vs {}", fresh[0].t_coil, want.t_coil);
    }

    /// Review R5: the first update after a restore keeps the minute's
    /// energy.  0.1.3 dropped the whole 59 s entry on the first step: an
    /// 8 W minute read 0.13 W two seconds later.
    #[test]
    fn the_minute_budget_survives_the_first_steps_after_a_restore() {
        let (cfg, mut sp) = hot();
        let g = &cfg.globals;
        for s in sp.iter_mut() {
            s.resume(60.0, 55.0, 8.0, 8.0);
        }
        let saved = decode(&encode(Duration::from_secs(100), Duration::from_secs(100), &sp, false)).unwrap();
        let mut fresh: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
        restore(&mut fresh, &saved, Duration::from_secs(100), TAIL, g);
        for _ in 0..20 {
            fresh[0].step(0.0, 0.1, g.t_ambient);
        }
        let mean = fresh[0].mean_power(60.0);
        assert!((mean - 8.0 * 58.0 / 60.0).abs() < 0.01, "minute mean {mean} W");
        // and it ages out over the minute, not at once
        for _ in 0..290 {
            fresh[0].step(0.0, 0.1, g.t_ambient);
        }
        let mean = fresh[0].mean_power(60.0);
        assert!((mean - 8.0 * 29.0 / 60.0).abs() < 0.01, "minute mean {mean} W after 31 s");
    }
}
