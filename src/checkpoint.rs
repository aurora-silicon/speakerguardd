// SPDX-License-Identifier: MIT
//! Durable thermal checkpoints. The state text is bound to this machine and
//! configuration; the clock and boot identity describe the age of the model.
use std::time::Duration;

use crate::state::{self, Saved};

const MAGIC: &str = "speakerguardd-checkpoint 1";
pub const INTERVAL: Duration = Duration::from_secs(9);
/// The last successful snapshot may be this old while the daemon renews
/// the volume lease. The remaining second covers the 250 ms lease and the
/// 500 ms playback queue, with 250 ms margin.
pub const MAX_PLAYABLE_AGE: Duration = Duration::from_secs(10);
/// The cadence plus a scheduling and shutdown allowance. An unclean save
/// charges this much full-gain playback before crediting idle cooling.
pub const UNCLEAN_BOUND: Duration = Duration::from_secs(11);
/// An explicit clean Linux stop may resume across a short reboot. The wall
/// clock is only an eligibility test here, never cooling credit.
pub const FAST_REBOOT: Duration = Duration::from_secs(90);

#[derive(Clone, Debug, Default)]
pub struct Clock {
    /// CLOCK_REALTIME nanoseconds since the Unix epoch (UTC).
    pub realtime: Option<Duration>,
    pub rtc: Option<Duration>,
    pub boot_id: Option<String>,
    pub boottime: Duration,
}

#[derive(Debug)]
pub struct Checkpoint {
    pub clock: Clock,
    pub machine: String,
    pub config: u64,
    pub active: bool,
    pub state: Saved,
}

fn hash(s: &str) -> u64 {
    s.bytes().fold(0xcbf29ce484222325, |h, b| (h ^ b as u64).wrapping_mul(0x100000001b3))
}

fn token(s: &str) -> bool {
    !s.is_empty() && !s.contains(char::is_whitespace)
}

pub fn encode(clock: &Clock, machine: &str, config: u64, active: bool, state: &str) -> Option<String> {
    let realtime = clock.realtime?;
    let boot_id = clock.boot_id.as_deref()?;
    if !token(machine) || !token(boot_id) || state::decode(state).is_none() {
        return None;
    }
    let rtc = clock.rtc.map_or("-".to_string(), |r| r.as_nanos().to_string());
    let body = format!(
        "machine {machine}\nconfig {config:016x}\nboot_id {boot_id}\nrealtime_ns {}\nrtc_ns {rtc}\nboottime_ns {}\nactive {}\n{state}",
        realtime.as_nanos(), clock.boottime.as_nanos(), active as u8
    );
    Some(format!("{MAGIC}\nsum {:016x}\n{body}", hash(&format!("{MAGIC}\n{body}"))))
}

pub fn decode(text: &str) -> Option<Checkpoint> {
    let rest = text.strip_prefix(MAGIC)?.strip_prefix('\n')?;
    let (sum, body) = rest.strip_prefix("sum ")?.split_once('\n')?;
    if sum.len() != 16 || u64::from_str_radix(sum, 16).ok()? != hash(&format!("{MAGIC}\n{body}")) {
        return None;
    }
    let mut lines = body.splitn(8, '\n');
    let mut field = |name: &str| lines.next()?.strip_prefix(name)?.strip_prefix(' ').map(str::to_string);
    let machine = field("machine")?;
    let config = u64::from_str_radix(&field("config")?, 16).ok()?;
    let boot_id = field("boot_id")?;
    let realtime = Duration::from_nanos(field("realtime_ns")?.parse().ok()?);
    let rtc = match field("rtc_ns")?.as_str() {
        "-" => None,
        s => Some(Duration::from_nanos(s.parse().ok()?)),
    };
    let boottime = Duration::from_nanos(field("boottime_ns")?.parse().ok()?);
    let active = match field("active")?.as_str() { "0" => false, "1" => true, _ => return None };
    let state = state::decode(lines.next()?)?;
    if !token(&machine) || !token(&boot_id) || !state.proven() || state.boottime > boottime {
        return None;
    }
    Some(Checkpoint { clock: Clock { realtime: Some(realtime), rtc, boot_id: Some(boot_id), boottime }, machine, config, active, state })
}

/// CLOCK_BOOTTIME is a lower bound within one boot, including suspend.
/// Realtime is only a veto for backwards jumps. RTC and realtime can both
/// be moved forward together, so neither proves any off time across boots.
pub fn elapsed(old: &Clock, now: &Clock) -> Duration {
    let Some(wall) = old.realtime.and_then(|a| now.realtime?.checked_sub(a)) else { return Duration::ZERO };
    let same_boot = old.boot_id.as_ref().zip(now.boot_id.as_ref()).is_some_and(|(a, b)| a == b);
    if !same_boot { return Duration::ZERO; }
    let Some(reference) = now.boottime.checked_sub(old.boottime) else { return Duration::ZERO };
    wall.min(reference)
}

/// Match the last clean Linux stop to the durable record. This keeps the
/// interim <90 s Linux-to-Linux behavior without using the clock as an
/// off-time witness: the resumed model receives zero cross-boot cooling.
pub fn fast_clean_reboot(saved: &Checkpoint, marker: &Checkpoint, now: &Clock) -> bool {
    if saved.active || marker.active || !saved.state.proven() || !marker.state.proven()
        || saved.machine != marker.machine || saved.config != marker.config
        || saved.clock.boot_id != marker.clock.boot_id
        || saved.clock.boot_id == now.boot_id
        || saved.clock.boottime != marker.clock.boottime
        || saved.clock.realtime != marker.clock.realtime {
        return false;
    }
    let Some(gap) = saved.clock.realtime.and_then(|t| now.realtime?.checked_sub(t)) else { return false };
    gap < FAST_REBOOT
}

pub fn plausible(saved: &Saved, ambient: f64) -> bool {
    let ok = |t: f64| t.is_finite() && t >= ambient - 1e-6;
    saved.speakers.iter().all(|s| ok(s.t_coil) && ok(s.t_magnet) && ok(s.t_peak))
        && saved.asahi.iter().all(|s| ok(s.t_coil) && ok(s.t_magnet) && ok(s.coil_hyst) && ok(s.magnet_hyst) && ok(s.t_peak))
        && saved.unified.iter().all(|s| ok(s.ff.0) && ok(s.ff.1) && ok(s.t_peak) && s.vi.is_none_or(|v| ok(v.0) && ok(v.1)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clock(boot: &str, real: u64, rtc: u64, up: u64) -> Clock {
        Clock { realtime: Some(Duration::from_secs(real)), rtc: Some(Duration::from_secs(rtc)), boot_id: Some(boot.into()), boottime: Duration::from_secs(up) }
    }

    #[test]
    fn long_power_off_and_backward_clock() {
        let old = clock("old", 100, 100, 20);
        assert_eq!(elapsed(&old, &clock("new", 3700, 3700, 1)), Duration::ZERO);
        assert_eq!(elapsed(&old, &clock("new", 90, 90, 1)), Duration::ZERO);
        assert_eq!(elapsed(&old, &clock("old", 90, 90, 21)), Duration::ZERO);
        assert_eq!(elapsed(&old, &clock("new", 3700, 100, 1)), Duration::ZERO);
        assert_eq!(elapsed(&old, &clock("old", 1100, 1100, 21)), Duration::from_secs(1));
        // Both wall and RTC may step forward together. Neither is an
        // independent witness to time that passed while Linux was off.
        assert_eq!(elapsed(&old, &clock("new", 100_000, 100_000, 1)), Duration::ZERO);
        assert_eq!(elapsed(&old, &clock("old", 100_000, 100_000, 21)), Duration::from_secs(1));
    }

    #[test]
    fn round_trip_and_damage_rejection() {
        let cfg = crate::config::Config::parse(include_str!("../conf/apple/j616.conf")).unwrap();
        let speakers: Vec<_> = cfg.speakers.iter().map(|s| crate::model::SpeakerState::new(s.clone(), cfg.globals.t_ambient)).collect();
        let model = state::encode_with(Duration::from_secs(20), Duration::from_secs(20), &speakers, None, false, true);
        let text = encode(&clock("boot", 100, 100, 20), "machine", 7, true, &model).unwrap();
        let saved = decode(&text).unwrap();
        assert_eq!(saved.machine, "machine");
        assert_eq!(saved.config, 7);
        assert!(saved.active);
        assert!(decode(&text.replacen("machine machine", "machine another", 1)).is_none());
        assert!(decode(&text.replacen("checkpoint 1", "checkpoint 2", 1)).is_none());
    }

    #[test]
    fn fast_reboot_requires_the_matching_clean_marker_and_short_positive_gap() {
        let cfg = crate::config::Config::parse(include_str!("../conf/apple/j616.conf")).unwrap();
        let speakers: Vec<_> = cfg.speakers.iter().map(|s| crate::model::SpeakerState::new(s.clone(), cfg.globals.t_ambient)).collect();
        let model = state::encode_with(Duration::from_secs(20), Duration::from_secs(20), &speakers, None, false, true);
        let saved = decode(&encode(&clock("first", 100, 100, 20), "machine", 7, false, &model).unwrap()).unwrap();
        let marker = decode(&encode(&clock("first", 100, 100, 20), "machine", 7, false, &model).unwrap()).unwrap();
        assert!(fast_clean_reboot(&saved, &marker, &clock("next", 189, 189, 1)));
        assert!(!fast_clean_reboot(&saved, &marker, &clock("next", 190, 190, 1)));
        assert!(!fast_clean_reboot(&saved, &marker, &clock("next", 99, 99, 1)));
        assert!(!fast_clean_reboot(&saved, &marker, &clock("first", 101, 101, 21)));
        let wrong = decode(&encode(&clock("other", 100, 100, 20), "machine", 7, false, &model).unwrap()).unwrap();
        assert!(!fast_clean_reboot(&saved, &wrong, &clock("next", 101, 101, 1)));
        let active = decode(&encode(&clock("first", 100, 100, 20), "machine", 7, true, &model).unwrap()).unwrap();
        assert!(!fast_clean_reboot(&saved, &active, &clock("next", 101, 101, 1)));
    }
}
