// SPDX-License-Identifier: MIT
//! The model's state across a reboot (`boot_resume = idle`; off by
//! default).  The state under /run (state.rs) dies with the boot, so every
//! boot starts at the coils' limit, latched: on the Neo that cost an 11.9
//! dB first lock, released over 7.6 min.  A clean stop made while the
//! system shuts down also writes its state here, under /var/lib.  The
//! first start of the next boot resumes it, charging the time since at the
//! worst case the card's own safe level lets through, as a restart within
//! a boot does.  Anything this file cannot prove starts latched, as
//! without it.
//!
//! Restore requirements and operating assumptions:
//! * **Continuity.** The writing boot is adjacent in the retained journal.
//!   This is only a necessary check. Enabling idle restore asserts the
//!   README's complete history/owner contract: no unrecorded Linux boots,
//!   no other OS, the same durable invalidation path for every owner, and
//!   no later owner above the safe level after the writing boot's save.
//!   A volatile, incomplete or vacuumed journal cannot establish this.
//! * **Origin.**  The same machine (/etc/machine-id) and the same parsed
//!   configuration (record::fingerprint), and a checksum over the whole
//!   record: a file from elsewhere, or damaged into valid text, is not
//!   resumed.  Every temperature is at least the configuration's ambient
//!   (no model goes colder): a colder one is damage.
//! * **Time.** Best-effort synchronized realtime must agree with an
//!   independent, accurate RTC at both ends. Overlapping setters can evade
//!   the timerfd watch. The restored interval therefore includes the full
//!   RTC uncertainty, not just the realtime gap. Missing RTC starts latched.
//!   RTC quantization, bounded read placement and 0.1% rate uncertainty are
//!   included, as is snapshot pairing. Restore bounds every interior age.
//!
//! Only a clean stop at shutdown is kept. The installation must guarantee
//! that after it no owner exceeds the card's safe level until power-off;
//! systemd stopping alone cannot prove this. Every start, with the option
//! or without, removes the file
//! (durably: the directory synced) before the lock can be lifted, and does
//! not unlock while it cannot.
//!
//! **What Linux cannot see.**  The journal knows Linux boots only.  On a
//! machine that also boots macOS (or anything else that drives the same
//! speakers), another OS may have run between the shutdown and this boot
//! and left a coil at its limit; this boot would start from the cooler
//! state saved before it.  On the J700 the review measured 58 K over the
//! limit that way.  Nothing Linux can read proves no other OS ran, so
//! `idle` requires the full README contract, including correct synchronized
//! clocks and complete retained Linux history, not just a Linux-only label.
//! The default keeps the latched start.

use std::time::Duration;

use crate::state::{self, Saved};

// Earlier formats did not require an independent RTC age bound.
// Do not promote their timestamp promise by changing only the header.
const MAGIC: &str = "speakerguardd-boot 4";
/// Two clocks read a little apart, and the RTC's whole seconds.
const SLACK: Duration = Duration::from_secs(2);
/// The most a clock read can sit away from the boot time read next to it.
pub const READ_EPS: Duration = Duration::from_millis(50);

/// The clocks and the boot, as read now.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Wall {
    pub realtime: Option<Duration>,
    /// Passed the best-effort synchronization/watch checks. These do not
    /// prove coherence under overlapping setters; gap also requires RTC.
    pub synced: bool,
    pub rtc: Option<Duration>,
    pub boot_id: Option<String>,
}

/// Where a record comes from: the machine and the parsed configuration.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Origin {
    pub machine_id: Option<String>,
    pub config: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Persisted {
    pub wall: Wall,
    pub origin: Origin,
    /// Boot time from the snapshot's end to after the clocks were read.
    pub pairing: Duration,
    pub state: Saved,
}

fn fnv(text: &str) -> u64 {
    text.bytes().fold(0xcbf29ce484222325u64, |h, b| (h ^ b as u64).wrapping_mul(0x100000001b3))
}

/// Bind the format promise as well as the data. A v2 body checksum must
/// not become a v3 record just by flipping the header's last bit.
fn record_sum(body: &str) -> u64 {
    fnv(&format!("{MAGIC}\n{body}"))
}

fn word(s: &str) -> Option<&str> {
    Some(s).filter(|s| !s.is_empty() && !s.contains(char::is_whitespace))
}

/// A boot id as the kernel and the journal write it (with or without
/// dashes), compared without them.
pub fn same_boot(a: &str, b: &str) -> bool {
    let n = |s: &str| s.trim().replace('-', "").to_ascii_lowercase();
    !n(a).is_empty() && n(a) == n(b)
}

/// The record for `state` (a clean stop's state text, its snapshot ending
/// `pairing` of boot time before the clocks in `wall` were read), or None
/// if the independent RTC is absent, the system clock fails its checks,
/// or the boot or machine has no id: such a record could not be resumed.
pub fn encode(wall: &Wall, origin: &Origin, pairing: Duration, state: &str) -> Option<String> {
    let boot_id = word(wall.boot_id.as_deref()?)?;
    let machine = word(origin.machine_id.as_deref()?)?;
    let realtime = wall.realtime.filter(|_| wall.synced)?;
    let rtc = wall.rtc?.as_nanos().to_string();
    let body = format!(
        "boot_id {boot_id}\nmachine_id {machine}\nconfig {:016x}\nrealtime_ns {}\nsynced 1\nrtc_ns {rtc}\npairing_ns {}\n{state}",
        origin.config,
        realtime.as_nanos(),
        pairing.as_nanos()
    );
    Some(format!("{MAGIC}\nsum {:016x}\n{body}", record_sum(&body)))
}

pub fn decode(text: &str) -> Option<Persisted> {
    let rest = text.strip_prefix(MAGIC)?.strip_prefix('\n')?;
    let (sum, body) = rest.strip_prefix("sum ")?.split_once('\n')?;
    if u64::from_str_radix(sum, 16).ok()? != record_sum(body) || sum.len() != 16 {
        return None;
    }
    let mut lines = body.splitn(8, '\n');
    let mut field = |key: &str| -> Option<String> { lines.next()?.strip_prefix(key)?.strip_prefix(' ').map(str::to_string) };
    let boot_id = field("boot_id")?;
    let machine = field("machine_id")?;
    let config = u64::from_str_radix(&field("config")?, 16).ok()?;
    let realtime: u64 = field("realtime_ns")?.parse().ok()?;
    if field("synced")? != "1" {
        return None;
    }
    let rtc = match field("rtc_ns")?.as_str() {
        "-" => None,
        v => Some(Duration::from_nanos(v.parse().ok()?)),
    };
    let pairing: u64 = field("pairing_ns")?.parse().ok()?;
    let state = state::decode(lines.next()?)?;
    word(&boot_id)?;
    word(&machine)?;
    Some(Persisted {
        wall: Wall { realtime: Some(Duration::from_nanos(realtime)), synced: true, rtc, boot_id: Some(boot_id) },
        origin: Origin { machine_id: Some(machine), config },
        pairing: Duration::from_nanos(pairing),
        state,
    })
}

/// Every temperature the record holds is at least `floor` (the
/// configuration's ambient, under which no model goes): a colder one is
/// damage, not a cold speaker.
pub fn plausible(p: &Persisted, floor: f64) -> bool {
    let ok = |t: f64| t.is_finite() && t >= floor - 1e-6;
    let s = &p.state;
    s.speakers.iter().all(|x| ok(x.t_coil) && ok(x.t_magnet) && ok(x.t_peak))
        && s.asahi.iter().all(|x| ok(x.t_coil) && ok(x.t_magnet) && ok(x.coil_hyst) && ok(x.magnet_hyst) && ok(x.t_peak))
        && s.unified.iter().all(|x| ok(x.ff.0) && ok(x.ff.1) && ok(x.t_peak) && x.vi.is_none_or(|v| ok(v.0) && ok(v.1)))
}

/// The shortest and the longest the time since the record's snapshot can
/// have been, by the clocks both ends trust, never shorter than this boot's
/// `uptime`; or why it cannot be resumed.  `previous` is the boot the
/// journal lists immediately before this one.
pub fn gap(p: &Persisted, now: &Wall, origin: &Origin, previous: Option<&str>, uptime: Duration) -> Result<(Duration, Duration), String> {
    if !p.state.proven() {
        return Err("not a clean stop".into());
    }
    if p.origin != *origin || origin.machine_id.is_none() {
        return Err("written on another machine or under another configuration".into());
    }
    let written = p.wall.boot_id.as_deref().unwrap_or("");
    match (&now.boot_id, previous) {
        (None, _) => return Err("this boot has no id".into()),
        (Some(b), _) if same_boot(b, written) => return Err("written in this boot".into()),
        (_, None) => return Err("the journal does not show the previous boot".into()),
        (_, Some(prev)) if !same_boot(prev, written) => return Err("another boot came between".into()),
        _ => {}
    }
    let (Some(then), Some(at), true, true) = (p.wall.realtime, now.realtime, p.wall.synced, now.synced) else {
        return Err("the system clock is not synchronized at both ends".into());
    };
    let g = at.checked_sub(then).ok_or("the system clock went back")?;
    // A CANCEL_ON_SET watch can miss two overlapping setters whose
    // notifications both inspect the restored offset. Do not use its
    // realtime sample alone as an elapsed-time proof. Require an
    // independent RTC and include its entire uncertainty interval.
    let (Some(a), Some(b)) = (p.wall.rtc, now.rtc) else {
        return Err("idle restore needs an independent RTC at both ends".into());
    };
    let r = b.checked_sub(a).ok_or("the RTC went back")?;
    let (short, long) = (r.min(g), r.max(g));
    if long - short > SLACK + long / 1000 {
        return Err(format!("the RTC and the system clock disagree ({:.1} s against {:.1} s)", r.as_secs_f64(), g.as_secs_f64()));
    }
    // Whole-second quantization at both ends, bounded read placement and
    // the supported RTC rate error of at most 0.1%. Divide by 999 rather
    // than 1000 to cover conversion from RTC elapsed time to true time.
    let uncertainty = SLACK + READ_EPS * 2 + r / 999 + Duration::from_nanos(1);
    let rtc_lo = r.saturating_sub(uncertainty);
    let rtc_hi = r.checked_add(uncertainty).and_then(|v| v.checked_add(p.pairing))
        .ok_or("RTC age interval overflow")?;
    if rtc_hi < uptime || g + SLACK < uptime {
        return Err("the clock interval is shorter than this boot's uptime".into());
    }
    let lo = g.saturating_sub(READ_EPS).min(rtc_lo).max(uptime);
    let hi = g.checked_add(p.pairing).and_then(|v| v.checked_add(READ_EPS))
        .ok_or("realtime age interval overflow")?.max(rtc_hi).max(uptime);
    Ok((lo, hi))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;
    use crate::model::SpeakerState;

    const J316: &str = include_str!("../conf/apple/j316.conf");
    const T: Duration = Duration::from_secs(5000);

    fn clean(t: f64) -> String {
        let cfg = Config::parse(J316).unwrap();
        let sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), t)).collect();
        state::encode(T, T, &sp, false, true)
    }

    fn wall(boot: &str, rt: u64, synced: bool, rtc: Option<u64>) -> Wall {
        Wall { realtime: Some(Duration::from_secs(rt)), synced, rtc: rtc.map(Duration::from_secs), boot_id: Some(boot.into()) }
    }

    fn origin() -> Origin {
        Origin { machine_id: Some("m1".into()), config: 7 }
    }

    #[test]
    fn round_trip_and_damage() {
        let w = wall("a1", 1_800_000_000, true, Some(1_800_000_001));
        let text = encode(&w, &origin(), Duration::from_millis(3), &clean(60.0)).unwrap();
        assert!(decode(&text.replacen(MAGIC, "speakerguardd-boot 2", 1)).is_none());
        let p = decode(&text).unwrap();
        assert_eq!((p.wall.clone(), p.origin.clone(), p.pairing), (w.clone(), origin(), Duration::from_millis(3)));
        assert!(p.state.proven() && plausible(&p, 50.0));
        for n in 0..text.len() {
            assert!(decode(&text[..n]).is_none(), "accepted a prefix of {n} bytes");
        }
        // damage into valid text: a colder coil, a cleared latch
        let i = text.find("speaker ").unwrap();
        let cold = format!("{}{}", &text[..i], text[i..].replacen("60.000000", "-47.598243", 1));
        assert!(decode(&cold).is_none(), "the checksum let a changed temperature through");
        // not written: no synchronized clock, no boot or machine id
        assert!(encode(&wall("a1", 1, false, Some(1)), &origin(), Duration::ZERO, &clean(60.0)).is_none());
        assert!(encode(&Wall { boot_id: None, ..w.clone() }, &origin(), Duration::ZERO, &clean(60.0)).is_none());
        assert!(encode(&w, &Origin { machine_id: None, config: 7 }, Duration::ZERO, &clean(60.0)).is_none());
        // a record colder than the ambient floor is implausible
        let p = decode(&encode(&w, &origin(), Duration::ZERO, &clean(-47.6)).unwrap()).unwrap();
        assert!(!plausible(&p, 50.0));
    }

    #[test]
    fn a_version_change_cannot_promote_an_old_checksum() {
        let good = encode(&wall("old", 1000, true, Some(500)), &origin(), Duration::ZERO, &clean(60.0)).unwrap();
        let body = good.splitn(3, '\n').nth(2).unwrap();
        // A v3 checksum binds its own header. Changing the format header
        // cannot turn that record into a v4 timestamp promise.
        let old = format!("speakerguardd-boot 3\nsum {:016x}\n{body}", fnv(&format!("speakerguardd-boot 3\n{body}")));
        let promoted = old.replacen("speakerguardd-boot 3", MAGIC, 1);
        assert_ne!(old, promoted);
        assert!(decode(&old).is_none());
        assert!(decode(&promoted).is_none(), "the version promise was not covered by the checksum");
        assert!(decode(&good).is_some());
    }

    /// Review of 753f9ca: only the boot just before this one, on the same
    /// machine and configuration, with the system clock synchronized at both
    /// ends (the RTC alone counts whole seconds), no shorter than the
    /// uptime; restored over the shortest and the longest time it can have
    /// been (the clocks were read `pairing` after the snapshot).
    #[test]
    fn only_a_proven_interval_is_resumed() {
        let p = decode(&encode(&wall("a1", 1000, true, Some(500)), &origin(), Duration::from_secs(1), &clean(60.0)).unwrap()).unwrap();
        let up = Duration::from_secs(8);
        let now = wall("b2", 1300, true, Some(800));
        let (lo, hi) = gap(&p, &now, &origin(), Some("a1"), up).unwrap();
        assert!(lo <= Duration::from_secs(300) && lo >= Duration::from_secs(297), "{lo:?}");
        assert!(hi >= Duration::from_secs(301) && hi <= Duration::from_secs(304), "{hi:?}");
        // dashes in one id and not the other are the same boot
        assert!(gap(&p, &now, &origin(), Some("A-1"), up).is_ok());
        for (why, w, o, prev) in [
            ("another boot between", now.clone(), origin(), Some("x9")),
            ("no journal", now.clone(), origin(), None),
            ("this boot", wall("a1", 1300, true, Some(800)), origin(), Some("a1")),
            ("another machine", now.clone(), Origin { machine_id: Some("m2".into()), config: 7 }, Some("a1")),
            ("another configuration", now.clone(), Origin { machine_id: Some("m1".into()), config: 8 }, Some("a1")),
            ("unsynchronized now", wall("b2", 1300, false, Some(800)), origin(), Some("a1")),
            ("RTC alone", Wall { realtime: None, synced: false, ..now.clone() }, origin(), Some("a1")),
            ("clock back", wall("b2", 990, true, Some(490)), origin(), Some("a1")),
            ("RTC back", wall("b2", 1300, true, Some(400)), origin(), Some("a1")),
            ("clocks disagree", wall("b2", 87400, true, Some(800)), origin(), Some("a1")),
            ("under the uptime", wall("b2", 1003, true, Some(503)), origin(), Some("a1")),
        ] {
            assert!(gap(&p, &w, &o, prev, up).is_err(), "{why}");
        }
        // a backward clock with the uptime under the slack (review P6)
        assert!(gap(&p, &wall("b2", 999, true, Some(499)), &origin(), Some("a1"), Duration::from_secs(1)).is_err());
        // a state that is no clean stop
        let cfg = Config::parse(J316).unwrap();
        let sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 60.0)).collect();
        let running = decode(&encode(&wall("a1", 1000, true, Some(500)), &origin(), Duration::ZERO, &state::encode(T, T, &sp, true, false)).unwrap()).unwrap();
        assert!(gap(&running, &now, &origin(), Some("a1"), up).is_err());
    }
    #[test]
    fn absent_rtc_cannot_write_an_idle_restore_promise() {
        assert!(encode(&wall("old", 1000, true, None), &origin(), Duration::ZERO, &clean(60.0)).is_none());
    }

}
