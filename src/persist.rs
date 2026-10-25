// SPDX-License-Identifier: MIT
//! The model's state across a reboot (`boot_resume = idle`; off by
//! default).  The state under /run (state.rs) dies with the boot, so every
//! boot starts at the coils' limit, latched: on the Neo that cost an 11.9
//! dB first lock, released over 7.6 min.  A clean stop made while the
//! system shuts down also writes its state here, under /var/lib, with the
//! boot's id, CLOCK_REALTIME (and whether the kernel called it
//! synchronized) and the RTC.  The first start of the next boot resumes it,
//! charging the whole time since at the worst case the card's own safe
//! level lets through, as a restart within a boot does.
//!
//! Only such a state is resumed: after a clean stop at shutdown nothing in
//! that boot can drive the speakers again (no restart, no OnFailure
//! fallback), and the card holds its safe level to the power-off.  A
//! periodic state is never resumed: after it the daemon, or speakersafetyd
//! after it, may have driven the speakers at their limit up to the reboot,
//! whose time no clock records.  Every start consumes the file before the
//! lock can be lifted, so a state is resumed at most once and never after
//! this boot has driven the speakers.
//!
//! **What Linux cannot see.**  The gap between the shutdown and this boot
//! is taken to be idle at the card's safe level.  On a machine that also
//! boots macOS (or anything else that drives the same speakers), another
//! OS may have run in that gap and left a coil at its limit; this boot
//! would then start from the cold state saved before it and could let a
//! coil run up to (limit - saved temperature) hotter than its limit.
//! Nothing Linux can read proves no other OS ran, so `idle` is for
//! machines that boot only Linux; the default keeps the latched start.
//!
//! The gap is measured by every trustworthy clock and the shortest is
//! taken: the RTC (it runs through the power-off) and CLOCK_REALTIME (only
//! if the kernel called it synchronized at both ends, unless the RTC
//! confirms it).  Clocks that went back, disagree by more than 2 s plus
//! 0.1 %, or measure less than this boot's own uptime, are not trusted: the
//! start is latched.

use std::time::Duration;

use crate::state::{self, Saved};

const MAGIC: &str = "speakerguardd-boot 1";
/// The RTC counts whole seconds, and two clocks are read a little apart.
const SLACK: Duration = Duration::from_secs(2);

/// The clocks and the boot, as read now.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Wall {
    pub realtime: Option<Duration>,
    /// The kernel called CLOCK_REALTIME synchronized.
    pub synced: bool,
    pub rtc: Option<Duration>,
    pub boot_id: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Persisted {
    pub wall: Wall,
    pub state: Saved,
}

/// The file for `state` (a clean stop's state text) written at `wall`, or
/// None if no clock could time the gap or the boot has no id.
pub fn encode(wall: &Wall, state: &str) -> Option<String> {
    let boot_id = wall.boot_id.as_deref().filter(|b| !b.is_empty() && !b.contains(char::is_whitespace))?;
    if wall.rtc.is_none() && !(wall.realtime.is_some() && wall.synced) {
        return None;
    }
    let opt = |d: Option<Duration>| d.map_or("-".to_string(), |d| d.as_nanos().to_string());
    Some(format!("{MAGIC}\nboot_id {boot_id}\nrealtime_ns {}\nsynced {}\nrtc_ns {}\n{state}", opt(wall.realtime), wall.synced as u8, opt(wall.rtc)))
}

pub fn decode(text: &str) -> Option<Persisted> {
    let mut lines = text.splitn(6, '\n');
    if lines.next()? != MAGIC {
        return None;
    }
    let boot_id = lines.next()?.strip_prefix("boot_id ")?.to_string();
    let ns = |v: &str| -> Option<Option<Duration>> {
        if v == "-" {
            Some(None)
        } else {
            v.parse::<u64>().ok().map(|n| Some(Duration::from_nanos(n)))
        }
    };
    let realtime = ns(lines.next()?.strip_prefix("realtime_ns ")?)?;
    let synced = match lines.next()?.strip_prefix("synced ")? {
        "0" => false,
        "1" => true,
        _ => return None,
    };
    let rtc = ns(lines.next()?.strip_prefix("rtc_ns ")?)?;
    let state = state::decode(lines.next()?)?;
    if boot_id.is_empty() || boot_id.contains(char::is_whitespace) {
        return None;
    }
    Some(Persisted { wall: Wall { realtime, synced, rtc, boot_id: Some(boot_id) }, state })
}

/// The time since the file's clean stop, by the clocks both ends trust, at
/// least this boot's `uptime`; or why it cannot be resumed.
pub fn gap(p: &Persisted, now: &Wall, uptime: Duration) -> Result<Duration, String> {
    if !p.state.proven() {
        return Err("not a clean stop".into());
    }
    match (&p.wall.boot_id, &now.boot_id) {
        (Some(a), Some(b)) if a != b => {}
        (_, None) => return Err("this boot has no id".into()),
        _ => return Err("written in this boot".into()),
    }
    let since = |a: Option<Duration>, b: Option<Duration>| match (a, b) {
        (Some(a), Some(b)) => Some(b.checked_sub(a).ok_or(())),
        _ => None,
    };
    let rtc = since(p.wall.rtc, now.rtc);
    let rt = since(p.wall.realtime, now.realtime).filter(|_| rtc.is_some() || (p.wall.synced && now.synced));
    let gaps: Vec<Duration> = match (rtc, rt) {
        (None, None) => return Err("no clock both boots trust".into()),
        (Some(Err(())), _) | (_, Some(Err(()))) => return Err("a clock went back".into()),
        (Some(Ok(a)), Some(Ok(b))) => {
            let (lo, hi) = (a.min(b), a.max(b));
            if hi - lo > SLACK + hi / 1000 {
                return Err(format!("the RTC and the system clock disagree ({:.1} s against {:.1} s)", a.as_secs_f64(), b.as_secs_f64()));
            }
            vec![a, b]
        }
        (Some(Ok(a)), None) | (None, Some(Ok(a))) => vec![a],
    };
    let g = gaps.into_iter().min().unwrap_or_default();
    if g + SLACK < uptime {
        return Err(format!("{:.1} s since the stop, under this boot's uptime of {:.1} s", g.as_secs_f64(), uptime.as_secs_f64()));
    }
    Ok(g.max(uptime))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;
    use crate::model::SpeakerState;

    const J316: &str = include_str!("../conf/apple/j316.conf");
    const T: Duration = Duration::from_secs(5000);

    fn clean() -> String {
        let cfg = Config::parse(J316).unwrap();
        let sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 60.0)).collect();
        state::encode(T, T, &sp, false, true)
    }

    fn wall(boot: &str, rt: u64, synced: bool, rtc: Option<u64>) -> Wall {
        Wall { realtime: Some(Duration::from_secs(rt)), synced, rtc: rtc.map(Duration::from_secs), boot_id: Some(boot.into()) }
    }

    #[test]
    fn round_trip_and_damage() {
        let w = wall("a1", 1_800_000_000, true, Some(1_800_000_001));
        let text = encode(&w, &clean()).unwrap();
        let p = decode(&text).unwrap();
        assert_eq!(p.wall, w);
        assert!(p.state.proven());
        for n in 0..text.rfind("\nend").unwrap() + 4 {
            assert!(decode(&text[..n]).is_none(), "accepted a prefix of {n} bytes");
        }
        assert!(decode(&text.replace("synced 1", "synced 2")).is_none());
        // nothing to time the gap by: not written
        assert!(encode(&wall("a1", 1, false, None), &clean()).is_none());
        assert!(encode(&Wall { boot_id: None, ..w.clone() }, &clean()).is_none());
    }

    /// The gap is the shortest trusted clock's, and never less than this
    /// boot's uptime; anything a clock cannot vouch for is latched.
    #[test]
    fn only_trusted_clocks_time_the_gap() {
        let p = |w: Wall| decode(&encode(&w, &clean()).unwrap()).unwrap();
        let up = Duration::from_secs(8);
        let then = p(wall("a1", 1000, false, Some(500)));
        // the RTC alone (the system clock unsynchronized at boot)
        assert_eq!(gap(&then, &wall("b2", 1300, false, Some(800)), up), Ok(Duration::from_secs(300)));
        // the shorter of two that agree
        assert_eq!(gap(&then, &wall("b2", 1299, false, Some(800)), up), Ok(Duration::from_secs(299)));
        // the same boot, no boot id, a clock gone back, two that disagree
        assert!(gap(&then, &wall("a1", 1300, false, Some(800)), up).is_err());
        assert!(gap(&then, &Wall { boot_id: None, ..wall("b2", 1300, false, Some(800)) }, up).is_err());
        assert!(gap(&then, &wall("b2", 1300, false, Some(400)), up).is_err());
        assert!(gap(&then, &wall("b2", 9300, false, Some(800)), up).is_err());
        // a gap shorter than this boot has been up
        assert!(gap(&then, &wall("b2", 1003, false, Some(503)), up).is_err());
        // the system clock alone: only synchronized at both ends
        let rt = p(wall("a1", 1000, true, None));
        assert_eq!(gap(&rt, &wall("b2", 1600, true, None), up), Ok(Duration::from_secs(600)));
        assert!(gap(&rt, &wall("b2", 1600, false, None), up).is_err());
        // a state that is no clean stop
        let cfg = Config::parse(J316).unwrap();
        let sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 60.0)).collect();
        let running = decode(&encode(&wall("a1", 1000, false, Some(500)), &state::encode(T, T, &sp, true, false)).unwrap()).unwrap();
        assert!(gap(&running, &wall("b2", 1300, false, Some(800)), up).is_err());
    }
}
