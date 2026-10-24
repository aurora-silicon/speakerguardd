// SPDX-License-Identifier: MIT
//! Cheap counters for how long the daemon is held up, to measure on the
//! machine without a sound: the gaps between renewals of the card's lock
//! (what its 250 ms lease bounds), the latency of each kind of call, and
//! the major page faults.  Summarised to the journal every
//! SUMMARY_INTERVAL, then started again.

use std::time::{Duration, Instant};

/// How often a summary is logged.
pub const SUMMARY_INTERVAL: Duration = Duration::from_secs(600);
/// The histogram's edges (ms): how many took longer than each.
const EDGES_MS: [u64; 5] = [10, 50, 100, 150, 250];

/// A count, the longest, and how many passed each edge.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct Hist {
    pub n: u64,
    pub max: Duration,
    pub over: [u64; EDGES_MS.len()],
}

impl Hist {
    pub fn add(&mut self, d: Duration) {
        self.n += 1;
        self.max = self.max.max(d);
        for (i, &e) in EDGES_MS.iter().enumerate() {
            if d > Duration::from_millis(e) {
                self.over[i] += 1;
            }
        }
    }

    fn text(&self) -> String {
        let over: Vec<String> = EDGES_MS.iter().zip(self.over).filter(|(_, n)| *n > 0).map(|(e, n)| format!(">{e}:{n}")).collect();
        format!("{} max {:.1} ms{}{}", self.n, self.max.as_secs_f64() * 1000.0, if over.is_empty() { "" } else { " " }, over.join(" "))
    }
}

/// The kinds of call that can hold the daemon up.
#[derive(Clone, Copy, Debug)]
pub enum Call {
    /// The sample-rate control read.
    Rate,
    /// The capture's position (its backlog).
    Position,
    /// A volume control write.
    Write,
    /// A lock renewal.
    Ping,
    /// Opening the sense capture.
    Open,
    /// Giving the lock up.
    Relinquish,
    /// Reading the change events.
    Events,
}

const CALLS: [&str; 7] = ["rate", "position", "write", "ping", "open", "relinquish", "events"];

pub struct Stats {
    since: Instant,
    calls: [Hist; CALLS.len()],
    ping_gap: Hist,
    last_ping: Option<Instant>,
    majflt: i64,
}

/// Major page faults of this process so far.
fn major_faults() -> i64 {
    // SAFETY: a valid, writable rusage.
    let mut u: libc::rusage = unsafe { std::mem::zeroed() };
    if unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut u) } == 0 { u.ru_majflt as i64 } else { 0 }
}

impl Stats {
    pub fn new(now: Instant) -> Stats {
        Stats { since: now, calls: [Hist::default(); CALLS.len()], ping_gap: Hist::default(), last_ping: None, majflt: major_faults() }
    }

    pub fn call(&mut self, c: Call, d: Duration) {
        self.calls[c as usize].add(d);
    }

    /// A renewal that took: the gap since the one before, while the lock
    /// was held.
    pub fn renewed(&mut self, at: Instant) {
        if let Some(last) = self.last_ping {
            self.ping_gap.add(at.saturating_duration_since(last));
        }
        self.last_ping = Some(at);
    }

    /// The lock was given up: the next renewal starts a new hold.
    pub fn released(&mut self) {
        self.last_ping = None;
    }

    pub fn due(&self, now: Instant) -> bool {
        now.saturating_duration_since(self.since) >= SUMMARY_INTERVAL
    }

    /// One line for the journal, and the counters started again.
    pub fn summary(&mut self, now: Instant) -> String {
        let faults = major_faults();
        let calls: Vec<String> = CALLS.iter().zip(&self.calls).filter(|(_, h)| h.n > 0).map(|(name, h)| format!("{name} {}", h.text())).collect();
        let line = format!(
            "held-up stats over {:.0} s: renewal gaps {}; calls: {}; major faults {}",
            now.saturating_duration_since(self.since).as_secs_f64(),
            self.ping_gap.text(),
            if calls.is_empty() { "none".to_string() } else { calls.join(", ") },
            faults - self.majflt
        );
        let last_ping = self.last_ping;
        *self = Stats::new(now);
        self.last_ping = last_ping;
        line
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn a_histogram_counts_each_edge_passed() {
        let mut h = Hist::default();
        for d in [5, 20, 120, 300] {
            h.add(ms(d));
        }
        assert_eq!(h.n, 4);
        assert_eq!(h.max, ms(300));
        assert_eq!(h.over, [3, 2, 2, 1, 1]);
        assert_eq!(h.text(), "4 max 300.0 ms >10:3 >50:2 >100:2 >150:1 >250:1");
    }

    #[test]
    fn renewal_gaps_count_only_while_the_lock_is_held() {
        let t = Instant::now();
        let mut s = Stats::new(t);
        s.renewed(t);
        s.renewed(t + ms(100));
        s.renewed(t + ms(260));
        s.released();
        // the next hold starts afresh: no gap across the hand-off
        s.renewed(t + ms(5000));
        s.renewed(t + ms(5100));
        assert_eq!(s.ping_gap.n, 3);
        assert_eq!(s.ping_gap.max, ms(160));
        assert_eq!(s.ping_gap.over[3], 1);
    }

    #[test]
    fn a_summary_says_everything_and_starts_again() {
        let t = Instant::now();
        let mut s = Stats::new(t);
        s.call(Call::Position, ms(3));
        s.call(Call::Position, ms(60));
        s.call(Call::Open, ms(110));
        s.renewed(t);
        s.renewed(t + ms(100));
        assert!(!s.due(t + ms(1000)));
        assert!(s.due(t + SUMMARY_INTERVAL));
        let line = s.summary(t + SUMMARY_INTERVAL);
        assert!(line.starts_with("held-up stats over 600 s: renewal gaps 1 max 100.0 ms >10:1 >50:1;"), "{line}");
        assert!(line.contains("position 2 max 60.0 ms >10:1 >50:1") && line.contains("open 1 max 110.0 ms"), "{line}");
        assert!(line.contains("major faults "), "{line}");
        // started again, the hold kept
        assert_eq!(s.calls[Call::Position as usize].n, 0);
        s.renewed(t + SUMMARY_INTERVAL + ms(100));
        assert_eq!(s.ping_gap.n, 1);
    }
}
