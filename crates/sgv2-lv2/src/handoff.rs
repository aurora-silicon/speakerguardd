//! Process-global thermal hand-off between streams and instances.
//!
//! A speaker stays hot across a pause, a suspend or a filter-chain rebuild.
//! PipeWire deactivates and reactivates the graph on those, and LV2 asks a
//! plugin to forget its history on `activate`, but forgetting the thermal
//! model gives a hot speaker full power back. macOS keeps the spp3 state
//! vector process-global and restores it aged for the gap (spp3 spec 10);
//! this is that record for every unit.
//!
//! `run` publishes the unit's state after every call into one slot, a
//! seqlock of atomics: no lock, no allocation, no system call (the
//! monotonic clock is read through the vDSO), and a publish that finds
//! another writer in the slot is skipped, never waited for. `activate`
//! (not real time) resets the unit and restores the latest record that
//! fits it, aged by the time since it was published. A new instance in the
//! same process therefore starts from the state the previous one left.
//! State does not survive the process.

use std::sync::atomic::{fence, AtomicU64, Ordering};
use std::sync::OnceLock;
use std::time::Instant;

use sgv2_core::{HandOff, HANDOFF_NODES, MAX_CHANNELS};

const HEADER: usize = 4;
const WORDS: usize = HEADER + MAX_CHANNELS * HANDOFF_NODES + MAX_CHANNELS;

/// One hand-off record: kind, channels, rate, publish time (ns since the
/// process epoch), then the nodes and gains as f64 bits.
pub struct Slot {
    /// Even: stable; odd: a writer is inside; 0: never written.
    seq: AtomicU64,
    words: [AtomicU64; WORDS],
}

impl Default for Slot {
    fn default() -> Self {
        Self::new()
    }
}

static EPOCH: OnceLock<Instant> = OnceLock::new();

/// The process's slot.
pub static SLOT: Slot = Slot::new();

/// Fix the clock epoch. Call off the RT thread (`instantiate`) before the
/// first `publish`; until then publishing is a no-op.
pub fn init() {
    EPOCH.get_or_init(Instant::now);
}

fn now_ns() -> Option<u64> {
    EPOCH.get().map(|e| u64::try_from(e.elapsed().as_nanos()).unwrap_or(u64::MAX))
}

impl Slot {
    pub const fn new() -> Self {
        Slot { seq: AtomicU64::new(0), words: [const { AtomicU64::new(0) }; WORDS] }
    }

    /// Real-time safe: never blocks. Skipped if the clock has no epoch yet
    /// or another writer holds the slot.
    pub fn publish(&self, s: &HandOff) {
        let Some(t) = now_ns() else { return };
        self.publish_at(s, t);
    }

    fn publish_at(&self, s: &HandOff, t_ns: u64) {
        let seq = self.seq.load(Ordering::Relaxed);
        if seq & 1 == 1 {
            return;
        }
        if self.seq.compare_exchange(seq, seq.wrapping_add(1), Ordering::Acquire, Ordering::Relaxed).is_err() {
            return;
        }
        fence(Ordering::Release);
        let w = &self.words;
        w[0].store(s.kind as u64, Ordering::Relaxed);
        w[1].store(s.channels as u64, Ordering::Relaxed);
        w[2].store(s.sample_rate as u64, Ordering::Relaxed);
        w[3].store(t_ns, Ordering::Relaxed);
        for (word, v) in w[HEADER..].iter().zip(s.node.iter().flatten().chain(&s.gain)) {
            word.store(v.to_bits(), Ordering::Relaxed);
        }
        // never 0 again once written, so 0 keeps meaning "empty"
        self.seq.store(seq.wrapping_add(2).max(2), Ordering::Release);
    }

    /// The latest record and its age in seconds, or `None` if nothing was
    /// ever published (or a writer kept the slot busy).
    pub fn latest(&self) -> Option<(HandOff, f64)> {
        let now = now_ns()?;
        self.latest_at(now)
    }

    fn latest_at(&self, now_ns: u64) -> Option<(HandOff, f64)> {
        for _ in 0..64 {
            let s1 = self.seq.load(Ordering::Acquire);
            if s1 == 0 {
                return None;
            }
            if s1 & 1 == 1 {
                std::hint::spin_loop();
                continue;
            }
            let w = &self.words;
            let mut h = HandOff::EMPTY;
            h.kind = w[0].load(Ordering::Relaxed) as u32;
            h.channels = w[1].load(Ordering::Relaxed) as u32;
            h.sample_rate = w[2].load(Ordering::Relaxed) as u32;
            let t = w[3].load(Ordering::Relaxed);
            for (v, word) in h.node.iter_mut().flatten().chain(h.gain.iter_mut()).zip(&w[HEADER..]) {
                *v = f64::from_bits(word.load(Ordering::Relaxed));
            }
            fence(Ordering::Acquire);
            if self.seq.load(Ordering::Relaxed) == s1 {
                return Some((h, now_ns.saturating_sub(t) as f64 * 1e-9));
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_record_round_trips_with_its_age() {
        let slot = Slot::new();
        assert!(slot.latest_at(5).is_none());
        let mut h = HandOff::EMPTY;
        h.kind = 7;
        h.channels = 2;
        h.sample_rate = 48_000;
        h.node[1][2] = 42.5;
        h.gain[1] = 0.25;
        slot.publish_at(&h, 1_000_000_000);
        let (got, age) = slot.latest_at(3_500_000_000).unwrap();
        assert_eq!(got, h);
        assert!((age - 2.5).abs() < 1e-12);
        // a later publish replaces it; a clock behind the record is age 0
        h.node[0][0] = 1.0;
        slot.publish_at(&h, 4_000_000_000);
        let (got, age) = slot.latest_at(3_000_000_000).unwrap();
        assert_eq!((got.node[0][0], age), (1.0, 0.0));
    }

    #[test]
    fn a_busy_slot_skips_the_publish_and_never_blocks() {
        let slot = Slot::new();
        let h = HandOff::EMPTY;
        slot.publish_at(&h, 1);
        slot.seq.fetch_add(1, Ordering::SeqCst); // a writer is inside
        let mut other = h;
        other.channels = 5;
        slot.publish_at(&other, 2);
        assert!(slot.latest_at(3).is_none()); // reader gives up, does not spin for ever
        slot.seq.fetch_add(1, Ordering::SeqCst);
        assert_eq!(slot.latest_at(3).unwrap().0.channels, 0);
    }
}
