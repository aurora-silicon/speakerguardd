//! V/I sense front end: per-block statistics of the telemetry bus and an
//! estimate of the program-to-sense delay.
//!
//! The sense capture runs on the same cluster clock as the speaker
//! playback, in the same PipeWire driver group, so each host cycle delivers
//! program and sense quanta of equal length with a constant offset `D`
//! between "frame left the unit" and "its V/I arrived". `D` is a property of
//! the configuration (playback queue plus capture period), not of the
//! signal, and changes only across a stream restart or an xrun.
//!
//! What uses what:
//! - Pilot-tone resistance thermometry (spp3 closed loop) needs only V and
//!   I of the *same* frames, so it does not depend on `D` at all.
//! - Comparing modelled with measured power needs `D` to about a block, not
//!   to a sample: the thermal time constants are seconds.
//!
//! The estimator therefore works on energy envelopes at `ENV` frames per
//! point. It takes their first differences (which removes the level and
//! leaves the onsets), and correlates the unit's output energy (summed over
//! channels) with the sensed voltage energy (summed over channels) at every
//! lag up to
//! `MAX_LAG` points, with exponential forgetting, and reports the argmax
//! once it has held still for `LOCK_BLOCKS` consecutive blocks. Cost: one
//! multiply-add per lag per envelope point, about 49k per 512-frame block.
//! These constants are signal-processing settings of the estimator, not
//! protection parameters: nothing in the protection path depends on them
//! while the closed loop is off.

use crate::MAX_CHANNELS;

/// Frames per envelope point.
pub const ENV: usize = 32;
/// Lags examined, in envelope points (8192 frames).
pub const MAX_LAG: usize = 256;
/// Envelope history kept (must exceed MAX_LAG).
const HIST: usize = 512;
/// Forgetting factor per envelope point (about 0.7 s memory at 48 kHz).
const LAMBDA: f32 = 0.999;
/// Blocks the argmax must hold still to be accepted.
const LOCK_BLOCKS: u32 = 16;
/// Accepted peak must exceed the mean correlation by this factor.
const PEAK_RATIO: f32 = 4.0;

/// Per-block sums over the raw (unscaled) sense samples.
#[derive(Clone, Copy, Debug, Default)]
pub struct BlockSums {
    pub i_sq: [f64; MAX_CHANNELS],
    pub v_sq: [f64; MAX_CHANNELS],
    pub vi: [f64; MAX_CHANNELS],
    pub frames: usize,
    /// Every sense sample in the block was present and finite.
    pub complete: bool,
}

impl BlockSums {
    pub fn clear(&mut self) {
        *self = BlockSums { complete: true, ..Default::default() };
    }
}

pub struct DelayEstimator {
    /// Differenced output envelope, written twice (at `pos` and
    /// `pos + HIST`) so every lag window is one contiguous slice.
    out_env: [f32; 2 * HIST],
    pos: usize,
    acc_out: f32,
    acc_v: f32,
    acc_n: usize,
    prev_out: f32,
    prev_v: f32,
    /// `corr[j]` is the correlation at lag `MAX_LAG - 1 - j`.
    corr: [f32; MAX_LAG],
    filled: usize,
    candidate: Option<usize>,
    stable: u32,
    locked: Option<u32>,
    /// The current envelope point holds a non-finite sum.
    poisoned: bool,
}

impl Default for DelayEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl DelayEstimator {
    pub const fn new() -> Self {
        DelayEstimator {
            out_env: [0.0; 2 * HIST],
            pos: 0,
            acc_out: 0.0,
            acc_v: 0.0,
            acc_n: 0,
            prev_out: 0.0,
            prev_v: 0.0,
            corr: [0.0; MAX_LAG],
            filled: 0,
            candidate: None,
            stable: 0,
            locked: None,
            poisoned: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Forget the correlation evidence but keep the accepted delay.
    fn restart(&mut self) {
        let locked = self.locked;
        *self = Self::new();
        self.locked = locked;
    }

    /// The accepted delay in frames.
    pub fn delay(&self) -> Option<u32> {
        self.locked
    }

    /// Frames still missing from the current envelope point.
    #[inline]
    pub fn room(&self) -> usize {
        ENV - self.acc_n
    }

    /// Feed `frames` frames (at most `room()`) whose output energy, summed
    /// over channels, is `out_e`, and whose sensed voltage energy, summed
    /// over channels, is `v_e`.
    #[inline]
    pub fn push_sums(&mut self, out_e: f32, v_e: f32, frames: usize) {
        // a non-finite point would stay in the forgotten correlations for
        // good: the point that holds one restarts the correlation instead
        // (at the point boundary, so how the frames arrived does not
        // matter; the lock is kept, the delay is a property of the
        // configuration)
        if out_e.is_finite() && v_e.is_finite() {
            self.acc_out += out_e;
            self.acc_v += v_e;
        } else {
            self.poisoned = true;
        }
        self.acc_n += frames;
        if self.acc_n >= ENV {
            if self.poisoned {
                self.restart();
                return;
            }
            self.point(self.acc_out, self.acc_v);
            self.acc_out = 0.0;
            self.acc_v = 0.0;
            self.acc_n = 0;
        }
    }

    /// One frame (tests and scalar callers).
    #[inline]
    pub fn push(&mut self, out_e: f32, v_e: f32) {
        self.push_sums(out_e, v_e, 1);
    }

    fn point(&mut self, o_level: f32, v_level: f32) {
        let o = o_level - self.prev_out;
        let v = v_level - self.prev_v;
        self.prev_out = o_level;
        self.prev_v = v_level;
        self.pos = (self.pos + 1) % HIST;
        self.out_env[self.pos] = o;
        self.out_env[self.pos + HIST] = o;
        self.filled = (self.filled + 1).min(HIST);
        // corr at lag L += v[k] * out[k - L]; window[j] = out[k - (MAX_LAG-1-j)]
        let end = self.pos + HIST + 1;
        let window = &self.out_env[end - MAX_LAG..end];
        for (c, &w) in self.corr.iter_mut().zip(window) {
            *c = *c * LAMBDA + v * w;
        }
    }

    /// Once per unit block: update the lock.
    pub fn end_block(&mut self) {
        if self.filled <= MAX_LAG {
            return;
        }
        let mut best_j = MAX_LAG - 1;
        let mut sum = 0.0f32;
        // scan from lag 0 upward so ties resolve to the shortest lag
        for j in (0..MAX_LAG).rev() {
            let c = self.corr[j];
            sum += c.abs();
            if c > self.corr[best_j] {
                best_j = j;
            }
        }
        if !sum.is_finite() {
            // finite inputs whose products overflowed
            self.restart();
            return;
        }
        let best = MAX_LAG - 1 - best_j;
        let mean = sum / MAX_LAG as f32;
        let peak = self.corr[best_j];
        let peaked = peak > 0.0 && peak > PEAK_RATIO * mean;
        if peaked && self.candidate == Some(best) {
            self.stable = self.stable.saturating_add(1);
        } else {
            self.candidate = if peaked { Some(best) } else { None };
            self.stable = 0;
        }
        // Silence keeps the previous lock (no evidence either way); a
        // different peak replaces it only once it has held still.
        if self.stable >= LOCK_BLOCKS {
            self.locked = Some((best * ENV) as u32);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_a_fixed_delay_in_noise_bursts() {
        let d = 1536usize; // three 512 blocks, a typical playback queue
        let n = 48_000 * 4;
        let mut s = 1u32;
        let mut prog = std::vec::Vec::with_capacity(n);
        for k in 0..n {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            let env = if (k / 2400) % 3 == 0 { 1.0 } else { 0.1 };
            prog.push(((s >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * env);
        }
        let mut e = DelayEstimator::new();
        for k in 0..n {
            let o = prog[k] * prog[k];
            let v = if k >= d { 0.7 * prog[k - d] * prog[k - d] } else { 0.0 };
            e.push(o, v);
            if k % 512 == 511 {
                e.end_block();
            }
        }
        assert_eq!(e.delay(), Some(d as u32));
    }

    #[test]
    fn silence_never_locks() {
        let mut e = DelayEstimator::new();
        for k in 0..48_000 * 2 {
            e.push(0.0, 0.0);
            if k % 512 == 511 {
                e.end_block();
            }
        }
        assert_eq!(e.delay(), None);
    }

    /// Synthetic noise bursts: the per-frame program energy, and the sensed
    /// voltage energy, which is the program's delayed by `d` frames.
    fn delayed_bursts(d: usize, n: usize) -> (std::vec::Vec<f32>, std::vec::Vec<f32>) {
        let mut s = 1u32;
        let mut o = std::vec::Vec::with_capacity(n);
        for k in 0..n {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            let env = if (k / 2400) % 3 == 0 { 1.0 } else { 0.1 };
            let x = ((s >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * env;
            o.push(x * x);
        }
        let v = (0..n).map(|k| if k >= d { 0.7 * o[k - d] } else { 0.0 }).collect();
        (o, v)
    }

    fn feed(e: &mut DelayEstimator, o: &[f32], v: &[f32]) {
        for k in 0..o.len() {
            e.push(o[k], v[k]);
            if k % 512 == 511 {
                e.end_block();
            }
        }
    }

    fn locked_at(d: usize) -> DelayEstimator {
        let (o, v) = delayed_bursts(d, 48_000 * 4);
        let mut e = DelayEstimator::new();
        feed(&mut e, &o, &v);
        assert_eq!(e.delay(), Some(d as u32));
        e
    }

    #[test]
    fn a_non_finite_point_restarts_the_evidence_but_keeps_the_lock() {
        for (bad_o, bad_v) in [(f32::NAN, 0.0), (0.0, f32::INFINITY), (f32::NEG_INFINITY, f32::NAN)] {
            let mut e = locked_at(1024);
            e.push(1.0, 1.0);
            e.push(bad_o, bad_v);
            // the poison is only acted on at the point boundary
            assert!(e.poisoned && e.filled > 0);
            while e.room() > 1 {
                e.push(1.0, 1.0);
            }
            e.push(1.0, 1.0);
            assert!(!e.poisoned, "the poisoned point is discarded");
            assert_eq!(e.filled, 0, "the correlation restarts");
            assert!(e.corr.iter().all(|c| *c == 0.0));
            assert_eq!(e.delay(), Some(1024), "the configuration's delay is kept");
            // the restarted estimator relocks at a new delay from clean evidence
            let (o, v) = delayed_bursts(512, 48_000 * 4);
            feed(&mut e, &o, &v);
            assert_eq!(e.delay(), Some(512));
            assert!(e.corr.iter().all(|c| c.is_finite()));
        }
    }

    #[test]
    fn finite_inputs_that_overflow_the_correlation_restart_it_and_keep_the_lock() {
        let mut e = locked_at(1536);
        for _ in 0..ENV * 4 {
            e.push(f32::MAX, f32::MAX);
        }
        e.end_block();
        assert_eq!(e.filled, 0);
        assert!(e.corr.iter().all(|c| *c == 0.0));
        assert_eq!(e.delay(), Some(1536));
    }

    #[test]
    fn silence_after_a_lock_keeps_it_and_reset_forgets_it() {
        let mut e = locked_at(1536);
        feed(&mut e, &[0.0; 48_000], &[0.0; 48_000]);
        assert_eq!(e.delay(), Some(1536), "no evidence either way");
        e.reset();
        assert_eq!(e.delay(), None);
        assert_eq!(e.room(), ENV);
    }

    #[test]
    fn frames_fed_in_sums_match_frames_fed_one_by_one() {
        let (o, v) = delayed_bursts(768, 48_000);
        let mut one = DelayEstimator::new();
        let mut sums = DelayEstimator::new();
        let mut k = 0;
        // uneven chunks, each at most `room()` frames
        let mut step = 1;
        while k < o.len() {
            let n = step.min(sums.room()).min(o.len() - k);
            let (so, sv) = (o[k..k + n].iter().sum::<f32>(), v[k..k + n].iter().sum::<f32>());
            sums.push_sums(so, sv, n);
            for j in k..k + n {
                one.push(o[j], v[j]);
            }
            assert_eq!(sums.room(), one.room());
            k += n;
            step = step % 13 + 1;
        }
        assert_eq!(sums.filled, one.filled);
        for (a, b) in sums.corr.iter().zip(&one.corr) {
            assert!((a - b).abs() <= 1e-3 * b.abs().max(1e-6), "{a} vs {b}");
        }
    }

    #[test]
    fn block_sums_clear_to_an_empty_complete_block() {
        let mut b = BlockSums {
            i_sq: [1.0; MAX_CHANNELS],
            v_sq: [2.0; MAX_CHANNELS],
            vi: [3.0; MAX_CHANNELS],
            frames: 7,
            complete: false,
        };
        b.clear();
        assert!(b.complete);
        assert_eq!(b.frames, 0);
        assert!(b.i_sq.iter().chain(&b.v_sq).chain(&b.vi).all(|x| *x == 0.0));
    }
}
