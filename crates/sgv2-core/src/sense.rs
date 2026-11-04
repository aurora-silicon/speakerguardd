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
    out_env: [f32; HIST],
    v_env: [f32; HIST],
    pos: usize,
    acc_out: f32,
    acc_v: f32,
    acc_n: usize,
    prev_out: f32,
    prev_v: f32,
    corr: [f32; MAX_LAG],
    filled: usize,
    candidate: Option<usize>,
    stable: u32,
    locked: Option<u32>,
}

impl Default for DelayEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl DelayEstimator {
    pub const fn new() -> Self {
        DelayEstimator {
            out_env: [0.0; HIST],
            v_env: [0.0; HIST],
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
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// The accepted delay in frames.
    pub fn delay(&self) -> Option<u32> {
        self.locked
    }

    /// Feed one frame-aligned run: `out_energy[n]` is the unit output's
    /// energy summed over channels, `v_energy[n]` the sensed voltage's.
    /// Called with per-sample energies by the unit (`push_run`).
    #[inline]
    pub fn push(&mut self, out_e: f32, v_e: f32) {
        self.acc_out += out_e;
        self.acc_v += v_e;
        self.acc_n += 1;
        if self.acc_n == ENV {
            self.point(self.acc_out, self.acc_v);
            self.acc_out = 0.0;
            self.acc_v = 0.0;
            self.acc_n = 0;
        }
    }

    fn point(&mut self, o_level: f32, v_level: f32) {
        let o = o_level - self.prev_out;
        let v = v_level - self.prev_v;
        self.prev_out = o_level;
        self.prev_v = v_level;
        self.pos = (self.pos + 1) % HIST;
        self.out_env[self.pos] = o;
        self.v_env[self.pos] = v;
        self.filled = (self.filled + 1).min(HIST);
        // corr[L] tracks sum over k of v[k] * out[k - L]
        for (l, c) in self.corr.iter_mut().enumerate() {
            let k = (self.pos + HIST - l) % HIST;
            *c = *c * LAMBDA + v * self.out_env[k];
        }
    }

    /// Once per unit block: update the lock.
    pub fn end_block(&mut self) {
        if self.filled <= MAX_LAG {
            return;
        }
        let mut best = 0;
        let mut sum = 0.0f32;
        for (l, &c) in self.corr.iter().enumerate() {
            sum += c.abs();
            if c > self.corr[best] {
                best = l;
            }
        }
        let mean = sum / MAX_LAG as f32;
        let peaked = self.corr[best] > 0.0 && self.corr[best] > PEAK_RATIO * mean;
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
}
