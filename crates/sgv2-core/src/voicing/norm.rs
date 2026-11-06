//! Norm: the stereo loudness normalizer at the head of the speaker program.
//!
//! One broadband gain, shared by every channel, steers the program's
//! loudness toward the tuning's target. The audio path is a fixed
//! look-ahead delay and that gain; everything else is the side chain.
//!
//! Side chain, per frame: each channel through the detector cascade (the
//! loudness weighting; the unit's side-chain high-pass only feeds its
//! loudness meters, not this detector), squared, and
//! the channels summed (a full and a silent channel read the same as the
//! full one alone; two equal channels read 3 dB louder). The sum is
//! accumulated over 64-frame steps; eight steps make a 512-frame block and
//! eight blocks a 4096-frame block. The counters run on the stream, not on
//! the host's calls.
//!
//! Meters (levels are `10 log10(power)` of the smoothed detector power; the
//! histogram works in loudness units, that minus 0.691):
//! - fast: the loudest 64-frame level of the call, less a fixed store;
//! - median: the lower median of the last 94 512-frame levels, less a fixed
//!   store, behind a release limiter that only lets go once the median sits
//!   more than its dead band under it;
//! - long: a 1001-bin histogram of 4096-frame window levels over -100..0 in
//!   0.1 dB bins, read as the power mean of the bins above a relative gate
//!   that trails the histogram's own smoothed energy. Every admitted window
//!   fades all counts by a factor whose time constant shortens as the counts
//!   lean below the reading, so the history spans seconds, not the whole
//!   program. A reset clears it and seeds the eleven bins around the
//!   expected loudness with one count each.
//! - the long reading passes a preconditioner against the 512-frame level
//!   (less its own store): it follows the reading while that level is above
//!   it and, once the reading overtakes it, holds the level it overtook.
//!
//! The call's candidate is the larger of fast and of median against the
//! conditioned long path; it is read once per call.
//!
//! Gain law: a 21-node table over -100..+10 in 5.5 dB steps, compiled from
//! the tuning (target, maximum gain, upward and downward ranges, ratio),
//! read with linear interpolation. The gain glides toward its target by a
//! tenth of the remaining distance per frame, in dB.
//!
//! No tuning value lives here; `aurora-speaker-tuning`'s `voicing::norm`
//! compiles [`Params`] from the Mac's saved state.

use alloc::boxed::Box;
use alloc::vec;

/// Gain-table nodes and their grid.
pub const NODES: usize = 21;
pub const NODE_LO_DB: f64 = -100.0;
pub const NODE_STEP_DB: f64 = 5.5;
/// Detector sections per channel, at most.
pub const MAX_SECTIONS: usize = 4;
/// Most channels one box runs.
pub const MAX_CHANNELS: usize = 2;
/// Longest look-ahead accepted, frames (100 ms at 48 kHz).
pub const MAX_LOOKAHEAD: usize = 4800;

/// Loudness offset of the power-to-loudness transform.
pub const LOUDNESS_OFFSET: f64 = -0.691;
const STEP: usize = 64;
const PER_BLOCK: usize = 8;
const PER_LONG: usize = 8;
const BINS: usize = 1001;
const BIN_LO_DB: f64 = -100.0;
const BIN_DB: f64 = 0.1;
const SEED_HALF: usize = 5;
const GLIDE: f64 = 0.1;
/// macOS renders the speaker graph in calls of this many frames; the unit
/// meters a whole call before it moves its gain across that call.
pub const CALL: usize = 512;

#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    /// Gain at each node, dB.
    pub table: [f64; NODES],
    /// Loudness the meters assume after a reset.
    pub expected_db: f64,
    /// Silence after which the meters reset, s (0: never).
    pub reset_timeout_s: f64,
    /// Audio delay ahead of the gain, frames.
    pub lookahead: usize,
    /// Detector cascade, `[b0, b1, b2, a1, a2]` per section.
    pub sections: usize,
    pub sos: [[f64; 5]; MAX_SECTIONS],
}

impl Default for Params {
    fn default() -> Self {
        Params {
            table: [0.0; NODES],
            expected_db: -20.0,
            reset_timeout_s: 0.0,
            lookahead: 0,
            sections: 0,
            sos: [[0.0; 5]; MAX_SECTIONS],
        }
    }
}

impl Params {
    pub const WORDS: usize = NODES + 4 + 5 * MAX_SECTIONS;

    pub fn to_words(&self, out: &mut [f64]) {
        out[..NODES].copy_from_slice(&self.table);
        out[NODES] = self.expected_db;
        out[NODES + 1] = self.reset_timeout_s;
        out[NODES + 2] = self.lookahead as f64;
        out[NODES + 3] = self.sections as f64;
        for (i, s) in self.sos.iter().enumerate() {
            out[NODES + 4 + 5 * i..NODES + 9 + 5 * i].copy_from_slice(s);
        }
    }

    pub fn from_words(w: &[f64]) -> Option<Params> {
        if w.len() != Self::WORDS || w.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let mut p = Params::default();
        p.table.copy_from_slice(&w[..NODES]);
        p.expected_db = w[NODES];
        p.reset_timeout_s = w[NODES + 1];
        let la = w[NODES + 2];
        let ns = w[NODES + 3];
        if la < 0.0 || la > MAX_LOOKAHEAD as f64 || la != libm::trunc(la) {
            return None;
        }
        if ns < 0.0 || ns > MAX_SECTIONS as f64 || ns != libm::trunc(ns) {
            return None;
        }
        p.lookahead = la as usize;
        p.sections = ns as usize;
        for (i, s) in p.sos.iter_mut().enumerate() {
            s.copy_from_slice(&w[NODES + 4 + 5 * i..NODES + 9 + 5 * i]);
        }
        p.valid().then_some(p)
    }

    fn valid(&self) -> bool {
        let table_ok = self.table.iter().all(|g| (-60.0..=60.0).contains(g));
        let expected_ok = (-100.0..=0.0).contains(&self.expected_db);
        let timeout_ok = (0.0..=3600.0).contains(&self.reset_timeout_s);
        // every section stable (poles inside the unit circle)
        let stable = self.sos[..self.sections].iter().all(|s| {
            let (a1, a2) = (s[3], s[4]);
            a2.abs() < 1.0 && a1.abs() < 1.0 + a2
        });
        let unused_zero = self.sos[self.sections..].iter().all(|s| s.iter().all(|v| *v == 0.0));
        table_ok && expected_ok && timeout_ok && stable && unused_zero
    }

    /// The table's gain for loudness `l`, dB.
    pub fn gain_db(&self, l: f64) -> f64 {
        let pos = ((l - NODE_LO_DB) / NODE_STEP_DB).clamp(0.0, (NODES - 1) as f64);
        if pos >= (NODES - 1) as f64 {
            return self.table[NODES - 1];
        }
        let i = pos as usize;
        let f = pos - i as f64;
        self.table[i] + f * (self.table[i + 1] - self.table[i])
    }
}

/// Detector level, dB, before the loudness offset.
fn level(power: f64) -> f64 {
    10.0 * libm::log10(libm::fabs(power).max(1e-20))
}

fn loudness(power: f64) -> f64 {
    level(power) + LOUDNESS_OFFSET
}

fn power_of(l: f64) -> f64 {
    libm::pow(10.0, (l - LOUDNESS_OFFSET) / 10.0)
}

fn bin_of(l: f64) -> usize {
    let b = libm::round((l - BIN_LO_DB).max(0.0) * 10.01);
    (b as usize).min(BINS - 1)
}

/// `exp(-1 / (tau * rate))` in single precision, as the unit forms it.
fn pole(tau_s: f32, rate: f32) -> f32 {
    libm::expf(-1.0 / (tau_s * rate))
}

/// The power every smoothed store starts from after a reset (-70 dB).
const SEED_POWER: f64 = 1e-7;
/// Long-path validity floor, loudness dB: a quieter window neither counts
/// nor ages the history.
const VALID_DB: f64 = -70.0;
/// The histogram is integrated from this far under its own smoothed
/// program energy, dB (the relative gate).
const GATE_DB: f64 = 20.0;
/// History and gate time constants, s. A balanced histogram (or one
/// weighted above its reading) keeps `HIST_LONG_S` of history; one weighted
/// `IMBALANCE_DB` or more below its reading keeps `HIST_SHORT_S`. The gate's
/// smoothing runs from `GATE_S` to twice that on the same measure.
const HIST_LONG_S: f32 = 5.0;
const HIST_SHORT_S: f32 = 2.0;
const GATE_S: f32 = 1.0;
const IMBALANCE_DB: f32 = 10.0;

/// The long-term gated histogram meter with its adaptive history.
struct LongTerm {
    hist: Box<[f64]>,
    energy: Box<[f64]>,
    reading: f64,
    gate_energy: f64,
    gate_a: f32,
    rate: f32,
}

impl LongTerm {
    fn new(rate: f32) -> Self {
        let energy: Box<[f64]> = (0..BINS).map(|i| power_of(BIN_LO_DB + BIN_DB * i as f64)).collect();
        LongTerm {
            hist: vec![0.0; BINS].into_boxed_slice(),
            energy,
            reading: 0.0,
            gate_energy: SEED_POWER,
            gate_a: 0.0,
            rate,
        }
    }

    fn reset(&mut self, expected: f64) {
        self.hist.iter_mut().for_each(|h| *h = 0.0);
        let c = bin_of(expected);
        for h in &mut self.hist[c.saturating_sub(SEED_HALF)..=(c + SEED_HALF).min(BINS - 1)] {
            *h = 1.0;
        }
        self.reading = expected;
        self.gate_energy = SEED_POWER;
        self.gate_a = pole(GATE_S, self.rate);
    }

    /// One window's mean power; the new reading.
    fn update(&mut self, power: f64) -> f64 {
        let l = loudness(power);
        if l <= VALID_DB {
            return self.reading;
        }
        let a = f64::from(self.gate_a);
        self.gate_energy = a * self.gate_energy + (1.0 - a) * power;
        let floor = loudness(self.gate_energy) - GATE_DB;
        self.hist[bin_of(l)] += 1.0;
        let start = bin_of(floor);
        let (mut n, mut e) = (0.0, 0.0);
        for (h, en) in self.hist[start..].iter().zip(&self.energy[start..]) {
            n += h;
            e += h * en;
        }
        if n > 1e-20 {
            self.reading = loudness(e / n);
        }
        // History: how far the counts lean below the new reading (the
        // reading's own bin excluded) sets how fast every count fades.
        let pivot = bin_of(self.reading);
        let above = (self.hist[pivot + 1..].iter().sum::<f64>() + 1e-20) as f32;
        let below = (self.hist[..pivot].iter().sum::<f64>() + 1e-20) as f32;
        let lean = 20.0 * libm::log10f(libm::fabsf(above / below) + 1e-6) / -IMBALANCE_DB;
        let x2 = lean.clamp(0.0, 1.0) * lean.clamp(0.0, 1.0);
        self.gate_a = pole(GATE_S * (1.0 + x2), self.rate);
        let fade = f64::from(pole(HIST_LONG_S - (HIST_LONG_S - HIST_SHORT_S) * x2, self.rate));
        self.hist.iter_mut().for_each(|h| *h *= fade);
        self.reading
    }
}

/// Per-sample smoothing of the summed detector power, s.
const EMA_S: f64 = 0.030;
/// The unit's level stores, dB: added to the call's loudest 64-frame level
/// (the fast path), to the lower median (the median path), and to the
/// 512-frame level that competes with the long path.
const FAST_STORE_DB: f64 = -8.0;
const MEDIAN_STORE_DB: f64 = -5.0;
const COMPETITOR_STORE_DB: f64 = -2.0;
/// The median path: this many 512-frame levels, read at this sorted index
/// (the lower median).
const MEDIAN_LEN: usize = 94;
const MEDIAN_AT: usize = 46;
/// The long path: a sliding mean of this many 64-frame powers, emitted to
/// the histogram every eight blocks.
const WINDOW_LEN: usize = 600;
/// Release dead band of the median path, dB, and its time constant, s.
const MID_DEADBAND_DB: f64 = 5.0;
const MID_RELEASE_S: f32 = 2.316;
/// Time constant of the long path's preconditioner glide, s. In single
/// precision its pole rounds to exactly 1, so the glide holds.
const PRECOND_S: f32 = 1e7;

/// The 512-frame median path with its release limiter.
struct Median {
    ring: [f64; MEDIAN_LEN],
    sorted: [f64; MEDIAN_LEN],
    at: usize,
    limited: f64,
    coef: f64,
}

impl Median {
    fn new(rate: f32) -> Self {
        Median {
            ring: [0.0; MEDIAN_LEN],
            sorted: [0.0; MEDIAN_LEN],
            at: 0,
            limited: 0.0,
            coef: f64::from(1.0 - pole(MID_RELEASE_S, rate)),
        }
    }

    fn reset(&mut self, seed: f64) {
        self.ring = [seed; MEDIAN_LEN];
        self.sorted = [seed; MEDIAN_LEN];
        self.at = 0;
        self.limited = seed;
    }

    /// Replace the oldest level with `v`; the limited median.
    fn push(&mut self, v: f64) -> f64 {
        let old = core::mem::replace(&mut self.ring[self.at], v);
        self.at = (self.at + 1) % MEDIAN_LEN;
        // remove `old` from the sorted copy and insert `v`
        let mut k = self.sorted.iter().position(|x| *x == old).unwrap_or(0);
        self.sorted[k] = v;
        while k > 0 && self.sorted[k - 1] > self.sorted[k] {
            self.sorted.swap(k - 1, k);
            k -= 1;
        }
        while k + 1 < MEDIAN_LEN && self.sorted[k + 1] < self.sorted[k] {
            self.sorted.swap(k + 1, k);
            k += 1;
        }
        let m = self.sorted[MEDIAN_AT] + MEDIAN_STORE_DB;
        if m > self.limited {
            self.limited = m;
        } else if m + MID_DEADBAND_DB < self.limited {
            self.limited += self.coef * (m + MID_DEADBAND_DB - self.limited);
        }
        self.limited
    }
}

/// The long path's preconditioner: passes the histogram reading while the
/// competing 512-frame level is above it; once the reading overtakes the
/// competitor it clamps to the competitor's last level and then glides
/// (here: holds) until the competitor rises above the reading again.
struct Precondition {
    out: f64,
    rising: bool,
    source: f64,
    coef: f64,
}

impl Precondition {
    fn new(rate: f32) -> Self {
        Precondition { out: 0.0, rising: false, source: 0.0, coef: f64::from(1.0 - pole(PRECOND_S, rate)) }
    }

    fn reset(&mut self) {
        self.out = 0.0;
        self.rising = false;
        self.source = 0.0;
    }

    fn update(&mut self, source: f64, reading: f64) -> f64 {
        let out = if source <= reading && self.out <= reading {
            if self.rising {
                reading.min(self.source)
            } else {
                self.out + self.coef * (reading - self.out)
            }
        } else {
            reading
        };
        self.out = out;
        self.source = source;
        self.rising = source > reading;
        out
    }
}

/// The long path's input: the sliding mean of the last 600 smoothed
/// powers, kept as a running sum.
struct Window {
    ring: Box<[f64]>,
    at: usize,
    sum: f64,
}

impl Window {
    fn new() -> Self {
        Window { ring: vec![0.0; WINDOW_LEN].into_boxed_slice(), at: 0, sum: 0.0 }
    }
    fn reset(&mut self, seed: f64) {
        self.ring.iter_mut().for_each(|v| *v = seed);
        self.at = 0;
        self.sum = seed * WINDOW_LEN as f64;
    }
    fn push(&mut self, v: f64) {
        let old = core::mem::replace(&mut self.ring[self.at], v);
        self.at = (self.at + 1) % WINDOW_LEN;
        self.sum += v - old;
    }
    fn mean(&self) -> f64 {
        self.sum / WINDOW_LEN as f64
    }
    fn resum(&mut self) {
        // recomputed exactly once per pass so the running sum cannot drift
        self.sum = self.ring.iter().sum();
    }
}

pub struct Run {
    p: Params,
    channels: usize,
    // look-ahead ring, per channel
    ring: Box<[f32]>,
    pos: usize,
    // one call of input and of output, per channel
    call_in: Box<[f32]>,
    call_out: Box<[f32]>,
    fill: usize,
    // detector state, per channel and section
    z: [[[f64; 2]; MAX_SECTIONS]; MAX_CHANNELS],
    ema: f64,
    ema_a: f64,
    in_step: usize,
    steps: usize,
    blocks: usize,
    window: Window,
    median: Median,
    long: LongTerm,
    pre: Precondition,
    /// The call's loudest 64-frame level, dB.
    peak: f64,
    /// The 512-frame candidate: limited median against the conditioned
    /// long path.
    held: f64,
    candidate: f64,
    target_db: f64,
    gain_db: f64,
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize) -> Run {
        let channels = channels.clamp(1, MAX_CHANNELS);
        let block_rate = (fs / (STEP * PER_BLOCK) as f64) as f32;
        let long_rate = (fs / (STEP * PER_BLOCK * PER_LONG) as f64) as f32;
        let mut r = Run {
            p: p.clone(),
            channels,
            ring: vec![0.0; channels * p.lookahead.max(1)].into_boxed_slice(),
            pos: 0,
            call_in: vec![0.0; channels * CALL].into_boxed_slice(),
            call_out: vec![0.0; channels * CALL].into_boxed_slice(),
            fill: 0,
            z: [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS],
            ema: 0.0,
            ema_a: libm::exp(-1.0 / (EMA_S * fs)),
            in_step: 0,
            steps: 0,
            blocks: 0,
            window: Window::new(),
            median: Median::new(block_rate),
            long: LongTerm::new(long_rate),
            pre: Precondition::new(fs as f32 / STEP as f32),
            peak: f64::NEG_INFINITY,
            held: 0.0,
            candidate: 0.0,
            target_db: 0.0,
            gain_db: 0.0,
        };
        r.reset();
        r
    }

    pub fn reset(&mut self) {
        let e = self.p.expected_db;
        self.ring.iter_mut().for_each(|v| *v = 0.0);
        self.pos = 0;
        self.call_in.iter_mut().for_each(|v| *v = 0.0);
        self.call_out.iter_mut().for_each(|v| *v = 0.0);
        self.fill = 0;
        self.z = [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS];
        self.ema = SEED_POWER;
        self.in_step = 0;
        self.steps = 0;
        self.blocks = 0;
        self.window.reset(SEED_POWER);
        self.median.reset(e);
        self.long.reset(e);
        self.pre.reset();
        self.peak = f64::NEG_INFINITY;
        self.held = e;
        self.candidate = e;
        self.target_db = self.p.gain_db(e);
        self.gain_db = self.target_db;
    }

    /// Frames: the unit's look-ahead plus one call, because the unit meters
    /// a whole call before it applies that call's gain.
    pub fn latency(&self) -> usize {
        self.p.lookahead + CALL
    }

    /// The gain at the end of the last whole call, dB.
    pub fn gain_db(&self) -> f64 {
        self.gain_db
    }

    /// The loudness the last whole call's gain was read for, dB.
    pub fn candidate_db(&self) -> f64 {
        self.candidate
    }

    fn detect(&mut self, ch: usize, x: f64) -> f64 {
        let mut v = x;
        for (s, z) in self.p.sos[..self.p.sections].iter().zip(self.z[ch].iter_mut()) {
            let y = s[0] * v + z[0];
            z[0] = s[1] * v - s[3] * y + z[1];
            z[1] = s[2] * v - s[4] * y;
            v = y;
        }
        v * v
    }

    /// Every 64 frames: the call's peak level and the window, and on their
    /// own boundaries the median (512) and long (4096) paths.
    fn on_step(&mut self) {
        let ema = self.ema;
        self.window.push(ema);
        let db = level(ema);
        self.peak = self.peak.max(db);
        self.steps += 1;
        if self.steps == PER_BLOCK {
            self.steps = 0;
            let mid = self.median.push(db);
            self.blocks += 1;
            if self.blocks == PER_LONG {
                self.blocks = 0;
                self.window.resum();
                self.long.update(self.window.mean());
            }
            let long = self.pre.update(db + COMPETITOR_STORE_DB, self.long.reading);
            self.held = mid.max(long);
        }
    }

    /// One call: meter all of it, then move the gain across it.
    fn run_call(&mut self) {
        let nch = self.channels;
        let la = self.p.lookahead;
        for n in 0..CALL {
            let mut power = 0.0;
            for ch in 0..nch {
                let x = self.call_in[ch * CALL + n];
                power += self.detect(ch, f64::from(x));
            }
            self.ema = self.ema_a * self.ema + (1.0 - self.ema_a) * power.max(1e-12);
            self.in_step += 1;
            if self.in_step == STEP {
                self.in_step = 0;
                self.on_step();
            }
        }
        if self.peak > f64::NEG_INFINITY {
            self.candidate = self.held.max(self.peak + FAST_STORE_DB);
            self.target_db = self.p.gain_db(self.candidate);
            self.peak = f64::NEG_INFINITY;
        }
        for n in 0..CALL {
            self.gain_db += GLIDE * (self.target_db - self.gain_db);
            let g = libm::pow(10.0, self.gain_db / 20.0) as f32;
            for ch in 0..nch {
                let x = self.call_in[ch * CALL + n];
                let y = if la == 0 {
                    x
                } else {
                    let slot = &mut self.ring[ch * la + self.pos];
                    core::mem::replace(slot, x)
                };
                self.call_out[ch * CALL + n] = y * g;
            }
            if la != 0 {
                self.pos = (self.pos + 1) % la;
            }
        }
    }

    /// In place on `channels` buffers of at least `frames` samples.
    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let nch = self.channels.min(bufs.len());
        for n in 0..frames {
            let f = self.fill;
            for (ch, b) in bufs.iter_mut().enumerate().take(nch) {
                let Some(s) = b.get_mut(n) else { continue };
                let x = if s.is_finite() { *s } else { 0.0 };
                self.call_in[ch * CALL + f] = x;
                *s = self.call_out[ch * CALL + f];
            }
            self.fill += 1;
            if self.fill == CALL {
                self.fill = 0;
                self.run_call();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A synthetic law: +4 dB below -40, falling 0.5 dB/dB to -10, flat after.
    fn params(lookahead: usize) -> Params {
        let mut table = [0.0; NODES];
        for (i, t) in table.iter_mut().enumerate() {
            let l = NODE_LO_DB + NODE_STEP_DB * i as f64;
            *t = (4.0 - 0.5 * (l + 40.0).max(0.0)).max(-11.0);
        }
        Params { table, expected_db: -30.0, reset_timeout_s: 8.0, lookahead, ..Params::default() }
    }

    #[test]
    fn words_round_trip_and_reject_bad_blocks() {
        let mut p = params(144);
        p.sections = 1;
        p.sos[0] = [0.9, -1.8, 0.9, -1.79, 0.81];
        let mut w = vec![0.0; Params::WORDS];
        p.to_words(&mut w);
        assert_eq!(Params::from_words(&w), Some(p.clone()));
        let mut bad = w.clone();
        bad[NODES + 2] = 0.5; // fractional look-ahead
        assert_eq!(Params::from_words(&bad), None);
        let mut bad = w.clone();
        bad[NODES + 4 + 4] = 1.5; // unstable section
        assert_eq!(Params::from_words(&bad), None);
        let mut bad = w;
        bad[3] = f64::NAN;
        assert_eq!(Params::from_words(&bad), None);
    }

    #[test]
    fn table_reads_interpolate_and_clamp() {
        let p = params(0);
        assert_eq!(p.gain_db(-200.0), p.table[0]);
        assert_eq!(p.gain_db(50.0), p.table[NODES - 1]);
        let mid = p.gain_db(NODE_LO_DB + 2.75);
        assert!((mid - 0.5 * (p.table[0] + p.table[1])).abs() < 1e-12);
    }

    #[test]
    fn starts_at_the_expected_loudness_gain_and_delays_by_the_latency() {
        let p = params(144);
        let mut r = Run::new(&p, 48_000.0, 2);
        assert!((r.gain_db() - p.gain_db(p.expected_db)).abs() < 1e-12);
        assert_eq!(r.latency(), 144 + CALL);
        let n = 2048;
        let mut a = vec![0.0f32; n];
        let mut b = vec![0.0f32; n];
        a[10] = 0.25;
        r.process(&mut [&mut a, &mut b], n);
        let at = 10 + r.latency();
        let g = libm::pow(10.0, p.gain_db(p.expected_db) / 20.0) as f32;
        assert!(a.iter().enumerate().all(|(i, v)| i == at || *v == 0.0));
        assert!((a[at] - 0.25 * g).abs() < 1e-6);
        assert!(b.iter().all(|v| *v == 0.0));
    }

    #[test]
    fn calls_of_any_size_give_the_same_output() {
        let p = params(144);
        let n = 48_000;
        let x: Vec<f32> =
            (0..n).map(|i| 0.3 * libm::sinf(i as f32 * 0.05) * if i > 20_000 { 1.0 } else { 0.1 }).collect();
        let mut whole = x.clone();
        Run::new(&p, 48_000.0, 1).process(&mut [&mut whole], n);
        let mut pieces = x;
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut at = 0;
        for (k, q) in [1usize, 7, 64, 333, 512, 1024, 4000].iter().cycle().enumerate() {
            if at >= n || k > 10_000 {
                break;
            }
            let m = (*q).min(n - at);
            r.process(&mut [&mut pieces[at..at + m]], m);
            at += m;
        }
        assert_eq!(whole, pieces);
    }

    #[test]
    fn one_gain_for_every_channel_and_silence_for_non_finite_input() {
        let p = params(0);
        let mut r = Run::new(&p, 48_000.0, 2);
        let lat = r.latency();
        let n = 48_000;
        let mut a: Vec<f32> = (0..n).map(|i| 0.5 * libm::sinf(i as f32 * 0.13)).collect();
        let mut b: Vec<f32> = (0..n).map(|i| 0.01 * libm::sinf(i as f32 * 0.07)).collect();
        a[100] = f32::NAN;
        b[200] = f32::INFINITY;
        let (a0, b0) = (a.clone(), b.clone());
        r.process(&mut [&mut a, &mut b], n);
        assert!(a.iter().chain(&b).all(|v| v.is_finite()));
        assert_eq!(a[100 + lat], 0.0);
        assert_eq!(b[200 + lat], 0.0);
        // the same gain on both channels, sample by sample
        for i in (1000..n).step_by(997) {
            let (x, y) = (a0[i - lat], b0[i - lat]);
            if x.abs() > 1e-3 && y.abs() > 1e-4 {
                assert!((a[i] / x - b[i] / y).abs() < 1e-4);
            }
        }
    }

    #[test]
    fn a_steady_level_settles_on_the_table() {
        // flat detector: a full-scale-ish square wave of known power
        let p = params(0);
        let mut r = Run::new(&p, 48_000.0, 1);
        let amp = 0.1f32; // power 0.01
        let level = 10.0 * libm::log10(0.01) + COMPETITOR_STORE_DB;
        let mut buf = vec![0.0f32; 4096];
        for _ in 0..(120 * 48_000 / 4096) {
            buf.iter_mut().enumerate().for_each(|(i, v)| *v = if i % 2 == 0 { amp } else { -amp });
            r.process(&mut [&mut buf], 4096);
        }
        // the seed fades as program accumulates; two minutes leave < 0.1 dB
        assert!((r.gain_db() - p.gain_db(level)).abs() < 0.1, "{} vs {}", r.gain_db(), p.gain_db(level));
    }

    #[test]
    fn the_long_meter_forgets_old_program() {
        // A minute loud, then quiet: a cumulative histogram would still read
        // between the two; the fading one follows the quiet program within
        // a few of its time constants.
        let rate = (48_000.0 / 4096.0) as f32;
        let mut m = LongTerm::new(rate);
        m.reset(-20.0);
        let loud = power_of(-10.0);
        let quiet = power_of(-30.0);
        for _ in 0..(60.0 * rate) as usize {
            m.update(loud);
        }
        assert!((m.reading + 10.0).abs() < 0.15, "{}", m.reading);
        for _ in 0..(20.0 * rate) as usize {
            m.update(quiet);
        }
        assert!((m.reading + 30.0).abs() < 0.3, "{}", m.reading);
        // windows under the validity floor neither count nor age it
        let before = m.reading;
        for _ in 0..(20.0 * rate) as usize {
            m.update(power_of(-80.0));
        }
        assert_eq!(m.reading, before);
    }

    #[test]
    fn the_preconditioner_holds_the_level_it_was_overtaken_at() {
        let mut p = Precondition::new(750.0);
        assert_eq!(p.coef, 0.0);
        // the reading climbs past a steady competitor at -12
        let mut out = 0.0;
        for r in [-20.0, -16.0, -13.0, -11.0, -10.5, -10.4] {
            out = p.update(-12.0, r);
        }
        assert_eq!(out, -12.0);
        // it follows the reading down, and again once the competitor is above
        assert_eq!(p.update(-12.0, -12.5), -12.5);
        assert_eq!(p.update(-5.0, -11.0), -11.0);
    }

    #[test]
    fn reset_restores_the_start() {
        let p = params(16);
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut buf = vec![0.3f32; 48_000];
        r.process(&mut [&mut buf], 48_000);
        r.reset();
        assert!((r.gain_db() - p.gain_db(p.expected_db)).abs() < 1e-12);
        let mut z = vec![0.0f32; 64];
        r.process(&mut [&mut z], 64);
        assert!(z.iter().all(|v| *v == 0.0));
    }
}
