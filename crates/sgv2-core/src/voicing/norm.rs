//! Norm: the stereo loudness normalizer at the head of the speaker program.
//!
//! One broadband gain, shared by every channel, steers the program's
//! loudness toward the tuning's target. The audio path is a fixed
//! look-ahead delay and that gain; everything else is the side chain.
//!
//! Side chain, per frame: each channel through the detector cascade (the
//! loudness weighting, its high-pass section replaced by the tuning's
//! side-chain high-pass when that is on), squared, and
//! the channels summed (a full and a silent channel read the same as the
//! full one alone; two equal channels read 3 dB louder). The sum is
//! accumulated over 64-frame steps; eight steps make a 512-frame block and
//! eight blocks a 4096-frame block. The counters run on the stream, not on
//! the host's calls.
//!
//! Meters (levels are `10 log10(power)` of the smoothed detector power; the
//! histogram works in loudness units, that minus 0.691):
//! - fast: the loudest 64-frame level of the call, less a fixed store, held
//!   and released only once the call sits more than a dead band under it,
//!   with a release that speeds up over the first seconds of audible program;
//! - median: the middle of the last 95 512-frame levels (every block from
//!   the reset on, frame 0's included), less a fixed store, behind a
//!   release limiter that only lets go once the median sits more than its
//!   dead band under it;
//! - long: a 1001-bin histogram of 4096-frame window levels over -100..0 in
//!   0.1 dB bins, read as the power mean of the bins above a relative gate
//!   that trails the histogram's own smoothed energy (a fixed one-second
//!   smoothing). A level's bin is the one below the nearest point of the
//!   unit's 10.01-per-dB index grid, for every use of the index (the
//!   window's count, the reset seed, the gate and the history pivot), while
//!   each bin still represents its own 0.1 dB step. Every admitted window
//!   fades all counts by a factor whose time constant shortens as the counts
//!   lean below the reading, so the history spans seconds, not the whole
//!   program. A reset clears it and seeds the eleven bins around the
//!   expected loudness with one count each.
//!   A window below the validity floor still emits one slot containing the
//!   retained reading; the following hold does not skip or reset that slot.
//! - the long reading reaches the arbitration `LONG_DELAY` long windows
//!   after the window that produced it, on that window boundary's own
//!   512-frame event (the reset reading fills the gap);
//! - the long reading passes a preconditioner against the 512-frame level
//!   (less its own store): it follows the reading while that level is above
//!   it and, once the reading overtakes it, holds the level it overtook.
//!
//! The call's candidate is the larger of fast and of median against the
//! conditioned long path; it is read once per call.
//!
//! Two start-up and low-level rules of the unit:
//! - a reset takes full effect one call late: after the first call that
//!   follows it the detector cascade's state is cleared again (the meters
//!   keep what that call gave them), so a program that starts abruptly with
//!   the reset sees its own onset twice;
//! - a settled low-level rise (the call's loudest 64-frame level, floored at
//!   the validity floor, more than 4 dB over its slow follower while moving
//!   less than one dB per 100 frames' worth of events, and more than 10 dB
//!   under the long reading) moves the whole retained long-path
//!   coordinate 1.5 dB colder: histogram, reading, the readings on their way
//!   to the arbitration, the conditioned output and, twice over, the held
//!   candidate.
//!
//! Gain law: a 21-node table over -100..+10 in 5.5 dB steps, compiled from
//! the tuning (target, maximum gain, upward and downward ranges, ratio),
//! read as a uniform cubic B-spline over the nodes (not by linear
//! interpolation: the unit rounds the table's knees). The applied gain is
//! linear: each frame it covers a fixed share of the remaining distance to
//! the call's target amplitude, a one-pole smoother with a 200 Hz corner
//! (time constant 1/(2 pi 200) s, about 0.8 ms, the same time at every
//! sample rate). Gliding in dB instead bends every large step's onset by
//! up to tenths of a dB.
//!
//! Algorithm 1 (the earliest version a strip can select) keeps the same
//! detector, meters and gain law with these differences:
//! - the detector smoothing time is the saved transient response, not the
//!   fixed 30 ms;
//! - a reset starts every meter silent: the median ring, the long reading
//!   and its delay line, the fast level and the held candidate all start at
//!   the histogram floor, so the first call is read from the program alone
//!   (the long histogram's relative gate starts from the same -70 dB seed as
//!   the other algorithms, not from the expected loudness: a gate seeded at
//!   an expected loudness of -14 lifts the second pause's long reading
//!   above the unit's); the smoothed detector power starts from a small fixed floor
//!   (`V1_DETECTOR_SEED_DB`) that fades with the detector's own time
//!   constant; there is no deferred second clear of the detector state;
//! - the applied gain starts its glide from `V1_START_GAIN_DB`;
//! - the fast level is a two-sided hysteresis on the maximum smoothed
//!   detector level in each completed 64-frame step, less a 4 dB store: it
//!   rises only when that stored maximum strictly clears it by 3 dB and
//!   falls to 8 dB above the stored maximum once that edge is under it;
//! - the median store is 1.691 dB and the median moves straight to its
//!   five-dB release edge; the competitor of the long path has no store and
//!   there is no low-level coordinate move;
//! - the candidate and the gain's target are read on every 64-frame step,
//!   so the gain moves within a call instead of at its boundary;
//! - the long path's preconditioner competes against the fast level (the
//!   hysteresis output of this step), not against the 512-frame level: it
//!   follows the long reading while the fast level is above it and, once
//!   the reading overtakes the fast level, holds the smaller of the reading
//!   and the fast level it overtook. So in a pause the long path keeps the
//!   reading that lands while the fast level is still releasing, and a
//!   following burst whose fast level stays under a newer reading keeps
//!   that hold;
//! - an invalid long window still ages the histogram.
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
/// Corner of the one-pole that glides the linear gain toward each call's
/// target, Hz.
const GLIDE_HZ: f64 = 200.0;
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
    /// Dead band of the fast path's release, dB: the held fast level only
    /// releases once the call's level is this far under it.
    pub fast_deadband_db: f64,
    /// Algorithm 1: see the module documentation.
    pub algorithm1: bool,
    /// Detector power smoothing time, s (algorithm 1: the saved transient
    /// response; algorithms 2 and 3: `EMA_S`).
    pub detector_ema_s: f64,
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
            fast_deadband_db: 7.0,
            algorithm1: false,
            detector_ema_s: EMA_S,
        }
    }
}

impl Params {
    /// The algorithm 2/3 payload.
    pub const LEGACY_WORDS: usize = NODES + 5 + 5 * MAX_SECTIONS;
    /// The algorithm-1 payload adds its selector and detector time.
    pub const WORDS: usize = Self::LEGACY_WORDS + 2;

    pub fn to_words(&self, out: &mut [f64]) {
        out[..NODES].copy_from_slice(&self.table);
        out[NODES] = self.expected_db;
        out[NODES + 1] = self.reset_timeout_s;
        out[NODES + 2] = self.lookahead as f64;
        out[NODES + 3] = self.sections as f64;
        for (i, s) in self.sos.iter().enumerate() {
            out[NODES + 4 + 5 * i..NODES + 9 + 5 * i].copy_from_slice(s);
        }
        out[NODES + 4 + 5 * MAX_SECTIONS] = self.fast_deadband_db;
        if out.len() >= Self::WORDS {
            out[Self::LEGACY_WORDS] = if self.algorithm1 { 1.0 } else { 0.0 };
            out[Self::LEGACY_WORDS + 1] = self.detector_ema_s;
        }
    }

    pub fn from_words(w: &[f64]) -> Option<Params> {
        if ![Self::LEGACY_WORDS, Self::WORDS].contains(&w.len()) || w.iter().any(|v| !v.is_finite()) {
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
        p.fast_deadband_db = w[NODES + 4 + 5 * MAX_SECTIONS];
        if w.len() == Self::WORDS {
            let selector = w[Self::LEGACY_WORDS];
            if selector != 0.0 && selector != 1.0 {
                return None;
            }
            p.algorithm1 = selector == 1.0;
            p.detector_ema_s = w[Self::LEGACY_WORDS + 1];
        }
        p.valid().then_some(p)
    }

    fn valid(&self) -> bool {
        let table_ok = self.table.iter().all(|g| (-60.0..=60.0).contains(g));
        let expected_ok = (-100.0..=0.0).contains(&self.expected_db);
        let timeout_ok = (0.0..=3600.0).contains(&self.reset_timeout_s);
        let deadband_ok = (0.0..=40.0).contains(&self.fast_deadband_db);
        let detector_ok = (1e-6..=1.0).contains(&self.detector_ema_s);
        // every section stable (poles inside the unit circle)
        let stable = self.sos[..self.sections].iter().all(|s| {
            let (a1, a2) = (s[3], s[4]);
            a2.abs() < 1.0 && a1.abs() < 1.0 + a2
        });
        let unused_zero = self.sos[self.sections..].iter().all(|s| s.iter().all(|v| *v == 0.0));
        table_ok && expected_ok && timeout_ok && deadband_ok && detector_ok && stable && unused_zero
    }

    /// The table's gain for loudness `l`, dB: a uniform cubic B-spline
    /// with the nodes as its control points, the end nodes repeated past
    /// either end of the grid, and `l` clamped to the grid. On a straight
    /// run of nodes it equals linear interpolation; at a node it reads
    /// `(left + 4 * node + right) / 6`, so every knee of the table is
    /// rounded off.
    pub fn gain_db(&self, l: f64) -> f64 {
        let pos = ((l - NODE_LO_DB) / NODE_STEP_DB).clamp(0.0, (NODES - 1) as f64);
        let i = (pos as usize).min(NODES - 2);
        let t = pos - i as f64;
        let node = |k: isize| self.table[k.clamp(0, NODES as isize - 1) as usize];
        let k = i as isize;
        let (p0, p1, p2, p3) = (node(k - 1), node(k), node(k + 1), node(k + 2));
        let (t2, t3) = (t * t, t * t * t);
        let u = 1.0 - t;
        (u * u * u * p0 + (3.0 * t3 - 6.0 * t2 + 4.0) * p1 + (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) * p2 + t3 * p3)
            / 6.0
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

/// The histogram bin of loudness `l`: one below the nearest point of the
/// 10.01-per-dB index grid, clamped to the histogram.
fn bin_of(l: f64) -> usize {
    let b = libm::round((l - BIN_LO_DB).max(0.0) * 10.01);
    ((b as usize).saturating_sub(1)).min(BINS - 1)
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
/// smoothing is `GATE_S` whatever the balance.
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
    /// Algorithm 1 keeps aging the counts on an invalid window.
    age_invalid: bool,
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
            age_invalid: false,
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

    /// Move the whole retained coordinate `db` colder (whole bins).
    fn shift_down(&mut self, db: f64) {
        let bins = (libm::round(db / BIN_DB).max(0.0) as usize).min(BINS);
        self.hist.copy_within(bins.., 0);
        self.hist[BINS - bins..].fill(0.0);
        self.reading -= db;
    }

    /// One window's mean power; the output slot. An invalid window returns
    /// the retained reading without changing or aging the histogram.
    fn update(&mut self, power: f64) -> f64 {
        let l = loudness(power);
        let valid = l > VALID_DB;
        if !valid && !self.age_invalid {
            return self.reading;
        }
        if valid {
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
        }
        // History: how far the counts lean below the new reading (the
        // reading's own bin excluded) sets how fast every count fades.
        let pivot = bin_of(self.reading);
        let above = (self.hist[pivot + 1..].iter().sum::<f64>() + 1e-20) as f32;
        let below = (self.hist[..pivot].iter().sum::<f64>() + 1e-20) as f32;
        let lean = 20.0 * libm::log10f(libm::fabsf(above / below) + 1e-6) / -IMBALANCE_DB;
        let x2 = lean.clamp(0.0, 1.0) * lean.clamp(0.0, 1.0);
        let fade = f64::from(pole(HIST_LONG_S - (HIST_LONG_S - HIST_SHORT_S) * x2, self.rate));
        self.hist.iter_mut().for_each(|h| *h *= fade);
        self.reading
    }
}

/// Per-sample smoothing of the summed detector power, s.
const EMA_S: f64 = 0.030;
/// The unit's level stores, dB: added to the call's loudest 64-frame level
/// (the fast path), to the median (the median path), and to the
/// 512-frame level that competes with the long path.
const FAST_STORE_DB: f64 = -8.0;
const MEDIAN_STORE_DB: f64 = -5.0;
/// Algorithm 1: fast store, its rise and fall hysteresis, the median store
/// and the level every meter starts from after a reset, dB.
const V1_FAST_STORE_DB: f64 = -4.0;
const V1_FAST_RISE_DB: f64 = 3.0;
const V1_FAST_FALL_DB: f64 = 8.0;
const V1_MEDIAN_STORE_DB: f64 = LOUDNESS_OFFSET - 1.0;
const V1_SILENT_DB: f64 = BIN_LO_DB;
/// Algorithm 1: where the applied gain starts its glide after a reset, dB.
/// Measured on the installed unit sample by sample (the 200 Hz glide's own
/// pole fits frames 3..39 of a 1 kHz onset to 24.0195..24.0187 dB, the same
/// on every algorithm-1 state); it is not the parameter default of 24 dB.
const V1_START_GAIN_DB: f64 = 24.0195;
/// Algorithm 1: the smoothed detector power after a reset, dB (power
/// `10^(dB / 10)`). Measured on the installed unit through the fast path's
/// +3 dB rise test: onsets whose rise margin is a few thousandths of a dB
/// resolve as the unit does only for a start between -53.5 and -51.5 dB
/// (a 40 Hz onset at -12 dBFS does not rise where the same onset at 0 and
/// -6 dBFS does, which no scale-free law explains).
const V1_DETECTOR_SEED_DB: f64 = -52.5;
const COMPETITOR_STORE_DB: f64 = -2.0;
/// The median path: the true median of this many 512-frame levels (an odd
/// count, read at its middle sorted index). Measured on the unit: after a
/// step it flips on the 48th new level, and its rank moves through a
/// random program exactly as a 95-level window does (96 levels read at
/// either middle rank miss it by up to 0.4 dB).
const MEDIAN_LEN: usize = 95;
const MEDIAN_AT: usize = MEDIAN_LEN / 2;
/// The long path: a sliding mean of this many 64-frame powers, emitted to
/// the histogram every eight blocks.
const WINDOW_LEN: usize = 600;
/// Long windows between a histogram reading and the arbitration that
/// first sees it, on that later window boundary's own 512-frame event.
/// Measured on the installed unit's audio (its long-path steps land exactly
/// this many 4096-frame windows after a reading is formed; the unit's
/// momentary-gain report trails its audio by one call); not yet attributed
/// to a stage of the unit.
const LONG_DELAY: usize = 3;
/// Release dead band of the median path, dB, and its time constant, s.
const MID_DEADBAND_DB: f64 = 5.0;
const MID_RELEASE_S: f32 = 2.316;
/// Fast-path release. The held fast level glides down with pole
/// `exp(-1000 / (k * span))`, `span` the frames per 64-frame event of the
/// call's rate (`fs` over the call's event count). `k` is a history weight
/// that starts at `FAST_HISTORY` and relaxes toward 1 by
/// `FAST_HISTORY_STEP / span` of the remaining distance per call whose fast
/// level is above `VALID_DB`.
const FAST_HISTORY: f64 = 1000.0;
const FAST_HISTORY_STEP: f64 = 100.0;
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
    store_db: f64,
}

impl Median {
    fn new(rate: f32, algorithm1: bool) -> Self {
        Median {
            ring: [0.0; MEDIAN_LEN],
            sorted: [0.0; MEDIAN_LEN],
            at: 0,
            limited: 0.0,
            // algorithm 1 moves straight to the release edge
            coef: if algorithm1 { 1.0 } else { f64::from(1.0 - pole(MID_RELEASE_S, rate)) },
            store_db: if algorithm1 { V1_MEDIAN_STORE_DB } else { MEDIAN_STORE_DB },
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
        let m = self.sorted[MEDIAN_AT] + self.store_db;
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

/// The low-level rise that moves the long-path coordinate.
const RISE_DB: f32 = 4.0;
/// The follower's rate, in follower steps per second of events.
const RISE_FOLLOW: f32 = 10.0;
/// The rise counts while the peak moves less than this many dB per
/// `RISE_SLOPE_FRAMES` worth of events.
const RISE_SLOPE: f32 = 1.0;
const RISE_SLOPE_FRAMES: f32 = 100.0;
/// Only this far or more under the long reading, dB.
const RISE_BELOW_DB: f32 = 10.0;
/// The coordinate move, dB (15 histogram bins).
const RISE_SHIFT_DB: f64 = 1.5;

/// Watches the call's peak for a settled low-level rise.
struct LowRise {
    rate: f32,
    follow: f32,
    last: f32,
}

impl LowRise {
    fn new(rate: f32) -> Self {
        LowRise { rate, follow: 0.0, last: 0.0 }
    }

    fn reset(&mut self) {
        self.follow = 0.0;
        self.last = 0.0;
    }

    /// The call's loudest 64-frame level, its event count and the long
    /// reading; the coordinate move this call asks for, dB.
    fn update(&mut self, peak: f64, events: usize, reading: f64) -> f64 {
        let raw = peak as f32;
        let peak = raw.max(VALID_DB as f32);
        let slope = self.rate * (peak - self.last) / events as f32 / RISE_SLOPE_FRAMES;
        self.last = peak;
        let rise = slope < RISE_SLOPE && self.follow + RISE_DB < peak;
        self.follow = if rise || self.follow > peak {
            peak
        } else {
            self.follow + (peak - self.follow) * RISE_FOLLOW * events as f32 / self.rate
        };
        if rise && raw < reading as f32 - RISE_BELOW_DB {
            RISE_SHIFT_DB
        } else {
            0.0
        }
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
    // detector state, per channel and section
    z: [[[f64; 2]; MAX_SECTIONS]; MAX_CHANNELS],
    ema: f64,
    ema_a: f64,
    /// Algorithm 1: maximum smoothed detector level in the current
    /// 64-frame step. The first partial step starts at the detector reset
    /// level; later steps contain only their own samples.
    step_peak: f64,
    in_step: usize,
    steps: usize,
    blocks: usize,
    window: Window,
    median: Median,
    long: LongTerm,
    pre: Precondition,
    rise: LowRise,
    fs: f64,
    /// The call's loudest 64-frame level, dB, and its 64-frame events.
    peak: f64,
    events: usize,
    /// The held fast level and its release weight.
    fast: f64,
    history: f64,
    /// This call's 512-frame event: level, limited median, long reading.
    pending: Option<(f64, f64, f64)>,
    /// The last `LONG_DELAY` long readings, oldest at `long_at`, and the
    /// reading the arbitration sees now.
    long_line: [f64; LONG_DELAY],
    long_at: usize,
    long_out: f64,
    /// The 512-frame candidate: limited median against the conditioned
    /// long path.
    held: f64,
    /// The reset's second, deferred clearing of the detector state is due
    /// after the next nonempty render call.
    clear_due: bool,
    candidate: f64,
    target_db: f64,
    /// Algorithm 1: the target read on each 64-frame step of the call.
    /// Algorithm 1: (frame in the call, target) of each 64-frame step of the
    /// current call, and how many.
    step_target: [(usize, f64); CALL / STEP + 1],
    steps_in_call: usize,
    /// The applied gain, linear, and the target it glides to.
    gain: f64,
    target: f64,
    /// Per-frame share of the remaining distance the gain covers.
    glide: f64,
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
            z: [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS],
            ema: 0.0,
            ema_a: libm::exp(-1.0 / (p.detector_ema_s * fs)),
            step_peak: f64::NEG_INFINITY,
            in_step: 0,
            steps: 0,
            blocks: 0,
            window: Window::new(),
            median: Median::new(block_rate, p.algorithm1),
            long: LongTerm::new(long_rate),
            pre: Precondition::new(fs as f32 / STEP as f32),
            rise: LowRise::new(fs as f32 / STEP as f32),
            fs,
            peak: f64::NEG_INFINITY,
            events: 0,
            fast: 0.0,
            history: FAST_HISTORY,
            pending: None,
            long_line: [p.expected_db; LONG_DELAY],
            long_at: 0,
            long_out: p.expected_db,
            held: 0.0,
            clear_due: false,
            candidate: 0.0,
            target_db: 0.0,
            step_target: [(0, 1.0); CALL / STEP + 1],
            steps_in_call: 0,
            gain: 1.0,
            target: 1.0,
            glide: 1.0 - libm::exp(-2.0 * core::f64::consts::PI * GLIDE_HZ / fs),
        };
        r.reset();
        r
    }

    pub fn reset(&mut self) {
        let e = self.p.expected_db;
        self.ring.iter_mut().for_each(|v| *v = 0.0);
        self.pos = 0;
        self.z = [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS];
        self.ema = SEED_POWER;
        self.step_peak = f64::NEG_INFINITY;
        self.in_step = 0;
        self.steps = 0;
        self.blocks = 0;
        self.window.reset(SEED_POWER);
        self.median.reset(e);
        self.long.reset(e);
        self.pre.reset();
        self.rise.reset();
        self.peak = f64::NEG_INFINITY;
        self.events = 0;
        self.fast = e;
        self.history = FAST_HISTORY;
        self.pending = None;
        self.long_line = [e; LONG_DELAY];
        self.long_at = 0;
        self.long_out = e;
        self.held = e;
        self.clear_due = true;
        self.candidate = e;
        self.target_db = self.p.gain_db(e);
        self.target = libm::pow(10.0, self.target_db / 20.0);
        self.gain = self.target;
        if self.p.algorithm1 {
            self.reset_algorithm1();
        }
    }

    /// Algorithm 1 starts every meter silent; the long histogram's relative
    /// gate keeps the common seed and the detector power its own floor.
    fn reset_algorithm1(&mut self) {
        let s = V1_SILENT_DB;
        self.ema = libm::pow(10.0, V1_DETECTOR_SEED_DB / 10.0);
        self.step_peak = level(self.ema);
        self.window.reset(0.0);
        self.median.reset(s);
        self.long.reset(s);
        self.long.age_invalid = true;
        self.long_line = [s; LONG_DELAY];
        self.long_out = s;
        self.fast = s;
        self.held = s;
        self.candidate = s;
        self.clear_due = false;
        self.target_db = self.p.gain_db(s);
        self.target = libm::pow(10.0, self.target_db / 20.0);
        self.steps_in_call = 0;
        self.gain = libm::pow(10.0, V1_START_GAIN_DB / 20.0);
    }

    /// Audio delay in frames. Metering and gain application complete inside
    /// the same render call, so only the configured look-ahead delays audio.
    pub fn latency(&self) -> usize {
        self.p.lookahead
    }

    /// The gain at the end of the last whole call, dB.
    pub fn gain_db(&self) -> f64 {
        20.0 * libm::log10(self.gain)
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

    /// One completed 64-frame step. `fast_db` is the maximum smoothed
    /// detector level in that step for algorithm 1; the other meters read
    /// the step-end detector state. The long path reads the window before
    /// the current power joins it, and its reading joins the 512-frame event
    /// on the boundary `LONG_DELAY` windows later.
    fn on_step(&mut self, fast_db: f64) {
        let ema = self.ema;
        let db = level(ema);
        self.peak = self.peak.max(fast_db);
        self.events += 1;
        if self.steps == 0 && self.blocks == 0 {
            self.window.resum();
            let reading = self.long.update(self.window.mean());
            self.long_out = core::mem::replace(&mut self.long_line[self.long_at], reading);
            self.long_at = (self.long_at + 1) % LONG_DELAY;
        }
        // the 512-frame event on a long window's own boundary pairs with the
        // reading that reaches the arbitration there
        let long = self.long_out;
        self.window.push(ema);
        if self.steps == 0 {
            let mid = self.median.push(db);
            self.pending = Some((db, mid, long));
            self.blocks = (self.blocks + 1) % PER_LONG;
        }
        self.steps = (self.steps + 1) % PER_BLOCK;
        if self.p.algorithm1 {
            self.arbitrate_step(fast_db);
        }
    }

    /// Algorithm 1, every 64-frame step: the fast hysteresis, the held
    /// candidate on a 512-frame event (the long path conditioned against
    /// the fast level), and the gain's target.
    fn arbitrate_step(&mut self, db: f64) {
        let fast = db + V1_FAST_STORE_DB;
        if fast > self.fast + V1_FAST_RISE_DB {
            self.fast = fast;
        } else if fast + V1_FAST_FALL_DB < self.fast {
            self.fast = fast + V1_FAST_FALL_DB;
        }
        if let Some((_, mid, long)) = self.pending.take() {
            // the long path competes against the fast level, not the
            // 512-frame level
            let long = self.pre.update(self.fast, long);
            self.held = mid.max(long);
        }
        self.candidate = self.held.max(self.fast);
        self.target_db = self.p.gain_db(self.candidate);
        self.target = libm::pow(10.0, self.target_db / 20.0);
    }

    /// End of a call: the fast path's held level, then the long path's
    /// preconditioner and the two maxima.
    fn arbitrate(&mut self) {
        let shift = self.rise.update(self.peak, self.events, self.long.reading);
        if shift > 0.0 {
            self.long.shift_down(shift);
            self.long_line.iter_mut().for_each(|v| *v -= shift);
            self.long_out -= shift;
            self.pre.out -= shift;
            // the held candidate carries the coordinate through two holds
            self.held -= 2.0 * shift;
            if let Some((_, _, long)) = self.pending.as_mut() {
                *long -= shift;
            }
        }
        let fast = self.peak + FAST_STORE_DB;
        let span = self.fs / self.events as f64;
        if fast > VALID_DB {
            self.history += (1.0 - self.history) * FAST_HISTORY_STEP / span;
        }
        let k = self.history.max(1.0);
        let pole = libm::exp(-1000.0 / (k * span));
        if fast > self.fast {
            self.fast = fast;
        } else if self.fast > fast + self.p.fast_deadband_db {
            // The dead-band edge is both the strict engagement threshold and
            // the release target. The held level approaches that edge, not
            // the colder call target below it.
            self.fast += (1.0 - pole) * (fast + self.p.fast_deadband_db - self.fast);
        }
        if let Some((db, mid, long)) = self.pending.take() {
            let long = self.pre.update(db + COMPETITOR_STORE_DB, long);
            self.held = mid.max(long);
        }
        self.candidate = self.held.max(self.fast);
        self.target_db = self.p.gain_db(self.candidate);
        self.target = libm::pow(10.0, self.target_db / 20.0);
    }

    /// In place on `channels` buffers of at least `frames` samples.
    ///
    /// Like the AU, meter the entire render call before moving the gain
    /// across that same call. The caller supplies the render boundary (512
    /// frames in the speaker graph); the meter counters remain stream based.
    /// A short final call is processed immediately, without padding or a flush.
    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        if frames == 0 {
            return;
        }
        if self.p.algorithm1 && frames > CALL {
            // Algorithm 1 moves the gain on every 64-frame step, so metering
            // a long call in pieces of at most one speaker call is the same
            // as metering it whole; it bounds the per-step target store.
            let nch = self.channels.min(bufs.len());
            let mut at = 0;
            while at < frames {
                let k = (frames - at).min(CALL);
                let mut piece: [&mut [f32]; MAX_CHANNELS] = [&mut [], &mut []];
                for (slot, b) in piece.iter_mut().zip(bufs.iter_mut()) {
                    let end = b.len().min(at + k);
                    let start = at.min(end);
                    *slot = &mut b[start..end];
                }
                self.process(&mut piece[..nch], k);
                at += k;
            }
            return;
        }
        self.steps_in_call = 0;
        let nch = self.channels.min(bufs.len());
        let la = self.p.lookahead;
        let start_target = self.target;
        for n in 0..frames {
            let mut power = 0.0;
            for (ch, b) in bufs.iter().enumerate().take(nch) {
                let x = b.get(n).copied().filter(|v| v.is_finite()).unwrap_or(0.0);
                power += self.detect(ch, f64::from(x));
            }
            self.ema = self.ema_a * self.ema + (1.0 - self.ema_a) * power.max(1e-12);
            if self.p.algorithm1 {
                self.step_peak = self.step_peak.max(level(self.ema));
            }
            let step_done = self.p.algorithm1 && self.in_step + 1 == STEP;
            if step_done {
                self.on_step(self.step_peak);
                self.step_peak = f64::NEG_INFINITY;
                if self.steps_in_call < self.step_target.len() {
                    // Meter the whole step first, then apply its target from
                    // that same step's first frame: the audio pass follows
                    // the side-chain pass for the complete render call. A
                    // step carried across a short call starts at frame zero
                    // of this call as far as its audio pass is concerned.
                    let start = (n + 1).saturating_sub(STEP);
                    self.step_target[self.steps_in_call] = (start, self.target);
                    self.steps_in_call += 1;
                }
            } else if !self.p.algorithm1 && self.in_step == 0 {
                self.on_step(level(self.ema));
            }
            self.in_step = (self.in_step + 1) % STEP;
        }
        if self.events != 0 {
            if !self.p.algorithm1 {
                self.arbitrate();
            }
            self.peak = f64::NEG_INFINITY;
            self.events = 0;
        }
        if self.clear_due {
            self.clear_due = false;
            self.z = [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS];
        }
        let mut next = 0;
        if self.p.algorithm1 {
            self.target = start_target;
        }
        for n in 0..frames {
            if next < self.steps_in_call && self.step_target[next].0 == n {
                self.target = self.step_target[next].1;
                next += 1;
            }
            self.gain += self.glide * (self.target - self.gain);
            let g = self.gain as f32;
            for (ch, b) in bufs.iter_mut().enumerate().take(nch) {
                let Some(s) = b.get_mut(n) else { continue };
                let x = if s.is_finite() { *s } else { 0.0 };
                let y = if la == 0 {
                    x
                } else {
                    let slot = &mut self.ring[ch * la + self.pos];
                    core::mem::replace(slot, x)
                };
                *s = y * g;
            }
            if la != 0 {
                self.pos = (self.pos + 1) % la;
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
    fn gain_glides_linearly_with_a_200_hz_one_pole() {
        let p = params(0);
        let mut r = Run::new(&p, 48_000.0, 1);
        let from = libm::pow(10.0, p.gain_db(p.expected_db) / 20.0);
        // a loud call moves the target; the applied amplitude covers the
        // same share of the remaining distance every frame
        let mut buf = vec![0.5f32; CALL * 3];
        r.process(&mut [&mut buf[..]], CALL * 3);
        let to = r.target;
        assert!((to - from).abs() > 0.1);
        let a = 1.0 - libm::exp(-2.0 * core::f64::consts::PI * 200.0 / 48_000.0);
        assert!((r.glide - a).abs() < 1e-15);
    }

    #[test]
    fn algorithm1_starts_silent_and_steps_its_fast_hysteresis() {
        let p = Params { algorithm1: true, detector_ema_s: 0.040, ..params(0) };
        let mut w = vec![0.0; Params::WORDS];
        p.to_words(&mut w);
        assert_eq!(Params::from_words(&w), Some(p.clone()));
        let mut r = Run::new(&p, 48_000.0, 1);
        assert_eq!(r.candidate, V1_SILENT_DB);
        assert!((r.gain_db() - V1_START_GAIN_DB).abs() < 1e-9);
        assert!((level(r.ema) - V1_DETECTOR_SEED_DB).abs() < 1e-9);
        assert!(!r.clear_due);
        r.fast = -20.0;
        r.arbitrate_step(-14.0); // stored -18 is inside +3 dB
        assert_eq!(r.fast, -20.0);
        r.arbitrate_step(-12.0); // stored -16 clears +3 dB
        assert_eq!(r.fast, -16.0);
        r.arbitrate_step(-30.0); // stored -34 + 8 is under the held level
        assert_eq!(r.fast, -26.0);
        // the target follows the step, not the call boundary
        assert_eq!(r.candidate, -26.0);
        assert!((r.target_db - p.gain_db(-26.0)).abs() < 1e-12);
    }

    #[test]
    fn algorithm1_fast_uses_the_completed_step_maximum() {
        let p = Params { algorithm1: true, detector_ema_s: 0.040, ..params(0) };
        let mut r = Run::new(&p, 48_000.0, 1);
        // Make the detector state equal each sample's power so the reducer
        // is isolated from its time constant. A step-start sampler sees
        // silence; the completed-step maximum sees the final sample.
        r.ema_a = 0.0;
        r.fast = V1_SILENT_DB;
        let mut buf = [0.0f32; STEP];
        buf[STEP - 1] = 1.0;
        r.process(&mut [&mut buf], STEP);
        assert_eq!(r.fast, V1_FAST_STORE_DB);
        assert_eq!(r.step_peak, f64::NEG_INFINITY);
    }

    #[test]
    fn algorithm1_conditions_the_long_path_against_the_fast_level() {
        let p = Params { algorithm1: true, detector_ema_s: 0.040, ..params(0) };
        let mut r = Run::new(&p, 48_000.0, 1);
        // a reading under the held fast level passes, though the 512-frame
        // level is far under it
        r.fast = -12.0;
        r.pending = Some((-40.0, V1_SILENT_DB, -20.0));
        r.arbitrate_step(-8.0);
        assert_eq!(r.held, -20.0);
        r.pending = Some((-60.0, V1_SILENT_DB, -18.0));
        r.arbitrate_step(-8.0);
        assert_eq!(r.held, -18.0);
        // the fast level releases under the next reading: hold that reading
        // (the smaller of it and the fast level it overtook), not the level
        // the 512-frame path fell to
        r.pending = Some((-60.0, V1_SILENT_DB, -17.0));
        r.arbitrate_step(-30.0);
        assert_eq!(r.fast, -26.0);
        assert_eq!(r.held, -17.0);
        // a burst whose fast level stays under a newer reading keeps the hold
        r.pending = Some((-5.0, V1_SILENT_DB, -15.0));
        r.arbitrate_step(-22.0);
        assert_eq!(r.fast, -26.0);
        assert_eq!(r.held, -17.0);
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
    fn table_reads_are_a_clamped_cubic_b_spline() {
        let p = params(0);
        let t = &p.table;
        let at = |i: usize| NODE_LO_DB + NODE_STEP_DB * i as f64;
        // flat ends, clamped past the grid
        assert!((p.gain_db(-200.0) - t[0]).abs() < 1e-12);
        assert!((p.gain_db(50.0) - t[NODES - 1]).abs() < 1e-12);
        // a straight run of nodes (12..14 fall 0.5 dB/dB) reads as the line
        assert!((p.gain_db(at(13)) - t[13]).abs() < 1e-12);
        let mid = p.gain_db(at(13) + 2.75);
        assert!((mid - 0.5 * (t[13] + t[14])).abs() < 1e-12);
        // the knee at node 11 is rounded: (left + 4 node + right) / 6
        let knee = (t[10] + 4.0 * t[11] + t[12]) / 6.0;
        assert!((p.gain_db(at(11)) - knee).abs() < 1e-12);
        assert!(knee < t[11]);
        // continuous across a node boundary
        let e = 1e-9;
        assert!((p.gain_db(at(11) - e) - p.gain_db(at(11) + e)).abs() < 1e-6);
    }

    #[test]
    fn starts_at_the_expected_loudness_gain_and_delays_by_the_latency() {
        let p = params(144);
        let mut r = Run::new(&p, 48_000.0, 2);
        assert!((r.gain_db() - p.gain_db(p.expected_db)).abs() < 1e-12);
        assert_eq!(r.latency(), 144);
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
    fn the_current_call_sets_its_own_gain_without_a_transport_call() {
        let p = params(0);
        let mut quiet = vec![0.001f32; CALL];
        let mut onset = quiet.clone();
        onset[CALL / 2..].fill(1.0);
        let mut a = Run::new(&p, 48_000.0, 1);
        let mut b = Run::new(&p, 48_000.0, 1);
        a.process(&mut [&mut quiet], CALL);
        b.process(&mut [&mut onset], CALL);
        // Identical input at frame zero, different later input in this call:
        // the later metered peak already affects frame zero's applied gain.
        assert!(onset[0] > 0.0);
        assert!(onset[0] < quiet[0]);
        assert_eq!(a.latency(), 0);
        assert_eq!(b.latency(), 0);
    }

    #[test]
    fn short_calls_keep_only_the_lookahead_and_reset_discards_old_audio() {
        for la in [0, 1, 144] {
            let p = Params { table: [0.0; NODES], lookahead: la, ..params(la) };
            let mut r = Run::new(&p, 48_000.0, 1);
            let mut output = Vec::new();
            let mut frame = 0;
            for q in [1, 7, 64, 333, 512, 19] {
                let mut x: Vec<f32> = (frame..frame + q).map(|i| if i == 0 { 0.25 } else { 0.0 }).collect();
                r.process(&mut [&mut x], q);
                output.extend(x);
                frame += q;
            }
            assert_eq!(r.latency(), la);
            for (i, x) in output.iter().enumerate() {
                assert_eq!(*x, if i == la { 0.25 } else { 0.0 });
            }
            let mut old = vec![0.5; CALL];
            r.process(&mut [&mut old], CALL);
            r.reset();
            let mut silence = vec![0.0; CALL];
            r.process(&mut [&mut silence], CALL);
            assert!(silence.iter().all(|v| *v == 0.0));
        }
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
            assert_eq!(m.update(power_of(-80.0)), before);
        }
        assert_eq!(m.reading, before);
    }

    #[test]
    fn a_level_counts_one_bin_below_the_nearest_index_grid_point() {
        // -20 sits at grid point 800.8 -> 801, so it counts in bin 800
        assert_eq!(bin_of(-20.0), 800);
        // the clamps: the floor of the scale and anything loud
        assert_eq!(bin_of(-100.0), 0);
        assert_eq!(bin_of(-150.0), 0);
        assert_eq!(bin_of(0.0), BINS - 1);
        assert_eq!(bin_of(6.0), BINS - 1);
        // each bin still represents its own 0.1 dB step
        let m = LongTerm::new(11.0);
        assert!((loudness(m.energy[800]) + 20.0).abs() < 1e-9);
    }

    #[test]
    fn the_median_takes_every_block_and_flips_at_its_middle() {
        let p = params(0);
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut buf = vec![0.3f32; CALL];
        r.process(&mut [&mut buf], CALL);
        // frame 0's block joins as well
        assert_eq!(r.median.ring.iter().filter(|v| **v != p.expected_db).count(), 1);
        let mut m = Median::new(93.75, false);
        m.reset(-20.0);
        for _ in 0..MEDIAN_AT {
            assert_eq!(m.push(-10.0), -20.0);
        }
        // the 48th louder level of 95 moves the middle
        assert_eq!(m.push(-10.0), -15.0);
    }

    #[test]
    fn the_long_reading_reaches_the_arbitration_three_windows_late() {
        let p = params(0);
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut readings = vec![];
        let mut buf = vec![0.0f32; 4096];
        for k in 0..12 {
            // a loud square wave: every window counts and moves the reading
            buf.iter_mut().enumerate().for_each(|(i, v)| *v = if i % 2 == 0 { 0.3 } else { -0.3 });
            r.process(&mut [&mut buf], 4096);
            // chunk k ran the window that starts it (frame 4096 k)
            readings.push(r.long.reading);
            let seen = if k < LONG_DELAY { p.expected_db } else { readings[k - LONG_DELAY] };
            assert_eq!(r.long_out, seen, "window {k}");
        }
        assert!(readings[11] > readings[1] + 1.0);
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
    fn fast_release_approaches_the_dead_band_edge_at_the_history_rate() {
        let p = Params { expected_db: -20.0, fast_deadband_db: 7.0, ..params(0) };
        let mut fast = Run::new(&p, 48_000.0, 1);
        fast.peak = -42.0; // call target: -50; release edge: -43
        fast.events = 8;
        fast.history = 1.0;
        fast.arbitrate();

        let span = 48_000.0 / 8.0;
        let pole = libm::exp(-1000.0 / span);
        let expected = -20.0 + (1.0 - pole) * (-43.0 - -20.0);
        assert!((fast.fast - expected).abs() < 1e-12, "{} vs {}", fast.fast, expected);

        let mut slow = Run::new(&p, 48_000.0, 1);
        slow.peak = -42.0;
        slow.events = 8;
        slow.history = FAST_HISTORY;
        slow.arbitrate();
        assert!(fast.fast < slow.fast, "{} vs {}", fast.fast, slow.fast);
    }

    #[test]
    fn a_settled_low_rise_asks_for_one_coordinate_move() {
        let mut r = LowRise::new(750.0);
        assert_eq!(r.update(-70.0, 8, -20.0), 0.0);
        // a jump is not settled
        assert_eq!(r.update(-65.0, 8, -20.0), 0.0);
        // the next, settled call more than 4 dB over the follower moves it
        assert_eq!(r.update(-65.0, 8, -20.0), RISE_SHIFT_DB);
        assert_eq!(r.update(-65.0, 8, -20.0), 0.0);
        // never within 10 dB of the long reading
        let mut r = LowRise::new(750.0);
        r.update(-70.0, 8, -60.0);
        r.update(-65.0, 8, -60.0);
        assert_eq!(r.update(-65.0, 8, -60.0), 0.0);
    }

    #[test]
    fn the_move_shifts_every_retained_long_state() {
        let p = Params { expected_db: -20.0, ..params(0) };
        let mut r = Run::new(&p, 48_000.0, 1);
        r.rise.follow = -70.0;
        r.rise.last = -65.0;
        r.peak = -65.0;
        r.events = 8;
        let before = r.long.hist.iter().position(|h| *h > 0.0).unwrap();
        r.arbitrate();
        assert_eq!(r.long.hist.iter().position(|h| *h > 0.0).unwrap(), before - 15);
        assert_eq!(r.long.reading, -21.5);
        assert_eq!(r.long_line, [-21.5; LONG_DELAY]);
        assert_eq!(r.long_out, -21.5);
        assert_eq!(r.held, -23.0);
    }

    #[test]
    fn the_detector_state_clears_again_after_the_first_call() {
        let p =
            Params { sections: 1, sos: [[1.0, -2.0, 1.0, -1.9, 0.9025], [0.0; 5], [0.0; 5], [0.0; 5]], ..params(0) };
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut buf = vec![0.25f32; CALL - 1];
        r.process(&mut [&mut buf], CALL - 1);
        // Reset clearing follows the actual render call, even a short one.
        assert!(!r.clear_due);
        assert_eq!(r.z[0][0], [0.0; 2]);
        let mut more = vec![0.25f32; CALL];
        r.process(&mut [&mut more], CALL);
        assert_ne!(r.z[0][0], [0.0; 2]);
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
