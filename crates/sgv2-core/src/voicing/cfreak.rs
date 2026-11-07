// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The linked look-ahead peak limiter that macOS runs per driver group
//! after the crossover (`aufx/cfrk`, "ControlFreak"): one gain for all the
//! channels it is given, so a group's drivers keep their balance.
//!
//! Behaviour, per frame `k` (`fs` the rate, `L` the look-ahead in frames,
//! `H = L + 1 + hold` frames):
//!
//! * detector `p[k]` = the largest `|x|` over the channels; in true-peak
//!   mode each channel's `|x|` is raised to the largest of its four-times
//!   interpolated values (ITU-R BS.1770-4 Annex 2 interpolator, no added
//!   delay) scaled by [`TRUE_PEAK_SCALE`];
//! * target `t = min(1, C / p)` with the ceiling `C = 10^(threshold/20)`
//!   times [`CEILING_TRIM`];
//! * the fast stage carries at most `MinFastGain` dB (`F = 10^(-mfg/20)`);
//!   what is left goes to a slow gain `s`: its target `u` is
//!   `min(1, t / F)` held at its minimum over the last `H + 1` frames; `s`
//!   drops to `u` at once and otherwise moves towards it with
//!   [`TAU_SLOW_S`];
//! * the combined target `s * max(t, F)` is held at its minimum over the
//!   last `H + 1` frames and smoothed by a normalised `L`-tap sine window
//!   (`w[j] = sin(pi (j + 1) / (L + 1))`);
//! * `y_c[k] = g[k] * x_c[k - L]` for every channel.
//!
//! The hold makes the window cover every delayed sample under it, so the
//! output's sample peak never exceeds `C`.
//!
//! State is f64; the unit keeps its slow gain in f32, which moves it by
//! less than 0.005 dB over a 10 s release.
//!
//! Output saturation (algorithm version 1, sine flavor), when switched on,
//! runs per channel ahead of the limiter's detector and delay line:
//!
//! * the signal is oversampled [`OVERSAMPLE`] times through a linear-phase
//!   windowed-sinc low pass `h` ([`FIR_TAPS`] taps at the oversampled rate,
//!   [`FIR_HALF_BASE`] base frames of delay, Kaiser window [`WINDOW_BETA`]
//!   over [`WINDOW_HALF_OS`] oversampled samples each side), cut at
//!   `19 kHz - 1 kHz * mode`
//!   (rates up to 48 kHz) or `fs / 4` (88.2 kHz and up);
//! * on the oversampled signal a de-esser cut `D` (the saved frequency
//!   `f0`, strength `S` dB and Q:
//!   `D(s) = (s^2 + s w0/Q + w0^2) / (s^2 + s w0/(Q g) + w0^2)`,
//!   `g = 10^(-S/20)`, bilinear at the oversampled rate) runs before the
//!   shaper and its exact inverse after it, so it only shapes how hard each
//!   band is driven;
//! * the shaper is `sin(k u) / k`, unity slope at zero; `k` is a per
//!   channel drive between [`K_MIN`] (no high-band content) and 1 set once
//!   per [`DRIVE_BLOCK`] frames from the ratio of the block's RMS through a
//!   fixed high-band weighting to its plain RMS (`k^2 = DRIVE_C * ratio`),
//!   smoothed in dB ([`DRIVE_SMOOTH`] of the way per block);
//! * the same low pass `h` reconstructs and decimates: a low-level impulse
//!   comes out as `h * h` sampled at the base rate, centred
//!   [`SATURATION_DELAY`] frames late.

use crate::MAX_CHANNELS;
use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

/// True-peak mode scales the interpolated values by +0.688 dB.
pub const TRUE_PEAK_SCALE_DB: f64 = 0.688;
/// The unit's sample ceiling sits 0.00002 dB under the threshold.
pub const CEILING_TRIM_DB: f64 = -0.00002;
/// Slow gain time constant (towards its target, from below).
///
/// TODO(SPEC): the unit releases the slow gain with a 0.27 s constant for a
/// while after an over and with this 10 s one afterwards; how long the fast
/// phase lasts depends on its render blocks and on signal history (about
/// 44 ms after an isolated over, about 9 ms after some sustained ones, none
/// after others). Until that is pinned the slow constant is used
/// throughout, which can only hold the gain lower than the unit's. This
/// only matters past `MinFastGain` dB of limiting.
pub const TAU_SLOW_S: f64 = 10.0;
/// Output saturation oversampling factor.
pub const OVERSAMPLE: usize = 3;
/// Delay of each of the two saturation low passes, in base frames.
pub const FIR_HALF_BASE: usize = 12;
/// Taps of the saturation low pass at the oversampled rate.
pub const FIR_TAPS: usize = 2 * FIR_HALF_BASE * OVERSAMPLE + 1;
/// The low pass's window: Kaiser `I0(beta sqrt(1 - (t/T)^2)) / I0(beta)`
/// over the 73 taps, `t` in oversampled samples from the centre.
///
/// TODO(SPEC): pinned black-box. The single filter, factored out of the
/// unit's low-level impulse at 88.2/96/192 kHz (a 25-tap half band), has
/// the same window in base frames at every rate; this Kaiser matches the
/// impulses at 44.1/48/96 kHz to 1.5e-5 and the 48 kHz stop band within
/// 0.06 dB (20 kHz: 0.004 dB). No standard window (Chebyshev, DPSS,
/// cosine sums, Kaiser-formula beta) matches exactly; the recipe is open.
pub const WINDOW_BETA: f64 = 11.143;
pub const WINDOW_HALF_OS: f64 = 38.0;
/// Saturation latency (up plus down low pass), base frames.
pub const SATURATION_DELAY: usize = 2 * FIR_HALF_BASE;
/// Shaper drive bounds: `sin(k u) / k` with `k` in `[K_MIN, 1]`.
pub const K_MIN: f64 = 9.0 / 16.0;
/// The drive is set once per this many frames.
pub const DRIVE_BLOCK: usize = 512;
/// Fraction of the dB distance to the block's target the drive moves.
pub const DRIVE_SMOOTH: f64 = 0.1;
/// The drive (dB) a fresh or reset unit starts from (black-box: tone
/// onsets at 100 Hz and 1 kHz, 0 to -24 dBFS, all give 9.0 dB).
pub const DRIVE_START_DB: f64 = 9.0;
/// Blocks whose input does not move the drive after a start. The unit
/// holds the start drive over its first two blocks and moves the third
/// block's drive by that block's own target; a streaming run knows a
/// block's target only once the block is over, so it moves the third
/// block's drive by the second block's target (one complete block
/// ignored).
pub const DRIVE_WARMUP_BLOCKS: usize = 1;
/// `k^2 = DRIVE_C * rms(weighted) / rms(input)`.
///
/// TODO(SPEC): the weighting's exact sections are open. Black-box (single
/// tones and two-tone ladders 3-20 kHz, 44.1/48/96 kHz) it is a fourth
/// order high-band weighting peaking near 10 kHz; carried as two RBJ high
/// passes at [`DRIVE_HP_HZ`] and one RBJ low pass at [`DRIVE_LP_HZ`], which
/// matches the measured weighting within 0.27 dB (1.5 % of `k`).
pub const DRIVE_C: f64 = 31.0381;
pub const DRIVE_HP_HZ: f64 = 7_073.447;
pub const DRIVE_HP_Q: f64 = 0.8625;
pub const DRIVE_LP_HZ: f64 = 13_166.079;
pub const DRIVE_LP_Q: f64 = 0.6428;

/// ITU-R BS.1770-4 Annex 2 true-peak interpolator, first two phases (the
/// other two are their mirror images).
const BS1770: [[f64; 12]; 2] = [
    [
        0.0017089843750,
        0.0109863281250,
        -0.0196533203125,
        0.0332031250000,
        -0.0594482421875,
        0.1373291015625,
        0.9721679687500,
        -0.1022949218750,
        0.0476074218750,
        -0.0266113281250,
        0.0148925781250,
        -0.0083007812500,
    ],
    [
        -0.0291748046875,
        0.0292968750000,
        -0.0517578125000,
        0.0891113281250,
        -0.1665039062500,
        0.4650878906250,
        0.7797851562500,
        -0.2003173828125,
        0.1015625000000,
        -0.0582275390625,
        0.0330810546875,
        -0.0189208984375,
    ],
];
const TAPS: usize = 12;

/// The longest look-ahead and hold the unit offers (1 ms, 20 ms), at up to
/// 192 kHz.
const MAX_LOOKAHEAD_S: f64 = 0.001;
const MAX_HOLD_S: f64 = 0.020;
const MAX_FS: f64 = 192_000.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    pub lookahead_s: f64,
    /// The unit's "fast gain release time": a hold, in seconds.
    pub hold_s: f64,
    pub threshold_db: f64,
    pub min_fast_gain_db: f64,
    pub true_peak: bool,
    /// Output saturation oversampling mode (zero disables it; modes 1-7
    /// are carried with flavor 2, the sine shaper).
    pub saturation_mode: u32,
    pub saturation_flavor: u32,
    /// The saved saturation amount (fraction). Dormant: the version-1 sine
    /// shaper's output does not depend on it (black-box, 0-100 %).
    pub saturation_mix: f64,
    pub input_gain_db: f64,
    pub output_gain_db: f64,
    /// De-esser centre frequency, strength (dB of cut) and Q; used only
    /// with output saturation.
    pub deesser_hz: f64,
    pub deesser_db: f64,
    pub deesser_q: f64,
}

impl Params {
    pub const WORDS: usize = 13;
    /// Words without the de-esser (states without output saturation).
    pub const PLAIN_WORDS: usize = 10;
    pub const LEGACY_WORDS: usize = 8;

    /// The number of words this state needs.
    pub fn words(&self) -> usize {
        if self.saturation_mode != 0 {
            Self::WORDS
        } else if self.input_gain_db == 0.0 && self.output_gain_db == 0.0 {
            Self::LEGACY_WORDS
        } else {
            Self::PLAIN_WORDS
        }
    }

    pub fn to_words(&self, out: &mut [f64]) {
        out[..Self::WORDS].copy_from_slice(&[
            self.lookahead_s,
            self.hold_s,
            self.threshold_db,
            self.min_fast_gain_db,
            if self.true_peak { 1.0 } else { 0.0 },
            self.saturation_mode as f64,
            self.saturation_flavor as f64,
            self.saturation_mix,
            self.input_gain_db,
            self.output_gain_db,
            self.deesser_hz,
            self.deesser_db,
            self.deesser_q,
        ]);
    }

    pub fn from_words(w: &[f64]) -> Option<Params> {
        if ![Self::LEGACY_WORDS, Self::PLAIN_WORDS, Self::WORDS].contains(&w.len()) || w.iter().any(|v| !v.is_finite())
        {
            return None;
        }
        let p = Params {
            lookahead_s: w[0],
            hold_s: w[1],
            threshold_db: w[2],
            min_fast_gain_db: w[3],
            true_peak: match w[4] {
                0.0 => false,
                1.0 => true,
                _ => return None,
            },
            saturation_mode: match w[5] {
                m if m == libm::trunc(m) && (0.0..=7.0).contains(&m) => m as u32,
                _ => return None,
            },
            saturation_flavor: match w[6] {
                0.0 => 0,
                2.0 => 2,
                _ => return None,
            },
            saturation_mix: w[7],
            input_gain_db: w.get(8).copied().unwrap_or(0.0),
            output_gain_db: w.get(9).copied().unwrap_or(0.0),
            deesser_hz: w.get(10).copied().unwrap_or(0.0),
            deesser_db: w.get(11).copied().unwrap_or(0.0),
            deesser_q: w.get(12).copied().unwrap_or(0.0),
        };
        let ok = (-120.0..=40.0).contains(&p.input_gain_db)
            && (-120.0..=40.0).contains(&p.output_gain_db)
            && (0.0..=MAX_LOOKAHEAD_S).contains(&p.lookahead_s)
            && (0.0..=MAX_HOLD_S).contains(&p.hold_s)
            && (-40.0..=6.0).contains(&p.threshold_db)
            && (0.0..=40.0).contains(&p.min_fast_gain_db);
        let saturation_ok = if p.saturation_mode == 0 {
            p.saturation_flavor == 0
                && p.saturation_mix == 0.0
                && p.deesser_hz == 0.0
                && p.deesser_db == 0.0
                && p.deesser_q == 0.0
        } else {
            // A saturating state must carry its de-esser (the 10-word form
            // of earlier artifacts does not).
            w.len() == Self::WORDS
                && p.saturation_flavor == 2
                && (0.0..=1.0).contains(&p.saturation_mix)
                && (20.0..=20_000.0).contains(&p.deesser_hz)
                && (0.0..=40.0).contains(&p.deesser_db)
                && (0.05..=20.0).contains(&p.deesser_q)
        };
        (ok && saturation_ok).then_some(p)
    }
}

/// One RBJ-form biquad, transposed direct form II.
#[derive(Clone, Copy, Default)]
struct Biquad {
    b: [f64; 3],
    a: [f64; 2],
}

#[derive(Clone, Copy, Default)]
struct BiquadState {
    z: [f64; 2],
}

impl Biquad {
    fn new(b: [f64; 3], a: [f64; 3]) -> Biquad {
        Biquad { b: [b[0] / a[0], b[1] / a[0], b[2] / a[0]], a: [a[1] / a[0], a[2] / a[0]] }
    }

    fn rbj(high: bool, f0: f64, q: f64, fs: f64) -> Biquad {
        let w0 = 2.0 * core::f64::consts::PI * f0 / fs;
        let (s, c) = (libm::sin(w0), libm::cos(w0));
        let al = s / (2.0 * q);
        let b = if high {
            [(1.0 + c) / 2.0, -(1.0 + c), (1.0 + c) / 2.0]
        } else {
            [(1.0 - c) / 2.0, 1.0 - c, (1.0 - c) / 2.0]
        };
        Biquad::new(b, [1.0 + al, -2.0 * c, 1.0 - al])
    }

    fn run(&self, st: &mut BiquadState, x: f64) -> f64 {
        let y = self.b[0] * x + st.z[0];
        st.z[0] = self.b[1] * x - self.a[0] * y + st.z[1];
        st.z[1] = self.b[2] * x - self.a[1] * y;
        y
    }
}

/// The de-esser cut and its inverse at the oversampled rate `fs`.
fn deesser(f0: f64, strength_db: f64, q: f64, fs: f64) -> (Biquad, Biquad) {
    let g = libm::pow(10.0, -strength_db / 20.0);
    let w0 = 2.0 * core::f64::consts::PI * f0.min(0.49 * fs) / fs;
    let (s, c) = (libm::sin(w0), libm::cos(w0));
    let al = s / (2.0 * q);
    let num = [1.0 + al, -2.0 * c, 1.0 - al];
    let den = [1.0 + al / g, -2.0 * c, 1.0 - al / g];
    (Biquad::new(num, den), Biquad::new(den, num))
}

/// Modified Bessel function of the first kind, order zero (power series).
fn bessel_i0(x: f64) -> f64 {
    let q = x * x / 4.0;
    let (mut term, mut sum) = (1.0, 1.0);
    for k in 1..200 {
        term *= q / (k * k) as f64;
        sum += term;
        if term < sum * 1e-17 {
            break;
        }
    }
    sum
}

/// The saturation low pass at the oversampled rate, unity DC gain.
fn saturation_fir(mode: u32, fs: f64) -> [f64; FIR_TAPS] {
    let fc = if fs >= 88_000.0 { fs / 4.0 } else { 19_000.0 - 1_000.0 * f64::from(mode) };
    let fs_os = fs * OVERSAMPLE as f64;
    let mut h = [0.0; FIR_TAPS];
    let mid = (FIR_TAPS / 2) as f64;
    for (i, v) in h.iter_mut().enumerate() {
        let t = i as f64 - mid;
        let a = 2.0 * fc / fs_os;
        let sinc =
            if t == 0.0 { 1.0 } else { libm::sin(core::f64::consts::PI * a * t) / (core::f64::consts::PI * a * t) };
        let r = t / WINDOW_HALF_OS;
        let win = bessel_i0(WINDOW_BETA * libm::sqrt((1.0 - r * r).max(0.0))) / bessel_i0(WINDOW_BETA);
        *v = a * sinc * win;
    }
    let sum: f64 = h.iter().sum();
    h.iter_mut().for_each(|v| *v /= sum);
    h
}

/// Per-channel output saturation state.
#[derive(Clone)]
struct Saturator {
    /// Base-rate input history (newest at `x_at`).
    x: [f64; PHASE_TAPS],
    x_at: usize,
    /// Oversampled shaper output history (newest at `v_at`).
    v: [f64; V_RING],
    v_at: usize,
    pre: BiquadState,
    post: BiquadState,
    det: [BiquadState; 3],
    /// Block energies of the input and of the weighted input.
    ex: f64,
    eg: f64,
    /// Smoothed drive, dB.
    drive_db: f64,
    k: f64,
}

const PHASE_TAPS: usize = FIR_TAPS.div_ceil(OVERSAMPLE);
/// The decimator reads `v[3n - i]`, `i < FIR_TAPS`, after `v[3n + 2]`.
const V_RING: usize = FIR_TAPS + OVERSAMPLE - 1;

impl Saturator {
    fn new() -> Saturator {
        Saturator {
            x: [0.0; PHASE_TAPS],
            x_at: 0,
            v: [0.0; V_RING],
            v_at: 0,
            pre: BiquadState::default(),
            post: BiquadState::default(),
            det: [BiquadState::default(); 3],
            ex: 0.0,
            eg: 0.0,
            drive_db: DRIVE_START_DB,
            k: 1.0,
        }
    }
}

/// Shared (per box) saturation coefficients.
struct Saturation {
    h: [f64; FIR_TAPS],
    pre: Biquad,
    post: Biquad,
    det: [Biquad; 3],
    pos: usize,
    /// Drive blocks completed since the start or a reset.
    blocks: usize,
    ch: Vec<Saturator>,
}

impl Saturation {
    fn new(p: &Params, fs: f64, channels: usize) -> Saturation {
        let fs_os = fs * OVERSAMPLE as f64;
        let (pre, post) = deesser(p.deesser_hz, p.deesser_db, p.deesser_q, fs_os);
        let hp = Biquad::rbj(true, DRIVE_HP_HZ.min(0.45 * fs), DRIVE_HP_Q, fs);
        let lp = Biquad::rbj(false, DRIVE_LP_HZ.min(0.45 * fs), DRIVE_LP_Q, fs);
        Saturation {
            h: saturation_fir(p.saturation_mode, fs),
            pre,
            post,
            det: [hp, hp, lp],
            pos: 0,
            blocks: 0,
            ch: vec![Saturator::new(); channels],
        }
    }

    fn reset(&mut self) {
        self.pos = 0;
        self.blocks = 0;
        self.ch.iter_mut().for_each(|c| *c = Saturator::new());
    }

    /// Ends a drive block: each channel's drive moves towards the block's
    /// target (the first [`DRIVE_WARMUP_BLOCKS`] after a start leave it).
    ///
    /// The unit sets a block's drive from that block's own input before
    /// rendering it; a streaming run only knows it once the block is over,
    /// so here it takes effect one block later.
    fn end_block(&mut self) {
        let warm = self.blocks >= DRIVE_WARMUP_BLOCKS;
        self.blocks = self.blocks.saturating_add(1);
        for c in &mut self.ch {
            if warm && c.ex > 0.0 {
                let k2 = DRIVE_C * libm::sqrt(c.eg / c.ex);
                let target = 10.0 * libm::log10(k2.max(1e-30));
                c.drive_db += DRIVE_SMOOTH * (target - c.drive_db);
                c.k = libm::pow(10.0, c.drive_db / 20.0).clamp(K_MIN, 1.0);
            }
            c.ex = 0.0;
            c.eg = 0.0;
        }
        self.pos = 0;
    }

    /// One base-rate sample of channel `c`; returns the sample delayed by
    /// [`SATURATION_DELAY`].
    fn run(&mut self, c: usize, x: f64) -> f64 {
        let h = &self.h;
        let s = &mut self.ch[c];
        let mut g = x;
        for (bq, st) in self.det.iter().zip(s.det.iter_mut()) {
            g = bq.run(st, g);
        }
        s.ex += x * x;
        s.eg += g * g;
        s.x_at = (s.x_at + 1) % PHASE_TAPS;
        s.x[s.x_at] = x;
        let k = s.k;
        for phase in 0..OVERSAMPLE {
            // u[3n + phase] = 3 * sum_j h[phase + 3 j] x[n - j]
            let mut u = 0.0;
            let mut i = phase;
            let mut j = 0;
            while i < FIR_TAPS {
                u += h[i] * s.x[(s.x_at + PHASE_TAPS - j) % PHASE_TAPS];
                i += OVERSAMPLE;
                j += 1;
            }
            u *= OVERSAMPLE as f64;
            let d = self.pre.run(&mut s.pre, u);
            let shaped = libm::sin(k * d) / k;
            let v = self.post.run(&mut s.post, shaped);
            s.v_at = (s.v_at + 1) % V_RING;
            s.v[s.v_at] = v;
        }
        // y[n] = sum_i h[i] v[3n - i]; v[3n] was pushed OVERSAMPLE - 1
        // samples before the newest.
        let newest_3n = (s.v_at + V_RING - (OVERSAMPLE - 1)) % V_RING;
        let mut y = 0.0;
        for (i, hv) in h.iter().enumerate() {
            y += hv * s.v[(newest_3n + V_RING - i) % V_RING];
        }
        y
    }
}

/// Sliding minimum over the last `cap` values (monotonic queue in a fixed
/// ring).
struct MinHold {
    val: Box<[f64]>,
    at: Box<[u64]>,
    head: usize,
    len: usize,
    cap: usize,
    now: u64,
}

impl MinHold {
    fn new(cap: usize) -> Self {
        MinHold {
            val: vec![0.0; cap].into_boxed_slice(),
            at: vec![0; cap].into_boxed_slice(),
            head: 0,
            len: 0,
            cap,
            now: 0,
        }
    }

    fn reset(&mut self) {
        self.head = 0;
        self.len = 0;
        self.now = 0;
    }

    /// Pushes `v` and returns the minimum of the last `cap` values pushed
    /// (values before the first push count as 1).
    fn push(&mut self, v: f64) -> f64 {
        let cap = self.cap;
        // drop values that have left the window
        while self.len > 0 && self.now - self.at[self.head] >= cap as u64 {
            self.head = (self.head + 1) % cap;
            self.len -= 1;
        }
        // drop values no smaller than v from the back
        while self.len > 0 {
            let back = (self.head + self.len - 1) % cap;
            if self.val[back] >= v {
                self.len -= 1;
            } else {
                break;
            }
        }
        let slot = (self.head + self.len) % cap;
        self.val[slot] = v;
        self.at[slot] = self.now;
        self.len += 1;
        self.now += 1;
        let m = self.val[self.head];
        if self.now < cap as u64 {
            m.min(1.0)
        } else {
            m
        }
    }
}

pub struct Run {
    channels: usize,
    lookahead: usize,
    true_peak: bool,
    ceiling: f64,
    floor: f64,
    tp_scale: f64,
    a_slow: f64,
    /// Window weights, `w[j]` applies to the held target `j` frames ago.
    weight: Box<[f64]>,
    /// Ring of the last `lookahead` held targets.
    held: Box<[f64]>,
    held_at: usize,
    slow_target: MinHold,
    target: MinHold,
    slow: f64,
    /// Per channel: the last `lookahead` input samples (ring) and the last
    /// `TAPS` for the interpolator (ring).
    line: Box<[f32]>,
    line_at: usize,
    hist: Box<[f64]>,
    hist_at: usize,
    input_gain: f64,
    output_gain: f64,
    saturation: Option<Box<Saturation>>,
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize) -> Run {
        let fs = if fs.is_finite() { fs.clamp(1.0, MAX_FS) } else { 48_000.0 };
        let channels = channels.clamp(1, MAX_CHANNELS);
        let lookahead = libm::round(p.lookahead_s * fs).max(1.0) as usize;
        let window = lookahead + 2 + libm::round(p.hold_s * fs) as usize;
        let mut weight = vec![0.0; lookahead].into_boxed_slice();
        for (j, w) in weight.iter_mut().enumerate() {
            *w = libm::sin(core::f64::consts::PI * (j + 1) as f64 / (lookahead + 1) as f64);
        }
        let sum: f64 = weight.iter().sum();
        weight.iter_mut().for_each(|w| *w /= sum);
        let coef = |tau: f64| 1.0 - libm::exp(-1.0 / (tau * fs));
        Run {
            channels,
            lookahead,
            true_peak: p.true_peak,
            ceiling: db(p.threshold_db + CEILING_TRIM_DB),
            floor: db(-p.min_fast_gain_db),
            tp_scale: db(TRUE_PEAK_SCALE_DB),
            a_slow: coef(TAU_SLOW_S),
            weight,
            held: vec![1.0; lookahead].into_boxed_slice(),
            held_at: 0,
            slow_target: MinHold::new(window),
            target: MinHold::new(window),
            slow: 1.0,
            line: vec![0.0; lookahead * channels].into_boxed_slice(),
            line_at: 0,
            hist: vec![0.0; TAPS * channels].into_boxed_slice(),
            hist_at: 0,
            input_gain: db(p.input_gain_db),
            output_gain: db(p.output_gain_db),
            saturation: (p.saturation_mode != 0).then(|| Box::new(Saturation::new(p, fs, channels))),
        }
    }

    pub fn reset(&mut self) {
        self.held.iter_mut().for_each(|v| *v = 1.0);
        self.held_at = 0;
        self.slow_target.reset();
        self.target.reset();
        self.slow = 1.0;
        self.line.iter_mut().for_each(|v| *v = 0.0);
        self.line_at = 0;
        self.hist.iter_mut().for_each(|v| *v = 0.0);
        self.hist_at = 0;
        if let Some(s) = &mut self.saturation {
            s.reset();
        }
    }

    pub fn latency(&self) -> usize {
        self.lookahead + if self.saturation.is_some() { SATURATION_DELAY } else { 0 }
    }

    /// The linked gain for this frame, given the newest input frame.
    fn gain(&mut self, peak: f64) -> f64 {
        let t = if peak > self.ceiling { self.ceiling / peak } else { 1.0 };
        let u = self.slow_target.push((t / self.floor).min(1.0));
        if u < self.slow {
            self.slow = u;
        } else {
            self.slow += self.a_slow * (u - self.slow);
        }
        let h = self.target.push(self.slow * t.max(self.floor));
        let l = self.lookahead;
        self.held_at = (self.held_at + 1) % l;
        self.held[self.held_at] = h;
        let mut g = 0.0;
        for (j, w) in self.weight.iter().enumerate() {
            g += w * self.held[(self.held_at + l - j) % l];
        }
        g.min(1.0)
    }

    /// In-place on `channels` buffers of at least `frames` samples.
    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let ch = self.channels.min(bufs.len());
        let frames = bufs[..ch].iter().fold(frames, |n, b| n.min(b.len()));
        let l = self.lookahead;
        for k in 0..frames {
            let mut peak = 0.0f64;
            let mut frame = [0.0f64; MAX_CHANNELS];
            self.hist_at = (self.hist_at + 1) % TAPS;
            if let Some(s) = &mut self.saturation {
                if s.pos == DRIVE_BLOCK {
                    s.end_block();
                }
                s.pos += 1;
            }
            for (c, b) in bufs[..ch].iter().enumerate() {
                let mut x = if b[k].is_finite() { f64::from(b[k]) } else { 0.0 };
                x *= self.input_gain;
                if let Some(s) = &mut self.saturation {
                    x = s.run(c, x);
                }
                frame[c] = x;
                let hist = &mut self.hist[c * TAPS..(c + 1) * TAPS];
                hist[self.hist_at] = x;
                let mut a = libm::fabs(x);
                if self.true_peak {
                    let mut best = 0.0f64;
                    for phase in 0..4 {
                        let mut acc = 0.0;
                        for i in 0..TAPS {
                            let coef = if phase < 2 { BS1770[phase][i] } else { BS1770[3 - phase][TAPS - 1 - i] };
                            acc += coef * hist[(self.hist_at + TAPS - i) % TAPS];
                        }
                        best = best.max(libm::fabs(acc));
                    }
                    a = a.max(self.tp_scale * best);
                }
                peak = peak.max(a);
            }
            let g = self.gain(peak);
            for (c, b) in bufs[..ch].iter_mut().enumerate() {
                let slot = c * l + self.line_at;
                let delayed = self.line[slot];
                self.line[slot] = frame[c] as f32;
                b[k] = (f64::from(delayed) * g * self.output_gain) as f32;
            }
            self.line_at = (self.line_at + 1) % l;
        }
    }
}

fn db(v: f64) -> f64 {
    libm::pow(10.0, v / 20.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;
    use proptest::prelude::*;

    fn params(threshold_db: f64, true_peak: bool) -> Params {
        Params {
            lookahead_s: 0.0005,
            hold_s: 0.0005,
            threshold_db,
            min_fast_gain_db: 20.0,
            true_peak,
            saturation_mode: 0,
            saturation_flavor: 0,
            saturation_mix: 0.0,
            deesser_hz: 0.0,
            deesser_db: 0.0,
            deesser_q: 0.0,
            input_gain_db: 0.0,
            output_gain_db: 0.0,
        }
    }

    fn render(p: &Params, chans: &[Vec<f32>]) -> Vec<Vec<f32>> {
        let mut r = Run::new(p, 48_000.0, chans.len());
        let mut out: Vec<Vec<f32>> = chans.to_vec();
        let mut refs: Vec<&mut [f32]> = out.iter_mut().map(|v| v.as_mut_slice()).collect();
        let n = refs[0].len();
        r.process(&mut refs, n);
        out
    }

    #[test]
    fn gain_stages_surround_the_detector_and_delay() {
        let mut p = params(-12.0, false);
        p.input_gain_db = 6.0;
        p.output_gain_db = -3.0;
        let x: Vec<f32> = (0..2048).map(|k| if k < 512 { 0.6 } else { 0.01 }).collect();
        let scaled: Vec<f32> = x.iter().map(|x| (f64::from(*x) * db(p.input_gain_db)) as f32).collect();
        let y = render(&p, &[x]);
        let mut reference = p;
        reference.input_gain_db = 0.0;
        reference.output_gain_db = 0.0;
        let expected = render(&reference, &[scaled]);
        for (a, b) in y[0].iter().zip(&expected[0]) {
            assert!((f64::from(*a) - f64::from(*b) * db(p.output_gain_db)).abs() < 1e-6);
        }
    }

    #[test]
    fn words_round_trip_and_refuse() {
        let p = params(-2.0, true);
        let mut w = [0.0; Params::WORDS];
        p.to_words(&mut w);
        assert_eq!(Params::from_words(&w), Some(p));
        assert_eq!(Params::from_words(&w[..Params::LEGACY_WORDS]), Some(p));
        w[4] = 0.5;
        assert_eq!(Params::from_words(&w), None);
        w[4] = 0.0;
        w[0] = 0.01;
        assert_eq!(Params::from_words(&w), None);
        assert_eq!(Params::from_words(&w[..4]), None);
    }

    #[test]
    fn below_threshold_is_a_pure_delay() {
        let x: Vec<f32> = (0..2000).map(|k| 0.5 * libm::sin(k as f64 * 0.05) as f32).collect();
        let y = render(&params(0.0, false), &[x.clone(), x.clone()]);
        let r = Run::new(&params(0.0, false), 48_000.0, 2);
        let l = r.latency();
        assert_eq!(l, 24);
        for k in 0..2000 - l {
            assert_eq!(y[0][k + l], x[k]);
            assert_eq!(y[1][k + l], x[k]);
        }
    }

    #[test]
    fn impulse_gain_shape_and_link() {
        // a lone +12 dB sample on channel 0: the linked gain ramps over the
        // look-ahead, holds, ramps back, and channel 1 follows it.
        let n = 400;
        let mut a = vec![0.0f32; n];
        a[100] = 4.0;
        let probe = vec![0.001f32; n];
        let y = render(&params(0.0, false), &[a, probe]);
        let g: Vec<f64> = y[1].iter().map(|v| f64::from(*v) / 0.001).collect();
        let c = db(CEILING_TRIM_DB) / 4.0;
        assert!((g[99] - 1.0).abs() < 1e-6);
        assert!(g[100] < 1.0 && g[100] > 0.99);
        for (k, v) in g.iter().enumerate().take(150).skip(123) {
            assert!((v - c).abs() < 1e-6, "hold at {k}: {v}");
        }
        assert!(g[150] > c && (g[172] - 1.0).abs() > 1e-6 && (g[173] - 1.0).abs() < 1e-6);
        assert!(f64::from(y[0][124]) <= db(CEILING_TRIM_DB) + 1e-6);
    }

    #[test]
    fn slow_gain_takes_the_reduction_past_min_fast_gain() {
        // +32 dB sample: 20 dB fast, 12 dB slow; after the fast stage
        // releases the slow gain is left and recovers slowly.
        let n = 3000;
        let mut a = vec![0.0f32; n];
        a[100] = 40.0;
        let probe = vec![0.001f32; n];
        let y = render(&params(0.0, false), &[a, probe]);
        let g = |k: usize| f64::from(y[1][k]) / 0.001;
        assert!((g(124) - db(CEILING_TRIM_DB) / 40.0).abs() < 1e-6);
        let s = g(180);
        assert!(s > 0.24 && s < 0.26, "{s}");
        assert!(g(2000) > s && g(2000) < 0.5);
    }

    #[test]
    fn true_peak_reads_intersample_peaks() {
        // a held DC level reads 1.0016 x the true-peak scale in true-peak mode
        let n = 600;
        let a = vec![2.0f32; n];
        let probe = vec![0.001f32; n];
        let y = render(&params(0.0, true), &[a, probe]);
        let g = f64::from(y[1][500]) / 0.001;
        let sum: f64 = BS1770[0].iter().sum();
        let want = db(CEILING_TRIM_DB) / (2.0 * sum * db(TRUE_PEAK_SCALE_DB));
        assert!((g - want).abs() < 1e-6, "{g} {want}");
    }

    fn saturating(mode: u32) -> Params {
        let mut p = params(0.0, false);
        p.lookahead_s = 0.001;
        p.saturation_mode = mode;
        p.saturation_flavor = 2;
        p.saturation_mix = 0.3;
        p.deesser_hz = 10_000.0;
        p.deesser_db = 10.0;
        p.deesser_q = 1.0;
        p
    }

    fn tone_gain(p: &Params, f: f64, amp: f64) -> f64 {
        let n = 24_000;
        let x: Vec<f32> =
            (0..n).map(|k| (amp * libm::sin(2.0 * core::f64::consts::PI * f * k as f64 / 48_000.0)) as f32).collect();
        let y = render(p, &[x]);
        let (mut c, mut s) = (0.0, 0.0);
        let l = Run::new(p, 48_000.0, 1).latency();
        for (k, v) in y[0].iter().enumerate().skip(n / 2) {
            let ph = 2.0 * core::f64::consts::PI * f * (k - l) as f64 / 48_000.0;
            c += f64::from(*v) * libm::cos(ph);
            s += f64::from(*v) * libm::sin(ph);
        }
        libm::sqrt(c * c + s * s) * 2.0 / (n / 2) as f64 / amp
    }

    #[test]
    fn saturation_low_level_response_is_linear_phase_and_cut_by_mode() {
        for mode in 1..=7 {
            let p = saturating(mode);
            let r = Run::new(&p, 48_000.0, 1);
            assert_eq!(r.latency(), 48 + SATURATION_DELAY);
            let mut x = vec![0.0f32; 400];
            x[100] = 1e-4;
            let y = render(&p, &[x]);
            let c = 100 + r.latency();
            let sum: f64 = y[0].iter().map(|v| f64::from(*v) / 1e-4).sum();
            assert!((sum - 1.0).abs() < 1e-4, "mode {mode}: dc {sum}");
            for d in 1..30 {
                let (a, b) = (f64::from(y[0][c - d]), f64::from(y[0][c + d]));
                assert!((a - b).abs() < 1e-9, "mode {mode}: asymmetric at {d}: {a} {b} centre {}", y[0][c]);
            }
            // the composite is the low pass applied twice: -12 dB at its cut
            let cut = 19_000.0 - 1_000.0 * f64::from(mode);
            let g = tone_gain(&p, cut, 1e-3);
            assert!((20.0 * libm::log10(g) + 12.04).abs() < 0.2, "mode {mode}: {} dB at {cut}", 20.0 * libm::log10(g));
        }
        assert!(20.0 * libm::log10(tone_gain(&saturating(3), 20_000.0, 1e-3)) < -60.0);
    }

    #[test]
    fn saturation_drive_follows_high_band_content() {
        // a low tone settles at the soft drive, a 5 kHz one at full drive
        // (cut by the de-esser), so the 5 kHz tone compresses less
        let mut p = saturating(3);
        p.threshold_db = 6.0;
        let comp = |f: f64, d: f64| 20.0 * libm::log10(tone_gain(&p, f, 0.5) / tone_gain(&p, f, 1e-3)) - d;
        let lf = comp(300.0, 0.0);
        // sin(k u) / k with k = 9/16 at amplitude 0.5: 2 J1(z) / z
        let z: f64 = K_MIN * 0.5;
        let want = 20.0 * libm::log10(1.0 - z * z / 8.0 + z * z * z * z / 192.0);
        assert!((lf - want).abs() < 0.002, "{lf} {want}");
        let mut s = Saturation::new(&p, 48_000.0, 1);
        for k in 0..48_000 {
            if s.pos == DRIVE_BLOCK {
                s.end_block();
            }
            s.pos += 1;
            s.run(0, 0.5 * libm::sin(2.0 * core::f64::consts::PI * 5_000.0 * k as f64 / 48_000.0));
        }
        assert_eq!(s.ch[0].k, 1.0);
    }

    #[test]
    fn non_finite_input_is_silence() {
        let mut a = vec![0.1f32; 64];
        a[10] = f32::NAN;
        a[11] = f32::INFINITY;
        let y = render(&params(0.0, true), &[a]);
        assert!(y[0].iter().all(|v| v.is_finite() && v.abs() <= 0.1));
    }

    proptest! {
        #[test]
        fn sample_ceiling_holds(
            thr in -6.0f64..0.0,
            tp in any::<bool>(),
            mfg in 3.0f64..30.0,
            hold in 0.0f64..0.002,
            seed in any::<u64>(),
            level_db in -10.0f64..40.0,
            ch in 1usize..5,
        ) {
            let p = Params {
                lookahead_s: 0.0005,
                hold_s: hold,
                threshold_db: thr,
                min_fast_gain_db: mfg,
                true_peak: tp,
                saturation_mode: 0,
                saturation_flavor: 0,
                saturation_mix: 0.0,
                deesser_hz: 0.0,
                deesser_db: 0.0,
                deesser_q: 0.0,
            input_gain_db: 0.0,
            output_gain_db: 0.0,
            };
            let mut s = seed | 1;
            let amp = db(level_db);
            let chans: Vec<Vec<f32>> = (0..ch).map(|_| (0..1500).map(|k| {
                s ^= s << 13; s ^= s >> 7; s ^= s << 17;
                let r = (s >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0;
                let burst = if (k / 200) % 2 == 0 { 1.0 } else { 0.05 };
                (amp * burst * r) as f32
            }).collect()).collect();
            let y = render(&p, &chans);
            let c = db(thr + CEILING_TRIM_DB) * (1.0 + 1e-6);
            for v in y.iter().flatten() {
                prop_assert!(v.is_finite() && f64::from(v.abs()) <= c, "{v} > {c}");
            }
        }
    }
}
