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
//! With the optional high-pass (algorithm version 1, location 0) every
//! channel is filtered after the input gain, ahead of the detector and the
//! look-ahead delay: an order-`n` Butterworth high-pass, see
//! [`highpass_sections`].
//!
//! The hold makes the window cover every delayed sample under it, so the
//! output's sample peak never exceeds `C`.
//!
//! State is f64; the unit keeps its slow gain in f32, which moves it by
//! less than 0.005 dB over a 10 s release.

use crate::MAX_CHANNELS;
use alloc::boxed::Box;
use alloc::vec;

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
/// Mode-3 output-saturation reconstruction: a ninth-order Butterworth low
/// pass followed by the unit's 24-frame reported-latency alignment.
pub const SATURATION_FILTER_HZ: f64 = 14_100.0;
pub const SATURATION_FILTER_ORDER: usize = 5;
pub const SATURATION_DELAY: usize = 24;

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
/// Highest high-pass order the unit offers.
pub const MAX_HIGHPASS_ORDER: usize = 10;
const HIGHPASS_SECTIONS: usize = MAX_HIGHPASS_ORDER.div_ceil(2);

/// The high-pass of `order` at `f0`: the bilinear transform, prewarped at
/// the corner, of the analog Butterworth prototype. Pole pairs become
/// second-order sections (`Q = 1 / (2 sin((2k + 1) pi / 2n))`), an odd
/// order's real pole one first-order section. The unit designs them in
/// single precision; at a corner of tens of hertz that rounding moves the
/// poles enough to shift the filter's decaying tail by tenths of a dB, so
/// the design is single precision here too (the filtering is not).
fn highpass_sections(order: usize, f0: f64, fs: f64) -> ([SatSection; HIGHPASS_SECTIONS], usize) {
    let mut s = [SatSection::default(); HIGHPASS_SECTIONS];
    let w = 2.0 * core::f32::consts::PI * f0 as f32 / fs as f32;
    let (sn, cs) = (libm::sinf(w), libm::cosf(w));
    let pairs = order / 2;
    for (k, sec) in s.iter_mut().enumerate().take(pairs) {
        let q = 1.0 / (2.0 * libm::sinf((2 * k + 1) as f32 * core::f32::consts::PI / (2 * order) as f32));
        let alpha = sn / (2.0 * q);
        let a0 = 1.0 + alpha;
        let b = (1.0 + cs) / (2.0 * a0);
        *sec = SatSection {
            b: [f64::from(b), f64::from(-2.0 * b), f64::from(b)],
            a: [f64::from(-2.0 * cs / a0), f64::from((1.0 - alpha) / a0)],
        };
    }
    let mut n = pairs;
    if order % 2 == 1 {
        let k = libm::tanf(w / 2.0);
        let a0 = 1.0 + k;
        s[n] = SatSection { b: [f64::from(1.0 / a0), f64::from(-1.0 / a0), 0.0], a: [f64::from((k - 1.0) / a0), 0.0] };
        n += 1;
    }
    (s, n)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    pub lookahead_s: f64,
    /// The unit's "fast gain release time": a hold, in seconds.
    pub hold_s: f64,
    pub threshold_db: f64,
    pub min_fast_gain_db: f64,
    pub true_peak: bool,
    /// Output saturation oversampling mode (zero disables it; mode 3 is
    /// carried with flavor 2, the sine shaper).
    pub saturation_mode: u32,
    pub saturation_flavor: u32,
    pub saturation_mix: f64,
    pub input_gain_db: f64,
    pub output_gain_db: f64,
    /// Butterworth high-pass order (0: off), corner and location (0: at
    /// the input, ahead of the detector and the look-ahead delay).
    pub highpass_order: u32,
    pub highpass_hz: f64,
    pub highpass_location: u32,
}

impl Params {
    pub const WORDS: usize = 10;
    pub const LEGACY_WORDS: usize = 8;
    /// With the high-pass: `WORDS` plus order, corner and location.
    pub const HIGHPASS_WORDS: usize = 13;

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
        ]);
        if out.len() >= Self::HIGHPASS_WORDS {
            out[Self::WORDS..Self::HIGHPASS_WORDS].copy_from_slice(&[
                self.highpass_order as f64,
                self.highpass_hz,
                self.highpass_location as f64,
            ]);
        }
    }

    pub fn from_words(w: &[f64]) -> Option<Params> {
        if ![Self::LEGACY_WORDS, Self::WORDS, Self::HIGHPASS_WORDS].contains(&w.len())
            || w.iter().any(|v| !v.is_finite())
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
                0.0 => 0,
                3.0 => 3,
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
            highpass_order: w.get(10).copied().unwrap_or(0.0) as u32,
            highpass_hz: w.get(11).copied().unwrap_or(0.0),
            highpass_location: w.get(12).copied().unwrap_or(0.0) as u32,
        };
        if w.len() == Self::HIGHPASS_WORDS {
            let order = w[10];
            let whole = libm::trunc(order) == order && (1.0..=MAX_HIGHPASS_ORDER as f64).contains(&order);
            if !(whole && p.highpass_hz > 0.0 && p.highpass_hz < 20_000.0) || w[12] != 0.0 {
                return None;
            }
        }
        let ok = (-120.0..=40.0).contains(&p.input_gain_db)
            && (-120.0..=40.0).contains(&p.output_gain_db)
            && (0.0..=MAX_LOOKAHEAD_S).contains(&p.lookahead_s)
            && (0.0..=MAX_HOLD_S).contains(&p.hold_s)
            && (-40.0..=6.0).contains(&p.threshold_db)
            && (0.0..=40.0).contains(&p.min_fast_gain_db);
        let saturation_ok = if p.saturation_mode == 0 {
            p.saturation_flavor == 0 && p.saturation_mix == 0.0
        } else {
            p.saturation_flavor == 2 && (0.0..=1.0).contains(&p.saturation_mix)
        };
        (ok && saturation_ok).then_some(p)
    }
}

#[derive(Clone, Copy, Default)]
struct SatSection {
    b: [f64; 3],
    a: [f64; 2],
}

#[derive(Clone, Copy, Default)]
struct SatState {
    z: [f64; 2],
}

impl SatSection {
    fn run(&self, state: &mut SatState, x: f64) -> f64 {
        let y = self.b[0] * x + state.z[0];
        state.z[0] = self.b[1] * x - self.a[0] * y + state.z[1];
        state.z[1] = self.b[2] * x - self.a[1] * y;
        y
    }
}

const SATURATION_SECTIONS: usize = SATURATION_FILTER_ORDER.div_ceil(2);

fn saturation_filter(fs: f64) -> [SatSection; SATURATION_SECTIONS] {
    let mut sections = [SatSection::default(); SATURATION_SECTIONS];
    let k = libm::tan(core::f64::consts::PI * SATURATION_FILTER_HZ / fs);
    for (i, section) in sections.iter_mut().take(SATURATION_FILTER_ORDER / 2).enumerate() {
        let theta = core::f64::consts::PI * (2 * i + 1) as f64 / (2 * SATURATION_FILTER_ORDER) as f64;
        let two_zeta = 2.0 * libm::sin(theta);
        let a0 = 1.0 + two_zeta * k + k * k;
        let gain = k * k / a0;
        *section = SatSection {
            b: [gain, 2.0 * gain, gain],
            a: [2.0 * (k * k - 1.0) / a0, (1.0 - two_zeta * k + k * k) / a0],
        };
    }
    if SATURATION_FILTER_ORDER % 2 == 1 {
        let a0 = 1.0 + k;
        sections[SATURATION_SECTIONS - 1] = SatSection { b: [k / a0, k / a0, 0.0], a: [(k - 1.0) / a0, 0.0] };
    }
    sections
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
    saturation_mix: f64,
    input_gain: f64,
    output_gain: f64,
    saturation_filter: [SatSection; SATURATION_SECTIONS],
    saturation_state: [[SatState; SATURATION_SECTIONS]; MAX_CHANNELS],
    saturation_line: [[f32; SATURATION_DELAY]; MAX_CHANNELS],
    saturation_at: usize,
    highpass: [SatSection; HIGHPASS_SECTIONS],
    highpass_sections: usize,
    highpass_state: [[SatState; HIGHPASS_SECTIONS]; MAX_CHANNELS],
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
        let (highpass, highpass_sections) = if p.highpass_order > 0 {
            highpass_sections(p.highpass_order as usize, p.highpass_hz, fs)
        } else {
            ([SatSection::default(); HIGHPASS_SECTIONS], 0)
        };
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
            saturation_mix: if p.saturation_mode == 3 { p.saturation_mix } else { 0.0 },
            saturation_filter: saturation_filter(fs),
            saturation_state: [[SatState::default(); SATURATION_SECTIONS]; MAX_CHANNELS],
            saturation_line: [[0.0; SATURATION_DELAY]; MAX_CHANNELS],
            saturation_at: 0,
            highpass,
            highpass_sections,
            highpass_state: [[SatState::default(); HIGHPASS_SECTIONS]; MAX_CHANNELS],
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
        self.saturation_state = [[SatState::default(); SATURATION_SECTIONS]; MAX_CHANNELS];
        self.saturation_line = [[0.0; SATURATION_DELAY]; MAX_CHANNELS];
        self.saturation_at = 0;
        self.highpass_state = [[SatState::default(); HIGHPASS_SECTIONS]; MAX_CHANNELS];
    }

    pub fn latency(&self) -> usize {
        self.lookahead + if self.saturation_mix > 0.0 { SATURATION_DELAY } else { 0 }
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
            for (c, b) in bufs[..ch].iter().enumerate() {
                let mut x = if b[k].is_finite() { f64::from(b[k]) } else { 0.0 };
                x *= self.input_gain;
                for section in 0..self.highpass_sections {
                    x = self.highpass[section].run(&mut self.highpass_state[c][section], x);
                }
                if self.saturation_mix > 0.0 {
                    x = (1.0 - self.saturation_mix) * x + self.saturation_mix * libm::sin(x);
                    for section in 0..SATURATION_SECTIONS {
                        x = self.saturation_filter[section].run(&mut self.saturation_state[c][section], x);
                    }
                    let held = self.saturation_line[c][self.saturation_at];
                    self.saturation_line[c][self.saturation_at] = x as f32;
                    x = f64::from(held);
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
            if self.saturation_mix > 0.0 {
                self.saturation_at = (self.saturation_at + 1) % SATURATION_DELAY;
            }
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
            input_gain_db: 0.0,
            output_gain_db: 0.0,
            highpass_order: 0,
            highpass_hz: 0.0,
            highpass_location: 0,
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
    fn highpass_is_a_butterworth_at_the_input() {
        let mut p = params(0.0, false);
        p.highpass_order = 3;
        p.highpass_hz = 40.0;
        let mut w = [0.0; Params::HIGHPASS_WORDS];
        p.to_words(&mut w);
        assert_eq!(Params::from_words(&w), Some(p));
        let mut bad = w;
        bad[12] = 1.0; // output location is not carried
        assert_eq!(Params::from_words(&bad), None);
        bad = w;
        bad[10] = 11.0;
        assert_eq!(Params::from_words(&bad), None);
        // below the threshold: the corner is 3 dB down, the pass band flat,
        // an octave under the corner 18 dB down (third order)
        for (f, want) in [(40.0, -3.0103), (1000.0, 0.0), (20.0, -18.1)] {
            let n = 96_000;
            let x: Vec<f32> = (0..n)
                .map(|k| 0.01 * libm::sin(2.0 * core::f64::consts::PI * f * k as f64 / 48_000.0) as f32)
                .collect();
            let y = render(&p, core::slice::from_ref(&x));
            let rms = |v: &[f32]| libm::sqrt(v.iter().map(|s| f64::from(*s).powi(2)).sum::<f64>() / v.len() as f64);
            let g = 20.0 * libm::log10(rms(&y[0][n / 2..]) / rms(&x[n / 2..]));
            assert!((g - want).abs() < 0.05, "{f} Hz: {g}");
        }
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
            input_gain_db: 0.0,
            output_gain_db: 0.0,
            highpass_order: 0,
            highpass_hz: 0.0,
            highpass_location: 0,
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
