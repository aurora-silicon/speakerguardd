// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The linked four-band dynamics box used ahead of speaker protection.
//!
//! The parameter compiler supplies the Audio Unit's saved controls.  This
//! module contains no machine tuning.  Detection is linked by taking the
//! largest per-channel band power, while filtering and saturation remain
//! per channel.  Construction owns all storage; [`Run::process`] allocates
//! nothing and accepts arbitrary frame counts.

use crate::MAX_CHANNELS;

pub const BANDS: usize = 4;
const XOVERS: usize = BANDS - 1;
const MAX_FILTERS: usize = 32;
const INPUT_BOUND: f32 = 16.0;

/// Processing selected for one analysis band.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u32)]
pub enum BandMode {
    #[default]
    Active = 0,
    Pass = 1,
    Mute = 2,
}

/// The three piecewise level branches and gain staging of one band.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Band {
    pub mode: BandMode,
    pub down_threshold_db: f64,
    pub down_ratio: f64,
    /// Most downward gain change, in dB (zero or negative).
    pub down_range_db: f64,
    pub up_threshold_db: f64,
    pub up_ratio: f64,
    /// Most upward gain change, in dB (zero or positive).
    pub up_range_db: f64,
    pub exp_threshold_db: f64,
    pub exp_ratio: f64,
    pub makeup_db: f64,
    pub input_gain_db: f64,
    /// `None` disables the band waveshaper; otherwise the saved flavor.
    pub saturation: Option<u32>,
}

impl Default for Band {
    fn default() -> Self {
        Self {
            mode: BandMode::Active,
            down_threshold_db: -10.0,
            down_ratio: 1.0,
            down_range_db: -10.0,
            up_threshold_db: -40.0,
            up_ratio: 1.0,
            up_range_db: 10.0,
            exp_threshold_db: -70.0,
            exp_ratio: 1.0,
            makeup_db: 0.0,
            input_gain_db: 0.0,
            saturation: None,
        }
    }
}

/// A saved compressor state, independent of any graph or machine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    pub crossover_hz: [f64; XOVERS],
    /// The unit accepts continuous order controls.  Exact fourth-order
    /// states use the measured phase-equalised Linkwitz-Riley path; other
    /// values remain continuous here and are not discarded by encoding.
    pub crossover_order: [f64; XOVERS],
    pub input_gain_db: f64,
    pub output_gain_db: f64,
    pub bandwidth_compensation: bool,
    pub release_s: f64,
    pub release_on: bool,
    pub render_quality: u32,
    pub overshoot_compensation: bool,
    pub algorithm: u32,
    pub automation_input: f64,
    pub automation_min: f64,
    pub automation_max: f64,
    pub band: [Band; BANDS],
}

impl Default for Params {
    fn default() -> Self {
        Self {
            crossover_hz: [100.0, 1000.0, 10_000.0],
            crossover_order: [4.0, 6.0, 4.0],
            input_gain_db: 0.0,
            output_gain_db: 0.0,
            bandwidth_compensation: true,
            release_s: 0.05,
            release_on: false,
            render_quality: 0,
            overshoot_compensation: false,
            algorithm: 0,
            automation_input: 0.0,
            automation_min: -100.0,
            automation_max: 0.0,
            band: [Band::default(); BANDS],
        }
    }
}

impl Params {
    const HEAD_WORDS: usize = 17;
    const BAND_WORDS: usize = 13;
    pub const WORDS: usize = Self::HEAD_WORDS + BANDS * Self::BAND_WORDS;

    pub fn to_words(&self, out: &mut [f64]) {
        if out.len() < Self::WORDS {
            return;
        }
        out[..Self::WORDS].fill(0.0);
        out[0..3].copy_from_slice(&self.crossover_hz);
        out[3..6].copy_from_slice(&self.crossover_order);
        out[6] = self.input_gain_db;
        out[7] = self.output_gain_db;
        out[8] = self.bandwidth_compensation as u8 as f64;
        out[9] = self.release_s;
        out[10] = self.release_on as u8 as f64;
        out[11] = self.render_quality as f64;
        out[12] = self.overshoot_compensation as u8 as f64;
        out[13] = self.algorithm as f64;
        out[14] = self.automation_input;
        out[15] = self.automation_min;
        out[16] = self.automation_max;
        for (i, b) in self.band.iter().enumerate() {
            let o = Self::HEAD_WORDS + i * Self::BAND_WORDS;
            out[o..o + Self::BAND_WORDS].copy_from_slice(&[
                b.mode as u32 as f64,
                b.down_threshold_db,
                b.down_ratio,
                b.down_range_db,
                b.up_threshold_db,
                b.up_ratio,
                b.up_range_db,
                b.exp_threshold_db,
                b.exp_ratio,
                b.makeup_db,
                b.input_gain_db,
                b.saturation.is_some() as u8 as f64,
                b.saturation.unwrap_or(0) as f64,
            ]);
        }
    }

    pub fn from_words(w: &[f64]) -> Option<Params> {
        if w.len() != Self::WORDS || w.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let boolean = |v: f64| {
            if v == 0.0 {
                Some(false)
            } else if v == 1.0 {
                Some(true)
            } else {
                None
            }
        };
        let integer = |v: f64, hi: u32| {
            if v >= 0.0 && v <= hi as f64 && libm::floor(v) == v {
                Some(v as u32)
            } else {
                None
            }
        };
        let mut p = Params {
            crossover_hz: [w[0], w[1], w[2]],
            crossover_order: [w[3], w[4], w[5]],
            input_gain_db: w[6],
            output_gain_db: w[7],
            bandwidth_compensation: boolean(w[8])?,
            release_s: w[9],
            release_on: boolean(w[10])?,
            render_quality: integer(w[11], 2)?,
            overshoot_compensation: boolean(w[12])?,
            algorithm: integer(w[13], 3)?,
            automation_input: w[14],
            automation_min: w[15],
            automation_max: w[16],
            ..Default::default()
        };
        for (i, b) in p.band.iter_mut().enumerate() {
            let o = Self::HEAD_WORDS + i * Self::BAND_WORDS;
            let mode = match integer(w[o], 2)? {
                0 => BandMode::Active,
                1 => BandMode::Pass,
                _ => BandMode::Mute,
            };
            let sat = boolean(w[o + 11])?;
            let flavor = integer(w[o + 12], 4)?;
            *b = Band {
                mode,
                down_threshold_db: w[o + 1],
                down_ratio: w[o + 2],
                down_range_db: w[o + 3],
                up_threshold_db: w[o + 4],
                up_ratio: w[o + 5],
                up_range_db: w[o + 6],
                exp_threshold_db: w[o + 7],
                exp_ratio: w[o + 8],
                makeup_db: w[o + 9],
                input_gain_db: w[o + 10],
                saturation: sat.then_some(flavor),
            };
        }
        p.valid(48_000.0).then_some(p)
    }

    /// Every control in range for sample rate `fs`.
    pub fn valid(&self, fs: f64) -> bool {
        let finite = |v: f64, lo: f64, hi: f64| v.is_finite() && v >= lo && v <= hi;
        self.crossover_hz[0] > 0.0
            && self.crossover_hz[0] < self.crossover_hz[1]
            && self.crossover_hz[1] < self.crossover_hz[2]
            && self.crossover_hz[2] < 0.49 * fs
            && self.crossover_order.iter().all(|v| finite(*v, 2.0, 20.0))
            && finite(self.input_gain_db, -40.0, 40.0)
            && finite(self.output_gain_db, -40.0, 40.0)
            && finite(self.release_s, 0.0, 1.0)
            && finite(self.automation_input, -100.0, 100.0)
            && finite(self.automation_min, -1000.0, 1000.0)
            && finite(self.automation_max, -1000.0, 1000.0)
            && self.automation_min <= self.automation_max
            && self.band.iter().all(|b| {
                finite(b.down_threshold_db, -120.0, 0.0)
                    && finite(b.down_ratio, 1.0, 20.0)
                    && finite(b.down_range_db, -60.0, 0.0)
                    && finite(b.up_threshold_db, -120.0, 0.0)
                    && finite(b.up_ratio, 1.0, 20.0)
                    && finite(b.up_range_db, 0.0, 60.0)
                    && finite(b.exp_threshold_db, -120.0, 0.0)
                    && finite(b.exp_ratio, 1.0, 20.0)
                    && finite(b.makeup_db, -40.0, 40.0)
                    && finite(b.input_gain_db, -40.0, 40.0)
                    && b.saturation.is_none_or(|v| v <= 4)
            })
    }
}

#[derive(Clone, Copy)]
struct Section {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
}

impl Section {
    const ID: Self = Self { b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0 };
}

#[derive(Clone, Copy)]
struct Path {
    section: [Section; MAX_FILTERS],
    count: usize,
}

impl Path {
    const EMPTY: Self = Self { section: [Section::ID; MAX_FILTERS], count: 0 };

    fn push(&mut self, s: Section) {
        if let Some(slot) = self.section.get_mut(self.count) {
            *slot = s;
            self.count += 1;
        }
    }
}

/// The Linkwitz-Riley order the unit runs for a saved order control: the
/// control truncated to an integer, then rounded up to the next even order
/// (2, 4, 6, ... 20). Measured: 3 and 4.5 render as 4, 5 as 6, 7 as 8.
pub fn effective_order(control: f64) -> usize {
    let n = libm::floor(control).clamp(2.0, 20.0) as usize;
    n + (n & 1)
}

/// Butterworth sections of order `n` (bilinear, prewarped to `fc`).
fn push_butter(path: &mut Path, high: bool, n: usize, fc: f64, fs: f64) {
    let k = libm::tan(core::f64::consts::PI * fc / fs);
    for i in 0..n / 2 {
        let q2 = 2.0 * libm::sin(core::f64::consts::PI * (2 * i + 1) as f64 / (2 * n) as f64);
        let d = 1.0 + q2 * k + k * k;
        let (a1, a2) = (2.0 * (k * k - 1.0) / d, (1.0 - q2 * k + k * k) / d);
        let (b0, b1, b2) = if high { (1.0 / d, -2.0 / d, 1.0 / d) } else { (k * k / d, 2.0 * k * k / d, k * k / d) };
        path.push(Section { b0, b1, b2, a1, a2 });
    }
    if n % 2 != 0 {
        let d = 1.0 + k;
        let (b0, b1) = if high { (1.0 / d, -1.0 / d) } else { (k / d, k / d) };
        path.push(Section { b0, b1, b2: 0.0, a1: (k - 1.0) / d, a2: 0.0 });
    }
}

/// One side of a Linkwitz-Riley crossover of even order `n`: the Butterworth
/// of order `n / 2` twice. When `n / 2` is odd the high side is inverted so
/// that low plus high is an allpass.
fn push_lr(path: &mut Path, high: bool, n: usize, fc: f64, fs: f64) {
    push_butter(path, high, n / 2, fc, fs);
    push_butter(path, high, n / 2, fc, fs);
    if high && (n / 2) % 2 == 1 {
        path.push(Section { b0: -1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0 });
    }
}

/// The allpass a Linkwitz-Riley pair of even order `n` sums to (low plus
/// high, with the inversion above).
fn push_lr_allpass(path: &mut Path, n: usize, fc: f64, fs: f64) {
    // A Butterworth of order m has denominator D(s); the LR sum is
    // D(-s)/D(s) up to sign. Built per Butterworth section as its own allpass.
    let m = n / 2;
    let k = libm::tan(core::f64::consts::PI * fc / fs);
    for i in 0..m / 2 {
        let q2 = 2.0 * libm::sin(core::f64::consts::PI * (2 * i + 1) as f64 / (2 * m) as f64);
        let d = 1.0 + q2 * k + k * k;
        let (a1, a2) = (2.0 * (k * k - 1.0) / d, (1.0 - q2 * k + k * k) / d);
        path.push(Section { b0: a2, b1: a1, b2: 1.0, a1, a2 });
    }
    if m % 2 != 0 {
        let d = 1.0 + k;
        let a1 = (k - 1.0) / d;
        // first order: (1 - s) / (1 + s) = (a1 + z^-1) / (1 + a1 z^-1)
        path.push(Section { b0: a1, b1: 1.0, b2: 0.0, a1, a2: 0.0 });
    }
}

/// The unit's band tree, measured: the program splits at the middle
/// crossover first; the low side splits at the lowest crossover and passes
/// the highest crossover's allpass, the high side splits at the highest and
/// passes the lowest crossover's allpass, so the four bands sum to one
/// allpass.
fn paths(p: &Params, fs: f64) -> [Path; BANDS] {
    let mut v = [Path::EMPTY; BANDS];
    let n = p.crossover_order.map(effective_order);
    let f = p.crossover_hz;
    push_lr(&mut v[0], false, n[1], f[1], fs);
    push_lr(&mut v[0], false, n[0], f[0], fs);
    push_lr_allpass(&mut v[0], n[2], f[2], fs);
    push_lr(&mut v[1], false, n[1], f[1], fs);
    push_lr(&mut v[1], true, n[0], f[0], fs);
    push_lr_allpass(&mut v[1], n[2], f[2], fs);
    push_lr(&mut v[2], true, n[1], f[1], fs);
    push_lr(&mut v[2], false, n[2], f[2], fs);
    push_lr_allpass(&mut v[2], n[0], f[0], fs);
    push_lr(&mut v[3], true, n[1], f[1], fs);
    push_lr(&mut v[3], true, n[2], f[2], fs);
    push_lr_allpass(&mut v[3], n[0], f[0], fs);
    v
}

/// The real-time state of one linked compressor instance.
pub struct Run {
    path: [Path; BANDS],
    band: [Band; BANDS],
    in_gain: f64,
    out_gain: f64,
    band_in: [f64; BANDS],
    release: f64,
    release_on: bool,
    channels: usize,
    env: [f64; BANDS],
    state: [[[[f64; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize) -> Run {
        let fs = if fs.is_finite() && fs > 1.0 { fs } else { 48_000.0 };
        let valid = p.valid(fs);
        let q = if valid { p } else { &Params::default() };
        Run {
            path: paths(q, fs),
            band: q.band,
            in_gain: libm::pow(10.0, q.input_gain_db / 20.0),
            out_gain: libm::pow(10.0, q.output_gain_db / 20.0),
            band_in: q.band.map(|b| libm::pow(10.0, b.input_gain_db / 20.0)),
            release: if q.release_s > 0.0 { libm::exp(-1.0 / (q.release_s * fs)) } else { 0.0 },
            release_on: q.release_on,
            channels: channels.min(MAX_CHANNELS),
            env: [0.0; BANDS],
            state: [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
        }
    }

    pub fn reset(&mut self) {
        self.env = [0.0; BANDS];
        self.state = [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS];
    }

    pub fn latency(&self) -> usize {
        0
    }

    #[inline]
    fn split(&mut self, c: usize, x: f64) -> [f64; BANDS] {
        let mut out = [0.0; BANDS];
        for (b, y) in out.iter_mut().enumerate() {
            let mut v = x;
            for i in 0..self.path[b].count {
                let s = self.path[b].section[i];
                let z = &mut self.state[c][b][i];
                let o = s.b0 * v + z[0];
                z[0] = s.b1 * v - s.a1 * o + z[1];
                z[1] = s.b2 * v - s.a2 * o;
                v = o;
            }
            *y = v * self.band_in[b];
        }
        out
    }

    fn gain(&self, b: usize) -> f64 {
        let p = self.band[b];
        if p.mode == BandMode::Mute {
            return 0.0;
        }
        if p.mode == BandMode::Pass {
            return 1.0;
        }
        let level = 10.0 * libm::log10(self.env[b].max(1e-30));
        let change = if level < p.exp_threshold_db && p.exp_ratio > 1.0 {
            ((p.exp_ratio - 1.0) * (level - p.exp_threshold_db)).max(-60.0)
        } else if level < p.up_threshold_db && p.up_ratio > 1.0 {
            ((1.0 - 1.0 / p.up_ratio) * (p.up_threshold_db - level)).min(p.up_range_db)
        } else if level > p.down_threshold_db && p.down_ratio > 1.0 {
            ((1.0 / p.down_ratio - 1.0) * (level - p.down_threshold_db)).max(p.down_range_db)
        } else {
            0.0
        };
        libm::pow(10.0, (p.makeup_db + change) / 20.0)
    }

    #[inline]
    fn saturate(flavor: Option<u32>, x: f64) -> f64 {
        let Some(f) = flavor else { return x };
        if f == 1 {
            // The enabled speaker state uses the smooth, odd flavor: a small
            // linear lift followed by a cubic-like soft bend.  The rational
            // form remains finite for every bounded input and reproduces its
            // low-level slope and third-harmonic growth.
            return 1.011_73 * x / (1.0 + 0.154 * x * x);
        }
        // Other public flavors are retained on the wire but use a
        // conservative magnitude-reducing curve until a compiler admits one.
        let strength = 0.125 * (f + 1) as f64;
        x / (1.0 + strength * x.abs())
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let n = bufs.len().min(self.channels).min(MAX_CHANNELS);
        let mut bands = [[0.0; BANDS]; MAX_CHANNELS];
        for i in 0..frames {
            let mut power = [0.0f64; BANDS];
            for c in 0..n {
                let x = bufs[c].get(i).copied().unwrap_or(0.0);
                let x = if x.is_finite() { x.clamp(-INPUT_BOUND, INPUT_BOUND) as f64 } else { 0.0 };
                bands[c] = self.split(c, self.in_gain * x);
                for b in 0..BANDS {
                    power[b] = power[b].max(bands[c][b] * bands[c][b]);
                }
            }
            for (e, p) in self.env.iter_mut().zip(power) {
                *e = if self.release_on { p.max(*e * self.release) } else { p };
            }
            let gain = [self.gain(0), self.gain(1), self.gain(2), self.gain(3)];
            for c in 0..n {
                let mut y = 0.0;
                for b in 0..BANDS {
                    y += Self::saturate(self.band[b].saturation, gain[b] * bands[c][b]);
                }
                y *= self.out_gain;
                if let Some(out) = bufs[c].get_mut(i) {
                    *out = if y.is_finite() { y as f32 } else { 0.0 };
                }
                if !y.is_finite() {
                    self.state[c] = [[[0.0; 2]; MAX_FILTERS]; BANDS];
                }
            }
            if self.env.iter().any(|v| !v.is_finite()) {
                self.env = [0.0; BANDS];
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> Params {
        Params {
            crossover_hz: [200.0, 1500.0, 8000.0],
            crossover_order: [4.0; 3],
            input_gain_db: -12.0,
            output_gain_db: 14.0,
            release_s: 0.05,
            release_on: true,
            band: [Band {
                down_threshold_db: -30.0,
                down_ratio: 4.0,
                down_range_db: -20.0,
                up_threshold_db: -50.0,
                up_ratio: 1.0,
                exp_threshold_db: -90.0,
                exp_ratio: 1.0,
                ..Band::default()
            }; BANDS],
            ..Params::default()
        }
    }

    fn tone(f: f64, a: f64, n: usize) -> std::vec::Vec<f32> {
        (0..n).map(|i| (a * libm::sin(2.0 * core::f64::consts::PI * f * i as f64 / 48_000.0)) as f32).collect()
    }

    fn level(x: &[f32], f: f64) -> f64 {
        let start = x.len() / 2;
        let (mut re, mut im) = (0.0, 0.0);
        for (i, v) in x[start..].iter().enumerate() {
            let w = 2.0 * core::f64::consts::PI * f * (i + start) as f64 / 48_000.0;
            re += *v as f64 * libm::cos(w);
            im += *v as f64 * libm::sin(w);
        }
        20.0 * libm::log10(2.0 * libm::sqrt(re * re + im * im) / (x.len() - start) as f64)
    }

    #[test]
    fn words_round_trip_and_validate() {
        let mut p = params();
        p.automation_min = -100.0;
        p.automation_max = 0.0;
        p.band[2].saturation = Some(1);
        let mut w = [0.0; Params::WORDS];
        p.to_words(&mut w);
        assert_eq!(Params::from_words(&w), Some(p));
        w[3] = 21.0;
        assert!(Params::from_words(&w).is_none());
    }

    #[test]
    fn fourth_order_bank_reconstructs_and_compresses() {
        let mut quiet = tone(997.0, 1e-4, 48_000);
        let input = level(&quiet, 997.0);
        let mut r = Run::new(&params(), 48_000.0, 1);
        r.process(&mut [&mut quiet], 48_000);
        assert!((level(&quiet, 997.0) - input - 2.0).abs() < 0.08);

        let mut loud = tone(80.0, 1.0, 48_000);
        let mut r = Run::new(&params(), 48_000.0, 1);
        r.process(&mut [&mut loud], 48_000);
        assert!(level(&loud, 80.0) < 1.0);
    }

    #[test]
    fn order_controls_round_up_to_even_linkwitz_riley_orders() {
        for (control, order) in [(2.0, 2), (3.0, 4), (4.0, 4), (4.5, 4), (5.0, 6), (6.0, 6), (7.0, 8), (20.0, 20)] {
            assert_eq!(effective_order(control), order, "control {control}");
        }
    }

    /// Response of one band path at `f`, as a magnitude.
    fn band_magnitude(path: &Path, f: f64) -> f64 {
        let w = 2.0 * core::f64::consts::PI * f / 48_000.0;
        let (c1, s1, c2, s2) = (libm::cos(w), libm::sin(w), libm::cos(2.0 * w), libm::sin(2.0 * w));
        let mut m = 1.0;
        for s in &path.section[..path.count] {
            let (nr, ni) = (s.b0 + s.b1 * c1 + s.b2 * c2, -s.b1 * s1 - s.b2 * s2);
            let (dr, di) = (1.0 + s.a1 * c1 + s.a2 * c2, -s.a1 * s1 - s.a2 * s2);
            m *= libm::sqrt((nr * nr + ni * ni) / (dr * dr + di * di));
        }
        m
    }

    /// Band levels of the unit measured offline on synthetic controls
    /// (crossovers 300, 1500, 6000 Hz): the middle-first tree, not a
    /// sequential one (that differs by 1.6 dB at band 0, 1 kHz, order 4).
    #[test]
    fn band_tree_matches_the_units_measured_levels() {
        let cases: [(f64, usize, f64, f64); 8] = [
            (4.0, 0, 1000.0, -43.5),
            (4.0, 1, 100.0, -38.2),
            (4.0, 2, 1000.0, -15.7),
            (4.0, 3, 3000.0, -26.4),
            (6.0, 0, 600.0, -36.3),
            (6.0, 2, 3000.0, -0.2),
            (2.0, 1, 1000.0, -3.9),
            (2.0, 3, 1500.0, -31.5),
        ];
        for (order, band, f, db) in cases {
            let mut p = params();
            p.crossover_hz = [300.0, 1500.0, 6000.0];
            p.crossover_order = [order; 3];
            let v = paths(&p, 48_000.0);
            let got = 20.0 * libm::log10(band_magnitude(&v[band], f));
            assert!((got - db).abs() < 0.1, "order {order} band {band} at {f} Hz: {got} dB, unit {db} dB");
        }
    }

    #[test]
    fn bands_sum_to_an_allpass_and_cross_at_minus_6_db() {
        for order in [2.0, 3.0, 4.0, 6.0, 8.0] {
            let mut p = params();
            p.crossover_order = [order; 3];
            p.input_gain_db = 0.0;
            p.output_gain_db = 0.0;
            p.release_on = false;
            for b in p.band.iter_mut() {
                b.mode = BandMode::Pass;
                b.saturation = None;
            }
            let v = paths(&p, 48_000.0);
            for (i, fc) in p.crossover_hz.iter().enumerate() {
                // the two bands meeting at crossover i are each -6 dB there
                let (lo, hi) = (band_magnitude(&v[i], *fc), band_magnitude(&v[i + 1], *fc));
                assert!((20.0 * libm::log10(lo) + 6.02).abs() < 0.3, "order {order} xo {i} low {lo}");
                assert!((20.0 * libm::log10(hi) + 6.02).abs() < 0.3, "order {order} xo {i} high {hi}");
            }
            for f in [40.0, 300.0, 1000.0, 3000.0, 12_000.0] {
                let mut x = tone(f, 0.25, 24_000);
                let mut run = Run::new(&p, 48_000.0, 1);
                run.process(&mut [&mut x], 24_000);
                assert!((level(&x, f) - 20.0 * libm::log10(0.25)).abs() < 0.01, "order {order} at {f} Hz");
            }
        }
    }

    #[test]
    fn upward_expander_link_and_saturation_are_bounded() {
        let mut p = params();
        p.input_gain_db = 0.0;
        p.output_gain_db = 0.0;
        p.band[0].down_ratio = 1.0;
        p.band[0].up_threshold_db = -30.0;
        p.band[0].up_ratio = 2.0;
        p.band[0].up_range_db = 10.0;
        let mut left = tone(80.0, 0.5, 24_000);
        let mut right = tone(80.0, 1e-3, 24_000);
        let before = level(&right, 80.0);
        let mut r = Run::new(&p, 48_000.0, 2);
        r.process(&mut [&mut left, &mut right], 24_000);
        // The loud channel's detector prevents an upward boost on either.
        assert!((level(&right, 80.0) - before).abs() < 0.1);
        assert!(Run::saturate(Some(4), 1.0).abs() <= 1.0);
    }

    #[test]
    fn non_fourth_orders_and_garbage_stay_finite() {
        let mut p = params();
        p.crossover_order = [3.0, 4.5, 8.0];
        let mut x = [f32::NAN, f32::INFINITY, -1e30, 1e30, 0.5];
        let mut r = Run::new(&p, 48_000.0, 1);
        r.process(&mut [&mut x], 5);
        assert!(x.iter().all(|v| v.is_finite()));
        assert_eq!(r.latency(), 0);
        r.reset();
    }
}
