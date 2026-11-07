// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The linked four-band dynamics box used ahead of speaker protection.
//!
//! The parameter compiler supplies the Audio Unit's saved controls.  This
//! module contains no machine tuning.  Detection uses the Butterworth half
//! of each Linkwitz-Riley band and is linked by taking the largest channel
//! magnitude.  Algorithms zero through two measure a phase-invariant complex
//! envelope; algorithm three retains the real-waveform estimator.  Overshoot
//! compensation sends each band-local detector signal through the same
//! four-band analysis bank once more and max-links that separate magnitude
//! arm into the ordinary detector state.  Audio filtering and saturation
//! remain per channel.
//! Construction owns all storage; [`Run::process`] allocates
//! nothing and accepts arbitrary frame counts.

use crate::MAX_CHANNELS;

pub const BANDS: usize = 4;
const XOVERS: usize = BANDS - 1;
pub const MAX_AUTOMATION_POINTS: usize = 16;
const MAX_FILTERS: usize = 32;
const MAX_DETECTOR_FILTERS: usize = 10;
const MAX_DETECTOR_WINDOW: usize = 1921;
const QUADRATURE_STAGES: usize = 4;
const MAX_GAIN_SRC_TAPS: usize = 225;
const INPUT_BOUND: f32 = 16.0;
const CURVE_POINTS: usize = 11;

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

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AutomationPoint {
    pub input: f64,
    pub value: f64,
}

/// A piecewise-linear automation curve for one band's downward threshold.
/// An empty curve leaves the saved threshold unchanged.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThresholdAutomation {
    pub points: [AutomationPoint; MAX_AUTOMATION_POINTS],
    pub count: usize,
}

impl Default for ThresholdAutomation {
    fn default() -> Self {
        Self { points: [AutomationPoint::default(); MAX_AUTOMATION_POINTS], count: 0 }
    }
}

impl ThresholdAutomation {
    fn valid(&self, input_min: f64, input_max: f64) -> bool {
        self.count <= MAX_AUTOMATION_POINTS
            && (self.count == 0 || self.count >= 2)
            && self.points[..self.count].iter().enumerate().all(|(i, p)| {
                p.input.is_finite()
                    && p.value.is_finite()
                    && (input_min..=input_max).contains(&p.input)
                    && (-120.0..=0.0).contains(&p.value)
                    && (i == 0 || p.input > self.points[i - 1].input)
            })
    }

    fn value(&self, input: f64) -> Option<f64> {
        let points = &self.points[..self.count];
        let first = *points.first()?;
        if input <= first.input {
            return Some(first.value);
        }
        for pair in points.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if input <= b.input {
                let t = (input - a.input) / (b.input - a.input);
                return Some(a.value + t * (b.value - a.value));
            }
        }
        Some(points[points.len() - 1].value)
    }
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
    /// The unit accepts continuous order controls. Processing truncates each
    /// value and rounds odd orders up to the next even Linkwitz-Riley order;
    /// the original control is retained by encoding.
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
    pub threshold_automation: [ThresholdAutomation; BANDS],
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
            threshold_automation: [ThresholdAutomation::default(); BANDS],
            band: [Band::default(); BANDS],
        }
    }
}

impl Params {
    const HEAD_WORDS: usize = 17;
    const BAND_WORDS: usize = 13;
    const AUTOMATION_WORDS: usize = 1 + 2 * MAX_AUTOMATION_POINTS;
    pub const WORDS: usize = Self::HEAD_WORDS + BANDS * (Self::BAND_WORDS + Self::AUTOMATION_WORDS);

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
        let base = Self::HEAD_WORDS + BANDS * Self::BAND_WORDS;
        for (band, curve) in self.threshold_automation.iter().enumerate() {
            let o = base + band * Self::AUTOMATION_WORDS;
            out[o] = curve.count as f64;
            for (i, point) in curve.points.iter().enumerate() {
                out[o + 1 + 2 * i] = point.input;
                out[o + 2 + 2 * i] = point.value;
            }
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
        let base = Self::HEAD_WORDS + BANDS * Self::BAND_WORDS;
        for (band, curve) in p.threshold_automation.iter_mut().enumerate() {
            let o = base + band * Self::AUTOMATION_WORDS;
            let count = integer(w[o], MAX_AUTOMATION_POINTS as u32)? as usize;
            curve.count = count;
            for (i, point) in curve.points.iter_mut().enumerate() {
                point.input = w[o + 1 + 2 * i];
                point.value = w[o + 2 + 2 * i];
            }
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
            && self.threshold_automation.iter().all(|curve| curve.valid(self.automation_min, self.automation_max))
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

#[derive(Clone, Copy)]
struct DetectorPath {
    section: [Section; MAX_DETECTOR_FILTERS],
    count: usize,
}

#[derive(Clone, Copy)]
struct LegacyStage {
    high: Path,
    allpass: Path,
}

impl LegacyStage {
    const EMPTY: Self = Self { high: Path::EMPTY, allpass: Path::EMPTY };
}

#[derive(Clone, Copy)]
struct HalfStage {
    low: Path,
    high: Path,
    high_sign: f64,
}

impl HalfStage {
    const EMPTY: Self = Self { low: Path::EMPTY, high: Path::EMPTY, high_sign: 1.0 };
}

impl DetectorPath {
    const EMPTY: Self = Self { section: [Section::ID; MAX_DETECTOR_FILTERS], count: 0 };

    fn push(&mut self, s: Section) {
        if let Some(slot) = self.section.get_mut(self.count) {
            *slot = s;
            self.count += 1;
        }
    }
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

fn push_detector_butter(path: &mut DetectorPath, high: bool, n: usize, fc: f64, fs: f64) {
    let k = libm::tan(core::f64::consts::PI * fc / fs);
    for i in 0..n / 2 {
        let q2 = 2.0 * libm::sin(core::f64::consts::PI * (2 * i + 1) as f64 / (2 * n) as f64);
        let d = 1.0 + q2 * k + k * k;
        let (a1, a2) = (2.0 * (k * k - 1.0) / d, (1.0 - q2 * k + k * k) / d);
        let (b0, b1, b2) = if high { (1.0 / d, -2.0 / d, 1.0 / d) } else { (k * k / d, 2.0 * k * k / d, k * k / d) };
        path.push(Section { b0, b1, b2, a1, a2 });
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

/// Split/merge stages used by the quality-1 algorithms.  Each stage keeps the
/// Linkwitz-Riley high branch and obtains the low branch as `allpass - high`.
/// Recombination phase-corrects the running low-band sum before adding the
/// next band.  This is the split/merge realization described by Google's
/// Apache-2.0 `MultiCrossoverFilter`, specialized here to four bands and the
/// saved independent crossover orders.
fn legacy_stages(p: &Params, fs: f64) -> [LegacyStage; XOVERS] {
    let mut out = [LegacyStage::EMPTY; XOVERS];
    for (stage, item) in out.iter_mut().enumerate() {
        let n = effective_order(p.crossover_order[stage]);
        push_lr(&mut item.high, true, n, p.crossover_hz[stage], fs);
        push_lr_allpass(&mut item.allpass, n, p.crossover_hz[stage], fs);
    }
    out
}

/// Analysis and synthesis each run one Butterworth half. Applying a moving
/// gain between these halves is not equivalent to the collapsed LR path.
fn half_stages(p: &Params, fs: f64) -> [HalfStage; XOVERS] {
    let mut out = [HalfStage::EMPTY; XOVERS];
    for (stage, item) in out.iter_mut().enumerate() {
        let half_order = effective_order(p.crossover_order[stage]) / 2;
        push_butter(&mut item.low, false, half_order, p.crossover_hz[stage], fs);
        push_butter(&mut item.high, true, half_order, p.crossover_hz[stage], fs);
        item.high_sign = if half_order % 2 == 0 { 1.0 } else { -1.0 };
    }
    out
}

/// Detector paths use one Butterworth half of each Linkwitz-Riley branch.
/// Their magnitudes are consequently the square root of the audio bands'
/// magnitudes.  Phase-equalising allpasses are unnecessary before magnitude
/// detection.
fn detector_paths(p: &Params, fs: f64) -> [DetectorPath; BANDS] {
    let mut v = [DetectorPath::EMPTY; BANDS];
    let n = p.crossover_order.map(effective_order);
    let f = p.crossover_hz;
    push_detector_butter(&mut v[0], false, n[1] / 2, f[1], fs);
    push_detector_butter(&mut v[0], false, n[0] / 2, f[0], fs);
    push_detector_butter(&mut v[1], false, n[1] / 2, f[1], fs);
    push_detector_butter(&mut v[1], true, n[0] / 2, f[0], fs);
    push_detector_butter(&mut v[2], true, n[1] / 2, f[1], fs);
    push_detector_butter(&mut v[2], false, n[2] / 2, f[2], fs);
    push_detector_butter(&mut v[3], true, n[1] / 2, f[1], fs);
    push_detector_butter(&mut v[3], true, n[2] / 2, f[2], fs);
    v
}

/// Analytic (mode-4) detector of algorithms 0-2 at quality 0 and 1.
///
/// Black-box law (SW-COMP2, installed unit, synthetic controls): the band
/// level has the Butterworth-half magnitude of the steady law above, but
/// the timing of the full Linkwitz-Riley band the audio takes.  Both hold
/// when the level is the geometric mean of two analytic envelopes: the
/// band's own Linkwitz-Riley filter `|B|^2 e^{-j2 arg D}` and the
/// Linkwitz-Riley allpass of the same crossovers `e^{-j2 arg D}`, whose
/// product has magnitude `|B|^2` and whose square root is `|B|`.  The tree
/// follows the audio split of the quality: quality 0 uses the band tree of
/// `paths`, quality 1 the split chain of `legacy_stages` (no passing
/// allpass in either; it carries no level information).
fn analytic_detector_paths(p: &Params, fs: f64) -> ([Path; BANDS], [Path; BANDS]) {
    let mut band = [Path::EMPTY; BANDS];
    let mut reference = [Path::EMPTY; BANDS];
    let n = p.crossover_order.map(effective_order);
    let f = p.crossover_hz;
    // (crossover, high side) per band, in processing order.
    let tree: [&[(usize, bool)]; BANDS] =
        [&[(1, false), (0, false)], &[(1, false), (0, true)], &[(1, true), (2, false)], &[(1, true), (2, true)]];
    let chain: [&[(usize, bool)]; BANDS] = [
        &[(0, false)],
        &[(0, true), (1, false)],
        &[(0, true), (1, true), (2, false)],
        &[(0, true), (1, true), (2, true)],
    ];
    let legacy = p.render_quality == 1 && p.algorithm < 3;
    for b in 0..BANDS {
        for &(x, high) in if legacy { chain[b] } else { tree[b] } {
            push_lr(&mut band[b], high, n[x], f[x], fs);
            push_lr_allpass(&mut reference[b], n[x], f[x], fs);
        }
    }
    (band, reference)
}

#[inline]
fn db_amp(db: f64) -> f64 {
    libm::pow(10.0, db / 20.0)
}

fn curve_points(p: Band, top_db: f64) -> [f64; CURVE_POINTS] {
    let mut out = [1.0; CURVE_POINTS];
    let step = (100.0 + top_db) / (CURVE_POINTS - 1) as f64;
    let makeup = db_amp(p.makeup_db);
    for (i, gain) in out.iter_mut().enumerate() {
        let level = -100.0 + i as f64 * step;
        let change = if level < p.up_threshold_db {
            ((1.0 - 1.0 / p.up_ratio) * (p.up_threshold_db - level)).min(p.up_range_db)
        } else if level >= p.down_threshold_db {
            ((1.0 / p.down_ratio - 1.0) * (level - p.down_threshold_db)).max(p.down_range_db)
        } else {
            0.0
        };
        let mut v = db_amp(p.makeup_db + change);
        if level < p.exp_threshold_db {
            let exp = db_amp((level - p.exp_threshold_db) * (p.exp_ratio - 1.0));
            v = (v * exp).max(makeup);
        }
        *gain = v;
    }
    out
}

#[inline]
fn spline_gain(points: &[f64; CURVE_POINTS], level_db: f64, top_db: f64) -> f64 {
    let z = ((level_db.clamp(-100.0, top_db) + 100.0) * 10.0 / (100.0 + top_db)).clamp(0.0, 10.0);
    let i = libm::floor(z) as usize;
    let t = z - i as f64;
    let p0 = points[i.saturating_sub(1)];
    let p1 = points[i];
    let p2 = points[(i + 1).min(CURVE_POINTS - 1)];
    let p3 = points[(i + 2).min(CURVE_POINTS - 1)];
    let c3 = (-p0 + 3.0 * p1 - 3.0 * p2 + p3) / 6.0;
    let c2 = (p0 - 2.0 * p1 + p2) / 2.0;
    let c1 = (-p0 + p2) / 2.0;
    let c0 = (p0 + 4.0 * p1 + p2) / 6.0;
    ((c3 * t + c2) * t + c1) * t + c0
}

/// Allocation-free monotonic queue for an exact trailing-window maximum.
/// The largest installed window is 10 ms; storage covers rates through
/// 192 kHz, while construction clamps hostile rates to the fixed bound.
#[derive(Clone, Copy)]
struct SlidingMax {
    value: [f64; MAX_DETECTOR_WINDOW],
    index: [u64; MAX_DETECTOR_WINDOW],
    head: usize,
    len: usize,
    next: u64,
}

/// State for four cascaded second-order allpasses. Each section implements
/// `(a^2 - z^-2) / (1 - a^2 z^-2)` with two interleaved delay lanes.
#[derive(Clone, Copy, Default)]
struct QuadratureArm {
    input_delay: [[f32; 2]; QUADRATURE_STAGES],
    output_delay: [[f32; 2]; QUADRATURE_STAGES],
}

impl QuadratureArm {
    #[inline]
    fn push(&mut self, mut input: f32, coefficient: &[f32; QUADRATURE_STAGES], phase: usize) -> f32 {
        for (stage, &a_squared) in coefficient.iter().enumerate() {
            let output =
                a_squared * input - self.input_delay[stage][phase] + a_squared * self.output_delay[stage][phase];
            self.input_delay[stage][phase] = input;
            self.output_delay[stage][phase] = output;
            input = output;
        }
        input
    }
}

/// Phase-invariant envelope used by algorithms 0-2 at qualities 0 and 1.
///
/// These are Olli Niemitalo's public-domain 90-degree allpass coefficients
/// from <https://yehar.com/blog/?p=368>. The installed unit uses the same
/// topology: arm A has one additional sample of delay and the detector takes
/// `sqrt(B(v)^2 + (z^-1 A(v))^2)`. The coefficients are the public design,
/// not values fitted to machine captures.
#[derive(Clone, Copy, Default)]
struct QuadratureEnvelope {
    arm_a: QuadratureArm,
    arm_b: QuadratureArm,
    arm_a_delay: f32,
    phase: usize,
}

impl QuadratureEnvelope {
    const A: [f32; QUADRATURE_STAGES] = [
        0.692_387_8_f32 * 0.692_387_8_f32,
        0.936_065_43_f32 * 0.936_065_43_f32,
        0.988_229_5_f32 * 0.988_229_5_f32,
        0.998_748_84_f32 * 0.998_748_84_f32,
    ];
    const B: [f32; QUADRATURE_STAGES] = [
        0.402_192_12_f32 * 0.402_192_12_f32,
        0.856_171_1_f32 * 0.856_171_1_f32,
        0.972_290_93_f32 * 0.972_290_93_f32,
        0.995_288_5_f32 * 0.995_288_5_f32,
    ];

    fn reset(&mut self) {
        *self = Self::default();
    }

    #[inline]
    fn push(&mut self, input: f64) -> f64 {
        let input = input as f32;
        let b = self.arm_b.push(input, &Self::B, self.phase);
        let a = self.arm_a.push(input, &Self::A, self.phase);
        let delayed_a = self.arm_a_delay;
        self.arm_a_delay = a;
        self.phase ^= 1;
        libm::sqrt((b * b + delayed_a * delayed_a) as f64)
    }

    /// The audio path of the same pair: `(z^-1 A(x) + B(x)) / sqrt(2)`.
    /// Both arms are allpass and stay 90 degrees apart, so the sum is an
    /// allpass with unity magnitude whose group delay matches the detector
    /// envelope above.  Measured against the installed unit's impulse
    /// response, quality 0/1 differs from quality 2 by exactly this allpass
    /// in every band (within 1.3 degrees from 15 Hz to 22 kHz at 48 kHz).
    #[cfg(test)]
    #[inline]
    fn push_audio(&mut self, input: f64) -> f64 {
        let (delayed_a, b) = self.push_components(input as f32);
        ((b + delayed_a) as f64) * core::f64::consts::FRAC_1_SQRT_2
    }

    #[inline]
    fn push_components(&mut self, input: f32) -> (f32, f32) {
        let b = self.arm_b.push(input, &Self::B, self.phase);
        let a = self.arm_a.push(input, &Self::A, self.phase);
        let delayed_a = self.arm_a_delay;
        self.arm_a_delay = a;
        self.phase ^= 1;
        (delayed_a, b)
    }
}

/// The mode-4 arm law keeps a direct gain and a 50 Hz bilinear low-pass copy.
/// The difference drives a second quadrature pair; its two components rotate
/// the audio arms during a gain transition. A bounded power-ratio crossfade
/// selects the direct or corrected path. This is asymmetric by construction:
/// on release `gain / lowpass > 1`, so the crossfade clamps to the corrected
/// path and retains the allpass state that produces the native overshoot.
#[derive(Clone, Copy)]
struct Mode4Gain {
    b: f32,
    a: f32,
    previous_input: f32,
    previous_output: f32,
    quadrature: QuadratureEnvelope,
}

#[derive(Clone, Copy, Default)]
struct Mode4Mix {
    gain: f32,
    lowpass: f32,
    arm_a: f32,
    arm_b: f32,
    correction_weight: f32,
}

impl Mode4Gain {
    fn new(sample_rate: f64) -> Self {
        let k = libm::tan(core::f64::consts::PI * 50.0 / sample_rate);
        let b = (k / (1.0 + k)) as f32;
        Self {
            b,
            a: ((1.0 - k) / (1.0 + k)) as f32,
            previous_input: 0.0,
            previous_output: 0.0,
            quadrature: QuadratureEnvelope::default(),
        }
    }

    fn reset(&mut self) {
        self.previous_input = 0.0;
        self.previous_output = 0.0;
        self.quadrature.reset();
    }

    #[inline]
    fn step(&mut self, gain: f64) -> Mode4Mix {
        let gain = gain as f32;
        let lowpass = libm::fmaf(self.b, gain, libm::fmaf(self.b, self.previous_input, self.a * self.previous_output));
        self.previous_input = gain;
        self.previous_output = lowpass;
        let (arm_a, arm_b) = self.quadrature.push_components(gain - lowpass);
        let ratio = mode4_ratio(gain, lowpass);
        let ratio_squared = ratio * ratio;
        let correction_weight = (1.0 - ratio_squared).clamp(0.0, 1.0);
        Mode4Mix { gain, lowpass, arm_a, arm_b, correction_weight }
    }
}

impl Mode4Mix {
    #[inline]
    fn apply_direct(self, arm_a: f64, arm_b: f64) -> f64 {
        (((arm_a as f32 + arm_b as f32) * self.gain) * core::f32::consts::FRAC_1_SQRT_2) as f64
    }

    #[inline]
    fn apply(self, arm_a: f64, arm_b: f64) -> f64 {
        let arm_a = arm_a as f32;
        let arm_b = arm_b as f32;
        let sum = arm_a + arm_b;
        let direct = (sum * self.gain) * core::f32::consts::FRAC_1_SQRT_2;
        let rotated = arm_a * self.arm_a + arm_b * self.arm_b;
        let corrected = (sum * self.lowpass + rotated) * core::f32::consts::FRAC_1_SQRT_2;
        let direct_weight = self.correction_weight;
        (corrected * (1.0 - direct_weight) + direct * direct_weight) as f64
    }
}

#[inline]
fn mode4_ratio(numerator: f32, denominator: f32) -> f32 {
    if denominator == 0.0 {
        return if numerator == 0.0 { 0.0 } else { f32::INFINITY };
    }
    #[cfg(target_arch = "aarch64")]
    {
        use core::arch::aarch64::{vrecpes_f32, vrecpss_f32};
        // SAFETY: scalar NEON reciprocal instructions have no memory effects
        // and are available on every AArch64 target supported by this crate.
        unsafe {
            let mut reciprocal = vrecpes_f32(denominator);
            reciprocal *= vrecpss_f32(denominator, reciprocal);
            let step = vrecpss_f32(denominator, reciprocal);
            (reciprocal * numerator) * step
        }
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        numerator / denominator
    }
}

/// Causal polyphase-equivalent interpolator used for a reduced-rate gain
/// vector. The reference algorithm constructs the FIR by convolving a normalized
/// one-millisecond sine window with the linear interpolation kernel. Running
/// the zero-stuffed convolution one audio frame at a time makes slice phase
/// explicit and keeps arbitrary process block sizes bit-for-bit continuous.
#[derive(Clone, Copy)]
struct GainSrc {
    taps: [f32; MAX_GAIN_SRC_TAPS],
    ring: [f32; MAX_GAIN_SRC_TAPS],
    len: usize,
    cursor: usize,
    factor: usize,
    phase: usize,
}

impl GainSrc {
    const EMPTY: Self =
        Self { taps: [0.0; MAX_GAIN_SRC_TAPS], ring: [0.0; MAX_GAIN_SRC_TAPS], len: 1, cursor: 0, factor: 1, phase: 0 };

    fn new(factor: usize, fs: f64) -> Self {
        let factor = factor.clamp(1, 8);
        let window_len = (libm::round(fs / 1000.0) as usize).clamp(1, 192);
        let triangle_len = 2 * factor - 1;
        let len = (window_len + triangle_len - 1).min(MAX_GAIN_SRC_TAPS);
        let mut window = [0.0f32; 192];
        let mut sum = 0.0f32;
        for (i, value) in window[..window_len].iter_mut().enumerate() {
            *value = libm::sin(core::f64::consts::PI * (i + 1) as f64 / (window_len + 1) as f64) as f32;
            sum += *value;
        }
        for value in &mut window[..window_len] {
            *value /= sum;
        }
        let mut out = Self { len, factor, ..Self::EMPTY };
        for i in 0..len {
            let mut value = 0.0f32;
            let first = i.saturating_sub(triangle_len - 1);
            let last = i.min(window_len - 1);
            for (j, window_value) in window.iter().enumerate().take(last + 1).skip(first) {
                let k = i - j;
                let triangle = if k < factor { (k + 1) as f32 } else { (triangle_len - k) as f32 } / factor as f32;
                value += *window_value * triangle;
            }
            out.taps[i] = value;
        }
        out
    }

    fn reset(&mut self) {
        self.ring = [0.0; MAX_GAIN_SRC_TAPS];
        self.cursor = 0;
        self.phase = 0;
    }

    /// The producer supplies a value every `factor` frames, starting at
    /// frame zero. All other ring entries are the explicit inserted zeros.
    #[inline]
    fn step(&mut self, input: Option<f64>) -> f64 {
        self.ring[self.cursor] = input.unwrap_or(0.0) as f32;
        let mut output = 0.0f32;
        let mut index =
            if self.cursor >= self.phase { self.cursor - self.phase } else { self.cursor + self.len - self.phase };
        // Keep the original order of every nonzero product and addition.
        // Omitting inserted zero products does not reassociate the sum or
        // fuse any operation; the accumulator still starts at positive zero.
        for tap in self.taps[self.phase..self.len].iter().step_by(self.factor) {
            output += *tap * self.ring[index];
            index = if index >= self.factor { index - self.factor } else { index + self.len - self.factor };
        }
        self.phase += 1;
        if self.phase == self.factor {
            self.phase = 0;
        }
        self.cursor += 1;
        if self.cursor == self.len {
            self.cursor = 0;
        }
        output as f64
    }
}

/// How the reduced-rate detector obtains one value per update.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Cadence {
    /// Every Nth detector-band sample, no prefilter (algorithm 3 and every
    /// quality-2 setting).
    Pick,
    /// A box carrying threshold automation renders in eight-frame
    /// automation sub-blocks and picks one real-waveform detector sample per
    /// sub-block in every band.
    PickEight,
    /// One value per fixed eight-frame hop in every band, at every rate
    /// (algorithms 0-2 at quality 0 and 1). The hop keeps the largest
    /// magnitude it saw, so no peak between updates is lost.
    HopPeak,
}

/// Observable cadence of the unit's gain updates (black-box: the first
/// output sample that a level step moves, swept over the step's offset).
/// Algorithm 3, and quality 2 with any algorithm, update bands 2-3 every
/// fourth frame and drop the samples in between: a tone whose samples
/// alternate at that stride swings the gain by more than 12 dB with its
/// start phase.  Algorithms 0-2 at quality 0 and 1 update all four bands
/// every eight frames at 44.1, 48 and 96 kHz alike, and their steady gain
/// does not depend on a tone's phase in any band.  With saved threshold
/// automation (AU parameter 79 curves) the picking cadence is eight frames
/// in every band instead: the onset sawtooth has period 8 in bands 1-3 and
/// tones at multiples of fs/16 swing the gain with their start phase; the
/// same preset without its automation curves returns to [1, 2, 4, 4].  With
/// overshoot compensation off, that eight-frame pick of the Butterworth-half
/// magnitude reproduces the unit's steady tone gains above 300 Hz exactly.
fn cadence(p: &Params) -> Cadence {
    let picks = p.algorithm >= 3 || p.render_quality >= 2;
    if picks && p.threshold_automation.iter().any(|curve| curve.count != 0) {
        Cadence::PickEight
    } else if picks {
        Cadence::Pick
    } else {
        Cadence::HopPeak
    }
}

fn detector_factors(p: &Params) -> [usize; BANDS] {
    // The embedded compressor, including its window and release state, runs
    // at fs / N; its gain vector is sent through the separate causal outer
    // SRC above.
    match cadence(p) {
        Cadence::Pick => [1, 2, 4, 4],
        Cadence::PickEight => [8; BANDS],
        Cadence::HopPeak => [8; BANDS],
    }
}

/// Algorithm zero reduces band zero until the first image reaches the
/// quality's Butterworth stop-band budget. The unit uses 40/50 dB for q0/q1.
fn band_zero_factor(p: &Params, fs: f64) -> usize {
    if p.algorithm != 0 || p.render_quality >= 2 || p.band[0].mode != BandMode::Active {
        return 1;
    }
    let half_order = ((libm::ceil(p.crossover_order[0] * 0.5) as usize).max(1)) as f64;
    let edge = p.crossover_hz[0].min(p.crossover_hz[1] / 1.25).min(p.crossover_hz[2] / (1.25 * 1.25));
    let budget = if p.render_quality == 0 { 40.0 } else { 50.0 };
    let separation = 2.0 * libm::pow(2.0, budget / (6.0 * half_order));
    libm::floor(fs / (separation * edge)).max(1.0) as usize
}

impl SlidingMax {
    const EMPTY: Self =
        Self { value: [0.0; MAX_DETECTOR_WINDOW], index: [0; MAX_DETECTOR_WINDOW], head: 0, len: 0, next: 0 };

    #[inline]
    fn push(&mut self, value: f64, window: usize) -> f64 {
        let now = self.next;
        self.next = self.next.wrapping_add(1);
        while self.len != 0 && now.wrapping_sub(self.index[self.head]) >= window as u64 {
            self.head = (self.head + 1) % MAX_DETECTOR_WINDOW;
            self.len -= 1;
        }
        while self.len != 0 {
            let back = (self.head + self.len - 1) % MAX_DETECTOR_WINDOW;
            if self.value[back] > value {
                break;
            }
            self.len -= 1;
        }
        let back = (self.head + self.len) % MAX_DETECTOR_WINDOW;
        self.value[back] = value;
        self.index[back] = now;
        self.len += 1;
        self.value[self.head]
    }
}

/// The real-time state of one linked compressor instance.
pub struct Run {
    path: [Path; BANDS],
    legacy_stage: [LegacyStage; XOVERS],
    half_stage: [HalfStage; XOVERS],
    detector_path: [DetectorPath; BANDS],
    band: [Band; BANDS],
    base_band: [Band; BANDS],
    threshold_automation: [ThresholdAutomation; BANDS],
    in_gain: f64,
    out_gain: f64,
    band_in: [f64; BANDS],
    detector_bandwidth: [f64; BANDS],
    curve: [[f64; CURVE_POINTS]; BANDS],
    curve_top_db: f64,
    release: [f64; BANDS],
    release_on: bool,
    algorithm: u32,
    legacy_recombination: bool,
    overshoot_compensation: bool,
    channels: usize,
    env: [f64; BANDS],
    gain_state: [f64; BANDS],
    gain_rest: [f64; BANDS],
    detector_window: [usize; BANDS],
    detector_factor: [usize; BANDS],
    detector_phase: [usize; BANDS],
    detector_max: [SlidingMax; BANDS],
    quadrature: [[QuadratureEnvelope; BANDS]; MAX_CHANNELS],
    overshoot_quadrature: [[[QuadratureEnvelope; BANDS]; BANDS]; MAX_CHANNELS],
    audio_quadrature: [QuadratureEnvelope; MAX_CHANNELS],
    mode4_gain: [Mode4Gain; BANDS],
    band_zero_factor: usize,
    band_zero_phase: usize,
    hop_peak: bool,
    detector_hop: [f64; BANDS],
    gain_src: [GainSrc; BANDS],
    state: [[[[f64; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
    state_b: [[[[f64; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
    legacy_high_state: [[[[f64; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    legacy_high_state_b: [[[[f64; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    legacy_split_allpass_state: [[[[f64; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    legacy_split_allpass_state_b: [[[[f64; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    legacy_merge_allpass_state: [[[[f64; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    half_analysis_low_a: [[[[f32; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    half_analysis_high_a: [[[[f32; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    half_analysis_low_b: [[[[f32; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    half_analysis_high_b: [[[[f32; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    half_synthesis_low: [[[[f32; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    half_synthesis_high: [[[[f32; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
    half_synthesis_phase_low: [[[f32; 2]; MAX_FILTERS]; MAX_CHANNELS],
    half_synthesis_phase_high: [[[f32; 2]; MAX_FILTERS]; MAX_CHANNELS],
    detector_state: [[[[f32; 2]; MAX_DETECTOR_FILTERS]; BANDS]; MAX_CHANNELS],
    analytic_band_path: [Path; BANDS],
    analytic_reference_path: [Path; BANDS],
    analytic_band_state: [[[[f32; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
    analytic_reference_state: [[[[f32; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
    reference_quadrature: [[QuadratureEnvelope; BANDS]; MAX_CHANNELS],
    overshoot_state: [[[[[f32; 2]; MAX_DETECTOR_FILTERS]; BANDS]; BANDS]; MAX_CHANNELS],
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize) -> Run {
        let fs = if fs.is_finite() && fs > 1.0 { fs } else { 48_000.0 };
        let valid = p.valid(fs);
        let q = if valid { p } else { &Params::default() };
        let curve_top_db = if q.algorithm >= 2 { 20.0 } else { 0.0 };
        let gain_rest = q.band.map(|b| spline_gain(&curve_points(b, curve_top_db), -100.0, curve_top_db));
        let edges = [0.0, q.crossover_hz[0], q.crossover_hz[1], q.crossover_hz[2], 20_000.0];
        let reference_edges = [0.0, 100.0, 1000.0, 10_000.0, 20_000.0];
        let mut detector_bandwidth = [1.0; BANDS];
        if q.bandwidth_compensation {
            for b in 0..BANDS {
                // The component applies the normalized reference-band width
                // directly to its detector magnitude.  This is deliberately
                // not converted through sqrt: doing so raises the detector in
                // wide low bands and lowers it in narrow upper bands, tilting
                // the multiband gain away from the saved curve.
                detector_bandwidth[b] = (reference_edges[b + 1] - reference_edges[b]) / (edges[b + 1] - edges[b]);
            }
        }
        let detector_factor = detector_factors(q);
        // Mode 4 shares one two-millisecond odd trailing maximum across its
        // first three arms. The direct fourth band keeps its one-millisecond
        // window. At the fixed eight-frame cadence this is 11/13/25 updates
        // at 44.1/48/96 kHz for bands zero through two.
        let window_ms = if cadence(q) == Cadence::HopPeak { [2.0, 2.0, 2.0, 1.0] } else { [10.0, 8.0, 2.0, 1.0] };
        let mut detector_window = core::array::from_fn(|b| {
            ((libm::round(window_ms[b] * fs / (1000.0 * detector_factor[b] as f64)) as usize) | 1)
                .clamp(1, MAX_DETECTOR_WINDOW)
        });
        let mut release = core::array::from_fn(|b| {
            if q.release_s > 0.0 {
                libm::exp(-(detector_factor[b] as f64) / (q.release_s * fs))
            } else {
                0.0
            }
        });
        let band_zero_factor = band_zero_factor(q, fs);
        let band_zero_rate = fs / band_zero_factor as f64;
        if band_zero_factor > 1 {
            detector_window[0] = ((libm::round(2.0 * band_zero_rate / (1000.0 * detector_factor[0] as f64)) as usize)
                | 1)
            .clamp(1, MAX_DETECTOR_WINDOW);
            release[0] = if q.release_s > 0.0 {
                libm::exp(-(detector_factor[0] as f64) / (q.release_s * band_zero_rate))
            } else {
                0.0
            };
        }
        let (analytic_band_path, analytic_reference_path) = analytic_detector_paths(q, fs);
        let mut run = Run {
            path: paths(q, fs),
            legacy_stage: legacy_stages(q, fs),
            half_stage: half_stages(q, fs),
            detector_path: detector_paths(q, fs),
            band: q.band,
            base_band: q.band,
            threshold_automation: q.threshold_automation,
            in_gain: db_amp(q.input_gain_db),
            out_gain: db_amp(q.output_gain_db),
            band_in: q.band.map(|b| db_amp(b.input_gain_db)),
            detector_bandwidth,
            curve: q.band.map(|b| curve_points(b, curve_top_db)),
            curve_top_db,
            release,
            release_on: q.release_on,
            algorithm: q.algorithm,
            legacy_recombination: q.render_quality == 1 && q.algorithm < 3,
            overshoot_compensation: q.overshoot_compensation,
            channels: channels.min(MAX_CHANNELS),
            env: [0.0; BANDS],
            gain_state: gain_rest,
            gain_rest,
            detector_window,
            detector_factor,
            detector_phase: [0; BANDS],
            detector_max: [SlidingMax::EMPTY; BANDS],
            quadrature: [[QuadratureEnvelope::default(); BANDS]; MAX_CHANNELS],
            overshoot_quadrature: [[[QuadratureEnvelope::default(); BANDS]; BANDS]; MAX_CHANNELS],
            audio_quadrature: [QuadratureEnvelope::default(); MAX_CHANNELS],
            mode4_gain: core::array::from_fn(|b| Mode4Gain::new(if b == 0 { band_zero_rate } else { fs })),
            band_zero_factor,
            band_zero_phase: 0,
            hop_peak: cadence(q) == Cadence::HopPeak,
            detector_hop: [0.0; BANDS],
            gain_src: core::array::from_fn(|b| {
                GainSrc::new(detector_factor[b], if b == 0 { band_zero_rate } else { fs })
            }),
            state: [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
            state_b: [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
            legacy_high_state: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            legacy_high_state_b: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            legacy_split_allpass_state: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            legacy_split_allpass_state_b: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            legacy_merge_allpass_state: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            half_analysis_low_a: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            half_analysis_high_a: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            half_analysis_low_b: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            half_analysis_high_b: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            half_synthesis_low: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            half_synthesis_high: [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS],
            half_synthesis_phase_low: [[[0.0; 2]; MAX_FILTERS]; MAX_CHANNELS],
            half_synthesis_phase_high: [[[0.0; 2]; MAX_FILTERS]; MAX_CHANNELS],
            detector_state: [[[[0.0; 2]; MAX_DETECTOR_FILTERS]; BANDS]; MAX_CHANNELS],
            analytic_band_path,
            analytic_reference_path,
            analytic_band_state: [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
            analytic_reference_state: [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS],
            reference_quadrature: [[QuadratureEnvelope::default(); BANDS]; MAX_CHANNELS],
            overshoot_state: [[[[[0.0; 2]; MAX_DETECTOR_FILTERS]; BANDS]; BANDS]; MAX_CHANNELS],
        };
        run.set_automation_input(q.automation_input);
        run
    }

    pub fn reset(&mut self) {
        self.env = [0.0; BANDS];
        self.gain_state = self.gain_rest;
        self.detector_max = [SlidingMax::EMPTY; BANDS];
        self.detector_phase = [0; BANDS];
        for envelope in &mut self.audio_quadrature {
            envelope.reset();
        }
        for channel in &mut self.quadrature {
            for envelope in channel {
                envelope.reset();
            }
        }
        for channel in &mut self.overshoot_quadrature {
            for band in channel {
                for envelope in band {
                    envelope.reset();
                }
            }
        }
        self.detector_hop = [0.0; BANDS];
        for src in &mut self.gain_src {
            src.reset();
        }
        for gain in &mut self.mode4_gain {
            gain.reset();
        }
        self.band_zero_phase = 0;
        self.state = [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS];
        self.state_b = [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS];
        self.legacy_high_state = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.legacy_high_state_b = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.legacy_split_allpass_state = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.legacy_split_allpass_state_b = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.legacy_merge_allpass_state = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.half_analysis_low_a = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.half_analysis_high_a = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.half_analysis_low_b = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.half_analysis_high_b = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.half_synthesis_low = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.half_synthesis_high = [[[[0.0; 2]; MAX_FILTERS]; XOVERS]; MAX_CHANNELS];
        self.half_synthesis_phase_low = [[[0.0; 2]; MAX_FILTERS]; MAX_CHANNELS];
        self.half_synthesis_phase_high = [[[0.0; 2]; MAX_FILTERS]; MAX_CHANNELS];
        self.detector_state = [[[[0.0; 2]; MAX_DETECTOR_FILTERS]; BANDS]; MAX_CHANNELS];
        self.analytic_band_state = [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS];
        self.analytic_reference_state = [[[[0.0; 2]; MAX_FILTERS]; BANDS]; MAX_CHANNELS];
        self.reference_quadrature = [[QuadratureEnvelope::default(); BANDS]; MAX_CHANNELS];
        self.overshoot_state = [[[[[0.0; 2]; MAX_DETECTOR_FILTERS]; BANDS]; BANDS]; MAX_CHANNELS];
    }

    pub fn latency(&self) -> usize {
        0
    }

    /// Sets the box automation input (AU parameter 79). Curves are clamped
    /// at their endpoints and update only the four downward thresholds.
    pub fn set_automation_input(&mut self, input: f64) {
        if !input.is_finite() {
            return;
        }
        for band in 0..BANDS {
            let mut next = self.base_band[band];
            if let Some(threshold) = self.threshold_automation[band].value(input) {
                next.down_threshold_db = threshold;
            }
            self.band[band] = next;
            self.curve[band] = curve_points(next, self.curve_top_db);
        }
    }

    #[cfg(test)]
    pub(crate) fn down_threshold_db(&self, band: usize) -> Option<f64> {
        self.band.get(band).map(|state| state.down_threshold_db)
    }

    #[inline]
    fn split(&mut self, c: usize, x: f64) -> [f64; BANDS] {
        if self.legacy_recombination {
            return self.legacy_split(c, x);
        }
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

    #[inline]
    fn split_b(&mut self, c: usize, x: f64) -> [f64; BANDS] {
        if self.legacy_recombination {
            return self.legacy_split_b(c, x);
        }
        let mut out = [0.0; BANDS];
        for (b, y) in out.iter_mut().enumerate() {
            let mut v = x;
            for i in 0..self.path[b].count {
                let s = self.path[b].section[i];
                let z = &mut self.state_b[c][b][i];
                let o = s.b0 * v + z[0];
                z[0] = s.b1 * v - s.a1 * o + z[1];
                z[1] = s.b2 * v - s.a2 * o;
                v = o;
            }
            *y = v * self.band_in[b];
        }
        out
    }

    #[inline]
    fn run_path(path: Path, state: &mut [[f64; 2]; MAX_FILTERS], mut value: f64) -> f64 {
        for (s, z) in path.section[..path.count].iter().zip(state.iter_mut()) {
            let output = s.b0 * value + z[0];
            z[0] = s.b1 * value - s.a1 * output + z[1];
            z[1] = s.b2 * value - s.a2 * output;
            value = output;
        }
        value
    }

    #[inline]
    fn legacy_split(&mut self, c: usize, x: f64) -> [f64; BANDS] {
        let mut out = [0.0; BANDS];
        let mut remaining = x;
        for (stage, band) in out.iter_mut().enumerate().take(XOVERS) {
            let filters = self.legacy_stage[stage];
            let high = Self::run_path(filters.high, &mut self.legacy_high_state[c][stage], remaining);
            let allpass = Self::run_path(filters.allpass, &mut self.legacy_split_allpass_state[c][stage], remaining);
            *band = (allpass - high) * self.band_in[stage];
            remaining = high;
        }
        out[BANDS - 1] = remaining * self.band_in[BANDS - 1];
        out
    }

    #[inline]
    fn legacy_split_b(&mut self, c: usize, x: f64) -> [f64; BANDS] {
        let mut out = [0.0; BANDS];
        let mut remaining = x;
        for (stage, band) in out.iter_mut().enumerate().take(XOVERS) {
            let filters = self.legacy_stage[stage];
            let high = Self::run_path(filters.high, &mut self.legacy_high_state_b[c][stage], remaining);
            let allpass = Self::run_path(filters.allpass, &mut self.legacy_split_allpass_state_b[c][stage], remaining);
            *band = (allpass - high) * self.band_in[stage];
            remaining = high;
        }
        out[BANDS - 1] = remaining * self.band_in[BANDS - 1];
        out
    }

    #[inline]
    fn legacy_recombine(&mut self, c: usize, bands: [f64; BANDS]) -> f64 {
        let mut output = bands[0];
        for (stage, band) in bands.iter().enumerate().take(XOVERS).skip(1) {
            output = Self::run_path(
                self.legacy_stage[stage].allpass,
                &mut self.legacy_merge_allpass_state[c][stage],
                output,
            ) + *band;
        }
        output + bands[BANDS - 1]
    }

    #[inline]
    fn half_analyze(
        stages: [HalfStage; XOVERS],
        legacy: bool,
        band_in: [f64; BANDS],
        low_state: &mut [[[f32; 2]; MAX_FILTERS]; XOVERS],
        high_state: &mut [[[f32; 2]; MAX_FILTERS]; XOVERS],
        input: f64,
    ) -> [f64; BANDS] {
        let mut out = [0.0; BANDS];
        if legacy {
            let mut remaining = input;
            for stage in 0..XOVERS {
                out[stage] =
                    Self::run_detector_path(&stages[stage].low, &mut low_state[stage], remaining) * band_in[stage];
                remaining = Self::run_detector_path(&stages[stage].high, &mut high_state[stage], remaining);
            }
            out[BANDS - 1] = remaining * band_in[BANDS - 1];
        } else {
            let low_mid = Self::run_detector_path(&stages[1].low, &mut low_state[1], input);
            let high_mid = Self::run_detector_path(&stages[1].high, &mut high_state[1], input);
            out[0] = Self::run_detector_path(&stages[0].low, &mut low_state[0], low_mid) * band_in[0];
            out[1] = Self::run_detector_path(&stages[0].high, &mut high_state[0], low_mid) * band_in[1];
            out[2] = Self::run_detector_path(&stages[2].low, &mut low_state[2], high_mid) * band_in[2];
            out[3] = Self::run_detector_path(&stages[2].high, &mut high_state[2], high_mid) * band_in[3];
        }
        out
    }

    #[inline]
    fn half_synthesize(
        stages: [HalfStage; XOVERS],
        legacy: bool,
        low_state: &mut [[[f32; 2]; MAX_FILTERS]; XOVERS],
        high_state: &mut [[[f32; 2]; MAX_FILTERS]; XOVERS],
        phase_low_state: &mut [[f32; 2]; MAX_FILTERS],
        phase_high_state: &mut [[f32; 2]; MAX_FILTERS],
        bands: [f64; BANDS],
    ) -> f64 {
        if legacy {
            let mut remaining = bands[BANDS - 1];
            for stage in (0..XOVERS).rev() {
                let low = Self::run_detector_path(&stages[stage].low, &mut low_state[stage], bands[stage]);
                let high = Self::run_detector_path(&stages[stage].high, &mut high_state[stage], remaining)
                    * stages[stage].high_sign;
                remaining = low + high;
            }
            remaining
        } else {
            let low_pair = Self::run_detector_path(&stages[0].low, &mut low_state[0], bands[0])
                + stages[0].high_sign * Self::run_detector_path(&stages[0].high, &mut high_state[0], bands[1]);
            let high_pair = Self::run_detector_path(&stages[2].low, &mut low_state[2], bands[2])
                + stages[2].high_sign * Self::run_detector_path(&stages[2].high, &mut high_state[2], bands[3]);
            let low_side = Self::run_detector_path(&stages[2].low, phase_low_state, low_pair);
            let high_side = stages[0].high_sign * Self::run_detector_path(&stages[0].high, phase_high_state, high_pair);
            Self::run_detector_path(&stages[1].low, &mut low_state[1], low_side)
                + stages[1].high_sign * Self::run_detector_path(&stages[1].high, &mut high_state[1], high_side)
        }
    }

    #[inline]
    fn detect(&mut self, c: usize, x: f64) -> ([f64; BANDS], [f64; BANDS]) {
        let mut base = [0.0; BANDS];
        let mut overshoot = [0.0; BANDS];
        for b in 0..BANDS {
            let mut v = x;
            for i in 0..self.detector_path[b].count {
                let s = self.detector_path[b].section[i];
                let z = &mut self.detector_state[c][b][i];
                let o = s.b0 * v + z[0] as f64;
                z[0] = (s.b1 * v - s.a1 * o + z[1] as f64) as f32;
                z[1] = (s.b2 * v - s.a2 * o) as f32;
                v = o;
            }
            let scale = self.band_in[b] * self.detector_bandwidth[b];
            let magnitude = if self.hop_peak {
                let band = Self::run_detector_path(&self.analytic_band_path[b], &mut self.analytic_band_state[c][b], x);
                let reference = Self::run_detector_path(
                    &self.analytic_reference_path[b],
                    &mut self.analytic_reference_state[c][b],
                    x,
                );
                libm::sqrt(self.quadrature[c][b].push(band) * self.reference_quadrature[c][b].push(reference))
            } else {
                v.abs()
            };
            base[b] = (magnitude * scale) as f32 as f64;
            if self.overshoot_compensation {
                let mut sum = 0.0;
                for analysis_band in 0..BANDS {
                    let mut branch = v;
                    for i in 0..self.detector_path[analysis_band].count {
                        let s = self.detector_path[analysis_band].section[i];
                        let z = &mut self.overshoot_state[c][b][analysis_band][i];
                        let o = s.b0 * branch + z[0] as f64;
                        z[0] = (s.b1 * branch - s.a1 * o + z[1] as f64) as f32;
                        z[1] = (s.b2 * branch - s.a2 * o) as f32;
                        branch = o;
                    }
                    sum += if self.hop_peak {
                        self.overshoot_quadrature[c][b][analysis_band].push(branch)
                    } else {
                        branch.abs()
                    };
                }
                overshoot[b] = (sum * scale) as f32 as f64;
            }
        }
        (base, overshoot)
    }

    /// Single-precision-state biquad cascade (the detector's storage width).
    #[inline]
    fn run_detector_path(path: &Path, state: &mut [[f32; 2]; MAX_FILTERS], mut v: f64) -> f64 {
        for (s, z) in path.section[..path.count].iter().zip(state.iter_mut()) {
            let o = s.b0 * v + z[0] as f64;
            z[0] = (s.b1 * v - s.a1 * o + z[1] as f64) as f32;
            z[1] = (s.b2 * v - s.a2 * o) as f32;
            v = o;
        }
        v
    }

    fn gain(&mut self, b: usize) -> f64 {
        let p = self.band[b];
        if p.mode == BandMode::Mute {
            return 0.0;
        }
        if p.mode == BandMode::Pass {
            return 1.0;
        }
        let level = 20.0 * libm::log10(self.env[b].max(1e-5));
        spline_gain(&self.curve[b], level, self.curve_top_db)
    }

    #[inline]
    fn window_max(&mut self, b: usize, value: f64) -> f64 {
        self.detector_max[b].push(value, self.detector_window[b])
    }

    #[inline]
    fn update_gain(&mut self, b: usize, held: f64) -> f64 {
        self.env[b] = held;
        let target = self.gain(b);
        let recovery = if self.release_on { self.release[b] } else { 0.0 };
        let gain =
            if target <= self.gain_state[b] { target } else { target + (self.gain_state[b] - target) * recovery };
        self.gain_state[b] = gain;
        gain
    }

    #[inline]
    fn saturate(flavor: Option<u32>, x: f64) -> f64 {
        let Some(f) = flavor else { return x };
        match f {
            0 => x.clamp(-1.0, 1.0),
            1 => {
                let v = x.clamp(-2.965, 2.965);
                1.011_726_737 * v - 0.153_421_387 * v * v * v
            }
            2 => libm::sin(x),
            3 => libm::sin(x.clamp(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2)),
            4 => 0.5 * (x + libm::sin(x)),
            _ => x,
        }
    }

    #[inline]
    fn band_output(flavor: Option<u32>, gain: f64, x: f64) -> f64 {
        gain * Self::saturate(flavor, x)
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let n = bufs.len().min(self.channels).min(MAX_CHANNELS);
        let mut bands_a = [[0.0; BANDS]; MAX_CHANNELS];
        let mut bands_b = [[0.0; BANDS]; MAX_CHANNELS];
        for i in 0..frames {
            let band_zero_sample = self.band_zero_phase == 0;
            let mut detector = [0.0f64; BANDS];
            let mut overshoot = [0.0f64; BANDS];
            for c in 0..n {
                let x = bufs[c].get(i).copied().unwrap_or(0.0);
                let x = if x.is_finite() { x.clamp(-INPUT_BOUND, INPUT_BOUND) as f64 } else { 0.0 };
                let input = self.in_gain * x;
                // Mode 4 keeps the two analytic components separate through
                // the band tree. They collapse only after the coupled gain
                // recurrence below; quality 2 keeps the real path.
                if self.hop_peak {
                    let (a, b) = self.audio_quadrature[c].push_components(input as f32);
                    bands_a[c] = Self::half_analyze(
                        self.half_stage,
                        self.legacy_recombination,
                        self.band_in,
                        &mut self.half_analysis_low_a[c],
                        &mut self.half_analysis_high_a[c],
                        f64::from(a),
                    );
                    bands_b[c] = Self::half_analyze(
                        self.half_stage,
                        self.legacy_recombination,
                        self.band_in,
                        &mut self.half_analysis_low_b[c],
                        &mut self.half_analysis_high_b[c],
                        f64::from(b),
                    );
                } else {
                    bands_a[c] = self.split(c, input);
                }
                let (detected, overshoot_arm) = self.detect(c, input);
                for b in 0..BANDS {
                    detector[b] = detector[b].max(detected[b]);
                    overshoot[b] = overshoot[b].max(overshoot_arm[b]);
                }
            }
            let mut gain = [0.0; BANDS];
            for (b, mut p) in detector.into_iter().enumerate() {
                if b == 0 && self.band_zero_factor > 1 && !band_zero_sample {
                    continue;
                }
                if self.overshoot_compensation && self.algorithm < 3 {
                    p = p.max(overshoot[b]);
                } else if self.overshoot_compensation && self.algorithm == 3 {
                    // Algorithm 3 stays on its inherited real-waveform
                    // detector; its exact estimator belongs to t-5368.
                    p = overshoot[b];
                }
                let update = self.detector_phase[b] == 0;
                let p = if self.hop_peak {
                    let peak = self.detector_hop[b].max(p);
                    self.detector_hop[b] = if update { 0.0 } else { peak };
                    peak
                } else {
                    p
                };
                let input = if update {
                    let held = self.window_max(b, p);
                    // Every algorithm releases the post-curve linear gain.
                    // Reduction applies immediately; recovery is a one-pole
                    // at the embedded detector rate.  Quality changes the
                    // detector producer and cadence, not this gain state.
                    Some(self.update_gain(b, held))
                } else {
                    None
                };
                gain[b] = match self.band[b].mode {
                    BandMode::Active => self.gain_src[b].step(input),
                    BandMode::Pass => 1.0,
                    BandMode::Mute => 0.0,
                };
                self.detector_phase[b] += 1;
                if self.detector_phase[b] == self.detector_factor[b] {
                    self.detector_phase[b] = 0;
                }
            }
            let mode4 = if self.hop_peak {
                core::array::from_fn(|b| {
                    if b == 0 && self.band_zero_factor > 1 && !band_zero_sample {
                        Mode4Mix::default()
                    } else {
                        self.mode4_gain[b].step(gain[b])
                    }
                })
            } else {
                [Mode4Mix::default(); BANDS]
            };
            for c in 0..n {
                let mut processed = [0.0; BANDS];
                for b in 0..BANDS {
                    processed[b] = if self.hop_peak {
                        let output = if b == 0 && self.band_zero_factor > 1 && !band_zero_sample {
                            0.0
                        } else if b + 1 == BANDS {
                            mode4[b].apply_direct(bands_a[c][b], bands_b[c][b])
                        } else {
                            mode4[b].apply(bands_a[c][b], bands_b[c][b])
                        };
                        let scale =
                            if b == 0 && self.band_zero_factor > 1 { self.band_zero_factor as f64 } else { 1.0 };
                        Self::saturate(self.band[b].saturation, output) * scale
                    } else {
                        Self::band_output(self.band[b].saturation, gain[b], bands_a[c][b])
                    };
                }
                let mut y = if self.hop_peak {
                    Self::half_synthesize(
                        self.half_stage,
                        self.legacy_recombination,
                        &mut self.half_synthesis_low[c],
                        &mut self.half_synthesis_high[c],
                        &mut self.half_synthesis_phase_low[c],
                        &mut self.half_synthesis_phase_high[c],
                        processed,
                    )
                } else if self.legacy_recombination {
                    self.legacy_recombine(c, processed)
                } else {
                    processed.into_iter().sum()
                };
                y *= self.out_gain;
                if let Some(out) = bufs[c].get_mut(i) {
                    *out = if y.is_finite() { y as f32 } else { 0.0 };
                }
                if !y.is_finite() {
                    self.state[c] = [[[0.0; 2]; MAX_FILTERS]; BANDS];
                    self.state_b[c] = [[[0.0; 2]; MAX_FILTERS]; BANDS];
                    self.legacy_high_state[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.legacy_high_state_b[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.legacy_split_allpass_state[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.legacy_split_allpass_state_b[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.legacy_merge_allpass_state[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.half_analysis_low_a[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.half_analysis_high_a[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.half_analysis_low_b[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.half_analysis_high_b[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.half_synthesis_low[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.half_synthesis_high[c] = [[[0.0; 2]; MAX_FILTERS]; XOVERS];
                    self.half_synthesis_phase_low[c] = [[0.0; 2]; MAX_FILTERS];
                    self.half_synthesis_phase_high[c] = [[0.0; 2]; MAX_FILTERS];
                    self.audio_quadrature[c].reset();
                }
            }
            if self.env.iter().any(|v| !v.is_finite()) {
                self.env = [0.0; BANDS];
            }
            self.band_zero_phase += 1;
            if self.band_zero_phase == self.band_zero_factor {
                self.band_zero_phase = 0;
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
    fn threshold_automation_interpolates_and_clamps() {
        let mut p = params();
        p.automation_input = -15.0;
        p.automation_min = -30.0;
        p.automation_max = 0.0;
        p.threshold_automation[0].count = 3;
        p.threshold_automation[0].points[..3].copy_from_slice(&[
            AutomationPoint { input: -30.0, value: -10.0 },
            AutomationPoint { input: -10.0, value: -20.0 },
            AutomationPoint { input: 0.0, value: -24.0 },
        ]);
        let mut run = Run::new(&p, 48_000.0, 1);
        assert_eq!(run.band[0].down_threshold_db, -17.5);
        run.set_automation_input(-100.0);
        assert_eq!(run.band[0].down_threshold_db, -10.0);
        run.set_automation_input(10.0);
        assert_eq!(run.band[0].down_threshold_db, -24.0);
        assert_eq!(run.band[1].down_threshold_db, p.band[1].down_threshold_db);
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
        sections_magnitude(&path.section[..path.count], f)
    }

    fn detector_band_magnitude(path: &DetectorPath, f: f64) -> f64 {
        sections_magnitude(&path.section[..path.count], f)
    }

    fn sections_magnitude(sections: &[Section], f: f64) -> f64 {
        let w = 2.0 * core::f64::consts::PI * f / 48_000.0;
        let (c1, s1, c2, s2) = (libm::cos(w), libm::sin(w), libm::cos(2.0 * w), libm::sin(2.0 * w));
        let mut m = 1.0;
        for s in sections {
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
            p.algorithm = 3;
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
            p.algorithm = 1;
            p.render_quality = 1;
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
                // Quality 1 runs the audio through the quadrature pair, whose
                // arms are 90 +- 0.7 degrees apart above 30 Hz: the sum is
                // unity within 0.06 dB.
                assert!((level(&x, f) - 20.0 * libm::log10(0.25)).abs() < 0.07, "order {order} at {f} Hz");
            }
        }
    }

    #[test]
    fn algorithms_one_and_two_share_the_fixed_recombination_path() {
        let mut expected = None;
        for algorithm in 1..=2 {
            let mut p = params();
            p.algorithm = algorithm;
            p.render_quality = 1;
            p.input_gain_db = 0.0;
            p.output_gain_db = 0.0;
            for band in &mut p.band {
                band.mode = BandMode::Pass;
                band.saturation = None;
            }
            let mut impulse = std::vec![0.0; 4096];
            impulse[0] = 1.0;
            Run::new(&p, 48_000.0, 1).process(&mut [&mut impulse], 4096);
            if let Some(expected) = &expected {
                assert_eq!(&impulse, expected);
            } else {
                expected = Some(impulse);
            }
        }
    }

    #[test]
    fn detector_is_the_butterworth_half_of_each_audio_band() {
        let mut p = params();
        p.algorithm = 3;
        let audio = paths(&p, 48_000.0);
        let detector = detector_paths(&p, 48_000.0);
        for b in 0..BANDS {
            for f in [40.0, 200.0, 1000.0, 5000.0, 16_000.0] {
                let a = band_magnitude(&audio[b], f);
                let d = detector_band_magnitude(&detector[b], f);
                assert!((a - d * d).abs() < 1e-10, "band {b} at {f} Hz: audio {a}, detector {d}");
            }
        }
    }

    #[test]
    fn bandwidth_compensation_applies_the_normalized_band_width() {
        let mut p = params();
        p.crossover_hz = [400.0, 1600.0, 8000.0];
        p.bandwidth_compensation = true;
        let run = Run::new(&p, 48_000.0, 1);
        let expected = [100.0 / 400.0, 900.0 / 1200.0, 9000.0 / 6400.0, 10_000.0 / 12_000.0];
        for (got, expected) in run.detector_bandwidth.into_iter().zip(expected) {
            assert!((got - expected).abs() < 1e-12, "got {got}, expected {expected}");
        }

        p.bandwidth_compensation = false;
        assert_eq!(Run::new(&p, 48_000.0, 1).detector_bandwidth, [1.0; BANDS]);
    }

    #[test]
    fn detector_cadences_scale_windows_and_keep_the_next_peak() {
        let run = Run::new(&params(), 48_000.0, 1);
        assert_eq!(run.detector_factor, [8; BANDS]);
        assert_eq!(run.band_zero_factor, 11);
        assert_eq!(run.detector_window, [1, 13, 13, 7]);
        for b in 0..BANDS {
            let rate = if b == 0 { 48_000.0 / run.band_zero_factor as f64 } else { 48_000.0 };
            let embedded_fs = rate / run.detector_factor[b] as f64;
            assert!((run.release[b] - libm::exp(-1.0 / (params().release_s * embedded_fs))).abs() < 1e-12);
        }
        let run = Run::new(&params(), 96_000.0, 1);
        assert_eq!(run.band_zero_factor, 23);
        assert_eq!(run.detector_window, [1, 25, 25, 13]);

        let mut p = params();
        p.algorithm = 3;
        let run = Run::new(&p, 48_000.0, 1);
        assert_eq!(run.detector_factor, [1, 2, 4, 4]);
        assert_eq!(run.detector_window, [481, 193, 25, 13]);
        p.algorithm = 0;
        p.render_quality = 2;
        assert_eq!(Run::new(&p, 48_000.0, 1).detector_factor, [1, 2, 4, 4]);

        // Threshold automation switches the picking cadence to eight frames.
        let mut automated = p;
        automated.threshold_automation[0].count = 2;
        automated.threshold_automation[0].points[0] = AutomationPoint { input: -100.0, value: -20.0 };
        automated.threshold_automation[0].points[1] = AutomationPoint { input: 0.0, value: -30.0 };
        let run = Run::new(&automated, 48_000.0, 1);
        assert_eq!(run.detector_factor, [8; BANDS]);
        assert_eq!(run.detector_window, [61, 49, 13, 7]);
        assert!(!run.hop_peak);

        let mut max = SlidingMax::EMPTY;
        assert_eq!(max.push(9.0, 3), 9.0);
        assert_eq!(max.push(7.0, 3), 9.0);
        assert_eq!(max.push(8.0, 3), 9.0);
        // The 9 has left, but the lower 8 is still in the trailing window.
        assert_eq!(max.push(1.0, 3), 8.0);
        assert_eq!(max.push(2.0, 3), 8.0);
        assert_eq!(max.push(3.0, 3), 3.0);
    }

    #[test]
    fn algorithm_zero_band_rate_matches_the_stopband_budget() {
        let mut p = params();
        p.algorithm = 0;
        p.render_quality = 0;
        p.crossover_hz[1..].copy_from_slice(&[6000.0, 16_000.0]);
        for (x0, expected) in
            [(100.0, 23), (150.0, 15), (200.0, 11), (250.0, 9), (300.0, 7), (400.0, 5), (500.0, 4), (600.0, 3)]
        {
            p.crossover_hz[0] = x0;
            assert_eq!(band_zero_factor(&p, 48_000.0), expected, "q0 x0={x0}");
        }
        p.render_quality = 1;
        for (x0, expected) in [(100.0, 13), (150.0, 8), (200.0, 6), (250.0, 5), (300.0, 4), (1000.0, 1)] {
            p.crossover_hz[0] = x0;
            assert_eq!(band_zero_factor(&p, 48_000.0), expected, "q1 x0={x0}");
        }
        p.crossover_hz[0] = 200.0;
        assert_eq!(band_zero_factor(&p, 96_000.0), 13);
        p.algorithm = 2;
        assert_eq!(band_zero_factor(&p, 48_000.0), 1);
    }

    #[test]
    fn quality_two_releases_the_linear_gain() {
        let mut p = params();
        p.algorithm = 3;
        p.render_quality = 2;
        p.release_on = true;
        let mut run = Run::new(&p, 48_000.0, 1);
        run.gain_state[0] = 0.25;

        let recovering = run.update_gain(0, 1e-5);
        let target = run.gain(0);
        let expected = target + (0.25 - target) * run.release[0];
        assert!((recovering - expected).abs() < 1e-12);

        let reducing = run.update_gain(0, 1.0);
        assert_eq!(reducing, run.gain(0));
        assert!(reducing < recovering);
    }

    #[test]
    fn quadrature_envelope_is_phase_invariant_and_sums_tones() {
        let single = |frequency: f64, phase: f64| {
            let mut envelope = QuadratureEnvelope::default();
            let mut maximum = 0.0f64;
            for i in 0..48_000 {
                let angle = 2.0 * core::f64::consts::PI * frequency * i as f64 / 48_000.0 + phase;
                let value = envelope.push(0.5 * libm::sin(angle));
                if i >= 47_520 {
                    maximum = maximum.max(value);
                }
            }
            maximum
        };
        for frequency in [50.0, 400.0, 2500.0, 15_000.0] {
            let a = single(frequency, 0.0);
            let b = single(frequency, 1.3);
            assert!((a - b).abs() < 2e-3, "{frequency} Hz phase spread: {a} vs {b}");
            assert!((a - 0.5).abs() < 0.04, "{frequency} Hz magnitude: {a}");
        }

        for phase in [0.0, core::f64::consts::FRAC_PI_2] {
            let mut envelope = QuadratureEnvelope::default();
            let mut maximum = 0.0f64;
            for i in 0..48_000 {
                let t = i as f64 / 48_000.0;
                let input = 0.2 * libm::sin(2.0 * core::f64::consts::PI * 50.0 * t)
                    + 0.2 * libm::sin(2.0 * core::f64::consts::PI * 150.0 * t + phase);
                let value = envelope.push(input);
                if i >= 47_520 {
                    maximum = maximum.max(value);
                }
            }
            assert!((maximum - 0.4).abs() < 0.03, "two-tone phase {phase}: {maximum}");
        }
    }

    #[test]
    fn gain_src_has_zero_startup_and_unity_dc() {
        for factor in [1, 2, 4] {
            let mut src = GainSrc::new(factor, 48_000.0);
            let first = src.step(Some(1.0));
            assert!(first > 0.0 && first < 0.01, "factor {factor}: {first}");
            let mut output = first;
            for i in 1..512 {
                output = src.step((i % factor == 0).then_some(1.0));
            }
            assert!((output - 1.0).abs() < 2e-6, "factor {factor}: {output}");
            src.reset();
            assert_eq!(src.step(None), 0.0);
        }
    }

    #[test]
    fn polyphase_gain_src_matches_zero_stuffed_convolution_bits() {
        // Independent direct FIR oracle: include every inserted zero, wrap
        // the history with modular indexing, and preserve sequential sums.
        for fs in [8_000.0, 44_100.0, 48_000.0, 96_000.0, 192_000.0] {
            for factor in [1, 2, 4, 8] {
                let mut src = GainSrc::new(factor, fs);
                let mut history = std::vec![0.0f32; src.len];
                let mut cursor = 0;
                let mut random = 0x1234_5678u32;
                for frame in 0..4096 {
                    if frame == 2048 {
                        src.reset();
                        history.fill(0.0);
                        cursor = 0;
                    }
                    random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    let value = match frame % 11 {
                        0 => -0.0f32,
                        1 => f32::from_bits(1),
                        2 => -f32::from_bits(1),
                        _ => (random as i32) as f32 / i32::MAX as f32,
                    };
                    let input = (frame % factor == 0).then_some(value as f64);
                    history[cursor] = input.unwrap_or(0.0) as f32;
                    let mut expected = 0.0f32;
                    for k in 0..src.len {
                        expected += src.taps[k] * history[(cursor + src.len - k) % src.len];
                    }
                    assert_eq!(
                        src.step(input).to_bits(),
                        (expected as f64).to_bits(),
                        "fs {fs}, factor {factor}, frame {frame}"
                    );
                    cursor = (cursor + 1) % src.len;
                }
            }
        }
    }

    #[test]
    fn detector_and_gain_src_are_slice_invariant() {
        let p = params();
        let mut whole = tone(2500.0, 0.8, 4096);
        let mut split = whole.clone();
        let mut run = Run::new(&p, 48_000.0, 1);
        run.process(&mut [&mut whole], 4096);
        let mut run = Run::new(&p, 48_000.0, 1);
        let (a, rest) = split.split_at_mut(333);
        run.process(&mut [a], 333);
        let (b, rest) = rest.split_at_mut(1789);
        run.process(&mut [b], 1789);
        let rest_len = rest.len();
        run.process(&mut [rest], rest_len);
        assert_eq!(whole, split);
    }

    #[test]
    fn reset_clears_the_detector_and_overshoot_bank() {
        let mut run = Run::new(&params(), 48_000.0, 1);
        run.env = [0.25; BANDS];
        run.overshoot_state[0][0][0][0] = [0.25; 2];
        run.quadrature[0][0].arm_a.input_delay[0][0] = 0.25;
        run.audio_quadrature[0].arm_b.output_delay[0][1] = 0.25;
        run.reset();
        assert_eq!(run.audio_quadrature[0].arm_b.output_delay, [[0.0; 2]; QUADRATURE_STAGES]);
        assert_eq!(run.env, [0.0; BANDS]);
        assert_eq!(run.overshoot_state[0][0][0][0], [0.0; 2]);
        assert_eq!(run.quadrature[0][0].arm_a.input_delay, [[0.0; 2]; QUADRATURE_STAGES]);
    }

    #[test]
    fn audio_quadrature_is_a_unity_allpass() {
        for frequency in [30.0, 100.0, 1_000.0, 6_000.0, 15_000.0] {
            let mut pair = QuadratureEnvelope::default();
            let n = 96_000;
            let mut output = std::vec::Vec::with_capacity(n);
            for i in 0..n {
                let x = 0.5 * libm::sin(2.0 * core::f64::consts::PI * frequency * i as f64 / 48_000.0);
                output.push(pair.push_audio(x) as f32);
            }
            let got = level(&output[n / 2..], frequency);
            // 90 +- 0.7 degrees between the arms above 30 Hz: unity within 0.06 dB.
            assert!((got - 20.0 * libm::log10(0.5)).abs() < 0.07, "{frequency} Hz: {got} dB");
        }
    }

    #[test]
    fn mode4_gain_uses_the_50_hz_bilinear_arm_recurrence() {
        for sample_rate in [44_100.0, 48_000.0, 96_000.0] {
            let mut state = Mode4Gain::new(sample_rate);
            let k = libm::tan(core::f64::consts::PI * 50.0 / sample_rate);
            let first = state.step(1.0);
            assert_eq!(first.lowpass, (k / (1.0 + k)) as f32);
            for _ in 0..(sample_rate as usize) {
                state.step(1.0);
            }
            assert!((state.previous_output - 1.0).abs() < 2e-5);
        }
    }

    #[test]
    fn mode4_power_ratio_is_directional_and_complementary() {
        let reducing = {
            let ratio = mode4_ratio(0.25, 0.5);
            (1.0 - ratio * ratio).clamp(0.0, 1.0)
        };
        let recovering = {
            let ratio = mode4_ratio(0.5, 0.25);
            (1.0 - ratio * ratio).clamp(0.0, 1.0)
        };
        assert!((reducing - 0.75).abs() < 1e-5);
        assert_eq!(recovering, 0.0);
        assert_eq!(reducing + (1.0 - reducing), 1.0);
    }

    #[test]
    fn quality_two_audio_path_has_no_quadrature_allpass() {
        let mut p = params();
        p.render_quality = 2;
        let mut run = Run::new(&p, 48_000.0, 1);
        let mut x = [0.0f32; 64];
        x[0] = 0.01;
        run.process(&mut [&mut x[..]], 64);
        assert!(!run.hop_peak);
        assert_eq!(run.audio_quadrature[0].arm_b.input_delay, [[0.0; 2]; QUADRATURE_STAGES]);
    }

    #[test]
    fn spline_curve_softens_the_knee_range_and_expander() {
        let band = Band { down_threshold_db: -20.0, down_ratio: 4.0, down_range_db: -6.0, ..Band::default() };
        let legacy = curve_points(band, 0.0);
        let modern = curve_points(band, 20.0);
        assert!((spline_gain(&legacy, -32.0, 0.0) - 1.0).abs() < 1e-12);
        assert!(spline_gain(&modern, -32.0, 20.0) < 1.0);
        assert!(20.0 * libm::log10(spline_gain(&modern, 0.0, 20.0)) > -6.0);

        // Expansion only removes upward gain and stops at makeup.
        let expanded = curve_points(
            Band {
                up_threshold_db: -30.0,
                up_ratio: 2.0,
                up_range_db: 10.0,
                exp_threshold_db: -60.0,
                exp_ratio: 2.0,
                ..Band::default()
            },
            20.0,
        );
        assert!((expanded[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn every_saturation_flavor_is_odd_and_bounded_on_program_input() {
        for flavor in 0..=4 {
            for x in [-1.0, -0.25, 0.0, 0.25, 1.0] {
                let y = Run::saturate(Some(flavor), x);
                assert!(y.is_finite());
                assert!((y + Run::saturate(Some(flavor), -x)).abs() < 1e-12);
                assert!(y.abs() <= 1.1);
            }
        }
    }

    #[test]
    fn saturation_precedes_detector_gain() {
        assert_eq!(Run::band_output(Some(0), 0.25, 2.0), 0.25);
        assert_eq!(Run::band_output(None, 0.25, 2.0), 0.5);
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
        assert!(level(&right, 80.0) - before < 0.1);
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
