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
//!   `min(1, t / F)`; `s`
//!   drops to `u` at once and otherwise moves towards it with the release
//!   coefficient selected by the detector history: [`TAU_FAST_S`] while a
//!   five-dB upward detector excursion has fast-release budget left, then
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
//!
//! Output saturation (algorithm version 1, sine flavor), when switched on,
//! runs per channel ahead of the limiter's detector and delay line:
//!
//! * the signal is oversampled [`OVERSAMPLE`] times through a linear-phase
//!   windowed-sinc low pass `h` ([`FIR_TAPS`] taps at the oversampled rate,
//!   [`FIR_HALF_BASE`] base frames of delay, a 90 dB Dolph-Chebyshev
//!   window), cut at
//!   `min(19 kHz - 1 kHz * mode, fs / 2)` (rates up to 48 kHz) or `fs / 4`
//!   (88.2 kHz and up);
//! * at base rate a direct-form-I de-esser cut `D` (the saved frequency
//!   `f0`, strength `S` dB and Q:
//!   `D(s) = (s^2 + s w0/Q + w0^2) / (s^2 + s w0/(Q g) + w0^2)`,
//!   `g = 10^(-S/20)`, bilinear at `fs`) runs before the interpolator and
//!   its independently stateful exact inverse runs after the decimator;
//! * the drive `k` scales each cut base-rate sample before it enters the
//!   interpolation delay, the memoryless shaper runs at 2fs, and `1/k`
//!   scales the decimated sample before the inverse de-esser.  A new block's
//!   scalar is used from its first base-rate sample; FIR histories and both
//!   biquad states continue unchanged. `k` is between [`K_MIN`] (no
//!   high-band content) and 1, linked over the channels, set per block
//!   from the ratio of the block's RMS (all channels) through a fixed
//!   high-band weighting to its plain RMS
//!   (`k^2 = DRIVE_C * (ratio + DRIVE_RATIO_FLOOR)`), smoothed in dB (one
//!   pole, [`DRIVE_TAU_S`], from [`start_db`], silent blocks included); a
//!   block is a render call split at [`DRIVE_BLOCK`] frames, and each block
//!   is shaped with its own drive (no look-ahead, no added latency);
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
/// First slow-gain release time constant (towards its target, from below).
pub const TAU_FAST_S: f64 = 0.270;
/// Long slow-gain release time constant (towards its target, from below).
pub const TAU_SLOW_S: f64 = 10.0;
/// Output saturation oversampling factor.
pub const OVERSAMPLE: usize = 2;
/// Delay of each of the two saturation low passes, in base frames.
pub const FIR_HALF_BASE: usize = 12;
/// Taps of the saturation low pass at the oversampled rate.
pub const FIR_TAPS: usize = 2 * FIR_HALF_BASE * OVERSAMPLE + 1;
/// Length of the max-normalised 90 dB Chebyshev reference window.  The FIR
/// constructor resamples it to [`FIR_TAPS`] with a clamped cubic B-spline.
pub const CHEB_REFERENCE_TAPS: usize = 100;
pub const CHEB_ATTENUATION_DB: f64 = 90.0;
/// Saturation latency (up plus down low pass), base frames.
pub const SATURATION_DELAY: usize = 2 * FIR_HALF_BASE;
/// The cubic (flavour 1) shaper: `f(CUBIC_SCALE k u) / k` with
/// `f(v) = 1.5 v - 0.5 v^3` (clamped to +-1 past the knee), small-signal
/// slope `1.5 CUBIC_SCALE` (+0.10 dB). Black-box: base-rate (mode 0) ramps,
/// least squares on `[x, x^3]`, residual 1.5e-7 of full scale.
pub const CUBIC_SCALE: f64 = 0.674_484_5;
/// A drive ceiling no block peak reaches: the ceiling switched off.
pub const CEILING_OFF_DB: f64 = 100.0;
/// Shaper drive bounds: `k` in `[K_MIN, 1]`; the low bound is -5 dB
/// (black-box: memoryless mode-0 fits of 100 Hz and 1 kHz ramps settle at
/// `k = 0.562342`, flavor 1 shows the same ratio).
pub const K_MIN: f64 = 0.562_341_325_190_349; // 10^(-1/4)
/// The drive is set once per block: each render call, split into pieces
/// of at most this many frames from its first frame (black-box: renders
/// in 1024-frame calls are bit-identical to 512-frame ones; 128/256/333/
/// 480-frame calls follow this rule, each block's drive from that block,
/// moved by its own length's share of [`DRIVE_TAU_S`]). The drive of a
/// block is set from that block's own input before the block is shaped,
/// so no look-ahead and no added latency are needed for any call size.
pub const DRIVE_BLOCK: usize = 512;
/// The drive (dB of `k`) follows each block's target with a one-pole of
/// this time constant: a block of `n` frames moves it
/// `1 - exp(-n / (DRIVE_TAU_S fs))` of the way (0.1012 for 512 frames at
/// 48 kHz). Black-box: steps between steady drives decay by exactly that
/// factor per block, and fresh tone onsets back-solve to one constant
/// start state, [`start_db`].
pub const DRIVE_TAU_S: f64 = 0.1;
/// Each block's target (dB of `k`) is
/// `10 log10(DRIVE_C (rms(weighted) / rms(input) + DRIVE_RATIO_FLOOR))`,
/// summed over the channels (the drive is linked), the RMS ratio taken as
/// zero for a silent block. Black-box (tone onsets 20 Hz-2.5 kHz, steps
/// between steady drives, silent gaps): this constant, the floor and the
/// weighting below reproduce every block's drive within 1e-4 dB.
pub const DRIVE_C: f64 = 31.622_776_601_683_793; // 10^1.5
pub const DRIVE_RATIO_FLOOR: f64 = 1e-6;
/// The weighting: a fourth-order Butterworth high pass at
/// [`DRIVE_HP_HZ`] and a second-order Butterworth low pass at
/// [`DRIVE_LP_HZ`], each section an RBJ (bilinear, prewarped) biquad.
pub const DRIVE_HP_HZ: f64 = 7_000.0;
pub const DRIVE_LP_HZ: f64 = 13_000.0;
/// Section Qs of a fourth-order Butterworth (`1 / (2 sin((2k + 1) pi / 8))`)
/// and of a second-order one.
const DRIVE_HP_Q: [f64; 2] = [0.541_196_100_146_197, 1.306_562_964_876_376_5];
const DRIVE_LP_Q: f64 = core::f64::consts::FRAC_1_SQRT_2;
/// Detector movement that adds one fast-release interval.
const RELEASE_STEP_DB: f64 = 5.0;
/// The release selector runs after a 4-by-10 control-rate reduction. Each
/// step adds ten control frames at 48 kHz (and the nearest corresponding
/// count at other rates).
const RELEASE_DECIMATE_1: usize = 4;
const RELEASE_DECIMATE_2: usize = 10;
const RELEASE_CONTROL_DECIMATION: usize = RELEASE_DECIMATE_1 * RELEASE_DECIMATE_2;
const RELEASE_ADD_RATE_HZ: f64 = 5_000.0;
const RELEASE_POWER_WINDOW_S: f64 = 0.030;
const RELEASE_POWER_FLOOR: f64 = 1.0e-10;
const RELEASE_FIR_TAPS: usize = 31;

/// A clean 31-tap least-squares low pass for the first 4:1 reduction.
///
/// Generated with SciPy's BSD-licensed `signal.firls`: pass band 0..0.0755,
/// stop band 0.125..0.5 (cycles/sample), stop weight 1500. The recipe is
/// deliberately public and reproducible; no installed-unit coefficient table
/// is carried here.
const RELEASE_FIR: [f32; RELEASE_FIR_TAPS] = [
    1.498_691_4e-4,
    -7.314_614_5e-4,
    -3.267_145_2e-3,
    -8.002_404e-3,
    -1.475_691_9e-2,
    -2.222_959_1e-2,
    -2.791_541e-2,
    -2.848_869_4e-2,
    -2.064_968_6e-2,
    -2.237_285_7e-3,
    2.674_28e-2,
    6.356_486e-2,
    1.029_868_1e-1,
    1.382_596_3e-1,
    1.627_121_9e-1,
    1.714_600_2e-1,
    1.627_121_9e-1,
    1.382_596_3e-1,
    1.029_868_1e-1,
    6.356_486e-2,
    2.674_28e-2,
    -2.237_285_7e-3,
    -2.064_968_6e-2,
    -2.848_869_4e-2,
    -2.791_541e-2,
    -2.222_959_1e-2,
    -1.475_691_9e-2,
    -8.002_404e-3,
    -3.267_145_2e-3,
    -7.314_614_5e-4,
    1.498_691_4e-4,
];

/// Public Favreau/Pure Data fourth-order quadrature pair, with the measured
/// Float32 numerator normalisation used by the control reducer. Each row is
/// `[b0, b1, b2, a1, a2]` for transposed direct-form II.
const RELEASE_HILBERT: [[[f32; 5]; 2]; 2] = [
    [[0.94657, -1.94630, 1.0, -1.94632, 0.94657], [0.06338, -0.83774, 1.0, -0.83774, 0.06338]],
    [[-0.260502, 0.02569, 1.0, 0.02569, -0.0260502], [1.1058, -2.373, 1.27, -1.8685, 0.870686]],
];

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

#[derive(Clone, Copy, Default)]
struct ReleaseBiquad {
    z: [f32; 2],
}

impl ReleaseBiquad {
    fn run(&mut self, c: &[f32; 5], x: f32) -> f32 {
        let y = x * c[0] + self.z[0];
        self.z[0] = x * c[1] - y * c[3] + self.z[1];
        self.z[1] = x * c[2] - y * c[4];
        y
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    pub lookahead_s: f64,
    /// The unit's "fast gain release time": a hold, in seconds.
    pub hold_s: f64,
    pub threshold_db: f64,
    pub min_fast_gain_db: f64,
    pub true_peak: bool,
    /// Output saturation oversampling mode (0: none, the shaper runs at the
    /// base rate; 1-7: oversampled).
    pub saturation_mode: u32,
    /// Output saturation flavour: 0 off, 1 the cubic shaper, 2 the sine
    /// shaper.
    pub saturation_flavor: u32,
    /// The block-peak ceiling of the shaper drive, dB: a 512-frame block's
    /// drive is scaled by `min(1, 10^(ceiling/20) / block peak)` (the macOS
    /// 27 unit's "max saturation"; the 25G83 unit has no such ceiling, which
    /// the compiler writes as [`CEILING_OFF_DB`]).
    pub saturation_ceiling_db: f64,
    pub input_gain_db: f64,
    pub output_gain_db: f64,
    /// De-esser centre frequency, strength (dB of cut) and Q; used only
    /// with output saturation.
    pub deesser_hz: f64,
    pub deesser_db: f64,
    pub deesser_q: f64,
    /// Butterworth high-pass order (0: off), corner and location (0: at
    /// the input, ahead of the detector and the look-ahead delay).
    pub highpass_order: u32,
    pub highpass_hz: f64,
    pub highpass_location: u32,
}

impl Params {
    /// All words: the plain ten, the high-pass three, the de-esser three.
    pub const WORDS: usize = 16;
    /// Without the high-pass and the de-esser.
    pub const PLAIN_WORDS: usize = 10;
    pub const LEGACY_WORDS: usize = 8;
    /// With the high-pass: the plain words plus order, corner and location.
    pub const HIGHPASS_WORDS: usize = 13;

    /// The number of words this state needs.
    pub fn words(&self) -> usize {
        if self.saturation_flavor != 0 {
            Self::WORDS
        } else if self.highpass_order != 0 {
            Self::HIGHPASS_WORDS
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
            self.saturation_ceiling_db,
            self.input_gain_db,
            self.output_gain_db,
            self.highpass_order as f64,
            self.highpass_hz,
            self.highpass_location as f64,
            self.deesser_hz,
            self.deesser_db,
            self.deesser_q,
        ]);
    }

    pub fn from_words(w: &[f64]) -> Option<Params> {
        if ![Self::LEGACY_WORDS, Self::PLAIN_WORDS, Self::HIGHPASS_WORDS, Self::WORDS].contains(&w.len())
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
                m if m == libm::trunc(m) && (0.0..=7.0).contains(&m) => m as u32,
                _ => return None,
            },
            saturation_flavor: match w[6] {
                0.0 => 0,
                1.0 => 1,
                2.0 => 2,
                _ => return None,
            },
            saturation_ceiling_db: w[7],
            input_gain_db: w.get(8).copied().unwrap_or(0.0),
            output_gain_db: w.get(9).copied().unwrap_or(0.0),
            highpass_order: w.get(10).copied().unwrap_or(0.0) as u32,
            highpass_hz: w.get(11).copied().unwrap_or(0.0),
            highpass_location: w.get(12).copied().unwrap_or(0.0) as u32,
            deesser_hz: w.get(13).copied().unwrap_or(0.0),
            deesser_db: w.get(14).copied().unwrap_or(0.0),
            deesser_q: w.get(15).copied().unwrap_or(0.0),
        };
        if w.len() >= Self::HIGHPASS_WORDS {
            // The 13-word form always carries a high-pass; the full form
            // may leave it off (order 0, corner 0).
            let order = w[10];
            let off = w.len() == Self::WORDS && order == 0.0 && w[11] == 0.0;
            let whole = libm::trunc(order) == order && (1.0..=MAX_HIGHPASS_ORDER as f64).contains(&order);
            if !(off || (whole && p.highpass_hz > 0.0 && p.highpass_hz < 20_000.0)) || w[12] != 0.0 {
                return None;
            }
        }
        let ok = (-120.0..=40.0).contains(&p.input_gain_db)
            && (-120.0..=40.0).contains(&p.output_gain_db)
            && (0.0..=MAX_LOOKAHEAD_S).contains(&p.lookahead_s)
            && (0.0..=MAX_HOLD_S).contains(&p.hold_s)
            && (-40.0..=6.0).contains(&p.threshold_db)
            && (0.0..=40.0).contains(&p.min_fast_gain_db);
        let saturation_ok = if p.saturation_flavor == 0 {
            p.saturation_mode == 0
                && p.saturation_ceiling_db == 0.0
                && p.deesser_hz == 0.0
                && p.deesser_db == 0.0
                && p.deesser_q == 0.0
        } else {
            // A saturating state must carry its de-esser (the 10-word form
            // of earlier artifacts does not).
            w.len() == Self::WORDS
                && (-100.0..=100.0).contains(&p.saturation_ceiling_db)
                // the base-rate (mode 0) de-esser is not established
                && (p.saturation_mode != 0 || p.deesser_db == 0.0)
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

/// The high-pass sections share the biquad form.
type SatSection = Biquad;
type SatState = BiquadState;

/// One single-precision direct-form-I section.  The four logical delay
/// samples are explicit so a drive-block boundary cannot accidentally map or
/// scale filter state.
#[derive(Clone, Copy, Default)]
struct Df1 {
    b: [f32; 3],
    a: [f32; 2],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Df1State {
    x: [f32; 2],
    y: [f32; 2],
}

impl Df1 {
    fn new(b: [f32; 3], a: [f32; 3]) -> Df1 {
        Df1 { b: [b[0] / a[0], b[1] / a[0], b[2] / a[0]], a: [a[1] / a[0], a[2] / a[0]] }
    }

    fn run(&self, st: &mut Df1State, x: f64) -> f64 {
        let x = x as f32;
        let y = self.b[0] * x + self.b[1] * st.x[0] + self.b[2] * st.x[1] - self.a[0] * st.y[0] - self.a[1] * st.y[1];
        st.x[1] = st.x[0];
        st.x[0] = x;
        st.y[1] = st.y[0];
        st.y[0] = y;
        f64::from(y)
    }
}

/// The base-rate de-esser cut and its independently stateful inverse.  The
/// coefficient construction is single precision, like the filtering path.
fn deesser(f0: f64, strength_db: f64, q: f64, fs: f64) -> (Df1, Df1) {
    let (f0, strength_db, q, fs) = (f0 as f32, strength_db as f32, q as f32, fs as f32);
    let g = libm::powf(10.0, -strength_db / 20.0);
    let w0 = 2.0 * core::f32::consts::PI * f0.min(0.49 * fs) / fs;
    let (s, c) = (libm::sinf(w0), libm::cosf(w0));
    let al = s / (2.0 * q);
    let num = [1.0 + al, -2.0 * c, 1.0 - al];
    let den = [1.0 + al / g, -2.0 * c, 1.0 - al / g];
    (Df1::new(num, den), Df1::new(den, num))
}

/// A symmetric, max-normalised 100-point, 90 dB Dolph-Chebyshev window.
///
/// This is the public analytic construction used by SciPy's BSD-licensed
/// `scipy.signal.windows.chebwin`, written as a direct DFT because the fixed
/// reference is small and is built only when the box is created.
fn chebyshev_reference() -> [f32; CHEB_REFERENCE_TAPS] {
    let m = CHEB_REFERENCE_TAPS;
    let order = (m - 1) as f64;
    let attenuation = libm::pow(10.0, CHEB_ATTENUATION_DB / 20.0);
    let beta = libm::cosh(libm::acosh(attenuation) / order);
    let mut p = [0.0; CHEB_REFERENCE_TAPS];
    for (k, v) in p.iter_mut().enumerate() {
        let x = beta * libm::cos(core::f64::consts::PI * k as f64 / m as f64);
        *v = if x > 1.0 {
            libm::cosh(order * libm::acosh(x))
        } else if x < -1.0 {
            -libm::cosh(order * libm::acosh(-x))
        } else {
            libm::cos(order * libm::acos(x.clamp(-1.0, 1.0)))
        };
    }

    // For even M SciPy phase-shifts p by exp(i*pi*k/M), takes its FFT,
    // then mirrors bins 1..=M/2.  Only those real bins are needed here.
    let mut bins = [0.0; CHEB_REFERENCE_TAPS / 2 + 1];
    for (j, v) in bins.iter_mut().enumerate().skip(1) {
        for (k, &pk) in p.iter().enumerate() {
            let phase = core::f64::consts::PI * k as f64 * (1.0 - 2.0 * j as f64) / m as f64;
            *v += pk * libm::cos(phase);
        }
    }
    let peak = bins[1..].iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let mut out = [0.0; CHEB_REFERENCE_TAPS];
    for (i, v) in out.iter_mut().enumerate() {
        let j = if i < m / 2 { m / 2 - i } else { i + 1 - m / 2 };
        *v = (bins[j] / peak) as f32;
    }
    out
}

fn cubic_bspline(x: f32) -> f32 {
    let a = libm::fabsf(x);
    if a < 1.0 {
        2.0 / 3.0 - x * x + 0.5 * a * a * a
    } else if a < 2.0 {
        let d = 2.0 - a;
        d * d * d / 6.0
    } else {
        0.0
    }
}

/// Resample the reference window to the FIR length with clamped cubic
/// B-spline control points.  Coordinates, weights and output are f32.
fn saturation_window() -> [f32; FIR_TAPS] {
    let reference = chebyshev_reference();
    let mut window = [0.0; FIR_TAPS];
    for (j, w) in window.iter_mut().enumerate() {
        let u = (CHEB_REFERENCE_TAPS - 1) as f32 * j as f32 / (FIR_TAPS - 1) as f32;
        let base = libm::floorf(u) as i32;
        let frac = u - base as f32;
        for offset in -1..=2 {
            let at = (base + offset).clamp(0, (CHEB_REFERENCE_TAPS - 1) as i32) as usize;
            *w += reference[at] * cubic_bspline(frac - offset as f32);
        }
    }
    window
}

/// The saturation low pass at the oversampled rate, unity DC gain.  Every
/// prototype value is rounded to f32 before the double-precision sum, then
/// the f32 normalisation is applied to the f32 value.
fn saturation_fir(mode: u32, fs: f64) -> [f32; FIR_TAPS] {
    let fc = if fs >= 88_200.0 { fs / 4.0 } else { (19_000.0 - 1_000.0 * f64::from(mode)).min(fs / 2.0) };
    let ratio = (2.0 * fc / fs).clamp(0.5, 1.0);
    let a = ratio / OVERSAMPLE as f64;
    let window = saturation_window();
    let mut q = [0.0; FIR_TAPS];
    let mut sum = 0.0;
    for (j, v) in q.iter_mut().enumerate() {
        let z = a * (j as f64 - (FIR_TAPS / 2) as f64);
        let sinc =
            if libm::fabs(z) < 1e-6 { 1.0 } else { libm::sin(core::f64::consts::PI * z) / (core::f64::consts::PI * z) };
        *v = (f64::from(window[j]) * sinc) as f32;
        sum += f64::from(*v);
    }
    let scale = (1.0 / sum) as f32;
    q.map(|v| v * scale)
}

/// Per-channel output saturation state.
#[derive(Clone)]
struct Saturator {
    /// Base-rate input history (newest at `x_at`).
    x: [f32; PHASE_TAPS],
    x_at: usize,
    /// Oversampled shaper output history (newest at `v_at`).
    v: [f32; V_RING],
    v_at: usize,
    pre: Df1State,
    post: Df1State,
    det: [BiquadState; 3],
}

const PHASE_TAPS: usize = FIR_TAPS.div_ceil(OVERSAMPLE);
/// The decimator reads `v[2n - i]`, `i < FIR_TAPS`, after `v[2n + 1]`.
const V_RING: usize = FIR_TAPS + OVERSAMPLE - 1;

impl Saturator {
    fn new() -> Saturator {
        Saturator {
            x: [0.0; PHASE_TAPS],
            x_at: 0,
            v: [0.0; V_RING],
            v_at: 0,
            pre: Df1State::default(),
            post: Df1State::default(),
            det: [BiquadState::default(); 3],
        }
    }
}

/// The drive (dB) a fresh or reset unit starts from: the target of a unit
/// ratio (black-box: onsets after silence and at the start back-solve to
/// it; the first blocks after a start are not special).
fn start_db() -> f64 {
    10.0 * libm::log10(DRIVE_C)
}

/// Shared (per box) saturation coefficients and the linked drive.
struct Saturation {
    /// Linear drive ceiling for a block's peak.
    ceiling: f64,
    flavor: u32,
    oversampled: bool,
    h: [f32; FIR_TAPS],
    pre: Df1,
    post: Df1,
    det: [Biquad; 3],
    /// Frames measured in the current drive block.
    pos: usize,
    /// Frames output since the start or a reset; with oversampling the
    /// first [`SATURATION_DELAY`] are silent (black-box: the unit's output
    /// is exactly zero over its latency, no low-pass pre-ringing).
    started: usize,
    /// The block's energies, summed over the channels, of the input and of
    /// the weighted input, and its largest input sample.
    ex: f64,
    eg: f64,
    peak: f64,
    /// Drive time constant, frames.
    tau_frames: f64,
    /// Smoothed drive (dB) and the drive in use (ceiling scale included).
    drive_db: f64,
    k: f64,
    /// Drive scalars in the same streaming coordinate as the delayed FIR
    /// output.  The reciprocal applied to a returned sample belongs to the
    /// base-rate carrier sample that entered the FIR pair 24 frames earlier,
    /// not merely to the preceding render piece.
    makeup: [f64; SATURATION_DELAY],
    makeup_at: usize,
    ch: Vec<Saturator>,
}

impl Saturation {
    fn new(p: &Params, fs: f64, channels: usize) -> Saturation {
        let (pre, post) = deesser(p.deesser_hz, p.deesser_db, p.deesser_q, fs);
        let hp = |q| Biquad::rbj(true, DRIVE_HP_HZ.min(0.45 * fs), q, fs);
        let lp = Biquad::rbj(false, DRIVE_LP_HZ.min(0.45 * fs), DRIVE_LP_Q, fs);
        Saturation {
            ceiling: db(p.saturation_ceiling_db),
            flavor: p.saturation_flavor,
            oversampled: p.saturation_mode != 0,
            h: saturation_fir(p.saturation_mode, fs),
            pre,
            post,
            det: [hp(DRIVE_HP_Q[0]), hp(DRIVE_HP_Q[1]), lp],
            pos: 0,
            started: 0,
            ex: 0.0,
            eg: 0.0,
            peak: 0.0,
            tau_frames: DRIVE_TAU_S * fs,
            drive_db: start_db(),
            k: 1.0,
            makeup: [1.0; SATURATION_DELAY],
            makeup_at: 0,
            ch: vec![Saturator::new(); channels],
        }
    }

    fn reset(&mut self) {
        self.pos = 0;
        self.started = 0;
        self.ex = 0.0;
        self.eg = 0.0;
        self.peak = 0.0;
        self.drive_db = start_db();
        self.k = 1.0;
        self.makeup.fill(1.0);
        self.makeup_at = 0;
        self.ch.iter_mut().for_each(|c| *c = Saturator::new());
    }

    /// Adds one frame of one channel to the drive block's measurements.
    fn measure(&mut self, c: usize, x: f64) {
        let s = &mut self.ch[c];
        let mut g = x;
        for (bq, st) in self.det.iter().zip(s.det.iter_mut()) {
            g = bq.run(st, g);
        }
        self.ex += x * x;
        self.eg += g * g;
        self.peak = self.peak.max(libm::fabs(x));
    }

    /// Ends a drive block of `self.pos` frames (measured, not yet shaped):
    /// the linked drive moves towards the block's target, and the block's
    /// peak sets the drive ceiling's scale (`min(1, ceiling / peak)`, the
    /// macOS 27 unit's "max saturation"; off on 25G83).
    fn end_block(&mut self) {
        let ratio = if self.ex > 0.0 { libm::sqrt(self.eg / self.ex) } else { 0.0 };
        let target = 10.0 * libm::log10(DRIVE_C * (ratio + DRIVE_RATIO_FLOOR));
        let alpha = 1.0 - libm::exp(-(self.pos as f64) / self.tau_frames);
        self.drive_db += alpha * (target - self.drive_db);
        let scale = if self.peak > self.ceiling { self.ceiling / self.peak } else { 1.0 };
        self.k = libm::pow(10.0, self.drive_db / 20.0).clamp(K_MIN, 1.0) * scale;
        self.ex = 0.0;
        self.eg = 0.0;
        self.peak = 0.0;
        self.pos = 0;
    }

    /// The added delay, base frames.
    fn delay(&self) -> usize {
        if self.oversampled {
            SATURATION_DELAY
        } else {
            0
        }
    }

    /// Associates the base-rate reciprocal with the FIR pair's returned
    /// output buffer.  Keeping this as a streaming history, rather than a
    /// fixed prefix of each render piece, also covers pieces shorter than the
    /// FIR latency and arbitrary host render-call boundaries.
    fn delayed_makeup(&mut self, k: f64) -> f64 {
        let makeup = self.makeup[self.makeup_at];
        self.makeup[self.makeup_at] = k;
        self.makeup_at = (self.makeup_at + 1) % SATURATION_DELAY;
        makeup
    }

    /// The shaper on an already driven value `v` (`k` times the signal).
    fn shape_driven(flavor: u32, v: f64) -> f64 {
        if flavor == 1 {
            let u = CUBIC_SCALE * v;
            if u > 1.0 {
                1.0
            } else if u < -1.0 {
                -1.0
            } else {
                1.5 * u - 0.5 * u * u * u
            }
        } else {
            libm::sin(v)
        }
    }

    /// The native Float32 shaper between the Float32 polyphase FIRs.
    fn shape_driven_f32(flavor: u32, v: f32) -> f32 {
        if flavor == 1 {
            let u = CUBIC_SCALE as f32 * v;
            if u > 1.0 {
                1.0
            } else if u < -1.0 {
                -1.0
            } else {
                1.5 * u - 0.5 * u * u * u
            }
        } else {
            libm::sinf(v)
        }
    }

    /// The memoryless shaper with its make-up, `f(k d) / k`.
    fn shape(flavor: u32, k: f64, d: f64) -> f64 {
        Self::shape_driven(flavor, k * d) / k
    }

    /// One base-rate sample of channel `c`.  With oversampling, the cut runs
    /// first at base rate, `k` scales the value entering the interpolation
    /// delay, the shaper runs at 2fs, and `1/k` precedes the base-rate inverse.
    /// At the base rate (mode 0), the sample is shaped directly.
    fn run(&mut self, c: usize, x: f64, k: f64, makeup: f64) -> f64 {
        let h = &self.h;
        let flavor = self.flavor;
        if !self.oversampled {
            return Self::shape(flavor, k, x);
        }
        let s = &mut self.ch[c];
        let cut = self.pre.run(&mut s.pre, x);
        let driven = cut as f32 * k as f32;
        s.x_at = (s.x_at + 1) % PHASE_TAPS;
        s.x[s.x_at] = driven;
        for phase in 0..OVERSAMPLE {
            // u[2n + phase] = sum_j (2 h[phase + 2 j]) (k d[n - j])
            let mut u = 0.0f32;
            let mut i = phase;
            let mut j = 0;
            while i < FIR_TAPS {
                let up = h[i] * OVERSAMPLE as f32;
                u += up * s.x[(s.x_at + PHASE_TAPS - j) % PHASE_TAPS];
                i += OVERSAMPLE;
                j += 1;
            }
            s.v_at = (s.v_at + 1) % V_RING;
            s.v[s.v_at] = Self::shape_driven_f32(flavor, u);
        }
        // y[n] = sum_i h[i] v[2n - i]; v[2n] was pushed OVERSAMPLE - 1
        // samples before the newest.
        let newest_2n = (s.v_at + V_RING - (OVERSAMPLE - 1)) % V_RING;
        let mut y = 0.0f32;
        for (i, hv) in h.iter().enumerate() {
            y += *hv * s.v[(newest_2n + V_RING - i) % V_RING];
        }
        let made_up = f64::from(y * (1.0f32 / makeup as f32));
        self.post.run(&mut s.post, made_up)
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
    a_fast: f32,
    a_slow: f32,
    release_up: f32,
    release_down: f32,
    release_anchor: f32,
    fast_release_left: usize,
    fast_release_add: usize,
    fast_release: bool,
    release_became_slow: bool,
    fast_lookahead_pole: f32,
    release_decimate_1_at: usize,
    release_decimate_2_at: usize,
    release_fir_state: [f32; RELEASE_FIR_TAPS],
    release_fir_at: usize,
    release_hilbert: [[ReleaseBiquad; 2]; 2],
    release_power: Box<[f32]>,
    release_power_at: usize,
    release_power_sum: f64,
    /// Window weights, `w[j]` applies to the held target `j` frames ago.
    weight: Box<[f64]>,
    /// Ring of the last `lookahead` held targets.
    held: Box<[f64]>,
    held_at: usize,
    slow_target: MinHold,
    target: MinHold,
    slow: f32,
    /// Per channel: the last `lookahead` input samples (ring) and the last
    /// `TAPS` for the interpolator (ring).
    line: Box<[f32]>,
    line_at: usize,
    hist: Box<[f64]>,
    hist_at: usize,
    input_gain: f64,
    output_gain: f64,
    /// Per channel: the current drive block after the input gain and
    /// high-pass.
    piece: Box<[f64]>,
    saturation: Option<Box<Saturation>>,
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
        let fs_f32 = fs as f32;
        let coef = |tau: f32| 1.0 - libm::expf(-1.0 / (tau * fs_f32));
        let a_fast = coef(TAU_FAST_S as f32);
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
            a_fast,
            a_slow: coef(TAU_SLOW_S as f32),
            release_up: db(RELEASE_STEP_DB) as f32,
            release_down: db(-RELEASE_STEP_DB) as f32,
            release_anchor: 0.0,
            fast_release_left: 0,
            fast_release_add: libm::ceil(fs / RELEASE_ADD_RATE_HZ) as usize,
            fast_release: true,
            release_became_slow: false,
            fast_lookahead_pole: libm::powf(1.0 - a_fast, lookahead as f32),
            release_decimate_1_at: 0,
            release_decimate_2_at: 0,
            release_fir_state: [0.0; RELEASE_FIR_TAPS],
            release_fir_at: 0,
            release_hilbert: [[ReleaseBiquad::default(); 2]; 2],
            release_power: vec![
                0.0;
                libm::round(fs / RELEASE_CONTROL_DECIMATION as f64 * RELEASE_POWER_WINDOW_S).max(1.0)
                    as usize
            ]
            .into_boxed_slice(),
            release_power_at: 0,
            release_power_sum: 0.0,
            weight,
            held: vec![1.0; lookahead].into_boxed_slice(),
            held_at: 0,
            slow_target: MinHold::new(1),
            target: MinHold::new(window),
            slow: 1.0,
            line: vec![0.0; lookahead * channels].into_boxed_slice(),
            line_at: 0,
            hist: vec![0.0; TAPS * channels].into_boxed_slice(),
            hist_at: 0,
            input_gain: db(p.input_gain_db),
            output_gain: db(p.output_gain_db),
            piece: vec![0.0; DRIVE_BLOCK * channels].into_boxed_slice(),
            saturation: (p.saturation_flavor != 0).then(|| Box::new(Saturation::new(p, fs, channels))),
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
        self.release_anchor = 0.0;
        self.fast_release_left = 0;
        self.fast_release = true;
        self.release_became_slow = false;
        self.release_decimate_1_at = 0;
        self.release_decimate_2_at = 0;
        self.release_fir_state = [0.0; RELEASE_FIR_TAPS];
        self.release_fir_at = 0;
        self.release_hilbert = [[ReleaseBiquad::default(); 2]; 2];
        self.release_power.fill(0.0);
        self.release_power_at = 0;
        self.release_power_sum = 0.0;
        self.line.iter_mut().for_each(|v| *v = 0.0);
        self.line_at = 0;
        self.hist.iter_mut().for_each(|v| *v = 0.0);
        self.hist_at = 0;
        if let Some(s) = &mut self.saturation {
            s.reset();
        }
        self.highpass_state = [[SatState::default(); HIGHPASS_SECTIONS]; MAX_CHANNELS];
    }

    pub fn latency(&self) -> usize {
        self.lookahead + self.saturation.as_ref().map_or(0, |s| s.delay())
    }

    /// Advances the linked release detector by one audio-rate sample.
    fn release_control(&mut self, linked: f32) {
        self.release_fir_at = (self.release_fir_at + 1) % RELEASE_FIR_TAPS;
        self.release_fir_state[self.release_fir_at] = linked;
        if self.release_decimate_1_at == 0 {
            let mut filtered = 0.0f32;
            for (i, tap) in RELEASE_FIR.iter().enumerate() {
                filtered +=
                    tap * self.release_fir_state[(self.release_fir_at + RELEASE_FIR_TAPS - i) % RELEASE_FIR_TAPS];
            }

            let mut quadrature = [filtered; 2];
            for ((value, states), coefficients) in
                quadrature.iter_mut().zip(self.release_hilbert.iter_mut()).zip(RELEASE_HILBERT.iter())
            {
                for (state, coefficient) in states.iter_mut().zip(coefficients) {
                    *value = state.run(coefficient, *value);
                }
            }
            let magnitude = libm::sqrtf(quadrature[0] * quadrature[0] + quadrature[1] * quadrature[1]);

            if self.release_decimate_2_at == 0 {
                let was_fast = self.fast_release;
                let value = magnitude * magnitude;
                self.release_power_sum -= f64::from(self.release_power[self.release_power_at]);
                self.release_power_sum += f64::from(value);
                self.release_power[self.release_power_at] = value;
                self.release_power_at = (self.release_power_at + 1) % self.release_power.len();
                let level = (self.release_power_sum / self.release_power.len() as f64 + RELEASE_POWER_FLOOR) as f32;
                if level > self.release_anchor * self.release_up {
                    self.release_anchor = level;
                    self.fast_release_left = self.fast_release_left.saturating_add(self.fast_release_add);
                } else if level < self.release_anchor * self.release_down {
                    self.release_anchor = level;
                }
                self.fast_release = self.fast_release_left > 0;
                self.fast_release_left = self.fast_release_left.saturating_sub(1);
                self.release_became_slow = was_fast && !self.fast_release;
            }
            self.release_decimate_2_at = (self.release_decimate_2_at + 1) % RELEASE_DECIMATE_2;
        }
        self.release_decimate_1_at = (self.release_decimate_1_at + 1) % RELEASE_DECIMATE_1;
    }

    /// The linked gain for this frame, given the newest input frame.
    fn gain(&mut self, peak: f64) -> f64 {
        let t = if peak > self.ceiling { self.ceiling / peak } else { 1.0 };
        let u = self.slow_target.push((t / self.floor).min(1.0)) as f32;
        if self.release_became_slow && u > 0.99 {
            // The native 10x/4x coefficient return is aligned to the start
            // of the look-ahead interval. Undo the fast-pole moves made over
            // that interval when the held coefficient changes to slow.
            self.slow = 1.0 - (1.0 - self.slow) / self.fast_lookahead_pole;
            self.release_became_slow = false;
        }
        if u < self.slow {
            self.slow = u;
        } else {
            let a = if self.fast_release { self.a_fast } else { self.a_slow };
            self.slow += a * (u - self.slow);
        }
        let h = self.target.push(f64::from(self.slow) * t.max(self.floor));
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
    ///
    /// The call is run in blocks of at most [`DRIVE_BLOCK`] frames from its
    /// first frame, as the unit renders a call: each block is measured
    /// first and shaped with its own drive.
    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let ch = self.channels.min(bufs.len());
        let frames = bufs[..ch].iter().fold(frames, |n, b| n.min(b.len()));
        let mut at = 0;
        while at < frames {
            let n = DRIVE_BLOCK.min(frames - at);
            self.process_block(bufs, ch, at, n);
            at += n;
        }
    }

    /// Frames `at..at + n` of the call: one drive block.
    fn process_block(&mut self, bufs: &mut [&mut [f32]], ch: usize, at: usize, n: usize) {
        for (c, b) in bufs[..ch].iter().enumerate() {
            let state = &mut self.highpass_state[c];
            for (i, v) in self.piece[c * DRIVE_BLOCK..c * DRIVE_BLOCK + n].iter_mut().enumerate() {
                let s = b[at + i];
                let mut x = if s.is_finite() { f64::from(s) } else { 0.0 };
                x *= self.input_gain;
                for (section, st) in self.highpass[..self.highpass_sections].iter().zip(state.iter_mut()) {
                    x = section.run(st, x);
                }
                *v = x;
            }
        }
        // This block is measured before it is shaped.  Its new scalar applies
        // from sample zero; the FIR delay itself retains prior driven samples.
        let mut k = 1.0;
        if let Some(s) = &mut self.saturation {
            for i in 0..n {
                for c in 0..ch {
                    s.measure(c, self.piece[c * DRIVE_BLOCK + i]);
                }
            }
            s.pos = n;
            s.end_block();
            k = s.k;
        }
        let l = self.lookahead;
        for i in 0..n {
            let makeup = if let Some(s) = &mut self.saturation {
                if s.oversampled {
                    s.delayed_makeup(k)
                } else {
                    k
                }
            } else {
                1.0
            };
            let mut peak = 0.0f64;
            let mut linked = 0.0f32;
            let mut frame = [0.0f64; MAX_CHANNELS];
            self.hist_at = (self.hist_at + 1) % TAPS;
            for (c, slot) in frame[..ch].iter_mut().enumerate() {
                let mut x = self.piece[c * DRIVE_BLOCK + i];
                if let Some(s) = &mut self.saturation {
                    x = s.run(c, x, k, makeup);
                    if s.started < s.delay() {
                        x = 0.0;
                    }
                }
                *slot = x;
                let hist = &mut self.hist[c * TAPS..(c + 1) * TAPS];
                hist[self.hist_at] = x;
                let mut a = libm::fabs(x);
                if self.true_peak {
                    let mut best = 0.0f64;
                    for phase in 0..4 {
                        let mut acc = 0.0;
                        for t in 0..TAPS {
                            let coef = if phase < 2 { BS1770[phase][t] } else { BS1770[3 - phase][TAPS - 1 - t] };
                            acc += coef * hist[(self.hist_at + TAPS - t) % TAPS];
                        }
                        best = best.max(libm::fabs(acc));
                    }
                    a = a.max(self.tp_scale * best);
                }
                peak = peak.max(a);
                linked += x as f32;
            }
            if let Some(s) = &mut self.saturation {
                s.started = s.started.saturating_add(1);
            }
            self.release_control(linked);
            let g = self.gain(peak);
            for (c, b) in bufs[..ch].iter_mut().enumerate() {
                let slot = c * l + self.line_at;
                let delayed = self.line[slot];
                self.line[slot] = frame[c] as f32;
                b[at + i] = (f64::from(delayed) * g * self.output_gain) as f32;
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
            saturation_ceiling_db: 0.0,
            deesser_hz: 0.0,
            deesser_db: 0.0,
            deesser_q: 0.0,
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
        let mut w = [0.0; Params::WORDS];
        p.to_words(&mut w);
        assert_eq!(Params::from_words(&w[..Params::HIGHPASS_WORDS]), Some(p));
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
    fn flavor_one_is_the_cubic_soft_clipper_in_the_drive_frame() {
        // small-signal slope 1.5 s (+0.10 dB), cubic term 0.5 s^3 k^2
        for k in [1.0, K_MIN] {
            let y = |x: f64| Saturation::shape(1, k, x);
            assert!((y(1e-4) / 1e-4 - 1.5 * CUBIC_SCALE).abs() < 1e-8);
            let x = 0.9;
            let want = 1.5 * CUBIC_SCALE * x - 0.5 * CUBIC_SCALE.powi(3) * k * k * x * x * x;
            assert!((y(x) - want).abs() < 1e-12);
            // odd, and flat past the knee
            assert_eq!(y(-x), -y(x));
            assert_eq!(Saturation::shape(1, k, 10.0), 1.0 / k);
        }
        assert!((Saturation::shape(2, 1.0, 0.5) - libm::sin(0.5)).abs() < 1e-15);
    }

    #[test]
    fn release_control_fir_is_symmetric() {
        for i in 0..RELEASE_FIR_TAPS {
            assert_eq!(RELEASE_FIR[i].to_bits(), RELEASE_FIR[RELEASE_FIR_TAPS - 1 - i].to_bits());
        }
    }

    #[test]
    fn release_control_cadence_and_reset_are_stable() {
        let p = params(0.0, false);
        let mut run = Run::new(&p, 48_000.0, 2);
        for _ in 0..512 {
            run.release_control(0.002);
        }
        assert_eq!(run.release_decimate_1_at, 0);
        assert_eq!(run.release_decimate_2_at, 8);
        run.reset();
        assert_eq!(run.release_decimate_1_at, 0);
        assert_eq!(run.release_decimate_2_at, 0);
        assert!(run.release_fir_state.iter().all(|value| *value == 0.0));
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
        p.saturation_ceiling_db = 0.3;
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
    fn chebyshev_window_and_fir_are_symmetric_and_normalised() {
        let reference = chebyshev_reference();
        assert_eq!(reference[CHEB_REFERENCE_TAPS / 2 - 1], 1.0);
        assert_eq!(reference[CHEB_REFERENCE_TAPS / 2], 1.0);
        for i in 0..CHEB_REFERENCE_TAPS {
            assert_eq!(reference[i], reference[CHEB_REFERENCE_TAPS - 1 - i]);
        }

        let h = saturation_fir(3, 48_000.0);
        assert_eq!((OVERSAMPLE, FIR_TAPS, SATURATION_DELAY), (2, 49, 24));
        let sum: f64 = h.iter().map(|&v| f64::from(v)).sum();
        assert!((sum - 1.0).abs() < 2e-7, "dc {sum}");
        for i in 0..FIR_TAPS {
            assert!((h[i] - h[FIR_TAPS - 1 - i]).abs() < 2e-7);
        }
    }

    #[test]
    fn a_new_block_drive_starts_at_its_first_base_sample_without_state_mapping() {
        let p = saturating(3);
        let mut sat = Saturation::new(&p, 48_000.0, 1);
        sat.run(0, 0.25, 0.7, 0.7);
        let old_at = sat.ch[0].x_at;
        let old_driven = sat.ch[0].x[old_at];
        let pre_before = sat.ch[0].pre;
        let mut expected_pre = pre_before;
        let cut = sat.pre.run(&mut expected_pre, -0.2);
        let expected_driven = cut as f32 * 0.9f32;

        sat.run(0, -0.2, 0.9, 0.7);

        assert_eq!(sat.ch[0].x[old_at], old_driven);
        assert_eq!(sat.ch[0].x[sat.ch[0].x_at], expected_driven);
        assert_eq!(sat.ch[0].pre, expected_pre);
    }

    #[test]
    fn makeup_follows_the_streaming_output_buffer_across_short_pieces() {
        let p = saturating(3);
        let mut sat = Saturation::new(&p, 48_000.0, 1);

        // Model three short host pieces whose total is exactly the FIR-pair
        // latency.  A per-piece prefix would restart at each boundary; the
        // streaming history must return the original scalar exactly once the
        // delayed carrier reaches the output buffer.
        for (frames, k) in [(7, 0.7), (5, 0.8), (12, 0.9)] {
            for _ in 0..frames {
                assert_eq!(sat.delayed_makeup(k), 1.0);
            }
        }
        assert_eq!(sat.delayed_makeup(0.6), 0.7);

        sat.reset();
        assert_eq!(sat.delayed_makeup(0.5), 1.0);
        assert_eq!(sat.makeup_at, 1);
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
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut x: Vec<f32> = (0..48_000)
            .map(|k| (0.5 * libm::sin(2.0 * core::f64::consts::PI * 5_000.0 * k as f64 / 48_000.0)) as f32)
            .collect();
        r.process(&mut [&mut x[..]], 48_000);
        assert_eq!(r.saturation.as_ref().map(|s| s.k), Some(1.0));
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
                saturation_ceiling_db: 0.0,
                deesser_hz: 0.0,
                deesser_db: 0.0,
                deesser_q: 0.0,
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
