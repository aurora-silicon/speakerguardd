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
//!
//! Output saturation (cubic flavor 1 or sine flavor 2), when switched on,
//! runs per channel ahead of the limiter's detector and delay line:
//!
//! * the signal is oversampled [`OVERSAMPLE`] times through a linear-phase
//!   windowed-sinc low pass `h` ([`FIR_TAPS`] taps at the oversampled rate,
//!   [`FIR_HALF_BASE`] base frames of delay, Kaiser window [`WINDOW_BETA`]
//!   over [`WINDOW_HALF_OS`] oversampled samples each side), cut at
//!   `min(19 kHz - 1 kHz * mode, fs / 2)` (rates up to 48 kHz) or `fs / 4`
//!   (88.2 kHz and up);
//! * on the oversampled signal a de-esser cut `D` (the saved frequency
//!   `f0`, strength `S` dB and Q:
//!   `D(s) = (s^2 + s w0/Q + w0^2) / (s^2 + s w0/(Q g) + w0^2)`,
//!   `g = 10^(-S/20)`, bilinear at the oversampled rate) runs before the
//!   shaper and its exact inverse after it, so it only shapes how hard each
//!   band is driven;
//! * the flavor-2 shaper is `sin(k u) / k`, unity slope at zero; flavor 1
//!   is the cubic law documented at [`CUBIC_SCALE`]; `k` is a
//!   drive between [`K_MIN`] (no high-band content) and 1, linked over the
//!   channels, set once per [`DRIVE_BLOCK`] frames from the ratio of the
//!   block's RMS (all channels) through a fixed high-band weighting to its
//!   plain RMS (`k^2 = DRIVE_C * (ratio + DRIVE_RATIO_FLOOR)`),
//!   smoothed in dB with [`DRIVE_TAU_S`]; an aligned
//!   512-frame host call is measured before it is shaped, so it uses its
//!   own block's drive;
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
/// Cubic saturation (flavor 1) is `f(CUBIC_SCALE * k * u) / k`, where
/// `f(v) = 1.5 v - 0.5 v^3` inside the knee and is flat at +/-1 beyond it.
/// Its small-signal slope is +0.10 dB for every drive.
pub const CUBIC_SCALE: f64 = 0.674_484_5;
/// A block-peak ceiling no admitted input can reach: ceiling disabled.
pub const CEILING_OFF_DB: f64 = 100.0;
/// Shaper drive bounds. The lower bound is exactly -5 dB.
pub const K_MIN: f64 = 0.562_341_325_190_349; // 10^(-1/4)
/// The drive is set once per this many frames.
pub const DRIVE_BLOCK: usize = 512;
/// Time constant of the per-block one-pole drive follower.
pub const DRIVE_TAU_S: f64 = 0.1;
/// `k^2 = DRIVE_C * (rms(weighted) / rms(input) + DRIVE_RATIO_FLOOR)`.
pub const DRIVE_C: f64 = 31.622_776_601_683_793; // 10^1.5
pub const DRIVE_RATIO_FLOOR: f64 = 1e-6;
/// Fourth-order Butterworth high pass followed by a second-order
/// Butterworth low pass for the spectral drive weighting.
pub const DRIVE_HP_HZ: f64 = 7_000.0;
pub const DRIVE_LP_HZ: f64 = 13_000.0;
const DRIVE_HP_Q: [f64; 2] = [0.541_196_100_146_197, 1.306_562_964_876_376_5];
const DRIVE_LP_Q: f64 = core::f64::consts::FRAC_1_SQRT_2;

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
    /// Output saturation oversampling mode (0: base-rate; 1-7:
    /// oversampled). Saturation is disabled when `saturation_flavor` is 0.
    pub saturation_mode: u32,
    /// 0: off; 1: cubic; 2: sine with legacy/dormant parameter 17;
    /// 4: sine with the source-selected block-peak ceiling.
    pub saturation_flavor: u32,
    /// Flavor 1's block-peak ceiling in dB (`CEILING_OFF_DB` disables it),
    /// or flavor 2's legacy saved parameter 17 divided by 100 (dormant).
    pub saturation_mix: f64,
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
            self.saturation_mix,
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
                4.0 => 4,
                _ => return None,
            },
            saturation_mix: w[7],
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
                && p.saturation_mix == 0.0
                && p.deesser_hz == 0.0
                && p.deesser_db == 0.0
                && p.deesser_q == 0.0
        } else {
            // A saturating state must carry its de-esser (the 10-word form
            // of earlier artifacts does not).
            let control_ok = match p.saturation_flavor {
                1 => (-100.0..=CEILING_OFF_DB).contains(&p.saturation_mix),
                2 => (-1.0..=1.0).contains(&p.saturation_mix),
                4 => (-100.0..=CEILING_OFF_DB).contains(&p.saturation_mix),
                _ => false,
            };
            w.len() == Self::WORDS
                && control_ok
                && (20.0..=20_000.0).contains(&p.deesser_hz)
                && (0.0..=40.0).contains(&p.deesser_db)
                && (0.05..=20.0).contains(&p.deesser_q)
                && (p.saturation_mode != 0 || p.deesser_db == 0.0)
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

/// Initial drive in dB: the target of a unit spectral ratio.
fn drive_start_db() -> f64 {
    10.0 * libm::log10(DRIVE_C)
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
    // 32 kHz: every mode cuts at fs/2 (measured); 44.1/48 kHz: per mode;
    // 88.2 kHz and up: fs/4 for every mode.
    let fc = if fs >= 88_000.0 { fs / 4.0 } else { (19_000.0 - 1_000.0 * f64::from(mode)).min(fs / 2.0) };
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
        }
    }
}

/// Shared (per box) saturation coefficients.
struct Saturation {
    flavor: u32,
    oversampled: bool,
    /// Whether parameter 17 is the macOS 27-era block-peak ceiling.
    peak_ceiling: Option<f64>,
    h: [f64; FIR_TAPS],
    pre: Biquad,
    post: Biquad,
    det: [Biquad; 3],
    pos: usize,
    /// The block's energies, summed over the channels, of the input and of
    /// the weighted input (the drive is linked).
    ex: f64,
    eg: f64,
    peak: f64,
    /// Per-block one-pole coefficient.
    alpha: f64,
    /// Smoothed drive (dB) and the drive in use.
    drive_db: f64,
    k: f64,
    peak_scale: f64,
    ch: Vec<Saturator>,
}

impl Saturation {
    fn new(p: &Params, fs: f64, channels: usize) -> Saturation {
        let fs_os = fs * OVERSAMPLE as f64;
        let (pre, post) = deesser(p.deesser_hz, p.deesser_db, p.deesser_q, fs_os);
        let hp = |q| Biquad::rbj(true, DRIVE_HP_HZ.min(0.45 * fs), q, fs);
        let lp = Biquad::rbj(false, DRIVE_LP_HZ.min(0.45 * fs), DRIVE_LP_Q, fs);
        Saturation {
            flavor: p.saturation_flavor,
            oversampled: p.saturation_mode != 0,
            peak_ceiling: matches!(p.saturation_flavor, 1 | 4).then(|| db(p.saturation_mix)),
            h: saturation_fir(p.saturation_mode, fs),
            pre,
            post,
            det: [hp(DRIVE_HP_Q[0]), hp(DRIVE_HP_Q[1]), lp],
            pos: 0,
            ex: 0.0,
            eg: 0.0,
            peak: 0.0,
            alpha: 1.0 - libm::exp(-(DRIVE_BLOCK as f64) / (DRIVE_TAU_S * fs)),
            drive_db: drive_start_db(),
            k: 1.0,
            peak_scale: 1.0,
            ch: vec![Saturator::new(); channels],
        }
    }

    fn reset(&mut self) {
        self.pos = 0;
        self.ex = 0.0;
        self.eg = 0.0;
        self.peak = 0.0;
        self.drive_db = drive_start_db();
        self.k = 1.0;
        self.peak_scale = 1.0;
        self.ch.iter_mut().for_each(|c| *c = Saturator::new());
    }

    /// Added latency in base-rate frames.
    fn delay(&self) -> usize {
        if self.oversampled {
            SATURATION_DELAY
        } else {
            0
        }
    }

    /// Add one already-filtered input sample to this linked drive block.
    fn measure(&mut self, c: usize, x: f64) {
        let s = &mut self.ch[c];
        let mut weighted = x;
        for (bq, st) in self.det.iter().zip(s.det.iter_mut()) {
            weighted = bq.run(st, weighted);
        }
        self.ex += x * x;
        self.eg += weighted * weighted;
        self.peak = self.peak.max(libm::fabs(x));
    }

    /// Ends a drive block: the linked drive moves towards this block's
    /// target before the centred portion of that block is shaped.
    fn end_block(&mut self) {
        let ratio = if self.ex > 0.0 { libm::sqrt(self.eg / self.ex) } else { 0.0 };
        let target = 10.0 * libm::log10(DRIVE_C * (ratio + DRIVE_RATIO_FLOOR));
        self.drive_db += self.alpha * (target - self.drive_db);
        self.k = libm::pow(10.0, self.drive_db / 20.0).clamp(K_MIN, 1.0);
        self.peak_scale =
            self.peak_ceiling.map_or(1.0, |ceiling| if self.peak > ceiling { ceiling / self.peak } else { 1.0 });
        self.ex = 0.0;
        self.eg = 0.0;
        self.peak = 0.0;
        self.pos = 0;
    }

    fn shape(flavor: u32, k: f64, x: f64) -> f64 {
        if flavor == 1 {
            let u = CUBIC_SCALE * k * x;
            let f = if u >= 1.0 {
                1.0
            } else if u <= -1.0 {
                -1.0
            } else {
                1.5 * u - 0.5 * u * u * u
            };
            f / k
        } else {
            libm::sin(k * x) / k
        }
    }

    /// One base-rate sample of channel `c`; returns the sample delayed by
    /// [`SATURATION_DELAY`].
    fn run(&mut self, c: usize, x: f64, k: f64) -> f64 {
        let h = &self.h;
        let flavor = self.flavor;
        let s = &mut self.ch[c];
        if !self.oversampled {
            return Self::shape(flavor, k, x);
        }
        s.x_at = (s.x_at + 1) % PHASE_TAPS;
        s.x[s.x_at] = x;
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
            let shaped = Self::shape(flavor, k, d);
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
    /// One call piece after input gain and the optional high-pass.
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
        let mut at = 0;
        while at < frames {
            let room = self.saturation.as_ref().map_or(DRIVE_BLOCK, |s| DRIVE_BLOCK - s.pos);
            let n = room.min(frames - at);
            self.process_piece(bufs, ch, at, n);
            at += n;
        }
    }

    /// Process one part of a call, ending no later than the current linked
    /// drive-block boundary. The input part is measured before it is shaped;
    /// an aligned 512-frame host call therefore uses that same block's drive.
    fn process_piece(&mut self, bufs: &mut [&mut [f32]], ch: usize, at: usize, n: usize) {
        for (c, b) in bufs[..ch].iter().enumerate() {
            let state = &mut self.highpass_state[c];
            let piece = &mut self.piece[c * DRIVE_BLOCK..c * DRIVE_BLOCK + n];
            for (i, slot) in piece.iter_mut().enumerate() {
                let sample = b[at + i];
                let mut x = if sample.is_finite() { f64::from(sample) } else { 0.0 };
                x *= self.input_gain;
                for (section, st) in self.highpass[..self.highpass_sections].iter().zip(state.iter_mut()) {
                    x = section.run(st, x);
                }
                *slot = x;
            }
        }
        let drive = if let Some(s) = &mut self.saturation {
            let block_at = s.pos;
            let oversampled = s.oversampled;
            let drive_before = s.k * s.peak_scale;
            for i in 0..n {
                for c in 0..ch {
                    s.measure(c, self.piece[c * DRIVE_BLOCK + i]);
                }
            }
            s.pos += n;
            if s.pos == DRIVE_BLOCK {
                s.end_block();
            }
            Some((drive_before, s.k * s.peak_scale, block_at, oversampled))
        } else {
            None
        };
        let l = self.lookahead;
        for i in 0..n {
            let mut peak = 0.0f64;
            let mut frame = [0.0f64; MAX_CHANNELS];
            self.hist_at = (self.hist_at + 1) % TAPS;
            for (c, slot) in frame[..ch].iter_mut().enumerate() {
                let mut x = self.piece[c * DRIVE_BLOCK + i];
                if let Some(s) = &mut self.saturation {
                    let (drive_before, drive_after, block_at, oversampled) = drive.expect("saturation drive");
                    // The oversampled shaper sees the new block drive once
                    // its interpolation filter is centred in the block.
                    let k = if oversampled && block_at + i < FIR_HALF_BASE { drive_before } else { drive_after };
                    x = s.run(c, x, k);
                }
                *slot = x;
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
            saturation_mix: 0.0,
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
        // sin(k u) / k with k at the -5 dB floor, amplitude 0.5: 2 J1(z) / z
        let z: f64 = K_MIN * 0.5;
        let want = 20.0 * libm::log10(1.0 - z * z / 8.0 + z * z * z * z / 192.0);
        assert!((lf - want).abs() < 0.002, "{lf} {want}");
        let mut s = Saturation::new(&p, 48_000.0, 1);
        for k in 0..48_000 {
            let x = 0.5 * libm::sin(2.0 * core::f64::consts::PI * 5_000.0 * k as f64 / 48_000.0);
            s.measure(0, x);
            s.pos += 1;
            if s.pos == DRIVE_BLOCK {
                s.end_block();
            }
            let drive = s.k * s.peak_scale;
            s.run(0, x, drive);
        }
        assert_eq!(s.k, 1.0);
    }

    #[test]
    fn flavor_one_is_the_cubic_soft_clipper_at_base_rate() {
        let mut p = saturating(1);
        p.saturation_mode = 0;
        p.saturation_flavor = 1;
        p.saturation_mix = 0.0;
        p.deesser_db = 0.0;
        let mut words = [0.0; Params::WORDS];
        p.to_words(&mut words);
        assert_eq!(Params::from_words(&words), Some(p));
        assert_eq!(Run::new(&p, 48_000.0, 1).latency(), 48);

        for x in [-2.0, -0.4, 0.0, 0.4, 2.0] {
            let u = CUBIC_SCALE * K_MIN * x;
            let want = if u >= 1.0 {
                1.0 / K_MIN
            } else if u <= -1.0 {
                -1.0 / K_MIN
            } else {
                (1.5 * u - 0.5 * u * u * u) / K_MIN
            };
            assert!((Saturation::shape(1, K_MIN, x) - want).abs() < 1e-12);
        }
    }

    #[test]
    fn payload_selected_peak_ceiling_is_linked() {
        let mut p = saturating(1);
        p.saturation_flavor = 1;
        p.saturation_mix = 0.0; // 0 dBFS block-peak ceiling
        let mut s = Saturation::new(&p, 48_000.0, 2);
        for _ in 0..DRIVE_BLOCK {
            s.measure(0, 0.5);
            s.measure(1, 2.0);
            s.pos += 1;
        }
        s.end_block();
        assert!((s.peak_scale - 0.5).abs() < 1e-12);
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
