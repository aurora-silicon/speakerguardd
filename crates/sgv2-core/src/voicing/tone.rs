// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Static sections used by the ToneMeister pre-stage and voicing box.
//!
//! Peaks and ordinary shelves use the matched-Z transform with the
//! two-zero magnitude correction described by Gunness and Chauhan,
//! "Optimizing the Magnitude Response of Matched Z Transform Filters".
//! The poles are the analog prototype's two roots, each mapped by
//! `z = exp(sT)`. The minimum-phase numerator is the real second-order FIR
//! whose magnitude matches the analog target at DC, one-third Nyquist and
//! two-thirds Nyquist. This avoids the high-frequency cramping of a bilinear
//! biquad.
//!
//! The box stores every section as five Float32 words and its output depends
//! on their last bits (a box's static cascade runs in Float32, and next to
//! poles this close to the unit circle one word changes the stop band by
//! hundredths of a dB). Every constructor here therefore follows the box's
//! own construction step by step: which values are Float32 and which
//! binary64, where each is rounded, which products are fused, and the
//! platform's complex magnitude. The dB gains are Float32 exponentials of
//! the dB value times Float32 `ln(10)/20` (or `/40`).
//!
//! The Float32 elementary functions are the platform's own
//! ([`crate::platform_math`]), which are not correctly rounded. The binary64
//! `sin`, `cos` and `tan` are the correctly rounded ones, which the
//! platform's differ from in rare last bits.

use crate::prestage::Section;
use core::f64::consts::PI;

fn exp32(x: f32) -> f32 {
    crate::platform_math::expf(x)
}

fn cos32(x: f32) -> f32 {
    crate::platform_math::cosf(x)
}

fn sin32(x: f32) -> f32 {
    crate::platform_math::sinf(x)
}

/// Float32 `ln(10)/20`: a dB value times it is the exponent of the linear
/// amplitude.
const DB_TO_EXPONENT: f32 = (core::f64::consts::LN_10 / 20.0) as f32;
/// Float32 `ln(10)/40`, for the square root of an amplitude.
const DB_TO_HALF_EXPONENT: f32 = (core::f64::consts::LN_10 / 40.0) as f32;

/// A dB gain as the box forms its linear word: `exp(dB * ln(10)/20)`, all
/// Float32. The box's input and output gains and its gain slots use it.
pub fn db_gain(gain_db: f32) -> f32 {
    exp32(gain_db * DB_TO_EXPONENT)
}

/// A flat gain as a section: the box applies a gain as `g*x + 0` in Float32,
/// which is exactly what this section computes in the box's fused Float32
/// step (its state stays zero).
pub fn gain_section(gain_db: f64) -> Section {
    Section { b0: f64::from(db_gain(gain_db as f32)), b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0 }
}

/// The platform's Float32 argument of a negative real number: pi rounded
/// down (black-box: `atan2f(+0, x)` returns this word for every negative
/// `x`, while the nearest Float32 to pi is one unit above it).
const NEGATIVE_REAL_ARGUMENT: f32 = f32::from_bits(0x4049_0fda);

/// Sine and cosine of half the Float32 argument of the real number `x`
/// (`x + 0i`): zero for `x >= 0`, half of [`NEGATIVE_REAL_ARGUMENT`] below.
fn half_argument(x: f32) -> (f32, f32) {
    if x < 0.0 {
        let half = NEGATIVE_REAL_ARGUMENT * 0.5;
        (sin32(half), cos32(half))
    } else {
        (0.0, 1.0)
    }
}

/// The principal square root of `x + 0i` in the box's Float32 polar form:
/// radius `sqrt(sqrt(x*x))` and half the argument.
fn complex_sqrt(x: f32) -> (f32, f32) {
    let radius = libm::sqrtf(libm::sqrtf(libm::fmaf(x, x, 0.0)));
    let (sin, cos) = half_argument(x);
    (cos * radius, sin * radius)
}

/// The Arm Float32 reciprocal-square-root estimate for a positive finite
/// value (Arm ARM `FPRSqrtEstimate`: exponent parity and the leading eight
/// significand bits, a subnormal normalised first).
fn arm_rsqrt_estimate(value: f32) -> f32 {
    let bits = value.to_bits();
    let mut exponent = ((bits >> 23) & 0xff) as i32;
    let mut fraction = u64::from(bits & 0x7f_ffff) << 29;
    if exponent == 0 {
        while (fraction >> 51) & 1 == 0 {
            fraction = (fraction << 1) & ((1 << 52) - 1);
            exponent -= 1;
        }
        fraction = (fraction << 1) & ((1 << 52) - 1);
    }
    let mut scaled = if exponent & 1 == 0 {
        0x100 | ((fraction >> 44) & 0xff) as u32
    } else {
        0x80 | ((fraction >> 45) & 0x7f) as u32
    };
    if scaled < 0x100 {
        scaled = 2 * scaled + 1;
    } else {
        scaled = ((scaled >> 1) * 2 + 1) * 2;
    }
    let mut root = 512u32;
    while u64::from(scaled) * u64::from(root + 1) * u64::from(root + 1) < (1u64 << 28) {
        root += 1;
    }
    let estimate = root.div_ceil(2);
    let result_exponent = (380 - exponent) / 2;
    f32::from_bits(((result_exponent as u32 & 0xff) << 23) | ((estimate & 0xff) << 15))
}

/// The Arm Float32 reciprocal-square-root step `(3 - a*b) / 2`, fused.
fn rsqrt_step(a: f32, b: f32) -> f32 {
    libm::fmaf(-a, b, 3.0) * 0.5
}

/// The platform's one-element Float32 complex magnitude on Arm: the fused
/// power sum `im*im + re*re`, the reciprocal-square-root estimate (zero for
/// a zero power) and two refinement steps, the last applied to the rounded
/// `power * estimate`.
pub(crate) fn platform_magnitude(real: f32, imaginary: f32) -> f32 {
    if real.is_nan() || imaginary.is_nan() {
        return f32::NAN;
    }
    let power = libm::fmaf(imaginary, imaginary, real * real);
    if !power.is_finite() {
        return f32::NAN;
    }
    let mut estimate = if power == 0.0 { 0.0 } else { arm_rsqrt_estimate(power) };
    let product = power * estimate;
    estimate *= rsqrt_step(estimate, product);
    let product = power * estimate;
    product * rsqrt_step(estimate, product)
}

/// The quotient of two Float32 complex numbers as the platform's split
/// complex division forms it: each output part fuses one product and rounds
/// the other, both are scaled by one reciprocal of the fused squared divisor.
fn complex_divide((nr, ni): (f32, f32), (dr, di): (f32, f32)) -> (f32, f32) {
    let real = libm::fmaf(nr, dr, ni * di);
    let imaginary = libm::fmaf(-nr, di, dr * ni);
    let inverse = 1.0 / libm::fmaf(dr, dr, di * di);
    (real * inverse, imaginary * inverse)
}

/// `(i x)^2` as a Float32 complex square of `0 + i x`.
fn imaginary_square(x: f32) -> (f32, f32) {
    let (re, im) = (0.0f32, x);
    (libm::fmaf(re, re, -im * im), libm::fmaf(re, im, im * re))
}

/// The denominator of a peak or ordinary shelf: the two roots of the analog
/// prototype `s^2 + s/q + base` (in units of the slot frequency), each mapped
/// by `z = exp(s w)` on its own, `w` the slot's angular frequency per sample.
/// The roots come from the Float32 polar square root of the discriminant,
/// with the radius scale `w * base^(1/4)` in binary64.
fn mapped_den(q: f32, w: f64, base: f64) -> [f32; 2] {
    let q = f64::from(q);
    let damping = (1.0 / (q + q)) as f32;
    let discriminant = (1.0 / ((q * 4.0) * q) - 1.0) as f32;
    let (u, v) = complex_sqrt(discriminant);
    let scale = (-w * libm::sqrt(libm::sqrt(base))) as f32;
    let (x0, y0) = ((damping + u) * scale, v * scale);
    let (x1, y1) = ((damping - u) * scale, -v * scale);
    let p0 = exp32(x0) * cos32(y0);
    let p1 = exp32(x1) * cos32(y1);
    [-(p0 + p1), exp32(x1 + x0) * cos32(y1 + y0)]
}

/// The reciprocals of the denominator's magnitude at DC, one-third and
/// two-thirds Nyquist, from its Float32 words in binary64.
fn inverse_den_magnitudes([a1, a2]: [f32; 2]) -> [f64; 3] {
    let (a, b) = (f64::from(a1), f64::from(a2));
    let d0 = ((1.0 + a) + b).abs();
    let d1 = libm::fma(b, b, libm::fma(a, b, libm::fma(a, a, (1.0 + a) - b)));
    let d2 = libm::fma(b, b, libm::fma(-a, b, libm::fma(a, a, (1.0 - a) - b)));
    [1.0 / d0, 1.0 / libm::sqrt(d1.abs()), 1.0 / libm::sqrt(d2.abs())]
}

/// `m - sqrt(x)` for a real `x`, as a real: its magnitude when the root is
/// imaginary, else its real part.
fn sqrt_difference(m: f32, x: f32) -> f32 {
    let (re, im) = complex_sqrt(x);
    let real = m - re;
    if x < 0.0 {
        platform_magnitude(real, -im)
    } else {
        real
    }
}

/// The minimum-phase second-order FIR `[b0, b1, b2]` whose magnitudes at
/// DC, one-third and two-thirds Nyquist are `h`: binary64 fused
/// discriminants, Float32 roots, `b0` kept in binary64 until it is stored.
fn fit([h0, h1, h2]: [f64; 3]) -> (f64, f32, f32) {
    let first = libm::fma(h2 + h2, h2, libm::fma(h0, h0, -((h1 + h1) * h1)));
    let b1 = sqrt_difference(h0 as f32, first as f32) * 0.5;
    let b1d = f64::from(b1);
    let mut second = libm::fma(-3.0 * h0, h0, (12.0 * h1) * h1);
    second = libm::fma(-6.0 * h0, b1d, second);
    second = libm::fma(-3.0 * b1d, b1d, second);
    let lead = ((h0 - b1d) * 3.0) as f32;
    let b2 = sqrt_difference(lead, second as f32) / 6.0;
    ((h0 - b1d) - f64::from(b2), b1, b2)
}

/// The slot's angular frequency per sample, from its Float32 frequency and
/// rate.
fn angular(f0: f64, fs: f64) -> f64 {
    f64::from(f0 as f32) * (2.0 * PI) / f64::from(fs as f32)
}

/// The two analog evaluation points `(pi/3)/w` and twice it, in Float32.
fn evaluation_points(w: f64) -> [f32; 2] {
    let x1 = ((PI / 3.0) / w) as f32;
    [x1, x1 + x1]
}

fn section(b: [f32; 3], a: [f32; 2]) -> Section {
    Section { b0: f64::from(b[0]), b1: f64::from(b[1]), b2: f64::from(b[2]), a1: f64::from(a[0]), a2: f64::from(a[1]) }
}

/// `|(z^2 + (g/q) z + 1) / (z^2 + z/q + 1)|` at `z = i x`, in Float32.
fn peak_analog_magnitude(x: f32, gain: f64, q: f32) -> f32 {
    let (zr, zi) = imaginary_square(x);
    let (re, im) = (0.0f32, x);
    let zeros = (gain / f64::from(q)) as f32;
    let poles = 1.0 / q;
    let num = ((zr + re * zeros) + 1.0, zi + im * zeros);
    let den = ((zr + re * poles) + 1.0, zi + im * poles);
    let (r, i) = complex_divide(num, den);
    platform_magnitude(r, i)
}

/// A decramped peaking section, as the box stores it.
///
/// A boost's prototype is `s^2 + (A^2/q') s + 1` over `s^2 + s/q' + 1` with
/// `q' = A q`, `A = 10^(g/40)`. A cut is the exact inverse of the
/// complementary boost: its pole quality takes `A` of the positive gain, but
/// its linear gain is the binary64 reciprocal of the Float32 exponential of
/// the negative gain, and the inverse is formed before the words are rounded.
pub fn peak(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    let g = gain_db as f32;
    let q = q as f32;
    let (amplitude, linear) = if g < 0.0 {
        (exp32(-g * DB_TO_HALF_EXPONENT), 1.0 / f64::from(exp32(g * DB_TO_EXPONENT)))
    } else {
        (exp32(g * DB_TO_HALF_EXPONENT), f64::from(exp32(g * DB_TO_EXPONENT)))
    };
    let qe = amplitude * q;
    let w = angular(f0, fs);
    let a = mapped_den(qe, w, 1.0);
    let inv = inverse_den_magnitudes(a);
    let [x1, x2] = evaluation_points(w);
    let h = [
        1.0 / inv[0],
        f64::from(peak_analog_magnitude(x1, linear, qe)) / inv[1],
        f64::from(peak_analog_magnitude(x2, linear, qe)) / inv[2],
    ];
    let (b0, b1, b2) = fit(h);
    if g < 0.0 {
        let k = |v: f64| (v / b0) as f32;
        section([(1.0 / b0) as f32, k(f64::from(a[0])), k(f64::from(a[1]))], [k(f64::from(b1)), k(f64::from(b2))])
    } else {
        section([b0 as f32, b1, b2], a)
    }
}

/// The quality a ToneMeister peak of the first algorithm version (box
/// parameter 12 = 0) is designed with. That version specifies a peak's
/// bandwidth on its pole: a boost's zeros keep the saved quality and its
/// poles take it times the linear gain, a cut is the inverse of the
/// matching boost. From about +6 dB on (where `A^2 - 2/A^2` reaches 1) the
/// quality is narrowed so the bandwidth stays where the response is 3 dB
/// below the peak. As a quality for [`peak`] (poles `q A`, zeros `q / A`):
/// `q A c`, `A = 10^(|g| / 40)`, `c = min(1, 1 / sqrt(A^2 - 2 / A^2))`.
/// Black-box: single-peak renders of the installed unit over gains -15 to
/// +15 dB, qualities 0.5 to 4 and 50 Hz to 12 kHz give the pole quality of
/// this law to 1e-5.
pub fn legacy_peak_quality(gain_db: f64, q: f64) -> f64 {
    let a = libm::pow(10.0, gain_db.abs() / 40.0);
    let a2 = a * a;
    let narrowing = a2 - 2.0 / a2;
    let c = if narrowing > 1.0 { 1.0 / libm::sqrt(narrowing) } else { 1.0 };
    q * a * c
}

/// A peaking section of the first ToneMeister algorithm version: [`peak`]
/// at [`legacy_peak_quality`].
pub fn legacy_peak(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    peak(gain_db, f0, legacy_peak_quality(gain_db, q), fs)
}

/// `|A (z^2 + c z + A) / (A z^2 + c z + 1)|` (low shelf) or
/// `|A (A z^2 + c z + 1) / (z^2 + c z + A)|` (high shelf) at `z = i x`, in
/// Float32, with `A = sqrt(G)` and `c = G^(1/4) / q` from two binary64
/// square roots rounded once.
fn shelf_analog_magnitude(high: bool, gain: f32, q: f32, x: f32) -> f32 {
    let a64 = libm::sqrt(f64::from(gain));
    let a = a64 as f32;
    let c1 = (libm::sqrt(a64) / f64::from(q)) as f32;
    let (zr, zi) = imaginary_square(x);
    let (cr, ci) = (0.0f32 * c1, x * c1);
    let unit_lead = (zr + (cr + a), zi + ci);
    let scaled_lead = (zr * a + (cr + 1.0), zi * a + ci);
    let (num, den) = if high { (scaled_lead, unit_lead) } else { (unit_lead, scaled_lead) };
    let num = (num.0 * a, num.1 * a);
    let (r, i) = complex_divide(num, den);
    platform_magnitude(r, i)
}

/// An ordinary (decramped) shelf, boost or cut, designed directly: its
/// poles are the prototype's roots with the natural frequency scaled by
/// `G^-1/2` (low shelf) or `G^1/2` (high shelf), `G = 10^(g/20)`.
fn shelf(high: bool, gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    let g = gain_db as f32;
    let gain = db_gain(g);
    let q = q as f32;
    let w = angular(f0, fs);
    let base = if high { f64::from(gain) } else { 1.0 / f64::from(gain) };
    let a = mapped_den(q, w, base);
    let inv = inverse_den_magnitudes(a);
    let [x1, x2] = evaluation_points(w);
    let dc = if high { 1.0 } else { f64::from(gain) };
    let h = [
        dc / inv[0],
        f64::from(shelf_analog_magnitude(high, gain, q, x1)) / inv[1],
        f64::from(shelf_analog_magnitude(high, gain, q, x2)) / inv[2],
    ];
    let (b0, b1, b2) = fit(h);
    section([b0 as f32, b1, b2], a)
}

/// A decramped low-shelf section. A 0 dB low shelf is the identity.
pub fn low_shelf(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    if gain_db as f32 == 0.0 {
        return Section::IDENTITY;
    }
    shelf(false, gain_db, f0, q, fs)
}

/// A low shelf rebuilt by the volume-automation object. Unlike a static
/// zero-gain slot, the running object keeps its cancelling pole/zero words
/// when its gain reaches zero.
pub(crate) fn automated_low_shelf(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    shelf(false, gain_db, f0, q, fs)
}

/// A decramped high-shelf section. A 0 dB high shelf keeps its section, a
/// pole and zero pair that cancel up to its Float32 words.
pub fn high_shelf(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    shelf(true, gain_db, f0, q, fs)
}

/// ToneMeister's phase-compensated high shelf and its companion allpass.
///
/// A channel selected by the slot's filter mask runs the shelf; a channel in
/// the separate phase mask runs the allpass. The shelf is a high shelf of
/// half the gain (in Float32) followed by its numerator-reversed twin, so its
/// magnitude is the half-gain magnitude squared and its phase is exactly
/// that of the allpass built on the same poles.
pub fn phase_compensated_high_shelf(gain_db: f64, f0: f64, q: f64, fs: f64) -> ([Section; 2], Section) {
    let half = high_shelf(f64::from(gain_db as f32 * 0.5), f0, q, fs);
    let twin = Section { b0: half.b2, b1: half.b1, b2: half.b0, a1: half.a1, a2: half.a2 };
    ([half, twin], allpass(half))
}

/// The allpass on a section's poles, `[a2, a1, 1, a1, a2]`.
pub fn allpass(s: Section) -> Section {
    Section { b0: s.a2, b1: s.a1, b2: 1.0, a1: s.a1, a2: s.a2 }
}

/// ToneMeister's resonant high-pass slot. `Q` is stored in an analog decay
/// convention; its bilinear equivalent is `1 / (2*sinh(1/(2*Q)))`.
pub fn resonant_high_pass(f0: f64, q: f64, fs: f64) -> Section {
    let w = 2.0 * PI * f0 / fs;
    let c = libm::cos(w);
    let alpha = libm::sin(w) * libm::sinh(1.0 / (2.0 * q));
    let n = 1.0 + alpha;
    Section {
        b0: (1.0 + c) / (2.0 * n),
        b1: -(1.0 + c) / n,
        b2: (1.0 + c) / (2.0 * n),
        a1: -2.0 * c / n,
        a2: (1.0 - alpha) / n,
    }
}

/// The highest Butterworth shelf order a ToneMeister box accepts
/// (`MaxAllowedFilterOrder`, parameter 5; the lowest is two).
pub const MAX_SHELF_ORDER: u32 = 50;
/// The box's shelf order limit when the tuning does not set one.
pub const DEFAULT_SHELF_ORDER: u32 = 10;
/// Most sections one cascade holds: a shelf at the highest order limit.
pub const MAX_CASCADE: usize = MAX_SHELF_ORDER as usize / 2 + 1;

/// A fixed-capacity static cascade returned by [`butterworth_shelf`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cascade {
    pub sections: [Section; MAX_CASCADE],
    pub count: usize,
}

impl Cascade {
    const EMPTY: Cascade = Cascade { sections: [Section::IDENTITY; MAX_CASCADE], count: 0 };

    fn push(&mut self, s: Section) {
        self.sections[self.count] = s;
        self.count += 1;
    }
}

/// A Butterworth low- or high-pass of integer order, as the box stores it:
/// the platform voicing library's Float32 construction (corner clamped to
/// 20 Hz .. 0.45 of the rate, rounded angle, sine and cosine, numerators
/// formed from the rounded poles), widened. No allocation (volume automation
/// recomputes it on the audio thread).
pub fn butterworth(high: bool, order: u32, f0: f64, fs: f64) -> Cascade {
    use super::vbass::{butter_words, Kind, Words};
    struct Sink(Cascade);
    impl Extend<Words> for Sink {
        fn extend<I: IntoIterator<Item = Words>>(&mut self, words: I) {
            for w in words {
                if self.0.count < MAX_CASCADE {
                    let [b0, b1, b2, a1, a2] = w.map(f64::from);
                    self.0.push(Section { b0, b1, b2, a1, a2 });
                }
            }
        }
    }
    let mut out = Sink(Cascade::EMPTY);
    let order = order.clamp(1, 20);
    let kind = if high { Kind::High } else { Kind::Low };
    butter_words(order as usize, (f0 as f32).max(20.0), fs as f32, kind, &mut out);
    out.0
}

/// The box's phase-matched Butterworth low-pass of an even `order`: two
/// copies of the half-order low-pass for a filtered channel, and the allpass
/// on the same poles for a phase channel. An odd half-order's first-order
/// allpass is the one-frame-delayed `[0, a1, 1, a1, 0]`; the filtered
/// channel then runs a one-frame delay after its two copies, so both paths
/// keep the same phase.
pub fn phase_matched_low_pass(order: u32, f0: f64, fs: f64) -> (Cascade, Cascade) {
    let half = butterworth(false, order / 2, f0, fs);
    let mut filtered = half;
    let mut phase = Cascade::EMPTY;
    for s in &half.sections[..half.count] {
        if filtered.count < MAX_CASCADE {
            filtered.push(*s);
        }
        phase.push(allpass(*s));
    }
    if (order / 2) % 2 != 0 && filtered.count < MAX_CASCADE {
        filtered.push(Section { b0: 0.0, b1: 1.0, b2: 0.0, a1: 0.0, a2: 0.0 });
    }
    (filtered, phase)
}

/// A double-double value `hi + lo`, `|lo| <= ulp(hi) / 2`.
type Dd = (f64, f64);

fn two_sum(a: f64, b: f64) -> Dd {
    let s = a + b;
    let v = s - a;
    (s, (a - (s - v)) + (b - v))
}

fn dd_add(a: Dd, b: Dd) -> Dd {
    let (s, e) = two_sum(a.0, b.0);
    two_sum(s, e + a.1 + b.1)
}

fn dd_mul(a: Dd, b: Dd) -> Dd {
    let p = a.0 * b.0;
    let e = libm::fma(a.0, b.0, -p) + (a.0 * b.1 + a.1 * b.0);
    two_sum(p, e)
}

fn dd_div(a: Dd, b: Dd) -> Dd {
    let q1 = a.0 / b.0;
    let r = dd_add(a, dd_mul(b, (-q1, 0.0)));
    let q2 = r.0 / b.0;
    let r = dd_add(r, dd_mul(b, (-q2, 0.0)));
    let q3 = r.0 / b.0;
    dd_add(two_sum(q1, q2), (q3, 0.0))
}

/// The natural logarithm of a positive normal `x`, correctly rounded to
/// binary64 (but for ties closer than about 2^-100 relative). The shelf's
/// real order is a difference of two logarithms, and its integer part
/// sets the section count, so it must not depend on a libm's last bit.
fn ln(x: f64) -> f64 {
    const LN_2: Dd = (core::f64::consts::LN_2, 2.319_046_813_846_299_6e-17);
    let bits = x.to_bits();
    let mut e = ((bits >> 52) & 0x7ff) as i32 - 1023;
    let mut m = f64::from_bits((bits & ((1 << 52) - 1)) | (1023 << 52));
    if m > core::f64::consts::SQRT_2 {
        m *= 0.5;
        e += 1;
    }
    // ln m = 2 atanh(s), s = (m - 1)/(m + 1), |s| < 0.172
    let s = dd_div((m - 1.0, 0.0), two_sum(m, 1.0));
    let s2 = dd_mul(s, s);
    let mut sum: Dd = (0.0, 0.0);
    for k in (0..24).rev() {
        let term = dd_div((1.0, 0.0), ((2 * k + 1) as f64, 0.0));
        sum = dd_add(term, dd_mul(s2, sum));
    }
    let lnm = dd_mul((2.0 * s.0, 2.0 * s.1), sum);
    let r = dd_add(dd_mul(LN_2, (e as f64, 0.0)), lnm);
    r.0 + r.1
}

/// The most sections one Butterworth shelf makes under an order limit:
/// one per pole pair (or real pole) of the highest integer order, plus the
/// real remainder section that is always present.
pub fn shelf_sections(max_order: u32) -> usize {
    (max_order.clamp(2, MAX_SHELF_ORDER) as usize).div_ceil(2) + 1
}

/// A Float32 complex root `(re, im)`; `im == 0` is a real root.
type Root = (f32, f32);

/// Maps an analog root (normalised to the prewarped midpoint) to the z
/// plane: `(1 + k q) / (1 - k q)`, one Float32 rounding per operation, the
/// quotient through a reciprocal of the squared denominator magnitude.
fn bilinear_root(k: f32, (qr, qi): Root) -> Root {
    let br = 1.0 + k * qr;
    let bi = k * qi;
    let dr = 1.0 + -k * qr;
    let di = -k * qi;
    let inv = 1.0 / libm::fmaf(dr, dr, di * di);
    (inv * libm::fmaf(br, dr, bi * di), inv * libm::fmaf(bi, dr, -(di * br)))
}

/// The `z^-1` and `z^-2` coefficients of the monic factor with the root
/// (and its conjugate when complex).
fn monic(q: Root) -> (f32, f32) {
    if q.1 == 0.0 {
        return (-q.0, 0.0);
    }
    let (re, im) = (-q.0 as f64, -q.1 as f64);
    (-2.0 * q.0, libm::fma(re, re, im * im) as f32)
}

/// One section from a pole and a zero root, its numerator scaled for unit
/// gain at Nyquist.
fn root_section(pole: Root, zero: Root) -> Section {
    let (a1, a2) = monic(pole);
    let (c1, c2) = monic(zero);
    let at_nyquist_den = (1.0 - a1) + a2;
    let at_nyquist_num = (1.0 - c1) + c2;
    let v = at_nyquist_num / at_nyquist_den;
    Section { b0: (1.0 / v) as f64, b1: (c1 / v) as f64, b2: (c2 / v) as f64, a1: a1 as f64, a2: a2 as f64 }
}

/// ToneMeister's type-2 Butterworth shelf over the slot's two edge
/// frequencies, in their saved order: rising edges put the gain below the
/// transition, falling edges above it. The design follows the unit's
/// Float32 values and arithmetic order, so each section is bit-exact.
///
/// The real order is the gain over three times the transition width in
/// octaves, at least one and at most `max_order` (two to
/// [`MAX_SHELF_ORDER`]). Each Butterworth pole angle is interpolated between
/// the two adjacent integer orders (a harmonic interpolation of the order),
/// every analog root takes an equal logarithmic share of the gain, and one
/// real remainder root carries the fractional order up to the adjacent
/// even integer. The remainder section is kept even when its pole and zero
/// cancel: the unit runs it, and its rounding is part of the output. The
/// roots are bilinear-mapped at the prewarped geometric-mean frequency. A
/// falling shelf is designed as the rising one with the opposite gain and
/// a flat gain folded into its first numerator.
pub fn butterworth_shelf(gain_db: f64, from_hz: f64, to_hz: f64, max_order: u32, fs: f64) -> Cascade {
    let mut out = Cascade::EMPTY;
    let fs = fs as f32;
    let top = (0.499 * fs).min(20_000.0);
    let edge = |f: f64| (f as f32).min(top).max(20.0);
    let (from, to) = (edge(from_hz), edge(to_hz));
    let gain = (gain_db as f32).clamp(-40.0, 40.0);
    if from == to {
        return out;
    }
    let (lo, hi, rising_gain, flat_db) = if from < to { (from, to, gain, 0.0) } else { (to, from, -gain, gain) };

    let width = ln(hi as f64) - ln(lo as f64);
    let per_octave = (rising_gain as f64).abs() / width * core::f64::consts::LN_2 / 6.0;
    let order = (per_octave + per_octave).max(1.0).min(max_order.clamp(2, MAX_SHELF_ORDER) as f64);
    let whole = libm::floor(order);
    let fraction = order - whole;
    let lower = whole as u32;
    let pairs = lower.div_ceil(2);

    let fm = libm::sqrtf(lo * hi);
    let k = libm::tan(fm as f64 * (2.0 * PI) / fs as f64 * 0.5) as f32;
    let share = rising_gain as f64 / (40.0 * order);
    let (pole_r, zero_r) = (libm::exp10(-share), libm::exp10(share));
    let angle = |j: u32, n: f64| (0.5 - (2 * j - 1) as f64 * 0.5 / n) * PI;
    for j in 1..=pairs {
        let theta = angle(j, lower as f64 + 1.0) * fraction + angle(j, lower as f64) * (1.0 - fraction);
        let (c, s) = (libm::cos(theta), libm::sin(theta));
        let pole = ((-pole_r * c) as f32, (pole_r * s) as f32);
        let zero = ((-zero_r * c) as f32, (zero_r * s) as f32);
        out.push(root_section(bilinear_root(k, pole), bilinear_root(k, zero)));
    }
    let even = if lower % 2 == 1 { libm::ceil(order) } else { whole };
    let t = (share * (even - order)) as f32;
    out.push(root_section(
        bilinear_root(k, (-crate::platform_math::exp10f(t), 0.0)),
        bilinear_root(k, (-crate::platform_math::exp10f(-t), 0.0)),
    ));

    let flat = libm::exp10(flat_db as f64 / 20.0);
    let first = &mut out.sections[0];
    for b in [&mut first.b0, &mut first.b1, &mut first.b2] {
        *b = (*b * flat) as f32 as f64;
    }
    out
}

/// ToneMeister's type-0 band shelf over its four edges: a falling-gain
/// shelf over the first two edges followed by a rising-gain one over the
/// last two. For ordered edges the outer plateaus cancel and the band
/// between the inner edges keeps the gain; no separate gain stage exists.
pub fn butterworth_band_shelf(gain_db: f64, edges: [f64; 4], max_order: u32, fs: f64) -> (Cascade, Cascade) {
    (
        butterworth_shelf(-(gain_db as f32) as f64, edges[0], edges[1], max_order, fs),
        butterworth_shelf(gain_db, edges[2], edges[3], max_order, fs),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn magnitude(s: Section, f: f64, fs: f64) -> f64 {
        s.magnitude(f, fs)
    }

    #[test]
    fn legacy_peaks_keep_the_saved_quality_on_one_side_up_to_6_db() {
        // up to +6 dB: zeros at the saved quality, poles at q A^2
        for g in [0.5, 1.0, 3.0, 6.0] {
            let a = libm::pow(10.0, g / 40.0);
            assert!((legacy_peak_quality(g, 1.5) - 1.5 * a).abs() < 1e-12, "{g}");
            assert_eq!(legacy_peak_quality(-g, 1.5), legacy_peak_quality(g, 1.5));
        }
        // above: narrowed by 1/sqrt(A^2 - 2/A^2), continuous at the knee
        let knee = 20.0 * libm::log10(2.0);
        let below = legacy_peak_quality(knee - 1e-9, 1.0);
        let above = legacy_peak_quality(knee + 1e-9, 1.0);
        assert!((below - above).abs() < 1e-8);
        let a: f64 = libm::pow(10.0, 12.0 / 40.0);
        let want = a / libm::sqrt(a * a - 2.0 / (a * a));
        assert!((legacy_peak_quality(12.0, 1.0) - want).abs() < 1e-12);
        // the same peak gain, a different bandwidth
        let fs = 48_000.0;
        let s = legacy_peak(12.0, 1000.0, 1.0, fs);
        let r = peak(12.0, 1000.0, 1.0, fs);
        assert!((20.0 * libm::log10(magnitude(s, 1000.0, fs)) - 12.0).abs() < 0.2);
        assert!(magnitude(s, 500.0, fs) < magnitude(r, 500.0, fs));
    }

    /// The analog matched-Z poles of `s^2 + s/q + base`, in binary64.
    fn matched_poles(q: f64, w: f64, base: f64) -> [f64; 2] {
        let (d, r2) = (1.0 / (2.0 * q), 1.0);
        let wn = w * libm::sqrt(libm::sqrt(base));
        if d * d <= r2 {
            let radius = libm::exp(-d * wn);
            [-2.0 * radius * libm::cos(libm::sqrt(r2 - d * d) * wn), radius * radius]
        } else {
            let spread = libm::sqrt(d * d - r2);
            let (p0, p1) = (libm::exp(-(d - spread) * wn), libm::exp(-(d + spread) * wn));
            [-(p0 + p1), p0 * p1]
        }
    }

    #[test]
    fn mapped_poles_are_the_analog_roots_in_float32() {
        let fs = 48_000.0;
        for base in [1.0, 0.25, 4.0] {
            for q in [0.2f32, 0.4, 0.5, 0.75, 1.0, 5.0] {
                for f0 in [40.0, 200.0, 8500.0] {
                    let w = angular(f0, fs);
                    let want = matched_poles(f64::from(q), w, base);
                    let got = mapped_den(q, w, base);
                    for (g, e) in got.into_iter().zip(want) {
                        assert!((f64::from(g) - e).abs() < 4e-6, "q {q} f {f0} base {base}: {g} vs {e}");
                    }
                }
            }
        }
    }

    #[test]
    fn a_negative_real_has_the_platform_argument() {
        // pi rounded down, its half's cosine as the platform returns it
        assert_eq!(NEGATIVE_REAL_ARGUMENT.to_bits() + 1, core::f32::consts::PI.to_bits());
        assert_eq!(half_argument(-2.0), (1.0, f32::from_bits(0x33a2_2169)));
        assert_eq!(half_argument(0.0), (0.0, 1.0));
        let (re, im) = complex_sqrt(-4.0);
        assert_eq!((re, im), (f32::from_bits(0x33a2_2169) * 2.0, 2.0));
    }

    #[test]
    fn a_peak_cut_inverts_its_complementary_boost_before_rounding() {
        let fs = 48_000.0;
        for (g, f0, q) in [(7.25, 1234.5, 2.5), (3.0, 90.0, 0.7), (12.0, 18_000.0, 2.0)] {
            let boost = peak(g, f0, q, fs);
            let cut = peak(-g, f0, q, fs);
            // the cut's zeros are the boost's poles
            assert!((cut.b1 / cut.b0 - boost.a1).abs() < 1e-6 && (cut.b2 / cut.b0 - boost.a2).abs() < 1e-6);
            for f in [20.0, f0 * 0.5, f0, f0 * 2.0, 20_000.0] {
                // exact inverses up to their Float32 words (a hundredth of a dB
                // next to poles close to the unit circle)
                assert!((magnitude(boost, f, fs) * magnitude(cut, f, fs) - 1.0).abs() < 1.2e-3, "{g} {f0} {f}");
            }
            for c in [cut.b0, cut.b1, cut.b2, cut.a1, cut.a2] {
                assert_eq!(c, c as f32 as f64);
            }
        }
    }

    #[test]
    fn gains_are_float32_exponentials_applied_as_a_fused_product() {
        assert_eq!(db_gain(-6.0).to_bits(), 0x3f00_4dcf);
        assert_eq!(db_gain(0.0), 1.0);
        let s = gain_section(-18.5);
        let g = s.b0 as f32;
        assert_eq!((s.b1, s.b2, s.a1, s.a2), (0.0, 0.0, 0.0, 0.0));
        assert!((20.0 * libm::log10(f64::from(g)) + 18.5).abs() < 1e-5);
        assert!((magnitude(s, 1000.0, 48_000.0) - f64::from(g)).abs() < 1e-12);
    }

    #[test]
    fn platform_magnitude_is_within_an_ulp_of_the_square_root() {
        for (re, im) in [(1.0f32, 0.0f32), (0.3, -0.4), (1e-3, 2.5), (-7.25, 0.125)] {
            let exact = libm::sqrt(f64::from(re) * f64::from(re) + f64::from(im) * f64::from(im));
            let got = f64::from(platform_magnitude(re, im));
            assert!((got - exact).abs() <= exact * 2.5e-7, "{re} {im}: {got} vs {exact}");
        }
    }

    #[test]
    fn complementary_peaks_are_exact_inverses() {
        let fs = 48_000.0;
        let boost = peak(12.0, 18_000.0, 2.0, fs);
        let cut = peak(-12.0, 18_000.0, 2.0, fs);
        for f in [20.0, 1000.0, 8000.0, 16_000.0, 20_000.0, 23_900.0] {
            assert!((magnitude(boost, f, fs) * magnitude(cut, f, fs) - 1.0).abs() < 2e-6);
        }
    }

    #[test]
    fn a_zero_gain_peak_is_a_real_section() {
        // The installed unit still runs a 0 dB peak; its section is the
        // same matched design (unity at the fit points), not the identity.
        let fs = 48_000.0;
        let flat = peak(0.0, 97.5, 3.0, fs);
        assert_ne!(flat, Section::IDENTITY);
        // its poles are the analog prototype's, as for any boost
        let boost = peak(4.0, 97.5, 3.0, fs);
        assert!((flat.a1 - boost.a1).abs() < 1e-3 && (flat.a2 - boost.a2).abs() < 1e-3);
        // unity to within its Float32 coefficients (a few thousandths of a
        // dB next to poles this close to the unit circle)
        for f in [0.0, 20.0, 97.5, 1000.0, 8_000.0, 16_000.0, 20_000.0] {
            assert!((magnitude(flat, f, fs) - 1.0).abs() < 1e-3, "{f} {}", magnitude(flat, f, fs));
        }
    }

    #[test]
    fn an_automated_zero_gain_low_shelf_keeps_its_cancelling_words() {
        let direct = low_shelf(0.0, 420.0, 0.8, 48_000.0);
        let automated = automated_low_shelf(0.0, 420.0, 0.8, 48_000.0);
        assert_eq!(direct, Section::IDENTITY);
        assert_ne!(automated, Section::IDENTITY);
        assert!((automated.magnitude(20.0, 48_000.0) - 1.0).abs() < 1e-5);
        assert!((automated.magnitude(20_000.0, 48_000.0) - 1.0).abs() < 1e-5);
    }

    fn cascade_db(c: &Cascade, f: f64, fs: f64) -> f64 {
        20.0 * libm::log10(c.sections[..c.count].iter().map(|s| magnitude(*s, f, fs)).product::<f64>())
    }

    #[test]
    fn ln_is_correctly_rounded() {
        // ln 2 and ln 10 to binary64, and values where a one-ulp libm
        // error moves a one-octave shelf's real order across 1
        assert_eq!(ln(2.0), core::f64::consts::LN_2);
        assert_eq!(ln(10.0), core::f64::consts::LN_10);
        assert_eq!(ln(1.0), 0.0);
        assert_eq!(ln(98.672_531_127_929_69), f64::from_bits(0x4012_5e02_8cb3_5b4f));
        assert_eq!(ln(2_892.636_718_75), f64::from_bits(0x401f_e133_af52_e423));
        for x in [20.0, 141.4, 1000.0, 19_999.0, 0.75, 1.5] {
            assert!((ln(x) - libm::log(x)).abs() <= 1.0e-15 * libm::log(x).abs().max(1.0));
        }
    }

    #[test]
    fn butterworth_shelf_sections_are_pole_pairs_plus_a_remainder() {
        let fs = 48_000.0;
        let lo = 120.0;
        let hi = 180.0;
        let gain_for = |order: f64| order * 3.0 * libm::log2(hi / lo);

        // One pole pair below order three, two above; the real remainder
        // section is always there.
        assert_eq!(butterworth_shelf(gain_for(2.75), lo, hi, 10, fs).count, 2);
        assert_eq!(butterworth_shelf(gain_for(3.25), lo, hi, 10, fs).count, 3);
        // An order under one is raised to one: one first-order section and
        // the remainder.
        assert_eq!(butterworth_shelf(0.5, lo, hi, 10, fs).count, 2);

        // MaxAllowedFilterOrder limits the real order, from two up.
        for (limit, count) in [(1, 2), (2, 2), (3, 3), (6, 4), (10, 5), (50, 5)] {
            assert_eq!(butterworth_shelf(gain_for(8.0), lo, hi, limit, fs).count, count, "limit {limit}");
            assert!(count <= shelf_sections(limit));
        }
        // the gain is clamped to 40 dB: a steep narrow shelf reaches the
        // highest limit
        let steep = 49.5 * 3.0 * libm::log2(130.0 / lo);
        assert_eq!(butterworth_shelf(steep, lo, 130.0, 50, fs).count, shelf_sections(50));
    }

    #[test]
    fn butterworth_shelf_reaches_its_asymptotes_and_half_gain_at_the_midpoint() {
        let fs = 48_000.0;
        for gain in [-9.0, -3.0, 4.5, 12.0] {
            for (from, to, low, high) in [(2000.0, 8000.0, gain, 0.0), (8000.0, 2000.0, 0.0, gain)] {
                let c = butterworth_shelf(gain, from, to, 10, fs);
                assert!((cascade_db(&c, 1.0, fs) - low).abs() < 1e-3, "{gain} {from}->{to} low");
                assert!((cascade_db(&c, 23_999.0, fs) - high).abs() < 1e-3, "{gain} {from}->{to} high");
                assert!((cascade_db(&c, 4000.0, fs) - gain / 2.0).abs() < 1e-3, "{gain} {from}->{to} mid");
                for s in &c.sections[..c.count] {
                    assert_eq!([s.b0, s.b1, s.b2, s.a1, s.a2].map(|v| v as f32 as f64), [s.b0, s.b1, s.b2, s.a1, s.a2]);
                }
            }
        }
        // a 0 dB shelf and equal edges are the identity
        assert!(cascade_db(&butterworth_shelf(0.0, 100.0, 200.0, 10, fs), 150.0, fs).abs() < 1e-5);
        assert_eq!(butterworth_shelf(6.0, 300.0, 300.0, 10, fs).count, 0);
    }

    #[test]
    fn an_even_order_shelf_still_runs_its_cancelling_remainder() {
        let fs = 48_000.0;
        // 12 dB over one octave: order four, two pole pairs and a
        // remainder whose pole and zero coincide
        let c = butterworth_shelf(12.0, 500.0, 1000.0, 10, fs);
        assert_eq!(c.count, 3);
        let r = c.sections[2];
        assert_eq!((r.b0, r.b1, r.b2, r.a2), (1.0, r.a1, 0.0, 0.0));
    }

    #[test]
    fn band_shelf_keeps_the_gain_between_its_inner_edges_only() {
        let fs = 48_000.0;
        for gain in [-6.0, 8.0] {
            let (first, second) = butterworth_band_shelf(gain, [100.0, 200.0, 2000.0, 4000.0], 10, fs);
            let db = |f: f64| cascade_db(&first, f, fs) + cascade_db(&second, f, fs);
            // Float32 coefficients of a low-frequency section move its DC
            // gain by a few thousandths of a dB, as the unit's do
            assert!(db(1.0).abs() < 0.01 && db(23_999.0).abs() < 1e-3, "{gain}: {} {}", db(1.0), db(23_999.0));
            assert!((db(630.0) - gain).abs() < 0.05, "{gain}: {}", db(630.0));
        }
    }

    #[test]
    fn matched_shelves_have_the_requested_asymptotes() {
        let fs = 48_000.0;
        let low = low_shelf(6.0, 1000.0, core::f64::consts::FRAC_1_SQRT_2, fs);
        let high = high_shelf(6.0, 1000.0, core::f64::consts::FRAC_1_SQRT_2, fs);
        assert!((20.0 * libm::log10(magnitude(low, 0.0, fs)) - 6.0).abs() < 1e-3);
        assert!(20.0 * libm::log10(magnitude(low, 23_999.0, fs)) < 0.01);
        assert!(20.0 * libm::log10(magnitude(high, 0.0, fs)).abs() < 1e-3);
        assert!((20.0 * libm::log10(magnitude(high, 23_999.0, fs)) - 6.0).abs() < 0.01);
        // a high-shelf cut is designed directly, not as the inverse boost
        let cut = high_shelf(-12.0, 6000.0, core::f64::consts::FRAC_1_SQRT_2, fs);
        assert!(20.0 * libm::log10(magnitude(cut, 0.0, fs)).abs() < 1e-3);
        assert!((20.0 * libm::log10(magnitude(cut, 16_000.0, fs)) + 12.0).abs() < 0.5);
        let b = high_shelf(12.0, 6000.0, core::f64::consts::FRAC_1_SQRT_2, fs);
        let inverse_boost =
            Section { b0: 1.0 / b.b0, b1: b.a1 / b.b0, b2: b.a2 / b.b0, a1: b.b1 / b.b0, a2: b.b2 / b.b0 };
        assert!((magnitude(cut, 4000.0, fs) - magnitude(inverse_boost, 4000.0, fs)).abs() > 1e-4);
    }

    #[test]
    fn phase_compensated_shelf_and_companion_share_phase() {
        let fs = 48_000.0;
        for gain in [-6.0, -2.0, 3.0, 6.0] {
            let ([half, twin], allpass) = phase_compensated_high_shelf(gain, 1200.0, 0.9, fs);
            let shelf = |f: f64| magnitude(half, f, fs) * magnitude(twin, f, fs);
            for f in [20.0, 300.0, 1200.0, 5000.0, 16_000.0] {
                assert!((magnitude(allpass, f, fs) - 1.0).abs() < 1e-12);
                // The twin mirrors the half section's zeros: equal magnitude.
                assert!((magnitude(half, f, fs) - magnitude(twin, f, fs)).abs() < 1e-12);
                // Same phase as the companion: the cascade over the allpass
                // is real and positive.
                let w = 2.0 * core::f64::consts::PI * f / fs;
                let ph = |s: Section| {
                    let z = |k: f64| (libm::cos(k * w), -libm::sin(k * w));
                    let (c1, c2) = (z(1.0), z(2.0));
                    let n = (s.b0 + s.b1 * c1.0 + s.b2 * c2.0, s.b1 * c1.1 + s.b2 * c2.1);
                    let d = (1.0 + s.a1 * c1.0 + s.a2 * c2.0, s.a1 * c1.1 + s.a2 * c2.1);
                    libm::atan2(n.1, n.0) - libm::atan2(d.1, d.0)
                };
                let diff = ph(half) + ph(twin) - ph(allpass);
                let wrapped = libm::atan2(libm::sin(diff), libm::cos(diff));
                assert!(wrapped.abs() < 1e-9, "{gain} dB at {f} Hz: phase {wrapped}");
            }
            assert!((shelf(0.0) - 1.0).abs() < 1e-4);
            assert!((20.0 * libm::log10(shelf(23_999.0)) - gain).abs() < 0.05);
        }
    }

    #[test]
    fn resonant_high_pass_uses_the_decay_quality_convention() {
        let fs = 48_000.0;
        let q = core::f64::consts::FRAC_1_SQRT_2;
        let s = resonant_high_pass(1000.0, q, fs);
        let expected = 1.0 / (2.0 * libm::sinh(1.0 / (2.0 * q)));
        assert!((magnitude(s, 1000.0, fs) - expected).abs() < 1e-12);
    }
}
