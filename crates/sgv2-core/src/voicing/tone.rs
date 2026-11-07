// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Static sections used by the ToneMeister pre-stage and voicing box.
//!
//! Peaks and ordinary shelves use the matched-Z transform with the
//! two-zero magnitude correction described by Gunness and Chauhan,
//! "Optimizing the Magnitude Response of Matched Z Transform Filters".
//! The poles of a boost are the analog prototype's poles mapped by
//! `z = exp(sT)`. Its minimum-phase numerator is the real second-order FIR
//! whose magnitude matches the analog target at DC, one-third Nyquist and
//! two-thirds Nyquist. A peak cut is the exact inverse of its complementary
//! boost; a shelf cut, low or high, is designed directly, the same way as a
//! boost (black-box: the unit's exact coefficients, read back from 300
//! single-slot impulse renders, carry the cut's own poles; the inverse of
//! the boost misses a high-shelf cut by 0.003, 0.018 and 0.034 dB at -6, -12
//! and -15 dB). A 0 dB low shelf is no filter at all. This avoids the
//! high-frequency cramping of a bilinear biquad.
//!
//! The peak is computed in the unit's own Float32 order. Its exact Float32
//! coefficients were read back from single-slot impulse renders of the
//! installed unit (the cascade step is known exactly, so an impulse response
//! fixes every coefficient) over 300 peaks (gains -15..15 dB, Q 0.5..8,
//! 100 Hz..12 kHz, plus 400 random and every extracted tuning's peaks): the
//! pole pair comes from a Float32 `A = exp(g ln10/40)`, the angular frequency
//! rounded once to Float32 and Float32 exponentials; a cut inverts the boost
//! before its numerator is rounded to Float32.

use crate::prestage::Section;
use core::f64::consts::PI;

#[derive(Clone, Copy)]
enum Shape {
    LowShelf,
    HighShelf,
}

#[cfg(test)]
fn inverse(s: Section) -> Section {
    Section { b0: 1.0 / s.b0, b1: s.a1 / s.b0, b2: s.a2 / s.b0, a1: s.b1 / s.b0, a2: s.b2 / s.b0 }
}

/// The denominator obtained by mapping the roots of
/// `c2*s^2 + c1*w0*s + c0*w0^2` with `z = exp(s/fs)`.
fn matched_den(c: [f64; 3], f0: f64, fs: f64) -> [f64; 2] {
    let d = c[1] / (2.0 * c[0]);
    let r2 = c[2] / c[0];
    let w = 2.0 * PI * f0 / fs;
    if d * d <= r2 {
        let radius = libm::exp(-d * w);
        let angle = libm::sqrt((r2 - d * d).max(0.0)) * w;
        [-2.0 * radius * libm::cos(angle), radius * radius]
    } else {
        let spread = libm::sqrt(d * d - r2);
        let p0 = libm::exp(-(d - spread) * w);
        let p1 = libm::exp(-(d + spread) * w);
        [-(p0 + p1), p0 * p1]
    }
}

/// A Float32 elementary function evaluated in f64 and rounded once (the
/// correctly rounded result in all but vanishingly rare halfway cases).
fn exp32(x: f32) -> f32 {
    libm::exp(f64::from(x)) as f32
}

fn cos32(x: f32) -> f32 {
    libm::cos(f64::from(x)) as f32
}

/// `A = 10^(g/40)` as the unit forms it: one Float32 exponential.
fn peak_amplitude(gain_db: f64) -> f32 {
    exp32((gain_db as f32) * 0.057_564_627)
}

/// The boost peak's pole pair, mapped in Float32 as the unit maps it: the
/// prototype `s^2 + s/(A Q) + 1`, its roots scaled by `w = 2 pi f/fs`
/// (rounded once) and mapped by `exp`. An overdamped prototype keeps its two
/// real roots.
fn peak_den(gain_db: f64, f0: f64, q: f64, fs: f64) -> [f64; 2] {
    let a = peak_amplitude(gain_db);
    let damping = 1.0 / (2.0 * a * q as f32);
    let w = (2.0 * PI * f0 / fs) as f32;
    let dw = damping * w;
    let a2 = exp32(-2.0 * dw);
    if damping > 1.0 {
        let spread = libm::sqrtf(damping * damping - 1.0);
        let p0 = exp32(-(damping - spread) * w);
        let p1 = exp32(-(damping + spread) * w);
        [f64::from(-(p0 + p1)), f64::from(a2)]
    } else {
        let radius = exp32(-dw);
        let angle = libm::sqrtf(libm::fmaf(-damping, damping, 1.0)) * w;
        [f64::from(-2.0 * (radius * cos32(angle))), f64::from(a2)]
    }
}

/// The Arm Float32 reciprocal-square-root estimate for a positive normal
/// (Arm ARM `FPRSqrtEstimate`: exponent parity and the leading eight
/// significand bits).
fn arm_rsqrt_estimate(value: f32) -> f32 {
    let bits = value.to_bits();
    let exponent = ((bits >> 23) & 0xff) as i32;
    let fraction = bits & 0x7f_ffff;
    let mut scaled = if exponent & 1 != 0 { 0x80 | (fraction >> 16) } else { 0x100 | (fraction >> 15) };
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
    f32::from_bits(((result_exponent as u32) << 23) | ((estimate & 0xff) << 15))
}

/// A Float32 vector magnitude: fused power sum, reciprocal-square-root
/// estimate and two Newton refinement steps.
fn platform_magnitude(real: f32, imaginary: f32) -> f32 {
    let power = libm::fmaf(imaginary, imaginary, real * real);
    let exponent = power.to_bits() >> 23 & 0xff;
    if power <= 0.0 || !power.is_finite() || exponent == 0 {
        return libm::sqrtf(power);
    }
    let mut reciprocal = arm_rsqrt_estimate(power);
    let mut product = power * reciprocal;
    let step = libm::fmaf(-reciprocal, product, 3.0) * 0.5;
    reciprocal *= step;
    product = power * reciprocal;
    let step = libm::fmaf(-reciprocal, product, 3.0) * 0.5;
    product * step
}

fn polynomial_power(c: [f64; 3], x: f64) -> f64 {
    let real = c[2] - c[0] * x * x;
    let imag = c[1] * x;
    real * real + imag * imag
}

fn denominator_magnitude(a: [f64; 2], w: f64) -> f64 {
    let c1 = libm::cos(w);
    let s1 = libm::sin(w);
    let c2 = libm::cos(2.0 * w);
    let s2 = libm::sin(2.0 * w);
    let (re, im) = (1.0 + a[0] * c1 + a[1] * c2, a[0] * s1 + a[1] * s2);
    libm::sqrt(re * re + im * im)
}

/// Minimum-phase real FIR of order two from its magnitudes at DC,
/// one-third Nyquist and two-thirds Nyquist.
fn curve_fit(m: [f64; 3]) -> [f64; 3] {
    let h0 = m[0] * m[0];
    let h1 = m[1] * m[1];
    let h2 = m[2] * m[2];
    let y = h1 - h2;
    let x = (h0 + 2.0 * h2) / 3.0;
    let z = (2.0 * h0 + h2) / 3.0 - h1;
    let xz = x + z;
    let u = (xz + libm::sqrt((xz * xz - y * y).max(0.0))) / 2.0;
    let sum = libm::sqrt(u.max(0.0));
    let b1 = y / (2.0 * sum);
    let difference = libm::sqrt((sum * sum - 2.0 * z).max(0.0));
    [(sum + difference) / 2.0, b1, (sum - difference) / 2.0]
}

fn peak_curve_fit(m: [f64; 3]) -> [f64; 3] {
    let [h0, h1, h2] = m;
    let first = h0 * h0 - 2.0 * h1 * h1 + 2.0 * h2 * h2;
    let first_root = libm::sqrtf(first.abs() as f32);
    let b1 = ((h0 as f32 - first_root) * 0.5) as f64;
    let second = -3.0 * h0 * h0 + 12.0 * h1 * h1 - 6.0 * h0 * b1 - 3.0 * b1 * b1;
    let second_root = libm::sqrtf(second.abs() as f32);
    let b2 = (((3.0 * (h0 - b1)) as f32 - second_root) / 6.0) as f64;
    [h0 - b1 - b2, b1, b2]
}

fn peak_analog_magnitude(x: f32, gain: f32, q: f32) -> f64 {
    // Evaluate (s^2 + gain*s/q + 1) / (s^2 + s/q + 1) at s=j*x in the
    // box's Float32 complex arithmetic: the real part 1 - x^2 rounds its
    // product; the quotient is the platform split-complex division, which
    // fuses the first product of the squared divisor and of each output
    // part, rounds the second, and scales both by one reciprocal.
    let real = 1.0 - x * x;
    let num_imag = x * (gain / q);
    let den_imag = x * (1.0 / q);
    let inverse = 1.0 / libm::fmaf(real, real, den_imag * den_imag);
    let out_real = inverse * libm::fmaf(real, real, num_imag * den_imag);
    let out_imag = inverse * libm::fmaf(num_imag, real, -(den_imag * real));
    platform_magnitude(out_real, out_imag) as f64
}

fn peak_magnitudes(gain_db: f64, f0: f64, q: f64, fs: f64, den_z: [f64; 2]) -> [f64; 3] {
    let gain = exp32((gain_db as f32) * 0.115_129_255);
    let q = (q as f32) * peak_amplitude(gain_db);
    let w0 = 2.0 * PI * f0 / fs;
    let x1 = ((PI / 3.0) / w0) as f32;
    let x2 = 2.0 * x1;
    let d0 = (1.0 + den_z[0] + den_z[1]).abs();
    let d1 =
        libm::sqrt((1.0 + den_z[0] - den_z[1] + den_z[0] * den_z[0] + den_z[0] * den_z[1] + den_z[1] * den_z[1]).abs());
    let d2 =
        libm::sqrt((1.0 - den_z[0] - den_z[1] + den_z[0] * den_z[0] - den_z[0] * den_z[1] + den_z[1] * den_z[1]).abs());
    [d0, peak_analog_magnitude(x1, gain, q) * d1, peak_analog_magnitude(x2, gain, q) * d2]
}

/// An ordinary shelf's pole pair, mapped in Float32 as the unit maps it:
/// the prototype `s^2 + s/Q + 1` at the shelf's natural frequency (`w`
/// scaled by `1/sqrt(A)` for a low shelf, `sqrt(A)` for a high shelf, with
/// `A` one Float32 exponential), its roots mapped by `exp`. An overdamped
/// prototype keeps its two real roots.
fn shelf_den(shape: Shape, gain_db: f64, f0: f64, q: f64, fs: f64) -> [f64; 2] {
    let a = peak_amplitude(gain_db);
    let root = libm::sqrtf(a);
    let natural = match shape {
        Shape::LowShelf => 1.0 / root,
        Shape::HighShelf => root,
    };
    let damping = 1.0 / (2.0 * q as f32);
    let w = natural * (2.0 * PI * f0 / fs) as f32;
    let dw = damping * w;
    let a2 = exp32(-2.0 * dw);
    if damping >= 1.0 {
        let spread = libm::sqrtf(damping * damping - 1.0);
        let p0 = exp32(-((damping - spread) * w));
        let p1 = exp32(-((damping + spread) * w));
        [f64::from(-(p0 + p1)), f64::from(a2)]
    } else {
        let radius = exp32(-dw);
        let angle = libm::sqrtf(1.0 - damping * damping) * w;
        [f64::from(-2.0 * (radius * cos32(angle))), f64::from(a2)]
    }
}

fn matched(shape: Shape, gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    if gain_db == 0.0 && matches!(shape, Shape::LowShelf) {
        return Section::IDENTITY;
    }

    let a = libm::pow(10.0, gain_db / 40.0);
    let root_a = libm::sqrt(a);
    let (num, den, scale) = match shape {
        // A * N/D gives A^2 at DC and unity at high frequency.
        Shape::LowShelf => ([1.0, root_a / q, a], [a, root_a / q, 1.0], a),
        // A * N/D gives unity at DC and A^2 at high frequency.
        Shape::HighShelf => ([a, root_a / q, 1.0], [1.0, root_a / q, a], a),
    };
    let den_z = shelf_den(shape, gain_db, f0, q, fs);
    let mut magnitude = [0.0; 3];
    for (i, w) in [0.0, PI / 3.0, 2.0 * PI / 3.0].into_iter().enumerate() {
        let x = if w == 0.0 { 0.0 } else { w * fs / (2.0 * PI * f0) };
        let target = scale * libm::sqrt(polynomial_power(num, x) / polynomial_power(den, x));
        magnitude[i] = target * denominator_magnitude(den_z, w);
    }
    let b = curve_fit(magnitude);
    Section { b0: b[0], b1: b[1], b2: b[2], a1: den_z[0], a2: den_z[1] }
}

fn round32(s: Section) -> Section {
    Section {
        b0: s.b0 as f32 as f64,
        b1: s.b1 as f32 as f64,
        b2: s.b2 as f32 as f64,
        a1: s.a1 as f32 as f64,
        a2: s.a2 as f32 as f64,
    }
}

/// A decramped peaking section, as the unit's Float32 coefficients.
///
/// A cut inverts the complementary boost before its numerator is rounded:
/// the boost's `b0` enters the inverse unrounded (black-box: about twice as
/// many cuts exact as when the rounded boost is inverted).
pub fn peak(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    let g = gain_db.abs();
    let den = peak_den(g, f0, q, fs);
    let b = peak_curve_fit(peak_magnitudes(g, f0, q, fs, den));
    round32(if gain_db < 0.0 {
        Section { b0: 1.0 / b[0], b1: den[0] / b[0], b2: den[1] / b[0], a1: b[1] / b[0], a2: b[2] / b[0] }
    } else {
        Section { b0: b[0], b1: b[1], b2: b[2], a1: den[0], a2: den[1] }
    })
}

/// A decramped low-shelf section.
pub fn low_shelf(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    matched(Shape::LowShelf, gain_db, f0, q, fs)
}

/// A decramped high-shelf section.
pub fn high_shelf(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    matched(Shape::HighShelf, gain_db, f0, q, fs)
}

/// ToneMeister's phase-compensated high shelf and its companion allpass.
///
/// A channel selected by the slot's filter mask runs the shelf; a channel in
/// the separate phase mask runs the allpass.  The shelf is the square of a
/// half-gain shelf: its analog zeros and poles sit a quarter of the gain
/// ratio either side of the slot frequency (`A^(-1/2) s^2 + A^(-1/4) s / Q +
/// 1` over `A^(1/2) s^2 + A^(1/4) s / Q + 1` for a cut, the reverse for a
/// boost), the poles are mapped by matched Z and the numerator is
/// magnitude-corrected at the same three frequencies as [`high_shelf`].  The
/// shelf runs that half-gain section followed by its numerator-reversed twin,
/// so its magnitude is the half-gain magnitude squared and its phase is
/// exactly that of the allpass built on the same poles (black-box: the unit's
/// shelf impulse is fourth order with a doubled pole pair and a zero pair
/// mirrored about the unit circle, and the companion's poles are the half
/// shelf's; cuts and boosts of 2-6 dB at Q 0.7-2 match to <= 0.01 dB).
pub fn phase_compensated_high_shelf(gain_db: f64, f0: f64, q: f64, fs: f64) -> ([Section; 2], Section) {
    if gain_db == 0.0 {
        return ([Section::IDENTITY; 2], Section::IDENTITY);
    }
    let a = libm::pow(10.0, gain_db.abs() / 40.0);
    let (num_power, den_power) = if gain_db < 0.0 { (-0.5, 0.5) } else { (0.5, -0.5) };
    let prototype = |power: f64| {
        let leading = libm::pow(a, power);
        [leading, libm::sqrt(leading) / q, 1.0]
    };
    let num = prototype(num_power);
    let den = prototype(den_power);
    let den_z = matched_den(den, f0, fs);
    let mut magnitude = [0.0; 3];
    for (i, w) in [0.0, PI / 3.0, 2.0 * PI / 3.0].into_iter().enumerate() {
        let x = if w == 0.0 { 0.0 } else { w * fs / (2.0 * PI * f0) };
        let target = libm::sqrt(polynomial_power(num, x) / polynomial_power(den, x));
        magnitude[i] = target * denominator_magnitude(den_z, w);
    }
    let b = curve_fit(magnitude);
    let half = Section { b0: b[0], b1: b[1], b2: b[2], a1: den_z[0], a2: den_z[1] };
    let twin = Section { b0: b[2], b1: b[1], b2: b[0], a1: den_z[0], a2: den_z[1] };
    let allpass = Section { b0: den_z[1], b1: den_z[0], b2: 1.0, a1: den_z[0], a2: den_z[1] };
    ([half, twin], allpass)
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

/// A bilinear-prewarped Butterworth low- or high-pass of integer order.
pub fn butterworth(high: bool, order: u32, f0: f64, fs: f64) -> Cascade {
    let mut out = Cascade::EMPTY;
    let order = order.clamp(1, 20);
    let k = libm::tan(PI * f0 / fs);
    for i in 0..order / 2 {
        let q2 = 2.0 * libm::sin(PI * (2 * i + 1) as f64 / (2 * order) as f64);
        let n = 1.0 + q2 * k + k * k;
        let (a1, a2) = (2.0 * (k * k - 1.0) / n, (1.0 - q2 * k + k * k) / n);
        let (b0, b1, b2) = if high { (1.0 / n, -2.0 / n, 1.0 / n) } else { (k * k / n, 2.0 * k * k / n, k * k / n) };
        out.sections[out.count] = Section { b0, b1, b2, a1, a2 };
        out.count += 1;
    }
    if order % 2 == 1 {
        let n = 1.0 + k;
        let a1 = (k - 1.0) / n;
        let (b0, b1) = if high { (1.0 / n, -1.0 / n) } else { (k / n, k / n) };
        out.sections[out.count] = Section { b0, b1, b2: 0.0, a1, a2: 0.0 };
        out.count += 1;
    }
    out
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
    out.push(root_section(bilinear_root(k, (-libm::exp10f(t), 0.0)), bilinear_root(k, (-libm::exp10f(-t), 0.0))));

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
    fn peak_poles_are_the_matched_analog_roots_in_float32() {
        let fs = 48_000.0;
        for gain_db in [0.0, 3.5, 6.0, 12.0] {
            let a = libm::pow(10.0, gain_db / 40.0);
            for q in [0.2, 0.4, 0.5, 1.0, 5.0] {
                for f0 in [40.0, 200.0, 8500.0] {
                    let expected = matched_den([1.0, 1.0 / (a * q), 1.0], f0, fs);
                    let actual = peak_den(gain_db, f0, q, fs);
                    for (got, want) in actual.into_iter().zip(expected) {
                        assert!((got - want).abs() < 2e-6, "{gain_db} dB, {f0} Hz, Q {q}: {got} vs {want}");
                        assert_eq!(got, got as f32 as f64);
                    }
                }
            }
        }
    }

    #[test]
    fn a_peak_cut_inverts_the_unrounded_boost() {
        let fs = 48_000.0;
        let (g, f0, q) = (7.25, 1234.5, 2.5);
        let den = peak_den(g, f0, q, fs);
        let b = peak_curve_fit(peak_magnitudes(g, f0, q, fs, den));
        let cut = peak(-g, f0, q, fs);
        assert_eq!(cut.b0, (1.0 / b[0]) as f32 as f64);
        assert_eq!(cut.a1, (b[1] / b[0]) as f32 as f64);
        assert_eq!(cut.b1, (den[0] / b[0]) as f32 as f64);
        // every coefficient is a Float32 value
        for c in [cut.b0, cut.b1, cut.b2, cut.a1, cut.a2] {
            assert_eq!(c, c as f32 as f64);
        }
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
        assert!((20.0 * libm::log10(magnitude(low, 0.0, fs)) - 6.0).abs() < 1e-9);
        assert!(20.0 * libm::log10(magnitude(low, 23_999.0, fs)) < 0.01);
        assert!(20.0 * libm::log10(magnitude(high, 0.0, fs)).abs() < 1e-9);
        assert!((20.0 * libm::log10(magnitude(high, 23_999.0, fs)) - 6.0).abs() < 0.01);
        // a high-shelf cut is designed directly, not as the inverse boost
        let cut = high_shelf(-12.0, 6000.0, core::f64::consts::FRAC_1_SQRT_2, fs);
        assert!(20.0 * libm::log10(magnitude(cut, 0.0, fs)).abs() < 1e-9);
        assert!((20.0 * libm::log10(magnitude(cut, 16_000.0, fs)) + 12.0).abs() < 0.5);
        let inverse_boost = inverse(high_shelf(12.0, 6000.0, core::f64::consts::FRAC_1_SQRT_2, fs));
        assert!((magnitude(cut, 4000.0, fs) - magnitude(inverse_boost, 4000.0, fs)).abs() > 1e-4);
        // so is a low-shelf cut: it keeps its own poles
        let low_cut = low_shelf(-9.0, 300.0, 0.8, fs);
        let low_inverse = inverse(low_shelf(9.0, 300.0, 0.8, fs));
        assert!((20.0 * libm::log10(magnitude(low_cut, 0.0, fs)) + 9.0).abs() < 1e-9);
        assert_ne!((low_cut.a1, low_cut.a2), (low_inverse.a1, low_inverse.a2));
        // a 0 dB low shelf is no filter; a 0 dB high shelf still is one
        assert_eq!(low_shelf(0.0, 300.0, 0.8, fs), Section::IDENTITY);
        assert_ne!(high_shelf(0.0, 300.0, 0.8, fs).a1, 0.0);
    }

    #[test]
    fn shelf_poles_are_the_scaled_prototype_roots_in_float32() {
        // Synthetic shelf, not any Mac's: the poles sit at the shelf's
        // natural frequency f0/sqrt(A) (low) or f0*sqrt(A) (high).
        let fs = 48_000.0;
        for (shelf, natural) in
            [(low_shelf as fn(f64, f64, f64, f64) -> Section, 500.0 / 2.0), (high_shelf, 500.0 * 2.0)]
        {
            let s = shelf(24.082_4, 500.0, 0.6, fs);
            let w = 2.0 * PI * natural / fs;
            let (d, root) = (1.0 / 1.2, libm::sqrt(1.0 - 1.0 / 1.44));
            let r = libm::exp(-d * w);
            assert!((s.a2 - r * r).abs() < 1e-6, "{} {}", s.a2, r * r);
            assert!((s.a1 + 2.0 * r * libm::cos(root * w)).abs() < 1e-6);
            assert_eq!(s.a1, s.a1 as f32 as f64);
        }
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
            assert!((shelf(0.0) - 1.0).abs() < 1e-9);
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
