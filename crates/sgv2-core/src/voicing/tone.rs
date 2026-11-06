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
//! two-thirds Nyquist. A cut is the exact inverse of its complementary
//! boost. This avoids the high-frequency cramping of a bilinear biquad.

use crate::prestage::Section;
use core::f64::consts::PI;

#[derive(Clone, Copy)]
enum Shape {
    Peak,
    LowShelf,
    HighShelf,
}

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
    libm::sqrt((1.0 + a[0] * c1 + a[1] * c2).powi(2) + (a[0] * s1 + a[1] * s2).powi(2))
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

fn matched(shape: Shape, gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    if gain_db == 0.0 {
        return Section::IDENTITY;
    }
    if gain_db < 0.0 {
        return inverse(matched(shape, -gain_db, f0, q, fs));
    }

    let a = libm::pow(10.0, gain_db / 40.0);
    let root_a = libm::sqrt(a);
    let (num, den, scale) = match shape {
        // Unity at both asymptotes; A^2 at the centre.
        Shape::Peak => ([1.0, a / q, 1.0], [1.0, 1.0 / (a * q), 1.0], 1.0),
        // A * N/D gives A^2 at DC and unity at high frequency.
        Shape::LowShelf => ([1.0, root_a / q, a], [a, root_a / q, 1.0], a),
        // A * N/D gives unity at DC and A^2 at high frequency.
        Shape::HighShelf => ([a, root_a / q, 1.0], [1.0, root_a / q, a], a),
    };
    let den_z = matched_den(den, f0, fs);
    let mut magnitude = [0.0; 3];
    for (i, w) in [0.0, PI / 3.0, 2.0 * PI / 3.0].into_iter().enumerate() {
        let x = if w == 0.0 { 0.0 } else { w * fs / (2.0 * PI * f0) };
        let target = scale * libm::sqrt(polynomial_power(num, x) / polynomial_power(den, x));
        magnitude[i] = target * denominator_magnitude(den_z, w);
    }
    let b = curve_fit(magnitude);
    Section { b0: b[0], b1: b[1], b2: b[2], a1: den_z[0], a2: den_z[1] }
}

/// A decramped peaking section.
pub fn peak(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    matched(Shape::Peak, gain_db, f0, q, fs)
}

/// A decramped low-shelf section.
pub fn low_shelf(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    matched(Shape::LowShelf, gain_db, f0, q, fs)
}

/// A decramped high-shelf section.
pub fn high_shelf(gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    matched(Shape::HighShelf, gain_db, f0, q, fs)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn magnitude(s: Section, f: f64, fs: f64) -> f64 {
        s.magnitude(f, fs)
    }

    #[test]
    fn complementary_peaks_are_exact_inverses() {
        let fs = 48_000.0;
        let boost = peak(12.0, 18_000.0, 2.0, fs);
        let cut = peak(-12.0, 18_000.0, 2.0, fs);
        for f in [20.0, 1000.0, 8000.0, 16_000.0, 20_000.0, 23_900.0] {
            assert!((magnitude(boost, f, fs) * magnitude(cut, f, fs) - 1.0).abs() < 1e-12);
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
