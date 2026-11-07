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

/// A fixed-capacity static cascade returned by [`butterworth_shelf`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cascade {
    pub sections: [Section; 10],
    pub count: usize,
}

/// A bilinear-prewarped Butterworth low- or high-pass of integer order.
pub fn butterworth(high: bool, order: u32, f0: f64, fs: f64) -> Cascade {
    let mut out = Cascade { sections: [Section::IDENTITY; 10], count: 0 };
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

fn bilinear_factor(num: [f64; 3], den: [f64; 3], k: f64) -> Section {
    let transform = |c: [f64; 3]| {
        if c[0] == 0.0 {
            let c1 = c[1] / k;
            return [c1 + c[2], -c1 + c[2], 0.0];
        }
        let c2 = c[0] / (k * k);
        let c1 = c[1] / k;
        [c2 + c1 + c[2], -2.0 * c2 + 2.0 * c[2], c2 - c1 + c[2]]
    };
    let b = transform(num);
    let a = transform(den);
    Section { b0: b[0] / a[0], b1: b[1] / a[0], b2: b[2] / a[0], a1: a[1] / a[0], a2: a[2] / a[0] }
}

fn shelf_factor(high: bool, boost: bool, a: f64, q2: Option<f64>, k: f64) -> Section {
    let (mut num, mut den) = match (high, q2) {
        (true, Some(q)) => ([a * a, q * a, 1.0], [1.0 / (a * a), q / a, 1.0]),
        (false, Some(q)) => ([1.0, q * a, a * a], [1.0, q / a, 1.0 / (a * a)]),
        (true, None) => ([0.0, a, 1.0], [0.0, 1.0 / a, 1.0]),
        (false, None) => ([0.0, 1.0, a], [0.0, 1.0, 1.0 / a]),
    };
    if boost {
        core::mem::swap(&mut num, &mut den);
    }
    bilinear_factor(num, den, k)
}

fn integer_shelf(high: bool, order: u32, gain_db: f64, fm: f64, fs: f64) -> Cascade {
    let mut out = Cascade { sections: [Section::IDENTITY; 10], count: 0 };
    if gain_db == 0.0 {
        return out;
    }
    let boost = gain_db > 0.0;
    let g = libm::pow(10.0, -gain_db.abs() / 20.0);
    let a = libm::pow(g, 1.0 / (2.0 * order as f64));
    let k = libm::tan(PI * fm / fs);
    for i in 0..order / 2 {
        let q2 = 2.0 * libm::sin(PI * (2 * i + 1) as f64 / (2 * order) as f64);
        out.sections[out.count] = shelf_factor(high, boost, a, Some(q2), k);
        out.count += 1;
    }
    if order % 2 != 0 {
        out.sections[out.count] = shelf_factor(high, boost, a, None, k);
        out.count += 1;
    }
    out
}

/// ToneMeister's type-2 Butterworth shelf. The two frequencies determine
/// direction, midpoint and the fractional order; at most five sections are
/// returned because the box caps the order at ten.
pub fn butterworth_shelf(gain_db: f64, f3: f64, f4: f64, max_order: u32, fs: f64) -> Cascade {
    if gain_db == 0.0 || f3 == f4 {
        return Cascade { sections: [Section::IDENTITY; 10], count: 0 };
    }
    let high = f3 > f4;
    let fm = libm::sqrt(f3 * f4);
    let octaves = libm::log2(f3.max(f4) / f3.min(f4));
    let n = (gain_db.abs() / (3.010_299_956_64 * octaves)).max(1.0).min(max_order.clamp(1, 10) as f64);
    if n == 1.0 {
        return integer_shelf(high, 1, gain_db, fm, fs);
    }

    let mut out = Cascade { sections: [Section::IDENTITY; 10], count: 0 };
    if n < 2.0 {
        let fraction = n - 1.0;
        let shape = n - 0.4 * fraction * (1.0 - fraction);
        let boost = gain_db > 0.0;
        let g = libm::pow(10.0, -gain_db.abs() / 20.0);
        let a = libm::pow(g, 0.25);
        let q2 = 2.0 * libm::sin(PI / (2.0 * shape));
        let k = libm::tan(PI * fm / fs);
        out.sections[0] = shelf_factor(high, boost, a, Some(q2), k);
        out.count = 1;
        return out;
    }

    // f64::fract and floor are std-only; the core builds without std
    let fraction = n - libm::trunc(n);
    let easing = (0.6 - 0.04 * n).max(0.2);
    let shape = n - easing * fraction * (1.0 - fraction);
    let pairs = libm::floor(n / 2.0) as u32;
    let boost = gain_db > 0.0;
    let g = libm::pow(10.0, -gain_db.abs() / 20.0);
    let a = libm::pow(g, 1.0 / (2.0 * shape));
    let k = libm::tan(PI * fm / fs);
    for i in 0..pairs {
        let q2 = 2.0 * libm::sin(PI * (2 * i + 1) as f64 / (2.0 * shape));
        out.sections[out.count] = shelf_factor(high, boost, a, Some(q2), k);
        out.count += 1;
    }
    let used = 2.0 * pairs as f64 / shape;
    let remainder_db = gain_db * (1.0 - used);
    if remainder_db.abs() > 1e-12 {
        let ar = libm::pow(10.0, -remainder_db.abs() / 40.0);
        out.sections[out.count] = shelf_factor(high, remainder_db > 0.0, ar, None, k);
        out.count += 1;
    }
    out
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
