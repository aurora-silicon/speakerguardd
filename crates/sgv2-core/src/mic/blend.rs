// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The adaptive low-frequency blend between a beam and a reference
//! spectrum: the law the beamformer's adaptive path and the standalone
//! three-input unit (`aufx/lwbo`: beam, comparison reference, blend
//! reference) share. Per frame, `k` a bin at frequency `f_k`:
//!
//! **Boost limiting** (bins under `cutoff_hz`, when enabled): the amplitude
//! ratio `r = sqrt(P_beam / P_compare)`; bins where either power is under
//! the absolute floor are left alone; where `20 log10 r` exceeds
//! `detect_db` the beam is scaled by `10^(max_boost_db/20) / r`, never under
//! `10^(scale_floor_db/20)`. The scale runs through an attack/release
//! smoother (`wind_attack_s` while it falls, `wind_release_s` while it
//! rises) and a second smoother (`scale_smooth_s`).
//!
//! **Crossover.** The target is the highest bin frequency at or under
//! `xover_max_hz` whose beam-to-reference amplitude ratio (blend reference)
//! is under `threshold_db`, among bins whose powers are over the absolute
//! floor; `xover_min_hz` when there is none; clamped to
//! `[xover_min_hz, xover_max_hz]`. A rising target is followed with
//! `attack_s`; a falling one is held `hold_s`, then followed with
//! `release_s`; a second smoother (`xover_smooth_s`) gives the crossover
//! `fc`.
//!
//! **Crossfade.** Complementary and linear in the spectrum: the beam's
//! weight is `clamp((f_k - fc) / (slope fc) + 1/2, 0, 1)` (a hard switch at
//! `fc`, beam at and above it, when `slope fc` is 0), the reference's its
//! complement. All smoothers use `exp(-hop / (Fs tau))`.
//!
//! TODO(SPEC): the clean summary gives this law's parts but not their
//! exact arithmetic (the target search, the hold and smoother order, the
//! crossfade's width, how the reference is chosen); there are no
//! black-box renders of the standalone unit yet. Every rule above is the
//! simplest reading of the summary, not a measured fit.

use super::fft::Complex;
use alloc::vec;
use alloc::vec::Vec;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BlendLaw {
    pub boost_limit: bool,
    pub cutoff_hz: f64,
    pub detect_db: f64,
    pub max_boost_db: f64,
    pub scale_floor_db: f64,
    pub scale_smooth_s: f64,
    pub threshold_db: f64,
    pub xover_max_hz: f64,
    pub attack_s: f64,
    pub release_s: f64,
    pub hold_s: f64,
    pub xover_smooth_s: f64,
    pub xover_min_hz: f64,
    /// dB on power.
    pub abs_floor_db: f64,
    pub wind_attack_s: f64,
    pub wind_release_s: f64,
    pub slope: f64,
}

impl BlendLaw {
    pub fn valid(&self, fs: f64) -> bool {
        let t = |v: f64| v.is_finite() && (0.0..=100.0).contains(&v);
        let db = |v: f64| v.is_finite() && (-400.0..=200.0).contains(&v);
        let hz = |v: f64| v.is_finite() && (0.0..=fs / 2.0).contains(&v);
        hz(self.cutoff_hz)
            && db(self.detect_db)
            && db(self.max_boost_db)
            && db(self.scale_floor_db)
            && t(self.scale_smooth_s)
            && db(self.threshold_db)
            && hz(self.xover_max_hz)
            && hz(self.xover_min_hz)
            && self.xover_min_hz <= self.xover_max_hz
            && t(self.attack_s)
            && t(self.release_s)
            && t(self.hold_s)
            && t(self.xover_smooth_s)
            && db(self.abs_floor_db)
            && t(self.wind_attack_s)
            && t(self.wind_release_s)
            && self.slope.is_finite()
            && (0.0..=16.0).contains(&self.slope)
    }
}

fn coef(hop: usize, fs: f64, s: f64) -> f64 {
    if s <= 0.0 {
        0.0
    } else {
        libm::exp(-(hop as f64) / (fs * s))
    }
}

fn pow(c: Complex) -> f64 {
    c.re * c.re + c.im * c.im
}

pub(super) struct Run {
    law: BlendLaw,
    fs: f64,
    hop: usize,
    bin_hz: f64,
    floor: f64,
    /// Boost scale per bin after the attack/release and second smoothers.
    scale_ar: Vec<f64>,
    scale: Vec<f64>,
    target_held: f64,
    hold_left: f64,
    xover_ar: f64,
    xover: f64,
}

impl Run {
    pub(super) fn new(law: &BlendLaw, bins: usize, hop: usize, fs: f64) -> Run {
        Run {
            law: *law,
            fs,
            hop,
            bin_hz: fs / (2.0 * hop as f64),
            floor: libm::pow(10.0, law.abs_floor_db / 10.0),
            scale_ar: vec![1.0; bins],
            scale: vec![1.0; bins],
            target_held: law.xover_min_hz,
            hold_left: 0.0,
            xover_ar: law.xover_min_hz,
            xover: law.xover_min_hz,
        }
    }

    pub(super) fn reset(&mut self) {
        self.scale_ar.fill(1.0);
        self.scale.fill(1.0);
        self.target_held = self.law.xover_min_hz;
        self.hold_left = 0.0;
        self.xover_ar = self.law.xover_min_hz;
        self.xover = self.law.xover_min_hz;
    }

    /// The crossover now, Hz.
    #[cfg(test)]
    pub(super) fn crossover(&self) -> f64 {
        self.xover
    }

    /// `y` (the beam, in place) against `compare` and `blend`. Real time.
    pub(super) fn process(&mut self, y: &mut [Complex], compare: &[Complex], blend: &[Complex]) {
        let l = self.law;
        let (fs, hop) = (self.fs, self.hop);
        if l.boost_limit {
            let (aa, ar) = (coef(hop, fs, l.wind_attack_s), coef(hop, fs, l.wind_release_s));
            let a2 = coef(hop, fs, l.scale_smooth_s);
            let max_boost = libm::pow(10.0, l.max_boost_db / 20.0);
            let floor_scale = libm::pow(10.0, l.scale_floor_db / 20.0);
            for (k, (v, c)) in y.iter_mut().zip(compare).enumerate() {
                let (pb, pc) = (pow(*v), pow(*c));
                let mut target = 1.0;
                if (k as f64) * self.bin_hz < l.cutoff_hz && pb > self.floor && pc > self.floor {
                    let r = libm::sqrt(pb / pc);
                    if 20.0 * libm::log10(r) > l.detect_db {
                        target = (max_boost / r).max(floor_scale);
                    }
                }
                let a = if target < self.scale_ar[k] { aa } else { ar };
                self.scale_ar[k] = a * self.scale_ar[k] + (1.0 - a) * target;
                self.scale[k] = a2 * self.scale[k] + (1.0 - a2) * self.scale_ar[k];
                v.re *= self.scale[k];
                v.im *= self.scale[k];
            }
        }
        // the target crossover
        let thr = libm::pow(10.0, l.threshold_db / 10.0);
        let mut target = l.xover_min_hz;
        for (k, (v, r)) in y.iter().zip(blend).enumerate() {
            let f = k as f64 * self.bin_hz;
            if f > l.xover_max_hz {
                break;
            }
            let (pb, pr) = (pow(*v), pow(*r));
            if pb > self.floor && pr > self.floor && pb < thr * pr {
                target = f;
            }
        }
        let target = target.clamp(l.xover_min_hz, l.xover_max_hz);
        let dt = hop as f64 / fs;
        if target >= self.target_held {
            self.target_held = target;
            self.hold_left = l.hold_s;
        } else if self.hold_left > 0.0 {
            self.hold_left -= dt;
        } else {
            self.target_held = target;
        }
        let a = if self.target_held > self.xover_ar { coef(hop, fs, l.attack_s) } else { coef(hop, fs, l.release_s) };
        self.xover_ar = a * self.xover_ar + (1.0 - a) * self.target_held;
        let a2 = coef(hop, fs, l.xover_smooth_s);
        self.xover = a2 * self.xover + (1.0 - a2) * self.xover_ar;
        let fc = self.xover;
        let width = l.slope * fc;
        for (k, (v, r)) in y.iter_mut().zip(blend).enumerate() {
            let f = k as f64 * self.bin_hz;
            let wb = if width > 0.0 {
                ((f - fc) / width + 0.5).clamp(0.0, 1.0)
            } else if f >= fc {
                1.0
            } else {
                0.0
            };
            if wb < 1.0 {
                v.re = wb * v.re + (1.0 - wb) * r.re;
                v.im = wb * v.im + (1.0 - wb) * r.im;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn law() -> BlendLaw {
        BlendLaw {
            boost_limit: false,
            cutoff_hz: 0.0,
            detect_db: 0.0,
            max_boost_db: 0.0,
            scale_floor_db: -95.0,
            scale_smooth_s: 0.0,
            threshold_db: -14.0,
            xover_max_hz: 1000.0,
            attack_s: 0.0,
            release_s: 0.0,
            hold_s: 0.0,
            xover_smooth_s: 0.0,
            xover_min_hz: 200.0,
            abs_floor_db: -190.0,
            wind_attack_s: 0.0,
            wind_release_s: 0.0,
            slope: 0.0,
        }
    }

    #[test]
    fn a_weak_low_band_beam_is_replaced_by_the_reference() {
        let mut r = Run::new(&law(), 481, 480, 48_000.0);
        // beam 20 dB under the reference up to 500 Hz (bin 10), equal above
        let reference = vec![Complex::new(1.0, 0.0); 481];
        let mut y: Vec<Complex> =
            (0..481).map(|k| if k <= 10 { Complex::new(0.1, 0.0) } else { Complex::new(1.0, 0.0) }).collect();
        r.process(&mut y, &reference, &reference);
        assert_eq!(r.crossover(), 500.0);
        assert!(y[..10].iter().all(|v| v.re == 1.0), "under the crossover: the reference");
        assert_eq!(y[10].re, 0.1, "at the crossover: the beam");
        assert!(y[11..].iter().all(|v| v.re == 1.0));
    }

    #[test]
    fn the_crossover_is_clamped_and_a_zero_range_passes_the_beam() {
        let mut l = law();
        l.xover_min_hz = 0.0;
        l.xover_max_hz = 0.0;
        let mut r = Run::new(&l, 481, 480, 48_000.0);
        let reference = vec![Complex::new(1.0, 0.0); 481];
        let beam = vec![Complex::new(0.01, 0.0); 481];
        let mut y = beam.clone();
        r.process(&mut y, &reference, &reference);
        assert_eq!(y, beam);
    }

    #[test]
    fn boost_limiting_caps_the_beam_at_the_reference() {
        let mut l = law();
        l.boost_limit = true;
        l.cutoff_hz = 4000.0;
        l.xover_max_hz = 0.0;
        l.xover_min_hz = 0.0;
        let mut r = Run::new(&l, 481, 480, 48_000.0);
        let reference = vec![Complex::new(1.0, 0.0); 481];
        let mut y = vec![Complex::new(4.0, 0.0); 481];
        r.process(&mut y, &reference, &reference);
        assert!((y[10].re - 1.0).abs() < 1e-12, "{}", y[10].re);
        assert_eq!(y[100].re, 4.0, "over the cutoff: untouched");
    }
}
