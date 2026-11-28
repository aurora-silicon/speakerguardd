// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! A small mixed-radix complex FFT (radices 2, 3, 4 and 5), for the
//! microphone chain's short-time transform.
//!
//! Decimation in time: the input is permuted once by mixed-radix digit
//! reversal, then each stage combines `p` sub-transforms of length `m` into
//! one of length `p m` in place, with a direct `p`-point DFT per butterfly.
//! Every table is made in [`Fft::new`]; [`Fft::forward`] and
//! [`Fft::inverse`] do not allocate. Arithmetic is f64.
//!
//! Forward is unscaled (`X[k] = sum x[n] e^{-2 pi i k n / N}`); inverse
//! scales by `1/N`, so `inverse(forward(x)) == x`.

use alloc::vec;
use alloc::vec::Vec;

/// Largest transform the chain builds (a 1024-bin analysis frame).
pub const MAX_SIZE: usize = 4096;
const MAX_STAGES: usize = 16;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub const ZERO: Complex = Complex { re: 0.0, im: 0.0 };
    #[inline]
    pub fn new(re: f64, im: f64) -> Complex {
        Complex { re, im }
    }
    #[inline]
    pub fn times(self, o: Complex) -> Complex {
        Complex { re: self.re * o.re - self.im * o.im, im: self.re * o.im + self.im * o.re }
    }
    #[inline]
    pub fn plus(self, o: Complex) -> Complex {
        Complex { re: self.re + o.re, im: self.im + o.im }
    }
    #[inline]
    pub fn conj(self) -> Complex {
        Complex { re: self.re, im: -self.im }
    }
}

#[derive(Clone, Debug)]
pub struct Fft {
    n: usize,
    /// Radices, outermost first.
    factors: [usize; MAX_STAGES],
    stages: usize,
    /// `perm[i]`: the input index that lands at position `i`.
    perm: Vec<usize>,
    /// `e^{-2 pi i j / N}` for `j` in `0..N`.
    twiddle: Vec<Complex>,
    scratch: Vec<Complex>,
}

impl Fft {
    /// `None` unless `n` is in `1..=MAX_SIZE` and has no prime factor above 5.
    pub fn new(n: usize) -> Option<Fft> {
        if n == 0 || n > MAX_SIZE {
            return None;
        }
        let mut factors = [0usize; MAX_STAGES];
        let mut stages = 0;
        let mut rest = n;
        for p in [4usize, 2, 3, 5] {
            while rest % p == 0 {
                if stages == MAX_STAGES {
                    return None;
                }
                factors[stages] = p;
                stages += 1;
                rest /= p;
            }
        }
        if rest != 1 {
            return None;
        }
        // digit reversal: n = r0 + p0 (r1 + p1 (r2 + ...)) lands at
        // r0 M0 + r1 M1 + ..., M_i = N / (p0 ... p_i)
        let mut perm = vec![0usize; n];
        for src in 0..n {
            let (mut v, mut m, mut pos) = (src, n, 0usize);
            for &p in &factors[..stages] {
                m /= p;
                pos += (v % p) * m;
                v /= p;
            }
            perm[pos] = src;
        }
        let twiddle = (0..n)
            .map(|j| {
                let a = -2.0 * core::f64::consts::PI * j as f64 / n as f64;
                Complex::new(libm::cos(a), libm::sin(a))
            })
            .collect();
        Some(Fft { n, factors, stages, perm, twiddle, scratch: vec![Complex::ZERO; n] })
    }

    pub fn len(&self) -> usize {
        self.n
    }

    pub fn is_empty(&self) -> bool {
        self.n == 0
    }

    /// In place, unscaled. `data.len()` must be [`Fft::len`]; a shorter
    /// slice is left untouched.
    pub fn forward(&mut self, data: &mut [Complex]) {
        self.run(data, false);
    }

    /// In place, scaled by `1/N`.
    pub fn inverse(&mut self, data: &mut [Complex]) {
        self.run(data, true);
        let s = 1.0 / self.n as f64;
        for v in data.iter_mut().take(self.n) {
            v.re *= s;
            v.im *= s;
        }
    }

    fn run(&mut self, data: &mut [Complex], inverse: bool) {
        let n = self.n;
        if data.len() < n {
            return;
        }
        for (i, &src) in self.perm.iter().enumerate() {
            self.scratch[i] = data[src];
        }
        data[..n].copy_from_slice(&self.scratch);
        let tw = |j: usize| -> Complex {
            let t = self.twiddle[j % n];
            if inverse {
                t.conj()
            } else {
                t
            }
        };
        // innermost stage first: sub-transform length m grows to p m
        let mut m = 1usize;
        for s in (0..self.stages).rev() {
            let p = self.factors[s];
            let l = p * m;
            let step = n / l;
            let mut t = [Complex::ZERO; 5];
            let mut block = 0;
            while block < n {
                for k in 0..m {
                    for (q, tq) in t.iter_mut().enumerate().take(p) {
                        *tq = data[block + q * m + k].times(tw(q * k * step));
                    }
                    for u in 0..p {
                        let mut acc = Complex::ZERO;
                        for (q, tq) in t.iter().enumerate().take(p) {
                            acc = acc.plus(tq.times(tw((q * u % p) * (n / p))));
                        }
                        data[block + k + m * u] = acc;
                    }
                }
                block += l;
            }
            m = l;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dft(x: &[Complex]) -> Vec<Complex> {
        let n = x.len();
        (0..n)
            .map(|k| {
                let mut acc = Complex::ZERO;
                for (j, v) in x.iter().enumerate() {
                    let a = -2.0 * core::f64::consts::PI * ((j * k) % n) as f64 / n as f64;
                    acc = acc.plus(v.times(Complex::new(libm::cos(a), libm::sin(a))));
                }
                acc
            })
            .collect()
    }

    #[test]
    fn matches_the_direct_dft_for_mixed_sizes() {
        for n in [1usize, 2, 3, 4, 5, 8, 12, 15, 60, 96, 120, 960] {
            let mut f = Fft::new(n).unwrap();
            let x: Vec<Complex> =
                (0..n).map(|i| Complex::new(libm::sin(i as f64 * 0.37 + 0.1), libm::cos(i as f64 * 1.3))).collect();
            let want = dft(&x);
            let mut got = x.clone();
            f.forward(&mut got);
            for (a, b) in got.iter().zip(&want) {
                assert!((a.re - b.re).abs() < 1e-9 * n as f64 && (a.im - b.im).abs() < 1e-9 * n as f64, "n={n}");
            }
            f.inverse(&mut got);
            for (a, b) in got.iter().zip(&x) {
                assert!((a.re - b.re).abs() < 1e-12 && (a.im - b.im).abs() < 1e-12, "n={n}");
            }
        }
    }

    #[test]
    fn refuses_other_primes_and_bad_sizes() {
        assert!(Fft::new(0).is_none());
        assert!(Fft::new(7).is_none());
        assert!(Fft::new(962).is_none());
        assert!(Fft::new(MAX_SIZE * 2).is_none());
    }
}
