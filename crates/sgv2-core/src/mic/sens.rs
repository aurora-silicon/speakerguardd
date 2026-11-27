// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The per-microphone noise-floor suppressor (`aufx/sens`).
//!
//! It runs on the graph's blocks of `hop` samples per microphone with its
//! own short-time transform: a frame of `2 hop` samples (the previous block
//! and this one), the power-complementary [`super::Window::Quadrant`]
//! window, `hop + 1` bins, and `hop` samples of latency. Per frame and
//! microphone (`k` a bin):
//!
//! ```text
//! P(k)  = E / (|A(e^{i w_k})|^2 Pw) Burg autoregressive estimate of the
//!                                   windowed frame, order `order`, over
//!                                   the window's mean power Pw (1/2)
//! S(k)  = a_s S(k) + (1 - a_s) P(k)            a_s = exp(-hop/(Fs tau_s))
//! N(k)  = profile_m(k + 1) * multiplier       (the last bin: its own entry)
//! g(k)  = clip(S / (S + N), floor, 1)
//! ```
//!
//! then the artifact step (when enabled), the gain smoother
//! `G = a_g G + (1 - a_g) g` (`a_g = exp(-hop/(Fs tau_g))`), and
//! `Y(k) = X(k) G(k) output`, `X` the windowed frame's spectrum.
//!
//! **Artifact step.** The retained-signal probability of the frame is
//! `p = sum g^2 S / sum S`. The gain is frequency-smoothed by an odd
//! boxcar of `L = 2 min_len - 1` bins, longer when `p` is under the
//! threshold: `L += 2 round(2 scale (threshold - p) / threshold)`. With the
//! aggressive switch the smoothed gain is multiplied by the same boxcar of
//! the local mask `1 / (1 + exp(-MASK_SLOPE (g - threshold)))`. The
//! boxcar averages over the bins it covers at DC (it does not reach below
//! DC) and counts zeros above Nyquist. The result is clipped to
//! `[floor, 1]` again.
//!
//! TODO(SPEC): the clean behaviour summary fixes the law above up to the
//! artifact step's exact shape and the profile's bin alignment. The boxcar
//! lengths, the edge handling, the mask's input and slope
//! ([`MASK_SLOPE`]), the Burg order, its window-power normalisation and the
//! profile's one-bin offset are the best fit to black-box renders of the
//! installed unit (mean gain per bin within about 0.2 dB from -95 to -65
//! dBFS), not a specification.

use super::fft::{Complex, Fft};
use super::Window;
use alloc::vec;
use alloc::vec::Vec;

/// Largest autoregressive order.
pub const MAX_ORDER: usize = 128;
/// The local mask's slope (per unit of gain). Fitted, see the module notes.
pub const MASK_SLOPE: f64 = 15.0;

/// The suppressor's compiled parameters (the strip preset's global scope,
/// IDs 0..13, and the bound noise profile).
#[derive(Clone, Debug, PartialEq)]
pub struct Sens {
    pub mics: usize,
    /// Block, and half the frame.
    pub hop: usize,
    /// ID 0: pass the input through (still `hop` samples late).
    pub bypass: bool,
    /// Autoregressive order of the PSD estimate (TODO(SPEC): fitted).
    pub order: usize,
    /// ID 3, ms.
    pub signal_ms: f64,
    /// ID 4, ms.
    pub gain_ms: f64,
    /// ID 5, dB.
    pub output_db: f64,
    /// ID 6, dB.
    pub floor_db: f64,
    /// ID 7, linear on power.
    pub multiplier: f64,
    /// ID 9.
    pub artifact: bool,
    /// ID 10.
    pub threshold: f64,
    /// ID 11.
    pub scale: f64,
    /// ID 12, bins.
    pub min_len: usize,
    /// ID 13.
    pub aggressive: bool,
    /// Noise power per microphone and bin, `m * bins + k`, in the PSD's
    /// sample-squared normalization.
    pub profile: Vec<f64>,
}

impl Sens {
    pub fn bins(&self) -> usize {
        self.hop + 1
    }

    pub fn valid(&self, mics: usize, fs: f64) -> bool {
        let fin = |v: f64| v.is_finite();
        self.mics == mics
            && (2..=super::MAX_HOP).contains(&self.hop)
            && Fft::new(2 * self.hop).is_some()
            && (1..=MAX_ORDER).contains(&self.order)
            && self.order < 2 * self.hop
            && fin(self.signal_ms)
            && (0.0..=10_000.0).contains(&self.signal_ms)
            && fin(self.gain_ms)
            && (0.0..=10_000.0).contains(&self.gain_ms)
            && fin(self.output_db)
            && self.output_db.abs() <= super::MAX_GAIN_DB
            && fin(self.floor_db)
            && self.floor_db <= 0.0
            && self.floor_db >= -200.0
            && fin(self.multiplier)
            && self.multiplier >= 0.0
            && fin(self.threshold)
            && (0.0..=1.0).contains(&self.threshold)
            && fin(self.scale)
            && (0.0..=1000.0).contains(&self.scale)
            && (1..=self.hop).contains(&self.min_len)
            && self.profile.len() == mics * self.bins()
            && self.profile.iter().all(|v| v.is_finite() && *v >= 0.0)
            && fs > 0.0
    }
}

/// Smoothing coefficient `exp(-hop / (Fs tau))`; 0 for no smoothing.
fn coef(hop: usize, fs: f64, ms: f64) -> f64 {
    if ms <= 0.0 {
        0.0
    } else {
        libm::exp(-(hop as f64) / (fs * ms / 1000.0))
    }
}

/// The running suppressor. Every buffer is made in [`Run::new`].
pub struct Run {
    p: Sens,
    fft: Fft,
    window: Vec<f64>,
    /// The window's mean power.
    wpow: f64,
    a_s: f64,
    a_g: f64,
    floor: f64,
    out: f64,
    /// Per mic, the previous block.
    prev: Vec<Vec<f64>>,
    /// Per mic, the synthesis overlap (second half of the last frame).
    ola: Vec<Vec<f64>>,
    /// Per mic and bin, the signal PSD and the smoothed gain.
    s: Vec<f64>,
    g: Vec<f64>,
    // scratch
    frame: Vec<Complex>,
    poly: Vec<Complex>,
    fwd: Vec<f64>,
    bwd: Vec<f64>,
    a: Vec<f64>,
    a_prev: Vec<f64>,
    raw: Vec<f64>,
    mask: Vec<f64>,
    ext: Vec<f64>,
    tmp: Vec<f64>,
}

impl Run {
    /// `None` if the transform cannot be built. Not real time.
    pub fn new(p: &Sens, fs: f64) -> Option<Run> {
        let n = 2 * p.hop;
        let bins = p.bins();
        Some(Run {
            fft: Fft::new(n)?,
            window: (0..n).map(|i| Window::Quadrant.at(i, n, false)).collect(),
            wpow: (0..n).map(|i| Window::Quadrant.at(i, n, false).powi(2)).sum::<f64>() / n as f64,
            a_s: coef(p.hop, fs, p.signal_ms),
            a_g: coef(p.hop, fs, p.gain_ms),
            floor: libm::pow(10.0, p.floor_db / 20.0),
            out: libm::pow(10.0, p.output_db / 20.0),
            prev: vec![vec![0.0; p.hop]; p.mics],
            ola: vec![vec![0.0; p.hop]; p.mics],
            s: vec![0.0; p.mics * bins],
            g: vec![1.0; p.mics * bins],
            frame: vec![Complex::ZERO; n],
            poly: vec![Complex::ZERO; n],
            fwd: vec![0.0; n],
            bwd: vec![0.0; n],
            a: vec![0.0; p.order + 1],
            a_prev: vec![0.0; p.order + 1],
            raw: vec![0.0; bins],
            mask: vec![0.0; bins],
            ext: vec![0.0; 3 * bins + 2],
            tmp: vec![0.0; bins],
            p: p.clone(),
        })
    }

    pub fn reset(&mut self) {
        self.prev.iter_mut().chain(self.ola.iter_mut()).for_each(|v| v.fill(0.0));
        self.s.fill(0.0);
        self.g.fill(1.0);
    }

    /// The Burg estimate of `fwd` (the windowed frame) into `raw`, over the
    /// window's mean power.
    fn burg(&mut self) {
        let n = self.fwd.len();
        let order = self.p.order;
        self.bwd.copy_from_slice(&self.fwd);
        let mut e = self.fwd.iter().map(|v| v * v).sum::<f64>() / n as f64;
        self.a.fill(0.0);
        self.a[0] = 1.0;
        for m in 0..order {
            let (mut num, mut den) = (0.0, 0.0);
            for (f, b) in self.fwd[m + 1..].iter().zip(&self.bwd[m..n - 1]) {
                num += f * b;
                den += f * f + b * b;
            }
            if den <= 0.0 {
                break;
            }
            let k = -2.0 * num / den;
            // descending, so `bwd[i - 1]` is still the previous stage's
            for i in (m + 1..n).rev() {
                let (f, b) = (self.fwd[i], self.bwd[i - 1]);
                self.fwd[i] = f + k * b;
                self.bwd[i] = b + k * f;
            }
            self.a_prev[..=m + 1].copy_from_slice(&self.a[..=m + 1]);
            for j in 1..=m + 1 {
                self.a[j] = self.a_prev[j] + k * self.a_prev[m + 1 - j];
            }
            e *= 1.0 - k * k;
        }
        self.poly.fill(Complex::ZERO);
        for (c, a) in self.poly.iter_mut().zip(&self.a) {
            c.re = *a;
        }
        self.fft.forward(&mut self.poly);
        for (r, c) in self.raw.iter_mut().zip(&self.poly) {
            let d = c.re * c.re + c.im * c.im;
            *r = if d > 0.0 { e / (d * self.wpow) } else { 0.0 };
        }
    }

    /// `src` boxcar-smoothed over `len` (odd) bins into `dst`: near DC the
    /// mean of the bins the box covers, above Nyquist zeros that count.
    fn boxcar(ext: &mut [f64], src: &[f64], dst: &mut [f64], len: usize) {
        let bins = src.len();
        let h = (len / 2).min(bins - 1);
        // ext: src then h zeros; window of bin k is ext[k-h ..= k+h] cut at 0
        let total = bins + h;
        ext[..bins].copy_from_slice(src);
        ext[bins..total].fill(0.0);
        let mut acc: f64 = ext[..h].iter().sum();
        for (k, d) in dst.iter_mut().enumerate() {
            acc += ext[k + h];
            if k > h {
                acc -= ext[k - h - 1];
            }
            let lo = k.saturating_sub(h);
            *d = acc / (k + h + 1 - lo) as f64;
        }
    }

    /// The gain of one frame for mic `m` from `raw` into `g`.
    fn gain(&mut self, m: usize) {
        let bins = self.p.bins();
        let s = &mut self.s[m * bins..(m + 1) * bins];
        let prof = &self.p.profile[m * bins..(m + 1) * bins];
        // the profile sits one bin up (measured; see the module notes)
        let shifted = prof[1..].iter().chain(&prof[bins - 1..]);
        for ((sv, r), (t, nz)) in s.iter_mut().zip(&self.raw).zip(self.tmp.iter_mut().zip(shifted)) {
            *sv = self.a_s * *sv + (1.0 - self.a_s) * r;
            let noise = nz * self.p.multiplier;
            let d = *sv + noise;
            *t = if d > 0.0 { (*sv / d).clamp(self.floor, 1.0) } else { 1.0 };
        }
        if self.p.artifact {
            let (mut kept, mut all) = (0.0, 0.0);
            for (t, sv) in self.tmp.iter().zip(s.iter()) {
                kept += t * t * sv;
                all += sv;
            }
            let prob = if all > 0.0 { kept / all } else { 1.0 };
            let mut len = 2 * self.p.min_len - 1;
            let thr = self.p.threshold;
            if prob < thr && thr > 0.0 {
                len += 2 * libm::round(2.0 * self.p.scale * (thr - prob) / thr) as usize;
            }
            if self.p.aggressive {
                for (mk, t) in self.mask.iter_mut().zip(&self.tmp) {
                    *mk = 1.0 / (1.0 + libm::exp(-MASK_SLOPE * (t - thr)));
                }
                let mut sm = core::mem::take(&mut self.raw);
                Self::boxcar(&mut self.ext, &self.mask, &mut sm, len);
                self.mask.copy_from_slice(&sm);
                self.raw = sm;
            } else {
                self.mask.fill(1.0);
            }
            let mut sm = core::mem::take(&mut self.raw);
            Self::boxcar(&mut self.ext, &self.tmp, &mut sm, len);
            for ((t, v), mk) in self.tmp.iter_mut().zip(&sm).zip(&self.mask) {
                *t = (v * mk).clamp(self.floor, 1.0);
            }
            self.raw = sm;
        }
        let g = &mut self.g[m * bins..(m + 1) * bins];
        for (gv, t) in g.iter_mut().zip(&self.tmp) {
            *gv = self.a_g * *gv + (1.0 - self.a_g) * t;
        }
    }

    /// One block of `hop` samples per microphone, in place: the output is
    /// the input `hop` samples late, suppressed. Real time.
    pub fn process(&mut self, blocks: &mut [Vec<f32>]) {
        let hop = self.p.hop;
        let n = 2 * hop;
        let bins = self.p.bins();
        for (m, blk) in blocks.iter_mut().enumerate().take(self.p.mics) {
            let blk = &mut blk[..hop];
            if self.p.bypass {
                for (x, p) in blk.iter_mut().zip(self.prev[m].iter_mut()) {
                    let v = f64::from(*x);
                    *x = *p as f32;
                    *p = v;
                }
                continue;
            }
            for (i, (f, w)) in self.fwd.iter_mut().zip(&self.window).enumerate() {
                *f = w * if i < hop { self.prev[m][i] } else { f64::from(blk[i - hop]) };
            }
            for (fr, f) in self.frame.iter_mut().zip(self.fwd.iter()) {
                *fr = Complex::new(*f, 0.0);
            }
            for (p, x) in self.prev[m].iter_mut().zip(blk.iter()) {
                *p = f64::from(*x);
            }
            self.burg();
            self.gain(m);
            self.fft.forward(&mut self.frame);
            let g = &self.g[m * bins..(m + 1) * bins];
            self.frame[0] = Complex::new(self.frame[0].re * g[0] * self.out, 0.0);
            self.frame[hop] = Complex::new(self.frame[hop].re * g[hop] * self.out, 0.0);
            for (k, gk) in g.iter().enumerate().take(hop).skip(1) {
                let v = self.frame[k];
                let s = gk * self.out;
                self.frame[k] = Complex::new(v.re * s, v.im * s);
                self.frame[n - k] = self.frame[k].conj();
            }
            self.fft.inverse(&mut self.frame);
            let ola = &mut self.ola[m];
            for (i, (x, o)) in blk.iter_mut().zip(ola.iter_mut()).enumerate() {
                *x = (*o + self.frame[i].re * self.window[i]) as f32;
                *o = self.frame[i + hop].re * self.window[i + hop];
            }
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) fn params(mics: usize, hop: usize) -> Sens {
        Sens {
            mics,
            hop,
            bypass: false,
            order: 16,
            signal_ms: 12.0,
            gain_ms: 6.0,
            output_db: 0.0,
            floor_db: -10.0,
            multiplier: 1.4,
            artifact: true,
            threshold: 0.35,
            scale: 9.0,
            min_len: 4,
            aggressive: true,
            profile: vec![1e-10; mics * (hop + 1)],
        }
    }

    fn noise(n: usize, amp: f64, seed: u64) -> Vec<f32> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                // sum of 4 uniforms, about unit variance after scaling
                let mut acc = 0.0;
                for _ in 0..4 {
                    s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    acc += (s >> 11) as f64 / (1u64 << 53) as f64 - 0.5;
                }
                (acc * amp * 1.732) as f32
            })
            .collect()
    }

    fn run(p: &Sens, x: &[Vec<f32>]) -> Vec<Vec<f32>> {
        let mut r = Run::new(p, 48_000.0).unwrap();
        let mut y: Vec<Vec<f32>> = x.to_vec();
        let hop = p.hop;
        let mut blocks = vec![vec![0.0f32; hop]; p.mics];
        for at in (0..x[0].len() / hop).map(|b| b * hop) {
            for (b, c) in blocks.iter_mut().zip(x) {
                b.copy_from_slice(&c[at..at + hop]);
            }
            r.process(&mut blocks);
            for (yc, b) in y.iter_mut().zip(&blocks) {
                yc[at..at + hop].copy_from_slice(b);
            }
        }
        y
    }

    fn rms(x: &[f32]) -> f64 {
        (x.iter().map(|v| f64::from(*v).powi(2)).sum::<f64>() / x.len() as f64).sqrt()
    }

    #[test]
    fn the_quadrant_window_is_power_complementary_and_near_vorbis() {
        for n in 0..480 {
            let a = Window::Quadrant.at(n, 960, false);
            let b = Window::Quadrant.at(n + 480, 960, false);
            assert!((a * a + b * b - 1.0).abs() < 1e-12);
            assert!((a - Window::Vorbis.at(n, 960, false)).abs() < 0.01);
        }
    }

    #[test]
    fn loud_signals_pass_one_block_late_and_noise_falls_to_the_floor() {
        let p = params(2, 480);
        let n = 48_000;
        // loud: noise with no energy near Nyquist (the artifact step's
        // boxcar ends at Nyquist, which tapers the top few bins)
        let white = noise(n + 3, 1e-2, 1);
        let loud: Vec<f32> = white.windows(4).map(|w| w.iter().sum::<f32>() / 4.0).collect();
        let quiet = noise(n, 1e-7, 2);
        let y = run(&p, &[loud.clone(), quiet.clone()]);
        // loud: the input 480 samples late, within a small error
        let err: Vec<f32> = (10_000..n).map(|i| y[0][i] - loud[i - 480]).collect();
        assert!(rms(&err) < 1e-3 * rms(&loud), "loud error {}", rms(&err) / rms(&loud));
        // quiet (well under the profile): attenuated to the floor gain
        let g = rms(&y[1][10_000..]) / rms(&quiet[10_000 - 480..n - 480]);
        assert!((20.0 * g.log10() + 10.0).abs() < 0.2, "quiet gain {} dB", 20.0 * g.log10());
    }

    #[test]
    fn bypass_is_a_pure_block_delay() {
        let mut p = params(1, 240);
        p.bypass = true;
        let x = noise(4800, 0.1, 3);
        let y = run(&p, core::slice::from_ref(&x));
        assert!((240..4800).all(|i| y[0][i] == x[i - 240]));
    }

    #[test]
    fn burg_of_white_noise_is_flat_at_its_variance() {
        let p = params(1, 480);
        let mut r = Run::new(&p, 48_000.0).unwrap();
        let x = noise(960, 0.1, 4);
        for (f, (v, w)) in r.fwd.iter_mut().zip(x.iter().zip(&r.window)) {
            *f = f64::from(*v) * w;
        }
        let var = x.iter().map(|v| f64::from(*v).powi(2)).sum::<f64>() / 960.0;
        r.burg();
        let mean = r.raw[1..480].iter().sum::<f64>() / 479.0;
        assert!((mean / var - 1.0).abs() < 0.1, "{mean} vs {var}");
        assert!(r.raw.iter().all(|v| (v / var) > 0.5 && (v / var) < 2.0));
    }

    #[test]
    fn the_boxcar_is_cut_at_dc_and_zero_past_nyquist() {
        let src = [1.0, 2.0, 3.0, 4.0, 5.0];
        let mut dst = [0.0; 5];
        let mut ext = [0.0; 20];
        Run::boxcar(&mut ext, &src, &mut dst, 5);
        let want = [2.0, 2.5, 3.0, (2.0 + 3.0 + 4.0 + 5.0) / 5.0, (3.0 + 4.0 + 5.0) / 5.0];
        for (d, w) in dst.iter().zip(want) {
            assert!((d - w).abs() < 1e-12, "{dst:?}");
        }
    }
}
