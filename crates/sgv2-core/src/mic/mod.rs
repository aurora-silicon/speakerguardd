// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The built-in microphone chain: the Mac's default capture graph from its
//! N microphone channels to one processed channel, in the graph's order.
//!
//! The shape and every value come from the Mac's own tuning bundle, compiled
//! by `aurora-speaker-tuning`'s `mic` module into [`Params`] (and the
//! versioned, CRC-checked [`artifact`]). The chain is:
//!
//! ```text
//! mics (N) -> pre stages (N channels) -> STFT analysis per mic
//!          -> fixed beam  Y(k) = sum_m W_m(k) X_m(k)
//!          -> STFT synthesis -> post stages (1 channel) -> out
//! ```
//!
//! Stages ([`Stage`]):
//!
//! * [`Stage::Gain`]: the graph's gain builtins (`gain`, `dbgain`):
//!   parameter 0 in dB, applied as a constant. Unlike the speaker voicing's
//!   gain node, a microphone gain may boost (the graph's pre-DSP gain does).
//! * [`Stage::DcBlock`]: the DC blocker unit: a second-order Butterworth
//!   high-pass (bilinear, prewarped) at its cutoff, then its gain in dB.
//!   Its start ramp is TODO(SPEC): carried as a linear fade-in over the
//!   ramp time from the first frame after a reset.
//! * [`Stage::Graph`]: filter boxes and nested graphs, run by the voicing
//!   chain ([`crate::voicing::chain`]) with its filter-box law.
//! * [`Stage::Identity`]: a stage the compiler could not characterise and
//!   carries as identity, only in a blob flagged partial, and only for
//!   stages documented as transparent or attenuating.
//!
//! The short-time transform ([`Stft`]) takes a frame of `size = 2 hop`
//! samples every `hop` samples, windows it, transforms it, applies the
//! beam, transforms back, windows again and overlap-adds. With a
//! power-complementary window (`w(n)^2 + w(n + hop)^2 = 1`) a unit beam
//! reconstructs the input exactly, `size - 1` samples late. The window
//! shape, its sample offset and the frame phase (which stream sample
//! completes a frame) are compiled parameters. TODO(SPEC): the unit's
//! exact window and phase; the compiler's default is the best fit to
//! black-box renders so far, not an exact match.
//!
//! The beam's adaptive parts (omni blend, wind and scratch limiting, beam
//! selection) are TODO(SPEC): the chain runs the fixed weights only.
//!
//! [`Engine::process`] is real-time code: every buffer is made in
//! [`Engine::new`].

pub mod artifact;
pub mod fft;

use crate::voicing::chain::{Chain, Spec};
use alloc::vec;
use alloc::vec::Vec;
use fft::{Complex, Fft};

/// Most microphone channels a chain takes.
pub const MAX_MICS: usize = 16;
/// Largest hop (and so half the largest frame).
pub const MAX_HOP: usize = 2048;
/// Frames per internal block; host blocks of any size are split into these.
pub const BLOCK: usize = 512;
/// Largest stage count on either side of the transform.
pub const MAX_STAGES: usize = 32;
/// Largest boost, dB, of a gain stage (any attenuation is allowed).
pub const MAX_GAIN_DB: f64 = 60.0;

/// The analysis/synthesis window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum Window {
    /// `sin(pi x / N)`.
    Sine = 1,
    /// `sin(pi/2 sin^2(pi x / N))` (power complementary, as the sine is).
    Vorbis = 2,
}

impl Window {
    pub fn from_u32(v: u32) -> Option<Window> {
        match v {
            1 => Some(Window::Sine),
            2 => Some(Window::Vorbis),
            _ => None,
        }
    }

    /// The window at sample `n` of `size`, `x = n + offset`.
    pub fn at(self, n: usize, size: usize, half_offset: bool) -> f64 {
        let x = n as f64 + if half_offset { 0.5 } else { 0.0 };
        let s = libm::sin(core::f64::consts::PI * x / size as f64);
        match self {
            Window::Sine => s,
            Window::Vorbis => libm::sin(core::f64::consts::FRAC_PI_2 * s * s),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stft {
    pub hop: usize,
    /// Frame length; always `2 * hop`.
    pub size: usize,
    pub window: Window,
    /// Evaluate the window at `n + 1/2` instead of `n`.
    pub half_offset: bool,
    /// A frame is taken once the stream's sample `t` with
    /// `t % hop == frame_phase` has arrived (`t = 0` is the first sample
    /// after a reset).
    pub frame_phase: usize,
}

impl Stft {
    pub fn bins(&self) -> usize {
        self.size / 2 + 1
    }

    /// Algorithmic latency, frames.
    pub fn latency(&self) -> usize {
        self.size - 1
    }
}

/// A fixed frequency-domain beam over the microphones.
#[derive(Clone, Debug, PartialEq)]
pub struct Beam {
    pub mics: usize,
    pub bins: usize,
    /// `(re, im)` of `W_m(k)` at `m * bins + k`.
    pub weights: Vec<[f64; 2]>,
    /// A real gain per bin applied to the beam output, if the compiler
    /// was asked to carry one. TODO(SPEC): whether and how the unit applies
    /// its per-bin `EQSelection` table.
    pub eq: Option<Vec<f64>>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Stage {
    /// A constant gain, dB.
    Gain { db: f64 },
    /// Second-order Butterworth high-pass at `cutoff_hz`, a gain of
    /// `gain_db`, and a linear fade-in over `ramp_ms` after a reset.
    DcBlock { cutoff_hz: f64, ramp_ms: f64, gain_db: f64 },
    /// A sub-graph (filter boxes, a nested graph), as many program channels
    /// and outputs as the stage is wide.
    Graph(Spec),
    /// An uncharacterised stage carried as identity (partial blobs only);
    /// the box's four-character subtype.
    Identity([u8; 4]),
}

impl Stage {
    fn valid(&self, width: usize, fs: f64) -> bool {
        match self {
            Stage::Gain { db } => db.is_finite() && *db <= MAX_GAIN_DB,
            Stage::DcBlock { cutoff_hz, ramp_ms, gain_db } => {
                cutoff_hz.is_finite()
                    && *cutoff_hz > 0.0
                    && *cutoff_hz < fs / 2.0
                    && ramp_ms.is_finite()
                    && (0.0..=10_000.0).contains(ramp_ms)
                    && gain_db.is_finite()
                    && gain_db.abs() <= MAX_GAIN_DB
            }
            Stage::Graph(spec) => spec.program_channels == width && spec.outputs.len() == width && spec.check().is_ok(),
            Stage::Identity(_) => true,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    pub sample_rate: u32,
    pub mics: usize,
    /// Stages ahead of the analysis, `mics` channels wide.
    pub pre: Vec<Stage>,
    pub stft: Stft,
    pub beam: Beam,
    /// Stages after the synthesis, one channel.
    pub post: Vec<Stage>,
    /// Some stage is carried as identity (see [`Stage::Identity`]).
    pub partial: bool,
}

impl Params {
    /// Every field in range and every width consistent.
    pub fn valid(&self) -> bool {
        let fs = self.sample_rate as f64;
        let s = &self.stft;
        let b = &self.beam;
        self.sample_rate > 0
            && (1..=MAX_MICS).contains(&self.mics)
            && (1..=MAX_HOP).contains(&s.hop)
            && s.size == 2 * s.hop
            && s.frame_phase < s.hop
            && fft::Fft::new(s.size).is_some()
            && b.mics == self.mics
            && b.bins == s.bins()
            && b.weights.len() == b.mics * b.bins
            && b.weights.iter().all(|w| w[0].is_finite() && w[1].is_finite())
            && b.eq.as_ref().is_none_or(|e| e.len() == b.bins && e.iter().all(|v| v.is_finite()))
            && self.pre.len() <= MAX_STAGES
            && self.post.len() <= MAX_STAGES
            && self.pre.iter().all(|st| st.valid(self.mics, fs))
            && self.post.iter().all(|st| st.valid(1, fs))
            && self.partial == self.pre.iter().chain(&self.post).any(|st| matches!(st, Stage::Identity(_)))
    }

    /// Algorithmic latency of the whole chain, frames (the transform's; the
    /// other stages add none).
    pub fn latency(&self) -> usize {
        self.stft.latency()
    }
}

enum Run {
    Gain(f32),
    DcBlock {
        b: [f64; 3],
        a: [f64; 2],
        /// Per channel `(s1, s2)`, transposed direct form II.
        state: Vec<[f64; 2]>,
        gain: f64,
        ramp: usize,
        at: usize,
    },
    Graph {
        chain: alloc::boxed::Box<Chain>,
        out: Vec<Vec<f32>>,
    },
    Identity,
}

impl Run {
    fn new(st: &Stage, width: usize, fs: f64) -> Option<Run> {
        Some(match st {
            Stage::Gain { db } => Run::Gain(libm::pow(10.0, db / 20.0) as f32),
            Stage::DcBlock { cutoff_hz, ramp_ms, gain_db } => {
                let c = crate::voicing::tone::butterworth(true, 2, *cutoff_hz, fs);
                let s = c.sections[0];
                Run::DcBlock {
                    b: [s.b0, s.b1, s.b2],
                    a: [s.a1, s.a2],
                    state: vec![[0.0; 2]; width],
                    gain: libm::pow(10.0, gain_db / 20.0),
                    ramp: libm::round(ramp_ms * fs / 1000.0) as usize,
                    at: 0,
                }
            }
            Stage::Graph(spec) => Run::Graph {
                chain: alloc::boxed::Box::new(Chain::new(spec.clone(), fs, BLOCK)?),
                out: vec![vec![0.0; BLOCK]; width],
            },
            Stage::Identity(_) => Run::Identity,
        })
    }

    fn reset(&mut self) {
        match self {
            Run::DcBlock { state, at, .. } => {
                state.iter_mut().for_each(|s| *s = [0.0; 2]);
                *at = 0;
            }
            Run::Graph { chain, .. } => chain.reset(),
            Run::Gain(_) | Run::Identity => {}
        }
    }

    /// In place on `bufs[..][..frames]`, `frames <= BLOCK`.
    fn process(&mut self, bufs: &mut [Vec<f32>], width: usize, frames: usize) {
        match self {
            Run::Gain(g) => {
                for b in bufs.iter_mut().take(width) {
                    b[..frames].iter_mut().for_each(|x| *x *= *g);
                }
            }
            Run::DcBlock { b, a, state, gain, ramp, at } => {
                let start = *at;
                for (c, buf) in bufs.iter_mut().take(width).enumerate() {
                    let s = &mut state[c];
                    for (i, x) in buf[..frames].iter_mut().enumerate() {
                        let v = f64::from(*x);
                        let y = b[0] * v + s[0];
                        s[0] = b[1] * v - a[0] * y + s[1];
                        s[1] = b[2] * v - a[1] * y;
                        // TODO(SPEC): the unit's start ramp; a linear fade-in
                        let n = start + i;
                        let r = if n < *ramp { n as f64 / *ramp as f64 } else { 1.0 };
                        *x = (y * *gain * r) as f32;
                    }
                }
                *at = start.saturating_add(frames).min(*ramp);
            }
            Run::Graph { chain, out } => {
                let ins: [&[f32]; MAX_MICS] =
                    core::array::from_fn(|c| if c < width { &bufs[c][..frames] } else { &[][..] });
                let mut it = out.iter_mut();
                let mut lanes: [&mut [f32]; MAX_MICS] = core::array::from_fn(|c| {
                    if c < width {
                        it.next().map_or(&mut [][..], |v| &mut v[..frames])
                    } else {
                        &mut [][..]
                    }
                });
                chain.process(&ins[..width], &mut lanes[..width], frames);
                for (dst, src) in bufs.iter_mut().zip(out.iter()).take(width) {
                    dst[..frames].copy_from_slice(&src[..frames]);
                }
            }
            Run::Identity => {}
        }
    }
}

/// The running chain.
pub struct Engine {
    params: Params,
    pre: Vec<Run>,
    post: Vec<Run>,
    fft: Fft,
    window: Vec<f64>,
    /// Per mic, the last `size` input samples (a ring at `ring_at`).
    ring: Vec<Vec<f32>>,
    ring_at: usize,
    /// The stream position modulo the hop.
    phase: usize,
    frame: Vec<Complex>,
    beam: Vec<Complex>,
    ola: Vec<f64>,
    queue: Vec<f32>,
    queue_at: usize,
    work: Vec<Vec<f32>>,
    mono: Vec<Vec<f32>>,
}

impl Engine {
    /// `None` if the parameters are not [`Params::valid`]. Not real time.
    pub fn new(params: Params) -> Option<Engine> {
        if !params.valid() {
            return None;
        }
        let fs = params.sample_rate as f64;
        let s = params.stft;
        let pre = params.pre.iter().map(|st| Run::new(st, params.mics, fs)).collect::<Option<Vec<_>>>()?;
        let post = params.post.iter().map(|st| Run::new(st, 1, fs)).collect::<Option<Vec<_>>>()?;
        let window = (0..s.size).map(|n| s.window.at(n, s.size, s.half_offset)).collect();
        Some(Engine {
            fft: Fft::new(s.size)?,
            window,
            ring: vec![vec![0.0; s.size]; params.mics],
            ring_at: 0,
            phase: 0,
            frame: vec![Complex::ZERO; s.size],
            beam: vec![Complex::ZERO; s.bins()],
            ola: vec![0.0; s.size],
            queue: vec![0.0; s.hop],
            queue_at: 0,
            work: vec![vec![0.0; BLOCK]; params.mics],
            mono: vec![vec![0.0; BLOCK]; 1],
            pre,
            post,
            params,
        })
    }

    pub fn params(&self) -> &Params {
        &self.params
    }

    pub fn mics(&self) -> usize {
        self.params.mics
    }

    pub fn latency(&self) -> usize {
        self.params.latency()
    }

    /// Back to the state [`Engine::new`] gives. Real time.
    pub fn reset(&mut self) {
        self.pre.iter_mut().chain(self.post.iter_mut()).for_each(Run::reset);
        self.ring.iter_mut().for_each(|r| r.fill(0.0));
        self.ring_at = 0;
        self.phase = 0;
        self.ola.fill(0.0);
        self.queue.fill(0.0);
        self.queue_at = 0;
    }

    /// One analysis frame from the rings, the beam, one synthesis frame.
    fn transform(&mut self) {
        let s = self.params.stft;
        let (n, hop, bins) = (s.size, s.hop, s.bins());
        let b = &self.params.beam;
        self.beam.fill(Complex::ZERO);
        for m in 0..self.params.mics {
            let ring = &self.ring[m];
            for (i, v) in self.frame.iter_mut().enumerate() {
                let x = ring[(self.ring_at + i) % n];
                *v = Complex::new(f64::from(x) * self.window[i], 0.0);
            }
            self.fft.forward(&mut self.frame);
            let w = &b.weights[m * bins..(m + 1) * bins];
            for ((acc, wk), x) in self.beam.iter_mut().zip(w).zip(&self.frame) {
                *acc = acc.plus(Complex::new(wk[0], wk[1]).times(*x));
            }
        }
        if let Some(eq) = &b.eq {
            for (v, g) in self.beam.iter_mut().zip(eq) {
                v.re *= g;
                v.im *= g;
            }
        }
        // A real signal's spectrum: DC and Nyquist are real (their
        // imaginary parts are dropped, as a packed real transform drops
        // them), the rest Hermitian.
        self.frame[0] = Complex::new(self.beam[0].re, 0.0);
        self.frame[n / 2] = Complex::new(self.beam[n / 2].re, 0.0);
        for k in 1..n / 2 {
            self.frame[k] = self.beam[k];
            self.frame[n - k] = self.beam[k].conj();
        }
        self.fft.inverse(&mut self.frame);
        for (i, o) in self.ola.iter_mut().enumerate() {
            *o += self.frame[i].re * self.window[i];
        }
        for (q, o) in self.queue.iter_mut().zip(&self.ola[..hop]) {
            *q = *o as f32;
        }
        self.ola.copy_within(hop.., 0);
        self.ola[n - hop..].fill(0.0);
        self.queue_at = 0;
    }

    /// `inputs` (one slice per microphone, at least `frames` long) into
    /// `output`. Wrong channel counts or short slices give silence.
    /// Non-finite input is treated as silence. Real time.
    pub fn process(&mut self, inputs: &[&[f32]], output: &mut [f32], frames: usize) {
        let mics = self.params.mics;
        let frames = frames.min(output.len());
        if inputs.len() != mics || inputs.iter().any(|x| x.len() < frames) {
            output[..frames].fill(0.0);
            return;
        }
        let s = self.params.stft;
        let mut at = 0;
        while at < frames {
            let len = BLOCK.min(frames - at);
            for (w, x) in self.work.iter_mut().zip(inputs) {
                for (d, v) in w[..len].iter_mut().zip(&x[at..at + len]) {
                    *d = if v.is_finite() { *v } else { 0.0 };
                }
            }
            for r in &mut self.pre {
                r.process(&mut self.work, mics, len);
            }
            for i in 0..len {
                for m in 0..mics {
                    self.ring[m][self.ring_at] = self.work[m][i];
                }
                self.ring_at = (self.ring_at + 1) % s.size;
                if self.phase == s.frame_phase {
                    self.transform();
                }
                self.phase = (self.phase + 1) % s.hop;
                let q = self.queue_at.min(s.hop - 1);
                self.mono[0][i] = self.queue[q];
                self.queue_at = (self.queue_at + 1).min(s.hop);
            }
            for r in &mut self.post {
                r.process(&mut self.mono, 1, len);
            }
            for (d, v) in output[at..at + len].iter_mut().zip(&self.mono[0][..len]) {
                *d = if v.is_finite() { *v } else { 0.0 };
            }
            at += len;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn unit_params(mics: usize, hop: usize) -> Params {
        let size = 2 * hop;
        let bins = hop + 1;
        let mut weights = vec![[0.0; 2]; mics * bins];
        for w in weights.iter_mut().take(bins) {
            *w = [1.0, 0.0];
        }
        Params {
            sample_rate: 48_000,
            mics,
            pre: Vec::new(),
            stft: Stft { hop, size, window: Window::Vorbis, half_offset: false, frame_phase: 0 },
            beam: Beam { mics, bins, weights, eq: None },
            post: Vec::new(),
            partial: false,
        }
    }

    fn render(e: &mut Engine, x: &[Vec<f32>], block: usize) -> Vec<f32> {
        let n = x[0].len();
        let mut y = vec![0.0f32; n];
        let mut at = 0;
        while at < n {
            let len = block.min(n - at);
            let ins: Vec<&[f32]> = x.iter().map(|c| &c[at..at + len]).collect();
            e.process(&ins, &mut y[at..at + len], len);
            at += len;
        }
        y
    }

    #[test]
    fn windows_are_power_complementary() {
        for w in [Window::Sine, Window::Vorbis] {
            for half in [false, true] {
                for n in 0..480 {
                    let a = w.at(n, 960, half);
                    let b = w.at(n + 480, 960, half);
                    assert!((a * a + b * b - 1.0).abs() < 1e-12);
                }
            }
        }
    }

    #[test]
    fn unit_beam_reconstructs_the_first_mic_one_frame_late() {
        for phase in [0usize, 7, 479] {
            let mut p = unit_params(3, 480);
            p.stft.frame_phase = phase;
            let mut e = Engine::new(p).unwrap();
            let n = 9600;
            let x: Vec<Vec<f32>> =
                (0..3).map(|c| (0..n).map(|i| libm::sin(i as f64 * 0.01 * (c + 1) as f64) as f32).collect()).collect();
            let y = render(&mut e, &x, 333);
            let lat = e.latency();
            assert_eq!(lat, 959);
            // past the first full frame
            for i in 2000..n {
                assert!((y[i] - x[0][i - lat]).abs() < 1e-5, "phase {phase} at {i}: {} vs {}", y[i], x[0][i - lat]);
            }
        }
    }

    #[test]
    fn a_delay_beam_delays_inside_the_frame() {
        let mut p = unit_params(1, 480);
        for k in 0..481 {
            let a = -2.0 * core::f64::consts::PI * k as f64 * 100.0 / 960.0;
            p.beam.weights[k] = [libm::cos(a), libm::sin(a)];
        }
        let mut e = Engine::new(p).unwrap();
        let mut x = vec![0.0f32; 4800];
        x[2400] = 1.0;
        let y = render(&mut e, &[x], 64);
        let peak = (0..y.len()).max_by(|&a, &b| y[a].abs().total_cmp(&y[b].abs())).unwrap();
        assert_eq!(peak, 2400 + 959 + 100);
    }

    #[test]
    fn gains_and_dc_blocker_and_block_size_independence() {
        let mut p = unit_params(2, 480);
        p.pre.push(Stage::DcBlock { cutoff_hz: 30.0, ramp_ms: 10.0, gain_db: 0.0 });
        p.pre.push(Stage::Gain { db: 6.0 });
        p.post.push(Stage::Gain { db: -6.0 });
        let x: Vec<Vec<f32>> =
            (0..2).map(|_| (0..20_000).map(|i| 0.3 + libm::sin(i as f64 * 0.05) as f32).collect()).collect();
        let mut a = Engine::new(p.clone()).unwrap();
        let mut b = Engine::new(p).unwrap();
        let ya = render(&mut a, &x, 512);
        let yb = render(&mut b, &x, 17);
        assert!(ya.iter().zip(&yb).all(|(u, v)| (u - v).abs() < 1e-6));
        // the DC offset is gone and the sine passes at unity (+6 -6 dB)
        let tail = &ya[15_000..];
        let mean: f32 = tail.iter().sum::<f32>() / tail.len() as f32;
        assert!(mean.abs() < 1e-2, "mean {mean}");
        let peak = tail.iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!((peak - 1.0).abs() < 0.01, "peak {peak}");
        // reset gives the same output again
        a.reset();
        let yc = render(&mut a, &x, 512);
        assert_eq!(ya, yc);
    }

    #[test]
    fn bad_params_are_refused_and_bad_io_is_silence() {
        let mut p = unit_params(2, 480);
        p.stft.size = 961;
        assert!(Engine::new(p).is_none());
        let mut p = unit_params(2, 480);
        p.partial = true;
        assert!(Engine::new(p).is_none(), "partial flag without an identity stage");
        let mut p = unit_params(2, 480);
        p.pre.push(Stage::Identity(*b"sens"));
        assert!(Engine::new(p.clone()).is_none(), "identity stage without the partial flag");
        p.partial = true;
        let mut e = Engine::new(p).unwrap();
        let x = vec![1.0f32; 64];
        let mut y = vec![9.0f32; 64];
        e.process(&[&x], &mut y, 64);
        assert!(y.iter().all(|v| *v == 0.0));
        let nan = vec![f32::NAN; 64];
        e.process(&[&nan, &nan], &mut y, 64);
        assert!(y.iter().all(|v| v.is_finite()));
    }
}
