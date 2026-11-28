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
//!          -> frequency stages (the low-frequency blend against
//!             reference beams of the same spectra)
//!          -> STFT synthesis -> post stages (1 channel) -> out
//! ```
//!
//! Stages ([`Stage`]):
//!
//! * [`Stage::Gain`]: the graph's gain builtins (`gain`, `dbgain`):
//!   parameter 0 in dB, applied as a constant. Unlike the speaker voicing's
//!   gain node, a microphone gain may boost (the graph's pre-DSP gain does).
//! * [`Stage::DcBlock`]: the DC blocker unit: a second-order Butterworth
//!   high-pass (bilinear, prewarped) at its cutoff, then its gain in dB,
//!   then its start ramp: a linear fade-in by `1/(ramp_ms Fs/1000)` per
//!   sample, beginning on the first block after a reset that holds a
//!   nonzero sample (prior silence does not consume it). A block is the
//!   engine's internal block (at most [`BLOCK`] frames of one host call).
//!   TODO(SPEC): black-box renders of the unit match this from the ramp's
//!   end on (over 140 dB), not during the ramp, where the unit's output
//!   holds a further term decaying with the high-pass's poles.
//! * [`Stage::Sens`]: the per-microphone noise-floor suppressor
//!   ([`sens`]), run on whole hop blocks with its own transform; it adds
//!   `hop` frames of latency. It and every pre stage after it run at the
//!   analysis frame rate on the latest hop block, so its blocks fall on
//!   the analysis frames' boundaries, as the graph's reblocker makes them.
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
//! completes a frame) are compiled parameters; [`Window::Quadrant`] at
//! phase 0 matches black-box renders of the graph's transform pair
//! exactly (single-bin and delay probes over 135 dB).
//!
//! The frequency stages ([`FreqStage`]) run the adaptive low-frequency
//! blend ([`blend`]) against extra fixed beams of the same spectra (the
//! graph's second beamformer feeding the standalone blend unit).
//!
//! [`Engine::process`] is real-time code: every buffer is made in
//! [`Engine::new`].

pub mod artifact;
pub mod blend;
pub mod fft;
pub mod sens;

use crate::voicing::chain::{Chain, Spec};
use alloc::vec;
use alloc::vec::Vec;
use blend::BlendLaw;
use fft::{Complex, Fft};
use sens::Sens;

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
    /// The clean summary's closed form: `h(x) = 1/2 - cos(c x)/2`,
    /// `c = acos(1 - sqrt 2)`, `q = 4x/N`; over the four quarters `h(q)`,
    /// `sqrt(1 - h(2-q)^2)`, `sqrt(1 - h(q-2)^2)`, `h(4-q)`. Power
    /// complementary, close to [`Window::Vorbis`].
    Quadrant = 3,
}

impl Window {
    pub fn from_u32(v: u32) -> Option<Window> {
        match v {
            1 => Some(Window::Sine),
            2 => Some(Window::Vorbis),
            3 => Some(Window::Quadrant),
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
            Window::Quadrant => {
                let c = libm::acos(1.0 - core::f64::consts::SQRT_2);
                let h = |v: f64| 0.5 - 0.5 * libm::cos(c * v);
                let q = 4.0 * x / size as f64;
                if q < 1.0 {
                    h(q)
                } else if q < 2.0 {
                    libm::sqrt((1.0 - h(2.0 - q).powi(2)).max(0.0))
                } else if q < 3.0 {
                    libm::sqrt((1.0 - h(q - 2.0).powi(2)).max(0.0))
                } else {
                    h(4.0 - q)
                }
            }
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
    /// `gain_db`, and a linear fade-in over `ramp_ms` from the first
    /// nonzero block after a reset.
    DcBlock { cutoff_hz: f64, ramp_ms: f64, gain_db: f64 },
    /// A sub-graph (filter boxes, a nested graph), as many program channels
    /// and outputs as the stage is wide.
    Graph(Spec),
    /// An uncharacterised stage carried as identity (partial blobs only);
    /// the box's four-character subtype.
    Identity([u8; 4]),
    /// The noise-floor suppressor ([`sens`]); pre stages only.
    Sens(Sens),
}

/// A stage between the beam and the synthesis.
#[derive(Clone, Debug, PartialEq)]
pub enum FreqStage {
    /// The low-frequency blend of the path against two of
    /// [`Params::refs`]: the comparison and the blend reference.
    LowBand { compare: usize, blend: usize, law: BlendLaw },
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
            Stage::Sens(s) => s.valid(width, fs),
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
    /// Further fixed beams of the same spectra, for the frequency stages.
    pub refs: Vec<Beam>,
    /// Stages between the beam and the synthesis.
    pub freq: Vec<FreqStage>,
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
            && self.post.iter().all(|st| st.valid(1, fs) && !matches!(st, Stage::Sens(_)))
            && self.pre.iter().all(|st| !matches!(st, Stage::Sens(x) if x.hop != s.hop))
            && (!self.pre.iter().any(|st| matches!(st, Stage::Sens(_))) || s.hop <= BLOCK)
            && self.refs.len() <= MAX_MICS
            && self.refs.iter().all(|r| {
                r.mics == self.mics
                    && r.bins == s.bins()
                    && r.weights.len() == r.mics * r.bins
                    && r.weights.iter().all(|w| w[0].is_finite() && w[1].is_finite())
                    && r.eq.is_none()
            })
            && self.freq.len() <= MAX_STAGES
            && self.freq.iter().all(|f| match f {
                FreqStage::LowBand { compare, blend, law } => {
                    *compare < self.refs.len() && *blend < self.refs.len() && law.valid(fs)
                }
            })
            && self.partial == self.pre.iter().chain(&self.post).any(|st| matches!(st, Stage::Identity(_)))
    }

    /// Algorithmic latency of the whole chain, frames: the transform's and
    /// a hop per suppressor.
    pub fn latency(&self) -> usize {
        self.stft.latency() + self.stft.hop * self.pre.iter().filter(|st| matches!(st, Stage::Sens(_))).count()
    }

    /// Index of the first pre stage that runs on hop blocks (the first
    /// suppressor), or `pre.len()`.
    pub fn block_start(&self) -> usize {
        self.pre.iter().position(|st| matches!(st, Stage::Sens(_))).unwrap_or(self.pre.len())
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
        /// Ramp step per sample (`1/(ramp_ms Fs/1000)`; 0: no ramp).
        step: f64,
        /// Samples into the ramp; `None` until a nonzero block.
        at: Option<usize>,
    },
    Sens(alloc::boxed::Box<sens::Run>),
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
                    step: if *ramp_ms > 0.0 { 1000.0 / (ramp_ms * fs) } else { 0.0 },
                    at: None,
                }
            }
            Stage::Sens(p) => Run::Sens(alloc::boxed::Box::new(sens::Run::new(p, fs)?)),
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
                *at = None;
            }
            Run::Sens(r) => r.reset(),
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
            Run::DcBlock { b, a, state, gain, step, at } => {
                if at.is_none() && bufs.iter().take(width).any(|c| c[..frames].iter().any(|v| *v != 0.0)) {
                    *at = Some(0);
                }
                // the ramp has not started (a silent block) or has ended
                let start = *at;
                for (c, buf) in bufs.iter_mut().take(width).enumerate() {
                    let s = &mut state[c];
                    for (i, x) in buf[..frames].iter_mut().enumerate() {
                        let v = f64::from(*x);
                        let y = b[0] * v + s[0];
                        s[0] = b[1] * v - a[0] * y + s[1];
                        s[1] = b[2] * v - a[1] * y;
                        let r = match start {
                            _ if *step == 0.0 => 1.0,
                            None => 0.0,
                            Some(n) => ((n + i) as f64 * *step).min(1.0),
                        };
                        *x = (y * *gain * r) as f32;
                    }
                }
                if let Some(n) = at.as_mut() {
                    if (*n as f64) * *step < 1.0 {
                        *n += frames;
                    }
                }
            }
            // whole hop blocks only: `Engine::frame_blocks`
            Run::Sens(r) => r.process(bufs),
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
    /// The reference beams, as `refs` in the parameters.
    refs: Vec<Vec<Complex>>,
    freq: Vec<blend::Run>,
    /// Pre stages from this index run on hop blocks at frame time.
    block_start: usize,
    /// Per mic, the latest hop block (block stages run on it in place).
    blk: Vec<Vec<f32>>,
    /// Per mic, the last `size` samples after the block stages.
    ring2: Vec<Vec<f32>>,
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
        let block_start = params.block_start();
        let freq = params
            .freq
            .iter()
            .map(|f| match f {
                FreqStage::LowBand { law, .. } => blend::Run::new(law, s.bins(), s.hop, fs),
            })
            .collect();
        Some(Engine {
            refs: vec![vec![Complex::ZERO; s.bins()]; params.refs.len()],
            freq,
            block_start,
            blk: vec![vec![0.0; s.hop.max(BLOCK)]; params.mics],
            ring2: vec![vec![0.0; s.size]; params.mics],
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
        self.freq.iter_mut().for_each(blend::Run::reset);
        self.ring.iter_mut().for_each(|r| r.fill(0.0));
        self.ring2.iter_mut().for_each(|r| r.fill(0.0));
        self.ring_at = 0;
        self.phase = 0;
        self.ola.fill(0.0);
        self.queue.fill(0.0);
        self.queue_at = 0;
    }

    /// The block stages on the latest hop block of every mic, into `ring2`.
    fn frame_blocks(&mut self) {
        let s = self.params.stft;
        let (n, hop) = (s.size, s.hop);
        let mics = self.params.mics;
        for (m, b) in self.blk.iter_mut().enumerate().take(mics) {
            for (i, v) in b[..hop].iter_mut().enumerate() {
                *v = self.ring[m][(self.ring_at + n - hop + i) % n];
            }
        }
        for r in &mut self.pre[self.block_start..] {
            r.process(&mut self.blk, mics, hop);
        }
        for (r2, b) in self.ring2.iter_mut().zip(&self.blk) {
            r2.copy_within(hop.., 0);
            r2[n - hop..].copy_from_slice(&b[..hop]);
        }
    }

    /// One analysis frame from the rings, the beam, one synthesis frame.
    fn transform(&mut self) {
        let s = self.params.stft;
        let (n, hop, bins) = (s.size, s.hop, s.bins());
        let blocked = self.block_start < self.pre.len();
        if blocked {
            self.frame_blocks();
        }
        let b = &self.params.beam;
        self.beam.fill(Complex::ZERO);
        self.refs.iter_mut().for_each(|r| r.fill(Complex::ZERO));
        for m in 0..self.params.mics {
            if blocked {
                let ring = &self.ring2[m];
                for (i, v) in self.frame.iter_mut().enumerate() {
                    *v = Complex::new(f64::from(ring[i]) * self.window[i], 0.0);
                }
            } else {
                let ring = &self.ring[m];
                for (i, v) in self.frame.iter_mut().enumerate() {
                    let x = ring[(self.ring_at + i) % n];
                    *v = Complex::new(f64::from(x) * self.window[i], 0.0);
                }
            }
            self.fft.forward(&mut self.frame);
            let w = &b.weights[m * bins..(m + 1) * bins];
            for ((acc, wk), x) in self.beam.iter_mut().zip(w).zip(&self.frame) {
                *acc = acc.plus(Complex::new(wk[0], wk[1]).times(*x));
            }
            for (r, rb) in self.refs.iter_mut().zip(&self.params.refs) {
                let w = &rb.weights[m * bins..(m + 1) * bins];
                for ((acc, wk), x) in r.iter_mut().zip(w).zip(&self.frame) {
                    *acc = acc.plus(Complex::new(wk[0], wk[1]).times(*x));
                }
            }
        }
        if let Some(eq) = &b.eq {
            for (v, g) in self.beam.iter_mut().zip(eq) {
                v.re *= g;
                v.im *= g;
            }
        }
        for (run, f) in self.freq.iter_mut().zip(&self.params.freq) {
            let FreqStage::LowBand { compare, blend, .. } = f;
            run.process(&mut self.beam, &self.refs[*compare], &self.refs[*blend]);
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
            for r in &mut self.pre[..self.block_start] {
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
            stft: Stft { hop, size, window: Window::Quadrant, half_offset: false, frame_phase: 0 },
            beam: Beam { mics, bins, weights, eq: None },
            refs: Vec::new(),
            freq: Vec::new(),
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
        for w in [Window::Sine, Window::Vorbis, Window::Quadrant] {
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
        for (k, w) in p.beam.weights.iter_mut().enumerate().take(481) {
            let a = -2.0 * core::f64::consts::PI * k as f64 * 100.0 / 960.0;
            *w = [libm::cos(a), libm::sin(a)];
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

    #[test]
    fn the_dc_ramp_starts_on_the_first_nonzero_block() {
        let mut p = unit_params(1, 480);
        // 10 ms at 48 kHz: 480 samples
        p.pre.push(Stage::DcBlock { cutoff_hz: 0.01, ramp_ms: 10.0, gain_db: 0.0 });
        let mut e = Engine::new(p.clone()).unwrap();
        let lat = e.latency();
        // 2000 samples of silence, then a constant step
        let n = 8000;
        let x: Vec<f32> = (0..n).map(|i| if i < 2000 { 0.0 } else { 0.5 }).collect();
        let y = render(&mut e, core::slice::from_ref(&x), 100);
        // the ramp begins at the block holding sample 2000 (blocks of 100)
        // and rises by 1/480 per sample; the 0.01 Hz high-pass barely moves
        // the step in 480 samples
        for k in [0usize, 100, 240, 479] {
            let got = y[2000 + k + lat];
            let want = 0.5 * k as f64 / 480.0;
            assert!((f64::from(got) - want).abs() < 2e-3, "k {k}: {got} vs {want}");
        }
        // the same step with no silence before it ramps from the start
        let mut e = Engine::new(p).unwrap();
        let x2: Vec<f32> = x[2000..].to_vec();
        let y2 = render(&mut e, &[x2], 100);
        assert!((y2[240 + lat] - y[2240 + lat]).abs() < 1e-3);
    }

    #[test]
    fn a_suppressor_adds_one_hop_of_latency_and_passes_loud_signals() {
        let mut p = unit_params(2, 480);
        let mut sp = sens::tests::params(2, 480);
        sp.profile.fill(1e-14);
        p.pre.push(Stage::Gain { db: 0.0 });
        p.pre.push(Stage::Sens(sp));
        p.pre.push(Stage::Gain { db: 6.0 });
        assert!(p.valid());
        assert_eq!(p.latency(), 959 + 480);
        let mut e = Engine::new(p).unwrap();
        let n = 24_000;
        let x: Vec<Vec<f32>> =
            (0..2).map(|_| (0..n).map(|i| (0.1 * libm::sin(i as f64 * 0.05)) as f32).collect()).collect();
        let y = render(&mut e, &x, 333);
        let lat = e.latency();
        for i in 8000..n {
            assert!((y[i] - 2.0 * x[0][i - lat]).abs() < 1e-3, "at {i}: {} vs {}", y[i], 2.0 * x[0][i - lat]);
        }
    }

    #[test]
    fn a_zero_range_blend_passes_the_beam_and_a_full_one_the_reference() {
        let mut p = unit_params(2, 480);
        // reference: mic 1 alone; beam: mic 0 alone
        let mut r = Beam { mics: 2, bins: 481, weights: vec![[0.0; 2]; 2 * 481], eq: None };
        for w in r.weights[481..].iter_mut() {
            *w = [1.0, 0.0];
        }
        p.refs.push(r);
        let law = BlendLaw {
            boost_limit: false,
            cutoff_hz: 0.0,
            detect_db: 0.0,
            max_boost_db: 0.0,
            scale_floor_db: -90.0,
            scale_smooth_s: 0.0,
            threshold_db: -110.0,
            xover_max_hz: 0.0,
            attack_s: 0.0,
            release_s: 0.0,
            hold_s: 0.0,
            xover_smooth_s: 0.0,
            xover_min_hz: 0.0,
            abs_floor_db: -180.0,
            wind_attack_s: 0.0,
            wind_release_s: 0.0,
            slope: 0.0,
        };
        p.freq.push(FreqStage::LowBand { compare: 0, blend: 0, law });
        assert!(p.valid());
        let n = 9600;
        let x: Vec<Vec<f32>> =
            (0..2).map(|c| (0..n).map(|i| libm::sin(i as f64 * 0.01 * (c + 1) as f64) as f32).collect()).collect();
        let y = render(&mut Engine::new(p.clone()).unwrap(), &x, 256);
        assert!((2000..n).all(|i| (y[i] - x[0][i - 959]).abs() < 1e-5));
        // the crossover pinned at Nyquist: the reference everywhere
        let FreqStage::LowBand { law, .. } = &mut p.freq[0];
        law.xover_min_hz = 24_000.0;
        law.xover_max_hz = 24_000.0;
        let y = render(&mut Engine::new(p).unwrap(), &x, 256);
        assert!((2000..n).all(|i| (y[i] - x[1][i - 959]).abs() < 1e-5));
    }
}
