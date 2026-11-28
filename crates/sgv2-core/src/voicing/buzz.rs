// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Linked multi-line suppression (`aufx/buzk`, "BuzzKill"), version zero.
//! See `docs/SPEC-BUZZKILL.md`.

use crate::prestage::Section;
use crate::MAX_CHANNELS;
use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

pub const BANDS: usize = 10;
const FFT: usize = 512;
const TRACKS: usize = 50;
const ANALYSIS_HOP: usize = 128;
const ANALYSIS_RATE: f64 = 12_000.0;
const DECIMATION: usize = 4;
const HOST_HOP: usize = ANALYSIS_HOP * DECIMATION;
const MIN_HZ: f64 = 50.0;
const MAX_HZ: [f64; 5] = [2500.0, 3000.0, 4000.0, 6000.0, 20_000.0];
const MATCH_RELATIVE: f64 = 0.035_264_92;
const CANDIDATE_LOWER: f32 = 0.965_936_3;
const CANDIDATE_UPPER: f32 = 1.035_265;
const REPLACE_GAIN: f64 = 0.82;
const ACTIVE_GAIN: f32 = 0.9999;
const NOTCH_QUALITY: f64 = 6.0;
const WET_SMOOTH_HZ: f64 = 5.0;
/// The first complete analysis block produces the native narrow D1 bank.
const WARMUP_DECISIONS: u32 = 0;
/// One line found by a decision.
#[derive(Clone, Copy, Default)]
struct Candidate {
    frequency_hz: f32,
    multiplier: f32,
    /// The candidate cell's released record gain: the primary order key.
    record_gain: f32,
    /// Initial multiplier when this line starts a new track (the head cell's own multiplier).
    /// (`multiplier` becomes the line's deepest cell multiplier once merged.)
    start_multiplier: f32,
}

// The analysis path low-passes the linked program at the operation limit
// (a second-order Butterworth, bilinear) and keeps every fourth sample. The
// gain of two puts lines in the amplitude domain the saved thresholds use.
const ANALYSIS_GAIN: f32 = 2.0;

/// The transform cells a decision reads: counting cell k as the band from
/// k to k+1 cell widths, those that end at or below the operation limit.
/// (Matches the installed unit on the parity gate: a 2500 Hz line in the
/// 2500 Hz band moves no track there, while the cell rounded to the nearest
/// centre would read it, a little low, through the first-difference
/// refinement.)
fn analysed_cells(maximum_hz: f64) -> usize {
    (libm::floor(maximum_hz * FFT as f64 / ANALYSIS_RATE) as usize).min(FFT / 2)
}

/// Folds candidates (already in decision order) that see the same line:
/// one inside an earlier candidate's duplicate band joins it, which keeps
/// its own frequency and start multiplier while taking the deeper continuation
/// multiplier. Returns the number of lines, compacted to the front.
fn merge_same_line(c: &mut [Candidate]) -> usize {
    let mut lines = 0usize;
    for i in 0..c.len() {
        let next = c[i];
        let f = next.frequency_hz;
        match c[..lines]
            .iter_mut()
            .find(|a| f > a.frequency_hz * CANDIDATE_LOWER && f < a.frequency_hz * CANDIDATE_UPPER)
        {
            Some(line) => {
                line.multiplier = line.multiplier.min(next.multiplier);
            }
            None => {
                c[lines] = next;
                lines += 1;
            }
        }
    }
    lines
}

fn analysis_lowpass(frequency_hz: f64, fs: f64) -> Section {
    let k = libm::tan(core::f64::consts::PI * frequency_hz / fs);
    let norm = 1.0 / (1.0 + core::f64::consts::SQRT_2 * k + k * k);
    Section {
        b0: k * k * norm,
        b1: 2.0 * k * k * norm,
        b2: k * k * norm,
        a1: 2.0 * (k * k - 1.0) * norm,
        a2: (1.0 - core::f64::consts::SQRT_2 * k + k * k) * norm,
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Band {
    pub enabled: bool,
    pub frequency_hz: f64,
    pub gain_db: f64,
    pub quality: f64,
}

impl Default for Band {
    fn default() -> Self {
        Band { enabled: false, frequency_hz: 1000.0, gain_db: 0.0, quality: 1.0 }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    pub operation_frequency: u32,
    pub lookahead_s: f64,
    pub threshold_db: f64,
    pub algorithm: u32,
    pub bands: [Band; BANDS],
    /// The detector's amplitude-record cascade: the two second-order
    /// sections `[b0, b1, b2, a1, a2]` of the quadrature matrix's second
    /// output. The unit's values are not shipped: the installer reads them
    /// from the Mac's own AudioDSP (see `docs/SPEC-BUZZKILL.md`).
    pub quadrature: [[f32; 5]; QUADRATURE_SECTIONS],
}

/// Second-order sections in the detector's amplitude-record cascade.
pub const QUADRATURE_SECTIONS: usize = 2;
/// A cascade that leaves the record unchanged (tests and defaults only; a
/// compiled box always carries the Mac's own matrix).
pub const QUADRATURE_IDENTITY: [[f32; 5]; QUADRATURE_SECTIONS] = [[1.0, 0.0, 0.0, 0.0, 0.0]; QUADRATURE_SECTIONS];

impl Default for Params {
    fn default() -> Self {
        Params {
            operation_frequency: 0,
            lookahead_s: 0.008,
            threshold_db: -20.0,
            algorithm: 0,
            bands: [Band::default(); BANDS],
            quadrature: QUADRATURE_IDENTITY,
        }
    }
}

/// Whether a section is finite with a strictly stable denominator.
pub fn stable_section(s: &[f32; 5]) -> bool {
    let [_, _, _, a1, a2] = *s;
    s.iter().all(|v| v.is_finite()) && a2.abs() < 1.0 && 1.0 + a1 + a2 > 0.0 && 1.0 - a1 + a2 > 0.0
}

impl Params {
    pub const WORDS: usize = 4 + 4 * BANDS + 5 * QUADRATURE_SECTIONS;
    pub fn to_words(&self, out: &mut [f64]) {
        if out.len() != Self::WORDS {
            return;
        }
        out[..4].copy_from_slice(&[
            self.operation_frequency as f64,
            self.lookahead_s,
            self.threshold_db,
            self.algorithm as f64,
        ]);
        for (i, b) in self.bands.iter().enumerate() {
            let o = 4 + 4 * i;
            out[o..o + 4].copy_from_slice(&[b.enabled as u8 as f64, b.frequency_hz, b.gain_db, b.quality]);
        }
        let q = 4 + 4 * BANDS;
        for (i, section) in self.quadrature.iter().enumerate() {
            for (k, v) in section.iter().enumerate() {
                out[q + 5 * i + k] = *v as f64;
            }
        }
    }
    pub fn from_words(w: &[f64]) -> Option<Params> {
        if w.len() != Self::WORDS || w.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let integer = |v: f64, hi: u32| (v >= 0.0 && v <= hi as f64 && libm::trunc(v) == v).then_some(v as u32);
        let mut p = Params {
            operation_frequency: integer(w[0], 4)?,
            lookahead_s: w[1],
            threshold_db: w[2],
            algorithm: integer(w[3], 3)?,
            ..Default::default()
        };
        if !(0.002..=0.015).contains(&p.lookahead_s) || !(-100.0..=10.0).contains(&p.threshold_db) {
            return None;
        }
        for (i, b) in p.bands.iter_mut().enumerate() {
            let o = 4 + 4 * i;
            *b = Band {
                enabled: match w[o] {
                    0.0 => false,
                    1.0 => true,
                    _ => return None,
                },
                frequency_hz: w[o + 1],
                gain_db: w[o + 2],
                quality: w[o + 3],
            };
            if !(10.0..=20_000.0).contains(&b.frequency_hz)
                || !(-30.0..=0.0).contains(&b.gain_db)
                || !(0.1..=100.0).contains(&b.quality)
            {
                return None;
            }
        }
        let q = 4 + 4 * BANDS;
        for (i, section) in p.quadrature.iter_mut().enumerate() {
            for (k, v) in section.iter_mut().enumerate() {
                let word = w[q + 5 * i + k];
                *v = word as f32;
                if *v as f64 != word {
                    return None;
                }
            }
            if !stable_section(section) {
                return None;
            }
        }
        Some(p)
    }
}

#[derive(Clone, Copy, Default)]
struct FilterState {
    z1: f32,
    z2: f32,
}

fn filter(section: Section, state: &mut FilterState, x: f32) -> f32 {
    let [b0, b1, b2, a1, a2] = [section.b0, section.b1, section.b2, section.a1, section.a2].map(|v| v as f32);
    let y = b0 * x + state.z1;
    state.z1 = b1 * x - a1 * y + state.z2;
    state.z2 = b2 * x - a2 * y;
    y
}

/// Unity-centre band-pass used by one moving rejection track.  BuzzKill's
/// tracked gain is a dry remainder, not a peaking-EQ gain: the unit subtracts
/// `(1 - gain)` times this narrow component from the signal.  Keeping the
/// resonator fixed while the mix moves is also what lets its state survive a
/// detector decision without a coefficient discontinuity.
fn rejection(frequency_hz: f64, quality: f64, fs: f64) -> Section {
    let w = 2.0 * core::f64::consts::PI * frequency_hz / fs;
    let alpha = libm::sin(w) / (2.0 * quality.max(0.5));
    let norm = 1.0 / (1.0 + alpha);
    Section { b0: alpha * norm, b1: 0.0, b2: -alpha * norm, a1: -2.0 * libm::cos(w) * norm, a2: (1.0 - alpha) * norm }
}

/// The host-rate rejection.  The unit states its notch quality in the 12 kHz
/// analysis domain, and its host-rate resonance rings out at that design's
/// rate: the pole radius per second of an analysis-rate band-pass of
/// `NOTCH_QUALITY`.  Measured on the installed unit: the step-off ring is
/// 1.5 % longer than a host-rate constant quality's at 500 Hz, 4.6 % at
/// 1 kHz, 13 % at 1.6 kHz and 22 % at 2 kHz.
fn audio_rejection(frequency_hz: f64, fs: f64) -> Section {
    let design = rejection(frequency_hz, NOTCH_QUALITY, ANALYSIS_RATE);
    // a2 is the squared pole radius per analysis sample.
    let radius = libm::pow(design.a2.clamp(0.0, 1.0), 0.5 * ANALYSIS_RATE / fs);
    let alpha = (1.0 - radius * radius) / (1.0 + radius * radius);
    let w = 2.0 * core::f64::consts::PI * frequency_hz / fs;
    let norm = 1.0 / (1.0 + alpha);
    Section { b0: alpha * norm, b1: 0.0, b2: -alpha * norm, a1: -2.0 * libm::cos(w) * norm, a2: (1.0 - alpha) * norm }
}

fn wet_smoother(fs: f64) -> Section {
    let k = libm::tan(core::f64::consts::PI * WET_SMOOTH_HZ / fs);
    let b = k / (1.0 + k);
    Section { b0: b, b1: b, b2: 0.0, a1: (k - 1.0) / (k + 1.0), a2: 0.0 }
}

fn threshold_peak(gain_db: f64, frequency_hz: f64, quality: f64, fs: f64) -> Section {
    let a = libm::pow(10.0, gain_db / 40.0);
    let w = 2.0 * core::f64::consts::PI * frequency_hz / fs;
    let alpha = libm::sin(w) * libm::sinh(0.5 / quality.max(0.1));
    let cosine = libm::cos(w);
    let norm = 1.0 / (1.0 + alpha / a);
    Section {
        b0: (1.0 + alpha * a) * norm,
        b1: -2.0 * cosine * norm,
        b2: (1.0 - alpha * a) * norm,
        a1: -2.0 * cosine * norm,
        a2: (1.0 - alpha / a) * norm,
    }
}

#[derive(Clone)]
struct Track {
    frequency_hz: f32,
    gain: f32,
    audio_section: Section,
    audio: [FilterState; MAX_CHANNELS],
    audio_wet: [FilterState; MAX_CHANNELS],
    audio_applied_wet: [f32; MAX_CHANNELS],
}

impl Default for Track {
    fn default() -> Self {
        Track {
            frequency_hz: 0.0,
            gain: 1.0,
            audio_section: Section::IDENTITY,
            audio: [FilterState::default(); MAX_CHANNELS],
            audio_wet: [FilterState::default(); MAX_CHANNELS],
            audio_applied_wet: [0.0; MAX_CHANNELS],
        }
    }
}

pub struct Run {
    channels: usize,
    fs: f64,
    maximum_hz: f64,
    lookahead: usize,
    delay: Box<[f32]>,
    delay_at: usize,
    history: [f32; FFT],
    /// Precomputed symmetric analysis window (`w[n] = (1 - cos(2*pi*n/511)) / (2*511)`).
    window: [f32; FFT],
    /// Precomputed radix-2 twiddle factors `exp(-j * 2*pi*k / FFT)` for `k in 0..FFT/2`.
    fft_twiddles: [(f32, f32); FFT / 2],
    /// Per-cell minimum threshold, constant once the Kill bands are configured.
    cell_thresholds: [f32; FFT / 2 + 1],
    /// Per-cell gain record of the detector: what the moving sections have
    /// removed from each transform cell, as the analyser sees it.
    bin_gains: [f32; FFT / 2 + 1],
    history_at: usize,
    history_len: usize,
    hop: usize,
    decimation_at: usize,
    analysis_lowpass: Section,
    analysis_state: FilterState,
    quadrature: [Section; QUADRATURE_SECTIONS],
    quadrature_state: [FilterState; QUADRATURE_SECTIONS],
    threshold_db: f64,
    threshold_shape: [Section; BANDS],
    audio_wet_section: Section,
    tracks: Vec<Track>,
    release: f32,
    warmup: u32,
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize) -> Run {
        let fs = if fs.is_finite() { fs.clamp(8_000.0, 192_000.0) } else { 48_000.0 };
        let channels = channels.clamp(1, MAX_CHANNELS);
        // The AU publishes the saved count as a span including the current
        // frame: an 8 ms value at 48 kHz maps input frame 1 to output frame
        // 384.  A ring containing the full saved count maps it one frame
        // late, so retain one fewer prior frame.
        let lookahead = (libm::round(p.lookahead_s * fs).clamp(2.0, 2880.0) as usize) - 1;
        // The saved coefficient uses the 128-frame analysis hop with the
        // host-rate timebase even though one decision advances 4*128 host
        // frames.  Keep those two counts distinct.
        let release = (1.0 - libm::exp(-(ANALYSIS_HOP as f64) / (0.050 * fs))) as f32;
        let maximum_hz = MAX_HZ[p.operation_frequency.min(4) as usize].min(0.49 * fs);
        let window = core::array::from_fn(|n| {
            0.5f32 * (1.0 - libm::cosf(2.0 * core::f32::consts::PI * n as f32 / (FFT - 1) as f32)) / (FFT - 1) as f32
        });
        let fft_twiddles = core::array::from_fn(|k| {
            let angle = -2.0 * core::f64::consts::PI * k as f64 / FFT as f64;
            (libm::cos(angle) as f32, libm::sin(angle) as f32)
        });
        let mut run = Run {
            channels,
            fs,
            maximum_hz,
            lookahead,
            delay: vec![0.0; lookahead * channels].into_boxed_slice(),
            delay_at: 0,
            history: [0.0; FFT],
            window,
            fft_twiddles,
            cell_thresholds: [0.0; FFT / 2 + 1],
            bin_gains: [1.0; FFT / 2 + 1],
            history_at: 0,
            history_len: 0,
            hop: HOST_HOP,
            decimation_at: 0,
            analysis_lowpass: analysis_lowpass(maximum_hz, fs),
            analysis_state: FilterState::default(),
            quadrature: p.quadrature.map(|[b0, b1, b2, a1, a2]| Section {
                b0: b0 as f64,
                b1: b1 as f64,
                b2: b2 as f64,
                a1: a1 as f64,
                a2: a2 as f64,
            }),
            quadrature_state: [FilterState::default(); QUADRATURE_SECTIONS],
            threshold_db: p.threshold_db,
            threshold_shape: core::array::from_fn(|i| {
                let b = p.bands[i];
                if b.enabled {
                    threshold_peak(b.gain_db, b.frequency_hz, b.quality, fs)
                } else {
                    Section::IDENTITY
                }
            }),
            audio_wet_section: wet_smoother(fs),
            tracks: vec![Track::default(); TRACKS],
            release,
            warmup: WARMUP_DECISIONS,
        };
        for bin in 0..=FFT / 2 {
            run.cell_thresholds[bin] = run.cell_threshold(bin) as f32;
        }
        run
    }

    pub fn reset(&mut self) {
        self.delay.fill(0.0);
        self.delay_at = 0;
        self.history.fill(0.0);
        self.bin_gains.fill(1.0);
        self.history_at = 0;
        self.history_len = 0;
        self.hop = HOST_HOP;
        self.decimation_at = 0;
        self.analysis_state = FilterState::default();
        self.quadrature_state = [FilterState::default(); QUADRATURE_SECTIONS];
        self.tracks.fill(Track::default());
        self.warmup = WARMUP_DECISIONS;
    }
    pub fn latency(&self) -> usize {
        self.lookahead
    }

    fn threshold(&self, frequency_hz: f64) -> f64 {
        let db = self
            .threshold_shape
            .iter()
            .fold(self.threshold_db, |v, s| v + 20.0 * libm::log10(s.magnitude(frequency_hz, self.fs).max(1.0e-20)));
        libm::pow(10.0, db / 20.0)
    }

    /// The saved curve limits a whole transform cell: its lowest value over
    /// the cell's frequency span, from half a cell below its centre to half
    /// a cell above. The endpoints cover monotone spans; a short ternary
    /// search finds a Kill dip inside the cell.
    fn cell_threshold(&self, bin: usize) -> f64 {
        let width = ANALYSIS_RATE / FFT as f64;
        let mut lo = (bin as f64 - 0.5).max(0.0) * width;
        let mut hi = (bin as f64 + 0.5) * width;
        let mut minimum = self.threshold(lo).min(self.threshold(hi));
        for _ in 0..12 {
            let third = (hi - lo) / 3.0;
            let (left, right) = (lo + third, hi - third);
            let (at_left, at_right) = (self.threshold(left), self.threshold(right));
            minimum = minimum.min(at_left).min(at_right);
            if at_left < at_right {
                hi = right;
            } else {
                lo = left;
            }
        }
        minimum
    }

    /// Computes doubled positive-frequency amplitudes and first-difference
    /// refined frequencies for bins `0..=last_bin` in a single 512-point
    /// radix-2 complex FFT by packing the windowed signal in the real lane
    /// and its windowed first difference in the imaginary lane.
    fn spectra(&self, last_bin: usize, amplitudes: &mut [f64; FFT / 2 + 1], frequencies: &mut [f64; FFT / 2 + 1]) {
        let last_bin = last_bin.min(FFT / 2);
        if self.history_len == 0 {
            for bin in 0..=last_bin {
                amplitudes[bin] = 0.0;
                frequencies[bin] = bin as f64 * ANALYSIS_RATE / FFT as f64;
            }
            return;
        }
        const SHIFT: u32 = u16::BITS - FFT.trailing_zeros();
        let mut re = [0.0f32; FFT];
        let mut im = [0.0f32; FFT];
        for n in 0..FFT {
            let at = (self.history_at + n) & (FFT - 1);
            let previous = (at + FFT - 1) & (FFT - 1);
            let raw = self.history[at];
            let w = self.window[n];
            let rev = ((n as u16).reverse_bits() >> SHIFT) as usize;
            re[rev] = raw * w;
            im[rev] = (raw - self.history[previous]) * w;
        }
        let mut half = 1usize;
        while half < FFT {
            let len = half << 1;
            let step = FFT / len;
            let mut base = 0usize;
            while base < FFT {
                for j in 0..half {
                    let (wr, wi) = self.fft_twiddles[j * step];
                    let even = base + j;
                    let odd = even + half;
                    let or = re[odd];
                    let oi = im[odd];
                    let tr = wr * or - wi * oi;
                    let ti = wr * oi + wi * or;
                    let er = re[even];
                    let ei = im[even];
                    re[even] = er + tr;
                    im[even] = ei + ti;
                    re[odd] = er - tr;
                    im[odd] = ei - ti;
                }
                base += len;
            }
            half = len;
        }
        // DC (bin 0): X[0] = re[0], D[0] = im[0].
        let dc_mag = re[0].abs();
        let dc_q = im[0].abs() / (dc_mag + 1.0e-40f32);
        amplitudes[0] = (2.0f32 * dc_mag) as f64;
        frequencies[0] =
            (ANALYSIS_RATE / core::f64::consts::PI * libm::asin((0.5 * dc_q as f64).clamp(-1.0, 1.0))) as f32 as f64;
        for bin in 1..=last_bin {
            let nk = FFT - bin;
            let (x_re, x_im, d_re, d_im) = if bin == FFT / 2 {
                (re[bin], 0.0, im[bin], 0.0)
            } else {
                (0.5 * (re[bin] + re[nk]), 0.5 * (im[bin] - im[nk]), 0.5 * (im[bin] + im[nk]), 0.5 * (re[nk] - re[bin]))
            };
            let magnitude = libm::sqrtf(x_re * x_re + x_im * x_im);
            let q = libm::sqrtf(d_re * d_re + d_im * d_im) / (magnitude + 1.0e-40f32);
            // Discrete first-difference magnitude identity: |1 - e^{-j w}| = 2 sin(w / 2).
            let frequency_hz =
                (ANALYSIS_RATE / core::f64::consts::PI * libm::asin((0.5 * q as f64).clamp(-1.0, 1.0))) as f32;
            // The analyser compares the doubled positive-frequency magnitude
            // published by its real-input transform.  This is a peak-domain
            // comparison; it does not convert the doubled line to RMS.
            amplitudes[bin] = (2.0f32 * magnitude) as f64;
            frequencies[bin] = frequency_hz as f64;
        }
    }

    #[cfg(test)]
    fn spectrum(&self, bin: usize) -> (f64, f64) {
        let mut amplitudes = [0.0; FFT / 2 + 1];
        let mut frequencies = [0.0; FFT / 2 + 1];
        self.spectra(bin, &mut amplitudes, &mut frequencies);
        (amplitudes[bin], frequencies[bin])
    }

    fn redesign(&mut self, index: usize) {
        let t = &mut self.tracks[index];
        t.audio_section = audio_rejection(t.frequency_hz as f64, self.fs);
    }

    /// Moves one line's track against the pre-decision snapshot of tracks.
    /// The first active cutting track (frequency > 0, gain < ACTIVE_GAIN)
    /// within the match distance owns the line: if unused in this decision it
    /// retunes and continues by the line's deepest cell multiplier; if already
    /// used in this decision the candidate is blocked so no second track is
    /// started for the same spectral region. Otherwise the line takes over the
    /// most-released unused track above REPLACE_GAIN (nearest on ties): if a
    /// released track (ACTIVE_GAIN <= gain < 1) already sat at that frequency,
    /// the line continues from its deepest cell multiplier (retiring the stale
    /// slot); fresh lines start at the head cell's start multiplier.
    // The index also addresses self.tracks and self.redesign(), which an iterator over used cannot borrow.
    #[allow(clippy::needless_range_loop)]
    fn update_track(
        &mut self,
        frequency_hz: f64,
        multiplier: f64,
        start_multiplier: f64,
        snap_freq: &[f32; TRACKS],
        snap_gain: &[f32; TRACKS],
        used: &mut [bool; TRACKS],
    ) {
        for i in 0..TRACKS {
            if snap_freq[i] == 0.0 || snap_gain[i] >= ACTIVE_GAIN {
                continue;
            }
            let distance = (frequency_hz - snap_freq[i] as f64).abs() / frequency_hz;
            if distance < MATCH_RELATIVE {
                if !used[i] {
                    let t = &mut self.tracks[i];
                    t.frequency_hz = frequency_hz as f32;
                    t.gain = (t.gain * multiplier as f32).clamp(1.0e-6, 1.0);
                    used[i] = true;
                    self.redesign(i);
                }
                return;
            }
        }
        let mut released_match = None;
        for i in 0..TRACKS {
            if used[i] || snap_freq[i] == 0.0 || snap_gain[i] >= 1.0 {
                continue;
            }
            let distance = (frequency_hz - snap_freq[i] as f64).abs() / frequency_hz;
            if distance < MATCH_RELATIVE {
                released_match = Some(i);
                break;
            }
        }
        let mut replacement = None;
        let mut replacement_gain = 0.0f32;
        let mut replacement_distance = f64::INFINITY;
        for i in 0..TRACKS {
            if used[i] {
                continue;
            }
            let gain = self.tracks[i].gain;
            if gain <= REPLACE_GAIN as f32 {
                continue;
            }
            let distance = (frequency_hz - self.tracks[i].frequency_hz as f64).abs() / frequency_hz;
            if gain > replacement_gain || (gain == replacement_gain && distance < replacement_distance) {
                replacement = Some(i);
                replacement_gain = gain;
                replacement_distance = distance;
            }
        }
        if let Some(i) = replacement {
            let initial = if let Some(old) = released_match {
                if old != i {
                    self.tracks[old].frequency_hz = 0.0;
                }
                multiplier
            } else {
                start_multiplier
            };
            let t = &mut self.tracks[i];
            t.frequency_hz = frequency_hz as f32;
            t.gain = initial.clamp(1.0e-6, 1.0) as f32;
            used[i] = true;
            self.redesign(i);
        }
    }

    fn analyse(&mut self) {
        for track in &mut self.tracks {
            track.gain += self.release * (1.0 - track.gain);
        }
        // The transformed program uses integer DFT cells; retain one extra
        // cell for candidate neighbour work.
        let bin_count = analysed_cells(self.maximum_hz);
        let max_bin = bin_count.saturating_sub(1);
        let mut amplitudes = [0.0; FFT / 2 + 1];
        let mut frequencies = [0.0; FFT / 2 + 1];
        self.spectra(max_bin + 1, &mut amplitudes, &mut frequencies);
        // The detector's cell record releases toward one like a track.
        for gain in &mut self.bin_gains {
            *gain += self.release * (1.0 - *gain);
        }
        // Every over-threshold cell updates the cell record for the next
        // decision, from this decision's snapshot: its own cell by the
        // multiplier, its two neighbours by the square root. Candidates are
        // then taken in the order of their released cell gain (deepest
        // first; equal gains in ascending cell order), and one whose refined
        // frequency lies inside an earlier one's duplicate band is the same
        // line seen from a neighbouring cell: it moves no second track, but
        // the earlier one takes its multiplier when it is deeper.
        // Fixed storage: the decision runs on the audio thread and never allocates.
        let mut cell_updates = [1.0f32; FFT / 2 + 1];
        let mut pending = [Candidate::default(); TRACKS];
        let mut count = 0usize;
        for bin in 1..=max_bin {
            let threshold = self.cell_thresholds[bin];
            // The transform reads the dry program; what the sections remove
            // reaches the comparison through the cell record.
            let amplitude = amplitudes[bin] as f32 * self.bin_gains[bin];
            if amplitude <= threshold || !amplitude.is_finite() {
                continue;
            }
            let multiplier = (threshold / amplitude).clamp(1.0e-6, 1.0);
            let neighbour = libm::sqrtf(multiplier);
            cell_updates[bin] *= multiplier;
            cell_updates[bin - 1] *= neighbour;
            if bin + 1 < cell_updates.len() {
                cell_updates[bin + 1] *= neighbour;
            }
            let frequency_hz = frequencies[bin] as f32;
            if !(frequency_hz as f64 > MIN_HZ && (frequency_hz as f64) < self.maximum_hz) {
                continue;
            }
            if count < TRACKS {
                pending[count] = Candidate {
                    frequency_hz,
                    multiplier,
                    record_gain: self.bin_gains[bin],
                    start_multiplier: multiplier,
                };
                count += 1;
            }
        }
        // A stable insertion sort: the list is short and allocation-free.
        for i in 1..count {
            let mut j = i;
            while j > 0 && pending[j].record_gain < pending[j - 1].record_gain {
                pending.swap(j, j - 1);
                j -= 1;
            }
        }
        let lines = merge_same_line(&mut pending[..count]);
        let mut snap_freq = [0.0f32; TRACKS];
        let mut snap_gain = [1.0f32; TRACKS];
        for i in 0..TRACKS {
            snap_freq[i] = self.tracks[i].frequency_hz;
            snap_gain[i] = self.tracks[i].gain;
        }
        let mut used = [false; TRACKS];
        for c in &pending[..lines] {
            self.update_track(
                c.frequency_hz as f64,
                c.multiplier as f64,
                c.start_multiplier as f64,
                &snap_freq,
                &snap_gain,
                &mut used,
            );
        }
        for (gain, update) in self.bin_gains.iter_mut().zip(cell_updates) {
            *gain *= update;
        }
    }

    pub fn process(&mut self, buffers: &mut [&mut [f32]], frames: usize) {
        let channels = self.channels.min(buffers.len());
        let frames = frames.min(buffers.iter().take(channels).map(|b| b.len()).min().unwrap_or(0));
        let mut at = 0;
        while at < frames {
            // The native analyser returns one decision-sized block, updates
            // the tracks, and only then runs the host-rate rejection bank on
            // that block's delayed audio.  Buffer the transported samples in
            // place so the decision at this boundary governs this whole
            // look-ahead block rather than starting one block late.
            let count = (frames - at).min(self.hop);
            for frame in at..at + count {
                let mut linked = 0.0f32;
                for (channel, buffer) in buffers.iter_mut().take(channels).enumerate() {
                    let raw = if buffer[frame].is_finite() { buffer[frame] } else { 0.0 };
                    linked += raw;
                    let slot = self.delay_at * self.channels + channel;
                    buffer[frame] = self.delay[slot];
                    self.delay[slot] = raw;
                }
                linked /= channels as f32;
                self.delay_at = (self.delay_at + 1) % self.lookahead;
                let filtered = filter(self.analysis_lowpass, &mut self.analysis_state, ANALYSIS_GAIN * linked);
                // Keep the first of every four filtered samples, then pass the
                // 12 kHz stream through the amplitude-record cascade.
                if self.decimation_at == 0 {
                    let mut record = filtered;
                    for (section, state) in self.quadrature.iter().zip(&mut self.quadrature_state) {
                        record = filter(*section, state, record);
                    }
                    self.history[self.history_at] = record;
                    self.history_at = (self.history_at + 1) % FFT;
                    self.history_len = (self.history_len + 1).min(FFT);
                }
                self.decimation_at = (self.decimation_at + 1) % DECIMATION;
            }
            self.hop -= count;
            if self.hop == 0 {
                if self.warmup == 0 {
                    self.analyse();
                } else {
                    self.warmup -= 1;
                }
                self.hop = HOST_HOP;
            }
            let audio_wet_section = self.audio_wet_section;
            for frame in at..at + count {
                for (channel, buffer) in buffers.iter_mut().take(channels).enumerate() {
                    let mut delayed = buffer[frame];
                    for track in &mut self.tracks {
                        let target_wet = 1.0 - track.gain;
                        if target_wet > f32::EPSILON || track.audio_applied_wet[channel] > f32::EPSILON {
                            track.audio_applied_wet[channel] =
                                filter(audio_wet_section, &mut track.audio_wet[channel], target_wet);
                            let rejected = filter(
                                track.audio_section,
                                &mut track.audio[channel],
                                track.audio_applied_wet[channel] * delayed,
                            );
                            delayed -= rejected;
                        }
                    }
                    buffer[frame] = delayed;
                }
            }
            at += count;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn words_round_trip_and_reset() {
        let p = Params::default();
        let mut w = [0.0; Params::WORDS];
        p.to_words(&mut w);
        assert_eq!(Params::from_words(&w), Some(p));
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut x = [1.0; 1024];
        r.process(&mut [&mut x], 1024);
        r.reset();
        let mut z = [0.0; 512];
        r.process(&mut [&mut z], 512);
        assert!(z.iter().all(|v| *v == 0.0));
        assert_eq!(r.latency(), 383);
    }

    fn line(frequency_hz: f64, amplitude: f64, frames: usize) -> Vec<f32> {
        (0..frames)
            .map(|n| (amplitude * libm::sin(2.0 * core::f64::consts::PI * frequency_hz * n as f64 / 48_000.0)) as f32)
            .collect()
    }

    #[test]
    fn the_first_decision_uses_the_first_complete_block() {
        let p = Params { threshold_db: -100.0, ..Default::default() };
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut x = line(1000.0, 0.7, HOST_HOP);
        r.process(&mut [&mut x], HOST_HOP);
        assert!(r.tracks.iter().any(|t| t.gain < 1.0));
        assert!(r.bin_gains.iter().any(|g| *g < 1.0));
    }

    #[test]
    fn every_over_threshold_cell_deepens_the_record_and_its_neighbours() {
        let p = Params { threshold_db: -100.0, ..Default::default() };
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut x = line(1000.0, 0.7, 2 * HOST_HOP);
        r.process(&mut [&mut x], 2 * HOST_HOP);
        // one decision from an all-ones record: the deepest cell holds at most
        // the deepest track's multiplier, and both its neighbours moved too
        let m = r.tracks.iter().map(|t| t.gain).fold(1.0f32, f32::min);
        assert!(m < 0.5, "{m}");
        let (k, g) = r.bin_gains.iter().enumerate().fold((0, 1.0f32), |b, (k, g)| if *g < b.1 { (k, *g) } else { b });
        assert!(g <= m * 1.000_001, "{g} {m}");
        assert!(r.bin_gains[k - 1] < 1.0 && r.bin_gains[k + 1] < 1.0);
        assert!(r.bin_gains.iter().all(|g| *g > 0.0 && *g <= 1.0));
    }

    #[test]
    fn a_line_seen_from_two_cells_takes_the_deeper_multiplier() {
        let c = |frequency_hz, multiplier, record_gain| Candidate {
            frequency_hz,
            multiplier,
            record_gain,
            start_multiplier: multiplier,
        };
        // decision order: 1000 Hz first; 1010 Hz is the same line (inside
        // 1/20 octave) and deeper; 1100 Hz is another line
        let mut v = [c(1000.0, 0.5, 0.2), c(1010.0, 0.3, 0.4), c(1100.0, 0.8, 0.9), c(990.0, 0.6, 1.0)];
        let n = merge_same_line(&mut v);
        assert_eq!(n, 2);
        assert_eq!(
            (v[0].frequency_hz, v[0].multiplier, v[0].start_multiplier, v[0].record_gain),
            (1000.0, 0.3, 0.5, 0.2)
        );
        assert_eq!((v[1].frequency_hz, v[1].multiplier, v[1].start_multiplier), (1100.0, 0.8, 0.8));
        // a shallower same-line cell leaves the line's multiplier alone
        let mut w = [c(500.0, 0.2, 0.5), c(505.0, 0.9, 0.6)];
        assert_eq!(merge_same_line(&mut w), 1);
        assert_eq!((w[0].multiplier, w[0].start_multiplier), (0.2, 0.2));
        // the start multiplier stays the head cell's own multiplier
        let mut h = [c(245.0, 0.8, 1.0), c(250.0, 0.6, 1.0)];
        assert_eq!(merge_same_line(&mut h), 1);
        assert_eq!(h[0].multiplier, 0.6);
        assert_eq!(h[0].start_multiplier, 0.8);
        // the band edges are strict
        let mut e = [c(1000.0, 0.5, 0.5), c(1000.0 * CANDIDATE_UPPER, 0.1, 0.6)];
        assert_eq!(merge_same_line(&mut e), 2);
        assert_eq!(e[0].multiplier, 0.5);
    }

    fn snapshot(r: &Run) -> ([f32; TRACKS], [f32; TRACKS]) {
        let mut sf = [0.0f32; TRACKS];
        let mut sg = [1.0f32; TRACKS];
        for i in 0..TRACKS {
            sf[i] = r.tracks[i].frequency_hz;
            sg[i] = r.tracks[i].gain;
        }
        (sf, sg)
    }

    #[test]
    fn a_line_starts_a_track_at_its_start_multiplier_and_deepens_a_cutting_one() {
        let mut r = Run::new(&Params::default(), 48_000.0, 1);
        let gain_at = |r: &Run, f: f32| r.tracks.iter().find(|t| t.frequency_hz == f).map(|t| t.gain);
        // an unassigned idle track is not matched directly: replacement installs it at start_multiplier
        let (sf, sg) = snapshot(&r);
        let mut used = [false; TRACKS];
        r.update_track(1010.0, 0.3, 0.6, &sf, &sg, &mut used);
        assert_eq!(gain_at(&r, 1010.0), Some(0.6));
        // a second line far from active tracks takes another idle track at start_multiplier
        r.update_track(3000.0, 0.2, 0.7, &sf, &sg, &mut used);
        assert_eq!(gain_at(&r, 3000.0), Some(0.7));
        // the next decision continues the cutting track by the deepest multiplier
        let (sf, sg) = snapshot(&r);
        let mut used = [false; TRACKS];
        r.update_track(1000.0, 0.5, 0.9, &sf, &sg, &mut used);
        assert_eq!(gain_at(&r, 1000.0), Some(0.3));
        assert_eq!(r.tracks.iter().filter(|t| t.gain < 1.0).count(), 2);
    }

    #[test]
    fn a_new_line_takes_an_idle_track_before_one_that_still_cuts() {
        let mut r = Run::new(&Params::default(), 48_000.0, 1);
        r.tracks[0].frequency_hz = 2000.0;
        r.tracks[0].gain = 0.9;
        // 1900 Hz is outside the match distance of 2000 Hz; an idle track is taken over first
        let (sf, sg) = snapshot(&r);
        let mut used = [false; TRACKS];
        r.update_track(1900.0, 0.5, 0.85, &sf, &sg, &mut used);
        assert_eq!((r.tracks[0].frequency_hz, r.tracks[0].gain), (2000.0, 0.9));
        let started = r.tracks.iter().position(|t| t.frequency_hz == 1900.0).expect("started");
        assert_eq!(r.tracks[started].gain, 0.85);
        // a second line of the same decision, outside the match distance,
        // leaves the just-started track alone
        r.update_track(1830.0, 0.4, 0.8, &sf, &sg, &mut used);
        assert_eq!((r.tracks[started].frequency_hz, r.tracks[started].gain), (1900.0, 0.85));
        assert_eq!((r.tracks[0].frequency_hz, r.tracks[0].gain), (2000.0, 0.9));
        assert!(r.tracks.iter().any(|t| t.frequency_hz == 1830.0 && t.gain == 0.8));
        // once no idle track is left, the most released one is taken over
        for t in r.tracks.iter_mut().filter(|t| t.gain == 1.0) {
            t.gain = 0.95;
        }
        r.tracks[7].gain = 0.97;
        let (sf, sg) = snapshot(&r);
        let mut used = [false; TRACKS];
        r.update_track(5000.0, 0.5, 0.6, &sf, &sg, &mut used);
        assert_eq!((r.tracks[7].frequency_hz, r.tracks[7].gain), (5000.0, 0.6));
    }

    #[test]
    fn active_track_wins_over_released_ghost_and_blocks_duplicate() {
        let mut r = Run::new(&Params::default(), 48_000.0, 1);
        r.tracks[0].frequency_hz = 118.4;
        r.tracks[0].gain = 0.9999995;
        r.tracks[1].frequency_hz = 125.9;
        r.tracks[1].gain = 0.7;
        let (sf, sg) = snapshot(&r);
        let mut used = [false; TRACKS];
        // 122.5 Hz matches active track 1 rather than released track 0
        r.update_track(122.5, 0.8, 0.8, &sf, &sg, &mut used);
        assert_eq!((r.tracks[0].frequency_hz, r.tracks[0].gain), (118.4, 0.9999995));
        assert_eq!((r.tracks[1].frequency_hz, r.tracks[1].gain), (122.5, 0.56));
        // a second candidate still inside track 1's snapshot match distance is blocked
        r.update_track(128.0, 0.7, 0.7, &sf, &sg, &mut used);
        assert_eq!(r.tracks.iter().filter(|t| t.gain < 0.99).count(), 1);
    }

    #[test]
    fn released_track_match_continues_in_replacement_slot_and_retires_stale_slot() {
        let mut r = Run::new(&Params::default(), 48_000.0, 1);
        r.tracks[0].frequency_hz = 1600.0;
        r.tracks[0].gain = 0.9999995;
        let (sf, sg) = snapshot(&r);
        let mut used = [false; TRACKS];
        r.update_track(1605.0, 0.3, 0.7, &sf, &sg, &mut used);
        assert_eq!(r.tracks[0].frequency_hz, 0.0);
        let installed = r.tracks.iter().position(|t| t.frequency_hz == 1605.0).expect("installed");
        assert_ne!(installed, 0);
        assert_eq!(r.tracks[installed].gain, 0.3);
    }

    #[test]
    fn stereo_channel_linking_averages_channels() {
        // Out-of-phase identical sines cancel under arithmetic mean linking,
        // so no track activates even far above threshold.
        let p = Params { threshold_db: -40.0, ..Default::default() };
        let mut r = Run::new(&p, 48_000.0, 2);
        let mut left = line(1000.0, 0.8, 4 * HOST_HOP);
        let mut right: Vec<f32> = left.iter().map(|v| -v).collect();
        r.process(&mut [&mut left, &mut right], 4 * HOST_HOP);
        assert!(r.tracks.iter().all(|t| t.gain == 1.0));
        // A single-channel tone has half the amplitude of an in-phase dual-channel tone.
        let mut r_one = Run::new(&p, 48_000.0, 2);
        let mut l1 = line(1000.0, 0.8, 4 * HOST_HOP);
        let mut r1 = vec![0.0f32; 4 * HOST_HOP];
        r_one.process(&mut [&mut l1, &mut r1], 4 * HOST_HOP);
        let mut r_both = Run::new(&p, 48_000.0, 2);
        let mut l2 = line(1000.0, 0.4, 4 * HOST_HOP);
        let mut r2 = l2.clone();
        r_both.process(&mut [&mut l2, &mut r2], 4 * HOST_HOP);
        let min_one = r_one.tracks.iter().map(|t| t.gain).fold(1.0f32, f32::min);
        let min_both = r_both.tracks.iter().map(|t| t.gain).fold(1.0f32, f32::min);
        assert!((min_one - min_both).abs() < 1.0e-5, "{min_one} vs {min_both}");
    }

    #[test]
    fn threshold_shape_is_designed_at_host_rate() {
        let mut p = Params { threshold_db: -20.0, ..Default::default() };
        p.bands[0] = Band { enabled: true, frequency_hz: 2000.0, gain_db: -12.0, quality: 2.0 };
        let r = Run::new(&p, 48_000.0, 1);
        let host_sec = threshold_peak(-12.0, 2000.0, 2.0, 48_000.0);
        let expected = libm::pow(10.0, (-20.0 + 20.0 * libm::log10(host_sec.magnitude(2000.0, 48_000.0))) / 20.0);
        assert!((r.threshold(2000.0) - expected).abs() < 1.0e-12);
    }

    #[test]
    fn first_difference_refinement_recovers_high_frequency_tones() {
        // At 2000 Hz (12 kHz analysis rate, w = pi/3), exact arcsin recovers 2000 Hz
        // without the small-angle bias of q^2 / sqrt(2*(1 - cos(q))).
        let mut r = Run::new(&Params::default(), 48_000.0, 1);
        for (n, slot) in r.history.iter_mut().enumerate().take(FFT) {
            *slot = libm::sin(2.0 * core::f64::consts::PI * 2000.0 * n as f64 / ANALYSIS_RATE) as f32;
        }
        r.history_len = FFT;
        let bin = libm::round(2000.0 * FFT as f64 / ANALYSIS_RATE) as usize;
        let (_amp, freq) = r.spectrum(bin);
        assert!((freq - 2000.0).abs() < 1.0, "expected ~2000 Hz, got {freq}");
    }

    #[test]
    fn packed_fft_spectra_matches_direct_dft() {
        let mut r = Run::new(&Params::default(), 48_000.0, 1);
        for (n, slot) in r.history.iter_mut().enumerate().take(FFT) {
            let t = n as f64 / ANALYSIS_RATE;
            *slot = (0.6 * libm::sin(2.0 * core::f64::consts::PI * 333.0 * t)
                + 0.35 * libm::cos(2.0 * core::f64::consts::PI * 1777.0 * t + 0.4)
                + 0.15 * libm::sin(2.0 * core::f64::consts::PI * 4123.0 * t)) as f32;
        }
        r.history_at = 73;
        r.history_len = FFT;
        let mut amps = [0.0; FFT / 2 + 1];
        let mut freqs = [0.0; FFT / 2 + 1];
        r.spectra(FFT / 2, &mut amps, &mut freqs);
        for bin in 0..=FFT / 2 {
            let (mut re, mut im, mut dre, mut dim) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
            for n in 0..FFT {
                let at = (r.history_at + n) % FFT;
                let prev = (at + FFT - 1) % FFT;
                let w = r.window[n] as f64;
                let raw = r.history[at] as f64;
                let x = raw * w;
                let d = (raw - r.history[prev] as f64) * w;
                let ang = 2.0 * core::f64::consts::PI * (bin * n) as f64 / FFT as f64;
                let (s, c) = (libm::sin(ang), libm::cos(ang));
                re += x * c;
                im -= x * s;
                dre += d * c;
                dim -= d * s;
            }
            let mag = libm::sqrt(re * re + im * im);
            let exp_amp = 2.0 * mag;
            assert!((amps[bin] - exp_amp).abs() < 1.0e-5, "bin {bin}: {} vs {exp_amp}", amps[bin]);
            if exp_amp > 1.0e-3 {
                let q = libm::sqrt(dre * dre + dim * dim) / (mag + 1.0e-40);
                let exp_freq = ANALYSIS_RATE / core::f64::consts::PI * libm::asin((0.5 * q).clamp(-1.0, 1.0));
                assert!((freqs[bin] - exp_freq).abs() < 0.05, "bin {bin}: {} vs {exp_freq}", freqs[bin]);
            }
        }
    }

    #[test]
    fn one_steady_line_keeps_one_deep_track() {
        // Neighbouring cells see the same line; only one track follows it.
        let p = Params { threshold_db: -20.0, ..Default::default() };
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut x = line(996.09375, 0.7, 96_000);
        r.process(&mut [&mut x], 96_000);
        let deep = r.tracks.iter().filter(|t| t.gain < 0.5 && (t.frequency_hz - 996.09375).abs() < 50.0).count();
        assert_eq!(deep, 1);
    }

    #[test]
    fn only_cells_that_end_below_the_operation_limit_are_analysed() {
        let width = ANALYSIS_RATE / FFT as f64;
        for limit in MAX_HZ {
            let n = analysed_cells(limit.min(0.49 * 48_000.0));
            // Cell k spans k..k+1 widths: every analysed cell ends at or
            // below the limit, and the next one would not.
            assert!((n as f64) * width <= limit);
            assert!(n == FFT / 2 || ((n + 1) as f64) * width > limit);
        }
        // 2500 Hz is 106.7 widths: cells 0..=105.
        assert_eq!(analysed_cells(MAX_HZ[0]), 106);
    }

    #[test]
    fn a_full_level_line_crosses_the_analysis_threshold() {
        let p = Params { threshold_db: -9.75, ..Default::default() };
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut x = [0.0; 4096];
        for (frame, sample) in x.iter_mut().enumerate() {
            *sample = libm::sin(2.0 * core::f64::consts::PI * 1600.0 * frame as f64 / 48_000.0) as f32;
        }
        r.process(&mut [&mut x], 4096);
        assert!(r.tracks.iter().any(|track| track.gain < 0.7));
    }

    #[test]
    fn rejection_uses_constant_quality_in_each_rate_domain() {
        for fs in [ANALYSIS_RATE, 48_000.0] {
            let f = 120.0;
            let section = rejection(f, NOTCH_QUALITY, fs);
            assert!((section.magnitude(f, fs) - 1.0).abs() < 1.0e-10);
            let half_bandwidth = f / (2.0 * NOTCH_QUALITY);
            assert!((section.magnitude(f + half_bandwidth, fs) - core::f64::consts::FRAC_1_SQRT_2).abs() < 0.02);
        }
    }

    #[test]
    fn host_rate_rejection_rings_at_the_analysis_rate() {
        let decay = |s: Section, fs: f64| -1.0 / (fs * 0.5 * libm::log(s.a2));
        for f in [250.0, 1000.0, 2000.0] {
            let analysis = decay(rejection(f, NOTCH_QUALITY, ANALYSIS_RATE), ANALYSIS_RATE);
            let host = audio_rejection(f, 48_000.0);
            assert!((decay(host, 48_000.0) / analysis - 1.0).abs() < 1.0e-9);
            assert!((host.magnitude(f, 48_000.0) - 1.0).abs() < 1.0e-9);
        }
        let constant = decay(rejection(2000.0, NOTCH_QUALITY, 48_000.0), 48_000.0);
        let ratio = decay(audio_rejection(2000.0, 48_000.0), 48_000.0) / constant;
        assert!((ratio - 1.1935).abs() < 0.001, "{ratio}");
    }
}
