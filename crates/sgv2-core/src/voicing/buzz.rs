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
const FIR: usize = 97;
const MIN_HZ: f64 = 50.0;
const MAX_HZ: [f64; 5] = [2500.0, 3000.0, 4000.0, 6000.0, 20_000.0];
const MATCH_RELATIVE: f64 = 0.035_264_92;
const CANDIDATE_LOWER: f32 = 0.965_936_3;
const CANDIDATE_UPPER: f32 = 1.035_265;
const REPLACE_GAIN: f64 = 0.82;
const NOTCH_QUALITY: f64 = 6.0;
const WET_SMOOTH_HZ: f64 = 5.0;
/// The analyser makes its first decision once the history holds two
/// decision blocks: the first boundary after reset passes without one.
const WARMUP_DECISIONS: u32 = 1;

/// One line found by a decision.
#[derive(Clone, Copy, Default)]
struct Candidate {
    frequency_hz: f32,
    multiplier: f32,
    bin: usize,
}

// The analysis path is a 4:1, 97-tap low-pass resampler. Its gain of two is
// intentional: the transformed-spectrum thresholds use this amplitude domain.
// Normalising these coefficients to unity leaves full-level lines below the
// saved threshold and prevents the tracker from engaging.
const FIR_COEFFICIENTS: [f64; FIR] = [
    1.72128143e-20,
    1.13013757e-05,
    2.97373317e-05,
    4.88926635e-05,
    5.23911403e-05,
    2.08011879e-05,
    -5.71931487e-05,
    -0.000170382104,
    -0.000275706348,
    -0.000303400098,
    -0.000182146425,
    0.000119562086,
    0.000551495876,
    0.000963210419,
    0.00113146496,
    0.00084096007,
    -1.36345804e-18,
    -0.00125418056,
    -0.00251949672,
    -0.00320996833,
    -0.00276050158,
    -0.000903327367,
    0.0020905328,
    0.00533255469,
    0.00749515882,
    0.00725257676,
    0.00386968628,
    -0.00227897777,
    -0.00951246265,
    -0.015152527,
    -0.0163532551,
    -0.0112452758,
    8.49141844e-18,
    0.0146492897,
    0.027784884,
    0.0336582139,
    0.0277282801,
    0.00876332633,
    -0.019767249,
    -0.049669344,
    -0.0696450695,
    -0.068290472,
    -0.0376776122,
    0.0235835277,
    0.108866006,
    0.204281732,
    0.291651368,
    0.352952898,
    0.374997437,
    0.352952898,
    0.291651368,
    0.204281732,
    0.108866006,
    0.0235835258,
    -0.0376776122,
    -0.0682904795,
    -0.0696450546,
    -0.049669344,
    -0.0197672509,
    0.0087633254,
    0.0277282801,
    0.0336582251,
    0.0277848784,
    0.0146492915,
    8.49142009e-18,
    -0.011245274,
    -0.0163532551,
    -0.0151525298,
    -0.00951245985,
    -0.00227897801,
    0.00386968697,
    0.00725257582,
    0.00749515928,
    0.00533255469,
    0.0020905328,
    -0.000903327425,
    -0.00276050158,
    -0.00320996833,
    -0.00251949672,
    -0.00125418056,
    -1.36345804e-18,
    0.00084096007,
    0.00113146461,
    0.000963210594,
    0.000551495876,
    0.000119562086,
    -0.00018214641,
    -0.000303400069,
    -0.000275706407,
    -0.000170382074,
    -5.71931487e-05,
    2.08011879e-05,
    5.23911403e-05,
    4.88926635e-05,
    2.97373335e-05,
    1.13013757e-05,
    1.72128159e-20,
];

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
}

impl Default for Params {
    fn default() -> Self {
        Params {
            operation_frequency: 0,
            lookahead_s: 0.008,
            threshold_db: -20.0,
            algorithm: 0,
            bands: [Band::default(); BANDS],
        }
    }
}

impl Params {
    pub const WORDS: usize = 4 + 4 * BANDS;
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
            frequency_hz: 1000.0,
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
    /// Per-cell gain record of the detector: what the moving sections have
    /// removed from each transform cell, as the analyser sees it.
    bin_gains: [f32; FFT / 2 + 1],
    history_at: usize,
    history_len: usize,
    hop: usize,
    decimation_at: usize,
    fir: [f64; FIR],
    fir_state: [f32; FIR],
    fir_at: usize,
    threshold_db: f64,
    threshold_shape: [Section; BANDS],
    tracks: Vec<Track>,
    release: f32,
    /// Decision boundaries still to pass before the first decision.
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
        Run {
            channels,
            fs,
            maximum_hz: MAX_HZ[p.operation_frequency.min(4) as usize].min(0.49 * fs),
            lookahead,
            delay: vec![0.0; lookahead * channels].into_boxed_slice(),
            delay_at: 0,
            history: [0.0; FFT],
            bin_gains: [1.0; FFT / 2 + 1],
            history_at: 0,
            history_len: 0,
            hop: HOST_HOP,
            decimation_at: 0,
            fir: FIR_COEFFICIENTS,
            fir_state: [0.0; FIR],
            fir_at: 0,
            threshold_db: p.threshold_db,
            threshold_shape: core::array::from_fn(|i| {
                let b = p.bands[i];
                if b.enabled {
                    threshold_peak(b.gain_db, b.frequency_hz, b.quality, ANALYSIS_RATE)
                } else {
                    Section::IDENTITY
                }
            }),
            tracks: vec![Track::default(); TRACKS],
            release,
            warmup: WARMUP_DECISIONS,
        }
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
        self.fir_state.fill(0.0);
        self.fir_at = 0;
        self.tracks.fill(Track::default());
        self.warmup = WARMUP_DECISIONS;
    }
    pub fn latency(&self) -> usize {
        self.lookahead
    }

    fn threshold(&self, frequency_hz: f64) -> f64 {
        let db = self.threshold_shape.iter().fold(self.threshold_db, |v, s| {
            v + 20.0 * libm::log10(s.magnitude(frequency_hz, ANALYSIS_RATE).max(1.0e-20))
        });
        libm::pow(10.0, db / 20.0)
    }

    fn spectrum(&self, bin: usize) -> (f64, f64) {
        let centre = bin as f32;
        if self.history_len == 0 {
            return (0.0, centre as f64 * ANALYSIS_RATE / FFT as f64);
        }
        let step = 2.0f32 * core::f32::consts::PI * centre / FFT as f32;
        let (ss, cc) = (libm::sinf(step), libm::cosf(step));
        let (mut s, mut c, mut re, mut im, mut dre, mut dim) = (0.0f32, 1.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32);
        for n in 0..FFT {
            let window = 0.5f32 * (1.0 - libm::cosf(2.0 * core::f32::consts::PI * n as f32 / (FFT - 1) as f32))
                / (FFT - 1) as f32;
            let at = (self.history_at + n) % FFT;
            let previous = (at + FFT - 1) % FFT;
            let raw = self.history[at];
            let x = raw * window;
            let difference = (raw - self.history[previous]) * window;
            re += x * c;
            im -= x * s;
            dre += difference * c;
            dim -= difference * s;
            (s, c) = (s * cc + c * ss, c * cc - s * ss);
        }
        let magnitude = libm::sqrtf(re * re + im * im);
        let q = libm::sqrtf(dre * dre + dim * dim) / (magnitude + 1.0e-40f32);
        let denominator = libm::sqrt(2.0 * (1.0 - libm::cos(q as f64))).max(1.0e-20);
        let frequency_hz = (ANALYSIS_RATE / (2.0 * core::f64::consts::PI) * (q * q) as f64 / denominator) as f32;
        // The analyser compares the doubled positive-frequency magnitude
        // published by its real-input transform.  This is a peak-domain
        // comparison; it does not convert the doubled line to RMS.
        ((2.0f32 * magnitude) as f64, frequency_hz as f64)
    }

    fn redesign(&mut self, index: usize) {
        let t = &mut self.tracks[index];
        t.audio_section = rejection(t.frequency_hz as f64, NOTCH_QUALITY, self.fs);
    }

    // The index also addresses self.tracks and self.redesign(), which an iterator over used cannot borrow.
    #[allow(clippy::needless_range_loop)]
    fn update_track(&mut self, frequency_hz: f64, multiplier: f64, used: &mut [bool; TRACKS]) {
        let mut replacement = None;
        let mut replacement_distance = f64::INFINITY;
        for i in 0..TRACKS {
            let distance = (frequency_hz - self.tracks[i].frequency_hz as f64).abs() / frequency_hz;
            if distance < MATCH_RELATIVE && !used[i] {
                let t = &mut self.tracks[i];
                t.frequency_hz = frequency_hz as f32;
                t.gain = (t.gain * multiplier as f32).clamp(1.0e-6, 1.0);
                used[i] = true;
                self.redesign(i);
                return;
            }
            if self.tracks[i].gain > REPLACE_GAIN as f32 && distance < replacement_distance {
                replacement = Some(i);
                replacement_distance = distance;
            }
        }
        if let Some(i) = replacement {
            let t = &mut self.tracks[i];
            t.frequency_hz = frequency_hz as f32;
            t.gain = multiplier.clamp(1.0e-6, 1.0) as f32;
            used[i] = true;
            self.redesign(i);
        }
    }

    fn analyse(&mut self) {
        for i in 0..TRACKS {
            self.tracks[i].gain += self.release * (1.0 - self.tracks[i].gain);
            self.redesign(i);
        }
        // The saved threshold curve is sampled at k + 1/2 even though the
        // transformed program uses integer DFT cells. Keep the upper edge
        // strict and retain one extra cell for candidate neighbour work.
        let bin_count = (libm::ceil(self.maximum_hz * FFT as f64 / ANALYSIS_RATE - 0.5) as usize).min(FFT / 2);
        let max_bin = bin_count.saturating_sub(1);
        let mut amplitudes = [0.0; FFT / 2 + 1];
        let mut frequencies = [0.0; FFT / 2 + 1];
        for (bin, value) in amplitudes.iter_mut().enumerate().take(max_bin + 2) {
            let (amplitude, frequency) = self.spectrum(bin);
            *value = amplitude;
            frequencies[bin] = frequency;
        }
        // The detector's cell record releases toward one like a track.
        for gain in &mut self.bin_gains {
            *gain += self.release * (1.0 - *gain);
        }
        // Over-threshold cells in ascending order. A cell whose refined
        // frequency lies inside an earlier candidate's duplicate band is the
        // same line seen from a neighbouring cell: it joins that candidate,
        // which keeps the deepest multiplier and the cell it came from, and
        // the first cell's frequency. (Black-box: with the first cell's
        // multiplier, lines between two cells, e.g. 630 Hz and 1 kHz, were
        // cut 2-5 dB less than macOS; with the deepest one they match
        // within 0.1 dB.)
        // Fixed storage: the decision runs on the audio thread and never allocates.
        let mut candidates = [Candidate::default(); TRACKS];
        let mut count = 0usize;
        for bin in 1..=max_bin {
            let frequency = (bin as f64 + 0.5) * ANALYSIS_RATE / FFT as f64;
            let threshold = self.threshold(frequency) as f32;
            // The transform reads the dry program; what the sections remove
            // reaches the comparison through the cell record.
            let amplitude = amplitudes[bin] as f32 * self.bin_gains[bin];
            if amplitude <= threshold || !amplitude.is_finite() {
                continue;
            }
            let multiplier = (threshold / amplitude).clamp(1.0e-6, 1.0);
            let frequency_hz = frequencies[bin] as f32;
            if !(frequency_hz as f64 > MIN_HZ && (frequency_hz as f64) < self.maximum_hz) {
                continue;
            }
            let same_line = candidates[..count].iter_mut().find(|c| {
                frequency_hz > c.frequency_hz * CANDIDATE_LOWER && frequency_hz < c.frequency_hz * CANDIDATE_UPPER
            });
            if let Some(c) = same_line {
                if multiplier < c.multiplier {
                    c.multiplier = multiplier;
                    c.bin = bin;
                }
            } else if count < TRACKS {
                candidates[count] = Candidate { frequency_hz, multiplier, bin };
                count += 1;
            }
        }
        // The cell record is updated from this decision's candidates after
        // all of them were formed: the candidate's cell by its multiplier,
        // its two neighbours by the square root.
        let mut cell_updates = [1.0f32; FFT / 2 + 1];
        let mut used = [false; TRACKS];
        for c in candidates.into_iter().take(count) {
            let neighbour = libm::sqrtf(c.multiplier);
            cell_updates[c.bin] *= c.multiplier;
            cell_updates[c.bin - 1] *= neighbour;
            if c.bin + 1 < cell_updates.len() {
                cell_updates[c.bin + 1] *= neighbour;
            }
            self.update_track(c.frequency_hz as f64, c.multiplier as f64, &mut used);
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
                    if raw.abs() > linked.abs() {
                        linked = raw;
                    }
                    let slot = self.delay_at * self.channels + channel;
                    buffer[frame] = self.delay[slot];
                    self.delay[slot] = raw;
                }
                self.delay_at = (self.delay_at + 1) % self.lookahead;
                self.fir_state[self.fir_at] = linked;
                self.fir_at = (self.fir_at + 1) % FIR;
                self.decimation_at += 1;
                if self.decimation_at == DECIMATION {
                    let mut filtered = 0.0f32;
                    for (tap, coefficient) in self.fir.iter().enumerate() {
                        filtered += *coefficient as f32 * self.fir_state[(self.fir_at + FIR - 1 - tap) % FIR];
                    }
                    self.history[self.history_at] = filtered;
                    self.history_at = (self.history_at + 1) % FFT;
                    self.history_len = (self.history_len + 1).min(FFT);
                    self.decimation_at = 0;
                }
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
            let audio_wet_section = wet_smoother(self.fs);
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
    fn the_first_decision_waits_for_two_blocks() {
        let p = Params { threshold_db: -20.0, ..Default::default() };
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut x = line(1000.0, 0.7, HOST_HOP);
        r.process(&mut [&mut x], HOST_HOP);
        assert!(r.tracks.iter().all(|t| t.gain == 1.0));
        assert!(r.bin_gains.iter().all(|g| *g == 1.0));
        let mut x = line(1000.0, 0.7, HOST_HOP);
        r.process(&mut [&mut x], HOST_HOP);
        assert!(r.tracks.iter().any(|t| t.gain < 1.0));
    }

    #[test]
    fn the_cell_record_takes_the_multiplier_and_its_neighbours_the_square_root() {
        let p = Params { threshold_db: -20.0, ..Default::default() };
        let mut r = Run::new(&p, 48_000.0, 1);
        let mut x = line(1000.0, 0.7, 2 * HOST_HOP);
        r.process(&mut [&mut x], 2 * HOST_HOP);
        // one decision: every cell started at one, so the deepest track's
        // multiplier (or less, where a neighbouring candidate added its root) is
        // in a cell whose two neighbours carry at least its square root
        let m = r.tracks.iter().map(|t| t.gain).fold(1.0f32, f32::min);
        assert!(m < 0.5, "{m}");
        let k = r.bin_gains.iter().position(|g| *g <= m * 1.000_001).expect("the candidate cell");
        for j in [k - 1, k + 1] {
            assert!(r.bin_gains[j] <= libm::sqrtf(m) * 1.000_001, "{j} {} {m}", r.bin_gains[j]);
        }
        assert!(r.bin_gains.iter().all(|g| *g > 0.0 && *g <= 1.0));
    }

    #[test]
    fn a_line_between_two_cells_is_cut_like_one_on_a_cell() {
        // 12 kHz / 512: cells every 23.4375 Hz; 1007.8 Hz sits on cell 43, 996 Hz between 42 and 43
        let p = Params { threshold_db: -20.0, ..Default::default() };
        let depth = |f: f64| {
            let mut r = Run::new(&p, 48_000.0, 1);
            let mut x = line(f, 0.7, 96_000);
            r.process(&mut [&mut x], 96_000);
            let tail = &x[96_000 - 24_000..];
            let rms = libm::sqrt(tail.iter().map(|v| (*v as f64) * (*v as f64)).sum::<f64>() / tail.len() as f64);
            20.0 * libm::log10(rms / (0.7 / core::f64::consts::SQRT_2))
        };
        let on_cell = depth(1007.8125);
        let between = depth(996.09375);
        assert!(on_cell < -6.0, "{on_cell}");
        assert!((on_cell - between).abs() < 1.5, "{on_cell} {between}");
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
}
