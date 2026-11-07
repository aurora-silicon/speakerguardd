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
const HOST_HOP: usize = 128;
const ANALYSIS_RATE: f64 = 12_000.0;
const DECIMATION: usize = 4;
const FIR: usize = 97;
const MIN_HZ: f64 = 50.0;
const MAX_HZ: [f64; 5] = [2500.0, 3000.0, 4000.0, 6000.0, 20_000.0];
const MATCH_RELATIVE: f64 = 0.035_264_92;
const REPLACE_GAIN: f64 = 0.82;
const NOTCH_BANDWIDTH_HZ: f64 = 6.0;

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
    z1: f64,
    z2: f64,
}

fn filter(section: Section, state: &mut FilterState, x: f64) -> f64 {
    let y = section.b0 * x + state.z1;
    state.z1 = section.b1 * x - section.a1 * y + state.z2;
    state.z2 = section.b2 * x - section.a2 * y;
    y
}

#[derive(Clone)]
struct Track {
    frequency_hz: f64,
    gain: f64,
    section: Section,
    audio: [FilterState; MAX_CHANNELS],
    analysis: [FilterState; MAX_CHANNELS],
}

impl Default for Track {
    fn default() -> Self {
        Track {
            frequency_hz: 1000.0,
            gain: 1.0,
            section: Section::IDENTITY,
            audio: [FilterState::default(); MAX_CHANNELS],
            analysis: [FilterState::default(); MAX_CHANNELS],
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
    history_at: usize,
    history_len: usize,
    hop: usize,
    decimation_at: usize,
    fir: [f64; FIR],
    fir_state: [f64; FIR],
    fir_at: usize,
    threshold_db: f64,
    threshold_shape: [Section; BANDS],
    tracks: Vec<Track>,
    release: f64,
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize) -> Run {
        let fs = if fs.is_finite() { fs.clamp(8_000.0, 192_000.0) } else { 48_000.0 };
        let channels = channels.clamp(1, MAX_CHANNELS);
        let lookahead = libm::round(p.lookahead_s * fs).clamp(1.0, 2880.0) as usize;
        let release = 1.0 - libm::exp(-(HOST_HOP as f64) / (0.050 * fs));
        Run {
            channels,
            fs,
            maximum_hz: MAX_HZ[p.operation_frequency.min(4) as usize].min(0.49 * fs),
            lookahead,
            delay: vec![0.0; lookahead * channels].into_boxed_slice(),
            delay_at: 0,
            history: [0.0; FFT],
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
                    super::tone::peak(b.gain_db, b.frequency_hz, b.quality, fs)
                } else {
                    Section::IDENTITY
                }
            }),
            tracks: vec![Track::default(); TRACKS],
            release,
        }
    }

    pub fn reset(&mut self) {
        self.delay.fill(0.0);
        self.delay_at = 0;
        self.history.fill(0.0);
        self.history_at = 0;
        self.history_len = 0;
        self.hop = HOST_HOP;
        self.decimation_at = 0;
        self.fir_state.fill(0.0);
        self.fir_at = 0;
        self.tracks.fill(Track::default());
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

    fn spectrum(&self, bin: usize) -> (f64, f64) {
        if self.history_len == 0 {
            return (0.0, bin as f64 * ANALYSIS_RATE / FFT as f64);
        }
        let step = 2.0 * core::f64::consts::PI * bin as f64 / FFT as f64;
        let (ss, cc) = (libm::sin(step), libm::cos(step));
        let (mut s, mut c, mut re, mut im, mut dre, mut dim) = (0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        for n in 0..FFT {
            let window =
                0.5 * (1.0 - libm::cos(2.0 * core::f64::consts::PI * n as f64 / (FFT - 1) as f64)) / (FFT - 1) as f64;
            let at = (self.history_at + n) % FFT;
            let previous = (at + FFT - 1) % FFT;
            let raw = self.history[at] as f64;
            let x = raw * window;
            let difference = (raw - self.history[previous] as f64) * window;
            re += x * c;
            im -= x * s;
            dre += difference * c;
            dim -= difference * s;
            (s, c) = (s * cc + c * ss, c * cc - s * ss);
        }
        let magnitude = libm::sqrt(re * re + im * im);
        let q = libm::sqrt(dre * dre + dim * dim) / (magnitude + 1.0e-20);
        let denominator = libm::sqrt(2.0 * (1.0 - libm::cos(q))).max(1.0e-20);
        let frequency_hz = ANALYSIS_RATE / (2.0 * core::f64::consts::PI) * q * q / denominator;
        // A real-input transform publishes twice the positive-frequency
        // complex magnitude. Converting that convention to line RMS leaves
        // sqrt(2) times the direct complex sum used here.
        (core::f64::consts::SQRT_2 * magnitude, frequency_hz)
    }

    fn redesign(&mut self, index: usize) {
        let t = &mut self.tracks[index];
        t.section = if t.gain >= 0.999_999 {
            Section::IDENTITY
        } else {
            super::tone::peak(
                20.0 * libm::log10(t.gain.max(1.0e-6)),
                t.frequency_hz,
                (t.frequency_hz / NOTCH_BANDWIDTH_HZ).max(0.5),
                self.fs,
            )
        };
    }

    // The index also addresses self.tracks and self.redesign(), which an iterator over used cannot borrow.
    #[allow(clippy::needless_range_loop)]
    fn update_track(&mut self, frequency_hz: f64, multiplier: f64, used: &mut [bool; TRACKS]) {
        let mut replacement = None;
        let mut replacement_distance = f64::INFINITY;
        for i in 0..TRACKS {
            let distance = (frequency_hz - self.tracks[i].frequency_hz).abs() / frequency_hz;
            if distance < MATCH_RELATIVE && !used[i] {
                let t = &mut self.tracks[i];
                t.frequency_hz = frequency_hz;
                t.gain = (t.gain * multiplier).clamp(1.0e-6, 1.0);
                used[i] = true;
                self.redesign(i);
                return;
            }
            if self.tracks[i].gain > REPLACE_GAIN && distance < replacement_distance {
                replacement = Some(i);
                replacement_distance = distance;
            }
        }
        if let Some(i) = replacement {
            let t = &mut self.tracks[i];
            t.frequency_hz = frequency_hz;
            t.gain = multiplier.clamp(1.0e-6, 1.0);
            used[i] = true;
            self.redesign(i);
        }
    }

    fn analyse(&mut self) {
        for i in 0..TRACKS {
            self.tracks[i].gain += self.release * (1.0 - self.tracks[i].gain);
            self.redesign(i);
        }
        let max_bin = (libm::ceil(self.maximum_hz * FFT as f64 / ANALYSIS_RATE) as usize).min(FFT / 2 - 1);
        let mut amplitudes = [0.0; FFT / 2 + 1];
        let mut frequencies = [0.0; FFT / 2 + 1];
        for (bin, value) in amplitudes.iter_mut().enumerate().take(max_bin + 2) {
            (*value, frequencies[bin]) = self.spectrum(bin);
        }
        let mut used = [false; TRACKS];
        let mut candidates = 0usize;
        for bin in 1..=max_bin {
            let frequency = bin as f64 * ANALYSIS_RATE / FFT as f64;
            let threshold = self.threshold(frequency);
            let amplitude = amplitudes[bin];
            if frequency < MIN_HZ || amplitude <= threshold || !amplitude.is_finite() {
                continue;
            }
            let frequency_hz = frequencies[bin];
            if (MIN_HZ..=self.maximum_hz).contains(&frequency_hz) {
                self.update_track(frequency_hz, (threshold / amplitude).clamp(1.0e-6, 1.0), &mut used);
                candidates += 1;
                if candidates == TRACKS {
                    break;
                }
            }
        }
    }

    pub fn process(&mut self, buffers: &mut [&mut [f32]], frames: usize) {
        let channels = self.channels.min(buffers.len());
        let frames = frames.min(buffers.iter().take(channels).map(|b| b.len()).min().unwrap_or(0));
        for frame in 0..frames {
            let mut linked = 0.0f64;
            for (channel, buffer) in buffers.iter_mut().take(channels).enumerate() {
                let raw = if buffer[frame].is_finite() { buffer[frame] as f64 } else { 0.0 };
                let mut detected = raw;
                for track in &mut self.tracks {
                    detected = filter(track.section, &mut track.analysis[channel], detected);
                }
                if detected.abs() > linked.abs() {
                    linked = detected;
                }
                let slot = self.delay_at * self.channels + channel;
                let mut delayed = self.delay[slot] as f64;
                self.delay[slot] = raw as f32;
                for track in &mut self.tracks {
                    delayed = filter(track.section, &mut track.audio[channel], delayed);
                }
                buffer[frame] = delayed as f32;
            }
            self.delay_at = (self.delay_at + 1) % self.lookahead;
            self.fir_state[self.fir_at] = linked;
            self.fir_at = (self.fir_at + 1) % FIR;
            self.decimation_at += 1;
            if self.decimation_at == DECIMATION {
                let mut filtered = 0.0;
                for (tap, coefficient) in self.fir.iter().enumerate() {
                    filtered += coefficient * self.fir_state[(self.fir_at + FIR - 1 - tap) % FIR];
                }
                self.history[self.history_at] = filtered as f32;
                self.history_at = (self.history_at + 1) % FFT;
                self.history_len = (self.history_len + 1).min(FFT);
                self.decimation_at = 0;
            }
            self.hop -= 1;
            if self.hop == 0 {
                self.analyse();
                self.hop = HOST_HOP;
            }
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
        assert_eq!(r.latency(), 384);
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
}
