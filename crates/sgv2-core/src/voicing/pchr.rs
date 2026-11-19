// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! A channel matrix used for fixed voicing transforms.
//!
//! Coefficients are row-major: `matrix[output][input]`. The implementation
//! is deliberately generic; device-specific coefficients are supplied by
//! the tuning compiler and do not live here.

use crate::MAX_CHANNELS;

/// Fixed-size channel-matrix parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    pub channels: f64,
    pub matrix: [[f64; MAX_CHANNELS]; MAX_CHANNELS],
}

impl Params {
    pub const WORDS: usize = 1 + MAX_CHANNELS * MAX_CHANNELS;

    pub fn to_words(&self, out: &mut [f64]) {
        if out.len() < Self::WORDS {
            return;
        }
        out[0] = self.channels;
        for (encoded, value) in out[1..].iter_mut().zip(self.matrix.iter().flatten()) {
            *encoded = *value;
        }
    }

    pub fn from_words(words: &[f64]) -> Option<Self> {
        if words.len() != Self::WORDS || !words[0].is_finite() {
            return None;
        }
        let channels = words[0] as usize;
        if channels == 0 || channels > MAX_CHANNELS || words[0] != channels as f64 {
            return None;
        }
        let mut matrix = [[0.0; MAX_CHANNELS]; MAX_CHANNELS];
        for (decoded, value) in matrix.iter_mut().flatten().zip(&words[1..]) {
            if !value.is_finite() {
                return None;
            }
            *decoded = *value;
        }
        Some(Self { channels: channels as f64, matrix })
    }
}

/// Allocation-free channel-matrix processor.
pub struct Run {
    channels: usize,
    matrix: [[f64; MAX_CHANNELS]; MAX_CHANNELS],
    valid: bool,
}

impl Run {
    pub fn new(params: &Params, _sample_rate: f64, channels: usize) -> Self {
        let valid = channels != 0
            && channels <= MAX_CHANNELS
            && params.channels == channels as f64
            && params.matrix[..channels].iter().all(|row| row[..channels].iter().all(|value| value.is_finite()));
        Self { channels, matrix: params.matrix, valid }
    }

    pub fn reset(&mut self) {}

    pub fn latency(&self) -> usize {
        0
    }

    pub fn process(&mut self, buffers: &mut [&mut [f32]], frames: usize) {
        let frames = buffers.iter().fold(frames, |count, buffer| count.min(buffer.len()));
        if !self.valid || buffers.len() != self.channels {
            for buffer in buffers {
                buffer[..frames].fill(0.0);
            }
            return;
        }
        let channels = self.channels;

        let mut input = [0.0; MAX_CHANNELS];
        let mut output = [0.0; MAX_CHANNELS];
        for frame in 0..frames {
            for (channel, slot) in input[..channels].iter_mut().enumerate() {
                let sample = buffers[channel][frame];
                *slot = if sample.is_finite() { sample as f64 } else { 0.0 };
            }
            for (row, out) in output[..channels].iter_mut().enumerate() {
                *out = self.matrix[row][..channels]
                    .iter()
                    .zip(&input[..channels])
                    .map(|(coefficient, sample)| coefficient * sample)
                    .sum();
            }
            for (buffer, value) in buffers[..channels].iter_mut().zip(&output[..channels]) {
                let sample = *value as f32;
                buffer[frame] = if sample.is_finite() { sample } else { 0.0 };
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn matrix(channels: usize, rows: &[&[f64]]) -> Params {
        let mut matrix = [[0.0; MAX_CHANNELS]; MAX_CHANNELS];
        for (row, values) in rows.iter().enumerate() {
            matrix[row][..values.len()].copy_from_slice(values);
        }
        Params { channels: channels as f64, matrix }
    }

    #[test]
    fn words_round_trip_and_reject_bad_shapes() {
        let params = matrix(2, &[&[0.25, -0.5], &[0.75, 1.0]]);
        let mut words = vec![0.0; Params::WORDS];
        params.to_words(&mut words);
        assert_eq!(Params::from_words(&words), Some(params));
        assert!(Params::from_words(&words[..words.len() - 1]).is_none());
        words[0] = 1.5;
        assert!(Params::from_words(&words).is_none());
        words[0] = 2.0;
        words[9] = f64::NAN;
        assert!(Params::from_words(&words).is_none());
    }

    #[test]
    fn applies_rows_to_outputs_without_scaling() {
        let params = matrix(2, &[&[2.0, 3.0], &[5.0, 7.0]]);
        let mut run = Run::new(&params, 48_000.0, 2);
        let mut left = [1.0, 2.0];
        let mut right = [10.0, 20.0];
        run.process(&mut [&mut left, &mut right], 2);
        assert_eq!(left, [32.0, 64.0]);
        assert_eq!(right, [75.0, 150.0]);
        assert_eq!(run.latency(), 0);
    }

    #[test]
    fn supports_more_than_two_channels() {
        let params = matrix(3, &[&[1.0, 0.0, 1.0], &[0.0, 1.0, 0.0], &[1.0, -1.0, 0.0]]);
        let mut run = Run::new(&params, 96_000.0, 3);
        let mut a = [1.0];
        let mut b = [2.0];
        let mut c = [4.0];
        run.process(&mut [&mut a, &mut b, &mut c], 1);
        assert_eq!((a[0], b[0], c[0]), (5.0, 2.0, -1.0));
    }

    #[test]
    fn non_finite_input_is_silence_and_short_buffers_are_safe() {
        let params = matrix(2, &[&[1.0, 1.0], &[1.0, -1.0]]);
        let mut run = Run::new(&params, 48_000.0, 2);
        let mut a = [f32::NAN, 3.0];
        let mut b = [2.0];
        run.process(&mut [&mut a, &mut b], 99);
        assert_eq!(a, [2.0, 3.0]);
        assert_eq!(b, [-2.0]);
    }

    #[test]
    fn mismatched_configuration_fails_closed() {
        let params = matrix(2, &[&[0.0, 1.0], &[1.0, 0.0]]);
        let mut run = Run::new(&params, 48_000.0, 1);
        let mut mono = [1.0, f32::INFINITY];
        run.process(&mut [&mut mono], 2);
        assert_eq!(mono, [0.0, 0.0]);
    }
}
