// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! AULowFlow (`aufx/flow/appl`) with both mitigation paths disabled.
//!
//! This exact subset is a fixed 256-frame alignment delay on every lane.
//! Active mitigation depends on the property-6620 speaker model and its
//! pressure-response masking law; the tuning compiler refuses that profile
//! until it meets the differential gate.

use alloc::boxed::Box;
use alloc::vec;

use crate::MAX_CHANNELS;

pub const LATENCY_FRAMES: usize = 256;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Params;

impl Params {
    pub const WORDS: usize = 0;

    pub fn to_words(&self, _out: &mut [f64]) {}

    pub fn from_words(words: &[f64]) -> Option<Params> {
        words.is_empty().then_some(Params)
    }
}

pub struct Run {
    channels: usize,
    at: usize,
    line: Box<[f32]>,
}

impl Run {
    pub fn new(_params: &Params, _fs: f64, channels: usize) -> Run {
        let channels = channels.min(MAX_CHANNELS);
        Run { channels, at: 0, line: vec![0.0; channels * LATENCY_FRAMES].into_boxed_slice() }
    }

    pub fn reset(&mut self) {
        self.line.fill(0.0);
        self.at = 0;
    }

    pub fn latency(&self) -> usize {
        LATENCY_FRAMES
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let channels = self.channels.min(bufs.len());
        let frames = bufs[..channels].iter().fold(frames, |n, buf| n.min(buf.len()));
        for frame in 0..frames {
            for (channel, buf) in bufs.iter_mut().take(channels).enumerate() {
                let slot = channel * LATENCY_FRAMES + self.at;
                let delayed = self.line[slot];
                self.line[slot] = if buf[frame].is_finite() { buf[frame] } else { 0.0 };
                buf[frame] = delayed;
            }
            self.at = (self.at + 1) % LATENCY_FRAMES;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disabled_mitigation_is_exact_delay() {
        let mut run = Run::new(&Params, 48_000.0, 2);
        let mut left = [0.0; 258];
        let mut right = [0.0; 258];
        left[0] = 0.25;
        right[1] = -0.5;
        run.process(&mut [&mut left, &mut right], 258);
        assert_eq!(left[256], 0.25);
        assert_eq!(right[257], -0.5);
        assert!(left[..256].iter().all(|v| *v == 0.0));
        assert!(right[..257].iter().all(|v| *v == 0.0));
        assert_eq!(run.latency(), 256);
    }

    #[test]
    fn reset_clears_delay_and_words_are_strict() {
        let mut run = Run::new(&Params, 48_000.0, 1);
        let mut one = [1.0];
        run.process(&mut [&mut one], 1);
        run.reset();
        let mut zeros = [0.0; 256];
        run.process(&mut [&mut zeros], 256);
        assert!(zeros.iter().all(|v| *v == 0.0));
        assert_eq!(Params::from_words(&[]), Some(Params));
        assert_eq!(Params::from_words(&[0.0]), None);
    }
}
