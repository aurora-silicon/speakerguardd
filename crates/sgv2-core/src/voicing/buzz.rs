// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! BuzzKill (`aufx/buzk/appl`) in its graph-bypassed form.
//!
//! Every supported speaker tuning that contains this box marks the Audio
//! Unit bypassed.  The exact active detector and time law are not carried;
//! `aurora-speaker-tuning` therefore refuses an active state.  This module
//! records the only implemented behaviour: zero-latency identity.  A chain
//! normally omits a bypassed box, but keeping the runner here makes that
//! contract independently testable.

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
}

impl Run {
    pub fn new(_params: &Params, _fs: f64, channels: usize) -> Run {
        Run { channels: channels.min(crate::MAX_CHANNELS) }
    }

    pub fn reset(&mut self) {}

    pub fn latency(&self) -> usize {
        0
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        for buf in bufs.iter_mut().take(self.channels) {
            for sample in buf.iter_mut().take(frames) {
                if !sample.is_finite() {
                    *sample = 0.0;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bypass_is_identity_and_sanitises_non_finite_input() {
        let mut run = Run::new(&Params, 48_000.0, 2);
        let mut left = [0.25, f32::NAN, -0.5];
        let mut right = [1.0, f32::INFINITY, -1.0];
        run.process(&mut [&mut left, &mut right], 3);
        assert_eq!(left, [0.25, 0.0, -0.5]);
        assert_eq!(right, [1.0, 0.0, -1.0]);
        assert_eq!(run.latency(), 0);
        assert_eq!(Params::from_words(&[]), Some(Params));
        assert_eq!(Params::from_words(&[0.0]), None);
    }
}
