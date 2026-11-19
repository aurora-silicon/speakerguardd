// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! ChannelSpecificDelay (`aufx/cdly/appl`).
//!
//! Each channel has an independent delay in whole frames.  The tuning
//! compiler converts the Audio Unit's millisecond values to frames using
//! nearest-integer rounding, matching its impulse response.  There is no
//! interpolation, feedback, gain, or tail.  Reported latency is the longest
//! channel delay.

use alloc::boxed::Box;
use alloc::vec;

use crate::MAX_CHANNELS;

/// The public Audio Unit range is 0--100 ms.  This bound covers that range
/// at the highest rate accepted by the voicing compiler.
pub const MAX_DELAY_FRAMES: usize = 19_200;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Params {
    pub channels: usize,
    pub delay_frames: [usize; MAX_CHANNELS],
}

impl Params {
    pub const WORDS: usize = 1 + MAX_CHANNELS;

    pub fn to_words(&self, out: &mut [f64]) {
        if out.len() < Self::WORDS {
            return;
        }
        out[0] = self.channels as f64;
        for (word, delay) in out[1..Self::WORDS].iter_mut().zip(self.delay_frames) {
            *word = delay as f64;
        }
    }

    pub fn from_words(words: &[f64]) -> Option<Params> {
        if words.len() != Self::WORDS || words.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let channels = usize::try_from(words[0] as i64).ok()?;
        if words[0] != channels as f64 || !(1..=MAX_CHANNELS).contains(&channels) {
            return None;
        }
        let mut delay_frames = [0; MAX_CHANNELS];
        for (i, word) in words[1..].iter().copied().enumerate() {
            let delay = usize::try_from(word as i64).ok()?;
            if word != delay as f64 || delay > MAX_DELAY_FRAMES || (i >= channels && delay != 0) {
                return None;
            }
            delay_frames[i] = delay;
        }
        Some(Params { channels, delay_frames })
    }
}

pub struct Run {
    channels: usize,
    delay_frames: [usize; MAX_CHANNELS],
    line_len: usize,
    line_at: usize,
    line: Box<[f32]>,
}

impl Run {
    pub fn new(params: &Params, _fs: f64, channels: usize) -> Run {
        let channels = channels.min(params.channels).min(MAX_CHANNELS);
        let mut delay_frames = params.delay_frames;
        for delay in &mut delay_frames {
            *delay = (*delay).min(MAX_DELAY_FRAMES);
        }
        let line_len = delay_frames[..channels].iter().copied().max().unwrap_or(0) + 1;
        Run {
            channels,
            delay_frames,
            line_len,
            line_at: 0,
            line: vec![0.0; channels.saturating_mul(line_len)].into_boxed_slice(),
        }
    }

    pub fn reset(&mut self) {
        self.line.fill(0.0);
        self.line_at = 0;
    }

    pub fn latency(&self) -> usize {
        self.delay_frames[..self.channels].iter().copied().max().unwrap_or(0)
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let channels = self.channels.min(bufs.len());
        for frame in 0..frames {
            for (channel, buf) in bufs.iter_mut().take(channels).enumerate() {
                if frame >= buf.len() {
                    continue;
                }
                let input = if buf[frame].is_finite() { buf[frame] } else { 0.0 };
                let base = channel * self.line_len;
                self.line[base + self.line_at] = input;
                let read = (self.line_at + self.line_len - self.delay_frames[channel]) % self.line_len;
                buf[frame] = self.line[base + read];
            }
            self.line_at += 1;
            if self.line_at == self.line_len {
                self.line_at = 0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(delays: &[usize]) -> Params {
        let mut delay_frames = [0; MAX_CHANNELS];
        delay_frames[..delays.len()].copy_from_slice(delays);
        Params { channels: delays.len(), delay_frames }
    }

    #[test]
    fn delays_channels_independently_and_resets() {
        let p = params(&[0, 2]);
        let mut run = Run::new(&p, 48_000.0, 2);
        let mut left = [1.0, 2.0, 3.0, 4.0];
        let mut right = [5.0, 6.0, 7.0, 8.0];
        run.process(&mut [&mut left, &mut right], 4);
        assert_eq!(left, [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(right, [0.0, 0.0, 5.0, 6.0]);
        assert_eq!(run.latency(), 2);
        run.reset();
        let mut left = [9.0];
        let mut right = [10.0];
        run.process(&mut [&mut left, &mut right], 1);
        assert_eq!(right, [0.0]);
    }

    #[test]
    fn words_are_strict_and_round_trip() {
        let p = params(&[48, 206]);
        let mut words = [0.0; Params::WORDS];
        p.to_words(&mut words);
        assert_eq!(Params::from_words(&words), Some(p));
        words[3] = 1.0;
        assert_eq!(Params::from_words(&words), None);
        words[3] = 0.0;
        words[1] = 0.5;
        assert_eq!(Params::from_words(&words), None);
    }

    #[test]
    fn non_finite_input_becomes_silence() {
        let mut run = Run::new(&params(&[1]), 48_000.0, 1);
        let mut channel = [f32::NAN, 1.0];
        run.process(&mut [&mut channel], 2);
        assert_eq!(channel, [0.0, 0.0]);
    }
}
