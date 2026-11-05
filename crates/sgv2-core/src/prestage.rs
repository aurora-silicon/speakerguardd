//! The pre-stage: the Mac's own per-speaker filtering ahead of the unit.
//!
//! On macOS the speaker graph shapes every speaker channel before the
//! protection unit sees it: a high-pass on the program, and per speaker the
//! crossover (the tweeters' high-pass, the woofers' low-pass), shelves,
//! gains and cuts. spp3 itself limits temperature only; how hard a cone is
//! driven at low frequency is set by that filtering. The pre-stage carries
//! it on Linux: per unit channel, a cascade of second-order sections and a
//! gain, compiled from the Mac's tuning (`aurora-speaker-tuning`'s
//! `prestage`) and run in the plugin right before the unit.
//!
//! The compiled response never exceeds unity at any frequency, so the
//! pre-stage only ever takes drive away. The plugin checks that again when
//! it loads the blob and refuses a section that is unstable or a response
//! that gains.
//!
//! Section layout, little-endian, after the unit payload of a version 2
//! blob ([`crate::blob`]):
//!
//! ```text
//! 0   magic "SGV2PRE\0"    16  body_len
//! 8   channels             20  body_crc (CRC-32 of the body)
//! 12  flags
//! 24  body: per channel, sections u32, 0 u32, gain f64,
//!     then per section b0 b1 b2 a1 a2 (f64)
//! ```
//!
//! Each section is `H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)`.

use crate::blob::{crc32, BlobError};
use crate::MAX_CHANNELS;

pub const MAGIC: [u8; 8] = *b"SGV2PRE\0";
pub const HEAD_LEN: usize = 24;
/// Most sections one channel may carry. The shipped tunings use under 30.
pub const MAX_SECTIONS: usize = 64;
/// `flags` bit: the sections were compiled from the Mac's own speaker graph
/// (its high-pass and crossover included). The supervisor tells the kernel
/// so, and only then does the kernel open the tweeters' amp high-pass.
pub const FLAG_FROM_TUNING: u32 = 1 << 0;
/// Largest response the plugin accepts on its check grid (rounding only).
pub const UNITY_TOLERANCE: f64 = 1.0 + 1e-9;
/// What the filters see at most: far above any program, it keeps the
/// state finite on garbage input.
const INPUT_BOUND: f32 = 16.0;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Section {
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub a1: f64,
    pub a2: f64,
}

impl Section {
    pub const IDENTITY: Section = Section { b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0 };

    fn finite(&self) -> bool {
        [self.b0, self.b1, self.b2, self.a1, self.a2].iter().all(|v| v.is_finite())
    }

    /// Both poles strictly inside the unit circle (the stability triangle).
    pub fn stable(&self) -> bool {
        self.finite() && self.a2.abs() < 1.0 && self.a1.abs() < 1.0 + self.a2
    }

    /// `|H|` at `f` Hz for sample rate `fs`.
    pub fn magnitude(&self, f: f64, fs: f64) -> f64 {
        let w = 2.0 * core::f64::consts::PI * f / fs;
        let (c1, s1) = (libm::cos(w), -libm::sin(w));
        let (c2, s2) = (libm::cos(2.0 * w), -libm::sin(2.0 * w));
        let nr = self.b0 + self.b1 * c1 + self.b2 * c2;
        let ni = self.b1 * s1 + self.b2 * s2;
        let dr = 1.0 + self.a1 * c1 + self.a2 * c2;
        let di = self.a1 * s1 + self.a2 * s2;
        libm::sqrt((nr * nr + ni * ni) / (dr * dr + di * di))
    }
}

/// One channel's filters, as the compiler hands them over.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Channel {
    pub sections: [Section; MAX_SECTIONS],
    pub count: usize,
    pub gain: f64,
}

impl Channel {
    pub const IDENTITY: Channel = Channel { sections: [Section::IDENTITY; MAX_SECTIONS], count: 0, gain: 1.0 };

    pub fn sections(&self) -> &[Section] {
        &self.sections[..self.count]
    }

    /// `|H|` of the whole channel at `f` Hz.
    pub fn magnitude(&self, f: f64, fs: f64) -> f64 {
        self.sections().iter().fold(self.gain.abs(), |m, s| m * s.magnitude(f, fs))
    }

    /// The largest `|H|` on [`check_grid`] for `fs`.
    pub fn peak(&self, fs: f64) -> f64 {
        let mut peak = 0.0f64;
        check_grid(fs, |f| peak = peak.max(self.magnitude(f, fs)));
        peak
    }
}

/// Calls `visit` with DC, 2048 log-spaced frequencies from 1 Hz to just
/// under Nyquist, and Nyquist.
pub fn check_grid(fs: f64, mut visit: impl FnMut(f64)) {
    const N: usize = 2048;
    let top = 0.4999 * fs;
    visit(0.0);
    for i in 0..N {
        visit(libm::exp(libm::log(top) * i as f64 / (N - 1) as f64));
    }
    visit(0.5 * fs);
}

/// A decoded pre-stage section of a blob.
#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    pub channels: usize,
    pub flags: u32,
    pub channel: [Channel; MAX_CHANNELS],
}

impl Params {
    /// Every channel passes unchanged; nothing derived from a tuning.
    pub fn identity(channels: usize) -> Params {
        Params { channels: channels.min(MAX_CHANNELS), flags: 0, channel: [Channel::IDENTITY; MAX_CHANNELS] }
    }

    pub fn from_tuning(&self) -> bool {
        self.flags & FLAG_FROM_TUNING != 0
    }

    /// Encoded section, ready to follow a unit payload.
    #[cfg(any(test, feature = "std"))]
    pub fn encode(&self) -> std::vec::Vec<u8> {
        let mut body = std::vec::Vec::new();
        for ch in &self.channel[..self.channels] {
            body.extend_from_slice(&(ch.count as u32).to_le_bytes());
            body.extend_from_slice(&0u32.to_le_bytes());
            body.extend_from_slice(&ch.gain.to_le_bytes());
            for s in ch.sections() {
                for v in [s.b0, s.b1, s.b2, s.a1, s.a2] {
                    body.extend_from_slice(&v.to_le_bytes());
                }
            }
        }
        let mut out = MAGIC.to_vec();
        for v in [self.channels as u32, self.flags, body.len() as u32, crc32(&body)] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.extend_from_slice(&body);
        out
    }

    /// Decodes and checks a section (`bytes` exactly the section). Refuses
    /// a channel count other than `channels`, an unstable or non-finite
    /// section, and a channel whose response exceeds unity at `fs`.
    pub fn decode(bytes: &[u8], channels: usize, fs: f64) -> Result<Params, BlobError> {
        if bytes.len() < HEAD_LEN {
            return Err(BlobError::Short);
        }
        if bytes[..8] != MAGIC {
            return Err(BlobError::Magic);
        }
        let word = |b: &[u8], i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        let (n, flags, len, crc) =
            (word(bytes, 8) as usize, word(bytes, 12), word(bytes, 16) as usize, word(bytes, 20));
        if n != channels || n == 0 || n > MAX_CHANNELS {
            return Err(BlobError::Channels);
        }
        let body = &bytes[HEAD_LEN..];
        if body.len() != len {
            return Err(BlobError::Length);
        }
        if crc32(body) != crc {
            return Err(BlobError::Crc);
        }
        let mut p = Params { channels: n, flags, channel: [Channel::IDENTITY; MAX_CHANNELS] };
        let f64_at = |i: usize| -> Result<f64, BlobError> {
            let b: [u8; 8] = body.get(i..i + 8).ok_or(BlobError::Length)?.try_into().map_err(|_| BlobError::Length)?;
            Ok(f64::from_le_bytes(b))
        };
        let mut at = 0usize;
        for ch in p.channel.iter_mut().take(n) {
            if body.len() < at + 16 {
                return Err(BlobError::Length);
            }
            let count = word(body, at) as usize;
            if count > MAX_SECTIONS || word(body, at + 4) != 0 {
                return Err(BlobError::Length);
            }
            ch.count = count;
            ch.gain = f64_at(at + 8)?;
            at += 16;
            for s in ch.sections.iter_mut().take(count) {
                *s = Section {
                    b0: f64_at(at)?,
                    b1: f64_at(at + 8)?,
                    b2: f64_at(at + 16)?,
                    a1: f64_at(at + 24)?,
                    a2: f64_at(at + 32)?,
                };
                at += 40;
                if !s.stable() {
                    return Err(BlobError::Unstable);
                }
            }
            if !ch.gain.is_finite() || ch.peak(fs) > UNITY_TOLERANCE {
                return Err(BlobError::Unstable);
            }
        }
        if at != body.len() {
            return Err(BlobError::Length);
        }
        Ok(p)
    }
}

/// The running pre-stage: transposed direct form II, f64 state.
pub struct PreStage {
    params: Params,
    state: [[[f64; 2]; MAX_SECTIONS]; MAX_CHANNELS],
}

impl PreStage {
    pub fn new(params: Params) -> PreStage {
        PreStage { params, state: [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS] }
    }

    pub fn params(&self) -> &Params {
        &self.params
    }

    /// Clears every filter's history (stream start).
    pub fn reset(&mut self) {
        self.state = [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS];
    }

    /// Filters channel `c` from `input` into `output` (equal lengths; the
    /// shorter decides). A channel the pre-stage does not have is muted.
    /// Real time: no allocation, no locks, no panics.
    pub fn process(&mut self, c: usize, input: &[f32], output: &mut [f32]) {
        let (Some(ch), Some(z)) = (self.params.channel.get(c), self.state.get_mut(c)) else {
            output.fill(0.0);
            return;
        };
        if c >= self.params.channels {
            output.fill(0.0);
            return;
        }
        let secs = &ch.sections[..ch.count.min(MAX_SECTIONS)];
        for (x, y) in input.iter().zip(output.iter_mut()) {
            let mut v = if x.is_finite() { x.clamp(-INPUT_BOUND, INPUT_BOUND) as f64 } else { 0.0 };
            for (s, st) in secs.iter().zip(z.iter_mut()) {
                let out = s.b0 * v + st[0];
                st[0] = s.b1 * v - s.a1 * out + st[1];
                st[1] = s.b2 * v - s.a2 * out;
                v = out;
            }
            v *= ch.gain;
            if v.is_finite() {
                *y = v as f32;
            } else {
                // never reached with stable sections and bounded input;
                // a poisoned history is cleared rather than kept
                *z = [[0.0; 2]; MAX_SECTIONS];
                *y = 0.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hp2(f0: f64, fs: f64) -> Section {
        // Butterworth high-pass, bilinear with prewarping
        let k = libm::tan(core::f64::consts::PI * f0 / fs);
        let q = core::f64::consts::SQRT_2;
        let n = 1.0 + q * k + k * k;
        Section { b0: 1.0 / n, b1: -2.0 / n, b2: 1.0 / n, a1: 2.0 * (k * k - 1.0) / n, a2: (1.0 - q * k + k * k) / n }
    }

    fn params(ch0: &[Section], gain: f64) -> Params {
        let mut p = Params::identity(2);
        p.flags = FLAG_FROM_TUNING;
        p.channel[0].count = ch0.len();
        p.channel[0].sections[..ch0.len()].copy_from_slice(ch0);
        p.channel[0].gain = gain;
        p
    }

    #[test]
    fn roundtrip_and_checks() {
        let p = params(&[hp2(1000.0, 48_000.0), hp2(1000.0, 48_000.0)], 0.5);
        let b = p.encode();
        assert_eq!(Params::decode(&b, 2, 48_000.0).unwrap(), p);
        assert_eq!(Params::decode(&b, 3, 48_000.0), Err(BlobError::Channels));
        let mut bad = b.clone();
        *bad.last_mut().unwrap() ^= 1;
        assert_eq!(Params::decode(&bad, 2, 48_000.0), Err(BlobError::Crc));
        // a gain above unity is refused, as is an unstable section
        let loud = params(&[hp2(1000.0, 48_000.0)], 1.5).encode();
        assert_eq!(Params::decode(&loud, 2, 48_000.0), Err(BlobError::Unstable));
        let wild = params(&[Section { b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 1.0 }], 1.0).encode();
        assert_eq!(Params::decode(&wild, 2, 48_000.0), Err(BlobError::Unstable));
    }

    #[test]
    fn high_pass_takes_low_frequencies_away() {
        let fs = 48_000.0;
        let mut s = PreStage::new(params(&[hp2(1000.0, fs), hp2(1000.0, fs)], 1.0));
        let n = 48_000;
        let x: std::vec::Vec<f32> =
            (0..n).map(|i| libm::sin(2.0 * core::f64::consts::PI * 100.0 * i as f64 / fs) as f32).collect();
        let mut y = std::vec![0.0f32; n];
        s.process(0, &x, &mut y);
        let peak = y[n / 2..].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        // fourth-order Butterworth at a decade below its corner: -80 dB
        assert!(peak < 1.2e-4 && peak > 0.8e-4, "{peak}");
        // channel 1 has no sections: passes unchanged
        s.process(1, &x, &mut y);
        assert_eq!(x, y);
        // a channel the pre-stage does not have is muted
        s.process(5, &x, &mut y);
        assert!(y.iter().all(|v| *v == 0.0));
    }

    #[test]
    fn garbage_input_keeps_the_state_finite() {
        let fs = 48_000.0;
        let mut s = PreStage::new(params(&[hp2(60.0, fs)], 1.0));
        let x = [f32::NAN, f32::INFINITY, 1e30, -1e30, 0.5];
        let mut y = [0.0f32; 5];
        s.process(0, &x, &mut y);
        assert!(y.iter().all(|v| v.is_finite()));
    }
}
