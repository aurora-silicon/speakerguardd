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
//!
//! With [`FLAG_DYNAMICS`], a dynamics block follows the channels: the
//! graph's multiband compressor ahead of the unit, as the tuning sets it
//! ([`Dynamics`]), run on every channel after its filters. Layout, f64:
//! crossovers (3), input gain dB, output gain dB, release s, then per band
//! (4) threshold dB, ratio, range dB, makeup dB, mode (0 compress, 1 pass,
//! 2 mute).
//!
//! With [`FLAG_BOXES`], the graph's compressor boxes follow instead (they
//! replace the dynamics block), each run at its own place in the lane:
//! `boxes` u32, 0 u32, then per box, per channel: `split` u32 (how many of
//! the channel's sections run before this box), 0 u32, `gain` f64 (the
//! channel's gain applied right before this box), then the box's
//! [`crate::voicing::mstk::Params`] words (f64). Splits never decrease from
//! box to box. The channel's own `gain` is applied after its last section.
//! [`PreStage::process_block`] runs sections, gains and boxes in that order;
//! a box links its detection across every channel, which is the graph's
//! own linking because channels that share a box channel carry identical
//! filtering up to it (the compiler checks that).
//!
//! With [`FLAG_DYNAMIC_FILTERS`], ordered ToneMeister dynamic slots follow:
//! count and padding, then per slot its compressor-box position, output and
//! sidechain masks, type, per-channel section splits, smoothing switch, and
//! seven f64 saved-law values (range, frequency, quality, ratio, threshold,
//! attack and release). [`PreStage::process_block`] runs each slot at that
//! recorded position. Its gain computer is the unit's 44-point linear-gain
//! B-spline. With time smoothing disabled, a quadrature-envelope controller
//! samples two all-pass branches at a fixed cadence, holds the control value,
//! and low-pass reconstructs it. The audio carrier remains causal and has no
//! added latency.
//!
//! The compressor: a four-band Linkwitz-Riley tree (fourth order, with the
//! second-order allpass compensation that keeps every band in phase, so the
//! bands sum to an allpass); per band, the louder channel's power, held
//! with an instant attack and the tuning's release; the tuning's
//! downward curve (threshold, ratio, range, makeup) on that level; output
//! `out_gain` times the sum of the band gains times the bands of `in_gain`
//! times the input: the unit's own gain staging, makeup and output gain
//! included, as macOS applies it. Its detector is stricter than macOS's
//! (instant attack, the band's peak power, a hard knee at or below macOS's
//! soft one at high level): it never reduces less at full scale.

use crate::blob::{crc32, BlobError};
use crate::MAX_CHANNELS;

pub const MAGIC: [u8; 8] = *b"SGV2PRE\0";
pub const HEAD_LEN: usize = 24;
/// Most sections one channel may carry. The shipped tunings use under 30.
pub const MAX_SECTIONS: usize = 64;
/// The complete voicing runs in a separate upstream plugin. The unit
/// pre-stage is identity and the blob must also carry its bound artifact.
pub const FLAG_EXTERNAL_VOICING: u32 = 1 << 4;
/// `flags` bit: the sections were compiled from the Mac's own speaker graph
/// (its high-pass and crossover included). The supervisor tells the kernel
/// so, and only then does the kernel open the tweeters' amp high-pass.
pub const FLAG_FROM_TUNING: u32 = 1 << 0;
/// `flags` bit: a [`Dynamics`] block follows the channels.
pub const FLAG_DYNAMICS: u32 = 1 << 1;
/// `flags` bit: compressor boxes at their graph places follow the channels
/// (see the module documentation); they replace [`FLAG_DYNAMICS`].
pub const FLAG_BOXES: u32 = 1 << 2;
/// `flags` bit: level-dependent ToneMeister filters follow the compressor
/// boxes in the blob and run at their recorded places in the lanes.
pub const FLAG_DYNAMIC_FILTERS: u32 = 1 << 3;
/// Most compressor boxes a shipped lane passes ahead of the unit.
pub const MAX_BOXES: usize = 2;
/// Most level-dependent ToneMeister slots carried by a lane.
pub const MAX_DYNAMIC_FILTERS: usize = 32;
/// Bands of the multiband dynamics.
pub const BANDS: usize = 4;

use crate::voicing::mstk;

/// The two level-dependent ToneMeister shapes carried by the pre-stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum DynamicKind {
    Notch = 15,
    HighShelf = 18,
}

/// One level-dependent filter's saved law.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicFilter {
    pub kind: DynamicKind,
    pub range_db: f64,
    pub frequency_hz: f64,
    pub quality: f64,
    pub ratio: f64,
    pub threshold_db: f64,
    pub attack_ms: f64,
    pub release_ms: f64,
    pub smooth: bool,
}

impl Default for DynamicFilter {
    fn default() -> Self {
        DynamicFilter {
            kind: DynamicKind::Notch,
            range_db: 0.0,
            frequency_hz: 1_000.0,
            quality: 1.0,
            ratio: 1.0,
            threshold_db: 0.0,
            attack_ms: 0.0,
            release_ms: 0.0,
            smooth: false,
        }
    }
}

impl DynamicFilter {
    pub(crate) fn valid(&self, fs: f64) -> bool {
        let finite = |v: f64, lo: f64, hi: f64| v.is_finite() && v >= lo && v <= hi;
        finite(self.range_db, -30.0, 0.0)
            && finite(self.frequency_hz, 1.0, 0.49 * fs)
            && finite(self.quality, 0.01, 100.0)
            && finite(self.ratio, 0.1, 10.0)
            && finite(self.threshold_db, -180.0, 0.0)
            && finite(self.attack_ms, 0.0, 1_000.0)
            && finite(self.release_ms, 0.0, 5_000.0)
    }
}

/// One dynamic slot at its graph place. `before_box` is how many compressor
/// boxes precede it; `split[c]` is how many static sections of channel `c`
/// precede it. The masks address pre-stage channels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicAt {
    pub before_box: usize,
    pub split: [usize; MAX_CHANNELS],
    pub active_mask: u32,
    pub sidechain_mask: u32,
    pub params: DynamicFilter,
}

impl Default for DynamicAt {
    fn default() -> Self {
        DynamicAt {
            before_box: 0,
            split: [0; MAX_CHANNELS],
            active_mask: 0,
            sidechain_mask: 0,
            params: DynamicFilter::default(),
        }
    }
}

/// One compressor box of the graph, at its place in every lane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxAt {
    /// Per channel: how many of its sections run before the box.
    pub split: [usize; MAX_CHANNELS],
    /// Per channel: the gain applied right before the box.
    pub gain: [f64; MAX_CHANNELS],
    pub params: mstk::Params,
}

impl Default for BoxAt {
    fn default() -> Self {
        BoxAt { split: [0; MAX_CHANNELS], gain: [1.0; MAX_CHANNELS], params: mstk::Params::default() }
    }
}
const DYN_WORDS: usize = 6 + 5 * BANDS;

/// How one band of the dynamics treats its signal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u32)]
pub enum BandMode {
    #[default]
    Compress = 0,
    /// Unity gain, no compression.
    Pass = 1,
    Mute = 2,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Band {
    pub threshold_db: f64,
    pub ratio: f64,
    /// The deepest reduction, dB (0 or below).
    pub range_db: f64,
    pub makeup_db: f64,
    pub mode: BandMode,
}

/// The graph's multiband compressor ahead of the unit, as the tuning sets it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Dynamics {
    pub crossover_hz: [f64; BANDS - 1],
    pub input_gain_db: f64,
    pub output_gain_db: f64,
    pub release_s: f64,
    pub band: [Band; BANDS],
}

impl Dynamics {
    pub(crate) fn valid(&self, fs: f64) -> bool {
        let x = self.crossover_hz;
        let finite = |v: f64, lo: f64, hi: f64| v.is_finite() && v >= lo && v <= hi;
        x[0] > 0.0
            && x[0] < x[1]
            && x[1] < x[2]
            && x[2] < 0.49 * fs
            && finite(self.input_gain_db, -40.0, 40.0)
            && finite(self.output_gain_db, -40.0, 40.0)
            && finite(self.release_s, 1e-4, 10.0)
            && self.band.iter().all(|b| {
                finite(b.threshold_db, -120.0, 0.0)
                    && finite(b.ratio, 1.0, 100.0)
                    && finite(b.range_db, -60.0, 0.0)
                    && finite(b.makeup_db, -40.0, 40.0)
            })
    }

    #[cfg(any(test, feature = "std"))]
    fn words(&self) -> [f64; DYN_WORDS] {
        let mut w = [0.0; DYN_WORDS];
        w[..3].copy_from_slice(&self.crossover_hz);
        w[3] = self.input_gain_db;
        w[4] = self.output_gain_db;
        w[5] = self.release_s;
        for (i, b) in self.band.iter().enumerate() {
            let o = 6 + 5 * i;
            w[o..o + 5].copy_from_slice(&[b.threshold_db, b.ratio, b.range_db, b.makeup_db, b.mode as u32 as f64]);
        }
        w
    }

    fn from_words(w: &[f64; DYN_WORDS]) -> Option<Dynamics> {
        let mut d = Dynamics {
            crossover_hz: [w[0], w[1], w[2]],
            input_gain_db: w[3],
            output_gain_db: w[4],
            release_s: w[5],
            ..Default::default()
        };
        for (i, b) in d.band.iter_mut().enumerate() {
            let o = 6 + 5 * i;
            let m = w[o + 4];
            let mode = if m == 0.0 {
                BandMode::Compress
            } else if m == 1.0 {
                BandMode::Pass
            } else if m == 2.0 {
                BandMode::Mute
            } else {
                return None;
            };
            *b = Band { threshold_db: w[o], ratio: w[o + 1], range_db: w[o + 2], makeup_db: w[o + 3], mode };
        }
        Some(d)
    }
}
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
    /// Present exactly when `flags` has [`FLAG_DYNAMICS`].
    pub dynamics: Option<Dynamics>,
    /// The first `box_count` are present exactly when `flags` has
    /// [`FLAG_BOXES`], in graph order.
    pub boxes: [BoxAt; MAX_BOXES],
    pub box_count: usize,
    /// The first `dynamic_count` entries are present when
    /// [`FLAG_DYNAMIC_FILTERS`] is set, in graph order.
    pub dynamic: [DynamicAt; MAX_DYNAMIC_FILTERS],
    pub dynamic_count: usize,
}

impl Params {
    /// Every channel passes unchanged; nothing derived from a tuning.
    pub fn identity(channels: usize) -> Params {
        Params {
            channels: channels.min(MAX_CHANNELS),
            flags: 0,
            channel: [Channel::IDENTITY; MAX_CHANNELS],
            dynamics: None,
            boxes: [BoxAt::default(); MAX_BOXES],
            box_count: 0,
            dynamic: [DynamicAt::default(); MAX_DYNAMIC_FILTERS],
            dynamic_count: 0,
        }
    }

    /// The compressor boxes, in graph order.
    pub fn boxes(&self) -> &[BoxAt] {
        &self.boxes[..self.box_count.min(MAX_BOXES)]
    }

    pub fn dynamic_filters(&self) -> &[DynamicAt] {
        &self.dynamic[..self.dynamic_count.min(MAX_DYNAMIC_FILTERS)]
    }

    /// `|H|` of channel `c`'s filters and gains at `f` Hz, the boxes left out.
    pub fn linear_magnitude(&self, c: usize, f: f64, fs: f64) -> f64 {
        self.boxes().iter().fold(self.channel[c].magnitude(f, fs), |m, b| m * b.gain[c].abs())
    }

    /// The largest [`Params::linear_magnitude`] of channel `c` on [`check_grid`].
    pub fn linear_peak(&self, c: usize, fs: f64) -> f64 {
        let mut peak = 0.0f64;
        check_grid(fs, |f| peak = peak.max(self.linear_magnitude(c, f, fs)));
        peak
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
        let mut flags = self.flags & !(FLAG_DYNAMICS | FLAG_BOXES | FLAG_DYNAMIC_FILTERS);
        if let Some(d) = &self.dynamics {
            flags |= FLAG_DYNAMICS;
            for v in d.words() {
                body.extend_from_slice(&v.to_le_bytes());
            }
        }
        if self.box_count > 0 {
            flags |= FLAG_BOXES;
            body.extend_from_slice(&(self.box_count as u32).to_le_bytes());
            body.extend_from_slice(&0u32.to_le_bytes());
            for b in self.boxes() {
                for c in 0..self.channels {
                    body.extend_from_slice(&(b.split[c] as u32).to_le_bytes());
                    body.extend_from_slice(&0u32.to_le_bytes());
                    body.extend_from_slice(&b.gain[c].to_le_bytes());
                }
                let mut w = [0.0f64; mstk::Params::WORDS];
                b.params.to_words(&mut w);
                for v in w {
                    body.extend_from_slice(&v.to_le_bytes());
                }
            }
        }
        if self.dynamic_count > 0 {
            flags |= FLAG_DYNAMIC_FILTERS;
            body.extend_from_slice(&(self.dynamic_count as u32).to_le_bytes());
            body.extend_from_slice(&0u32.to_le_bytes());
            for d in self.dynamic_filters() {
                for v in [d.before_box as u32, d.active_mask, d.sidechain_mask, d.params.kind as u32] {
                    body.extend_from_slice(&v.to_le_bytes());
                }
                for c in 0..self.channels {
                    body.extend_from_slice(&(d.split[c] as u32).to_le_bytes());
                }
                body.extend_from_slice(&(d.params.smooth as u32).to_le_bytes());
                for v in [
                    d.params.range_db,
                    d.params.frequency_hz,
                    d.params.quality,
                    d.params.ratio,
                    d.params.threshold_db,
                    d.params.attack_ms,
                    d.params.release_ms,
                ] {
                    body.extend_from_slice(&v.to_le_bytes());
                }
            }
        }
        let mut out = MAGIC.to_vec();
        for v in [self.channels as u32, flags, body.len() as u32, crc32(&body)] {
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
        let mut p = Params::identity(n);
        p.flags = flags;
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
            if !ch.gain.is_finite() {
                return Err(BlobError::Unstable);
            }
        }
        if flags & FLAG_DYNAMICS != 0 {
            let mut w = [0.0; DYN_WORDS];
            for (k, v) in w.iter_mut().enumerate() {
                *v = f64_at(at + 8 * k)?;
            }
            at += 8 * DYN_WORDS;
            let d = Dynamics::from_words(&w).ok_or(BlobError::Length)?;
            if !d.valid(fs) {
                return Err(BlobError::Unstable);
            }
            p.dynamics = Some(d);
        }
        if flags & FLAG_BOXES != 0 {
            if flags & FLAG_DYNAMICS != 0 || body.len() < at + 8 {
                return Err(BlobError::Length);
            }
            let count = word(body, at) as usize;
            if count == 0 || count > MAX_BOXES || word(body, at + 4) != 0 {
                return Err(BlobError::Length);
            }
            at += 8;
            p.box_count = count;
            for k in 0..count {
                let mut b = BoxAt::default();
                for c in 0..n {
                    if body.len() < at + 16 || word(body, at + 4) != 0 {
                        return Err(BlobError::Length);
                    }
                    b.split[c] = word(body, at) as usize;
                    b.gain[c] = f64_at(at + 8)?;
                    at += 16;
                    let floor = if k == 0 { 0 } else { p.boxes[k - 1].split[c] };
                    if b.split[c] < floor || b.split[c] > p.channel[c].count || !b.gain[c].is_finite() {
                        return Err(BlobError::Length);
                    }
                }
                let mut w = [0.0f64; mstk::Params::WORDS];
                for (i, v) in w.iter_mut().enumerate() {
                    *v = f64_at(at + 8 * i)?;
                }
                at += 8 * mstk::Params::WORDS;
                b.params = mstk::Params::from_words(&w).ok_or(BlobError::Length)?;
                if !b.params.valid(fs) {
                    return Err(BlobError::Unstable);
                }
                p.boxes[k] = b;
            }
        }
        if flags & FLAG_DYNAMIC_FILTERS != 0 {
            if body.len() < at + 8 {
                return Err(BlobError::Length);
            }
            let count = word(body, at) as usize;
            if count == 0 || count > MAX_DYNAMIC_FILTERS || word(body, at + 4) != 0 {
                return Err(BlobError::Length);
            }
            at += 8;
            p.dynamic_count = count;
            let all_channels = if n == 32 { u32::MAX } else { (1u32 << n) - 1 };
            let mut previous_box = 0;
            let mut previous_split = [0usize; MAX_CHANNELS];
            for k in 0..count {
                if body.len() < at + 16 + 4 * n + 4 + 56 {
                    return Err(BlobError::Length);
                }
                let before_box = word(body, at) as usize;
                let active_mask = word(body, at + 4);
                let sidechain_mask = word(body, at + 8);
                let kind = match word(body, at + 12) {
                    15 => DynamicKind::Notch,
                    18 => DynamicKind::HighShelf,
                    _ => return Err(BlobError::Length),
                };
                at += 16;
                let mut split = [0usize; MAX_CHANNELS];
                for value in split.iter_mut().take(n) {
                    *value = word(body, at) as usize;
                    at += 4;
                }
                let smooth = match word(body, at) {
                    0 => false,
                    1 => true,
                    _ => return Err(BlobError::Length),
                };
                at += 4;
                let params = DynamicFilter {
                    kind,
                    range_db: f64_at(at)?,
                    frequency_hz: f64_at(at + 8)?,
                    quality: f64_at(at + 16)?,
                    ratio: f64_at(at + 24)?,
                    threshold_db: f64_at(at + 32)?,
                    attack_ms: f64_at(at + 40)?,
                    release_ms: f64_at(at + 48)?,
                    smooth,
                };
                at += 56;
                if before_box < previous_box
                    || before_box > p.box_count
                    || active_mask == 0
                    || active_mask & !all_channels != 0
                    || sidechain_mask == 0
                    || sidechain_mask & !all_channels != 0
                    || !params.valid(fs)
                {
                    return Err(BlobError::Length);
                }
                for c in 0..n {
                    let floor = if before_box == previous_box { previous_split[c] } else { 0 };
                    let ceiling =
                        if before_box < p.box_count { p.boxes[before_box].split[c] } else { p.channel[c].count };
                    if split[c] < floor || split[c] > ceiling {
                        return Err(BlobError::Length);
                    }
                }
                p.dynamic[k] = DynamicAt { before_box, split, active_mask, sidechain_mask, params };
                previous_box = before_box;
                previous_split = split;
            }
        }
        // filters and gains together never exceed unity (the boxes are the
        // graph's own dynamics, carried as macOS stages them)
        for c in 0..n {
            if p.linear_peak(c, fs) > UNITY_TOLERANCE {
                return Err(BlobError::Unstable);
            }
        }
        if flags & !(FLAG_FROM_TUNING | FLAG_DYNAMICS | FLAG_BOXES | FLAG_DYNAMIC_FILTERS | FLAG_EXTERNAL_VOICING) != 0
        {
            return Err(BlobError::Length);
        }
        if at != body.len() {
            return Err(BlobError::Length);
        }
        Ok(p)
    }
}

/// Sections of the dynamics' band tree, per channel.
const TREE: usize = 15;

fn butter2(high: bool, fc: f64, fs: f64) -> Section {
    let k = libm::tan(core::f64::consts::PI * fc / fs);
    let r2 = core::f64::consts::SQRT_2;
    let n = 1.0 / (1.0 + r2 * k + k * k);
    let (a1, a2) = (2.0 * (k * k - 1.0) * n, (1.0 - r2 * k + k * k) * n);
    if high {
        Section { b0: n, b1: -2.0 * n, b2: n, a1, a2 }
    } else {
        Section { b0: k * k * n, b1: 2.0 * k * k * n, b2: k * k * n, a1, a2 }
    }
}

/// The allpass a fourth-order Linkwitz-Riley pair sums to.
fn allpass2(fc: f64, fs: f64) -> Section {
    let s = butter2(false, fc, fs);
    Section { b0: s.a2, b1: s.a1, b2: 1.0, a1: s.a1, a2: s.a2 }
}

struct DynRun {
    tree: [Section; TREE],
    in_gain: f64,
    out_scale: f64,
    release: f64,
    band: [Band; BANDS],
    env: [f64; BANDS],
    state: [[[f64; 2]; TREE]; MAX_CHANNELS],
}

impl DynRun {
    fn new(d: &Dynamics, fs: f64) -> DynRun {
        let [f0, f1, f2] = d.crossover_hz;
        let (lp0, hp0, lp1, hp1, lp2, hp2) = (
            butter2(false, f0, fs),
            butter2(true, f0, fs),
            butter2(false, f1, fs),
            butter2(true, f1, fs),
            butter2(false, f2, fs),
            butter2(true, f2, fs),
        );
        let (ap1, ap2) = (allpass2(f1, fs), allpass2(f2, fs));
        DynRun {
            tree: [lp0, lp0, hp0, hp0, ap1, ap2, lp1, lp1, hp1, hp1, ap2, lp2, lp2, hp2, hp2],
            in_gain: libm::pow(10.0, d.input_gain_db / 20.0),
            out_scale: libm::pow(10.0, d.output_gain_db / 20.0),
            release: libm::exp(-1.0 / (d.release_s * fs)),
            band: d.band,
            env: [0.0; BANDS],
            state: [[[0.0; 2]; TREE]; MAX_CHANNELS],
        }
    }

    fn reset(&mut self) {
        self.env = [0.0; BANDS];
        self.state = [[[0.0; 2]; TREE]; MAX_CHANNELS];
    }

    #[inline]
    fn step(s: &Section, z: &mut [f64; 2], v: f64) -> f64 {
        let o = s.b0 * v + z[0];
        z[0] = s.b1 * v - s.a1 * o + z[1];
        z[1] = s.b2 * v - s.a2 * o;
        o
    }

    /// The four bands of one sample of channel `c`.
    fn split(&mut self, c: usize, x: f64) -> [f64; BANDS] {
        let (t, z) = (&self.tree, &mut self.state[c]);
        let mut lo = Self::step(&t[0], &mut z[0], x);
        lo = Self::step(&t[1], &mut z[1], lo);
        let mut hi = Self::step(&t[2], &mut z[2], x);
        hi = Self::step(&t[3], &mut z[3], hi);
        let lo_ap = Self::step(&t[4], &mut z[4], lo);
        let b0 = Self::step(&t[5], &mut z[5], lo_ap);
        let mut mid = Self::step(&t[6], &mut z[6], hi);
        mid = Self::step(&t[7], &mut z[7], mid);
        let mut hh = Self::step(&t[8], &mut z[8], hi);
        hh = Self::step(&t[9], &mut z[9], hh);
        let b1 = Self::step(&t[10], &mut z[10], mid);
        let mut b2 = Self::step(&t[11], &mut z[11], hh);
        b2 = Self::step(&t[12], &mut z[12], b2);
        let mut b3 = Self::step(&t[13], &mut z[13], hh);
        b3 = Self::step(&t[14], &mut z[14], b3);
        [b0, b1, b2, b3]
    }

    fn gain(&self, b: usize) -> f64 {
        let band = &self.band[b];
        match band.mode {
            BandMode::Mute => 0.0,
            BandMode::Pass => 1.0,
            BandMode::Compress => {
                let level = 10.0 * libm::log10(self.env[b].max(1e-30));
                let over = level - band.threshold_db;
                let gr = if over > 0.0 { (1.0 / band.ratio - 1.0) * over } else { 0.0 };
                libm::pow(10.0, (band.makeup_db + gr.max(band.range_db)) / 20.0)
            }
        }
    }

    fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let n = bufs.len().min(MAX_CHANNELS);
        let mut split = [[0.0f64; BANDS]; MAX_CHANNELS];
        for i in 0..frames {
            let mut power = [0.0f64; BANDS];
            for c in 0..n {
                let x = bufs[c].get(i).copied().unwrap_or(0.0);
                let x = if x.is_finite() { x.clamp(-INPUT_BOUND, INPUT_BOUND) as f64 } else { 0.0 };
                split[c] = self.split(c, self.in_gain * x);
                for (p, v) in power.iter_mut().zip(split[c].iter()) {
                    *p = p.max(v * v);
                }
            }
            for (e, p) in self.env.iter_mut().zip(power.iter()) {
                *e = p.max(*e * self.release);
            }
            let g = [self.gain(0), self.gain(1), self.gain(2), self.gain(3)];
            for c in 0..n {
                let y = self.out_scale * (0..BANDS).map(|b| g[b] * split[c][b]).sum::<f64>();
                if let Some(o) = bufs[c].get_mut(i) {
                    *o = if y.is_finite() { y as f32 } else { 0.0 };
                }
                if !y.is_finite() {
                    self.state[c] = [[0.0; 2]; TREE];
                }
            }
            if self.env.iter().any(|e| !e.is_finite()) {
                self.env = [0.0; BANDS];
            }
        }
    }
}

/// Running state for one level-dependent ToneMeister slot. Both shapes run
/// `dry + (gain - 1) * carrier` with a fixed carrier (type 15: the detector's
/// constant-peak band pass; type 18: a first-order high-pass at half the
/// corner). The control law:
///
/// - the side chain passes the detector (type 15: the band pass; type 18: a
///   second-order Butterworth high-pass at the corner);
/// - two all-pass branches at half and twice the analysis centre form a
///   quadrature pair; their magnitude is taken on the first frame of every
///   16 (the phase carries across render calls) and mapped through the
///   44-point response curve; the Float32 result is held for 16 frames;
/// - the held gain passes a 20 Hz second-order low-pass whose Float32
///   coefficients and histories follow the platform's single-precision
///   multichannel biquad ([`ControlFilter`]), primed at unity.
///
/// The low-pass's near-unity poles make its Float32 arithmetic part of the
/// response: it settles the idle carrier slightly above unity and every
/// held floor a little away from the curve. That is the installed unit's
/// behaviour and is reproduced, not corrected.
struct DynamicRun {
    fs: f64,
    detector: Section,
    component: Section,
    detector_state: [f32; 2],
    component_state: [[f32; 2]; MAX_CHANNELS],
    quadrature: [Section; 2],
    quadrature_state: [[f32; 2]; 2],
    control_phase: usize,
    held: f32,
    control: ControlFilter,
    envelope: f64,
}

/// Frames per control-filter call; a multiple of the filter's group.
const CONTROL_CHUNK: usize = 64;

impl DynamicRun {
    fn new(params: &DynamicFilter, fs: f64) -> DynamicRun {
        let detector = match params.kind {
            DynamicKind::Notch => {
                // ToneMeister filter type 14: the prewarped bilinear
                // constant-peak band pass used by both the detector and
                // the fixed parallel carrier.
                let k = libm::tan(core::f64::consts::PI * params.frequency_hz / fs);
                let kq = k / params.quality;
                let k2 = k * k;
                let n = 1.0 / (1.0 + kq + k2);
                Section { b0: kq * n, b1: 0.0, b2: -kq * n, a1: 2.0 * (k2 - 1.0) * n, a2: (1.0 - kq + k2) * n }
            }
            DynamicKind::HighShelf => {
                // ToneMeister filter type 9: a second-order Butterworth
                // high-pass at the saved corner.
                butter2(true, params.frequency_hz, fs)
            }
        };
        let component = match params.kind {
            DynamicKind::Notch => detector,
            DynamicKind::HighShelf => {
                let k = libm::tan(core::f64::consts::PI * 0.5 * params.frequency_hz / fs);
                let n = 1.0 / (1.0 + k);
                Section { b0: n, b1: -n, b2: 0.0, a1: (k - 1.0) * n, a2: 0.0 }
            }
        };
        let analysis_hz = match params.kind {
            DynamicKind::Notch => params.frequency_hz,
            DynamicKind::HighShelf => libm::sqrt(params.frequency_hz * 20_000.0).clamp(40.0, 10_000.0),
        };
        let quadrature = [Self::allpass(0.5 * analysis_hz, 0.37, fs), Self::allpass(2.0 * analysis_hz, 0.37, fs)];
        DynamicRun {
            fs,
            detector,
            component,
            detector_state: [0.0; 2],
            component_state: [[0.0; 2]; MAX_CHANNELS],
            quadrature,
            quadrature_state: [[0.0; 2]; 2],
            control_phase: 0,
            held: 1.0,
            control: ControlFilter::new(&Self::lowpass(20.0, f64::from(0.707f32), fs)),
            envelope: 0.0,
        }
    }

    /// Rebuild the coefficient sets for volume automation without resetting
    /// any detector, controller or carrier history. These ToneMeister shapes
    /// update one fixed parallel carrier in place.
    fn reconfigure(&mut self, params: &DynamicFilter, fs: f64) {
        let fresh = Self::new(params, fs);
        self.fs = fs;
        self.detector = fresh.detector;
        self.component = fresh.component;
        self.quadrature = fresh.quadrature;
        self.control.set(&Self::lowpass(20.0, f64::from(0.707f32), fs));
    }

    fn reset(&mut self) {
        self.detector_state = [0.0; 2];
        self.component_state = [[0.0; 2]; MAX_CHANNELS];
        self.quadrature_state = [[0.0; 2]; 2];
        self.control_phase = 0;
        self.held = 1.0;
        self.control.prime(&Self::lowpass(20.0, f64::from(0.707f32), self.fs), 1.0);
        self.envelope = 0.0;
    }

    fn allpass(frequency_hz: f64, quality: f64, fs: f64) -> Section {
        let w = 2.0 * core::f64::consts::PI * frequency_hz.clamp(1.0, 0.49 * fs) / fs;
        let alpha = libm::sin(w) * libm::sinh(0.5 / quality);
        let n = 1.0 / (1.0 + alpha);
        let a1 = -2.0 * libm::cos(w) * n;
        let a2 = (1.0 - alpha) * n;
        Section { b0: a2, b1: a1, b2: 1.0, a1, a2 }
    }

    fn lowpass(frequency_hz: f64, quality: f64, fs: f64) -> Section {
        let w = 2.0 * core::f64::consts::PI * frequency_hz.clamp(1.0, 0.49 * fs) / fs;
        let alpha = libm::sin(w) * libm::sinh(0.5 / quality);
        let n = 1.0 / (1.0 + alpha);
        let b0 = 0.5 * (1.0 - libm::cos(w)) * n;
        Section { b0, b1: 2.0 * b0, b2: b0, a1: -2.0 * libm::cos(w) * n, a2: (1.0 - alpha) * n }
    }

    #[inline]
    fn step_f32(section: &Section, state: &mut [f32; 2], input: f32) -> f32 {
        let (b0, b1, b2, a1, a2) =
            (section.b0 as f32, section.b1 as f32, section.b2 as f32, section.a1 as f32, section.a2 as f32);
        let output = b0 * input + state[0];
        state[0] = b1 * input - a1 * output + state[1];
        state[1] = b2 * input - a2 * output;
        output
    }

    fn curve(params: &DynamicFilter, level: f64) -> f64 {
        const POINTS: usize = 44;
        const STEP: f64 = 100.0 / 43.0;
        let slope = if params.ratio >= 9.9 { -1.0 } else { 1.0 / params.ratio.clamp(0.1, 10.0) - 1.0 };
        let lo = params.range_db.min(0.0);
        let hi = params.range_db.max(0.0);
        let sample = |i: usize| {
            let level_db = -100.0 + STEP * i.min(POINTS - 1) as f64;
            let gain_db = ((level_db - params.threshold_db) * slope).clamp(lo, hi);
            libm::exp(gain_db * core::f64::consts::LN_10 / 20.0)
        };
        let db = 20.0 * libm::log10(level.clamp(1e-5, 1.0));
        let x = ((db + 100.0) * 0.43).clamp(0.0, 43.0);
        let i = x as usize;
        let t = x - i as f64;
        let (previous, current, next, next2) =
            (sample(i.saturating_sub(1)), sample(i), sample((i + 1).min(43)), sample((i + 2).min(43)));
        let a = 0.5 * current - previous / 6.0 - 0.5 * next + next2 / 6.0;
        let b = -current + 0.5 * previous + 0.5 * next;
        let c = -0.5 * previous + 0.5 * next;
        let d = (previous + 4.0 * current + next) / 6.0;
        (((a * t + b) * t + c) * t + d).clamp(libm::exp(lo * core::f64::consts::LN_10 / 20.0), 1.0)
    }

    /// The side-chain sample: the mean of the selected channels.
    #[inline]
    fn sidechain(at: &DynamicAt, bufs: &[&mut [f32]], n: usize, sidechains: usize, i: usize) -> f64 {
        let mut input = 0.0;
        for (c, buf) in bufs.iter().enumerate().take(n) {
            if at.sidechain_mask >> c & 1 != 0 {
                let v = buf[i];
                input += if v.is_finite() { f64::from(v.clamp(-INPUT_BOUND, INPUT_BOUND)) } else { 0.0 };
            }
        }
        input / sidechains as f64
    }

    fn process(&mut self, at: &DynamicAt, bufs: &mut [&mut [f32]], frames: usize) {
        let n = bufs.len().min(MAX_CHANNELS);
        let frames = frames.min(bufs.iter().take(n).map(|b| b.len()).min().unwrap_or(0));
        if frames == 0 {
            return;
        }
        let sidechains = (0..n).filter(|c| at.sidechain_mask >> c & 1 != 0).count();
        if sidechains == 0 {
            return;
        }
        if at.params.smooth {
            // Explicit attack/release: a block envelope on the detector.
            let mut sum_sq = 0.0;
            for i in 0..frames {
                let input = Self::sidechain(at, bufs, n, sidechains, i);
                let detected = Self::step_f32(&self.detector, &mut self.detector_state, input as f32);
                sum_sq += f64::from(detected) * f64::from(detected);
            }
            let measured = core::f64::consts::SQRT_2 * libm::sqrt(sum_sq / frames as f64);
            let time_ms = if measured > self.envelope { at.params.attack_ms } else { at.params.release_ms };
            let coefficient =
                if time_ms <= 0.0 { 0.0 } else { libm::exp(-(frames as f64) / (time_ms * self.fs / 1_000.0)) };
            self.envelope += (1.0 - coefficient) * (measured - self.envelope);
            self.held = Self::curve(&at.params, self.envelope) as f32;
        }
        let mut gain = [0.0f32; CONTROL_CHUNK];
        let mut from = 0;
        while from < frames {
            let len = (frames - from).min(CONTROL_CHUNK);
            for (k, held) in gain.iter_mut().enumerate().take(len) {
                if !at.params.smooth {
                    let input = Self::sidechain(at, bufs, n, sidechains, from + k);
                    let detected = Self::step_f32(&self.detector, &mut self.detector_state, input as f32);
                    let a = Self::step_f32(&self.quadrature[0], &mut self.quadrature_state[0], detected);
                    let b = Self::step_f32(&self.quadrature[1], &mut self.quadrature_state[1], detected);
                    if self.control_phase == 0 {
                        self.envelope = f64::from(libm::sqrtf(a * a + b * b));
                        self.held = Self::curve(&at.params, self.envelope) as f32;
                    }
                    self.control_phase = (self.control_phase + 1) & 15;
                }
                *held = self.held;
            }
            self.control.run(&mut gain[..len]);
            for (c, buf) in bufs.iter_mut().enumerate().take(n) {
                if at.active_mask >> c & 1 == 0 {
                    continue;
                }
                for (sample, g) in buf[from..from + len].iter_mut().zip(gain.iter()) {
                    let input = if sample.is_finite() { sample.clamp(-INPUT_BOUND, INPUT_BOUND) } else { 0.0 };
                    let component = Self::step_f32(&self.component, &mut self.component_state[c], input);
                    let output = input + (g - 1.0) * component;
                    *sample = if output.is_finite() { output } else { 0.0 };
                }
            }
            from += len;
        }
    }
}

/// The ToneMeister control low-pass, computed the way the platform's
/// single-precision multichannel biquad computes one section on one channel.
///
/// The unit designs the section in double, rounds the five coefficients to
/// Float32 and hands those (widened) to the platform filter, which:
///
/// - prepares, in double, the first eight terms of the denominator response
///   `D` and of the complete impulse response `H`, each rounded once to
///   Float32;
/// - keeps a transposed state `z1, z2` plus the last two inputs and outputs;
/// - runs every call as whole eight-frame groups, then one four-frame group
///   if four or more frames remain, then single frames; a call boundary
///   restarts that schedule but keeps the state;
/// - evaluates a group as a triangular matrix of `H` over the group's inputs
///   plus `D` over the entry state, lane by lane as one rounded product and
///   a chain of Float32 fused multiply-adds; lanes 4 to 7 of an eight-frame
///   group sum two partial chains (first half of the inputs with `z1`;
///   second half with `z2`) with a final plain addition;
/// - republishes `z1, z2` after a group from its last two inputs and
///   outputs, and steps single frames as fused transposed direct form II.
///
/// With poles this close to unity the Float32 rounding is part of the
/// response (the idle carrier settles at about 1.01366, not at the
/// coefficients' DC gain). The unit primes the filter by running a constant
/// through an identity section before installing the low-pass.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ControlFilter {
    /// Direct coefficients as supplied (Float32 values, widened).
    coefficients: [f64; 5],
    b: [f32; 3],
    a: [f32; 2],
    d: [f32; 8],
    h: [f32; 8],
    z: [f32; 2],
    /// x[n-2], x[n-1], y[n-2], y[n-1]
    history: [f32; 4],
}

impl ControlFilter {
    const IDENTITY: [f64; 5] = [1.0, 0.0, 0.0, 0.0, 0.0];

    fn new(section: &Section) -> ControlFilter {
        let mut filter = ControlFilter {
            coefficients: [0.0; 5],
            b: [0.0; 3],
            a: [0.0; 2],
            d: [0.0; 8],
            h: [0.0; 8],
            z: [0.0; 2],
            history: [0.0; 4],
        };
        filter.prime(section, 1.0);
        filter
    }

    fn rounded(section: &Section) -> [f64; 5] {
        [section.b0, section.b1, section.b2, section.a1, section.a2].map(|v| f64::from(v as f32))
    }

    /// Reset, run `value` twice through an identity section, then install
    /// the section.
    fn prime(&mut self, section: &Section, value: f32) {
        self.z = [0.0; 2];
        self.history = [0.0; 4];
        self.install(Self::IDENTITY);
        let mut constant = [value; 2];
        self.run(&mut constant);
        self.install(Self::rounded(section));
    }

    /// Install new coefficients: rebuild the group terms and convert the
    /// published histories to the new transposed state.
    fn set(&mut self, section: &Section) {
        let c = Self::rounded(section);
        if c != self.coefficients {
            self.install(c);
        }
    }

    fn install(&mut self, c: [f64; 5]) {
        let [b0, b1, b2, a1, a2] = c;
        let mut g = [0.0f64; 8];
        let mut h = [0.0f64; 8];
        g[0] = 1.0;
        g[1] = -a1;
        for i in 2..8 {
            g[i] = libm::fma(-a1, g[i - 1], (-a2) * g[i - 2]);
        }
        h[0] = b0;
        h[1] = libm::fma(g[1], b0, b1);
        for i in 2..8 {
            h[i] = libm::fma(g[i], b0, libm::fma(g[i - 1], b1, b2 * g[i - 2]));
        }
        self.coefficients = c;
        self.d = g.map(|v| v as f32);
        self.h = h.map(|v| v as f32);
        self.b = [b0 as f32, b1 as f32, b2 as f32];
        self.a = [a1 as f32, a2 as f32];
        let [x2, x1, y2, y1] = self.history.map(f64::from);
        let mut z1 = b1 * x1;
        z1 = libm::fma(b2, x2, z1);
        z1 = libm::fma(-a2, y2, z1);
        z1 = libm::fma(-a1, y1, z1);
        let z2 = libm::fma(b2, x1, (-a2) * y1);
        self.z = [z1 as f32, z2 as f32];
    }

    #[inline]
    fn d(&self, k: isize) -> f32 {
        if k < 0 {
            0.0
        } else {
            self.d[k as usize]
        }
    }

    #[inline]
    fn h(&self, k: isize) -> f32 {
        if k < 0 {
            0.0
        } else {
            self.h[k as usize]
        }
    }

    /// One lane's chain: a rounded product, then fused multiply-adds.
    #[inline]
    fn chain(terms: &[(f32, f32)]) -> f32 {
        let mut acc = terms[0].0 * terms[0].1;
        for &(c, v) in &terms[1..] {
            acc = libm::fmaf(c, v, acc);
        }
        acc
    }

    fn group4(&self, x: &[f32; 4], y: &mut [f32]) {
        let [z1, z2] = self.z;
        for (i, out) in y.iter_mut().enumerate().take(4) {
            let i = i as isize;
            *out = Self::chain(&[
                (self.h(i), x[0]),
                (self.h(i - 1), x[1]),
                (self.h(i - 2), x[2]),
                (self.h(i - 3), x[3]),
                (self.d(i), z1),
                (self.d(i - 1), z2),
            ]);
        }
    }

    fn group8(&self, x: &[f32; 8], y: &mut [f32]) {
        let [z1, z2] = self.z;
        self.group4(&[x[0], x[1], x[2], x[3]], &mut y[..4]);
        for i in 4..8isize {
            let first = Self::chain(&[
                (self.h(i), x[0]),
                (self.h(i - 1), x[1]),
                (self.h(i - 2), x[2]),
                (self.h(i - 3), x[3]),
                (self.d(i), z1),
            ]);
            let second = Self::chain(&[
                (self.h(i - 4), x[4]),
                (self.d(i - 1), z2),
                (self.h(i - 5), x[5]),
                (self.h(i - 6), x[6]),
                (self.h(i - 7), x[7]),
            ]);
            y[i as usize] = first + second;
        }
    }

    /// The transposed state after a group, from its last two inputs and
    /// outputs (Float32; the zero terms are kept for signed-zero parity).
    fn publish(&mut self, x: (f32, f32), y: (f32, f32)) {
        let [_, b1, b2] = self.b;
        let [a1, a2] = self.a;
        let mut n1 = b2 * x.0;
        let mut n2 = 0.0 * x.0;
        n1 = libm::fmaf(b1, x.1, n1);
        n2 = libm::fmaf(b2, x.1, n2);
        n1 = libm::fmaf(-a2, y.0, n1);
        n2 = libm::fmaf(-0.0, y.0, n2);
        n1 = libm::fmaf(-a1, y.1, n1);
        n2 = libm::fmaf(-a2, y.1, n2);
        self.z = [n1, n2];
    }

    /// Filters one call's frames in place.
    fn run(&mut self, buf: &mut [f32]) {
        let n = buf.len();
        if n == 0 {
            return;
        }
        let (old_x1, old_y1) = (self.history[1], self.history[3]);
        let (mut last_x, mut prev_x) = (0.0f32, 0.0f32);
        let mut at = 0;
        while n - at >= 8 {
            let mut x = [0.0f32; 8];
            x.copy_from_slice(&buf[at..at + 8]);
            self.group8(&x, &mut buf[at..at + 8]);
            self.publish((x[6], x[7]), (buf[at + 6], buf[at + 7]));
            (prev_x, last_x) = (x[6], x[7]);
            at += 8;
        }
        if n - at >= 4 {
            let mut x = [0.0f32; 4];
            x.copy_from_slice(&buf[at..at + 4]);
            self.group4(&x, &mut buf[at..at + 4]);
            self.publish((x[2], x[3]), (buf[at + 2], buf[at + 3]));
            (prev_x, last_x) = (x[2], x[3]);
            at += 4;
        }
        let [b0, b1, b2] = self.b;
        let [a1, a2] = self.a;
        while at < n {
            let x = buf[at];
            let [z1, z2] = self.z;
            let y = libm::fmaf(b0, x, z1);
            let t = libm::fmaf(b1, x, z2);
            let next1 = libm::fmaf(-a1, y, t);
            let p = (-a2) * y;
            let next2 = libm::fmaf(b2, x, p);
            self.z = [next1, next2];
            buf[at] = y;
            (prev_x, last_x) = (last_x, x);
            at += 1;
        }
        self.history = if n == 1 { [old_x1, last_x, old_y1, buf[0]] } else { [prev_x, last_x, buf[n - 2], buf[n - 1]] };
    }
}

/// The running pre-stage: transposed direct form II, f64 state.
pub struct PreStage {
    params: Params,
    state: [[[f64; 2]; MAX_SECTIONS]; MAX_CHANNELS],
    dynamics: Option<DynRun>,
    /// Each box's run state (filters and detectors for every channel) is
    /// allocated once here, never in the real-time path.
    boxes: [Option<alloc::boxed::Box<mstk::Run>>; MAX_BOXES],
    dynamic: [Option<DynamicRun>; MAX_DYNAMIC_FILTERS],
}

impl PreStage {
    /// At 48 kHz; see [`PreStage::with_rate`].
    pub fn new(params: Params) -> PreStage {
        PreStage::with_rate(params, 48_000.0)
    }

    /// `fs`: the rate the parameters were checked for (the dynamics'
    /// filters are built for it).
    pub fn with_rate(params: Params, fs: f64) -> PreStage {
        let dynamics = params.dynamics.as_ref().map(|d| DynRun::new(d, fs));
        let boxes = core::array::from_fn(|k| {
            (k < params.box_count.min(MAX_BOXES))
                .then(|| alloc::boxed::Box::new(mstk::Run::new(&params.boxes[k].params, fs, params.channels)))
        });
        let dynamic = core::array::from_fn(|k| {
            (k < params.dynamic_count.min(MAX_DYNAMIC_FILTERS)).then(|| DynamicRun::new(&params.dynamic[k].params, fs))
        });
        PreStage { params, state: [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS], dynamics, boxes, dynamic }
    }

    pub fn params(&self) -> &Params {
        &self.params
    }

    /// Replaces one channel's coefficients without touching its history.
    /// The caller must supply a checked channel. This is bounded and does
    /// not allocate, so graph-parameter updates may use it in real time.
    pub(crate) fn set_channel(&mut self, c: usize, channel: Channel) {
        if c < self.params.channels {
            self.params.channel[c] = channel;
        }
    }

    pub(crate) fn channel_mut(&mut self, c: usize) -> Option<&mut Channel> {
        (c < self.params.channels).then(|| &mut self.params.channel[c])
    }

    pub(crate) fn set_dynamic_filter(&mut self, index: usize, params: DynamicFilter, fs: f64) {
        if index < self.params.dynamic_count.min(MAX_DYNAMIC_FILTERS) && params.valid(fs) {
            self.params.dynamic[index].params = params;
            if let Some(run) = &mut self.dynamic[index] {
                run.reconfigure(&params, fs);
            } else {
                self.dynamic[index] = Some(DynamicRun::new(&params, fs));
            }
        }
    }

    /// Clears every filter's history and the dynamics' detectors (stream start).
    pub fn reset(&mut self) {
        self.state = [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS];
        if let Some(d) = &mut self.dynamics {
            d.reset();
        }
        for b in self.boxes.iter_mut().flatten() {
            b.reset();
        }
        for d in self.dynamic.iter_mut().flatten() {
            d.reset();
        }
    }

    /// Runs channel `c`'s sections `from..to` in place on `buf`, then `gain`.
    fn run_sections(&mut self, c: usize, from: usize, to: usize, gain: f64, buf: &mut [f32]) {
        let (Some(ch), Some(z)) = (self.params.channel.get(c), self.state.get_mut(c)) else {
            buf.fill(0.0);
            return;
        };
        let to = to.min(ch.count).min(MAX_SECTIONS);
        let from = from.min(to);
        for y in buf.iter_mut() {
            let mut v = if y.is_finite() { y.clamp(-INPUT_BOUND, INPUT_BOUND) as f64 } else { 0.0 };
            for (s, st) in ch.sections[from..to].iter().zip(z[from..to].iter_mut()) {
                let out = s.b0 * v + st[0];
                st[0] = s.b1 * v - s.a1 * out + st[1];
                st[1] = s.b2 * v - s.a2 * out;
                v = out;
            }
            v *= gain;
            if v.is_finite() {
                *y = v as f32;
            } else {
                for st in z[from..to].iter_mut() {
                    *st = [0.0; 2];
                }
                *y = 0.0;
            }
        }
    }

    /// Float-coefficient counterpart of [`Self::run_sections`] for a
    /// ToneMeister box. Its static cascade rounds after every section.
    fn run_sections_float(&mut self, c: usize, from: usize, to: usize, gain: f64, buf: &mut [f32]) {
        let (Some(ch), Some(z)) = (self.params.channel.get(c), self.state.get_mut(c)) else {
            buf.fill(0.0);
            return;
        };
        let to = to.min(ch.count).min(MAX_SECTIONS);
        let from = from.min(to);
        for y in buf.iter_mut() {
            let mut v = if y.is_finite() { y.clamp(-INPUT_BOUND, INPUT_BOUND) } else { 0.0 };
            for (s, st) in ch.sections[from..to].iter().zip(z[from..to].iter_mut()) {
                let out = s.b0 as f32 * v + st[0] as f32;
                let z0 = s.b1 as f32 * v - s.a1 as f32 * out + st[1] as f32;
                let z1 = s.b2 as f32 * v - s.a2 as f32 * out;
                *st = [z0 as f64, z1 as f64];
                v = out;
            }
            v *= gain as f32;
            if v.is_finite() {
                *y = v;
            } else {
                for st in z[from..to].iter_mut() {
                    *st = [0.0; 2];
                }
                *y = 0.0;
            }
        }
    }

    /// The whole pre-stage in place on every channel (`bufs[c]` holds
    /// channel `c`'s input and gets its output): each channel's sections
    /// and gains up to the first box, the box (linked), the next sections,
    /// and so on, then the channel's own gain; the dynamics block, if any,
    /// last. Channels the pre-stage does not have are muted. Real time.
    pub fn process_block(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let n = bufs.len().min(MAX_CHANNELS);
        for (c, b) in bufs.iter_mut().enumerate().take(n) {
            if c >= self.params.channels {
                b.fill(0.0);
            }
        }
        let n = n.min(self.params.channels);
        let count = self.params.box_count.min(MAX_BOXES);
        let mut from = [0usize; MAX_CHANNELS];
        let mut dynamic_index = 0usize;
        for k in 0..=count {
            while dynamic_index < self.params.dynamic_count.min(MAX_DYNAMIC_FILTERS)
                && self.params.dynamic[dynamic_index].before_box == k
            {
                let at = self.params.dynamic[dynamic_index];
                for c in 0..n {
                    let len = frames.min(bufs[c].len());
                    self.run_sections(c, from[c], at.split[c], 1.0, &mut bufs[c][..len]);
                    from[c] = at.split[c];
                }
                if let Some(run) = &mut self.dynamic[dynamic_index] {
                    run.process(&at, &mut bufs[..n], frames);
                }
                dynamic_index += 1;
            }
            for c in 0..n {
                let len = frames.min(bufs[c].len());
                let (to, gain) = if k < count {
                    (self.params.boxes[k].split[c], self.params.boxes[k].gain[c])
                } else {
                    (self.params.channel[c].count, self.params.channel[c].gain)
                };
                self.run_sections(c, from[c], to, gain, &mut bufs[c][..len]);
                from[c] = to;
            }
            if k < count {
                if let Some(run) = &mut self.boxes[k] {
                    run.process(&mut bufs[..n], frames);
                }
            }
        }
        if let Some(d) = &mut self.dynamics {
            d.process(&mut bufs[..n], frames);
        }
    }

    /// [`Self::process_block`] with the installed Tone box's float static
    /// cascade. Dynamic slots remain linked and ordered across channels.
    pub(crate) fn process_block_float(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let n = bufs.len().min(MAX_CHANNELS);
        for (c, b) in bufs.iter_mut().enumerate().take(n) {
            if c >= self.params.channels {
                b.fill(0.0);
            }
        }
        let n = n.min(self.params.channels);
        let count = self.params.box_count.min(MAX_BOXES);
        let mut from = [0usize; MAX_CHANNELS];
        let mut dynamic_index = 0usize;
        for k in 0..=count {
            while dynamic_index < self.params.dynamic_count.min(MAX_DYNAMIC_FILTERS)
                && self.params.dynamic[dynamic_index].before_box == k
            {
                let at = self.params.dynamic[dynamic_index];
                for c in 0..n {
                    let len = frames.min(bufs[c].len());
                    self.run_sections_float(c, from[c], at.split[c], 1.0, &mut bufs[c][..len]);
                    from[c] = at.split[c];
                }
                if let Some(run) = &mut self.dynamic[dynamic_index] {
                    run.process(&at, &mut bufs[..n], frames);
                }
                dynamic_index += 1;
            }
            for c in 0..n {
                let len = frames.min(bufs[c].len());
                let (to, gain) = if k < count {
                    (self.params.boxes[k].split[c], self.params.boxes[k].gain[c])
                } else {
                    (self.params.channel[c].count, self.params.channel[c].gain)
                };
                self.run_sections_float(c, from[c], to, gain, &mut bufs[c][..len]);
                from[c] = to;
            }
            if k < count {
                if let Some(run) = &mut self.boxes[k] {
                    run.process(&mut bufs[..n], frames);
                }
            }
        }
        if let Some(d) = &mut self.dynamics {
            d.process(&mut bufs[..n], frames);
        }
    }

    /// Runs the dynamics on every channel at once (after [`PreStage::process`]
    /// has filtered each into `bufs`), in place. No dynamics: unchanged.
    /// Real time.
    pub fn process_dynamics(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        if let Some(d) = &mut self.dynamics {
            d.process(bufs, frames);
        }
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

    /// The filter-only float path used by a ToneMeister box. The installed
    /// box runs its biquad cascade in float, whose rounding is observable
    /// when many low-frequency sections are cascaded. The protection
    /// pre-stage deliberately keeps the f64 path above.
    pub(crate) fn process_float(&mut self, c: usize, input: &[f32], output: &mut [f32]) {
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
            let mut v = if x.is_finite() { x.clamp(-INPUT_BOUND, INPUT_BOUND) } else { 0.0 };
            for (s, st) in secs.iter().zip(z.iter_mut()) {
                let out = s.b0 as f32 * v + st[0] as f32;
                let z0 = s.b1 as f32 * v - s.a1 as f32 * out + st[1] as f32;
                let z1 = s.b2 as f32 * v - s.a2 as f32 * out;
                st[0] = z0 as f64;
                st[1] = z1 as f64;
                v = out;
            }
            v *= ch.gain as f32;
            if v.is_finite() {
                *y = v;
            } else {
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

    fn dyn_params() -> Params {
        let mut p = Params::identity(2);
        p.flags = FLAG_FROM_TUNING | FLAG_DYNAMICS;
        let b = |t: f64, r: f64, rng: f64, mk: f64| Band {
            threshold_db: t,
            ratio: r,
            range_db: rng,
            makeup_db: mk,
            mode: BandMode::Compress,
        };
        p.dynamics = Some(Dynamics {
            crossover_hz: [200.0, 1500.0, 8000.0],
            input_gain_db: -12.0,
            output_gain_db: 14.0,
            release_s: 0.05,
            band: [
                b(-30.0, 4.0, -20.0, 2.0),
                b(-25.0, 2.5, -18.0, 0.0),
                b(-15.0, 1.8, -18.0, 0.0),
                b(-14.0, 1.5, -16.0, 0.0),
            ],
        });
        p
    }

    fn tone(f: f64, a: f64, n: usize) -> std::vec::Vec<f32> {
        (0..n).map(|i| (a * libm::sin(2.0 * core::f64::consts::PI * f * i as f64 / 48_000.0)) as f32).collect()
    }

    fn run_dyn(p: &Params, x: &[f32]) -> std::vec::Vec<f32> {
        let mut s = PreStage::with_rate(p.clone(), 48_000.0);
        let mut a = x.to_vec();
        let mut b = x.to_vec();
        for k in (0..x.len()).step_by(512) {
            let e = (k + 512).min(x.len());
            let mut bufs: [&mut [f32]; 2] = [&mut a[k..e], &mut b[k..e]];
            s.process_dynamics(&mut bufs, e - k);
        }
        a
    }

    fn level(y: &[f32], f: f64) -> f64 {
        let n = y.len() / 2;
        let (mut re, mut im) = (0.0, 0.0);
        for (i, v) in y[n..].iter().enumerate() {
            let w = 2.0 * core::f64::consts::PI * f * (i + n) as f64 / 48_000.0;
            re += *v as f64 * libm::cos(w);
            im += *v as f64 * libm::sin(w);
        }
        20.0 * libm::log10(2.0 * libm::sqrt(re * re + im * im) / n as f64)
    }

    #[test]
    fn dynamics_roundtrip_and_refusals() {
        let p = dyn_params();
        let b = p.encode();
        let d = Params::decode(&b, 2, 48_000.0).unwrap();
        assert_eq!(d, p);
        let mut bad = p.clone();
        bad.dynamics.as_mut().unwrap().crossover_hz = [2000.0, 1500.0, 8000.0];
        assert_eq!(Params::decode(&bad.encode(), 2, 48_000.0), Err(BlobError::Unstable));
        let mut bad = p.clone();
        bad.dynamics.as_mut().unwrap().band[0].ratio = 0.5;
        assert_eq!(Params::decode(&bad.encode(), 2, 48_000.0), Err(BlobError::Unstable));
    }

    #[test]
    fn dynamics_carry_the_units_gain_and_reduce_at_full_scale() {
        let p = dyn_params();
        let n = 24_000;
        // quiet: the unit's own small-signal gain, input + output + the band's
        // makeup (bands sum in phase), never more than the largest of them
        let d = p.dynamics.unwrap();
        let net = d.input_gain_db + d.output_gain_db;
        for (f, band) in [(30.0, 0), (50.0, 0), (500.0, 1), (3500.0, 2), (16_000.0, 3)] {
            let y = run_dyn(&p, &tone(f, 1e-3, n));
            let g = level(&y, f) - 20.0 * libm::log10(1e-3);
            let want = net + d.band[band].makeup_db;
            assert!((g - want).abs() < 0.1, "{f} Hz: {g} dB, want {want}");
        }
        let top = net + d.band.iter().map(|b| b.makeup_db).fold(f64::NEG_INFINITY, f64::max);
        for f in [200.0, 1500.0, 8000.0] {
            let y = run_dyn(&p, &tone(f, 1e-3, n));
            let g = level(&y, f) - 20.0 * libm::log10(1e-3);
            assert!(g <= top + 0.01 && g > net - 0.5, "{f} Hz: {g} dB");
        }
        // full scale in band 0: the hard-knee curve, from the band level
        let y = run_dyn(&p, &tone(80.0, 1.0, n));
        let lvl = d.input_gain_db; // band 0 carries the whole 80 Hz tone (to within 0.1 dB)
        let gr = ((1.0 / 4.0 - 1.0) * (lvl - (-30.0))).max(-20.0);
        let want = d.input_gain_db + d.output_gain_db + 2.0 + gr;
        // within 1 dB: the held power droops between the tone's peaks with
        // the release, so the mean reduction is a little under the curve's
        assert!((level(&y, 80.0) - want).abs() < 1.0, "{} vs {want}", level(&y, 80.0));
        // a muted band takes its range away
        let mut m = dyn_params();
        m.dynamics.as_mut().unwrap().band[0].mode = BandMode::Mute;
        assert!(level(&run_dyn(&m, &tone(50.0, 0.1, n)), 50.0) < -40.0);
    }

    fn dynamic_filter_params(kind: DynamicKind) -> Params {
        let mut p = Params::identity(1);
        p.flags = FLAG_FROM_TUNING | FLAG_DYNAMIC_FILTERS;
        p.dynamic_count = 1;
        p.dynamic[0] = DynamicAt {
            active_mask: 1,
            sidechain_mask: 1,
            params: DynamicFilter {
                kind,
                range_db: if kind == DynamicKind::Notch { -12.0 } else { -3.6 },
                frequency_hz: if kind == DynamicKind::Notch { 1_000.0 } else { 9_500.0 },
                quality: 2.0,
                ratio: if kind == DynamicKind::Notch { 4.0 } else { 3.0 },
                threshold_db: if kind == DynamicKind::Notch { -20.0 } else { -29.67 },
                attack_ms: 20.0,
                release_ms: 100.0,
                smooth: false,
            },
            ..Default::default()
        };
        p
    }

    fn run_filter(p: Params, f: f64, amplitude: f64) -> f64 {
        let n = 48_000;
        let mut signal = tone(f, amplitude, n);
        let mut run = PreStage::with_rate(p, 48_000.0);
        for at in (0..n).step_by(512) {
            let end = (at + 512).min(n);
            let mut bufs = [&mut signal[at..end]];
            run.process_block(&mut bufs, end - at);
        }
        level(&signal, f) - 20.0 * libm::log10(amplitude)
    }

    #[test]
    fn dynamic_filters_roundtrip_and_follow_level() {
        for kind in [DynamicKind::Notch, DynamicKind::HighShelf] {
            let p = dynamic_filter_params(kind);
            assert_eq!(Params::decode(&p.encode(), 1, 48_000.0).unwrap(), p);
        }
        let notch = dynamic_filter_params(DynamicKind::Notch);
        // Quiet input leaves the carrier at the control filter's idle
        // point: unity or a fraction of a decibel above it.
        let quiet = run_filter(notch.clone(), 1_000.0, 1e-3);
        assert!((-0.001..0.15).contains(&quiet), "{quiet}");
        assert!(run_filter(notch, 1_000.0, 1.0) < -11.5);
        let shelf = dynamic_filter_params(DynamicKind::HighShelf);
        assert!(run_filter(shelf.clone(), 1_000.0, 1.0).abs() < 0.15);
        assert!(run_filter(shelf, 12_000.0, 1.0) < -3.0);
    }

    #[test]
    fn type18_bounds_the_quadrature_analysis_center() {
        let p = dynamic_filter_params(DynamicKind::HighShelf);
        let run = DynamicRun::new(&p.dynamic[0].params, 48_000.0);
        assert_eq!(run.quadrature[0], DynamicRun::allpass(5_000.0, 0.37, 48_000.0));
        assert_eq!(run.quadrature[1], DynamicRun::allpass(20_000.0, 0.37, 48_000.0));
    }

    #[test]
    fn control_filter_starts_settled_at_its_primed_value() {
        let mut filter = ControlFilter::new(&DynamicRun::lowpass(20.0, f64::from(0.707f32), 48_000.0));
        let mut ones = [1.0f32; 64];
        filter.run(&mut ones);
        assert!(ones.iter().all(|v| (v - 1.0).abs() < 1e-3));
        let mut floor = [0.5f32; 64];
        filter.run(&mut floor);
        // Second-order low-pass: the first frames barely move.
        assert!(floor[0] > 0.999 && floor[63] < floor[0]);
    }

    #[test]
    fn dynamic_filter_control_is_sampled_every_16_frames_across_calls() {
        let p = dynamic_filter_params(DynamicKind::Notch);
        let at = p.dynamic[0];
        let mut whole = DynamicRun::new(&at.params, 48_000.0);
        let mut split = DynamicRun::new(&at.params, 48_000.0);
        let x = tone(1_000.0, 1.0, 1024);
        let mut a = x.clone();
        whole.process(&at, &mut [&mut a], 1024);
        let mut b = x.clone();
        let (left, right) = b.split_at_mut(512);
        split.process(&at, &mut [left], 512);
        split.process(&at, &mut [right], 512);
        assert_eq!(a, b);
        assert_eq!(whole.control_phase, split.control_phase);
    }

    #[test]
    fn dynamic_filter_automation_retains_running_state() {
        let p = dynamic_filter_params(DynamicKind::Notch);
        let at = p.dynamic[0];
        let mut run = DynamicRun::new(&at.params, 48_000.0);
        let mut signal = tone(1_000.0, 1.0, 512);
        run.process(&at, &mut [&mut signal], 512);

        let detector_state = run.detector_state;
        let component_state = run.component_state;
        let quadrature_state = run.quadrature_state;
        let control = run.control.history;
        let phase = run.control_phase;
        let held = run.held;
        let envelope = run.envelope;

        let mut moved = at.params;
        moved.frequency_hz = 1_200.0;
        run.reconfigure(&moved, 48_000.0);
        assert_eq!(run.detector_state, detector_state);
        assert_eq!(run.component_state, component_state);
        assert_eq!(run.quadrature_state, quadrature_state);
        assert_eq!(run.control.history, control);
        assert_eq!(run.control_phase, phase);
        assert_eq!(run.held, held);
        assert_eq!(run.envelope, envelope);
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
