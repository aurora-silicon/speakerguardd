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
//!
//! With [`FLAG_MID_SIDE`], a mid/side block follows (after the dynamics
//! block, when there is one): a pair of two-channel matrix boxes around a
//! filter box, as a graph that equalizes mid and side apart sets them
//! (`toMS`, `msEQ`, `fromMS`). Layout: the matrix into the mid/side domain
//! (4 f64, row-major), the matrix back (4 f64), then the mid and the side
//! channel as in the body above. It runs on channels 0 and 1 after the
//! dynamics: `out = from * diag(mid, side) * to * in`. Each output's worst
//! case over in-phase inputs (the sum of its row's magnitudes) stays at or
//! under unity at every frequency, so the block never adds drive either.

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
/// `flags` bit: a [`Dynamics`] block follows the channels.
pub const FLAG_DYNAMICS: u32 = 1 << 1;
/// `flags` bit: a [`MidSide`] block follows (after the dynamics block).
pub const FLAG_MID_SIDE: u32 = 1 << 2;
/// Bands of the multiband dynamics.
pub const BANDS: usize = 4;
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
    fn valid(&self, fs: f64) -> bool {
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

    /// `H` at `f` Hz for sample rate `fs`, as (re, im).
    pub fn response(&self, f: f64, fs: f64) -> (f64, f64) {
        let w = 2.0 * core::f64::consts::PI * f / fs;
        let (c1, s1) = (libm::cos(w), -libm::sin(w));
        let (c2, s2) = (libm::cos(2.0 * w), -libm::sin(2.0 * w));
        let (nr, ni) = (self.b0 + self.b1 * c1 + self.b2 * c2, self.b1 * s1 + self.b2 * s2);
        let (dr, di) = (1.0 + self.a1 * c1 + self.a2 * c2, self.a1 * s1 + self.a2 * s2);
        let d = dr * dr + di * di;
        ((nr * dr + ni * di) / d, (ni * dr - nr * di) / d)
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

    /// `H` of the whole channel at `f` Hz, as (re, im).
    pub fn response(&self, f: f64, fs: f64) -> (f64, f64) {
        self.sections().iter().fold((self.gain, 0.0), |(ar, ai), s| {
            let (br, bi) = s.response(f, fs);
            (ar * br - ai * bi, ar * bi + ai * br)
        })
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

/// A mid/side section: `out = from * diag(mid, side) * to * in` on
/// channels 0 and 1, matrices row-major.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MidSide {
    pub to: [f64; 4],
    pub from: [f64; 4],
    pub mid: Channel,
    pub side: Channel,
}

impl MidSide {
    /// The block's two outputs at `f` Hz: for each, the sum of the
    /// magnitudes of its row of `from * diag(mid, side) * to` (the largest
    /// output any pair of inputs with magnitude 1 can make).
    pub fn row_gain(&self, f: f64, fs: f64) -> [f64; 2] {
        let (m, s) = (self.mid.response(f, fs), self.side.response(f, fs));
        let (t, b) = (self.to, self.from);
        let mut out = [0.0; 2];
        for (r, o) in out.iter_mut().enumerate() {
            for c in 0..2 {
                // from[r][0] * mid * to[0][c] + from[r][1] * side * to[1][c]
                let (km, ks) = (b[2 * r] * t[c], b[2 * r + 1] * t[2 + c]);
                let (re, im) = (km * m.0 + ks * s.0, km * m.1 + ks * s.1);
                *o += libm::sqrt(re * re + im * im);
            }
        }
        out
    }

    /// The larger output's worst case on [`check_grid`].
    pub fn peak(&self, fs: f64) -> f64 {
        let mut peak = 0.0f64;
        check_grid(fs, |f| {
            let [a, b] = self.row_gain(f, fs);
            peak = peak.max(a).max(b);
        });
        peak
    }

    fn valid(&self, fs: f64) -> bool {
        self.to.iter().chain(self.from.iter()).all(|v| v.is_finite())
            && [&self.mid, &self.side].iter().all(|c| c.gain.is_finite() && c.sections().iter().all(|s| s.stable()))
            && self.peak(fs) <= UNITY_TOLERANCE
    }
}

/// A decoded pre-stage section of a blob.
#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    pub channels: usize,
    pub flags: u32,
    pub channel: [Channel; MAX_CHANNELS],
    /// Present exactly when `flags` has [`FLAG_DYNAMICS`].
    pub dynamics: Option<Dynamics>,
    /// Present exactly when `flags` has [`FLAG_MID_SIDE`].
    pub mid_side: Option<MidSide>,
}

impl Params {
    /// Every channel passes unchanged; nothing derived from a tuning.
    pub fn identity(channels: usize) -> Params {
        Params {
            channels: channels.min(MAX_CHANNELS),
            flags: 0,
            channel: [Channel::IDENTITY; MAX_CHANNELS],
            dynamics: None,
            mid_side: None,
        }
    }

    pub fn from_tuning(&self) -> bool {
        self.flags & FLAG_FROM_TUNING != 0
    }

    /// Encoded section, ready to follow a unit payload.
    #[cfg(any(test, feature = "std"))]
    pub fn encode(&self) -> std::vec::Vec<u8> {
        let mut body = std::vec::Vec::new();
        let put = |body: &mut std::vec::Vec<u8>, ch: &Channel| {
            body.extend_from_slice(&(ch.count as u32).to_le_bytes());
            body.extend_from_slice(&0u32.to_le_bytes());
            body.extend_from_slice(&ch.gain.to_le_bytes());
            for s in ch.sections() {
                for v in [s.b0, s.b1, s.b2, s.a1, s.a2] {
                    body.extend_from_slice(&v.to_le_bytes());
                }
            }
        };
        for ch in &self.channel[..self.channels] {
            put(&mut body, ch);
        }
        let mut flags = self.flags & !(FLAG_DYNAMICS | FLAG_MID_SIDE);
        if let Some(d) = &self.dynamics {
            flags |= FLAG_DYNAMICS;
            for v in d.words() {
                body.extend_from_slice(&v.to_le_bytes());
            }
        }
        if let Some(ms) = &self.mid_side {
            flags |= FLAG_MID_SIDE;
            for v in ms.to.iter().chain(ms.from.iter()) {
                body.extend_from_slice(&v.to_le_bytes());
            }
            put(&mut body, &ms.mid);
            put(&mut body, &ms.side);
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
        let mut p =
            Params { channels: n, flags, channel: [Channel::IDENTITY; MAX_CHANNELS], dynamics: None, mid_side: None };
        let f64_at = |i: usize| -> Result<f64, BlobError> {
            let b: [u8; 8] = body.get(i..i + 8).ok_or(BlobError::Length)?.try_into().map_err(|_| BlobError::Length)?;
            Ok(f64::from_le_bytes(b))
        };
        let mut at = 0usize;
        // one channel's filters at `at`; stable sections, finite gain
        let get = |at: &mut usize, ch: &mut Channel| -> Result<(), BlobError> {
            if body.len() < *at + 16 {
                return Err(BlobError::Length);
            }
            let count = word(body, *at) as usize;
            if count > MAX_SECTIONS || word(body, *at + 4) != 0 {
                return Err(BlobError::Length);
            }
            ch.count = count;
            ch.gain = f64_at(*at + 8)?;
            *at += 16;
            for s in ch.sections.iter_mut().take(count) {
                *s = Section {
                    b0: f64_at(*at)?,
                    b1: f64_at(*at + 8)?,
                    b2: f64_at(*at + 16)?,
                    a1: f64_at(*at + 24)?,
                    a2: f64_at(*at + 32)?,
                };
                *at += 40;
                if !s.stable() {
                    return Err(BlobError::Unstable);
                }
            }
            if !ch.gain.is_finite() {
                return Err(BlobError::Unstable);
            }
            Ok(())
        };
        for ch in p.channel.iter_mut().take(n) {
            get(&mut at, ch)?;
            if ch.peak(fs) > UNITY_TOLERANCE {
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
        if flags & FLAG_MID_SIDE != 0 {
            if n < 2 {
                return Err(BlobError::Channels);
            }
            let mut m = [0.0; 8];
            for (k, v) in m.iter_mut().enumerate() {
                *v = f64_at(at + 8 * k)?;
            }
            at += 64;
            let mut ms = MidSide {
                to: [m[0], m[1], m[2], m[3]],
                from: [m[4], m[5], m[6], m[7]],
                mid: Channel::IDENTITY,
                side: Channel::IDENTITY,
            };
            get(&mut at, &mut ms.mid)?;
            get(&mut at, &mut ms.side)?;
            if !ms.valid(fs) {
                return Err(BlobError::Unstable);
            }
            p.mid_side = Some(ms);
        }
        if flags & !(FLAG_FROM_TUNING | FLAG_DYNAMICS | FLAG_MID_SIDE) != 0 {
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

/// The running pre-stage: transposed direct form II, f64 state.
pub struct PreStage {
    params: Params,
    state: [[[f64; 2]; MAX_SECTIONS]; MAX_CHANNELS],
    dynamics: Option<DynRun>,
    /// Mid and side filter histories of the [`MidSide`] block.
    ms_state: [[[f64; 2]; MAX_SECTIONS]; 2],
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
        PreStage {
            params,
            state: [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS],
            dynamics,
            ms_state: [[[0.0; 2]; MAX_SECTIONS]; 2],
        }
    }

    pub fn params(&self) -> &Params {
        &self.params
    }

    /// Clears every filter's history and the dynamics' detectors (stream start).
    pub fn reset(&mut self) {
        self.state = [[[0.0; 2]; MAX_SECTIONS]; MAX_CHANNELS];
        self.ms_state = [[[0.0; 2]; MAX_SECTIONS]; 2];
        if let Some(d) = &mut self.dynamics {
            d.reset();
        }
    }

    /// Runs the dynamics on every channel at once (after [`PreStage::process`]
    /// has filtered each into `bufs`), then the mid/side block on channels 0
    /// and 1, in place. Neither: unchanged. Real time.
    pub fn process_dynamics(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        if let Some(d) = &mut self.dynamics {
            d.process(bufs, frames);
        }
        let Some(ms) = &self.params.mid_side else { return };
        let [l, r, ..] = bufs else { return };
        let n = frames.min(l.len()).min(r.len());
        let run = |ch: &Channel, z: &mut [[f64; 2]; MAX_SECTIONS], x: f64| -> f64 {
            let mut v = x;
            for (s, st) in ch.sections().iter().zip(z.iter_mut()) {
                let out = s.b0 * v + st[0];
                st[0] = s.b1 * v - s.a1 * out + st[1];
                st[1] = s.b2 * v - s.a2 * out;
                v = out;
            }
            v * ch.gain
        };
        let (t, f) = (ms.to, ms.from);
        for i in 0..n {
            let x = |v: f32| if v.is_finite() { v.clamp(-INPUT_BOUND, INPUT_BOUND) as f64 } else { 0.0 };
            let (a, b) = (x(l[i]), x(r[i]));
            let [zm, zs] = &mut self.ms_state;
            let m = run(&ms.mid, zm, t[0] * a + t[1] * b);
            let s = run(&ms.side, zs, t[2] * a + t[3] * b);
            let (ya, yb) = (f[0] * m + f[1] * s, f[2] * m + f[3] * s);
            if ya.is_finite() && yb.is_finite() {
                l[i] = ya as f32;
                r[i] = yb as f32;
            } else {
                self.ms_state = [[[0.0; 2]; MAX_SECTIONS]; 2];
                l[i] = 0.0;
                r[i] = 0.0;
            }
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

    fn ms_params(mid_gain: f64, side: &[Section], side_gain: f64) -> Params {
        let mut p = Params::identity(2);
        p.flags = FLAG_FROM_TUNING | FLAG_MID_SIDE;
        let mut ms = MidSide {
            to: [1.0, 1.0, 1.0, -1.0],
            from: [1.0, 1.0, 1.0, -1.0],
            mid: Channel::IDENTITY,
            side: Channel::IDENTITY,
        };
        ms.mid.gain = mid_gain;
        ms.side.count = side.len();
        ms.side.sections[..side.len()].copy_from_slice(side);
        ms.side.gain = side_gain;
        p.mid_side = Some(ms);
        p
    }

    fn shelf_cut(fs: f64) -> Section {
        // RBJ high shelf, -2 dB at 1.5 kHz, Q 1
        let a = libm::pow(10.0, -2.0 / 40.0);
        let w = 2.0 * core::f64::consts::PI * 1500.0 / fs;
        let (c, s) = (libm::cos(w), libm::sin(w));
        let sa = 2.0 * libm::sqrt(a) * s / 2.0;
        let b = [a * ((a + 1.0) + (a - 1.0) * c + sa), -2.0 * a * ((a - 1.0) + (a + 1.0) * c), a * ((a + 1.0) + (a - 1.0) * c - sa)];
        let d = [(a + 1.0) - (a - 1.0) * c + sa, 2.0 * ((a - 1.0) - (a + 1.0) * c), (a + 1.0) - (a - 1.0) * c - sa];
        Section { b0: b[0] / d[0], b1: b[1] / d[0], b2: b[2] / d[0], a1: d[1] / d[0], a2: d[2] / d[0] }
    }

    #[test]
    fn mid_side_roundtrip_and_unity_check() {
        let fs = 48_000.0;
        let side = 0.5 * libm::pow(10.0, -2.0 / 20.0);
        let p = ms_params(0.5, &[], side);
        let b = p.encode();
        assert_eq!(Params::decode(&b, 2, fs).unwrap(), p);
        // the unscaled pair at a mid gain over one half sums above unity: refused
        let loud = ms_params(0.5012, &[], side).encode();
        assert_eq!(Params::decode(&loud, 2, fs), Err(BlobError::Unstable));
        // a side filter whose phase turns against the mid's: some pair of
        // inputs sums above unity at its transition, so it is refused until
        // scaled by its peak
        let mut shelf = ms_params(0.5, &[shelf_cut(fs)], side);
        assert_eq!(Params::decode(&shelf.encode(), 2, fs), Err(BlobError::Unstable));
        let ms = shelf.mid_side.as_mut().unwrap();
        let peak = ms.peak(fs);
        assert!(peak > 1.0 && peak < 1.05, "{peak}");
        ms.mid.gain /= peak;
        ms.side.gain /= peak;
        assert_eq!(Params::decode(&shelf.encode(), 2, fs).unwrap(), shelf);
        // an in-phase pair (L = R) sees the mid path: 2 * 0.5 = unity
        let ms = p.mid_side.unwrap();
        let [g0, g1] = ms.row_gain(100.0, fs);
        assert!((g0 - 1.0).abs() < 1e-9 && (g1 - 1.0).abs() < 1e-9, "{g0} {g1}");
        // a mid/side block needs two channels
        let mut one = p.clone();
        one.channels = 1;
        assert_eq!(Params::decode(&one.encode(), 1, fs), Err(BlobError::Channels));
    }

    #[test]
    fn mid_side_keeps_mid_and_cuts_side() {
        let fs = 48_000.0;
        let side_gain = 0.5 * libm::pow(10.0, -2.0 / 20.0);
        let mut s = PreStage::with_rate(ms_params(0.5, &[], side_gain), fs);
        let x = tone(1000.0, 0.5, 4096);
        let (mut a, mut b) = (x.clone(), x.clone());
        s.process_dynamics(&mut [&mut a[..], &mut b[..]], 4096);
        // L = R: mid only, unchanged
        assert!(a.iter().zip(&x).all(|(y, v)| (y - v).abs() < 1e-6));
        // L = -R: side only, -2 dB
        s.reset();
        let (mut a, mut b): (std::vec::Vec<f32>, std::vec::Vec<f32>) = (x.clone(), x.iter().map(|v| -v).collect());
        s.process_dynamics(&mut [&mut a[..], &mut b[..]], 4096);
        let want = libm::pow(10.0, -2.0 / 20.0) as f32;
        assert!(a.iter().zip(&x).all(|(y, v)| (y - want * v).abs() < 1e-6));
        assert!(b.iter().zip(&x).all(|(y, v)| (y + want * v).abs() < 1e-6));
        // non-finite input is silence; one channel only: unchanged
        let (mut a, mut b) = (std::vec![f32::NAN; 8], std::vec![0.0f32; 8]);
        s.process_dynamics(&mut [&mut a[..], &mut b[..]], 8);
        assert!(a.iter().chain(b.iter()).all(|v| *v == 0.0));
        let mut only = x.clone();
        s.process_dynamics(&mut [&mut only[..]], 4096);
        assert_eq!(only, x);
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
