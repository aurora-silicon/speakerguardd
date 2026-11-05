//! `atsp`: audio-only two-node thermal protection (the 13-inch MacBook Pro
//! M1/M2, the MacBook Air M1 and the one-speaker desktops), from the
//! behaviour specification (SPEC-ATSP, 25G83; section 11 for the gain
//! path):
//!
//! - **Routing** (spec 4). Each in-range stream channel takes the first of
//!   slots A, B, C whose 16-bit assignment mask has its bit; a channel no
//!   mask claims is routed to A. Mask bits at or above the stream width are
//!   ignored, so wide masks (`0xffff` on a one-channel desktop) and a zero A
//!   mask are valid.
//! - **Controller tick** (spec 11). The controller is clocked by a 5 ms
//!   buffer of `N = round(fs * 5 ms)` frames (240 at 48 kHz) on a grid that
//!   starts at the first active frame and runs across host calls. Host
//!   calls longer than 512 frames are split into 512-frame fragments; in a
//!   fragment, the input windows it completes are evaluated before its
//!   audio is rendered, and the post-gain windows it completes after (the
//!   Q2/3, Q256 and Q512+ families all follow from this one order).
//! - **Thermal model** (spec 11.2), per channel: two parallel nodes, each a
//!   binary64 matched exponential per tick, `theta = fma(b, theta, (1 - b) p)`
//!   with `b = exp(-1 / (tau 200 Hz))`, toward its share of the window's
//!   Float32 power
//!   `P = (Vfs rms(x))^2 / Rdc`, `Tvc = Tamb + theta_mg + theta_vc`. Two
//!   instances run: the input-referred one on the unattenuated program and
//!   the post-gain one on the unit's own output.
//! - **Target** (spec 11.2), every tick and every channel:
//!   `g_req = sqrt(clamp(f32(((Tlim - Tamb) - theta_mg_post) / (theta_vc_input + 1)), 0, 1))`;
//!   the Float32 private target moves toward it by the attack or release
//!   coefficient `k = 1 - exp(ln(0.01) 5 ms / t)`, as one fused
//!   `g = fma(g_req - g, k, g)` (equal leaves it). When the
//!   post-gain coil is strictly above `Tlim + headroom` the target is the
//!   20 dB panic target `f32(0.1)` on that tick.
//! - **Linking.** A slot's `Gain link` replaces its channels' effective
//!   targets by their minimum, then `Gain link all audio channels` (ID 40)
//!   replaces every one by the global minimum; the private targets keep
//!   their own values. The effective target is held, bit-constant, for the
//!   next `N` frames.
//! - **Output.** `y = x * d`, held to full scale, where `d` is the output
//!   dezipper: a Float32 RBJ low pass (20 Hz, Q 0.6513333, transposed
//!   direct form II with fused multiply-adds) of the held target. Its input
//!   is zero until the first tick (the 5 ms startup mute). Its Float32 fixed
//!   point is the reference's "+0.052 dB trim" (1.0060077 at 48 kHz, rate
//!   dependent): gain state, not a stage. An `AudioUnitReset` does not
//!   restart it; silence advances it. Zero latency.
//! - **Host reset** (`AudioUnitReset`): changes nothing. The thermal models,
//!   targets, grid and dezipper carry on as if the call had not happened.
//! - **Bypass** (ID 0, at the next call): a bit-exact copy that freezes all
//!   thermal, gain, link and startup state; clearing it resumes that state.
//!   The controller grid keeps its place; the audio of the partial input
//!   and output windows is dropped (they finish on silence).
//! - **Input.** The program is held to twice full scale ([`INPUT_FS`])
//!   before it is metered or gained.
//!
//! Non-finite input gives a zero output sample and the fail-safe report, and
//! a non-finite model state latches the fail-safe until reset, so neither
//! can poison the thermal state.
//!
//! One-channel streams take the reference's one-channel dezipper path
//! (SPEC-ATSP 11.4): the same low pass, computed per call as eight-frame
//! blocks, then a four-frame group, then scalar transitions, with only the
//! transposed state crossing calls. Its Float32 rounding settles about
//! 0.7 % above the multichannel path's at 48 kHz.

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::{
    gap_or_zero, release_toward_unity, ChannelState, HandOff, Health, Io, ProtectionUnit, Telemetry, UnitKind,
    FAILSAFE_GAIN, MAX_CHANNELS,
};

/// The 25G83 dezipper's settled value at 48 kHz (its Float32 fixed point;
/// rate and path dependent).
pub const NOMINAL_TRIM: f32 = 1.006_007_7;
/// The output dezipper's low pass (rig-confirmed, bit-exact on 25G83): an
/// RBJ low pass at exactly 20 Hz, Q 0.6513333.
const DEZIPPER_HZ: f64 = 20.0;
/// The most the dezipper can put out at 48 kHz for targets in `[0, 1]`
/// ([`dezipper_peak`]): about 1.030 on the multichannel path and 1.037 on
/// the one-channel path, whose rounding settles higher. The reference does
/// not cap it; its multichannel startup step peaks at 1.02834, 1886
/// frames after the unmute.
pub const DEZIPPER_PEAK: f32 = 1.04;

/// The most the dezipper can put out for targets in `[0, 1]` at `rate`:
/// the sum of the positive part of its Float32-coefficient impulse
/// response (1.0298 at 48 kHz, 1.0472 at 96 kHz; the rounding of the
/// coefficients sets it), scaled by the Float32 filter's settled drift
/// above its linear DC gain, with a 3 % margin for the filter's own
/// rounding (its fixed point moves with the level at high rates).
/// The reference does not cap it. The engine's output stays held to full
/// scale.
pub fn dezipper_peak(rate: f64) -> f32 {
    let d = Dezipper::new(rate);
    let (b0, b1, a1, a2) = (d.b0 as f64, d.b1 as f64, d.a1 as f64, d.a2 as f64);
    let (mut x1, mut x2, mut y1, mut y2) = (0.0f64, 0.0, 0.0, 0.0);
    let mut pos = 0.0;
    // the poles' time constant is under 20 ms at every rate
    for i in 0..(rate as usize).max(4096) {
        let x = if i == 0 { 1.0 } else { 0.0 };
        let y = b0 * x + b1 * x1 + b0 * x2 - a1 * y1 - a2 * y2;
        (x2, x1, y2, y1) = (x1, x, y1, y);
        pos += y.max(0.0);
    }
    // the Float32 filter settles off the linear DC gain (its "trim");
    // scale by that drift where it is above
    let dc = (b0 + b1 + b0) / (1.0 + a1 + a2);
    let mut f = Dezipper::new(rate);
    f.settle(1.0);
    // the one-channel path settles on its own rounding
    let mut m = Dezipper::new(rate);
    let mut ones = [1.0f32; FRAGMENT];
    for _ in 0..(1usize << 18) / FRAGMENT {
        ones.fill(1.0);
        m.mono_call(&mut ones);
    }
    let drift = (f.last().max(m.last()) as f64 / dc).max(1.0);
    (pos * drift * (1.0 + 3e-2)) as f32
}
const DEZIPPER_Q: f64 = 0.651_333_3;
/// Silence after initialisation: the unfilled first target buffer, one
/// controller tick (spec 11; 240 frames at 48 kHz).
pub const STARTUP_MUTE_S: f64 = 0.005;

/// The output dezipper (rig-confirmed, bit-exact): the RBJ coefficients
/// computed in f64 and rounded once to f32 (`b1 = 2 b0`, `b2 = b0`), run as
/// a transposed direct form II with fused multiply-adds:
/// `y = fma(b0, x, s1)`, `s1 = fma(-a1, y, fma(b1, x, s2))`,
/// `s2 = fma(-a2, y, b0 x)`. Its Float32 fixed point on a unit step is the
/// reference's +0.052 dB "trim" at 48 kHz.
///
/// A one-channel stream takes the one-channel path ([`Dezipper::mono_call`])
/// for every fragment. On more channels, a one-frame fragment takes the
/// reference's one-frame path instead
/// (SPEC-ATSP 11.3), a direct form I on the last two inputs and outputs:
/// `y = fma(b0, x, fma(-a1, y1, fma(b1, x1, fma(b2, x2, -a2 y2))))`. Both
/// keep both histories, so a stream may mix the two.
#[derive(Clone, Copy)]
struct Dezipper {
    b0: f32,
    b1: f32,
    a1: f32,
    a2: f32,
    s: [f32; 2],
    /// The last two inputs and outputs (`x[n-1], x[n-2], y[n-1], y[n-2]`).
    h: [f32; 4],
    y: f32,
    /// The one-channel block path's tables (SPEC-ATSP 11.4): the expanded
    /// numerator response and the all-pole response, each formed in
    /// binary64 and rounded once.
    nh: [f32; 8],
    pg: [f32; 8],
}

impl Dezipper {
    fn new(rate: f64) -> Self {
        let w = 2.0 * core::f64::consts::PI * DEZIPPER_HZ / rate;
        let al = libm::sin(w) / (2.0 * DEZIPPER_Q);
        let a0 = 1.0 + al;
        let c = libm::cos(w);
        let b0 = ((1.0 - c) / 2.0 / a0) as f32;
        let (b1, a1, a2) = (2.0 * b0, (-2.0 * c / a0) as f32, ((1.0 - al) / a0) as f32);
        let (fb0, fb1, fa1, fa2) = (b0 as f64, b1 as f64, a1 as f64, a2 as f64);
        let (mut h, mut g) = ([0.0f64; 8], [0.0f64; 8]);
        h[0] = fb0;
        h[1] = fb1 - fa1 * h[0];
        h[2] = fb0 - fa1 * h[1] - fa2 * h[0];
        g[0] = 1.0;
        g[1] = -fa1;
        for k in 2..8 {
            if k >= 3 {
                h[k] = -fa1 * h[k - 1] - fa2 * h[k - 2];
            }
            g[k] = -fa1 * g[k - 1] - fa2 * g[k - 2];
        }
        Dezipper { b0, b1, a1, a2, s: [0.0; 2], h: [0.0; 4], y: 0.0, nh: h.map(|v| v as f32), pg: g.map(|v| v as f32) }
    }

    /// `H(i)`: the numerator response, zero out of range.
    #[inline]
    fn hh(&self, i: isize) -> f32 {
        if (0..8).contains(&i) {
            self.nh[i as usize]
        } else {
            0.0
        }
    }

    /// `G(i)`: the all-pole response, zero out of range.
    #[inline]
    fn gg(&self, i: isize) -> f32 {
        if (0..8).contains(&i) {
            self.pg[i as usize]
        } else {
            0.0
        }
    }

    /// One call's targets through the one-channel path, in place (SPEC-ATSP
    /// 11.4): complete groups of eight on the block kernel, then a group of
    /// four if four or more remain, then one scalar transition per frame
    /// left. Grouping starts at every call; only the transposed state
    /// crosses calls.
    fn mono_call(&mut self, buf: &mut [f32]) {
        let mut i = 0;
        while buf.len() - i >= 8 {
            self.mono8(&mut buf[i..i + 8]);
            i += 8;
        }
        if buf.len() - i >= 4 {
            self.mono4(&mut buf[i..i + 4]);
            i += 4;
        }
        for v in &mut buf[i..] {
            *v = self.mono1(*v);
        }
        if let Some(&y) = buf.last() {
            self.y = y;
        }
    }

    /// Output `k` (0..=3) of a group from its first four inputs and the
    /// state: one fused chain.
    #[inline]
    fn mono_lane(&self, x: &[f32], k: isize) -> f32 {
        let [s1, s2] = self.s;
        let mut z = self.hh(k) * x[0];
        z = libm::fmaf(self.hh(k - 1), x[1], z);
        z = libm::fmaf(self.hh(k - 2), x[2], z);
        z = libm::fmaf(self.hh(k - 3), x[3], z);
        z = libm::fmaf(self.gg(k), s1, z);
        libm::fmaf(self.gg(k - 1), s2, z)
    }

    /// The state a group leaves, from its last two inputs and outputs.
    #[inline]
    fn mono_state(&mut self, x6: f32, x7: f32, y6: f32, y7: f32) {
        let mut n1 = self.b0 * x6;
        n1 = libm::fmaf(self.b1, x7, n1);
        n1 = libm::fmaf(-self.a2, y6, n1);
        n1 = libm::fmaf(-self.a1, y7, n1);
        let mut n2 = 0.0f32 * x6;
        n2 = libm::fmaf(self.b0, x7, n2);
        n2 = libm::fmaf(0.0, y6, n2);
        n2 = libm::fmaf(-self.a2, y7, n2);
        self.s = [n1, n2];
    }

    /// A complete eight-frame group: lanes 0-3 one chain each, lanes 4-7
    /// a chain on the first four inputs and the first state, a chain on
    /// the second state and the last four inputs, and one rounded add.
    fn mono8(&mut self, b: &mut [f32]) {
        let x: [f32; 8] = [b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]];
        let [s1, s2] = self.s;
        let mut y = [0.0f32; 8];
        for (k, yk) in y.iter_mut().enumerate().take(4) {
            *yk = self.mono_lane(&x, k as isize);
        }
        for (k, yk) in y.iter_mut().enumerate().skip(4) {
            let k = k as isize;
            let mut lo = self.hh(k) * x[0];
            lo = libm::fmaf(self.hh(k - 1), x[1], lo);
            lo = libm::fmaf(self.hh(k - 2), x[2], lo);
            lo = libm::fmaf(self.hh(k - 3), x[3], lo);
            lo = libm::fmaf(self.gg(k), s1, lo);
            let mut hi = self.hh(k - 4) * x[4];
            hi = libm::fmaf(self.gg(k - 1), s2, hi);
            hi = libm::fmaf(self.hh(k - 5), x[5], hi);
            hi = libm::fmaf(self.hh(k - 6), x[6], hi);
            hi = libm::fmaf(self.hh(k - 7), x[7], hi);
            *yk = lo + hi;
        }
        self.mono_state(x[6], x[7], y[6], y[7]);
        b.copy_from_slice(&y);
    }

    /// A four-frame group: the lane 0-3 chains, the state from frames 2, 3.
    fn mono4(&mut self, b: &mut [f32]) {
        let x: [f32; 4] = [b[0], b[1], b[2], b[3]];
        let mut y = [0.0f32; 4];
        for (k, yk) in y.iter_mut().enumerate() {
            *yk = self.mono_lane(&x, k as isize);
        }
        self.mono_state(x[2], x[3], y[2], y[3]);
        b.copy_from_slice(&y);
    }

    /// One scalar transition (calls and remainders of one to three frames).
    #[inline]
    fn mono1(&mut self, x: f32) -> f32 {
        let [s1, s2] = self.s;
        let y = libm::fmaf(self.b0, x, s1);
        let n1 = libm::fmaf(-self.a1, y, libm::fmaf(self.b1, x, s2));
        let n2 = libm::fmaf(self.b0, x, -self.a2 * y);
        self.s = [n1, n2];
        y
    }
    #[inline]
    fn step(&mut self, x: f32) -> f32 {
        let y = libm::fmaf(self.b0, x, self.s[0]);
        self.s[0] = libm::fmaf(-self.a1, y, libm::fmaf(self.b1, x, self.s[1]));
        self.s[1] = libm::fmaf(-self.a2, y, self.b0 * x);
        self.h = [x, self.h[0], y, self.h[2]];
        self.y = y;
        y
    }
    /// One frame on the one-frame path.
    #[inline]
    fn step1(&mut self, x: f32) -> f32 {
        let [x1, x2, y1, y2] = self.h;
        let mut y = libm::fmaf(self.b0, x2, -self.a2 * y2);
        y = libm::fmaf(self.b1, x1, y);
        y = libm::fmaf(-self.a1, y1, y);
        y = libm::fmaf(self.b0, x, y);
        // the transposed state this frame leaves, as `step` would
        let s2 = libm::fmaf(-self.a2, y1, self.b0 * x1);
        self.s[0] = libm::fmaf(-self.a1, y, libm::fmaf(self.b1, x, s2));
        self.s[1] = libm::fmaf(-self.a2, y, self.b0 * x);
        self.h = [x, x1, y, y1];
        self.y = y;
        y
    }
    /// The last output.
    fn last(&self) -> f32 {
        self.y
    }
    /// Settled on a constant input (a hand-off: the stream was already
    /// running).
    fn settle(&mut self, x: f32) {
        for _ in 0..1 << 18 {
            self.step(x);
        }
    }
}
/// The panic target, 20 dB.
pub const PANIC_GAIN: f64 = 0.1;

/// AU parameter IDs (spec 2).
pub mod ids {
    pub const BYPASS: u32 = 0;
    /// The first ID of slot `t` is `1 + 13 * t`.
    pub const SLOT_STRIDE: u32 = 13;
    pub const GAIN_LINK_ALL: u32 = 40;
}

/// One speaker-type slot (A, B or C).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Slot {
    /// `Audio channel assignment`: bit c claims stream channel c.
    pub channel_mask: u32,
    pub gain_link: u32,
    pub amplifier_sensitivity: f32,
    pub rdc: f32,
    pub rth_vc: f32,
    pub tau_vc: f32,
    pub rth_mg: f32,
    pub tau_mg: f32,
    pub ambient: f32,
    pub temperature_limit: f32,
    pub hard_limit_headroom: f32,
    pub attack_ms: f32,
    pub release_ms: f32,
}

impl Slot {
    /// The eleven continuous constants in AU parameter order (spec 2).
    fn fields(&self) -> [f32; 11] {
        [
            self.amplifier_sensitivity,
            self.rdc,
            self.rth_vc,
            self.tau_vc,
            self.rth_mg,
            self.tau_mg,
            self.ambient,
            self.temperature_limit,
            self.attack_ms,
            self.release_ms,
            self.hard_limit_headroom,
        ]
    }
    fn set_fields(&mut self, f: [f32; 11]) {
        [
            self.amplifier_sensitivity,
            self.rdc,
            self.rth_vc,
            self.tau_vc,
            self.rth_mg,
            self.tau_mg,
            self.ambient,
            self.temperature_limit,
            self.attack_ms,
            self.release_ms,
            self.hard_limit_headroom,
        ] = f;
    }
}

/// The published range of each slot constant, in AU parameter order
/// (sensitivity V/FS, Rdc, Rvc, tau_vc s, Rmg, tau_mg s, ambient degC,
/// limit degC, attack ms, release ms, headroom degC): the unit's own
/// `AudioUnitParameterInfo` minimum and maximum, which it holds every
/// value to.
pub const SLOT_RANGES: [(f32, f32); 11] = [
    (1.0, 100.0),
    (1.0, 100.0),
    (1.0, 100.0),
    (1.0, 10_000.0),
    (1.0, 100.0),
    (1.0, 10_000.0),
    (0.0, 100.0),
    (50.0, 200.0),
    (10.0, 10_000.0),
    (1_000.0, 100_000.0),
    (0.0, 100.0),
];

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Params {
    pub channels: u32,
    pub sample_rate: u32,
    /// Reserved: the unit is sample-continuous (spec 6).
    pub block_frames: u32,
    pub bypass: u32,
    pub gain_link_all: u32,
    pub slot: [Slot; 3],
}

// SAFETY: repr(C) of u32/f32 fields and arrays of a repr(C) struct of
// them, no padding.
unsafe impl Pod for Params {}
// SAFETY: as above.
unsafe impl Pod for Slot {}

const _: () = assert!(core::mem::size_of::<Params>() == (5 + 3 * 13) * 4);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InitError {
    Blob(BlobError),
    Invalid(&'static str),
    Unsupported(&'static str),
}

impl Params {
    /// The slot each stream channel is routed to (spec 4).
    pub fn route(&self) -> [usize; MAX_CHANNELS] {
        let mut r = [0usize; MAX_CHANNELS];
        for (c, slot) in r.iter_mut().enumerate().take((self.channels as usize).min(MAX_CHANNELS)) {
            *slot = (0..3).find(|&s| self.slot[s].channel_mask & (1 << c) != 0).unwrap_or(0);
        }
        r
    }

    /// Set one AU parameter (spec 2) in the payload, held to the unit's
    /// published range as the reference holds it ([`SLOT_RANGES`]). False
    /// for an unknown ID or a non-finite value.
    pub fn set(&mut self, id: u32, v: f32) -> bool {
        if !v.is_finite() {
            return false;
        }
        let flag = (v != 0.0) as u32;
        match id {
            ids::BYPASS => self.bypass = flag,
            ids::GAIN_LINK_ALL => self.gain_link_all = flag,
            1..=39 => {
                let s = &mut self.slot[((id - 1) / ids::SLOT_STRIDE) as usize];
                let f = ((id - 1) % ids::SLOT_STRIDE) as usize;
                let v = match SLOT_RANGES.get(f) {
                    Some(&(lo, hi)) => v.clamp(lo, hi),
                    None => v,
                };
                match f {
                    0 => s.amplifier_sensitivity = v,
                    1 => s.rdc = v,
                    2 => s.rth_vc = v,
                    3 => s.tau_vc = v,
                    4 => s.rth_mg = v,
                    5 => s.tau_mg = v,
                    6 => s.ambient = v,
                    7 => s.temperature_limit = v,
                    8 => s.attack_ms = v,
                    9 => s.release_ms = v,
                    10 => s.hard_limit_headroom = v,
                    11 => s.gain_link = flag,
                    _ => s.channel_mask = v.clamp(0.0, 65535.0) as u32,
                }
            }
            _ => return false,
        }
        true
    }

    /// Every slot constant held to the unit's published range
    /// ([`SLOT_RANGES`]): the reference runs a value outside it at the
    /// nearest bound (25G83: a 0.3 s voice-coil time constant behaves as
    /// 1 s). A non-finite value is left for [`Params::validate`] to refuse.
    pub fn clamped(&self) -> Params {
        let mut p = *self;
        for s in p.slot.iter_mut() {
            let mut f = s.fields();
            for (v, &(lo, hi)) in f.iter_mut().zip(SLOT_RANGES.iter()) {
                if v.is_finite() {
                    *v = v.clamp(lo, hi);
                }
            }
            s.set_fields(f);
        }
        p
    }

    pub fn validate(&self) -> Result<(), InitError> {
        let n = self.channels as usize;
        if n == 0 || n > MAX_CHANNELS || self.sample_rate == 0 {
            return Err(InitError::Invalid("geometry"));
        }
        let route = self.route();
        let pos = |v: f32| v.is_finite() && v > 0.0;
        for (i, s) in self.slot.iter().enumerate() {
            if !route[..n].contains(&i) {
                continue;
            }
            let positive = [s.amplifier_sensitivity, s.rdc, s.rth_vc, s.tau_vc, s.tau_mg, s.attack_ms, s.release_ms]
                .iter()
                .all(|&v| pos(v));
            let finite = [s.ambient, s.temperature_limit, s.hard_limit_headroom].iter().all(|v| v.is_finite());
            let magnet = s.rth_mg.is_finite() && s.rth_mg >= 0.0;
            if !(positive && finite && magnet) {
                return Err(InitError::Invalid("slot constants"));
            }
        }
        Ok(())
    }

    pub fn from_blob(bytes: &[u8]) -> Result<Self, InitError> {
        let (h, payload) = crate::blob::Header::parse(bytes).map_err(InitError::Blob)?;
        if h.unit != UnitKind::Atsp as u32 {
            return Err(InitError::Invalid("unit kind"));
        }
        let p = Params::from_bytes(payload).map_err(InitError::Blob)?;
        if p.channels != h.channels || p.sample_rate != h.sample_rate {
            return Err(InitError::Invalid("header and payload disagree"));
        }
        p.validate()?;
        Ok(p)
    }
}

/// 99 %-settling smoothing coefficient for a time in ms, per sample
/// 99 %-settling smoothing coefficient for a time in ms, per sample
/// (`1 - exp(ln(0.01) / (t rate))`): the hand-off's release ageing.
pub fn settle_coeff(rate: f64, ms: f64) -> f64 {
    1.0 - libm::exp(libm::log(0.01) / (ms * 1e-3 * rate))
}

/// The controller tick, ms (SPEC-ATSP 11).
pub const TICK_MS: f32 = 5.0;

/// The controller rate, Hz: `f32(1000 / tick_ms)` (SPEC-ATSP 11).
const CONTROLLER_RATE: f32 = 1000.0 / TICK_MS;

/// The longest internal render fragment, frames: host calls longer than
/// this are split (SPEC-ATSP 11.3).
pub const FRAGMENT: usize = 512;

/// The controller tick in frames: `max(1, round(rate * 5 ms))`, 240 at
/// 48 kHz (SPEC-ATSP 11).
pub fn window_frames(rate: f64) -> usize {
    (libm::round(rate * TICK_MS as f64 / 1000.0) as usize).max(1)
}

/// The one-tick 99 %-settling coefficient for an attack or release time
/// in ms, with the Float32 rounding SPEC-ATSP 11.1 fixes:
/// `f32(1 - expf(f32(ln(0.01) / f32(f32(ms * 200) / 1000))))`.
pub fn tick_coeff(ms: f32) -> f32 {
    // f32(-4.60517025): ln(0.01) in Float32
    const LN_HUNDREDTH: f32 = -4.605_170_2;
    let den = (ms * CONTROLLER_RATE) / 1000.0;
    1.0 - libm::expf(LN_HUNDREDTH / den)
}

/// `sqrt(sum(x^2) / n)` in Float32 with the reference's reduction order
/// (SPEC-ATSP 11.2 fixes the RMS to Accelerate's `vDSP_rmsqv`, whose
/// result this reproduces bit for bit at the 240-frame window): the
/// squares are fused-multiply-added into eight four-lane accumulators
/// 32 frames at a time while 64 or more remain, the next 16 into the first
/// four, which then take the last four; 16 at a time into those four while
/// more than 16 remain; the four fold pairwise into one, which takes four
/// frames at a time; its lanes add as `(l0 + l1) + (l2 + l3)`, and any last
/// frames fuse into that sum one by one.
#[allow(clippy::needless_range_loop)] // the lanes and accumulators by index, as specified
pub fn rms(x: &[f32]) -> f32 {
    let n = x.len();
    if n == 0 {
        return 0.0;
    }
    let mut a = [[0.0f32; 4]; 8];
    let sq = |acc: &mut f32, v: f32| *acc = libm::fmaf(v, v, *acc);
    let mut i = 0;
    while n - i >= 64 {
        for (v, acc) in a.iter_mut().enumerate() {
            for (l, s) in acc.iter_mut().enumerate() {
                sq(s, x[i + 4 * v + l]);
            }
        }
        i += 32;
    }
    if n - i >= 16 {
        for (v, acc) in a.iter_mut().take(4).enumerate() {
            for (l, s) in acc.iter_mut().enumerate() {
                sq(s, x[i + 4 * v + l]);
            }
        }
        i += 16;
    }
    for v in 0..4 {
        for l in 0..4 {
            a[v][l] += a[v + 4][l];
        }
    }
    while n - i > 16 {
        for (v, acc) in a.iter_mut().take(4).enumerate() {
            for (l, s) in acc.iter_mut().enumerate() {
                sq(s, x[i + 4 * v + l]);
            }
        }
        i += 16;
    }
    for v in 0..2 {
        for l in 0..4 {
            a[v][l] += a[v + 2][l];
        }
    }
    for l in 0..4 {
        a[0][l] += a[1][l];
    }
    while n - i >= 4 {
        for l in 0..4 {
            sq(&mut a[0][l], x[i + l]);
        }
        i += 4;
    }
    let mut s = (a[0][0] + a[0][1]) + (a[0][2] + a[0][3]);
    for &v in &x[i..] {
        sq(&mut s, v);
    }
    libm::sqrtf(s / n as f32)
}

/// One two-node model (SPEC-ATSP 11.2): the coil's rise over the magnet
/// and the magnet's over ambient, each a binary64 matched exponential per
/// tick toward its share of the window's power. The decay and the drive
/// fuse (`fma(b, theta, a p)`): the reference's rounding, which the rig
/// distinguishes from a separately rounded sum.
#[derive(Clone, Copy, Default)]
struct Nodes {
    vc: f64,
    mg: f64,
}

impl Nodes {
    /// One completed window of Float32 power `p`.
    #[inline]
    fn step(&mut self, k: &Coeffs, p: f32) {
        let pvc = p * k.rvc;
        let pmg = p * k.rmg;
        self.vc = libm::fma(k.bvc, self.vc, k.avc * pvc as f64);
        self.mg = libm::fma(k.bmg, self.mg, k.amg * pmg as f64);
    }
    #[inline]
    fn finite(&self) -> bool {
        self.vc.is_finite() && self.mg.is_finite()
    }
}

/// Per channel constants, from the routed slot (SPEC-ATSP 11.1).
#[derive(Clone, Copy)]
struct Coeffs {
    slot: usize,
    vfs: f32,
    /// `f32(1 / Rdc)`.
    inv_rdc: f32,
    rvc: f32,
    rmg: f32,
    ambient: f32,
    /// `f64(f32(Tlim - Tamb))`: the controller's numerator base.
    span: f64,
    /// `f32(Tlim + headroom)`: the panic threshold.
    hard: f32,
    bvc: f64,
    avc: f64,
    bmg: f64,
    amg: f64,
    ka: f32,
    kr: f32,
    tau_vc: f64,
    tau_mg: f64,
    /// Per-sample release coefficient (the hand-off's gap ageing).
    kr_sample: f64,
}

impl Coeffs {
    fn new(p: &Params, slot: usize, rate: f64) -> Coeffs {
        let s = &p.slot[slot];
        let hz = CONTROLLER_RATE as f64;
        let bvc = libm::exp(-1.0 / (s.tau_vc as f64 * hz));
        let bmg = libm::exp(-1.0 / (s.tau_mg as f64 * hz));
        Coeffs {
            slot,
            vfs: s.amplifier_sensitivity,
            inv_rdc: 1.0 / s.rdc,
            rvc: s.rth_vc,
            rmg: s.rth_mg,
            ambient: s.ambient,
            span: (s.temperature_limit - s.ambient) as f64,
            hard: s.temperature_limit + s.hard_limit_headroom,
            bvc,
            avc: 1.0 - bvc,
            bmg,
            amg: 1.0 - bmg,
            ka: tick_coeff(s.attack_ms),
            kr: tick_coeff(s.release_ms),
            tau_vc: s.tau_vc as f64,
            tau_mg: s.tau_mg as f64,
            kr_sample: settle_coeff(rate, s.release_ms as f64),
        }
    }

    /// One window's Float32 power (SPEC-ATSP 11.2):
    /// `f32(f32(volts^2) * f32(1 / Rdc))`, `volts = f32(rms * Vfs)`.
    #[inline]
    fn power(&self, window: &[f32]) -> f32 {
        let volts = rms(window) * self.vfs;
        (volts * volts) * self.inv_rdc
    }

    /// `Tamb + theta_mg + theta_vc` of a node pair, in Float32.
    #[inline]
    fn temp(&self, n: &Nodes) -> f32 {
        (self.ambient as f64 + n.mg + n.vc) as f32
    }
}

struct Chan {
    k: Coeffs,
    /// The current input window (the program as the meters see it).
    win_in: Vec<f32>,
    /// The current output window (the post-gain model's input).
    win_out: Vec<f32>,
    /// The input-referred model (the unattenuated program).
    input: Nodes,
    /// The post-gain model (the unit's own output).
    post: Nodes,
    /// The last window's post-gain power, W (telemetry).
    p_post: f32,
    /// The last tick's request.
    request: f32,
    /// The private target (SPEC-ATSP 11.2 `g_private`).
    target: f32,
    /// The linked effective target: the dezipper's input while scheduled.
    linked: f32,
}

impl Chan {
    fn new(k: Coeffs, win: usize) -> Chan {
        Chan {
            k,
            win_in: vec![0.0; win],
            win_out: vec![0.0; win],
            input: Nodes::default(),
            post: Nodes::default(),
            p_post: 0.0,
            request: 1.0,
            target: 1.0,
            linked: 1.0,
        }
    }

    fn reset(&mut self) {
        self.input = Nodes::default();
        self.post = Nodes::default();
        self.p_post = 0.0;
        self.request = 1.0;
        self.target = 1.0;
        self.linked = 1.0;
    }

    /// One completed input window (SPEC-ATSP 11.2): the input-referred
    /// model steps, then the private target, from the post-gain model as it
    /// stands.
    fn tick(&mut self) {
        let k = self.k;
        let p = k.power(&self.win_in);
        self.input.step(&k, p);
        let ratio = ((k.span - self.post.mg) / (self.input.vc + 1.0)) as f32;
        // a NaN state requests the floor (and latches the fail-safe)
        let req = if ratio.is_nan() { 0.0 } else { libm::sqrtf(ratio.clamp(0.0, 1.0)) };
        self.request = req;
        let mut g = self.target;
        if req > g {
            g = libm::fmaf(req - g, k.kr, g);
        } else if g > req {
            g = libm::fmaf(req - g, k.ka, g);
        }
        // strict: equality does not panic; a NaN coil does
        let t_post = k.temp(&self.post);
        if t_post > k.hard || t_post.is_nan() {
            g = PANIC_GAIN as f32;
        }
        self.target = g;
    }

    /// One completed output window: the post-gain model steps.
    fn post_window(&mut self) {
        let k = self.k;
        let p = k.power(&self.win_out);
        self.p_post = p;
        self.post.step(&k, p);
    }

    fn finite(&self) -> bool {
        self.input.finite() && self.post.finite() && self.target.is_finite()
    }
}

struct Run {
    n: usize,
    /// The controller tick, frames.
    win: usize,
    /// Frames into the current input window (the grid runs on active
    /// frames).
    pos: usize,
    /// Frames into the current output window.
    out_pos: usize,
    /// Frames of the linked targets still scheduled; none before the
    /// first tick (the startup mute).
    pending: usize,
    link_slot: [bool; 3],
    link_all: bool,
    ch: Vec<Chan>,
    /// Per channel: the output dezipper.
    dez: Vec<Dezipper>,
    /// Per channel, one fragment of the dezipper's input.
    sched: Vec<f32>,
    dez_now: f32,
    frame: u64,
    /// A model state went non-finite: the fail-safe is latched until reset.
    fault: bool,
}

pub struct Unit {
    params: Params,
    bypass: bool,
    run: Option<Run>,
}

impl Unit {
    pub fn new(params: Params) -> Self {
        let run = Run::new(&params).ok();
        Unit { params, bypass: params.bypass != 0, run }
    }
    pub fn init_error(params: &Params) -> Option<InitError> {
        Run::new(params).err()
    }
    pub fn params(&self) -> &Params {
        &self.params
    }
    pub fn is_failsafe(&self) -> bool {
        self.run.is_none()
    }
    pub fn bypassed(&self) -> bool {
        self.bypass
    }
    /// The first channel's dezipper output at the last rendered frame.
    pub fn dezipper_gain(&self) -> f32 {
        self.run.as_ref().map_or(0.0, |r| r.dez_now)
    }
    /// The most this unit's gain can be ([`dezipper_peak`] at its rate).
    pub fn gain_peak(&self) -> f32 {
        dezipper_peak(self.params.sample_rate as f64)
    }
    /// The controller tick, frames.
    pub fn window_frames(&self) -> usize {
        self.run.as_ref().map_or(0, |r| r.win)
    }
}

/// The unit holds its input to twice full scale before it meters or gains
/// it (rig: a -2.08 FS sample of a pink step leaves at -2 FS times the
/// gain).
pub const INPUT_FS: f32 = 2.0;

/// The highest sample rate the unit runs (its 5 ms tick is then 1920
/// frames per channel window).
pub const MAX_RATE: u32 = 384_000;

impl Run {
    fn new(p: &Params) -> Result<Run, InitError> {
        p.validate()?;
        if p.sample_rate > MAX_RATE {
            return Err(InitError::Unsupported("sample rate"));
        }
        let p = &p.clamped();
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let win = window_frames(rate);
        let route = p.route();
        let ch = route[..n].iter().map(|&s| Chan::new(Coeffs::new(p, s, rate), win)).collect();
        Ok(Run {
            n,
            win,
            pos: 0,
            out_pos: 0,
            pending: 0,
            link_slot: [p.slot[0].gain_link != 0, p.slot[1].gain_link != 0, p.slot[2].gain_link != 0],
            link_all: p.gain_link_all != 0,
            ch,
            dez: vec![Dezipper::new(rate); n],
            sched: vec![0.0; n * FRAGMENT],
            dez_now: 0.0,
            frame: 0,
            fault: false,
        })
    }

    /// New constants from changed parameters, keeping every state
    /// (an AU parameter change at run time).
    fn retune(&mut self, p: &Params) {
        let p = &p.clamped();
        let rate = p.sample_rate as f64;
        let route = p.route();
        for (c, ch) in self.ch.iter_mut().enumerate() {
            ch.k = Coeffs::new(p, route[c], rate);
        }
        self.link_slot = [p.slot[0].gain_link != 0, p.slot[1].gain_link != 0, p.slot[2].gain_link != 0];
        self.link_all = p.gain_link_all != 0;
    }

    /// Models, targets and windows restart; the dezipper does not (rig
    /// law 5), and it holds unity until the first tick.
    fn reset(&mut self) {
        self.ch.iter_mut().for_each(Chan::reset);
        self.pos = 0;
        self.out_pos = 0;
        self.pending = self.win;
        self.frame = 0;
        self.fault = false;
    }

    /// Every channel's input window is complete: update the models and
    /// targets, link, and schedule the linked targets for one tick.
    fn tick(&mut self) {
        for ch in self.ch.iter_mut() {
            ch.tick();
        }
        self.link();
        self.pending = self.win;
    }

    /// The linked targets from the private targets.
    fn link(&mut self) {
        let n = self.n;
        let mut lk = [1.0f32; MAX_CHANNELS];
        for (c, ch) in self.ch.iter().enumerate() {
            lk[c] = ch.target;
        }
        for s in 0..3 {
            if self.link_slot[s] {
                let m = self.ch.iter().filter(|c| c.k.slot == s).fold(1.0f32, |m, c| nan_min(m, c.target));
                for (c, ch) in self.ch.iter().enumerate() {
                    if ch.k.slot == s {
                        lk[c] = m;
                    }
                }
            }
        }
        if self.link_all {
            let m = lk[..n].iter().fold(1.0f32, |m, &r| nan_min(m, r));
            lk[..n].iter_mut().for_each(|r| *r = m);
        }
        for (c, ch) in self.ch.iter_mut().enumerate() {
            ch.linked = if lk[c].is_finite() { lk[c] } else { 0.0 };
        }
        if !self.ch.iter().all(Chan::finite) {
            self.fault = true;
        }
    }

    /// Schedule `m` frames of the held targets from `at` in the fragment
    /// (as many as are still scheduled).
    #[inline]
    fn schedule(&mut self, at: usize, m: usize) {
        let z = self.pending.min(m);
        for (c, ch) in self.ch.iter().enumerate() {
            self.sched[c * FRAGMENT + at..c * FRAGMENT + at + z].fill(ch.linked);
        }
        self.pending -= z;
    }

    /// One internal fragment of `m <= FRAGMENT` frames from `f` (SPEC-ATSP
    /// 11.3). Returns whether any input sample was non-finite.
    fn fragment(&mut self, io: &mut Io<'_, '_>, f: usize, m: usize) -> bool {
        let win = self.win;
        let mut failed = false;
        for c in 0..self.n {
            self.sched[c * FRAGMENT..c * FRAGMENT + m].fill(0.0);
        }
        // 1. what the last tick scheduled plays first
        self.schedule(0, m);
        // 2. every input window this fragment completes ticks now
        let mut done = 0;
        while done < m {
            let k = (win - self.pos).min(m - done);
            for (c, ch) in self.ch.iter_mut().enumerate() {
                let x = &io.input[c][f + done..f + done + k];
                for (w, &v) in ch.win_in[self.pos..self.pos + k].iter_mut().zip(x) {
                    failed |= !v.is_finite();
                    *w = held(v);
                }
            }
            self.pos += k;
            done += k;
            if self.pos == win {
                self.pos = 0;
                self.tick();
                self.schedule(done, m - done);
            }
        }
        // 3. the scheduled targets through the dezipper; 4. the output
        // windows
        for c in 0..self.n {
            let ch = &mut self.ch[c];
            let d = &mut self.dez[c];
            let sched = &mut self.sched[c * FRAGMENT..c * FRAGMENT + m];
            // the targets become the gains in place
            if self.n == 1 {
                d.mono_call(sched);
            } else {
                for t in sched.iter_mut() {
                    *t = if m == 1 { d.step1(*t) } else { d.step(*t) };
                }
            }
            let mut op = self.out_pos;
            for (j, &g) in sched.iter().enumerate() {
                let y = held(io.input[c][f + j]) * g;
                io.output[c][f + j] = crate::dsp::bound1(y);
                ch.win_out[op] = crate::dsp::meter1(y);
                op += 1;
                if op == win {
                    op = 0;
                    ch.post_window();
                }
            }
        }
        self.out_pos = (self.out_pos + m) % win;
        if let Some(d) = self.dez.first() {
            self.dez_now = d.last();
        }
        if !self.ch.iter().all(Chan::finite) {
            self.fault = true;
        }
        failed
    }

    /// Returns whether any input sample was non-finite (the meter failure:
    /// those frames are silent, and the call reports the fail-safe).
    fn process(&mut self, io: &mut Io<'_, '_>) -> bool {
        let frames = io.frames;
        let mut failed = false;
        let mut f = 0;
        while f < frames {
            let m = FRAGMENT.min(frames - f);
            failed |= self.fragment(io, f, m);
            f += m;
        }
        self.frame += frames as u64;
        if self.fault {
            crate::failsafe::fill_from(io, 0, FAILSAFE_GAIN);
        }
        failed
    }
}

/// The program as the unit takes it: held to [`INPUT_FS`], a non-finite
/// sample 0. Both the meters and the gain see the held sample.
#[inline]
fn held(x: f32) -> f32 {
    if x.is_finite() {
        x.clamp(-INPUT_FS, INPUT_FS)
    } else {
        0.0
    }
}

/// `min` that keeps a NaN (a NaN target must never read as unity).
#[inline]
fn nan_min(a: f32, b: f32) -> f32 {
    if a.is_nan() || b.is_nan() {
        f32::NAN
    } else {
        a.min(b)
    }
}

impl ProtectionUnit for Unit {
    fn kind(&self) -> UnitKind {
        UnitKind::Atsp
    }
    fn latency_frames(&self) -> u32 {
        0
    }
    fn process(&mut self, mut io: Io<'_, '_>) -> Health {
        let frames = io.frames;
        let Some(run) = self.run.as_mut() else {
            crate::spp3::failsafe_io(&mut io);
            return Health(Health::FAILSAFE | Health::NO_PARAMS);
        };
        let fits = |b: &[f32]| b.len() >= frames;
        if io.input.len() != run.n
            || io.output.len() != run.n
            || !io.input.iter().all(|b| fits(b))
            || !io.output.iter().all(|b| fits(b))
        {
            crate::spp3::failsafe_io(&mut io);
            return Health(Health::FAILSAFE | Health::CHANNEL_MISMATCH);
        }
        if run.fault {
            crate::spp3::failsafe_io(&mut io);
            return Health(Health::FAILSAFE | Health::MODEL_FAULT);
        }
        if self.bypass {
            // a bit-exact copy of in-range program; held to full scale (a
            // non-finite sample 0) like every engine output
            for c in 0..run.n {
                for (o, &x) in io.output[c][..frames].iter_mut().zip(&io.input[c][..frames]) {
                    *o = crate::dsp::bound1(x);
                }
            }
            run.frame += frames as u64;
            // the partial windows' audio is not kept across a bypass (rig:
            // a run/bypass/resume is bit-exact only so); their grid is
            for ch in run.ch.iter_mut() {
                ch.win_in.fill(0.0);
                ch.win_out.fill(0.0);
            }
            return Health::OK;
        }
        let failed = run.process(&mut io);
        if run.fault {
            return Health(Health::FAILSAFE | Health::MODEL_FAULT);
        }
        Health(if failed { Health::FAILSAFE } else { 0 })
    }
    fn telemetry(&self, out: &mut Telemetry) {
        out.clear();
        let Some(run) = self.run.as_ref() else {
            out.health = Health(Health::FAILSAFE | Health::NO_PARAMS);
            for ch in out.channels.iter_mut() {
                ch.gain = FAILSAFE_GAIN;
            }
            return;
        };
        out.frame = run.frame;
        if run.fault {
            out.health = Health(Health::FAILSAFE | Health::MODEL_FAULT);
        }
        for (i, c) in run.ch.iter().enumerate() {
            let mut s = ChannelState::ABSENT;
            s.modeled_power_w = c.p_post;
            s.coil_temp_c = c.k.temp(&c.post);
            s.magnet_temp_c = (c.k.ambient as f64 + c.post.mg) as f32;
            s.ol_temp_c = s.coil_temp_c;
            s.control_temp_c = c.k.temp(&c.input);
            s.axis_gain[4] = c.request;
            s.gain = if run.fault {
                FAILSAFE_GAIN
            } else if self.bypass {
                1.0
            } else {
                c.linked
            };
            out.channels[i] = s;
        }
    }
    fn reset(&mut self) {
        if let Some(r) = self.run.as_mut() {
            r.reset();
        }
    }
    /// `AudioUnitReset` leaves every processing state as it is: the thermal
    /// models, the targets, the controller grid, the startup and output
    /// dezipper, bypass and a latched fault. The reference's audio is
    /// bit-identical with and without the call, in limiting, in the startup
    /// mute, bypassed, at one-frame host slices and for one-, two- and
    /// four-channel presets. A stream start ([`ProtectionUnit::reset`]) is
    /// not this call.
    fn host_reset(&mut self) {}
    fn save_state(&self, out: &mut HandOff) -> bool {
        let Some(run) = self.run.as_ref().filter(|r| !r.fault) else { return false };
        *out = HandOff::EMPTY;
        out.kind = UnitKind::Atsp as u32;
        out.channels = run.n as u32;
        out.sample_rate = self.params.sample_rate;
        for (i, c) in run.ch.iter().enumerate() {
            out.node[i][..4].copy_from_slice(&[c.input.vc, c.input.mg, c.post.vc, c.post.mg]);
            out.gain[i] = c.target.clamp(f32::MIN_POSITIVE, 1.0) as f64;
        }
        true
    }

    /// Both predictors' nodes cooled for the gap (each series node decays
    /// with its own constant), the target released toward unity. The
    /// reference resets on reinitialisation (spec 5); a speaker does not
    /// cool because a stream restarted, so here it never does. The startup
    /// dezipper is not repeated after a hand-off.
    fn restore_state(&mut self, state: &HandOff, gap_s: f64) -> bool {
        let rate = self.params.sample_rate;
        let Some(run) = self.run.as_mut() else { return false };
        if !state.fits(UnitKind::Atsp, run.n, rate) {
            return false;
        }
        let t = gap_or_zero(gap_s);
        for (i, c) in run.ch.iter_mut().enumerate() {
            let ev = libm::exp(-t / c.k.tau_vc);
            let em = libm::exp(-t / c.k.tau_mg);
            let s = &state.node[i];
            c.input = Nodes { vc: s[0] * ev, mg: s[1] * em };
            c.post = Nodes { vc: s[2] * ev, mg: s[3] * em };
            c.target = release_toward_unity(state.gain[i], 1.0 - c.k.kr_sample, t * rate as f64) as f32;
            c.request = c.target;
        }
        // the linked targets play from the first frame: no startup mute
        run.link();
        run.pending = run.win - run.pos;
        for (c, d) in run.dez.iter_mut().enumerate() {
            d.settle(run.ch[c].linked);
        }
        run.dez_now = run.dez.first().map_or(0.0, Dezipper::last);
        true
    }

    fn set_parameter(&mut self, id: u32, value: f32) -> bool {
        if id == ids::BYPASS {
            self.bypass = value != 0.0;
            self.params.bypass = self.bypass as u32;
            return true;
        }
        let mut p = self.params;
        if !p.set(id, value) || p.validate().is_err() {
            return false;
        }
        self.params = p;
        if let Some(r) = self.run.as_mut() {
            r.retune(&p);
        }
        true
    }
}

#[cfg(test)]
mod mono_tests {
    //! The one-channel dezipper path against SPEC-ATSP 11.4's call rules.
    extern crate std;
    use super::Dezipper;
    use alloc::vec::Vec;
    use proptest::prelude::*;

    /// Run `x` through a fresh one-channel dezipper as calls of `sizes`
    /// (cycled), returning the output bits.
    fn run(rate: f64, x: &[f32], sizes: &[usize]) -> Vec<u32> {
        let mut d = Dezipper::new(rate);
        let mut buf = x.to_vec();
        let (mut i, mut k) = (0, 0);
        while i < buf.len() {
            let m = sizes[k % sizes.len()].min(buf.len() - i);
            d.mono_call(&mut buf[i..i + m]);
            i += m;
            k += 1;
        }
        buf.iter().map(|v| v.to_bits()).collect()
    }

    /// An independent reading of the law, written as the spec states it:
    /// one call's frames as groups of eight, then four, then one.
    fn reference(rate: f64, x: &[f32], sizes: &[usize]) -> Vec<u32> {
        let d = Dezipper::new(rate);
        let (b0, b1, a1, a2) = (d.b0, d.b1, d.a1, d.a2);
        let hh = |i: isize| if (0..8).contains(&i) { d.nh[i as usize] } else { 0.0 };
        let gg = |i: isize| if (0..8).contains(&i) { d.pg[i as usize] } else { 0.0 };
        let fma = libm::fmaf;
        let (mut s1, mut s2) = (0.0f32, 0.0f32);
        let mut out = Vec::new();
        let (mut i, mut k) = (0, 0);
        while i < x.len() {
            let m = sizes[k % sizes.len()].min(x.len() - i);
            let call = &x[i..i + m];
            let mut j = 0;
            while j < m {
                let group = if m - j >= 8 {
                    8
                } else if m - j >= 4 {
                    4
                } else {
                    1
                };
                let xs = &call[j..j + group];
                let mut y = [0.0f32; 8];
                if group == 1 {
                    let yy = fma(b0, xs[0], s1);
                    let n1 = fma(-a1, yy, fma(b1, xs[0], s2));
                    let n2 = fma(b0, xs[0], -a2 * yy);
                    (s1, s2) = (n1, n2);
                    out.push(yy.to_bits());
                    j += 1;
                    continue;
                }
                for (kk, yk) in y.iter_mut().enumerate().take(group) {
                    let kk = kk as isize;
                    let mut lo = hh(kk) * xs[0];
                    for t in 1..4 {
                        lo = fma(hh(kk - t), xs[t as usize], lo);
                    }
                    lo = fma(gg(kk), s1, lo);
                    *yk = if kk < 4 {
                        fma(gg(kk - 1), s2, lo)
                    } else {
                        let mut hi = hh(kk - 4) * xs[4];
                        hi = fma(gg(kk - 1), s2, hi);
                        for t in 5..8 {
                            hi = fma(hh(kk - t), xs[t as usize], hi);
                        }
                        lo + hi
                    };
                }
                let (p, q) = (group - 2, group - 1);
                let n1 = fma(-a1, y[q], fma(-a2, y[p], fma(b1, xs[q], b0 * xs[p])));
                let n2 = fma(-a2, y[q], fma(0.0, y[p], fma(b0, xs[q], 0.0 * xs[p])));
                (s1, s2) = (n1, n2);
                out.extend(y[..group].iter().map(|v| v.to_bits()));
                j += group;
            }
            i += m;
            k += 1;
        }
        out
    }

    fn program(seed: u64, len: usize) -> Vec<f32> {
        // piecewise-constant targets in [0, 1], as the controller holds them,
        // with some per-frame noise to reach every lane's arithmetic
        let mut s = seed | 1;
        let mut next = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 40) as f32 / (1u64 << 24) as f32
        };
        let mut v = Vec::with_capacity(len);
        let mut level = 0.0;
        for i in 0..len {
            if i % 240 == 0 {
                level = next();
            }
            v.push(if i % 3 == 0 { next() } else { level });
        }
        v
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(256))]

        /// Any partition into calls follows the 8 / 4 / scalar call law.
        #[test]
        fn any_call_partition_follows_the_law(
            seed in any::<u64>(), len in 1usize..3000,
            sizes in proptest::collection::vec(1usize..600, 1..12),
            rate in prop_oneof![Just(48_000.0f64), Just(44_100.0), Just(96_000.0)],
        ) {
            let x = program(seed, len);
            prop_assert_eq!(run(rate, &x, &sizes), reference(rate, &x, &sizes));
        }

        /// A call of `m` frames is calls of eight, then one call of the rest:
        /// grouping restarts at every call and only the state crosses it.
        #[test]
        fn a_call_is_its_eights_then_its_rest(seed in any::<u64>(), m in 1usize..=512, reps in 1usize..6) {
            let x = program(seed, m * reps);
            let mut split = alloc::vec![8usize; m / 8];
            if m % 8 != 0 {
                split.push(m % 8);
            }
            prop_assert_eq!(run(48_000.0, &x, &[m]), run(48_000.0, &x, &split));
        }

        /// The residual: a call of five to seven frames is a four-frame call
        /// and one-frame calls; one to three frames are one-frame calls.
        #[test]
        fn short_calls_are_a_group_of_four_and_scalars(seed in any::<u64>(), m in 1usize..=7, reps in 1usize..200) {
            let x = program(seed, m * reps);
            let split: Vec<usize> = if m >= 4 { core::iter::once(4).chain(core::iter::repeat_n(1, m - 4)).collect() } else { alloc::vec![1; m] };
            prop_assert_eq!(run(48_000.0, &x, &[m]), run(48_000.0, &x, &split));
        }

        /// Calls of whole groups of eight agree with one another.
        #[test]
        fn multiples_of_eight_agree(seed in any::<u64>(), len in 1usize..4000, a in 1usize..64, b in 1usize..64) {
            let x = program(seed, len);
            prop_assert_eq!(run(48_000.0, &x, &[8 * a]), run(48_000.0, &x, &[8 * b]));
        }
    }

    /// It is the same low pass as the multichannel path: a step settles to
    /// within 1 % of the filter's gain, as the transposed form does.
    #[test]
    fn same_filter_as_the_multichannel_path() {
        let x = alloc::vec![0.5f32; 48_000];
        let mut d = Dezipper::new(48_000.0);
        let mut buf = x.clone();
        for c in buf.chunks_mut(512) {
            d.mono_call(c);
        }
        let mut t = Dezipper::new(48_000.0);
        let want: Vec<f32> = x.iter().map(|&v| t.step(v)).collect();
        for (k, (a, b)) in buf.iter().zip(&want).enumerate().skip(4800) {
            assert!((a - b).abs() < 0.01 * 0.5, "{k}: {a} vs {b}");
        }
    }
}
