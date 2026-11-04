//! `atsp`: audio-only two-node thermal protection (13" MacBook Pro M1/M2,
//! MacBook Air M1 and the desktop/one-channel presets), from the behaviour
//! specification (SPEC-ATSP, 25G83):
//!
//! - **Routing** (spec 4). Each in-range stream channel takes the first of
//!   slots A, B, C whose 16-bit assignment mask has its bit; a channel no
//!   mask claims is routed to A. Mask bits at or above the channel count are
//!   ignored, so wide masks (`0xffff`) and a zero A mask are valid.
//! - **Thermal model** (spec 3), per channel and per sample, on the signal
//!   after the channel's controller gain and before the final trim:
//!   `p = (Vfs g x)^2 / Rdc`, backward Euler with `a = dt / (tau + dt)`.
//!   The two nodes are in series: `theta_vc` is the coil's rise over the
//!   magnet (target `p Rvc`), `theta_mg` the magnet's over ambient (target
//!   `p Rmg`), and `Tvc = Tamb + theta_mg + theta_vc`.
//! - **Controller** (spec 3). No forecasting: the request is unity until the
//!   modelled coil is strictly above `Tlim`, and returns to unity once it is
//!   at or below it. While above,
//!   `g_req = min(1, sqrt(P_allow / P_drive))`,
//!   `P_allow = max(0, (Tlim - Tamb - theta_mg) / Rvc)`, `P_drive` the drive
//!   power estimate of the unattenuated input. Strictly above
//!   `Tlim + headroom` the request is also capped at the 20 dB panic target
//!   0.1.
//! - **Linking.** A slot's `Gain link` replaces the requests of its channels
//!   by their minimum; then `Gain link all audio channels` (ID 40) replaces
//!   every request by the global minimum. Each channel smooths the result.
//! - **Smoothing.** Toward the request with the slot's attack (down) or
//!   release (up) time, as 99 %-settling times:
//!   `a = 1 - exp(ln(0.01) / (t * rate))`, per sample.
//! - **Output.** `y = clamp(x * g * d, -1, 1)`, where `d` is the output dezipper
//!   (rig-confirmed, bit-exact): zero for the first 5 ms after
//!   initialisation, then a Float32 RBJ low pass (20 Hz, Q 0.6513333,
//!   transposed direct form II with fused multiply-adds) of a unit step. Its Float32 fixed point is the reference's "+0.052 dB
//!   trim" (1.0060077 at 48 kHz, rate dependent): gain state, not a stage.
//!   An `AudioUnitReset` does not restart it; silence advances it. Its
//!   trajectory passes as the reference's does, bounded at its own peak
//!   ([`DEZIPPER_PEAK`]); the output is held to full scale. Zero latency.
//! - **Bypass** (ID 0, at the next call): a bit-exact copy that freezes all
//!   thermal, gain, link and startup state; clearing it resumes that state.
//!
//! Non-finite input contributes zero power and gives a zero output sample,
//! so it can never poison the thermal state. The output is the program held
//! to full scale times the gain; both meters see it unclamped as the
//! reference does, bounded at `dsp::METER_FS` (16 FS).
//!
//! The payload's `block_frames` sets only when a panic may release (below);
//! otherwise the unit is
//! sample-continuous for every host slice of two frames or more (spec 6).
//!
//! TODO(SPEC):
//! - **Thermal recurrence.** The specification gives the coupled form
//!   `theta_vc += a_vc (theta_mg_old + p Rvc - theta_vc)`, `Tvc = Tamb +
//!   theta_vc`. The 25G83 entry times contradict it: on the 1 kHz ramp the
//!   first attenuated sample is at 0.788 s (AID4) and 0.875 s (AID3); the
//!   series form above crosses `Tlim` at 0.789 s and 0.872 s, the coupled
//!   form at 0.823 s and 0.917 s. Both have the same steady state.
//! - **Drive estimator.** The specification names `P_drive` as the mean of
//!   `(Vfs x)^2 / Rdc` "in the controller power estimator" but not its
//!   window. Bit-identical output for host slices 2..4096 rules out a
//!   per-call mean. Here it is a one-pole mean of the instantaneous drive
//!   power with the slot's attack coefficient.
//! - **Smoothing order.** The 25G83 gain leaves unity quadratically (1e-5
//!   below unity 2 ms after entry, against 1.6e-2 for the specified
//!   one-pole toward a request near 0.53), holds about 0.5 s before
//!   releasing and recovers along an S-curve: not the first-order
//!   99 %-settling law the specification states and this unit implements.
//! - **Panic hold.** The rig (law 4) has the 0.1 target held about
//!   5000-5750 samples before release; here it holds to the next
//!   controller block boundary once the coil is back at the hard limit.
//! - **Q = 1.** macOS takes a distinct path for one-frame host calls (the
//!   tone-burst gain is 1.0019 to 1.0027 there); it is not specified, and
//!   this unit renders Q = 1 like any other slice.

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::{
    gap_or_zero, release_toward_unity, ChannelState, HandOff, Health, Io, ProtectionUnit, Telemetry, UnitKind,
    FAILSAFE_GAIN, MAX_CHANNELS,
};

const EPS: f64 = f32::MIN_POSITIVE as f64;
/// The 25G83 dezipper's settled value at 48 kHz (its Float32 fixed point;
/// rate and path dependent). The engine caps the applied gain at unity.
pub const NOMINAL_TRIM: f32 = 1.006_007_7;
/// The output dezipper's low pass (rig-confirmed, bit-exact on 25G83): an
/// RBJ low pass at exactly 20 Hz, Q 0.6513333.
const DEZIPPER_HZ: f64 = 20.0;
/// The dezipper's largest value on its startup step (rig: 1.02834 at
/// 48 kHz, 1886 samples after unmute): the most the engine's atsp gain can
/// be above unity.
pub const DEZIPPER_PEAK: f32 = 1.028_35;
const DEZIPPER_Q: f64 = 0.651_333_3;
/// Silence after initialisation (spec 5; 240 frames at 48 kHz).
pub const STARTUP_MUTE_S: f64 = 0.005;

/// The output dezipper (rig-confirmed, bit-exact): the RBJ coefficients
/// computed in f64 and rounded once to f32 (`b1 = 2 b0`, `b2 = b0`), run as
/// a transposed direct form II with fused multiply-adds:
/// `y = fma(b0, x, s1)`, `s1 = fma(-a1, y, fma(b1, x, s2))`,
/// `s2 = fma(-a2, y, b0 x)`. Its Float32 fixed point on a unit step is the
/// reference's +0.052 dB "trim" at 48 kHz.
#[derive(Clone, Copy)]
struct Dezipper {
    b0: f32,
    b1: f32,
    a1: f32,
    a2: f32,
    s: [f32; 2],
}

impl Dezipper {
    fn new(rate: f64) -> Self {
        let w = 2.0 * core::f64::consts::PI * DEZIPPER_HZ / rate;
        let al = libm::sin(w) / (2.0 * DEZIPPER_Q);
        let a0 = 1.0 + al;
        let c = libm::cos(w);
        let b0 = ((1.0 - c) / 2.0 / a0) as f32;
        Dezipper { b0, b1: 2.0 * b0, a1: (-2.0 * c / a0) as f32, a2: ((1.0 - al) / a0) as f32, s: [0.0; 2] }
    }
    #[inline]
    fn step(&mut self, x: f32) -> f32 {
        let y = libm::fmaf(self.b0, x, self.s[0]);
        self.s[0] = libm::fmaf(-self.a1, y, libm::fmaf(self.b1, x, self.s[1]));
        self.s[1] = libm::fmaf(-self.a2, y, self.b0 * x);
        y
    }
    /// Settled on the step (a hand-off: the stream was already running).
    fn settle(&mut self) {
        for _ in 0..1 << 18 {
            self.step(1.0);
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

    /// Set one AU parameter (spec 2) in the payload. False for an unknown ID
    /// or an unusable mask value.
    pub fn set(&mut self, id: u32, v: f32) -> bool {
        let flag = (v != 0.0) as u32;
        match id {
            ids::BYPASS => self.bypass = flag,
            ids::GAIN_LINK_ALL => self.gain_link_all = flag,
            1..=39 => {
                let s = &mut self.slot[((id - 1) / ids::SLOT_STRIDE) as usize];
                match (id - 1) % ids::SLOT_STRIDE {
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
                    _ => {
                        if !(v.is_finite() && v >= 0.0) {
                            return false;
                        }
                        s.channel_mask = v as u32 & 0xffff;
                    }
                }
            }
            _ => return false,
        }
        true
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

/// 99 %-settling smoothing coefficient for a time in ms (spec 3).
pub fn settle_coeff(rate: f64, ms: f64) -> f64 {
    1.0 - libm::exp(libm::log(0.01) / (ms * 1e-3 * rate))
}

struct Chan {
    slot: usize,
    /// Vfs^2 / Rdc: watts per unit mean square.
    elec: f64,
    rth_vc: f64,
    rth_mg: f64,
    a_vc: f64,
    a_mg: f64,
    tau_vc: f64,
    tau_mg: f64,
    ambient: f64,
    limit: f64,
    hard_limit: f64,
    a_attack: f64,
    a_release: f64,
    theta_vc: f64,
    theta_mg: f64,
    /// Drive power estimate of the unattenuated input, W.
    p_drive: f64,
    /// Modelled power of the protected signal, W (the last sample).
    p_model: f64,
    limiting: bool,
    panic: bool,
    request: f64,
    gain: f64,
}

struct Run {
    n: usize,
    link_slot: [bool; 3],
    link_all: bool,
    ch: Vec<Chan>,
    /// Active (not bypassed) frames since initialisation: the dezipper clock.
    active: u64,
    mute: u64,
    /// Per channel: the output dezipper and its last value. Its input is the
    /// gain target: 0 during the startup mute, f32(0.1) in a panic, else 1
    /// (rig laws 4 and 6).
    dez: Vec<Dezipper>,
    dez_now: f32,
    frame: u64,
    /// A model state went non-finite: the fail-safe is latched until reset.
    fault: bool,
    /// The controller block (`block_frames`; 0 is the host call) and the
    /// position in it.
    block: usize,
    blk_pos: usize,
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
    /// The startup dezipper's current value.
    pub fn dezipper_gain(&self) -> f32 {
        self.run.as_ref().map_or(0.0, |r| r.dez_now)
    }
}

impl Run {
    fn new(p: &Params) -> Result<Run, InitError> {
        p.validate()?;
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let dt = 1.0 / rate;
        let route = p.route();
        let mut ch = Vec::with_capacity(n);
        for &si in &route[..n] {
            let s = &p.slot[si];
            let v = s.amplifier_sensitivity as f64;
            ch.push(Chan {
                slot: si,
                elec: v * v / s.rdc as f64,
                rth_vc: s.rth_vc as f64,
                rth_mg: s.rth_mg as f64,
                a_vc: dt / (s.tau_vc as f64 + dt),
                a_mg: dt / (s.tau_mg as f64 + dt),
                tau_vc: s.tau_vc as f64,
                tau_mg: s.tau_mg as f64,
                ambient: s.ambient as f64,
                limit: s.temperature_limit as f64,
                hard_limit: s.temperature_limit as f64 + s.hard_limit_headroom as f64,
                a_attack: settle_coeff(rate, s.attack_ms as f64),
                a_release: settle_coeff(rate, s.release_ms as f64),
                theta_vc: 0.0,
                theta_mg: 0.0,
                p_drive: 0.0,
                p_model: 0.0,
                limiting: false,
                panic: false,
                request: 1.0,
                gain: 1.0,
            });
        }
        Ok(Run {
            n,
            link_slot: [p.slot[0].gain_link != 0, p.slot[1].gain_link != 0, p.slot[2].gain_link != 0],
            link_all: p.gain_link_all != 0,
            ch,
            active: 0,
            mute: libm::round(STARTUP_MUTE_S * rate) as u64,
            dez: vec![Dezipper::new(rate); n],
            dez_now: 0.0,
            frame: 0,
            fault: false,
            block: p.block_frames as usize,
            blk_pos: 0,
        })
    }

    fn reset(&mut self) {
        for c in self.ch.iter_mut() {
            c.theta_vc = 0.0;
            c.theta_mg = 0.0;
            c.p_drive = 0.0;
            c.p_model = 0.0;
            c.limiting = false;
            c.panic = false;
            c.request = 1.0;
            c.gain = 1.0;
        }
        // the dezipper is not restarted by a reset (rig law)
        self.frame = 0;
        self.fault = false;
        self.blk_pos = 0;
    }

    /// Advance channel `c`'s output dezipper one active frame (module docs).
    #[inline]
    fn dezipper(&mut self, c: usize, panic: bool) -> f64 {
        let x = if self.active < self.mute {
            0.0
        } else if panic {
            PANIC_GAIN as f32
        } else {
            1.0
        };
        let y = self.dez[c].step(x);
        if c == 0 {
            self.dez_now = y;
        }
        y as f64
    }

    /// Returns whether any input sample was non-finite (the meter failure:
    /// those frames are silent, and the call reports the fail-safe).
    fn process(&mut self, io: &mut Io<'_, '_>) -> bool {
        let n = self.n;
        let mut failed = false;
        for k in 0..io.frames {
            // controller blocks of `block_frames` (0: the host call)
            let boundary = if self.block == 0 { k == 0 } else { self.blk_pos == 0 };
            self.blk_pos = if self.block == 0 { 0 } else { (self.blk_pos + 1) % self.block };
            // 1. requests from the state before this sample
            let mut req = [1.0f64; MAX_CHANNELS];
            let mut pan = [false; MAX_CHANNELS];
            for (c, ch) in self.ch.iter_mut().enumerate() {
                let t_vc = ch.ambient + ch.theta_mg + ch.theta_vc;
                // `>` enters, `<=` recovers; a NaN state stays restrictive
                ch.limiting = t_vc > ch.limit || t_vc.is_nan();
                // the panic enters at once and releases only at a controller
                // block boundary with the coil back at the hard limit (the
                // 25G83 controller is block-wise once it limits)
                if t_vc > ch.hard_limit || t_vc.is_nan() {
                    ch.panic = true;
                } else if boundary {
                    ch.panic = false;
                }
                let mut r = 1.0;
                if ch.limiting {
                    let allow = ((ch.limit - ch.ambient - ch.theta_mg) / ch.rth_vc).max(0.0);
                    r = libm::sqrt(allow / ch.p_drive.max(EPS)).min(1.0);
                    if !r.is_finite() {
                        r = PANIC_GAIN;
                    }
                }
                if ch.panic {
                    r = r.min(PANIC_GAIN);
                }
                ch.request = r;
                req[c] = r;
                pan[c] = ch.panic;
            }
            for s in 0..3 {
                if self.link_slot[s] {
                    let m = self.ch.iter().filter(|c| c.slot == s).fold(1.0f64, |m, c| nan_min(m, c.request));
                    let any = self.ch.iter().any(|c| c.slot == s && c.panic);
                    for (c, ch) in self.ch.iter().enumerate() {
                        if ch.slot == s {
                            req[c] = m;
                            pan[c] |= any;
                        }
                    }
                }
            }
            if self.link_all {
                let m = req[..n].iter().fold(1.0f64, |m, &r| nan_min(m, r));
                req[..n].iter_mut().for_each(|r| *r = m);
                let any = pan[..n].iter().any(|&p| p);
                pan[..n].iter_mut().for_each(|p| *p |= any);
            }
            // 2. smooth, apply, integrate
            let mut dz = [0.0f64; MAX_CHANNELS];
            for (c, d) in dz.iter_mut().enumerate().take(n) {
                *d = self.dezipper(c, pan[c]);
            }
            for (c, ch) in self.ch.iter_mut().enumerate() {
                let d = dz[c];
                let a = if req[c] < ch.gain { ch.a_attack } else { ch.a_release };
                ch.gain += a * (req[c] - ch.gain);
                // the program held to digital full scale before the gain and
                // both meters; a non-finite sample is 0
                let raw = io.input[c][k];
                failed |= !raw.is_finite();
                // the output takes the program held to full scale; the
                // meters see it unclamped (to METER_FS) as the reference
                let x = crate::dsp::bound1(raw) as f64;
                let xm = crate::dsp::meter1(raw) as f64;
                // The dezipper's own trajectory passes as the reference's
                // does (its 1.0283 overshoot and its +0.052 dB fixed point),
                // bounded at its peak; the output is held to full scale.
                // The panic feeds the dezipper a 0.1 target (rig law 4: the
                // reference's 1 -> 0.081 -> 0.101 trajectory over about
                // 40 ms) and, stricter than the reference for those 40 ms,
                // also caps the gain at 0.1 at once (coordinator decision).
                let g = ch.gain * d;
                let g = if g.is_finite() { g.clamp(0.0, DEZIPPER_PEAK as f64) } else { 0.0 };
                let g = if pan[c] { g.min(PANIC_GAIN) } else { g };
                io.output[c][k] = ((x * g) as f32).clamp(-1.0, 1.0);
                ch.p_drive += ch.a_attack * (ch.elec * xm * xm - ch.p_drive);
                let u = ch.gain * xm;
                let p = ch.elec * u * u;
                ch.p_model = p;
                // series network: the coil rise over the magnet plus the
                // magnet rise over ambient (module docs, TODO(SPEC))
                ch.theta_vc += ch.a_vc * (p * ch.rth_vc - ch.theta_vc);
                ch.theta_mg += ch.a_mg * (p * ch.rth_mg - ch.theta_mg);
            }
            self.active = self.active.saturating_add(1);
            // any non-finite model or controller state latches the fail-safe
            if !self.ch.iter().all(|c| c.theta_vc.is_finite() && c.theta_mg.is_finite() && c.gain.is_finite()) {
                self.fault = true;
                self.frame += k as u64 + 1;
                crate::failsafe::fill_from(io, k + 1, FAILSAFE_GAIN);
                for c in 0..n {
                    io.output[c][k] = crate::dsp::bound1(io.input[c][k]) * FAILSAFE_GAIN;
                }
                return failed;
            }
        }
        self.frame += io.frames as u64;
        failed
    }
}

/// `min` that keeps a NaN (a NaN request must never read as unity).
#[inline]
fn nan_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
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
            s.modeled_power_w = c.p_model as f32;
            s.coil_temp_c = (c.ambient + c.theta_mg + c.theta_vc) as f32;
            s.magnet_temp_c = (c.ambient + c.theta_mg) as f32;
            s.ol_temp_c = s.coil_temp_c;
            s.axis_gain[4] = c.request as f32;
            s.gain = if run.fault {
                FAILSAFE_GAIN
            } else if self.bypass {
                1.0
            } else {
                c.gain as f32
            };
            out.channels[i] = s;
        }
    }
    fn reset(&mut self) {
        if let Some(r) = self.run.as_mut() {
            r.reset();
        }
    }
    fn save_state(&self, out: &mut HandOff) -> bool {
        let Some(run) = self.run.as_ref().filter(|r| !r.fault) else { return false };
        *out = HandOff::EMPTY;
        out.kind = UnitKind::Atsp as u32;
        out.channels = run.n as u32;
        out.sample_rate = self.params.sample_rate;
        for (i, c) in run.ch.iter().enumerate() {
            out.node[i][..2].copy_from_slice(&[c.theta_vc, c.theta_mg]);
            out.gain[i] = c.gain;
        }
        true
    }

    /// The coil and magnet rises cooled for the gap by the continuous
    /// solution of the series pair (the coil rise over the magnet decays with
    /// its own constant), the gain released toward unity. The reference
    /// resets on reinitialisation (spec 5); a speaker does not cool because a
    /// stream restarted, so here it never does. The startup dezipper is not
    /// repeated after a hand-off.
    fn restore_state(&mut self, state: &HandOff, gap_s: f64) -> bool {
        let rate = self.params.sample_rate;
        let Some(run) = self.run.as_mut() else { return false };
        if !state.fits(UnitKind::Atsp, run.n, rate) {
            return false;
        }
        let t = gap_or_zero(gap_s);
        for (i, c) in run.ch.iter_mut().enumerate() {
            c.theta_vc = state.node[i][0] * libm::exp(-t / c.tau_vc);
            c.theta_mg = state.node[i][1] * libm::exp(-t / c.tau_mg);
            let r_release = 1.0 - c.a_release;
            c.gain = release_toward_unity(state.gain[i], r_release, t * rate as f64);
            c.request = c.gain;
        }
        run.active = run.active.max(run.mute);
        run.dez.iter_mut().for_each(Dezipper::settle);
        true
    }

    fn set_parameter(&mut self, id: u32, value: f32) -> bool {
        if id == ids::BYPASS {
            self.bypass = value != 0.0;
            self.params.bypass = self.bypass as u32;
            return true;
        }
        false
    }
}
