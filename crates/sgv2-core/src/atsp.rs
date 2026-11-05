//! `atsp`: audio-only two-node thermal protection (the 13-inch MacBook Pro
//! M1/M2, the MacBook Air M1 and the one-speaker desktops), from the
//! behaviour specification (SPEC-ATSP, 25G83; section 10 for the
//! controller):
//!
//! - **Routing** (spec 4). Each in-range stream channel takes the first of
//!   slots A, B, C whose 16-bit assignment mask has its bit; a channel no
//!   mask claims is routed to A. Mask bits at or above the stream width are
//!   ignored, so wide masks (`0xffff` on a one-channel desktop) and a zero A
//!   mask are valid.
//! - **Windows** (spec 10). The controller runs on non-overlapping
//!   rectangular windows of `ceil(fs * 5.333 ms)` frames (256 at 48 kHz) on
//!   a grid that starts at the first active frame and runs across host
//!   calls. A call of fewer frames than a window evaluates each window as
//!   its last frame is in, after that frame's output; a longer call stages
//!   two windows at a time and evaluates every window it completes before
//!   passing the staged output through the dezipper (the Q2/3, Q256 and
//!   Q512+ families).
//! - **Thermal model** (spec 10), per channel: two series nodes, each a
//!   matched exponential per window (`b = exp(-N / (fs tau))`) toward its
//!   share of the window's power, `Tvc = Tamb + theta_mg + theta_vc`. Two
//!   instances run: the input-referred predictor on the unattenuated
//!   program (`Pdrive = (Vfs rms(x))^2 / Rdc`), and the post-gain predictor
//!   on each completed window of the unit's own output.
//! - **Controller** (spec 10). The window limits when the hotter predictor
//!   is strictly above `Tlim`; the request is then
//!   `sqrt(clamp(((Tlim - Tamb) - theta_mg_post) / (theta_vc_input + 1), 0, 1))`,
//!   else unity. The target follows with the slot's attack or release time
//!   as a 99 % settling per window, `1 - exp(ln(0.01) N / (t fs))`; equal
//!   leaves it. When the post-gain coil is strictly above
//!   `Tlim + headroom` the target is the 20 dB panic target `f32(0.1)` at
//!   once (spec 10, rig law 4).
//! - **Linking.** A slot's `Gain link` replaces its channels' targets by
//!   their minimum, then `Gain link all audio channels` (ID 40) replaces
//!   every target by the global minimum.
//! - **Output.** `y = x * d`, held to full scale, where `d` is the output
//!   dezipper (rig-confirmed, bit-exact): zero for the first 5 ms after
//!   initialisation, then a Float32 RBJ low pass (20 Hz, Q 0.6513333,
//!   transposed direct form II with fused multiply-adds) of the linked
//!   target, which reaches each new value by a linear ramp over one window.
//!   Its Float32 fixed point is the reference's "+0.052 dB trim" (1.0060077
//!   at 48 kHz, rate dependent): gain state, not a stage. An
//!   `AudioUnitReset` does not restart it; silence advances it. Zero
//!   latency.
//! - **Bypass** (ID 0, at the next call): a bit-exact copy that freezes all
//!   thermal, gain, link and startup state; clearing it resumes that state.
//!
//! Non-finite input gives a zero output sample and the fail-safe report, and
//! a non-finite model state latches the fail-safe until reset, so neither
//! can poison the thermal state.
//!
//! TODO(SPEC): the dezipper input. The 25G83 gain path is not the
//! per-window target the specification states: under limiting its
//! dezipper input moves in Float32 per-sample increments (integer
//! multiples of the gain's own ULP, stalling when the increment is below
//! half of one) plus instantaneous steps on a 5 ms grid whose size scales
//! with the gain's binary exponent (Daybreak t-4682). This unit tracks
//! that trajectory's time average: the modelled coil temperature its output
//! produces is within 0.1 °C of the reference's, and cooler where it
//! differs more, on the rig's limiting cases, but its audio is not within
//! the 3e-4 rig tolerance under limiting. Q = 1 takes a distinct path on
//! macOS that is not specified; this unit renders Q = 1 like Q = 2.

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::{
    gap_or_zero, release_toward_unity, ChannelState, HandOff, Health, Io, ProtectionUnit, Telemetry, UnitKind,
    FAILSAFE_GAIN, MAX_CHANNELS,
};

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
    y: f32,
}

impl Dezipper {
    fn new(rate: f64) -> Self {
        let w = 2.0 * core::f64::consts::PI * DEZIPPER_HZ / rate;
        let al = libm::sin(w) / (2.0 * DEZIPPER_Q);
        let a0 = 1.0 + al;
        let c = libm::cos(w);
        let b0 = ((1.0 - c) / 2.0 / a0) as f32;
        Dezipper { b0, b1: 2.0 * b0, a1: (-2.0 * c / a0) as f32, a2: ((1.0 - al) / a0) as f32, s: [0.0; 2], y: 0.0 }
    }
    #[inline]
    fn step(&mut self, x: f32) -> f32 {
        let y = libm::fmaf(self.b0, x, self.s[0]);
        self.s[0] = libm::fmaf(-self.a1, y, libm::fmaf(self.b1, x, self.s[1]));
        self.s[1] = libm::fmaf(-self.a2, y, self.b0 * x);
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
/// (`1 - exp(ln(0.01) / (t rate))`).
pub fn settle_coeff(rate: f64, ms: f64) -> f64 {
    window_coeff(rate, ms, 1)
}

/// The same 99 %-settling coefficient for one controller window of `n`
/// frames (SPEC-ATSP 10): `1 - exp(ln(0.01) n / (t rate / 1000))`.
pub fn window_coeff(rate: f64, ms: f64, n: usize) -> f64 {
    1.0 - libm::exp(libm::log(0.01) * n as f64 / (ms * 1e-3 * rate))
}

/// The controller window in frames: `ceil(rate * 5.333333333 ms)`, 256 at
/// 48 kHz (SPEC-ATSP 10).
pub fn window_frames(rate: f64) -> usize {
    libm::ceil(rate * WINDOW_MS * 1e-3) as usize
}

/// The window's length in ms (SPEC-ATSP 10).
const WINDOW_MS: f64 = 5.333_333_333;

/// One two-node series model (SPEC-ATSP 10): the coil's rise over the
/// magnet and the magnet's over ambient, each a matched exponential per
/// window toward its share of the window's power.
#[derive(Clone, Copy, Default)]
struct Nodes {
    vc: f64,
    mg: f64,
}

impl Nodes {
    #[inline]
    fn step(&mut self, c: &Coeffs, p: f64) {
        self.vc = c.bvc * self.vc + (1.0 - c.bvc) * (c.rth_vc * p);
        self.mg = c.bmg * self.mg + (1.0 - c.bmg) * (c.rth_mg * p);
    }
    #[inline]
    fn finite(&self) -> bool {
        self.vc.is_finite() && self.mg.is_finite()
    }
}

/// Per channel constants, from the routed slot.
#[derive(Clone, Copy)]
struct Coeffs {
    slot: usize,
    /// Vfs^2 / Rdc: watts per unit mean square.
    elec: f64,
    rth_vc: f64,
    rth_mg: f64,
    tau_vc: f64,
    tau_mg: f64,
    bvc: f64,
    bmg: f64,
    ambient: f64,
    limit: f64,
    hard_limit: f64,
    ka: f64,
    kr: f64,
    /// Per-sample release coefficient (the hand-off's gap ageing).
    kr_sample: f64,
}

impl Coeffs {
    fn new(p: &Params, slot: usize, rate: f64, n: usize) -> Coeffs {
        let s = &p.slot[slot];
        let v = s.amplifier_sensitivity as f64;
        let w = n as f64 / rate;
        let tvc = s.tau_vc as f64;
        let tmg = s.tau_mg as f64;
        Coeffs {
            slot,
            elec: v * v / s.rdc as f64,
            rth_vc: s.rth_vc as f64,
            rth_mg: s.rth_mg as f64,
            tau_vc: s.tau_vc as f64,
            tau_mg: s.tau_mg as f64,
            bvc: libm::exp(-w / tvc),
            bmg: libm::exp(-w / tmg),
            ambient: s.ambient as f64,
            limit: s.temperature_limit as f64,
            hard_limit: s.temperature_limit as f64 + s.hard_limit_headroom as f64,
            ka: window_coeff(rate, s.attack_ms as f64, n),
            kr: window_coeff(rate, s.release_ms as f64, n),
            kr_sample: settle_coeff(rate, s.release_ms as f64),
        }
    }
}

struct Chan {
    k: Coeffs,
    /// The current window's sum of squares of the program (metered, as
    /// the reference's input-referred estimator sees it).
    acc: f64,
    /// The current output window's sum of squares (the post-gain
    /// predictor's input).
    acc_out: f64,
    /// The input-referred predictor (driven by the unattenuated program).
    input: Nodes,
    /// The post-gain predictor (driven by the program at the applied
    /// target).
    post: Nodes,
    /// The last window's drive power, W.
    p_drive: f64,
    /// The last window's post-gain power, W.
    p_post: f64,
    limiting: bool,
    panic: bool,
    /// This channel or one linked to it is in the panic: the applied gain is
    /// held at the panic target at once (stricter than the reference, whose
    /// dezipper takes about 40 ms to reach it).
    capped: bool,
    /// The last window's unsmoothed request.
    request: f64,
    /// The smoothed per-channel target.
    target: f64,
    /// The linked target: the dezipper's input after the startup mute.
    linked: f32,
    /// The dezipper's input ramps from this value to `linked` over one
    /// window after each new target.
    ramp_from: f32,
    /// Frames into that ramp.
    ramp_pos: usize,
}

impl Chan {
    fn new(k: Coeffs) -> Chan {
        Chan {
            k,
            acc: 0.0,
            acc_out: 0.0,
            input: Nodes::default(),
            post: Nodes::default(),
            p_drive: 0.0,
            p_post: 0.0,
            limiting: false,
            panic: false,
            capped: false,
            request: 1.0,
            target: 1.0,
            linked: 1.0,
            ramp_from: 1.0,
            ramp_pos: usize::MAX,
        }
    }

    fn reset(&mut self) {
        *self = Chan::new(self.k);
    }

    /// The output of one window is complete: the post-gain predictor
    /// takes its power.
    fn post_window(&mut self, n: usize) -> bool {
        let k = self.k;
        let pp = k.elec * self.acc_out / n as f64;
        self.acc_out = 0.0;
        self.p_post = pp;
        self.post.step(&k, pp);
        let t_post = k.ambient + self.post.mg + self.post.vc;
        self.panic = t_post > k.hard_limit || t_post.is_nan();
        if self.panic {
            self.target = PANIC_GAIN as f32 as f64;
        }
        self.panic
    }

    /// One completed input window (SPEC-ATSP 10).
    fn window(&mut self, n: usize) {
        let k = self.k;
        let pd = k.elec * self.acc / n as f64;
        self.acc = 0.0;
        self.p_drive = pd;
        self.input.step(&k, pd);
        let t_in = k.ambient + self.input.mg + self.input.vc;
        let t_post = k.ambient + self.post.mg + self.post.vc;
        // `>` enters, `<=` recovers, on the hotter of the two predictors;
        // a NaN state stays restrictive
        let t = if t_in.is_nan() || t_post.is_nan() { f64::NAN } else { t_in.max(t_post) };
        self.limiting = t > k.limit || t.is_nan();
        let mut r = 1.0;
        if self.limiting {
            let ratio = ((k.limit - k.ambient) - self.post.mg) / (self.input.vc + 1.0);
            r = libm::sqrt(if ratio.is_nan() { 0.0 } else { ratio.clamp(0.0, 1.0) });
        }
        self.request = r;
        if r < self.target {
            self.target += k.ka * (r - self.target);
        } else if r > self.target {
            self.target += k.kr * (r - self.target);
        }
    }

    /// The dezipper's input at the next frame: the linked target, reached
    /// by a linear ramp over one window of `w` frames.
    #[inline]
    fn current(&self, w: usize) -> f32 {
        if self.ramp_pos >= w {
            self.linked
        } else {
            self.ramp_from + (self.linked - self.ramp_from) * ((self.ramp_pos + 1) as f32 / w as f32)
        }
    }

    fn finite(&self) -> bool {
        self.input.finite() && self.post.finite() && self.target.is_finite() && self.acc.is_finite()
    }
}

struct Run {
    n: usize,
    /// The controller window, frames.
    win: usize,
    /// Frames into the current input window (the grid runs on active
    /// frames).
    pos: usize,
    /// Frames into the current output window.
    out_pos: usize,
    link_slot: [bool; 3],
    link_all: bool,
    ch: Vec<Chan>,
    /// Active (not bypassed) frames since initialisation: the dezipper clock.
    active: u64,
    mute: u64,
    /// Per channel: the output dezipper. Its input is 0 during the startup
    /// mute, then the linked window target (rig laws 4 and 6).
    dez: Vec<Dezipper>,
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
    /// The startup dezipper's current value.
    pub fn dezipper_gain(&self) -> f32 {
        self.run.as_ref().map_or(0.0, |r| r.dez_now)
    }
    /// The controller window, frames.
    pub fn window_frames(&self) -> usize {
        self.run.as_ref().map_or(0, |r| r.win)
    }
}

impl Run {
    fn new(p: &Params) -> Result<Run, InitError> {
        p.validate()?;
        let p = &p.clamped();
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let win = window_frames(rate);
        if win == 0 {
            return Err(InitError::Invalid("geometry"));
        }
        let route = p.route();
        let ch = route[..n].iter().map(|&s| Chan::new(Coeffs::new(p, s, rate, win))).collect();
        Ok(Run {
            n,
            win,
            pos: 0,
            out_pos: 0,
            link_slot: [p.slot[0].gain_link != 0, p.slot[1].gain_link != 0, p.slot[2].gain_link != 0],
            link_all: p.gain_link_all != 0,
            ch,
            active: 0,
            mute: libm::round(STARTUP_MUTE_S * rate) as u64,
            dez: vec![Dezipper::new(rate); n],
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
            ch.k = Coeffs::new(p, route[c], rate, self.win);
        }
        self.link_slot = [p.slot[0].gain_link != 0, p.slot[1].gain_link != 0, p.slot[2].gain_link != 0];
        self.link_all = p.gain_link_all != 0;
    }

    fn reset(&mut self) {
        self.ch.iter_mut().for_each(Chan::reset);
        // the dezipper is not restarted by a reset (rig law 5)
        self.pos = 0;
        self.out_pos = 0;
        self.frame = 0;
        self.fault = false;
    }

    /// Every channel's window is complete: update the targets, then link.
    fn windows(&mut self) {
        let win = self.win;
        for ch in self.ch.iter_mut() {
            ch.window(win);
        }
        self.link();
    }

    /// The linked targets from the per-channel targets.
    fn link(&mut self) {
        let n = self.n;
        let mut lk = [1.0f64; MAX_CHANNELS];
        for (c, ch) in self.ch.iter().enumerate() {
            lk[c] = ch.target;
        }
        for s in 0..3 {
            if self.link_slot[s] {
                let m = self.ch.iter().filter(|c| c.k.slot == s).fold(1.0f64, |m, c| nan_min(m, c.target));
                for (c, ch) in self.ch.iter().enumerate() {
                    if ch.k.slot == s {
                        lk[c] = m;
                    }
                }
            }
        }
        if self.link_all {
            let m = lk[..n].iter().fold(1.0f64, |m, &r| nan_min(m, r));
            lk[..n].iter_mut().for_each(|r| *r = m);
        }
        self.caps();
        for (c, ch) in self.ch.iter_mut().enumerate() {
            let cur = ch.current(self.win);
            ch.ramp_from = cur;
            ch.ramp_pos = 0;
            ch.linked = if lk[c].is_finite() { lk[c] as f32 } else { 0.0 };
        }
        if !self.ch.iter().all(Chan::finite) {
            self.fault = true;
        }
    }

    /// The panic caps, linked as the targets are: a channel is capped while
    /// it or a channel linked to it is in the panic, decided at each window
    /// of the grid (so on no host slicing).
    fn caps(&mut self) {
        let n = self.n;
        let mut pan = [false; MAX_CHANNELS];
        for (c, ch) in self.ch.iter().enumerate() {
            pan[c] = ch.panic;
        }
        for s in 0..3 {
            if self.link_slot[s] && self.ch.iter().any(|c| c.k.slot == s && c.panic) {
                for (c, ch) in self.ch.iter().enumerate() {
                    pan[c] |= ch.k.slot == s;
                }
            }
        }
        if self.link_all && pan[..n].iter().any(|&p| p) {
            pan[..n].iter_mut().for_each(|p| *p = true);
        }
        for (c, ch) in self.ch.iter_mut().enumerate() {
            ch.capped = pan[c];
        }
    }

    /// Accumulate frames `a..b` of the program into the current window.
    #[inline]
    fn accumulate(&mut self, io: &Io<'_, '_>, a: usize, b: usize) -> bool {
        let mut failed = false;
        for (c, ch) in self.ch.iter_mut().enumerate() {
            let x = &io.input[c][a..b];
            failed |= !x.iter().all(|v| v.is_finite());
            let mut s = 0.0f64;
            for &v in x {
                let m = crate::dsp::meter1(v) as f64;
                s += m * m;
            }
            ch.acc += s;
        }
        self.pos += b - a;
        failed
    }

    /// Frames `a..b` of the output through each channel's dezipper. Each
    /// output window completed feeds the post-gain predictor.
    fn emit(&mut self, io: &mut Io<'_, '_>, a: usize, b: usize) {
        let win = self.win;
        let mut f = a;
        while f < b {
            let m = (win - self.out_pos).min(b - f);
            for c in 0..self.n {
                let ch = &mut self.ch[c];
                let d = &mut self.dez[c];
                let mut eo = 0.0f64;
                for (active, k) in (self.active..).zip(f..f + m) {
                    let target = ch.current(win);
                    ch.ramp_pos = ch.ramp_pos.saturating_add(1);
                    let x = if active < self.mute { 0.0 } else { target };
                    // the dezipper's trajectory passes as the reference's
                    // does, bounded at its startup peak
                    let y = d.step(x).min(DEZIPPER_PEAK);
                    let y = if ch.capped { y.min(PANIC_GAIN as f32) } else { y };
                    let o = io.input[c][k] * y;
                    let om = crate::dsp::meter1(o) as f64;
                    eo += om * om;
                    io.output[c][k] = crate::dsp::bound1(o);
                }
                ch.acc_out += eo;
            }
            self.active = self.active.saturating_add(m as u64);
            self.out_pos += m;
            f += m;
            let mut any = false;
            if self.out_pos == win {
                self.out_pos = 0;
                let win = self.win;
                for c in self.ch.iter_mut() {
                    any |= c.post_window(win);
                }
            }
            if any {
                self.link();
            }
        }
        if let Some(d) = self.dez.first() {
            self.dez_now = d.last();
        }
    }

    /// Returns whether any input sample was non-finite (the meter failure:
    /// those frames are silent, and the call reports the fail-safe).
    fn process(&mut self, io: &mut Io<'_, '_>) -> bool {
        let frames = io.frames;
        let mut failed = false;
        // the staging buffer: two windows (SPEC-ATSP 10, Q >= 512)
        let stage = 2 * self.win;
        let mut f = 0;
        if frames < self.win {
            // fragments: each window is evaluated once its last frame is
            // in, after that frame's output
            while f < frames {
                let m = (self.win - self.pos).min(frames - f);
                failed |= self.accumulate(io, f, f + m);
                self.emit(io, f, f + m);
                f += m;
                if self.pos == self.win {
                    self.pos = 0;
                    self.windows();
                }
            }
        } else {
            // staged: every window completed in a stage is evaluated
            // before the stage's output
            while f < frames {
                let m = stage.min(frames - f);
                let mut g = f;
                while g < f + m {
                    let k = (self.win - self.pos).min(f + m - g);
                    failed |= self.accumulate(io, g, g + k);
                    g += k;
                    if self.pos == self.win {
                        self.pos = 0;
                        self.windows();
                    }
                }
                self.emit(io, f, f + m);
                f += m;
            }
        }
        self.frame += frames as u64;
        if self.fault {
            crate::failsafe::fill_from(io, 0, FAILSAFE_GAIN);
        }
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
            s.modeled_power_w = c.p_post as f32;
            s.coil_temp_c = (c.k.ambient + c.post.mg + c.post.vc) as f32;
            s.magnet_temp_c = (c.k.ambient + c.post.mg) as f32;
            s.ol_temp_c = s.coil_temp_c;
            s.control_temp_c = (c.k.ambient + c.input.mg + c.input.vc) as f32;
            s.axis_gain[4] = c.request as f32;
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
    fn save_state(&self, out: &mut HandOff) -> bool {
        let Some(run) = self.run.as_ref().filter(|r| !r.fault) else { return false };
        *out = HandOff::EMPTY;
        out.kind = UnitKind::Atsp as u32;
        out.channels = run.n as u32;
        out.sample_rate = self.params.sample_rate;
        for (i, c) in run.ch.iter().enumerate() {
            out.node[i][..4].copy_from_slice(&[c.input.vc, c.input.mg, c.post.vc, c.post.mg]);
            out.gain[i] = (c.target as f32).clamp(f32::MIN_POSITIVE, 1.0) as f64;
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
            c.target = release_toward_unity(state.gain[i], 1.0 - c.k.kr_sample, t * rate as f64);
            c.request = c.target;
            c.linked = c.target as f32;
        }
        run.active = run.active.max(run.mute);
        for (c, d) in run.dez.iter_mut().enumerate() {
            d.settle(run.ch[c].linked);
        }
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
