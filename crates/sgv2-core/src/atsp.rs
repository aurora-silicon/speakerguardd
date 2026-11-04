//! `atsp`: audio-only two-node thermal protection (13" MacBook Pro M1/M2,
//! MacBook Air M1), from the behaviour specification:
//!
//! - three speaker-type slots, each with its own model and limits and a
//!   16-bit channel assignment mask (zero disables the slot);
//! - per channel, `v = AmplifierSensitivity * x`, `p = v^2 / Rdc`, `P` the
//!   mean over the update interval (the graph's block, as for spp3: the
//!   unit runs on its own block clock and is split-invariant);
//! - the coupled two-node backward-Euler model with the old magnet state in
//!   the coil target;
//! - the infinite-ratio ceiling: `g = min(1, sqrt(P_allow / P_drive))` with
//!   `P_allow = (T_limit - ambient - theta_mg) / Rth_vc` from the current
//!   magnet state;
//! - attack/release smoothing of the target in linear gain, per sample;
//! - linking: a slot's `Gain link` takes the minimum over its channels, and
//!   `Gain link all audio channels` the minimum over every active channel;
//! - the 20 dB panic while `T_vc > T_limit + headroom`, at once;
//! - reset at ambient and unity gain.
//!
//! A tuning with the global `Bypass` set is refused (the unit then runs the
//! fail-safe): bypass is never a protection state. A channel no slot claims
//! is unconstrained by the specification; it is refused too, since an
//! unprotected speaker channel is never acceptable here.
//!
//! TODO(SPEC): backward Euler versus exact exponential (spec 8), the
//! smoothing pole formula (`exp(-1/(tau*rate))`), and the panic comparison
//! at equality (strict here).

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::{dsp, ChannelState, Health, Io, ProtectionUnit, Telemetry, UnitKind, FAILSAFE_GAIN, MAX_CHANNELS};

const EPS: f64 = f32::MIN_POSITIVE as f64;

/// One speaker-type slot (A, B or C).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Slot {
    /// `Audio channel assignment`: bit c claims host channel c.
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
    pub fn validate(&self) -> Result<(), InitError> {
        let n = self.channels as usize;
        if n == 0 || n > MAX_CHANNELS || self.sample_rate == 0 {
            return Err(InitError::Invalid("geometry"));
        }
        if self.block_frames == 0 || self.block_frames > 16384 {
            return Err(InitError::Invalid("block frames"));
        }
        let pos = |v: f32| v.is_finite() && v > 0.0;
        let mut claimed = 0u32;
        for s in &self.slot {
            if s.channel_mask == 0 {
                continue;
            }
            if s.channel_mask >> n != 0 {
                return Err(InitError::Invalid("channel mask beyond the channel count"));
            }
            if claimed & s.channel_mask != 0 {
                return Err(InitError::Invalid("channel claimed by two slots"));
            }
            claimed |= s.channel_mask;
            let positive = [s.amplifier_sensitivity, s.rdc, s.rth_vc, s.tau_vc, s.tau_mg, s.attack_ms, s.release_ms]
                .iter()
                .all(|&v| pos(v));
            let finite = [s.ambient, s.temperature_limit, s.hard_limit_headroom].iter().all(|v| v.is_finite());
            let magnet = s.rth_mg.is_finite() && s.rth_mg >= 0.0;
            if !(positive && finite && magnet) {
                return Err(InitError::Invalid("slot constants"));
            }
        }
        if claimed != (1u32 << n) - 1 {
            return Err(InitError::Invalid("a channel no slot protects"));
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

struct Chan {
    elec: f64,
    span: f64,
    rth_vc: f64,
    rth_mg: f64,
    a_vc: f64,
    a_mg: f64,
    hard_limit: f64,
    ambient: f64,
    group: usize,
    acc: dsp::LaneAcc,
    acc_in: dsp::LaneAcc,
    theta_vc: f64,
    theta_mg: f64,
    p_model: f64,
    g_target: f64,
    hard: bool,
}

struct Group {
    r_attack: f32,
    r_release: f32,
    gain: f32,
    hard: bool,
}

struct Run {
    n: usize,
    block: usize,
    ch: Vec<Chan>,
    groups: Vec<Group>,
    /// Gain of every frame of the current block, per group.
    curve: Vec<f32>,
    pos: usize,
    frame: u64,
    last_frame: u64,
}

pub struct Unit {
    params: Params,
    run: Option<Run>,
}

impl Unit {
    pub fn new(params: Params) -> Self {
        let run = Run::new(&params).ok();
        Unit { params, run }
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
}

fn pole(rate: f64, tau_s: f64) -> f32 {
    libm::exp(-1.0 / (tau_s * rate)) as f32
}

impl Run {
    fn new(p: &Params) -> Result<Run, InitError> {
        p.validate()?;
        if p.bypass != 0 {
            return Err(InitError::Unsupported("Bypass"));
        }
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let block = p.block_frames as usize;
        let dt = block as f64 / rate;
        let mut groups: Vec<Group> = Vec::new();
        let mut ch: Vec<Option<Chan>> = (0..n).map(|_| None).collect();
        // one group for everything when linked globally
        let all = if p.gain_link_all != 0 { Some(0) } else { None };
        for s in p.slot.iter().filter(|s| s.channel_mask != 0) {
            let mk = |groups: &mut Vec<Group>| {
                groups.push(Group {
                    r_attack: pole(rate, s.attack_ms as f64 * 1e-3),
                    r_release: pole(rate, s.release_ms as f64 * 1e-3),
                    gain: 1.0,
                    hard: false,
                });
                groups.len() - 1
            };
            let slot_group = if all.is_some() {
                if groups.is_empty() {
                    Some(mk(&mut groups))
                } else {
                    Some(0)
                }
            } else if s.gain_link != 0 {
                Some(mk(&mut groups))
            } else {
                None
            };
            for (c, slot_ch) in ch.iter_mut().enumerate() {
                if s.channel_mask & (1 << c) == 0 {
                    continue;
                }
                let group = slot_group.unwrap_or_else(|| mk(&mut groups));
                let v = s.amplifier_sensitivity as f64;
                *slot_ch = Some(Chan {
                    elec: v * v / s.rdc as f64,
                    span: s.temperature_limit as f64 - s.ambient as f64,
                    rth_vc: s.rth_vc as f64,
                    rth_mg: s.rth_mg as f64,
                    a_vc: dt / (s.tau_vc as f64 + dt),
                    a_mg: dt / (s.tau_mg as f64 + dt),
                    hard_limit: s.temperature_limit as f64 + s.hard_limit_headroom as f64,
                    ambient: s.ambient as f64,
                    group,
                    acc: dsp::LaneAcc::new(),
                    acc_in: dsp::LaneAcc::new(),
                    theta_vc: 0.0,
                    theta_mg: 0.0,
                    p_model: 0.0,
                    g_target: 1.0,
                    hard: false,
                });
            }
        }
        let ch: Vec<Chan> =
            ch.into_iter().map(|c| c.ok_or(InitError::Invalid("unclaimed channel"))).collect::<Result<_, _>>()?;
        let curve = vec![1.0; groups.len() * block];
        Ok(Run { n, block, ch, groups, curve, pos: 0, frame: 0, last_frame: 0 })
    }

    fn reset(&mut self) {
        for c in self.ch.iter_mut() {
            c.acc.clear();
            c.acc_in.clear();
            c.theta_vc = 0.0;
            c.theta_mg = 0.0;
            c.p_model = 0.0;
            c.g_target = 1.0;
            c.hard = false;
        }
        for g in self.groups.iter_mut() {
            g.gain = 1.0;
            g.hard = false;
        }
        self.curve.iter_mut().for_each(|g| *g = 1.0);
        self.pos = 0;
        self.frame = 0;
        self.last_frame = 0;
    }

    fn segment(&mut self, io: &mut Io<'_, '_>, off: usize, len: usize) {
        let phase = self.pos;
        for (c, ch) in self.ch.iter_mut().enumerate() {
            let inp = &io.input[c][off..off + len];
            let out = &mut io.output[c][off..off + len];
            let curve = &self.curve[ch.group * self.block + phase..][..len];
            let mut acc = ch.acc_in;
            acc.add_sq(inp, phase);
            if !acc.is_finite() {
                acc = ch.acc_in;
                for (k, &v) in inp.iter().enumerate() {
                    acc.add_sq(&[if v.is_finite() { v } else { 0.0 }], phase + k);
                }
            }
            ch.acc_in = acc;
            dsp::mul_to(inp, curve, out);
            if dsp::peak_abs(out) == f32::INFINITY {
                for ((o, &x), &g) in out.iter_mut().zip(inp).zip(curve) {
                    *o = if x.is_finite() { x * g.min(FAILSAFE_GAIN) } else { 0.0 };
                }
            }
            // P covers the protected drive x[n]
            ch.acc.add_sq(out, phase);
        }
        self.frame += len as u64;
    }

    fn end_block(&mut self) {
        let blk = self.block as f64;
        let mut tgt = [1.0f64; MAX_CHANNELS];
        let mut hard = [false; MAX_CHANNELS];
        for c in self.ch.iter_mut() {
            let p = c.elec * c.acc.total() / blk;
            c.acc.clear();
            c.p_model = p;
            let old = c.theta_mg;
            c.theta_vc += c.a_vc * (old + p * c.rth_vc - c.theta_vc);
            c.theta_mg += c.a_mg * (p * c.rth_mg - c.theta_mg);
            // the ceiling is against the unprotected drive
            let p_drive = c.elec * c.acc_in.total() / blk;
            c.acc_in.clear();
            let allow = (c.span - c.theta_mg) / c.rth_vc;
            c.g_target = libm::sqrt(allow.max(0.0) / p_drive.max(EPS)).min(1.0);
            c.hard = c.ambient + c.theta_vc > c.hard_limit;
            tgt[c.group] = tgt[c.group].min(c.g_target);
            hard[c.group] |= c.hard;
        }
        for (gi, g) in self.groups.iter_mut().enumerate() {
            let t = tgt[gi] as f32;
            let r = if t < g.gain { g.r_attack } else { g.r_release };
            let curve = &mut self.curve[gi * self.block..][..self.block];
            let mut v = g.gain;
            for c in curve.iter_mut() {
                v = t + r * (v - t);
                *c = v;
            }
            g.gain = v;
            g.hard = hard[gi];
            if g.hard {
                curve.iter_mut().for_each(|c| *c *= FAILSAFE_GAIN);
            }
        }
        self.last_frame = self.frame - self.block as u64;
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
        let mut off = 0;
        while off < frames {
            let len = (run.block - run.pos).min(frames - off);
            run.segment(&mut io, off, len);
            off += len;
            run.pos += len;
            if run.pos == run.block {
                run.end_block();
                run.pos = 0;
            }
        }
        Health::OK
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
        out.frame = run.last_frame;
        for (i, c) in run.ch.iter().enumerate() {
            let g = &run.groups[c.group];
            let mut s = ChannelState::ABSENT;
            s.modeled_power_w = c.p_model as f32;
            s.coil_temp_c = (c.ambient + c.theta_vc) as f32;
            s.magnet_temp_c = (c.ambient + c.theta_mg) as f32;
            s.ol_temp_c = s.coil_temp_c;
            s.axis_gain[4] = c.g_target as f32;
            s.gain = if g.hard { g.gain * FAILSAFE_GAIN } else { g.gain };
            out.channels[i] = s;
        }
    }
    fn reset(&mut self) {
        if let Some(r) = self.run.as_mut() {
            r.reset();
        }
    }
}
