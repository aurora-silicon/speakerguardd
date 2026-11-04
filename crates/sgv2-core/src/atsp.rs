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
//! - reset at ambient and unity gain;
//! - hand-off (`save_state`/`restore_state`) of the coil and magnet rises
//!   and the slot gain to the next stream or instance, cooled for the gap
//!   by the continuous two-node solution and the gain released toward
//!   unity. The reference resets on reinitialisation (spec 5); a speaker
//!   does not cool because a stream restarted, so here it never does.
//!
//! The output is the program held to digital full scale (`clamp(x, -1,
//! 1)`, a non-finite sample 0) times the gain. Both power meters see the
//! program unclamped as the reference meters it, bounded only at
//! `dsp::METER_FS` (16 FS) so no finite sample can overflow them. A model state that goes non-finite
//! latches the 20 dB fail-safe (`Health::MODEL_FAULT`) until reset.
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
use crate::{
    cool_coupled, dsp, gap_or_zero, release_toward_unity, ChannelState, HandOff, Health, Io, ProtectionUnit, Telemetry,
    UnitKind, FAILSAFE_GAIN, MAX_CHANNELS,
};

const EPS: f64 = f32::MIN_POSITIVE as f64;
/// Longest update in host-block mode (`block_frames = 0`), frames.
const HOST_MAX: usize = 4096;
/// Stack chunk for the metered drive.
const BOUND_CHUNK: usize = 64;

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
        if self.block_frames > 16384 {
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
    tau_vc: f64,
    tau_mg: f64,
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
    /// A non-finite program sample in the current block.
    meter_failed: bool,
}

struct Group {
    r_attack: f32,
    r_release: f32,
    /// Smoother state: the gain of the last frame generated.
    gain: f32,
    /// The current block's target and pole (set at the block end before).
    target: f32,
    r: f32,
    hard: bool,
}

struct Run {
    n: usize,
    block: usize,
    dynamic: bool,
    rate: f64,
    ch: Vec<Chan>,
    groups: Vec<Group>,
    /// Gain of the current segment's frames, per group, generated as the
    /// frames are processed (never a whole host-block-mode capacity ahead).
    curve: Vec<f32>,
    pos: usize,
    frame: u64,
    last_frame: u64,
    /// A model state went non-finite: the fail-safe is latched until reset.
    fault: bool,
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
        // 0: the graph has no reblocker, so the model updates once per host
        // call (calls longer than HOST_MAX are split).
        let dynamic = p.block_frames == 0;
        let block = if dynamic { HOST_MAX } else { p.block_frames as usize };
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
                    target: 1.0,
                    r: 0.0,
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
                    tau_vc: s.tau_vc as f64,
                    tau_mg: s.tau_mg as f64,
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
                    meter_failed: false,
                });
            }
        }
        let ch: Vec<Chan> =
            ch.into_iter().map(|c| c.ok_or(InitError::Invalid("unclaimed channel"))).collect::<Result<_, _>>()?;
        let curve = vec![1.0; groups.len() * block];
        Ok(Run { n, block, dynamic, rate, ch, groups, curve, pos: 0, frame: 0, last_frame: 0, fault: false })
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
            c.meter_failed = false;
        }
        for g in self.groups.iter_mut() {
            g.gain = 1.0;
            g.target = 1.0;
            g.r = 0.0;
            g.hard = false;
        }
        self.pos = 0;
        self.frame = 0;
        self.last_frame = 0;
        self.fault = false;
    }

    /// Returns whether any input sample was non-finite (the meter failure:
    /// those frames get at most the fail-safe gain).
    fn segment(&mut self, io: &mut Io<'_, '_>, off: usize, len: usize) -> bool {
        let phase = self.pos;
        let mut failed = false;
        // per group: the smoother toward this block's target, frame by frame
        for (gi, g) in self.groups.iter_mut().enumerate() {
            let curve = &mut self.curve[gi * self.block..][..len];
            let (t, r) = (g.target, g.r);
            let mut v = g.gain;
            for c in curve.iter_mut() {
                v = t + r * (v - t);
                *c = v;
            }
            g.gain = v;
            if g.hard {
                curve.iter_mut().for_each(|c| *c *= FAILSAFE_GAIN);
            }
        }
        for (c, ch) in self.ch.iter_mut().enumerate() {
            let inp = &io.input[c][off..off + len];
            let out = &mut io.output[c][off..off + len];
            let curve = &self.curve[ch.group * self.block..][..len];
            // the unprotected and the protected drive as the meters see them
            let mut k = 0;
            while k < len {
                let m = (len - k).min(BOUND_CHUNK);
                let mut x = [0.0f32; BOUND_CHUNK];
                x[..m].copy_from_slice(&inp[k..k + m]);
                dsp::meter(&mut x[..m]);
                ch.acc_in.add_sq(&x[..m], phase + k);
                k += m;
            }
            // meter failure: at most the fail-safe gain from the first
            // non-finite sample to the end of its block, whatever the host
            // slicing
            let bad = dsp::mul_bounded_to(inp, curve, out) || ch.meter_failed;
            let from = if ch.meter_failed { 0 } else { inp.iter().position(|x| !x.is_finite()).unwrap_or(len) };
            if bad {
                ch.meter_failed = true;
                for ((o, &x), &g) in out[from..].iter_mut().zip(&inp[from..]).zip(&curve[from..]) {
                    *o = dsp::bound1(x) * g.min(FAILSAFE_GAIN);
                }
                failed = true;
            }
            // P covers the protected drive x[n]
            let mut k = 0;
            while k < len {
                let m = (len - k).min(BOUND_CHUNK);
                let mut d = [0.0f32; BOUND_CHUNK];
                dsp::mul_metered_to(&inp[k..k + m], &curve[k..k + m], &mut d[..m]);
                if bad {
                    for (j, dv) in d[..m].iter_mut().enumerate() {
                        if k + j >= from {
                            *dv = dsp::meter1(inp[k + j]) * curve[k + j].min(FAILSAFE_GAIN);
                        }
                    }
                }
                ch.acc.add_sq(&d[..m], phase + k);
                k += m;
            }
        }
        self.frame += len as u64;
        failed
    }

    fn end_block(&mut self) {
        let blk = self.pos as f64;
        if self.dynamic {
            let dt = blk / self.rate;
            for c in self.ch.iter_mut() {
                c.a_vc = dt / (c.tau_vc + dt);
                c.a_mg = dt / (c.tau_mg + dt);
            }
        }
        let mut tgt = [1.0f64; MAX_CHANNELS];
        let mut hard = [false; MAX_CHANNELS];
        for c in self.ch.iter_mut() {
            c.meter_failed = false;
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
            c.g_target = if allow.is_nan() || p_drive.is_nan() {
                f64::NAN
            } else {
                libm::sqrt(allow.max(0.0) / p_drive.max(EPS)).min(1.0)
            };
            c.hard = c.ambient + c.theta_vc > c.hard_limit;
            tgt[c.group] =
                if tgt[c.group].is_nan() || c.g_target.is_nan() { f64::NAN } else { tgt[c.group].min(c.g_target) };
            hard[c.group] |= c.hard;
        }
        // any non-finite model or controller state latches the fail-safe
        if !self.ch.iter().all(|c| c.theta_vc.is_finite() && c.theta_mg.is_finite() && c.g_target.is_finite()) {
            self.fault = true;
        }
        for (gi, g) in self.groups.iter_mut().enumerate() {
            let t = tgt[gi] as f32;
            g.r = if t < g.gain { g.r_attack } else { g.r_release };
            g.target = t;
            if !(g.gain.is_finite() && t.is_finite()) {
                self.fault = true;
            }
            g.hard = hard[gi];
        }
        self.last_frame = self.frame - self.pos as u64;
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
        let mut health = 0;
        let mut off = 0;
        while off < frames {
            let len = (run.block - run.pos).min(frames - off);
            if run.segment(&mut io, off, len) {
                health |= Health::FAILSAFE;
            }
            off += len;
            run.pos += len;
            if run.pos == run.block {
                run.end_block();
                run.pos = 0;
                if run.fault {
                    crate::failsafe::fill_from(&mut io, off, FAILSAFE_GAIN);
                    return Health(Health::FAILSAFE | Health::MODEL_FAULT);
                }
            }
        }
        if run.dynamic && run.pos > 0 {
            run.end_block();
            run.pos = 0;
        }
        Health(health)
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
        if run.fault {
            out.health = Health(Health::FAILSAFE | Health::MODEL_FAULT);
        }
        for (i, c) in run.ch.iter().enumerate() {
            let g = &run.groups[c.group];
            let mut s = ChannelState::ABSENT;
            s.modeled_power_w = c.p_model as f32;
            s.coil_temp_c = (c.ambient + c.theta_vc) as f32;
            s.magnet_temp_c = (c.ambient + c.theta_mg) as f32;
            s.ol_temp_c = s.coil_temp_c;
            s.axis_gain[4] = c.g_target as f32;
            s.gain = if run.fault {
                FAILSAFE_GAIN
            } else if g.hard {
                g.gain * FAILSAFE_GAIN
            } else {
                g.gain
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
            out.gain[i] = run.groups[c.group].gain as f64;
        }
        true
    }

    fn restore_state(&mut self, state: &HandOff, gap_s: f64) -> bool {
        let rate = self.params.sample_rate;
        let Some(run) = self.run.as_mut() else { return false };
        if !state.fits(UnitKind::Atsp, run.n, rate) {
            return false;
        }
        let t = gap_or_zero(gap_s);
        let mut g = [1.0f64; MAX_CHANNELS];
        let mut hard = [false; MAX_CHANNELS];
        for (i, c) in run.ch.iter_mut().enumerate() {
            (c.theta_vc, c.theta_mg) = cool_coupled(state.node[i][0], state.node[i][1], c.tau_vc, c.tau_mg, t);
            c.hard = c.ambient + c.theta_vc > c.hard_limit;
            g[c.group] = g[c.group].min(state.gain[i]);
            hard[c.group] |= c.hard;
        }
        // the first block holds the restored gain (and the panic)
        for (gi, grp) in run.groups.iter_mut().enumerate() {
            grp.gain = release_toward_unity(g[gi], grp.r_release as f64, t * rate as f64) as f32;
            grp.target = grp.gain;
            grp.r = 0.0;
            grp.hard = hard[gi];
        }
        true
    }
}

#[cfg(test)]
mod fault_tests {
    use super::*;

    fn params() -> Params {
        let s = Slot {
            channel_mask: 0b11,
            gain_link: 1,
            amplifier_sensitivity: 10.0,
            rdc: 8.0,
            rth_vc: 30.0,
            tau_vc: 1.0,
            rth_mg: 40.0,
            tau_mg: 20.0,
            ambient: 40.0,
            temperature_limit: 110.0,
            hard_limit_headroom: 8.0,
            attack_ms: 500.0,
            release_ms: 5000.0,
        };
        Params {
            channels: 2,
            sample_rate: 48_000,
            block_frames: 512,
            slot: [s, Slot::default(), Slot::default()],
            ..Params::default()
        }
    }

    fn run(u: &mut Unit, x: f32, n: usize) -> (Vec<f32>, Health) {
        let input = vec![x; n];
        let ins: [&[f32]; 2] = [&input, &input];
        let mut o0 = vec![0.0f32; n];
        let mut o1 = vec![0.0f32; n];
        let mut outs: [&mut [f32]; 2] = [&mut o0, &mut o1];
        let h = u.process(Io { frames: n, input: &ins, sense: &[], output: &mut outs });
        (o0, h)
    }

    /// Review finding 1: a NaN magnet used to give `allow.max(0)` = 0 and
    /// mute for good, or unity through `min`; it now latches 20 dB.
    #[test]
    fn a_non_finite_model_state_latches_the_failsafe_until_reset() {
        let poisons: [fn(&mut Run); 3] =
            [|r| r.ch[0].theta_mg = f64::NAN, |r| r.ch[1].theta_vc = f64::INFINITY, |r| r.groups[0].gain = f32::NAN];
        for poison in poisons {
            let mut u = Unit::new(params());
            let (o, h) = run(&mut u, 0.01, 512);
            assert_eq!((o[511], h), (0.01, Health::OK));
            poison(u.run.as_mut().unwrap());
            // the block's end latches (here the end of this call)
            let (_, h) = run(&mut u, 0.01, 512);
            assert!(h.0 & Health::MODEL_FAULT != 0 && h.is_failsafe());
            for _ in 0..4 {
                let (o, h) = run(&mut u, 1.0, 300);
                assert!(h.0 & Health::MODEL_FAULT != 0);
                assert!(o.iter().all(|&v| (v - FAILSAFE_GAIN).abs() < 1e-7));
            }
            let mut t = Telemetry::new();
            u.telemetry(&mut t);
            assert!(t.health.0 & Health::MODEL_FAULT != 0 && t.channels[0].gain == FAILSAFE_GAIN);
            // a state the unit no longer trusts is never handed on
            let mut rec = HandOff::EMPTY;
            assert!(!u.save_state(&mut rec));
            u.reset();
            let (o, h) = run(&mut u, 0.01, 512);
            assert_eq!((o[511], h), (0.01, Health::OK));
        }
    }
}
