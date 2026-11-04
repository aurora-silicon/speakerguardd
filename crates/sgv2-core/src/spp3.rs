//! `spp3`: feed-forward thermal protection per channel with linked speaker
//! groups, and (later) closed-loop temperature from the V/I telemetry bus.
//! Implemented from the behaviour specification, sections 3 to 6 and 10:
//!
//! - **Block clock.** The model updates once per `block_frames` frames, the
//!   size of the reblocker in front of the unit in the Mac's graph. Hosts
//!   may call `process` with any frame count: the unit splits each call at
//!   its block boundaries, so its state trajectory does not depend on the
//!   host quantum and no reblocking latency is added.
//! - **Step 2.** The current protected gain is applied to the audio, which
//!   is not delayed (the unit reports latency 0; black-box reference: the
//!   output tracks the input with no delay).
//! - **Step 3.** Modelled dissipated power of the undelayed protected drive
//!   (spec 16): `p = (ag*x)^2 * Reb_ref / (Reb_ref + Rshunt)^2`, the mean
//!   over the block.
//! - **Step 4.** Coupled two-node thermal model, backward Euler over the
//!   block (`dt = block_frames / rate`), the coil target using the magnet
//!   state from before this update.
//! - **Step 6.** Power ceiling:
//!   `g = min(1, sqrt(max(0, (TemperatureLimit - ambient - theta_mg) / T_sett_vc) / max(P_drive, eps)))`.
//! - **Step 7.** Minimum within each `SpeakerGroup`.
//! - **Step 8.** The group gain is a per-sample first-order one-pole in
//!   linear gain toward the block's target (spec 16), attack or release by
//!   the sign of the step, evaluated in closed form per block frame so it
//!   stays split-invariant.
//! - **Hard path.** While `T_OL > TemperatureLimit + HardTempLimitHeadroom`
//!   (decided per block) the group takes the unit's 20 dB at once.
//! - **Fail-safe.** A channel-count mismatch, or a non-finite meter value
//!   (the "simple-meter processing failure"), gives 20 dB for the affected
//!   frames.
//! - **Reset.** Thermal state at the parameters' start state, unity gain,
//!   look-ahead cleared, CL temperature reported as 20 degC until measured.
//!   The spec's reset state is ambient. The reference unit's default start
//!   (AU property 64007 unset) is near the limit instead; which one macOS
//!   uses on a real Mac is TODO(SPEC), so the start rises
//!   (`initial_theta_vc`/`initial_theta_mg`) are parameters the compiler
//!   fills.
//!
//! TODO(SPEC), each isolated so the rig can pin it, none adding margin:
//! - the pole formula (`pole`: `exp(-1/(tau*rate))`);
//! - which samples `P_drive` covers (the block's undelayed input) and `eps`
//!   (the smallest normal f32);
//! - whether the hard path's 20 dB rides the thermal attack (it does not
//!   here: a smoothed panic lets the coil run far past the hard limit);
//! - the role of `Thermal_LookaheadDelay_ms` now that the audio is not
//!   delayed (kept as a parameter, `lookahead_frames`);
//! - how a nonzero `ThermalFFSpeedupFactor` shapes the predictive target,
//!   and the `Thermal_SafeTlim*` state machine (spec section 15: open). A
//!   nonzero factor runs the ceiling above unchanged. A factor of zero is
//!   specified: it holds the feed-forward thermal reduction at unity while
//!   the model keeps integrating (the hard path still acts), and that is
//!   what this unit does;
//! - the closed loop (`TlimAdj`) and its pilot tone: the spec directs a
//!   first release with the closed loop disabled, so channels whose tuning
//!   enables CL thermal feedback run the open-loop kernel (reported as CL
//!   off, `TlimAdj` 0) and no pilot is generated;
//! - displacement control and CPMS power control: unspecified, and off in
//!   every shipped laptop preset. A preset that enables either, or that
//!   disables thermal protection, is refused at `new` and the unit runs the
//!   fail-safe, rather than silently running without what the tuning asked
//!   for.
//!
//! Tuning keys ([`TuningLookup`]) are the sidecar names where the sidecar
//! has the value, otherwise the public AU parameter name with spaces
//! removed: `ThermalProtectionEnabled` (global 0), `EnableCPMSControl`
//! (18), `EnableCPMSClosedLoopControl` (19), `CLThermalFeedbackEnabled`
//! (part 22). TODO(lane A): confirm the reader exposes these.

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::dsp;
use crate::sense::{BlockSums, DelayEstimator};
use crate::{ChannelState, Health, Io, ProtectionUnit, Telemetry, UnitKind, FAILSAFE_GAIN, MAX_CHANNELS};

/// CL temperature reported before any valid measurement (spec section 10).
const T_CL_UNMEASURED: f32 = 20.0;
/// Denominator floor of the ceiling target (TODO(SPEC): see module docs).
const EPS: f64 = f32::MIN_POSITIVE as f64;

/// Per physical channel, in the unit's channel order. Field names follow
/// the sidecar's keys.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChannelParams {
    /// Host program port feeding this unit channel. The permutation comes
    /// from the tuning reader (graph wiring), never from code.
    pub source: u32,
    /// `SpeakerGroup`.
    pub group: u32,
    /// `IgnoreTelemetry` (0 or 1).
    pub ignore_telemetry: u32,
    pub t_sett_vc: f32,
    pub t_sett_mg: f32,
    pub tau_tvc: f32,
    pub tau_tmg: f32,
    pub temperature_limit: f32,
    pub hard_temp_limit_headroom: f32,
    pub reb_ref: f32,
    pub rshunt: f32,
    pub thermal_ff_speedup_factor: f32,
    /// `CLThermalFeedbackEnabled` (0 or 1). Recorded; the closed loop is
    /// not run in this release.
    pub cl_thermal_feedback_enabled: u32,
    pub rampout: f32,
    /// `OL_disp`/`ag`: amplifier volts at digital full scale.
    pub ag: f32,
    pub imon_scale: f32,
    pub vmon_scale: f32,
    pub i_pwr_thres: f32,
    pub v_pwr_thres: f32,
    /// Sense port (index into `Io::sense`) of this channel's current and
    /// voltage, from `SpeakerInputDataSourceOrder`.
    pub sense_i: u32,
    pub sense_v: u32,
    /// Start (and reset) rises over ambient of the coil and magnet, K.
    /// TODO(SPEC): the reference unit's default start state (property 64007).
    pub initial_theta_vc: f32,
    pub initial_theta_mg: f32,
}

/// The spp3 blob payload.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Params {
    pub channels: u32,
    pub sample_rate: u32,
    /// Frames per unit call in the Mac's graph (its reblocker size).
    pub block_frames: u32,
    pub thermal_ambient_temp: f32,
    pub thermal_gain_attack_time_s: f32,
    pub thermal_gain_release_time_s: f32,
    pub thermal_lookahead_delay_ms: f32,
    pub thermal_safe_tlim: f32,
    pub thermal_safe_tlim_offset: f32,
    pub thermal_safe_tlim_time_min: f32,
    pub enable_displacement_control: u32,
    pub enable_cpms_control: u32,
    pub enable_cpms_closed_loop_control: u32,
    pub thermal_protection_enabled: u32,
    pub ch: [ChannelParams; MAX_CHANNELS],
}

// SAFETY: repr(C), only u32/f32 fields and an array of a repr(C) struct of
// u32/f32 fields: 14 + 8 * 23 four-byte words, no padding.
unsafe impl Pod for Params {}
// SAFETY: as above, 23 four-byte words.
unsafe impl Pod for ChannelParams {}

const _: () = assert!(core::mem::size_of::<ChannelParams>() == 23 * 4);
const _: () = assert!(core::mem::size_of::<Params>() == (14 + MAX_CHANNELS * 23) * 4);

impl Default for Params {
    fn default() -> Self {
        Params {
            channels: 0,
            sample_rate: 0,
            block_frames: 0,
            thermal_ambient_temp: 0.0,
            thermal_gain_attack_time_s: 0.0,
            thermal_gain_release_time_s: 0.0,
            thermal_lookahead_delay_ms: 0.0,
            thermal_safe_tlim: 0.0,
            thermal_safe_tlim_offset: 0.0,
            thermal_safe_tlim_time_min: 0.0,
            enable_displacement_control: 0,
            enable_cpms_control: 0,
            enable_cpms_closed_loop_control: 0,
            thermal_protection_enabled: 0,
            ch: [ChannelParams::default(); MAX_CHANNELS],
        }
    }
}

/// What the tuning reader (lane A) provides to build [`Params`]: a lookup by
/// sidecar key, globally or per unit channel, plus the graph-derived
/// channel permutation, sense order and block size.
pub trait TuningLookup {
    fn channels(&self) -> usize;
    fn sample_rate(&self) -> Option<u32>;
    fn block_frames(&self) -> Option<u32>;
    fn global(&self, key: &str) -> Option<f64>;
    fn channel(&self, ch: usize, key: &str) -> Option<f64>;
    /// Unit channel -> host program port.
    fn source(&self, ch: usize) -> Option<u32>;
    /// Unit channel -> (current, voltage) sense port.
    fn sense(&self, ch: usize) -> Option<(u32, u32)>;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InitError {
    Blob(BlobError),
    Invalid(&'static str),
    /// The tuning enables a controller this implementation does not have.
    Unsupported(&'static str),
}

impl Params {
    /// Build from the tuning reader. Every essential key must be present
    /// and finite; a missing one is an essential-parse failure (fail-safe),
    /// never a default.
    pub fn from_lookup(t: &dyn TuningLookup) -> Result<Self, InitError> {
        let n = t.channels();
        if n == 0 || n > MAX_CHANNELS {
            return Err(InitError::Invalid("channel count"));
        }
        let g = |k: &'static str| t.global(k).filter(|v| v.is_finite()).map(|v| v as f32).ok_or(InitError::Invalid(k));
        let flag = |k: &'static str| -> Result<u32, InitError> { Ok((g(k)? != 0.0) as u32) };
        let mut p = Params {
            channels: n as u32,
            sample_rate: t.sample_rate().ok_or(InitError::Invalid("sample rate"))?,
            block_frames: t.block_frames().ok_or(InitError::Invalid("block frames"))?,
            thermal_ambient_temp: g("Thermal_AmbientTemp")?,
            thermal_gain_attack_time_s: g("Thermal_GainAttackTime_s")?,
            thermal_gain_release_time_s: g("Thermal_GainReleaseTime_s")?,
            thermal_lookahead_delay_ms: g("Thermal_LookaheadDelay_ms")?,
            thermal_safe_tlim: g("Thermal_SafeTlim")?,
            thermal_safe_tlim_offset: g("Thermal_SafeTlimOffset")?,
            thermal_safe_tlim_time_min: g("Thermal_SafeTlimTimeMin")?,
            enable_displacement_control: flag("EnableDisplacementControl")?,
            enable_cpms_control: flag("EnableCPMSControl")?,
            enable_cpms_closed_loop_control: flag("EnableCPMSClosedLoopControl")?,
            thermal_protection_enabled: flag("ThermalProtectionEnabled")?,
            ch: [ChannelParams::default(); MAX_CHANNELS],
        };
        for c in 0..n {
            let f = |k: &'static str| {
                t.channel(c, k).filter(|v| v.is_finite()).map(|v| v as f32).ok_or(InitError::Invalid(k))
            };
            let (si, sv) = t.sense(c).ok_or(InitError::Invalid("sense order"))?;
            p.ch[c] = ChannelParams {
                source: t.source(c).ok_or(InitError::Invalid("channel source"))?,
                group: f("SpeakerGroup")? as u32,
                // Nonessential: a malformed ignore list is ignored, so
                // telemetry is honoured (spec section 10).
                ignore_telemetry: t.channel(c, "IgnoreTelemetry").map_or(0, |v| (v != 0.0) as u32),
                t_sett_vc: f("T_sett_vc")?,
                t_sett_mg: f("T_sett_mg")?,
                tau_tvc: f("tau_Tvc")?,
                tau_tmg: f("tau_Tmg")?,
                temperature_limit: f("TemperatureLimit")?,
                hard_temp_limit_headroom: f("HardTempLimitHeadroom")?,
                reb_ref: f("Reb_ref")?,
                rshunt: f("Rshunt")?,
                thermal_ff_speedup_factor: f("ThermalFFSpeedupFactor")?,
                cl_thermal_feedback_enabled: (f("CLThermalFeedbackEnabled")? != 0.0) as u32,
                rampout: f("Rampout")?,
                ag: f("ag")?,
                imon_scale: f("IMONScale")?,
                vmon_scale: f("VMONScale")?,
                i_pwr_thres: f("IPwrThres")?,
                v_pwr_thres: f("VPwrThres")?,
                sense_i: si,
                sense_v: sv,
                // The spec's reset state is ambient (zero rises). TODO(SPEC):
                // the start state the reference unit uses on a real Mac.
                initial_theta_vc: t.channel(c, "sgv2.InitialThetaVc").unwrap_or(0.0) as f32,
                initial_theta_mg: t.channel(c, "sgv2.InitialThetaMg").unwrap_or(0.0) as f32,
            };
        }
        p.validate()?;
        Ok(p)
    }

    /// Structural checks only: the model trusts the tuning's values.
    pub fn validate(&self) -> Result<(), InitError> {
        let n = self.channels as usize;
        if n == 0 || n > MAX_CHANNELS {
            return Err(InitError::Invalid("channel count"));
        }
        if self.sample_rate == 0 {
            return Err(InitError::Invalid("sample rate"));
        }
        if self.block_frames == 0 || self.block_frames > 16384 {
            return Err(InitError::Invalid("block frames"));
        }
        let pos = |v: f32| v.is_finite() && v > 0.0;
        if !(pos(self.thermal_gain_attack_time_s) && pos(self.thermal_gain_release_time_s)) {
            return Err(InitError::Invalid("gain attack/release"));
        }
        let look = self.thermal_lookahead_delay_ms;
        if !(look.is_finite() && look >= 0.0 && look * 1e-3 * self.sample_rate as f32 <= 16384.0) {
            return Err(InitError::Invalid("look-ahead"));
        }
        if !self.thermal_ambient_temp.is_finite() {
            return Err(InitError::Invalid("ambient"));
        }
        let mut seen = 0u32;
        for c in &self.ch[..n] {
            if c.source as usize >= n || seen & (1 << c.source) != 0 {
                return Err(InitError::Invalid("channel source"));
            }
            seen |= 1 << c.source;
            if !(pos(c.tau_tvc) && pos(c.tau_tmg) && pos(c.t_sett_vc) && c.t_sett_mg.is_finite() && c.t_sett_mg >= 0.0)
            {
                return Err(InitError::Invalid("thermal constants"));
            }
            if !(c.initial_theta_vc.is_finite() && c.initial_theta_mg.is_finite()) {
                return Err(InitError::Invalid("start state"));
            }
            if c.rampout != 0.0 {
                // TODO(SPEC): Rampout's nonzero behaviour (some AIDs set it).
                return Err(InitError::Invalid("Rampout"));
            }
            if !(c.temperature_limit.is_finite() && c.hard_temp_limit_headroom.is_finite()) {
                return Err(InitError::Invalid("temperature limit"));
            }
            if !(pos(c.reb_ref) && c.rshunt.is_finite() && c.rshunt >= 0.0 && pos(c.ag)) {
                return Err(InitError::Invalid("electrical constants"));
            }
        }
        Ok(())
    }

    /// Decode and check a complete blob.
    pub fn from_blob(bytes: &[u8]) -> Result<Self, InitError> {
        let (h, payload) = crate::blob::Header::parse(bytes).map_err(InitError::Blob)?;
        if h.unit != UnitKind::Spp3 as u32 {
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

/// One-pole smoothing coefficient for time constant `tau` at step `dt`
/// Per-sample one-pole pole for time constant `tau` at `rate` (spec 16:
/// a first-order one-pole in linear gain, coefficients from the named
/// seconds and the sample rate). TODO(SPEC): the exact coefficient formula;
/// this is the exponential form.
pub fn pole(rate: f64, tau: f64) -> f64 {
    libm::exp(-1.0 / (tau * rate))
}

fn powers(r: f64, n: usize) -> Vec<f32> {
    let mut v = Vec::with_capacity(n);
    let mut p = 1.0f64;
    for _ in 0..n {
        p *= r;
        v.push(p as f32);
    }
    v
}

struct Chan {
    init_vc: f64,
    init_mg: f64,
    src: usize,
    group: usize,
    /// ag^2 * Reb_ref / (Reb_ref + Rshunt)^2: watts per FS^2.
    elec: f64,
    /// TemperatureLimit - ambient.
    span: f64,
    t_sett_vc: f64,
    t_sett_mg: f64,
    a_vc: f64,
    a_mg: f64,
    hard_limit: f64,
    /// `ThermalFFSpeedupFactor != 0`: the feed-forward reduction acts.
    ff_active: bool,
    sense: Option<(usize, usize)>,
    vi_scale: f64,
    // block accumulators (split-invariant)
    acc_out: dsp::LaneAcc,
    acc_in: dsp::LaneAcc,
    acc_i: dsp::LaneAcc,
    acc_v: dsp::LaneAcc,
    acc_vi: dsp::LaneAcc,
    // state
    theta_vc: f64,
    theta_mg: f64,
    p_model: f64,
    g_target: f64,
    hard: bool,
    applied: f32,
    p_meas: f32,
    vi_power: f64,
}

impl Chan {
    fn clear_acc(&mut self) {
        self.acc_out.clear();
        self.acc_in.clear();
        self.acc_i.clear();
        self.acc_v.clear();
        self.acc_vi.clear();
    }
}

pub struct Unit {
    params: Params,
    /// `None` when the parameters could not be used: the unit is then the
    /// fail-safe.
    run: Option<Run>,
}

struct Run {
    n: usize,
    block: usize,
    look: usize,
    ambient: f64,
    /// `pole^(n+1)` for block frame n, attack and release.
    pow_attack: Vec<f32>,
    pow_release: Vec<f32>,
    ch: Vec<Chan>,
    groups: usize,
    /// Smoothed gain per group at the start of the current block.
    group_gain: [f32; MAX_CHANNELS],
    /// The gain of every frame of the current block, per group.
    curve: Vec<f32>,
    hard_group: [bool; MAX_CHANNELS],
    /// Frames into the current block.
    pos: usize,
    /// Frames processed since reset.
    frame: u64,
    /// First frame of the last completed block.
    last_block_frame: u64,
    sums: BlockSums,
    sense_ok: bool,
    estimator: DelayEstimator,
}

impl Unit {
    /// Build the unit. Allocation happens here, never in `process`.
    /// Unusable parameters give a unit that is the fail-safe; `init_error`
    /// says why.
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

    /// In-phase V/I power of unit channel `c` over the last block, W, with
    /// the tuning's IMON/VMON scales (NaN without complete sense). For the
    /// supervisor's plausibility checks; not a macOS meter.
    pub fn sense_power(&self, c: usize) -> f64 {
        self.run.as_ref().and_then(|r| r.ch.get(c)).map_or(f64::NAN, |c| c.vi_power)
    }

    /// The model path's look-ahead, frames (audio is not delayed).
    pub fn lookahead_frames(&self) -> usize {
        self.run.as_ref().map_or(0, |r| r.look)
    }

    /// The accepted program-to-sense delay, in frames.
    pub fn sense_delay(&self) -> Option<u32> {
        self.run.as_ref().and_then(|r| r.estimator.delay())
    }
}

impl Run {
    fn new(p: &Params) -> Result<Run, InitError> {
        p.validate()?;
        if p.enable_displacement_control != 0 {
            return Err(InitError::Unsupported("EnableDisplacementControl"));
        }
        if p.enable_cpms_control != 0 {
            return Err(InitError::Unsupported("EnableCPMSControl"));
        }
        if p.enable_cpms_closed_loop_control != 0 {
            return Err(InitError::Unsupported("EnableCPMSClosedLoopControl"));
        }
        if p.thermal_protection_enabled == 0 {
            return Err(InitError::Unsupported("thermal protection disabled"));
        }
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let block = p.block_frames as usize;
        let dt = block as f64 / rate;
        let look = libm::round(p.thermal_lookahead_delay_ms as f64 * 1e-3 * rate) as usize;
        let ambient = p.thermal_ambient_temp as f64;
        let mut ids = [u32::MAX; MAX_CHANNELS];
        let mut groups = 0;
        let mut ch = Vec::with_capacity(n);
        for c in &p.ch[..n] {
            let gi = match ids[..groups].iter().position(|&g| g == c.group) {
                Some(i) => i,
                None => {
                    ids[groups] = c.group;
                    groups += 1;
                    groups - 1
                }
            };
            let (reb, rs, ag) = (c.reb_ref as f64, c.rshunt as f64, c.ag as f64);
            ch.push(Chan {
                src: c.source as usize,
                group: gi,
                elec: ag * ag * reb / ((reb + rs) * (reb + rs)),
                span: c.temperature_limit as f64 - ambient,
                t_sett_vc: c.t_sett_vc as f64,
                t_sett_mg: c.t_sett_mg as f64,
                a_vc: dt / (c.tau_tvc as f64 + dt),
                a_mg: dt / (c.tau_tmg as f64 + dt),
                hard_limit: c.temperature_limit as f64 + c.hard_temp_limit_headroom as f64,
                ff_active: c.thermal_ff_speedup_factor != 0.0,
                sense: if c.ignore_telemetry != 0 { None } else { Some((c.sense_i as usize, c.sense_v as usize)) },
                vi_scale: c.imon_scale as f64 * c.vmon_scale as f64,
                acc_out: dsp::LaneAcc::new(),
                acc_in: dsp::LaneAcc::new(),
                acc_i: dsp::LaneAcc::new(),
                acc_v: dsp::LaneAcc::new(),
                acc_vi: dsp::LaneAcc::new(),
                init_vc: c.initial_theta_vc as f64,
                init_mg: c.initial_theta_mg as f64,
                theta_vc: c.initial_theta_vc as f64,
                theta_mg: c.initial_theta_mg as f64,
                p_model: 0.0,
                g_target: 1.0,
                hard: false,
                applied: 1.0,
                p_meas: 0.0,
                vi_power: f64::NAN,
            });
        }
        let mut sums = BlockSums::default();
        sums.clear();
        Ok(Run {
            n,
            block,
            look,
            ambient,
            pow_attack: powers(pole(rate, p.thermal_gain_attack_time_s as f64), block),
            pow_release: powers(pole(rate, p.thermal_gain_release_time_s as f64), block),
            ch,
            groups,
            group_gain: [1.0; MAX_CHANNELS],
            curve: vec![1.0; groups * block],
            hard_group: [false; MAX_CHANNELS],
            pos: 0,
            frame: 0,
            last_block_frame: 0,
            sums,
            sense_ok: false,
            estimator: DelayEstimator::new(),
        })
    }

    fn reset(&mut self) {
        for c in self.ch.iter_mut() {
            c.clear_acc();
            c.theta_vc = c.init_vc;
            c.theta_mg = c.init_mg;
            c.p_model = 0.0;
            c.g_target = 1.0;
            c.hard = false;
            c.applied = 1.0;
            c.p_meas = 0.0;
            c.vi_power = f64::NAN;
        }
        self.group_gain = [1.0; MAX_CHANNELS];
        self.curve.iter_mut().for_each(|g| *g = 1.0);
        self.hard_group = [false; MAX_CHANNELS];
        self.pos = 0;
        self.frame = 0;
        self.last_block_frame = 0;
        self.sums.clear();
        self.sense_ok = false;
        self.estimator.reset();
    }

    /// One segment inside a block: frames `off..off+len` of the host
    /// buffers, starting at block frame `self.pos`.
    fn segment(&mut self, io: &mut Io<'_, '_>, off: usize, len: usize, health: &mut u32) {
        let phase = self.pos;
        for c in self.ch.iter_mut() {
            let inp = &io.input[c.src][off..off + len];
            let out = &mut io.output[c.src][off..off + len];
            let mut acc = c.acc_in;
            acc.add_sq(inp, phase);
            if !acc.is_finite() {
                // only finite samples count as drive (they are muted below)
                acc = c.acc_in;
                for (k, &v) in inp.iter().enumerate() {
                    let v = if v.is_finite() { v } else { 0.0 };
                    acc.add_sq(&[v], phase + k);
                }
            }
            c.acc_in = acc;
            // step 2: the sample-smoothed group gain on the audio
            let curve = &self.curve[c.group * self.block + phase..][..len];
            dsp::mul_to(inp, curve, out);
            if dsp::peak_abs(out) == f32::INFINITY {
                // Meter failure on these frames: at most 20 dB below the
                // input, and the model sees the drive that actually leaves
                // (non-finite samples muted).
                for ((o, &x), &g) in out.iter_mut().zip(inp).zip(curve) {
                    *o = if x.is_finite() { x * g.min(FAILSAFE_GAIN) } else { 0.0 };
                }
                *health |= Health::FAILSAFE;
            }
            // step 3 input: the undelayed protected drive, as it leaves
            c.acc_out.add_sq(out, phase);
        }
        // Sense statistics over the same frames.
        let sense = io.sense;
        if sense.is_empty() {
            self.sums.complete = false;
        }
        for c in self.ch.iter_mut() {
            let Some((si, sv)) = c.sense else { continue };
            match (sense.get(si), sense.get(sv)) {
                (Some(i), Some(v)) if i.len() >= off + len && v.len() >= off + len => {
                    let (i, v) = (&i[off..off + len], &v[off..off + len]);
                    c.acc_i.add_sq(i, phase);
                    c.acc_v.add_sq(v, phase);
                    c.acc_vi.add_prod(v, i, phase);
                }
                _ => self.sums.complete = false,
            }
        }
        self.sums.frames += len;
        if self.sums.complete {
            // Energy envelopes for the delay estimator: protected output
            // against sensed voltage, each summed over channels, in runs that
            // end on the estimator's envelope points.
            let mut k = off;
            while k < off + len {
                let m = self.estimator.room().min(off + len - k);
                let mut oe = 0.0f64;
                let mut ve = 0.0f64;
                for c in self.ch.iter() {
                    oe += dsp::sum_sq(&io.output[c.src][k..k + m]);
                    if let Some((_, sv)) = c.sense {
                        ve += dsp::sum_sq(&sense[sv][k..k + m]);
                    }
                }
                self.estimator.push_sums(oe as f32, ve as f32, m);
                k += m;
            }
        }
        self.frame += len as u64;
    }

    fn end_block(&mut self) {
        let blk = self.block as f64;
        for c in self.ch.iter_mut() {
            // step 3
            let p = c.elec * c.acc_out.total() / blk;
            c.p_model = p;
            // step 4
            let old_mg = c.theta_mg;
            c.theta_vc += c.a_vc * (old_mg + p * c.t_sett_vc - c.theta_vc);
            c.theta_mg += c.a_mg * (p * c.t_sett_mg - c.theta_mg);
            // step 6
            let p_drive = c.elec * c.acc_in.total() / blk;
            let ceiling = (c.span - c.theta_mg) / c.t_sett_vc;
            c.g_target = if c.ff_active { libm::sqrt(ceiling.max(0.0) / p_drive.max(EPS)).min(1.0) } else { 1.0 };
            c.hard = self.ambient + c.theta_vc > c.hard_limit;
        }
        // step 7: the group minimum
        let mut tgt = [1.0f64; MAX_CHANNELS];
        let mut hard = [false; MAX_CHANNELS];
        for c in &self.ch {
            tgt[c.group] = tgt[c.group].min(c.g_target);
            hard[c.group] |= c.hard;
        }
        // step 8: per-sample one-pole toward the target over the next block,
        // in closed form g[n] = t + (g0 - t) * pole^(n+1)
        for gi in 0..self.groups {
            let g0 = self.group_gain[gi];
            let t = tgt[gi] as f32;
            let pw = if t < g0 { &self.pow_attack } else { &self.pow_release };
            let curve = &mut self.curve[gi * self.block..][..self.block];
            for (g, &r) in curve.iter_mut().zip(pw.iter()) {
                *g = t + (g0 - t) * r;
            }
            self.group_gain[gi] = curve[self.block - 1];
            // The hard path: decided at the controller cadence and applied
            // at once as the unit's 20 dB on the applied gain, not through
            // the 4.6 s attack (a smoothed panic lets the coil run far past
            // the hard limit). TODO(SPEC): spec 16 says the panic gain is
            // "sample-smoothed/applied by the common gain path"; confirm it
            // is not meant to ride the thermal attack.
            if hard[gi] {
                curve.iter_mut().for_each(|g| *g *= FAILSAFE_GAIN);
            }
            self.hard_group[gi] = hard[gi];
        }
        let mut complete = self.sums.complete && self.sums.frames == self.block;
        for c in self.ch.iter() {
            if c.sense.is_some() && !(c.acc_i.is_finite() && c.acc_v.is_finite() && c.acc_vi.is_finite()) {
                complete = false;
            }
        }
        for c in self.ch.iter_mut() {
            let g = self.group_gain[c.group];
            c.applied = if self.hard_group[c.group] { g * FAILSAFE_GAIN } else { g };
            // Measured in-phase V/I power, reported only (the closed loop
            // is off). TODO(SPEC): whether the meter is mean(i^2)*Rdc or
            // the in-phase product.
            // Without a calibration overlay the reference unit ignores
            // telemetry and meters 0 W; this release has no calibration
            // path, so it meters 0 W too. The in-phase V/I power stays
            // available through `sense_power`.
            c.p_meas = 0.0;
            c.vi_power = if c.sense.is_some() && complete { c.acc_vi.total() * c.vi_scale / blk } else { f64::NAN };
            c.clear_acc();
        }
        self.sense_ok = complete;
        self.estimator.end_block();
        self.sums.clear();
        self.last_block_frame = self.frame - self.block as u64;
    }
}

impl ProtectionUnit for Unit {
    fn kind(&self) -> UnitKind {
        UnitKind::Spp3
    }

    fn latency_frames(&self) -> u32 {
        // The look-ahead is in the model path; audio is not delayed.
        0
    }

    fn process(&mut self, mut io: Io<'_, '_>) -> Health {
        let frames = io.frames;
        let Some(run) = self.run.as_mut() else {
            failsafe_io(&mut io);
            return Health(Health::FAILSAFE | Health::NO_PARAMS);
        };
        // The initialised channel count must match the audio.
        let fits = |b: &[f32]| b.len() >= frames;
        if io.input.len() != run.n
            || io.output.len() != run.n
            || !io.input.iter().all(|b| fits(b))
            || !io.output.iter().all(|b| fits(b))
        {
            failsafe_io(&mut io);
            return Health(Health::FAILSAFE | Health::CHANNEL_MISMATCH);
        }
        let mut health = 0u32;
        let mut off = 0;
        while off < frames {
            let len = (run.block - run.pos).min(frames - off);
            run.segment(&mut io, off, len, &mut health);
            off += len;
            run.pos += len;
            if run.pos == run.block {
                run.end_block();
                run.pos = 0;
            }
        }
        if !run.sense_ok {
            health |= Health::OPEN_LOOP;
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
        out.frame = run.last_block_frame;
        out.health = Health(if run.sense_ok { 0 } else { Health::OPEN_LOOP });
        for c in run.ch.iter() {
            let t_vc = (run.ambient + c.theta_vc) as f32;
            let mut s = ChannelState::ABSENT;
            s.modeled_power_w = c.p_model as f32;
            s.measured_power_w = c.p_meas;
            s.coil_temp_c = t_vc;
            s.magnet_temp_c = (run.ambient + c.theta_mg) as f32;
            s.ol_temp_c = t_vc;
            s.cl_temp_c = T_CL_UNMEASURED;
            s.tlim_adj_c = 0.0;
            s.closed_loop = false;
            // axis 4 is temperature
            s.axis_gain[4] = c.g_target as f32;
            s.gain = c.applied;
            out.channels[c.src] = s;
        }
    }

    fn reset(&mut self) {
        if let Some(r) = self.run.as_mut() {
            r.reset();
        }
    }
}

/// 20 dB on whatever geometry the host gave, never indexing past a buffer.
pub(crate) fn failsafe_io(io: &mut Io<'_, '_>) {
    let frames = io.frames;
    for (c, out) in io.output.iter_mut().enumerate() {
        let n = frames.min(out.len());
        let out = &mut out[..n];
        match io.input.get(c) {
            Some(inp) => {
                let m = n.min(inp.len());
                dsp::scale_to(&inp[..m], FAILSAFE_GAIN, &mut out[..m]);
                dsp::sanitize(&mut out[..m]);
                out[m..].fill(0.0);
            }
            None => out.fill(0.0),
        }
    }
}
