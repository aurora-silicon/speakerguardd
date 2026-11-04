//! `lsm2`: signal-only loudspeaker model and five-axis constraint
//! controller (MacBook Neo, MacBook Air M4/M5), from the behaviour
//! specification. This is the model-only configuration the Neo ships
//! (static model, system ID bypassed, no closed-loop temperature); a tuning
//! that asks for the DSP estimator or closed-loop temperature is refused.
//!
//! Implemented as specified:
//! - **Latency.** Audio is delayed by two filterbank stages plus the speaker
//!   response look-ahead (`2*FilterbankLatency + SpeakerResponseLookAhead`),
//!   and the unit reports it. State persists across host calls of any size.
//! - **Model.** Per channel `u = V_FS * x`, and stable digital realisations
//!   (bilinear transform) of the one-resonator transfer functions
//!   `I/U = 1/(R + Bl^2/Zm)`, `X/U = Bl/(R (Mms s^2 + Rms s + Kms) + Bl^2 s)`
//!   and `V = sX`, with `R = Rdc + Rpar` and `Zm = Mms s + Rms + Kms/s`. The
//!   current uses the admittance block's mechanics; displacement and
//!   velocity use the displacement block's.
//! - **Coil power and thermal model.** `Pcoil = i^2 Rdc` and the two-node
//!   recurrence per sample: `Tvc += avc (Tmag + P Rth_vc - Tvc)`,
//!   `Tmag += amag (Tamb + P Rth_mag - Tmag)`.
//! - **Candidates**, linear in [0, 1], a disabled axis giving 1:
//!   - sample peak and displacement: `min(1, limit / max|q|)`;
//!   - power: four one-pole averages, `min(1, sqrt(budget_j / Pbar_j))`, the
//!     minimum, floored at `-MaxPowerAttenuation`;
//!   - velocity: `min(1, limit / max|v|)`, floored at
//!     `-MaxVelocityAttenuation`;
//!   - temperature: floored at `-MaxTemperatureAttenuation`.
//! - **Combination.** Fast gain `min(peak, displacement, velocity)`, slow gain
//!   `min(power, temperature)`, slow gains linked across channels when
//!   `LinkSlowGainsAcrossChannels`; the applied gain is the minimum of the
//!   smoothed fast and slow gains. Downward moves use `AttackTime`; recovery
//!   uses `FastReleaseTime` (fast) and `SlowReleaseTime` (slow).
//!   `EnableMitigation = 0` keeps the modelling and latency and attenuates
//!   nothing on macOS; here it is refused (fail-safe), as is a tuning with
//!   neither thermal axis (power or temperature) enabled: either would run
//!   the speakers unprotected while health reports OK.
//! - **Latency floor.** The controller decides every 256 frames over the
//!   samples still in the delay line, so a latency below 256 frames lets a
//!   transient leave before any decision has seen it; such a tuning is
//!   refused. (The Neo's is 352.)
//! - **Controller cadence.** Candidates update every 256 frames over the
//!   samples still in the delay line (everything that has not yet left).
//!
//! Full scale: the program is held to digital full scale (`clamp(x, -1,
//! 1)`, a non-finite sample 0) before the model and on the output, so the
//! model sees the drive the speaker can get and a huge finite sample cannot
//! leave it hot for minutes. A model state that goes non-finite latches the
//! 20 dB fail-safe (`Health::MODEL_FAULT`) until reset.
//!
//! Failure: the reference unit fails initialisation outright when neither
//! estimator can be built. Here an unusable parameter set, or a geometry
//! mismatch, gives the 20 dB fail-safe, which on the Neo equals the
//! kernel's own floor.
//!
//! TODO(SPEC), each a stand-in that is never less restrictive than the
//! specification permits (spec 6.1), none a margin on a specified quantity:
//! - **Filterbank.** The band split is not specified, so displacement and
//!   velocity act broadband. A broadband gain at the band candidate's value
//!   reduces every band at least as much as the band gain would.
//!   `VelocityIgnoreLowBand/HighBand` therefore have no effect yet.
//! - **Velocity masking.** The 8-12 kHz detector and knot curve are not
//!   specified; the limit is the lowest knot, `VelocityLimit0` (unmasked).
//! - **Temperature knee.** Not specified. The stand-in applies the full
//!   `MaxTemperatureAttenuation` once the predicted coil temperature
//!   (`Tvc + TemperatureHeadroom`) reaches the soft limit, unity below.
//! - **One-pole convention** (`pole`): `exp(-1/(tau*rate))`, per sample.
//! - **Amplifier input power.** The budgets apply to the low-rail affine
//!   model `idle + ratio * max(0, u*i)`; whether the load term is `u*i` or
//!   coil power is not specified. Mid/high rails, battery input, CPMS and
//!   soft knees below 100 % are refused.
//! - **Property 6632** (flow model): not applied, the preset's model is
//!   used (spec: nonfatal, not graph-conformant).

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::delay::Delay;
use crate::{dsp, ChannelState, Health, Io, ProtectionUnit, Telemetry, UnitKind, FAILSAFE_GAIN, MAX_CHANNELS};

/// Controller update cadence in frames (spec 10: exactly 256).
pub const CONTROLLER_FRAMES: usize = 256;
const EPS: f64 = f32::MIN_POSITIVE as f64;

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChannelParams {
    /// Host program port feeding this channel.
    pub source: u32,
    pub volts_full_scale: f32,
    pub parasitic_resistance: f32,
    /// Admittance (current) model of the channel's speaker type.
    pub rdc: f32,
    pub bl: f32,
    pub mms_kg: f32,
    pub rms: f32,
    pub kms: f32,
    /// Displacement/velocity model.
    pub disp_bl: f32,
    pub disp_mms_kg: f32,
    pub disp_rms: f32,
    pub disp_kms: f32,
    /// Thermal type block.
    pub rth_vc: f32,
    pub tau_vc: f32,
    pub rth_mag: f32,
    pub tau_mag: f32,
    pub ambient: f32,
    /// Limits.
    pub amplitude_limit: f32,
    pub displacement_limit: f32,
    pub velocity_limit0: f32,
    pub max_velocity_attenuation_db: f32,
    pub temperature_soft_limit: f32,
    pub temperature_hard_limit: f32,
    pub temperature_headroom: f32,
    pub max_temperature_attenuation_db: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Params {
    pub channels: u32,
    pub sample_rate: u32,
    pub enable_mitigation: u32,
    pub control_sample_peak: u32,
    pub control_displacement: u32,
    pub control_velocity: u32,
    pub control_power: u32,
    pub control_temperature: u32,
    pub link_slow_gains: u32,
    pub unlink_temperature_gains: u32,
    /// Model-only operation: `BypassSysId = 1` and
    /// `EnableClosedLoopTemperature = 0`.
    pub model_only: u32,
    pub attack_time_ms: f32,
    pub fast_release_time_ms: f32,
    pub slow_release_time_ms: f32,
    pub filterbank_latency_ms: f32,
    pub speaker_response_lookahead_ms: f32,
    pub power_time_scale_s: [f32; 4],
    pub power_hard_limit_w: [f32; 4],
    pub power_soft_limit_pct: [f32; 4],
    pub max_power_attenuation_db: f32,
    pub low_rail_idle_power_w: f32,
    pub low_rail_power_ratio: f32,
    /// Nonzero when mid/high rails or battery input are in use.
    pub other_power_features: u32,
    /// Runtime CPMS budgets (property 64008, set by the platform after
    /// initialisation): when present they replace the four power hard
    /// limits, a key the platform does not send being 50 W (spec 9.x).
    /// TODO(SPEC): the Linux source of the value and whether it changes.
    pub runtime_budget_present: u32,
    pub runtime_budget_w: [f32; 4],
    pub ch: [ChannelParams; MAX_CHANNELS],
}

// SAFETY: repr(C) of u32/f32 fields and arrays of them, no padding.
unsafe impl Pod for Params {}
// SAFETY: as above.
unsafe impl Pod for ChannelParams {}

const _: () = assert!(core::mem::size_of::<ChannelParams>() == 25 * 4);
const _: () = assert!(core::mem::size_of::<Params>() == (37 + MAX_CHANNELS * 25) * 4);

impl Default for Params {
    fn default() -> Self {
        Params {
            channels: 0,
            sample_rate: 0,
            enable_mitigation: 0,
            control_sample_peak: 0,
            control_displacement: 0,
            control_velocity: 0,
            control_power: 0,
            control_temperature: 0,
            link_slow_gains: 0,
            unlink_temperature_gains: 0,
            model_only: 0,
            attack_time_ms: 0.0,
            fast_release_time_ms: 0.0,
            slow_release_time_ms: 0.0,
            filterbank_latency_ms: 0.0,
            speaker_response_lookahead_ms: 0.0,
            power_time_scale_s: [0.0; 4],
            power_hard_limit_w: [0.0; 4],
            power_soft_limit_pct: [0.0; 4],
            max_power_attenuation_db: 0.0,
            low_rail_idle_power_w: 0.0,
            low_rail_power_ratio: 0.0,
            other_power_features: 0,
            runtime_budget_present: 0,
            runtime_budget_w: [0.0; 4],
            ch: [ChannelParams::default(); MAX_CHANNELS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InitError {
    Blob(BlobError),
    Invalid(&'static str),
    Unsupported(&'static str),
}

impl Params {
    /// Structural checks only: the model trusts the tuning's values.
    pub fn validate(&self) -> Result<(), InitError> {
        let n = self.channels as usize;
        if n == 0 || n > MAX_CHANNELS || self.sample_rate == 0 {
            return Err(InitError::Invalid("geometry"));
        }
        let pos = |v: f32| v.is_finite() && v > 0.0;
        let nonneg = |v: f32| v.is_finite() && v >= 0.0;
        if !(pos(self.attack_time_ms) && pos(self.fast_release_time_ms) && pos(self.slow_release_time_ms)) {
            return Err(InitError::Invalid("controller times"));
        }
        if !(nonneg(self.filterbank_latency_ms) && nonneg(self.speaker_response_lookahead_ms)) {
            return Err(InitError::Invalid("latency"));
        }
        let lat =
            (2.0 * self.filterbank_latency_ms + self.speaker_response_lookahead_ms) * 1e-3 * self.sample_rate as f32;
        if lat > 16384.0 {
            return Err(InitError::Invalid("latency"));
        }
        if self.latency_frames() < CONTROLLER_FRAMES {
            return Err(InitError::Invalid("latency below the controller cadence"));
        }
        if self.enable_mitigation == 0 {
            return Err(InitError::Invalid("mitigation disabled"));
        }
        if self.control_power == 0 && self.control_temperature == 0 {
            return Err(InitError::Invalid("no thermal axis enabled"));
        }
        for j in 0..4 {
            if !(pos(self.power_time_scale_s[j])
                && pos(self.power_hard_limit_w[j])
                && self.power_soft_limit_pct[j].is_finite())
            {
                return Err(InitError::Invalid("power budgets"));
            }
        }
        if self.runtime_budget_present != 0 && !self.runtime_budget_w.iter().all(|&v| pos(v)) {
            return Err(InitError::Invalid("runtime power budgets"));
        }
        if !(nonneg(self.max_power_attenuation_db)
            && nonneg(self.low_rail_idle_power_w)
            && pos(self.low_rail_power_ratio))
        {
            return Err(InitError::Invalid("amplifier model"));
        }
        let mut seen = 0u32;
        for c in &self.ch[..n] {
            if c.source as usize >= n || seen & (1 << c.source) != 0 {
                return Err(InitError::Invalid("channel source"));
            }
            seen |= 1 << c.source;
            let mech = [c.rdc, c.bl, c.mms_kg, c.rms, c.kms, c.disp_bl, c.disp_mms_kg, c.disp_rms, c.disp_kms];
            if !(pos(c.volts_full_scale) && nonneg(c.parasitic_resistance) && mech.iter().all(|&v| pos(v))) {
                return Err(InitError::Invalid("loudspeaker model"));
            }
            if !(pos(c.rth_vc) && pos(c.tau_vc) && pos(c.rth_mag) && pos(c.tau_mag) && c.ambient.is_finite()) {
                return Err(InitError::Invalid("thermal model"));
            }
            if ![c.amplitude_limit, c.displacement_limit, c.velocity_limit0].iter().all(|&v| pos(v)) {
                return Err(InitError::Invalid("limits"));
            }
            if !(c.temperature_soft_limit.is_finite()
                && c.temperature_hard_limit.is_finite()
                && nonneg(c.temperature_headroom)
                && nonneg(c.max_temperature_attenuation_db)
                && nonneg(c.max_velocity_attenuation_db))
            {
                return Err(InitError::Invalid("temperature limits"));
            }
        }
        Ok(())
    }

    /// The audio latency in frames: two filterbank stages plus the speaker
    /// response look-ahead, each rounded to frames.
    pub fn latency_frames(&self) -> usize {
        let rate = self.sample_rate as f64;
        let frames = |ms: f32| libm::round(ms as f64 * 1e-3 * rate) as usize;
        2 * frames(self.filterbank_latency_ms) + frames(self.speaker_response_lookahead_ms)
    }

    /// Decode and check a complete blob.
    pub fn from_blob(bytes: &[u8]) -> Result<Self, InitError> {
        let (h, payload) = crate::blob::Header::parse(bytes).map_err(InitError::Blob)?;
        if h.unit != UnitKind::Lsm2 as u32 {
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

/// Per-sample one-pole pole (TODO(SPEC): the coefficient convention).
pub fn pole(rate: f64, tau_s: f64) -> f64 {
    libm::exp(-1.0 / (tau_s * rate))
}

fn db_to_lin(db: f64) -> f64 {
    libm::pow(10.0, -db / 20.0)
}

/// Direct-form-I biquad in f64 (the model states are small and stiff).
#[derive(Clone, Copy, Default)]
struct Biquad {
    b: [f64; 3],
    a: [f64; 2],
    x: [f64; 2],
    y: [f64; 2],
}

impl Biquad {
    /// Bilinear transform of `(n[2] s^2 + n[1] s + n[0]) / (d[2] s^2 + d[1] s + d[0])`.
    fn bilinear(n: [f64; 3], d: [f64; 3], rate: f64) -> Self {
        let k = 2.0 * rate;
        let k2 = k * k;
        let b0 = n[2] * k2 + n[1] * k + n[0];
        let b1 = 2.0 * (n[0] - n[2] * k2);
        let b2 = n[2] * k2 - n[1] * k + n[0];
        let a0 = d[2] * k2 + d[1] * k + d[0];
        let a1 = 2.0 * (d[0] - d[2] * k2);
        let a2 = d[2] * k2 - d[1] * k + d[0];
        Biquad { b: [b0 / a0, b1 / a0, b2 / a0], a: [a1 / a0, a2 / a0], x: [0.0; 2], y: [0.0; 2] }
    }
    #[inline]
    fn step(&mut self, x: f64) -> f64 {
        let y = self.b[0] * x + self.b[1] * self.x[0] + self.b[2] * self.x[1]
            - self.a[0] * self.y[0]
            - self.a[1] * self.y[1];
        self.x = [x, self.x[0]];
        self.y = [y, self.y[0]];
        y
    }
    fn clear(&mut self) {
        self.x = [0.0; 2];
        self.y = [0.0; 2];
    }
}

struct Chan {
    src: usize,
    delay: Delay,
    vfs: f64,
    rdc: f64,
    current: Biquad,
    disp: Biquad,
    vel: Biquad,
    rth_vc: f64,
    rth_mag: f64,
    a_vc: f64,
    a_mag: f64,
    ambient: f64,
    t_vc: f64,
    t_mag: f64,
    pbar: [f64; 4],
    /// Per-sample |input|, |x|, |v| of the samples still in the delay line.
    hist_peak: Vec<f32>,
    hist_disp: Vec<f32>,
    hist_vel: Vec<f32>,
    amp_limit: f64,
    disp_limit: f64,
    vel_limit: f64,
    vel_floor: f64,
    t_soft: f64,
    t_headroom: f64,
    temp_floor: f64,
    cand: [f64; 5],
    fast_target: f32,
    slow_target: f32,
    fast: f32,
    slow: f32,
    last_x: f64,
    last_v: f64,
}

pub struct Unit {
    params: Params,
    run: Option<Run>,
}

struct Run {
    n: usize,
    latency: usize,
    hist_len: usize,
    hist_pos: usize,
    pos: usize,
    frame: u64,
    last_frame: u64,
    mitigate: bool,
    ctl: [bool; 5],
    link: bool,
    unlink_temp: bool,
    p_pole: [f64; 4],
    budget: [f64; 4],
    power_floor: f64,
    idle: f64,
    ratio: f64,
    r_attack: f32,
    r_fast: f32,
    r_slow: f32,
    ch: Vec<Chan>,
    /// A model state went non-finite: the fail-safe is latched until reset.
    fault: bool,
}

impl Unit {
    /// Allocation happens here, never in `process`. Unusable parameters
    /// give a unit that is the fail-safe; `init_error` says why.
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

impl Run {
    fn new(p: &Params) -> Result<Run, InitError> {
        p.validate()?;
        if p.model_only == 0 {
            return Err(InitError::Unsupported("system ID or closed-loop temperature"));
        }
        if p.other_power_features != 0 {
            return Err(InitError::Unsupported("mid/high rail or battery power model"));
        }
        if p.power_soft_limit_pct.iter().any(|&v| v != 100.0) {
            return Err(InitError::Unsupported("power soft knee below 100 %"));
        }
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let latency = p.latency_frames();
        let hist_len = latency + 1;
        let dt = 1.0 / rate;
        let mut ch = Vec::with_capacity(n);
        for c in &p.ch[..n] {
            let r = c.rdc as f64 + c.parasitic_resistance as f64;
            let (bl, m, rm, k) = (c.bl as f64, c.mms_kg as f64, c.rms as f64, c.kms as f64);
            // I/U = (m s^2 + rm s + k) / (r m s^2 + (r rm + bl^2) s + r k)
            let current = Biquad::bilinear([k, rm, m], [r * k, r * rm + bl * bl, r * m], rate);
            let (bl2, m2, rm2, k2) = (c.disp_bl as f64, c.disp_mms_kg as f64, c.disp_rms as f64, c.disp_kms as f64);
            let den = [r * k2, r * rm2 + bl2 * bl2, r * m2];
            ch.push(Chan {
                src: c.source as usize,
                delay: Delay::new(latency),
                vfs: c.volts_full_scale as f64,
                rdc: c.rdc as f64,
                current,
                disp: Biquad::bilinear([bl2, 0.0, 0.0], den, rate),
                vel: Biquad::bilinear([0.0, bl2, 0.0], den, rate),
                rth_vc: c.rth_vc as f64,
                rth_mag: c.rth_mag as f64,
                a_vc: dt / (c.tau_vc as f64 + dt),
                a_mag: dt / (c.tau_mag as f64 + dt),
                ambient: c.ambient as f64,
                t_vc: c.ambient as f64,
                t_mag: c.ambient as f64,
                pbar: [0.0; 4],
                hist_peak: vec![0.0; hist_len],
                hist_disp: vec![0.0; hist_len],
                hist_vel: vec![0.0; hist_len],
                amp_limit: c.amplitude_limit as f64,
                disp_limit: c.displacement_limit as f64,
                vel_limit: c.velocity_limit0 as f64,
                vel_floor: db_to_lin(c.max_velocity_attenuation_db as f64),
                t_soft: c.temperature_soft_limit as f64,
                t_headroom: c.temperature_headroom as f64,
                temp_floor: db_to_lin(c.max_temperature_attenuation_db as f64),
                cand: [1.0; 5],
                fast_target: 1.0,
                slow_target: 1.0,
                fast: 1.0,
                slow: 1.0,
                last_x: 0.0,
                last_v: 0.0,
            });
        }
        let mut p_pole = [0.0; 4];
        let mut budget = [0.0; 4];
        for j in 0..4 {
            p_pole[j] = pole(rate, p.power_time_scale_s[j] as f64);
            budget[j] =
                if p.runtime_budget_present != 0 { p.runtime_budget_w[j] } else { p.power_hard_limit_w[j] } as f64;
        }
        Ok(Run {
            n,
            latency,
            hist_len,
            hist_pos: 0,
            pos: 0,
            frame: 0,
            last_frame: 0,
            mitigate: p.enable_mitigation != 0,
            ctl: [
                p.control_sample_peak != 0,
                p.control_displacement != 0,
                p.control_velocity != 0,
                p.control_power != 0,
                p.control_temperature != 0,
            ],
            link: p.link_slow_gains != 0,
            unlink_temp: p.unlink_temperature_gains != 0,
            p_pole,
            budget,
            power_floor: db_to_lin(p.max_power_attenuation_db as f64),
            idle: p.low_rail_idle_power_w as f64,
            ratio: p.low_rail_power_ratio as f64,
            r_attack: pole(rate, p.attack_time_ms as f64 * 1e-3) as f32,
            r_fast: pole(rate, p.fast_release_time_ms as f64 * 1e-3) as f32,
            r_slow: pole(rate, p.slow_release_time_ms as f64 * 1e-3) as f32,
            ch,
            fault: false,
        })
    }

    fn reset(&mut self) {
        for c in self.ch.iter_mut() {
            c.delay.clear();
            c.current.clear();
            c.disp.clear();
            c.vel.clear();
            c.t_vc = c.ambient;
            c.t_mag = c.ambient;
            c.pbar = [0.0; 4];
            c.hist_peak.iter_mut().for_each(|v| *v = 0.0);
            c.hist_disp.iter_mut().for_each(|v| *v = 0.0);
            c.hist_vel.iter_mut().for_each(|v| *v = 0.0);
            c.cand = [1.0; 5];
            c.fast_target = 1.0;
            c.slow_target = 1.0;
            c.fast = 1.0;
            c.slow = 1.0;
            c.last_x = 0.0;
            c.last_v = 0.0;
        }
        self.hist_pos = 0;
        self.pos = 0;
        self.frame = 0;
        self.last_frame = 0;
        self.fault = false;
    }

    /// Model one input sample of every channel.
    #[inline]
    fn model(&mut self, io: &Io<'_, '_>, k: usize) {
        let hp = self.hist_pos;
        for c in self.ch.iter_mut() {
            let x = dsp::bound1(io.input[c.src][k]) as f64;
            let u = c.vfs * x;
            let i = c.current.step(u);
            let xd = c.disp.step(u);
            let v = c.vel.step(u);
            let p = i * i * c.rdc;
            let old_mag = c.t_mag;
            c.t_vc += c.a_vc * (old_mag + p * c.rth_vc - c.t_vc);
            c.t_mag += c.a_mag * (c.ambient + p * c.rth_mag - c.t_mag);
            let pamp = self.idle + self.ratio * (u * i).max(0.0);
            for j in 0..4 {
                c.pbar[j] = pamp + self.p_pole[j] * (c.pbar[j] - pamp);
            }
            c.hist_peak[hp] = x.abs() as f32;
            c.hist_disp[hp] = xd.abs() as f32;
            c.hist_vel[hp] = v.abs() as f32;
            c.last_x = xd;
            c.last_v = v;
        }
        self.hist_pos = (hp + 1) % self.hist_len;
    }

    /// Controller update over every sample still in the delay line.
    fn control(&mut self) {
        for c in self.ch.iter_mut() {
            let pk = c.hist_peak.iter().fold(0.0f32, |m, &v| m.max(v));
            let xd = c.hist_disp.iter().fold(0.0f32, |m, &v| m.max(v));
            let vl = c.hist_vel.iter().fold(0.0f32, |m, &v| m.max(v));
            let ratio = |limit: f64, q: f32| (limit / (q as f64).max(EPS)).min(1.0);
            let g_peak = if self.ctl[0] { ratio(c.amp_limit, pk) } else { 1.0 };
            let g_disp = if self.ctl[1] { ratio(c.disp_limit, xd) } else { 1.0 };
            let g_vel = if self.ctl[2] { ratio(c.vel_limit, vl).max(c.vel_floor) } else { 1.0 };
            let g_pow = if self.ctl[3] {
                let mut g = 1.0f64;
                for j in 0..4 {
                    g = g.min(libm::sqrt(self.budget[j] / c.pbar[j].max(EPS)));
                }
                g.max(self.power_floor)
            } else {
                1.0
            };
            let g_temp = if self.ctl[4] {
                // TODO(SPEC): the knee (restrictive stand-in, module docs)
                let predicted = c.t_vc + c.t_headroom;
                // a non-finite temperature takes the restrictive path
                if predicted >= c.t_soft || predicted.is_nan() {
                    c.temp_floor
                } else {
                    1.0
                }
            } else {
                1.0
            };
            c.cand = [g_peak, g_disp, g_vel, g_pow, g_temp];
            c.fast_target = g_peak.min(g_disp).min(g_vel) as f32;
            c.slow_target = g_pow.min(g_temp) as f32;
            // any non-finite model or controller state latches the fail-safe
            let model = [c.t_vc, c.t_mag, c.last_x, c.last_v].iter().chain(&c.pbar).all(|v| v.is_finite());
            let gains = c.cand.iter().all(|v| v.is_finite()) && c.fast.is_finite() && c.slow.is_finite();
            if !(model && gains) {
                self.fault = true;
            }
        }
        if self.link && self.n > 1 {
            let (mut slow, mut pow) = (1.0f32, 1.0f32);
            for c in &self.ch {
                slow = slow.min(c.slow_target);
                pow = pow.min(c.cand[3] as f32);
            }
            for c in self.ch.iter_mut() {
                c.slow_target = if self.unlink_temp { pow.min(c.cand[4] as f32) } else { slow };
            }
        }
    }

    fn process(&mut self, io: &mut Io<'_, '_>) {
        let frames = io.frames;
        let mut k = 0;
        while k < frames {
            let m = (CONTROLLER_FRAMES - self.pos).min(frames - k);
            for s in k..k + m {
                self.model(io, s);
            }
            for c in self.ch.iter_mut() {
                let inp = &io.input[c.src][k..k + m];
                let out = &mut io.output[c.src][k..k + m];
                // SAFETY: distinct slices of `m` samples.
                unsafe { c.delay.run(inp.as_ptr(), out.as_mut_ptr(), m) };
                dsp::bound(out);
                if self.mitigate {
                    for o in out.iter_mut() {
                        let rf = if c.fast_target < c.fast { self.r_attack } else { self.r_fast };
                        c.fast = c.fast_target + rf * (c.fast - c.fast_target);
                        let rs = if c.slow_target < c.slow { self.r_attack } else { self.r_slow };
                        c.slow = c.slow_target + rs * (c.slow - c.slow_target);
                        *o *= c.fast.min(c.slow);
                    }
                }
            }
            k += m;
            self.pos += m;
            self.frame += m as u64;
            if self.pos == CONTROLLER_FRAMES {
                self.pos = 0;
                self.control();
                self.last_frame = self.frame - CONTROLLER_FRAMES as u64;
                if self.fault {
                    crate::failsafe::fill_from(io, k, FAILSAFE_GAIN);
                    return;
                }
            }
        }
    }
}

impl ProtectionUnit for Unit {
    fn kind(&self) -> UnitKind {
        UnitKind::Lsm2
    }
    fn latency_frames(&self) -> u32 {
        self.run.as_ref().map_or(0, |r| r.latency as u32)
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
        run.process(&mut io);
        if run.fault {
            return Health(Health::FAILSAFE | Health::MODEL_FAULT);
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
        if run.fault {
            out.health = Health(Health::FAILSAFE | Health::MODEL_FAULT);
        }
        for c in &run.ch {
            let mut s = ChannelState::ABSENT;
            s.modeled_power_w = c.pbar[2] as f32;
            s.coil_temp_c = c.t_vc as f32;
            s.magnet_temp_c = c.t_mag as f32;
            s.ol_temp_c = c.t_vc as f32;
            s.displacement_m = c.last_x as f32;
            s.velocity_m_s = c.last_v as f32;
            for (a, &g) in s.axis_gain.iter_mut().zip(&c.cand) {
                *a = g as f32;
            }
            s.gain = if run.fault {
                FAILSAFE_GAIN
            } else if run.mitigate {
                c.fast.min(c.slow)
            } else {
                1.0
            };
            out.channels[c.src] = s;
        }
    }
    fn reset(&mut self) {
        if let Some(r) = self.run.as_mut() {
            r.reset();
        }
    }
}

#[cfg(test)]
mod fault_tests {
    use super::*;

    fn params() -> Params {
        let mut p = Params {
            channels: 2,
            sample_rate: 48_000,
            enable_mitigation: 1,
            control_power: 1,
            control_temperature: 1,
            link_slow_gains: 1,
            model_only: 1,
            attack_time_ms: 2.0,
            fast_release_time_ms: 150.0,
            slow_release_time_ms: 10_000.0,
            filterbank_latency_ms: 2.5,
            speaker_response_lookahead_ms: 1.0,
            power_time_scale_s: [0.002, 0.2, 2.0, 30.0],
            power_hard_limit_w: [50.0, 20.0, 2.0, 1.5],
            power_soft_limit_pct: [100.0; 4],
            max_power_attenuation_db: 4.0,
            low_rail_power_ratio: 1.0,
            ..Params::default()
        };
        for c in 0..2 {
            p.ch[c] = ChannelParams {
                source: c as u32,
                volts_full_scale: 6.0,
                parasitic_resistance: 0.2,
                rdc: 4.0,
                bl: 0.8,
                mms_kg: 0.0002,
                rms: 0.2,
                kms: 1500.0,
                disp_bl: 0.8,
                disp_mms_kg: 0.00022,
                disp_rms: 0.25,
                disp_kms: 1600.0,
                rth_vc: 30.0,
                tau_vc: 2.0,
                rth_mag: 20.0,
                tau_mag: 60.0,
                ambient: 40.0,
                amplitude_limit: 1.0,
                displacement_limit: 0.0005,
                velocity_limit0: 0.5,
                max_velocity_attenuation_db: 18.0,
                temperature_soft_limit: 110.0,
                temperature_hard_limit: 125.0,
                temperature_headroom: 4.0,
                max_temperature_attenuation_db: 10.0,
            };
        }
        p
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

    /// Review finding 1: a non-finite state latches 20 dB until reset.
    #[test]
    fn a_non_finite_model_state_latches_the_failsafe_until_reset() {
        let poisons: [fn(&mut Run); 3] =
            [|r| r.ch[0].t_mag = f64::NAN, |r| r.ch[1].pbar[2] = f64::INFINITY, |r| r.ch[0].slow = f32::NAN];
        for poison in poisons {
            let mut u = Unit::new(params());
            let (o, h) = run(&mut u, 0.01, 1024);
            assert_eq!((o[1023], h), (0.01, Health::OK));
            poison(u.run.as_mut().unwrap());
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
            u.reset();
            let (o, h) = run(&mut u, 0.01, 1024);
            assert_eq!((o[1023], h), (0.01, Health::OK));
        }
    }
}
