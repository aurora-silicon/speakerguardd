//! `lsm2`: signal-only loudspeaker model and five-axis constraint
//! controller (MacBook Neo, MacBook Air M4/M5), from the behaviour
//! specification (SPEC-LSM2 v1.1). This is the model-only configuration the
//! Neo ships (static model, system ID bypassed, no closed-loop
//! temperature); a tuning that asks for the DSP estimator or closed-loop
//! temperature is refused.
//!
//! Signal flow per channel, at the sample rate:
//!
//! ```text
//! x ─┬─ five-band complementary FIR bank (319 taps, centred) ─ c[k] ─┬─ look-ahead ─ Σ g[k]·c[k] ─ × min(gpeak, gslow) ─ y
//!    │                                                               └─ u[k]=Vfs·c[k] → x/v models → band candidates
//!    ├─ |x| → sample-peak candidate
//!    └─ pressure FIR → 8–12 kHz band → mask energy → velocity limit
//! y ─ u=Vfs·y → current model → u·i → four power averages, two thermal nodes (the monitor)
//! ```
//!
//! - **Models** (confirmed on the 25G83 rig, lsm2 law 1): current,
//!   displacement and velocity each use their own block's `RdcOhms`
//!   (`R = Rdc`); `ParasiticResistance` enters no model state, only the
//!   amplifier load term `(Rdc_adm + Rpar) / Rdc_adm` with the admittance
//!   block's `Rdc`. Displacement and velocity are plain bilinear
//!   (unwarped) discretisations of `X/U = Bl/(R(Mms s^2 + Rms s + Kms) +
//!   Bl^2 s)` and `V = sX` at any rate. The controller cadence is 5.333 ms
//!   (256 frames at 48 kHz, 512 at 96 kHz), with eight power substeps.
//! - **Latency** `2*FilterbankLatency + SpeakerResponseLookAhead` (352 frames
//!   at 48 kHz): the FIR bank's centre (`FilterbankLatency - 1`, 159) plus
//!   the look-ahead the controller acts in. With every gain at unity the
//!   output is the input delayed exactly, bit for bit (spec 6.1: the bank is
//!   bypassed). A latency below the 256-frame controller cadence is refused
//!   (a transient could leave before any decision saw it).
//! - **Filterbank** (rig law 10; the spec's random-order LR is not it): on
//!   the design grid of `M = 2*FilterbankLatency*fs` points (320 at 48 kHz,
//!   150 Hz bins), each band centre snapped to a bin; cumulative low pass k
//!   is 1 up to centre bin k, 0 from centre bin k+1, `cos^2` between; bands
//!   are successive differences, inverse real DFT, centred, the `M/2` tap
//!   dropped (319 taps), generated in f64 and rounded once to f32. The
//!   fifth band is the delayed input minus the other four, so the bank sums
//!   to the delay exactly.
//! - **Candidates** every controller frame (256 frames, spec 10), linear in
//!   [0, 1], a disabled axis giving 1, combined by `min`:
//!   - sample peak: `min(1, (AmplitudeLimit - pilot) / max|x|)`, the pilot
//!     tone's linear amplitude reserved even with the pilot off (rig law 6);
//!   - displacement and velocity: the models run on the program the bank
//!     resynthesises, `min(1, limit / max|q|)`, applied as band gains:
//!     displacement to every band, velocity (floored at
//!     `-MaxVelocityAttenuation`) to every band but the outer ones when
//!     `VelocityIgnoreLow/HighBand`; the velocity limit follows the masking
//!     curve (spec 6.2);
//!   - power (rig law 7): one gain for all systems,
//!     `min_j min(1, sqrt((B_j - N idle) / sum (Phat_j - idle)))`, `B_j` the
//!     total budget (preset or property 64008, times `PowerBudgetsScale`),
//!     `Phat_j` each system's largest unattenuated average over the frame;
//!     `MaxPowerAttenuation` does not bound it;
//!   - temperature: the linear-in-dB knee of spec 6.1 on the coil
//!     temperature.
//!   - Slow gains (power, temperature) are linked across channels.
//! - **Combination**: each band's applied gain is the minimum of its own
//!   (displacement/velocity) gain and the broadband peak and slow gains.
//! - **Smoothing** per sample with the exponential one-pole (spec 6.1):
//!   attack `AttackTime`; release `LowBandReleaseTime` (band 0),
//!   `FastReleaseTime` (other bands, peak), `SlowReleaseTime` (slow).
//! - **Monitor** (measured against the 25G83 controller logs, see the
//!   TODO(SPEC) list): the power averages and the thermal model run on the
//!   protected output. `Pamp = idle + ratio * max(mean32(u*i), 0) *
//!   (Rdc+Rpar)/Rdc` every 32 frames into the four exponential averages;
//!   the thermal input is the substep mean of `u*i` (no load factor), the
//!   nodes independent matched one-poles per substep with Float32
//!   coefficients and state, the coil in series on the magnet:
//!   `Tvc = Tamb + theta_mag + theta_vc` (rig laws 3 and 4: the monitor
//!   reads `Vfs * output` bit for bit).
//!
//! Full scale: the output is held to digital full scale (`clamp(x, -1,
//! 1)`, a non-finite sample 0). The bank and every model see the program
//! unclamped as the reference does, bounded only at `dsp::METER_FS` (16
//! FS), so a huge finite sample cannot overflow them. A model state that
//! goes non-finite latches the 20 dB fail-safe (`Health::MODEL_FAULT`) until
//! reset. `EnableMitigation = 0` (macOS: model only, no attenuation) and a
//! tuning with neither thermal axis are refused: either would run the
//! speakers unprotected while health reports OK.
//!
//! Hand-off: `save_state`/`restore_state` carry the coil and magnet rises,
//! the four power averages and the slow gain to the next stream or
//! instance, cooled for the gap (the series nodes decay with their own
//! constants; the averages toward idle) with the slow gain released toward
//! unity. The reference resets on reinitialisation (spec 8); a speaker does
//! not cool because a stream restarted, so here it never does.
//!
//! Meters (rig law 11), over the last complete controller frame:
//! `AmplifierInputPowerPerSpeaker` is the largest 1 ms average of system 0
//! alone; `ModeledAmplifierPower` the sum over systems of each one's
//! largest 1 s average.
//!
//! Failure: the reference unit fails initialisation outright when neither
//! estimator can be built. Here an unusable parameter set, or a geometry
//! mismatch, gives the 20 dB fail-safe, which on the Neo equals the
//! kernel's own floor.
//!
//! TODO(SPEC), each the most literal reading of the specification or the
//! measured 25G83 behaviour where the two disagree:
//! - **Current model.** The logged current is a second-order filter with
//!   the physical zeros and `1/Rdc` at DC and Nyquist, but its poles are
//!   about 0.9 % more damped than the admittance block's
//!   `Rdc, Bl, Mms, Rms, Kms` give (pole Q 1.9225 against 1.9402). The
//!   bilinear admittance of the block is used: 0.011 dB too much current at
//!   1 kHz, 0.08 dB at resonance.
//! - **Candidate windows.** Which predicted samples a controller frame
//!   examines; here every sample that entered since the previous frame.
//! - **Band candidates.** How displacement/velocity per band are formed.
//!   Each band's own motion against the whole limit gives twice the 25G83
//!   velocity gain on a 1 kHz tone (0.738 against 0.3645: the tone sits on
//!   the 1031 Hz crossover); the full-band motion matches, so the
//!   full-band candidate is applied to the bands it may act on.
//! - **Mask detector.** The 8-12 kHz band filter, the mean-square window
//!   and the dB reference; here second-order Butterworth high/low passes
//!   on the pressure FIR's output, a controller-frame mean square, 10 log10
//!   of it, smoothed in dB at the controller cadence.
//! - **Pressure path.** Which modeled signal the 1,024-tap response
//!   filters; here the normalised input.
//! - **Applied temperature gain** (rig law 8, confirmed part): the linked
//!   minimum of the temperature candidates is followed by its own law: a
//!   cut to 0.99 x the candidate over the attack time when it falls below
//!   the applied gain, else one 99 %-settling `SlowReleaseTime` step per
//!   controller frame. The larger cuts near `TemperatureHardLimit +
//!   TemperatureHeadroom` and the candidate itself (the axis's own thermal
//!   predictor) are still open: the rig settles the ceiling at
//!   `0.99 sqrt((C - Tamb - theta_mag) / (Rvc Pin))` with `C = Thard +
//!   max(0, Headroom)` on the hottest output-referred channel and `Pin` the
//!   ungained input power (temperature axis alone; other axes untested),
//!   but the approach transient is unexplained, so neither is implemented.
//!   Daybreak t-4670.
//! - **Temperature candidate.** The 25G83 temperature gain falls from about
//!   63 degC under the 1 kHz and 200 Hz ramps (0.886 at 63.4 degC) with a
//!   trajectory that does not depend on the coil temperature, where the
//!   knee (soft 120 degC, headroom 5) gives unity. The knee is used.
//!   Daybreak t-4670.
//! - Mid/high rails, battery input and soft knees below 100 % are refused.

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::{
    dsp, gap_or_zero, release_toward_unity, ChannelState, HandOff, Health, Io, ProtectionUnit, Telemetry, UnitKind,
    FAILSAFE_GAIN, MAX_CHANNELS,
};

/// Controller update cadence in frames at 48 kHz (spec 10: 256). The
/// cadence is a time, 5.333 ms: 512 frames at 96 kHz (25G83 rig), see
/// [`controller_frames`].
pub const CONTROLLER_FRAMES: usize = 256;
/// Power/thermal substep in frames at 48 kHz (spec 6.1: 32, eight per
/// controller frame).
pub const SUBSTEP_FRAMES: usize = 32;
/// Substeps per controller frame.
pub const SUBSTEPS: usize = CONTROLLER_FRAMES / SUBSTEP_FRAMES;

/// The controller frame at `rate`: 256 frames per 48 kHz, i.e. 5.333 ms.
pub fn controller_frames(rate: u32) -> usize {
    let f = libm::round(CONTROLLER_FRAMES as f64 * rate as f64 / 48_000.0) as usize;
    // whole substeps, at least one frame each
    (f / SUBSTEPS).max(1) * SUBSTEPS
}
/// Filterbank bands.
pub const BANDS: usize = 5;
/// Taps of the property-6632 pressure response.
pub const PRESSURE_TAPS: usize = 1024;
const EPS: f64 = f32::MIN_POSITIVE as f64;

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
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
    /// Displacement model.
    pub disp_rdc: f32,
    pub disp_bl: f32,
    pub disp_mms_kg: f32,
    pub disp_rms: f32,
    pub disp_kms: f32,
    /// Velocity model.
    pub vel_rdc: f32,
    pub vel_bl: f32,
    pub vel_mms_kg: f32,
    pub vel_rms: f32,
    pub vel_kms: f32,
    /// Thermal type block.
    pub rth_vc: f32,
    pub tau_vc: f32,
    pub rth_mag: f32,
    pub tau_mag: f32,
    pub ambient: f32,
    /// Limits.
    pub amplitude_limit: f32,
    pub displacement_limit: f32,
    /// Velocity plateaus `VelocityLimit0..2`, m/s.
    pub velocity_limit: [f32; 3],
    /// Mask energy knots `MaskEnergy0..3`, dB.
    pub mask_energy_db: [f32; 4],
    /// `MaskBandMin`, `MaskBandMax`, Hz.
    pub mask_band_hz: [f32; 2],
    /// `MaskAttack`, `MaskRelease`, s.
    pub mask_attack_s: f32,
    pub mask_release_s: f32,
    /// `MaskAttackFast`, s, and `MaskAttackRange`, dB.
    pub mask_attack_fast_s: f32,
    pub mask_attack_range_db: f32,
    pub max_velocity_attenuation_db: f32,
    pub velocity_ignore_low_band: u32,
    pub velocity_ignore_high_band: u32,
    pub temperature_soft_limit: f32,
    pub temperature_hard_limit: f32,
    pub temperature_headroom: f32,
    pub max_temperature_attenuation_db: f32,
    /// `BandCenter0..4`, Hz.
    pub band_center_hz: [f32; BANDS],
    /// `Bandwidth` enum 0/1/2.
    pub bandwidth: u32,
    /// The sample-peak reserve: the linear `PilotToneAmplitude_dB` of the
    /// channel's tuning bank, kept free even with the pilot off (rig law 6).
    pub pilot_reserve: f32,
    /// Number of pressure-response taps (0 without property 6632, else
    /// [`PRESSURE_TAPS`]).
    pub pressure_taps: u32,
    pub pressure_fir: [f32; PRESSURE_TAPS],
}

impl Default for ChannelParams {
    fn default() -> Self {
        ChannelParams {
            source: 0,
            volts_full_scale: 0.0,
            parasitic_resistance: 0.0,
            rdc: 0.0,
            bl: 0.0,
            mms_kg: 0.0,
            rms: 0.0,
            kms: 0.0,
            disp_rdc: 0.0,
            disp_bl: 0.0,
            disp_mms_kg: 0.0,
            disp_rms: 0.0,
            disp_kms: 0.0,
            vel_rdc: 0.0,
            vel_bl: 0.0,
            vel_mms_kg: 0.0,
            vel_rms: 0.0,
            vel_kms: 0.0,
            rth_vc: 0.0,
            tau_vc: 0.0,
            rth_mag: 0.0,
            tau_mag: 0.0,
            ambient: 0.0,
            amplitude_limit: 0.0,
            displacement_limit: 0.0,
            velocity_limit: [0.0; 3],
            mask_energy_db: [0.0; 4],
            mask_band_hz: [0.0; 2],
            mask_attack_s: 0.0,
            mask_release_s: 0.0,
            mask_attack_fast_s: 0.0,
            mask_attack_range_db: 0.0,
            max_velocity_attenuation_db: 0.0,
            velocity_ignore_low_band: 0,
            velocity_ignore_high_band: 0,
            temperature_soft_limit: 0.0,
            temperature_hard_limit: 0.0,
            temperature_headroom: 0.0,
            max_temperature_attenuation_db: 0.0,
            band_center_hz: [0.0; BANDS],
            bandwidth: 0,
            pilot_reserve: 0.0,
            pressure_taps: 0,
            pressure_fir: [0.0; PRESSURE_TAPS],
        }
    }
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
    pub low_band_release_time_ms: f32,
    pub filterbank_latency_ms: f32,
    pub speaker_response_lookahead_ms: f32,
    pub power_time_scale_s: [f32; 4],
    pub power_hard_limit_w: [f32; 4],
    pub power_soft_limit_pct: [f32; 4],
    pub power_budgets_scale: f32,
    pub max_power_attenuation_db: f32,
    pub low_rail_idle_power_w: f32,
    pub low_rail_power_ratio: f32,
    /// Nonzero when mid/high rails or battery input are in use.
    pub other_power_features: u32,
    /// Runtime CPMS budgets (property 64008, set by the platform after
    /// initialisation): when present they replace the four power hard
    /// limits, a key the platform does not send being 50 W (spec 6.1, 8).
    pub runtime_budget_present: u32,
    pub runtime_budget_w: [f32; 4],
    pub ch: [ChannelParams; MAX_CHANNELS],
}

// SAFETY: repr(C) of u32/f32 fields and arrays of them, no padding.
unsafe impl Pod for Params {}
// SAFETY: as above.
unsafe impl Pod for ChannelParams {}

const CH_WORDS: usize = 53 + PRESSURE_TAPS;
const _: () = assert!(core::mem::size_of::<ChannelParams>() == CH_WORDS * 4);
const _: () = assert!(core::mem::size_of::<Params>() == (39 + MAX_CHANNELS * CH_WORDS) * 4);

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
            low_band_release_time_ms: 0.0,
            filterbank_latency_ms: 0.0,
            speaker_response_lookahead_ms: 0.0,
            power_time_scale_s: [0.0; 4],
            power_hard_limit_w: [0.0; 4],
            power_soft_limit_pct: [0.0; 4],
            power_budgets_scale: 1.0,
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
        if !(pos(self.attack_time_ms)
            && pos(self.fast_release_time_ms)
            && pos(self.slow_release_time_ms)
            && pos(self.low_band_release_time_ms))
        {
            return Err(InitError::Invalid("controller times"));
        }
        if !(nonneg(self.filterbank_latency_ms) && nonneg(self.speaker_response_lookahead_ms)) {
            return Err(InitError::Invalid("latency"));
        }
        // The bank needs a centre tap and at least one on each side.
        if self.filterbank_frames() < 2 || self.latency_frames() > 16384 {
            return Err(InitError::Invalid("latency"));
        }
        if self.latency_frames() < controller_frames(self.sample_rate) {
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
        if !(pos(self.power_budgets_scale)
            && nonneg(self.max_power_attenuation_db)
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
            let mech = [
                c.rdc,
                c.bl,
                c.mms_kg,
                c.rms,
                c.kms,
                c.disp_rdc,
                c.disp_bl,
                c.disp_mms_kg,
                c.disp_rms,
                c.disp_kms,
                c.vel_rdc,
                c.vel_bl,
                c.vel_mms_kg,
                c.vel_rms,
                c.vel_kms,
            ];
            if !(pos(c.volts_full_scale) && nonneg(c.parasitic_resistance) && mech.iter().all(|&v| pos(v))) {
                return Err(InitError::Invalid("loudspeaker model"));
            }
            if !(pos(c.rth_vc) && pos(c.tau_vc) && pos(c.rth_mag) && pos(c.tau_mag) && c.ambient.is_finite()) {
                return Err(InitError::Invalid("thermal model"));
            }
            if !(nonneg(c.pilot_reserve) && c.pilot_reserve < c.amplitude_limit) {
                return Err(InitError::Invalid("pilot reserve"));
            }
            if !(pos(c.amplitude_limit) && pos(c.displacement_limit) && c.velocity_limit.iter().all(|&v| pos(v))) {
                return Err(InitError::Invalid("limits"));
            }
            if !(c.mask_energy_db.iter().all(|v| v.is_finite())
                && c.mask_energy_db.windows(2).all(|w| w[0] <= w[1])
                && c.velocity_limit.windows(2).all(|w| w[0] <= w[1]))
            {
                return Err(InitError::Invalid("velocity mask curve"));
            }
            let nyq = self.sample_rate as f32 / 2.0;
            if !(pos(c.mask_band_hz[0])
                && c.mask_band_hz[0] < c.mask_band_hz[1]
                && c.mask_band_hz[1] < nyq
                && pos(c.mask_attack_s)
                && pos(c.mask_release_s)
                && pos(c.mask_attack_fast_s)
                && nonneg(c.mask_attack_range_db))
            {
                return Err(InitError::Invalid("velocity mask detector"));
            }
            if !(c.band_center_hz.iter().all(|&f| pos(f) && f < nyq)
                && c.band_center_hz.windows(2).all(|w| w[0] < w[1])
                && c.bandwidth <= 2)
            {
                return Err(InitError::Invalid("filterbank"));
            }
            if !(c.pressure_taps == 0 || c.pressure_taps as usize == PRESSURE_TAPS)
                || !c.pressure_fir.iter().all(|v| v.is_finite())
            {
                return Err(InitError::Invalid("pressure response"));
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

    /// One filterbank stage in frames.
    pub fn filterbank_frames(&self) -> usize {
        libm::round(self.filterbank_latency_ms as f64 * 1e-3 * self.sample_rate as f64) as usize
    }

    /// The audio latency in frames: two filterbank stages plus the speaker
    /// response look-ahead, each rounded to frames.
    pub fn latency_frames(&self) -> usize {
        let la = libm::round(self.speaker_response_lookahead_ms as f64 * 1e-3 * self.sample_rate as f64) as usize;
        2 * self.filterbank_frames() + la
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

/// The exponential one-pole coefficient of spec 6.1 for an update every
/// `delta_s` seconds: `exp(-delta/tau)`.
pub fn pole(delta_s: f64, tau_s: f64) -> f64 {
    libm::exp(-delta_s / tau_s)
}

fn db_to_lin(db: f64) -> f64 {
    libm::pow(10.0, -db / 20.0)
}

/// The five centred FIR bands of the complementary filterbank (spec 6.3)
/// for a filterbank stage of `stage` frames: `2*stage - 1` taps, the centre
/// at `stage - 1`, designed on a grid of `2*stage` points; generated in f64
/// and rounded once to f32. `h[k][j]` multiplies `x[n - j]`; the bands sum
/// to a delay of `stage - 1` in f64.
pub fn filterbank(rate: f64, centers: &[f32; BANDS], bandwidth: u32, stage: usize) -> [Vec<f32>; BANDS] {
    // TODO(SPEC): Bandwidth 1 is bit-identical to 0 on 25G83, and 2 turns
    // the velocity attenuation off; neither changes the bank here.
    let _ = bandwidth;
    let half = stage - 1;
    let grid = 2 * stage;
    // rig law 10: each centre snapped to a bin of the design grid
    let df = rate / grid as f64;
    let bin: [f64; BANDS] = core::array::from_fn(|k| libm::round(centers[k] as f64 / df));
    // zero-phase magnitudes on the design grid, bins 0..=grid/2: the
    // cumulative low pass k is 1 up to centre bin k, 0 from centre bin k+1,
    // a cos^2 crossfade between
    let bins = grid / 2 + 1;
    let mut mag: [Vec<f64>; BANDS] = core::array::from_fn(|_| vec![0.0; bins]);
    for b in 0..bins {
        let bf = b as f64;
        let mut prev = 0.0;
        for (k, m) in mag.iter_mut().enumerate() {
            let cum = if k == BANDS - 1 || bf <= bin[k] {
                1.0
            } else if bf >= bin[k + 1] {
                0.0
            } else {
                let c = libm::cos(core::f64::consts::FRAC_PI_2 * (bf - bin[k]) / (bin[k + 1] - bin[k]));
                c * c
            };
            m[b] = cum - prev;
            prev = cum;
        }
    }
    let n = grid as f64;
    core::array::from_fn(|k| {
        (0..2 * half + 1)
            .map(|j| {
                // tap j is time t = j - half of the zero-phase response
                let t = j as f64 - half as f64;
                let mut acc = mag[k][0] + mag[k][bins - 1] * libm::cos(core::f64::consts::PI * t);
                for (b, &m) in mag[k].iter().enumerate().take(bins - 1).skip(1) {
                    acc += 2.0 * m * libm::cos(2.0 * core::f64::consts::PI * b as f64 * t / n);
                }
                (acc / n) as f32
            })
            .collect()
    })
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
    /// Second-order Butterworth high or low pass at `f` Hz (bilinear,
    /// prewarped).
    fn butter2(f: f64, rate: f64, high: bool) -> Self {
        let w = 2.0 * rate * libm::tan(core::f64::consts::PI * f / rate);
        let q = core::f64::consts::SQRT_2;
        let d = [w * w, w * q, 1.0];
        let n = if high { [0.0, 0.0, 1.0] } else { [w * w, 0.0, 0.0] };
        Biquad::bilinear(n, d, rate)
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

/// A history kept twice over, so the last `len` samples are always one
/// contiguous slice (newest last) for the FIR dot products.
struct History {
    buf: Vec<f32>,
    len: usize,
    pos: usize,
}

impl History {
    fn new(len: usize) -> Self {
        History { buf: vec![0.0; 2 * len], len, pos: 0 }
    }
    #[inline]
    fn push(&mut self, x: f32) {
        self.buf[self.pos] = x;
        self.buf[self.pos + self.len] = x;
        self.pos += 1;
        if self.pos == self.len {
            self.pos = 0;
        }
    }
    /// The last `len` samples, oldest first.
    #[inline]
    fn window(&self) -> &[f32] {
        &self.buf[self.pos..self.pos + self.len]
    }
    /// The sample `age` samples ago (0 = newest).
    #[inline]
    fn ago(&self, age: usize) -> f32 {
        self.buf[self.pos + self.len - 1 - age]
    }
    fn clear(&mut self) {
        self.buf.iter_mut().for_each(|v| *v = 0.0);
        self.pos = 0;
    }
}

/// FIR dot product `sum taps[j] * x[n - j]` over a window whose newest
/// sample is last. Sequential f32 accumulation: the scalar reference.
#[inline]
pub fn fir_scalar(taps: &[f32], window: &[f32]) -> f32 {
    let n = taps.len();
    let mut acc = 0.0f32;
    for j in 0..n {
        acc += taps[j] * window[n - 1 - j];
    }
    acc
}

/// The same dot product with the taps stored reversed, so both run
/// forward (vectorised on aarch64, see [`crate::dsp::dot_f32`]).
#[inline]
pub fn fir_reversed(rev_taps: &[f32], window: &[f32]) -> f32 {
    crate::dsp::dot_f32(rev_taps, window)
}

/// One one-pole gain smoother at the sample rate: `attack` when moving
/// down, `release` up. Snaps to the target within one f32 step, so a gain
/// that returns to unity is exactly unity (and the bank is bypassed).
#[inline]
fn smooth(g: &mut f32, target: f32, attack: f32, release: f32) {
    let r = if target < *g { attack } else { release };
    let next = target + r * (*g - target);
    *g = if (next - target).abs() <= f32::EPSILON * target.abs() { target } else { next };
}

struct Chan {
    src: usize,
    vfs: f64,
    /// Input history: the bank's window plus the look-ahead. The bands are
    /// evaluated at the output, and only those whose gain differs from the
    /// top band's (unity audio costs no FIR at all).
    hist: History,
    /// Bank taps reversed (dot-product order), bands 0..3.
    taps_rev: [Vec<f32>; BANDS - 1],
    /// Prediction models on the bank's centre (the program the bands
    /// resynthesise).
    disp: Biquad,
    vel: Biquad,
    /// Peaks since the previous controller frame.
    pk_in: f32,
    pk_disp: f32,
    pk_vel: f32,
    /// Per-band velocity models and peaks (the provisional water-fill).
    #[cfg(feature = "lsm2-waterfill")]
    vel_band: [Biquad; BANDS],
    #[cfg(feature = "lsm2-waterfill")]
    pk_vel_band: [f32; BANDS],
    /// Masking path.
    pressure_rev: Vec<f32>,
    p_hist: History,
    mask_hp: Biquad,
    mask_lp: Biquad,
    mask_acc: f64,
    mask_db: f64,
    mask_init: bool,
    vel_lim: [f64; 3],
    mask_knots: [f64; 4],
    mask_a: f64,
    mask_af: f64,
    mask_r: f64,
    mask_range: f64,
    vel_limit_now: f64,
    // Monitor on the output.
    current: Biquad,
    /// The power path of the unattenuated program (rig law 7): the current
    /// model on the bank's centre, its substep load and four averages, and
    /// their largest values over the running controller frame.
    current_in: Biquad,
    sub_acc_in: f64,
    pbar_in: [f64; 4],
    pbar_in_peak: [f64; 4],
    load_scale: f64,
    rth_vc: f64,
    rth_mag: f64,
    /// Matched one-pole of each node per substep, Float32 (rig law 3):
    /// `b = fl32(exp(-dt/tau))`, `a = fl32(1 - exp(-dt/tau))`; the coil
    /// updates as `theta += a (P Rvc - theta)`.
    a_vc: f32,
    b_mag: f32,
    a_mag: f32,
    tau_vc: f64,
    tau_mag: f64,
    ambient: f64,
    /// Node rises, Float32 state as the reference keeps them.
    th_vc: f32,
    th_mag: f32,
    sub_acc: f64,
    pbar: [f64; 4],
    /// Largest 1 ms average over the running controller frame, and over
    /// the last complete one (the per-speaker meter).
    pbar1_peak: f64,
    pbar1_frame: f64,
    /// The same for the 1 s average (the modeled-power meter).
    pbar1s_peak: f64,
    pbar1s_frame: f64,
    // Limits.
    amp_limit: f64,
    disp_limit: f64,
    vel_floor: f64,
    ignore_low: bool,
    ignore_high: bool,
    t_soft: f64,
    t_hard: f64,
    t_headroom: f64,
    t_max_db: f64,
    /// The coil passed TemperatureHardLimit (temperature axis enabled): the
    /// 20 dB fail-safe gain at once until it is back below the soft limit.
    hard_armed: bool,
    // Candidates and gains.
    cand: [f64; 5],
    band_target: [f32; BANDS],
    band_g: [f32; BANDS],
    peak_target: f32,
    peak_g: f32,
    slow_target: f32,
    slow_g: f32,
    /// The applied temperature gain (rig law 8, the confirmed part): cut to
    /// 0.99 x the linked candidate over the attack time when the candidate
    /// falls below it, else released toward it once per controller frame.
    temp_a: f32,
    temp_ramp: f32,
    temp_left: u32,
}

pub struct Unit {
    params: Params,
    run: Option<Run>,
}

struct Run {
    n: usize,
    latency: usize,
    /// The bank's centre tap.
    half: usize,
    /// Frames between the bank's centre and the output.
    look: usize,
    /// A model state went non-finite: the fail-safe is latched until reset.
    fault: bool,
    /// Controller frame and power substep, frames at this rate.
    ctl_frames: usize,
    sub_frames: usize,
    pos: usize,
    sub: usize,
    frame: u64,
    last_frame: u64,
    mitigate: bool,
    ctl: [bool; 5],
    link: bool,
    unlink_temp: bool,
    p_pole: [f64; 4],
    budget: [f64; 4],
    idle: f64,
    ratio: f64,
    r_attack: f32,
    r_fast: f32,
    r_low: f32,
    r_slow: f32,
    linked_power: f64,
    /// The temperature law's attack ramp (frames) and per-frame release.
    attack_frames: u32,
    temp_release: f32,
    ch: Vec<Chan>,
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

/// Bilinear models `X/U` and `V/U` of one block, `R = Rdc` (see the
/// TODO(SPEC) on the model resistance).
fn motion(rdc: f32, bl: f32, mms: f32, rms: f32, kms: f32, rate: f64, vel: bool) -> Biquad {
    let (r, bl, m, rm, k) = (rdc as f64, bl as f64, mms as f64, rms as f64, kms as f64);
    let den = [r * k, r * rm + bl * bl, r * m];
    let num = if vel { [0.0, bl, 0.0] } else { [bl, 0.0, 0.0] };
    Biquad::bilinear(num, den, rate)
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
        let stage = p.filterbank_frames();
        let half = stage - 1;
        let look = latency - half;
        let dt = 1.0 / rate;
        let ctl_frames = controller_frames(p.sample_rate);
        let sub_frames = ctl_frames / SUBSTEPS;
        let ctl_dt = ctl_frames as f64 / rate;
        let sub_dt = sub_frames as f64 / rate;
        let mut ch = Vec::with_capacity(n);
        for c in &p.ch[..n] {
            let h = filterbank(rate, &c.band_center_hz, c.bandwidth, stage);
            let taps_rev: [Vec<f32>; BANDS - 1] = core::array::from_fn(|k| h[k].iter().rev().copied().collect());
            let (r, bl, m, rm, k) = (c.rdc as f64, c.bl as f64, c.mms_kg as f64, c.rms as f64, c.kms as f64);
            // I/U = (m s^2 + rm s + k) / (r m s^2 + (r rm + bl^2) s + r k)
            let current = Biquad::bilinear([k, rm, m], [r * k, r * rm + bl * bl, r * m], rate);
            let disp = motion(c.disp_rdc, c.disp_bl, c.disp_mms_kg, c.disp_rms, c.disp_kms, rate, false);
            let vel = motion(c.vel_rdc, c.vel_bl, c.vel_mms_kg, c.vel_rms, c.vel_kms, rate, true);
            let taps = c.pressure_taps as usize;
            let pressure_rev: Vec<f32> = (0..taps).map(|j| c.pressure_fir[taps - 1 - j]).collect();
            ch.push(Chan {
                src: c.source as usize,
                vfs: c.volts_full_scale as f64,
                hist: History::new(2 * half + 1 + look),
                taps_rev,
                disp,
                vel,
                pk_in: 0.0,
                pk_disp: 0.0,
                pk_vel: 0.0,
                #[cfg(feature = "lsm2-waterfill")]
                vel_band: [vel; BANDS],
                #[cfg(feature = "lsm2-waterfill")]
                pk_vel_band: [0.0; BANDS],
                pressure_rev,
                p_hist: History::new(taps.max(1)),
                mask_hp: Biquad::butter2(c.mask_band_hz[0] as f64, rate, true),
                mask_lp: Biquad::butter2(c.mask_band_hz[1] as f64, rate, false),
                mask_acc: 0.0,
                mask_db: 0.0,
                mask_init: false,
                vel_lim: c.velocity_limit.map(|v| v as f64),
                mask_knots: c.mask_energy_db.map(|v| v as f64),
                mask_a: pole(ctl_dt, c.mask_attack_s as f64),
                mask_af: pole(ctl_dt, c.mask_attack_fast_s as f64),
                mask_r: pole(ctl_dt, c.mask_release_s as f64),
                mask_range: c.mask_attack_range_db as f64,
                vel_limit_now: c.velocity_limit[0] as f64,
                current,
                current_in: current,
                sub_acc_in: 0.0,
                pbar_in: [0.0; 4],
                pbar_in_peak: [0.0; 4],
                load_scale: (c.rdc as f64 + c.parasitic_resistance as f64) / c.rdc as f64,
                rth_vc: c.rth_vc as f64,
                rth_mag: c.rth_mag as f64,
                a_vc: (1.0 - pole(sub_dt, c.tau_vc as f64)) as f32,
                b_mag: pole(sub_dt, c.tau_mag as f64) as f32,
                a_mag: (1.0 - pole(sub_dt, c.tau_mag as f64)) as f32,
                tau_vc: c.tau_vc as f64,
                tau_mag: c.tau_mag as f64,
                ambient: c.ambient as f64,
                th_vc: 0.0,
                th_mag: 0.0,
                sub_acc: 0.0,
                pbar: [0.0; 4],
                pbar1_peak: 0.0,
                pbar1_frame: 0.0,
                pbar1s_peak: 0.0,
                pbar1s_frame: 0.0,
                amp_limit: c.amplitude_limit as f64 - c.pilot_reserve as f64,
                disp_limit: c.displacement_limit as f64,
                vel_floor: db_to_lin(c.max_velocity_attenuation_db as f64),
                ignore_low: c.velocity_ignore_low_band != 0,
                ignore_high: c.velocity_ignore_high_band != 0,
                t_soft: c.temperature_soft_limit as f64,
                t_hard: c.temperature_hard_limit as f64,
                t_headroom: c.temperature_headroom as f64,
                t_max_db: c.max_temperature_attenuation_db as f64,
                hard_armed: false,
                cand: [1.0; 5],
                band_target: [1.0; BANDS],
                band_g: [1.0; BANDS],
                peak_target: 1.0,
                peak_g: 1.0,
                slow_target: 1.0,
                slow_g: 1.0,
                temp_a: 1.0,
                temp_ramp: 0.0,
                temp_left: 0,
            });
        }
        let mut p_pole = [0.0; 4];
        let mut budget = [0.0; 4];
        for j in 0..4 {
            p_pole[j] = pole(sub_dt, p.power_time_scale_s[j] as f64);
            let b = if p.runtime_budget_present != 0 { p.runtime_budget_w[j] } else { p.power_hard_limit_w[j] };
            budget[j] = b as f64 * p.power_budgets_scale as f64;
        }
        let sample = |ms: f32| pole(dt, ms as f64 * 1e-3) as f32;
        Ok(Run {
            n,
            latency,
            half,
            look,
            fault: false,
            ctl_frames,
            sub_frames,
            pos: 0,
            sub: 0,
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
            idle: p.low_rail_idle_power_w as f64,
            ratio: p.low_rail_power_ratio as f64,
            r_attack: sample(p.attack_time_ms),
            r_fast: sample(p.fast_release_time_ms),
            r_low: sample(p.low_band_release_time_ms),
            r_slow: sample(p.slow_release_time_ms),
            linked_power: 1.0,
            attack_frames: libm::round(p.attack_time_ms as f64 * 1e-3 * rate) as u32,
            temp_release: (1.0
                - libm::exp(-libm::log(100.0) * ctl_frames as f64 / (rate * p.slow_release_time_ms as f64 * 1e-3)))
                as f32,
            ch,
        })
    }

    fn reset(&mut self) {
        for c in self.ch.iter_mut() {
            c.hist.clear();
            c.disp.clear();
            c.vel.clear();
            c.pk_in = 0.0;
            c.pk_disp = 0.0;
            c.pk_vel = 0.0;
            #[cfg(feature = "lsm2-waterfill")]
            {
                c.vel_band.iter_mut().for_each(Biquad::clear);
                c.pk_vel_band = [0.0; BANDS];
            }
            c.p_hist.clear();
            c.mask_hp.clear();
            c.mask_lp.clear();
            c.mask_acc = 0.0;
            c.mask_db = 0.0;
            c.mask_init = false;
            c.vel_limit_now = c.vel_lim[0];
            c.current.clear();
            c.current_in.clear();
            c.sub_acc_in = 0.0;
            c.pbar_in = [0.0; 4];
            c.pbar_in_peak = [0.0; 4];
            c.th_vc = 0.0;
            c.th_mag = 0.0;
            c.sub_acc = 0.0;
            c.pbar = [0.0; 4];
            c.pbar1_peak = 0.0;
            c.pbar1_frame = 0.0;
            c.pbar1s_peak = 0.0;
            c.pbar1s_frame = 0.0;
            c.cand = [1.0; 5];
            c.hard_armed = false;
            c.band_target = [1.0; BANDS];
            c.band_g = [1.0; BANDS];
            c.peak_target = 1.0;
            c.peak_g = 1.0;
            c.slow_target = 1.0;
            c.slow_g = 1.0;
            c.temp_a = 1.0;
            c.temp_ramp = 0.0;
            c.temp_left = 0;
        }
        self.pos = 0;
        self.sub = 0;
        self.frame = 0;
        self.last_frame = 0;
        self.linked_power = 1.0;
        self.fault = false;
    }

    /// One input sample of one channel through the analysis paths; returns
    /// the protected output sample.
    #[inline]
    fn sample(&mut self, ci: usize, x: f32) -> f32 {
        let (mitigate, look, half) = (self.mitigate, self.look, self.half);
        let (r_attack, r_fast, r_low, r_slow) = (self.r_attack, self.r_fast, self.r_low, self.r_slow);
        let c = &mut self.ch[ci];
        // Everything sees the drive as delivered (unclamped: the peak axis
        // gives a sample above full scale the gain that brings it to the
        // limit), bounded only at METER_FS; a non-finite sample is 0 to the
        // models and the most restrictive to the peak axis.
        c.pk_in = c.pk_in.max(if x.is_finite() { dsp::meter1(x).abs() } else { dsp::METER_FS });
        let x = dsp::meter1(x);
        c.hist.push(x);
        // The program the bands resynthesise, at the bank's centre.
        let centre = c.hist.ago(half);
        let u = c.vfs * centre as f64;
        let xd = c.disp.step(u);
        let v = c.vel.step(u);
        c.sub_acc_in += u * c.current_in.step(u);
        c.pk_disp = c.pk_disp.max(libm::fabs(xd) as f32);
        c.pk_vel = c.pk_vel.max(libm::fabs(v) as f32);
        #[cfg(feature = "lsm2-waterfill")]
        {
            // each band's velocity at the bank's centre (band 4 the rest)
            let w = c.hist.window();
            let w = &w[w.len() - (2 * half + 1)..];
            let mut rest = centre;
            for k in 0..BANDS {
                let b = if k < BANDS - 1 { fir_reversed(&c.taps_rev[k], w) } else { rest };
                rest -= b;
                let vk = c.vel_band[k].step(c.vfs * b as f64);
                c.pk_vel_band[k] = c.pk_vel_band[k].max(libm::fabs(vk) as f32);
            }
        }
        // Masking path.
        if !c.pressure_rev.is_empty() {
            c.p_hist.push(x);
            let p = fir_reversed(&c.pressure_rev, c.p_hist.window()) as f64;
            let m = c.mask_lp.step(c.mask_hp.step(p));
            c.mask_acc += m * m;
        }
        // Output: the delayed input with one gain per band, each the
        // minimum of its band gain and the broadband (peak, slow) gains
        // (spec 2: candidates combine by min, never by product). Bands 0..3
        // by FIR at the output's window, band 4 the complement:
        // y = e4 dry + sum_k<4 (e_k - e4) c_k.
        let dry = c.hist.ago(half + look);
        let mut y = dry;
        if mitigate {
            for k in 0..BANDS {
                let r = if k == 0 { r_low } else { r_fast };
                smooth(&mut c.band_g[k], c.band_target[k], r_attack, r);
            }
            smooth(&mut c.peak_g, c.peak_target, r_attack, r_fast);
            smooth(&mut c.slow_g, c.slow_target, r_attack, r_slow);
            if c.temp_left > 0 {
                c.temp_a += c.temp_ramp;
                c.temp_left -= 1;
            }
            let backstop = if c.hard_armed { FAILSAFE_GAIN } else { 1.0 };
            let broad = c.peak_g.min(c.slow_g).min(c.temp_a).min(backstop);
            let e: [f32; BANDS] = core::array::from_fn(|k| c.band_g[k].min(broad));
            let e4 = e[BANDS - 1];
            if e.iter().any(|&g| g != 1.0) {
                let w = c.hist.window();
                let w = &w[w.len() - look - (2 * half + 1)..w.len() - look];
                y = e4 * dry;
                for (taps, &g) in c.taps_rev.iter().zip(&e) {
                    if g != e4 {
                        y += (g - e4) * fir_reversed(taps, w);
                    }
                }
            }
        }
        // Monitor on the protected output as the reference produces it
        // (unclamped, within METER_FS times the gain) ...
        let u = c.vfs * y as f64;
        let i = c.current.step(u);
        let p = u * i;
        c.sub_acc += p;
        // ... and the output held to digital full scale: an over-range
        // program and band corrections on a full-scale input can exceed it.
        y.clamp(-1.0, 1.0)
    }

    /// The 32-frame power substep of every channel.
    fn substep(&mut self) {
        for c in self.ch.iter_mut() {
            let p = c.sub_acc / self.sub_frames as f64;
            c.sub_acc = 0.0;
            // Thermal (rig law 3): the substep mean of u*i, two independent
            // matched one-poles in Float32, the coil in series on the magnet.
            c.th_mag = c.b_mag * c.th_mag + (c.a_mag as f64 * p * c.rth_mag) as f32;
            c.th_vc += (c.a_vc as f64 * (p * c.rth_vc - c.th_vc as f64)) as f32;
            let load = p * c.load_scale;
            let pamp = self.idle + self.ratio * load.max(0.0);
            for j in 0..4 {
                c.pbar[j] = self.p_pole[j] * c.pbar[j] + (1.0 - self.p_pole[j]) * pamp;
            }
            c.pbar1_peak = c.pbar1_peak.max(c.pbar[0]);
            c.pbar1s_peak = c.pbar1s_peak.max(c.pbar[2]);
            let load_in = c.sub_acc_in / self.sub_frames as f64 * c.load_scale;
            c.sub_acc_in = 0.0;
            let pin = self.idle + self.ratio * load_in.max(0.0);
            for j in 0..4 {
                c.pbar_in[j] = self.p_pole[j] * c.pbar_in[j] + (1.0 - self.p_pole[j]) * pin;
                c.pbar_in_peak[j] = c.pbar_in_peak[j].max(c.pbar_in[j]);
            }
        }
    }

    /// Controller update.
    fn control(&mut self) {
        let ratio = |limit: f64, q: f32| (limit / (q as f64).max(EPS)).min(1.0);
        // Power (rig law 7): one gain for all systems. Each budget is the
        // total over the N systems; the gain scales only the load part of
        // the affine amplifier power, and each system's demand is the
        // largest unattenuated average over the frame:
        //   g_j = min(1, sqrt((B_j - N idle) / sum_ch (Phat_j - idle))).
        // MaxPowerAttenuation does not bound it.
        let g_pow = if self.ctl[3] {
            let mut g = 1.0f64;
            let nidle = self.n as f64 * self.idle;
            for j in 0..4 {
                let demand: f64 = self.ch.iter().map(|c| c.pbar_in_peak[j] - self.idle).sum();
                if demand > 0.0 {
                    g = g.min(libm::sqrt((self.budget[j] - nidle).max(0.0) / demand));
                }
            }
            g
        } else {
            1.0
        };
        for c in self.ch.iter_mut() {
            c.pbar1_frame = c.pbar1_peak;
            c.pbar1_peak = 0.0;
            c.pbar1s_frame = c.pbar1s_peak;
            c.pbar1s_peak = 0.0;
            // velocity limit from the mask level
            if !c.pressure_rev.is_empty() {
                let e = 10.0 * libm::log10((c.mask_acc / self.ctl_frames as f64).max(1e-30));
                if !c.mask_init {
                    c.mask_db = e;
                    c.mask_init = true;
                } else {
                    let r = if e > c.mask_db {
                        if e - c.mask_db >= c.mask_range {
                            c.mask_af
                        } else {
                            c.mask_a
                        }
                    } else {
                        c.mask_r
                    };
                    c.mask_db = e + r * (c.mask_db - e);
                }
                c.mask_acc = 0.0;
                c.vel_limit_now = mask_curve(c.mask_db, &c.mask_knots, &c.vel_lim);
            }
            let g_peak = if self.ctl[0] { ratio(c.amp_limit, c.pk_in) } else { 1.0 };
            let g_disp = if self.ctl[1] { ratio(c.disp_limit, c.pk_disp) } else { 1.0 };
            let g_vel = if self.ctl[2] { ratio(c.vel_limit_now, c.pk_vel).max(c.vel_floor) } else { 1.0 };
            #[cfg(not(feature = "lsm2-waterfill"))]
            for k in 0..BANDS {
                let ignored = (k == 0 && c.ignore_low) || (k == BANDS - 1 && c.ignore_high);
                c.band_target[k] = (if ignored { g_disp } else { g_disp.min(g_vel) }) as f32;
            }
            #[cfg(feature = "lsm2-waterfill")]
            {
                // PROVISIONAL rig law 5: the attenuable bands are levelled at
                // a common share s so the band velocities sum to the limit:
                // sum_k min(p_k, s) + sum_ignored p_k = Vlim, g_k = min(1, s/p_k)
                let gv = waterfill(&c.pk_vel_band, c.vel_limit_now, c.ignore_low, c.ignore_high, c.vel_floor);
                for (t, &g) in c.band_target.iter_mut().zip(&gv) {
                    let v = if self.ctl[2] && g_vel < 1.0 { g } else { 1.0 };
                    *t = g_disp.min(v) as f32;
                }
                c.pk_vel_band = [0.0; BANDS];
            }
            c.pk_in = 0.0;
            c.pk_disp = 0.0;
            c.pk_vel = 0.0;
            c.pbar_in_peak = [0.0; 4];
            let t_coil = c.ambient + c.th_mag as f64 + c.th_vc as f64;
            let g_temp =
                if self.ctl[4] { temperature_gain(t_coil, c.t_headroom, c.t_soft, c.t_hard, c.t_max_db) } else { 1.0 };
            // hard-limit backstop of the temperature axis (beyond the
            // reference, which stops at the knee's cap), with hysteresis
            // down to the soft limit
            if !self.ctl[4] {
                c.hard_armed = false;
            } else if t_coil > c.t_hard || t_coil.is_nan() {
                c.hard_armed = true;
            } else if t_coil < c.t_soft.min(c.t_hard) {
                c.hard_armed = false;
            }
            c.cand = [g_peak, g_disp, g_vel, g_pow, g_temp];
            c.peak_target = g_peak as f32;
            // the slow smoother carries power; temperature has its own law
            c.slow_target = g_pow as f32;
            // any non-finite model or controller state latches the fail-safe
            let model = [c.th_vc as f64, c.th_mag as f64, c.mask_db]
                .iter()
                .chain(&c.pbar)
                .chain(&c.pbar_in)
                .all(|v| v.is_finite());
            let motion = [&c.disp, &c.vel, &c.current].iter().all(|b| b.y[0].is_finite());
            let gains = c.cand.iter().all(|v| v.is_finite())
                && c.band_g.iter().all(|v| v.is_finite())
                && c.peak_g.is_finite()
                && c.slow_g.is_finite()
                && c.temp_a.is_finite();
            if !(model && motion && gains) {
                self.fault = true;
            }
        }
        let mut pow = 1.0f64;
        for c in &self.ch {
            pow = pow.min(c.cand[3]);
        }
        self.linked_power = if self.link { pow } else { f64::NAN };
        if self.link && self.n > 1 {
            for c in self.ch.iter_mut() {
                c.slow_target = pow as f32;
            }
        }
        // Temperature (rig law 8): the applied gain follows the minimum over
        // the linked channels of the logged candidates. Below the applied
        // gain the target is 0.99 x the candidate, reached over the attack
        // time from this frame boundary; otherwise one 99 %-settling release
        // step per controller frame toward the candidate.
        let linked_temp = self.link && !self.unlink_temp;
        let gm_all = self.ch.iter().map(|c| c.cand[4]).fold(1.0f64, f64::min);
        for c in self.ch.iter_mut() {
            let gm = (if linked_temp { gm_all } else { c.cand[4] }) as f32;
            if gm < c.temp_a {
                let target = 0.99 * gm;
                c.temp_left = self.attack_frames.max(1);
                c.temp_ramp = (target - c.temp_a) / c.temp_left as f32;
            } else if c.temp_left == 0 {
                c.temp_a += self.temp_release * (gm - c.temp_a);
                if (gm - c.temp_a).abs() <= f32::EPSILON * gm {
                    c.temp_a = gm;
                }
            }
        }
    }

    fn process(&mut self, io: &mut Io<'_, '_>) {
        for k in 0..io.frames {
            for ci in 0..self.n {
                let src = self.ch[ci].src;
                let y = self.sample(ci, io.input[src][k]);
                io.output[src][k] = y;
            }
            self.sub += 1;
            if self.sub == self.sub_frames {
                self.sub = 0;
                self.substep();
            }
            self.pos += 1;
            self.frame += 1;
            if self.pos == self.ctl_frames {
                self.pos = 0;
                self.control();
                self.last_frame = self.frame - self.ctl_frames as u64;
                if self.fault {
                    crate::failsafe::fill_from(io, k + 1, FAILSAFE_GAIN);
                    return;
                }
            }
        }
    }
}

/// PROVISIONAL rig law 5: per-band velocity gains water-filled to the
/// limit. `p` are the bands' velocity peaks; ignored outer bands keep unity
/// and their share counts in full.
#[cfg(feature = "lsm2-waterfill")]
fn waterfill(p: &[f32; BANDS], vlim: f64, ignore_low: bool, ignore_high: bool, floor: f64) -> [f64; BANDS] {
    let ign = |k: usize| (k == 0 && ignore_low) || (k == BANDS - 1 && ignore_high);
    let fixed: f64 = (0..BANDS).filter(|&k| ign(k)).map(|k| p[k] as f64).sum();
    let budget = (vlim - fixed).max(0.0);
    let used = |s: f64| (0..BANDS).filter(|&k| !ign(k)).map(|k| (p[k] as f64).min(s)).sum::<f64>();
    let top = (0..BANDS).filter(|&k| !ign(k)).map(|k| p[k] as f64).fold(0.0, f64::max);
    let mut g = [1.0f64; BANDS];
    if used(top) <= budget {
        return g;
    }
    // bisection on the share s in [0, top]
    let (mut lo, mut hi) = (0.0f64, top);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if used(mid) > budget {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    for k in 0..BANDS {
        if !ign(k) && p[k] as f64 > lo {
            g[k] = (lo / p[k] as f64).max(floor);
        }
    }
    g
}

/// The linear-in-dB temperature knee of spec 6.1. Equal limits form a step;
/// reversed or non-finite inputs take the capped branch.
pub fn temperature_gain(t: f64, headroom: f64, soft: f64, hard: f64, max_db: f64) -> f64 {
    let x = t + headroom - soft;
    let span = hard - soft;
    let z = if !(x.is_finite() && span.is_finite()) || span < 0.0 {
        1.0
    } else if span == 0.0 {
        if x >= 0.0 {
            1.0
        } else {
            0.0
        }
    } else {
        (x / span).clamp(0.0, 1.0)
    };
    libm::pow(10.0, -(max_db * z) / 20.0)
}

/// The velocity limit of spec 6.2 for smoothed mask level `e` dB.
pub fn mask_curve(e: f64, k: &[f64; 4], v: &[f64; 3]) -> f64 {
    let lerp = |a: f64, b: f64, t: f64| a + (b - a) * t;
    if e <= k[0] {
        v[0]
    } else if e < k[1] {
        lerp(v[0], v[1], (e - k[0]) / (k[1] - k[0]))
    } else if e <= k[2] {
        v[1]
    } else if e < k[3] {
        lerp(v[1], v[2], (e - k[2]) / (k[3] - k[2]))
    } else {
        v[2]
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
        let (mut amp_1s, mut amp_now) = (0.0f64, 0.0f64);
        for c in &run.ch {
            let mut s = ChannelState::ABSENT;
            let t = c.ambient + c.th_mag as f64 + c.th_vc as f64;
            s.modeled_power_w = c.pbar[2] as f32;
            s.coil_temp_c = t as f32;
            s.magnet_temp_c = (c.ambient + c.th_mag as f64) as f32;
            s.control_temp_c = t as f32;
            s.ol_temp_c = t as f32;
            s.velocity_limit_m_s = c.vel_limit_now as f32;
            s.mask_db = if c.mask_init { c.mask_db as f32 } else { f32::NAN };
            for (a, &g) in s.axis_gain.iter_mut().zip(&c.cand) {
                *a = g as f32;
            }
            if run.linked_power.is_finite() {
                s.axis_gain[3] = run.linked_power as f32;
            }
            s.band_gain = c.band_g;
            let backstop = if c.hard_armed { FAILSAFE_GAIN } else { 1.0 };
            s.gain = if run.fault { FAILSAFE_GAIN } else { c.peak_g.min(c.slow_g).min(c.temp_a).min(backstop) };
            out.channels[c.src] = s;
            amp_1s += c.pbar1s_frame;
            if c.src == 0 {
                amp_now = c.pbar1_frame;
            }
        }
        let g = &mut out.global;
        g.power_hard_limit_w = run.budget.map(|b| b as f32);
        g.power_gain = (if run.linked_power.is_finite() { run.linked_power } else { 1.0 }) as f32;
        g.modeled_amp_power_w = amp_1s as f32;
        g.amp_input_power_w = amp_now as f32;
    }
    fn reset(&mut self) {
        if let Some(r) = self.run.as_mut() {
            r.reset();
        }
    }

    fn save_state(&self, out: &mut HandOff) -> bool {
        let Some(run) = self.run.as_ref().filter(|r| !r.fault) else { return false };
        *out = HandOff::EMPTY;
        out.kind = UnitKind::Lsm2 as u32;
        out.channels = run.n as u32;
        out.sample_rate = self.params.sample_rate;
        for (i, c) in run.ch.iter().enumerate() {
            let n = &mut out.node[i];
            // rises (coil over magnet, magnet over ambient); a node below
            // its reference carries no heat
            n[0] = (c.th_vc as f64).max(0.0);
            n[1] = (c.th_mag as f64).max(0.0);
            n[2..6].copy_from_slice(&c.pbar);
            out.gain[i] = c.slow_g.min(c.temp_a) as f64;
        }
        true
    }

    fn restore_state(&mut self, state: &HandOff, gap_s: f64) -> bool {
        let rate = self.params.sample_rate;
        let Some(run) = self.run.as_mut() else { return false };
        if !state.fits(UnitKind::Lsm2, run.n, rate) {
            return false;
        }
        let t = gap_or_zero(gap_s);
        let samples = t * rate as f64;
        let substeps = samples / run.sub_frames as f64;
        for (i, c) in run.ch.iter_mut().enumerate() {
            let s = &state.node[i];
            // the series nodes cool independently, each with its own constant
            c.th_vc = (s[0] * libm::exp(-t / c.tau_vc)) as f32;
            c.th_mag = (s[1] * libm::exp(-t / c.tau_mag)) as f32;
            for j in 0..4 {
                c.pbar[j] = run.idle + (s[2 + j] - run.idle) * libm::pow(run.p_pole[j], substeps);
            }
            let g = release_toward_unity(state.gain[i], run.r_slow as f64, samples) as f32;
            c.slow_g = g;
            c.slow_target = g;
            c.temp_a = g;
        }
        true
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
            low_band_release_time_ms: 150.0,
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
                disp_rdc: 4.0,
                disp_bl: 0.8,
                disp_mms_kg: 0.00022,
                disp_rms: 0.25,
                disp_kms: 1600.0,
                vel_rdc: 4.0,
                vel_bl: 0.8,
                vel_mms_kg: 0.0002,
                vel_rms: 0.2,
                vel_kms: 1500.0,
                rth_vc: 30.0,
                tau_vc: 2.0,
                rth_mag: 20.0,
                tau_mag: 60.0,
                ambient: 40.0,
                amplitude_limit: 1.0,
                displacement_limit: 0.0005,
                velocity_limit: [0.5, 0.6, 0.9],
                mask_energy_db: [-40.0, -30.0, -20.0, -10.0],
                mask_band_hz: [8000.0, 12000.0],
                mask_attack_s: 0.01,
                mask_release_s: 0.1,
                mask_attack_fast_s: 0.005,
                mask_attack_range_db: 70.0,
                max_velocity_attenuation_db: 18.0,
                temperature_soft_limit: 110.0,
                temperature_hard_limit: 125.0,
                temperature_headroom: 4.0,
                max_temperature_attenuation_db: 10.0,
                band_center_hz: [60.0, 300.0, 650.0, 1600.0, 5000.0],
                ..ChannelParams::default()
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

    /// A non-finite state latches 20 dB until reset.
    #[test]
    fn a_non_finite_model_state_latches_the_failsafe_until_reset() {
        let poisons: [fn(&mut Run); 3] =
            [|r| r.ch[0].th_mag = f32::NAN, |r| r.ch[1].pbar[2] = f64::INFINITY, |r| r.ch[0].slow_g = f32::NAN];
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
            let mut rec = HandOff::EMPTY;
            assert!(!u.save_state(&mut rec));
            u.reset();
            let (o, h) = run(&mut u, 0.01, 1024);
            assert_eq!((o[1023], h), (0.01, Health::OK));
        }
    }
}
