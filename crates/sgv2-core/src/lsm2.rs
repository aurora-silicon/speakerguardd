//! `lsm2`: signal-only loudspeaker model and five-axis constraint
//! controller (MacBook Neo, MacBook Air M4/M5), from the behaviour
//! specification (SPEC-LSM2 v1.1 with the section 15 binary closure of
//! 2026-11-05) and the 25G83 differential-rig laws. This is the model-only
//! configuration the Neo ships (static model, system ID bypassed, no
//! closed-loop temperature); a tuning that asks for the DSP estimator or
//! closed-loop temperature is refused.
//!
//! The unit is frame-synchronous, as the reference is: every controller
//! frame `j` (`N = 256` frames at 48 kHz, 5.333 ms) takes the frame's whole
//! input, decides, and produces output frame `j`. The reference does this
//! inside the host's render call, so its output depends on the host slice
//! (25G83 rig: slices of 512, 1024 and 4096 frames are bit-identical; 1, 7
//! and 160 differ). This unit buffers one controller frame instead, so its
//! output never depends on how the host slices the stream, and equals the
//! reference rendered at its production slicing, delayed by exactly one
//! controller frame. The reported latency is therefore
//! `2 * FilterbankLatency + SpeakerResponseLookAhead + N` (352 + 256 = 608
//! frames at 48 kHz), and PipeWire compensates it.
//!
//! Per channel, with `FB = FilterbankLatency` (160 frames), `OD = FB +
//! SpeakerResponseLookAhead` (192) and `W = N + OD` (448):
//!
//! ```text
//! x ─┬─ analysis bank (319 centred taps, delay FB) ─ a[k] ─┬─ delay OD ─ Σ g[k]·a[k] ─ y
//!    │                                                     ├─ x/v models per band ─ frame peaks ─ candidates
//!    │                                                     └─ Σ a[k] (before any gain) ─ u·i ─ power, temperature axes
//!    └─ pressure FIR ─ 8–12 kHz 4th-order edges ─ frame mean square ─ dB ─ one limit per pair
//! y ─ u = Vfs·y ─ current model ─ u·i ─ four averages, two series thermal nodes (the monitor)
//! ```
//!
//! - **Models** (rig laws 1 and 4, spec 15.1 and 15.2). Displacement and
//!   velocity are the plain bilinear versions of `X/U = Bl / (R (Mms s^2 +
//!   Rms s + Kms) + Bl^2 s)` and `V = s X`, each block with its own `RdcOhms`
//!   (`ParasiticResistance` enters no model). A block whose
//!   `UseGenericModelParameters*` flag is set takes the tuning's embedded
//!   family row (the compiler resolves it). The current model is a Float32
//!   biquad whose zeros are the exponentially mapped mechanical roots and
//!   whose poles are the exponentially mapped electrical ones, with
//!   `1/Rdc` at DC.
//! - **Filterbank** (rig law 10): the raised-cosine crossfade bank on the
//!   design grid of `M = 2 FB` points, each centre snapped to a bin, the
//!   `M/2` tap dropped (319 taps), generated in f64 and rounded once to f32.
//!   Band 4 is the delayed input minus the other four, so the bank sums to
//!   the delay exactly and unity gains give the input back bit for bit.
//! - **Window** (25G83 rig, exact on the frame logs): frame `j` evaluates
//!   the bank-stage stream over the `W = 448` frames that end with the
//!   frame, i.e. the output region of the frame (`N` frames) plus the `OD`
//!   frames of look-ahead behind it, in 32-frame substeps.
//! - **Candidates**, linear in [0, 1], a disabled axis giving 1:
//!   - sample peak (rig law 6): `min(1, (AmplitudeLimit - pilot) / max|x|)`;
//!   - displacement: `min(1, DisplacementLimit / max|x_d|)` on the
//!     full-band model, applied to every band;
//!   - velocity (spec 15.3, 15.4): the logged candidate is the full-band
//!     `Vlim / max|v|`; the band gains water-fill the five band velocities'
//!     frame peaks scaled by the applied slow gain, `sum min(s pk, L) =
//!     Vlim` over the active bands, floored at `-MaxVelocityAttenuation`, so
//!     the output velocity stays at the limit while a thermal gain falls;
//!     the limit follows the masking curve of spec 6.2 driven by the
//!     pressure-filtered 8–12 kHz program, one level for the pair;
//!   - power (rig law 7): one gain for all systems, `min_j min(1, sqrt((B_j -
//!     N idle) / sum (Phat_j - idle)))`, `Phat_j` each system's largest
//!     average over the window of the bank-stage program before any gain
//!     (input-referred, rig law 4);
//!   - temperature (spec 15.5): the axis's own series predictor driven by
//!     the input-referred power times its own gain squared, the regulating
//!     target `q^2 = clamp((Thard - Tamb - theta_mag) / (Rvc Pin + 1), 1e-4,
//!     1)` once the prediction strictly exceeds `TemperatureSoftLimit`, and a
//!     linear release in `gt^2`.
//! - **Gains.** Fast (band, peak): below the applied gain the target is 0.99
//!   x the request, ramped linearly over `AttackTime`; otherwise a
//!   per-sample release toward it (99 % settling over `FastReleaseTime`,
//!   `LowBandReleaseTime` for band 0). Slow (power, temperature, linked
//!   across channels): the same 0.99 attack; otherwise one 99 %-settling
//!   `SlowReleaseTime` step per frame (rig law 8). Before committing, the
//!   slow gain is checked against the output-referred monitor: if the
//!   frame would take the hottest coil above `TemperatureHardLimit +
//!   max(TemperatureHeadroom, 0)`, it is cut to `0.99 sqrt((C - Tamb -
//!   theta_mag) / (Rvc Pin))`, at most four passes (spec 15.5). Each band's
//!   applied gain is the minimum of its fast gain, the peak gain and the
//!   slow gain (spec 2: never a product).
//! - **Monitor** (rig laws 3, 4, 11): on the protected output, `u = Vfs y`;
//!   the substep mean of `u i` drives the two series matched one-poles
//!   (Float32), and `Pamp = idle + ratio max(mean(u i), 0) (Rdc + Rpar) /
//!   Rdc` the four exponential averages the meters report.
//!
//! Full scale: the output is held to digital full scale, a non-finite input
//! sample is 0 to the audio and the models and the most restrictive to the
//! peak axis. A model state that goes non-finite latches the 20 dB
//! fail-safe (`Health::MODEL_FAULT`) until reset. `EnableMitigation = 0` and
//! a tuning with neither thermal axis are refused.
//!
//! Hand-off: `save_state`/`restore_state` carry the monitor's and the
//! temperature predictor's coil and magnet rises, the four power averages
//! and the slow gain to the next stream, cooled for the gap.
//!
//! Known residuals against 25G83 (rig report; each errs on the protective
//! side):
//! - the current model's poles: the reference's are about 0.9 % more damped
//!   than the electrical poles give (spec 15.2's Delta-sigma; its coefficient
//!   builder is not specified), so near the resonance this unit models up to
//!   0.9 % more power;
//! - the temperature candidate: spec 15.5's target cuts at once on a steady
//!   tone where the reference approaches it over seconds;
//! - the mask level: the masker's band edges and dB reference are fitted on
//!   the rig (`MASK_DB_OFFSET`), and the limit is held at `VelocityLimit2`
//!   where the reference's active limit rises above it on loud music.

// The DSP loops index several parallel per-band and per-channel arrays at
// once; an index reads better there than zipped iterators.
#![allow(clippy::needless_range_loop)]

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::{
    dsp, gap_or_zero, ChannelState, HandOff, Health, Io, ProtectionUnit, Telemetry, UnitKind, FAILSAFE_GAIN,
    MAX_CHANNELS,
};

/// Controller frame in frames at 48 kHz (spec 10: 256). The cadence is a
/// time, 5.333 ms: 512 frames at 96 kHz, see [`controller_frames`].
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
/// The 0.99 downward target of every gain law (rig laws 5, 8; spec 15.4).
const CUT: f64 = 0.99;
/// The mask level's dB offset: `E = 10 log10(Ems) + MASK_DB_OFFSET`.
/// Spec 15.3 names 20 uPa (+94 dB); on the 25G83 rig that reference puts a
/// 1 kHz tone with no 8-12 kHz content far above every knot (the reference
/// stays unmasked there, `lsm2-law-fb-vel-*`), and the active velocity limit
/// of the music traces is matched best (median 0.07 m/s within the curve,
/// `lsm2-tokyo_30s`) by this offset with fourth-order edges. Interim, from
/// the rig, until the binary closure gives the exact level (Daybreak
/// follow-up): a lower level only ever lowers the velocity limit.
const MASK_DB_OFFSET: f64 = 30.6;
/// Ceiling correction passes per frame (spec 15.5).
const CEILING_PASSES: usize = 4;

/// Bits of [`ChannelParams::model_generic`]: the manager's
/// `UseGenericModelParametersRdc` and `...Resonator1` flags. A generic field
/// takes the tuning's embedded family row (resolved by the compiler) and
/// does not follow later writes of the block's own field (rig law 1).
pub mod companion;

pub mod generic {
    pub const ADM_RDC: u32 = 1 << 0;
    pub const ADM_RES1: u32 = 1 << 1;
    pub const DISP_RDC: u32 = 1 << 2;
    pub const DISP_RES1: u32 = 1 << 3;
    pub const VEL_RDC: u32 = 1 << 4;
    pub const VEL_RES1: u32 = 1 << 5;
}

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
    /// [`generic`] flags of the three models.
    pub model_generic: u32,
    /// The tuning's embedded family row (spec 15.1): Rdc, Bl, Mms (kg),
    /// Rms, Kms. A field whose generic flag is set takes its value from here.
    pub family: [f32; 5],
    /// Each model block's own fields, admittance, displacement, velocity, in
    /// the same order; the model fields above are these resolved against the
    /// flags (see [`ChannelParams::resolve_models`]).
    pub explicit: [[f32; 5]; 3],
    /// Number of pressure-response taps (0 without property 6632, else
    /// [`PRESSURE_TAPS`]).
    pub pressure_taps: u32,
    pub pressure_fir: [f32; PRESSURE_TAPS],
}

impl ChannelParams {
    /// Set the three models' fields from their blocks' own values and the
    /// family row, by the generic flags (spec 15.1, rig law 1).
    pub fn resolve_models(&mut self) {
        let flags = [
            (generic::ADM_RDC, generic::ADM_RES1),
            (generic::DISP_RDC, generic::DISP_RES1),
            (generic::VEL_RDC, generic::VEL_RES1),
        ];
        let mut r = [[0.0f32; 5]; 3];
        for (m, &(rdc_bit, res_bit)) in flags.iter().enumerate() {
            for k in 0..5 {
                let bit = if k == 0 { rdc_bit } else { res_bit };
                r[m][k] = if self.model_generic & bit != 0 { self.family[k] } else { self.explicit[m][k] };
            }
        }
        [self.rdc, self.bl, self.mms_kg, self.rms, self.kms] = r[0];
        [self.disp_rdc, self.disp_bl, self.disp_mms_kg, self.disp_rms, self.disp_kms] = r[1];
        [self.vel_rdc, self.vel_bl, self.vel_mms_kg, self.vel_rms, self.vel_kms] = r[2];
    }
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
            model_generic: 0,
            family: [0.0; 5],
            explicit: [[0.0; 5]; 3],
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

const CH_WORDS: usize = 74 + PRESSURE_TAPS;
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

/// The unit's AU parameter IDs (specification section 9.7): all global
/// scope, the per-type and per-channel fields encoded in the ID block.
pub mod ids {
    pub const ENABLE_CLOSED_LOOP_TEMPERATURE: u32 = 1;
    pub const BYPASS_SYS_ID: u32 = 2048 + 3;
    pub const CH_TUNING: u32 = 2048 + 4;
    pub const CH_SPEAKER_TYPE: u32 = 21504;
    /// Model managers within a type: impedance, admittance, displacement,
    /// velocity, acceleration.
    pub const MANAGER_IMPEDANCE: u32 = 0;
    pub const MANAGER_ADMITTANCE: u32 = 1;
    pub const MANAGER_DISPLACEMENT: u32 = 2;
    pub const MANAGER_VELOCITY: u32 = 3;
    pub fn model(manager: u32, ty: u32, k: u32) -> u32 {
        3072 + 1024 * (3 * manager + ty) + k
    }
    pub fn thermal(ty: u32, k: u32) -> u32 {
        18432 + 1024 * ty + k
    }
    /// Pilot blocks of tuning banks 0, 1, 2.
    pub const PILOT: [u32; 3] = [1024, 22528, 23552];
    pub const P_AMPLITUDE_DB: u32 = 2;
    pub fn controller(k: u32) -> u32 {
        1048576 + k
    }
    pub fn amp0(k: u32) -> u32 {
        1049600 + k
    }
    /// Amplifier models 1 and 2 (no `IgnoreCPMS` slot: offset 0 is the
    /// low-rail idle power).
    pub fn amp(model: u32, k: u32) -> u32 {
        1050624 + 1024 * (model - 1) + k
    }
    pub fn channel(c: u32, k: u32) -> u32 {
        1058816 + 1024 * c + k
    }
    // Model-block field offsets.
    pub const M_USE_GENERIC_RDC: u32 = 0;
    pub const M_RDC: u32 = 1;
    pub const M_ENABLE_RES1: u32 = 2;
    pub const M_USE_GENERIC_RES1: u32 = 3;
    pub const M_BL: u32 = 4;
    pub const M_MMS_G: u32 = 5;
    pub const M_RMS: u32 = 6;
    pub const M_KMS: u32 = 7;
    pub const M_ENABLE_RES2: u32 = 8;
    pub const M_ENABLE_LR2: u32 = 13;
    pub const M_CREEP: u32 = 18;
    pub const M_USE_COMPENSATION: u32 = 24;
    pub const M_PASSIVE_RADIATOR: u32 = 29;
    // Thermal-block offsets.
    pub const T_RTH_VC: u32 = 3;
    pub const T_TAU_VC: u32 = 4;
    pub const T_RTH_MG: u32 = 5;
    pub const T_TAU_MG: u32 = 6;
    pub const T_AMBIENT: u32 = 7;
    pub const T_ENABLE: u32 = 15;
    // Controller-general offsets.
    pub const C_FAST_RELEASE: u32 = 1;
    pub const C_SLOW_RELEASE: u32 = 2;
    pub const C_ENABLE_MITIGATION: u32 = 3;
    pub const C_CONTROL_SAMPLE_PEAK: u32 = 4;
    pub const C_CONTROL_DISPLACEMENT: u32 = 5;
    pub const C_CONTROL_VELOCITY: u32 = 6;
    pub const C_CONTROL_POWER: u32 = 7;
    pub const C_CONTROL_TEMPERATURE: u32 = 8;
    pub const C_LINK_SLOW: u32 = 10;
    pub const C_COMPENSATE_RESONATORS: u32 = 11;
    pub const C_USE_FAST_PEAK: u32 = 14;
    pub const C_LOOKAHEAD: u32 = 16;
    pub const C_ATTACK: u32 = 17;
    pub const C_LOW_BAND_RELEASE: u32 = 18;
    pub const C_UNLINK_TEMPERATURE: u32 = 19;
    pub const C_POWER_BUDGETS_SCALE: u32 = 20;
    pub const C_FILTERBANK_LATENCY: u32 = 21;
    // Amplifier model 0 offsets (1..12 are the model fields).
    pub const A_LOW_IDLE: u32 = 1;
    pub const A_LOW_RATIO: u32 = 2;
    pub const A_ENABLE_MID: u32 = 3;
    pub const A_ENABLE_HIGH: u32 = 8;
    pub const A_TIME_SCALE: u32 = 13;
    pub const A_HARD_LIMIT: u32 = 17;
    pub const A_SOFT_LIMIT: u32 = 21;
    pub const A_MAX_ATTENUATION: u32 = 25;
    pub const A_USE_BATTERY: u32 = 26;
    // Channel-block offsets.
    pub const CH_VOLTS_FS: u32 = 0;
    pub const CH_PARASITIC: u32 = 1;
    pub const CH_DISPLACEMENT_LIMIT: u32 = 2;
    pub const CH_TEMP_HARD: u32 = 3;
    pub const CH_TEMP_SOFT: u32 = 4;
    pub const CH_MASK_BAND_MIN: u32 = 5;
    pub const CH_MASK_BAND_MAX: u32 = 6;
    pub const CH_MASK_ATTACK: u32 = 7;
    pub const CH_MASK_RELEASE: u32 = 8;
    pub const CH_VELOCITY_LIMIT0: u32 = 9;
    pub const CH_MASK_ENERGY0: u32 = 12;
    pub const CH_AMPLITUDE_LIMIT: u32 = 16;
    pub const CH_BAND_CENTER0: u32 = 17;
    pub const CH_MAX_VELOCITY_ATTEN: u32 = 22;
    pub const CH_VELOCITY_IGNORE_LOW: u32 = 23;
    pub const CH_VELOCITY_IGNORE_HIGH: u32 = 24;
    pub const CH_MAX_TEMP_ATTEN: u32 = 25;
    pub const CH_TEMP_HEADROOM: u32 = 26;
    pub const CH_MASK_ATTACK_FAST_MS: u32 = 27;
    pub const CH_MASK_ATTACK_RANGE: u32 = 28;
    pub const CH_BANDWIDTH: u32 = 29;
    pub const CH_POWER_AMP_MODEL: u32 = 32;
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
        // The bank needs a centre tap and at least one on each side, and its
        // look-ahead region must fit in one controller frame.
        let ctl = controller_frames(self.sample_rate);
        if self.filterbank_frames() < 2 || self.latency_frames() > 16384 {
            return Err(InitError::Invalid("latency"));
        }
        if self.output_offset_frames() >= ctl || self.output_offset_frames() % (ctl / SUBSTEPS) != 0 {
            return Err(InitError::Invalid("look-ahead region against the controller cadence"));
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
                && c.temperature_headroom.is_finite()
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

    /// The speaker-response look-ahead in frames.
    pub fn lookahead_frames(&self) -> usize {
        libm::round(self.speaker_response_lookahead_ms as f64 * 1e-3 * self.sample_rate as f64) as usize
    }

    /// Frames between the bank stage and the output: one more filterbank
    /// stage plus the look-ahead (192 at 48 kHz).
    pub fn output_offset_frames(&self) -> usize {
        self.filterbank_frames() + self.lookahead_frames()
    }

    /// The reference unit's algorithmic latency in frames: two filterbank
    /// stages plus the speaker response look-ahead (352 at 48 kHz).
    pub fn latency_frames(&self) -> usize {
        2 * self.filterbank_frames() + self.lookahead_frames()
    }

    /// This unit's latency: the reference's plus the one controller frame
    /// it buffers to be independent of the host slicing (608 at 48 kHz).
    pub fn unit_latency_frames(&self) -> usize {
        self.latency_frames() + controller_frames(self.sample_rate)
    }

    /// The protection a unit built from parameters must carry: mitigation on
    /// and at least one thermal axis (safety review of #31, finding 7). The
    /// reference runs any configuration (with mitigation off it only delays
    /// the program), and so does the unit's `set_parameter` for the
    /// differential rig.
    pub fn check_protects(&self) -> Result<(), InitError> {
        if self.enable_mitigation == 0 {
            return Err(InitError::Invalid("mitigation disabled"));
        }
        if self.control_power == 0 && self.control_temperature == 0 {
            return Err(InitError::Invalid("no thermal axis enabled"));
        }
        Ok(())
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
        p.check_protects()?;
        Ok(p)
    }

    /// Set one AU parameter by its ID (section 9.7), as the host does after
    /// initialisation. False for an ID the payload does not carry, a value
    /// that is not a selector where one is needed, or a field the unit
    /// fixes at initialisation (`FilterbankLatency`: rig law 10). Fields a
    /// generic flag resolves keep the family row (rig law 1).
    pub fn set(&mut self, id: u32, v: f32) -> bool {
        use ids as d;
        let flag = |v: f32| (v != 0.0) as u32;
        let n = self.channels;
        if (d::controller(0)..=d::controller(22)).contains(&id) {
            let k = id - d::controller(0);
            match k {
                d::C_FAST_RELEASE => self.fast_release_time_ms = v,
                d::C_SLOW_RELEASE => self.slow_release_time_ms = v,
                d::C_ENABLE_MITIGATION => self.enable_mitigation = flag(v),
                d::C_CONTROL_SAMPLE_PEAK => self.control_sample_peak = flag(v),
                d::C_CONTROL_DISPLACEMENT => self.control_displacement = flag(v),
                d::C_CONTROL_VELOCITY => self.control_velocity = flag(v),
                d::C_CONTROL_POWER => self.control_power = flag(v),
                d::C_CONTROL_TEMPERATURE => self.control_temperature = flag(v),
                d::C_LINK_SLOW => self.link_slow_gains = flag(v),
                d::C_LOOKAHEAD => self.speaker_response_lookahead_ms = v,
                d::C_ATTACK => self.attack_time_ms = v,
                d::C_LOW_BAND_RELEASE => self.low_band_release_time_ms = v,
                d::C_UNLINK_TEMPERATURE => self.unlink_temperature_gains = flag(v),
                d::C_POWER_BUDGETS_SCALE => self.power_budgets_scale = v,
                // fixed at initialisation: a later write has no effect (rig law 10)
                d::C_FILTERBANK_LATENCY => {}
                _ => return false,
            }
            return true;
        }
        if (d::amp0(0)..=d::amp0(36)).contains(&id) {
            let k = id - d::amp0(0);
            match k {
                d::A_LOW_IDLE => self.low_rail_idle_power_w = v,
                d::A_LOW_RATIO => self.low_rail_power_ratio = v,
                k if (d::A_TIME_SCALE..d::A_TIME_SCALE + 4).contains(&k) => {
                    self.power_time_scale_s[(k - d::A_TIME_SCALE) as usize] = v
                }
                k if (d::A_HARD_LIMIT..d::A_HARD_LIMIT + 4).contains(&k) => {
                    self.power_hard_limit_w[(k - d::A_HARD_LIMIT) as usize] = v
                }
                k if (d::A_SOFT_LIMIT..d::A_SOFT_LIMIT + 4).contains(&k) => {
                    self.power_soft_limit_pct[(k - d::A_SOFT_LIMIT) as usize] = v
                }
                d::A_MAX_ATTENUATION => self.max_power_attenuation_db = v,
                _ => return false,
            }
            return true;
        }
        // Type-A pilot amplitude: the peak reserve of channels on bank 0.
        if id == d::PILOT[0] + d::P_AMPLITUDE_DB {
            for c in &mut self.ch[..n as usize] {
                c.pilot_reserve = libm::pow(10.0, v as f64 / 20.0) as f32;
            }
            return true;
        }
        // the rest of the pilot block: the reserve holds with the pilot off
        // or on (rig law 6), and this unit generates no pilot
        if (d::PILOT[0]..d::PILOT[0] + 8).contains(&id) {
            return true;
        }
        // Type-A thermal and model blocks (both channels select type A on
        // every lsm2 tuning the compiler accepts with one type).
        if (d::thermal(0, 0)..d::thermal(0, 16)).contains(&id) {
            let k = id - d::thermal(0, 0);
            for c in &mut self.ch[..n as usize] {
                match k {
                    d::T_RTH_VC => c.rth_vc = v,
                    d::T_TAU_VC => c.tau_vc = v,
                    d::T_RTH_MG => c.rth_mag = v,
                    d::T_TAU_MG => c.tau_mag = v,
                    d::T_AMBIENT => c.ambient = v,
                    _ => return false,
                }
            }
            return true;
        }
        // Type-A model blocks. The impedance block and the fields no model
        // uses (inductance, second resonator, creep, compensation, passive
        // radiator) are accepted and inert (rig law 2; the family row does
        // not follow the impedance block, rig law 1).
        if (d::model(d::MANAGER_IMPEDANCE, 0, 0)..d::model(d::MANAGER_IMPEDANCE, 0, 37)).contains(&id) {
            return true;
        }
        for (mi, m, rdc_bit, res_bit) in [
            (0usize, d::MANAGER_ADMITTANCE, generic::ADM_RDC, generic::ADM_RES1),
            (1, d::MANAGER_DISPLACEMENT, generic::DISP_RDC, generic::DISP_RES1),
            (2, d::MANAGER_VELOCITY, generic::VEL_RDC, generic::VEL_RES1),
        ] {
            if !(d::model(m, 0, 0)..d::model(m, 0, 37)).contains(&id) {
                continue;
            }
            let k = id - d::model(m, 0, 0);
            for c in &mut self.ch[..n as usize] {
                match k {
                    d::M_USE_GENERIC_RDC => c.model_generic = (c.model_generic & !rdc_bit) | (flag(v) * rdc_bit),
                    d::M_USE_GENERIC_RES1 => c.model_generic = (c.model_generic & !res_bit) | (flag(v) * res_bit),
                    d::M_RDC => c.explicit[mi][0] = v,
                    d::M_BL => c.explicit[mi][1] = v,
                    d::M_MMS_G => c.explicit[mi][2] = v * 1e-3,
                    d::M_RMS => c.explicit[mi][3] = v,
                    d::M_KMS => c.explicit[mi][4] = v,
                    _ => {}
                }
                c.resolve_models();
            }
            return true;
        }
        for c in 0..n {
            if !(d::channel(c, 0)..d::channel(c, 33)).contains(&id) {
                continue;
            }
            let k = id - d::channel(c, 0);
            let x = &mut self.ch[c as usize];
            match k {
                d::CH_VOLTS_FS => x.volts_full_scale = v,
                d::CH_PARASITIC => x.parasitic_resistance = v,
                d::CH_DISPLACEMENT_LIMIT => x.displacement_limit = v,
                d::CH_TEMP_HARD => x.temperature_hard_limit = v,
                d::CH_TEMP_SOFT => x.temperature_soft_limit = v,
                d::CH_MASK_BAND_MIN => x.mask_band_hz[0] = v,
                d::CH_MASK_BAND_MAX => x.mask_band_hz[1] = v,
                d::CH_MASK_ATTACK => x.mask_attack_s = v,
                d::CH_MASK_RELEASE => x.mask_release_s = v,
                k if (d::CH_VELOCITY_LIMIT0..d::CH_VELOCITY_LIMIT0 + 3).contains(&k) => {
                    x.velocity_limit[(k - d::CH_VELOCITY_LIMIT0) as usize] = v
                }
                k if (d::CH_MASK_ENERGY0..d::CH_MASK_ENERGY0 + 4).contains(&k) => {
                    x.mask_energy_db[(k - d::CH_MASK_ENERGY0) as usize] = v
                }
                d::CH_AMPLITUDE_LIMIT => x.amplitude_limit = v,
                k if (d::CH_BAND_CENTER0..d::CH_BAND_CENTER0 + 5).contains(&k) => {
                    x.band_center_hz[(k - d::CH_BAND_CENTER0) as usize] = v
                }
                d::CH_MAX_VELOCITY_ATTEN => x.max_velocity_attenuation_db = v,
                d::CH_VELOCITY_IGNORE_LOW => x.velocity_ignore_low_band = flag(v),
                d::CH_VELOCITY_IGNORE_HIGH => x.velocity_ignore_high_band = flag(v),
                d::CH_MAX_TEMP_ATTEN => x.max_temperature_attenuation_db = v,
                d::CH_TEMP_HEADROOM => x.temperature_headroom = v,
                d::CH_MASK_ATTACK_FAST_MS => x.mask_attack_fast_s = v * 1e-3,
                d::CH_MASK_ATTACK_RANGE => x.mask_attack_range_db = v,
                d::CH_BANDWIDTH if v >= 0.0 && libm::floorf(v) == v && v <= 2.0 => x.bandwidth = v as u32,
                _ => return false,
            }
            return true;
        }
        false
    }
}

/// The exponential one-pole coefficient of spec 6.1 for an update every
/// `delta_s` seconds: `exp(-delta/tau)`.
pub fn pole(delta_s: f64, tau_s: f64) -> f64 {
    libm::exp(-delta_s / tau_s)
}

/// The 99 %-settling step of a follower updated every `delta_s` seconds
/// toward its target over `time_s` (rig law 8): `1 - exp(ln(0.01) delta /
/// time)`.
pub fn settle_step(delta_s: f64, time_s: f64) -> f64 {
    1.0 - libm::exp(libm::log(0.01) * delta_s / time_s)
}

fn db_to_lin(db: f64) -> f64 {
    libm::pow(10.0, -db / 20.0)
}

/// The five centred FIR bands of the complementary filterbank (rig law 10)
/// for a filterbank stage of `stage` frames: `2*stage - 1` taps, the centre
/// at `stage - 1`, designed on a grid of `2*stage` points; generated in f64
/// and rounded once to f32. `h[k][j]` multiplies `x[n - j]`; the bands sum
/// to a delay of `stage - 1` in f64.
pub fn filterbank(rate: f64, centers: &[f32; BANDS], bandwidth: u32, stage: usize) -> [Vec<f32>; BANDS] {
    // Bandwidth 1 is bit-identical to 0 on 25G83 (rig law 10). Bandwidth 2
    // only switches the velocity attenuation off (handled by the unit).
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

/// Direct-form-I biquad in f64 (the motion states are small and stiff).
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

/// The current model of spec 15.2: a Float32 biquad, every output rounded
/// to f32, `i[n] = f32(b0 u[n] + b1 u[n-1] + b2 u[n-2] - a1 i[n-1] - a2
/// i[n-2])`.
#[derive(Clone, Copy, Default)]
struct Current {
    b: [f32; 3],
    a: [f32; 2],
    u: [f32; 2],
    i: [f32; 2],
}

/// `1 + c1 z^-1 + c2 z^-2` whose roots are `exp(p T)` for the roots `p` of
/// `m s^2 + r s + k` (all positive): the exponentially mapped pair.
fn matched_pair(m: f64, r: f64, k: f64, t: f64) -> [f64; 2] {
    let sigma = r / (2.0 * m);
    let w2 = k / m - sigma * sigma;
    if w2 >= 0.0 {
        let e = libm::exp(-sigma * t);
        [-2.0 * e * libm::cos(libm::sqrt(w2) * t), e * e]
    } else {
        // two real roots
        let d = libm::sqrt(-w2);
        let (z1, z2) = (libm::exp((-sigma + d) * t), libm::exp((-sigma - d) * t));
        [-(z1 + z2), z1 * z2]
    }
}

impl Current {
    /// `I/U = (Mms s^2 + Rms s + Kms) / (R Mms s^2 + (R Rms + Bl^2) s + R Kms)`
    /// with matched zeros and poles and `1/R` at DC.
    fn new(rdc: f64, bl: f64, m: f64, rms: f64, k: f64, rate: f64) -> Self {
        let t = 1.0 / rate;
        let n = matched_pair(m, rms, k, t);
        let d = matched_pair(m, rms + bl * bl / rdc, k, t);
        let g = (1.0 + d[0] + d[1]) / ((1.0 + n[0] + n[1]) * rdc);
        Current {
            b: [g as f32, (g * n[0]) as f32, (g * n[1]) as f32],
            a: [d[0] as f32, d[1] as f32],
            u: [0.0; 2],
            i: [0.0; 2],
        }
    }
    #[inline]
    fn step(&mut self, u: f32) -> f32 {
        let b = self.b.map(|v| v as f64);
        let a = self.a.map(|v| v as f64);
        let i = (b[0] * u as f64 + b[1] * self.u[0] as f64 + b[2] * self.u[1] as f64
            - a[0] * self.i[0] as f64
            - a[1] * self.i[1] as f64) as f32;
        self.u = [u, self.u[0]];
        self.i = [i, self.i[0]];
        i
    }
    fn clear(&mut self) {
        self.u = [0.0; 2];
        self.i = [0.0; 2];
    }
    fn is_finite(&self) -> bool {
        self.i[0].is_finite() && self.i[1].is_finite()
    }
}

/// A fourth-order Butterworth edge (two bilinear, pre-warped biquads), high
/// or low pass: the masker's 8 kHz and 12 kHz edges.
#[derive(Clone, Copy, Default)]
struct Edge4 {
    s: [Biquad; 2],
}

impl Edge4 {
    fn new(f: f64, rate: f64, high: bool) -> Self {
        // RBJ cookbook sections with the fourth-order Butterworth Qs
        let w = 2.0 * core::f64::consts::PI * f / rate;
        let (sn, cs) = (libm::sin(w), libm::cos(w));
        let sec = |q: f64| {
            let alpha = sn / (2.0 * q);
            let a0 = 1.0 + alpha;
            let (b0, b1) = if high { ((1.0 + cs) / 2.0, -(1.0 + cs)) } else { ((1.0 - cs) / 2.0, 1.0 - cs) };
            Biquad { b: [b0 / a0, b1 / a0, b0 / a0], a: [-2.0 * cs / a0, (1.0 - alpha) / a0], x: [0.0; 2], y: [0.0; 2] }
        };
        Edge4 { s: [sec(0.541_196_100_146_197), sec(1.306_562_964_876_376_6)] }
    }
    #[inline]
    fn step(&mut self, x: f64) -> f64 {
        let y = self.s[0].step(x);
        self.s[1].step(y)
    }
    fn clear(&mut self) {
        self.s.iter_mut().for_each(Biquad::clear);
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
    #[inline]
    fn window(&self) -> &[f32] {
        &self.buf[self.pos..self.pos + self.len]
    }
    fn clear(&mut self) {
        self.buf.iter_mut().for_each(|v| *v = 0.0);
        self.pos = 0;
    }
}

/// One gain follower of the fast or slow law: a linear ramp toward 0.99 x
/// a lower request, else a follower toward the request.
#[derive(Clone, Copy)]
struct Follower {
    g: f32,
    step: f32,
    left: u32,
    target: f32,
}

impl Follower {
    const UNITY: Follower = Follower { g: 1.0, step: 0.0, left: 0, target: 1.0 };
    /// A new request at a frame boundary: below the gain, ramp to 0.99 x
    /// the request over `attack` samples.
    #[inline]
    fn request(&mut self, req: f32, attack: u32) {
        if req < self.g {
            let to = (CUT * req as f64) as f32;
            self.left = attack.max(1);
            self.step = (to - self.g) / self.left as f32;
            self.target = req;
        } else {
            self.target = req;
        }
    }
    /// One sample: the ramp, else the per-sample release `r` toward the
    /// request (fast gains).
    #[inline]
    fn sample(&mut self, r: f32) -> f32 {
        if self.left > 0 {
            self.g += self.step;
            self.left -= 1;
        } else if self.g < self.target {
            self.g += r * (self.target - self.g);
            if self.target - self.g <= f32::EPSILON * self.target {
                self.g = self.target;
            }
        }
        self.g
    }
    /// One sample of a slow gain: the ramp only (its release is a step at
    /// the frame boundary).
    #[inline]
    fn sample_ramp(&mut self) -> f32 {
        if self.left > 0 {
            self.g += self.step;
            self.left -= 1;
        }
        self.g
    }
    fn finite(&self) -> bool {
        self.g.is_finite() && self.step.is_finite() && self.target.is_finite()
    }
}

/// Substeps in the decision window: the frame's eight plus the look-ahead
/// region's (six at 48 kHz). Bounded by one controller frame of look-ahead.
const RING: usize = 2 * SUBSTEPS;

struct Chan {
    src: usize,
    vfs: f32,
    /// Input history: the bank's 2 FB - 1 previous samples, then the frame.
    xb: Vec<f32>,
    /// Centred bank taps, bands 0..3 (symmetric, so in dot-product order).
    bank: [Vec<f32>; BANDS - 1],
    /// Bank-stage bands: the OD frames before the frame, then the frame.
    ab: [Vec<f32>; BANDS],
    /// Bank-stage full band (the delayed input), same layout.
    xc: Vec<f32>,
    // Motion models on the bank stage.
    disp: Biquad,
    vel: Biquad,
    vel_band: [Biquad; BANDS - 1],
    /// Per-substep peaks over the window (ring of RING slots).
    pk_x: [f32; RING],
    pk_d: [f32; RING],
    pk_v: [f32; RING],
    pk_vb: [[f32; RING]; BANDS],
    // Masking path, on the program at the input.
    pressure: Vec<f32>,
    p_hist: History,
    mask_hp: Edge4,
    mask_lp: Edge4,
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
    // Fast gains.
    band: [Follower; BANDS],
    peak: Follower,
    /// The fast gains over the frame's output region, per sample.
    eg: [Vec<f32>; BANDS],
    // Power path (fast-gained bank stage): the committed state at the
    // start of the frame's output region.
    cur_in: Current,
    pbar_in: [f64; 4],
    /// Per-substep `u i` of the fast-gained bank stage, output region.
    p_sub: [f64; SUBSTEPS],
    // Temperature axis: own series predictor and its gain.
    tp_vc: f32,
    tp_mg: f32,
    gt: f64,
    // Slow gain.
    slow: Follower,
    // Monitor on the output.
    cur_out: Current,
    th_vc: f32,
    th_mg: f32,
    pbar: [f64; 4],
    pbar1_frame: f64,
    pbar1s_frame: f64,
    // Per-channel constants.
    load_scale: f64,
    rth_vc: f64,
    rth_mag: f64,
    b_vc: f32,
    a_vc: f32,
    b_mg: f32,
    a_mg: f32,
    tau_vc: f64,
    tau_mag: f64,
    ambient: f64,
    amp_limit: f64,
    disp_limit: f64,
    vel_floor: f64,
    ignore: [bool; BANDS],
    vel_off: bool,
    t_soft: f64,
    t_hard: f64,
    t_headroom: f64,
    /// The logged candidates: peak, displacement, velocity, linked power,
    /// temperature.
    cand: [f64; 5],
}

pub struct Unit {
    params: Params,
    run: Option<Run>,
    companion: Option<companion::Companion>,
    estimator: Option<crate::lsid::Estimator>,
    handoff: Option<crate::lsid::Handoff>,
    model_frames: u64,
}

struct Run {
    n: usize,
    /// Controller frame, substep, bank stage, output offset, frames.
    ctl: usize,
    sub: usize,
    fb: usize,
    od: usize,
    latency: usize,
    /// A model state went non-finite: the fail-safe is latched until reset.
    fault: bool,
    /// Frames into the current controller frame (input FIFO fill).
    pos: usize,
    frame: u64,
    last_frame: u64,
    frames_done: u64,
    /// Ring slot of the next substep.
    ring: usize,
    ring_len: usize,
    axis: [bool; 5],
    link: bool,
    unlink_temp: bool,
    p_pole: [f64; 4],
    budget: [f64; 4],
    idle: f64,
    ratio: f64,
    attack: u32,
    r_fast: f32,
    r_low: f32,
    slow_step: f64,
    temp_release: f64,
    linked_power: f64,
    /// `10^(-MaxPowerAttenuation/20)`.
    power_floor: f64,
    /// Per channel: the frame's input and output (FIFO).
    xin: Vec<Vec<f32>>,
    yout: Vec<Vec<f32>>,
    ch: Vec<Chan>,
}

impl Unit {
    /// Allocation happens here, never in `process`. Unusable parameters
    /// give a unit that is the fail-safe; `init_error` says why.
    pub fn new(params: Params) -> Self {
        let run = params.check_protects().and_then(|()| Run::new(&params)).ok();
        Unit { params, run, companion: None, estimator: None, handoff: None, model_frames: 0 }
    }
    pub fn from_blob(bytes: &[u8]) -> Result<Self, InitError> {
        let (params, companion) = companion::decode_blob(bytes)?;
        let mut unit = Self::new(params);
        if let Some(c) = &companion {
            if let Some(topology) = c.topology {
                let preset =
                    crate::lsid::Preset::decode(&c.lsid_records).map_err(|_| InitError::Invalid("LSID preset"))?;
                let estimator = crate::lsid::Estimator::new(preset, topology, params.sample_rate)
                    .map_err(|_| InitError::Invalid("LSID seed"))?;
                unit.handoff = Some(crate::lsid::Handoff::new(estimator.model().clone()));
                unit.estimator = Some(estimator);
            }
        }
        unit.companion = companion;
        Ok(unit)
    }
    pub fn companion(&self) -> Option<&companion::Companion> {
        self.companion.as_ref()
    }
    pub fn estimator(&self) -> Option<&crate::lsid::Estimator> {
        self.estimator.as_ref()
    }
    pub fn model_generation(&self) -> Option<&crate::lsid::ModelGeneration> {
        self.handoff.as_ref().map(|h| h.current())
    }
    pub fn stage_model(&mut self, model: crate::lsid::ModelGeneration) -> Result<(), crate::lsid::Error> {
        self.handoff.as_mut().ok_or(crate::lsid::Error::Unsupported)?.stage(model)
    }
    pub fn set_runtime_budgets(&mut self, budgets: [f32; 4]) {
        self.params.runtime_budget_present = 1;
        self.params.runtime_budget_w = budgets;
        self.run = self.params.check_protects().and_then(|()| Run::new(&self.params)).ok();
    }
    /// Why [`Unit::new`] gives the fail-safe for `params`: unprotected
    /// (mitigation off, or no thermal axis) or unusable.
    pub fn init_error(params: &Params) -> Option<InitError> {
        params.check_protects().and_then(|()| Run::new(params)).err()
    }
    pub fn params(&self) -> &Params {
        &self.params
    }
    pub fn is_failsafe(&self) -> bool {
        self.run.is_none() || self.companion.as_ref().is_some_and(|c| c.gates != 0)
    }
}

/// Bilinear models `X/U` and `V/U` of one block, `R = Rdc` (rig law 1).
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
        let ctl = controller_frames(p.sample_rate);
        let sub = ctl / SUBSTEPS;
        let fb = p.filterbank_frames();
        let od = p.output_offset_frames();
        let ring_len = (ctl + od) / sub;
        let sub_dt = sub as f64 / rate;
        let ctl_dt = ctl as f64 / rate;
        let mut ch = Vec::with_capacity(n);
        for c in &p.ch[..n] {
            let h = filterbank(rate, &c.band_center_hz, c.bandwidth, fb);
            let bank: [Vec<f32>; BANDS - 1] = core::array::from_fn(|k| h[k].clone());
            let taps = c.pressure_taps as usize;
            let pressure: Vec<f32> = (0..taps).map(|j| c.pressure_fir[taps - 1 - j]).collect();
            let vel = motion(c.vel_rdc, c.vel_bl, c.vel_mms_kg, c.vel_rms, c.vel_kms, rate, true);
            let cur = Current::new(c.rdc as f64, c.bl as f64, c.mms_kg as f64, c.rms as f64, c.kms as f64, rate);
            let bvc = pole(sub_dt, c.tau_vc as f64);
            let bmg = pole(sub_dt, c.tau_mag as f64);
            let ignore: [bool; BANDS] = core::array::from_fn(|k| {
                (k == 0 && c.velocity_ignore_low_band != 0) || (k == BANDS - 1 && c.velocity_ignore_high_band != 0)
            });
            ch.push(Chan {
                src: c.source as usize,
                vfs: c.volts_full_scale,
                xb: vec![0.0; 2 * fb - 1 + ctl],
                bank,
                ab: core::array::from_fn(|_| vec![0.0; od + ctl]),
                xc: vec![0.0; od + ctl],
                disp: motion(c.disp_rdc, c.disp_bl, c.disp_mms_kg, c.disp_rms, c.disp_kms, rate, false),
                vel,
                vel_band: [vel; BANDS - 1],
                pk_x: [0.0; RING],
                pk_d: [0.0; RING],
                pk_v: [0.0; RING],
                pk_vb: [[0.0; RING]; BANDS],
                pressure,
                p_hist: History::new(taps.max(1)),
                mask_hp: Edge4::new(c.mask_band_hz[0] as f64, rate, true),
                mask_lp: Edge4::new(c.mask_band_hz[1] as f64, rate, false),
                mask_acc: 0.0,
                mask_db: 0.0,
                mask_init: false,
                vel_lim: c.velocity_limit.map(|v| v as f64),
                mask_knots: c.mask_energy_db.map(|v| v as f64),
                mask_a: 1.0 - pole(ctl_dt, c.mask_attack_s as f64),
                mask_af: 1.0 - pole(ctl_dt, c.mask_attack_fast_s as f64),
                mask_r: 1.0 - pole(ctl_dt, c.mask_release_s as f64),
                mask_range: c.mask_attack_range_db as f64,
                vel_limit_now: c.velocity_limit[0] as f64,
                band: [Follower::UNITY; BANDS],
                peak: Follower::UNITY,
                eg: core::array::from_fn(|_| vec![1.0; ctl]),
                cur_in: cur,
                pbar_in: [0.0; 4],
                p_sub: [0.0; SUBSTEPS],
                tp_vc: 0.0,
                tp_mg: 0.0,
                gt: 1.0,
                slow: Follower::UNITY,
                cur_out: cur,
                th_vc: 0.0,
                th_mg: 0.0,
                pbar: [0.0; 4],
                pbar1_frame: 0.0,
                pbar1s_frame: 0.0,
                load_scale: (c.rdc as f64 + c.parasitic_resistance as f64) / c.rdc as f64,
                rth_vc: c.rth_vc as f64,
                rth_mag: c.rth_mag as f64,
                b_vc: bvc as f32,
                a_vc: (1.0 - bvc) as f32,
                b_mg: bmg as f32,
                a_mg: (1.0 - bmg) as f32,
                tau_vc: c.tau_vc as f64,
                tau_mag: c.tau_mag as f64,
                ambient: c.ambient as f64,
                amp_limit: c.amplitude_limit as f64 - c.pilot_reserve as f64,
                disp_limit: c.displacement_limit as f64,
                vel_floor: db_to_lin(c.max_velocity_attenuation_db as f64),
                ignore,
                // rig law 10: Bandwidth 2 gives no velocity attenuation
                vel_off: c.bandwidth == 2,
                t_soft: c.temperature_soft_limit as f64,
                t_hard: c.temperature_hard_limit as f64,
                t_headroom: c.temperature_headroom as f64,
                cand: [1.0; 5],
            });
        }
        let mut p_pole = [0.0; 4];
        let mut budget = [0.0; 4];
        for j in 0..4 {
            p_pole[j] = pole(sub_dt, p.power_time_scale_s[j] as f64);
            let b = if p.runtime_budget_present != 0 { p.runtime_budget_w[j] } else { p.power_hard_limit_w[j] };
            budget[j] = b as f64 * p.power_budgets_scale as f64;
        }
        let sample = |ms: f32| settle_step(1.0 / rate, ms as f64 * 1e-3) as f32;
        Ok(Run {
            n,
            ctl,
            sub,
            fb,
            od,
            latency: p.unit_latency_frames(),
            fault: false,
            pos: 0,
            frame: 0,
            last_frame: 0,
            frames_done: 0,
            ring: 0,
            ring_len,
            // EnableMitigation = 0: every axis off, the models and meters run
            axis: [
                p.control_sample_peak,
                p.control_displacement,
                p.control_velocity,
                p.control_power,
                p.control_temperature,
            ]
            .map(|f| f != 0 && p.enable_mitigation != 0),
            link: p.link_slow_gains != 0,
            unlink_temp: p.unlink_temperature_gains != 0,
            p_pole,
            budget,
            idle: p.low_rail_idle_power_w as f64,
            ratio: p.low_rail_power_ratio as f64,
            attack: libm::round(p.attack_time_ms as f64 * 1e-3 * rate) as u32,
            r_fast: sample(p.fast_release_time_ms),
            r_low: sample(p.low_band_release_time_ms),
            slow_step: settle_step(ctl_dt, p.slow_release_time_ms as f64 * 1e-3),
            temp_release: 1.0 - pole(ctl_dt, p.slow_release_time_ms as f64 * 1e-3),
            linked_power: 1.0,
            power_floor: db_to_lin(p.max_power_attenuation_db as f64),
            xin: vec![vec![0.0; ctl]; n],
            yout: vec![vec![0.0; ctl]; n],
            ch,
        })
    }

    fn reset(&mut self) {
        let ctl = self.ctl;
        for c in self.ch.iter_mut() {
            c.xb.iter_mut().for_each(|v| *v = 0.0);
            c.ab.iter_mut().for_each(|b| b.iter_mut().for_each(|v| *v = 0.0));
            c.xc.iter_mut().for_each(|v| *v = 0.0);
            c.disp.clear();
            c.vel.clear();
            c.vel_band.iter_mut().for_each(Biquad::clear);
            c.pk_x = [0.0; RING];
            c.pk_d = [0.0; RING];
            c.pk_v = [0.0; RING];
            c.pk_vb = [[0.0; RING]; BANDS];
            c.p_hist.clear();
            c.mask_hp.clear();
            c.mask_lp.clear();
            c.mask_acc = 0.0;
            c.mask_db = 0.0;
            c.mask_init = false;
            c.vel_limit_now = c.vel_lim[0];
            c.band = [Follower::UNITY; BANDS];
            c.peak = Follower::UNITY;
            c.eg.iter_mut().for_each(|e| e.iter_mut().for_each(|v| *v = 1.0));
            c.cur_in.clear();
            c.pbar_in = [0.0; 4];
            c.p_sub = [0.0; SUBSTEPS];
            c.tp_vc = 0.0;
            c.tp_mg = 0.0;
            c.gt = 1.0;
            c.slow = Follower::UNITY;
            c.cur_out.clear();
            c.th_vc = 0.0;
            c.th_mg = 0.0;
            c.pbar = [0.0; 4];
            c.pbar1_frame = 0.0;
            c.pbar1s_frame = 0.0;
            c.cand = [1.0; 5];
        }
        for b in self.xin.iter_mut().chain(self.yout.iter_mut()) {
            b[..ctl].iter_mut().for_each(|v| *v = 0.0);
        }
        self.pos = 0;
        self.frame = 0;
        self.last_frame = 0;
        self.frames_done = 0;
        self.ring = 0;
        self.linked_power = 1.0;
        self.fault = false;
    }

    /// Analysis of one channel's frame: the bank stage, the motion models,
    /// the per-substep peaks and the mask energy.
    fn analyse(&mut self, ci: usize) {
        let (ctl, sub, fb, od, ring_len) = (self.ctl, self.sub, self.fb, self.od, self.ring_len);
        let mut slot = self.ring;
        let c = &mut self.ch[ci];
        let xin = &self.xin[ci];
        let hist = 2 * fb - 1;
        c.xb[hist..hist + ctl].copy_from_slice(&xin[..ctl]);
        let vfs = c.vfs as f64;
        let (mut px, mut pd, mut pv) = (0.0f32, 0.0f32, 0.0f32);
        let mut pvb = [0.0f32; BANDS];
        for i in 0..ctl {
            let w = &c.xb[i..i + hist];
            let xc = w[fb - 1];
            let mut rest = xc;
            let mut a = [0.0f32; BANDS];
            for k in 0..BANDS - 1 {
                a[k] = fir_reversed(&c.bank[k], w);
                rest -= a[k];
            }
            a[BANDS - 1] = rest;
            for k in 0..BANDS {
                c.ab[k][od + i] = a[k];
            }
            c.xc[od + i] = xc;
            let u = vfs * xc as f64;
            let d = c.disp.step(u);
            let v = c.vel.step(u);
            let mut vrest = v;
            for k in 0..BANDS - 1 {
                let vk = c.vel_band[k].step(vfs * a[k] as f64);
                vrest -= vk;
                pvb[k] = pvb[k].max(libm::fabs(vk) as f32);
            }
            pvb[BANDS - 1] = pvb[BANDS - 1].max(libm::fabs(vrest) as f32);
            px = px.max(xc.abs());
            pd = pd.max(libm::fabs(d) as f32);
            pv = pv.max(libm::fabs(v) as f32);
            // the masker sees the program at the input (spec 15.3)
            if !c.pressure.is_empty() {
                c.p_hist.push(xin[i]);
                let p = fir_reversed(&c.pressure, c.p_hist.window()) as f64;
                let m = c.mask_lp.step(c.mask_hp.step(p));
                c.mask_acc += m * m;
            }
            if (i + 1) % sub == 0 {
                c.pk_x[slot] = px;
                c.pk_d[slot] = pd;
                c.pk_v[slot] = pv;
                for k in 0..BANDS {
                    c.pk_vb[k][slot] = pvb[k];
                }
                px = 0.0;
                pd = 0.0;
                pv = 0.0;
                pvb = [0.0; BANDS];
                slot = (slot + 1) % ring_len;
            }
        }
        // keep the bank's history
        c.xb.copy_within(ctl..ctl + hist, 0);
    }

    /// One controller frame: every channel's input in `xin`, its output to
    /// `yout`.
    fn process_frame(&mut self) {
        // a gain state that went non-finite since the last frame is caught
        // before this frame's min/max could turn it into a number
        if !self.ch.iter().all(|c| c.band.iter().all(Follower::finite) && c.peak.finite() && c.slow.finite()) {
            self.fault = true;
            return;
        }
        for ci in 0..self.n {
            self.analyse(ci);
        }
        self.ring = (self.ring + SUBSTEPS) % self.ring_len;
        let (ctl, sub, od, attack) = (self.ctl, self.sub, self.od, self.attack);
        let (r_fast, r_low) = (self.r_fast, self.r_low);
        let ring_len = self.ring_len;
        let window_max = |r: &[f32; RING]| r[..ring_len].iter().copied().fold(0.0f32, f32::max);
        let ratio = |limit: f64, q: f32| (limit / (q as f64).max(EPS)).min(1.0);
        // ---- fast axes: candidates, band requests, the frame's trajectories
        let linked_ems = self
            .ch
            .iter()
            .filter(|c| !c.pressure.is_empty())
            .map(|c| (c.mask_acc / ctl as f64) as f32 as f64)
            .fold(f64::INFINITY, f64::min);
        for c in self.ch.iter_mut() {
            c.mask_acc = 0.0;
        }
        for c in self.ch.iter_mut() {
            // velocity limit from the smoothed mask level (spec 15.3), one
            // level for the pair: the 25G83 frame logs give both channels the
            // same active velocity limit; the quieter channel's masker sets it
            if !c.pressure.is_empty() {
                let e = 10.0 * libm::log10(linked_ems.max(EPS)) + MASK_DB_OFFSET;
                // the smoothed level starts at 0 dB (25G83: the first frame's
                // limit is one attack step up from there)
                let a = if e > c.mask_db {
                    if e - c.mask_db >= c.mask_range {
                        c.mask_af
                    } else {
                        c.mask_a
                    }
                } else {
                    c.mask_r
                };
                c.mask_db += a * (e - c.mask_db);
                c.mask_init = true;
                c.vel_limit_now = mask_curve(c.mask_db, &c.mask_knots, &c.vel_lim);
            }
            let g_peak = if self.axis[0] { ratio(c.amp_limit, window_max(&c.pk_x)) } else { 1.0 };
            let g_disp = if self.axis[1] { ratio(c.disp_limit, window_max(&c.pk_d)) } else { 1.0 };
            let vel_on = self.axis[2] && !c.vel_off;
            let g_vel = if self.axis[2] { ratio(c.vel_limit_now, window_max(&c.pk_v)).max(c.vel_floor) } else { 1.0 };
            // spec 15.4: the five bands' peaks over the frame itself (its
            // eight newest substeps), not the decision window
            let first = (self.ring + ring_len - SUBSTEPS) % ring_len;
            let pk: [f64; BANDS] = core::array::from_fn(|k| {
                (0..SUBSTEPS).map(|j| c.pk_vb[k][(first + j) % ring_len]).fold(0.0f32, f32::max) as f64
            });
            let active: [bool; BANDS] = core::array::from_fn(|k| vel_on && !c.ignore[k] && pk[k] > 0.0);
            // the velocity limit holds on the program the slow gain already
            // shapes (25G83 thermal ramps: the output velocity stays at the
            // limit while the temperature gain falls): water filling on the
            // slow-gained peaks, then each band's gain made absolute
            let sg = (c.slow.g as f64).clamp(EPS, 1.0);
            let pks: [f64; BANDS] = core::array::from_fn(|k| pk[k] * sg);
            let rel = waterfill(&pks, &active, c.vel_limit_now, c.vel_floor);
            let gv: [f64; BANDS] = core::array::from_fn(|k| if rel[k] < 1.0 { (rel[k] * sg).min(1.0) } else { 1.0 });
            for k in 0..BANDS {
                let req = g_disp.min(gv[k]) as f32;
                c.band[k].request(req, attack);
            }
            c.peak.request(g_peak as f32, attack);
            c.cand[0] = g_peak;
            c.cand[1] = g_disp;
            c.cand[2] = g_vel;
            for i in 0..ctl {
                let pg = c.peak.sample(r_fast);
                for k in 0..BANDS {
                    let r = if k == 0 { r_low } else { r_fast };
                    c.eg[k][i] = c.band[k].sample(r).min(pg);
                }
            }
        }
        // ---- power path on the fast-gained bank stage, from the committed
        // state at the start of the output region through the look-ahead
        let mut phat = [[0.0f64; 4]; MAX_CHANNELS];
        for (ci, c) in self.ch.iter_mut().enumerate() {
            let mut cur = c.cur_in;
            let mut pb = c.pbar_in;
            let mut acc = 0.0f64;
            let vfs = c.vfs;
            for idx in 0..ctl + od {
                // input-referred (rig law 4): the power and temperature
                // candidates see the bank stage before any gain
                let u = vfs * c.xc[idx];
                let i = cur.step(u);
                acc += u as f64 * i as f64;
                if (idx + 1) % sub == 0 {
                    let p = acc / sub as f64;
                    acc = 0.0;
                    let s = idx / sub;
                    if s < SUBSTEPS {
                        c.p_sub[s] = p;
                    }
                    let pamp = self.idle + self.ratio * (p * c.load_scale).max(0.0);
                    for j in 0..4 {
                        pb[j] = self.p_pole[j] * pb[j] + (1.0 - self.p_pole[j]) * pamp;
                        phat[ci][j] = if idx + 1 == sub { pb[j] } else { phat[ci][j].max(pb[j]) };
                    }
                    if idx + 1 == ctl {
                        c.cur_in = cur;
                        c.pbar_in = pb;
                    }
                }
            }
        }
        // power (rig law 7): one gain for all systems
        let g_pow = if self.axis[3] {
            let mut g = 1.0f64;
            let nidle = self.n as f64 * self.idle;
            for j in 0..4 {
                let demand: f64 = phat[..self.n].iter().map(|p| p[j] - self.idle).sum();
                if demand > 0.0 {
                    g = g.min(libm::sqrt((self.budget[j] - nidle).max(0.0) / demand));
                }
            }
            g.min(1.0)
        } else {
            1.0
        };
        self.linked_power = g_pow;
        // ---- temperature axis (spec 15.5): own predictor and candidate
        for c in self.ch.iter_mut() {
            let pin = c.p_sub.iter().sum::<f64>() / SUBSTEPS as f64;
            let g2 = c.gt * c.gt;
            for &p in &c.p_sub {
                let p = p.max(0.0) * g2;
                c.tp_vc = c.b_vc * c.tp_vc + (c.a_vc as f64 * c.rth_vc * p) as f32;
                c.tp_mg = c.b_mg * c.tp_mg + (c.a_mg as f64 * c.rth_mag * p) as f32;
            }
            let t_pred = c.ambient + c.tp_mg as f64 + c.tp_vc as f64;
            let release = libm::sqrt((g2 + self.temp_release).min(1.0));
            c.gt = if self.axis[4] && t_pred > c.t_soft {
                let q2 = ((c.t_hard - c.ambient - c.tp_mg as f64) / (c.rth_vc * pin.max(0.0) + 1.0)).clamp(1e-4, 1.0);
                let q = libm::sqrt(q2);
                if q < c.gt {
                    q
                } else {
                    release
                }
            } else {
                release
            };
            if !self.axis[4] {
                c.gt = 1.0;
            }
            c.cand[3] = g_pow;
            c.cand[4] = c.gt;
        }
        let linked_temp = self.link && !self.unlink_temp;
        let gt_all = self.ch.iter().map(|c| c.gt).fold(1.0f64, f64::min);
        // ---- slow gain: the 0.99 law, then the hard ceiling on the monitor
        let n = self.n;
        let mut req = [1.0f64; MAX_CHANNELS];
        for (ci, c) in self.ch.iter().enumerate() {
            let gt = if linked_temp { gt_all } else { c.gt };
            // the applied power gain is capped at MaxPowerAttenuation (spec
            // 6.1; 25G83 rig: the logged candidate is not, the audio is)
            req[ci] = g_pow.max(self.power_floor).min(gt);
        }
        if self.link && n > 1 {
            let m = req[..n].iter().copied().fold(1.0f64, f64::min);
            req[..n].iter_mut().for_each(|r| *r = m);
        }
        let mut commit = [0.0f64; MAX_CHANNELS];
        for (ci, c) in self.ch.iter().enumerate() {
            let a = c.slow.g as f64;
            commit[ci] = if req[ci] < a { CUT * req[ci] } else { a + self.slow_step * (req[ci] - a) };
        }
        if self.axis[4] {
            for _ in 0..CEILING_PASSES {
                let mut cut = f64::INFINITY;
                for (ci, c) in self.ch.iter().enumerate() {
                    let ceiling = c.t_hard + c.t_headroom.max(0.0);
                    let pin = c.p_sub.iter().sum::<f64>() / SUBSTEPS as f64;
                    if pin <= 0.0 {
                        continue;
                    }
                    // the monitor after this frame at the proposed gain
                    let g2 = commit[ci] * commit[ci];
                    let (mut vc, mut mg) = (c.th_vc, c.th_mg);
                    for &p in &c.p_sub {
                        let p = p.max(0.0) * g2;
                        vc = c.b_vc * vc + (c.a_vc as f64 * c.rth_vc * p) as f32;
                        mg = c.b_mg * mg + (c.a_mg as f64 * c.rth_mag * p) as f32;
                    }
                    if c.ambient + mg as f64 + vc as f64 > ceiling {
                        let room = (ceiling - c.ambient - mg as f64).max(0.0);
                        let g = CUT * libm::sqrt(room / (c.rth_vc * pin));
                        cut = cut.min(g.min(commit[ci]));
                    }
                }
                if !cut.is_finite() {
                    break;
                }
                for ci in 0..n {
                    if self.link || cut < commit[ci] {
                        commit[ci] = commit[ci].min(cut);
                    }
                }
            }
        }
        for (ci, c) in self.ch.iter_mut().enumerate() {
            let to = commit[ci] as f32;
            if to < c.slow.g {
                c.slow.left = attack.max(1);
                c.slow.step = (to - c.slow.g) / c.slow.left as f32;
            } else {
                c.slow.left = 0;
                c.slow.g = to.min(1.0);
            }
        }
        // ---- output region and the monitor
        let (idle, ratio_p) = (self.idle, self.ratio);
        for (ci, c) in self.ch.iter_mut().enumerate() {
            let y = &mut self.yout[ci];
            let mut acc = 0.0f64;
            let mut s = 0;
            let (mut p1, mut p1s) = (0.0f64, 0.0f64);
            for (i, yo) in y.iter_mut().enumerate().take(ctl) {
                let sg = c.slow.sample_ramp();
                let g0 = c.eg[0][i].min(sg);
                let mut same = true;
                let mut g = [0.0f32; BANDS];
                for k in 0..BANDS {
                    g[k] = c.eg[k][i].min(sg);
                    same &= g[k] == g0;
                }
                let v = if same {
                    g0 * c.xc[i]
                } else {
                    let mut v = 0.0f32;
                    for k in 0..BANDS {
                        v += g[k] * c.ab[k][i];
                    }
                    v
                };
                let u = c.vfs * v;
                let iv = c.cur_out.step(u);
                acc += u as f64 * iv as f64;
                *yo = v.clamp(-1.0, 1.0);
                if (i + 1) % sub == 0 {
                    let p = acc / sub as f64;
                    acc = 0.0;
                    // thermal (rig law 3): series matched one-poles in Float32
                    let pp = p.max(0.0);
                    c.th_vc = c.b_vc * c.th_vc + (c.a_vc as f64 * c.rth_vc * pp) as f32;
                    c.th_mg = c.b_mg * c.th_mg + (c.a_mg as f64 * c.rth_mag * pp) as f32;
                    let pamp = idle + ratio_p * (p * c.load_scale).max(0.0);
                    for j in 0..4 {
                        c.pbar[j] = self.p_pole[j] * c.pbar[j] + (1.0 - self.p_pole[j]) * pamp;
                    }
                    p1 = if s == 0 { c.pbar[0] } else { p1.max(c.pbar[0]) };
                    p1s = if s == 0 { c.pbar[2] } else { p1s.max(c.pbar[2]) };
                    s += 1;
                }
            }
            c.pbar1_frame = p1;
            c.pbar1s_frame = p1s;
            // shift the bank-stage buffers: keep the look-ahead region
            for b in c.ab.iter_mut() {
                b.copy_within(ctl..ctl + od, 0);
            }
            c.xc.copy_within(ctl..ctl + od, 0);
            let model = [c.th_vc, c.th_mg, c.tp_vc, c.tp_mg].iter().all(|v| v.is_finite())
                && c.mask_db.is_finite()
                && c.pbar.iter().chain(&c.pbar_in).all(|v| v.is_finite())
                && c.gt.is_finite();
            let motion = [&c.disp, &c.vel].iter().all(|b| b.y[0].is_finite())
                && c.vel_band.iter().all(|b| b.y[0].is_finite())
                && c.cur_in.is_finite()
                && c.cur_out.is_finite();
            let gains = c.cand.iter().all(|v| v.is_finite())
                && c.band.iter().all(Follower::finite)
                && c.peak.finite()
                && c.slow.finite();
            if !(model && motion && gains) {
                self.fault = true;
            }
        }
        let _ = sub;
        self.last_frame = self.frames_done;
        self.frames_done += ctl as u64;
    }

    fn process(&mut self, io: &mut Io<'_, '_>) {
        for k in 0..io.frames {
            let pos = self.pos;
            for ci in 0..self.n {
                let src = self.ch[ci].src;
                let x = io.input[src][k];
                // a non-finite sample is 0 to the audio and the models; the
                // peak axis sees it as the most restrictive
                self.xin[ci][pos] = if x.is_finite() { dsp::meter1(x) } else { dsp::METER_FS };
                io.output[src][k] = self.yout[ci][pos];
            }
            self.pos += 1;
            self.frame += 1;
            if self.pos == self.ctl {
                self.pos = 0;
                self.process_frame();
                if self.fault {
                    crate::failsafe::fill_from(io, k + 1, FAILSAFE_GAIN);
                    return;
                }
            }
        }
    }
}

/// Spec 15.4: water filling on the five band velocity frame peaks `pk`.
/// The unique level `L >= 0` with `sum(active, min(pk, L)) = vlim`; an
/// active band above it gets `L / pk`, floored at `floor`; every other band
/// 1.
pub fn waterfill(pk: &[f64; BANDS], active: &[bool; BANDS], vlim: f64, floor: f64) -> [f64; BANDS] {
    let mut g = [1.0f64; BANDS];
    let total: f64 = (0..BANDS).filter(|&k| active[k]).map(|k| pk[k]).sum();
    // a NaN total also leaves every band at unity (the fault check latches)
    if total.partial_cmp(&vlim) != Some(core::cmp::Ordering::Greater) {
        return g;
    }
    // sort the active peaks ascending and find the segment holding L
    let mut v = [0.0f64; BANDS];
    let mut m = 0;
    for k in 0..BANDS {
        if active[k] {
            v[m] = pk[k];
            m += 1;
        }
    }
    let v = &mut v[..m];
    v.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let mut below = 0.0f64;
    let mut level = 0.0f64;
    for i in 0..m {
        // L in [v[i-1], v[i]]: below + (m - i) L = vlim
        let l = (vlim - below) / (m - i) as f64;
        if l <= v[i] {
            level = l.max(0.0);
            break;
        }
        below += v[i];
    }
    for k in 0..BANDS {
        if active[k] && pk[k] > level {
            g[k] = (level / pk[k]).max(floor);
        }
    }
    g
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
        if let Some(estimator) = &mut self.estimator {
            // Io has no authenticated source identity. Never infer it from port
            // count: adaptation stays disabled, even when all twelve ports exist.
            estimator.observe("", io.sense, frames);
        }
        let cadence = controller_frames(self.params.sample_rate) as u64;
        let end = self.model_frames.saturating_add(frames as u64);
        if self.model_frames % cadence == 0 || end / cadence > self.model_frames / cadence {
            if let Some(handoff) = &mut self.handoff {
                handoff.commit();
            }
        }
        self.model_frames = end;
        if self.companion.as_ref().is_some_and(|c| c.gates != 0) {
            if let Some(run) = &mut self.run {
                if !run.fault
                    && io.input.len() == run.n
                    && io.output.len() == run.n
                    && io.input.iter().all(|b| b.len() >= frames)
                    && io.output.iter().all(|b| b.len() >= frames)
                {
                    run.process(&mut io);
                }
            }
            // Section-10 release gates are unresolved. The complete model is
            // preserved, but it cannot grant audible output or relax protection.
            crate::failsafe::fill_from(&mut io, 0, 0.0);
            return Health(Health::FAILSAFE | Health::NO_PARAMS);
        }
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
        if self.companion.as_ref().is_some_and(|c| c.gates != 0) {
            out.health = Health(Health::FAILSAFE | Health::NO_PARAMS);
            for channel in &mut out.channels {
                channel.gain = 0.0;
            }
            return;
        }
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
            let t = c.ambient + c.th_mg as f64 + c.th_vc as f64;
            s.modeled_power_w = c.pbar[2] as f32;
            s.coil_temp_c = t as f32;
            s.magnet_temp_c = (c.ambient + c.th_mg as f64) as f32;
            s.control_temp_c = t as f32;
            s.ol_temp_c = t as f32;
            s.velocity_limit_m_s = c.vel_limit_now as f32;
            s.mask_db = if c.mask_init { c.mask_db as f32 } else { f32::NAN };
            for (a, &g) in s.axis_gain.iter_mut().zip(&c.cand) {
                *a = g as f32;
            }
            s.band_gain = core::array::from_fn(|k| c.band[k].g);
            s.gain = if run.fault { FAILSAFE_GAIN } else { c.slow.g.min(c.peak.g) };
            out.channels[c.src] = s;
            amp_1s += c.pbar1s_frame;
            if c.src == 0 {
                amp_now = c.pbar1_frame;
            }
        }
        let g = &mut out.global;
        g.power_hard_limit_w = run.budget.map(|b| b as f32);
        g.power_gain = run.linked_power as f32;
        g.modeled_amp_power_w = amp_1s as f32;
        g.amp_input_power_w = amp_now as f32;
    }
    fn reset(&mut self) {
        self.model_frames = 0;
        if let Some(e) = &mut self.estimator {
            e.reset();
        }
        if let Some(h) = &mut self.handoff {
            h.discard_pending();
        }
        if let Some(r) = self.run.as_mut() {
            r.reset();
        }
    }

    fn set_parameter(&mut self, id: u32, value: f32) -> bool {
        // A change rebuilds the unit, so only before the first frame (the
        // host's post-initialisation setup, as on macOS). This is the
        // reference AU's parameter path, which only the differential rig
        // drives: it may switch axes or mitigation off as the reference
        // allows. A unit built from a blob (`Unit::new`) always protects.
        if self.run.as_ref().is_some_and(|r| r.frame != 0) {
            return false;
        }
        let mut p = self.params;
        if !p.set(id, value) {
            return false;
        }
        self.params = p;
        self.run = Run::new(&self.params).ok();
        true
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
            n[1] = (c.th_mg as f64).max(0.0);
            n[2..6].copy_from_slice(&c.pbar.map(|v| v.max(0.0)));
            n[6] = (c.tp_vc as f64).max(0.0);
            n[7] = (c.tp_mg as f64).max(0.0);
            out.gain[i] = (c.slow.g as f64).min(c.gt).clamp(f64::MIN_POSITIVE, 1.0);
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
        let frames = t * rate as f64 / run.ctl as f64;
        let substeps = t * rate as f64 / run.sub as f64;
        for (i, c) in run.ch.iter_mut().enumerate() {
            let s = &state.node[i];
            // the series nodes cool independently, each with its own constant
            let (ev, em) = (libm::exp(-t / c.tau_vc), libm::exp(-t / c.tau_mag));
            c.th_vc = (s[0] * ev) as f32;
            c.th_mg = (s[1] * em) as f32;
            c.tp_vc = (s[6] * ev) as f32;
            c.tp_mg = (s[7] * em) as f32;
            for j in 0..4 {
                let v = run.idle + (s[2 + j] - run.idle) * libm::pow(run.p_pole[j], substeps);
                c.pbar[j] = v;
                c.pbar_in[j] = v;
            }
            // the slow gain released over the gap by its per-frame law, the
            // temperature axis's linearly in gt^2
            let g = state.gain[i];
            let slow = 1.0 - (1.0 - g) * libm::pow(1.0 - run.slow_step, frames);
            c.slow.g = slow.clamp(0.0, 1.0) as f32;
            c.slow.left = 0;
            c.gt = libm::sqrt((g * g + run.temp_release * frames).min(1.0));
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
            filterbank_latency_ms: 2.0,
            speaker_response_lookahead_ms: 4.0 / 3.0,
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

    #[test]
    fn gated_static_model_handles_malformed_buffers_without_losing_mute() {
        let mut unit = Unit::new(params());
        assert!(!unit.is_failsafe());
        unit.companion = Some(companion::Companion {
            gates: companion::LEGACY_ESTIMATOR,
            channels: 2,
            lsm2_records: vec![(0, 0.0)],
            lsid_records: vec![(0, 0.0)],
            topology: None,
            flow: Vec::new(),
            property_64004: 0,
            flow_roles: [[0; 2]; 6],
            systems: [0; MAX_CHANNELS],
        });
        assert!(unit.is_failsafe());
        for (inputs, frames) in [(0, 9), (1, 9), (2, 10)] {
            let input = [0.5; 8];
            let ins: [&[f32]; 2] = [&input, &input];
            let mut first = [9.0; 8];
            let mut second = [9.0; 8];
            let health =
                unit.process(Io { frames, input: &ins[..inputs], sense: &[], output: &mut [&mut first, &mut second] });
            assert!(health.is_failsafe());
            assert_eq!(first, [0.0; 8]);
            assert_eq!(second, [0.0; 8]);
        }
        unit.reset();
        assert!(unit.is_failsafe());
    }

    /// A non-finite state latches 20 dB until reset.
    #[test]
    fn a_non_finite_model_state_latches_the_failsafe_until_reset() {
        let poisons: [fn(&mut Run); 3] =
            [|r| r.ch[0].th_mg = f32::NAN, |r| r.ch[1].pbar[2] = f64::INFINITY, |r| r.ch[0].slow.g = f32::NAN];
        for poison in poisons {
            let mut u = Unit::new(params());
            let (o, h) = run(&mut u, 0.01, 2048);
            assert_eq!((o[2047], h), (0.01, Health::OK));
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
            let (o, h) = run(&mut u, 0.01, 2048);
            assert_eq!((o[2047], h), (0.01, Health::OK));
        }
    }

    #[test]
    fn waterfill_levels_the_active_peaks() {
        let all = [true; BANDS];
        // no attenuation when the sum fits
        assert_eq!(waterfill(&[0.1, 0.1, 0.1, 0.1, 0.1], &all, 0.6, 0.1), [1.0; BANDS]);
        // two equal bands over the limit: each halves
        let g = waterfill(&[0.0, 0.5, 0.5, 0.0, 0.0], &all, 0.5, 0.0);
        assert!((g[1] - 0.5).abs() < 1e-12 && (g[2] - 0.5).abs() < 1e-12 && g[0] == 1.0);
        // a small band stays whole, the large one takes the rest
        let g = waterfill(&[0.1, 0.9, 0.0, 0.0, 0.0], &all, 0.5, 0.0);
        assert_eq!(g[0], 1.0);
        assert!((g[1] - 0.4 / 0.9).abs() < 1e-12);
        // inactive bands are not counted and keep unity
        let mut act = all;
        act[1] = false;
        let g = waterfill(&[0.3, 0.9, 0.3, 0.0, 0.0], &act, 0.4, 0.0);
        assert_eq!(g[1], 1.0);
        assert!((g[0] - 0.2 / 0.3).abs() < 1e-12 && (g[2] - 0.2 / 0.3).abs() < 1e-12);
        // the floor
        let g = waterfill(&[0.0, 10.0, 0.0, 0.0, 0.0], &all, 0.1, 0.1);
        assert_eq!(g[1], 0.1);
    }

    #[test]
    fn the_current_model_has_one_over_rdc_at_dc() {
        let c = Current::new(4.0, 0.8, 0.2e-3, 0.2, 1500.0, 48_000.0);
        let b: f64 = c.b.iter().map(|&v| v as f64).sum();
        let a = 1.0 + c.a[0] as f64 + c.a[1] as f64;
        assert!((b / a * 4.0 - 1.0).abs() < 1e-5);
    }
}
