//! `lsm2`: signal-only loudspeaker model and five-axis constraint
//! controller (MacBook Neo, MacBook Air M4/M5), from the behaviour
//! specification (SPEC-LSM2 v1.1 with the section 15 binary closure of
//! 2026-11-05, as corrected by SPEC-LSM2-RESIDUALS) and the 25G83
//! differential-rig laws. This is the model-only
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
//!    ├─ velocity model ─ Hann window ─ DFT ─ pressure weighting ─ median bucket power ─ level ─ limit
//!    └─ current model ─ u·i ─ input thermal model ─ temperature candidate
//! y ─ u = Vfs·y ─ current model ─ u·i ─ four averages, output thermal model (the monitor)
//! ```
//!
//! - **Models** (rig laws 1 and 4, spec 15.1 and 15.2). Displacement and
//!   velocity are the plain bilinear versions of `X/U = Bl / (R (Mms s^2 +
//!   Rms s + Kms) + Bl^2 s)` and `V = s X`, each block with its own `RdcOhms`
//!   (`ParasiticResistance` enters no model). A block whose
//!   `UseGenericModelParameters*` flag is set takes the tuning's embedded
//!   family row (the compiler resolves it). The current model is a Float32
//!   biquad whose zeros are the exponentially mapped mechanical roots and
//!   whose denominator is the minimum-phase quadratic matching the analog
//!   section's magnitude at 0, Fs/6 and Fs/3, with `1/Rdc` on the numerator
//!   (residuals section 1; the extra pole damping is an output of this
//!   construction, not a fitted loss).
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
//!     the limit follows the four-anchor velocity curve (residuals 4,
//!     extrapolated past the last anchor) of the mask level: each channel's
//!     modeled input velocity over the retained window, energy-normalised
//!     Hann, real DFT, pressure-response weighting, the median bucket power
//!     of the mask band, a spectral-shape penalty, the level tracker and the
//!     three-state follower (residuals 3); the largest level of the pair;
//!   - power (rig law 7): one gain for all systems, `min_j min(1, sqrt((B_j -
//!     N idle) / sum (Phat_j - idle)))`, `Phat_j` each system's largest
//!     average over the window of the bank-stage program before any gain
//!     (input-referred, rig law 4);
//!   - temperature (residuals 2): the input thermal model, unattenuated,
//!     on the input's own current model; its peak over the selected span of
//!     the temperature buffer, an exponential knee from the soft toward the
//!     hard limit, and the room partitioned between the coil and body rises,
//!     moving toward the DC split as the output body heats.
//! - **Gains.** Fast (band, peak): below the applied gain the target is 0.99
//!   x the request, ramped linearly over `AttackTime`; otherwise a
//!   per-sample release toward it (99 % settling over `FastReleaseTime`,
//!   `LowBandReleaseTime` for band 0). Slow (power, temperature, linked
//!   across channels): the same 0.99 attack; otherwise one 99 %-settling
//!   `SlowReleaseTime` step per frame (rig law 8). Before committing, the
//!   slow gain is checked against the output-referred monitor (residuals
//!   2.4): on the provisional trajectory the monitor at each substep is
//!   `B + g L + g^2 Q` under an extra gain `g`, and multiplicative
//!   quadratic-root corrections at the hottest substep bring every substep
//!   within `TemperatureHardLimit + max(TemperatureHeadroom, 0)`. A lower
//!   result retargets the follower: 0.99 x the request along a raised-cosine
//!   half-window of `AttackTime`, restartable. The retargeted trajectory is
//!   checked again over the frame and one attack window beyond it. The
//!   ceiling is held to 0.99 of its rise over ambient (interim). Each band's
//!   applied gain is the minimum of its fast gain, the peak gain and the
//!   slow gain (spec 2: never a product).
//! - **Monitor** (rig laws 3, 4, 11): on the protected output, `u = Vfs y`;
//!   the Float32 substep mean of `u i` drives the two thermal branches
//!   (residuals 2.1: `theta += alpha (R_eff P - theta)`), and `Pamp = idle + ratio max(mean(u i), 0) (Rdc + Rpar) /
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
//! Known residuals against 25G83 (rig report):
//! - velocity (residuals 5, 5.2): the signed allocation with the
//!   post-retarget seeded solve. Interim safety layer: it allocates to 0.99
//!   of the limit. On fb3-p30 and fb6-p62 this unit settles at 0.99-1.0 x
//!   the limit where the reference keeps 0.79-0.82 x;
//! - output ceiling (residuals 2.4, native order): exact, with an interim
//!   safety layer holding the coil to 0.99 of the ceiling's rise over
//!   ambient;
//! - the masker lattice is an input extracted from the Mac's macOS; without
//!   it there is no masking credit;
//! - host slices: at q1, q7 and q160 this unit is slice-independent by
//!   design.

// The DSP loops index several parallel per-band and per-channel arrays at
// once; an index reads better there than zipped iterators.
#![allow(clippy::needless_range_loop)]

use alloc::vec;
use alloc::vec::Vec;

mod laws;
pub use laws::{
    current_coefficients, mask_lattice_resized, minimum_phase_triplet, release_lambda, temperature_candidate,
    transform_len, velocity_band_gains, velocity_limit, BandGain, Fft, Masker, ThermalBranch,
};

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
/// The declared maximum of the `VelocityLimit*` parameters, m/s: the
/// velocity curve's last anchor (residuals section 4), not a runtime cap.
const VELOCITY_PARAM_MAX: f32 = 2.0;

/// Bits of [`ChannelParams::model_generic`]: the manager's
/// `UseGenericModelParametersRdc` and `...Resonator1` flags. A generic field
/// takes the tuning's embedded family row (resolved by the compiler) and
/// does not follow later writes of the block's own field (rig law 1).
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
    /// The masker's bucket lattice for the 640-point transform (residuals
    /// 3): 100 exclusive end bins, extracted from the Mac's own macOS at
    /// provisioning. `mask_lattice_len` is 100 when present, else 0; without
    /// it the masker gives no masking credit (the velocity limit stays at
    /// `VelocityLimit0`).
    pub mask_lattice_len: u32,
    pub mask_lattice: [u32; laws::MASK_BUCKETS],
    pub ch: [ChannelParams; MAX_CHANNELS],
}

// SAFETY: repr(C) of u32/f32 fields and arrays of them, no padding.
unsafe impl Pod for Params {}
// SAFETY: as above.
unsafe impl Pod for ChannelParams {}

const CH_WORDS: usize = 74 + PRESSURE_TAPS;
const _: () = assert!(core::mem::size_of::<ChannelParams>() == CH_WORDS * 4);
const _: () = assert!(core::mem::size_of::<Params>() == (40 + laws::MASK_BUCKETS + MAX_CHANNELS * CH_WORDS) * 4);

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
            mask_lattice_len: 0,
            mask_lattice: [0; laws::MASK_BUCKETS],
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

impl Current {
    /// `I/U = (Mms s^2 + Rms s + Kms) / (R Mms s^2 + (R Rms + Bl^2) s + R Kms)`
    /// by the generative builder of residuals section 1.
    fn new(rdc: f64, bl: f64, m: f64, rms: f64, k: f64, rate: f64) -> Self {
        let (b, a) = current_coefficients(rdc, bl, m, rms, k, rate);
        Current { b, a, u: [0.0; 2], i: [0.0; 2] }
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
    fn finite(&self) -> bool {
        self.g.is_finite() && self.step.is_finite() && self.target.is_finite()
    }
}

/// The applied slow gain (residuals 2.4): a downward decision targets 0.99
/// x the request along the first half of a periodic raised cosine of `len`
/// samples from the gain already reached, then holds; restartable. An
/// upward decision sets the frame's provisionally advanced gain.
#[derive(Clone, Copy)]
struct SlowGain {
    g: f32,
    start: f32,
    target: f32,
    n: u32,
    len: u32,
}

impl SlowGain {
    fn unity(len: u32) -> Self {
        SlowGain { g: 1.0, start: 1.0, target: 1.0, n: len, len }
    }
    /// An upward step: the gain is `g` from now on.
    fn set(&mut self, g: f32) {
        self.g = g;
        self.start = g;
        self.target = g;
        self.n = self.len;
    }
    #[inline]
    fn sample(&mut self) -> f32 {
        if self.n < self.len {
            self.n += 1;
            let w = 0.5 * (1.0 + libm::cos(core::f64::consts::PI * self.n as f64 / self.len as f64));
            self.g = (self.target as f64 + (self.start as f64 - self.target as f64) * w) as f32;
        } else {
            self.g = self.target;
        }
        self.g
    }
    fn finite(&self) -> bool {
        self.g.is_finite() && self.start.is_finite() && self.target.is_finite()
    }
}

/// Substeps in the decision window: the frame's eight plus the look-ahead
/// region's (six at 48 kHz). Bounded by one controller frame of look-ahead.
const RING: usize = 2 * SUBSTEPS;
/// The input temperature model's point ring (at least the retained history
/// plus one frame's points).
const TRING: usize = 4 * SUBSTEPS;
/// The ceiling's longest horizon in controller frames (an attack of up to
/// about 80 ms at 48 kHz).
const CEIL_FRAMES: usize = 16;

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
    /// The velocity model in its zero state (residuals 5: five fresh models
    /// per decision over the retained window).
    vel_proto: Biquad,
    /// Per-substep peaks over the window (ring of RING slots).
    pk_x: [f32; RING],
    pk_d: [f32; RING],
    pk_v: [f32; RING],
    pk_vb: [[f32; RING]; BANDS],
    // Masking path (residuals 3): the velocity model on the input, its
    // retained window (oldest first) and the spectral detector.
    masker: Option<Masker>,
    mvel: Biquad,
    vwin: Vec<f32>,
    vel_lim: [f32; 3],
    mask_knots: [f32; 4],
    vel_limit_now: f64,
    // Fast gains.
    band: [BandGain; BANDS],
    /// The modeled band velocities over the decision window: the output
    /// region's look-ahead behind it, then the frame (as `ab`).
    vb: [Vec<f32>; BANDS],
    /// Bands the velocity allocation may cut (the others are ignored).
    vel_active: (usize, usize),
    peak: Follower,
    /// The fast gains over the frame's output region, per sample.
    eg: [Vec<f32>; BANDS],
    // Power path (fast-gained bank stage): the committed state at the
    // start of the frame's output region.
    cur_in: Current,
    pbar_in: [f64; 4],
    /// Per-substep `u i` of the fast-gained bank stage, output region.
    p_sub: [f64; SUBSTEPS],
    // Temperature axis (residuals 2): the input model, unattenuated, on
    // the input's own current model; its temperature points by global
    // substep (ring), the newest at `tn - 1`.
    tin_cur: Current,
    tin_acc: f32,
    tp_vc: f32,
    tp_mg: f32,
    tring: [f32; TRING],
    tn: usize,
    gt: f64,
    // Slow gain.
    slow: SlowGain,
    // Monitor on the output.
    cur_out: Current,
    th_vc: f32,
    th_mg: f32,
    pbar: [f64; 4],
    pbar1_frame: f64,
    pbar1s_frame: f64,
    // Per-channel constants.
    load_scale: f64,
    br_vc: ThermalBranch,
    br_mg: ThermalBranch,
    tau_vc: f64,
    tau_mag: f64,
    ambient: f64,
    amp_limit: f64,
    disp_limit: f64,
    vel_floor: f64,
    vel_off: bool,
    t_soft: f64,
    t_hard: f64,
    t_headroom: f64,
    /// The logged candidates: peak, displacement, velocity, linked power,
    /// temperature.
    cand: [f64; 5],
}

impl Chan {
    /// Output sample `i` of the frame at slow gain `sg`: each band at the
    /// minimum of its fast gain and the slow gain.
    #[inline]
    fn out_sample(&self, i: usize, sg: f32) -> f32 {
        // past the frame (the look-ahead) the fast gains hold their last value
        let e = i.min(self.eg[0].len() - 1);
        let g0 = self.eg[0][e].min(sg);
        let mut same = true;
        let mut g = [0.0f32; BANDS];
        for k in 0..BANDS {
            g[k] = self.eg[k][e].min(sg);
            same &= g[k] == g0;
        }
        if same {
            g0 * self.xc[i]
        } else {
            let mut v = 0.0f32;
            for k in 0..BANDS {
                v += g[k] * self.ab[k][i];
            }
            v
        }
    }

    /// The output-temperature ceiling (residuals 2.4): on the frame's
    /// provisional output under the slow trajectory `f`, the output model at
    /// substep `k` under an extra gain `g` is `B_k + g L_k + g^2 Q_k` (the
    /// retained state, the current model's retained response crossed with
    /// the new signal, the new signal alone); multiplicative quadratic-root
    /// corrections at the hottest substep until every substep is within
    /// `TemperatureHardLimit + max(TemperatureHeadroom, 0)`.
    fn ceiling_scale(&self, mut f: SlowGain, g0: f32, ctl: usize, sub: usize) -> f32 {
        // the retained span: the frame and its known look-ahead
        let od = self.xc.len() - ctl;
        let horizon = ctl + od;
        let subs = horizon / sub;
        let mut base = [0.0f32; CEIL_FRAMES * SUBSTEPS];
        let mut lin = [0.0f32; CEIL_FRAMES * SUBSTEPS];
        let mut quad = [0.0f32; CEIL_FRAMES * SUBSTEPS];
        let mut plin = [0.0f32; CEIL_FRAMES * SUBSTEPS];
        let mut pquad = [0.0f32; CEIL_FRAMES * SUBSTEPS];
        let mut zi = self.cur_out;
        let mut zs = self.cur_out;
        zs.clear();
        let (mut bv, mut bm) = (self.th_vc, self.th_mg);
        let (mut lv, mut lm, mut qv, mut qm) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        let (mut al, mut aq) = (0.0f32, 0.0f32);
        let amb = self.ambient as f32;
        for i in 0..horizon {
            let u = self.vfs * self.out_sample(i, f.sample());
            al += u * zi.step(0.0);
            aq += u * zs.step(u);
            if (i + 1) % sub == 0 {
                let k = i / sub;
                let (pl, pq) = (al / sub as f32, aq / sub as f32);
                plin[k] = pl;
                pquad[k] = pq;
                al = 0.0;
                aq = 0.0;
                bv = self.br_vc.step(bv, 0.0);
                bm = self.br_mg.step(bm, 0.0);
                lv = self.br_vc.step(lv, pl);
                lm = self.br_mg.step(lm, pl);
                qv = self.br_vc.step(qv, pq);
                qm = self.br_mg.step(qm, pq);
                base[k] = amb + bv + bm;
                lin[k] = lv + lm;
                quad[k] = qv + qm;
            }
        }
        // interim safety layer until this unit's full audio matches the
        // reference (the exact solve lands 0.01-0.04 C nearer the ceiling
        // than the reference on the rig): the ceiling's rise over ambient
        // by the 0.99 law
        let limit = (self.ambient + CUT * (self.t_hard + self.t_headroom.max(0.0) - self.ambient)) as f32;
        Self::ceiling_solve(&base[..subs], &lin[..subs], &quad[..subs], limit, g0)
    }

    /// The multiplicative quadratic-root solve of residuals 2.4 on the
    /// substep decomposition `base + g lin + g^2 quad`.
    fn ceiling_solve(base: &[f32], lin: &[f32], quad: &[f32], limit: f32, g0: f32) -> f32 {
        let subs = base.len();
        let mut g = g0.min(1.0);
        loop {
            let (mut kmax, mut tmax) = (0usize, f32::NEG_INFINITY);
            for k in 0..subs {
                let t = (g * lin[k] + (g * g) * quad[k]) + base[k];
                if t > tmax {
                    tmax = t;
                    kmax = k;
                }
            }
            if tmax <= limit || tmax.is_nan() {
                return g;
            }
            let room = limit - base[kmax];
            if room <= 0.0 {
                return 0.0;
            }
            let (a, b) = (quad[kmax], lin[kmax]);
            let ratio = if a > 0.0 {
                let disc = b * b + (4.0 * a) * room;
                (libm::sqrtf(disc.max(0.0)) - b) / (a + a)
            } else if b > 0.0 {
                room / b
            } else {
                return 0.0;
            };
            let ratio = ratio.clamp(0.0, 1.0);
            if ratio == 1.0 {
                return g;
            }
            let next = g * ratio;
            if next == 0.0 || next == g {
                return next;
            }
            g = next;
        }
    }
}

pub struct Unit {
    params: Params,
    run: Option<Run>,
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
    /// The temperature buffer (residuals 2.3): `tb_hist` retained points,
    /// the peak over its oldest `tb_sel` points with the frame's.
    tb_hist: usize,
    tb_sel: usize,
    axis: [bool; 5],
    link: bool,
    unlink_temp: bool,
    p_pole: [f64; 4],
    budget: [f64; 4],
    idle: f64,
    ratio: f64,
    attack: u32,
    r_fast: f32,
    a_fast: f64,
    a_low: f64,
    slow_step: f64,
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
        Unit { params, run }
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
        self.run.is_none()
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
        // residuals 2.4: AttackTime clamped to 10 ms, the raised-cosine
        // window at most one controller frame
        let attack_len = (libm::round((p.attack_time_ms as f64 * 1e-3).min(0.010) * rate) as u32).clamp(1, ctl as u32);
        let mut ch = Vec::with_capacity(n);
        for c in &p.ch[..n] {
            let h = filterbank(rate, &c.band_center_hz, c.bandwidth, fb);
            let bank: [Vec<f32>; BANDS - 1] = core::array::from_fn(|k| h[k].clone());
            let taps = c.pressure_taps as usize;
            let mwin = (ctl + p.latency_frames()).min(laws::MASK_MAX_POINTS);
            // no extracted lattice (or no pressure response): no masking
            // credit, the velocity limit stays at its lowest anchor
            let masker = if taps > 0 && p.mask_lattice_len as usize == laws::MASK_BUCKETS {
                Masker::new(
                    &p.mask_lattice,
                    mwin,
                    rate,
                    &c.pressure_fir[..taps],
                    c.mask_band_hz,
                    c.mask_attack_s,
                    c.mask_release_s,
                    c.mask_attack_fast_s,
                    c.mask_attack_range_db,
                    ctl,
                )
            } else {
                None
            };
            let vel = motion(c.vel_rdc, c.vel_bl, c.vel_mms_kg, c.vel_rms, c.vel_kms, rate, true);
            let cur = Current::new(c.rdc as f64, c.bl as f64, c.mms_kg as f64, c.rms as f64, c.kms as f64, rate);
            let br_vc = ThermalBranch::new(c.rth_vc as f64, c.tau_vc as f64, sub_dt);
            let br_mg = ThermalBranch::new(c.rth_mag as f64, c.tau_mag as f64, sub_dt);
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
                vel_proto: vel,
                pk_x: [0.0; RING],
                pk_d: [0.0; RING],
                pk_v: [0.0; RING],
                pk_vb: [[0.0; RING]; BANDS],
                masker,
                mvel: vel,
                vwin: vec![0.0; mwin],
                vel_lim: c.velocity_limit,
                mask_knots: c.mask_energy_db,
                vel_limit_now: c.velocity_limit[0] as f64,
                band: [BandGain::unity(attack_len); BANDS],
                vb: core::array::from_fn(|_| vec![0.0; od + ctl]),
                vel_active: (
                    (c.velocity_ignore_low_band != 0) as usize,
                    if c.velocity_ignore_high_band != 0 { BANDS - 1 } else { BANDS },
                ),
                peak: Follower::UNITY,
                eg: core::array::from_fn(|_| vec![1.0; ctl]),
                cur_in: cur,
                pbar_in: [0.0; 4],
                p_sub: [0.0; SUBSTEPS],
                tin_cur: cur,
                tin_acc: 0.0,
                tp_vc: 0.0,
                tp_mg: 0.0,
                tring: [c.ambient; TRING],
                tn: 0,
                gt: 1.0,
                slow: SlowGain::unity(attack_len),
                cur_out: cur,
                th_vc: 0.0,
                th_mg: 0.0,
                pbar: [0.0; 4],
                pbar1_frame: 0.0,
                pbar1s_frame: 0.0,
                load_scale: (c.rdc as f64 + c.parasitic_resistance as f64) / c.rdc as f64,
                br_vc,
                br_mg,
                tau_vc: c.tau_vc as f64,
                tau_mag: c.tau_mag as f64,
                ambient: c.ambient as f64,
                amp_limit: c.amplitude_limit as f64 - c.pilot_reserve as f64,
                disp_limit: c.displacement_limit as f64,
                vel_floor: db_to_lin(c.max_velocity_attenuation_db as f64),
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
            tb_hist: p.latency_frames() / sub,
            tb_sel: p.latency_frames() / sub + SUBSTEPS - fb / sub,
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
            a_fast: release_lambda(rate, p.fast_release_time_ms as f64 * 1e-3),
            a_low: release_lambda(rate, p.low_band_release_time_ms as f64 * 1e-3),
            slow_step: settle_step(ctl_dt, p.slow_release_time_ms as f64 * 1e-3),
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
            if let Some(m) = c.masker.as_mut() {
                m.clear();
            }
            c.mvel.clear();
            c.vwin.iter_mut().for_each(|v| *v = 0.0);
            c.vel_limit_now = c.vel_lim[0] as f64;
            c.band = [BandGain::unity(c.slow.len); BANDS];
            c.vb.iter_mut().for_each(|b| b.iter_mut().for_each(|v| *v = 0.0));
            c.peak = Follower::UNITY;
            c.eg.iter_mut().for_each(|e| e.iter_mut().for_each(|v| *v = 1.0));
            c.cur_in.clear();
            c.pbar_in = [0.0; 4];
            c.p_sub = [0.0; SUBSTEPS];
            c.tin_cur.clear();
            c.tin_acc = 0.0;
            c.tp_vc = 0.0;
            c.tp_mg = 0.0;
            c.tring = [c.ambient as f32; TRING];
            c.tn = 0;
            c.gt = 1.0;
            c.slow = SlowGain::unity(c.slow.len);
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
        if c.masker.is_some() {
            c.vwin.copy_within(ctl.., 0);
        }
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
            // the masker's modeled velocity of the input (residuals 3)
            if c.masker.is_some() {
                let mw = c.vwin.len();
                c.vwin[mw - ctl + i] = c.mvel.step(vfs * xin[i] as f64) as f32;
            }
            // the input temperature model, unattenuated (residuals 2)
            let ut = c.vfs * xin[i];
            let it = c.tin_cur.step(ut);
            c.tin_acc += ut * it;
            if (i + 1) % sub == 0 {
                let p = (c.tin_acc / sub as f32).max(0.0);
                c.tin_acc = 0.0;
                c.tp_vc = c.br_vc.step(c.tp_vc, p);
                c.tp_mg = c.br_mg.step(c.tp_mg, p);
                c.tring[c.tn % TRING] = c.ambient as f32 + c.tp_vc + c.tp_mg;
                c.tn += 1;
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
        if !self.ch.iter().all(|c| c.band.iter().all(BandGain::finite) && c.peak.finite() && c.slow.finite()) {
            self.fault = true;
            return;
        }
        for ci in 0..self.n {
            self.analyse(ci);
        }
        self.ring = (self.ring + SUBSTEPS) % self.ring_len;
        let (ctl, sub, od, attack) = (self.ctl, self.sub, self.od, self.attack);
        let r_fast = self.r_fast;
        let ring_len = self.ring_len;
        let window_max = |r: &[f32; RING]| r[..ring_len].iter().copied().fold(0.0f32, f32::max);
        let ratio = |limit: f64, q: f32| (limit / (q as f64).max(EPS)).min(1.0);
        let (a_fast, a_low) = (self.a_fast, self.a_low);
        // the allocator span excludes the first millisecond (t-4719)
        let vfirst = libm::round(self.ctl as f64 * 3.0 / 16.0) as usize;
        // ---- fast axes: candidates, band requests, the frame's trajectories
        // mask level (residuals 3): each channel's detector, the largest
        // level for every channel's velocity curve
        let mut level = f32::NEG_INFINITY;
        for c in self.ch.iter_mut() {
            if let Some(m) = c.masker.as_mut() {
                level = level.max(m.update(&c.vwin));
            }
        }
        for c in self.ch.iter_mut() {
            if c.masker.is_some() {
                c.vel_limit_now = velocity_limit(level, &c.mask_knots, &c.vel_lim, VELOCITY_PARAM_MAX) as f64;
            }
            let g_peak = if self.axis[0] { ratio(c.amp_limit, window_max(&c.pk_x)) } else { 1.0 };
            let g_disp = if self.axis[1] { ratio(c.disp_limit, window_max(&c.pk_d)) } else { 1.0 };
            let vel_on = self.axis[2] && !c.vel_off;
            let g_vel = if self.axis[2] { ratio(c.vel_limit_now, window_max(&c.pk_v)).max(c.vel_floor) } else { 1.0 };
            // residuals 5: the signed worst-sample allocation on the band
            // velocities under the applied slow gain, then the absolute floor
            // (the logged candidate is only a diagnostic). It runs over the
            // whole decision window, the output region and its look-ahead,
            // so every sample these gains will shape holds the limit.
            let sg = (c.slow.g as f64).clamp(EPS, 1.0) as f32;
            if vel_on {
                // residuals 5 (t-4719): five fresh zero-state velocity models,
                // one per band including the complementary fifth, over the
                // retained bank window (output region and look-ahead), on
                // the unattenuated bank signals at full-scale volts
                let vfs = c.vfs as f64;
                for k in 0..BANDS {
                    let mut m = c.vel_proto;
                    for i in 0..c.vb[k].len() {
                        c.vb[k][i] = m.step(vfs * c.ab[k][i] as f64) as f32;
                    }
                }
            }
            let gv: [f64; BANDS] = if vel_on {
                // interim safety layer: allocate to 0.99 of the limit so the
                // attack transient stays within it
                let vlim_g = (CUT * c.vel_limit_now) as f32;
                let floor = c.vel_floor as f32;
                let absolute = |rel: [f32; BANDS]| -> [f32; BANDS] {
                    core::array::from_fn(|k| if rel[k] < 1.0 { (sg * rel[k]).max(floor).min(1.0) } else { 1.0 })
                };
                // the ordinary solve, from unity
                let p = absolute(velocity_band_gains(&c.vb, vfirst, vlim_g, sg, c.vel_active, [1.0; BANDS]));
                // residuals 5.2 (t-4713): a lane is upward when its primary
                // request is above its applied gain; then one second solve,
                // seeded from the predicted one-frame endpoint of upward lanes
                // still 3 dB or more below their request; upward lanes take the
                // larger of the two results, the others the second
                let up: [bool; BANDS] = core::array::from_fn(|k| p[k] > c.band[k].g);
                if up.iter().any(|&u| u) {
                    let guard = libm::pow(10.0, 3.0 / 20.0) as f32;
                    let seed: [f32; BANDS] = core::array::from_fn(|k| {
                        let end =
                            c.band[k].end_after((g_disp as f32).min(p[k]), ctl, if k == 0 { a_low } else { a_fast });
                        if up[k] && guard * end <= p[k] {
                            (end / sg).min(1.0)
                        } else {
                            1.0
                        }
                    });
                    let q = absolute(velocity_band_gains(&c.vb, vfirst, vlim_g, sg, c.vel_active, seed));
                    core::array::from_fn(|k| if up[k] { p[k].max(q[k]) as f64 } else { q[k] as f64 })
                } else {
                    p.map(|v| v as f64)
                }
            } else {
                [1.0; BANDS]
            };
            for k in 0..BANDS {
                let req = g_disp.min(gv[k]) as f32;
                c.band[k].decide(req);
            }
            c.peak.request(g_peak as f32, attack);
            c.cand[0] = g_peak;
            c.cand[1] = g_disp;
            c.cand[2] = g_vel;
            for i in 0..ctl {
                let pg = c.peak.sample(r_fast);
                for k in 0..BANDS {
                    let a = if k == 0 { a_low } else { a_fast };
                    c.eg[k][i] = c.band[k].sample(a).min(pg);
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
        // ---- temperature axis (residuals 2): the input model's selected
        // peak, its current rises and the output body rise before this frame
        let (hist, sel) = (self.tb_hist, self.tb_sel);
        for c in self.ch.iter_mut() {
            // buffer: the `hist` retained points then the frame's, oldest
            // first; the peak over its first `sel`
            let start = c.tn + TRING - hist - SUBSTEPS;
            let tpk = (0..sel).map(|j| c.tring[(start + j) % TRING]).fold(f32::NEG_INFINITY, f32::max);
            c.gt = if self.axis[4] {
                temperature_candidate(
                    tpk,
                    c.tp_vc,
                    c.tp_mg,
                    c.th_mg,
                    c.ambient as f32,
                    c.t_soft as f32,
                    c.t_hard as f32,
                    c.br_vc.r_eff,
                    c.br_mg.r_eff,
                ) as f64
            } else {
                1.0
            };
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
        // residuals 2.4, in its native order: (1) the slow follower is
        // advanced provisionally toward unity for the frame; (2) the
        // ordinary request becomes a scalar on that trajectory and the
        // output-temperature ceiling is solved once from it on the retained
        // frame and its look-ahead; (3) a lower result makes one second
        // follower call, from the provisionally advanced value toward 0.99
        // x the request
        let mut prov = [SlowGain::unity(1); MAX_CHANNELS];
        let mut end = [1.0f32; MAX_CHANNELS];
        let mut want = [1.0f32; MAX_CHANNELS];
        for (ci, c) in self.ch.iter().enumerate() {
            let mut f = c.slow;
            if f.n >= f.len {
                let a = f.g as f64;
                f.set((a + self.slow_step * (1.0 - a)).min(1.0) as f32);
            }
            prov[ci] = f;
            let mut e = f;
            for _ in 0..ctl {
                e.sample();
            }
            end[ci] = e.g;
            want[ci] = (req[ci] as f32 / e.g.max(f32::MIN_POSITIVE)).min(1.0);
        }
        if self.axis[4] {
            for (ci, c) in self.ch.iter().enumerate() {
                want[ci] = c.ceiling_scale(prov[ci], want[ci], ctl, sub);
            }
        }
        if self.link && n > 1 {
            let m = want[..n].iter().copied().fold(1.0f32, f32::min);
            want[..n].iter_mut().for_each(|v| *v = m);
        }
        for (ci, c) in self.ch.iter_mut().enumerate() {
            let request = want[ci] * end[ci];
            if request < end[ci] {
                let mut f = prov[ci];
                for _ in 0..ctl {
                    f.sample();
                }
                // the second call starts at the provisionally advanced value
                f.start = f.g;
                f.target = (CUT * request as f64) as f32;
                f.n = 0;
                c.slow = f;
            } else {
                c.slow = prov[ci];
            }
        }
        // ---- output region and the monitor
        let (idle, ratio_p) = (self.idle, self.ratio);
        for (ci, c) in self.ch.iter_mut().enumerate() {
            let y = &mut self.yout[ci];
            let mut acc = 0.0f64;
            let mut acc32 = 0.0f32;
            let mut s = 0;
            let (mut p1, mut p1s) = (0.0f64, 0.0f64);
            for (i, yo) in y.iter_mut().enumerate().take(ctl) {
                let sg = c.slow.sample();
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
                acc32 += u * iv;
                *yo = v.clamp(-1.0, 1.0);
                if (i + 1) % sub == 0 {
                    let p = acc / sub as f64;
                    acc = 0.0;
                    // the output thermal model (residuals 2.1)
                    let pp = (acc32 / sub as f32).max(0.0);
                    acc32 = 0.0;
                    c.th_vc = c.br_vc.step(c.th_vc, pp);
                    c.th_mg = c.br_mg.step(c.th_mg, pp);
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
                && c.masker.as_ref().is_none_or(Masker::is_finite)
                && c.pbar.iter().chain(&c.pbar_in).all(|v| v.is_finite())
                && c.gt.is_finite();
            let motion = [&c.disp, &c.vel].iter().all(|b| b.y[0].is_finite())
                && c.vel_band.iter().all(|b| b.y[0].is_finite())
                && c.cur_in.is_finite()
                && c.cur_out.is_finite();
            let gains = c.cand.iter().all(|v| v.is_finite())
                && c.band.iter().all(BandGain::finite)
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
            let t = c.ambient + c.th_mg as f64 + c.th_vc as f64;
            s.modeled_power_w = c.pbar[2] as f32;
            s.coil_temp_c = t as f32;
            s.magnet_temp_c = (c.ambient + c.th_mg as f64) as f32;
            s.control_temp_c = t as f32;
            s.ol_temp_c = t as f32;
            s.velocity_limit_m_s = c.vel_limit_now as f32;
            s.mask_db = c.masker.as_ref().map_or(f32::NAN, |m| m.level);
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
            // the input model's buffer holds the cooled temperature
            c.tring = [c.ambient as f32 + c.tp_vc + c.tp_mg; TRING];
            for j in 0..4 {
                let v = run.idle + (s[2 + j] - run.idle) * libm::pow(run.p_pole[j], substeps);
                c.pbar[j] = v;
                c.pbar_in[j] = v;
            }
            // the slow gain released over the gap by its per-frame law; the
            // temperature candidate follows from the restored states
            let g = state.gain[i];
            let slow = 1.0 - (1.0 - g) * libm::pow(1.0 - run.slow_step, frames);
            c.slow.set(slow.clamp(0.0, 1.0) as f32);
            c.gt = 1.0;
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
}
