//! `spp3`: feed-forward thermal protection per channel with linked speaker
//! groups, from the behaviour specification (sections 3 to 6, 10, 12, 16).
//!
//! - **Cadence.** The thermal model and controller update every 256 frames
//!   (the reference splits its 512-frame graph call into two thermal
//!   updates). Hosts may call `process` with any frame count; state and
//!   audio are bit-identical for any slicing.
//! - **Audio.** Not delayed (latency 0). `out = in * headroom * g[n]`, where
//!   `headroom = 1 - 10^(PilotAmplHi_dB/20)` on channels with
//!   `PilotToneEnabled` (else 1) and `g[n]` is the group's dezippered gain.
//! - **Model input.** The protected drive passes through the
//!   `Thermal_LookaheadDelay_ms` model-alignment history (96 frames at
//!   48 kHz) before metering, so the first update after reset sees 96 empty
//!   frames. `p = (ag*x)^2 * Reb / (Reb + Rshunt + Rampout)^2`, the mean over
//!   the update.
//! - **Thermal filters.** Trapezoidal-input, linear-decay recurrences at
//!   `dt = 256/fs`: `theta_j = (1 - dt/tau_j) theta_j + 0.5 T_sett_j (dt/tau_j)
//!   (P + P_prev)` for the coil (`T_sett_vc`, `tau_Tvc`), the magnet
//!   (`T_sett_mg`, `tau_Tmg`) and the feed-forward virtual coil (`T_sett_vc`,
//!   `tau_ff = ThermalFFSpeedupFactor * tau_Tvc`). `T_OL = ambient +
//!   theta_vc + theta_mg`.
//! - **Candidate.** `g_ff = sqrt(clamp((Tlim - ambient - theta_mg) /
//!   (theta_ff + 1 C), 0.01, 1))`; a nonpositive speed-up factor disables
//!   it (unity). The virtual coil is driven by the unprotected program
//!   (before gain and pilot headroom), which is what makes the spec's
//!   steady state `g = sqrt(P_ceiling / P_drive)` and reproduces the
//!   reference's 137 C pilot-mode equilibrium; see `FF_FROM_UNPROTECTED`.
//! - **Smoother state.** After the hard path the gain releases from the
//!   selected value (the smoother's state is the selected gain), as the
//!   reference does; TODO(SPEC) to confirm in the text.
//! - **Smoothing.** The group minimum of `g_ff` is smoothed per update in
//!   linear gain with `k = 1 - exp(ln(0.01) / (T * fs/256))` (attack or
//!   release by the sign of the step: the named times are 99 % settling).
//! - **Hard path.** Arms per channel when `T_OL > Tlim + HardTempLimitHeadroom`,
//!   disarms when `T_OL < Tlim`; while armed `g_hard = sqrt(clamp((Tlim - 5 C
//!   - ambient - theta_mg) / (theta_ff + 1 C), 0.01, 1))`. The selected
//!   target is `min(smoothed g_ff, group min g_hard)`, bypassing the attack.
//! - **Dezipper.** The selected target crosses two cascaded per-sample
//!   one-poles, `k = 1 - exp(ln(0.01) / (0.005 fs))`.
//! - **Fail-safe.** A channel-count mismatch, or non-finite drive (the
//!   simple-meter failure), gives 20 dB for the affected frames.
//! - **Reset.** The parameters' start state (64007 = 2: ambient, the macOS
//!   configuration), unity gains, model history cleared, CL temperature
//!   reported as 20 C.
//!
//! TODO(SPEC):
//! - the generated pilot sine and its content-gate envelope (spec 18: the
//!   exact threshold/envelope transfer is open). The program headroom is
//!   applied; the pilot itself is not generated yet, so its power is absent
//!   from the drive and the model;
//! - `Thermal_SafeTlim*` (open; never relaxes below the ordinary limit, so
//!   leaving it out is not less restrictive);
//! - the hard candidate when the feed-forward filter is disabled (the
//!   virtual coil is then taken as the real coil);
//! - closed loop (spec 7), implemented: V and I mean squares per
//!   `TempSenseWindowTime` window with the independent mean-square floors,
//!   `R = RdcScale * smooth(sqrt(V_ms / I_ms))` accepted in `RebRange`,
//!   `T_CL = T_cal + (R / R_cal - 1) / alpha(T_cal)` with alpha linear
//!   between `a_t_20C` and `a_t_35C`, and the corrected limit
//!   `clamp(Tlim + T_OL - T_CL, Tlim, Tlim + TlimAdjCap)` smoothed with the
//!   99 % law over `TlimErrDecayTime`, falling back to `Tlim` on rejection.
//!   A channel runs it only with CL feedback configured and a calibration
//!   record inside `CalTempRange`/`RebRange`. Still TODO(SPEC): the
//!   sensor's DC-block/band-limit filter (raw samples are used, exact for a
//!   resistive load), the resistance smoother's coefficient convention and
//!   start (99 % law, starting at the calibration), and whether the hard
//!   path follows the corrected limit (it does here, as the reference's
//!   closed-loop runs show);
//!
//! A preset that enables displacement control or CPMS, or disables thermal
//! protection, is refused and the unit runs the fail-safe.
//!
//! Tuning keys ([`TuningLookup`]) are the sidecar names where the sidecar
//! has the value, otherwise the public AU parameter name with spaces
//! removed: `ThermalProtectionEnabled` (global 0), `EnableCPMSControl`
//! (18), `EnableCPMSClosedLoopControl` (19), `PilotToneEnabled` (part 21),
//! `CLThermalFeedbackEnabled` (part 22), `PilotAmplHi_dB` (part 27).
//! TODO(lane A): confirm the reader exposes these.

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::delay::Delay;
use crate::dsp;
use crate::sense::{BlockSums, DelayEstimator};
use crate::{ChannelState, Health, Io, ProtectionUnit, Telemetry, UnitKind, FAILSAFE_GAIN, MAX_CHANNELS};

/// CL temperature reported before any valid measurement (spec section 10).
const T_CL_UNMEASURED: f32 = 20.0;

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
    /// `PilotToneEnabled` (part 21, 0 or 1) and `PilotAmplHi_dB` (part 27).
    pub pilot_tone_enabled: u32,
    pub pilot_ampl_hi_db: f32,
    /// CL dictionary.
    pub temp_sense_window_time_s: f32,
    pub temp_sense_smoothing_tau_s: f32,
    pub rdc_scale: f32,
    pub reb_range_min: f32,
    pub reb_range_max: f32,
    pub cal_temp_range_min: f32,
    pub cal_temp_range_max: f32,
    pub a_t_20c: f32,
    pub a_t_35c: f32,
    pub tlim_err_decay_time_s: f32,
    pub tlim_adj_cap: f32,
    /// Per-unit calibration (SpTS record bound by data source; 64009):
    /// present flag, resistance (ohm) and temperature (C).
    pub cal_present: u32,
    pub cal_r: f32,
    pub cal_t: f32,
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
// u32/f32 fields: 14 + 8 * 39 four-byte words, no padding.
unsafe impl Pod for Params {}
// SAFETY: as above, 39 four-byte words.
unsafe impl Pod for ChannelParams {}

const _: () = assert!(core::mem::size_of::<ChannelParams>() == 39 * 4);
const _: () = assert!(core::mem::size_of::<Params>() == (14 + MAX_CHANNELS * 39) * 4);

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
                pilot_tone_enabled: (f("PilotToneEnabled")? != 0.0) as u32,
                pilot_ampl_hi_db: f("PilotAmplHi_dB")?,
                temp_sense_window_time_s: f("TempSenseWindowTime")?,
                temp_sense_smoothing_tau_s: f("TempSenseSmoothingTau")?,
                rdc_scale: f("RdcScale")?,
                reb_range_min: f("RebRangeMin")?,
                reb_range_max: f("RebRangeMax")?,
                cal_temp_range_min: f("CalTempRangeMin")?,
                cal_temp_range_max: f("CalTempRangeMax")?,
                a_t_20c: f("a_t_20C")?,
                a_t_35c: f("a_t_35C")?,
                tlim_err_decay_time_s: f("TlimErrDecayTime")?,
                tlim_adj_cap: f("TlimAdjCap")?,
                // runtime calibration overlay; absent means open loop
                cal_present: t.channel(c, "sgv2.CalPresent").is_some_and(|v| v != 0.0) as u32,
                cal_r: t.channel(c, "sgv2.CalR").unwrap_or(0.0) as f32,
                cal_t: t.channel(c, "sgv2.CalT").unwrap_or(0.0) as f32,
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
            if !(c.rampout.is_finite() && c.rampout >= 0.0) {
                return Err(InitError::Invalid("Rampout"));
            }
            if c.pilot_tone_enabled != 0 && !(c.pilot_ampl_hi_db.is_finite() && c.pilot_ampl_hi_db <= 0.0) {
                return Err(InitError::Invalid("PilotAmplHi_dB"));
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

/// Thermal update cadence, frames (spec 6: 256 at 48 kHz).
pub const THERMAL_FRAMES: usize = 256;
/// Hard-coded prediction margin in the candidate, C (spec 6).
const PREDICTION_MARGIN: f64 = 1.0;
/// Hard-path target below the limit, C (spec 6).
const HARD_TARGET_BELOW: f64 = 5.0;
/// Clamp of the candidate ratio (spec 6).
const RATIO_MIN: f64 = 0.01;
/// Dezipper 99 % settling time, s (spec 6).
const DEZIPPER_S: f64 = 0.005;
const MODEL_SCRATCH: usize = 256;
/// TODO(SPEC): which power drives the feed-forward virtual coil. The spec's
/// recurrence names `P`; its steady-state statement (`g = sqrt(P_ceiling /
/// P_drive)`, unprotected `P_drive`) holds only if the virtual coil sees
/// the unprotected drive. With the protected drive the equilibrium sits
/// above the limit whenever the gain is well below unity.
const FF_FROM_UNPROTECTED: bool = true;
/// TODO(SPEC): the resistance smoother's coefficient convention.
const CL_EFOLD: bool = false;

/// The reference's coefficient law: a one-pole that settles to 99 % in
/// `seconds` at `rate` updates per second.
pub fn settle99(seconds: f64, rate: f64) -> f64 {
    1.0 - libm::exp(libm::log(0.01) / (seconds * rate))
}

/// Trapezoidal-input, linear-decay thermal filter (spec 5).
#[derive(Clone, Copy, Default)]
struct Thermal {
    a: f64,
    b: f64,
    theta: f64,
}

impl Thermal {
    fn new(r_th: f64, tau: f64, dt: f64, start: f64) -> Self {
        Thermal { a: 1.0 - dt / tau, b: 0.5 * r_th * dt / tau, theta: start }
    }
    #[inline]
    fn step(&mut self, p: f64, p_prev: f64) {
        self.theta = self.a * self.theta + self.b * (p + p_prev);
    }
}

/// Closed-loop resistance thermometry and the corrected limit (spec 7).
struct Cl {
    window: usize,
    n: usize,
    v_sq: f64,
    i_sq: f64,
    v_scale2: f64,
    i_scale2: f64,
    v_floor: f64,
    i_floor: f64,
    k_r: f64,
    r_s: Option<f64>,
    rdc_scale: f64,
    reb_min: f64,
    reb_max: f64,
    cal_r: f64,
    cal_t: f64,
    inv_temp_co: f64,
    cap: f64,
    k_lim: f64,
    /// Latest accepted window, until the next update consumes it.
    accepted: Option<f64>,
    valid: bool,
    t_cl: f64,
    limit_s: f64,
    /// Diagnostics of the last completed window (NaN before the first).
    last_v_ms: f64,
    last_i_ms: f64,
}

impl Cl {
    /// `None` (open-loop kernel) unless CL thermal feedback is configured
    /// and a calibration record inside the calibration ranges is present.
    fn new(c: &ChannelParams, rate: f64, r: f64) -> Option<Cl> {
        if c.cl_thermal_feedback_enabled == 0 || c.cal_present == 0 || c.ignore_telemetry != 0 {
            return None;
        }
        let (rc, tc) = (c.cal_r as f64, c.cal_t as f64);
        if !(rc.is_finite() && tc.is_finite() && rc > 0.0) {
            return None;
        }
        if !(tc >= c.cal_temp_range_min as f64 && tc <= c.cal_temp_range_max as f64) {
            return None;
        }
        if !(rc >= c.reb_range_min as f64 && rc <= c.reb_range_max as f64) {
            return None;
        }
        let window = libm::round(c.temp_sense_window_time_s as f64 * rate) as usize;
        if window == 0 || !(c.temp_sense_smoothing_tau_s > 0.0 && c.tlim_err_decay_time_s > 0.0) {
            return None;
        }
        let (a20, a35) = (c.a_t_20c as f64, c.a_t_35c as f64);
        let alpha = a20 + (tc - 20.0) * (a35 - a20) / 15.0;
        if !(alpha.is_finite() && alpha > 0.0) {
            return None;
        }
        let (vs, is) = (c.vmon_scale as f64, c.imon_scale as f64);
        Some(Cl {
            window,
            n: 0,
            v_sq: 0.0,
            i_sq: 0.0,
            v_scale2: vs * vs,
            i_scale2: is * is,
            v_floor: c.v_pwr_thres as f64,
            i_floor: c.i_pwr_thres as f64,
            // TODO(SPEC): the smoothing coefficient convention; the
            // reference's 99 %-settling law at the window rate
            k_r: if CL_EFOLD {
                1.0 - libm::exp(-(window as f64 / rate) / c.temp_sense_smoothing_tau_s as f64)
            } else {
                settle99(c.temp_sense_smoothing_tau_s as f64, rate / window as f64)
            },
            // the smoothed resistance starts at the calibration (T_CL starts
            // at the calibration temperature, as the reference reports)
            r_s: Some(rc / c.rdc_scale.max(f32::MIN_POSITIVE) as f64),
            rdc_scale: c.rdc_scale as f64,
            reb_min: c.reb_range_min as f64,
            reb_max: c.reb_range_max as f64,
            cal_r: rc,
            cal_t: tc,
            inv_temp_co: 1.0 / alpha,
            cap: c.tlim_adj_cap as f64,
            k_lim: settle99(c.tlim_err_decay_time_s as f64, r),
            accepted: None,
            valid: false,
            t_cl: tc,
            limit_s: c.temperature_limit as f64,
            last_v_ms: f64::NAN,
            last_i_ms: f64::NAN,
        })
    }

    fn reset(&mut self, limit: f64) {
        self.n = 0;
        self.v_sq = 0.0;
        self.i_sq = 0.0;
        self.r_s = Some(self.cal_r / self.rdc_scale.max(f64::MIN_POSITIVE));
        self.accepted = None;
        self.valid = false;
        self.t_cl = self.cal_t;
        self.limit_s = limit;
        self.last_v_ms = f64::NAN;
        self.last_i_ms = f64::NAN;
    }

    /// Raw telemetry of one segment. TODO(SPEC): the sensor's DC-block /
    /// band-limit filter is not specified; the mean squares are taken on
    /// the raw scaled samples (exact for a resistive load).
    fn feed(&mut self, i: &[f32], v: &[f32]) {
        let mut k = 0;
        while k < i.len() {
            let m = (self.window - self.n).min(i.len() - k);
            self.v_sq += dsp::sum_sq(&v[k..k + m]);
            self.i_sq += dsp::sum_sq(&i[k..k + m]);
            self.n += m;
            k += m;
            if self.n == self.window {
                let nn = self.window as f64;
                let v_ms = self.v_sq * self.v_scale2 / nn;
                let i_ms = self.i_sq * self.i_scale2 / nn;
                self.n = 0;
                self.v_sq = 0.0;
                self.i_sq = 0.0;
                self.last_v_ms = v_ms;
                self.last_i_ms = i_ms;
                if v_ms.is_finite() && i_ms.is_finite() && v_ms > self.v_floor && i_ms > self.i_floor {
                    let r = libm::sqrt(v_ms / i_ms);
                    let rs = match self.r_s {
                        Some(p) => p + self.k_r * (r - p),
                        None => r,
                    };
                    self.r_s = Some(rs);
                    let re = self.rdc_scale * rs;
                    if re >= self.reb_min && re <= self.reb_max {
                        self.accepted = Some(self.cal_t + (re / self.cal_r - 1.0) * self.inv_temp_co);
                        self.valid = true;
                    } else {
                        self.valid = false;
                    }
                } else {
                    self.valid = false;
                }
            }
        }
    }

    /// Per thermal update: the smoothed corrected limit.
    fn update(&mut self, t_ol: f64, limit: f64) -> f64 {
        if let Some(t) = self.accepted.take() {
            self.t_cl = t;
        }
        let raw = if self.valid { (limit + (t_ol - self.t_cl)).clamp(limit, limit + self.cap) } else { limit };
        self.limit_s += self.k_lim * (raw - self.limit_s);
        self.limit_s
    }
}

struct Chan {
    src: usize,
    group: usize,
    headroom: f32,
    /// Model-alignment history.
    delay: Delay,
    scratch: Vec<f32>,
    elec: f64,
    limit: f64,
    hard_on: f64,
    vc: Thermal,
    mg: Thermal,
    ff: Option<Thermal>,
    init_vc: f64,
    init_mg: f64,
    p_prev: f64,
    p_ff_prev: f64,
    acc: dsp::LaneAcc,
    /// Unprotected drive through its own alignment history (feed-forward).
    delay_in: Delay,
    acc_in: dsp::LaneAcc,
    sense: Option<(usize, usize)>,
    vi_scale: f64,
    acc_vi: dsp::LaneAcc,
    cl: Option<Cl>,
    // published state
    p_model: f64,
    g_ff: f64,
    g_hard: f64,
    armed: bool,
    vi_power: f64,
}

#[derive(Clone, Copy)]
struct Group {
    /// Smoothed feed-forward gain.
    g: f64,
    /// Selected target and the two dezipper stages.
    sel: f32,
    z1: f32,
    z2: f32,
}

struct Run {
    n: usize,
    look: usize,
    ambient: f64,
    k_attack: f64,
    k_release: f64,
    k_dez: f32,
    ch: Vec<Chan>,
    groups: Vec<Group>,
    /// Per group, the dezippered gain of the current segment's frames.
    curve: Vec<f32>,
    pos: usize,
    frame: u64,
    last_frame: u64,
    sums: BlockSums,
    sense_ok: bool,
    estimator: DelayEstimator,
}

pub struct Unit {
    params: Params,
    /// `None` when the parameters could not be used: the unit is then the
    /// fail-safe.
    run: Option<Run>,
}

impl Unit {
    /// Build the unit. Allocation happens here, never in `process`.
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
    /// In-phase V/I power of unit channel `c` over the last update, W (NaN
    /// without complete sense). For the supervisor's plausibility checks.
    pub fn sense_power(&self, c: usize) -> f64 {
        self.run.as_ref().and_then(|r| r.ch.get(c)).map_or(f64::NAN, |c| c.vi_power)
    }
    /// The model-alignment history, frames.
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
        let dt = THERMAL_FRAMES as f64 / rate;
        let r = rate / THERMAL_FRAMES as f64;
        let look = libm::round(p.thermal_lookahead_delay_ms as f64 * 1e-3 * rate) as usize;
        let ambient = p.thermal_ambient_temp as f64;
        let mut ids = [u32::MAX; MAX_CHANNELS];
        let mut ngroups = 0;
        let mut ch = Vec::with_capacity(n);
        for c in &p.ch[..n] {
            let gi = match ids[..ngroups].iter().position(|&g| g == c.group) {
                Some(i) => i,
                None => {
                    ids[ngroups] = c.group;
                    ngroups += 1;
                    ngroups - 1
                }
            };
            let (reb, series, ag) = (c.reb_ref as f64, (c.rshunt + c.rampout) as f64, c.ag as f64);
            let ff_factor = c.thermal_ff_speedup_factor as f64;
            // SPEC-SPP3 section 10: the feed-forward node starts equal to the
            // coil node (zero cold, `theta_vc0` hot).
            let ff = (ff_factor > 0.0)
                .then(|| Thermal::new(c.t_sett_vc as f64, ff_factor * c.tau_tvc as f64, dt, c.initial_theta_vc as f64));
            ch.push(Chan {
                src: c.source as usize,
                group: gi,
                headroom: if c.pilot_tone_enabled != 0 {
                    (1.0 - libm::pow(10.0, c.pilot_ampl_hi_db as f64 / 20.0)) as f32
                } else {
                    1.0
                },
                delay: Delay::new(look),
                scratch: vec![0.0; MODEL_SCRATCH],
                elec: ag * ag * reb / ((reb + series) * (reb + series)),
                limit: c.temperature_limit as f64,
                hard_on: c.temperature_limit as f64 + c.hard_temp_limit_headroom as f64,
                vc: Thermal::new(c.t_sett_vc as f64, c.tau_tvc as f64, dt, c.initial_theta_vc as f64),
                mg: Thermal::new(c.t_sett_mg as f64, c.tau_tmg as f64, dt, c.initial_theta_mg as f64),
                ff,
                init_vc: c.initial_theta_vc as f64,
                init_mg: c.initial_theta_mg as f64,
                p_prev: 0.0,
                p_ff_prev: 0.0,
                acc: dsp::LaneAcc::new(),
                delay_in: Delay::new(look),
                acc_in: dsp::LaneAcc::new(),
                sense: if c.ignore_telemetry != 0 { None } else { Some((c.sense_i as usize, c.sense_v as usize)) },
                vi_scale: c.imon_scale as f64 * c.vmon_scale as f64,
                acc_vi: dsp::LaneAcc::new(),
                cl: Cl::new(c, rate, r),
                p_model: 0.0,
                g_ff: 1.0,
                g_hard: 1.0,
                armed: false,
                vi_power: f64::NAN,
            });
        }
        let mut sums = BlockSums::default();
        sums.clear();
        Ok(Run {
            n,
            look,
            ambient,
            k_attack: settle99(p.thermal_gain_attack_time_s as f64, r),
            k_release: settle99(p.thermal_gain_release_time_s as f64, r),
            k_dez: settle99(DEZIPPER_S, rate) as f32,
            ch,
            groups: vec![Group { g: 1.0, sel: 1.0, z1: 1.0, z2: 1.0 }; ngroups],
            curve: vec![1.0; ngroups * THERMAL_FRAMES],
            pos: 0,
            frame: 0,
            last_frame: 0,
            sums,
            sense_ok: false,
            estimator: DelayEstimator::new(),
        })
    }

    fn reset(&mut self) {
        for c in self.ch.iter_mut() {
            c.delay.clear();
            c.vc.theta = c.init_vc;
            c.mg.theta = c.init_mg;
            if let Some(f) = c.ff.as_mut() {
                f.theta = c.init_vc;
            }
            c.p_prev = 0.0;
            c.p_ff_prev = 0.0;
            c.acc.clear();
            c.delay_in.clear();
            c.acc_in.clear();
            c.acc_vi.clear();
            if let Some(cl) = c.cl.as_mut() {
                cl.reset(c.limit);
            }
            c.p_model = 0.0;
            c.g_ff = 1.0;
            c.g_hard = 1.0;
            c.armed = false;
            c.vi_power = f64::NAN;
        }
        for g in self.groups.iter_mut() {
            *g = Group { g: 1.0, sel: 1.0, z1: 1.0, z2: 1.0 };
        }
        self.pos = 0;
        self.frame = 0;
        self.last_frame = 0;
        self.sums.clear();
        self.sense_ok = false;
        self.estimator.reset();
    }

    /// Frames `off..off+len` of the host buffers, inside one thermal update.
    fn segment(&mut self, io: &mut Io<'_, '_>, off: usize, len: usize, health: &mut u32) {
        let phase = self.pos;
        let k = self.k_dez;
        // per group: the two cascaded per-sample dezipper stages
        for (gi, g) in self.groups.iter_mut().enumerate() {
            let curve = &mut self.curve[gi * THERMAL_FRAMES..][..len];
            for v in curve.iter_mut() {
                g.z1 += k * (g.sel - g.z1);
                g.z2 += k * (g.z1 - g.z2);
                *v = g.z2;
            }
        }
        for c in self.ch.iter_mut() {
            let inp = &io.input[c.src][off..off + len];
            let out = &mut io.output[c.src][off..off + len];
            let curve = &self.curve[c.group * THERMAL_FRAMES..][..len];
            dsp::mul_to(inp, curve, out);
            if c.headroom != 1.0 {
                dsp::scale_in_place(out, c.headroom);
            }
            if dsp::peak_abs(out) == f32::INFINITY {
                // simple-meter failure: at most 20 dB below the input on
                // these frames, non-finite samples muted
                for ((o, &x), &g) in out.iter_mut().zip(inp).zip(curve) {
                    *o = if x.is_finite() { x * (g * c.headroom).min(FAILSAFE_GAIN) } else { 0.0 };
                }
                *health |= Health::FAILSAFE;
            }
            // model input: the protected drive through the alignment history
            let mut m0 = 0;
            while m0 < len {
                let m = (len - m0).min(c.scratch.len());
                // SAFETY: `out[m0..m0+m]` and `scratch[..m]` are distinct and hold `m` samples.
                unsafe { c.delay.run(out.as_ptr().add(m0), c.scratch.as_mut_ptr(), m) };
                c.acc.add_sq(&c.scratch[..m], phase + m0);
                if FF_FROM_UNPROTECTED {
                    // SAFETY: as above, for the input.
                    unsafe { c.delay_in.run(inp.as_ptr().add(m0), c.scratch.as_mut_ptr(), m) };
                    dsp::sanitize(&mut c.scratch[..m]);
                    c.acc_in.add_sq(&c.scratch[..m], phase + m0);
                }
                m0 += m;
            }
        }
        // sense statistics over the same frames
        let sense = io.sense;
        if sense.is_empty() {
            self.sums.complete = false;
        }
        for c in self.ch.iter_mut() {
            let Some((si, sv)) = c.sense else { continue };
            match (sense.get(si), sense.get(sv)) {
                (Some(i), Some(v)) if i.len() >= off + len && v.len() >= off + len => {
                    let (i, v) = (&i[off..off + len], &v[off..off + len]);
                    c.acc_vi.add_prod(v, i, phase);
                    if let Some(cl) = c.cl.as_mut() {
                        cl.feed(i, v);
                    }
                }
                _ => self.sums.complete = false,
            }
        }
        self.sums.frames += len;
        if self.sums.complete {
            let mut kk = off;
            while kk < off + len {
                let m = self.estimator.room().min(off + len - kk);
                let mut oe = 0.0f64;
                let mut ve = 0.0f64;
                for c in self.ch.iter() {
                    oe += dsp::sum_sq(&io.output[c.src][kk..kk + m]);
                    if let Some((_, sv)) = c.sense {
                        ve += dsp::sum_sq(&sense[sv][kk..kk + m]);
                    }
                }
                self.estimator.push_sums(oe as f32, ve as f32, m);
                kk += m;
            }
        }
        self.frame += len as u64;
    }

    /// One thermal update (every `THERMAL_FRAMES`).
    fn update(&mut self) {
        let n = THERMAL_FRAMES as f64;
        let mut ff_t = [1.0f64; MAX_CHANNELS];
        let mut hard_t = [1.0f64; MAX_CHANNELS];
        for c in self.ch.iter_mut() {
            let p = c.elec * c.acc.total() / n;
            c.acc.clear();
            c.p_model = p;
            c.vc.step(p, c.p_prev);
            c.mg.step(p, c.p_prev);
            let p_ff = if FF_FROM_UNPROTECTED { c.elec * c.acc_in.total() / n } else { p };
            c.acc_in.clear();
            if let Some(f) = c.ff.as_mut() {
                f.step(p_ff, c.p_ff_prev);
            }
            c.p_prev = p;
            c.p_ff_prev = p_ff;
            let t_ol = self.ambient + c.vc.theta + c.mg.theta;
            // closed loop: windows completed since the last update move the
            // corrected limit; it is smoothed at the thermal cadence
            let limit = match c.cl.as_mut() {
                Some(cl) => cl.update(t_ol, c.limit),
                None => c.limit,
            };
            let num = limit - self.ambient - c.mg.theta;
            // TODO(SPEC): with the feed-forward filter disabled the hard
            // candidate takes the real coil as its virtual coil.
            let vcff = c.ff.map_or(c.vc.theta, |f| f.theta);
            let ratio = |num: f64| libm::sqrt((num / (vcff + PREDICTION_MARGIN)).clamp(RATIO_MIN, 1.0));
            c.g_ff = if c.ff.is_some() { ratio(num) } else { 1.0 };
            // the hard path follows the corrected limit (reference: a
            // closed-loop run holds T_OL far above limit + headroom without
            // the panic)
            if t_ol > c.hard_on + (limit - c.limit) {
                c.armed = true;
            } else if t_ol < limit {
                c.armed = false;
            }
            c.g_hard = if c.armed { ratio(num - HARD_TARGET_BELOW) } else { 1.0 };
            ff_t[c.group] = ff_t[c.group].min(c.g_ff);
            hard_t[c.group] = hard_t[c.group].min(c.g_hard);
        }
        for (gi, g) in self.groups.iter_mut().enumerate() {
            let t = ff_t[gi];
            let k = if t < g.g { self.k_attack } else { self.k_release };
            g.g += k * (t - g.g);
            let sel = g.g.min(hard_t[gi]);
            // The smoother's state is the selected gain: after the hard path
            // disarms, the gain releases from where the hard path left it
            // (no step back up). Black-box reference: thermal_ramp_1k.
            // TODO(SPEC): confirm with Daybreak (spec 6 states only the
            // min selection).
            g.g = sel;
            g.sel = sel as f32;
        }
        let complete = self.sums.complete && self.sums.frames == THERMAL_FRAMES;
        for c in self.ch.iter_mut() {
            c.vi_power = if c.sense.is_some() && complete && c.acc_vi.is_finite() {
                c.acc_vi.total() * c.vi_scale / n
            } else {
                f64::NAN
            };
            c.acc_vi.clear();
        }
        self.sense_ok = complete;
        self.estimator.end_block();
        self.sums.clear();
        self.last_frame = self.frame - THERMAL_FRAMES as u64;
    }
}

impl ProtectionUnit for Unit {
    fn kind(&self) -> UnitKind {
        UnitKind::Spp3
    }

    fn latency_frames(&self) -> u32 {
        // The look-ahead is the model-alignment history; audio is not delayed.
        0
    }

    fn process(&mut self, mut io: Io<'_, '_>) -> Health {
        let frames = io.frames;
        let Some(run) = self.run.as_mut() else {
            failsafe_io(&mut io);
            return Health(Health::FAILSAFE | Health::NO_PARAMS);
        };
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
            let len = (THERMAL_FRAMES - run.pos).min(frames - off);
            run.segment(&mut io, off, len, &mut health);
            off += len;
            run.pos += len;
            if run.pos == THERMAL_FRAMES {
                run.update();
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
        out.frame = run.last_frame;
        out.health = Health(if run.sense_ok { 0 } else { Health::OPEN_LOOP });
        for c in run.ch.iter() {
            let g = &run.groups[c.group];
            let t_ol = (run.ambient + c.vc.theta + c.mg.theta) as f32;
            let mut s = ChannelState::ABSENT;
            s.modeled_power_w = c.p_model as f32;
            // the closed loop is not run: the published meter reads 0 W
            s.measured_power_w = 0.0;
            s.coil_temp_c = t_ol;
            s.magnet_temp_c = (run.ambient + c.mg.theta) as f32;
            s.ol_temp_c = t_ol;
            match c.cl.as_ref() {
                Some(cl) => {
                    s.cl_temp_c = cl.t_cl as f32;
                    s.tlim_adj_c = (cl.limit_s - c.limit) as f32;
                    s.closed_loop = true;
                    s.sense_v_ms = cl.last_v_ms as f32;
                    s.sense_i_ms = cl.last_i_ms as f32;
                    s.sense_r_ohm = libm::sqrt(cl.last_v_ms / cl.last_i_ms) as f32;
                }
                None => {
                    s.cl_temp_c = T_CL_UNMEASURED;
                    s.tlim_adj_c = 0.0;
                    s.closed_loop = false;
                }
            }
            // axis 4 is temperature: the selected (pre-dezipper) target
            s.axis_gain[4] = g.sel;
            s.gain = g.z2;
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
