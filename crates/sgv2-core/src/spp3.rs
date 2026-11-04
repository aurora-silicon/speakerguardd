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
    /// `PilotToneEnabled` (part 21, 0 or 1) and the pilot dictionary:
    /// `PilotAmplHi_dB`/`PilotAmplLo_dB` (dB), `PilotUpperThres`/
    /// `PilotLowerThres` (C, on `T_OL`), `PilotDecayTime`/
    /// `PilotDecayTimeStage2` (s, 99 % settling), `PilotFreq` (Hz) and
    /// `PilotEnableThres` (W, on the pre-gain drive power).
    pub pilot_tone_enabled: u32,
    pub pilot_ampl_hi_db: f32,
    pub pilot_ampl_lo_db: f32,
    pub pilot_upper_thres: f32,
    pub pilot_lower_thres: f32,
    pub pilot_decay_time_s: f32,
    pub pilot_decay_time_stage2_s: f32,
    pub pilot_freq: f32,
    pub pilot_enable_thres: f32,
    /// `ThermalMeasurementMethod`: 0 none, 1 pilot tone, 2 LP-MLS, 3 open
    /// loop.
    pub thermal_measurement_method: u32,
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
    /// Property 64007 `Thermal State Override` as the graph's property
    /// strip sets it, 0 when the strip leaves it unset (spec 10): 2 is the
    /// cold start at ambient, every other value the hot fallback.
    pub thermal_state_override: u32,
    pub ch: [ChannelParams; MAX_CHANNELS],
}

// SAFETY: repr(C), only u32/f32 fields and an array of a repr(C) struct of
// u32/f32 fields: 15 + 8 * 45 four-byte words, no padding.
unsafe impl Pod for Params {}
// SAFETY: as above, 45 four-byte words.
unsafe impl Pod for ChannelParams {}

const _: () = assert!(core::mem::size_of::<ChannelParams>() == 45 * 4);
const _: () = assert!(core::mem::size_of::<Params>() == (15 + MAX_CHANNELS * 45) * 4);

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
            thermal_state_override: 0,
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
            // Essential: the reader must say 0 when the strip leaves 64007
            // unset, so a reader that does not know the key cannot silently
            // select the hot start.
            thermal_state_override: g("ThermalStateOverride")? as u32,
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
                pilot_tone_enabled: (f("PilotToneEnabled")? != 0.0) as u32,
                pilot_ampl_hi_db: f("PilotAmplHi_dB")?,
                pilot_ampl_lo_db: f("PilotAmplLo_dB")?,
                pilot_upper_thres: f("PilotUpperThres")?,
                pilot_lower_thres: f("PilotLowerThres")?,
                pilot_decay_time_s: f("PilotDecayTime")?,
                pilot_decay_time_stage2_s: f("PilotDecayTimeStage2")?,
                pilot_freq: f("PilotFreq")?,
                pilot_enable_thres: f("PilotEnableThres")?,
                thermal_measurement_method: f("ThermalMeasurementMethod")? as u32,
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
        if !(self.thermal_safe_tlim.is_finite()
            && self.thermal_safe_tlim_offset.is_finite()
            && self.thermal_safe_tlim_time_min.is_finite()
            && self.thermal_safe_tlim_time_min >= 0.0)
        {
            return Err(InitError::Invalid("SafeTlim"));
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
            if !(c.rampout.is_finite() && c.rampout >= 0.0) {
                return Err(InitError::Invalid("Rampout"));
            }
            if c.pilot_tone_enabled != 0 {
                if !(c.pilot_ampl_hi_db.is_finite() && c.pilot_ampl_hi_db <= 0.0) {
                    return Err(InitError::Invalid("PilotAmplHi_dB"));
                }
                if !(c.pilot_ampl_lo_db.is_finite() && c.pilot_ampl_lo_db <= c.pilot_ampl_hi_db) {
                    return Err(InitError::Invalid("PilotAmplLo_dB"));
                }
                if !(c.pilot_upper_thres.is_finite()
                    && c.pilot_lower_thres.is_finite()
                    && c.pilot_enable_thres.is_finite()
                    && pos(c.pilot_decay_time_s)
                    && pos(c.pilot_decay_time_stage2_s)
                    && pos(c.pilot_freq)
                    && c.pilot_freq < self.sample_rate as f32 / 2.0)
                {
                    return Err(InitError::Invalid("pilot"));
                }
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
/// Clamp of the candidate ratio before the square root (spec 6, 16): a
/// 0.01 (-40 dB) linear-gain floor.
const RATIO_MIN: f64 = 1e-4;
/// Dezipper 99 % settling time, s (spec 6).
const DEZIPPER_S: f64 = 0.005;
const MODEL_SCRATCH: usize = 256;
/// `ThermalMeasurementMethod` of the pilot-tone sensor (spec 7).
pub const METHOD_PILOT_TONE: u32 = 1;
/// Property 64007 value of the cold start (spec 10).
pub const THERMAL_STATE_COLD: u32 = 2;
/// TODO(SPEC): which program drives `Pdrv` (the feed-forward node and the
/// pilot content gate). Spec 4/16 say the headroom-reduced program
/// `(1 - A_hi) * u`; the `spp3-cl-edge-pilot-dc-ulp` golden logs
/// `Pdrv = (ag*x)^2 Reb/(Reb+Rshunt+Rampout)^2` for the raw DC level `x`
/// (0.0299999776 W at the level made for 0.03 W without the headroom), and
/// gates the pilot on at exactly that level. The golden wins until the text
/// is reconciled.
const PDRV_WITH_HEADROOM: bool = false;

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
    fn new(r_th: f64, tau: f64, dt: f64) -> Self {
        Thermal { a: 1.0 - dt / tau, b: 0.5 * r_th * dt / tau, theta: 0.0 }
    }
    #[inline]
    fn step(&mut self, p: f64, p_prev: f64) {
        self.theta = self.a * self.theta + self.b * (p + p_prev);
    }
}

/// `ceil(SafeTlimTimeMin * 60 * fs / thermal_period)` in the reference's
/// Float32 multiply/divide order (spec 6), or `None` past `u32`.
pub fn safe_count_init(time_min: f32, sample_rate: u32) -> Option<u32> {
    let n = time_min * 60.0 * sample_rate as f32 / THERMAL_FRAMES as f32;
    let n = libm::ceilf(n);
    (n.is_finite() && n >= 0.0 && n <= u32::MAX as f32).then_some(n as u32)
}

/// The per-channel SafeTlim state machine (spec 6): a one-way change of the
/// base limit from `TemperatureLimit` to `SafeTlim` after `init`
/// consecutive controller updates with `T_OL > Tlim_eff + SafeTlimOffset`.
#[derive(Clone, Copy)]
struct SafeTlim {
    normal: f64,
    safe: f64,
    offset: f32,
    init: u32,
    count: u32,
    latched: bool,
}

impl SafeTlim {
    fn new(normal: f32, safe: f32, offset: f32, init: u32) -> Self {
        SafeTlim {
            normal: normal as f64,
            // setup clamps: SafeTlim never above the ordinary limit, a
            // positive offset becomes zero (negative ones are kept)
            safe: safe.min(normal) as f64,
            offset: offset.min(0.0),
            init,
            // the zero seed: the first update takes the reload branch
            count: 0,
            latched: false,
        }
    }
    fn base(&self) -> f64 {
        if self.latched {
            self.safe
        } else {
            self.normal
        }
    }
    /// After this update's candidates; the new base applies from the next
    /// update.
    fn update(&mut self, t_ol: f64, tlim_eff: f64) {
        // Float32 comparison, equality (and NaN) reload
        if (t_ol as f32) > (tlim_eff as f32) + self.offset {
            self.count = self.count.wrapping_sub(1);
        } else {
            self.count = self.init;
        }
        if self.count == 0 {
            self.latched = true;
        }
    }
    /// Kernel construction (and the trait's stream-start reset).
    fn reconstruct(&mut self) {
        self.count = 0;
        self.latched = false;
    }
}

/// Single-bin Goertzel state of one signal (spec 7).
#[derive(Clone, Copy, Default)]
struct Goertzel {
    s1: f64,
    s2: f64,
}

impl Goertzel {
    #[inline]
    fn push(&mut self, x: f64, a: f64) {
        let s0 = x + a * self.s1 - self.s2;
        self.s2 = self.s1;
        self.s1 = s0;
    }
    /// `(s1^2 + s2^2 - a s1 s2) / N^2`, then cleared for the next period.
    fn take(&mut self, a: f64, n: f64) -> f64 {
        let p = (self.s1 * self.s1 + self.s2 * self.s2 - a * self.s1 * self.s2) / (n * n);
        *self = Goertzel::default();
        p
    }
}

/// Closed-loop pilot-tone resistance thermometry and the corrected limit
/// (spec 7).
struct Cl {
    /// Goertzel period `N` and its coefficient `2 cos(2 pi m / N)`.
    window: usize,
    coef: f64,
    n: usize,
    gv: Goertzel,
    gi: Goertzel,
    v_scale2: f64,
    i_scale2: f64,
    v_floor: f64,
    i_floor: f64,
    k_r: f64,
    r_s: f64,
    rdc_scale: f64,
    reb_min: f64,
    reb_max: f64,
    cal_r: f64,
    cal_t: f64,
    inv_temp_co: f64,
    cap: f64,
    k_lim: f64,
    /// The latest completed window passed every gate.
    valid: bool,
    /// Latest window resistance (telemetry).
    r_window: f64,
    t_cl: f64,
    /// Smoothed corrected limit, absolute C (`Tlim_eff`).
    limit_s: f64,
}

/// The calibration record of a channel lies inside its ranges (spec 7).
fn cal_in_range(c: &ChannelParams) -> bool {
    let (r, t) = (c.cal_r, c.cal_t);
    c.cal_present != 0
        && r.is_finite()
        && t.is_finite()
        && r > 0.0
        && t >= c.cal_temp_range_min
        && t <= c.cal_temp_range_max
        && r >= c.reb_range_min
        && r <= c.reb_range_max
}

impl Cl {
    /// `None` (open-loop kernel) unless the channel is configured for
    /// pilot-tone closed loop. The caller has already checked the unit's
    /// calibration vector.
    fn new(c: &ChannelParams, rate: f64, r: f64) -> Option<Cl> {
        if c.cl_thermal_feedback_enabled == 0
            || c.thermal_measurement_method != METHOD_PILOT_TONE
            || c.ignore_telemetry != 0
            || !cal_in_range(c)
        {
            return None;
        }
        let (rc, tc) = (c.cal_r as f64, c.cal_t as f64);
        let samples = libm::round(c.temp_sense_window_time_s as f64 * rate);
        if !(samples >= 1.0 && samples <= (1u32 << 20) as f64) {
            return None;
        }
        let window = (samples as usize).next_power_of_two();
        let m = libm::round(c.pilot_freq as f64 * window as f64 / rate);
        if !(c.temp_sense_smoothing_tau_s > 0.0 && c.tlim_err_decay_time_s > 0.0 && c.rdc_scale > 0.0) {
            return None;
        }
        let (a20, a35) = (c.a_t_20c as f64, c.a_t_35c as f64);
        let alpha = a20 + (tc - 20.0) * (a35 - a20) / 15.0;
        if !(alpha.is_finite() && alpha > 0.0) {
            return None;
        }
        let (vs, is) = (c.vmon_scale as f64, c.imon_scale as f64);
        let limit = c.temperature_limit as f64;
        Some(Cl {
            window,
            coef: 2.0 * libm::cos(2.0 * core::f64::consts::PI * m / window as f64),
            n: 0,
            gv: Goertzel::default(),
            gi: Goertzel::default(),
            v_scale2: vs * vs,
            i_scale2: is * is,
            v_floor: c.v_pwr_thres as f64,
            i_floor: c.i_pwr_thres as f64,
            // an ordinary e-folding constant per Goertzel period
            k_r: 1.0 - libm::exp(-(window as f64) / (rate * c.temp_sense_smoothing_tau_s as f64)),
            r_s: rc,
            rdc_scale: c.rdc_scale as f64,
            reb_min: c.reb_range_min as f64,
            reb_max: c.reb_range_max as f64,
            cal_r: rc,
            cal_t: tc,
            inv_temp_co: 1.0 / alpha,
            cap: c.tlim_adj_cap.max(0.0) as f64,
            k_lim: settle99(c.tlim_err_decay_time_s as f64, r),
            valid: false,
            r_window: rc,
            t_cl: tc,
            limit_s: limit,
        })
    }

    /// `AudioUnitReset` (spec 10): `T_CL = T_cal`, `R = R_cal`, zero
    /// adjustment, no accepted window until the next acquisition.
    fn reset(&mut self, base: f64) {
        self.n = 0;
        self.gv = Goertzel::default();
        self.gi = Goertzel::default();
        self.r_s = self.cal_r;
        self.r_window = self.cal_r;
        self.valid = false;
        self.t_cl = self.cal_t;
        self.limit_s = base;
    }

    /// Raw telemetry of one segment, frame-aligned with the program.
    fn feed(&mut self, i: &[f32], v: &[f32]) {
        let a = self.coef;
        for (&ii, &vv) in i.iter().zip(v) {
            self.gv.push(vv as f64, a);
            self.gi.push(ii as f64, a);
            self.n += 1;
            if self.n == self.window {
                self.n = 0;
                let nn = self.window as f64;
                let pv = self.gv.take(a, nn) * self.v_scale2;
                let pi = self.gi.take(a, nn) * self.i_scale2;
                self.window_done(pv, pi);
            }
        }
    }

    fn window_done(&mut self, pv: f64, pi: f64) {
        // the band-power gates run before the ratio (spec 16)
        if !(pv.is_finite() && pi.is_finite() && pv > self.v_floor && pi > self.i_floor) {
            self.valid = false;
            return;
        }
        let r = libm::sqrt(pv / pi);
        self.r_window = r;
        self.r_s += self.k_r * (r - self.r_s);
        let t = self.cal_t + (self.rdc_scale * self.r_s / self.cal_r - 1.0) * self.inv_temp_co;
        // a window outside the range still publishes its temperature; the
        // range test is on the instantaneous window resistance
        let re = self.rdc_scale * r;
        if t.is_finite() && self.r_s.is_finite() {
            self.t_cl = t;
            self.valid = re >= self.reb_min && re <= self.reb_max;
        } else {
            self.valid = false;
        }
    }

    /// Per thermal update: the smoothed corrected limit for `base`.
    fn update(&mut self, t_ol: f64, base: f64) -> f64 {
        let raw = if self.valid { base + (t_ol - self.t_cl).clamp(0.0, self.cap) } else { base };
        self.limit_s += self.k_lim * (raw - self.limit_s);
        self.limit_s
    }
}

/// The generated pilot tone of one channel (spec 7, pilot method).
struct Pilot {
    a_hi: f32,
    a_lo: f32,
    upper: f64,
    lower: f64,
    enable: f32,
    k1: f32,
    k2: f32,
    /// Phase increment and the running phase, radians.
    dphi: f64,
    phi: f64,
    /// Selected high/low amplitude (hysteresis on `T_OL`) and whether the
    /// content gate is on; the target is their product.
    high: bool,
    on: bool,
    z1: f32,
    z2: f32,
}

impl Pilot {
    fn new(c: &ChannelParams, rate: f64) -> Pilot {
        let a = |db: f32| libm::pow(10.0, db as f64 / 20.0) as f32;
        Pilot {
            a_hi: a(c.pilot_ampl_hi_db),
            a_lo: a(c.pilot_ampl_lo_db),
            upper: c.pilot_upper_thres as f64,
            lower: c.pilot_lower_thres as f64,
            enable: c.pilot_enable_thres,
            k1: settle99(c.pilot_decay_time_s as f64, rate) as f32,
            k2: settle99(c.pilot_decay_time_stage2_s as f64, rate) as f32,
            dphi: 2.0 * core::f64::consts::PI * c.pilot_freq as f64 / rate,
            phi: 0.0,
            high: false,
            on: false,
            z1: 0.0,
            z2: 0.0,
        }
    }
    fn reset(&mut self) {
        self.phi = 0.0;
        self.high = false;
        self.on = false;
        self.z1 = 0.0;
        self.z2 = 0.0;
    }
    /// Per thermal update, from that update's `Pdrv` and `T_OL`.
    fn update(&mut self, p_drv: f64, t_ol: f64) {
        self.on = (p_drv as f32) >= self.enable;
        if t_ol > self.upper {
            self.high = true;
        } else if t_ol < self.lower {
            self.high = false;
        }
    }
    /// Add the pilot to `out`. TODO(SPEC): the sine's phase origin, the
    /// generator frequency (`PilotFreq` here; the analyser uses its bin)
    /// and the exact form of the two "interpolation stages" (two cascaded
    /// per-sample 99 % one-poles here). The `spp3-cal-nominal` golden's
    /// onset is about 2.2x this model's `k1*k2*A` (see the B5 report).
    fn add(&mut self, out: &mut [f32]) {
        let target = if !self.on {
            0.0
        } else if self.high {
            self.a_hi
        } else {
            self.a_lo
        };
        for o in out.iter_mut() {
            self.z1 += self.k1 * (target - self.z1);
            self.z2 += self.k2 * (self.z1 - self.z2);
            *o += self.z2 * libm::sin(self.phi) as f32;
            self.phi += self.dphi;
            if self.phi >= core::f64::consts::TAU {
                self.phi -= core::f64::consts::TAU;
            }
        }
    }
}

struct Chan {
    src: usize,
    group: usize,
    headroom: f32,
    scratch: Vec<f32>,
    elec: f64,
    headroom2: f64,
    safe: SafeTlim,
    headroom_c: f64,
    vc: Thermal,
    mg: Thermal,
    ff: Option<Thermal>,
    /// Start rises: coil, magnet, feed-forward (spec 10).
    seed: (f64, f64, f64),
    p_prev: f64,
    p_ff_prev: f64,
    /// Post-gain coil drive (before the pilot) and pre-gain drive.
    acc: dsp::LaneAcc,
    acc_in: dsp::LaneAcc,
    sense: Option<(usize, usize)>,
    vi_scale: f64,
    acc_vi: dsp::LaneAcc,
    cl: Option<Cl>,
    pilot: Option<Pilot>,
    // published state
    p_model: f64,
    p_drv: f64,
    tlim_eff: f64,
    g_ff: f64,
    g_hard: f64,
    armed: bool,
    vi_power: f64,
}

impl Chan {
    /// Kernel construction state of the thermal model and controller.
    fn seed(&mut self) {
        self.vc.theta = self.seed.0;
        self.mg.theta = self.seed.1;
        if let Some(f) = self.ff.as_mut() {
            f.theta = self.seed.2;
        }
        self.p_prev = 0.0;
        self.p_ff_prev = 0.0;
        self.acc.clear();
        self.acc_in.clear();
        self.acc_vi.clear();
        self.safe.reconstruct();
        if let Some(cl) = self.cl.as_mut() {
            cl.reset(self.safe.base());
        }
        if let Some(p) = self.pilot.as_mut() {
            p.reset();
        }
        self.p_model = 0.0;
        self.p_drv = 0.0;
        self.tlim_eff = self.safe.base();
        self.g_ff = 1.0;
        self.g_hard = 1.0;
        self.armed = false;
        self.vi_power = f64::NAN;
    }
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

const GROUP_START: Group = Group { g: 1.0, sel: 1.0, z1: 1.0, z2: 1.0 };

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
    /// Unit channel `c` has latched its SafeTlim base limit.
    pub fn safe_tlim_latched(&self, c: usize) -> bool {
        self.run.as_ref().and_then(|r| r.ch.get(c)).is_some_and(|c| c.safe.latched)
    }
    /// The effective limit `Tlim_eff` unit channel `c` used in its last
    /// update, C.
    pub fn effective_limit(&self, c: usize) -> f64 {
        self.run.as_ref().and_then(|r| r.ch.get(c)).map_or(f64::NAN, |c| c.tlim_eff)
    }
    /// Pre-gain drive power `Pdrv` of unit channel `c` in its last update, W.
    pub fn drive_power(&self, c: usize) -> f64 {
        self.run.as_ref().and_then(|r| r.ch.get(c)).map_or(f64::NAN, |c| c.p_drv)
    }
    /// The pilot generator of unit channel `c`: `(on, high)` of its last
    /// update, `None` without a generator.
    pub fn pilot_state(&self, c: usize) -> Option<(bool, bool)> {
        self.run.as_ref().and_then(|r| r.ch.get(c)).and_then(|c| c.pilot.as_ref()).map(|p| (p.on, p.high))
    }
    /// Per-update controller state of unit channel `c`, in the order of
    /// the reference's `Reb_Data` record where it has the field:
    /// `[smoothed g_ff, selected target, Tlim_eff, theta_ff, theta_vc,
    /// theta_mg, Pcoil, Pdrv, T_CL, CL window accepted, pilot target
    /// amplitude]`. Diagnostics for the rig; NaN where absent.
    pub fn controller_state(&self, c: usize) -> [f64; 11] {
        let Some(r) = self.run.as_ref() else { return [f64::NAN; 11] };
        let Some(ch) = r.ch.get(c) else { return [f64::NAN; 11] };
        let g = &r.groups[ch.group];
        let pilot = ch.pilot.as_ref().map_or(f64::NAN, |p| {
            (if !p.on {
                0.0
            } else if p.high {
                p.a_hi
            } else {
                p.a_lo
            }) as f64
        });
        [
            ch.g_ff,
            g.g,
            ch.tlim_eff,
            ch.ff.map_or(f64::NAN, |f| f.theta),
            ch.vc.theta,
            ch.mg.theta,
            ch.p_model,
            ch.p_drv,
            ch.cl.as_ref().map_or(f64::NAN, |l| l.t_cl),
            ch.cl.as_ref().map_or(f64::NAN, |l| l.valid as u8 as f64),
            pilot,
        ]
    }
    /// Per-update controller state of unit channel `c`, in the order of
    /// the reference's `Reb_Data` record where it has the field:
    /// `[smoothed g_ff, selected target, Tlim_eff, theta_ff, theta_vc,
    /// theta_mg, Pcoil, Pdrv, T_CL, CL window accepted, pilot target
    /// amplitude]`. Diagnostics for the rig; NaN where absent.
    pub fn controller_state(&self, c: usize) -> [f64; 11] {
        let Some(r) = self.run.as_ref() else { return [f64::NAN; 11] };
        let Some(ch) = r.ch.get(c) else { return [f64::NAN; 11] };
        let g = &r.groups[ch.group];
        let pilot = ch.pilot.as_ref().map_or(f64::NAN, |p| {
            (if !p.on {
                0.0
            } else if p.high {
                p.a_hi
            } else {
                p.a_lo
            }) as f64
        });
        [
            ch.g_ff,
            g.g,
            ch.tlim_eff,
            ch.ff.map_or(f64::NAN, |f| f.theta),
            ch.vc.theta,
            ch.mg.theta,
            ch.p_model,
            ch.p_drv,
            ch.cl.as_ref().map_or(f64::NAN, |l| l.t_cl),
            ch.cl.as_ref().map_or(f64::NAN, |l| l.valid as u8 as f64),
            pilot,
        ]
    }
    /// `AudioUnitReset` (spec 10): clears only the closed-loop estimator.
    /// The open-loop coil, magnet and feed-forward temperatures, the gain
    /// state and the SafeTlim state continue; an open-loop kernel is
    /// unchanged. Wall-clock idle is never thermal time.
    pub fn audio_unit_reset(&mut self) {
        if let Some(r) = self.run.as_mut() {
            for c in r.ch.iter_mut() {
                let base = c.safe.base();
                if let Some(cl) = c.cl.as_mut() {
                    cl.reset(base);
                }
            }
        }
    }
}

impl Run {
    fn new(p: &Params) -> Result<Run, InitError> {
        p.validate()?;
        if p.enable_displacement_control != 0 {
            // TODO(SPEC): the predictor coefficient builder and the
            // feedback integrator (spec 8 gives their outputs, not the laws)
            return Err(InitError::Unsupported("EnableDisplacementControl"));
        }
        if p.enable_cpms_control != 0 {
            // TODO(SPEC): the 100 ms / 1000 ms / thermal power-state laws
            return Err(InitError::Unsupported("EnableCPMSControl"));
        }
        if p.enable_cpms_closed_loop_control != 0 {
            return Err(InitError::Unsupported("EnableCPMSClosedLoopControl"));
        }
        if p.thermal_protection_enabled == 0 {
            // Specified (spec 10: the model runs and only the thermal
            // candidate is gated out), but a preset that switches speaker
            // protection off is refused by policy: the fail-safe runs.
            return Err(InitError::Unsupported("thermal protection disabled"));
        }
        let safe_init = safe_count_init(p.thermal_safe_tlim_time_min, p.sample_rate)
            .ok_or(InitError::Invalid("SafeTlimTimeMin"))?;
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let dt = THERMAL_FRAMES as f64 / rate;
        let r = rate / THERMAL_FRAMES as f64;
        let look = libm::round(p.thermal_lookahead_delay_ms as f64 * 1e-3 * rate) as usize;
        let ambient = p.thermal_ambient_temp as f64;
        let cold = p.thermal_state_override == THERMAL_STATE_COLD;
        // The calibration vector: one record per kernel. Without it the
        // pilot generator is not configured; with any record outside its
        // ranges every channel runs the open-loop kernel (the rig's
        // cal-*-out-of-range goldens).
        let chans = &p.ch[..n];
        let cal_vector = chans.iter().all(|c| c.cal_present != 0);
        let cal_ok = cal_vector && chans.iter().all(cal_in_range);
        let mut ids = [u32::MAX; MAX_CHANNELS];
        let mut ngroups = 0;
        let mut ch = Vec::with_capacity(n);
        for c in chans {
            let gi = match ids[..ngroups].iter().position(|&g| g == c.group) {
                Some(i) => i,
                None => {
                    ids[ngroups] = c.group;
                    ngroups += 1;
                    ngroups - 1
                }
            };
            // the series resistance includes the amplifier output
            // resistance, fractional and unrounded (spec 12)
            let (reb, series, ag) = (c.reb_ref as f64, c.rshunt as f64 + c.rampout as f64, c.ag as f64);
            let ff_factor = c.thermal_ff_speedup_factor as f64;
            let ff = (ff_factor > 0.0).then(|| Thermal::new(c.t_sett_vc as f64, ff_factor * c.tau_tvc as f64, dt));
            let (tvc, tmg) = (c.t_sett_vc as f64, c.t_sett_mg as f64);
            // hot fallback: the steady-state split at the ordinary limit
            let seed = if cold {
                (0.0, 0.0, 0.0)
            } else {
                let p0 = (c.temperature_limit as f64 - ambient) / (tvc + tmg);
                (p0 * tvc, p0 * tmg, p0 * tvc)
            };
            let headroom = if c.pilot_tone_enabled != 0 {
                (1.0 - libm::pow(10.0, c.pilot_ampl_hi_db as f64 / 20.0)) as f32
            } else {
                1.0
            };
            let pilot_cfg = c.pilot_tone_enabled != 0
                && c.cl_thermal_feedback_enabled != 0
                && c.thermal_measurement_method == METHOD_PILOT_TONE
                && cal_vector;
            let mut chan = Chan {
                src: c.source as usize,
                group: gi,
                headroom,
                scratch: vec![0.0; MODEL_SCRATCH],
                elec: ag * ag * reb / ((reb + series) * (reb + series)),
                headroom2: (headroom as f64) * (headroom as f64),
                safe: SafeTlim::new(c.temperature_limit, p.thermal_safe_tlim, p.thermal_safe_tlim_offset, safe_init),
                headroom_c: c.hard_temp_limit_headroom as f64,
                vc: Thermal::new(tvc, c.tau_tvc as f64, dt),
                mg: Thermal::new(tmg, c.tau_tmg as f64, dt),
                ff,
                seed,
                p_prev: 0.0,
                p_ff_prev: 0.0,
                acc: dsp::LaneAcc::new(),
                acc_in: dsp::LaneAcc::new(),
                sense: if c.ignore_telemetry != 0 { None } else { Some((c.sense_i as usize, c.sense_v as usize)) },
                vi_scale: c.imon_scale as f64 * c.vmon_scale as f64,
                acc_vi: dsp::LaneAcc::new(),
                cl: if cal_ok { Cl::new(c, rate, r) } else { None },
                pilot: pilot_cfg.then(|| Pilot::new(c, rate)),
                p_model: 0.0,
                p_drv: 0.0,
                tlim_eff: c.temperature_limit as f64,
                g_ff: 1.0,
                g_hard: 1.0,
                armed: false,
                vi_power: f64::NAN,
            };
            chan.seed();
            ch.push(chan);
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
            groups: vec![GROUP_START; ngroups],
            curve: vec![1.0; ngroups * THERMAL_FRAMES],
            pos: 0,
            frame: 0,
            last_frame: 0,
            sums,
            sense_ok: false,
            estimator: DelayEstimator::new(),
        })
    }

    /// Kernel reconstruction: the configured start state (64007), SafeTlim
    /// NORMAL, unity gains, estimator and pilot cleared.
    fn reset(&mut self) {
        for c in self.ch.iter_mut() {
            c.seed();
        }
        for g in self.groups.iter_mut() {
            *g = GROUP_START;
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
            // Pcoil: the protected, headroom-reduced drive before the pilot,
            // on the update's own frames (spec 4, 16)
            c.acc.add_sq(out, phase);
            // Pdrv: the unprotected program on the same frames
            let mut m0 = 0;
            while m0 < len {
                let m = (len - m0).min(c.scratch.len());
                c.scratch[..m].copy_from_slice(&inp[m0..m0 + m]);
                dsp::sanitize(&mut c.scratch[..m]);
                c.acc_in.add_sq(&c.scratch[..m], phase + m0);
                m0 += m;
            }
            if let Some(p) = c.pilot.as_mut() {
                p.add(out);
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
            let mut p_ff = c.elec * c.acc_in.total() / n;
            if PDRV_WITH_HEADROOM {
                p_ff *= c.headroom2;
            }
            c.acc_in.clear();
            c.p_drv = p_ff;
            if let Some(f) = c.ff.as_mut() {
                f.step(p_ff, c.p_ff_prev);
            }
            c.p_prev = p;
            c.p_ff_prev = p_ff;
            let t_ol = self.ambient + c.vc.theta + c.mg.theta;
            // the base limit (TemperatureLimit, or SafeTlim once latched)
            // and the closed-loop correction smoothed around it
            let base = c.safe.base();
            let limit = match c.cl.as_mut() {
                Some(cl) => cl.update(t_ol, base),
                None => base,
            };
            c.tlim_eff = limit;
            let num = limit - self.ambient - c.mg.theta;
            // TODO(SPEC): with the feed-forward filter disabled the hard
            // candidate takes the real coil as its virtual coil.
            let vcff = c.ff.map_or(c.vc.theta, |f| f.theta);
            let ratio = |num: f64| libm::sqrt((num / (vcff + PREDICTION_MARGIN)).clamp(RATIO_MIN, 1.0));
            c.g_ff = if c.ff.is_some() { ratio(num) } else { 1.0 };
            // the hard path follows the effective limit
            if t_ol > limit + c.headroom_c {
                c.armed = true;
            } else if t_ol < limit {
                c.armed = false;
            }
            c.g_hard = if c.armed { ratio(num - HARD_TARGET_BELOW) } else { 1.0 };
            ff_t[c.group] = ff_t[c.group].min(c.g_ff);
            hard_t[c.group] = hard_t[c.group].min(c.g_hard);
            // then SafeTlim, for the next update
            c.safe.update(t_ol, limit);
            if let Some(pl) = c.pilot.as_mut() {
                pl.update(p_ff, t_ol);
            }
        }
        for (gi, g) in self.groups.iter_mut().enumerate() {
            let t = ff_t[gi];
            let k = if t < g.g { self.k_attack } else { self.k_release };
            g.g += k * (t - g.g);
            // the smoother's state is the selected minimum (spec 6)
            let sel = g.g.min(hard_t[gi]);
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
                    s.tlim_adj_c = (cl.limit_s - c.safe.base()) as f32;
                    s.closed_loop = true;
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

    fn host_reset(&mut self) {
        self.audio_unit_reset();
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
