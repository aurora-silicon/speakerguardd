//! `spp3`: feed-forward thermal protection per channel with linked speaker
//! groups, optional pilot-tone closed loop, and the SafeTlim latch, from the
//! behaviour specification (SPEC-SPP3 sections 3 to 7, 10, 12, 16).
//!
//! - **Cadence.** The thermal model and controller update every 256 frames.
//!   Hosts may call `process` with any frame count. As in the reference,
//!   the output depends on the host slice: each call starts its dezipper at
//!   the selected target, and the closed loop takes a whole render block's
//!   telemetry ahead when the call holds it.
//! - **Audio.** Not delayed (latency 0). `out = in * headroom * g[n] +
//!   pilot[n]`, `headroom = 1 - 10^(PilotAmplHi_dB/20)` on channels with
//!   `PilotToneEnabled` (else 1), `g[n]` the group's dezippered gain.
//! - **Powers.** On each update's own 256 frames (no history shift):
//!   `Pcoil = elec * mean(out^2)` drives the coil and magnet, `Pdrv = elec *
//!   mean(in^2)` drives the feed-forward node and the pilot gate, with
//!   `elec = ag^2 Reb / (Reb + Rshunt + Rampout)^2` (Rampout fractional,
//!   unrounded). See `PDRV_WITH_HEADROOM` and `PCOIL_WITH_PILOT`.
//! - **Thermal filters.** Trapezoidal-input, linear-decay recurrences at
//!   `dt = 256/fs` for the coil, the magnet and the feed-forward node
//!   (`tau_ff = ThermalFFSpeedupFactor * tau_Tvc`). `T_OL = ambient +
//!   theta_vc + theta_mg`.
//! - **Start state (64007).** 2: every node at zero (ambient). Any other
//!   value, including unset (0): the hot fallback, the steady-state split at
//!   the ordinary limit (`theta_ff = theta_vc`).
//! - **Candidates.** `g_ff = sqrt(clamp((Tlim_eff - ambient - theta_mg) /
//!   (theta_ff + 1 C), 1e-4, 1))`, smoothed per update with the 99 % law
//!   (attack 4.6 s / release 20 s on AID35) from the previous selected
//!   minimum, through a 1 s release hold of the candidate (rig law 5).
//!   The hard path arms when `T_OL > Tlim_eff + headroom`, disarms
//!   when `T_OL < Tlim_eff`, and while armed offers the same ratio 5 C lower.
//!   The group takes the minimum; two 5 ms dezipper stages follow.
//! - **SafeTlim.** Per channel: after `ceil(TimeMin * 60 * fs / 256)`
//!   consecutive updates with `T_OL > Tlim_eff + SafeTlimOffset` (Float32,
//!   strict) the base limit becomes `SafeTlim` from the next update, with no
//!   runtime exit. The base feeds the candidates, the hard thresholds and the
//!   closed-loop correction.
//! - **Closed loop (method 1, pilot tone).** One accepted calibration record
//!   per kernel inside the ranges, else every channel is open loop. Per
//!   Goertzel period `N = next_pow2(round(TempSenseWindowTime fs))` on bin
//!   `m = round(PilotFreq N / fs)`: band-power gates on scaled V and I, then
//!   `R = sqrt(Pv / Pi)`, `R_s` e-folding from `R_cal`, `T_CL` from `R_s`
//!   with `alpha` interpolated at `T_cal`; accepted when the smoothed
//!   `R` is in `RebRange`. `Tlim_eff` smooths `base + clamp(T_OL - T_CL, 0,
//!   cap)` (or `base` when the last window was rejected) over
//!   `TlimErrDecayTime` with the 99 % law.
//!   TODO(SPEC) safety: the relaxation is capped at the base (`CL_RELAXATION`)
//!   after the J616s live-test incident; CL never raises the limit.
//!   Beyond the reference, a plausibility guard (stricter only) stops a
//!   channel's relaxation for good when its resistance falls while the model
//!   heats, or it reads far below its calibration temperature under drive
//!   (`Health::SENSE_IMPLAUSIBLE`; see `Cl::guard`).
//! - **Pilot.** With a calibration vector: on in an update iff
//!   `f32(Pdrv) >= PilotEnableThres`; high above `PilotUpperThres`, low below
//!   `PilotLowerThres` on `T_OL`, held between.
//! - **Resets.** `reset` reconstructs the kernel (start state, SafeTlim
//!   NORMAL). `host_reset` is `AudioUnitReset`: only the closed-loop
//!   estimator returns to its calibration; thermal, gain, pilot and SafeTlim
//!   state continue.
//! - **Input clamp.** The program is held to +-1.5 FS (rig law (b)) before
//!   the gain and before both model meters; in-range audio is unchanged.
//!   The output samples are then held to +-1 FS after metering (policy).
//! - **Fail-safe.** A channel-count mismatch, or non-finite drive (the
//!   simple-meter failure), gives 20 dB for the affected frames. A model
//!   state that goes non-finite latches the 20 dB fail-safe
//!   (`Health::MODEL_FAULT`) until reset; candidate selection propagates
//!   NaN, so it can never fall back to unity.
//! - **Hand-off.** `save_state`/`restore_state` carry the coil, magnet and
//!   virtual-coil nodes, the hard-path arm flag and the selected gain to the
//!   next stream or instance (spec 10's state vector): each node aged by
//!   `exp(-gap/tau)`, the gain advanced toward unity with `k_release`, the
//!   arm flag kept only while the aged `T_OL` is still at or above the
//!   effective limit. The closed loop restarts invalid and SafeTlim restarts
//!   NORMAL, as the reference's rebuild does (rig law 6: the latch survives
//!   only AudioUnitReset). Policy divergence: the reference restores the
//!   record only under 64007 = 0 and cold-seeds under 2; the hand-off is
//!   restored for every start state here, which never runs cooler.
//! - **No feed-forward** (`ThermalFFSpeedupFactor <= 0`): the ordinary
//!   candidate is unity. The reference leaves such a channel unprotected
//!   (rig law 6: its FF state is NaN, the thermal state is re-seeded hot
//!   every update so T_OL sits at the limit, and a NaN hard target means
//!   unity). By policy this engine does not reproduce that: while armed the
//!   hard path holds the fixed 20 dB fail-safe gain, which holds the limit.
//!   No shipped preset sets a nonpositive factor.
//!
//! A preset that enables displacement control or CPMS (TODO(SPEC): the
//! predictor coefficient builder, the feedback integrator and the CPMS
//! power-state laws are given by their outputs, not their equations), or
//! disables thermal protection (specified, refused by policy), runs the
//! fail-safe. Every shipped laptop preset disables displacement and CPMS and
//! enables thermal protection.
//!
//! Tuning keys ([`TuningLookup`]) are the sidecar names where the sidecar
//! has the value, otherwise the public AU parameter name with spaces
//! removed: `ThermalProtectionEnabled` (global 0), `EnableCPMSControl`
//! (18), `EnableCPMSClosedLoopControl` (19), `PilotToneEnabled` (part 21),
//! `CLThermalFeedbackEnabled` (part 22), `PilotAmplHi_dB` (part 27).
//! The global `ThermalStateOverride` is the raw 64007 value the strip sets,
//! 0 when unset; it is essential so a reader cannot drop it silently.

use alloc::vec;
use alloc::vec::Vec;

use crate::blob::{BlobError, Pod};
use crate::dsp;
use crate::sense::{BlockSums, DelayEstimator};
use crate::{
    gap_or_zero, release_toward_unity, ChannelState, HandOff, Health, Io, ProtectionUnit, Telemetry, UnitKind,
    FAILSAFE_GAIN, MAX_CHANNELS,
};

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
    /// `CLThermalFeedbackEnabled` (0 or 1).
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
        if self.sample_rate == 0 || thermal_frames(self.sample_rate) > MAX_UPDATE {
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
        // the reference's documented minimum for the thermal time constants
        // (its parameter info gives 0.1 s; the AU accepts less and then
        // overshoots far above the hard limit at cold start, rig finding
        // 15), and never below one thermal update (the forward recurrence
        // `1 - dt/tau` diverges there)
        let dt = thermal_frames(self.sample_rate) as f32 / self.sample_rate as f32;
        let tau_min = TAU_MIN_S.max(dt);
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
            if !(c.tau_tvc >= tau_min && c.tau_tmg >= tau_min) {
                return Err(InitError::Invalid("thermal time constant below 0.1 s"));
            }
            // nonpositive disables the feed-forward candidate (spec 6); NaN
            // or infinity is no setting at all
            if !c.thermal_ff_speedup_factor.is_finite() {
                return Err(InitError::Invalid("ThermalFFSpeedupFactor"));
            }
            if c.thermal_ff_speedup_factor > 0.0 && c.thermal_ff_speedup_factor * c.tau_tvc < tau_min {
                return Err(InitError::Invalid("feed-forward pole below 0.1 s"));
            }
            if !(c.rampout.is_finite() && c.rampout >= 0.0) {
                return Err(InitError::Invalid("Rampout"));
            }
            if c.pilot_tone_enabled != 0 {
                // any finite level: the amplitude is clamped to 0.3 (rig law
                // 4), and the low level may lie above the high one (the rig's
                // envelope cases)
                if !c.pilot_ampl_hi_db.is_finite() {
                    return Err(InitError::Invalid("PilotAmplHi_dB"));
                }
                if !c.pilot_ampl_lo_db.is_finite() {
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
            if c.cl_thermal_feedback_enabled != 0 {
                c.validate_closed_loop()?;
            }
        }
        self.validate_physical()
    }

    /// Physical ranges. A blob can be finite and well-formed and still
    /// describe a speaker that never heats or a limit no coil has, and then
    /// full scale passes at unity (`TemperatureLimit = f32::MAX`, a
    /// smoother that never moves, a kilohm of series resistance). Every
    /// bound is far outside any laptop speaker yet tight enough that the
    /// model always heats and the controller always acts:
    /// - ambient -40..100 C, limits -40..250 C (voice coils are rated well
    ///   below 250 C; the closed loop can add at most `TlimAdjCap`), hard
    ///   headroom within +-50 K;
    /// - gain attack and release at most 600 s, thermal time constants at
    ///   most 3600 s;
    /// - thermal resistances 0.01..1000 K/W for the coil, at most 1000 K/W
    ///   for the magnet;
    /// - `Reb_ref` 0.1..100 ohm and the series resistance (`Rshunt +
    ///   Rampout`) at most `Reb_ref`: a sense shunt and an amplifier output
    ///   are a small fraction of the coil, and with more than the coil in
    ///   series the modelled power `ag^2 Reb / (Reb + Rs)^2` collapses;
    /// - amplifier volts at full scale `ag` 0.1..1000 V.
    fn validate_physical(&self) -> Result<(), InitError> {
        let within = |v: f32, lo: f32, hi: f32| v.is_finite() && v >= lo && v <= hi;
        if !within(self.thermal_ambient_temp, -40.0, 100.0) {
            return Err(InitError::Invalid("ambient outside -40..100 C"));
        }
        if !(within(self.thermal_gain_attack_time_s, 0.0, 600.0)
            && within(self.thermal_gain_release_time_s, 0.0, 600.0))
        {
            return Err(InitError::Invalid("gain attack/release above 600 s"));
        }
        for c in &self.ch[..self.channels as usize] {
            if !within(c.temperature_limit, -40.0, 250.0) {
                return Err(InitError::Invalid("TemperatureLimit outside -40..250 C"));
            }
            if !within(c.hard_temp_limit_headroom, -50.0, 50.0) {
                return Err(InitError::Invalid("HardTempLimitHeadroom outside +-50 K"));
            }
            if !(within(c.tau_tvc, 0.0, 3600.0) && within(c.tau_tmg, 0.0, 3600.0)) {
                return Err(InitError::Invalid("thermal time constant above 3600 s"));
            }
            if !(within(c.t_sett_vc, 0.01, 1000.0) && within(c.t_sett_mg, 0.0, 1000.0)) {
                return Err(InitError::Invalid("thermal resistance outside 0.01..1000 K/W"));
            }
            if !within(c.reb_ref, 0.1, 100.0) || !within(c.rshunt + c.rampout, 0.0, c.reb_ref) {
                return Err(InitError::Invalid("Reb_ref outside 0.1..100 ohm or series resistance above it"));
            }
            if !within(c.ag, 0.1, 1000.0) {
                return Err(InitError::Invalid("ag outside 0.1..1000 V"));
            }
            if c.cl_thermal_feedback_enabled != 0 && c.tlim_adj_cap > 100.0 {
                return Err(InitError::Invalid("TlimAdjCap above 100 K"));
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

impl ChannelParams {
    /// The closed-loop dictionary, when CL feedback is configured: every
    /// field the window, the resistance smoother and the corrected limit
    /// use, so the limit can only ever move inside `[base, base + cap]`.
    /// (The calibration record itself is checked against these ranges when
    /// the unit is built; one outside them runs the unit open loop.)
    fn validate_closed_loop(&self) -> Result<(), InitError> {
        let pos = |v: f32| v.is_finite() && v > 0.0;
        let nonneg = |v: f32| v.is_finite() && v >= 0.0;
        if !(pos(self.temp_sense_window_time_s)
            && pos(self.temp_sense_smoothing_tau_s)
            && pos(self.tlim_err_decay_time_s)
            && self.temp_sense_window_time_s <= 2.0)
        {
            return Err(InitError::Invalid("closed-loop times"));
        }
        if !(pos(self.imon_scale) && pos(self.vmon_scale) && nonneg(self.i_pwr_thres) && nonneg(self.v_pwr_thres)) {
            return Err(InitError::Invalid("closed-loop sense scales"));
        }
        if !(pos(self.rdc_scale) && pos(self.reb_range_min) && self.reb_range_max.is_finite())
            || self.reb_range_min > self.reb_range_max
        {
            return Err(InitError::Invalid("RebRange"));
        }
        let (t0, t1) = (self.cal_temp_range_min, self.cal_temp_range_max);
        if !(t0.is_finite() && t1.is_finite() && t0 <= t1) {
            return Err(InitError::Invalid("CalTempRange"));
        }
        if !(self.a_t_20c.is_finite() && self.a_t_35c.is_finite()) {
            return Err(InitError::Invalid("temperature coefficients"));
        }
        if !nonneg(self.tlim_adj_cap) {
            return Err(InitError::Invalid("TlimAdjCap"));
        }
        Ok(())
    }
}

/// Thermal update cadence, frames (spec 6: 256 at 48 kHz).
pub const THERMAL_FRAMES: usize = 256;
/// Largest thermal update supported (192 kHz).
const MAX_UPDATE: usize = 1024;

/// The thermal update cadence at `sample_rate`, frames: 256 at 44.1 and
/// 48 kHz, 512 at 96 kHz (rig: `next_pow2(round(256 fs / 48000))`).
pub fn thermal_frames(sample_rate: u32) -> usize {
    let n = libm::round(THERMAL_FRAMES as f64 * sample_rate as f64 / 48_000.0).max(1.0) as usize;
    n.next_power_of_two()
}
/// Hard-coded prediction margin in the candidate, C (spec 6).
const PREDICTION_MARGIN: f64 = 1.0;
/// Hard-path target below the limit, C (spec 6).
const HARD_TARGET_BELOW: f64 = 5.0;
/// Clamp of the candidate ratio before the square root (spec 6, 16): a
/// 0.01 (-40 dB) linear-gain floor.
const RATIO_MIN: f64 = 1e-4;
/// The unit clamps its program input to +-1.5 FS before the gain and the
/// drive meter (rig law (b): Pdrv of a 2 FS sine is that of the clamped
/// sine; the output never exceeds 1.5 x the gain). The output stays bounded.
const INPUT_CLAMP: f32 = 1.5;
/// Shortest accepted thermal time constant (coil, magnet and the
/// feed-forward pole), s: the reference's documented parameter minimum.
pub const TAU_MIN_S: f32 = 0.1;
/// Feed-forward node rise beyond which the reference re-seeds its state
/// (rig law 6a), K.
const FF_RANGE_K: f64 = 1000.0;
/// Closed-loop plausibility guard (beyond the reference, stricter only):
/// driven windows in the resistance trend, the T_OL span they must cover,
/// K, and the coldness bound below min(T_cal, ambient), K, held for
/// `COLD_N` windows.
const TREND_N: usize = 8;
const TREND_SPAN_K: f64 = 10.0;
/// A trend trips only when R_s falls by at least this fraction of the
/// coil's honest rise per kelvin (alpha * R_cal), so a flat reading and its
/// Float32 noise never trip it.
const TREND_SLOPE_FRAC: f64 = 0.25;
const COLD_MARGIN_K: f64 = 10.0;
const COLD_N: u32 = 3;
/// Release hold of the ordinary candidate, s (rig law 5: fixed).
const HOLD_S: f64 = 1.0;
/// Dezipper 99 % settling time, s (spec 6).
const DEZIPPER_S: f64 = 0.005;
const MODEL_SCRATCH: usize = 256;
/// `ThermalMeasurementMethod` of the pilot-tone sensor (spec 7).
pub const METHOD_PILOT_TONE: u32 = 1;
/// Property 64007 value of the cold start (spec 10).
pub const THERMAL_STATE_COLD: u32 = 2;
/// `Pdrv` (the feed-forward node and the pilot content gate) is the raw
/// program, without the pilot headroom (rig law 1, against spec 4/16's
/// text; to 2.6e-7 relative over 180 s, open and closed loop).
const PDRV_WITH_HEADROOM: bool = false;
/// TODO(SPEC) safety, J616s live test 2026-11-05: with real music over full
/// scale (Tokyo Drifting at volume 0.99, about 36 W woofer drive, T_OL
/// 129 C) the accepted windows read the coil COOLER as it heated (T_CL 89 ->
/// 33 C, window R 3.90 -> 3.46 ohm) and the correction relaxed the limit by
/// up to +38.9 C. Until that is understood against the reference, the
/// closed loop runs and publishes T_CL but never raises the limit above the
/// base. (The CL goldens match bit-for-bit with relaxation on; they carry
/// no program energy near the pilot bin.)
const CL_RELAXATION: bool = false;
/// `Pcoil` is the power of the unit's output samples, generated pilot
/// included (rig law 2, against spec 4's text; to 3e-7 relative).
const PCOIL_WITH_PILOT: bool = true;

/// The reference's coefficient law: a one-pole that settles to 99 % in
/// `seconds` at `rate` updates per second.
pub fn settle99(seconds: f64, rate: f64) -> f64 {
    1.0 - libm::exp(libm::log(0.01) / (seconds * rate))
}

/// Trapezoidal-input, linear-decay thermal filter (spec 5).
#[derive(Clone, Copy, Default)]
struct Thermal {
    a: f32,
    b: f32,
    tau: f64,
    /// Float32-valued (kept in an f64 for the controller arithmetic).
    theta: f64,
}

impl Thermal {
    /// The reference's Float32 coefficients (rig: the controller record's
    /// nodes are bit-exact on 89-99.99 % of updates with these, against
    /// 0.1 % for a double-precision recurrence): `a = 1 - dt/tau`,
    /// `b = 0.5 * R * dt / tau`, left to right, `dt = frames / fs`.
    fn new(r_th: f32, tau: f32, dt: f32) -> Self {
        Thermal { a: 1.0 - dt / tau, b: 0.5 * r_th * dt / tau, tau: tau as f64, theta: 0.0 }
    }
    /// `theta = fmaf(a, theta, b * (P + P_prev))` in Float32.
    #[inline]
    fn step(&mut self, p: f64, p_prev: f64) {
        let sum = p as f32 + p_prev as f32;
        self.theta = libm::fmaf(self.a, self.theta as f32, self.b * sum) as f64;
    }
}

/// `ceil(SafeTlimTimeMin * 60 * fs / thermal_period)` in the reference's
/// Float32 multiply/divide order (spec 6), or `None` past `u32`.
pub fn safe_count_init(time_min: f32, sample_rate: u32) -> Option<u32> {
    let n = time_min * 60.0 * sample_rate as f32 / thermal_frames(sample_rate) as f32;
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
            // seeded full: the rig's hot rebuild (64007 = 1, predicate true
            // from its first update) latches one full count after the
            // rebuild, not never (a zero seed would wrap); a cold start
            // reloads on its first update either way
            count: init,
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
        // a zero count (TimeMin 0) latches on the first update either way
        if self.count == 0 || self.init == 0 {
            self.latched = true;
        }
    }
    /// Kernel construction (and the trait's stream-start reset).
    fn reconstruct(&mut self) {
        self.count = self.init;
        self.latched = false;
    }
}

/// Single-bin Goertzel state of one signal (spec 7).
#[derive(Clone, Copy, Default)]
struct Goertzel {
    s1: f32,
    s2: f32,
}

/// Single-bin Goertzel in Float32, as the reference accumulates it (rig
/// law (a): the Float32 recurrence over 16384 samples biases R by about
/// 1e-3 relative, and the goldens carry that bias).
impl Goertzel {
    #[inline]
    fn push(&mut self, x: f32, a: f32) {
        // fused a*s1 + x, then - s2 (rig law (a))
        let s0 = libm::fmaf(a, self.s1, x) - self.s2;
        self.s2 = self.s1;
        self.s1 = s0;
    }
    /// `(s1^2 + s2^2 - a s1 s2) / N^2`, then cleared for the next period.
    fn take(&mut self, a: f32, n: f32) -> f32 {
        // fmaf(-(a*s1), s2, fmaf(s1, s1, s2*s2)) / N^2 (rig law (a))
        let (s1, s2) = (self.s1, self.s2);
        let p = libm::fmaf(-(a * s1), s2, libm::fmaf(s1, s1, s2 * s2)) / (n * n);
        *self = Goertzel::default();
        p
    }
}

/// Closed-loop pilot-tone resistance thermometry and the corrected limit
/// (spec 7).
struct Cl {
    /// Thermal update, frames.
    upd: usize,
    /// Goertzel period `N` and its coefficient `2 cos(2 pi m / N)`.
    window: usize,
    coef: f32,
    n: usize,
    gv: Goertzel,
    gi: Goertzel,
    /// V and I scales (applied to each sample) and the band-power floors.
    v_scale: f32,
    i_scale: f32,
    v_floor: f32,
    i_floor: f32,
    /// Resistance smoother (e-folding per period) and its state.
    k_r: f32,
    r_s: f32,
    rdc_scale: f64,
    reb_min: f64,
    reb_max: f64,
    cal_r: f64,
    cal_t: f64,
    /// `invTempCo` and the series resistance, Float32 as the reference
    /// keeps them.
    inv_temp_co: f32,
    rshunt: f32,
    cap: f64,
    k_lim: f64,
    /// The latest completed window passed every gate.
    valid: bool,
    /// Frames since the last accepted window.
    age: usize,
    /// Latest window resistance (telemetry).
    r_window: f32,
    /// Diagnostics: the last period's pilot-band powers before the gates
    /// (NaN before the first). Bring-up observables only.
    last_pv: f32,
    last_pi: f32,
    t_cl: f64,
    /// Smoothed corrected limit, absolute C (`Tlim_eff` while active).
    limit_s: f64,
    /// A window has been accepted since construction or the last reset.
    active: bool,
    /// Plausibility guard (beyond the reference, stricter only): a window
    /// accepted since the last update, the recent driven windows' (T_OL,
    /// R_s) pairs, consecutive driven windows far below T_cal, and the
    /// sticky verdict.
    fresh: bool,
    drive_thres: f32,
    trend: [(f64, f64); TREND_N],
    trend_n: usize,
    cold_n: u32,
    cold_floor: f64,
    /// [`CL_RELAXATION`], except in an offline [`ClReplay`].
    relax: bool,
    implausible: bool,
}

/// The Goertzel period `N = next_power_of_two(round(TempSenseWindowTime *
/// fs))` (spec 7), at least 1.
fn pilot_period(c: &ChannelParams, rate: f64) -> usize {
    let samples = libm::round(c.temp_sense_window_time_s as f64 * rate);
    if samples >= 1.0 && samples <= (1u32 << 20) as f64 {
        (samples as usize).next_power_of_two()
    } else {
        1
    }
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
    fn new(c: &ChannelParams, rate: f64, r: f64, ambient: f64) -> Option<Cl> {
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
        // alpha interpolated at T_cal in double from the Float32 inputs,
        // its reciprocal stored as Float32 (spec 7; rig law 3)
        let (a20, a35) = (c.a_t_20c as f64, c.a_t_35c as f64);
        let alpha = a20 + (tc - 20.0) * (a35 - a20) / 15.0;
        if !(alpha.is_finite() && alpha > 0.0) {
            return None;
        }
        let inv_temp_co = (1.0 / alpha) as f32;
        // the temperature ratio is of the coil resistance net of Rshunt
        if !(inv_temp_co.is_finite() && c.cal_r - c.rshunt > 0.0) {
            return None;
        }
        let (vs, is) = (c.vmon_scale as f64, c.imon_scale as f64);
        let limit = c.temperature_limit as f64;
        Some(Cl {
            upd: thermal_frames(rate as u32),
            window,
            coef: (2.0 * libm::cos(2.0 * core::f64::consts::PI * m / window as f64)) as f32,
            n: 0,
            gv: Goertzel::default(),
            gi: Goertzel::default(),
            v_scale: vs as f32,
            i_scale: is as f32,
            v_floor: c.v_pwr_thres,
            i_floor: c.i_pwr_thres,
            // an ordinary e-folding constant per Goertzel period
            k_r: (1.0 - libm::exp(-(window as f64) / (rate * c.temp_sense_smoothing_tau_s as f64))) as f32,
            r_s: c.cal_r,
            rdc_scale: c.rdc_scale as f64,
            reb_min: c.reb_range_min as f64,
            reb_max: c.reb_range_max as f64,
            cal_r: rc,
            cal_t: tc,
            inv_temp_co,
            rshunt: c.rshunt,
            cap: c.tlim_adj_cap.max(0.0) as f64,
            k_lim: settle99(c.tlim_err_decay_time_s as f64, r),
            valid: false,
            age: 0,
            r_window: c.cal_r,
            last_pv: f32::NAN,
            last_pi: f32::NAN,
            t_cl: tc,
            limit_s: limit,
            active: false,
            fresh: false,
            drive_thres: c.pilot_enable_thres,
            trend: [(0.0, 0.0); TREND_N],
            trend_n: 0,
            cold_n: 0,
            // the coldest a coil can honestly read: below both its
            // calibration and the model's ambient by the margin
            cold_floor: tc.min(ambient) - COLD_MARGIN_K,
            relax: CL_RELAXATION,
            implausible: false,
        })
    }

    /// `AudioUnitReset` (spec 10): `T_CL = T_cal`, `R = R_cal`, zero
    /// adjustment, no accepted window until the next acquisition.
    fn reset(&mut self, base: f64) {
        self.n = 0;
        self.gv = Goertzel::default();
        self.gi = Goertzel::default();
        self.r_s = self.cal_r as f32;
        self.r_window = self.cal_r as f32;
        self.last_pv = f32::NAN;
        self.last_pi = f32::NAN;
        self.valid = false;
        self.age = 0;
        self.t_cl = self.t_of_rs();
        self.limit_s = base;
        self.active = false;
        self.fresh = false;
        self.trend_n = 0;
        self.cold_n = 0;
        self.implausible = false;
    }

    /// `AudioUnitReset`: the estimator restarts (the window grid from this
    /// frame, the partial period discarded, `R = R_cal`, `T_CL = T_cal`) and
    /// the loop is inactive until its next accepted window, the effective
    /// limit the base meanwhile. The corrected-limit smoother keeps its
    /// state and resumes from it on reacquisition (the
    /// `spp3-cl-edge-reset-midlimit` golden: 140 C while inactive, then
    /// 177.630 C at the first accepted window, which is the pre-reset
    /// smoother 177.612 C stepped once toward 200 C).
    fn host_reset(&mut self) {
        let (keep, verdict) = (self.limit_s, self.implausible);
        self.reset(keep);
        self.limit_s = keep;
        // the guard's verdict holds until the kernel is rebuilt
        self.implausible = verdict;
    }

    /// `T_CL` of the current smoothed resistance: `fmaf(invTempCo, (Rw -
    /// Rshunt) / (R_cal - Rshunt) - 1, T_cal)` in Float32 with `Rw =
    /// RdcScale * R_s`, the logged window resistance (rig law 3). Also the
    /// start value: with RdcScale != 1 the reference's T_CL starts at this,
    /// not at `T_cal` (rig, spp3-law-tcl-rdcscale: 3.63 ohm logged from
    /// block 0 at RdcScale 1.1).
    fn t_of_rs(&self) -> f64 {
        let rw = self.rdc_scale as f32 * self.r_s;
        let (rc, rs) = (self.cal_r as f32, self.rshunt);
        let x = (rw - rs) / (rc - rs) - 1.0;
        libm::fmaf(self.inv_temp_co, x, self.cal_t as f32) as f64
    }

    /// The sense bus could not feed a segment: the window is invalid and
    /// its partial state is discarded (spec 7: a missing bus invalidates
    /// the window).
    fn lose(&mut self) {
        self.n = 0;
        self.gv = Goertzel::default();
        self.gi = Goertzel::default();
        self.valid = false;
    }

    /// Raw telemetry of one segment, frame-aligned with the program.
    fn feed(&mut self, i: &[f32], v: &[f32]) {
        let a = self.coef;
        for (&ii, &vv) in i.iter().zip(v) {
            // scaled first, in Float32 (rig law (a))
            self.gv.push(self.v_scale * vv, a);
            self.gi.push(self.i_scale * ii, a);
            self.n += 1;
            if self.n == self.window {
                self.n = 0;
                let nn = self.window as f32;
                let pv = self.gv.take(a, nn);
                let pi = self.gi.take(a, nn);
                self.window_done(pv, pi);
            }
        }
    }

    /// One completed Goertzel period (rig law (a)): the band-power gates,
    /// then `R_w = sqrt(Pv / Pi)` and the smoother `R_s += k_R (R_w - R_s)`,
    /// which also takes out-of-range windows; accepted iff the smoothed
    /// `R_s` lies in `RebRange`.
    fn window_done(&mut self, pv: f32, pi: f32) {
        self.last_pv = pv;
        self.last_pi = pi;
        if !(pv.is_finite() && pi.is_finite() && pv > self.v_floor && pi > self.i_floor) {
            self.valid = false;
            return;
        }
        let r = libm::sqrtf(pv / pi);
        self.r_window = r;
        self.r_s += self.k_r * (r - self.r_s);
        // T_CL = fmaf(invTempCo, (Rw - Rshunt)/(R_cal - Rshunt) - 1, T_cal) in
        // Float32, Rw = RdcScale * R_s, the logged window resistance (rig
        // law 3)
        let t = self.t_of_rs();
        // TODO(SPEC): the range is tested on R_s here; the rig confirmed it
        // at RdcScale 1, so whether RdcScale applies first is open
        let re = self.r_s as f64;
        if t.is_finite() && self.r_s.is_finite() {
            self.t_cl = t;
            self.valid = re >= self.reb_min && re <= self.reb_max;
            if self.valid {
                self.age = 0;
                self.fresh = true;
            }
        } else {
            self.valid = false;
        }
    }

    /// The plausibility guard, on each accepted window (beyond the
    /// reference, stricter only: it can only stop relaxation). A coil's
    /// resistance rises with its temperature; across the last
    /// [`TREND_N`] driven windows, while the model's T_OL rises by at least
    /// [`TREND_SPAN_K`], a window resistance falling against T_OL faster
    /// than [`TREND_SLOPE_FRAC`] of an honest coil's rise means the sense
    /// is not reading the coil (the J616s live test: V and I swapped, R
    /// falling as the coils heated, correlation -0.95). So does a coil read
    /// [`COLD_MARGIN_K`] below both its calibration temperature and the
    /// model's ambient on [`COLD_N`] consecutive driven windows (a driven
    /// coil cannot sit below its surroundings). The trend test only judges
    /// rings across which T_OL rose by the span (first to last window), so
    /// music that lets the model and the coil wander in opposite
    /// directions is not a trip. Either latches `implausible`
    /// until the kernel is rebuilt.
    fn guard(&mut self, t_ol: f64, driven: bool) {
        if !driven {
            self.trend_n = 0;
            self.cold_n = 0;
            return;
        }
        if self.trend_n == TREND_N {
            self.trend.copy_within(1.., 0);
            self.trend_n -= 1;
        }
        // the window's own resistance, not the smoother: R_s e-folds from
        // R_cal after every (re)start, which falls in a cold room while the
        // model heats
        self.trend[self.trend_n] = (t_ol, self.r_window as f64);
        self.trend_n += 1;
        if self.trend_n == TREND_N {
            let pts = &self.trend[..];
            // a net rise across the ring: music that lets the model fall
            // back while the coil wanders is not judged
            if pts[TREND_N - 1].0 - pts[0].0 >= TREND_SPAN_K {
                let n = TREND_N as f64;
                let (mt, mr) = (pts.iter().map(|p| p.0).sum::<f64>() / n, pts.iter().map(|p| p.1).sum::<f64>() / n);
                let cov: f64 = pts.iter().map(|p| (p.0 - mt) * (p.1 - mr)).sum();
                let var: f64 = pts.iter().map(|p| (p.0 - mt) * (p.0 - mt)).sum();
                // ohm per kelvin; an honest coil's sensed R_s gains
                // alpha * (R_cal - Rshunt) / RdcScale
                let slope = cov / var;
                let honest = (self.cal_r - self.rshunt as f64) / (self.inv_temp_co as f64 * self.rdc_scale);
                if slope < -TREND_SLOPE_FRAC * honest {
                    self.implausible = true;
                }
            }
        }
        self.cold_n = if self.t_cl < self.cold_floor { self.cold_n + 1 } else { 0 };
        if self.cold_n >= COLD_N {
            self.implausible = true;
        }
    }

    /// The correction may relax the limit: never while capped, never after
    /// the plausibility guard has tripped.
    fn relax_allowed(&self) -> bool {
        self.relax && !self.implausible
    }

    /// Per thermal update: the smoothed corrected limit for `base`.
    fn update(&mut self, t_ol: f64, base: f64, driven: bool) -> f64 {
        if self.valid {
            self.active = true;
        }
        if self.fresh {
            self.fresh = false;
            self.guard(t_ol, driven);
        }
        // a tripped guard drops any relaxation at once (TlimAdj -> 0)
        if self.implausible {
            self.limit_s = base;
            return base;
        }
        if !self.active {
            return base;
        }
        // a measurement older than one window (plus this update) is stale
        self.age = self.age.saturating_add(self.upd);
        if self.age > self.window.saturating_add(self.upd) {
            self.valid = false;
        }
        // max/min, not clamp: a NaN error falls back to the plain base.
        let relax = if self.relax_allowed() { (t_ol - self.t_cl).max(0.0).min(self.cap) } else { 0.0 };
        let raw = if self.valid { base + relax } else { base };
        self.limit_s += self.k_lim * (raw - self.limit_s);
        self.limit_s
    }
}

/// Offline replay of one channel's closed-loop estimator, limit smoother
/// and plausibility guard from logged telemetry (window resistances and
/// per-update T_OL and drive), with the relaxation chosen by the caller.
/// For qualifying a relaxed build against live captures; the shipping
/// unit never constructs it.
#[doc(hidden)]
pub struct ClReplay {
    cl: Cl,
    base: f64,
}

impl ClReplay {
    /// `None` when the channel has no usable calibration (as the unit).
    pub fn new(p: &Params, ch: usize, relax: bool) -> Option<ClReplay> {
        let c = p.ch.get(ch)?;
        let rate = p.sample_rate as f64;
        let r = rate / thermal_frames(p.sample_rate) as f64;
        let mut cl = Cl::new(c, rate, r, p.thermal_ambient_temp as f64)?;
        cl.relax = relax;
        let base = c.temperature_limit as f64;
        cl.limit_s = base;
        Some(ClReplay { cl, base })
    }

    /// One completed Goertzel period with its scaled pilot-band V and I
    /// powers, through the unit's own gates, smoother and range test.
    /// True when the window was accepted.
    pub fn window(&mut self, pv: f32, pi: f32) -> bool {
        self.cl.window_done(pv, pi);
        self.cl.valid
    }

    /// One thermal update: returns (TlimAdj, T_CL, implausible).
    pub fn update(&mut self, t_ol: f64, p_drv: f64) -> (f64, f64, bool) {
        let driven = (p_drv as f32) >= self.cl.drive_thres;
        let lim = self.cl.update(t_ol, self.base, driven);
        (lim - self.base, self.cl.t_cl, self.cl.implausible)
    }

    /// Thermal updates per Goertzel window.
    pub fn updates_per_window(&self) -> f64 {
        self.cl.window as f64 / self.cl.upd as f64
    }
}

/// The generated pilot tone of one channel (spec 7, pilot method).
struct Pilot {
    a_hi: f32,
    a_lo: f32,
    upper: f64,
    lower: f64,
    enable: f32,
    /// Block stage (once per thermal update, `PilotDecayTime` at the
    /// update rate) and the two per-sample stages (`PilotDecayTimeStage2 /
    /// 2` each), all with the 99 % law.
    kb: f32,
    ks: f32,
    /// The sine is `sin(2 pi m n / N)` on the analyser's bin `m` of its
    /// period `N`, `n` the absolute frame since construction, running while
    /// the pilot is off (43.9453125 Hz for `PilotFreq` 43.0664).
    period: usize,
    bin: f64,
    n: usize,
    /// High/low selection (hysteresis on `T_OL`) and the content gate.
    high: bool,
    on: bool,
    /// Block-stage output, held over the next update's frames, and the two
    /// per-sample stages.
    s1: f32,
    a: f32,
    b: f32,
}

/// `min(10^(dB/20), 0.3)`: the pilot amplitude, clamped (rig law 4).
fn pilot_amplitude(db: f32) -> f32 {
    (libm::pow(10.0, db as f64 / 20.0) as f32).min(PILOT_AMPLITUDE_MAX)
}

/// Largest pilot amplitude (rig law 4: -10 and -10.4 dB both give 0.3).
const PILOT_AMPLITUDE_MAX: f32 = 0.3;

impl Pilot {
    fn new(c: &ChannelParams, rate: f64) -> Pilot {
        let period = pilot_period(c, rate);
        Pilot {
            a_hi: pilot_amplitude(c.pilot_ampl_hi_db),
            a_lo: pilot_amplitude(c.pilot_ampl_lo_db),
            upper: c.pilot_upper_thres as f64,
            lower: c.pilot_lower_thres as f64,
            enable: c.pilot_enable_thres,
            kb: settle99(c.pilot_decay_time_s as f64, rate / thermal_frames(rate as u32) as f64) as f32,
            ks: settle99(c.pilot_decay_time_stage2_s as f64 / 2.0, rate) as f32,
            period,
            bin: libm::round(c.pilot_freq as f64 * period as f64 / rate),
            n: 0,
            high: false,
            on: false,
            s1: 0.0,
            a: 0.0,
            b: 0.0,
        }
    }
    fn reset(&mut self) {
        self.n = 0;
        self.high = false;
        self.on = false;
        self.s1 = 0.0;
        self.a = 0.0;
        self.b = 0.0;
    }
    /// Per thermal update, from that update's `Pdrv` and `T_OL`: the target
    /// and the block stage, which the next update's frames see (one block
    /// of latency on and off).
    fn update(&mut self, p_drv: f64, t_ol: f64) {
        self.on = (p_drv as f32) >= self.enable;
        if t_ol > self.upper {
            self.high = true;
        } else if t_ol < self.lower {
            self.high = false;
        }
        let tgt = self.target();
        self.s1 += self.kb * (tgt - self.s1);
    }
    fn target(&self) -> f32 {
        if !self.on {
            0.0
        } else if self.high {
            self.a_hi
        } else {
            self.a_lo
        }
    }
    /// Add the pilot to `out`, unscaled by the protection gain (rig law 4).
    fn add(&mut self, out: &mut [f32]) {
        let w = 2.0 * core::f64::consts::PI * self.bin / self.period as f64;
        for o in out.iter_mut() {
            self.a += self.ks * (self.s1 - self.a);
            self.b += self.ks * (self.a - self.b);
            *o += self.b * libm::sin(w * self.n as f64) as f32;
            self.n += 1;
            if self.n == self.period {
                self.n = 0;
            }
        }
    }
}

struct Chan {
    src: usize,
    group: usize,
    headroom: f32,
    scratch: Vec<f32>,
    /// A non-finite program sample in the current thermal update: the rest
    /// of the update gets the meter-failure 20 dB.
    meter_failed: bool,
    elec: f64,
    headroom2: f64,
    safe: SafeTlim,
    headroom_c: f64,
    vc: Thermal,
    mg: Thermal,
    /// The feed-forward virtual coil on the unprotected drive (`tau_ff`).
    /// With the candidate disabled it runs at the physical `tau_Tvc` and
    /// only the hand-off reads it.
    ff: Thermal,
    ff_on: bool,
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
    hold: Hold,
    g_hard: f64,
    armed: bool,
    /// The feed-forward node is above [`FF_RANGE_K`].
    out_of_range: bool,
    vi_power: f64,
}

impl Chan {
    /// Kernel construction state of the thermal model and controller.
    fn seed(&mut self) {
        // the nodes are Float32 state
        self.vc.theta = self.seed.0 as f32 as f64;
        self.mg.theta = self.seed.1 as f32 as f64;
        self.ff.theta = self.seed.2 as f32 as f64;
        self.p_prev = 0.0;
        self.p_ff_prev = 0.0;
        self.meter_failed = false;
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
        self.hold = HOLD_START;
        self.g_hard = 1.0;
        self.armed = false;
        self.out_of_range = false;
        self.vi_power = f64::NAN;
    }
}

/// The release hold of the ordinary candidate (rig law 5): when the fresh
/// candidate first reaches the previous selected gain (`>=`: the hard-path
/// floor tie starts it), the previous candidate is held for `N = ceil(1 s *
/// update rate)` updates (188 at 48 kHz); a fresh candidate below the
/// selected gain cancels it, and after a completed hold the fresh
/// candidate passes until it falls below the selected gain again.
#[derive(Clone, Copy)]
struct Hold {
    state: HoldState,
    cnt: u32,
    held: f64,
    u_prev: f64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum HoldState {
    Attack,
    Holding,
    Released,
}

const HOLD_START: Hold = Hold { state: HoldState::Attack, cnt: 0, held: 1.0, u_prev: 1.0 };

impl Hold {
    /// The candidate fed to the smoother this update.
    fn step(&mut self, g: f64, sel_prev: f64, n: u32) -> f64 {
        let u = self.decide(g, sel_prev, n);
        self.u_prev = u;
        u
    }
    fn decide(&mut self, g: f64, sel_prev: f64, n: u32) -> f64 {
        if self.cnt > 0 {
            if g >= sel_prev {
                self.cnt -= 1;
                if self.cnt == 0 {
                    self.state = HoldState::Released;
                }
                return self.held;
            }
            self.cnt = 0;
        }
        if g >= sel_prev {
            if self.state == HoldState::Attack {
                self.held = self.u_prev;
                self.cnt = n.saturating_sub(1);
                self.state = if self.cnt == 0 { HoldState::Released } else { HoldState::Holding };
                self.held
            } else {
                g
            }
        } else {
            self.state = HoldState::Attack;
            g
        }
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
    /// Release-hold length, updates (`ceil(1 s * fs / 256)`).
    hold_n: u32,
    ch: Vec<Chan>,
    groups: Vec<Group>,
    /// Per group, the dezippered gain of the current segment's frames.
    curve: Vec<f32>,
    /// Thermal update, frames ([`thermal_frames`]).
    upd: usize,
    pos: usize,
    frame: u64,
    last_frame: u64,
    /// The graph's render block, frames, and the end of the sense the
    /// closed loop has already taken ahead for the current block.
    block: usize,
    prefed_until: u64,
    sums: BlockSums,
    sense_ok: bool,
    /// A non-finite sense sample in the current update: it runs open loop.
    sense_bad: bool,
    estimator: DelayEstimator,
    /// A model state went non-finite: the fail-safe is latched until reset.
    fault: bool,
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
    /// Unit channel `c`'s closed-loop plausibility guard has tripped.
    pub fn sense_implausible(&self, c: usize) -> bool {
        self.run.as_ref().and_then(|r| r.ch.get(c)).and_then(|c| c.cl.as_ref()).is_some_and(|l| l.implausible)
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
        let pilot = ch.pilot.as_ref().map_or(f64::NAN, |p| p.target() as f64);
        [
            ch.g_ff,
            g.g,
            ch.tlim_eff,
            if ch.ff_on { ch.ff.theta } else { f64::NAN },
            ch.vc.theta,
            ch.mg.theta,
            ch.p_model,
            ch.p_drv,
            ch.cl.as_ref().map_or(f64::NAN, |l| l.t_cl),
            ch.cl.as_ref().map_or(f64::NAN, |l| l.valid as u8 as f64),
            pilot,
        ]
    }
    /// `AudioUnitReset` (spec 10): clears the closed-loop estimator and the
    /// pilot generator. The open-loop coil, magnet and feed-forward
    /// temperatures, the gain state and the SafeTlim state continue; an
    /// open-loop kernel is unchanged. Wall-clock idle is never thermal time.
    pub fn audio_unit_reset(&mut self) {
        if let Some(r) = self.run.as_mut() {
            for c in r.ch.iter_mut() {
                if let Some(cl) = c.cl.as_mut() {
                    cl.host_reset();
                }
                // and the pilot generator restarts: envelope at zero, phase
                // from this frame, the low level (rig law-restore-cl-reset:
                // 5.7e-7 against 0.029 with the generator kept)
                if let Some(p) = c.pilot.as_mut() {
                    p.reset();
                }
                // the effective limit is the base until reacquisition
                c.tlim_eff = c.safe.base();
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
        let upd = thermal_frames(p.sample_rate);
        let r = rate / upd as f64;
        let look = libm::round(p.thermal_lookahead_delay_ms as f64 * 1e-3 * rate) as usize;
        let ambient = p.thermal_ambient_temp as f64;
        let cold = p.thermal_state_override == THERMAL_STATE_COLD;
        // a hot seed below ambient would allow more drive than a cold start
        if !cold && p.ch[..p.channels as usize].iter().any(|c| (c.temperature_limit as f64) < ambient) {
            return Err(InitError::Invalid("hot start below ambient"));
        }
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
            let ff_on = ff_factor > 0.0;
            let ff_tau = if ff_on { c.thermal_ff_speedup_factor * c.tau_tvc } else { c.tau_tvc };
            let dt32 = upd as f32 / p.sample_rate as f32;
            let ff = Thermal::new(c.t_sett_vc, ff_tau, dt32);
            let (tvc, tmg) = (c.t_sett_vc as f64, c.t_sett_mg as f64);
            // hot fallback: the steady-state split at the ordinary limit
            let seed = if cold {
                (0.0, 0.0, 0.0)
            } else {
                let p0 = (c.temperature_limit as f64 - ambient) / (tvc + tmg);
                (p0 * tvc, p0 * tmg, p0 * tvc)
            };
            let headroom = if c.pilot_tone_enabled != 0 { 1.0 - pilot_amplitude(c.pilot_ampl_hi_db) } else { 1.0 };
            let pilot_cfg = c.pilot_tone_enabled != 0
                && c.cl_thermal_feedback_enabled != 0
                && c.thermal_measurement_method == METHOD_PILOT_TONE
                && cal_vector;
            let mut chan = Chan {
                src: c.source as usize,
                group: gi,
                headroom,
                scratch: vec![0.0; MODEL_SCRATCH],
                meter_failed: false,
                elec: ag * ag * reb / ((reb + series) * (reb + series)),
                headroom2: (headroom as f64) * (headroom as f64),
                safe: SafeTlim::new(c.temperature_limit, p.thermal_safe_tlim, p.thermal_safe_tlim_offset, safe_init),
                headroom_c: c.hard_temp_limit_headroom as f64,
                vc: Thermal::new(c.t_sett_vc, c.tau_tvc, dt32),
                mg: Thermal::new(c.t_sett_mg, c.tau_tmg, dt32),
                ff,
                ff_on,
                seed,
                p_prev: 0.0,
                p_ff_prev: 0.0,
                acc: dsp::LaneAcc::new(),
                acc_in: dsp::LaneAcc::new(),
                sense: if c.ignore_telemetry != 0 { None } else { Some((c.sense_i as usize, c.sense_v as usize)) },
                vi_scale: c.imon_scale as f64 * c.vmon_scale as f64,
                acc_vi: dsp::LaneAcc::new(),
                cl: if cal_ok { Cl::new(c, rate, r, ambient) } else { None },
                pilot: pilot_cfg.then(|| Pilot::new(c, rate)),
                p_model: 0.0,
                p_drv: 0.0,
                tlim_eff: c.temperature_limit as f64,
                g_ff: 1.0,
                hold: HOLD_START,
                g_hard: 1.0,
                armed: false,
                out_of_range: false,
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
            hold_n: libm::ceil(HOLD_S * r) as u32,
            ch,
            groups: vec![GROUP_START; ngroups],
            curve: vec![1.0; ngroups * upd],
            upd,
            pos: 0,
            frame: 0,
            last_frame: 0,
            block: p.block_frames as usize,
            prefed_until: 0,
            sums,
            sense_ok: false,
            sense_bad: false,
            estimator: DelayEstimator::new(),
            fault: false,
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
        self.prefed_until = 0;
        self.sums.clear();
        self.sense_ok = false;
        self.sense_bad = false;
        self.estimator.reset();
        self.fault = false;
    }

    /// The reference consumes a whole render block's telemetry before the
    /// block's first thermal update, so a Goertzel period ending inside the
    /// block acts from that first update (rig law (a), at the graph's
    /// 512-frame slices). With the whole block in this call, take its
    /// sense now; otherwise the segments feed it as they come.
    fn prefeed(&mut self, io: &Io<'_, '_>, off: usize) {
        let n = self.block;
        let sense = io.sense;
        let fits = |p: usize| sense.get(p).is_some_and(|b| b.len() >= off + n);
        if !self.ch.iter().all(|c| c.sense.is_none_or(|(si, sv)| fits(si) && fits(sv))) {
            return;
        }
        for c in self.ch.iter_mut() {
            if let (Some((si, sv)), Some(cl)) = (c.sense, c.cl.as_mut()) {
                cl.feed(&sense[si][off..off + n], &sense[sv][off..off + n]);
            }
        }
        self.prefed_until = self.frame + n as u64;
    }

    /// Frames `off..off+len` of the host buffers, inside one thermal update.
    fn segment(&mut self, io: &mut Io<'_, '_>, off: usize, len: usize, health: &mut u32) {
        let phase = self.pos;
        let k = self.k_dez;
        // per group: the two cascaded per-sample dezipper stages
        for (gi, g) in self.groups.iter_mut().enumerate() {
            let curve = &mut self.curve[gi * self.upd..][..len];
            for v in curve.iter_mut() {
                g.z1 += k * (g.sel - g.z1);
                g.z2 += k * (g.z1 - g.z2);
                *v = g.z2;
            }
        }
        for c in self.ch.iter_mut() {
            let inp = &io.input[c.src][off..off + len];
            let out = &mut io.output[c.src][off..off + len];
            let curve = &self.curve[c.group * self.upd..][..len];
            // the program held to full scale, times the gain
            let non_finite = dsp::mul_clamped_to(inp, curve, out, INPUT_CLAMP);
            if c.headroom != 1.0 {
                dsp::scale_in_place(out, c.headroom);
            }
            if c.meter_failed || non_finite {
                // simple-meter failure: at most 20 dB below the input from
                // the first non-finite sample to the end of its thermal
                // update (where the host split its buffers does not move
                // it), non-finite samples muted
                let from = if c.meter_failed { 0 } else { inp.iter().position(|x| !x.is_finite()).unwrap_or(len) };
                c.meter_failed = true;
                for ((o, &x), &g) in out[from..].iter_mut().zip(&inp[from..]).zip(&curve[from..]) {
                    *o = dsp::clamp1(x, INPUT_CLAMP) * (g * c.headroom).min(FAILSAFE_GAIN);
                }
                *health |= Health::FAILSAFE;
            }
            // Pcoil: the protected, headroom-reduced drive on the update's
            // own frames, metered after the pilot is added (rig law 2).
            if PCOIL_WITH_PILOT {
                if let Some(p) = c.pilot.as_mut() {
                    p.add(out);
                }
            }
            c.acc.add_sq(out, phase);
            // the meters see the reference's +-1.5 FS drive; the samples that
            // leave the unit are held to digital full scale (policy: the
            // output stays bounded; only |y| > 1 differs from the reference)
            dsp::clamp_in_place(out, 1.0);
            // Pdrv: the unprotected program on the same frames
            let mut m0 = 0;
            while m0 < len {
                let m = (len - m0).min(c.scratch.len());
                c.scratch[..m].copy_from_slice(&inp[m0..m0 + m]);
                // held to full scale like the program the output carries
                dsp::clamp_in_place(&mut c.scratch[..m], INPUT_CLAMP);
                c.acc_in.add_sq(&c.scratch[..m], phase + m0);
                m0 += m;
            }
            if !PCOIL_WITH_PILOT {
                if let Some(p) = c.pilot.as_mut() {
                    p.add(out);
                }
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
                    // non-finite sense is unusable sense: the block runs open
                    // loop (the closed loop rejects such a window itself)
                    if dsp::peak_abs(i) == f32::INFINITY || dsp::peak_abs(v) == f32::INFINITY {
                        self.sense_bad = true;
                    }
                    c.acc_vi.add_prod(v, i, phase);
                    if let Some(cl) = c.cl.as_mut().filter(|_| self.frame >= self.prefed_until) {
                        cl.feed(i, v);
                    }
                }
                _ => {
                    self.sums.complete = false;
                    if let Some(cl) = c.cl.as_mut() {
                        cl.lose();
                    }
                }
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
                // (non-finite or overflowing sums restart the estimator's
                // correlation at the envelope point that holds them)
                self.estimator.push_sums(oe as f32, ve as f32, m);
                kk += m;
            }
        }
        self.frame += len as u64;
    }

    /// One thermal update (every [`thermal_frames`]).
    fn update(&mut self) {
        let n = self.upd as f64;
        let mut ff_t = [1.0f64; MAX_CHANNELS];
        let mut hard_t = [1.0f64; MAX_CHANNELS];
        for c in self.ch.iter_mut() {
            c.meter_failed = false;
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
            c.ff.step(p_ff, c.p_ff_prev);
            c.p_prev = p;
            c.p_ff_prev = p_ff;
            let t_ol = self.ambient + c.vc.theta + c.mg.theta;
            // the base limit (TemperatureLimit, or SafeTlim once latched)
            // and the closed-loop correction smoothed around it
            let base = c.safe.base();
            let limit = match c.cl.as_mut() {
                Some(cl) => {
                    // driven: the update's drive meets the pilot content gate
                    let driven = (p_ff as f32) >= cl.drive_thres;
                    cl.update(t_ol, base, driven)
                }
                None => base,
            };
            c.tlim_eff = limit;
            let num = limit - self.ambient - c.mg.theta;
            let vcff = c.ff.theta;
            let ratio = |num: f64| libm::sqrt((num / (vcff + PREDICTION_MARGIN)).clamp(RATIO_MIN, 1.0));
            let fresh = if c.ff_on { ratio(num) } else { 1.0 };
            // release hold (rig law 5) against the previous selected gain
            c.g_ff = c.hold.step(fresh, self.groups[c.group].g, self.hold_n);
            // the hard path follows the effective limit
            if t_ol > limit + c.headroom_c {
                c.armed = true;
            } else if t_ol < limit {
                c.armed = false;
            }
            // Rig law 6a: with the feed-forward node above 1000 K the
            // reference re-seeds all three nodes to the hot split, which
            // snaps T_OL down to about TemperatureLimit while the drive
            // continues (163 -> 140.7 C on law-clip). By policy the engine
            // keeps integrating the real state and holds the hard path
            // while the node is out of range, flagged MODEL_RANGE.
            c.out_of_range = c.ff_on && c.ff.theta > FF_RANGE_K;
            if c.out_of_range {
                c.armed = true;
            }
            // without the feed-forward filter the reference has no working
            // hard path (rig law 6); by policy the fixed 20 dB holds the
            // limit here (module docs)
            c.g_hard = if !c.armed {
                1.0
            } else if c.ff_on {
                ratio(num - HARD_TARGET_BELOW)
            } else {
                FAILSAFE_GAIN as f64
            };
            ff_t[c.group] = min_nan(ff_t[c.group], c.g_ff);
            hard_t[c.group] = min_nan(hard_t[c.group], c.g_hard);
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
            let sel = min_nan(g.g, hard_t[gi]);
            g.g = sel;
            g.sel = sel as f32;
        }
        let complete = self.sums.complete
            && !self.sense_bad
            && self.sums.frames == self.upd
            && self.ch.iter().all(|c| c.sense.is_none() || c.acc_vi.is_finite());
        for c in self.ch.iter_mut() {
            c.vi_power = if c.sense.is_some() && complete && c.acc_vi.is_finite() {
                c.acc_vi.total() * c.vi_scale / n
            } else {
                f64::NAN
            };
            c.acc_vi.clear();
        }
        self.sense_ok = complete;
        self.sense_bad = false;
        self.estimator.end_block();
        self.sums.clear();
        self.last_frame = self.frame - self.upd as u64;
        // any non-finite model or controller state latches the fail-safe
        let chans = self.ch.iter().all(|c| {
            let lim = c.cl.as_ref().is_none_or(|cl| cl.limit_s.is_finite() && cl.t_cl.is_finite());
            c.vc.theta.is_finite()
                && c.mg.theta.is_finite()
                && c.ff.theta.is_finite()
                && lim
                && c.g_ff.is_finite()
                && c.g_hard.is_finite()
        });
        let groups = self.groups.iter().all(|g| g.g.is_finite() && g.sel.is_finite());
        if !(chans && groups) {
            self.fault = true;
        }
    }
}

/// `min` that propagates NaN (`f64::min` drops it, which here would turn a
/// poisoned candidate into unity gain).
#[inline]
fn min_nan(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.min(b)
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
        if run.fault {
            failsafe_io(&mut io);
            return Health(Health::FAILSAFE | Health::MODEL_FAULT);
        }
        // at the start of every render call both dezipper stages take the
        // current selected target (rig dezipper law: 1 ulp on every sample
        // at host slices 128..1024); inside the call they glide per sample
        for g in run.groups.iter_mut() {
            g.z1 = g.sel;
            g.z2 = g.sel;
        }
        let mut health = 0u32;
        let mut off = 0;
        while off < frames {
            let len = (run.upd - run.pos).min(frames - off);
            if run.frame >= run.prefed_until
                && run.block > 0
                && run.frame % run.block as u64 == 0
                && frames - off >= run.block
            {
                run.prefeed(&io, off);
            }
            run.segment(&mut io, off, len, &mut health);
            off += len;
            run.pos += len;
            if run.pos == run.upd {
                run.update();
                run.pos = 0;
                if run.fault {
                    crate::failsafe::fill_from(&mut io, off, FAILSAFE_GAIN);
                    return Health(Health::FAILSAFE | Health::MODEL_FAULT);
                }
            }
        }
        if !run.sense_ok {
            health |= Health::OPEN_LOOP;
        }
        if run.ch.iter().any(|c| c.out_of_range) {
            health |= Health::MODEL_RANGE;
        }
        if run.ch.iter().any(|c| c.cl.as_ref().is_some_and(|l| l.implausible)) {
            health |= Health::SENSE_IMPLAUSIBLE;
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
        if run.fault {
            out.health = Health(Health::FAILSAFE | Health::MODEL_FAULT);
        }
        if run.ch.iter().any(|c| c.out_of_range) {
            out.health = Health(out.health.0 | Health::MODEL_RANGE);
        }
        if run.ch.iter().any(|c| c.cl.as_ref().is_some_and(|l| l.implausible)) {
            out.health = Health(out.health.0 | Health::SENSE_IMPLAUSIBLE);
        }
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
            // the effective limit against the ordinary TemperatureLimit, as the
            // reference's controller record logs it: a latched SafeTlim shows
            // as SafeTlim - TemperatureLimit, a closed-loop relaxation as the
            // positive excess
            s.tlim_adj_c = (c.tlim_eff - c.safe.normal) as f32;
            match c.cl.as_ref() {
                Some(cl) => {
                    s.cl_temp_c = cl.t_cl as f32;
                    s.closed_loop = true;
                    s.sense_v_ms = cl.last_pv;
                    s.sense_i_ms = cl.last_pi;
                    s.sense_r_ohm = cl.r_window;
                }
                None => {
                    s.cl_temp_c = T_CL_UNMEASURED;
                    s.closed_loop = false;
                }
            }
            // the part meters (spp3 part 64/65/66): thermal is the applied
            // gain; displacement and CPMS power are disabled and unity
            s.axis_gain[1] = 1.0;
            s.axis_gain[3] = 1.0;
            s.axis_gain[4] = if run.fault { FAILSAFE_GAIN } else { g.z2 };
            s.gain = if run.fault { FAILSAFE_GAIN } else { g.z2 };
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

    fn save_state(&self, out: &mut HandOff) -> bool {
        let Some(run) = self.run.as_ref().filter(|r| !r.fault) else { return false };
        *out = HandOff::EMPTY;
        out.kind = UnitKind::Spp3 as u32;
        out.channels = run.n as u32;
        out.sample_rate = self.params.sample_rate;
        for (i, c) in run.ch.iter().enumerate() {
            out.node[i][..4].copy_from_slice(&[c.vc.theta, c.mg.theta, c.ff.theta, c.armed as u8 as f64]);
            out.gain[i] = run.groups[c.group].g;
        }
        true
    }

    fn restore_state(&mut self, state: &HandOff, gap_s: f64) -> bool {
        let rate = self.params.sample_rate;
        let Some(run) = self.run.as_mut() else { return false };
        if !state.fits(UnitKind::Spp3, run.n, rate) {
            return false;
        }
        let t = gap_or_zero(gap_s);
        let updates = t * rate as f64 / run.upd as f64;
        let mut g = [1.0f64; MAX_CHANNELS];
        for (i, c) in run.ch.iter_mut().enumerate() {
            let s = &state.node[i];
            c.vc.theta = s[0] * libm::exp(-t / c.vc.tau);
            c.mg.theta = s[1] * libm::exp(-t / c.mg.tau);
            c.ff.theta = s[2] * libm::exp(-t / c.ff.tau);
            // hysteresis: an armed hard path stays armed until T_OL < limit
            c.armed = s[3] != 0.0 && run.ambient + c.vc.theta + c.mg.theta >= c.safe.base();
            // a restored channel does not re-trigger the release hold: the
            // rig's newinst=0 releases from the restored gain toward the
            // fresh candidate at once (ctl[0] 0.528 from the first update)
            c.hold = Hold { state: HoldState::Released, ..HOLD_START };
            g[c.group] = g[c.group].min(state.gain[i]);
        }
        let k = 1.0 - run.k_release;
        for (gi, grp) in run.groups.iter_mut().enumerate() {
            let v = release_toward_unity(g[gi], k, updates);
            *grp = Group { g: v, sel: v as f32, z1: v as f32, z2: v as f32 };
        }
        run.curve.iter_mut().for_each(|v| *v = 1.0);
        true
    }
}

/// 20 dB on whatever geometry the host gave, never indexing past a buffer,
/// finite and bounded for any input (see [`crate::failsafe::fill_from`]).
pub(crate) fn failsafe_io(io: &mut Io<'_, '_>) {
    crate::failsafe::fill_from(io, 0, FAILSAFE_GAIN);
}

#[cfg(test)]
mod fault_tests {
    use super::*;

    fn params() -> Params {
        let mut p = Params {
            channels: 2,
            sample_rate: 48_000,
            block_frames: 512,
            thermal_ambient_temp: 40.0,
            thermal_gain_attack_time_s: 0.5,
            thermal_gain_release_time_s: 2.0,
            thermal_lookahead_delay_ms: 1.0,
            thermal_protection_enabled: 1,
            thermal_state_override: THERMAL_STATE_COLD,
            thermal_safe_tlim: 100.0,
            thermal_safe_tlim_offset: -15.0,
            thermal_safe_tlim_time_min: 10.0,
            ..Params::default()
        };
        for c in 0..2 {
            p.ch[c] = ChannelParams {
                source: c as u32,
                t_sett_vc: 20.0,
                t_sett_mg: 10.0,
                tau_tvc: 0.4,
                tau_tmg: 3.0,
                temperature_limit: 120.0,
                hard_temp_limit_headroom: 8.0,
                reb_ref: 4.0,
                rshunt: 0.2,
                ag: 10.0,
                thermal_ff_speedup_factor: 0.5,
                ..Default::default()
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

    /// Review finding 1: a NaN state used to vanish in `f64::min` and leave
    /// unity gain. Every way a state can go non-finite now latches 20 dB.
    #[test]
    fn a_non_finite_model_state_latches_the_failsafe_until_reset() {
        let poisons: [fn(&mut Run); 4] = [
            |r| r.ch[0].mg.theta = f64::NAN,
            |r| r.ch[1].vc.theta = f64::INFINITY,
            |r| r.ch[0].ff.theta = f64::NAN,
            |r| r.groups[0].g = f64::NAN,
        ];
        for poison in poisons {
            let mut u = Unit::new(params());
            let (o, h) = run(&mut u, 0.5, 512);
            assert_eq!((o[511], h.0 & Health::FAILSAFE), (0.5, 0));
            poison(u.run.as_mut().unwrap());
            // the next update latches; the frames after it get 20 dB
            let (o, h) = run(&mut u, 0.5, 512);
            assert!(h.0 & Health::MODEL_FAULT != 0 && h.is_failsafe());
            assert!((o[511] - 0.05).abs() < 1e-7, "{}", o[511]);
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
            let (o, h) = run(&mut u, 0.5, 512);
            assert_eq!((o[511], h.0 & Health::FAILSAFE), (0.5, 0));
        }
    }

    /// Review finding 3: an accepted measurement expires one window (plus
    /// one update) after it was taken, even if no segment ever reports the
    /// bus lost.
    #[test]
    fn a_closed_loop_measurement_expires_after_one_window() {
        let mut c = params().ch[0];
        c.cl_thermal_feedback_enabled = 1;
        c.cal_present = 1;
        c.cal_r = 4.0;
        c.cal_t = 25.0;
        c.imon_scale = 1.0;
        c.vmon_scale = 1.0;
        c.temp_sense_window_time_s = 0.02;
        c.temp_sense_smoothing_tau_s = 0.5;
        c.rdc_scale = 1.0;
        c.reb_range_min = 2.0;
        c.reb_range_max = 8.0;
        c.cal_temp_range_min = 10.0;
        c.cal_temp_range_max = 50.0;
        c.a_t_20c = 0.004;
        c.a_t_35c = 0.004;
        c.tlim_err_decay_time_s = 1.0;
        c.tlim_adj_cap = 30.0;
        c.thermal_measurement_method = METHOD_PILOT_TONE;
        c.pilot_freq = 43.0664;
        let mut cl = Cl::new(&c, 48_000.0, 48_000.0 / THERMAL_FRAMES as f64, 25.0).unwrap();
        let w = cl.window;
        // a sine on the analyser bin, R = 4 ohm
        let m = libm::round(c.pilot_freq as f64 * w as f64 / 48_000.0);
        let v: Vec<f32> =
            (0..w).map(|k| libm::sin(2.0 * core::f64::consts::PI * m * k as f64 / w as f64) as f32).collect();
        let i: Vec<f32> = v.iter().map(|x| x * 0.25).collect();
        cl.feed(&i, &v);
        assert!(cl.valid);
        let mut updates = 0;
        while cl.valid {
            cl.update(200.0, 120.0, true);
            updates += 1;
            assert!(updates * THERMAL_FRAMES <= w + 2 * THERMAL_FRAMES, "still valid after {updates} updates");
        }
        // and a lost segment invalidates at once
        cl.feed(&i, &v);
        assert!(cl.valid);
        cl.lose();
        assert!(!cl.valid && cl.n == 0);
    }
}
