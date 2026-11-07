// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Parameter-ID names for the protection units' saved AudioUnit state.
//!
//! Every entry comes from the behaviour specification (spp3: section 11,
//! "AU SaveState layout and complete ID map").  An ID the specification does
//! not name stays unnamed; callers then see the raw ID.
//!
//! atsp: the specification's AU parameter map (section 2) is complete, 41
//! global IDs: bypass, three 13-ID speaker-type slots, gain link all; the
//! compiler reads every one of them.
//!
//! TODO(SPEC): the lsm2 ID map.  The lsm2 specification names fields by their
//! preset names without IDs; until it states them, lsm2 exposes raw
//! `(scope, element, id, value)`.

use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ParamName {
    pub id: u32,
    pub name: &'static str,
    pub unit: &'static str,
    /// The version-9 sidecar key that carries the same quantity, if the
    /// specification states a one-to-one correspondence.
    pub sidecar: Option<&'static str>,
}

const fn p(id: u32, name: &'static str, unit: &'static str, sidecar: Option<&'static str>) -> ParamName {
    ParamName { id, name, unit, sidecar }
}

/// spp3 global scope (0), element 0: the 15 saved IDs.
pub const SPP3_GLOBAL: &[ParamName] = &[
    p(0, "thermal protection enabled", "bool", None),
    p(1, "displacement protection enabled", "bool", None),
    p(2, "thermal/power gain attack time", "s", Some("Thermal_GainAttackTime_s")),
    p(3, "thermal/power gain release time", "s", Some("Thermal_GainReleaseTime_s")),
    p(4, "ambient temperature", "degC", Some("Thermal_AmbientTemp")),
    p(5, "SafeTlim", "degC", Some("Thermal_SafeTlim")),
    p(6, "SafeTlimTimeMin", "s", Some("Thermal_SafeTlimTimeMin")),
    p(7, "SafeTlimOffset", "degC", Some("Thermal_SafeTlimOffset")),
    p(8, "thermal look-ahead", "ms", Some("Thermal_LookaheadDelay_ms")),
    p(9, "displacement peak attack", "s", Some("Disp_PeakAttackTime_s")),
    p(10, "displacement peak decay", "s", Some("Disp_PeakDecayTime_s")),
    p(11, "displacement feedback integration", "s", Some("Disp_FBIntegrationTime_s")),
    p(14, "reported-speaker-power averaging window", "s", None),
    p(18, "CPMS power control enabled", "bool", None),
    p(19, "CPMS closed-loop power control enabled", "bool", None),
];

/// spp3 global runtime meters (published, not saved).
pub const SPP3_GLOBAL_METERS: &[ParamName] = &[
    p(12, "thermal gain", "dB", None),
    p(13, "displacement gain", "dB", None),
    p(15, "modeled speaker power", "W", None),
    p(16, "measured speaker power", "W", None),
    p(17, "power-control gain", "dB", None),
];

/// spp3 part scope (4), one element per channel: the 57 saved IDs.
pub const SPP3_PART: &[ParamName] = &[
    p(0, "temperature limit", "degC", Some("OL_thermal.TemperatureLimit")),
    p(1, "hard-limit headroom", "degC", Some("OL_thermal.HardTempLimitHeadroom")),
    p(2, "T_sett_vc", "K/W", Some("OL_thermal.T_sett_vc")),
    p(3, "T_sett_mg", "K/W", Some("OL_thermal.T_sett_mg")),
    p(4, "tau_Tvc", "s", Some("OL_thermal.tau_Tvc")),
    p(5, "tau_Tmg", "s", Some("OL_thermal.tau_Tmg")),
    p(6, "ThermalFFSpeedupFactor", "1", Some("OL_thermal.ThermalFFSpeedupFactor")),
    p(9, "Reb_ref", "ohm", Some("OL_thermal.Reb_ref")),
    p(10, "Rshunt", "ohm", Some("OL_thermal.Rshunt")),
    p(11, "Rampout", "1", Some("OL_thermal.Rampout")),
    p(12, "mt", "kg", Some("OL_disp.mt")),
    p(13, "ct", "N*s/m", Some("OL_disp.ct")),
    p(14, "kt", "N/m", Some("OL_disp.kt")),
    p(15, "ag", "V/FS", Some("OL_disp.ag")),
    p(16, "g_bw", "1", Some("OL_disp.g_bw")),
    p(17, "Q_d", "1", Some("OL_disp.Q_d")),
    p(18, "phi", "N/A", Some("OL_disp.phi")),
    p(19, "x_lim", "TODO(SPEC) unit", Some("OL_disp.x_lim")),
    p(20, "ThermalMeasurementMethod", "enum", Some("CL.ThermalMeasurementMethod")),
    p(21, "pilot-tone enabled", "bool", None),
    p(22, "CL thermal feedback enabled", "bool", None),
    p(23, "TlimErrDecayTime", "s", Some("CL.TlimErrDecayTime")),
    p(24, "TempSenseWindowTime", "s", Some("CL.TempSenseWindowTime")),
    p(25, "TempSenseSmoothingTau", "s", Some("CL.TempSenseSmoothingTau")),
    p(26, "a_t_inv (legacy, superseded by sidecar alphas)", "1", None),
    p(27, "PilotAmplHi_dB", "dB", Some("CL.PilotAmplHi_dB")),
    p(28, "PilotAmplLo_dB", "dB", Some("CL.PilotAmplLo_dB")),
    p(29, "PilotUpperThres", "TODO(SPEC) unit", Some("CL.PilotUpperThres")),
    p(30, "PilotLowerThres", "TODO(SPEC) unit", Some("CL.PilotLowerThres")),
    p(31, "PilotDecayTime", "s", Some("CL.PilotDecayTime")),
    p(32, "PilotFreq", "Hz", Some("CL.PilotFreq")),
    p(33, "LPMLSPreGain", "1", Some("CL.LPMLSPreGain")),
    p(34, "LPMLSPostGain", "1", Some("CL.LPMLSPostGain")),
    p(35, "LPMLSLowerCorner", "TODO(SPEC) unit", Some("CL.LPMLSLowerCorner")),
    p(37, "mu_Re", "1", None),
    p(38, "mu_Le", "1", None),
    p(39, "mu mechanical (PU)", "1", None),
    p(42, "DisplacementProtectionType", "enum", Some("CL.DisplacementProtectionType")),
    p(47, "Convoy peak-finder upper threshold", "1", None),
    p(48, "Convoy peak-finder lower threshold", "1", None),
    p(49, "Convoy peak-detector attack", "s", None),
    p(50, "Convoy peak-detector release", "s", None),
    p(51, "Convoy normalization-factor upper cap", "1", None),
    p(52, "Convoy normalization-factor lower cap", "1", None),
    p(53, "Convoy normalization-factor attack", "s", None),
    p(54, "Convoy normalization-factor release", "s", None),
    p(55, "Convoy positive peak-ratio block threshold", "1", None),
    p(56, "Convoy negative peak-ratio block threshold", "1", None),
    p(57, "Convoy minimum excursion", "1", None),
    p(58, "Convoy maximum excursion", "1", None),
    p(59, "Convoy relative-X-limit upper", "1", None),
    p(60, "Convoy relative-X-limit lower", "1", None),
    p(61, "Convoy in-window peak-ratio count threshold", "1", None),
    p(62, "Convoy normalization-factor threshold", "%", None),
    p(63, "Convoy dirty-buffer count threshold", "1", None),
    p(67, "PilotDecayTimeStage2", "s", Some("CL.PilotDecayTimeStage2")),
    p(68, "PilotEnableThres", "TODO(SPEC) unit", Some("CL.PilotEnableThres")),
];

/// spp3 part runtime IDs (published, not saved).
pub const SPP3_PART_METERS: &[ParamName] = &[
    p(7, "temperature", "degC", None),
    p(8, "OL temperature", "degC", None),
    p(36, "LP-MLS pre-clip level", "TODO(SPEC) unit", None),
    p(40, "relative-displacement minimum", "1", None),
    p(41, "relative-displacement maximum", "1", None),
    p(43, "dirty-buffer sticky flag", "bool", None),
    p(44, "block flag", "bool", None),
    p(45, "relative X limit", "%", None),
    p(46, "Convoy normalization factor", "1", None),
    p(64, "thermal gain", "dB", None),
    p(65, "displacement gain", "dB", None),
    p(66, "power gain", "dB", None),
];

/// atsp global scope (0), element 0: the 41 saved IDs (spec 2): bypass,
/// slot `t` at `1 + 13 t` (eleven continuous constants in AU order, gain
/// link, channel assignment mask), gain link all.
pub const ATSP_GLOBAL: &[ParamName] = &[
    p(0, "bypass", "bool", None),
    p(1, "slot 0 amplifier sensitivity", "V/FS", None),
    p(2, "slot 0 Rdc", "ohm", None),
    p(3, "slot 0 Rth voice coil", "K/W", None),
    p(4, "slot 0 tau voice coil", "s", None),
    p(5, "slot 0 Rth magnet", "K/W", None),
    p(6, "slot 0 tau magnet", "s", None),
    p(7, "slot 0 ambient temperature", "degC", None),
    p(8, "slot 0 temperature limit", "degC", None),
    p(9, "slot 0 attack", "ms", None),
    p(10, "slot 0 release", "ms", None),
    p(11, "slot 0 hard-limit headroom", "K", None),
    p(12, "slot 0 gain link", "bool", None),
    p(13, "slot 0 audio channel assignment", "mask", None),
    p(14, "slot 1 amplifier sensitivity", "V/FS", None),
    p(15, "slot 1 Rdc", "ohm", None),
    p(16, "slot 1 Rth voice coil", "K/W", None),
    p(17, "slot 1 tau voice coil", "s", None),
    p(18, "slot 1 Rth magnet", "K/W", None),
    p(19, "slot 1 tau magnet", "s", None),
    p(20, "slot 1 ambient temperature", "degC", None),
    p(21, "slot 1 temperature limit", "degC", None),
    p(22, "slot 1 attack", "ms", None),
    p(23, "slot 1 release", "ms", None),
    p(24, "slot 1 hard-limit headroom", "K", None),
    p(25, "slot 1 gain link", "bool", None),
    p(26, "slot 1 audio channel assignment", "mask", None),
    p(27, "slot 2 amplifier sensitivity", "V/FS", None),
    p(28, "slot 2 Rdc", "ohm", None),
    p(29, "slot 2 Rth voice coil", "K/W", None),
    p(30, "slot 2 tau voice coil", "s", None),
    p(31, "slot 2 Rth magnet", "K/W", None),
    p(32, "slot 2 tau magnet", "s", None),
    p(33, "slot 2 ambient temperature", "degC", None),
    p(34, "slot 2 temperature limit", "degC", None),
    p(35, "slot 2 attack", "ms", None),
    p(36, "slot 2 release", "ms", None),
    p(37, "slot 2 hard-limit headroom", "K", None),
    p(38, "slot 2 gain link", "bool", None),
    p(39, "slot 2 audio channel assignment", "mask", None),
    p(40, "gain link all", "bool", None),
];

/// The public scope numbers the specification uses.
pub const SCOPE_GLOBAL: u32 = 0;
pub const SCOPE_PART: u32 = 4;

pub fn lookup(table: &[ParamName], id: u32) -> Option<&ParamName> {
    table.iter().find(|p| p.id == id)
}

/// Name for a saved atsp ID (global scope only), if the specification names it.
pub fn atsp(scope: u32, id: u32) -> Option<&'static ParamName> {
    match scope {
        SCOPE_GLOBAL => lookup(ATSP_GLOBAL, id),
        _ => None,
    }
}

/// Name for a saved spp3 ID, if the specification names it.
pub fn spp3(scope: u32, id: u32) -> Option<&'static ParamName> {
    match scope {
        SCOPE_GLOBAL => lookup(SPP3_GLOBAL, id).or_else(|| lookup(SPP3_GLOBAL_METERS, id)),
        SCOPE_PART => lookup(SPP3_PART, id).or_else(|| lookup(SPP3_PART_METERS, id)),
        _ => None,
    }
}
