// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The spp3 (`AUSpeakerProtectionV3`) sidecar: the version-9 property list
//! that `.propstrip` property 64003 names, and the cross-check of its values
//! against the unit's saved AudioUnit state in the strip.
//!
//! Field meanings and units are those of the behaviour specification
//! (sections 4-9 and 11); [`crate::units`] tabulates them.  The macOS unit
//! applies the sidecar *after* its saved state, so where both carry a value
//! the sidecar's is the one in force; [`cross_check`] reports disagreements.

use crate::aupreset::ParamState;
use crate::error::{Diagnostics, Error, Result};
use crate::names;
use crate::plist::{self, Dict, Value};
use serde::Serialize;

/// Sidecar schema versions this reader knows.  A new macOS release that
/// changes the schema must be reviewed before it is accepted.
pub const KNOWN_VERSIONS: &[i128] = &[9];

/// Upper bound on channels: the protection core's maximum.
pub const MAX_CHANNELS: usize = 16;

/// One field of a sidecar dictionary.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct FieldDef {
    pub key: &'static str,
    pub unit: &'static str,
    pub meaning: &'static str,
    pub kind: FieldKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum FieldKind {
    Number,
    /// Optional number: absent in some shipped sidecars of the same version.
    OptNumber,
    Bool,
    Text,
    /// An integer-valued number (written as integer or real).
    Enum,
}

macro_rules! block {
    ($(#[$m:meta])* $name:ident, $table:ident, { $($field:ident : $key:literal, $kind:ident, $unit:literal, $meaning:literal;)* }) => {
        $(#[$m])*
        #[derive(Clone, Debug, PartialEq, Serialize)]
        pub struct $name {
            $(
                #[doc = concat!("`", $key, "` (", $unit, "): ", $meaning)]
                pub $field: block!(@ty $kind),
            )*
        }

        pub const $table: &[FieldDef] = &[
            $(FieldDef { key: $key, unit: $unit, meaning: $meaning, kind: FieldKind::$kind },)*
        ];

        impl $name {
            fn from_dict(d: &Dict, ctx: &str, diags: &mut Diagnostics) -> Result<Self> {
                for k in d.keys() {
                    if !$table.iter().any(|f| f.key == k) {
                        diags.warn("spp3-unknown-key", ctx, format!("unknown key {k:?} (not in the known schema)"));
                    }
                }
                Ok($name { $($field: block!(@get $kind, d, $key, ctx),)* })
            }

            /// A field by its sidecar key, as a number (bools as 0/1).
            pub fn value(&self, key: &str) -> Option<f64> {
                match key {
                    $($key => block!(@f64 $kind, self.$field),)*
                    _ => None,
                }
            }
        }
    };
    (@ty Number) => { f64 };
    (@ty Enum) => { i64 };
    (@ty Bool) => { bool };
    (@ty OptNumber) => { Option<f64> };
    (@f64 Number, $v:expr) => { Some($v) };
    (@f64 OptNumber, $v:expr) => { $v };
    (@f64 Enum, $v:expr) => { Some($v as f64) };
    (@f64 Bool, $v:expr) => { Some(if $v { 1.0 } else { 0.0 }) };
    (@get Number, $d:expr, $key:literal, $ctx:expr) => { number($d, $key, $ctx)? };
    (@get Enum, $d:expr, $key:literal, $ctx:expr) => { enumeration($d, $key, $ctx)? };
    (@get Bool, $d:expr, $key:literal, $ctx:expr) => { boolean($d, $key, $ctx)? };
    (@get OptNumber, $d:expr, $key:literal, $ctx:expr) => {
        if $d.get($key).is_some() { Some(number($d, $key, $ctx)?) } else { None }
    };
}

fn number(d: &Dict, key: &str, ctx: &str) -> Result<f64> {
    let v = d.get(key).ok_or_else(|| Error::missing(ctx, key))?;
    let n = v.as_number().ok_or_else(|| Error::Type {
        context: ctx.into(),
        field: key.into(),
        expected: "integer or real",
        found: v.kind(),
    })?;
    if !n.is_finite() {
        return Err(Error::range(ctx, key, "is not finite"));
    }
    Ok(n)
}

fn enumeration(d: &Dict, key: &str, ctx: &str) -> Result<i64> {
    let n = number(d, key, ctx)?;
    if n.fract() != 0.0 || n.abs() > i32::MAX as f64 {
        return Err(Error::range(ctx, key, format!("{n} is not an integer")));
    }
    Ok(n as i64)
}

fn boolean(d: &Dict, key: &str, ctx: &str) -> Result<bool> {
    match d.get(key) {
        Some(Value::Bool(b)) => Ok(*b),
        Some(Value::Integer(0)) => Ok(false),
        Some(Value::Integer(1)) => Ok(true),
        Some(o) => Err(Error::Type { context: ctx.into(), field: key.into(), expected: "bool", found: o.kind() }),
        None => Err(Error::missing(ctx, key)),
    }
}

block!(
    /// `OL_thermal`: the open-loop two-node thermal model (spec sections 4-6).
    OlThermal, OL_THERMAL_FIELDS, {
    temperature_limit: "TemperatureLimit", Number, "degC", "ordinary controller target temperature";
    hard_temp_limit_headroom: "HardTempLimitHeadroom", Number, "K", "margin above the limit that enters the 20 dB panic path (not hysteresis)";
    t_sett_vc: "T_sett_vc", Number, "K/W", "voice-coil thermal resistance (coil above magnet)";
    t_sett_mg: "T_sett_mg", Number, "K/W", "magnet thermal resistance (magnet above ambient)";
    tau_tvc: "tau_Tvc", Number, "s", "voice-coil thermal time constant";
    tau_tmg: "tau_Tmg", Number, "s", "magnet thermal time constant";
    reb_ref: "Reb_ref", Number, "ohm", "reference voice-coil DC resistance";
    rshunt: "Rshunt", Number, "ohm", "series/parasitic resistance in the drive path";
    thermal_ff_speedup_factor: "ThermalFFSpeedupFactor", Number, "1", "feed-forward predictor factor; TODO(SPEC) exact use";
    rampout: "Rampout", Number, "TODO(SPEC)", "ramp-out; zero or fractional in shipped sidecars; nonzero behaviour TODO(SPEC)";
});

block!(
    /// `OL_disp`: the lumped mechanical model (spec section 8).  Inactive in
    /// every shipped laptop preset, but `ag` is also the thermal power model's
    /// amplifier volts at full scale.
    OlDisp, OL_DISP_FIELDS, {
    mt: "mt", Number, "kg", "moving mass";
    ct: "ct", Number, "N*s/m", "mechanical damping";
    kt: "kt", Number, "N/m", "stiffness";
    phi: "phi", Number, "N/A", "force factor";
    ag: "ag", Number, "V/FS", "amplifier volts at digital full scale (used by thermal power)";
    g_bw: "g_bw", Number, "1", "model bandwidth gain";
    q_d: "Q_d", Number, "1", "displacement detector/controller Q";
    x_lim: "x_lim", Number, "TODO(SPEC)", "excursion limit; unit conversion open";
});

block!(
    /// `CL`: closed-loop resistance thermometry from V/I sense (spec section 7).
    ClosedLoop, CL_FIELDS, {
    vmon_scale: "VMONScale", Number, "V/FS", "telemetry voltage scale";
    imon_scale: "IMONScale", Number, "A/FS", "telemetry current scale";
    v_pwr_thres: "VPwrThres", Number, "TODO(SPEC)", "voltage power floor for accepting a window (mean-square vs RMS open)";
    i_pwr_thres: "IPwrThres", Number, "TODO(SPEC)", "current power floor for accepting a window (mean-square vs RMS open)";
    reb_range_min: "RebRangeMin", Number, "ohm", "lowest acceptable resistance";
    reb_range_max: "RebRangeMax", Number, "ohm", "highest acceptable resistance";
    cal_temp_range_min: "CalTempRangeMin", Number, "degC", "lowest acceptable calibration temperature";
    cal_temp_range_max: "CalTempRangeMax", Number, "degC", "highest acceptable calibration temperature";
    a_t_20c: "a_t_20C", Number, "1/K", "resistance temperature coefficient at 20 degC";
    a_t_35c: "a_t_35C", Number, "1/K", "resistance temperature coefficient at 35 degC";
    rdc_scale: "RdcScale", Number, "1", "scale on the measured in-phase resistance";
    thermal_measurement_method: "ThermalMeasurementMethod", Enum, "enum", "0 None, 1 Pilot Tone, 2 LP MLS, 3 Open Loop";
    pilot_freq: "PilotFreq", Number, "Hz", "pilot-tone frequency";
    pilot_ampl_hi_db: "PilotAmplHi_dB", Number, "dB", "pilot high level";
    pilot_ampl_lo_db: "PilotAmplLo_dB", Number, "dB", "pilot low level";
    pilot_upper_thres: "PilotUpperThres", Number, "TODO(SPEC)", "content threshold (upper) for the pilot fade";
    pilot_lower_thres: "PilotLowerThres", Number, "TODO(SPEC)", "content threshold (lower) for the pilot fade";
    pilot_decay_time: "PilotDecayTime", Number, "s", "pilot fade stage 1";
    pilot_decay_time_stage2: "PilotDecayTimeStage2", OptNumber, "s", "pilot fade stage 2; absent on some AIDs (TODO(SPEC) behaviour when absent)";
    pilot_enable_thres: "PilotEnableThres", OptNumber, "TODO(SPEC)", "pilot enable threshold; absent on some AIDs (TODO(SPEC) behaviour when absent)";
    temp_sense_window_time: "TempSenseWindowTime", Number, "s", "V/I collection window";
    temp_sense_smoothing_tau: "TempSenseSmoothingTau", Number, "s", "smoothing of accepted temperature";
    tlim_err_decay_time: "TlimErrDecayTime", Number, "s", "decay of stale TlimAdj error toward zero";
    tlim_adj_cap: "TlimAdjCap", Number, "K", "cap on positive TlimAdj relaxation";
    lpmls_lower_corner: "LPMLSLowerCorner", Number, "TODO(SPEC)", "LP-MLS lower corner (method 2 only)";
    lpmls_pre_gain: "LPMLSPreGain", Number, "1", "LP-MLS pre gain (method 2 only)";
    lpmls_post_gain: "LPMLSPostGain", Number, "1", "LP-MLS post gain (method 2 only)";
    displacement_protection_type: "DisplacementProtectionType", Enum, "enum", "0 open loop; others TODO(SPEC)";
});

block!(
    /// Global sidecar scalars (spec sections 6, 8, 12).
    Globals, GLOBAL_FIELDS, {
    thermal_ambient_temp: "Thermal_AmbientTemp", Number, "degC", "ambient; the model's zero state";
    thermal_gain_attack_time_s: "Thermal_GainAttackTime_s", Number, "s", "attenuation attack";
    thermal_gain_release_time_s: "Thermal_GainReleaseTime_s", Number, "s", "gain recovery";
    thermal_lookahead_delay_ms: "Thermal_LookaheadDelay_ms", Number, "ms", "audio look-ahead delay";
    thermal_safe_tlim: "Thermal_SafeTlim", Number, "degC", "safe-limit state (TODO(SPEC) transition)";
    thermal_safe_tlim_offset: "Thermal_SafeTlimOffset", Number, "K", "safe-limit offset";
    thermal_safe_tlim_time_min: "Thermal_SafeTlimTimeMin", Number, "s", "safe-limit minimum time";
    enable_displacement_control: "EnableDisplacementControl", Bool, "bool", "displacement controller on";
    enable_cpms_control: "EnableCPMSControl", Bool, "bool", "CPMS power controller on";
    enable_cpms_closed_loop_control: "EnableCPMSClosedLoopControl", Bool, "bool", "CPMS closed loop on";
    disp_peak_attack_time_s: "Disp_PeakAttackTime_s", Number, "s", "displacement peak detector attack";
    disp_peak_decay_time_s: "Disp_PeakDecayTime_s", Number, "s", "displacement peak detector decay";
    disp_fb_integration_time_s: "Disp_FBIntegrationTime_s", Number, "s", "displacement feedback integration";
});

/// Keys of a `ChannelN` dictionary other than its three blocks.
pub const CHANNEL_KEYS: &[FieldDef] = &[
    FieldDef {
        key: "SpeakerName",
        unit: "",
        meaning: "diagnostic label; NOT a routing directive",
        kind: FieldKind::Text,
    },
    FieldDef { key: "SpeakerGroup", unit: "", meaning: "gain-link group (minimum linear gain)", kind: FieldKind::Enum },
    FieldDef { key: "IgnoreTelemetry", unit: "bool", meaning: "per-channel open-loop opt-out", kind: FieldKind::Bool },
];

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Channel {
    /// Index `N` of `ChannelN`: the unit's part element `N`, i.e. channel `N`
    /// of the unit's audio bus.  Bindings go by this index, never by name.
    pub index: usize,
    /// Diagnostic label only (the spec: names may disagree with routing).
    pub speaker_name: Option<String>,
    /// Gain-link group.  `None` when absent: TODO(SPEC) the named default
    /// ("group 0 linking") is stated for malformed optional sections; the
    /// engine applies it from the spec, the reader does not invent it.
    pub speaker_group: Option<i64>,
    /// `None` when absent (the default is to honour telemetry).
    pub ignore_telemetry: Option<bool>,
    pub ol_thermal: OlThermal,
    pub ol_disp: OlDisp,
    /// `None` when the channel has no `CL` block (open loop only).
    pub cl: Option<ClosedLoop>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Sidecar {
    pub version: i128,
    pub num_channels: usize,
    pub globals: Globals,
    pub channels: Vec<Channel>,
}

impl Sidecar {
    /// Parses and validates a sidecar.  Missing or mistyped essential fields
    /// are errors (the unit's 20 dB fail-safe case); unknown keys and
    /// anomalies go to `diags`.
    pub fn parse(bytes: &[u8], ctx: &str, diags: &mut Diagnostics) -> Result<Self> {
        let (root, _) = plist::parse(bytes).map_err(|e| Error::plist(ctx, e))?;
        let root = root.as_dict().ok_or_else(|| Error::format(ctx, "root is not a dictionary"))?;
        let version =
            root.get("Version").and_then(Value::as_integer).ok_or_else(|| Error::missing(ctx, "Version (integer)"))?;
        if !KNOWN_VERSIONS.contains(&version) {
            return Err(Error::version(ctx, format!("spp3 sidecar Version {version}; known: {KNOWN_VERSIONS:?}")));
        }
        let n = enumeration(root, "NumChannels", ctx)?;
        if n < 1 || n as usize > MAX_CHANNELS {
            return Err(Error::range(ctx, "NumChannels", format!("{n} not in 1..={MAX_CHANNELS}")));
        }
        let n = n as usize;
        let mut known: Vec<&str> = GLOBAL_FIELDS.iter().map(|f| f.key).collect();
        known.extend(["Version", "NumChannels", "ChannelSpecificParams"]);
        let mut gd = Dict::new();
        for (k, v) in root.iter() {
            if !known.contains(&k) {
                diags.warn("spp3-unknown-key", ctx, format!("unknown top-level key {k:?}"));
            } else if GLOBAL_FIELDS.iter().any(|f| f.key == k) {
                gd.insert_unique(k.to_string(), v.clone());
            }
        }
        let globals = Globals::from_dict(&gd, ctx, diags)?;
        let csp = crate::strip::dict(root, "ChannelSpecificParams", ctx)?;
        for k in csp.keys() {
            let ok = k
                .strip_prefix("Channel")
                .and_then(|s| s.parse::<usize>().ok())
                .is_some_and(|i| i < n && k == format!("Channel{i}"));
            if !ok {
                return Err(Error::format(
                    ctx,
                    format!("ChannelSpecificParams has {k:?}, expected Channel0..Channel{}", n - 1),
                ));
            }
        }
        let mut channels = Vec::with_capacity(n);
        for i in 0..n {
            let cctx = format!("{ctx}: Channel{i}");
            let c = crate::strip::dict(csp, &format!("Channel{i}"), &format!("{ctx}: ChannelSpecificParams"))?;
            for k in c.keys() {
                if !matches!(k, "OL_thermal" | "OL_disp" | "CL") && !CHANNEL_KEYS.iter().any(|f| f.key == k) {
                    diags.warn("spp3-unknown-key", &cctx, format!("unknown key {k:?}"));
                }
            }
            let speaker_name = match c.get("SpeakerName") {
                None => None,
                Some(Value::String(s)) => Some(s.clone()),
                Some(o) => {
                    diags.warn("spp3-malformed-optional", &cctx, format!("SpeakerName is a {}; ignored", o.kind()));
                    None
                }
            };
            let speaker_group = match c.get("SpeakerGroup") {
                None => None,
                Some(_) => Some(enumeration(c, "SpeakerGroup", &cctx)?),
            };
            if speaker_group.is_some_and(|g| g < 0) {
                return Err(Error::range(&cctx, "SpeakerGroup", "is negative"));
            }
            let ignore_telemetry = match c.get("IgnoreTelemetry") {
                None => None,
                Some(_) => Some(boolean(c, "IgnoreTelemetry", &cctx)?),
            };
            let tctx = format!("{cctx}.OL_thermal");
            let ol_thermal = OlThermal::from_dict(crate::strip::dict(c, "OL_thermal", &cctx)?, &tctx, diags)?;
            let dctx = format!("{cctx}.OL_disp");
            let ol_disp = OlDisp::from_dict(crate::strip::dict(c, "OL_disp", &cctx)?, &dctx, diags)?;
            let cl = match c.get("CL") {
                None => None,
                Some(_) => {
                    let lctx = format!("{cctx}.CL");
                    Some(ClosedLoop::from_dict(crate::strip::dict(c, "CL", &cctx)?, &lctx, diags)?)
                }
            };
            if let Some(l) = &cl {
                for (k, v) in
                    [("PilotDecayTimeStage2", l.pilot_decay_time_stage2), ("PilotEnableThres", l.pilot_enable_thres)]
                {
                    if v.is_none() {
                        diags.warn(
                            "spp3-optional-absent",
                            format!("{cctx}.CL"),
                            format!("{k} is absent: TODO(SPEC) the unit's value when absent"),
                        );
                    }
                }
            }
            channels.push(Channel { index: i, speaker_name, speaker_group, ignore_telemetry, ol_thermal, ol_disp, cl });
        }
        let s = Sidecar { version, num_channels: n, globals, channels };
        s.check_ranges(ctx)?;
        Ok(s)
    }

    /// Physical admissibility only (positive resistances and time constants,
    /// ordered ranges, known enumerations).  These are not protection
    /// margins: a value outside them cannot describe a real speaker.
    fn check_ranges(&self, ctx: &str) -> Result<()> {
        let g = &self.globals;
        let pos = |c: &str, k: &str, v: f64| {
            if v > 0.0 {
                Ok(())
            } else {
                Err(Error::range(c, k, format!("{v} must be > 0")))
            }
        };
        let nonneg = |c: &str, k: &str, v: f64| {
            if v >= 0.0 {
                Ok(())
            } else {
                Err(Error::range(c, k, format!("{v} must be >= 0")))
            }
        };
        pos(ctx, "Thermal_GainAttackTime_s", g.thermal_gain_attack_time_s)?;
        pos(ctx, "Thermal_GainReleaseTime_s", g.thermal_gain_release_time_s)?;
        nonneg(ctx, "Thermal_LookaheadDelay_ms", g.thermal_lookahead_delay_ms)?;
        nonneg(ctx, "Thermal_SafeTlimTimeMin", g.thermal_safe_tlim_time_min)?;
        for c in &self.channels {
            let t = &c.ol_thermal;
            let cc = format!("{ctx}: Channel{}.OL_thermal", c.index);
            pos(&cc, "T_sett_vc", t.t_sett_vc)?;
            pos(&cc, "T_sett_mg", t.t_sett_mg)?;
            pos(&cc, "tau_Tvc", t.tau_tvc)?;
            pos(&cc, "tau_Tmg", t.tau_tmg)?;
            pos(&cc, "Reb_ref", t.reb_ref)?;
            nonneg(&cc, "Rshunt", t.rshunt)?;
            nonneg(&cc, "HardTempLimitHeadroom", t.hard_temp_limit_headroom)?;
            if t.temperature_limit <= g.thermal_ambient_temp {
                return Err(Error::range(&cc, "TemperatureLimit", "must exceed Thermal_AmbientTemp"));
            }
            let dc = format!("{ctx}: Channel{}.OL_disp", c.index);
            pos(&dc, "ag", c.ol_disp.ag)?;
            if let Some(l) = &c.cl {
                let lc = format!("{ctx}: Channel{}.CL", c.index);
                pos(&lc, "VMONScale", l.vmon_scale)?;
                pos(&lc, "IMONScale", l.imon_scale)?;
                if l.reb_range_min >= l.reb_range_max {
                    return Err(Error::range(&lc, "RebRangeMin", "must be below RebRangeMax"));
                }
                if l.cal_temp_range_min >= l.cal_temp_range_max {
                    return Err(Error::range(&lc, "CalTempRangeMin", "must be below CalTempRangeMax"));
                }
                if !(0..=3).contains(&l.thermal_measurement_method) {
                    return Err(Error::range(&lc, "ThermalMeasurementMethod", "not in the public enumeration 0..=3"));
                }
                pos(&lc, "TempSenseWindowTime", l.temp_sense_window_time)?;
                pos(&lc, "TlimErrDecayTime", l.tlim_err_decay_time)?;
                nonneg(&lc, "TlimAdjCap", l.tlim_adj_cap)?;
            }
        }
        Ok(())
    }

    /// Gain-link groups: group number to its channels, ascending.
    pub fn groups(&self) -> Vec<(i64, Vec<usize>)> {
        let mut out: Vec<(i64, Vec<usize>)> = Vec::new();
        for c in &self.channels {
            if let Some(g) = c.speaker_group {
                match out.iter_mut().find(|(k, _)| *k == g) {
                    Some((_, v)) => v.push(c.index),
                    None => out.push((g, vec![c.index])),
                }
            }
        }
        out.sort_by_key(|(g, _)| *g);
        out
    }
}

fn sidecar_value(s: &Sidecar, channel: Option<usize>, path: &str) -> Option<f64> {
    let (block, key) = path.split_once('.').unwrap_or(("", path));
    match (channel, block) {
        (None, "") => s.globals.value(key),
        (Some(i), "OL_thermal") => s.channels.get(i)?.ol_thermal.value(key),
        (Some(i), "OL_disp") => s.channels.get(i)?.ol_disp.value(key),
        (Some(i), "CL") => s.channels.get(i)?.cl.as_ref()?.value(key),
        _ => None,
    }
}

/// Compares the sidecar with the saved AU state in the strip, for every
/// ID the specification maps one-to-one.  The sidecar wins at runtime, so a
/// difference is reported, not fatal; a missing part element or a channel
/// count disagreement is an error (the unit's channel-count fail-safe).
pub fn cross_check(s: &Sidecar, state: &ParamState, ctx: &str, diags: &mut Diagnostics) {
    let parts = state.elements(names::SCOPE_PART);
    if parts.len() != s.num_channels {
        diags.error(
            "spp3-preset-channels",
            ctx,
            format!("saved state has {} part elements, sidecar NumChannels is {}", parts.len(), s.num_channels),
        );
    }
    for (i, p) in parts.iter().enumerate() {
        if p.element as usize != i {
            diags.error("spp3-preset-elements", ctx, format!("part elements are not 0..{}", parts.len()));
            return;
        }
    }
    let close = |a: f64, b: f32| ((a as f32) - b).abs() <= f32::EPSILON * (b.abs().max(1.0)) * 4.0;
    if let Some(g) = state.global() {
        for n in names::SPP3_GLOBAL {
            if let (Some(key), Some(v)) = (n.sidecar, g.get(n.id)) {
                if let Some(sv) = sidecar_value(s, None, key) {
                    if !close(sv, v) {
                        diags.info(
                            "spp3-preset-differs",
                            ctx,
                            format!("global ID {} ({}) saved {v}, sidecar {key} {sv} (sidecar applies)", n.id, n.name),
                        );
                    }
                }
            }
        }
    } else {
        diags.warn("spp3-preset-no-global", ctx, "saved state has no global section");
    }
    for p in parts {
        let ch = p.element as usize;
        for n in names::SPP3_PART {
            if let (Some(key), Some(v)) = (n.sidecar, p.get(n.id)) {
                match sidecar_value(s, Some(ch), key) {
                    Some(sv) if !close(sv, v) => diags.info(
                        "spp3-preset-differs",
                        ctx,
                        format!("Channel{ch} ID {} ({}) saved {v}, sidecar {key} {sv} (sidecar applies)", n.id, n.name),
                    ),
                    _ => {}
                }
            }
        }
        for (id, _) in &p.params {
            if names::spp3(names::SCOPE_PART, *id).is_none() {
                diags.info("spp3-unnamed-id", ctx, format!("part ID {id} is not in the specification's map"));
            }
        }
    }
}
