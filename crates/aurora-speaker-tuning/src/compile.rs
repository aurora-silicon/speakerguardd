// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compiles a [`Tuning`] into the protection core's parameter blob
//! (`sgv2_core::blob`): the fixed header and the unit's `#[repr(C)]`
//! payload.  The plugin loads the result in `instantiate()`; it never parses
//! Apple's files.
//!
//! Every payload field comes from the Mac's own tuning through a stated
//! source, and the compiled blob is decoded again with the core's own
//! `Params::from_blob` before it is returned, so a blob the engine would
//! reject is never written.
//!
//! **Refusal.** [`compile`] returns `Err`, and no blob, for any tuning a
//! protection engine must not run: a finding of error severity from
//! [`Tuning::load`] (a bypassed unit, a channel-count mismatch, a unit
//! mismatch, ...), an unverified bundle, a machine the bundle or the
//! description does not match, a channel count other than the machine's
//! amp count, or parameters the engine itself would answer with the
//! fail-safe.  Callers cannot get an unsafe blob by ignoring diagnostics.
//!
//! **Header.** The blob names the board and the amps' operating point
//! (`sgv2_core::blob::build_for`, header bytes 36 to 48), which the
//! supervisor checks against the running card; see [`crate::machine`].
//!
//! * **spp3** goes through the core's `TuningLookup`: sidecar keys first (the
//!   unit applies the sidecar after its saved state), then the saved-state
//!   IDs the specification names (`ThermalProtectionEnabled` = global 0,
//!   `PilotToneEnabled` = part 21, `CLThermalFeedbackEnabled` = part 22), and
//!   for a key the sidecar lacks, its one-to-one saved ID.
//!
//!   Unit channel `c` takes physical data source `spA<k>`, `k` from
//!   `SpeakerInputDataSourceOrder` (property 64005) when the property strip
//!   sets it, else `k = c` (the default order; SPEC-SPP3 section 2:
//!   coreaudiod sets none).  That source gives both the sense pair, ports
//!   `(2k+1, 2k)` of the `[V0, I0, V1, I1, ...]` bus, and the factory
//!   calibration, the SpTS record `spA<k>` when the record file's checksum
//!   holds.  A record no channel takes is refused.
//!
//!   The start state follows `Thermal State Override` (64007) per AID, as
//!   SPEC-SPP3 section 10 lists it: 2 is the cold state (zero rises); unset,
//!   0, 1 and any other value take the hot fallback, the steady-state split
//!   of the channel's `TemperatureLimit` over ambient (there is no hand-off
//!   record to restore on Linux).  AID8/9/14/15 leave it unset.
//! * **lsm2** reads the saved state by the specification's numeric-ID rule
//!   (section 9.7), with the model blocks at `3072 + 1024 * (3 * manager +
//!   type)`, the order the shipped presets use.  Model features the payload
//!   cannot carry are refused, never dropped.
//! * **atsp** reads the 41 global IDs of the specification's map; a graph
//!   that fixes no block for the unit gives `block_frames = 0` (host block).
//!
//! Every unit's channel `c` is fed by plugin port `c` (`source = c`): the
//! graph generator delivers the unit's input bus in the unit's own order,
//! which [`crate::tuning::Protection::lanes`] records.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::machine::{Description, Machine};
use crate::names::{SCOPE_GLOBAL, SCOPE_PART};
use crate::plist::Value;
use crate::strip::PropValue;
use crate::tuning::{Tuning, UnitKind};
use sgv2_core::blob::{build_for, machine_field, Pod, MACHINE_LEN};
use sgv2_core::{atsp, lsm2, spp3, MAX_CHANNELS};

/// `Thermal State Override` (spp3 property 64007) value of the cold start
/// state the specification defines (zero rises over ambient).
pub const SPP3_COLD_START: u32 = 2;
pub const PROP_SPP3_THERMAL_STATE_OVERRIDE: u32 = 64007;
/// `SpeakerInputDataSourceOrder` (spp3 property 64005).
pub const PROP_SPP3_SOURCE_ORDER: u32 = 64005;

/// How a compile may treat the bundle's provenance.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Provenance {
    /// Only a bundle verified against its manifest (the default).
    #[default]
    Verified,
    /// Development: a folder read in place is accepted, with a warning.
    AllowUnverified,
}

/// What the blob header carries, checked before any payload is built.
struct Bound {
    machine: [u8; MACHINE_LEN],
    description: &'static Description,
}

fn preflight(t: &Tuning, m: &Machine, provenance: Provenance, diags: &mut Diagnostics) -> Result<Bound> {
    if !t.refusals.is_empty() {
        let list: Vec<String> = t.refusals.iter().take(8).map(|d| format!("{}: {}", d.code, d.message)).collect();
        return Err(bad(format!(
            "the tuning has {} error(s), so it must not run: {}",
            t.refusals.len(),
            list.join("; ")
        )));
    }
    if !t.verified {
        match provenance {
            Provenance::Verified => return Err(bad("the bundle is not verified against a manifest")),
            Provenance::AllowUnverified => {
                diags.warn("compile-unverified", "compile", "compiling an unverified bundle (development only)")
            }
        }
    }
    if let Some(a) = m.acoustic_id {
        if a != t.acoustic_id {
            return Err(bad(format!("the bundle is {}, this Mac is AID{a}", t.aid)));
        }
    }
    let machine = machine_field(&m.board).ok_or_else(|| bad(format!("board {:?} does not fit the header", m.board)))?;
    let description = m.description().ok_or_else(|| bad(format!("no machine description for board {}", m.board)))?;
    let n = t.protection.lanes.len();
    if n != description.amps {
        return Err(bad(format!("the unit has {n} channels, the {} has {} amps", m.board, description.amps)));
    }
    Ok(Bound { machine, description })
}

/// Hundredths of a dB between two voltages.
fn centi_db(a_mv: f64, b_mv: f64) -> f64 {
    2000.0 * (a_mv / b_mv).log10()
}

fn bad(message: impl Into<String>) -> Error {
    Error::format("compile", message)
}

fn sample_rate(t: &Tuning) -> Result<u32> {
    let from_db = t.selection.sample_rate.and_then(|r| u32::try_from(r).ok());
    let from_fs = t.selection.substitutions.get("fs").and_then(|v| v.parse::<u32>().ok());
    match (from_db, from_fs) {
        (Some(a), Some(b)) if a != b => Err(bad(format!("database sample rate {a} disagrees with fs {b}"))),
        (Some(a), _) | (None, Some(a)) if a > 0 => Ok(a),
        _ => Err(bad("the sample rate is unknown (no DSP database in the bundle)")),
    }
}

fn channels(t: &Tuning) -> Result<usize> {
    let n = t.protection.lanes.len();
    if n == 0 || n > MAX_CHANNELS {
        return Err(bad(format!("{n} unit channels; the engine supports 1..={MAX_CHANNELS}")));
    }
    Ok(n)
}

/// The compiled blob, ready to publish.
#[derive(Debug)]
pub struct Compiled {
    pub unit: UnitKind,
    pub channels: u32,
    pub sense_channels: u32,
    pub sample_rate: u32,
    /// Header fields: the board and the operating point.
    pub board: String,
    pub full_scale_mv: u32,
    pub blob: Vec<u8>,
}

/// Compiles a verified tuning for machine `m`.  `Err` for anything the
/// engine must not run (see the module documentation).
pub fn compile(t: &Tuning, m: &Machine, diags: &mut Diagnostics) -> Result<Compiled> {
    compile_with(t, m, Provenance::Verified, diags)
}

pub fn compile_with(t: &Tuning, m: &Machine, provenance: Provenance, diags: &mut Diagnostics) -> Result<Compiled> {
    let b = preflight(t, m, provenance, diags)?;
    match t.protection.kind {
        UnitKind::Spp3 => compile_spp3(t, &b, diags),
        UnitKind::Lsm2 => compile_lsm2(t, &b, diags),
        UnitKind::Atsp => compile_atsp(t, &b, diags),
    }
}

// ------------------------------------------------------------------ spp3

struct Spp3Lookup<'a> {
    t: &'a Tuning,
    sidecar: &'a crate::spp3::Sidecar,
    rate: u32,
    /// Physical data source `spA<k>` of each unit channel.
    order: Vec<u32>,
}

/// `spA<k>`, the data-source ID of physical speaker source `k`.
pub fn data_source(k: u32) -> Option<FourCc> {
    (k < 10).then(|| FourCc::from_bytes([b's', b'p', b'A', b'0' + k as u8]))
}

fn source_index(id: FourCc) -> Option<u32> {
    match id.bytes() {
        [b's', b'p', b'A', d @ b'0'..=b'9'] => Some(u32::from(d - b'0')),
        _ => None,
    }
}

/// The physical source of each of the `n` unit channels: property 64005
/// when the strip sets it, else the default order `0..n`.  The property is
/// read as an array of data-source IDs (`spA<k>`, as four-character strings
/// or integers), one per unit channel in channel order; any other form is
/// refused rather than guessed.  TODO(SPEC): the property's encoding; no
/// shipped strip sets it.
pub fn source_order(t: &Tuning, n: usize) -> Result<Vec<u32>> {
    let Some(p) = t.protection.strip_properties.iter().find(|p| p.number == PROP_SPP3_SOURCE_ORDER) else {
        return Ok((0..n as u32).collect());
    };
    let what = "SpeakerInputDataSourceOrder (64005)";
    let items = match &p.value {
        PropValue::Other { value: Value::Array(a), .. } => a,
        v => return Err(bad(format!("{what} is {v:?}, not an array of data-source IDs"))),
    };
    if items.len() != n {
        return Err(bad(format!("{what} names {} sources for {n} channels", items.len())));
    }
    let mut order = Vec::with_capacity(n);
    for it in items {
        let id = match it {
            Value::String(s) => FourCc::from_str_exact(s),
            Value::Integer(v) => u32::try_from(*v).ok().map(FourCc),
            _ => None,
        };
        let k = id
            .and_then(source_index)
            .filter(|k| (*k as usize) < n)
            .ok_or_else(|| bad(format!("{what} entry {it:?} is not a data source spA0..spA{}", n - 1)))?;
        if order.contains(&k) {
            return Err(bad(format!("{what} names spA{k} twice")));
        }
        order.push(k);
    }
    Ok(order)
}

impl Spp3Lookup<'_> {
    /// The factory calibration record of unit channel `ch`: the SpTS record
    /// of the channel's data source.  `None` (open loop) without a valid
    /// record file or a matching record.
    fn calibration(&self, ch: usize) -> Option<&crate::spts::Record> {
        let s = self.t.spts.as_ref().filter(|s| s.checksum_valid)?;
        s.by_source(data_source(*self.order.get(ch)?)?)
    }
}

impl spp3::TuningLookup for Spp3Lookup<'_> {
    fn channels(&self) -> usize {
        self.sidecar.num_channels
    }
    fn sample_rate(&self) -> Option<u32> {
        Some(self.rate)
    }
    fn block_frames(&self) -> Option<u32> {
        self.t.protection.block_size.filter(|b| *b > 0.0 && b.fract() == 0.0).map(|b| b as u32)
    }
    fn global(&self, key: &str) -> Option<f64> {
        if let Some(v) = self.sidecar.globals.value(key) {
            return Some(v);
        }
        let id = match key {
            "ThermalProtectionEnabled" => 0,
            _ => return None,
        };
        self.t.protection.state.section(SCOPE_GLOBAL, 0)?.get(id).map(f64::from)
    }
    fn channel(&self, ch: usize, key: &str) -> Option<f64> {
        let c = self.sidecar.channels.get(ch)?;
        match key {
            "SpeakerGroup" => return c.speaker_group.map(|g| g as f64),
            "IgnoreTelemetry" => return c.ignore_telemetry.map(|b| b as u8 as f64),
            // Seeded after the lookup (see `start_state`).
            "sgv2.InitialThetaVc" | "sgv2.InitialThetaMg" => return Some(0.0),
            "sgv2.CalPresent" => return Some(self.calibration(ch).is_some() as u8 as f64),
            "sgv2.CalR" => return self.calibration(ch).map(|r| r.r_cal_ohm),
            "sgv2.CalT" => return self.calibration(ch).map(|r| r.t_cal_c),
            _ => {}
        }
        let saved = |id: u32| self.t.protection.state.section(SCOPE_PART, ch as u32)?.get(id).map(f64::from);
        // Saved-state-only per-channel values, by the specification's names.
        let saved_only = match key {
            "PilotToneEnabled" => Some(21),
            "CLThermalFeedbackEnabled" => Some(22),
            _ => None,
        };
        if let Some(id) = saved_only {
            return saved(id);
        }
        // The sidecar applies after the saved state, so it wins; a key the
        // sidecar lacks falls back to its one-to-one saved ID.
        c.ol_thermal
            .value(key)
            .or_else(|| c.ol_disp.value(key))
            .or_else(|| c.cl.as_ref().and_then(|l| l.value(key)))
            .or_else(|| {
                let n = crate::names::SPP3_PART
                    .iter()
                    .find(|n| n.sidecar.and_then(|s| s.split_once('.')).is_some_and(|(_, k)| k == key))?;
                saved(n.id)
            })
    }
    fn source(&self, ch: usize) -> Option<u32> {
        Some(ch as u32)
    }
    fn sense(&self, ch: usize) -> Option<(u32, u32)> {
        let k = *self.order.get(ch)?;
        Some((2 * k + 1, 2 * k))
    }
}

/// The start state `Thermal State Override` (64007) selects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartState {
    /// 2: zero rises over ambient.
    Cold,
    /// Unset, 0, 1 or any other value: the hot fallback.
    Hot,
}

pub fn start_state(t: &Tuning) -> Result<StartState> {
    match t.protection.strip_properties.iter().find(|p| p.number == PROP_SPP3_THERMAL_STATE_OVERRIDE).map(|p| &p.value)
    {
        None => Ok(StartState::Hot),
        Some(PropValue::UInt32(SPP3_COLD_START)) => Ok(StartState::Cold),
        Some(PropValue::UInt32(_)) => Ok(StartState::Hot),
        Some(v) => Err(bad(format!("Thermal State Override is {v:?}, not a UInt32"))),
    }
}

/// SPEC-SPP3 section 10, the hot fallback: the steady-state split of the
/// channel's ordinary limit over ambient, `P0 = (TemperatureLimit -
/// ambient) / (T_sett_vc + T_sett_mg)`, `theta_vc0 = P0 T_sett_vc`,
/// `theta_mg0 = P0 T_sett_mg` (the engine starts its feed-forward node at
/// `theta_vc0`).
fn seed_hot(p: &mut spp3::Params) -> Result<()> {
    let ambient = f64::from(p.thermal_ambient_temp);
    for (c, x) in p.ch[..p.channels as usize].iter_mut().enumerate() {
        let r = f64::from(x.t_sett_vc) + f64::from(x.t_sett_mg);
        let rise = f64::from(x.temperature_limit) - ambient;
        if !(r > 0.0 && rise.is_finite() && rise >= 0.0) {
            return Err(bad(format!("channel {c}: no hot start state (limit {rise} K over ambient, R {r} K/W)")));
        }
        let p0 = rise / r;
        x.initial_theta_vc = (p0 * f64::from(x.t_sett_vc)) as f32;
        x.initial_theta_mg = (p0 * f64::from(x.t_sett_mg)) as f32;
    }
    Ok(())
}

fn compile_spp3(t: &Tuning, b: &Bound, diags: &mut Diagnostics) -> Result<Compiled> {
    let sidecar = t.protection.spp3.as_ref().ok_or_else(|| bad("spp3 without its sidecar"))?;
    let n = channels(t)?;
    if sidecar.num_channels != n {
        return Err(bad(format!("sidecar has {} channels, the graph feeds {n}", sidecar.num_channels)));
    }
    let rate = sample_rate(t)?;
    let start = start_state(t)?;
    let order = source_order(t, n)?;
    // Every valid record must belong to a channel: one that does not means
    // the records and the channels disagree, and a guessed binding would
    // seed one speaker's closed loop with another's calibration.
    if let Some(s) = t.spts.as_ref().filter(|s| s.checksum_valid) {
        for r in &s.records {
            if !source_index(r.data_source).is_some_and(|k| order.contains(&k)) {
                return Err(bad(format!("SpTS record {} belongs to no unit channel", r.data_source)));
            }
        }
    }
    let lk = Spp3Lookup { t, sidecar, rate, order };
    let mut p = spp3::Params::from_lookup(&lk).map_err(|e| bad(format!("spp3 parameters: {e:?}")))?;
    if start == StartState::Hot {
        seed_hot(&mut p)?;
    }
    diags.info("compile-start-state", "compile", format!("Thermal State Override selects the {start:?} start state"));
    let sense = match &t.protection.telemetry {
        Some(tm) => tm.channels.unwrap_or(0),
        None => 0,
    };
    if sense != 0 && sense as usize != 2 * n {
        return Err(bad(format!("telemetry bus has {sense} channels for {n} unit channels")));
    }
    if sense == 0 {
        diags.warn("compile-no-sense", "compile", "the graph has no telemetry bus: closed loop cannot run");
    }
    // TODO(SPEC): spp3's output scaling.  Until the specification fixes it,
    // the operating point is the one the supervisor reads back from the
    // machine's amp gain; the tuning's `ag` is reported against it.
    let full_scale_mv = b.description.full_scale_mv();
    let ag_mv = 1000.0 * f64::from(p.ch[0].ag);
    if ag_mv > 0.0 && centi_db(ag_mv, f64::from(full_scale_mv)).abs() > 25.0 {
        diags.info(
            "compile-full-scale-ag",
            "compile",
            format!(
                "the tuning's ag is {:.0} mV, the machine's full scale {full_scale_mv} mV ({:+.2} dB; TODO(SPEC) spp3 output scaling)",
                ag_mv,
                centi_db(ag_mv, f64::from(full_scale_mv)) / 100.0
            ),
        );
    }
    let blob =
        build_for(sgv2_core::UnitKind::Spp3, p.channels, sense, p.sample_rate, b.machine, full_scale_mv, p.as_bytes());
    spp3::Params::from_blob(&blob).map_err(|e| bad(format!("the engine rejects the compiled blob: {e:?}")))?;
    if let Some(e) = spp3::Unit::init_error(&p) {
        return Err(bad(format!("the engine would run the fail-safe: {e:?}")));
    }
    Ok(Compiled {
        unit: UnitKind::Spp3,
        channels: p.channels,
        sense_channels: sense,
        sample_rate: p.sample_rate,
        board: b.description.board.to_string(),
        full_scale_mv,
        blob,
    })
}

// ------------------------------------------------------------------ lsm2

/// Saved-state IDs (specification section 9.7).
pub mod lsm2_ids {
    pub const ENABLE_CLOSED_LOOP_TEMPERATURE: u32 = 1;
    pub const BYPASS_SYS_ID: u32 = 2048 + 3;
    pub const CH_TUNING: u32 = 2048 + 4;
    pub const CH_SPEAKER_TYPE: u32 = 21504;
    /// Model managers within a type: impedance, admittance, displacement,
    /// velocity, acceleration.
    pub const MANAGER_ADMITTANCE: u32 = 1;
    pub const MANAGER_DISPLACEMENT: u32 = 2;
    pub fn model(manager: u32, ty: u32, k: u32) -> u32 {
        3072 + 1024 * (3 * manager + ty) + k
    }
    pub fn thermal(ty: u32, k: u32) -> u32 {
        18432 + 1024 * ty + k
    }
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
    pub const M_RDC: u32 = 1;
    pub const M_ENABLE_RES1: u32 = 2;
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
    pub const CH_VELOCITY_LIMIT0: u32 = 9;
    pub const CH_AMPLITUDE_LIMIT: u32 = 16;
    pub const CH_MAX_VELOCITY_ATTEN: u32 = 22;
    pub const CH_MAX_TEMP_ATTEN: u32 = 25;
    pub const CH_TEMP_HEADROOM: u32 = 26;
    pub const CH_POWER_AMP_MODEL: u32 = 32;
}

fn compile_lsm2(t: &Tuning, b: &Bound, diags: &mut Diagnostics) -> Result<Compiled> {
    use lsm2_ids as id;
    let n = channels(t)?;
    let rate = sample_rate(t)?;
    let g = t.protection.state.section(SCOPE_GLOBAL, 0).ok_or_else(|| bad("lsm2 saved state has no global section"))?;
    let get = |i: u32| -> Result<f32> { g.get(i).ok_or_else(|| bad(format!("lsm2 saved ID {i} is missing"))) };
    let flag = |i: u32| -> Result<u32> { Ok((get(i)? != 0.0) as u32) };
    let whole = |i: u32, what: &str| -> Result<u32> {
        let v = get(i)?;
        if v < 0.0 || v.fract() != 0.0 {
            return Err(bad(format!("{what} {v} is not a selector")));
        }
        Ok(v as u32)
    };
    let refuse = |what: &str| bad(format!("{what}: the engine's payload cannot represent it (TODO(lane B))"));
    if get(id::controller(id::C_POWER_BUDGETS_SCALE))? != 1.0 {
        return Err(refuse("PowerBudgetsScale != 1"));
    }
    if get(id::controller(id::C_USE_FAST_PEAK))? != 0.0 {
        return Err(refuse("UseFastPeakSampleControl"));
    }
    if get(id::controller(id::C_COMPENSATE_RESONATORS))? != 0.0 {
        return Err(refuse("CompensateResonators"));
    }
    let model_only = (get(id::BYPASS_SYS_ID)? == 1.0 && get(id::ENABLE_CLOSED_LOOP_TEMPERATURE)? == 0.0) as u32;

    let mut p = lsm2::Params {
        channels: n as u32,
        sample_rate: rate,
        enable_mitigation: flag(id::controller(id::C_ENABLE_MITIGATION))?,
        control_sample_peak: flag(id::controller(id::C_CONTROL_SAMPLE_PEAK))?,
        control_displacement: flag(id::controller(id::C_CONTROL_DISPLACEMENT))?,
        control_velocity: flag(id::controller(id::C_CONTROL_VELOCITY))?,
        control_power: flag(id::controller(id::C_CONTROL_POWER))?,
        control_temperature: flag(id::controller(id::C_CONTROL_TEMPERATURE))?,
        link_slow_gains: flag(id::controller(id::C_LINK_SLOW))?,
        unlink_temperature_gains: flag(id::controller(id::C_UNLINK_TEMPERATURE))?,
        model_only,
        attack_time_ms: get(id::controller(id::C_ATTACK))?,
        fast_release_time_ms: get(id::controller(id::C_FAST_RELEASE))?,
        slow_release_time_ms: get(id::controller(id::C_SLOW_RELEASE))?,
        filterbank_latency_ms: get(id::controller(id::C_FILTERBANK_LATENCY))?,
        speaker_response_lookahead_ms: get(id::controller(id::C_LOOKAHEAD))?,
        power_time_scale_s: [0.0; 4],
        power_hard_limit_w: [0.0; 4],
        power_soft_limit_pct: [0.0; 4],
        max_power_attenuation_db: get(id::amp0(id::A_MAX_ATTENUATION))?,
        low_rail_idle_power_w: 0.0,
        low_rail_power_ratio: 0.0,
        other_power_features: 0,
        // Platform runtime budgets (property 64008): the supervisor fills
        // them; the bundle has none.
        runtime_budget_present: 0,
        runtime_budget_w: [0.0; 4],
        ch: [lsm2::ChannelParams::default(); MAX_CHANNELS],
    };
    for j in 0..4u32 {
        p.power_time_scale_s[j as usize] = get(id::amp0(id::A_TIME_SCALE + j))?;
        p.power_hard_limit_w[j as usize] = get(id::amp0(id::A_HARD_LIMIT + j))?;
        p.power_soft_limit_pct[j as usize] = get(id::amp0(id::A_SOFT_LIMIT + j))?;
    }
    let mut amp_model = None;
    for c in 0..n as u32 {
        if whole(id::CH_TUNING + c, "ChTuning")? != 0 {
            return Err(refuse(&format!("Ch{c}Tuning selects a bank other than 0 (TODO(SPEC) banks)")));
        }
        let ty = whole(id::CH_SPEAKER_TYPE + c, "ChSpeakerType")?;
        if ty > 2 {
            return Err(bad(format!("Ch{c}SpeakerType {ty} is not a type A/B/C")));
        }
        if get(id::thermal(ty, id::T_ENABLE))? == 0.0 {
            return Err(bad(format!("channel {c}'s thermal type {ty} is disabled")));
        }
        for m in [id::MANAGER_ADMITTANCE, id::MANAGER_DISPLACEMENT] {
            let f = |k: u32| get(id::model(m, ty, k));
            if f(id::M_ENABLE_RES1)? == 0.0 {
                return Err(refuse(&format!("channel {c} model without resonator 1")));
            }
            for (k, what) in [
                (id::M_ENABLE_RES2, "resonator 2"),
                (id::M_ENABLE_LR2, "LR2 inductance"),
                (id::M_CREEP, "creep model"),
                (id::M_USE_COMPENSATION, "compensation"),
                (id::M_PASSIVE_RADIATOR, "passive radiator"),
            ] {
                if f(k)? != 0.0 {
                    return Err(refuse(&format!("channel {c} model enables {what}")));
                }
            }
        }
        let a = |k: u32| get(id::model(id::MANAGER_ADMITTANCE, ty, k));
        let d = |k: u32| get(id::model(id::MANAGER_DISPLACEMENT, ty, k));
        let th = |k: u32| get(id::thermal(ty, k));
        let ch = |k: u32| get(id::channel(c, k));
        let model = whole(id::channel(c, id::CH_POWER_AMP_MODEL), "PowerAmpModel")?;
        if model > 2 {
            return Err(bad(format!("channel {c} PowerAmpModel {model}")));
        }
        if amp_model.is_some_and(|m| m != model) {
            return Err(refuse("channels select different amplifier models"));
        }
        amp_model = Some(model);
        p.ch[c as usize] = lsm2::ChannelParams {
            source: c,
            volts_full_scale: ch(id::CH_VOLTS_FS)?,
            parasitic_resistance: ch(id::CH_PARASITIC)?,
            rdc: a(id::M_RDC)?,
            bl: a(id::M_BL)?,
            mms_kg: a(id::M_MMS_G)? * 1e-3,
            rms: a(id::M_RMS)?,
            kms: a(id::M_KMS)?,
            disp_bl: d(id::M_BL)?,
            disp_mms_kg: d(id::M_MMS_G)? * 1e-3,
            disp_rms: d(id::M_RMS)?,
            disp_kms: d(id::M_KMS)?,
            rth_vc: th(id::T_RTH_VC)?,
            tau_vc: th(id::T_TAU_VC)?,
            rth_mag: th(id::T_RTH_MG)?,
            tau_mag: th(id::T_TAU_MG)?,
            ambient: th(id::T_AMBIENT)?,
            amplitude_limit: ch(id::CH_AMPLITUDE_LIMIT)?,
            displacement_limit: ch(id::CH_DISPLACEMENT_LIMIT)?,
            velocity_limit0: ch(id::CH_VELOCITY_LIMIT0)?,
            max_velocity_attenuation_db: ch(id::CH_MAX_VELOCITY_ATTEN)?,
            temperature_soft_limit: ch(id::CH_TEMP_SOFT)?,
            temperature_hard_limit: ch(id::CH_TEMP_HARD)?,
            temperature_headroom: ch(id::CH_TEMP_HEADROOM)?,
            max_temperature_attenuation_db: ch(id::CH_MAX_TEMP_ATTEN)?,
        };
    }
    let model = amp_model.unwrap_or(0);
    let (idle, ratio, mid, high) = if model == 0 {
        (id::amp0(id::A_LOW_IDLE), id::amp0(id::A_LOW_RATIO), id::amp0(id::A_ENABLE_MID), id::amp0(id::A_ENABLE_HIGH))
    } else {
        // Models 1 and 2 have the same fields one slot earlier.
        let m = |k: u32| id::amp(model, k - 1);
        (m(id::A_LOW_IDLE), m(id::A_LOW_RATIO), m(id::A_ENABLE_MID), m(id::A_ENABLE_HIGH))
    };
    if get(mid)? != 0.0 || get(high)? != 0.0 {
        return Err(refuse("the amplifier model enables a mid or high rail"));
    }
    p.low_rail_idle_power_w = get(idle)?;
    p.low_rail_power_ratio = get(ratio)?;
    p.other_power_features = (get(mid)? != 0.0) as u32
        | ((get(high)? != 0.0) as u32) << 1
        | ((get(id::amp0(id::A_USE_BATTERY))? != 0.0) as u32) << 2;
    // The operating point is the tuning's VoltsFullScale (KERNEL.md 2.3),
    // one value for every channel: the amps share one gain setting.
    let vfs = p.ch[0].volts_full_scale;
    if !(vfs.is_finite() && vfs > 0.0) || p.ch[..n].iter().any(|c| (c.volts_full_scale - vfs).abs() > 1e-3 * vfs) {
        return Err(bad("the channels' VoltsFullScale differ or are not positive: no single operating point"));
    }
    let full_scale_mv = (f64::from(vfs) * 1000.0).round() as u32;
    let machine_mv = b.description.full_scale_mv();
    if centi_db(f64::from(full_scale_mv), f64::from(machine_mv)).abs() > 25.0 {
        diags.warn(
            "compile-full-scale",
            "compile",
            format!("VoltsFullScale is {full_scale_mv} mV, the machine's amp gain gives {machine_mv} mV: the supervisor will not grant"),
        );
    }
    let blob =
        build_for(sgv2_core::UnitKind::Lsm2, p.channels, 0, p.sample_rate, b.machine, full_scale_mv, p.as_bytes());
    lsm2::Params::from_blob(&blob).map_err(|e| bad(format!("the engine rejects the compiled blob: {e:?}")))?;
    if let Some(e) = lsm2::Unit::init_error(&p) {
        return Err(bad(format!("the engine would run the fail-safe: {e:?}")));
    }
    Ok(Compiled {
        unit: UnitKind::Lsm2,
        channels: p.channels,
        sense_channels: 0,
        sample_rate: p.sample_rate,
        board: b.description.board.to_string(),
        full_scale_mv,
        blob,
    })
}

// ------------------------------------------------------------------ atsp

fn compile_atsp(t: &Tuning, b: &Bound, _diags: &mut Diagnostics) -> Result<Compiled> {
    let n = channels(t)?;
    let rate = sample_rate(t)?;
    // A graph without a reblocker runs the unit on the host's block: 0.
    let block = match t.protection.block_size {
        None => 0,
        Some(b) if b > 0.0 && b.fract() == 0.0 => b as u32,
        Some(b) => return Err(bad(format!("atsp block size {b}"))),
    };
    let g = t.protection.state.section(SCOPE_GLOBAL, 0).ok_or_else(|| bad("atsp saved state has no global section"))?;
    let get = |i: u32| -> Result<f32> { g.get(i).ok_or_else(|| bad(format!("atsp saved ID {i} is missing"))) };
    let mut p = atsp::Params {
        channels: n as u32,
        sample_rate: rate,
        block_frames: block,
        bypass: (get(0)? != 0.0) as u32,
        gain_link_all: (get(40)? != 0.0) as u32,
        slot: [atsp::Slot::default(); 3],
    };
    for s in 0..3u32 {
        let b = 1 + 13 * s;
        let mask = get(b + 12)?;
        if mask < 0.0 || mask.fract() != 0.0 || mask > 65535.0 {
            return Err(bad(format!("slot {s} channel assignment {mask} is not a 16-bit mask")));
        }
        p.slot[s as usize] = atsp::Slot {
            channel_mask: mask as u32,
            gain_link: (get(b + 11)? != 0.0) as u32,
            amplifier_sensitivity: get(b)?,
            rdc: get(b + 1)?,
            rth_vc: get(b + 2)?,
            tau_vc: get(b + 3)?,
            rth_mg: get(b + 4)?,
            tau_mg: get(b + 5)?,
            ambient: get(b + 6)?,
            temperature_limit: get(b + 7)?,
            attack_ms: get(b + 8)?,
            release_ms: get(b + 9)?,
            hard_limit_headroom: get(b + 10)?,
        };
    }
    // TODO(SPEC): atsp's output scaling; the machine's amp gain meanwhile.
    let full_scale_mv = b.description.full_scale_mv();
    let blob =
        build_for(sgv2_core::UnitKind::Atsp, p.channels, 0, p.sample_rate, b.machine, full_scale_mv, p.as_bytes());
    atsp::Params::from_blob(&blob).map_err(|e| bad(format!("the engine rejects the compiled blob: {e:?}")))?;
    if let Some(e) = atsp::Unit::init_error(&p) {
        return Err(bad(format!("the engine would run the fail-safe: {e:?}")));
    }
    Ok(Compiled {
        unit: UnitKind::Atsp,
        channels: p.channels,
        sense_channels: 0,
        sample_rate: p.sample_rate,
        board: b.description.board.to_string(),
        full_scale_mv,
        blob,
    })
}
