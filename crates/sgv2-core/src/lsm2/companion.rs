// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Versioned, CRC-bound companion payload. Absent on existing model-only
//! bundles, so their bytes and ABI remain unchanged. TODO(SPEC): qualify flow,
//! system-selector encoding and appended feature laws. Unqualified configurations
//! are loadable for offline work but are muted by the single governing unit.
use super::{ids, InitError, Params};
use crate::{
    blob::{BlobError, Header, Pod},
    lsid, UnitKind, MAX_CHANNELS,
};
use alloc::vec::Vec;

const MAGIC: &[u8; 8] = b"LSIDBND1";
pub const UNQUALIFIED_FLOW: u32 = 1;
pub const UNQUALIFIED_MODEL: u32 = 2;
pub const LEGACY_ESTIMATOR: u32 = 4;
pub const UNQUALIFIED_VOICING: u32 = 8;
const MAX_FLOW: usize = 1 << 20;

/// Build-matched names for the append-only 26B8125 fields. These offsets make
/// diagnostics and future feature admission explicit; they do not enable a
/// feature merely because its stored name is known.
pub mod extension {
    pub const GENERAL_SYS_ID_FFT_SIZE: u32 = 3;

    pub const M_SENSITIVITY_ENABLE: u32 = 37;
    pub const M_SENSITIVITY_MIN_GAIN_DB: u32 = 38;
    pub const M_SENSITIVITY_MAX_GAIN_DB: u32 = 39;
    pub const M_SENSITIVITY_MAX_GAIN_DIFF_DB: u32 = 40;
    pub const M_SENSITIVITY_MAX_QTS_DIFF: u32 = 41;
    pub const M_SENSITIVITY_LINK_TYPE_A: u32 = 42;
    pub const M_SENSITIVITY_LINK_TYPE_B: u32 = 43;
    pub const M_SENSITIVITY_LINK_TYPE_C: u32 = 44;
    pub const M_STATIC_CREEP_Q: u32 = 45;
    pub const M_MIMO_COMPENSATION_ENABLE: u32 = 46;
    pub const M_MIMO_TARGET_Q: u32 = 47;
    pub const M_MIMO_TARGET_MODE: u32 = 48;
    pub const M_MIMO_MAX_FC_RATIO: u32 = 49;
    pub const M_MIMO_MAX_Q_RATIO: u32 = 50;
    pub const M_MIMO_SD_CM2: u32 = 51;
    pub const M_MIMO_VBOX_CM3: u32 = 52;

    pub const T_NUM_PARALLEL_SPEAKERS: u32 = 16;
    pub const T_RDC_TOLERANCE_PERCENT: u32 = 17;
    pub const PILOT_TUNING_RESERVED: u32 = 21_512;

    pub const CONTROLLER_DISPLACEMENT_RELEASE: u32 = 23;
    pub const CONTROLLER_VELOCITY_RELEASE: u32 = 24;
    pub const CONTROLLER_ADAPTIVE_RELEASE_ENABLE: u32 = 25;
    pub const CONTROLLER_NUM_POWER_GROUPS: u32 = 26;
    pub const CONTROLLER_TEMPERATURE_RATE_FACTOR: u32 = 27;
    pub const CHANNEL_POWER_GROUP_NUMBER: u32 = 33;

    pub const POWER_GROUP_TIME_SCALE: core::ops::Range<u32> = 0..4;
    pub const POWER_GROUP_HARD_LIMIT: core::ops::Range<u32> = 4..8;
    pub const POWER_GROUP_SOFT_LIMIT: core::ops::Range<u32> = 8..12;
}

pub fn base_ids() -> Vec<u32> {
    let mut out: Vec<u32> = (0..3).chain(1024..1033).chain(2048..2062).collect();
    for b in 0..15 {
        out.extend(3072 + 1024 * b..3109 + 1024 * b);
    }
    for t in 0..3 {
        out.extend(18432 + 1024 * t..18448 + 1024 * t);
    }
    out.extend(21504..21512);
    out.extend(22528..22537);
    out.extend(23552..23561);
    out.extend(1048576..1048599);
    out.extend(1049600..1049637);
    out.extend(1050624..1050645);
    out.extend(1051648..1051669);
    for c in 0..8 {
        out.extend(1058816 + 1024 * c..1058849 + 1024 * c);
    }
    out
}
/// Exact append map; arbitrary new IDs are never treated as disabled fields.
pub fn extension_ids() -> Vec<u32> {
    let mut out = alloc::vec![3];
    for b in 0..15 {
        out.extend(3109 + 1024 * b..3125 + 1024 * b);
    }
    for t in 0..3 {
        out.extend(18448 + 1024 * t..18450 + 1024 * t);
    }
    out.push(21512);
    out.extend(24576..24584);
    for b in [25600, 26624, 27648] {
        out.extend(b..b + 4);
    }
    out.extend(1048599..1048604);
    for c in 0..8 {
        out.push(1058849 + 1024 * c);
    }
    out.extend(1067008..1067020);
    out.extend(1068032..1068044);
    out
}
pub fn schema_ids() -> Vec<u32> {
    let mut ids = base_ids();
    ids.extend(extension_ids());
    ids.sort_unstable();
    ids
}
pub fn validate_records(records: &[(u32, f32)], extended: bool) -> Result<(), BlobError> {
    let mut sorted = records.to_vec();
    sorted.sort_unstable_by_key(|r| r.0);
    if sorted.iter().any(|r| !r.1.is_finite()) || sorted.windows(2).any(|w| w[0].0 == w[1].0) {
        return Err(BlobError::Unstable);
    }
    if extended {
        if !sorted.iter().map(|r| r.0).eq(schema_ids()) {
            return Err(BlobError::Length);
        }
    } else {
        if sorted.len() == 1021 {
            let mut known = base_ids();
            known.sort_unstable();
            if !sorted.iter().map(|r| r.0).eq(known) {
                return Err(BlobError::Version);
            }
        } else if sorted.len() > 1021 {
            return Err(BlobError::Length);
        }
        // Smaller legacy schemas remain opaque behind LEGACY_ESTIMATOR;
        // their omitted/renumbered fields must never be interpreted as a
        // modern model or granted audible output.
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq)]
pub struct Companion {
    /// Nonzero blocks audible release; no flag can enable adaptation.
    pub gates: u32,
    pub channels: u32,
    pub lsm2_records: Vec<(u32, f32)>,
    pub lsid_records: Vec<(u32, f32)>,
    pub topology: Option<lsid::Topology>,
    /// Complete property 6632 dictionary, without guessed flow semantics.
    pub flow: Vec<u8>,
    pub property_64004: u32,
    /// Output lane -> (side record, role: 0 woofer / 1 tweeter).
    pub flow_roles: [[u32; 2]; 6],
    /// Logical controller labels, independently of the preserved raw
    /// SystemNumber selectors whose label/base mapping is unqualified.
    pub systems: [u32; MAX_CHANNELS],
}
impl Companion {
    pub fn validate(&self) -> Result<(), BlobError> {
        if self.channels == 0
            || self.channels as usize > MAX_CHANNELS
            || self.flow.len() > MAX_FLOW
            || self.gates == 0
            || self.gates & !15 != 0
        {
            return Err(BlobError::Channels);
        }
        validate_records(&self.lsm2_records, self.channels == 6)?;
        if self.channels == 6 {
            if self.gates & UNQUALIFIED_FLOW == 0 || self.property_64004 != 1 || self.flow.is_empty() {
                return Err(BlobError::Version);
            }
            lsid::Preset::decode(&self.lsid_records).map_err(|_| BlobError::Version)?;
            self.topology.ok_or(BlobError::Channels)?.validate().map_err(|_| BlobError::Channels)?;
            if self.geometry_error().is_some() {
                return Err(BlobError::Channels);
            }
        } else {
            if self.topology.is_some() || self.gates & LEGACY_ESTIMATOR == 0 {
                return Err(BlobError::Version);
            }
            let mut records = self.lsid_records.clone();
            records.sort_unstable_by_key(|r| r.0);
            if records.is_empty()
                || records.iter().any(|r| !r.1.is_finite())
                || records.windows(2).any(|w| w[0].0 == w[1].0)
            {
                return Err(BlobError::Unstable);
            }
        }
        Ok(())
    }
    pub fn geometry_error(&self) -> Option<&'static str> {
        if self.channels != 6 {
            return None;
        }
        let get = |id| self.lsm2_records.iter().find(|r| r.0 == id).map(|r| r.1);
        if get(ids::controller(9)) != Some(2.0) {
            return Some("six-channel NumSystems differs from clean schema");
        }
        if get(ids::controller(ids::C_LINK_SLOW)) != Some(1.0) {
            return Some("six-channel slow gain linking differs from clean schema");
        }
        for c in 0..8 {
            let system = if c < 6 { lsid::SYSTEM_ASSIGNMENT[c] } else { 0 };
            if self.systems[c] != system {
                return Some("six-channel logical controller grouping differs from clean schema");
            }
            if c < 6 {
                if get(ids::CH_TUNING + c as u32) != Some(lsid::BANK_ASSIGNMENT[c] as f32) {
                    return Some("six-channel tuning bank differs from clean schema");
                }
                if get(ids::CH_SPEAKER_TYPE + c as u32) != Some(lsid::FAMILY_ASSIGNMENT[c] as f32) {
                    return Some("six-channel speaker family differs from clean schema");
                }
            }
        }
        if self.flow_roles != Self::six_flow_roles() {
            return Some("six-channel flow role map differs from clean schema");
        }
        None
    }
    pub fn six_systems() -> [u32; MAX_CHANNELS] {
        [1, 1, 1, 1, 2, 2, 0, 0]
    }
    pub fn six_flow_roles() -> [[u32; 2]; 6] {
        [[0, 0], [1, 0], [0, 0], [1, 0], [0, 1], [1, 1]]
    }
    pub fn encode(&self) -> Result<Vec<u8>, BlobError> {
        self.validate()?;
        let mut out = MAGIC.to_vec();
        let push = |out: &mut Vec<u8>, v: u32| out.extend_from_slice(&v.to_le_bytes());
        for v in [
            self.gates,
            self.channels,
            self.lsm2_records.len() as u32,
            self.lsid_records.len() as u32,
            self.flow.len() as u32,
            self.property_64004,
            u32::from(self.topology.is_some()),
        ] {
            push(&mut out, v);
        }
        for role in self.flow_roles {
            for v in role {
                push(&mut out, v);
            }
        }
        for v in self.systems {
            push(&mut out, v);
        }
        if let Some(t) = self.topology {
            out.extend_from_slice(&t.source);
            for v in t.lanes.into_iter().chain(t.delay_frames) {
                push(&mut out, v);
            }
        }
        for &(id, v) in self.lsm2_records.iter().chain(&self.lsid_records) {
            push(&mut out, id);
            push(&mut out, v.to_bits());
        }
        out.extend_from_slice(&self.flow);
        Ok(out)
    }
    pub fn decode(bytes: &[u8]) -> Result<Self, BlobError> {
        struct Reader<'a>(&'a [u8]);
        impl<'a> Reader<'a> {
            fn take(&mut self, n: usize) -> Result<&'a [u8], BlobError> {
                let part = self.0.get(..n).ok_or(BlobError::Short)?;
                self.0 = &self.0[n..];
                Ok(part)
            }
            fn word(&mut self) -> Result<u32, BlobError> {
                Ok(u32::from_le_bytes(self.take(4)?.try_into().map_err(|_| BlobError::Short)?))
            }
            fn records(&mut self, n: usize) -> Result<Vec<(u32, f32)>, BlobError> {
                let raw = self.take(n.checked_mul(8).ok_or(BlobError::Length)?)?;
                Ok(raw
                    .chunks_exact(8)
                    .map(|c| {
                        (u32::from_le_bytes(c[..4].try_into().unwrap()), f32::from_le_bytes(c[4..].try_into().unwrap()))
                    })
                    .collect())
            }
        }
        let mut r = Reader(bytes);
        if r.take(8)? != MAGIC {
            return Err(BlobError::Magic);
        }
        let gates = r.word()?;
        let channels = r.word()?;
        let n = r.word()? as usize;
        let m = r.word()? as usize;
        let f = r.word()? as usize;
        if n > 1326 || m > lsid::RECORDS || f > MAX_FLOW {
            return Err(BlobError::Length);
        }
        let property_64004 = r.word()?;
        let has_topology = r.word()?;
        let mut flow_roles = [[0; 2]; 6];
        for role in &mut flow_roles {
            for v in role {
                *v = r.word()?;
            }
        }
        let mut systems = [0; MAX_CHANNELS];
        for v in &mut systems {
            *v = r.word()?;
        }
        let topology = match has_topology {
            0 => None,
            1 => {
                let source = r.take(64)?.try_into().map_err(|_| BlobError::Short)?;
                let mut lanes = [0; lsid::LANES];
                let mut delay_frames = [0; lsid::LANES];
                for v in lanes.iter_mut().chain(&mut delay_frames) {
                    *v = r.word()?;
                }
                Some(lsid::Topology { source, lanes, delay_frames })
            }
            _ => return Err(BlobError::Version),
        };
        let lsm2_records = r.records(n)?;
        let lsid_records = r.records(m)?;
        let flow = r.take(f)?.to_vec();
        if !r.0.is_empty() {
            return Err(BlobError::Length);
        }
        let out =
            Self { gates, channels, lsm2_records, lsid_records, topology, flow, property_64004, flow_roles, systems };
        out.validate()?;
        Ok(out)
    }
}

/// Decode the whole payload as one transaction. Parameters alone may not be
/// extracted from a gated bundle and used to bypass the gate.
pub fn decode_blob(bytes: &[u8]) -> Result<(Params, Option<Companion>), InitError> {
    let (h, payload) = Header::parse(bytes).map_err(InitError::Blob)?;
    if h.unit != UnitKind::Lsm2 as u32 {
        return Err(InitError::Invalid("unit kind"));
    }
    let len = core::mem::size_of::<Params>();
    if payload.len() == len {
        return Params::from_blob(bytes).map(|p| (p, None));
    }
    let raw = payload.get(..len).ok_or(InitError::Blob(BlobError::Short))?;
    let p = Params::from_bytes(raw).map_err(InitError::Blob)?;
    let c = Companion::decode(&payload[len..]).map_err(InitError::Blob)?;
    if h.channels != p.channels
        || h.channels != c.channels
        || h.sample_rate != p.sample_rate
        || h.sense_channels != 2 * h.channels
    {
        return Err(InitError::Invalid("companion geometry"));
    }
    if p.channels == 0 || p.channels as usize > MAX_CHANNELS || p.sample_rate == 0 {
        return Err(InitError::Invalid("companion geometry"));
    }
    p.check_protects()?;
    Ok((p, Some(c)))
}
