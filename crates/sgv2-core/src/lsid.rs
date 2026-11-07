// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! LSID companion contract. TODO(SPEC): qualify the numerical solver before
//! enabling adaptation. The numerical solver and property-6630 coefficient
//! law are unqualified: adaptation cannot be enabled. Sensing never changes the
//! protection parameters. This object runs inside the existing LSM2 unit.

use alloc::vec::Vec;

pub const RECORDS: usize = 603;
pub const LANES: usize = 12;
pub const BANKS: usize = 3;
pub const BANK_ASSIGNMENT: [u32; 6] = [0, 1, 0, 1, 2, 2];
pub const FAMILY_ASSIGNMENT: [u32; 6] = [0, 0, 0, 0, 1, 1];
pub const SYSTEM_ASSIGNMENT: [u32; 6] = [1, 1, 1, 1, 2, 2];
/// Destination lane -> lane just after stereo duplication.
pub const PERMUTATION: [usize; 6] = [0, 3, 1, 4, 2, 5];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Schema,
    NonFinite,
    Unsupported,
    Geometry,
    Generation,
}

/// Exact saved ID geometry, independent of record order.
pub fn schema_ids() -> Vec<u32> {
    let mut ids: Vec<u32> = (0..11).chain(1024..1026).collect();
    for bank in 0..3 {
        for (base, count) in [(2048, 7), (3072, 33), (4096, 1), (5120, 22), (6144, 41), (7168, 41), (8192, 41)] {
            ids.extend((base..base + count).map(|id| id + bank * 7168));
        }
        ids.extend(23552 + bank * 1024..23560 + bank * 1024);
    }
    ids.extend(26624..26632);
    ids.sort_unstable();
    ids
}

/// Typed block boundaries; opaque fields are retained exactly and never given
/// speculative behaviour. All values, including disabled branches, are finite.
#[derive(Clone, Debug, PartialEq)]
pub struct Bank {
    pub impedance: [f32; 7],
    pub rdc: [f32; 33],
    pub pilot: f32,
    pub lr2: [f32; 22],
    pub resonance: [[f32; 41]; 3],
    pub auxiliary: [f32; 8],
}
impl Bank {
    pub fn current_scale(&self) -> f32 {
        self.impedance[0]
    }
    pub fn voltage_scale(&self) -> f32 {
        self.impedance[1]
    }
    pub fn default_rdc(&self) -> f32 {
        self.rdc[3]
    }
    pub fn fft_length(&self) -> f32 {
        self.impedance[6]
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Preset {
    pub general: [f32; 11],
    pub hold_decay: [f32; 2],
    pub banks: [Bank; BANKS],
    pub selectors: [f32; 8],
}
impl Preset {
    pub fn decode(records: &[(u32, f32)]) -> Result<Self, Error> {
        if records.len() != RECORDS {
            return Err(Error::Schema);
        }
        let mut sorted = records.to_vec();
        sorted.sort_unstable_by_key(|r| r.0);
        if !sorted.iter().map(|r| r.0).eq(schema_ids()) {
            return Err(Error::Schema);
        }
        if sorted.iter().any(|r| !r.1.is_finite()) {
            return Err(Error::NonFinite);
        }
        let get = |id| sorted.binary_search_by_key(&id, |r| r.0).map(|i| sorted[i].1).unwrap_or(0.0);
        Ok(Self {
            general: core::array::from_fn(|k| get(k as u32)),
            hold_decay: core::array::from_fn(|k| get(1024 + k as u32)),
            banks: core::array::from_fn(|b| {
                let offset = b as u32 * 7168;
                Bank {
                    impedance: core::array::from_fn(|k| get(2048 + offset + k as u32)),
                    rdc: core::array::from_fn(|k| get(3072 + offset + k as u32)),
                    pilot: get(4096 + offset),
                    lr2: core::array::from_fn(|k| get(5120 + offset + k as u32)),
                    resonance: core::array::from_fn(|r| {
                        core::array::from_fn(|k| get(6144 + offset + r as u32 * 1024 + k as u32))
                    }),
                    auxiliary: core::array::from_fn(|k| get(23552 + b as u32 * 1024 + k as u32)),
                }
            }),
            selectors: core::array::from_fn(|k| get(26624 + k as u32)),
        })
    }

    /// Canonical ID order, preserving Float32 bits including opaque fields.
    pub fn records(&self) -> Vec<(u32, f32)> {
        let mut out = Vec::with_capacity(RECORDS);
        out.extend(self.general.iter().enumerate().map(|(k, &v)| (k as u32, v)));
        out.extend(self.hold_decay.iter().enumerate().map(|(k, &v)| (1024 + k as u32, v)));
        for (b, bank) in self.banks.iter().enumerate() {
            let off = b as u32 * 7168;
            for (base, values) in [
                (2048, bank.impedance.as_slice()),
                (3072, bank.rdc.as_slice()),
                (4096, core::slice::from_ref(&bank.pilot)),
                (5120, bank.lr2.as_slice()),
                (6144, bank.resonance[0].as_slice()),
                (7168, bank.resonance[1].as_slice()),
                (8192, bank.resonance[2].as_slice()),
            ] {
                out.extend(values.iter().enumerate().map(|(k, &v)| (base + off + k as u32, v)));
            }
            out.extend(bank.auxiliary.iter().enumerate().map(|(k, &v)| (23552 + b as u32 * 1024 + k as u32, v)));
        }
        out.extend(self.selectors.iter().enumerate().map(|(k, &v)| (26624 + k as u32, v)));
        out.sort_unstable_by_key(|r| r.0);
        out
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Topology {
    /// Runtime source identity must match this graph input (NUL-padded ASCII).
    pub source: [u8; 64],
    pub lanes: [u32; LANES],
    /// Telemetry only: never part of program latency.
    pub delay_frames: [u32; LANES],
}
impl Topology {
    pub fn new(source: &str) -> Result<Self, Error> {
        if source.is_empty() || source.len() > 64 || !source.is_ascii() || source.as_bytes().contains(&0) {
            return Err(Error::Geometry);
        }
        let mut id = [0; 64];
        id[..source.len()].copy_from_slice(source.as_bytes());
        Ok(Self {
            source: id,
            lanes: core::array::from_fn(|k| k as u32),
            delay_frames: core::array::from_fn(|k| u32::from(k == 1)),
        })
    }
    pub fn source_matches(&self, source: &str) -> bool {
        Self::new(source).is_ok_and(|t| t.source == self.source)
    }
    pub fn validate(&self) -> Result<(), Error> {
        let end = self.source.iter().position(|&b| b == 0).unwrap_or(64);
        let s = core::str::from_utf8(&self.source[..end]).map_err(|_| Error::Geometry)?;
        if Self::new(s)? != *self {
            return Err(Error::Geometry);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Seeded,
    Acquiring,
    Tracking,
    Decaying,
    Reset,
}

/// A complete immutable transaction. No coefficient setter is exposed: all
/// banks and validity fields travel together under a monotonic generation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BankModel {
    pub fast_rdc: f32,
    pub slow_rdc: f32,
    pub lr2: [f32; 3],
    pub resonance: [[f32; 4]; 3],
    pub thermal_valid: bool,
    pub calibration_valid: bool,
    pub confidence: f32,
}
#[derive(Clone, Debug, PartialEq)]
pub struct ModelGeneration {
    generation: u64,
    banks: [BankModel; BANKS],
}
impl ModelGeneration {
    pub fn seeded(generation: u64, preset: &Preset) -> Result<Self, Error> {
        let banks = core::array::from_fn(|k| {
            let bank = &preset.banks[k];
            BankModel {
                fast_rdc: bank.default_rdc(),
                slow_rdc: bank.default_rdc(),
                lr2: core::array::from_fn(|i| bank.lr2[i]),
                resonance: core::array::from_fn(|r| core::array::from_fn(|i| bank.resonance[r][4 + i])),
                thermal_valid: false,
                calibration_valid: false,
                confidence: 0.0,
            }
        });
        if generation == 0
            || !banks.iter().all(|b| {
                b.fast_rdc.is_finite()
                    && b.fast_rdc > 0.0
                    && b.lr2.iter().chain(b.resonance.iter().flatten()).all(|v| v.is_finite())
            })
        {
            return Err(Error::Generation);
        }
        Ok(Self { generation, banks })
    }
    pub fn generation(&self) -> u64 {
        self.generation
    }
    pub fn banks(&self) -> &[BankModel; BANKS] {
        &self.banks
    }
}

/// Preallocated single-owner mailbox. Staging never changes the accepted
/// generation; commit is called only at a controller boundary. A future solver
/// must extend the qualified transaction before adapted models can be accepted.
#[derive(Clone, Debug)]
pub struct Handoff {
    current: ModelGeneration,
    pending: Option<ModelGeneration>,
}
impl Handoff {
    pub fn new(seed: ModelGeneration) -> Self {
        Self { current: seed, pending: None }
    }
    pub fn current(&self) -> &ModelGeneration {
        &self.current
    }
    pub fn stage(&mut self, next: ModelGeneration) -> Result<(), Error> {
        if next.generation <= self.pending.as_ref().unwrap_or(&self.current).generation
            || next.banks != self.current.banks
        {
            return Err(Error::Generation);
        }
        self.pending = Some(next);
        Ok(())
    }
    pub fn commit(&mut self) {
        if let Some(next) = self.pending.take() {
            self.current = next;
        }
    }
    pub fn discard_pending(&mut self) {
        self.pending = None;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BankState {
    pub partial_frames: usize,
    pub stale_frames: u64,
    pub fast_rdc: f32,
    pub slow_rdc: f32,
    pub valid: bool,
}

pub struct Estimator {
    preset: Preset,
    topology: Topology,
    state: State,
    bank_state: [BankState; BANKS],
    rate: u32,
    aligned_current: f32,
    valid: bool,
    model: ModelGeneration,
}
impl Estimator {
    pub fn new(preset: Preset, topology: Topology, rate: u32) -> Result<Self, Error> {
        topology.validate()?;
        if rate == 0 {
            return Err(Error::Geometry);
        }
        let model = ModelGeneration::seeded(1, &preset)?;
        let bank_state = core::array::from_fn(|b| BankState {
            partial_frames: 0,
            stale_frames: 0,
            fast_rdc: preset.banks[b].default_rdc(),
            slow_rdc: preset.banks[b].default_rdc(),
            valid: false,
        });
        Ok(Self { preset, topology, state: State::Seeded, bank_state, rate, aligned_current: 0.0, valid: false, model })
    }
    pub fn adaptation_enabled(&self) -> bool {
        false
    }
    pub fn enable_adaptation(&mut self) -> Result<(), Error> {
        Err(Error::Unsupported)
    }
    pub fn state(&self) -> State {
        self.state
    }
    pub fn sample_rate(&self) -> u32 {
        self.rate
    }
    /// The qualified host/solver owns hold timing. Preserve the stored timing
    /// records without guessing their units or decay law. Decay of a disabled
    /// estimator is the identity: its model is already the static seed.
    pub fn mark_stale(&mut self) {
        self.state = State::Decaying;
    }
    pub fn model(&self) -> &ModelGeneration {
        &self.model
    }
    pub fn partial_frames(&self) -> usize {
        self.bank_state[0].partial_frames
    }
    pub fn banks(&self) -> &[BankState; BANKS] {
        &self.bank_state
    }
    pub fn sensing_valid(&self) -> bool {
        self.valid
    }
    /// Measured power remains unavailable until V/I ordering and its meter law
    /// pass the rig; never substitute it for modeled protection power.
    pub fn measured_power(&self) -> Option<f32> {
        None
    }
    pub fn observe(&mut self, source: &str, sense: &[&[f32]], frames: usize) {
        self.valid = frames != 0
            && self.topology.source_matches(source)
            && sense.len() == LANES
            && sense.iter().all(|lane| lane.len() >= frames);
        if self.state == State::Seeded || self.state == State::Reset {
            self.state = State::Acquiring;
        }
        if self.valid {
            for (f, &current) in sense[1][..frames].iter().enumerate() {
                let delayed = self.aligned_current;
                self.aligned_current = current;
                for pair in 0..6 {
                    let bank = &self.preset.banks[BANK_ASSIGNMENT[pair] as usize];
                    let v = sense[2 * pair][f] * bank.voltage_scale();
                    let i = if pair == 0 { delayed } else { sense[2 * pair + 1][f] } * bank.current_scale();
                    if !v.is_finite() || !i.is_finite() || v == 0.0 || i == 0.0 {
                        self.valid = false;
                    }
                }
            }
        }
        // No solver runs, so there can be no accepted adaptive window. Retain
        // static coefficients while stale state decays; do not decay protection.
        for (bank, state) in self.preset.banks.iter().zip(&mut self.bank_state) {
            state.stale_frames = state.stale_frames.saturating_add(frames as u64);
            state.valid = self.valid;
            let fft = bank.fft_length() as usize;
            state.partial_frames =
                if self.valid && fft > 0 { state.partial_frames.saturating_add(frames) % fft } else { 0 };
        }
    }

    pub fn reset(&mut self) {
        for (b, state) in self.bank_state.iter_mut().enumerate() {
            state.partial_frames = 0;
            state.stale_frames = 0;
            state.valid = false;
            state.fast_rdc = self.preset.banks[b].default_rdc();
            state.slow_rdc = state.fast_rdc;
        }
        self.aligned_current = 0.0;
        self.valid = false;
        self.state = State::Reset;
    }
}
