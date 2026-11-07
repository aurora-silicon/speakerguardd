// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! LSID companion contract and clean-room numerical estimator.
//!
//! The solver produces complete, generation-tagged property-6630 candidates,
//! but the surrounding companion release gate remains mandatory. The normal
//! host path has no authenticated sense-source identity and therefore cannot
//! accept a window or apply an adaptive model. This object runs inside the
//! existing LSM2 unit; it never creates a second protection path.

#![allow(clippy::needless_range_loop)] // bank, pair, and FFT indices are part of the specified geometry

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
    Configuration,
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
    fn window_length(&self) -> Option<usize> {
        let saved = self.fft_length();
        if !saved.is_finite() || saved != libm::roundf(saved) || saved <= 0.0 {
            return None;
        }
        let raw = saved as usize;
        let length = if raw < 16 { 1usize.checked_shl(raw as u32)? } else { raw };
        ((16..=65_536).contains(&length) && length.is_power_of_two()).then_some(length)
    }
    fn desired_rate(&self, fallback: u32) -> u32 {
        let v = self.impedance[5];
        if v.is_finite() && v >= 1.0 && v <= fallback as f32 {
            libm::roundf(v) as u32
        } else {
            fallback
        }
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
    pub voice_coil_temp_c: f32,
    pub confidence: f32,
}
#[derive(Clone, Debug, PartialEq)]
pub struct ModelGeneration {
    generation: u64,
    banks: [BankModel; BANKS],
    accepted_bank_mask: u8,
    measured_power_w: Option<f32>,
    solver_validated: bool,
}
impl ModelGeneration {
    pub fn seeded(generation: u64, preset: &Preset) -> Result<Self, Error> {
        let banks = core::array::from_fn(|k| {
            let bank = &preset.banks[k];
            BankModel {
                fast_rdc: bank.default_rdc(),
                slow_rdc: bank.default_rdc(),
                lr2: core::array::from_fn(|i| bank.lr2[i]),
                resonance: core::array::from_fn(|r| core::array::from_fn(|i| bank.resonance[r][3 + i])),
                thermal_valid: false,
                calibration_valid: false,
                voice_coil_temp_c: 0.0,
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
        Ok(Self { generation, banks, accepted_bank_mask: 0, measured_power_w: None, solver_validated: false })
    }
    pub fn generation(&self) -> u64 {
        self.generation
    }
    pub fn banks(&self) -> &[BankModel; BANKS] {
        &self.banks
    }
    pub fn accepted_bank_mask(&self) -> u8 {
        self.accepted_bank_mask
    }
    pub fn measured_power_w(&self) -> Option<f32> {
        self.measured_power_w
    }
    fn valid(&self) -> bool {
        self.generation != 0
            && self.accepted_bank_mask & !0b111 == 0
            && (!self.solver_validated || self.accepted_bank_mask != 0)
            && self.measured_power_w.is_none_or(|p| p.is_finite() && p >= 0.0)
            && self.banks.iter().all(|b| {
                b.fast_rdc.is_finite()
                    && b.fast_rdc > 0.0
                    && b.slow_rdc.is_finite()
                    && b.slow_rdc > 0.0
                    && b.lr2.iter().chain(b.resonance.iter().flatten()).all(|v| v.is_finite())
                    && b.confidence.is_finite()
                    && (0.0..=1.0).contains(&b.confidence)
                    && (!b.thermal_valid || b.voice_coil_temp_c.is_finite())
            })
    }
}

/// Preallocated single-owner mailbox. Staging never changes the accepted
/// generation; commit is called only at a controller boundary. The mailbox is
/// the clean internal representation of property 6630. It deliberately does
/// not claim the native binary serialization.
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
        let reference = self.pending.as_ref().unwrap_or(&self.current);
        if next.generation <= reference.generation
            || !next.valid()
            || (!next.solver_validated && next.banks != reference.banks)
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
    pub coherence: f32,
    pub fit_error: f32,
    pub valid: bool,
}

#[derive(Clone, Copy, Default)]
struct Complex {
    re: f64,
    im: f64,
}
impl Complex {
    fn add(self, other: Self) -> Self {
        Self { re: self.re + other.re, im: self.im + other.im }
    }
    fn sub(self, other: Self) -> Self {
        Self { re: self.re - other.re, im: self.im - other.im }
    }
    fn mul(self, other: Self) -> Self {
        Self { re: self.re * other.re - self.im * other.im, im: self.re * other.im + self.im * other.re }
    }
    fn conj(self) -> Self {
        Self { re: self.re, im: -self.im }
    }
    fn scale(self, scale: f64) -> Self {
        Self { re: self.re * scale, im: self.im * scale }
    }
    fn norm2(self) -> f64 {
        self.re * self.re + self.im * self.im
    }
    fn div(self, other: Self) -> Self {
        let d = other.norm2().max(1e-30);
        self.mul(other.conj()).scale(1.0 / d)
    }
}

struct Window {
    voltage: Vec<f32>,
    current: Vec<f32>,
    spectrum_v: Vec<Complex>,
    spectrum_i: Vec<Complex>,
    fill: usize,
    phase: u32,
    accepted_input_frame: u64,
}
impl Window {
    fn new(len: usize) -> Self {
        Self {
            voltage: alloc::vec![0.0; len],
            current: alloc::vec![0.0; len],
            spectrum_v: alloc::vec![Complex::default(); len],
            spectrum_i: alloc::vec![Complex::default(); len],
            fill: 0,
            phase: 0,
            accepted_input_frame: 0,
        }
    }
    fn clear_partial(&mut self) {
        self.fill = 0;
        self.phase = 0;
    }
}

#[derive(Clone, Copy)]
struct Fit {
    model: BankModel,
    coherence: f32,
    error: f32,
}

pub struct Estimator {
    preset: Preset,
    topology: Topology,
    state: State,
    bank_state: [BankState; BANKS],
    windows: [Window; BANKS],
    rate: u32,
    aligned_current: f32,
    aligned_current_valid: bool,
    valid: bool,
    model: ModelGeneration,
    measured_power: Option<f32>,
    input_frames: u64,
}
impl Estimator {
    pub fn new(preset: Preset, topology: Topology, rate: u32) -> Result<Self, Error> {
        topology.validate()?;
        if rate == 0 {
            return Err(Error::Geometry);
        }
        let model = ModelGeneration::seeded(1, &preset)?;
        let mut lengths = [0usize; BANKS];
        for b in 0..BANKS {
            lengths[b] = preset.banks[b].window_length().ok_or(Error::Configuration)?;
        }
        if preset.banks.iter().any(|b| {
            !b.current_scale().is_finite()
                || !b.voltage_scale().is_finite()
                || b.current_scale() <= 0.0
                || b.voltage_scale() <= 0.0
        }) {
            return Err(Error::Configuration);
        }
        let bank_state = core::array::from_fn(|b| BankState {
            partial_frames: 0,
            stale_frames: 0,
            fast_rdc: preset.banks[b].default_rdc(),
            slow_rdc: preset.banks[b].default_rdc(),
            coherence: 0.0,
            fit_error: f32::INFINITY,
            valid: false,
        });
        let windows = core::array::from_fn(|b| Window::new(lengths[b]));
        Ok(Self {
            preset,
            topology,
            state: State::Seeded,
            bank_state,
            windows,
            rate,
            aligned_current: 0.0,
            aligned_current_valid: false,
            valid: false,
            model,
            measured_power: None,
            input_frames: 0,
        })
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
    pub fn mark_stale(&mut self) {
        self.state = State::Decaying;
        self.valid = false;
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
    pub fn measured_power(&self) -> Option<f32> {
        self.measured_power
    }
    pub fn observe(&mut self, source: &str, sense: &[&[f32]], frames: usize) {
        let input_frame_base = self.input_frames;
        self.valid = frames != 0
            && self.topology.source_matches(source)
            && sense.len() == LANES
            && sense.iter().all(|lane| {
                lane.len() >= frames
                    && lane[..frames].iter().all(|sample| sample.is_finite())
                    && lane[..frames].iter().any(|sample| *sample != 0.0)
            });
        if self.state == State::Seeded || self.state == State::Reset {
            self.state = State::Acquiring;
        }
        let mut power_sum = 0.0f64;
        let mut power_n = 0usize;
        let mut accepted = [None; BANKS];
        if self.valid {
            for frame in 0..frames {
                let delayed = self.aligned_current;
                let delayed_valid = self.aligned_current_valid;
                self.aligned_current = sense[1][frame];
                self.aligned_current_valid = true;
                let mut sum_v = [0.0f64; BANKS];
                let mut sum_i = [0.0f64; BANKS];
                let mut count = [0u32; BANKS];
                for pair in 0..6 {
                    let b = BANK_ASSIGNMENT[pair] as usize;
                    let bank = &self.preset.banks[b];
                    let v = sense[2 * pair][frame] * bank.voltage_scale();
                    let i = if pair == 0 {
                        if !delayed_valid {
                            continue;
                        }
                        delayed
                    } else {
                        sense[2 * pair + 1][frame]
                    } * bank.current_scale()
                        * if bank.impedance[4] != 0.0 { -1.0 } else { 1.0 };
                    if !v.is_finite() || !i.is_finite() {
                        self.valid = false;
                        break;
                    }
                    sum_v[b] += v as f64;
                    sum_i[b] += i as f64;
                    count[b] += 1;
                    power_sum += (v as f64 * i as f64).max(0.0);
                    power_n += 1;
                }
                if !self.valid {
                    break;
                }
                for b in 0..BANKS {
                    if count[b] == 0 {
                        continue;
                    }
                    let desired = self.preset.banks[b].desired_rate(self.rate);
                    self.windows[b].phase = self.windows[b].phase.saturating_add(desired);
                    if self.windows[b].phase < self.rate {
                        continue;
                    }
                    self.windows[b].phase -= self.rate;
                    let k = self.windows[b].fill;
                    if k < self.windows[b].voltage.len() {
                        self.windows[b].voltage[k] = (sum_v[b] / count[b] as f64) as f32;
                        self.windows[b].current[k] = (sum_i[b] / count[b] as f64) as f32;
                        self.windows[b].fill += 1;
                    }
                    if self.windows[b].fill == self.windows[b].voltage.len() {
                        let input_frame = input_frame_base.saturating_add(frame as u64 + 1);
                        let fit = Self::fit_bank(
                            &self.preset.banks[b],
                            &mut self.windows[b],
                            self.rate,
                            desired,
                            input_frame,
                            self.bank_state[b],
                            self.model.banks[b],
                        );
                        if let Some(fit) = fit {
                            self.windows[b].accepted_input_frame = input_frame;
                            accepted[b] = Some(fit);
                        }
                        self.windows[b].fill = 0;
                    }
                }
            }
        }
        if self.valid && power_n != 0 {
            self.measured_power = Some((power_sum / (frames.max(1) as f64)) as f32);
        } else {
            self.measured_power = None;
            self.aligned_current = 0.0;
            self.aligned_current_valid = false;
            for window in &mut self.windows {
                window.clear_partial();
            }
        }

        let mut next_banks = self.model.banks;
        let mut mask = 0u8;
        for b in 0..BANKS {
            let state = &mut self.bank_state[b];
            state.partial_frames = self.windows[b].fill;
            state.stale_frames = state.stale_frames.saturating_add(frames as u64);
            state.valid = false;
            if let Some(fit) = accepted[b] {
                state.fast_rdc = fit.model.fast_rdc;
                state.slow_rdc = fit.model.slow_rdc;
                state.coherence = fit.coherence;
                state.fit_error = fit.error;
                state.stale_frames = 0;
                state.valid = true;
                next_banks[b] = fit.model;
                mask |= 1 << b;
            }
        }
        if mask != 0 {
            let next = ModelGeneration {
                generation: self.model.generation.saturating_add(1),
                banks: next_banks,
                accepted_bank_mask: mask,
                measured_power_w: self.measured_power,
                solver_validated: true,
            };
            if next.valid() {
                self.model = next;
                self.state = State::Tracking;
            }
        } else if self.valid && self.state != State::Tracking {
            self.state = State::Acquiring;
        }
        self.input_frames = input_frame_base.saturating_add(frames as u64);
        self.apply_stale_decay(frames);
    }

    fn apply_stale_decay(&mut self, frames: usize) {
        let hold = seconds_to_frames(self.preset.hold_decay[0], self.rate);
        let tau = self.preset.hold_decay[1];
        let mut decaying = false;
        for b in 0..BANKS {
            if self.bank_state[b].stale_frames <= hold {
                continue;
            }
            decaying = true;
            let default = self.preset.banks[b].default_rdc();
            let alpha = smooth_alpha(frames as u64, self.rate, tau);
            self.bank_state[b].fast_rdc += alpha * (default - self.bank_state[b].fast_rdc);
            self.bank_state[b].slow_rdc += alpha * (default - self.bank_state[b].slow_rdc);
            self.bank_state[b].valid = false;
        }
        if decaying {
            self.state = State::Decaying;
        }
    }

    fn fit_bank(
        bank: &Bank,
        window: &mut Window,
        input_rate: u32,
        desired_rate: u32,
        input_frame: u64,
        state: BankState,
        previous: BankModel,
    ) -> Option<Fit> {
        let n = window.fill;
        if n < 16 || desired_rate == 0 || desired_rate > input_rate {
            return None;
        }
        if bank.rdc[1] == 0.0 {
            return None;
        }
        if bank.auxiliary[0] != 0.0 {
            // The auxiliary block is the MIMO-coupled fit. Never collapse it
            // into the independent single-driver law below.
            return None;
        }
        // Optional laws that are represented in the saved schema but not in
        // this addendum must fail closed as a whole, never produce a partial
        // generation with silently omitted dynamics.
        if bank
            .resonance
            .iter()
            .any(|branch| branch[27] != 0.0 || branch[34] != 0.0 || branch[35] != 0.0 || branch[40] != 0.0)
        {
            return None;
        }
        let cadence = seconds_to_frames(bank.impedance[3], input_rate);
        if window.accepted_input_frame != 0 && input_frame.saturating_sub(window.accepted_input_frame) < cadence {
            return None;
        }
        for k in 0..n {
            let phase = 2.0 * core::f64::consts::PI * k as f64 / (n - 1) as f64;
            let hann = 0.5 - 0.5 * libm::cos(phase);
            window.spectrum_v[k] = Complex { re: window.voltage[k] as f64 * hann, im: 0.0 };
            window.spectrum_i[k] = Complex { re: window.current[k] as f64 * hann, im: 0.0 };
        }
        fft(&mut window.spectrum_v[..n]);
        fft(&mut window.spectrum_i[..n]);
        let high = n / 2;
        let (fit_low_hz, _, _) = fit_geometry(bank, desired_rate);
        let rdc_high_hz = if fit_low_hz > 1.0 { fit_low_hz } else { desired_rate as f64 / 16.0 };
        let mut cross = Complex::default();
        let mut vv = 0.0;
        let mut ii = 0.0;
        let mut r_num = 0.0;
        let mut r_den = 0.0;
        let mut bins = 0usize;
        for k in 1..high {
            let i = window.spectrum_i[k];
            let v = window.spectrum_v[k];
            if i.norm2() <= 1e-20 || v.norm2() <= 1e-20 {
                continue;
            }
            let c = v.mul(i.conj());
            cross = cross.add(c);
            vv += v.norm2();
            ii += i.norm2();
            let hz = k as f64 * desired_rate as f64 / n as f64;
            if hz <= rdc_high_hz {
                r_num += c.re;
                r_den += i.norm2();
            }
            bins += 1;
        }
        if bins < 2 || r_den <= 1e-20 || vv <= 1e-20 || ii <= 1e-20 {
            return None;
        }
        let coherence = (cross.norm2() / (vv * ii)).clamp(0.0, 1.0) as f32;
        let threshold = bank.rdc[2].clamp(0.0, 1.0);
        let parallel = if bank.rdc[31].is_finite() && bank.rdc[31] >= 1.0 { libm::roundf(bank.rdc[31]) } else { 1.0 };
        let raw_rdc = (r_num / r_den) as f32 * parallel;
        let (rmin, rmax) = positive_bounds(bank.rdc[4], bank.rdc[5])?;
        if coherence < threshold || !raw_rdc.is_finite() || !(rmin..=rmax).contains(&raw_rdc) {
            return None;
        }
        let tolerance = bank.rdc[32].max(0.0);
        let default = bank.default_rdc();
        let default_delta = 100.0 * (raw_rdc - default).abs() / default.abs().max(1e-12);
        if tolerance > 0.0 && default_delta > tolerance {
            return None;
        }
        let seconds = n as f32 / desired_rate as f32;
        let fast_alpha = smooth_alpha_seconds(seconds, bank.rdc[0]);
        let slow_alpha = smooth_alpha_seconds(seconds, bank.rdc[10]);
        let fast = previous.fast_rdc + fast_alpha * (raw_rdc - previous.fast_rdc);
        let delta_pct = 100.0 * (fast - previous.slow_rdc).abs() / previous.slow_rdc.abs().max(1e-12);
        let slow = if bank.rdc[12] == 0.0 || delta_pct >= bank.rdc[13].max(0.0) {
            previous.slow_rdc + slow_alpha * (fast - previous.slow_rdc)
        } else {
            previous.slow_rdc
        };
        let mut model = previous;
        model.fast_rdc = fast.clamp(rmin, rmax);
        model.slow_rdc = slow.clamp(rmin, rmax);
        let (lr2, resonance, error) =
            fit_impedance(bank, &window.spectrum_v[..n], &window.spectrum_i[..n], desired_rate, model)?;
        model.lr2 = lr2;
        model.resonance = resonance;
        let calibration = calibration_temperature(bank, model.slow_rdc);
        model.calibration_valid = calibration.is_some();
        model.thermal_valid = calibration.is_some() && bank.rdc[29] != 0.0;
        model.voice_coil_temp_c = calibration.unwrap_or(0.0);
        model.confidence = (coherence * (1.0 - error).clamp(0.0, 1.0)).clamp(0.0, 1.0);
        let changed = 100.0 * (model.fast_rdc - previous.fast_rdc).abs() / previous.fast_rdc.abs().max(1e-12);
        let required = bank.rdc[9].max(0.0);
        if state.valid && changed < required && error >= state.fit_error {
            return None;
        }
        Some(Fit { model, coherence, error })
    }

    pub fn reset(&mut self) {
        for (b, state) in self.bank_state.iter_mut().enumerate() {
            state.partial_frames = 0;
            state.stale_frames = 0;
            state.valid = false;
            state.fast_rdc = self.preset.banks[b].default_rdc();
            state.slow_rdc = state.fast_rdc;
            state.coherence = 0.0;
            state.fit_error = f32::INFINITY;
        }
        for window in &mut self.windows {
            window.clear_partial();
            window.accepted_input_frame = 0;
        }
        self.aligned_current = 0.0;
        self.aligned_current_valid = false;
        self.valid = false;
        self.measured_power = None;
        self.input_frames = 0;
        self.state = State::Reset;
    }
}

fn seconds_to_frames(seconds: f32, rate: u32) -> u64 {
    if !seconds.is_finite() || seconds <= 0.0 {
        0
    } else {
        libm::round(seconds as f64 * rate as f64) as u64
    }
}
fn smooth_alpha(frames: u64, rate: u32, tau: f32) -> f32 {
    smooth_alpha_seconds(frames as f32 / rate.max(1) as f32, tau)
}
fn smooth_alpha_seconds(seconds: f32, tau: f32) -> f32 {
    if !tau.is_finite() || tau <= 0.0 {
        1.0
    } else {
        (1.0 - libm::expf(-seconds.max(0.0) / tau)).clamp(0.0, 1.0)
    }
}
fn positive_bounds(lo: f32, hi: f32) -> Option<(f32, f32)> {
    (lo.is_finite() && hi.is_finite() && lo > 0.0 && hi >= lo).then_some((lo, hi))
}

fn fft(values: &mut [Complex]) {
    let n = values.len();
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            values.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let angle = -2.0 * core::f64::consts::PI / len as f64;
        let step = Complex { re: libm::cos(angle), im: libm::sin(angle) };
        for start in (0..n).step_by(len) {
            let mut w = Complex { re: 1.0, im: 0.0 };
            for k in 0..len / 2 {
                let even = values[start + k];
                let odd = values[start + k + len / 2].mul(w);
                values[start + k] = even.add(odd);
                values[start + k + len / 2] = even.sub(odd);
                w = w.mul(step);
            }
        }
        len <<= 1;
    }
}

fn lr2_impedance(p: [f32; 3], omega: f64) -> Complex {
    let le = p[0] as f64;
    let le2 = p[1] as f64;
    let rle2 = p[2] as f64;
    let jwle2 = Complex { re: 0.0, im: omega * le2 };
    Complex { re: 0.0, im: omega * le }
        .add(jwle2.mul(Complex { re: rle2, im: 0.0 }).div(Complex { re: rle2, im: omega * le2 }))
}
fn resonance_impedance(p: [f32; 4], omega: f64) -> Complex {
    let bl = p[0] as f64;
    let mass = p[1] as f64 * 1e-3;
    let damping = p[2] as f64;
    let stiffness = p[3] as f64;
    Complex { re: bl * bl, im: 0.0 }.div(Complex { re: damping, im: omega * mass - stiffness / omega.max(1e-12) })
}
fn model_impedance(rdc: f32, lr2: [f32; 3], resonance: [[f32; 4]; 3], enabled: [bool; 3], omega: f64) -> Complex {
    let mut z = Complex { re: rdc as f64, im: 0.0 }.add(lr2_impedance(lr2, omega));
    for r in 0..3 {
        if enabled[r] {
            z = z.add(resonance_impedance(resonance[r], omega));
        }
    }
    z
}
fn fit_impedance(
    bank: &Bank,
    voltage: &[Complex],
    current: &[Complex],
    rate: u32,
    previous: BankModel,
) -> Option<([f32; 3], [[f32; 4]; 3], f32)> {
    let mut lr2 = previous.lr2;
    let mut resonance = previous.resonance;
    let lr_enabled = bank.lr2[21] != 0.0;
    let lr_adapt = lr_enabled && bank.lr2[3] != 0.0;
    let res_enabled = core::array::from_fn(|r| bank.resonance[r][0] != 0.0);
    let effective_lr2 = if lr_enabled { lr2 } else { [0.0; 3] };
    let mut best =
        impedance_error(bank, voltage, current, rate, previous.fast_rdc, effective_lr2, resonance, res_enabled)?;
    if lr_adapt {
        let bounds = [
            positive_bounds(bank.lr2[8], bank.lr2[9])?,
            positive_bounds(bank.lr2[10], bank.lr2[11])?,
            positive_bounds(bank.lr2[12], bank.lr2[13])?,
        ];
        coordinate_fit(
            &mut lr2,
            bounds,
            bank.lr2[4],
            |candidate| {
                impedance_error(bank, voltage, current, rate, previous.fast_rdc, candidate, resonance, res_enabled)
                    .unwrap_or(f32::INFINITY)
            },
            &mut best,
        );
        let a = smooth_alpha_seconds(voltage.len() as f32 / rate as f32, bank.lr2[7]);
        for k in 0..3 {
            lr2[k] = previous.lr2[k] + a * (lr2[k] - previous.lr2[k]);
        }
        best = impedance_error(bank, voltage, current, rate, previous.fast_rdc, lr2, resonance, res_enabled)?;
    }
    for r in 0..3 {
        let p = &bank.resonance[r];
        if !res_enabled[r] || p[1] == 0.0 {
            continue;
        }
        let bounds = [
            positive_bounds(p[22], p[23])?,
            positive_bounds(p[16], p[17])?,
            positive_bounds(p[18], p[19])?,
            positive_bounds(p[20], p[21])?,
        ];
        let mut fit = resonance[r];
        let enabled = [p[24] != 0.0, p[2] != 0.0, true, true];
        let electrical = if lr_enabled { lr2 } else { [0.0; 3] };
        coordinate_fit_masked(
            &mut fit,
            bounds,
            enabled,
            p[9],
            |candidate| {
                let mut all = resonance;
                all[r] = candidate;
                impedance_error(bank, voltage, current, rate, previous.fast_rdc, electrical, all, res_enabled)
                    .unwrap_or(f32::INFINITY)
            },
            &mut best,
        );
        let a = smooth_alpha_seconds(voltage.len() as f32 / rate as f32, p[10]);
        for k in 0..4 {
            resonance[r][k] = previous.resonance[r][k] + a * (fit[k] - previous.resonance[r][k]);
        }
        best = impedance_error(bank, voltage, current, rate, previous.fast_rdc, electrical, resonance, res_enabled)?;
    }
    let mut threshold =
        if lr_enabled && bank.lr2[20].is_finite() && bank.lr2[20] > 0.0 { bank.lr2[20] } else { f32::INFINITY };
    for r in 0..3 {
        let value = bank.resonance[r][25];
        if res_enabled[r] && value.is_finite() && value > 0.0 {
            threshold = threshold.min(value);
        }
    }
    if threshold.is_finite() && best > threshold {
        return None;
    }
    Some((lr2, resonance, best.clamp(0.0, 1.0)))
}

fn coordinate_fit<const N: usize>(
    values: &mut [f32; N],
    bounds: [(f32, f32); N],
    iterations: f32,
    objective: impl Fn([f32; N]) -> f32,
    best: &mut f32,
) {
    coordinate_fit_masked(values, bounds, [true; N], iterations, objective, best)
}
fn coordinate_fit_masked<const N: usize>(
    values: &mut [f32; N],
    bounds: [(f32, f32); N],
    enabled: [bool; N],
    iterations: f32,
    objective: impl Fn([f32; N]) -> f32,
    best: &mut f32,
) {
    let rounds = if iterations.is_finite() { libm::roundf(iterations).clamp(1.0, 24.0) as usize } else { 1 };
    let mut step = 0.5f32;
    for _ in 0..rounds {
        for k in 0..N {
            if !enabled[k] {
                continue;
            }
            for scale in [1.0 - step, 1.0 + step] {
                let mut candidate = *values;
                candidate[k] = (candidate[k] * scale).clamp(bounds[k].0, bounds[k].1);
                let error = objective(candidate);
                if error < *best {
                    *best = error;
                    *values = candidate;
                }
            }
        }
        step *= 0.5;
    }
}

#[allow(clippy::too_many_arguments)] // the objective is intentionally a pure function of one complete candidate
fn impedance_error(
    bank: &Bank,
    voltage: &[Complex],
    current: &[Complex],
    rate: u32,
    rdc: f32,
    lr2: [f32; 3],
    resonance: [[f32; 4]; 3],
    enabled: [bool; 3],
) -> Option<f32> {
    let n = voltage.len();
    let (min_hz, max_hz, stride) = fit_geometry(bank, rate);
    let mut error = 0.0;
    let mut signal = 0.0;
    let mut used = 0usize;
    for k in (1..n / 2).step_by(stride) {
        let hz = k as f64 * rate as f64 / n as f64;
        if hz < min_hz || hz > max_hz || current[k].norm2() <= 1e-20 {
            continue;
        }
        let measured = voltage[k].div(current[k]);
        let predicted = model_impedance(rdc, lr2, resonance, enabled, 2.0 * core::f64::consts::PI * hz);
        error += measured.sub(predicted).norm2();
        signal += measured.norm2();
        used += 1;
    }
    (used >= 2 && signal > 1e-20).then_some(libm::sqrt(error / signal) as f32)
}

fn fit_geometry(bank: &Bank, rate: u32) -> (f64, f64, usize) {
    let mut low = f64::INFINITY;
    let mut high = 0.0f64;
    let mut stride = usize::MAX;
    if bank.lr2[21] != 0.0 {
        low = low.min(bank.lr2[5].max(1.0) as f64);
        high = high.max(bank.lr2[6] as f64);
        stride = stride.min(libm::roundf(bank.lr2[17]).max(1.0) as usize);
    }
    for branch in &bank.resonance {
        if branch[0] == 0.0 {
            continue;
        }
        low = low.min(branch[7].max(1.0) as f64);
        high = high.max(branch[8] as f64);
        stride = stride.min(libm::roundf(branch[26]).max(1.0) as usize);
    }
    if !low.is_finite() || high <= low {
        (1.0, rate as f64 * 0.45, 1)
    } else {
        (low, high.min(rate as f64 * 0.5), stride.max(1))
    }
}

fn calibration_temperature(bank: &Bank, rdc: f32) -> Option<f32> {
    let (temperature, resistance, alpha) = (bank.rdc[14], bank.rdc[15], bank.rdc[16]);
    if !temperature.is_finite()
        || !resistance.is_finite()
        || !alpha.is_finite()
        || resistance <= 0.0
        || alpha <= 0.0
        || !(bank.rdc[24]..=bank.rdc[25]).contains(&temperature)
        || !(bank.rdc[26]..=bank.rdc[27]).contains(&resistance)
    {
        return None;
    }
    let value = temperature + (rdc / resistance - 1.0) / alpha;
    value.is_finite().then_some(value)
}

#[cfg(test)]
mod numerical_tests {
    use super::*;

    #[test]
    fn synthetic_complex_impedance_exercises_lr2_and_all_resonance_branches() {
        let mut bank = Bank {
            impedance: [1.0, 1.0, 0.0, 0.0, 0.0, 48_000.0, 256.0],
            rdc: [0.0; 33],
            pilot: 0.0,
            lr2: [0.0; 22],
            resonance: [[0.0; 41]; 3],
            auxiliary: [0.0; 8],
        };
        bank.rdc[3] = 4.0;
        bank.lr2[0..3].copy_from_slice(&[0.000_2, 0.000_08, 2.5]);
        bank.lr2[3] = 1.0;
        bank.lr2[4] = 8.0;
        bank.lr2[5] = 20.0;
        bank.lr2[6] = 20_000.0;
        bank.lr2[8..14].copy_from_slice(&[0.000_05, 0.001, 0.000_02, 0.001, 0.5, 8.0]);
        bank.lr2[17] = 1.0;
        bank.lr2[20] = 1.0;
        bank.lr2[21] = 1.0;
        let resonance = [[2.0, 0.8, 0.6, 800.0], [1.6, 0.5, 0.5, 2_000.0], [1.2, 0.3, 0.4, 4_500.0]];
        for (saved, values) in bank.resonance.iter_mut().zip(resonance) {
            saved[0..3].copy_from_slice(&[1.0, 1.0, 1.0]);
            saved[3..7].copy_from_slice(&values);
            saved[9] = 8.0;
            saved[16..24].copy_from_slice(&[
                values[1] * 0.25,
                values[1] * 4.0,
                values[2] * 0.25,
                values[2] * 4.0,
                values[3] * 0.25,
                values[3] * 4.0,
                values[0] * 0.25,
                values[0] * 4.0,
            ]);
            saved[24] = 1.0;
            saved[25] = 1.0;
        }
        let previous = BankModel {
            fast_rdc: 4.0,
            slow_rdc: 4.0,
            lr2: [0.000_2, 0.000_08, 2.5],
            resonance,
            thermal_valid: false,
            calibration_valid: false,
            voice_coil_temp_c: 0.0,
            confidence: 0.0,
        };
        let mut current = [Complex::default(); 256];
        let mut voltage = [Complex::default(); 256];
        for k in 1..128 {
            current[k] = Complex { re: 1.0, im: 0.25 };
            let hz = k as f64 * 48_000.0 / 256.0;
            voltage[k] =
                model_impedance(4.0, previous.lr2, previous.resonance, [true; 3], 2.0 * core::f64::consts::PI * hz)
                    .mul(current[k]);
        }
        let (lr2, fitted, error) = fit_impedance(&bank, &voltage, &current, 48_000, previous).unwrap();
        assert!(error < 1e-5, "{error}");
        for (actual, expected) in lr2.into_iter().zip(previous.lr2) {
            assert!((actual - expected).abs() < expected.abs().max(1e-8) * 1e-4);
        }
        for (actual, expected) in fitted.into_iter().flatten().zip(previous.resonance.into_iter().flatten()) {
            assert!((actual - expected).abs() < expected.abs().max(1e-8) * 1e-4);
        }
    }
}
