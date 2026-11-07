// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! AULowFlow (`aufx/flow/appl`), a 256-frame look-ahead velocity limiter.
//!
//! Low-frequency lanes come first and their corresponding high-frequency
//! lanes follow. Each pair supplies an electromechanical velocity detector
//! and an acoustic masking detector. Only low-frequency lanes are gained;
//! every lane receives the fixed delay. `SystemVolume` changes only the
//! saved attenuation ceiling; [`Run::set_volume`] is RT-safe.
//!
//! The default (exact) build is a fixed-hop STFT gain shaper. Each 128-frame
//! hop, the velocity target from the two previously completed hops sets a
//! water-fill cap on the Hann-window magnitudes of the affected bins, and the
//! per-bin gains (immediate attack, saved release) scale the square-root Hann
//! analysis spectrum before overlap-add. A separate detector transforms a
//! `latency + 128` frame sliding history through a unit-energy Hann. It
//! applies the saved tweeter-response magnitude, reduces squared magnitudes
//! on the unit's 99-band grid, and smooths the resulting linked mask level.
//! The velocity detector independently reads current LF.
//!
//! The optional `lowflow-conservative` compile feature is deliberately not an
//! exact-law claim. It applies a fixed ceiling 6 dB below the saved
//! volume-dependent ceiling to every enabled low-frequency lane. This makes
//! the fallback independent of the unresolved detector transition and release
//! history while retaining the saved volume law and the exact lane delay.
//! Since the unit's instantaneous gain is bounded below by its saved ceiling,
//! `g_fallback <= 10^(-6/20) * g_unit` sample by sample. Disabled and HF lanes
//! remain exact delays, while muted lanes remain silent.

use crate::MAX_CHANNELS;
use alloc::{boxed::Box, vec, vec::Vec};

pub const LATENCY_FRAMES: usize = 256;
pub const RESPONSE_TAPS: usize = 1024;
pub const MAX_MODELS: usize = MAX_CHANNELS / 2;
#[cfg(not(feature = "lowflow-conservative"))]
const SPECTRAL_HOP: usize = 128;
#[cfg(not(feature = "lowflow-conservative"))]
const MASK_BANDS: usize = 99;
#[cfg(not(feature = "lowflow-conservative"))]
const MASK_DIRECT_BANDS: usize = 74;
#[cfg(not(feature = "lowflow-conservative"))]
const MASK_REFERENCE_FS: f64 = 48_000.0;
const MODEL_SCALARS: usize = 28;
const MAX_FS: f64 = 192_000.0;
const CONSERVATIVE_GUARD_DB: f64 = 6.0;
#[cfg(not(feature = "lowflow-conservative"))]
const MAX_MASK_VELOCITY: f32 = 1.2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Model {
    pub bl: f64,
    pub kms: f64,
    pub kms_front_port: f64,
    pub mms: f64,
    pub mms_front_port: f64,
    pub reb: f64,
    pub rms: f64,
    pub rms_front_port: f64,
    pub analog_gain: f64,
    pub tweeter_gain: f64,
    pub mask_band_min_hz: f64,
    pub mask_band_max_hz: f64,
    pub min_velocity: f64,
    pub min_mask_energy_db: f64,
    pub mid_velocity: f64,
    pub mid_mask_energy_db: f64,
    pub high_velocity: f64,
    pub high_mask_energy_db: f64,
    pub max_mask_energy_db: f64,
    pub mask_attack_s: f64,
    pub mask_release_s: f64,
    pub max_attenuation_db: f64,
    pub release_s: f64,
    pub min_frequency_hz: f64,
    pub max_frequency_hz: f64,
    pub mid_attenuation_db: f64,
    pub mid_level_db: f64,
    pub enabled: bool,
    pub mute: bool,
    pub pressure_response: [f64; RESPONSE_TAPS],
    pub tweeter_response: [f64; RESPONSE_TAPS],
}

impl Default for Model {
    fn default() -> Self {
        Self {
            bl: 0.0,
            kms: 0.0,
            kms_front_port: 0.0,
            mms: 0.0,
            mms_front_port: 0.0,
            reb: 0.0,
            rms: 0.0,
            rms_front_port: 0.0,
            analog_gain: 0.0,
            tweeter_gain: 0.0,
            mask_band_min_hz: 0.0,
            mask_band_max_hz: 0.0,
            min_velocity: 0.0,
            min_mask_energy_db: 0.0,
            mid_velocity: 0.0,
            mid_mask_energy_db: 0.0,
            high_velocity: 0.0,
            high_mask_energy_db: 0.0,
            max_mask_energy_db: 0.0,
            mask_attack_s: 0.0,
            mask_release_s: 0.0,
            max_attenuation_db: 0.0,
            release_s: 0.0,
            min_frequency_hz: 0.0,
            max_frequency_hz: 0.0,
            mid_attenuation_db: 0.0,
            mid_level_db: 0.0,
            enabled: false,
            mute: false,
            pressure_response: [0.0; RESPONSE_TAPS],
            tweeter_response: [0.0; RESPONSE_TAPS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    pub model_count: usize,
    pub link_channel_energies: bool,
    pub latency_frames: usize,
    /// Use the guarded whole-lane fallback because the exact sidecar model is incomplete.
    pub conservative_fallback: bool,
    pub models: [Model; MAX_MODELS],
}

impl Params {
    const MODEL_WORDS: usize = MODEL_SCALARS + 2 * RESPONSE_TAPS;
    pub const WORDS: usize = 4 + MAX_MODELS * Self::MODEL_WORDS;

    pub fn to_words(&self, out: &mut [f64]) {
        if out.len() < Self::WORDS {
            return;
        }
        out.fill(0.0);
        out[0] = self.model_count as f64;
        out[1] = f64::from(self.link_channel_energies);
        out[2] = self.latency_frames as f64;
        out[3] = f64::from(self.conservative_fallback);
        for (i, model) in self.models.iter().enumerate() {
            let at = 4 + i * Self::MODEL_WORDS;
            out[at..at + MODEL_SCALARS].copy_from_slice(&scalars(model));
            out[at + MODEL_SCALARS..at + MODEL_SCALARS + RESPONSE_TAPS].copy_from_slice(&model.pressure_response);
            out[at + MODEL_SCALARS + RESPONSE_TAPS..at + Self::MODEL_WORDS].copy_from_slice(&model.tweeter_response);
        }
    }

    pub fn from_words(words: &[f64]) -> Option<Self> {
        if words.len() != Self::WORDS || words.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let model_count = usize::try_from(words[0] as i64).ok()?;
        if words[0] != model_count as f64 || !(1..=MAX_MODELS).contains(&model_count) {
            return None;
        }
        let link_channel_energies = bool_word(words[1])?;
        let latency_frames = usize::try_from(words[2] as i64).ok()?;
        if words[2] != latency_frames as f64 || !(1..=4096).contains(&latency_frames) {
            return None;
        }
        let conservative_fallback = bool_word(words[3])?;
        let mut models = [Model::default(); MAX_MODELS];
        for (i, model) in models.iter_mut().enumerate() {
            let at = 4 + i * Self::MODEL_WORDS;
            *model = from_scalars(&words[at..at + MODEL_SCALARS])?;
            model.pressure_response.copy_from_slice(&words[at + MODEL_SCALARS..at + MODEL_SCALARS + RESPONSE_TAPS]);
            model.tweeter_response.copy_from_slice(&words[at + MODEL_SCALARS + RESPONSE_TAPS..at + Self::MODEL_WORDS]);
        }
        (models[..model_count].iter().all(valid_model) && models[model_count..].iter().all(|m| *m == Model::default()))
            .then_some(Self { model_count, link_channel_energies, latency_frames, conservative_fallback, models })
    }
}

fn scalars(m: &Model) -> [f64; MODEL_SCALARS] {
    [
        m.bl,
        m.kms,
        m.kms_front_port,
        m.mms,
        m.mms_front_port,
        m.reb,
        m.rms,
        m.rms_front_port,
        m.analog_gain,
        m.tweeter_gain,
        m.mask_band_min_hz,
        m.mask_band_max_hz,
        m.min_velocity,
        m.min_mask_energy_db,
        m.mid_velocity,
        m.mid_mask_energy_db,
        m.high_velocity,
        m.high_mask_energy_db,
        m.max_mask_energy_db,
        m.mask_attack_s,
        m.mask_release_s,
        m.max_attenuation_db,
        m.release_s,
        m.min_frequency_hz,
        m.max_frequency_hz,
        m.mid_attenuation_db,
        m.mid_level_db,
        f64::from(m.enabled) + 2.0 * f64::from(m.mute),
    ]
}

fn from_scalars(s: &[f64]) -> Option<Model> {
    let (enabled, mute) = match s[27] {
        0.0 => (false, false),
        1.0 => (true, false),
        2.0 => (false, true),
        3.0 => (true, true),
        _ => return None,
    };
    Some(Model {
        bl: s[0],
        kms: s[1],
        kms_front_port: s[2],
        mms: s[3],
        mms_front_port: s[4],
        reb: s[5],
        rms: s[6],
        rms_front_port: s[7],
        analog_gain: s[8],
        tweeter_gain: s[9],
        mask_band_min_hz: s[10],
        mask_band_max_hz: s[11],
        min_velocity: s[12],
        min_mask_energy_db: s[13],
        mid_velocity: s[14],
        mid_mask_energy_db: s[15],
        high_velocity: s[16],
        high_mask_energy_db: s[17],
        max_mask_energy_db: s[18],
        mask_attack_s: s[19],
        mask_release_s: s[20],
        max_attenuation_db: s[21],
        release_s: s[22],
        min_frequency_hz: s[23],
        max_frequency_hz: s[24],
        mid_attenuation_db: s[25],
        mid_level_db: s[26],
        enabled,
        mute,
        ..Model::default()
    })
}

fn bool_word(v: f64) -> Option<bool> {
    match v {
        0.0 => Some(false),
        1.0 => Some(true),
        _ => None,
    }
}

fn valid_model(m: &Model) -> bool {
    [
        m.bl,
        m.kms,
        m.kms_front_port,
        m.mms,
        m.mms_front_port,
        m.reb,
        m.rms,
        m.rms_front_port,
        m.analog_gain,
        m.tweeter_gain,
        m.min_velocity,
        m.mid_velocity,
        m.high_velocity,
    ]
    .iter()
    .all(|v| *v > 0.0 && v.is_finite())
        && m.mask_band_min_hz >= 0.0
        && m.mask_band_max_hz > m.mask_band_min_hz
        && m.min_mask_energy_db <= m.mid_mask_energy_db
        && m.mid_mask_energy_db <= m.high_mask_energy_db
        && m.high_mask_energy_db <= m.max_mask_energy_db
        && (0.0..=5.0).contains(&m.mask_attack_s)
        && (0.0..=5.0).contains(&m.mask_release_s)
        && (0.0..=24.0).contains(&m.max_attenuation_db)
        && (0.0..=24.0).contains(&m.mid_attenuation_db)
        && (-100.0..0.0).contains(&m.mid_level_db)
        && (0.0..=5.0).contains(&m.release_s)
        && m.min_frequency_hz >= 0.0
        && m.max_frequency_hz > m.min_frequency_hz
        && m.max_frequency_hz <= MAX_FS / 2.0
}

#[cfg(feature = "lowflow-conservative")]
#[derive(Clone, Copy)]
struct Biquad {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    z1: f64,
    z2: f64,
}

/// The cone and front port are two mechanical impedances driven by the same
/// voice-coil current. The electrical damping therefore belongs to their
/// common denominator; treating it as two independent dampers changes both
/// the cancellation notch and the onset ringing.
#[cfg(not(feature = "lowflow-conservative"))]
struct VelocityCascade {
    input_gain: f32,
    sections: [VelocityBiquad; 2],
}

#[cfg(not(feature = "lowflow-conservative"))]
#[derive(Clone, Copy, Default)]
struct VelocityBiquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

#[cfg(not(feature = "lowflow-conservative"))]
impl VelocityBiquad {
    fn from_analog(mut c: [f32; 5]) -> Self {
        // Unprewarped bilinear transform at the fixed 48 kHz model rate.
        // Intermediates are double precision and coefficient words are
        // rounded back to float before use.
        const K: f64 = 2.0 * 48_000.0;
        let b0 = f64::from(c[0]);
        let b1 = f64::from(c[1]);
        let b2 = f64::from(c[2]);
        let a1 = f64::from(c[3]);
        let a2 = f64::from(c[4]);
        let k2 = K * K;
        let d = k2 + a1 * K + a2;
        c[0] = ((b0 * k2 + b1 * K + b2) / d) as f32;
        c[1] = ((-2.0 * b0 * k2 + 2.0 * b2) / d) as f32;
        c[2] = ((b0 * k2 - b1 * K + b2) / d) as f32;
        c[3] = ((-2.0 * k2 + 2.0 * a2) / d) as f32;
        c[4] = ((k2 - a1 * K + a2) / d) as f32;
        Self { b0: c[0], b1: c[1], b2: c[2], a1: c[3], a2: c[4], z1: 0.0, z2: 0.0 }
    }

    fn run(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        if y.is_finite() {
            y
        } else {
            0.0
        }
    }

    fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }
}

#[cfg(not(feature = "lowflow-conservative"))]
impl VelocityCascade {
    fn new(model: &Model) -> Self {
        // Preserve the binary's float32 construction order.  Let Zc and Zp
        // be the cone and port mechanical impedances and e = Bl^2/Reb.  The
        // shared-current transfer is
        //
        //   Bl/Reb * s*(Zc + Zp) / (Zc*Zp + e*s*(Zc + Zp)).
        let bl = model.bl as f32;
        let reb = model.reb as f32;
        let mc = model.mms as f32;
        let mp = model.mms_front_port as f32;
        let rc = model.rms as f32;
        let rp = model.rms_front_port as f32;
        let kc = model.kms as f32;
        let kp = model.kms_front_port as f32;
        let force = bl / reb;
        let electrical = bl * force;
        let numerator = [force * (mc + mp), force * (rc + rp), force * (kc + kp)];
        let d1 = libm::fmaf(electrical, mc + mp, libm::fmaf(mc, rp, mp * rc));
        let d2 = libm::fmaf(electrical, rc + rp, libm::fmaf(kc, mp, libm::fmaf(mc, kp, rc * rp)));
        let d3 = libm::fmaf(electrical, kc + kp, libm::fmaf(kc, rp, rc * kp));
        let denominator = [mc * mp, d1, d2, d3, kc * kp];

        let zero_b = (f64::from(numerator[1]) / f64::from(numerator[0])) as f32;
        let zero_c = (f64::from(numerator[2]) / f64::from(numerator[0])) as f32;
        let mut poles = factor_quartic(denominator);
        // The companion-matrix eigensolver returns the higher-frequency pair
        // first; section ordering matters once each section rounds its state.
        poles.sort_by(|a, b| b.1.total_cmp(&a.1));
        let gain = numerator[0] / denominator[0];
        let first = VelocityBiquad::from_analog([1.0, zero_b, zero_c, poles[0].0 as f32, poles[0].1 as f32]);
        let second = VelocityBiquad::from_analog([0.0, gain, 0.0, poles[1].0 as f32, poles[1].1 as f32]);
        Self { input_gain: model.analog_gain as f32, sections: [first, second] }
    }

    fn run(&mut self, x: f32) -> f32 {
        let mut y = x * self.input_gain;
        for section in &mut self.sections {
            y = section.run(y);
        }
        y
    }

    fn reset(&mut self) {
        for section in &mut self.sections {
            section.reset();
        }
    }
}

#[cfg(not(feature = "lowflow-conservative"))]
fn factor_quartic(c: [f32; 5]) -> [(f64, f64); 2] {
    let lead = f64::from(c[0]);
    let a = f64::from(c[1]) / lead;
    let b = f64::from(c[2]) / lead;
    let cc = f64::from(c[3]) / lead;
    let d = f64::from(c[4]) / lead;
    let p = b - 3.0 * a * a / 8.0;
    let q = a * a * a / 8.0 - a * b / 2.0 + cc;
    let r = -3.0 * a * a * a * a / 256.0 + a * a * b / 16.0 - a * cc / 4.0 + d;

    // Ferrari's factorisation follows from
    // (y^2 + u*y + v) * (y^2 - u*y + w), where y = s + a/4.
    let cubic_a = 2.0 * p;
    let cubic_b = p * p - 4.0 * r;
    let cubic_c = -q * q;
    let depressed_p = cubic_b - cubic_a * cubic_a / 3.0;
    let depressed_q = 2.0 * cubic_a * cubic_a * cubic_a / 27.0 - cubic_a * cubic_b / 3.0 + cubic_c;
    let discriminant = depressed_q * depressed_q / 4.0 + depressed_p * depressed_p * depressed_p / 27.0;
    let z = if discriminant >= 0.0 {
        libm::cbrt(-depressed_q / 2.0 + libm::sqrt(discriminant))
            + libm::cbrt(-depressed_q / 2.0 - libm::sqrt(discriminant))
            - cubic_a / 3.0
    } else {
        let magnitude = 2.0 * libm::sqrt(-depressed_p / 3.0);
        let cosine = ((3.0 * depressed_q / (2.0 * depressed_p)) * libm::sqrt(-3.0 / depressed_p)).clamp(-1.0, 1.0);
        let phase = libm::acos(cosine);
        (0..3)
            .map(|index| magnitude * libm::cos((phase + 2.0 * core::f64::consts::PI * index as f64) / 3.0))
            .map(|root| root - cubic_a / 3.0)
            .fold(0.0_f64, f64::max)
    };
    let u = libm::sqrt(z);
    let v = (p + z - q / u) / 2.0;
    let w = (p + z + q / u) / 2.0;
    let shift = a / 4.0;
    [(2.0 * shift + u, shift * shift + u * shift + v), (2.0 * shift - u, shift * shift - u * shift + w)]
}

#[cfg(feature = "lowflow-conservative")]
impl Biquad {
    fn run(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        if y.is_finite() {
            y
        } else {
            0.0
        }
    }
    fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }
    #[cfg(feature = "lowflow-conservative")]
    fn identity() -> Self {
        Self { b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0, z1: 0.0, z2: 0.0 }
    }
    #[cfg(feature = "lowflow-conservative")]
    fn butterworth(fs: f64, hz: f64, highpass: bool) -> Self {
        if hz <= 0.0 {
            return Self::identity();
        }
        let w = libm::tan(core::f64::consts::PI * hz.min(fs * 0.49) / fs);
        let n = 1.0 / (1.0 + core::f64::consts::SQRT_2 * w + w * w);
        let (b0, b1, b2) = if highpass { (n, -2.0 * n, n) } else { (w * w * n, 2.0 * w * w * n, w * w * n) };
        Self {
            b0,
            b1,
            b2,
            a1: 2.0 * (w * w - 1.0) * n,
            a2: (1.0 - core::f64::consts::SQRT_2 * w + w * w) * n,
            z1: 0.0,
            z2: 0.0,
        }
    }
    fn velocity(fs: f64, mass: f64, damping: f64, stiffness: f64, force: f64) -> Self {
        let k = 2.0 * fs;
        let d0 = mass * k * k + damping * k + stiffness;
        Self {
            b0: force * k / d0,
            b1: 0.0,
            b2: -force * k / d0,
            a1: (-2.0 * mass * k * k + 2.0 * stiffness) / d0,
            a2: (mass * k * k - damping * k + stiffness) / d0,
            z1: 0.0,
            z2: 0.0,
        }
    }
}

struct State {
    #[cfg(feature = "lowflow-conservative")]
    mask_hp: Biquad,
    #[cfg(feature = "lowflow-conservative")]
    mask_lp: Biquad,
    #[cfg(feature = "lowflow-conservative")]
    mask_hp2: Biquad,
    #[cfg(feature = "lowflow-conservative")]
    mask_lp2: Biquad,
    #[cfg(feature = "lowflow-conservative")]
    cone: Biquad,
    #[cfg(feature = "lowflow-conservative")]
    front: Biquad,
    #[cfg(not(feature = "lowflow-conservative"))]
    velocity: VelocityCascade,
    #[cfg(feature = "lowflow-conservative")]
    pressure_taps: Box<[f64]>,
    #[cfg(feature = "lowflow-conservative")]
    tweeter_taps: Box<[f64]>,
    #[cfg(feature = "lowflow-conservative")]
    woofer_history: Box<[f64]>,
    #[cfg(feature = "lowflow-conservative")]
    tweeter_history: Box<[f64]>,
    #[cfg(feature = "lowflow-conservative")]
    history_at: usize,
    #[cfg(feature = "lowflow-conservative")]
    mask_energy: f64,
    #[cfg(feature = "lowflow-conservative")]
    mask_attack: f64,
    #[cfg(feature = "lowflow-conservative")]
    mask_release: f64,
    #[cfg(feature = "lowflow-conservative")]
    gain: f64,
    #[cfg(feature = "lowflow-conservative")]
    gain_release: f64,
    floor: f64,
    #[cfg(feature = "lowflow-conservative")]
    conservative_floor: f64,
    #[cfg(not(feature = "lowflow-conservative"))]
    velocity_block_energy: f64,
    #[cfg(not(feature = "lowflow-conservative"))]
    velocity_block_frames: usize,
    #[cfg(not(feature = "lowflow-conservative"))]
    hop_velocity: f64,
    #[cfg(not(feature = "lowflow-conservative"))]
    velocity_history: [f64; 2],
    #[cfg(not(feature = "lowflow-conservative"))]
    mask: MaskDetector,
    #[cfg(not(feature = "lowflow-conservative"))]
    mask_output_db: f64,
    #[cfg(not(feature = "lowflow-conservative"))]
    spectral: Spectral,
}

#[cfg(not(feature = "lowflow-conservative"))]
#[derive(Clone, Copy, Default)]
struct Complex {
    re: f64,
    im: f64,
}

#[cfg(not(feature = "lowflow-conservative"))]
impl Complex {
    fn mul(self, other: Self) -> Self {
        Self { re: self.re * other.re - self.im * other.im, im: self.re * other.im + self.im * other.re }
    }
}

/// Acoustic-mask analyzer whose `n`-frame sliding history advances by the
/// same 128-frame hop as the limiter STFT but has its own unit-energy Hann and
/// response-magnitude reduction.
#[cfg(not(feature = "lowflow-conservative"))]
struct MaskDetector {
    n: usize,
    at: usize,
    low_history: Box<[f64]>,
    high_history: Box<[f64]>,
    low: Box<[Complex]>,
    high: Box<[Complex]>,
    scratch: Box<[Complex]>,
    twiddles: Box<[Complex]>,
    window: Box<[f64]>,
    pressure_magnitude: Box<[f64]>,
    tweeter_magnitude: Box<[f64]>,
    boundaries: [usize; MASK_BANDS],
    band_energy: [f64; MASK_BANDS],
    mask_start: usize,
    mask_end: usize,
    concentration: f64,
    peak_db: f64,
    db: f64,
    attack: f64,
    release: f64,
}

#[cfg(not(feature = "lowflow-conservative"))]
impl MaskDetector {
    fn new(model: &Model, fs: f64, latency: usize) -> Self {
        let n = latency.saturating_add(SPECTRAL_HOP).clamp(2 * SPECTRAL_HOP, 4 * SPECTRAL_HOP);
        let mut window = vec![0.0; n];
        for (i, value) in window.iter_mut().enumerate() {
            let phase = 2.0 * core::f64::consts::PI * i as f64 / n as f64;
            *value = 0.5 - 0.5 * libm::cos(phase);
        }
        let norm = libm::sqrt(window.iter().map(|value| value * value).sum::<f64>()).max(1.0e-30);
        for value in &mut window {
            *value /= norm;
        }
        let mut twiddles = vec![Complex::default(); n];
        for (i, value) in twiddles.iter_mut().enumerate() {
            let phase = -2.0 * core::f64::consts::PI * i as f64 / n as f64;
            *value = Complex { re: libm::cos(phase), im: libm::sin(phase) };
        }
        let pressure_magnitude = response_magnitude(&model.pressure_response, n, model.analog_gain, &twiddles);
        let tweeter_magnitude = response_magnitude(&model.tweeter_response, n, model.tweeter_gain, &twiddles);
        let boundaries = mask_boundaries(n);
        let lower_hz = |band: usize| {
            if band == 0 {
                0.0
            } else {
                boundaries[band - 1] as f64 * MASK_REFERENCE_FS / n as f64
            }
        };
        let upper_hz = |band: usize| boundaries[band] as f64 * MASK_REFERENCE_FS / n as f64;
        let closest = |hz: f64, edge: &dyn Fn(usize) -> f64| {
            (0..MASK_BANDS).min_by(|a, b| (edge(*a) - hz).abs().total_cmp(&(edge(*b) - hz).abs())).unwrap_or(0)
        };
        let mask_start = closest(model.mask_band_min_hz, &lower_hz);
        let mask_end = closest(model.mask_band_max_hz, &upper_hz).max(mask_start + 1).min(MASK_BANDS);
        Self {
            n,
            at: 0,
            low_history: vec![0.0; n].into_boxed_slice(),
            high_history: vec![0.0; n].into_boxed_slice(),
            low: vec![Complex::default(); n].into_boxed_slice(),
            high: vec![Complex::default(); n].into_boxed_slice(),
            scratch: vec![Complex::default(); n].into_boxed_slice(),
            twiddles: twiddles.into_boxed_slice(),
            window: window.into_boxed_slice(),
            pressure_magnitude,
            tweeter_magnitude,
            boundaries,
            band_energy: [0.0; MASK_BANDS],
            mask_start,
            mask_end,
            concentration: 1.0,
            peak_db: 0.0,
            db: 0.0,
            attack: hop_coefficient(model.mask_attack_s, fs),
            release: hop_coefficient(model.mask_release_s, fs),
        }
    }

    fn reset(&mut self) {
        self.at = 0;
        self.low_history.fill(0.0);
        self.high_history.fill(0.0);
        self.low.fill(Complex::default());
        self.high.fill(Complex::default());
        self.scratch.fill(Complex::default());
        self.band_energy.fill(0.0);
        self.concentration = 1.0;
        self.peak_db = 0.0;
        self.db = 0.0;
    }

    /// Adds one pre-delay LF/HF frame and returns a new smoothed level at the
    /// end of each 128-frame hop.
    fn push(&mut self, low: f32, high: f32) -> Option<f64> {
        if self.at == 0 {
            self.low_history.copy_within(SPECTRAL_HOP.., 0);
            self.high_history.copy_within(SPECTRAL_HOP.., 0);
        }
        let history_at = self.n - SPECTRAL_HOP + self.at;
        self.low_history[history_at] = f64::from(finite(low));
        self.high_history[history_at] = f64::from(finite(high));
        self.at += 1;
        if self.at != SPECTRAL_HOP {
            return None;
        }
        self.at = 0;
        for i in 0..self.n {
            self.low[i] = Complex { re: self.low_history[i] * self.window[i], im: 0.0 };
            self.high[i] = Complex { re: self.high_history[i] * self.window[i], im: 0.0 };
        }
        transform(&mut self.low, &mut self.scratch, &self.twiddles, false);
        transform(&mut self.high, &mut self.scratch, &self.twiddles, false);
        let bin_count = self.n / 2 + 1;
        let mut sum = 0.0;
        let mut square_sum = 0.0;
        for bin in 0..bin_count {
            // vDSP's real forward DFT has a factor of two. The response path
            // removes its own factor with 0.5; the program path retains it.
            let low = 2.0 * complex_magnitude(self.low[bin]) * self.pressure_magnitude[bin];
            let high = 2.0 * complex_magnitude(self.high[bin]) * self.tweeter_magnitude[bin];
            let magnitude = low + high;
            sum += magnitude;
            square_sum += magnitude * magnitude;
            if bin < MASK_DIRECT_BANDS {
                self.band_energy[bin] = magnitude * magnitude;
            } else if bin < self.band_energy.len() {
                self.band_energy[bin] = 0.0;
            }
            self.scratch[bin].re = magnitude * magnitude;
        }
        let concentration = libm::sqrt(bin_count as f64 * square_sum) / (sum + 1.0e-9);
        self.concentration = 0.1 * self.concentration + 0.9 * concentration;
        for band in MASK_DIRECT_BANDS..MASK_BANDS {
            let start = self.boundaries[band - 1].min(bin_count);
            let end = self.boundaries[band].clamp(start + 1, bin_count);
            self.band_energy[band] =
                self.scratch[start..end].iter().map(|value| value.re).sum::<f64>() / (end - start) as f64;
        }
        let region = &mut self.band_energy[self.mask_start..self.mask_end];
        region.sort_by(f64::total_cmp);
        let middle = region.len() / 2;
        let median = if region.len() % 2 == 0 { 0.5 * (region[middle - 1] + region[middle]) } else { region[middle] };
        let raw_db = 10.0 * libm::log10(libm::fabs(median + 1.0e-30) + 1.0e-20) + 97.0
            - 4.0 * (4.0 - self.concentration.clamp(1.0, 4.0));
        if raw_db > self.peak_db + 3.0 || raw_db + 12.0 < self.peak_db {
            self.peak_db = raw_db;
        }
        let alpha = if self.peak_db > self.db { self.attack } else { self.release };
        self.db += alpha * (self.peak_db - self.db);
        self.low.fill(Complex::default());
        self.high.fill(Complex::default());
        Some(self.db)
    }
}

#[cfg(not(feature = "lowflow-conservative"))]
fn complex_magnitude(value: Complex) -> f64 {
    libm::sqrt(value.re * value.re + value.im * value.im)
}

#[cfg(not(feature = "lowflow-conservative"))]
fn response_magnitude(taps: &[f64], n: usize, gain: f64, twiddles: &[Complex]) -> Box<[f64]> {
    let mut bins = vec![Complex::default(); n];
    for (dst, tap) in bins.iter_mut().zip(taps) {
        dst.re = *tap;
    }
    let mut scratch = vec![Complex::default(); n];
    transform(&mut bins, &mut scratch, twiddles, false);
    bins[..n / 2 + 1]
        .iter()
        .copied()
        .map(|value| complex_magnitude(value) * gain)
        .collect::<Vec<_>>()
        .into_boxed_slice()
}

#[cfg(not(feature = "lowflow-conservative"))]
fn mask_boundaries(n: usize) -> [usize; MASK_BANDS] {
    let mut centres = [0.0; MASK_BANDS];
    for (index, centre) in centres.iter_mut().enumerate() {
        *centre = bark_hz(index as f64 * 0.25) * n as f64 / MASK_REFERENCE_FS;
    }
    let mut cumulative = [0; MASK_BANDS];
    cumulative[0] = (libm::round(centres[0]) as usize).max(1);
    for index in 1..MASK_BANDS {
        let width = (libm::round(centres[index] - centres[index - 1]) as usize).max(1);
        cumulative[index] = cumulative[index - 1] + width;
    }
    let mut provisional = [0; MASK_BANDS];
    provisional[0] = 1;
    for index in 1..MASK_BANDS {
        provisional[index] = (cumulative[index - 1] + 1).max(libm::round(centres[index]) as usize);
    }
    let mut widths = [0; MASK_BANDS - 1];
    for (index, width) in widths.iter_mut().enumerate() {
        *width = provisional[index + 1] - provisional[index];
    }
    widths.sort_unstable();
    let mut boundaries = [0; MASK_BANDS];
    boundaries[0] = 1;
    for index in 1..MASK_BANDS {
        boundaries[index] = boundaries[index - 1] + widths[index - 1];
    }
    boundaries
}

#[cfg(not(feature = "lowflow-conservative"))]
fn bark_hz(bark: f64) -> f64 {
    let adjusted = if bark < 2.0 {
        (bark - 0.3) / 0.85
    } else if bark > 20.1 {
        (bark + 4.422) / 1.22
    } else {
        bark
    };
    1960.0 * (adjusted + 0.53) / (26.28 - adjusted)
}

/// Fixed-hop frequency-selective gain application. Construction allocates all
/// scratch; [`Spectral::prepare_hop`] and [`Spectral::render_hop`] are
/// allocation-free.
///
/// Each hop transforms the newest `n` low-lane frames twice: once through the
/// square-root Hann analysis window that is also the overlap-add synthesis
/// window, and once through the plain Hann window. The water-fill cap is
/// solved on the Hann-window magnitudes and the resulting per-bin gains scale
/// the analysis spectrum. The Hann window confines a steady tone to its own
/// and the two neighbouring bins, so tones outside the affected interval do
/// not leak into the capped bins.
#[cfg(not(feature = "lowflow-conservative"))]
struct Spectral {
    n: usize,
    min_bin: usize,
    max_bin: usize,
    history: Box<[f64]>,
    overlap: Box<[f64]>,
    window: Box<[f64]>,
    detector_window: Box<[f64]>,
    bins: Box<[Complex]>,
    detector_bins: Box<[Complex]>,
    detector_magnitude: Box<[f64]>,
    scratch: Box<[Complex]>,
    twiddles: Box<[Complex]>,
    gains: Box<[f64]>,
    target: f64,
    release: f64,
}

#[cfg(not(feature = "lowflow-conservative"))]
impl Spectral {
    fn new(model: &Model, fs: f64, latency: usize) -> Self {
        let n = latency.saturating_add(SPECTRAL_HOP).clamp(2 * SPECTRAL_HOP, 4 * SPECTRAL_HOP);
        let mut window = vec![0.0; n];
        let mut detector_window = vec![0.0; n];
        for (i, (value, detector)) in window.iter_mut().zip(detector_window.iter_mut()).enumerate() {
            let phase = 2.0 * core::f64::consts::PI * i as f64 / n as f64;
            let hann = 0.5 - 0.5 * libm::cos(phase);
            *value = libm::sqrt((hann * SPECTRAL_HOP as f64 / n as f64).max(0.0));
            *detector = hann;
        }
        let mut twiddles = vec![Complex::default(); n];
        for (i, value) in twiddles.iter_mut().enumerate() {
            let phase = -2.0 * core::f64::consts::PI * i as f64 / n as f64;
            *value = Complex { re: libm::cos(phase), im: libm::sin(phase) };
        }
        let min_bin = libm::round(model.min_frequency_hz * n as f64 / fs) as usize;
        let max_bin = libm::round(model.max_frequency_hz * n as f64 / fs) as usize;
        let bin_count = n / 2 + 1;
        Self {
            n,
            min_bin: min_bin.min(bin_count - 1),
            max_bin: max_bin.clamp(1, bin_count),
            history: vec![0.0; n].into_boxed_slice(),
            overlap: vec![0.0; n].into_boxed_slice(),
            window: window.into_boxed_slice(),
            detector_window: detector_window.into_boxed_slice(),
            bins: vec![Complex::default(); n].into_boxed_slice(),
            detector_bins: vec![Complex::default(); n].into_boxed_slice(),
            detector_magnitude: vec![0.0; bin_count].into_boxed_slice(),
            scratch: vec![Complex::default(); n].into_boxed_slice(),
            twiddles: twiddles.into_boxed_slice(),
            gains: vec![1.0; bin_count].into_boxed_slice(),
            target: 1.0,
            release: hop_coefficient(model.release_s, fs),
        }
    }

    fn reset(&mut self) {
        self.history.fill(0.0);
        self.overlap.fill(0.0);
        self.bins.fill(Complex::default());
        self.detector_bins.fill(Complex::default());
        self.detector_magnitude.fill(0.0);
        self.scratch.fill(Complex::default());
        self.gains.fill(1.0);
        self.target = 1.0;
    }

    fn set_target(&mut self, target: f64) {
        self.target = target.clamp(0.0, 1.0);
    }

    /// Shifts one completed low-lane hop into the analysis history and
    /// transforms it through both windows.
    fn prepare_hop(&mut self, low: &[f32]) {
        debug_assert_eq!(low.len(), SPECTRAL_HOP);
        self.history.copy_within(SPECTRAL_HOP.., 0);
        let tail = self.n - SPECTRAL_HOP;
        for (dst, src) in self.history[tail..].iter_mut().zip(low.iter()) {
            *dst = f64::from(finite(*src));
        }
        for i in 0..self.n {
            self.bins[i] = Complex { re: self.history[i] * self.window[i], im: 0.0 };
            self.detector_bins[i] = Complex { re: self.history[i] * self.detector_window[i], im: 0.0 };
        }
        transform(&mut self.bins, &mut self.scratch, &self.twiddles, false);
        transform(&mut self.detector_bins, &mut self.scratch, &self.twiddles, false);
        for (magnitude, value) in self.detector_magnitude.iter_mut().zip(self.detector_bins.iter()) {
            *magnitude = libm::sqrt(value.re * value.re + value.im * value.im);
        }
    }

    fn render_hop(&mut self, io: &mut [f32]) {
        debug_assert_eq!(io.len(), SPECTRAL_HOP);
        for i in 0..self.n {
            let bin = if i <= self.n / 2 { i } else { self.n - i };
            self.bins[i].re *= self.gains[bin];
            self.bins[i].im *= self.gains[bin];
        }
        transform(&mut self.bins, &mut self.scratch, &self.twiddles, true);
        for i in 0..self.n {
            self.overlap[i] += 2.0 * self.window[i] * self.bins[i].re;
        }
        for (dst, value) in io.iter_mut().zip(&self.overlap[..SPECTRAL_HOP]) {
            *dst = if value.is_finite() { *value as f32 } else { 0.0 };
        }
        self.overlap.copy_within(SPECTRAL_HOP.., 0);
        self.overlap[self.n - SPECTRAL_HOP..].fill(0.0);
        self.target = 1.0;
    }

    /// Water-fill on the detector magnitudes `a[k]` of the affected bins:
    /// solve `rms(min(a, C)) = target * rms(a)`, then per bin
    /// `min(C / a, 1)`, with immediate attack and the saved per-bin release.
    fn update_gains(&mut self) {
        let start = self.min_bin.min(self.gains.len());
        let end = self.max_bin.max(start).min(self.gains.len());
        let affected = &self.detector_magnitude[start..end];
        let mut cap = f64::INFINITY;
        if self.target < 1.0 && start < end {
            let count = (end - start) as f64;
            let sum: f64 = affected.iter().map(|a| a * a).sum();
            let wanted = self.target * libm::sqrt(sum / count);
            if wanted > 0.0 {
                let mut lo = 0.0;
                let mut hi = affected.iter().copied().fold(0.0_f64, f64::max);
                for _ in 0..48 {
                    let cap = 0.5 * (lo + hi);
                    let clipped: f64 = affected.iter().map(|a| a.min(cap) * a.min(cap)).sum();
                    if libm::sqrt(clipped / count) > wanted {
                        hi = cap;
                    } else {
                        lo = cap;
                    }
                }
                cap = 0.5 * (lo + hi);
            }
        }
        for (bin, gain) in self.gains.iter_mut().enumerate() {
            let target = if (start..end).contains(&bin) && cap.is_finite() {
                let magnitude = self.detector_magnitude[bin];
                if magnitude > 0.0 {
                    (cap / magnitude).clamp(0.0, 1.0)
                } else {
                    1.0
                }
            } else {
                1.0
            };
            let released = *gain + self.release * (target - *gain);
            *gain = target.min(released).clamp(0.0, 1.0);
        }
    }
}

#[cfg(not(feature = "lowflow-conservative"))]
fn transform(values: &mut [Complex], scratch: &mut [Complex], twiddles: &[Complex], inverse: bool) {
    let n = values.len();
    if n.is_power_of_two() {
        fft_pow2(values, twiddles, inverse);
    } else {
        debug_assert_eq!(n, 3 * SPECTRAL_HOP);
        for r in 0..3 {
            for k in 0..SPECTRAL_HOP {
                scratch[r * SPECTRAL_HOP + k] = values[3 * k + r];
            }
            fft_pow2(&mut scratch[r * SPECTRAL_HOP..(r + 1) * SPECTRAL_HOP], twiddles, inverse);
        }
        for q in 0..3 {
            for k in 0..SPECTRAL_HOP {
                let index = k + q * SPECTRAL_HOP;
                let mut sum = Complex::default();
                for r in 0..3 {
                    let mut twiddle = twiddles[index * r % n];
                    if inverse {
                        twiddle.im = -twiddle.im;
                    }
                    let value = scratch[r * SPECTRAL_HOP + k].mul(twiddle);
                    sum.re += value.re;
                    sum.im += value.im;
                }
                values[index] = sum;
            }
        }
    }
    if inverse {
        for value in values {
            value.re /= n as f64;
            value.im /= n as f64;
        }
    }
}

#[cfg(not(feature = "lowflow-conservative"))]
fn fft_pow2(values: &mut [Complex], twiddles: &[Complex], inverse: bool) {
    let n = values.len();
    let table_n = twiddles.len();
    let mut j = 0;
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
        for base in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let mut twiddle = twiddles[k * table_n / len];
                if inverse {
                    twiddle.im = -twiddle.im;
                }
                let even = values[base + k];
                let odd = values[base + k + len / 2].mul(twiddle);
                values[base + k] = Complex { re: even.re + odd.re, im: even.im + odd.im };
                values[base + k + len / 2] = Complex { re: even.re - odd.re, im: even.im - odd.im };
            }
        }
        len <<= 1;
    }
}

pub struct Run {
    channels: usize,
    models: usize,
    link_masks: bool,
    params: [Model; MAX_MODELS],
    state: Box<[State]>,
    line: Box<[f32]>,
    line_at: usize,
    latency_frames: usize,
    #[cfg(not(feature = "lowflow-conservative"))]
    conservative_fallback: bool,
    volume_db: f64,
}

impl Run {
    pub fn new(params: &Params, fs: f64, channels: usize) -> Self {
        let fs = if fs.is_finite() { fs.clamp(8_000.0, MAX_FS) } else { 48_000.0 };
        let channels = channels.clamp(1, MAX_CHANNELS);
        let models = params.model_count.min(channels / 2).min(MAX_MODELS);
        let mut state = Vec::with_capacity(models);
        for m in &params.models[..models] {
            #[cfg(feature = "lowflow-conservative")]
            let electrical_damping = m.bl * m.bl / m.reb;
            #[cfg(feature = "lowflow-conservative")]
            let force = m.bl / m.reb * m.analog_gain;
            state.push(State {
                #[cfg(feature = "lowflow-conservative")]
                mask_hp: Biquad::butterworth(fs, m.mask_band_min_hz, true),
                #[cfg(feature = "lowflow-conservative")]
                mask_lp: Biquad::butterworth(fs, m.mask_band_max_hz, false),
                #[cfg(feature = "lowflow-conservative")]
                mask_hp2: Biquad::butterworth(fs, m.mask_band_min_hz, true),
                #[cfg(feature = "lowflow-conservative")]
                mask_lp2: Biquad::butterworth(fs, m.mask_band_max_hz, false),
                #[cfg(feature = "lowflow-conservative")]
                cone: Biquad::velocity(fs, m.mms, m.rms + electrical_damping, m.kms, force),
                #[cfg(feature = "lowflow-conservative")]
                front: Biquad::velocity(
                    fs,
                    m.mms_front_port,
                    m.rms_front_port + electrical_damping,
                    m.kms_front_port,
                    force,
                ),
                #[cfg(not(feature = "lowflow-conservative"))]
                velocity: VelocityCascade::new(m),
                #[cfg(feature = "lowflow-conservative")]
                pressure_taps: m.pressure_response.to_vec().into_boxed_slice(),
                #[cfg(feature = "lowflow-conservative")]
                tweeter_taps: m.tweeter_response.to_vec().into_boxed_slice(),
                #[cfg(feature = "lowflow-conservative")]
                woofer_history: vec![0.0; RESPONSE_TAPS].into_boxed_slice(),
                #[cfg(feature = "lowflow-conservative")]
                tweeter_history: vec![0.0; RESPONSE_TAPS].into_boxed_slice(),
                #[cfg(feature = "lowflow-conservative")]
                history_at: 0,
                #[cfg(feature = "lowflow-conservative")]
                mask_energy: 0.0,
                #[cfg(feature = "lowflow-conservative")]
                mask_attack: coefficient(m.mask_attack_s, fs),
                #[cfg(feature = "lowflow-conservative")]
                mask_release: coefficient(m.mask_release_s, fs),
                #[cfg(feature = "lowflow-conservative")]
                gain: 1.0,
                #[cfg(feature = "lowflow-conservative")]
                gain_release: gain_release_coefficient(m.release_s, fs),
                floor: 1.0,
                #[cfg(feature = "lowflow-conservative")]
                conservative_floor: 1.0,
                #[cfg(not(feature = "lowflow-conservative"))]
                velocity_block_energy: 0.0,
                #[cfg(not(feature = "lowflow-conservative"))]
                velocity_block_frames: 0,
                #[cfg(not(feature = "lowflow-conservative"))]
                hop_velocity: 0.0,
                #[cfg(not(feature = "lowflow-conservative"))]
                velocity_history: [0.0; 2],
                #[cfg(not(feature = "lowflow-conservative"))]
                mask: MaskDetector::new(m, fs, params.latency_frames),
                #[cfg(not(feature = "lowflow-conservative"))]
                mask_output_db: 0.0,
                #[cfg(not(feature = "lowflow-conservative"))]
                spectral: Spectral::new(m, fs, params.latency_frames),
            });
        }
        let mut run = Self {
            channels,
            models,
            link_masks: params.link_channel_energies,
            params: params.models,
            state: state.into_boxed_slice(),
            line: vec![0.0; channels * params.latency_frames].into_boxed_slice(),
            line_at: 0,
            latency_frames: params.latency_frames,
            #[cfg(not(feature = "lowflow-conservative"))]
            conservative_fallback: params.conservative_fallback,
            volume_db: 0.0,
        };
        run.set_volume(0.0);
        run
    }

    /// Sets graph `mvol`/parameter 4 in dB without allocation or locking.
    pub fn set_volume(&mut self, db: f64) {
        self.volume_db = if db.is_finite() { db.clamp(-100.0, 0.0) } else { -100.0 };
        for (state, model) in self.state.iter_mut().zip(self.params.iter()) {
            let t = ((self.volume_db - model.mid_level_db) / -model.mid_level_db).clamp(0.0, 1.0);
            let attenuation = model.mid_attenuation_db + t * (model.max_attenuation_db - model.mid_attenuation_db);
            state.floor = db_gain(-attenuation);
            #[cfg(feature = "lowflow-conservative")]
            {
                state.conservative_floor = db_gain(-(attenuation + CONSERVATIVE_GUARD_DB));
            }
        }
    }

    pub fn reset(&mut self) {
        self.line.fill(0.0);
        self.line_at = 0;
        for state in self.state.iter_mut() {
            #[cfg(feature = "lowflow-conservative")]
            state.cone.reset();
            #[cfg(feature = "lowflow-conservative")]
            state.front.reset();
            #[cfg(feature = "lowflow-conservative")]
            {
                state.mask_hp.reset();
                state.mask_lp.reset();
                state.mask_hp2.reset();
                state.mask_lp2.reset();
                state.woofer_history.fill(0.0);
                state.tweeter_history.fill(0.0);
                state.history_at = 0;
                state.mask_energy = 0.0;
            }
            #[cfg(feature = "lowflow-conservative")]
            {
                state.gain = 1.0;
            }
            #[cfg(not(feature = "lowflow-conservative"))]
            {
                state.velocity.reset();
                state.velocity_block_energy = 0.0;
                state.velocity_block_frames = 0;
                state.hop_velocity = 0.0;
                state.velocity_history = [0.0; 2];
                state.mask.reset();
                state.mask_output_db = 0.0;
                state.spectral.reset();
            }
        }
    }

    pub fn latency(&self) -> usize {
        self.latency_frames
    }

    /// Offline diagnostics: the linked mask level (dB) and newest completed
    /// hop velocity of one model, as of the last processed hop.
    #[doc(hidden)]
    pub fn probe(&self, model: usize) -> [f64; 2] {
        #[cfg(not(feature = "lowflow-conservative"))]
        {
            self.state.get(model).map_or([0.0; 2], |state| [state.mask_output_db, state.velocity_history[0]])
        }
        #[cfg(feature = "lowflow-conservative")]
        {
            let _ = model;
            [0.0; 2]
        }
    }

    #[allow(clippy::needless_range_loop)] // one frame across planar buffers
    #[cfg(feature = "lowflow-conservative")]
    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let channels = self.channels.min(bufs.len());
        let frames = bufs[..channels].iter().fold(frames, |n, b| n.min(b.len()));
        for frame in 0..frames {
            let mut energies = [0.0; MAX_MODELS];
            #[cfg(not(feature = "lowflow-conservative"))]
            let mut velocities = [0.0; MAX_MODELS];
            for c in 0..self.models {
                let woofer = f64::from(finite(bufs[c][frame]));
                let tweeter = f64::from(finite(bufs[c + self.models][frame]));
                let state = &mut self.state[c];
                state.woofer_history[state.history_at] = woofer;
                state.tweeter_history[state.history_at] = tweeter;
                let pressure = fir(&state.pressure_taps, &state.woofer_history, state.history_at)
                    * self.params[c].analog_gain
                    + fir(&state.tweeter_taps, &state.tweeter_history, state.history_at) * self.params[c].tweeter_gain;
                state.history_at = (state.history_at + 1) % RESPONSE_TAPS;
                let mask_band = state.mask_lp2.run(state.mask_lp.run(state.mask_hp2.run(state.mask_hp.run(pressure))));
                energies[c] = mask_band * mask_band;
                #[cfg(not(feature = "lowflow-conservative"))]
                {
                    let band = state.detector_lp.run(state.detector_hp.run(woofer));
                    let velocity = state.cone.run(band) + state.front.run(band);
                    velocities[c] = state.velocity_scale * libm::fabs(velocity);
                }
            }
            if self.link_masks && self.models > 1 {
                let total: f64 = energies[..self.models].iter().sum();
                energies[..self.models].fill(total);
            }
            for c in 0..self.models {
                let model = &self.params[c];
                let state = &mut self.state[c];
                let alpha = if energies[c] > state.mask_energy { state.mask_attack } else { state.mask_release };
                state.mask_energy += alpha * (energies[c] - state.mask_energy);
                #[cfg(not(feature = "lowflow-conservative"))]
                let limit = {
                    let energy_db = 10.0 * libm::log10(state.mask_energy.max(1.0e-30));
                    velocity_limit(model, energy_db)
                };
                #[cfg(feature = "lowflow-conservative")]
                let target = if !model.enabled {
                    1.0
                } else if model.mute {
                    0.0
                } else {
                    state.conservative_floor
                };
                #[cfg(not(feature = "lowflow-conservative"))]
                let target = if !model.enabled {
                    1.0
                } else if model.mute {
                    0.0
                } else if velocities[c] > limit {
                    (limit / velocities[c]).max(state.floor)
                } else {
                    1.0
                };
                if target < state.gain {
                    state.gain = target;
                } else {
                    state.gain += state.gain_release * (target - state.gain);
                }
            }
            for c in 0..channels {
                let slot = c * self.latency_frames + self.line_at;
                let delayed = self.line[slot];
                self.line[slot] = finite(bufs[c][frame]);
                bufs[c][frame] =
                    if c < self.models { (f64::from(delayed) * self.state[c].gain) as f32 } else { delayed };
            }
            self.line_at = (self.line_at + 1) % self.latency_frames;
        }
    }

    #[allow(clippy::needless_range_loop)] // one frame across planar buffers
    #[cfg(not(feature = "lowflow-conservative"))]
    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let channels = self.channels.min(bufs.len());
        let frames = bufs[..channels].iter().fold(frames, |n, b| n.min(b.len()));
        let mut chunk_at = 0;
        while chunk_at < frames {
            let chunk_frames = SPECTRAL_HOP.min(frames - chunk_at);
            let full_hop = chunk_frames == SPECTRAL_HOP;
            for frame in chunk_at..chunk_at + chunk_frames {
                for c in 0..self.models {
                    let woofer_sample = finite(bufs[c][frame]);
                    let tweeter_sample =
                        if c + self.models < channels { finite(bufs[c + self.models][frame]) } else { 0.0 };
                    let state = &mut self.state[c];
                    state.mask.push(0.0, tweeter_sample);
                    let velocity = f64::from(state.velocity.run(woofer_sample));
                    state.velocity_block_energy += velocity * velocity;
                    state.velocity_block_frames += 1;
                    // The physical detector reports the largest 8-frame RMS
                    // in each spectral hop, preserving the signed cone/front
                    // cancellation before energy is measured.
                    if state.velocity_block_frames == 8 {
                        let rms = libm::sqrt(state.velocity_block_energy / 8.0);
                        state.hop_velocity = state.hop_velocity.max(rms);
                        state.velocity_block_energy = 0.0;
                        state.velocity_block_frames = 0;
                    }
                }
                if self.link_masks && self.models > 1 {
                    let linked = self.state[..self.models].iter().map(|state| state.mask.db).fold(0.0_f64, f64::max);
                    for state in &mut self.state[..self.models] {
                        state.mask_output_db = linked;
                    }
                } else {
                    for state in &mut self.state[..self.models] {
                        state.mask_output_db = state.mask.db.max(0.0);
                    }
                }
                for c in 0..channels {
                    let slot = c * self.latency_frames + self.line_at;
                    let delayed = self.line[slot];
                    self.line[slot] = finite(bufs[c][frame]);
                    if c >= self.models {
                        bufs[c][frame] = delayed;
                    } else if self.conservative_fallback || !full_hop {
                        let fallback = self.state[c].floor * db_gain(-CONSERVATIVE_GUARD_DB);
                        bufs[c][frame] = (f64::from(delayed) * fallback) as f32;
                    }
                }
                self.line_at = (self.line_at + 1) % self.latency_frames;
            }
            if full_hop && !self.conservative_fallback {
                for c in 0..self.models {
                    self.state[c].spectral.prepare_hop(&bufs[c][chunk_at..chunk_at + chunk_frames]);
                }
                for c in 0..self.models {
                    let model = &self.params[c];
                    let state = &mut self.state[c];
                    let limit = velocity_limit(model, state.mask_output_db);
                    let target = detector_target(model.enabled, model.mute, state.floor, limit, state.velocity_history);
                    state.spectral.set_target(target);
                    // The mask runtime has already transformed the current
                    // detector hop. Target/water-fill and gain release run
                    // before the current output transform consumes that
                    // released vector.
                    state.spectral.update_gains();
                    // The target controls the frame already in flight, so the
                    // just-completed detector hop becomes history afterwards.
                    state.velocity_history = [state.hop_velocity, state.velocity_history[0]];
                    state.hop_velocity = 0.0;
                    if model.mute {
                        bufs[c][chunk_at..chunk_at + chunk_frames].fill(0.0);
                        state.spectral.reset();
                    } else {
                        state.spectral.render_hop(&mut bufs[c][chunk_at..chunk_at + chunk_frames]);
                    }
                }
            } else {
                for state in self.state.iter_mut() {
                    state.spectral.reset();
                }
            }
            chunk_at += chunk_frames;
        }
    }
}

#[cfg(not(feature = "lowflow-conservative"))]
fn detector_target(enabled: bool, mute: bool, floor: f64, limit: f64, velocity_history: [f64; 2]) -> f64 {
    if !enabled {
        return 1.0;
    }
    if mute {
        return 0.0;
    }
    let velocity = limit.max(velocity_history[0]).max(velocity_history[1]);
    if velocity > 0.0 {
        (limit / velocity).clamp(floor, 1.0)
    } else {
        1.0
    }
}

#[cfg(feature = "lowflow-conservative")]
fn coefficient(seconds: f64, fs: f64) -> f64 {
    if seconds <= 0.0 {
        1.0
    } else {
        1.0 - libm::exp(-1.0 / (seconds * fs))
    }
}
#[cfg(feature = "lowflow-conservative")]
fn gain_release_coefficient(seconds: f64, fs: f64) -> f64 {
    coefficient(seconds, fs)
}
#[cfg(not(feature = "lowflow-conservative"))]
fn hop_coefficient(seconds: f64, fs: f64) -> f64 {
    if seconds <= 0.0 {
        1.0
    } else {
        1.0 - libm::exp(-(SPECTRAL_HOP as f64) / (seconds * fs))
    }
}
#[cfg(feature = "lowflow-conservative")]
fn fir(taps: &[f64], history: &[f64], newest: usize) -> f64 {
    let mut sum = 0.0;
    let mut at = newest;
    for tap in taps {
        sum += *tap * history[at];
        at = if at == 0 { history.len() - 1 } else { at - 1 };
    }
    sum
}
#[cfg(not(feature = "lowflow-conservative"))]
fn velocity_limit(m: &Model, energy_db: f64) -> f64 {
    let energy = energy_db as f32;
    let min_mask = m.min_mask_energy_db as f32;
    let mid_mask = m.mid_mask_energy_db as f32;
    let high_mask = m.high_mask_energy_db as f32;
    let max_mask = m.max_mask_energy_db as f32;
    let min_velocity = m.min_velocity as f32;
    let mid_velocity = m.mid_velocity as f32;
    let high_velocity = m.high_velocity as f32;
    if energy <= min_mask {
        return f64::from(min_velocity);
    }
    if energy < mid_mask {
        let slope = (mid_velocity - min_velocity) / (mid_mask - min_mask);
        return f64::from(libm::fmaf(slope, energy - min_mask, min_velocity));
    }
    if energy < high_mask {
        let slope = (high_velocity - mid_velocity) / (high_mask - mid_mask);
        return f64::from(libm::fmaf(slope, energy - mid_mask, mid_velocity));
    }
    // The final segment reaches the fixed fourth velocity knot at the saved
    // maximum mask energy. It is a line, not a clamp: still-higher mask
    // levels continue to raise the allowed velocity.
    if max_mask > high_mask {
        let slope = (MAX_MASK_VELOCITY - high_velocity) / (max_mask - high_mask);
        f64::from(libm::fmaf(slope, energy - high_mask, high_velocity))
    } else {
        f64::from(high_velocity)
    }
}
fn finite(x: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        0.0
    }
}
fn db_gain(db: f64) -> f64 {
    libm::pow(10.0, db / 20.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    fn synthetic() -> Params {
        let mut models = [Model::default(); MAX_MODELS];
        for model in &mut models[..2] {
            *model = Model {
                bl: 1.2,
                kms: 6_000.0,
                kms_front_port: 70_000.0,
                mms: 0.001,
                mms_front_port: 0.0008,
                reb: 4.0,
                rms: 1.0,
                rms_front_port: 3.0,
                analog_gain: 8.0,
                tweeter_gain: 8.0,
                mask_band_min_hz: 4_000.0,
                mask_band_max_hz: 14_000.0,
                min_velocity: 0.04,
                min_mask_energy_db: -10.0,
                mid_velocity: 0.08,
                mid_mask_energy_db: 10.0,
                high_velocity: 0.16,
                high_mask_energy_db: 30.0,
                max_mask_energy_db: 40.0,
                mask_attack_s: 0.0,
                mask_release_s: 0.05,
                max_attenuation_db: 12.0,
                release_s: 0.05,
                min_frequency_hz: 300.0,
                max_frequency_hz: 2_500.0,
                mid_attenuation_db: 0.0,
                mid_level_db: -30.0,
                enabled: true,
                mute: false,
                pressure_response: {
                    let mut x = [0.0; RESPONSE_TAPS];
                    x[0] = 1.0;
                    x
                },
                tweeter_response: {
                    let mut x = [0.0; RESPONSE_TAPS];
                    x[0] = 0.02;
                    x
                },
            };
        }
        Params {
            model_count: 2,
            link_channel_energies: true,
            latency_frames: LATENCY_FRAMES,
            conservative_fallback: false,
            models,
        }
    }

    fn sine(hz: f64, amp: f32, n: usize) -> Vec<f32> {
        (0..n).map(|i| amp * libm::sin(2.0 * core::f64::consts::PI * hz * i as f64 / 48_000.0) as f32).collect()
    }

    #[test]
    fn words_round_trip_and_reject_damage() {
        let p = synthetic();
        let mut words = vec![0.0; Params::WORDS];
        p.to_words(&mut words);
        assert_eq!(Params::from_words(&words), Some(p));
        words[12] = f64::NAN;
        assert_eq!(Params::from_words(&words), None);
    }

    #[test]
    fn upper_lanes_are_exact_delay() {
        let p = synthetic();
        let mut run = Run::new(&p, 48_000.0, 4);
        let mut b = [vec![0.0; 1024], vec![0.0; 1024], sine(8_000.0, 0.7, 1024), sine(9_000.0, 0.5, 1024)];
        let original = b.clone();
        let mut refs: Vec<&mut [f32]> = b.iter_mut().map(Vec::as_mut_slice).collect();
        run.process(&mut refs, 1024);
        assert_eq!(&b[2][LATENCY_FRAMES..], &original[2][..1024 - LATENCY_FRAMES]);
        assert_eq!(&b[3][LATENCY_FRAMES..], &original[3][..1024 - LATENCY_FRAMES]);
    }

    #[cfg(not(feature = "lowflow-conservative"))]
    #[test]
    fn mixed_radix_transforms_round_trip() {
        for n in [384, 512] {
            let mut values: Vec<_> =
                (0..n).map(|i| Complex { re: libm::sin(0.031 * i as f64), im: libm::cos(0.017 * i as f64) }).collect();
            let original = values.clone();
            let mut scratch = vec![Complex::default(); n];
            let twiddles: Vec<_> = (0..n)
                .map(|i| {
                    let phase = -2.0 * core::f64::consts::PI * i as f64 / n as f64;
                    Complex { re: libm::cos(phase), im: libm::sin(phase) }
                })
                .collect();
            transform(&mut values, &mut scratch, &twiddles, false);
            transform(&mut values, &mut scratch, &twiddles, true);
            for (actual, expected) in values.iter().zip(original) {
                assert!((actual.re - expected.re).abs() < 1.0e-9);
                assert!((actual.im - expected.im).abs() < 1.0e-9);
            }
        }
    }

    #[cfg(not(feature = "lowflow-conservative"))]
    #[test]
    fn disabled_lower_lane_is_unity_with_reported_latency() {
        let mut p = synthetic();
        p.models[0].enabled = false;
        p.models[1].enabled = false;
        let mut run = Run::new(&p, 48_000.0, 4);
        let mut b = [sine(400.0, 0.7, 2048), vec![0.0; 2048], vec![0.0; 2048], vec![0.0; 2048]];
        let original = b[0].clone();
        let mut refs: Vec<&mut [f32]> = b.iter_mut().map(Vec::as_mut_slice).collect();
        run.process(&mut refs, 2048);
        for (actual, expected) in b[0][LATENCY_FRAMES..].iter().zip(&original[..2048 - LATENCY_FRAMES]) {
            assert!((actual - expected).abs() < 2.0e-5, "{actual} != {expected}");
        }
    }

    #[cfg(not(feature = "lowflow-conservative"))]
    #[test]
    fn waterfill_hits_band_rms_target() {
        let p = synthetic();
        let mut spectral = Spectral::new(&p.models[0], 48_000.0, LATENCY_FRAMES);
        spectral.min_bin = 2;
        spectral.max_bin = 7;
        spectral.release = 1.0;
        for (i, magnitude) in [1.0, 2.0, 4.0, 8.0, 16.0].into_iter().enumerate() {
            spectral.detector_magnitude[2 + i] = magnitude;
        }
        spectral.target = 0.4;
        spectral.update_gains();
        let before = [1.0_f64, 2.0, 4.0, 8.0, 16.0];
        let input_rms = libm::sqrt(before.iter().map(|v| v * v).sum::<f64>() / before.len() as f64);
        let output_rms = libm::sqrt(
            before
                .iter()
                .enumerate()
                .map(|(i, value)| (value * spectral.gains[2 + i]) * (value * spectral.gains[2 + i]))
                .sum::<f64>()
                / before.len() as f64,
        );
        assert!((output_rms / input_rms - 0.4).abs() < 1.0e-8);
        assert!(spectral.gains[0] == 1.0 && spectral.gains[8] == 1.0);
    }

    #[cfg(not(feature = "lowflow-conservative"))]
    #[test]
    fn detector_target_uses_two_completed_hops() {
        assert_eq!(detector_target(false, false, 0.25, 0.5, [2.0, 4.0]), 1.0);
        assert_eq!(detector_target(true, true, 0.25, 0.5, [2.0, 4.0]), 0.0);
        assert_eq!(detector_target(true, false, 0.25, 0.5, [0.2, 0.4]), 1.0);
        assert_eq!(detector_target(true, false, 0.1, 0.5, [2.0, 4.0]), 0.125);
        assert_eq!(detector_target(true, false, 0.25, 0.5, [2.0, 4.0]), 0.25);
    }

    #[cfg(not(feature = "lowflow-conservative"))]
    #[test]
    fn velocity_curve_uses_and_extrapolates_the_fourth_knot() {
        let model = synthetic().models[0];
        assert_eq!(velocity_limit(&model, model.high_mask_energy_db), f64::from(model.high_velocity as f32));
        assert_eq!(velocity_limit(&model, model.max_mask_energy_db), f64::from(MAX_MASK_VELOCITY));
        let expected = libm::fmaf(2.0, MAX_MASK_VELOCITY - model.high_velocity as f32, model.high_velocity as f32);
        assert_eq!(velocity_limit(&model, 50.0), f64::from(expected));
    }

    #[cfg(not(feature = "lowflow-conservative"))]
    #[test]
    fn coupled_velocity_cascade_is_stable_and_has_two_distinct_modes() {
        let factors = factor_quartic([1.0, 6.0, 38.0, 80.0, 200.0]);
        let rebuilt = [
            1.0,
            factors[0].0 + factors[1].0,
            factors[0].1 + factors[1].1 + factors[0].0 * factors[1].0,
            factors[0].0 * factors[1].1 + factors[1].0 * factors[0].1,
            factors[0].1 * factors[1].1,
        ];
        for (actual, expected) in rebuilt.into_iter().zip([1.0, 6.0, 38.0, 80.0, 200.0]) {
            assert!((actual - expected).abs() < 1.0e-9);
        }
        let model = synthetic().models[0];
        let mut velocity = VelocityCascade::new(&model);
        assert!(velocity.sections.iter().all(|section| {
            [section.b0, section.b1, section.b2, section.a1, section.a2].iter().all(|value| value.is_finite())
                && section.a2.abs() < 1.0
        }));
        assert_ne!(velocity.sections[0].a1, velocity.sections[1].a1);
        let peak =
            (0..4096).map(|frame| velocity.run(if frame == 0 { 1.0 } else { 0.0 }).abs()).fold(0.0_f32, f32::max);
        assert!(peak.is_finite() && peak > 0.0);
    }

    #[cfg(not(feature = "lowflow-conservative"))]
    #[test]
    fn mask_grid_has_the_recovered_384_point_boundaries() {
        let boundaries = mask_boundaries(384);
        for (index, boundary) in boundaries[..MASK_DIRECT_BANDS].iter().enumerate() {
            assert_eq!(*boundary, index + 1);
        }
        assert_eq!(
            &boundaries[MASK_DIRECT_BANDS..],
            &[
                76, 78, 80, 82, 84, 86, 88, 90, 92, 95, 98, 101, 104, 107, 111, 115, 119, 124, 129, 135, 141, 148, 156,
                165, 175
            ]
        );
    }

    #[cfg(not(feature = "lowflow-conservative"))]
    #[test]
    fn mask_reads_high_lane_before_its_output_delay() {
        let p = synthetic();
        let mut run = Run::new(&p, 48_000.0, 4);
        let mut b = [vec![0.0; 128], vec![0.0; 128], vec![0.0; 128], vec![0.0; 128]];
        b[2][64] = 0.7;
        let mut refs: Vec<&mut [f32]> = b.iter_mut().map(Vec::as_mut_slice).collect();
        run.process(&mut refs, 128);
        assert!(run.state[0].mask.db > 0.0);
        assert!(run.state[0].mask_output_db > 0.0);
        assert!(b[2].iter().all(|value| *value == 0.0));
    }

    #[cfg(not(feature = "lowflow-conservative"))]
    #[test]
    fn flagged_fallback_uses_guarded_ceiling() {
        let mut p = synthetic();
        p.conservative_fallback = true;
        let mut run = Run::new(&p, 48_000.0, 4);
        let source = sine(400.0, 0.7, 1024);
        let mut b = [source.clone(), vec![0.0; 1024], vec![0.0; 1024], vec![0.0; 1024]];
        let mut refs: Vec<&mut [f32]> = b.iter_mut().map(Vec::as_mut_slice).collect();
        run.process(&mut refs, 1024);
        let ceiling = db_gain(-(12.0 + CONSERVATIVE_GUARD_DB));
        for (actual, input) in b[0][LATENCY_FRAMES..].iter().zip(&source[..1024 - LATENCY_FRAMES]) {
            assert!(f64::from(actual.abs()) <= f64::from(input.abs()) * ceiling + 1.0e-6);
        }
    }

    #[test]
    fn volume_interpolates_only_attenuation_floor() {
        let p = synthetic();
        let mut run = Run::new(&p, 48_000.0, 4);
        assert!((run.state[0].floor - db_gain(-12.0)).abs() < 1.0e-12);
        run.set_volume(-15.0);
        assert!((run.state[0].floor - db_gain(-6.0)).abs() < 1.0e-12);
        run.set_volume(-30.0);
        assert!((run.state[0].floor - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn zero_release_is_immediate() {
        #[cfg(feature = "lowflow-conservative")]
        {
            assert_eq!(coefficient(0.0, 48_000.0), 1.0);
            assert!(coefficient(0.05, 48_000.0) < 1.0);
        }
        #[cfg(not(feature = "lowflow-conservative"))]
        {
            assert_eq!(hop_coefficient(0.0, 48_000.0), 1.0);
            assert!(hop_coefficient(0.05, 48_000.0) < 1.0);
        }
    }

    #[cfg(feature = "lowflow-conservative")]
    #[test]
    fn conservative_mode_uses_guarded_fixed_ceiling() {
        let p = synthetic();
        let mut run = Run::new(&p, 48_000.0, 4);
        assert!((run.state[0].conservative_floor - db_gain(-18.0)).abs() < 1.0e-12);
        run.set_volume(-15.0);
        assert!((run.state[0].conservative_floor - db_gain(-12.0)).abs() < 1.0e-12);
        run.set_volume(-30.0);
        assert!((run.state[0].conservative_floor - db_gain(-6.0)).abs() < 1.0e-12);
        assert!(run.state[0].conservative_floor <= run.state[0].floor * db_gain(-CONSERVATIVE_GUARD_DB) + 1.0e-12);
    }

    #[test]
    fn reset_clears_delay_and_detectors() {
        let p = synthetic();
        let mut run = Run::new(&p, 48_000.0, 4);
        let mut b = [vec![1.0; 512], vec![0.0; 512], vec![0.0; 512], vec![0.0; 512]];
        let mut refs: Vec<&mut [f32]> = b.iter_mut().map(Vec::as_mut_slice).collect();
        run.process(&mut refs, 512);
        run.reset();
        assert!(run.line.iter().all(|v| *v == 0.0));
        #[cfg(feature = "lowflow-conservative")]
        {
            assert!(run.state.iter().all(|s| s.mask_energy == 0.0));
            assert!(run.state.iter().all(|s| s.gain == 1.0));
        }
        #[cfg(not(feature = "lowflow-conservative"))]
        {
            assert!(run.state.iter().all(|s| s.spectral.gains.iter().all(|g| *g == 1.0)));
            assert!(run.state.iter().all(|s| s.spectral.history.iter().all(|v| *v == 0.0)));
            assert!(run.state.iter().all(|s| s.velocity_history == [0.0; 2]));
            assert!(run.state.iter().all(|s| s.mask.db == 0.0 && s.mask.at == 0));
        }
    }
}
