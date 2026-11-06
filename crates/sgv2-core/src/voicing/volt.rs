// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! AUVolumeTaper (`aufx/volt/appl`), a volume-scalar to broadband-gain map.
//!
//! The custom law has 17 uniformly spaced gain points and a five-control
//! offset curve. The gain points are linearly interpolated. The offsets use
//! the unit's uniform cubic B-spline. [`Run::set_volume`] is RT-safe and the
//! box has no latency or tail.
//!
//! Graph construction first loads the saved slider, then applies the graph's
//! declared volume scalar before the first slice. [`Run::new`] mirrors the
//! first step. The chain must call [`Run::prime_volume`] for the second and
//! use its returned ID-100 value to prime any downstream parameter relay.

pub const CURVE_POINTS: usize = 17;
pub const OFFSET_POINTS: usize = 5;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    /// Slider restored by the box's saved state before graph controls apply.
    pub saved_scalar: f64,
    pub pre_gain_db: f64,
    pub curve_db: [f64; CURVE_POINTS],
    pub offset_db: [f64; OFFSET_POINTS],
}

impl Params {
    pub const WORDS: usize = 2 + CURVE_POINTS + OFFSET_POINTS;

    pub fn to_words(&self, out: &mut [f64]) {
        if out.len() < Self::WORDS {
            return;
        }
        out[0] = self.saved_scalar;
        out[1] = self.pre_gain_db;
        out[2..2 + CURVE_POINTS].copy_from_slice(&self.curve_db);
        out[2 + CURVE_POINTS..Self::WORDS].copy_from_slice(&self.offset_db);
    }

    pub fn from_words(words: &[f64]) -> Option<Self> {
        if words.len() != Self::WORDS || words.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let mut curve_db = [0.0; CURVE_POINTS];
        let mut offset_db = [0.0; OFFSET_POINTS];
        curve_db.copy_from_slice(&words[2..2 + CURVE_POINTS]);
        offset_db.copy_from_slice(&words[2 + CURVE_POINTS..]);
        let p = Self { saved_scalar: words[0], pre_gain_db: words[1], curve_db, offset_db };
        p.valid().then_some(p)
    }

    fn valid(&self) -> bool {
        (0.0..=1.0).contains(&self.saved_scalar)
            && (-120.0..=20.0).contains(&self.pre_gain_db)
            && self.curve_db.iter().all(|v| (-120.0..=20.0).contains(v))
            && self.offset_db.iter().all(|v| (-120.0..=20.0).contains(v))
    }

    fn db_at(&self, scalar: f64) -> f64 {
        let scalar = if scalar.is_finite() { scalar.clamp(0.0, 1.0) } else { 0.0 };
        let curve_pos = scalar * (CURVE_POINTS - 1) as f64;
        let curve_index = libm::floor(curve_pos) as usize;
        let curve = if curve_index >= CURVE_POINTS - 1 {
            self.curve_db[CURVE_POINTS - 1]
        } else {
            let fraction = curve_pos - curve_index as f64;
            self.curve_db[curve_index] + fraction * (self.curve_db[curve_index + 1] - self.curve_db[curve_index])
        };

        let offset_pos = scalar * (OFFSET_POINTS - 1) as f64;
        let centre = libm::floor(offset_pos) as isize;
        let mut offset = 0.0;
        for index in centre - 1..=centre + 2 {
            let control = index.clamp(0, (OFFSET_POINTS - 1) as isize) as usize;
            offset += self.offset_db[control] * cubic_bspline(offset_pos - index as f64);
        }
        self.pre_gain_db + curve + offset
    }
}

/// Centred cardinal cubic B-spline basis, with support `[-2, 2]`.
fn cubic_bspline(x: f64) -> f64 {
    let x = x.abs();
    if x < 1.0 {
        (4.0 - 6.0 * x * x + 3.0 * x * x * x) / 6.0
    } else if x < 2.0 {
        let y = 2.0 - x;
        y * y * y / 6.0
    } else {
        0.0
    }
}

pub struct Run {
    params: Params,
    channels: usize,
    gain_db: f64,
    gain: f32,
}

impl Run {
    pub fn new(params: &Params, _fs: f64, channels: usize) -> Self {
        let mut run = Self { params: *params, channels, gain_db: 0.0, gain: 1.0 };
        run.set_volume(params.saved_scalar);
        run
    }

    pub fn reset(&mut self) {}

    pub fn latency(&self) -> usize {
        0
    }

    /// Sets the graph's system-volume scalar (`mvsc`), in `[0, 1]`.
    pub fn set_volume(&mut self, scalar: f64) {
        self.gain_db = self.params.db_at(scalar);
        self.gain = libm::pow(10.0, self.gain_db / 20.0) as f32;
        if !self.gain.is_finite() {
            self.gain = 0.0;
        }
    }

    /// Applies the graph's declared initial control before the first slice.
    ///
    /// The returned value is the box's published Gain (ID 100). A graph
    /// relay uses it to initialize the compressor automation input before
    /// either box processes audio. No silent pre-roll is needed.
    pub fn prime_volume(&mut self, scalar: f64) -> f64 {
        self.set_volume(scalar);
        self.gain_db
    }

    /// The unit's read-only Gain parameter (ID 100), used by graph relays.
    pub fn gain_db(&self) -> f64 {
        self.gain_db
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        for buf in bufs.iter_mut().take(self.channels) {
            for sample in buf.iter_mut().take(frames) {
                let input = if sample.is_finite() { *sample } else { 0.0 };
                let output = input * self.gain;
                *sample = if output.is_finite() { output } else { 0.0 };
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> Params {
        let mut curve = [0.0; CURVE_POINTS];
        for (i, value) in curve.iter_mut().enumerate() {
            *value = i as f64 - 16.0;
        }
        Params { saved_scalar: 0.25, pre_gain_db: -1.0, curve_db: curve, offset_db: [0.0, 0.0, 0.0, 6.0, 6.0] }
    }

    #[test]
    fn words_round_trip_and_reject_bad_values() {
        let p = params();
        let mut words = [0.0; Params::WORDS];
        p.to_words(&mut words);
        assert_eq!(Params::from_words(&words), Some(p));
        words[4] = f64::NAN;
        assert_eq!(Params::from_words(&words), None);
        assert_eq!(Params::from_words(&words[..Params::WORDS - 1]), None);
    }

    #[test]
    fn linear_curve_and_cubic_offset() {
        let p = params();
        // At scalar 0.5 the linear curve is -8 dB and the cubic spline
        // gives one sixth of the first non-zero offset: +1 dB.
        assert!((p.db_at(0.5) - -8.0).abs() < 1e-12);
        assert!((p.db_at(0.625) - -4.0).abs() < 1e-12);
    }

    #[test]
    fn process_is_broadband_and_silences_non_finite_input() {
        let p = Params {
            saved_scalar: 0.25,
            pre_gain_db: 0.0,
            curve_db: [-6.0; CURVE_POINTS],
            offset_db: [0.0; OFFSET_POINTS],
        };
        let mut run = Run::new(&p, 48_000.0, 2);
        run.set_volume(0.37);
        let mut left = [1.0, f32::NAN];
        let mut right = [-1.0, f32::INFINITY];
        run.process(&mut [&mut left, &mut right], 2);
        let want = 10f32.powf(-6.0 / 20.0);
        assert!((left[0] - want).abs() < 1e-6);
        assert!((right[0] + want).abs() < 1e-6);
        assert_eq!((left[1], right[1]), (0.0, 0.0));
        assert_eq!(run.latency(), 0);
    }

    #[test]
    fn non_finite_control_uses_quiet_endpoint() {
        let p = params();
        let mut run = Run::new(&p, 48_000.0, 1);
        run.set_volume(f64::NAN);
        assert_eq!(run.gain_db(), p.db_at(0.0));
    }

    #[test]
    fn startup_loads_saved_then_primes_declared_control_and_relay() {
        let p = params();
        let mut run = Run::new(&p, 48_000.0, 1);
        assert_eq!(run.gain_db(), p.db_at(p.saved_scalar));

        let relay = run.prime_volume(0.75);
        assert_eq!(relay, p.db_at(0.75));
        let mut audio = [1.0f32; 2];
        run.process(&mut [&mut audio], 2);
        let want = libm::pow(10.0, relay / 20.0) as f32;
        assert_eq!(audio, [want; 2]);

        run.reset();
        assert_eq!(run.gain_db(), relay);
    }
}
