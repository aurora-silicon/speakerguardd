// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! AUVolumeTaper (`aufx/volt/appl`), a volume-scalar to broadband-gain map.
//!
//! The custom law has 17 uniformly spaced gain points and a five-control
//! offset curve. The gain points are linearly interpolated. The offsets use
//! the unit's uniform cubic B-spline. [`Run::set_volume`] is RT-safe and the
//! box has no latency or tail. Run-time slider and mute moves ramp like the
//! unit's ([`Run`]).

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

/// The unit's run-time ramp (macOS 25G83): a host render is cut into
/// internal chunks of at most [`CHUNK`] frames. A chunk that sees a new
/// endpoint (the slider's curve value, or zero when muted) feeds a linear
/// amplitude ramp from the old endpoint to the new one, over the chunk,
/// into a one-pole low-pass `y = p y + (1 - p) u` with `p = exp(-600 / fs)`
/// (Float32). Later chunks feed the endpoint until the filtered value is
/// within 0.01 dB of it at a chunk's end; from the next chunk on the
/// endpoint is used directly, and the filter's state stays where it was.
/// A value set before the first block is installed at once.
pub const CHUNK: usize = 512;
const SETTLED_DB: f32 = 0.01;

pub struct Run {
    params: Params,
    channels: usize,
    gain_db: f64,
    /// The slider's endpoint (linear) and the mute.
    gain: f32,
    mute: bool,
    /// The ramp: the endpoint in force, the filter's state, its pole.
    end: f32,
    y: f32,
    pole: f32,
    settled: bool,
    started: bool,
}

fn db(x: f32) -> f32 {
    20.0 * libm::log10f(x.abs() + 1e-20)
}

impl Run {
    pub fn new(params: &Params, fs: f64, channels: usize) -> Self {
        let pole = libm::exp(-600.0 / fs) as f32;
        let mut run = Self {
            params: *params,
            channels,
            gain_db: 0.0,
            gain: 1.0,
            mute: false,
            end: 1.0,
            y: 1.0,
            pole,
            settled: true,
            started: false,
        };
        run.set_volume(params.saved_scalar);
        run
    }

    pub fn reset(&mut self) {}

    pub fn latency(&self) -> usize {
        0
    }

    fn target(&self) -> f32 {
        if self.mute {
            0.0
        } else {
            self.gain
        }
    }

    fn install_if_idle(&mut self) {
        if !self.started {
            self.end = self.target();
            self.y = self.end;
            self.settled = true;
        }
    }

    /// Sets the graph's system-volume scalar (`mvsc`), in `[0, 1]`.
    pub fn set_volume(&mut self, scalar: f64) {
        self.gain_db = self.params.db_at(scalar);
        self.gain = libm::pow(10.0, self.gain_db / 20.0) as f32;
        if !self.gain.is_finite() {
            self.gain = 0.0;
        }
        self.install_if_idle();
    }

    /// The graph's system mute (`mmut`, parameter 0).
    pub fn set_mute(&mut self, mute: bool) {
        self.mute = mute;
        self.install_if_idle();
    }

    /// Applies the graph's declared initial control before the first slice
    /// and returns the read-only Gain value published as parameter 100.
    pub fn prime_volume(&mut self, scalar: f64) -> f64 {
        self.set_volume(scalar);
        self.gain_db
    }

    /// The unit's read-only Gain parameter (ID 100), used by graph relays.
    pub fn gain_db(&self) -> f64 {
        self.gain_db
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        self.started = true;
        let mut at = 0;
        while at < frames {
            let len = CHUNK.min(frames - at);
            let target = self.target();
            let (from, ramp) = if target != self.end {
                let from = self.end;
                self.end = target;
                self.settled = false;
                (from, true)
            } else {
                (self.end, false)
            };
            let direct = self.settled;
            let step = if ramp && len > 1 { (self.end - from) / (len - 1) as f32 } else { 0.0 };
            for i in 0..len {
                let g = if direct {
                    self.end
                } else {
                    let u = if ramp {
                        if len > 1 {
                            from + step * i as f32
                        } else {
                            self.end
                        }
                    } else {
                        self.end
                    };
                    self.y = self.pole * self.y + (1.0 - self.pole) * u;
                    self.y
                };
                for buf in bufs.iter_mut().take(self.channels) {
                    let input = if buf[at + i].is_finite() { buf[at + i] } else { 0.0 };
                    let output = input * g;
                    buf[at + i] = if output.is_finite() { output } else { 0.0 };
                }
            }
            if !direct && (db(self.end) - db(self.y)).abs() < SETTLED_DB {
                self.settled = true;
            }
            at += len;
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

    /// macOS 25G83 (AID44 strip, 0.15 -> 0.50 at a host-slice boundary):
    /// normalized amplitude progress every 64 frames, host slices of 512
    /// and of 64 frames.
    #[test]
    fn slider_moves_ramp_like_the_unit() {
        let p = params();
        for (q, want) in [
            (512usize, &[0.040, 0.126, 0.234, 0.352, 0.473, 0.597, 0.722, 0.847, 0.931, 0.969][..]),
            (64, &[0.321, 0.695, 0.863, 0.938][..]),
        ] {
            let mut run = Run::new(&p, 48_000.0, 1);
            run.set_volume(0.15);
            let mut buf = vec![1.0f32; q];
            run.process(&mut [&mut buf[..]], q);
            let (g0, g1) = (buf[q - 1], libm::pow(10.0, p.db_at(0.5) / 20.0) as f32);
            run.set_volume(0.5);
            let mut out = Vec::new();
            for _ in 0..(2048 / q) {
                let mut b = vec![1.0f32; q];
                run.process(&mut [&mut b[..]], q);
                out.extend_from_slice(&b);
            }
            for (k, w) in want.iter().enumerate() {
                let got = (out[(k + 1) * 64] - g0) / (g1 - g0);
                assert!((got - w).abs() < 0.006, "q{q} at {}: {got} vs {w}", (k + 1) * 64);
            }
            assert_eq!(*out.last().unwrap(), g1, "settled on the exact endpoint");
        }
    }

    #[test]
    fn mute_ramps_to_silence_and_an_initial_value_is_installed() {
        let p = params();
        let mut run = Run::new(&p, 48_000.0, 1);
        run.set_volume(0.5);
        let mut b = vec![1.0f32; 512];
        run.process(&mut [&mut b[..]], 512);
        let g = libm::pow(10.0, p.db_at(0.5) / 20.0) as f32;
        assert!(b.iter().all(|v| (*v - g).abs() < 1e-7), "installed, not ramped");
        run.set_mute(true);
        let mut b = vec![1.0f32; 512];
        run.process(&mut [&mut b[..]], 512);
        assert!(b[0] > 0.9 * g && b[511] < 0.2 * g, "ramping down");
    }

    #[test]
    fn non_finite_control_uses_quiet_endpoint() {
        let p = params();
        let mut run = Run::new(&p, 48_000.0, 1);
        run.set_volume(f64::NAN);
        assert_eq!(run.gain_db(), p.db_at(0.0));
    }
}
