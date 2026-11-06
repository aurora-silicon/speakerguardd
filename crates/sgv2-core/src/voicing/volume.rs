// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! System-volume control for the voicing chain.
//!
//! The HAL owns the scalar-to-decibel curve.  The graph then distributes
//! both representations: `mvsc` is the scalar and `mvol` is the HAL result
//! in dB.  Keeping those two values separate matters: a variable EQ or
//! volume-taper box consumes the scalar while a `dbgn` consumes dB.
//!
//! [`Law::map`] is real-time safe: a law is validated and copied into fixed
//! storage before the stream starts; mapping performs no allocation, lock,
//! syscall or mutation.  A non-finite input maps to the quiet end.

/// Most measured points accepted for one HAL curve.
pub const MAX_POINTS: usize = 128;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point {
    pub scalar: f64,
    pub db: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LawError {
    TooFewPoints,
    TooManyPoints,
    NonFinite,
    ScalarRange,
    ScalarOrder,
    DbPositive,
    DbOrder,
    IncompleteRange,
    FullScaleNotUnity,
}

/// A measured HAL scalar-to-dB curve, linearly interpolated between points.
///
/// Points cover the complete scalar range, are strictly ordered by scalar,
/// and never rise above 0 dB.  Requiring `(1, 0 dB)` and a nondecreasing dB
/// curve prevents an invalid host curve from adding drive.
#[derive(Clone, Debug, PartialEq)]
pub struct Law {
    points: [Point; MAX_POINTS],
    len: usize,
}

impl Law {
    pub fn new(points: &[Point]) -> Result<Self, LawError> {
        if points.len() < 2 {
            return Err(LawError::TooFewPoints);
        }
        if points.len() > MAX_POINTS {
            return Err(LawError::TooManyPoints);
        }
        let mut storage = [Point::default(); MAX_POINTS];
        for (i, &p) in points.iter().enumerate() {
            if !p.scalar.is_finite() || !p.db.is_finite() {
                return Err(LawError::NonFinite);
            }
            if !(0.0..=1.0).contains(&p.scalar) {
                return Err(LawError::ScalarRange);
            }
            if p.db > 0.0 {
                return Err(LawError::DbPositive);
            }
            if i != 0 {
                if p.scalar <= points[i - 1].scalar {
                    return Err(LawError::ScalarOrder);
                }
                if p.db < points[i - 1].db {
                    return Err(LawError::DbOrder);
                }
            }
            storage[i] = p;
        }
        if points[0].scalar != 0.0 || points[points.len() - 1].scalar != 1.0 {
            return Err(LawError::IncompleteRange);
        }
        if points[points.len() - 1].db != 0.0 {
            return Err(LawError::FullScaleNotUnity);
        }
        Ok(Self { points: storage, len: points.len() })
    }

    pub fn points(&self) -> &[Point] {
        &self.points[..self.len]
    }

    /// Map a user scalar to the graph's `mvsc` and `mvol` values.
    pub fn map(&self, scalar: f64) -> Frame {
        let scalar = if scalar.is_finite() { scalar.clamp(0.0, 1.0) } else { 0.0 };
        let p = self.points();
        let upper = p.partition_point(|q| q.scalar < scalar);
        let db = if upper == 0 {
            p[0].db
        } else if upper == p.len() {
            p[p.len() - 1].db
        } else {
            let (a, b) = (p[upper - 1], p[upper]);
            let t = (scalar - a.scalar) / (b.scalar - a.scalar);
            (a.db + t * (b.db - a.db)).min(0.0)
        };
        Frame { scalar, db }
    }
}

/// The two simultaneous graph-level representations of system volume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    /// Graph parameter `mvsc`, 0 through 1.
    pub scalar: f64,
    /// Graph parameter `mvol`, never positive.
    pub db: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Input {
    /// The graph's `mvsc` value.
    Scalar,
    /// The graph's `mvol` value.
    Decibels,
}

impl Frame {
    #[inline]
    pub fn value(self, input: Input) -> f64 {
        match input {
            Input::Scalar => self.scalar,
            Input::Decibels => self.db,
        }
    }

    #[inline]
    pub fn update(self, binding: Binding) -> Update {
        Update { parameter: binding.parameter, value: self.value(binding.input) }
    }
}

/// A volume-controlled parameter understood by a voicing box.
///
/// Each box module exposes [`VolumeHook`] and accepts only its own variants:
///
/// * VarEQ: `VarEqCurrentGain` (AU ID 5, scalar);
/// * ToneMeister: `ToneSystemVolume` (AU ID 3, scalar or taper output);
/// * Norm: `NormSystemVolume` (AU ID 1/`svol`, dB);
/// * XTC: `XtcSystemVolume` (AU ID 16/`svol`, dB; it selects the saved
///   wet/dry taper);
/// * LowFlow: `LowFlowSystemVolume` (AU ID 4, dB);
/// * VolumeTaper: `VolumeTaperScalar` (AU ID 4, scalar), and makes its
///   read-only ID 100 available as `VolumeTaperOutput` for graph relays;
/// * graph `dbgn`: `GainDb` (ID 0, dB).
///
/// The graph's final `Ramp` is controlled by `rpss`, not by system volume,
/// so it deliberately has no variant here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Parameter {
    VarEqCurrentGain,
    ToneSystemVolume,
    NormSystemVolume,
    XtcSystemVolume,
    LowFlowSystemVolume,
    VolumeTaperScalar,
    VolumeTaperOutput,
    GainDb,
    CompressorVolume,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Binding {
    pub input: Input,
    pub parameter: Parameter,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Update {
    pub parameter: Parameter,
    pub value: f64,
}

/// Common volume hook required from every volume-aware box runner.
///
/// Setters and getters are called only at control boundaries.  They must not
/// allocate or lock; any coefficient rebuild is written into preallocated
/// state.  Returning `false` refuses a semantic parameter the box does not
/// implement instead of silently ignoring a volume wire.
pub trait VolumeHook {
    fn set_volume(&mut self, update: Update) -> bool;

    /// Only a VolumeTaper normally publishes a value (its parameter 100).
    fn volume_output(&self, _parameter: Parameter) -> Option<f64> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn law() -> Law {
        Law::new(&[Point { scalar: 0.0, db: -60.0 }, Point { scalar: 0.5, db: -20.0 }, Point { scalar: 1.0, db: 0.0 }])
            .unwrap()
    }

    #[test]
    fn interpolation_and_clamping() {
        let l = law();
        assert_eq!(l.map(0.25), Frame { scalar: 0.25, db: -40.0 });
        assert_eq!(l.map(0.75), Frame { scalar: 0.75, db: -10.0 });
        assert_eq!(l.map(-1.0), Frame { scalar: 0.0, db: -60.0 });
        assert_eq!(l.map(2.0), Frame { scalar: 1.0, db: 0.0 });
        assert_eq!(l.map(f64::NAN), Frame { scalar: 0.0, db: -60.0 });
    }

    #[test]
    fn rejects_a_curve_that_can_add_drive_or_has_gaps() {
        assert_eq!(Law::new(&[Point { scalar: 0.0, db: -1.0 }]), Err(LawError::TooFewPoints));
        assert_eq!(
            Law::new(&[Point { scalar: 0.1, db: -10.0 }, Point { scalar: 1.0, db: 0.0 }]),
            Err(LawError::IncompleteRange)
        );
        assert_eq!(
            Law::new(&[Point { scalar: 0.0, db: -10.0 }, Point { scalar: 1.0, db: 0.1 }]),
            Err(LawError::DbPositive)
        );
        assert_eq!(
            Law::new(&[
                Point { scalar: 0.0, db: -10.0 },
                Point { scalar: 0.5, db: -20.0 },
                Point { scalar: 1.0, db: 0.0 },
            ]),
            Err(LawError::DbOrder)
        );
    }

    #[test]
    fn binding_keeps_scalar_and_db_apart() {
        let f = law().map(0.5);
        assert_eq!(
            f.update(Binding { input: Input::Scalar, parameter: Parameter::VarEqCurrentGain }),
            Update { parameter: Parameter::VarEqCurrentGain, value: 0.5 }
        );
        assert_eq!(
            f.update(Binding { input: Input::Decibels, parameter: Parameter::GainDb }),
            Update { parameter: Parameter::GainDb, value: -20.0 }
        );
    }
}
