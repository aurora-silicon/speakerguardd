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

/// `mvol` for a silent or invalid scalar under [`Curve::Cubic`].
pub const QUIET_DB: f64 = -200.0;

/// The speaker volume control's scalar-to-dB law, as the chain artifact
/// carries it: the HAL law over the Mac's own dB range, or the legacy
/// cubic taper when the bundle names no range.
///
/// The HAL law is the CoreAudio level control with the 2-over-1 transfer
/// function (`kAudioLevelControlTranferFunction2Over1`): the scalar is the
/// square of the dB position within the control's range, so
///
/// ```text
/// dB(s) = min_db + (max_db - min_db) * sqrt(s)
/// ```
///
/// This is measured, not assumed: macOS's built-in speaker control follows
/// it to 1e-3 dB on every Mac read so far.  The range is not a constant
/// here: the tuning compiler reads it from the Mac's own tuning bundle
/// (its device configuration), so a new Mac needs no table entry.
///
/// [`Curve::db`] is real-time safe: no allocation, lock or syscall.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Curve {
    /// No HAL range known: the PipeWire cubic taper, `60 * log10(s)`.
    #[default]
    Cubic,
    /// The HAL law over `[min_db, max_db]`; build it with [`Curve::hal`].
    Hal { min_db: f64, max_db: f64 },
}

impl Curve {
    /// A HAL law over the control's dB range.  The range must be finite,
    /// non-empty, end at 0 dB (full scale is unity, so the law never adds
    /// drive) and start no lower than [`QUIET_DB`].
    pub fn hal(min_db: f64, max_db: f64) -> Result<Self, LawError> {
        if !min_db.is_finite() || !max_db.is_finite() {
            return Err(LawError::NonFinite);
        }
        if max_db != 0.0 {
            return Err(LawError::FullScaleNotUnity);
        }
        if min_db >= max_db || min_db < QUIET_DB {
            return Err(LawError::DbOrder);
        }
        Ok(Self::Hal { min_db, max_db })
    }

    /// Re-check a decoded value (a `Hal` built without [`Curve::hal`]).
    pub fn valid(&self) -> bool {
        match *self {
            Self::Cubic => true,
            Self::Hal { min_db, max_db } => Self::hal(min_db, max_db).is_ok(),
        }
    }

    /// `mvol` for user scalar `s`.  A non-finite scalar maps to the quiet end.
    #[inline]
    pub fn db(&self, s: f64) -> f64 {
        let s = if s.is_finite() { s.clamp(0.0, 1.0) } else { 0.0 };
        match *self {
            Self::Cubic if s > 0.0 => 60.0 * libm::log10(s),
            Self::Cubic => QUIET_DB,
            Self::Hal { min_db, max_db } => (min_db + (max_db - min_db) * libm::sqrt(s)).min(0.0),
        }
    }

    /// Map a user scalar to the graph's `mvsc` and `mvol` values.
    #[inline]
    pub fn map(&self, s: f64) -> Frame {
        let scalar = if s.is_finite() { s.clamp(0.0, 1.0) } else { 0.0 };
        Frame { scalar, db: self.db(scalar) }
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
    fn hal_curve_is_the_two_over_one_law() {
        // A synthetic range: the law, not any Mac's value.
        let c = Curve::hal(-40.0, 0.0).unwrap();
        assert_eq!(c.db(1.0), 0.0);
        assert_eq!(c.db(0.0), -40.0);
        // dB position = sqrt(scalar): a quarter scalar is half the range.
        assert_eq!(c.db(0.25), -20.0);
        assert!((c.db(0.81) - -4.0).abs() < 1e-12);
        // The inverse, scalar = position^2, round-trips.
        for i in 0..=100 {
            let s = i as f64 / 100.0;
            let pos = (c.db(s) + 40.0) / 40.0;
            assert!((pos * pos - s).abs() < 1e-12, "{s}");
        }
        assert_eq!(c.map(f64::NAN), Frame { scalar: 0.0, db: -40.0 });
        assert_eq!(c.map(3.0), Frame { scalar: 1.0, db: 0.0 });
        // Monotone, never positive.
        let mut last = f64::NEG_INFINITY;
        for i in 0..=1000 {
            let d = c.db(i as f64 / 1000.0);
            assert!(d >= last && d <= 0.0);
            last = d;
        }
    }

    #[test]
    fn cubic_curve_is_the_legacy_taper() {
        let c = Curve::default();
        assert_eq!(c, Curve::Cubic);
        assert!((c.db(0.5) - 60.0 * libm::log10(0.5)).abs() < 1e-12);
        assert_eq!(c.db(1.0), 0.0);
        assert_eq!(c.db(0.0), QUIET_DB);
        assert_eq!(c.db(-1.0), QUIET_DB);
        assert_eq!(c.db(f64::NAN), QUIET_DB);
        assert_eq!(c.db(2.0), 0.0);
    }

    #[test]
    fn hal_range_that_could_add_drive_or_is_empty_is_refused() {
        assert_eq!(Curve::hal(-40.0, 1.0), Err(LawError::FullScaleNotUnity));
        assert_eq!(Curve::hal(0.0, 0.0), Err(LawError::DbOrder));
        assert_eq!(Curve::hal(10.0, 0.0), Err(LawError::DbOrder));
        assert_eq!(Curve::hal(-500.0, 0.0), Err(LawError::DbOrder));
        assert_eq!(Curve::hal(f64::NAN, 0.0), Err(LawError::NonFinite));
        assert!(!Curve::Hal { min_db: -10.0, max_db: 3.0 }.valid());
        assert!(Curve::Hal { min_db: -10.0, max_db: 0.0 }.valid());
        assert!(Curve::Cubic.valid());
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
