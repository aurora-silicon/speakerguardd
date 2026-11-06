//! Compiles the virtual bass box (`aufx/vben`) from its saved state.
//!
//! Saved-state IDs (global scope):
//!
//! | ID | meaning                                   | carried as |
//! |---:|-------------------------------------------|------------|
//! | 0  | level control in front of the device      | on / off |
//! | 1  | monitor (1 = the full processed sum)      | only 1 |
//! | 2  | crossover corner, Hz                      | `fc_hz` |
//! | 3  | crossover order (any real)                | Linkwitz-Riley order: round, then up to even |
//! | 4  | harmonic band upper corner, Hz            | `fu_hz` |
//! | 5  | harmonic band upper order                 | round, then up to even |
//! | 6  | harmonic band lower order                 | round, then up to even |
//! | 7  | harmonic gain, dB                         | linear |
//! | 8  | low-end blend gain, dB                    | linear |
//! | 9  | nonlinear device type                     | only the types carried |
//! | 10 | harmonic compressor on                    | on / off |
//! | 11 | harmonic compressor threshold, dB         | linear |
//! | 12 | harmonic compressor ratio                 | ratio |
//! | 13 | input gain, dB                            | linear |
//! | 14 | output gain, dB                           | linear |
//! | 15 | bypass                                    | `Ok(None)` |
use crate::error::{Diagnostics, Error, Result};
use crate::tuning::Stage;
use sgv2_core::voicing::vbass::{Device, Params};

fn bad(message: impl Into<String>) -> Error {
    Error::format("voicing vben", message)
}

fn db(v: f64) -> f64 {
    10f64.powf(v / 20.0)
}

/// A real order parameter becomes the Linkwitz-Riley order that carries it:
/// the nearest integer (halves up), raised to even.
fn lr_order(v: f64) -> Option<usize> {
    if !(2.0..=20.0).contains(&v) {
        return None;
    }
    let n = (v + 0.5).floor() as usize;
    Some(n + n % 2)
}

pub fn compile(stage: &Stage, fs: f64, _diags: &mut Diagnostics) -> Result<Option<Params>> {
    let ctx = format!("virtual bass box {}", stage.name);
    if stage.kind != "aufx/vben/appl" {
        return Err(bad(format!("{ctx}: is a {}, not aufx/vben/appl", stage.kind)));
    }
    if stage.bypass {
        return Ok(None);
    }
    if sgv2_core::voicing::vbass::sub_stride(fs).is_none() {
        return Err(bad(format!("{ctx}: rate {fs} Hz is not a whole multiple of the box's sub rate")));
    }
    let g = stage
        .state
        .as_ref()
        .and_then(|s| s.global())
        .ok_or_else(|| bad(format!("{ctx}: the strip gives it no saved state")))?;
    let req = |id: u32| -> Result<f64> {
        g.get(id)
            .map(f64::from)
            .filter(|v| v.is_finite())
            .ok_or_else(|| bad(format!("{ctx}: saved parameter {id} is missing")))
    };
    let opt = |id: u32| g.get(id).map(f64::from).unwrap_or(0.0);
    // absent bypass = active (the unit's default; it can only add processing)
    if opt(15) != 0.0 {
        return Ok(None);
    }
    let monitor = req(1)?;
    if monitor != 1.0 {
        return Err(bad(format!("{ctx}: monitor {monitor} (only the full sum, 1, is carried)")));
    }
    // Every value that can change the level must be in the saved state
    // (AUD: a missing negative gain must not read as 0 dB).  Compressor
    // threshold and ratio matter only when the saved switch enables it.
    let harmonic_compressor = match req(10)? {
        0.0 => false,
        1.0 => true,
        v => return Err(bad(format!("{ctx}: harmonic compressor {v}"))),
    };
    let level_control = match req(0)? {
        0.0 => false,
        1.0 => true,
        v => return Err(bad(format!("{ctx}: level control {v}"))),
    };
    let device = match req(9)? {
        3.0 => Device::Saturator3,
        v => return Err(bad(format!("{ctx}: nonlinear device type {v} is not carried"))),
    };
    let nyquist = fs / 2.0;
    let corner = |id: u32| -> Result<f64> {
        let v = req(id)?;
        if !(v > 0.0 && v < nyquist) {
            return Err(bad(format!("{ctx}: corner (parameter {id}) {v} Hz")));
        }
        Ok(v)
    };
    let order = |id: u32| -> Result<usize> {
        let v = req(id)?;
        lr_order(v).ok_or_else(|| bad(format!("{ctx}: order (parameter {id}) {v}")))
    };
    let fc_hz = corner(2)?;
    let fu_hz = corner(4)?;
    if fu_hz <= fc_hz {
        return Err(bad(format!("{ctx}: harmonic band upper corner at or below the crossover")));
    }
    let p = Params {
        fc_hz,
        hp_order: order(3)?,
        fu_hz,
        upper_order: order(5)?,
        lower_order: order(6)?,
        harmonic_gain: db(req(7)?),
        blend_gain: db(req(8)?),
        input_gain: db(req(13)?),
        output_gain: db(req(14)?),
        device,
        level_control,
        harmonic_compressor,
        compressor_threshold: if harmonic_compressor { db(req(11)?) } else { 1.0 },
        compressor_ratio: if harmonic_compressor { req(12)? } else { 1.0 },
    };
    let mut w = [0.0; Params::WORDS];
    p.to_words(&mut w);
    if Params::from_words(&w).is_none() {
        return Err(bad(format!("{ctx}: saved state out of the carried range")));
    }
    Ok(Some(p))
}

#[cfg(test)]
mod tests {
    use super::lr_order;

    #[test]
    fn orders_round_then_go_even() {
        assert_eq!(lr_order(4.0), Some(4));
        assert_eq!(lr_order(4.4), Some(4));
        assert_eq!(lr_order(4.5), Some(6));
        assert_eq!(lr_order(5.0), Some(6));
        assert_eq!(lr_order(6.4), Some(6));
        assert_eq!(lr_order(6.6), Some(8));
        assert_eq!(lr_order(2.0), Some(2));
        assert_eq!(lr_order(1.5), None);
        assert_eq!(lr_order(21.0), None);
    }
}
