//! Compile `aufx/ldnm/appl` (Norm, the loudness normalizer) from its saved
//! state.
//!
//! Carried: target loudness, maximum gain, upward and downward ranges, the
//! downward ratio (when the state enables it), the compressor clamp,
//! look-ahead, the expected loudness after a reset and the reset timeout. Everything the speaker strips leave switched off
//! (AGC, voice activity, environmental-noise adaptation, personalization,
//! volume automation, the output ceiling, the side-chain low-pass, the
//! shaping filter, other gain curves and weightings, adaptation hold) is
//! refused when switched on, so a state this module cannot carry exactly
//! never compiles.
//!
//! The side-chain high-pass (IDs 10 and 11) sets up the unit's input and
//! output loudness meters only; the gain-control detector is the loudness
//! weighting alone, so the high-pass is not part of the box.
//!
//! The system-volume parameter (ID 1, `svol`) is read only by volume
//! automation; with automation off (the only mode carried) the unit's
//! output does not depend on it, so the box has no volume input.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::tuning::Stage;
use sgv2_core::voicing::norm::{Params, MAX_SECTIONS, NODES, NODE_LO_DB, NODE_STEP_DB};

pub const LDNM: FourCc = FourCc::from_bytes(*b"ldnm");

// Saved-state parameter IDs (the unit's public parameter list).
const ENABLE: u32 = 0;
const TARGET: u32 = 2;
const RENDER_QUALITY: u32 = 3;
const MAX_GAIN: u32 = 4;
const DOWN_RANGE: u32 = 5;
const LOOKAHEAD_MS: u32 = 6;
const METER_IN: u32 = 7;
const METER_OUT: u32 = 8;
const UP_RANGE: u32 = 9;
const RESET_MODE: u32 = 12;
const VERSION: u32 = 13;
const AUTOMATION: u32 = 15;
const EXPECTED: u32 = 28;
const RESET_TIMEOUT: u32 = 29;
const AGC: u32 = 30;
const COMPRESSOR: u32 = 31;
const GAIN_CURVE: u32 = 32;
const SHAPING: u32 = 33;
const VAD_INPUT: u32 = 35;
const WEIGHTING: u32 = 36;
const USE_RATIO: u32 = 37;
const RATIO: u32 = 38;
const LP_ENABLE: u32 = 39;
const HOLD: u32 = 40;
const CEILING_ENABLE: u32 = 43;
const PEAK_OFFSET: u32 = 44;
const INPUT_GAIN: u32 = 55;
const OUTPUT_GAIN: u32 = 56;
const NOISE_CAP: u32 = 58;
const PERSONALIZATION: u32 = 61;

/// The one meter mode carried: the gated integrated meter.
const METER_GATED: f64 = 2.0;
/// The reset mode carried: history and gain reset, expected-loudness seed.
const RESET_SEEDED: f64 = 13.0;
/// The render quality carried: full-rate side chain.
const QUALITY_FULL: f64 = 3.0;
const SUPPORTED_FS: f64 = 48_000.0;
/// Fast-path release dead band by algorithm version, dB.
const FAST_DEADBAND_V2_DB: f64 = 10.0;
const FAST_DEADBAND_V3_DB: f64 = 7.0;

fn bad(message: impl Into<String>) -> Error {
    Error::format("Norm", message)
}

/// One node of the gain table: the gain for loudness `l`, dB.
pub fn node_gain(l: f64, target: f64, max_gain: f64, up: f64, down: f64, ratio: Option<f64>, clamp: bool) -> f64 {
    let mut g = if l < target - (up + max_gain) {
        max_gain
    } else if l < target {
        (target - l) * max_gain / (up + max_gain)
    } else if let Some(r) = ratio {
        (target - l) * (1.0 - 1.0 / r.min(100.0))
    } else {
        (1.0 - l / target) * (target + down)
    };
    if clamp {
        g = g.min(max_gain - l);
    }
    g
}

/// The loudness weighting of weighting mode 0: the two-section K weighting
/// of ITU-R BS.1770 (its published 48 kHz coefficients, as single-precision
/// values like the unit's).
fn weighting(fs: f64) -> Vec<[f64; 5]> {
    debug_assert_eq!(fs, SUPPORTED_FS);
    let f = |v: f64| f64::from(v as f32);
    vec![
        [
            f(1.535_124_859_586_97),
            f(-2.691_696_189_406_38),
            f(1.198_392_810_852_85),
            f(-1.690_659_293_182_41),
            f(0.732_480_774_215_85),
        ],
        [1.0, -2.0, 1.0, f(-1.990_047_454_833_98), f(0.990_072_250_366_21)],
    ]
}

pub fn compile(stage: &Stage, fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    let _ = diags;
    if stage.unit.map(|u| u.subtype) != Some(LDNM) {
        return Err(bad(format!("{} is not a loudness normalizer", stage.name)));
    }
    if stage.bypass {
        return Ok(None);
    }
    let g = stage
        .state
        .as_ref()
        .and_then(|s| s.global())
        .ok_or_else(|| bad(format!("{}: the strip gives it no saved state", stage.name)))?;
    let req = |id: u32| -> Result<f64> {
        g.get(id)
            .map(f64::from)
            .filter(|v| v.is_finite())
            .ok_or_else(|| bad(format!("{}: saved parameter {id} is missing", stage.name)))
    };
    // switches the saved state may leave out: absent is the unit's off
    let off = |id: u32, what: &str| -> Result<()> {
        match g.get(id) {
            None => Ok(()),
            Some(0.0) => Ok(()),
            Some(v) => Err(bad(format!("{}: {what} is set ({v}); not carried", stage.name))),
        }
    };
    if req(ENABLE)? == 0.0 {
        return Ok(None);
    }
    if fs != SUPPORTED_FS {
        return Err(bad(format!("sample rate {fs} Hz: only {SUPPORTED_FS} Hz is carried")));
    }
    for (id, what) in [
        (AUTOMATION, "volume automation"),
        (AGC, "AGC"),
        (GAIN_CURVE, "gain curve other than the plain curve"),
        (SHAPING, "shaping filter"),
        (VAD_INPUT, "voice-activity input"),
        (WEIGHTING, "weighting other than mode 0"),
        (LP_ENABLE, "side-chain low-pass"),
        (HOLD, "adaptation hold"),
        (CEILING_ENABLE, "output ceiling"),
        (PEAK_OFFSET, "peak level offset"),
        (INPUT_GAIN, "input gain"),
        (OUTPUT_GAIN, "output gain"),
        (NOISE_CAP, "environmental-noise gain cap"),
        (PERSONALIZATION, "personalization"),
    ] {
        off(id, what)?;
    }
    for (id, want, what) in [
        (METER_IN, METER_GATED, "input meter mode"),
        (METER_OUT, METER_GATED, "output meter mode"),
        (RESET_MODE, RESET_SEEDED, "reset mode"),
        (RENDER_QUALITY, QUALITY_FULL, "render quality"),
    ] {
        let v = req(id)?;
        if v != want {
            return Err(bad(format!("{}: {what} {v} is not carried", stage.name)));
        }
    }
    // the fast path's release dead band is the one law that differs between
    // the algorithm versions the speaker strips use
    let version = req(VERSION)?;
    let fast_deadband_db = if version == 2.0 {
        FAST_DEADBAND_V2_DB
    } else if version == 3.0 {
        FAST_DEADBAND_V3_DB
    } else {
        return Err(bad(format!("{}: algorithm version {version} is not carried", stage.name)));
    };
    let target = req(TARGET)?;
    let max_gain = req(MAX_GAIN)?;
    let up = req(UP_RANGE)?;
    let down = req(DOWN_RANGE)?;
    let clamp = req(COMPRESSOR)? != 0.0;
    let ratio = (req(USE_RATIO)? != 0.0).then(|| req(RATIO)).transpose()?;
    if target >= 0.0
        || target.is_nan()
        || !(0.0..=40.0).contains(&max_gain)
        || !(0.0..=40.0).contains(&up)
        || !(0.0..=40.0).contains(&down)
    {
        return Err(bad(format!("{}: gain law out of the unit's range", stage.name)));
    }
    if up + max_gain <= 0.0 {
        return Err(bad(format!("{}: zero upward span", stage.name)));
    }
    if ratio.is_some_and(|r| !(1.0..=100.0).contains(&r)) {
        return Err(bad(format!("{}: ratio out of the unit's range", stage.name)));
    }
    let mut table = [0.0; NODES];
    for (i, t) in table.iter_mut().enumerate() {
        *t = node_gain(NODE_LO_DB + NODE_STEP_DB * i as f64, target, max_gain, up, down, ratio, clamp);
    }
    let sos = weighting(fs);
    if sos.len() > MAX_SECTIONS {
        return Err(bad("detector needs more sections than carried"));
    }
    let lookahead = (req(LOOKAHEAD_MS)? * fs / 1000.0).round();
    let expected = req(EXPECTED)?;
    let timeout = req(RESET_TIMEOUT)?;
    let mut p = Params {
        table,
        expected_db: expected,
        reset_timeout_s: timeout,
        lookahead: lookahead.max(0.0) as usize,
        sections: sos.len(),
        fast_deadband_db,
        ..Params::default()
    };
    for (d, s) in p.sos.iter_mut().zip(&sos) {
        *d = *s;
    }
    let mut words = vec![0.0; Params::WORDS];
    p.to_words(&mut words);
    Params::from_words(&words).ok_or_else(|| bad(format!("{}: compiled parameters out of range", stage.name))).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_is_flat_below_the_span_and_zero_at_target() {
        let g = |l| node_gain(l, -20.0, 4.0, 8.0, 6.0, None, true);
        assert_eq!(g(-60.0), 4.0);
        assert!((g(-20.0)).abs() < 1e-12);
        assert!((g(-26.0) - 2.0).abs() < 1e-12);
        // above target without a ratio: linear to (target + down) at 0
        assert!((g(0.0) - (-14.0)).abs() < 1e-12);
        let r = |l| node_gain(l, -20.0, 4.0, 8.0, 6.0, Some(4.0), false);
        assert!((r(-8.0) - (-9.0)).abs() < 1e-12);
    }
}
