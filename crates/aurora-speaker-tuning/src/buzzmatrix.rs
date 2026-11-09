// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The BuzzKill detector's quadrature matrix the installer extracts.
//!
//! macOS's BuzzKill unit filters its 12 kHz analysis stream through a
//! two-output matrix of second-order sections before the transform. Its
//! coefficients are a table inside the Mac's own AudioDSP component, so the
//! installer reads them from the user's macOS (`buzzkill_matrix.py`) into
//! the tuning bundle, as it does the lsm2 lattice, and this crate never
//! carries them. This module checks the bundle's copy with the installer's
//! structural rule and hands the voicing compiler the amplitude-record
//! cascade (the matrix's second output). A bundle without a valid matrix
//! compiles no Buzz box: the voicing reports it and the Mac stays on the
//! interim voicing.

use crate::error::{Error, Result};

/// The bundle file the installer writes.
pub const NAME: &str = "buzzkill-quadrature.bin";
/// The installer's record version and byte format.
pub const VERSION: u32 = 1;
pub const FORMAT: &str = "f32le-sos4-output-major";
/// Four second-order sections `[b0, b1, b2, a1, a2]`, output-major.
pub const SECTIONS: usize = 4;
pub const SIZE: usize = SECTIONS * 5 * 4;

/// The matrix: output 0 sections 0 and 1, then output 1 sections 0 and 1.
pub type Matrix = [[f32; 5]; SECTIONS];

fn stable(s: &[f32; 5]) -> bool {
    sgv2_core::voicing::buzz::stable_section(s)
}

fn pattern(s: &[f32; 5], unit: bool, b0_is_a2: bool, b1_is_a1: bool) -> bool {
    let [b0, b1, b2, a1, a2] = *s;
    (b2 == 1.0) == unit && (b0 == a2) == b0_is_a2 && (b1 == a1) == b1_is_a1
}

/// The matrix in `bytes`, checked with the installer's structural rule: 80
/// little-endian binary32 words, every section finite and strictly stable,
/// and the four sections' equality pattern (see `buzzkill_matrix.py`).
pub fn parse(bytes: &[u8]) -> Result<Matrix> {
    if bytes.len() != SIZE {
        return Err(Error::format(NAME, format!("{} bytes, expected {SIZE}", bytes.len())));
    }
    let mut m = [[0.0f32; 5]; SECTIONS];
    for (k, word) in bytes.chunks_exact(4).enumerate() {
        m[k / 5][k % 5] = f32::from_le_bytes([word[0], word[1], word[2], word[3]]);
    }
    if !m.iter().all(stable) {
        return Err(Error::format(NAME, "a section is not finite and stable"));
    }
    let shaped = pattern(&m[0], true, true, false)
        && pattern(&m[1], true, true, true)
        && pattern(&m[2], true, false, true)
        && pattern(&m[3], false, false, false);
    if !shaped {
        return Err(Error::format(NAME, "the sections do not have the matrix's structure"));
    }
    Ok(m)
}

/// The development override: `SGV2_BUZZKILL_MATRIX` names an 80-byte
/// matrix file (extracted with `buzzkill_matrix.py --out`). Rigs and the
/// parity gate use it for raw tuning trees, which have no installer bundle;
/// an installed bundle never needs it.
pub fn from_env() -> Result<Option<Matrix>> {
    match std::env::var_os("SGV2_BUZZKILL_MATRIX") {
        None => Ok(None),
        Some(path) => {
            let bytes =
                std::fs::read(&path).map_err(|e| Error::format(NAME, format!("SGV2_BUZZKILL_MATRIX {path:?}: {e}")))?;
            parse(&bytes).map(Some)
        }
    }
}

/// The amplitude-record cascade: the matrix's second output.
pub fn amplitude_cascade(m: &Matrix) -> [[f32; 5]; 2] {
    [m[2], m[3]]
}

#[cfg(test)]
mod tests {
    use super::*;

    // Synthetic sections with the structure only (not the unit's values).
    fn synthetic() -> Matrix {
        [
            [0.25, -0.5, 1.0, 0.1, 0.25],
            [0.3, -0.4, 1.0, -0.4, 0.3],
            [0.7, 0.2, 1.0, 0.2, -0.1],
            [1.1, -1.9, 0.9, -1.8, 0.81],
        ]
    }

    fn bytes(m: &Matrix) -> Vec<u8> {
        m.iter().flatten().flat_map(|v| v.to_le_bytes()).collect()
    }

    #[test]
    fn the_structural_rule_accepts_the_shape_and_returns_output_one() {
        let m = synthetic();
        let parsed = parse(&bytes(&m)).unwrap();
        assert_eq!(parsed, m);
        assert_eq!(amplitude_cascade(&parsed), [m[2], m[3]]);
    }

    #[test]
    fn anything_else_is_refused() {
        assert!(parse(&bytes(&synthetic())[..76]).is_err());
        let mut m = synthetic();
        m[1][0] = 0.31; // section 2 no longer an exact allpass
        assert!(parse(&bytes(&m)).is_err());
        let mut m = synthetic();
        m[3][4] = 1.2; // unstable
        assert!(parse(&bytes(&m)).is_err());
        let mut m = synthetic();
        m[0][2] = f32::NAN;
        assert!(parse(&bytes(&m)).is_err());
    }
}
