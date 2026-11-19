// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The lsm2 masker's bucket lattice, as the installer provisions it.
//!
//! lsm2 reduces its 640-point spectrum to 100 buckets.  The bucket edges
//! are a table inside macOS's AudioDSP component, not a formula, so the
//! installer reads them from the Mac's own macOS and stores them in the
//! tuning bundle as [`NAME`]: 100 little-endian `u64` exclusive bucket ends
//! (record version [`VERSION`], format [`FORMAT`]).  The installer finds the
//! table by the rule [`parse`] checks again here: the 64 single-bin buckets
//! first (ends 1 to 64), strictly increasing, every end within the one-sided
//! spectrum (1 to 321), the last end 321.
//!
//! Without the field (a bundle collected before the installer read it) the
//! unit gets no lattice and gives no masking credit.

use crate::error::{Error, Result};

/// Bundle path of the lattice.
pub const NAME: &str = "lsm2-mask-lattice.bin";
/// The manifest record's `version` and `format`.
pub const VERSION: u32 = 1;
pub const FORMAT: &str = "u64le-exclusive-ends";
/// Buckets, and the leading single-bin buckets.
pub const ENTRIES: usize = 100;
pub const SINGLETONS: usize = 64;
/// The transform the lattice is defined for, and its exclusive last end.
pub const POINTS: u64 = 640;
pub const FINAL_END: u64 = POINTS / 2 + 1;
/// Bytes of the field.
pub const SIZE: usize = ENTRIES * 8;

/// The 100 exclusive bucket ends of a provisioned lattice, validated.
pub fn parse(bytes: &[u8]) -> Result<[u32; ENTRIES]> {
    if bytes.len() != SIZE {
        return Err(Error::format(NAME, format!("{} bytes; a lattice is {SIZE}", bytes.len())));
    }
    let ends: Vec<u64> = bytes.chunks_exact(8).map(|c| u64::from_le_bytes(c.try_into().unwrap())).collect();
    if let Some(i) = (0..SINGLETONS).find(|&i| ends[i] != i as u64 + 1) {
        return Err(Error::format(NAME, format!("bucket {i} is not the single bin {}", i + 1)));
    }
    if let Some(i) = (1..ENTRIES).find(|&i| ends[i] <= ends[i - 1]) {
        return Err(Error::format(NAME, format!("bucket ends do not increase strictly at {i}")));
    }
    if ends.iter().any(|&e| !(1..=FINAL_END).contains(&e)) {
        return Err(Error::format(NAME, format!("a bucket end lies outside 1..={FINAL_END}")));
    }
    if ends[ENTRIES - 1] != FINAL_END {
        return Err(Error::format(NAME, format!("the last bucket end is not {FINAL_END}")));
    }
    let mut out = [0u32; ENTRIES];
    for (o, e) in out.iter_mut().zip(&ends) {
        *o = *e as u32;
    }
    Ok(out)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// 1 to 64, then 36 evenly widening ends up to 321.  Synthetic: not
    /// macOS's table.
    pub(crate) fn synthetic_ends() -> Vec<u64> {
        let mut e: Vec<u64> = (1..=64).collect();
        e.extend((0..36u64).map(|j| 64 + ((j + 1) * 257).div_ceil(36)));
        e
    }

    pub(crate) fn pack(ends: &[u64]) -> Vec<u8> {
        ends.iter().flat_map(|e| e.to_le_bytes()).collect()
    }

    #[test]
    fn accepts_a_valid_lattice() {
        let ends = synthetic_ends();
        let got = parse(&pack(&ends)).unwrap();
        assert_eq!(got.iter().map(|&v| v as u64).collect::<Vec<_>>(), ends);
        assert_eq!(got[99], 321);
    }

    #[test]
    fn every_condition_is_required() {
        let ends = synthetic_ends();
        let bad = |f: &dyn Fn(&mut Vec<u64>)| {
            let mut e = ends.clone();
            f(&mut e);
            pack(&e)
        };
        for (name, bytes) in [
            ("short", pack(&ends[..99])),
            ("long", bad(&|e| e.push(322))),
            ("final edge", bad(&|e| e[99] = 320)),
            ("out of range", bad(&|e| e[99] = 322)),
            ("not strictly increasing", bad(&|e| e[70] = e[69])),
            ("singleton prefix", bad(&|e| e[10] = 12)),
            ("zero start", bad(&|e| e[0] = 0)),
            ("all zero", vec![0; SIZE]),
            ("huge end", bad(&|e| e[80] = u64::MAX)),
        ] {
            assert!(parse(&bytes).is_err(), "{name}");
        }
    }
}
