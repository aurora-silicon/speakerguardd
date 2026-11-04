// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The flat parameter region of an AU preset `data` blob inside a strip:
//! a 12-byte header, then big-endian `(u32 parameter ID, f32 value)`
//! records. atsp and lsm2 keep their parameters here; spp3 also has a
//! sidecar plist. Names for IDs come from the behaviour specification
//! (TODO(SPEC)); until then callers get raw IDs.
//!
//! TODO(lane A): the region's end for every unit (some blobs carry nested
//! data after the flat records), validated against the specification.

pub const HEADER_LEN: usize = 12;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParamRecord {
    pub id: u32,
    pub value: f32,
}

/// Decode `count` records after the header. Fails if the blob is short.
pub fn flat_records(blob: &[u8], count: usize) -> Option<Vec<ParamRecord>> {
    let end = HEADER_LEN.checked_add(count.checked_mul(8)?)?;
    let body = blob.get(HEADER_LEN..end)?;
    Some(
        body.chunks_exact(8)
            .map(|r| ParamRecord {
                id: u32::from_be_bytes([r[0], r[1], r[2], r[3]]),
                value: f32::from_be_bytes([r[4], r[5], r[6], r[7]]),
            })
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_synthetic_records() {
        let mut blob = vec![0u8; HEADER_LEN];
        for (id, v) in [(1u32, 2.5f32), (7, -15.0)] {
            blob.extend_from_slice(&id.to_be_bytes());
            blob.extend_from_slice(&v.to_be_bytes());
        }
        let recs = flat_records(&blob, 2).unwrap();
        assert_eq!(recs, vec![ParamRecord { id: 1, value: 2.5 }, ParamRecord { id: 7, value: -15.0 }]);
        assert!(flat_records(&blob, 3).is_none());
    }
}
