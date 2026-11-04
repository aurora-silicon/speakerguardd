// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! `speaker-thiele-small` (SpTS): the per-unit factory speaker calibration.
//!
//! Layout: little-endian header `u16 version, u8 record size, u8 count`,
//! `count` records, then a `u16` CRC. Per record: `u16` at offset 4 is the
//! DC resistance (raw / 1024 ohm), `u16` at offset 6 the calibration
//! temperature (raw / 128 degrees C). Version 2 is the known layout.
//!
//! TODO(lane A): CRC verification once its polynomial is established, and
//! the record's driver index and tag.

use crate::{Error, Result};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Driver {
    pub rdc_ohm: f32,
    pub tcal_c: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Spts {
    pub version: u16,
    pub drivers: Vec<Driver>,
    pub crc_stored: u16,
}

pub fn decode(data: &[u8]) -> Result<Spts> {
    let bad = |m: &str| Error::Invalid(format!("speaker-thiele-small: {m}"));
    if data.len() < 6 {
        return Err(bad("short"));
    }
    let version = u16::from_le_bytes([data[0], data[1]]);
    let size = data[2] as usize;
    let count = data[3] as usize;
    if version != 2 {
        return Err(bad("version"));
    }
    if size < 8 || data.len() != 4 + size * count + 2 {
        return Err(bad("length"));
    }
    let drivers = (0..count)
        .map(|i| {
            let r = &data[4 + size * i..4 + size * (i + 1)];
            Driver {
                rdc_ohm: u16::from_le_bytes([r[4], r[5]]) as f32 / 1024.0,
                tcal_c: u16::from_le_bytes([r[6], r[7]]) as f32 / 128.0,
            }
        })
        .collect();
    let crc_stored = u16::from_le_bytes([data[data.len() - 2], data[data.len() - 1]]);
    Ok(Spts { version, drivers, crc_stored })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic(rdc: &[u16]) -> Vec<u8> {
        let mut d = vec![2, 0, 22, rdc.len() as u8];
        for &r in rdc {
            let mut rec = [0u8; 22];
            rec[4..6].copy_from_slice(&r.to_le_bytes());
            rec[6..8].copy_from_slice(&(25u16 * 128).to_le_bytes());
            d.extend_from_slice(&rec);
        }
        d.extend_from_slice(&[0x34, 0x12]);
        d
    }

    #[test]
    fn decodes_a_synthetic_record() {
        let s = decode(&synthetic(&[4096, 2048])).unwrap();
        assert_eq!(s.drivers, vec![Driver { rdc_ohm: 4.0, tcal_c: 25.0 }, Driver { rdc_ohm: 2.0, tcal_c: 25.0 }]);
        assert_eq!(s.crc_stored, 0x1234);
    }

    #[test]
    fn rejects_malformed() {
        let mut d = synthetic(&[4096]);
        d.pop();
        assert!(decode(&d).is_err());
        let mut v = synthetic(&[4096]);
        v[0] = 3;
        assert!(decode(&v).is_err());
    }
}
