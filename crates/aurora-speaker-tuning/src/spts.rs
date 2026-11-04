// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! `speaker-thiele-small` (SpTS): the per-unit factory speaker calibration
//! from the device tree.
//!
//! Layout (behaviour specification section 9), little-endian:
//!
//! ```text
//! u16 version (2), u8 record size (22), u8 record count,
//! count x 22-byte record,
//! u16 checksum
//! ```
//!
//! Per record: bytes 4..6 are an unsigned Q6.10 resistance (`R_cal = raw /
//! 1024` ohm), bytes 6..8 an unsigned Q9.7 temperature (`T_cal = raw / 128`
//! degC), and bytes 18..22 the data-source FourCC (`spA0`, ... stored
//! little-endian).  Records bind to unit channels through the data-source ID
//! and the graph's `SpeakerInputDataSourceOrder`, never by record position
//! or speaker name.
//!
//! TODO(SPEC): the remaining record bytes (0..4, 8..18: the other
//! Thiele-Small fields of the decoded calibration schema) and the checksum
//! algorithm.  They are exposed raw and not interpreted.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use serde::Serialize;

pub const VERSION: u16 = 2;
pub const RECORD_LEN: usize = 22;
const HEADER_LEN: usize = 4;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Record {
    /// Position in the file (not a channel index).
    pub position: usize,
    /// Data-source ID (`spA<n>`).
    pub data_source: FourCc,
    pub r_raw: u16,
    pub t_raw: u16,
    /// DC resistance at calibration, ohm (Q6.10).
    pub r_cal_ohm: f64,
    /// Calibration temperature, degC (Q9.7).
    pub t_cal_c: f64,
    /// The whole record, for fields the specification has not named yet.
    #[serde(serialize_with = "hex")]
    pub raw: [u8; RECORD_LEN],
}

fn hex<S: serde::Serializer>(b: &[u8; RECORD_LEN], s: S) -> std::result::Result<S::Ok, S::Error> {
    s.serialize_str(&b.iter().map(|x| format!("{x:02x}")).collect::<String>())
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Spts {
    pub version: u16,
    pub records: Vec<Record>,
    /// Stored trailer.  TODO(SPEC): algorithm; not verified.
    pub checksum_stored: u16,
}

impl Spts {
    pub fn by_source(&self, id: FourCc) -> Option<&Record> {
        self.records.iter().find(|r| r.data_source == id)
    }
}

pub fn decode(data: &[u8], ctx: &str, diags: &mut Diagnostics) -> Result<Spts> {
    if data.len() < HEADER_LEN + 2 {
        return Err(Error::format(ctx, format!("{} bytes is too short", data.len())));
    }
    let version = u16::from_le_bytes([data[0], data[1]]);
    if version != VERSION {
        return Err(Error::version(ctx, format!("SpTS version {version}; known: {VERSION}")));
    }
    let size = data[2] as usize;
    let count = data[3] as usize;
    if size != RECORD_LEN {
        return Err(Error::format(ctx, format!("record size {size}; the known layout is {RECORD_LEN}")));
    }
    if count == 0 {
        return Err(Error::format(ctx, "no records"));
    }
    let expect = HEADER_LEN + size * count + 2;
    if data.len() != expect {
        return Err(Error::format(ctx, format!("{} bytes; header says {count} records = {expect} bytes", data.len())));
    }
    let mut records: Vec<Record> = Vec::with_capacity(count);
    for i in 0..count {
        let r: [u8; RECORD_LEN] = data[HEADER_LEN + size * i..HEADER_LEN + size * (i + 1)].try_into().unwrap();
        let data_source = FourCc(u32::from_le_bytes([r[18], r[19], r[20], r[21]]));
        if !data_source.is_printable() {
            return Err(Error::format(ctx, format!("record {i}: data-source ID {data_source} is not a FourCC")));
        }
        if records.iter().any(|x| x.data_source == data_source) {
            return Err(Error::format(ctx, format!("record {i}: data-source ID {data_source} repeats")));
        }
        let r_raw = u16::from_le_bytes([r[4], r[5]]);
        let t_raw = u16::from_le_bytes([r[6], r[7]]);
        if r_raw == 0 {
            return Err(Error::range(ctx, &format!("record {i} R_cal"), "is zero"));
        }
        let unknown: Vec<usize> = (0..4).chain(8..18).filter(|&k| r[k] != 0).collect();
        if !unknown.is_empty() {
            diags.info(
                "spts-unnamed-bytes",
                ctx,
                format!("record {i} ({data_source}): nonzero bytes at {unknown:?} (TODO(SPEC) field meanings)"),
            );
        }
        records.push(Record {
            position: i,
            data_source,
            r_raw,
            t_raw,
            r_cal_ohm: f64::from(r_raw) / 1024.0,
            t_cal_c: f64::from(t_raw) / 128.0,
            raw: r,
        });
    }
    let checksum_stored = u16::from_le_bytes([data[data.len() - 2], data[data.len() - 1]]);
    diags.info("spts-checksum-unverified", ctx, "checksum not verified: TODO(SPEC) algorithm");
    Ok(Spts { version, records, checksum_stored })
}

/// Builds a record file, for synthetic fixtures.
pub fn encode(records: &[(FourCc, u16, u16)], checksum: u16) -> Vec<u8> {
    let mut d = vec![2, 0, RECORD_LEN as u8, records.len() as u8];
    for (id, r, t) in records {
        let mut rec = [0u8; RECORD_LEN];
        rec[4..6].copy_from_slice(&r.to_le_bytes());
        rec[6..8].copy_from_slice(&t.to_le_bytes());
        rec[18..22].copy_from_slice(&id.0.to_le_bytes());
        d.extend_from_slice(&rec);
    }
    d.extend_from_slice(&checksum.to_le_bytes());
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(s: &str) -> FourCc {
        FourCc::from_str_exact(s).unwrap()
    }

    #[test]
    fn decodes_a_synthetic_record() {
        let mut d = Diagnostics::default();
        let s =
            decode(&encode(&[(id("spA0"), 4096, 25 * 128), (id("spA1"), 2048, 3200)], 0x1234), "t", &mut d).unwrap();
        assert_eq!(s.records[0].r_cal_ohm, 4.0);
        assert_eq!(s.records[0].t_cal_c, 25.0);
        assert_eq!(s.records[1].r_cal_ohm, 2.0);
        assert_eq!(s.records[1].t_cal_c, 25.0);
        assert_eq!(s.by_source(id("spA1")).unwrap().position, 1);
        assert_eq!(s.checksum_stored, 0x1234);
    }

    #[test]
    fn rejects_malformed() {
        let mut d = Diagnostics::default();
        let good = encode(&[(id("spA0"), 4096, 3200)], 0);
        let mut short = good.clone();
        short.pop();
        assert!(decode(&short, "t", &mut d).is_err());
        let mut v = good.clone();
        v[0] = 3;
        assert!(matches!(decode(&v, "t", &mut d), Err(Error::Version { .. })));
        let mut sz = good.clone();
        sz[2] = 20;
        assert!(decode(&sz, "t", &mut d).is_err());
        let dup = encode(&[(id("spA0"), 4096, 3200), (id("spA0"), 4096, 3200)], 0);
        assert!(decode(&dup, "t", &mut d).is_err());
        let zero = encode(&[(id("spA0"), 0, 3200)], 0);
        assert!(decode(&zero, "t", &mut d).is_err());
    }
}
