// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! `speaker-thiele-small` (SpTS): the per-unit factory speaker calibration
//! from the device tree.
//!
//! Layout (Daybreak t-4672, the SpTS wire layout and binding
//! specification), little-endian:
//!
//! ```text
//! u16 version (2), u8 size, u8 count,
//! N x 22-byte record,
//! u16 checksum
//! ```
//!
//! The file describes itself.  Version 2 fixes the record stride at 22
//! bytes; the size byte takes one of two meanings, told apart by the
//! file's own length `L`, never by the board:
//!
//! * **total length**: size byte `== L`; the count byte is `N` or 0 (0 =
//!   infer `N = (L - 6) / 22`);
//! * **record stride**: size byte `== 22`; the count byte is `N`.
//!
//! Both forms occur (a four-record total-length file on the M2, a
//! six-record stride file on the M4 Pro).  `N` always follows from `L`, and
//! every inconsistency (unknown version, a size byte that is neither, a
//! count that disagrees, truncation, trailing bytes) is refused rather
//! than reinterpreted.
//!
//! Per record: nine u16 fields and a u32 data-source ID (see [`Record`]),
//! each u16 scaled by a decimal constant (`R_cal = raw * 0.001` ohm,
//! `T_cal = raw * 0.01` degC), as a single-precision product.
//!
//! **Binding is positional**: record `i` is the calibration of unit
//! channel `i`.  The data-source ID (`spA0`, ...) is provenance only: it is
//! not an index, a sort key or a join key, and IDs need not be dense (the
//! M2's four records are `spA0 spA1 spA4 spA5`).
//!
//! The trailer is a 16-bit additive checksum: the two's complement of the
//! byte sum of everything before it, little-endian.  A record file whose
//! checksum fails is decoded (for the dump) but marked unusable.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use serde::Serialize;

pub const VERSION: u16 = 2;
pub const RECORD_LEN: usize = 22;
const HEADER_LEN: usize = 4;
const TRAILER_LEN: usize = 2;
/// The checksum must start at an offset a byte can hold; larger files would
/// need a format nobody has specified (at most 11 records).
const MAX_CHECKSUM_OFFSET: usize = 255;

/// What the header's size byte means in this file.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum SizeForm {
    /// The size byte is the file length.
    TotalLength,
    /// The size byte is the record stride (22).
    RecordStride,
}

/// One record: nine u16 fields at offsets 0, 2, ... 16 and the u32
/// data-source ID at 18.  The scaled values are what the unit receives.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Record {
    /// Position in the file: the unit channel this record calibrates.
    pub position: usize,
    /// Data-source ID (`spA<n>`), provenance only.
    pub data_source: FourCc,
    pub r_raw: u16,
    pub t_raw: u16,
    /// DC resistance at calibration, ohm (`raw * 0.001`).
    pub r_cal_ohm: f64,
    /// Calibration temperature, degC (`raw * 0.01`).
    pub t_cal_c: f64,
    /// The other Thiele-Small fields, scaled.  Decoded for the dump; the
    /// engine does not use them.
    pub mechanical: Mechanical,
    #[serde(serialize_with = "hex")]
    pub raw: [u8; RECORD_LEN],
}

/// The record's other fields, each with its scale constant.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Mechanical {
    pub fs: f64,
    pub qts: f64,
    pub le: f64,
    pub kms: f64,
    pub mms: f64,
    pub bl: f64,
    pub rms: f64,
}

fn hex<S: serde::Serializer>(b: &[u8; RECORD_LEN], s: S) -> std::result::Result<S::Ok, S::Error> {
    s.serialize_str(&b.iter().map(|x| format!("{x:02x}")).collect::<String>())
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Spts {
    pub version: u16,
    pub form: SizeForm,
    pub records: Vec<Record>,
    pub checksum_stored: u16,
    /// The stored checksum equals the computed one.
    pub checksum_valid: bool,
}

/// The two's complement of the 16-bit byte sum of `body`.
pub fn checksum(body: &[u8]) -> u16 {
    let sum = body.iter().fold(0u16, |s, b| s.wrapping_add(u16::from(*b)));
    sum.wrapping_neg()
}

/// `raw * k` in single precision, the conversion the unit applies.
fn scaled(raw: u16, k: f32) -> f64 {
    f64::from(f32::from(raw) * k)
}

fn word(r: &[u8; RECORD_LEN], at: usize) -> u16 {
    u16::from_le_bytes([r[at], r[at + 1]])
}

pub fn decode(data: &[u8], ctx: &str, diags: &mut Diagnostics) -> Result<Spts> {
    let len = data.len();
    if len < HEADER_LEN + RECORD_LEN + TRAILER_LEN {
        return Err(Error::format(ctx, format!("{len} bytes is too short for one record")));
    }
    let version = u16::from_le_bytes([data[0], data[1]]);
    if version != VERSION {
        return Err(Error::version(ctx, format!("SpTS version {version}; known: {VERSION}")));
    }
    if (len - HEADER_LEN - TRAILER_LEN) % RECORD_LEN != 0 {
        return Err(Error::format(ctx, format!("{len} bytes is not a whole number of {RECORD_LEN}-byte records")));
    }
    let n = (len - HEADER_LEN - TRAILER_LEN) / RECORD_LEN;
    if HEADER_LEN + RECORD_LEN * n > MAX_CHECKSUM_OFFSET {
        return Err(Error::format(ctx, format!("{n} records is more than version {VERSION} can frame")));
    }
    let (size, count) = (data[2] as usize, data[3] as usize);
    let form = if size == len {
        if count != 0 && count != n {
            return Err(Error::format(ctx, format!("{len} bytes holds {n} records; the header says {count}")));
        }
        SizeForm::TotalLength
    } else if size == RECORD_LEN {
        if count != n {
            return Err(Error::format(ctx, format!("{len} bytes holds {n} records; the header says {count}")));
        }
        SizeForm::RecordStride
    } else {
        return Err(Error::format(
            ctx,
            format!("size byte {size} is neither the file length ({len}) nor the record size ({RECORD_LEN})"),
        ));
    };
    let mut records: Vec<Record> = Vec::with_capacity(n);
    for i in 0..n {
        let at = HEADER_LEN + RECORD_LEN * i;
        let r: [u8; RECORD_LEN] = data[at..at + RECORD_LEN].try_into().unwrap();
        let data_source = FourCc(u32::from_le_bytes([r[18], r[19], r[20], r[21]]));
        let (r_raw, t_raw) = (word(&r, 4), word(&r, 6));
        if r_raw == 0 {
            return Err(Error::range(ctx, &format!("record {i} R_cal"), "is zero"));
        }
        if records.iter().any(|x| x.data_source == data_source) {
            // Binding is positional, so a repeated ID does not change it.
            diags.info("spts-source-repeat", ctx, format!("record {i}: data-source ID {data_source} repeats"));
        }
        records.push(Record {
            position: i,
            data_source,
            r_raw,
            t_raw,
            r_cal_ohm: scaled(r_raw, 0.001),
            t_cal_c: scaled(t_raw, 0.01),
            mechanical: Mechanical {
                fs: scaled(word(&r, 0), 0.1),
                qts: scaled(word(&r, 2), 0.001),
                le: scaled(word(&r, 8), 0.01),
                kms: scaled(word(&r, 10), 1.0),
                mms: scaled(word(&r, 12), 0.0001),
                bl: scaled(word(&r, 14), 0.0001),
                rms: scaled(word(&r, 16), 0.001),
            },
            raw: r,
        });
    }
    let checksum_stored = u16::from_le_bytes([data[len - 2], data[len - 1]]);
    let checksum_valid = checksum(&data[..len - TRAILER_LEN]) == checksum_stored;
    if !checksum_valid {
        diags.warn(
            "spts-checksum",
            ctx,
            format!("checksum 0x{checksum_stored:04x} does not match; the calibration is unusable (open loop)"),
        );
    }
    Ok(Spts { version, form, records, checksum_stored, checksum_valid })
}

/// Builds a record file in the stride form with a valid checksum, for
/// synthetic fixtures.  (`checksum` is ignored; kept for call-site
/// compatibility.)
pub fn encode(records: &[(FourCc, u16, u16)], _checksum: u16) -> Vec<u8> {
    encode_as(SizeForm::RecordStride, records)
}

/// Builds a record file in either header form with a valid checksum.
pub fn encode_as(form: SizeForm, records: &[(FourCc, u16, u16)]) -> Vec<u8> {
    let len = HEADER_LEN + RECORD_LEN * records.len() + TRAILER_LEN;
    let size = match form {
        SizeForm::TotalLength => len,
        SizeForm::RecordStride => RECORD_LEN,
    };
    let mut d = vec![2, 0, size as u8, records.len() as u8];
    for (id, r, t) in records {
        let mut rec = [0u8; RECORD_LEN];
        rec[4..6].copy_from_slice(&r.to_le_bytes());
        rec[6..8].copy_from_slice(&t.to_le_bytes());
        rec[18..22].copy_from_slice(&id.0.to_le_bytes());
        d.extend_from_slice(&rec);
    }
    let c = checksum(&d);
    d.extend_from_slice(&c.to_le_bytes());
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(s: &str) -> FourCc {
        FourCc::from_str_exact(s).unwrap()
    }

    fn ids(v: &[&str]) -> Vec<(FourCc, u16, u16)> {
        v.iter().enumerate().map(|(i, s)| (id(s), 3000 + i as u16, 2500 + i as u16)).collect()
    }

    /// Recomputes the trailer after an edit.
    fn reseal(d: &mut [u8]) {
        let n = d.len();
        let c = checksum(&d[..n - 2]);
        d[n - 2..].copy_from_slice(&c.to_le_bytes());
    }

    #[test]
    fn decodes_a_synthetic_record() {
        let mut d = Diagnostics::default();
        let s = decode(&encode(&[(id("spA0"), 4000, 2500), (id("spA1"), 2048, 3200)], 0x1234), "t", &mut d).unwrap();
        assert_eq!(s.form, SizeForm::RecordStride);
        assert_eq!(s.records[0].r_cal_ohm, f64::from(4000f32 * 0.001));
        assert!((s.records[0].r_cal_ohm - 4.0).abs() < 1e-6);
        assert!((s.records[0].t_cal_c - 25.0).abs() < 1e-5);
        assert!((s.records[1].r_cal_ohm - 2.048).abs() < 1e-6);
        assert!((s.records[1].t_cal_c - 32.0).abs() < 1e-5);
        assert!(s.checksum_valid);
        let mut bad = encode(&[(id("spA0"), 4096, 3200)], 0);
        let n = bad.len();
        bad[n - 1] ^= 1;
        assert!(!decode(&bad, "t", &mut d).unwrap().checksum_valid);
        // Additive: 1 + 2 + 3 = 6, stored as -6.
        assert_eq!(checksum(&[1, 2, 3]), 0xFFFA);
    }

    #[test]
    fn scales_are_decimal_not_fixed_point() {
        // Regression: R and T were once read as Q6.10 (raw / 1024) and Q9.7
        // (raw / 128).  The unit's scales are decimal: raw * 0.001 ohm and
        // raw * 0.01 degC, as Float32 products.
        let mut d = Diagnostics::default();
        let s = decode(&encode(&[(id("spA0"), 3500, 3000)], 0), "t", &mut d).unwrap();
        let r = &s.records[0];
        assert_eq!(r.r_cal_ohm, f64::from(3500f32 * 0.001));
        assert_eq!(r.t_cal_c, f64::from(3000f32 * 0.01));
        assert!((r.r_cal_ohm - 3.5).abs() < 1e-6 && (r.t_cal_c - 30.0).abs() < 1e-5);
        assert!((r.r_cal_ohm - 3500.0 / 1024.0).abs() > 0.05 && (r.t_cal_c - 3000.0 / 128.0).abs() > 5.0);
    }

    #[test]
    fn both_header_forms_decode_the_same_records() {
        // Shaped like the two samples: four sparse IDs in the total-length
        // form, six dense IDs in the stride form.
        let mut d = Diagnostics::default();
        let four = ids(&["spA0", "spA1", "spA4", "spA5"]);
        let a = encode_as(SizeForm::TotalLength, &four);
        assert_eq!((a.len(), a[2], a[3]), (94, 94, 4));
        let s = decode(&a, "t", &mut d).unwrap();
        assert_eq!(s.form, SizeForm::TotalLength);
        let got: Vec<String> = s.records.iter().map(|r| r.data_source.to_string()).collect();
        assert_eq!(got, ["spA0", "spA1", "spA4", "spA5"]);
        assert_eq!(s.records.iter().map(|r| r.position).collect::<Vec<_>>(), [0, 1, 2, 3]);
        assert_eq!(s.records, decode(&encode_as(SizeForm::RecordStride, &four), "t", &mut d).unwrap().records);
        let six = encode_as(SizeForm::RecordStride, &ids(&["spA0", "spA1", "spA2", "spA3", "spA4", "spA5"]));
        assert_eq!((six.len(), six[2], six[3]), (138, 22, 6));
        assert_eq!(decode(&six, "t", &mut d).unwrap().records.len(), 6);
        // The total-length form may leave the count to the length.
        let mut z = a.clone();
        z[3] = 0;
        reseal(&mut z);
        assert_eq!(decode(&z, "t", &mut d).unwrap().records.len(), 4);
    }

    #[test]
    fn stored_order_is_kept() {
        // Reversed IDs stay in stored order: no sorting by ID.
        let mut d = Diagnostics::default();
        let s = decode(&encode(&ids(&["spA5", "spA4", "spA1", "spA0"]), 0), "t", &mut d).unwrap();
        let got: Vec<(String, u16)> = s.records.iter().map(|r| (r.data_source.to_string(), r.r_raw)).collect();
        assert_eq!(got[0], ("spA5".into(), 3000));
        assert_eq!(got[3], ("spA0".into(), 3003));
    }

    #[test]
    fn mechanical_fields_scale() {
        let mut d = Diagnostics::default();
        let mut f = encode(&ids(&["spA0"]), 0);
        for (k, v) in [(0usize, 1000u16), (2, 500), (8, 300), (10, 7), (12, 20), (14, 40), (16, 900)] {
            f[4 + k..6 + k].copy_from_slice(&v.to_le_bytes());
        }
        reseal(&mut f);
        let m = decode(&f, "t", &mut d).unwrap().records[0].mechanical;
        let close = |a: f64, b: f64| (a - b).abs() < 1e-6;
        assert!(close(m.fs, 100.0) && close(m.qts, 0.5) && close(m.le, 3.0) && close(m.kms, 7.0));
        assert!(close(m.mms, 0.002) && close(m.bl, 0.004) && close(m.rms, 0.9));
    }

    #[test]
    fn rejects_malformed() {
        let mut d = Diagnostics::default();
        let good = encode(&ids(&["spA0", "spA1"]), 0);
        let total = encode_as(SizeForm::TotalLength, &ids(&["spA0", "spA1"]));
        let edit = |base: &[u8], at: usize, v: u8| {
            let mut x = base.to_vec();
            x[at] = v;
            reseal(&mut x);
            x
        };
        // Truncated, trailing bytes, no records.
        let mut short = good.clone();
        short.pop();
        assert!(decode(&short, "t", &mut d).is_err());
        let mut long = good.clone();
        long.push(0);
        assert!(decode(&long, "t", &mut d).is_err());
        assert!(decode(&[2, 0, 6, 0, 0xf8, 0xff], "t", &mut d).is_err());
        // Unknown version.
        assert!(matches!(decode(&edit(&good, 0, 3), "t", &mut d), Err(Error::Version { .. })));
        // A size byte that is neither the length nor the stride.
        assert!(decode(&edit(&good, 2, 20), "t", &mut d).is_err());
        // Counts that disagree with the length, in both forms.
        assert!(decode(&edit(&good, 3, 1), "t", &mut d).is_err());
        assert!(decode(&edit(&good, 3, 0), "t", &mut d).is_err());
        assert!(decode(&edit(&total, 3, 1), "t", &mut d).is_err());
        assert!(decode(&edit(&total, 3, 3), "t", &mut d).is_err());
        // A zero resistance.
        assert!(decode(&encode(&[(id("spA0"), 0, 3200)], 0), "t", &mut d).is_err());
        // More records than a version-2 header frames (12 > 11).
        let many: Vec<_> = (0..12).map(|k| (FourCc(0x7370_4100 + k), 3000, 2500)).collect();
        assert!(decode(&encode(&many, 0), "t", &mut d).is_err());
        let eleven: Vec<_> = (0..11).map(|k| (FourCc(0x7370_4100 + k), 3000, 2500)).collect();
        assert_eq!(decode(&encode(&eleven, 0), "t", &mut d).unwrap().records.len(), 11);
    }

    #[test]
    fn ids_are_not_validated() {
        // Any u32 is accepted as an ID; a repeat is reported, not refused.
        let mut d = Diagnostics::default();
        let f = encode(&[(FourCc(7), 3000, 2500), (FourCc(7), 3001, 2500)], 0);
        assert_eq!(decode(&f, "t", &mut d).unwrap().records.len(), 2);
    }
}
