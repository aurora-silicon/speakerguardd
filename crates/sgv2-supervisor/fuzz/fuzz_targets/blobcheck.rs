//! What the supervisor checks before it grants: the board from
//! `/proc/device-tree/compatible`, every channel's full scale from the
//! parameter blob (`blob_full_scales`), the amp gain on a DB_SCALE TLV and
//! its tolerance. Nothing may panic; an accepted full scale is a real
//! operating point (finite, positive), the header's agrees with the
//! smallest channel's, and an accepted tolerance is positive and no wider
//! than the stated one.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_core::blob::{self, Pod};
use sgv2_core::{atsp, lsm2, spp3, UnitKind};
use sgv2_supervisor::kernel::{db_scale_centi, db_scale_step};
use sgv2_supervisor::machine::{
    blob_full_scales, board_from_compatible, full_scale_mv, gain_tolerance_db, same_operating_point,
    OPERATING_POINT_TOLERANCE_DB,
};

fn word(d: &[u8], i: usize) -> u32 {
    let mut w = [0u8; 4];
    for (k, b) in w.iter_mut().enumerate() {
        *b = d.get(4 * i + k).copied().unwrap_or(0);
    }
    u32::from_le_bytes(w)
}

fuzz_target!(|data: &[u8]| {
    let Some((&sel, rest)) = data.split_first() else { return };
    if let Some(b) = board_from_compatible(rest) {
        assert!(!b.is_empty() && !b.contains('\0'));
    }

    // the TLV and a control value, from the first words
    let tlv = [word(rest, 0), word(rest, 1), word(rest, 2), word(rest, 3)];
    let tlv = &tlv[..(sel as usize % 5)];
    let value = i64::from(word(rest, 4) as i32);
    let min = i64::from(word(rest, 5) as i32);
    if let Some(c) = db_scale_centi(tlv, value, min) {
        let mv = full_scale_mv(c);
        assert!(!mv.is_nan() && mv >= 0.0, "full scale {mv} mV for {c} cdB");
    }
    if let Some(step) = db_scale_step(tlv) {
        assert!(step > 0);
        let tol = gain_tolerance_db(step).unwrap();
        assert!(tol > 0.0 && tol <= OPERATING_POINT_TOLERANCE_DB, "tolerance {tol}");
    }

    // the blob: raw bytes, or a real header over raw or mutated payload
    let bytes = match sel % 4 {
        0 => rest.to_vec(),
        k => {
            let n = 1 + (sel as usize >> 2) % 8;
            let (kind, mut payload) = match k {
                1 => (UnitKind::Spp3, spp3::Params { channels: n as u32, ..Default::default() }.as_bytes().to_vec()),
                2 => (UnitKind::Lsm2, lsm2::Params { channels: n as u32, ..Default::default() }.as_bytes().to_vec()),
                _ => (UnitKind::Atsp, atsp::Params { channels: n as u32, ..Default::default() }.as_bytes().to_vec()),
            };
            // fuzzed words over the payload, at fuzzed places
            for ch in rest.chunks_exact(8) {
                let at = 4 * (u32::from_le_bytes(ch[..4].try_into().unwrap()) as usize % (payload.len() / 4));
                payload[at..at + 4].copy_from_slice(&ch[4..]);
            }
            let fs = word(rest, 6) % 50_000;
            blob::build_for(kind, n as u32, 2 * n as u32, 48_000, blob::machine_field("j000").unwrap(), fs, &payload)
        }
    };
    if let Ok(per) = blob_full_scales(&bytes) {
        let (h, _) = blob::Header::parse(&bytes).unwrap();
        assert_eq!(per.len(), h.channels as usize);
        assert!(per.iter().all(|&mv| mv > 0), "a channel with no operating point: {per:?}");
        let low = *per.iter().min().unwrap();
        assert!(same_operating_point(f64::from(h.full_scale_mv), low));
    }
});
