//! The persisted thermal state (`/run/speakerguardd/<uid>/thermal`), the
//! boot id it is bound to, and the trust check on the engine's `saved`
//! record (written by the PipeWire process): `Persisted::decode` is the
//! exact inverse of `encode`; a record `refusal` admits, or `trusted`
//! keeps, fits the blob's unit and restores into it.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_core::blob::{self, Header};
use sgv2_core::UnitKind;
use sgv2_supervisor::thermal::{parse_boot_id, trusted, Persisted, FILE_LEN, MAX_AGE_NS};

fn header(unit: u32, channels: u32, rate: u32, crc: u32) -> Header {
    let b = blob::build(UnitKind::Spp3, 1, 2, 48_000, &[0u8; 4]);
    let (h, _) = Header::parse(&b).unwrap();
    Header { unit, channels, sample_rate: rate, payload_crc: crc, ..h }
}

fuzz_target!(|data: &[u8]| {
    let Some((&mode, rest)) = data.split_first() else { return };
    // the boot id parser on any text
    if let Some(id) = parse_boot_id(&String::from_utf8_lossy(rest)) {
        let hex: String = id.iter().map(|b| format!("{b:02x}")).collect();
        let text = String::from_utf8_lossy(rest).trim().replace('-', "").to_ascii_lowercase();
        assert_eq!(hex, text, "the boot id is its 32 hex digits");
    }
    // the file: as given, or padded to its length and with its CRC fixed
    let mut file = rest.to_vec();
    if mode & 1 != 0 {
        file.resize(FILE_LEN, 0);
        file[..8].copy_from_slice(b"SGV2THM1");
        let crc = blob::crc32(&file[..FILE_LEN - 4]);
        file[FILE_LEN - 4..].copy_from_slice(&crc.to_le_bytes());
    }
    let Some(p) = Persisted::decode(&file) else { return };
    assert_eq!(p.encode(), file, "decode is not the inverse of encode");
    let kinds = [UnitKind::Spp3 as u32, UnitKind::Lsm2 as u32, UnitKind::Atsp as u32, p.record.kind];
    let kind = kinds[(mode >> 1) as usize % kinds.len()];
    let channels = if mode & 0x10 != 0 { p.record.channels } else { 1 + (mode as u32 >> 5) };
    let h = header(kind, channels, p.record.sample_rate, p.params_crc);
    let now = p.stamp_boottime_ns.saturating_add(u64::from(mode) * MAX_AGE_NS / 128);
    if p.refusal(&p.boot_id, &p.board, &h, now).is_none() {
        assert!(trusted(&p.record, &h, p.params_crc, 0), "an admitted record that is not trusted");
    }
    if trusted(&p.record, &h, p.params_crc, 0) {
        let n = h.channels as usize;
        for g in &p.record.gain[..n] {
            assert!(g.is_finite() && *g > 0.0 && *g <= 1.0, "trusted gain {g}");
        }
        for v in p.record.node[..n].iter().flatten() {
            assert!(v.is_finite() && *v >= 0.0, "trusted node {v}");
        }
    }
});
