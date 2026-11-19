//! The speakers' thermal state across engine and PipeWire restarts.
//!
//! The engine's hand-off record lives in its process (`sgv2_lv2::handoff`),
//! so a PipeWire restart mid-playback used to start the models cold while
//! the coils were still hot (J616s: 140-150 degC modelled, restarted at the
//! 50 degC start state). macOS cold-starts too after a coreaudiod restart;
//! this is deliberately more protective.
//!
//! - The plugin publishes every trusted record into the page's `saved`
//!   slot, stamped with CLOCK_BOOTTIME (which counts a suspend).
//! - The supervisor's thread takes a record only if it fits the published
//!   blob (unit kind, channels, rate), the page runs that blob, and the
//!   engine reports no fault ([`UNTRUSTED`]). It writes the record into the
//!   page's `provided` slot with the blob's CRC, and to
//!   `/run/speakerguardd/<uid>/thermal` ([`Persisted`]), bound to the boot,
//!   the board, the blob CRC and the geometry.
//! - At start the supervisor reloads that file into `provided` if it
//!   matches all of those and is at most [`MAX_AGE_NS`] old.
//! - A new engine process (its own slot empty) restores `provided` with
//!   the unit's own `restore_state`, which ages every node by the elapsed
//!   time with the model's decay. It is never colder than the aged value,
//!   because the speakers cannot cool faster than the model while nothing
//!   plays.
//! - A reboot (another boot id, and /run is empty anyway), more than ten
//!   minutes, or any mismatch falls back to the parameters' start state.

use sgv2_core::blob::Header;
use sgv2_core::{HandOff, Health, HANDOFF_NODES, MAX_CHANNELS};

/// Older than this, the speakers have cooled: the start state applies.
pub const MAX_AGE_NS: u64 = 600_000_000_000;

/// Engine health bits under which a record is not trusted. MODEL_RANGE is
/// not one: the engine still protects then (B5), and dropping those
/// records would persist an older, cooler state instead.
pub const UNTRUSTED: u32 = Health::FAILSAFE | Health::NO_PARAMS | Health::CHANNEL_MISMATCH | Health::MODEL_FAULT;

const MAGIC: [u8; 8] = *b"SGV2THM1";
const WORDS: usize = MAX_CHANNELS * HANDOFF_NODES + MAX_CHANNELS;
/// magic, boot id (16), board (8), crc, kind, channels, rate, stamp, words, crc32
pub const FILE_LEN: usize = 8 + 16 + 8 + 4 * 4 + 8 + WORDS * 8 + 4;

/// The persisted record and what it is bound to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Persisted {
    pub boot_id: [u8; 16],
    pub board: [u8; 8],
    pub params_crc: u32,
    pub stamp_boottime_ns: u64,
    pub record: HandOff,
}

/// `/proc/sys/kernel/random/boot_id` ("xxxxxxxx-xxxx-...") as 16 bytes.
pub fn parse_boot_id(text: &str) -> Option<[u8; 16]> {
    let hex: String = text.trim().chars().filter(|c| *c != '-').collect();
    if hex.len() != 32 {
        return None;
    }
    let mut out = [0u8; 16];
    for (i, b) in out.iter_mut().enumerate() {
        *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(out)
}

/// Whether the engine's `saved` record may be kept: it fits the blob, the
/// page runs that blob, and the engine reports no fault.
pub fn trusted(rec: &HandOff, blob: &Header, page_crc: u32, page_health: u32) -> bool {
    let Some(kind) = unit_kind(blob.unit) else { return false };
    page_crc == blob.payload_crc
        && blob.payload_crc != 0
        && page_health & UNTRUSTED == 0
        && rec.fits(kind, blob.channels as usize, blob.sample_rate)
}

fn unit_kind(fourcc: u32) -> Option<sgv2_core::UnitKind> {
    use sgv2_core::UnitKind::*;
    [Spp3, Lsm2, Atsp].into_iter().find(|k| *k as u32 == fourcc)
}

impl Persisted {
    pub fn encode(&self) -> Vec<u8> {
        let mut v = Vec::with_capacity(FILE_LEN);
        v.extend_from_slice(&MAGIC);
        v.extend_from_slice(&self.boot_id);
        v.extend_from_slice(&self.board);
        for w in [self.params_crc, self.record.kind, self.record.channels, self.record.sample_rate] {
            v.extend_from_slice(&w.to_le_bytes());
        }
        v.extend_from_slice(&self.stamp_boottime_ns.to_le_bytes());
        for x in self.record.node.iter().flatten().chain(&self.record.gain) {
            v.extend_from_slice(&x.to_bits().to_le_bytes());
        }
        let crc = sgv2_core::blob::crc32(&v);
        v.extend_from_slice(&crc.to_le_bytes());
        v
    }

    pub fn decode(b: &[u8]) -> Option<Persisted> {
        if b.len() != FILE_LEN || b[..8] != MAGIC {
            return None;
        }
        let (body, tail) = b.split_at(FILE_LEN - 4);
        if sgv2_core::blob::crc32(body) != u32::from_le_bytes(tail.try_into().ok()?) {
            return None;
        }
        let u32_at = |o: usize| u32::from_le_bytes(b[o..o + 4].try_into().unwrap_or_default());
        let u64_at = |o: usize| u64::from_le_bytes(b[o..o + 8].try_into().unwrap_or_default());
        let mut record = HandOff::EMPTY;
        record.kind = u32_at(36);
        record.channels = u32_at(40);
        record.sample_rate = u32_at(44);
        let mut o = 56;
        for x in record.node.iter_mut().flatten().chain(record.gain.iter_mut()) {
            *x = f64::from_bits(u64_at(o));
            o += 8;
        }
        Some(Persisted {
            boot_id: b[8..24].try_into().ok()?,
            board: b[24..32].try_into().ok()?,
            params_crc: u32_at(32),
            stamp_boottime_ns: u64_at(48),
            record,
        })
    }

    /// Why this file must not be restored now, or `None` if it may.
    pub fn refusal(&self, boot_id: &[u8; 16], board: &[u8; 8], blob: &Header, now_boottime_ns: u64) -> Option<String> {
        if &self.boot_id != boot_id {
            return Some("from another boot".into());
        }
        if &self.board != board {
            return Some("from another board".into());
        }
        if self.params_crc != blob.payload_crc {
            return Some(format!("for blob {:#010x}, not {:#010x}", self.params_crc, blob.payload_crc));
        }
        if !trusted(&self.record, blob, self.params_crc, 0) {
            return Some("does not fit the blob's unit".into());
        }
        if self.stamp_boottime_ns > now_boottime_ns {
            return Some("dated in the future".into());
        }
        let age = now_boottime_ns - self.stamp_boottime_ns;
        if age > MAX_AGE_NS {
            return Some(format!("{} s old", age / 1_000_000_000));
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sgv2_core::blob::{build_for, machine_field};
    use sgv2_core::UnitKind;

    fn blob() -> Header {
        let b = build_for(UnitKind::Spp3, 6, 12, 48_000, machine_field("j616s").unwrap(), 11_890, &[1u8, 2, 3, 4]);
        Header::parse(&b).unwrap().0
    }

    fn hot(h: &Header) -> HandOff {
        let mut r = HandOff::EMPTY;
        r.kind = UnitKind::Spp3 as u32;
        r.channels = h.channels;
        r.sample_rate = h.sample_rate;
        for c in 0..6 {
            r.node[c][0] = 95.0; // coil rise over ambient, K
            r.node[c][1] = 30.0;
            r.gain[c] = 0.5;
        }
        r
    }

    const BOOT: [u8; 16] = [7; 16];
    const BOARD: [u8; 8] = *b"j616s\0\0\0";

    #[test]
    fn a_record_round_trips_and_any_corruption_is_refused() {
        let h = blob();
        let p =
            Persisted { boot_id: BOOT, board: BOARD, params_crc: h.payload_crc, stamp_boottime_ns: 5, record: hot(&h) };
        let bytes = p.encode();
        assert_eq!(bytes.len(), FILE_LEN);
        assert_eq!(Persisted::decode(&bytes), Some(p));
        for i in [0, 9, 30, 40, 60, FILE_LEN - 1] {
            let mut bad = bytes.clone();
            bad[i] ^= 1;
            assert_eq!(Persisted::decode(&bad), None, "byte {i}");
        }
        assert_eq!(Persisted::decode(&bytes[..FILE_LEN - 1]), None);
    }

    #[test]
    fn only_this_boot_board_blob_and_geometry_within_ten_minutes_restores() {
        let h = blob();
        let now = 1_000_000_000_000;
        let p = Persisted {
            boot_id: BOOT,
            board: BOARD,
            params_crc: h.payload_crc,
            stamp_boottime_ns: now - 30_000_000_000,
            record: hot(&h),
        };
        assert_eq!(p.refusal(&BOOT, &BOARD, &h, now), None);
        assert!(p.refusal(&[8; 16], &BOARD, &h, now).unwrap().contains("boot"));
        assert!(p.refusal(&BOOT, b"j316s\0\0\0", &h, now).unwrap().contains("board"));
        let other = Header { payload_crc: h.payload_crc ^ 1, ..h };
        assert!(p.refusal(&BOOT, &BOARD, &other, now).unwrap().contains("blob"));
        let four = Header { channels: 4, ..h };
        assert!(p.refusal(&BOOT, &BOARD, &four, now).is_some());
        let at_limit = Persisted { stamp_boottime_ns: now - MAX_AGE_NS, ..p };
        assert_eq!(at_limit.refusal(&BOOT, &BOARD, &h, now), None);
        let old = Persisted { stamp_boottime_ns: now - MAX_AGE_NS - 1, ..p };
        assert!(old.refusal(&BOOT, &BOARD, &h, now).unwrap().contains("s old"));
        let future = Persisted { stamp_boottime_ns: now + 1, ..p };
        assert!(future.refusal(&BOOT, &BOARD, &h, now).is_some());
    }

    #[test]
    fn a_faulted_or_foreign_record_is_never_kept() {
        let h = blob();
        let r = hot(&h);
        assert!(trusted(&r, &h, h.payload_crc, 0));
        assert!(trusted(&r, &h, h.payload_crc, Health::OPEN_LOOP));
        assert!(trusted(&r, &h, h.payload_crc, Health::MODEL_RANGE));
        for bad in [Health::FAILSAFE, Health::NO_PARAMS, Health::CHANNEL_MISMATCH, Health::MODEL_FAULT] {
            assert!(!trusted(&r, &h, h.payload_crc, bad), "{bad:#x}");
        }
        // the page runs another blob
        assert!(!trusted(&r, &h, h.payload_crc ^ 1, 0));
        // a non-finite or negative node, or a gain out of (0, 1]
        let mut nan = r;
        nan.node[2][0] = f64::NAN;
        assert!(!trusted(&nan, &h, h.payload_crc, 0));
        let mut neg = r;
        neg.node[0][1] = -1.0;
        assert!(!trusted(&neg, &h, h.payload_crc, 0));
        let mut g = r;
        g.gain[1] = 1.5;
        assert!(!trusted(&g, &h, h.payload_crc, 0));
        // another unit or rate
        let mut k = r;
        k.kind = UnitKind::Lsm2 as u32;
        assert!(!trusted(&k, &h, h.payload_crc, 0));
        let mut rate = r;
        rate.sample_rate = 44_100;
        assert!(!trusted(&rate, &h, h.payload_crc, 0));
    }

    #[test]
    fn boot_ids_parse() {
        let b = parse_boot_id("0f1e2d3c-4b5a-6978-8796-a5b4c3d2e1f0\n").unwrap();
        assert_eq!(b[0], 0x0f);
        assert_eq!(b[15], 0xf0);
        assert_eq!(parse_boot_id("nope"), None);
    }
}
