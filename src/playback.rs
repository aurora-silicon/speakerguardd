// SPDX-License-Identifier: MIT
//! Versioned playback grant control. All units/counters refer to immutable
//! submitted payloads, never to a prediction of a recent sense period.

use std::io;

pub const BYTES: usize = 512;
pub const SPEAKERS: usize = 6;
pub const UNIT_RATE: f64 = 48000.0;
pub const UNIT_SCALE: f64 = (1u64 << 30) as f64;
pub const UNITY: u32 = 1 << 31;
pub const AVAILABLE: u32 = 1;
pub const CONFIGURED: u32 = 2;
pub const TRANSFER_VALID: u32 = 4;
pub const FAILED: u32 = 8;
pub const OWNED: u32 = 16;
pub const LIVE: u32 = 32;
pub const PLAYING: u32 = 64;
pub const REQUIRED: u32 = 128;
const REVOKE: u32 = 1 << 31;

fn invalid() -> io::Error {
    io::Error::from_raw_os_error(libc::EPROTO)
}
fn get32(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(b[at..at + 4].try_into().unwrap())
}
fn get64(b: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(b[at..at + 8].try_into().unwrap())
}
fn put32(b: &mut [u8], at: usize, x: u32) {
    b[at..at + 4].copy_from_slice(&x.to_le_bytes());
}
fn put64(b: &mut [u8], at: usize, x: u64) {
    b[at..at + 8].copy_from_slice(&x.to_le_bytes());
}

#[derive(Debug, Clone)]
pub struct Snapshot {
    wire: [u8; BYTES],
    order: [usize; SPEAKERS],
    pub flags: u32,
    pub epoch: u64,
    pub sequence: u64,
    pub clock_ns: u64,
    pub deadline_ns: u64,
    pub rate: u32,
    pub channels: usize,
    pub speakers: usize,
    pub format: u32,
    pub transfer: u32,
    pub amp_gain_millidbv: u32,
    pub safe_reduction_millidb: u32,
    pub pending: usize,
    pub slots: [u32; SPEAKERS],
    pub submitted: [u64; SPEAKERS],
    pub completed: [u64; SPEAKERS],
    pub source_submitted: [u64; SPEAKERS],
    pub source_completed: [u64; SPEAKERS],
    pub refunded: [u64; SPEAKERS],
    pub unbudgeted: [u64; SPEAKERS],
    pub ceiling: [u64; SPEAKERS],
    pub gains: [u32; SPEAKERS],
    pub card: String,
}

impl Snapshot {
    pub fn decode(wire: [u8; BYTES]) -> io::Result<Self> {
        if get32(&wire, 0) != 1 || wire[488..].iter().any(|&b| b != 0) {
            return Err(invalid());
        }
        let end = wire[456..488]
            .iter()
            .position(|&b| b == 0)
            .ok_or_else(invalid)?;
        let card = std::str::from_utf8(&wire[456..456 + end])
            .map_err(|_| invalid())?
            .to_owned();
        if card.is_empty() || !card.bytes().all(|b| b.is_ascii_alphanumeric()) {
            return Err(invalid());
        }
        let a64 = |at| std::array::from_fn(|k| get64(&wire, at + k * 8));
        let a32 = |at| std::array::from_fn(|k| get32(&wire, at + k * 4));
        let out = Self {
            flags: get32(&wire, 4),
            epoch: get64(&wire, 8),
            sequence: get64(&wire, 16),
            clock_ns: get64(&wire, 24),
            deadline_ns: get64(&wire, 32),
            rate: get32(&wire, 40),
            channels: get32(&wire, 44) as usize,
            speakers: get32(&wire, 48) as usize,
            format: get32(&wire, 52),
            transfer: get32(&wire, 56),
            amp_gain_millidbv: get32(&wire, 60),
            safe_reduction_millidb: get32(&wire, 64),
            pending: get32(&wire, 68) as usize,
            slots: a32(72),
            submitted: a64(96),
            completed: a64(144),
            source_submitted: a64(192),
            source_completed: a64(240),
            refunded: a64(288),
            unbudgeted: a64(336),
            ceiling: a64(384),
            gains: a32(432),
            card,
            wire,
            order: std::array::from_fn(|k| k),
        };
        if out.flags & !255 != 0
            || out.speakers > SPEAKERS
            || out.channels > SPEAKERS
            || out.pending > 8
            || out.rate > 192000
            || out.amp_gain_millidbv > 100000
            || out.safe_reduction_millidb > 200000
        {
            return Err(invalid());
        }
        if out.flags & CONFIGURED != 0
            && (out.rate < 8000
                || out.speakers == 0
                || out.channels == 0
                || ![2, 6, 10].contains(&out.format))
        {
            return Err(invalid());
        }
        if out.flags & LIVE != 0 && out.deadline_ns <= out.clock_ns {
            return Err(invalid());
        }
        for k in 0..SPEAKERS {
            if out.slots[k] != k as u32
                || out.completed[k] > out.submitted[k]
                || out.source_completed[k] > out.source_submitted[k]
                || out.submitted[k] > out.ceiling[k]
                || out.refunded[k] > out.completed[k]
                || out.gains[k] > UNITY
            {
                return Err(invalid());
            }
        }
        Ok(out)
    }

    /// Present physical counters in packaged model order. The wire receipt
    /// stays unchanged; writes map model ceilings back to physical slots.
    pub fn for_profile(mut self, cfg: &crate::config::Config) -> io::Result<Self> {
        let n = cfg.speakers.len();
        if cfg.playback_slots.len() != n || n > SPEAKERS {
            return Err(invalid());
        }
        let order: [usize; SPEAKERS] =
            std::array::from_fn(|k| cfg.playback_slots.get(k).copied().unwrap_or(k));
        if order.iter().any(|k| *k >= SPEAKERS) || (0..n).any(|k| !order[..n].contains(&k)) {
            return Err(invalid());
        }
        if order == self.order {
            return Ok(self);
        }
        if self.order != std::array::from_fn(|k| k) {
            return Err(invalid());
        }
        macro_rules! reorder {($($field:ident),*)=>{$(self.$field=std::array::from_fn(|k|self.$field[order[k]]);)*};}
        reorder!(
            slots,
            submitted,
            completed,
            source_submitted,
            source_completed,
            refunded,
            unbudgeted,
            ceiling,
            gains
        );
        self.order = order;
        Ok(self)
    }

    /// Only a kernel certificate for the complete downstream path qualifies.
    /// Unknown versions/profiles keep the existing conservative policy.
    pub fn qualified(&self, cfg: &crate::config::Config) -> bool {
        let required = AVAILABLE | CONFIGURED | TRANSFER_VALID | OWNED | REQUIRED;
        self.flags & required == required
            && self.flags & FAILED == 0
            && [1, 2].contains(&self.transfer)
            && cfg.playback_cards.iter().any(|c| c == &self.card)
            && cfg.playback_slots.len() == cfg.speakers.len()
            && self.speakers == cfg.speakers.len()
            && self.channels == self.speakers
            && cfg.globals.amp_gain_dbv * 1000.0 + 1e-6 >= f64::from(self.amp_gain_millidbv)
            && f64::from(self.safe_reduction_millidb) / 1000.0 >= cfg.globals.safe_reduction
    }

    pub fn grant(&self, ceiling: &[u64], gains: &[u32]) -> io::Result<[u8; BYTES]> {
        if ceiling.len() != self.speakers || gains.len() != self.speakers {
            return Err(invalid());
        }
        let mut wire = self.wire;
        put32(&mut wire, 4, 0);
        let deadline = self.clock_ns.checked_add(250_000_000).ok_or_else(invalid)?;
        put64(&mut wire, 32, deadline);
        for k in 0..SPEAKERS {
            let cap = ceiling.get(k).copied().unwrap_or(0);
            let gain = gains.get(k).copied().unwrap_or(0);
            if gain > UNITY || (k < self.speakers && cap < self.submitted[k]) {
                return Err(invalid());
            }
            put64(&mut wire, 384 + self.order[k] * 8, cap);
            put32(&mut wire, 432 + self.order[k] * 4, gain);
        }
        Ok(wire)
    }

    pub fn wire(&self) -> [u8; BYTES] {
        self.wire
    }

    pub fn revoke(&self) -> [u8; BYTES] {
        let mut wire = self.wire;
        put32(&mut wire, 4, REVOKE);
        wire
    }
}

pub fn joules(units: u64, watts: f64) -> f64 {
    // Converting a delta may round slightly up or down. Round the physical
    // charge upward; do not convert/subtract lifetime totals in floating point.
    (units as f64 / UNIT_SCALE / UNIT_RATE * watts) * (1.0 + 1e-12)
        + if units > 0 { 1e-15 } else { 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn receipt() -> Snapshot {
        let mut wire = [0u8; BYTES];
        put32(&mut wire, 0, 1);
        put32(
            &mut wire,
            4,
            AVAILABLE | CONFIGURED | TRANSFER_VALID | OWNED | REQUIRED,
        );
        put64(&mut wire, 8, 1);
        put64(&mut wire, 16, 1);
        put64(&mut wire, 24, 1);
        put32(&mut wire, 40, 48000);
        put32(&mut wire, 44, 6);
        put32(&mut wire, 48, 6);
        put32(&mut wire, 52, 6);
        put32(&mut wire, 56, 2);
        put32(&mut wire, 60, 18500);
        put32(&mut wire, 64, 100000);
        wire[456..465].copy_from_slice(b"AppleJ616");
        for k in 0..SPEAKERS {
            put32(&mut wire, 72 + k * 4, k as u32);
            put64(&mut wire, 96 + k * 8, 100 + k as u64);
            put64(&mut wire, 384 + k * 8, 1000);
            put32(&mut wire, 432 + k * 4, UNITY);
        }
        Snapshot::decode(wire).unwrap()
    }
    #[test]
    fn j616_maps_model_speakers_to_the_dt_playback_slots() {
        let cfg = crate::config::Config::parse(include_str!("../conf/apple/j616.conf")).unwrap();
        let raw = receipt();
        assert!(raw.qualified(&cfg));
        let mapped = raw.for_profile(&cfg).unwrap();
        assert_eq!(&mapped.submitted[..6], &[100, 101, 104, 105, 102, 103]);
        // Reusing a mapped receipt for the saved-state close is idempotent.
        assert_eq!(
            mapped.clone().for_profile(&cfg).unwrap().submitted,
            mapped.submitted
        );
        let packet = mapped
            .grant(&[200, 201, 202, 203, 204, 205], &[UNITY; 6])
            .unwrap();
        assert_eq!(
            (0..6)
                .map(|k| get64(&packet, 384 + 8 * k))
                .collect::<Vec<_>>(),
            vec![200, 201, 204, 205, 202, 203]
        );
        assert_eq!(
            (0..6)
                .map(|k| get32(&packet, 72 + 4 * k))
                .collect::<Vec<_>>(),
            vec![0, 1, 2, 3, 4, 5]
        );
    }
    #[test]
    fn an_available_control_without_qualification_never_enables_grants() {
        let cfg = crate::config::Config::parse(include_str!("../conf/apple/j616.conf")).unwrap();
        let mut s = receipt();
        s.flags &= !TRANSFER_VALID;
        assert!(!s.qualified(&cfg));
        let mut s = receipt();
        s.transfer = 0;
        assert!(!s.qualified(&cfg));
        let mut s = receipt();
        s.card = "AppleUnknown".into();
        assert!(!s.qualified(&cfg));
        let mut s = receipt();
        s.flags |= FAILED;
        assert!(!s.qualified(&cfg));
        let mut s = receipt();
        s.amp_gain_millidbv += 1;
        assert!(!s.qualified(&cfg));
    }
    #[test]
    fn corrupt_receipts_are_errors_instead_of_empty_energy() {
        let s = receipt();
        for at in [0, 4, 488] {
            let mut wire = s.wire;
            wire[at] = 0xff;
            assert!(Snapshot::decode(wire).is_err());
        }
        let mut wire = s.wire;
        put64(&mut wire, 144, 10000);
        assert!(Snapshot::decode(wire).is_err());
        let mut wire = s.wire;
        put32(&mut wire, 4, s.flags | LIVE);
        put64(&mut wire, 32, 1);
        assert!(Snapshot::decode(wire).is_err());
    }
}
