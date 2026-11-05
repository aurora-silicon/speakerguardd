//! The compiled parameter blob the supervisor publishes and the plugin
//! loads in `instantiate()`. The plugin never parses Apple's files itself.
//!
//! Layout (little-endian): a fixed header, then a unit-specific payload.
//! The payload layouts are TODO(SPEC): they are the parameter sets the
//! behaviour specification names, in the order it fixes.
//!
//! ```text
//! 0   magic "SGV2PRM\0"     24  sample_rate
//! 8   version               28  payload_len
//! 12  unit (fourcc)         32  payload_crc
//! 16  channels              36  machine (8 bytes, ASCII, NUL-padded)
//! 20  sense_channels        44  full_scale_mv
//! ```
//!
//! `machine` and `full_scale_mv` were reserved zero bytes before; a blob
//! that leaves them zero names no machine, and the supervisor refuses it.
//!
//! Version 2 adds the pre-stage ([`crate::prestage`]): `payload_len` bytes
//! of unit payload, then the pre-stage section, which runs to the end of the
//! blob and carries its own length and CRC. A version 1 blob has no
//! pre-stage and is refused.

pub const MAGIC: [u8; 8] = *b"SGV2PRM\0";
pub const VERSION: u32 = 2;
pub const HEADER_LEN: usize = 48;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub version: u32,
    /// `UnitKind` as its four-character code.
    pub unit: u32,
    pub channels: u32,
    pub sense_channels: u32,
    pub sample_rate: u32,
    pub payload_len: u32,
    /// CRC-32 (IEEE) of the payload.
    pub payload_crc: u32,
    /// The Mac whose tuning this is: its board, the device tree's first
    /// root compatible without the vendor ("j316s" for "apple,j316s"),
    /// ASCII and NUL-padded. All zero: no machine named.
    pub machine: [u8; MACHINE_LEN],
    /// The amps' operating point the parameters assume: the output peak
    /// voltage at digital full scale, in millivolts (lsm2 `VoltsFullScale`;
    /// spp3's output scaling is TODO(SPEC)). 0: not stated.
    pub full_scale_mv: u32,
}

/// Bytes of [`Header::machine`].
pub const MACHINE_LEN: usize = 8;

/// `name` as a header machine field, or `None` if it does not fit or is not
/// plain lowercase ASCII.
pub fn machine_field(name: &str) -> Option<[u8; MACHINE_LEN]> {
    if name.is_empty()
        || name.len() > MACHINE_LEN
        || !name.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
    {
        return None;
    }
    let mut m = [0u8; MACHINE_LEN];
    m[..name.len()].copy_from_slice(name.as_bytes());
    Some(m)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlobError {
    Short,
    Magic,
    Version,
    Length,
    Crc,
    Channels,
    /// A pre-stage section that is unstable, non-finite or gains.
    Unstable,
}

impl Header {
    pub fn parse(bytes: &[u8]) -> Result<(Header, &[u8]), BlobError> {
        if bytes.len() < HEADER_LEN {
            return Err(BlobError::Short);
        }
        if bytes[..8] != MAGIC {
            return Err(BlobError::Magic);
        }
        let word = |i: usize| u32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]);
        let header = Header {
            version: word(8),
            unit: word(12),
            channels: word(16),
            sense_channels: word(20),
            sample_rate: word(24),
            payload_len: word(28),
            payload_crc: word(32),
            machine: bytes[36..44].try_into().unwrap_or([0; MACHINE_LEN]),
            full_scale_mv: word(44),
        };
        if header.version != VERSION {
            return Err(BlobError::Version);
        }
        if header.channels == 0 || header.channels as usize > crate::MAX_CHANNELS {
            return Err(BlobError::Channels);
        }
        let rest = &bytes[HEADER_LEN..];
        let n = header.payload_len as usize;
        if rest.len() < n {
            return Err(BlobError::Length);
        }
        let (payload, pre) = rest.split_at(n);
        if crc32(payload) != header.payload_crc {
            return Err(BlobError::Crc);
        }
        check_prestage(pre)?;
        Ok((header, payload))
    }

    pub fn write(&self, out: &mut [u8; HEADER_LEN]) {
        out.fill(0);
        out[..8].copy_from_slice(&MAGIC);
        for (i, v) in [
            self.version,
            self.unit,
            self.channels,
            self.sense_channels,
            self.sample_rate,
            self.payload_len,
            self.payload_crc,
        ]
        .iter()
        .enumerate()
        {
            out[8 + 4 * i..12 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        out[36..44].copy_from_slice(&self.machine);
        out[44..48].copy_from_slice(&self.full_scale_mv.to_le_bytes());
    }

    /// The machine field as text (`None` when no machine is named or it is
    /// not ASCII).
    pub fn machine(&self) -> Option<&str> {
        let end = self.machine.iter().position(|&b| b == 0).unwrap_or(MACHINE_LEN);
        if end == 0 || self.machine[end..].iter().any(|&b| b != 0) {
            return None;
        }
        core::str::from_utf8(&self.machine[..end]).ok()
    }
}

/// The structure of a pre-stage section (magic, length, CRC); its filters
/// are checked by [`crate::prestage::Params::decode`].
fn check_prestage(pre: &[u8]) -> Result<(), BlobError> {
    use crate::prestage::{HEAD_LEN, MAGIC};
    if pre.len() < HEAD_LEN {
        return Err(BlobError::Short);
    }
    if pre[..8] != MAGIC {
        return Err(BlobError::Magic);
    }
    let word = |i: usize| u32::from_le_bytes([pre[i], pre[i + 1], pre[i + 2], pre[i + 3]]);
    if pre.len() - HEAD_LEN != word(16) as usize {
        return Err(BlobError::Length);
    }
    if crc32(&pre[HEAD_LEN..]) != word(20) {
        return Err(BlobError::Crc);
    }
    Ok(())
}

/// The pre-stage section of a blob, decoded and checked for `fs`.
pub fn prestage(bytes: &[u8], fs: f64) -> Result<crate::prestage::Params, BlobError> {
    let (h, payload) = Header::parse(bytes)?;
    let at = HEADER_LEN + payload.len();
    crate::prestage::Params::decode(&bytes[at..], h.channels as usize, fs)
}

/// `blob` with its pre-stage replaced by `pre` (the header is unchanged).
#[cfg(any(test, feature = "std"))]
pub fn with_prestage(blob: &[u8], pre: &crate::prestage::Params) -> Result<std::vec::Vec<u8>, BlobError> {
    let (_, payload) = Header::parse(blob)?;
    let mut v = blob[..HEADER_LEN + payload.len()].to_vec();
    v.extend_from_slice(&pre.encode());
    Ok(v)
}

/// Plain-old-data payloads: every field is a 4-byte `f32` or `u32` (or an
/// array of them), little-endian on every supported target, so the
/// `#[repr(C)]` struct is its own serialisation.
///
/// # Safety
/// Implement only for `#[repr(C)]` structs made of `f32`/`u32` fields and
/// arrays thereof, with no padding, for which every bit pattern is valid.
pub unsafe trait Pod: Copy + Sized {
    fn as_bytes(&self) -> &[u8] {
        // SAFETY: Pod contract: no padding, plain 4-byte fields.
        unsafe { core::slice::from_raw_parts(self as *const Self as *const u8, core::mem::size_of::<Self>()) }
    }
    fn from_bytes(b: &[u8]) -> Result<Self, BlobError> {
        if b.len() != core::mem::size_of::<Self>() {
            return Err(BlobError::Length);
        }
        // SAFETY: Pod contract: every bit pattern is valid; unaligned read.
        Ok(unsafe { core::ptr::read_unaligned(b.as_ptr() as *const Self) })
    }
}

/// Build a complete blob (header and payload) for `unit`, naming no machine.
/// The supervisor refuses such a blob; see [`build_for`].
#[cfg(any(test, feature = "std"))]
pub fn build(
    unit: crate::UnitKind,
    channels: u32,
    sense_channels: u32,
    sample_rate: u32,
    payload: &[u8],
) -> std::vec::Vec<u8> {
    build_for(unit, channels, sense_channels, sample_rate, [0; MACHINE_LEN], 0, payload)
}

/// Build a complete blob for `unit` on one machine and operating point.
#[cfg(any(test, feature = "std"))]
pub fn build_for(
    unit: crate::UnitKind,
    channels: u32,
    sense_channels: u32,
    sample_rate: u32,
    machine: [u8; MACHINE_LEN],
    full_scale_mv: u32,
    payload: &[u8],
) -> std::vec::Vec<u8> {
    let h = Header {
        version: VERSION,
        unit: unit as u32,
        channels,
        sense_channels,
        sample_rate,
        payload_len: payload.len() as u32,
        payload_crc: crc32(payload),
        machine,
        full_scale_mv,
    };
    let mut head = [0u8; HEADER_LEN];
    h.write(&mut head);
    let mut v = head.to_vec();
    v.extend_from_slice(payload);
    // no pre-stage derived yet: every channel passes, nothing claimed
    v.extend_from_slice(&crate::prestage::Params::identity(channels as usize).encode());
    v
}

/// CRC-32 (IEEE 802.3, reflected, init and xorout 0xFFFFFFFF).
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn header_roundtrip_and_rejections() {
        let payload = [1u8, 2, 3, 4];
        let h = Header {
            version: VERSION,
            unit: u32::from_be_bytes(*b"lsm2"),
            channels: 2,
            sense_channels: 0,
            sample_rate: 48_000,
            payload_len: 4,
            payload_crc: crc32(&payload),
            machine: machine_field("j700").unwrap(),
            full_scale_mv: 7300,
        };
        let mut head = [0u8; HEADER_LEN];
        h.write(&mut head);
        let mut blob = head.to_vec();
        blob.extend_from_slice(&payload);
        // a version 2 blob without its pre-stage is refused
        assert_eq!(Header::parse(&blob), Err(BlobError::Short));
        blob.extend_from_slice(&crate::prestage::Params::identity(2).encode());
        assert_eq!(Header::parse(&blob).unwrap(), (h, &payload[..]));
        assert_eq!(Header::parse(&blob).unwrap().0.machine(), Some("j700"));
        let mut bad = blob.clone();
        bad[HEADER_LEN] ^= 1;
        assert_eq!(Header::parse(&bad), Err(BlobError::Crc));
        let mut bad = blob.clone();
        *bad.last_mut().unwrap() ^= 1;
        assert_eq!(Header::parse(&bad), Err(BlobError::Crc));
        let mut old = blob.clone();
        old[8] = 1;
        assert_eq!(Header::parse(&old), Err(BlobError::Version));
        assert_eq!(Header::parse(&blob[..10]), Err(BlobError::Short));
        let mut magic = blob.clone();
        magic[0] = b'X';
        assert_eq!(Header::parse(&magic), Err(BlobError::Magic));
    }

    #[test]
    fn machine_field_and_unnamed_blobs() {
        assert_eq!(machine_field("j316s").map(|m| m[..5] == *b"j316s"), Some(true));
        assert_eq!(machine_field(""), None);
        assert_eq!(machine_field("j316s-too-long"), None);
        assert_eq!(machine_field("J316S"), None);
        let b = build(crate::UnitKind::Lsm2, 2, 0, 48_000, &[0u8; 4]);
        let (h, _) = Header::parse(&b).unwrap();
        assert_eq!((h.machine(), h.full_scale_mv), (None, 0));
        let b = build_for(crate::UnitKind::Lsm2, 2, 0, 48_000, machine_field("j616s").unwrap(), 11_899, &[0u8; 4]);
        let (h, _) = Header::parse(&b).unwrap();
        assert_eq!((h.machine(), h.full_scale_mv), (Some("j616s"), 11_899));
    }
}
