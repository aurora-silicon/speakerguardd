//! The compiled parameter blob the supervisor publishes and the plugin
//! loads in `instantiate()`. The plugin never parses Apple's files itself.
//!
//! Layout (little-endian): a fixed header, then a unit-specific payload.
//! The payload layouts are TODO(SPEC): they are the parameter sets the
//! behaviour specification names, in the order it fixes.

pub const MAGIC: [u8; 8] = *b"SGV2PRM\0";
pub const VERSION: u32 = 1;
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
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlobError {
    Short,
    Magic,
    Version,
    Length,
    Crc,
    Channels,
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
        };
        if header.version != VERSION {
            return Err(BlobError::Version);
        }
        if header.channels == 0 || header.channels as usize > crate::MAX_CHANNELS {
            return Err(BlobError::Channels);
        }
        let payload = &bytes[HEADER_LEN..];
        if payload.len() != header.payload_len as usize {
            return Err(BlobError::Length);
        }
        if crc32(payload) != header.payload_crc {
            return Err(BlobError::Crc);
        }
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
    }
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

/// Build a complete blob (header and payload) for `unit`.
#[cfg(any(test, feature = "std"))]
pub fn build(
    unit: crate::UnitKind,
    channels: u32,
    sense_channels: u32,
    sample_rate: u32,
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
    };
    let mut head = [0u8; HEADER_LEN];
    h.write(&mut head);
    let mut v = head.to_vec();
    v.extend_from_slice(payload);
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
        };
        let mut head = [0u8; HEADER_LEN];
        h.write(&mut head);
        let mut blob = head.to_vec();
        blob.extend_from_slice(&payload);
        assert_eq!(Header::parse(&blob).unwrap(), (h, &payload[..]));
        let mut bad = blob.clone();
        *bad.last_mut().unwrap() ^= 1;
        assert_eq!(Header::parse(&bad), Err(BlobError::Crc));
        assert_eq!(Header::parse(&blob[..10]), Err(BlobError::Short));
        let mut magic = blob.clone();
        magic[0] = b'X';
        assert_eq!(Header::parse(&magic), Err(BlobError::Magic));
    }
}
