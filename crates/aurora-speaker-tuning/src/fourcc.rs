//! Four-character codes.

use serde::{Serialize, Serializer};
use std::fmt;

#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FourCc(pub u32);

impl FourCc {
    pub const fn from_bytes(b: [u8; 4]) -> Self {
        FourCc(u32::from_be_bytes(b))
    }

    pub const fn bytes(self) -> [u8; 4] {
        self.0.to_be_bytes()
    }

    /// Exactly four printable ASCII characters.
    pub fn from_str_exact(s: &str) -> Option<Self> {
        let b = s.as_bytes();
        if b.len() == 4 && b.iter().all(|c| (0x20..0x7F).contains(c)) {
            Some(FourCc::from_bytes([b[0], b[1], b[2], b[3]]))
        } else {
            None
        }
    }

    pub fn is_printable(self) -> bool {
        self.bytes().iter().all(|c| (0x20..0x7F).contains(c))
    }
}

impl fmt::Display for FourCc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_printable() {
            for c in self.bytes() {
                write!(f, "{}", c as char)?;
            }
            Ok(())
        } else {
            write!(f, "0x{:08x}", self.0)
        }
    }
}

impl fmt::Debug for FourCc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "'{self}'")
    }
}

impl Serialize for FourCc {
    fn serialize<S: Serializer>(&self, s: S) -> std::result::Result<S::Ok, S::Error> {
        s.serialize_str(&self.to_string())
    }
}
