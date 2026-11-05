// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The machine a blob is compiled for.
//!
//! The blob header (`sgv2_core::blob`, bytes 36 to 48) names the board and
//! the amps' operating point, and the supervisor refuses a blob that names
//! another board or an operating point the card does not run:
//!
//! * the board: the device tree's first root `compatible` without its
//!   `apple,` vendor (`j616s`);
//! * the operating point: the output peak voltage at digital full scale,
//!   in millivolts, from the tuning (spp3 `ag`, lsm2 `VoltsFullScale`).
//!
//! The compiler knows nothing else about the board, and has no table of
//! boards: a new Mac compiles on day one. The amp count and the analog gain
//! are the running Mac's, and are checked there: the self-check refuses a
//! tuning whose unit has another channel count than the device tree's amps,
//! and the supervisor checks the blob's channels against the self-check's
//! record and its operating point against the amps' "Amp Gain Volume"
//! readback (the gain the kernel fixed, KERNEL.md 2.3).
//!
//! The speaker-assembly ID (`spid`): macOS publishes it as the IORegistry
//! `speakerID` of the built-in speaker, read from two board straps (AP
//! GPIO 52/53 on the J616s).  The installer records it in the bundle
//! manifest (`speaker_id`).  Linux has no passive reader of the straps:
//! requesting the GPIO lines could change their ownership.  The independent
//! route is a boot-time export by m1n1 as `/chosen/asahi,speaker-id` (one
//! big-endian u32), designed but not implemented yet; [`speaker_id`] reads
//! it when present.  Selection order: an explicit override, the manifest,
//! then the device tree ([`crate::tuning::Tuning::load`]).
//!
//! The unit: SpTS is one unit's factory calibration, so a bundle copied
//! from another Mac of the same model would pass every other check and seed
//! the closed loop with the wrong resistance (about 8 K of temperature error
//! per 3 % of R).  The installer records [`unit_id`] of the root
//! `serial-number` of the Mac it collects on (macOS's `IODeviceTree` root,
//! the ADT property aura copies into the device tree root unchanged), and
//! the board ([`board_from_target_type`]).  [`Machine::from_device_tree`]
//! reads the same property on Linux.  Only the hash is stored; no serial
//! number reaches the bundle, the blob, a diagnostic or a log.  The rules
//! are in [`crate::compile`]: another unit or board refuses, an unknown
//! unit runs open loop.

use crate::error::{Error, Result};
use std::path::Path;

/// The live device tree.
pub const DEVICE_TREE: &str = "/proc/device-tree";
/// The loader's speaker-ID export, under `chosen`.
pub const SPEAKER_ID_PROPERTY: &str = "asahi,speaker-id";

/// The machine a blob is compiled for.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Machine {
    /// The board: the first root compatible without `apple,`.
    pub board: String,
    /// The acoustic-id known independently of the bundle, if the platform
    /// exports one; the bundle's own must equal it.  TODO: no loader exports
    /// it yet and the blob header has no room for it (see docs/DESIGN.md),
    /// so on Linux this is `None`.  The unit identity below binds the bundle
    /// more tightly: a matching unit was collected on this very Mac.
    pub acoustic_id: Option<u32>,
    /// The unit the blob is compiled for ([`unit_id`] of its serial number).
    pub unit: Unit,
}

/// What the compiler knows about the unit it compiles for.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Unit {
    /// [`unit_id`] of the device tree's root `serial-number`.
    Known(String),
    /// No unit identity: no device tree (`--board`), or a device tree
    /// without a `serial-number`.
    Unknown,
    /// The device tree's `serial-number` is not one serial number.  Treated
    /// as [`Unit::Unknown`] (open loop), never as a match.
    Malformed,
}

impl Machine {
    pub fn new(board: &str) -> Result<Machine> {
        if sgv2_core::blob::machine_field(board).is_none() {
            return Err(Error::format("machine", format!("board {board:?} does not fit the blob header")));
        }
        Ok(Machine { board: board.to_string(), acoustic_id: None, unit: Unit::Unknown })
    }

    /// A machine of a known unit, for compiling offline for another Mac
    /// (`--unit-id`) and for tests.
    pub fn with_unit(board: &str, unit_id: &str) -> Result<Machine> {
        if !valid_unit_id(unit_id) {
            return Err(Error::format("machine", "a unit ID is 64 lowercase hex digits"));
        }
        Ok(Machine { unit: Unit::Known(unit_id.to_string()), ..Machine::new(board)? })
    }

    /// Reads the running machine from a device tree directory.
    pub fn from_device_tree(root: &Path) -> Result<Machine> {
        let p = root.join("compatible");
        let raw = std::fs::read(&p).map_err(|e| Error::Io { path: p.display().to_string(), message: e.to_string() })?;
        let board = board_from_compatible(&raw)
            .ok_or_else(|| Error::format("device tree", "the first root compatible is not apple,<board>"))?;
        let serial = root.join(SERIAL_NUMBER);
        let unit = match std::fs::read(&serial) {
            Ok(raw) => unit_id(&raw).map_or(Unit::Malformed, Unit::Known),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Unit::Unknown,
            Err(e) => return Err(Error::Io { path: serial.display().to_string(), message: e.to_string() }),
        };
        Ok(Machine { unit, ..Machine::new(&board)? })
    }
}

/// The device tree's root property both macOS (ADT) and Linux (aura's copy)
/// carry.
pub const SERIAL_NUMBER: &str = "serial-number";
/// Domain separation of [`unit_id`]: the hash of a serial number is no
/// plain SHA-256 that a table of serial hashes could look up.
pub const UNIT_ID_DOMAIN: &[u8] = b"aurora-speaker-unit-v1\0";

/// The unit ID of a raw `serial-number` property: SHA-256 over
/// [`UNIT_ID_DOMAIN`] and the serial's bytes without trailing NULs (macOS
/// pads the property to 32 bytes, the device tree carries one NUL).  `None`
/// unless what remains is 1 to 64 printable, non-space ASCII bytes.  No
/// case or whitespace folding: both sides read the same bytes.
pub fn unit_id(raw: &[u8]) -> Option<String> {
    let end = raw.iter().rposition(|&b| b != 0).map_or(0, |i| i + 1);
    let serial = &raw[..end];
    if serial.is_empty() || serial.len() > 64 || !serial.iter().all(|b| (0x21..=0x7e).contains(b)) {
        return None;
    }
    let mut data = UNIT_ID_DOMAIN.to_vec();
    data.extend_from_slice(serial);
    Some(crate::manifest::sha256_hex(&data))
}

/// 64 lowercase hex digits.
pub fn valid_unit_id(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'))
}

/// A canonical board name, as the device tree's compatible carries it:
/// lowercase letters and digits that fit the blob header.
pub fn valid_board(board: &str) -> bool {
    !board.is_empty()
        && board.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit())
        && sgv2_core::blob::machine_field(board).is_some()
}

/// The board of the ADT root's `target-type` (`J616s` -> `j616s`; a trailing
/// `AP`, as `target-sub-type` has, is dropped), as the installer records it.
pub fn board_from_target_type(raw: &str) -> Option<String> {
    let t = raw.trim_end_matches('\0');
    let t = t.strip_suffix("AP").unwrap_or(t).to_ascii_lowercase();
    valid_board(&t).then_some(t)
}

/// The first `compatible` entry without its `apple,` vendor.
pub fn board_from_compatible(raw: &[u8]) -> Option<String> {
    let first = raw.split(|&b| b == 0).next()?;
    let s = std::str::from_utf8(first).ok()?;
    s.strip_prefix("apple,").filter(|b| !b.is_empty()).map(str::to_owned)
}

/// The speaker ID the loader exported into the device tree, if any.  A
/// malformed property is an error, never a guess.
pub fn speaker_id(root: &Path) -> Result<Option<u32>> {
    let p = root.join("chosen").join(SPEAKER_ID_PROPERTY);
    match std::fs::read(&p) {
        Ok(b) => match <[u8; 4]>::try_from(b.as_slice()) {
            Ok(w) => Ok(Some(u32::from_be_bytes(w))),
            Err(_) => Err(Error::format("device tree", format!("{SPEAKER_ID_PROPERTY} is {} bytes, not 4", b.len()))),
        },
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(Error::Io { path: p.display().to_string(), message: e.to_string() }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_the_board_and_speaker_id() {
        let d = std::env::temp_dir().join(format!("ast-machine-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(d.join("chosen")).unwrap();
        std::fs::write(d.join("compatible"), b"apple,j616s\0apple,t6041\0apple,arm-platform\0").unwrap();
        assert_eq!(Machine::from_device_tree(&d).unwrap().board, "j616s");
        assert_eq!(speaker_id(&d).unwrap(), None);
        std::fs::write(d.join("chosen").join(SPEAKER_ID_PROPERTY), 3u32.to_be_bytes()).unwrap();
        assert_eq!(speaker_id(&d).unwrap(), Some(3));
        std::fs::write(d.join("chosen").join(SPEAKER_ID_PROPERTY), [0, 3]).unwrap();
        assert!(speaker_id(&d).is_err());
        // The unit: absent, a serial number, a malformed one.
        assert_eq!(Machine::from_device_tree(&d).unwrap().unit, Unit::Unknown);
        std::fs::write(d.join(SERIAL_NUMBER), b"TESTSERIAL0\0").unwrap();
        let want = unit_id(b"TESTSERIAL0").unwrap();
        assert_eq!(Machine::from_device_tree(&d).unwrap().unit, Unit::Known(want.clone()));
        // macOS's 32-byte padded property gives the same ID.
        let mut padded = b"TESTSERIAL0".to_vec();
        padded.resize(32, 0);
        assert_eq!(unit_id(&padded), Some(want.clone()));
        std::fs::write(d.join(SERIAL_NUMBER), b"TEST SERIAL\0").unwrap();
        assert_eq!(Machine::from_device_tree(&d).unwrap().unit, Unit::Malformed);
        std::fs::remove_file(d.join(SERIAL_NUMBER)).unwrap();
        std::fs::write(d.join("compatible"), b"vendor,board\0").unwrap();
        assert!(Machine::from_device_tree(&d).is_err());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn unit_ids() {
        // A known vector: the domain tag and the bytes, nothing else.
        assert_eq!(unit_id(b"TESTSERIAL0"), Some(crate::manifest::sha256_hex(b"aurora-speaker-unit-v1\0TESTSERIAL0")));
        assert_ne!(unit_id(b"TESTSERIAL0"), Some(crate::manifest::sha256_hex(b"TESTSERIAL0")));
        // The vector every implementation shares (the installer's Python
        // and Swift collectors test the same one).
        assert_eq!(
            unit_id(b"SYNTHETIC0A\0").as_deref(),
            Some("22fea1b624a25cb3e103ed668fc37e1ef5c0d445e195029caf273827923c793d")
        );
        assert_ne!(unit_id(b"TESTSERIAL0"), unit_id(b"testserial0"));
        for raw in [b"".as_slice(), b"\0\0", b"A B", b"A\0B", b"\xff", b"\tA", &[b'A'; 65]] {
            assert_eq!(unit_id(raw), None, "{raw:?}");
        }
        assert!(valid_unit_id(&unit_id(b"X").unwrap()));
        assert!(!valid_unit_id("ABC"));
        assert!(Machine::with_unit("j413", "ABC").is_err());
        assert_eq!(Machine::with_unit("j413", &"a".repeat(64)).unwrap().unit, Unit::Known("a".repeat(64)));
    }

    #[test]
    fn boards_from_target_type() {
        assert_eq!(board_from_target_type("J616s").as_deref(), Some("j616s"));
        assert_eq!(board_from_target_type("J616sAP").as_deref(), Some("j616s"));
        assert_eq!(board_from_target_type("J413\0").as_deref(), Some("j413"));
        assert_eq!(board_from_target_type("J700").as_deref(), Some("j700"));
        for bad in ["", "AP", "J 413", "J413-x", "Mac14,2"] {
            assert_eq!(board_from_target_type(bad), None, "{bad}");
        }
        assert!(valid_board("j375c"));
        assert!(!valid_board("J413"));
    }
}
