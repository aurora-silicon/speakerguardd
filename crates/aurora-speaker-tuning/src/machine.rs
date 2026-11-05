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
    /// Independently exported by aura in chosen/asahi,acoustic-id (FDT u32, big endian).
    pub acoustic_id: Option<u32>,
    /// SHA-256 of the root serial-number, without trailing NULs.
    pub unit_id_sha256: Option<String>,
}

impl Machine {
    pub fn new(board: &str) -> Result<Machine> {
        if sgv2_core::blob::machine_field(board).is_none() {
            return Err(Error::format("machine", format!("board {board:?} does not fit the blob header")));
        }
        Ok(Machine { board: board.to_string(), acoustic_id: None, unit_id_sha256: None })
    }

    /// Reads the running machine from a device tree directory.
    pub fn from_device_tree(root: &Path) -> Result<Machine> {
        let p = root.join("compatible");
        let raw = std::fs::read(&p).map_err(|e| Error::Io { path: p.display().to_string(), message: e.to_string() })?;
        let board = board_from_compatible(&raw)
            .ok_or_else(|| Error::format("device tree", "the first root compatible is not apple,<board>"))?;
        let mut machine = Machine::new(&board)?;
        machine.acoustic_id = acoustic_id(root)?;
        let serial = root.join("serial-number");
        machine.unit_id_sha256 = match std::fs::read(&serial) {
            Ok(raw) => Some(unit_id(&raw).ok_or_else(|| Error::format("device tree", "malformed serial-number"))?),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
            Err(e) => return Err(Error::Io { path: serial.display().to_string(), message: e.to_string() }),
        };
        Ok(machine)
    }
}

/// Canonical target-type: lowercase board name, without the ADT's AP suffix.
pub fn valid_board(board: &str) -> bool {
    !board.is_empty()
        && board.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit())
        && sgv2_core::blob::machine_field(board).is_some()
}

/// Same bytes on macOS (ADT) and Linux (aura's root serial-number copy).
/// No whitespace/case normalization and no raw identifier in diagnostics.
pub fn unit_id(raw: &[u8]) -> Option<String> {
    let end = raw.iter().rposition(|&b| b != 0).map_or(0, |i| i + 1);
    let serial = &raw[..end];
    if serial.is_empty() || serial.len() > 128 || !serial.iter().all(|b| (0x21..=0x7e).contains(b)) {
        return None;
    }
    Some(crate::manifest::sha256_hex(serial))
}

pub const ACOUSTIC_ID_PROPERTY: &str = "asahi,acoustic-id";

/// Missing exports cannot authorize a production compile. Malformed exports
/// are errors, never inferred from the bundle or a board-to-AID table.
pub fn acoustic_id(root: &Path) -> Result<Option<u32>> {
    let p = root.join("chosen").join(ACOUSTIC_ID_PROPERTY);
    match std::fs::read(&p) {
        Ok(b) => {
            let word: [u8; 4] = b.try_into().map_err(|_| Error::format("device tree", "malformed acoustic-id"))?;
            let value = u32::from_be_bytes(word);
            if value == 0 || value >= 100000 {
                return Err(Error::format("device tree", "acoustic-id outside 1..100000"));
            }
            Ok(Some(value))
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(Error::Io { path: p.display().to_string(), message: e.to_string() }),
    }
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
        assert_eq!(Machine::from_device_tree(&d).unwrap().acoustic_id, None);
        std::fs::write(d.join("chosen").join(ACOUSTIC_ID_PROPERTY), 35u32.to_be_bytes()).unwrap();
        std::fs::write(d.join("serial-number"), b"abc\0\0").unwrap();
        let m = Machine::from_device_tree(&d).unwrap();
        assert_eq!(m.acoustic_id, Some(35));
        assert_eq!(m.unit_id_sha256, Some(crate::manifest::sha256_hex(b"abc")));
        for raw in [vec![35], 0u32.to_be_bytes().to_vec(), 35u32.to_le_bytes().to_vec()] {
            std::fs::write(d.join("chosen").join(ACOUSTIC_ID_PROPERTY), raw).unwrap();
            assert!(Machine::from_device_tree(&d).is_err());
        }
        std::fs::write(d.join("chosen").join(ACOUSTIC_ID_PROPERTY), 35u32.to_be_bytes()).unwrap();
        for raw in [b"".as_slice(), b"abc def", b"a\0b", b"\xff"] {
            assert_eq!(unit_id(raw), None);
        }
        std::fs::write(d.join("serial-number"), b"a\0b").unwrap();
        assert!(Machine::from_device_tree(&d).is_err());
        std::fs::remove_file(d.join("serial-number")).unwrap();
        std::fs::write(d.join("chosen").join(SPEAKER_ID_PROPERTY), 3u32.to_be_bytes()).unwrap();
        assert_eq!(speaker_id(&d).unwrap(), Some(3));
        std::fs::write(d.join("chosen").join(SPEAKER_ID_PROPERTY), [0, 3]).unwrap();
        assert!(speaker_id(&d).is_err());
        std::fs::write(d.join("compatible"), b"vendor,board\0").unwrap();
        assert!(Machine::from_device_tree(&d).is_err());
        let _ = std::fs::remove_dir_all(&d);
    }
}
