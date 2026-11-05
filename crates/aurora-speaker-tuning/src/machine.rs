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
    /// The acoustic-id known independently of the bundle, if the platform
    /// exports one; the bundle's own must equal it.  TODO: no loader exports
    /// it yet and the blob header has no room for it (see docs/DESIGN.md),
    /// so on Linux this is `None`.
    pub acoustic_id: Option<u32>,
}

impl Machine {
    pub fn new(board: &str) -> Result<Machine> {
        if sgv2_core::blob::machine_field(board).is_none() {
            return Err(Error::format("machine", format!("board {board:?} does not fit the blob header")));
        }
        Ok(Machine { board: board.to_string(), acoustic_id: None })
    }

    /// Reads the running machine from a device tree directory.
    pub fn from_device_tree(root: &Path) -> Result<Machine> {
        let p = root.join("compatible");
        let raw = std::fs::read(&p).map_err(|e| Error::Io { path: p.display().to_string(), message: e.to_string() })?;
        let board = board_from_compatible(&raw)
            .ok_or_else(|| Error::format("device tree", "the first root compatible is not apple,<board>"))?;
        Machine::new(&board)
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
        std::fs::write(d.join("chosen").join(SPEAKER_ID_PROPERTY), 3u32.to_be_bytes()).unwrap();
        assert_eq!(speaker_id(&d).unwrap(), Some(3));
        std::fs::write(d.join("chosen").join(SPEAKER_ID_PROPERTY), [0, 3]).unwrap();
        assert!(speaker_id(&d).is_err());
        std::fs::write(d.join("compatible"), b"vendor,board\0").unwrap();
        assert!(Machine::from_device_tree(&d).is_err());
        let _ = std::fs::remove_dir_all(&d);
    }
}
