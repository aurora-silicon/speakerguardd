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
//!   The machine description below gives the amp and the analog gain the
//!   kernel fixes ("Amp Gain Volume", `cfg->amp_gain`; KERNEL.md 2.3), which
//!   the compiler uses as a cross-check: [`Description::full_scale_mv`]
//!   converts the gain's dBV (rms-referenced) to peak volts (Daybreak
//!   t-4662).
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

/// The speaker amp family.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Amp {
    /// TI SN012776 (tas2764): "Amp Gain Volume" is 11 dB plus 0.5 dB a
    /// step, read as the full-scale output peak in dBV.
    Sn012776 { gain_step: u32 },
    /// A fixed-gain amp: its full-scale output peak, mV.
    Fixed { full_scale_mv: u32 },
}

/// What the compiler needs to know about a board.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Description {
    pub board: &'static str,
    pub amps: usize,
    pub amp: Amp,
}

const fn sn(board: &'static str, amps: usize, gain_step: u32) -> Description {
    Description { board, amps, amp: Amp::Sn012776 { gain_step } }
}

/// The boards the supervisor supports (`sgv2-supervisor` machine table),
/// with the analog gain the kernel fixes on each: 15 on J3xx/J4xx/J6xx
/// (KERNEL.md 2.3).
pub const MACHINES: &[Description] = &[
    sn("j316s", 6, 15),
    sn("j316c", 6, 15),
    sn("j616s", 6, 15),
    sn("j616c", 6, 15),
    sn("j413", 4, 15),
    // 14-inch M2 Pro: six SN012776 (two woofer pairs and two tweeters), gain
    // 15 (macaudio j314 configuration, DT t602x-j414-j416).
    sn("j414s", 6, 15),
    // No gain control: 7.30 V peak at full scale (AID44 VoltsFullScale,
    // Daybreak t-4662).
    Description { board: "j700", amps: 2, amp: Amp::Fixed { full_scale_mv: 7_300 } },
];

pub fn lookup(board: &str) -> Option<&'static Description> {
    MACHINES.iter().find(|m| m.board == board)
}

impl Description {
    /// The amp's analog gain in hundredths of a dB, if it has a control.
    pub fn amp_gain_centi_db(&self) -> Option<i32> {
        match self.amp {
            Amp::Sn012776 { gain_step } => Some(1100 + 50 * gain_step as i32),
            Amp::Fixed { .. } => None,
        }
    }

    /// The output peak voltage at digital full scale, mV.  The gain's TLV
    /// is dBV relative to 1 V rms, so the peak is `sqrt(2) 10^(dB/20)`
    /// (Daybreak t-4662: 18.5 dBV is 11.899 V peak).
    pub fn full_scale_mv(&self) -> u32 {
        match self.amp {
            Amp::Sn012776 { .. } => {
                let db = f64::from(self.amp_gain_centi_db().unwrap()) / 100.0;
                (1000.0 * std::f64::consts::SQRT_2 * 10f64.powf(db / 20.0)).round() as u32
            }
            Amp::Fixed { full_scale_mv } => full_scale_mv,
        }
    }
}

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

    pub fn description(&self) -> Option<&'static Description> {
        lookup(&self.board)
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

    #[test]
    fn operating_points() {
        // J616s: gain 15 = 18.5 dBV = 8.414 V rms = 11.899 V peak.
        let j = lookup("j616s").unwrap();
        assert_eq!(j.amp_gain_centi_db(), Some(1850));
        assert_eq!(j.full_scale_mv(), 11899);
        assert_eq!(lookup("j700").unwrap().full_scale_mv(), 7300);
        assert!(lookup("j274").is_none());
        for m in MACHINES {
            assert!(sgv2_core::blob::machine_field(m.board).is_some(), "{}", m.board);
        }
    }
}
