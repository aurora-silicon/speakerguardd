// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The speaker volume control's dB range, from the Mac's own bundle.
//!
//! macOS's built-in speaker volume is a CoreAudio level control: a dB range
//! and the 2-over-1 transfer function ([`sgv2_core::voicing::volume::Curve`]).
//! The range is per-AID device configuration, which every installer bundle
//! already carries (`Config/AID<n>.plist`, required by the manifest): the
//! `Built-in Speaker` DSP device's `OutputVolumeMin` and `OutputVolumeMax`.
//! Reading it here means a Mac nobody has measured gets its own law on day
//! one, with no table in this repository and no step on the Mac.
//!
//! A bundle without the file or the keys (a development tree, or a future
//! macOS that renames them) keeps the legacy cubic taper, with a warning.

use serde::Serialize;
use sgv2_core::voicing::volume::Curve;

use crate::bundle::Bundle;
use crate::error::Diagnostics;
use crate::plist::{self, Value};
use crate::select::SPEAKER_DEVICE;

/// Where the range lives inside the device configuration.
pub const DSP_DEVICES: &str = "DSP Devices";
pub const PARAMETERS: &str = "Parameters";
pub const MIN_KEY: &str = "OutputVolumeMin";
pub const MAX_KEY: &str = "OutputVolumeMax";

/// The range the bundle states, and the file it came from.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct VolumeRange {
    pub min_db: f64,
    pub max_db: f64,
    pub source: String,
}

impl VolumeRange {
    pub fn curve(&self) -> Option<Curve> {
        Curve::hal(self.min_db, self.max_db).ok()
    }
}

/// The device configuration's bundle path.
pub fn config_path(b: &Bundle) -> String {
    format!("Config/{}.plist", b.manifest.aid)
}

/// The speaker control's range, or `None` (with a warning) when the bundle
/// does not state a usable one.
pub fn speaker_range(b: &Bundle, diags: &mut Diagnostics) -> Option<VolumeRange> {
    let path = config_path(b);
    let Some(bytes) = b.file(&path) else {
        diags.warn("hal-volume-absent", &path, "no device configuration: the volume law stays cubic");
        return None;
    };
    match range_from_config(bytes) {
        Ok(Some((min_db, max_db))) => {
            let r = VolumeRange { min_db, max_db, source: path.clone() };
            if r.curve().is_none() {
                diags.warn(
                    "hal-volume-invalid",
                    &path,
                    format!("{SPEAKER_DEVICE} range [{min_db}, {max_db}] dB is not a usable law: the volume law stays cubic"),
                );
                return None;
            }
            diags.info(
                "hal-volume",
                &path,
                format!("{SPEAKER_DEVICE} volume law: HAL 2-over-1 over the bundle's range"),
            );
            Some(r)
        }
        Ok(None) => {
            diags.warn(
                "hal-volume-absent",
                &path,
                format!("no {SPEAKER_DEVICE} {MIN_KEY}/{MAX_KEY}: the volume law stays cubic"),
            );
            None
        }
        Err(e) => {
            diags.warn("hal-volume-unreadable", &path, format!("{e}: the volume law stays cubic"));
            None
        }
    }
}

/// `(OutputVolumeMin, OutputVolumeMax)` of the speaker DSP device, if both
/// are present and numeric.
pub fn range_from_config(bytes: &[u8]) -> Result<Option<(f64, f64)>, String> {
    let (root, _) = plist::parse(bytes).map_err(|e| format!("not a property list: {e:?}"))?;
    let params = root
        .as_dict()
        .and_then(|d| d.get(DSP_DEVICES))
        .and_then(Value::as_dict)
        .and_then(|d| d.get(SPEAKER_DEVICE))
        .and_then(Value::as_dict)
        .and_then(|d| d.get(PARAMETERS))
        .and_then(Value::as_dict);
    let Some(p) = params else { return Ok(None) };
    let num = |k: &str| p.get(k).and_then(Value::as_number);
    Ok(match (num(MIN_KEY), num(MAX_KEY)) {
        (Some(lo), Some(hi)) => Some((lo, hi)),
        _ => None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(min: &str, max: &str) -> Vec<u8> {
        format!(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
<key>DSP Devices</key><dict>
  <key>Built-in Headphone</key><dict><key>Parameters</key><dict>
    <key>OutputVolumeMax</key><integer>0</integer><key>OutputVolumeMin</key><real>-11</real></dict></dict>
  <key>Built-in Speaker</key><dict><key>Parameters</key><dict>
    <key>OutputVolumeMax</key>{max}<key>OutputVolumeMin</key>{min}</dict></dict>
</dict></dict></plist>"#
        )
        .into_bytes()
    }

    #[test]
    fn reads_the_speaker_range_not_the_headphone_one() {
        // Synthetic values: the reader, not any Mac's configuration.
        let c = config("<real>-42.5</real>", "<integer>0</integer>");
        assert_eq!(range_from_config(&c), Ok(Some((-42.5, 0.0))));
    }

    #[test]
    fn missing_keys_are_none_and_garbage_is_an_error() {
        let c = config("<string>x</string>", "<integer>0</integer>");
        assert_eq!(range_from_config(&c), Ok(None));
        assert!(range_from_config(b"not a plist").is_err());
        let empty = br#"<?xml version="1.0"?><plist version="1.0"><dict/></plist>"#;
        assert_eq!(range_from_config(empty), Ok(None));
    }

    #[test]
    fn a_range_that_adds_drive_gives_no_curve() {
        let r = VolumeRange { min_db: -30.0, max_db: 2.0, source: String::new() };
        assert_eq!(r.curve(), None);
        let r = VolumeRange { min_db: -30.0, max_db: 0.0, source: String::new() };
        assert_eq!(r.curve(), Some(Curve::Hal { min_db: -30.0, max_db: 0.0 }));
    }
}
