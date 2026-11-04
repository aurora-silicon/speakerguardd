// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Reader and validator for a Mac's own speaker tuning bundle.
//!
//! The Aurora installer copies, read-only, the Mac's `AID<n>` tuning folder
//! (selected by the device tree's `acoustic-id`) and its per-unit
//! `speaker-thiele-small` record into the vendor firmware, with a manifest
//! of sizes and SHA-256 hashes. This crate finds that bundle, verifies it
//! and exposes:
//! - the protection unit and its parameter sets (spp3, lsm2, atsp);
//! - the speaker graph and strip (the voicing chain around the unit);
//! - the per-driver factory calibration.
//!
//! No Apple data is part of this crate or its tests: fixtures are synthetic.
//!
//! Module ownership: lane A (tuning reader). See docs/CONVENTIONS.md.

pub mod bundle;
pub mod manifest;
pub mod preset;
pub mod spts;
// TODO(lane A): plist (binary and XML), austrip, propstrip, dspg, spp3
// sidecar, unit selection, and the compiler to `sgv2_core::blob`.

/// Where the vendor firmware is unpacked on Linux.
pub const VENDOR_DIR: &str = "/lib/firmware/vendor/apple/audio-tuning";
/// The member prefix inside the ESP's `vendorfw/firmware.tar`.
pub const TAR_PREFIX: &str = "apple/audio-tuning/";
/// ESP mount points searched for `vendorfw/firmware.tar`.
pub const ESP_ROOTS: &[&str] = &["/boot/efi", "/boot", "/efi", "/run/.system-efi"];

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    /// The bundle does not match its manifest, or is malformed.
    Invalid(String),
    NotFound,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Invalid(m) => write!(f, "invalid tuning bundle: {m}"),
            Error::NotFound => write!(f, "no speaker tuning bundle found"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

pub type Result<T> = std::result::Result<T, Error>;
