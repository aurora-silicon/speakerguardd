// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! macOS release tracking.
//!
//! Apple can change a tuning schema or the units' behaviour in any macOS
//! update.  The reader accepts any build whose files parse and validate, but
//! says so when the build is not one the behaviour specification was
//! established on; the supervisor decides what that means for qualification.
//! Schema versions inside the files (spp3 sidecar `Version`, SpTS version,
//! manifest schema) are checked where they are read and fail hard when
//! unknown.

use crate::error::Diagnostics;

/// macOS builds the specification and this reader were validated against.
pub const VALIDATED_BUILDS: &[&str] = &["25G83"];

pub fn check(build: Option<&str>, diags: &mut Diagnostics) {
    match build {
        Some(b) if VALIDATED_BUILDS.contains(&b) => {}
        Some(b) => diags.warn(
            "macos-build-unvalidated",
            "manifest",
            format!(
                "macOS {b} is not a validated build ({}); re-run the differential rig",
                VALIDATED_BUILDS.join(", ")
            ),
        ),
        None => diags.warn("macos-build-unknown", "manifest", "the bundle does not record its macOS build"),
    }
}
