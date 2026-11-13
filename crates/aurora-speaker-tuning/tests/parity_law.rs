// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The shipped macOS parity attestations must be for this code's voicing
//! law. A change to any file `build.rs` hashes makes `parity.json` stale,
//! and `voicing-check` (so `speakerguardd-selfcheck verify` and
//! `aurora-speaker-upgrade`) then refuses every Mac. Re-run the gate
//! (`tools/diffrig/aidgate/README.md`, "When it runs") and commit the new
//! file in the same change.

use aurora_speaker_tuning::voicing::parity::{Attestations, LAW};
use std::path::Path;

#[test]
fn shipped_attestations_match_this_law() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../packaging/session/parity.json");
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let a = Attestations::parse(&bytes).expect("packaging/session/parity.json parses");
    assert_eq!(
        a.law, LAW,
        "packaging/session/parity.json attests voicing law {} but this code is law {}: \
         re-gate every release block with tools/diffrig/aidgate/attest.py and commit the new file",
        a.law, LAW
    );
}
