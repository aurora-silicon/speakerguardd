// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compile BuzzKill (`aufx/buzk/appl`).
//!
//! The supported AID states all set the strip-level bypass.  Native bypass
//! is exact identity with zero effective latency, so the chain omits it.
//! The active unit's linked detector and moving-notch time law have not been
//! established closely enough for the conformance envelope; an active state
//! is therefore a hard refusal instead of an approximation.

use crate::error::{Diagnostics, Error, Result};
use crate::tuning::Stage;
use sgv2_core::voicing::buzz::Params;

pub fn compile(stage: &Stage, _fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    if stage.bypass {
        diags.info("buzz-bypassed", &stage.name, "strip bypass is exact identity; box omitted");
        return Ok(None);
    }
    Err(Error::version(
        &stage.name,
        "active BuzzKill detector/envelope law is not carried; only the shipped bypassed state is exact",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fourcc::FourCc;

    fn stage(bypass: bool) -> Stage {
        Stage {
            name: "Buzz".into(),
            kind: "aufx/buzk/appl".into(),
            unit: Some(crate::aupreset::UnitId {
                kind: FourCc::from_bytes(*b"aufx"),
                subtype: FourCc::from_bytes(*b"buzk"),
                manufacturer: FourCc::from_bytes(*b"appl"),
            }),
            inputs: 1,
            outputs: 1,
            channels: Some(2),
            bypass,
            preset_name: Some("synthetic".into()),
            state: None,
            preset_extra_keys: Vec::new(),
            preset_extra: Default::default(),
            strip_properties: Vec::new(),
            property_files: Vec::new(),
            before_protection: true,
        }
    }

    #[test]
    fn accepts_only_graph_bypass() {
        let mut diags = Diagnostics::default();
        assert_eq!(compile(&stage(true), 48_000.0, &mut diags).unwrap(), None);
        assert!(diags.items.iter().any(|d| d.code == "buzz-bypassed"));
        assert!(compile(&stage(false), 48_000.0, &mut Diagnostics::default()).is_err());
    }
}
