// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Public lsm2 AudioUnit parameter metadata collected on the Mac that owns
//! the tuning bundle. The installer hashes this file in `manifest.json`;
//! no platform value is compiled into speakerguardd.

use crate::error::{Error, Result};
use serde::{Deserialize, Serialize};

pub const NAME: &str = "lsm2-metadata.json";
pub const VERSION: u32 = 1;
pub const KIND: &str = "aurora-lsm2-parameter-metadata";
pub const MAX_BYTES: u64 = 4096;

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Metadata {
    pub velocity_parameter_max: f32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MetadataJson {
    kind: String,
    version: u32,
    velocity_parameter_max: f32,
}

pub fn parse(bytes: &[u8]) -> Result<Metadata> {
    if bytes.len() as u64 > MAX_BYTES {
        return Err(Error::format(NAME, "metadata file is over budget"));
    }
    let value: MetadataJson =
        serde_json::from_slice(bytes).map_err(|e| Error::format(NAME, format!("not lsm2 metadata: {e}")))?;
    if value.kind != KIND {
        return Err(Error::format(NAME, format!("kind {:?}", value.kind)));
    }
    if value.version != VERSION {
        return Err(Error::version(NAME, format!("metadata version {}; known: {VERSION}", value.version)));
    }
    if !value.velocity_parameter_max.is_finite() || value.velocity_parameter_max <= 0.0 {
        return Err(Error::format(NAME, "velocity_parameter_max is not finite and positive"));
    }
    Ok(Metadata { velocity_parameter_max: value.velocity_parameter_max })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_metadata_is_strict() {
        let good = br#"{"kind":"aurora-lsm2-parameter-metadata","version":1,"velocity_parameter_max":1.75}"#;
        assert_eq!(parse(good).unwrap().velocity_parameter_max, 1.75);
        for bad in [
            br#"{"kind":"wrong","version":1,"velocity_parameter_max":1.75}"#.as_slice(),
            br#"{"kind":"aurora-lsm2-parameter-metadata","version":2,"velocity_parameter_max":1.75}"#.as_slice(),
            br#"{"kind":"aurora-lsm2-parameter-metadata","version":1,"velocity_parameter_max":0}"#.as_slice(),
            br#"{"kind":"aurora-lsm2-parameter-metadata","version":1,"velocity_parameter_max":1.75,"extra":1}"#
                .as_slice(),
        ] {
            assert!(parse(bad).is_err());
        }
    }
}
