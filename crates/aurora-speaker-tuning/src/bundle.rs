// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Bundle discovery and loading.
//!
//! Search order: the unpacked vendor firmware tree, then the ESP's
//! `vendorfw/firmware.tar`, then an explicit path (a directory, or a
//! collector bundle `audio-tuning-AID<n>.tar.gz` from aurora-audio-dsp).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use crate::manifest::{admitted, Manifest};
use crate::{Error, Result};

/// A verified bundle held in memory: every file named by the manifest,
/// by its manifest path (`AID35/DSP/...`, `speaker-thiele-small.bin`).
#[derive(Debug)]
pub struct Bundle {
    pub manifest: Manifest,
    pub files: BTreeMap<String, Vec<u8>>,
    pub source: PathBuf,
}

impl Bundle {
    /// Load and verify a bundle from an unpacked directory.
    pub fn from_dir(dir: &Path) -> Result<Bundle> {
        let manifest = Manifest::parse(&std::fs::read(dir.join(crate::manifest::NAME))?)?;
        let mut files = BTreeMap::new();
        for rec in manifest.records() {
            if !admitted(&rec.path) {
                return Err(Error::Invalid(format!("path {:?}", rec.path)));
            }
            let data = std::fs::read(dir.join(&rec.path))?;
            rec.verify(&data)?;
            files.insert(rec.path.clone(), data);
        }
        Ok(Bundle { manifest, files, source: dir.to_owned() })
    }

    /// The default search. TODO(lane A): the ESP tar and collector bundles.
    pub fn discover() -> Result<Bundle> {
        let dir = Path::new(crate::VENDOR_DIR);
        if dir.join(crate::manifest::NAME).exists() {
            return Bundle::from_dir(dir);
        }
        Err(Error::NotFound)
    }

    pub fn file(&self, path: &str) -> Option<&[u8]> {
        self.files.get(path).map(Vec::as_slice)
    }

    /// A file inside the `AID<n>` folder, by its path relative to it.
    pub fn tuning(&self, relative: &str) -> Option<&[u8]> {
        self.file(&format!("{}/{relative}", self.manifest.aid))
    }
}
