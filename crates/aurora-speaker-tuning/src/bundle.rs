// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Bundle discovery and loading.
//!
//! Sources, in the supervisor's search order:
//! 1. the unpacked vendor firmware tree ([`crate::VENDOR_DIR`]);
//! 2. the ESP's `vendorfw/firmware.tar`, members under [`crate::TAR_PREFIX`];
//! 3. an explicit path: a directory, a `.tar` or a `.tar.gz` (including
//!    aurora-audio-dsp collector bundles, whose members sit under one
//!    top-level directory).
//!
//! Every file the manifest lists is loaded and verified (size and SHA-256);
//! nothing else is read.  For development only, [`Bundle::from_tunings_root`]
//! reads an `AID<n>` folder straight from a macOS `Tunings` directory; such a
//! bundle is marked unverified.

use std::collections::BTreeMap;
use std::io::Read;
use std::path::{Path, PathBuf};

use crate::error::{Error, Result};
use crate::manifest::{self, admitted, Manifest, Source};

/// A bundle held in memory: files by bundle path (`AID<n>/DSP/...`,
/// `speaker-thiele-small.bin`, ...).
#[derive(Debug)]
pub struct Bundle {
    pub manifest: Manifest,
    pub files: BTreeMap<String, Vec<u8>>,
    pub source: PathBuf,
    /// False only for [`Bundle::from_tunings_root`].
    pub verified: bool,
    /// Members present in an archive but not listed by the manifest (ignored).
    pub unlisted: Vec<String>,
}

fn io(path: &Path, e: std::io::Error) -> Error {
    Error::Io { path: path.display().to_string(), message: e.to_string() }
}

/// Files a bundle may carry beside the manifest's list, if listed: the DSP
/// database subtree and per-AID configuration (see [`crate::select`]).
pub use crate::manifest::DSP_DATABASE;

impl Bundle {
    fn from_map(mut all: BTreeMap<String, Vec<u8>>, source: PathBuf) -> Result<Bundle> {
        let manifest = if let Some(m) = all.remove(manifest::NAME) {
            Manifest::parse(&m)?
        } else if let Some(m) = all.remove(manifest::COLLECTOR_NAME) {
            Manifest::parse_collector(&m)?
        } else {
            return Err(Error::format(&source.display().to_string(), "no manifest.json or metadata.json"));
        };
        let mut files = BTreeMap::new();
        for rec in manifest.records() {
            let data = all.remove(&rec.path).ok_or_else(|| Error::Integrity {
                path: rec.path.clone(),
                message: "listed in the manifest but absent".into(),
            })?;
            rec.verify(&data)?;
            files.insert(rec.path.clone(), data);
        }
        let unlisted = all.into_keys().collect();
        Ok(Bundle { manifest, files, source, verified: true, unlisted })
    }

    /// Loads and verifies a bundle from an unpacked directory.
    pub fn from_dir(dir: &Path) -> Result<Bundle> {
        let mname = if dir.join(manifest::NAME).exists() { manifest::NAME } else { manifest::COLLECTOR_NAME };
        let mbytes = std::fs::read(dir.join(mname)).map_err(|e| io(&dir.join(mname), e))?;
        let manifest =
            if mname == manifest::NAME { Manifest::parse(&mbytes)? } else { Manifest::parse_collector(&mbytes)? };
        let mut files = BTreeMap::new();
        for rec in manifest.records() {
            let p = dir.join(&rec.path);
            // A listed path must stay inside the bundle directory.
            let meta = std::fs::symlink_metadata(&p).map_err(|e| io(&p, e))?;
            if !meta.file_type().is_file() {
                return Err(Error::Integrity { path: rec.path.clone(), message: "not a regular file".into() });
            }
            let data = std::fs::read(&p).map_err(|e| io(&p, e))?;
            rec.verify(&data)?;
            files.insert(rec.path.clone(), data);
        }
        Ok(Bundle { manifest, files, source: dir.to_owned(), verified: true, unlisted: Vec::new() })
    }

    /// Loads a `.tar` or `.tar.gz`.  `prefix` selects members under a path
    /// (`apple/audio-tuning/` for the ESP archive); `None` finds the
    /// directory holding the manifest.
    pub fn from_tar(path: &Path, prefix: Option<&str>) -> Result<Bundle> {
        let raw = std::fs::read(path).map_err(|e| io(path, e))?;
        let bytes = if raw.starts_with(&[0x1F, 0x8B]) {
            let mut out = Vec::new();
            flate2::read::GzDecoder::new(&raw[..])
                .take(manifest::MAX_BYTES * 2)
                .read_to_end(&mut out)
                .map_err(|e| io(path, e))?;
            out
        } else {
            raw
        };
        let ctx = path.display().to_string();
        let mut members: BTreeMap<String, Vec<u8>> = BTreeMap::new();
        let mut total = 0u64;
        let mut ar = tar::Archive::new(&bytes[..]);
        for entry in ar.entries().map_err(|e| io(path, e))? {
            let mut entry = entry.map_err(|e| io(path, e))?;
            if !entry.header().entry_type().is_file() {
                continue;
            }
            let name = entry.path().map_err(|e| io(path, e))?.to_string_lossy().into_owned();
            let name = name.trim_start_matches("./").to_string();
            if let Some(p) = prefix {
                if !name.starts_with(p) {
                    continue;
                }
            }
            total += entry.size();
            if members.len() >= 4 * manifest::MAX_FILES || total > manifest::MAX_BYTES * 2 {
                return Err(Error::format(&ctx, "archive exceeds the bundle limits"));
            }
            let mut data = Vec::with_capacity(entry.size() as usize);
            entry.read_to_end(&mut data).map_err(|e| io(path, e))?;
            if members.insert(name.clone(), data).is_some() {
                return Err(Error::format(&ctx, format!("member {name} appears twice")));
            }
        }
        let root = match prefix {
            Some(p) => p.to_string(),
            None => {
                let m = members
                    .keys()
                    .find(|k| k.ends_with(manifest::NAME) || k.ends_with(manifest::COLLECTOR_NAME))
                    .ok_or_else(|| Error::format(&ctx, "no manifest.json or metadata.json in the archive"))?;
                m.rsplit_once('/').map(|(d, _)| format!("{d}/")).unwrap_or_default()
            }
        };
        let mut rel = BTreeMap::new();
        for (k, v) in members {
            if let Some(r) = k.strip_prefix(&root) {
                if !admitted(r) {
                    return Err(Error::format(&ctx, format!("inadmissible member {k:?}")));
                }
                rel.insert(r.to_string(), v);
            }
        }
        Bundle::from_map(rel, path.to_owned())
    }

    /// Opens an explicit path: directory, `.tar` or `.tar.gz`.
    pub fn open(path: &Path) -> Result<Bundle> {
        if path.is_dir() {
            Bundle::from_dir(path)
        } else {
            Bundle::from_tar(path, None)
        }
    }

    /// The default search: vendor firmware tree, then the ESP archive.
    pub fn discover() -> Result<Bundle> {
        let dir = Path::new(crate::VENDOR_DIR);
        if dir.join(manifest::NAME).exists() {
            return Bundle::from_dir(dir);
        }
        for root in crate::ESP_ROOTS {
            let tarp = Path::new(root).join("vendorfw/firmware.tar");
            if tarp.is_file() {
                match Bundle::from_tar(&tarp, Some(crate::TAR_PREFIX)) {
                    Ok(b) => return Ok(b),
                    Err(Error::Format { message, .. }) if message.contains("no manifest") => continue,
                    Err(e) => return Err(e),
                }
            }
        }
        Err(Error::Io { path: crate::VENDOR_DIR.into(), message: "no speaker tuning bundle found".into() })
    }

    /// Development only: an `AID<n>` folder read in place from a macOS
    /// `Tunings` directory, plus its `DeviceDSPDatabase.plist` entry if the
    /// directory has one.  Nothing is verified.
    pub fn from_tunings_root(root: &Path, acoustic_id: u32) -> Result<Bundle> {
        let aid = format!("AID{acoustic_id}");
        let base = root.join(&aid);
        let mut files = BTreeMap::new();
        let mut stack = vec![base.clone()];
        while let Some(d) = stack.pop() {
            for e in std::fs::read_dir(&d).map_err(|e| io(&d, e))? {
                let e = e.map_err(|e| io(&d, e))?;
                let p = e.path();
                let ft = e.file_type().map_err(|e| io(&p, e))?;
                if ft.is_dir() {
                    stack.push(p);
                } else if ft.is_file() {
                    let rel = format!("{aid}/{}", p.strip_prefix(&base).unwrap().to_string_lossy());
                    if admitted(&rel) {
                        files.insert(rel, std::fs::read(&p).map_err(|e| io(&p, e))?);
                    }
                }
            }
        }
        let db = root.join(DSP_DATABASE);
        if db.is_file() {
            files.insert(DSP_DATABASE.to_string(), std::fs::read(&db).map_err(|e| io(&db, e))?);
        }
        let manifest = Manifest {
            source: Source::Collector,
            acoustic_id,
            aid,
            macos_build: None,
            machine_model: None,
            speaker_thiele_small: None,
            files: Vec::new(),
        };
        Ok(Bundle { manifest, files, source: root.to_owned(), verified: false, unlisted: Vec::new() })
    }

    pub fn file(&self, path: &str) -> Option<&[u8]> {
        self.files.get(path).map(Vec::as_slice)
    }

    /// A file inside the `AID<n>` folder, by its path relative to it.
    pub fn tuning(&self, relative: &str) -> Option<&[u8]> {
        self.file(&format!("{}/{relative}", self.manifest.aid))
    }

    pub fn spts(&self) -> Option<&[u8]> {
        self.file(manifest::SPTS_NAME)
    }
}
