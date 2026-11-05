// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Bundle integrity records.
//!
//! * `manifest.json`, schema 2, kind `aurora-audio-tuning`: what the Aurora
//!   installer writes next to the collected folder.  Schema 2 states that
//!   the set is complete (`complete`, `source_file_count`), names both
//!   selection files, and may carry the speaker-assembly ID (`speaker_id`).
//! * `metadata.json`, version 1, kind `aurora-audio-dsp tuning bundle`: what
//!   aurora-audio-dsp's macOS collector writes (development bundles).
//!
//! Both list every file with its size and SHA-256; both are read into one
//! [`Manifest`].

use crate::error::{Error, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const NAME: &str = "manifest.json";
pub const COLLECTOR_NAME: &str = "metadata.json";
pub const SPTS_NAME: &str = "speaker-thiele-small.bin";
/// Files a manifest may list outside `AID<n>/`: macOS's DSP database (the
/// graph and strip selection) and the AID's device configuration.  This is
/// the shared database of macOS 26 and earlier; macOS 27 keeps one per AID
/// instead, [`aid_dsp_database`], inside the folder.
pub const DSP_DATABASE: &str = "DeviceDSPDatabase.plist";

/// The per-AID DSP database of macOS 27 (`AID<n>/DSP/DSPDatabase.plist`).
pub fn aid_dsp_database(aid: &str) -> String {
    format!("{aid}/DSP/DSPDatabase.plist")
}
/// Limits the installer applies when collecting; a bundle beyond them is
/// not one it wrote.
pub const MAX_FILES: usize = 1024;
pub const MAX_BYTES: u64 = 64 << 20;
/// The installer manifest schema this reader knows.
pub const SCHEMA: u32 = 2;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Record {
    pub path: String,
    pub bytes: u64,
    pub sha256: String,
}

impl Record {
    pub fn verify(&self, data: &[u8]) -> Result<()> {
        if data.len() as u64 != self.bytes {
            return Err(Error::Integrity {
                path: self.path.clone(),
                message: format!("{} bytes, manifest says {}", data.len(), self.bytes),
            });
        }
        let got = sha256_hex(data);
        if got != self.sha256 {
            return Err(Error::Integrity {
                path: self.path.clone(),
                message: format!("sha256 {got}, manifest says {}", self.sha256),
            });
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Source {
    Installer,
    Collector,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Manifest {
    pub source: Source,
    pub acoustic_id: u32,
    /// `AID<n>`.
    pub aid: String,
    /// macOS build the tuning was collected from (`kern.osversion`), if known.
    pub macos_build: Option<String>,
    pub machine_model: Option<String>,
    /// The speaker-assembly ID (macOS `speakerID`, HAL `spid`) the
    /// installer read on this Mac, if it could.
    pub speaker_id: Option<u32>,
    /// The DSP database the installer named: [`DSP_DATABASE`] or
    /// [`aid_dsp_database`].  `None` when the manifest names none (the
    /// collector), and the reader looks for either.
    pub dsp_database: Option<String>,
    pub speaker_thiele_small: Option<Record>,
    pub files: Vec<Record>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct InstallerJson {
    #[serde(rename = "schema")]
    _schema: u32,
    kind: String,
    source_file_count: u64,
    complete: bool,
    acoustic_id: u32,
    aid: String,
    macos_build: String,
    speaker_thiele_small: Option<Record>,
    device_dsp_database: String,
    config: String,
    files: Vec<Record>,
    based_on: Option<String>,
    /// Python writes `null` when unknown, Swift omits the key.
    #[serde(default)]
    speaker_id: Option<u32>,
}

#[derive(Deserialize)]
struct CollectorJson {
    kind: String,
    version: u32,
    machine_model: Option<String>,
    acoustic_id_name: String,
    files: Vec<Record>,
}

impl Manifest {
    pub fn records(&self) -> impl Iterator<Item = &Record> {
        self.files.iter().chain(self.speaker_thiele_small.iter())
    }

    /// Parses `manifest.json` (installer).
    pub fn parse(bytes: &[u8]) -> Result<Manifest> {
        // The schema first, so an older manifest reports its version rather
        // than a missing field.
        #[derive(Deserialize)]
        struct Schema {
            schema: u32,
        }
        let v: Schema =
            serde_json::from_slice(bytes).map_err(|e| Error::format(NAME, format!("not a manifest: {e}")))?;
        if v.schema != SCHEMA {
            return Err(Error::version(NAME, format!("manifest schema {}; known: {SCHEMA}", v.schema)));
        }
        let j: InstallerJson = serde_json::from_slice(bytes)
            .map_err(|e| Error::format(NAME, format!("not a schema-{SCHEMA} manifest: {e}")))?;
        if j.kind != "aurora-audio-tuning" {
            return Err(Error::format(NAME, format!("kind {:?}", j.kind)));
        }
        let _ = j.based_on;
        // Completeness: the installer writes only a full set, and says so.
        if !j.complete {
            return Err(Error::format(NAME, "the installer marked the set incomplete"));
        }
        let config = format!("Config/{}.plist", j.aid);
        let per_aid = aid_dsp_database(&j.aid);
        if (j.device_dsp_database != DSP_DATABASE && j.device_dsp_database != per_aid) || j.config != config {
            return Err(Error::format(
                NAME,
                format!(
                    "selection files {:?} and {:?}; expected {DSP_DATABASE:?} or {per_aid:?}, and {config:?}",
                    j.device_dsp_database, j.config
                ),
            ));
        }
        for name in [j.device_dsp_database.as_str(), config.as_str()] {
            if !j.files.iter().any(|r| r.path == name) {
                return Err(Error::format(NAME, format!("{name} is named but not listed")));
            }
        }
        let prefix = format!("{}/", j.aid);
        let folder = j.files.iter().filter(|r| r.path.starts_with(&prefix)).count() as u64;
        if j.source_file_count == 0 || j.source_file_count != folder {
            return Err(Error::format(
                NAME,
                format!("source_file_count {} but {folder} {prefix} entries are listed", j.source_file_count),
            ));
        }
        let m = Manifest {
            source: Source::Installer,
            acoustic_id: j.acoustic_id,
            aid: j.aid,
            macos_build: (j.macos_build != "unknown").then_some(j.macos_build),
            machine_model: None,
            speaker_id: j.speaker_id,
            dsp_database: Some(j.device_dsp_database),
            speaker_thiele_small: j.speaker_thiele_small,
            files: j.files,
        };
        m.check(NAME)?;
        Ok(m)
    }

    /// Parses aurora-audio-dsp's `metadata.json` (collector).
    pub fn parse_collector(bytes: &[u8]) -> Result<Manifest> {
        let j: CollectorJson = serde_json::from_slice(bytes)
            .map_err(|e| Error::format(COLLECTOR_NAME, format!("not a collector metadata file: {e}")))?;
        if j.kind != "aurora-audio-dsp tuning bundle" {
            return Err(Error::format(COLLECTOR_NAME, format!("kind {:?}", j.kind)));
        }
        if j.version != 1 {
            return Err(Error::version(COLLECTOR_NAME, format!("metadata version {}; known: 1", j.version)));
        }
        let acoustic_id = aid_number(&j.acoustic_id_name)
            .ok_or_else(|| Error::format(COLLECTOR_NAME, format!("acoustic_id_name {:?}", j.acoustic_id_name)))?;
        let (spts, files): (Vec<Record>, Vec<Record>) = j.files.into_iter().partition(|r| r.path == SPTS_NAME);
        let m = Manifest {
            source: Source::Collector,
            acoustic_id,
            aid: j.acoustic_id_name,
            macos_build: None,
            machine_model: j.machine_model,
            speaker_id: None,
            dsp_database: None,
            speaker_thiele_small: spts.into_iter().next(),
            files,
        };
        m.check(COLLECTOR_NAME)?;
        Ok(m)
    }

    fn check(&self, ctx: &str) -> Result<()> {
        if aid_number(&self.aid) != Some(self.acoustic_id) {
            return Err(Error::format(
                ctx,
                format!("aid {:?} does not match acoustic_id {}", self.aid, self.acoustic_id),
            ));
        }
        if self.files.is_empty() || self.files.len() > MAX_FILES {
            return Err(Error::format(ctx, format!("{} files (1..={MAX_FILES})", self.files.len())));
        }
        let mut total = 0u64;
        let mut seen = std::collections::BTreeSet::new();
        for r in self.records() {
            if !admitted(&r.path) {
                return Err(Error::format(ctx, format!("inadmissible path {:?}", r.path)));
            }
            if r.sha256.len() != 64 || !r.sha256.bytes().all(|c| matches!(c, b'0'..=b'9' | b'a'..=b'f')) {
                return Err(Error::format(ctx, format!("{}: sha256 is not 64 lowercase hex digits", r.path)));
            }
            if !seen.insert(r.path.as_str()) {
                return Err(Error::format(ctx, format!("{} listed twice", r.path)));
            }
            total = total.saturating_add(r.bytes);
        }
        if total > MAX_BYTES {
            return Err(Error::format(ctx, format!("{total} bytes exceeds {MAX_BYTES}")));
        }
        let config = format!("Config/{}.plist", self.aid);
        for r in &self.files {
            let extra = r.path == DSP_DATABASE || r.path == config;
            if !extra && !r.path.starts_with(&format!("{}/", self.aid)) {
                return Err(Error::format(ctx, format!("{} is outside {}/", r.path, self.aid)));
            }
        }
        if let Some(s) = &self.speaker_thiele_small {
            if s.path != SPTS_NAME {
                return Err(Error::format(ctx, format!("calibration record path {:?}", s.path)));
            }
        }
        Ok(())
    }
}

fn aid_number(aid: &str) -> Option<u32> {
    let n = aid.strip_prefix("AID")?;
    if n.is_empty() || n.starts_with('0') || !n.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    n.parse().ok()
}

/// A manifest path is relative, with no empty, `.` or `..` component and
/// no AppleDouble or Finder sidecar.
pub fn admitted(path: &str) -> bool {
    !path.is_empty()
        && !path.starts_with('/')
        && !path.contains('\\')
        && !path.contains('\0')
        && path.split('/').all(|p| !p.is_empty() && p != "." && p != ".." && !p.starts_with("._") && p != ".DS_Store")
}

pub fn sha256_hex(data: &[u8]) -> String {
    Sha256::digest(data).iter().map(|b| format!("{b:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_known_vectors() {
        assert_eq!(sha256_hex(b""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
        assert_eq!(sha256_hex(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    }

    #[test]
    fn path_admission() {
        assert!(admitted("AID35/DSP/Graphs/builtin_speaker_out.dspg"));
        assert!(!admitted("../x"));
        assert!(!admitted("/abs"));
        assert!(!admitted("AID35//x"));
        assert!(!admitted("AID35/._x"));
        assert!(!admitted("AID35/.DS_Store"));
    }

    #[test]
    fn record_verification() {
        let rec = Record { path: "a".into(), bytes: 3, sha256: sha256_hex(b"abc") };
        assert!(rec.verify(b"abc").is_ok());
        assert!(rec.verify(b"abd").is_err());
        assert!(rec.verify(b"ab").is_err());
    }

    fn selection() -> String {
        let sha = sha256_hex(b"x");
        format!(
            r#"{{"path":"DeviceDSPDatabase.plist","bytes":1,"sha256":"{sha}"}},{{"path":"Config/AID7.plist","bytes":1,"sha256":"{sha}"}}"#
        )
    }

    fn installer_with(files: &str, count: u64, extra: &str) -> String {
        let sel = selection();
        let files = if files.is_empty() { sel } else { format!("{files},{sel}") };
        format!(
            r#"{{"schema":2,"kind":"aurora-audio-tuning","source_file_count":{count},"complete":true,
               "acoustic_id":7,"aid":"AID7","macos_build":"25A1","speaker_thiele_small":null,
               "device_dsp_database":"DeviceDSPDatabase.plist","config":"Config/AID7.plist",
               "files":[{files}],"based_on":"x"{extra}}}"#
        )
    }

    fn installer(files: &str) -> String {
        installer_with(files, 1, "")
    }

    #[test]
    fn installer_manifest() {
        let sha = sha256_hex(b"x");
        let m = Manifest::parse(installer(&format!(r#"{{"path":"AID7/a","bytes":1,"sha256":"{sha}"}}"#)).as_bytes())
            .unwrap();
        assert_eq!(m.aid, "AID7");
        assert_eq!(m.macos_build.as_deref(), Some("25A1"));
        // Outside the AID folder, traversal, duplicates, bad hash.
        for bad in [
            format!(r#"{{"path":"AID8/a","bytes":1,"sha256":"{sha}"}}"#),
            format!(r#"{{"path":"AID7/../a","bytes":1,"sha256":"{sha}"}}"#),
            format!(r#"{{"path":"AID7/a","bytes":1,"sha256":"{sha}"}},{{"path":"AID7/a","bytes":1,"sha256":"{sha}"}}"#),
            r#"{"path":"AID7/a","bytes":1,"sha256":"XYZ"}"#.to_string(),
        ] {
            assert!(Manifest::parse(installer(&bad).as_bytes()).is_err(), "{bad}");
        }
        let wrong_schema = installer("").replace("\"schema\":2", "\"schema\":1");
        assert!(matches!(Manifest::parse(wrong_schema.as_bytes()), Err(Error::Version { .. })));
    }

    #[test]
    fn installer_manifest_completeness() {
        let sha = sha256_hex(b"x");
        let one = format!(r#"{{"path":"AID7/a","bytes":1,"sha256":"{sha}"}}"#);
        assert!(Manifest::parse(installer(&one).as_bytes()).is_ok());
        // A short or inflated count, an incomplete flag, missing selection.
        for bad in [
            installer_with(&one, 2, ""),
            installer_with(&one, 0, ""),
            installer(&one).replace("\"complete\":true", "\"complete\":false"),
            installer(&one).replace(",\"complete\":true", ""),
            installer(&one).replace(r#","config":"Config/AID7.plist""#, r#","config":"Config/AID8.plist""#),
            installer(&one).replace(r#",{"path":"Config/AID7.plist""#, r#",{"path":"AID7/Config.plist""#),
        ] {
            assert!(Manifest::parse(bad.as_bytes()).is_err(), "{bad}");
        }
    }

    #[test]
    fn installer_manifest_speaker_id() {
        let sha = sha256_hex(b"x");
        let one = format!(r#"{{"path":"AID7/a","bytes":1,"sha256":"{sha}"}}"#);
        // Swift omits it, Python writes null.
        assert_eq!(Manifest::parse(installer(&one).as_bytes()).unwrap().speaker_id, None);
        let m = Manifest::parse(installer_with(&one, 1, r#","speaker_id":null"#).as_bytes()).unwrap();
        assert_eq!(m.speaker_id, None);
        for (v, want) in [("3", 3u32), ("65535", 65535), ("4294967295", u32::MAX)] {
            let m = Manifest::parse(installer_with(&one, 1, &format!(r#","speaker_id":{v}"#)).as_bytes()).unwrap();
            assert_eq!(m.speaker_id, Some(want));
        }
        for v in ["-1", "4294967296", "3.0", "true", "\"3\"", "[3]"] {
            let j = installer_with(&one, 1, &format!(r#","speaker_id":{v}"#));
            assert!(Manifest::parse(j.as_bytes()).is_err(), "{v}");
        }
    }

    #[test]
    fn collector_metadata() {
        let sha = sha256_hex(b"x");
        let j = format!(
            r#"{{"kind":"aurora-audio-dsp tuning bundle","version":1,"machine_model":"MacTest1,1","acoustic_id_name":"AID7",
               "files":[{{"path":"speaker-thiele-small.bin","bytes":1,"sha256":"{sha}"}},{{"path":"AID7/a","bytes":1,"sha256":"{sha}"}}]}}"#
        );
        let m = Manifest::parse_collector(j.as_bytes()).unwrap();
        assert_eq!(m.acoustic_id, 7);
        assert!(m.speaker_thiele_small.is_some());
        assert_eq!(m.files.len(), 1);
    }
}
