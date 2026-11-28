// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Non-realtime startup: compile verified tuning, check the exact self-check
//! binding, then publish. No engine page or device may be opened before this
//! returns successfully. Kept private to the binary; tests inject every host
//! dependency, with no production fixture-root or precompiled-blob bypass.

use std::path::{Path, PathBuf};

use aurora_speaker_tuning::{
    bundle::Bundle,
    compile::{self, PreStageMode, Provenance},
    machine::Machine as CompileMachine,
    manifest::sha256_hex,
    tuning::{Options as TuningOptions, Tuning},
    Diagnostics,
};
use sgv2_core::blob::Header;
use sgv2_supervisor::{
    machine::{blob_full_scales, Machine},
    selfcheck,
};

pub const USAGE: &str =
    "usage: speakerguardd [--card N|ID] --pcm D --uid UID [--bundle PATH] [--voicing-pre] [--selfcheck RECORD]";

#[derive(Debug, PartialEq, Eq)]
pub struct Options {
    pub pcm: u32,
    pub uid: u32,
    pub card: Option<String>,
    pub bundle: Option<PathBuf>,
    pub voicing_pre: bool,
    pub selfcheck: Option<String>,
}

impl Options {
    pub fn parse(args: impl IntoIterator<Item = String>) -> Result<Self, String> {
        let (mut pcm, mut uid, mut card, mut bundle, mut selfcheck, mut voicing_pre) =
            (None, None, None, None, None, false);
        let mut args = args.into_iter();
        while let Some(a) = args.next() {
            match a.as_str() {
                "--voicing-pre" if !voicing_pre => voicing_pre = true,
                "--pcm" if pcm.is_none() => pcm = Some(number(&mut args, &a)?),
                "--uid" if uid.is_none() => uid = Some(number(&mut args, &a)?),
                "--card" if card.is_none() => card = Some(value(&mut args, &a)?),
                "--bundle" if bundle.is_none() => bundle = Some(PathBuf::from(value(&mut args, &a)?)),
                "--selfcheck" if selfcheck.is_none() => selfcheck = Some(value(&mut args, &a)?),
                _ => return Err(format!("unknown or repeated argument {a}")),
            }
        }
        Ok(Self {
            pcm: pcm.ok_or("missing --pcm")?,
            uid: uid.ok_or("missing --uid")?,
            card,
            bundle,
            voicing_pre,
            selfcheck,
        })
    }
}

fn value(args: &mut impl Iterator<Item = String>, name: &str) -> Result<String, String> {
    args.next().filter(|s| !s.is_empty() && !s.starts_with('-')).ok_or_else(|| format!("missing value for {name}"))
}

fn number(args: &mut impl Iterator<Item = String>, name: &str) -> Result<u32, String> {
    value(args, name)?.parse().map_err(|_| format!("bad {name}"))
}

pub trait Environment {
    fn identity(&mut self) -> Result<(CompileMachine, Option<u32>), String>;
    fn bundle(&mut self, path: Option<&Path>) -> aurora_speaker_tuning::Result<Bundle>;
    fn selfcheck(&mut self) -> Result<selfcheck::Bind, String>;
    fn kernel_release(&mut self) -> Result<String, String>;
    fn publish(&mut self, blob: &[u8]) -> Result<(), String>;
}

#[derive(Debug)]
pub enum Error {
    Config(String),
    Unavailable(String),
}

pub struct Prepared {
    pub board: String,
    pub bind: selfcheck::Bind,
    pub machine: &'static Machine,
    pub header: Header,
    pub crossover: bool,
    pub full_scales: Vec<u32>,
    pub engine_crc: u32,
}

pub fn start(env: &mut impl Environment, path: Option<&Path>, voicing_pre: bool) -> Result<Prepared, Error> {
    use Error::{Config, Unavailable};
    let (identity, speaker_id) = env.identity().map_err(Config)?;
    let board = &identity.board;
    let bind = env.selfcheck().map_err(Config)?;
    let machine = selfcheck::machine_for(board, &bind).map_err(Config)?;
    // Bundle I/O is eligible for a later systemd start, including missing
    // sources; it need not be transient and start limits still apply.
    // Other loader errors remain configuration refusals.
    let bundle = env.bundle(path).map_err(|e| match e {
        aurora_speaker_tuning::Error::Io { .. } => Unavailable(e.to_string()),
        _ => Config(e.to_string()),
    })?;
    let mut diags = Diagnostics::default();
    let compiled = (|| {
        let tuning = Tuning::load(
            &bundle,
            &TuningOptions { speaker_id: None, device_tree_speaker_id: speaker_id, buzzkill_matrix: None },
            &mut diags,
        )?;
        // Exactly the deployment compiler choices: identity pre-stage by
        // default; full voicing only when provisioning explicitly selected it.
        // Provenance and unit binding are the compiler's, without an escape.
        if voicing_pre {
            compile::compile_voicing_with(&tuning, &identity, Provenance::Verified, &mut diags)
        } else {
            compile::compile_opts(&tuning, &identity, Provenance::Verified, PreStageMode::Identity, &mut diags)
        }
    })();
    for diagnostic in &diags.items {
        eprintln!("{diagnostic}");
    }
    let blob = compiled.map_err(|e| Config(e.to_string()))?.blob;
    let (header, _) = Header::parse(&blob).map_err(|e| Config(format!("not a valid parameter blob ({e:?})")))?;
    if header.machine() != Some(board.as_str()) {
        return Err(Config(format!("compiled for {:?}, this Mac is {board}", header.machine())));
    }
    if header.channels as usize != machine.amps {
        return Err(Config(format!("{} channels, {board} has {} amps", header.channels, machine.amps)));
    }
    if header.full_scale_mv == 0 {
        return Err(Config("states no operating point".into()));
    }
    if header.payload_crc == 0 {
        return Err(Config("payload CRC 0".into()));
    }
    if !(8_000..=192_000).contains(&header.sample_rate) {
        return Err(Config(format!("sample rate {}", header.sample_rate)));
    }
    sgv2_core::blob::voicing(&blob).map_err(|e| Config(format!("voicing rejected ({e:?})")))?;
    let crossover = sgv2_core::blob::prestage(&blob, header.sample_rate as f64)
        .map_err(|e| Config(format!("pre-stage rejected ({e:?})")))?
        .from_tuning();
    let full_scales = blob_full_scales(&blob).map_err(Config)?;
    let engine_crc = match sgv2_core::blob::engine_crc(&blob) {
        Ok(crc) if crc != 0 => crc,
        _ => return Err(Config("invalid deployment identity".into())),
    };
    let params_sha256 = sha256_hex(&blob);
    let kernel_rel = env.kernel_release().map_err(Unavailable)?;
    selfcheck::covers(&bind, board, &kernel_rel, &params_sha256).map_err(Config)?;
    eprintln!(
        "self-check: {} wiring {:016x} ({}), kernel {}, params {}",
        board,
        bind.wiring,
        bind.method,
        bind.kernel,
        &params_sha256[..12]
    );
    env.publish(&blob).map_err(Unavailable)?;
    Ok(Prepared { board: identity.board, bind, machine, header, crossover, full_scales, engine_crc })
}

/// The existing publication contract: prepare the user directory, write a
/// sibling temporary file, and rename the complete checked bytes into place.
pub fn publish_atomic(run_dir: &Path, user_dir: &Path, blob: &[u8]) -> std::io::Result<()> {
    std::fs::create_dir_all(user_dir)?;
    let tmp = run_dir.join(".params.bin.new");
    std::fs::write(&tmp, blob)?;
    std::fs::rename(&tmp, run_dir.join("params.bin"))
}

#[cfg(test)]
mod tests;
