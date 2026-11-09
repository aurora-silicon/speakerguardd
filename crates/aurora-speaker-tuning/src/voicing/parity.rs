// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The voicing chain's offline self-check: what `speakerguardd-selfcheck
//! verify` runs before v2 may take a Mac's speakers.
//!
//! Three parts, all offline and silent:
//!
//! * **Compiler gaps.** A chain with a stub, an unwired graph parameter or a
//!   box the compiler itself carries only in part ([`super::chain::Compiled::gaps`]:
//!   "parity pending") is refused. Nothing is guessed.
//! * **Render invariants.** The compiled chain renders deterministic test
//!   signals here, on this Mac, through the same engine the graph runs: no
//!   non-finite sample, at the clip edge included; silence renders as
//!   silence (no idle tone, no limit cycle); a lane that ends in the
//!   limiter box never passes the limiter's own saved ceiling; two runs from
//!   reset are identical. How far other host block sizes move the render is
//!   reported (the graph locks the 512-frame quantum the gate measures).
//! * **macOS parity attestation.** The only true reference for Apple's
//!   nonlinear boxes is Apple's own Audio Units, which run on macOS, not on
//!   the Mac being checked. The gate (`tools/diffrig/aidgate`) renders every
//!   AID of a macOS release through that release's installed units and
//!   through this code, automatically, for all of them. Its result is an
//!   attestation per tuning [`key`] (a hash of what the voicing compiler
//!   reads, nothing else), bound to [`LAW`] (a hash of the rendering code
//!   this binary was built from). A key with no attestation, an attestation
//!   for other rendering code, or a failing one refuses. The file holds
//!   hashes, verdicts and summary numbers only: no Apple data.
//!
//! The attestation does not choose boards: it records, for every tuning of
//! the release, whether this code renders it like macOS. A Mac nobody here
//! has seen is checked exactly like the lab's.

use super::chain::{self, Compiled};
use crate::error::{Error, Result};
use crate::tuning::Tuning;
use crate::Diagnostics;
use serde::{Deserialize, Serialize};
use sgv2_core::voicing::chain::{Chain, GraphParam, Kind, Source};

/// Hash of the rendering code this binary was built from (see `build.rs`).
pub const LAW: &str = env!("SGV2_VOICING_LAW");
/// The attestation format this binary reads.
pub const FORMAT: u32 = 1;
const FS: f64 = 48_000.0;

fn bad(msg: impl Into<String>) -> Error {
    Error::format("voicing parity", msg.into())
}

/// The tuning key: a SHA-256 over what the voicing compiler reads from
/// `t` (the chain, every box's saved state and unit dictionaries, the
/// graph's plan and parameter wiring, the selection, the volume range and
/// which macOS generation the units follow), and nothing per unit or per
/// collection (board, unit ID, calibration, the build string itself).
/// The same Apple tuning gives the same key whether it was read from a
/// macOS Tunings folder or from a Mac's installer bundle.
pub fn key(t: &Tuning) -> Result<String> {
    let mut v = serde_json::to_value(t).map_err(|e| bad(e.to_string()))?;
    let o = v.as_object_mut().ok_or_else(|| bad("tuning is not an object"))?;
    for k in [
        "macos_build",
        "board",
        "unit_id",
        "verified",
        "speaker_id",
        "speaker_id_source",
        "refusals",
        "protection",
        "spts",
        "mask_lattice_present",
    ] {
        o.remove(k);
    }
    // The selection's names (configuration, file paths, speaker ID) are left
    // out: two speaker IDs whose strips hold the same states render alike
    // and get one key. What the selection substitutes into the graph stays.
    if let Some(sel) = o.get_mut("selection").and_then(|s| s.as_object_mut()) {
        sel.retain(|k, _| k == "substitutions" || k == "sample_rate");
    }
    o.insert("macos27".into(), serde_json::Value::Bool(super::cfreak::macos27(t.macos_build.as_deref())));
    let extra: Vec<String> = t
        .chain
        .iter()
        .map(|s| {
            format!("{}\0{}", s.name, crate::plist::xml::write(&crate::plist::Value::Dict(s.preset_extra.clone())))
        })
        .collect();
    o.insert("preset_extra".into(), serde_json::Value::from(extra));
    let plan = super::topology::plan(t)?;
    o.insert("plan".into(), serde_json::to_value(&plan).map_err(|e| bad(e.to_string()))?);
    let text = format!("sgv2 voicing key v1\n{}", serde_json::to_string(&v).map_err(|e| bad(e.to_string()))?);
    Ok(crate::manifest::sha256_hex(text.as_bytes()))
}

// ---------------------------------------------------------------- attestation

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct Tolerance {
    /// A lane louder than macOS by more than this (dB), in any measured
    /// cell, fails.
    pub louder_db: f64,
    /// A lane quieter than macOS by more than this (dB, positive) fails.
    pub quieter_db: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Entry {
    pub key: String,
    /// Informational: where the gate found the tuning.
    #[serde(default)]
    pub aid: Option<u32>,
    #[serde(default)]
    pub speaker_id: Option<i64>,
    /// "pass" or "refused".
    pub verdict: String,
    #[serde(default)]
    pub reason: String,
    #[serde(default)]
    pub worst_louder_db: Option<f64>,
    #[serde(default)]
    pub worst_quieter_db: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Attestations {
    pub format: u32,
    /// The [`LAW`] of the code the gate rendered with.
    pub law: String,
    /// The macOS build whose Audio Units were the reference.
    pub reference: String,
    pub tolerance: Tolerance,
    pub entries: Vec<Entry>,
}

impl Attestations {
    pub fn parse(bytes: &[u8]) -> Result<Attestations> {
        let a: Attestations = serde_json::from_slice(bytes).map_err(|e| bad(format!("attestations: {e}")))?;
        if a.format != FORMAT {
            return Err(bad(format!("attestations format {} (this build reads {FORMAT})", a.format)));
        }
        Ok(a)
    }
}

/// The attestation's answer for `key`: Ok(entry) when it passes for this
/// code, else the refusal.
pub fn attested<'a>(a: &'a Attestations, key: &str) -> std::result::Result<&'a Entry, String> {
    if a.law != LAW {
        return Err(format!(
            "the macOS parity attestations are for other rendering code (law {} vs this build's {}): re-run the gate",
            short(&a.law),
            short(LAW)
        ));
    }
    let e = a.entries.iter().find(|e| e.key == key).ok_or_else(|| {
        format!("no macOS parity attestation for this tuning (key {}) against {}", short(key), a.reference)
    })?;
    if e.verdict != "pass" {
        return Err(format!("macOS parity gate ({}) refused this tuning: {}", a.reference, e.reason));
    }
    Ok(e)
}

fn short(s: &str) -> &str {
    &s[..s.len().min(12)]
}

// ---------------------------------------------------------------- invariants

#[derive(Clone, Debug, Serialize)]
pub struct Check {
    pub name: String,
    /// "pass", "refused" or "info" (reported, never refuses).
    pub result: &'static str,
    pub detail: String,
}

fn check(name: &str, ok: bool, detail: String) -> Check {
    Check { name: name.into(), result: if ok { "pass" } else { "refused" }, detail }
}

/// Compiler gaps, stubs and unwired parameters: the compiler's own word
/// that the chain is not at parity.
pub fn gaps(c: &Compiled) -> Check {
    let mut why: Vec<String> = c.stubs.iter().map(|(n, s)| format!("{n} ({s}): no box module")).collect();
    why.extend(c.unwired.iter().map(|u| format!("graph parameter {u} is not carried")));
    why.extend(c.gaps.iter().map(|(n, g)| format!("{n}: {g}")));
    check("compiler-gaps", why.is_empty(), if why.is_empty() { "none".into() } else { why.join("; ") })
}

/// A stereo program: left, right.
type Stereo = (Vec<f32>, Vec<f32>);

/// A deterministic stereo test signal.
fn signal(kind: &str, f: f64, db: f64, secs: f64) -> Stereo {
    let n = (secs * FS) as usize;
    let a = 10f64.powf(db / 20.0);
    let mut l = vec![0f32; n];
    match kind {
        "silence" => {}
        "sine" => {
            for (i, x) in l.iter_mut().enumerate() {
                *x = (a * (2.0 * std::f64::consts::PI * f * i as f64 / FS).sin()) as f32;
            }
        }
        // a sine `db` over full scale, clipped at full scale: the clip edge
        "clip" => {
            for (i, x) in l.iter_mut().enumerate() {
                *x = (a * (2.0 * std::f64::consts::PI * f * i as f64 / FS).sin()).clamp(-1.0, 1.0) as f32;
            }
        }
        // log sweep 20 Hz - 20 kHz
        "sweep" => {
            let (f0, f1) = (20.0f64, 20_000.0f64);
            let k = (f1 / f0).ln() / secs;
            for (i, x) in l.iter_mut().enumerate() {
                let t = i as f64 / FS;
                *x = (a * (2.0 * std::f64::consts::PI * f0 * ((k * t).exp() - 1.0) / k).sin()) as f32;
            }
        }
        // deterministic white noise (xorshift), peak `a`
        "noise" => {
            let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
            for x in l.iter_mut() {
                s ^= s << 13;
                s ^= s >> 7;
                s ^= s << 17;
                *x = (a * ((s >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0)) as f32;
            }
        }
        _ => unreachable!(),
    }
    let r = l.clone();
    (l, r)
}

/// Renders `prog` (stereo) through a fresh chain from `spec`, host block
/// `block`, at full system volume. Returns lanes x frames.
fn render(c: &Compiled, prog: &Stereo, block: usize) -> Result<Vec<Vec<f32>>> {
    let mut chain = Chain::new(c.spec.clone(), FS, block).ok_or_else(|| bad("the engine refuses the chain"))?;
    chain.set_param(GraphParam::VolumeScalar, 1.0);
    chain.set_param(GraphParam::VolumeDb, 0.0);
    let pc = c.spec.program_channels;
    let lanes = c.spec.outputs.len();
    let frames = prog.0.len();
    let mut out = vec![vec![0f32; frames]; lanes];
    let mut buf = vec![vec![0f32; block]; pc];
    let mut ob = vec![vec![0f32; block]; lanes];
    let mut f = 0;
    while f < frames {
        let n = block.min(frames - f);
        for (ch, b) in buf.iter_mut().enumerate() {
            let src = if ch % 2 == 0 { &prog.0 } else { &prog.1 };
            b[..n].copy_from_slice(&src[f..f + n]);
        }
        let pr: Vec<&[f32]> = buf.iter().map(|v| &v[..n]).collect();
        let mut or: Vec<&mut [f32]> = ob.iter_mut().map(|v| &mut v[..n]).collect();
        chain.process(&pr, &mut or, n);
        for (o, b) in out.iter_mut().zip(&ob) {
            o[f..f + n].copy_from_slice(&b[..n]);
        }
        f += n;
    }
    Ok(out)
}

fn db(x: f64) -> f64 {
    20.0 * x.max(1e-15).log10()
}

fn rms(x: &[f32]) -> f64 {
    if x.is_empty() {
        return 0.0;
    }
    (x.iter().map(|v| (*v as f64) * (*v as f64)).sum::<f64>() / x.len() as f64).sqrt()
}

fn peak(x: &[f32]) -> f64 {
    x.iter().fold(0f64, |m, v| m.max((*v as f64).abs()))
}

/// Silence floor: an idle chain puts out nothing above this (dBFS).
pub const SILENCE_DBFS: f64 = -120.0;
/// A lane ending in the limiter box may pass its saved ceiling by at most
/// this (dB): inter-block rounding, never a transient.
pub const CEILING_MARGIN_DB: f64 = 0.1;
/// Reported (not refused): a lane rendered at another host block size
/// louder than the 512-frame render by more than this (dB, per 10 ms window
/// above -60 dBFS). The graph forces and locks a 512-frame quantum, the
/// size macOS renders the same units at and the gate measures; several
/// Apple units meter per render call, so other sizes are not parity.
pub const BLOCK_DB: f64 = 0.25;
/// Host block sizes PipeWire may run the graph at.
pub const BLOCKS: [usize; 3] = [128, 256, 1024];
/// Lanes more than this far under the loudest lane are a crossover stop
/// band and are not measured (as in the gate).
const STOPBAND_DB: f64 = 60.0;

/// The render invariants on `c`.
pub fn invariants(c: &Compiled) -> Result<Vec<Check>> {
    let mut out = Vec::new();
    let lanes = c.spec.outputs.len();
    let progs: Vec<(String, Stereo)> = vec![
        ("silence".into(), signal("silence", 0.0, 0.0, 1.0)),
        ("sweep_0".into(), signal("sweep", 0.0, 0.0, 4.0)),
        ("noise_0".into(), signal("noise", 0.0, 0.0, 2.0)),
        ("clip_100_+6".into(), signal("clip", 100.0, 6.0, 1.0)),
        ("clip_1000_+12".into(), signal("clip", 1000.0, 12.0, 1.0)),
    ]
    .into_iter()
    .chain(
        [40.0, 100.0, 250.0, 1000.0, 6300.0]
            .iter()
            .flat_map(|f| [0.0, -12.0].iter().map(move |l| (format!("tone_{f}_{l}"), signal("sine", *f, *l, 2.0)))),
    )
    .collect();

    // 1. finite + 2. silence
    let mut nonfinite = Vec::new();
    let mut silence_worst = f64::NEG_INFINITY;
    let mut renders = Vec::new();
    for (name, p) in &progs {
        let r = render(c, p, 512)?;
        if r.iter().any(|l| l.iter().any(|v| !v.is_finite())) {
            nonfinite.push(name.clone());
        }
        if name == "silence" {
            silence_worst = r.iter().map(|l| db(peak(l))).fold(f64::NEG_INFINITY, f64::max);
        }
        renders.push(r);
    }
    out.push(check(
        "finite",
        nonfinite.is_empty(),
        if nonfinite.is_empty() { "every sample finite".into() } else { format!("non-finite output on {nonfinite:?}") },
    ));
    out.push(check(
        "silence",
        silence_worst <= SILENCE_DBFS,
        format!("idle output peak {silence_worst:.1} dBFS (limit {SILENCE_DBFS})"),
    ));

    // 4. the limiter's own saved ceiling, on lanes the limiter ends
    let mut ceil = Vec::new();
    let mut ceil_lanes = 0;
    for lane in 0..lanes {
        let Source::Node(n, _) = c.spec.outputs[lane] else { continue };
        let Kind::Cfreak(p) = &c.spec.nodes[n].kind else { continue };
        if p.saturation_flavor != 0 {
            continue;
        }
        ceil_lanes += 1;
        let limit = p.threshold_db + p.output_gain_db;
        for ((name, _), r) in progs.iter().zip(&renders) {
            let pk = db(peak(&r[lane]));
            if pk > limit + CEILING_MARGIN_DB {
                ceil.push(format!("{name} lane {lane}: peak {pk:.2} dBFS over its ceiling {limit:.2}"));
            }
        }
    }
    out.push(check(
        "limiter-ceiling",
        ceil.is_empty(),
        if ceil.is_empty() {
            format!("{ceil_lanes} limiter lane(s) within their saved ceiling")
        } else {
            ceil.join(", ")
        },
    ));

    // 5. host block size independence; 6. reset determinism
    let mut blk = Vec::new();
    let mut det = Vec::new();
    for (i, (name, p)) in progs.iter().enumerate() {
        if !(name.starts_with("sweep") || name.starts_with("noise") || name.starts_with("clip")) {
            continue;
        }
        let base = &renders[i];
        let again = render(c, p, 512)?;
        if &again != base {
            det.push(name.clone());
        }
        let top = base.iter().map(|l| peak(l)).fold(0.0, f64::max);
        for b in BLOCKS {
            let r = render(c, p, b)?;
            let mut worst = 0f64;
            for (x, y) in base.iter().zip(&r) {
                for (wx, wy) in x.chunks_exact(480).zip(y.chunks_exact(480)) {
                    let (rx, ry) = (rms(wx), rms(wy));
                    if db(rx.max(ry)) < db(top) - STOPBAND_DB || db(rx.max(ry)) < -60.0 {
                        continue;
                    }
                    worst = worst.max(db(ry) - db(rx));
                }
            }
            if worst > BLOCK_DB {
                blk.push(format!("{name} at {b} frames: +{worst:.2} dB"));
            }
        }
    }
    out.push(Check {
        name: "block-size".into(),
        result: "info",
        detail: if blk.is_empty() {
            format!("blocks {BLOCKS:?} within {BLOCK_DB} dB of 512")
        } else {
            format!("the graph's locked 512-frame quantum is required: {}", blk.join(", "))
        },
    });
    out.push(check(
        "deterministic",
        det.is_empty(),
        if det.is_empty() { "identical from reset".into() } else { format!("differs between runs: {det:?}") },
    ));
    Ok(out)
}

/// The whole voicing check for `t`: `(key, compiled-or-error, checks)`.
/// `attest` is the attestation file's content, if one was found.
pub fn run(t: &Tuning, attest: Option<&[u8]>, diags: &mut Diagnostics) -> (Option<String>, Vec<Check>) {
    let mut checks = Vec::new();
    let k = match key(t) {
        Ok(k) => Some(k),
        Err(e) => {
            checks.push(check("key", false, e.to_string()));
            None
        }
    };
    let c = match chain::compile(t, FS, diags) {
        Ok(c) => c,
        Err(e) => {
            checks.push(check("compile", false, e.to_string()));
            return (k, checks);
        }
    };
    checks.push(gaps(&c));
    match invariants(&c) {
        Ok(v) => checks.extend(v),
        Err(e) => checks.push(check("render", false, e.to_string())),
    }
    let parity = match (attest, &k) {
        (None, _) => Err("no macOS parity attestation file".to_string()),
        (_, None) => Err("no tuning key".to_string()),
        (Some(b), Some(k)) => match Attestations::parse(b) {
            Err(e) => Err(e.to_string()),
            Ok(a) => attested(&a, k).map(|e| {
                format!(
                    "{} (AID{}): worst louder {:+.3} dB, quieter {:+.3} dB vs {}",
                    short(k),
                    e.aid.map_or("?".into(), |v| v.to_string()),
                    e.worst_louder_db.unwrap_or(0.0),
                    e.worst_quieter_db.unwrap_or(0.0),
                    a.reference
                )
            }),
        },
    };
    checks.push(match parity {
        Ok(d) => check("macos-parity", true, d),
        Err(d) => check("macos-parity", false, d),
    });
    (k, checks)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn att(law: &str, verdict: &str) -> Attestations {
        Attestations {
            format: FORMAT,
            law: law.into(),
            reference: "test".into(),
            tolerance: Tolerance { louder_db: 0.25, quieter_db: 1.0 },
            entries: vec![Entry {
                key: "k1".into(),
                aid: Some(1),
                speaker_id: None,
                verdict: verdict.into(),
                reason: "lane 0 +4.35 dB louder than macOS (step_up_125)".into(),
                worst_louder_db: Some(4.35),
                worst_quieter_db: None,
            }],
        }
    }

    #[test]
    fn attestation_fails_closed() {
        assert!(attested(&att(LAW, "pass"), "k1").is_ok());
        // unknown tuning
        assert!(attested(&att(LAW, "pass"), "k2").unwrap_err().contains("no macOS parity attestation"));
        // a j414s-like refusal
        assert!(attested(&att(LAW, "refused"), "k1").unwrap_err().contains("+4.35 dB louder"));
        // stale: the rendering code changed after the gate ran
        assert!(attested(&att("0000", "pass"), "k1").unwrap_err().contains("other rendering code"));
    }

    #[test]
    fn attestation_format_is_checked() {
        let mut a = att(LAW, "pass");
        a.format = FORMAT + 1;
        assert!(Attestations::parse(&serde_json::to_vec(&a).unwrap()).is_err());
        a.format = FORMAT;
        assert!(Attestations::parse(&serde_json::to_vec(&a).unwrap()).is_ok());
        assert!(Attestations::parse(b"{").is_err());
    }

    #[test]
    fn law_is_a_hash() {
        assert_eq!(LAW.len(), 64);
        assert!(LAW.bytes().all(|b| b.is_ascii_hexdigit()));
    }

    #[test]
    fn signals_are_deterministic() {
        assert_eq!(signal("noise", 0.0, 0.0, 0.1), signal("noise", 0.0, 0.0, 0.1));
        let (l, r) = signal("clip", 100.0, 6.0, 0.1);
        assert_eq!(l, r);
        assert!(l.iter().all(|v| v.abs() <= 1.0));
        assert!(peak(&signal("silence", 0.0, 0.0, 0.1).0) == 0.0);
    }
}
