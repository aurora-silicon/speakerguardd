// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Choosing the speaker graph, strip and property strip, as the DSP
//! database does on macOS.
//!
//! The database (`DeviceDSPDatabase.plist` on macOS 26 and earlier,
//! `AID<n>/DSP/DSPDatabase.plist` on macOS 27 and in aurora-audio-dsp
//! collector bundles; key `AID<n>` in both; an installer manifest names
//! which)
//! maps each device (`Built-in Speaker`) and configuration to
//! `Output[].Hardware[]` entries of type `DSPGraph` (with `Substitutions`),
//! `AUStrip` and `PropertyStrip`, plus `UnderlyingDeviceProperties`
//! selectors (`nsrt` sample rate, `spid` speaker ID) and
//! `DSPDeviceProperties` (`cdes` with a `kind`).
//!
//! The protection chain is the configuration whose `cdes` kind is that of
//! the database's own "General" configurations, for this machine's speaker
//! ID.  With no database the reader falls back to the conventional file
//! names, which cannot tell vendor strips apart; that is reported.

use crate::bundle::{Bundle, DSP_DATABASE};
use crate::error::{Diagnostics, Error, Result};
use crate::manifest::aid_dsp_database;
use crate::plist::{self, Dict, Value};
use serde::Serialize;
use std::collections::BTreeMap;

pub const SPEAKER_DEVICE: &str = "Built-in Speaker";
/// The `spid` value of the database's vendor-neutral entries.
pub const SPID_GENERIC: i128 = 65535;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Configuration {
    pub name: String,
    pub cdes_kind: Option<i128>,
    pub sample_rate: Option<i128>,
    pub speaker_id: Option<i128>,
    pub graph: Option<String>,
    pub substitutions: BTreeMap<String, String>,
    pub strip: Option<String>,
    pub propstrip: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Selection {
    /// `None` when chosen by file-name convention.
    pub configuration: Option<String>,
    pub graph: String,
    pub strip: String,
    pub propstrip: Option<String>,
    pub substitutions: BTreeMap<String, String>,
    pub sample_rate: Option<i128>,
    pub speaker_id: Option<i128>,
    /// Every speaker configuration in the database, for the dump.
    pub all: Vec<Configuration>,
}

/// The DSP database the manifest names; without one (collector bundles,
/// folders read in place), the shared one, else the per-AID one.
fn database(b: &Bundle, diags: &mut Diagnostics) -> Result<Option<Dict>> {
    let per_aid = aid_dsp_database(&b.manifest.aid);
    let (path, bytes) = match &b.manifest.dsp_database {
        Some(p) => (
            p.clone(),
            b.file(p).ok_or_else(|| Error::format(p, "the manifest names this DSP database; it is missing"))?,
        ),
        None => match (b.file(DSP_DATABASE), b.file(&per_aid)) {
            (Some(x), _) => (DSP_DATABASE.to_string(), x),
            (None, Some(x)) => (per_aid, x),
            (None, None) => return Ok(None),
        },
    };
    let (v, _) = plist::parse(bytes).map_err(|e| Error::plist(&path, e))?;
    let root = v.as_dict().ok_or_else(|| Error::format(&path, "root is not a dictionary"))?;
    match root.get(&b.manifest.aid) {
        Some(Value::Dict(d)) => Ok(Some(d.clone())),
        Some(_) => Err(Error::format(&path, format!("{} is not a dictionary", b.manifest.aid))),
        None => {
            diags.warn("select-db-no-aid", &path, format!("no {} entry", b.manifest.aid));
            Ok(None)
        }
    }
}

fn selector<'a>(list: Option<&'a Value>, sel: &str) -> Option<&'a Value> {
    list?
        .as_array()?
        .iter()
        .filter_map(Value::as_dict)
        .find(|d| d.get("Selector").and_then(Value::as_str) == Some(sel))?
        .get("Value")
}

fn configuration(name: &str, d: &Dict, aid: &str, ctx: &str) -> Result<Configuration> {
    let mut c = Configuration {
        name: name.to_string(),
        cdes_kind: selector(d.get("DSPDeviceProperties"), "cdes")
            .and_then(Value::as_dict)
            .and_then(|k| k.get("kind"))
            .and_then(Value::as_integer),
        sample_rate: selector(d.get("UnderlyingDeviceProperties"), "nsrt").and_then(Value::as_integer),
        speaker_id: selector(d.get("UnderlyingDeviceProperties"), "spid").and_then(Value::as_integer),
        graph: None,
        substitutions: BTreeMap::new(),
        strip: None,
        propstrip: None,
    };
    let outputs = d.get("Output").and_then(Value::as_array).unwrap_or(&[]);
    for o in outputs {
        for h in o.as_dict().and_then(|o| o.get("Hardware")).and_then(Value::as_array).unwrap_or(&[]) {
            let h =
                h.as_dict().ok_or_else(|| Error::format(ctx, format!("{name}: Hardware entry is not a dictionary")))?;
            let ty = h.get("Type").and_then(Value::as_str).unwrap_or("");
            let path = h.get("Path").and_then(Value::as_str).map(str::to_string);
            let slot = match ty {
                "DSPGraph" => &mut c.graph,
                "AUStrip" => &mut c.strip,
                "PropertyStrip" => &mut c.propstrip,
                _ => continue,
            };
            if slot.is_some() {
                return Err(Error::format(ctx, format!("{name}: more than one {ty}")));
            }
            *slot = path;
            if ty == "DSPGraph" {
                if let Some(Value::Dict(s)) = h.get("Substitutions") {
                    for (k, v) in s.iter() {
                        let v = match v {
                            Value::String(x) => x.clone(),
                            Value::Integer(i) => i.to_string(),
                            Value::Real(r) => r.to_string(),
                            o => return Err(Error::format(ctx, format!("{name}: substitution {k} is a {}", o.kind()))),
                        };
                        c.substitutions.insert(k.to_string(), v);
                    }
                }
            }
        }
    }
    for p in [&c.graph, &c.strip, &c.propstrip].into_iter().flatten() {
        if !p.starts_with(&format!("{aid}/")) {
            return Err(Error::format(ctx, format!("{name}: {p} is outside {aid}/")));
        }
    }
    Ok(c)
}

/// Chooses the speaker protection chain.  `speaker_id` is the machine's
/// `spid`, if the bundle or device tree supplies it.
pub fn speaker_chain(b: &Bundle, speaker_id: Option<i128>, diags: &mut Diagnostics) -> Result<Selection> {
    let aid = b.manifest.aid.clone();
    let ctx = "DSP database";
    let Some(db) = database(b, diags)? else {
        diags.warn(
            "select-no-database",
            ctx,
            "no DSP database in the bundle: using the conventional file names; the speaker-vendor strip variant and \
             the sample-rate substitution are unknown",
        );
        let strip = format!("{aid}/DSP/Strips/builtin_speaker_out_general.austrip");
        let prop = format!("{aid}/DSP/Strips/builtin_speaker_out_general.propstrip");
        return Ok(Selection {
            configuration: None,
            graph: format!("{aid}/DSP/Graphs/builtin_speaker_out.dspg"),
            propstrip: b.file(&prop).map(|_| prop),
            strip,
            substitutions: BTreeMap::new(),
            sample_rate: None,
            speaker_id,
            all: Vec::new(),
        });
    };
    let dev = match db.get(SPEAKER_DEVICE) {
        Some(Value::Dict(d)) => d,
        _ => return Err(Error::format(ctx, format!("{aid} has no {SPEAKER_DEVICE} device"))),
    };
    let mut all = Vec::new();
    for (name, v) in dev.iter() {
        if let Value::Dict(d) = v {
            all.push(configuration(name, d, &aid, ctx)?);
        }
    }
    // The "General" kind, read from the database's own names.
    let general: Vec<i128> =
        all.iter().filter(|c| c.name.contains("General")).filter_map(|c| c.cdes_kind).collect::<Vec<_>>();
    let kind = match general.first() {
        Some(k) if general.iter().all(|x| x == k) => *k,
        _ => return Err(Error::format(ctx, "cannot identify the General configuration kind")),
    };
    let candidates: Vec<&Configuration> =
        all.iter().filter(|c| c.cdes_kind == Some(kind) && c.graph.is_some() && c.strip.is_some()).collect();
    let neutral = |c: &&Configuration| c.speaker_id.is_none() || c.speaker_id == Some(SPID_GENERIC);
    // A database whose General configurations name no speaker vendor (the
    // M2's, the J700's) has one chain for every vendor: the speaker ID
    // selects nothing there, so it is neither needed nor checked.
    let by_vendor = candidates.iter().any(|c| !neutral(c));
    let pick: Vec<&Configuration> = match speaker_id {
        _ if !by_vendor => {
            if let Some(id) = speaker_id {
                diags.info(
                    "select-speaker-id-unused",
                    ctx,
                    format!("speaker ID {id}: the database has one configuration for every speaker vendor"),
                );
            }
            candidates.iter().copied().filter(neutral).collect()
        }
        Some(id) => candidates.iter().copied().filter(|c| c.speaker_id == Some(id)).collect(),
        None => {
            diags.warn(
                "select-speaker-id-unknown",
                ctx,
                format!("speaker ID unknown: using the vendor-neutral (spid {SPID_GENERIC} or none) configuration"),
            );
            candidates.iter().copied().filter(neutral).collect()
        }
    };
    let c = match pick.as_slice() {
        [c] => *c,
        [] => {
            return Err(Error::format(
                ctx,
                format!("no {SPEAKER_DEVICE} configuration of kind {kind} for speaker ID {speaker_id:?}"),
            ))
        }
        many => {
            return Err(Error::format(
                ctx,
                format!(
                    "{} configurations match: {}",
                    many.len(),
                    many.iter().map(|c| c.name.as_str()).collect::<Vec<_>>().join(", ")
                ),
            ))
        }
    };
    Ok(Selection {
        configuration: Some(c.name.clone()),
        graph: c.graph.clone().unwrap(),
        strip: c.strip.clone().unwrap(),
        propstrip: c.propstrip.clone(),
        substitutions: c.substitutions.clone(),
        sample_rate: c.sample_rate,
        speaker_id: c.speaker_id.or(speaker_id),
        all,
    })
}
