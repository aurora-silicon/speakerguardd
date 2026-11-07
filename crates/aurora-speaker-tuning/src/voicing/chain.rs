// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compiles a Mac's whole voicing chain (`sgv2_core::voicing::chain`) from
//! its graph: the [`super::topology`] plan for the shape, and each box from
//! its saved state.
//!
//! Boxes carried today: the non-finite guard and gain builtins, the filter
//! boxes (`tmst`, through [`crate::prestage::voicing_tone`], unscaled) and the
//! multiband compressor (`mstk`, [`crate::prestage::compressor`]). Every
//! other box type compiles to a stub until its module lands; a box module
//! adds its arm to [`compile_node`]. [`Compiled::stubs`] lists what is
//! still stubbed and [`Compiled::gaps`] what is carried only in part; the rig
//! marks every result that depends on either, and a Mac with any stays on the
//! interim voicing.

use super::topology::{self, NodeKind, Plan, Source as PSource};
use crate::error::{Diagnostics, Error, Result};
use crate::tuning::{Stage, Tuning};
use sgv2_core::voicing::chain::{GraphParam, Kind, Link, NodeSpec, Source, Spec};

pub struct Compiled {
    pub plan: Plan,
    pub spec: Spec,
    /// Node names still stubs, with their subtype.
    pub stubs: Vec<(String, String)>,
    /// Boxes carried only in part (at full volume, say), with what is missing.
    pub gaps: Vec<(String, String)>,
    /// Graph parameters wired into the chain that it does not drive.
    pub unwired: Vec<String>,
}

fn bad(msg: impl Into<String>) -> Error {
    Error::format("voicing chain", msg.into())
}

fn source(s: &PSource) -> Source {
    match *s {
        PSource::Program { channel } => Source::Program(channel as usize),
        PSource::Node { node, channel } => Source::Node(node, channel as usize),
    }
}

fn graph_param(name: &str) -> Option<GraphParam> {
    Some(match name {
        "mvsc" => GraphParam::VolumeScalar,
        "mvol" => GraphParam::VolumeDb,
        "mmut" => GraphParam::Mute,
        "lbal" => GraphParam::BalanceLeft,
        "rbal" => GraphParam::BalanceRight,
        _ => return None,
    })
}

/// A filter box at its saved state and, when the graph wires `mvsc` to its
/// SystemVolume parameter, its gain/frequency volume automation.
fn tone(
    stage: &Stage,
    width: usize,
    volume_wired: bool,
    fs: f64,
    diags: &mut Diagnostics,
    gaps: &mut Vec<(String, String)>,
) -> Result<Kind> {
    let tone = crate::prestage::voicing_tone(stage, width, volume_wired, fs, diags)?;
    if !tone.dynamic.is_empty() {
        gaps.push((
            stage.name.clone(),
            "level-dependent filters carry native multirate reconstruction and a knee-qualified release-to-idle carrier handoff; active-floor and release-tracking parity are pending"
                .into(),
        ));
    }
    Ok(Kind::Tone(tone))
}

/// A box of a carried type whose saved state the module cannot carry: a
/// stub, with the reason as a gap.
fn uncarried(n: &topology::Node, subtype: &str, e: Error, gaps: &mut Vec<(String, String)>) -> Kind {
    gaps.push((n.name.clone(), format!("not carried: {e}")));
    let b = subtype.as_bytes();
    Kind::Stub([b[0], b[1], b[2], b[3]])
}

/// One box. Box modules add their subtype's arm here.
fn compile_node(
    t: &Tuning,
    n: &topology::Node,
    volume_wired: bool,
    fs: f64,
    diags: &mut Diagnostics,
    gaps: &mut Vec<(String, String)>,
) -> Result<Kind> {
    let stage =
        t.chain.iter().find(|s| s.name == n.name).ok_or_else(|| bad(format!("box {} is not in the chain", n.name)))?;
    Ok(match &n.kind {
        NodeKind::NonFinite => Kind::NonFinite,
        NodeKind::Gain => Kind::Gain,
        NodeKind::Unit { .. } if n.bypass => Kind::Bypass,
        NodeKind::Unit { subtype } => match subtype.as_str() {
            "tmst" => match tone(stage, n.width(), volume_wired, fs, diags, gaps) {
                Ok(k) => k,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            // the pre-stage dynamics are proven against macOS on the spp3 states
            // (t-4752); the full box module covers the rest, conformance pending
            "mstk" => match crate::prestage::compressor(stage) {
                Ok(Some(d)) => Kind::Comp(d),
                Ok(None) => Kind::Bypass,
                Err(e0) => match super::mstk::compile(stage, fs, diags) {
                    Ok(Some(p)) => {
                        gaps.push((n.name.clone(), format!("mstk box module (rig conformance pending): {e0}")));
                        Kind::Mstk(Box::new(p))
                    }
                    Ok(None) => Kind::Bypass,
                    Err(e) => uncarried(n, subtype, e, gaps),
                },
            },
            "cfrk" => match super::cfreak::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Cfreak(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            "buzk" => match super::buzz::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Buzz(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            "cdly" => match super::cdelay::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Cdelay(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            "flow" => match super::lowflow::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Lowflow(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            "vben" => match super::vbass::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Vbass(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            "axtc" => match super::xtc::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Xtc(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            other => {
                let b = other.as_bytes();
                if b.len() != 4 {
                    return Err(bad(format!("box {}: subtype {other}", n.name)));
                }
                Kind::Stub([b[0], b[1], b[2], b[3]])
            }
        },
    })
}

/// The whole chain of `t`'s unit at rate `fs`.
pub fn compile(t: &Tuning, fs: f64, diags: &mut Diagnostics) -> Result<Compiled> {
    let plan = topology::plan(t)?;
    let mut nodes = Vec::with_capacity(plan.nodes.len());
    let mut stubs = Vec::new();
    let mut gaps = Vec::new();
    for (i, n) in plan.nodes.iter().enumerate() {
        let wired = plan.params.iter().any(|l| l.node == i && l.param == "mvsc");
        let kind = compile_node(t, n, wired, fs, diags, &mut gaps)?;
        if let Kind::Stub(s) = kind {
            stubs.push((n.name.clone(), String::from_utf8_lossy(&s).into_owned()));
        }
        nodes.push(NodeSpec { kind, inputs: n.inputs.iter().map(source).collect() });
    }
    let mut links = Vec::new();
    let mut unwired = Vec::new();
    for l in &plan.params {
        match graph_param(&l.param) {
            Some(param) => links.push(Link { param, node: l.node, id: l.id }),
            None => unwired.push(format!("{} -> {} {}", l.param, plan.nodes[l.node].name, l.id)),
        }
    }
    let spec = Spec {
        program_channels: plan.program_channels as usize,
        nodes,
        outputs: plan.outputs.iter().map(source).collect(),
        links,
    };
    spec.check().map_err(|e| bad(format!("{e:?}")))?;
    Ok(Compiled { plan, spec, stubs, gaps, unwired })
}
