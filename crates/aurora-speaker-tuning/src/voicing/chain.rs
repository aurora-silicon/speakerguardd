// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compiles a Mac's whole voicing chain (`sgv2_core::voicing::chain`) from
//! its graph: the [`super::topology`] plan for the shape, and each box from
//! its saved state.
//!
//! Boxes carried today: the non-finite guard and gain builtins, the filter
//! boxes (`tmst`, through [`crate::prestage::voicing_tone`], unscaled) and the
//! multiband compressor (`mstk`, [`super::mstk`]). Every
//! other box type compiles to a stub until its module lands; a box module
//! adds its arm to [`compile_node`]. [`Compiled::stubs`] lists what is
//! still stubbed and [`Compiled::gaps`] what is carried only in part (a
//! filter box's volume automation the chain cannot follow exactly); the rig
//! marks every result that depends on either, and a Mac with any stays on the
//! interim voicing.

use super::topology::{self, NodeKind, Plan, Source as PSource};
use crate::error::{Diagnostics, Error, Result};
use crate::tuning::{Stage, Tuning};
use sgv2_core::voicing::chain::{GraphParam, Kind, Link, NodeSpec, Relay, Source, Spec};

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
        "lids" => GraphParam::Lid,
        _ => return None,
    })
}

/// A filter box at its saved state and, when the graph wires `mvsc` to its
/// SystemVolume parameter, its gain/frequency volume automation, which the
/// chain applies exactly at every volume. An automated slot whose law the
/// chain does not apply exactly is carried at the setting that passes least
/// over every volume and reported as a gap (`gaps`), which keeps the Mac on
/// the interim voicing.
fn tone(
    stage: &Stage,
    width: usize,
    volume_wired: bool,
    fs: f64,
    diags: &mut Diagnostics,
    gaps: &mut Vec<(String, String)>,
) -> Result<Kind> {
    let (tone, partial) = crate::prestage::voicing_tone(stage, width, volume_wired, fs, diags)?;
    if !partial.is_empty() {
        gaps.push((
            stage.name.clone(),
            format!("tmst volume automation carried at its least-passing setting (entries {partial:?})"),
        ));
    }
    if tone.dynamic.iter().any(|d| d.params.kind == sgv2_core::prestage::DynamicKind::Notch) {
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
    if let NodeKind::ParameterAdd { values } = &n.kind {
        return Ok(Kind::ParameterAdd(values.clone()));
    }
    let stage =
        t.chain.iter().find(|s| s.name == n.name).ok_or_else(|| bad(format!("box {} is not in the chain", n.name)))?;
    Ok(match &n.kind {
        NodeKind::ParameterAdd { .. } => unreachable!("control node compiled above"),
        NodeKind::NonFinite => Kind::NonFinite,
        NodeKind::Sum { .. } => Kind::Sum(n.out_width()),
        NodeKind::Gain => Kind::Gain,
        NodeKind::Unit { .. } if n.bypass => Kind::Bypass,
        NodeKind::Unit { subtype } => match subtype.as_str() {
            "tmst" => match controlled_tone(t, stage, n.width(), volume_wired, fs, diags, gaps) {
                Ok(k) => k,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            // the box module runs the unit's own gain law (soft knee, level
            // detector, bandwidth behaviour); the pre-stage's interim
            // peak/hard-knee law tilted the whole chain against macOS
            // (2-3.5 dB under at 50-200 Hz, 0.5-2.5 dB over above 300 Hz)
            "mstk" => match super::mstk::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Mstk(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
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
            "ldnm" => match super::norm::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Norm(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            "vrq2" => match super::vareq::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Vareq(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            "axtc" => match super::xtc::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Xtc(Box::new(p)),
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            // the taper's slider follows the system volume only where the
            // graph wires `mvsc` to the box; otherwise it would sit at full
            // volume, not at its saved value
            "volt" => match super::volt::compile(stage, fs, diags) {
                Ok(Some(p)) => {
                    if !volume_wired {
                        gaps.push((n.name.clone(), "volt slider not wired to the system volume".into()));
                    }
                    Kind::Volt(Box::new(p))
                }
                Ok(None) => Kind::Bypass,
                Err(e) => uncarried(n, subtype, e, gaps),
            },
            "pchr" => match super::pchr::compile(stage, fs, diags) {
                Ok(Some(p)) => Kind::Pchr(Box::new(p)),
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

fn controlled_tone(
    t: &Tuning,
    stage: &Stage,
    width: usize,
    volume_wired: bool,
    fs: f64,
    diags: &mut Diagnostics,
    gaps: &mut Vec<(String, String)>,
) -> Result<Kind> {
    let Some(control) = super::controls::source(t, &stage.name) else {
        return tone(stage, width, volume_wired, fs, diags, gaps);
    };
    let default_mask = super::controls::mask(t, control, None)?;
    let mut variants = Vec::new();
    for lid in [0.0, 1.0] {
        let mask = super::controls::mask(t, control, Some(lid))?;
        if variants.iter().any(|v: &sgv2_core::voicing::chain::ToneVariant| v.mask == mask) {
            continue;
        }
        let mut stage = stage.clone();
        let global = stage
            .state
            .as_mut()
            .and_then(|s| s.sections.iter_mut().find(|s| s.scope == 0 && s.element == 0))
            .ok_or_else(|| bad("group-controlled tone has no global state"))?;
        global.params.retain(|(id, _)| *id != 1);
        global.params.push((1, mask as f32));
        // Group switches are exact bypasses, independent of volume wiring.
        let Kind::Tone(tone) = tone(&stage, width, true, fs, diags, gaps)? else { unreachable!() };
        if !volume_wired && (!tone.automation.is_empty() || !tone.dynamic_automation.is_empty()) {
            return Err(bad("group-controlled tone also needs an explicit volume wire"));
        }
        variants.push(sgv2_core::voicing::chain::ToneVariant { mask, tone });
    }
    Ok(Kind::ToneGroups { default_mask, variants })
}

/// The whole chain of `t`'s unit at rate `fs`.
pub fn compile(t: &Tuning, fs: f64, diags: &mut Diagnostics) -> Result<Compiled> {
    let mut plan = topology::plan(t)?;
    let mut relays = super::controls::prepare(t, &mut plan)?;
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
    if t.graph.box_param_links.iter().any(|link| {
        t.graph.find_box(&link.from.node).is_some_and(|b| b.subtype().is_some_and(|s| s.bytes() == *b"volt"))
            && (plan.node(&link.from.node).is_some() || plan.node(&link.to.node).is_some())
    }) {
        let volume = super::volume::compile_tuning(t)?;
        relays.reserve(volume.relays.len());
        for relay in volume.relays {
            let from_node = plan
                .node(&relay.from_box)
                .ok_or_else(|| bad(format!("volume relay source {} is not in the voicing plan", relay.from_box)))?;
            let to_node = plan
                .node(&relay.to_box)
                .ok_or_else(|| bad(format!("volume relay target {} is not in the voicing plan", relay.to_box)))?;
            relays.push(Relay { from_node, from_id: relay.from_id, to_node, to_id: relay.to_id });
        }
    }
    let spec = Spec {
        program_channels: plan.program_channels as usize,
        nodes,
        outputs: plan.outputs.iter().map(source).collect(),
        links,
        relays,
    };
    spec.check().map_err(|e| bad(format!("{e:?}")))?;
    Ok(Compiled { plan, spec, stubs, gaps, unwired })
}
