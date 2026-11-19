// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Separate LSID companion, graph scheduling, and inert control-node proof.
use crate::aupreset::ParamState;
use crate::dspg::{analysis, BoxKind, Endpoint, Graph};
use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::strip::{AuStrip, PropStrip};
use serde::Serialize;
use sgv2_core::lsid::{Preset, Topology, LANES};

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Companion {
    pub box_name: String,
    pub state: ParamState,
    #[serde(skip)]
    pub preset: Option<Preset>,
    #[serde(skip)]
    pub topology: Option<Topology>,
    pub source_port: String,
    pub source_lanes: Vec<u32>,
    pub delay_frames: Vec<u32>,
    pub adaptation_enabled: bool,
}
fn bad(s: impl Into<String>) -> Error {
    Error::format("lsid", s.into())
}

pub fn load(
    g: &Graph,
    strip: &AuStrip,
    protection: &str,
    channels: usize,
    diags: &mut Diagnostics,
) -> Result<Option<Companion>> {
    let boxes: Vec<_> = g.boxes_of(FourCc::from_bytes(*b"lsid")).collect();
    let bx = match boxes.as_slice() {
        [] => return Ok(None),
        [b] => b,
        _ => return Err(bad("multiple estimator companions")),
    };
    let e = strip.effect(&bx.name).ok_or_else(|| bad("companion preset missing"))?;
    if !bx.matches_unit(&e.unit)
        || e.unit.kind.bytes() != *b"aufx"
        || e.unit.manufacturer.bytes() != *b"appl"
        || e.bypass
        || bx.inputs != 1
        || bx.outputs != 1
    {
        return Err(bad("invalid companion identity or port geometry"));
    }
    if e.preset.state.sections.len() != 1 {
        return Err(bad("expected one global saved-state section"));
    }
    let global = e.preset.state.global().ok_or_else(|| bad("missing global saved state"))?;
    let preset = if channels == 6 || global.params.len() == sgv2_core::lsid::RECORDS {
        Some(Preset::decode(&global.params).map_err(|err| bad(format!("603-record schema: {err:?}")))?)
    } else {
        // Older companions are retained, never interpreted or enabled. No
        // numerical or ID law for their state is claimed by the clean spec.
        diags.warn("lsid-legacy-opaque", &bx.name, "legacy estimator state preserved; adaptation unavailable");
        None
    };
    let lanes = analysis::lanes_into(g, &Endpoint { node: bx.name.clone(), port: 0 })?;
    if lanes.len() != 2 * channels || lanes.is_empty() {
        return Err(bad("sense width does not match physical pairs"));
    }
    let port = lanes[0].origin.node.clone();
    if !g.inputs.contains(&port)
        || lanes.iter().enumerate().any(|(i, l)| l.origin.node != port || l.origin.channel != i as u32)
    {
        return Err(bad("sense source/order differs from the graph contract"));
    }
    let wire = g.wires.iter().find(|w| w.src.node == port).ok_or_else(|| bad("sense source missing"))?;
    let fmt = g.wire_format(wire).ok_or_else(|| bad("sense source format missing"))?;
    if !fmt.interleaved || analysis::wire_channels(g, wire)? as usize != 2 * channels {
        return Err(bad("sense source must be interleaved V/I pairs"));
    }
    let mut delays = vec![0; lanes.len()];
    for (i, lane) in lanes.iter().enumerate() {
        for pass in &lane.through {
            let b = g.find_box(&pass.node).ok_or_else(|| bad("unknown sense box"))?;
            match &b.kind {
                BoxKind::Builtin(k) if k == "srlconverter" || k == "reblocker" => (),
                BoxKind::Builtin(k) if k == "delay" => {
                    let links: Vec<_> = g.param_links.iter().filter(|l| l.target.node == b.name).collect();
                    let [link] = links.as_slice() else {
                        return Err(bad("delay must have one explicit graph parameter"));
                    };
                    let value = g
                        .params
                        .iter()
                        .find(|p| p.name == link.name)
                        .ok_or_else(|| bad("delay parameter missing"))?
                        .default;
                    if link.target.id.value() != Some(0.0) || value != 1.0 {
                        return Err(bad("unsupported sense delay"));
                    }
                    delays[i] += 1;
                }
                _ if channels != 6 => (), // opaque legacy side chain, kept disabled
                _ => return Err(bad("unsupported box on the twelve-lane sensing path")),
            }
        }
    }
    let topology = if channels == 6 {
        let top = Topology::new(&port).map_err(|_| bad("source identity is too long"))?;
        if lanes.len() != LANES || delays.as_slice() != top.delay_frames {
            return Err(bad("expected lane 1 alone delayed by one frame"));
        }
        Some(top)
    } else {
        None
    };
    let consumers: Vec<_> = g.wires.iter().filter(|w| w.src.node == bx.name).collect();
    let [out] = consumers.as_slice() else {
        return Err(bad("LSID output must terminate in one dead end"));
    };
    if analysis::wire_channels(g, out)? as usize != 2 * channels
        || g.wire_format(out).is_none_or(|f| f.interleaved || f.frequency_domain || f.sample_rate != fmt.sample_rate)
        || !g
            .find_box(&out.dst.node)
            .is_some_and(|b| b.kind == BoxKind::Builtin("dead".into()) && b.inputs == 1 && b.outputs == 0)
    {
        return Err(bad("LSID output is consumed by the audio graph"));
    }
    let handoffs: Vec<_> = g.box_property_links.iter().filter(|l| l.from.node == bx.name).collect();
    let global = |target: &crate::dspg::Target| {
        target.rest.len() == 2 && matches!(target.rest[0].as_str(), "0" | "global") && target.rest[1] == "0"
    };
    if handoffs.len() != 1
        || !global(&handoffs[0].from)
        || !global(&handoffs[0].to)
        || handoffs[0].from.id.value() != Some(6630.0)
        || handoffs[0].to.node != protection
        || handoffs[0].to.id.value() != Some(6630.0)
    {
        return Err(bad("missing or ambiguous estimator-to-controller property 6630"));
    }
    diags.warn(
        "lsid-disabled",
        &bx.name,
        "LSID solver unqualified and disabled; LSM2 retains its static/open-loop model; measured power unavailable",
    );
    Ok(Some(Companion {
        box_name: bx.name.clone(),
        state: e.preset.state.clone(),
        preset,
        topology,
        source_port: port,
        source_lanes: lanes.iter().map(|l| l.origin.channel).collect(),
        delay_frames: delays,
        adaptation_enabled: false,
    }))
}

/// Prove absence of *all* graph connectivity, including non-audio wires,
/// before eliminating a zero-port control component. Stored parameters are
/// inert; a property-strip setter would make it non-orphan and is refused.
pub fn eliminate_dbcontrol(g: &Graph, strip: &AuStrip, props: Option<&PropStrip>) -> Result<Vec<String>> {
    let mut eliminated = Vec::new();
    for bx in &g.boxes {
        if bx.kind != BoxKind::Builtin("dbcontrol".into()) {
            continue;
        }
        let n = &bx.name;
        if bx.inputs != 0
            || bx.outputs != 0
            || g.wires.iter().any(|w| &w.src.node == n || &w.dst.node == n)
            || g.param_links.iter().chain(&g.property_links).any(|l| &l.target.node == n)
            || g.box_param_links.iter().chain(&g.box_property_links).any(|l| &l.from.node == n || &l.to.node == n)
            || props.and_then(|p| p.get(n)).is_some_and(|p| !p.properties.is_empty())
        {
            return Err(bad("connected dbcontrol is unsupported"));
        }
        let e = strip.effect(n).ok_or_else(|| bad("dbcontrol preset missing"))?;
        if e.unit.kind.bytes() != *b"cntl"
            || e.unit.subtype.bytes() != *b"dbct"
            || e.unit.manufacturer.bytes() != *b"abox"
            || e.preset.state.sections.len() != 1
            || e.preset.state.global().is_none_or(|s| {
                s.params.len() != 2 || s.params.iter().any(|r| !r.1.is_finite()) || s.params[0].0 == s.params[1].0
            })
        {
            return Err(bad("dbcontrol identity or saved-state geometry differs"));
        }
        eliminated.push(n.clone());
    }
    Ok(eliminated)
}
