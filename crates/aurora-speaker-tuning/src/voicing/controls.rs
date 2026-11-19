// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Zero-audio-port parameter arithmetic. Constants retain their declared
//! defaults; the lid input is live. No control box becomes an audio bypass.

use super::topology::{Node, NodeKind, ParamLink, Plan, Role, Source};
use crate::dspg::{BoxKind, Target};
use crate::error::{Error, Result};
use crate::tuning::Tuning;
use sgv2_core::voicing::chain::Relay;

fn bad(message: impl Into<String>) -> Error {
    Error::format("graph controls", message)
}

fn input(target: &Target) -> Result<usize> {
    match target.rest.as_slice() {
        [scope, element] if scope == "input" && target.id.value() == Some(0.0) => element
            .parse::<usize>()
            .ok()
            .filter(|n| *n < 16)
            .ok_or_else(|| bad("parameter_add input element is outside 0..16")),
        _ => Err(bad("parameter_add requires parameter 0 at input scope and a bounded element")),
    }
}

pub fn values(t: &Tuning, name: &str, lid: Option<f64>) -> Result<Vec<f64>> {
    let mut values = Vec::new();
    for link in t.graph.param_links.iter().filter(|l| l.target.node == name) {
        let slot = input(&link.target)?;
        let param = t
            .graph
            .params
            .iter()
            .find(|p| p.name == link.name)
            .ok_or_else(|| bad("parameter_add input has no graph parameter"))?;
        if !param.constant && param.name != "lids" {
            return Err(bad("parameter_add input is neither a constant nor the carried lid input"));
        }
        if param.name == "lids" && ![0.0, 1.0].contains(&param.default) {
            return Err(bad("lid default is not binary"));
        }
        if values.len() <= slot {
            values.resize(slot + 1, None);
        }
        if values[slot].is_some() {
            return Err(bad("parameter_add input is wired twice"));
        }
        values[slot] =
            Some(if !param.constant && param.name == "lids" { lid.unwrap_or(param.default) } else { param.default });
    }
    if values.is_empty() {
        return Err(bad("parameter_add has no inputs"));
    }
    values.into_iter().map(|v| v.ok_or_else(|| bad("parameter_add input elements have a hole"))).collect()
}

pub fn mask(t: &Tuning, name: &str, lid: Option<f64>) -> Result<u32> {
    let sum: f64 = values(t, name, lid)?.iter().sum();
    if !(0.0..=1023.0).contains(&sum) || sum.fract() != 0.0 {
        return Err(bad("parameter_add does not produce a ToneMeister group mask"));
    }
    Ok(sum as u32)
}

pub fn source<'a>(t: &'a Tuning, target: &str) -> Option<&'a str> {
    t.graph
        .box_param_links
        .iter()
        .find(|l| l.to.node == target && l.to.id.value() == Some(1.0))
        .and_then(|l| t.graph.find_box(&l.from.node))
        .filter(|b| matches!(&b.kind, BoxKind::Builtin(k) if k == "parameter_add"))
        .map(|b| b.name.as_str())
}

/// Prepend connected control nodes, adjusting every audio-node reference.
/// The supported arithmetic has direct graph inputs, so control dependencies
/// cannot form a cycle. Unsupported connected controls refuse explicitly.
pub fn prepare(t: &Tuning, plan: &mut Plan) -> Result<Vec<Relay>> {
    let mut controls = Vec::new();
    for b in &t.graph.boxes {
        if !matches!(&b.kind, BoxKind::Builtin(k) if k == "parameter_add") {
            continue;
        }
        let outgoing: Vec<_> = t.graph.box_param_links.iter().filter(|l| l.from.node == b.name).collect();
        if outgoing.is_empty() {
            continue;
        }
        if b.inputs != 0
            || b.outputs != 0
            || t.graph.box_param_links.iter().any(|l| l.to.node == b.name)
            || t.graph.property_links.iter().any(|l| l.target.node == b.name)
            || t.graph.box_property_links.iter().any(|l| l.from.node == b.name || l.to.node == b.name)
        {
            return Err(bad("parameter_add has unsupported port or control connectivity"));
        }
        for l in outgoing {
            if l.from.id.value() != Some(0.0)
                || l.from.rest != ["output", "0"]
                || l.to.id.value() != Some(1.0)
                || !l.to.rest.is_empty()
                || plan.node(&l.to.node).is_none_or(|n| plan.nodes[n].subtype() != Some("tmst"))
                || t.graph
                    .box_param_links
                    .iter()
                    .filter(|other| other.to.node == l.to.node && other.to.id == l.to.id)
                    .count()
                    != 1
            {
                return Err(bad("parameter_add output is not a unique ToneMeister group-bypass wire"));
            }
        }
        controls.push(Node {
            name: b.name.clone(),
            kind: NodeKind::ParameterAdd { values: values(t, &b.name, None)? },
            role: Role::Program,
            inputs: vec![],
            lanes: vec![],
            bypass: false,
        });
    }
    let count = controls.len();
    let shift = |s: &mut Source| {
        if let Source::Node { node, .. } = s {
            *node += count;
        }
    };
    for node in &mut plan.nodes {
        for input in &mut node.inputs {
            shift(input);
        }
    }
    for output in &mut plan.outputs {
        shift(output);
    }
    for param in &mut plan.params {
        param.node += count;
    }
    for (node, control) in controls.iter().enumerate() {
        for l in t.graph.param_links.iter().filter(|l| l.target.node == control.name) {
            let param = t.graph.params.iter().find(|p| p.name == l.name).expect("checked parameter");
            if !param.constant {
                plan.params.push(ParamLink {
                    param: param.name.clone(),
                    default: Some(param.default),
                    node,
                    id: input(&l.target)? as u32,
                    rest: vec![],
                });
            }
        }
    }
    controls.append(&mut plan.nodes);
    plan.nodes = controls;
    let mut relays = Vec::new();
    for l in &t.graph.box_param_links {
        if let Some(from_node) = plan.node(&l.from.node) {
            if matches!(plan.nodes[from_node].kind, NodeKind::ParameterAdd { .. }) {
                relays.push(Relay {
                    from_node,
                    from_id: 0,
                    to_node: plan.node(&l.to.node).expect("checked target"),
                    to_id: 1,
                });
            }
        }
    }
    Ok(relays)
}
