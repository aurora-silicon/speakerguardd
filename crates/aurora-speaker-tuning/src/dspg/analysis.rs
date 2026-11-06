// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Structural analysis of a parsed graph: processing order and channel lanes.
//!
//! **Processing order** is a topological order of the boxes over the wires
//! (ties broken by declaration order).  It is what the engine must rebuild;
//! the declaration order of `box` statements is not the signal order.
//!
//! **Lanes** trace where each channel of a box input comes from, through the
//! graph's plumbing.  The rules are structural and stated here so the engine
//! can audit them:
//!
//! * a graph input port with an N-channel wire starts N lanes
//!   (`port`, channel 0..N);
//! * `split` with one input: its output ports take consecutive channel
//!   ranges of the input, in port order, each as wide as that port's wires
//!   (every wire leaving one port must agree on the width);
//! * `join`: its output is the concatenation of its inputs in port order;
//!   each lane records the join slot it entered as a [`Hop`];
//! * any other box whose output 0 is as wide as its input 0: lane `k` of the
//!   output is lane `k` of input 0.  This is lane *position*, not signal
//!   content: an effect may mix channels (crosstalk cancellation does), but
//!   it keeps the channel's place in the bus;
//! * otherwise the box starts new lanes at its output (origin = the box).
//!
//! The split/join rules follow from how the shipped graphs use these
//! builtins (a split's outputs are always narrower parts of its input, a
//! join's input widths always sum to its output).  TODO(SPEC): confirm the
//! builtin semantics in the behaviour specification.

use super::{Endpoint, Graph, Wire};
use crate::error::{Error, Result};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct Origin {
    /// A graph input port, or the box that started the lane.
    pub node: String,
    pub port: u32,
    pub channel: u32,
}

/// A lane entering a `join` input.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Hop {
    pub join: String,
    pub input: u32,
    /// The lane's channel in the join's output.
    pub channel: u32,
}

/// A lane passing through a box that keeps it in place (any box but a
/// `split` or `join`): the lane's channel on that box.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Pass {
    pub node: String,
    pub channel: u32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Lane {
    pub origin: Origin,
    pub hops: Vec<Hop>,
    /// The boxes the lane passes through, upstream first, with its channel
    /// on each.
    pub through: Vec<Pass>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Analysis {
    /// Box names in processing order (boxes reachable from a graph input
    /// or with no inputs).
    pub order: Vec<String>,
    /// Boxes not reachable from any graph input (dead or parameter-only).
    pub unreached: Vec<String>,
}

pub fn wire_channels(g: &Graph, w: &Wire) -> Result<u32> {
    let f = g.wire_format(w).ok_or_else(|| {
        Error::format("dspg", format!("line {}: wire {}:{} has no format", w.line, w.src.node, w.src.port))
    })?;
    match f.channels.value() {
        Some(c) if c >= 0.0 && c.fract() == 0.0 && c <= super::MAX_WIRE_CHANNELS as f64 => Ok(c as u32),
        Some(c) if c > super::MAX_WIRE_CHANNELS as f64 => Err(Error::format(
            "dspg",
            format!("line {}: wire carries {c} channels; at most {}", w.line, super::MAX_WIRE_CHANNELS),
        )),
        _ => Err(Error::format(
            "dspg",
            format!("line {}: wire channel count is unresolved ({:?}); supply the substitution", w.line, f.channels),
        )),
    }
}

/// Topological processing order.
pub fn processing_order(g: &Graph) -> Result<Analysis> {
    let idx: BTreeMap<&str, usize> = g.boxes.iter().enumerate().map(|(i, b)| (b.name.as_str(), i)).collect();
    let mut indeg = vec![0usize; g.boxes.len()];
    let mut succ: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); g.boxes.len()];
    let mut reached = vec![false; g.boxes.len()];
    let mut stack: Vec<usize> = Vec::new();
    for w in &g.wires {
        match (idx.get(w.src.node.as_str()), idx.get(w.dst.node.as_str())) {
            (Some(&s), Some(&d)) => {
                if succ[s].insert(d) {
                    indeg[d] += 1;
                }
            }
            (None, Some(&d)) => stack.push(d),
            _ => {}
        }
    }
    while let Some(b) = stack.pop() {
        if !reached[b] {
            reached[b] = true;
            stack.extend(succ[b].iter().copied());
        }
    }
    let mut ready: BTreeSet<usize> = (0..g.boxes.len()).filter(|&i| indeg[i] == 0).collect();
    let mut order = Vec::new();
    while let Some(&b) = ready.iter().next() {
        ready.remove(&b);
        order.push(b);
        for &s in &succ[b] {
            indeg[s] -= 1;
            if indeg[s] == 0 {
                ready.insert(s);
            }
        }
    }
    if order.len() != g.boxes.len() {
        let stuck: Vec<&str> = (0..g.boxes.len()).filter(|i| indeg[*i] > 0).map(|i| g.boxes[i].name.as_str()).collect();
        return Err(Error::format("dspg", format!("the graph has a cycle through {}", stuck.join(", "))));
    }
    let names = |keep: bool| -> Vec<String> {
        order.iter().filter(|&&i| reached[i] == keep).map(|&i| g.boxes[i].name.clone()).collect()
    };
    Ok(Analysis { order: names(true), unreached: names(false) })
}

/// Traces the lanes arriving at `dst`.
pub fn lanes_into(g: &Graph, dst: &Endpoint) -> Result<Vec<Lane>> {
    let mut memo: BTreeMap<Endpoint, Vec<Lane>> = BTreeMap::new();
    let mut active = BTreeSet::new();
    let w = g
        .wires
        .iter()
        .find(|w| &w.dst == dst)
        .ok_or_else(|| Error::format("dspg", format!("nothing feeds {}:{}", dst.node, dst.port)))?;
    let lanes = out_lanes(g, &w.src, &mut memo, &mut active, 0)?;
    let n = wire_channels(g, w)? as usize;
    if lanes.len() != n {
        return Err(Error::format(
            "dspg",
            format!("line {}: wire carries {n} channels but its source has {}", w.line, lanes.len()),
        ));
    }
    Ok(lanes)
}

/// Longest box chain the lane tracer follows (it recurses once per box).
/// The shipped graphs chain a few tens of boxes.
pub const MAX_TRACE_DEPTH: usize = 256;

fn in_wire<'a>(g: &'a Graph, node: &str, port: u32) -> Option<&'a Wire> {
    g.wires.iter().find(|w| w.dst.node == node && w.dst.port == port)
}

fn out_lanes(
    g: &Graph,
    src: &Endpoint,
    memo: &mut BTreeMap<Endpoint, Vec<Lane>>,
    active: &mut BTreeSet<String>,
    depth: usize,
) -> Result<Vec<Lane>> {
    if depth > MAX_TRACE_DEPTH {
        return Err(Error::format(
            "dspg",
            format!("the signal path into {} is longer than {MAX_TRACE_DEPTH} boxes", src.node),
        ));
    }
    if let Some(l) = memo.get(src) {
        return Ok(l.clone());
    }
    let lanes = if g.inputs.contains(&src.node) {
        let w = g.wires.iter().find(|w| w.src == *src).expect("source has a wire");
        let n = wire_channels(g, w)?;
        (0..n)
            .map(|c| Lane {
                origin: Origin { node: src.node.clone(), port: 0, channel: c },
                hops: Vec::new(),
                through: Vec::new(),
            })
            .collect()
    } else {
        let b = g.find_box(&src.node).ok_or_else(|| Error::format("dspg", format!("unknown node {}", src.node)))?;
        if !active.insert(b.name.clone()) {
            return Err(Error::format("dspg", format!("cycle through {}", b.name)));
        }
        let input = |port: u32, memo: &mut BTreeMap<Endpoint, Vec<Lane>>, active: &mut BTreeSet<String>| match in_wire(
            g, &b.name, port,
        ) {
            Some(w) => {
                let l = out_lanes(g, &w.src, memo, active, depth + 1)?;
                let n = wire_channels(g, w)? as usize;
                if l.len() != n {
                    return Err(Error::format(
                        "dspg",
                        format!("line {}: wire carries {n} channels, source has {}", w.line, l.len()),
                    ));
                }
                Ok(Some(l))
            }
            None => Ok(None),
        };
        let out_width = |port: u32| -> Result<Option<u32>> {
            let mut width = None;
            for w in g.wires.iter().filter(|w| w.src.node == b.name && w.src.port == port) {
                let c = wire_channels(g, w)?;
                if width.is_some_and(|x| x != c) {
                    return Err(Error::format("dspg", format!("{}:{} feeds wires of different widths", b.name, port)));
                }
                width = Some(c);
            }
            Ok(width)
        };
        let fresh = |n: u32| -> Vec<Lane> {
            (0..n)
                .map(|c| Lane {
                    origin: Origin { node: b.name.clone(), port: src.port, channel: c },
                    hops: Vec::new(),
                    through: Vec::new(),
                })
                .collect()
        };
        let kind = match &b.kind {
            super::BoxKind::Builtin(k) => k.as_str(),
            super::BoxKind::AudioUnit { .. } => "",
        };
        let result = match kind {
            "split" if b.inputs == 1 => {
                let inl = input(0, memo, active)?.unwrap_or_default();
                let mut start = 0usize;
                let mut mine = Vec::new();
                for p in 0..b.outputs {
                    let wdt = out_width(p)?.ok_or_else(|| {
                        Error::format(
                            "dspg",
                            format!("split {} output {p} is unconnected; its width is unknown", b.name),
                        )
                    })? as usize;
                    if start + wdt > inl.len() {
                        return Err(Error::format(
                            "dspg",
                            format!("split {} outputs are wider than its {}-channel input", b.name, inl.len()),
                        ));
                    }
                    if p == src.port {
                        mine = inl[start..start + wdt].to_vec();
                    }
                    start += wdt;
                }
                if start != inl.len() {
                    return Err(Error::format(
                        "dspg",
                        format!("split {} outputs cover {start} of {} input channels", b.name, inl.len()),
                    ));
                }
                mine
            }
            "join" if b.outputs == 1 => {
                let mut all = Vec::new();
                for p in 0..b.inputs {
                    let l = input(p, memo, active)?
                        .ok_or_else(|| Error::format("dspg", format!("join {} input {p} is unconnected", b.name)))?;
                    for mut lane in l {
                        lane.hops.push(Hop { join: b.name.clone(), input: p, channel: all.len() as u32 });
                        all.push(lane);
                    }
                }
                all
            }
            _ => {
                let width = out_width(src.port)?.unwrap_or(0) as usize;
                match (src.port, input(0, memo, active)?) {
                    (0, Some(inl)) if inl.len() == width => inl
                        .into_iter()
                        .enumerate()
                        .map(|(c, mut lane)| {
                            lane.through.push(Pass { node: b.name.clone(), channel: c as u32 });
                            lane
                        })
                        .collect(),
                    _ => fresh(width as u32),
                }
            }
        };
        active.remove(&b.name);
        result
    };
    memo.insert(src.clone(), lanes.clone());
    Ok(lanes)
}
