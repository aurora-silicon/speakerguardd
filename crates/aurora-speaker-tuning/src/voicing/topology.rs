// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The speaker chain's shape, from the Mac's own graph: which boxes run,
//! in what order, on which channels, how the program fans out to the
//! unit's lanes, and which box parameters the graph's volume parameters
//! drive. Structure only: no box's saved state is read here.
//!
//! The plan is built from the lanes the graph traces into the protection
//! unit ([`crate::dspg::analysis::Lane`]). Every box a lane passes, up to
//! the unit, becomes a [`Node`] of the plan. For each of a node's channels
//! the plan records where that channel comes from: a program channel, or a
//! channel of an earlier node. Splits and joins vanish into those sources;
//! a fan-out (one channel feeding several later channels, the stereo
//! program feeding the crossover's woofer and tweeter lanes) is a source
//! named more than once. Reblockers and format converters change block
//! size or layout only and are left out.
//!
//! A node's [`Role`] says where it sits:
//!
//! * [`Role::Program`]: upstream of the fan-out (no wider than the
//!   program, and no node before it is wider); a per-side gain runs on one
//!   program channel. macOS's stereo boxes: NFP, Norm, VirtBass, the tone boxes on
//!   the program, the per-side gain and balance, XTC, VarEQ, Comp.
//! * [`Role::Lane`]: after the fan-out, every lane passes it (the analytical
//!   crossover box, AULowFlow on the crossover bus).
//! * [`Role::Group`]: after the fan-out, only some lanes pass it (the per-group limiters
//!   ControlFreakLF/HF, Buzz, Delay).
//!
//! A channel of a node no unit lane passes cannot be fed; the plan refuses
//! it, as it refuses a channel two lanes reach from different sources.
//!
//! A `sum` builtin is a node of [`NodeKind::Sum`]: its channels are its
//! inputs' channels, input after input (input `q`'s channel `c` at
//! `q * width + c`), and its output channel `c` is their sum; later boxes
//! name only its first `width` channels.

use crate::dspg::analysis::Lane;
use crate::error::{Error, Result};
use crate::tuning::Tuning;
use serde::Serialize;

/// Builtins that only re-block or re-lay a bus: no level or content change.
const PLUMBING: &[&str] = &["reblocker", "srlconverter"];

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum NodeKind {
    /// An AudioUnit box: its subtype (`ldnm`, `vben`, `tmst`, ...).
    Unit { subtype: String },
    /// The graph's gain builtin (`dbgn`): per-side gain, balance, ramp.
    Gain,
    /// The graph's non-finite protection builtin (`nfnp`).
    NonFinite,
    /// The graph's `sum` builtin over `inputs` equally wide inputs.
    Sum { inputs: u32 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    Program,
    Lane,
    Group,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(tag = "from", rename_all = "snake_case")]
pub enum Source {
    /// Channel `channel` of the program (the graph's audio input).
    Program { channel: u32 },
    /// Channel `channel` of node `node` (an index into [`Plan::nodes`]).
    Node { node: usize, channel: u32 },
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Node {
    /// The box's name in the graph.
    pub name: String,
    pub kind: NodeKind,
    pub role: Role,
    /// Per channel of the box, where its input comes from.
    pub inputs: Vec<Source>,
    /// The unit lanes that pass it.
    pub lanes: Vec<usize>,
    /// The strip bypasses it: it passes its input through unchanged.
    pub bypass: bool,
}

impl Node {
    pub fn width(&self) -> usize {
        self.inputs.len()
    }
    /// The channels the node puts out (a sum's inputs are wider).
    pub fn out_width(&self) -> usize {
        match self.kind {
            NodeKind::Sum { inputs } => self.inputs.len() / inputs.max(1) as usize,
            _ => self.inputs.len(),
        }
    }
    pub fn subtype(&self) -> Option<&str> {
        match &self.kind {
            NodeKind::Unit { subtype } => Some(subtype),
            _ => None,
        }
    }
}

/// A graph parameter wired to a parameter of a box in the plan.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ParamLink {
    /// The graph parameter (`mvsc`, `mvol`, `mmut`, `lbal`, `rbal`, ...).
    pub param: String,
    /// Its default in the graph.
    pub default: Option<f64>,
    pub node: usize,
    pub id: u32,
    /// Scope and element words, if the link names them.
    pub rest: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Plan {
    /// Program channels (the graph input's width).
    pub program_channels: u32,
    /// In processing order.
    pub nodes: Vec<Node>,
    /// Per unit channel, in the unit's lane order: what feeds it.
    pub outputs: Vec<Source>,
    /// Graph parameters wired into the plan's boxes.
    pub params: Vec<ParamLink>,
}

impl Plan {
    /// Index of the node named `name`.
    pub fn node(&self, name: &str) -> Option<usize> {
        self.nodes.iter().position(|n| n.name == name)
    }

    /// Program nodes, then lane and group nodes, in processing order: the
    /// node after which the program has fanned out to the lanes.
    pub fn fanout_at(&self) -> usize {
        self.nodes.iter().position(|n| n.role != Role::Program).unwrap_or(self.nodes.len())
    }
}

fn bad(msg: impl Into<String>) -> Error {
    Error::format("voicing topology", msg.into())
}

fn source_of(lane: &Lane, step: usize, kept: &[Option<usize>]) -> Source {
    // the nearest kept box upstream of `step` on the lane, else the program
    for k in (0..step).rev() {
        if let Some(n) = kept[k] {
            return Source::Node { node: n, channel: lane.through[k].channel };
        }
    }
    Source::Program { channel: lane.origin.channel }
}

/// Whether `lane`, or a lane a sum on it adds in, passes box `name`.
fn passes(lane: &Lane, name: &str) -> bool {
    lane.through.iter().any(|p| p.node == name) || lane.summed.iter().any(|s| s.lanes.iter().any(|l| passes(l, name)))
}

struct Found<'a> {
    names: &'a [&'a str],
    sources: Vec<Vec<Option<Source>>>,
    lanes: Vec<Vec<usize>>,
}

impl Found<'_> {
    fn index(&self, name: &str) -> Option<usize> {
        self.names.iter().position(|n| *n == name)
    }

    fn set(&mut self, n: usize, ch: usize, src: Source, li: usize) -> Result<()> {
        let name = self.names[n];
        let slot = self.sources[n]
            .get_mut(ch)
            .ok_or_else(|| bad(format!("lane {li} enters box {name} on channel {ch}, past its width")))?;
        match slot {
            None => *slot = Some(src),
            Some(prev) if *prev == src => {}
            Some(prev) => {
                return Err(bad(format!("box {name} channel {ch} is reached from {prev:?} and from {src:?}")))
            }
        }
        if !self.lanes[n].contains(&li) {
            self.lanes[n].push(li);
        }
        Ok(())
    }

    /// Record every box `lane` (of unit lane `li`) passes, and what feeds
    /// each; a sum on the lane takes its other inputs from the lanes it adds.
    /// Returns what the lane carries at its end.
    fn lane(&mut self, lane: &Lane, li: usize, widths: &[usize]) -> Result<Source> {
        let kept: Vec<Option<usize>> = lane.through.iter().map(|p| self.index(&p.node)).collect();
        for (k, pass) in lane.through.iter().enumerate() {
            let Some(n) = kept[k] else { continue };
            let ch = pass.channel as usize;
            self.set(n, ch, source_of(lane, k, &kept), li)?;
            for s in lane.summed.iter().filter(|s| s.at == k) {
                for (q, other) in s.lanes.iter().enumerate() {
                    let src = self.lane(other, li, widths)?;
                    self.set(n, (q + 1) * widths[n] + ch, src, li)?;
                }
            }
        }
        Ok(source_of(lane, lane.through.len(), &kept))
    }
}

/// The chain plan of `t`'s protection unit.
pub fn plan(t: &Tuning) -> Result<Plan> {
    let lanes = &t.protection.lanes;
    if lanes.is_empty() {
        return Err(bad("the graph traces no lanes into the unit"));
    }
    let port = &lanes[0].origin.node;
    if lanes.iter().any(|l| &l.origin.node != port || !t.graph.inputs.contains(&l.origin.node)) {
        return Err(bad("the unit's lanes do not all start at one graph input"));
    }
    let program_channels = t
        .graph
        .wires
        .iter()
        .find(|w| &w.src.node == port)
        .and_then(|w| crate::dspg::analysis::wire_channels(&t.graph, w).ok())
        .ok_or_else(|| bad(format!("the width of graph input {port} is unknown")))?;

    // Boxes on the lanes, in the chain's processing order.
    let mut names: Vec<&str> = Vec::new();
    for s in t.chain.iter().take(t.protection.position) {
        if PLUMBING.contains(&s.kind.as_str()) {
            continue;
        }
        if lanes.iter().any(|l| passes(l, &s.name)) {
            names.push(&s.name);
        }
    }

    let mut nodes: Vec<Node> = Vec::with_capacity(names.len());
    // per node: its channels (a sum's inputs, input after input) and its
    // output channels
    let mut widths: Vec<(usize, usize)> = Vec::with_capacity(names.len());
    for name in &names {
        let s = t.chain.iter().find(|s| s.name == *name).expect("chain stage");
        // a strip may name a builtin by its unit subtype (`abox summ`)
        let sum =
            s.kind == "sum" || s.kind == "summ" || s.unit.as_ref().is_some_and(|u| u.subtype.to_string() == "summ");
        let kind = match (&s.unit, s.kind.as_str()) {
            _ if sum => {
                let inputs = t.graph.find_box(name).map_or(0, |b| b.inputs);
                if inputs == 0 {
                    return Err(bad(format!("box {name}: a sum with no inputs")));
                }
                NodeKind::Sum { inputs }
            }
            (Some(u), _) if u.subtype.to_string() == "nfnp" => NodeKind::NonFinite,
            (Some(u), _) if u.subtype.to_string() == "dbgn" => NodeKind::Gain,
            (Some(u), _) => NodeKind::Unit { subtype: u.subtype.to_string() },
            (None, "dbgn") => NodeKind::Gain,
            (None, "nfnp") => NodeKind::NonFinite,
            (None, k) => return Err(bad(format!("box {name}: builtin {k} on the unit's lanes is not carried"))),
        };
        let out = s.channels.ok_or_else(|| bad(format!("box {name}: its channel count is unknown")))? as usize;
        let width = match kind {
            NodeKind::Sum { inputs } => out * inputs as usize,
            _ => out,
        };
        if out == 0 || width > sgv2_core::MAX_CHANNELS {
            return Err(bad(format!("box {name}: {width} channels")));
        }
        widths.push((width, out));
        nodes.push(Node {
            name: (*name).to_string(),
            kind,
            role: Role::Program,
            inputs: Vec::new(),
            lanes: Vec::new(),
            bypass: s.bypass,
        });
    }

    // Sources per node channel, from every lane that passes it.
    let out_widths: Vec<usize> = widths.iter().map(|w| w.1).collect();
    let mut found = Found {
        names: &names,
        sources: widths.iter().map(|w| vec![None; w.0]).collect(),
        lanes: vec![Vec::new(); nodes.len()],
    };
    let mut outputs = Vec::with_capacity(lanes.len());
    for (li, lane) in lanes.iter().enumerate() {
        outputs.push(found.lane(lane, li, &out_widths)?);
    }
    for (n, l) in found.lanes.into_iter().enumerate() {
        nodes[n].lanes = l;
    }
    for (n, chans) in found.sources.into_iter().enumerate() {
        let mut inputs = Vec::with_capacity(chans.len());
        for (c, s) in chans.into_iter().enumerate() {
            inputs.push(s.ok_or_else(|| bad(format!("box {} channel {c} is on none of the unit's lanes", names[n])))?);
        }
        nodes[n].inputs = inputs;
    }

    // Roles: everything up to the first box wider than the program runs on
    // the program (per-side gains run on one of its channels); after it,
    // a box every lane passes is a lane box, any other a group box.
    let all = lanes.len();
    let mut widened = false;
    for n in nodes.iter_mut() {
        widened |= n.out_width() > program_channels as usize;
        n.role = if !widened {
            Role::Program
        } else if n.lanes.len() == all {
            Role::Lane
        } else {
            Role::Group
        };
    }

    // Graph parameters into the plan's boxes.
    let mut params = Vec::new();
    for l in &t.graph.param_links {
        let Some(node) = nodes.iter().position(|n| n.name == l.target.node) else { continue };
        let id =
            l.target.id.value().filter(|v| *v >= 0.0 && v.fract() == 0.0 && *v <= u32::MAX as f64).ok_or_else(
                || bad(format!("graph parameter {} -> {}: unresolved parameter ID", l.name, l.target.node)),
            )?;
        params.push(ParamLink {
            param: l.name.clone(),
            default: t.graph.params.iter().find(|p| p.name == l.name).map(|p| p.default),
            node,
            id: id as u32,
            rest: l.target.rest.clone(),
        });
    }

    Ok(Plan { program_channels, nodes, outputs, params })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dspg::analysis::{Origin, Pass};

    fn lane(ch: u32, through: &[(&str, u32)]) -> Lane {
        Lane {
            origin: Origin { node: "in".into(), port: 0, channel: ch },
            hops: Vec::new(),
            through: through.iter().map(|(n, c)| Pass { node: (*n).into(), channel: *c }).collect(),
            summed: Vec::new(),
        }
    }

    #[test]
    fn source_walks_back_past_dropped_boxes() {
        let l = lane(1, &[("A", 1), ("R", 0), ("B", 3)]);
        let kept = [Some(0), None, Some(1)];
        assert_eq!(source_of(&l, 0, &kept), Source::Program { channel: 1 });
        assert_eq!(source_of(&l, 2, &kept), Source::Node { node: 0, channel: 1 });
        assert_eq!(source_of(&l, 3, &kept), Source::Node { node: 1, channel: 3 });
    }
}
