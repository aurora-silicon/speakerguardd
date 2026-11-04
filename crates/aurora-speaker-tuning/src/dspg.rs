// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! `.dspg`: the text language of a DSP graph.
//!
//! One statement per line; `;` starts a comment.  `[def NAME VALUE...]`
//! defines a macro and `[NAME]` expands it (to its tokens).  Macros the file
//! leaves undefined (typically `fs`) are supplied by the configuration that
//! selects the graph (`Substitutions` in the DSP database); an expansion
//! that stays unresolved is kept as [`Expr::Unresolved`] so a standalone file
//! still parses.
//!
//! Statements:
//!
//! | keyword | form |
//! |---|---|
//! | `graphName` | `"name"` |
//! | `in` / `out` | port name |
//! | `box` | `NAME KIND IN OUT`, KIND is a builtin word or `(type subtype [manufacturer])` |
//! | `format` | `NAME (SPEC)` |
//! | `wire` | `SRC DST [FORMAT]`, endpoints `NAME` or `(NAME PORT)`, FORMAT inline `(...)` or a name |
//! | `param` | `NAME DEFAULT [out]` |
//! | `wireGraphParam` | `NAME (BOX ID [SCOPE ELEMENT])` |
//! | `property` | `NAME in` |
//! | `wireGraphProperty` | `NAME (BOX ID SCOPE ELEMENT)` |
//! | `wireParam` / `wireProperty` | `(BOX ID ...) (BOX ID ...) [FLAG]` |
//!
//! A format spec is `([freq] RATE CHANNELS[i] [BLOCK])`: an optional
//! frequency-domain marker, the sample rate, the channel count (suffix `i`:
//! interleaved) and an optional fixed block size.
//!
//! Unknown keywords are errors: the reader never silently drops a statement.

use crate::aupreset::UnitId;
use crate::error::{Error, Result};
use crate::fourcc::FourCc;
use serde::Serialize;
use std::collections::BTreeMap;

pub mod analysis;
mod pre;

pub use pre::{MAX_EXPANDED_TOKENS, MAX_NESTING};

/// Largest graph file the reader accepts.  The shipped graphs are a few
/// kilobytes.
pub const MAX_GRAPH_BYTES: usize = 1 << 20;
/// Most boxes and wires a graph may declare.  The shipped graphs have tens.
pub const MAX_BOXES: usize = 1024;
pub const MAX_WIRES: usize = 4096;
/// Widest bus a wire may declare.  The engine's buses are far narrower;
/// the cap keeps the lane tracer from allocating per-lane state for a
/// hostile channel count.
pub const MAX_WIRE_CHANNELS: u32 = 256;
/// Longest statement text quoted in an error message.
const MAX_QUOTED: usize = 160;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub enum Expr {
    Word(String),
    Str(String),
    Group(Vec<Expr>),
    /// `[NAME]` with no definition and no substitution.
    Unresolved(String),
}

impl Expr {
    fn word(&self) -> Option<&str> {
        match self {
            Expr::Word(w) => Some(w),
            _ => None,
        }
    }
    fn number(&self) -> Option<f64> {
        self.word().and_then(|w| w.parse::<f64>().ok()).filter(|v| v.is_finite())
    }
    fn uint(&self) -> Option<u32> {
        self.word().and_then(|w| w.parse::<u32>().ok())
    }
}

/// A quantity that may depend on an unresolved macro.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(untagged)]
pub enum Num {
    Value(f64),
    Unresolved(String),
}

impl Num {
    pub fn value(&self) -> Option<f64> {
        match self {
            Num::Value(v) => Some(*v),
            Num::Unresolved(_) => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub enum BoxKind {
    /// `(type subtype [manufacturer])`: an AudioUnit.
    AudioUnit { kind: FourCc, subtype: FourCc, manufacturer: Option<FourCc> },
    /// A graph builtin (`split`, `join`, `dbgn`, `reblocker`, ...).
    Builtin(String),
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct GraphBox {
    pub name: String,
    pub kind: BoxKind,
    pub inputs: u32,
    pub outputs: u32,
    pub line: usize,
}

impl GraphBox {
    pub fn subtype(&self) -> Option<FourCc> {
        match self.kind {
            BoxKind::AudioUnit { subtype, .. } => Some(subtype),
            BoxKind::Builtin(_) => None,
        }
    }

    /// True if an effect's unit identity matches this box.
    pub fn matches_unit(&self, unit: &UnitId) -> bool {
        match self.kind {
            BoxKind::AudioUnit { kind, subtype, manufacturer } => {
                kind == unit.kind && subtype == unit.subtype && manufacturer.is_none_or(|m| m == unit.manufacturer)
            }
            BoxKind::Builtin(_) => false,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct FormatSpec {
    pub frequency_domain: bool,
    pub sample_rate: Num,
    pub channels: Num,
    pub interleaved: bool,
    pub block_size: Option<Num>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub enum WireFormat {
    Inline(FormatSpec),
    Named(String),
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct Endpoint {
    /// A box name, or a graph port declared with `in`/`out`.
    pub node: String,
    pub port: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Wire {
    pub src: Endpoint,
    pub dst: Endpoint,
    pub format: Option<WireFormat>,
    pub line: usize,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct GraphParam {
    pub name: String,
    pub default: f64,
    /// Declared `out` (published by the graph); `in` or nothing otherwise.
    pub output: bool,
}

/// `(BOX ID [SCOPE] [ELEMENT])`, the target of a parameter or property link.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Target {
    pub node: String,
    pub id: Num,
    pub rest: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Link {
    pub name: String,
    pub target: Target,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct BoxLink {
    pub from: Target,
    pub to: Target,
    pub flag: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Graph {
    pub name: Option<String>,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
    /// Boxes in declaration order (not processing order: see [`analysis`]).
    pub boxes: Vec<GraphBox>,
    pub formats: BTreeMap<String, FormatSpec>,
    pub wires: Vec<Wire>,
    pub params: Vec<GraphParam>,
    pub param_links: Vec<Link>,
    pub properties: Vec<(String, String)>,
    pub property_links: Vec<Link>,
    pub box_param_links: Vec<BoxLink>,
    pub box_property_links: Vec<BoxLink>,
    /// Macro values after definition and substitution, as text.
    pub macros: BTreeMap<String, String>,
    /// Macros referenced but never resolved.
    pub unresolved: Vec<String>,
    /// Non-fatal observations (macro redefinitions, substitution overridden).
    pub notes: Vec<String>,
}

impl Graph {
    pub fn find_box(&self, name: &str) -> Option<&GraphBox> {
        self.boxes.iter().find(|b| b.name == name)
    }

    pub fn is_port(&self, name: &str) -> bool {
        self.inputs.iter().chain(&self.outputs).any(|p| p == name)
    }

    /// The format a wire carries, with named formats resolved.
    pub fn wire_format<'a>(&'a self, w: &'a Wire) -> Option<&'a FormatSpec> {
        match &w.format {
            Some(WireFormat::Inline(f)) => Some(f),
            Some(WireFormat::Named(n)) => self.formats.get(n),
            None => None,
        }
    }

    /// Boxes whose AudioUnit subtype is `subtype`.
    pub fn boxes_of(&self, subtype: FourCc) -> impl Iterator<Item = &GraphBox> {
        self.boxes.iter().filter(move |b| b.subtype() == Some(subtype))
    }
}

use pre::Tok;

fn exprs(toks: &[Tok], line: usize, ctx: &str) -> Result<Vec<Expr>> {
    fn go(toks: &[Tok], i: &mut usize, depth: usize, line: usize, ctx: &str) -> Result<Vec<Expr>> {
        let mut out = Vec::new();
        while *i < toks.len() {
            let t = &toks[*i];
            *i += 1;
            match t {
                Tok::Word(w) => out.push(Expr::Word(w.clone())),
                Tok::Str(s) => out.push(Expr::Str(s.clone())),
                Tok::Unresolved(n) => out.push(Expr::Unresolved(n.clone())),
                Tok::Open => {
                    if depth >= MAX_NESTING {
                        return Err(Error::format(
                            ctx,
                            format!("line {line}: parentheses nested deeper than {MAX_NESTING}"),
                        ));
                    }
                    out.push(Expr::Group(go(toks, i, depth + 1, line, ctx)?))
                }
                Tok::Newline => {}
                Tok::Close => {
                    if depth == 0 {
                        return Err(Error::format(ctx, format!("line {line}: unbalanced )")));
                    }
                    return Ok(out);
                }
            }
        }
        if depth > 0 {
            return Err(Error::format(ctx, format!("line {line}: unbalanced (")));
        }
        Ok(out)
    }
    let mut i = 0;
    go(toks, &mut i, 0, line, ctx)
}

// ---------------------------------------------------------------- parsing

fn num(e: &Expr) -> Option<Num> {
    match e {
        Expr::Unresolved(n) => Some(Num::Unresolved(n.clone())),
        _ => e.number().map(Num::Value),
    }
}

fn format_spec(items: &[Expr]) -> Option<FormatSpec> {
    let (freq, rest) = match items.first() {
        Some(Expr::Word(w)) if w == "freq" => (true, &items[1..]),
        _ => (false, items),
    };
    if rest.len() < 2 || rest.len() > 3 {
        return None;
    }
    let sample_rate = num(&rest[0])?;
    let (channels, interleaved) = match &rest[1] {
        Expr::Word(w) if w.ends_with('i') => (Num::Value(w[..w.len() - 1].parse::<u32>().ok()? as f64), true),
        e => (num(e)?, false),
    };
    let block_size = match rest.get(2) {
        Some(e) => Some(num(e)?),
        None => None,
    };
    Some(FormatSpec { frequency_domain: freq, sample_rate, channels, interleaved, block_size })
}

fn endpoint(e: &Expr) -> Option<Endpoint> {
    match e {
        Expr::Word(w) => Some(Endpoint { node: w.clone(), port: 0 }),
        Expr::Group(g) => match g.as_slice() {
            [Expr::Word(w)] => Some(Endpoint { node: w.clone(), port: 0 }),
            [Expr::Word(w), p] => Some(Endpoint { node: w.clone(), port: p.uint()? }),
            _ => None,
        },
        _ => None,
    }
}

fn target(e: &Expr) -> Option<Target> {
    let Expr::Group(g) = e else { return None };
    let (node, id) = match g.as_slice() {
        [Expr::Word(n), id, ..] => (n.clone(), num(id)?),
        _ => return None,
    };
    let mut rest = Vec::new();
    for x in &g[2..] {
        match x {
            Expr::Word(w) => rest.push(w.clone()),
            Expr::Unresolved(u) => rest.push(format!("[{u}]")),
            _ => return None,
        }
    }
    Some(Target { node, id, rest })
}

fn fourcc(e: &Expr) -> Option<FourCc> {
    FourCc::from_str_exact(e.word()?)
}

/// Parses a graph.  `subst` supplies values for macros the file does not
/// define (the configuration's `Substitutions`).
pub fn parse(text: &str, subst: &BTreeMap<String, String>, ctx: &str) -> Result<Graph> {
    if text.len() > MAX_GRAPH_BYTES {
        return Err(Error::format(ctx, format!("{} bytes; a graph is at most {MAX_GRAPH_BYTES}", text.len())));
    }
    let pre = pre::run(text, subst, ctx)?;
    let mut g = Graph::default();
    for (line, toks) in &pre.statements {
        let line = *line;
        let raw_line = toks
            .iter()
            .map(|t| match t {
                Tok::Word(w) => w.clone(),
                Tok::Str(s) => format!("{s:?}"),
                Tok::Open => "(".into(),
                Tok::Close => ")".into(),
                Tok::Unresolved(u) => format!("[{u}]"),
                Tok::Newline => String::new(),
            })
            .collect::<Vec<_>>()
            .join(" ");
        let raw_line = match raw_line.char_indices().nth(MAX_QUOTED) {
            Some((cut, _)) => format!("{}...", &raw_line[..cut]),
            None => raw_line,
        };
        let ex = exprs(toks, line, ctx)?;
        let bad = |what: &str| Error::format(ctx, format!("line {line}: malformed {what}: {}", raw_line.trim()));
        let Some(Expr::Word(kw)) = ex.first() else { return Err(bad("statement")) };
        let a = &ex[1..];
        match kw.as_str() {
            "graphName" => match a {
                [Expr::Str(s)] if g.name.is_none() => g.name = Some(s.clone()),
                _ => return Err(bad("graphName")),
            },
            "in" | "out" => {
                let [Expr::Word(p)] = a else { return Err(bad(kw)) };
                if g.is_port(p) || g.find_box(p).is_some() {
                    return Err(bad("port (name already used)"));
                }
                if kw == "in" {
                    g.inputs.push(p.clone())
                } else {
                    g.outputs.push(p.clone())
                }
            }
            "box" => {
                let [Expr::Word(name), kind, i, o] = a else { return Err(bad("box")) };
                let kind = match kind {
                    Expr::Word(w) => BoxKind::Builtin(w.clone()),
                    Expr::Group(parts) => match parts.as_slice() {
                        [t, s] => BoxKind::AudioUnit {
                            kind: fourcc(t).ok_or_else(|| bad("box unit type"))?,
                            subtype: fourcc(s).ok_or_else(|| bad("box unit subtype"))?,
                            manufacturer: None,
                        },
                        [t, s, m] => BoxKind::AudioUnit {
                            kind: fourcc(t).ok_or_else(|| bad("box unit type"))?,
                            subtype: fourcc(s).ok_or_else(|| bad("box unit subtype"))?,
                            manufacturer: Some(fourcc(m).ok_or_else(|| bad("box unit manufacturer"))?),
                        },
                        _ => return Err(bad("box unit")),
                    },
                    _ => return Err(bad("box kind")),
                };
                if g.find_box(name).is_some() || g.is_port(name) {
                    return Err(bad("box (name already used)"));
                }
                if g.boxes.len() >= MAX_BOXES {
                    return Err(bad(&format!("box (more than {MAX_BOXES})")));
                }
                g.boxes.push(GraphBox {
                    name: name.clone(),
                    kind,
                    inputs: i.uint().ok_or_else(|| bad("box input count"))?,
                    outputs: o.uint().ok_or_else(|| bad("box output count"))?,
                    line,
                });
            }
            "format" => {
                let [Expr::Word(name), Expr::Group(spec)] = a else { return Err(bad("format")) };
                let spec = format_spec(spec).ok_or_else(|| bad("format spec"))?;
                if g.formats.insert(name.clone(), spec).is_some() {
                    return Err(bad("format (defined twice)"));
                }
            }
            "wire" => {
                let (src, dst, fmt) = match a {
                    [s, d] => (s, d, None),
                    [s, d, f] => (s, d, Some(f)),
                    _ => return Err(bad("wire")),
                };
                let format = match fmt {
                    None => None,
                    Some(Expr::Word(n)) => Some(WireFormat::Named(n.clone())),
                    Some(Expr::Group(spec)) => {
                        Some(WireFormat::Inline(format_spec(spec).ok_or_else(|| bad("wire format"))?))
                    }
                    Some(_) => return Err(bad("wire format")),
                };
                if g.wires.len() >= MAX_WIRES {
                    return Err(bad(&format!("wire (more than {MAX_WIRES})")));
                }
                g.wires.push(Wire {
                    src: endpoint(src).ok_or_else(|| bad("wire source"))?,
                    dst: endpoint(dst).ok_or_else(|| bad("wire destination"))?,
                    format,
                    line,
                });
            }
            "param" => {
                let (name, default, output) = match a {
                    [n, d] => (n, d, false),
                    [n, d, Expr::Word(o)] if o == "out" => (n, d, true),
                    [n, d, Expr::Word(o)] if o == "in" => (n, d, false),
                    _ => return Err(bad("param")),
                };
                g.params.push(GraphParam {
                    name: name.word().ok_or_else(|| bad("param name"))?.to_string(),
                    default: default.number().ok_or_else(|| bad("param default"))?,
                    output,
                });
            }
            "property" => {
                let [Expr::Word(n), Expr::Word(dir)] = a else { return Err(bad("property")) };
                g.properties.push((n.clone(), dir.clone()));
            }
            "wireGraphParam" | "wireGraphProperty" => {
                let [Expr::Word(n), t] = a else { return Err(bad(kw)) };
                let link = Link { name: n.clone(), target: target(t).ok_or_else(|| bad(kw))? };
                if kw == "wireGraphParam" {
                    g.param_links.push(link)
                } else {
                    g.property_links.push(link)
                }
            }
            "wireParam" | "wireProperty" => {
                let (f, t, flag) = match a {
                    [f, t] => (f, t, None),
                    [f, t, Expr::Word(w)] => (f, t, Some(w.clone())),
                    _ => return Err(bad(kw)),
                };
                let link =
                    BoxLink { from: target(f).ok_or_else(|| bad(kw))?, to: target(t).ok_or_else(|| bad(kw))?, flag };
                if kw == "wireParam" {
                    g.box_param_links.push(link)
                } else {
                    g.box_property_links.push(link)
                }
            }
            other => {
                return Err(Error::format(ctx, format!("line {line}: unknown statement {other:?}")));
            }
        }
    }
    // Every wire names known nodes and existing ports.
    for w in &g.wires {
        for (ep, is_src) in [(&w.src, true), (&w.dst, false)] {
            let ok = if let Some(b) = g.find_box(&ep.node) {
                ep.port < if is_src { b.outputs } else { b.inputs }
            } else if is_src {
                g.inputs.contains(&ep.node) && ep.port == 0
            } else {
                g.outputs.contains(&ep.node) && ep.port == 0
            };
            if !ok {
                return Err(Error::format(
                    ctx,
                    format!(
                        "line {}: wire {} {}:{} is not a {} port",
                        w.line,
                        if is_src { "source" } else { "destination" },
                        ep.node,
                        ep.port,
                        if is_src { "source" } else { "destination" }
                    ),
                ));
            }
        }
        if let Some(WireFormat::Named(n)) = &w.format {
            if !g.formats.contains_key(n) {
                return Err(Error::format(ctx, format!("line {}: wire uses undefined format {n}", w.line)));
            }
        }
    }
    // A destination port takes one wire.
    let mut dsts: Vec<&Endpoint> = g.wires.iter().map(|w| &w.dst).collect();
    dsts.sort();
    if let Some(d) = dsts.windows(2).find(|p| p[0] == p[1]) {
        return Err(Error::format(ctx, format!("{}:{} is fed by more than one wire", d[0].node, d[0].port)));
    }
    for l in g.param_links.iter().chain(&g.property_links) {
        if g.find_box(&l.target.node).is_none() {
            return Err(Error::format(ctx, format!("link {} targets unknown box {}", l.name, l.target.node)));
        }
    }
    g.macros = pre.macros;
    for (k, v) in subst {
        g.macros.entry(k.clone()).or_insert_with(|| v.clone());
    }
    g.unresolved = pre.unresolved;
    g.notes = pre.notes;
    Ok(g)
}
