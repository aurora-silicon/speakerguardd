// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The built-in microphone chain: selects the Mac's default capture graph
//! from its DSP database, reads the graph, its strip and property strip and
//! the files they name, and compiles it into `sgv2_core::mic::Params` and
//! the bound microphone blob (`sgv2_core::mic::artifact`), with a report of
//! every stage.
//!
//! **Selection.** The DSP database's `Built-in Microphone` device lists
//! configurations whose hardware (`Input[].Hardware[]`) names a graph, a
//! strip and a property strip. The chain is the configuration of the
//! database's own "General" kind (as for the speaker); every configuration
//! of that kind must name the same files, or nothing is selected.
//!
//! **Shape.** The graph must be one path from its input port to its output
//! port: every box on it feeds the next through its output 0 (a box's other
//! outputs may only end in `dead` boxes; a box with several inputs takes
//! the same source on each, as the beamformer takes the analysis spectrum
//! on both of its buses). The first `fft` box on the path is the analysis,
//! the second the synthesis, and exactly one beamformer sits between them,
//! or one fan-out: the analysis feeding several beamformers whose outputs
//! 0 meet in one low-frequency blend unit (`aufx/lwbo`, inputs beam,
//! comparison reference, blend reference) that continues the path.
//!
//! **Boxes**, by kind, with what the report calls them:
//!
//! | box | carried as | status |
//! |---|---|---|
//! | `aufx/dccb` | DC blocker: Butterworth high-pass at parameter 0 (Hz), gain parameter 2 (dB), linear start ramp over parameter 1 (ms) from the first nonzero block; steady state black-box exact, the output during the ramp TODO(SPEC) | approximate |
//! | `gain`, `dbgn` builtins | parameter 0: a linear factor (`gain`) or dB (`dbgn`); parameter 1 must be 0 | exact |
//! | `aufx/tmst` | the voicing chain's filter box (`crate::prestage::voicing_tone`), at its saved state; notch slots (type 6) by [`notch`] | exact |
//! | `aufx/dspg` (nested graph) | its graph, flattened ([`nested`]) | exact when every box in it is |
//! | `reblocker`, `dead` | nothing (structure) | structural |
//! | `fft` pair | the short-time transform, quadrant window, frame phase 0 (black-box exact) | exact |
//! | `aufx/bmfd` | the fixed beam `sum_m W_m(k) X_m(k)` of its property-4630 file, in fixed-beam mode ([`beam_mode`]; the adaptive blend is not engaged there, measured); any other mode refuses | exact |
//! | `aufx/sens` | the noise-floor suppressor (`sgv2_core::mic::sens`): the strip's global parameters and the property-1000 noise profile; artifact step fitted, TODO(SPEC) | approximate |
//! | `aufx/lwbo` | the low-frequency blend (`sgv2_core::mic::blend`) against the reference beams; law from the summary, unmeasured, TODO(SPEC) | approximate |
//! | `aufx/ocns` | identity, partial blobs only (documented transparent or attenuating) | uncharacterised |
//! | bypassed in the strip | nothing | bypassed |
//! | anything else | refused, partial or not | refused |
//!
//! **Refusal.** The default compile refuses, and writes no blob, when any
//! stage is uncharacterised or refused. `allow_partial` admits the
//! uncharacterised ones as identity and flags the blob partial; a refused
//! stage refuses always. No value is ever invented: a stage is the Mac's
//! data through a stated law, identity, or a refusal. Where the clean
//! behaviour summary leaves a law's detail open, an approximate stage uses
//! the closest reading of the summary, with any constant fitted to
//! black-box renders of the installed unit; its report lists each one as a
//! TODO(SPEC).
//!
//! **Binding.** As the speaker blob ([`crate::compile`]): the board, the
//! acoustic ID and, when known on both sides, the unit must be this Mac's.
//! The blob carries the board, the AID and the unit hash.

use crate::aupreset::ParamState;
use crate::bundle::Bundle;
use crate::dspg::{self, analysis, BoxKind, Graph, GraphBox, Num};
use crate::error::{Diagnostics, Error, Result};
use crate::machine::{Machine, Unit};
use crate::plist::{Dict, Value};
use crate::strip::{AuStrip, Effect, PropStrip, PropValue};
use crate::tuning::{bundle_path, Stage as TStage};
use serde::Serialize;
use sgv2_core::mic::{self, artifact, Beam, Params, Stage, Stft, Window};
use sgv2_core::prestage::Channel;
use sgv2_core::voicing::chain::{Kind, NodeSpec, Source, Spec, Tone};
use std::collections::BTreeMap;

pub const MIC_DEVICE: &str = "Built-in Microphone";
/// The beamformer's property carrying its coefficient file.
pub const PROP_BEAM_FILE: u32 = 4630;
/// The suppressor's property carrying its noise profile file.
pub const PROP_SENS_FILE: u32 = 1000;
/// The suppressor's processing mode (parameter 1) the black-box renders
/// cover: the unit's default (its parameter information's default, also its
/// maximum); any other refuses.
pub const SENS_MODE: f64 = 6.0;
/// The suppressor's autoregressive order. TODO(SPEC): fitted to black-box
/// renders (the summary names the estimator, not its order).
pub const SENS_ORDER: usize = 48;
/// A nested graph box's graph, property strip and AU strip properties.
pub const PROP_GRAPH: u32 = u32::from_be_bytes(*b"dspg");
pub const PROP_PROPSTRIP: u32 = u32::from_be_bytes(*b"pstp");
pub const PROP_AUSTRIP: u32 = u32::from_be_bytes(*b"ausp");
/// Unit kinds documented as transparent at normal levels or as attenuating
/// only (noise suppressors): the only ones a partial blob may carry as
/// identity.
pub const TRANSPARENT_OR_ATTENUATING: &[&str] = &["ocns"];

/// Compile options. The transform defaults are the best fit to black-box
/// renders so far (TODO(SPEC)); the others exist to test alternatives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Options {
    pub allow_partial: bool,
    pub window: Window,
    pub half_offset: bool,
    pub frame_phase: usize,
    /// Apply the beam's per-bin `EQSelection` table to its output.
    /// TODO(SPEC): its use in the unit; off by default.
    pub beam_eq: bool,
    /// Development analysis only: filter-box slot types (bit per type) to
    /// switch off before compiling. Any slot this drops adds a refusal, so
    /// no blob is ever written with it; the offline harness uses it to
    /// measure what an uncarried slot type does.
    pub dev_drop_tone_types: u64,
    /// The suppressor's autoregressive order (default [`SENS_ORDER`]).
    pub sens_order: usize,
    /// Development analysis only: replace the DC blocker's ramp time, ms
    /// (adds a refusal: never a blob).
    pub dev_dccb_ramp_ms: Option<f64>,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            allow_partial: false,
            window: Window::Quadrant,
            half_offset: false,
            frame_phase: 0,
            beam_eq: false,
            dev_drop_tone_types: 0,
            sens_order: SENS_ORDER,
            dev_dccb_ramp_ms: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum Status {
    Exact,
    Approximate,
    Structural,
    Bypassed,
    Uncharacterised,
    Refused,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StageReport {
    #[serde(rename = "box")]
    pub name: String,
    pub kind: String,
    /// `time` or `freq`.
    pub domain: &'static str,
    pub status: Status,
    pub detail: String,
    /// Open specification items this stage depends on.
    pub todo: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Report {
    pub aid: String,
    pub configuration: Option<String>,
    pub graph: String,
    pub strip: String,
    pub propstrip: Option<String>,
    pub sample_rate: u32,
    pub mics: usize,
    pub latency_frames: usize,
    pub partial: bool,
    /// No blob: why.
    pub refusals: Vec<String>,
    pub stages: Vec<StageReport>,
    pub notes: Vec<String>,
}

impl Report {
    fn stage(&mut self, name: &str, kind: &str, domain: &'static str, status: Status, detail: impl Into<String>) {
        self.stages.push(StageReport {
            name: name.into(),
            kind: kind.into(),
            domain,
            status,
            detail: detail.into(),
            todo: Vec::new(),
        });
    }
    fn todo(&mut self, item: &str) {
        if let Some(s) = self.stages.last_mut() {
            s.todo.push(item.into());
        }
    }
}

pub struct Compiled {
    /// The chain as far as it could be built: `None` when no chain could
    /// be (no transform or beam, parameters that do not validate). A chain
    /// whose report lists refusals is never encoded ([`compile_blob`]); it is
    /// returned only for offline analysis.
    pub params: Option<Params>,
    pub report: Report,
}

impl Compiled {
    /// No blob may be written.
    pub fn refused(&self) -> bool {
        !self.report.refusals.is_empty() || self.params.is_none()
    }
}

fn bad(msg: impl Into<String>) -> Error {
    Error::format("microphone chain", msg.into())
}

/// The selected configuration's files.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Selection {
    pub configuration: String,
    pub graph: String,
    pub strip: String,
    pub propstrip: Option<String>,
    pub substitutions: BTreeMap<String, String>,
}

/// The DSP database's entry for the bundle's AID: the file the manifest
/// names, else the shared one, else the per-AID one.
fn database(b: &Bundle) -> Result<Dict> {
    let per_aid = crate::manifest::aid_dsp_database(&b.manifest.aid);
    let path = match &b.manifest.dsp_database {
        Some(p) => p.clone(),
        None if b.file(crate::bundle::DSP_DATABASE).is_some() => crate::bundle::DSP_DATABASE.to_string(),
        None => per_aid,
    };
    let bytes = b.file(&path).ok_or_else(|| bad(format!("no DSP database ({path}) in the bundle")))?;
    let (v, _) = crate::plist::parse(bytes).map_err(|e| Error::plist(&path, e))?;
    match v.as_dict().and_then(|d| d.get(&b.manifest.aid)) {
        Some(Value::Dict(d)) => Ok(d.clone()),
        _ => Err(bad(format!("the DSP database has no {} entry", b.manifest.aid))),
    }
}

/// One microphone configuration: its `cdes` kind and the graph, strip and
/// property strip its `Input[].Hardware[]` names.
struct Configuration {
    name: String,
    cdes_kind: Option<i128>,
    graph: Option<String>,
    strip: Option<String>,
    propstrip: Option<String>,
    substitutions: BTreeMap<String, String>,
}

fn configuration(name: &str, d: &Dict, aid: &str, ctx: &str) -> Result<Configuration> {
    let cdes_kind = d
        .get("DSPDeviceProperties")
        .and_then(Value::as_array)
        .and_then(|l| {
            l.iter().filter_map(Value::as_dict).find(|x| x.get("Selector").and_then(Value::as_str) == Some("cdes"))
        })
        .and_then(|x| x.get("Value"))
        .and_then(Value::as_dict)
        .and_then(|k| k.get("kind"))
        .and_then(Value::as_integer);
    let mut c = Configuration {
        name: name.to_string(),
        cdes_kind,
        graph: None,
        strip: None,
        propstrip: None,
        substitutions: BTreeMap::new(),
    };
    for input in d.get("Input").and_then(Value::as_array).unwrap_or(&[]) {
        for h in input.as_dict().and_then(|x| x.get("Hardware")).and_then(Value::as_array).unwrap_or(&[]) {
            let h =
                h.as_dict().ok_or_else(|| Error::format(ctx, format!("{name}: Hardware entry is not a dictionary")))?;
            let ty = h.get("Type").and_then(Value::as_str).unwrap_or("");
            let slot = match ty {
                "DSPGraph" => &mut c.graph,
                "AUStrip" => &mut c.strip,
                "PropertyStrip" => &mut c.propstrip,
                _ => continue,
            };
            if slot.is_some() {
                return Err(Error::format(ctx, format!("{name}: more than one {ty}")));
            }
            *slot = h.get("Path").and_then(Value::as_str).map(str::to_string);
            if ty == "DSPGraph" {
                if let Some(Value::Dict(sub)) = h.get("Substitutions") {
                    for (k, v) in sub.iter() {
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
        if !p.starts_with(&format!("{aid}/")) || !crate::manifest::admitted(p) {
            return Err(Error::format(ctx, format!("{name}: {p} is outside {aid}/")));
        }
    }
    Ok(c)
}

/// A graph parameter wired by name into a nested graph box
/// (`wireGraphParam NAME (BOX CODE)`, CODE the nested graph's own
/// parameter), which the shared graph reader does not take.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct NamedLink {
    pub param: String,
    pub node: String,
    pub code: String,
}

/// Prepares a microphone graph's text for the shared reader: named
/// parameter links are taken out (returned, the line left as a comment so
/// line numbers hold) and an unquoted `graphName` is quoted.
pub fn prepare_graph(text: &str) -> (String, Vec<NamedLink>) {
    let mut links = Vec::new();
    let mut out = String::with_capacity(text.len() + 16);
    for line in text.lines() {
        let body = line.split(';').next().unwrap_or("");
        let toks: Vec<&str> = body.split_whitespace().collect();
        let tidy: Vec<String> = toks.iter().map(|t| t.trim_matches(|c| c == '(' || c == ')').to_string()).collect();
        match toks.first().copied() {
            Some("wireGraphParam")
                if tidy.len() == 4
                    && toks[2].starts_with('(')
                    && toks[3].ends_with(')')
                    && tidy[3].parse::<f64>().is_err()
                    && !tidy[3].starts_with('[')
                    && tidy[3].len() == 4
                    && tidy[3].bytes().all(|b| b.is_ascii_graphic()) =>
            {
                links.push(NamedLink { param: tidy[1].clone(), node: tidy[2].clone(), code: tidy[3].clone() });
                out.push_str("; ");
                out.push_str(line);
            }
            Some("graphName") if toks.len() == 2 && !toks[1].starts_with('"') => {
                out.push_str(&format!("graphName \"{}\"", toks[1]));
            }
            _ => out.push_str(line),
        }
        out.push('\n');
    }
    (out, links)
}

/// Chooses the default microphone chain from the DSP database.
pub fn select(b: &Bundle, diags: &mut Diagnostics) -> Result<Selection> {
    let ctx = "DSP database";
    let aid = &b.manifest.aid;
    let _ = diags;
    let db = database(b)?;
    let dev = match db.get(MIC_DEVICE) {
        Some(Value::Dict(d)) => d,
        _ => return Err(bad(format!("{aid} has no {MIC_DEVICE} device"))),
    };
    let mut all = Vec::new();
    for (name, v) in dev.iter() {
        if let Value::Dict(d) = v {
            all.push(configuration(name, d, aid, ctx)?);
        }
    }
    let general: Vec<i128> = all.iter().filter(|c| c.name.contains("General")).filter_map(|c| c.cdes_kind).collect();
    let kind = match general.first() {
        Some(k) if general.iter().all(|x| x == k) => *k,
        _ => return Err(bad("cannot identify the General microphone configuration kind")),
    };
    let picks: Vec<_> =
        all.iter().filter(|c| c.cdes_kind == Some(kind) && c.graph.is_some() && c.strip.is_some()).collect();
    let first = picks.first().ok_or_else(|| bad(format!("no {MIC_DEVICE} configuration of kind {kind}")))?;
    if let Some(other) = picks.iter().find(|c| {
        (&c.graph, &c.strip, &c.propstrip, &c.substitutions)
            != (&first.graph, &first.strip, &first.propstrip, &first.substitutions)
    }) {
        return Err(bad(format!(
            "configurations {} and {} of the General kind name different files",
            first.name, other.name
        )));
    }
    Ok(Selection {
        configuration: first.name.clone(),
        graph: first.graph.clone().unwrap(),
        strip: first.strip.clone().unwrap(),
        propstrip: first.propstrip.clone(),
        substitutions: first.substitutions.clone(),
    })
}

fn need<'a>(b: &'a Bundle, path: &str, what: &str) -> Result<&'a [u8]> {
    b.file(path).ok_or_else(|| Error::missing("bundle", &format!("{what} {path}")))
}

fn graph_text<'a>(b: &'a Bundle, path: &str) -> Result<&'a str> {
    std::str::from_utf8(need(b, path, "graph")?).map_err(|_| Error::format(path, "graph is not UTF-8 text"))
}

fn subtype(bx: &GraphBox) -> String {
    match &bx.kind {
        BoxKind::AudioUnit { subtype, .. } => subtype.to_string(),
        BoxKind::Builtin(w) => w.clone(),
    }
}

fn kind_name(bx: &GraphBox) -> String {
    match &bx.kind {
        BoxKind::AudioUnit { kind, subtype, manufacturer } => match manufacturer {
            Some(m) => format!("{kind}/{subtype}/{m}"),
            None => format!("{kind}/{subtype}"),
        },
        BoxKind::Builtin(w) => w.clone(),
    }
}

fn is_dead(g: &Graph, node: &str) -> bool {
    g.find_box(node).is_some_and(|b| matches!(&b.kind, BoxKind::Builtin(w) if w == "dead"))
}

fn parameter_box(bx: &GraphBox) -> bool {
    matches!(&bx.kind, BoxKind::Builtin(w) if w.starts_with("parameter_"))
}

fn wire_ch(g: &Graph, w: &dspg::Wire) -> Result<usize> {
    Ok(analysis::wire_channels(g, w)? as usize)
}

/// The global parameter `id` of a strip effect's saved state.
fn global(e: &Effect, id: u32) -> Option<f64> {
    e.preset.state.global().and_then(|g| g.get(id)).map(f64::from)
}

/// The tuning reader's stage record for a strip effect, so the filter-box
/// compiler can run on it.
fn tstage(bx: &GraphBox, e: Option<&Effect>, channels: usize, overrides: &[(u32, f64)]) -> TStage {
    let mut state = e.map(|e| e.preset.state.clone());
    if !overrides.is_empty() {
        let st = state.get_or_insert_with(ParamState::default);
        if st.global().is_none() {
            st.sections.insert(0, crate::aupreset::ParamSection { scope: 0, element: 0, params: Vec::new() });
        }
        let g = st.sections.iter_mut().find(|s| s.scope == 0 && s.element == 0).unwrap();
        for (id, v) in overrides {
            g.params.retain(|(i, _)| i != id);
            g.params.push((*id, *v as f32));
        }
    }
    TStage {
        name: bx.name.clone(),
        kind: kind_name(bx),
        unit: e.map(|e| e.unit),
        inputs: bx.inputs,
        outputs: bx.outputs,
        channels: Some(channels as u32),
        bypass: e.is_some_and(|e| e.bypass),
        preset_name: e.and_then(|e| e.preset.name.clone()),
        state,
        preset_extra_keys: e.map(|e| e.preset.extra_keys()).unwrap_or_default(),
        preset_extra: e.map(|e| e.preset.extra.clone()).unwrap_or_default(),
        strip_properties: Vec::new(),
        property_files: Vec::new(),
        before_protection: false,
    }
}

/// A second-order notch (band reject), bilinear and prewarped, at `f0`
/// with quality `q`: the filter box's slot type 6, which the shared filter
/// box law does not carry. Black-box renders of the installed unit (a
/// quality-10 notch at 13 kHz) match this design within 0.004 dB and
/// 0.0002 rad from 2 to 23 kHz.
pub fn notch(f0: f64, q: f64, fs: f64) -> sgv2_core::prestage::Section {
    let w0 = 2.0 * std::f64::consts::PI * f0 / fs;
    let alpha = w0.sin() / (2.0 * q);
    let c = w0.cos();
    let a0 = 1.0 + alpha;
    sgv2_core::prestage::Section {
        b0: 1.0 / a0,
        b1: -2.0 * c / a0,
        b2: 1.0 / a0,
        a1: -2.0 * c / a0,
        a2: (1.0 - alpha) / a0,
    }
}

/// Slot type 6 (see [`notch`]).
const NOTCH_SLOT: f64 = 6.0;

/// A filter box at its saved state (the shared filter-box law,
/// `crate::prestage::voicing_tone`, at the box's saved SystemVolume, as the
/// speaker voicing carries a box the graph does not drive), as chain nodes
/// `width` wide. Its running notch slots (type 6) are switched off for the
/// shared law and carried by a node of notches ahead of the box: static
/// sections commute, and the box's level-dependent slots, which detect its
/// whole static response, then see the notches too. A notch slot with a
/// gain, a group bit or volume automation is refused.
fn tone_nodes(bx: &GraphBox, e: &Effect, width: usize, overrides: &[(u32, f64)], fs: f64) -> Result<Vec<Kind>> {
    let base = tstage(bx, Some(e), width, overrides);
    let g = base.state.as_ref().and_then(|s| s.global()).cloned().unwrap_or(crate::aupreset::ParamSection {
        scope: 0,
        element: 0,
        params: Vec::new(),
    });
    let get = |id: u32| g.get(id).map(f64::from);
    let group_bypass = get(1).unwrap_or(0.0) as u32;
    let mut notches = vec![Vec::new(); width];
    let mut off = overrides.to_vec();
    for slot in 0..50u32 {
        let at = 14 + 10 * slot;
        let running = get(at).is_some_and(|st| (st as u32) & 2 != 0);
        if !running || get(at + 1) != Some(NOTCH_SLOT) {
            continue;
        }
        let (gain, f0, q) =
            (get(at + 2).unwrap_or(f64::NAN), get(at + 3).unwrap_or(f64::NAN), get(at + 4).unwrap_or(f64::NAN));
        let (mask, group) = (get(at + 7).unwrap_or(0.0), get(at + 9).unwrap_or(0.0));
        if gain != 0.0
            || q.is_nan()
            || q <= 0.0
            || !(f0 > 0.0 && f0 < fs / 2.0)
            || group.fract() != 0.0
            || !(0.0..10.0).contains(&group)
        {
            return Err(bad(format!(
                "filter box {}: notch slot {slot}: gain {gain}, frequency {f0}, quality {q}, group {group}",
                bx.name
            )));
        }
        if !crate::prestage::automation(&base).is_empty() {
            return Err(bad(format!("filter box {}: notch slot {slot} in a box with volume automation", bx.name)));
        }
        off.push((at, 0.0));
        if group_bypass >> (group as u32) & 1 != 0 {
            continue;
        }
        for (c, n) in notches.iter_mut().enumerate() {
            if (mask as u64) >> c & 1 != 0 {
                n.push(notch(f0, q, fs));
            }
        }
    }
    let st = tstage(bx, Some(e), width, &off);
    let mut d = Diagnostics::default();
    let (tone, partial) = crate::prestage::voicing_tone(&st, width, true, fs, &mut d)?;
    if !partial.is_empty() {
        return Err(bad(format!("filter box {}: volume automation entries {partial:?} not carried exactly", bx.name)));
    }
    let saved = get(3).unwrap_or(0.0);
    let mut out = Vec::new();
    if notches.iter().any(|n| !n.is_empty()) {
        let channels = notches
            .iter()
            .map(|n| {
                let mut ch = Channel::IDENTITY;
                ch.count = n.len();
                ch.sections[..n.len()].copy_from_slice(n);
                ch
            })
            .collect();
        out.push(Kind::Tone(Tone {
            channels,
            automation: Vec::new(),
            dynamic: Vec::new(),
            dynamic_automation: Vec::new(),
        }));
    }
    out.push(Kind::Tone(tone.at_volume(fs, saved)));
    Ok(out)
}

/// Kinds in series, each `width` wide, as one chain.
fn series(kinds: Vec<Kind>, width: usize) -> Spec {
    let mut nodes: Vec<NodeSpec> = Vec::new();
    for k in kinds {
        let inputs = match nodes.len() {
            0 => (0..width).map(Source::Program).collect(),
            n => (0..width).map(|c| Source::Node(n - 1, c)).collect(),
        };
        nodes.push(NodeSpec { kind: k, inputs });
    }
    let last = nodes.len() - 1;
    Spec {
        program_channels: width,
        nodes,
        outputs: (0..width).map(|c| Source::Node(last, c)).collect(),
        links: Vec::new(),
        relays: Vec::new(),
        volume: Default::default(),
    }
}

/// Development analysis: overrides switching off the running slots of the
/// types in `mask` (slot `i` state at ID `14 + 10 i`, type at `15 + 10 i`).
fn dev_drops(e: &Effect, mask: u64) -> Vec<(u32, f64)> {
    let mut out = Vec::new();
    if mask == 0 {
        return out;
    }
    for slot in 0..50u32 {
        let (state, kind) = (global(e, 14 + 10 * slot), global(e, 15 + 10 * slot));
        if let (Some(st), Some(k)) = (state, kind) {
            if (st as u32) & 2 != 0 && (0.0..64.0).contains(&k) && (mask >> (k as u32)) & 1 != 0 {
                out.push((14 + 10 * slot, 0.0));
            }
        }
    }
    out
}

/// A gain builtin's value in dB and its parameter 1. The `dbgn` builtin's
/// parameter 0 is in dB; the `gain` builtin's is a linear factor (measured:
/// whole-graph renders differ from a dB reading by exactly
/// `p0 - 20 log10 p0` dB on two tunings), which must be positive.
fn gain_db(name: &str, builtin: &str, e: Option<&Effect>, overrides: &[(u32, f64)]) -> Result<(f64, bool)> {
    let get =
        |id: u32| overrides.iter().find(|(i, _)| *i == id).map(|(_, v)| *v).or_else(|| e.and_then(|e| global(e, id)));
    let p0 = get(0).ok_or_else(|| bad(format!("gain box {name}: no parameter 0 in its saved state")))?;
    let p1 = get(1).unwrap_or(0.0);
    let db = match builtin {
        "gain" if p0 > 0.0 && p0.is_finite() => 20.0 * p0.log10(),
        "gain" => return Err(bad(format!("gain box {name}: linear factor {p0}"))),
        _ => p0,
    };
    if !db.is_finite() || db > mic::MAX_GAIN_DB {
        return Err(bad(format!("gain box {name}: {db} dB")));
    }
    Ok((db, p1 != 0.0))
}

/// The files a property strip names on a box, mapped into the bundle.
fn box_path(b: &Bundle, ps: Option<&PropStrip>, name: &str, number: u32) -> Result<Option<String>> {
    let Some(pb) = ps.and_then(|p| p.get(name)) else { return Ok(None) };
    let Some(p) = pb.properties.iter().find(|p| p.number == number) else { return Ok(None) };
    let path = match &p.value {
        PropValue::Path(s) => s.clone(),
        PropValue::Other { value: Value::String(s), .. } => s.clone(),
        other => return Err(bad(format!("box {name} property {number}: not a path ({other:?})"))),
    };
    Ok(Some(bundle_path(b, &path)?))
}

fn plist_dict(b: &Bundle, path: &str) -> Result<Dict> {
    let (v, _) = crate::plist::parse(need(b, path, "property file")?).map_err(|e| Error::plist(path, e))?;
    v.as_dict().cloned().ok_or_else(|| Error::format(path, "root is not a dictionary"))
}

fn be_f32s(v: Option<&Value>, n: usize, what: &str) -> Result<Vec<f64>> {
    let d = v.and_then(Value::as_data).ok_or_else(|| bad(format!("beam file: {what} missing or not data")))?;
    if d.len() != 4 * n {
        return Err(bad(format!("beam file: {what} has {} bytes, not {}", d.len(), 4 * n)));
    }
    let out: Vec<f64> = d.chunks_exact(4).map(|c| f64::from(f32::from_be_bytes([c[0], c[1], c[2], c[3]]))).collect();
    if out.iter().any(|v| !v.is_finite()) {
        return Err(bad(format!("beam file: {what} is not finite")));
    }
    Ok(out)
}

fn int(d: &Dict, key: &str) -> Result<i128> {
    d.get(key)
        .and_then(Value::as_number)
        .filter(|v| v.fract() == 0.0)
        .map(|v| v as i128)
        .ok_or_else(|| bad(format!("beam file: {key} missing or not an integer")))
}

/// The beam of a beamformer's coefficient file: beam 0's complex weights
/// per microphone and bin. The file must describe one beam over `mics`
/// microphones at `bins` bins and the graph's rate.
pub fn beam(file: &Dict, mics: usize, bins: usize, fs: u32, use_eq: bool, notes: &mut Vec<String>) -> Result<Beam> {
    let n_beams = int(file, "nBeams")?;
    let n_mics = int(file, "nMicsUsed")?;
    let n_coef = int(file, "nCoefficients")?;
    let rate = int(file, "SampleRate")?;
    if n_beams != 1 {
        // TODO(SPEC): the unit's beam selection among several beams
        return Err(bad(format!("beam file: {n_beams} beams; only a single fixed beam is characterised")));
    }
    if n_mics as usize != mics || n_coef as usize != bins || rate != fs as i128 {
        return Err(bad(format!(
            "beam file: {n_mics} microphones, {n_coef} coefficients at {rate} Hz; the graph has {mics} at {bins} bins, {fs} Hz"
        )));
    }
    let b0 = match file.get("Beam0") {
        Some(Value::Dict(d)) => d,
        _ => return Err(bad("beam file: no Beam0")),
    };
    let real = match b0.get("RealCoefficients") {
        Some(Value::Dict(d)) => d,
        _ => return Err(bad("beam file: Beam0 has no RealCoefficients")),
    };
    let imag = match b0.get("ImagCoefficients") {
        Some(Value::Dict(d)) => d,
        _ => return Err(bad("beam file: Beam0 has no ImagCoefficients")),
    };
    let nonzero = be_f32s(b0.get("NonZeroMics"), mics, "NonZeroMics")?;
    let mut weights = Vec::with_capacity(mics * bins);
    for (m, nz) in nonzero.iter().enumerate() {
        let key = format!("Mic{m}");
        let re = be_f32s(real.get(&key), bins, &format!("RealCoefficients.{key}"))?;
        let im = be_f32s(imag.get(&key), bins, &format!("ImagCoefficients.{key}"))?;
        if *nz == 0.0 {
            // a microphone the beam marks unused contributes nothing
            if re.iter().chain(&im).any(|v| *v != 0.0) {
                notes.push(format!("beam: {key} is marked unused but has nonzero weights; it is left out"));
            }
            weights.extend(std::iter::repeat_n([0.0, 0.0], bins));
        } else {
            weights.extend(re.iter().zip(&im).map(|(r, i)| [*r, *i]));
        }
    }
    let eq = if use_eq { Some(be_f32s(b0.get("EQSelection"), bins, "EQSelection")?) } else { None };
    Ok(Beam { mics, bins, weights, eq })
}

/// The beamformer's fixed-beam mode: the automatic-selection switches
/// (controls 1, 3, 4, 5), the selection-metric normalisations (21, 22),
/// boost limiting (23) and the uplink switch crossfade (42) off, the fixed
/// beam (control 2) on. In this mode black-box renders of the unit equal
/// the fixed beam of its coefficient file (coherent, single-microphone,
/// independent and wind-like stimuli, over 130 dB): its adaptive blend
/// (controls 28..41) is not engaged. `Err` names what differs.
pub fn beam_mode(e: Option<&Effect>) -> std::result::Result<(), String> {
    let e = e.ok_or_else(|| "no saved state in the strip".to_string())?;
    let want: [(u32, f64); 9] =
        [(1, 0.0), (2, 1.0), (3, 0.0), (4, 0.0), (5, 0.0), (21, 0.0), (22, 0.0), (23, 0.0), (42, 0.0)];
    for (id, v) in want {
        match global(e, id) {
            Some(x) if x == v => {}
            Some(x) => return Err(format!("control {id} is {x}: only the fixed-beam mode is characterised")),
            None => return Err(format!("control {id} missing from the saved state")),
        }
    }
    Ok(())
}

fn need_param(e: &Effect, id: u32, what: &str) -> Result<f64> {
    global(e, id)
        .filter(|v| v.is_finite())
        .ok_or_else(|| bad(format!("{what}: parameter {id} missing from the saved state")))
}

/// The suppressor of a strip effect (global scope: black-box renders
/// follow the global scope, not the per-element one) and its noise profile
/// file, for `mics` microphones in blocks of `hop` at `fs`.
pub fn sens(e: &Effect, file: &Dict, mics: usize, hop: usize, fs: u32, order: usize) -> Result<mic::sens::Sens> {
    let w = "suppressor";
    let get = |id| need_param(e, id, w);
    if get(1)? != SENS_MODE {
        return Err(bad(format!("{w}: processing mode {} is not characterised", get(1)?)));
    }
    if get(2)? != 0.0 {
        return Err(bad(format!("{w}: PSD estimator {} is not characterised", get(2)?)));
    }
    if get(8)? != 0.0 {
        return Err(bad(format!("{w}: noise profile learning is on")));
    }
    let num = |k: &str| file.get(k).and_then(Value::as_number);
    if num("SampleRate") != Some(fs as f64)
        || num("BlockSize") != Some(hop as f64)
        || num("NumberOfChannels") != Some(mics as f64)
    {
        return Err(bad(format!(
            "{w}: profile file for {:?} Hz, {:?} frames, {:?} channels; the graph has {fs}, {hop}, {mics}",
            num("SampleRate"),
            num("BlockSize"),
            num("NumberOfChannels")
        )));
    }
    let bins = hop + 1;
    let mut profile = Vec::with_capacity(mics * bins);
    for m in 0..mics {
        let key = format!("NoiseProfile{m}");
        let v = be_f32s(file.get(&key), bins, &key)?;
        if v.iter().any(|x| *x < 0.0) {
            return Err(bad(format!("{w}: {key} has negative power")));
        }
        profile.extend(v);
    }
    let min_len = get(12)?;
    if min_len.fract() != 0.0 || min_len < 1.0 || min_len as usize > hop {
        return Err(bad(format!("{w}: minimum length {min_len}")));
    }
    let s = mic::sens::Sens {
        mics,
        hop,
        bypass: get(0)? != 0.0,
        order,
        signal_ms: get(3)?,
        gain_ms: get(4)?,
        output_db: get(5)?,
        floor_db: get(6)?,
        multiplier: get(7)?,
        artifact: get(9)? != 0.0,
        threshold: get(10)?,
        scale: get(11)?,
        min_len: min_len as usize,
        aggressive: get(13)? != 0.0,
        profile,
    };
    if !s.valid(mics, fs as f64) {
        return Err(bad(format!("{w}: parameters out of range")));
    }
    Ok(s)
}

/// The low-frequency blend unit's law from its saved state (IDs 0..17).
/// Its reference channel (ID 0) must name input 1, the comparison
/// reference (TODO(SPEC): its meaning).
pub fn lwbo_law(e: &Effect, fs: f64) -> Result<mic::blend::BlendLaw> {
    let w = "low-frequency blend";
    let get = |id| need_param(e, id, w);
    if get(0)? != 1.0 {
        return Err(bad(format!("{w}: reference channel {} is not characterised", get(0)?)));
    }
    let l = mic::blend::BlendLaw {
        boost_limit: true,
        cutoff_hz: get(1)?,
        detect_db: get(2)?,
        max_boost_db: get(3)?,
        scale_floor_db: get(5)?,
        scale_smooth_s: get(6)?,
        threshold_db: get(7)?,
        xover_max_hz: get(8)?,
        attack_s: get(9)?,
        release_s: get(10)?,
        hold_s: get(11)?,
        xover_smooth_s: get(12)?,
        xover_min_hz: get(13)?,
        abs_floor_db: get(14)?,
        wind_attack_s: get(15)?,
        wind_release_s: get(16)?,
        slope: get(17)?,
    };
    if !l.valid(fs) {
        return Err(bad(format!("{w}: parameters out of range")));
    }
    Ok(l)
}

/// Static values of a graph's parameter logic: every parameter box's
/// outputs and every audio box parameter the graph drives, at the graph
/// parameters' values `params` (its defaults, with what the parent wires).
/// Parameter boxes: `parameter_add` (sum of inputs), `parameter_multiply`
/// (product), `parameter_logical_not` (1 when input 0 is 0, else 0).
pub fn static_params(g: &Graph, params: &BTreeMap<String, f64>) -> Result<BTreeMap<String, Vec<(u32, f64)>>> {
    let mut inputs: BTreeMap<String, BTreeMap<u32, f64>> = BTreeMap::new();
    let mut outputs: BTreeMap<String, f64> = BTreeMap::new();
    let mut audio: BTreeMap<String, Vec<(u32, f64)>> = BTreeMap::new();
    let slot = |rest: &[String]| -> Option<u32> {
        match rest {
            [d, k] if d == "input" => k.parse().ok(),
            _ => None,
        }
    };
    let id_of = |t: &dspg::Target| -> Result<u32> {
        match &t.id {
            Num::Value(v) if *v >= 0.0 && v.fract() == 0.0 => Ok(*v as u32),
            other => Err(bad(format!("parameter link to {}: id {other:?}", t.node))),
        }
    };
    for l in &g.param_links {
        let v = *params.get(&l.name).ok_or_else(|| bad(format!("graph parameter {} is not declared", l.name)))?;
        let bx = g.find_box(&l.target.node).ok_or_else(|| bad(format!("link to unknown box {}", l.target.node)))?;
        if parameter_box(bx) {
            let k = slot(&l.target.rest).ok_or_else(|| bad(format!("link into {}: {:?}", bx.name, l.target.rest)))?;
            inputs.entry(bx.name.clone()).or_default().insert(k, v);
        } else {
            audio.entry(bx.name.clone()).or_default().push((id_of(&l.target)?, v));
        }
    }
    let boxes: Vec<&GraphBox> = g.boxes.iter().filter(|b| parameter_box(b)).collect();
    for _ in 0..=boxes.len() {
        for bx in &boxes {
            if outputs.contains_key(&bx.name) {
                continue;
            }
            // inputs from other parameter boxes, once they are known
            let mut ready = true;
            for l in g.box_param_links.iter().filter(|l| l.to.node == bx.name) {
                match outputs.get(&l.from.node) {
                    Some(v) => {
                        let k =
                            slot(&l.to.rest).ok_or_else(|| bad(format!("link into {}: {:?}", bx.name, l.to.rest)))?;
                        inputs.entry(bx.name.clone()).or_default().insert(k, *v);
                    }
                    None => ready = false,
                }
            }
            if !ready {
                continue;
            }
            let ins: Vec<f64> = inputs.get(&bx.name).map(|m| m.values().copied().collect()).unwrap_or_default();
            let v = match &bx.kind {
                BoxKind::Builtin(w) if w == "parameter_add" => ins.iter().sum(),
                BoxKind::Builtin(w) if w == "parameter_multiply" => ins.iter().product(),
                BoxKind::Builtin(w) if w == "parameter_logical_not" => {
                    if ins.first().copied().unwrap_or(0.0) == 0.0 {
                        1.0
                    } else {
                        0.0
                    }
                }
                _ => {
                    return Err(bad(format!("parameter box {} of kind {}: not characterised", bx.name, kind_name(bx))))
                }
            };
            outputs.insert(bx.name.clone(), v);
        }
    }
    if let Some(bx) = boxes.iter().find(|b| !outputs.contains_key(&b.name)) {
        return Err(bad(format!("parameter box {} never resolves (a cycle or a missing input)", bx.name)));
    }
    for l in &g.box_param_links {
        let Some(bx) = g.find_box(&l.to.node) else { return Err(bad(format!("link to unknown box {}", l.to.node))) };
        if parameter_box(bx) {
            continue;
        }
        let v = *outputs
            .get(&l.from.node)
            .ok_or_else(|| bad(format!("parameter link from {}: not a parameter box", l.from.node)))?;
        audio.entry(bx.name.clone()).or_default().push((id_of(&l.to)?, v));
    }
    Ok(audio)
}

/// What [`nested`] flattens: the nested graph, its strips, the parent's
/// parameters wired into it by name, the parent wire's width and the rate.
#[derive(Clone, Copy)]
pub struct Nest<'a> {
    pub graph: &'a Graph,
    pub strip: &'a AuStrip,
    pub props: Option<&'a PropStrip>,
    pub parent_params: &'a BTreeMap<String, f64>,
    pub width: usize,
    pub fs: f64,
    /// See [`Options::dev_drop_tone_types`]; `dropped` lists what it hit.
    pub dev_drop_tone_types: u64,
}

/// A nested graph (`aufx/dspg` box) flattened into one voicing chain: its
/// filter boxes, `split`/`join` routing, `asum` sums and dB gains, at the
/// static values of its parameter logic. `width` is the parent's wire.
/// Any other box, or a box whose saved state cannot be carried, refuses.
pub fn nested(n: &Nest<'_>, notes: &mut Vec<String>, dropped: &mut Vec<String>) -> Result<Spec> {
    let Nest { graph: g, strip, props, parent_params, width, fs, dev_drop_tone_types } = *n;
    let mut params: BTreeMap<String, f64> = g.params.iter().map(|p| (p.name.clone(), p.default)).collect();
    for (k, v) in parent_params {
        if params.contains_key(k) {
            params.insert(k.clone(), *v);
        }
    }
    let overrides = static_params(g, &params)?;
    let order = analysis::processing_order(g)?;
    let (Some(input), Some(output)) = (g.inputs.first(), g.outputs.first()) else {
        return Err(bad("nested graph without an input or output port"));
    };
    if g.inputs.len() != 1 || g.outputs.len() != 1 {
        return Err(bad("nested graph with several input or output ports"));
    }
    // channels each box output port carries, as chain sources
    let mut ports: BTreeMap<(String, u32), Vec<Source>> = BTreeMap::new();
    ports.insert((input.clone(), 0), (0..width).map(Source::Program).collect());
    let mut nodes: Vec<NodeSpec> = Vec::new();
    let source_of = |ports: &BTreeMap<(String, u32), Vec<Source>>, node: &str, port: u32| -> Result<Vec<Source>> {
        let w = g
            .wires
            .iter()
            .find(|w| w.dst.node == node && w.dst.port == port)
            .ok_or_else(|| bad(format!("nested graph: {node} input {port} is not wired")))?;
        let src = ports.get(&(w.src.node.clone(), w.src.port)).ok_or_else(|| {
            bad(format!("nested graph: {}:{} feeds {node} before it is computed", w.src.node, w.src.port))
        })?;
        if wire_ch(g, w)? != src.len() {
            return Err(bad(format!(
                "nested graph: wire into {node}:{port} is {} wide, its source {}",
                wire_ch(g, w)?,
                src.len()
            )));
        }
        Ok(src.clone())
    };
    for name in &order.order {
        let bx = g.find_box(name).unwrap();
        if parameter_box(bx) {
            continue;
        }
        let ins: Vec<Vec<Source>> = (0..bx.inputs).map(|p| source_of(&ports, name, p)).collect::<Result<_>>()?;
        let e = strip.effect(name);
        let ov = overrides.get(name).cloned().unwrap_or_default();
        let sub = subtype(bx);
        let mut out_port = |p: u32, v: Vec<Source>| ports.insert((name.clone(), p), v);
        match sub.as_str() {
            "split" => {
                let all = ins.concat();
                let mut at = 0;
                for p in 0..bx.outputs {
                    let widths: Vec<usize> = g
                        .wires
                        .iter()
                        .filter(|w| w.src.node == *name && w.src.port == p)
                        .map(|w| wire_ch(g, w))
                        .collect::<Result<_>>()?;
                    let Some(&w) = widths.first() else { continue };
                    if widths.iter().any(|x| *x != w) || at + w > all.len() {
                        return Err(bad(format!("nested graph: split {name} output {p} widths {widths:?}")));
                    }
                    out_port(p, all[at..at + w].to_vec());
                    at += w;
                }
            }
            "join" => {
                out_port(0, ins.concat());
            }
            "asum" | "sum" => {
                let w = ins.first().map_or(0, Vec::len);
                if w == 0 || ins.iter().any(|i| i.len() != w) {
                    return Err(bad(format!("nested graph: sum {name} inputs of unequal width")));
                }
                nodes.push(NodeSpec { kind: Kind::Sum(w), inputs: ins.concat() });
                let n = nodes.len() - 1;
                out_port(0, (0..w).map(|c| Source::Node(n, c)).collect());
            }
            "dbgn" | "gain" => {
                let (db, mute) = gain_db(name, &sub, e, &ov)?;
                if e.is_some_and(|e| e.bypass) {
                    out_port(0, ins.concat());
                    continue;
                }
                let gain = if mute { 0.0 } else { 10f64.powf(db / 20.0) };
                let x = ins.concat();
                let mut ch = Channel::IDENTITY;
                ch.gain = gain;
                nodes.push(NodeSpec {
                    kind: Kind::Tone(Tone {
                        channels: vec![ch; x.len()],
                        automation: Vec::new(),
                        dynamic: Vec::new(),
                        dynamic_automation: Vec::new(),
                    }),
                    inputs: x.clone(),
                });
                let n = nodes.len() - 1;
                out_port(0, (0..x.len()).map(|c| Source::Node(n, c)).collect());
                if let Some(pb) = props.and_then(|p| p.get(name)) {
                    notes.push(format!(
                        "nested gain {name}: properties {:?} not applied (the gain is constant: its ramp never runs)",
                        pb.properties.iter().map(|p| p.number).collect::<Vec<_>>()
                    ));
                }
            }
            "tmst" => {
                let e = e.ok_or_else(|| bad(format!("nested filter box {name} has no preset in its strip")))?;
                let x = ins.concat();
                if e.bypass {
                    out_port(0, x);
                    continue;
                }
                let mut ov = ov.clone();
                let d = dev_drops(e, dev_drop_tone_types);
                if !d.is_empty() {
                    dropped
                        .push(format!("{name} slots {:?}", d.iter().map(|(id, _)| (id - 14) / 10).collect::<Vec<_>>()));
                    ov.extend(d);
                }
                let mut src = x.clone();
                for kind in tone_nodes(bx, e, x.len(), &ov, fs)? {
                    nodes.push(NodeSpec { kind, inputs: src });
                    let n = nodes.len() - 1;
                    src = (0..x.len()).map(|c| Source::Node(n, c)).collect();
                }
                out_port(0, src);
            }
            _ => return Err(bad(format!("nested graph: box {name} ({}) is not characterised", kind_name(bx)))),
        }
    }
    let outputs = source_of(&ports, output, 0)?;
    if outputs.len() != width {
        return Err(bad(format!("nested graph: {} output channels for {width} in", outputs.len())));
    }
    let spec = Spec {
        program_channels: width,
        nodes,
        outputs,
        links: Vec::new(),
        relays: Vec::new(),
        volume: Default::default(),
    };
    spec.check().map_err(|e| bad(format!("nested graph: {e:?}")))?;
    Ok(spec)
}

/// The black-box exact transform: the quadrant window at frame phase 0.
fn transform_exact(o: &Options) -> bool {
    o.window == Window::Quadrant && !o.half_offset && o.frame_phase == 0
}

/// A beamformer box's fixed beam: its mode, its coefficient file.
#[allow(clippy::too_many_arguments)]
fn beamformer(
    b: &Bundle,
    props: Option<&PropStrip>,
    name: &str,
    e: Option<&Effect>,
    mics: usize,
    stft: Option<Stft>,
    fs: u32,
    opts: &Options,
    notes: &mut Vec<String>,
) -> Result<Beam> {
    if e.is_some_and(|e| e.bypass) {
        return Err(bad(format!("{name}: a bypassed beamformer is not characterised")));
    }
    beam_mode(e).map_err(|m| bad(format!("{name}: {m}")))?;
    let bins = stft.map(|s| s.bins()).unwrap_or(0);
    let p = box_path(b, props, name, PROP_BEAM_FILE)?
        .ok_or_else(|| bad(format!("{name}: the property strip names no coefficient file")))?;
    beam(&plist_dict(b, &p)?, mics, bins, fs, opts.beam_eq, notes)
}

struct FanCtx<'a> {
    b: &'a Bundle,
    g: &'a Graph,
    strip: &'a AuStrip,
    props: Option<&'a PropStrip>,
    opts: &'a Options,
    fs: u32,
    stft: Option<Stft>,
}

struct Fan {
    beam: Beam,
    refs: Vec<Beam>,
    stage: mic::FreqStage,
    visited: Vec<String>,
    last: String,
}

/// The analysis `node` feeding several beamformers that meet in one
/// low-frequency blend unit: its input 0 the main beam (the beamformer's
/// output 0), inputs 1 and 2 the comparison and blend references (output 0
/// of other beamformers, each a plain fixed beam of its own file).
fn fan_out(c: &FanCtx<'_>, node: &str, r: &mut Report) -> Result<Fan> {
    let g = c.g;
    let live: Vec<&dspg::Wire> = g.wires.iter().filter(|w| w.src.node == node && !is_dead(g, &w.dst.node)).collect();
    let mut beams: Vec<String> = live.iter().map(|w| w.dst.node.clone()).collect();
    beams.sort();
    beams.dedup();
    let mics = wire_ch(g, live[0])?;
    let mut blend: Option<String> = None;
    for name in &beams {
        let bx = g.find_box(name).unwrap();
        if subtype(bx) != "bmfd" {
            return Err(bad(format!("{node} feeds {name} ({}) beside other boxes", kind_name(bx))));
        }
        let ins = live.iter().filter(|w| w.dst.node == *name).count();
        if ins as u32 != bx.inputs {
            return Err(bad(format!("{name}: {ins} of its {} inputs come from {node}", bx.inputs)));
        }
        for w in g.wires.iter().filter(|w| w.src.node == *name && !is_dead(g, &w.dst.node)) {
            let dst = g.find_box(&w.dst.node).ok_or_else(|| bad(format!("{name} feeds an unknown box")))?;
            if w.src.port != 0 || subtype(dst) != "lwbo" {
                return Err(bad(format!(
                    "{name} output {} feeds {} ({}): only a blend unit may join the beams",
                    w.src.port,
                    dst.name,
                    kind_name(dst)
                )));
            }
            if wire_ch(g, w)? != 1 {
                return Err(bad(format!("{name}: output 0 is not one channel")));
            }
            match &blend {
                Some(l) if *l != dst.name => return Err(bad("the beams meet in more than one blend unit")),
                _ => blend = Some(dst.name.clone()),
            }
        }
    }
    let lname = blend.ok_or_else(|| bad(format!("{node} feeds several beamformers that never meet")))?;
    let lbox = g.find_box(&lname).unwrap();
    let source = |port: u32| -> Result<String> {
        let ws: Vec<_> = g.wires.iter().filter(|w| w.dst.node == lname && w.dst.port == port).collect();
        match ws.as_slice() {
            [w] if beams.contains(&w.src.node) => Ok(w.src.node.clone()),
            _ => Err(bad(format!("{lname} input {port} is not one beamformer output"))),
        }
    };
    if lbox.inputs != 3 {
        return Err(bad(format!("{lname}: {} inputs, not 3", lbox.inputs)));
    }
    let (main, cmp, bl) = (source(0)?, source(1)?, source(2)?);
    if main == cmp || main == bl {
        return Err(bad(format!("{lname}: the main beam is also a reference")));
    }
    let mut out = Vec::new();
    let mut mk = |name: &str, r: &mut Report| -> Result<Beam> {
        let e = c.strip.effect(name);
        let bx = g.find_box(name).unwrap();
        let bm = beamformer(c.b, c.props, name, e, mics, c.stft, c.fs, c.opts, &mut r.notes)?;
        r.stage(
            name,
            &kind_name(bx),
            "freq",
            if c.opts.beam_eq && name == main { Status::Approximate } else { Status::Exact },
            format!("fixed beam over {mics} microphones, fixed-beam mode (adaptive blend not engaged)"),
        );
        out.push(name.to_string());
        Ok(bm)
    };
    let beam = mk(&main, r)?;
    let mut refs = vec![{
        let mut x = mk(&cmp, r)?;
        x.eq = None;
        x
    }];
    let blend_idx = if bl == cmp {
        0
    } else {
        let mut x = mk(&bl, r)?;
        x.eq = None;
        refs.push(x);
        1
    };
    let e = c.strip.effect(&lname).ok_or_else(|| bad(format!("{lname}: no preset in the strip")))?;
    if e.bypass {
        return Err(bad(format!("{lname}: a bypassed blend unit is not characterised")));
    }
    let law = lwbo_law(e, c.fs as f64)?;
    r.stage(
        &lname,
        &kind_name(lbox),
        "freq",
        Status::Approximate,
        format!(
            "low-frequency blend: beam {main}, comparison {cmp}, blend reference {bl}; boost limit under {} Hz, crossover {}..{} Hz",
            law.cutoff_hz, law.xover_min_hz, law.xover_max_hz
        ),
    );
    r.todo("lwbo law from the clean summary only (target search, hold/smoother order, crossfade width); no black-box renders");
    if !beams.iter().all(|x| out.contains(x)) {
        return Err(bad(format!("{node} feeds a beamformer the blend unit does not take")));
    }
    let mut visited = beams.clone();
    visited.push(lname.clone());
    Ok(Fan { beam, refs, stage: mic::FreqStage::LowBand { compare: 0, blend: blend_idx, law }, visited, last: lname })
}

/// Where a compile stands while walking the path.
#[derive(PartialEq)]
enum Domain {
    PreTime,
    Freq,
    PostTime,
}

/// Compiles the bundle's microphone chain. `Err` only when the bundle
/// cannot describe one at all; a chain it cannot carry comes back with
/// [`Compiled::params`] `None` and the reasons in the report.
pub fn compile(b: &Bundle, opts: &Options, diags: &mut Diagnostics) -> Result<Compiled> {
    let sel = select(b, diags)?;
    let mut r = Report {
        aid: b.manifest.aid.clone(),
        configuration: Some(sel.configuration.clone()),
        graph: sel.graph.clone(),
        strip: sel.strip.clone(),
        propstrip: sel.propstrip.clone(),
        ..Default::default()
    };
    let (gtext, named) = prepare_graph(graph_text(b, &sel.graph)?);
    let g = dspg::parse(&gtext, &sel.substitutions, &sel.graph)?;
    let strip = AuStrip::parse(need(b, &sel.strip, "strip")?, &sel.strip)?;
    let props = match &sel.propstrip {
        Some(p) => Some(PropStrip::parse(need(b, p, "property strip")?, p)?),
        None => None,
    };
    let fs: u32 = sel
        .substitutions
        .get("fs")
        .or_else(|| g.macros.get("fs"))
        .and_then(|v| v.parse::<f64>().ok())
        .filter(|v| *v > 0.0 && v.fract() == 0.0)
        .map(|v| v as u32)
        .ok_or_else(|| bad("the configuration gives no sample rate (fs)"))?;
    r.sample_rate = fs;
    for e in strip.effects() {
        if g.find_box(&e.box_name).is_none() {
            r.notes.push(format!("strip effect {} names no box of the graph", e.box_name));
        }
    }
    let graph_params: BTreeMap<String, f64> = g.params.iter().map(|p| (p.name.clone(), p.default)).collect();
    let overrides = static_params(&g, &graph_params)?;
    for l in &g.param_links {
        r.notes.push(format!(
            "graph parameter {} (default {}) drives {} {:?}: carried at its default",
            l.name,
            graph_params.get(&l.name).copied().unwrap_or(0.0),
            l.target.node,
            l.target.id
        ));
    }
    for l in &named {
        r.notes.push(format!(
            "graph parameter {} drives nested graph {} parameter {}: carried at its default",
            l.param, l.node, l.code
        ));
        if g.find_box(&l.node).is_none_or(|b| subtype(b) != "dspg") || !graph_params.contains_key(&l.param) {
            r.refusals.push(format!(
                "named parameter link {} -> {} {}: not a nested graph box parameter",
                l.param, l.node, l.code
            ));
        }
    }
    let (Some(input), Some(output)) = (g.inputs.first(), g.outputs.first()) else {
        return Err(bad("graph without an input or output port"));
    };
    let first = g.wires.iter().find(|w| w.src.node == *input).ok_or_else(|| bad("the graph input is not wired"))?;
    let mics = wire_ch(&g, first)?;
    if !(1..=mic::MAX_MICS).contains(&mics) {
        return Err(bad(format!("{mics} microphone channels")));
    }
    r.mics = mics;

    let mut pre: Vec<Stage> = Vec::new();
    let mut post: Vec<Stage> = Vec::new();
    let mut stft: Option<Stft> = None;
    let mut beam_p: Option<Beam> = None;
    let mut refs: Vec<Beam> = Vec::new();
    let mut freq: Vec<mic::FreqStage> = Vec::new();
    let mut sens_hop: Option<usize> = None;
    let mut domain = Domain::PreTime;
    let mut node = input.clone();
    let mut visited = vec![input.clone()];
    loop {
        // the one live consumer of `node`'s output 0
        let live: Vec<&dspg::Wire> =
            g.wires.iter().filter(|w| w.src.node == node && !is_dead(&g, &w.dst.node)).collect();
        if let Some(w) = live.iter().find(|w| w.src.port != 0) {
            r.refusals.push(format!(
                "{node} output {} feeds {}: only output 0 may continue the path",
                w.src.port, w.dst.node
            ));
            break;
        }
        let mut next: Vec<&str> = live.iter().map(|w| w.dst.node.as_str()).collect();
        next.dedup();
        if domain == Domain::Freq && beam_p.is_none() && next.len() > 1 {
            // the analysis feeding several beamformers that meet in a blend
            match fan_out(&FanCtx { b, g: &g, strip: &strip, props: props.as_ref(), opts, fs, stft }, &node, &mut r) {
                Ok(f) => {
                    beam_p = Some(f.beam);
                    refs = f.refs;
                    freq.push(f.stage);
                    visited.extend(f.visited);
                    node = f.last;
                }
                Err(err) => {
                    r.refusals.push(err.to_string());
                    break;
                }
            }
            continue;
        }
        let [nx] = next.as_slice() else {
            r.refusals.push(format!("{node} feeds {next:?}: the chain is not a single path"));
            break;
        };
        let nx = nx.to_string();
        // a box with several inputs takes this same source on each
        let fan: Vec<_> = live.iter().filter(|w| w.dst.node == nx).collect();
        let in_w = wire_ch(&g, fan[0])?;
        if nx == *output {
            if domain != Domain::PostTime || in_w != 1 {
                r.refusals.push("the output is not one time-domain channel after the synthesis".into());
            }
            break;
        }
        if visited.contains(&nx) {
            return Err(bad(format!("the path returns to {nx}")));
        }
        visited.push(nx.clone());
        let bx = g.find_box(&nx).unwrap();
        if (fan.len() as u32) != bx.inputs {
            r.refusals.push(format!("{nx}: {} of its {} inputs come from {node}", fan.len(), bx.inputs));
            break;
        }
        let e = strip.effect(&nx);
        let ov = overrides.get(&nx).cloned().unwrap_or_default();
        let kind = kind_name(bx);
        let sub = subtype(bx);
        let dom = if domain == Domain::Freq { "freq" } else { "time" };
        let out_w = g
            .wires
            .iter()
            .find(|w| w.src.node == nx && w.src.port == 0)
            .map(|w| wire_ch(&g, w))
            .transpose()?
            .unwrap_or(in_w);
        if e.is_some_and(|e| e.bypass) && !matches!(sub.as_str(), "bmfd") {
            if in_w != out_w {
                r.stage(&nx, &kind, dom, Status::Refused, "bypassed but changes the channel count");
                r.refusals.push(format!("{nx}: bypassed box changes the channel count"));
            } else {
                r.stage(&nx, &kind, dom, Status::Bypassed, "bypassed in the strip");
            }
            node = nx;
            continue;
        }
        let stages = if domain == Domain::PostTime { &mut post } else { &mut pre };
        match (sub.as_str(), &domain) {
            ("reblocker", Domain::PreTime) | ("reblocker", Domain::PostTime) => {
                r.stage(&nx, &kind, dom, Status::Structural, "regroups samples into the graph's blocks");
            }
            ("dccb", Domain::PreTime | Domain::PostTime) => {
                let e = e.ok_or_else(|| bad(format!("{nx}: no preset in the strip")))?;
                let (Some(fc), Some(ramp), Some(gdb)) = (global(e, 0), global(e, 1), global(e, 2)) else {
                    r.stage(&nx, &kind, dom, Status::Refused, "saved state lacks parameter 0, 1 or 2");
                    r.refusals.push(format!("{nx}: DC blocker state incomplete"));
                    node = nx;
                    continue;
                };
                let ramp = match opts.dev_dccb_ramp_ms {
                    Some(v) => {
                        r.refusals.push(format!("{nx}: development ramp override (analysis only, never a blob)"));
                        v
                    }
                    None => ramp,
                };
                stages.push(Stage::DcBlock { cutoff_hz: fc, ramp_ms: ramp, gain_db: gdb });
                r.stage(
                    &nx,
                    &kind,
                    dom,
                    Status::Approximate,
                    format!(
                        "2nd-order Butterworth high-pass, gain, {ramp} ms linear start ramp from the first nonzero \
                         block (the engine's block, not the host's)"
                    ),
                );
                r.todo(
                    "dccb output during the start ramp: black-box renders are not ramp x high-pass there (an extra \
                     decaying term with the high-pass poles); steady state exact",
                );
            }
            ("gain" | "dbgn", Domain::PreTime | Domain::PostTime) => match gain_db(&nx, &sub, e, &ov) {
                Ok((db, false)) => {
                    stages.push(Stage::Gain { db });
                    r.stage(&nx, &kind, dom, Status::Exact, format!("{db:+.2} dB"));
                }
                Ok((_, true)) => {
                    r.stage(&nx, &kind, dom, Status::Refused, "parameter 1 is set (meaning not characterised)");
                    r.refusals.push(format!("{nx}: gain parameter 1 set"));
                }
                Err(err) => {
                    r.stage(&nx, &kind, dom, Status::Refused, err.to_string());
                    r.refusals.push(err.to_string());
                }
            },
            ("tmst", Domain::PreTime | Domain::PostTime) => {
                let mut ov = ov.clone();
                if let Some(e) = e {
                    let dropped = dev_drops(e, opts.dev_drop_tone_types);
                    if !dropped.is_empty() {
                        r.refusals.push(format!(
                            "{nx}: development option switched off slots {:?} (analysis only, never a blob)",
                            dropped.iter().map(|(id, _)| (id - 14) / 10).collect::<Vec<_>>()
                        ));
                        ov.extend(dropped);
                    }
                }
                let res = e
                    .ok_or_else(|| bad(format!("{nx}: no preset in the strip")))
                    .and_then(|e| tone_nodes(bx, e, in_w, &ov, fs as f64));
                match res {
                    Ok(k) => {
                        stages.push(Stage::Graph(series(k, in_w)));
                        r.stage(&nx, &kind, dom, Status::Exact, "filter box at its saved state");
                    }
                    Err(err) => {
                        r.stage(&nx, &kind, dom, Status::Refused, err.to_string());
                        r.refusals.push(err.to_string());
                    }
                }
            }
            ("dspg", Domain::PreTime | Domain::PostTime) => {
                let res = (|| -> Result<Spec> {
                    let gp = box_path(b, props.as_ref(), &nx, PROP_GRAPH)?
                        .ok_or_else(|| bad(format!("{nx}: the property strip names no graph")))?;
                    let sp = box_path(b, props.as_ref(), &nx, PROP_AUSTRIP)?
                        .ok_or_else(|| bad(format!("{nx}: the property strip names no strip")))?;
                    let pp = box_path(b, props.as_ref(), &nx, PROP_PROPSTRIP)?;
                    let mut subst = sel.substitutions.clone();
                    subst.entry("sampleRate".into()).or_insert_with(|| fs.to_string());
                    let (ntext, nnamed) = prepare_graph(graph_text(b, &gp)?);
                    if let Some(l) = nnamed.first() {
                        return Err(bad(format!(
                            "{nx}: nested graph links {} by name into {}: not characterised",
                            l.param, l.node
                        )));
                    }
                    let ng = dspg::parse(&ntext, &subst, &gp)?;
                    let ns = AuStrip::parse(need(b, &sp, "strip")?, &sp)?;
                    let np = match &pp {
                        Some(p) => Some(PropStrip::parse(need(b, p, "property strip")?, p)?),
                        None => None,
                    };
                    // the parent's parameters wired by name into the box
                    let mut pass = BTreeMap::new();
                    for l in named.iter().filter(|l| l.node == nx) {
                        pass.insert(l.code.clone(), graph_params.get(&l.param).copied().unwrap_or(0.0));
                    }
                    let mut dropped = Vec::new();
                    let nest = Nest {
                        graph: &ng,
                        strip: &ns,
                        props: np.as_ref(),
                        parent_params: &pass,
                        width: in_w,
                        fs: fs as f64,
                        dev_drop_tone_types: opts.dev_drop_tone_types,
                    };
                    let spec = nested(&nest, &mut r.notes, &mut dropped)?;
                    if !dropped.is_empty() {
                        r.refusals.push(format!(
                            "{nx}: development option switched off {} (analysis only, never a blob)",
                            dropped.join(", ")
                        ));
                    }
                    Ok(spec)
                })();
                match res {
                    Ok(spec) => {
                        let n = spec.nodes.len();
                        stages.push(Stage::Graph(spec));
                        r.stage(&nx, &kind, dom, Status::Exact, format!("nested graph flattened to {n} nodes"));
                    }
                    Err(err) => {
                        r.stage(&nx, &kind, dom, Status::Refused, err.to_string());
                        r.refusals.push(err.to_string());
                    }
                }
            }
            ("fft", Domain::PreTime) => {
                let w = g.wires.iter().find(|w| w.src.node == nx && w.src.port == 0).unwrap();
                let f = g.wire_format(w).ok_or_else(|| bad(format!("{nx}: output wire has no format")))?;
                let hop = f.block_size.as_ref().and_then(Num::value).filter(|v| *v > 0.0 && v.fract() == 0.0);
                match (f.frequency_domain, hop) {
                    (true, Some(h)) if (h as usize) <= mic::MAX_HOP => {
                        let hop = h as usize;
                        stft = Some(Stft {
                            hop,
                            size: 2 * hop,
                            window: opts.window,
                            half_offset: opts.half_offset,
                            frame_phase: opts.frame_phase.min(hop - 1),
                        });
                        domain = Domain::Freq;
                        let exact = transform_exact(opts);
                        r.stage(
                            &nx,
                            &kind,
                            "time",
                            if exact { Status::Exact } else { Status::Approximate },
                            format!("analysis, hop {hop}, frame {}, {:?} window", 2 * hop, opts.window),
                        );
                        if !exact {
                            r.todo("development transform option: not the black-box exact window/phase");
                        }
                        if sens_hop.is_some_and(|sh| sh != hop) {
                            r.refusals.push(format!("{nx}: hop {hop}, but the suppressor runs on {sens_hop:?}"));
                        }
                    }
                    _ => {
                        r.stage(
                            &nx,
                            &kind,
                            "time",
                            Status::Refused,
                            "analysis output is not a blocked frequency-domain wire",
                        );
                        r.refusals.push(format!("{nx}: unsupported analysis format"));
                    }
                }
            }
            ("fft", Domain::Freq) => {
                if beam_p.is_none() {
                    r.refusals.push(format!("{nx}: synthesis without a beamformer before it"));
                }
                domain = Domain::PostTime;
                let exact = transform_exact(opts);
                r.stage(
                    &nx,
                    &kind,
                    "freq",
                    if exact { Status::Exact } else { Status::Approximate },
                    "synthesis, overlap-add",
                );
                if !exact {
                    r.todo("development transform option: not the black-box exact window/phase");
                }
            }
            ("bmfd", Domain::Freq) if beam_p.is_none() => {
                let res = (|| -> Result<Beam> {
                    if out_w != 1 {
                        return Err(bad(format!("{nx}: output 0 carries {out_w} channels")));
                    }
                    beamformer(b, props.as_ref(), &nx, e, in_w, stft, fs, opts, &mut r.notes)
                })();
                match res {
                    Ok(bm) => {
                        beam_p = Some(bm);
                        r.stage(
                            &nx,
                            &kind,
                            "freq",
                            if opts.beam_eq { Status::Approximate } else { Status::Exact },
                            format!("fixed beam over {in_w} microphones, fixed-beam mode (adaptive blend not engaged)"),
                        );
                        if opts.beam_eq {
                            r.todo("development option: EQSelection applied to the uplink (the unit does not)");
                        }
                    }
                    Err(err) => {
                        r.stage(&nx, &kind, "freq", Status::Refused, err.to_string());
                        r.refusals.push(err.to_string());
                    }
                }
                node = nx;
                continue;
            }
            ("sens", Domain::PreTime) if in_w == out_w => {
                let res = (|| -> Result<mic::sens::Sens> {
                    let e = e.ok_or_else(|| bad(format!("{nx}: no preset in the strip")))?;
                    let hop = g
                        .wire_format(fan[0])
                        .and_then(|f| f.block_size.as_ref())
                        .and_then(Num::value)
                        .filter(|v| *v >= 2.0 && v.fract() == 0.0 && *v as usize <= mic::BLOCK)
                        .ok_or_else(|| {
                            bad(format!("{nx}: its input is not blocked in at most {} frames", mic::BLOCK))
                        })? as usize;
                    let p = box_path(b, props.as_ref(), &nx, PROP_SENS_FILE)?
                        .ok_or_else(|| bad(format!("{nx}: the property strip names no noise profile")))?;
                    sens(e, &plist_dict(b, &p)?, in_w, hop, fs, opts.sens_order)
                })();
                match res {
                    Ok(sp) => {
                        if sens_hop.is_some_and(|h| h != sp.hop) {
                            r.refusals.push(format!("{nx}: suppressors on different blocks"));
                        }
                        sens_hop = Some(sp.hop);
                        let detail = format!(
                            "noise-floor suppressor, {} microphones, block {}, Burg order {}{}",
                            sp.mics,
                            sp.hop,
                            sp.order,
                            if sp.bypass { ", bypass (a block of delay)" } else { "" }
                        );
                        stages.push(Stage::Sens(sp));
                        r.stage(&nx, &kind, dom, Status::Approximate, detail);
                        r.todo("sens artifact step: boxcar lengths, edges, mask slope fitted to black-box renders");
                        r.todo("sens Burg order fitted");
                        r.todo("sens gain/signal smoother start-up state (gain 1, PSD 0 after reset)");
                    }
                    Err(err) => {
                        r.stage(&nx, &kind, dom, Status::Refused, err.to_string());
                        r.refusals.push(err.to_string());
                    }
                }
            }
            (s, _) if TRANSPARENT_OR_ATTENUATING.contains(&s) && in_w == out_w => {
                let code = s.as_bytes();
                if domain != Domain::Freq {
                    stages.push(Stage::Identity([code[0], code[1], code[2], code[3]]));
                }
                r.stage(
                    &nx,
                    &kind,
                    dom,
                    Status::Uncharacterised,
                    "carried as identity (documented transparent or attenuating)",
                );
                if !opts.allow_partial {
                    r.refusals
                        .push(format!("{nx} ({kind}) is uncharacterised; --allow-partial carries it as identity"));
                }
            }
            _ => {
                r.stage(&nx, &kind, dom, Status::Refused, "box kind not characterised here");
                r.refusals.push(format!("{nx} ({kind}) is not characterised"));
            }
        }
        if out_w != in_w && sub != "bmfd" {
            r.refusals.push(format!("{nx}: {in_w} channels in, {out_w} out"));
        }
        node = nx;
    }
    // every box off the path must be a dead end, parameter logic, or bypassed
    for bx in &g.boxes {
        if visited.contains(&bx.name) || is_dead(&g, &bx.name) || parameter_box(bx) {
            continue;
        }
        if strip.effect(&bx.name).is_some_and(|e| e.bypass) {
            r.stage(&bx.name, &kind_name(bx), "-", Status::Bypassed, "off the path, bypassed");
        } else {
            r.stage(&bx.name, &kind_name(bx), "-", Status::Refused, "off the single path");
            r.refusals.push(format!("{} ({}) is off the single path", bx.name, kind_name(bx)));
        }
    }
    let (Some(stft), Some(beam_p)) = (stft, beam_p) else {
        if r.refusals.is_empty() {
            r.refusals.push("no analysis/beamformer/synthesis found".into());
        }
        return Ok(Compiled { params: None, report: r });
    };
    let partial = pre.iter().chain(&post).any(|s| matches!(s, Stage::Identity(_)))
        || r.stages.iter().any(|s| s.status == Status::Uncharacterised);
    let mut p = Params { sample_rate: fs, mics, pre, stft, beam: beam_p, refs, freq, post, partial };
    r.partial = partial;
    r.latency_frames = p.latency();
    // an uncharacterised frequency-domain stage carries no Identity entry;
    // the flag must still say partial
    if partial && !p.pre.iter().chain(&p.post).any(|s| matches!(s, Stage::Identity(_))) {
        p.pre.push(Stage::Identity(*b"freq"));
    }
    if !p.valid() {
        r.refusals.push("the compiled parameters do not validate".into());
        return Ok(Compiled { params: None, report: r });
    }
    Ok(Compiled { params: Some(p), report: r })
}

/// The suppressor of the bundle's default microphone chain alone (its
/// first `aufx/sens` box), for offline analysis of that stage.
pub fn stage_sens(b: &Bundle, opts: &Options, diags: &mut Diagnostics) -> Result<mic::sens::Sens> {
    let sel = select(b, diags)?;
    let (gtext, _) = prepare_graph(graph_text(b, &sel.graph)?);
    let g = dspg::parse(&gtext, &sel.substitutions, &sel.graph)?;
    let strip = AuStrip::parse(need(b, &sel.strip, "strip")?, &sel.strip)?;
    let props = match &sel.propstrip {
        Some(p) => Some(PropStrip::parse(need(b, p, "property strip")?, p)?),
        None => None,
    };
    let fs: u32 = sel
        .substitutions
        .get("fs")
        .or_else(|| g.macros.get("fs"))
        .and_then(|v| v.parse::<f64>().ok())
        .map(|v| v as u32)
        .ok_or_else(|| bad("the configuration gives no sample rate (fs)"))?;
    let bx = g.boxes.iter().find(|x| subtype(x) == "sens").ok_or_else(|| bad("no suppressor in the chain"))?;
    let w = g.wires.iter().find(|w| w.dst.node == bx.name).ok_or_else(|| bad("the suppressor is not wired"))?;
    let hop = g
        .wire_format(w)
        .and_then(|f| f.block_size.as_ref())
        .and_then(Num::value)
        .ok_or_else(|| bad("the suppressor's input is not blocked"))? as usize;
    let e = strip.effect(&bx.name).ok_or_else(|| bad("no suppressor preset"))?;
    let p = box_path(b, props.as_ref(), &bx.name, PROP_SENS_FILE)?.ok_or_else(|| bad("no suppressor profile"))?;
    sens(e, &plist_dict(b, &p)?, wire_ch(&g, w)?, hop, fs, opts.sens_order)
}

/// The binding of a microphone blob, checked as the speaker blob's is.
pub fn bind(b: &Bundle, m: &Machine, allow_unverified: bool, diags: &mut Diagnostics) -> Result<artifact::Binding> {
    if !b.verified {
        if !allow_unverified {
            return Err(bad("the bundle is not verified against a manifest"));
        }
        diags.warn("compile-unverified", "mic-compile", "compiling an unverified bundle (development only)");
    }
    if let Some(a) = m.acoustic_id {
        if a != b.manifest.acoustic_id {
            return Err(bad(format!("the bundle is {}, this Mac is AID{a}", b.manifest.aid)));
        }
    }
    if let Some(board) = &b.manifest.board {
        if *board != m.board {
            return Err(bad(format!(
                "the tuning bundle was collected on a {board}, this Mac is a {}; {}",
                m.board,
                crate::compile::RECOLLECT
            )));
        }
    }
    let unit = match (&b.manifest.unit_id, &m.unit) {
        (Some(u), Unit::Known(l)) if u == l => hex32(u),
        (Some(_), Unit::Known(_)) => {
            return Err(bad(format!(
                "the tuning bundle was collected on another Mac (its unit ID is not this Mac's); {}",
                crate::compile::RECOLLECT
            )))
        }
        _ => [0u8; artifact::UNIT_LEN],
    };
    let machine = sgv2_core::blob::machine_field(&m.board)
        .ok_or_else(|| bad(format!("board {:?} does not fit the header", m.board)))?;
    Ok(artifact::Binding { machine, acoustic_id: b.manifest.acoustic_id, unit })
}

fn hex32(s: &str) -> [u8; artifact::UNIT_LEN] {
    let mut out = [0u8; artifact::UNIT_LEN];
    if s.len() == 2 * artifact::UNIT_LEN {
        for (i, o) in out.iter_mut().enumerate() {
            *o = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap_or(0);
        }
    }
    out
}

/// Compile and encode: the blob, or the report's refusals as the error.
pub fn compile_blob(
    b: &Bundle,
    m: &Machine,
    opts: &Options,
    allow_unverified: bool,
    diags: &mut Diagnostics,
) -> Result<(Option<Vec<u8>>, Report)> {
    let c = compile(b, opts, diags)?;
    if c.refused() {
        return Ok((None, c.report));
    }
    let Some(p) = c.params else { return Ok((None, c.report)) };
    let binding = bind(b, m, allow_unverified, diags)?;
    let blob = artifact::encode(&p, &binding).map_err(|e| bad(format!("encode: {e:?}")))?;
    Ok((Some(blob), c.report))
}
