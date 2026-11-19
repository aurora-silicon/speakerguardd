// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The whole speaker tuning of one Mac, typed and validated: what the
//! protection engine and the graph generator consume.

use crate::aupreset::{ParamState, UnitId};
use crate::bundle::Bundle;
use crate::dspg::analysis::{self, Lane};
use crate::dspg::{self, BoxKind, Endpoint, FormatSpec, Graph};
use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::select::{self, Selection};
use crate::spp3::{self, Sidecar};
use crate::spts::{self, Spts};
use crate::strip::{AuStrip, PropStrip, PropValue};
use crate::versions;
use serde::Serialize;

/// The protection unit kinds this reader knows.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum UnitKind {
    Spp3,
    Atsp,
    Lsm2,
}

impl UnitKind {
    pub fn from_subtype(s: FourCc) -> Option<Self> {
        match &s.bytes() {
            b"spp3" => Some(UnitKind::Spp3),
            b"atsp" => Some(UnitKind::Atsp),
            b"lsm2" => Some(UnitKind::Lsm2),
            _ => None,
        }
    }
    pub fn fourcc(self) -> FourCc {
        FourCc::from_bytes(match self {
            UnitKind::Spp3 => *b"spp3",
            UnitKind::Atsp => *b"atsp",
            UnitKind::Lsm2 => *b"lsm2",
        })
    }
}

/// spp3 `.propstrip` property naming the sidecar (behaviour spec, graph
/// integration; the AID graphs name it as the sidecar path property).
pub const PROP_SPP3_SIDECAR: u32 = 64003;
/// macOS location prefix of tuning paths inside property strips.
pub const MACOS_TUNINGS: &str = "/System/Library/Audio/Tunings/";

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Stage {
    pub name: String,
    /// `aufx/xxxx/appl` for AudioUnits, the builtin word otherwise.
    pub kind: String,
    pub unit: Option<UnitId>,
    pub inputs: u32,
    pub outputs: u32,
    /// Channels on output 0 (from its wires), if known.
    pub channels: Option<u32>,
    pub bypass: bool,
    /// The saved parameter state from the strip, if the strip configures it.
    pub preset_name: Option<String>,
    pub state: Option<ParamState>,
    /// Preset keys beyond the saved state (unit-specific dictionaries).
    pub preset_extra_keys: Vec<String>,
    /// Unit-specific ClassInfo values. Kept out of dumps: consumers expose
    /// only fields they have validated and interpreted.
    #[serde(skip)]
    pub preset_extra: crate::plist::Dict,
    /// Properties the property strip sets on this box.
    pub strip_properties: Vec<crate::strip::Property>,
    /// Parsed files named by `Path` properties on this box.
    pub property_files: Vec<PropertyFile>,
    pub before_protection: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Telemetry {
    /// The graph input port feeding the unit's second input.
    pub port: String,
    pub channels: Option<u32>,
    pub interleaved: bool,
    /// Every box between the port and the unit.
    pub path: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct GraphLink {
    pub name: String,
    pub id: Option<f64>,
    pub rest: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Protection {
    pub kind: UnitKind,
    pub box_name: String,
    pub unit: UnitId,
    /// Index of the unit in [`Tuning::chain`].
    pub position: usize,
    /// Fixed block size of the unit's input wire, if the graph fixes one.
    pub block_size: Option<f64>,
    /// Where each channel of the unit's audio input comes from.
    pub lanes: Vec<Lane>,
    pub telemetry: Option<Telemetry>,
    /// Graph parameters wired to the unit (`wireGraphParam`).
    pub params: Vec<GraphLink>,
    /// Graph properties wired to the unit (`wireGraphProperty`).
    pub properties: Vec<GraphLink>,
    /// Properties the property strip sets on the unit.
    pub strip_properties: Vec<crate::strip::Property>,
    /// The unit's saved AudioUnit state from the strip.
    pub state: ParamState,
    /// spp3 only.
    pub spp3: Option<Sidecar>,
    /// Every property-list file a `Path` property on the unit names (the
    /// spp3 sidecar among them), parsed but not interpreted beyond spp3's.
    /// lsm2, for one, can receive a per-channel calibration file this way
    /// file: TODO(SPEC) its meaning and use.
    pub property_files: Vec<PropertyFile>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct PropertyFile {
    pub number: u32,
    /// Bundle path.
    pub path: String,
    /// Top-level keys, for the dump.
    pub keys: Vec<String>,
    #[serde(skip)]
    pub value: crate::plist::Value,
}

/// Maps a macOS tuning path from a property strip into the bundle.
pub fn bundle_path(b: &Bundle, path: &str) -> Result<String> {
    let rel = path
        .strip_prefix(MACOS_TUNINGS)
        .ok_or_else(|| Error::format("propstrip", format!("path {path:?} is outside {MACOS_TUNINGS}")))?;
    if !crate::manifest::admitted(rel) || !rel.starts_with(&format!("{}/", b.manifest.aid)) {
        return Err(Error::format("propstrip", format!("path {path:?} is not inside {}", b.manifest.aid)));
    }
    Ok(rel.to_string())
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Tuning {
    pub aid: String,
    pub acoustic_id: u32,
    pub macos_build: Option<String>,
    /// The build whose laws compile this tuning and whose parity block
    /// checks it: the reference (25G83) when the speaker files equal
    /// 25G83's, else `macos_build` ([`crate::reference`]).
    pub law_build: Option<String>,
    /// The collecting Mac's board and unit ID, from the manifest
    /// ([`crate::manifest::Manifest::board`], [`crate::manifest::Manifest::unit_id`]).
    pub board: Option<String>,
    pub unit_id: Option<String>,
    pub verified: bool,
    /// The machine's speaker-assembly ID the selection used, and where it
    /// came from (`override`, `manifest`, `device-tree`), if known.
    pub speaker_id: Option<u32>,
    pub speaker_id_source: Option<&'static str>,
    /// Every error-severity finding of [`Tuning::load`].  These make the
    /// tuning unsafe to run: [`crate::compile::compile`] refuses a tuning
    /// that has any.
    pub refusals: Vec<crate::error::Diagnostic>,
    pub selection: Selection,
    pub graph_name: Option<String>,
    pub lsid: Option<crate::lsid::Companion>,
    pub eliminated_controls: Vec<String>,
    /// Boxes in processing order.
    pub chain: Vec<Stage>,
    pub protection: Protection,
    pub spts: Option<Spts>,
    /// The lsm2 masker's bucket lattice from the bundle, validated
    /// ([`crate::lattice`]); `None` when the bundle has none.  Kept out of
    /// dumps; `mask_lattice_present` says whether it is there.
    #[serde(skip)]
    pub mask_lattice: Option<[u32; crate::lattice::ENTRIES]>,
    pub mask_lattice_present: bool,
    /// Private owner-local missing records; excluded from tuning dumps.
    #[serde(skip)]
    pub lsm2_param_fill: Option<Vec<(u32, f32)>>,
    /// The speaker volume control's dB range from the bundle's device
    /// configuration ([`crate::hal_volume`]); `None` keeps the cubic law.
    pub volume_range: Option<crate::hal_volume::VolumeRange>,
    #[serde(skip)]
    pub graph: Graph,
}

#[derive(Clone, Debug, Default)]
pub struct Options {
    /// An explicit speaker ID (`spid`): a test or development override.
    /// It wins over every other source.
    pub speaker_id: Option<i128>,
    /// The speaker ID the loader exported into the device tree
    /// ([`crate::machine::speaker_id`]), used when the manifest has none.
    pub device_tree_speaker_id: Option<u32>,
}

/// The speaker ID for selection: the override, then the manifest, then the
/// device tree.  The manifest and the device tree must agree when both
/// give one.
fn resolve_speaker_id(b: &Bundle, opts: &Options) -> Result<(Option<i128>, Option<&'static str>)> {
    if let Some(v) = opts.speaker_id {
        return Ok((Some(v), Some("override")));
    }
    match (b.manifest.speaker_id, opts.device_tree_speaker_id) {
        (Some(m), Some(d)) if m != d => Err(Error::format(
            "bundle",
            format!("the manifest's speaker ID {m} differs from the device tree's {d}: the bundle is not this Mac's"),
        )),
        (Some(m), _) => Ok((Some(m.into()), Some("manifest"))),
        (None, Some(d)) => Ok((Some(d.into()), Some("device-tree"))),
        (None, None) => Ok((None, None)),
    }
}

fn need<'a>(b: &'a Bundle, path: &str, what: &str) -> Result<&'a [u8]> {
    b.file(path).ok_or_else(|| Error::missing("bundle", &format!("{what} {path}")))
}

fn wire_into<'a>(g: &'a Graph, node: &str, port: u32) -> Option<&'a dspg::Wire> {
    g.wires.iter().find(|w| w.dst.node == node && w.dst.port == port)
}

fn out_channels(g: &Graph, node: &str) -> Option<u32> {
    g.wires
        .iter()
        .find(|w| w.src.node == node && w.src.port == 0)
        .and_then(|w| g.wire_format(w))
        .and_then(|f| f.channels.value())
        .map(|c| c as u32)
}

fn property_files(
    b: &Bundle,
    properties: &[crate::strip::Property],
    box_name: &str,
    diags: &mut Diagnostics,
) -> Result<Vec<PropertyFile>> {
    let mut files = Vec::new();
    for sp in properties {
        if let PropValue::Path(p) = &sp.value {
            let rel = bundle_path(b, p)?;
            let bytes = need(b, &rel, &format!("{box_name} property {} file", sp.number))?;
            let (value, _) = crate::plist::parse(bytes).map_err(|e| Error::plist(&rel, e))?;
            let keys = value.as_dict().map(|d| d.keys().map(str::to_string).collect()).unwrap_or_default();
            files.push(PropertyFile { number: sp.number, path: rel, keys, value });
        }
    }
    if !files.is_empty() {
        diags.info("box-property-files", box_name, format!("loaded {} property-list sidecar(s)", files.len()));
    }
    Ok(files)
}

fn link(l: &dspg::Link) -> GraphLink {
    GraphLink { name: l.name.clone(), id: l.target.id.value(), rest: l.target.rest.clone() }
}

impl Tuning {
    /// Loads everything; returns the tuning and the validation findings.
    /// An `Err` means the bundle cannot describe a protection chain at all.
    pub fn load(b: &Bundle, opts: &Options, diags: &mut Diagnostics) -> Result<Tuning> {
        let first = diags.items.len();
        let mut t = Self::load_inner(b, opts, diags)?;
        t.refusals =
            diags.items[first..].iter().filter(|d| d.severity == crate::error::Severity::Error).cloned().collect();
        Ok(t)
    }

    fn load_inner(b: &Bundle, opts: &Options, diags: &mut Diagnostics) -> Result<Tuning> {
        if !b.verified {
            diags.warn("bundle-unverified", "bundle", "read in place without a manifest (development only)");
        }
        for u in &b.unlisted {
            diags.warn("bundle-unlisted", u, "present in the archive but not in the manifest; ignored");
        }
        versions::check(b.manifest.macos_build.as_deref(), diags);
        let (speaker_id, speaker_id_source) = resolve_speaker_id(b, opts)?;
        let sel = select::speaker_chain(b, speaker_id, diags)?;

        let gtext = std::str::from_utf8(need(b, &sel.graph, "graph")?)
            .map_err(|_| Error::format(&sel.graph, "graph is not UTF-8 text"))?;
        let graph = dspg::parse(gtext, &sel.substitutions, &sel.graph)?;
        for n in &graph.notes {
            diags.info("dspg-note", &sel.graph, n.clone());
        }
        if !graph.unresolved.is_empty() {
            diags.warn("dspg-unresolved", &sel.graph, format!("unresolved macros: {}", graph.unresolved.join(", ")));
        }
        let order = analysis::processing_order(&graph)?;

        let strip = AuStrip::parse(need(b, &sel.strip, "strip")?, &sel.strip)?;
        let props = match &sel.propstrip {
            Some(p) => Some(PropStrip::parse(need(b, p, "property strip")?, p)?),
            None => None,
        };

        let eliminated_controls = crate::lsid::eliminate_dbcontrol(&graph, &strip, props.as_ref())?;
        for name in &eliminated_controls {
            diags.info(
                "dbcontrol-eliminated",
                name,
                "zero audio/control/parameter/property connectivity proven; orphan eliminated",
            );
        }

        // The protection unit: exactly one known protection box.
        let prot: Vec<(&dspg::GraphBox, UnitKind)> = graph
            .boxes
            .iter()
            .filter_map(|bx| bx.subtype().and_then(UnitKind::from_subtype).map(|k| (bx, k)))
            .collect();
        let (pbox, kind) = match prot.as_slice() {
            [one] => *one,
            [] => {
                return Err(Error::format(&sel.graph, "the speaker graph has no protection unit (spp3, atsp or lsm2)"))
            }
            _ => return Err(Error::format(&sel.graph, "the speaker graph has more than one protection unit")),
        };
        if !order.order.contains(&pbox.name) {
            return Err(Error::format(&sel.graph, format!("protection box {} is not on the signal path", pbox.name)));
        }
        for u in &order.unreached {
            diags.info("dspg-unreached", &sel.graph, format!("box {u} is not reached from a graph input"));
        }

        // Stages, in processing order, with their strip presets.
        let ppos = order.order.iter().position(|n| *n == pbox.name).unwrap();
        let mut chain = Vec::new();
        for (i, name) in order.order.iter().enumerate() {
            let bx = graph.find_box(name).unwrap();
            let effect = strip.effect(name);
            let (kindtxt, unit) = match &bx.kind {
                BoxKind::AudioUnit { kind, subtype, manufacturer } => {
                    let m = effect.map(|e| e.unit.manufacturer).or(*manufacturer);
                    (
                        format!("{kind}/{subtype}/{}", m.map_or("?".to_string(), |m| m.to_string())),
                        effect.map(|e| e.unit),
                    )
                }
                BoxKind::Builtin(w) => (w.clone(), effect.map(|e| e.unit)),
            };
            if let Some(e) = effect {
                if matches!(bx.kind, BoxKind::AudioUnit { .. }) && !bx.matches_unit(&e.unit) {
                    diags.error(
                        "strip-unit-mismatch",
                        &sel.strip,
                        format!("{name}: strip unit {} does not match the graph box", e.unit),
                    );
                }
            } else if matches!(bx.kind, BoxKind::AudioUnit { .. }) {
                diags.warn(
                    "strip-missing-effect",
                    &sel.strip,
                    format!("{name}: AudioUnit box has no preset in the strip"),
                );
            }
            let stage_properties =
                props.as_ref().and_then(|p| p.get(name)).map(|p| p.properties.clone()).unwrap_or_default();
            let stage_property_files = property_files(b, &stage_properties, name, diags)?;
            chain.push(Stage {
                name: name.clone(),
                kind: kindtxt,
                unit,
                inputs: bx.inputs,
                outputs: bx.outputs,
                channels: out_channels(&graph, name),
                bypass: effect.is_some_and(|e| e.bypass),
                preset_name: effect.and_then(|e| e.preset.name.clone()),
                state: effect.map(|e| e.preset.state.clone()),
                preset_extra_keys: effect.map(|e| e.preset.extra_keys()).unwrap_or_default(),
                preset_extra: effect.map(|e| e.preset.extra.clone()).unwrap_or_default(),
                strip_properties: stage_properties,
                property_files: stage_property_files,
                before_protection: i < ppos,
            });
        }
        for e in strip.effects() {
            if graph.find_box(&e.box_name).is_none() {
                diags.warn("strip-unknown-box", &sel.strip, format!("effect {} names no graph box", e.box_name));
            }
        }

        let peffect = strip
            .effect(&pbox.name)
            .ok_or_else(|| Error::missing(&sel.strip, &format!("preset for the protection box {}", pbox.name)))?;
        if !peffect.preset.has_state {
            return Err(Error::missing(&sel.strip, &format!("{} preset data (saved state)", pbox.name)));
        }
        if peffect.bypass {
            diags.error("protection-bypassed", &sel.strip, format!("the strip bypasses {}", pbox.name));
        }

        let lanes = analysis::lanes_into(&graph, &Endpoint { node: pbox.name.clone(), port: 0 })?;
        let lsid = crate::lsid::load(&graph, &strip, &pbox.name, lanes.len(), diags)?;
        if lsid.is_some() && kind != UnitKind::Lsm2 {
            return Err(Error::format("lsid", "estimator companion requires LSM2"));
        }
        let block_size = wire_into(&graph, &pbox.name, 0)
            .and_then(|w| graph.wire_format(w))
            .and_then(|f: &FormatSpec| f.block_size.as_ref())
            .and_then(dspg::Num::value);
        let telemetry = if pbox.inputs > 1 {
            match wire_into(&graph, &pbox.name, 1) {
                Some(w) => Some(trace_telemetry(&graph, w)?),
                None => {
                    diags.warn("protection-no-telemetry-wire", &sel.graph, "the unit's second input is unconnected");
                    None
                }
            }
        } else {
            None
        };
        let params = graph.param_links.iter().filter(|l| l.target.node == pbox.name).map(link).collect();
        let properties = graph.property_links.iter().filter(|l| l.target.node == pbox.name).map(link).collect();
        let pprops = props.as_ref().and_then(|p| p.get(&pbox.name));
        let strip_properties = chain[ppos].strip_properties.clone();

        let spp3 = if kind == UnitKind::Spp3 {
            let path = match pprops.and_then(|p| p.property(PROP_SPP3_SIDECAR)) {
                Some(PropValue::Path(p)) => p.clone(),
                Some(other) => {
                    return Err(Error::format(
                        "propstrip",
                        format!("property {PROP_SPP3_SIDECAR} is {other:?}, not a path"),
                    ))
                }
                None => {
                    return Err(Error::missing(
                        "propstrip",
                        &format!("{} property {PROP_SPP3_SIDECAR} (spp3 sidecar)", pbox.name),
                    ))
                }
            };
            let rel = bundle_path(b, &path)?;
            let rel = rel.as_str();
            let s = Sidecar::parse(need(b, rel, "spp3 sidecar")?, rel, diags)?;
            spp3::cross_check(&s, &peffect.preset.state, &format!("{} preset", pbox.name), diags);
            if s.num_channels != lanes.len() {
                diags.error(
                    "spp3-channel-count",
                    rel,
                    format!("NumChannels {} but the graph feeds the unit {} channels", s.num_channels, lanes.len()),
                );
            }
            for c in &s.channels {
                if c.speaker_group.is_none() {
                    diags.warn(
                        "spp3-no-group",
                        rel,
                        format!("Channel{} has no SpeakerGroup (TODO(SPEC) default)", c.index),
                    );
                }
            }
            if let Some(t) = &telemetry {
                if let Some(n) = t.channels {
                    if n as usize != 2 * s.num_channels {
                        diags.error(
                            "spp3-telemetry-width",
                            &sel.graph,
                            format!(
                                "telemetry bus has {n} channels, expected 2 x {} (I/V per channel)",
                                s.num_channels
                            ),
                        );
                    }
                }
            }
            Some(s)
        } else {
            if kind == UnitKind::Lsm2 {
                diags.info(
                    "params-raw-ids",
                    &pbox.name,
                    format!("{kind:?} parameters are exposed as raw IDs: TODO(SPEC) the ID map"),
                );
            }
            None
        };

        let property_files = chain[ppos].property_files.clone();

        let volume_range = crate::hal_volume::speaker_range(b, diags);
        let spts = match b.spts() {
            Some(bytes) => Some(spts::decode(bytes, crate::manifest::SPTS_NAME, diags)?),
            None => {
                if kind == UnitKind::Spp3 {
                    diags.warn(
                        "spts-absent",
                        "bundle",
                        "no speaker-thiele-small calibration: closed loop cannot seed R_cal",
                    );
                }
                None
            }
        };
        if let (Some(t), Some(s)) = (&spts, &spp3) {
            if t.records.len() != s.num_channels {
                diags.warn(
                    "spts-count",
                    crate::manifest::SPTS_NAME,
                    format!("{} records for {} channels", t.records.len(), s.num_channels),
                );
            }
            diags.info(
                "spts-binding",
                crate::manifest::SPTS_NAME,
                "records bind to unit channels by position; the data-source IDs are provenance",
            );
        }

        // The masker lattice: checked again with the installer's rule.  A
        // listed lattice that fails it makes the tuning unsafe to run; an
        // absent one leaves lsm2 without masking credit.
        let mask_lattice = match b.mask_lattice() {
            Some(bytes) => match crate::lattice::parse(bytes) {
                Ok(l) => Some(l),
                Err(e) => {
                    diags.error("mask-lattice-invalid", crate::lattice::NAME, e.to_string());
                    None
                }
            },
            None => {
                if kind == UnitKind::Lsm2 {
                    diags.warn(
                        "mask-lattice-absent",
                        "bundle",
                        "no lsm2 masker lattice in the bundle (collected before the installer read it): \
                         no masking credit, the velocity limit stays at VelocityLimit0; collect the tuning again on macOS",
                    );
                }
                None
            }
        };

        let lsm2_param_fill = if kind == UnitKind::Lsm2 {
            match &b.manifest.param_fill {
                Some(record) => {
                    let bytes = need(b, &record.path, "lsm2 fill")?;
                    let saved = peffect
                        .preset
                        .state
                        .global()
                        .ok_or_else(|| Error::format(crate::param_fill::NAME, "no global lsm2 state"))?;
                    crate::param_fill::read(bytes, b, &sel.strip, saved)?
                }
                None => None,
            }
        } else {
            None
        };

        Ok(Tuning {
            aid: b.manifest.aid.clone(),
            acoustic_id: b.manifest.acoustic_id,
            macos_build: b.manifest.macos_build.clone(),
            law_build: crate::reference::law_build(&b.manifest.aid, &b.files, b.manifest.macos_build.as_deref()),
            board: b.manifest.board.clone(),
            unit_id: b.manifest.unit_id.clone(),
            verified: b.verified,
            speaker_id: speaker_id.and_then(|v| u32::try_from(v).ok()),
            speaker_id_source,
            refusals: Vec::new(),
            graph_name: graph.name.clone(),
            lsid,
            eliminated_controls,
            selection: sel,
            chain,
            protection: Protection {
                kind,
                box_name: pbox.name.clone(),
                unit: peffect.unit,
                position: ppos,
                block_size,
                lanes,
                telemetry,
                params,
                properties,
                strip_properties,
                state: peffect.preset.state.clone(),
                spp3,
                property_files,
            },
            spts,
            mask_lattice_present: mask_lattice.is_some(),
            lsm2_param_fill,
            volume_range,
            mask_lattice,
            graph,
        })
    }
}

fn trace_telemetry(g: &Graph, w: &dspg::Wire) -> Result<Telemetry> {
    let mut path = Vec::new();
    let mut cur = w;
    let fmt = g.wire_format(w);
    loop {
        if g.inputs.contains(&cur.src.node) {
            break;
        }
        path.push(cur.src.node.clone());
        if path.len() > g.boxes.len() {
            return Err(Error::format("dspg", "telemetry path loops"));
        }
        cur = wire_into(g, &cur.src.node, 0)
            .ok_or_else(|| Error::format("dspg", format!("telemetry path breaks at {}", cur.src.node)))?;
    }
    path.reverse();
    Ok(Telemetry {
        port: cur.src.node.clone(),
        channels: fmt.and_then(|f| f.channels.value()).map(|c| c as u32),
        interleaved: fmt.is_some_and(|f| f.interleaved),
        path,
    })
}
