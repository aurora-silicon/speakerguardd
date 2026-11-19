// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Compile graph volume wires into semantic box hooks.
//!
//! This compiler does not guess from a box's name.  It checks the graph box
//! kind, exact parameter ID/scope/element, and (for an AudioUnit) the strip's
//! matching unit.  A volume wire not listed in [`direct_parameter`] or
//! [`relay_parameter`] is a hard refusal.

use crate::dspg::{BoxKind, Graph, GraphBox, Num, Target};
use crate::error::{Error, Result};
use crate::strip::AuStrip;
use crate::tuning::Tuning;
use sgv2_core::voicing::volume::{Binding, Input, Parameter};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Direct {
    pub box_name: String,
    pub id: u32,
    pub scope: u32,
    pub element: u32,
    pub binding: Binding,
}

/// One `wireParam`: after the source box changes, read its published volume
/// value and set the destination hook.  Current graphs use VolumeTaper ID
/// 100 as the source.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Relay {
    pub from_box: String,
    pub from_id: u32,
    pub from_scope: u32,
    pub from_element: u32,
    pub from_parameter: Parameter,
    pub to_box: String,
    pub to_id: u32,
    pub to_scope: u32,
    pub to_element: u32,
    pub to_parameter: Parameter,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Wiring {
    pub direct: Vec<Direct>,
    pub relays: Vec<Relay>,
}

fn bad(message: impl Into<String>) -> Error {
    Error::format("voicing volume", message)
}

fn uint(n: &Num, what: &str) -> Result<u32> {
    let Some(v) = n.value() else { return Err(bad(format!("{what} is unresolved"))) };
    if !v.is_finite() || v < 0.0 || v > u32::MAX as f64 || v.fract() != 0.0 {
        return Err(bad(format!("{what} {v} is not a parameter ID")));
    }
    Ok(v as u32)
}

fn location(t: &Target) -> Result<(u32, u32, u32)> {
    let id = uint(&t.id, &format!("{} parameter", t.node))?;
    let number = |s: &str, what: &str| {
        s.parse::<u32>().map_err(|_| bad(format!("{} parameter {id}: {what} {s:?} is not numeric", t.node)))
    };
    match t.rest.as_slice() {
        [] => Ok((id, 0, 0)),
        [scope, element] => Ok((id, number(scope, "scope")?, number(element, "element")?)),
        other => Err(bad(format!("{} parameter {id}: unsupported location {other:?}", t.node))),
    }
}

fn subtype(b: &GraphBox) -> Option<[u8; 4]> {
    match b.kind {
        BoxKind::AudioUnit { subtype, .. } => Some(subtype.bytes()),
        BoxKind::Builtin(_) => None,
    }
}

fn direct_parameter(b: &GraphBox, input: Input, id: u32, scope: u32, element: u32) -> Option<Parameter> {
    if scope != 0 || element != 0 {
        return None;
    }
    match (&b.kind, subtype(b), input, id) {
        (BoxKind::Builtin(k), _, Input::Decibels, 0) if k == "dbgn" => Some(Parameter::GainDb),
        (_, Some(s), Input::Scalar, 5) if s == *b"vrq2" => Some(Parameter::VarEqCurrentGain),
        (_, Some(s), Input::Scalar, 3) if s == *b"tmst" => Some(Parameter::ToneSystemVolume),
        (_, Some(s), Input::Decibels, 1 | 1_937_141_612) if s == *b"ldnm" => Some(Parameter::NormSystemVolume),
        (_, Some(s), Input::Decibels, 16 | 1_937_141_612) if s == *b"axtc" => Some(Parameter::XtcSystemVolume),
        (_, Some(s), Input::Decibels, 4) if s == *b"flow" => Some(Parameter::LowFlowSystemVolume),
        (_, Some(s), Input::Scalar, 4) if s == *b"volt" => Some(Parameter::VolumeTaperScalar),
        _ => None,
    }
}

fn relay_parameter(b: &GraphBox, id: u32, scope: u32, element: u32, source: bool) -> Option<Parameter> {
    if scope != 0 || element != 0 {
        return None;
    }
    match (subtype(b), id, source) {
        (Some(s), 100, true) if s == *b"volt" => Some(Parameter::VolumeTaperOutput),
        (Some(s), 3, false) if s == *b"tmst" => Some(Parameter::ToneSystemVolume),
        (Some(s), 79, false) if s == *b"mstk" => Some(Parameter::CompressorVolume),
        (Some(s), 4, false) if s == *b"flow" => Some(Parameter::LowFlowSystemVolume),
        _ => None,
    }
}

fn checked_box<'a, F>(graph: &'a Graph, name: &str, check: &mut F) -> Result<&'a GraphBox>
where
    F: FnMut(&GraphBox) -> Result<()>,
{
    let b = graph.find_box(name).ok_or_else(|| bad(format!("wire names absent box {name}")))?;
    check(b)?;
    Ok(b)
}

fn compile_checked<F>(graph: &Graph, mut check: F) -> Result<Wiring>
where
    F: FnMut(&GraphBox) -> Result<()>,
{
    let mut out = Wiring::default();
    for link in &graph.param_links {
        let input = match link.name.as_str() {
            "mvsc" => Input::Scalar,
            "mvol" => Input::Decibels,
            _ => continue,
        };
        let b = checked_box(graph, &link.target.node, &mut check)?;
        let (id, scope, element) = location(&link.target)?;
        let parameter = direct_parameter(b, input, id, scope, element).ok_or_else(|| {
            bad(format!("uncarried {} wire to {} parameter {id}/{scope}/{element}", link.name, link.target.node))
        })?;
        out.direct.push(Direct {
            box_name: link.target.node.clone(),
            id,
            scope,
            element,
            binding: Binding { input, parameter },
        });
    }
    for link in &graph.box_param_links {
        let from = checked_box(graph, &link.from.node, &mut check)?;
        // Parameter arithmetic and other control boxes are not volume
        // publishers. Their named scopes belong to their own compiler.
        if subtype(from) != Some(*b"volt") {
            continue;
        }
        let to = checked_box(graph, &link.to.node, &mut check)?;
        let (from_id, from_scope, from_element) = location(&link.from)?;
        let (to_id, to_scope, to_element) = location(&link.to)?;
        let Some(from_parameter) = relay_parameter(from, from_id, from_scope, from_element, true) else {
            // A non-volume box relay is outside this module.
            continue;
        };
        let to_parameter = relay_parameter(to, to_id, to_scope, to_element, false).ok_or_else(|| {
            bad(format!(
                "uncarried volume relay {} {from_id}/{from_scope}/{from_element} -> {} {to_id}/{to_scope}/{to_element}",
                link.from.node, link.to.node
            ))
        })?;
        out.relays.push(Relay {
            from_box: link.from.node.clone(),
            from_id,
            from_scope,
            from_element,
            from_parameter,
            to_box: link.to.node.clone(),
            to_id,
            to_scope,
            to_element,
            to_parameter,
        });
    }
    if !graph.params.iter().any(|p| p.name == "mvsc") {
        return Err(bad("graph has no mvsc parameter"));
    }
    if out.direct.is_empty() {
        return Err(bad("graph has no carried volume wire"));
    }
    Ok(out)
}

/// Compile directly from the parsed graph and strip.  This entry point is
/// useful while assembling a [`Tuning`].
pub fn compile(graph: &Graph, strip: &AuStrip) -> Result<Wiring> {
    compile_checked(graph, |b| {
        if let BoxKind::AudioUnit { .. } = b.kind {
            let e =
                strip.effect(&b.name).ok_or_else(|| bad(format!("AudioUnit box {} has no strip effect", b.name)))?;
            if !b.matches_unit(&e.unit) {
                return Err(bad(format!("box {} and its strip effect name different units", b.name)));
            }
        }
        Ok(())
    })
}

/// Compile from a fully assembled tuning.  `Tuning::chain` is the checked
/// graph/strip join, so callers do not need to retain the raw strip.
pub fn compile_tuning(tuning: &Tuning) -> Result<Wiring> {
    compile_checked(&tuning.graph, |b| {
        if let BoxKind::AudioUnit { .. } = b.kind {
            let stage = tuning
                .chain
                .iter()
                .find(|s| s.name == b.name)
                .ok_or_else(|| bad(format!("AudioUnit box {} is absent from the checked chain", b.name)))?;
            let unit = stage
                .unit
                .as_ref()
                .ok_or_else(|| bad(format!("AudioUnit box {} has no checked strip unit", b.name)))?;
            if !b.matches_unit(unit) {
                return Err(bad(format!("box {} and its checked strip stage name different units", b.name)));
            }
        }
        Ok(())
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{AuPreset, ParamState, UnitId};
    use crate::dspg;
    use crate::fourcc::FourCc;
    use crate::plist::Dict;
    use crate::strip::Effect;
    use std::collections::BTreeMap;

    fn cc(s: &str) -> FourCc {
        FourCc::from_str_exact(s).unwrap()
    }

    fn effect(name: &str, subtype: &str) -> Effect {
        let unit = UnitId { kind: cc("aufx"), subtype: cc(subtype), manufacturer: cc("appl") };
        Effect {
            box_name: name.into(),
            bypass: false,
            unit,
            preset: AuPreset {
                unit,
                name: None,
                version: None,
                has_state: true,
                state: ParamState::default(),
                extra: Dict::new(),
            },
        }
    }

    fn graph(extra: &str) -> (Graph, AuStrip) {
        let text = format!(
            "box V (aufx vrq2) 1 1\nbox T (aufx tmst) 1 1\nbox G dbgn 1 1\nbox VT (aufx volt) 1 1\nbox C (aufx mstk) 1 1\nparam mvsc 1\nparam mvol 0\nwireGraphParam mvsc (V 5)\nwireGraphParam mvol (G 0)\n{extra}"
        );
        let g = dspg::parse(&text, &BTreeMap::new(), "volume test").unwrap();
        let strip = AuStrip {
            strips: vec![vec![effect("V", "vrq2"), effect("T", "tmst"), effect("VT", "volt"), effect("C", "mstk")]],
        };
        (g, strip)
    }

    #[test]
    fn compiles_scalar_db_and_taper_relay() {
        let (g, s) =
            graph("wireGraphParam mvsc (T 3 0 0)\nwireGraphParam mvsc (VT 4 0 0)\nwireParam (VT 100 0 0) (C 79 0 0)");
        let w = compile(&g, &s).unwrap();
        assert_eq!(w.direct.len(), 4);
        assert_eq!(w.direct[0].binding.parameter, Parameter::VarEqCurrentGain);
        assert_eq!(w.direct[1].binding.parameter, Parameter::GainDb);
        assert_eq!(w.relays.len(), 1);
        assert_eq!(w.relays[0].from_parameter, Parameter::VolumeTaperOutput);
        assert_eq!(w.relays[0].to_parameter, Parameter::CompressorVolume);
    }

    #[test]
    fn refuses_unknown_volume_target_and_wrong_strip_unit() {
        let (g, s) = graph("wireGraphParam mvsc (C 999)");
        assert!(compile(&g, &s).unwrap_err().to_string().contains("uncarried mvsc"));

        let (g, mut s) = graph("");
        s.strips[0][0] = effect("V", "axtc");
        assert!(compile(&g, &s).unwrap_err().to_string().contains("different units"));
    }
}
