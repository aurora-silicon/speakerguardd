// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//! Private intermediate producer, built only by aid-inventory.py's temporary overlay.
//! Never publish its stdout: diagnostics may contain tuning values.
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::voicing::{chain, topology};
use aurora_speaker_tuning::Diagnostics;
use serde_json::{json, Value};
use sgv2_core::voicing::chain::{Chain, GraphParam, Kind};

fn graph_boxes(b: &Bundle) -> aurora_speaker_tuning::error::Result<Vec<Value>> {
    use aurora_speaker_tuning::{dspg, select, strip::AuStrip};
    let selected = select::speaker_chain(b, b.manifest.speaker_id.map(i128::from), &mut Diagnostics::default())?;
    let bytes =
        b.file(&selected.graph).ok_or_else(|| aurora_speaker_tuning::Error::missing("inventory", "selected graph"))?;
    let text =
        std::str::from_utf8(bytes).map_err(|_| aurora_speaker_tuning::Error::format("inventory", "graph UTF-8"))?;
    let graph = dspg::parse(text, &selected.substitutions, &selected.graph)?;
    let strip = b.file(&selected.strip).and_then(|bytes| AuStrip::parse(bytes, &selected.strip).ok());
    Ok(graph
        .boxes
        .iter()
        .map(|bx| {
            let subtype = match &bx.kind {
                dspg::BoxKind::AudioUnit { subtype, .. } => subtype.to_string(),
                dspg::BoxKind::Builtin(k) => k.clone(),
            };
            let effect = strip.as_ref().and_then(|strip| strip.effect(&bx.name));
            let channels = graph
                .wires
                .iter()
                .find(|w| w.src.node == bx.name && w.src.port == 0)
                .and_then(|w| dspg::analysis::wire_channels(&graph, w).ok());
            let mut entry = json!({"name":bx.name,"subtype":subtype,"input_ports":bx.inputs,"output_ports":bx.outputs,
            "bypass":effect.is_some_and(|e| e.bypass),"channels":channels,
            "before_protection":false,"protection":matches!(subtype.as_str(),"spp3"|"atsp"|"lsm2")});
            if subtype == "mstk" {
                if let Some(g) = effect.and_then(|e| e.preset.state.global()) {
                    let flavors: Vec<_> = (0..4)
                        .filter(|i| g.get(71 + 2 * i).unwrap_or(0.0) != 0.0)
                        .map(|i| g.get(72 + 2 * i).unwrap_or(0.0) as u32)
                        .collect();
                    entry["mstk"] = json!({"algorithm":g.get(70).unwrap_or(0.0) as u32,
                    "quality":g.get(64).unwrap_or(0.0) as u32,"ovs":g.get(69).unwrap_or(0.0) as u32,
                    "enabled_saturation_flavors":flavors});
                }
            }
            if subtype == "tmst" {
                if let Some(g) = effect.and_then(|e| e.preset.state.global()) {
                    let mut slots = Vec::new();
                    for i in 0..aurora_speaker_tuning::prestage::SLOTS {
                        let at = 14 + 10 * i;
                        if let (Some(state), Some(kind)) = (g.get(at), g.get(at + 1)) {
                            let group = g.get(at + 9).unwrap_or(0.0) as u32;
                            let bypass = g.get(1).unwrap_or(0.0) as u32;
                            slots.push(json!({"slot":i,"type":kind as u32,
                            "enabled":state as u32 & 2 != 0 && g.get(0).unwrap_or(0.0)==0.0,
                            "group_bypassed":group<32 && bypass>>group & 1 != 0}));
                        }
                    }
                    entry["tmst_slots"] = json!(slots);
                }
            }
            entry
        })
        .collect())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let bundle = if args[1] == "--bundle" {
        Bundle::open(args[2].as_ref())
    } else {
        Bundle::from_tunings_root(args[2].as_ref(), args[4].parse().expect("aid"))
    };
    let mut diags = Diagnostics::default();
    let mut fallback = Vec::new();
    let mut fallback_aid = None;
    let t = match bundle.and_then(|b| {
        fallback_aid = Some(b.manifest.acoustic_id);
        fallback = graph_boxes(&b).unwrap_or_default();
        Tuning::load(&b, &Options::default(), &mut diags)
    }) {
        Ok(t) => t,
        Err(e) => {
            let unit = fallback.iter().find(|b| b["protection"] == true).map(|b| b["subtype"].clone());
            println!(
                "{}",
                json!({"load_error": e.to_string(), "diagnostics": diags,
                "aid":fallback_aid,"unit":unit,"boxes":fallback})
            );
            return;
        }
    };
    let mut boxes = Vec::new();
    for stage in &t.chain {
        let subtype = stage.unit.map(|u| u.subtype.to_string()).unwrap_or_else(|| stage.kind.clone());
        let mut b = json!({"name":stage.name, "subtype":subtype, "bypass":stage.bypass,"channels":stage.channels,
            "input_ports":stage.inputs,"output_ports":stage.outputs,
            "before_protection": stage.before_protection, "protection":stage.name == t.protection.box_name});
        if let Some(g) = stage.state.as_ref().and_then(|s| s.global()) {
            if subtype == "mstk" {
                // Discrete controls only, never coefficients, thresholds or gains.
                let flavors: Vec<_> = (0..4)
                    .filter(|i| g.get(71 + 2 * i).unwrap_or(0.0) != 0.0)
                    .map(|i| g.get(72 + 2 * i).unwrap_or(0.0) as u32)
                    .collect();
                b["mstk"] = json!({"algorithm":g.get(70).unwrap_or(0.0) as u32,
                    "enabled_saturation_flavors":flavors,
                    "quality":g.get(64).unwrap_or(0.0) as u32,"ovs":g.get(69).unwrap_or(0.0) as u32});
            }
            if subtype == "tmst" {
                let mut slots = Vec::new();
                for i in 0..aurora_speaker_tuning::prestage::SLOTS {
                    let at = 14 + 10 * i;
                    if let (Some(state), Some(kind)) = (g.get(at), g.get(at + 1)) {
                        let enabled = state as u32 & 2 != 0 && g.get(0).unwrap_or(0.0) == 0.0;
                        let group = g.get(at + 9).unwrap_or(0.0) as u32;
                        let bypass = g.get(1).unwrap_or(0.0) as u32;
                        slots.push(json!({"slot":i,"type":kind as u32,"enabled":enabled,
                            "group_bypassed":group < 32 && bypass >> group & 1 != 0}));
                    }
                }
                b["tmst_slots"] = json!(slots);
            }
        }
        boxes.push(b);
    }
    for bx in fallback {
        if matches!(bx["subtype"].as_str(), Some("lsid" | "dbcontrol"))
            && !boxes.iter().any(|b| b["name"] == bx["name"])
        {
            boxes.push(bx);
        }
    }
    let mut result = json!({"aid":t.acoustic_id,"unit":t.protection.kind,"boxes":boxes,
        "binding":{"board":t.board,"unit_id":t.unit_id}});
    match topology::plan(&t) {
        Ok(p) => result["plan"] = json!(p),
        Err(e) => result["plan_error"] = json!(e.to_string()),
    }
    match chain::compile(&t, 48_000.0, &mut diags) {
        Err(e) => result["chain_error"] = json!(e.to_string()),
        Ok(c) => {
            result["stubs"] = json!(c.stubs);
            result["gaps"] = json!(c.gaps);
            result["unwired"] = json!(c.unwired);
            let mut kinds = Vec::new();
            for (n, spec) in c.plan.nodes.iter().zip(&c.spec.nodes) {
                let (kind, configured): (&str, Vec<&str>) = match &spec.kind {
                    Kind::Bypass => ("bypass", vec![]),
                    Kind::Stub(_) => ("stub", vec![]),
                    Kind::Gain => ("gain", vec!["gain-positive-clamp"]),
                    Kind::NonFinite => ("nonfinite", vec![]),
                    Kind::Mstk(_) => ("mstk", vec!["input-amplitude-bound"]),
                    Kind::Cfreak(p) => {
                        let mut bounds = vec!["cfrk-slow-release-TODO(SPEC)", "cfrk-ceiling-trim"];
                        if p.true_peak {
                            bounds.push("cfrk-true-peak-scale");
                        }
                        ("cfrk", bounds)
                    }
                    Kind::Lowflow(p) => {
                        let active = p.models[..p.model_count].iter().any(|m| m.enabled && !m.mute);
                        (
                            "flow",
                            if active && sgv2_core::voicing::lowflow::CONSERVATIVE {
                                vec!["lowflow-conservative"]
                            } else {
                                vec![]
                            },
                        )
                    }
                    Kind::Tone(p) => {
                        let mut bounds = vec!["input-amplitude-bound"];
                        if p.dynamic
                            .iter()
                            .any(|d| d.params.kind == sgv2_core::prestage::DynamicKind::HighShelf && !d.params.smooth)
                        {
                            bounds.push("tmst-type18-TODO(SPEC)");
                        }
                        ("tmst", bounds)
                    }
                    _ => ("carried", vec![]),
                };
                let mut metadata = json!({"name":n.name,"kind":kind,"configured_bounds":configured});
                if let Kind::Tone(p) = &spec.kind {
                    let mut counts = std::collections::BTreeMap::<u32, usize>::new();
                    for d in &p.dynamic {
                        *counts.entry(d.params.kind as u32).or_default() += 1;
                    }
                    metadata["compiled_dynamic_type_counts"] = json!(counts);
                }
                kinds.push(metadata);
            }
            result["compiled_boxes"] = json!(kinds);
            let pc = c.spec.program_channels;
            let lanes = c.spec.outputs.len();
            let mut run = Chain::new(c.spec, 48_000.0, 512).expect("compiled spec");
            run.set_param(GraphParam::VolumeScalar, 1.0);
            run.set_param(GraphParam::VolumeDb, 0.0);
            sgv2_core::aid_inventory_trace::reset();
            // Exactly the same deterministic program is sent to voicing-render.
            // Offline stress/quiet/release segments; there is no hardware output.
            let mut input = vec![vec![0.0f32; 512]; pc];
            let mut output = vec![vec![0.0f32; 512]; lanes];
            let mut seed = 1u32;
            for block in 0..96 {
                for i in 0..512 {
                    seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                    let amplitude = if (16..32).contains(&block) {
                        32.0
                    } else if block < 48 {
                        0.1
                    } else {
                        0.0
                    };
                    let sample = ((seed >> 8) as f32 / 8388608.0 - 1.0) * amplitude;
                    for (ch, channel) in input.iter_mut().enumerate() {
                        channel[i] = if ch & 1 == 0 { sample } else { -sample * 0.75 };
                    }
                }
                let ins: Vec<_> = input.iter().map(|x| x.as_slice()).collect();
                let mut outs: Vec<_> = output.iter_mut().map(|x| x.as_mut_slice()).collect();
                run.process(&ins, &mut outs, 512);
            }
            result["trace"] = Value::Array(
                (0..c.plan.nodes.len())
                    .map(|i| json!({"name":c.plan.nodes[i].name,"mask":sgv2_core::aid_inventory_trace::get(i)}))
                    .collect(),
            );
        }
    }
    result["diagnostics"] = json!(diags);
    println!("{result}");
}
