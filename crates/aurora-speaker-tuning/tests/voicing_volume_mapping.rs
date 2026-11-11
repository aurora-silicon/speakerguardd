// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The system-volume mapping the macOS parity gate attests at every slider:
//! which box parameter receives the user scalar (`mvsc`) and which the HAL
//! dB (`mvol`), and the dB the volume law gives each gated slider. A wrong
//! input type here moves a volume-dependent stage (Tone automation, LowFlow,
//! the gain box) only below full scale, where the full-volume gate is blind.
//! Synthetic graphs and ranges only: no tuning data.

use aurora_speaker_tuning::aupreset::{AuPreset, ParamState, UnitId};
use aurora_speaker_tuning::dspg;
use aurora_speaker_tuning::fourcc::FourCc;
use aurora_speaker_tuning::plist::Dict;
use aurora_speaker_tuning::strip::{AuStrip, Effect};
use aurora_speaker_tuning::voicing::volume::compile;
use sgv2_core::voicing::volume::{Curve, Input, Parameter};
use std::collections::BTreeMap;

fn effect(name: &str, subtype: &str) -> Effect {
    let cc = |s: &str| FourCc::from_str_exact(s).unwrap();
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

/// Every volume consumer the graphs use, wired as the speaker graphs wire them.
const GRAPH: &str = "\
box V (aufx vrq2) 1 1
box T (aufx tmst) 1 1
box G dbgn 1 1
box N (aufx ldnm) 1 1
box X (aufx axtc) 1 1
box F (aufx flow) 1 1
box VT (aufx volt) 1 1
box C (aufx mstk) 1 1
box T2 (aufx tmst) 1 1
box F2 (aufx flow) 1 1
param mvsc 1
param mvol 0
wireGraphParam mvsc (V 5)
wireGraphParam mvsc (T 3)
wireGraphParam mvol (G 0)
wireGraphParam mvol (N 1)
wireGraphParam mvol (X 16)
wireGraphParam mvol (F 4)
wireGraphParam mvsc (VT 4)
wireParam (VT 100 0 0) (C 79 0 0)
wireParam (VT 100 0 0) (T2 3 0 0)
wireParam (VT 100 0 0) (F2 4 0 0)
";

fn wiring() -> aurora_speaker_tuning::voicing::volume::Wiring {
    let g = dspg::parse(GRAPH, &BTreeMap::new(), "volume mapping").unwrap();
    let strip = AuStrip {
        strips: vec![vec![
            effect("V", "vrq2"),
            effect("T", "tmst"),
            effect("N", "ldnm"),
            effect("X", "axtc"),
            effect("F", "flow"),
            effect("VT", "volt"),
            effect("C", "mstk"),
            effect("T2", "tmst"),
            effect("F2", "flow"),
        ]],
    };
    compile(&g, &strip).unwrap()
}

#[test]
fn each_consumer_gets_the_scalar_or_the_db_it_reads() {
    let w = wiring();
    let got: Vec<(&str, Input, Parameter)> =
        w.direct.iter().map(|d| (d.box_name.as_str(), d.binding.input, d.binding.parameter)).collect();
    assert_eq!(
        got,
        vec![
            ("V", Input::Scalar, Parameter::VarEqCurrentGain),
            ("T", Input::Scalar, Parameter::ToneSystemVolume),
            ("G", Input::Decibels, Parameter::GainDb),
            ("N", Input::Decibels, Parameter::NormSystemVolume),
            ("X", Input::Decibels, Parameter::XtcSystemVolume),
            ("F", Input::Decibels, Parameter::LowFlowSystemVolume),
            ("VT", Input::Scalar, Parameter::VolumeTaperScalar),
        ]
    );
    // the taper's published gain, not the slider, feeds Comp, a relayed Tone and a relayed LowFlow
    let relays: Vec<(&str, Parameter, Parameter)> =
        w.relays.iter().map(|r| (r.to_box.as_str(), r.from_parameter, r.to_parameter)).collect();
    assert_eq!(
        relays,
        vec![
            ("C", Parameter::VolumeTaperOutput, Parameter::CompressorVolume),
            ("T2", Parameter::VolumeTaperOutput, Parameter::ToneSystemVolume),
            ("F2", Parameter::VolumeTaperOutput, Parameter::LowFlowSystemVolume),
        ]
    );
}

#[test]
fn a_consumer_wired_to_the_other_representation_is_refused() {
    // Tone automation reads the scalar: an mvol (dB) wire to it would feed -20 for "0.46"
    for (line, what) in [
        ("wireGraphParam mvol (T 3)", "mvol wire to T"),
        ("wireGraphParam mvsc (G 0)", "mvsc wire to G"),
        ("wireGraphParam mvsc (F 4)", "mvsc wire to F"),
        ("wireGraphParam mvol (V 5)", "mvol wire to V"),
    ] {
        let text = format!("{}{line}\n", GRAPH);
        let g = dspg::parse(&text, &BTreeMap::new(), "volume mapping").unwrap();
        let strip = AuStrip {
            strips: vec![vec![
                effect("V", "vrq2"),
                effect("T", "tmst"),
                effect("N", "ldnm"),
                effect("X", "axtc"),
                effect("F", "flow"),
                effect("VT", "volt"),
                effect("C", "mstk"),
                effect("T2", "tmst"),
                effect("F2", "flow"),
            ]],
        };
        let e = compile(&g, &strip).unwrap_err().to_string();
        assert!(e.contains("uncarried"), "{what}: {e}");
    }
}

#[test]
fn the_gated_sliders_map_through_the_hal_law() {
    // A synthetic range. dB = min * (1 - sqrt(s)) for a range ending at 0 dB.
    let c = Curve::hal(-40.0, 0.0).unwrap();
    for (s, want) in
        [(1.0, 0.0), (0.7, -40.0 * (1.0 - 0.7f64.sqrt())), (0.46, -40.0 * (1.0 - 0.46f64.sqrt())), (0.25, -20.0)]
    {
        let f = c.map(s);
        assert_eq!(f.scalar, s);
        assert!((f.db - want).abs() < 1e-12, "slider {s}: {} vs {want}", f.db);
        // each binding takes its own representation of the same slider
        assert_eq!(f.value(Input::Scalar), s);
        assert_eq!(f.value(Input::Decibels), f.db);
    }
    // the law is monotone over the gated sliders and never adds drive
    let db: Vec<f64> = [0.25, 0.46, 0.7, 1.0].iter().map(|&s| c.db(s)).collect();
    assert!(db.windows(2).all(|w| w[0] < w[1]) && db[3] == 0.0);
}
