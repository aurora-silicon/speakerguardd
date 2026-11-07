// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The units-and-semantics table: every field the reader exposes, with its
//! unit and meaning as the behaviour specification states them.  Entries
//! whose unit or meaning the specification leaves open say `TODO(SPEC)`.

use crate::names;
use crate::spp3::{CHANNEL_KEYS, CL_FIELDS, GLOBAL_FIELDS, OL_DISP_FIELDS, OL_THERMAL_FIELDS};
use serde::Serialize;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Row {
    pub source: &'static str,
    pub field: String,
    pub unit: &'static str,
    pub meaning: String,
}

pub fn table() -> Vec<Row> {
    let mut rows = Vec::new();
    for (src, defs) in [
        ("spp3 sidecar (global)", GLOBAL_FIELDS),
        ("spp3 sidecar ChannelN", CHANNEL_KEYS),
        ("spp3 sidecar ChannelN.OL_thermal", OL_THERMAL_FIELDS),
        ("spp3 sidecar ChannelN.OL_disp", OL_DISP_FIELDS),
        ("spp3 sidecar ChannelN.CL", CL_FIELDS),
    ] {
        for f in defs {
            rows.push(Row { source: src, field: f.key.to_string(), unit: f.unit, meaning: f.meaning.to_string() });
        }
    }
    for (src, t) in [
        ("spp3 saved state, global scope 0", names::SPP3_GLOBAL),
        ("spp3 meters, global scope 0", names::SPP3_GLOBAL_METERS),
        ("spp3 saved state, part scope 4", names::SPP3_PART),
        ("spp3 meters, part scope 4", names::SPP3_PART_METERS),
        ("atsp saved state, global scope 0", names::ATSP_GLOBAL),
    ] {
        for p in t {
            let meaning = match p.sidecar {
                Some(s) => format!("{} (sidecar {s}; the sidecar applies)", p.name),
                None => p.name.to_string(),
            };
            rows.push(Row { source: src, field: format!("ID {}", p.id), unit: p.unit, meaning });
        }
    }
    rows.push(Row {
        source: "SpTS record",
        field: "R_cal".into(),
        unit: "ohm",
        meaning: "factory DC resistance, raw * 0.001".into(),
    });
    rows.push(Row {
        source: "SpTS record",
        field: "T_cal".into(),
        unit: "degC",
        meaning: "calibration temperature, raw * 0.01".into(),
    });
    rows.push(Row {
        source: "SpTS record",
        field: "data_source".into(),
        unit: "FourCC",
        meaning: "provenance only; record i calibrates unit channel i".into(),
    });
    rows.push(Row {
        source: "SpTS record",
        field: "fs qts le kms mms bl rms".into(),
        unit: "scaled",
        meaning: "the other Thiele-Small fields (x0.1, x0.001, x0.01, x1, x0.0001, x0.0001, x0.001); dump only".into(),
    });
    rows.push(Row {
        source: "lsm2 saved state",
        field: "all IDs".into(),
        unit: "TODO(SPEC)",
        meaning: "ID-to-name map not yet in the specification; exposed raw".into(),
    });
    rows
}
