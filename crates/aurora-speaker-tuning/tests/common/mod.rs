// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Synthetic bundles, generated at test time.  Every value here is made up:
//! the structure mirrors a tuning bundle, the numbers describe no real Mac.

#![allow(dead_code)]

use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::fourcc::FourCc;
use aurora_speaker_tuning::manifest::sha256_hex;
use aurora_speaker_tuning::plist::{binary, Dict, Value};
use aurora_speaker_tuning::spts;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

pub const AID: &str = "AID901";
pub const ACOUSTIC_ID: u32 = 901;

pub fn fcc(s: &str) -> i128 {
    FourCc::from_str_exact(s).unwrap().0 as i128
}

pub fn dict(entries: Vec<(&str, Value)>) -> Value {
    let mut d = Dict::new();
    for (k, v) in entries {
        assert!(d.insert_unique(k.to_string(), v));
    }
    Value::Dict(d)
}

pub fn r(x: f64) -> Value {
    Value::Real(x)
}
pub fn i(x: i128) -> Value {
    Value::Integer(x)
}
pub fn s(x: &str) -> Value {
    Value::String(x.to_string())
}

/// The fixture speaker volume range's floor: synthetic, not any Mac's.
pub const FIXTURE_VOLUME_MIN_DB: f64 = -48.5;

/// A device configuration (`Config/AID<n>.plist`) whose speaker states a
/// volume range of `[min_db, 0]`.
pub fn device_config(min_db: f64) -> Value {
    let params = dict(vec![("OutputVolumeMax", i(0)), ("OutputVolumeMin", r(min_db))]);
    dict(vec![("DSP Devices", dict(vec![("Built-in Speaker", dict(vec![("Parameters", params)]))]))])
}

/// A four-channel graph: stereo duplicated to [L, L, R, R] lanes, permuted
/// to [L, R, L, R] before the protection unit, with an 8-channel telemetry
/// bus through a converter.  Names are this test's own.
pub fn graph_text(unit: &str) -> String {
    format!(
        r#"graphName "test_speaker_out"
; synthetic graph
[def chIn 2]
[def chOut 4] ; four drivers
[def blk 256]
[def USE_ALT 0]
in in
in sense
out out
box Guard nfnp 1 1
box Eq (aufx vrq2) 1 1
box Spl split 1 2
box Up join 4 1
box Mid split 1 4
box Perm join 4 1
[if [== [USE_ALT] 1] {{
box Prot dbgn 1 1
}}{{
box Prot (aufx {unit}) 2 1
}}]
box Conv srlconverter 1 1
box Fade dbgn 1 1
format senseFmt ([fs] 8i [blk])
wire in Guard ([fs] [chIn])
wire Guard Eq ([fs] [chIn])
wire Eq Spl ([fs] [chIn])
wire (Spl 0) (Up 0) ([fs] 1)
wire (Spl 0) (Up 1) ([fs] 1)
wire (Spl 1) (Up 2) ([fs] 1)
wire (Spl 1) (Up 3) ([fs] 1)
wire Up Mid ([fs] [chOut])
wire (Mid 0) (Perm 0) ([fs] 1)
wire (Mid 2) (Perm 1) ([fs] 1)
wire (Mid 1) (Perm 2) ([fs] 1)
wire (Mid 3) (Perm 3) ([fs] 1)
wire Perm (Prot 0) ([fs] [chOut] [blk])
wire sense Conv senseFmt
wire Conv (Prot 1) senseFmt
wire Prot Fade ([fs] [chOut])
wire Fade out ([fs] [chOut])
param mvol 0
[def x 1]
param gpow 0 out
wireGraphParam gpow (Prot 15 global 0)
property tdso in
wireGraphProperty tdso (Prot 64005 0 0)
"#
    )
}

pub fn preset(sub: &str, name: &str, state: &ParamState) -> Value {
    dict(vec![
        ("data", Value::Data(state.encode())),
        ("manufacturer", i(fcc("appl"))),
        ("name", s(name)),
        ("subtype", i(fcc(sub))),
        ("type", i(fcc("aufx"))),
        ("version", i(0)),
    ])
}

pub fn effect(box_name: &str, sub: &str, preset_v: Value) -> Value {
    dict(vec![
        ("aupreset", preset_v),
        ("bypass", i(0)),
        ("displayname", s(box_name)),
        ("unit", dict(vec![("manufacturer", i(fcc("appl"))), ("subtype", i(fcc(sub))), ("type", i(fcc("aufx")))])),
    ])
}

pub fn spp3_state(channels: u32) -> ParamState {
    let mut sections =
        vec![ParamSection { scope: 0, element: 0, params: vec![(0, 1.0), (2, 3.5), (3, 11.0), (4, 41.0)] }];
    for e in 0..channels {
        sections.push(ParamSection {
            scope: 4,
            element: e,
            params: vec![(0, 121.0), (2, 17.0), (9, 3.3), (21, 1.0), (22, 1.0)],
        });
    }
    ParamState { sections }
}

pub fn strip(unit: &str, channels: u32) -> Vec<u8> {
    strip_with_state(unit, &spp3_state(channels))
}

pub fn strip_with_state(unit: &str, state: &ParamState) -> Vec<u8> {
    let eq = ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: vec![(1, -2.5), (2, 900.0)] }] };
    let builtin = dict(vec![
        ("classname", s("ChannelSplitter")),
        ("manufacturer", i(fcc("tsta"))),
        ("subtype", i(fcc("cspl"))),
        ("type", i(fcc("aufx"))),
        ("version", i(0)),
    ]);
    let builtin_effect = dict(vec![
        ("aupreset", builtin),
        ("bypass", i(0)),
        ("displayname", s("Spl")),
        ("unit", dict(vec![("manufacturer", i(fcc("tsta"))), ("subtype", i(fcc("cspl"))), ("type", i(fcc("aufx")))])),
    ]);
    binary::write(&dict(vec![(
        "strips",
        Value::Array(vec![dict(vec![(
            "effects",
            Value::Array(vec![
                effect("Eq", "vrq2", preset("vrq2", "eq_test", &eq)),
                builtin_effect,
                effect("Prot", unit, preset(unit, "prot_test", state)),
            ]),
        )])]),
    )]))
}

pub fn propstrip(sidecar: &str) -> Vec<u8> {
    propstrip_with(sidecar, Some(2))
}

/// The property strip, with `Thermal State Override` (64007) if given.
pub fn propstrip_with(sidecar: &str, thermal_state: Option<i128>) -> Vec<u8> {
    propstrip_extra(sidecar, thermal_state, Vec::new())
}

/// The property strip with further property dictionaries.
pub fn propstrip_extra(sidecar: &str, thermal_state: Option<i128>, extra: Vec<Value>) -> Vec<u8> {
    let mut props = vec![dict(vec![("Number", i(64004)), ("UInt32", i(1))])];
    props.extend(extra);
    if let Some(v) = thermal_state {
        props.push(dict(vec![("Number", i(64007)), ("UInt32", i(v))]));
    }
    props.push(dict(vec![
        ("Number", i(64003)),
        ("Path", s(&format!("/System/Library/Audio/Tunings/{AID}/DSP/Strips/{sidecar}"))),
    ]));
    binary::write(&dict(vec![(
        "Boxes",
        Value::Array(vec![dict(vec![("Name", s("Prot")), ("Properties", Value::Array(props))])]),
    )]))
}

pub fn channel(name: &str, group: i128, cl: bool) -> Value {
    let ol_thermal = dict(vec![
        ("HardTempLimitHeadroom", r(7.0)),
        ("Rampout", i(0)),
        ("Reb_ref", r(3.3)),
        ("Rshunt", r(0.05)),
        ("T_sett_mg", r(19.0)),
        ("T_sett_vc", r(17.0)),
        ("TemperatureLimit", i(121)),
        ("ThermalFFSpeedupFactor", r(0.5)),
        ("tau_Tmg", r(90.0)),
        ("tau_Tvc", r(2.5)),
    ]);
    let ol_disp = dict(vec![
        ("Q_d", r(1.1)),
        ("ag", r(11.89)),
        ("ct", r(0.3)),
        ("g_bw", i(1)),
        ("kt", i(2000)),
        ("mt", r(0.0002)),
        ("phi", r(0.8)),
        ("x_lim", r(0.3)),
    ]);
    let clb = dict(vec![
        ("CalTempRangeMax", r(45.0)),
        ("CalTempRangeMin", i(15)),
        ("DisplacementProtectionType", i(0)),
        ("IMONScale", r(2.0)),
        ("IPwrThres", r(1e-6)),
        ("LPMLSLowerCorner", r(0.2)),
        ("LPMLSPostGain", r(1.0)),
        ("LPMLSPreGain", r(1.0)),
        ("PilotAmplHi_dB", i(-33)),
        ("PilotAmplLo_dB", r(-44.0)),
        ("PilotDecayTime", r(0.04)),
        ("PilotDecayTimeStage2", r(0.04)),
        ("PilotEnableThres", r(0.05)),
        ("PilotFreq", r(51.0)),
        ("PilotLowerThres", r(70.0)),
        ("PilotUpperThres", i(85)),
        ("RdcScale", r(1.0)),
        ("RebRangeMax", r(6.0)),
        ("RebRangeMin", r(2.0)),
        ("TempSenseSmoothingTau", r(2.0)),
        ("TempSenseWindowTime", r(0.2)),
        ("ThermalMeasurementMethod", i(1)),
        ("TlimAdjCap", r(40.0)),
        ("TlimErrDecayTime", r(25.0)),
        ("VMONScale", i(10)),
        ("VPwrThres", r(0.002)),
        ("a_t_20C", r(0.0039)),
        ("a_t_35C", r(0.0037)),
    ]);
    let mut e = vec![("IgnoreTelemetry", Value::Bool(false)), ("OL_disp", ol_disp), ("OL_thermal", ol_thermal)];
    if cl {
        e.push(("CL", clb));
    }
    e.push(("SpeakerGroup", i(group)));
    e.push(("SpeakerName", s(name)));
    dict(e)
}

pub fn sidecar_value(channels: usize) -> Value {
    let names = ["A", "B", "C", "D", "E", "F"];
    let csp: Vec<(String, Value)> =
        (0..channels).map(|c| (format!("Channel{c}"), channel(names[c], (c / 2) as i128, true))).collect();
    let mut d = Dict::new();
    for (k, v) in csp {
        d.insert_unique(k, v);
    }
    dict(vec![
        ("ChannelSpecificParams", Value::Dict(d)),
        ("Disp_FBIntegrationTime_s", r(2e-5)),
        ("Disp_PeakAttackTime_s", r(0.0002)),
        ("Disp_PeakDecayTime_s", r(0.02)),
        ("EnableCPMSClosedLoopControl", Value::Bool(false)),
        ("EnableCPMSControl", Value::Bool(false)),
        ("EnableDisplacementControl", Value::Bool(false)),
        ("NumChannels", i(channels as i128)),
        ("Thermal_AmbientTemp", r(41.0)),
        ("Thermal_GainAttackTime_s", r(3.5)),
        ("Thermal_GainReleaseTime_s", r(11.0)),
        ("Thermal_LookaheadDelay_ms", r(1.5)),
        ("Thermal_SafeTlim", i(90)),
        ("Thermal_SafeTlimOffset", r(-9.0)),
        ("Thermal_SafeTlimTimeMin", r(8.0)),
        ("Version", i(9)),
    ])
}

pub fn database(spids: &[(i128, &str)]) -> Vec<u8> {
    let mut cfgs = Dict::new();
    for (spid, strip) in spids {
        let name = if *spid == 65535 { "48k - General".to_string() } else { format!("48k - General - V{spid}") };
        cfgs.insert_unique(
            name,
            dict(vec![
                (
                    "DSPDeviceProperties",
                    Value::Array(vec![dict(vec![("Selector", s("cdes")), ("Value", dict(vec![("kind", i(0))]))])]),
                ),
                (
                    "Output",
                    Value::Array(vec![dict(vec![(
                        "Hardware",
                        Value::Array(vec![
                            dict(vec![
                                ("Path", s(&format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"))),
                                ("Substitutions", dict(vec![("fs", s("48000"))])),
                                ("Type", s("DSPGraph")),
                            ]),
                            dict(vec![("Path", s(&format!("{AID}/DSP/Strips/{strip}"))), ("Type", s("AUStrip"))]),
                            dict(vec![
                                ("Path", s(&format!("{AID}/DSP/Strips/builtin_speaker_out_general.propstrip"))),
                                ("Type", s("PropertyStrip")),
                            ]),
                        ]),
                    )])]),
                ),
                (
                    "UnderlyingDeviceProperties",
                    Value::Array(vec![
                        dict(vec![("Selector", s("nsrt")), ("Value", i(48000))]),
                        dict(vec![("Selector", s("spid")), ("Value", i(*spid))]),
                    ]),
                ),
            ]),
        );
        // A non-General configuration of the same device, to be ignored.
    }
    cfgs.insert_unique(
        "48k - Voice".into(),
        dict(vec![(
            "DSPDeviceProperties",
            Value::Array(vec![dict(vec![("Selector", s("cdes")), ("Value", dict(vec![("kind", i(1))]))])]),
        )]),
    );
    binary::write(&dict(vec![(AID, dict(vec![("Built-in Speaker", Value::Dict(cfgs))]))]))
}

/// All files of a synthetic spp3 bundle, by bundle path (no manifest).
pub fn spp3_files(channels: usize) -> BTreeMap<String, Vec<u8>> {
    let mut f = BTreeMap::new();
    let sc = format!("{}-aufx-spp3-appl.plist", AID.to_lowercase());
    f.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph_text("spp3").into_bytes());
    f.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"), strip("spp3", channels as u32));
    f.insert(format!("{AID}/DSP/Strips/builtin_speaker_out_general.propstrip"), propstrip(&sc));
    f.insert(format!("{AID}/DSP/Strips/{sc}"), binary::write(&sidecar_value(channels)));
    f.insert("DeviceDSPDatabase.plist".into(), database(&[(65535, "builtin_speaker_out_general.austrip")]));
    f.insert(format!("Config/{AID}.plist"), binary::write(&device_config(FIXTURE_VOLUME_MIN_DB)));
    let recs: Vec<(FourCc, u16, u16)> =
        (0..channels).map(|c| (FourCc::from_str_exact(&format!("spA{c}")).unwrap(), 3000 + c as u16, 3100)).collect();
    f.insert("speaker-thiele-small.bin".into(), spts::encode(&recs, 0xBEEF));
    f
}

/// Bundle path of the installer's lsm2 masker lattice.
pub const LATTICE: &str = "lsm2-mask-lattice.bin";
/// The installer's BuzzKill quadrature matrix file.
pub const BUZZKILL: &str = "buzzkill-quadrature.bin";

/// Made-up sections with the matrix's structure only (not macOS's values).
pub fn synthetic_buzzkill() -> [[f32; 5]; 4] {
    [
        [0.25, -0.5, 1.0, 0.125, 0.25],
        [0.375, -0.25, 1.0, -0.25, 0.375],
        [0.75, 0.5, 1.0, 0.5, -0.125],
        [1.125, -1.875, 0.875, -1.75, 0.8125],
    ]
}

pub fn buzzkill_bytes(m: &[[f32; 5]; 4]) -> Vec<u8> {
    m.iter().flatten().flat_map(|v| v.to_le_bytes()).collect()
}

/// A synthetic lsm2 masker lattice (not the macOS one): the 64 single bins,
/// then 36 evenly widening ends up to 321, as 100 little-endian u64.
pub fn synthetic_lattice_ends() -> Vec<u64> {
    let mut e: Vec<u64> = (1..=64).collect();
    e.extend((0..36u64).map(|j| 64 + ((j + 1) * 257).div_ceil(36)));
    e
}

pub fn lattice_bytes(ends: &[u64]) -> Vec<u8> {
    ends.iter().flat_map(|e| e.to_le_bytes()).collect()
}

/// The synthetic unit the fixtures are collected on: the unit ID of a made-up
/// serial number, never a real one.
pub fn test_unit() -> String {
    aurora_speaker_tuning::machine::unit_id(b"SYNTHETIC0A").unwrap()
}

/// Another synthetic unit.
pub fn other_unit() -> String {
    aurora_speaker_tuning::machine::unit_id(b"SYNTHETIC0B").unwrap()
}

/// The installer's manifest, collected on [`test_unit`] (no board named).
pub fn manifest_json(files: &BTreeMap<String, Vec<u8>>, build: &str) -> String {
    manifest_json_with(files, build, None)
}

/// The installer's schema-2 manifest, with `speaker_id` when given,
/// collected on [`test_unit`].
pub fn manifest_json_with(files: &BTreeMap<String, Vec<u8>>, build: &str, speaker_id: Option<u32>) -> String {
    manifest_json_ident(files, build, speaker_id, None, Some(&test_unit()))
}

/// The installer's schema-2 manifest with the collecting Mac's identity as
/// given (`None`: the key is left out, as in bundles collected before it).
pub fn manifest_json_ident(
    files: &BTreeMap<String, Vec<u8>>,
    build: &str,
    speaker_id: Option<u32>,
    board: Option<&str>,
    unit_id: Option<&str>,
) -> String {
    let rec = |p: &str, d: &[u8]| format!(r#"{{"path":"{p}","bytes":{},"sha256":"{}"}}"#, d.len(), sha256_hex(d));
    let spts = files.get("speaker-thiele-small.bin").map(|d| rec("speaker-thiele-small.bin", d));
    // The installer's lsm2 masker lattice record, when the files hold one.
    let lattice = files.get(LATTICE).map_or(String::new(), |d| {
        format!(
            r#","lsm2_mask_lattice":{{"path":"{LATTICE}","bytes":{},"sha256":"{}","version":1,"format":"u64le-exclusive-ends","entries":100}}"#,
            d.len(),
            sha256_hex(d)
        )
    });
    // The installer's BuzzKill matrix record, when the files hold one.
    let buzzkill = files.get(BUZZKILL).map_or(String::new(), |d| {
        format!(
            r#","buzzkill_quadrature":{{"path":"{BUZZKILL}","bytes":{},"sha256":"{}","version":1,"format":"f32le-sos4-output-major","sections":4}}"#,
            d.len(),
            sha256_hex(d)
        )
    });
    let list: Vec<String> = files
        .iter()
        .filter(|(p, _)| *p != "speaker-thiele-small.bin" && *p != LATTICE && *p != BUZZKILL)
        .map(|(p, d)| rec(p, d))
        .collect();
    let count = files.keys().filter(|p| p.starts_with(&format!("{AID}/"))).count();
    let spid = speaker_id.map_or(String::new(), |v| format!(r#","speaker_id":{v}"#));
    let ident = board.map_or(String::new(), |b| format!(r#","board":"{b}""#))
        + &unit_id.map_or(String::new(), |u| format!(r#","unit_id":"{u}""#));
    format!(
        r#"{{"schema":2,"kind":"aurora-audio-tuning","source_file_count":{count},"complete":true,
"acoustic_id":{ACOUSTIC_ID},"aid":"{AID}","macos_build":"{build}",
"speaker_thiele_small":{},"device_dsp_database":"DeviceDSPDatabase.plist","config":"Config/{AID}.plist",
"files":[{}],"based_on":"test"{spid}{ident}{lattice}{buzzkill}}}"#,
        spts.unwrap_or_else(|| "null".into()),
        list.join(",")
    )
}

/// aurora-audio-dsp's collector `metadata.json` (development bundles).
pub fn collector_json(files: &BTreeMap<String, Vec<u8>>) -> String {
    let rec = |p: &str, d: &[u8]| format!(r#"{{"path":"{p}","bytes":{},"sha256":"{}"}}"#, d.len(), sha256_hex(d));
    let list: Vec<String> = files.iter().map(|(p, d)| rec(p, d)).collect();
    format!(
        r#"{{"kind":"aurora-audio-dsp tuning bundle","version":1,"machine_model":"MacTest1,1","acoustic_id_name":"{AID}","files":[{}]}}"#,
        list.join(",")
    )
}

/// A fresh empty directory under the target dir.
pub fn scratch(name: &str) -> PathBuf {
    let d = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

pub fn write_dir(dir: &Path, files: &BTreeMap<String, Vec<u8>>, manifest: &str) {
    for (p, d) in files {
        let full = dir.join(p);
        std::fs::create_dir_all(full.parent().unwrap()).unwrap();
        std::fs::write(full, d).unwrap();
    }
    std::fs::write(dir.join("manifest.json"), manifest).unwrap();
}
