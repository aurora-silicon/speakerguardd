//! Shared harness for the tuning-reader fuzz targets: the crate's own
//! synthetic fixtures (`tests/common`, nothing from a real Mac), an
//! in-memory bundle, value-level mutations, and the checks a compiled blob
//! must pass.
//!
//! Targets: `plist` (both decoders, write/parse round trips), `aupreset`
//! (saved parameter state and presets), `dspg` (the graph language and its
//! analysis), `formats` (SpTS, manifests, the lsm2 lattice, strips, the
//! device configuration's volume range, the machine identity readers) and
//! `tuning` (a synthetic spp3, lsm2 or atsp bundle with fuzzed values,
//! through `Tuning::load` and `compile`: whatever compiles must be a blob
//! the plugin runs as a protecting unit and the supervisor accepts). Run
//! from `crates/aurora-speaker-tuning` with cargo-fuzz; a stable toolchain
//! works with `RUSTC_BOOTSTRAP=1` (cargo-fuzz passes `-Zsanitizer`):
//!
//! ```text
//! RUSTC_BOOTSTRAP=1 cargo fuzz run -O tuning -- -max_total_time=900 -rss_limit_mb=1024
//! ```
//!
//! Seeds and regression inputs in the repository are synthetic only. Real
//! tuning files are Apple's: never seed a corpus that is committed from
//! them.

#[path = "../../tests/common/mod.rs"]
pub mod common;

use std::collections::BTreeMap;
use std::path::PathBuf;

use arbitrary::{Result, Unstructured};
use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::compile;
use aurora_speaker_tuning::compile::lsm2_ids as id;
use aurora_speaker_tuning::machine::Machine;
use aurora_speaker_tuning::manifest::Manifest;
use aurora_speaker_tuning::plist::Value;
use aurora_speaker_tuning::tuning::UnitKind;
use common::*;
use sgv2_lv2::{load_unit, AnyUnit, Role};

pub use arbitrary;

/// A synthetic lsm2 saved state with every axis on (the compiler tests'
/// `lsm2_state`, without its tweak).
pub fn lsm2_state() -> ParamState {
    let mut m: BTreeMap<u32, f32> = BTreeMap::new();
    m.insert(id::ENABLE_CLOSED_LOOP_TEMPERATURE, 0.0);
    m.insert(id::BYPASS_SYS_ID, 1.0);
    m.insert(1024 + 2, -40.0);
    for c in 0..8 {
        m.insert(id::CH_TUNING + c, 0.0);
        m.insert(id::CH_SPEAKER_TYPE + c, 0.0);
    }
    for mgr in 0..5 {
        for ty in 0..3 {
            for k in 0..37 {
                m.insert(id::model(mgr, ty, k), 0.0);
            }
            let mut set = |k, v| {
                m.insert(id::model(mgr, ty, k), v);
            };
            set(id::M_RDC, 5.0 + mgr as f32);
            set(id::M_ENABLE_RES1, 1.0);
            set(id::M_BL, 0.9);
            set(id::M_MMS_G, 0.3);
            set(id::M_RMS, 0.2);
            set(id::M_KMS, 1500.0 + 10.0 * mgr as f32);
        }
    }
    for ty in 0..3 {
        for (k, v) in [(id::T_RTH_VC, 30.0), (id::T_TAU_VC, 3.0), (id::T_RTH_MG, 20.0), (id::T_TAU_MG, 80.0)] {
            m.insert(id::thermal(ty, k), v);
        }
        m.insert(id::thermal(ty, id::T_AMBIENT), 45.0);
        m.insert(id::thermal(ty, id::T_ENABLE), 1.0);
    }
    for k in 0..23 {
        m.insert(id::controller(k), 1.0);
    }
    for (k, v) in [
        (id::C_ATTACK, 4.0),
        (id::C_FAST_RELEASE, 150.0),
        (id::C_SLOW_RELEASE, 15000.0),
        (id::C_LATENCY, 22.0 / 3.0),
        (id::C_FILTERBANK_LATENCY, 10.0 / 3.0),
        (id::C_LOOKAHEAD, 2.0 / 3.0),
        (id::C_UNLINK_TEMPERATURE, 0.0),
        (id::C_COMPENSATE_RESONATORS, 0.0),
        (id::C_USE_FAST_PEAK, 0.0),
    ] {
        m.insert(id::controller(k), v);
    }
    for k in 0..37 {
        m.insert(id::amp0(k), 0.0);
    }
    for j in 0..4 {
        m.insert(id::amp0(id::A_TIME_SCALE + j), [0.01, 0.2, 2.0, 30.0][j as usize]);
        m.insert(id::amp0(id::A_HARD_LIMIT + j), [50.0, 8.0, 3.0, 2.0][j as usize]);
        m.insert(id::amp0(id::A_SOFT_LIMIT + j), 100.0);
    }
    m.insert(id::amp0(id::A_LOW_IDLE), 0.02);
    m.insert(id::amp0(id::A_LOW_RATIO), 1.2);
    m.insert(id::amp0(id::A_MAX_ATTENUATION), 4.0);
    for c in 0..8 {
        for k in 0..33 {
            m.insert(id::channel(c, k), 1.0);
        }
        for (k, v) in [
            (id::CH_VOLTS_FS, 11.89),
            (id::CH_PARASITIC, 0.2),
            (id::CH_DISPLACEMENT_LIMIT, 0.0005),
            (id::CH_TEMP_HARD, 130.0),
            (id::CH_TEMP_SOFT, 115.0),
            (id::CH_MAX_VELOCITY_ATTEN, 18.0),
            (id::CH_MAX_TEMP_ATTEN, 10.0),
            (id::CH_TEMP_HEADROOM, 4.0),
            (id::CH_POWER_AMP_MODEL, 0.0),
            (id::CH_MASK_BAND_MIN, 8000.0),
            (id::CH_MASK_BAND_MAX, 12000.0),
            (id::CH_MASK_ATTACK, 0.01),
            (id::CH_MASK_RELEASE, 0.1),
            (id::CH_MASK_ATTACK_FAST_MS, 5.0),
            (id::CH_MASK_ATTACK_RANGE, 70.0),
            (id::CH_BANDWIDTH, 0.0),
        ] {
            m.insert(id::channel(c, k), v);
        }
        for (j, v) in [0.5, 0.6, 0.9].into_iter().enumerate() {
            m.insert(id::channel(c, id::CH_VELOCITY_LIMIT0 + j as u32), v);
        }
        for (j, v) in [10.0, 20.0, 30.0, 40.0].into_iter().enumerate() {
            m.insert(id::channel(c, id::CH_MASK_ENERGY0 + j as u32), v);
        }
        for (j, v) in [60.0, 300.0, 650.0, 1600.0, 5000.0].into_iter().enumerate() {
            m.insert(id::channel(c, id::CH_BAND_CENTER0 + j as u32), v);
        }
    }
    ParamState { sections: vec![ParamSection { scope: 0, element: 0, params: m.into_iter().collect() }] }
}

/// A synthetic atsp saved state (the compiler tests' `atsp_slots_from_the_id_map`).
pub fn atsp_state() -> ParamState {
    let mut params: Vec<(u32, f32)> = vec![(0, 0.0), (40, 1.0)];
    for s in 0..3u32 {
        let b = 1 + 13 * s;
        let mask = if s == 0 { 15.0 } else { 0.0 };
        let vals = [11.89, 8.5, 35.0, 2.0, 45.0, 70.0, 40.0, 125.0, 700.0, 18000.0, 9.0, 1.0, mask];
        for (k, v) in vals.iter().enumerate() {
            params.push((b + k as u32, *v));
        }
    }
    ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] }
}

/// The unit a synthetic bundle runs, and its saved state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    Spp3,
    Lsm2,
    Atsp,
}

impl Family {
    pub fn name(self) -> &'static str {
        match self {
            Family::Spp3 => "spp3",
            Family::Lsm2 => "lsm2",
            Family::Atsp => "atsp",
        }
    }
    pub fn state(self) -> ParamState {
        match self {
            Family::Spp3 => spp3_state(4),
            Family::Lsm2 => lsm2_state(),
            Family::Atsp => atsp_state(),
        }
    }
}

pub fn strip_path() -> String {
    format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip")
}

pub fn graph_path() -> String {
    format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg")
}

/// The four-channel synthetic bundle of `fam`, by bundle path, with the
/// protection unit's saved state `state`.
pub fn files(fam: Family, state: &ParamState) -> BTreeMap<String, Vec<u8>> {
    let mut f = spp3_files(4);
    f.insert(graph_path(), graph_text(fam.name()).into_bytes());
    f.insert(strip_path(), strip_with_state(fam.name(), state));
    f
}

/// A verified bundle held in memory, as `Bundle::from_dir` makes it from
/// these files and the installer's manifest for them.
pub fn bundle(files: BTreeMap<String, Vec<u8>>) -> Option<Bundle> {
    let manifest = Manifest::parse(manifest_json(&files, "25G83").as_bytes()).ok()?;
    let mut held = BTreeMap::new();
    for rec in manifest.records() {
        let data = files.get(&rec.path)?.clone();
        rec.verify(&data).ok()?;
        held.insert(rec.path.clone(), data);
    }
    Some(Bundle { manifest, files: held, source: PathBuf::from("fuzz"), verified: true, unlisted: Vec::new() })
}

/// A Mac's own bundle the `tuning` target may start from instead of the
/// synthetic fixtures, and the machine it was collected on.
pub struct Real {
    pub bundle: Bundle,
    pub machine: Machine,
}

/// The bundles named by `AURORA_FUZZ_BUNDLES` (directories or archives,
/// `:`-separated), loaded and verified once; none when it is unset. They
/// are the Mac's own (Apple) data: a corpus grown from them must stay with
/// them, out of every repository.
pub fn real_bundles() -> &'static [Real] {
    static REAL: std::sync::OnceLock<Vec<Real>> = std::sync::OnceLock::new();
    REAL.get_or_init(|| {
        let Ok(list) = std::env::var("AURORA_FUZZ_BUNDLES") else { return Vec::new() };
        list.split(':')
            .filter(|p| !p.is_empty())
            .map(|p| {
                let bundle = Bundle::open(std::path::Path::new(p)).unwrap_or_else(|e| panic!("{p}: {e}"));
                let board = bundle.manifest.board.clone().unwrap_or_else(|| "j413".into());
                let machine = match &bundle.manifest.unit_id {
                    Some(unit) => Machine::with_unit(&board, unit),
                    None => Machine::new(&board),
                }
                .unwrap_or_else(|e| panic!("{p}: {e}"));
                Real { bundle, machine }
            })
            .collect()
    })
}

/// A value the fuzzer chooses, weighted to the ones that break limits.
pub fn hostile_f64(u: &mut Unstructured) -> Result<f64> {
    Ok(match u.int_in_range(0..=15u8)? {
        0 => f64::NAN,
        1 => f64::INFINITY,
        2 => f64::NEG_INFINITY,
        3 => 0.0,
        4 => -0.0,
        5 => -1.0,
        6 => 1e-30,
        7 => 1e30,
        8 => f64::from(f32::MAX) * 2.0,
        9 => f64::MIN_POSITIVE,
        10 => -1e6,
        11 => f64::from(u.int_in_range(-200..=200i32)?),
        12 => f64::from(u.arbitrary::<u8>()?),
        _ => u.arbitrary::<f64>()?,
    })
}

/// Every numeric, boolean or string leaf of a plist value, by path.
fn leaves(v: &Value, path: &mut Vec<String>, out: &mut Vec<Vec<String>>) {
    match v {
        Value::Dict(d) => {
            for (k, x) in d.iter() {
                path.push(k.to_string());
                leaves(x, path, out);
                path.pop();
            }
        }
        Value::Array(a) => {
            for (i, x) in a.iter().enumerate() {
                path.push(format!("#{i}"));
                leaves(x, path, out);
                path.pop();
            }
        }
        _ => out.push(path.clone()),
    }
}

fn at_path<'a>(v: &'a mut Value, path: &[String]) -> Option<&'a mut Value> {
    let Some((first, rest)) = path.split_first() else { return Some(v) };
    let next = match v {
        Value::Dict(d) => d.get_mut(first)?,
        Value::Array(a) => a.get_mut(first.strip_prefix('#')?.parse::<usize>().ok()?)?,
        _ => return None,
    };
    at_path(next, rest)
}

/// Replace one leaf of a plist with a hostile value of its own kind (or,
/// now and then, of another kind).
pub fn mutate_plist(v: &mut Value, u: &mut Unstructured) -> Result<()> {
    let mut all = Vec::new();
    leaves(v, &mut Vec::new(), &mut all);
    if all.is_empty() {
        return Ok(());
    }
    let path = &all[u.choose_index(all.len())?];
    let Some(leaf) = at_path(v, path) else { return Ok(()) };
    let retype = u.ratio(1, 8)?;
    *leaf = match leaf {
        Value::Real(_) if !retype => Value::Real(hostile_f64(u)?),
        Value::Integer(i) if !retype => Value::Integer(match u.int_in_range(0..=5u8)? {
            0 => 0,
            1 => -1,
            2 => *i + 1,
            3 => i128::from(u32::MAX) + 1,
            4 => i128::from(u.arbitrary::<i64>()?),
            _ => i128::from(u.arbitrary::<u8>()?),
        }),
        Value::Bool(b) if !retype => Value::Bool(!*b),
        Value::Data(d) if !retype => {
            let mut d = std::mem::take(d);
            match ParamState::decode(&d, "fuzz") {
                Ok(mut st) => {
                    mutate_state(&mut st, u)?;
                    d = st.encode();
                }
                Err(_) if !d.is_empty() => {
                    let at = u.choose_index(d.len())?;
                    d[at] = u.arbitrary()?;
                }
                Err(_) => {}
            }
            Value::Data(d)
        }
        Value::String(_) if !retype => {
            let n = u.int_in_range(0..=12usize)?;
            Value::String(String::from_utf8_lossy(u.bytes(n)?).into_owned())
        }
        _ => match u.int_in_range(0..=3u8)? {
            0 => Value::Real(hostile_f64(u)?),
            1 => Value::Integer(i128::from(u.arbitrary::<i32>()?)),
            2 => Value::String(String::new()),
            _ => Value::Bool(u.arbitrary()?),
        },
    };
    Ok(())
}

/// Change, add or remove one saved parameter.
pub fn mutate_state(st: &mut ParamState, u: &mut Unstructured) -> Result<()> {
    if st.sections.is_empty() {
        return Ok(());
    }
    let s = u.choose_index(st.sections.len())?;
    let sec = &mut st.sections[s];
    match u.int_in_range(0..=9u8)? {
        0 if !sec.params.is_empty() => {
            let k = u.choose_index(sec.params.len())?;
            sec.params.remove(k);
        }
        1 => {
            let id = u.int_in_range(0..=40_000u32)?;
            if sec.get(id).is_none() {
                sec.params.push((id, hostile_f64(u)? as f32));
            }
        }
        _ if !sec.params.is_empty() => {
            let k = u.choose_index(sec.params.len())?;
            let old = sec.params[k].1;
            sec.params[k].1 = match u.int_in_range(0..=3u8)? {
                0 => old * hostile_f64(u)? as f32,
                1 => -old,
                _ => hostile_f64(u)? as f32,
            };
        }
        _ => {}
    }
    Ok(())
}

/// What a compiled blob must be: the header the compiler reports, a
/// protecting unit in the plugin (never its fail-safe), a pre-stage the
/// plugin runs, and everything the supervisor checks before it grants.
pub fn check_compiled(c: &compile::Compiled) {
    let blob = &c.blob;
    let (h, _) = sgv2_core::blob::Header::parse(blob).expect("a compiled blob has a header");
    assert_eq!((h.channels, h.sense_channels, h.sample_rate), (c.channels, c.sense_channels, c.sample_rate));
    assert_eq!((h.machine(), h.full_scale_mv), (Some(c.board.as_str()), c.full_scale_mv));
    assert!(c.full_scale_mv > 0, "no operating point");
    // the plugin's path
    let (unit, err, crc) =
        load_unit(Some(blob), f64::from(c.sample_rate), c.channels as usize, c.sense_channels as usize, Role::Protect);
    assert!(err.is_none(), "the plugin refuses a compiled blob: {err:?}");
    let kind_ok = match (&unit, c.unit) {
        (AnyUnit::Spp3(u), UnitKind::Spp3) => !u.is_failsafe(),
        (AnyUnit::Lsm2(_), UnitKind::Lsm2) => true,
        (AnyUnit::Atsp(u), UnitKind::Atsp) => !u.is_failsafe(),
        _ => false,
    };
    assert!(kind_ok, "a compiled {:?} blob loads as another unit or the fail-safe", c.unit);
    assert!(crc != 0, "a compiled blob without a deployment identity");
    assert!(
        sgv2_lv2::load_prestage(Some(blob), f64::from(c.sample_rate), c.channels as usize, &unit).is_some(),
        "the plugin refuses the compiled pre-stage"
    );
    // the supervisor's checks before it grants
    sgv2_core::blob::voicing(blob).expect("voicing rejected");
    sgv2_core::blob::prestage(blob, f64::from(h.sample_rate)).expect("pre-stage rejected");
    let per = sgv2_supervisor::machine::blob_full_scales(blob).expect("full scales rejected");
    assert!(per.iter().all(|&mv| mv > 0));
}

#[cfg(test)]
mod tests {
    use super::*;
    use aurora_speaker_tuning::tuning::{Options, Tuning};
    use aurora_speaker_tuning::Diagnostics;

    /// The unmutated fixtures compile and pass every check, so the fuzzer
    /// starts from blobs that run, not from refusals.
    #[test]
    fn every_fixture_compiles_to_a_blob_that_runs() {
        for fam in [Family::Spp3, Family::Lsm2, Family::Atsp] {
            let b = bundle(files(fam, &fam.state())).expect("the fixture bundle verifies");
            let mut d = Diagnostics::default();
            let t = Tuning::load(&b, &Options::default(), &mut d).unwrap();
            let m = Machine::with_unit("j413", &test_unit()).unwrap();
            let c = compile::compile(&t, &m, &mut d).unwrap_or_else(|e| panic!("{fam:?}: {e}"));
            check_compiled(&c);
        }
    }

    /// Writes synthetic seed corpora (from these fixtures only) under
    /// `$SEED_DIR/<target>/`: `SEED_DIR=... cargo test --lib -- --ignored`.
    #[test]
    #[ignore]
    fn write_seeds() {
        let Ok(dir) = std::env::var("SEED_DIR") else { return };
        let put = |t: &str, name: &str, data: &[u8]| {
            let d = std::path::Path::new(&dir).join(t);
            std::fs::create_dir_all(&d).unwrap();
            std::fs::write(d.join(name), data).unwrap();
        };
        for fam in [Family::Spp3, Family::Lsm2, Family::Atsp] {
            let f = files(fam, &fam.state());
            for (path, data) in &f {
                let name = format!("{}-{}", fam.name(), path.replace('/', "_"));
                if aurora_speaker_tuning::plist::sniff(data).is_some() {
                    put("plist", &name, data);
                    put("formats", &format!("4-{name}"), &[&[4u8][..], data].concat());
                }
                if path.ends_with(".dspg") {
                    put("dspg", &name, data);
                }
                if path.ends_with(".bin") {
                    put("formats", &format!("0-{name}"), &[&[0u8][..], data].concat());
                }
            }
            put("aupreset", &format!("{}-state", fam.name()), &fam.state().encode());
            let m = manifest_json(&f, "25G83");
            put("formats", &format!("1-{}-manifest", fam.name()), &[b"\x01".as_slice(), m.as_bytes()].concat());
            put(
                "formats",
                &format!("2-{}-collector", fam.name()),
                &[&[2u8][..], collector_json(&f).as_bytes()].concat(),
            );
        }
        let cfg = aurora_speaker_tuning::plist::xml::write(&device_config(FIXTURE_VOLUME_MIN_DB));
        put("formats", "5-config-xml", &[&[5u8][..], cfg.as_bytes()].concat());
        put("formats", "3-lattice", &[&[3u8][..], &lattice_bytes(&synthetic_lattice_ends())[..]].concat());
        put("formats", "6-compatible", b"\x06apple,j413\0apple,t8103\0");
        put("formats", "8-build", b"\x0825G83");
        let law = aurora_speaker_tuning::voicing::parity::LAW;
        let parity = format!(
            "\x0725G83\0k1\0{{\"format\":2,\"law\":\"{law}\",\"tolerance\":{{\"louder_db\":0.25,\"quieter_db\":1.0}},\
             \"releases\":[{{\"build\":\"25G83\",\"reference\":\"r\",\"sliders\":[0.5],\"entries\":[{{\"key\":\"k1\",\
             \"verdict\":\"pass\",\"sliders\":[{{\"slider\":0.5,\"verdict\":\"pass\"}}]}}]}}]}}"
        );
        put("formats", "7-parity", parity.as_bytes());
        // the pipeline: empty input is the unmutated bundle of each family
        for k in 0..3u8 {
            put("tuning", &format!("family-{k}"), &[k]);
        }
    }
}
