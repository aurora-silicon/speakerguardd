// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! `aurora-speaker-tuning`: inspect and validate this Mac's speaker tuning.
//!
//! ```text
//! aurora-speaker-tuning dump     [--json] [--speaker-id N] [SOURCE]
//! aurora-speaker-tuning validate [--json] [--speaker-id N] [SOURCE]
//! aurora-speaker-tuning compile  [--speaker-id N] [--board B [--unit-id U]] [--development] [--prestage identity|tuning] [SOURCE | --bundle PATH] -o BLOB
//! aurora-speaker-tuning identify [--json] [SOURCE]   (the bundle's identity, for provisioning)
//! aurora-speaker-tuning blob     BLOB [BLOB]     (decode; with two, diff field by field)
//! aurora-speaker-tuning units    [--json]
//! aurora-speaker-tuning plist    FILE            (any plist as XML)
//!
//! SOURCE: PATH (directory, .tar, .tar.gz)
//!       | --tunings-root DIR --aid N   (development: read macOS Tunings in place)
//!       | nothing                      (vendor firmware, then the ESP archive)
//! ```
//!
//! `show` and `verify` are accepted for `dump` and `validate`.  `validate`
//! exits 0 when there are no errors, 1 when there are, 2 on usage errors.
//!
//! The speaker ID is `--speaker-id`, else the manifest's, else the device
//! tree's `/chosen/asahi,speaker-id`.  `compile` binds the blob to a board:
//! `--board` (compiling for another Mac), else this Mac's device tree; the
//! device tree's speaker ID is read only without `--board`.  `compile`
//! refuses an unverified source (`--tunings-root`) unless `--development`.
//! The unit is this Mac's device tree's (`serial-number`), or with
//! `--board` the `--unit-id` given (the bundle's own, to compile offline
//! for the Mac it came from), else unknown: then per-unit calibration is
//! not used (open loop).  See `compile` for the binding rules.
//! `--prestage` picks what runs ahead of the unit: `identity` (the default,
//! pass-through: the voicing graph feeds the unit) or `tuning` (the Mac's
//! own filtering, derived from the tuning; rigs and the full macOS chain).
//!
//! `identify` loads and verifies the bundle (nothing is parsed beyond the
//! manifest) and prints where it was found and its digest
//! ([`Bundle::digest`]), its AID, macOS build and speaker ID, and the
//! reader's version: what a compiled blob is bound to.

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::compile::{self, Provenance};
use aurora_speaker_tuning::machine::{self, Machine};
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::{plist, units, Diagnostics, Severity};
use std::path::PathBuf;
use std::process::exit;

struct Args {
    cmd: String,
    json: bool,
    speaker_id: Option<i128>,
    tunings_root: Option<PathBuf>,
    aid: Option<u32>,
    path: Option<PathBuf>,
    path2: Option<PathBuf>,
    out: Option<PathBuf>,
    board: Option<String>,
    unit_id: Option<String>,
    development: bool,
    prestage: compile::PreStageMode,
}

fn usage() -> ! {
    eprintln!(
        "usage: aurora-speaker-tuning {{dump|validate}} [--json] [--speaker-id N] [PATH | --tunings-root DIR --aid N]\n       \
         aurora-speaker-tuning compile [--speaker-id N] [--board B [--unit-id U]] [--development] [--prestage identity|tuning] [--bundle PATH | --tunings-root DIR --aid N] -o BLOB\n       \
         aurora-speaker-tuning identify [--json] [PATH | --tunings-root DIR --aid N]\n       \
         aurora-speaker-tuning blob BLOB [BLOB]\n       \
         aurora-speaker-tuning units [--json]\n       aurora-speaker-tuning plist FILE"
    );
    exit(2)
}

fn parse_args() -> Args {
    let mut it = std::env::args().skip(1);
    let cmd = match it.next().as_deref() {
        Some("dump" | "show") => "dump",
        Some("validate" | "verify") => "validate",
        Some("units") => "units",
        Some("plist") => "plist",
        Some("compile") => "compile",
        Some("blob") => "blob",
        Some("identify") => "identify",
        _ => usage(),
    }
    .to_string();
    let mut a = Args {
        cmd,
        json: false,
        speaker_id: None,
        tunings_root: None,
        aid: None,
        path: None,
        path2: None,
        out: None,
        board: None,
        unit_id: None,
        development: false,
        prestage: compile::PreStageMode::default(),
    };
    while let Some(x) = it.next() {
        match x.as_str() {
            "--json" => a.json = true,
            "--speaker-id" => a.speaker_id = Some(it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| usage())),
            "--tunings-root" => a.tunings_root = Some(it.next().map(PathBuf::from).unwrap_or_else(|| usage())),
            "--aid" => {
                a.aid =
                    Some(it.next().and_then(|v| v.trim_start_matches("AID").parse().ok()).unwrap_or_else(|| usage()))
            }
            "--bundle" => a.path = Some(it.next().map(PathBuf::from).unwrap_or_else(|| usage())),
            "--board" => a.board = Some(it.next().unwrap_or_else(|| usage())),
            "--unit-id" => a.unit_id = Some(it.next().unwrap_or_else(|| usage())),
            "--development" => a.development = true,
            "--prestage" => {
                a.prestage = match it.next().as_deref() {
                    Some("identity") => compile::PreStageMode::Identity,
                    Some("tuning") => compile::PreStageMode::Tuning,
                    _ => usage(),
                }
            }
            "-o" | "--output" => a.out = Some(it.next().map(PathBuf::from).unwrap_or_else(|| usage())),
            p if !p.starts_with('-') && a.path.is_none() => a.path = Some(PathBuf::from(p)),
            p if !p.starts_with('-') && a.path2.is_none() => a.path2 = Some(PathBuf::from(p)),
            _ => usage(),
        }
    }
    // A unit ID names the Mac an offline compile is for; it never stands
    // in for the running Mac's device tree.
    if a.unit_id.is_some() && (a.board.is_none() || a.cmd != "compile") {
        usage();
    }
    a
}

/// Writes `data` to a temporary file beside `out`, then renames it into
/// place, so a reader never sees a partial blob.
fn write_atomic(out: &std::path::Path, data: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    let name = out.file_name().ok_or_else(|| std::io::Error::other("no file name"))?;
    let tmp = out.with_file_name(format!(".{}.tmp{}", name.to_string_lossy(), std::process::id()));
    let r = (|| {
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(data)?;
        f.sync_all()?;
        std::fs::rename(&tmp, out)
    })();
    if r.is_err() {
        let _ = std::fs::remove_file(&tmp);
    }
    r
}

fn print_diags(d: &Diagnostics) {
    for x in &d.items {
        eprintln!("{x}");
    }
    eprintln!(
        "{} error(s), {} warning(s), {} note(s)",
        d.count(Severity::Error),
        d.count(Severity::Warning),
        d.count(Severity::Info)
    );
}

fn fmt_f(v: f64) -> String {
    format!("{v}")
}

fn human(t: &Tuning) {
    let p = &t.protection;
    println!("{} (acoustic-id {}), macOS {}", t.aid, t.acoustic_id, t.macos_build.as_deref().unwrap_or("unknown"));
    println!(
        "selection: {} | graph {} | strip {} | propstrip {}",
        t.selection.configuration.as_deref().unwrap_or("(file-name convention)"),
        t.selection.graph,
        t.selection.strip,
        t.selection.propstrip.as_deref().unwrap_or("-")
    );
    if !t.selection.substitutions.is_empty() {
        println!("substitutions: {:?}", t.selection.substitutions);
    }
    println!("\nprocessing chain ({} boxes):", t.chain.len());
    for (i, s) in t.chain.iter().enumerate() {
        let mark = if i == p.position { "  <== protection" } else { "" };
        let params = s.state.as_ref().map_or(String::new(), |st| {
            st.sections
                .iter()
                .map(|x| format!("({},{})x{}", x.scope, x.element, x.params.len()))
                .collect::<Vec<_>>()
                .join(" ")
        });
        println!(
            "  {i:2} {:<20} {:<16} ch={:<4} {}{}{}",
            s.name,
            s.kind,
            s.channels.map_or("?".into(), |c| c.to_string()),
            s.preset_name.as_deref().map(|n| format!("preset={n} ")).unwrap_or_default(),
            params,
            mark
        );
    }
    println!("\nprotection: {:?} box {} ({}), block {:?}", p.kind, p.box_name, p.unit, p.block_size);
    println!("input lanes (unit channel <- origin, join slots):");
    for (k, l) in p.lanes.iter().enumerate() {
        let hops: Vec<String> = l.hops.iter().map(|h| format!("{}[{}]->{}", h.join, h.input, h.channel)).collect();
        let name = p
            .spp3
            .as_ref()
            .and_then(|s| s.channels.get(k))
            .and_then(|c| c.speaker_name.clone())
            .map(|n| format!("  sidecar label {n:?}"))
            .unwrap_or_default();
        println!("  {k}: {}:{} ch{} via {}{}", l.origin.node, l.origin.port, l.origin.channel, hops.join(" "), name);
    }
    if let Some(tm) = &p.telemetry {
        println!(
            "telemetry: port {:?}, {} ch{}, path {}",
            tm.port,
            tm.channels.map_or("?".into(), |c| c.to_string()),
            if tm.interleaved { " interleaved" } else { "" },
            tm.path.join(" -> ")
        );
    }
    for l in &p.params {
        println!("graph param {} -> unit param {:?} {:?}", l.name, l.id, l.rest);
    }
    for l in &p.properties {
        println!("graph property {} -> unit property {:?} {:?}", l.name, l.id, l.rest);
    }
    for sp in &p.strip_properties {
        println!("strip property {} = {:?}", sp.number, sp.value);
    }
    for f in &p.property_files {
        println!("property {} file {}: keys {:?}", f.number, f.path, f.keys);
    }
    if let Some(s) = &p.spp3 {
        let g = &s.globals;
        println!("\nspp3 sidecar v{}: {} channels, groups {:?}", s.version, s.num_channels, s.groups());
        println!(
            "  ambient {} degC, attack {} s, release {} s, look-ahead {} ms, safe Tlim {}/{}/{} s, displacement {}, CPMS {}/{}",
            g.thermal_ambient_temp,
            g.thermal_gain_attack_time_s,
            g.thermal_gain_release_time_s,
            g.thermal_lookahead_delay_ms,
            g.thermal_safe_tlim,
            g.thermal_safe_tlim_offset,
            g.thermal_safe_tlim_time_min,
            g.enable_displacement_control,
            g.enable_cpms_control,
            g.enable_cpms_closed_loop_control
        );
        for c in &s.channels {
            let t = &c.ol_thermal;
            println!(
                "  ch{} {:<6} grp {:?}: Rvc/Rmg {}/{} K/W, tau {}/{} s, limit {}+{} degC, Reb {} Rshunt {} ohm, ag {} V/FS, FF {}{}",
                c.index,
                c.speaker_name.as_deref().unwrap_or("-"),
                c.speaker_group,
                fmt_f(t.t_sett_vc),
                fmt_f(t.t_sett_mg),
                fmt_f(t.tau_tvc),
                fmt_f(t.tau_tmg),
                fmt_f(t.temperature_limit),
                fmt_f(t.hard_temp_limit_headroom),
                fmt_f(t.reb_ref),
                fmt_f(t.rshunt),
                fmt_f(c.ol_disp.ag),
                fmt_f(t.thermal_ff_speedup_factor),
                c.cl.as_ref().map_or(" | no CL".to_string(), |l| format!(
                    " | CL method {} V/I {}/{} Reb {}..{} Tcal {}..{} alpha {}/{} pilot {} Hz",
                    l.thermal_measurement_method,
                    l.vmon_scale,
                    l.imon_scale,
                    l.reb_range_min,
                    l.reb_range_max,
                    l.cal_temp_range_min,
                    l.cal_temp_range_max,
                    l.a_t_20c,
                    l.a_t_35c,
                    l.pilot_freq
                ))
            );
        }
    } else {
        println!("\nsaved state of {} (raw IDs; TODO(SPEC) names):", p.box_name);
        for s in &p.state.sections {
            let vals: Vec<String> = s.params.iter().map(|(i, v)| format!("{i}={v}")).collect();
            println!("  scope {} element {} ({} params): {}", s.scope, s.element, s.params.len(), vals.join(" "));
        }
    }
    if let Some(sp) = &t.spts {
        println!(
            "\nSpTS v{} ({:?} header): {} records (checksum 0x{:04x} {}); record i calibrates unit channel i",
            sp.version,
            sp.form,
            sp.records.len(),
            sp.checksum_stored,
            if sp.checksum_valid { "valid" } else { "INVALID" }
        );
        for r in &sp.records {
            println!("  {} R_cal {:.4} ohm  T_cal {:.3} degC", r.data_source, r.r_cal_ohm, r.t_cal_c);
        }
    }
}

/// Decodes a blob with the engine's own decoder, one field per line.
fn decode_blob(p: &std::path::Path) -> String {
    use sgv2_core::blob::Header;
    let bytes = std::fs::read(p).unwrap_or_else(|e| {
        eprintln!("{}: {e}", p.display());
        exit(1)
    });
    let (h, _) = Header::parse(&bytes).unwrap_or_else(|e| {
        eprintln!("{}: {e:?}", p.display());
        exit(1)
    });
    let unit = h.unit;
    let body: Result<String, String> = if unit == sgv2_core::UnitKind::Spp3 as u32 {
        sgv2_core::spp3::Params::from_blob(&bytes)
            .map(|x| trim(format!("{x:#?}"), x.channels))
            .map_err(|e| format!("{e:?}"))
    } else if unit == sgv2_core::UnitKind::Lsm2 as u32 {
        sgv2_core::lsm2::Params::from_blob(&bytes)
            .map(|x| trim(format!("{x:#?}"), x.channels))
            .map_err(|e| format!("{e:?}"))
    } else {
        sgv2_core::atsp::Params::from_blob(&bytes).map(|x| format!("{x:#?}")).map_err(|e| format!("{e:?}"))
    };
    match body {
        Ok(b) => format!("header: {h:?}\nboard: {:?}, full scale {} mV\n{b}", h.machine(), h.full_scale_mv),
        Err(e) => {
            eprintln!("{}: the engine rejects it: {e}", p.display());
            exit(1)
        }
    }
}

/// Drops the unused channel slots after the first `n` from a Debug dump.
fn trim(s: String, n: u32) -> String {
    let mut out = Vec::new();
    let mut depth_ch = None::<usize>;
    let mut idx = 0u32;
    for line in s.lines() {
        if line.trim_start().starts_with("ch: [") {
            depth_ch = Some(line.len() - line.trim_start().len());
            out.push(line.to_string());
            continue;
        }
        if let Some(d) = depth_ch {
            let ind = line.len() - line.trim_start().len();
            if ind == d && line.trim_start().starts_with(']') {
                depth_ch = None;
                out.push(line.to_string());
                continue;
            }
            if ind == d + 4 && line.trim_start().starts_with("ChannelParams") {
                idx += 1;
            }
            if idx > n {
                continue;
            }
        }
        out.push(line.to_string());
    }
    out.join("\n")
}

/// `identify`: the bundle's source and identity, or why there is none.
fn identify(bundle: aurora_speaker_tuning::Result<Bundle>, json: bool) {
    let b = match bundle {
        Ok(b) => b,
        Err(e) => {
            if json {
                println!("{}", serde_json::json!({ "error": e.to_string() }));
            } else {
                eprintln!("{e}");
            }
            exit(1)
        }
    };
    let m = &b.manifest;
    let v = serde_json::json!({
        "source": b.source.display().to_string(),
        "verified": b.verified,
        "kind": m.source,
        "acoustic_id": m.acoustic_id,
        "aid": m.aid,
        "macos_build": m.macos_build,
        "speaker_id": m.speaker_id,
        "board": m.board,
        "unit_bound": m.unit_id.is_some(),
        "files": m.records().count().max(b.files.len()),
        "digest": b.digest(),
        "reader": env!("CARGO_PKG_VERSION"),
    });
    if json {
        println!("{}", serde_json::to_string_pretty(&v).unwrap());
    } else {
        for (k, x) in v.as_object().unwrap() {
            println!("{k}={}", x.as_str().map_or_else(|| x.to_string(), str::to_owned));
        }
    }
}

fn main() {
    let a = parse_args();
    match a.cmd.as_str() {
        "units" => {
            let t = units::table();
            if a.json {
                println!("{}", serde_json::to_string_pretty(&t).unwrap());
            } else {
                for r in t {
                    println!("{:<36} {:<28} {:<16} {}", r.source, r.field, r.unit, r.meaning);
                }
            }
            return;
        }
        "plist" => {
            let p = a.path.unwrap_or_else(|| usage());
            let bytes = std::fs::read(&p).unwrap_or_else(|e| {
                eprintln!("{}: {e}", p.display());
                exit(1)
            });
            match plist::parse(&bytes) {
                Ok((v, _)) => print!("{}", plist::xml::write(&v)),
                Err(e) => {
                    eprintln!("{}: {e}", p.display());
                    exit(1)
                }
            }
            return;
        }
        "blob" => {
            let p = a.path.clone().unwrap_or_else(|| usage());
            let one = decode_blob(&p);
            match &a.path2 {
                None => println!("{one}"),
                Some(q) => {
                    let two = decode_blob(q);
                    let (l1, l2): (Vec<&str>, Vec<&str>) = (one.lines().collect(), two.lines().collect());
                    let mut diffs = 0;
                    for k in 0..l1.len().max(l2.len()) {
                        let (x, y) = (l1.get(k).copied().unwrap_or(""), l2.get(k).copied().unwrap_or(""));
                        if x != y {
                            diffs += 1;
                            println!("- {}\n+ {}", x.trim(), y.trim());
                        }
                    }
                    println!("{} field line(s) differ", diffs);
                    if diffs > 0 {
                        exit(1);
                    }
                }
            }
            return;
        }
        _ => {}
    }
    let bundle = match (&a.tunings_root, a.aid, &a.path) {
        (Some(root), Some(aid), None) => Bundle::from_tunings_root(root, aid),
        (None, None, Some(p)) => Bundle::open(p),
        (None, None, None) => Bundle::discover(),
        _ => usage(),
    };
    if a.cmd == "identify" {
        identify(bundle, a.json);
        return;
    }
    let mut diags = Diagnostics::default();
    let dt = std::path::Path::new(machine::DEVICE_TREE);
    let live = a.board.is_none() && dt.is_dir();
    let device_tree_speaker_id = if live {
        machine::speaker_id(dt).unwrap_or_else(|e| {
            eprintln!("{e}");
            exit(1)
        })
    } else {
        None
    };
    let opts = Options { speaker_id: a.speaker_id, device_tree_speaker_id };
    let result = bundle.and_then(|b| Tuning::load(&b, &opts, &mut diags));
    match result {
        Ok(t) => {
            if a.cmd == "compile" {
                let out = a.out.clone().unwrap_or_else(|| usage());
                let m = match (&a.board, &a.unit_id) {
                    (Some(b), Some(u)) => Machine::with_unit(b, u),
                    (Some(b), None) => Machine::new(b),
                    (None, _) => Machine::from_device_tree(dt),
                };
                let provenance = if a.development { Provenance::AllowUnverified } else { Provenance::Verified };
                match m.and_then(|m| compile::compile_opts(&t, &m, provenance, a.prestage, &mut diags)) {
                    Ok(c) => {
                        if let Err(e) = write_atomic(&out, &c.blob) {
                            eprintln!("{}: {e}", out.display());
                            exit(1);
                        }
                        eprintln!(
                            "wrote {} ({} bytes): {:?}, {} channels, {} sense, {} Hz, board {}, full scale {} mV, {}",
                            out.display(),
                            c.blob.len(),
                            c.unit,
                            c.channels,
                            c.sense_channels,
                            c.sample_rate,
                            c.board,
                            c.full_scale_mv,
                            if c.calibration { "unit calibration in use" } else { "no unit calibration" }
                        );
                    }
                    Err(e) => diags.from_error("compile", &e),
                }
            } else if a.cmd == "dump" {
                if a.json {
                    println!(
                        "{}",
                        serde_json::to_string_pretty(&serde_json::json!({"tuning": t, "diagnostics": diags})).unwrap()
                    );
                } else {
                    human(&t);
                }
            } else if a.json {
                println!("{}", serde_json::to_string_pretty(&diags).unwrap());
            }
            if !a.json || a.cmd == "validate" {
                print_diags(&diags);
            }
            if diags.has_errors() {
                exit(1);
            }
        }
        Err(e) => {
            diags.from_error("load", &e);
            if a.json {
                println!("{}", serde_json::to_string_pretty(&diags).unwrap());
            }
            print_diags(&diags);
            exit(1);
        }
    }
}
