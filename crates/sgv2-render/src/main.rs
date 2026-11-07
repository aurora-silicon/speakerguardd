//! `sgv2-render`: the Linux side of the differential rig.
//!
//! Runs one `sgv2-core` unit offline over a program signal (and, for V/I
//! units, a recorded sense signal) in host slices of the case's sizes. It
//! writes one JSON line per slice in the schema of `tools/diffrig/schema.json`
//! and, optionally, the rendered audio.
//!
//! ```text
//! sgv2-render --unit spp3|lsm2|atsp [--blob PARAMS.blob] --case NAME
//!             --in PROGRAM.f32 --ch N [--sense SENSE.f32 --sense-ch M]
//!             [--slices 512 | --slices 1,7,31,...] [--out TRACE.jsonl]
//!             [--audio-out OUT.f32] [--cfprop ID=FILE ...] [--events F:reset,...]
//! ```
//!
//! `--file-ch N` reads a program file of N channels, stream channel c taking
//! file channel c mod N (the rig host's mapping). `--limit-frames N` renders
//! only the first N frames.
//!
//! `--preset-param ID=VALUE` sets an atsp AU parameter in the decoded
//! parameters before initialisation; `--param ID=VALUE` sets one at run time
//! after initialisation, `--param-at FRAME:ID=VALUE` at the slice starting
//! on that frame (the unit's `set_parameter`).
//!
//! `--cfprop ID=FILE` hands the unit the same run-time and case properties
//! (by ID, from the same fixture files) as the macOS reference got. Every
//! file must be readable, and each trace line lists the IDs in `cfprops`, so
//! the rig can check that the inputs reached this side. Consumed: 64008 on
//! lsm2, the CPMS power budgets the platform sets right after
//! initialisation (a JSON object of `cpms_ts_i`, `cpms_ts_100`,
//! `cpms_ts_1000`, `cpms_ts_thermal` in mW). Others are listed only; the
//! spp3 calibration (64009) reaches the unit through its blob.
//!
//! `--events` replays the case's host events: `FRAME:reset` calls the unit's
//! `host_reset` (AudioUnitReset) before that frame; `sleep=`/`tsjump=` are
//! wall-clock events with no effect offline.
//!
//! Signals are raw interleaved little-endian float32, the rig's format. A
//! slice list is used cyclically. Without `--blob`, or with a blob that does
//! not parse, the unit is the fail-safe, exactly as in the graph.

use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::process::ExitCode;

use sgv2_core::blob::Header;
use sgv2_core::{
    atsp, lsm2, spp3, ChannelState, FailSafe, HandOff, Health, Io, ProtectionUnit, Telemetry, UnitKind, MAX_CHANNELS,
};

struct Args {
    unit: UnitKind,
    blob: Option<String>,
    case: String,
    input: String,
    channels: usize,
    sense: Option<String>,
    sense_channels: usize,
    slices: Vec<usize>,
    out: Option<String>,
    audio_out: Option<String>,
    cfprops: Vec<(u32, String)>,
    file_channels: usize,
    limit_frames: usize,
    preset_params: Vec<(u32, f32)>,
    /// atsp: the stream rate the reference ran (`--rate`), 0 for the blob's.
    rate: u32,
    params_at: Vec<(usize, u32, f32)>,
    /// Host events by frame (`--events`), in order.
    events: Vec<(usize, Event)>,
}

fn usage() -> ExitCode {
    eprintln!(
        "usage: sgv2-render --unit spp3|lsm2|atsp [--blob P] --case NAME --in X.f32 --ch N \
         [--sense S.f32 --sense-ch M] [--slices 512|1,7,31] [--out T.jsonl] [--audio-out O.f32] \
         [--cfprop ID=FILE ...] [--file-ch N] [--limit-frames N] [--preset-param ID=V] \
         [--param ID=V] [--param-at FRAME:ID=V]"
    );
    ExitCode::from(2)
}

fn parse_args() -> Option<Args> {
    let mut a = Args {
        unit: UnitKind::Spp3,
        blob: None,
        case: String::from("case"),
        input: String::new(),
        channels: 0,
        sense: None,
        sense_channels: 0,
        slices: vec![512],
        out: None,
        audio_out: None,
        cfprops: Vec::new(),
        file_channels: 0,
        limit_frames: 0,
        preset_params: Vec::new(),
        rate: 0,
        params_at: Vec::new(),
        events: Vec::new(),
    };
    let id_val = |v: &str| -> Option<(u32, f32)> {
        let (i, x) = v.split_once('=')?;
        Some((i.parse().ok()?, x.parse().ok()?))
    };
    let mut it = std::env::args().skip(1);
    while let Some(k) = it.next() {
        let v = it.next()?;
        match k.as_str() {
            "--unit" => a.unit = UnitKind::from_fourcc(v.as_bytes().try_into().ok()?)?,
            "--blob" => a.blob = Some(v),
            "--case" => a.case = v,
            "--in" => a.input = v,
            "--ch" => a.channels = v.parse().ok()?,
            "--sense" => a.sense = Some(v),
            "--sense-ch" => a.sense_channels = v.parse().ok()?,
            "--slices" => {
                a.slices = v.split(',').map(|s| s.parse().ok()).collect::<Option<Vec<usize>>>()?;
                if a.slices.is_empty() || a.slices.contains(&0) {
                    return None;
                }
            }
            "--out" => a.out = Some(v),
            "--audio-out" => a.audio_out = Some(v),
            "--file-ch" => a.file_channels = v.parse().ok()?,
            "--limit-frames" => a.limit_frames = v.parse().ok()?,
            "--preset-param" => a.preset_params.push(id_val(&v)?),
            "--rate" => a.rate = v.parse().ok()?,
            "--param" => a.params_at.push((0, id_val(&v)?.0, id_val(&v)?.1)),
            "--param-at" => {
                let (f, rest) = v.split_once(':')?;
                let (i, x) = id_val(rest)?;
                a.params_at.push((f.parse().ok()?, i, x));
            }
            "--events" => {
                for e in v.split(',').filter(|e| !e.is_empty()) {
                    let (f, what) = e.split_once(':')?;
                    a.events.push((f.parse().ok()?, Event::parse(what)?));
                }
            }
            "--cfprop" => {
                let (id, path) = v.split_once('=')?;
                if path.is_empty() {
                    return None;
                }
                a.cfprops.push((id.parse().ok()?, path.to_string()));
            }
            _ => return None,
        }
    }
    if a.input.is_empty() || a.channels == 0 || a.channels > MAX_CHANNELS {
        return None;
    }
    if !a.case.bytes().all(|b| b.is_ascii_alphanumeric() || b"-_.".contains(&b)) {
        return None;
    }
    Some(a)
}

/// Read a raw interleaved float32 file into one vector per channel.
fn read_f32(path: &str, channels: usize) -> io::Result<Vec<Vec<f32>>> {
    let mut bytes = Vec::new();
    File::open(path)?.read_to_end(&mut bytes)?;
    Ok(deinterleave(&bytes, channels))
}

/// Stream channel c takes file channel c mod `file` (the rig host's map).
fn map_channels(file: Vec<Vec<f32>>, channels: usize) -> Vec<Vec<f32>> {
    (0..channels).map(|c| file[c % file.len()].clone()).collect()
}

fn deinterleave(bytes: &[u8], channels: usize) -> Vec<Vec<f32>> {
    let frames = bytes.len() / 4 / channels;
    let mut out = vec![Vec::with_capacity(frames); channels];
    for (i, w) in bytes.chunks_exact(4).take(frames * channels).enumerate() {
        out[i % channels].push(f32::from_le_bytes([w[0], w[1], w[2], w[3]]));
    }
    out
}

/// The lsm2 runtime power budgets of a CPMS dictionary (property 64008):
/// keys in mW, mapped in order to the 1 ms, 100 ms, 1 s and 60 s budgets; a
/// key the dictionary lacks is 50 W.
fn parse_cpms(text: &str) -> Option<[f32; 4]> {
    const KEYS: [&str; 4] = ["cpms_ts_i", "cpms_ts_100", "cpms_ts_1000", "cpms_ts_thermal"];
    let body = text.trim().strip_prefix('{')?.strip_suffix('}')?;
    let mut w = [50.0f32; 4];
    for item in body.split(',').filter(|x| !x.trim().is_empty()) {
        let (k, v) = item.split_once(':')?;
        let k = k.trim().trim_matches('"');
        let j = KEYS.iter().position(|&x| x == k)?;
        let mw: f64 = v.trim().parse().ok()?;
        w[j] = (mw * 1e-3) as f32;
    }
    Some(w)
}

/// Build the unit. A missing or unusable blob gives the fail-safe, as the
/// plugin does.
fn make_unit(
    kind: UnitKind,
    blob: Option<&[u8]>,
    cpms: Option<[f32; 4]>,
    preset: &[(u32, f32)],
    rate: u32,
) -> (Box<dyn ProtectionUnit>, String) {
    let Some(bytes) = blob else {
        return (Box::new(FailSafe::new(kind, Health::NO_PARAMS)), String::from("none"));
    };
    match Header::parse(bytes) {
        Ok((h, _)) if h.unit != kind as u32 => {
            return (Box::new(FailSafe::new(kind, Health::NO_PARAMS)), String::from("blob-unit-mismatch"))
        }
        Err(e) => return (Box::new(FailSafe::new(kind, Health::NO_PARAMS)), format!("blob-{e:?}")),
        Ok(_) => {}
    }
    let unit: Result<Box<dyn ProtectionUnit>, String> = match kind {
        UnitKind::Spp3 => {
            spp3::Params::from_blob(bytes).map(|p| Box::new(spp3::Unit::new(p)) as _).map_err(|e| format!("{e:?}"))
        }
        UnitKind::Lsm2 => lsm2::Unit::from_blob(bytes)
            .map(|mut unit| {
                if let Some(w) = cpms {
                    unit.set_runtime_budgets(w);
                }
                Box::new(unit) as _
            })
            .map_err(|e| format!("{e:?}")),
        UnitKind::Atsp => atsp::Params::from_blob(bytes).map_err(|e| format!("{e:?}")).and_then(|mut p| {
            if rate != 0 {
                p.sample_rate = rate;
            }
            for &(id, v) in preset {
                if !p.set(id, v) {
                    return Err(format!("preset parameter {id}"));
                }
            }
            p.validate().map_err(|e| format!("{e:?}"))?;
            Ok(Box::new(atsp::Unit::new(p)) as _)
        }),
    };
    match unit {
        Ok(u) => (u, String::from(if cpms.is_some() { "blob+64008" } else { "blob" })),
        Err(e) => (Box::new(FailSafe::new(kind, Health::NO_PARAMS)), format!("blob-{}", e.replace(['"', '\\'], ""))),
    }
}

fn num(x: f32) -> String {
    if x.is_finite() {
        format!("{x:.9e}")
    } else {
        String::from("null")
    }
}

fn arr(xs: &[f32]) -> String {
    let v: Vec<String> = xs.iter().map(|&x| num(x)).collect();
    format!("[{}]", v.join(","))
}

fn channel_json(c: &ChannelState, audio_gain: Option<f32>) -> String {
    format!(
        "{{\"gain\":{},\"audio_gain\":{},\"modeled_power_w\":{},\"measured_power_w\":{},\
         \"coil_temp_c\":{},\"magnet_temp_c\":{},\"control_temp_c\":{},\"ol_temp_c\":{},\"cl_temp_c\":{},\
         \"tlim_adj_c\":{},\"displacement_m\":{},\"velocity_m_s\":{},\"velocity_limit_m_s\":{},\"mask_db\":{},\
         \"axis_gain\":{},\"band_gain\":{}}}",
        num(c.gain),
        audio_gain.map_or(String::from("null"), num),
        num(c.modeled_power_w),
        num(c.measured_power_w),
        num(c.coil_temp_c),
        num(c.magnet_temp_c),
        num(c.control_temp_c),
        num(c.ol_temp_c),
        num(c.cl_temp_c),
        num(c.tlim_adj_c),
        num(c.displacement_m),
        num(c.velocity_m_s),
        num(c.velocity_limit_m_s),
        num(c.mask_db),
        arr(&c.axis_gain),
        arr(&c.band_gain)
    )
}

fn global_json(g: &sgv2_core::GlobalState) -> String {
    format!(
        "{{\"modeled_power_w\":{},\"measured_power_w\":{},\"thermal_gain_db\":{},\
         \"power_hard_limit_w\":{},\"power_gain\":{},\"modeled_amp_power_w\":{},\
         \"amp_input_power_per_speaker_w\":{}}}",
        num(g.modeled_power_w),
        num(g.measured_power_w),
        num(g.thermal_gain_db),
        arr(&g.power_hard_limit_w),
        num(g.power_gain),
        num(g.modeled_amp_power_w),
        num(g.amp_input_power_w)
    )
}

/// Broadband gain of one slice measured from the audio, aligned by the
/// unit's latency: sqrt(energy out / energy in), or `None` below -120 dBFS.
fn audio_gain(input: &[f32], start: usize, latency: usize, out: &[f32]) -> Option<f32> {
    let (mut ei, mut eo) = (0.0f64, 0.0f64);
    for (k, o) in out.iter().enumerate() {
        let i = (start + k).checked_sub(latency).map_or(0.0, |j| input.get(j).copied().unwrap_or(0.0));
        ei += (i as f64) * (i as f64);
        eo += (*o as f64) * (*o as f64);
    }
    if ei < out.len() as f64 * 1e-12 {
        None
    } else {
        Some((eo / ei).sqrt() as f32)
    }
}

fn fourcc(u: UnitKind) -> String {
    String::from_utf8_lossy(&(u as u32).to_be_bytes()).into_owned()
}

fn open_out(p: &Option<String>) -> io::Result<Option<BufWriter<File>>> {
    p.as_ref().map(|p| File::create(p).map(BufWriter::new)).transpose()
}

/// A host event of the rig (`rig_host --event FRAME:WHAT`).
#[derive(Clone, Copy, Debug, PartialEq)]
enum Event {
    /// `AudioUnitReset`: the unit's `host_reset`.
    Reset,
    /// Wall-clock idle before the next call, s. No thermal time passes,
    /// not even across a rebuild (the reference ignores it too).
    Sleep,
    /// A render-timestamp jump, or a ClassInfo get/set: no effect.
    None,
    /// `uninit`/`reinit`/`newinst`/`newinst_ci` with an optional 64007
    /// value: the kernel is rebuilt with that start state (the strip's when
    /// absent); under 0 it restores the hand-off record of the last update.
    /// `in_place` is an Uninitialize + Initialize of the same AU instance.
    Rebuild { state: Option<u32>, in_place: bool },
}

impl Event {
    fn parse(what: &str) -> Option<Event> {
        let (k, v) = what.split_once('=').map_or((what, None), |(k, v)| (k, Some(v)));
        Some(match k {
            "reset" => Event::Reset,
            "sleep" => {
                v?.parse::<f64>().ok()?;
                Event::Sleep
            }
            "tsjump" | "classinfo" => Event::None,
            "uninit" | "reinit" | "newinst" | "newinst_ci" => Event::Rebuild {
                state: v.map(|v| v.parse()).transpose().ok()?,
                in_place: matches!(k, "uninit" | "reinit"),
            },
            _ => return None,
        })
    }
}

/// Rebuilds the unit for an [`Event::Rebuild`]; `None` when the unit kind
/// has no rebuild model.
type Rebuild<'a> = &'a dyn Fn(Option<u32>, bool) -> Option<Box<dyn ProtectionUnit>>;

/// Render the whole program, writing trace lines and audio as it goes.
#[allow(clippy::too_many_arguments)]
fn render(
    unit: &mut Box<dyn ProtectionUnit>,
    params: &str,
    kind: UnitKind,
    case: &str,
    program: &[Vec<f32>],
    sense: &[Vec<f32>],
    slices: &[usize],
    params_at: &[(usize, u32, f32)],
    cfprops: &[u32],
    events: &[(usize, Event)],
    rebuild: Rebuild<'_>,
    trace: &mut dyn Write,
    mut audio: Option<&mut dyn Write>,
) -> io::Result<()> {
    let frames = program.first().map_or(0, Vec::len);
    let channels = program.len();
    let latency = unit.latency_frames();
    let max_slice = slices.iter().copied().max().unwrap_or(512);
    let mut outbuf = vec![vec![0.0f32; max_slice]; channels];
    let zeros = vec![0.0f32; max_slice];
    let mut t = Telemetry::new();
    let unit_name = fourcc(kind);
    let cfprop_ids: Vec<String> = cfprops.iter().map(u32::to_string).collect();
    let cfprop_ids = cfprop_ids.join(",");
    let (mut pos, mut k) = (0usize, 0usize);
    let mut next_event = 0usize;
    while pos < frames {
        // rig_host applies an event before the first call that starts at or
        // after its frame; it never splits a call
        let due = events[next_event..].iter().take_while(|(f, _)| *f <= pos).count();
        for (_, e) in &events[next_event..next_event + due] {
            match *e {
                Event::Reset => unit.host_reset(),
                Event::Sleep => {}
                Event::None => {}
                Event::Rebuild { state: p, in_place } => {
                    let mut rec = HandOff::EMPTY;
                    let saved = unit.save_state(&mut rec);
                    let new = rebuild(p, in_place).ok_or_else(|| io::Error::other("no rebuild model for this unit"))?;
                    *unit = new;
                    // only 64007 = 0 restores the record, and with no ageing:
                    // the rig's newinst=0 after 2 s of idle continues at the
                    // saved temperature (rig law 6)
                    if p == Some(0) && saved {
                        unit.restore_state(&rec, 0.0);
                    }
                }
            }
        }
        next_event += due;
        for &(f, id, v) in params_at {
            if f == pos && !unit.set_parameter(id, v) {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("parameter {id} is not run-time settable"),
                ));
            }
        }
        let n = slices[k % slices.len()].min(frames - pos);
        k += 1;
        let ins: Vec<&[f32]> = program.iter().map(|c| &c[pos..pos + n]).collect();
        // Sense shorter than the program reads as silence (dead sense).
        let sense_slices: Vec<&[f32]> =
            sense.iter().map(|c| if c.len() >= pos + n { &c[pos..pos + n] } else { &zeros[..n] }).collect();
        let health = {
            let mut outs: Vec<&mut [f32]> = outbuf.iter_mut().map(|c| &mut c[..n]).collect();
            unit.process(Io { frames: n, input: &ins, sense: &sense_slices, output: &mut outs })
        };
        t.frame = pos as u64;
        unit.telemetry(&mut t);
        let ch: Vec<String> = (0..channels)
            .map(|c| channel_json(&t.channels[c], audio_gain(&program[c], pos, latency as usize, &outbuf[c][..n])))
            .collect();
        let cl: Vec<&str> =
            t.channels[..channels].iter().map(|c| if c.closed_loop { "true" } else { "false" }).collect();
        writeln!(
            trace,
            "{{\"schema\":1,\"side\":\"sgv2\",\"unit\":\"{unit_name}\",\"case\":\"{case}\",\"frame\":{pos},\
             \"frames\":{n},\"latency\":{latency},\"params\":\"{params}\",\"cfprops\":[{cfprop_ids}],\"health\":{},\
             \"mode\":{{\"failsafe\":{},\"closed_loop\":[{}]}},\"global\":{},\"channels\":[{}]}}",
            health.0,
            health.is_failsafe(),
            cl.join(","),
            global_json(&t.global),
            ch.join(",")
        )?;
        if let Some(w) = audio.as_deref_mut() {
            let mut buf = Vec::with_capacity(n * channels * 4);
            for i in 0..n {
                for c in &outbuf {
                    buf.extend_from_slice(&c[i].to_le_bytes());
                }
            }
            w.write_all(&buf)?;
        }
        pos += n;
    }
    trace.flush()?;
    if let Some(w) = audio {
        w.flush()?;
    }
    Ok(())
}

fn run(a: &Args) -> io::Result<()> {
    let mut program = if a.file_channels > 0 {
        map_channels(read_f32(&a.input, a.file_channels)?, a.channels)
    } else {
        read_f32(&a.input, a.channels)?
    };
    if a.limit_frames > 0 {
        program.iter_mut().for_each(|c| c.truncate(a.limit_frames));
    }
    let sense = match &a.sense {
        Some(p) if a.sense_channels > 0 => read_f32(p, a.sense_channels)?,
        _ => Vec::new(),
    };
    let blob = a.blob.as_ref().map(std::fs::read).transpose()?;
    let mut cpms = None;
    for (id, path) in &a.cfprops {
        let bytes = std::fs::read(path).map_err(|e| io::Error::new(e.kind(), format!("cfprop {id}: {path}: {e}")))?;
        if *id == 64008 && a.unit == UnitKind::Lsm2 {
            let text = String::from_utf8_lossy(&bytes);
            cpms = Some(parse_cpms(&text).ok_or_else(|| {
                io::Error::new(io::ErrorKind::InvalidData, format!("cfprop 64008: {path}: not a CPMS dictionary"))
            })?);
        }
    }
    let cfprop_ids: Vec<u32> = a.cfprops.iter().map(|(id, _)| *id).collect();
    // atsp parameters at frame 0 that the unit cannot change at run time
    // are the rig's pre-render settings: apply them to the decoded preset
    let mut preset = a.preset_params.clone();
    let mut params_at = a.params_at.clone();
    if a.unit == UnitKind::Atsp {
        params_at.retain(|&(f, id, v)| {
            let pre = f == 0 && id != 0;
            if pre {
                preset.push((id, v));
            }
            !pre
        });
    }
    let (mut unit, params) = make_unit(a.unit, blob.as_deref(), cpms, &preset, a.rate);
    let mut events = a.events.clone();
    events.sort_by_key(|(f, _)| *f);
    let kind = a.unit;
    let rebuild = |p: Option<u32>, in_place: bool| -> Option<Box<dyn ProtectionUnit>> {
        let b = blob.as_deref()?;
        match kind {
            UnitKind::Spp3 => {
                let mut q = spp3::Params::from_blob(b).ok()?;
                if let Some(v) = p {
                    q.thermal_state_override = v;
                }
                // the reference's in-place re-Initialize takes the pilot's low
                // level as its high level too (headroom 1 - A_lo, the high
                // target never selected again; rig, restore-uninit/reinit).
                // A reference quirk the plugin does not reproduce: the rig
                // renders it so the comparison is like for like.
                if in_place {
                    for c in q.ch.iter_mut() {
                        c.pilot_ampl_hi_db = c.pilot_ampl_lo_db;
                    }
                }
                Some(Box::new(spp3::Unit::new(q)))
            }
            _ => None,
        }
    };
    let mut stdout_trace;
    let mut file_trace = open_out(&a.out)?;
    let trace: &mut dyn Write = match file_trace.as_mut() {
        Some(f) => f,
        None => {
            stdout_trace = BufWriter::new(io::stdout());
            &mut stdout_trace
        }
    };
    let mut audio = open_out(&a.audio_out)?;
    render(
        &mut unit,
        &params,
        a.unit,
        &a.case,
        &program,
        &sense,
        &a.slices,
        &params_at,
        &cfprop_ids,
        &events,
        &rebuild,
        trace,
        audio.as_mut().map(|w| w as &mut dyn Write),
    )
}

fn main() -> ExitCode {
    let Some(a) = parse_args() else { return usage() };
    match run(&a) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("sgv2-render: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deinterleave_two_channels() {
        let mut b = Vec::new();
        for x in [1.0f32, -1.0, 2.0, -2.0, 3.0] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        let d = deinterleave(&b, 2);
        assert_eq!(d, vec![vec![1.0, 2.0], vec![-1.0, -2.0]]);
    }

    #[test]
    fn audio_gain_aligns_by_latency() {
        let input: Vec<f32> = (0..64).map(|i| ((i * 7) % 5) as f32 - 2.0).collect();
        let mut out = vec![0.0f32; 32];
        for (k, o) in out.iter_mut().enumerate() {
            *o = 0.5 * input[16 + k - 4];
        }
        let g = audio_gain(&input, 16, 4, &out).unwrap();
        assert!((g - 0.5).abs() < 1e-6);
        assert_eq!(audio_gain(&[0.0; 8], 0, 0, &[0.0; 8]), None);
    }

    #[test]
    fn no_blob_renders_failsafe_in_any_slicing() {
        let program = vec![vec![0.25f32; 1000], vec![-0.5f32; 1000]];
        for slices in [&[512usize][..], &[1, 7, 31, 96, 127, 160][..]] {
            let (mut unit, params) = make_unit(UnitKind::Lsm2, None, None, &[], 0);
            let mut trace = Vec::new();
            let mut audio = Vec::new();
            render(
                &mut unit,
                &params,
                UnitKind::Lsm2,
                "t",
                &program,
                &[],
                slices,
                &[],
                &[64009, 7],
                &[],
                &|_, _| None,
                &mut trace,
                Some(&mut audio),
            )
            .unwrap();
            let text = String::from_utf8(trace).unwrap();
            let lines: Vec<&str> = text.lines().collect();
            let mut frames = 0;
            for l in &lines {
                assert!(l.contains("\"failsafe\":true"), "{l}");
                let n: usize = l.split("\"frames\":").nth(1).unwrap().split(',').next().unwrap().parse().unwrap();
                frames += n;
            }
            assert_eq!(frames, 1000);
            let out = deinterleave(&audio, 2);
            assert!(out[0].iter().all(|&x| (x - 0.025).abs() < 1e-7));
            assert!(out[1].iter().all(|&x| (x + 0.05).abs() < 1e-7));
        }
    }

    #[test]
    fn cfprop_fixtures_must_exist_and_are_listed() {
        let dir = std::env::temp_dir().join(format!("sgv2-render-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let input = dir.join("in.f32");
        std::fs::write(&input, [0u8; 4 * 2 * 64]).unwrap();
        let fixture = dir.join("fx.json");
        std::fs::write(&fixture, b"{}").unwrap();
        let out = dir.join("t.jsonl");
        let mut a = Args {
            unit: UnitKind::Lsm2,
            blob: None,
            case: String::from("t"),
            input: input.to_string_lossy().into_owned(),
            channels: 2,
            sense: None,
            sense_channels: 0,
            slices: vec![32],
            out: Some(out.to_string_lossy().into_owned()),
            audio_out: None,
            cfprops: vec![(64009, fixture.to_string_lossy().into_owned())],
            file_channels: 0,
            limit_frames: 0,
            preset_params: Vec::new(),
            rate: 0,
            params_at: Vec::new(),
            events: Vec::new(),
        };
        run(&a).unwrap();
        let text = std::fs::read_to_string(&out).unwrap();
        assert_eq!(text.lines().count(), 2);
        assert!(text.lines().all(|l| l.contains("\"cfprops\":[64009]")), "{text}");
        a.cfprops.push((7, dir.join("absent.json").to_string_lossy().into_owned()));
        assert!(run(&a).is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn host_events_parse() {
        assert_eq!(Event::parse("reset"), Some(Event::Reset));
        assert_eq!(Event::parse("sleep=2"), Some(Event::Sleep));
        assert_eq!(Event::parse("tsjump=240000"), Some(Event::None));
        assert_eq!(Event::parse("classinfo"), Some(Event::None));
        assert_eq!(Event::parse("newinst=0"), Some(Event::Rebuild { state: Some(0), in_place: false }));
        assert_eq!(Event::parse("uninit"), Some(Event::Rebuild { state: None, in_place: true }));
        assert_eq!(Event::parse("newinst_ci=1"), Some(Event::Rebuild { state: Some(1), in_place: false }));
        assert_eq!(Event::parse("bogus"), None);
        assert_eq!(Event::parse("sleep"), None);
    }

    #[test]
    fn blob_errors_fall_back_to_failsafe() {
        let (_, p) = make_unit(UnitKind::Spp3, Some(b"short"), None, &[], 0);
        assert_eq!(p, "blob-Short");
    }

    #[test]
    fn cpms_dictionary() {
        assert_eq!(parse_cpms("{\"cpms_ts_100\": 5453}\n"), Some([50.0, 5.453, 50.0, 50.0]));
        assert_eq!(parse_cpms("{}"), Some([50.0; 4]));
        assert_eq!(parse_cpms("{\"other\": 1}"), None);
    }
}
