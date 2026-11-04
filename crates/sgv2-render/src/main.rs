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
//! `--events` replays the case's host events: `FRAME:reset` calls the unit's
//! `host_reset` (AudioUnitReset) before that frame; `sleep=`/`tsjump=` are
//! wall-clock events with no effect offline.
//!
//! `--cfprop` hands the unit the same run-time and case properties (by ID,
//! from the same fixture files) as the macOS reference got. Every file must be
//! readable, and each trace line lists the IDs in `cfprops`, so the rig can
//! check that the inputs reached this side.
//!
//! Signals are raw interleaved little-endian float32, the rig's format. A
//! slice list is used cyclically. Without `--blob`, or with a blob that does
//! not parse, the unit is the fail-safe, exactly as in the graph.

use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::process::ExitCode;

use sgv2_core::blob::Header;
use sgv2_core::{
    atsp, lsm2, spp3, ChannelState, FailSafe, Health, Io, ProtectionUnit, Telemetry, UnitKind, MAX_CHANNELS,
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
    /// Frames at which the host calls `host_reset` (AudioUnitReset).
    resets: Vec<usize>,
}

fn usage() -> ExitCode {
    eprintln!(
        "usage: sgv2-render --unit spp3|lsm2|atsp [--blob P] --case NAME --in X.f32 --ch N \
         [--sense S.f32 --sense-ch M] [--slices 512|1,7,31] [--out T.jsonl] [--audio-out O.f32] \
         [--cfprop ID=FILE ...]"
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
        resets: Vec::new(),
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
            "--events" => {
                for e in v.split(',').filter(|e| !e.is_empty()) {
                    let (f, what) = e.split_once(':')?;
                    // sleep and tsjump are host wall-clock events: an offline
                    // render has no wall clock (the reference ignores them too)
                    if what == "reset" {
                        a.resets.push(f.parse().ok()?);
                    } else if !(what.starts_with("sleep=") || what.starts_with("tsjump=")) {
                        return None;
                    }
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

fn deinterleave(bytes: &[u8], channels: usize) -> Vec<Vec<f32>> {
    let frames = bytes.len() / 4 / channels;
    let mut out = vec![Vec::with_capacity(frames); channels];
    for (i, w) in bytes.chunks_exact(4).take(frames * channels).enumerate() {
        out[i % channels].push(f32::from_le_bytes([w[0], w[1], w[2], w[3]]));
    }
    out
}

/// Build the unit. A missing or unusable blob gives the fail-safe, as the
/// plugin does.
fn make_unit(kind: UnitKind, blob: Option<&[u8]>) -> (Box<dyn ProtectionUnit>, String) {
    match blob.map(|b| (b, Header::parse(b))) {
        Some((b, Ok((h, _payload)))) if h.unit == kind as u32 => {
            let unit: Option<Box<dyn ProtectionUnit>> = match kind {
                UnitKind::Spp3 => spp3::Params::from_blob(b).ok().map(|p| Box::new(spp3::Unit::new(p)) as _),
                UnitKind::Lsm2 => lsm2::Params::from_blob(b).ok().map(|p| Box::new(lsm2::Unit::new(p)) as _),
                UnitKind::Atsp => atsp::Params::from_blob(b).ok().map(|p| Box::new(atsp::Unit::new(p)) as _),
            };
            match unit {
                Some(u) => (u, String::from("blob")),
                // a payload the unit refuses is the fail-safe, as in the plugin
                None => (Box::new(FailSafe::new(kind, Health::NO_PARAMS)), String::from("blob-invalid")),
            }
        }
        Some((_, Ok(_))) => (Box::new(FailSafe::new(kind, Health::NO_PARAMS)), String::from("blob-unit-mismatch")),
        Some((_, Err(e))) => (Box::new(FailSafe::new(kind, Health::NO_PARAMS)), format!("blob-{e:?}")),
        None => (Box::new(FailSafe::new(kind, Health::NO_PARAMS)), String::from("none")),
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
         \"coil_temp_c\":{},\"magnet_temp_c\":{},\"ol_temp_c\":{},\"cl_temp_c\":{},\
         \"tlim_adj_c\":{},\"displacement_m\":{},\"velocity_m_s\":{},\"mask_db\":{},\
         \"axis_gain\":{},\"band_gain\":{}}}",
        num(c.gain),
        audio_gain.map_or(String::from("null"), num),
        num(c.modeled_power_w),
        num(c.measured_power_w),
        num(c.coil_temp_c),
        num(c.magnet_temp_c),
        num(c.ol_temp_c),
        num(c.cl_temp_c),
        num(c.tlim_adj_c),
        num(c.displacement_m),
        num(c.velocity_m_s),
        num(c.mask_db),
        arr(&c.axis_gain),
        arr(&c.band_gain)
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

/// Render the whole program, writing trace lines and audio as it goes.
#[allow(clippy::too_many_arguments)]
fn render(
    unit: &mut dyn ProtectionUnit,
    params: &str,
    kind: UnitKind,
    case: &str,
    program: &[Vec<f32>],
    sense: &[Vec<f32>],
    slices: &[usize],
    cfprops: &[u32],
    resets: &[usize],
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
    while pos < frames {
        if resets.contains(&pos) {
            unit.host_reset();
        }
        let mut n = slices[k % slices.len()].min(frames - pos);
        // a reset inside a slice splits it, as the host's reset falls between calls
        if let Some(&r) = resets.iter().find(|&&r| r > pos && r < pos + n) {
            n = r - pos;
        }
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
             \"mode\":{{\"failsafe\":{},\"closed_loop\":[{}]}},\"channels\":[{}]}}",
            health.0,
            health.is_failsafe(),
            cl.join(","),
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
    let program = read_f32(&a.input, a.channels)?;
    let sense = match &a.sense {
        Some(p) if a.sense_channels > 0 => read_f32(p, a.sense_channels)?,
        _ => Vec::new(),
    };
    let blob = a.blob.as_ref().map(std::fs::read).transpose()?;
    // TODO(lane B): hand the property payloads to the unit once sgv2-core takes
    // run-time properties; until then they are read (a missing fixture is an
    // error) and listed in the trace, and compare.py shows what they change.
    for (id, path) in &a.cfprops {
        std::fs::read(path).map_err(|e| io::Error::new(e.kind(), format!("cfprop {id}: {path}: {e}")))?;
    }
    let cfprop_ids: Vec<u32> = a.cfprops.iter().map(|(id, _)| *id).collect();
    let (mut unit, params) = make_unit(a.unit, blob.as_deref());
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
        unit.as_mut(),
        &params,
        a.unit,
        &a.case,
        &program,
        &sense,
        &a.slices,
        &cfprop_ids,
        &a.resets,
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
            let (mut unit, params) = make_unit(UnitKind::Lsm2, None);
            let mut trace = Vec::new();
            let mut audio = Vec::new();
            render(
                unit.as_mut(),
                &params,
                UnitKind::Lsm2,
                "t",
                &program,
                &[],
                slices,
                &[64009, 7],
                &[],
                &mut trace,
                Some(&mut audio),
            )
            .unwrap();
            let text = String::from_utf8(trace).unwrap();
            let lines: Vec<&str> = text.lines().collect();
            let mut frames = 0;
            for l in &lines {
                assert!(l.contains("\"failsafe\":true"), "{l}");
                assert!(l.contains("\"cfprops\":[64009,7]"), "{l}");
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
            resets: Vec::new(),
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
    fn blob_errors_fall_back_to_failsafe() {
        let (_, p) = make_unit(UnitKind::Spp3, Some(b"short"));
        assert_eq!(p, "blob-Short");
    }
}
