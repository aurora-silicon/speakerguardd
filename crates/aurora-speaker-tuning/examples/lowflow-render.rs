//! Offline renderer for the LowFlow differential rig.
//!
//! Input and output are interleaved float32. This helper opens no audio
//! device; all tuning-derived output belongs in the task-private rig area.

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::voicing::lowflow;
use aurora_speaker_tuning::Diagnostics;
use sgv2_core::voicing::lowflow::Run;
use std::io::Write;

fn main() {
    let mut args = std::env::args().skip(1);
    let (mut root, mut aid, mut input, mut output) = (None, None, None, None);
    let (mut volume, mut block, mut inspect) = (0.0, 256usize, false);
    while let Some(arg) = args.next() {
        let mut value = || args.next().expect("missing argument value");
        match arg.as_str() {
            "--tunings-root" => root = Some(value()),
            "--aid" => aid = Some(value().parse::<u32>().expect("AID number")),
            "--in" => input = Some(value()),
            "--out" => output = Some(value()),
            "--volume" => volume = value().parse::<f64>().expect("volume dB"),
            "--block" => block = value().parse::<usize>().expect("block frames"),
            "--inspect" => inspect = true,
            other => panic!("unknown argument {other}"),
        }
    }
    let root = root.expect("--tunings-root");
    let bundle = Bundle::from_tunings_root(std::path::Path::new(&root), aid.expect("--aid")).expect("tuning bundle");
    let mut diagnostics = Diagnostics::default();
    let tuning = Tuning::load(&bundle, &Options::default(), &mut diagnostics).expect("load tuning");
    let stage = tuning.chain.iter().find(|stage| stage.kind == "aufx/flow/appl").expect("LowFlow stage");
    let params =
        lowflow::compile(stage, 48_000.0, &mut diagnostics).expect("compile LowFlow").expect("LowFlow is bypassed");
    let channels = stage.channels.expect("known channel count") as usize;
    if inspect {
        for (index, model) in params.models[..params.model_count].iter().enumerate() {
            eprintln!(
                "model {index}: velocity={}/{}/{} mask={}/{}/{}/{} band={}/{} affected={}/{} attenuation={}/{}@{} enabled={} mute={}",
                model.min_velocity, model.mid_velocity, model.high_velocity,
                model.min_mask_energy_db, model.mid_mask_energy_db,
                model.high_mask_energy_db, model.max_mask_energy_db,
                model.mask_band_min_hz, model.mask_band_max_hz,
                model.min_frequency_hz, model.max_frequency_hz,
                model.mid_attenuation_db, model.max_attenuation_db,
                model.mid_level_db, model.enabled, model.mute
            );
        }
        return;
    }
    let mut run = Run::new(&params, 48_000.0, channels);
    run.set_volume(volume);

    let bytes = std::fs::read(input.expect("--in")).expect("read input");
    let samples: Vec<f32> = bytes.chunks_exact(4).map(|b| f32::from_le_bytes(b.try_into().unwrap())).collect();
    assert_eq!(samples.len() % channels, 0, "input channel alignment");
    let frames = samples.len() / channels;
    let mut planar = vec![vec![0.0f32; frames]; channels];
    for frame in 0..frames {
        for channel in 0..channels {
            planar[channel][frame] = samples[frame * channels + channel];
        }
    }
    let mut at = 0;
    while at < frames {
        let count = block.min(frames - at);
        let mut refs: Vec<&mut [f32]> = planar.iter_mut().map(|v| &mut v[at..at + count]).collect();
        run.process(&mut refs, count);
        at += count;
    }
    let mut writer = std::io::BufWriter::new(std::fs::File::create(output.expect("--out")).unwrap());
    for frame in 0..frames {
        for channel in &planar {
            writer.write_all(&channel[frame].to_le_bytes()).unwrap();
        }
    }
}
