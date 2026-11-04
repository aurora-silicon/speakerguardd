// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! `aurora-speaker-tuning`: inspect and verify this Mac's speaker tuning.
//!
//! TODO(lane A): `show` (unit, channels, parameter sets, graph and strip)
//! and `verify` against the manifest, with JSON output for the supervisor.

use aurora_speaker_tuning::bundle::Bundle;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let result = match args.get(1).map(String::as_str) {
        Some("verify") => match args.get(2) {
            Some(dir) => Bundle::from_dir(std::path::Path::new(dir)),
            None => Bundle::discover(),
        },
        _ => {
            eprintln!("usage: aurora-speaker-tuning verify [DIR]");
            std::process::exit(2);
        }
    };
    match result {
        Ok(b) => println!("{} ({} files) OK", b.manifest.aid, b.files.len()),
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(1);
        }
    }
}
