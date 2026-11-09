//! Writes `src/reference-25G83.json`: per AID of a macOS Tunings folder, the
//! digest of the speaker files [`aurora_speaker_tuning::reference`] compares.
//!
//! ```text
//! reference-digests TUNINGS_DIR BUILD > crates/aurora-speaker-tuning/src/reference-25G83.json
//! ```
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::reference;
use std::collections::BTreeMap;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (root, build) = (&a[1], &a[2]);
    let mut aids: Vec<u32> = std::fs::read_dir(root)
        .expect("tunings dir")
        .filter_map(|e| e.ok()?.file_name().to_str()?.strip_prefix("AID")?.parse().ok())
        .collect();
    aids.sort_unstable();
    let mut out = BTreeMap::new();
    for aid in aids {
        match Bundle::from_tunings_root(root.as_ref(), aid) {
            Ok(b) => {
                out.insert(format!("AID{aid}"), reference::digest(&format!("AID{aid}"), &b.files));
            }
            Err(e) => eprintln!("AID{aid}: {e}"),
        }
    }
    let doc = serde_json::json!({"build": build, "aids": out});
    println!("{}", serde_json::to_string_pretty(&doc).unwrap());
}
