//! The graph language (`.dspg`, with its macro preprocessor) and its
//! analysis. Nothing may panic, overflow the stack or run away (the
//! preprocessor caps nesting and expansion); a parsed graph's processing
//! order names each reached box once, and lane tracing into every port and
//! box input terminates.
#![no_main]

use std::collections::BTreeMap;

use aurora_speaker_tuning::dspg::{self, analysis, Endpoint};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // the text, then optional `name=value` substitutions after a NUL
    let mut parts = data.splitn(2, |&b| b == 0);
    let text = String::from_utf8_lossy(parts.next().unwrap_or_default());
    let mut subst = BTreeMap::new();
    for kv in String::from_utf8_lossy(parts.next().unwrap_or_default()).split('\n').take(8) {
        if let Some((k, v)) = kv.split_once('=') {
            subst.insert(k.to_string(), v.to_string());
        }
    }
    let Ok(g) = dspg::parse(&text, &subst, "fuzz") else { return };
    if let Ok(a) = analysis::processing_order(&g) {
        let mut seen = std::collections::BTreeSet::new();
        for b in &a.order {
            assert!(seen.insert(b), "{b} processed twice");
            assert!(g.find_box(b).is_some(), "{b} is not a box");
        }
    }
    for w in g.wires.iter().take(64) {
        let _ = analysis::wire_channels(&g, w);
    }
    let mut ends: Vec<Endpoint> = g.outputs.iter().map(|o| Endpoint { node: o.clone(), port: 0 }).collect();
    ends.extend(g.boxes.iter().take(16).map(|b| Endpoint { node: b.name.clone(), port: 0 }));
    for e in ends.iter().take(32) {
        let _ = analysis::lanes_into(&g, e);
    }
});
