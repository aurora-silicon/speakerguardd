//! AudioUnit saved state and presets. `ParamState::decode` is strict (the
//! sections cover the blob, nothing repeats, every value finite), so it is
//! the exact inverse of `encode`; `parse_file` and `AuPreset::from_dict`
//! never panic, and whatever state they accept holds only finite values.
#![no_main]

use aurora_speaker_tuning::aupreset::{parse_file, AuPreset, ParamState};
use aurora_speaker_tuning::plist::Value;
use libfuzzer_sys::fuzz_target;

fn finite(s: &ParamState) -> bool {
    s.sections.iter().flat_map(|x| &x.params).all(|(_, v)| v.is_finite())
}

fuzz_target!(|data: &[u8]| {
    if let Ok(st) = ParamState::decode(data, "fuzz") {
        assert_eq!(st.encode(), data, "decode is not the inverse of encode");
        assert!(finite(&st));
        for s in &st.sections {
            assert_eq!(st.section(s.scope, s.element), Some(s));
        }
    }
    if let Ok(p) = parse_file(data, "fuzz") {
        assert!(finite(&p.state));
        assert!(p.has_state || p.state.sections.is_empty());
    }
    // a preset dictionary whose `data` is the input
    let v = aurora_speaker_tuning_fuzz::common::preset("spp3", "fuzz", &ParamState::default());
    if let Value::Dict(mut d) = v {
        *d.get_mut("data").unwrap() = Value::Data(data.to_vec());
        if let Ok(p) = AuPreset::from_dict(&d, "fuzz") {
            assert_eq!(p.state.encode(), data);
        }
    }
});
