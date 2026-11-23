//! A synthetic spp3, lsm2 or atsp bundle (the crate's own fixtures) with
//! fuzzed saved parameters, plist values and file bytes, through
//! `Tuning::load` and `compile` for the fixtures' four-amp machine. Nothing
//! may panic. Whatever compiles is a blob the plugin runs as a protecting
//! unit (never its fail-safe: the compiler refuses those), with the header
//! the compiler reports, and one the supervisor accepts: its voicing and
//! pre-stage decode, every channel states a real operating point, and the
//! deployment has an identity.
#![no_main]

use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::compile;
use aurora_speaker_tuning::machine::Machine;
use aurora_speaker_tuning::plist::{self, binary};
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::Diagnostics;
use aurora_speaker_tuning_fuzz::arbitrary::{Result, Unstructured};
use aurora_speaker_tuning_fuzz::common::test_unit;
use aurora_speaker_tuning_fuzz::*;
use libfuzzer_sys::fuzz_target;

fn run(u: &mut Unstructured) -> Result<()> {
    let real = real_bundles();
    let pick = u.int_in_range(0..=2 + real.len())?;
    let mut f = if pick < 3 {
        let fam = [Family::Spp3, Family::Lsm2, Family::Atsp][pick];
        let mut state = fam.state();
        for _ in 0..u.int_in_range(0..=6u8)? {
            mutate_state(&mut state, u)?;
        }
        files(fam, &state)
    } else {
        real[pick - 3].bundle.files.clone()
    };
    let paths: Vec<String> = f.keys().cloned().collect();
    for _ in 0..u.int_in_range(0..=3u8)? {
        let p = &paths[u.choose_index(paths.len())?];
        let Some(data) = f.get_mut(p) else { continue };
        match u.int_in_range(0..=7u8)? {
            // a value inside a plist (the austrip's own leaves included)
            0..=4 => {
                if let Ok((mut v, fmt)) = plist::parse(data) {
                    mutate_plist(&mut v, u)?;
                    *data = match fmt {
                        plist::Format::Binary => binary::write(&v),
                        plist::Format::Xml => plist::xml::write(&v).into_bytes(),
                    };
                }
            }
            // bytes of any file (SpTS, the graph text, ...)
            5 | 6 if !data.is_empty() => {
                for _ in 0..u.int_in_range(1..=4u8)? {
                    let at = u.choose_index(data.len())?;
                    data[at] = u.arbitrary()?;
                }
            }
            _ => {
                f.remove(p);
            }
        }
    }
    // a Mac's own bundle keeps its manifest: the files stand for verified
    // ones with these contents (the reader trusts what the manifest proved)
    let (b, m) = match pick.checked_sub(3).map(|k| &real[k]) {
        None => match bundle(f) {
            Some(b) => (b, Machine::with_unit("j413", &test_unit()).unwrap()),
            None => return Ok(()),
        },
        Some(r) => {
            let b = Bundle {
                manifest: r.bundle.manifest.clone(),
                files: f,
                source: r.bundle.source.clone(),
                verified: true,
                unlisted: Vec::new(),
            };
            (b, r.machine.clone())
        }
    };
    let mut d = Diagnostics::default();
    let Ok(t) = Tuning::load(&b, &Options::default(), &mut d) else { return Ok(()) };
    let Ok(c) = compile::compile(&t, &m, &mut d) else { return Ok(()) };
    check_compiled(&c);
    Ok(())
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let _ = run(&mut u);
});
