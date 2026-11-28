//! The bundle's smaller formats: the SpTS factory calibration, the
//! installer's and the collector's manifests, the lsm2 masker lattice,
//! `.austrip`/`.propstrip`, the spp3 sidecar, the device configuration's speaker volume range,
//! the machine identity readers and the macOS parity attestations. Nothing
//! may panic; a calibration record that decodes is physical (finite, a
//! positive resistance), a manifest names only admissible paths inside the
//! bundle, a volume range that yields a curve never adds drive, and an
//! attestation only answers for this code's law, the build and the key
//! asked for, with a passing verdict at every slider its release was gated
//! at.
#![no_main]

use aurora_speaker_tuning::hal_volume::{range_from_config, VolumeRange};
use aurora_speaker_tuning::manifest::{admitted, Manifest};
use aurora_speaker_tuning::spp3::Sidecar;
use aurora_speaker_tuning::strip::{AuStrip, PropStrip};
use aurora_speaker_tuning::voicing::parity::{attested, Attestations, LAW};
use aurora_speaker_tuning::{lattice, machine, spts, versions, Diagnostics};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let Some((&sel, rest)) = data.split_first() else { return };
    let mut d = Diagnostics::default();
    match sel % 9 {
        0 => {
            if let Ok(s) = spts::decode(rest, "fuzz", &mut d) {
                for r in &s.records {
                    assert!(r.r_cal_ohm.is_finite() && r.r_cal_ohm > 0.0, "R_cal {}", r.r_cal_ohm);
                    assert!(r.t_cal_c.is_finite());
                }
                assert_eq!(s.checksum_valid, spts::checksum(&rest[..rest.len() - 2]) == s.checksum_stored);
            }
        }
        1 | 2 => {
            let m = if sel % 9 == 1 { Manifest::parse(rest) } else { Manifest::parse_collector(rest) };
            if let Ok(m) = m {
                for r in m.records() {
                    assert!(admitted(&r.path), "inadmissible path {:?}", r.path);
                    assert!(!r.path.starts_with('/') && !r.path.split('/').any(|c| c == ".."), "{:?}", r.path);
                }
            }
        }
        3 => {
            if let Ok(ends) = lattice::parse(rest) {
                assert!(ends.windows(2).all(|w| w[0] < w[1]), "lattice not increasing");
            }
        }
        4 => {
            let _ = AuStrip::parse(rest, "fuzz");
            let _ = PropStrip::parse(rest, "fuzz");
            if let Ok(sc) = Sidecar::parse(rest, "fuzz", &mut d) {
                assert_eq!(sc.channels.len(), sc.num_channels, "sidecar channel count");
            }
        }
        5 => {
            if let Ok(Some((lo, hi))) = range_from_config(rest) {
                let r = VolumeRange { min_db: lo, max_db: hi, source: String::new() };
                if r.curve().is_some() {
                    assert!(lo.is_finite() && hi.is_finite() && lo < hi && hi <= 0.0, "curve for {lo}..{hi}");
                }
            }
        }
        6 => {
            if let Some(id) = machine::unit_id(rest) {
                assert!(machine::valid_unit_id(&id));
            }
            // the compatible's first entry, verbatim: whether it is a
            // board a blob can name is Machine::new's to decide
            if let Some(b) = machine::board_from_compatible(rest) {
                assert!(!b.is_empty() && !b.contains('\0') && rest.starts_with(format!("apple,{b}").as_bytes()));
                if let Ok(m) = machine::Machine::new(&b) {
                    assert!(sgv2_core::blob::machine_field(&m.board).is_some());
                }
            }
            if let Some(b) = machine::board_from_target_type(&String::from_utf8_lossy(rest)) {
                assert!(machine::valid_board(&b), "{b:?}");
            }
        }
        7 => {
            // a build and a key (NUL-terminated), then the attestation file
            let mut parts = rest.splitn(3, |&b| b == 0);
            let build = String::from_utf8_lossy(parts.next().unwrap_or_default()).into_owned();
            let key = String::from_utf8_lossy(parts.next().unwrap_or_default()).into_owned();
            if let Ok(a) = Attestations::parse(parts.next().unwrap_or_default()) {
                let _ = attested(&a, None, &key);
                if let Ok((r, e)) = attested(&a, Some(&build), &key) {
                    assert!(a.law == LAW && r.build == build && e.key == key && e.verdict == "pass");
                    for s in &r.sliders {
                        let pass = e.sliders.iter().any(|v| (v.slider - s).abs() < 1e-9 && v.verdict == "pass");
                        assert!(pass, "attested without a pass at slider {s}");
                    }
                }
            }
        }
        _ => {
            let s = String::from_utf8_lossy(rest);
            versions::check(Some(&s), &mut d);
            let _ = aurora_speaker_tuning::manifest::aid_dsp_database(&s);
        }
    }
});
