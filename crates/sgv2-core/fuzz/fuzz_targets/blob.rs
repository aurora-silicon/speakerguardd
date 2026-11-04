//! Parameter blobs (property 5): raw bytes, and synthetic blobs with fuzzed
//! payload words and header fields (with or without a fixed-up CRC), through
//! every unit's `from_blob` and through `sgv2_lv2::load_unit`, the path the
//! plugin takes. Nothing may panic; whatever loads obeys properties 1, 2 and
//! 7, and a rejection is the 20 dB fail-safe.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_core::blob::{self, Pod};
use sgv2_core::{atsp, lsm2, spp3, ProtectionUnit, UnitKind, FAILSAFE_GAIN};
use sgv2_core_fuzz::arbitrary::{Result, Unstructured};
use sgv2_core_fuzz::*;
use sgv2_lv2::{load_unit, AnyUnit, Role};

#[global_allocator]
static GLOBAL: Counting = Counting;

fn probe(unit: &mut dyn ProtectionUnit, n: usize, lat: usize, gain: f32, x: &[Vec<f32>], what: &str) {
    let n = n.clamp(1, 8);
    let mut x: Vec<Vec<f32>> = (0..n).map(|c| x[c % x.len()].clone()).collect();
    if gain < 1.0 {
        finite_only(&mut x); // FINDING 2
    }
    let sense = synth_sense(&x, 2 * n, 0);
    let r = render(unit, &x, &sense, &[1, 4096, 511, 96], &[], &mut nothing);
    check_bound(&x, &r.out, lat, gain, what);
}

fn run(u: &mut Unstructured) -> Result<()> {
    let mode = u.int_in_range(0..=3u8)?;
    let n = u.int_in_range(1..=8usize)?;
    let bytes = if mode == 0 {
        let k = u.int_in_range(0..=2048usize)?;
        u.bytes(k)?.to_vec()
    } else {
        let (kind, sense, mut payload) = match mode {
            1 => (UnitKind::Spp3, 2 * n as u32, spp3_base(n, 512, 48_000, n % 2 == 0).as_bytes().to_vec()),
            2 => (UnitKind::Lsm2, 0, lsm2_base(n, 48_000).as_bytes().to_vec()),
            _ => (UnitKind::Atsp, 0, atsp_base(n, 512, 48_000).as_bytes().to_vec()),
        };
        mutate_words(&mut payload, u)?;
        if kind == UnitKind::Spp3 {
            if let Ok(mut p) = spp3::Params::from_bytes(&payload) {
                spp3_exclusions(&mut p); // FINDING 4
                payload = p.as_bytes().to_vec();
            }
        }
        if u.ratio(1, 8)? {
            let l = u.int_in_range(0..=payload.len())?;
            payload.truncate(l);
        }
        let mut b = blob::build(kind, n as u32, sense, 48_000, &payload);
        if u.ratio(1, 4)? {
            // header fields, then maybe fix the CRC up again
            let mut head = b[..blob::HEADER_LEN].to_vec();
            mutate_words(&mut head, u)?;
            b[..blob::HEADER_LEN].copy_from_slice(&head);
            if u.arbitrary()? {
                let crc = blob::crc32(&b[blob::HEADER_LEN..]);
                b[32..36].copy_from_slice(&crc.to_le_bytes());
            }
        }
        if u.ratio(1, 16)? {
            let l = u.int_in_range(0..=b.len())?;
            b.truncate(l);
        }
        b
    };
    let x = signal(u, 1, 6000)?;

    let _ = blob::Header::parse(&bytes);
    let _ = sgv2_core::placeholder::Params::decode(&bytes);
    if let Ok(p) = spp3::Params::from_blob(&bytes) {
        let mut un = spp3::Unit::new(p);
        let g = if un.is_failsafe() { FAILSAFE_GAIN } else { 1.0 };
        probe(&mut un, p.channels as usize, 0, g, &x, "spp3 from_blob");
    }
    if let Ok(p) = lsm2::Params::from_blob(&bytes) {
        let mut un = lsm2::Unit::new(p);
        let (g, l) = if un.is_failsafe() { (FAILSAFE_GAIN, 0) } else { (1.0, un.latency_frames() as usize) };
        probe(&mut un, p.channels as usize, l, g, &x, "lsm2 from_blob");
    }
    if let Ok(p) = atsp::Params::from_blob(&bytes) {
        let mut un = atsp::Unit::new(p);
        let g = if un.is_failsafe() { FAILSAFE_GAIN } else { 1.0 };
        probe(&mut un, p.channels as usize, 0, g, &x, "atsp from_blob");
    }

    // the plugin's own path, at the header's geometry or a fuzzed one
    let (hn, hs, hr) = match blob::Header::parse(&bytes) {
        Ok((h, _)) if u.ratio(3, 4)? => (h.channels as usize, h.sense_channels as usize, h.sample_rate as f64),
        _ => (n, u.int_in_range(0..=24usize)?, [48_000.0, 44_100.0, 0.0, f64::NAN][u.int_in_range(0..=3usize)?]),
    };
    let (mut unit, err, _) = load_unit(Some(&bytes), hr, hn, hs, Role::Protect);
    let (g, lat) = match &unit {
        AnyUnit::FailSafe(_) => (FAILSAFE_GAIN, 0),
        AnyUnit::Placeholder(_) => panic!("load_unit gave the placeholder for Role::Protect"),
        other => {
            assert!(err.is_none());
            (1.0, other.unit().latency_frames() as usize)
        }
    };
    if err.is_some() {
        assert!(matches!(unit, AnyUnit::FailSafe(_)), "an error must give the fail-safe");
    }
    probe(unit.as_dyn(), hn, lat, g, &x, "load_unit");
    Ok(())
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let _ = run(&mut u);
});
