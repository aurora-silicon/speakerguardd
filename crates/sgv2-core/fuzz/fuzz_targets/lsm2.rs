//! lsm2 under fuzzed parameters, audio and host slicing: properties 1-3, 6
//! (sense is ignored) and 7.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_core::blob::Pod;
use sgv2_core::lsm2::{Params, Unit};
use sgv2_core::{ProtectionUnit, FAILSAFE_GAIN};
use sgv2_core_fuzz::arbitrary::{Result, Unstructured};
use sgv2_core_fuzz::*;

#[global_allocator]
static GLOBAL: Counting = Counting;

fn run(u: &mut Unstructured) -> Result<()> {
    let n = u.int_in_range(1..=8usize)?;
    let mut base = lsm2_base(n, rate(u)?);
    for f in [
        &mut base.control_sample_peak,
        &mut base.control_displacement,
        &mut base.control_velocity,
        &mut base.control_power,
        &mut base.control_temperature,
        &mut base.link_slow_gains,
        &mut base.unlink_temperature_gains,
    ] {
        *f = u.arbitrary::<bool>()? as u32;
    }
    let mut bytes = base.as_bytes().to_vec();
    mutate_words(&mut bytes, u)?;
    let p = Params::from_bytes(&bytes).unwrap();
    let x = signal(u, n, 48_000)?;
    let cuts = slices(u)?;
    let junk = synth_sense(&x, u.int_in_range(0..=4usize)?, 3);
    let modes: Vec<u8> = (0..u.int_in_range(1..=8usize)?).map(|_| u.arbitrary().unwrap_or(0)).collect();

    let ok = Unit::init_error(&p).is_none();
    let real = ok && p.channels as usize == n;
    let gain = if real { 1.0 } else { FAILSAFE_GAIN };
    let mut ua = Unit::new(p);
    let lat = if real { ua.latency_frames() as usize } else { 0 };

    let a = render(&mut ua, &x, &[], &[256], &[], &mut nothing);
    check_bound(&x, &a.out, lat, gain, "lsm2 A");
    let b = render(&mut Unit::new(p), &x, &[], &cuts, &[], &mut nothing);
    check_bound(&x, &b.out, lat, gain, "lsm2 B");
    // lsm2 sanitizes per sample, so invariance holds on any input
    if real {
        assert!(bits(&a.out) == bits(&b.out), "lsm2: audio depends on host slicing");
        assert_eq!(tele_bits(&a.tele), tele_bits(&b.tele), "lsm2: state depends on host slicing");
        ua.reset();
        let r = render(&mut ua, &x, &[], &cuts, &[], &mut nothing);
        assert!(bits(&r.out) == bits(&b.out), "lsm2: reset is not the initial state");
        assert_eq!(tele_bits(&r.tele), tele_bits(&b.tele), "lsm2: reset state differs");
    }
    let c = render(&mut Unit::new(p), &x, &junk, &cuts, &modes, &mut nothing);
    assert!(bits(&c.out) == bits(&b.out), "lsm2: sense changed the audio");
    if real {
        for ch in &b.tele.channels[..n] {
            assert!(!ch.gain.is_nan() && (0.0..=1.0).contains(&ch.gain), "lsm2 gain {}", ch.gain);
        }
    }
    Ok(())
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let _ = run(&mut u);
});
