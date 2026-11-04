//! spp3 under fuzzed parameters (closed loop on or off), audio, host slicing
//! and sense: properties 1-4, 6 and 7 (see the harness and
//! `tests/adversarial.rs`); the hard path is checked by the oracle.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_core::blob::Pod;
use sgv2_core::spp3::{Params, Unit};
use sgv2_core::{ProtectionUnit, FAILSAFE_GAIN};
use sgv2_core_fuzz::arbitrary::{Result, Unstructured};
use sgv2_core_fuzz::*;

#[global_allocator]
static GLOBAL: Counting = Counting;

fn run(u: &mut Unstructured) -> Result<()> {
    let n = u.int_in_range(1..=8usize)?;
    let cl = u.arbitrary::<bool>()?;
    let base = spp3_base(n, block(u)?, rate(u)?, cl);
    let mut bytes = base.as_bytes().to_vec();
    mutate_words(&mut bytes, u)?;
    let mut p = Params::from_bytes(&bytes).unwrap();
    spp3_exclusions(&mut p);
    let closed = spp3_closed_loop(&p);
    let x = signal(u, n, 96_000)?;
    let ports = 2 * n + u.int_in_range(0..=2usize)?;
    let delay = u.int_in_range(0..=4096usize)?;
    let sense = synth_sense(&x, ports, delay);
    let cuts = slices(u)?;
    let modes: Vec<u8> = (0..u.int_in_range(1..=16usize)?).map(|_| u.arbitrary().unwrap_or(0)).collect();

    let real = Unit::init_error(&p).is_none() && p.channels as usize == n;
    let gain = if real { 1.0 } else { FAILSAFE_GAIN };

    // A: one thermal update per call, under the hard-path oracle (open loop
    // only: the corrected limit is internal)
    let f = sgv2_core::spp3::THERMAL_FRAMES;
    let mut ua = Unit::new(p);
    let mut oracle = (real && !closed).then(|| Spp3HardOracle::new(&p));
    let mut last = 0;
    let a = render(&mut ua, &x, &sense, &[f], &[], &mut |d, t, o| {
        if let Some(or) = oracle.as_mut() {
            if d - last == f {
                let ins: Vec<&[f32]> = x.iter().map(|b| &b[last..d]).collect();
                let outs: Vec<&[f32]> = o.iter().map(|b| &b[last..d]).collect();
                or.step(&ins, &outs, t);
            }
        }
        last = d;
    });
    check_bound(&x, &a.out, 0, gain, "spp3 A");

    // B: the fuzzer's slicing; bit-identical to A on finite input
    let mut ub = Unit::new(p);
    let b = render(&mut ub, &x, &sense, &cuts, &[], &mut nothing);
    check_bound(&x, &b.out, 0, gain, "spp3 B");
    if real && all_finite(&x) {
        assert!(bits(&a.out) == bits(&b.out), "spp3: audio depends on host slicing");
        assert_eq!(tele_bits(&a.tele), tele_bits(&b.tele), "spp3: state depends on host slicing");
        for c in 0..n {
            assert_eq!(ua.sense_power(c).to_bits(), ub.sense_power(c).to_bits(), "spp3: sense power vs slicing");
        }
        assert_eq!(ua.sense_delay(), ub.sense_delay(), "spp3: sense delay depends on host slicing");
        // reset returns to the initial state
        ua.reset();
        let r = render(&mut ua, &x, &sense, &cuts, &[], &mut nothing);
        assert!(bits(&r.out) == bits(&b.out), "spp3: reset is not the initial state");
        assert_eq!(tele_bits(&r.tele), tele_bits(&b.tele), "spp3: reset state differs");
    }

    // C: B's slicing with sense dropouts; D: no sense at all
    let mut uc = Unit::new(p);
    let c = render(&mut uc, &x, &sense, &cuts, &modes, &mut nothing);
    check_bound(&x, &c.out, 0, gain, "spp3 C");
    let d = render(&mut Unit::new(p), &x, &[], &cuts, &[], &mut nothing);
    if !closed {
        check_not_louder(&c.out, &b.out, "spp3 sense dropout vs valid sense");
        check_not_louder(&c.out, &d.out, "spp3 sense dropout vs open loop");
    }
    for ch in 0..n {
        let v = uc.sense_power(ch);
        assert!(v.is_nan() || v.is_finite());
    }
    // model state stays finite (input is clamped below the FINDING 1 range)
    if real {
        for ch in &b.tele.channels[..n] {
            assert!(!ch.gain.is_nan() && (0.0..=1.0).contains(&ch.gain), "spp3 gain {}", ch.gain);
            assert!(!ch.coil_temp_c.is_nan() && !ch.magnet_temp_c.is_nan(), "spp3 NaN model state");
        }
    }
    Ok(())
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let _ = run(&mut u);
});
