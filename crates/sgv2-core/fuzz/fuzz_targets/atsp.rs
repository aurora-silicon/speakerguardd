//! atsp under fuzzed parameters, audio and host slicing: properties 1-4, 6
//! (sense is ignored) and 7.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_core::atsp::{Params, Unit};
use sgv2_core::blob::Pod;
use sgv2_core::{ProtectionUnit, FAILSAFE_GAIN};
use sgv2_core_fuzz::arbitrary::{Result, Unstructured};
use sgv2_core_fuzz::*;

#[global_allocator]
static GLOBAL: Counting = Counting;

fn run(u: &mut Unstructured) -> Result<()> {
    let n = u.int_in_range(1..=8usize)?;
    let mut base = atsp_base(n, block(u)?, rate(u)?);
    // split the channels over the three slots
    if u.ratio(1, 2)? {
        let mut masks = [0u32; 3];
        for c in 0..n {
            masks[u.int_in_range(0..=2usize)?] |= 1 << c;
        }
        let proto = base.slot[0];
        for (s, m) in base.slot.iter_mut().zip(masks) {
            *s = proto;
            s.channel_mask = m;
            s.gain_link = u.arbitrary::<bool>()? as u32;
        }
        base.gain_link_all = u.arbitrary::<bool>()? as u32;
    }
    let mut bytes = base.as_bytes().to_vec();
    mutate_words(&mut bytes, u)?;
    let p = Params::from_bytes(&bytes).unwrap();
    let x = signal(u, n, 96_000)?;
    let cuts = slices(u)?;
    let junk = synth_sense(&x, u.int_in_range(0..=4usize)?, 3);
    let modes: Vec<u8> = (0..u.int_in_range(1..=8usize)?).map(|_| u.arbitrary().unwrap_or(0)).collect();

    let ok = Unit::init_error(&p).is_none();
    let real = ok && p.channels as usize == n;
    let gain = if real { 1.0 } else { FAILSAFE_GAIN };
    let blk = if ok { p.block_frames as usize } else { 512 };

    let mut chans = vec![(0usize, f64::INFINITY); n];
    if real {
        for (si, s) in p.slot.iter().enumerate() {
            for (c, ch) in chans.iter_mut().enumerate() {
                if s.channel_mask & (1 << c) != 0 {
                    let g = if p.gain_link_all != 0 {
                        0
                    } else if s.gain_link != 0 {
                        100 + si
                    } else {
                        200 + c
                    };
                    *ch = (g, s.temperature_limit as f64 + s.hard_limit_headroom as f64);
                }
            }
        }
    }
    let mut hard = HardPath { x: &x, chans, armed: Vec::new(), last: 0, hits: 0 };
    let mut ua = Unit::new(p);
    let a = render(&mut ua, &x, &[], &[blk], &[], &mut |d, t, o| {
        if real {
            hard.step(d, t, o)
        }
    });
    check_bound(&x, &a.out, 0, gain, "atsp A");

    let b = render(&mut Unit::new(p), &x, &[], &cuts, &[], &mut nothing);
    check_bound(&x, &b.out, 0, gain, "atsp B");
    if real && all_finite(&x) {
        assert!(bits(&a.out) == bits(&b.out), "atsp: audio depends on host slicing");
        assert_eq!(tele_bits(&a.tele), tele_bits(&b.tele), "atsp: state depends on host slicing");
        ua.reset();
        let r = render(&mut ua, &x, &[], &cuts, &[], &mut nothing);
        assert!(bits(&r.out) == bits(&b.out), "atsp: reset is not the initial state");
        assert_eq!(tele_bits(&r.tele), tele_bits(&b.tele), "atsp: reset state differs");
    }
    // the bus is ignored
    let c = render(&mut Unit::new(p), &x, &junk, &cuts, &modes, &mut nothing);
    assert!(bits(&c.out) == bits(&b.out), "atsp: sense changed the audio");
    if real {
        for ch in &b.tele.channels[..n] {
            assert!(!ch.gain.is_nan() && (0.0..=1.0).contains(&ch.gain), "atsp gain {}", ch.gain);
            assert!(!ch.coil_temp_c.is_nan() && !ch.magnet_temp_c.is_nan(), "atsp NaN model state");
        }
    }
    Ok(())
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let _ = run(&mut u);
});
