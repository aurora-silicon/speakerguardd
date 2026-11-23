//! atsp under fuzzed parameters, audio and host slicing: properties 1-4, 6
//! (sense is ignored) and 7, each as `tests/adversarial.rs` states it for
//! the SPEC-ATSP 11 gain path.
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
    // block_frames is reserved: the controller ticks every 5 ms at any value
    let mut base = atsp_base(n, if u.ratio(1, 5)? { 0 } else { block(u)? }, rate(u)?);
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
    let mut ua = Unit::new(p);
    let gain = if real { atsp_peak(&ua) } else { FAILSAFE_GAIN };

    // properties 1, 2, 7 on any host slicing; the bus is ignored
    let b = render(&mut Unit::new(p), &x, &[], &cuts, &[], &mut nothing);
    check_bound(&x, &b.out, 0, gain, "atsp B");
    let c = render(&mut Unit::new(p), &x, &junk, &cuts, &modes, &mut nothing);
    assert!(bits(&c.out) == bits(&b.out), "atsp: sense changed the audio");
    if !real {
        let a = render(&mut ua, &x, &[], &[512], &[], &mut nothing);
        check_bound(&x, &a.out, 0, gain, "atsp A");
        return Ok(());
    }
    for ch in &b.tele.channels[..n] {
        assert!(!ch.gain.is_nan() && (0.0..=1.0).contains(&ch.gain), "atsp gain {}", ch.gain);
        assert!(!ch.coil_temp_c.is_nan() && !ch.magnet_temp_c.is_nan(), "atsp NaN model state");
    }

    // property 4: calls of at most one tick make the telemetry read
    // at each call boundary what the next tick reads. The unit runs every
    // constant held to its published range; the glide's time is stated at
    // 48 kHz (tests/adversarial.rs)
    let win = ua.window_frames();
    // (a long input in calls of a few frames costs seconds: at most 4096 calls)
    let call = u.int_in_range(2..=win.max(2))?.max(x[0].len() / 4096).min(win.max(2));
    let held = p.clamped();
    let chans = (0..n)
        .map(|c| {
            let si = (0..3).find(|&si| held.slot[si].channel_mask & (1 << c) != 0).unwrap_or(0);
            let s = &held.slot[si];
            let g = if held.gain_link_all != 0 {
                0
            } else if s.gain_link != 0 {
                100 + si
            } else {
                200 + c
            };
            (g, s.temperature_limit as f64 + s.hard_limit_headroom as f64)
        })
        .collect();
    let mut hard = HardPath::atsp(&x, chans, p.sample_rate);
    let check_hard = p.sample_rate == 48_000;
    let a = render(&mut ua, &x, &[], &[call], &[], &mut |d, t, o| {
        if check_hard {
            hard.step(d, t, o)
        }
    });
    check_bound(&x, &a.out, 0, gain, "atsp A");

    // property 3 (FINDING 6: on finite input): within one slicing family
    // the audio and the state are the same bit for bit
    if !all_finite(&x) {
        return Ok(());
    }
    let (staged, mono) = (u.arbitrary()?, n == 1);
    let sa = atsp_family_slices(u, win, staged, mono)?;
    let sb = atsp_family_slices(u, win, staged, mono)?;
    // one channel: every call, the last too, whole groups of eight; a
    // one-frame last call (the input running out) is a family of its own
    let mut len = if mono { x[0].len() / 8 * 8 } else { x[0].len() };
    if len > 0 && (ends_on_one(&sa, len) || ends_on_one(&sb, len)) {
        len -= 1;
    }
    if len == 0 || ends_on_one(&sa, len) || ends_on_one(&sb, len) {
        return Ok(());
    }
    let x: Vec<Vec<f32>> = x.iter().map(|c| c[..len].to_vec()).collect();
    let (mut u1, mut u2) = (Unit::new(p), Unit::new(p));
    let r1 = render(&mut u1, &x, &[], &sa, &[], &mut nothing);
    let r2 = render(&mut u2, &x, &[], &sb, &[], &mut nothing);
    assert!(bits(&r1.out) == bits(&r2.out), "atsp: audio depends on host slicing");
    assert_eq!(tele_bits(&r1.tele), tele_bits(&r2.tele), "atsp: state depends on host slicing");
    // reset returns the thermal and gain state to `new`'s; the output
    // dezipper is not restarted, so compare two units that took the same path
    u1.reset();
    u2.reset();
    let r3 = render(&mut u1, &x, &[], &sb, &[], &mut nothing);
    let r4 = render(&mut u2, &x, &[], &sa, &[], &mut nothing);
    assert!(bits(&r3.out) == bits(&r4.out), "atsp: audio differs after reset");
    assert_eq!(tele_bits(&r3.tele), tele_bits(&r4.tele), "atsp: reset state differs");
    Ok(())
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let _ = run(&mut u);
});
