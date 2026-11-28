//! A thermal hand-off record from outside the process (the supervisor's
//! `provided` slot, fed from the engine page and the persisted file) into
//! `sgv2_lv2::start_stream` on a protecting spp3, lsm2 or atsp unit. Only a
//! record that fits the unit and is fresh may be restored; whatever is
//! restored, the unit then obeys properties 1, 2 and 7 and reports gains
//! in [0, 1].
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_core::{atsp, lsm2, spp3, HandOff, ProtectionUnit, UnitKind, HANDOFF_NODES, MAX_CHANNELS};
use sgv2_core_fuzz::arbitrary::{Result, Unstructured};
use sgv2_core_fuzz::*;
use sgv2_lv2::{handoff::Slot, start_stream, PROVIDED_MAX_AGE_S};

#[global_allocator]
static GLOBAL: Counting = Counting;

fn value(u: &mut Unstructured) -> Result<f64> {
    Ok(match u.int_in_range(0..=9u8)? {
        0 => f64::NAN,
        1 => f64::INFINITY,
        2 => -1.0,
        3 => 0.0,
        4 => 1e300,
        5 => f64::MIN_POSITIVE,
        6 => f64::from(u.int_in_range(0..=400u16)?),
        7 => f64::from(u.int_in_range(0..=1000u16)?) / 1000.0,
        _ => f64::from_bits(u.arbitrary()?),
    })
}

fn run(u: &mut Unstructured) -> Result<()> {
    let n = u.int_in_range(1..=8usize)?;
    let which = u.int_in_range(0..=2u8)?;
    type Case = (UnitKind, Box<dyn ProtectionUnit>, usize, f32, f32);
    let (kind, mut unit, lat, gain, allowance): Case = match which {
        0 => {
            let p = spp3_base(n, 512, 48_000, u.arbitrary()?);
            (UnitKind::Spp3, Box::new(spp3::Unit::new(p)), 0, 1.0, spp3_pilot(&p))
        }
        1 => {
            let un = lsm2::Unit::new(lsm2_base(n, 48_000));
            let lat = un.latency_frames() as usize;
            (UnitKind::Lsm2, Box::new(un), lat, 1.0, LSM2_ALLOWANCE)
        }
        _ => {
            let un = atsp::Unit::new(atsp_base(n, 512, 48_000));
            let peak = atsp_peak(&un);
            (UnitKind::Atsp, Box::new(un), 0, peak, 0.0)
        }
    };
    let mut rec = HandOff::EMPTY;
    rec.kind = if u.ratio(7, 8)? { kind as u32 } else { u.arbitrary()? };
    rec.channels = if u.ratio(7, 8)? { n as u32 } else { u.int_in_range(0..=(MAX_CHANNELS as u32 + 1))? };
    rec.sample_rate = if u.ratio(7, 8)? { 48_000 } else { u.arbitrary()? };
    for c in 0..MAX_CHANNELS {
        for k in 0..HANDOFF_NODES {
            rec.node[c][k] = value(u)?;
        }
        rec.gain[c] = value(u)?;
    }
    let age = value(u)?;
    let x = signal(u, n, 24_000)?;

    let slot = Slot::new();
    let restored = start_stream(unit.as_mut(), &slot, Some((rec, age)));
    if restored {
        assert!(rec.fits(kind, n, 48_000), "restored a record that does not fit the unit");
        assert!(age.is_finite() && age <= PROVIDED_MAX_AGE_S, "restored a record {age} s old");
    }
    let r = render(unit.as_mut(), &x, &[], &[256], &[], &mut nothing);
    check_bound_pilot(&x, &r.out, lat, gain, allowance, "after the hand-off");
    for ch in &r.tele.channels[..n] {
        assert!(!ch.gain.is_nan() && (0.0..=1.0).contains(&ch.gain), "gain {} after the hand-off", ch.gain);
    }
    Ok(())
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let _ = run(&mut u);
});
