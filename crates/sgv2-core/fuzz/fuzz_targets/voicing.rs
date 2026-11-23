//! The blob's tail: the pre-stage the plugin runs ahead of the unit, and
//! the voicing artifact (the external voicing graph's boxes) the voicing
//! plugin builds its chain from. Synthetic artifacts (the crate's
//! `tests/voicing_fixture`) with fuzzed words and a fixed-up CRC, and
//! pre-stages with fuzzed words, through `blob::voicing`,
//! `sgv2_lv2::voicing::Voicing::load` and `sgv2_lv2::load_prestage`. Nothing
//! may panic; whatever loads turns finite program into finite output at any
//! volume, and allocates nothing while it runs.
#![no_main]

#[path = "../../tests/voicing_fixture/mod.rs"]
mod voicing_fixture;

use libfuzzer_sys::fuzz_target;
use sgv2_core::blob;
use sgv2_core_fuzz::arbitrary::{Result, Unstructured};
use sgv2_core_fuzz::*;
use sgv2_lv2::voicing::{Voicing, BLOCK};
use sgv2_lv2::{load_unit, AnyUnit, Role};

#[global_allocator]
static GLOBAL: Counting = Counting;

/// Overwrite a few words of `b[from..to]` with hostile f64 or u32 values.
fn mutate(b: &mut [u8], from: usize, to: usize, u: &mut Unstructured) -> Result<()> {
    if to <= from {
        return Ok(());
    }
    for _ in 0..u.int_in_range(0..=6u8)? {
        let at = from + 4 * u.choose_index((to - from) / 4)?;
        match u.int_in_range(0..=3u8)? {
            0 if at + 8 <= to => {
                let v =
                    [f64::NAN, f64::INFINITY, -1.0, 0.0, 1e300, -1e-300, 48_000.0, 2.0][u.int_in_range(0..=7usize)?];
                let v = if u.ratio(1, 3)? { f64::from_bits(u.arbitrary()?) } else { v };
                b[at..at + 8].copy_from_slice(&v.to_le_bytes());
            }
            1 => b[at..at + 4].copy_from_slice(&hostile_word(u)?.to_le_bytes()),
            2 => b[at..at + 4].copy_from_slice(&u.int_in_range(0..=64u32)?.to_le_bytes()),
            _ => b[at] = u.arbitrary()?,
        }
    }
    Ok(())
}

fn finite_out(lanes: &[Vec<f32>], what: &str) {
    for (c, l) in lanes.iter().enumerate() {
        for (k, v) in l.iter().enumerate() {
            assert!(v.is_finite(), "{what}: lane {c}[{k}] = {v}");
        }
    }
}

fn run(u: &mut Unstructured) -> Result<()> {
    let lanes = [1usize, 2, 4, 6][u.int_in_range(0..=3usize)?];
    let base = voicing_fixture::unit("j413", lanes);
    if u.arbitrary()? {
        // the voicing artifact, bound to `base`
        let Ok(art) = sgv2_core::voicing::artifact::encode(&voicing_fixture::spec(lanes), &base) else {
            return Ok(());
        };
        let mut art = art;
        let n = art.len();
        mutate(&mut art, 20, n - 4, u)?;
        if u.ratio(7, 8)? {
            let crc = blob::crc32(&art[..n - 4]);
            art[n - 4..].copy_from_slice(&crc.to_le_bytes());
        }
        let joined = [base.as_slice(), &art].concat();
        let spec = blob::voicing(&joined);
        let mut v = Voicing::load(Some(&joined), "j413", 48_000.0, lanes);
        assert_eq!(v.ready(), spec.as_ref().is_ok_and(|s| s.is_some()), "load disagrees with blob::voicing");
        if !v.ready() {
            return Ok(());
        }
        let mut x = signal(u, 2, 4 * BLOCK)?;
        finite_only(&mut x);
        let frames = x[0].len().min(BLOCK);
        let mut out = vec![vec![0.0f32; BLOCK]; lanes];
        for _ in 0..u.int_in_range(1..=4u8)? {
            let (scalar, db) = (u.arbitrary::<f64>()?, u.arbitrary::<f64>()?);
            let input: [&[f32]; 2] = [&x[0][..frames], &x[1][..frames]];
            let mut o: Vec<&mut [f32]> = out.iter_mut().map(|b| &mut b[..]).collect();
            let before = allocs();
            v.process(&input, &mut o, frames, scalar, db);
            assert_eq!(allocs(), before, "voicing allocated in process");
            drop(o);
            finite_out(&out, "voicing");
        }
    } else {
        // the pre-stage of a protecting blob (no external voicing)
        let p = spp3_base(lanes, 512, 48_000, false);
        let b = blob::build(
            sgv2_core::UnitKind::Spp3,
            lanes as u32,
            2 * lanes as u32,
            48_000,
            sgv2_core::blob::Pod::as_bytes(&p),
        );
        let pre = sgv2_core::prestage::Params::identity(lanes);
        let Ok(mut b) = blob::with_prestage(&b, &pre) else { return Ok(()) };
        let n = b.len();
        mutate(&mut b, blob::HEADER_LEN + core::mem::size_of_val(&p), n, u)?;
        let (unit, _, _) = load_unit(Some(&b), 48_000.0, lanes, 2 * lanes, Role::Protect);
        assert!(!matches!(unit, AnyUnit::Placeholder(_)));
        let Some(mut pre) = sgv2_lv2::load_prestage(Some(&b), 48_000.0, lanes, &unit) else { return Ok(()) };
        let mut x: Vec<Vec<f32>> = signal(u, lanes, 4 * BLOCK)?;
        finite_only(&mut x);
        let frames = x[0].len();
        let mut bufs: Vec<&mut [f32]> = x.iter_mut().map(|b| &mut b[..frames]).collect();
        let before = allocs();
        pre.process_block(&mut bufs, frames);
        assert_eq!(allocs(), before, "pre-stage allocated in process");
        drop(bufs);
        finite_out(&x, "pre-stage");
    }
    Ok(())
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let _ = run(&mut u);
});
