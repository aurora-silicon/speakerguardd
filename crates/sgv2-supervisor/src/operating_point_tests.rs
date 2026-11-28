// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The operating-point match at its boundaries (KERNEL.md 2.3): the step a
//! gain control's dB TLV states, the tolerance taken from it, and what the
//! supervisor grants and refuses with them.
//!
//! Every number here is synthetic: a made-up card whose amps sit at 16 dBV
//! (8.923 V peak), and made-up parameter full scales around it. No value
//! comes from a machine's tuning.

use crate::kernel::{
    db_scale_centi, db_scale_step, AmpFault, AmpLevel, Kernel, KernelError, Recorder, GAIN_CONTROL_SUFFIX,
};
use crate::lease::Observation;
use crate::machine::{
    full_scale_mv, gain_tolerance_db, same_operating_point, same_operating_point_within, Amp, Machine,
    OPERATING_POINT_TOLERANCE_DB,
};
use crate::supervise::{Event, Expect, Supervisor};

/// The synthetic card's amp gain: 16 dBV, in hundredths of a dB.
const GAIN_CDB: i32 = 1600;

/// Clean as far as the observer can tell: it never vouches for the
/// kernel-side checks.
const CLEAN: Observation = Observation {
    heartbeat_fresh: true,
    plugin_ok: true,
    links_ok: true,
    carrier_ok: true,
    pcm_owner_ok: true,
    pcm_state_ok: true,
    route_ok: false,
    amps_ok: false,
    demand: true,
};

/// Peak mV at full scale for a gain of `centi_db` hundredths of a dBV,
/// worked out here and not through the crate: sqrt(2) * 10^(dBV/20) volts.
fn peak_mv(centi_db: f64) -> f64 {
    1000.0 * 2f64.sqrt() * 10f64.powf(centi_db / 2000.0)
}

/// The full scale a parameter blob would state for a gain `offset` centi-dB
/// away from the card's, in whole mV.
fn stored(offset_cdb: f64) -> u32 {
    peak_mv(f64::from(GAIN_CDB) + offset_cdb).round() as u32
}

const SN6: Machine = Machine { board: "t6s", card_id: "AppleT6", amps: 6, amp: Amp::Sn012776 };
const TAS4: Machine = Machine { board: "t4", card_id: "AppleT4", amps: 4, amp: Amp::Tas2770 };
const FIXED2: Machine = Machine { board: "t2", card_id: "AppleT2", amps: 2, amp: Amp::Max98360a };

/// A healthy card for `m` at the synthetic gain, its gain TLV stating
/// `step` centi-dB a step.
fn card(m: &Machine, step: Option<i32>) -> Recorder {
    let mut k = Recorder::healthy(m.amps);
    k.step_centi_db = step;
    match m.amp {
        Amp::Sn012776 => k.gain = Some((10, GAIN_CDB)),
        Amp::Tas2770 => {
            k.gain = Some((10, GAIN_CDB));
            k.amp_faults.clear();
            k.volume = Some(("Speaker Playback Volume".into(), (201, Some(0)), (161, Some(-2000))));
        }
        Amp::Max98360a => {
            k.gain = None;
            k.amp_faults.clear();
            k.amps = vec![String::new()];
            k.volume = Some(("Speaker Playback Volume".into(), (127, Some(0)), (87, Some(-2000))));
        }
    }
    k
}

fn supervisor(m: &Machine, full_scale_mv: Vec<u32>) -> Supervisor {
    Supervisor::new(Expect::new(m, full_scale_mv, "Secondary"))
}

/// Whether the card is granted on the second clean tick (the first tick
/// never grants), with nothing renewed or held when it is not.
fn grants(s: &mut Supervisor, k: &mut dyn Kernel) -> bool {
    assert_eq!(s.tick(CLEAN, k), Ok(Event::StayedReleased));
    match s.tick(CLEAN, k) {
        Ok(Event::Granted) => {
            assert!(s.granted());
            assert_eq!(s.problem(), None);
            true
        }
        Ok(Event::StayedReleased) => {
            // it stays refused, however long the card looks like this
            for _ in 0..3 {
                assert_eq!(s.tick(CLEAN, k), Ok(Event::StayedReleased));
            }
            assert!(!s.granted());
            assert!(!k.granted().unwrap());
            assert!(s.problem().is_some());
            false
        }
        other => panic!("{other:?}"),
    }
}

/// A card whose gain controls state their own step per amp.
struct StepPerAmp(Recorder, Vec<Option<i32>>);

impl Kernel for StepPerAmp {
    fn renew(&mut self) -> Result<(), KernelError> {
        self.0.renew()
    }
    fn release(&mut self) -> Result<(), KernelError> {
        self.0.release()
    }
    fn granted(&mut self) -> Result<bool, KernelError> {
        self.0.granted()
    }
    fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError> {
        self.0.faults()
    }
    fn levels(&mut self, suffix: &str) -> Result<Vec<AmpLevel>, KernelError> {
        let mut v = self.0.levels(suffix)?;
        if suffix == GAIN_CONTROL_SUFFIX {
            for (l, step) in v.iter_mut().zip(&self.1) {
                l.step_centi_db = *step;
            }
        }
        Ok(v)
    }
    fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
        self.0.speaker_route()
    }
    fn hold_route(&mut self) -> Result<bool, KernelError> {
        self.0.hold_route()
    }
    fn free_route(&mut self) -> Result<(), KernelError> {
        self.0.free_route()
    }
}

#[test]
fn the_synthetic_card_is_what_the_comments_say() {
    // 16 dBV is 6.3096 V rms, 8.9231 V peak; 20 dBV is 10 V rms
    assert!((peak_mv(1600.0) - 8923.1).abs() < 0.05, "{}", peak_mv(1600.0));
    assert!((full_scale_mv(GAIN_CDB) - peak_mv(1600.0)).abs() < 1e-6);
    assert!((full_scale_mv(2000) - 14_142.136).abs() < 1e-3);
    assert_eq!(stored(0.0), 8923);
}

#[test]
fn the_step_is_read_only_from_a_whole_db_scale_tlv() {
    // [type, length in bytes, minimum, step | mute << 16]
    assert_eq!(db_scale_step(&[1, 8, 0, 25]), Some(25));
    // the smallest and the largest step the 16-bit field holds
    assert_eq!(db_scale_step(&[1, 8, 0, 1]), Some(1));
    assert_eq!(db_scale_step(&[1, 8, 0, 300]), Some(300));
    assert_eq!(db_scale_step(&[1, 8, 0, 0xffff]), Some(0xffff));
    // the mute flag is not part of the step, and alone it is no step
    assert_eq!(db_scale_step(&[1, 8, 0, 0x1_0000 | 300]), Some(300));
    assert_eq!(db_scale_step(&[1, 8, 0, 0x1_0000]), None);
    // the minimum is not the step
    assert_eq!(db_scale_step(&[1, 8, 77, 25]), Some(25));
    assert_eq!(db_scale_step(&[1, 8, 77, 0]), None);
    // a longer TLV still starts with the two words
    assert_eq!(db_scale_step(&[1, 12, 0, 25, 0]), Some(25));
    // every other TLV type has no fixed step, whatever sits in its words:
    // container, linear, range, min/max, min/max with mute
    for other in [0, 2, 3, 4, 5, 6, 0x101, u32::MAX] {
        assert_eq!(db_scale_step(&[other, 8, 0, 25]), None, "type {other}");
    }
    // a length word short of the two data words
    for len in [0, 4, 7] {
        assert_eq!(db_scale_step(&[1, len, 0, 25]), None, "length {len}");
    }
    // a truncated read: fewer words than the header promises (no panic)
    assert_eq!(db_scale_step(&[1, 8, 0]), None);
    assert_eq!(db_scale_step(&[1, 8]), None);
    assert_eq!(db_scale_step(&[1]), None);
    assert_eq!(db_scale_step(&[]), None);
}

/// The step and the level come from the same TLV, so they must agree: one
/// setting up is one step up, as `db_scale_centi` reads the two settings.
#[test]
fn the_step_is_the_distance_between_neighbouring_settings() {
    for step in [1u32, 2, 25, 50, 100, 255, 256, 300, 1000] {
        for mute in [0u32, 0x1_0000] {
            for min in [-6350i32, 0, 1100] {
                let tlv = [1, 8, min as u32, step | mute];
                let s = db_scale_step(&tlv).unwrap();
                assert_eq!(s, step as i32);
                for value in 1..20 {
                    let lo = db_scale_centi(&tlv, value, 0).unwrap();
                    let hi = db_scale_centi(&tlv, value + 1, 0).unwrap();
                    assert_eq!(hi - lo, s, "{tlv:?} at {value}");
                }
            }
        }
    }
}

#[test]
fn the_gain_tolerance_at_its_edges() {
    let tol = |step| gain_tolerance_db(step);
    // half the step, below the cap
    assert_eq!(tol(1), Some(0.005));
    assert_eq!(tol(2), Some(0.01));
    assert_eq!(tol(10), Some(0.05));
    assert_eq!(tol(49), Some(0.245));
    // the cap is reached at 0.5 dB a step and holds above it
    assert_eq!(tol(50), Some(OPERATING_POINT_TOLERANCE_DB));
    assert_eq!(tol(51), Some(OPERATING_POINT_TOLERANCE_DB));
    assert_eq!(tol(0xffff), Some(OPERATING_POINT_TOLERANCE_DB));
    assert_eq!(tol(i32::MAX), Some(OPERATING_POINT_TOLERANCE_DB));
    // no step, no tolerance
    assert_eq!(tol(0), None);
    assert_eq!(tol(-1), None);
    assert_eq!(tol(i32::MIN), None);
    // never wider than the cap, never zero or negative, and growing with
    // the step until the cap
    let mut last = 0.0;
    for step in 1..=400 {
        let t = tol(step).unwrap();
        assert!(t > 0.0 && t <= OPERATING_POINT_TOLERANCE_DB, "step {step}: {t}");
        assert!(t >= last, "step {step}");
        assert!(t > last || step > 50, "step {step}");
        last = t;
    }
}

#[test]
fn the_match_window_is_symmetric_in_db_and_closed() {
    let expected = stored(0.0);
    for tol_cdb in [0.5, 5.0, 12.5, 25.0] {
        let tol = tol_cdb / 100.0;
        // just inside and just outside, louder and quieter
        for sign in [1.0, -1.0] {
            let inside = f64::from(expected) * 10f64.powf(sign * (tol - 1e-9) / 20.0);
            let outside = f64::from(expected) * 10f64.powf(sign * (tol + 1e-9) / 20.0);
            assert!(same_operating_point_within(inside, expected, tol), "{sign} {tol}");
            assert!(!same_operating_point_within(outside, expected, tol), "{sign} {tol}");
        }
    }
    // the very value is the operating point at any tolerance, zero included
    assert!(same_operating_point_within(f64::from(expected), expected, 0.0));
    assert!(same_operating_point_within(f64::from(expected), expected, 0.125));
    // a tolerance that is no number, or negative, matches nothing
    assert!(!same_operating_point_within(f64::from(expected), expected, f64::NAN));
    assert!(!same_operating_point_within(f64::from(expected), expected, -0.125));
    // a reading that is no voltage matches nothing
    for measured in [0.0, -0.0, -f64::from(expected), f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(!same_operating_point_within(measured, expected, 0.25), "{measured}");
        assert!(!same_operating_point_within(measured, expected, 1e6), "{measured}");
        assert!(!same_operating_point(measured, expected), "{measured}");
    }
    // parameters without a full scale match nothing
    assert!(!same_operating_point_within(f64::from(expected), 0, 0.25));
    assert!(!same_operating_point_within(0.0, 0, 0.25));
    // the two guards (a positive reading, a positive full scale) do not
    // lean on the tolerance being finite
    for measured in [0.0, -0.0, -f64::from(expected), f64::NAN, f64::NEG_INFINITY] {
        assert!(!same_operating_point_within(measured, expected, f64::INFINITY), "{measured}");
    }
    assert!(!same_operating_point_within(f64::from(expected), 0, f64::INFINITY));
    // the wrapper is the 0.25 dB window
    assert!(same_operating_point(peak_mv(1624.0), expected));
    assert!(!same_operating_point(peak_mv(1626.0), expected));
    assert!(same_operating_point(peak_mv(1576.0), expected));
    assert!(!same_operating_point(peak_mv(1574.0), expected));
}

/// What the half-step rule is for: a stored full scale names one gain
/// setting. Over every whole-mV full scale from 1 V to 20 V and every
/// setting of a control with the given step, no value matches two settings.
#[test]
fn no_full_scale_matches_two_settings_of_one_control() {
    for step in [1, 2, 5, 10, 25, 49, 50, 51, 100, 300] {
        let tol = gain_tolerance_db(step).unwrap();
        let mut matched = 0u32;
        for mv in 1_000u32..=20_000 {
            // the setting nearest to this full scale, and its neighbours
            let cdb = 2000.0 * (f64::from(mv) / (1000.0 * 2f64.sqrt())).log10();
            let nearest = (cdb / f64::from(step)).round() as i32;
            let hits = (nearest - 2..=nearest + 2)
                .filter(|k| same_operating_point_within(full_scale_mv(k * step), mv, tol))
                .count();
            assert!(hits <= 1, "step {step}: {mv} mV matches {hits} settings");
            matched += hits as u32;
        }
        // and the window is not empty: up to 0.5 dB a step every value
        // within reach of a setting finds it (allowing for values that
        // fall on the edge between two)
        if step <= 50 {
            assert!(matched >= 18_900, "step {step}: only {matched} of 19001 values matched a setting");
        }
    }
}

/// A full scale kept to two decimals of a volt, rounded or cut, still names
/// its own setting at 0.25 dB a step and coarser.
#[test]
fn two_decimals_of_a_volt_still_name_their_setting() {
    for step in [25, 50, 100] {
        let tol = gain_tolerance_db(step).unwrap();
        for k in 0..=(2600 / step) {
            let mv = full_scale_mv(k * step);
            for kept in [(mv / 10.0).floor() * 10.0, (mv / 10.0).round() * 10.0] {
                assert!(same_operating_point_within(mv, kept as u32, tol), "step {step}, {mv} mV kept as {kept}");
            }
        }
    }
}

/// The supervisor's grant at each side of the window, for parameter blobs
/// compiled a little away from the card's gain. Offsets are centi-dB.
#[test]
fn the_grant_follows_half_the_step_and_never_more_than_the_cap() {
    // (step, an offset inside the window, an offset outside it)
    let rows: [(i32, f64, f64); 9] = [
        (1, 0.0, 1.0),
        (2, 0.5, 1.5),
        (10, 4.0, 6.0),
        (25, 11.5, 13.5),
        (49, 24.0, 25.0),
        (50, 24.0, 26.0),
        // half of these steps is wider than the cap: the cap decides
        (51, 24.75, 25.25),
        (100, 24.0, 26.0),
        (1000, 24.0, 26.0),
    ];
    for m in [&SN6, &TAS4] {
        for (step, inside, outside) in rows {
            for sign in [1.0, -1.0] {
                let mut k = card(m, Some(step));
                let mut s = supervisor(m, vec![stored(sign * inside); m.amps]);
                assert!(grants(&mut s, &mut k), "{:?} step {step}, offset {}", m.amp, sign * inside);

                let mut k = card(m, Some(step));
                let mut s = supervisor(m, vec![stored(sign * outside); m.amps]);
                assert!(!grants(&mut s, &mut k), "{:?} step {step}, offset {}", m.amp, sign * outside);
                assert!(s.problem().unwrap().contains("operating point"), "{:?}", s.problem());
                assert_eq!(k.renewals, 0);
            }
            // the neighbouring setting, either way, is another operating
            // point at every step
            for neighbour in [f64::from(step), -f64::from(step)] {
                let mut k = card(m, Some(step));
                let mut s = supervisor(m, vec![stored(neighbour); m.amps]);
                assert!(!grants(&mut s, &mut k), "{:?} step {step}, neighbour {neighbour}", m.amp);
            }
        }
    }
}

#[test]
fn every_channel_is_held_to_the_step_not_only_the_first() {
    // 0.25 dB a step: 0.2 dB off is inside the old fixed 0.25 dB, outside
    // half this control's step
    for wrong in 0..SN6.amps {
        let mut full_scales = vec![stored(0.0); SN6.amps];
        full_scales[wrong] = stored(-20.0);
        let mut k = card(&SN6, Some(25));
        let mut s = supervisor(&SN6, full_scales.clone());
        assert!(!grants(&mut s, &mut k), "channel {wrong}");
        let p = s.problem().unwrap();
        assert!(p.contains("operating point") && p.contains(&format!("(channel {wrong})")), "{p}");
        // the same blob on a 0.5 dB card is one operating point
        let mut k = card(&SN6, Some(50));
        let mut s = supervisor(&SN6, full_scales);
        assert!(grants(&mut s, &mut k), "channel {wrong}");
    }
}

#[test]
fn a_gain_step_that_is_not_positive_never_grants() {
    for m in [&SN6, &TAS4] {
        for step in [0, -1, -50, i32::MIN] {
            // the gain itself is exactly the blob's
            let mut k = card(m, Some(step));
            let mut s = supervisor(m, vec![stored(0.0); m.amps]);
            assert!(!grants(&mut s, &mut k), "step {step}");
            assert!(s.problem().unwrap().contains(&format!("amp gain step {step} cdB")), "{:?}", s.problem());
            assert_eq!(k.renewals, 0);
        }
        // and none at all
        let mut k = card(m, None);
        let mut s = supervisor(m, vec![stored(0.0); m.amps]);
        assert!(!grants(&mut s, &mut k));
        assert!(s.problem().unwrap().contains("no fixed dB step"), "{:?}", s.problem());
        assert_eq!(k.renewals, 0);
    }
}

#[test]
fn every_amp_must_state_the_same_step() {
    let n = SN6.amps;
    let blob = || vec![stored(0.0); n];
    // all agree
    let mut k = StepPerAmp(card(&SN6, Some(50)), vec![Some(50); n]);
    assert!(grants(&mut supervisor(&SN6, blob()), &mut k));
    for odd in 0..n {
        // one amp on a finer step, a coarser one, a step of nothing, or
        // without a step at all: wherever it sits in the list
        for other in [Some(25), Some(100), Some(0), None] {
            let mut steps = vec![Some(50); n];
            steps[odd] = other;
            let mut k = StepPerAmp(card(&SN6, Some(50)), steps);
            let mut s = supervisor(&SN6, blob());
            assert!(!grants(&mut s, &mut k), "amp {odd} at {other:?}");
            let p = s.problem().unwrap();
            if odd == 0 && other.is_none() {
                assert!(p.contains("no fixed dB step"), "{p}");
            } else if odd == 0 {
                // the others differ from the first
                assert!(p.contains("Amp 1 gain step"), "{p}");
            } else {
                assert!(p.contains(&format!("Amp {odd} gain step")), "{p}");
            }
            assert_eq!(k.0.renewals, 0);
        }
    }
    // only the first amp has a step
    let mut steps = vec![None; n];
    steps[0] = Some(50);
    let mut k = StepPerAmp(card(&SN6, Some(50)), steps);
    assert!(!grants(&mut supervisor(&SN6, blob()), &mut k));
}

#[test]
fn a_held_grant_is_released_on_the_tick_the_step_stops_holding() {
    let n = SN6.amps;
    let held = || {
        let mut k = StepPerAmp(card(&SN6, Some(25)), vec![Some(25); n]);
        let mut s = supervisor(&SN6, vec![stored(0.0); n]);
        assert!(grants(&mut s, &mut k));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Renewed));
        (s, k)
    };
    let released = |s: &mut Supervisor, k: &mut StepPerAmp, what: &str| {
        assert!(matches!(s.tick(CLEAN, k), Ok(Event::Released { .. })), "{what}");
        assert!(!k.0.held, "{what}");
        assert!(!s.granted(), "{what}");
        assert!(s.problem().unwrap().contains(what), "{:?}", s.problem());
        // not a latch: nothing is granted while it lasts
        assert_eq!(s.latched_fault(), None);
        assert_eq!(s.tick(CLEAN, k), Ok(Event::StayedReleased));
    };

    // the gain moves one 0.25 dB setting: inside the old fixed 0.25 dB
    for moved in [GAIN_CDB - 25, GAIN_CDB + 25] {
        let (mut s, mut k) = held();
        k.0.gain = Some((9, moved));
        released(&mut s, &mut k, "operating point");
    }
    // the last amp's control starts stating another step
    let (mut s, mut k) = held();
    k.1[n - 1] = Some(50);
    released(&mut s, &mut k, "Amp 5 gain step");
    // one amp's step stops reading
    let (mut s, mut k) = held();
    k.1[3] = None;
    released(&mut s, &mut k, "Amp 3 gain step");
    // every amp's step reads 0
    let (mut s, mut k) = held();
    k.1 = vec![Some(0); n];
    released(&mut s, &mut k, "amp gain step 0 cdB");

    // and the card is granted again once the step is back (two clean ticks)
    k.1 = vec![Some(25); n];
    assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
    assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
    assert_eq!(s.problem(), None);

    // a held grant whose step gets coarser keeps its grant: the gain is
    // still the blob's, and the window only widens to the cap
    let (mut s, mut k) = held();
    k.1 = vec![Some(100); n];
    assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Renewed));
}

/// Amps without a gain control have no step to read: their channels are
/// held to the blob's smallest full scale within the 0.25 dB cap, whatever
/// the card's other controls state.
#[test]
fn fixed_gain_amps_keep_the_quarter_db_window() {
    let base = 5_000u32;
    let off = |cdb: f64| (f64::from(base) * 10f64.powf(cdb / 2000.0)).round() as u32;
    for step in [None, Some(0), Some(1), Some(25), Some(50)] {
        // 0.24 dB apart: one operating point (wider than half of 0.25 dB)
        for full_scales in [vec![base, off(24.0)], vec![off(24.0), base], vec![base, base]] {
            let mut k = card(&FIXED2, step);
            let mut s = supervisor(&FIXED2, full_scales.clone());
            assert!(grants(&mut s, &mut k), "{step:?} {full_scales:?}");
        }
        // 0.26 dB apart: not one operating point (inside twice the cap)
        for full_scales in [vec![base, off(26.0)], vec![off(26.0), base]] {
            let mut k = card(&FIXED2, step);
            let mut s = supervisor(&FIXED2, full_scales.clone());
            assert!(!grants(&mut s, &mut k), "{step:?} {full_scales:?}");
            assert!(s.problem().unwrap().contains("operating point"), "{:?}", s.problem());
            assert_eq!(k.renewals, 0);
        }
    }
}
