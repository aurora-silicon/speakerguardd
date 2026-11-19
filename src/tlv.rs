// SPDX-License-Identifier: MIT
//! The dB scale of the volume controls, read from the controls' own ALSA
//! TLV (`include/uapi/sound/tlv.h`).  The protective attenuation is written
//! in control steps, so the size of one step has to be the card's, not a
//! constant: a card with 1 dB steps given 0.5 dB arithmetic would apply
//! twice the reduction asked for, and one with 0.25 dB steps only half.
//!
//! Only scales with one fixed dB step per control value are accepted: the
//! daemon rounds a reduction up to whole steps so that the card never
//! applies less than the model asked for, which needs the step to be the
//! same everywhere in the range.  Anything else (no TLV, a linear-gain
//! scale, ranges with differing steps, a min/max scale whose step depends
//! on a range the card narrows while the volume is locked) is refused, and
//! the daemon then never takes the lock: the speakers stay at the card's
//! safe level.

/// `SNDRV_CTL_TLVT_*`
const TLVT_CONTAINER: u32 = 0;
const TLVT_DB_SCALE: u32 = 1;
const TLVT_DB_LINEAR: u32 = 2;
const TLVT_DB_RANGE: u32 = 3;
const TLVT_DB_MINMAX: u32 = 4;
const TLVT_DB_MINMAX_MUTE: u32 = 5;
/// `TLV_DB_SCALE_MASK` / `TLV_DB_SCALE_MUTE` in the DB_SCALE step word.
const DB_SCALE_STEP_MASK: u32 = 0xffff;
const DB_SCALE_MUTE: u32 = 0x10000;

/// One volume control's dB scale: a fixed step per control value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DbStep {
    /// The step in 0.01 dB, as the TLV states it.
    pub centi_db: u32,
    /// The control's lowest value mutes (TLV_DB_SCALE_MUTE).
    pub mute_at_min: bool,
}

impl DbStep {
    /// Control values per dB, the factor the attenuation arithmetic uses
    /// (2.0 for 0.5 dB steps).
    pub fn steps_per_db(&self) -> f64 {
        100.0 / self.centi_db as f64
    }
}

/// One TLV item: its type, its payload and what follows it.
fn item(words: &[u32]) -> Result<(u32, &[u32], &[u32]), String> {
    let [ty, len, rest @ ..] = words else {
        return Err("TLV shorter than its header".into());
    };
    if len % 4 != 0 {
        return Err(format!("TLV type {ty} length {len} is not whole words"));
    }
    let n = (*len / 4) as usize;
    if n > rest.len() {
        return Err(format!("TLV type {ty} length {len} runs past the {} bytes read", rest.len() * 4));
    }
    Ok((*ty, &rest[..n], &rest[n..]))
}

fn db_scale(payload: &[u32]) -> Result<(i32, DbStep), String> {
    let [min, step, ..] = payload else {
        return Err("DB_SCALE TLV without its minimum and step".into());
    };
    let centi_db = step & DB_SCALE_STEP_MASK;
    if centi_db == 0 {
        return Err("DB_SCALE TLV with a 0 dB step".into());
    }
    Ok((*min as i32, DbStep { centi_db, mute_at_min: step & DB_SCALE_MUTE != 0 }))
}

/// The fixed dB step of a control from the TLV words the kernel returned
/// (type, length in bytes, payload; a container is looked into).
pub fn parse(words: &[u32]) -> Result<DbStep, String> {
    let (ty, payload, _) = item(words)?;
    match ty {
        TLVT_CONTAINER => {
            // the first dB item inside
            let mut rest = payload;
            while !rest.is_empty() {
                let (inner, _, next) = item(rest)?;
                if matches!(inner, TLVT_DB_SCALE | TLVT_DB_LINEAR | TLVT_DB_RANGE | TLVT_DB_MINMAX | TLVT_DB_MINMAX_MUTE) {
                    return parse(rest);
                }
                rest = next;
            }
            Err("TLV container holds no dB scale".into())
        }
        TLVT_DB_SCALE => Ok(db_scale(payload)?.1),
        TLVT_DB_RANGE => {
            // (min value, max value, DB_SCALE item) per range; accepted only
            // when the ranges join up into one scale with one step
            let mut rest = payload;
            let mut prev: Option<(i64, i64, DbStep)> = None; // (next value, its dB, step)
            while !rest.is_empty() {
                let [lo, hi, tail @ ..] = rest else {
                    return Err("DB_RANGE TLV with a truncated range".into());
                };
                let (inner, inner_payload, next) = item(tail)?;
                if inner != TLVT_DB_SCALE {
                    return Err(format!("DB_RANGE TLV with a type {inner} sub-range; only fixed-step (DB_SCALE) ranges give one step"));
                }
                let (min_db, step) = db_scale(inner_payload)?;
                let (lo, hi) = (*lo as i32 as i64, *hi as i32 as i64);
                if hi < lo {
                    return Err(format!("DB_RANGE TLV range {lo}..{hi} is empty"));
                }
                if let Some((value, db, s)) = prev {
                    if s.centi_db != step.centi_db {
                        return Err(format!("DB_RANGE TLV steps differ ({} and {} x0.01 dB)", s.centi_db, step.centi_db));
                    }
                    if lo != value || min_db as i64 != db {
                        return Err(format!("DB_RANGE TLV range {lo}..{hi} does not continue the one before it"));
                    }
                }
                let mute = prev.map_or(step.mute_at_min, |(_, _, s)| s.mute_at_min);
                let step = DbStep { mute_at_min: mute, ..step };
                prev = Some((hi + 1, min_db as i64 + (hi - lo + 1) * step.centi_db as i64, step));
                rest = next;
            }
            prev.map(|(_, _, s)| s).ok_or_else(|| "DB_RANGE TLV with no ranges".into())
        }
        TLVT_DB_LINEAR => Err("DB_LINEAR TLV: the dB step is not the same across the range".into()),
        TLVT_DB_MINMAX | TLVT_DB_MINMAX_MUTE => {
            Err("DB_MINMAX TLV: its step depends on the control's range, which the card narrows while the volume is locked".into())
        }
        other => Err(format!("TLV type {other} is not a dB scale")),
    }
}

/// The one step every volume control shares: each control's TLV read
/// result, in order.  A control without a readable dB scale, or controls
/// that disagree, refuse the card.
pub fn volume_step(names: &[String], tlvs: &[std::io::Result<Vec<u32>>]) -> Result<DbStep, String> {
    let mut step: Option<DbStep> = None;
    for (name, tlv) in names.iter().zip(tlvs) {
        let words = tlv.as_ref().map_err(|e| format!("{name:?}: no dB scale (TLV read: {e})"))?;
        let s = parse(words).map_err(|e| format!("{name:?}: {e}"))?;
        match step {
            Some(p) if p.centi_db != s.centi_db => {
                return Err(format!("{name:?}: a {} dB step where the other volume controls have {} dB", s.centi_db as f64 / 100.0, p.centi_db as f64 / 100.0));
            }
            // mute at the minimum only if every control mutes there
            Some(p) => step = Some(DbStep { mute_at_min: p.mute_at_min && s.mute_at_min, ..p }),
            None => step = Some(s),
        }
    }
    step.ok_or_else(|| "no volume controls".into())
}

/// The value to write for a reduction of `db` below `max`: rounded up to
/// the control's step, so the card never applies less than asked, and no
/// lower than `min`.
pub fn reduction_value(max: i64, min: i64, db: f64, steps_per_db: f64) -> i64 {
    (max as f64 - (db * steps_per_db - 1e-9).ceil()).max(min as f64) as i64
}

/// The reduction (dB below `max`) that `value` applies.
pub fn value_reduction(max: i64, value: i64, steps_per_db: f64) -> f64 {
    (max - value) as f64 / steps_per_db
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scale(min_centi: i32, step_centi: u32, mute: bool) -> Vec<u32> {
        vec![TLVT_DB_SCALE, 8, min_centi as u32, step_centi | if mute { DB_SCALE_MUTE } else { 0 }]
    }

    /// The step the daemon used before it read the TLV, and its arithmetic
    /// verbatim, for the comparison below.
    const OLD_STEPS_PER_DB: f64 = 2.0;
    fn old_value(max: i64, min: i64, db: f64) -> i64 {
        (max as f64 - (db * OLD_STEPS_PER_DB - 1e-9).ceil()).max(min as f64) as i64
    }
    fn old_reduction(max: i64, value: i64) -> f64 {
        (max - value) as f64 / OLD_STEPS_PER_DB
    }

    /// The J700's "Speaker Playback Volume" as the kernel reports it
    /// (amixer: min=0, max=127, dBscale-min=-63.50dB, step=0.50dB, mute=1):
    /// every reduction the old constant produced comes out the same, bit
    /// for bit, so nothing changes on that machine.
    #[test]
    fn t8140_scale_is_the_old_constant_exactly() {
        let s = parse(&scale(-6350, 50, true)).unwrap();
        assert_eq!(s, DbStep { centi_db: 50, mute_at_min: true });
        assert_eq!(s.steps_per_db().to_bits(), OLD_STEPS_PER_DB.to_bits());
        assert_eq!((1.0 / s.steps_per_db()).to_bits(), (1.0 / OLD_STEPS_PER_DB).to_bits());
        // the unlocked range, and the locked one (20 dB down) as maximum
        for (max, min) in [(127i64, 0i64), (87, 0)] {
            for i in 0..=8000 {
                let db = i as f64 * 0.01;
                let v = reduction_value(max, min, db, s.steps_per_db());
                assert_eq!(v, old_value(max, min, db), "{db} dB");
                assert_eq!(value_reduction(max, v, s.steps_per_db()).to_bits(), old_reduction(max, v).to_bits());
            }
            for v in min..=max {
                assert_eq!(value_reduction(max, v, s.steps_per_db()).to_bits(), old_reduction(max, v).to_bits());
            }
        }
        // the card's safe level, 40 steps under the maximum: 20 dB
        assert_eq!(value_reduction(127, 87, s.steps_per_db()), 20.0);
    }

    /// The macaudio amplifier volumes on the M1 Pro, M2 and M4 Pro
    /// (dBscale-min=-100.50dB, step=0.50dB, mute=1, 0..201).
    #[test]
    fn amp_speaker_volume_scale() {
        let s = parse(&scale(-10050, 50, true)).unwrap();
        assert_eq!(s.steps_per_db(), 2.0);
        assert_eq!(reduction_value(201, 0, 20.0, s.steps_per_db()), 161);
    }

    #[test]
    fn one_db_steps() {
        let s = parse(&scale(-6300, 100, false)).unwrap();
        assert_eq!(s, DbStep { centi_db: 100, mute_at_min: false });
        assert_eq!(s.steps_per_db(), 1.0);
        // 3.2 dB asked: 4 whole dB applied, never less than asked
        let v = reduction_value(63, 0, 3.2, s.steps_per_db());
        assert_eq!(v, 59);
        assert_eq!(value_reduction(63, v, s.steps_per_db()), 4.0);
        assert_eq!(reduction_value(63, 0, 3.0, s.steps_per_db()), 60);
        // with the old 0.5 dB arithmetic this card would have taken 7 dB
        // for 3.2: the mis-scaling this removes
        assert_eq!(63 - old_value(63, 0, 3.2), 7);
    }

    #[test]
    fn quarter_db_steps() {
        let s = parse(&scale(-6375, 25, true)).unwrap();
        assert_eq!(s.steps_per_db(), 4.0);
        let v = reduction_value(255, 0, 3.1, s.steps_per_db());
        assert_eq!(v, 255 - 13);
        assert_eq!(value_reduction(255, v, s.steps_per_db()), 3.25);
        // the old arithmetic would have applied only 1.75 dB for 3.1 here
        assert_eq!(value_reduction(255, old_value(255, 0, 3.1), 4.0), 1.75);
    }

    #[test]
    fn every_reduction_is_at_least_what_was_asked() {
        for centi in [1u32, 10, 25, 33, 50, 75, 100, 150, 300] {
            let s = parse(&scale(-10000, centi, true)).unwrap();
            let max = 10000 / centi as i64 + 1000;
            for i in 0..=4000 {
                let db = i as f64 * 0.01;
                let v = reduction_value(max, 0, db, s.steps_per_db());
                let applied = value_reduction(max, v, s.steps_per_db());
                assert!(applied >= db - 1e-6, "{centi}: asked {db}, applied {applied}");
                assert!(applied < db + centi as f64 / 100.0 + 1e-6, "{centi}: asked {db}, applied {applied}");
            }
        }
    }

    #[test]
    fn container_and_uniform_range_are_read() {
        let mut c = vec![TLVT_CONTAINER, 16];
        c.extend(scale(-6350, 50, true));
        assert_eq!(parse(&c).unwrap().centi_db, 50);
        // 0..9 from -63.5 dB and 10..127 continuing it, both 0.5 dB
        let r = vec![TLVT_DB_RANGE, 2 * 24, 0, 9, TLVT_DB_SCALE, 8, -6350i32 as u32, 50 | DB_SCALE_MUTE, 10, 127, TLVT_DB_SCALE, 8, -5850i32 as u32, 50];
        assert_eq!(parse(&r).unwrap(), DbStep { centi_db: 50, mute_at_min: true });
    }

    #[test]
    fn scales_without_one_step_are_refused() {
        // ranges with different steps
        let r = vec![TLVT_DB_RANGE, 2 * 24, 0, 9, TLVT_DB_SCALE, 8, -6350i32 as u32, 50, 10, 127, TLVT_DB_SCALE, 8, -5850i32 as u32, 100];
        assert!(parse(&r).unwrap_err().contains("steps differ"));
        // ranges that jump
        let r = vec![TLVT_DB_RANGE, 2 * 24, 0, 9, TLVT_DB_SCALE, 8, -6350i32 as u32, 50, 10, 127, TLVT_DB_SCALE, 8, -4000i32 as u32, 50];
        assert!(parse(&r).unwrap_err().contains("does not continue"));
        assert!(parse(&[TLVT_DB_LINEAR, 8, -6350i32 as u32, 0]).unwrap_err().contains("DB_LINEAR"));
        assert!(parse(&[TLVT_DB_MINMAX, 8, -6350i32 as u32, 0]).unwrap_err().contains("DB_MINMAX"));
        assert!(parse(&[TLVT_DB_MINMAX_MUTE, 8, -6350i32 as u32, 0]).unwrap_err().contains("DB_MINMAX"));
        assert!(parse(&scale(-6350, 0, true)).unwrap_err().contains("0 dB step"));
        assert!(parse(&[0x101, 8, 3, 4]).unwrap_err().contains("not a dB scale"));
        // truncated and malformed
        assert!(parse(&[]).is_err());
        assert!(parse(&[TLVT_DB_SCALE]).is_err());
        assert!(parse(&[TLVT_DB_SCALE, 8, 0]).is_err());
        assert!(parse(&[TLVT_DB_SCALE, 6, 0, 50]).is_err());
        assert!(parse(&[TLVT_DB_SCALE, 4, 0]).is_err());
        assert!(parse(&[TLVT_CONTAINER, 0]).is_err());
    }

    /// A volume control without a TLV (the kernel's ENXIO) refuses the card.
    #[test]
    fn missing_tlv_refuses_the_card() {
        let names = vec!["Speaker Playback Volume".to_string()];
        let err = volume_step(&names, &[Err(std::io::Error::from_raw_os_error(libc::ENXIO))]).unwrap_err();
        assert!(err.contains("Speaker Playback Volume") && err.contains("no dB scale"), "{err}");
        let err = volume_step(&names, &[Ok(vec![])]).unwrap_err();
        assert!(err.contains("Speaker Playback Volume"), "{err}");
        assert!(volume_step(&[], &[]).is_err());
    }

    #[test]
    fn volume_controls_must_agree() {
        let names = vec!["Left Speaker Volume".to_string(), "Right Speaker Volume".to_string()];
        let ok = volume_step(&names, &[Ok(scale(-6350, 50, true)), Ok(scale(-6350, 50, false))]).unwrap();
        assert_eq!(ok, DbStep { centi_db: 50, mute_at_min: false });
        let err = volume_step(&names, &[Ok(scale(-6350, 50, true)), Ok(scale(-6300, 100, true))]).unwrap_err();
        assert!(err.contains("Right Speaker Volume"), "{err}");
        let err = volume_step(&names, &[Ok(scale(-6350, 50, true)), Err(std::io::Error::from_raw_os_error(libc::ENXIO))]).unwrap_err();
        assert!(err.contains("Right Speaker Volume"), "{err}");
    }
}
