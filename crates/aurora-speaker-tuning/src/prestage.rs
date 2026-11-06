// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The pre-stage: the speaker graph's own filtering ahead of the unit.
//!
//! macOS shapes every speaker channel before its protection unit: the
//! graph's parametric-filter boxes (`aufx/tmst`) high-pass the program and,
//! per speaker, apply the crossover (high-pass on the tweeters, low-pass on
//! the woofers), shelves, gains and cuts. spp3 limits coil temperature
//! only; how hard a cone is driven at low frequency is set by this
//! filtering. [`derive`] reads it from the tuning, for every unit channel
//! along its lane through the graph, and returns second-order sections for
//! `sgv2_core::prestage`.
//!
//! A filter box's saved state, as the unit reads it:
//!
//! * global: 0 bypass, 1 group bypass (bit per group), 4 input gain (dB),
//!   5 highest filter order, 9 polarity flip (bit per channel), 11 channel
//!   mute (bit per channel), 13 output gain (dB);
//! * 50 filter slots, slot `i` at IDs `14 + 10 i ..`: state, type, gain
//!   (dB), frequency, quality, order, a fourth frequency, channel mask (bit
//!   per box channel), phase channels, group;
//! * a slot runs when bit 1 of its state is set.
//!
//! Slot types, from the unit's measured response on test signals:
//!
//! | type | response | carried as |
//! |---|---|---|
//! | 1 | peaking cut or boost | RBJ peaking biquad |
//! | 3 | Butterworth low-pass of the slot's order, -3 dB at the frequency | bilinear, prewarped |
//! | 4 | Butterworth high-pass, likewise | bilinear, prewarped |
//! | 9 | flat gain | gain |
//! | 11 | low shelf | RBJ low shelf |
//! | 12 | high shelf | RBJ high shelf |
//! | 0, 2 | band and high shelves that stay between 0 dB and their gain | left out when the gain is 0 dB or more (leaving out a boost only takes drive away); refused for a cut |
//!
//! Every other type is refused, as are a group bypass on a running slot
//! (its meaning is not settled), a slot whose values are out of range, and
//! a volume automation aimed at a high-pass, low-pass, gain or shelf slot.
//! Polarity flips are not carried: they change no level. A muted channel
//! gets gain 0.
//!
//! The graph's multiband compressor box (`aufx/mstk`) ahead of the unit is
//! carried as the pre-stage's dynamics ([`dynamics`]), from its saved
//! state: input and output gain (1, 2), three fourth-order crossovers
//! (4, 6, 8; orders 5, 7, 9), per band `b` at `10 + 13 b` enable, bypass,
//! solo, mute, downward threshold, ratio and range, upward and expander
//! ratios, makeup; release time (62, ms) and release switch (63); per-band
//! input gains (65-68) and saturation (71, 73, 75, 77). A compressor the
//! dynamics cannot carry exactly in the direction that matters (upward or
//! expanding branches, saturation, solo, per-band input gain, another
//! crossover order, release switched off) refuses the compile.
//!
//! The compressor runs at its place in the graph: on each lane the filters
//! of the boxes before it run ahead of the dynamics and the rest after
//! (`FLAG_ORDERED`), so its detector sees what macOS's sees (the sub-bass a
//! later crossover removes included).
//!
//! The channel's filter response is then scaled so it never exceeds unity
//! at any frequency (stricter than macOS only where the tuning boosts). The
//! dynamics carry the compressor's own gain staging, as macOS applies it.
//!
//! Every value read must be in the saved state: a field left out refuses
//! the compile rather than reading as "off" or 0 dB. A filter box the strip
//! gives no preset refuses, as does any box ahead of the unit other than the
//! filter and compressor boxes, the voicing boxes the voicing graph stands
//! in for ([`NOT_CARRIED`]) and the built-ins that never add drive
//! ([`BUILTIN_PASS`]): the crossover the supervisor declares to the kernel
//! is then the graph's whole filtering, never a partial one.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::tuning::{Stage, Tuning};
use sgv2_core::prestage::{
    Band, BandMode, Channel, Dynamics, Params, Section, BANDS, FLAG_FROM_TUNING, FLAG_ORDERED, MAX_SECTIONS,
};
use sgv2_core::MAX_CHANNELS;
use std::f64::consts::PI;

/// The parametric filter unit's subtype.
pub const TMST: FourCc = FourCc::from_bytes(*b"tmst");
/// The multiband compressor unit's subtype.
pub const MSTK: FourCc = FourCc::from_bytes(*b"mstk");
/// The other boxes a speaker graph may run ahead of its protection unit,
/// which the pre-stage does not carry: the voicing graph that feeds the
/// unit stands in for them (loudness normaliser, virtual bass, crosstalk
/// canceller, variable EQ, peak limiter, buzz and delay, low-flow limiter,
/// volume taper, channel matrix). A box of any other kind ahead of the
/// unit refuses the compile: what it does to the drive is unknown.
pub const NOT_CARRIED: [&str; 10] = ["ldnm", "vben", "axtc", "vrq2", "cfrk", "buzk", "cdly", "flow", "volt", "pchr"];
/// The graph's own built-in boxes (manufacturer `abox`) the pre-stage passes:
/// routing and block-format boxes that do not change a channel's level
/// (channel split and join, reblocker, serial converter), non-finite samples
/// to zero, the dB gain that carries the system volume and balance (0 dB at
/// most at full volume, where the pre-stage runs), and the channel sum of a
/// downmixing graph, which the voicing graph stands in for like the boxes in
/// [`NOT_CARRIED`]. Any other built-in ahead of the unit refuses.
pub const BUILTIN_PASS: [&str; 7] = ["chsp", "chjn", "rebl", "srlc", "nfnp", "dbgn", "summ"];
const ABOX: FourCc = FourCc::from_bytes(*b"abox");
pub const SLOTS: u32 = 50;
const SLOT_BASE: u32 = 14;
const SLOT_STRIDE: u32 = 10;
/// Volume automation entries after the slots (514 and 515 are the range):
/// `target filter` then `state`, then the entry's set points.
const AUTOMATION_BASE: u32 = 516;
const AUTOMATION_STRIDE: u32 = 10;
const AUTOMATION_ENTRIES: u32 = 24;
/// The highest Butterworth order the unit offers.
pub const MAX_ORDER: u32 = 10;

fn bad(message: impl Into<String>) -> Error {
    Error::format("prestage", message)
}

/// One filter box's contribution to one of its channels.
#[derive(Debug, Default)]
struct Part {
    sections: Vec<Section>,
    gain_db: f64,
    muted: bool,
}

fn butterworth(high: bool, order: u32, f0: f64, fs: f64) -> Vec<Section> {
    let k = (PI * f0 / fs).tan();
    let mut v = Vec::new();
    for i in 0..order / 2 {
        let q2 = 2.0 * (PI * (2 * i + 1) as f64 / (2 * order) as f64).sin();
        let n = 1.0 + q2 * k + k * k;
        let (a1, a2) = (2.0 * (k * k - 1.0) / n, (1.0 - q2 * k + k * k) / n);
        let (b0, b1, b2) = if high { (1.0 / n, -2.0 / n, 1.0 / n) } else { (k * k / n, 2.0 * k * k / n, k * k / n) };
        v.push(Section { b0, b1, b2, a1, a2 });
    }
    if order % 2 == 1 {
        let n = 1.0 + k;
        let a1 = (k - 1.0) / n;
        let (b0, b1) = if high { (1.0 / n, -1.0 / n) } else { (k / n, k / n) };
        v.push(Section { b0, b1, b2: 0.0, a1, a2: 0.0 });
    }
    v
}

#[derive(Clone, Copy)]
enum Rbj {
    Peak,
    LowShelf,
    HighShelf,
}

fn rbj(kind: Rbj, gain_db: f64, f0: f64, q: f64, fs: f64) -> Section {
    let a = 10f64.powf(gain_db / 40.0);
    let w = 2.0 * PI * f0 / fs;
    let (c, s) = (w.cos(), w.sin());
    let al = s / (2.0 * q);
    let sa = 2.0 * a.sqrt() * al;
    let (b, d) = match kind {
        Rbj::Peak => ([1.0 + al * a, -2.0 * c, 1.0 - al * a], [1.0 + al / a, -2.0 * c, 1.0 - al / a]),
        Rbj::LowShelf => (
            [
                a * ((a + 1.0) - (a - 1.0) * c + sa),
                2.0 * a * ((a - 1.0) - (a + 1.0) * c),
                a * ((a + 1.0) - (a - 1.0) * c - sa),
            ],
            [(a + 1.0) + (a - 1.0) * c + sa, -2.0 * ((a - 1.0) + (a + 1.0) * c), (a + 1.0) + (a - 1.0) * c - sa],
        ),
        Rbj::HighShelf => (
            [
                a * ((a + 1.0) + (a - 1.0) * c + sa),
                -2.0 * a * ((a - 1.0) + (a + 1.0) * c),
                a * ((a + 1.0) + (a - 1.0) * c - sa),
            ],
            [(a + 1.0) - (a - 1.0) * c + sa, 2.0 * ((a - 1.0) - (a + 1.0) * c), (a + 1.0) - (a - 1.0) * c - sa],
        ),
    };
    Section { b0: b[0] / d[0], b1: b[1] / d[0], b2: b[2] / d[0], a1: d[1] / d[0], a2: d[2] / d[0] }
}

/// The filters a filter box (`aufx/tmst`) applies to its channel `ch`, as
/// the box runs them: sections, flat gain (dB) and mute, with no scaling
/// to unity (the pre-stage scales its whole lane; the voicing chain runs
/// each box as macOS does).
///
/// The box's volume automation is not applied: the filters are its saved
/// state. The last value is the automation entries that are switched on
/// (whatever slot they aim at), so the caller can treat the box as not yet
/// carried at other volumes.
pub fn tone(stage: &Stage, ch: u32, fs: f64, diags: &mut Diagnostics) -> Result<(Vec<Section>, f64, bool, Vec<u32>)> {
    let p = part_with(stage, ch, fs, diags, false)?;
    Ok((p.sections, p.gain_db, p.muted, automation(stage)))
}

/// The volume automation entries of a filter box that move something: on
/// (state not 0), aimed at a slot, with two distinct volume set points
/// (entry words 2 and 3). An entry whose set points coincide was measured
/// to change nothing at any volume (black-box renders of the unit), so it
/// is not counted.
pub fn automation(stage: &Stage) -> Vec<u32> {
    let Some(g) = stage.state.as_ref().and_then(|s| s.global()) else { return Vec::new() };
    let get = |id: u32| g.get(id).map(f64::from).unwrap_or(0.0);
    (0..AUTOMATION_ENTRIES)
        .filter(|e| {
            let entry = AUTOMATION_BASE + AUTOMATION_STRIDE * e;
            get(entry + 1) != 0.0 && get(entry) >= 0.0 && get(entry + 2) != get(entry + 3)
        })
        .collect()
}

/// The filters box `stage` applies to its channel `ch`.
fn part(stage: &Stage, ch: u32, fs: f64, diags: &mut Diagnostics) -> Result<Part> {
    part_with(stage, ch, fs, diags, true)
}

fn part_with(stage: &Stage, ch: u32, fs: f64, diags: &mut Diagnostics, strict_automation: bool) -> Result<Part> {
    let ctx = format!("filter box {}", stage.name);
    let mut p = Part::default();
    if stage.bypass {
        return Ok(p);
    }
    let g = stage
        .state
        .as_ref()
        .and_then(|s| s.global())
        .ok_or_else(|| bad(format!("{ctx}: the strip gives it no saved state")))?;
    // Every value this reads must be in the saved state: a field left out
    // would be read as "off" or 0 dB, which drops a cut, a filter or a mute.
    let req = |id: u32| -> Result<f64> {
        g.get(id).map(f64::from).ok_or_else(|| bad(format!("{ctx}: saved parameter {id} is missing")))
    };
    // automation entries past the ones the unit saves are off
    let get = |id: u32| g.get(id).map(f64::from).unwrap_or(0.0);
    let bits = |id: u32| -> Result<u32> {
        let v = req(id)?;
        if !(0.0..=65535.0).contains(&v) || v.fract() != 0.0 {
            return Err(bad(format!("{ctx}: parameter {id} is {v}, not a bit mask")));
        }
        Ok(v as u32)
    };
    if req(0)? != 0.0 {
        return Ok(p);
    }
    if bits(11)? >> ch & 1 != 0 {
        p.muted = true;
        return Ok(p);
    }
    let max_order = match g.get(5) {
        Some(v) if v >= 1.0 && v.fract() == 0.0 => (v as u32).min(MAX_ORDER),
        Some(v) => return Err(bad(format!("{ctx}: highest filter order {v}"))),
        None => MAX_ORDER,
    };
    p.gain_db = req(4)? + req(13)?;
    let group_bypass = bits(1)?;
    let nyquist = fs / 2.0;
    let mut kept = Vec::new();
    for i in 0..SLOTS {
        let at = |k: u32| req(SLOT_BASE + SLOT_STRIDE * i + k);
        let state = at(0)?;
        if !(0.0..=7.0).contains(&state) || state.fract() != 0.0 {
            return Err(bad(format!("{ctx}: slot {i} state {state}")));
        }
        if (state as u32) & 2 == 0 {
            continue;
        }
        let mask = at(7)?;
        if !(0.0..=65535.0).contains(&mask) || mask.fract() != 0.0 {
            return Err(bad(format!("{ctx}: slot {i} channel mask {mask}")));
        }
        if (mask as u32) >> ch & 1 == 0 {
            continue;
        }
        let group = at(9)?;
        if !(0.0..=9.0).contains(&group) || group.fract() != 0.0 {
            return Err(bad(format!("{ctx}: slot {i} group {group}")));
        }
        if group_bypass >> group as u32 & 1 != 0 {
            return Err(bad(format!("{ctx}: slot {i} runs in bypassed group {group}; group bypass is not settled")));
        }
        let (kind, gain, f0, q, order) = (at(1)?, at(2)?, at(3)?, at(4)?, at(5)?);
        if kind.fract() != 0.0 || !gain.is_finite() || !q.is_finite() || !order.is_finite() {
            return Err(bad(format!("{ctx}: slot {i} has a non-numeric field")));
        }
        let freq_ok = f0.is_finite() && f0 > 0.0 && f0 < nyquist;
        let need_freq = |what: &str| -> Result<()> {
            if freq_ok {
                Ok(())
            } else {
                Err(bad(format!("{ctx}: slot {i} ({what}) frequency {f0} Hz is outside 0 to {nyquist} Hz")))
            }
        };
        match kind as i64 {
            3 | 4 => {
                need_freq("Butterworth")?;
                if order.fract() != 0.0 || order < 1.0 || order as u32 > max_order {
                    return Err(bad(format!("{ctx}: slot {i} order {order} (1 to {max_order})")));
                }
                p.sections.extend(butterworth(kind as i64 == 4, order as u32, f0, fs));
                kept.push(i);
            }
            9 => {
                p.gain_db += gain;
                kept.push(i);
            }
            1 | 11 | 12 => {
                need_freq("peak or shelf")?;
                if q <= 0.0 || !(-30.0..=30.0).contains(&gain) {
                    return Err(bad(format!("{ctx}: slot {i} quality {q}, gain {gain} dB")));
                }
                let r = match kind as i64 {
                    1 => Rbj::Peak,
                    11 => Rbj::LowShelf,
                    _ => Rbj::HighShelf,
                };
                p.sections.push(rbj(r, gain, f0, q, fs));
                if !matches!(r, Rbj::Peak) {
                    kept.push(i);
                }
            }
            0 | 2 if gain >= 0.0 => {
                if gain > 0.0 {
                    diags.info(
                        "prestage-boost-left-out",
                        ctx.clone(),
                        format!("slot {i}: a shelf boost of {gain} dB is left out (it would only add drive)"),
                    );
                }
            }
            t => {
                return Err(bad(format!(
                    "{ctx}: slot {i} (type {t}, gain {gain} dB) is a filter the pre-stage cannot carry; \
                     without it the speaker would be driven harder than macOS drives it"
                )))
            }
        }
    }
    // Volume automation moves a slot with the system volume. One aimed at
    // a slot that sets the band limits is refused rather than frozen.
    for e in 0..AUTOMATION_ENTRIES {
        let entry = AUTOMATION_BASE + AUTOMATION_STRIDE * e;
        let (target, state) = (get(entry), get(entry + 1));
        if strict_automation && state != 0.0 && target >= 0.0 && kept.contains(&(target as u32)) {
            return Err(bad(format!("{ctx}: volume automation {e} moves slot {target}, which sets a band limit")));
        }
    }
    if p.sections.iter().any(|s| !s.stable()) {
        return Err(bad(format!("{ctx}: a filter of channel {ch} is unstable at {fs} Hz")));
    }
    Ok(p)
}

/// The multiband dynamics on the unit's lanes: the one compressor box every
/// lane passes before the unit, or none. Refuses a compressor on some lanes
/// only, two of them, or one the dynamics cannot carry.
pub fn dynamics(chain: &[Stage], lanes: &[crate::dspg::analysis::Lane]) -> Result<Option<Dynamics>> {
    let mut found: Option<&Stage> = None;
    for s in chain.iter().filter(|s| s.before_protection && s.unit.map(|u| u.subtype) == Some(MSTK)) {
        let on = lanes.iter().filter(|l| l.through.iter().any(|p| p.node == s.name)).count();
        if on == 0 {
            continue;
        }
        if on != lanes.len() {
            return Err(bad(format!("compressor {} is on {on} of the unit's {} lanes", s.name, lanes.len())));
        }
        if found.is_some() {
            return Err(bad("two compressor boxes ahead of the unit"));
        }
        found = Some(s);
    }
    let Some(st) = found else { return Ok(None) };
    compressor(st)
}

/// One multiband compressor box's dynamics, or none when it is bypassed;
/// refuses one the dynamics cannot carry.
pub fn compressor(st: &Stage) -> Result<Option<Dynamics>> {
    let ctx = format!("compressor box {}", st.name);
    if st.bypass {
        return Ok(None);
    }
    let g = st
        .state
        .as_ref()
        .and_then(|s| s.global())
        .ok_or_else(|| bad(format!("{ctx}: the strip gives it no saved state")))?;
    let req = |id: u32| -> Result<f64> {
        g.get(id)
            .map(f64::from)
            .filter(|v| v.is_finite())
            .ok_or_else(|| bad(format!("{ctx}: saved parameter {id} is missing")))
    };
    // optional: absent is the unit's off/0 default
    let opt = |id: u32| g.get(id).map(f64::from).unwrap_or(0.0);
    if req(0)? != 0.0 {
        return Ok(None);
    }
    for (id, what) in [(5, "crossover 0"), (7, "crossover 1"), (9, "crossover 2")] {
        if req(id)? != 4.0 {
            return Err(bad(format!("{ctx}: {what} order {} (only fourth order is carried)", req(id)?)));
        }
    }
    if req(63)? != 1.0 {
        return Err(bad(format!("{ctx}: release is switched off")));
    }
    for id in [65, 66, 67, 68] {
        if opt(id) != 0.0 {
            return Err(bad(format!("{ctx}: per-band input gain (parameter {id}) is not carried")));
        }
    }
    for id in [71, 73, 75, 77] {
        if opt(id) != 0.0 {
            return Err(bad(format!("{ctx}: saturation (parameter {id}) is not carried")));
        }
    }
    let mut band = [Band::default(); BANDS];
    for (b, out) in band.iter_mut().enumerate() {
        let o = 10 + 13 * b as u32;
        let (enable, bypass, solo, mute) = (req(o)?, req(o + 1)?, req(o + 2)?, req(o + 3)?);
        if solo != 0.0 {
            return Err(bad(format!("{ctx}: band {b} is soloed")));
        }
        if req(o + 8)? != 1.0 || req(o + 11)? != 1.0 {
            return Err(bad(format!("{ctx}: band {b} has an upward or expanding branch")));
        }
        let mode = if mute != 0.0 {
            BandMode::Mute
        } else if enable == 0.0 || bypass != 0.0 {
            BandMode::Pass
        } else {
            BandMode::Compress
        };
        *out = Band {
            threshold_db: req(o + 4)?,
            ratio: req(o + 5)?,
            range_db: req(o + 6)?,
            makeup_db: req(o + 12)?,
            mode,
        };
    }
    Ok(Some(Dynamics {
        crossover_hz: [req(4)?, req(6)?, req(8)?],
        input_gain_db: req(1)?,
        output_gain_db: req(2)?,
        release_s: req(62)? / 1000.0,
        band,
    }))
}

/// The pre-stage of every unit channel, from the filter boxes on its lane.
pub fn derive(t: &Tuning, fs: f64, channels: usize, diags: &mut Diagnostics) -> Result<Params> {
    derive_lanes(&t.chain, &t.protection.lanes, fs, channels, diags)
}

/// [`derive`] from the chain and the lanes into the unit.
fn derive_lanes(
    chain: &[Stage],
    lanes: &[crate::dspg::analysis::Lane],
    fs: f64,
    channels: usize,
    diags: &mut Diagnostics,
) -> Result<Params> {
    if channels == 0 || channels > MAX_CHANNELS {
        return Err(bad(format!("{channels} unit channels")));
    }
    if lanes.len() != channels {
        return Err(bad(format!(
            "the unit takes {channels} channels but the graph traces {} lanes into it",
            lanes.len()
        )));
    }
    let mut out = Params::identity(channels);
    out.dynamics = dynamics(chain, lanes)?;
    // the compressor runs where the graph has it: filters ahead of it on
    // the lane run before the dynamics, the rest after
    let comp = out.dynamics.as_ref().and_then(|_| {
        chain
            .iter()
            .find(|s| {
                s.before_protection
                    && s.unit.map(|u| u.subtype) == Some(MSTK)
                    && lanes.iter().all(|l| l.through.iter().any(|p| p.node == s.name))
            })
            .map(|s| s.name.clone())
    });
    out.flags = FLAG_FROM_TUNING | if comp.is_some() { FLAG_ORDERED } else { 0 };
    let mut skipped: Vec<String> = Vec::new();
    for (c, lane) in lanes.iter().enumerate() {
        let mut sections = Vec::new();
        let (mut gain_db, mut muted) = (0.0, false);
        let (mut pre, mut pre_gain_db) = (0usize, 0.0);
        for pass in &lane.through {
            let Some(stage) = chain.iter().find(|s| s.name == pass.node) else {
                return Err(bad(format!("lane {c} passes {}, which is not in the chain", pass.node)));
            };
            if !stage.before_protection {
                continue;
            }
            if comp.as_deref() == Some(stage.name.as_str()) {
                (pre, pre_gain_db) = (sections.len(), gain_db);
                gain_db = 0.0;
                continue;
            }
            let Some(sub) = box_subtype(stage) else { continue };
            if stage.unit.is_some_and(|u| u.manufacturer == ABOX) {
                if BUILTIN_PASS.contains(&sub.as_str()) {
                    continue;
                }
                return Err(bad(format!(
                    "built-in box {} ({sub}) ahead of the unit is of a kind the pre-stage does not know",
                    stage.name
                )));
            }
            if sub == "tmst" {
                if stage.unit.is_none() && !stage.bypass {
                    return Err(bad(format!(
                        "filter box {} has no preset in the strip; without its filters the speakers \
                         would be driven harder than macOS drives them",
                        stage.name
                    )));
                }
            } else if sub == "mstk" {
                // carried as the dynamics; a bypassed one passes
                continue;
            } else if NOT_CARRIED.contains(&sub.as_str()) {
                if !skipped.contains(&stage.name) {
                    skipped.push(stage.name.clone());
                }
                continue;
            } else {
                return Err(bad(format!(
                    "box {} ({}) ahead of the unit is of a kind the pre-stage does not know",
                    stage.name, stage.kind
                )));
            }
            if stage.unit.is_none() {
                continue;
            }
            let p = part(stage, pass.channel, fs, diags)?;
            sections.extend(p.sections);
            gain_db += p.gain_db;
            muted |= p.muted;
        }
        if sections.len() > MAX_SECTIONS {
            return Err(bad(format!("channel {c} needs {} filter sections; at most {MAX_SECTIONS}", sections.len())));
        }
        let mut ch = Channel::IDENTITY;
        ch.count = sections.len();
        ch.sections[..sections.len()].copy_from_slice(&sections);
        if comp.is_some() {
            ch.pre = pre;
            ch.pre_gain = if muted { 0.0 } else { 10f64.powf(pre_gain_db / 20.0) };
        } else {
            gain_db += pre_gain_db;
        }
        ch.gain = if muted { 0.0 } else { 10f64.powf(gain_db / 20.0) };
        if !ch.gain.is_finite() || !ch.pre_gain.is_finite() {
            return Err(bad(format!("channel {c}: gain {} dB", gain_db + pre_gain_db)));
        }
        // The filters never exceed unity anywhere (the gain after them
        // takes the excess).
        let peak = ch.peak(fs);
        if peak > 1.0 {
            ch.gain /= peak;
            diags.info(
                "prestage-scaled",
                format!("channel {c}"),
                format!("the graph's filters peak at {:+.2} dB; scaled to 0 dB", 20.0 * peak.log10()),
            );
        }
        out.channel[c] = ch;
    }
    if !skipped.is_empty() {
        diags.info(
            "prestage-not-carried",
            "pre-stage".to_string(),
            format!("boxes ahead of the unit left to the voicing graph: {}", skipped.join(", ")),
        );
    }
    Ok(out)
}

/// The four-letter subtype of an AudioUnit box (from the strip's unit, else
/// the graph's `aufx/xxxx/...` kind), or `None` for a built-in box.
fn box_subtype(stage: &Stage) -> Option<String> {
    if let Some(u) = stage.unit {
        return Some(u.subtype.to_string());
    }
    let mut it = stage.kind.split('/');
    match (it.next(), it.next()) {
        (Some(k), Some(sub)) if k.len() == 4 && sub.len() == 4 => Some(sub.to_string()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState, UnitId};

    /// A filter box with the given global values and slots (state, type,
    /// gain, frequency, quality, order, channel mask, group), over a
    /// complete saved state of zeros (every global and slot present, as the
    /// unit saves them).
    fn tmst(globals: &[(u32, f32)], slots: &[(u32, [f32; 8])]) -> Stage {
        let mut full: std::collections::BTreeMap<u32, f32> =
            (0..SLOT_BASE + SLOT_STRIDE * SLOTS).map(|id| (id, 0.0)).collect();
        full.insert(5, MAX_ORDER as f32);
        for (id, v) in globals {
            full.insert(*id, *v);
        }
        for (i, [state, kind, gain, f0, q, order, mask, group]) in slots {
            let b = SLOT_BASE + SLOT_STRIDE * i;
            for (id, v) in [(b, *state), (b + 1, *kind), (b + 2, *gain), (b + 3, *f0), (b + 4, *q), (b + 5, *order)] {
                full.insert(id, v);
            }
            full.insert(b + 7, *mask);
            full.insert(b + 9, *group);
        }
        let params: Vec<(u32, f32)> = full.into_iter().collect();
        Stage {
            name: "Tone".into(),
            kind: "aufx/tmst/appl".into(),
            unit: Some(UnitId {
                kind: FourCc::from_bytes(*b"aufx"),
                subtype: TMST,
                manufacturer: FourCc::from_bytes(*b"appl"),
            }),
            inputs: 1,
            outputs: 1,
            channels: Some(2),
            bypass: false,
            preset_name: None,
            state: Some(ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] }),
            preset_extra_keys: Vec::new(),
            preset_extra: Default::default(),
            before_protection: true,
        }
    }

    fn mstk(globals: &[(u32, f32)]) -> Stage {
        let mut st = tmst(&[], &[]);
        st.name = "Comp".into();
        st.unit = Some(UnitId {
            kind: FourCc::from_bytes(*b"aufx"),
            subtype: MSTK,
            manufacturer: FourCc::from_bytes(*b"appl"),
        });
        let mut params: Vec<(u32, f32)> = vec![
            (0, 0.0),
            (1, -12.0),
            (2, 14.0),
            (4, 200.0),
            (5, 4.0),
            (6, 1500.0),
            (7, 4.0),
            (8, 8000.0),
            (9, 4.0),
            (62, 50.0),
            (63, 1.0),
        ];
        for b in 0..4u32 {
            let o = 10 + 13 * b;
            params.extend([
                (o, 1.0),
                (o + 1, 0.0),
                (o + 2, 0.0),
                (o + 3, 0.0),
                (o + 4, -30.0),
                (o + 5, 3.0),
                (o + 6, -20.0),
            ]);
            params.extend([(o + 8, 1.0), (o + 11, 1.0), (o + 12, 0.0)]);
        }
        for (id, v) in globals {
            params.retain(|(i, _)| i != id);
            params.push((*id, *v));
        }
        st.state = Some(ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] });
        st
    }

    #[test]
    fn compressor_rules() {
        use crate::dspg::analysis::{Lane, Origin, Pass};
        let lane = |c| Lane {
            origin: Origin { node: "in".into(), port: 0, channel: c },
            hops: Vec::new(),
            through: vec![Pass { node: "Comp".into(), channel: c }],
        };
        let base = |st: Stage| -> Result<Option<Dynamics>> { dynamics(&[st], &[lane(0), lane(1)]) };
        let d = base(mstk(&[])).unwrap().unwrap();
        assert_eq!((d.crossover_hz, d.release_s, d.band[0].mode), ([200.0, 1500.0, 8000.0], 0.05, BandMode::Compress));
        assert!(base(mstk(&[(0, 1.0)])).unwrap().is_none());
        assert_eq!(base(mstk(&[(10 + 13 + 3, 1.0)])).unwrap().unwrap().band[1].mode, BandMode::Mute);
        assert_eq!(base(mstk(&[(10 + 1, 1.0)])).unwrap().unwrap().band[0].mode, BandMode::Pass);
        for refused in [(5, 6.0), (63, 0.0), (65, 3.0), (71, 1.0), (12, 1.0), (18, 2.0), (21, 2.0)] {
            assert!(base(mstk(&[refused])).is_err(), "{refused:?}");
        }
        // on some lanes only: refused
        let mut other = lane(1);
        other.through.clear();
        assert!(dynamics(&[mstk(&[])], &[lane(0), other]).is_err());
    }

    #[test]
    fn slot_rules() {
        let fs = 48_000.0;
        let mut d = Diagnostics::default();
        // a high-pass on channel 1 only, a flat cut on both, an inactive slot
        let st = tmst(
            &[(4, -1.0)],
            &[
                (0, [3.0, 4.0, 0.0, 200.0, 0.7, 4.0, 2.0, 0.0]),
                (1, [7.0, 9.0, -6.0, 1000.0, 1.0, 2.0, 3.0, 0.0]),
                (2, [1.0, 4.0, 0.0, 5000.0, 0.7, 8.0, 3.0, 0.0]),
            ],
        );
        let p0 = part(&st, 0, fs, &mut d).unwrap();
        assert_eq!((p0.sections.len(), p0.gain_db), (0, -7.0));
        let p1 = part(&st, 1, fs, &mut d).unwrap();
        assert_eq!((p1.sections.len(), p1.gain_db), (2, -7.0));
        // a band-shelf boost is left out; a cut of a type not carried is refused
        let boost = tmst(&[], &[(0, [3.0, 0.0, 2.0, 700.0, 800.0, 1800.0, 3.0, 0.0])]);
        assert_eq!(part(&boost, 0, fs, &mut d).unwrap().sections.len(), 0);
        let cut = tmst(&[], &[(0, [3.0, 0.0, -2.0, 700.0, 800.0, 1800.0, 3.0, 0.0])]);
        assert!(part(&cut, 0, fs, &mut d).is_err());
        let odd = tmst(&[], &[(0, [3.0, 13.0, 0.0, 700.0, 0.7, 2.0, 3.0, 0.0])]);
        assert!(part(&odd, 0, fs, &mut d).is_err());
        // a running slot in a bypassed group is refused; a muted channel is muted
        let grouped = tmst(&[(1, 2.0)], &[(0, [3.0, 4.0, 0.0, 200.0, 0.7, 2.0, 3.0, 1.0])]);
        assert!(part(&grouped, 0, fs, &mut d).is_err());
        let muted = tmst(&[(11, 1.0)], &[]);
        assert!(part(&muted, 0, fs, &mut d).unwrap().muted);
        assert!(!part(&muted, 1, fs, &mut d).unwrap().muted);
        // an order over the box's limit, or a frequency past Nyquist, is refused
        let deep = tmst(&[(5, 4.0)], &[(0, [3.0, 4.0, 0.0, 200.0, 0.7, 6.0, 3.0, 0.0])]);
        assert!(part(&deep, 0, fs, &mut d).is_err());
        let high = tmst(&[], &[(0, [3.0, 3.0, 0.0, 30_000.0, 0.7, 2.0, 3.0, 0.0])]);
        assert!(part(&high, 0, fs, &mut d).is_err());
        // a volume automation aimed at a band-limiting slot is refused
        let auto = tmst(
            &[(AUTOMATION_BASE, 0.0), (AUTOMATION_BASE + 1, 1.0)],
            &[(0, [3.0, 4.0, 0.0, 200.0, 0.7, 2.0, 3.0, 0.0])],
        );
        assert!(part(&auto, 0, fs, &mut d).is_err());
        // a bypassed box contributes nothing
        let off = tmst(&[(0, 1.0)], &[(0, [3.0, 4.0, 0.0, 200.0, 0.7, 2.0, 3.0, 0.0])]);
        assert_eq!(part(&off, 0, fs, &mut d).unwrap().sections.len(), 0);
        // a field the saved state leaves out refuses, never reads as 0: a
        // global gain, the mute bits, a running slot's gain or the state of
        // any slot
        let full = tmst(&[], &[(0, [3.0, 9.0, -6.0, 1000.0, 1.0, 2.0, 3.0, 0.0])]);
        for id in [4, 11, 13, 1, SLOT_BASE + 2, SLOT_BASE + SLOT_STRIDE * 7] {
            let mut st = full.clone();
            st.state.as_mut().unwrap().sections[0].params.retain(|(i, _)| *i != id);
            assert!(part(&st, 0, fs, &mut d).is_err(), "parameter {id} left out");
        }
        assert_eq!(part(&full, 0, fs, &mut d).unwrap().gain_db, -6.0);
    }

    /// The pre-stage of `chain` with every one of `n` lanes passing every box.
    fn derive_chain(chain: Vec<Stage>, n: usize, fs: f64, d: &mut Diagnostics) -> Result<Params> {
        use crate::dspg::analysis::{Lane, Origin, Pass};
        let lanes: Vec<Lane> = (0..n as u32)
            .map(|c| Lane {
                origin: Origin { node: "in".into(), port: 0, channel: c },
                hops: Vec::new(),
                through: chain.iter().map(|s| Pass { node: s.name.clone(), channel: c }).collect(),
            })
            .collect();
        derive_lanes(&chain, &lanes, fs, n, d)
    }

    #[test]
    fn compressor_runs_where_the_graph_has_it() {
        let fs = 48_000.0;
        let hp = |name: &str, f0: f32, gain: f32| {
            let mut st = tmst(&[(4, gain)], &[(0, [3.0, 4.0, 0.0, f0, 0.7, 2.0, 3.0, 0.0])]);
            st.name = name.into();
            st
        };
        let mut d = Diagnostics::default();
        let p =
            derive_chain(vec![hp("Tone1", 30.0, -2.0), mstk(&[]), hp("Tone3", 100.0, -3.0)], 2, fs, &mut d).unwrap();
        assert!(p.ordered() && p.from_tuning());
        let ch = p.channel[0];
        assert_eq!((ch.count, ch.pre), (2, 1));
        assert!((20.0 * ch.pre_gain.log10() + 2.0).abs() < 1e-9 && (20.0 * ch.gain.log10() + 3.0).abs() < 1e-9);
        // no compressor: everything before, nothing ordered
        let p = derive_chain(vec![hp("Tone1", 30.0, -2.0), hp("Tone3", 100.0, -3.0)], 2, fs, &mut d).unwrap();
        assert!(!p.ordered() && p.channel[0].pre == 0 && (20.0 * p.channel[0].gain.log10() + 5.0).abs() < 1e-9);
    }

    #[test]
    fn boxes_the_prestage_cannot_account_for_refuse() {
        let fs = 48_000.0;
        let mut d = Diagnostics::default();
        let hp = tmst(&[], &[(0, [3.0, 4.0, 0.0, 100.0, 0.7, 2.0, 3.0, 0.0])]);
        assert!(derive_chain(vec![hp.clone()], 2, fs, &mut d).is_ok());
        // a filter box the strip gives no preset: its filters would be lost
        let mut lost = hp.clone();
        (lost.unit, lost.state) = (None, None);
        assert!(derive_chain(vec![lost], 2, fs, &mut d).is_err());
        // a known voicing box is left to the voicing graph; an unknown one refuses
        let mut vb = hp.clone();
        (vb.name, vb.kind, vb.unit) = ("VirtBass".into(), "aufx/vben/appl".into(), None);
        assert!(derive_chain(vec![hp.clone(), vb], 2, fs, &mut d).is_ok());
        let mut odd = hp.clone();
        (odd.name, odd.kind, odd.unit) = ("Odd".into(), "aufx/zzzz/appl".into(), None);
        assert!(derive_chain(vec![hp, odd], 2, fs, &mut d).is_err());
    }

    #[test]
    fn butterworth_is_minus_3_db_at_its_corner_and_matches_the_order() {
        let fs = 48_000.0;
        for order in 1..=MAX_ORDER {
            for (high, f0) in [(true, 60.0), (true, 2500.0), (false, 1000.0)] {
                let mut ch = Channel::IDENTITY;
                let s = butterworth(high, order, f0, fs);
                ch.count = s.len();
                ch.sections[..s.len()].copy_from_slice(&s);
                let at = |f: f64| 20.0 * ch.magnitude(f, fs).log10();
                assert!((at(f0) + 3.0103).abs() < 1e-3, "order {order} f0 {f0}: {}", at(f0));
                assert!(ch.peak(fs) <= 1.0 + 1e-12);
                // an octave into the stop band: close to 6 dB per order
                let stop = if high { at(f0 / 2.0) } else { at((2.0 * f0).min(23_000.0)) };
                if !high && 2.0 * f0 >= 23_000.0 {
                    continue;
                }
                assert!(stop < -6.0 * order as f64 + 3.5, "order {order} f0 {f0} high {high}: {stop}");
            }
        }
    }

    #[test]
    fn rbj_shelves_and_peaks_reach_their_gain() {
        let fs = 48_000.0;
        let mag = |s: Section, f: f64| 20.0 * s.magnitude(f, fs).log10();
        assert!((mag(rbj(Rbj::Peak, -6.0, 500.0, 2.0, fs), 500.0) + 6.0).abs() < 1e-9);
        assert!((mag(rbj(Rbj::LowShelf, 12.0, 300.0, 0.707, fs), 5.0) - 12.0).abs() < 0.01);
        assert!(mag(rbj(Rbj::LowShelf, 12.0, 300.0, 0.707, fs), 20_000.0).abs() < 0.01);
        assert!((mag(rbj(Rbj::HighShelf, -4.0, 3000.0, 0.707, fs), 23_000.0) + 4.0).abs() < 0.05);
    }
}
