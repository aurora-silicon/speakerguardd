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
//! | 1 | peaking cut or boost | matched-Z, two-zero magnitude-corrected biquad |
//! | 3 | Butterworth low-pass of the slot's order, -3 dB at the frequency | bilinear, prewarped |
//! | 4 | Butterworth high-pass, likewise | bilinear, prewarped |
//! | 9 | flat gain | gain |
//! | 11 | low shelf | matched-Z, two-zero magnitude-corrected low shelf |
//! | 12 | high shelf | matched-Z, two-zero magnitude-corrected high shelf |
//! | 0 | a Butterworth band shelf; four frequencies set its two edges | two Butterworth shelves and the compensating flat gain |
//! | 2 | a Butterworth shelf; the first two frequencies set its direction, midpoint and slope | bilinear, prewarped Butterworth shelf sections |
//! | 7 | a four-section constant tilt, from `-gain` to `+gain` around the slot frequency | four bilinear first-order shelves and a flat `-gain` |
//! | 14 | resonant second-order high-pass; gain and order are ignored | bilinear high-pass with the saved analog-decay quality convention |
//! | 15 | a level-dependent notch | saved detector, range, ratio, threshold and attack/release law |
//! | 18 | a level-dependent high shelf | saved detector, range, ratio, threshold and attack/release law |
//!
//! A peak or shelf of 0 dB is left out (the identity). Every other type is
//! refused, as is a slot whose values are out of range.
//! Polarity flips are not carried: they change no level. A muted channel
//! gets gain 0.
//!
//! Group bypass (parameter 1, a bit per group) removes every slot in the
//! selected group. The voicing box follows that exactly. The protection
//! pre-stage keeps the older conservative envelope: it carries cuts and band
//! limits but omits boosts, so it never passes more than either state.
//!
//! Volume automation (`AUTOMATION_BASE`, ten entries) moves a slot's gain
//! and frequency with the system volume: below set point A the slot keeps
//! its own values, from A it glides to set point B's (through M's when M is
//! enabled), and from B up it holds B's. The order, quality and type stay.
//! The system volume is the unit's parameter 3 (0 to 1). This reading is
//! from the unit's measured response on test signals at volumes 0 to 1; a
//! set point's gain does not act on a Butterworth slot, as the slot's own
//! gain does not.
//! The pre-stage does not follow the volume; it carries, for every slot an
//! entry moves, the one setting that passes no more than any setting the
//! slot can take, at every frequency:
//!
//! * high-pass: its highest corner; low-pass: its lowest (a Butterworth
//!   response falls as its corner moves into the band), plus the deepest
//!   cut a set point gives (stricter than the unit, which ignores it);
//! * flat gain: its lowest;
//! * peak or shelf: the lowest setting when the frequency does not move;
//!   otherwise nothing when no setting cuts, the one cut exactly when no
//!   glide reaches it, and else a flat gain of the deepest cut (a peak or
//!   shelf stays between 0 dB and its gain).
//!
//! On a Mac whose tuning raises a high-pass and fades boosts as the volume
//! rises, that is the setting macOS runs at full volume. An entry aimed at
//! a running slot with a state other than 1 or 2, or with set points out of
//! order, refuses the compile.
//!
//! The graph's multiband compressor boxes (`aufx/mstk`) ahead of the unit
//! are carried at their places in the lanes ([`boxes`]): each compiled by
//! [`crate::voicing::mstk`] from its saved state, recorded with how many of
//! every channel's filter sections (and what gain) run before it, and run
//! by the pre-stage in that order, linked across the channels as the box
//! links its own. A box that only some of the unit's lanes pass, lanes that
//! reach boxes in different orders, or two lanes that share a box channel
//! but are filtered differently up to it refuse the compile.
//!
//! So every compressor's detector sees what macOS's sees (the sub-bass a
//! later crossover removes included).
//!
//! The channel's filters and gains together are then scaled so they never
//! exceed unity at any frequency (stricter than macOS only where the tuning
//! boosts). The compressor boxes carry their own gain staging, as macOS
//! applies it.
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
    Band, BandMode, BoxAt, Channel, DynamicAt, DynamicFilter, DynamicKind, Dynamics, Params, Section, BANDS,
    FLAG_FROM_TUNING, MAX_BOXES, MAX_DYNAMIC_FILTERS, MAX_SECTIONS,
};
use sgv2_core::voicing::chain::{Tone, ToneAutomation, TonePoint, ToneShape, ToneTarget};
use sgv2_core::voicing::tone as tone_filter;
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
/// Volume automation entries after the slots (514 and 515 are the range of
/// the system volume they follow): target filter, state, set point A, set
/// point B with its gain and frequency, set point M's enable, M, its gain
/// and frequency. Ten entries; the slots' attack and release follow (616
/// on).
const AUTOMATION_BASE: u32 = 516;
const AUTOMATION_STRIDE: u32 = 10;
const AUTOMATION_ENTRIES: u32 = 10;
/// The highest low/high-pass order the unit offers. The separate global
/// order field caps type-0/2 Butterworth shelves, not these crossovers.
pub const MAX_ORDER: u32 = 20;

fn automation_on(state: f64) -> bool {
    state != 0.0
}

fn bad(message: impl Into<String>) -> Error {
    Error::format("prestage", message)
}

/// One filter box's contribution to one of its channels.
#[derive(Debug, Default)]
struct Part {
    sections: Vec<Section>,
    dynamics: Vec<PartDynamic>,
    gain_db: f64,
    muted: bool,
    slots: Vec<SlotPart>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct SlotPart {
    index: u32,
    shape: ToneShape,
    gain_db: f64,
    frequency_hz: f64,
    quality: f64,
    fixed: [f64; 3],
    max_order: u32,
    section: usize,
}

#[derive(Clone, Copy, Debug)]
struct PartDynamic {
    slot: u32,
    split: usize,
    active: bool,
    sidechain: bool,
    params: DynamicFilter,
}

fn butterworth(high: bool, order: u32, f0: f64, fs: f64) -> Vec<Section> {
    let made = tone_filter::butterworth(high, order, f0, fs);
    made.sections[..made.count].to_vec()
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
/// A slot the box's volume automation moves is carried at the setting
/// that passes least over every volume (as the pre-stage carries it), so
/// the box never passes more than macOS at any volume. The last value is
/// the automation entries that are switched on, so the caller can report
/// the box as not yet exact at every volume.
pub fn tone(stage: &Stage, ch: u32, fs: f64, diags: &mut Diagnostics) -> Result<(Vec<Section>, f64, bool, Vec<u32>)> {
    let p = part_with(stage, ch, fs, diags, false)?;
    Ok((p.sections, p.gain_db, p.muted, automation(stage)))
}

/// A filter box for the whole-chain runner, including the slot gain/frequency
/// automation driven by the box's SystemVolume parameter. Entries with equal
/// set points do not move the shipped unit and are omitted.
pub(crate) fn voicing_tone(
    stage: &Stage,
    width: usize,
    volume_wired: bool,
    fs: f64,
    diags: &mut Diagnostics,
) -> Result<Tone> {
    let ctx = format!("filter box {}", stage.name);
    let mut parts = Vec::with_capacity(width);
    let mut channels = Vec::with_capacity(width);
    for c in 0..width {
        let p = part_with(stage, c as u32, fs, diags, true)?;
        if p.sections.len() > MAX_SECTIONS {
            return Err(bad(format!("{ctx} channel {c}: {} sections", p.sections.len())));
        }
        let mut ch = Channel::IDENTITY;
        ch.count = p.sections.len();
        ch.sections[..p.sections.len()].copy_from_slice(&p.sections);
        ch.gain = if p.muted { 0.0 } else { 10f64.powf(p.gain_db / 20.0) };
        if !ch.gain.is_finite() {
            return Err(bad(format!("{ctx} channel {c}: gain {} dB", p.gain_db)));
        }
        parts.push(p);
        channels.push(ch);
    }
    if !volume_wired || stage.bypass {
        return Ok(Tone { channels, automation: Vec::new(), dynamic: tone_dynamic(&parts, &ctx)? });
    }
    let g = stage
        .state
        .as_ref()
        .and_then(|s| s.global())
        .ok_or_else(|| bad(format!("{ctx}: the strip gives it no saved state")))?;
    let get = |id: u32| g.get(id).map(f64::from).unwrap_or(0.0);
    let mut out: Vec<ToneAutomation> = Vec::new();
    let mut automated_slots = Vec::new();
    for e in 0..AUTOMATION_ENTRIES {
        let entry = AUTOMATION_BASE + AUTOMATION_STRIDE * e;
        let state = get(entry + 1);
        let (a_scalar, b_scalar) = (get(entry + 2), get(entry + 3));
        if !automation_on(state) || a_scalar == b_scalar {
            continue;
        }
        let target = get(entry);
        if target.fract() != 0.0 || !(0.0..SLOTS as f64).contains(&target) {
            return Err(bad(format!("{ctx}: volume automation {e} target {target}")));
        }
        if !a_scalar.is_finite() || !b_scalar.is_finite() || a_scalar >= b_scalar {
            return Err(bad(format!("{ctx}: volume automation {e} set points {a_scalar}, {b_scalar}")));
        }
        let target = target as u32;
        if automated_slots.contains(&target) {
            return Err(bad(format!("{ctx}: two active volume automations target slot {target}")));
        }
        let mut saved: Option<SlotPart> = None;
        let mut targets = Vec::new();
        for (channel, p) in parts.iter().enumerate() {
            let Some(slot) = p.slots.iter().find(|s| s.index == target).copied() else { continue };
            if let Some(first) = saved {
                if (first.shape, first.gain_db, first.frequency_hz, first.quality, first.fixed, first.max_order)
                    != (slot.shape, slot.gain_db, slot.frequency_hz, slot.quality, slot.fixed, slot.max_order)
                {
                    return Err(bad(format!("{ctx}: slot {target} differs between channels")));
                }
            } else {
                saved = Some(slot);
            }
            targets.push(ToneTarget { channel, section: slot.section });
        }
        let Some(saved) = saved else {
            return Err(bad(format!("{ctx}: volume automation {e} targets inactive or uncarried slot {target}")));
        };
        let point = |scalar: f64, gain_db: f64, frequency_hz: f64, which: &str| -> Result<TonePoint> {
            let freq_ok = matches!(saved.shape, ToneShape::Gain)
                || frequency_hz.is_finite() && frequency_hz > 0.0 && frequency_hz < fs / 2.0;
            if !gain_db.is_finite() || !(-30.0..=30.0).contains(&gain_db) || !freq_ok {
                return Err(bad(format!(
                    "{ctx}: volume automation {e} {which}: gain {gain_db} dB, frequency {frequency_hz} Hz"
                )));
            }
            Ok(TonePoint { scalar, gain_db, frequency_hz: frequency_hz.max(1.0) })
        };
        let a = point(a_scalar, saved.gain_db, saved.frequency_hz, "A")?;
        let b = point(b_scalar, get(entry + 4), get(entry + 5), "B")?;
        let midpoint = match get(entry + 6) {
            0.0 => None,
            1.0 => {
                let scalar = get(entry + 7);
                if scalar <= a_scalar || scalar >= b_scalar {
                    return Err(bad(format!("{ctx}: volume automation {e} midpoint {scalar}")));
                }
                Some(point(scalar, get(entry + 8), get(entry + 9), "midpoint")?)
            }
            v => return Err(bad(format!("{ctx}: volume automation {e} midpoint enable {v}"))),
        };
        out.push(ToneAutomation {
            shape: saved.shape,
            quality: saved.quality,
            fixed: saved.fixed,
            max_order: saved.max_order,
            a,
            b,
            midpoint,
            targets,
        });
        automated_slots.push(target);
    }
    Ok(Tone { channels, automation: out, dynamic: tone_dynamic(&parts, &ctx)? })
}

/// Merge the per-channel records of a Tone box into linked runtime slots.
fn tone_dynamic(parts: &[Part], ctx: &str) -> Result<Vec<DynamicAt>> {
    let Some(first_part) = parts.first() else { return Ok(Vec::new()) };
    let order: Vec<u32> = first_part.dynamics.iter().map(|d| d.slot).collect();
    for (channel, part) in parts.iter().enumerate().skip(1) {
        let lane_order: Vec<u32> = part.dynamics.iter().map(|d| d.slot).collect();
        if lane_order != order {
            return Err(bad(format!("{ctx}: channel {channel} passes dynamic slots {lane_order:?}, not {order:?}")));
        }
    }
    if order.len() > MAX_DYNAMIC_FILTERS {
        return Err(bad(format!("{ctx}: {} dynamic slots; at most {MAX_DYNAMIC_FILTERS}", order.len())));
    }
    let mut out = Vec::with_capacity(order.len());
    for k in 0..order.len() {
        let first = first_part.dynamics[k];
        let mut at = DynamicAt { params: first.params, ..Default::default() };
        for (channel, part) in parts.iter().enumerate() {
            let d = part.dynamics[k];
            if d.params != first.params {
                return Err(bad(format!("{ctx}: dynamic slot {} differs between channels", first.slot)));
            }
            at.split[channel] = d.split;
            at.active_mask |= (d.active as u32) << channel;
            at.sidechain_mask |= (d.sidechain as u32) << channel;
        }
        if at.active_mask == 0 || at.sidechain_mask == 0 {
            return Err(bad(format!("{ctx}: dynamic slot {} has no output or sidechain channels", first.slot)));
        }
        out.push(at);
    }
    Ok(out)
}

/// The volume automation entries of a filter box that move something: on
/// (state nonzero, including 2 and the legacy value 1000), aimed at a slot, with two distinct volume set points
/// (entry words 2 and 3). An entry whose set points coincide was measured
/// to change nothing at any volume (black-box renders of the unit), so it
/// is not counted.
pub fn automation(stage: &Stage) -> Vec<u32> {
    let Some(g) = stage.state.as_ref().and_then(|s| s.global()) else { return Vec::new() };
    let get = |id: u32| g.get(id).map(f64::from).unwrap_or(0.0);
    (0..AUTOMATION_ENTRIES)
        .filter(|e| {
            let entry = AUTOMATION_BASE + AUTOMATION_STRIDE * e;
            automation_on(get(entry + 1)) && get(entry) >= 0.0 && get(entry + 2) != get(entry + 3)
        })
        .collect()
}

/// Bilinear-transform one real analog factor. Coefficients are in descending
/// powers (`c2 s^2 + c1 s + c0`); a first-order factor has `c2 == 0`.
fn bilinear_factor(num: [f64; 3], den: [f64; 3], k: f64) -> Section {
    let transform = |c: [f64; 3]| {
        if c[0] == 0.0 {
            let c1 = c[1] / k;
            return [c1 + c[2], -c1 + c[2], 0.0];
        }
        let c2 = c[0] / (k * k);
        let c1 = c[1] / k;
        [c2 + c1 + c[2], -2.0 * c2 + 2.0 * c[2], c2 - c1 + c[2]]
    };
    let b = transform(num);
    let a = transform(den);
    Section { b0: b[0] / a[0], b1: b[1] / a[0], b2: b[2] / a[0], a1: a[1] / a[0], a2: a[2] / a[0] }
}

/// One real factor of a high- or low-shelf cut. `a` is the pole/zero
/// radius ratio and `q2` is the coefficient of a complex Butterworth pair;
/// `None` selects the real first-order factor. A boost is the exact inverse.
fn shelf_factor(high: bool, boost: bool, a: f64, q2: Option<f64>, k: f64) -> Section {
    let (mut num, mut den) = match (high, q2) {
        (true, Some(q)) => ([a * a, q * a, 1.0], [1.0 / (a * a), q / a, 1.0]),
        (false, Some(q)) => ([1.0, q * a, a * a], [1.0, q / a, 1.0 / (a * a)]),
        (true, None) => ([0.0, a, 1.0], [0.0, 1.0 / a, 1.0]),
        (false, None) => ([0.0, 1.0, a], [0.0, 1.0, 1.0 / a]),
    };
    if boost {
        std::mem::swap(&mut num, &mut den);
    }
    bilinear_factor(num, den, k)
}

fn integer_shelf(high: bool, order: u32, gain_db: f64, fm: f64, fs: f64) -> Vec<Section> {
    if gain_db == 0.0 {
        return Vec::new();
    }
    let boost = gain_db > 0.0;
    let g = 10f64.powf(-gain_db.abs() / 20.0);
    let a = g.powf(1.0 / (2.0 * order as f64));
    let k = (PI * fm / fs).tan();
    let mut out = Vec::new();
    for i in 0..order / 2 {
        let q2 = 2.0 * (PI * (2 * i + 1) as f64 / (2 * order) as f64).sin();
        out.push(shelf_factor(high, boost, a, Some(q2), k));
    }
    if order % 2 != 0 {
        out.push(shelf_factor(high, boost, a, None, k));
    }
    out
}

/// ToneMeister's type-2 shelf. At integer order this is the exact
/// Butterworth shelf
///
/// `|H|^2 = (1 + g^2 r) / (1 + r)`, `r = x^(2N) / g`,
///
/// with a bilinear-prewarped `x` and midpoint `sqrt(f3*f4)`. The unit caps
/// the order at its global maximum. Between integer orders it rounds the
/// Butterworth factors gently; the real-factor construction below follows
/// that measured response to within the pre-stage's 0.05 dB rig tolerance.
fn butterworth_shelf(gain_db: f64, f3: f64, f4: f64, max_order: u32, fs: f64) -> Vec<Section> {
    let made = tone_filter::butterworth_shelf(gain_db, f3, f4, max_order, fs);
    made.sections[..made.count].to_vec()
}

/// ToneMeister's type-7 constant tilt. Four equal first-order shelves are
/// spaced symmetrically in octaves around the pivot; the small gain-squared
/// spreading is the unit's measured constant-slope law. Q and the remaining
/// slot fields do not participate.
fn constant_tilt(gain_db: f64, f0: f64, fs: f64) -> Vec<Section> {
    let g2 = gain_db * gain_db;
    let outer = 3.110 + 0.001_305 * g2;
    let inner = 1.214 + 0.000_345 * g2;
    let mut out = Vec::new();
    for octaves in [-outer, -inner, inner, outer] {
        out.extend(integer_shelf(true, 1, gain_db / 2.0, f0 * 2f64.powf(octaves), fs));
    }
    out
}

/// A slot's gain and frequency at one volume.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Point {
    gain: f64,
    f0: f64,
}

/// One volume automation entry: below set point A the slot keeps its own
/// values, from A it glides to B's (through M's when M is enabled), and at
/// B and above it holds B's.
#[derive(Clone, Copy, Debug)]
struct Automation {
    target: u32,
    state: f64,
    a: f64,
    b_at: f64,
    b: Point,
    middle: Option<(f64, Point)>,
    m_enable: f64,
}

impl Automation {
    /// The entry is aimed at a running slot of type `kind`: its values must
    /// be ones whose meaning is settled.
    fn check(&self, ctx: &str, e: u32, kind: i64) -> Result<()> {
        let fail = |why: &str| Err(bad(format!("{ctx}: volume automation {e} on slot {}: {why}", self.target)));
        if self.state != 1.0 && self.state != 2.0 {
            return fail(&format!("state {}", self.state));
        }
        if self.m_enable != 0.0 && self.m_enable != 1.0 {
            return fail(&format!("set point M enable {}", self.m_enable));
        }
        let mut at = vec![self.a];
        at.extend(self.middle.map(|(v, _)| v));
        at.push(self.b_at);
        if at.iter().any(|v| !v.is_finite() || !(-100.0..=1.0).contains(v)) || at.windows(2).any(|w| w[0] > w[1]) {
            return fail(&format!("set points {at:?} are not in order inside the volume range"));
        }
        let mut pts = vec![self.b];
        pts.extend(self.middle.map(|(_, p)| p));
        // a flat gain or a shelf boost never reads the frequency
        let reads_freq = matches!(kind, 1 | 3 | 4 | 11 | 12 | 14);
        if pts.iter().any(|p| !p.gain.is_finite() || (reads_freq && !p.f0.is_finite())) {
            return fail("a set point is not numeric");
        }
        Ok(())
    }
}

/// The box's volume automation entries that are switched on, by entry.
fn automation_entries(get: &dyn Fn(u32) -> f64, ctx: &str) -> Result<Vec<(u32, Automation)>> {
    let mut v = Vec::new();
    for e in 0..AUTOMATION_ENTRIES {
        let at = |k: u32| get(AUTOMATION_BASE + AUTOMATION_STRIDE * e + k);
        let (target, state) = (at(0), at(1));
        if state == 0.0 {
            continue;
        }
        if !(0.0..SLOTS as f64).contains(&target) || target.fract() != 0.0 {
            return Err(bad(format!("{ctx}: volume automation {e} is aimed at slot {target}")));
        }
        let m_enable = at(6);
        v.push((
            e,
            Automation {
                target: target as u32,
                state,
                a: at(2),
                b_at: at(3),
                b: Point { gain: at(4), f0: at(5) },
                middle: (m_enable != 0.0).then(|| (at(7), Point { gain: at(8), f0: at(9) })),
                m_enable,
            },
        ));
    }
    Ok(v)
}

/// How a peak or shelf whose gain and frequency move is carried.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Carry {
    /// The filter at this point: it passes no more than any other point, or
    /// any setting gliding between two of them, at every frequency.
    Exact(usize),
    /// A flat gain (dB): the deepest cut of any point. A peak or shelf stays
    /// between 0 dB and its gain, so no setting passes less.
    Flat(f64),
}

/// `points[0]` is the slot's own setting; `spans` are the glides between
/// points `(from, to, the glide covers some volumes)`.
fn carried(points: &[Point], spans: &[(usize, usize, bool)]) -> Carry {
    let lowest = (0..points.len()).min_by(|&x, &y| points[x].gain.total_cmp(&points[y].gain)).unwrap_or(0);
    if points.iter().all(|p| p.f0 == points[0].f0) {
        // same frequency and quality: the response falls with the gain
        return Carry::Exact(lowest);
    }
    let lo = points[lowest].gain;
    if lo >= 0.0 {
        return Carry::Flat(0.0);
    }
    let cuts: Vec<usize> = (0..points.len()).filter(|&k| points[k].gain < 0.0).collect();
    // A single cut that nothing glides into or out of: every other setting
    // is a boost, which passes at least 0 dB everywhere.
    if cuts.len() == 1 && spans.iter().all(|&(u, v, glides)| !glides || (u != cuts[0] && v != cuts[0])) {
        return Carry::Exact(cuts[0]);
    }
    Carry::Flat(lo)
}

/// The filters box `stage` applies to its channel `ch`.
fn part(stage: &Stage, ch: u32, fs: f64, diags: &mut Diagnostics) -> Result<Part> {
    part_with(stage, ch, fs, diags, false)
}

fn part_with(stage: &Stage, ch: u32, fs: f64, diags: &mut Diagnostics, exact_volume: bool) -> Result<Part> {
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
    let automation = automation_entries(&get, &ctx)?;
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
        let active = (mask as u32) >> ch & 1 != 0;
        let kind_value = at(1)?;
        if kind_value.fract() != 0.0 {
            return Err(bad(format!("{ctx}: slot {i} type {kind_value}")));
        }
        let dynamic_kind = match kind_value as i64 {
            15 => Some(DynamicKind::Notch),
            18 => Some(DynamicKind::HighShelf),
            _ => None,
        };
        if !active && dynamic_kind.is_none() {
            continue;
        }
        let group = at(9)?;
        if !(0.0..=9.0).contains(&group) || group.fract() != 0.0 {
            return Err(bad(format!("{ctx}: slot {i} group {group}")));
        }
        // The voicing box skips a bypassed group exactly. The protection
        // pre-stage carries only its cuts and band limits, never its boosts.
        let in_bypassed_group = group_bypass >> group as u32 & 1 != 0;
        if in_bypassed_group {
            diags.info(
                "prestage-group-bypass",
                ctx.clone(),
                format!("slot {i} is in bypassed group {group}: carried only as far as it cuts"),
            );
            // The box itself bypasses every slot in the selected group.
            // The pre-stage remains conservative for speaker-drive safety;
            // the voicing box must reproduce the bypass exactly.
            if exact_volume {
                continue;
            }
        }
        let (kind, gain, f0, q, order) = (kind_value, at(2)?, at(3)?, at(4)?, at(5)?);
        if kind.fract() != 0.0 || !gain.is_finite() || !q.is_finite() || !order.is_finite() {
            return Err(bad(format!("{ctx}: slot {i} has a non-numeric field")));
        }
        let kind = kind as i64;
        // The slot's own values, then every set point a volume automation
        // moves it to.
        let has_automation = automation.iter().any(|(_, a)| a.target == i);
        let mut points = vec![Point { gain, f0 }];
        let mut spans = Vec::new();
        for (e, a) in automation.iter().filter(|(_, a)| a.target == i && !exact_volume) {
            a.check(&ctx, *e, kind)?;
            let base = points.len();
            points.extend(a.middle.map(|(_, m)| m));
            points.push(a.b);
            // gliding spans: own values -> M -> B, or own values -> B
            let mut from = (0usize, a.a);
            for (k, v) in a.middle.map(|(v, _)| v).into_iter().chain([a.b_at]).enumerate() {
                spans.push((from.0, base + k, v > from.1));
                from = (base + k, v);
            }
        }
        let automated = points.len() > 1;
        let need_freq = |what: &str, f: f64| -> Result<()> {
            if f.is_finite() && f > 0.0 && f < nyquist {
                Ok(())
            } else {
                Err(bad(format!("{ctx}: slot {i} ({what}) frequency {f} Hz is outside 0 to {nyquist} Hz")))
            }
        };
        if let Some(dynamic_kind) = dynamic_kind {
            if has_automation {
                return Err(bad(format!("{ctx}: slot {i}: volume automation on a level-dependent filter")));
            }
            let threshold = at(6)?;
            let sidechain_mask = at(8)?;
            let attack = get(616 + 3 * i);
            let release = get(617 + 3 * i);
            let smooth = get(618 + 3 * i);
            if !(0.0..=65535.0).contains(&sidechain_mask) || sidechain_mask.fract() != 0.0 {
                return Err(bad(format!("{ctx}: slot {i} sidechain mask {sidechain_mask}")));
            }
            if !(-30.0..=0.0).contains(&gain)
                || !f0.is_finite()
                || q <= 0.0
                || !q.is_finite()
                || !(0.1..=10.0).contains(&order)
                || !(-180.0..=0.0).contains(&threshold)
                || !(0.0..=1_000.0).contains(&attack)
                || !(0.0..=5_000.0).contains(&release)
                || (smooth != 0.0 && smooth != 1.0)
            {
                return Err(bad(format!(
                    "{ctx}: slot {i} dynamic fields range {gain}, frequency {f0}, quality {q}, ratio {order}, \
                     threshold {threshold}, attack {attack}, release {release}, smooth {smooth}"
                )));
            }
            need_freq("level-dependent filter", f0)?;
            let sidechain_bits = sidechain_mask as u32;
            p.dynamics.push(PartDynamic {
                slot: i,
                split: p.sections.len(),
                active,
                sidechain: (if sidechain_bits == 0 { mask as u32 } else { sidechain_bits }) >> ch & 1 != 0,
                params: DynamicFilter {
                    kind: dynamic_kind,
                    range_db: gain,
                    frequency_hz: f0,
                    quality: q,
                    ratio: order,
                    threshold_db: threshold,
                    attack_ms: attack,
                    release_ms: release,
                    smooth: smooth != 0.0,
                },
            });
            diags.info(
                "prestage-dynamic-filter",
                ctx.clone(),
                format!(
                    "slot {i}: carried as a level-dependent {:?} with its saved detector and time law",
                    dynamic_kind
                ),
            );
            continue;
        }
        if !active {
            continue;
        }
        match kind {
            3 | 4 => {
                let high = kind == 4;
                if order.fract() != 0.0 || order < 1.0 || order as u32 > MAX_ORDER {
                    return Err(bad(format!("{ctx}: slot {i} order {order} (1 to {MAX_ORDER})")));
                }
                let section = p.sections.len();
                for pt in &points {
                    need_freq("Butterworth", pt.f0)?;
                }
                // The corner glides between its set points: the highest
                // high-pass corner (lowest low-pass corner) passes the least
                // at every frequency. The slot's own gain does not act on a
                // Butterworth slot; a set point's cut is carried.
                let corner = points.iter().map(|pt| pt.f0).fold(f0, |c, f| if high == (f > c) { f } else { c });
                let cut = points[1..].iter().map(|pt| pt.gain).fold(0.0, f64::min);
                p.sections.extend(butterworth(high, order as u32, corner, fs));
                p.gain_db += cut;
                if automated {
                    diags.info(
                        "prestage-volume-automation",
                        ctx.clone(),
                        format!(
                            "slot {i}: the volume-automated {} corner is carried at {corner} Hz (own {f0} Hz), cut {cut} dB",
                            if high { "high-pass" } else { "low-pass" }
                        ),
                    );
                }
                p.slots.push(SlotPart {
                    index: i,
                    shape: if high { ToneShape::HighPass(order as u32) } else { ToneShape::LowPass(order as u32) },
                    gain_db: gain,
                    frequency_hz: f0,
                    quality: q,
                    fixed: [0.0; 3],
                    max_order,
                    section,
                });
            }
            9 => {
                let mut g = points.iter().map(|pt| pt.gain).fold(f64::INFINITY, f64::min);
                if in_bypassed_group {
                    g = g.min(0.0);
                }
                p.gain_db += g;
                if automated {
                    diags.info(
                        "prestage-volume-automation",
                        ctx.clone(),
                        format!("slot {i}: the volume-automated gain is carried at its lowest, {g} dB"),
                    );
                }
                p.slots.push(SlotPart {
                    index: i,
                    shape: ToneShape::Gain,
                    gain_db: gain,
                    frequency_hz: f0.max(1.0),
                    quality: q,
                    fixed: [0.0; 3],
                    max_order,
                    section: p.sections.len(),
                });
            }
            // A resonant second-order high-pass. Its gain, order and fourth
            // frequency fields do not participate. The quality uses the
            // unit's analog decay convention rather than the bilinear Q.
            14 => {
                if q <= 0.0 {
                    return Err(bad(format!("{ctx}: slot {i} resonant high-pass quality {q}")));
                }
                for pt in &points {
                    need_freq("resonant high-pass", pt.f0)?;
                }
                let corner = points.iter().map(|pt| pt.f0).fold(f0, f64::max);
                let section = p.sections.len();
                p.sections.push(tone_filter::resonant_high_pass(corner, q, fs));
                p.slots.push(SlotPart {
                    index: i,
                    shape: ToneShape::ResonantHighPass,
                    gain_db: gain,
                    frequency_hz: f0,
                    quality: q,
                    fixed: [0.0; 3],
                    max_order,
                    section,
                });
                if automated {
                    diags.info(
                        "prestage-volume-automation",
                        ctx.clone(),
                        format!("slot {i}: the volume-automated resonant high-pass corner is carried at {corner} Hz"),
                    );
                }
            }
            // A peak or shelf of 0 dB at every point is the identity at
            // any frequency and quality.
            1 | 11 | 12 if points.iter().all(|pt| pt.gain == 0.0) && !(exact_volume && has_automation) => {}
            7 => {
                if automated {
                    return Err(bad(format!("{ctx}: slot {i}: volume automation on a constant tilt")));
                }
                need_freq("constant tilt", f0)?;
                if !(-30.0..=30.0).contains(&gain) {
                    return Err(bad(format!("{ctx}: slot {i} tilt gain {gain} dB")));
                }
                if !(in_bypassed_group && gain > 0.0) {
                    p.sections.extend(constant_tilt(gain, f0, fs));
                    p.gain_db -= gain;
                }
            }
            1 | 11 | 12 => {
                for pt in &points {
                    need_freq("peak or shelf", pt.f0)?;
                    if q <= 0.0 || !(-30.0..=30.0).contains(&pt.gain) {
                        return Err(bad(format!("{ctx}: slot {i} quality {q}, gain {} dB", pt.gain)));
                    }
                }
                let r = match kind {
                    1 => Rbj::Peak,
                    11 => Rbj::LowShelf,
                    _ => Rbj::HighShelf,
                };
                let section = p.sections.len();
                match carried(&points, &spans) {
                    Carry::Exact(k) if in_bypassed_group && points[k].gain > 0.0 => {}
                    Carry::Exact(k) => p.sections.push(match r {
                        Rbj::Peak => tone_filter::peak(points[k].gain, points[k].f0, q, fs),
                        Rbj::LowShelf => tone_filter::low_shelf(points[k].gain, points[k].f0, q, fs),
                        Rbj::HighShelf => tone_filter::high_shelf(points[k].gain, points[k].f0, q, fs),
                    }),
                    Carry::Flat(g) => p.gain_db += g,
                }
                if automated {
                    diags.info(
                        "prestage-volume-automation",
                        ctx.clone(),
                        match carried(&points, &spans) {
                            Carry::Exact(k) => format!(
                                "slot {i}: the volume-automated filter is carried at {} dB, {} Hz (it passes the least)",
                                points[k].gain, points[k].f0
                            ),
                            Carry::Flat(g) => format!("slot {i}: the volume-automated filter is carried as a flat {g} dB"),
                        },
                    );
                }
                p.slots.push(SlotPart {
                    index: i,
                    shape: match kind as i64 {
                        1 => ToneShape::Peak,
                        11 => ToneShape::LowShelf,
                        _ => ToneShape::HighShelf,
                    },
                    gain_db: gain,
                    frequency_hz: f0,
                    quality: q,
                    fixed: [0.0; 3],
                    max_order,
                    section,
                });
            }
            0 | 2 => {
                if exact_volume && has_automation {
                    if !(-30.0..=30.0).contains(&gain) {
                        return Err(bad(format!("{ctx}: slot {i} gain {gain} dB")));
                    }
                    let section = p.sections.len();
                    if kind == 2 {
                        need_freq("Butterworth shelf", f0)?;
                        need_freq("Butterworth shelf", q)?;
                        let made = tone_filter::butterworth_shelf(gain, f0, q, max_order, fs);
                        p.sections.extend_from_slice(&made.sections[..5]);
                        p.slots.push(SlotPart {
                            index: i,
                            shape: ToneShape::ButterworthShelf,
                            gain_db: gain,
                            frequency_hz: f0,
                            quality: 0.0,
                            fixed: [q, 0.0, 0.0],
                            max_order,
                            section,
                        });
                    } else {
                        let f5 = order;
                        let f6 = at(6)?;
                        for f in [f0, q, f5, f6] {
                            need_freq("Butterworth band shelf", f)?;
                        }
                        let first = tone_filter::butterworth_shelf(gain, q, f0, max_order, fs);
                        let second = tone_filter::butterworth_shelf(gain, f5, f6, max_order, fs);
                        let mut reserved = [Section::IDENTITY; 10];
                        reserved[..first.count].copy_from_slice(&first.sections[..first.count]);
                        reserved[5..5 + second.count].copy_from_slice(&second.sections[..second.count]);
                        p.sections.extend_from_slice(&reserved);
                        p.gain_db -= gain;
                        p.slots.push(SlotPart {
                            index: i,
                            shape: ToneShape::ButterworthBandShelf,
                            gain_db: gain,
                            frequency_hz: f0,
                            quality: 0.0,
                            fixed: [q, f5, f6],
                            max_order,
                            section,
                        });
                    }
                    continue;
                }
                if automated {
                    // Automation supplies only one frequency, not the full
                    // two-/four-frequency shape. Preserve the established
                    // safe envelope: the deepest automated cut is below
                    // every shelf setting at every frequency.
                    let lo = points.iter().map(|pt| pt.gain).fold(0.0, f64::min);
                    p.gain_db += lo;
                    diags.info(
                        "prestage-volume-automation",
                        ctx.clone(),
                        format!("slot {i}: the volume-automated shelf is carried as a flat {lo} dB"),
                    );
                    continue;
                }
                if !(-30.0..=30.0).contains(&gain) {
                    return Err(bad(format!("{ctx}: slot {i} gain {gain} dB")));
                }
                // A boost in a bypassed group is left out; a cut is still
                // carried so this path cannot drive harder than either
                // interpretation of the group bit.
                if in_bypassed_group && gain > 0.0 {
                    continue;
                }
                if kind == 2 {
                    need_freq("Butterworth shelf", f0)?;
                    need_freq("Butterworth shelf", q)?;
                    p.sections.extend(butterworth_shelf(gain, f0, q, max_order, fs));
                } else {
                    let f5 = order;
                    let f6 = at(6)?;
                    for f in [f0, q, f5, f6] {
                        need_freq("Butterworth band shelf", f)?;
                    }
                    // Type 0 is exactly a rising shelf over f3..f4 and a
                    // falling shelf over f5..f6, with a compensating flat
                    // gain so both outer asymptotes are 0 dB.
                    p.sections.extend(butterworth_shelf(gain, q, f0, max_order, fs));
                    p.sections.extend(butterworth_shelf(gain, f5, f6, max_order, fs));
                    p.gain_db -= gain;
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

/// One lane's walk: its sections, and per box passed (graph order) the box,
/// the box channel, how many sections run before it and the gain since the
/// previous box.
struct Walk {
    sections: Vec<Section>,
    boxes: Vec<(String, u32, usize, f64)>,
    dynamics: Vec<WalkDynamic>,
    tail_gain_db: f64,
    muted: bool,
}

#[derive(Clone, Debug)]
struct WalkDynamic {
    key: (String, u32),
    split: usize,
    before_box: usize,
    active: bool,
    sidechain: bool,
    params: DynamicFilter,
}

/// The pre-stage of every unit channel, from the filter and compressor boxes
/// on its lane.
pub fn derive(t: &Tuning, fs: f64, channels: usize, diags: &mut Diagnostics) -> Result<Params> {
    if t.protection.kind != crate::tuning::UnitKind::Atsp {
        interim_compressor_rules(&t.chain, &t.protection.lanes)?;
    }
    derive_lanes(&t.chain, &t.protection.lanes, fs, channels, diags)
}

/// Until the compressor box's gain law is the unit's own, the speakers whose
/// cones the pre-stage protects (spp3, lsm2) run only the compressor states
/// its interim law was verified on against macOS: one compressor box on the
/// lanes, fourth-order crossovers, release on, no per-band input gain, no
/// saturation, no solo, no upward or expanding branch. Any other state
/// refuses the compile (the interim law can drive harder than macOS there).
pub fn interim_compressor_rules(chain: &[Stage], lanes: &[crate::dspg::analysis::Lane]) -> Result<()> {
    let mut found = 0;
    for st in chain.iter().filter(|s| s.before_protection && !s.bypass && s.unit.map(|u| u.subtype) == Some(MSTK)) {
        if !lanes.iter().any(|l| l.through.iter().any(|p| p.node == st.name)) {
            continue;
        }
        let ctx = format!("compressor box {}", st.name);
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
        let opt = |id: u32| g.get(id).map(f64::from).unwrap_or(0.0);
        if req(0)? != 0.0 {
            continue;
        }
        found += 1;
        if found > 1 {
            return Err(bad("two compressor boxes ahead of the unit (not carried until the compressor law is exact)"));
        }
        for (id, what) in [(5, "crossover 0"), (7, "crossover 1"), (9, "crossover 2")] {
            if req(id)? != 4.0 {
                return Err(bad(format!(
                    "{ctx}: {what} order {} (only fourth order is carried until the compressor law is exact)",
                    req(id)?
                )));
            }
        }
        if req(63)? != 1.0 {
            return Err(bad(format!("{ctx}: release is switched off")));
        }
        for id in [65, 66, 67, 68] {
            if opt(id) != 0.0 {
                return Err(bad(format!(
                    "{ctx}: per-band input gain (parameter {id}) is not carried until the compressor law is exact"
                )));
            }
        }
        for id in [71, 73, 75, 77] {
            if opt(id) != 0.0 {
                return Err(bad(format!(
                    "{ctx}: saturation (parameter {id}) is not carried until the compressor law is exact"
                )));
            }
        }
        for b in 0..4u32 {
            let o = 10 + 13 * b;
            if req(o + 2)? != 0.0 {
                return Err(bad(format!("{ctx}: band {b} is soloed")));
            }
            if req(o + 8)? != 1.0 || req(o + 11)? != 1.0 {
                return Err(bad(format!("{ctx}: band {b} has an upward or expanding branch")));
            }
        }
    }
    Ok(())
}

/// [`derive`] from the graph's stages and the unit's lanes.
pub fn derive_lanes(
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
    let mut compiled: Vec<(String, Option<sgv2_core::voicing::mstk::Params>)> = Vec::new();
    let mut walks = Vec::new();
    let mut skipped: Vec<String> = Vec::new();
    for (c, lane) in lanes.iter().enumerate() {
        let mut w =
            Walk { sections: Vec::new(), boxes: Vec::new(), dynamics: Vec::new(), tail_gain_db: 0.0, muted: false };
        for pass in &lane.through {
            let Some(stage) = chain.iter().find(|s| s.name == pass.node) else {
                return Err(bad(format!("lane {c} passes {}, which is not in the chain", pass.node)));
            };
            if !stage.before_protection {
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
            if sub == "tmst" && stage.unit.is_none() && !stage.bypass {
                return Err(bad(format!(
                    "filter box {} has no preset in the strip; without its filters the speakers \
                     would be driven harder than macOS drives them",
                    stage.name
                )));
            }
            if NOT_CARRIED.contains(&sub.as_str()) {
                if !skipped.contains(&stage.name) {
                    skipped.push(stage.name.clone());
                }
                continue;
            }
            if sub != "tmst" && sub != "mstk" {
                return Err(bad(format!(
                    "box {} ({}) ahead of the unit is of a kind the pre-stage does not know",
                    stage.name, stage.kind
                )));
            }
            match stage.unit.map(|u| u.subtype) {
                Some(TMST) => {
                    let p = part(stage, pass.channel, fs, diags)?;
                    let base = w.sections.len();
                    for d in &p.dynamics {
                        w.dynamics.push(WalkDynamic {
                            key: (stage.name.clone(), d.slot),
                            split: base + d.split,
                            before_box: w.boxes.len(),
                            active: d.active,
                            sidechain: d.sidechain,
                            params: d.params,
                        });
                    }
                    w.sections.extend(p.sections);
                    w.tail_gain_db += p.gain_db;
                    w.muted |= p.muted;
                }
                Some(MSTK) => {
                    if !compiled.iter().any(|(n, _)| *n == stage.name) {
                        let p = crate::voicing::mstk::compile(stage, fs, diags)?;
                        compiled.push((stage.name.clone(), p));
                    }
                    let carried = compiled.iter().any(|(n, p)| *n == stage.name && p.is_some());
                    if carried {
                        w.boxes.push((stage.name.clone(), pass.channel, w.sections.len(), w.tail_gain_db));
                        w.tail_gain_db = 0.0;
                    }
                }
                _ => {}
            }
        }
        if w.sections.len() > MAX_SECTIONS {
            return Err(bad(format!("channel {c} needs {} filter sections; at most {MAX_SECTIONS}", w.sections.len())));
        }
        walks.push(w);
    }
    // every lane passes the same boxes in the same order
    let order: Vec<&str> = walks[0].boxes.iter().map(|b| b.0.as_str()).collect();
    for (c, w) in walks.iter().enumerate() {
        let o: Vec<&str> = w.boxes.iter().map(|b| b.0.as_str()).collect();
        if o != order {
            return Err(bad(format!(
                "lane {c} passes the compressor boxes {o:?} but lane 0 passes {order:?}; a box on some lanes only is not carried"
            )));
        }
    }
    if order.len() > MAX_BOXES {
        return Err(bad(format!("{} compressor boxes ahead of the unit; at most {MAX_BOXES}", order.len())));
    }
    let dynamic_order: Vec<&(String, u32)> = walks[0].dynamics.iter().map(|d| &d.key).collect();
    for (c, w) in walks.iter().enumerate() {
        let lane_order: Vec<&(String, u32)> = w.dynamics.iter().map(|d| &d.key).collect();
        if lane_order != dynamic_order {
            return Err(bad(format!(
                "lane {c} passes the dynamic filters {lane_order:?} but lane 0 passes {dynamic_order:?}"
            )));
        }
    }
    if dynamic_order.len() > MAX_DYNAMIC_FILTERS {
        return Err(bad(format!(
            "{} level-dependent filters ahead of the unit; at most {MAX_DYNAMIC_FILTERS}",
            dynamic_order.len()
        )));
    }
    // lanes that share a box channel carry the same signal into it (the
    // linked detection then equals the box's own)
    for (k, name) in order.iter().enumerate() {
        for i in 0..walks.len() {
            for j in i + 1..walks.len() {
                let (a, b) = (&walks[i], &walks[j]);
                if a.boxes[k].1 != b.boxes[k].1 {
                    continue;
                }
                let same_filters = a.sections[..a.boxes[k].2] == b.sections[..b.boxes[k].2];
                let same_gains = (0..=k).all(|q| a.boxes[q].3 == b.boxes[q].3);
                if !same_filters || !same_gains {
                    return Err(bad(format!(
                        "lanes {i} and {j} share channel {} of compressor box {name} but are filtered differently ahead of it",
                        a.boxes[k].1
                    )));
                }
            }
        }
    }
    let mut out = Params::identity(channels);
    out.flags = FLAG_FROM_TUNING;
    out.box_count = order.len();
    out.dynamic_count = dynamic_order.len();
    for k in 0..dynamic_order.len() {
        let first = &walks[0].dynamics[k];
        let mut d = DynamicAt { before_box: first.before_box, params: first.params, ..Default::default() };
        for (c, w) in walks.iter().enumerate() {
            let lane = &w.dynamics[k];
            if lane.before_box != first.before_box || lane.params != first.params {
                return Err(bad(format!("dynamic filter {:?} differs between lanes", first.key)));
            }
            d.split[c] = lane.split;
            d.active_mask |= (lane.active as u32) << c;
            d.sidechain_mask |= (lane.sidechain as u32) << c;
        }
        if d.active_mask == 0 || d.sidechain_mask == 0 {
            return Err(bad(format!("dynamic filter {:?} has no output or sidechain lanes", first.key)));
        }
        out.dynamic[k] = d;
    }
    for (k, name) in order.iter().enumerate() {
        let params = compiled.iter().find(|(n, _)| n == name).and_then(|(_, p)| *p).expect("compiled above");
        let mut b = BoxAt { params, ..Default::default() };
        for (c, w) in walks.iter().enumerate() {
            b.split[c] = w.boxes[k].2;
            b.gain[c] = 10f64.powf(w.boxes[k].3 / 20.0);
        }
        out.boxes[k] = b;
    }
    for (c, w) in walks.iter().enumerate() {
        let mut ch = Channel::IDENTITY;
        ch.count = w.sections.len();
        ch.sections[..w.sections.len()].copy_from_slice(&w.sections);
        ch.gain = if w.muted { 0.0 } else { 10f64.powf(w.tail_gain_db / 20.0) };
        if !ch.gain.is_finite() || out.boxes().iter().any(|b| !b.gain[c].is_finite()) {
            return Err(bad(format!("channel {c}: gain out of range")));
        }
        out.channel[c] = ch;
        // Filters and gains together never more than unity anywhere: the
        // pre-stage's own filtering only takes drive away (the compressor
        // boxes carry the graph's own gain staging).
        let peak = out.linear_peak(c, fs);
        if peak > 1.0 {
            out.channel[c].gain /= peak;
            diags.info(
                "prestage-scaled",
                format!("channel {c}"),
                format!("the graph's filters peak at {:+.2} dB; scaled to 0 dB", 20.0 * peak.log10()),
            );
        }
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
            (3, 1.0),
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
            params.extend([(o + 7, -40.0), (o + 8, 1.0), (o + 9, 10.0), (o + 10, -70.0), (o + 11, 1.0), (o + 12, 0.0)]);
        }
        for (id, v) in globals {
            params.retain(|(i, _)| i != id);
            params.push((*id, *v));
        }
        st.state = Some(ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] });
        st
    }

    struct G(Vec<Stage>, Vec<crate::dspg::analysis::Lane>);
    fn tuning(chain: Vec<Stage>, lanes: Vec<crate::dspg::analysis::Lane>) -> G {
        G(chain, lanes)
    }
    fn derive(g: &G, fs: f64, channels: usize, d: &mut Diagnostics) -> Result<Params> {
        derive_lanes(&g.0, &g.1, fs, channels, d)
    }

    fn lane(c: u32, through: &[(&str, u32)]) -> crate::dspg::analysis::Lane {
        use crate::dspg::analysis::{Lane, Origin, Pass};
        Lane {
            origin: Origin { node: "in".into(), port: 0, channel: c },
            hops: Vec::new(),
            through: through.iter().map(|(n, ch)| Pass { node: (*n).into(), channel: *ch }).collect(),
        }
    }

    #[test]
    fn compressor_boxes_run_at_their_places() {
        let fs = 48_000.0;
        let mut d = Diagnostics::default();
        let mut tone_a = tmst(&[(4, -2.0)], &[(0, [3.0, 4.0, 0.0, 100.0, 0.7, 2.0, 3.0, 0.0])]);
        tone_a.name = "ToneA".into();
        let mut tone_b = tmst(&[], &[(0, [3.0, 3.0, 0.0, 5000.0, 0.7, 2.0, 3.0, 0.0])]);
        tone_b.name = "ToneB".into();
        let mut up = mstk(&[(18, 2.0), (31, 2.0)]);
        up.name = "UpComp".into();
        let comp = mstk(&[(5, 3.0)]);
        let through = [("UpComp", 0), ("ToneA", 0), ("Comp", 0), ("ToneB", 0)];
        let t = tuning(vec![up, tone_a.clone(), comp.clone(), tone_b.clone()], vec![lane(0, &through)]);
        let p = derive(&t, fs, 1, &mut d).unwrap();
        assert_eq!(p.box_count, 2);
        assert_eq!((p.boxes[0].split[0], p.boxes[1].split[0], p.channel[0].count), (0, 1, 2));
        assert!((20.0 * p.boxes[1].gain[0].log10() + 2.0).abs() < 1e-9, "ToneA's -2 dB runs before Comp");
        assert_eq!(p.boxes[1].params.crossover_order[0], 3.0);
        // the blob carries it and the engine accepts it
        let back = Params::decode(&p.encode(), 1, fs).unwrap();
        assert_eq!(back.box_count, 2);
        // a box on some lanes only, or out of order, is refused
        let t2 = tuning(vec![comp.clone(), tone_b.clone()], vec![lane(0, &[("Comp", 0)]), lane(1, &[("ToneB", 0)])]);
        assert!(derive(&t2, fs, 2, &mut d).is_err());
        // lanes sharing a box channel but filtered differently ahead of it: refused
        let t3 = tuning(
            vec![tone_b.clone(), comp.clone()],
            vec![lane(0, &[("ToneB", 0), ("Comp", 0)]), lane(1, &[("Comp", 0)])],
        );
        assert!(derive(&t3, fs, 2, &mut d).is_err());
        // a bypassed compressor contributes nothing
        let off = mstk(&[(0, 1.0)]);
        let t4 = tuning(vec![off], vec![lane(0, &[("Comp", 0)])]);
        assert_eq!(derive(&t4, fs, 1, &mut d).unwrap().box_count, 0);
    }

    #[test]
    fn interim_compressor_rules_keep_the_verified_states() {
        let comp = mstk(&[]);
        let l = |c| lane(c, &[("Comp", c)]);
        assert!(interim_compressor_rules(std::slice::from_ref(&comp), &[l(0), l(1)]).is_ok());
        for refused in [(5, 3.0), (7, 6.0), (63, 0.0), (66, -6.0), (71, 1.0), (12, 1.0), (18, 2.0), (21, 2.0)] {
            let mut st = mstk(&[refused]);
            st.name = "Comp".into();
            assert!(interim_compressor_rules(&[st], &[l(0)]).is_err(), "{refused:?}");
        }
        let mut up = mstk(&[]);
        up.name = "UpComp".into();
        let two = [lane(0, &[("UpComp", 0), ("Comp", 0)])];
        assert!(interim_compressor_rules(&[up, comp.clone()], &two).is_err());
        // bypassed: no compressor
        let mut off = mstk(&[(0, 1.0), (5, 3.0)]);
        off.name = "Comp".into();
        assert!(interim_compressor_rules(&[off], &[l(0)]).is_ok());
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
        // Type-0 band shelves are carried as their two exact edges plus the
        // flat compensation (offset 6 is supplied separately by this terse
        // synthetic-slot helper).
        let boost = tmst(&[(20, 3000.0)], &[(0, [3.0, 0.0, 2.0, 700.0, 800.0, 1800.0, 3.0, 0.0])]);
        let pb = part(&boost, 0, fs, &mut d).unwrap();
        assert_eq!((pb.sections.len(), pb.gain_db), (3, -2.0));
        let cut = tmst(&[(20, 3000.0)], &[(0, [3.0, 0.0, -2.0, 700.0, 800.0, 1800.0, 3.0, 0.0])]);
        let pc = part(&cut, 0, fs, &mut d).unwrap();
        assert_eq!((pc.sections.len(), pc.gain_db), (3, 2.0));
        // a 0 dB peak at 0 Hz is the identity; a level-dependent cut is a
        // runtime dynamic slot rather than a fixed full-depth section
        let flat = tmst(&[], &[(0, [3.0, 1.0, 0.0, 0.0, 0.1, 2.0, 3.0, 0.0])]);
        assert_eq!(part(&flat, 0, fs, &mut d).unwrap().sections.len(), 0);
        let dynamic = tmst(&[], &[(0, [3.0, 15.0, -9.0, 200.0, 3.0, 3.0, 3.0, 0.0])]);
        let pd = part(&dynamic, 0, fs, &mut d).unwrap();
        assert_eq!((pd.sections.len(), pd.dynamics.len()), (0, 1));
        assert_eq!(pd.dynamics[0].params.kind, DynamicKind::Notch);
        let tone = voicing_tone(&dynamic, 2, false, fs, &mut d).unwrap();
        assert_eq!(tone.dynamic.len(), 1);
        assert_eq!((tone.dynamic[0].active_mask, tone.dynamic[0].sidechain_mask), (3, 3));
        let tilt = tmst(&[], &[(0, [3.0, 7.0, 12.0, 300.0, 9.0, 8.0, 3.0, 0.0])]);
        let pt = part(&tilt, 0, fs, &mut d).unwrap();
        assert_eq!((pt.sections.len(), pt.gain_db), (4, -12.0));
        assert!(pt.sections.iter().all(Section::stable));
        let high_cut = tmst(&[], &[(0, [3.0, 18.0, -4.0, 8000.0, 2.0, 3.0, 3.0, 0.0])]);
        let ph = part(&high_cut, 0, fs, &mut d).unwrap();
        assert_eq!((ph.sections.len(), ph.dynamics.len()), (0, 1));
        assert_eq!(ph.dynamics[0].params.kind, DynamicKind::HighShelf);
        let odd = tmst(&[], &[(0, [3.0, 13.0, 0.0, 700.0, 0.7, 2.0, 3.0, 0.0])]);
        assert!(part(&odd, 0, fs, &mut d).is_err());
        // a running slot in a bypassed group is carried only as a cut
        let grouped = tmst(
            &[(1, 2.0)],
            &[
                (0, [3.0, 4.0, 0.0, 200.0, 0.7, 2.0, 3.0, 1.0]),
                (1, [3.0, 1.0, 6.0, 2000.0, 1.0, 2.0, 3.0, 1.0]),
                (2, [3.0, 1.0, -6.0, 3000.0, 1.0, 2.0, 3.0, 1.0]),
                (3, [3.0, 9.0, 3.0, 1000.0, 1.0, 2.0, 3.0, 1.0]),
                (4, [3.0, 9.0, -3.0, 1000.0, 1.0, 2.0, 3.0, 1.0]),
            ],
        );
        let pg = part(&grouped, 0, fs, &mut d).unwrap();
        assert_eq!((pg.sections.len(), pg.gain_db), (2, -3.0));
        // a muted channel is muted
        let muted = tmst(&[(11, 1.0)], &[]);
        assert!(part(&muted, 0, fs, &mut d).unwrap().muted);
        assert!(!part(&muted, 1, fs, &mut d).unwrap().muted);
        // the global shelf-order field does not cap a low/high pass; an
        // order over the crossover limit, or a frequency past Nyquist, is refused
        let deep = tmst(&[(5, 4.0)], &[(0, [3.0, 4.0, 0.0, 200.0, 0.7, 21.0, 3.0, 0.0])]);
        assert!(part(&deep, 0, fs, &mut d).is_err());
        let high = tmst(&[], &[(0, [3.0, 3.0, 0.0, 30_000.0, 0.7, 2.0, 3.0, 0.0])]);
        assert!(part(&high, 0, fs, &mut d).is_err());
        // volume automation: see `volume_automation`; one with values whose
        // meaning is not settled is refused
        let auto = tmst(
            &[
                (AUTOMATION_BASE, 0.0),
                (AUTOMATION_BASE + 1, 1.0),
                (AUTOMATION_BASE + 2, 0.8),
                (AUTOMATION_BASE + 3, 0.2),
            ],
            &[(0, [3.0, 4.0, 0.0, 200.0, 0.7, 2.0, 3.0, 0.0])],
        );
        assert!(part(&auto, 0, fs, &mut d).is_err());
        let state = tmst(
            &[(AUTOMATION_BASE, 0.0), (AUTOMATION_BASE + 1, 5.0), (AUTOMATION_BASE + 3, 1.0)],
            &[(0, [3.0, 4.0, 0.0, 200.0, 0.7, 2.0, 3.0, 0.0])],
        );
        assert!(part(&state, 0, fs, &mut d).is_err());
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
        assert!(p.from_tuning());
        assert_eq!((p.box_count, p.boxes[0].split[0], p.channel[0].count), (1, 1, 2));
        assert!((20.0 * p.boxes[0].gain[0].log10() + 2.0).abs() < 1e-9);
        assert!((20.0 * p.channel[0].gain.log10() + 3.0).abs() < 1e-9);
        // no compressor: every filter and gain in one run
        let p = derive_chain(vec![hp("Tone1", 30.0, -2.0), hp("Tone3", 100.0, -3.0)], 2, fs, &mut d).unwrap();
        assert!(p.box_count == 0 && (20.0 * p.channel[0].gain.log10() + 5.0).abs() < 1e-9);
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

    /// One automation entry on `slot`: A, B and B's gain and frequency, and
    /// optionally M, its gain and frequency.
    fn auto(e: u32, slot: f32, a: f32, b: (f32, f32, f32), m: Option<(f32, f32, f32)>) -> Vec<(u32, f32)> {
        let o = AUTOMATION_BASE + AUTOMATION_STRIDE * e;
        let mut v = vec![(o, slot), (o + 1, 2.0), (o + 2, a), (o + 3, b.0), (o + 4, b.1), (o + 5, b.2)];
        if let Some((at, g, f)) = m {
            v.extend([(o + 6, 1.0), (o + 7, at), (o + 8, g), (o + 9, f)]);
        }
        v
    }

    #[test]
    fn volume_automation() {
        let fs = 48_000.0;
        let mut d = Diagnostics::default();
        let mag = |p: &Part, f: f64| {
            let mut ch = Channel::IDENTITY;
            ch.count = p.sections.len();
            ch.sections[..p.sections.len()].copy_from_slice(&p.sections);
            20.0 * ch.magnitude(f, fs).log10() + p.gain_db
        };
        // a high-pass whose corner rises with the volume: the highest corner
        // is carried, with the set points' cut
        let hp = |entries: Vec<(u32, f32)>| tmst(&entries, &[(1, [3.0, 4.0, 0.0, 45.0, 15.0, 4.0, 3.0, 0.0])]);
        let p = part(&hp(auto(0, 1.0, 0.2, (1.0, -0.1, 100.0), Some((0.8, -0.05, 80.0)))), 0, fs, &mut d).unwrap();
        assert_eq!(p.sections, butterworth(true, 4, 100.0f32 as f64, fs));
        assert!((p.gain_db + 0.1).abs() < 1e-6);
        // one falling with the volume: the slot's own corner is the highest
        let p = part(&hp(auto(0, 1.0, 0.2, (1.0, 0.0, 30.0), None)), 0, fs, &mut d).unwrap();
        assert_eq!(p.sections, butterworth(true, 4, 45.0, fs));
        // a low-pass: the lowest corner
        let lp = tmst(&auto(3, 0.0, 0.0, (1.0, 0.0, 300.0), None), &[(0, [3.0, 3.0, 0.0, 900.0, 0.7, 2.0, 3.0, 0.0])]);
        assert_eq!(part(&lp, 0, fs, &mut d).unwrap().sections, butterworth(false, 2, 300.0, fs));
        // a flat gain: its lowest
        let flat =
            tmst(&auto(0, 0.0, 0.1, (1.0, -4.0, 1000.0), None), &[(0, [3.0, 9.0, -1.0, 1000.0, 1.0, 2.0, 3.0, 0.0])]);
        assert!((part(&flat, 0, fs, &mut d).unwrap().gain_db + 4.0).abs() < 1e-6);
        // a peak whose boost fades to a small cut as the frequency moves: a
        // flat cut of that depth, never more than any setting passes
        let pk = |b: (f32, f32, f32)| {
            tmst(&auto(0, 2.0, 0.16, b, Some((0.6, 2.0, 9200.0))), &[(2, [3.0, 1.0, 3.3, 9300.0, 0.4, 2.0, 3.0, 0.0])])
        };
        let p = part(&pk((0.85, -0.5, 9190.0)), 0, fs, &mut d).unwrap();
        assert_eq!((p.sections.len(), p.gain_db), (0, -0.5f32 as f64));
        // that fades to 0 dB: nothing
        let p = part(&pk((0.85, 0.0, 9190.0)), 0, fs, &mut d).unwrap();
        assert_eq!((p.sections.len(), p.gain_db), (0, 0.0));
        // at one frequency: the lowest setting, exactly
        let same = tmst(
            &auto(0, 2.0, 0.16, (0.85, -1.0, 9300.0), Some((0.6, 2.0, 9300.0))),
            &[(2, [3.0, 1.0, 3.3, 9300.0, 0.4, 2.0, 3.0, 0.0])],
        );
        let p = part(&same, 0, fs, &mut d).unwrap();
        assert_eq!(p.sections.len(), 1);
        assert_eq!(p.sections[0], tone_filter::peak(-1.0, 9300.0, 0.4f32 as f64, fs));
        // a cut that nothing glides into (A = B): it is carried exactly
        let still =
            tmst(&auto(0, 0.0, 0.0, (0.0, 0.0, 20.0), None), &[(0, [3.0, 1.0, -2.0, 11200.0, 2.0, 2.0, 3.0, 0.0])]);
        let p = part(&still, 0, fs, &mut d).unwrap();
        assert_eq!((p.sections.len(), p.gain_db), (1, 0.0));
        // a band shelf whose boost fades to a cut: a flat cut
        let shelf = tmst(
            &auto(1, 0.0, 0.1, (1.0, -0.07, 193.0), Some((0.56, 4.3, 193.0))),
            &[(0, [7.0, 2.0, 8.4, 189.0, 300.0, 1000.0, 3.0, 0.0])],
        );
        let p = part(&shelf, 0, fs, &mut d).unwrap();
        assert_eq!((p.sections.len(), p.gain_db), (0, -0.07f32 as f64));
        // the carried response never passes more than any set point does
        for f in [20.0, 50.0, 100.0, 200.0, 1000.0, 9200.0] {
            let carried = mag(&part(&pk((0.85, -0.5, 9190.0)), 0, fs, &mut d).unwrap(), f);
            for (g, f0) in [(3.3, 9300.0), (2.0, 9200.0), (-0.5, 9190.0)] {
                let s = tone_filter::peak(g, f0, 0.4f32 as f64, fs);
                assert!(carried <= 20.0 * s.magnitude(f, fs).log10() + 1e-9, "{f} Hz vs {g} dB at {f0}");
            }
        }
        // an entry aimed at a slot that does not run is not read
        let idle = tmst(
            &[(AUTOMATION_BASE, 0.0), (AUTOMATION_BASE + 1, 1000.0)],
            &[(0, [1.0, 4.0, 0.0, 200.0, 0.7, 2.0, 3.0, 0.0])],
        );
        assert_eq!(part(&idle, 0, fs, &mut d).unwrap().sections.len(), 0);
        // the slots' attack and release after the ten entries are not entries
        let ramps = tmst(
            &[(AUTOMATION_BASE + AUTOMATION_STRIDE * AUTOMATION_ENTRIES + 1, 29.0)],
            &[(0, [3.0, 4.0, 0.0, 200.0, 0.7, 2.0, 3.0, 0.0])],
        );
        assert_eq!(part(&ramps, 0, fs, &mut d).unwrap().sections.len(), 1);
    }

    #[test]
    fn voicing_compiles_volume_automation_and_skips_equal_points() {
        let fs = 48_000.0;
        let b = AUTOMATION_BASE;
        let globals = [
            (b, 0.0),
            (b + 1, 1000.0), // legacy nonzero state is active, like state 2
            (b + 2, 0.2),
            (b + 3, 0.8),
            (b + 4, 0.0),
            (b + 5, 1600.0),
            (b + 6, 1.0),
            (b + 7, 0.5),
            (b + 8, 9.0),
            (b + 9, 200.0),
        ];
        let st = tmst(&globals, &[(0, [3.0, 11.0, 12.0, 100.0, core::f32::consts::FRAC_1_SQRT_2, 2.0, 3.0, 0.0])]);
        let mut d = Diagnostics::default();
        let tone = voicing_tone(&st, 2, true, fs, &mut d).unwrap();
        assert_eq!(tone.channels.len(), 2);
        assert_eq!(tone.automation.len(), 1);
        let a = &tone.automation[0];
        assert_eq!(a.shape, ToneShape::LowShelf);
        assert!((a.a.scalar - 0.2).abs() < 1e-6);
        assert_eq!((a.a.gain_db, a.a.frequency_hz), (12.0, 100.0));
        assert!((a.b.scalar - 0.8).abs() < 1e-6);
        assert_eq!((a.b.gain_db, a.b.frequency_hz), (0.0, 1600.0));
        assert!((a.midpoint.unwrap().scalar - 0.5).abs() < 1e-6);
        assert_eq!(a.targets, vec![ToneTarget { channel: 0, section: 0 }, ToneTarget { channel: 1, section: 0 }]);

        let mut equal = globals;
        equal[3].1 = equal[2].1;
        let st = tmst(&equal, &[(0, [3.0, 11.0, 12.0, 100.0, core::f32::consts::FRAC_1_SQRT_2, 2.0, 3.0, 0.0])]);
        assert!(voicing_tone(&st, 2, true, fs, &mut d).unwrap().automation.is_empty());
        assert!(voicing_tone(&st, 2, false, fs, &mut d).unwrap().automation.is_empty());

        let dynamic = tmst(
            &auto(0, 0.0, 0.2, (0.8, -12.0, 1000.0), None),
            &[(0, [3.0, 15.0, -6.0, 800.0, 2.0, 4.0, -20.0, 0.0])],
        );
        assert!(voicing_tone(&dynamic, 2, true, fs, &mut d).is_err());
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
                assert!(ch.peak(fs) <= 1.0 + 1e-8);
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

    #[test]
    fn butterworth_shelf_has_the_requested_asymptotes_and_midpoint() {
        let fs = 48_000.0;
        for gain in [-6.0, 6.0] {
            for (f3, f4, low, high) in [(8000.0, 2000.0, 0.0, gain), (2000.0, 8000.0, gain, 0.0)] {
                let sections = butterworth_shelf(gain, f3, f4, MAX_ORDER, fs);
                let mut ch = Channel::IDENTITY;
                ch.count = sections.len();
                ch.sections[..sections.len()].copy_from_slice(&sections);
                let db = |f| 20.0 * ch.magnitude(f, fs).log10();
                assert!((db(1.0) - low).abs() < 0.01, "gain {gain}, {f3}/{f4}: low {}", db(1.0));
                assert!((db(23_900.0) - high).abs() < 0.01, "gain {gain}, {f3}/{f4}: high {}", db(23_900.0));
                assert!((db(4000.0) - gain / 2.0).abs() < 0.01, "gain {gain}, {f3}/{f4}: mid {}", db(4000.0));
            }
        }
    }
}
