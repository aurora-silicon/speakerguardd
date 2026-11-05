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
//! The channel's whole response is then scaled so it never exceeds unity
//! at any frequency (stricter than macOS only where the tuning boosts): the
//! pre-stage never adds drive, and its output stays inside the converter's
//! range whenever its input does.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::tuning::{Stage, Tuning};
use sgv2_core::prestage::{Channel, Params, Section, FLAG_FROM_TUNING, MAX_SECTIONS};
use sgv2_core::MAX_CHANNELS;
use std::f64::consts::PI;

/// The parametric filter unit's subtype.
pub const TMST: FourCc = FourCc::from_bytes(*b"tmst");
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

/// Boards whose woofers the pre-stage alone still drives harder at low
/// frequency than macOS does (its multiband dynamics are not carried yet):
/// until they are, the pre-stage also caps every channel's cone excursion
/// at the tuning's own limit ([`excursion_cap`]). Policy, not a hardware
/// value: each board's numbers come from its tuning.
///
/// - `j413`: MacBook Air M2, measured 2026-11-06 (woofers about 1.2x
///   macOS's peak excursion at 125-400 Hz).
pub const EXCURSION_CAP_INTERIM: &[&str] = &["j413"];

/// Whether `board` gets the interim excursion cap.
pub fn excursion_cap_interim(board: &str) -> bool {
    EXCURSION_CAP_INTERIM.contains(&board)
}

/// The static excursion cap of one speaker from its tuning's lumped model
/// (spp3 sidecar `OL_disp`: moving mass, damping, stiffness, force factor,
/// amp volts at full scale, controller Q and excursion limit; `OL_thermal`
/// `Reb_ref`).
///
/// The cone's excursion per unit of program is `X0 / D(s)`, with
/// `D(s) = s^2/w0^2 + s/(w0 Q) + 1`, `w0 = sqrt(kt/mt)` and
/// `X0 = ag phi / (Reb_ref kt)` at full scale. The cap is
/// `C(s) = k D(s) / (s^2/wp^2 + 2 s/wp + 1)` with `k = x_lim / X0` and
/// `wp = w0 / sqrt(k)`: excursion becomes `k X0` over a critically damped
/// second-order low-pass, which never overshoots, so a program no larger
/// than full scale never drives the cone past `x_lim`. `C` is unity at high
/// frequency and never above it. `Q` is the larger of the controller Q and
/// the model's own (the more resonant, the more is taken away). A model
/// whose full-scale excursion is already within `x_lim` needs no cap.
pub fn excursion_cap(ch: &crate::spp3::Channel, fs: f64) -> Result<Option<Section>> {
    let d = &ch.ol_disp;
    let m = Cone {
        mt: d.mt,
        ct: d.ct,
        kt: d.kt,
        phi: d.phi,
        ag: d.ag,
        q_d: d.q_d,
        x_lim: d.x_lim,
        reb: ch.ol_thermal.reb_ref,
    };
    cap_section(&m, fs).map_err(|e| bad(format!("Channel{}: {e}", ch.index)))
}

/// A speaker's lumped model, as the tuning states it.
#[derive(Clone, Copy, Debug)]
pub struct Cone {
    pub mt: f64,
    pub ct: f64,
    pub kt: f64,
    pub phi: f64,
    pub ag: f64,
    pub q_d: f64,
    pub x_lim: f64,
    pub reb: f64,
}

impl Cone {
    pub fn w0(&self) -> f64 {
        (self.kt / self.mt).sqrt()
    }
    pub fn q(&self) -> f64 {
        self.q_d.max((self.kt * self.mt).sqrt() / (self.ct + self.phi * self.phi / self.reb))
    }
    /// Excursion at full scale below resonance, in the unit of `x_lim` (mm).
    pub fn x0(&self) -> f64 {
        self.ag * self.phi / (self.reb * self.kt) * 1e3
    }
}

fn cap_section(m: &Cone, fs: f64) -> std::result::Result<Option<Section>, String> {
    let all = [m.mt, m.ct, m.kt, m.phi, m.ag, m.q_d, m.x_lim, m.reb];
    if all.iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return Err("the excursion model (OL_disp, Reb_ref) is incomplete or not physical".into());
    }
    let (w0, q, x0) = (m.w0(), m.q(), m.x0());
    let k = m.x_lim / x0;
    if k >= 1.0 {
        return Ok(None);
    }
    let wp = w0 / k.sqrt();
    if !wp.is_finite() || wp >= PI * fs {
        return Err("the excursion cap's corner is past Nyquist".into());
    }
    // bilinear transform of (n2 s^2 + n1 s + n0) / (d2 s^2 + d1 s + d0)
    let (n2, n1, n0) = (k / (w0 * w0), k / (w0 * q), k);
    let (d2, d1, d0) = (1.0 / (wp * wp), 2.0 / wp, 1.0);
    let c = 2.0 * fs;
    let a0 = d2 * c * c + d1 * c + d0;
    let sec = Section {
        b0: (n2 * c * c + n1 * c + n0) / a0,
        b1: (2.0 * n0 - 2.0 * n2 * c * c) / a0,
        b2: (n2 * c * c - n1 * c + n0) / a0,
        a1: (2.0 * d0 - 2.0 * d2 * c * c) / a0,
        a2: (d2 * c * c - d1 * c + d0) / a0,
    };
    if !sec.stable() {
        return Err("the excursion cap is unstable".into());
    }
    Ok(Some(sec))
}

/// Adds the interim excursion cap of every channel to `pre` (see
/// [`EXCURSION_CAP_INTERIM`]); refuses when the tuning lacks a channel's
/// model.
pub fn add_excursion_cap(
    pre: &mut Params,
    sidecar: &crate::spp3::Sidecar,
    fs: f64,
    diags: &mut Diagnostics,
) -> Result<()> {
    for c in 0..pre.channels {
        let ch = sidecar
            .channels
            .iter()
            .find(|ch| ch.index == c)
            .ok_or_else(|| bad(format!("the sidecar has no Channel{c} for the excursion cap")))?;
        let Some(sec) = excursion_cap(ch, fs)? else { continue };
        let p = &mut pre.channel[c];
        if p.count >= MAX_SECTIONS {
            return Err(bad(format!("channel {c}: no room for the excursion cap")));
        }
        p.sections[p.count] = sec;
        p.count += 1;
        let peak = p.peak(fs);
        if peak > 1.0 {
            p.gain /= peak;
        }
        diags.info(
            "prestage-excursion-cap",
            format!("channel {c}"),
            "excursion capped at the tuning's limit (interim)",
        );
    }
    Ok(())
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

/// The filters box `stage` applies to its channel `ch`.
fn part(stage: &Stage, ch: u32, fs: f64, diags: &mut Diagnostics) -> Result<Part> {
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
    // A value the saved state leaves out is the unit's default; the
    // defaults that matter here are all "off" or 0 dB.
    let get = |id: u32| g.get(id).map(f64::from).unwrap_or(0.0);
    let bits = |id: u32| -> Result<u32> {
        let v = get(id);
        if !(0.0..=65535.0).contains(&v) || v.fract() != 0.0 {
            return Err(bad(format!("{ctx}: parameter {id} is {v}, not a bit mask")));
        }
        Ok(v as u32)
    };
    if get(0) != 0.0 {
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
    p.gain_db = get(4) + get(13);
    let group_bypass = bits(1)?;
    let nyquist = fs / 2.0;
    let mut kept = Vec::new();
    for i in 0..SLOTS {
        let at = |k: u32| get(SLOT_BASE + SLOT_STRIDE * i + k);
        let state = at(0);
        if !(0.0..=7.0).contains(&state) || state.fract() != 0.0 {
            return Err(bad(format!("{ctx}: slot {i} state {state}")));
        }
        if (state as u32) & 2 == 0 {
            continue;
        }
        let mask = at(7);
        if !(0.0..=65535.0).contains(&mask) || mask.fract() != 0.0 {
            return Err(bad(format!("{ctx}: slot {i} channel mask {mask}")));
        }
        if (mask as u32) >> ch & 1 == 0 {
            continue;
        }
        let group = at(9);
        if !(0.0..=9.0).contains(&group) || group.fract() != 0.0 {
            return Err(bad(format!("{ctx}: slot {i} group {group}")));
        }
        if group_bypass >> group as u32 & 1 != 0 {
            return Err(bad(format!("{ctx}: slot {i} runs in bypassed group {group}; group bypass is not settled")));
        }
        let (kind, gain, f0, q, order) = (at(1), at(2), at(3), at(4), at(5));
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
        if state != 0.0 && target >= 0.0 && kept.contains(&(target as u32)) {
            return Err(bad(format!("{ctx}: volume automation {e} moves slot {target}, which sets a band limit")));
        }
    }
    if p.sections.iter().any(|s| !s.stable()) {
        return Err(bad(format!("{ctx}: a filter of channel {ch} is unstable at {fs} Hz")));
    }
    Ok(p)
}

/// The pre-stage of every unit channel, from the filter boxes on its lane.
pub fn derive(t: &Tuning, fs: f64, channels: usize, diags: &mut Diagnostics) -> Result<Params> {
    if channels == 0 || channels > MAX_CHANNELS {
        return Err(bad(format!("{channels} unit channels")));
    }
    if t.protection.lanes.len() != channels {
        return Err(bad(format!(
            "the unit takes {channels} channels but the graph traces {} lanes into it",
            t.protection.lanes.len()
        )));
    }
    let mut out = Params::identity(channels);
    out.flags = FLAG_FROM_TUNING;
    for (c, lane) in t.protection.lanes.iter().enumerate() {
        let mut sections = Vec::new();
        let (mut gain_db, mut muted) = (0.0, false);
        for pass in &lane.through {
            let Some(stage) = t.chain.iter().find(|s| s.name == pass.node) else {
                return Err(bad(format!("lane {c} passes {}, which is not in the chain", pass.node)));
            };
            if !stage.before_protection || stage.unit.map(|u| u.subtype) != Some(TMST) {
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
        ch.gain = if muted { 0.0 } else { 10f64.powf(gain_db / 20.0) };
        if !ch.gain.is_finite() {
            return Err(bad(format!("channel {c}: gain {gain_db} dB")));
        }
        // Never more than unity anywhere: the pre-stage only takes drive away.
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
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState, UnitId};

    /// A filter box with the given global values and slots (state, type,
    /// gain, frequency, quality, order, channel mask, group).
    fn tmst(globals: &[(u32, f32)], slots: &[(u32, [f32; 8])]) -> Stage {
        let mut params: Vec<(u32, f32)> = globals.to_vec();
        for (i, [state, kind, gain, f0, q, order, mask, group]) in slots {
            let b = SLOT_BASE + SLOT_STRIDE * i;
            params.extend([(b, *state), (b + 1, *kind), (b + 2, *gain), (b + 3, *f0), (b + 4, *q), (b + 5, *order)]);
            params.extend([(b + 7, *mask), (b + 9, *group)]);
        }
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
            before_protection: true,
        }
    }

    #[test]
    fn excursion_cap_holds_the_limit() {
        let fs = 48_000.0;
        // a small woofer that a full-scale program drives past its limit
        let m = Cone { mt: 5e-4, ct: 0.6, kt: 4000.0, phi: 0.8, ag: 12.0, q_d: 1.5, x_lim: 0.4, reb: 4.0 };
        assert!(m.x0() > m.x_lim);
        let cap = cap_section(&m, fs).unwrap().unwrap();
        let mut ch = Channel::IDENTITY;
        ch.sections[0] = cap;
        ch.count = 1;
        assert!(ch.peak(fs) <= 1.0 + 1e-9);
        assert!((ch.magnitude(1.0, fs) - m.x_lim / m.x0()).abs() < 1e-3);
        assert!((ch.magnitude(23_000.0, fs) - 1.0).abs() < 0.02);
        // the cone model, as a digital filter (bilinear)
        let (w0, q, c) = (m.w0(), m.q(), 2.0 * fs);
        let (n2, n1) = (1.0 / (w0 * w0), 1.0 / (w0 * q));
        let a0 = n2 * c * c + n1 * c + 1.0;
        let cone = Section {
            b0: m.x0() / a0,
            b1: 2.0 * m.x0() / a0,
            b2: m.x0() / a0,
            a1: (2.0 - 2.0 * n2 * c * c) / a0,
            a2: (n2 * c * c - n1 * c + 1.0) / a0,
        };
        let run = |s: &Section, x: &[f64]| -> Vec<f64> {
            let (mut z1, mut z2) = (0.0, 0.0);
            x.iter()
                .map(|&v| {
                    let o = s.b0 * v + z1;
                    z1 = s.b1 * v - s.a1 * o + z2;
                    z2 = s.b2 * v - s.a2 * o;
                    o
                })
                .collect()
        };
        // full-scale square bursts at and below resonance: the worst crest
        for f in [50.0, 120.0, 250.0, 450.0, 900.0] {
            let x: Vec<f64> = (0..24_000)
                .map(|i| {
                    if i < 480 {
                        0.0
                    } else if (2.0 * PI * f * i as f64 / fs).sin() >= 0.0 {
                        1.0
                    } else {
                        -1.0
                    }
                })
                .collect();
            let uncapped = run(&cone, &x).iter().fold(0.0f64, |a, v| a.max(v.abs()));
            let capped = run(&cone, &run(&cap, &x)).iter().fold(0.0f64, |a, v| a.max(v.abs()));
            assert!(uncapped > m.x_lim, "{f} Hz: {uncapped}");
            assert!(capped <= m.x_lim * 1.001, "{f} Hz: {capped} mm against {}", m.x_lim);
        }
        // a model already inside its limit needs no cap; a broken one is refused
        assert!(cap_section(&Cone { x_lim: 10.0, ..m }, fs).unwrap().is_none());
        assert!(cap_section(&Cone { kt: 0.0, ..m }, fs).is_err());
        assert!(excursion_cap_interim("j413") && !excursion_cap_interim("j316s"));
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
