//! The voicing chain: the Mac's speaker graph from the stereo program to
//! the protection unit's lanes, box by box, in the graph's own order.
//!
//! The shape comes from the tuning (`aurora-speaker-tuning`'s
//! `voicing::topology` plan, compiled by `voicing::chain`): every box on
//! the unit's lanes, and for each of its channels, where the channel comes
//! from (a program channel or a channel of an earlier box). The program's
//! fan-out to the crossover lanes is a source named more than once.
//!
//! Box kinds the chain runs:
//!
//! * [`Kind::NonFinite`]: the graph's non-finite protection (a non-finite
//!   sample becomes silence);
//! * [`Kind::Sum`]: the graph's `sum` builtin: its channels are its inputs,
//!   input after input, and output channel `c` adds channel `c` of every
//!   input (later boxes name only its first `width` channels);
//! * [`Kind::Gain`]: the graph's gain builtin. Parameter 0 is the gain in
//!   dB, 1 the mute; the graph's volume and balance parameters drive them
//!   ([`Chain::set_param`]);
//! * [`Kind::Tone`]: a filter box, as second-order sections and a gain per
//!   channel ([`crate::prestage::Channel`]), plus its system-volume-driven
//!   slot gain/frequency automation;
//! * [`Kind::Comp`]: the multiband compressor, linked over the box's
//!   channels ([`crate::prestage::Dynamics`]);
//! * [`Kind::Stub`]: a box type whose module has not landed yet. It passes
//!   its input through and says so ([`Chain::stubs`]); the rig flags every
//!   result that depends on one. A box module replaces its stub by adding a
//!   variant here and its arm in `Run`.
//! * [`Kind::Bypass`]: a box the Mac's strip bypasses.
//!
//! Dropping a box module in (every module has the `Params`/`Run` shape of
//! [`super`]):
//!
//! 1. here: a `Kind::<Box>(<box>::Params)` variant, a `Run::<Box>(<box>::Run)`
//!    arm in `Run::new` (`<box>::Run::new(p, fs, width)`), in `Run::process`
//!    (copy `x` into `y`, then the box's `process` in place on `y`), in
//!    `Run::reset`, and, for a box the graph drives (VarEQ's `mvsc`, LowFlow's
//!    `mvol`), in `Run::set` by the parameter ID the graph wires;
//! 2. in `aurora-speaker-tuning`'s `voicing::chain::compile_node`: the
//!    subtype's arm calling the module's `compile` (`None` is a bypass).
//!
//! The stub disappears from [`Chain::stubs`] and the rig measures the box.
//!
//! `process` is real-time code: all storage is made in [`Chain::new`].

use crate::prestage::{
    Channel, DynamicAt, Dynamics, Params as PreParams, PreStage, Section, FLAG_DYNAMIC_FILTERS, MAX_DYNAMIC_FILTERS,
    MAX_SECTIONS,
};
use crate::MAX_CHANNELS;
use alloc::vec;
use alloc::vec::Vec;

/// Most boxes a chain may have; the shipped graphs have under 24 on the
/// unit's lanes.
pub const MAX_NODES: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Program(usize),
    /// Node index (earlier in the chain), channel.
    Node(usize, usize),
}

/// The graph's own parameters that reach the chain's boxes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GraphParam {
    /// The system volume as a scalar, 0 to 1 (`mvsc`).
    VolumeScalar,
    /// The system volume in dB (`mvol`).
    VolumeDb,
    /// Mute (`mmut`).
    Mute,
    /// Left and right balance, dB (`lbal`, `rbal`).
    BalanceLeft,
    BalanceRight,
    /// Lid state: closed (0) or open (1).
    Lid,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Link {
    pub param: GraphParam,
    pub node: usize,
    pub id: u32,
}

/// A box-published parameter copied to a later box at each control update.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Relay {
    pub from_node: usize,
    pub from_id: u32,
    pub to_node: usize,
    pub to_id: u32,
}

/// The part of one ToneMeister slot that volume automation may move.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToneShape {
    Peak,
    /// The fourth automation path of a box containing a type-18 slot. At
    /// its B endpoint this peak republishes frequency through the platform
    /// Float32 logarithm/exponential pair before construction.
    EndpointPublishedPeak,
    /// A two-point peak in a high-pass/low-shelf box publishes the upper
    /// frequency endpoint as the next Float32 word before construction.
    UpperEndpointPeak,
    /// A low shelf in the same automation object keeps its cancelling
    /// section when the moving gain reaches zero.
    AutomationLowShelf,
    /// A peak of the first ToneMeister algorithm version ([`super::tone::legacy_peak`]).
    LegacyPeak,
    LowPass(u32),
    HighPass(u32),
    Gain,
    LowShelf,
    HighShelf,
    ResonantHighPass,
    ButterworthShelf,
    ButterworthBandShelf,
}

/// One channel copy of an automated slot in a [`Tone`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ToneTarget {
    pub channel: usize,
    /// First section occupied by the slot (a gain slot's gain section).
    pub section: usize,
}

/// Gain and frequency at one volume set point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TonePoint {
    pub scalar: f64,
    pub gain_db: f64,
    pub frequency_hz: f64,
}

/// One slot's volume automation. The slot's saved gain/frequency are the A
/// point; B is explicit, and M optionally splits the interpolation in two.
/// Gain is linear in dB and frequency is logarithmic in Hz. Outside A..B,
/// the nearest endpoint is held.
#[derive(Clone, Debug, PartialEq)]
pub struct ToneAutomation {
    pub shape: ToneShape,
    pub quality: f64,
    /// The remaining saved edge frequencies of a type-2 shelf (its second)
    /// or a type-0 band shelf (its second to fourth); `a.frequency_hz` is
    /// the first. The automation moves the shelf as a whole, scaling every
    /// edge (`moved_shelf`).
    pub fixed: [f64; 3],
    pub max_order: u32,
    pub a: TonePoint,
    pub b: TonePoint,
    pub midpoint: Option<TonePoint>,
    pub targets: Vec<ToneTarget>,
}

/// Volume automation for one level-dependent Tone slot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicToneAutomation {
    pub index: usize,
    pub a: TonePoint,
    pub b: TonePoint,
    pub midpoint: Option<TonePoint>,
}

/// A filter box at its saved state plus the slots driven by system volume.
#[derive(Clone, Debug, PartialEq)]
pub struct Tone {
    pub channels: Vec<Channel>,
    pub automation: Vec<ToneAutomation>,
    /// Level-dependent slots in graph order. They have no compressor-box
    /// boundary inside a single Tone box, so every `before_box` is zero.
    pub dynamic: Vec<DynamicAt>,
    pub dynamic_automation: Vec<DynamicToneAutomation>,
}

impl Tone {
    /// The box as it runs at one fixed system-volume scalar: every volume
    /// automation entry applied once at `scalar`, none left to follow. A
    /// box whose SystemVolume input the graph leaves alone runs its
    /// automation at the saved value of that parameter, never at a volume
    /// the user sets.
    pub fn at_volume(&self, fs: f64, scalar: f64) -> Tone {
        let width = self.channels.len().min(MAX_CHANNELS);
        let count = self.dynamic.len().min(crate::prestage::MAX_DYNAMIC_FILTERS);
        let mut p = PreParams::identity(width);
        p.channel[..width].copy_from_slice(&self.channels[..width]);
        p.dynamic_count = count;
        p.dynamic[..count].copy_from_slice(&self.dynamic[..count]);
        let mut stage = PreStage::with_rate(p, fs);
        apply_tone_automation(&mut stage, &self.channels[..width], &self.automation, fs, scalar);
        apply_dynamic_tone_automation(&mut stage, &self.dynamic[..count], &self.dynamic_automation, fs, scalar);
        let params = stage.params();
        Tone {
            channels: params.channel[..width].to_vec(),
            automation: Vec::new(),
            dynamic: params.dynamic[..count].to_vec(),
            dynamic_automation: Vec::new(),
        }
    }
}

/// A precompiled group-bypass mask of one filter box.
#[derive(Clone, Debug, PartialEq)]
pub struct ToneVariant {
    pub mask: u32,
    pub tone: Tone,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Kind {
    NonFinite,
    /// The graph's `sum` builtin: output channels (each input's width).
    Sum(usize),
    Gain,
    /// Zero audio ports; input values sum into published parameter 0.
    ParameterAdd(Vec<f64>),
    /// Filter alternatives selected by group-bypass parameter 1.
    ToneGroups {
        default_mask: u32,
        variants: Vec<ToneVariant>,
    },
    /// One entry per box channel.
    Tone(Tone),
    Comp(Dynamics),
    /// Crosstalk cancellation (`axtc`).
    Xtc(alloc::boxed::Box<super::xtc::Params>),
    /// Linked group peak limiter (`cfrk`).
    Cfreak(alloc::boxed::Box<super::cfreak::Params>),
    /// Multiband compressor, full (`mstk` box module), for states the pre-stage dynamics refuse.
    Mstk(alloc::boxed::Box<super::mstk::Params>),
    /// Buzz (`buzk`), exact subset.
    Buzz(alloc::boxed::Box<super::buzz::Params>),
    /// Channel delay (`cdly`).
    Cdelay(alloc::boxed::Box<super::cdelay::Params>),
    /// AULowFlow (`flow`), exact subset.
    Lowflow(alloc::boxed::Box<super::lowflow::Params>),
    /// Virtual bass (`vben`).
    Vbass(alloc::boxed::Box<super::vbass::Params>),
    /// Volume-dependent EQ (`vrq2`); parameter 5 follows the system-volume
    /// scalar.
    Vareq(alloc::boxed::Box<super::vareq::Params>),
    /// Loudness normalizer (`ldnm`).
    Norm(alloc::boxed::Box<super::norm::Params>),
    /// Volume taper (`volt`), custom curve, with the unit's run-time ramp; its slider
    /// (parameter 4) follows the system-volume scalar.
    Volt(alloc::boxed::Box<super::volt::Params>),
    /// Channel matrix (`pchr`).
    Pchr(alloc::boxed::Box<super::pchr::Params>),
    /// The box's four-character subtype.
    Stub([u8; 4]),
    Bypass,
}

impl Kind {
    pub(crate) fn tones(&self) -> impl Iterator<Item = &Tone> {
        let tone = if let Self::Tone(tone) = self { Some(tone) } else { None };
        let variants = if let Self::ToneGroups { variants, .. } = self { variants.as_slice() } else { &[] };
        tone.into_iter().chain(variants.iter().map(|v| &v.tone))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct NodeSpec {
    pub kind: Kind,
    /// Per box channel, its source.
    pub inputs: Vec<Source>,
}

impl NodeSpec {
    /// The channels later boxes may name: a sum's output width, else the
    /// box's width.
    pub fn out_width(&self) -> usize {
        match self.kind {
            Kind::Sum(out) => out.min(self.inputs.len()),
            _ => self.inputs.len(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Spec {
    pub program_channels: usize,
    pub nodes: Vec<NodeSpec>,
    /// Per unit channel.
    pub outputs: Vec<Source>,
    pub links: Vec<Link>,
    pub relays: Vec<Relay>,
    /// The speaker volume control's scalar-to-dB law: how the plugin turns
    /// the user scalar (`mvsc`) into `mvol`.  The tuning compiler derives it
    /// from the Mac's own bundle; [`super::volume::Curve::Cubic`] when the
    /// bundle names no range.
    pub volume: super::volume::Curve,
}

#[derive(Debug, PartialEq, Eq)]
pub enum SpecError {
    TooMany,
    BadSource { node: usize, channel: usize },
    BadWidth { node: usize },
    BadLink(usize),
    BadParams { node: usize },
}

impl Spec {
    /// Every source points backwards and inside its node; widths fit.
    pub fn check(&self) -> Result<(), SpecError> {
        if self.nodes.len() > MAX_NODES
            || self.outputs.is_empty()
            || self.outputs.len() > MAX_CHANNELS
            || self.program_channels == 0
            || self.program_channels > MAX_CHANNELS
        {
            return Err(SpecError::TooMany);
        }
        let ok = |s: &Source, before: usize| match *s {
            Source::Program(c) => c < self.program_channels,
            Source::Node(n, c) => n < before && c < self.nodes[n].out_width(),
        };
        for (n, node) in self.nodes.iter().enumerate() {
            let w = node.inputs.len();
            if (w == 0 && !matches!(node.kind, Kind::ParameterAdd(_))) || w > MAX_CHANNELS {
                return Err(SpecError::BadWidth { node: n });
            }
            if let Kind::Sum(out) = node.kind {
                if out == 0 || w % out != 0 {
                    return Err(SpecError::BadWidth { node: n });
                }
            }
            match &node.kind {
                Kind::ParameterAdd(values)
                    if w != 0 || values.is_empty() || values.len() > 16 || values.iter().any(|v| !v.is_finite()) =>
                {
                    return Err(SpecError::BadParams { node: n })
                }
                Kind::ToneGroups { default_mask, variants }
                    if variants.is_empty()
                        || variants.len() > 16
                        || !variants.iter().any(|v| v.mask == *default_mask)
                        || variants.iter().any(|v| v.mask > 1023)
                        || variants.iter().enumerate().any(|(i, v)| variants[..i].iter().any(|u| u.mask == v.mask)) =>
                {
                    return Err(SpecError::BadParams { node: n })
                }
                _ => {}
            }
            for tone in node.kind.tones() {
                let all_channels = if w == 32 { u32::MAX } else { (1u32 << w) - 1 };
                if tone.channels.len() != w
                    || tone.channels.iter().any(|c| !c.sections().iter().all(|s| s.stable()) || !c.gain.is_finite())
                    || tone.automation.iter().any(|a| !automation_valid(a, &tone.channels))
                    || tone.dynamic.len() > MAX_DYNAMIC_FILTERS
                    || tone.dynamic.iter().any(|d| {
                        d.before_box != 0
                            || d.active_mask == 0
                            || d.active_mask & !all_channels != 0
                            || d.sidechain_mask == 0
                            || d.sidechain_mask & !all_channels != 0
                            || (0..w).any(|c| d.split[c] > tone.channels[c].count)
                    })
                    || tone.dynamic_automation.iter().any(|a| {
                        a.index >= tone.dynamic.len()
                            || !point_valid(a.a)
                            || !point_valid(a.b)
                            || a.a.scalar >= a.b.scalar
                            || a.midpoint
                                .is_some_and(|m| !point_valid(m) || m.scalar <= a.a.scalar || m.scalar >= a.b.scalar)
                    })
                {
                    return Err(SpecError::BadParams { node: n });
                }
            }
            if let Some(c) = node.inputs.iter().position(|s| !ok(s, n)) {
                return Err(SpecError::BadSource { node: n, channel: c });
            }
        }
        if let Some(c) = self.outputs.iter().position(|s| !ok(s, self.nodes.len())) {
            return Err(SpecError::BadSource { node: self.nodes.len(), channel: c });
        }
        if let Some(i) = self.links.iter().position(|l| l.node >= self.nodes.len()) {
            return Err(SpecError::BadLink(i));
        }
        if let Some(i) = self
            .relays
            .iter()
            .position(|r| r.from_node >= self.nodes.len() || r.to_node >= self.nodes.len() || r.from_node >= r.to_node)
        {
            return Err(SpecError::BadLink(self.links.len() + i));
        }
        Ok(())
    }
}

fn shape_sections(shape: ToneShape, max_order: u32) -> usize {
    match shape {
        ToneShape::LowPass(n) | ToneShape::HighPass(n) => (n as usize).div_ceil(2),
        ToneShape::Gain => 1,
        ToneShape::ButterworthShelf => super::tone::shelf_sections(max_order),
        ToneShape::ButterworthBandShelf => 2 * super::tone::shelf_sections(max_order),
        _ => 1,
    }
}

fn point_valid(p: TonePoint) -> bool {
    p.scalar.is_finite() && p.gain_db.is_finite() && p.frequency_hz.is_finite() && p.frequency_hz > 0.0
}

fn automation_valid(a: &ToneAutomation, channels: &[Channel]) -> bool {
    let order_ok = match a.shape {
        ToneShape::LowPass(n) | ToneShape::HighPass(n) => (1..=20).contains(&n),
        _ => true,
    };
    let quality_ok = matches!(
        a.shape,
        ToneShape::Gain
            | ToneShape::LowPass(_)
            | ToneShape::HighPass(_)
            | ToneShape::ButterworthShelf
            | ToneShape::ButterworthBandShelf
    ) || a.quality.is_finite() && a.quality > 0.0;
    let midpoint_ok =
        a.midpoint.map(|m| point_valid(m) && m.scalar > a.a.scalar && m.scalar < a.b.scalar).unwrap_or(true);
    order_ok
        && quality_ok
        && point_valid(a.a)
        && point_valid(a.b)
        && a.a.scalar < a.b.scalar
        && midpoint_ok
        && (1..=super::tone::MAX_SHELF_ORDER).contains(&a.max_order)
        && a.fixed.iter().all(|v| v.is_finite() && *v >= 0.0)
        && !a.targets.is_empty()
        && a.targets.iter().all(|t| {
            channels.get(t.channel).is_some_and(|c| {
                t.section <= MAX_SECTIONS && shape_sections(a.shape, a.max_order) <= c.count.saturating_sub(t.section)
            })
        })
}

fn interpolate(a: TonePoint, b: TonePoint, scalar: f64) -> (f64, f64) {
    let t = ((scalar - a.scalar) / (b.scalar - a.scalar)).clamp(0.0, 1.0);
    // At a set point the slot takes that point's own saved values exactly.
    if t == 0.0 {
        return (a.gain_db, a.frequency_hz);
    }
    if t == 1.0 {
        return (b.gain_db, b.frequency_hz);
    }
    let gain = a.gain_db + t * (b.gain_db - a.gain_db);
    let frequency = libm::exp(libm::log(a.frequency_hz) + t * (libm::log(b.frequency_hz) - libm::log(a.frequency_hz)));
    (gain, frequency)
}

fn automated_values(a: &ToneAutomation, scalar: f64) -> (f64, f64) {
    automated_point_values(a.a, a.b, a.midpoint, scalar)
}

/// The automation controller treats the last fifteen Float32 relative ULPs
/// as the B endpoint. This includes values reconstructed from a dB-domain
/// full-volume word that land just below the saved endpoint.
fn at_b_endpoint(a: &ToneAutomation, scalar: f64) -> bool {
    let t = (scalar - a.a.scalar) / (a.b.scalar - a.a.scalar);
    t >= 1.0 - 15.0 * f64::from(f32::EPSILON)
}

fn published_frequency(frequency_hz: f64) -> f64 {
    let frequency_hz = frequency_hz as f32;
    f64::from(crate::platform_math::expf(libm::log(f64::from(frequency_hz)) as f32))
}

fn upper_endpoint_frequency(frequency_hz: f64) -> f64 {
    let frequency_hz = frequency_hz as f32;
    f64::from(f32::from_bits(frequency_hz.to_bits() + 1))
}

/// The edge frequencies a Butterworth shelf (type 2: two; band shelf, type
/// 0: four) takes at `scalar`, with the gain. The installed unit moves the
/// shelf as a whole (black-box renders): the set points' frequency replaces
/// the shelf's reference, and every saved edge scales by the same ratio. The
/// reference is a type-2 shelf's first edge and a band shelf's geometric
/// centre of its two inner edges; it glides from there like any slot's
/// frequency (log in Hz, gain linear in dB).
fn moved_shelf(a: &ToneAutomation, scalar: f64) -> (f64, [f64; 4]) {
    let band = matches!(a.shape, ToneShape::ButterworthBandShelf);
    let edges = [a.a.frequency_hz, a.fixed[0], a.fixed[1], a.fixed[2]];
    let reference = if band { libm::sqrt(edges[1] * edges[2]) } else { edges[0] };
    let from = TonePoint { frequency_hz: reference, ..a.a };
    let (gain_db, frequency_hz) = automated_point_values(from, a.b, a.midpoint, scalar);
    let scale = frequency_hz / reference;
    (gain_db, edges.map(|f| f * scale))
}

fn automated_point_values(a: TonePoint, b: TonePoint, midpoint: Option<TonePoint>, scalar: f64) -> (f64, f64) {
    match midpoint {
        Some(m) if scalar <= m.scalar => interpolate(a, m, scalar),
        Some(m) => interpolate(m, b, scalar),
        None => interpolate(a, b, scalar),
    }
}

fn apply_tone_automation(stage: &mut PreStage, base: &[Channel], automation: &[ToneAutomation], fs: f64, scalar: f64) {
    for (c, channel) in base.iter().copied().enumerate() {
        stage.set_channel(c, channel);
    }
    for a in automation {
        let (gain_db, frequency_hz) = automated_values(a, scalar);
        for target in &a.targets {
            let Some(channel) = stage.channel_mut(target.channel) else { continue };
            match a.shape {
                ToneShape::Gain => {
                    channel.sections[target.section] = super::tone::gain_section(gain_db);
                }
                ToneShape::Peak
                | ToneShape::EndpointPublishedPeak
                | ToneShape::UpperEndpointPeak
                | ToneShape::LegacyPeak
                | ToneShape::LowShelf
                | ToneShape::AutomationLowShelf
                | ToneShape::HighShelf => {
                    let section = match a.shape {
                        ToneShape::Peak | ToneShape::EndpointPublishedPeak | ToneShape::UpperEndpointPeak => {
                            let frequency_hz = if at_b_endpoint(a, scalar) {
                                match a.shape {
                                    ToneShape::EndpointPublishedPeak => published_frequency(frequency_hz),
                                    ToneShape::UpperEndpointPeak => upper_endpoint_frequency(frequency_hz),
                                    _ => frequency_hz,
                                }
                            } else {
                                frequency_hz
                            };
                            super::tone::peak(gain_db, frequency_hz, a.quality, fs)
                        }
                        ToneShape::LegacyPeak => super::tone::legacy_peak(gain_db, frequency_hz, a.quality, fs),
                        ToneShape::LowShelf => super::tone::low_shelf(gain_db, frequency_hz, a.quality, fs),
                        ToneShape::AutomationLowShelf => {
                            super::tone::automated_low_shelf(gain_db, frequency_hz, a.quality, fs)
                        }
                        _ => super::tone::high_shelf(gain_db, frequency_hz, a.quality, fs),
                    };
                    if section.stable() {
                        channel.sections[target.section] = section;
                    }
                }
                ToneShape::ResonantHighPass => {
                    let section = super::tone::resonant_high_pass(frequency_hz, a.quality, fs);
                    if section.stable() {
                        channel.sections[target.section] = section;
                    }
                }
                ToneShape::ButterworthShelf => {
                    let (gain_db, [f3, f4, _, _]) = moved_shelf(a, scalar);
                    let made = super::tone::butterworth_shelf(gain_db, f3, f4, a.max_order, fs);
                    let n = super::tone::shelf_sections(a.max_order);
                    let sections = &mut channel.sections[target.section..target.section + n];
                    sections.fill(Section::IDENTITY);
                    sections[..made.count].copy_from_slice(&made.sections[..made.count]);
                }
                ToneShape::ButterworthBandShelf => {
                    let (gain_db, edges) = moved_shelf(a, scalar);
                    let (first, second) = super::tone::butterworth_band_shelf(gain_db, edges, a.max_order, fs);
                    let n = super::tone::shelf_sections(a.max_order);
                    let sections = &mut channel.sections[target.section..target.section + 2 * n];
                    sections.fill(Section::IDENTITY);
                    sections[..first.count].copy_from_slice(&first.sections[..first.count]);
                    sections[n..n + second.count].copy_from_slice(&second.sections[..second.count]);
                }
                ToneShape::LowPass(order) | ToneShape::HighPass(order) => {
                    let n = shape_sections(a.shape, a.max_order);
                    let sections = &mut channel.sections[target.section..target.section + n];
                    let made =
                        super::tone::butterworth(matches!(a.shape, ToneShape::HighPass(_)), order, frequency_hz, fs);
                    sections.copy_from_slice(&made.sections[..n]);
                    if sections.iter().any(|s| !s.stable()) {
                        sections.copy_from_slice(&base[target.channel].sections[target.section..target.section + n]);
                    }
                }
            }
        }
    }
}

fn apply_dynamic_tone_automation(
    stage: &mut PreStage,
    base: &[DynamicAt],
    automation: &[DynamicToneAutomation],
    fs: f64,
    scalar: f64,
) {
    for a in automation {
        let Some(at) = base.get(a.index) else { continue };
        let (range_db, frequency_hz) = automated_point_values(a.a, a.b, a.midpoint, scalar);
        let mut params = at.params;
        params.range_db = range_db;
        params.frequency_hz = frequency_hz;
        stage.set_dynamic_filter(a.index, params, fs);
    }
}

/// Frames per coefficient update during a tone automation glide.
pub const TONE_RAMP_STEP: usize = 16;

/// macOS's ToneMeister applies a new automation input (parameter 3) as a
/// per-frame glide: progress `clamp(n / len, 0, 1)^2` from the current value,
/// `len` = 40 ms. The glide begins `onset` frames after the change (two
/// 512-frame host slices in the speaker graph: macOS starts it at a later
/// slice boundary). A new value during a glide starts a new glide from where
/// the value is.
#[derive(Clone, Copy, Debug)]
struct ToneRamp {
    from: f64,
    to: f64,
    n: usize,
    len: usize,
    /// Frames left before the glide begins.
    wait: usize,
    onset: usize,
    started: bool,
}

impl ToneRamp {
    fn new(fs: f64) -> ToneRamp {
        let len = libm::round(0.040 * fs).max(1.0) as usize;
        let onset = libm::round(1024.0 * fs / 48_000.0) as usize;
        ToneRamp { from: 0.0, to: 0.0, n: len, len, wait: 0, onset, started: false }
    }
    fn value_at(&self, n: f64) -> f64 {
        let p = (n / self.len as f64).clamp(0.0, 1.0);
        self.from + (self.to - self.from) * p * p
    }
    /// A new target; true when it is to be applied at once (nothing played yet).
    fn set(&mut self, v: f64) -> bool {
        if !self.started {
            self.from = v;
            self.to = v;
            self.n = self.len;
            return true;
        }
        if v != self.to {
            self.from = self.value_at(self.n as f64);
            self.to = v;
            self.n = 0;
            self.wait = self.onset;
        }
        false
    }
    fn active(&self) -> bool {
        self.n < self.len
    }
    /// Frames (of `frames`) still to run before the glide begins.
    fn waiting(&mut self, frames: usize) -> usize {
        let w = self.wait.min(frames);
        self.wait -= w;
        w
    }
    /// The value for the next `frames` frames (at their middle), or None
    /// when the glide is over and the target is already applied.
    fn advance(&mut self, frames: usize) -> Option<f64> {
        if !self.active() {
            return None;
        }
        let mid = self.n as f64 + frames as f64 / 2.0;
        self.n += frames;
        Some(if self.n >= self.len { self.to } else { self.value_at(mid) })
    }
}

/// The filter box over frames `at..at + n` of every channel.
fn process_filters(stage: &mut PreStage, dynamic: bool, x: &[Vec<f32>], y: &mut [Vec<f32>], at: usize, n: usize) {
    if dynamic {
        for (i, o) in x.iter().zip(y.iter_mut()) {
            o[at..at + n].copy_from_slice(&i[at..at + n]);
        }
        let mut it = y.iter_mut();
        let mut refs: [&mut [f32]; MAX_CHANNELS] =
            core::array::from_fn(|_| it.next().map(|v| &mut v[at..at + n]).unwrap_or(&mut []));
        stage.process_block_float(&mut refs[..x.len().min(MAX_CHANNELS)], n);
    } else {
        for (c, (i, o)) in x.iter().zip(y.iter_mut()).enumerate() {
            stage.process_float(c, &i[at..at + n], &mut o[at..at + n]);
        }
    }
}

enum Run {
    ParameterAdd(Vec<f64>),
    ToneGroups {
        runs: Vec<(u32, Run)>,
        active: usize,
    },
    NonFinite,
    Sum(usize),
    Gain {
        gain_db: f64,
        mute: bool,
        /// The parameters' linear target (0 when muted).
        desired: f32,
        /// The ramp: the value on the next sample, its target, the per-sample
        /// increment and the samples left. A change during a ramp waits for it.
        current: f32,
        target: f32,
        inc: f32,
        left: usize,
        /// 20 ms at the box's rate.
        ramp: usize,
        started: bool,
    },
    Filters {
        stage: PreStage,
        base: Vec<Channel>,
        automation: Vec<ToneAutomation>,
        dynamic_base: Vec<DynamicAt>,
        dynamic_automation: Vec<DynamicToneAutomation>,
        fs: f64,
        dynamic: bool,
        /// The automation input (parameter 3) as macOS's unit applies it: a new value
        /// glides from the current one over 40 ms with progress (n / len)^2, the
        /// coefficients following the glided value. Coefficients are recomputed every
        /// [`TONE_RAMP_STEP`] frames of the glide (from the value at the step's middle).
        ramp: ToneRamp,
    },
    Comp(PreStage),
    Xtc(alloc::boxed::Box<super::xtc::Run>),
    Cfreak(alloc::boxed::Box<super::cfreak::Run>),
    Mstk(alloc::boxed::Box<super::mstk::Run>),
    Buzz(alloc::boxed::Box<super::buzz::Run>),
    Cdelay(alloc::boxed::Box<super::cdelay::Run>),
    Lowflow(alloc::boxed::Box<super::lowflow::Run>),
    Vbass(alloc::boxed::Box<super::vbass::Run>),
    Vareq(alloc::boxed::Box<super::vareq::Run>),
    Norm(alloc::boxed::Box<super::norm::Run>),
    Volt(alloc::boxed::Box<super::volt::Run>),
    Pchr(alloc::boxed::Box<super::pchr::Run>),
    Through,
}

impl Run {
    fn new(kind: &Kind, width: usize, fs: f64, max_frames: usize) -> Run {
        match kind {
            Kind::NonFinite => Run::NonFinite,
            Kind::Sum(out) => Run::Sum(*out),
            Kind::Gain => Run::Gain {
                gain_db: 0.0,
                mute: false,
                desired: 1.0,
                current: 1.0,
                target: 1.0,
                inc: 0.0,
                left: 0,
                ramp: libm::floor(0.020 * libm::floor(fs)) as usize,
                started: false,
            },
            Kind::ParameterAdd(values) => Run::ParameterAdd(values.clone()),
            Kind::ToneGroups { default_mask, variants } => Run::ToneGroups {
                active: variants.iter().position(|v| v.mask == *default_mask).unwrap_or(0),
                runs: variants
                    .iter()
                    .map(|v| (v.mask, Run::new(&Kind::Tone(v.tone.clone()), width, fs, max_frames)))
                    .collect(),
            },
            Kind::Tone(tone) => {
                let mut p = PreParams::identity(width);
                p.channel[..width].copy_from_slice(&tone.channels[..width]);
                p.dynamic_count = tone.dynamic.len();
                p.dynamic[..tone.dynamic.len()].copy_from_slice(&tone.dynamic);
                if p.dynamic_count != 0 {
                    p.flags |= FLAG_DYNAMIC_FILTERS;
                }
                Run::Filters {
                    stage: PreStage::with_rate(p, fs),
                    base: tone.channels.clone(),
                    automation: tone.automation.clone(),
                    dynamic_base: tone.dynamic.clone(),
                    dynamic_automation: tone.dynamic_automation.clone(),
                    fs,
                    dynamic: !tone.dynamic.is_empty(),
                    ramp: ToneRamp::new(fs),
                }
            }
            Kind::Comp(d) => {
                let mut p = PreParams::identity(width);
                p.dynamics = Some(*d);
                p.flags |= crate::prestage::FLAG_DYNAMICS;
                Run::Comp(PreStage::with_rate(p, fs))
            }
            Kind::Xtc(p) => Run::Xtc(alloc::boxed::Box::new(super::xtc::Run::new(p, fs, width))),
            Kind::Cfreak(p) => Run::Cfreak(alloc::boxed::Box::new(super::cfreak::Run::new(p, fs, width))),
            Kind::Mstk(p) => Run::Mstk(alloc::boxed::Box::new(super::mstk::Run::new(p, fs, width))),
            Kind::Buzz(p) => Run::Buzz(alloc::boxed::Box::new(super::buzz::Run::new(p, fs, width))),
            Kind::Cdelay(p) => Run::Cdelay(alloc::boxed::Box::new(super::cdelay::Run::new(p, fs, width))),
            Kind::Lowflow(p) => Run::Lowflow(alloc::boxed::Box::new(super::lowflow::Run::new(p, fs, width))),
            Kind::Vbass(p) => Run::Vbass(alloc::boxed::Box::new(super::vbass::Run::new(p, fs, width, max_frames))),
            Kind::Vareq(p) => Run::Vareq(alloc::boxed::Box::new(super::vareq::Run::new(p, fs, width))),
            Kind::Norm(p) => Run::Norm(alloc::boxed::Box::new(super::norm::Run::new(p, fs, width))),
            Kind::Volt(p) => Run::Volt(alloc::boxed::Box::new(super::volt::Run::new(p, fs, width))),
            Kind::Pchr(p) => Run::Pchr(alloc::boxed::Box::new(super::pchr::Run::new(p, fs, width))),
            Kind::Stub(_) | Kind::Bypass => Run::Through,
        }
    }

    fn set(&mut self, id: u32, v: f64) {
        match self {
            Run::ParameterAdd(values) if v.is_finite() => {
                if let Some(slot) = values.get_mut(id as usize) {
                    *slot = v;
                }
            }
            Run::ToneGroups { runs, active } => {
                if id == 1 && v.is_finite() && v >= 0.0 {
                    if let Some(index) = runs.iter().position(|(mask, _)| f64::from(*mask) == v) {
                        *active = index;
                    }
                } else {
                    for (_, run) in runs {
                        run.set(id, v);
                    }
                }
            }
            Run::Gain { gain_db, mute, desired, current, target, started, .. } => {
                match id {
                    // a gain box only ever attenuates: it never adds drive
                    0 if v.is_finite() => *gain_db = v.min(0.0),
                    1 => *mute = v != 0.0,
                    _ => {}
                }
                *desired = if *mute { 0.0 } else { libm::pow(10.0, *gain_db / 20.0) as f32 };
                // before the first block a value is installed, not ramped to
                if !*started {
                    *current = *desired;
                    *target = *desired;
                }
            }
            Run::Filters { stage, base, automation, dynamic_base, dynamic_automation, fs, ramp, .. }
                if id == 3 && v.is_finite() =>
            {
                // before the first block a value is installed, not glided to
                if ramp.set(v) {
                    apply_tone_automation(stage, base, automation, *fs, v);
                    apply_dynamic_tone_automation(stage, dynamic_base, dynamic_automation, *fs, v);
                }
            }
            // the volume taper's slider; a non-finite value is its quiet end
            // the volume taper's mute (graph mmut)
            Run::Volt(r) if id == 0 => r.set_mute(v != 0.0),
            Run::Volt(r) if id == 4 => {
                r.prime_volume(v);
            }
            // VarEQ's volume scalar; the box ignores a non-finite value
            Run::Vareq(r) if id == 5 => r.set_volume_scalar(v),
            // AULowFlow's attenuation ceiling follows the system volume in dB
            Run::Lowflow(r) if id == 4 => r.set_volume(v),
            Run::Mstk(r) if id == 79 => r.set_automation_input(v),
            _ => {}
        }
    }

    fn get(&self, id: u32) -> Option<f64> {
        match self {
            Run::ParameterAdd(values) if id == 0 => Some(values.iter().sum()),
            Run::Volt(r) if id == 100 => Some(r.gain_db()),
            _ => None,
        }
    }

    fn reset(&mut self) {
        match self {
            Run::ToneGroups { runs, .. } => {
                for (_, run) in runs {
                    run.reset();
                }
            }
            Run::Filters { stage, .. } | Run::Comp(stage) => stage.reset(),
            Run::Xtc(r) => r.reset(),
            Run::Vbass(r) => r.reset(),
            Run::Vareq(r) => r.reset(),
            Run::Norm(r) => r.reset(),
            Run::Cfreak(r) => r.reset(),
            Run::Mstk(r) => r.reset(),
            Run::Buzz(r) => r.reset(),
            Run::Cdelay(r) => r.reset(),
            Run::Lowflow(r) => r.reset(),
            Run::Volt(r) => r.reset(),
            Run::Pchr(r) => r.reset(),
            _ => {}
        }
    }

    /// `x` in, `y` out (both the box's width).
    fn process(&mut self, x: &[Vec<f32>], y: &mut [Vec<f32>], frames: usize) {
        match self {
            Run::ParameterAdd(_) => {}
            Run::ToneGroups { runs, active } => runs[*active].1.process(x, y, frames),
            // input 0 plus each later input, channel by channel
            Run::Sum(out) => {
                let out = *out;
                for (c, o) in y.iter_mut().enumerate() {
                    if c >= out {
                        o[..frames].fill(0.0);
                        continue;
                    }
                    o[..frames].copy_from_slice(&x[c][..frames]);
                    for input in x.iter().skip(out + c).step_by(out) {
                        for (b, a) in o[..frames].iter_mut().zip(input[..frames].iter()) {
                            *b += *a;
                        }
                    }
                }
            }
            Run::NonFinite => {
                for (i, o) in x.iter().zip(y.iter_mut()) {
                    for (a, b) in i[..frames].iter().zip(o[..frames].iter_mut()) {
                        *b = if a.is_finite() { *a } else { 0.0 };
                    }
                }
            }
            // The graph's gain builtin ramps every change linearly in amplitude over
            // 20 ms: the current value on the first sample, then a constant step, held
            // at the target. A change during a ramp starts its own ramp after it.
            Run::Gain { desired, current, target, inc, left, ramp, started, .. } => {
                *started = true;
                if *left == 0 && *desired != *target {
                    *target = *desired;
                    *left = (*ramp).max(1);
                    *inc = (*target - *current) / *left as f32;
                }
                let (c0, inc0, left0, tgt) = (*current, *inc, *left, *target);
                for (i, o) in x.iter().zip(y.iter_mut()) {
                    let (mut g, mut l) = (c0, left0);
                    for (a, b) in i[..frames].iter().zip(o[..frames].iter_mut()) {
                        *b = *a * g;
                        if l > 0 {
                            l -= 1;
                            g = if l == 0 { tgt } else { g + inc0 };
                        }
                    }
                    *current = g;
                    *left = l;
                }
                if x.is_empty() {
                    let n = frames.min(left0);
                    *left = left0 - n;
                    *current = if *left == 0 { tgt } else { c0 + inc0 * n as f32 };
                }
            }
            Run::Filters { stage, base, automation, dynamic_base, dynamic_automation, fs, dynamic, ramp } => {
                ramp.started = true;
                if ramp.active() {
                    // the glide: sub-blocks with the glided automation value
                    let mut at = 0;
                    let w = ramp.waiting(frames);
                    if w > 0 {
                        process_filters(stage, *dynamic, x, y, 0, w);
                        at = w;
                    }
                    while at < frames {
                        let n = if ramp.active() { TONE_RAMP_STEP.min(frames - at) } else { frames - at };
                        if let Some(v) = ramp.advance(n) {
                            apply_tone_automation(stage, base, automation, *fs, v);
                            apply_dynamic_tone_automation(stage, dynamic_base, dynamic_automation, *fs, v);
                        }
                        process_filters(stage, *dynamic, x, y, at, n);
                        at += n;
                    }
                    return;
                }
                if *dynamic {
                    for (i, o) in x.iter().zip(y.iter_mut()) {
                        o[..frames].copy_from_slice(&i[..frames]);
                    }
                    let mut it = y.iter_mut();
                    let mut refs: [&mut [f32]; MAX_CHANNELS] =
                        core::array::from_fn(|_| it.next().map(|v| &mut v[..frames]).unwrap_or(&mut []));
                    stage.process_block_float(&mut refs[..x.len().min(MAX_CHANNELS)], frames);
                } else {
                    for (c, (i, o)) in x.iter().zip(y.iter_mut()).enumerate() {
                        stage.process_float(c, &i[..frames], &mut o[..frames]);
                    }
                }
            }
            Run::Comp(p) => {
                for (i, o) in x.iter().zip(y.iter_mut()) {
                    o[..frames].copy_from_slice(&i[..frames]);
                }
                let mut it = y.iter_mut();
                let mut refs: [&mut [f32]; MAX_CHANNELS] =
                    core::array::from_fn(|_| it.next().map(|v| &mut v[..frames]).unwrap_or(&mut []));
                let w = x.len().min(MAX_CHANNELS);
                p.process_dynamics(&mut refs[..w], frames);
            }
            Run::Xtc(_)
            | Run::Cfreak(_)
            | Run::Mstk(_)
            | Run::Buzz(_)
            | Run::Cdelay(_)
            | Run::Lowflow(_)
            | Run::Vbass(_)
            | Run::Vareq(_)
            | Run::Norm(_)
            | Run::Volt(_)
            | Run::Pchr(_) => {
                for (i, o) in x.iter().zip(y.iter_mut()) {
                    o[..frames].copy_from_slice(&i[..frames]);
                }
                let mut it = y.iter_mut();
                let mut refs: [&mut [f32]; MAX_CHANNELS] =
                    core::array::from_fn(|_| it.next().map(|v| &mut v[..frames]).unwrap_or(&mut []));
                let refs = &mut refs[..x.len().min(MAX_CHANNELS)];
                match self {
                    Run::Xtc(r) => r.process(refs, frames),
                    Run::Cfreak(r) => r.process(refs, frames),
                    Run::Mstk(r) => r.process(refs, frames),
                    Run::Buzz(r) => r.process(refs, frames),
                    Run::Cdelay(r) => r.process(refs, frames),
                    Run::Lowflow(r) => r.process(refs, frames),
                    Run::Vbass(r) => r.process(refs, frames),
                    Run::Vareq(r) => r.process(refs, frames),
                    Run::Norm(r) => r.process(refs, frames),
                    Run::Volt(r) => r.process(refs, frames),
                    Run::Pchr(r) => r.process(refs, frames),
                    _ => {}
                }
            }
            Run::Through => {
                for (i, o) in x.iter().zip(y.iter_mut()) {
                    o[..frames].copy_from_slice(&i[..frames]);
                }
            }
        }
    }
}

pub struct Chain {
    spec: Spec,
    runs: Vec<Run>,
    /// Per node, per channel: the node's output for the current block.
    out: Vec<Vec<Vec<f32>>>,
    scratch: Vec<Vec<f32>>,
    max_frames: usize,
}

impl Chain {
    /// Builds the chain; `None` if the spec does not check.
    pub fn new(spec: Spec, fs: f64, max_frames: usize) -> Option<Chain> {
        spec.check().ok()?;
        if spec.nodes.iter().any(|node| matches!(&node.kind, Kind::Vareq(p) if !p.transitions_valid_at(fs))) {
            return None;
        }
        if !fs.is_finite()
            || fs <= 0.0
            || spec
                .nodes
                .iter()
                .any(|node| node.kind.tones().any(|tone| tone.dynamic.iter().any(|d| !d.params.valid(fs))))
        {
            return None;
        }
        let max_frames = max_frames.max(1);
        let runs = spec.nodes.iter().map(|n| Run::new(&n.kind, n.inputs.len(), fs, max_frames)).collect();
        let out = spec.nodes.iter().map(|n| vec![vec![0.0; max_frames]; n.inputs.len()]).collect();
        let scratch = vec![vec![0.0; max_frames]; MAX_CHANNELS];
        let mut c = Chain { spec, runs, out, scratch, max_frames };
        // the graph's own parameter defaults: full volume, no mute, centred
        for p in [GraphParam::VolumeDb, GraphParam::Mute, GraphParam::BalanceLeft, GraphParam::BalanceRight] {
            c.set_param(p, 0.0);
        }
        c.set_param(GraphParam::VolumeScalar, 1.0);
        Some(c)
    }

    pub fn spec(&self) -> &Spec {
        &self.spec
    }

    /// Boxes still stubs, by node index.
    pub fn stubs(&self) -> impl Iterator<Item = (usize, [u8; 4])> + '_ {
        self.spec.nodes.iter().enumerate().filter_map(|(i, n)| match n.kind {
            Kind::Stub(s) => Some((i, s)),
            _ => None,
        })
    }

    /// Sets one of the graph's parameters on every box it is wired to.
    /// Real time.
    pub fn set_param(&mut self, p: GraphParam, v: f64) {
        for l in self.spec.links.iter().filter(|l| l.param == p) {
            if let Some(r) = self.runs.get_mut(l.node) {
                r.set(l.id, v);
            }
        }
        self.apply_relays();
    }

    fn apply_relays(&mut self) {
        for relay in self.spec.relays.iter().copied() {
            let Some(value) = self.runs.get(relay.from_node).and_then(|run| run.get(relay.from_id)) else { continue };
            if let Some(run) = self.runs.get_mut(relay.to_node) {
                run.set(relay.to_id, value);
            }
        }
    }

    pub fn reset(&mut self) {
        for r in &mut self.runs {
            r.reset();
        }
    }

    pub fn max_frames(&self) -> usize {
        self.max_frames
    }

    /// Node `n`'s output channel `c` from the last block processed.
    pub fn node_output(&self, n: usize, c: usize) -> Option<&[f32]> {
        self.out.get(n)?.get(c).map(|v| &v[..])
    }

    /// Runs one block: `program` (the program channels) into `lanes` (the
    /// unit's channels). At most [`Chain::max_frames`] frames; more are cut.
    /// Real time.
    pub fn process(&mut self, program: &[&[f32]], lanes: &mut [&mut [f32]], frames: usize) {
        self.process_observed(program, lanes, frames, |_, _| {});
    }

    /// Offline profiling hook. `observe(node, true)` precedes input routing
    /// and `observe(node, false)` follows the node's processing. The ordinary
    /// real-time entry point specializes this with an empty callback, so it
    /// performs no clock reads or other profiling work.
    #[inline]
    pub fn process_observed(
        &mut self,
        program: &[&[f32]],
        lanes: &mut [&mut [f32]],
        frames: usize,
        mut observe: impl FnMut(usize, bool),
    ) {
        let frames = frames.min(self.max_frames);
        for n in 0..self.runs.len() {
            observe(n, true);
            let w = self.spec.nodes[n].inputs.len();
            for c in 0..w {
                let src = self.spec.nodes[n].inputs[c];
                let dst = &mut self.scratch[c][..frames];
                match src {
                    Source::Program(p) => match program.get(p) {
                        Some(x) if x.len() >= frames => dst.copy_from_slice(&x[..frames]),
                        _ => dst.fill(0.0),
                    },
                    Source::Node(m, k) => dst.copy_from_slice(&self.out[m][k][..frames]),
                }
            }
            let (x, y) = (&self.scratch[..w], &mut self.out[n][..]);
            self.runs[n].process(x, y, frames);
            observe(n, false);
        }
        for (c, lane) in lanes.iter_mut().enumerate() {
            let n = frames.min(lane.len());
            match self.spec.outputs.get(c) {
                Some(Source::Node(m, k)) => lane[..n].copy_from_slice(&self.out[*m][*k][..n]),
                Some(Source::Program(p)) => match program.get(*p) {
                    Some(x) if x.len() >= n => lane[..n].copy_from_slice(&x[..n]),
                    _ => lane[..n].fill(0.0),
                },
                None => lane[..n].fill(0.0),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fanout_spec() -> Spec {
        // program -> gain per side -> stub on the program -> four lanes
        Spec {
            program_channels: 2,
            nodes: vec![
                NodeSpec { kind: Kind::Gain, inputs: vec![Source::Program(0)] },
                NodeSpec { kind: Kind::Gain, inputs: vec![Source::Program(1)] },
                NodeSpec { kind: Kind::Stub(*b"vrq2"), inputs: vec![Source::Node(0, 0), Source::Node(1, 0)] },
                NodeSpec {
                    kind: Kind::NonFinite,
                    inputs: vec![Source::Node(2, 0), Source::Node(2, 1), Source::Node(2, 0), Source::Node(2, 1)],
                },
            ],
            outputs: vec![Source::Node(3, 0), Source::Node(3, 1), Source::Node(3, 2), Source::Node(3, 3)],
            links: vec![
                Link { param: GraphParam::VolumeDb, node: 0, id: 0 },
                Link { param: GraphParam::VolumeDb, node: 1, id: 0 },
                Link { param: GraphParam::Mute, node: 0, id: 1 },
            ],
            relays: vec![],
            volume: Default::default(),
        }
    }

    #[test]
    fn fans_out_and_applies_volume() {
        let mut c = Chain::new(fanout_spec(), 48_000.0, 64).unwrap();
        assert_eq!(c.stubs().collect::<Vec<_>>(), vec![(2, *b"vrq2")]);
        let l = [0.5f32; 64];
        let r = [-0.25f32; 64];
        let mut o = [[0.0f32; 64]; 4];
        let (a, rest) = o.split_at_mut(1);
        let (b, rest) = rest.split_at_mut(1);
        let (cc, d) = rest.split_at_mut(1);
        let mut lanes: [&mut [f32]; 4] = [&mut a[0], &mut b[0], &mut cc[0], &mut d[0]];
        c.process(&[&l, &r], &mut lanes, 64);
        assert_eq!(lanes[0][10], 0.5);
        assert_eq!(lanes[3][10], -0.25);
        // a volume or mute change ramps over 20 ms (960 samples at 48 kHz)
        c.set_param(GraphParam::VolumeDb, -20.0);
        for _ in 0..16 {
            c.process(&[&l, &r], &mut lanes, 64);
        }
        assert!((lanes[2][5] - 0.05).abs() < 1e-6);
        c.set_param(GraphParam::Mute, 1.0);
        for _ in 0..16 {
            c.process(&[&l, &r], &mut lanes, 64);
        }
        assert_eq!(lanes[0][5], 0.0);
        assert!((lanes[1][5] + 0.025).abs() < 1e-6);
    }

    #[test]
    fn gain_changes_ramp_linearly_in_amplitude_over_20_ms() {
        let mut c = Chain::new(fanout_spec(), 48_000.0, 512).unwrap();
        // set before the first block: installed, not ramped to
        c.set_param(GraphParam::VolumeDb, -6.0);
        let one = [1.0f32; 512];
        let mut o = vec![[0.0f32; 512]; 4];
        let run = |c: &mut Chain, o: &mut Vec<[f32; 512]>| -> Vec<f32> {
            let mut it = o.iter_mut();
            let mut lanes: Vec<&mut [f32]> = (0..4).map(|_| &mut it.next().unwrap()[..]).collect();
            c.process(&[&one, &one], &mut lanes, 512);
            lanes[0].to_vec()
        };
        let g0 = libm::pow(10.0, -6.0 / 20.0) as f32;
        assert!(run(&mut c, &mut o).iter().all(|v| (*v - g0).abs() < 1e-7));
        c.set_param(GraphParam::VolumeDb, -12.0);
        let g1 = libm::pow(10.0, -12.0 / 20.0) as f32;
        let mut y = run(&mut c, &mut o);
        y.extend(run(&mut c, &mut o));
        let inc = (g1 - g0) / 960.0;
        assert!((y[0] - g0).abs() < 1e-7, "the current value on the first sample");
        assert!((y[480] - (g0 + 480.0 * inc)).abs() < 1e-5, "linear in amplitude");
        assert!(y[960..].iter().all(|v| *v == g1), "held at the target after 960 samples");
        // a change during a ramp waits for it to finish
        c.set_param(GraphParam::VolumeDb, 0.0);
        let a = run(&mut c, &mut o);
        c.set_param(GraphParam::VolumeDb, -20.0);
        let b = run(&mut c, &mut o);
        assert!(a[0] == g1 && b[0] > a[511] && b[0] < 1.0, "still ramping toward 0 dB");
    }

    #[test]
    fn sum_adds_its_inputs_and_exposes_only_its_output() {
        // stereo program summed to mono, then fanned to two lanes
        let spec = Spec {
            program_channels: 2,
            nodes: vec![
                NodeSpec { kind: Kind::Sum(1), inputs: vec![Source::Program(0), Source::Program(1)] },
                NodeSpec { kind: Kind::NonFinite, inputs: vec![Source::Node(0, 0)] },
            ],
            outputs: vec![Source::Node(1, 0), Source::Node(1, 0)],
            links: vec![],
            relays: vec![],
            volume: Default::default(),
        };
        let mut c = Chain::new(spec.clone(), 48_000.0, 8).unwrap();
        let l = [0.5f32; 8];
        let r = [-0.125f32; 8];
        let mut o = [[0.0f32; 8]; 2];
        let mut it = o.iter_mut();
        let mut lanes: [&mut [f32]; 2] = core::array::from_fn(|_| &mut it.next().unwrap()[..]);
        c.process(&[&l, &r], &mut lanes, 8);
        assert_eq!(lanes[0][3], 0.375);
        assert_eq!(lanes[1][7], 0.375);
        // a later box may not name a sum's input-only channel
        let mut bad = spec.clone();
        bad.nodes[1].inputs[0] = Source::Node(0, 1);
        assert_eq!(bad.check(), Err(SpecError::BadSource { node: 1, channel: 0 }));
        let mut bad = spec;
        bad.nodes[0].kind = Kind::Sum(0);
        assert_eq!(bad.check(), Err(SpecError::BadWidth { node: 0 }));
    }

    #[test]
    fn volume_never_boosts_and_garbage_is_silenced() {
        let mut c = Chain::new(fanout_spec(), 48_000.0, 8).unwrap();
        c.set_param(GraphParam::VolumeDb, 12.0);
        let l = [0.5f32, f32::NAN, f32::INFINITY, 0.5, 0.5, 0.5, 0.5, 0.5];
        let mut o = [[0.0f32; 8]; 4];
        let mut it = o.iter_mut();
        let mut lanes: [&mut [f32]; 4] = core::array::from_fn(|_| &mut it.next().unwrap()[..]);
        c.process(&[&l, &l], &mut lanes, 8);
        assert_eq!(lanes[0][0], 0.5);
        assert_eq!(lanes[0][1], 0.0);
        assert_eq!(lanes[0][2], 0.0);
    }

    #[test]
    fn rejects_forward_sources() {
        let mut s = fanout_spec();
        s.nodes[0].inputs[0] = Source::Node(2, 0);
        assert_eq!(s.check(), Err(SpecError::BadSource { node: 0, channel: 0 }));
    }

    #[test]
    fn volume_taper_relay_primes_compressor_before_audio() {
        let volt = super::super::volt::Params {
            saved_scalar: 0.25,
            pre_gain_db: 0.0,
            curve_db: [-6.0; super::super::volt::CURVE_POINTS],
            offset_db: [0.0; super::super::volt::OFFSET_POINTS],
        };
        let mut points = [super::super::mstk::AutomationPoint::default(); super::super::mstk::MAX_AUTOMATION_POINTS];
        points[..2].copy_from_slice(&[
            super::super::mstk::AutomationPoint { input: -12.0, value: -10.0 },
            super::super::mstk::AutomationPoint { input: 0.0, value: -22.0 },
        ]);
        let curve = super::super::mstk::ThresholdAutomation { points, count: 2 };
        let comp = super::super::mstk::Params {
            automation_min: -12.0,
            automation_max: 0.0,
            threshold_automation: [
                curve,
                super::super::mstk::ThresholdAutomation::default(),
                super::super::mstk::ThresholdAutomation::default(),
                super::super::mstk::ThresholdAutomation::default(),
            ],
            ..super::super::mstk::Params::default()
        };
        let spec = Spec {
            program_channels: 1,
            nodes: vec![
                NodeSpec { kind: Kind::Volt(Box::new(volt)), inputs: vec![Source::Program(0)] },
                NodeSpec { kind: Kind::Mstk(Box::new(comp)), inputs: vec![Source::Node(0, 0)] },
            ],
            outputs: vec![Source::Node(1, 0)],
            links: vec![Link { param: GraphParam::VolumeScalar, node: 0, id: 4 }],
            relays: vec![Relay { from_node: 0, from_id: 100, to_node: 1, to_id: 79 }],
            volume: Default::default(),
        };
        let chain = Chain::new(spec, 48_000.0, 16).unwrap();
        let Run::Mstk(comp) = &chain.runs[1] else { panic!("compressor runner") };
        assert_eq!(comp.down_threshold_db(0), Some(-16.0));
    }

    fn automated_tone() -> Spec {
        let fs = 48_000.0;
        let mut channel = Channel::IDENTITY;
        channel.count = 1;
        channel.sections[0] = crate::voicing::tone::low_shelf(12.0, 100.0, core::f64::consts::FRAC_1_SQRT_2, fs);
        Spec {
            program_channels: 1,
            nodes: vec![NodeSpec {
                kind: Kind::Tone(Tone {
                    channels: vec![channel],
                    dynamic: Vec::new(),
                    dynamic_automation: Vec::new(),
                    automation: vec![ToneAutomation {
                        shape: ToneShape::LowShelf,
                        quality: core::f64::consts::FRAC_1_SQRT_2,
                        fixed: [0.0; 3],
                        max_order: 10,
                        a: TonePoint { scalar: 0.2, gain_db: 12.0, frequency_hz: 100.0 },
                        b: TonePoint { scalar: 0.8, gain_db: 0.0, frequency_hz: 1600.0 },
                        midpoint: Some(TonePoint { scalar: 0.5, gain_db: 9.0, frequency_hz: 200.0 }),
                        targets: vec![ToneTarget { channel: 0, section: 0 }],
                    }],
                }),
                inputs: vec![Source::Program(0)],
            }],
            outputs: vec![Source::Node(0, 0)],
            links: vec![Link { param: GraphParam::VolumeScalar, node: 0, id: 3 }],
            relays: vec![],
            volume: Default::default(),
        }
    }

    fn tone_channel(chain: &Chain) -> &Channel {
        match &chain.runs[0] {
            Run::Filters { stage, .. } => &stage.params().channel[0],
            _ => panic!("not a filter run"),
        }
    }

    #[test]
    fn tone_automation_is_piecewise_db_and_log_hz_and_clamps() {
        let fs = 48_000.0;
        let mut c = Chain::new(automated_tone(), fs, 8).unwrap();
        // Chain construction applies the graph's full-volume default: B.
        assert!(tone_channel(&c).magnitude(0.0, fs).log10().abs() < 1e-10);

        c.set_param(GraphParam::VolumeScalar, 0.35);
        let ch = tone_channel(&c);
        let dc_db = 20.0 * libm::log10(ch.magnitude(0.0, fs));
        // the matched-Z shelf's DC gain, to within its Float32 words
        assert!((dc_db - 10.5).abs() < 1e-3, "{dc_db}");
        // Halfway from A to M: gain is halfway in dB, frequency is the
        // geometric mean. A Butterworth-shaped shelf is half its gain there.
        let f = libm::sqrt(100.0 * 200.0);
        let at_f = 20.0 * libm::log10(ch.magnitude(f, fs));
        assert!((at_f - 5.25).abs() < 0.01, "{at_f}");

        c.set_param(GraphParam::VolumeScalar, -10.0);
        let below = 20.0 * libm::log10(tone_channel(&c).magnitude(0.0, fs));
        assert!((below - 12.0).abs() < 1e-3, "{below}");
        c.set_param(GraphParam::VolumeScalar, 10.0);
        assert!(tone_channel(&c).magnitude(0.0, fs).log10().abs() < 1e-10);
    }

    fn shelf_tone(shape: ToneShape, saved: [f64; 4], gain_db: f64, b: TonePoint) -> (Spec, Channel) {
        let fs = 48_000.0;
        let band = shape == ToneShape::ButterworthBandShelf;
        let mut channel = Channel::IDENTITY;
        let n = crate::voicing::tone::shelf_sections(10);
        channel.count = if band { 2 * n } else { n };
        let (first, second) = if band {
            let (first, second) = crate::voicing::tone::butterworth_band_shelf(gain_db, saved, 10, fs);
            (first, Some(second))
        } else {
            (crate::voicing::tone::butterworth_shelf(gain_db, saved[0], saved[1], 10, fs), None)
        };
        channel.sections[..first.count].copy_from_slice(&first.sections[..first.count]);
        if let Some(second) = second {
            channel.sections[n..n + second.count].copy_from_slice(&second.sections[..second.count]);
        }
        let spec = Spec {
            program_channels: 1,
            nodes: vec![NodeSpec {
                kind: Kind::Tone(Tone {
                    channels: vec![channel],
                    dynamic: Vec::new(),
                    dynamic_automation: Vec::new(),
                    automation: vec![ToneAutomation {
                        shape,
                        quality: 0.0,
                        fixed: [saved[1], saved[2], saved[3]],
                        max_order: 10,
                        a: TonePoint { scalar: 0.0, gain_db, frequency_hz: saved[0] },
                        b,
                        midpoint: None,
                        targets: vec![ToneTarget { channel: 0, section: 0 }],
                    }],
                }),
                inputs: vec![Source::Program(0)],
            }],
            outputs: vec![Source::Node(0, 0)],
            links: vec![Link { param: GraphParam::VolumeScalar, node: 0, id: 3 }],
            relays: vec![],
            volume: Default::default(),
        };
        (spec, channel)
    }

    #[test]
    fn an_unwired_tone_runs_its_automation_at_one_fixed_volume() {
        let fs = 48_000.0;
        let Kind::Tone(tone) = automated_tone().nodes[0].kind.clone() else { panic!("not a tone") };
        // below set point A: the saved section itself
        let held = tone.at_volume(fs, 0.0);
        assert!(held.automation.is_empty() && held.dynamic_automation.is_empty());
        assert_eq!(held.channels[0].sections[0], tone.channels[0].sections[0]);
        // at the midpoint: the midpoint's own shelf, as the running chain sets it
        let mid = tone.at_volume(fs, 0.5);
        assert_eq!(
            mid.channels[0].sections[0],
            crate::voicing::tone::low_shelf(9.0, 200.0, core::f64::consts::FRAC_1_SQRT_2, fs)
        );
        let mut chain = Chain::new(automated_tone(), fs, 512).unwrap();
        chain.set_param(GraphParam::VolumeScalar, 0.5);
        assert_eq!(tone_channel(&chain).sections[0], mid.channels[0].sections[0]);
        assert_eq!(tone_channel(&chain).gain, mid.channels[0].gain);
    }

    #[test]
    fn endpoint_published_peak_uses_the_b_path_only_at_its_float32_endpoint() {
        let fs = 48_000.0;
        let mut channel = Channel::IDENTITY;
        channel.count = 1;
        let automation = ToneAutomation {
            shape: ToneShape::EndpointPublishedPeak,
            quality: 10.0,
            fixed: [0.0; 3],
            max_order: 10,
            a: TonePoint { scalar: 0.0, gain_db: -3.0, frequency_hz: 300.0 },
            b: TonePoint { scalar: 1.0, gain_db: -12.0, frequency_hz: 333.0 },
            midpoint: Some(TonePoint { scalar: 0.5, gain_db: -6.0, frequency_hz: 320.0 }),
            targets: vec![ToneTarget { channel: 0, section: 0 }],
        };
        assert!(at_b_endpoint(&automation, 1.0 - 15.0 * f64::from(f32::EPSILON)));
        assert!(!at_b_endpoint(&automation, 1.0 - 16.0 * f64::from(f32::EPSILON)));
        let tone = Tone {
            channels: vec![channel],
            automation: vec![automation],
            dynamic: Vec::new(),
            dynamic_automation: Vec::new(),
        };
        let endpoint = tone.at_volume(fs, 1.0).channels[0].sections[0];
        let direct = crate::voicing::tone::peak(-12.0, 333.0, 10.0, fs);
        let published = crate::voicing::tone::peak(-12.0, published_frequency(333.0), 10.0, fs);
        assert_ne!(direct, published);
        assert_eq!(endpoint, published);
    }

    #[test]
    fn upper_endpoint_peak_publishes_the_next_float32_frequency_only_at_b() {
        let fs = 48_000.0;
        let mut channel = Channel::IDENTITY;
        channel.count = 1;
        let automation = ToneAutomation {
            shape: ToneShape::UpperEndpointPeak,
            quality: 2.0,
            fixed: [0.0; 3],
            max_order: 10,
            a: TonePoint { scalar: 0.0, gain_db: -2.0, frequency_hz: 700.0 },
            b: TonePoint { scalar: 1.0, gain_db: -4.0, frequency_hz: 1_300.0 },
            midpoint: None,
            targets: vec![ToneTarget { channel: 0, section: 0 }],
        };
        let tone = Tone {
            channels: vec![channel],
            automation: vec![automation.clone()],
            dynamic: Vec::new(),
            dynamic_automation: Vec::new(),
        };
        let endpoint = tone.at_volume(fs, 1.0).channels[0].sections[0];
        let published = crate::voicing::tone::peak(-4.0, upper_endpoint_frequency(1_300.0), 2.0, fs);
        assert_eq!((upper_endpoint_frequency(1_300.0) as f32).to_bits(), 1_300.0f32.to_bits() + 1);
        assert_eq!(endpoint, published);
        let (gain_db, frequency_hz) = automated_values(&automation, 0.5);
        assert_eq!(
            tone.at_volume(fs, 0.5).channels[0].sections[0],
            crate::voicing::tone::peak(gain_db, frequency_hz, 2.0, fs)
        );
    }

    #[test]
    fn tone_automation_moves_a_butterworth_shelf_as_a_whole() {
        let fs = 48_000.0;
        let db = |c: &Channel, f: f64| 20.0 * libm::log10(c.magnitude(f, fs));
        // A band shelf (type 0): every edge scales by B's frequency over the
        // geometric centre of the inner edges (black-box renders of the
        // installed unit), the gain glides in dB.
        let saved = [40.0, 60.0, 135.0, 180.0];
        let b = TonePoint { scalar: 1.0, gain_db: 4.0, frequency_hz: 2.0 * libm::sqrt(60.0 * 135.0) };
        let (spec, at_a) = shelf_tone(ToneShape::ButterworthBandShelf, saved, 10.0, b);
        let mut c = Chain::new(spec, fs, 8).unwrap();
        let (_, moved) = shelf_tone(ToneShape::ButterworthBandShelf, saved.map(|f| 2.0 * f), 4.0, b);
        for f in [30.0, 80.0, 180.0, 300.0, 1000.0, 10_000.0] {
            assert!((db(tone_channel(&c), f) - db(&moved, f)).abs() < 1e-9, "{f} Hz at B");
        }
        // at or below A it is the saved band shelf
        c.set_param(GraphParam::VolumeScalar, 0.0);
        for f in [30.0, 80.0, 180.0, 300.0, 1000.0, 10_000.0] {
            assert!((db(tone_channel(&c), f) - db(&at_a, f)).abs() < 1e-9, "{f} Hz at A");
        }
        // halfway: half the gain in dB, the centre at the geometric mean
        c.set_param(GraphParam::VolumeScalar, 0.5);
        let (_, half) = shelf_tone(ToneShape::ButterworthBandShelf, saved.map(|f| libm::sqrt(2.0) * f), 7.0, b);
        for f in [30.0, 80.0, 180.0, 300.0, 1000.0, 10_000.0] {
            assert!((db(tone_channel(&c), f) - db(&half, f)).abs() < 1e-9, "{f} Hz halfway");
        }
        // A type-2 shelf: both edges scale by B's frequency over the first.
        let saved = [120.0, 180.0, 0.0, 0.0];
        let b = TonePoint { scalar: 1.0, gain_db: 3.0, frequency_hz: 240.0 };
        let (spec, _) = shelf_tone(ToneShape::ButterworthShelf, saved, 6.0, b);
        let c = Chain::new(spec, fs, 8).unwrap();
        let (_, moved) = shelf_tone(ToneShape::ButterworthShelf, [240.0, 360.0, 0.0, 0.0], 3.0, b);
        for f in [30.0, 150.0, 300.0, 1000.0] {
            assert!((db(tone_channel(&c), f) - db(&moved, f)).abs() < 1e-9, "{f} Hz type 2 at B");
        }
    }

    #[test]
    fn tone_automation_spec_rejects_bad_midpoint_and_target() {
        let mut s = automated_tone();
        let Kind::Tone(tone) = &mut s.nodes[0].kind else { unreachable!() };
        tone.automation[0].midpoint.as_mut().unwrap().scalar = 0.9;
        assert_eq!(s.check(), Err(SpecError::BadParams { node: 0 }));
        let Kind::Tone(tone) = &mut s.nodes[0].kind else { unreachable!() };
        tone.automation[0].midpoint.as_mut().unwrap().scalar = 0.5;
        tone.automation[0].targets[0].section = 1;
        assert_eq!(s.check(), Err(SpecError::BadParams { node: 0 }));
    }

    fn dynamic_tone() -> Spec {
        Spec {
            program_channels: 1,
            nodes: vec![NodeSpec {
                kind: Kind::Tone(Tone {
                    channels: vec![Channel::IDENTITY],
                    automation: Vec::new(),
                    dynamic_automation: Vec::new(),
                    dynamic: vec![DynamicAt {
                        active_mask: 1,
                        sidechain_mask: 1,
                        params: crate::prestage::DynamicFilter {
                            kind: crate::prestage::DynamicKind::Notch,
                            range_db: -12.0,
                            frequency_hz: 1_000.0,
                            quality: 2.0,
                            ratio: 4.0,
                            threshold_db: -20.0,
                            attack_ms: 0.0,
                            release_ms: 0.0,
                            smooth: false,
                        },
                        ..Default::default()
                    }],
                }),
                inputs: vec![Source::Program(0)],
            }],
            outputs: vec![Source::Node(0, 0)],
            links: Vec::new(),
            relays: vec![],
            volume: Default::default(),
        }
    }

    fn run_dynamic_tone(amplitude: f64) -> f64 {
        let fs = 48_000.0;
        let n = 48_000;
        let input: Vec<f32> = (0..n)
            .map(|i| (amplitude * libm::sin(2.0 * core::f64::consts::PI * 1_000.0 * i as f64 / fs)) as f32)
            .collect();
        let mut output = vec![0.0f32; n];
        let mut chain = Chain::new(dynamic_tone(), fs, 512).unwrap();
        for at in (0..n).step_by(512) {
            let end = (at + 512).min(n);
            let mut lanes = [&mut output[at..end]];
            chain.process(&[&input[at..end]], &mut lanes, end - at);
        }
        let power = output[n / 2..].iter().map(|v| f64::from(*v) * f64::from(*v)).sum::<f64>() / (n / 2) as f64;
        20.0 * libm::log10(libm::sqrt(2.0 * power) / amplitude)
    }

    #[test]
    fn tone_chain_runs_level_dependent_slots() {
        let neutral = run_dynamic_tone(1e-3);
        assert!(neutral > 0.10 && neutral < 0.13, "{neutral}");
        assert!(run_dynamic_tone(1.0) < -11.5);

        let mut bad = dynamic_tone();
        let Kind::Tone(tone) = &mut bad.nodes[0].kind else { unreachable!() };
        tone.dynamic[0].split[0] = 1;
        assert_eq!(bad.check(), Err(SpecError::BadParams { node: 0 }));
    }

    #[test]
    fn dynamic_tone_volume_automation_updates_range_and_frequency() {
        let mut spec = dynamic_tone();
        let Kind::Tone(tone) = &mut spec.nodes[0].kind else { unreachable!() };
        tone.dynamic_automation.push(DynamicToneAutomation {
            index: 0,
            a: TonePoint { scalar: 0.2, gain_db: -6.0, frequency_hz: 500.0 },
            b: TonePoint { scalar: 0.8, gain_db: -12.0, frequency_hz: 2_000.0 },
            midpoint: None,
        });
        spec.links.push(Link { param: GraphParam::VolumeScalar, node: 0, id: 3 });
        let mut chain = Chain::new(spec, 48_000.0, 8).unwrap();
        let params = |chain: &Chain| match &chain.runs[0] {
            Run::Filters { stage, .. } => stage.params().dynamic[0].params,
            _ => panic!("not a filter run"),
        };
        assert_eq!(params(&chain).range_db, -12.0);
        assert!((params(&chain).frequency_hz - 2_000.0).abs() < 1e-9);
        chain.set_param(GraphParam::VolumeScalar, 0.2);
        assert_eq!(params(&chain).range_db, -6.0);
        assert!((params(&chain).frequency_hz - 500.0).abs() < 1e-9);
    }
}
