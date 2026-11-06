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
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Link {
    pub param: GraphParam,
    pub node: usize,
    pub id: u32,
}

/// The part of one ToneMeister slot that volume automation may move.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToneShape {
    Peak,
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
    /// First section occupied by the slot. Ignored for [`ToneShape::Gain`].
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
    /// Shape fields that do not move with volume (the remaining shelf
    /// edge frequencies for type 0/2 slots).
    pub fixed: [f64; 3],
    pub max_order: u32,
    pub a: TonePoint,
    pub b: TonePoint,
    pub midpoint: Option<TonePoint>,
    pub targets: Vec<ToneTarget>,
}

/// A filter box at its saved state plus the slots driven by system volume.
#[derive(Clone, Debug, PartialEq)]
pub struct Tone {
    pub channels: Vec<Channel>,
    pub automation: Vec<ToneAutomation>,
    /// Level-dependent slots in graph order. They have no compressor-box
    /// boundary inside a single Tone box, so every `before_box` is zero.
    pub dynamic: Vec<DynamicAt>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Kind {
    NonFinite,
    Gain,
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
    /// The box's four-character subtype.
    Stub([u8; 4]),
    Bypass,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NodeSpec {
    pub kind: Kind,
    /// Per box channel, its source.
    pub inputs: Vec<Source>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Spec {
    pub program_channels: usize,
    pub nodes: Vec<NodeSpec>,
    /// Per unit channel.
    pub outputs: Vec<Source>,
    pub links: Vec<Link>,
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
            Source::Node(n, c) => n < before && c < self.nodes[n].inputs.len(),
        };
        for (n, node) in self.nodes.iter().enumerate() {
            let w = node.inputs.len();
            if w == 0 || w > MAX_CHANNELS {
                return Err(SpecError::BadWidth { node: n });
            }
            if let Kind::Tone(tone) = &node.kind {
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
        Ok(())
    }
}

fn shape_sections(shape: ToneShape) -> usize {
    match shape {
        ToneShape::LowPass(n) | ToneShape::HighPass(n) => (n as usize).div_ceil(2),
        ToneShape::Gain => 0,
        ToneShape::ButterworthShelf => 5,
        ToneShape::ButterworthBandShelf => 10,
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
        && (1..=10).contains(&a.max_order)
        && a.fixed.iter().all(|v| v.is_finite() && *v >= 0.0)
        && !a.targets.is_empty()
        && a.targets.iter().all(|t| {
            channels.get(t.channel).is_some_and(|c| {
                t.section <= MAX_SECTIONS && shape_sections(a.shape) <= c.count.saturating_sub(t.section)
            })
        })
}

fn interpolate(a: TonePoint, b: TonePoint, scalar: f64) -> (f64, f64) {
    let t = ((scalar - a.scalar) / (b.scalar - a.scalar)).clamp(0.0, 1.0);
    let gain = a.gain_db + t * (b.gain_db - a.gain_db);
    let frequency = libm::exp(libm::log(a.frequency_hz) + t * (libm::log(b.frequency_hz) - libm::log(a.frequency_hz)));
    (gain, frequency)
}

fn automated_values(a: &ToneAutomation, scalar: f64) -> (f64, f64) {
    match a.midpoint {
        Some(m) if scalar <= m.scalar => interpolate(a.a, m, scalar),
        Some(m) => interpolate(m, a.b, scalar),
        None => interpolate(a.a, a.b, scalar),
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
                    channel.gain *= libm::pow(10.0, (gain_db - a.a.gain_db) / 20.0);
                }
                ToneShape::Peak | ToneShape::LowShelf | ToneShape::HighShelf => {
                    let section = match a.shape {
                        ToneShape::Peak => super::tone::peak(gain_db, frequency_hz, a.quality, fs),
                        ToneShape::LowShelf => super::tone::low_shelf(gain_db, frequency_hz, a.quality, fs),
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
                    let made = super::tone::butterworth_shelf(gain_db, frequency_hz, a.fixed[0], a.max_order, fs);
                    let sections = &mut channel.sections[target.section..target.section + 5];
                    sections.fill(Section::IDENTITY);
                    sections[..made.count].copy_from_slice(&made.sections[..made.count]);
                }
                ToneShape::ButterworthBandShelf => {
                    let first = super::tone::butterworth_shelf(gain_db, a.fixed[0], frequency_hz, a.max_order, fs);
                    let second = super::tone::butterworth_shelf(gain_db, a.fixed[1], a.fixed[2], a.max_order, fs);
                    let sections = &mut channel.sections[target.section..target.section + 10];
                    sections.fill(Section::IDENTITY);
                    sections[..first.count].copy_from_slice(&first.sections[..first.count]);
                    sections[5..5 + second.count].copy_from_slice(&second.sections[..second.count]);
                    channel.gain *= libm::pow(10.0, -(gain_db - a.a.gain_db) / 20.0);
                }
                ToneShape::LowPass(order) | ToneShape::HighPass(order) => {
                    let n = shape_sections(a.shape);
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

enum Run {
    NonFinite,
    Gain { gain_db: f64, mute: bool, scale: f32 },
    Filters { stage: PreStage, base: Vec<Channel>, automation: Vec<ToneAutomation>, fs: f64, dynamic: bool },
    Comp(PreStage),
    Xtc(alloc::boxed::Box<super::xtc::Run>),
    Cfreak(alloc::boxed::Box<super::cfreak::Run>),
    Mstk(alloc::boxed::Box<super::mstk::Run>),
    Buzz(alloc::boxed::Box<super::buzz::Run>),
    Cdelay(alloc::boxed::Box<super::cdelay::Run>),
    Lowflow(alloc::boxed::Box<super::lowflow::Run>),
    Vbass(alloc::boxed::Box<super::vbass::Run>),
    Through,
}

impl Run {
    fn new(kind: &Kind, width: usize, fs: f64) -> Run {
        match kind {
            Kind::NonFinite => Run::NonFinite,
            Kind::Gain => Run::Gain { gain_db: 0.0, mute: false, scale: 1.0 },
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
                    fs,
                    dynamic: !tone.dynamic.is_empty(),
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
            Kind::Vbass(p) => Run::Vbass(alloc::boxed::Box::new(super::vbass::Run::new(p, fs, width))),
            Kind::Stub(_) | Kind::Bypass => Run::Through,
        }
    }

    fn set(&mut self, id: u32, v: f64) {
        match self {
            Run::Gain { gain_db, mute, scale } => {
                match id {
                    // a gain box only ever attenuates: it never adds drive
                    0 if v.is_finite() => *gain_db = v.min(0.0),
                    1 => *mute = v != 0.0,
                    _ => {}
                }
                *scale = if *mute { 0.0 } else { libm::pow(10.0, *gain_db / 20.0) as f32 };
            }
            Run::Filters { stage, base, automation, fs, .. } if id == 3 && v.is_finite() => {
                apply_tone_automation(stage, base, automation, *fs, v);
            }
            _ => {}
        }
    }

    fn reset(&mut self) {
        match self {
            Run::Filters { stage, .. } | Run::Comp(stage) => stage.reset(),
            Run::Xtc(r) => r.reset(),
            Run::Vbass(r) => r.reset(),
            Run::Cfreak(r) => r.reset(),
            Run::Mstk(r) => r.reset(),
            Run::Buzz(r) => r.reset(),
            Run::Cdelay(r) => r.reset(),
            Run::Lowflow(r) => r.reset(),
            _ => {}
        }
    }

    /// `x` in, `y` out (both the box's width).
    fn process(&mut self, x: &[Vec<f32>], y: &mut [Vec<f32>], frames: usize) {
        match self {
            Run::NonFinite => {
                for (i, o) in x.iter().zip(y.iter_mut()) {
                    for (a, b) in i[..frames].iter().zip(o[..frames].iter_mut()) {
                        *b = if a.is_finite() { *a } else { 0.0 };
                    }
                }
            }
            Run::Gain { scale, .. } => {
                for (i, o) in x.iter().zip(y.iter_mut()) {
                    for (a, b) in i[..frames].iter().zip(o[..frames].iter_mut()) {
                        *b = *a * *scale;
                    }
                }
            }
            Run::Filters { stage, dynamic, .. } => {
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
            | Run::Vbass(_) => {
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
        if !fs.is_finite()
            || fs <= 0.0
            || spec
                .nodes
                .iter()
                .any(|node| matches!(&node.kind, Kind::Tone(tone) if tone.dynamic.iter().any(|d| !d.params.valid(fs))))
        {
            return None;
        }
        let max_frames = max_frames.max(1);
        let runs = spec.nodes.iter().map(|n| Run::new(&n.kind, n.inputs.len(), fs)).collect();
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
        let frames = frames.min(self.max_frames);
        for n in 0..self.runs.len() {
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
        c.set_param(GraphParam::VolumeDb, -20.0);
        c.process(&[&l, &r], &mut lanes, 64);
        assert!((lanes[2][5] - 0.05).abs() < 1e-6);
        c.set_param(GraphParam::Mute, 1.0);
        c.process(&[&l, &r], &mut lanes, 64);
        assert_eq!(lanes[0][5], 0.0);
        assert!((lanes[1][5] + 0.025).abs() < 1e-6);
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
        assert!((dc_db - 10.5).abs() < 1e-8, "{dc_db}");
        // Halfway from A to M: gain is halfway in dB, frequency is the
        // geometric mean. A Butterworth-shaped shelf is half its gain there.
        let f = libm::sqrt(100.0 * 200.0);
        let at_f = 20.0 * libm::log10(ch.magnitude(f, fs));
        assert!((at_f - 5.25).abs() < 0.01, "{at_f}");

        c.set_param(GraphParam::VolumeScalar, -10.0);
        let below = 20.0 * libm::log10(tone_channel(&c).magnitude(0.0, fs));
        assert!((below - 12.0).abs() < 1e-7, "{below}");
        c.set_param(GraphParam::VolumeScalar, 10.0);
        assert!(tone_channel(&c).magnitude(0.0, fs).log10().abs() < 1e-10);
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
        assert!(run_dynamic_tone(1e-3).abs() < 0.02);
        assert!(run_dynamic_tone(1.0) < -11.5);

        let mut bad = dynamic_tone();
        let Kind::Tone(tone) = &mut bad.nodes[0].kind else { unreachable!() };
        tone.dynamic[0].split[0] = 1;
        assert_eq!(bad.check(), Err(SpecError::BadParams { node: 0 }));
    }
}
