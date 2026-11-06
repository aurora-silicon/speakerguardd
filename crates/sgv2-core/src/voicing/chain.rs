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
//!   channel ([`crate::prestage::Channel`]);
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

use crate::prestage::{Channel, Dynamics, Params as PreParams, PreStage};
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

#[derive(Clone, Debug, PartialEq)]
pub enum Kind {
    NonFinite,
    Gain,
    /// One entry per box channel.
    Tone(Vec<Channel>),
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
            if let Kind::Tone(ch) = &node.kind {
                if ch.len() != w || ch.iter().any(|c| !c.sections().iter().all(|s| s.stable()) || !c.gain.is_finite()) {
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

enum Run {
    NonFinite,
    Gain { gain_db: f64, mute: bool, scale: f32 },
    Filters(PreStage),
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
            Kind::Tone(ch) => {
                let mut p = PreParams::identity(width);
                p.channel[..width].copy_from_slice(&ch[..width]);
                Run::Filters(PreStage::with_rate(p, fs))
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
        if let Run::Gain { gain_db, mute, scale } = self {
            match id {
                // a gain box only ever attenuates: it never adds drive
                0 if v.is_finite() => *gain_db = v.min(0.0),
                1 => *mute = v != 0.0,
                _ => {}
            }
            *scale = if *mute { 0.0 } else { libm::pow(10.0, *gain_db / 20.0) as f32 };
        }
    }

    fn reset(&mut self) {
        match self {
            Run::Filters(p) | Run::Comp(p) => p.reset(),
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
            Run::Filters(p) => {
                for (c, (i, o)) in x.iter().zip(y.iter_mut()).enumerate() {
                    p.process(c, &i[..frames], &mut o[..frames]);
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
            Run::Xtc(_) | Run::Cfreak(_) | Run::Mstk(_) | Run::Buzz(_) | Run::Cdelay(_) | Run::Lowflow(_) | Run::Vbass(_) => {
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
}
