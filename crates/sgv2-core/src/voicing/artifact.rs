// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Versioned deployment encoding of the complete voicing spec. No native
//! struct layout, pointers or runtime state are serialized. Integers are u32
//! and floats are IEEE f64, little endian. Box parameter words use each box's
//! existing checked codec. Changing this wire layout requires a version bump.
//! Decode and chain construction allocate and belong outside the RT callback.
//!
//! Header: magic (8 bytes), schema version u32, build flavor u32, CRC of
//! the preceding unit blob u32. The Spec follows in declared field order;
//! vector lengths and enum tags are u32. A trailing u32 CRC covers all of
//! the artifact before it. The build flavor binds the LowFlow law variant.
use super::chain::*;
use crate::blob::{crc32, BlobError, Header};
use crate::prestage::*;
use alloc::{boxed::Box, vec, vec::Vec};

pub const MAGIC: [u8; 8] = *b"SGVOICE\0";
pub const VERSION: u32 = 1;
// The conservative LowFlow build intentionally runs a different law. Do not
// silently load an artifact made for the other build of that same schema.
const FLAVOR: u32 = super::lowflow::CONSERVATIVE as u32;
pub const MAX_BYTES: usize = 4 * 1024 * 1024;
const MAX_ITEMS: usize = 4096;
type Result<T> = core::result::Result<T, BlobError>;

struct Reader<'a>(&'a [u8]);
impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> Result<[u8; N]> {
        let v = self.0.get(..N).ok_or(BlobError::Short)?.try_into().map_err(|_| BlobError::Short)?;
        self.0 = &self.0[N..];
        Ok(v)
    }
}
trait Wire: Sized {
    fn put(&self, out: &mut Vec<u8>);
    fn get(r: &mut Reader<'_>) -> Result<Self>;
}
impl Wire for u32 {
    fn put(&self, o: &mut Vec<u8>) {
        o.extend_from_slice(&self.to_le_bytes());
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        Ok(Self::from_le_bytes(r.take()?))
    }
}
impl Wire for usize {
    fn put(&self, o: &mut Vec<u8>) {
        (*self as u32).put(o);
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        Ok(u32::get(r)? as usize)
    }
}
impl Wire for f64 {
    fn put(&self, o: &mut Vec<u8>) {
        o.extend_from_slice(&self.to_le_bytes());
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        let v = Self::from_le_bytes(r.take()?);
        if v.is_finite() {
            Ok(v)
        } else {
            Err(BlobError::Unstable)
        }
    }
}
impl Wire for bool {
    fn put(&self, o: &mut Vec<u8>) {
        (*self as u32).put(o);
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        match u32::get(r)? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(BlobError::Length),
        }
    }
}
impl<T: Wire> Wire for Vec<T> {
    fn put(&self, o: &mut Vec<u8>) {
        self.len().put(o);
        for v in self {
            v.put(o);
        }
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        let n = usize::get(r)?;
        if n > MAX_ITEMS || n > r.0.len() {
            return Err(BlobError::Length);
        }
        (0..n).map(|_| T::get(r)).collect()
    }
}
impl<T: Wire> Wire for Option<T> {
    fn put(&self, o: &mut Vec<u8>) {
        self.is_some().put(o);
        if let Some(v) = self {
            v.put(o);
        }
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        if bool::get(r)? {
            Ok(Some(T::get(r)?))
        } else {
            Ok(None)
        }
    }
}
impl<T: Wire, const N: usize> Wire for [T; N] {
    fn put(&self, o: &mut Vec<u8>) {
        for v in self {
            v.put(o);
        }
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        let v: Vec<T> = (0..N).map(|_| T::get(r)).collect::<Result<_>>()?;
        v.try_into().map_err(|_| BlobError::Length)
    }
}
macro_rules! record {
    ($t:ty, $($field:ident),+ $(,)?) => {
        impl Wire for $t {
            fn put(&self, o: &mut Vec<u8>) { $(self.$field.put(o);)+ }
            fn get(r: &mut Reader<'_>) -> Result<Self> { Ok(Self { $($field: Wire::get(r)?,)+ }) }
        }
    };
}
record!(Spec, program_channels, nodes, outputs, links, relays);
record!(NodeSpec, kind, inputs);
record!(Link, param, node, id);
record!(Relay, from_node, from_id, to_node, to_id);
record!(Tone, channels, automation, dynamic, dynamic_automation);
record!(ToneAutomation, shape, quality, fixed, max_order, a, b, midpoint, targets);
record!(DynamicToneAutomation, index, a, b, midpoint);
record!(TonePoint, scalar, gain_db, frequency_hz);
record!(ToneTarget, channel, section);
record!(Channel, sections, count, gain);
record!(Section, b0, b1, b2, a1, a2);
record!(Dynamics, crossover_hz, input_gain_db, output_gain_db, release_s, band);
record!(Band, threshold_db, ratio, range_db, makeup_db, mode);
record!(DynamicAt, before_box, split, active_mask, sidechain_mask, params);
record!(DynamicFilter, kind, range_db, frequency_hz, quality, ratio, threshold_db, attack_ms, release_ms, smooth);

macro_rules! choices {
    ($t:ty, $($v:ident = $n:literal),+ $(,)?) => {
        impl Wire for $t {
            fn put(&self, o: &mut Vec<u8>) { match self { $(Self::$v => ($n as u32).put(o),)+ } }
            fn get(r: &mut Reader<'_>) -> Result<Self> {
                match u32::get(r)? { $($n => Ok(Self::$v),)+ _ => Err(BlobError::Length) }
            }
        }
    };
}
choices!(GraphParam, VolumeScalar = 0, VolumeDb = 1, Mute = 2, BalanceLeft = 3, BalanceRight = 4);
choices!(BandMode, Compress = 0, Pass = 1, Mute = 2);
choices!(DynamicKind, Notch = 15, HighShelf = 18);
impl Wire for Source {
    fn put(&self, o: &mut Vec<u8>) {
        match self {
            Self::Program(c) => {
                0u32.put(o);
                c.put(o);
            }
            Self::Node(n, c) => {
                1u32.put(o);
                n.put(o);
                c.put(o);
            }
        }
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        match u32::get(r)? {
            0 => Ok(Self::Program(usize::get(r)?)),
            1 => Ok(Self::Node(usize::get(r)?, usize::get(r)?)),
            _ => Err(BlobError::Length),
        }
    }
}
impl Wire for ToneShape {
    fn put(&self, o: &mut Vec<u8>) {
        let (tag, arg) = match *self {
            Self::Peak => (0, 0),
            Self::LowPass(n) => (1, n),
            Self::HighPass(n) => (2, n),
            Self::Gain => (3, 0),
            Self::LowShelf => (4, 0),
            Self::HighShelf => (5, 0),
            Self::ResonantHighPass => (6, 0),
            Self::ButterworthShelf => (7, 0),
            Self::ButterworthBandShelf => (8, 0),
        };
        (tag as u32).put(o);
        arg.put(o);
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        let tag = u32::get(r)?;
        let arg = u32::get(r)?;
        Ok(match (tag, arg) {
            (0, 0) => Self::Peak,
            (1, n) => Self::LowPass(n),
            (2, n) => Self::HighPass(n),
            (3, 0) => Self::Gain,
            (4, 0) => Self::LowShelf,
            (5, 0) => Self::HighShelf,
            (6, 0) => Self::ResonantHighPass,
            (7, 0) => Self::ButterworthShelf,
            (8, 0) => Self::ButterworthBandShelf,
            _ => return Err(BlobError::Length),
        })
    }
}
impl Wire for Kind {
    fn put(&self, o: &mut Vec<u8>) {
        match self {
            Self::NonFinite => 0u32.put(o),
            Self::Gain => 1u32.put(o),
            Self::Tone(p) => {
                2u32.put(o);
                p.put(o);
            }
            Self::Comp(p) => {
                3u32.put(o);
                p.put(o);
            }
            Self::Bypass => 4u32.put(o),
            Self::Stub(_) => 5u32.put(o), // never deploy a stub
            Self::Sum(out) => {
                17u32.put(o);
                out.put(o);
            }
            Self::Xtc(p) => {
                6u32.put(o);
                let mut w = vec![0.0; super::xtc::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Cfreak(p) => {
                7u32.put(o);
                let mut w = vec![0.0; super::cfreak::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Mstk(p) => {
                8u32.put(o);
                let mut w = vec![0.0; super::mstk::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Buzz(p) => {
                9u32.put(o);
                let mut w = vec![0.0; super::buzz::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Cdelay(p) => {
                10u32.put(o);
                let mut w = vec![0.0; super::cdelay::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Lowflow(p) => {
                11u32.put(o);
                let mut w = vec![0.0; super::lowflow::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Vbass(p) => {
                12u32.put(o);
                let mut w = vec![0.0; super::vbass::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Vareq(p) => {
                13u32.put(o);
                let mut w = vec![0.0; super::vareq::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Norm(p) => {
                14u32.put(o);
                let mut w = vec![0.0; super::norm::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Volt(p) => {
                15u32.put(o);
                let mut w = vec![0.0; super::volt::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
            Self::Pchr(p) => {
                16u32.put(o);
                let mut w = vec![0.0; super::pchr::Params::WORDS];
                p.to_words(&mut w);
                for v in w {
                    v.put(o);
                }
            }
        }
    }
    fn get(r: &mut Reader<'_>) -> Result<Self> {
        Ok(match u32::get(r)? {
            0 => Self::NonFinite,
            1 => Self::Gain,
            2 => Self::Tone(Tone::get(r)?),
            3 => Self::Comp(Dynamics::get(r)?),
            4 => Self::Bypass,
            17 => Self::Sum(usize::get(r)?),
            6 => {
                let w: Vec<f64> = (0..super::xtc::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Xtc(Box::new(super::xtc::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            7 => {
                let w: Vec<f64> = (0..super::cfreak::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Cfreak(Box::new(super::cfreak::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            8 => {
                let w: Vec<f64> = (0..super::mstk::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Mstk(Box::new(super::mstk::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            9 => {
                let w: Vec<f64> = (0..super::buzz::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Buzz(Box::new(super::buzz::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            10 => {
                let w: Vec<f64> = (0..super::cdelay::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Cdelay(Box::new(super::cdelay::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            11 => {
                let w: Vec<f64> = (0..super::lowflow::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Lowflow(Box::new(super::lowflow::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            12 => {
                let w: Vec<f64> = (0..super::vbass::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Vbass(Box::new(super::vbass::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            13 => {
                let w: Vec<f64> = (0..super::vareq::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Vareq(Box::new(super::vareq::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            14 => {
                let w: Vec<f64> = (0..super::norm::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Norm(Box::new(super::norm::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            15 => {
                let w: Vec<f64> = (0..super::volt::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Volt(Box::new(super::volt::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            16 => {
                let w: Vec<f64> = (0..super::pchr::Params::WORDS).map(|_| f64::get(r)).collect::<Result<_>>()?;
                Self::Pchr(Box::new(super::pchr::Params::from_words(&w).ok_or(BlobError::Unstable)?))
            }
            _ => return Err(BlobError::Length),
        })
    }
}

/// A checked, bound deployment spec. Its CRC covers the identity and spec.
pub fn encode(spec: &Spec, unit_blob: &[u8]) -> Result<Vec<u8>> {
    let (h, _) = Header::parse(unit_blob)?;
    validate(spec, h.sample_rate as f64, h.channels as usize)?;
    let mut out = MAGIC.to_vec();
    VERSION.put(&mut out);
    FLAVOR.put(&mut out);
    // Bind to the complete unit blob, including the board and pre-stage.
    crc32(unit_blob).put(&mut out);
    spec.put(&mut out);
    if out.len() > MAX_BYTES - 4 {
        return Err(BlobError::Length);
    }
    crc32(&out).put(&mut out);
    // Exercise exactly the deployed decoder before publishing.
    decode(&out, unit_blob)?;
    Ok(out)
}

/// The caller supplies the unit blob without its optional voicing extension.
pub fn decode(bytes: &[u8], unit_blob: &[u8]) -> Result<Spec> {
    if bytes.len() < 24 || bytes.len() > MAX_BYTES {
        return Err(BlobError::Length);
    }
    let (data, crc) = bytes.split_at(bytes.len() - 4);
    if crc32(data) != u32::from_le_bytes(crc.try_into().map_err(|_| BlobError::Length)?) {
        return Err(BlobError::Crc);
    }
    let mut r = Reader(data);
    if r.take::<8>()? != MAGIC {
        return Err(BlobError::Magic);
    }
    if u32::get(&mut r)? != VERSION || u32::get(&mut r)? != FLAVOR {
        return Err(BlobError::Version);
    }
    if u32::get(&mut r)? != crc32(unit_blob) {
        return Err(BlobError::Crc);
    }
    let (h, _) = Header::parse(unit_blob)?;
    let spec = Spec::get(&mut r)?;
    if !r.0.is_empty() {
        return Err(BlobError::Length);
    }
    validate(&spec, h.sample_rate as f64, h.channels as usize)?;
    Ok(spec)
}

fn validate(spec: &Spec, fs: f64, lanes: usize) -> Result<()> {
    if spec.program_channels != 2
        || spec.outputs.len() != lanes
        || spec.nodes.iter().any(|n| matches!(n.kind, Kind::Stub(_)))
    {
        return Err(BlobError::Channels);
    }
    // Check counts before Spec::check calls Channel::sections().
    if spec.nodes.iter().any(|n| matches!(&n.kind, Kind::Tone(t) if t.channels.iter().any(|c| c.count > MAX_SECTIONS)))
    {
        return Err(BlobError::Length);
    }
    spec.check().map_err(|_| BlobError::Unstable)?;
    for node in &spec.nodes {
        let width = node.inputs.len();
        match &node.kind {
            Kind::Xtc(_) | Kind::Vbass(_) if width != 2 => return Err(BlobError::Channels),
            Kind::Norm(_) if width > super::norm::MAX_CHANNELS => return Err(BlobError::Channels),
            Kind::Cdelay(p) if p.channels != width => return Err(BlobError::Channels),
            Kind::Lowflow(p) if p.model_count * 2 != width => return Err(BlobError::Channels),
            Kind::Pchr(p) if p.channels != width as f64 => return Err(BlobError::Channels),
            Kind::Tone(t) if t.dynamic.iter().any(|d| !d.params.valid(fs)) => return Err(BlobError::Unstable),
            Kind::Mstk(p) if !p.valid(fs) => return Err(BlobError::Unstable),
            Kind::Comp(p) if !p.valid(fs) => return Err(BlobError::Unstable),
            _ => {}
        }
    }
    Ok(())
}
