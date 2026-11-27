// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The microphone blob: a versioned, CRC-checked encoding of [`Params`],
//! bound to the Mac it was compiled for. No native layout, pointers or
//! runtime state are serialized. Integers are u32 and floats IEEE f64,
//! little endian. Changing the layout requires a version bump. Decoding
//! allocates and belongs outside the real-time callback.
//!
//! ```text
//! 0   magic "SGV2MIC\0"          28  machine (8 bytes, ASCII, NUL-padded)
//! 8   version                    36  acoustic id
//! 12  flags (bit 0: partial)     40  unit (32 bytes: the unit hash, or zero)
//! 16  sample rate                72  payload length
//! 20  microphones                76  payload
//! 24  latency, frames            ..  CRC-32 of everything before it
//! ```
//!
//! `machine` is the board the speaker blob names (`crate::blob::machine_field`),
//! `unit` the raw SHA-256 the speaker bundle's `unit_id` spells in hex
//! (zero when the bundle names no unit). The payload holds the pre stages,
//! the transform, the beam and the post stages. A [`Stage::Graph`] is the
//! subset of a voicing chain Spec the microphone compiler emits: filter
//! nodes at a fixed state (sections, gain, level-dependent slots; no
//! volume automation) and sum nodes, with no graph links or relays.

use super::{Beam, Params, Stage, Stft, Window, MAX_MICS, MAX_STAGES};
use crate::blob::{crc32, BlobError, MACHINE_LEN};
use crate::prestage::{Channel, DynamicAt, DynamicFilter, DynamicKind, Section, MAX_DYNAMIC_FILTERS, MAX_SECTIONS};
use crate::voicing::chain::{Kind, NodeSpec, Source, Spec, Tone, MAX_NODES};
use crate::MAX_CHANNELS;
use alloc::vec::Vec;

pub const MAGIC: [u8; 8] = *b"SGV2MIC\0";
pub const VERSION: u32 = 1;
pub const HEADER_LEN: usize = 76;
pub const MAX_BYTES: usize = 4 * 1024 * 1024;
pub const FLAG_PARTIAL: u32 = 1;
pub const UNIT_LEN: usize = 32;

/// What the blob is bound to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Binding {
    pub machine: [u8; MACHINE_LEN],
    pub acoustic_id: u32,
    pub unit: [u8; UNIT_LEN],
}

impl Binding {
    /// The board, if the field holds one.
    pub fn machine(&self) -> Option<&str> {
        let end = self.machine.iter().position(|&b| b == 0).unwrap_or(MACHINE_LEN);
        if end == 0 {
            return None;
        }
        core::str::from_utf8(&self.machine[..end]).ok()
    }

    pub fn unit_bound(&self) -> bool {
        self.unit.iter().any(|&b| b != 0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub version: u32,
    pub flags: u32,
    pub sample_rate: u32,
    pub mics: u32,
    pub latency: u32,
    pub binding: Binding,
    pub payload_len: u32,
}

type Result<T> = core::result::Result<T, BlobError>;

struct Reader<'a>(&'a [u8]);

impl<'a> Reader<'a> {
    fn bytes(&mut self, n: usize) -> Result<&'a [u8]> {
        if self.0.len() < n {
            return Err(BlobError::Short);
        }
        let (a, b) = self.0.split_at(n);
        self.0 = b;
        Ok(a)
    }
    fn u32(&mut self) -> Result<u32> {
        let b = self.bytes(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
    fn count(&mut self, max: usize) -> Result<usize> {
        let n = self.u32()? as usize;
        if n > max {
            return Err(BlobError::Length);
        }
        Ok(n)
    }
    fn f64(&mut self) -> Result<f64> {
        let b = self.bytes(8)?;
        let v = f64::from_le_bytes(b.try_into().map_err(|_| BlobError::Short)?);
        if v.is_finite() {
            Ok(v)
        } else {
            Err(BlobError::Unstable)
        }
    }
    fn bool(&mut self) -> Result<bool> {
        match self.u32()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(BlobError::Length),
        }
    }
}

fn put_u32(o: &mut Vec<u8>, v: u32) {
    o.extend_from_slice(&v.to_le_bytes());
}

fn put_f64(o: &mut Vec<u8>, v: f64) {
    o.extend_from_slice(&v.to_le_bytes());
}

fn put_source(o: &mut Vec<u8>, src: &Source) {
    match *src {
        Source::Program(c) => {
            put_u32(o, 0);
            put_u32(o, c as u32);
        }
        Source::Node(n, c) => {
            put_u32(o, 1);
            put_u32(o, n as u32);
            put_u32(o, c as u32);
        }
    }
}

fn get_source(r: &mut Reader<'_>) -> Result<Source> {
    Ok(match r.u32()? {
        0 => Source::Program(r.count(MAX_CHANNELS)?),
        1 => Source::Node(r.count(MAX_NODES)?, r.count(MAX_CHANNELS)?),
        _ => return Err(BlobError::Length),
    })
}

/// Whether a chain is in the encodable subset (see the module notes).
pub fn graph_encodable(spec: &Spec) -> bool {
    spec.links.is_empty()
        && spec.relays.is_empty()
        && spec.nodes.iter().all(|n| match &n.kind {
            Kind::Tone(t) => t.automation.is_empty() && t.dynamic_automation.is_empty(),
            Kind::Sum(_) => true,
            _ => false,
        })
}

fn put_spec(o: &mut Vec<u8>, spec: &Spec) {
    put_u32(o, spec.program_channels as u32);
    put_u32(o, spec.nodes.len() as u32);
    for n in &spec.nodes {
        put_u32(o, n.inputs.len() as u32);
        for src in &n.inputs {
            put_source(o, src);
        }
        match &n.kind {
            Kind::Sum(w) => {
                put_u32(o, 1);
                put_u32(o, *w as u32);
            }
            Kind::Tone(t) => {
                put_u32(o, 2);
                for c in &t.channels {
                    put_u32(o, c.count as u32);
                    for sec in c.sections() {
                        for v in [sec.b0, sec.b1, sec.b2, sec.a1, sec.a2] {
                            put_f64(o, v);
                        }
                    }
                    put_f64(o, c.gain);
                }
                put_u32(o, t.dynamic.len() as u32);
                for d in &t.dynamic {
                    put_u32(o, d.before_box as u32);
                    for v in d.split {
                        put_u32(o, v as u32);
                    }
                    put_u32(o, d.active_mask);
                    put_u32(o, d.sidechain_mask);
                    let p = &d.params;
                    put_u32(o, p.kind as u32);
                    for v in [p.range_db, p.frequency_hz, p.quality, p.ratio, p.threshold_db, p.attack_ms, p.release_ms]
                    {
                        put_f64(o, v);
                    }
                    put_u32(o, p.smooth as u32);
                }
            }
            // never written: `encode` checks `graph_encodable` first
            _ => put_u32(o, 0),
        }
    }
    put_u32(o, spec.outputs.len() as u32);
    for src in &spec.outputs {
        put_source(o, src);
    }
}

fn get_spec(r: &mut Reader<'_>, fs: f64) -> Result<Spec> {
    let program_channels = r.count(MAX_CHANNELS)?;
    let count = r.count(MAX_NODES)?;
    let mut nodes = Vec::with_capacity(count);
    for _ in 0..count {
        let width = r.count(MAX_CHANNELS)?;
        let inputs = (0..width).map(|_| get_source(r)).collect::<Result<Vec<_>>>()?;
        let kind = match r.u32()? {
            1 => Kind::Sum(r.count(MAX_CHANNELS)?),
            2 => {
                let mut channels = Vec::with_capacity(width);
                for _ in 0..width {
                    let mut c = Channel::IDENTITY;
                    c.count = r.count(MAX_SECTIONS)?;
                    for sec in c.sections.iter_mut().take(c.count) {
                        *sec = Section { b0: r.f64()?, b1: r.f64()?, b2: r.f64()?, a1: r.f64()?, a2: r.f64()? };
                    }
                    c.gain = r.f64()?;
                    channels.push(c);
                }
                let nd = r.count(MAX_DYNAMIC_FILTERS)?;
                let mut dynamic = Vec::with_capacity(nd);
                for _ in 0..nd {
                    let mut d = DynamicAt { before_box: r.count(MAX_NODES)?, ..Default::default() };
                    for v in d.split.iter_mut() {
                        *v = r.count(MAX_SECTIONS)?;
                    }
                    d.active_mask = r.u32()?;
                    d.sidechain_mask = r.u32()?;
                    let kind = match r.u32()? {
                        15 => DynamicKind::Notch,
                        17 => DynamicKind::LowShelf,
                        18 => DynamicKind::HighShelf,
                        _ => return Err(BlobError::Length),
                    };
                    d.params = DynamicFilter {
                        kind,
                        range_db: r.f64()?,
                        frequency_hz: r.f64()?,
                        quality: r.f64()?,
                        ratio: r.f64()?,
                        threshold_db: r.f64()?,
                        attack_ms: r.f64()?,
                        release_ms: r.f64()?,
                        smooth: r.bool()?,
                    };
                    if !d.params.valid(fs) {
                        return Err(BlobError::Unstable);
                    }
                    dynamic.push(d);
                }
                Kind::Tone(Tone { channels, automation: Vec::new(), dynamic, dynamic_automation: Vec::new() })
            }
            _ => return Err(BlobError::Length),
        };
        nodes.push(NodeSpec { kind, inputs });
    }
    let n_out = r.count(MAX_CHANNELS)?;
    let outputs = (0..n_out).map(|_| get_source(r)).collect::<Result<Vec<_>>>()?;
    let spec =
        Spec { program_channels, nodes, outputs, links: Vec::new(), relays: Vec::new(), volume: Default::default() };
    spec.check().map_err(|_| BlobError::Unstable)?;
    Ok(spec)
}

fn put_stages(o: &mut Vec<u8>, stages: &[Stage]) {
    put_u32(o, stages.len() as u32);
    for st in stages {
        match st {
            Stage::Gain { db } => {
                put_u32(o, 1);
                put_f64(o, *db);
            }
            Stage::DcBlock { cutoff_hz, ramp_ms, gain_db } => {
                put_u32(o, 2);
                put_f64(o, *cutoff_hz);
                put_f64(o, *ramp_ms);
                put_f64(o, *gain_db);
            }
            Stage::Graph(spec) => {
                put_u32(o, 3);
                put_spec(o, spec);
            }
            Stage::Identity(code) => {
                put_u32(o, 4);
                o.extend_from_slice(code);
            }
        }
    }
}

fn get_stages(r: &mut Reader<'_>, fs: f64) -> Result<Vec<Stage>> {
    let n = r.count(MAX_STAGES)?;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        out.push(match r.u32()? {
            1 => Stage::Gain { db: r.f64()? },
            2 => Stage::DcBlock { cutoff_hz: r.f64()?, ramp_ms: r.f64()?, gain_db: r.f64()? },
            3 => Stage::Graph(get_spec(r, fs)?),
            4 => {
                let b = r.bytes(4)?;
                Stage::Identity([b[0], b[1], b[2], b[3]])
            }
            _ => return Err(BlobError::Length),
        });
    }
    Ok(out)
}

fn put_payload(o: &mut Vec<u8>, p: &Params) {
    put_stages(o, &p.pre);
    let s = &p.stft;
    put_u32(o, s.hop as u32);
    put_u32(o, s.size as u32);
    put_u32(o, s.window as u32);
    put_u32(o, s.half_offset as u32);
    put_u32(o, s.frame_phase as u32);
    let b = &p.beam;
    put_u32(o, b.mics as u32);
    put_u32(o, b.bins as u32);
    for w in &b.weights {
        put_f64(o, w[0]);
        put_f64(o, w[1]);
    }
    put_u32(o, b.eq.is_some() as u32);
    if let Some(eq) = &b.eq {
        for v in eq {
            put_f64(o, *v);
        }
    }
    put_stages(o, &p.post);
}

fn get_payload(r: &mut Reader<'_>, h: &Header) -> Result<Params> {
    let fs = h.sample_rate as f64;
    let pre = get_stages(r, fs)?;
    let hop = r.count(super::MAX_HOP)?;
    let size = r.count(2 * super::MAX_HOP)?;
    let window = Window::from_u32(r.u32()?).ok_or(BlobError::Length)?;
    let half_offset = r.bool()?;
    let frame_phase = r.count(super::MAX_HOP)?;
    let mics = r.count(MAX_MICS)?;
    let bins = r.count(super::MAX_HOP + 1)?;
    if mics != h.mics as usize || bins != size / 2 + 1 {
        return Err(BlobError::Channels);
    }
    let mut weights = Vec::with_capacity(mics * bins);
    for _ in 0..mics * bins {
        weights.push([r.f64()?, r.f64()?]);
    }
    let eq = if r.bool()? { Some((0..bins).map(|_| r.f64()).collect::<Result<Vec<_>>>()?) } else { None };
    let post = get_stages(r, fs)?;
    Ok(Params {
        sample_rate: h.sample_rate,
        mics,
        pre,
        stft: Stft { hop, size, window, half_offset, frame_phase },
        beam: Beam { mics, bins, weights, eq },
        post,
        partial: h.flags & FLAG_PARTIAL != 0,
    })
}

/// A checked, bound blob. Refuses parameters the engine would refuse.
pub fn encode(p: &Params, binding: &Binding) -> Result<Vec<u8>> {
    let graphs = p.pre.iter().chain(&p.post).filter_map(|st| if let Stage::Graph(g) = st { Some(g) } else { None });
    if !p.valid() || !graphs.into_iter().all(graph_encodable) {
        return Err(BlobError::Unstable);
    }
    let mut payload = Vec::new();
    put_payload(&mut payload, p);
    let mut out = Vec::with_capacity(HEADER_LEN + payload.len() + 4);
    out.extend_from_slice(&MAGIC);
    put_u32(&mut out, VERSION);
    put_u32(&mut out, if p.partial { FLAG_PARTIAL } else { 0 });
    put_u32(&mut out, p.sample_rate);
    put_u32(&mut out, p.mics as u32);
    put_u32(&mut out, p.latency() as u32);
    out.extend_from_slice(&binding.machine);
    put_u32(&mut out, binding.acoustic_id);
    out.extend_from_slice(&binding.unit);
    put_u32(&mut out, payload.len() as u32);
    out.extend_from_slice(&payload);
    if out.len() > MAX_BYTES - 4 {
        return Err(BlobError::Length);
    }
    let crc = crc32(&out);
    put_u32(&mut out, crc);
    // exercise exactly the deployed decoder before handing it out
    decode(&out)?;
    Ok(out)
}

/// The header alone, after the CRC check.
pub fn header(bytes: &[u8]) -> Result<Header> {
    if bytes.len() < HEADER_LEN + 4 {
        return Err(BlobError::Short);
    }
    if bytes.len() > MAX_BYTES {
        return Err(BlobError::Length);
    }
    let (data, crc) = bytes.split_at(bytes.len() - 4);
    if crc32(data) != u32::from_le_bytes([crc[0], crc[1], crc[2], crc[3]]) {
        return Err(BlobError::Crc);
    }
    let mut r = Reader(data);
    if r.bytes(8)? != MAGIC {
        return Err(BlobError::Magic);
    }
    let version = r.u32()?;
    if version != VERSION {
        return Err(BlobError::Version);
    }
    let flags = r.u32()?;
    let sample_rate = r.u32()?;
    let mics = r.u32()?;
    let latency = r.u32()?;
    let machine: [u8; MACHINE_LEN] = r.bytes(MACHINE_LEN)?.try_into().map_err(|_| BlobError::Short)?;
    let acoustic_id = r.u32()?;
    let unit: [u8; UNIT_LEN] = r.bytes(UNIT_LEN)?.try_into().map_err(|_| BlobError::Short)?;
    let payload_len = r.u32()?;
    if flags & !FLAG_PARTIAL != 0 || sample_rate == 0 || mics == 0 || mics as usize > MAX_MICS {
        return Err(BlobError::Channels);
    }
    if r.0.len() != payload_len as usize {
        return Err(BlobError::Length);
    }
    Ok(Header {
        version,
        flags,
        sample_rate,
        mics,
        latency,
        binding: Binding { machine, acoustic_id, unit },
        payload_len,
    })
}

/// The whole blob: header, CRC, payload, and the parameters' validity.
pub fn decode(bytes: &[u8]) -> Result<(Header, Params)> {
    let h = header(bytes)?;
    let mut r = Reader(&bytes[HEADER_LEN..bytes.len() - 4]);
    let p = get_payload(&mut r, &h)?;
    if !r.0.is_empty() {
        return Err(BlobError::Length);
    }
    if !p.valid() || p.latency() != h.latency as usize {
        return Err(BlobError::Unstable);
    }
    Ok((h, p))
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    fn params() -> Params {
        let mut p = super::super::tests::unit_params(3, 480);
        p.pre.push(Stage::DcBlock { cutoff_hz: 40.0, ramp_ms: 20.0, gain_db: 0.0 });
        p.pre.push(Stage::Identity(*b"sens"));
        p.pre.push(Stage::Gain { db: 12.0 });
        p.partial = true;
        let mut ch = Channel::IDENTITY;
        ch.count = 1;
        ch.sections[0] = crate::voicing::tone::peak(-3.0, 1000.0, 1.0, 48_000.0);
        assert!(ch.sections[0] != Section::IDENTITY);
        let spec = Spec {
            program_channels: 1,
            nodes: vec![NodeSpec {
                kind: Kind::Tone(Tone {
                    channels: vec![ch],
                    automation: vec![],
                    dynamic: vec![],
                    dynamic_automation: vec![],
                }),
                inputs: vec![Source::Program(0)],
            }],
            outputs: vec![Source::Node(0, 0)],
            links: vec![],
            relays: vec![],
            volume: Default::default(),
        };
        p.post.push(Stage::Graph(spec));
        // a nested graph: a filter node with a level-dependent notch on all
        // three channels, then channel 2 replaced by a sum of 0 and 2
        let dynamic = DynamicAt {
            active_mask: 7,
            sidechain_mask: 7,
            params: DynamicFilter {
                kind: DynamicKind::Notch,
                range_db: -6.0,
                frequency_hz: 5000.0,
                quality: 2.0,
                ratio: 2.0,
                threshold_db: -30.0,
                attack_ms: 10.0,
                release_ms: 100.0,
                smooth: false,
            },
            ..Default::default()
        };
        let nest = Spec {
            program_channels: 3,
            nodes: vec![
                NodeSpec {
                    kind: Kind::Tone(Tone {
                        channels: vec![Channel::IDENTITY; 3],
                        automation: vec![],
                        dynamic: vec![dynamic],
                        dynamic_automation: vec![],
                    }),
                    inputs: vec![Source::Program(0), Source::Program(1), Source::Program(2)],
                },
                NodeSpec { kind: Kind::Sum(1), inputs: vec![Source::Node(0, 0), Source::Node(0, 2)] },
            ],
            outputs: vec![Source::Node(0, 0), Source::Node(0, 1), Source::Node(1, 0)],
            links: vec![],
            relays: vec![],
            volume: Default::default(),
        };
        p.pre.insert(0, Stage::Graph(nest));
        p.beam.eq = Some(vec![0.5; 481]);
        p
    }

    fn binding() -> Binding {
        Binding { machine: crate::blob::machine_field("j000").unwrap(), acoustic_id: 99, unit: [7; UNIT_LEN] }
    }

    #[test]
    fn round_trips_and_binds() {
        let p = params();
        let b = encode(&p, &binding()).unwrap();
        let (h, q) = decode(&b).unwrap();
        assert_eq!(q, p);
        assert_eq!(h.binding, binding());
        assert_eq!(h.binding.machine(), Some("j000"));
        assert!(h.binding.unit_bound());
        assert_eq!(h.flags, FLAG_PARTIAL);
        assert_eq!(h.latency, 959);
        assert!(super::super::Engine::new(q).is_some());
    }

    #[test]
    fn every_corruption_is_refused() {
        let b = encode(&params(), &binding()).unwrap();
        for i in (0..b.len()).step_by(37) {
            let mut c = b.clone();
            c[i] ^= 0x10;
            assert!(decode(&c).is_err(), "byte {i}");
        }
        assert!(decode(&b[..b.len() - 1]).is_err());
        assert_eq!(decode(&[]), Err(BlobError::Short));
    }

    #[test]
    fn invalid_params_are_not_encoded() {
        let mut p = params();
        p.partial = false;
        assert_eq!(encode(&p, &binding()), Err(BlobError::Unstable));
    }
}
