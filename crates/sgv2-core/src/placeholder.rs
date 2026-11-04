//! A test unit: per-channel gain, no protection, no latency. It exists so
//! the graph, the sense ingestion and the benchmarks can be exercised
//! before a protection unit is qualified. It is never selected from a
//! Mac's tuning, and its default gain is the fail-safe level, so a
//! placeholder installed by mistake is never louder than the fail-safe.

use crate::blob::{BlobError, Pod};
use crate::{dsp, ChannelState, Health, Io, ProtectionUnit, Telemetry, UnitKind, FAILSAFE_GAIN, MAX_CHANNELS};

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    pub channels: u32,
    /// Linear gain per channel, clamped to [0, 1] at construction.
    pub gain: [f32; MAX_CHANNELS],
}

// SAFETY: repr(C), nine four-byte fields, no padding.
unsafe impl Pod for Params {}

impl Default for Params {
    fn default() -> Self {
        Params { channels: 0, gain: [FAILSAFE_GAIN; MAX_CHANNELS] }
    }
}

impl Params {
    pub fn decode(payload: &[u8]) -> Result<Self, BlobError> {
        let p = Self::from_bytes(payload)?;
        if p.channels == 0 || p.channels as usize > MAX_CHANNELS {
            return Err(BlobError::Channels);
        }
        Ok(p)
    }
}

pub struct Unit {
    n: usize,
    gain: [f32; MAX_CHANNELS],
    frame: u64,
}

impl Unit {
    pub fn new(p: Params) -> Self {
        let mut gain = [0.0; MAX_CHANNELS];
        for (g, &v) in gain.iter_mut().zip(&p.gain) {
            // never above unity, NaN becomes silence
            *g = if v.is_finite() { v.clamp(0.0, 1.0) } else { 0.0 };
        }
        Unit { n: (p.channels as usize).min(MAX_CHANNELS), gain, frame: 0 }
    }
}

impl ProtectionUnit for Unit {
    fn kind(&self) -> UnitKind {
        // Reported as spp3 so hosts treat it like the unit it stands in for.
        UnitKind::Spp3
    }
    fn latency_frames(&self) -> u32 {
        0
    }
    fn process(&mut self, io: Io<'_, '_>) -> Health {
        let frames = io.frames;
        let ok = io.input.len() == self.n
            && io.output.len() == self.n
            && io.input.iter().all(|b| b.len() >= frames)
            && io.output.iter().all(|b| b.len() >= frames);
        for (c, out) in io.output.iter_mut().enumerate() {
            let n = frames.min(out.len());
            let g = if ok { self.gain[c.min(MAX_CHANNELS - 1)] } else { FAILSAFE_GAIN.min(self.gain[0]) };
            match io.input.get(c) {
                Some(inp) if inp.len() >= n => {
                    dsp::scale_to(&inp[..n], g, &mut out[..n]);
                    dsp::sanitize(&mut out[..n]);
                }
                _ => out[..n].fill(0.0),
            }
        }
        self.frame += frames as u64;
        if ok {
            Health::OK
        } else {
            Health(Health::FAILSAFE | Health::CHANNEL_MISMATCH)
        }
    }
    fn telemetry(&self, out: &mut Telemetry) {
        out.clear();
        out.frame = self.frame;
        for c in 0..self.n {
            out.channels[c] = ChannelState { gain: self.gain[c], ..ChannelState::ABSENT };
        }
    }
    fn reset(&mut self) {
        self.frame = 0;
    }
}
