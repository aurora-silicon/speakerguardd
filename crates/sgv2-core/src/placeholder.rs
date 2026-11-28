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
                    for (o, &x) in out[..n].iter_mut().zip(&inp[..n]) {
                        *o = dsp::bound1(x) * g;
                    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blob::Pod;

    fn params(channels: u32, gain: f32) -> Params {
        Params { channels, gain: [gain; MAX_CHANNELS] }
    }

    fn run(
        u: &mut Unit,
        inputs: &[&[f32]],
        outputs: usize,
        frames: usize,
    ) -> (Health, std::vec::Vec<std::vec::Vec<f32>>) {
        let mut out: std::vec::Vec<std::vec::Vec<f32>> = (0..outputs).map(|_| std::vec![9.0; frames]).collect();
        let health = {
            let mut o: std::vec::Vec<&mut [f32]> = out.iter_mut().map(|b| b.as_mut_slice()).collect();
            u.process(Io { frames, input: inputs, sense: &[], output: &mut o })
        };
        (health, out)
    }

    #[test]
    fn the_default_gain_is_the_fail_safe_level() {
        let p = Params::default();
        assert!(p.gain.iter().all(|g| *g == FAILSAFE_GAIN));
        assert_eq!(p.channels, 0);
    }

    #[test]
    fn decode_refuses_a_wrong_length_and_a_channel_count_out_of_range() {
        let good = params(2, 0.05);
        assert_eq!(Params::decode(good.as_bytes()), Ok(good));
        let b = good.as_bytes();
        assert_eq!(Params::decode(&b[..b.len() - 1]), Err(BlobError::Length));
        assert_eq!(Params::decode(&[]), Err(BlobError::Length));
        for n in [0, MAX_CHANNELS as u32 + 1, u32::MAX] {
            assert_eq!(Params::decode(params(n, 0.05).as_bytes()), Err(BlobError::Channels), "{n} channels");
        }
        assert!(Params::decode(params(MAX_CHANNELS as u32, 0.05).as_bytes()).is_ok());
    }

    #[test]
    fn a_gain_above_unity_is_unity_and_a_negative_or_non_finite_gain_is_silence() {
        let mut p = params(MAX_CHANNELS as u32, 0.0);
        p.gain = [4.0, -0.5, f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 0.25, 1.0, 0.0];
        let u = Unit::new(p);
        let mut t = Telemetry::new();
        u.telemetry(&mut t);
        let gains: std::vec::Vec<f32> = t.channels.iter().map(|c| c.gain).collect();
        assert_eq!(gains, [1.0, 0.0, 0.0, 0.0, 0.0, 0.25, 1.0, 0.0]);
    }

    #[test]
    fn matching_geometry_applies_each_channels_gain_to_the_bounded_input() {
        let mut p = params(2, 0.0);
        p.gain[0] = 0.5;
        p.gain[1] = 2.0;
        let mut u = Unit::new(p);
        let a = [0.5f32, 4.0, f32::NAN];
        let b = [-0.5f32, -4.0, f32::INFINITY];
        let (h, out) = run(&mut u, &[&a, &b], 2, 3);
        assert_eq!(h, Health::OK);
        assert_eq!(out[0], [0.25, 0.5, 0.0]);
        assert_eq!(out[1], [-0.5, -1.0, 0.0]);
    }

    #[test]
    fn a_geometry_mismatch_is_the_fail_safe_and_never_louder_than_it() {
        let mut u = Unit::new(params(2, 1.0));
        let a = [1.0f32; 4];
        let mismatch = Health(Health::FAILSAFE | Health::CHANNEL_MISMATCH);
        // one input and one output for a two-channel unit
        let (h, out) = run(&mut u, &[&a], 1, 4);
        assert_eq!(h, mismatch);
        assert_eq!(out[0], [FAILSAFE_GAIN; 4]);
        // more outputs than inputs: the unfed outputs are silent
        let (h, out) = run(&mut u, &[&a, &a], 3, 4);
        assert_eq!(h, mismatch);
        assert_eq!(out[0], [FAILSAFE_GAIN; 4]);
        assert_eq!(out[2], [0.0; 4]);
        // an input shorter than the quantum is silent, not read past its end
        let short = [1.0f32; 2];
        let (h, out) = run(&mut u, &[&a, &short], 2, 4);
        assert_eq!(h, mismatch);
        assert_eq!(out[0], [FAILSAFE_GAIN; 4]);
        assert_eq!(out[1], [0.0; 4]);
        // a quieter configured gain stays quieter in the fail-safe
        let mut quiet = Unit::new(params(2, 0.01));
        let (_, out) = run(&mut quiet, &[&a], 1, 4);
        assert_eq!(out[0], [0.01; 4]);
    }

    #[test]
    fn telemetry_counts_frames_until_reset_and_leaves_unused_channels_absent() {
        let mut u = Unit::new(params(2, 0.5));
        let a = [0.1f32; 160];
        run(&mut u, &[&a, &a], 2, 160);
        run(&mut u, &[&a, &a], 2, 96);
        let mut t = Telemetry::new();
        u.telemetry(&mut t);
        assert_eq!(t.frame, 256);
        assert_eq!(t.health, Health::OK);
        assert_eq!(t.channels[1].gain, 0.5);
        assert!(t.channels[1].coil_temp_c.is_nan());
        assert!(t.channels[2].gain.is_nan());
        u.reset();
        u.telemetry(&mut t);
        assert_eq!(t.frame, 0);
        assert_eq!(u.kind(), UnitKind::Spp3);
        assert_eq!(u.latency_frames(), 0);
    }
}
