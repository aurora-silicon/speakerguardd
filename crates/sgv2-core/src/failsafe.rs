//! The fail-safe unit: a fixed attenuation on every channel.
//!
//! This is what the core does whenever it cannot protect from valid
//! parameters, and what every unit does until its behaviour is implemented.

use crate::{dsp, Health, Io, ProtectionUnit, Telemetry, UnitKind, FAILSAFE_GAIN};

pub struct FailSafe {
    kind: UnitKind,
    gain: f32,
    reason: u32,
}

impl FailSafe {
    pub fn new(kind: UnitKind, reason: u32) -> Self {
        Self { kind, gain: FAILSAFE_GAIN, reason }
    }

    /// Apply the fail-safe gain to `io`. Shared by every unit's fault paths.
    #[inline]
    pub fn apply(gain: f32, mut io: Io<'_, '_>) {
        fill_from(&mut io, 0, gain);
    }
}

/// The fail-safe output for frames `from..io.frames` of every output, on
/// whatever geometry the host gave and never indexing past a buffer:
/// `y = gain * clamp(x, -1, 1)`, a non-finite sample muted, an output with
/// no input silent. The result is finite and at most `gain` (itself held
/// to [0, 1]) in magnitude for any input, so the converter never sees a
/// non-finite or over-range fail-safe sample.
pub fn fill_from(io: &mut Io<'_, '_>, from: usize, gain: f32) {
    let g = if gain.is_finite() { gain.clamp(0.0, 1.0) } else { 0.0 };
    let frames = io.frames;
    for (c, out) in io.output.iter_mut().enumerate() {
        let n = frames.min(out.len());
        let s = from.min(n);
        let out = &mut out[s..n];
        match io.input.get(c) {
            Some(inp) => {
                let src = inp.get(s..n.min(inp.len())).unwrap_or(&[]);
                for (o, &x) in out.iter_mut().zip(src) {
                    *o = dsp::bound1(x) * g;
                }
                out[src.len()..].fill(0.0);
            }
            None => out.fill(0.0),
        }
    }
}

impl ProtectionUnit for FailSafe {
    fn kind(&self) -> UnitKind {
        self.kind
    }
    fn latency_frames(&self) -> u32 {
        0
    }
    fn process(&mut self, io: Io<'_, '_>) -> Health {
        Self::apply(self.gain, io);
        Health(Health::FAILSAFE | self.reason)
    }
    fn telemetry(&self, out: &mut Telemetry) {
        out.clear();
        out.health = Health(Health::FAILSAFE | self.reason);
        for ch in out.channels.iter_mut() {
            ch.gain = self.gain;
        }
    }
    fn reset(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn attenuates_by_exactly_the_failsafe_amount_and_zeroes_unfed_outputs() {
        let a = [1.0f32, -0.5, 0.25];
        let mut o0 = [9.0f32; 3];
        let mut o1 = [9.0f32; 3];
        let input: [&[f32]; 1] = [&a];
        let mut output: [&mut [f32]; 2] = [&mut o0, &mut o1];
        let mut unit = FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS);
        let health = unit.process(Io { frames: 3, input: &input, sense: &[], output: &mut output });
        assert!(health.is_failsafe());
        assert!((o0[0] - 0.1).abs() < 1e-7 && (o0[1] + 0.05).abs() < 1e-7);
        assert_eq!(o1, [0.0; 3]);
    }
}
