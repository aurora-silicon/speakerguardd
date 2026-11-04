//! The fail-safe unit: a fixed attenuation on every channel.
//!
//! This is what the core does whenever it cannot protect from valid
//! parameters, and what every unit does until its behaviour is implemented.

use crate::{Health, Io, ProtectionUnit, Telemetry, UnitKind, FAILSAFE_GAIN};

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
    pub fn apply(gain: f32, io: Io<'_, '_>) {
        let frames = io.frames;
        for (ch, out) in io.output.iter_mut().enumerate() {
            let out = &mut out[..frames];
            match io.input.get(ch) {
                Some(input) => {
                    for (o, i) in out.iter_mut().zip(&input[..frames]) {
                        *o = *i * gain;
                    }
                }
                None => out.fill(0.0),
            }
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
