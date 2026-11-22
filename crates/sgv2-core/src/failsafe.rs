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

    /// `fill_from` on one input/output pair, returning the output.
    fn fill(input: &[f32], out_len: usize, frames: usize, from: usize, gain: f32) -> std::vec::Vec<f32> {
        let mut o = std::vec![9.0f32; out_len];
        {
            let inputs: [&[f32]; 1] = [input];
            let mut outputs: [&mut [f32]; 1] = [&mut o];
            fill_from(&mut Io { frames, input: &inputs, sense: &[], output: &mut outputs }, from, gain);
        }
        o
    }

    #[test]
    fn the_gain_is_held_to_zero_to_one_and_a_nonfinite_gain_mutes() {
        let x = [0.5f32, -0.5, 1.0];
        assert_eq!(fill(&x, 3, 3, 0, 4.0), x, "above unity is unity");
        assert_eq!(fill(&x, 3, 3, 0, -0.5), [0.0; 3], "negative is silence");
        for g in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert_eq!(fill(&x, 3, 3, 0, g), [0.0; 3], "gain {g}");
        }
        assert_eq!(fill(&x, 3, 3, 0, 0.5), [0.25, -0.25, 0.5]);
    }

    #[test]
    fn any_input_gives_a_finite_output_bounded_by_the_gain() {
        let x = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 3.0, -1e30, f32::MAX, 0.05];
        let o = fill(&x, x.len(), x.len(), 0, FAILSAFE_GAIN);
        assert_eq!(&o[..3], &[0.0; 3], "a non-finite sample is muted, not clamped");
        assert_eq!(o[3], FAILSAFE_GAIN);
        assert_eq!(o[4], -FAILSAFE_GAIN);
        assert_eq!(o[5], FAILSAFE_GAIN);
        assert!((o[6] - 0.05 * FAILSAFE_GAIN).abs() < 1e-9);
    }

    #[test]
    fn only_frames_from_the_offset_to_the_quantum_end_are_written() {
        let x = [0.1f32, 0.2, 0.3, 0.4, 0.5, 0.6];
        // frames before `from` keep what the unit already wrote
        let o = fill(&x, 6, 6, 4, 1.0);
        assert_eq!(o, [9.0, 9.0, 9.0, 9.0, 0.5, 0.6]);
        // an offset at or past the end writes nothing
        assert_eq!(fill(&x, 6, 6, 6, 1.0), [9.0; 6]);
        assert_eq!(fill(&x, 6, 6, 100, 1.0), [9.0; 6]);
        // an output longer than the quantum is written only up to `frames`
        assert_eq!(fill(&x, 6, 3, 0, 1.0), [0.1, 0.2, 0.3, 9.0, 9.0, 9.0]);
    }

    #[test]
    fn short_buffers_never_index_out_of_range_and_the_unfed_tail_is_silent() {
        // input shorter than the quantum: the rest of the output is zero
        assert_eq!(fill(&[0.5, 0.5], 5, 5, 0, 1.0), [0.5, 0.5, 0.0, 0.0, 0.0]);
        // ... also when the offset is past the input's end
        assert_eq!(fill(&[0.5, 0.5], 5, 5, 3, 1.0), [9.0, 9.0, 9.0, 0.0, 0.0]);
        // output shorter than the quantum: written to its own end only
        assert_eq!(fill(&[0.5; 8], 3, 8, 0, 1.0), [0.5; 3]);
        // nothing at all
        assert_eq!(fill(&[], 0, 4, 0, 1.0), std::vec::Vec::<f32>::new());
        assert_eq!(fill(&[], 2, 2, 0, 1.0), [0.0; 2]);
    }

    #[test]
    fn telemetry_reports_the_fail_safe_and_its_reason() {
        let unit = FailSafe::new(UnitKind::Lsm2, Health::CHANNEL_MISMATCH);
        assert_eq!(unit.kind(), UnitKind::Lsm2);
        assert_eq!(unit.latency_frames(), 0);
        let mut t = Telemetry::new();
        t.channels[0].coil_temp_c = 50.0;
        unit.telemetry(&mut t);
        assert_eq!(t.health.0, Health::FAILSAFE | Health::CHANNEL_MISMATCH);
        assert!(t.channels.iter().all(|c| c.gain == FAILSAFE_GAIN));
        assert!(t.channels[0].coil_temp_c.is_nan(), "stale state is cleared");
    }
}
