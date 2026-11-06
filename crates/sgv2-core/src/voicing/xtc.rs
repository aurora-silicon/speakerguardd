//! XTC's compiled stereo FIR matrix.
//!
//! The tuning reader turns the unit's frequency-domain ClassInfo state into
//! the finite filter which the saved state applies.  The real-time side only
//! sees four real tap vectors.  Keeping that transform out of this crate is
//! deliberate: this crate is `no_std`, contains no tuning, and does no work or
//! allocation while a parameter block is installed.

/// The longest XTC filter accepted from a tuning.  The public unit permits a
/// 20 ms optimizer length; at the supported 48 kHz rate that is 960 taps.
/// The small power-of-two margin also makes hostile parameter blocks bounded.
pub const MAX_TAPS: usize = 1024;
const MATRIX: usize = 4;
const HEADER_WORDS: usize = 2;
const MAX_ABS_TAP: f64 = 128.0;
const MAX_L1: f64 = 512.0;

/// A row-major 2x2 FIR matrix: `[LL, LR, RL, RR]`.
#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    pub taps: [[f64; MAX_TAPS]; MATRIX],
    pub count: usize,
    /// The latency reported by the AudioUnit.  This can differ from the
    /// largest tap when a ClassInfo snapshot carries a cached optimizer.
    pub reported_latency: usize,
}

impl Default for Params {
    fn default() -> Self {
        Self { taps: [[0.0; MAX_TAPS]; MATRIX], count: 0, reported_latency: 0 }
    }
}

impl Params {
    pub const WORDS: usize = HEADER_WORDS + MATRIX * MAX_TAPS;

    pub fn identity() -> Self {
        let mut p = Self { count: 1, ..Self::default() };
        p.taps[0][0] = 1.0;
        p.taps[3][0] = 1.0;
        p
    }

    fn valid(&self) -> bool {
        if self.count == 0 || self.count > MAX_TAPS || self.reported_latency > MAX_TAPS {
            return false;
        }
        self.taps.iter().all(|v| {
            let mut l1 = 0.0;
            for (i, x) in v.iter().enumerate() {
                if !x.is_finite() || x.abs() > MAX_ABS_TAP || (i >= self.count && *x != 0.0) {
                    return false;
                }
                if i < self.count {
                    l1 += x.abs();
                }
            }
            l1 <= MAX_L1
        })
    }

    /// Encodes exactly [`WORDS`](Self::WORDS) plain words.  A wrong-sized
    /// destination is left untouched.
    pub fn to_words(&self, out: &mut [f64]) {
        if out.len() != Self::WORDS {
            return;
        }
        out[0] = self.count as f64;
        out[1] = self.reported_latency as f64;
        for (m, taps) in self.taps.iter().enumerate() {
            let at = HEADER_WORDS + m * MAX_TAPS;
            out[at..at + MAX_TAPS].copy_from_slice(taps);
        }
    }

    pub fn from_words(words: &[f64]) -> Option<Self> {
        if words.len() != Self::WORDS {
            return None;
        }
        let integer = |x: f64| -> Option<usize> {
            if x.is_finite() && x >= 0.0 && x <= MAX_TAPS as f64 && x.fract() == 0.0 {
                Some(x as usize)
            } else {
                None
            }
        };
        let mut p = Self { count: integer(words[0])?, reported_latency: integer(words[1])?, ..Self::default() };
        for m in 0..MATRIX {
            let at = HEADER_WORDS + m * MAX_TAPS;
            p.taps[m].copy_from_slice(&words[at..at + MAX_TAPS]);
        }
        p.valid().then_some(p)
    }
}

/// Allocation-free direct convolution.  Shipped filters are 96 taps; the
/// bounded direct form is cheaper there than partition setup and has no block
/// latency.  All storage is made with the runner.
pub struct Run {
    params: Params,
    history: [[f64; MAX_TAPS]; 2],
    pos: usize,
    enabled: bool,
}

impl Run {
    pub fn new(params: &Params, fs: f64, channels: usize) -> Self {
        Self {
            params: params.clone(),
            history: [[0.0; MAX_TAPS]; 2],
            pos: 0,
            enabled: channels == 2 && fs.is_finite() && fs > 0.0 && params.valid(),
        }
    }

    pub fn reset(&mut self) {
        self.history = [[0.0; MAX_TAPS]; 2];
        self.pos = 0;
    }

    pub fn latency(&self) -> usize {
        if self.enabled {
            self.params.reported_latency
        } else {
            0
        }
    }

    /// Processes the first two buffers in place.  Short buffers, a channel
    /// mismatch, and non-finite samples are all handled without panicking.
    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        if !self.enabled || bufs.len() < 2 {
            return;
        }
        let (one, rest) = bufs.split_at_mut(1);
        let left = &mut *one[0];
        let right = &mut *rest[0];
        let n = frames.min(left.len()).min(right.len());
        let count = self.params.count;
        for frame in 0..n {
            let x0 = if left[frame].is_finite() { left[frame] as f64 } else { 0.0 };
            let x1 = if right[frame].is_finite() { right[frame] as f64 } else { 0.0 };
            self.history[0][self.pos] = x0;
            self.history[1][self.pos] = x1;
            let mut y = [0.0f64; 2];
            for k in 0..count {
                let at = (self.pos + MAX_TAPS - k) % MAX_TAPS;
                y[0] += self.params.taps[0][k] * self.history[0][at] + self.params.taps[1][k] * self.history[1][at];
                y[1] += self.params.taps[2][k] * self.history[0][at] + self.params.taps[3][k] * self.history[1][at];
            }
            left[frame] = finite_f32(y[0]);
            right[frame] = finite_f32(y[1]);
            self.pos += 1;
            if self.pos == MAX_TAPS {
                self.pos = 0;
            }
        }
    }
}

fn finite_f32(x: f64) -> f32 {
    if !x.is_finite() {
        0.0
    } else if x > f32::MAX as f64 {
        f32::MAX
    } else if x < -(f32::MAX as f64) {
        -f32::MAX
    } else {
        x as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn words_round_trip_and_refuse_hostile_values() {
        let mut p = Params::identity();
        p.reported_latency = 7;
        let mut words = [0.0; Params::WORDS];
        p.to_words(&mut words);
        assert_eq!(Params::from_words(&words), Some(p));
        words[2] = f64::NAN;
        assert!(Params::from_words(&words).is_none());
    }

    #[test]
    fn delayed_matrix_convolution_is_stereo_and_stateful() {
        let mut p = Params { count: 3, reported_latency: 2, ..Params::default() };
        p.taps[0][2] = 0.5;
        p.taps[1][2] = 0.25;
        p.taps[2][2] = -0.25;
        p.taps[3][2] = 0.5;
        let mut run = Run::new(&p, 48_000.0, 2);
        let (mut l0, mut r0) = ([1.0, 0.0], [0.0, 0.0]);
        run.process(&mut [&mut l0, &mut r0], 2);
        let (mut l1, mut r1) = ([0.0, 0.0], [0.0, 0.0]);
        run.process(&mut [&mut l1, &mut r1], 2);
        assert_eq!(l1, [0.5, 0.0]);
        assert_eq!(r1, [-0.25, 0.0]);
        assert_eq!(run.latency(), 2);
    }

    #[test]
    fn nonfinite_input_is_silence_and_wrong_channel_count_is_passthrough() {
        let p = Params::identity();
        let mut run = Run::new(&p, 48_000.0, 2);
        let (mut l, mut r) = ([f32::NAN], [1.0]);
        run.process(&mut [&mut l, &mut r], usize::MAX);
        assert_eq!(l, [0.0]);
        assert_eq!(r, [1.0]);

        let mut off = Run::new(&p, 48_000.0, 1);
        let mut mono = [2.0];
        off.process(&mut [&mut mono], 1);
        assert_eq!(mono, [2.0]);
    }
}
