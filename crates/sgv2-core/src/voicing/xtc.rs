//! XTC's compiled stereo FIR matrix.
//!
//! The tuning reader turns the unit's frequency-domain ClassInfo state into
//! the finite matrices applied during startup, after settling, and on reset.
//! The real-time side only sees real taps. Keeping that transform out of this
//! crate is deliberate: this crate is `no_std`, contains no tuning, and does
//! no allocation in `process` or `reset`.

use alloc::boxed::Box;

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
    /// Filters queued while ClassInfo and the stream format are installed.
    /// The fixed-filter word codec remains the legacy codec; deployment
    /// artifacts encode this optional state with a separate node tag.
    pub startup: Option<Box<Startup>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Startup {
    pub initial: Box<Params>,
    pub restored: Box<Params>,
    pub crossfade_frames: usize,
}

impl Default for Params {
    fn default() -> Self {
        Self { taps: [[0.0; MAX_TAPS]; MATRIX], count: 0, reported_latency: 0, startup: None }
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

    /// Attach the two finite filters queued before the settled cached FIR.
    pub fn with_startup(mut self, initial: Self, restored: Self, crossfade_frames: usize) -> Option<Self> {
        if self.startup.is_some()
            || initial.startup.is_some()
            || restored.startup.is_some()
            || !self.valid()
            || !initial.valid()
            || !restored.valid()
            || !(1..=192_000).contains(&crossfade_frames)
        {
            return None;
        }
        self.startup =
            Some(Box::new(Startup { initial: Box::new(initial), restored: Box::new(restored), crossfade_frames }));
        Some(self)
    }

    pub(crate) fn valid(&self) -> bool {
        if self.startup.as_ref().is_some_and(|s| {
            !(1..=192_000).contains(&s.crossfade_frames)
                || s.initial.startup.is_some()
                || s.restored.startup.is_some()
                || !s.initial.valid()
                || !s.restored.valid()
        }) {
            return false;
        }
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

    /// Encodes the settled filter in exactly [`WORDS`](Self::WORDS) legacy
    /// words. Startup is carried separately by the deployment artifact.
    /// A wrong-sized destination is left untouched.
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
            if x.is_finite() && x >= 0.0 && x <= MAX_TAPS as f64 && libm::trunc(x) == x {
                Some(x as usize)
            } else {
                None
            }
        };
        let mut p = Self { count: integer(words[0])?, reported_latency: integer(words[1])?, ..Self::default() };
        for (m, taps) in p.taps.iter_mut().enumerate() {
            let at = HEADER_WORDS + m * MAX_TAPS;
            taps.copy_from_slice(&words[at..at + MAX_TAPS]);
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
    phase: u8,
    fade_at: usize,
    cached_pending: bool,
}

impl Run {
    pub fn new(params: &Params, fs: f64, channels: usize) -> Self {
        Self {
            params: params.clone(),
            history: [[0.0; MAX_TAPS]; 2],
            pos: 0,
            phase: if params.startup.is_some() { 0 } else { 2 },
            fade_at: 0,
            cached_pending: params.startup.is_some(),
            enabled: channels == 2 && fs.is_finite() && fs > 0.0 && params.valid(),
        }
    }

    pub fn reset(&mut self) {
        self.history = [[0.0; MAX_TAPS]; 2];
        self.pos = 0;
        // Reset realizes the restored controls immediately. A cached FIR
        // still waiting in the queue survives; an already installed one
        // does not. Repeated resets before rendering do not consume it.
        self.phase = if self.params.startup.is_none() {
            2
        } else if self.cached_pending {
            1
        } else {
            3
        };
        self.fade_at = 0;
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
        // The unit subdivides larger renders into at most 512 frames. A
        // completed fade hands its queued successor over at that boundary.
        for start in (0..n).step_by(512) {
            let end = (start + 512).min(n);
            if let Some(startup) = &self.params.startup {
                if self.phase < 2 && self.fade_at > startup.crossfade_frames {
                    self.phase += 1;
                    self.fade_at = 0;
                }
            }
            if self.phase == 1 {
                self.cached_pending = false;
            }
            for frame in start..end {
                let x0 = if left[frame].is_finite() { left[frame] as f64 } else { 0.0 };
                let x1 = if right[frame].is_finite() { right[frame] as f64 } else { 0.0 };
                self.history[0][self.pos] = x0;
                self.history[1][self.pos] = x1;
                let y = if let Some(startup) = &self.params.startup {
                    let (from, to) = match self.phase {
                        0 => (&*startup.initial, &*startup.restored),
                        1 => (&*startup.restored, &self.params),
                        3 => (&*startup.restored, &*startup.restored),
                        _ => (&self.params, &self.params),
                    };
                    let a = self.convolve(from);
                    if self.phase < 2 {
                        let b = self.convolve(to);
                        let mix = self.fade_at.min(startup.crossfade_frames) as f64 / startup.crossfade_frames as f64;
                        self.fade_at = self.fade_at.saturating_add(1);
                        [a[0] + mix * (b[0] - a[0]), a[1] + mix * (b[1] - a[1])]
                    } else {
                        a
                    }
                } else {
                    self.convolve(&self.params)
                };
                left[frame] = finite_f32(y[0]);
                right[frame] = finite_f32(y[1]);
                self.pos = (self.pos + 1) % MAX_TAPS;
            }
        }
    }

    fn convolve(&self, params: &Params) -> [f64; 2] {
        let mut y = [0.0; 2];
        for k in 0..params.count {
            let at = (self.pos + MAX_TAPS - k) % MAX_TAPS;
            y[0] += params.taps[0][k] * self.history[0][at] + params.taps[1][k] * self.history[1][at];
            y[1] += params.taps[2][k] * self.history[0][at] + params.taps[3][k] * self.history[1][at];
        }
        y
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

    fn scaled_identity(gain: f64) -> Params {
        let mut p = Params::identity();
        p.taps[0][0] = gain;
        p.taps[3][0] = gain;
        p
    }

    #[test]
    fn queued_fades_wait_for_render_boundaries_and_reset_realizes_restored_filter() {
        let p = scaled_identity(0.25).with_startup(scaled_identity(1.0), scaled_identity(0.5), 4).unwrap();
        let mut run = Run::new(&p, 48_000.0, 2);
        let (mut l, mut r) = ([1.0; 6], [1.0; 6]);
        run.process(&mut [&mut l, &mut r], 6);
        assert_eq!(l, [1.0, 0.875, 0.75, 0.625, 0.5, 0.5]);
        assert_eq!(l, r);
        let (mut l, mut r) = ([1.0; 6], [1.0; 6]);
        run.process(&mut [&mut l, &mut r], 6);
        assert_eq!(l, [0.5, 0.4375, 0.375, 0.3125, 0.25, 0.25]);
        run.reset();
        let (mut l, mut r) = ([1.0; 6], [1.0; 6]);
        run.process(&mut [&mut l, &mut r], 6);
        assert_eq!(l, [0.5; 6]);
    }

    #[test]
    fn long_renders_subdivide_the_crossfade_queue() {
        let p = scaled_identity(0.25).with_startup(scaled_identity(1.0), scaled_identity(0.5), 4).unwrap();
        let mut run = Run::new(&p, 48_000.0, 2);
        let (mut l, mut r) = ([1.0; 1024], [1.0; 1024]);
        run.process(&mut [&mut l, &mut r], 1024);
        assert_eq!(l[511], 0.5);
        assert_eq!(l[512], 0.5);
        assert_eq!(l[513], 0.4375);
        assert_eq!(l[1023], 0.25);
        assert!(Params::identity().with_startup(Params::identity(), Params::identity(), 0).is_none());
        assert!(Params::identity().with_startup(p, Params::identity(), 4).is_none());
    }

    #[test]
    fn reset_preserves_an_unconsumed_cached_filter() {
        let p = scaled_identity(0.25).with_startup(scaled_identity(1.0), scaled_identity(0.5), 4).unwrap();
        let mut run = Run::new(&p, 48_000.0, 2);
        run.reset();
        run.reset();
        let (mut l, mut r) = ([1.0; 6], [1.0; 6]);
        run.process(&mut [&mut l, &mut r], 6);
        assert_eq!(l, [0.5, 0.4375, 0.375, 0.3125, 0.25, 0.25]);
        run.reset();
        let (mut l, mut r) = ([1.0; 6], [1.0; 6]);
        run.process(&mut [&mut l, &mut r], 6);
        assert_eq!(l, [0.5; 6]);
    }

    #[test]
    fn a_fade_endpoint_on_a_boundary_is_rendered_before_queue_handoff() {
        let p = scaled_identity(0.25).with_startup(scaled_identity(1.0), scaled_identity(0.5), 4).unwrap();
        let mut run = Run::new(&p, 48_000.0, 2);
        let (mut l, mut r) = ([1.0; 4], [1.0; 4]);
        run.process(&mut [&mut l, &mut r], 4);
        assert_eq!(l, [1.0, 0.875, 0.75, 0.625]);
        let (mut l, mut r) = ([1.0; 4], [1.0; 4]);
        run.process(&mut [&mut l, &mut r], 4);
        assert_eq!(l, [0.5; 4]);
        let (mut l, mut r) = ([1.0; 4], [1.0; 4]);
        run.process(&mut [&mut l, &mut r], 4);
        assert_eq!(l, [0.5, 0.4375, 0.375, 0.3125]);
    }

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
