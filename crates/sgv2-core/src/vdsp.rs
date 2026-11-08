//! The platform's single-precision multichannel biquad, as boxes call it.
//!
//! Several macOS voicing boxes design their sections in double, store each
//! as five Float32 values and run them through the platform's Float32
//! multichannel biquad on every render call. With poles near unity (low
//! corners at 48 kHz, or any corner at a low selected rate) that filter's
//! Float32 arithmetic is part of the response, so a box that must match the
//! unit runs the same arithmetic. Black-box behaviour of the public API
//! (checked against the installed library with our own calls):
//!
//! - the five coefficients are taken as Float32 values (widened);
//! - for each section, the first eight terms of the denominator response `D`
//!   and of the complete impulse response `H` are prepared in double and
//!   rounded once to Float32;
//! - each section keeps a transposed state `z1, z2` plus its last two inputs
//!   and outputs;
//! - a call runs every section over the call's frames in turn: whole
//!   eight-frame groups, then one four-frame group if four or more frames
//!   remain, then single frames (a call boundary restarts the schedule);
//! - a group is a triangular matrix of `H` over the group's inputs plus `D`
//!   over the entry state, lane by lane as one rounded product and a chain of
//!   Float32 fused multiply-adds; lanes 4 to 7 of an eight-frame group sum
//!   two partial chains (first half of the inputs with `z1`; second half
//!   with `z2`) with a final plain addition;
//! - after a group `z1, z2` are republished from its last two inputs and
//!   outputs; single frames step as fused transposed direct form II.
//!
//! A setup with two or more channels takes another path: every frame of
//! every section steps as fused transposed direct form II
//! (`y = fma(b0, x, z1)`, `z1 = fma(-a1, y, fma(b1, x, z2))`,
//! `z2 = fma(-a2, y, b2 x)`), whatever the call length.

/// One section's coefficients `[b0, b1, b2, a1, a2]` (double, as designed).
pub type Coefficients = [f64; 5];

/// One section of the platform filter on one channel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Section {
    b: [f32; 3],
    a: [f32; 2],
    d: [f32; 8],
    h: [f32; 8],
    z: [f32; 2],
    /// x[n-2], x[n-1], y[n-2], y[n-1]
    history: [f32; 4],
}

impl Section {
    /// A cleared section with `c` installed (rounded to Float32).
    pub fn new(c: Coefficients) -> Section {
        let mut s = Section { b: [0.0; 3], a: [0.0; 2], d: [0.0; 8], h: [0.0; 8], z: [0.0; 2], history: [0.0; 4] };
        s.install(c.map(|v| f64::from(v as f32)));
        s
    }

    pub fn reset(&mut self) {
        self.z = [0.0; 2];
        self.history = [0.0; 4];
    }

    /// Installed Float32 coefficient words, for synthetic parity probes.
    #[doc(hidden)]
    pub fn coefficient_bits(&self) -> [u32; 5] {
        [self.b[0].to_bits(), self.b[1].to_bits(), self.b[2].to_bits(), self.a[0].to_bits(), self.a[1].to_bits()]
    }

    fn install(&mut self, c: [f64; 5]) {
        let [b0, b1, b2, a1, a2] = c;
        let mut g = [0.0f64; 8];
        let mut h = [0.0f64; 8];
        g[0] = 1.0;
        g[1] = -a1;
        for i in 2..8 {
            g[i] = libm::fma(-a1, g[i - 1], (-a2) * g[i - 2]);
        }
        h[0] = b0;
        h[1] = libm::fma(g[1], b0, b1);
        for i in 2..8 {
            h[i] = libm::fma(g[i], b0, libm::fma(g[i - 1], b1, b2 * g[i - 2]));
        }
        self.d = g.map(|v| v as f32);
        self.h = h.map(|v| v as f32);
        self.b = [b0 as f32, b1 as f32, b2 as f32];
        self.a = [a1 as f32, a2 as f32];
    }

    #[inline]
    fn d(&self, k: isize) -> f32 {
        if k < 0 {
            0.0
        } else {
            self.d[k as usize]
        }
    }

    #[inline]
    fn h(&self, k: isize) -> f32 {
        if k < 0 {
            0.0
        } else {
            self.h[k as usize]
        }
    }

    #[inline]
    fn chain(terms: &[(f32, f32)]) -> f32 {
        let mut acc = terms[0].0 * terms[0].1;
        for &(c, v) in &terms[1..] {
            acc = libm::fmaf(c, v, acc);
        }
        acc
    }

    fn group4(&self, x: &[f32; 4], y: &mut [f32]) {
        let [z1, z2] = self.z;
        for (i, out) in y.iter_mut().enumerate().take(4) {
            let i = i as isize;
            *out = Self::chain(&[
                (self.h(i), x[0]),
                (self.h(i - 1), x[1]),
                (self.h(i - 2), x[2]),
                (self.h(i - 3), x[3]),
                (self.d(i), z1),
                (self.d(i - 1), z2),
            ]);
        }
    }

    fn group8(&self, x: &[f32; 8], y: &mut [f32]) {
        let [z1, z2] = self.z;
        self.group4(&[x[0], x[1], x[2], x[3]], &mut y[..4]);
        for i in 4..8isize {
            let first = Self::chain(&[
                (self.h(i), x[0]),
                (self.h(i - 1), x[1]),
                (self.h(i - 2), x[2]),
                (self.h(i - 3), x[3]),
                (self.d(i), z1),
            ]);
            let second = Self::chain(&[
                (self.h(i - 4), x[4]),
                (self.d(i - 1), z2),
                (self.h(i - 5), x[5]),
                (self.h(i - 6), x[6]),
                (self.h(i - 7), x[7]),
            ]);
            y[i as usize] = first + second;
        }
    }

    fn publish(&mut self, x: (f32, f32), y: (f32, f32)) {
        let [_, b1, b2] = self.b;
        let [a1, a2] = self.a;
        let mut n1 = b2 * x.0;
        let mut n2 = 0.0 * x.0;
        n1 = libm::fmaf(b1, x.1, n1);
        n2 = libm::fmaf(b2, x.1, n2);
        n1 = libm::fmaf(-a2, y.0, n1);
        n2 = libm::fmaf(-0.0, y.0, n2);
        n1 = libm::fmaf(-a1, y.1, n1);
        n2 = libm::fmaf(-a2, y.1, n2);
        self.z = [n1, n2];
    }

    /// Filters one call's frames in place, frame by frame (the path of a
    /// setup with two or more channels).
    pub fn run_frames(&mut self, buf: &mut [f32]) {
        let [b0, b1, b2] = self.b;
        let [a1, a2] = self.a;
        let [mut z1, mut z2] = self.z;
        for v in buf.iter_mut() {
            let x = *v;
            let y = libm::fmaf(b0, x, z1);
            z1 = libm::fmaf(-a1, y, libm::fmaf(b1, x, z2));
            z2 = libm::fmaf(-a2, y, b2 * x);
            *v = y;
        }
        self.z = [z1, z2];
    }

    /// Filters one call's frames in place (the path of a one-channel setup).
    pub fn run(&mut self, buf: &mut [f32]) {
        let n = buf.len();
        if n == 0 {
            return;
        }
        let (old_x1, old_y1) = (self.history[1], self.history[3]);
        let (mut last_x, mut prev_x) = (0.0f32, 0.0f32);
        let mut at = 0;
        while n - at >= 8 {
            let mut x = [0.0f32; 8];
            x.copy_from_slice(&buf[at..at + 8]);
            self.group8(&x, &mut buf[at..at + 8]);
            self.publish((x[6], x[7]), (buf[at + 6], buf[at + 7]));
            (prev_x, last_x) = (x[6], x[7]);
            at += 8;
        }
        if n - at >= 4 {
            let mut x = [0.0f32; 4];
            x.copy_from_slice(&buf[at..at + 4]);
            self.group4(&x, &mut buf[at..at + 4]);
            self.publish((x[2], x[3]), (buf[at + 2], buf[at + 3]));
            (prev_x, last_x) = (x[2], x[3]);
            at += 4;
        }
        let [b0, b1, b2] = self.b;
        let [a1, a2] = self.a;
        while at < n {
            let x = buf[at];
            let [z1, z2] = self.z;
            let y = libm::fmaf(b0, x, z1);
            let t = libm::fmaf(b1, x, z2);
            let next1 = libm::fmaf(-a1, y, t);
            let p = (-a2) * y;
            let next2 = libm::fmaf(b2, x, p);
            self.z = [next1, next2];
            buf[at] = y;
            (prev_x, last_x) = (last_x, x);
            at += 1;
        }
        self.history = if n == 1 { [old_x1, last_x, old_y1, buf[0]] } else { [prev_x, last_x, buf[n - 2], buf[n - 1]] };
    }
}

/// Most sections one cascade carries.
pub const MAX_SECTIONS: usize = 32;

/// A cascade on one channel: every section runs over the whole call in turn.
/// `multichannel`: the channel belongs to a setup of two or more channels.
#[derive(Clone, Copy, Debug)]
pub struct Cascade {
    n: usize,
    multichannel: bool,
    s: [Section; MAX_SECTIONS],
}

impl Cascade {
    pub fn new(sections: &[Coefficients], multichannel: bool) -> Cascade {
        let mut c = Cascade { n: 0, multichannel, s: [Section::new([1.0, 0.0, 0.0, 0.0, 0.0]); MAX_SECTIONS] };
        for &q in sections.iter().take(MAX_SECTIONS) {
            c.s[c.n] = Section::new(q);
            c.n += 1;
        }
        c
    }

    pub fn len(&self) -> usize {
        self.n
    }

    pub fn is_empty(&self) -> bool {
        self.n == 0
    }

    /// Copies installed coefficient words into `out`, returning the count.
    #[doc(hidden)]
    pub fn coefficient_bits(&self, out: &mut [[u32; 5]]) -> usize {
        let n = self.n.min(out.len());
        for (dst, section) in out[..n].iter_mut().zip(&self.s[..n]) {
            *dst = section.coefficient_bits();
        }
        n
    }

    pub fn reset(&mut self) {
        for s in &mut self.s[..self.n] {
            s.reset();
        }
    }

    pub fn run(&mut self, buf: &mut [f32]) {
        for s in &mut self.s[..self.n] {
            if self.multichannel {
                s.run_frames(buf);
            } else {
                s.run(buf);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_passes_and_groups_match_single_frames_for_an_fir() {
        // An FIR section has exact Float32 terms: the group form and the
        // single-frame form agree bit for bit on small integers.
        let x: [f32; 23] = core::array::from_fn(|i| (i as f32) - 7.0);
        let mut a = x;
        Cascade::new(&[[1.0, 0.0, 0.0, 0.0, 0.0]], false).run(&mut a);
        assert_eq!(a, x);
        let fir = [0.5, 0.25, -1.0, 0.0, 0.0];
        let mut whole = x;
        Cascade::new(&[fir], false).run(&mut whole);
        let mut single = x;
        let mut c = Cascade::new(&[fir], false);
        for f in single.chunks_mut(1) {
            c.run(f);
        }
        assert_eq!(whole, single);
        assert_eq!(whole[2], 0.5 * x[2] + 0.25 * x[1] - x[0]);
    }

    #[test]
    fn call_split_changes_only_the_one_channel_path() {
        let section = [0.000_1, 0.000_2, 0.000_1, -1.98, 0.980_1];
        let input: [f32; 512] = core::array::from_fn(|i| libm::sinf(i as f32 * 0.013) * 0.25);

        let run = |multichannel: bool, split: bool| {
            let mut out = input;
            let mut c = Cascade::new(&[section], multichannel);
            if split {
                for frame in out.chunks_mut(1) {
                    c.run(frame);
                }
            } else {
                c.run(&mut out);
            }
            out
        };

        assert_eq!(run(true, false), run(true, true));
        assert_ne!(run(false, false), run(false, true));
    }
}
