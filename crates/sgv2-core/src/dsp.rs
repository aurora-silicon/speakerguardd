//! Per-channel vector kernels for the real-time path.
//!
//! Every function works on one channel's contiguous `f32` slice, does not
//! allocate, and has a NEON body on aarch64 (all Apple Silicon) with a
//! scalar body elsewhere, so the same code runs in host tests on x86_64.
//! The NEON bodies process 16 samples per iteration with four independent
//! accumulators, which hides the FMA latency on Firestorm/Avalanche cores.
//!
//! Reductions accumulate in `f32` lanes and finish in `f64`: over one
//! 512-frame block the relative error is below 1e-6, far inside the
//! differential tolerances, and the result does not depend on how the host
//! split the block (callers accumulate block sums in `f64`).

#[cfg(target_arch = "aarch64")]
use core::arch::aarch64::*;

/// Sum of squares of `x`.
#[inline]
pub fn sum_sq(x: &[f32]) -> f64 {
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let n = x.len();
        let p = x.as_ptr();
        let mut a0 = vdupq_n_f32(0.0);
        let mut a1 = vdupq_n_f32(0.0);
        let mut a2 = vdupq_n_f32(0.0);
        let mut a3 = vdupq_n_f32(0.0);
        let mut i = 0;
        while i + 16 <= n {
            let v0 = vld1q_f32(p.add(i));
            let v1 = vld1q_f32(p.add(i + 4));
            let v2 = vld1q_f32(p.add(i + 8));
            let v3 = vld1q_f32(p.add(i + 12));
            a0 = vfmaq_f32(a0, v0, v0);
            a1 = vfmaq_f32(a1, v1, v1);
            a2 = vfmaq_f32(a2, v2, v2);
            a3 = vfmaq_f32(a3, v3, v3);
            i += 16;
        }
        let s = vaddq_f32(vaddq_f32(a0, a1), vaddq_f32(a2, a3));
        let mut acc = vaddvq_f32(s) as f64;
        while i < n {
            let v = *p.add(i) as f64;
            acc += v * v;
            i += 1;
        }
        acc
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        x.iter().map(|&v| (v as f64) * (v as f64)).sum()
    }
}

/// Sum of `a[n] * b[n]` (in-phase V/I product).
#[inline]
pub fn dot(a: &[f32], b: &[f32]) -> f64 {
    let n = a.len().min(b.len());
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let (pa, pb) = (a.as_ptr(), b.as_ptr());
        let mut a0 = vdupq_n_f32(0.0);
        let mut a1 = vdupq_n_f32(0.0);
        let mut a2 = vdupq_n_f32(0.0);
        let mut a3 = vdupq_n_f32(0.0);
        let mut i = 0;
        while i + 16 <= n {
            a0 = vfmaq_f32(a0, vld1q_f32(pa.add(i)), vld1q_f32(pb.add(i)));
            a1 = vfmaq_f32(a1, vld1q_f32(pa.add(i + 4)), vld1q_f32(pb.add(i + 4)));
            a2 = vfmaq_f32(a2, vld1q_f32(pa.add(i + 8)), vld1q_f32(pb.add(i + 8)));
            a3 = vfmaq_f32(a3, vld1q_f32(pa.add(i + 12)), vld1q_f32(pb.add(i + 12)));
            i += 16;
        }
        let s = vaddq_f32(vaddq_f32(a0, a1), vaddq_f32(a2, a3));
        let mut acc = vaddvq_f32(s) as f64;
        while i < n {
            acc += *pa.add(i) as f64 * *pb.add(i) as f64;
            i += 1;
        }
        acc
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        a[..n].iter().zip(&b[..n]).map(|(&x, &y)| x as f64 * y as f64).sum()
    }
}

/// `sum a[n] * b[n]` in f32, sequential: the scalar reference of
/// [`dot_f32`] (the lsm2 FIR banks).
#[inline]
pub fn dot_f32_scalar(a: &[f32], b: &[f32]) -> f32 {
    let n = a.len().min(b.len());
    let mut acc = 0.0f32;
    for i in 0..n {
        acc += a[i] * b[i];
    }
    acc
}

/// `sum a[n] * b[n]` in f32: the lsm2 FIR banks' kernel. On aarch64, NEON
/// with four independent 4-lane FMA accumulators (16 products per
/// iteration), the lanes summed pairwise at the end; elsewhere the scalar
/// reference. The re-association changes only rounding: the difference
/// from [`dot_f32_scalar`] is below `n * 2^-23 * sum |a[n] b[n]|` (tested
/// at `2e-6` of that sum for the 319- and 1,024-tap banks).
#[inline]
pub fn dot_f32(a: &[f32], b: &[f32]) -> f32 {
    let n = a.len().min(b.len());
    #[cfg(target_arch = "aarch64")]
    // SAFETY: every access is below `n`, the shorter of the two slices.
    unsafe {
        let (pa, pb) = (a.as_ptr(), b.as_ptr());
        let mut a0 = vdupq_n_f32(0.0);
        let mut a1 = vdupq_n_f32(0.0);
        let mut a2 = vdupq_n_f32(0.0);
        let mut a3 = vdupq_n_f32(0.0);
        let mut i = 0;
        while i + 16 <= n {
            a0 = vfmaq_f32(a0, vld1q_f32(pa.add(i)), vld1q_f32(pb.add(i)));
            a1 = vfmaq_f32(a1, vld1q_f32(pa.add(i + 4)), vld1q_f32(pb.add(i + 4)));
            a2 = vfmaq_f32(a2, vld1q_f32(pa.add(i + 8)), vld1q_f32(pb.add(i + 8)));
            a3 = vfmaq_f32(a3, vld1q_f32(pa.add(i + 12)), vld1q_f32(pb.add(i + 12)));
            i += 16;
        }
        while i + 4 <= n {
            a0 = vfmaq_f32(a0, vld1q_f32(pa.add(i)), vld1q_f32(pb.add(i)));
            i += 4;
        }
        let mut acc = vaddvq_f32(vaddq_f32(vaddq_f32(a0, a1), vaddq_f32(a2, a3)));
        while i < n {
            acc += *pa.add(i) * *pb.add(i);
            i += 1;
        }
        acc
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        dot_f32_scalar(a, b)
    }
}

/// Largest `|x[n]|`, NaN-propagating as "infinite" (a non-finite sample
/// must never look quiet to a peak check).
#[inline]
pub fn peak_abs(x: &[f32]) -> f32 {
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let n = x.len();
        let p = x.as_ptr();
        let mut m0 = vdupq_n_f32(0.0);
        let mut m1 = vdupq_n_f32(0.0);
        let mut i = 0;
        while i + 8 <= n {
            // vmaxq propagates NaN, unlike vmaxnmq.
            m0 = vmaxq_f32(m0, vabsq_f32(vld1q_f32(p.add(i))));
            m1 = vmaxq_f32(m1, vabsq_f32(vld1q_f32(p.add(i + 4))));
            i += 8;
        }
        let mut m = vmaxvq_f32(vmaxq_f32(m0, m1));
        while i < n {
            let v = (*p.add(i)).abs();
            m = if v.is_nan() || v > m { v } else { m };
            i += 1;
        }
        if m.is_nan() {
            f32::INFINITY
        } else {
            m
        }
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let mut m = 0.0f32;
        for &v in x {
            let a = v.abs();
            if a.is_nan() {
                return f32::INFINITY;
            }
            m = m.max(a);
        }
        m
    }
}

/// `y[n] = g * x[n]`.
#[inline]
pub fn scale_to(x: &[f32], g: f32, y: &mut [f32]) {
    let n = x.len().min(y.len());
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let (px, py) = (x.as_ptr(), y.as_mut_ptr());
        let vg = vdupq_n_f32(g);
        let mut i = 0;
        while i + 16 <= n {
            vst1q_f32(py.add(i), vmulq_f32(vld1q_f32(px.add(i)), vg));
            vst1q_f32(py.add(i + 4), vmulq_f32(vld1q_f32(px.add(i + 4)), vg));
            vst1q_f32(py.add(i + 8), vmulq_f32(vld1q_f32(px.add(i + 8)), vg));
            vst1q_f32(py.add(i + 12), vmulq_f32(vld1q_f32(px.add(i + 12)), vg));
            i += 16;
        }
        while i < n {
            *py.add(i) = *px.add(i) * g;
            i += 1;
        }
    }
    #[cfg(not(target_arch = "aarch64"))]
    for (o, &v) in y[..n].iter_mut().zip(&x[..n]) {
        *o = v * g;
    }
}

/// `y[n] = a[n] * b[n]`.
#[inline]
pub fn mul_to(a: &[f32], b: &[f32], y: &mut [f32]) {
    let n = a.len().min(b.len()).min(y.len());
    #[cfg(target_arch = "aarch64")]
    // SAFETY: every access is below `n`, the shortest of the three slices.
    unsafe {
        let (pa, pb, py) = (a.as_ptr(), b.as_ptr(), y.as_mut_ptr());
        let mut i = 0;
        while i + 16 <= n {
            vst1q_f32(py.add(i), vmulq_f32(vld1q_f32(pa.add(i)), vld1q_f32(pb.add(i))));
            vst1q_f32(py.add(i + 4), vmulq_f32(vld1q_f32(pa.add(i + 4)), vld1q_f32(pb.add(i + 4))));
            vst1q_f32(py.add(i + 8), vmulq_f32(vld1q_f32(pa.add(i + 8)), vld1q_f32(pb.add(i + 8))));
            vst1q_f32(py.add(i + 12), vmulq_f32(vld1q_f32(pa.add(i + 12)), vld1q_f32(pb.add(i + 12))));
            i += 16;
        }
        while i < n {
            *py.add(i) = *pa.add(i) * *pb.add(i);
            i += 1;
        }
    }
    #[cfg(not(target_arch = "aarch64"))]
    for ((o, &x), &g) in y[..n].iter_mut().zip(&a[..n]).zip(&b[..n]) {
        *o = x * g;
    }
}

/// `y[n] = bound1(a[n]) * b[n]`: the program held to digital full scale
/// (a non-finite sample 0) times the gain. Returns whether any `a[n]` was
/// non-finite. For `|a[n]| <= 1` the result is bit-identical to `mul_to`.
#[inline]
pub fn mul_bounded_to(a: &[f32], b: &[f32], y: &mut [f32]) -> bool {
    let n = a.len().min(b.len()).min(y.len());
    #[cfg(not(target_arch = "aarch64"))]
    let (i, mut bad) = (0, false);
    #[cfg(target_arch = "aarch64")]
    // SAFETY: every access is below `n`, the shortest of the three slices.
    let (i, mut bad) = unsafe {
        let mut i = 0;
        let (pa, pb, py) = (a.as_ptr(), b.as_ptr(), y.as_mut_ptr());
        let one = vdupq_n_f32(1.0);
        let neg = vdupq_n_f32(-1.0);
        let inf = vdupq_n_f32(f32::INFINITY);
        let zero = vdupq_n_f32(0.0);
        let mut nf = vdupq_n_u32(0);
        while i + 4 <= n {
            let x = vld1q_f32(pa.add(i));
            // |x| < inf is false for NaN and for +-inf
            let fin = vcltq_f32(vabsq_f32(x), inf);
            let c = vbslq_f32(fin, vminq_f32(vmaxq_f32(x, neg), one), zero);
            vst1q_f32(py.add(i), vmulq_f32(c, vld1q_f32(pb.add(i))));
            nf = vorrq_u32(nf, vmvnq_u32(fin));
            i += 4;
        }
        (i, vmaxvq_u32(nf) != 0)
    };
    for ((o, &x), &g) in y[i..n].iter_mut().zip(&a[i..n]).zip(&b[i..n]) {
        bad |= !x.is_finite();
        *o = bound1(x) * g;
    }
    bad
}

/// In-place `x[n] *= g`.
#[inline]
pub fn scale_in_place(x: &mut [f32], g: f32) {
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let n = x.len();
        let p = x.as_mut_ptr();
        let vg = vdupq_n_f32(g);
        let mut i = 0;
        while i + 16 <= n {
            vst1q_f32(p.add(i), vmulq_f32(vld1q_f32(p.add(i)), vg));
            vst1q_f32(p.add(i + 4), vmulq_f32(vld1q_f32(p.add(i + 4)), vg));
            vst1q_f32(p.add(i + 8), vmulq_f32(vld1q_f32(p.add(i + 8)), vg));
            vst1q_f32(p.add(i + 12), vmulq_f32(vld1q_f32(p.add(i + 12)), vg));
            i += 16;
        }
        while i < n {
            *p.add(i) *= g;
            i += 1;
        }
    }
    #[cfg(not(target_arch = "aarch64"))]
    for v in x.iter_mut() {
        *v *= g;
    }
}

/// `y[n] = g * x[n]` and returns the sum of squares of the *input* and of
/// the *output* in one pass (spp3 needs the unprotected drive power for its
/// gain target and the protected drive power for its thermal model).
#[inline]
pub fn scale_to_with_power(x: &[f32], g: f32, y: &mut [f32]) -> (f64, f64) {
    let n = x.len().min(y.len());
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let (px, py) = (x.as_ptr(), y.as_mut_ptr());
        let vg = vdupq_n_f32(g);
        let mut a0 = vdupq_n_f32(0.0);
        let mut a1 = vdupq_n_f32(0.0);
        let mut a2 = vdupq_n_f32(0.0);
        let mut a3 = vdupq_n_f32(0.0);
        let mut i = 0;
        while i + 16 <= n {
            let v0 = vld1q_f32(px.add(i));
            let v1 = vld1q_f32(px.add(i + 4));
            let v2 = vld1q_f32(px.add(i + 8));
            let v3 = vld1q_f32(px.add(i + 12));
            a0 = vfmaq_f32(a0, v0, v0);
            a1 = vfmaq_f32(a1, v1, v1);
            a2 = vfmaq_f32(a2, v2, v2);
            a3 = vfmaq_f32(a3, v3, v3);
            vst1q_f32(py.add(i), vmulq_f32(v0, vg));
            vst1q_f32(py.add(i + 4), vmulq_f32(v1, vg));
            vst1q_f32(py.add(i + 8), vmulq_f32(v2, vg));
            vst1q_f32(py.add(i + 12), vmulq_f32(v3, vg));
            i += 16;
        }
        let s = vaddq_f32(vaddq_f32(a0, a1), vaddq_f32(a2, a3));
        let mut acc = vaddvq_f32(s) as f64;
        while i < n {
            let v = *px.add(i);
            acc += v as f64 * v as f64;
            *py.add(i) = v * g;
            i += 1;
        }
        let gg = g as f64 * g as f64;
        (acc, acc * gg)
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let mut acc = 0.0f64;
        for (o, &v) in y[..n].iter_mut().zip(&x[..n]) {
            acc += v as f64 * v as f64;
            *o = v * g;
        }
        (acc, acc * g as f64 * g as f64)
    }
}

/// A block accumulator whose result does not depend on how the block was
/// split into host calls: the sample at block frame `k` always lands in
/// lane `k % 16`, every lane adds its samples in frame order, and the lanes
/// are reduced in a fixed order at the end. Multiplies and adds are kept
/// separate (never fused), in the NEON body and the scalar head/tail alike,
/// so both paths round identically. The NEON body is the same four-register
/// unrolled loop as `sum_sq`, run whenever the segment is lane-aligned.
#[derive(Clone, Copy, Debug)]
pub struct LaneAcc {
    lanes: [f32; 16],
}

impl Default for LaneAcc {
    fn default() -> Self {
        Self::new()
    }
}

impl LaneAcc {
    pub const fn new() -> Self {
        LaneAcc { lanes: [0.0; 16] }
    }

    pub fn clear(&mut self) {
        self.lanes = [0.0; 16];
    }

    /// The block total, reduced in a fixed order.
    pub fn total(&self) -> f64 {
        self.lanes.iter().map(|&v| v as f64).sum()
    }

    pub fn is_finite(&self) -> bool {
        self.lanes.iter().all(|v| v.is_finite())
    }

    /// Add `a[n] * b[n]` for a segment whose first frame is block frame
    /// `phase` (use the same slice twice for squares).
    #[inline]
    pub fn add_prod(&mut self, a: &[f32], b: &[f32], phase: usize) {
        let n = a.len().min(b.len());
        let mut i = 0;
        let mut lane = phase & 15;
        // scalar head up to the next lane-0 frame
        while i < n && lane != 0 {
            self.lanes[lane] += a[i] * b[i];
            i += 1;
            lane = (lane + 1) & 15;
        }
        #[cfg(target_arch = "aarch64")]
        // SAFETY: loads stay inside `a[..n]`/`b[..n]` (i + 16 <= n); the
        // lane array is 16 f32s, stored back as four 4-lane vectors.
        unsafe {
            let (pa, pb) = (a.as_ptr(), b.as_ptr());
            let l = self.lanes.as_mut_ptr();
            let mut a0 = vld1q_f32(l);
            let mut a1 = vld1q_f32(l.add(4));
            let mut a2 = vld1q_f32(l.add(8));
            let mut a3 = vld1q_f32(l.add(12));
            while i + 16 <= n {
                a0 = vaddq_f32(a0, vmulq_f32(vld1q_f32(pa.add(i)), vld1q_f32(pb.add(i))));
                a1 = vaddq_f32(a1, vmulq_f32(vld1q_f32(pa.add(i + 4)), vld1q_f32(pb.add(i + 4))));
                a2 = vaddq_f32(a2, vmulq_f32(vld1q_f32(pa.add(i + 8)), vld1q_f32(pb.add(i + 8))));
                a3 = vaddq_f32(a3, vmulq_f32(vld1q_f32(pa.add(i + 12)), vld1q_f32(pb.add(i + 12))));
                i += 16;
            }
            vst1q_f32(l, a0);
            vst1q_f32(l.add(4), a1);
            vst1q_f32(l.add(8), a2);
            vst1q_f32(l.add(12), a3);
        }
        // scalar tail (and the whole body off aarch64), lane 0 onward
        let mut lane = 0;
        while i < n {
            self.lanes[lane] += a[i] * b[i];
            i += 1;
            lane = (lane + 1) & 15;
        }
    }

    #[inline]
    pub fn add_sq(&mut self, x: &[f32], phase: usize) {
        self.add_prod(x, x, phase);
    }
}

/// Replace every non-finite sample with 0 and return how many there were.
/// (Apple's graph has a non-finite guard at its head; the unit keeps one of
/// its own so a NaN can never poison the thermal state.)
#[inline]
pub fn sanitize(x: &mut [f32]) -> usize {
    let mut bad = 0;
    for v in x.iter_mut() {
        if !v.is_finite() {
            *v = 0.0;
            bad += 1;
        }
    }
    bad
}

/// One sample held to digital full scale: `clamp(x, -1, 1)`, a non-finite
/// sample 0. No converter can deliver more than full scale, so this is the
/// drive a sample can actually produce; every model meter and every output
/// sample goes through it, which also bounds every square sum (16 lanes of
/// at most 1024 unit squares per update never approach `f32::MAX`).
#[inline]
pub fn bound1(x: f32) -> f32 {
    if x.abs() < f32::INFINITY {
        x.clamp(-1.0, 1.0)
    } else {
        0.0
    }
}

/// [`bound1`] in place; returns how many samples were non-finite.
#[inline]
pub fn bound(x: &mut [f32]) -> usize {
    let mut bad = 0;
    for v in x.iter_mut() {
        bad += !v.is_finite() as usize;
        *v = bound1(*v);
    }
    bad
}

/// Deinterleave `frames` frames of `ch`-channel S16 into per-channel f32
/// (`out[c][n]`), scaled to [-1, 1). Used by the direct-ALSA sense reader
/// and by the bench; the LV2 path receives planar f32 from PipeWire.
pub fn deinterleave_s16(src: &[i16], ch: usize, out: &mut [&mut [f32]]) {
    const K: f32 = 1.0 / 32768.0;
    let frames = src.len() / ch.max(1);
    for (c, o) in out.iter_mut().enumerate().take(ch) {
        let m = frames.min(o.len());
        let o = &mut o[..m];
        for (n, s) in o.iter_mut().enumerate() {
            *s = src[n * ch + c] as f32 * K;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sig(n: usize, seed: u32) -> Vec<f32> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0
            })
            .collect()
    }

    #[test]
    fn kernels_match_scalar() {
        for n in [0, 1, 7, 15, 16, 17, 96, 511, 512, 1024, 1031] {
            let x = sig(n, 3 + n as u32);
            let y = sig(n, 9 + n as u32);
            let ss: f64 = x.iter().map(|&v| v as f64 * v as f64).sum();
            assert!((sum_sq(&x) - ss).abs() <= 1e-5 * ss.max(1.0), "sum_sq {n}");
            let d: f64 = x.iter().zip(&y).map(|(&a, &b)| a as f64 * b as f64).sum();
            assert!((dot(&x, &y) - d).abs() <= 1e-5 * ss.max(1.0), "dot {n}");
            let pk = x.iter().fold(0.0f32, |m, v| m.max(v.abs()));
            assert_eq!(peak_abs(&x), pk);
            let mut o = vec![0.0; n];
            let (pi, po) = scale_to_with_power(&x, 0.5, &mut o);
            assert!((pi - ss).abs() <= 1e-5 * ss.max(1.0));
            assert!((po - ss * 0.25).abs() <= 1e-5 * ss.max(1.0));
            for i in 0..n {
                assert_eq!(o[i], x[i] * 0.5);
            }
        }
    }

    #[test]
    fn lane_acc_is_split_invariant_and_matches_scalar() {
        let x = sig(2048, 5);
        let mut whole = LaneAcc::new();
        whole.add_sq(&x, 0);
        let exact: f64 = x.iter().map(|&v| v as f64 * v as f64).sum();
        assert!((whole.total() - exact).abs() < 1e-5 * exact);
        for split in [1usize, 3, 7, 16, 17, 100, 511] {
            let mut a = LaneAcc::new();
            let mut pos = 0;
            while pos < x.len() {
                let n = split.min(x.len() - pos);
                a.add_sq(&x[pos..pos + n], pos);
                pos += n;
            }
            assert_eq!(a.total().to_bits(), whole.total().to_bits(), "split {split}");
        }
    }

    #[test]
    fn peak_sees_nan() {
        let mut x = vec![0.1f32; 40];
        x[33] = f32::NAN;
        assert_eq!(peak_abs(&x), f32::INFINITY);
        x[33] = 0.0;
        x[3] = f32::NAN;
        assert_eq!(peak_abs(&x), f32::INFINITY);
    }
}
