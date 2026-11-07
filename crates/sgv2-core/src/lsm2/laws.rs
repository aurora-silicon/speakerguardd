//! The lsm2 residual laws (SPEC-LSM2-RESIDUALS, 2026-11-05): the current
//! model's coefficient builder, the thermal branch realization, the
//! temperature candidate, the masker's spectral detector and the
//! four-anchor velocity curve. Written from the specification's
//! mathematics; the binary32 rounding points are part of each law.

use alloc::vec;
use alloc::vec::Vec;

use core::f64::consts::PI;

/// Round to binary32 and back.
#[inline]
fn r32(x: f64) -> f32 {
    x as f32
}

/// The current model's coefficients (residuals section 1): the mechanical
/// zeros mapped by `exp(p/Fs)`, the denominator the stable minimum-phase
/// quadratic whose magnitude at `0, pi/3, 2pi/3` matches the analog
/// section's, and `1/R` on the numerator. Returns `(b, [a1, a2])`.
pub fn current_coefficients(r: f64, bl: f64, m: f64, d: f64, k: f64, rate: f64) -> ([f32; 3], [f32; 2]) {
    let sigma = d / (2.0 * m);
    let disc = k / m - sigma * sigma;
    let zeros: [f32; 3] = if disc >= 0.0 {
        let radius = libm::exp(-sigma / rate);
        [1.0, r32(-2.0 * radius * libm::cos(libm::sqrt(disc) / rate)), r32(radius * radius)]
    } else {
        let root = libm::sqrt(-disc);
        let (z0, z1) = (libm::exp((-sigma + root) / rate), libm::exp((-sigma - root) / rate));
        [1.0, r32(-z0 - z1), r32(z0 * z1)]
    };
    // the normalized analog section, its coefficients quantized first
    let nz1 = r32(d / m) as f64;
    let nz2 = r32(k / m) as f64;
    let dp1 = r32((d + bl * bl / r) / m) as f64;
    let mut q = [0.0f32; 3];
    for (j, qj) in q.iter_mut().enumerate() {
        let w = j as f64 * PI / 3.0;
        let s = w * rate;
        // H(i s) = (nz2 - s^2 + i nz1 s) / (nz2 - s^2 + i dp1 s)
        let an = libm::hypot(nz2 - s * s, nz1 * s);
        let ad = libm::hypot(nz2 - s * s, dp1 * s);
        // Z(e^{i w}) = z0 + z1 e^{-i w} + z2 e^{-2 i w}
        let (z0, z1, z2) = (zeros[0] as f64, zeros[1] as f64, zeros[2] as f64);
        let re = z0 + z1 * libm::cos(w) + z2 * libm::cos(2.0 * w);
        let im = -z1 * libm::sin(w) - z2 * libm::sin(2.0 * w);
        let zd = libm::hypot(re, im);
        *qj = r32(zd / (an / ad));
    }
    let (d0, d1, d2) = minimum_phase_triplet(q);
    let g = r32(1.0 / r);
    let b = zeros.map(|z| (z / d0) * g);
    (b, [d1 / d0, d2 / d0])
}

/// The stable real quadratic `d0 + d1 z^-1 + d2 z^-2` with magnitudes `q`
/// at `0, pi/3, 2pi/3`, by spectral factorization, every step in binary32.
/// The four contractions are explicit: native's ARM64 builder uses FMA,
/// while expanding them into separate operations reproduces its inactive
/// x86 slice but moves the running current model and thermal power.
pub fn minimum_phase_triplet(q: [f32; 3]) -> (f32, f32, f32) {
    let [q0, q1, q2] = q;
    let omega = r32(PI / 3.0);
    let c = r32(libm::cos(omega as f64));
    let c2 = r32(libm::cos((omega + omega) as f64));
    let v = libm::fmaf(q1, q1, -(q2 * q2)) / (4.0 * c);
    let disc = libm::fmaf(q0, q0, -(4.0 * v));
    let middle = if disc >= 0.0 { (q0 - libm::sqrtf(disc)) * 0.5 } else { libm::sqrtf(q0 * q0 - disc) * 0.5 };
    let total = q0 - middle;
    let aa = (1.0 - c2) * 2.0;
    let bb = (c2 - 1.0) * (total + total);
    let cc = libm::fmaf(total, total, middle * middle);
    let cc = libm::fmaf((total + total) * middle, c, cc);
    let cc = libm::fmaf(-q1, q1, cc);
    let det = libm::fmaf(bb, bb, -((4.0 * aa) * cc));
    let num = if det >= 0.0 { -bb - libm::sqrtf(det) } else { libm::sqrtf(bb * bb - det) };
    let last = num / (aa + aa);
    (total - last, middle, last)
}

/// One first-order thermal branch (residuals 2.1): the generated equal
/// numerator pair and pole kept as `alpha = 1 + a1` and the effective DC
/// gain `R_eff = 2 b0 / alpha`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ThermalBranch {
    pub alpha: f32,
    pub r_eff: f32,
}

impl ThermalBranch {
    /// The matched pole of `tau` for an update every `dt` seconds, with the
    /// binary32 gain `(1 - exp(-dt/tau)) * (Rth / alpha)`: the ideal step
    /// rounded once to binary32 times the binary32 quotient of `Rth` by the
    /// rounded step `alpha`.
    pub fn new(rth: f64, tau: f64, dt: f64) -> Self {
        let ideal = 1.0 - libm::exp(-dt / tau);
        let alpha = 1.0f32 - r32(1.0 - ideal);
        ThermalBranch { alpha, r_eff: r32(ideal) * (r32(rth) / alpha) }
    }
    /// `theta + alpha (R_eff P - theta)`, both products fused:
    /// `fma(alpha, fma(R_eff, P, -theta), theta)`.
    #[inline]
    pub fn step(&self, theta: f32, p: f32) -> f32 {
        libm::fmaf(self.alpha, libm::fmaf(self.r_eff, p, -theta), theta)
    }
    /// The rise after `n` updates at no power (the gap's cooling).
    pub fn cool(&self, theta: f64, n: f64) -> f64 {
        theta * libm::pow(1.0 - self.alpha as f64, n)
    }
}

/// The temperature candidate (residuals 2.2), binary32 throughout: `tpk`
/// the input model's selected peak temperature, `vc`, `mg` its current
/// branch rises, `om` the output model's body rise before this decision.
#[allow(clippy::too_many_arguments)]
pub fn temperature_candidate(
    tpk: f32,
    vc: f32,
    mg: f32,
    om: f32,
    amb: f32,
    soft: f32,
    hard: f32,
    rv: f32,
    rm: f32,
) -> f32 {
    if tpk <= soft {
        return 1.0;
    }
    let width = hard - soft;
    let knee = soft + width * (1.0 - libm::expf(-(tpk - soft) / width.max(1e-15)));
    // the second strict guard: in binary32 the goal can reach the peak
    if tpk <= knee {
        return 1.0;
    }
    let room = (knee - amb).max(0.0);
    let (rv1, rm1) = (rv.max(1e-6), rm.max(1e-6));
    let rho = (rv1 / (rv1 + rm1)).min(0.995);
    let (vc1, mg1) = (vc.max(0.001), mg.max(0.001));
    let phi = (vc1 / (vc1 + mg1)).min(0.995);
    let body_ceiling = (1.0 - rho) * hard;
    let mix = if om >= body_ceiling { 1.0 } else { (om / body_ceiling).max(0.0) };
    let mix = mix * mix;
    // The native arm64 path does not form an interpolated split and then
    // multiply it by the room.  It first scales both endpoint splits by the
    // room, then interpolates each allowance with one binary32 FMA.  The two
    // forms are algebraically equal but differ by a ULP near the first bite.
    let phi_vc = room * phi;
    let phi_mg = room - phi_vc;
    let rho_vc = room * rho;
    let rho_mg = room - rho_vc;
    let allowed_vc = libm::fmaf(mix, rho_vc - phi_vc, phi_vc);
    let allowed_mg = libm::fmaf(mix, rho_mg - phi_mg, phi_mg);
    let mut g = 1.0f32;
    for (allowed, present) in [(allowed_vc, vc), (allowed_mg, mg)] {
        if allowed < present {
            g = g.min(allowed / present);
        }
    }
    libm::sqrtf(g.max(0.0))
}

/// The velocity limit (residuals section 4): four energy anchors, the last
/// segment toward `vmax` at `E3` and extrapolated past it, no upper clamp.
pub fn velocity_limit(e: f32, k: &[f32; 4], v: &[f32; 3], vmax: f32) -> f32 {
    if e <= k[0] {
        return v[0];
    }
    let (a, b, lo, hi) = if e < k[1] {
        (v[0], v[1], k[0], k[1])
    } else if e < k[2] {
        (v[1], v[2], k[1], k[2])
    } else {
        (v[2], vmax, k[2], k[3])
    };
    let slope = (b - a) / (hi - lo);
    a + (e - lo) * slope
}

/// The velocity allocation (residuals 5): on the signed sum of the five
/// band velocities `vb[k][n]` over `first..`, under the applied slow gain
/// `s`, find the worst sample, orient every band's contribution to its sign
/// and water-fill the active bands (`active`) into the room the ignored
/// bands leave; repeat until the waveform is within `vlim`. Returns each
/// band's gain relative to `s` (ignored bands 1), before the floor. The
/// ordinary solve starts from unity; the post-retarget solve (residuals
/// 5.2) from a lane-selective `seed`.
pub fn velocity_band_gains(
    vb: &[alloc::vec::Vec<f32>; 5],
    resid: &[f32],
    first: usize,
    vlim: f32,
    s: f32,
    active: (usize, usize),
    seed: [f32; 5],
) -> [f32; 5] {
    let mut g = seed;
    let n = vb[0].len();
    if first >= n {
        return g;
    }
    let is_active = |k: usize| k >= active.0 && k < active.1;
    loop {
        let (mut idx, mut worst) = (first, 0.0f32);
        for i in first..n {
            let mut y = if resid.is_empty() { 0.0f32 } else { resid[i] };
            for k in 0..5 {
                y += (g[k] * s) * vb[k][i];
            }
            if y.abs() > worst.abs() {
                worst = y;
                idx = i;
            }
        }
        if worst.abs() <= vlim || worst.is_nan() {
            break;
        }
        let sign = if worst >= 0.0 { 1.0f32 } else { -1.0 };
        let mut fixed = if resid.is_empty() { 0.0f32 } else { resid[idx] };
        for k in (0..5).filter(|&k| !is_active(k)) {
            fixed += (g[k] * s) * vb[k][idx];
        }
        let room = vlim - sign * fixed;
        let mut share = [0.0f32; 5];
        let mut sorted = [0.0f32; 5];
        let mut m = 0;
        for k in (0..5).filter(|&k| is_active(k)) {
            share[k] = (sign * s) * (g[k] * vb[k][idx]);
            sorted[m] = share[k];
            m += 1;
        }
        if m == 0 {
            break;
        }
        let sorted = &mut sorted[..m];
        sorted.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        let mut remain = room;
        let mut level = sorted[m - 1];
        for (j, &v) in sorted.iter().enumerate() {
            let count = (m - j) as f32;
            if v * count >= remain {
                level = remain / count;
                break;
            }
            remain -= v;
        }
        let mut changed = false;
        for k in (0..5).filter(|&k| is_active(k)) {
            if share[k] > level {
                let r = (level / share[k]).clamp(0.0, 1.0);
                let next = g[k] * r;
                if next != g[k] {
                    g[k] = next;
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    g
}

/// Signed velocity allocation against the retained full-band bound. The
/// retained prefix uses `old_limit`. A rising bound switches to `new_limit`
/// at `split`; a falling bound linearly hands off from old to new over the
/// remaining samples, with the first sample at `split` taking the first
/// interpolation step. Each pass selects the least index having the greatest
/// absolute trial value, then evaluates the bound only at that index.
/// `room_floor` is threshold headroom, but an effective cut still targets the
/// retained bound (never less than that floor).
#[allow(clippy::too_many_arguments)]
pub fn velocity_band_gains_window(
    vb: &[alloc::vec::Vec<f32>; 5],
    resid: &[f32],
    first: usize,
    old_limit: f32,
    new_limit: f32,
    split: usize,
    room_floor: f32,
    s: f32,
    active: (usize, usize),
    seed: [f32; 5],
) -> [f32; 5] {
    let n = vb[0].len();
    velocity_band_gains_with_limits(vb, resid, first, room_floor, s, active, seed, |i| {
        if i < split {
            old_limit
        } else if new_limit >= old_limit {
            new_limit
        } else {
            let count = n - split;
            let step = (new_limit - old_limit) / count as f32;
            old_limit + step * (i - split + 1) as f32
        }
    })
}

/// Signed velocity allocation against a producer window containing one
/// retained limit for every modeled sample.
#[allow(clippy::too_many_arguments)]
pub fn velocity_band_gains_limit_window(
    vb: &[alloc::vec::Vec<f32>; 5],
    resid: &[f32],
    first: usize,
    limits: &[f32],
    room_floor: f32,
    s: f32,
    active: (usize, usize),
    seed: [f32; 5],
) -> [f32; 5] {
    debug_assert_eq!(limits.len(), vb[0].len());
    velocity_band_gains_with_limits(vb, resid, first, room_floor, s, active, seed, |i| limits[i])
}

/// Choose the producer for a newly admitted retained-window segment.
///
/// Native retains the greater of the streaming protected envelope and the
/// decision-local protected trial. This is a producer comparison: shifted
/// history and a zero trial naturally leave the streaming source in control.
pub fn arbitrate_producer_segment(onset: f32, streaming: f32) -> f32 {
    onset.max(streaming)
}

#[allow(clippy::too_many_arguments)]
fn velocity_band_gains_with_limits(
    vb: &[alloc::vec::Vec<f32>; 5],
    resid: &[f32],
    first: usize,
    room_floor: f32,
    s: f32,
    active: (usize, usize),
    seed: [f32; 5],
    limit_at: impl Fn(usize) -> f32,
) -> [f32; 5] {
    let mut g = seed;
    let n = vb[0].len();
    if first >= n {
        return g;
    }
    let is_active = |k: usize| k >= active.0 && k < active.1;
    loop {
        let (mut idx, mut worst) = (first, 0.0f32);
        for i in first..n {
            let mut y = if resid.is_empty() { 0.0 } else { resid[i] };
            for k in 0..5 {
                y += (g[k] * s) * vb[k][i];
            }
            if y.abs() > worst.abs() {
                worst = y;
                idx = i;
            }
        }
        let limit = limit_at(idx);
        if worst.abs() <= limit + room_floor || worst.is_nan() {
            break;
        }
        let sign = if worst >= 0.0 { 1.0 } else { -1.0 };
        let mut fixed = if resid.is_empty() { 0.0 } else { resid[idx] };
        for k in (0..5).filter(|&k| !is_active(k)) {
            fixed += (g[k] * s) * vb[k][idx];
        }
        let room = (limit - sign * fixed).max(room_floor);
        let mut share = [0.0f32; 5];
        let mut sorted = [0.0f32; 5];
        let mut m = 0;
        for k in (0..5).filter(|&k| is_active(k)) {
            share[k] = (sign * s) * (g[k] * vb[k][idx]);
            sorted[m] = share[k];
            m += 1;
        }
        if m == 0 {
            break;
        }
        let sorted = &mut sorted[..m];
        sorted.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        let mut remain = room;
        let mut level = sorted[m - 1];
        for (j, &v) in sorted.iter().enumerate() {
            let count = (m - j) as f32;
            if v * count >= remain {
                level = remain / count;
                break;
            }
            remain -= v;
        }
        let mut changed = false;
        for k in (0..5).filter(|&k| is_active(k)) {
            if share[k] > level {
                let next = g[k] * (level / share[k]).clamp(0.0, 1.0);
                changed |= next != g[k];
                g[k] = next;
            }
        }
        if !changed {
            break;
        }
    }
    g
}

/// A fast band gain (residuals 5.1, decision-local): at every decision the
/// boundary gain is the start; a request below it is an attack decision (a
/// raised-cosine half-window of `len` samples to 0.99 x the request, then
/// hold for the rest of the decision); otherwise a recovery decision on the
/// 99 % response table realigned to the boundary,
/// `g[n] = start + r[n] (request - start)`, where the binary32 table is
/// generated by `r[n] = r[n-1] + lambda (1-r[n-1])` from zero.
/// Above 0.9988494 the gain snaps to unity when the request is unity.
#[derive(Clone, Copy, Debug)]
pub struct BandGain {
    pub g: f32,
    start: f32,
    target: f32,
    request: f32,
    n: u32,
    len: u32,
    response: f32,
    attack: bool,
}

impl BandGain {
    pub const fn unity(len: u32) -> Self {
        BandGain { g: 1.0, start: 1.0, target: 1.0, request: 1.0, n: 0, len, response: 0.0, attack: false }
    }
    /// The request of the current decision.
    pub fn requested(&self) -> f32 {
        self.request
    }
    /// The gain after `n` samples of a decision with request `req` and
    /// release `lambda`, state unchanged.
    pub fn end_after(&self, req: f32, n: usize, lambda: f64) -> f32 {
        let mut b = *self;
        b.decide(req);
        for _ in 0..n {
            b.sample(lambda);
        }
        b.g
    }
    /// A decision with final request `req`.
    pub fn decide(&mut self, req: f32) {
        self.request = req;
        self.start = self.g;
        self.n = 0;
        self.response = 0.0;
        self.attack = req < self.g;
        if self.attack {
            self.target = 0.99 * req;
        }
    }
    /// One sample; `lambda = ln(100) / (Fs T)` of the band's release time.
    #[inline]
    pub fn sample(&mut self, lambda: f64) -> f32 {
        if self.attack {
            if self.n < self.len {
                self.n += 1;
                let w = 0.5 * (1.0 + libm::cos(core::f64::consts::PI * self.n as f64 / self.len as f64));
                self.g = (self.target as f64 + (self.start as f64 - self.target as f64) * w) as f32;
            } else {
                self.g = self.target;
            }
        } else if self.start != self.request {
            self.n += 1;
            self.response += (lambda as f32) * (1.0 - self.response);
            self.response = self.response.min(1.0);
            self.g = self.start + self.response * (self.request - self.start);
            if self.request >= 1.0 && self.g > 0.998_849_4 {
                self.g = 1.0;
            }
        }
        self.g
    }
    pub fn finite(&self) -> bool {
        self.g.is_finite()
            && self.start.is_finite()
            && self.target.is_finite()
            && self.request.is_finite()
            && self.response.is_finite()
    }
}

/// The release rate of a 99 % settling time of `t` seconds at `rate`:
/// `ln(100) / (rate t)`.
pub fn release_lambda(rate: f64, t: f64) -> f64 {
    libm::log(100.0) / (rate * t)
}

/// Buckets of the masker's spectral lattice.
pub const MASK_BUCKETS: usize = 100;
/// The masker window and transform are capped at this many points.
pub const MASK_MAX_POINTS: usize = 1024;

/// The smallest supported transform length at least `window`:
/// `2^k`, `3 2^k`, `5 2^k` or `15 2^k`, at most [`MASK_MAX_POINTS`].
pub fn transform_len(window: usize) -> usize {
    let mut best = MASK_MAX_POINTS;
    for base in [1usize, 3, 5, 15] {
        let mut n = base;
        while n < window {
            n *= 2;
        }
        if n < best {
            best = n;
        }
    }
    best.max(window.min(MASK_MAX_POINTS))
}

/// The transform length the extracted lattice is defined for.
pub const MASK_BASE_POINTS: usize = 640;

/// The masker's 100-bucket partition for an `m`-point transform from the
/// extracted 640-point base (residuals 3): the first 99 exclusive ends
/// scaled by `m/640`, rounded to nearest with halves up, at least their
/// one-based bucket ordinal; the last end is `m/2 + 1`. `None` when the base
/// is not a valid lattice (strictly increasing, the last end 321).
pub fn mask_lattice_resized(base: &[u32; MASK_BUCKETS], m: usize) -> Option<Vec<usize>> {
    let valid = base.windows(2).all(|w| w[1] > w[0])
        && base[0] >= 1
        && base[MASK_BUCKETS - 1] as usize == MASK_BASE_POINTS / 2 + 1;
    if !valid {
        return None;
    }
    let bins = m / 2 + 1;
    let mut out: Vec<usize> = base[..MASK_BUCKETS - 1]
        .iter()
        .enumerate()
        .map(|(i, &e)| {
            let num = e as usize * m;
            let r = (2 * num + MASK_BASE_POINTS) / (2 * MASK_BASE_POINTS);
            r.max(i + 1).min(bins)
        })
        .collect();
    out.push(bins);
    Some(out)
}

/// In-place-free mixed-radix (2, 3, 5) complex FFT of a fixed length, all
/// tables built at construction.
pub struct Fft {
    n: usize,
    factors: Vec<usize>,
    /// `exp(-2 pi i k / n)`.
    tw: Vec<(f64, f64)>,
}

impl Fft {
    pub fn new(n: usize) -> Self {
        let mut factors = Vec::new();
        let mut m = n;
        for r in [4usize, 2, 3, 5] {
            while m % r == 0 {
                factors.push(r);
                m /= r;
            }
        }
        assert!(m == 1, "unsupported transform length");
        let tw = (0..n)
            .map(|k| {
                let a = -2.0 * PI * k as f64 / n as f64;
                (libm::cos(a), libm::sin(a))
            })
            .collect();
        Fft { n, factors, tw }
    }
    pub fn len(&self) -> usize {
        self.n
    }
    pub fn is_empty(&self) -> bool {
        self.n == 0
    }
    /// The transform of `x` (length `n`) into `out` (length `n`).
    pub fn forward(&self, x: &[(f64, f64)], out: &mut [(f64, f64)]) {
        self.rec(x, 0, 1, out, self.n, 0);
    }
    fn rec(&self, x: &[(f64, f64)], off: usize, stride: usize, out: &mut [(f64, f64)], n: usize, f: usize) {
        if n == 1 {
            out[0] = x[off];
            return;
        }
        let r = self.factors[f];
        let m = n / r;
        for q in 0..r {
            self.rec(x, off + q * stride, stride * r, &mut out[q * m..(q + 1) * m], m, f + 1);
        }
        // butterflies: X[k + p m] = sum_q W_n^{q k} Y_q[k] W_r^{q p}
        let step = self.n / n;
        let rstep = self.n / r;
        let mut t = [(0.0f64, 0.0f64); 5];
        for k in 0..m {
            for q in 0..r {
                let (a, b) = out[q * m + k];
                let (c, s) = self.tw[(q * k * step) % self.n];
                t[q] = (a * c - b * s, a * s + b * c);
            }
            for p in 0..r {
                let (mut re, mut im) = (0.0, 0.0);
                for q in 0..r {
                    let (c, s) = self.tw[((q * p) % r) * rstep];
                    re += t[q].0 * c - t[q].1 * s;
                    im += t[q].0 * s + t[q].1 * c;
                }
                out[k + p * m] = (re, im);
            }
        }
    }
}

/// The masker of one channel (residuals section 3): modeled velocity over
/// the retained window, the energy-normalised periodic Hann window, the
/// real transform, pressure-response weighting, the median bucket power in
/// the mask band and the shape penalty, then the level tracker and the
/// three-state follower.
pub struct Masker {
    win: Vec<f32>,
    weight: Vec<f32>,
    /// Exclusive bucket ends and the band's bucket range.
    ends: Vec<usize>,
    band: (usize, usize),
    fft: Fft,
    xin: Vec<(f64, f64)>,
    spec: Vec<(f64, f64)>,
    amp: Vec<f32>,
    pow: Vec<f32>,
    sel: Vec<f32>,
    /// Coefficients per controller frame: attack, release, fast attack.
    ca: f32,
    cr: f32,
    cf: f32,
    range: f32,
    shape: f32,
    clip: f32,
    slow: f32,
    fast: f32,
    pub level: f32,
}

impl Masker {
    /// `window` points of velocity, the pressure response `h`, the band in
    /// Hz, the follower times in seconds and range in dB, `frames` input
    /// frames per update.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        lattice: &[u32; MASK_BUCKETS],
        window: usize,
        rate: f64,
        h: &[f32],
        band_hz: [f32; 2],
        attack: f32,
        release: f32,
        fast: f32,
        range: f32,
        frames: usize,
    ) -> Option<Self> {
        let m = transform_len(window);
        let bins = m / 2 + 1;
        let w = window;
        let hann: Vec<f32> = (0..w).map(|n| r32(0.5 - 0.5 * libm::cos(2.0 * PI * n as f64 / w as f64))).collect();
        let norm = libm::sqrt(hann.iter().map(|&v| v as f64 * v as f64).sum::<f64>()) as f32;
        let win = hann.iter().map(|&v| v / norm).collect();
        let fft = Fft::new(m);
        let mut xin = vec![(0.0, 0.0); m];
        for (j, x) in xin.iter_mut().enumerate().take(h.len().min(m)) {
            *x = (h[j] as f64, 0.0);
        }
        let mut spec = vec![(0.0, 0.0); m];
        fft.forward(&xin, &mut spec);
        let weight = spec[..bins].iter().map(|&(a, b)| r32(libm::hypot(a, b))).collect();
        let ends = mask_lattice_resized(lattice, m)?;
        let hz = |b: usize| b as f64 * rate / m as f64;
        let nearest = |it: &mut dyn Iterator<Item = (usize, usize)>, f: f64| {
            let mut best = (f64::INFINITY, 0usize);
            for (i, b) in it {
                let d = libm::fabs(hz(b) - f);
                if d < best.0 {
                    best = (d, i);
                }
            }
            best.1
        };
        let lo = nearest(&mut (0..MASK_BUCKETS).map(|i| (i, if i == 0 { 0 } else { ends[i - 1] })), band_hz[0] as f64);
        let hi = nearest(&mut (0..MASK_BUCKETS - 1).map(|i| (i, ends[i])), band_hz[1] as f64);
        let band = (lo, hi.max(lo + 1));
        let coef = |t: f32| 1.0 - r32(libm::exp(r32(-(frames as f64) / (rate * t as f64)) as f64));
        Some(Masker {
            win,
            weight,
            ends,
            band,
            fft,
            xin,
            spec,
            amp: vec![0.0; bins],
            pow: vec![0.0; MASK_BUCKETS],
            sel: vec![0.0; MASK_BUCKETS],
            ca: coef(attack),
            cr: coef(release),
            cf: coef(fast),
            range,
            shape: 1.0,
            clip: 0.0,
            slow: 0.0,
            fast: 0.0,
            level: 0.0,
        })
    }

    pub fn clear(&mut self) {
        self.shape = 1.0;
        self.clip = 0.0;
        self.slow = 0.0;
        self.fast = 0.0;
        self.level = 0.0;
    }

    pub fn is_finite(&self) -> bool {
        [self.shape, self.clip, self.slow, self.fast, self.level].iter().all(|v| v.is_finite())
    }

    /// One update on the velocity window `v` (oldest first, the window's
    /// length): the new smoothed level in dB.
    pub fn update(&mut self, v: &[f32]) -> f32 {
        for (j, x) in self.xin.iter_mut().enumerate() {
            *x = if j < self.win.len() { ((v[j] * self.win[j]) as f64, 0.0) } else { (0.0, 0.0) };
        }
        self.fft.forward(&self.xin, &mut self.spec);
        let bins = self.amp.len();
        let (mut sum, mut sq) = (0.0f32, 0.0f32);
        for b in 0..bins {
            let (re, im) = self.spec[b];
            let a = r32(2.0 * libm::hypot(re, im)) * self.weight[b];
            self.amp[b] = a;
            sum += a;
            sq += a * a;
        }
        let mut lo = 0;
        for (i, &hi) in self.ends.iter().enumerate() {
            let s: f32 = self.amp[lo..hi].iter().map(|&a| a * a).sum();
            self.pow[i] = s / (hi - lo).max(1) as f32;
            lo = hi;
        }
        let (a, e) = self.band;
        let n = e - a;
        self.sel[..n].copy_from_slice(&self.pow[a..e]);
        let sel = &mut self.sel[..n];
        sel.sort_unstable_by(|x, y| x.partial_cmp(y).unwrap_or(core::cmp::Ordering::Equal));
        let median = if n % 2 == 1 { sel[n / 2] } else { (sel[n / 2 - 1] + sel[n / 2]) * 0.5 };
        let shape_now = libm::sqrtf(bins as f32) * libm::sqrtf(sq) / (sum + 1e-9);
        self.shape = 0.1 * self.shape + 0.9 * shape_now;
        let penalty = -4.0 * (self.shape.max(1.0) - 1.0).min(4.0);
        let raw = 10.0 * libm::log10f(median + 1e-30) + 97.0 + penalty;
        if raw > self.clip + 3.0 {
            self.clip = raw;
        } else if self.clip > raw + 12.0 {
            self.clip = raw + 12.0;
        }
        let c = self.clip;
        self.slow += (if c > self.slow { self.ca } else { self.cr }) * (c - self.slow);
        self.fast += self.cf * (c - self.fast);
        let mix = ((self.fast - self.slow) / self.range).clamp(0.0, 1.0);
        let alpha = if c <= self.level { self.cr } else { self.ca * (1.0 - mix) + self.cf * mix };
        self.level += alpha * (c - self.level);
        self.level
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform_lengths() {
        assert_eq!(transform_len(608), 640);
        assert_eq!(transform_len(512), 512);
        assert_eq!(transform_len(700), 768);
        assert_eq!(transform_len(2000), 1024);
    }

    #[test]
    fn fft_matches_a_direct_dft() {
        for n in [640usize, 768, 1024, 60] {
            let f = Fft::new(n);
            let x: Vec<(f64, f64)> =
                (0..n).map(|j| (libm::sin(j as f64 * 0.37) + 0.1 * j as f64 / n as f64, 0.0)).collect();
            let mut y = vec![(0.0, 0.0); n];
            f.forward(&x, &mut y);
            for k in [0usize, 1, 7, n / 3, n / 2, n - 1] {
                let (mut re, mut im) = (0.0, 0.0);
                for (j, &(a, _)) in x.iter().enumerate() {
                    let ang = -2.0 * PI * (j * k % n) as f64 / n as f64;
                    re += a * libm::cos(ang);
                    im += a * libm::sin(ang);
                }
                assert!((y[k].0 - re).abs() < 1e-9 && (y[k].1 - im).abs() < 1e-9, "n {n} k {k}");
            }
        }
    }

    #[test]
    fn the_lattice_resizes_and_refuses_a_bad_base() {
        // a synthetic base: 64 single bins, then widening buckets to 321
        let mut base = [0u32; MASK_BUCKETS];
        for (i, b) in base.iter_mut().enumerate() {
            *b = if i < 64 { i as u32 + 1 } else { 64 + ((i as u32 - 63) * 257).div_ceil(36) };
        }
        base[MASK_BUCKETS - 1] = 321;
        assert_eq!(
            mask_lattice_resized(&base, 640).unwrap().iter().map(|&v| v as u32).collect::<Vec<_>>(),
            base.to_vec()
        );
        let r = mask_lattice_resized(&base, 1024).unwrap();
        assert_eq!(r.len(), MASK_BUCKETS);
        assert_eq!(*r.last().unwrap(), 513);
        assert!(r.windows(2).all(|w| w[1] >= w[0]) && r[0] >= 1);
        let mut bad = base;
        bad[10] = bad[9];
        assert!(mask_lattice_resized(&bad, 640).is_none());
        assert!(mask_lattice_resized(&[0; MASK_BUCKETS], 640).is_none());
    }

    #[test]
    fn the_current_builder_keeps_one_over_r_at_dc_and_is_stable() {
        let (b, a) = current_coefficients(4.0, 0.8, 0.2e-3, 0.2, 1500.0, 48_000.0);
        assert_eq!(b.map(f32::to_bits), [0x3e7d_d90c, 0xbefa_d279, 0x3e78_9d32]);
        assert_eq!(a.map(f32::to_bits), [0xbffa_d6ed, 0x3f76_7f25]);
        let dc = (b[0] + b[1] + b[2]) as f64 / (1.0 + a[0] as f64 + a[1] as f64);
        assert!((dc * 4.0 - 1.0).abs() < 1e-4, "{dc}");
        assert!((a[1] as f64) < 1.0 && (a[1] as f64) > 0.0);
    }

    #[test]
    fn the_velocity_curve_extrapolates_past_the_last_anchor() {
        let k = [10.0, 20.0, 30.0, 40.0];
        let v = [0.5, 0.6, 1.0];
        assert_eq!(velocity_limit(0.0, &k, &v, 2.0), 0.5);
        assert!((velocity_limit(25.0, &k, &v, 2.0) - 0.8).abs() < 1e-6);
        assert!((velocity_limit(40.0, &k, &v, 2.0) - 2.0).abs() < 1e-6);
        assert!((velocity_limit(50.0, &k, &v, 2.0) - 3.0).abs() < 1e-6);
    }

    #[test]
    fn retained_velocity_bounds_switch_at_the_handoff() {
        let vb = [vec![2.0, 2.0], vec![0.0, 0.0], vec![0.0, 0.0], vec![0.0, 0.0], vec![0.0, 0.0]];
        let got = velocity_band_gains_window(&vb, &[], 0, 1.0, 2.0, 1, 0.1, 1.0, (0, 5), [1.0; 5]);
        assert_eq!(got, [0.5, 1.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn retained_velocity_uses_the_limit_at_the_selected_window_sample() {
        let vb = [vec![1.0, 2.0, 1.5], vec![0.0; 3], vec![0.0; 3], vec![0.0; 3], vec![0.0; 3]];
        let got = velocity_band_gains_limit_window(&vb, &[], 0, &[0.2, 1.0, 0.3], 0.0, 1.0, (0, 5), [1.0; 5]);
        assert_eq!(got, [0.5, 1.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn producer_arbitration_retains_the_greater_source() {
        assert_eq!(arbitrate_producer_segment(0.20, 0.56), 0.56);
        assert_eq!(arbitrate_producer_segment(0.62, 0.59), 0.62);
        assert_eq!(arbitrate_producer_segment(0.0, 0.15), 0.15);
        assert_eq!(arbitrate_producer_segment(0.20, 0.20), 0.20);
    }

    #[test]
    fn a_falling_retained_bound_starts_interpolation_one_sample_after_the_handoff() {
        let vb = [vec![0.0, 0.0, 1.6, 0.0], vec![0.0; 4], vec![0.0; 4], vec![0.0; 4], vec![0.0; 4]];
        let got = velocity_band_gains_window(&vb, &[], 0, 2.0, 1.0, 2, 0.0, 1.0, (0, 5), [1.0; 5]);
        // The first post-split sample uses old + (new - old) / 2 = 1.5.
        assert_eq!(got, [0.9375, 1.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn a_varying_bound_is_checked_at_the_first_largest_magnitude() {
        let vb = [vec![0.0, 0.0, 1.6, 1.6], vec![0.0; 4], vec![0.0; 4], vec![0.0; 4], vec![0.0; 4]];
        let got = velocity_band_gains_window(&vb, &[], 0, 2.0, 1.0, 2, 0.0, 1.0, (0, 5), [1.0; 5]);
        // The limits are 1.5 then 1.0. The first pass selects sample 2 by
        // least-index tie handling and cuts to 1.5. The next pass selects the
        // same equal-magnitude sample and stops at its limit; native does not
        // instead maximize excess or ratio and therefore does not cut to
        // sample 3's 1.0 limit.
        assert_eq!(got, [0.9375, 1.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn band_release_restarts_a_binary32_response_table_each_decision() {
        let lambda = release_lambda(48_000.0, 0.2);
        let mut gain = BandGain::unity(160);
        gain.g = 0.4;
        gain.decide(0.8);

        let mut response = 0.0f32;
        for _ in 0..256 {
            response += (lambda as f32) * (1.0 - response);
            let expected = 0.4 + response * (0.8 - 0.4);
            assert_eq!(gain.sample(lambda).to_bits(), expected.to_bits());
        }

        let start = gain.g;
        gain.decide(0.8);
        let first = (lambda as f32) * (1.0 - 0.0f32);
        assert_eq!(gain.sample(lambda).to_bits(), (start + first * (0.8 - start)).to_bits());
    }

    #[test]
    fn the_temperature_candidate_is_unity_below_the_soft_limit_and_cuts_above() {
        assert_eq!(temperature_candidate(100.0, 40.0, 20.0, 10.0, 40.0, 110.0, 125.0, 30.0, 20.0), 1.0);
        let g = temperature_candidate(130.0, 70.0, 30.0, 20.0, 40.0, 110.0, 125.0, 30.0, 20.0);
        assert!(g < 1.0 && g > 0.0);
    }
}
