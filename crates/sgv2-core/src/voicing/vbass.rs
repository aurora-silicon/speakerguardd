//! Virtual bass (`aufx/vben`): a mid/side bass box.
//!
//! Behaviour (clean-room, from the box's behaviour specification):
//!
//! ```text
//!  per channel c:  hp_c  = AP_u(AP_l(HP(x_c)))
//!  mid M = (L+R)/2: lo    = LP(M)
//!                   low   = s_b * blend * AP_u(AP_l(lo))
//!                   harm  = s_h * LP_u(COMP(gain * LC(nld, HP_l, lo)))
//!                   mid   = SUB(delay(low + harm))
//!  y_c = out_gain * (hp_c + mid), `latency` frames late
//! ```
//!
//! - `HP`/`LP` are a Linkwitz-Riley pair at the crossover `fc`; `AP_u` and
//!   `AP_l` are the all-pass sections of the harmonic band's upper (`fu`)
//!   and lower (`fc`) edges; `HP_l`/`LP_u` are that band's Linkwitz-Riley
//!   edges. Order parameters round to the nearest integer, then up to even.
//! - `s_b` inverts the blend for an odd-order crossover; `s_h` inverts the
//!   harmonics when the crossover and the lower band edge have Butterworth
//!   orders of the same parity.
//! - `nld` is the odd saturating device of the saved type. `LC` is the
//!   level control: off, it is `HP_l(nld(lo))`; on, the device input is
//!   normalised by a peak follower and its band-limited output rescaled by
//!   a second one (see [`law`]).
//! - `COMP` is the optional soft-knee harmonic compressor. Its peak follower
//!   smooths the requested dB gain reduction before applying it. The box runs
//!   it after harmonic gain and before the upper edge's second Butterworth
//!   half (see [`law`]).
//! - `SUB` band-limits the mid branches (a low pass designed at a quarter
//!   of the rate); each odd-order band-edge all-pass adds one frame of
//!   output latency and a further branch delay.
//! - With the level control on and an odd lower band edge, that edge's
//!   frame of branch delay sits in front of the device: the device reads the
//!   previous selected-rate frame, driven by that frame's input level, while
//!   the output follower's restore uses the current input level (one frame
//!   ahead of the device). The branch's total delay is unchanged.
//! - Every filter is one platform single-precision multichannel-biquad
//!   object ([`crate::vdsp`]) holding Float32 section words, called once per
//!   render call over the call's frames: the stereo crossover high pass, the
//!   stereo band-edge all-passes, the mono crossover low pass, and at the
//!   selected rate the blend all-passes, the harmonic band (lower edge twice,
//!   then the first upper half) and the second upper half, then the mono
//!   reconstruction low pass. The selected-rate objects run once per call over
//!   that call's selected frames (the decimation phase carries across calls).
//! - Section words come from the unit's own Float32 Butterworth construction
//!   ([`butter_words`]): the corner is limited to 0.45 of the rate, the
//!   angular frequency is rounded once, its sine and cosine are rounded, and
//!   every pole and numerator word is formed in Float32 from those; a
//!   first-order section uses the rounded cotangent of half the angle. The
//!   numerators are derived from the rounded poles, so no ULP adjustment is
//!   needed anywhere.
//! - Odd sections carry their own delay and polarity: an odd all-pass is the
//!   one-frame-delayed form, the mono low pass embeds the mid average and the
//!   crossover polarity (`+-1/channels` on its second half's first section),
//!   the harmonic band's odd lower edge is delayed and inverted, the second
//!   upper half's odd section delayed. The blend gain scales the blend
//!   all-pass's first numerator; the harmonic gain sits between the two upper
//!   halves. Zero insertion keeps the selected samples as they are; the
//!   reconstruction low pass carries the stride's gain in its first section.
//! - The upper band corner's internal word is the Float32 value just above
//!   the saved one (the saved-state import rounds it up).
//! - The high-pass path runs independently on every L/R channel. In M/S
//!   terms both `HP(M)` and `HP(S)` survive; only the reconstructed low and
//!   harmonic branches are mono and added equally to every channel.
//!
//! No tuning value lives here: corners, orders and gains come in [`Params`]
//! from the Mac's own saved state; [`law`] holds the box's own constants.
use alloc::vec;
use alloc::vec::Vec;

use crate::platform_math;

/// `ln 10 / 20` in Float32: the compressor's dB-to-exponent factor.
const LN10_OVER_20: f32 = f32::from_bits(0x3deb_c8e3);

/// One designed biquad (double): the reconstruction design and tests.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Bq {
    b: [f64; 3],
    a: [f64; 2],
}

/// A maximum-order high pass has ten Linkwitz-Riley sections followed by two
/// five-section phase companions. Keep the complete mono cascade.
const MAX_SECTIONS: usize = 20;

#[derive(Clone, Debug, Default)]
struct Cascade {
    n: usize,
    s: [Bq; MAX_SECTIONS],
}

impl Cascade {
    fn push(&mut self, q: Bq) {
        if self.n < MAX_SECTIONS {
            self.s[self.n] = q;
            self.n += 1;
        }
    }
}

/// One stored section: `[b0, b1, b2, a1, a2]` as the object keeps it.
pub(crate) type Words = [f32; 5];

/// The platform voicing library's Float32 Butterworth sections of order `n`
/// at `fc` for a filter object at rate `fs` (both as the box keeps them,
/// Float32), in pole-pair order with the first-order section last for odd
/// `n`. VirtualBass and the ToneMeister's Butterworth slots share it. The
/// words go to `out` one by one (no allocation of its own).
pub(crate) fn butter_words(n: usize, fc: f32, fs: f32, kind: Kind, out: &mut impl Extend<Words>) {
    let f = fc.min((fs * 0.5) * 0.9);
    let w = (f64::from(f) * (2.0 * core::f64::consts::PI) / f64::from(fs)) as f32;
    let s = libm::sin(f64::from(w)) as f32;
    let c = libm::cos(f64::from(w)) as f32;
    for i in 1..=n / 2 {
        let t = libm::sin((2 * i - 1) as f64 * core::f64::consts::PI / (2 * n) as f64) as f32;
        let p = t * s;
        let a2 = (1.0 - p) / (1.0 + p);
        let u = 1.0 + a2;
        let uc = u * c;
        let a1 = -uc;
        out.extend([match kind {
            Kind::Low => {
                let g = (u - uc) * 0.25;
                [g, g + g, g, a1, a2]
            }
            Kind::High => {
                let g = (u - uc) * 0.25;
                high_from_low([g, g + g, g], a1, a2)
            }
            Kind::AllPass => [a2, a1, 1.0, a1, a2],
        }]);
    }
    if n % 2 == 1 {
        let q = (1.0 / libm::tan(f64::from(w) * 0.5)) as f32;
        let a1 = (1.0 - q) / (1.0 + q);
        out.extend([match kind {
            Kind::Low => {
                let g = 1.0 / (2.0 / (1.0 + a1));
                [g, g, 0.0, a1, 0.0]
            }
            Kind::High => {
                let g = 1.0 / (2.0 / (1.0 + a1));
                high_from_low([g, g, 0.0], a1, 0.0)
            }
            // the one-frame-delayed first-order all-pass
            Kind::AllPass => [0.0, a1, 1.0, a1, 0.0],
        }]);
    }
}

/// A high-pass section from the low-pass section on the same poles, as the
/// platform library forms it: `z -> -z` on the stored low-pass numerator,
/// then that numerator scaled by the Float32 ratio of the denominator at
/// Nyquist to the low-pass numerator's sum at DC, so the section has unity
/// gain at Nyquist. The ratio is rounded on its own before it scales each
/// word, so the words can differ by a unit in the last place from the
/// closed form `(1 - a1 + a2) / 4`.
fn high_from_low(lp: [f32; 3], a1: f32, a2: f32) -> Words {
    let d = (1.0 - a1) + a2;
    let n = (lp[0] + lp[1]) + lp[2];
    let k = d / n;
    [lp[0] * k, -(lp[1] * k), lp[2] * k, a1, a2]
}

/// Scales a stored section's numerator (each word rounded on its own).
fn scale_numerator(q: &mut Words, k: f32) {
    for b in &mut q[..3] {
        *b *= k;
    }
}

/// A platform object from stored words.
fn object(words: &[Words], multichannel: bool) -> crate::vdsp::Cascade {
    let mut c = [[0.0f64; 5]; crate::vdsp::MAX_SECTIONS];
    let n = words.len().min(c.len());
    for (dst, w) in c.iter_mut().zip(&words[..n]) {
        *dst = w.map(f64::from);
    }
    crate::vdsp::Cascade::new(&c[..n], multichannel)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Kind {
    Low,
    High,
    AllPass,
}

/// Minimal complex arithmetic for the elliptic design (setup only).
#[derive(Clone, Copy, Debug, PartialEq)]
struct C {
    re: f64,
    im: f64,
}

impl C {
    const fn new(re: f64, im: f64) -> C {
        C { re, im }
    }
    fn add(self, o: C) -> C {
        C::new(self.re + o.re, self.im + o.im)
    }
    fn sub(self, o: C) -> C {
        C::new(self.re - o.re, self.im - o.im)
    }
    fn mul(self, o: C) -> C {
        C::new(self.re * o.re - self.im * o.im, self.re * o.im + self.im * o.re)
    }
    fn div(self, o: C) -> C {
        let d = o.re * o.re + o.im * o.im;
        C::new((self.re * o.re + self.im * o.im) / d, (self.im * o.re - self.re * o.im) / d)
    }
    fn scale(self, k: f64) -> C {
        C::new(self.re * k, self.im * k)
    }
    fn abs(self) -> f64 {
        libm::hypot(self.re, self.im)
    }
    fn sqrt(self) -> C {
        let r = libm::sqrt(self.abs());
        let t = libm::atan2(self.im, self.re) / 2.0;
        C::new(r * libm::cos(t), r * libm::sin(t))
    }
    fn sin(self) -> C {
        C::new(libm::sin(self.re) * libm::cosh(self.im), libm::cos(self.re) * libm::sinh(self.im))
    }
    fn cos(self) -> C {
        C::new(libm::cos(self.re) * libm::cosh(self.im), -libm::sin(self.re) * libm::sinh(self.im))
    }
    fn ln(self) -> C {
        C::new(libm::log(self.abs()), libm::atan2(self.im, self.re))
    }
    /// acos z = -i ln(z + i sqrt(1 - z^2))
    fn acos(self) -> C {
        let one = C::new(1.0, 0.0);
        let i = C::new(0.0, 1.0);
        let w = self.add(i.mul(one.sub(self.mul(self)).sqrt())).ln();
        C::new(w.im, -w.re)
    }
}

/// Descending Landen moduli of `k`.
const LANDEN: usize = 8;
fn landen(k: f64) -> [f64; LANDEN] {
    let mut v = [0.0; LANDEN];
    let mut prev = k;
    for x in v.iter_mut() {
        let t = prev / (1.0 + libm::sqrt(1.0 - prev * prev));
        *x = t * t;
        prev = *x;
    }
    v
}

/// Jacobi `cd(u K, k)` (or `sn` with `sine`) by descending Landen steps.
fn jacobi(u: C, k: f64, sine: bool) -> C {
    let v = landen(k);
    let half = u.scale(core::f64::consts::FRAC_PI_2);
    let mut w = if sine { half.sin() } else { half.cos() };
    let one = C::new(1.0, 0.0);
    for &vn in v.iter().rev() {
        w = w.scale(1.0 + vn).div(one.add(w.mul(w).scale(vn)));
    }
    w
}

/// Inverse of `sn(u K, k)`: the `u`.
fn asne(w: C, k: f64) -> C {
    let v = landen(k);
    let one = C::new(1.0, 0.0);
    let mut w = w;
    let mut prev = k;
    for &vn in v.iter() {
        w = w.div(one.add(one.sub(w.mul(w).scale(prev * prev)).sqrt())).scale(2.0 / (1.0 + vn));
        prev = vn;
    }
    // u = 1 - (2/pi) acos(w)
    one.sub(w.acos().scale(2.0 / core::f64::consts::PI))
}

/// Elliptic (Cauer) low pass of odd order `n` (unity gain at DC), pass-band
/// ripple `rp_db`, stop-band attenuation `rs_db`, edge `fp`, by the bilinear
/// transform at `fs` with the edge pre-warped.
fn elliptic(n: usize, rp_db: f64, rs_db: f64, fp: f64, fs: f64) -> Cascade {
    let ep = libm::sqrt(libm::pow(10.0, rp_db / 10.0) - 1.0);
    let es = libm::sqrt(libm::pow(10.0, rs_db / 10.0) - 1.0);
    let k1 = ep / es;
    let pairs = n / 2;
    // degree equation
    let kp1 = libm::sqrt(1.0 - k1 * k1);
    let mut prod = 1.0;
    for i in 1..=pairs {
        let ui = (2 * i - 1) as f64 / n as f64;
        prod *= jacobi(C::new(ui, 0.0), kp1, true).re;
    }
    let kp = libm::pow(kp1, n as f64) * libm::pow(prod, 4.0);
    let k = libm::sqrt(1.0 - kp * kp);
    let wp = libm::tan(core::f64::consts::PI * fp / fs);
    let j = C::new(0.0, 1.0);
    let v0 = j.mul(asne(j.scale(1.0 / ep), k1)).scale(-1.0 / n as f64);
    let mut c = Cascade::default();
    for i in 1..=pairs {
        let ui = C::new((2 * i - 1) as f64 / n as f64, 0.0);
        let zeta = jacobi(ui, k, false).re;
        let wz = wp / (k * zeta); // zeros at +-j wz
        let pole = j.mul(jacobi(ui.sub(j.mul(v0)), k, false)).scale(wp);
        let (a, cc) = (wz * wz, pole.re * pole.re + pole.im * pole.im);
        let b = -2.0 * pole.re;
        let g = cc / a;
        let d0 = 1.0 + b + cc;
        c.push(Bq {
            b: [g * (1.0 + a) / d0, g * 2.0 * (a - 1.0) / d0, g * (1.0 + a) / d0],
            a: [2.0 * (cc - 1.0) / d0, (1.0 - b + cc) / d0],
        });
    }
    if n % 2 == 1 {
        // The first-order section's numerator is the band edge, not its own
        // pole; the first pole pair takes the difference so the cascade keeps
        // unity gain at DC.
        let p0 = j.mul(jacobi(j.mul(v0), k, true)).scale(wp).re;
        let d0 = 1.0 - p0;
        c.push(Bq { b: [wp / d0, wp / d0, 0.0], a: [-(1.0 + p0) / d0, 0.0] });
        if c.n > 1 {
            let back = -p0 / wp;
            for b in &mut c.s[0].b {
                *b *= back;
            }
        }
    }
    c
}

/// The nonlinear devices carried. The compiler refuses the other types.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Device {
    /// Type 3: an odd saturating curve (see [`law`]).
    Saturator3,
}

impl Device {
    fn code(self) -> f64 {
        match self {
            Device::Saturator3 => 3.0,
        }
    }
    fn from_code(c: f64) -> Option<Device> {
        if c == 3.0 {
            Some(Device::Saturator3)
        } else {
            None
        }
    }
}

/// The box's own laws: the same on every Mac, never from a tuning.
pub mod law {
    /// Type 3 device: `-AMP * tanh(SLOPE * u)` (SPEC-vben §2, type 3; the
    /// sign is carried by the branch polarity rule).
    pub const DEVICE3_AMP: f64 = 3.8;
    pub const DEVICE3_SLOPE: f64 = 0.8;
    /// Level control: one peak follower on the device input sets the
    /// drive `1 / max(peak, LC_FLOOR)` (input clipped to `LC_CLIP`); a
    /// second one on the band-limited device output rescales it by
    /// `peak_in / max(peak_out, LC_FLOOR)`.
    /// SPEC-vben §1 (floor, clip).
    pub const LC_FLOOR: f64 = 0.05;
    pub const LC_CLIP: f64 = 1.2;
    /// Upper clip of both level-control followers' outputs.
    pub const LC_CEILING: f64 = 1000.0;
    /// Follower: attack and release one-poles, a hold that restarts while
    /// the input stays within `LC_HOLD_RATIO` of the level, and a smoothing
    /// one-pole that follows a rising level at the attack rate.
    /// SPEC-vben §1 (follower constants).
    pub const LC_ATTACK_S: f64 = 50e-6;
    pub const LC_RELEASE_S: f64 = 0.5;
    pub const LC_HOLD_S: f64 = 0.5;
    pub const LC_HOLD_RATIO: f64 = 0.9;
    pub const LC_SMOOTH_S: f64 = 0.5;
    /// Retained envelope and smoothing stores start at this linear level on
    /// construction and reset.  The follower floor is applied later, at the
    /// two level-control divisions; it is not the state seed.
    pub const LC_SEED: f64 = 1e-3;
    /// Harmonic compressor. The detector converts `abs(x) + epsilon` to dB,
    /// computes a conventional soft-knee gain-reduction request, then runs
    /// that non-negative request through the same peak-follower topology as
    /// the level control. These are the box defaults; the saved state exposes
    /// only enable, threshold and ratio.
    pub const COMP_DETECTOR_EPSILON: f64 = 1e-6;
    pub const COMP_KNEE_DB: f64 = 10.0;
    pub const COMP_ATTACK_S: f64 = 50e-6;
    pub const COMP_RELEASE_S: f64 = 0.05;
    pub const COMP_HOLD_S: f64 = 0.03;
    pub const COMP_HOLD_RATIO: f64 = 0.9;
    pub const COMP_SMOOTH_S: f64 = 0.05;
    /// The mid branches run at the selected rate (SPEC-vben §3): the
    /// crossover low pass at the stream rate, every `fs / SUB_RATE_HZ`-th
    /// frame taken, the
    /// rest at the selected rate, then zero insertion and an elliptic
    /// reconstruction low pass at the stream rate.
    pub const SUB_RATE_HZ: f64 = 8_000.0;
    pub const RECON_ORDER: usize = 7;
    pub const RECON_RIPPLE_DB: f64 = 1.0;
    pub const RECON_STOP_DB: f64 = 100.0;
    pub const RECON_EDGE_HZ: f64 = 2_000.0;
}

/// Largest stream-to-selected rate ratio carried.
const STRIDE_MAX: usize = 24;

/// Frames per selected-rate frame at `fs`, if `fs` is a whole multiple of
/// the selected rate.
pub fn sub_stride(fs: f64) -> Option<usize> {
    let r = fs / law::SUB_RATE_HZ;
    (r >= 1.0 && libm::trunc(r) == r && r as usize <= STRIDE_MAX).then_some(r as usize)
}

#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    /// Crossover (high-pass) corner, Hz, and its Linkwitz-Riley order (even).
    pub fc_hz: f64,
    pub hp_order: usize,
    /// Harmonic band upper corner, Hz, and the band's upper/lower orders
    /// (even).
    pub fu_hz: f64,
    pub upper_order: usize,
    pub lower_order: usize,
    /// Linear gains.
    pub harmonic_gain: f64,
    pub blend_gain: f64,
    pub input_gain: f64,
    pub output_gain: f64,
    pub device: Device,
    /// Level control in front of the device.
    pub level_control: bool,
    /// Optional soft-knee compressor on the harmonic branch.
    pub harmonic_compressor: bool,
    pub compressor_threshold_db: f64,
    pub compressor_ratio: f64,
}

impl Params {
    pub const WORDS: usize = 14;

    pub fn to_words(&self, out: &mut [f64]) {
        let w = [
            self.fc_hz,
            self.hp_order as f64,
            self.fu_hz,
            self.upper_order as f64,
            self.lower_order as f64,
            self.harmonic_gain,
            self.blend_gain,
            self.input_gain,
            self.output_gain,
            self.device.code(),
            if self.level_control { 1.0 } else { 0.0 },
            if self.harmonic_compressor { 1.0 } else { 0.0 },
            self.compressor_threshold_db,
            self.compressor_ratio,
        ];
        out[..Self::WORDS].copy_from_slice(&w);
    }

    pub fn from_words(w: &[f64]) -> Option<Params> {
        if w.len() < Self::WORDS || w[..Self::WORDS].iter().any(|v| !v.is_finite()) {
            return None;
        }
        let order = |v: f64| -> Option<usize> {
            ((2.0..=20.0).contains(&v) && libm::trunc(v) == v && (v as usize) % 2 == 0).then_some(v as usize)
        };
        let corner = |v: f64| (v > 0.0 && v < 20_000.0).then_some(v);
        let gain = |v: f64| (0.0..=100.0).contains(&v).then_some(v);
        let level_control = if w[10] == 0.0 {
            false
        } else if w[10] == 1.0 {
            true
        } else {
            return None;
        };
        let harmonic_compressor = if w[11] == 0.0 {
            false
        } else if w[11] == 1.0 {
            true
        } else {
            return None;
        };
        let compressor_threshold_db = w[12];
        let compressor_ratio = w[13];
        if !(-100.0..=100.0).contains(&compressor_threshold_db) || !(1.0..=100.0).contains(&compressor_ratio) {
            return None;
        }
        Some(Params {
            fc_hz: corner(w[0])?,
            hp_order: order(w[1])?,
            fu_hz: corner(w[2])?,
            upper_order: order(w[3])?,
            lower_order: order(w[4])?,
            harmonic_gain: gain(w[5])?,
            blend_gain: gain(w[6])?,
            input_gain: gain(w[7])?,
            output_gain: gain(w[8])?,
            device: Device::from_code(w[9])?,
            level_control,
            harmonic_compressor,
            compressor_threshold_db,
            compressor_ratio,
        })
    }
}

/// Peak follower of the level control.
#[derive(Clone, Copy, Debug)]
struct Follower {
    // single precision, as the box's own follower state
    ratio: f32,
    /// Closest hold-restart test to macOS's own threshold since last
    /// drained (relative margin), for the differential rig.
    tie: f32,
    e: f32,
    s: f32,
    held: usize,
    hold: usize,
    a_att: f32,
    a_rel: f32,
    a_smo: f32,
    /// The level control's stores update with separate Float32 multiplies
    /// and adds; the harmonic compressor's detector keeps its fused update.
    fused: bool,
}

impl Follower {
    /// A level-control follower: its stores start at the level-control
    /// seed. The harmonic compressor's follower starts from zero.
    fn new(fs: f64, ratio: f64) -> Follower {
        let mut f =
            Follower::with_times(fs, ratio, law::LC_ATTACK_S, law::LC_RELEASE_S, law::LC_HOLD_S, law::LC_SMOOTH_S);
        f.e = law::LC_SEED as f32;
        f.s = law::LC_SEED as f32;
        f.fused = false;
        f
    }

    fn with_times(fs: f64, ratio: f64, attack_s: f64, release_s: f64, hold_s: f64, smooth_s: f64) -> Follower {
        let fs = fs as f32;
        let pole = |t: f64| crate::platform_math::expf(-1.0 / (t as f32 * fs));
        Follower {
            ratio: ratio as f32,
            tie: f32::INFINITY,
            e: 0.0,
            s: 0.0,
            held: 0,
            hold: (hold_s as f32 * fs) as usize + 1,
            a_att: pole(attack_s),
            a_rel: pole(release_s),
            a_smo: pole(smooth_s),
            fused: true,
        }
    }
    #[inline]
    fn run(&mut self, x: f64) -> f64 {
        let u = libm::fabsf(x as f32);
        let (a, b) = if u > self.e {
            self.held = 0;
            (self.a_att, if self.e > self.s { self.a_att } else { self.a_smo })
        } else if u < self.e {
            if self.held < self.hold {
                self.held += 1;
                let nominal = law::LC_HOLD_RATIO as f32 * self.e;
                if nominal > 0.0 {
                    self.tie = self.tie.min(libm::fabsf(u - nominal) / nominal);
                }
                if u >= self.ratio * self.e {
                    self.held = 0;
                }
                (1.0, self.a_smo)
            } else {
                (self.a_rel, self.a_smo)
            }
        } else {
            (self.a_rel, self.a_smo)
        };
        // The retained stores and both associations are Float32.
        if self.fused {
            self.e = libm::fmaf(a, self.e, (1.0 - a) * u);
            self.s = libm::fmaf(b, self.s, (1.0 - b) * self.e);
        } else {
            self.e = a * self.e + (1.0 - a) * u;
            self.s = b * self.s + (1.0 - b) * self.e;
        }
        self.s as f64
    }
}

#[derive(Clone, Copy, Debug)]
struct HarmonicCompressor {
    threshold_db: f32,
    ratio: f32,
    follower: Follower,
}

impl HarmonicCompressor {
    fn new(fs: f64, threshold_db: f64, ratio: f64) -> HarmonicCompressor {
        HarmonicCompressor {
            threshold_db: threshold_db as f32,
            ratio: ratio as f32,
            follower: Follower::with_times(
                fs,
                law::COMP_HOLD_RATIO,
                law::COMP_ATTACK_S,
                law::COMP_RELEASE_S,
                law::COMP_HOLD_S,
                law::COMP_SMOOTH_S,
            ),
        }
    }

    #[inline]
    fn run(&mut self, x: f64) -> f64 {
        let x = x as f32;
        let level_db = 20.0 * platform_math::vlog10f(libm::fabsf(x) + law::COMP_DETECTOR_EPSILON as f32);
        let d = level_db - self.threshold_db;
        let half_knee = 0.5 * law::COMP_KNEE_DB as f32;
        let slope = 1.0 - 1.0 / self.ratio;
        let requested_db = if d > half_knee {
            slope * d
        } else if libm::fabsf(d) <= half_knee {
            slope * (d + half_knee) * (d + half_knee) / (2.0 * law::COMP_KNEE_DB as f32)
        } else {
            0.0
        };
        let reduction_db = self.follower.run(requested_db as f64) as f32;
        (x * platform_math::vexpf(-reduction_db * LN10_OVER_20)) as f64
    }
}

/// Level control state (only used when the saved state switches it on).
#[derive(Clone, Copy, Debug)]
struct LevelControl {
    input: Follower,
    output: Follower,
}

impl LevelControl {
    fn new(fs: f64) -> LevelControl {
        LevelControl { input: Follower::new(fs, law::LC_HOLD_RATIO), output: Follower::new(fs, law::LC_HOLD_RATIO) }
    }
}

pub struct Run {
    p: Params,
    fs: f64,
    channels: usize,
    /// Stream rate, every channel: the crossover high pass, then the lower
    /// and upper band-edge all-passes.
    hp: Vec<crate::vdsp::Cascade>,
    ap: Vec<crate::vdsp::Cascade>,
    /// Stream rate, mono: the crossover low pass of the channel sum.
    lp: crate::vdsp::Cascade,
    /// Selected rate, mono: blend all-passes, harmonic band, second upper half.
    blend: crate::vdsp::Cascade,
    band: crate::vdsp::Cascade,
    upper: crate::vdsp::Cascade,
    /// Stream rate, mono: reconstruction low pass.
    recon: crate::vdsp::Cascade,
    lc: LevelControl,
    /// Selected-rate frames the device runs behind the input follower
    /// (1 with the level control on and an odd lower band edge, else 0).
    lead: usize,
    /// The previous frame's device input and its input level.
    prev_lo: f64,
    prev_p_in: f64,
    compressor: HarmonicCompressor,
    stride: usize,
    /// Position in the selected-rate grid (carried across calls).
    phase: usize,
    latency: usize,
    /// Storage made with the graph and reused by real-time processing.
    work: Vec<Vec<f32>>,
    mono: Vec<f32>,
    picked: Vec<f32>,
    at: Vec<usize>,
    low: Vec<f32>,
    harm: Vec<f32>,
    p_in: Vec<f64>,
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize, max_frames: usize) -> Run {
        Self::build(p, fs, channels, max_frames, None)
    }

    /// `lead`: test hook forcing the level-control lead off.
    fn build(p: &Params, fs: f64, channels: usize, max_frames: usize, lead: Option<usize>) -> Run {
        let stride = sub_stride(fs).unwrap_or(1);
        let (fs32, fs8) = (fs as f32, (fs / stride as f64) as f32);
        let (nh, nu, nl) = (p.hp_order / 2, p.upper_order / 2, p.lower_order / 2);
        let fc = p.fc_hz as f32;
        let fu = f32::from_bits((p.fu_hz as f32).to_bits() + 1);
        let channels = channels.max(1);
        let lead = lead.unwrap_or(if p.level_control { nl % 2 } else { 0 });
        let multi = channels >= 2;

        let mut hp = Vec::new();
        butter_words(nh, fc, fs32, Kind::High, &mut hp);
        let half = hp.clone();
        hp.extend_from_slice(&half);
        let mut ap = Vec::new();
        butter_words(nl, fc, fs32, Kind::AllPass, &mut ap);
        butter_words(nu, fu, fs32, Kind::AllPass, &mut ap);

        let mut lp = Vec::new();
        butter_words(nh, fc, fs32, Kind::Low, &mut lp);
        let k = lp.len();
        let half = lp.clone();
        lp.extend_from_slice(&half);
        let avg = 1.0 / channels as f32;
        scale_numerator(&mut lp[k], if nh % 2 == 1 { -avg } else { avg });

        let mut blend = Vec::new();
        butter_words(nl, fc, fs8, Kind::AllPass, &mut blend);
        butter_words(nu, fu, fs8, Kind::AllPass, &mut blend);
        scale_numerator(&mut blend[0], p.blend_gain as f32);

        let mut band = Vec::new();
        butter_words(nl, fc, fs8, Kind::High, &mut band);
        let half = band.clone();
        band.extend_from_slice(&half);
        if nl % 2 == 1 {
            // The odd lower edge's frame of delay and its polarity; with the
            // lead that frame sits in front of the device instead.
            let last = band.len() - 1;
            let g = band[last][0];
            band[last][..3].copy_from_slice(&if lead > 0 { [-g, g, 0.0] } else { [0.0, -g, g] });
        }
        butter_words(nu, fu, fs8, Kind::Low, &mut band);
        let mut upper = Vec::new();
        butter_words(nu, fu, fs8, Kind::Low, &mut upper);
        if nu % 2 == 1 {
            let last = upper.len() - 1;
            let g = upper[last][0];
            upper[last][..3].copy_from_slice(&[0.0, g, g]);
        }

        let rd = elliptic(law::RECON_ORDER, law::RECON_RIPPLE_DB, law::RECON_STOP_DB, law::RECON_EDGE_HZ, fs);
        let mut recon: Vec<Words> =
            rd.s[..rd.n].iter().map(|q| [q.b[0], q.b[1], q.b[2], q.a[0], q.a[1]].map(|v| v as f32)).collect();
        if let Some(first) = recon.first_mut() {
            scale_numerator(first, stride as f32);
        }

        let max_frames = max_frames.max(1);
        let sel = max_frames.div_ceil(stride) + 1;
        Run {
            p: p.clone(),
            fs,
            channels,
            hp: (0..channels).map(|_| object(&hp, multi)).collect(),
            ap: (0..channels).map(|_| object(&ap, multi)).collect(),
            lp: object(&lp, false),
            blend: object(&blend, false),
            band: object(&band, false),
            upper: object(&upper, false),
            recon: object(&recon, false),
            lc: LevelControl::new(f64::from(fs8)),
            lead,
            prev_lo: 0.0,
            prev_p_in: 0.0,
            compressor: HarmonicCompressor::new(f64::from(fs8), p.compressor_threshold_db, p.compressor_ratio),
            stride,
            phase: 0,
            latency: nu % 2 + nl % 2,
            work: vec![vec![0.0; max_frames]; channels],
            mono: vec![0.0; max_frames],
            picked: vec![0.0; sel],
            at: vec![0; sel],
            low: vec![0.0; sel],
            harm: vec![0.0; sel],
            p_in: vec![0.0; sel],
        }
    }

    pub fn reset(&mut self) {
        for o in self.hp.iter_mut().chain(self.ap.iter_mut()) {
            o.reset();
        }
        for o in [&mut self.lp, &mut self.blend, &mut self.band, &mut self.upper, &mut self.recon] {
            o.reset();
        }
        let fs8 = self.fs / self.stride as f64;
        self.lc = LevelControl::new(fs8);
        self.prev_lo = 0.0;
        self.prev_p_in = 0.0;
        self.compressor = HarmonicCompressor::new(fs8, self.p.compressor_threshold_db, self.p.compressor_ratio);
        self.phase = 0;
    }

    pub fn latency(&self) -> usize {
        self.latency
    }

    /// Closest level-control hold-restart test to macOS's threshold since
    /// the last call (relative margin; infinity if none). Differential rig
    /// only: it shows where a near tie may resolve differently from macOS.
    #[doc(hidden)]
    pub fn take_tie_margin(&mut self) -> f64 {
        let m = self.lc.input.tie.min(self.lc.output.tie);
        self.lc.input.tie = f32::INFINITY;
        self.lc.output.tie = f32::INFINITY;
        m as f64
    }

    pub fn sample_rate(&self) -> f64 {
        self.fs
    }

    /// Installed crossover-low-pass coefficient words for parity probes.
    #[doc(hidden)]
    pub fn crossover_lowpass_bits(&self, out: &mut [[u32; 5]]) -> usize {
        self.lp.coefficient_bits(out)
    }

    /// The saturating device in its Float32 vector form.
    #[inline]
    fn device(&self, u: f32) -> f32 {
        match self.p.device {
            Device::Saturator3 => law::DEVICE3_AMP as f32 * platform_math::vtanhf(law::DEVICE3_SLOPE as f32 * u),
        }
    }

    /// Both mid branches over one call's selected frames (`n` of them, in
    /// `picked`); leaves their sum in `low`.
    /// A level follower's output as the branch reads it.
    #[inline]
    fn level(p: f32) -> f32 {
        p.clamp(law::LC_FLOOR as f32, law::LC_CEILING as f32)
    }

    /// Both mid branches over one call's selected frames (`n` of them, in
    /// `picked`); leaves their sum in `low`. The level-controlled harmonic
    /// path is Float32 throughout: normalise by the reciprocal of the input
    /// level, clip, run the device, band-limit, restore by the ratio of the
    /// input level to the output level, then apply the harmonic gain.
    fn selected(&mut self, n: usize) {
        self.low[..n].copy_from_slice(&self.picked[..n]);
        self.blend.run(&mut self.low[..n]);
        let clip = law::LC_CLIP as f32;
        for k in 0..n {
            let lo = self.picked[k];
            let x = if self.p.level_control {
                // the restore takes the input level as measured; the drive and
                // the output level are clipped to the level range
                let p_in = self.lc.input.run(f64::from(lo)) as f32;
                self.p_in[k] = f64::from(p_in);
                // The device runs `lead` frames behind the input follower, driven
                // by its own frame's level; the restore uses the current one.
                let (x, p_drive) = if self.lead > 0 {
                    let d = (self.prev_lo as f32, self.prev_p_in as f32);
                    self.prev_lo = f64::from(lo);
                    self.prev_p_in = f64::from(p_in);
                    d
                } else {
                    (lo, p_in)
                };
                (x * platform_math::vdiv(1.0, Self::level(p_drive))).clamp(-clip, clip)
            } else {
                lo
            };
            self.harm[k] = self.device(x);
        }
        self.band.run(&mut self.harm[..n]);
        let gain = self.p.harmonic_gain as f32;
        for k in 0..n {
            let mut h = self.harm[k];
            if self.p.level_control {
                let p_out = Self::level(self.lc.output.run(f64::from(h)) as f32);
                h *= platform_math::vdiv(self.p_in[k] as f32, p_out);
            }
            h *= gain;
            if self.p.harmonic_compressor {
                h = self.compressor.run(f64::from(h)) as f32;
            }
            self.harm[k] = h;
        }
        self.upper.run(&mut self.harm[..n]);
        for k in 0..n {
            self.low[k] += self.harm[k];
        }
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let ch = self.channels.min(bufs.len());
        if ch == 0 {
            return;
        }
        let frames = bufs[..ch].iter().fold(frames.min(self.mono.len()), |n, b| n.min(b.len()));
        let ig = self.p.input_gain as f32;
        self.mono[..frames].fill(0.0);
        for (c, b) in bufs[..ch].iter().enumerate() {
            let w = &mut self.work[c][..frames];
            for (o, v) in w.iter_mut().zip(b.iter()) {
                let x = if v.is_finite() { *v } else { 0.0 };
                *o = if ig == 1.0 { x } else { x * ig };
            }
            for (m, x) in self.mono[..frames].iter_mut().zip(w.iter()) {
                *m += *x;
            }
            self.hp[c].run(w);
            self.ap[c].run(w);
        }
        self.lp.run(&mut self.mono[..frames]);
        let mut n = 0;
        for i in 0..frames {
            if self.phase == 0 && n < self.picked.len() {
                self.picked[n] = self.mono[i];
                self.at[n] = i;
                n += 1;
            }
            self.phase = (self.phase + 1) % self.stride;
        }
        self.selected(n);
        self.mono[..frames].fill(0.0);
        for k in 0..n {
            self.mono[self.at[k]] = self.low[k];
        }
        self.recon.run(&mut self.mono[..frames]);
        let og = self.p.output_gain as f32;
        for (c, b) in bufs[..ch].iter_mut().enumerate() {
            for ((y, h), m) in b[..frames].iter_mut().zip(&self.work[c][..frames]).zip(&self.mono[..frames]) {
                let v = *h + *m;
                *y = if og == 1.0 { v } else { v * og };
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Synthetic state, not any Mac's.
    fn params(level_control: bool) -> Params {
        Params {
            fc_hz: 100.0,
            hp_order: 8,
            fu_hz: 200.0,
            upper_order: 4,
            lower_order: 6,
            harmonic_gain: 1.0,
            blend_gain: 1.0,
            input_gain: 1.0,
            output_gain: 1.0,
            device: Device::Saturator3,
            level_control,
            harmonic_compressor: false,
            compressor_threshold_db: 0.0,
            compressor_ratio: 1.0,
        }
    }

    fn tone(f: f64, a: f64, n: usize) -> Vec<f32> {
        (0..n).map(|i| (a * libm::sin(2.0 * core::f64::consts::PI * f * i as f64 / 48_000.0)) as f32).collect()
    }

    fn amp(x: &[f32], f: f64) -> f64 {
        let (mut re, mut im) = (0.0, 0.0);
        for (i, v) in x.iter().enumerate() {
            let w = 2.0 * core::f64::consts::PI * f * i as f64 / 48_000.0;
            re += *v as f64 * libm::cos(w);
            im += *v as f64 * libm::sin(w);
        }
        2.0 * libm::sqrt(re * re + im * im) / x.len() as f64
    }

    #[test]
    fn words_round_trip() {
        for lc in [false, true] {
            for compressor in [false, true] {
                let mut p = params(lc);
                p.harmonic_compressor = compressor;
                p.compressor_threshold_db = -3.0;
                p.compressor_ratio = 3.5;
                let mut w = [0.0; Params::WORDS];
                p.to_words(&mut w);
                assert_eq!(Params::from_words(&w), Some(p));
            }
        }
        let mut w = [0.0; Params::WORDS];
        params(false).to_words(&mut w);
        w[1] = 7.0; // odd order
        assert_eq!(Params::from_words(&w), None);
        w[1] = f64::NAN;
        assert_eq!(Params::from_words(&w), None);
    }

    #[test]
    fn float32_butterworth_words_follow_their_rounded_poles() {
        // Synthetic corners, not any Mac's.
        for (n, fc, fs) in [(4usize, 100.0f32, 48_000.0f32), (3, 180.0, 8_000.0), (5, 95.0, 48_000.0)] {
            let mut lp = Vec::new();
            let mut hp = Vec::new();
            let mut ap = Vec::new();
            butter_words(n, fc, fs, Kind::Low, &mut lp);
            butter_words(n, fc, fs, Kind::High, &mut hp);
            butter_words(n, fc, fs, Kind::AllPass, &mut ap);
            assert_eq!((lp.len(), hp.len(), ap.len()), (n.div_ceil(2), n.div_ceil(2), n.div_ceil(2)));
            for ((l, h), a) in lp.iter().zip(&hp).zip(&ap) {
                // one shared denominator per pole pair
                assert_eq!((l[3], l[4]), (h[3], h[4]));
                assert_eq!((l[3], l[4]), (a[3], a[4]));
                let dc = f64::from(l[0] + l[1] + l[2]) / f64::from(1.0 + l[3] + l[4]);
                let ny = f64::from(h[0] - h[1] + h[2]) / f64::from(1.0 - h[3] + h[4]);
                assert!((dc - 1.0).abs() < 2e-3, "{n} {fc} dc {dc}");
                assert!((ny - 1.0).abs() < 1e-5, "{n} {fc} nyquist {ny}");
                if l[4] != 0.0 {
                    // quadratic: the pole words come from the rounded sine and cosine
                    // with no adjustment; the all-pass numerator mirrors them
                    assert_eq!(l[1], l[0] + l[0]);
                    assert_eq!(*a, [a[4], a[3], 1.0, a[3], a[4]]);
                } else {
                    // first order: the delayed all-pass form
                    assert_eq!(*a, [0.0, a[3], 1.0, a[3], 0.0]);
                    assert_eq!(l[0], l[1]);
                }
            }
        }
        // the corner is limited to 0.45 of the object's rate
        let (mut a, mut b) = (Vec::new(), Vec::new());
        butter_words(2, 3_900.0, 8_000.0, Kind::Low, &mut a);
        butter_words(2, 3_600.0, 8_000.0, Kind::Low, &mut b);
        assert_eq!(a, b);
    }

    #[test]
    fn mono_low_pass_embeds_the_average_and_the_crossover_polarity() {
        for (order, sign) in [(8usize, 1.0f32), (6, -1.0)] {
            let mut p = params(false);
            p.hp_order = order;
            let r = Run::new(&p, 48_000.0, 2, 64);
            let mut w = [[0u32; 5]; 32];
            let n = r.crossover_lowpass_bits(&mut w);
            assert_eq!(n, 2 * (order / 2).div_ceil(2));
            let half = n / 2;
            let first = f32::from_bits(w[0][0]);
            let scaled = f32::from_bits(w[half][0]);
            assert_eq!(scaled, first * (sign * 0.5));
            assert_eq!(w[1..half], w[half + 1..n]);
        }
    }

    #[test]
    fn harmonic_compressor_tracks_soft_knee_gain_reduction() {
        let mut c = HarmonicCompressor::new(8_000.0, 0.0, 4.0);
        assert!((c.run(-0.1) + 0.1).abs() < 1e-6);
        let mut y = 0.0;
        for _ in 0..8_000 {
            y = c.run(10.0);
        }
        // 20 dB in at 4:1 settles to 5 dB out (15 dB reduction).
        assert!((20.0 * libm::log10(y) - 5.0).abs() < 0.01, "{y}");
    }

    #[test]
    fn one_frame_latency_and_unity_above_the_band() {
        let mut r = Run::new(&params(false), 48_000.0, 2, 48_000);
        // upper band edge order 4 (even sections), lower 6 (odd): one frame
        assert_eq!(r.latency(), 1);
        let n = 48_000;
        let mut l = tone(5_000.0, 0.5, n);
        let mut rr = l.clone();
        r.process(&mut [&mut l, &mut rr], n);
        let g = amp(&l[n / 2..], 5_000.0) / 0.5;
        assert!((g - 1.0).abs() < 1e-3, "{g}");
    }

    #[test]
    fn side_gets_only_the_high_pass() {
        // L = -R: the mid is zero, so no bass and no harmonics anywhere.
        let mut r = Run::new(&params(true), 48_000.0, 2, 48_000);
        let n = 48_000;
        let mut l = tone(40.0, 0.9, n);
        let mut rr: Vec<f32> = l.iter().map(|v| -v).collect();
        r.process(&mut [&mut l, &mut rr], n);
        for k in 2..8 {
            // The Float32 high-path filter is linear but its deterministic
            // round-off floor is visible to this long coherent fit.
            assert!(amp(&l[n / 2..], 40.0 * k as f64) < 1e-5);
        }
        // 40 Hz is far below the 100 Hz eighth-order high pass.
        assert!(amp(&l[n / 2..], 40.0) < 1e-3);
    }

    #[test]
    fn mono_and_equal_stereo_share_the_complete_high_order_path() {
        let mut p = params(true);
        p.hp_order = 20;
        p.upper_order = 20;
        p.lower_order = 20;
        let n = 8_192;
        let input = tone(125.0, 0.4, n);
        let mut mono = input.clone();
        let mut left = input.clone();
        let mut right = input;
        Run::new(&p, 48_000.0, 1, n).process(&mut [&mut mono], n);
        Run::new(&p, 48_000.0, 2, n).process(&mut [&mut left, &mut right], n);
        let error = mono.iter().zip(&left).map(|(a, b)| (*a - *b).abs()).fold(0.0f32, f32::max);
        // a one-channel object runs the platform's grouped Float32 schedule,
        // a stereo one steps frame by frame: the same filter, different roundoff
        assert!(error < 2e-3, "{error}");
        assert_eq!(left, right);
    }

    #[test]
    fn mid_bass_is_kept_by_the_blend() {
        let mut p = params(false);
        p.harmonic_gain = 0.0;
        let mut r = Run::new(&p, 48_000.0, 2, 96_000);
        let n = 96_000;
        let mut l = tone(20.0, 0.1, n);
        let mut rr = l.clone();
        r.process(&mut [&mut l, &mut rr], n);
        let g = amp(&l[n / 2..], 20.0) / 0.1;
        assert!((g - 1.0).abs() < 2e-3, "{g}");
    }

    #[test]
    fn non_finite_input_is_silence() {
        let mut r = Run::new(&params(true), 48_000.0, 2, 64);
        let mut l = [f32::NAN; 64];
        let mut rr = [f32::INFINITY; 64];
        r.process(&mut [&mut l, &mut rr], 64);
        assert!(l.iter().chain(rr.iter()).all(|v| *v == 0.0));
    }

    #[test]
    fn level_control_normalises_and_tracks_the_input() {
        // Steady 40 Hz on the mid: the harmonic output follows the input
        // level 1:1 above the follower floor (same harmonic structure at
        // -20 and -10 dBFS), and stays finite and bounded.
        let level = |a: f64| {
            let mut p = params(true);
            p.blend_gain = 0.0;
            let mut r = Run::new(&p, 48_000.0, 2, 4 * 48_000);
            let n = 4 * 48_000;
            let mut l = tone(40.0, a, n);
            let mut rr = l.clone();
            r.process(&mut [&mut l, &mut rr], n);
            assert!(l.iter().all(|v| v.is_finite() && v.abs() < 4.0));
            amp(&l[3 * 48_000..], 120.0)
        };
        let (lo, hi) = (level(0.1), level(0.316_227_766));
        let step = 20.0 * libm::log10(hi / lo);
        assert!((step - 10.0).abs() < 0.2, "{step}");
    }

    #[test]
    fn odd_lower_edge_moves_one_frame_in_front_of_the_device() {
        // params(): lower order 6 (three sections, odd), upper order 4 (even).
        let lc = Run::new(&params(true), 48_000.0, 2, 1);
        assert_eq!((lc.latency(), lc.lead), (1, 1));
        let plain = Run::new(&params(false), 48_000.0, 2, 1);
        assert_eq!((plain.latency(), plain.lead), (1, 0));
        let mut p = params(true);
        p.lower_order = 4;
        let even = Run::new(&p, 48_000.0, 2, 1);
        assert_eq!((even.latency(), even.lead), (0, 0));
    }

    #[test]
    fn level_control_lead_keeps_the_steady_response() {
        // Moving the frame of delay in front of the device changes only the
        // transient: the steady harmonic and blend output is the same as with
        // the delay after the branch (both are the same LTI delay in steady state).
        let steady = |lead: bool| {
            let mut r = Run::build(&params(true), 48_000.0, 2, 2 * 48_000, (!lead).then_some(0));
            let n = 2 * 48_000;
            let mut l = tone(60.0, 0.5, n);
            let mut rr = l.clone();
            r.process(&mut [&mut l, &mut rr], n);
            (amp(&l[n / 2..], 60.0), amp(&l[n / 2..], 180.0))
        };
        let (a, b) = (steady(true), steady(false));
        assert!((a.0 - b.0).abs() < 1e-5 * b.0.max(1e-3), "{a:?} {b:?}");
        assert!((a.1 - b.1).abs() < 1e-3 * b.1.max(1e-3), "{a:?} {b:?}");
    }

    #[test]
    fn level_control_holds_then_releases() {
        let mut f = Follower::new(48_000.0, law::LC_HOLD_RATIO);
        assert_eq!(f.e, law::LC_SEED as f32);
        assert_eq!(f.s, law::LC_SEED as f32);
        for _ in 0..4_800 {
            f.run(1.0);
        }
        let top = f.run(0.0);
        // hold: no fall for most of the hold time
        for _ in 0..20_000 {
            f.run(0.0);
        }
        assert!(f.run(0.0) > 0.95 * top);
        for _ in 0..96_000 {
            f.run(0.0);
        }
        assert!(f.run(0.0) < 0.2 * top);
    }

    fn gain_db(c: &Cascade, f: f64, fs: f64) -> f64 {
        let w = 2.0 * core::f64::consts::PI * f / fs;
        let z1 = C::new(libm::cos(w), -libm::sin(w));
        let z2 = z1.mul(z1);
        let mut h = C::new(1.0, 0.0);
        for q in &c.s[..c.n] {
            let num = C::new(q.b[0], 0.0).add(z1.scale(q.b[1])).add(z2.scale(q.b[2]));
            let den = C::new(1.0, 0.0).add(z1.scale(q.a[0])).add(z2.scale(q.a[1]));
            h = h.mul(num.div(den));
        }
        20.0 * libm::log10(h.abs())
    }

    #[test]
    fn elliptic_meets_its_specification() {
        let c = elliptic(7, 1.0, 100.0, 2_000.0, 48_000.0);
        assert!(gain_db(&c, 0.0, 48_000.0).abs() < 1e-6);
        for f in [100.0, 500.0, 1_000.0, 1_500.0, 1_999.0] {
            let g = gain_db(&c, f, 48_000.0);
            assert!((-1.0 - 1e-6..=1e-6).contains(&g), "{f} {g}");
        }
        assert!((gain_db(&c, 2_000.0, 48_000.0) + 1.0).abs() < 1e-3);
        for f in [4_000.0, 8_000.0, 16_000.0, 23_000.0] {
            assert!(gain_db(&c, f, 48_000.0) < -99.9, "{f}");
        }
    }
}
