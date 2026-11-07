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
//! - The side signal only ever takes the high-pass path: stereo width is
//!   untouched, and the bass is folded to the mid.
//!
//! No tuning value lives here: corners, orders and gains come in [`Params`]
//! from the Mac's own saved state; [`law`] holds the box's own constants.
use alloc::vec;
use alloc::vec::Vec;

/// One direct-form-II-transposed biquad, f64 state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Bq {
    b: [f64; 3],
    a: [f64; 2],
}

#[derive(Clone, Copy, Debug, Default)]
struct BqState {
    z: [f64; 2],
}

impl Bq {
    #[inline]
    fn run(&self, s: &mut BqState, x: f64) -> f64 {
        let y = self.b[0] * x + s.z[0];
        s.z[0] = self.b[1] * x - self.a[0] * y + s.z[1];
        s.z[1] = self.b[2] * x - self.a[1] * y;
        y
    }
}

/// Most biquads one cascade needs (a Linkwitz-Riley of order 20 is two
/// Butterworths of order 10: ten sections).
const MAX_SECTIONS: usize = 10;

#[derive(Clone, Debug, Default)]
struct Cascade {
    n: usize,
    s: [Bq; MAX_SECTIONS],
}

#[derive(Clone, Copy, Debug, Default)]
struct CascadeState {
    z: [BqState; MAX_SECTIONS],
}

impl Cascade {
    fn push(&mut self, q: Bq) {
        if self.n < MAX_SECTIONS {
            self.s[self.n] = q;
            self.n += 1;
        }
    }
    fn extend(&mut self, other: &Cascade) {
        for q in &other.s[..other.n] {
            self.push(*q);
        }
    }
    #[inline]
    fn run(&self, st: &mut CascadeState, mut x: f64) -> f64 {
        for i in 0..self.n {
            x = self.s[i].run(&mut st.z[i], x);
        }
        x
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Kind {
    Low,
    High,
    AllPass,
}

/// Butterworth of order `n` at `fc` by the bilinear transform with the
/// corner pre-warped, as second-order sections (one first-order section
/// for odd `n`). `AllPass` is the all-pass with the same poles.
fn butterworth(n: usize, fc: f64, fs: f64, kind: Kind) -> Cascade {
    let mut c = Cascade::default();
    let k = libm::tan(core::f64::consts::PI * fc / fs);
    let pairs = n / 2;
    for i in 0..pairs {
        // analogue pole pair angle; q = 1 / (2 cos θ)
        let theta = core::f64::consts::PI * (2 * i + 1) as f64 / (2 * n) as f64;
        let two_zeta = 2.0 * libm::sin(theta);
        let a0 = 1.0 + two_zeta * k + k * k;
        let a1 = 2.0 * (k * k - 1.0) / a0;
        let a2 = (1.0 - two_zeta * k + k * k) / a0;
        let b = match kind {
            Kind::Low => {
                let g = k * k / a0;
                [g, 2.0 * g, g]
            }
            Kind::High => {
                let g = 1.0 / a0;
                [g, -2.0 * g, g]
            }
            Kind::AllPass => [a2, a1, 1.0],
        };
        c.push(Bq { b, a: [a1, a2] });
    }
    if n % 2 == 1 {
        let a0 = 1.0 + k;
        let a1 = (k - 1.0) / a0;
        let b = match kind {
            Kind::Low => [k / a0, k / a0, 0.0],
            Kind::High => [1.0 / a0, -1.0 / a0, 0.0],
            Kind::AllPass => [a1, 1.0, 0.0],
        };
        c.push(Bq { b, a: [a1, 0.0] });
    }
    c
}

/// Linkwitz-Riley of even order `order`: the Butterworth of half the order,
/// twice.
fn linkwitz_riley(order: usize, fc: f64, fs: f64, kind: Kind) -> Cascade {
    let half = butterworth(order / 2, fc, fs, kind);
    let mut c = half.clone();
    c.extend(&half);
    c
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
        let p0 = j.mul(jacobi(j.mul(v0), k, true)).scale(wp).re;
        let d0 = 1.0 - p0;
        c.push(Bq { b: [-p0 / d0, -p0 / d0, 0.0], a: [-(1.0 + p0) / d0, 0.0] });
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
    /// Follower: attack and release one-poles, a hold that restarts while
    /// the input stays within `LC_HOLD_RATIO` of the level, and a smoothing
    /// one-pole that follows a rising level at the attack rate.
    /// SPEC-vben §1 (follower constants).
    pub const LC_ATTACK_S: f64 = 50e-6;
    pub const LC_RELEASE_S: f64 = 0.5;
    pub const LC_HOLD_S: f64 = 0.5;
    pub const LC_HOLD_RATIO: f64 = 0.9;
    pub const LC_SMOOTH_S: f64 = 0.5;
    /// Never-louder margin on the hold-restart test (not part of macOS's
    /// law): this box's arithmetic is not bit-identical to macOS's, so a
    /// near-tie restart could fall either way. The input follower restarts
    /// its hold only clearly above the ratio (it then releases no later than
    /// macOS's), the output follower already slightly below it (it then
    /// releases no earlier): a tie always resolves to the quieter output.
    pub const LC_HOLD_MARGIN: f64 = 1e-3;
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

/// Selected-rate delay line (sub-rate frames): at most two odd sections.
const SUB_LINE: usize = 3;
/// Output delay line length (frames): at most two odd sections.
const OUT_LINE: usize = 3;
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
            (v >= 2.0 && v <= (2 * MAX_SECTIONS) as f64 && libm::trunc(v) == v && (v as usize) % 2 == 0)
                .then_some(v as usize)
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
}

impl Follower {
    fn new(fs: f64, ratio: f64) -> Follower {
        Follower::with_times(fs, ratio, law::LC_ATTACK_S, law::LC_RELEASE_S, law::LC_HOLD_S, law::LC_SMOOTH_S)
    }

    fn with_times(fs: f64, ratio: f64, attack_s: f64, release_s: f64, hold_s: f64, smooth_s: f64) -> Follower {
        let fs = fs as f32;
        let pole = |t: f64| libm::expf(-1.0 / (t as f32 * fs));
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
        self.e = a * self.e + (1.0 - a) * u;
        self.s = b * self.s + (1.0 - b) * self.e;
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
        let level_db = 20.0 * libm::log10f(libm::fabsf(x) + law::COMP_DETECTOR_EPSILON as f32);
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
        (x * libm::powf(10.0, -reduction_db / 20.0)) as f64
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
        LevelControl {
            input: Follower::new(fs, law::LC_HOLD_RATIO * (1.0 + law::LC_HOLD_MARGIN)),
            output: Follower::new(fs, law::LC_HOLD_RATIO * (1.0 - law::LC_HOLD_MARGIN)),
        }
    }
}

pub struct Run {
    p: Params,
    fs: f64,
    channels: usize,
    /// Per channel at the stream rate: high pass then both all-passes.
    hp: Cascade,
    hp_st: Vec<CascadeState>,
    /// Crossover low pass of the mid, at the stream rate.
    lp: Cascade,
    lp_st: CascadeState,
    /// Selected rate: both band-edge all-passes (blend), the band's lower
    /// edge with the first half of its upper edge, then the second half.
    ap: Cascade,
    ap_st: CascadeState,
    band_hp: Cascade,
    band_hp_st: CascadeState,
    band_lp: Cascade,
    band_lp_st: CascadeState,
    /// Polarities: blend inverted for an odd-order crossover; harmonics
    /// inverted when the crossover and the band's lower edge differ in
    /// Butterworth-order parity.
    lp_sign: f64,
    harm_sign: f64,
    lc: LevelControl,
    /// Selected-rate frames the device runs behind the input follower
    /// (1 with the level control on and an odd lower band edge, else 0).
    lead: usize,
    /// The previous frame's device input and its input level, and the blend
    /// branch held back by `lead` frames to stay aligned with the harmonics.
    prev_lo: f64,
    prev_p_in: f64,
    prev_low: f64,
    compressor: HarmonicCompressor,
    stride: usize,
    /// Selected-rate delay: one sub-rate frame per odd band-edge section.
    sline: [f64; SUB_LINE],
    spos: usize,
    sdelay: usize,
    /// Position in the selected-rate grid (carried across blocks).
    phase: usize,
    /// Reconstruction low pass (stream rate).
    recon: Cascade,
    recon_st: CascadeState,
    /// High-pass path delay, `latency` frames per channel (the mid
    /// branches carry theirs at the selected rate).
    latency: usize,
    oline: Vec<[f64; OUT_LINE]>,
    opos: usize,
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize) -> Run {
        let stride = sub_stride(fs).unwrap_or(1);
        let fs8 = fs / stride as f64;
        let (nh, nu, nl) = (p.hp_order / 2, p.upper_order / 2, p.lower_order / 2);
        let mut ap48 = butterworth(nu, p.fu_hz, fs, Kind::AllPass);
        ap48.extend(&butterworth(nl, p.fc_hz, fs, Kind::AllPass));
        let mut hp = linkwitz_riley(p.hp_order, p.fc_hz, fs, Kind::High);
        hp.extend(&ap48);
        let lp = linkwitz_riley(p.hp_order, p.fc_hz, fs, Kind::Low);
        let mut ap = butterworth(nu, p.fu_hz, fs8, Kind::AllPass);
        ap.extend(&butterworth(nl, p.fc_hz, fs8, Kind::AllPass));
        let mut band_hp = linkwitz_riley(p.lower_order, p.fc_hz, fs8, Kind::High);
        band_hp.extend(&butterworth(nu, p.fu_hz, fs8, Kind::Low));
        let band_lp = butterworth(nu, p.fu_hz, fs8, Kind::Low);
        let latency = nu % 2 + nl % 2;
        let lead = if p.level_control { nl % 2 } else { 0 };
        Run {
            p: p.clone(),
            fs,
            channels,
            hp,
            hp_st: vec![CascadeState::default(); channels],
            lp,
            lp_st: CascadeState::default(),
            ap,
            ap_st: CascadeState::default(),
            band_hp,
            band_hp_st: CascadeState::default(),
            band_lp,
            band_lp_st: CascadeState::default(),
            lp_sign: if nh % 2 == 1 { -1.0 } else { 1.0 },
            harm_sign: if nh % 2 == nl % 2 { 1.0 } else { -1.0 },
            lc: LevelControl::new(fs8),
            lead,
            prev_lo: 0.0,
            prev_p_in: 0.0,
            prev_low: 0.0,
            compressor: HarmonicCompressor::new(fs8, p.compressor_threshold_db, p.compressor_ratio),
            stride,
            sline: [0.0; SUB_LINE],
            spos: 0,
            sdelay: latency - lead,
            phase: 0,
            recon: elliptic(law::RECON_ORDER, law::RECON_RIPPLE_DB, law::RECON_STOP_DB, law::RECON_EDGE_HZ, fs),
            recon_st: CascadeState::default(),
            latency,
            oline: vec![[0.0; OUT_LINE]; channels],
            opos: 0,
        }
    }

    pub fn reset(&mut self) {
        for s in &mut self.hp_st {
            *s = CascadeState::default();
        }
        self.lp_st = CascadeState::default();
        self.ap_st = CascadeState::default();
        self.band_hp_st = CascadeState::default();
        self.band_lp_st = CascadeState::default();
        self.lc = LevelControl::new(self.fs / self.stride as f64);
        self.prev_lo = 0.0;
        self.prev_p_in = 0.0;
        self.prev_low = 0.0;
        self.compressor = HarmonicCompressor::new(
            self.fs / self.stride as f64,
            self.p.compressor_threshold_db,
            self.p.compressor_ratio,
        );
        self.sline = [0.0; SUB_LINE];
        self.phase = 0;
        self.recon_st = CascadeState::default();
        self.oline.fill([0.0; OUT_LINE]);
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

    #[inline]
    fn device(&self, u: f64) -> f64 {
        match self.p.device {
            Device::Saturator3 => law::DEVICE3_AMP * libm::tanh(law::DEVICE3_SLOPE * u),
        }
    }

    #[inline]
    fn compress_harmonic(&mut self, x: f64) -> f64 {
        if !self.p.harmonic_compressor {
            return x;
        }
        self.compressor.run(x)
    }

    /// One selected-rate frame of both mid branches.
    #[inline]
    fn sub_frame(&mut self, lo: f64) -> f64 {
        let mut low = self.lp_sign * self.p.blend_gain * self.ap.run(&mut self.ap_st, lo);
        if self.lead > 0 {
            core::mem::swap(&mut low, &mut self.prev_low);
        }
        let harm = if self.p.level_control {
            let p_in = self.lc.input.run(lo);
            // The device runs `lead` frames behind the input follower, driven
            // by its own frame's level; the restore below uses the current one.
            let (x, p_drive) = if self.lead > 0 {
                let d = (self.prev_lo, self.prev_p_in);
                self.prev_lo = lo;
                self.prev_p_in = p_in;
                d
            } else {
                (lo, p_in)
            };
            let z = (x / p_drive.max(law::LC_FLOOR)).clamp(-law::LC_CLIP, law::LC_CLIP);
            let dev = self.device(z);
            let hp = self.band_hp.run(&mut self.band_hp_st, dev);
            let p_out = self.lc.output.run(hp);
            hp * p_in / p_out.max(law::LC_FLOOR)
        } else {
            let dev = self.device(lo);
            self.band_hp.run(&mut self.band_hp_st, dev)
        };
        // The compressor sees harmonic gain and precedes the upper edge's
        // second Butterworth half. Both details affect its detector without
        // changing the complementary LF fold.
        let harmonic_gain = self.p.harmonic_gain;
        let harm = self.compress_harmonic(harmonic_gain * harm);
        let harm = self.band_lp.run(&mut self.band_lp_st, harm);
        let v = low + self.harm_sign * harm;
        if self.sdelay == 0 {
            return v;
        }
        self.sline[self.spos] = v;
        let out = self.sline[(self.spos + SUB_LINE - self.sdelay) % SUB_LINE];
        self.spos = (self.spos + 1) % SUB_LINE;
        out
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        let ch = self.channels.min(bufs.len());
        if ch == 0 {
            return;
        }
        let frames = bufs[..ch].iter().fold(frames, |n, b| n.min(b.len()));
        let ig = self.p.input_gain;
        for i in 0..frames {
            let mut mid = 0.0;
            for b in bufs[..ch].iter() {
                let v = b[i] as f64;
                mid += if v.is_finite() { v * ig } else { 0.0 };
            }
            mid /= ch as f64;
            let lo = self.lp.run(&mut self.lp_st, mid);
            let up = if self.phase == 0 { self.sub_frame(lo) * self.stride as f64 } else { 0.0 };
            self.phase = (self.phase + 1) % self.stride;
            let common = self.recon.run(&mut self.recon_st, up);
            let (l, pos) = (self.latency, self.opos);
            for (c, b) in bufs[..ch].iter_mut().enumerate() {
                let v = b[i] as f64;
                let x = if v.is_finite() { v * ig } else { 0.0 };
                let line = &mut self.oline[c];
                line[pos] = self.hp.run(&mut self.hp_st[c], x);
                let y = (line[(pos + OUT_LINE - l) % OUT_LINE] + common) * self.p.output_gain;
                b[i] = y as f32;
            }
            self.opos = (pos + 1) % OUT_LINE;
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
        let mut r = Run::new(&params(false), 48_000.0, 2);
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
        let mut r = Run::new(&params(true), 48_000.0, 2);
        let n = 48_000;
        let mut l = tone(40.0, 0.9, n);
        let mut rr: Vec<f32> = l.iter().map(|v| -v).collect();
        r.process(&mut [&mut l, &mut rr], n);
        for k in 2..8 {
            assert!(amp(&l[n / 2..], 40.0 * k as f64) < 1e-6);
        }
        // 40 Hz is far below the 100 Hz eighth-order high pass.
        assert!(amp(&l[n / 2..], 40.0) < 1e-3);
    }

    #[test]
    fn mid_bass_is_kept_by_the_blend() {
        let mut p = params(false);
        p.harmonic_gain = 0.0;
        let mut r = Run::new(&p, 48_000.0, 2);
        let n = 96_000;
        let mut l = tone(20.0, 0.1, n);
        let mut rr = l.clone();
        r.process(&mut [&mut l, &mut rr], n);
        let g = amp(&l[n / 2..], 20.0) / 0.1;
        assert!((g - 1.0).abs() < 2e-3, "{g}");
    }

    #[test]
    fn non_finite_input_is_silence() {
        let mut r = Run::new(&params(true), 48_000.0, 2);
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
            let mut r = Run::new(&p, 48_000.0, 2);
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
        let lc = Run::new(&params(true), 48_000.0, 2);
        assert_eq!((lc.latency(), lc.lead, lc.sdelay), (1, 1, 0));
        let plain = Run::new(&params(false), 48_000.0, 2);
        assert_eq!((plain.latency(), plain.lead, plain.sdelay), (1, 0, 1));
        let mut p = params(true);
        p.lower_order = 4;
        let even = Run::new(&p, 48_000.0, 2);
        assert_eq!((even.latency(), even.lead, even.sdelay), (0, 0, 0));
    }

    #[test]
    fn level_control_lead_keeps_the_steady_response() {
        // Moving the frame of delay in front of the device changes only the
        // transient: the steady harmonic and blend output is the same as with
        // the delay after the branch (both are the same LTI delay in steady state).
        let steady = |lead: bool| {
            let mut r = Run::new(&params(true), 48_000.0, 2);
            if !lead {
                r.lead = 0;
                r.sdelay = r.latency;
            }
            let n = 2 * 48_000;
            let mut l = tone(60.0, 0.5, n);
            let mut rr = l.clone();
            r.process(&mut [&mut l, &mut rr], n);
            (amp(&l[n / 2..], 60.0), amp(&l[n / 2..], 180.0))
        };
        let (a, b) = (steady(true), steady(false));
        assert!((a.0 - b.0).abs() < 1e-6 * b.0.max(1e-3), "{a:?} {b:?}");
        assert!((a.1 - b.1).abs() < 1e-3 * b.1.max(1e-3), "{a:?} {b:?}");
    }

    #[test]
    fn level_control_holds_then_releases() {
        let mut f = Follower::new(48_000.0, law::LC_HOLD_RATIO);
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
