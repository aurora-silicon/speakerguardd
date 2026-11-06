//! Virtual bass (`aufx/vben`): a mid/side bass box.
//!
//! Behaviour (clean-room, from the box's behaviour specification):
//!
//! ```text
//!  per channel c:  hp_c  = AP_u(AP_l(HP(x_c)))
//!  mid M = (L+R)/2: lo    = LP(M)
//!                   low   = s_b * blend * AP_u(AP_l(lo))
//!                   harm  = s_h * gain * LP_u(LC(nld, HP_l, lo))
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
//! - `SUB` band-limits the mid branches (a low pass designed at a quarter
//!   of the rate); each odd-order band-edge all-pass adds one frame of
//!   output latency and a further branch delay.
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

/// Chebyshev type I low pass of order `n`, pass-band ripple `rp_db`, edge
/// `fp`, by the bilinear transform at `fs` with the edge pre-warped. Unity
/// gain at DC for odd `n` (the only orders used).
fn chebyshev1(n: usize, rp_db: f64, fp: f64, fs: f64) -> Cascade {
    let mut c = Cascade::default();
    let eps = libm::sqrt(libm::pow(10.0, rp_db / 10.0) - 1.0);
    let mu = libm::asinh(1.0 / eps) / n as f64;
    let k = libm::tan(core::f64::consts::PI * fp / fs);
    for i in 0..n / 2 {
        let theta = core::f64::consts::PI * (2 * i + 1) as f64 / (2 * n) as f64;
        // analogue pole -sigma ± j omega (unit edge), scaled by k
        let sigma = libm::sinh(mu) * libm::sin(theta) * k;
        let omega = libm::cosh(mu) * libm::cos(theta) * k;
        let w2 = sigma * sigma + omega * omega;
        // H(s) = w2 / (s^2 + 2 sigma s + w2), s = (1 - z^-1) / (1 + z^-1)
        let a0 = 1.0 + 2.0 * sigma + w2;
        let g = w2 / a0;
        c.push(Bq { b: [g, 2.0 * g, g], a: [2.0 * (w2 - 1.0) / a0, (1.0 - 2.0 * sigma + w2) / a0] });
    }
    if n % 2 == 1 {
        let sigma = libm::sinh(mu) * k;
        let a0 = 1.0 + sigma;
        let g = sigma / a0;
        c.push(Bq { b: [g, g, 0.0], a: [(sigma - 1.0) / a0, 0.0] });
    }
    c
}

/// A cascade designed at `fs / STRIDE` run at `fs` with every delay
/// `STRIDE` frames long: the same response below `fs / (2 * STRIDE)`.
const STRIDE_MAX: usize = 8;

#[derive(Clone, Copy, Debug, Default)]
struct StridedState {
    x: [[f64; 2 * STRIDE_MAX]; MAX_SECTIONS],
    y: [[f64; 2 * STRIDE_MAX]; MAX_SECTIONS],
    pos: usize,
}

fn run_strided(c: &Cascade, stride: usize, st: &mut StridedState, mut x: f64) -> f64 {
    let len = 2 * stride;
    let p = st.pos;
    let d1 = (p + len - stride) % len;
    for i in 0..c.n {
        let q = &c.s[i];
        let y = q.b[0] * x + q.b[1] * st.x[i][d1] + q.b[2] * st.x[i][p] - q.a[0] * st.y[i][d1] - q.a[1] * st.y[i][p];
        st.x[i][p] = x;
        st.y[i][p] = y;
        x = y;
    }
    st.pos = (p + 1) % len;
    x
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
    /// Type 3 device: `-AMP * tanh(SLOPE * u)`.
    pub const DEVICE3_AMP: f64 = 3.8;
    pub const DEVICE3_SLOPE: f64 = 0.8;
    /// Level control: one peak follower on the device input sets the
    /// drive `1 / max(peak, LC_FLOOR)` (input clipped to `LC_CLIP`); a
    /// second one on the band-limited device output rescales it by
    /// `peak_in / max(peak_out, LC_FLOOR)`.
    pub const LC_FLOOR: f64 = 0.05;
    pub const LC_CLIP: f64 = 1.2;
    /// Follower: attack and release one-poles, a hold that restarts while
    /// the input stays within `LC_HOLD_RATIO` of the level, and a smoothing
    /// one-pole that follows a rising level at the attack rate.
    pub const LC_ATTACK_S: f64 = 50e-6;
    pub const LC_RELEASE_S: f64 = 0.5;
    pub const LC_HOLD_S: f64 = 0.5;
    pub const LC_HOLD_RATIO: f64 = 0.9;
    pub const LC_SMOOTH_S: f64 = 0.5;
    /// The mid branches are band-limited by a Chebyshev type I low pass
    /// working at a quarter-rate design.
    pub const SUB_RATE_HZ: f64 = 12_000.0;
    pub const SUB_LP_ORDER: usize = 7;
    pub const SUB_LP_RIPPLE_DB: f64 = 1.0;
    pub const SUB_LP_EDGE_HZ: f64 = 2_000.0;
    /// Each odd-order all-pass section of the harmonic band edges delays
    /// the whole output by one frame and the mid branches by this many
    /// sub-rate frames.
    pub const ODD_AP_BRANCH_DELAY_SUB: f64 = 1.1625;
}

/// Branch delay line length (frames); covers two odd sections at the
/// largest stride.
const BRANCH_LINE: usize = 32;
/// Output delay line length (frames): at most two odd sections.
const OUT_LINE: usize = 3;

/// Frames per sub-rate frame at `fs`, if `fs` is a whole multiple of the
/// sub rate.
pub fn sub_stride(fs: f64) -> Option<usize> {
    let r = fs / law::SUB_RATE_HZ;
    (r >= 1.0 && r.fract() == 0.0 && r as usize <= STRIDE_MAX).then_some(r as usize)
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
}

impl Params {
    pub const WORDS: usize = 11;

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
        ];
        out[..Self::WORDS].copy_from_slice(&w);
    }

    pub fn from_words(w: &[f64]) -> Option<Params> {
        if w.len() < Self::WORDS || w[..Self::WORDS].iter().any(|v| !v.is_finite()) {
            return None;
        }
        let order = |v: f64| -> Option<usize> {
            (v >= 2.0 && v <= (2 * MAX_SECTIONS) as f64 && v.fract() == 0.0 && (v as usize) % 2 == 0)
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
        })
    }
}

/// Peak follower of the level control.
#[derive(Clone, Copy, Debug)]
struct Follower {
    e: f64,
    s: f64,
    held: usize,
    hold: usize,
    a_att: f64,
    a_rel: f64,
    a_smo: f64,
}

impl Follower {
    fn new(fs: f64) -> Follower {
        let pole = |t: f64| libm::exp(-1.0 / (t * fs));
        Follower {
            e: 0.0,
            s: 0.0,
            held: 0,
            hold: (law::LC_HOLD_S * fs) as usize + 1,
            a_att: pole(law::LC_ATTACK_S),
            a_rel: pole(law::LC_RELEASE_S),
            a_smo: pole(law::LC_SMOOTH_S),
        }
    }
    #[inline]
    fn run(&mut self, x: f64) -> f64 {
        let u = libm::fabs(x);
        let (a, b) = if u > self.e {
            self.held = 0;
            (self.a_att, if self.e > self.s { self.a_att } else { self.a_smo })
        } else if self.held < self.hold {
            self.held += 1;
            if u >= law::LC_HOLD_RATIO * self.e {
                self.held = 0;
            }
            (1.0, self.a_smo)
        } else {
            (self.a_rel, self.a_smo)
        };
        self.e = a * self.e + (1.0 - a) * u;
        self.s = b * self.s + (1.0 - b) * self.e;
        self.s
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
        LevelControl { input: Follower::new(fs), output: Follower::new(fs) }
    }
}

pub struct Run {
    p: Params,
    fs: f64,
    channels: usize,
    /// Per channel: high pass then both all-passes.
    hp: Cascade,
    hp_st: Vec<CascadeState>,
    lp: Cascade,
    lp_st: CascadeState,
    /// The low-end blend of an odd-order crossover is taken inverted, so
    /// that low pass and high pass still sum to an all-pass (the harmonic
    /// branch is not).
    lp_sign: f64,
    /// The harmonic branch is inverted when the crossover and the band's
    /// lower edge are both of even or both of odd Butterworth order.
    harm_sign: f64,
    ap: Cascade,
    ap_st: CascadeState,
    /// Harmonic band: the lower edge's high pass with the first half of the
    /// upper edge's low pass, then the second half (the level control's
    /// limiter looks between the two).
    band_hp: Cascade,
    band_hp_st: CascadeState,
    band_lp: Cascade,
    band_lp_st: CascadeState,
    sub: Cascade,
    sub_st: StridedState,
    stride: usize,
    /// Mid-branch delay: whole frames in a ring, the fraction by a
    /// first-order all-pass (coefficient `frac_a`).
    bline: [f64; BRANCH_LINE],
    bpos: usize,
    bdelay: usize,
    frac_a: f64,
    frac_x: f64,
    frac_y: f64,
    lc: LevelControl,
    /// Output delay, `latency` frames per channel.
    latency: usize,
    oline: Vec<[f64; OUT_LINE]>,
    opos: usize,
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize) -> Run {
        let (nu, nl) = (p.upper_order / 2, p.lower_order / 2);
        let mut ap = butterworth(nu, p.fu_hz, fs, Kind::AllPass);
        ap.extend(&butterworth(nl, p.fc_hz, fs, Kind::AllPass));
        let mut hp = linkwitz_riley(p.hp_order, p.fc_hz, fs, Kind::High);
        hp.extend(&ap);
        let lp = linkwitz_riley(p.hp_order, p.fc_hz, fs, Kind::Low);
        // the upper edge's Linkwitz-Riley low pass is two Butterworths: the
        // limiter's detector sits between them
        let mut band_hp = linkwitz_riley(p.lower_order, p.fc_hz, fs, Kind::High);
        band_hp.extend(&butterworth(p.upper_order / 2, p.fu_hz, fs, Kind::Low));
        let band_lp = butterworth(p.upper_order / 2, p.fu_hz, fs, Kind::Low);
        let stride = sub_stride(fs).unwrap_or(1);
        let sub = chebyshev1(law::SUB_LP_ORDER, law::SUB_LP_RIPPLE_DB, law::SUB_LP_EDGE_HZ, fs / stride as f64);
        let latency = nu % 2 + nl % 2;
        let d = latency as f64 * law::ODD_AP_BRANCH_DELAY_SUB * stride as f64;
        // whole part such that the fraction lies in [0.5, 1.5): the
        // first-order all-pass delay approximation is accurate there
        let (bdelay, frac) = if d < 0.5 {
            (0, 0.0)
        } else {
            let n = libm::floor(d - 0.5);
            ((n as usize).min(BRANCH_LINE - 1), d - n)
        };
        let frac_a = if frac == 0.0 { 0.0 } else { (1.0 - frac) / (1.0 + frac) };
        Run {
            p: p.clone(),
            fs,
            channels,
            hp,
            hp_st: vec![CascadeState::default(); channels],
            lp,
            lp_st: CascadeState::default(),
            lp_sign: if (p.hp_order / 2) % 2 == 1 { -1.0 } else { 1.0 },
            harm_sign: if (p.hp_order / 2) % 2 == nl % 2 { -1.0 } else { 1.0 },
            ap,
            ap_st: CascadeState::default(),
            band_hp,
            band_hp_st: CascadeState::default(),
            band_lp,
            band_lp_st: CascadeState::default(),
            sub,
            sub_st: StridedState::default(),
            stride,
            bline: [0.0; BRANCH_LINE],
            bpos: 0,
            bdelay,
            frac_a: if frac == 0.0 { f64::NAN } else { frac_a },
            frac_x: 0.0,
            frac_y: 0.0,
            lc: LevelControl::new(fs),
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
        self.sub_st = StridedState::default();
        self.bline = [0.0; BRANCH_LINE];
        self.frac_x = 0.0;
        self.frac_y = 0.0;
        self.lc = LevelControl::new(self.fs);
        for l in &mut self.oline {
            *l = [0.0; OUT_LINE];
        }
    }

    pub fn latency(&self) -> usize {
        self.latency
    }

    pub fn sample_rate(&self) -> f64 {
        self.fs
    }

    #[inline]
    fn device(&self, u: f64) -> f64 {
        match self.p.device {
            Device::Saturator3 => -law::DEVICE3_AMP * libm::tanh(law::DEVICE3_SLOPE * u),
        }
    }

    #[inline]
    fn branch_delay(&mut self, v: f64) -> f64 {
        let mut x = v;
        if self.bdelay > 0 {
            self.bline[self.bpos] = v;
            let r = (self.bpos + BRANCH_LINE - self.bdelay) % BRANCH_LINE;
            x = self.bline[r];
            self.bpos = (self.bpos + 1) % BRANCH_LINE;
        }
        if self.frac_a.is_nan() {
            return x;
        }
        // first-order all-pass fractional delay
        let y = self.frac_a * x + self.frac_x - self.frac_a * self.frac_y;
        self.frac_x = x;
        self.frac_y = y;
        y
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
            let low = self.lp_sign * self.p.blend_gain * self.ap.run(&mut self.ap_st, lo);
            let harm = if self.p.level_control {
                let p_in = self.lc.input.run(lo);
                let z = (lo / p_in.max(law::LC_FLOOR)).clamp(-law::LC_CLIP, law::LC_CLIP);
                let dev = self.device(z);
                let hp = self.band_hp.run(&mut self.band_hp_st, dev);
                let p_out = self.lc.output.run(hp);
                let y = hp * p_in / p_out.max(law::LC_FLOOR);
                self.p.harmonic_gain * self.band_lp.run(&mut self.band_lp_st, y)
            } else {
                let dev = self.device(lo);
                let hp = self.band_hp.run(&mut self.band_hp_st, dev);
                self.p.harmonic_gain * self.band_lp.run(&mut self.band_lp_st, hp)
            };
            let delayed = self.branch_delay(low + self.harm_sign * harm);
            let common = run_strided(&self.sub, self.stride, &mut self.sub_st, delayed);
            let (l, pos) = (self.latency, self.opos);
            for (c, b) in bufs[..ch].iter_mut().enumerate() {
                let v = b[i] as f64;
                let x = if v.is_finite() { v * ig } else { 0.0 };
                let y = (self.hp.run(&mut self.hp_st[c], x) + common) * self.p.output_gain;
                let line = &mut self.oline[c];
                line[pos] = y;
                b[i] = line[(pos + OUT_LINE - l) % OUT_LINE] as f32;
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
            let p = params(lc);
            let mut w = [0.0; Params::WORDS];
            p.to_words(&mut w);
            assert_eq!(Params::from_words(&w), Some(p));
        }
        let mut w = [0.0; Params::WORDS];
        params(false).to_words(&mut w);
        w[1] = 7.0; // odd order
        assert_eq!(Params::from_words(&w), None);
        w[1] = f64::NAN;
        assert_eq!(Params::from_words(&w), None);
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
    fn level_control_holds_then_releases() {
        let mut f = Follower::new(48_000.0);
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
}
