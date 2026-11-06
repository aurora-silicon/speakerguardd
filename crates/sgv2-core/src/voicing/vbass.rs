//! Virtual bass (`aufx/vben`): a mid/side bass box.
//!
//! Behaviour (clean-room, from the box's observed input/output only):
//!
//! ```text
//!  per channel c:  hp_c  = AP_u(AP_l(HP(x_c)))
//!  mid M = (L+R)/2: lo    = LP(M)
//!                   low   = blend * AP_u(AP_l(lo))
//!                   harm  = gain  * BP(nld(drive(lo)))
//!  y_c = out_gain * (hp_c + low + harm), one frame late
//! ```
//!
//! - `HP`/`LP` are a Linkwitz-Riley pair at the crossover `fc` (each a
//!   Butterworth of half the order, squared). `AP_u` and `AP_l` are the
//!   all-pass sections of the upper (`fu`) and lower (`fc`) band edges of
//!   the harmonic band, so every path has the same phase as the high pass.
//! - `BP` is the harmonic band: Linkwitz-Riley high pass at `fc` (lower
//!   order) times Linkwitz-Riley low pass at `fu` (upper order).
//! - `nld` is the memoryless odd device of the saved type; `drive` is the
//!   level control (unity when the level control is off).
//! - The side signal only ever takes the high-pass path: stereo width is
//!   untouched, and the bass is folded to the mid.
//!
//! No tuning value lives here: corners, orders and gains come in [`Params`]
//! from the Mac's own saved state.
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
mod law {
    /// Type 3 device: `-AMP * tanh(SLOPE * u)`.
    pub const DEVICE3_AMP: f64 = 3.92134;
    pub const DEVICE3_SLOPE: f64 = 0.78597;
    /// Level control: envelope the drive normalises to (linear peak),
    /// envelope attack/release (s) and the largest drive.
    pub const LC_TARGET: f64 = 1.0;
    pub const LC_ATTACK_S: f64 = 0.010;
    pub const LC_RELEASE_S: f64 = 0.100;
    pub const LC_MAX_DRIVE: f64 = 1000.0;
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

pub struct Run {
    p: Params,
    fs: f64,
    channels: usize,
    /// Per channel: high pass then both all-passes.
    hp: Cascade,
    hp_st: Vec<CascadeState>,
    lp: Cascade,
    lp_st: CascadeState,
    ap: Cascade,
    ap_st: CascadeState,
    band: Cascade,
    band_st: CascadeState,
    env: f64,
    att: f64,
    rel: f64,
    /// One frame of output latency per channel.
    held: Vec<f64>,
}

impl Run {
    pub fn new(p: &Params, fs: f64, channels: usize) -> Run {
        let mut ap = butterworth(p.upper_order / 2, p.fu_hz, fs, Kind::AllPass);
        ap.extend(&butterworth(p.lower_order / 2, p.fc_hz, fs, Kind::AllPass));
        let mut hp = linkwitz_riley(p.hp_order, p.fc_hz, fs, Kind::High);
        hp.extend(&ap);
        let lp = linkwitz_riley(p.hp_order, p.fc_hz, fs, Kind::Low);
        let mut band = linkwitz_riley(p.lower_order, p.fc_hz, fs, Kind::High);
        band.extend(&linkwitz_riley(p.upper_order, p.fu_hz, fs, Kind::Low));
        let (att, rel) = (libm::exp(-1.0 / (law::LC_ATTACK_S * fs)), libm::exp(-1.0 / (law::LC_RELEASE_S * fs)));
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
            band,
            band_st: CascadeState::default(),
            env: 0.0,
            att,
            rel,
            held: vec![0.0; channels],
        }
    }

    pub fn reset(&mut self) {
        for s in &mut self.hp_st {
            *s = CascadeState::default();
        }
        self.lp_st = CascadeState::default();
        self.ap_st = CascadeState::default();
        self.band_st = CascadeState::default();
        self.env = 0.0;
        for h in &mut self.held {
            *h = 0.0;
        }
    }

    pub fn latency(&self) -> usize {
        1
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
            let low = self.p.blend_gain * self.ap.run(&mut self.ap_st, lo);
            let drive = if self.p.level_control {
                let a = libm::fabs(lo);
                let c = if a > self.env { self.att } else { self.rel };
                self.env = a + c * (self.env - a);
                (law::LC_TARGET / self.env.max(1e-30)).min(law::LC_MAX_DRIVE)
            } else {
                1.0
            };
            let d = self.device(drive * lo) / drive;
            let harm = self.p.harmonic_gain * self.band.run(&mut self.band_st, d);
            let common = low + harm;
            for (c, b) in bufs[..ch].iter_mut().enumerate() {
                let v = b[i] as f64;
                let x = if v.is_finite() { v * ig } else { 0.0 };
                let y = self.hp.run(&mut self.hp_st[c], x) + common;
                let out = core::mem::replace(&mut self.held[c], y * self.p.output_gain);
                b[i] = out as f32;
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
}
