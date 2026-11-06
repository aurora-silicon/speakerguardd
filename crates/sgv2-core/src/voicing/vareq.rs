//! VarEQ (`aufx/vrq2`): the volume-dependent EQ ahead of the compressor.
//!
//! The unit holds five equalizer settings ("elements"), each an element gain
//! and eight second-order bands, and four volume breakpoints. The graph's
//! master volume scalar (`mvsc`, 0..1) picks the setting: element 0 at or
//! below the first breakpoint, element `k` at breakpoint `k + 1`, element 4
//! at full volume, and in between the two neighbouring elements' parameters
//! are interpolated linearly in the volume (frequency in Hz, gains in dB,
//! bandwidth in octaves). Only the compiled parameters cross into this
//! crate; every value comes from the Mac's own saved state.
//!
//! Behaviour (black-box measured on the unit, see the box notes):
//! - Peaking: `a = (1 + α, -2 cos w, 1 - α)`, `b = (1 + αG, -2 cos w, 1 - αG)`
//!   with `α = sin w · sinh(ln 2 / 2 · bw · w / sin w)`, `G = 10^(g/20)`; the
//!   denominator does not depend on the gain.
//! - Shelves: the cookbook shelves with slope 1; the bandwidth is unused.
//! - Low and high pass: second-order Butterworth (Q = 1/√2); a low pass at or
//!   above Nyquist is a wire.
//! - A band that is on in only one of the two neighbouring elements fades by
//!   the on side's weight `s`: gain bands scale their gain by `s` and keep
//!   the on side's frequency and bandwidth; a low pass moves from Nyquist to
//!   its frequency; a high pass takes `s` times the upper element's
//!   frequency, never below [`HP_FADE_FLOOR_HZ`].
//! - A new setting is reached by a linear crossfade of the outputs of the old
//!   and the new setting over [`FADE_S`], started by any change of the volume
//!   value (even one that keeps the setting); a volume change during a fade waits
//!   for the fade's end and then fades to the latest volume. A change before
//!   the first processed frame applies at once.
//!
//! TODO(SPEC): during a crossfade the unit's new-setting filters start from a
//! state that is not a copy of the old filters' state (the difference decays
//! within the 20 ms fade); this box copies the old state. Steady state and
//! the fade law are exact.

use libm::{cos, pow, sin, sinh, sqrt};

pub const BANDS: usize = 8;
pub const ELEMENTS: usize = 5;
pub const BREAKPOINTS: usize = ELEMENTS - 1;
pub const MAX_CHANNELS: usize = 8;
/// Crossfade time between two settings.
pub const FADE_S: f64 = 0.020;
/// Lowest high-pass frequency while a high pass fades in or out.
pub const HP_FADE_FLOOR_HZ: f64 = 40.0;

const BAND_WORDS: usize = 5;
const ELEMENT_WORDS: usize = 1 + BANDS * BAND_WORDS;
const GAIN_MIN_DB: f64 = -96.0;
const GAIN_MAX_DB: f64 = 24.0;
/// The unit's pre-gain range tops out at +12 dB.
const PRE_GAIN_MAX_DB: f64 = 12.0;
const BW_MIN: f64 = 0.01;
const BW_MAX: f64 = 10.0;
const FREQ_MAX_HZ: f64 = 1.0e6;

/// Band filter type, numbered as the unit stores it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Peak = 0,
    LowPass = 1,
    HighPass = 2,
    LowShelf = 7,
    HighShelf = 8,
}

impl Kind {
    pub fn from_code(code: f64) -> Option<Kind> {
        if code.fract() != 0.0 {
            return None;
        }
        match code as i64 {
            0 => Some(Kind::Peak),
            1 => Some(Kind::LowPass),
            2 => Some(Kind::HighPass),
            7 => Some(Kind::LowShelf),
            8 => Some(Kind::HighShelf),
            _ => None,
        }
    }

    fn has_gain(self) -> bool {
        matches!(self, Kind::Peak | Kind::LowShelf | Kind::HighShelf)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Band {
    pub on: bool,
    pub kind: Kind,
    pub freq_hz: f64,
    pub gain_db: f64,
    pub bw_oct: f64,
}

impl Default for Band {
    fn default() -> Self {
        Self { on: false, kind: Kind::Peak, freq_hz: 1000.0, gain_db: 0.0, bw_oct: 1.0 }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Element {
    pub gain_db: f64,
    pub bands: [Band; BANDS],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Params {
    /// The unit's pre-gain (dB), applied with the element gain.
    pub pre_gain_db: f64,
    /// Volume scalars at which elements 0..3 sit; element 4 sits at 1.
    pub breakpoints: [f64; BREAKPOINTS],
    pub elements: [Element; ELEMENTS],
}

impl Params {
    pub const WORDS: usize = 1 + BREAKPOINTS + ELEMENTS * ELEMENT_WORDS;

    fn valid(&self) -> bool {
        let gain_ok = |g: f64| g.is_finite() && (GAIN_MIN_DB..=GAIN_MAX_DB).contains(&g);
        if !(gain_ok(self.pre_gain_db) && self.pre_gain_db <= PRE_GAIN_MAX_DB) {
            return false;
        }
        let mut prev = 0.0;
        for &b in &self.breakpoints {
            if !(b.is_finite() && b > prev && b < 1.0) {
                return false;
            }
            prev = b;
        }
        for e in &self.elements {
            if !gain_ok(e.gain_db) {
                return false;
            }
            for b in &e.bands {
                let freq_ok = b.freq_hz > 0.0 && b.freq_hz <= FREQ_MAX_HZ;
                let bw_ok = (BW_MIN..=BW_MAX).contains(&b.bw_oct);
                if !freq_ok || !gain_ok(b.gain_db) || !bw_ok {
                    return false;
                }
            }
        }
        // Two neighbouring settings with the same band on must agree on its
        // type: the unit's law across a type change is not specified.
        for pair in self.elements.windows(2) {
            for (a, b) in pair[0].bands.iter().zip(pair[1].bands.iter()) {
                if a.on && b.on && a.kind != b.kind {
                    return false;
                }
            }
        }
        true
    }

    /// Encodes exactly [`WORDS`](Self::WORDS) words; a wrong-sized
    /// destination is left untouched.
    pub fn to_words(&self, out: &mut [f64]) {
        if out.len() != Self::WORDS {
            return;
        }
        out[0] = self.pre_gain_db;
        out[1..1 + BREAKPOINTS].copy_from_slice(&self.breakpoints);
        for (i, e) in self.elements.iter().enumerate() {
            let at = 1 + BREAKPOINTS + i * ELEMENT_WORDS;
            out[at] = e.gain_db;
            for (j, b) in e.bands.iter().enumerate() {
                let w = &mut out[at + 1 + j * BAND_WORDS..at + 1 + (j + 1) * BAND_WORDS];
                w[0] = if b.on { 1.0 } else { 0.0 };
                w[1] = b.kind as i64 as f64;
                w[2] = b.freq_hz;
                w[3] = b.gain_db;
                w[4] = b.bw_oct;
            }
        }
    }

    pub fn from_words(words: &[f64]) -> Option<Params> {
        if words.len() != Self::WORDS {
            return None;
        }
        let mut p = Params { pre_gain_db: words[0], ..Params::default() };
        p.breakpoints.copy_from_slice(&words[1..1 + BREAKPOINTS]);
        for (i, e) in p.elements.iter_mut().enumerate() {
            let at = 1 + BREAKPOINTS + i * ELEMENT_WORDS;
            e.gain_db = words[at];
            for (j, b) in e.bands.iter_mut().enumerate() {
                let w = &words[at + 1 + j * BAND_WORDS..at + 1 + (j + 1) * BAND_WORDS];
                b.on = if w[0] == 1.0 {
                    true
                } else if w[0] == 0.0 {
                    false
                } else {
                    return None;
                };
                b.kind = Kind::from_code(w[1])?;
                b.freq_hz = w[2];
                b.gain_db = w[3];
                b.bw_oct = w[4];
            }
        }
        p.valid().then_some(p)
    }
}

/// Normalized biquad `b0, b1, b2, a1, a2`.
type Coeffs = [f64; 5];

fn db_to_lin(db: f64) -> f64 {
    pow(10.0, db / 20.0)
}

fn lerp(a: f64, b: f64, w: f64) -> f64 {
    a + (b - a) * w
}

fn norm(b: [f64; 3], a: [f64; 3]) -> Coeffs {
    [b[0] / a[0], b[1] / a[0], b[2] / a[0], a[1] / a[0], a[2] / a[0]]
}

/// One band's filter at sample rate `fs`, or `None` for a wire.
fn design(kind: Kind, f: f64, g: f64, bw: f64, fs: f64) -> Option<Coeffs> {
    let nyq = fs / 2.0;
    if f.is_nan() || f <= 0.0 || f >= nyq {
        // A low pass at or above Nyquist passes everything; the compiler
        // refuses every other band there, so a hostile block gets a wire.
        return None;
    }
    if kind.has_gain() && g == 0.0 {
        return None;
    }
    let w = 2.0 * core::f64::consts::PI * f / fs;
    let (s, c) = (sin(w), cos(w));
    Some(match kind {
        Kind::Peak => {
            let al = s * sinh(core::f64::consts::LN_2 / 2.0 * bw * w / s);
            let gl = db_to_lin(g);
            norm([1.0 + al * gl, -2.0 * c, 1.0 - al * gl], [1.0 + al, -2.0 * c, 1.0 - al])
        }
        Kind::LowPass | Kind::HighPass => {
            let al = s / core::f64::consts::SQRT_2;
            let (b0, b1) =
                if kind == Kind::LowPass { ((1.0 - c) / 2.0, 1.0 - c) } else { ((1.0 + c) / 2.0, -(1.0 + c)) };
            norm([b0, b1, b0], [1.0 + al, -2.0 * c, 1.0 - al])
        }
        Kind::LowShelf | Kind::HighShelf => {
            let a = pow(10.0, g / 40.0);
            let sa = 2.0 * sqrt(a) * s / core::f64::consts::SQRT_2;
            let (ap, am) = (a + 1.0, a - 1.0);
            if kind == Kind::LowShelf {
                norm(
                    [a * (ap - am * c + sa), 2.0 * a * (am - ap * c), a * (ap - am * c - sa)],
                    [ap + am * c + sa, -2.0 * (am + ap * c), ap + am * c - sa],
                )
            } else {
                norm(
                    [a * (ap + am * c + sa), -2.0 * a * (am + ap * c), a * (ap + am * c - sa)],
                    [ap - am * c + sa, 2.0 * (am - ap * c), ap - am * c - sa],
                )
            }
        }
    })
}

/// One resolved setting: the output gain and each band's filter.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Setting {
    gain: f64,
    filters: [Option<Coeffs>; BANDS],
}

fn resolve(p: &Params, volume: f64, fs: f64) -> Setting {
    let v = if volume.is_finite() { volume.clamp(0.0, 1.0) } else { 1.0 };
    let at = |i: usize| if i < BREAKPOINTS { p.breakpoints[i] } else { 1.0 };
    let (lo, hi, w) = if v <= at(0) {
        (0, 0, 0.0)
    } else {
        let i = (1..ELEMENTS).find(|&i| v <= at(i)).unwrap_or(ELEMENTS - 1);
        (i - 1, i, (v - at(i - 1)) / (at(i) - at(i - 1)))
    };
    let (el, eh) = (&p.elements[lo], &p.elements[hi]);
    let gain = db_to_lin(lerp(el.gain_db, eh.gain_db, w) + p.pre_gain_db);
    let mut filters = [None; BANDS];
    for (k, f) in filters.iter_mut().enumerate() {
        let (a, b) = (&el.bands[k], &eh.bands[k]);
        *f = match (a.on, b.on) {
            (false, false) => None,
            (true, true) => design(
                a.kind,
                lerp(a.freq_hz, b.freq_hz, w),
                lerp(a.gain_db, b.gain_db, w),
                lerp(a.bw_oct, b.bw_oct, w),
                fs,
            ),
            (lo_on, _) => {
                let (on, s) = if lo_on { (a, 1.0 - w) } else { (b, w) };
                match on.kind {
                    Kind::LowPass => design(on.kind, lerp(fs / 2.0, on.freq_hz, s), 0.0, on.bw_oct, fs),
                    Kind::HighPass => design(on.kind, (s * b.freq_hz).max(HP_FADE_FLOOR_HZ), 0.0, on.bw_oct, fs),
                    _ => design(on.kind, on.freq_hz, s * on.gain_db, on.bw_oct, fs),
                }
            }
        };
    }
    Setting { gain, filters }
}

/// Direct-form-I state per channel and band: `x1, x2, y1, y2`.
type State = [[[f64; 4]; BANDS]; MAX_CHANNELS];

fn run_setting(s: &Setting, st: &mut [[f64; 4]; BANDS], x: f64) -> f64 {
    let mut v = x;
    for (f, z) in s.filters.iter().zip(st.iter_mut()) {
        if let Some([b0, b1, b2, a1, a2]) = f {
            let y = b0 * v + b1 * z[0] + b2 * z[1] - a1 * z[2] - a2 * z[3];
            *z = [v, z[0], y, z[2]];
            v = y;
        }
    }
    v * s.gain
}

pub struct Run {
    params: Params,
    fs: f64,
    channels: usize,
    enabled: bool,
    cur: Setting,
    cur_state: State,
    next: Setting,
    next_state: State,
    fade_len: usize,
    /// Frames into the running crossfade, `None` when not fading.
    fade: Option<usize>,
    pending: Option<f64>,
    /// The volume scalar of the current (or, while fading, the next) setting.
    volume: f64,
    started: bool,
}

impl Run {
    /// Builds the box at full volume (`mvsc` = 1).
    pub fn new(params: &Params, fs: f64, channels: usize) -> Run {
        let enabled = params.valid() && fs.is_finite() && fs > 0.0 && (1..=MAX_CHANNELS).contains(&channels);
        let fs_ok = if fs.is_finite() && fs > 0.0 { fs } else { 48_000.0 };
        let cur = resolve(params, 1.0, fs_ok);
        let fade_len = libm::round(FADE_S * fs_ok).max(1.0) as usize;
        Run {
            params: *params,
            fs: fs_ok,
            channels: channels.min(MAX_CHANNELS),
            enabled,
            cur,
            cur_state: [[[0.0; 4]; BANDS]; MAX_CHANNELS],
            next: cur,
            next_state: [[[0.0; 4]; BANDS]; MAX_CHANNELS],
            fade_len,
            fade: None,
            pending: None,
            volume: 1.0,
            started: false,
        }
    }

    /// The graph's master volume scalar (`mvsc`). Takes effect at the next
    /// processed frame (at once before the first one, or after a reset).
    pub fn set_volume_scalar(&mut self, mvsc: f64) {
        if mvsc.is_finite() {
            self.pending = Some(mvsc);
            if !self.started {
                self.take_pending();
            }
        }
    }

    pub fn reset(&mut self) {
        if self.fade.take().is_some() {
            self.cur = self.next;
        }
        self.cur_state = [[[0.0; 4]; BANDS]; MAX_CHANNELS];
        self.next_state = self.cur_state;
        self.started = false;
        self.take_pending();
    }

    pub fn latency(&self) -> usize {
        0
    }

    fn take_pending(&mut self) {
        let Some(v) = self.pending.take() else { return };
        let v = v.clamp(0.0, 1.0);
        let s = resolve(&self.params, v, self.fs);
        if !self.started {
            self.cur = s;
            self.next = s;
        } else if v != self.volume {
            // The unit fades on every change of the volume value, even one
            // that resolves to the same setting; that fade still holds off
            // the next change until it ends.
            self.next = s;
            self.next_state = self.cur_state;
            self.fade = Some(0);
        }
        self.volume = v;
    }

    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize) {
        if !self.enabled {
            return;
        }
        let ch = self.channels.min(bufs.len());
        let frames = bufs[..ch].iter().fold(frames, |n, b| n.min(b.len()));
        for i in 0..frames {
            if self.fade.is_none() && self.pending.is_some() {
                self.take_pending();
            }
            self.started = true;
            match self.fade {
                None => {
                    for (c, b) in bufs[..ch].iter_mut().enumerate() {
                        let x = if b[i].is_finite() { b[i] as f64 } else { 0.0 };
                        b[i] = run_setting(&self.cur, &mut self.cur_state[c], x) as f32;
                    }
                }
                Some(k) => {
                    let r = k as f64 / self.fade_len as f64;
                    for (c, b) in bufs[..ch].iter_mut().enumerate() {
                        let x = if b[i].is_finite() { b[i] as f64 } else { 0.0 };
                        let yo = run_setting(&self.cur, &mut self.cur_state[c], x);
                        let yn = run_setting(&self.next, &mut self.next_state[c], x);
                        b[i] = ((1.0 - r) * yo + r * yn) as f32;
                    }
                    if k + 1 >= self.fade_len {
                        self.cur = self.next;
                        self.cur_state = self.next_state;
                        self.fade = None;
                    } else {
                        self.fade = Some(k + 1);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic() -> Params {
        let mut p = Params { breakpoints: [0.2, 0.4, 0.6, 0.8], ..Params::default() };
        for (i, e) in p.elements.iter_mut().enumerate() {
            e.gain_db = i as f64;
            e.bands[0] =
                Band { on: true, kind: Kind::LowShelf, freq_hz: 150.0, gain_db: 12.0 - 3.0 * i as f64, bw_oct: 0.5 };
        }
        p.elements[4].bands[7] = Band { on: true, kind: Kind::HighPass, freq_hz: 100.0, gain_db: 0.0, bw_oct: 0.5 };
        p
    }

    #[test]
    fn words_round_trip() {
        let p = synthetic();
        let mut w = [0.0; Params::WORDS];
        p.to_words(&mut w);
        assert_eq!(Params::from_words(&w), Some(p));
        w[1] = 0.9; // breakpoints out of order
        assert_eq!(Params::from_words(&w), None);
    }

    #[test]
    fn type_change_between_neighbours_is_refused() {
        let mut p = synthetic();
        p.elements[1].bands[0].kind = Kind::HighShelf;
        let mut w = [0.0; Params::WORDS];
        p.to_words(&mut w);
        assert_eq!(Params::from_words(&w), None);
    }

    #[test]
    fn element_gain_interpolates_in_db() {
        let p = synthetic();
        let s = resolve(&p, 0.3, 48_000.0);
        assert!((20.0 * libm::log10(s.gain) - 0.5).abs() < 1e-12);
        assert_eq!(resolve(&p, -1.0, 48_000.0), resolve(&p, 0.1, 48_000.0));
        assert_eq!(resolve(&p, 2.0, 48_000.0), resolve(&p, 1.0, 48_000.0));
    }

    #[test]
    fn high_pass_fade_floor() {
        let p = synthetic();
        // 5 % of the way into element 4: 5 Hz, held at the floor.
        let s = resolve(&p, 0.81, 48_000.0);
        let floor = design(Kind::HighPass, HP_FADE_FLOOR_HZ, 0.0, 0.5, 48_000.0);
        assert_eq!(s.filters[7], floor);
    }

    #[test]
    fn crossfade_is_linear_and_twenty_ms() {
        let mut p = Params { breakpoints: [0.2, 0.4, 0.6, 0.8], ..Params::default() };
        p.elements[4].gain_db = -12.0;
        let mut r = Run::new(&p, 48_000.0, 1);
        r.set_volume_scalar(0.8);
        let mut a = [0.5f32; 64];
        r.process(&mut [&mut a], 64);
        assert_eq!(a[63], 0.5);
        r.set_volume_scalar(1.0);
        let mut b = [0.5f32; 2000];
        r.process(&mut [&mut b], 2000);
        let g = db_to_lin(-12.0);
        assert_eq!(b[0], 0.5);
        assert!(((b[480] as f64) - 0.5 * (0.5 + 0.5 * g)).abs() < 1e-6);
        assert!(((b[960] as f64) - 0.5 * g).abs() < 1e-6);
    }

    #[test]
    fn a_volume_change_within_one_setting_still_holds_off_the_next() {
        let mut p = Params { breakpoints: [0.2, 0.4, 0.6, 0.8], ..Params::default() };
        p.elements[4].gain_db = -12.0;
        let mut r = Run::new(&p, 48_000.0, 1);
        r.set_volume_scalar(0.05);
        let mut a = [0.5f32; 64];
        r.process(&mut [&mut a], 64);
        r.set_volume_scalar(0.1); // same setting: a silent fade
        r.process(&mut [&mut a], 64);
        r.set_volume_scalar(1.0); // waits for that fade
        let mut b = [0.5f32; 1000];
        r.process(&mut [&mut b], 1000);
        assert_eq!(b[895], 0.5);
        assert!(b[897] < 0.5);
    }

    #[test]
    fn non_finite_input_is_silence() {
        let mut r = Run::new(&synthetic(), 48_000.0, 2);
        let mut a = [f32::NAN, 0.0, f32::INFINITY, 0.0];
        let mut b = [0.0f32; 4];
        r.process(&mut [&mut a, &mut b], 4);
        assert!(a.iter().all(|x| x.is_finite()));
    }
}
