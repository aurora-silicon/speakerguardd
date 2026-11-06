//! Compile `aufx/axtc/appl` (XTC) from its ClassInfo preset.
//!
//! XTC stores four 512-bin complex spectra in
//! `DspLibXTC_customFilterDictionary/Setup_0/W`.  Each real and imaginary
//! vector is a raw-DEFLATE stream of little-endian `f32`.  The AudioUnit
//! completes the real spectrum, applies its zero-phase crossover and
//! wet/dry law, performs a 1024-point inverse DFT, moves the requested group
//! delay into the finite record, and windows that record.  The result, not
//! the Apple state, is what crosses into `sgv2-core`.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::plist::{Dict, Value};
use crate::tuning::Stage;
use flate2::read::DeflateDecoder;
use sgv2_core::voicing::xtc::{Params, MAX_TAPS};
use std::f64::consts::PI;
use std::io::Read;

pub const AXTC: FourCc = FourCc::from_bytes(*b"axtc");
const AUFX: FourCc = FourCc::from_bytes(*b"aufx");
const APPL: FourCc = FourCc::from_bytes(*b"appl");
const FFT: usize = 1024;
const BINS: usize = FFT / 2;
/// The AudioUnit constructs its cached ClassInfo FIR before the host's 48 kHz
/// stream format arrives, at the AU's 44.1 kHz construction rate.
const CONSTRUCTION_RATE: f64 = 44_100.0;
const CACHED_LENGTH_MS: f64 = 2.0;
const CACHED_HP_HZ: f64 = 900.0;
const CACHED_HP_ORDER: u32 = 2;
const CACHED_LP_HZ: f64 = 10_000.0;
const CACHED_LP_ORDER: u32 = 4;

#[derive(Clone, Copy, Debug, Default)]
struct Complex {
    re: f64,
    im: f64,
}

impl Complex {
    fn sub_real(self, x: f64) -> Self {
        Self { re: self.re - x, im: self.im }
    }
    fn add_real(self, x: f64) -> Self {
        Self { re: self.re + x, im: self.im }
    }
    fn scale(self, x: f64) -> Self {
        Self { re: self.re * x, im: self.im * x }
    }
}

fn bad(message: impl Into<String>) -> Error {
    Error::format("XTC", message)
}

fn dict<'a>(d: &'a Dict, key: &str) -> Result<&'a Dict> {
    match d.get(key) {
        Some(Value::Dict(v)) => Ok(v),
        Some(v) => Err(bad(format!("{key} is {}, expected a dictionary", v.kind()))),
        None => Err(Error::missing("XTC", key)),
    }
}

fn number(d: &Dict, key: &str) -> Result<f64> {
    d.get(key)
        .and_then(Value::as_number)
        .filter(|v| v.is_finite())
        .ok_or_else(|| bad(format!("{key} is missing or not a finite number")))
}

fn compressed_f32(d: &Dict, key: &str) -> Result<[f64; BINS]> {
    let bytes =
        d.get(key).and_then(Value::as_data).ok_or_else(|| bad(format!("Setup_0/W/{key} is missing or not data")))?;
    let mut raw = Vec::new();
    DeflateDecoder::new(bytes)
        .read_to_end(&mut raw)
        .map_err(|e| bad(format!("Setup_0/W/{key} raw-DEFLATE stream: {e}")))?;
    if raw.len() != BINS * 4 {
        return Err(bad(format!("Setup_0/W/{key} expands to {} bytes, expected {}", raw.len(), BINS * 4)));
    }
    let mut out = [0.0; BINS];
    for (dst, b) in out.iter_mut().zip(raw.chunks_exact(4)) {
        let v = f32::from_le_bytes(b.try_into().unwrap()) as f64;
        if !v.is_finite() {
            return Err(bad(format!("Setup_0/W/{key} contains a non-finite coefficient")));
        }
        *dst = v;
    }
    Ok(out)
}

fn spectrum(w: &Dict, name: &str) -> Result<[Complex; BINS]> {
    let re = compressed_f32(w, &format!("W{name}real"))?;
    let im = compressed_f32(w, &format!("W{name}imag"))?;
    let mut out = [Complex::default(); BINS];
    for i in 0..BINS {
        out[i] = Complex { re: re[i], im: im[i] };
    }
    Ok(out)
}

fn taper_at(volume: f64, v0: f64, w0: f64, v1: f64, w1: f64) -> Result<f64> {
    if !(-100.0..=0.0).contains(&v0)
        || !(-100.0..=0.0).contains(&v1)
        || !(0.0..=100.0).contains(&w0)
        || !(0.0..=100.0).contains(&w1)
        || v0 > v1
    {
        return Err(bad(format!("invalid wet/dry taper ({v0}, {w0}) to ({v1}, {w1})")));
    }
    if v0 == v1 {
        if w0 != w1 {
            return Err(bad("wet/dry taper has two different mixers at one volume"));
        }
        return Ok(w0 / 100.0);
    }
    let t = ((volume - v0) / (v1 - v0)).clamp(0.0, 1.0);
    Ok((w0 + t * (w1 - w0)) / 100.0)
}

fn band_weight(f: f64, hp: f64, hp_order: u32, lp: f64, lp_order: u32) -> f64 {
    if f <= 0.0 {
        return 0.0;
    }
    // The crossover uses the Butterworth power response, not its amplitude
    // response.  This is why an order-two edge is 12 dB/octave here.
    let high = 1.0 / (1.0 + (hp / f).powi(hp_order as i32));
    let low = 1.0 / (1.0 + (f / lp).powi(lp_order as i32));
    high * low
}

fn tukey(i: usize, n: usize, alpha: f64) -> f64 {
    if n <= 1 || alpha <= 0.0 {
        return 1.0;
    }
    let x = i as f64 / (n - 1) as f64;
    if x < alpha / 2.0 {
        0.5 * (1.0 + (PI * (2.0 * x / alpha - 1.0)).cos())
    } else if x > 1.0 - alpha / 2.0 {
        0.5 * (1.0 + (PI * (2.0 * x / alpha - 2.0 / alpha + 1.0)).cos())
    } else {
        1.0
    }
}

/// A real inverse DFT from bins 0 through Nyquist.  The preset has bins
/// 0..511; crossover makes DC and Nyquist exact direct-only endpoints.
fn finite_fir(
    freq: &[[Complex; BINS]; 4],
    nyquist: [f64; 4],
    count: usize,
    delay: usize,
    alpha: f64,
) -> [[f64; MAX_TAPS]; 4] {
    let mut out = [[0.0; MAX_TAPS]; 4];
    for m in 0..4 {
        for (i, tap) in out[m].iter_mut().enumerate().take(count) {
            let source = (FFT + i - delay % FFT) % FFT;
            let mut x = freq[m][0].re + if source % 2 == 0 { nyquist[m] } else { -nyquist[m] };
            for (k, bin) in freq[m].iter().enumerate().skip(1) {
                let phase = 2.0 * PI * k as f64 * source as f64 / FFT as f64;
                x += 2.0 * (bin.re * phase.cos() - bin.im * phase.sin());
            }
            *tap = x / FFT as f64 * tukey(i, count, alpha);
        }
    }
    out
}

/// Compile the one exact state used by the builtin-speaker graph.  States
/// which would require live orientation, a different setup, test signals,
/// muting, swapping, or an optimizer bypass are refused.
pub fn compile(stage: &Stage, fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    let Some(unit) = stage.unit else { return Err(bad("stage has no AudioUnit identity")) };
    if unit.kind != AUFX || unit.subtype != AXTC || unit.manufacturer != APPL {
        return Err(bad(format!("stage is {unit}, expected aufx/axtc/appl")));
    }
    if stage.bypass {
        return Ok(None);
    }
    if stage.channels != Some(2) {
        return Err(bad(format!("stage has {:?} channels, expected stereo", stage.channels)));
    }
    let g = stage.state.as_ref().and_then(|s| s.global()).ok_or_else(|| bad("preset has no global parameter state"))?;
    let req = |id: u32| -> Result<f64> {
        g.get(id)
            .map(f64::from)
            .filter(|v| v.is_finite())
            .ok_or_else(|| bad(format!("saved parameter {id} is missing")))
    };
    let exact = |id: u32, wanted: f64, what: &str| -> Result<()> {
        let got = req(id)?;
        if got == wanted {
            Ok(())
        } else {
            Err(bad(format!("{what} (parameter {id}) is {got}, expected {wanted}")))
        }
    };

    let mode = req(0)?;
    if mode == 0.0 || mode == 1.0 {
        return Ok(None);
    }
    exact(0, 2.0, "bypass mode")?;
    exact(1, 0.0, "orientation")?;
    exact(2, 0.0, "crossover bypass")?;
    exact(3, 0.0, "FIR optimizer bypass")?;
    for (id, what) in [
        (8, "effect mute"),
        (9, "direct mute"),
        (10, "cross-term swap"),
        (11, "left/right swap"),
        (15, "test generator"),
        (30, "mixer bypass"),
    ] {
        exact(id, 0.0, what)?;
    }
    exact(16, 0.0, "system volume")?;
    if g.get(1_869_769_061).is_some_and(|v| v != req(1).unwrap_or(f64::NAN) as f32)
        || g.get(1_937_141_612).is_some_and(|v| v != req(16).unwrap_or(f64::NAN) as f32)
    {
        return Err(bad("FourCC parameter aliases disagree with orientation or system volume"));
    }
    for id in 23..=29 {
        exact(id, 0.0, "orientation setup")?;
    }
    for id in (31..=63).filter(|id| !(35..=39).contains(id) && !(51..=55).contains(id)) {
        exact(id, 0.0, "reserved parameter")?;
    }

    let hp = req(6)?;
    let lp = req(4)?;
    let order = |id: u32, what: &str| -> Result<u32> {
        let v = req(id)?;
        if (1.0..=10.0).contains(&v) && v.fract() == 0.0 {
            Ok(v as u32)
        } else {
            Err(bad(format!("{what} order {v}")))
        }
    };
    let _lp_order = order(5, "low-pass")?;
    let _hp_order = order(7, "high-pass")?;
    if !(0.0 < hp && hp < lp && lp < fs / 2.0) {
        return Err(bad(format!("crossover {hp}..{lp} Hz is invalid at {fs} Hz")));
    }
    let length_ms = req(12)?;
    if length_ms != CACHED_LENGTH_MS {
        return Err(bad(format!("saved FIR length is {length_ms} ms; only the exact 2 ms ClassInfo cache is carried")));
    }
    let saved_alpha = req(13)?;
    let saved_delay_pct = req(14)?;
    if !(0.0..=1.0).contains(&saved_alpha) || !(0.0..=50.0).contains(&saved_delay_pct) {
        return Err(bad(format!("Tukey {saved_alpha}, group delay {saved_delay_pct}%")));
    }
    let (v0, w0, v1, w1) = (req(17)?, req(18)?, req(19)?, req(20)?);
    let wet = taper_at(0.0, v0, w0, v1, w1)?;
    if w0 != w1 {
        return Err(bad(format!(
            "wet/dry snapshot changes from {w0}% to {w1}%; only a constant saved taper is identified exactly"
        )));
    }
    let trim = req(21)?;
    if !(-6.0..=6.0).contains(&trim) {
        return Err(bad(format!("gain trim {trim} dB")));
    }
    let trim_gain = 10f64.powf(trim / 20.0);

    let custom = dict(&stage.preset_extra, "DspLibXTC_customFilterDictionary")?;
    let setup = dict(custom, "Setup_0")?;
    let state_rate = number(setup, "SampleRate")?;
    if state_rate != fs || fs != 48_000.0 {
        return Err(bad(format!(
            "filter state is {state_rate} Hz but compile rate is {fs} Hz (only exact 48 kHz is carried)"
        )));
    }
    let w = dict(setup, "W")?;
    if w.get("WDataIsCompressed").and_then(Value::as_bool) != Some(true) {
        return Err(bad("Setup_0/W is not marked raw-DEFLATE compressed"));
    }
    let mut raw = [spectrum(w, "11")?, spectrum(w, "12")?, spectrum(w, "21")?, spectrum(w, "22")?];
    // ClassInfo creates the FIR as soon as it reads the custom dictionary.
    // At that point the ordinary AU parameter blob has not restored the
    // saved crossover yet, so the construction defaults are normative.
    for (m, matrix) in raw.iter_mut().enumerate() {
        let diagonal = m == 0 || m == 3;
        for (k, bin) in matrix.iter_mut().enumerate() {
            let f = k as f64 * state_rate / FFT as f64;
            let c = band_weight(f, CACHED_HP_HZ, CACHED_HP_ORDER, CACHED_LP_HZ, CACHED_LP_ORDER);
            let identity = if diagonal { 1.0 } else { 0.0 };
            *bin = bin.sub_real(identity).scale(c).add_real(identity);
        }
    }

    // The stream format is still at the AudioUnit's construction rate.  The
    // optimizer linearly remaps the setup's frequency bins before its IDFT.
    let mut remapped = [[Complex::default(); BINS]; 4];
    for (m, destination) in remapped.iter_mut().enumerate() {
        for (k, bin) in destination.iter_mut().enumerate() {
            let x = k as f64 * CONSTRUCTION_RATE / state_rate;
            let lo = x.floor() as usize;
            let t = x - lo as f64;
            *bin = if lo + 1 >= BINS {
                raw[m][BINS - 1]
            } else {
                Complex {
                    re: raw[m][lo].re * (1.0 - t) + raw[m][lo + 1].re * t,
                    im: raw[m][lo].im * (1.0 - t) + raw[m][lo + 1].im * t,
                }
            };
        }
    }

    let snapshot_gain = if v0 == v1 { 1.5 } else { 1.875 };
    let mut freq = [[Complex::default(); BINS]; 4];
    for (m, destination) in freq.iter_mut().enumerate() {
        let identity = if m == 0 || m == 3 { 1.0 } else { 0.0 };
        for (k, bin) in destination.iter_mut().enumerate() {
            let mixed = remapped[m][k].scale(wet).add_real((1.0 - wet) * identity);
            *bin = mixed.sub_real(identity).scale(snapshot_gain).add_real(identity).scale(trim_gain);
        }
    }

    // DC and Nyquist are exact direct-only endpoints.
    let nyquist = [trim_gain, 0.0, 0.0, trim_gain];

    // floor(2 ms * 44.1 kHz) minus the duplicated endpoint: 87 stored taps.
    // The cached optimizer is at its default 50% delay and Tukey=1 (Hann),
    // independently of the later-restored saved parameter values.
    let count = (CONSTRUCTION_RATE * CACHED_LENGTH_MS / 1000.0).floor() as usize - 1;
    let applied_delay = count.div_ceil(2);
    let mut taps = finite_fir(&freq, nyquist, count, applied_delay, 1.0);

    // The optimizer restores each matrix entry's exact DC after windowing by
    // spreading the residual uniformly across its finite record.
    for (m, vector) in taps.iter_mut().enumerate() {
        let target = if m == 0 || m == 3 { trim_gain } else { 0.0 };
        let correction = (target - vector[..count].iter().sum::<f64>()) / count as f64;
        for x in &mut vector[..count] {
            *x += correction;
        }
    }

    diags.info(
        "xtc-setup",
        stage.name.clone(),
        format!(
            "Setup_0, {count}-tap cached stereo FIR, {wet:.3} wet at svol 0 dB, reported delay {:.3}%",
            saved_delay_pct
        ),
    );
    let reported_latency = (fs * length_ms / 1000.0 * saved_delay_pct / 100.0).round() as usize;
    let p = Params { taps, count, reported_latency };
    let mut words = vec![0.0; Params::WORDS];
    p.to_words(&mut words);
    Params::from_words(&words).ok_or_else(|| bad("compiled FIR violates the bounded core parameter format")).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aupreset::{ParamSection, ParamState, UnitId};
    use flate2::write::DeflateEncoder;
    use flate2::Compression;
    use std::io::Write;

    fn data(value: f32) -> Vec<u8> {
        let mut e = DeflateEncoder::new(Vec::new(), Compression::default());
        for _ in 0..BINS {
            e.write_all(&value.to_le_bytes()).unwrap();
        }
        e.finish().unwrap()
    }

    fn insert(d: &mut Dict, key: &str, value: Value) {
        assert!(d.insert_unique(key.into(), value));
    }

    fn stage(orientation: f32) -> Stage {
        let mut params: Vec<(u32, f32)> = (0..=63).map(|id| (id, 0.0)).collect();
        let set = |p: &mut Vec<(u32, f32)>, id, v| p[id as usize] = (id, v);
        set(&mut params, 0, 2.0);
        set(&mut params, 1, orientation);
        set(&mut params, 4, 8_000.0);
        set(&mut params, 5, 4.0);
        set(&mut params, 6, 900.0);
        set(&mut params, 7, 4.0);
        set(&mut params, 12, 2.0);
        set(&mut params, 13, 0.2);
        set(&mut params, 14, 10.0);
        set(&mut params, 17, -50.0);
        set(&mut params, 18, 40.0);
        set(&mut params, 19, 0.0);
        set(&mut params, 20, 40.0);
        set(&mut params, 22, 100.0);
        params.push((1_869_769_061, orientation));
        params.push((1_937_141_612, 0.0));

        let mut w = Dict::new();
        insert(&mut w, "WDataIsCompressed", Value::Bool(true));
        for name in ["11", "12", "21", "22"] {
            let re = if name == "11" || name == "22" { 1.0 } else { 0.0 };
            insert(&mut w, &format!("W{name}real"), Value::Data(data(re)));
            insert(&mut w, &format!("W{name}imag"), Value::Data(data(0.0)));
        }
        let mut setup = Dict::new();
        insert(&mut setup, "SampleRate", Value::Real(48_000.0));
        insert(&mut setup, "W", Value::Dict(w));
        let mut custom = Dict::new();
        insert(&mut custom, "Setup_0", Value::Dict(setup));
        let mut extra = Dict::new();
        insert(&mut extra, "DspLibXTC_customFilterDictionary", Value::Dict(custom));

        Stage {
            name: "XTC".into(),
            kind: "aufx/axtc/appl".into(),
            unit: Some(UnitId { kind: AUFX, subtype: AXTC, manufacturer: APPL }),
            inputs: 1,
            outputs: 1,
            channels: Some(2),
            bypass: false,
            preset_name: Some("synthetic".into()),
            state: Some(ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] }),
            preset_extra_keys: vec!["DspLibXTC_customFilterDictionary".into()],
            preset_extra: extra,
            before_protection: true,
        }
    }

    #[test]
    fn compiles_synthetic_identity_without_shipping_tuning() {
        let mut d = Diagnostics::default();
        let p = compile(&stage(0.0), 48_000.0, &mut d).unwrap().unwrap();
        assert_eq!(p.count, 87);
        assert_eq!(p.reported_latency, 10);
        assert!(p.taps[1][..p.count].iter().all(|x| x.abs() < 1e-12));
        assert!(p.taps[2][..p.count].iter().all(|x| x.abs() < 1e-12));
        assert!((p.taps[0][..p.count].iter().sum::<f64>() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn refuses_an_orientation_not_carried_by_setup_zero() {
        let e = compile(&stage(1.0), 48_000.0, &mut Diagnostics::default()).unwrap_err();
        assert!(e.to_string().contains("orientation"));
    }

    #[test]
    fn refuses_bad_compressed_vector_length() {
        let mut s = stage(0.0);
        let custom = match s.preset_extra.get_mut("DspLibXTC_customFilterDictionary") {
            Some(Value::Dict(d)) => d,
            _ => unreachable!(),
        };
        let setup = match custom.get_mut("Setup_0") {
            Some(Value::Dict(d)) => d,
            _ => unreachable!(),
        };
        let w = match setup.get_mut("W") {
            Some(Value::Dict(d)) => d,
            _ => unreachable!(),
        };
        *w.get_mut("W11real").unwrap() = Value::Data(vec![1, 2, 3]);
        assert!(compile(&s, 48_000.0, &mut Diagnostics::default()).is_err());
    }
}
