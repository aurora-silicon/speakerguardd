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
const CACHED_HP_ORDER: f64 = 2.0;
const CACHED_LP_HZ: f64 = 10_000.0;
const CACHED_LP_ORDER: f64 = 4.0;
/// The unit's construction-default effect mixer (both wet/dry taper points).
/// The cached filter is built before the saved parameters are restored, so
/// it carries this mix, not the saved one; likewise the saved gain trim does
/// not reach the cached filter (its construction default is 0 dB).
const CACHED_EFFECT_MIX_PERCENT: f64 = 75.0;

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
    if let Some(Value::Array(values)) = d.get(key) {
        if values.len() != BINS {
            return Err(bad("uncompressed spectrum has the wrong bin count"));
        }
        let mut out = [0.0; BINS];
        for (dst, value) in out.iter_mut().zip(values) {
            let v = value
                .as_number()
                .filter(|v| v.is_finite() && (*v as f32).is_finite())
                .ok_or_else(|| bad("invalid uncompressed spectrum number"))?;
            *dst = v as f32 as f64;
        }
        return Ok(out);
    }
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

fn band_weight(f: f64, hp: f64, hp_order: f64, lp: f64, lp_order: f64) -> f64 {
    if f <= 0.0 {
        return 0.0;
    }
    // The crossover uses the Butterworth power response, not its amplitude
    // response.  This is why an order-two edge is 12 dB/octave here.
    // Orders are continuous controls. Preserve the existing integer path
    // for construction defaults while retaining fractional saved orders.
    let power = |x: f64, order: f64| {
        if order.fract() == 0.0 {
            x.powi(order as i32)
        } else {
            x.powf(order)
        }
    };
    let high = 1.0 / (1.0 + power(hp / f, hp_order));
    let low = 1.0 / (1.0 + power(f / lp, lp_order));
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

// Both initialization filters and the cached endpoint use the same
// spectrum-to-finite-record law. Their controls and construction rates
// differ because ClassInfo restores controls before the stream is installed.
#[allow(clippy::too_many_arguments)]
fn optimize(
    mut raw: [[Complex; BINS]; 4],
    state_rate: f64,
    rate: f64,
    hp: f64,
    hp_order: f64,
    lp: f64,
    lp_order: f64,
    length_ms: f64,
    delay: usize,
    alpha: f64,
    wet: f64,
) -> Params {
    // Apply the crossover at the dictionary's spectral sample rate before
    // remapping to the rate at which this filter was constructed.
    for (m, matrix) in raw.iter_mut().enumerate() {
        let diagonal = m == 0 || m == 3;
        for (k, bin) in matrix.iter_mut().enumerate() {
            let f = k as f64 * state_rate / FFT as f64;
            let c = band_weight(f, hp, hp_order, lp, lp_order);
            let identity = if diagonal { 1.0 } else { 0.0 };
            *bin = bin.sub_real(identity).scale(c).add_real(identity);
        }
    }

    // The optimizer linearly remaps the setup's bins before its IDFT.
    let mut remapped = [[Complex::default(); BINS]; 4];
    for (m, destination) in remapped.iter_mut().enumerate() {
        for (k, bin) in destination.iter_mut().enumerate() {
            let x = k as f64 * rate / state_rate;
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

    let mut freq = [[Complex::default(); BINS]; 4];
    for (m, destination) in freq.iter_mut().enumerate() {
        let identity = if m == 0 || m == 3 { 1.0 } else { 0.0 };
        for (k, bin) in destination.iter_mut().enumerate() {
            *bin = remapped[m][k].scale(wet).add_real((1.0 - wet) * identity);
        }
    }

    // DC is the exact direct-only endpoint (the crossover is zero there).
    // The destination Nyquist bin is remapped like every other bin; only its
    // real part survives a real transform.
    let mut nyquist = [0.0; 4];
    for (m, value) in nyquist.iter_mut().enumerate() {
        let identity = if m == 0 || m == 3 { 1.0 } else { 0.0 };
        let x = BINS as f64 * rate / state_rate;
        let lo = x.floor() as usize;
        let t = x - lo as f64;
        let re = if lo + 1 >= BINS { raw[m][BINS - 1].re } else { raw[m][lo].re * (1.0 - t) + raw[m][lo + 1].re * t };
        *value = re * wet + (1.0 - wet) * identity;
    }

    let count = (rate * length_ms / 1000.0).floor() as usize - 1;
    let mut taps = finite_fir(&freq, nyquist, count, delay, alpha);

    // The optimizer restores each matrix entry's exact DC after windowing by
    // spreading the residual uniformly across its finite record.
    for (m, vector) in taps.iter_mut().enumerate() {
        let target = if m == 0 || m == 3 { 1.0 } else { 0.0 };
        let correction = (target - vector[..count].iter().sum::<f64>()) / count as f64;
        for x in &mut vector[..count] {
            *x += correction;
        }
    }

    Params { taps, count, ..Params::default() }
}

/// Saved delay is a position on the finite FIR's tap grid. The percentage
/// conversion and nearest-tap rounding are Float32; the center tap is the
/// largest position exposed by the control.
fn group_delay(count: usize, percent: f64) -> usize {
    let fraction = percent as f32 * 0.01_f32;
    ((count as f32 * fraction).round() as usize).min(count / 2)
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
    for id in (31..=63).filter(|id| ![33, 49].contains(id) && !(35..=39).contains(id) && !(51..=55).contains(id)) {
        exact(id, 0.0, "reserved parameter")?;
    }

    // These boolean fields do not alter an explicitly supplied Setup_0/W.
    // Both live changes and pre-restoration ClassInfo controls were measured.
    for id in [33, 49] {
        if !matches!(req(id)?, 0.0 | 1.0) {
            return Err(bad(format!("reserved saved-filter flag {id} is not boolean")));
        }
    }

    // The restored controls affect the intermediate startup filter and
    // reset state. The settled cached endpoint retains construction defaults.
    let length_ms = req(12)?;
    let saved_delay_pct = req(14)?;
    if !(1.0..=20.0).contains(&length_ms) || !(0.0..=50.0).contains(&saved_delay_pct) {
        return Err(bad(format!("FIR length {length_ms} ms, group delay {saved_delay_pct}%")));
    }

    let custom = dict(&stage.preset_extra, "DspLibXTC_customFilterDictionary")?;
    let setup = dict(custom, "Setup_0")?;
    let state_rate = number(setup, "SampleRate")?;
    if state_rate != fs || fs != 48_000.0 {
        return Err(bad(format!(
            "filter state is {state_rate} Hz but compile rate is {fs} Hz (only exact 48 kHz is carried)"
        )));
    }
    let w = dict(setup, "W")?;
    let compressed = match w.get("WDataIsCompressed") {
        None | Some(Value::Bool(false)) => false,
        Some(Value::Bool(true)) => true,
        Some(_) => return Err(bad("spectrum compression flag is not boolean")),
    };
    for name in ["11", "12", "21", "22"] {
        for part in ["real", "imag"] {
            let value = w.get(&format!("W{name}{part}"));
            if (compressed && !matches!(value, Some(Value::Data(_))))
                || (!compressed && !matches!(value, Some(Value::Array(_))))
            {
                return Err(bad("spectrum storage disagrees with its compression flag"));
            }
        }
    }
    let raw = [spectrum(w, "11")?, spectrum(w, "12")?, spectrum(w, "21")?, spectrum(w, "22")?];
    let cached_count = (CONSTRUCTION_RATE * CACHED_LENGTH_MS / 1000.0).floor() as usize - 1;
    let mut p = optimize(
        raw,
        state_rate,
        CONSTRUCTION_RATE,
        CACHED_HP_HZ,
        CACHED_HP_ORDER,
        CACHED_LP_HZ,
        CACHED_LP_ORDER,
        CACHED_LENGTH_MS,
        cached_count.div_ceil(2),
        1.0,
        CACHED_EFFECT_MIX_PERCENT / 100.0,
    );
    let restored_count = (fs * length_ms / 1000.0).floor() as usize - 1;
    let reported_latency = group_delay(restored_count, saved_delay_pct);
    p.reported_latency = reported_latency;

    let order = |id| -> Result<f64> {
        let value = req(id)?;
        if !(1.0..=10.0).contains(&value) {
            return Err(bad(format!("crossover order {id} is {value}")));
        }
        Ok(value)
    };
    let hp = req(6)?;
    let lp = req(4)?;
    let alpha = req(13)?;
    let wet = req(20)? / 100.0;
    let crossfade_ms = req(22)?;
    if !(0.0..fs / 2.0).contains(&hp)
        || hp == 0.0
        || !(0.0..fs / 2.0).contains(&lp)
        || lp == 0.0
        || !(0.0..=1.0).contains(&alpha)
        || !(0.0..=1.0).contains(&wet)
        || !(100.0..=1000.0).contains(&crossfade_ms)
    {
        return Err(bad("restored crossover, window, mix or crossfade duration is out of range"));
    }
    // At the carried system volume (0 dB) the upper taper point applies.
    // Saved negative endpoints also reach this endpoint. Coincident or
    // reversed points are carried only for a constant wet/dry value.
    let upper_volume = req(19)?;
    if !(-100.0..=0.0).contains(&upper_volume)
        || !(-100.0..=0.0).contains(&req(17)?)
        || (req(17)? >= upper_volume && req(18)? != req(20)?)
    {
        return Err(bad("wet/dry taper volumes do not bracket the carried system volume"));
    }
    exact(21, 0.0, "startup gain trim")?;
    let initial_count = (fs * CACHED_LENGTH_MS / 1000.0).floor() as usize - 1;
    let initial = optimize(
        raw,
        state_rate,
        fs,
        CACHED_HP_HZ,
        CACHED_HP_ORDER,
        CACHED_LP_HZ,
        CACHED_LP_ORDER,
        CACHED_LENGTH_MS,
        initial_count / 2,
        1.0,
        CACHED_EFFECT_MIX_PERCENT / 100.0,
    );
    let restored = optimize(raw, state_rate, fs, hp, order(7)?, lp, order(5)?, length_ms, reported_latency, alpha, wet);
    let crossfade_frames = (fs * crossfade_ms / 1000.0).round() as usize;

    diags.info(
        "xtc-setup",
        stage.name.clone(),
        format!(
            "Setup_0, {}-tap cached stereo FIR with queued startup filters, reported delay {:.3}%",
            p.count, saved_delay_pct
        ),
    );
    p.with_startup(initial, restored, crossfade_frames)
        .ok_or_else(|| bad("compiled FIR violates the bounded core parameter format"))
        .map(Some)
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
            strip_properties: Vec::new(),
            property_files: Vec::new(),
            before_protection: true,
        }
    }

    #[test]
    fn compiles_synthetic_identity_without_shipping_tuning() {
        let mut d = Diagnostics::default();
        let p = compile(&stage(0.0), 48_000.0, &mut d).unwrap().unwrap();
        assert_eq!(p.count, 87);
        assert_eq!(p.reported_latency, 9);
        assert!(p.taps[1][..p.count].iter().all(|x| x.abs() < 1e-12));
        assert!(p.taps[2][..p.count].iter().all(|x| x.abs() < 1e-12));
        assert!((p.taps[0][..p.count].iter().sum::<f64>() - 1.0).abs() < 1e-12);
    }

    fn with_params(mut s: Stage, changes: &[(u32, f32)]) -> Stage {
        let params = &mut s.state.as_mut().unwrap().sections[0].params;
        for &(id, v) in changes {
            params.iter_mut().find(|(i, _)| *i == id).unwrap().1 = v;
        }
        s
    }

    fn effect_stage(cross: f32) -> Stage {
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
        *w.get_mut("W12real").unwrap() = Value::Data(data(cross));
        *w.get_mut("W21real").unwrap() = Value::Data(data(cross));
        s
    }

    #[test]
    fn saved_controls_do_not_change_the_settled_filter() {
        let base = compile(&effect_stage(-0.5), 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        assert!(base.taps[1][..base.count].iter().any(|x| x.abs() > 1e-6));
        for changes in [
            &[(18, 65.0), (20, 65.0)][..],
            &[(17, -50.0), (18, 45.0), (20, 45.0)][..],
            &[(4, 9_000.0), (5, 5.99), (6, 2_000.0), (7, 2.0), (13, 1.0)][..],
        ] {
            let s = with_params(effect_stage(-0.5), changes);
            let p = compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
            assert_eq!(p.taps, base.taps, "{changes:?}");
        }
    }

    #[test]
    fn carries_restored_controls_and_crossfade_duration_separately() {
        let s = with_params(effect_stage(-0.5), &[(22, 250.0), (20, 60.0)]);
        let p = compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        let startup = p.startup.as_ref().unwrap();
        assert_eq!(startup.crossfade_frames, 12_000);
        assert_eq!(startup.initial.count, 95);
        assert_eq!(startup.restored.count, 95);
        assert_ne!(startup.initial.taps, startup.restored.taps);
        assert!(compile(&with_params(s, &[(21, 3.0)]), 48_000.0, &mut Diagnostics::default()).is_err());
        let fractional = |order| {
            compile(&with_params(effect_stage(-0.5), &[(5, order)]), 48_000.0, &mut Diagnostics::default())
                .unwrap()
                .unwrap()
                .startup
                .unwrap()
                .restored
        };
        assert_ne!(fractional(5.25).taps, fractional(5.75).taps);
    }

    #[test]
    fn cached_cross_term_uses_the_construction_mix() {
        // A flat cross spectrum c inside the crossover band reaches the
        // filter scaled by the construction mix (75 %) and the crossover.
        let p = compile(&effect_stage(-0.5), 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        let peak = p.taps[1][..p.count].iter().fold(0.0f64, |m, x| m.max(x.abs()));
        let q = compile(&effect_stage(-1.0), 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        let peak2 = q.taps[1][..q.count].iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!((peak2 / peak - 2.0).abs() < 1e-9);
        assert!(p.taps[1][..p.count].iter().sum::<f64>().abs() < 1e-12);
    }

    #[test]
    fn delay_uses_the_finite_tap_grid_and_center_limit() {
        for (length_ms, percent, frames) in [
            (2.0, 10.0, 9),
            (2.0, 10.416, 10),
            (2.0, 10.417, 10),
            (2.0, 50.0, 47),
            (2.01, 50.0, 47),
            (1.5, 25.0, 18),
            (3.0, 33.3, 48),
        ] {
            let count = (48.0_f64 * length_ms).floor() as usize - 1;
            assert_eq!(group_delay(count, percent), frames);
        }
    }

    #[test]
    fn uncompressed_spectra_and_dormant_flags_preserve_the_filter() {
        let mut s = effect_stage(-0.5);
        let before = compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
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
        for name in ["11", "12", "21", "22"] {
            for part in ["real", "imag"] {
                let key = format!("W{name}{part}");
                let values = compressed_f32(w, &key).unwrap();
                *w.get_mut(&key).unwrap() = Value::Array(values.into_iter().map(Value::Real).collect());
            }
        }
        *w.get_mut("WDataIsCompressed").unwrap() = Value::Bool(false);
        let uncompressed = compile(&s, 48_000.0, &mut Diagnostics::default()).unwrap().unwrap();
        assert_eq!(before, uncompressed);
        let flags = with_params(s.clone(), &[(33, 1.0), (49, 1.0), (17, -1.0), (19, -1.0), (18, 40.0), (20, 40.0)]);
        assert_eq!(uncompressed, compile(&flags, 48_000.0, &mut Diagnostics::default()).unwrap().unwrap());
        assert!(compile(&with_params(s, &[(33, 2.0)]), 48_000.0, &mut Diagnostics::default()).is_err());
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
