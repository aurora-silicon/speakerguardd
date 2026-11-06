//! Compile `aufx/vrq2/appl` (VarEQ) from its saved state.
//!
//! Global scope: parameters 1-4 are the volume breakpoints of elements 0-3
//! (element 4 sits at full volume), 5 is the current volume scalar (driven
//! by the graph's `mvsc` at run time, so the saved value is not used) and
//! 7 the pre-gain in dB (0 when not saved). Group scope (4), elements 0-4:
//! parameter 0 is the element gain in dB and, for band `b` in 0..8,
//! `1000 + b` bypass, `2000 + b` filter type, `3000 + b` frequency,
//! `4000 + b` gain (dB) and `5000 + b` bandwidth (octaves).
//!
//! The whole volume dependence is carried: `sgv2_core::voicing::vareq::Run`
//! takes the volume scalar at run time. States the box cannot carry exactly
//! are refused: unknown filter types, a type change between two neighbouring
//! elements that both enable the band, breakpoints out of order, bands at or
//! above Nyquist (other than a low pass), or parameters this compiler does
//! not know.

use crate::error::{Diagnostics, Error, Result};
use crate::fourcc::FourCc;
use crate::tuning::Stage;
use sgv2_core::voicing::vareq::{Band, Kind, Params, BANDS, BREAKPOINTS, ELEMENTS};

pub const VRQ2: FourCc = FourCc::from_bytes(*b"vrq2");
const AUFX: FourCc = FourCc::from_bytes(*b"aufx");
const APPL: FourCc = FourCc::from_bytes(*b"appl");
const GLOBAL: u32 = 0;
const GROUP: u32 = 4;
const CURRENT_VOLUME: u32 = 5;
const PRE_GAIN: u32 = 7;

fn bad(message: impl Into<String>) -> Error {
    Error::format("VarEQ", message)
}

pub fn compile(stage: &Stage, fs: f64, diags: &mut Diagnostics) -> Result<Option<Params>> {
    let Some(unit) = stage.unit else { return Err(bad("stage has no AudioUnit identity")) };
    if unit.kind != AUFX || unit.subtype != VRQ2 || unit.manufacturer != APPL {
        return Err(bad(format!("stage is {unit}, expected aufx/vrq2/appl")));
    }
    if stage.bypass {
        return Ok(None);
    }
    if !(fs.is_finite() && fs > 0.0) {
        return Err(bad(format!("sample rate {fs} is not usable")));
    }
    let state = stage.state.as_ref().ok_or_else(|| bad("preset has no saved parameter state"))?;
    for s in &state.sections {
        if s.scope != GLOBAL && s.scope != GROUP {
            return Err(bad(format!("saved state has scope {} element {}, not carried", s.scope, s.element)));
        }
    }
    let g = state.global().ok_or_else(|| bad("preset has no global parameter state"))?;
    for &(id, _) in &g.params {
        if !(1..=BREAKPOINTS as u32).contains(&id) && id != CURRENT_VOLUME && id != PRE_GAIN {
            return Err(bad(format!("global parameter {id} is not carried")));
        }
    }
    let num = |v: Option<f32>, what: String| -> Result<f64> {
        v.map(f64::from).filter(|x| x.is_finite()).ok_or_else(|| bad(format!("{what} is missing or not finite")))
    };
    let mut p = Params { pre_gain_db: g.get(PRE_GAIN).map(f64::from).unwrap_or(0.0), ..Params::default() };
    for (i, b) in p.breakpoints.iter_mut().enumerate() {
        *b = num(g.get(i as u32 + 1), format!("breakpoint parameter {}", i + 1))?;
    }

    let elements = state.elements(GROUP);
    if elements.len() != ELEMENTS || elements.iter().enumerate().any(|(i, s)| s.element != i as u32) {
        return Err(bad(format!("expected group elements 0-{}, found {}", ELEMENTS - 1, elements.len())));
    }
    let nyquist = fs / 2.0;
    for (e, sec) in p.elements.iter_mut().zip(elements) {
        let ctx = |what: &str| format!("element {} {what}", sec.element);
        for &(id, _) in &sec.params {
            let band_param = (1000..6000).contains(&id) && id % 1000 < BANDS as u32;
            if id != 0 && !band_param {
                return Err(bad(ctx(&format!("parameter {id} is not carried"))));
            }
        }
        e.gain_db = num(sec.get(0), ctx("gain"))?;
        for (b, band) in e.bands.iter_mut().enumerate() {
            let get = |base: u32, what: &str| num(sec.get(base + b as u32), ctx(&format!("band {b} {what}")));
            let bypass = get(1000, "bypass")?;
            if bypass != 0.0 && bypass != 1.0 {
                return Err(bad(ctx(&format!("band {b} bypass is {bypass}"))));
            }
            let code = get(2000, "type")?;
            let kind = Kind::from_code(code)
                .ok_or_else(|| bad(ctx(&format!("band {b} filter type {code} is not carried"))))?;
            *band = Band {
                on: bypass == 0.0,
                kind,
                freq_hz: get(3000, "frequency")?,
                gain_db: get(4000, "gain")?,
                bw_oct: get(5000, "bandwidth")?,
            };
            if kind != Kind::LowPass && band.freq_hz >= nyquist {
                return Err(bad(ctx(&format!("band {b} frequency {} Hz is at or above Nyquist", band.freq_hz))));
            }
        }
    }
    let mut words = vec![0.0; Params::WORDS];
    p.to_words(&mut words);
    let p = Params::from_words(&words).ok_or_else(|| {
        bad("saved state is out of range (breakpoints not increasing in (0, 1), gains, bandwidths, or a band type change between neighbouring elements)")
    })?;
    if g.get(CURRENT_VOLUME).is_some() {
        diags.info(
            "vareq-volume-from-graph",
            "VarEQ",
            "the saved volume scalar is replaced by the graph's mvsc at run time",
        );
    }
    Ok(Some(p))
}
