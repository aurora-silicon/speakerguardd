// SPDX-License-Identifier: MIT
//! Raw sense recordings and offline replay.
//!
//! A recording is a text header line followed by one binary record per
//! complete sense period: the speakers' sample rate (u32), CLOCK_BOOTTIME
//! (u64 ns), the reduction the card applied to each speaker during the
//! period (f32 dB), and the period's words as the PCM delivered them (S16 or
//! S32, little endian).  Replay runs every governor on it as if each one had
//! written the card (their own gains decide the power they see), optionally
//! louder by `--gain-db`, and reports the reductions they asked for and how
//! warm their models got.  Below the amplifiers' clipping the sense data
//! scales linearly with the gain, so a quiet daytime recording shows what a
//! governor would do at higher levels without playing them.

use std::fmt::Write as _;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;
use std::time::Duration;

use crate::config::{Config, Governor, SenseFormat};
use crate::engine::{Engine, Feature, Start};
use crate::model::{self, SpeakerState};

const MAGIC: &str = "speakerguardd-record 1";

pub struct Recorder {
    out: BufWriter<File>,
    s16: bool,
    left: u64,
}

impl Recorder {
    pub fn create(path: &Path, cfg: &Config, max_bytes: u64) -> io::Result<Recorder> {
        let mut out = BufWriter::new(File::create(path)?);
        let s16 = cfg.globals.sense_format == SenseFormat::S16;
        writeln!(out, "{MAGIC} channels={} period={} s16={} speakers={}", cfg.globals.channels, cfg.globals.period, s16 as u8, cfg.speakers.len())?;
        Ok(Recorder { out, s16, left: max_bytes })
    }

    pub fn write(&mut self, rate: u32, boottime: Duration, applied: &[f64], words: &[i32]) -> io::Result<()> {
        let mut b = Vec::with_capacity(12 + 4 * applied.len() + 4 * words.len());
        b.extend_from_slice(&rate.to_le_bytes());
        b.extend_from_slice(&(boottime.as_nanos() as u64).to_le_bytes());
        for &a in applied {
            b.extend_from_slice(&(a as f32).to_le_bytes());
        }
        if self.s16 {
            for &w in words {
                b.extend_from_slice(&((w >> 16) as i16).to_le_bytes());
            }
        } else {
            for &w in words {
                b.extend_from_slice(&w.to_le_bytes());
            }
        }
        self.out.write_all(&b)?;
        self.left = self.left.saturating_sub(b.len() as u64);
        if self.left == 0 {
            self.out.flush()?;
        }
        Ok(())
    }

    pub fn full(&self) -> bool {
        self.left == 0
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        let _ = self.out.flush();
    }
}

pub struct Period {
    pub rate: u32,
    pub boottime: Duration,
    pub applied: Vec<f64>,
    pub words: Vec<i32>,
}

pub struct Reader {
    inp: BufReader<File>,
    channels: usize,
    period: usize,
    s16: bool,
    speakers: usize,
}

impl Reader {
    pub fn open(path: &Path, cfg: &Config) -> Result<Reader, Box<dyn std::error::Error>> {
        let mut inp = BufReader::new(File::open(path)?);
        let mut header = Vec::new();
        let mut byte = [0u8; 1];
        while inp.read(&mut byte)? == 1 && byte[0] != b'\n' {
            header.push(byte[0]);
            if header.len() > 200 {
                return Err("not a speakerguardd recording".into());
            }
        }
        let header = String::from_utf8(header)?;
        let rest = header.strip_prefix(MAGIC).ok_or("not a speakerguardd recording")?;
        let mut kv = std::collections::BTreeMap::new();
        for w in rest.split_whitespace() {
            if let Some((k, v)) = w.split_once('=') {
                kv.insert(k.to_string(), v.parse::<usize>()?);
            }
        }
        let get = |k: &str| kv.get(k).copied().ok_or_else(|| format!("recording header lacks {k}"));
        let r = Reader { inp, channels: get("channels")?, period: get("period")?, s16: get("s16")? == 1, speakers: get("speakers")? };
        if r.channels != cfg.globals.channels || r.period != cfg.globals.period || r.speakers != cfg.speakers.len() {
            return Err("recording geometry does not match the configuration".into());
        }
        Ok(r)
    }

    pub fn next(&mut self) -> io::Result<Option<Period>> {
        let mut head = [0u8; 12];
        match self.inp.read_exact(&mut head) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(e),
        }
        let rate = u32::from_le_bytes(head[0..4].try_into().unwrap());
        let ns = u64::from_le_bytes(head[4..12].try_into().unwrap());
        let mut applied = Vec::with_capacity(self.speakers);
        for _ in 0..self.speakers {
            let mut f = [0u8; 4];
            self.inp.read_exact(&mut f)?;
            applied.push(f32::from_le_bytes(f) as f64);
        }
        let n = self.channels * self.period;
        let mut words = vec![0i32; n];
        if self.s16 {
            let mut raw = vec![0u8; n * 2];
            self.inp.read_exact(&mut raw)?;
            for (w, c) in words.iter_mut().zip(raw.chunks_exact(2)) {
                *w = (i16::from_le_bytes([c[0], c[1]]) as i32) << 16;
            }
        } else {
            let mut raw = vec![0u8; n * 4];
            self.inp.read_exact(&mut raw)?;
            for (w, c) in words.iter_mut().zip(raw.chunks_exact(4)) {
                *w = i32::from_le_bytes(c.try_into().unwrap());
            }
        }
        Ok(Some(Period { rate, boottime: Duration::from_nanos(ns), applied, words }))
    }
}

/// Distribution of a governor's card-wide reduction over playing time.
#[derive(Default)]
struct Dist {
    samples: Vec<(f64, f64)>,
}

impl Dist {
    fn push(&mut self, dt: f64, r: f64) {
        self.samples.push((r, dt));
    }

    fn mean(&self) -> f64 {
        let t: f64 = self.samples.iter().map(|s| s.1).sum();
        if t > 0.0 { self.samples.iter().map(|s| s.0 * s.1).sum::<f64>() / t } else { 0.0 }
    }

    fn quantile(&self, q: f64) -> f64 {
        let mut v = self.samples.clone();
        v.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let t: f64 = v.iter().map(|s| s.1).sum();
        let mut acc = 0.0;
        for (r, dt) in v {
            acc += dt;
            if acc >= q * t {
                return r;
            }
        }
        0.0
    }
}

/// Run every governor on a recording; returns the summary text.
pub fn replay(path: &Path, cfg: &Config, gain_db: f64, csv: Option<&Path>) -> Result<String, Box<dyn std::error::Error>> {
    let mut rd = Reader::open(path, cfg)?;
    let g = &cfg.globals;
    let mut engine = Engine::new(cfg, Start::Cold);
    let mut legacy: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), g.t_ambient)).collect();
    let mut csv = match csv {
        Some(p) => Some(BufWriter::new(File::create(p)?)),
        None => None,
    };
    if let Some(c) = csv.as_mut() {
        let mut h = String::from("t,legacy,asahi,unified");
        for s in &cfg.speakers {
            let n = s.name.replace(' ', "_");
            let _ = write!(h, ",{n}_asahi_gain,{n}_asahi_coil,{n}_ff_coil,{n}_vi_coil,{n}_legacy_coil,{n}_p_ff,{n}_p_vi");
        }
        writeln!(c, "{h}")?;
    }
    let k_gain = 10f64.powf(gain_db / 10.0);
    let (mut d_leg, mut d_asa, mut d_uni) = (Dist::default(), Dist::default(), Dist::default());
    let mut max_leg: f64 = f64::MIN;
    let mut last_t: Option<Duration> = None;
    let mut periods = 0u64;
    let mut gaps = 0.0;
    let zeros = vec![0.0; cfg.speakers.len()];
    while let Some(p) = rd.next()? {
        if p.rate == 0 {
            continue;
        }
        let dt = g.period as f64 / p.rate as f64;
        // time between recorded periods beyond one period: the speakers
        // were closed (or the stream was reopened): cool
        if let Some(t0) = last_t {
            let gap = p.boottime.saturating_sub(t0).as_secs_f64() - dt;
            if gap > 0.05 {
                gaps += gap;
                engine.idle(gap);
                for s in legacy.iter_mut() {
                    s.step(0.0, gap, g.t_ambient);
                    s.govern(g);
                }
            }
        }
        last_t = Some(p.boottime);
        let post = engine.extractor.extract(cfg, &p.words, p.rate as f64);
        // the power before the card's gain, made louder by gain_db
        let pre: Vec<Feature> = post
            .iter()
            .zip(&p.applied)
            .map(|(f, &a)| {
                let k = 10f64.powf(a / 10.0) * k_gain;
                Feature { v_ms: f.v_ms * k, vi: f.vi.map(|v| v * k), i_ms: f.i_ms.map(|v| v * k), r_lf: f.r_lf }
            })
            .collect();
        let r_leg = model::reduction_db(&legacy);
        for (s, f) in legacy.iter_mut().zip(&pre) {
            let power = f.v_ms / s.spec.z_nominal * 10f64.powf(-r_leg / 10.0);
            s.v_rms = (f.v_ms * 10f64.powf(-r_leg / 10.0)).sqrt();
            s.step_at(power, dt, g.t_ambient, r_leg);
            s.govern(g);
        }
        engine.period(&pre, &zeros, dt, r_leg);
        let r_leg = model::reduction_db(&legacy);
        let ru = engine.reductions(Governor::Unified);
        let ra = engine.reductions(Governor::Asahi);
        let max = |v: &[f64]| v.iter().cloned().fold(0.0, f64::max);
        d_leg.push(dt, r_leg);
        d_uni.push(dt, max(&ru));
        if engine.has_asahi() {
            d_asa.push(dt, max(&ra));
        }
        for s in &legacy {
            max_leg = max_leg.max(s.t_coil);
        }
        periods += 1;
        if let Some(c) = csv.as_mut() {
            let mut line = format!("{:.4},{:.3},{:.3},{:.3}", p.boottime.as_secs_f64(), r_leg, max(&ra), max(&ru));
            for (k, s) in legacy.iter().enumerate() {
                let (ac, ff, vi) = engine.temps(k);
                let _ = write!(line, ",{:.4},{:.4},{:.4},{:.4},{:.4},{:.6},{:.6}", -ra[k], ac, ff, vi, s.t_coil, pre[k].v_ms / cfg.speakers[k].r_ff, pre[k].vi.unwrap_or(0.0));
            }
            writeln!(c, "{line}")?;
        }
    }
    let t = &engine.total;
    let mut o = String::new();
    let _ = writeln!(o, "replay {} at {:+.1} dB: {periods} periods, {:.1} s playing, {:.1} s gaps", path.display(), gain_db, t.playing_s, gaps);
    let _ = writeln!(o, "governor  mean_dB  p50_dB  p95_dB  max_dB");
    for (name, d) in [("legacy", &d_leg), ("asahi", &d_asa), ("unified", &d_uni)] {
        if d.samples.is_empty() {
            continue;
        }
        let _ = writeln!(o, "{name:<9} {:>7.2} {:>7.2} {:>7.2} {:>7.2}", d.mean(), d.quantile(0.5), d.quantile(0.95), d.quantile(1.0));
    }
    let _ = writeln!(o, "max coil (C): legacy model {:.1}, unified ff {:.1}, unified vi {}, asahi {}", max_leg, t.max_coil_ff, if t.max_coil_vi > f64::MIN { format!("{:.1}", t.max_coil_vi) } else { "-".into() }, if t.max_coil_asahi > f64::MIN { format!("{:.1}", t.max_coil_asahi) } else { "-".into() });
    let targets: Vec<String> = cfg.speakers.iter().map(|s| format!("{}={:.0}", s.name, s.t_limit - s.t_headroom - g.t_ctrl_headroom)).collect();
    let _ = writeln!(o, "unified target (C): {}", targets.join(" "));
    if !d_asa.samples.is_empty() {
        let _ = writeln!(o, "unified louder than asahi by {:.2} dB (mean), {:.2} dB (p95)", d_asa.mean() - d_uni.mean(), d_asa.quantile(0.95) - d_uni.quantile(0.95));
    }
    let _ = writeln!(o, "unified louder than legacy by {:.2} dB (mean), {:.2} dB (p95)", d_leg.mean() - d_uni.mean(), d_leg.quantile(0.95) - d_uni.quantile(0.95));
    Ok(o)
}
