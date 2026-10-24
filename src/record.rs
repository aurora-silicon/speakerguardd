// SPDX-License-Identifier: MIT
//! Recordings of what the guard saw, and their replay through the guard.
//!
//! A recording is a text header line and then tagged binary records, all
//! little endian, every one stamped with CLOCK_BOOTTIME (u64 ns):
//!
//! * `S` the start: the governor, --max-reduction, and the model state the
//!   guard held when the recording began (the state file's text).
//! * `R` a sample-rate read that differed from the last one recorded, or
//!   failed (-errno); `W` a change event that woke the idle wait; `O` a
//!   sense open and its result.
//! * `P` a complete sense period as the PCM delivered it: when it was read,
//!   the capture's backlog, the rate, whether it was a measurement, the
//!   reduction the card applied to each speaker over it, and its words.
//!   `T` the frames read at a stream's close.  `L` a lost or unusable
//!   period (short, stalled, a capture or position error).
//! * `A` the lock taken or handed to the kernel, with the reason; `H` the
//!   governing model's hard latch set or cleared; `X` a failed control
//!   write.  `Z` the end, when the recording stopped early.
//!
//! The recording starts with the model the live guard started from, written
//! as a clean stop's state at that instant, so that the replayed guard
//! resumes exactly that model (latched where the live one was latched).
//!
//! The replay runs the real guard on a simulated card driven by the
//! recording: the speakers open and close where the live daemon saw them
//! do so, the periods arrive when it read them, losses, failed rate reads
//! and failed opens recur at their times.  The program before the card's
//! gain is rebuilt from each period's words and the gain the card applied
//! (rounded up to the control's 0.5 dB step), and played through the gains
//! the replayed guard writes, rounded the same way, `--gain-db` louder if
//! asked, clipped at full scale.  The simulated
//! card keeps the kernel's rules: the lease in playing time, the one-shot
//! notice, the safe level whenever the lock is not held.
//!
//! What a recording cannot reproduce is marked unqualifiable: time lost to
//! the capture, bad sense data, failed rate reads or opens, periods played
//! louder than recorded where the recording cannot say what the program
//! was (the card held more than PRE_GAIN_MAX_DB, or the words clipped),
//! failed control writes and the daemon's own lateness (neither is
//! replayed), and anything after an early end.  A replay with any of them
//! refuses to certify.  Equivalence compares the replayed guard's lock and
//! latch transitions with the live ones.
//!
//! The replay also reports the replayed guard's account: every span is
//! measured, charged or cooled, and every uncovered interval is listed as
//! charged.  Lost frames and bad sense data are never measured; a replay
//! that measured any of them would refuse to certify.

use std::error::Error;
use std::fmt::Write as _;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::config::{Config, Governor, SenseFormat};
use crate::guard::{Accounted, Guard, Hw, Trace, KERNEL_LOCK_TIMEOUT, MUTE_DB};
/// The most card gain a replay rebuilds a program through (dB): past it the
/// recorded words are too quiet to say what the program was.
const PRE_GAIN_MAX_DB: f64 = 30.0;

const MAGIC: &str = "speakerguardd-record 2";

/// Loss kinds (`L`).
pub const LOSS_SHORT: u8 = 1;
pub const LOSS_STALL: u8 = 2;
pub const LOSS_CAPTURE: u8 = 3;
pub const LOSS_POSITION: u8 = 4;
/// Control failure kinds (`X`).
pub const FAIL_MIN: u8 = 1;
pub const FAIL_TARGET: u8 = 2;
/// Why a recording ended early (`Z`).
const END_FULL: u8 = 1;

/// A short fingerprint of a configuration, so that a replay can say it is
/// not running the configuration that was recorded.
pub fn fingerprint(cfg: &Config) -> u64 {
    // FNV-1a over the parsed configuration's debug form
    let text = format!("{:?}{:?}{:?}", cfg.globals, cfg.controls, cfg.speakers);
    text.bytes().fold(0xcbf29ce484222325u64, |h, b| (h ^ b as u64).wrapping_mul(0x100000001b3))
}

/// The control's step (the card rounds a reduction up to it): 0.5 dB.
fn rounded(db: f64) -> f64 {
    ((db * 2.0 - 1e-9).ceil() / 2.0).max(0.0)
}

pub struct Recorder {
    out: Box<dyn Write>,
    s16: bool,
    left: u64,
}

impl Recorder {
    pub fn create(path: &Path, cfg: &Config, events: bool, max_bytes: u64) -> io::Result<Recorder> {
        Recorder::to(Box::new(BufWriter::new(File::create(path)?)), cfg, events, max_bytes)
    }

    pub fn to(mut out: Box<dyn Write>, cfg: &Config, events: bool, max_bytes: u64) -> io::Result<Recorder> {
        let s16 = cfg.globals.sense_format == SenseFormat::S16;
        writeln!(
            out,
            "{MAGIC} channels={} period={} s16={} speakers={} events={} config={:016x}",
            cfg.globals.channels,
            cfg.globals.period,
            s16 as u8,
            cfg.speakers.len(),
            events as u8,
            fingerprint(cfg)
        )?;
        Ok(Recorder { out, s16, left: max_bytes })
    }

    fn put(&mut self, b: &[u8]) -> io::Result<()> {
        self.out.write_all(b)?;
        self.left = self.left.saturating_sub(b.len() as u64);
        Ok(())
    }

    /// The size limit was reached: the recording ends here.
    pub fn full(&self) -> bool {
        self.left == 0
    }

    fn head(tag: u8, t: u64) -> Vec<u8> {
        let mut b = Vec::with_capacity(64);
        b.push(tag);
        b.extend_from_slice(&t.to_le_bytes());
        b
    }

    fn words(&self, b: &mut Vec<u8>, words: &[i32]) {
        if self.s16 {
            for &w in words {
                b.extend_from_slice(&((w >> 16) as i16).to_le_bytes());
            }
        } else {
            for &w in words {
                b.extend_from_slice(&w.to_le_bytes());
            }
        }
    }

    pub fn start(&mut self, t: u64, governor: Governor, max_reduction: f64, state: &str) -> io::Result<()> {
        let mut b = Self::head(b'S', t);
        b.push(governor as u8);
        b.extend_from_slice(&max_reduction.to_le_bytes());
        b.extend_from_slice(&(state.len() as u32).to_le_bytes());
        b.extend_from_slice(state.as_bytes());
        self.put(&b)
    }

    pub fn rate(&mut self, t: u64, value: i64) -> io::Result<()> {
        let mut b = Self::head(b'R', t);
        b.extend_from_slice(&value.to_le_bytes());
        self.put(&b)
    }

    pub fn wake(&mut self, t: u64) -> io::Result<()> {
        self.put(&Self::head(b'W', t))
    }

    pub fn open(&mut self, t: u64, errno: i32) -> io::Result<()> {
        let mut b = Self::head(b'O', t);
        b.extend_from_slice(&errno.to_le_bytes());
        self.put(&b)
    }

    pub fn period(&mut self, t: u64, queued: u64, rate: u32, measured: bool, applied: &[f64], words: &[i32]) -> io::Result<()> {
        let mut b = Self::head(b'P', t);
        b.extend_from_slice(&queued.to_le_bytes());
        b.extend_from_slice(&rate.to_le_bytes());
        b.push(measured as u8);
        for &a in applied {
            b.extend_from_slice(&(a.min(1e6) as f32).to_le_bytes());
        }
        self.words(&mut b, words);
        self.put(&b)
    }

    pub fn tail(&mut self, t: u64, queued: u64, frames: usize, words: &[i32]) -> io::Result<()> {
        let mut b = Self::head(b'T', t);
        b.extend_from_slice(&queued.to_le_bytes());
        b.extend_from_slice(&(frames as u32).to_le_bytes());
        self.words(&mut b, words);
        self.put(&b)
    }

    pub fn loss(&mut self, t: u64, kind: u8, errno: i32) -> io::Result<()> {
        let mut b = Self::head(b'L', t);
        b.push(kind);
        b.extend_from_slice(&errno.to_le_bytes());
        self.put(&b)
    }

    pub fn permit(&mut self, t: u64, armed: bool, why: &str) -> io::Result<()> {
        let mut b = Self::head(b'A', t);
        b.push(armed as u8);
        let why = &why.as_bytes()[..why.len().min(200)];
        b.extend_from_slice(&(why.len() as u16).to_le_bytes());
        b.extend_from_slice(why);
        self.put(&b)
    }

    pub fn latch(&mut self, t: u64, set: bool) -> io::Result<()> {
        let mut b = Self::head(b'H', t);
        b.push(set as u8);
        self.put(&b)
    }

    pub fn control_failure(&mut self, t: u64, kind: u8) -> io::Result<()> {
        let mut b = Self::head(b'X', t);
        b.push(kind);
        self.put(&b)
    }

    pub fn end(&mut self, t: u64) -> io::Result<()> {
        let mut b = Self::head(b'Z', t);
        b.push(END_FULL);
        self.out.write_all(&b)?;
        self.out.flush()
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        let _ = self.out.flush();
    }
}

/// A complete period's place in the file.
struct Period {
    t: Duration,
    queued: u64,
    rate: u32,
    measured: bool,
    applied: Vec<f64>,
    offset: u64,
}

struct Tail {
    t: Duration,
    queued: u64,
    frames: usize,
    offset: u64,
}

/// What a recording holds, without the words (they stay in the file).
struct Timeline {
    channels: usize,
    period: usize,
    s16: bool,
    events: bool,
    config: u64,
    start: Duration,
    governor: Governor,
    max_reduction: f64,
    state: String,
    /// Sample-rate observations: (when, rate or -errno), the opening ones
    /// moved back to the change event that announced them.
    rates: Vec<(Duration, i64)>,
    opens: Vec<(Duration, i32)>,
    periods: Vec<Period>,
    tails: Vec<Tail>,
    losses: Vec<(Duration, u8, i32)>,
    permits: Vec<(Duration, bool, String)>,
    latches: Vec<(Duration, bool)>,
    failures: Vec<(Duration, u8)>,
    truncated: Option<Duration>,
    last: Duration,
}

fn read_u8(r: &mut impl Read) -> io::Result<u8> {
    let mut b = [0u8; 1];
    r.read_exact(&mut b)?;
    Ok(b[0])
}

fn read_u16(r: &mut impl Read) -> io::Result<u16> {
    let mut b = [0u8; 2];
    r.read_exact(&mut b)?;
    Ok(u16::from_le_bytes(b))
}

fn read_u32(r: &mut impl Read) -> io::Result<u32> {
    let mut b = [0u8; 4];
    r.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}

fn read_u64(r: &mut impl Read) -> io::Result<u64> {
    let mut b = [0u8; 8];
    r.read_exact(&mut b)?;
    Ok(u64::from_le_bytes(b))
}

fn read_f64(r: &mut impl Read) -> io::Result<f64> {
    Ok(f64::from_bits(read_u64(r)?))
}

fn governor_of(v: u8) -> Option<Governor> {
    [Governor::Legacy, Governor::Asahi, Governor::Unified].into_iter().find(|g| *g as u8 == v)
}

impl Timeline {
    fn read(path: &Path, cfg: &Config) -> Result<Timeline, Box<dyn Error>> {
        let mut inp = BufReader::new(File::open(path)?);
        let mut header = Vec::new();
        loop {
            let b = read_u8(&mut inp).map_err(|_| "not a speakerguardd recording")?;
            if b == b'\n' {
                break;
            }
            header.push(b);
            if header.len() > 300 {
                return Err("not a speakerguardd recording".into());
            }
        }
        let header = String::from_utf8(header)?;
        let rest = header.strip_prefix(MAGIC).ok_or("not a speakerguardd recording (or an older format)")?;
        let mut kv = std::collections::BTreeMap::new();
        for w in rest.split_whitespace() {
            if let Some((k, v)) = w.split_once('=') {
                kv.insert(k.to_string(), v.to_string());
            }
        }
        let get = |k: &str| kv.get(k).cloned().ok_or_else(|| format!("recording header lacks {k}"));
        let num = |k: &str| -> Result<usize, Box<dyn Error>> { Ok(get(k)?.parse()?) };
        let (channels, period, speakers) = (num("channels")?, num("period")?, num("speakers")?);
        if channels != cfg.globals.channels || period != cfg.globals.period || speakers != cfg.speakers.len() {
            return Err("recording geometry does not match the configuration".into());
        }
        let s16 = num("s16")? == 1;
        let events = num("events")? == 1;
        let config = u64::from_str_radix(&get("config")?, 16)?;
        let word = if s16 { 2 } else { 4 };
        let mut offset = header.len() as u64 + 1;
        let mut tl = Timeline {
            channels,
            period,
            s16,
            events,
            config,
            start: Duration::ZERO,
            governor: Governor::Legacy,
            max_reduction: 20.0,
            state: String::new(),
            rates: Vec::new(),
            opens: Vec::new(),
            periods: Vec::new(),
            tails: Vec::new(),
            losses: Vec::new(),
            permits: Vec::new(),
            latches: Vec::new(),
            failures: Vec::new(),
            truncated: None,
            last: Duration::ZERO,
        };
        let mut wakes = Vec::new();
        let mut started = false;
        loop {
            let tag = match read_u8(&mut inp) {
                Ok(t) => t,
                Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => break,
                Err(e) => return Err(e.into()),
            };
            let t = match read_u64(&mut inp) {
                Ok(t) => Duration::from_nanos(t),
                // a record cut off by the end of the file: the recording ends
                Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => {
                    tl.truncated.get_or_insert(tl.last);
                    break;
                }
                Err(e) => return Err(e.into()),
            };
            let mut size = 9u64;
            let body: io::Result<()> = (|| {
                match tag {
                    b'S' => {
                        tl.start = t;
                        tl.governor = governor_of(read_u8(&mut inp)?).ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "governor"))?;
                        tl.max_reduction = read_f64(&mut inp)?;
                        let n = read_u32(&mut inp)? as usize;
                        let mut s = vec![0u8; n];
                        inp.read_exact(&mut s)?;
                        tl.state = String::from_utf8(s).map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "state"))?;
                        size += 1 + 8 + 4 + n as u64;
                        started = true;
                    }
                    b'R' => {
                        tl.rates.push((t, read_u64(&mut inp)? as i64));
                        size += 8;
                    }
                    b'W' => wakes.push(t),
                    b'O' => {
                        tl.opens.push((t, read_u32(&mut inp)? as i32));
                        size += 4;
                    }
                    b'P' => {
                        let queued = read_u64(&mut inp)?;
                        let rate = read_u32(&mut inp)?;
                        let measured = read_u8(&mut inp)? == 1;
                        let mut applied = Vec::with_capacity(speakers);
                        for _ in 0..speakers {
                            applied.push(f32::from_bits(read_u32(&mut inp)?) as f64);
                        }
                        size += 8 + 4 + 1 + 4 * speakers as u64;
                        let n = (channels * period * word) as u64;
                        tl.periods.push(Period { t, queued, rate, measured, applied, offset: offset + size });
                        inp.seek_relative(n as i64)?;
                        size += n;
                    }
                    b'T' => {
                        let queued = read_u64(&mut inp)?;
                        let frames = read_u32(&mut inp)? as usize;
                        size += 12;
                        if frames > period {
                            return Err(io::Error::new(io::ErrorKind::InvalidData, "tail longer than a period"));
                        }
                        tl.tails.push(Tail { t, queued, frames, offset: offset + size });
                        let n = (channels * frames * word) as u64;
                        inp.seek_relative(n as i64)?;
                        size += n;
                    }
                    b'L' => {
                        let kind = read_u8(&mut inp)?;
                        tl.losses.push((t, kind, read_u32(&mut inp)? as i32));
                        size += 5;
                    }
                    b'A' => {
                        let armed = read_u8(&mut inp)? == 1;
                        let n = read_u16(&mut inp)? as usize;
                        let mut s = vec![0u8; n];
                        inp.read_exact(&mut s)?;
                        tl.permits.push((t, armed, String::from_utf8_lossy(&s).into_owned()));
                        size += 3 + n as u64;
                    }
                    b'H' => {
                        tl.latches.push((t, read_u8(&mut inp)? == 1));
                        size += 1;
                    }
                    b'X' => {
                        tl.failures.push((t, read_u8(&mut inp)?));
                        size += 1;
                    }
                    b'Z' => {
                        let _ = read_u8(&mut inp)?;
                        tl.truncated = Some(t);
                        size += 1;
                    }
                    _ => return Err(io::Error::new(io::ErrorKind::InvalidData, format!("unknown record {tag:#x}"))),
                }
                Ok(())
            })();
            match body {
                Ok(()) => {}
                Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => {
                    // the last record was cut off: everything after it is unknown
                    tl.periods.retain(|p| p.t < t);
                    tl.tails.retain(|p| p.t < t);
                    tl.truncated.get_or_insert(t);
                    break;
                }
                Err(e) => return Err(e.into()),
            }
            offset += size;
            tl.last = tl.last.max(t);
            if tl.truncated.is_some() {
                break;
            }
        }
        if !started {
            return Err("the recording has no start record".into());
        }
        // an opening seen at a change event started there, not at the read
        let mut prev = tl.start;
        for i in 0..tl.rates.len() {
            let (t, v) = tl.rates[i];
            let was = if i == 0 { 0 } else { tl.rates[i - 1].1 };
            if v > 0 && was == 0 {
                if let Some(&w) = wakes.iter().rev().find(|&&w| w <= t && w >= prev && t - w <= Duration::from_millis(150)) {
                    tl.rates[i].0 = w;
                }
            }
            prev = t;
        }
        Ok(tl)
    }

    fn dt(&self, rate: u32) -> Duration {
        Duration::from_secs_f64(self.period as f64 / rate.max(1) as f64)
    }

    /// The end of a period's frames, as the live daemon placed them.
    fn end_of(&self, p: &Period) -> Duration {
        p.t.saturating_sub(Duration::from_secs_f64(p.queued as f64 / p.rate.max(1) as f64))
    }

    /// The last rate observation at or before `t` (0 before the first).
    fn rate_at(&self, t: Duration) -> i64 {
        let i = self.rates.partition_point(|r| r.0 <= t);
        if i == 0 { 0 } else { self.rates[i - 1].1 }
    }

    /// The first change of the rate in (from, to].
    fn change_in(&self, from: Duration, to: Duration) -> Option<Duration> {
        let mut was = self.rate_at(from) > 0;
        for &(t, v) in &self.rates {
            if t > from && t <= to {
                if (v > 0) != was || v < 0 {
                    return Some(t);
                }
                was = v > 0;
            }
        }
        None
    }

    /// Intervals the recording cannot reproduce, with what they are.
    fn ambiguous(&self) -> Vec<(Duration, Duration, String)> {
        let mut out = Vec::new();
        for &(t, kind, errno) in &self.losses {
            let what = match kind {
                LOSS_SHORT => "a short or overrun period".to_string(),
                LOSS_STALL => "a stalled capture".to_string(),
                LOSS_CAPTURE => format!("a capture error ({errno})"),
                LOSS_POSITION => format!("a position error ({errno})"),
                _ => "a lost period".to_string(),
            };
            out.push((t.saturating_sub(self.dt(48000)), t, what));
        }
        let mut fail: Option<Duration> = None;
        for &(t, v) in &self.rates {
            match (v < 0, fail) {
                (true, None) => fail = Some(t),
                (false, Some(f)) => {
                    out.push((f, t, "failed sample-rate reads".into()));
                    fail = None;
                }
                _ => {}
            }
        }
        if let Some(f) = fail {
            out.push((f, self.last, "failed sample-rate reads".into()));
        }
        for (i, &(t, e)) in self.opens.iter().enumerate() {
            if e != 0 {
                let next = self.opens[i + 1..].iter().find(|o| o.1 == 0).map_or(self.last, |o| o.0);
                out.push((t, next, format!("failed sense opens ({e})")));
            }
        }
        let mut prev_end: Option<Duration> = None;
        for p in &self.periods {
            let end = self.end_of(p);
            let start = end.saturating_sub(self.dt(p.rate));
            if !p.measured {
                out.push((start, end, "bad sense data".into()));
            }
            if let Some(pe) = prev_end {
                // a gap between periods' frames while the speakers stayed open
                if start > pe + Duration::from_millis(2) && self.change_in(pe, start).is_none() {
                    out.push((pe, start, "frames the capture lost".into()));
                }
            }
            prev_end = Some(end);
        }
        for &(t, kind) in &self.failures {
            let what = match kind {
                FAIL_MIN => "a failed minimum write",
                _ => "a failed volume write",
            };
            out.push((t.saturating_sub(KERNEL_LOCK_TIMEOUT), t + KERNEL_LOCK_TIMEOUT, format!("{what} (not replayed)")));
        }
        for (t, armed, why) in &self.permits {
            if !armed && why.contains("lapsed") {
                // the live daemon was late: its own timing is not replayed
                out.push((t.saturating_sub(2 * KERNEL_LOCK_TIMEOUT), *t, "the daemon was late (not replayed)".into()));
            }
        }
        if let Some(t) = self.truncated {
            out.push((t, t.max(self.last), "the recording stopped here".into()));
        }
        out.sort_by_key(|a| a.0);
        out
    }
}

/// The card as the kernel runs it, for the replay: per-speaker controls,
/// the volume lock and its lease in playing time.
struct SimCard {
    safe: f64,
    per: Vec<f64>,
    unlocked: bool,
    lease_left: Duration,
    notice: bool,
}

impl SimCard {
    fn effective(&self, k: usize) -> f64 {
        let r = rounded(self.per[k]);
        if self.unlocked { r } else { r.max(self.safe) }
    }
}

/// The replay's machine: the recording's environment and a simulated card.
pub struct ReplayHw {
    tl: Timeline,
    file: File,
    t0: Instant,
    /// CLOCK_BOOTTIME now.
    bt: Duration,
    gain_db: f64,
    card: SimCard,
    sense_open: bool,
    next_period: usize,
    next_loss: usize,
    used_tail: usize,
    position_error: Option<i32>,
    queued: u64,
    events_seen_to: Duration,
    lease_t: Duration,
    /// The speaker each sense channel belongs to.
    chan_speaker: Vec<Option<usize>>,
    pub clipped_periods: u32,
    /// Periods replayed louder than recorded where the recording cannot
    /// say what was played: it clipped there, or the card held more than
    /// PRE_GAIN_MAX_DB.  Unqualifiable.
    pub clip_ambiguous: Vec<(Duration, Duration, &'static str)>,
    pub delivered: u32,
}

impl ReplayHw {
    fn new(tl: Timeline, cfg: &Config, path: &Path, gain_db: f64) -> io::Result<ReplayHw> {
        let mut chan_speaker = vec![None; cfg.globals.channels];
        for (k, s) in cfg.speakers.iter().enumerate() {
            chan_speaker[s.vs_chan] = Some(k);
            if let Some(ic) = s.is_chan {
                chan_speaker[ic] = Some(k);
            }
        }
        let n = cfg.speakers.len();
        Ok(ReplayHw {
            bt: tl.start,
            lease_t: tl.start,
            events_seen_to: tl.start,
            tl,
            file: File::open(path)?,
            t0: Instant::now(),
            gain_db,
            card: SimCard { safe: cfg.globals.safe_reduction, per: vec![cfg.globals.safe_reduction; n], unlocked: false, lease_left: Duration::ZERO, notice: false },
            sense_open: false,
            next_period: 0,
            next_loss: 0,
            used_tail: 0,
            position_error: None,
            queued: 0,
            chan_speaker,
            clipped_periods: 0,
            clip_ambiguous: Vec::new(),
            delivered: 0,
        })
    }

    pub fn boottime_now(&self) -> Duration {
        self.bt
    }

    fn playing(&self, t: Duration) -> bool {
        // a failed read counts as playing: the kernel's lease may run
        self.tl.rate_at(t) != 0
    }

    pub fn playing_now(&self) -> bool {
        self.playing(self.bt)
    }

    /// The gain the card applies to speaker `k` now.
    pub fn effective(&self, k: usize) -> f64 {
        self.card.effective(k)
    }

    /// Run the lease down by the playing time since it was last looked at.
    fn lapse(&mut self) {
        let step = Duration::from_micros(500);
        let mut a = self.lease_t;
        while a < self.bt && !self.card.lease_left.is_zero() {
            let b = (a + step).min(self.bt);
            if self.playing(a) {
                self.card.lease_left = self.card.lease_left.saturating_sub(b - a);
                if self.card.lease_left.is_zero() && self.card.unlocked {
                    self.card.unlocked = false;
                    self.card.notice = true;
                }
            }
            a = b;
        }
        self.lease_t = self.bt;
    }

    fn advance_to(&mut self, t: Duration) {
        if t > self.bt {
            self.bt = t;
        }
        self.lapse();
    }

    /// Words from the file at `offset`, `frames` frames of them, rebuilt
    /// before the recorded gain and played through the card's gains now.
    fn load(&mut self, offset: u64, frames: usize, applied: &[f64], start: Duration, end: Duration, words: &mut [i32]) -> io::Result<()> {
        let ch = self.tl.channels;
        let n = frames * ch;
        let word = if self.tl.s16 { 2 } else { 4 };
        let mut raw = vec![0u8; n * word];
        self.file.seek(SeekFrom::Start(offset))?;
        self.file.read_exact(&mut raw)?;
        let (lo, hi) = if self.tl.s16 { ((i16::MIN as i64) << 16, (i16::MAX as i64) << 16) } else { (i32::MIN as i64, i32::MAX as i64) };
        let mut unknown = false;
        let factor: Vec<f64> = (0..ch)
            .map(|c| match self.chan_speaker[c] {
                Some(k) => {
                    let recorded = rounded(applied.get(k).copied().unwrap_or(0.0));
                    let mut louder = recorded - self.card.effective(k) + self.gain_db;
                    if recorded > PRE_GAIN_MAX_DB && louder > 0.0 {
                        // under a card that held this much the words are
                        // noise: the program played louder is unknown
                        unknown = true;
                        louder = louder.min(PRE_GAIN_MAX_DB - self.card.effective(k) + self.gain_db).max(0.0);
                    }
                    10f64.powf(louder / 20.0)
                }
                None => 1.0,
            })
            .collect();
        if unknown {
            self.clip_ambiguous.push((start, end, "the card held more than 30 dB there, and the replay plays it louder"));
        }
        let (mut clipped, mut ambiguous) = (false, false);
        for i in 0..n {
            let w = if self.tl.s16 { (i16::from_le_bytes([raw[2 * i], raw[2 * i + 1]]) as i64) << 16 } else { i32::from_le_bytes(raw[4 * i..4 * i + 4].try_into().unwrap()) as i64 };
            let f = factor[i % ch];
            let x = (w as f64 * f).round();
            if x > hi as f64 || x < lo as f64 {
                clipped = true;
            }
            if f > 1.0 && (w >= hi || w <= lo) {
                // full scale in the recording: louder than that is unknown
                ambiguous = true;
            }
            words[i] = x.clamp(lo as f64, hi as f64) as i32;
        }
        if clipped {
            self.clipped_periods += 1;
        }
        if ambiguous {
            self.clip_ambiguous.push((start, end, "clipped in the recording, replayed louder"));
        }
        Ok(())
    }
}

impl Hw for ReplayHw {
    fn now(&self) -> Instant {
        self.t0 + self.bt.saturating_sub(self.tl.start)
    }
    fn boottime(&self) -> Duration {
        self.bt
    }
    fn sleep(&mut self, d: Duration) {
        let t = self.bt + d;
        self.advance_to(t);
    }
    fn take_lock(&mut self) -> io::Result<()> {
        Ok(())
    }
    fn ping(&mut self) -> io::Result<()> {
        self.lapse();
        if self.card.notice {
            self.card.notice = false;
            return Err(io::Error::from_raw_os_error(libc::ETIMEDOUT));
        }
        self.card.unlocked = true;
        self.card.lease_left = KERNEL_LOCK_TIMEOUT;
        Ok(())
    }
    fn set_reduction(&mut self, db: f64) -> io::Result<f64> {
        let n = self.card.per.len();
        Ok(self.set_reductions(&vec![db; n])?.into_iter().fold(0.0, f64::max))
    }
    fn set_reductions(&mut self, db: &[f64]) -> io::Result<Vec<f64>> {
        self.lapse();
        if !self.card.unlocked && db.iter().any(|&d| d < self.card.safe) {
            return Err(io::Error::from_raw_os_error(libc::EINVAL));
        }
        for (p, &d) in self.card.per.iter_mut().zip(db) {
            *p = d;
        }
        // the card rounds each up to its 0.5 dB step
        Ok(db.iter().map(|&d| rounded(d)).collect())
    }
    fn step_db(&self) -> f64 {
        0.5
    }
    fn set_min(&mut self) -> io::Result<()> {
        self.lapse();
        for p in self.card.per.iter_mut() {
            *p = MUTE_DB;
        }
        Ok(())
    }
    fn relinquish(&mut self) -> io::Result<()> {
        self.lapse();
        self.card.unlocked = false;
        self.card.lease_left = Duration::ZERO;
        self.card.notice = false;
        Ok(())
    }
    fn sample_rate(&mut self) -> io::Result<i64> {
        self.lapse();
        match self.tl.rate_at(self.bt) {
            v if v < 0 => Err(io::Error::from_raw_os_error(((-v) as i32).max(1))),
            v => Ok(v),
        }
    }
    fn open_sense(&mut self, _rate: u32) -> io::Result<()> {
        self.lapse();
        // the live open nearest before now decides
        let i = self.tl.opens.partition_point(|o| o.0 <= self.bt + Duration::from_millis(10));
        if i > 0 && self.tl.opens[i - 1].1 != 0 && self.bt.saturating_sub(self.tl.opens[i - 1].0) < Duration::from_millis(500) {
            return Err(io::Error::from_raw_os_error(self.tl.opens[i - 1].1));
        }
        // frames captured before this capture existed are not in it
        let open = self.bt.saturating_sub(Duration::from_millis(20));
        while self.next_period < self.tl.periods.len() {
            let p = &self.tl.periods[self.next_period];
            if self.tl.end_of(p).saturating_sub(self.tl.dt(p.rate)) >= open {
                break;
            }
            self.next_period += 1;
        }
        while self.next_loss < self.tl.losses.len() && self.tl.losses[self.next_loss].0 < self.bt {
            self.next_loss += 1;
        }
        self.sense_open = true;
        Ok(())
    }
    fn close_sense(&mut self) {
        self.sense_open = false;
    }
    fn read_period(&mut self, words: &mut [i32]) -> io::Result<Option<bool>> {
        let wait_end = self.bt + Duration::from_millis(50);
        let next_p = self.tl.periods.get(self.next_period).map(|p| p.t);
        let next_l = self.tl.losses.get(self.next_loss).map(|l| l.0);
        let next = match (next_p, next_l) {
            (Some(p), Some(l)) => Some(p.min(l)),
            (a, b) => a.or(b),
        };
        // a sample-rate change ends the wait early
        if self.tl.events {
            if let Some(c) = self.tl.change_in(self.bt, wait_end) {
                if next.is_none_or(|n| c < n) {
                    self.advance_to(c + Duration::from_micros(200));
                    return Ok(None);
                }
            }
        }
        let Some(te) = next.filter(|&t| t <= wait_end) else {
            self.advance_to(wait_end);
            return Ok(None);
        };
        self.advance_to(te);
        if next_l == Some(te) && next_p != Some(te) {
            let (_, kind, errno) = self.tl.losses[self.next_loss];
            self.next_loss += 1;
            return match kind {
                LOSS_SHORT => Ok(Some(false)),
                LOSS_CAPTURE => Err(io::Error::from_raw_os_error(errno.max(1))),
                LOSS_POSITION => {
                    self.position_error = Some(errno.max(1));
                    words.iter_mut().for_each(|w| *w = 0);
                    Ok(Some(true))
                }
                // a stall is only the absence of periods
                _ => Ok(None),
            };
        }
        let i = self.next_period;
        self.next_period += 1;
        let (offset, applied, t, queued, rate) = {
            let p = &self.tl.periods[i];
            (p.offset, p.applied.clone(), p.t, p.queued, p.rate)
        };
        let end = self.tl.end_of(&self.tl.periods[i]);
        let dt = self.tl.dt(rate);
        self.load(offset, self.tl.period, &applied, end.saturating_sub(dt), end, words)?;
        self.queued = queued + (self.bt.saturating_sub(t).as_secs_f64() * rate as f64) as u64;
        self.delivered += 1;
        Ok(Some(true))
    }
    fn queued(&mut self) -> io::Result<u64> {
        if let Some(e) = self.position_error.take() {
            return Err(io::Error::from_raw_os_error(e));
        }
        Ok(self.queued)
    }
    fn read_tail(&mut self, words: &mut [i32]) -> io::Result<usize> {
        if !self.sense_open {
            return Ok(0);
        }
        let near = |t: Duration, bt: Duration| t + Duration::from_millis(300) >= bt && t <= bt + Duration::from_millis(50);
        let Some(i) = (self.used_tail..self.tl.tails.len()).find(|&i| near(self.tl.tails[i].t, self.bt)) else { return Ok(0) };
        self.used_tail = i + 1;
        let (offset, frames, t, queued) = {
            let x = &self.tl.tails[i];
            (x.offset, x.frames, x.t, x.queued)
        };
        // the tail was read under the gain of the stream's last period
        let applied = self.tl.periods[..self.next_period].last().map_or_else(|| vec![0.0; self.card.per.len()], |p| p.applied.clone());
        self.load(offset, frames, &applied, t, t, words)?;
        self.queued = queued;
        Ok(frames)
    }
    fn wait_rate_change(&mut self, timeout: Duration) -> bool {
        let to = self.bt + timeout;
        if self.tl.events {
            if let Some(c) = self.tl.change_in(self.bt, to) {
                self.advance_to(c + Duration::from_micros(200));
                return true;
            }
        }
        self.advance_to(to);
        false
    }
    fn rate_events(&mut self) -> bool {
        let changed = self.tl.events && self.tl.change_in(self.events_seen_to, self.bt).is_some();
        self.events_seen_to = self.bt;
        changed
    }
    fn events_available(&self) -> bool {
        self.tl.events
    }
}

/// What a replay asks for.
pub struct Replay {
    pub gain_db: f64,
    /// Default: the recorded governor and --max-reduction.
    pub governor: Option<Governor>,
    pub max_reduction: Option<f64>,
    pub csv: Option<PathBuf>,
}

/// A replay's outcome.
pub struct Outcome {
    pub text: String,
    pub certified: bool,
}

/// Distribution of what the card applied over playing time.
#[derive(Default)]
struct Dist {
    samples: Vec<(f64, f64)>,
}

impl Dist {
    fn push(&mut self, dt: f64, r: f64) {
        if dt > 0.0 {
            self.samples.push((r, dt));
        }
    }

    fn quantile(&self, q: f64) -> f64 {
        let mut v = self.samples.clone();
        v.sort_by(|a, b| a.0.total_cmp(&b.0));
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

    fn mean(&self) -> f64 {
        let t: f64 = self.samples.iter().map(|s| s.1).sum();
        if t > 0.0 { self.samples.iter().map(|s| s.0 * s.1).sum::<f64>() / t } else { 0.0 }
    }
}

/// Match two transition lists within `tol`: (matched, only in a, only in b).
fn compare<T: PartialEq + Copy>(a: &[(Duration, T)], b: &[(Duration, T)], tol: Duration) -> (usize, usize, usize) {
    let mut used = vec![false; b.len()];
    let mut matched = 0;
    for &(t, v) in a {
        let hit = (0..b.len()).filter(|&j| !used[j] && b[j].1 == v && (b[j].0.max(t) - b[j].0.min(t)) <= tol).min_by_key(|&j| b[j].0.max(t) - b[j].0.min(t));
        if let Some(j) = hit {
            used[j] = true;
            matched += 1;
        }
    }
    (matched, a.len() - matched, used.iter().filter(|u| !**u).count())
}

/// Replay a recording through the guard.
pub fn replay(path: &Path, cfg: &Config, opts: &Replay) -> Result<Outcome, Box<dyn Error>> {
    let tl = Timeline::read(path, cfg)?;
    let mut cfg = cfg.clone();
    // offline: no card is touched
    cfg.allow_unified = true;
    cfg.allow_unqualified = true;
    cfg.globals.governor = opts.governor.unwrap_or(tl.governor);
    let max_reduction = opts.max_reduction.unwrap_or(tl.max_reduction);
    let same = cfg.globals.governor == tl.governor && opts.gain_db == 0.0 && max_reduction == tl.max_reduction && fingerprint(&cfg) == tl.config;
    let mut ambiguous = tl.ambiguous();
    let live_permits: Vec<(Duration, bool)> = tl.permits.iter().map(|p| (p.0, p.1)).collect();
    let live_latches = tl.latches.clone();
    let (start, last, config_matches) = (tl.start, tl.last, fingerprint(&cfg) == tl.config);
    let initial = tl.state.clone();
    let recorded_gov = tl.governor;
    // the recorded model state, as the replayed guard's state file
    static RUN: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let run = RUN.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("speakerguardd-replay-{}-{run}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir)?;
    let state = dir.join("state");
    std::fs::write(&state, &initial)?;
    // what the replayed guard must never measure: the recording's lost
    // frames and bad sense data
    let never_measured: Vec<(Duration, Duration)> = ambiguous.iter().filter(|a| a.2 == "frames the capture lost" || a.2 == "bad sense data").map(|a| (a.0, a.1)).collect();
    let mut hw = ReplayHw::new(tl, &cfg, path, opts.gain_db)?;
    let mut g = Guard::start_with(&mut hw, &cfg, max_reduction, Some(state), &[])?;
    g.start_trace(&hw);
    let mut csv = match &opts.csv {
        Some(p) => Some(BufWriter::new(File::create(p)?)),
        None => None,
    };
    if let Some(c) = csv.as_mut() {
        let mut h = String::from("t,armed,card_max_db");
        for s in &cfg.speakers {
            let n = crate::engine::state_name(&s.name);
            let _ = write!(h, ",{n}_card_db,{n}_legacy_coil,{n}_asahi_coil,{n}_ff_coil,{n}_vi_coil");
        }
        writeln!(c, "{h}")?;
    }
    let n = cfg.speakers.len();
    let mut card = Dist::default();
    let (mut playing_s, mut kernel_s) = (0.0, 0.0);
    let mut hottest = vec![(f64::MIN, f64::MIN, f64::MIN, f64::MIN); n];
    let mut fatal = None;
    // the replay ends where the recording does: after its last record the
    // live daemon's view is not known
    let end = last;
    let mut delivered = 0;
    while hw.boottime_now() < end {
        let before = hw.boottime_now();
        let was_playing = hw.playing_now();
        if let Err(e) = g.tick(&mut hw) {
            fatal = Some(format!("at {:.3} s: {e}", hw.boottime_now().saturating_sub(start).as_secs_f64()));
            break;
        }
        let dt = hw.boottime_now().saturating_sub(before).as_secs_f64();
        if was_playing {
            playing_s += dt;
            let applied = (0..n).map(|k| hw.effective(k)).fold(0.0, f64::max);
            card.push(dt, applied);
            if applied >= cfg.globals.safe_reduction {
                kernel_s += dt;
            }
        }
        for k in 0..n {
            let (a, ff, vi) = g.engine.as_ref().map_or((f64::NAN, f64::NAN, f64::NAN), |e| e.temps(k));
            let h = &mut hottest[k];
            h.0 = h.0.max(g.speakers[k].t_coil);
            if a.is_finite() {
                h.1 = h.1.max(a);
            }
            if ff.is_finite() {
                h.2 = h.2.max(ff);
            }
            if vi.is_finite() {
                h.3 = h.3.max(vi);
            }
        }
        if hw.delivered != delivered {
            delivered = hw.delivered;
            if let Some(c) = csv.as_mut() {
                let mut line = format!("{:.4},{},{:.2}", hw.boottime_now().as_secs_f64(), g.armed_now() as u8, (0..n).map(|k| hw.effective(k)).fold(0.0, f64::max));
                for k in 0..n {
                    let (a, ff, vi) = g.engine.as_ref().map_or((f64::NAN, f64::NAN, f64::NAN), |e| e.temps(k));
                    let _ = write!(line, ",{:.2},{:.3},{a:.3},{ff:.3},{vi:.3}", hw.effective(k), g.speakers[k].t_coil);
                }
                writeln!(c, "{line}")?;
            }
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
    for &(a, b, why) in &hw.clip_ambiguous {
        ambiguous.push((a, b, why.into()));
    }
    ambiguous.sort_by_key(|a| a.0);
    let trace = g.trace.take().unwrap_or_default();
    let rep_permits: Vec<(Duration, bool)> = trace.iter().filter_map(|(t, e)| if let Trace::Permit(a, _) = e { Some((*t, *a)) } else { None }).collect();
    let rep_latches: Vec<(Duration, bool)> = trace.iter().filter_map(|(t, e)| if let Trace::Latch(s) = e { Some((*t, *s)) } else { None }).collect();
    let handoffs: Vec<&String> = trace.iter().filter_map(|(_, e)| if let Trace::Permit(false, why) = e { Some(why) } else { None }).collect();
    let spans: Vec<(Duration, Duration, Accounted)> = trace.iter().filter_map(|(_, e)| if let Trace::Account(a, b, how) = e { Some((*a, *b, *how)) } else { None }).collect();
    let total_of = |how: Accounted| spans.iter().filter(|s| s.2 == how).map(|s| (s.1 - s.0).as_secs_f64()).sum::<f64>() + 0.0;
    // adjacent charges are one uncovered interval
    let mut charged: Vec<(Duration, Duration)> = Vec::new();
    for &(a, b, how) in &spans {
        if how != Accounted::Charged {
            continue;
        }
        match charged.last_mut() {
            Some(last) if last.1 == a => last.1 = b,
            _ => charged.push((a, b)),
        }
    }
    let slack = Duration::from_millis(1);
    let measured_there: Vec<(Duration, Duration)> = never_measured
        .iter()
        .copied()
        .filter(|&(a, b)| spans.iter().any(|s| s.2 == Accounted::Measured && s.0 + slack < b && a + slack < s.1))
        .collect();

    let rel = |t: Duration| t.saturating_sub(start).as_secs_f64();
    let mut o = String::new();
    let _ = writeln!(
        o,
        "replay {}: governor {} at {:+.1} dB (recorded {}), {} periods, {:.1} s playing over {:.1} s",
        path.display(),
        cfg.globals.governor.name(),
        opts.gain_db,
        recorded_gov.name(),
        hw.delivered,
        playing_s,
        rel(last)
    );
    let unlocks = rep_permits.iter().filter(|p| p.1).count();
    let _ = writeln!(o, "guard: {unlocks} unlocks, {} hand-offs, {} latches; {:.1} s of playback at the kernel's level", handoffs.len(), rep_latches.iter().filter(|l| l.1).count(), kernel_s);
    let mut reasons: Vec<(String, usize)> = Vec::new();
    for why in handoffs {
        match reasons.iter_mut().find(|r| &r.0 == why) {
            Some(r) => r.1 += 1,
            None => reasons.push((why.clone(), 1)),
        }
    }
    for (why, count) in &reasons {
        let _ = writeln!(o, "  hand-off: {why} ({count})");
    }
    if let Some(f) = &fatal {
        let _ = writeln!(o, "guard exited {f}");
    }
    let _ = writeln!(o, "card while playing (dB): mean {:.2}, p50 {:.2}, p95 {:.2}, max {:.2}", card.mean(), card.quantile(0.5), card.quantile(0.95), card.quantile(1.0));
    for (k, s) in cfg.speakers.iter().enumerate() {
        let h = hottest[k];
        let f = |v: f64| if v > f64::MIN { format!("{v:.1}") } else { "-".into() };
        let _ = writeln!(o, "  {}: hottest coil legacy {} asahi {} unified ff {} vi {} C", s.name, f(h.0), f(h.1), f(h.2), f(h.3));
    }
    let _ = writeln!(o, "account: {:.3} s measured, {:.3} s charged at full scale (not measured), {:.3} s cooled", total_of(Accounted::Measured), total_of(Accounted::Charged), total_of(Accounted::Cooled));
    let mut longest = charged.clone();
    longest.sort_by_key(|c| std::cmp::Reverse(c.1 - c.0));
    let _ = writeln!(o, "charged, not measured: {} uncovered intervals", charged.len());
    for c in longest.iter().take(10) {
        let _ = writeln!(o, "  {:.3} to {:.3} s ({:.1} ms)", rel(c.0), rel(c.1), (c.1 - c.0).as_secs_f64() * 1000.0);
    }
    for m in &measured_there {
        let _ = writeln!(o, "  measured where the recording lost or rejected the frames: {:.3} to {:.3} s", rel(m.0), rel(m.1));
    }
    let _ = writeln!(o, "clipping: {} periods clipped at full scale", hw.clipped_periods);
    let total: f64 = ambiguous.iter().map(|a| a.1.saturating_sub(a.0).as_secs_f64()).sum::<f64>() + 0.0;
    let _ = writeln!(o, "ambiguous: {} intervals, {:.3} s", ambiguous.len(), total);
    for a in ambiguous.iter().take(20) {
        let _ = writeln!(o, "  {:.3} to {:.3} s: {}", rel(a.0), rel(a.1), a.2);
    }
    if ambiguous.len() > 20 {
        let _ = writeln!(o, "  ({} more)", ambiguous.len() - 20);
    }
    let mut equivalent = None;
    if same {
        let tol = Duration::from_millis(300);
        let (pm, pl, pr) = compare(&live_permits, &rep_permits, tol);
        let (lm, ll, lr) = compare(&live_latches, &rep_latches, tol);
        let _ = writeln!(o, "equivalence with the live run: lock {pm} matched, {pl} live only, {pr} replay only; latch {lm} matched, {ll} live only, {lr} replay only");
        equivalent = Some(pl + pr + ll + lr == 0);
    } else {
        let _ = writeln!(o, "equivalence with the live run: not compared (another governor, gain, --max-reduction or configuration)");
    }
    let mut refused = Vec::new();
    if !measured_there.is_empty() {
        refused.push("the replay measured frames the recording lost or rejected".into());
    }
    if !ambiguous.is_empty() {
        refused.push(format!("{} ambiguous intervals", ambiguous.len()));
    }
    if !config_matches {
        refused.push("not the recorded configuration".into());
    }
    if fatal.is_some() {
        refused.push("the guard exited".into());
    }
    if equivalent == Some(false) {
        refused.push("the replayed decisions differ from the live ones".into());
    }
    let certified = refused.is_empty();
    if certified {
        let _ = writeln!(o, "certification: the recording is complete and the replay {}", if equivalent == Some(true) { "matches the live decisions" } else { "is free of ambiguous intervals" });
    } else {
        let _ = writeln!(o, "certification: refused ({})", refused.join(", "));
    }
    Ok(Outcome { text: o, certified })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::guard::fake::FakeHw;
    use crate::guard::tests::{j413, j700, run_until, tmp_state, vi_state};
    use std::cell::RefCell;
    use std::rc::Rc;

    fn secs(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    /// A Write into shared memory, so that a test can read the recording.
    #[derive(Clone, Default)]
    struct Mem(Rc<RefCell<Vec<u8>>>);

    impl Write for Mem {
        fn write(&mut self, b: &[u8]) -> io::Result<usize> {
            self.0.borrow_mut().extend_from_slice(b);
            Ok(b.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    fn tmp(name: &str) -> PathBuf {
        let p = tmp_state().unwrap();
        p.with_file_name(name)
    }

    /// Record a live run of the guard on the fake card into `path`.
    fn record(cfg: &Config, hw: &mut FakeHw, state: Option<PathBuf>, until: Duration, path: &Path) -> Guard {
        let mut g = Guard::start(hw, cfg, cfg.globals.max_reduction, state).unwrap();
        g.attach_recorder(hw, Recorder::create(path, cfg, hw.events, 1 << 30).unwrap());
        run_until(&mut g, hw, until).unwrap();
        g.rec = None;
        g
    }

    /// A healthy recording replays through the guard with the live
    /// decisions, and certifies: nothing in it is ambiguous.
    #[test]
    fn a_clean_recording_replays_to_the_live_decisions() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.6);
        hw.playing = Box::new(|t| t < secs(6.0) || t > secs(7.0));
        let path = tmp("clean.rec");
        let csv = path.with_file_name("clean.csv");
        let live = record(&cfg, &mut hw, tmp_state(), secs(10.0), &path);
        assert!(live.armed());
        let out = replay(&path, &cfg, &Replay { gain_db: 0.0, governor: None, max_reduction: None, csv: Some(csv.clone()) }).unwrap();
        assert!(out.text.contains("equivalence with the live run: lock"), "{}", out.text);
        assert!(out.certified, "{}", out.text);
        assert!(out.text.contains("live only") && !out.text.contains("certification: refused"), "{}", out.text);
        let csv = std::fs::read_to_string(csv).unwrap();
        assert!(csv.lines().count() > 100);
    }

    /// A V/I recording certifies too: the periods before the lock, played
    /// muted, replay muted, so their program does not matter.
    #[test]
    fn a_clean_vi_recording_replays_to_the_live_decisions() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.05);
        let path = tmp("vi.rec");
        let live = record(&cfg, &mut hw, Some(vi_state(&cfg, 110.0, 100.0)), secs(8.0), &path);
        assert!(live.armed());
        let out = replay(&path, &cfg, &Replay { gain_db: 0.0, governor: None, max_reduction: None, csv: None }).unwrap();
        assert!(out.certified, "{}", out.text);
        // louder: the periods before the lock are muted in the replay too
        // (the guard turns the controls down at its start), the rest is
        // played 6 dB louder and governed harder
        let loud = replay(&path, &cfg, &Replay { gain_db: 6.0, governor: None, max_reduction: None, csv: None }).unwrap();
        assert!(loud.text.contains("at +6.0 dB") && !loud.text.contains("more than 30 dB"), "{}", loud.text);
        // another governor on the same program: not compared, still run
        let uni = replay(&path, &cfg, &Replay { gain_db: 0.0, governor: Some(Governor::Unified), max_reduction: None, csv: None }).unwrap();
        assert!(uni.text.contains("governor unified") && uni.text.contains("not compared"), "{}", uni.text);
    }

    /// Bad sense data in a recording replays as what it was: uncovered time,
    /// charged at full scale, never measured, and the replay does not
    /// certify it.
    #[test]
    fn bad_sense_data_replays_charged_not_measured() {
        let cfg = j413();
        let ch = cfg.globals.channels;
        let ics: Vec<usize> = cfg.speakers.iter().map(|s| s.is_chan.unwrap()).collect();
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.02);
        hw.tweak = Some(Box::new(move |n, w| {
            if n == 30 {
                for f in w.chunks_exact_mut(ch) {
                    for &ic in &ics {
                        f[ic] = -f[ic];
                    }
                }
            }
        }));
        let path = tmp("bad.rec");
        record(&cfg, &mut hw, Some(vi_state(&cfg, 55.0, 52.0)), secs(5.0), &path);
        let tl = Timeline::read(&path, &cfg).unwrap();
        let bad: Vec<_> = tl.periods.iter().filter(|p| !p.measured).collect();
        assert_eq!(bad.len(), 1, "the bad period was not recorded as such");
        let out = replay(&path, &cfg, &Replay { gain_db: 0.0, governor: None, max_reduction: None, csv: None }).unwrap();
        assert!(!out.certified && out.text.contains("bad sense data"), "{}", out.text);
        assert!(!out.text.contains("measured where the recording"), "{}", out.text);
        assert!(out.text.contains("hand-off: the sense data is not a measurement"), "{}", out.text);
    }

    /// Review F7: two zero periods 0.5 s apart replayed as a 0.4 s cooling
    /// gap, while the live guard charged it and handed off.  The replay
    /// runs the guard, which charges the gap as the live one did, and it
    /// refuses to certify a recording with lost frames in it.
    #[test]
    fn a_gap_in_an_open_stream_is_charged_and_not_certified() {
        let cfg = j413();
        let mut hw = FakeHw::per_speaker(&cfg);
        hw.level = Box::new(|_| 0.0);
        hw.lossy = true;
        hw.control_delay = Box::new(|t| if (secs(2.0)..secs(2.05)).contains(&t) { Duration::from_millis(500) } else { Duration::ZERO });
        let path = tmp("gap.rec");
        let live = record(&cfg, &mut hw, Some(vi_state(&cfg, 55.0, 52.0)), secs(4.0), &path);
        let live_latched = live.engine.as_ref().unwrap().asahi.as_ref().unwrap().iter().any(|a| a.hard);
        let out = replay(&path, &cfg, &Replay { gain_db: 0.0, governor: None, max_reduction: None, csv: None }).unwrap();
        assert!(!out.certified, "{}", out.text);
        assert!(out.text.contains("frames the capture lost") || out.text.contains("stalled"), "{}", out.text);
        // the gap is an uncovered interval: charged, never measured
        assert!(out.text.contains("charged, not measured:") && !out.text.contains("measured where the recording"), "{}", out.text);
        let longest = out.text.lines().skip_while(|l| !l.starts_with("charged, not measured")).nth(1).unwrap_or("");
        assert!(longest.contains("ms)"), "{}", out.text);
        assert!(out.text.contains("hand-off"), "the replayed guard never handed off over the gap:\n{}", out.text);
        assert!(live_latched, "the live guard did not charge the gap");
        assert!(!out.text.contains("0 latches"), "{}", out.text);
    }

    /// The transitions the recording keeps: the start state, lock changes
    /// with their reasons, the latch, losses and failed control writes.
    #[test]
    fn the_recording_keeps_the_guards_transitions() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.1);
        hw.sensing = Box::new(|t| !(secs(2.0)..secs(3.0)).contains(&t));
        // while the lock is held (after the outage, 0.1.8 keeps the card
        // with the kernel until the charged second has cooled)
        hw.volume_fail = Box::new(|t| (secs(1.0)..secs(1.3)).contains(&t));
        let path = tmp("transitions.rec");
        record(&cfg, &mut hw, tmp_state(), secs(7.0), &path);
        let tl = Timeline::read(&path, &cfg).unwrap();
        assert!(tl.state.starts_with("speakerguardd-state 2"), "{}", tl.state);
        assert!(tl.permits.iter().any(|p| p.1) && tl.permits.iter().any(|p| !p.1 && p.2.contains("no complete sense data")), "{:?}", tl.permits);
        assert!(tl.permits.iter().any(|p| !p.1 && p.2.contains("volume control")), "{:?}", tl.permits);
        assert!(tl.losses.iter().any(|l| l.1 == LOSS_STALL), "{:?}", tl.losses);
        assert!(tl.failures.iter().any(|f| f.1 == FAIL_TARGET || f.1 == FAIL_MIN));
        let amb = tl.ambiguous();
        assert!(amb.iter().any(|a| a.2.contains("stalled")) && amb.iter().any(|a| a.2.contains("not replayed")), "{amb:?}");
    }

    /// Louder replays play the rebuilt program through the replayed guard's
    /// own gains and clip at full scale: at +20 dB the J700 program of 0.3
    /// of full scale is 3 times full scale before the card's gain and clips
    /// wherever the governor takes less than 9.5 dB, and the governor takes
    /// more reduction than at 0 dB.
    #[test]
    fn a_louder_replay_clips_and_governs_harder() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.level = Box::new(|_| 0.3);
        let path = tmp("loud.rec");
        record(&cfg, &mut hw, tmp_state(), secs(20.0), &path);
        let at = |gain_db: f64| replay(&path, &cfg, &Replay { gain_db, governor: None, max_reduction: None, csv: None }).unwrap();
        let (quiet, loud) = (at(0.0), at(20.0));
        let mean = |o: &Outcome| -> f64 {
            let l = o.text.lines().find(|l| l.starts_with("card while playing")).unwrap();
            l.split("mean ").nth(1).unwrap().split(',').next().unwrap().parse().unwrap()
        };
        assert!(loud.text.contains("clipping: ") && !loud.text.contains("clipping: 0 periods"), "{}", loud.text);
        assert!(mean(&loud) > mean(&quiet) + 3.0, "quiet:\n{}\nloud:\n{}", quiet.text, loud.text);
        assert!(loud.text.contains("not compared"), "{}", loud.text);
    }

    /// A recording that stopped at its size limit says so, and what follows
    /// is not certified.
    #[test]
    fn a_full_recording_ends_and_is_not_certified() {
        let cfg = j700();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        let path = tmp("full.rec");
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        g.attach_recorder(&hw, Recorder::create(&path, &cfg, hw.events, 400_000).unwrap());
        run_until(&mut g, &mut hw, secs(3.0)).unwrap();
        assert!(g.rec.is_none(), "the recorder did not stop at its limit");
        let tl = Timeline::read(&path, &cfg).unwrap();
        assert!(tl.truncated.is_some());
        let out = replay(&path, &cfg, &Replay { gain_db: 0.0, governor: None, max_reduction: None, csv: None }).unwrap();
        assert!(!out.certified && out.text.contains("stopped here"), "{}", out.text);
        // and an in-memory recorder sees the same header
        let mem = Mem::default();
        let _r = Recorder::to(Box::new(mem.clone()), &cfg, true, 1 << 20).unwrap();
        assert!(String::from_utf8_lossy(&mem.0.borrow()).starts_with(MAGIC));
    }
}
