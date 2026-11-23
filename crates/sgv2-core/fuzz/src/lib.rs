//! Shared harness for the sgv2-core fuzz targets: input decoding, the host
//! loop, and the property checks (see `tests/adversarial.rs` for the
//! properties). Every check is an `assert!`, so a violation is a crash
//! libFuzzer records and minimises.
//!
//! Targets: `blob` (property 5, through every `from_blob` and
//! `sgv2_lv2::load_unit`), `spp3`, `atsp` and `lsm2` (properties 1-4, 6, 7
//! under fuzzed parameter words, audio, host slicing and sense), `handoff`
//! (a thermal hand-off record from outside the process into
//! `sgv2_lv2::start_stream`) and `voicing` (the blob's pre-stage and voicing
//! artifact, through both plugins' loaders and their processing). Run from
//! `crates/sgv2-core` with cargo-fuzz; a stable toolchain works with
//! `RUSTC_BOOTSTRAP=1` (cargo-fuzz passes `-Zsanitizer`):
//!
//! ```text
//! RUSTC_BOOTSTRAP=1 cargo fuzz run -O blob -- -max_total_time=900
//! RUSTC_BOOTSTRAP=1 cargo fuzz run -O -s none --target-dir fuzz/target-nosan spp3 -- -max_len=65536
//! ```
//!
//! spp3 and atsp have no `unsafe` on x86_64, so they run fastest without a
//! sanitizer; lsm2 (the `Delay` pointer code) and blob run under ASan.
//!
//! Known defects are kept out of the generated inputs so the fuzzer keeps
//! looking for new ones; each exclusion names its finding:
//! - FINDING 1: finite samples are clamped to +-1e15 (larger ones overflow
//!   the f32 block accumulators and poison the thermal state);
//! - FINDING 3: spp3 sense dropouts are only compared with the references
//!   when no channel runs the closed loop;
//! - FINDING 4: a negative or NaN `TlimAdjCap` is replaced by its magnitude
//!   (or 0) before spp3 is built;
//! - FINDING 5: the `FailSafe` unit is only probed with finite input;
//! - FINDING 6: spp3/atsp slice invariance is only checked on finite input;
//!   atsp's is checked within one slicing family (SPEC-ATSP 11.3), as
//!   `tests/adversarial.rs` does;
//! - FINDING 9: spp3 thermal time constants are raised to at least one
//!   update after mutation;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use arbitrary::{Result, Unstructured};
use sgv2_core::{HandOff, Health, Io, ProtectionUnit, Telemetry, FAILSAFE_GAIN};

pub use arbitrary;

// ---------------------------------------------------------------------------
// Property 7: count allocations on this thread.

pub struct Counting;

thread_local! {
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

// SAFETY: forwards to the system allocator; the counter never allocates.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.realloc(ptr, layout, new_size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

pub fn allocs() -> usize {
    ALLOCS.with(|c| c.get())
}

// ---------------------------------------------------------------------------
// Decoding.

/// Largest finite magnitude fed to the units (FINDING 1).
pub const MAX_FINITE: f32 = 1e15;

pub fn sample(bits: u32) -> f32 {
    let x = f32::from_bits(bits);
    if x.is_finite() {
        x.clamp(-MAX_FINITE, MAX_FINITE)
    } else {
        x
    }
}

/// `n` channels of up to `max_len` frames: short fuzzer-chosen patterns
/// (any bit pattern, so NaN, Inf and subnormals included) tiled over
/// segments with fuzzer-chosen gains, so long, loud signals stay cheap.
pub fn signal(u: &mut Unstructured, n: usize, max_len: usize) -> Result<Vec<Vec<f32>>> {
    // mostly short (cheap executions), sometimes up to `max_len` (long
    // enough to heat the coil past its limits)
    let len = if u.ratio(1, 4)? { u.int_in_range(1..=max_len)? } else { u.int_in_range(1..=4096.min(max_len))? };
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let plen = u.int_in_range(1..=64usize)?;
        let mut pat = Vec::with_capacity(plen);
        for _ in 0..plen {
            // mostly audio-range values, sometimes raw bit patterns
            let v = if u.ratio(1, 4)? {
                sample(u.arbitrary()?)
            } else {
                (u.int_in_range(-32768..=32767i32)? as f32) / 32768.0
            };
            pat.push(v);
        }
        let mut v = Vec::with_capacity(len);
        let mut k = 0usize;
        while v.len() < len {
            let seg = u.int_in_range(1..=16384usize)?.min(len - v.len());
            let gain = u.int_in_range(0..=255u8)? as f32 / 64.0;
            for _ in 0..seg {
                let s = pat[k % plen];
                v.push(if s.is_finite() { s * gain } else { s });
                k += 1;
            }
            if u.is_empty() {
                // out of entropy: repeat the pattern at the last gain
                while v.len() < len {
                    let s = pat[k % plen];
                    v.push(if s.is_finite() { s * gain } else { s });
                    k += 1;
                }
            }
        }
        out.push(v);
    }
    Ok(out)
}

pub fn finite_only(x: &mut [Vec<f32>]) {
    for c in x.iter_mut() {
        for v in c.iter_mut() {
            if !v.is_finite() {
                *v = 0.0;
            }
        }
    }
}

pub fn all_finite(x: &[Vec<f32>]) -> bool {
    x.iter().all(|c| c.iter().all(|v| v.is_finite()))
}

/// Host slices, 1..=4096 frames each.
pub fn slices(u: &mut Unstructured) -> Result<Vec<usize>> {
    let k = u.int_in_range(1..=32usize)?;
    let mut v = Vec::with_capacity(k);
    for _ in 0..k {
        v.push(u.int_in_range(1..=4096usize)?);
    }
    Ok(v)
}

pub fn hostile_word(u: &mut Unstructured) -> Result<u32> {
    Ok(match u.int_in_range(0..=15u8)? {
        0 => f32::NAN.to_bits(),
        1 => f32::INFINITY.to_bits(),
        2 => f32::NEG_INFINITY.to_bits(),
        3 => 0,
        4 => (-0.0f32).to_bits(),
        5 => f32::MAX.to_bits(),
        6 => f32::MIN.to_bits(),
        7 => f32::MIN_POSITIVE.to_bits(),
        8 => 1,
        9 => u32::MAX,
        10 => 0x8000_0000,
        11 => 16_385,
        12 => (-1.0f32).to_bits(),
        13 => 1e30f32.to_bits(),
        14 => u.int_in_range(0..=9u32)?,
        _ => u.arbitrary()?,
    })
}

/// Overwrite or rescale up to 12 four-byte words of a `Pod` payload.
pub fn mutate_words(bytes: &mut [u8], u: &mut Unstructured) -> Result<()> {
    let words = bytes.len() / 4;
    if words == 0 {
        return Ok(());
    }
    let k = u.int_in_range(0..=12usize)?;
    for _ in 0..k {
        let i = u.choose_index(words)? * 4;
        let old = u32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]);
        let new = match u.int_in_range(0..=2u8)? {
            0 => hostile_word(u)?,
            1 => {
                // rescale an f32 field by 2^k: explores ranges, keeps sign
                let e = u.int_in_range(-60..=60i32)?;
                (f32::from_bits(old) * 2f32.powi(e)).to_bits()
            }
            _ => old ^ (1 << u.int_in_range(0..=31u32)?),
        };
        bytes[i..i + 4].copy_from_slice(&new.to_le_bytes());
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// The host loop.

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sense {
    Valid,
    Empty,
    Missing,
    Short,
    Fill(f32),
}

pub fn sense_mode(b: u8) -> Sense {
    match b % 9 {
        0 => Sense::Empty,
        1 => Sense::Missing,
        2 => Sense::Short,
        3 => Sense::Fill(f32::NAN),
        4 => Sense::Fill(f32::INFINITY),
        5 => Sense::Fill(1e30),
        6 => Sense::Fill(0.0),
        7 => Sense::Fill(-3.0),
        _ => Sense::Valid,
    }
}

pub struct Render {
    pub out: Vec<Vec<f32>>,
    pub tele: Telemetry,
}

/// Feed `x` through `unit` in `cuts` (cycled). `modes[call % len]` picks the
/// sense delivered on each call (all `Valid` when empty). Asserts property 7.
/// `each` sees the telemetry after every call with the frames done so far.
pub fn render(
    unit: &mut dyn ProtectionUnit,
    x: &[Vec<f32>],
    sense: &[Vec<f32>],
    cuts: &[usize],
    modes: &[u8],
    each: &mut Each<'_>,
) -> Render {
    let len = x[0].len();
    let mut out = vec![vec![0.0f32; len]; x.len()];
    let mut t = Telemetry::new();
    let mut scratch: Vec<Vec<f32>> = sense.to_vec();
    let mut handoff = HandOff::EMPTY;
    let mut done = 0;
    let mut call = 0;
    while done < len {
        let m = cuts[call % cuts.len()].min(len - done);
        let mode = if modes.is_empty() { Sense::Valid } else { sense_mode(modes[call % modes.len()]) };
        if let Sense::Fill(f) = mode {
            for v in scratch.iter_mut() {
                v[done..done + m].fill(f);
            }
        }
        let ins: Vec<&[f32]> = x.iter().map(|b| &b[done..done + m]).collect();
        let sns: Vec<&[f32]> = match mode {
            Sense::Empty => Vec::new(),
            Sense::Missing => scratch.iter().take(scratch.len() / 2).map(|b| &b[done..done + m]).collect(),
            Sense::Short => scratch.iter().map(|b| &b[done..done + m / 2]).collect(),
            _ => scratch.iter().map(|b| &b[done..done + m]).collect(),
        };
        let mut outs: Vec<&mut [f32]> = out.iter_mut().map(|b| &mut b[done..done + m]).collect();
        let before = allocs();
        let _h: Health = unit.process(Io { frames: m, input: &ins, sense: &sns, output: &mut outs });
        unit.telemetry(&mut t);
        let _ = unit.save_state(&mut handoff);
        assert_eq!(allocs(), before, "{:?} allocated in process/telemetry/save_state ({m} frames)", unit.kind());
        drop(outs);
        done += m;
        call += 1;
        each(done, &t, &out);
    }
    Render { out, tele: t }
}

/// Called after every host call with the frames done, the telemetry and the
/// output so far.
pub type Each<'a> = dyn FnMut(usize, &Telemetry, &[Vec<f32>]) + 'a;

pub fn nothing(_: usize, _: &Telemetry, _: &[Vec<f32>]) {}

// ---------------------------------------------------------------------------
// Checks.

/// Properties 1 and 2: finite, and never louder than `gain` times the input
/// `latency` frames earlier (a non-finite input sample leaves as 0).
pub fn check_bound(x: &[Vec<f32>], out: &[Vec<f32>], latency: usize, gain: f32, what: &str) {
    check_bound_pilot(x, out, latency, gain, 0.0, what)
}

/// [`check_bound`] with an additive allowance for a generated pilot tone
/// (spp3: at most 0.3 by law) and never above full scale when one may play.
pub fn check_bound_pilot(x: &[Vec<f32>], out: &[Vec<f32>], latency: usize, gain: f32, pilot: f32, what: &str) {
    for (c, (xi, yo)) in x.iter().zip(out).enumerate() {
        for (k, &o) in yo.iter().enumerate() {
            assert!(o.is_finite(), "{what}: non-finite output {o} at ch{c}[{k}]");
            let i = if k >= latency { xi[k - latency] } else { 0.0 };
            let prog = if i.is_finite() { (i * gain).abs() } else { 0.0 };
            // above unity (atsp's dezipper peak) the output is still held
            // to full scale
            let lim = if pilot > 0.0 {
                ((prog + pilot) * (1.0 + 1e-6)).min(1.0)
            } else if gain > 1.0 {
                prog.min(1.0)
            } else {
                prog
            };
            assert!(o.abs() <= lim, "{what}: |out| {o:e} > {lim:e} (in {i:e}, pilot {pilot}) at ch{c}[{k}]");
        }
    }
}

/// The largest pilot an spp3 set may add on any channel: 0.3 by law when
/// any channel enables the pilot tone.
pub fn spp3_pilot(p: &sgv2_core::spp3::Params) -> f32 {
    let n = (p.channels as usize).min(8);
    if p.ch[..n].iter().any(|c| c.pilot_tone_enabled != 0) {
        0.3
    } else {
        0.0
    }
}

/// Property 1's gain for a protecting atsp unit: its output dezipper's own
/// trajectory passes, as the reference's, so the gain is bounded by
/// `gain_peak` at the unit's rate (as in `tests/adversarial.rs`), not by
/// unity. The peak is a long filter run, so it is kept per rate.
pub fn atsp_peak(unit: &sgv2_core::atsp::Unit) -> f32 {
    let rate = unit.params().sample_rate;
    thread_local!(static PEAKS: std::cell::RefCell<Vec<(u32, f32)>> = const { std::cell::RefCell::new(Vec::new()) });
    PEAKS.with(|p| {
        let mut p = p.borrow_mut();
        if let Some(&(_, g)) = p.iter().find(|e| e.0 == rate) {
            return g;
        }
        let g = unit.gain_peak();
        // a fuzzed rate word is any u32: keep the table small
        if p.len() < 64 {
            p.push((rate, g));
        }
        g
    })
}

/// The allowance over property 1 an lsm2 unit takes: it is a five-band
/// limiter (an output sample mixes hundreds of input samples with per-band
/// gains, and its resonator compensation runs ahead of the limiter), so it
/// is bounded by digital full scale, not by the one input sample at its
/// delay, as in `tests/adversarial.rs`. Passed as a pilot allowance of full
/// scale to [`check_bound_pilot`].
pub const LSM2_ALLOWANCE: f32 = 1.0;

/// Property 6: `y` is nowhere louder than `reference`.
pub fn check_not_louder(y: &[Vec<f32>], reference: &[Vec<f32>], what: &str) {
    for (c, (a, b)) in y.iter().zip(reference).enumerate() {
        for (k, (&p, &q)) in a.iter().zip(b).enumerate() {
            assert!(p.abs() <= q.abs(), "{what}: ch{c}[{k}] {p:e} louder than {q:e}");
        }
    }
}

/// Property 4 for atsp, used from `render`'s `each` with equal calls of at
/// most one controller tick (as `check_hard_path` in `tests/adversarial.rs`):
/// the panic is a target step through the output dezipper, so once a group
/// has been past its hard limit at every call boundary for `settle` frames,
/// its channels leave at most `bound` times the input for as long as it
/// stays past it.
pub struct HardPath<'a> {
    pub x: &'a [Vec<f32>],
    /// Per host channel: (group, hard limit).
    pub chans: Vec<(usize, f64)>,
    /// Per group past its hard limit: the frame it has been so from.
    pub hot_since: Vec<(usize, usize)>,
    pub settle: usize,
    pub bound: f32,
    pub last: usize,
    pub hits: usize,
}

impl<'a> HardPath<'a> {
    /// atsp's glide at `rate`: the panic target is scheduled at the first
    /// tick after the coil is seen hot and the dezipper glides to within
    /// 3 % of the 20 dB target well inside 60 ms.
    pub fn atsp(x: &'a [Vec<f32>], chans: Vec<(usize, f64)>, rate: u32) -> Self {
        let settle = (0.06 * rate as f64) as usize;
        HardPath { x, chans, hot_since: Vec::new(), settle, bound: FAILSAFE_GAIN * 1.03, last: 0, hits: 0 }
    }

    pub fn step(&mut self, done: usize, t: &Telemetry, out: &[Vec<f32>]) {
        // the call just rendered was decided at the previous boundary
        let mut any = false;
        for (c, &(g, _)) in self.chans.iter().enumerate() {
            let Some(&(_, since)) = self.hot_since.iter().find(|h| h.0 == g) else { continue };
            any = true;
            let from = since.saturating_add(self.settle).max(self.last).min(done);
            for (k, (&i, &o)) in self.x[c][from..done].iter().zip(&out[c][from..done]).enumerate() {
                let k = from + k;
                let lim = if i.is_finite() { (i * self.bound).abs() } else { 0.0 };
                assert!(o.abs() <= lim, "hard path missed: ch{c}[{k}] {o} from {i} (group {g} over its hard limit)");
            }
        }
        if any {
            self.hits += 1;
        }
        let mut hot: Vec<usize> = Vec::new();
        for (c, &(g, limit)) in self.chans.iter().enumerate() {
            let temp = t.channels[c].coil_temp_c as f64;
            // f32 telemetry: skip boundaries the rounding could decide either
            // way, and temperatures beyond f32 range
            if temp.is_finite() && temp > limit + 1e-4 * limit.abs().max(1.0) && !hot.contains(&g) {
                hot.push(g);
            }
        }
        self.hot_since.retain(|h| hot.contains(&h.0));
        for g in hot {
            if !self.hot_since.iter().any(|h| h.0 == g) {
                self.hot_since.push((g, done));
            }
        }
        self.last = done;
    }
}

/// Host slicings of one atsp family (SPEC-ATSP 11.3), whose output is the
/// same bit for bit, as `atsp_family_slices` in `tests/adversarial.rs`:
/// whole multiples of the fragment (`staged`), or calls of 2 frames up to
/// one tick; one channel groups each call by eight (SPEC-ATSP 11.4), so its
/// small family is whole groups of eight up to one tick.
pub fn atsp_family_slices(u: &mut Unstructured, win: usize, staged: bool, mono: bool) -> Result<Vec<usize>> {
    use sgv2_core::atsp::FRAGMENT;
    let small = win.min(FRAGMENT);
    let staged = staged || small < 2 || (mono && small < 8);
    let k = u.int_in_range(1..=32usize)?;
    let mut v = Vec::with_capacity(k);
    for _ in 0..k {
        v.push(if staged {
            FRAGMENT * u.int_in_range(1..=8usize)?
        } else if mono {
            8 * u.int_in_range(1..=small / 8)?
        } else {
            u.int_in_range(2..=small)?
        });
    }
    Ok(v)
}

/// Whether slicing `x`'s `len` frames by `cuts` ends on a one-frame call
/// (the input running out): atsp's one-frame dezipper path, a family of
/// its own.
pub fn ends_on_one(cuts: &[usize], len: usize) -> bool {
    let (mut done, mut k) = (0, 0);
    while done < len {
        let m = cuts[k % cuts.len()].min(len - done);
        if done + m == len {
            return m == 1;
        }
        done += m;
        k += 1;
    }
    false
}

pub fn tele_bits(t: &Telemetry) -> Vec<u64> {
    let mut v = vec![t.frame, t.health.0 as u64];
    for s in &t.channels {
        for f in [
            s.modeled_power_w,
            s.measured_power_w,
            s.coil_temp_c,
            s.magnet_temp_c,
            s.ol_temp_c,
            s.cl_temp_c,
            s.tlim_adj_c,
            s.displacement_m,
            s.velocity_m_s,
            s.mask_db,
            s.gain,
        ] {
            v.push(f.to_bits() as u64);
        }
        v.extend(s.axis_gain.iter().chain(&s.band_gain).map(|f| f.to_bits() as u64));
        v.push(s.closed_loop as u64);
    }
    v
}

pub fn bits(out: &[Vec<f32>]) -> Vec<Vec<u32>> {
    out.iter().map(|c| c.iter().map(|v| v.to_bits()).collect()).collect()
}

/// Plausible V/I for the drive: `v = 10 x`, `i = v / 4.2`, delayed.
pub fn synth_sense(x: &[Vec<f32>], ports: usize, delay: usize) -> Vec<Vec<f32>> {
    let len = x[0].len();
    (0..ports)
        .map(|p| {
            let s = &x[(p / 2) % x.len()];
            (0..len)
                .map(|k| {
                    let v = if k >= delay { s[k - delay] } else { 0.0 };
                    let v = if v.is_finite() { v.clamp(-1.0, 1.0) } else { 0.0 };
                    if p % 2 == 1 {
                        10.0 * v
                    } else {
                        10.0 * v / 4.2
                    }
                })
                .collect()
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Synthetic base parameter sets (the crate's own test builders, generalised
// to `n` channels). No Mac's tuning.

pub fn spp3_base(n: usize, block: u32, rate: u32, cl: bool) -> sgv2_core::spp3::Params {
    use sgv2_core::spp3::{ChannelParams, Params};
    let mut p = Params {
        channels: n as u32,
        sample_rate: rate,
        block_frames: block,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 0.5,
        thermal_gain_release_time_s: 2.0,
        thermal_lookahead_delay_ms: 1.5,
        thermal_protection_enabled: 1,
        // SafeTlim never latches within a fuzz run (the oracle models the
        // ordinary limit)
        thermal_safe_tlim: 120.0,
        thermal_safe_tlim_time_min: 1e3,
        ..Params::default()
    };
    for c in 0..n {
        p.ch[c] = ChannelParams {
            source: c as u32,
            group: if c < n.div_ceil(2) { 7 } else { 3 },
            t_sett_vc: 20.0 + c as f32,
            t_sett_mg: 10.0,
            tau_tvc: 0.4,
            tau_tmg: 3.0,
            temperature_limit: 120.0,
            hard_temp_limit_headroom: 8.0,
            reb_ref: 4.0,
            rshunt: 0.2,
            thermal_ff_speedup_factor: 0.5,
            cl_thermal_feedback_enabled: 1,
            ag: 10.0,
            // with these scales `synth_sense` reads 4.2 ohm
            imon_scale: 1.0,
            vmon_scale: 1.0,
            i_pwr_thres: 1e-6,
            v_pwr_thres: 1e-6,
            sense_i: 2 * c as u32,
            sense_v: 2 * c as u32 + 1,
            pilot_ampl_hi_db: -30.0,
            temp_sense_window_time_s: 0.01,
            temp_sense_smoothing_tau_s: 0.1,
            rdc_scale: 1.0,
            reb_range_min: 2.0,
            reb_range_max: 8.0,
            cal_temp_range_min: 10.0,
            cal_temp_range_max: 50.0,
            a_t_20c: 0.004,
            a_t_35c: 0.0038,
            tlim_err_decay_time_s: 1.0,
            tlim_adj_cap: 30.0,
            cal_present: cl as u32,
            cal_r: 4.0,
            cal_t: 25.0,
            ..Default::default()
        };
    }
    p
}

/// FINDING 4 and 9 exclusions (see the module docs).
pub fn spp3_exclusions(p: &mut sgv2_core::spp3::Params) {
    let dt = sgv2_core::spp3::THERMAL_FRAMES as f32 / p.sample_rate.max(1) as f32;
    for c in p.ch.iter_mut() {
        c.tlim_adj_cap = if c.tlim_adj_cap.is_nan() { 0.0 } else { c.tlim_adj_cap.abs() };
        let f = c.thermal_ff_speedup_factor;
        let shortest = if f > 0.0 && f.is_finite() { f.min(1.0) } else { 1.0 };
        // and the 0.1 s parameter minimum (macOS's documented floor)
        if c.tau_tvc.is_finite() {
            c.tau_tvc = c.tau_tvc.max(dt / shortest).max(0.1);
        }
        if c.tau_tmg.is_finite() {
            c.tau_tmg = c.tau_tmg.max(dt).max(0.1);
        }
    }
}

/// Whether any spp3 channel would run the closed loop (a superset: the unit
/// also needs the calibration inside its ranges).
pub fn spp3_closed_loop(p: &sgv2_core::spp3::Params) -> bool {
    p.ch[..(p.channels as usize).min(8)]
        .iter()
        .any(|c| c.cl_thermal_feedback_enabled != 0 && c.cal_present != 0 && c.ignore_telemetry == 0)
}

pub fn atsp_base(n: usize, block: u32, rate: u32) -> sgv2_core::atsp::Params {
    use sgv2_core::atsp::{Params, Slot};
    let s = Slot {
        channel_mask: (1u32 << n) - 1,
        gain_link: 1,
        amplifier_sensitivity: 10.0,
        rdc: 8.0,
        rth_vc: 30.0,
        tau_vc: 1.0,
        rth_mg: 40.0,
        tau_mg: 20.0,
        ambient: 40.0,
        temperature_limit: 110.0,
        hard_limit_headroom: 8.0,
        attack_ms: 500.0,
        release_ms: 5000.0,
    };
    let mut b = s;
    b.channel_mask = 0;
    Params { channels: n as u32, sample_rate: rate, block_frames: block, bypass: 0, gain_link_all: 0, slot: [s, b, b] }
}

pub fn lsm2_base(n: usize, rate: u32) -> sgv2_core::lsm2::Params {
    use sgv2_core::lsm2::{ChannelParams, Params};
    let mut p = Params {
        channels: n as u32,
        sample_rate: rate,
        enable_mitigation: 1,
        control_sample_peak: 1,
        control_displacement: 1,
        control_velocity: 1,
        control_power: 1,
        control_temperature: 1,
        link_slow_gains: 1,
        model_only: 1,
        attack_time_ms: 2.0,
        fast_release_time_ms: 150.0,
        slow_release_time_ms: 10_000.0,
        low_band_release_time_ms: 150.0,
        filterbank_latency_ms: 2.0,
        speaker_response_lookahead_ms: 4.0 / 3.0,
        power_time_scale_s: [0.002, 0.2, 2.0, 30.0],
        power_hard_limit_w: [50.0, 20.0, 2.0, 1.5],
        power_soft_limit_pct: [100.0; 4],
        max_power_attenuation_db: 4.0,
        low_rail_idle_power_w: 0.0,
        low_rail_power_ratio: 1.0,
        ..Params::default()
    };
    for c in 0..n {
        p.ch[c] = ChannelParams {
            source: c as u32,
            volts_full_scale: 6.0,
            parasitic_resistance: 0.2,
            rdc: 4.0,
            bl: 0.8,
            mms_kg: 0.0002,
            rms: 0.2,
            kms: 1500.0,
            disp_rdc: 4.0,
            disp_bl: 0.8,
            disp_mms_kg: 0.00022,
            disp_rms: 0.25,
            disp_kms: 1600.0,
            vel_rdc: 4.0,
            vel_bl: 0.8,
            vel_mms_kg: 0.0002,
            vel_rms: 0.2,
            vel_kms: 1500.0,
            rth_vc: 30.0,
            tau_vc: 2.0,
            rth_mag: 20.0,
            tau_mag: 60.0,
            ambient: 40.0,
            amplitude_limit: 1.0,
            displacement_limit: 0.0005,
            velocity_limit: [0.5, 0.6, 0.9],
            mask_energy_db: [0.0, 10.0, 20.0, 30.0],
            mask_band_hz: [8000.0, 12000.0],
            mask_attack_s: 0.01,
            mask_release_s: 0.1,
            mask_attack_fast_s: 0.005,
            mask_attack_range_db: 70.0,
            max_velocity_attenuation_db: 18.0,
            velocity_ignore_low_band: 1,
            velocity_ignore_high_band: 1,
            temperature_soft_limit: 110.0,
            temperature_hard_limit: 125.0,
            temperature_headroom: 4.0,
            max_temperature_attenuation_db: 10.0,
            band_center_hz: [60.0, 300.0, 650.0, 1600.0, 5000.0],
            ..ChannelParams::default()
        };
    }
    p
}

pub fn rate(u: &mut Unstructured) -> Result<u32> {
    Ok(match u.int_in_range(0..=4u8)? {
        0 => 44_100,
        1 => 96_000,
        2 => u.int_in_range(8_000..=192_000u32)?,
        _ => 48_000,
    })
}

pub fn block(u: &mut Unstructured) -> Result<u32> {
    Ok(match u.int_in_range(0..=3u8)? {
        0 => u.int_in_range(1..=2048u32)?,
        1 => 256,
        _ => 512,
    })
}

// ---------------------------------------------------------------------------
// The spp3 hard-path oracle (the same as in tests/adversarial.rs).

/// The spp3 controller oracle; see `HardOracle` in `tests/adversarial.rs`.
pub struct Spp3HardOracle {
    /// Frames per thermal update at this rate (`sgv2_core::spp3::thermal_frames`).
    frames: usize,
    updates: usize,
    /// Per unit channel, the update at which its SafeTlim latched.
    latched_at: Vec<Option<usize>>,
    amb: f64,
    k_dez: f64,
    k_attack: f64,
    k_release: f64,
    hold_n: u32,
    ch: Vec<OracleChan>,
    /// Per group: smoothed gain, selected target, the dezipper stages.
    g: Vec<f64>,
    sel: Vec<f64>,
    allowed: Vec<f64>,
    z1: Vec<f64>,
    z2: Vec<f64>,
    /// The exact model is still in step with the unit.
    exact: bool,
    /// Per group, the unit's selected target published at the last update.
    unit_sel_prev: Vec<f64>,
    /// Updates at which some channel was certainly armed.
    pub armed_updates: usize,
    /// Updates checked exactly.
    pub exact_updates: usize,
}

#[derive(Clone, Copy, PartialEq)]
enum HoldState {
    Attack,
    Holding,
    Released,
}

struct OracleChan {
    src: usize,
    group: usize,
    elec: f64,
    ff_on: bool,
    /// Float32 node coefficients, as the unit computes them.
    a: f32,
    b: f32,
    theta_ff: f32,
    p_prev: f64,
    headroom_c: f64,
    safe: OracleSafe,
    headroom: f32,
    armed: Option<bool>,
    /// The hold states consistent with the unit so far (a comparison within
    /// rounding forks them; the unit's published candidate prunes them).
    holds: Vec<(HoldState, u32, f64, f64)>,
}

const ORACLE_CLAMP: f32 = 1.5;

/// SafeTlim as specified: the base limit leaves `TemperatureLimit` for
/// `min(SafeTlim, TemperatureLimit)` after `init = ceil(TimeMin * 60 * fs /
/// update)` consecutive updates with `f32(T_OL) > f32(Tlim_eff) + min(offset,
/// 0)` (strict; equality or NaN reloads the count), one way, from the next
/// update. A rebuild (`new`, `reset`) restarts it; AudioUnitReset keeps it.
#[derive(Clone, Copy)]
struct OracleSafe {
    normal: f64,
    safe: f64,
    offset: f32,
    init: u32,
    count: u32,
    latched: bool,
}

impl OracleSafe {
    fn base(&self) -> f64 {
        if self.latched {
            self.safe
        } else {
            self.normal
        }
    }
    fn update(&mut self, t_ol: f32, tlim_eff: f64) {
        if t_ol > tlim_eff as f32 + self.offset {
            self.count = self.count.wrapping_sub(1);
        } else {
            self.count = self.init;
        }
        if self.count == 0 {
            self.latched = true;
        }
    }
}

impl Spp3HardOracle {
    pub fn new(p: &sgv2_core::spp3::Params) -> Self {
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let upd = sgv2_core::spp3::thermal_frames(p.sample_rate);
        let dt32 = upd as f32 / p.sample_rate as f32;
        let r = rate / upd as f64;
        let amb = p.thermal_ambient_temp as f64;
        let init =
            sgv2_core::spp3::safe_count_init(p.thermal_safe_tlim_time_min, p.sample_rate).expect("valid TimeMin");
        let cold = p.thermal_state_override == sgv2_core::spp3::THERMAL_STATE_COLD;
        let mut ids: Vec<u32> = Vec::new();
        let ch = p.ch[..n]
            .iter()
            .map(|c| {
                let group = ids.iter().position(|&g| g == c.group).unwrap_or_else(|| {
                    ids.push(c.group);
                    ids.len() - 1
                });
                let (reb, series, ag) = (c.reb_ref as f64, (c.rshunt + c.rampout) as f64, c.ag as f64);
                let ff_on = c.thermal_ff_speedup_factor > 0.0;
                let tau = if ff_on { c.thermal_ff_speedup_factor * c.tau_tvc } else { c.tau_tvc };
                let (tvc, tmg) = (c.t_sett_vc as f64, c.t_sett_mg as f64);
                let seed = if cold { 0.0 } else { (c.temperature_limit as f64 - amb) / (tvc + tmg) * tvc };
                OracleChan {
                    src: c.source as usize,
                    group,
                    elec: ag * ag * reb / ((reb + series) * (reb + series)),
                    ff_on,
                    a: 1.0 - dt32 / tau,
                    b: 0.5 * c.t_sett_vc * dt32 / tau,
                    theta_ff: seed as f32,
                    p_prev: 0.0,
                    headroom_c: c.hard_temp_limit_headroom as f64,
                    safe: OracleSafe {
                        normal: c.temperature_limit as f64,
                        safe: p.thermal_safe_tlim.min(c.temperature_limit) as f64,
                        offset: p.thermal_safe_tlim_offset.min(0.0),
                        init,
                        count: init,
                        latched: false,
                    },
                    // pilot amplitude capped at 0.3 (rig law 4)
                    headroom: if c.pilot_tone_enabled != 0 {
                        1.0 - (10f64.powf(c.pilot_ampl_hi_db as f64 / 20.0) as f32).min(0.3)
                    } else {
                        1.0
                    },
                    armed: Some(false),
                    holds: vec![(HoldState::Attack, 0, 1.0, 1.0)],
                }
            })
            .collect();
        let g = ids.len();
        let settle = |s: f64, rate: f64| 1.0 - (0.01f64.ln() / (s * rate)).exp();
        Spp3HardOracle {
            frames: upd,
            updates: 0,
            latched_at: vec![None; n],
            amb,
            k_dez: settle(0.005, rate),
            k_attack: settle(p.thermal_gain_attack_time_s as f64, r),
            k_release: settle(p.thermal_gain_release_time_s as f64, r),
            hold_n: (1.0 * r).ceil() as u32,
            ch,
            g: vec![1.0; g],
            sel: vec![1.0; g],
            allowed: vec![1.0; g],
            z1: vec![1.0; g],
            z2: vec![1.0; g],
            exact: true,
            unit_sel_prev: vec![1.0; g],
            armed_updates: 0,
            exact_updates: 0,
        }
    }

    /// The release hold (rig law 5). Returns the candidate and whether the
    /// comparison was within rounding.
    /// The release hold (rig law 5), one step with the comparison
    /// `g >= sel_prev` given as `ge`.
    fn hold_step(h: (HoldState, u32, f64, f64), g: f64, ge: bool, n: u32) -> ((HoldState, u32, f64, f64), f64) {
        let (mut state, mut cnt, mut held, _) = h;
        let u = if cnt > 0 && ge {
            cnt -= 1;
            if cnt == 0 {
                state = HoldState::Released;
            }
            held
        } else {
            cnt = 0;
            if ge {
                if state == HoldState::Attack {
                    held = h.3;
                    cnt = n.saturating_sub(1);
                    state = if cnt == 0 { HoldState::Released } else { HoldState::Holding };
                    held
                } else {
                    g
                }
            } else {
                state = HoldState::Attack;
                g
            }
        };
        ((state, cnt, held, u), u)
    }

    /// The hold for this update over every state still consistent with the
    /// unit. A comparison within what the published temperatures resolve
    /// (the candidate rests on the f32 magnet temperature; exact only when
    /// clamped at unity or the floor) forks the state into both branches;
    /// branches whose candidate disagrees with the unit's published one
    /// (`controller_state()[0]`) are dropped. Returns the candidate, or
    /// `None` when no lawful branch gives the unit's candidate although a
    /// fork was involved (undecided). Without a fork the model's own
    /// prediction is returned, and the exact check judges it.
    fn hold(holds: &mut Vec<(HoldState, u32, f64, f64)>, g: f64, sel_prev: f64, n: u32, unit_u: f64) -> Option<f64> {
        let exact_sides = (g == 1.0 || g == 0.01) && g == sel_prev;
        let tie = !exact_sides && (g - sel_prev).abs() <= 1e-3 * sel_prev.abs().max(1e-3);
        let near = |a: f64, b: f64| (a - b).abs() <= 1e-4 * b.abs().max(1e-3);
        let forked = tie || holds.len() > 1;
        let mut next: Vec<((HoldState, u32, f64, f64), f64)> = Vec::new();
        for &h in holds.iter() {
            let branches: &[bool] = if tie {
                &[true, false]
            } else if g >= sel_prev {
                &[true]
            } else {
                &[false]
            };
            for &ge in branches {
                let (nh, u) = Self::hold_step(h, g, ge, n);
                if !next.iter().any(|(o, _)| o.0 == nh.0 && o.1 == nh.1 && near(o.2, nh.2) && near(o.3, nh.3)) {
                    next.push((nh, u));
                }
            }
        }
        let first = next[0].1;
        let mut keep: Vec<_> = next.iter().filter(|(_, u)| near(*u, unit_u)).cloned().collect();
        // the closest first, so a full set drops the least likely states
        keep.sort_by(|a, b| (a.1 - unit_u).abs().total_cmp(&(b.1 - unit_u).abs()));
        if keep.is_empty() {
            if forked {
                *holds = vec![next[0].0];
                return None;
            }
            *holds = vec![next[0].0];
            return Some(first);
        }
        *holds = keep.iter().take(32).map(|(h, _)| *h).collect();
        Some(keep[0].1)
    }

    /// One thermal update: `x` and `out` are the update's 256 frames per host
    /// channel, `t` the telemetry after it.
    /// `unit_cs[c]` is unit channel `c`'s `sgv2_core::spp3::Unit::controller_state(c)`:
    /// `[candidate, selected target, Tlim_eff, theta_ff, theta_vc, theta_mg,
    /// ..]` (the telemetry's axis 4 is the applied, dezippered gain). The
    /// oracle checks the unit's `theta_ff` against its own independent
    /// virtual coil, then takes one step of the controller laws from the
    /// unit's published state, so rounding in the published temperatures
    /// never compounds.
    pub fn step(&mut self, x: &[&[f32]], out: &[&[f32]], t: &Telemetry, unit_cs: &[[f64; 11]]) {
        let frames = x[0].len();
        assert_eq!(frames, self.frames, "the oracle steps one thermal update per call");
        // each render call (one per update here) starts both dezipper stages
        // at the selected target (rig dezipper law); the audio of this update
        // then glides toward the target set at the last one
        for g in 0..self.allowed.len() {
            self.z1[g] = self.allowed[g];
            self.z2[g] = self.allowed[g];
        }
        for k in 0..frames {
            for g in 0..self.allowed.len() {
                self.z1[g] += self.k_dez * (self.allowed[g] - self.z1[g]);
                self.z2[g] += self.k_dez * (self.z1[g] - self.z2[g]);
            }
            for c in &self.ch {
                let i = x[c.src][k];
                if !i.is_finite() {
                    continue; // check_bound: it leaves as 0
                }
                let drive = i.clamp(-ORACLE_CLAMP, ORACLE_CLAMP).abs() as f64;
                // two f32 roundings, absolute on a subnormal input
                let bound = (drive * (self.z2[c.group] * c.headroom as f64 * (1.0 + 1e-4) + 1e-6)).min(1.0)
                    + 2.0 * f32::from_bits(1) as f64;
                let o = out[c.src][k].abs() as f64;
                assert!(
                    o <= bound,
                    "spp3 controller: ch{} sample {k} of the update leaves at {o:e} for input {i:e}, allowed {:.6} x",
                    c.src,
                    self.z2[c.group] * c.headroom as f64
                );
            }
        }
        // the feed-forward node, on this update's own unprotected frames
        for c in self.ch.iter_mut() {
            let acc: f64 = x[c.src]
                .iter()
                .map(|&v| if v.is_finite() { v.clamp(-ORACLE_CLAMP, ORACLE_CLAMP) as f64 } else { 0.0 })
                .map(|v| v * v)
                .sum();
            let p = c.elec * acc / frames as f64;
            // the Float32 fused recurrence
            c.theta_ff = c.a.mul_add(c.theta_ff, c.b * (p as f32 + c.p_prev as f32));
            c.p_prev = p;
        }
        // candidates, hold, arming, from the published temperatures
        let ng = self.g.len();
        let mut ff_t = vec![1.0f64; ng];
        let mut hard_t = vec![1.0f64; ng];
        let mut hard_bound = vec![1.0f64; ng];
        let mut certain = true;
        let mut any = false;
        let base_used: Vec<f64> = self.ch.iter().map(|c| c.safe.base()).collect();
        let ratio = |num: f64, vcff: f64| (num / (vcff + 1.0)).clamp(1e-4, 1.0).sqrt();
        for (ci, c) in self.ch.iter_mut().enumerate() {
            let s = &t.channels[c.src];
            let (t_ol, mag) = (s.coil_temp_c as f64, s.magnet_temp_c as f64);
            if !(t_ol.is_finite() && mag.is_finite()) {
                c.armed = None;
                certain = false;
                continue;
            }
            // the published T_OL is the unit's f64 value rounded to f32: within
            // a few f32 ulps a comparison could go either way
            let tol = 4.0 * (f32::from_bits((t_ol as f32).abs().to_bits() + 1) - (t_ol as f32).abs()) as f64;
            // the unit: arm above `hard_on`, else disarm below `limit`, else
            // keep; a comparison within rounding takes both branches
            let cmp = |a: f64, b: f64| {
                if a > b + tol {
                    Some(true)
                } else if a < b - tol {
                    Some(false)
                } else {
                    None
                }
            };
            let keep_or_disarm = |prev: Option<bool>| match cmp(t_ol, c.safe.base()) {
                // above the limit: keep; below: disarm
                Some(true) => prev,
                Some(false) => Some(false),
                None => (prev == Some(false)).then_some(false),
            };
            let prev = c.armed;
            let (limit, hard_on) = (c.safe.base(), c.safe.base() + c.headroom_c);
            c.armed = match cmp(t_ol, hard_on) {
                Some(true) => Some(true),
                Some(false) => keep_or_disarm(prev),
                None => (keep_or_disarm(prev) == Some(true)).then_some(true),
            };
            // the virtual coil, independently: the unit's must agree
            let cs = &unit_cs[ci];
            if c.ff_on {
                let (mine, unit) = (c.theta_ff as f64, cs[3]);
                assert!(
                    (mine - unit).abs() <= 1e-4 * unit.abs().max(1.0),
                    "spp3 feed-forward node: ch{} theta_ff {unit}, the specified recurrence gives {mine} (update {})",
                    c.src,
                    self.updates
                );
            }
            let theta_mg = if cs[5].is_finite() { cs[5] } else { mag - self.amb };
            let num = limit - self.amb - theta_mg;
            let theta_ff = if c.ff_on && cs[3].is_finite() { cs[3] } else { c.theta_ff as f64 };
            let fresh = if c.ff_on { ratio(num, theta_ff) } else { 1.0 };
            let g_ff = match Self::hold(&mut c.holds, fresh, self.g[c.group], self.hold_n, unit_cs[ci][0]) {
                Some(u) => u,
                None => {
                    certain = false;
                    unit_cs[ci][0]
                }
            };
            ff_t[c.group] = ff_t[c.group].min(g_ff);
            match c.armed {
                Some(true) => {
                    any = true;
                    let g_hard = if c.ff_on { ratio(num - 5.0, theta_ff) } else { FAILSAFE_GAIN as f64 };
                    hard_t[c.group] = hard_t[c.group].min(g_hard);
                    // the bound: generous in theta_mg and theta_ff rounding
                    let slack = 1e-4 * (limit.abs() + mag.abs() + 1.0);
                    let gb = if c.ff_on {
                        ratio(num - 5.0 + slack, theta_ff * (1.0 - 1e-4) - 1e-6)
                    } else {
                        FAILSAFE_GAIN as f64
                    };
                    hard_bound[c.group] = hard_bound[c.group].min(gb);
                }
                Some(false) => {}
                None => certain = false,
            }
            // SafeTlim, for the next update: the unit's own Float32 T_OL
            // (the published value) against the base it used
            let was = c.safe.latched;
            c.safe.update(s.coil_temp_c, limit);
            if c.safe.latched && !was {
                self.latched_at[ci] = Some(self.updates);
            }
        }
        self.updates += 1;
        if any {
            self.armed_updates += 1;
        }
        if !certain {
            self.exact = false;
        }
        for gi in 0..ng {
            let tg = ff_t[gi];
            let k = if tg < self.g[gi] { self.k_attack } else { self.k_release };
            let g = self.g[gi] + k * (tg - self.g[gi]);
            self.sel[gi] = g.min(hard_t[gi]);
            self.g[gi] = self.sel[gi];
        }
        // the checks, then the targets the next update's audio may follow
        for (ci, c) in self.ch.iter().enumerate() {
            let sel = unit_cs[ci][1];
            let hb = hard_bound[c.group];
            // the base limit the unit used (TlimAdj meter: effective limit
            // against TemperatureLimit; open loop, so the effective limit is
            // the SafeTlim base), exactly
            let adj = t.channels[c.src].tlim_adj_c as f64;
            let want_adj = (base_used[ci] - c.safe.normal) as f32 as f64;
            assert!(
                (adj - want_adj).abs() <= 1e-4 * want_adj.abs().max(1.0),
                "spp3 SafeTlim: ch{} effective limit {} against {} (update {})",
                c.src,
                c.safe.normal + adj,
                base_used[ci],
                self.updates - 1
            );
            assert!(sel <= hb * (1.0 + 1e-4) + 1e-6, "spp3 hard path: group target {sel} above the hard target {hb}");
            if self.exact {
                let want = self.sel[c.group];
                assert!(
                    (sel - want).abs() <= 1e-3 * want + 1e-6,
                    "spp3 controller: group target {sel}, the specified laws give {want} (update {})",
                    self.exact_updates
                );
            }
        }
        if self.exact {
            self.exact_updates += 1;
        }
        for ((a, &sel), &hb) in self.allowed.iter_mut().zip(&self.sel).zip(&hard_bound) {
            *a = if self.exact { sel * (1.0 + 1e-3) + 1e-6 } else { hb };
        }
        // each update is checked as one step of the laws from the unit's own
        // previous selected gain (checked above), so rounding in the
        // published temperatures never accumulates in the model's smoother
        for (ci, c) in self.ch.iter().enumerate() {
            self.g[c.group] = unit_cs[ci][1];
        }
        for (ci, c) in self.ch.iter().enumerate() {
            self.unit_sel_prev[c.group] = unit_cs[ci][1];
        }
    }
}

#[cfg(test)]
#[path = "../../tests/voicing_fixture/mod.rs"]
mod voicing_fixture;

#[cfg(test)]
mod tests {
    use super::*;

    /// The `voicing` target's bases load: every variant's artifact on the
    /// voicing plugin, and an identity pre-stage on the protect plugin.
    #[test]
    fn every_voicing_base_loads() {
        for lanes in [1usize, 2, 4, 6] {
            let base = voicing_fixture::unit("j413", lanes);
            let spec = voicing_fixture::spec(lanes);
            let joined = sgv2_core::blob::with_voicing(&base, &spec).unwrap_or_else(|e| panic!("{lanes}: {e:?}"));
            assert!(sgv2_lv2::voicing::Voicing::load(Some(&joined), "j413", 48_000.0, lanes).ready(), "{lanes}");
        }
    }

    /// The mutation bases must start valid, or every mutation explores only
    /// the refusal path (the lsm2 base drifted out of step with the unit's
    /// parameters once already).
    #[test]
    fn every_base_is_a_valid_unit() {
        for n in 1..=8 {
            for rate in [44_100, 48_000, 96_000] {
                if rate == 48_000 {
                    // lsm2's controller cadence fixes its latency per rate:
                    // the base is made for 48 kHz, as the unit's own tests
                    assert_eq!(sgv2_core::lsm2::Unit::init_error(&lsm2_base(n, rate)), None, "lsm2 {n} ch");
                }
                assert_eq!(sgv2_core::atsp::Unit::init_error(&atsp_base(n, 512, rate)), None, "atsp {n} ch {rate} Hz");
                for cl in [false, true] {
                    let p = spp3_base(n, 512, rate, cl);
                    assert_eq!(sgv2_core::spp3::Unit::init_error(&p), None, "spp3 {n} ch {rate} Hz cl {cl}");
                }
            }
        }
    }
}
