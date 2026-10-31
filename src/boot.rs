// SPDX-License-Identifier: MIT
//! The model's starting point at the first start of a boot, from evidence
//! (`boot_resume = evidence`, the default).
//!
//! macOS's loudspeaker manager starts its thermal model at the chassis
//! ambient every time it starts, and plays at full level straight away.
//! The previous default here started every boot at the coils' hard limit,
//! latched, which held the Neo 20 dB down for half a minute and then
//! released 12 dB over eight minutes.  This start is as loud as macOS's
//! wherever that can be shown to be safe, and no louder.
//!
//! **The one assumption.**  Whatever drove the speakers before this boot
//! kept every voice coil at or under its hard limit `T` (`t_limit`): this
//! daemon (its latch and floor hand the card over before a coil can get
//! there), the kernel's safe level when no daemon held the card (far under
//! it), or another OS's own protection (macOS's loudspeaker manager on
//! these machines enforces the same hard limit, from the same tuning).
//! No other assumption is made: not that no other OS ran, not that the
//! clocks are right beyond what they are checked for, not that the machine
//! was off.
//!
//! **The magnet bound.**  With the two-node model (the coil follows the
//! magnet plus `P Rc` with `tau_c`, the magnet follows ambient plus `P Rm`
//! with `tau_m`), let `r = Rm / Rc` and `Y = tau_m M - r tau_c C`.
//! Eliminating `P` between the two equations gives
//!
//! ```text
//! Y' = ta + r (1 - e) C - (1 + r) Y / tau_m,   e = (1 + r) tau_c / tau_m
//! ```
//!
//! With `e < 1` the coefficient of `C` is positive, so while `C <= T`,
//! `Y` relaxes at least as fast as toward `Y* = tau_m (ta + r (1 - e) T) /
//! (1 + r)` from wherever it started, whatever the power did.  Then
//! `M = (Y + r tau_c C) / tau_m <= (Y + r tau_c T) / tau_m`, which at `Y*`
//! is exactly `M* = (ta + r T) / (1 + r)`: the magnet of a coil held at its
//! limit for ever.  A magnet can never be pumped above that by bursts, and
//! from a known (saved) magnet it approaches it no faster than the formula
//! says (`magnet_bound`).  On the Neo `M*` is 84.5 C, where the latched
//! start assumed 135 C and waited for it to cool with a 100 s time constant.
//!
//! **The start.**  Every model node starts from the hottest it can be now:
//! the coil at `T` and the magnet at the bound above at the moment the last
//! owner stopped, then at least this boot's uptime at the kernel's safe
//! level (no daemon held the card before this one: no state under /run),
//! for any length of time from there on (`tail_bound`).  A record from the
//! last run (a clean stop, or a checkpoint written every few seconds while
//! running, see `Record`) narrows the magnet's bound to its saved magnet
//! plus what the time since can have added; without one, or one that
//! cannot be trusted, the bound is `M*` itself.  Chassis sensors (hwmon)
//! can only raise a start, never lower it: a coil can be 80 K over its
//! chassis a second after loud playback, so no sensor can vouch for it.
//!
//! **Clocks.** Only the RTC interval since the record narrows anything,
//! and only by bounding how long another owner had to warm the magnet: a
//! clock that runs fast or jumps forward only makes the start hotter.  A
//! clock that went back, an RTC that disagrees with the system clock at
//! either end, or a record written without a synchronized clock, gives the
//! interval no upper bound (the start is `M*`'s).  The cooling credit is
//! this boot's uptime alone, which no clock setting can change.

use std::path::Path;
use std::time::Duration;

use crate::engine::Thermal;

/// Discretization margin on the magnet bound (K): the daemon's model steps
/// the magnet before the coil in 10 ms substeps, the proof is continuous.
/// The randomized test below shows the discrete model inside 0.05 K of it.
pub const MAGNET_MARGIN_K: f64 = 1.0;
/// Margin over a chassis sensor that raises a start (K).
pub const SENSOR_MARGIN_K: f64 = 2.0;
/// How often a running daemon checkpoints its model for the next boot.
pub const CHECKPOINT_INTERVAL: Duration = Duration::from_secs(10);
/// Two clocks read a little apart, and the RTC's whole seconds.
const SLACK: Duration = Duration::from_secs(2);
const MAGIC: &str = "speakerguardd-evidence 1";

/// The hottest a magnet can be after `d` seconds (None: any time) of any
/// program that kept its coil at or under `t_hard`, from a saved magnet
/// `m0` (None: unknown).  See the module comment.
pub fn magnet_bound(th: &Thermal, t_hard: f64, ta: f64, m0: Option<f64>, d: Option<f64>) -> f64 {
    let r = th.tr_magnet / th.tr_coil;
    let e = (1.0 + r) * th.tau_coil / th.tau_magnet;
    let valid = [r, e, t_hard, ta].iter().all(|v| v.is_finite()) && r > 0.0 && e < 1.0;
    if !valid {
        // the bound needs the magnet much slower than the coil; without it
        // only the independent one holds: nothing over the hard limit
        return m0.map_or(t_hard, |m| m.max(t_hard));
    }
    let y_star = th.tau_magnet * (ta + r * (1.0 - e) * t_hard) / (1.0 + r);
    let y = match m0 {
        None => y_star,
        Some(m0) => {
            // the coil was at least the ambient (no model node goes
            // colder), which only makes this starting Y larger
            let y0 = th.tau_magnet * m0 - r * th.tau_coil * ta;
            match d {
                _ if y0 >= y_star => y0,
                None => y_star,
                Some(d) => y_star + (y0 - y_star) * (-(1.0 + r) * d.max(0.0) / th.tau_magnet).exp(),
            }
        }
    };
    (y + r * th.tau_coil * t_hard) / th.tau_magnet + MAGNET_MARGIN_K
}

/// The hottest the model's coil and magnet can be at any time `lo` seconds
/// or more after (coil, magnet), at a constant `power` (the card's safe
/// level's worst case) all along.
pub fn tail_bound(th: &Thermal, coil: f64, magnet: f64, power: f64, lo: f64, ta: f64) -> (f64, f64) {
    let mut t = th.clone();
    t.t_coil = coil;
    t.t_magnet = magnet;
    t.hold(power, lo.max(0.0), ta);
    let (c_at, m_at) = (t.t_coil, t.t_magnet);
    // from there on, for as long as it may be: the end of a very long hold
    // is the steady state, and `hold` returns its bounds over the span
    let (c, m) = t.hold(power, 1e7, ta);
    let steady_m = ta + power * th.tr_magnet;
    let steady_c = steady_m + power * th.tr_coil;
    (c.max(c_at).max(steady_c), m.max(m_at).max(steady_m))
}

/// The start of one node pair: the coil at `t_hard` (or the saved coil, if
/// hotter) and the bounded magnet when the last owner stopped, then at
/// least `uptime` at `safe_power`.
pub fn start_bound(th: &Thermal, t_hard: f64, ta: f64, safe_power: f64, uptime: f64, saved: Option<(f64, f64)>, d: Option<f64>) -> (f64, f64) {
    let m = magnet_bound(th, t_hard, ta, saved.map(|s| s.1), d);
    let m = saved.map_or(m, |s| m.max(s.1));
    let c = saved.map_or(t_hard, |s| s.0.max(t_hard)).max(m);
    tail_bound(th, c, m, safe_power, uptime, ta)
}

/// The most energy (J) a coil kept at or under `t_hard` can have taken in
/// `len` seconds: from tau_c C' = M + P Rc - C, the energy is
/// (tau_c (C_end - C_start) + integral (C - M)) / Rc, with C <= t_hard and
/// C, M >= ta.
pub fn protected_energy(th: &Thermal, t_hard: f64, ta: f64, len: f64) -> f64 {
    if !(len > 0.0) {
        return 0.0;
    }
    let rise = (t_hard - ta).max(0.0);
    (th.tau_coil * rise + len * rise) / th.tr_coil
}

/// The power budgets' history at a start (newest last, (seconds, watts)):
/// at the safe level since this boot began, and before that at most
/// `older` joules in the rest of the minute.
pub fn budget_history(uptime: f64, safe_power: f64, older: f64) -> Vec<(f64, f64)> {
    let recent = uptime.clamp(0.0, 60.0);
    let len = 60.0 - recent;
    let mut h = Vec::new();
    if len > 0.0 {
        h.push((len, older.max(0.0) / len));
    }
    if recent > 0.0 {
        h.push((recent, safe_power));
    }
    h
}

/// The hottest chassis or ambient sensor the SMC reports (°C), if any.
pub fn chassis_temperature(hwmon: &Path) -> Option<f64> {
    let mut best: Option<f64> = None;
    for dev in std::fs::read_dir(hwmon).ok()?.flatten() {
        let dir = dev.path();
        let name = std::fs::read_to_string(dir.join("name")).unwrap_or_default();
        if name.trim() != "macsmc_hwmon" {
            continue;
        }
        let Ok(entries) = std::fs::read_dir(&dir) else { continue };
        for e in entries.flatten() {
            let file = e.file_name().to_string_lossy().to_string();
            let Some(stem) = file.strip_suffix("_label") else { continue };
            let label = std::fs::read_to_string(e.path()).unwrap_or_default();
            let label = label.trim();
            if label != "Ambient" && !label.starts_with("Chassis") {
                continue;
            }
            let Some(v) = std::fs::read_to_string(dir.join(format!("{stem}_input"))).ok().and_then(|v| v.trim().parse::<f64>().ok()) else { continue };
            let c = v / 1000.0;
            // a reading outside what a chassis can be is a broken sensor
            if c.is_finite() && (-40.0..=150.0).contains(&c) {
                best = Some(best.map_or(c, |b: f64| b.max(c)));
            }
        }
    }
    best
}

/// What a record says about the moment it was written.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Stamp {
    pub boot_id: Option<String>,
    pub machine_id: Option<String>,
    /// The RTC (whole seconds since the epoch).
    pub rtc: Option<u64>,
    /// CLOCK_REALTIME.
    pub realtime: Option<Duration>,
    /// The system clock was synchronized (NTP) when read.
    pub synced: bool,
}

/// A record of the model for the next boot under /var/lib: a clean stop's
/// (`clean`), or a checkpoint of a running daemon (`running`).  Both are
/// used the same way (a starting magnet and the time since); a checkpoint
/// carries no promise that the program stopped with it, and needs none.
#[derive(Debug, Clone, PartialEq)]
pub struct Record {
    pub clean: bool,
    pub stamp: Stamp,
    pub config: u64,
    /// Boot time from the snapshot to the clock reads.
    pub pairing: Duration,
    /// The state text (state.rs) of the snapshot.
    pub state: String,
}

fn fnv(text: &str) -> u64 {
    text.bytes().fold(0xcbf29ce484222325u64, |h, b| (h ^ b as u64).wrapping_mul(0x100000001b3))
}

fn word(s: &Option<String>) -> String {
    s.as_deref().filter(|w| !w.is_empty() && !w.contains(char::is_whitespace)).unwrap_or("-").to_string()
}

pub fn encode(r: &Record) -> String {
    let body = format!(
        "kind {}\nboot_id {}\nmachine_id {}\nconfig {:016x}\nrtc_s {}\nrealtime_ns {}\nsynced {}\npairing_ns {}\n{}",
        if r.clean { "clean" } else { "running" },
        word(&r.stamp.boot_id),
        word(&r.stamp.machine_id),
        r.config,
        r.stamp.rtc.map_or("-".into(), |v| v.to_string()),
        r.stamp.realtime.map_or("-".into(), |v| v.as_nanos().to_string()),
        r.stamp.synced as u8,
        r.pairing.as_nanos(),
        r.state
    );
    format!("{MAGIC}\nsum {:016x}\n{body}", fnv(&format!("{MAGIC}\n{body}")))
}

pub fn decode(text: &str) -> Option<Record> {
    let rest = text.strip_prefix(MAGIC)?.strip_prefix('\n')?;
    let (sum, body) = rest.strip_prefix("sum ")?.split_once('\n')?;
    if sum.len() != 16 || u64::from_str_radix(sum, 16).ok()? != fnv(&format!("{MAGIC}\n{body}")) {
        return None;
    }
    let mut lines = body.splitn(9, '\n');
    let mut field = |key: &str| -> Option<String> { lines.next()?.strip_prefix(key)?.strip_prefix(' ').map(str::to_string) };
    let clean = match field("kind")?.as_str() {
        "clean" => true,
        "running" => false,
        _ => return None,
    };
    let opt = |v: String| (v != "-").then_some(v);
    let boot_id = opt(field("boot_id")?);
    let machine_id = opt(field("machine_id")?);
    let config = u64::from_str_radix(&field("config")?, 16).ok()?;
    let rtc = match field("rtc_s")?.as_str() {
        "-" => None,
        v => Some(v.parse().ok()?),
    };
    let realtime = match field("realtime_ns")?.as_str() {
        "-" => None,
        v => Some(Duration::from_nanos(v.parse().ok()?)),
    };
    let synced = match field("synced")?.as_str() {
        "0" => false,
        "1" => true,
        _ => return None,
    };
    let pairing = Duration::from_nanos(field("pairing_ns")?.parse().ok()?);
    let state = lines.next()?.to_string();
    Some(Record { clean, stamp: Stamp { boot_id, machine_id, rtc, realtime, synced }, config, pairing, state })
}

/// A boot id as the kernel writes it, compared without dashes or case.
fn same_id(a: &str, b: &str) -> bool {
    let n = |s: &str| s.trim().replace('-', "").to_ascii_lowercase();
    !n(a).is_empty() && n(a) == n(b)
}

fn agree(s: &Stamp) -> bool {
    match (s.rtc, s.realtime) {
        (Some(rtc), Some(rt)) => Duration::from_secs(rtc).abs_diff(rt) <= SLACK,
        _ => false,
    }
}

/// Why a record cannot be used at all (another machine, configuration or
/// this very boot), or the longest the time since its snapshot can be (s),
/// None when the clocks give no upper bound.
pub fn usable(r: &Record, now: &Stamp, config: u64) -> Result<Option<f64>, String> {
    match (&r.stamp.machine_id, &now.machine_id) {
        (Some(a), Some(b)) if a == b => {}
        _ => return Err("written on another machine (or without a machine id)".into()),
    }
    if r.config != config {
        return Err("written under another configuration".into());
    }
    match (&r.stamp.boot_id, &now.boot_id) {
        (Some(a), Some(b)) if same_id(a, b) => return Err("written in this boot".into()),
        (Some(_), Some(_)) => {}
        _ => return Err("no boot id".into()),
    }
    Ok(elapsed_max(r, now))
}

/// The longest the time since the record's snapshot can have been (s), by
/// an RTC that agreed with a synchronized system clock when the record was
/// written and agrees with the system clock now; None otherwise.
pub fn elapsed_max(r: &Record, now: &Stamp) -> Option<f64> {
    if !(r.stamp.synced && agree(&r.stamp) && agree(now)) {
        return None;
    }
    let then = r.stamp.rtc?;
    let at = now.rtc?;
    let rtc = at.checked_sub(then)? as f64;
    // whole seconds at both ends, the reads' placement, 0.1% rate error,
    // and the time from the snapshot to its clock reads
    let hi = rtc + SLACK.as_secs_f64() + 0.1 + rtc / 999.0 + r.pairing.as_secs_f64();
    hi.is_finite().then_some(hi)
}

/// The clocks and ids as read now (Linux).
pub fn read_stamp() -> Stamp {
    let read = |p: &str| std::fs::read_to_string(p).ok().map(|s| s.trim().to_string()).filter(|s| !s.is_empty());
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: a valid, writable timespec.
    let ok = unsafe { libc::clock_gettime(libc::CLOCK_REALTIME, &mut ts) } == 0 && ts.tv_sec > 0;
    let realtime = ok.then(|| Duration::new(ts.tv_sec as u64, ts.tv_nsec as u32));
    Stamp {
        boot_id: read("/proc/sys/kernel/random/boot_id"),
        machine_id: read("/etc/machine-id"),
        rtc: read("/sys/class/rtc/rtc0/since_epoch").and_then(|s| s.parse().ok()).filter(|&s| s > 0),
        realtime,
        synced: crate::wallclock::synchronized_realtime().is_some(),
    }
}

/// Writes checkpoints for the next boot off the protection loop: a durable
/// write can block for longer than the card's 250 ms lease.  The loop hands
/// over the state text and its boot time; the thread reads the clocks
/// (measuring how long after the snapshot that was) and writes the record.
pub struct Checkpointer {
    tx: Option<std::sync::mpsc::SyncSender<(String, Duration)>>,
    thread: Option<std::thread::JoinHandle<()>>,
}

pub fn boottime() -> Duration {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: a valid, writable timespec.
    unsafe { libc::clock_gettime(libc::CLOCK_BOOTTIME, &mut ts) };
    Duration::new(ts.tv_sec.max(0) as u64, ts.tv_nsec.clamp(0, 999_999_999) as u32)
}

impl Checkpointer {
    pub fn spawn(path: std::path::PathBuf, config: u64, write: fn(&Path, &str) -> std::io::Result<()>) -> Checkpointer {
        let (tx, rx) = std::sync::mpsc::sync_channel::<(String, Duration)>(1);
        let thread = std::thread::Builder::new()
            .name("checkpoint".into())
            .spawn(move || {
                let mut failed = false;
                while let Ok((state, at)) = rx.recv() {
                    let stamp = read_stamp();
                    let pairing = boottime().saturating_sub(at);
                    let text = encode(&Record { clean: false, stamp, config, pairing, state });
                    match write(&path, &text) {
                        Ok(()) => failed = false,
                        Err(e) if !failed => {
                            crate::warn!("{}: {e}; the next boot starts from its own bounds", path.display());
                            failed = true;
                        }
                        Err(_) => {}
                    }
                }
            })
            .ok();
        Checkpointer { tx: thread.as_ref().map(|_| tx), thread }
    }

    /// Hand a snapshot over; dropped if the last one is still being written.
    pub fn offer(&self, state: String, at: Duration) {
        if let Some(tx) = &self.tx {
            let _ = tx.try_send((state, at));
        }
    }

    /// Wait for the thread to finish what it holds (before a clean record).
    pub fn finish(&mut self) {
        self.tx = None;
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for Checkpointer {
    fn drop(&mut self) {
        self.finish();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn neo() -> Thermal {
        Thermal::new(38.0, 3.3, 26.0, 100.0, 50.0)
    }

    /// The bound itself: M* for the Neo, and the saved magnet's approach.
    #[test]
    fn the_magnet_bound_is_the_held_coil_steady_state() {
        let th = neo();
        let m_star = (50.0 + 26.0 / 38.0 * 135.0) / (1.0 + 26.0 / 38.0);
        assert!((magnet_bound(&th, 135.0, 50.0, None, None) - MAGNET_MARGIN_K - m_star).abs() < 1e-9);
        assert!((m_star - 84.53).abs() < 0.01, "{m_star}");
        // from a cold magnet: rises with the time allowed, never past M*
        let mut last = 0.0;
        for d in [0.0, 1.0, 10.0, 30.0, 100.0, 1000.0, 1e6] {
            let m = magnet_bound(&th, 135.0, 50.0, Some(50.0), Some(d));
            assert!(m >= last && m <= m_star + MAGNET_MARGIN_K + 1e-9, "{d}: {m}");
            last = m;
        }
        // a hot saved magnet keeps its own bound (no credit from Y)
        assert!(magnet_bound(&th, 135.0, 50.0, Some(120.0), Some(1e6)) >= 120.0);
    }

    /// Any program at all that keeps the discrete model's coil under the
    /// limit leaves its magnet under the bound, from any magnet, for any
    /// length of time: random bursts, held power, and coil-chasing
    /// bang-bang drives that hold the coil at the limit.
    #[test]
    fn no_program_under_the_coil_limit_beats_the_magnet_bound() {
        use crate::config::Config;
        let cfg = Config::parse(include_str!("../conf/apple/j700.conf")).unwrap();
        let spec = cfg.speakers[0].clone();
        let ta = cfg.globals.t_ambient;
        let th = neo();
        let mut seed = 0x9e3779b97f4a7c15u64;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut worst: f64 = f64::NEG_INFINITY;
        for case in 0..300 {
            let mut s = crate::model::SpeakerState::new(spec.clone(), ta);
            let m0 = ta + rnd() * 40.0;
            s.t_magnet = m0;
            s.t_coil = m0 + rnd() * (spec.t_limit - m0);
            let full = s.worst_case_power(0.0);
            let mut t = 0.0;
            let style = case % 3;
            while t < 600.0 {
                let dt = 0.01;
                let want = match style {
                    0 => if rnd() < 0.3 { full * rnd() } else { 0.0 },
                    1 => full,
                    _ => if rnd() < 0.5 { full } else { full * 0.1 },
                };
                // the most of `want` that keeps the coil under the limit
                let mut p = want;
                for _ in 0..40 {
                    if s.peak_if(p, dt, ta) < spec.t_limit { break; }
                    p *= 0.7;
                }
                if s.peak_if(p, dt, ta) >= spec.t_limit { p = 0.0; }
                s.step(p, dt, ta);
                t += dt;
                let bound = magnet_bound(&th, spec.t_limit, ta, Some(m0), Some(t)) - MAGNET_MARGIN_K;
                worst = worst.max(s.t_magnet - bound);
                assert!(s.t_coil < spec.t_limit);
            }
        }
        assert!(worst < 0.05, "the discrete model beat the continuous bound by {worst} K");
    }

    #[test]
    fn records_round_trip_and_refuse_damage() {
        let stamp = Stamp { boot_id: Some("ab-cd".into()), machine_id: Some("m".into()), rtc: Some(1000), realtime: Some(Duration::from_millis(1_000_400)), synced: true };
        let r = Record { clean: true, stamp: stamp.clone(), config: 7, pairing: Duration::from_millis(3), state: "speakerguardd-state 3\nboottime_ns 1\nend\n".into() };
        let text = encode(&r);
        assert_eq!(decode(&text), Some(r.clone()));
        for n in 0..text.len() {
            assert!(decode(&text[..n]).is_none(), "accepted a prefix of {n} bytes");
        }
        assert!(decode(&text.replacen("kind clean", "kind running", 1)).is_none(), "the checksum let a kind change through");
        assert!(decode(&text.replacen("rtc_s 1000", "rtc_s 900", 1)).is_none());
        let running = Record { clean: false, stamp: Stamp { rtc: None, realtime: None, synced: false, ..stamp }, ..r };
        assert_eq!(decode(&encode(&running)), Some(running));
    }

    fn stamp(boot: &str, rtc: u64, rt_ms: u64, synced: bool) -> Stamp {
        Stamp { boot_id: Some(boot.into()), machine_id: Some("m".into()), rtc: Some(rtc), realtime: Some(Duration::from_millis(rt_ms)), synced }
    }

    #[test]
    fn only_checked_clocks_bound_the_time_since() {
        let rec = |s: Stamp| Record { clean: true, stamp: s, config: 1, pairing: Duration::from_millis(5), state: String::new() };
        let r = rec(stamp("a", 1000, 1_000_500, true));
        let hi = usable(&r, &stamp("b", 1030, 1_030_200, false), 1).unwrap().unwrap();
        assert!(hi >= 30.0 && hi < 33.0, "{hi}");
        // the clock went back: no bound
        assert_eq!(usable(&r, &stamp("b", 990, 990_000, false), 1), Ok(None));
        // RTC and system clock disagree now, or then; unsynchronized then
        assert_eq!(usable(&r, &stamp("b", 1030, 1_090_000, false), 1), Ok(None));
        assert_eq!(usable(&rec(stamp("a", 1000, 1_100_000, true)), &stamp("b", 1030, 1_030_000, false), 1), Ok(None));
        assert_eq!(usable(&rec(stamp("a", 1000, 1_000_000, false)), &stamp("b", 1030, 1_030_000, false), 1), Ok(None));
        // a missing RTC
        let mut no_rtc = stamp("b", 1030, 1_030_000, false);
        no_rtc.rtc = None;
        assert_eq!(usable(&r, &no_rtc, 1), Ok(None));
        // refused outright
        assert!(usable(&r, &stamp("A", 1030, 1_030_000, false), 1).is_err(), "same boot");
        assert!(usable(&r, &stamp("b", 1030, 1_030_000, false), 2).is_err(), "another configuration");
        let mut other = stamp("b", 1030, 1_030_000, false);
        other.machine_id = Some("n".into());
        assert!(usable(&r, &other, 1).is_err(), "another machine");
    }

    #[test]
    fn a_start_is_never_under_any_protected_past() {
        // Every past that kept the coil under 135 C, then `uptime` or
        // more at the safe level, ends under the start bound.
        let th = neo();
        let safe = 7.33f64.powi(2) / 4.2 / 100.0;
        for &uptime in &[1.0, 8.5, 30.0] {
            let (c, m) = start_bound(&th, 135.0, 50.0, safe, uptime, None, None);
            for &(c0, m0) in &[(135.0, 84.5), (135.0, 80.0), (100.0, 84.5), (60.0, 60.0)] {
                for &after in &[uptime, uptime + 1.0, uptime + 60.0, uptime + 3600.0] {
                    let mut t = th.clone();
                    t.t_coil = c0;
                    t.t_magnet = m0;
                    t.hold(safe, after, 50.0);
                    assert!(t.t_coil <= c + 1e-9 && t.t_magnet <= m + 1e-9, "uptime {uptime}, from {c0}/{m0} after {after}: {t:?} over {c}/{m}");
                }
            }
        }
    }

    #[test]
    fn sensors_are_read_from_the_smc_only() {
        let dir = std::env::temp_dir().join(format!("sg-hwmon-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let smc = dir.join("hwmon4");
        let nvme = dir.join("hwmon0");
        std::fs::create_dir_all(&smc).unwrap();
        std::fs::create_dir_all(&nvme).unwrap();
        std::fs::write(smc.join("name"), "macsmc_hwmon\n").unwrap();
        for (i, label, v) in [(1, "Ambient", 29130), (2, "Chassis 3", 46812), (3, "P-cluster Temperature 1", 91000), (4, "Chassis 9", 999000)] {
            std::fs::write(smc.join(format!("temp{i}_label")), format!("{label}\n")).unwrap();
            std::fs::write(smc.join(format!("temp{i}_input")), format!("{v}\n")).unwrap();
        }
        std::fs::write(nvme.join("name"), "nvme\n").unwrap();
        std::fs::write(nvme.join("temp1_label"), "Chassis\n").unwrap();
        std::fs::write(nvme.join("temp1_input"), "70000\n").unwrap();
        assert_eq!(chassis_temperature(&dir), Some(46.812));
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(chassis_temperature(Path::new("/nonexistent-hwmon")), None);
    }
}
