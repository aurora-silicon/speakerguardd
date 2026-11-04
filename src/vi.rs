// SPDX-License-Identifier: MIT
// Copyright The Asahi Linux Contributors
// Copyright The Aurora Silicon Contributors
//! Measured V/I protection. Policy and arithmetic follow speakersafetyd 2.0.1
//! (MIT, 63a92658a4daec170c6472e38162374206e593fb, src/types.rs).
//! No voltage-power estimate, minimum impedance, prediction or energy budget
//! exists in this path. Invalid capture freezes history, caps, and starts the
//! next admission at the hard temperature on BOTH nodes. Epoch heartbeats
//! make the last grant atomic with the kernel's PM cap.
use crate::{
    config::{Config, Speaker},
    debug,
    guard::{transient, Hw},
    info, warn,
};
use std::{
    error::Error,
    time::{Duration, Instant},
};

#[derive(Clone, Debug)]
pub struct Model {
    pub coil: f64,
    pub magnet: f64,
    coil_hold: f32,
    magnet_hold: f32,
    pub gain: f32,
    min_gain: f32,
    spec: Speaker,
}
impl Model {
    fn new(s: &Speaker, cfg: &Config) -> Self {
        let g = &cfg.globals;
        let max_power =
            (s.t_limit as f32 - g.t_ambient as f32) / (s.tr_coil as f32 + s.tr_magnet as f32);
        let peak_power = 2.0 * 10f32.powf(g.amp_gain_dbv as f32 / 10.0) / s.z_nominal as f32;
        let mut m = Self {
            coil: 0.0,
            magnet: 0.0,
            coil_hold: 0.0,
            magnet_hold: 0.0,
            gain: 0.0,
            min_gain: (10.0 * (max_power / peak_power).log10()).min(0.0),
            spec: s.clone(),
        };
        m.assume_hot();
        m
    }
    fn assume_hot(&mut self) {
        let hard = self.spec.t_limit + self.spec.t_headroom;
        self.coil = self.coil.max(hard);
        self.magnet = self.magnet.max(hard);
        self.coil_hold = self.coil_hold.max(hard as f32);
        self.magnet_hold = self.magnet_hold.max(hard as f32);
    }
    /// Exact 2.0.1 sample arithmetic, including f32 conversion/coefficients.
    /// Run on a copy: no speaker's state or output is committed until all
    /// outputs and the complete period validate.
    fn period(&mut self, cfg: &Config, words: &[i32], rate: u32) -> Result<(), &'static str> {
        let s = &self.spec;
        let step = 1f32 / rate as f32;
        let ac = (step / (s.tau_coil as f32 + step)) as f64;
        let am = (step / (s.tau_magnet as f32 + step)) as f64;
        let hard = (s.t_limit as f32 + s.t_headroom as f32) as f64;
        let mut power_sum = 0f32;
        for frame in words.chunks_exact(cfg.globals.channels) {
            let v = (frame[s.vs_chan] >> 16) as f32 / 32768.0 * s.vs_scale as f32;
            let i = (frame[s.is_chan.ok_or("missing current sense")?] >> 16) as f32 / 32768.0
                * s.is_scale as f32;
            let power = v * i;
            let tc = self.magnet + (power * s.tr_coil as f32) as f64;
            let tm = (cfg.globals.t_ambient as f32 + power * s.tr_magnet as f32) as f64;
            self.coil = tc * ac + self.coil * (1.0 - ac);
            self.magnet = tm * am + self.magnet * (1.0 - am);
            if !self.coil.is_finite()
                || !self.magnet.is_finite()
                || self.coil > hard
                || self.magnet > hard
            {
                return Err("thermal hard limit");
            }
            power_sum += power;
        }
        let mean = power_sum / (words.len() / cfg.globals.channels) as f32;
        if !mean.is_finite() || mean < -0.01 {
            return Err("invalid V/I power");
        }
        self.coil_hold = self
            .coil_hold
            .max(self.coil as f32)
            .min(self.coil as f32 + cfg.globals.t_hysteresis as f32);
        self.magnet_hold = self
            .magnet_hold
            .max(self.magnet as f32)
            .min(self.magnet as f32 + cfg.globals.t_hysteresis as f32);
        let temperature = self.coil_hold.max(self.magnet_hold);
        let start = s.t_limit as f32 - cfg.globals.t_window as f32;
        self.gain = self.min_gain * ((temperature - start) / cfg.globals.t_window as f32).max(0.0);
        if self.gain > -0.01 {
            self.gain = 0.0;
        }
        if !self.gain.is_finite() {
            return Err("invalid thermal gain");
        }
        Ok(())
    }
}

pub struct ViGuard {
    cfg: Config,
    pub models: Vec<Model>,
    words: Vec<i32>,
    epoch: Option<u32>,
    rate: u32,
    open: bool,
    pub armed: bool,
    pub fresh: u32,
    last_data: Option<Instant>,
}
impl ViGuard {
    pub fn start<H: Hw>(hw: &mut H, cfg: &Config) -> Result<Self, Box<dyn Error>> {
        cfg.check_governor()?;
        let epoch = hw
            .protection_epoch()?
            .ok_or("measured_vi needs Speaker Protection Epoch; boot the paired kernel kit")?;
        if epoch == 0 || !hw.events_available() {
            return Err("measured_vi needs valid epoch and rate events".into());
        }
        hw.take_lock()?;
        hw.set_min()?;
        info!("measured V/I only: FF=off budgets=off impedance-gate=off; capped admission");
        Ok(Self {
            cfg: cfg.clone(),
            models: cfg.speakers.iter().map(|s| Model::new(s, cfg)).collect(),
            words: vec![0; cfg.globals.period * cfg.globals.channels],
            epoch: Some(epoch),
            rate: 0,
            open: false,
            armed: false,
            fresh: 0,
            last_data: None,
        })
    }
    fn cap<H: Hw>(&mut self, hw: &mut H, reason: &str) -> Result<(), Box<dyn Error>> {
        // Always revoke, even if a final heartbeat returned an ambiguous error.
        hw.relinquish()?;
        hw.set_min()?;
        hw.close_sense();
        self.armed = false;
        self.open = false;
        self.fresh = 0;
        self.last_data = None;
        self.epoch = None;
        for m in &mut self.models {
            m.assume_hot();
        }
        warn!("measured V/I capped: {reason}; worst-case thermal admission");
        Ok(())
    }
    pub fn stop<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        self.cap(hw, "stop")
    }
    pub fn tick<H: Hw>(&mut self, hw: &mut H) -> Result<(), Box<dyn Error>> {
        let epoch = match hw.protection_epoch() {
            Ok(Some(e)) if e > 0 => e,
            _ => {
                self.cap(hw, "epoch unavailable")?;
                hw.sleep(Duration::from_millis(50));
                return Ok(());
            }
        };
        if self.epoch.is_some_and(|e| e != epoch) {
            self.cap(hw, "PM epoch changed")?;
        }
        let rate = match hw.sample_rate() {
            Ok(r) if (0..=192000).contains(&r) => r as u32,
            _ => {
                self.cap(hw, "sample rate unavailable")?;
                hw.sleep(Duration::from_millis(50));
                return Ok(());
            }
        };
        let changed = hw.rate_events();
        if self.open && (self.rate != rate || changed) {
            self.cap(hw, "rate transition")?;
        }
        if rate == 0 {
            if self.armed || self.open {
                self.cap(hw, "speakers closed")?;
            }
            hw.wait_rate_change(Duration::from_millis(50));
            return Ok(());
        }
        if !self.open {
            // Token is sampled BEFORE opening a fresh capture; a PM boundary
            // during open, read, model validation, write or ping invalidates it.
            self.epoch = Some(epoch);
            self.rate = rate;
            match hw.open_sense(rate) {
                Ok(()) => self.open = true,
                Err(_) => {
                    self.cap(hw, "capture open")?;
                    hw.sleep(Duration::from_millis(50));
                    return Ok(());
                }
            }
        }
        match hw.read_period(&mut self.words) {
            Ok(Some(true)) => {}
            Ok(Some(false)) => {
                self.cap(hw, "short or partial period")?;
                return Ok(());
            }
            Ok(None) => {
                let bound = Duration::from_secs_f64(self.cfg.globals.period as f64 / rate as f64)
                    + Duration::from_millis(10);
                if self
                    .last_data
                    .is_some_and(|t| hw.now().saturating_duration_since(t) > bound)
                {
                    self.cap(hw, "sense overdue")?;
                }
                return Ok(());
            }
            Err(e) if transient(&e) => {
                self.cap(hw, "capture interrupted")?;
                return Ok(());
            }
            Err(e) => {
                self.cap(hw, "capture failure")?;
                return Err(e.into());
            }
        }
        if hw.protection_epoch().ok().flatten() != self.epoch
            || hw.sample_rate().ok() != Some(rate as i64)
            || hw.rate_events()
        {
            self.cap(hw, "period crossed a PM/rate boundary")?;
            return Ok(());
        }
        let queued = match hw.queued() {
            Ok(q) => q,
            Err(_) => {
                self.cap(hw, "capture position")?;
                return Ok(());
            }
        };
        let mut next = self.models.clone();
        for m in &mut next {
            if let Err(why) = m.period(&self.cfg, &self.words, rate) {
                self.cap(hw, why)?;
                return Ok(());
            }
        }
        self.models = next;
        self.fresh = self.fresh.saturating_add(1);
        self.last_data = Some(hw.now());
        // Model queued complete periods too, but never grant on a backlog.
        if self.fresh < 2 || queued >= self.cfg.globals.period as u64 {
            return Ok(());
        }
        let own: Vec<f64> = self.models.iter().map(|m| -(m.gain as f64)).collect();
        let reductions = crate::engine::link(&self.cfg, &own);
        if !self.armed {
            hw.set_min()?;
        }
        // The kernel compares this token and grants the lease in one critical
        // section with PM prepare. No clock tolerance and no check/ping race.
        match hw.ping_epoch(self.epoch.unwrap()) {
            Ok(()) => {}
            Err(e) if transient(&e) || e.raw_os_error() == Some(libc::ESTALE) => {
                self.cap(hw, "epoch heartbeat rejected")?;
                return Ok(());
            }
            Err(e) => {
                self.cap(hw, "heartbeat failure")?;
                return Err(e.into());
            }
        }
        if !self.armed {
            info!(
                "measured V/I admission accepted: epoch={} rate={} complete_periods={}",
                self.epoch.unwrap(),
                rate,
                self.fresh
            );
        }
        self.armed = true;
        let applied = match hw.set_reductions(&reductions) {
            Ok(a) => a,
            Err(_) => {
                self.cap(hw, "volume write")?;
                return Ok(());
            }
        };
        if applied.len() != reductions.len()
            || applied
                .iter()
                .zip(&reductions)
                .any(|(a, r)| !a.is_finite() || a + 1e-9 < *r)
        {
            self.cap(hw, "insufficient applied attenuation")?;
            return Ok(());
        }
        for (s, m) in self.cfg.speakers.iter().zip(&self.models) {
            debug!(
                "measured_vi {} coil={:.3} magnet={:.3} gain={:.3}",
                s.name, m.coil, m.magnet, m.gain
            );
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::guard::fake::FakeHw;
    fn config() -> Config {
        let mut c = Config::parse(include_str!("../conf/apple/j613.conf")).unwrap();
        c.allow_unqualified = true;
        c
    }
    fn fixture() -> (FakeHw, ViGuard) {
        let c = config();
        let mut hw = FakeHw::per_speaker(&c);
        hw.level = Box::new(|_| 0.001);
        let g = ViGuard::start(&mut hw, &c).unwrap();
        (hw, g)
    }
    fn advance(hw: &mut FakeHw, g: &mut ViGuard, n: usize) {
        for _ in 0..n {
            g.tick(hw).unwrap();
        }
    }
    #[test]
    fn vi_only_startup_requires_two_complete_stable_periods() {
        let (mut h, mut g) = fixture();
        assert!(!g.armed && h.pings.is_empty());
        while h.delivered < 1 {
            g.tick(&mut h).unwrap();
        }
        assert!(!g.armed && h.pings.is_empty());
        advance(&mut h, &mut g, 6);
        assert!(g.armed);
    }
    #[test]
    fn vi_only_idle_never_unlocks() {
        let (mut h, mut g) = fixture();
        h.playing = Box::new(|_| false);
        advance(&mut h, &mut g, 10);
        assert!(!g.armed && h.pings.is_empty());
    }
    #[test]
    fn vi_only_short_and_estrpipe_cap_and_assume_hot() {
        for short in [true, false] {
            let (mut h, mut g) = fixture();
            advance(&mut h, &mut g, 8);
            assert!(g.armed);
            if short {
                h.capture_short = true;
            } else {
                h.capture_errno = Some(libc::ESTRPIPE);
            }
            g.tick(&mut h).unwrap();
            assert!(!g.armed && g.fresh == 0);
            assert!(g
                .models
                .iter()
                .all(|m| m.coil >= 150.0 && m.magnet >= 150.0));
            advance(&mut h, &mut g, 8);
            assert!(g.armed);
        }
    }
    #[test]
    fn vi_only_short_sleep_epoch_needs_new_capture_and_hot_admission() {
        let (mut h, mut g) = fixture();
        advance(&mut h, &mut g, 8);
        assert!(g.armed);
        h.protection_epoch += 1; // even a 1 ns sleep; no userspace clock tolerance
        h.capture_errno = Some(libc::ESTRPIPE);
        g.tick(&mut h).unwrap();
        assert!(!g.armed && g.fresh == 0);
        assert!(g.models.iter().all(|m| m.coil >= 150.0));
        advance(&mut h, &mut g, 8);
        assert!(g.armed);
    }
    #[test]
    fn vi_only_suspend_inside_final_heartbeat_cannot_unlock() {
        let (mut h, mut g) = fixture();
        h.suspend_on_ping = true;
        advance(&mut h, &mut g, 4);
        assert!(h.pings.is_empty(), "stale epoch reached the unlock");
        assert!(!g.armed && g.fresh == 0);
        assert!(g.models.iter().all(|m| m.coil >= 150.0));
        advance(&mut h, &mut g, 8);
        assert!(g.armed);
    }
    #[test]
    fn vi_only_rate_change_reopens_before_readmission() {
        let (mut h, mut g) = fixture();
        advance(&mut h, &mut g, 8);
        assert!(g.armed);
        h.rate_value = Some(44100);
        h.capture_errno = Some(libc::EAGAIN);
        g.tick(&mut h).unwrap();
        assert!(!g.armed && g.fresh == 0 && g.rate == 44100);
    }
    #[test]
    fn vi_only_invalid_last_speaker_prevents_any_unlock() {
        let (mut h, mut g) = fixture();
        // Negative mean power only on the last physical output.
        h.tweak = Some(Box::new(|_, w| {
            for f in w.chunks_exact_mut(8) {
                f[6] = 1 << 29;
                f[7] = -(1 << 29);
            }
        }));
        advance(&mut h, &mut g, 10);
        assert!(!g.armed && h.pings.is_empty());
    }
    #[test]
    fn vi_only_never_reads_inactive_ff_or_impedance_policy() {
        let mut c = config();
        c.globals.horizon_s = f64::NAN;
        c.globals.t_reduction_max = f64::NAN;
        c.globals.max_reduction = f64::NAN;
        c.globals.t_ctrl_headroom = f64::NAN;
        for s in &mut c.speakers {
            s.r_ff = f64::NAN;
            s.vi_tr_coil = f64::NAN;
        }
        let mut h = FakeHw::per_speaker(&c);
        h.level = Box::new(|_| 0.001);
        let mut g = ViGuard::start(&mut h, &c).unwrap();
        advance(&mut h, &mut g, 8);
        assert!(g.armed);
    }
    #[test]
    fn vi_only_matches_201_f32_measured_model_and_ramp() {
        let c = config();
        for spec in &c.speakers {
            let mut m = Model::new(spec, &c);
            m.coil = 125.0;
            m.magnet = 90.0;
            m.coil_hold = 125.0;
            m.magnet_hold = 90.0;
            let mut tc = m.coil;
            let mut tm = m.magnet;
            let mut hc = m.coil_hold;
            let mut hm = m.magnet_hold;
            let rate = 48000u32;
            let dt = 1f32 / rate as f32;
            let ac = (dt / (spec.tau_coil as f32 + dt)) as f64;
            let am = (dt / (spec.tau_magnet as f32 + dt)) as f64;
            for amplitude in [100i16, 500, 1000, 2000, 3000, 1000, 0] {
                let mut words = vec![0; 8 * c.globals.period];
                for f in words.chunks_exact_mut(8) {
                    f[spec.vs_chan] = i32::from(amplitude) << 16;
                    f[spec.is_chan.unwrap()] = i32::from(amplitude) << 16;
                }
                // Independent scalar transcription of the 2.0.1 policy.
                let v = amplitude as f32 / 32768.0 * spec.vs_scale as f32;
                let i = amplitude as f32 / 32768.0 * spec.is_scale as f32;
                for _ in 0..c.globals.period {
                    let drive_c = tm + (v * i * spec.tr_coil as f32) as f64;
                    let drive_m =
                        (c.globals.t_ambient as f32 + v * i * spec.tr_magnet as f32) as f64;
                    tc = drive_c * ac + tc * (1.0 - ac);
                    tm = drive_m * am + tm * (1.0 - am);
                }
                hc = hc
                    .max(tc as f32)
                    .min(tc as f32 + c.globals.t_hysteresis as f32);
                hm = hm
                    .max(tm as f32)
                    .min(tm as f32 + c.globals.t_hysteresis as f32);
                let max_p = (spec.t_limit as f32 - c.globals.t_ambient as f32)
                    / (spec.tr_coil as f32 + spec.tr_magnet as f32);
                let peak_p =
                    10f32.powf(c.globals.amp_gain_dbv as f32 / 10.0) / spec.z_nominal as f32 * 2.0;
                let min_gain = (10.0 * (max_p / peak_p).log10()).min(0.0);
                let mut gain = min_gain
                    * ((hc.max(hm) - (spec.t_limit as f32 - c.globals.t_window as f32))
                        / c.globals.t_window as f32)
                        .max(0.0);
                if gain > -0.01 {
                    gain = 0.0;
                }
                m.period(&c, &words, rate).unwrap();
                assert_eq!((m.coil, m.magnet, m.gain), (tc, tm, gain));
            }
        }
    }
}

#[cfg(test)]
mod boundaries {
    use super::*;
    #[test]
    fn vi_only_hard_limit_and_window_keep_201_meaning() {
        let c = Config::parse(include_str!("../conf/apple/j613.conf")).unwrap();
        for s in &c.speakers {
            let mut m = Model::new(s, &c);
            let words = vec![0; c.globals.period * c.globals.channels];
            // The original ramp ends at 140, the hard fault is 150 (not
            // the predictive adapter's 130/140 interpretation).
            m.coil = 140.0;
            m.magnet = 140.0;
            m.coil_hold = 140.0;
            m.magnet_hold = 140.0;
            m.period(&c, &words, 48000).unwrap();
            assert!(m.gain < 0.0 && m.coil < 150.0);
            m.coil = 151.0;
            m.magnet = 151.0;
            assert!(m.period(&c, &words, 48000).is_err());
        }
    }
}
