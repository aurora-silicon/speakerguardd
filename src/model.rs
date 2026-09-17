// SPDX-License-Identifier: MIT
//! The protection model: the power delivered to each voice coil is estimated
//! from the words on the speaker wire (feed-forward, there is no current
//! sense), fed into a two-stage lumped thermal model (coil into magnet,
//! magnet into ambient, each a first-order lag), and the coil temperature
//! drives a gain governor with a window below the limit and hysteresis.
//! A windowed power budget (mean power over the last second and the last
//! minute) reduces the gain at once when a burst would exceed what the
//! amplifier/speaker pair is rated for, before the coil has warmed up.

use crate::config::{Globals, Speaker};

/// Thermal state of one driver.
#[derive(Debug, Clone)]
pub struct SpeakerState {
    pub spec: Speaker,
    /// Voice coil and magnet temperatures (°C).
    pub t_coil: f64,
    pub t_magnet: f64,
    /// Power of the last step (W) and its RMS voltage (V), for logging.
    pub power: f64,
    pub v_rms: f64,
    /// Gain reduction applied by the governor (dB, >= 0).
    pub reduction_db: f64,
    reducing: bool,
    /// Power history for the budgets: (seconds, watts) per step, newest last.
    history: std::collections::VecDeque<(f64, f64)>,
    history_s: f64,
}

impl SpeakerState {
    pub fn new(spec: Speaker, t_ambient: f64) -> SpeakerState {
        SpeakerState {
            spec,
            t_coil: t_ambient,
            t_magnet: t_ambient,
            power: 0.0,
            v_rms: 0.0,
            reduction_db: 0.0,
            reducing: false,
            history: std::collections::VecDeque::new(),
            history_s: 0.0,
        }
    }

    /// The RMS voltage a period of sense words puts across the coil.
    pub fn v_rms_of(&self, words: &[i32], channels: usize) -> f64 {
        let ch = self.spec.vs_chan;
        let frames = words.len() / channels;
        if frames == 0 {
            return 0.0;
        }
        let scale = self.spec.vs_scale / 2147483648.0;
        let sum: f64 = words
            .chunks_exact(channels)
            .map(|f| {
                let v = f[ch] as f64 * scale;
                v * v
            })
            .sum();
        (sum / frames as f64).sqrt()
    }

    /// Advance the thermal model by `dt` seconds with `power` watts in the coil.
    pub fn step(&mut self, power: f64, dt: f64, t_ambient: f64) {
        let s = &self.spec;
        self.power = power;
        // keep one minute of history for the budgets
        self.history.push_back((dt, power));
        self.history_s += dt;
        while self.history_s > 60.0 {
            if let Some((old_dt, _)) = self.history.pop_front() {
                self.history_s -= old_dt;
            } else {
                break;
            }
        }
        // magnet: first-order lag toward ambient + P * tr_magnet
        let k_magnet = 1.0 - (-dt / s.tau_magnet).exp();
        let magnet_target = t_ambient + power * s.tr_magnet;
        self.t_magnet += (magnet_target - self.t_magnet) * k_magnet;
        // coil: first-order lag toward magnet + P * tr_coil
        let k_coil = 1.0 - (-dt / s.tau_coil).exp();
        let coil_target = self.t_magnet + power * s.tr_coil;
        self.t_coil += (coil_target - self.t_coil) * k_coil;
    }

    /// Mean power over the last `seconds` (W); time before the first step
    /// counts as silence.
    pub fn mean_power(&self, seconds: f64) -> f64 {
        let mut t = 0.0;
        let mut e = 0.0;
        for (dt, p) in self.history.iter().rev() {
            let take = dt.min(seconds - t);
            if take <= 0.0 {
                break;
            }
            e += p * take;
            t += take;
        }
        e / seconds
    }

    /// The reduction (dB) the power budgets ask for: enough to bring the
    /// windowed mean power back to its limit.
    fn budget_reduction(&self) -> f64 {
        let mut db: f64 = 0.0;
        for (seconds, limit) in [(1.0, self.spec.p_limit_1s), (60.0, self.spec.p_limit_60s)] {
            if limit > 0.0 {
                let mean = self.mean_power(seconds);
                if mean > limit {
                    db = db.max(10.0 * (mean / limit).log10());
                }
            }
        }
        db
    }

    /// Governor: the reduction (dB) that keeps the coil inside the window
    /// and the mean power inside the budgets.  Thermal reduction grows
    /// linearly across the window from 0 dB at `limit - headroom - window`
    /// to `t_reduction_max` at `limit - headroom` (and keeps growing at the
    /// same rate above it), and is released only once the coil is
    /// `hysteresis` below where it started.
    pub fn govern(&mut self, globals: &Globals) -> f64 {
        let ceiling = self.spec.t_limit - self.spec.t_headroom;
        let start = ceiling - globals.t_window;
        if self.t_coil > start {
            self.reducing = true;
        } else if self.t_coil < start - globals.t_hysteresis {
            self.reducing = false;
        }
        let thermal = if self.reducing {
            ((self.t_coil - start) / globals.t_window * globals.t_reduction_max).max(0.0)
        } else {
            0.0
        };
        let wanted = thermal.max(self.budget_reduction());
        // attack at once, release at most 0.5 dB per step so the level does
        // not pump
        self.reduction_db = if wanted >= self.reduction_db { wanted } else { (self.reduction_db - 0.5).max(wanted) };
        self.reduction_db
    }
}

/// The card-wide result of one step: the reduction to apply to the shared
/// volume control is the largest any speaker needs.
pub fn reduction_db(states: &[SpeakerState]) -> f64 {
    states.iter().map(|s| s.reduction_db).fold(0.0, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> Speaker {
        Speaker {
            name: "Left".into(),
            group: 0,
            tr_coil: 60.0,
            tau_coil: 3.0,
            tr_magnet: 40.0,
            tau_magnet: 200.0,
            t_limit: 110.0,
            t_headroom: 10.0,
            z_nominal: 4.43,
            vs_scale: 3.3,
            vs_chan: 0,
            p_limit_1s: 0.0,
            p_limit_60s: 0.0,
        }
    }

    fn globals() -> Globals {
        Globals { sense_pcm: 2, t_ambient: 35.0, t_hysteresis: 5.0, t_window: 20.0, channels: 2, period: 4096, t_reduction_max: 20.0 }
    }

    #[test]
    fn full_scale_sine_is_the_amplifier_rms() {
        let s = SpeakerState::new(spec(), 35.0);
        let n = 4096;
        let words: Vec<i32> = (0..n)
            .flat_map(|i| {
                let v = ((i as f64) * 2.0 * std::f64::consts::PI * 1000.0 / 48000.0).sin();
                [(v * 2147483647.0) as i32, 0]
            })
            .collect();
        let v = s.v_rms_of(&words, 2);
        assert!((v - 3.3 / 2f64.sqrt()).abs() < 0.01, "{v}");
        assert_eq!(s.v_rms_of(&vec![0; 8192], 2), 0.0);
    }

    #[test]
    fn thermal_steady_state_and_time_constants() {
        let mut s = SpeakerState::new(spec(), 35.0);
        // 1 W for a long time: coil = ambient + P * (tr_magnet + tr_coil)
        for _ in 0..(3000 * 10) {
            s.step(1.0, 0.1, 35.0);
        }
        assert!((s.t_magnet - 75.0).abs() < 0.1, "{}", s.t_magnet);
        assert!((s.t_coil - 135.0).abs() < 0.1, "{}", s.t_coil);
        // cooling: after one coil time constant the coil closed ~63% of the gap to the magnet
        let gap0 = s.t_coil - s.t_magnet;
        for _ in 0..30 {
            s.step(0.0, 0.1, 35.0);
        }
        let gap = s.t_coil - s.t_magnet;
        assert!((gap / gap0 - (-1f64).exp()).abs() < 0.05, "{}", gap / gap0);
    }

    #[test]
    fn ceiling_power_never_governs() {
        // -20 dBFS on the wire at 3.3 V full scale into 4.43 ohm: ~2.5 mW sine
        let mut s = SpeakerState::new(spec(), 35.0);
        let v = 3.3 * 0.1 / 2f64.sqrt();
        let p = v * v / 4.43;
        for _ in 0..(3600 * 10) {
            s.step(p, 0.1, 35.0);
            assert_eq!(s.govern(&globals()), 0.0);
        }
        // 12 mW * (60 + 40) K/W = 1.2 K above ambient
        assert!(s.t_coil < 37.0, "{}", s.t_coil);
    }

    #[test]
    fn governor_window_and_hysteresis() {
        let mut s = SpeakerState::new(spec(), 35.0);
        let g = globals(); // t_reduction_max 20 dB at the working limit
        // heat the coil into the window: limit 110 - headroom 10 = 100, window starts at 80
        s.t_coil = 90.0;
        s.t_magnet = 60.0;
        let r = s.govern(&g);
        assert!((r - 10.0).abs() < 1e-9, "{r}");
        s.t_coil = 100.0;
        assert_eq!(s.govern(&g), 20.0);
        // beyond the working limit the reduction keeps growing at the same rate
        s.t_coil = 110.0;
        assert_eq!(s.govern(&g), 30.0);
        // back inside the window but above start - hysteresis: still reducing
        s.t_coil = 78.0;
        let r = s.govern(&g);
        assert!(r > 0.0 && r < 30.0, "{r}");
        // released only below start - hysteresis, 0.5 dB per step
        s.t_coil = 70.0;
        let mut last = r;
        for _ in 0..80 {
            let now = s.govern(&g);
            assert!(now <= last && last - now <= 0.5 + 1e-9);
            last = now;
        }
        assert_eq!(last, 0.0);
    }

    #[test]
    fn power_budget_reduces_before_the_coil_warms() {
        // the J700's own numbers
        let mut spec = spec();
        spec.tr_coil = 38.0;
        spec.tau_coil = 3.3;
        spec.tr_magnet = 26.0;
        spec.tau_magnet = 100.0;
        spec.t_limit = 135.0;
        spec.t_headroom = 15.0;
        spec.p_limit_1s = 2.8154;
        spec.p_limit_60s = 2.5;
        let mut s = SpeakerState::new(spec, 35.0);
        let g = globals();
        // a 6.3 W burst (a full-scale sine into 4.2 ohm at 7.3 V): after 0.5 s
        // the 1 s mean is 3.15 W, over the budget, while the coil (69 C) is
        // still well below the thermal window (100 C)
        for _ in 0..5 {
            s.step(6.3, 0.1, 35.0);
        }
        assert!(s.t_coil < 75.0, "{}", s.t_coil);
        let r = s.govern(&g);
        assert!((r - 10.0 * (3.15f64 / 2.8154).log10()).abs() < 0.05, "{r}");
        // silence: the mean falls back under the budget and the reduction releases
        for _ in 0..20 {
            s.step(0.0, 0.1, 35.0);
            s.govern(&g);
        }
        assert_eq!(s.govern(&g), 0.0);
        // the minute budget: 2.6 W for a minute ends up over 2.5 W
        for _ in 0..600 {
            s.step(2.6, 0.1, 35.0);
        }
        assert!((s.mean_power(60.0) - 2.6).abs() < 0.01);
        assert!(s.budget_reduction() > 0.1);
    }
}
