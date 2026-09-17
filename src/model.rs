// SPDX-License-Identifier: MIT
//! The protection model: the power delivered to each voice coil is estimated
//! from the words on the speaker wire (feed-forward, there is no current
//! sense), fed into a two-stage lumped thermal model (coil into magnet,
//! magnet into ambient, each a first-order lag), and the coil temperature
//! drives a gain governor with a window below the limit and hysteresis.

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
}

impl SpeakerState {
    pub fn new(spec: Speaker, t_ambient: f64) -> SpeakerState {
        SpeakerState { spec, t_coil: t_ambient, t_magnet: t_ambient, power: 0.0, v_rms: 0.0, reduction_db: 0.0, reducing: false }
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
        // magnet: first-order lag toward ambient + P * tr_magnet
        let k_magnet = 1.0 - (-dt / s.tau_magnet).exp();
        let magnet_target = t_ambient + power * s.tr_magnet;
        self.t_magnet += (magnet_target - self.t_magnet) * k_magnet;
        // coil: first-order lag toward magnet + P * tr_coil
        let k_coil = 1.0 - (-dt / s.tau_coil).exp();
        let coil_target = self.t_magnet + power * s.tr_coil;
        self.t_coil += (coil_target - self.t_coil) * k_coil;
    }

    /// Governor: the reduction (dB) that keeps the coil inside the window.
    /// Reduction grows linearly across the window from 0 dB at
    /// `limit - headroom - window` to `max_db` at `limit - headroom`, and is
    /// released only once the coil is `hysteresis` below where it started.
    pub fn govern(&mut self, globals: &Globals, max_db: f64) -> f64 {
        let ceiling = self.spec.t_limit - self.spec.t_headroom;
        let start = ceiling - globals.t_window;
        if self.t_coil > start {
            self.reducing = true;
        } else if self.t_coil < start - globals.t_hysteresis {
            self.reducing = false;
        }
        let wanted = if self.reducing {
            ((self.t_coil - start) / globals.t_window * max_db).clamp(0.0, max_db)
        } else {
            0.0
        };
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
        }
    }

    fn globals() -> Globals {
        Globals { sense_pcm: 2, t_ambient: 35.0, t_hysteresis: 5.0, t_window: 20.0, channels: 2, period: 4096 }
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
            assert_eq!(s.govern(&globals(), 7.0), 0.0);
        }
        // 12 mW * (60 + 40) K/W = 1.2 K above ambient
        assert!(s.t_coil < 37.0, "{}", s.t_coil);
    }

    #[test]
    fn governor_window_and_hysteresis() {
        let mut s = SpeakerState::new(spec(), 35.0);
        let g = globals();
        // heat the coil into the window: limit 110 - headroom 10 = 100, window starts at 80
        s.t_coil = 90.0;
        s.t_magnet = 60.0;
        let r = s.govern(&g, 7.0);
        assert!((r - 3.5).abs() < 1e-9, "{r}");
        s.t_coil = 100.0;
        assert_eq!(s.govern(&g, 7.0), 7.0);
        s.t_coil = 200.0;
        assert_eq!(s.govern(&g, 7.0), 7.0);
        // back inside the window but above start - hysteresis: still reducing
        s.t_coil = 78.0;
        let r = s.govern(&g, 7.0);
        assert!(r > 0.0 && r < 7.0, "{r}");
        // released only below start - hysteresis, 0.5 dB per step
        s.t_coil = 70.0;
        let mut last = r;
        for _ in 0..20 {
            let now = s.govern(&g, 7.0);
            assert!(now <= last && last - now <= 0.5 + 1e-9);
            last = now;
        }
        assert_eq!(last, 0.0);
    }
}
