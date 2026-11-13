//! Single-resonator compensation, from the measured transfer specified in
//! `docs/RESONATOR-COMPONENT.md`. The caller supplies every model parameter.
//!
//! This is a component, not a [`crate::ProtectionUnit`]. LSM2 runs it ahead
//! of its models and limiter on a channel whose tuning requests it
//! (`CompensateResonators` and the type's acceleration-manager block with
//! `UseCompensation`), see `docs/RESONATOR-COMPONENT.md`.

/// Runtime physical model and requested compensation. SI units, except for
/// frequency (Hz), sample rate (Hz), and the two nonnegative limits (dB).
#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub rdc: f64,
    pub bl: f64,
    pub mass_kg: f64,
    pub damping: f64,
    pub stiffness: f64,
    pub desired_hz: f64,
    pub desired_q: f64,
    pub max_boost_db: f64,
    pub max_cut_db: f64,
    pub rate: f64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    InvalidParameters,
    OutsideValidatedDomain,
    UnstableFilter,
    NonfiniteInput,
    NonfiniteState,
    Faulted,
}

/// A direct-form-I biquad with independent history per instance. No process
/// call allocates, logs, blocks, or opens a device.
#[derive(Clone, Debug)]
pub struct Filter {
    b: [f64; 3],
    a: [f64; 2],
    x: [f64; 2],
    y: [f64; 2],
    faulted: bool,
}

impl Filter {
    pub fn new(p: Params) -> Result<Self, Error> {
        let positive = [p.rdc, p.mass_kg, p.damping, p.stiffness, p.desired_hz, p.desired_q, p.rate];
        let nonnegative = [p.bl, p.max_boost_db, p.max_cut_db];
        if !positive.iter().all(|v| v.is_finite() && *v > 0.0)
            || !nonnegative.iter().all(|v| v.is_finite() && *v >= 0.0)
        {
            return Err(Error::InvalidParameters);
        }
        let w0 = libm::sqrt(p.stiffness / p.mass_kg);
        let d0 = (p.damping + p.bl * p.bl / p.rdc) / p.mass_kg;
        let w1 = (2.0 * core::f64::consts::PI * p.desired_hz).max(w0 * libm::pow(10.0, -p.max_boost_db / 40.0));
        if !w0.is_finite() || !w1.is_finite() || w0.max(w1) >= core::f64::consts::PI * p.rate {
            return Err(Error::OutsideValidatedDomain);
        }
        let cut = d0 * libm::pow(10.0, p.max_cut_db / 20.0);
        let delta = (w1 * w1 - w0 * w0) / w0;
        let radicand = cut * cut - delta * delta;
        if !radicand.is_finite() || radicand <= 0.0 {
            return Err(Error::OutsideValidatedDomain);
        }
        let d1 = (w1 / p.desired_q).min(libm::sqrt(radicand));
        let c = 2.0 * p.rate;
        let norm = c * c + d1 * c + w1 * w1;
        let b = [(c * c + d0 * c + w0 * w0) / norm, 2.0 * (w0 * w0 - c * c) / norm, (c * c - d0 * c + w0 * w0) / norm];
        let a = [2.0 * (w1 * w1 - c * c) / norm, (c * c - d1 * c + w1 * w1) / norm];
        // Jury's strict second-order stability conditions. Check the rounded
        // coefficients, not just the continuous-time positive damping.
        if !b.iter().chain(&a).all(|v| v.is_finite())
            || 1.0 + a[0] + a[1] <= 0.0
            || 1.0 - a[0] + a[1] <= 0.0
            || 1.0 - a[1] <= 0.0
        {
            return Err(Error::UnstableFilter);
        }
        Ok(Self { b, a, x: [0.0; 2], y: [0.0; 2], faulted: false })
    }

    /// Filter a mono block in place. On any failure the entire supplied
    /// block is zeroed and later calls remain closed until an explicit reset.
    /// There is no bypass or substitution of the uncompensated signal.
    pub fn process(&mut self, samples: &mut [f32]) -> Result<(), Error> {
        if self.faulted {
            samples.fill(0.0);
            return Err(Error::Faulted);
        }
        if !samples.iter().all(|v| v.is_finite()) {
            samples.fill(0.0);
            self.faulted = true;
            return Err(Error::NonfiniteInput);
        }
        for i in 0..samples.len() {
            let x = samples[i] as f64;
            let y = self.b[0] * x + self.b[1] * self.x[0] + self.b[2] * self.x[1]
                - self.a[0] * self.y[0]
                - self.a[1] * self.y[1];
            let out = y as f32;
            if !y.is_finite() || !out.is_finite() {
                samples.fill(0.0);
                self.faulted = true;
                return Err(Error::NonfiniteState);
            }
            self.x = [x, self.x[0]];
            self.y = [y, self.y[0]];
            samples[i] = out;
        }
        Ok(())
    }

    /// Reset history and the failure latch. The validated coefficients stay
    /// unchanged; a different model requires construction of a new filter.
    pub fn reset(&mut self) {
        self.x = [0.0; 2];
        self.y = [0.0; 2];
        self.faulted = false;
    }
}
