//! Single-resonator compensation, from the measured transfer specified in
//! `docs/RESONATOR-COMPONENT.md`. The caller supplies every model parameter.
//!
//! This remains a component rather than a [`crate::ProtectionUnit`]. LSM2
//! owns its placement before the nonlinear controller, target selection and
//! atomic model-generation updates; the file renderer uses it directly.

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
    fn coefficients(p: Params) -> Result<([f64; 3], [f64; 2]), Error> {
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
        Ok((b, a))
    }

    pub fn new(p: Params) -> Result<Self, Error> {
        let (b, a) = Self::coefficients(p)?;
        Ok(Self { b, a, x: [0.0; 2], y: [0.0; 2], faulted: false })
    }

    /// Install a new validated physical model without discontinuing the
    /// input/output delay history. An explicit [`Self::reset`] is the only
    /// operation that clears history or the failure latch.
    pub fn reconfigure(&mut self, p: Params) -> Result<(), Error> {
        let (b, a) = Self::coefficients(p)?;
        self.b = b;
        self.a = a;
        Ok(())
    }

    pub(crate) fn process_sample(&mut self, sample: f32) -> Result<f32, Error> {
        if self.faulted {
            return Err(Error::Faulted);
        }
        if !sample.is_finite() {
            self.faulted = true;
            return Err(Error::NonfiniteInput);
        }
        let x = sample as f64;
        let y = self.b[0] * x + self.b[1] * self.x[0] + self.b[2] * self.x[1]
            - self.a[0] * self.y[0]
            - self.a[1] * self.y[1];
        let out = y as f32;
        if !y.is_finite() || !out.is_finite() {
            self.faulted = true;
            return Err(Error::NonfiniteState);
        }
        self.x = [x, self.x[0]];
        self.y = [y, self.y[0]];
        Ok(out)
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
            match self.process_sample(samples[i]) {
                Ok(out) => samples[i] = out,
                Err(e) => {
                    samples.fill(0.0);
                    return Err(e);
                }
            }
        }
        Ok(())
    }

    /// Reset history and the failure latch. The currently validated
    /// coefficients stay unchanged.
    pub fn reset(&mut self) {
        self.x = [0.0; 2];
        self.y = [0.0; 2];
        self.faulted = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(hz: f64) -> Params {
        Params {
            rdc: 4.0,
            bl: 0.7,
            mass_kg: 0.0003,
            damping: 0.2,
            stiffness: 1500.0,
            desired_hz: hz,
            desired_q: 0.8,
            max_boost_db: 12.0,
            max_cut_db: 12.0,
            rate: 48_000.0,
        }
    }

    #[test]
    fn reconfigure_retains_history_and_fault_until_reset() {
        let mut retained = Filter::new(params(300.0)).unwrap();
        let mut prefix = [0.0; 64];
        prefix[63] = 0.25;
        retained.process(&mut prefix).unwrap();
        retained.reconfigure(params(500.0)).unwrap();

        let mut fresh = Filter::new(params(500.0)).unwrap();
        let mut a = [0.0; 16];
        let mut b = a;
        retained.process(&mut a).unwrap();
        fresh.process(&mut b).unwrap();
        assert_ne!(a, b, "the retained old-model tail must survive a coefficient update");

        let mut faulted = Filter::new(params(300.0)).unwrap();
        assert_eq!(faulted.process(&mut [f32::NAN]), Err(Error::NonfiniteInput));
        faulted.reconfigure(params(500.0)).unwrap();
        assert_eq!(faulted.process(&mut [0.0]), Err(Error::Faulted));
        faulted.reset();
        assert_eq!(faulted.process(&mut [0.0]), Ok(()));
    }
}
