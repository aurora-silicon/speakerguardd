//! `lsm2`: loudspeaker electro-mechanical and two-node thermal model, an
//! optional system-ID estimator, and a controller with independently
//! enabled sample-peak, displacement, velocity (mask-aware), power (four
//! time scales) and temperature axes, combined by minimum linear gain, a
//! five-band filterbank plus a broadband gain path. Targets: the MacBook
//! Neo (static model, no V/I, every axis on) and the MacBook Air M4/M5
//! (V/I, system ID and closed-loop temperature).
//!
//! TODO(SPEC): every block above, its state, timing paths and the
//! behaviour when an estimator, or both, is unavailable.

use crate::{FailSafe, Health, Io, ProtectionUnit, Telemetry, UnitKind};

/// The unit's parameter set, decoded from the compiled blob.
/// TODO(SPEC): fields, in the order and units the specification fixes.
#[derive(Clone, Debug, Default)]
pub struct Params {
    pub channels: u32,
    pub sample_rate: u32,
}

/// Until the behaviour is implemented this unit is the fail-safe.
pub struct Unit {
    params: Params,
    failsafe: FailSafe,
}

impl Unit {
    pub fn new(params: Params) -> Self {
        Self { params, failsafe: FailSafe::new(UnitKind::Lsm2, Health::NO_PARAMS) }
    }
    pub fn params(&self) -> &Params {
        &self.params
    }
}

impl ProtectionUnit for Unit {
    fn kind(&self) -> UnitKind {
        UnitKind::Lsm2
    }
    fn latency_frames(&self) -> u32 {
        // TODO(SPEC): the unit's algorithmic latency from its parameters.
        0
    }
    fn process(&mut self, io: Io<'_, '_>) -> Health {
        // TODO(SPEC): the unit's behaviour. Fail-safe until then.
        self.failsafe.process(io)
    }
    fn telemetry(&self, out: &mut Telemetry) {
        self.failsafe.telemetry(out)
    }
    fn reset(&mut self) {}
}
