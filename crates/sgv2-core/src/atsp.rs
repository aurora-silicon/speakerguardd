//! `atsp`: audio-only two-node coil/magnet thermal model with a temperature
//! limit, hard-limit headroom and attack/release. Target: the 13" MacBook
//! Pro M1/M2 and the MacBook Air M1. Follows spp3 and lsm2.
//!
//! TODO(SPEC): the thermal model, ceiling and panic-reduction behaviour.

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
        Self { params, failsafe: FailSafe::new(UnitKind::Atsp, Health::NO_PARAMS) }
    }
    pub fn params(&self) -> &Params {
        &self.params
    }
}

impl ProtectionUnit for Unit {
    fn kind(&self) -> UnitKind {
        UnitKind::Atsp
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
