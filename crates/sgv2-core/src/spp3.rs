//! `spp3`: feed-forward thermal and displacement models per channel, with
//! optional closed-loop temperature from the V/I telemetry bus (pilot-tone
//! resistance thermometry). Target: every V/I MacBook; the 16" M4 Pro first.
//!
//! TODO(SPEC): the open-loop thermal model, power model, closed-loop
//! correction (TlimAdj with decay and cap), pilot tone, telemetry
//! plausibility, gain computation and smoothing, group linking, look-ahead.

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
        Self { params, failsafe: FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS) }
    }
    pub fn params(&self) -> &Params {
        &self.params
    }
}

impl ProtectionUnit for Unit {
    fn kind(&self) -> UnitKind {
        UnitKind::Spp3
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
