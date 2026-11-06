//! In-graph speaker protection core.
//!
//! One framework hosts every protection unit macOS uses on Apple Silicon
//! Macs: `spp3` (V/I Macs), `lsm2` (MacBook Neo, MacBook Air M4/M5) and,
//! later, `atsp`. Each unit implements [`ProtectionUnit`]; the hosts (the
//! LV2 plugin, the offline renderer) only ever see the trait.
//!
//! Rules for everything in this crate (see docs/CONVENTIONS.md):
//! - `process` is real-time code: no allocation, no locks, no syscalls, no
//!   logging, no panics on any input. `no_std` enforces most of that.
//! - Behaviour comes only from the behaviour specification. A gap is a
//!   `TODO(SPEC)`, never a guess, a margin or a heuristic.
//! - Until a unit's behaviour is implemented it attenuates by the
//!   fail-safe amount; nothing in this crate ever passes audio through
//!   unprotected.
#![cfg_attr(not(any(test, feature = "std")), no_std)]

// Storage (look-ahead lines, per-channel state) is allocated once, when a
// unit is built from its parameters; `process` never allocates.
extern crate alloc;

pub mod atsp;
pub mod blob;
pub mod delay;
pub mod dsp;
pub mod failsafe;
pub mod lsm2;
pub mod machine;
pub mod placeholder;
pub mod ring;
pub mod sense;
pub mod spp3;
pub mod telemetry;

pub use failsafe::FailSafe;
pub use telemetry::{ChannelState, GlobalState, Telemetry};

/// The most speaker channels any supported Mac has (six on the 14" and 16"
/// MacBook Pro), rounded up for headroom. Fixed so state is preallocated.
pub const MAX_CHANNELS: usize = 8;

/// Attenuation applied when the unit cannot protect from valid parameters:
/// missing or invalid essential parameters, a channel-count mismatch, or a
/// metering failure. This is the spp3 behaviour; lsm2's is TODO(SPEC).
pub const FAILSAFE_ATTENUATION_DB: f32 = 20.0;

/// Which macOS protection unit a parameter set describes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum UnitKind {
    /// `aufx`/`spp3`/`appl`: feed-forward thermal and displacement models
    /// with optional V/I closed-loop temperature.
    Spp3 = u32::from_be_bytes(*b"spp3"),
    /// `aufx`/`lsm2`/`appl`: loudspeaker model, optional system ID, and a
    /// multi-axis constraint controller.
    Lsm2 = u32::from_be_bytes(*b"lsm2"),
    /// `aufx`/`atsp`/`appl`: audio-only two-node thermal protection.
    Atsp = u32::from_be_bytes(*b"atsp"),
}

impl UnitKind {
    pub fn from_fourcc(code: [u8; 4]) -> Option<Self> {
        match &code {
            b"spp3" => Some(Self::Spp3),
            b"lsm2" => Some(Self::Lsm2),
            b"atsp" => Some(Self::Atsp),
            _ => None,
        }
    }
}

/// Health of the unit after a `process` call, as bits.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Health(pub u32);

impl Health {
    pub const OK: Health = Health(0);
    /// Fail-safe attenuation is applied to this quantum.
    pub const FAILSAFE: u32 = 1 << 0;
    /// Parameters are absent or invalid.
    pub const NO_PARAMS: u32 = 1 << 1;
    /// The audio's channel count does not match the parameters.
    pub const CHANNEL_MISMATCH: u32 = 1 << 2;
    /// Sense data was expected and is unusable; affected channels run
    /// open loop. This is a normal state, not a fault.
    pub const OPEN_LOOP: u32 = 1 << 3;
    /// A model state went non-finite or a protective allocation remained
    /// unsafe after closing every controllable band. The unit has latched
    /// the fail-safe until `reset`; it never trusts that state again.
    pub const MODEL_FAULT: u32 = 1 << 4;
    /// A model state left its plausible range (spp3: the feed-forward node
    /// above 1000 K). The unit holds its hard path while it lasts; the
    /// state keeps integrating (the reference would re-seed it lower).
    pub const MODEL_RANGE: u32 = 1 << 5;
    /// spp3: a channel's closed-loop readings failed the plausibility guard
    /// (resistance falling while the model heats, or a coil reading far
    /// below its calibration temperature under drive). Its correction can no
    /// longer relax the limit until the kernel is rebuilt.
    pub const SENSE_IMPLAUSIBLE: u32 = 1 << 6;

    pub fn is_failsafe(self) -> bool {
        self.0 & Self::FAILSAFE != 0
    }
}

/// One host call's buffers. Every slice in `input` and `output` has
/// `frames` samples; `sense` carries the unit's sense channels in the order
/// the parameters name (empty for units without sense, such as lsm2 on the
/// MacBook Neo or atsp).
pub struct Io<'a, 'b> {
    pub frames: usize,
    pub input: &'a [&'b [f32]],
    pub sense: &'a [&'b [f32]],
    pub output: &'a mut [&'b mut [f32]],
}

/// A macOS protection unit. Hosts call `process` with any frame count,
/// including 1 and non-powers of two: lsm2 on macOS consumes exactly the
/// host's frame count and the core must too.
pub trait ProtectionUnit {
    fn kind(&self) -> UnitKind;
    /// Algorithmic latency in frames, reported to the host
    /// (spp3: its look-ahead; lsm2: filterbank delays plus look-ahead).
    fn latency_frames(&self) -> u32;
    /// Real-time: must not allocate, lock, block or panic.
    fn process(&mut self, io: Io<'_, '_>) -> Health;
    /// Snapshot of the internal state the differential rig compares.
    fn telemetry(&self, out: &mut Telemetry);
    /// Return to the initial state (stream start): the parameters' start
    /// state, empty histories, unity gains.
    fn reset(&mut self);
    /// The host's in-stream reset (`AudioUnitReset` on macOS). spp3 clears
    /// only its closed-loop estimator and keeps the thermal state; units
    /// whose reset semantics are not specified apart from construction
    /// reconstruct.
    fn host_reset(&mut self) {
        self.reset();
    }
    /// Change one AU parameter at run time (the unit's own ID map), taking
    /// effect at the next call. False when the unit has no such run-time
    /// parameter.
    fn set_parameter(&mut self, _id: u32, _value: f32) -> bool {
        false
    }
    /// Copy the persistent thermal state for a later stream or instance of
    /// the same unit. Real-time safe. False when there is nothing to hand
    /// off (a fail-safe unit, or a latched model fault: never hand off a
    /// state the unit itself no longer trusts).
    fn save_state(&self, _out: &mut HandOff) -> bool {
        false
    }
    /// Seed the thermal state from `state` after `gap_s` seconds of
    /// silence, as the reference restores its hand-off record (spec spp3
    /// 10: every node aged by the gap, the saved gain advanced toward unity
    /// with the release coefficient). Call right after [`reset`]; audio
    /// histories stay empty. A record for another unit kind, geometry or
    /// rate, or with a non-finite or negative value, is refused (false) and
    /// leaves the reset state. A non-finite or negative gap counts as 0
    /// (no cooling). Not real time.
    ///
    /// [`reset`]: ProtectionUnit::reset
    fn restore_state(&mut self, _state: &HandOff, _gap_s: f64) -> bool {
        false
    }
}

/// Thermal nodes per channel in a [`HandOff`] record.
pub const HANDOFF_NODES: usize = 8;

/// A unit's persistent thermal state, handed from one stream to the next.
/// A speaker stays hot across a pause, a suspend or a graph rebuild, so a
/// restarted stream must not start cold: macOS keeps this state vector
/// process-global and restores it, aged for the gap (spp3 spec 10).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HandOff {
    /// `UnitKind` as u32.
    pub kind: u32,
    pub channels: u32,
    pub sample_rate: u32,
    /// Per unit channel, the unit's thermal nodes (layout per unit: spp3
    /// coil, magnet, virtual-coil rises over ambient in K and the hard-path
    /// arm flag; atsp coil and magnet rises; lsm2 coil and magnet rises,
    /// the four power averages in W and the hard-limit arm flag).
    pub node: [[f64; HANDOFF_NODES]; MAX_CHANNELS],
    /// Per unit channel, the slow (thermal) controller gain, linear.
    pub gain: [f64; MAX_CHANNELS],
}

impl HandOff {
    pub const EMPTY: HandOff = HandOff {
        kind: 0,
        channels: 0,
        sample_rate: 0,
        node: [[0.0; HANDOFF_NODES]; MAX_CHANNELS],
        gain: [1.0; MAX_CHANNELS],
    };

    /// Whether this record fits a unit, with every value it carries for its
    /// channels finite, every node nonnegative and every gain in (0, 1].
    pub fn fits(&self, kind: UnitKind, channels: usize, sample_rate: u32) -> bool {
        if self.kind != kind as u32 || self.channels as usize != channels || self.sample_rate != sample_rate {
            return false;
        }
        let n = channels.min(MAX_CHANNELS);
        self.node[..n].iter().flatten().all(|v| v.is_finite() && *v >= 0.0)
            && self.gain[..n].iter().all(|g| g.is_finite() && *g > 0.0 && *g <= 1.0)
    }
}

/// `gap_s` as a cooling time: a non-finite or negative gap is no cooling.
pub fn gap_or_zero(gap_s: f64) -> f64 {
    if gap_s.is_finite() && gap_s > 0.0 {
        gap_s
    } else {
        0.0
    }
}

/// Zero-input cooling of a coupled coil/magnet pair over `t` seconds, the
/// continuous solution of `dm/dt = -m/tau_m`, `dv/dt = (m - v)/tau_v` (the
/// atsp and lsm2 two-node models: the coil relaxes toward the magnet).
/// Returns the coil and magnet rises.
pub fn cool_coupled(v0: f64, m0: f64, tau_v: f64, tau_m: f64, t: f64) -> (f64, f64) {
    let ev = libm::exp(-t / tau_v);
    let em = libm::exp(-t / tau_m);
    let m = m0 * em;
    let d = tau_m - tau_v;
    let v =
        if libm::fabs(d) <= 1e-9 * tau_m { (v0 + m0 * t / tau_v) * ev } else { v0 * ev + m0 * tau_m / d * (em - ev) };
    (v.max(0.0), m.max(0.0))
}

/// A gain state `g` after `steps` steps of release toward unity with the
/// one-pole factor `r` per step (`g <- 1 + r (g - 1)`).
pub fn release_toward_unity(g: f64, r: f64, steps: f64) -> f64 {
    1.0 - (1.0 - g) * libm::pow(r, steps)
}

/// Linear gain of [`FAILSAFE_ATTENUATION_DB`]: 10^(-20/20) = 0.1.
pub const FAILSAFE_GAIN: f32 = 0.1;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fourcc_roundtrip() {
        assert_eq!(UnitKind::from_fourcc(*b"spp3"), Some(UnitKind::Spp3));
        assert_eq!(UnitKind::from_fourcc(*b"lsm2"), Some(UnitKind::Lsm2));
        assert_eq!(UnitKind::from_fourcc(*b"atsp"), Some(UnitKind::Atsp));
        assert_eq!(UnitKind::from_fourcc(*b"mstk"), None);
        assert_eq!(UnitKind::Spp3 as u32, 0x7370_7033);
    }

    #[test]
    fn failsafe_gain_is_twenty_db() {
        assert!((20.0 * libm_log10(FAILSAFE_GAIN) + FAILSAFE_ATTENUATION_DB).abs() < 1e-5);
    }

    fn libm_log10(x: f32) -> f32 {
        (x as f64).log10() as f32
    }
}
