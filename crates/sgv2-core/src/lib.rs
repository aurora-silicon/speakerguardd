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
pub mod placeholder;
pub mod ring;
pub mod sense;
pub mod spp3;
pub mod telemetry;

pub use failsafe::FailSafe;
pub use telemetry::{ChannelState, Telemetry};

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
    /// Return to the initial state (stream start).
    fn reset(&mut self);
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
