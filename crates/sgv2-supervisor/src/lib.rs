//! speakerguardd v2: the supervisor. It does no DSP. It makes the Linux
//! path structurally what macOS's is: the only way to the amps runs through
//! the protection unit, with this Mac's parameters, in time. When that does
//! not hold, it gives the amps back to the kernel's safe level at once.
//!
//! Ownership: lane D owns `kernel` (lock and lease, safe level, amp gain,
//! sense period); the integrity state machine in `lease` is shared.

pub mod kernel;
pub mod lease;
pub mod machine;
pub mod observe;
pub mod pace;
pub mod selfcheck;
pub mod supervise;
pub mod thermal;

// In the library, not under tests/: an integration test would also build
// the `speakerguardd` binary, and in release that binary (panic = "abort")
// and the test harness (unwinding) each need their own build of the
// sgv2-lv2 cdylib, whose two outputs share one file name.
#[cfg(test)]
mod lapse_tests;
#[cfg(test)]
mod operating_point_tests;
