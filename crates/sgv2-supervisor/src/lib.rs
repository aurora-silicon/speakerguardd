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
pub mod selfcheck;
pub mod supervise;
