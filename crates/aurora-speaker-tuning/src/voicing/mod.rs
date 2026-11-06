//! Compiles the voicing boxes (`sgv2_core::voicing`) from a Mac's own
//! speaker graph: one module per box type, each reading that box's saved
//! state from the strip and returning its `sgv2_core::voicing::<box>::Params`,
//! or refusing a state it cannot carry exactly in the direction that matters
//! (never louder than macOS).
//!
//! Shape of every module:
//!
//! ```text
//! pub fn compile(stage: &crate::tuning::Stage, fs: f64, diags: &mut crate::error::Diagnostics)
//!     -> crate::error::Result<Option<sgv2_core::voicing::<box>::Params>>;
//! ```
//!
//! `Ok(None)` means the box is bypassed in the saved state (it contributes
//! nothing). Graph-level parameters are taken at full system volume (graph
//! `mvsc` = 1, `mvol` = 0 dB); a box whose saved state depends on another
//! volume refuses rather than guesses.

pub mod buzz;
pub mod cdelay;
pub mod lowflow;
pub mod xtc;
