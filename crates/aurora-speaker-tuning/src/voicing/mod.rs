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
//! nothing). Dynamic graph parameters are explicit run-time inputs. In
//! particular AULowFlow parameter 4 is driven by graph `mvol` through its
//! RT-safe `Run::set_volume(db)` method; it is not frozen to the saved state.
//! A box whose graph parameter has no implemented run-time input refuses.

pub mod lowflow;
