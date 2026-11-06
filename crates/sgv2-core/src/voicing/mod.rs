//! The voicing: the speaker graph's boxes ahead of the protection unit that
//! the per-lane pre-stage ([`crate::prestage`]) does not carry on its own.
//!
//! macOS renders the speaker program through one chain per Mac: stereo
//! boxes on the program (loudness normalizer, virtual bass, tone, XTC,
//! variable EQ, compressor), a fan-out to the speaker lanes, per-lane
//! filtering, then linked peak limiters per driver group, then the
//! protection unit. Each box here is a clean-room implementation of one
//! box type from its behaviour specification, compiled from the Mac's own
//! saved state by `aurora-speaker-tuning`'s `voicing` module. No tuning
//! value lives in this crate.
//!
//! Every box module has the same shape, so the chain can host it without
//! knowing its insides:
//!
//! ```text
//! pub struct Params { .. }                    // plain values, f64
//! impl Params {
//!     pub const WORDS: usize;                 // fixed encoded length
//!     pub fn to_words(&self, out: &mut [f64]); // exactly WORDS
//!     pub fn from_words(w: &[f64]) -> Option<Params>; // range-checked
//! }
//! pub struct Run { .. }                       // all storage made in new
//! impl Run {
//!     pub fn new(p: &Params, fs: f64, channels: usize) -> Run;
//!     pub fn reset(&mut self);
//!     pub fn latency(&self) -> usize;         // frames
//!     pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize);
//! }
//! ```
//!
//! `process` runs in place on `channels` buffers and is real-time code: no
//! allocation, no locks, no panics on any input (non-finite input is
//! treated as silence). A box's state is f64 unless its specification says
//! the unit's arithmetic differs.
//!
//! Box modules are added one per box type (`norm`, `vbass`, `xtc`,
//! `vareq`, `cfreak`, `buzz`, `cdelay`, ...).

pub mod buzz;
pub mod cdelay;
pub mod cfreak;
pub mod chain;
pub mod lowflow;
pub mod mstk;
pub mod pchr;
pub mod vareq;
pub mod vbass;
pub mod volt;
pub mod volume;
pub mod xtc;
