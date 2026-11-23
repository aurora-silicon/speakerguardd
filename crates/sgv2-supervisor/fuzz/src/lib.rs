//! Shared by the supervisor's fuzz targets. Targets: `selfcheck` (the
//! self-check record), `thermal` (the persisted thermal state and its boot
//! id), `procfs` (`/proc` and session-helper readers), `blobcheck` (the
//! grant-time blob, compatible and TLV checks) and `observe` (the
//! path-integrity checks and the lease over fuzzed tick sequences). Run
//! from `crates/sgv2-supervisor` with cargo-fuzz; a stable toolchain works
//! with `RUSTC_BOOTSTRAP=1` (cargo-fuzz passes `-Zsanitizer`). `observe`
//! is built with `-a` so an arithmetic overflow is a crash too:
//!
//! ```text
//! RUSTC_BOOTSTRAP=1 cargo fuzz run -O selfcheck -- -max_total_time=900 -detect_leaks=0
//! RUSTC_BOOTSTRAP=1 cargo fuzz run -O -a observe -- -max_total_time=900
//! ```
//!
//! `selfcheck` needs `-detect_leaks=0`: `machine_for` leaks the machine it
//! builds on purpose (it lives for the supervisor's lifetime).

pub use arbitrary;
