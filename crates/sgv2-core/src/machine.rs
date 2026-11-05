//! Per-board facts the engine needs, from the board the blob's header names
//! (the device tree's first root compatible, as the compiler wrote it).
//! Machine description, not a user setting: a board enters a list here only
//! with the evidence the list states.

/// Boards whose relaxed closed loop is proven live: a full-volume run with
/// the closed-loop correction on, each unit's own speaker calibration, the
/// coils measured within the limit and the offline replay exact. Only these
/// may let the correction raise the temperature limit above the base; every
/// other board, and a blob naming no board, runs the closed loop capped at
/// the base until it has its own such run. The plausibility guard applies
/// on every board.
///
/// - `j616s`, `j616c`: the 16-inch MacBook Pro (M4 Pro, M4 Max), one
///   speaker system; proven on the J616s on 2026-11-05.
const CL_RELAXATION_PROVEN: &[&str] = &["j616s", "j616c"];

/// Whether `board` may relax the closed-loop limit (see
/// [`CL_RELAXATION_PROVEN`]).
pub fn cl_relaxation_proven(board: Option<&str>) -> bool {
    board.is_some_and(|b| CL_RELAXATION_PROVEN.contains(&b))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_proven_boards_relax() {
        assert!(cl_relaxation_proven(Some("j616s")));
        assert!(cl_relaxation_proven(Some("j616c")));
        for b in [None, Some(""), Some("j316s"), Some("j316c"), Some("j413"), Some("j614s"), Some("j700"), Some("j999")]
        {
            assert!(!cl_relaxation_proven(b), "{b:?}");
        }
    }
}
