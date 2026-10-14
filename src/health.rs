// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ryan Murray <ryan@aurorasilicon.org>
use std::time::{Duration, Instant};

#[derive(Debug)]
pub struct SenseExpired;
impl std::fmt::Display for SenseExpired {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("no valid speaker sense data for 250 ms; returning to kernel protection")
    }
}
impl std::error::Error for SenseExpired {}

#[derive(Default)]
pub struct SenseHealth {
    last_valid: Option<Instant>,
}
impl SenseHealth {
    pub fn active(&mut self, now: Instant) {
        self.last_valid.get_or_insert(now);
    }
    pub fn idle(&mut self) {
        self.last_valid = None;
    }
    pub fn require(&self, now: Instant) -> Result<(), SenseExpired> {
        if self
            .last_valid
            .is_some_and(|last| now.duration_since(last) >= Duration::from_millis(250))
        {
            Err(SenseExpired)
        } else {
            Ok(())
        }
    }
    /// A valid sample at/after the deadline must not erase a preceding gap.
    pub fn valid(&mut self, now: Instant) -> Result<(), SenseExpired> {
        self.require(now)?;
        self.last_valid = Some(now);
        Ok(())
    }
    /// Check before and after potentially blocking I/O before the caller can
    /// accept its result, renew health, or renew the kernel unlock timer.
    pub fn checked<T>(&self, op: impl FnOnce() -> T) -> Result<T, SenseExpired> {
        self.require(Instant::now())?;
        self.checked_with_clock(op, Instant::now)
    }
    fn checked_with_clock<T>(
        &self,
        op: impl FnOnce() -> T,
        now: impl FnOnce() -> Instant,
    ) -> Result<T, SenseExpired> {
        let value = op();
        self.require(now())?;
        Ok(value)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use std::rc::Rc;
    #[test]
    fn valid_at_or_after_deadline_cannot_renew() {
        for elapsed in [250, 251, 1000] {
            let now = Instant::now();
            let mut h = SenseHealth::default();
            h.active(now);
            assert!(h.valid(now + Duration::from_millis(elapsed)).is_err());
            assert!(h.require(now + Duration::from_millis(elapsed)).is_err());
        }
    }
    #[test]
    fn valid_before_deadline_and_idle_transition() {
        let now = Instant::now();
        let mut h = SenseHealth::default();
        h.active(now);
        h.valid(now + Duration::from_millis(249)).unwrap();
        h.require(now + Duration::from_millis(498)).unwrap();
        h.idle();
        h.require(now + Duration::from_secs(10)).unwrap();
    }
    #[test]
    fn actual_post_io_guard_rejects_late_success_error_empty_and_overrun() {
        let now = Instant::now();
        let mut h = SenseHealth::default();
        h.active(now);
        // Same wrapper as production's control read, capture open and read_period.
        for result in [Ok(Some(true)), Ok(Some(false)), Ok(None), Err(5)] {
            assert!(h
                .checked_with_clock(|| result, || now + Duration::from_millis(250))
                .is_err());
        }
    }
    #[test]
    fn repeated_open_control_errors_and_overruns_do_not_renew() {
        let now = Instant::now();
        let mut h = SenseHealth::default();
        h.active(now);
        for elapsed in [50, 100, 200, 249] {
            let result =
                h.checked_with_clock(|| Err::<(), _>(5), || now + Duration::from_millis(elapsed));
            assert!(result.unwrap().is_err());
            h.active(now + Duration::from_millis(elapsed));
        }
        assert!(h.require(now + Duration::from_millis(250)).is_err());
    }
    #[test]
    fn fatal_post_io_return_releases_owned_handles() {
        struct Owned(Rc<Cell<u32>>);
        impl Drop for Owned {
            fn drop(&mut self) {
                self.0.set(self.0.get() + 1);
            }
        }
        fn run(
            count: Rc<Cell<u32>>,
            h: &SenseHealth,
            deadline: Instant,
        ) -> Result<(), SenseExpired> {
            let _control = Owned(count.clone());
            let _capture = Owned(count);
            h.checked_with_clock(|| Some(true), || deadline)?;
            Ok(())
        }
        let now = Instant::now();
        let mut h = SenseHealth::default();
        h.active(now);
        let count = Rc::new(Cell::new(0));
        assert!(run(count.clone(), &h, now + Duration::from_millis(250)).is_err());
        assert_eq!(count.get(), 2);
    }
    #[test]
    fn expired_budget_never_starts_an_unlock_retry() {
        let mut h = SenseHealth::default();
        h.active(Instant::now() - Duration::from_secs(1));
        let called = Cell::new(false);
        assert!(h.checked(|| called.set(true)).is_err());
        assert!(!called.get());
    }
}
