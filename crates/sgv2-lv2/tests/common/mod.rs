//! Process-local inputs for tests that instantiate the protection plugin.

use std::ffi::OsString;
use std::sync::{Mutex, MutexGuard};

static ENV_LOCK: Mutex<()> = Mutex::new(());
const KEYS: [&str; 3] = ["SGV2_PARAMS", "SGV2_ENGINE_PAGE", "SGV2_GRANT_GATE"];

pub fn lock() -> MutexGuard<'static, ()> {
    ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

/// Keep the lock until the original environment has been restored, also on
/// unwind. Never let a synthetic C ABI test start the real card watcher.
/// Tests may override the two paths while this guard is alive.
pub struct TestEnv<'a> {
    previous: [Option<OsString>; 3],
    _lock: &'a MutexGuard<'static, ()>,
}

impl<'a> TestEnv<'a> {
    pub fn new(lock: &'a MutexGuard<'static, ()>) -> Self {
        let previous = KEYS.map(std::env::var_os);
        std::env::set_var("SGV2_PARAMS", "/nonexistent/sgv2-test");
        std::env::set_var("SGV2_ENGINE_PAGE", "/nonexistent/sgv2-test");
        std::env::set_var("SGV2_GRANT_GATE", "0");
        Self { previous, _lock: lock }
    }
}

impl Drop for TestEnv<'_> {
    fn drop(&mut self) {
        for (key, value) in KEYS.iter().zip(&self.previous) {
            match value {
                Some(value) => std::env::set_var(key, value),
                None => std::env::remove_var(key),
            }
        }
    }
}
