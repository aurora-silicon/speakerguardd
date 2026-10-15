// SPDX-License-Identifier: MIT
//! Log lines on stderr, stamped with the kernel's monotonic clock so they
//! line up with dmesg.

use std::sync::atomic::{AtomicU8, Ordering};

#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub enum Level {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
}

static VERBOSITY: AtomicU8 = AtomicU8::new(Level::Info as u8);

pub fn set_verbosity(level: Level) {
    VERBOSITY.store(level as u8, Ordering::Relaxed);
}

pub fn log(level: Level, msg: std::fmt::Arguments) {
    if level as u8 <= VERBOSITY.load(Ordering::Relaxed) {
        let tag = match level {
            Level::Error => "E",
            Level::Warn => "W",
            Level::Info => "I",
            Level::Debug => "D",
        };
        let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: a valid, writable timespec.
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        eprintln!("[{:5}.{:06}] {tag} {msg}", ts.tv_sec, ts.tv_nsec / 1000);
    }
}

#[macro_export]
macro_rules! error { ($($t:tt)*) => { $crate::log::log($crate::log::Level::Error, format_args!($($t)*)) } }
#[macro_export]
macro_rules! warn { ($($t:tt)*) => { $crate::log::log($crate::log::Level::Warn, format_args!($($t)*)) } }
#[macro_export]
macro_rules! info { ($($t:tt)*) => { $crate::log::log($crate::log::Level::Info, format_args!($($t)*)) } }
#[macro_export]
macro_rules! debug { ($($t:tt)*) => { $crate::log::log($crate::log::Level::Debug, format_args!($($t)*)) } }
