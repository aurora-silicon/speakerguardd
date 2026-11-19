//! Keeping the plugin on the supervisor's current engine page.
//!
//! The supervisor creates `/run/speakerguardd/<uid>/engine`, and the plugin
//! maps it in `instantiate`. If that file is replaced while PipeWire keeps
//! the plugin loaded, the plugin would go on beating into the old, unlinked
//! page, the new supervisor would never see a heartbeat, and the speakers
//! would stay at the kernel's floor until PipeWire reloads the graph. That
//! happens when the supervisor is stopped and started again (its runtime
//! directory goes, unless preserved), when the page is removed by hand, or
//! when the graph loaded before the supervisor first created the page.
//!
//! A watcher thread (not real time) looks at the path every
//! [`WATCH_PERIOD`]. When it names another file than the one mapped (device
//! and inode), or a page appears where there was none, it maps the new file,
//! starts its header, and swaps it in. The real-time thread only loads an
//! atomic pointer at the start of a call ([`Attach::enter`]) and marks the
//! call done ([`Attach::leave`]); no allocation, lock or system call. A
//! replaced page is unmapped only once no call that could still hold it is
//! running: none was in progress, or one has finished since the swap.
//!
//! Until the plugin beats into the new page the supervisor sees no
//! heartbeat and grants nothing, so the floor holds; re-attaching is what
//! lets it grant again without a PipeWire restart.

use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU64, Ordering};
use std::os::unix::fs::MetadataExt;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use crate::page::{EnginePage, PAGE_BYTES};

/// How often the watcher looks at the page path.
pub const WATCH_PERIOD: Duration = Duration::from_millis(100);

/// What `start` writes into a freshly mapped page's header.
#[derive(Clone, Copy, Debug)]
pub struct Header {
    pub params_crc: u32,
    pub latency: u32,
    pub channels: u32,
    pub sense: u32,
}

struct Shared {
    path: String,
    header: Header,
    cur: AtomicPtr<EnginePage>,
    /// 1 while a real-time call holds a page pointer.
    in_call: AtomicU64,
    /// Calls finished so far.
    calls_done: AtomicU64,
    stop: AtomicBool,
    /// The mapped file's (device, inode), and pages swapped out but not yet
    /// unmapped, with `calls_done` at the swap. Watcher side only.
    state: Mutex<WatchState>,
}

#[derive(Default)]
struct WatchState {
    id: Option<(u64, u64)>,
    retired: Vec<(Box<EnginePage>, u64)>,
}

pub struct Attach {
    shared: Arc<Shared>,
    watcher: Option<JoinHandle<()>>,
}

fn file_id(path: &str) -> Option<(u64, u64)> {
    let m = std::fs::metadata(path).ok()?;
    (m.is_file() && m.len() >= PAGE_BYTES as u64).then(|| (m.dev(), m.ino()))
}

impl Shared {
    /// One look at the path: re-attach if it names another page.
    fn check(&self) -> bool {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        let mut attached = false;
        if let Some(id) = file_id(&self.path) {
            if st.id != Some(id) {
                if let Ok(p) = EnginePage::open(&self.path) {
                    // the file may have been swapped between the stat and
                    // the open: name it by what was opened next time
                    let h = self.header;
                    p.start(h.params_crc, h.latency, h.channels, h.sense);
                    let new = Box::into_raw(Box::new(p));
                    let old = self.cur.swap(new, Ordering::SeqCst);
                    let done = self.calls_done.load(Ordering::SeqCst);
                    if !old.is_null() {
                        // SAFETY: every pointer in `cur` came from
                        // `Box::into_raw` here or in `new`, and was swapped
                        // out exactly once.
                        st.retired.push((unsafe { Box::from_raw(old) }, done));
                    }
                    st.id = file_id(&self.path).or(Some(id));
                    attached = true;
                }
            }
        }
        // A retired page is safe to unmap once no call can still hold it:
        // none is in progress now (a later call loads the new pointer), or
        // a call has finished since the swap (calls on an instance never
        // overlap, so the one that may have held it is over).
        let idle = self.in_call.load(Ordering::SeqCst) == 0;
        let now = self.calls_done.load(Ordering::SeqCst);
        st.retired.retain(|(_, at)| !(idle || now > *at));
        attached
    }
}

impl Attach {
    /// Map the page at `path` if there is one, and start the watcher.
    pub fn new(path: String, header: Header) -> Self {
        let shared = Arc::new(Shared {
            path,
            header,
            cur: AtomicPtr::new(core::ptr::null_mut()),
            in_call: AtomicU64::new(0),
            calls_done: AtomicU64::new(0),
            stop: AtomicBool::new(false),
            state: Mutex::new(WatchState::default()),
        });
        shared.check();
        let s = shared.clone();
        let watcher = std::thread::Builder::new()
            .name("sgv2-page-watch".into())
            .spawn(move || {
                loop {
                    // woken early by `drop`, so cleanup never waits a period
                    std::thread::park_timeout(WATCH_PERIOD);
                    if s.stop.load(Ordering::Acquire) {
                        break;
                    }
                    s.check();
                }
            })
            .ok();
        Attach { shared, watcher }
    }

    /// Real time: the current page for this call, if any. Pair with
    /// [`Attach::leave`].
    #[inline]
    pub fn enter(&self) -> Option<&EnginePage> {
        self.shared.in_call.store(1, Ordering::SeqCst);
        let p = self.shared.cur.load(Ordering::SeqCst);
        // SAFETY: a page in `cur`, or swapped out during this call, stays
        // mapped until `leave` (see `Shared::check`).
        unsafe { p.as_ref() }
    }

    /// Real time: the call that `enter` began is over.
    #[inline]
    pub fn leave(&self) {
        self.shared.calls_done.fetch_add(1, Ordering::SeqCst);
        self.shared.in_call.store(0, Ordering::SeqCst);
    }

    /// Look at the path now (tests, and `instantiate`).
    pub fn check(&self) -> bool {
        self.shared.check()
    }

    /// Pages swapped out and still mapped (tests).
    pub fn retired(&self) -> usize {
        self.shared.state.lock().map(|s| s.retired.len()).unwrap_or(0)
    }
}

impl Drop for Attach {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        if let Some(w) = self.watcher.take() {
            w.thread().unpark();
            let _ = w.join();
        }
        let p = self.shared.cur.swap(core::ptr::null_mut(), Ordering::SeqCst);
        if !p.is_null() {
            // SAFETY: from `Box::into_raw` in `check`; no call runs during
            // cleanup.
            drop(unsafe { Box::from_raw(p) });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::page::MAGIC;
    use sgv2_core::Health;

    const H: Header = Header { params_crc: 0x1234, latency: 96, channels: 4, sense: 8 };

    fn tmp(name: &str) -> String {
        let d = std::env::temp_dir().join(format!("sgv2-attach-{}-{name}", std::process::id()));
        let _ = std::fs::create_dir_all(&d);
        d.join("engine").to_string_lossy().into_owned()
    }

    fn beat(a: &Attach, n: u64) {
        let p = a.enter().expect("attached");
        p.beat(n * 512, Health::OK, -1);
        a.leave();
    }

    /// The supervisor stopped and started: its page file was removed and
    /// created again. The plugin beats into the new file on its own.
    #[test]
    fn a_replaced_page_is_reattached_and_the_old_one_freed() {
        let path = tmp("replace");
        let first = EnginePage::create(&path).unwrap();
        let a = Attach::new(path.clone(), H);
        beat(&a, 1);
        assert_eq!(first.header().seq.load(Ordering::Acquire), 1);
        std::fs::remove_file(&path).unwrap();
        // gone: the plugin keeps the page it has
        assert!(!a.check());
        beat(&a, 2);
        let second = EnginePage::create(&path).unwrap();
        assert_eq!(second.header().magic.load(Ordering::Acquire), 0);
        assert!(a.check(), "re-attaches to the new file");
        assert_eq!(second.header().magic.load(Ordering::Acquire), MAGIC, "the new header is started");
        assert_eq!(second.header().params_crc.load(Ordering::Relaxed), 0x1234);
        beat(&a, 3);
        assert_eq!(second.header().seq.load(Ordering::Acquire), 1);
        assert_eq!(first.header().seq.load(Ordering::Acquire), 2, "nothing more into the old page");
        // no call was running at the swap: freed on the next look
        a.check();
        assert_eq!(a.retired(), 0);
        // the same file again: nothing to do
        assert!(!a.check());
        let _ = std::fs::remove_file(&path);
    }

    /// The graph loaded before the supervisor ever made a page.
    #[test]
    fn a_page_that_appears_later_is_attached() {
        let path = tmp("late");
        let _ = std::fs::remove_file(&path);
        let a = Attach::new(path.clone(), H);
        assert!(a.enter().is_none());
        a.leave();
        let p = EnginePage::create(&path).unwrap();
        assert!(a.check());
        beat(&a, 1);
        assert_eq!(p.header().seq.load(Ordering::Acquire), 1);
        let _ = std::fs::remove_file(&path);
    }

    /// A page swapped out during a call stays mapped until that call ends.
    #[test]
    fn a_page_held_by_a_running_call_is_not_unmapped() {
        let path = tmp("held");
        let _first = EnginePage::create(&path).unwrap();
        let a = Attach::new(path.clone(), H);
        let held = a.enter().unwrap() as *const EnginePage;
        std::fs::remove_file(&path).unwrap();
        let _second = EnginePage::create(&path).unwrap();
        assert!(a.check());
        assert_eq!(a.retired(), 1, "kept while the call runs");
        // SAFETY: still mapped, as asserted.
        unsafe { (*held).beat(512, Health::OK, -1) };
        a.leave();
        a.check();
        assert_eq!(a.retired(), 0);
        let _ = std::fs::remove_file(&path);
    }

    /// The watcher thread does it without being asked.
    #[test]
    fn the_watcher_reattaches_by_itself() {
        let path = tmp("watch");
        let _first = EnginePage::create(&path).unwrap();
        let a = Attach::new(path.clone(), H);
        std::fs::remove_file(&path).unwrap();
        let second = EnginePage::create(&path).unwrap();
        let t0 = std::time::Instant::now();
        while second.header().magic.load(Ordering::Acquire) != MAGIC {
            assert!(t0.elapsed() < Duration::from_secs(5), "the watcher did not re-attach");
            std::thread::sleep(Duration::from_millis(10));
        }
        beat(&a, 1);
        assert_eq!(second.header().seq.load(Ordering::Acquire), 1);
        drop(a);
        let _ = std::fs::remove_file(&path);
    }
}
