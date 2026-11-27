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
//! A watcher thread (not real time) watches the page's parent directory with
//! inotify on Linux (falling back to polling every [`WATCH_PERIOD`] while the
//! directory is absent, while a retired page waits to be unmapped, or on
//! non-Linux). When the path names another file than the one mapped (device
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

/// Fallback poll period when the parent directory does not exist yet, while a
/// retired page awaits unmap, or when inotify is unavailable.
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
    #[cfg(target_os = "linux")]
    wake_fd: i32,
}

fn file_id(path: &str) -> Option<(u64, u64)> {
    let m = std::fs::metadata(path).ok()?;
    (m.is_file() && m.len() >= PAGE_BYTES as u64).then(|| (m.dev(), m.ino()))
}

#[cfg(target_os = "linux")]
mod linux_watch {
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;
    use std::os::unix::fs::MetadataExt;
    use std::path::{Path, PathBuf};
    use std::sync::atomic::Ordering;

    use super::{Shared, WATCH_PERIOD};

    pub(super) fn new_wake_fd() -> i32 {
        // SAFETY: plain non-blocking eventfd owned by Attach.
        unsafe { libc::eventfd(0, libc::EFD_NONBLOCK | libc::EFD_CLOEXEC) }
    }

    pub(super) fn wake(fd: i32) {
        if fd >= 0 {
            let one: u64 = 1;
            // SAFETY: writing an 8-byte counter to our own eventfd.
            unsafe { libc::write(fd, (&one as *const u64).cast(), 8) };
        }
    }

    pub(super) fn close_fd(fd: i32) {
        if fd >= 0 {
            // SAFETY: closing an fd we own after the watcher thread joined.
            unsafe { libc::close(fd) };
        }
    }

    type DirId = (u64, u64);
    type WatchedDirs = Vec<(PathBuf, DirId)>;

    fn dir_id(dir: &Path) -> Option<DirId> {
        let m = std::fs::symlink_metadata(dir).ok()?;
        (!m.file_type().is_symlink() && m.is_dir()).then(|| (m.dev(), m.ino()))
    }

    fn plain_file_id(path: &str) -> Option<DirId> {
        let m = std::fs::symlink_metadata(path).ok()?;
        (!m.file_type().is_symlink() && m.is_file() && m.len() >= super::PAGE_BYTES as u64).then(|| (m.dev(), m.ino()))
    }

    /// Watch `parent(path)` for file changes and every higher ancestor below
    /// `/` for self-move/delete, so renaming or removing any ancestor directory
    /// wakes `poll()` immediately without waking on unrelated sibling activity
    /// in `/run` or `/tmp`. Relative or `..`-containing paths return `None` to
    /// keep the bounded poll fallback.
    fn open_dir_watch(path: &str) -> Option<(i32, WatchedDirs)> {
        let p = Path::new(path);
        if !p.is_absolute() || p.components().any(|c| matches!(c, std::path::Component::ParentDir)) {
            return None;
        }
        let parent = p.parent().filter(|d| !d.as_os_str().is_empty())?;
        let mut dirs = Vec::new();
        let mut seen_ids = Vec::new();
        for anc in parent.ancestors() {
            if anc.as_os_str().is_empty() || anc == Path::new("/") {
                break;
            }
            let id = dir_id(anc)?;
            if !seen_ids.contains(&id) {
                seen_ids.push(id);
                dirs.push((anc.to_path_buf(), id));
            }
        }
        if dirs.is_empty() {
            return None;
        }
        // SAFETY: plain inotify syscalls on descriptors we own.
        unsafe {
            let fd = libc::inotify_init1(libc::IN_NONBLOCK | libc::IN_CLOEXEC);
            if fd < 0 {
                return None;
            }
            // Include IN_DELETE on the immediate parent: when an unlinked
            // engine page stays mapped until re-attach, its dentry pins the
            // parent directory inode, so IN_DELETE_SELF on rmdir may not
            // arrive until the old mapping is dropped.
            let leaf_mask = libc::IN_CREATE
                | libc::IN_DELETE
                | libc::IN_MOVED_TO
                | libc::IN_CLOSE_WRITE
                | libc::IN_ATTRIB
                | libc::IN_DELETE_SELF
                | libc::IN_MOVE_SELF;
            let anc_mask = libc::IN_DELETE_SELF | libc::IN_MOVE_SELF;
            for (i, (dir, id)) in dirs.iter().enumerate() {
                let Some(cpath) = CString::new(dir.as_os_str().as_bytes()).ok() else {
                    libc::close(fd);
                    return None;
                };
                let mask = if i == 0 { leaf_mask } else { anc_mask };
                if libc::inotify_add_watch(fd, cpath.as_ptr(), mask) < 0 || dir_id(dir) != Some(*id) {
                    libc::close(fd);
                    return None;
                }
            }
            Some((fd, dirs))
        }
    }

    fn watches_valid(dirs: &[(PathBuf, DirId)]) -> bool {
        !dirs.is_empty() && dirs.iter().all(|(d, id)| dir_id(d) == Some(*id))
    }

    /// Drain pending inotify events; true if any watched directory went away,
    /// moved, or the queue overflowed so the watches must be re-established.
    fn drain_inotify(fd: i32) -> bool {
        let mut buf = [0u8; 4096];
        let mut invalidated = false;
        let hdr = core::mem::size_of::<libc::inotify_event>();
        loop {
            // SAFETY: reading into our own buffer from a non-blocking inotify fd.
            let n = unsafe { libc::read(fd, buf.as_mut_ptr().cast(), buf.len()) };
            if n <= 0 {
                break;
            }
            let mut off = 0usize;
            let end = n as usize;
            while off + hdr <= end {
                // SAFETY: `off + hdr <= end`, unaligned read of POD struct.
                let ev = unsafe { core::ptr::read_unaligned(buf.as_ptr().add(off).cast::<libc::inotify_event>()) };
                if ev.mask & (libc::IN_DELETE_SELF | libc::IN_MOVE_SELF | libc::IN_IGNORED | libc::IN_Q_OVERFLOW) != 0 {
                    invalidated = true;
                }
                let step = hdr.saturating_add(ev.len as usize);
                if step == 0 || off + step > end {
                    break;
                }
                off += step;
            }
        }
        invalidated
    }

    pub(super) fn run(s: &Shared, wake_fd: i32) {
        if wake_fd < 0 {
            loop {
                std::thread::park_timeout(WATCH_PERIOD);
                if s.stop.load(Ordering::Acquire) {
                    break;
                }
                s.check();
            }
            return;
        }
        let mut ifd: i32 = -1;
        let mut watched: WatchedDirs = Vec::new();
        let poll_ms = WATCH_PERIOD.as_millis().min(i32::MAX as u128) as i32;
        loop {
            if s.stop.load(Ordering::Acquire) {
                break;
            }
            if ifd >= 0 && !watches_valid(&watched) {
                close_fd(ifd);
                ifd = -1;
                watched.clear();
            }
            if ifd < 0 {
                if let Some((fd, dirs)) = open_dir_watch(&s.path) {
                    ifd = fd;
                    watched = dirs;
                    // Catch a file created before the watch was installed.
                    s.check();
                    if s.stop.load(Ordering::Acquire) {
                        break;
                    }
                }
            }
            // Enter the indefinite wait only when the watches cover a
            // symlink-free live path, no retired mapping waits to unmap, and
            // the current regular file identifies the page we actually mapped.
            let current = plain_file_id(&s.path);
            let settled = ifd >= 0
                && current.is_some()
                && s.state.lock().map(|st| st.retired.is_empty() && st.id == current).unwrap_or(false);
            let timeout = if settled { -1 } else { poll_ms };
            let mut pfds = [
                libc::pollfd { fd: wake_fd, events: libc::POLLIN, revents: 0 },
                libc::pollfd { fd: ifd, events: libc::POLLIN, revents: 0 },
            ];
            let nfds: libc::nfds_t = if ifd >= 0 { 2 } else { 1 };
            // SAFETY: `pfds[..nfds]` holds live file descriptors.
            let r = unsafe { libc::poll(pfds.as_mut_ptr(), nfds, timeout) };
            if s.stop.load(Ordering::Acquire) || (r > 0 && pfds[0].revents & libc::POLLIN != 0) {
                break;
            }
            if ifd >= 0
                && r > 0
                && pfds[1].revents != 0
                && (pfds[1].revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 || drain_inotify(ifd))
            {
                close_fd(ifd);
                ifd = -1;
                watched.clear();
            }
            s.check();
        }
        close_fd(ifd);
    }
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
    /// Map the page at `path` if there is one, without spawning the watcher
    /// thread (for deterministic synchronous `check()` tests).
    #[cfg(test)]
    fn new_unwatched(path: String, header: Header) -> Self {
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
        Attach {
            shared,
            watcher: None,
            #[cfg(target_os = "linux")]
            wake_fd: -1,
        }
    }

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
        #[cfg(target_os = "linux")]
        let wake_fd = linux_watch::new_wake_fd();
        let watcher = std::thread::Builder::new()
            .name("sgv2-page-watch".into())
            .spawn(move || {
                #[cfg(target_os = "linux")]
                linux_watch::run(&s, wake_fd);
                #[cfg(not(target_os = "linux"))]
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
        Attach {
            shared,
            watcher,
            #[cfg(target_os = "linux")]
            wake_fd,
        }
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
        #[cfg(target_os = "linux")]
        linux_watch::wake(self.wake_fd);
        if let Some(w) = self.watcher.take() {
            w.thread().unpark();
            let _ = w.join();
        }
        #[cfg(target_os = "linux")]
        linux_watch::close_fd(self.wake_fd);
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
        let a = Attach::new_unwatched(path.clone(), H);
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
        let a = Attach::new_unwatched(path.clone(), H);
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
        let a = Attach::new_unwatched(path.clone(), H);
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

    /// The watcher recovers if the whole parent directory is removed and
    /// recreated (e.g. a non-preserved runtime directory across restart).
    #[test]
    fn the_watcher_reattaches_after_parent_dir_recreate() {
        let dir = std::env::temp_dir().join(format!("sgv2-attach-dir-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("engine").to_string_lossy().into_owned();
        let _first = EnginePage::create(&path).unwrap();
        let a = Attach::new(path.clone(), H);
        std::thread::sleep(Duration::from_millis(20));
        std::fs::remove_dir_all(&dir).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        std::fs::create_dir_all(&dir).unwrap();
        let second = EnginePage::create(&path).unwrap();
        let t0 = std::time::Instant::now();
        while second.header().magic.load(Ordering::Acquire) != MAGIC {
            assert!(t0.elapsed() < Duration::from_secs(5), "the watcher did not recover after dir recreate");
            std::thread::sleep(Duration::from_millis(10));
        }
        beat(&a, 1);
        assert_eq!(second.header().seq.load(Ordering::Acquire), 1);
        drop(a);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Renaming a runtime directory above the watched UID directory must still
    /// wake the watcher and re-attach when the original path is recreated.
    #[test]
    fn the_watcher_reattaches_after_ancestor_dir_rename() {
        let base = std::env::temp_dir().join(format!("sgv2-attach-anc-{}", std::process::id()));
        let moved = std::env::temp_dir().join(format!("sgv2-attach-anc-moved-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        let _ = std::fs::remove_dir_all(&moved);
        let uid_dir = base.join("1000");
        std::fs::create_dir_all(&uid_dir).unwrap();
        let path = uid_dir.join("engine").to_string_lossy().into_owned();
        let _first = EnginePage::create(&path).unwrap();
        let a = Attach::new(path.clone(), H);
        // Let the watcher block in poll(-1).
        std::thread::sleep(Duration::from_millis(30));
        std::fs::rename(&base, &moved).unwrap();
        std::fs::create_dir_all(&uid_dir).unwrap();
        let second = EnginePage::create(&path).unwrap();
        let t0 = std::time::Instant::now();
        while second.header().magic.load(Ordering::Acquire) != MAGIC {
            assert!(t0.elapsed() < Duration::from_secs(5), "the watcher missed ancestor directory rename");
            std::thread::sleep(Duration::from_millis(10));
        }
        beat(&a, 1);
        assert_eq!(second.header().seq.load(Ordering::Acquire), 1);
        drop(a);
        let _ = std::fs::remove_dir_all(&base);
        let _ = std::fs::remove_dir_all(&moved);
    }

    /// A symlinked engine page path keeps the bounded poll fallback so
    /// replacing the referent in another directory still re-attaches.
    #[test]
    fn a_symlinked_page_reattaches_when_referent_is_replaced() {
        let base = std::env::temp_dir().join(format!("sgv2-attach-sym-{}", std::process::id()));
        let target_dir = base.join("target");
        let link_dir = base.join("link");
        let _ = std::fs::remove_dir_all(&base);
        std::fs::create_dir_all(&target_dir).unwrap();
        std::fs::create_dir_all(&link_dir).unwrap();
        let target = target_dir.join("engine").to_string_lossy().into_owned();
        let link = link_dir.join("engine");
        let _first = EnginePage::create(&target).unwrap();
        std::os::unix::fs::symlink(&target, &link).unwrap();
        let a = Attach::new(link.to_string_lossy().into_owned(), H);
        std::thread::sleep(Duration::from_millis(30));
        std::fs::remove_file(&target).unwrap();
        let second = EnginePage::create(&target).unwrap();
        let t0 = std::time::Instant::now();
        while second.header().magic.load(Ordering::Acquire) != MAGIC {
            assert!(t0.elapsed() < Duration::from_secs(5), "symlinked page did not re-attach");
            std::thread::sleep(Duration::from_millis(10));
        }
        beat(&a, 1);
        assert_eq!(second.header().seq.load(Ordering::Acquire), 1);
        drop(a);
        let _ = std::fs::remove_dir_all(&base);
    }

    /// A path containing `..` (where leaf and ancestor resolve to the same
    /// directory inode) falls back to polling and never loses page replacement.
    #[test]
    fn a_dotdot_page_path_reattaches_when_replaced() {
        let base = std::env::temp_dir().join(format!("sgv2-attach-dotdot-{}", std::process::id()));
        let child = base.join("child");
        let _ = std::fs::remove_dir_all(&base);
        std::fs::create_dir_all(&child).unwrap();
        let real = base.join("engine").to_string_lossy().into_owned();
        let dotdot = child.join("..").join("engine").to_string_lossy().into_owned();
        let _first = EnginePage::create(&real).unwrap();
        let a = Attach::new(dotdot, H);
        std::thread::sleep(Duration::from_millis(30));
        std::fs::remove_file(&real).unwrap();
        let second = EnginePage::create(&real).unwrap();
        let t0 = std::time::Instant::now();
        while second.header().magic.load(Ordering::Acquire) != MAGIC {
            assert!(t0.elapsed() < Duration::from_secs(5), "dotdot path did not re-attach");
            std::thread::sleep(Duration::from_millis(10));
        }
        beat(&a, 1);
        assert_eq!(second.header().seq.load(Ordering::Acquire), 1);
        drop(a);
        let _ = std::fs::remove_dir_all(&base);
    }
}
