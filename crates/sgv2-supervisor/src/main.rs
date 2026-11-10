//! speakerguardd v2: the supervisor loop.
//!
//! ```text
//! speakerguardd [--card <N|ID>] --pcm <D> --uid <UID> --params <blob> [--selfcheck <record>]
//! ```
//!
//! - Identifies the Mac from the device tree and refuses a parameter blob
//!   compiled for another board, for another channel count, or with no
//!   operating point or CRC (exit 78, not restarted).
//! - Refuses to run without a passing first-boot self-check
//!   (speakerguardd-selfcheck, `--selfcheck`, default
//!   /var/lib/speakerguardd/selfcheck.bind) for exactly this board, kernel
//!   release and parameter blob, and grants only while the session helper
//!   reports that the protect stage was built with the wiring it proved. A
//!   board the supervisor's table does not know runs from the record's
//!   derived description.
//! - Publishes the blob for the plugin (`/run/speakerguardd/params.bin`, by
//!   atomic rename), keeps or creates the plugin's engine page for user
//!   `UID`, and gives the session helper a directory of its own
//!   (`/run/speakerguardd/<uid>/helper/`, setgid root) where it renames its
//!   report into place.
//! - Waits, with a bounded backoff and the systemd watchdog fed, for the
//!   speaker card (`--card`, by index or id; the machine's own card by
//!   default) and its speaker PCM, then takes ownership of "Speaker
//!   Protection". A card without that control is not a v2 kernel: exit 78.
//! - Every 10 ms (absolute deadlines, SCHED_FIFO when permitted, memory
//!   locked) it reads the engine page, the speaker PCM's state and the
//!   session helper's report, and renews the 30 ms lease only while every
//!   check holds; any fault releases at once. The PCM's `/proc` status can
//!   block for tens of ms while a stream closes, so a separate thread reads
//!   it, and the loop only takes that thread's latest snapshot: a snapshot
//!   older than `PCM_SNAPSHOT_MAX_NS` is no proof. When the reader sees the
//!   PCM go to XRUN, paused, suspended, unreadable or a new owner, it wakes
//!   the loop for an extra tick at once (an eventfd), so a paused or
//!   suspended stream or a new owner is released within the reader's 1 ms
//!   poll instead of at the next 10 ms tick (B2 measured 11.7 ms from XRUN
//!   to release, 31.0 ms to the floor, on 0d7ee6c). An XRUN is now held
//!   for the sink's recovery, up to the heartbeat bound (see `observe`).
//! - Exits on losing ownership or any kernel error: closing the control file
//!   withdraws protection at once.
//!
//! TODO(lane A): compile the blob from the bundle instead of `--params`.

#[cfg(target_os = "linux")]
fn main() {
    std::process::exit(linux::run());
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("speakerguardd runs on Linux only");
    std::process::exit(78);
}

#[cfg(target_os = "linux")]
mod linux {
    use std::io::Read;
    use std::os::unix::fs::PermissionsExt;
    use std::path::{Path, PathBuf};
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;

    use sgv2_lv2::page::{EnginePage, LAYOUT, MAGIC, PAGE_BYTES};
    use sgv2_supervisor::kernel::{Kernel, KernelError, Macaudio};
    use sgv2_supervisor::machine::{blob_full_scales, board_from_compatible, Machine};
    use sgv2_supervisor::observe::{
        combine, helper_is, owner_to_tgid, parse_pcm_info, parse_pcm_status, parse_session, parse_tgid,
        speaker_pcm_problem, EngineState, EngineView, Observer, PcmInfo, PcmState, PcmView, SessionReport, SessionView,
    };
    use sgv2_supervisor::selfcheck;
    use sgv2_supervisor::supervise::{Event, Expect, Fatal, Supervisor};

    const TICK_NS: u64 = 10_000_000;
    const EX_CONFIG: i32 = 78;
    const EX_UNAVAILABLE: i32 = 69;
    /// The PCM reader's period.
    const PCM_POLL_NS: u64 = 1_000_000;
    /// A PCM snapshot whose read started longer ago than this is Unknown.
    const PCM_SNAPSHOT_MAX_NS: u64 = 12_000_000;
    /// How long a status read blocked behind an open in progress reads as
    /// Opening: no stream can start before that open returns, and the read
    /// that follows it proves the owner at once. The J700's speaker PCM takes
    /// ~190 ms to open (its first open after a grant).
    const PCM_OPENING_MAX_NS: u64 = sgv2_supervisor::observe::OPENING_GRACE_NS;
    /// A helper liveness check older than this vouches for nothing.
    const HELPER_CHECK_MAX_NS: u64 = 12_000_000;
    /// Waiting for the card: 50 ms doubling to 2 s.
    const BACKOFF_MIN_MS: u64 = 50;
    const BACKOFF_MAX_MS: u64 = 2_000;
    const SELFCHECK: &str = "/var/lib/speakerguardd/selfcheck.bind";

    /// The kernel release (`uname -r`).
    fn kernel_release() -> Option<String> {
        // SAFETY: a zeroed utsname is a valid out-buffer.
        let mut u: libc::utsname = unsafe { core::mem::zeroed() };
        // SAFETY: valid out-pointer.
        if unsafe { libc::uname(&mut u) } != 0 {
            return None;
        }
        // SAFETY: uname NUL-terminates release.
        let r = unsafe { std::ffi::CStr::from_ptr(u.release.as_ptr()) };
        r.to_str().ok().map(str::to_owned)
    }

    /// The self-check record: root's, not writable by anyone else, parsed.
    fn read_selfcheck(path: &str) -> Result<selfcheck::Bind, String> {
        use std::os::unix::fs::MetadataExt;
        let md =
            std::fs::symlink_metadata(path).map_err(|e| format!("{path}: {e} (has speakerguardd-selfcheck run?)"))?;
        if !md.file_type().is_file() || md.uid() != 0 || md.mode() & 0o022 != 0 {
            return Err(format!("{path}: not a regular file of root's, writable by root only"));
        }
        let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
        selfcheck::parse(&text).map_err(|e| format!("{path}: {e}"))
    }

    struct Page(EnginePage);
    impl EngineView for Page {
        fn read(&mut self) -> EngineState {
            let h = self.0.header();
            // seq, then the fields it publishes, then seq again: a beat in
            // between is read again (the plugin never waits)
            let mut seq = h.seq.load(Ordering::Acquire);
            let mut frames = h.frames.load(Ordering::Relaxed);
            let mut mono_ns = h.mono_ns.load(Ordering::Relaxed);
            for _ in 0..3 {
                let again = h.seq.load(Ordering::Acquire);
                if again == seq {
                    break;
                }
                seq = again;
                frames = h.frames.load(Ordering::Relaxed);
                mono_ns = h.mono_ns.load(Ordering::Relaxed);
            }
            EngineState {
                valid: h.magic.load(Ordering::Acquire) == MAGIC && h.layout.load(Ordering::Relaxed) == LAYOUT,
                pid: h.pid.load(Ordering::Relaxed),
                params_crc: h.params_crc.load(Ordering::Relaxed),
                seq,
                frames,
                mono_ns,
                health: h.health.load(Ordering::Relaxed),
                min_gain_bits: h.min_gain_bits.load(Ordering::Relaxed),
            }
        }
    }

    /// The PCM reader's latest snapshot: the packed state, and when (before
    /// the read began) it was taken.
    #[derive(Default)]
    struct PcmSnapshot {
        state: AtomicU64,
        taken_ns: AtomicU64,
        /// When the read in progress began, 0 between reads.
        reading_ns: AtomicU64,
    }

    /// What the RT loop may take from the reader: the snapshot while it is
    /// fresh; while a read has been blocked since right after a Closed
    /// snapshot (the PCM's `/proc` status waits for an open in progress),
    /// Opening for at most `PCM_OPENING_MAX_NS` (the observer then tolerates
    /// the heartbeat pause the open causes); otherwise Unknown, which is no
    /// proof. Before this, the first open after a grant (its status read
    /// blocked ~190 ms on the J700) read as Unknown and released the grant
    /// it had just given, and PipeWire's start hit the floor.
    fn snapshot_view(state: PcmState, taken: u64, reading: u64, now: u64) -> PcmState {
        if taken != 0 && now.saturating_sub(taken) <= PCM_SNAPSHOT_MAX_NS {
            return state;
        }
        let opening = matches!(state, PcmState::Closed)
            && taken != 0
            && reading >= taken
            && reading - taken <= PCM_SNAPSHOT_MAX_NS
            && now.saturating_sub(reading) <= PCM_OPENING_MAX_NS;
        if opening {
            PcmState::Opening
        } else {
            PcmState::Unknown
        }
    }

    /// The RT side: never touches /proc.
    struct PcmLatest(Arc<PcmSnapshot>);
    impl PcmView for PcmLatest {
        fn read(&mut self) -> PcmState {
            let reading = self.0.reading_ns.load(Ordering::Acquire);
            let taken = self.0.taken_ns.load(Ordering::Acquire);
            let state = PcmState::unpack(self.0.state.load(Ordering::Acquire));
            snapshot_view(state, taken, reading, now_ns())
        }
    }

    fn read_small(path: &str, buf: &mut String) -> bool {
        buf.clear();
        std::fs::File::open(path).and_then(|mut f| f.read_to_string(buf)).is_ok()
    }

    /// The reader thread: every substream's `/proc` status, owners mapped
    /// from thread to process id, folded into one state.
    fn spawn_pcm_reader(
        dir: String,
        subs: u32,
        out: Arc<PcmSnapshot>,
        wake: i32,
        report: std::ffi::CString,
        uid: u32,
        live: Arc<HelperLive>,
    ) -> std::io::Result<()> {
        std::thread::Builder::new().name("sgv2-pcm".into()).spawn(move || {
            // SCHED_FIFO 10, below the loop's 70: desktop load (SCHED_OTHER)
            // cannot starve the snapshots the loop needs every tick. A
            // blocked /proc read only sleeps this thread.
            let sp = libc::sched_param { sched_priority: 10 };
            // SAFETY: this thread's own policy; failure is tolerated.
            if unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &sp) } != 0 {
                eprintln!("PCM reader: SCHED_FIFO unavailable; normal priority");
            }
            no_frequency_floor();
            let paths: Vec<String> = (0..subs).map(|s| format!("{dir}/sub{s}/status")).collect();
            let mut buf = String::with_capacity(1024);
            let mut tbuf = String::with_capacity(2048);
            let mut states = Vec::with_capacity(paths.len());
            let mut last = PcmState::Closed;
            loop {
                let started = now_ns();
                out.reading_ns.store(started, Ordering::Release);
                states.clear();
                for p in &paths {
                    let st = if read_small(p, &mut buf) { parse_pcm_status(&buf) } else { PcmState::Unknown };
                    let mut tgid = |tid: u32| {
                        read_small(&format!("/proc/{tid}/status"), &mut tbuf).then(|| parse_tgid(&tbuf)).flatten()
                    };
                    states.push(owner_to_tgid(st, &mut tgid));
                }
                let now = combine(&states);
                out.state.store(now.pack(), Ordering::Release);
                out.taken_ns.store(started, Ordering::Release);
                out.reading_ns.store(0, Ordering::Release);
                if wakes(last, now) {
                    let one: u64 = 1;
                    // SAFETY: an eventfd we own; an 8-byte counter write.
                    unsafe { libc::write(wake, (&one as *const u64).cast(), 8) };
                }
                last = now;
                sleep_until(started + PCM_POLL_NS);
            }
        })?;
        // The helper's liveness on its own thread: a PCM status read blocked
        // behind an open or close must not leave the report unvouched (the
        // J700 released its first grant of every boot and wake that way).
        std::thread::Builder::new().name("sgv2-helper".into()).spawn(move || {
            let sp = libc::sched_param { sched_priority: 10 };
            // SAFETY: this thread's own policy; failure is tolerated.
            if unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &sp) } != 0 {
                eprintln!("helper checker: SCHED_FIFO unavailable; normal priority");
            }
            no_frequency_floor();
            let mut hbuf = String::with_capacity(1024);
            loop {
                let started = now_ns();
                check_helper(&report, uid, &live, &mut hbuf);
                sleep_until(started + PCM_POLL_NS);
            }
        })?;
        Ok(())
    }

    /// Whether a change from `a` to `b` is worth a tick now: the PCM
    /// stopped, became unreadable, or changed hands.
    fn wakes(a: PcmState, b: PcmState) -> bool {
        let owner = |s: PcmState| s.owner();
        let bad = |s: PcmState| matches!(s, PcmState::Xrun { .. } | PcmState::Stopped { .. } | PcmState::Unknown);
        (bad(b) && !bad(a)) || (owner(b).is_some() && owner(a).is_some() && owner(a) != owner(b))
    }

    /// Sleep until `deadline` or a wake from the PCM reader. True if woken.
    fn wait_until(deadline: u64, wake: i32) -> bool {
        loop {
            let now = now_ns();
            if now >= deadline {
                return false;
            }
            let left = deadline - now;
            let ts = libc::timespec { tv_sec: (left / 1_000_000_000) as i64, tv_nsec: (left % 1_000_000_000) as i64 };
            let mut pfd = libc::pollfd { fd: wake, events: libc::POLLIN, revents: 0 };
            // SAFETY: one live pollfd and a valid timeout.
            let r = unsafe { libc::ppoll(&mut pfd, 1, &ts, core::ptr::null()) };
            if r > 0 && pfd.revents & libc::POLLIN != 0 {
                let mut v: u64 = 0;
                // SAFETY: clearing our eventfd's counter into a live u64.
                unsafe { libc::read(wake, (&mut v as *mut u64).cast(), 8) };
                return true;
            }
        }
    }

    /// The reader thread's verdict on the report's writer: one word
    /// (pid, the low 31 bits of its start time, alive) and the time of the
    /// check, so the RT loop never sees a half-written verdict. The checker
    /// stores the verdict before the time and the loop loads the time
    /// first, so a verdict is never older than its time. (A seqlock here
    /// failed under desktop load: a checker preempted inside its write left
    /// the RT loop's three tries all odd, and the report read as absent,
    /// B2 on the J616s at 2852.451 s.)
    #[derive(Default)]
    struct HelperLive {
        verdict: AtomicU64,
        checked_ns: AtomicU64,
    }

    fn verdict_word(pid: u32, start: u64, alive: bool) -> u64 {
        u64::from(pid) << 32 | (start & 0x7fff_ffff) << 1 | u64::from(alive)
    }

    impl HelperLive {
        fn publish(&self, pid: u32, start: u64, alive: bool, at: u64) {
            self.verdict.store(verdict_word(pid, start, alive), Ordering::Release);
            self.checked_ns.store(at, Ordering::Release);
        }

        /// Whether `helper` was checked alive within `HELPER_CHECK_MAX_NS`.
        fn vouches(&self, helper: Option<(u32, u64)>, now: u64) -> bool {
            let Some((pid, start)) = helper else { return false };
            let at = self.checked_ns.load(Ordering::Acquire);
            let v = self.verdict.load(Ordering::Acquire);
            v == verdict_word(pid, start, true) && now.saturating_sub(at) <= HELPER_CHECK_MAX_NS
        }
    }

    /// The helper's report and its writer's liveness. The report is read on
    /// the RT path (a tmpfs file, never blocking); the writer is checked
    /// against /proc by the reader thread.
    struct Session(std::ffi::CString, Arc<HelperLive>);
    impl SessionView for Session {
        fn read(&mut self) -> SessionReport {
            let mut r = read_report(&self.0);
            r.helper_alive = self.1.vouches(r.helper, now_ns());
            r
        }
    }

    /// The helper's report: opened without following links and without
    /// blocking, a regular file only, at most 255 bytes, read into a stack
    /// buffer. The helper renames each report into place, so a read sees
    /// one whole report or the one before.
    fn read_report(path: &std::ffi::CStr) -> SessionReport {
        {
            let flags = libc::O_RDONLY | libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC;
            // SAFETY: a NUL-terminated path.
            let fd = unsafe { libc::open(path.as_ptr(), flags) };
            if fd < 0 {
                return SessionReport::default();
            }
            let mut buf = [0u8; 256];
            // SAFETY: a zeroed stat is a valid out-buffer.
            let mut st: libc::stat = unsafe { core::mem::zeroed() };
            // SAFETY: fd is ours, the buffers are live and sized.
            let n = unsafe {
                if libc::fstat(fd, &mut st) == 0 && st.st_mode & libc::S_IFMT == libc::S_IFREG {
                    libc::read(fd, buf.as_mut_ptr().cast(), buf.len())
                } else {
                    -1
                }
            };
            // SAFETY: closing our own fd.
            unsafe { libc::close(fd) };
            if n <= 0 || n as usize >= buf.len() {
                return SessionReport::default();
            }
            core::str::from_utf8(&buf[..n as usize]).map(parse_session).unwrap_or_default()
        }
    }

    /// Check the report's writer: alive, started when it says, the user's.
    fn check_helper(path: &std::ffi::CStr, uid: u32, live: &HelperLive, buf: &mut String) {
        use std::os::unix::fs::MetadataExt;
        let r = read_report(path);
        let at = now_ns();
        let Some((pid, start)) = r.helper else {
            live.publish(0, 0, false, at);
            return;
        };
        let dir = format!("/proc/{pid}");
        let owner = std::fs::metadata(&dir).ok().map(|m| m.uid());
        let stat = read_small(&format!("{dir}/stat"), buf).then_some(buf.as_str());
        let alive = helper_is(&r, stat, owner, uid);
        live.publish(pid, start, alive, at);
    }

    fn now_ns() -> u64 {
        let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: valid out-pointer.
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
    }

    /// The supervisor's own hang watch. systemd's service watchdog runs on
    /// CLOCK_MONOTONIC, which keeps counting while a system sleep freezes
    /// userspace (the freezer, the device suspend and resume, a pm_test
    /// delay), so a frozen supervisor looks hung at the thaw however long
    /// WatchdogSec is. This watch counts a stall only when no system sleep
    /// happened since it last looked (the kernel's suspend counters), and
    /// aborts a supervisor whose loop really stopped; systemd then restarts
    /// it (Restart=on-failure). The unit's WatchdogSec stays as a last
    /// resort for a wedged process, set far above any freeze.
    pub(crate) const HANG_NS: u64 = 10_000_000_000;
    static HEARTBEAT_NS: AtomicU64 = AtomicU64::new(0);

    /// The loop is alive: feed systemd and the hang watch.
    fn alive(notify: &Notify) {
        HEARTBEAT_NS.store(now_ns(), Ordering::Relaxed);
        notify.send("WATCHDOG=1");
    }

    /// Every suspend attempt the kernel finished, successful or not
    /// (/sys/power/suspend_stats); None without the counters.
    fn sleeps() -> Option<u64> {
        let n =
            |f: &str| std::fs::read_to_string(format!("/sys/power/suspend_stats/{f}")).ok()?.trim().parse::<u64>().ok();
        Some(n("success")? + n("fail")?)
    }

    /// CLOCK_BOOTTIME minus CLOCK_MONOTONIC: grows by the time the system
    /// spent suspended (not by a freeze with the clock running).
    fn suspended_ns() -> u64 {
        let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: valid out-pointer.
        unsafe { libc::clock_gettime(libc::CLOCK_BOOTTIME, &mut ts) };
        (ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64).saturating_sub(now_ns())
    }

    /// The watch's decision: hung only when the heartbeat is older than the
    /// limit and no sleep happened since the last look (`slept`).
    pub(crate) fn hung(now: u64, heartbeat: u64, limit: u64, slept: bool) -> bool {
        heartbeat != 0 && !slept && now.saturating_sub(heartbeat) > limit
    }

    fn spawn_hang_watch() -> std::io::Result<()> {
        std::thread::Builder::new().name("hang-watch".into()).spawn(|| {
            let (mut seen, mut asleep) = (sleeps(), suspended_ns());
            loop {
                std::thread::sleep(std::time::Duration::from_secs(1));
                let (count, susp) = (sleeps(), suspended_ns());
                // a sleep since the last look: its freeze is not a stall
                let slept = count != seen || susp > asleep + 500_000_000;
                if slept {
                    // the loop gets a full limit after the thaw
                    HEARTBEAT_NS.store(now_ns(), Ordering::Relaxed);
                }
                seen = count;
                asleep = susp;
                let hb = HEARTBEAT_NS.load(Ordering::Relaxed);
                if hung(now_ns(), hb, HANG_NS, slept) {
                    eprintln!("supervisor loop stalled for over {} s outside any system sleep; aborting for a restart (the kernel floor holds)", HANG_NS / 1_000_000_000);
                    std::process::abort();
                }
            }
        })?;
        Ok(())
    }

    /// The kernel's `sched_attr` up to the utilization clamps (size 56).
    #[repr(C)]
    struct SchedAttr {
        size: u32,
        sched_policy: u32,
        sched_flags: u64,
        sched_nice: i32,
        sched_priority: u32,
        sched_runtime: u64,
        sched_deadline: u64,
        sched_period: u64,
        sched_util_min: u32,
        sched_util_max: u32,
    }
    const SCHED_FLAG_KEEP_POLICY: u64 = 0x08;
    const SCHED_FLAG_KEEP_PARAMS: u64 = 0x10;
    const SCHED_FLAG_UTIL_CLAMP_MIN: u64 = 0x20;

    /// Drop this thread's utilization floor to 0, keeping its policy and
    /// priority. A real-time thread with no clamp of its own inherits
    /// `sched_util_clamp_min_rt_default` (1024), and schedutil then runs its
    /// cluster at the top frequency every time it wakes. The pollers wake
    /// every millisecond for a few microseconds of /proc reads, which held
    /// the M2's performance cluster at 2.4-3.2 GHz with nothing playing.
    /// They need the CPU on time, not fast. Failure (no uclamp support) is
    /// harmless and ignored.
    fn no_frequency_floor() {
        let attr = SchedAttr {
            size: core::mem::size_of::<SchedAttr>() as u32,
            sched_policy: 0,
            sched_flags: SCHED_FLAG_KEEP_POLICY | SCHED_FLAG_KEEP_PARAMS | SCHED_FLAG_UTIL_CLAMP_MIN,
            sched_nice: 0,
            sched_priority: 0,
            sched_runtime: 0,
            sched_deadline: 0,
            sched_period: 0,
            sched_util_min: 0,
            sched_util_max: 0,
        };
        // SAFETY: this thread's own attributes; a valid, fully initialised
        // sched_attr whose size field matches the struct.
        unsafe { libc::syscall(libc::SYS_sched_setattr, 0, &attr as *const SchedAttr, 0u32) };
    }

    fn sleep_until(t: u64) {
        let ts = libc::timespec { tv_sec: (t / 1_000_000_000) as i64, tv_nsec: (t % 1_000_000_000) as i64 };
        // SAFETY: valid timespec; absolute monotonic deadline.
        unsafe { libc::clock_nanosleep(libc::CLOCK_MONOTONIC, libc::TIMER_ABSTIME, &ts, core::ptr::null_mut()) };
    }

    /// systemd notification (READY=1, WATCHDOG=1, STATUS=) without libsystemd.
    struct Notify(Option<std::os::unix::net::UnixDatagram>, Option<PathBuf>, Vec<u8>);
    impl Notify {
        fn new() -> Self {
            let Ok(path) = std::env::var("NOTIFY_SOCKET") else { return Notify(None, None, Vec::new()) };
            let sock = std::os::unix::net::UnixDatagram::unbound().ok();
            if let Some(abs) = path.strip_prefix('@') {
                let mut name = vec![0u8];
                name.extend_from_slice(abs.as_bytes());
                Notify(sock, None, name)
            } else {
                Notify(sock, Some(path.into()), Vec::new())
            }
        }
        fn send(&self, msg: &str) {
            let Some(s) = &self.0 else { return };
            if let Some(p) = &self.1 {
                let _ = s.send_to(msg.as_bytes(), p);
            } else {
                use std::os::linux::net::SocketAddrExt;
                if let Ok(a) = std::os::unix::net::SocketAddr::from_abstract_name(&self.2[1..]) {
                    let _ = s.send_to_addr(msg.as_bytes(), &a);
                }
            }
        }
    }

    fn arg(args: &[String], name: &str) -> Option<String> {
        args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned()
    }

    /// The card index for `--card` (an index or an ALSA card id), once the
    /// card is there.
    fn card_index(card: &str) -> Option<u32> {
        if let Ok(n) = card.parse::<u32>() {
            return Path::new(&format!("/proc/asound/card{n}")).exists().then_some(n);
        }
        let link = std::fs::read_link(format!("/proc/asound/{card}")).ok()?;
        link.to_str()?.strip_prefix("card")?.parse().ok()
    }

    enum Wait {
        /// Not there yet, or refused: try again.
        Retry(String),
        /// Wrong for good: exit 78.
        Config(String),
    }

    /// One attempt at the card: its identity, the speaker PCM, ownership.
    fn try_card(machine: &Machine, card: &str, pcm: u32) -> Result<(Macaudio, u32, PcmInfo), Wait> {
        let Some(index) = card_index(card) else { return Err(Wait::Retry(format!("no card {card} yet"))) };
        let id = std::fs::read_to_string(format!("/proc/asound/card{index}/id")).unwrap_or_default();
        if id.trim() != machine.card_id {
            return Err(Wait::Config(format!("card {index} is {:?}, not {}", id.trim(), machine.card_id)));
        }
        let info_path = format!("/proc/asound/card{index}/pcm{pcm}p/info");
        let Ok(text) = std::fs::read_to_string(&info_path) else {
            return Err(Wait::Retry(format!("{info_path}: not there yet")));
        };
        let Some(info) = parse_pcm_info(&text) else { return Err(Wait::Config(format!("{info_path}: unreadable"))) };
        if let Some(p) = speaker_pcm_problem(&info, index, pcm) {
            return Err(Wait::Config(format!("--pcm {pcm}: {p}")));
        }
        match Macaudio::open(index) {
            Ok(k) => Ok((k, index, info)),
            Err(KernelError::NotV2) => {
                Err(Wait::Config(format!("card {index} has no \"Speaker Protection\" control: not a v2 kernel")))
            }
            Err(e) if e.retryable() => Err(Wait::Retry(format!("taking ownership of card {index}: {e:?}"))),
            Err(e) => Err(Wait::Config(format!("taking ownership of card {index}: {e:?}"))),
        }
    }

    /// The engine page: kept across restarts (the unit preserves the runtime
    /// directory), so a plugin that already maps it keeps writing to the
    /// file the supervisor reads. A file that is not a regular page of the
    /// right size is replaced; a live mapping is never truncated.
    ///
    /// It is the session user's, group root, 0660: the service has no
    /// CAP_DAC_OVERRIDE, so a restarted supervisor opens the page it handed
    /// to the user through the group. Ownership and mode are set (CAP_CHOWN,
    /// CAP_FOWNER) before the open, so a page left 0600 by an earlier
    /// version is reopened too (B2: "engine page: Permission denied" after
    /// every restart on 6b0b0e3).
    fn engine_page(path: &Path, uid: u32) -> std::io::Result<EnginePage> {
        let s = path.to_string_lossy().into_owned();
        let hand_over = |p: &Path| -> std::io::Result<()> {
            std::os::unix::fs::lchown(p, Some(uid), Some(0))?;
            std::fs::set_permissions(p, std::fs::Permissions::from_mode(0o660))
        };
        match std::fs::symlink_metadata(path) {
            Ok(m) if m.file_type().is_file() && m.len() == PAGE_BYTES as u64 => {
                hand_over(path)?;
                EnginePage::open(&s)
            }
            Ok(_) => {
                std::fs::remove_file(path)?;
                let page = EnginePage::create(&s)?;
                hand_over(path)?;
                Ok(page)
            }
            Err(_) => {
                let page = EnginePage::create(&s)?;
                hand_over(path)?;
                Ok(page)
            }
        }
    }

    /// The thermal hand-off across engine and PipeWire restarts (see the
    /// `thermal` module): every 100 ms take the engine's `saved` record if it
    /// is trusted, offer it in the page's `provided` slot, and persist it to
    /// `<user dir>/thermal` (root's, 0600) at most once a second. At start,
    /// offer the persisted record if this boot, board, blob and geometry
    /// match and it is at most ten minutes old. Normal priority, off the RT
    /// loop.
    fn spawn_thermal(
        page_path: PathBuf,
        file: PathBuf,
        header: sgv2_core::blob::Header,
        board: [u8; 8],
        engine_crc: u32,
    ) {
        use sgv2_supervisor::thermal::{parse_boot_id, trusted, Persisted};
        let _ = std::thread::Builder::new().name("sgv2-thermal".into()).spawn(move || {
            let Ok(page) = EnginePage::open(&page_path.to_string_lossy()) else {
                eprintln!("thermal: cannot map the engine page; no hand-off across restarts");
                return;
            };
            let boot_id =
                std::fs::read_to_string("/proc/sys/kernel/random/boot_id").ok().and_then(|t| parse_boot_id(&t));
            let now = sgv2_lv2::handoff::boottime_ns;
            // Live trust names the complete deployment; persistent thermal
            // state remains tied to the protection payload across voicing updates.
            let engine_header = sgv2_core::blob::Header { payload_crc: engine_crc, ..header };
            match (boot_id, std::fs::read(&file).ok().and_then(|b| Persisted::decode(&b))) {
                (Some(id), Some(p)) => match p.refusal(&id, &board, &header, now()) {
                    None => {
                        page.provide(&p.record, p.stamp_boottime_ns, engine_crc);
                        eprintln!(
                            "thermal: offering the state saved {} s ago",
                            now().saturating_sub(p.stamp_boottime_ns) / 1_000_000_000
                        );
                    }
                    Some(why) => eprintln!("thermal: not restoring the saved state: {why}"),
                },
                (None, _) => eprintln!("thermal: no boot id; the state is not persisted"),
                _ => {}
            }
            let tmp = file.with_extension("new");
            let mut last_stamp = 0u64;
            let mut last_write = 0u64;
            loop {
                let h = page.header();
                if let Some((rec, stamp)) = h.saved.read() {
                    let ok = trusted(
                        &rec,
                        &engine_header,
                        h.params_crc.load(Ordering::Acquire),
                        h.health.load(Ordering::Acquire),
                    );
                    if ok && stamp != last_stamp {
                        last_stamp = stamp;
                        page.provide(&rec, stamp, engine_crc);
                        if let Some(id) = boot_id {
                            if now().saturating_sub(last_write) >= 1_000_000_000 {
                                last_write = now();
                                let p = Persisted {
                                    boot_id: id,
                                    board,
                                    params_crc: header.payload_crc,
                                    stamp_boottime_ns: stamp,
                                    record: rec,
                                };
                                let written = std::fs::write(&tmp, p.encode())
                                    .and_then(|()| {
                                        std::fs::set_permissions(&tmp, std::fs::Permissions::from_mode(0o600))
                                    })
                                    .and_then(|()| std::fs::rename(&tmp, &file));
                                if let Err(e) = written {
                                    eprintln!("thermal: {e}");
                                }
                            }
                        }
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
        });
    }

    /// The helper's own directory: the user's, group root, setgid, 02750,
    /// so its reports (created 0640) are readable by the supervisor through
    /// the group, without CAP_DAC_OVERRIDE.
    fn helper_dir(dir: &Path, uid: u32) -> std::io::Result<()> {
        match std::fs::symlink_metadata(dir) {
            Ok(m) if m.file_type().is_dir() => {}
            Ok(_) => {
                std::fs::remove_file(dir)?;
                std::fs::create_dir(dir)?;
            }
            Err(_) => std::fs::create_dir(dir)?,
        }
        std::os::unix::fs::lchown(dir, Some(uid), Some(0))?;
        std::fs::set_permissions(dir, std::fs::Permissions::from_mode(0o2750))?;
        Ok(())
    }

    pub fn run() -> i32 {
        let args: Vec<String> = std::env::args().collect();
        let (Some(pcm), Some(uid), Some(params)) = (arg(&args, "--pcm"), arg(&args, "--uid"), arg(&args, "--params"))
        else {
            eprintln!("usage: speakerguardd [--card N|ID] --pcm D --uid UID --params BLOB");
            return EX_CONFIG;
        };
        let (Ok(pcm), Ok(uid)) = (pcm.parse::<u32>(), uid.parse::<u32>()) else {
            eprintln!("bad --pcm or --uid");
            return EX_CONFIG;
        };
        let compatible = std::fs::read("/proc/device-tree/compatible").unwrap_or_default();
        let Some(board) = board_from_compatible(&compatible) else {
            eprintln!("no Apple board in /proc/device-tree/compatible");
            return EX_CONFIG;
        };
        let bind = match read_selfcheck(&arg(&args, "--selfcheck").unwrap_or_else(|| SELFCHECK.to_owned())) {
            Ok(b) => b,
            Err(why) => {
                eprintln!("no self-check record: {why}; not granting (the kernel floor stays)");
                return EX_CONFIG;
            }
        };
        let machine = match selfcheck::machine_for(&board, &bind) {
            Ok(m) => m,
            Err(why) => {
                eprintln!("{why}; not granting (the kernel floor stays)");
                return EX_CONFIG;
            }
        };
        let card = arg(&args, "--card").unwrap_or_else(|| machine.card_id.to_owned());

        let blob = match std::fs::read(&params) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("{params}: {e}");
                return EX_CONFIG;
            }
        };
        let header = match sgv2_core::blob::Header::parse(&blob) {
            Ok((h, _)) => h,
            Err(e) => {
                eprintln!("{params}: not a valid parameter blob ({e:?})");
                return EX_CONFIG;
            }
        };
        let refuse = |why: String| {
            eprintln!("{params}: {why}; refusing it (the kernel floor stays)");
            EX_CONFIG
        };
        if header.machine() != Some(board.as_str()) {
            return refuse(format!("compiled for {:?}, this Mac is {board}", header.machine()));
        }
        if header.channels as usize != machine.amps {
            return refuse(format!("{} channels, {board} has {} amps", header.channels, machine.amps));
        }
        if header.full_scale_mv == 0 {
            return refuse("states no operating point".into());
        }
        if header.payload_crc == 0 {
            return refuse("payload CRC 0".into());
        }
        if !(8_000..=192_000).contains(&header.sample_rate) {
            return refuse(format!("sample rate {}", header.sample_rate));
        }
        // The Mac's own filtering ahead of the unit: the plugin runs it and
        // refuses a blob whose pre-stage is bad, so the supervisor does too.
        if let Err(e) = sgv2_core::blob::voicing(&blob) {
            return refuse(format!("voicing rejected ({e:?})"));
        }
        let crossover = match sgv2_core::blob::prestage(&blob, header.sample_rate as f64) {
            Ok(p) => p.from_tuning(),
            Err(e) => return refuse(format!("pre-stage rejected ({e:?})")),
        };
        let full_scales = match blob_full_scales(&blob) {
            Ok(v) => v,
            Err(why) => return refuse(why),
        };
        let engine_crc = match sgv2_core::blob::engine_crc(&blob) {
            Ok(crc) if crc != 0 => crc,
            _ => return refuse("invalid deployment identity".into()),
        };
        let params_sha256 = aurora_speaker_tuning::manifest::sha256_hex(&blob);
        let Some(kernel_rel) = kernel_release() else {
            eprintln!("uname failed");
            return EX_UNAVAILABLE;
        };
        if let Err(why) = selfcheck::covers(&bind, &board, &kernel_rel, &params_sha256) {
            eprintln!("{why}; not granting (the kernel floor stays)");
            return EX_CONFIG;
        }
        eprintln!(
            "self-check: {} wiring {:016x} ({}), kernel {}, params {}",
            board,
            bind.wiring,
            bind.method,
            bind.kernel,
            &params_sha256[..12]
        );

        let run_dir = Path::new("/run/speakerguardd");
        let user_dir = run_dir.join(uid.to_string());
        let publish = || -> std::io::Result<()> {
            std::fs::create_dir_all(&user_dir)?;
            let tmp = run_dir.join(".params.bin.new");
            std::fs::write(&tmp, &blob)?;
            std::fs::rename(&tmp, run_dir.join("params.bin"))?;
            Ok(())
        };
        if let Err(e) = publish() {
            eprintln!("publishing parameters: {e}");
            return EX_UNAVAILABLE;
        }
        let page_path = user_dir.join("engine");
        let page = match engine_page(&page_path, uid) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("engine page: {e}");
                return EX_UNAVAILABLE;
            }
        };
        let board_field = sgv2_core::blob::machine_field(&board).unwrap_or([0; 8]);
        spawn_thermal(page_path.clone(), user_dir.join("thermal"), header, board_field, engine_crc);
        let helper = user_dir.join("helper");
        if let Err(e) = helper_dir(&helper, uid) {
            eprintln!("helper directory: {e}");
            return EX_UNAVAILABLE;
        }
        // the in-place report and the granted flag of earlier versions
        let _ = std::fs::remove_file(user_dir.join("session"));
        let _ = std::fs::remove_file(run_dir.join("granted"));
        let _ = std::fs::remove_file(run_dir.join(".granted.new"));
        let Ok(session_path) = std::ffi::CString::new(helper.join("session").to_string_lossy().into_owned()) else {
            return EX_UNAVAILABLE;
        };

        // Up, holding nothing: the kernel floor is the state. The watchdog
        // guards the wait for the card as well as the loop.
        let notify = Notify::new();
        HEARTBEAT_NS.store(now_ns(), Ordering::Relaxed);
        if let Err(e) = spawn_hang_watch() {
            eprintln!("hang watch: {e}");
            return EX_UNAVAILABLE;
        }
        notify.send("READY=1\nSTATUS=waiting for the speaker card");
        let mut attempt = 0u32;
        let mut last_why = String::new();
        let (mut kernel, index, info) = loop {
            match try_card(machine, &card, pcm) {
                Ok((mut k, index, info)) => {
                    // Before any grant: the tweeters' amp high-pass opens
                    // only for an engine that runs the Mac's crossover.
                    match k.declare_crossover(crossover) {
                        Ok(true) => eprintln!("crossover declared to the kernel: {crossover}"),
                        Ok(false) => {
                            eprintln!("kernel has no crossover declaration; it keeps its own high-pass policy")
                        }
                        Err(e) => {
                            eprintln!("declaring the crossover: {e:?}; exiting (the kernel floor stays)");
                            return EX_CONFIG;
                        }
                    }
                    break (k, index, info);
                }
                Err(Wait::Config(why)) => {
                    eprintln!("{why}; exiting");
                    return EX_CONFIG;
                }
                Err(Wait::Retry(why)) => {
                    if why != last_why {
                        eprintln!("{why}; waiting");
                        notify.send(&format!("STATUS={why}"));
                        last_why = why;
                    }
                    let ms = (BACKOFF_MIN_MS << attempt.min(6)).min(BACKOFF_MAX_MS);
                    attempt += 1;
                    let until = now_ns() + ms * 1_000_000;
                    loop {
                        alive(&notify);
                        let now = now_ns();
                        if now >= until {
                            break;
                        }
                        sleep_until((now + 200_000_000).min(until));
                    }
                }
            }
        };
        eprintln!(
            "{board}: card {index} ({}), speaker PCM {pcm} \"{}\" ({} substreams), blob {:#010x} at {} mV full scale",
            machine.card_id, info.id, info.subdevices, header.payload_crc, header.full_scale_mv
        );

        let snapshot = Arc::new(PcmSnapshot::default());
        let live = Arc::new(HelperLive::default());
        // SAFETY: a plain eventfd, non-blocking and close-on-exec.
        let wake = unsafe { libc::eventfd(0, libc::EFD_NONBLOCK | libc::EFD_CLOEXEC) };
        if wake < 0 {
            eprintln!("eventfd: {}", std::io::Error::last_os_error());
            return EX_UNAVAILABLE;
        }
        // before SCHED_FIFO: the reader runs at normal priority
        let pcm_dir = format!("/proc/asound/card{index}/pcm{pcm}p");
        if let Err(e) =
            spawn_pcm_reader(pcm_dir, info.subdevices, snapshot.clone(), wake, session_path.clone(), uid, live.clone())
        {
            eprintln!("PCM reader: {e}");
            return EX_UNAVAILABLE;
        }
        // SAFETY: process-wide calls; failure is tolerated and logged.
        unsafe {
            if libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) != 0 {
                eprintln!("mlockall failed; continuing");
            }
            let sp = libc::sched_param { sched_priority: 70 };
            if libc::sched_setscheduler(0, libc::SCHED_FIFO, &sp) != 0 {
                eprintln!("SCHED_FIFO unavailable; continuing at normal priority");
            }
        }
        let mut engine = Page(page);
        let mut pcm_view = PcmLatest(snapshot);
        let mut session = Session(session_path, live);
        let mut observer = Observer::new(engine_crc, header.sample_rate, TICK_NS);
        observer.expect_wiring(bind.wiring);
        let mut sup = Supervisor::new(Expect::new(machine, full_scales, &info.id));
        let mut deadline = now_ns();
        let mut ticks = 0u64;
        let mut last_problem: Option<String> = None;
        let mut last_wiring: Option<Option<u64>> = None;
        notify.send("STATUS=supervising");
        let mut scheduled = true;
        // CLOCK_BOOTTIME - CLOCK_MONOTONIC grows only while the system is
        // suspended: a jump since the last tick means the engine's last beat
        // predates a sleep (see observe's module notes).
        // The kernel's suspend counters catch a pm_test sleep (the clocks
        // keep running) too; reading them is two small sysfs files a tick.
        let mut suspended = suspended_ns();
        let mut slept = sleeps();
        let mut waiting_after_sleep = false;
        loop {
            let (s, n) = (suspended_ns(), sleeps());
            if s > suspended.saturating_add(1_000_000) || n != slept {
                observer.note_sleep();
            }
            (suspended, slept) = (s, n);
            let obs = observer.observe(&mut now_ns, &mut engine, &mut pcm_view, &mut session);
            if observer.waiting_after_sleep() != waiting_after_sleep {
                waiting_after_sleep = observer.waiting_after_sleep();
                eprintln!(
                    "{}",
                    if waiting_after_sleep {
                        "system sleep: waiting for the engine's first beat after the thaw before trusting its heartbeat"
                    } else {
                        "the engine beats again after the sleep"
                    }
                );
            }
            let event = sup.tick(obs, &mut kernel);
            match event {
                Ok(Event::Granted) => {
                    observer.note_grant(now_ns());
                    eprintln!("protection granted");
                }
                Ok(Event::Released { why }) => {
                    let session = observer.session_why();
                    eprintln!(
                        "protection released: {why:?}{}{session}",
                        if session.is_empty() { "" } else { "; session: " }
                    );
                }
                Ok(Event::HardFault(f)) => eprintln!("amp fault, protection withdrawn until restart: {f}"),
                Ok(Event::Lapsed) => eprintln!("lease lapsed; re-qualifying"),
                Ok(_) => {}
                Err(Fatal::Kernel(e)) => {
                    eprintln!("kernel: {e:?}; exiting (the kernel floor holds)");
                    let _ = kernel.release();
                    return EX_UNAVAILABLE;
                }
            }
            if observer.wrong_wiring() != last_wiring {
                if let Some(w) = observer.wrong_wiring() {
                    let named = w.map_or("no wiring".to_owned(), |w| format!("wiring {w:016x}"));
                    eprintln!(
                        "not granting: the protect stage was built with {named}, the self-check proved {:016x}",
                        bind.wiring
                    );
                }
                last_wiring = observer.wrong_wiring();
            }
            if sup.problem() != last_problem.as_deref() {
                if let Some(p) = sup.problem() {
                    eprintln!("not granting: {p}");
                }
                last_problem = sup.problem().map(str::to_owned);
            }
            if scheduled {
                if ticks % 20 == 0 {
                    // every 200 ms: systemd and the hang watch
                    alive(&notify);
                }
                ticks += 1;
                deadline += TICK_NS;
                let now = now_ns();
                if now > deadline + TICK_NS {
                    // overran by more than a tick: resynchronise, never burst
                    deadline = now;
                }
            }
            // an extra tick when the PCM reader wakes us; the 10 ms deadlines
            // are unchanged
            scheduled = !wait_until(deadline, wake);
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn a_freeze_for_a_sleep_is_not_a_hang() {
            let s = 1_000_000_000u64;
            // fresh heartbeat: alive
            assert!(!hung(100 * s, 99 * s, HANG_NS, false));
            // stalled past the limit with no sleep: hung
            assert!(hung(120 * s, 99 * s, HANG_NS, false));
            // the same gap across a system sleep (pm_test or a long device suspend): not a hang
            assert!(!hung(140 * s, 99 * s, HANG_NS, true));
            // not started yet
            assert!(!hung(140 * s, 0, HANG_NS, false));
            // exactly at the limit: not yet
            assert!(!hung(109 * s, 99 * s, HANG_NS, false));
        }

        #[test]
        fn a_status_read_blocked_by_an_open_reads_as_opening_for_a_while() {
            let ms = 1_000_000u64;
            let closed = PcmState::Closed;
            // fresh: as read
            assert_eq!(snapshot_view(closed, 100 * ms, 0, 105 * ms), closed);
            // stale and no read in progress (a starved reader): no proof
            assert_eq!(snapshot_view(closed, 100 * ms, 0, 120 * ms), PcmState::Unknown);
            // a read blocked since right after a Closed snapshot: Opening for up to the bound
            assert_eq!(snapshot_view(closed, 100 * ms, 101 * ms, 300 * ms), PcmState::Opening);
            assert_eq!(snapshot_view(closed, 100 * ms, 101 * ms, 101 * ms + PCM_OPENING_MAX_NS), PcmState::Opening);
            assert_eq!(snapshot_view(closed, 100 * ms, 101 * ms, 102 * ms + PCM_OPENING_MAX_NS), PcmState::Unknown);
            // a read that began long after the last snapshot proves nothing
            assert_eq!(snapshot_view(closed, 100 * ms, 140 * ms, 150 * ms), PcmState::Unknown);
            // never taken
            assert_eq!(snapshot_view(closed, 0, 1, 2), PcmState::Unknown);
        }

        #[test]
        fn a_blocked_read_after_an_open_stream_is_no_proof() {
            let ms = 1_000_000u64;
            let open = parse_pcm_status("state: RUNNING\nowner_pid   : 42\n");
            assert_ne!(open, PcmState::Closed);
            assert_eq!(snapshot_view(open, 100 * ms, 105 * ms, 110 * ms), open);
            assert_eq!(snapshot_view(open, 100 * ms, 101 * ms, 150 * ms), PcmState::Unknown);
        }

        #[test]
        fn the_helper_verdict_is_one_word_and_never_reads_half_written() {
            let live = HelperLive::default();
            assert!(!live.vouches(Some((77, 1234)), 10));
            live.publish(77, 1234, true, 1_000);
            assert!(live.vouches(Some((77, 1234)), 1_000 + HELPER_CHECK_MAX_NS));
            // stale, another pid, another start, or dead
            assert!(!live.vouches(Some((77, 1234)), 1_001 + HELPER_CHECK_MAX_NS));
            assert!(!live.vouches(Some((78, 1234)), 1_000));
            assert!(!live.vouches(Some((77, 1235)), 1_000));
            assert!(!live.vouches(None, 1_000));
            live.publish(77, 1234, false, 2_000);
            assert!(!live.vouches(Some((77, 1234)), 2_000));
            // a checker preempted between its two stores leaves the last
            // whole verdict, not a torn one
            live.publish(77, 1234, true, 3_000);
            live.verdict.store(verdict_word(77, 1234, true), Ordering::Release);
            assert!(live.vouches(Some((77, 1234)), 3_000));
        }
    }
}
