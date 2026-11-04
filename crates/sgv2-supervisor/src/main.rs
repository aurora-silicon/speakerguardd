//! speakerguardd v2: the supervisor loop.
//!
//! ```text
//! speakerguardd [--card <N|ID>] --pcm <D> --uid <UID> --params <blob>
//! ```
//!
//! - Identifies the Mac from the device tree and refuses a parameter blob
//!   compiled for another board, for another channel count, or with no
//!   operating point or CRC (exit 78, not restarted).
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
//!   the loop for an extra tick at once (an eventfd), so a stopped engine is
//!   released within the reader's 1 ms poll instead of at the next 10 ms
//!   tick (B2 measured 11.7 ms from XRUN to release, 31.0 ms to the floor,
//!   on 0d7ee6c).
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
    use sgv2_supervisor::machine::{blob_full_scales, board_from_compatible, lookup, Machine};
    use sgv2_supervisor::observe::{
        combine, owner_to_tgid, parse_pcm_info, parse_pcm_status, parse_session, parse_tgid, speaker_pcm_problem,
        EngineState, EngineView, Observer, PcmInfo, PcmState, PcmView, SessionReport, SessionView,
    };
    use sgv2_supervisor::supervise::{Event, Expect, Fatal, Supervisor};

    const TICK_NS: u64 = 10_000_000;
    const EX_CONFIG: i32 = 78;
    const EX_UNAVAILABLE: i32 = 69;
    /// The PCM reader's period.
    const PCM_POLL_NS: u64 = 1_000_000;
    /// A PCM snapshot whose read started longer ago than this is Unknown.
    const PCM_SNAPSHOT_MAX_NS: u64 = 12_000_000;
    /// Waiting for the card: 50 ms doubling to 2 s.
    const BACKOFF_MIN_MS: u64 = 50;
    const BACKOFF_MAX_MS: u64 = 2_000;

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
    }

    /// The RT side: never touches /proc.
    struct PcmLatest(Arc<PcmSnapshot>);
    impl PcmView for PcmLatest {
        fn read(&mut self) -> PcmState {
            let taken = self.0.taken_ns.load(Ordering::Acquire);
            let state = PcmState::unpack(self.0.state.load(Ordering::Acquire));
            if taken == 0 || now_ns().saturating_sub(taken) > PCM_SNAPSHOT_MAX_NS {
                return PcmState::Unknown;
            }
            state
        }
    }

    fn read_small(path: &str, buf: &mut String) -> bool {
        buf.clear();
        std::fs::File::open(path).and_then(|mut f| f.read_to_string(buf)).is_ok()
    }

    /// The reader thread: every substream's `/proc` status, owners mapped
    /// from thread to process id, folded into one state.
    fn spawn_pcm_reader(dir: String, subs: u32, out: Arc<PcmSnapshot>, wake: i32) -> std::io::Result<()> {
        std::thread::Builder::new().name("sgv2-pcm".into()).spawn(move || {
            let paths: Vec<String> = (0..subs).map(|s| format!("{dir}/sub{s}/status")).collect();
            let mut buf = String::with_capacity(1024);
            let mut tbuf = String::with_capacity(2048);
            let mut states = Vec::with_capacity(paths.len());
            let mut last = PcmState::Closed;
            loop {
                let started = now_ns();
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
                if wakes(last, now) {
                    let one: u64 = 1;
                    // SAFETY: an eventfd we own; an 8-byte counter write.
                    unsafe { libc::write(wake, (&one as *const u64).cast(), 8) };
                }
                last = now;
                sleep_until(started + PCM_POLL_NS);
            }
        })?;
        Ok(())
    }

    /// Whether a change from `a` to `b` is worth a tick now: the PCM
    /// stopped, became unreadable, or changed hands.
    fn wakes(a: PcmState, b: PcmState) -> bool {
        let owner = |s: PcmState| match s {
            PcmState::Open { owner } | PcmState::Stopped { owner } | PcmState::Running { owner } => Some(owner),
            _ => None,
        };
        let bad = |s: PcmState| matches!(s, PcmState::Stopped { .. } | PcmState::Unknown);
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

    /// The helper's report: opened without following links and without
    /// blocking, a regular file only, at most 255 bytes, read into a stack
    /// buffer. The helper renames each report into place, so a read sees
    /// one whole report or the one before.
    struct Session(std::ffi::CString);
    impl SessionView for Session {
        fn read(&mut self) -> SessionReport {
            let flags = libc::O_RDONLY | libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC;
            // SAFETY: a NUL-terminated path.
            let fd = unsafe { libc::open(self.0.as_ptr(), flags) };
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

    fn now_ns() -> u64 {
        let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: valid out-pointer.
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
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
        let Some(machine) = lookup(&board) else {
            eprintln!("{board}: not a machine speakerguardd v2 supports");
            return EX_CONFIG;
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
        let full_scales = match blob_full_scales(&blob) {
            Ok(v) => v,
            Err(why) => return refuse(why),
        };

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
        notify.send("READY=1\nSTATUS=waiting for the speaker card");
        let mut attempt = 0u32;
        let mut last_why = String::new();
        let (mut kernel, index, info) = loop {
            match try_card(machine, &card, pcm) {
                Ok(k) => break k,
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
                        notify.send("WATCHDOG=1");
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
        // SAFETY: a plain eventfd, non-blocking and close-on-exec.
        let wake = unsafe { libc::eventfd(0, libc::EFD_NONBLOCK | libc::EFD_CLOEXEC) };
        if wake < 0 {
            eprintln!("eventfd: {}", std::io::Error::last_os_error());
            return EX_UNAVAILABLE;
        }
        // before SCHED_FIFO: the reader runs at normal priority
        let pcm_dir = format!("/proc/asound/card{index}/pcm{pcm}p");
        if let Err(e) = spawn_pcm_reader(pcm_dir, info.subdevices, snapshot.clone(), wake) {
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
        let mut session = Session(session_path);
        let mut observer = Observer::new(header.payload_crc, header.sample_rate, TICK_NS);
        let mut sup = Supervisor::new(Expect::new(machine, full_scales, &info.id));
        let mut deadline = now_ns();
        let mut ticks = 0u64;
        let mut last_problem: Option<String> = None;
        notify.send("STATUS=supervising");
        let mut scheduled = true;
        loop {
            let obs = observer.observe(&mut now_ns, &mut engine, &mut pcm_view, &mut session);
            let event = sup.tick(obs, &mut kernel);
            match event {
                Ok(Event::Granted) => {
                    observer.note_grant(now_ns());
                    eprintln!("protection granted");
                }
                Ok(Event::Released { why }) => eprintln!("protection released: {why:?}"),
                Ok(Event::HardFault(f)) => eprintln!("amp fault, protection withdrawn until restart: {f}"),
                Ok(Event::Lapsed) => eprintln!("lease lapsed; re-qualifying"),
                Ok(_) => {}
                Err(Fatal::Kernel(e)) => {
                    eprintln!("kernel: {e:?}; exiting (the kernel floor holds)");
                    let _ = kernel.release();
                    return EX_UNAVAILABLE;
                }
            }
            if sup.problem() != last_problem.as_deref() {
                if let Some(p) = sup.problem() {
                    eprintln!("not granting: {p}");
                }
                last_problem = sup.problem().map(str::to_owned);
            }
            if scheduled {
                if ticks % 20 == 0 {
                    // every 200 ms, under the unit's WatchdogSec
                    notify.send("WATCHDOG=1");
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
}
