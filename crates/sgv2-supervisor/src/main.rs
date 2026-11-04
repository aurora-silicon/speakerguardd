//! speakerguardd v2: the supervisor loop.
//!
//! ```text
//! speakerguardd --card <N> --pcm <D> --uid <UID> --params <blob>
//! ```
//!
//! - Publishes the parameter blob for the plugin
//!   (`/run/speakerguardd/params.bin`, by atomic rename) and creates the
//!   plugin's engine page for user `UID`.
//! - Takes ownership of the card's "Speaker Protection" control. On a kernel
//!   without it (no v2 interface) it exits at once and nothing changes.
//! - Every 10 ms (absolute deadlines, SCHED_FIFO when permitted, memory
//!   locked) it reads the engine page, the speaker PCM's /proc status and the
//!   session helper's report, and renews the 30 ms lease only while every
//!   check holds; any fault releases at once.
//! - Exits on losing ownership or any kernel error: closing the control file
//!   withdraws protection at once.
//!
//! TODO(lane A): compile the blob from the bundle instead of `--params`.
//! TODO(lane B): the session helper that writes `session` (links, carrier).

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
    use std::sync::atomic::Ordering;

    use sgv2_lv2::page::{EnginePage, MAGIC};
    use sgv2_supervisor::kernel::{Kernel, KernelError, Macaudio};
    use sgv2_supervisor::observe::{
        parse_pcm_status, stale_ns, EngineState, EngineView, Observer, PcmState, PcmView, SessionReport, SessionView,
    };
    use sgv2_supervisor::supervise::{Event, Fatal, Supervisor};

    const TICK_NS: u64 = 10_000_000;
    const EX_CONFIG: i32 = 78;
    const EX_UNAVAILABLE: i32 = 69;

    struct Page(EnginePage);
    impl EngineView for Page {
        fn read(&mut self) -> EngineState {
            let h = self.0.header();
            EngineState {
                valid: h.magic.load(Ordering::Acquire) == MAGIC,
                pid: h.pid.load(Ordering::Relaxed),
                params_crc: h.params_crc.load(Ordering::Relaxed),
                seq: h.seq.load(Ordering::Acquire),
                mono_ns: h.mono_ns.load(Ordering::Relaxed),
                health: h.health.load(Ordering::Relaxed),
            }
        }
    }

    struct Proc(String);
    impl PcmView for Proc {
        fn read(&mut self) -> PcmState {
            match std::fs::read_to_string(&self.0) {
                Ok(t) => parse_pcm_status(&t),
                Err(_) => PcmState::Unknown,
            }
        }
    }

    /// `links=1 carrier=1 mono_ns=<t>`, rewritten by the session helper at
    /// least every 100 ms; a report older than 250 ms counts as absent.
    struct Session(String);
    impl SessionView for Session {
        fn read(&mut self) -> SessionReport {
            let Ok(t) = std::fs::read_to_string(&self.0) else { return SessionReport::default() };
            let get =
                |k: &str| t.split_whitespace().find_map(|w| w.strip_prefix(k)).and_then(|v| v.parse::<u64>().ok());
            let fresh = get("mono_ns=").is_some_and(|m| now_ns().saturating_sub(m) < 250_000_000);
            SessionReport { present: fresh, links_ok: get("links=") == Some(1), carrier_ok: get("carrier=") == Some(1) }
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

    /// systemd notification (READY=1, WATCHDOG=1) without libsystemd.
    struct Notify(Option<std::os::unix::net::UnixDatagram>, Option<std::path::PathBuf>, Vec<u8>);
    impl Notify {
        fn new() -> Self {
            let Ok(path) = std::env::var("NOTIFY_SOCKET") else { return Notify(None, None, Vec::new()) };
            let sock = std::os::unix::net::UnixDatagram::unbound().ok();
            if let Some(abs) = path.strip_prefix('@') {
                // abstract namespace: a leading NUL
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

    pub fn run() -> i32 {
        let args: Vec<String> = std::env::args().collect();
        let (Some(card), Some(pcm), Some(uid), Some(params)) =
            (arg(&args, "--card"), arg(&args, "--pcm"), arg(&args, "--uid"), arg(&args, "--params"))
        else {
            eprintln!("usage: speakerguardd --card N --pcm D --uid UID --params BLOB");
            return EX_CONFIG;
        };
        let (Ok(card), Ok(uid)) = (card.parse::<u32>(), uid.parse::<u32>()) else {
            eprintln!("bad --card or --uid");
            return EX_CONFIG;
        };
        let blob = match std::fs::read(&params) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("{params}: {e}");
                return EX_CONFIG;
            }
        };
        let crc = match sgv2_core::blob::Header::parse(&blob) {
            Ok((h, _)) => h.payload_crc,
            Err(e) => {
                eprintln!("{params}: not a valid parameter blob ({e:?})");
                return EX_CONFIG;
            }
        };
        let run_dir = std::path::Path::new("/run/speakerguardd");
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
        let page_str = page_path.to_string_lossy().into_owned();
        let page = match EnginePage::create(&page_str) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("engine page: {e}");
                return EX_UNAVAILABLE;
            }
        };
        // the session helper's report: created empty here (the directory is
        // root's), so the user's helper can only rewrite it in place
        let session_path = user_dir.join("session");
        if let Err(e) = std::fs::write(&session_path, b"") {
            eprintln!("session file: {e}");
            return EX_UNAVAILABLE;
        }
        for path in [page_str.clone(), session_path.to_string_lossy().into_owned()] {
            if let Ok(c) = std::ffi::CString::new(path) {
                // SAFETY: paths we created; hand them to the session user.
                unsafe {
                    libc::chown(c.as_ptr(), uid, u32::MAX);
                    libc::chmod(c.as_ptr(), 0o600);
                }
            }
        }
        let granted_path = run_dir.join("granted");
        let granted_tmp = run_dir.join(".granted.new");
        let write_granted = |on: bool| {
            let _ = std::fs::write(&granted_tmp, format!("{} {}\n", on as u8, now_ns()));
            let _ = std::fs::rename(&granted_tmp, &granted_path);
        };
        write_granted(false);
        let mut kernel = match Macaudio::open(card) {
            Ok(k) => k,
            Err(KernelError::NoCard) => {
                eprintln!("card {card} has no \"Speaker Protection\" control: not a v2 kernel; exiting");
                return EX_CONFIG;
            }
            Err(e) => {
                eprintln!("taking ownership: {e:?}");
                return EX_UNAVAILABLE;
            }
        };
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
        let mut proc_ = Proc(format!("/proc/asound/card{card}/pcm{pcm}p/sub0/status"));
        let mut session = Session(user_dir.join("session").to_string_lossy().into_owned());
        // the unit's 512-frame quantum at 48 kHz bounds the heartbeat age
        let mut observer = Observer::new(crc, stale_ns(512, 48_000, TICK_NS));
        let mut sup = Supervisor::default();
        let mut deadline = now_ns();
        let notify = Notify::new();
        let mut ticks = 0u64;
        loop {
            let obs = observer.observe(now_ns(), &mut engine, &mut proc_, &mut session);
            let event = sup.tick(obs, &mut kernel);
            // "1 <mono_ns>" while granted, rewritten every tick; readers treat
            // a file older than 100 ms as 0
            write_granted(sup.granted());
            match event {
                Ok(Event::Granted) => eprintln!("protection granted"),
                Ok(Event::Released { why }) => eprintln!("protection released: {why:?}"),
                Ok(Event::HardFault(f)) => eprintln!("amp fault, protection withdrawn until restart: {f}"),
                Ok(Event::Lapsed) => eprintln!("lease lapsed; re-qualifying"),
                Ok(_) => {}
                Err(Fatal::Kernel(e)) => {
                    eprintln!("kernel: {e:?}; exiting (the kernel floor holds)");
                    let _ = kernel.release();
                    let _ = std::fs::remove_file(&granted_path);
                    return EX_UNAVAILABLE;
                }
            }
            if ticks == 0 {
                notify.send("READY=1");
            }
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
            sleep_until(deadline);
        }
    }
}
