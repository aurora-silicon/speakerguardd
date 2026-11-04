//! speakerguardd v2. TODO(lane D and the supervisor): the 10 ms loop,
//! bundle loading and blob publication, heartbeat reading, and the kernel
//! implementation. Until those exist the binary refuses to run, so it can
//! never hold the interlock without protection behind it.

fn main() {
    eprintln!("speakerguardd v2 is not functional yet; the kernel safe level stays in force");
    std::process::exit(78); // EX_CONFIG: systemd will not restart-loop on it.
}
