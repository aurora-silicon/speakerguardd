# Exact kernel test model

These files exercise the Linux playback ledger and private payload ownership
implementation directly. They are not a Rust approximation or production
bypass. `SOURCE.json` records each upstream path and source digest. The newly
authored ledger, payload helper, adapter API and host adapter are dual licensed
GPL-2.0-only OR MIT; the shared syscall structure has the Linux syscall note.
No pre-existing Linux driver or KUnit implementation was relicensed or copied.

The test process builds a **host-only shared library** once, under its admitted
test job, using the minimal generated headers in `build-host.py`. Every test
run verifies the pinned source hashes. Only `cfg(test)` code builds or loads it.
Production builds need neither this library nor a C compiler or Python; the
library is not linked into or deployed with the daemon. The same kernel sources run under real kernel
primitives in KUnit. The host adapter does not open devices, register a driver,
play audio or record a microphone.

The host's transfer identifiers describe declared reference plants. They do not
qualify a real TAS HPF, firmware path or unknown speaker. Production negotiation
still requires an authenticated kernel certificate and a packaged card match.
