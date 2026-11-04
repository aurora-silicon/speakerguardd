//! Read what the supervisor checks on a speaker card, without owning it:
//! the speaker mux, every amp's gain and volume with their dB, and the
//! fault controls. `card-probe <card index>`. Reads only.

#[cfg(target_os = "linux")]
fn main() {
    use sgv2_supervisor::kernel::{Kernel, Macaudio, GAIN_CONTROL_SUFFIX};
    let card: u32 = std::env::args().nth(1).and_then(|a| a.parse().ok()).expect("usage: card-probe <card index>");
    let mut k = Macaudio::open_unowned(card).expect("open the control device");
    println!("speaker mux: {:?}", k.speaker_route());
    for suffix in [GAIN_CONTROL_SUFFIX, "Speaker Volume", "Speaker Playback Volume"] {
        for l in k.levels(suffix).unwrap_or_default() {
            println!("{suffix}: {:<16} value {:>4}  {:?} cdB", l.amp, l.value, l.centi_db);
        }
    }
    println!("faults: {:?}", k.faults());
    println!("granted: {:?}", k.granted());
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("Linux only");
}
