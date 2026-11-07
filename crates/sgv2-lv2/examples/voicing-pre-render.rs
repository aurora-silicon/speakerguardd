//! Offline host of the deployed LV2 ABI. Only accepts a compiled user blob;
//! no ALSA/PipeWire or device is opened. The board is explicit for offline
//! fixture checks; the live descriptor always reads the device tree.
use sgv2_lv2::voicing;
use std::io::Write;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |name: &str| -> &str { &args[args.iter().position(|s| s == name).expect(name) + 1] };
    let bytes = std::fs::read(get("--params")).unwrap();
    let board = get("--board");
    let (header, _) = sgv2_core::blob::Header::parse(&bytes).unwrap();
    let n = header.channels as usize;
    assert!(voicing::Voicing::load(Some(&bytes), board, header.sample_rate as f64, n).ready());
    let index = voicing::VARIANTS.iter().position(|v| v.1 == n).unwrap();
    // SAFETY: the offline host owns the live handle and all connected buffers;
    // descriptor storage is static and every callback length fits those buffers.
    let d = unsafe { &*voicing::descriptor(index) };
    let h = voicing::instantiate_offline(Some(&bytes), board, header.sample_rate as f64, n);
    let input = std::fs::read(get("--in")).unwrap();
    assert_eq!(input.len() % 8, 0);
    let input: Vec<f32> = input.chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
    let mut scalar: f32 = get("--mvsc").parse().unwrap();
    let mut db: f32 =
        if let Some(at) = args.iter().position(|s| s == "--mvol") { args[at + 1].parse().unwrap() } else { 1.0 };
    let mut health = 0.0f32;
    let mut program = [[0.0f32; voicing::BLOCK]; 2];
    let mut output = vec![[0.0f32; voicing::BLOCK]; n];
    let mut out = std::io::BufWriter::new(std::fs::File::create(get("--out")).unwrap());
    // SAFETY: the offline host owns the live handle and all connected buffers;
    // descriptor storage is static and every callback length fits those buffers.
    unsafe {
        for (c, p) in program.iter_mut().enumerate() {
            (d.connect_port)(h, c as u32, p.as_mut_ptr().cast());
        }
        for (c, p) in output.iter_mut().enumerate() {
            (d.connect_port)(h, (c + 2) as u32, p.as_mut_ptr().cast());
        }
        (d.connect_port)(h, (n + 2) as u32, (&mut scalar as *mut f32).cast());
        (d.connect_port)(h, (n + 3) as u32, (&mut db as *mut f32).cast());
        (d.connect_port)(h, (n + 4) as u32, (&mut health as *mut f32).cast());
        (d.activate.unwrap())(h);
    }
    for at in (0..input.len() / 2).step_by(voicing::BLOCK) {
        let len = voicing::BLOCK.min(input.len() / 2 - at);
        for (c, p) in program.iter_mut().enumerate() {
            for (i, v) in p[..len].iter_mut().enumerate() {
                *v = input[2 * (at + i) + c];
            }
        }
        // SAFETY: the offline host owns the live handle and all connected buffers;
        // descriptor storage is static and every callback length fits those buffers.
        unsafe {
            (d.run)(h, len as u32);
        }
        assert_eq!(health, 0.0);
        for i in 0..len {
            for p in &output {
                out.write_all(&p[i].to_le_bytes()).unwrap();
            }
        }
    }
    // SAFETY: the offline host owns the live handle and all connected buffers;
    // descriptor storage is static and every callback length fits those buffers.
    unsafe {
        (d.cleanup)(h);
    }
}
