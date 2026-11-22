// The shared test fixtures (`../tests/common`) name a scratch directory
// with `env!("CARGO_TARGET_TMPDIR")`, which cargo sets only for integration
// tests. The fuzz targets build every bundle in memory and never write one;
// this gives the fixtures a directory to name.
fn main() {
    let out = std::env::var("OUT_DIR").unwrap();
    println!("cargo:rustc-env=CARGO_TARGET_TMPDIR={out}");
}
