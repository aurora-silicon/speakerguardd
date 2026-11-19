//! Regenerate the LV2 bundle's Turtle files from the plugin's variants.
fn main() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/lv2/sgv2.lv2/");
    let (manifest, plugin) = sgv2_lv2::ttl::generate();
    std::fs::write(format!("{dir}manifest.ttl"), manifest).unwrap();
    std::fs::write(format!("{dir}sgv2.ttl"), plugin).unwrap();
    println!("wrote {dir}manifest.ttl and {dir}sgv2.ttl");
}
