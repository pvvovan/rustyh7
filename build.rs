fn main() {
    cc::Build::new().file("src/h7system.S").compile("h7system");
    println!("cargo:rustc-link-lib=h7system");
    println!("cargo:rerun-if-changed=src/h7system.S");
    println!("cargo:rerun-if-changed=link.x");
}
