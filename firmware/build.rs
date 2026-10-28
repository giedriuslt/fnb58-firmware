use std::{env, fs, path::PathBuf};

fn main() -> std::io::Result<()> {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo provides OUT_DIR"));
    fs::copy("memory.x", out.join("memory.x"))?;
    println!("cargo:rustc-link-arg-bins=-L{}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    Ok(())
}
