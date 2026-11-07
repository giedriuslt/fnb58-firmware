//! Bootloader build: memory.x for the linker (cortex-m-rt link.x), plus
//! the vendor AT32 USB CDC stack compiled into libvendorusb.a and linked
//! into the ELF. This is the same C stack the app has run since round 15
//! (the hand-rolled Rust CDC in the HAL never enumerated and was retired)
//! — a USB stack that has never worked is the wrong foundation for the
//! recovery-critical image.

use std::{env, path::PathBuf, process::Command};

fn main() {
    // The lib's pure logic host-tests on x86_64; the memory.x/link.x
    // args and the C library are device-only. Gate on the target arch.
    if env::var("CARGO_CFG_TARGET_ARCH").as_deref() != Ok("arm") {
        return;
    }

    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo provides OUT_DIR"));
    std::fs::copy("memory.x", out.join("memory.x")).expect("memory.x");
    println!("cargo:rustc-link-arg-bins=-L{}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");

    let workspace = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap())
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf();
    let gcc = workspace.join("armgcc/xpack-arm-none-eabi-gcc-12.3.1-1.1/bin");
    let gcc = if gcc.exists() {
        gcc
    } else {
        PathBuf::from(std::env::var("ARM_GCC_BIN").expect(
            "armgcc toolchain missing: extract armgcc.zip or set ARM_GCC_BIN",
        ))
    };

    let vendor = PathBuf::from("C:/git/fnb58/AT32F403A_407_Firmware_Library");
    // The conf headers (at32f403a_407_conf.h, usb_conf.h) stay with the
    // app's glue — exactly one copy of them in the tree.
    let app_c = PathBuf::from("../firmware/c");
    let c_dir = PathBuf::from("c");
    let includes = [
        vendor.join("libraries/cmsis/cm4/core_support"),
        vendor.join("libraries/cmsis/cm4/device_support"),
        vendor.join("libraries/drivers/inc"),
        vendor.join("middlewares/usbd_drivers/inc"),
        vendor.join("middlewares/usbd_class/cdc"),
        app_c.clone(),
        c_dir.clone(),
    ];

    // USB-only subset: no HID, no LCD, no GPIO/SPI/ACC — the bootloader
    // drives everything but the USB cell from Rust. If the link ever
    // reports a missing C symbol, the needed vendor source goes here.
    let sources = [
        c_dir.join("bl_usb.c"),
        vendor.join("libraries/drivers/src/at32f403a_407_usb.c"),
        vendor.join("libraries/drivers/src/at32f403a_407_crm.c"),
        vendor.join("middlewares/usbd_drivers/src/usbd_core.c"),
        vendor.join("middlewares/usbd_drivers/src/usbd_int.c"),
        vendor.join("middlewares/usbd_drivers/src/usbd_sdr.c"),
        vendor.join("middlewares/usbd_class/cdc/cdc_class.c"),
        vendor.join("middlewares/usbd_class/cdc/cdc_desc.c"),
    ];

    let mut objects = Vec::new();
    for source in &sources {
        let obj = out.join(
            source
                .file_stem()
                .unwrap()
                .to_string_lossy()
                .replace(['-', ' '], "_")
                + ".o",
        );
        let mut cmd = Command::new(gcc.join("arm-none-eabi-gcc.exe"));
        cmd.args([
            "-c",
            "-mcpu=cortex-m4",
            "-mthumb",
            "-mfloat-abi=hard",
            "-mfpu=fpv4-sp-d16",
            "-O2",
            "-ffunction-sections",
            "-fdata-sections",
            "-ffreestanding",
            "-include",
            "stddef.h",
            "-DAT32F403ACGT7",
            "-DUSB_MODULE_ENABLED",
            "-DCRM_MODULE_ENABLED",
            "-std=gnu99",
            "-Wall",
        ]);
        for include in &includes {
            cmd.arg("-I").arg(include);
        }
        cmd.arg(source).arg("-o").arg(&obj);
        let status = cmd.status().expect("failed to run arm-none-eabi-gcc");
        if !status.success() {
            panic!("C compilation failed for {}", source.display());
        }
        println!("cargo:rerun-if-changed={}", source.display());
        objects.push(obj);
    }
    // Rebuild when the borrowed conf headers change too.
    for header in ["at32f403a_407_conf.h", "usb_conf.h"] {
        println!("cargo:rerun-if-changed={}", app_c.join(header).display());
    }

    let ar = gcc.join("arm-none-eabi-ar.exe");
    let lib = out.join("libvendorusb.a");
    let _ = std::fs::remove_file(&lib);
    let mut cmd = Command::new(ar);
    cmd.arg("rcs").arg(&lib);
    for obj in &objects {
        cmd.arg(obj);
    }
    let status = cmd.status().expect("failed to run arm-none-eabi-ar");
    if !status.success() {
        panic!("archiving vendorusb failed");
    }
    println!("cargo:rustc-link-search=native={}", out.display());
    println!("cargo:rustc-link-lib=static=vendorusb");
}
