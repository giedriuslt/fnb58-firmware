//! Compiles the vendor AT32 USB stack (C) into a static library and links
//! it into the firmware. The toolchain is the xPack GNU Arm Embedded GCC
//! extracted at ../armgcc (no installer).

use std::{env, path::PathBuf, process::Command};

fn main() {
    // memory.x for the linker (cortex-m-rt's link.x includes it).
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
    let c_dir = PathBuf::from("c");
    let includes = [
        vendor.join("libraries/cmsis/cm4/core_support"),
        vendor.join("libraries/cmsis/cm4/device_support"),
        vendor.join("libraries/drivers/inc"),
        vendor.join("libraries/drivers/inc"),
        vendor.join("middlewares/usbd_drivers/inc"),
        vendor.join("middlewares/usbd_class/cdc"),
        vendor.join("middlewares/i2c_application_library"),
        c_dir.clone(),
    ];

    let sources = [
        c_dir.join("fnb58_usb.c"),
        vendor.join("libraries/drivers/src/at32f403a_407_usb.c"),
        vendor.join("libraries/drivers/src/at32f403a_407_crm.c"),
        vendor.join("libraries/drivers/src/at32f403a_407_acc.c"),
        vendor.join("libraries/drivers/src/at32f403a_407_i2c.c"),
        vendor.join("libraries/drivers/src/at32f403a_407_gpio.c"),
        c_dir.join("fnb58_i2c.c"),
        vendor.join("middlewares/i2c_application_library/i2c_application.c"),
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
            "-ffreestanding", "-include", "stddef.h",
            "-DAT32F403ACGT7", "-DUSB_MODULE_ENABLED", "-DCRM_MODULE_ENABLED", "-DACC_MODULE_ENABLED",
            "-DI2C_MODULE_ENABLED", "-DGPIO_MODULE_ENABLED", "-DDMA_MODULE_ENABLED",
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

    // Also compile the CMSIS system file? Not needed: our Rust startup owns
    // SystemInit duties (clocks are configured from Rust).

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
