#!/usr/bin/env python3
"""Build the bootloader + firmware and emit -dev-suffixed test binaries.

The Windows-native twin of make-bin.sh for TESTING builds: same steps
(cargo build, objcopy, the size guards, mkfull.py's trailer, md5s) but
the artifacts carry a -dev suffix and the tracked shipping names
(fnb58-bl.bin / fnb58-app.bin / fnb58-full.bin) are NEVER touched —
a dev build cannot clobber the last verified shipping binaries.

Outputs (repo root):
  fnb58-bl-dev.bin     bootloader raw binary (guard: <= 64 KiB)
  fnb58-app-dev.bin    application raw binary (linked at 0x08010000;
                       guard: <= 958 KiB)
  fnb58-full-dev.bin   one-time install image (exactly 1 MiB, trailer
                       embedded — flash via the BOOT0 DFU at 0x08000000)

Flash the app over CDC afterwards with:
  python tools/fnb58-flash.py COMx fnb58-app-dev.bin
"""

import hashlib
import os
import shutil
import subprocess
import sys

ROUND = "dev"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET = "thumbv7em-none-eabihf"
ELF_DIR = os.path.join("target", TARGET, "release")
BL_LIMIT = 65536
APP_LIMIT = 0xEF800

# cargo may not be on PATH in some shells; the default rustup location
# is tried second. Same for rust-objcopy (rustup ships it per
# toolchain, not on PATH).
CARGO_PATHS = [
    "cargo",
    os.path.expanduser("~/.cargo/bin/cargo.exe"),
]
OBJCOPY_PATHS = [
    shutil.which("rust-objcopy"),
    os.path.expanduser(
        "~/.rustup/toolchains/stable-x86_64-pc-windows-gnu/lib/rustlib/"
        "x86_64-pc-windows-gnu/bin/rust-objcopy.exe"
    ),
]


def first_existing(paths):
    for p in paths:
        if p and os.path.isfile(p):
            return p
    sys.exit(f"FATAL: none of these exist: {paths}")


def run(cmd, env=None):
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=ROOT, env=env)


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def main():
    os.chdir(ROOT)

    env = os.environ.copy()
    env["PATH"] = os.path.expanduser("~/.cargo/bin") + os.pathsep + env["PATH"]
    cargo = first_existing(CARGO_PATHS)
    run([cargo, "build", "--release", f"--target={TARGET}"], env)

    objcopy = first_existing(OBJCOPY_PATHS)

    elf_bl = os.path.join(ELF_DIR, "fnb58-bootloader")
    elf_app = os.path.join(ELF_DIR, "fnb58-firmware")
    bl = f"fnb58-bl-{ROUND}.bin"
    app = f"fnb58-app-{ROUND}.bin"
    run([objcopy, "-O", "binary", elf_bl, bl])
    run([objcopy, "-O", "binary", elf_app, app])

    bl_size = os.path.getsize(bl)
    app_size = os.path.getsize(app)
    if bl_size > BL_LIMIT:
        sys.exit(f"FATAL: bootloader is {bl_size} bytes, limit is {BL_LIMIT}")
    if app_size > APP_LIMIT:
        sys.exit(f"FATAL: app is {app_size} bytes, limit is {APP_LIMIT}")

    # Pad the bootloader to exactly 64 KiB with 0xFF (the erased-flash
    # state) so the app starts at 0x08010000 in the concatenated image.
    pad = BL_LIMIT - bl_size
    if pad > 0:
        with open(bl, "ab") as f:
            f.write(b"\xff" * pad)

    # The full image includes the trailer sector (mkfull.py derives its
    # inputs from the round tag, so "dev" reads the -dev bins).
    run([sys.executable, "-I", "tools/mkfull.py", ROUND])

    print(f"bootloader: {bl_size} bytes (limit {BL_LIMIT})")
    print(f"app:        {app_size} bytes (limit {APP_LIMIT})")
    print(f"full:       {os.path.getsize(f'fnb58-full-{ROUND}.bin')} bytes "
          f"(install image incl. trailer)")
    for path in (bl, app, f"fnb58-full-{ROUND}.bin"):
        print(f"{md5(path)} *{path}")
    print("shipping names untouched: fnb58-bl.bin / fnb58-app.bin / fnb58-full.bin")


if __name__ == "__main__":
    main()
