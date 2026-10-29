#!/bin/sh
# Build the firmware and emit a flashable binary at 0x08000000.
set -e
cd "$(dirname "$0")"
export PATH="$HOME/.cargo/bin:$PATH"
cargo build --release --target thumbv7em-none-eabihf
~/.rustup/toolchains/stable-x86_64-pc-windows-gnu/lib/rustlib/x86_64-pc-windows-gnu/bin/rust-objcopy.exe \
  -O binary target/thumbv7em-none-eabihf/release/fnb58-firmware fnb58-firmware.bin
ls -la fnb58-firmware.bin
