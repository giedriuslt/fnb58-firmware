#!/bin/sh
# Build the bootloader + firmware and emit round-tagged binaries:
#
#   fnb58-bl-$ROUND.bin    bootloader raw binary (guards: <= 64 KiB)
#   fnb58-app-$ROUND.bin   application raw binary (linked at 0x08010000;
#                          guards: <= 958 KiB)
#   fnb58-full-$ROUND.bin  ONE-TIME install image (exactly 1 MiB): the
#                          bootloader (padded to 64 KiB), the app, and
#                          the trailer sector at 0x080FF800 — flash via
#                          the BOOT0 DFU at 0x08000000
#                          (fnb58-research/at32dfu.py). The embedded
#                          trailer makes the first boot run the app
#                          directly instead of waiting in update mode.
#                          After that, app updates go through the
#                          bootloader CDC (OK held at reset) with
#                          tools/fnb58-flash.py.
#
# Size guards fail loudly; nothing is ever truncated silently.
set -e
cd "$(dirname "$0")"
export PATH="$HOME/.cargo/bin:$PATH"
ROUND="${1:-r180}"

cargo build --release --target thumbv7em-none-eabihf

OBJCOPY="$HOME/.rustup/toolchains/stable-x86_64-pc-windows-gnu/lib/rustlib/x86_64-pc-windows-gnu/bin/rust-objcopy.exe"
"$OBJCOPY" -O binary target/thumbv7em-none-eabihf/release/fnb58-bootloader "fnb58-bl-$ROUND.bin"
"$OBJCOPY" -O binary target/thumbv7em-none-eabihf/release/fnb58-firmware "fnb58-app-$ROUND.bin"

BL_LIMIT=65536
APP_LIMIT=$((0xEF800))   # 0x08010000..0x080FF800 (958 KiB)
BL_SIZE=$(stat -c %s "fnb58-bl-$ROUND.bin")
APP_SIZE=$(stat -c %s "fnb58-app-$ROUND.bin")

if [ "$BL_SIZE" -gt "$BL_LIMIT" ]; then
  echo "FATAL: bootloader is $BL_SIZE bytes, limit is $BL_LIMIT" >&2
  exit 1
fi
if [ "$APP_SIZE" -gt "$APP_LIMIT" ]; then
  echo "FATAL: app is $APP_SIZE bytes, limit is $APP_LIMIT" >&2
  exit 1
fi

# Pad the bootloader to exactly 64 KiB with 0xFF (the erased-flash state)
# so the app starts at 0x08010000 in the concatenated image.
PAD=$((BL_LIMIT - BL_SIZE))
if [ "$PAD" -gt 0 ]; then
  head -c "$PAD" /dev/zero | tr '\000' '\377' >> "fnb58-bl-$ROUND.bin"
fi

# The full image INCLUDES the trailer sector (tools/mkfull.py): a bare
# bl+app concatenation leaves the trailer erased, and the bootloader
# refuses to jump on an empty trailer — the DFU-installed device would
# sit in update mode forever with a perfectly good app at 0x08010000.
python -I tools/mkfull.py "$ROUND"

echo "bootloader: $BL_SIZE bytes (limit $BL_LIMIT)"
echo "app:        $APP_SIZE bytes (limit $APP_LIMIT)"
echo "full:       $(stat -c %s "fnb58-full-$ROUND.bin") bytes (install image incl. trailer)"
md5sum "fnb58-bl-$ROUND.bin" "fnb58-app-$ROUND.bin" "fnb58-full-$ROUND.bin"

# r195: fixed-name copies for git — the tracked artifacts are overwritten
# every round (no version-number accumulation in the repo); the
# round-tagged files above stay as untracked scratch copies.
cp "fnb58-bl-$ROUND.bin" fnb58-bl.bin
cp "fnb58-app-$ROUND.bin" fnb58-app.bin
cp "fnb58-full-$ROUND.bin" fnb58-full.bin
