#!/usr/bin/env python3
"""Build the one-time install image: bootloader + app + trailer.

Layout written (exactly 1 MiB, 0xFF = erased-flash state elsewhere):

  0x00000  bootloader (already padded to 64 KiB by make-bin.sh)
  0x10000  application (linked at 0x08010000)
  0xFF800  trailer: magic 'FNB5', size, CRC-32(app), ~CRC

The trailer is the bootloader's boot gate: without it (an erased sector)
the device refuses to jump and waits in update mode FOREVER — even right
after a DFU install of a perfectly good app. So the install image embeds
the same four words the bootloader's COMMIT would write, computed from
this build's app binary. The gate stays real: the trailer only matches
when the bytes at 0x08010000 in flash are exactly the app this image
carries; any CDC upload without a fresh COMMIT makes them disagree and
the bootloader falls back to update mode.
"""

import struct
import sys
import zlib

round_tag = sys.argv[1]
bl = open(f"fnb58-bl-{round_tag}.bin", "rb").read()
app = open(f"fnb58-app-{round_tag}.bin", "rb").read()

if len(bl) != 0x10000:
    sys.exit(f"FATAL: bootloader is {len(bl)} bytes, must be padded to exactly 0x10000")
if len(app) == 0 or len(app) > 0xEF800:
    sys.exit(f"FATAL: app size {len(app)} out of range (0 < size <= 0xEF800)")

size = len(app)
crc = zlib.crc32(app) & 0xFFFFFFFF
trailer = struct.pack(
    "<IIII", int.from_bytes(b"FNB5", "little"), size, crc, crc ^ 0xFFFFFFFF
)

image = bytearray(b"\xff" * 0x100000)
image[0 : len(bl)] = bl
image[0x10000 : 0x10000 + len(app)] = app
image[0xFF800 : 0xFF800 + len(trailer)] = trailer

out = f"fnb58-full-{round_tag}.bin"
open(out, "wb").write(bytes(image))
print(f"trailer: size={size} crc={crc:08X}")
print(f"{out}: 0x100000 bytes")
