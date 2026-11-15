#!/usr/bin/env python3
"""FNB58 screenshot tool (r223).

The panel's data line is write-only (no MISO, the r45 pin trace), so
GRAM read-back is impossible. Instead the firmware's console `shot`
command forces a full repaint of the CURRENT screen with the LCD's
capture sink armed: every draw primitive (fills and glyphs) streams to
the console as one line each — this script replays them into a PNG.

Primitive lines (lcd.rs capture_fill / capture_char):
  F x0 y0 x1 y1 cccc      fill, inclusive rect, RGB565 color in hex
  C x y s ffff bbbb hh    glyph: x, y, scale, fg, bg, char as two hex
                          digits; the 5x7 font table is this script's
                          own copy of lcd.rs's (bit0 = top pixel row,
                          each font pixel is s x s screen pixels, the
                          glyph cell is painted opaque fg/bg)

Usage:
  python fnb58-shot.py COM6 out.png
  python fnb58-shot.py COM6 out.png --timeout 20

Navigate the device to the screen you want FIRST, then run the tool.
A PD exchange in flight refuses the shot ("[shot] refused: pd busy").
"""

import argparse
import re
import struct
import sys
import time
import zlib

import serial

W, H = 320, 240

# --- the 5x7 font, copied from firmware/src/lcd.rs (keep in sync!) ---

FONT_CHARS = b"0123456789. VAWmu-PSCDBEFGHIKLMNORTUY:/QXksdivp~<>+h"

FONT_5X7 = [
    [0x3E, 0x51, 0x49, 0x45, 0x3E],  # 0
    [0x00, 0x42, 0x7F, 0x40, 0x00],  # 1
    [0x62, 0x51, 0x49, 0x49, 0x46],  # 2
    [0x41, 0x49, 0x49, 0x49, 0x36],  # 3
    [0x18, 0x14, 0x12, 0x7F, 0x10],  # 4
    [0x27, 0x45, 0x45, 0x45, 0x39],  # 5
    [0x3C, 0x4A, 0x49, 0x49, 0x30],  # 6
    [0x01, 0x71, 0x09, 0x05, 0x03],  # 7
    [0x36, 0x49, 0x49, 0x49, 0x36],  # 8
    [0x06, 0x49, 0x49, 0x29, 0x1E],  # 9
    [0x00, 0x60, 0x60, 0x00, 0x00],  # .
    [0x00, 0x00, 0x00, 0x00, 0x00],  # space
    [0x1F, 0x20, 0x40, 0x20, 0x1F],  # V
    [0x7E, 0x09, 0x09, 0x09, 0x7E],  # A
    [0x3F, 0x40, 0x38, 0x40, 0x3F],  # W
    [0x7C, 0x08, 0x10, 0x08, 0x7C],  # m
    [0x3C, 0x40, 0x40, 0x20, 0x7C],  # u
    [0x08, 0x08, 0x08, 0x08, 0x08],  # -
    [0x7F, 0x09, 0x09, 0x09, 0x06],  # P
    [0x46, 0x49, 0x49, 0x49, 0x31],  # S
    [0x3E, 0x41, 0x41, 0x41, 0x22],  # C
    [0x7F, 0x41, 0x41, 0x41, 0x3E],  # D
    [0x7F, 0x49, 0x49, 0x49, 0x36],  # B
    [0x7F, 0x49, 0x49, 0x49, 0x41],  # E
    [0x7F, 0x09, 0x09, 0x09, 0x01],  # F
    [0x3E, 0x41, 0x41, 0x51, 0x32],  # G
    [0x7F, 0x08, 0x08, 0x08, 0x7F],  # H
    [0x00, 0x41, 0x7F, 0x41, 0x00],  # I
    [0x7F, 0x08, 0x14, 0x22, 0x41],  # K
    [0x7F, 0x40, 0x40, 0x40, 0x40],  # L
    [0x7F, 0x02, 0x0C, 0x02, 0x7F],  # M
    [0x7F, 0x04, 0x08, 0x10, 0x7F],  # N
    [0x3E, 0x41, 0x41, 0x41, 0x3E],  # O
    [0x7F, 0x09, 0x19, 0x29, 0x46],  # R
    [0x01, 0x01, 0x7F, 0x01, 0x01],  # T
    [0x3F, 0x40, 0x40, 0x40, 0x3F],  # U
    [0x03, 0x04, 0x78, 0x04, 0x03],  # Y
    [0x00, 0x36, 0x36, 0x00, 0x00],  # :
    [0x00, 0x60, 0x18, 0x06, 0x01],  # /
    [0x3E, 0x51, 0x49, 0x45, 0x3B],  # Q (r189: tail on the O)
    [0x33, 0x33, 0x1E, 0x33, 0x33],  # X (r189)
    [0x7C, 0x10, 0x28, 0x44, 0x00],  # k (r199, x-height like m/u)
    [0x58, 0x14, 0x54, 0x14, 0x70],  # s (r199)
    [0x78, 0x44, 0x44, 0x44, 0x7F],  # d (r199, tall right stem)
    [0x00, 0x00, 0x7A, 0x00, 0x00],  # i (r199, dot + stem one column)
    [0x0C, 0x1C, 0x70, 0x1C, 0x0C],  # v (r199)
    [0x7F, 0x24, 0x24, 0x24, 0x3C],  # p (r199, bowl no descender)
    [0x08, 0x04, 0x08, 0x10, 0x08],  # ~ (r199, nominal-value mark)
    [0x08, 0x14, 0x22, 0x00, 0x00],  # < (r224, the overview hint)
    [0x00, 0x00, 0x22, 0x14, 0x08],  # > (r224)
    [0x08, 0x08, 0x3E, 0x08, 0x08],  # + (r225, the D+ card label)
    [0x7C, 0x04, 0x04, 0x78, 0x00],  # h (r226, the Ah/Wh units)
]

FILL_RE = re.compile(r"^F (\d+) (\d+) (\d+) (\d+) ([0-9a-fA-F]{4})$")
CHAR_RE = re.compile(r"^C (\d+) (\d+) (\d+) ([0-9a-fA-F]{4}) ([0-9a-fA-F]{4}) ([0-9a-fA-F]{2})$")


def rgb565_to_rgb888(v):
    r = (v >> 11) & 0x1F
    g = (v >> 5) & 0x3F
    b = v & 0x1F
    return ((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2))


def replay(lines):
    """Replay primitive lines into a W*H*3 RGB888 bytearray."""
    px = bytearray(W * H * 3)
    black = rgb565_to_rgb888(0x0000)
    for i in range(W * H):
        px[i * 3 : i * 3 + 3] = black

    n_fills = n_chars = 0
    for line in lines:
        m = FILL_RE.match(line)
        if m:
            x0 = int(m.group(1))
            y0 = int(m.group(2))
            x1 = int(m.group(3))
            y1 = int(m.group(4))
            rgb = rgb565_to_rgb888(int(m.group(5), 16))
            x0, x1 = max(x0, 0), min(x1, W - 1)
            y0, y1 = max(y0, 0), min(y1, H - 1)
            if x1 < x0 or y1 < y0:
                continue
            n_fills += 1
            for y in range(y0, y1 + 1):
                base = (y * W + x0) * 3
                run = rgb * (x1 - x0 + 1)
                px[base : base + len(run)] = run
            continue
        m = CHAR_RE.match(line)
        if m:
            x, y, scale = int(m.group(1)), int(m.group(2)), int(m.group(3))
            fg = rgb565_to_rgb888(int(m.group(4), 16))
            bg = rgb565_to_rgb888(int(m.group(5), 16))
            ch = int(m.group(6), 16)
            idx = FONT_CHARS.find(ch)
            glyph = FONT_5X7[idx] if idx >= 0 else FONT_5X7[11]
            n_chars += 1
            for r in range(7 * scale):
                for c in range(5 * scale):
                    bit = (glyph[c // scale] >> (r // scale)) & 1
                    px_y = y + r
                    px_x = x + c
                    if 0 <= px_x < W and 0 <= px_y < H:
                        base = (px_y * W + px_x) * 3
                        px[base : base + 3] = fg if bit else bg
    return px, n_fills, n_chars


def write_png(path, px):
    """Minimal RGB888 PNG writer (stdlib zlib only)."""
    def chunk(tag, data):
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    rows = b"".join(
        b"\x00" + bytes(px[y * W * 3 : (y + 1) * W * 3]) for y in range(H)
    )
    ihdr = struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", ihdr))
        f.write(chunk(b"IDAT", zlib.compress(rows, 9)))
        f.write(chunk(b"IEND", b""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("out")
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"shot\r\n")

    lines = []
    done = False
    t0 = time.time()
    buf = b""
    while time.time() - t0 < args.timeout:
        buf += ser.read(4096)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", "replace").strip()
            if "[shot] refused" in line:
                print(line)
                ser.close()
                sys.exit(1)
            if "[shot] done" in line:
                done = True
            if FILL_RE.match(line) or CHAR_RE.match(line):
                lines.append(line)
        if done and not buf:
            break
    ser.close()

    if not done:
        print(f"ERROR: no [shot] done within {args.timeout}s "
              f"({len(lines)} primitives received)")
        sys.exit(1)

    px, n_fills, n_chars = replay(lines)
    write_png(args.out, px)
    print(f"{args.out}: {W}x{H}, {n_fills} fills + {n_chars} glyphs replayed")


if __name__ == "__main__":
    main()
