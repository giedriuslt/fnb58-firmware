#!/usr/bin/env python3
"""FNB58 CDC bootloader flasher.

Talks to fnb58-bootloader over the CDC virtual COM port (VID 0x2E3C
PID 0x5740 — the same VID/PID as the app console, so Windows reuses the
existing COM port). The device enters this mode on reset with OK held,
or automatically when the app image is empty/invalid.

Safety rule, mirroring the bootloader's own: nothing is written until
the device has answered INFO with a `[bl]` line. If the port belongs to
the app console (device booted normally), the flasher stops with a loud
error instead of writing bytes into a console parser.

Protocol (per bootloader/src/main.rs):
  INFO                  -> [bl] INFO tag=bl1 state=... size=... crc=...
  BEGIN <size>          -> [bl] OK begin size=<n> blocks=<n>
  2056-byte blocks      -> [bl] ACK <seq> | [bl] NAK <seq> <reason>
     ([u32 seq LE][u32 crc32 LE][2048 bytes], one per 2 KiB sector)
  COMMIT <size> <crc8X> -> [bl] OK committed crc=... | [bl] ERR ...
  RESET                 -> [bl] OK reset

Usage:
  python fnb58-flash.py COM7 fnb58-app-r180.bin
  python fnb58-flash.py COM7 --adopt fnb58-app-r180.bin
  python fnb58-flash.py COM7 --info-only

--adopt writes no data: it sends COMMIT for an app that is ALREADY in
flash (e.g. installed via the BOOT0 DFU, whose install image
tools/mkfull.py normally embeds the trailer itself). The device CRCs
its own flash content and only writes the trailer when it matches the
local image — a mismatch is a loud ERR and no trailer, so a DFU install
of something else can never be adopted by accident.
"""

import argparse
import struct
import sys
import time
import zlib

try:
    import serial  # pyserial; pip install pyserial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

BLOCK = 2048
WIRE = 8 + BLOCK
RETRIES = 5


_rx_leftover = bytearray()


def read_line(ser, timeout_s):
    """One \\n-terminated line, or None on timeout. Bytes after the first
    newline are kept for the next call — a chatty device must not make
    one response swallow the following ones."""
    global _rx_leftover
    deadline = time.monotonic() + timeout_s
    while True:
        if b"\n" in _rx_leftover:
            line, _, _rx_leftover = _rx_leftover.partition(b"\n")
            return line.strip(b"\r")
        if time.monotonic() >= deadline:
            return None
        n = ser.in_waiting
        chunk = ser.read(n if n else 1)
        if chunk:
            _rx_leftover += chunk
        elif not _rx_leftover:
            time.sleep(0.01)


def expect(ser, prefix, timeout_s=10.0):
    line = read_line(ser, timeout_s)
    if line is None:
        sys.exit(f"FATAL: no response from device (expected {prefix!r})")
    if not line.startswith(prefix):
        sys.exit(f"FATAL: unexpected response {line!r} (expected {prefix!r})")
    return line


def flush_input(ser):
    """Drop anything pending on the wire AND any half-line already read
    into the leftover buffer."""
    global _rx_leftover
    ser.reset_input_buffer()
    _rx_leftover = bytearray()


def wait_for_port(port, timeout_s):
    """Wait for the CDC port to (re)appear after a reboot. Returns an open
    handle; prefers the original COM name, falls back to any 2E3C:5740
    device if the OS renumbered it."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        time.sleep(0.5)
        try:
            return serial.Serial(port, 115200, timeout=0.1)
        except serial.SerialException:
            pass
        # The reboot may have renumbered the port: scan for the device.
        try:
            for p in serial.tools.list_ports.comports():
                if p.vid == 0x2E3C and p.pid == 0x5740 and p.device != port:
                    print(f"port renumbered: {port} -> {p.device}")
                    return serial.Serial(p.device, 115200, timeout=0.1)
        except Exception:
            pass
    sys.exit(f"FATAL: {port} never came back after the reboot")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", help="COM port of the device (bootloader or console)")
    ap.add_argument("image", nargs="?", help="application raw binary (linked at 0x08010000)")
    ap.add_argument("--adopt", action="store_true",
                    help="no upload: COMMIT the app already in flash (size/CRC taken "
                         "from the local image) and reset")
    ap.add_argument("--info-only", action="store_true", help="just query and print INFO")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.1)

    # Identify FIRST. The app console will not answer with [bl].
    flush_input(ser)
    ser.write(b"INFO\n")
    line = read_line(ser, 5.0)
    if line is None or not line.startswith(b"[bl] INFO "):
        # r198: the tester is probably in LOG (app console) mode — it answers
        # INFO with the echo '[flash] cmd: I N F O' and heartbeats. Reboot it
        # into the bootloader via the run-bootloader command, then wait for
        # the CDC re-enumeration and re-identify.
        print("not in bootloader mode - requesting run-bootloader")
        ser.write(b"run-bootloader\n")
        # The ack is '[app] bootloader requested rb=.., rebooting'; the port
        # then detaches as the device resets.
        deadline = time.monotonic() + 5.0
        acked = False
        while time.monotonic() < deadline:
            l = read_line(ser, 0.5)
            if l and b"bootloader requested" in l:
                acked = True
                break
        if not acked:
            # The port may already have detached mid-read; that also counts.
            print("no ack seen (port may have detached) - continuing")
        ser.close()
        # Wait for the CDC to come back (same VID/PID; the OS may renumber
        # the COM port, so re-scan if the original does not return).
        ser = wait_for_port(args.port, 20.0)
        flush_input(ser)
        ser.write(b"INFO\n")
        line = read_line(ser, 5.0)
    if line is None:
        sys.exit("FATAL: no response from device after reboot attempt")
    if not line.startswith(b"[bl] INFO "):
        sys.exit(f"FATAL: unexpected response {line!r} (expected '[bl] INFO ')")
    info = line
    print(info.decode("ascii", "replace"))

    if args.info_only:
        return

    if args.image is None:
        sys.exit("FATAL: image path required (or use --info-only)")

    data = open(args.image, "rb").read()
    size = len(data)
    if size == 0:
        sys.exit("FATAL: empty image")
    if size % 4:
        # The final partial word is programmed 0xFF-padded by both sides;
        # pad the host copy so block CRCs match the wire.
        data += b"\xff" * (4 - size % 4)
    if size > 0xEF800:
        sys.exit(f"FATAL: image {size} bytes exceeds the 958 KiB app region")

    crc = zlib.crc32(data[:size]) & 0xFFFFFFFF
    blocks = (size + BLOCK - 1) // BLOCK
    print(f"image: {size} bytes, {blocks} blocks, crc32 {crc:08X}")

    if args.adopt:
        # No upload: the device CRCs what is already in flash and writes
        # the trailer only on an exact match against the local image.
        flush_input(ser)
        ser.write(f"COMMIT {size} {crc:08X}\n".encode())
        expect(ser, b"[bl] OK committed", timeout_s=30.0)
        flush_input(ser)
        ser.write(b"RESET\n")
        expect(ser, b"[bl] OK reset", timeout_s=5.0)
        print("adopted — device reset; the flashed app should boot")
        return

    # BEGIN
    flush_input(ser)
    ser.write(f"BEGIN {size}\n".encode())
    expect(ser, b"[bl] OK begin ", timeout_s=5.0)

    # Block stream, lockstep: one block in, one ACK/NAK out.
    for seq in range(blocks):
        chunk = data[seq * BLOCK : (seq + 1) * BLOCK].ljust(BLOCK, b"\xff")
        pkt = struct.pack("<II", seq, zlib.crc32(chunk) & 0xFFFFFFFF) + chunk
        for attempt in range(1, RETRIES + 1):
            flush_input(ser)
            ser.write(pkt)
            line = read_line(ser, timeout_s=15.0)  # erase + program time
            if line == f"[bl] ACK {seq}".encode():
                if seq % 10 == 0 or seq == blocks - 1:
                    print(f"  block {seq + 1}/{blocks} ok")
                break
            if line is None:
                print(f"  block {seq}: timeout (attempt {attempt}/{RETRIES})")
            else:
                print(f"  block {seq}: {line.decode('ascii', 'replace')} "
                      f"(attempt {attempt}/{RETRIES})")
        else:
            sys.exit(f"FATAL: block {seq} failed after {RETRIES} attempts")

    # COMMIT — the device re-verifies the whole region before writing the
    # trailer; a mismatch leaves the app invalid (recovery intact).
    flush_input(ser)
    ser.write(f"COMMIT {size} {crc:08X}\n".encode())
    expect(ser, b"[bl] OK committed", timeout_s=30.0)

    # RESET into the freshly written app.
    flush_input(ser)
    ser.write(b"RESET\n")
    expect(ser, b"[bl] OK reset", timeout_s=5.0)

    print("done — device reset; the new app should boot (watch the console tag)")


if __name__ == "__main__":
    main()
