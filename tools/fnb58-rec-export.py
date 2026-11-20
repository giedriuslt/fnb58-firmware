#!/usr/bin/env python3
"""FNB58 offline-recording exporter.

Reads the meter's recorded samples out of the device over the CDC
console (VID 0x2E3C PID 0x5740) and writes a CSV. The recorder
(firmware/src/rec.rs) stores calibrated samples as 4-byte records
(u16 mV LE, u16 mA LE — magnitudes) in the W25Q128's free area; the
console commands `ri` (log info) and `rd <first> <count>` (hex dump,
6 records per 64-byte line) are the wire interface.

Usage:
  python fnb58-rec-export.py COM7 recording.csv
  python fnb58-rec-export.py COM7 recording.csv --start 100 --count 5000
  python fnb58-rec-export.py COM7 --info-only

CSV columns: t_s (sample index x the log's interval), v_v, a_a, w_w
(power derives from the record, mv x ma / 1e6). The interval comes from
the log's own meta (`ri` reports it) — a resumed log keeps its original
interval, so the axis is always sample-count x interval.
"""

import argparse
import sys
import time

try:
    import serial  # pyserial; pip install pyserial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

RECORDS_PER_LINE = 6
CHUNK = 600  # records per `rd` command (100 lines)


class Port:
    def __init__(self, dev):
        self.ser = serial.Serial(dev, timeout=0.1)
        self.leftover = bytearray()

    def read_line(self, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while True:
            if b"\n" in self.leftover:
                line, _, self.leftover = self.leftover.partition(b"\n")
                return line.strip(b"\r")
            if time.monotonic() >= deadline:
                return None
            n = self.ser.in_waiting
            chunk = self.ser.read(n if n else 1)
            if chunk:
                self.leftover += chunk

    def send(self, text):
        self.ser.reset_input_buffer()
        self.leftover.clear()
        self.ser.write(text.encode() + b"\r\n")
        self.ser.flush()


def info(port):
    """`ri` -> (state, count, interval_s) or None."""
    port.send("ri")
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        line = port.read_line(1.0)
        if line is None:
            return None
        text = line.decode(errors="replace")
        if text.startswith("[rec] state="):
            fields = dict(
                part.split("=", 1) for part in text.split(" ")[1:] if "=" in part
            )
            return (
                fields.get("state", "?"),
                int(fields.get("n", "0")),
                int(fields.get("int", "1").rstrip("s")),
            )
        if "[rec]" in text and "ERR" in text:
            sys.exit(f"FATAL: device answered {text!r}")
    return None


def dump(port, first, total, emit):
    """`rd <first> <count>` in chunks; emit(mv, ma) per record."""
    got = 0
    while got < total:
        n = min(CHUNK, total - got)
        port.send(f"rd {first + got} {n}")
        deadline = time.monotonic() + 30
        lines = 0
        while got < total and time.monotonic() < deadline:
            line = port.read_line(2.0)
            if line is None:
                break
            text = line.decode(errors="replace")
            if not text.startswith("[rec] d "):
                if "ERR" in text:
                    sys.exit(f"FATAL: device answered {text!r}")
                continue  # the echo line or stray traffic
            hexpart = text[8:].strip()
            if len(hexpart) % 8 != 0:
                sys.exit(f"FATAL: bad hex line {text!r}")
            for i in range(0, len(hexpart), 8):
                word = int(hexpart[i : i + 8], 16)
                emit(word & 0xFFFF, (word >> 16) & 0xFFFF)
                got += 1
            lines += 1
        if got < total and lines == 0:
            sys.exit(f"FATAL: dump stalled at record {first + got}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", help="CDC COM port (e.g. COM7)")
    ap.add_argument("csv", nargs="?", help="output .csv path")
    ap.add_argument("--start", type=int, default=0, help="first record to export")
    ap.add_argument("--count", type=int, default=None, help="records to export")
    ap.add_argument("--info-only", action="store_true")
    args = ap.parse_args()

    port = Port(args.port)
    state, count, interval = info(port) or (None, None, None)
    if state is None:
        sys.exit("FATAL: no [rec] answer — is this the app console?")
    print(f"log: state={state} n={count} interval={interval}s")
    if args.info_only:
        return
    if state == "idle" or count == 0:
        sys.exit("FATAL: no samples recorded (the REC screen: OK -> START)")
    first = max(0, min(args.start, count - 1))
    total = min(args.count if args.count is not None else count, count - first)
    print(f"exporting {total} records from {first} ({total * interval} s span)")

    with open(args.csv, "w", newline="") as f:
        f.write("t_s,v_v,a_a,w_w\n")
        last_pct = -1

        def emit(mv, ma):
            nonlocal last_pct
            n = emit.i
            emit.i += 1
            t = (first + n) * interval
            f.write(
                f"{t},{mv / 1000:.3f},{ma / 1000:.3f},{mv * ma / 1_000_000:.6f}\n"
            )
            pct = (n * 100) // total
            if pct != last_pct and pct % 5 == 0:
                print(f"  {pct}%")
                last_pct = pct

        emit.i = 0
        dump(port, first, total, emit)
    print(f"done: {emit.i} records -> {args.csv}")


if __name__ == "__main__":
    main()
