# fnb58-firmware

Open firmware for the **FNIRSI FNB58** USB tester (Artery AT32F403A),
built on [`embassy-at32`](../at32f403a-embassy-rs) and ported from
[kws-x1-standalone-rs](https://github.com/kralonur/kws-x1-standalone-rs).
Status: **r126** — working USB CDC console, 320x240 UI with
accelerometer-driven 180° screen flip and click feedback (buzzer),
live V/A/W meter, and a PD sink flow on the vendored `fusb302-pd`
stack (PC-gated, one-shot per OK — the charger's Request acknowledgment
is the one open hardware question).

## Architecture

- **USB CDC**: the vendor AT32 C stack (usbd_core/cdc_class) compiled
  into the firmware by `build.rs` (`c/fnb58_usb.c` glue). Proven path —
  the hand-rolled Rust USB stack was retired after rounds 1-14.
- **I²C1**: **Rust on the embassy HAL** (r112) — every I²C transaction
  (TPA626 meter, FUSB302 PD) runs through the HAL's `i2c::I2c` driver
  on PB6/PB7; the vendor C i2c code (glue + `i2c_application`
  middleware) is removed from the build.
- **PD**: the **vendored [`fusb302-pd`] crate, unchanged** (r113), with
  `pd-protocol` (Selection/Trial/Limits sink engine) and `usbpd-traits`
  pinned to the revisions kws-x1 ships against. The board supplies
  `pd_phy.rs` (FnbPlatform: power-on baseline, MDAC, VBUS via the
  TPA626; FnbLimits sink policy) and the `GatedBus` over the shared
  I2C1 (one Mutex lease per transaction, transfers spun to completion
  inside their lease). The sink session runs as its own embassy task;
  the UI renders its published view.

[`fusb302-pd`]: https://github.com/kralonur/kws-x1-standalone-rs/tree/master/fusb302-pd
- **LCD**: **Rust on the embassy HAL** (r111, `src/lcd.rs`) — SPI2 +
  GPIO pins driven by the HAL's `Spi`/`OutputPin`, a 1:1 port of the
  proven C driver (same init sequence, 5x7 font, 12 MHz rate); bulk
  pixel data uses the HAL's busy-polled bulk write. The C glue
  (`c/fnb58_lcd.c`) stays compiled but is no longer called.
- **Everything else is Rust** on `embassy-at32`: clocks (192 MHz PLL
  from the 8 MHz HEXT crystal + USB DIV_4), GPIO, EXINT, the UI,
  buttons, console queue, the KXTJ3 accelerometer driver (`kxtj3.rs`,
  screen-flip detection), and the buzzer (`buzzer.rs`, PA8).
- **Interrupts**: buttons and the FUSB302 INT_N run on EXINT (ISR
  latches a line bit and wakes the main loop via a signal); the main
  loop waits on the doorbell with a 10 ms fallback tick. PD RX reacts
  to INT edges instead of polling blind.

## Hardware pin map (LQFP48, probed on the real board)

Physical pin table — `Side` = where the pin sits on the board (probing
reference). Functions marked ✓ are verified on hardware; "unclaimed"
pins have no known connection.

| Pin | Name | Side | Function |
|-----|------|------|----------|
| 1 | VBAT | Left | battery-backup input (unconnected?) |
| 2 | PC13 | Left | ✓ FUSB302 INT_N (open-drain, EXINT13 falling) |
| 3 | PC14 / LEXT_IN | Left | ✓ 32 kHz crystal (confirmed present, `lext=1`) |
| 4 | PC15 / LEXT_OUT | Left | ✓ 32 kHz crystal |
| 5 | PD0 / HEXT_IN | Left | ✓ 8 MHz main crystal |
| 6 | PD1 / HEXT_OUT | Left | ✓ 8 MHz main crystal |
| 7 | NRST | Left | reset input |
| 8 | VSSA / VREF− | Left | analog ground |
| 9 | VDDA / VREF+ | Left | analog supply |
| 10 | PA0 / WKUP | Left | unclaimed |
| 11 | PA1 | Left | unclaimed |
| 12 | PA2 | Left | unclaimed |
| 13 | PA3 | Bottom | unclaimed |
| 14 | PA4 | Bottom | unclaimed |
| 15 | PA5 | Bottom | unclaimed |
| 16 | PA6 | Bottom | unclaimed |
| 17 | PA7 | Bottom | unclaimed |
| 18 | PB0 | Bottom | unclaimed |
| 19 | PB1 | Bottom | unclaimed |
| 20 | PB2 | Bottom | ✓ RS2228 USB-mux SELECT — DUT data lines: passthrough ↔ MCU measure path (polarity + landing pins unknown) |
| 21 | PB10 | Bottom | unclaimed |
| 22 | PB11 | Bottom | ✓ LCD DC |
| 23 | VSS_1 | Bottom | ground |
| 24 | VDD_1 | Bottom | digital supply |
| 25 | PB12 | Right | ✓ LCD CS (active low) |
| 26 | PB13 | Right | ✓ SPI2 SCK (LCD + W25Q128 shared) |
| 27 | PB14 | Right | ✓ SPI2 MISO (W25Q128 flash) |
| 28 | PB15 | Right | ✓ SPI2 MOSI (LCD + W25Q128 shared) |
| 29 | PA8 | Right | ✓ buzzer (2 kHz square wave) |
| 30 | PA9 | Right | unclaimed |
| 31 | PA10 | Right | unclaimed |
| 32 | PA11 | Right | ✓ USB D− |
| 33 | PA12 | Right | ✓ USB D+ |
| 34 | PA13 | Right | ✓ W25Q128 flash CS (also SWDIO — see flashing note) |
| 35 | VSS_2 | Right | ground |
| 36 | VDD_2 | Right | digital supply |
| 37 | PA14 | Top | SWCLK (debug only — keep free) |
| 38 | PA15 | Top | ✓ LEFT button (to GND, internal pull-up) |
| 39 | PB3 | Top | ✓ LCD backlight |
| 40 | PB4 | Top | ✓ OK button (to GND, internal pull-up) |
| 41 | PB5 | Top | ✓ load-bank MOSFET gate (FET SMD-marked 3400): HIGH = bank ON |
| 42 | PB6 | Top | ✓ I2C1 SCL (TPA626, FUSB302, KXTJ3) |
| 43 | PB7 | Top | ✓ I2C1 SDA |
| 44 | BOOT0 | Top | boot selector: high at reset = USB DFU (flash via BOOT0-high) |
| 45 | PB8 | Top | ✓ RIGHT button (to GND, internal pull-up) |
| 46 | PB9 | Top | ✓ BACK button (to GND, internal pull-up) |
| 47 | VSS_3 | Top | ground |
| 48 | VDD_3 | Top | digital supply |

Unclaimed: PA0–PA7, PA9, PA10, PB0/PB1 (pins 10–20, 30, 31) —
two more pins are now traced: **PB5 (pin 41), load-bank FET gate
(SMD-marked 3400), active-high** — driving PB5 high (or enabling a
pull-up) turns the whole 4×20Ω bank ON (~1 A at 5 V, ~5 W); and
**PB2 (pin 20), RS2228 USB-mux SELECT** — it switches the DUT-side
USB data lines between passthrough (the stock FNB58 behavior) and a
measurement path into the MCU (measuring/trigger functions — NOT a
USB-device connection; the MCU-side landing pins are unknown). The
CDC console is permanently wired to its own connector (micro-B →
PA11/PA12) and never goes through the mux. Select polarity is UNKNOWN
yet — never drive PB2 blindly. **Never drive unclaimed pins blindly,
and never pull PB5 up outside deliberate load-bank tests** (r24/25
heated the board doing exactly that).

I2C1 bus device map (0x0E-verified r20): TPA626 power monitor @ 0x40
(mfr 0x5549, die 0x2260), FUSB302B PD PHY @ 0x22 (INT_N = PC13),
KXTJ3-1057 accelerometer @ 0x0E (whoami 0x35), 0x0C = the TPA626's
SMBus Alert Response Address. The W25Q128 flash (JEDEC EF 40 18) shares
SPI2 with the LCD via CS=PA13.

## PD sink flow (r104)

1. Boot does **not** touch the FUSB302 (boot-time init browned the
   board out on charger-only power — r87).
2. **OK on the PD screen** = a scan: full FUSB302 re-init (orientation
   measured per line with the measure block powered, single-CC BMC TX
   driver, auto-GoodCRC rev 2.0 + retries), Soft_Reset, then
   Get_Source_Cap (type **7** — type 5 is Ping, which cost rounds
   r88-r98), capture, and a Request for the first fixed PDO.
3. While PD is on, the 10 ms / INT-driven poll answers every
   Source_Capabilities with a Request (tSenderResponse is 24-30 ms).
4. The scan line carries the diagnostics:
   `[pd] scan cc=2 m=03 r=2 g=1 x=0 f=…` — cc = routed line, m = BC_LVL
   of both lines (high nibble CC1, low CC2; 0 = no Rp there), r = last
   TX outcome (0 acked / 1 retryfail / 2 never started), g = hardware
   ever auto-acked an incoming frame, x = transmit collision, f = raw
   hex of whatever still sits in the RX FIFO.

Known issue: the Request itself usually comes back `r=2` (the
transmitter does not start for the second back-to-back message), so the
contract does not hold yet — the caps/scan path is fully working. The
FUSB302's CC2 line is the charger's active line on this unit.

## Console

115200 8N1 CDC. Boot banner, `[i2c]`/`[meter]` bring-up lines,
`[boot] rst=…` (fresh CRM reset-cause flags, sticky ones cleared each
boot) with the LEXT probe result, `[exti] isr line=N ok` once per
interrupt line (buttons = the canary), `[diag rNNN] t= q= f= i= pd=…`
heartbeats, and the PD lines above. The console queue is non-blocking;
a slow host delays lines instead of losing them.

## PC link (r134/r135) — OpenFNB58 protocol, hardware-verified

**Hold OK while plugging in** and the device enumerates as the stock
FNB58 measurement interface instead of the CDC console: VID `2E3C`,
PID `5558`, one HID interface, interrupt endpoints `0x83`/`0x03`,
64-byte un-numbered input/output reports. The console stays the
default — no button, no change. In PC-link mode the buzzer beeps at
700 Hz (console boots at 500 Hz), the LCD shows the live power screen
plus an `r135 i1=… i2=… p=…` status line (host command tallies), no
PD task runs (the OK press that selected the mode must not trigger a
session), and the TPA626 is reconfigured for fresh samples at 100 Hz
(AVG=1, 588 µs conversions) instead of the console's AVG=64.

Wire protocol as implemented by [OpenFNB58](https://github.com/yeckel/OpenFNB58)
(`fnb58.py` and the Qt app work unmodified): host commands are
64-byte reports `AA <cmd> … CRC8` (CRC-8 poly 0x39 init 0x42 over
bytes 1–62, stored in byte 63) — `81` INIT1 / `82` INIT2 / `83` POLL.
The device streams from enumeration (25 reports/s, `AA 04` + 4 ×
15-byte samples): voltage u32/100 000 V, current u32/100 000 A,
D+ u16/1000 V, D− u16/1000 V, unknown u8, temp u16/10 °C. **Voltage
and current are measured (TPA626); temperature (r136) is the internal
die sensor — ADC1 CH16, VREFINT (CH17)-corrected, refreshed every
report and shown on the status line and in the console heartbeat;
D+ and D− remain 0 sentinels** until their ADC taps are traced (the
protocol heuristics then read "USB SDP"). Trigger commands (`85`/`86`)
are accepted and ignored — the trigger circuit is not wired.

Verified on hardware (r135): full-rate stream through a stock hidapi
client, live voltage on the open input, reports flowing before the
INITs so the app's drain→init→read sequence behaves as it expects.

Two bring-up findings worth keeping. Windows may bind WinUSB/libusb
to `2E3C:5558` (leftover driver experiments) — then hidapi sees
nothing while libusb tools still write; the tell in a USBPcap is no
report-descriptor GET after SET CONFIG (fix: uninstall the device +
driver, replug). And the vendor stack's `usbd_eptn_handler` passes
class in/out handlers the endpoint **number** (3), not the address —
compare like `usbd_core.c` does (`ept_num & 0x7F == ept_addr & 0x7F`);
r134 compared against `0x83` and sent exactly one report, ever.

Implementation: `c/fnb58_hid.c` (descriptors, endpoints, command
mailbox — the vendor `custom_hid` class as a template), `fnb58_usb.c`
picks the class at boot (`fnb58_usb_init_hid`), and the Rust
main loop samples the meter and builds the reports.

## QC2.0/QC3.0 trigger (r189–r197) — hardware-verified

The QC screen (3rd in the LEFT/RIGHT cycle) triggers QC2 voltages
5/9/12/20 V; the QC3 screen (4th) triggers continuous-mode targets in
native 200 mV steps (3.6–20 V) — the charger steps one pulse at a time
toward the target (`SET` on screen tracks it). Both work through the
traced data-line front end: **PA5 (DAC2_OUT) drives D+ via 100 Ω, PA4
(DAC1_OUT) drives D− via 100 Ω; PA3/PA2 (ADC12_IN3/IN2) read D+/D− via
1 kΩ each**. The protocol state machines are the vendored `qc-protocol`
crate (pure logic, host-tested); the board layer is `dac.rs`
(drive/release) + `adc.rs` (`read_line_mv`) + `qc_task.rs` (the task).

- **Levels are DAC codes** (0.6 V ≈ 745, 3.3 V = 4095, VDDA-ratiometric
  via VREFINT) — no resistor ladder needed.
- **Release = channel power-down** (`d1en`/`d2en` clear), NOT pad-mode
  switching: the r193 hardware round proved the DAC output buffer
  drives its pad in every GPIO mode on this part (the code-0 outputs
  held both lines at ~0 V and forced `NODCP`); with the channels
  disabled the lines belong to the charger again. Drive = code write,
  then channel enable.
- **Entry handshake**: D+ held at 0.6 V, D− released (the charger
  drives it — shorts it to D+ during the glitch window, then pulls it
  low to accept requests). Every transition logs `[qc] …` with `t=`
  stamps; failures name the reason (`NODCP` = the short was never
  seen, `NOENTRY` = seen but never dropped).
- **QC3 continuous mode** (r196): after entry, the `(Mid, High)` hold
  requests continuous; each pulse (Up = both High, Down = both Mid,
  1 ms) moves the output 200 mV. Leaving continuous = the 5 V pair
  HELD (a sustained pair outlasts the pulse filter, so the charger
  reads it as a fixed request and exits) before the next request.
- **Verification is closed-loop**: the request levels are read back on
  the taps (screen + diag `dp=`/`dm=`) and the output is followed on
  the published TPA626 bus voltage (`vb=`) — `FOLLOWED`/`IGNORED`
  verdicts per fixed request.

The `run-bootloader` console command (r184–r188, BPR battery-domain
cookie + the BPR *peripheral clock* fix) reboots into the update
bootloader without the OK button; `tools/fnb58-flash.py` installs a
new app over the bootloader's CDC link.

## Build and flash

```sh
./make-bin.sh    # cargo build --release + rust-objcopy
                 # -> fnb58-bl-rNNN.bin (bootloader, <= 64 KiB guard)
                 # -> fnb58-app-rNNN.bin (firmware, linked at 0x08010000)
                 # -> fnb58-full-rNNN.bin (bl padded to 64 KiB + app) + md5s
```

### r180: flash layout under the custom bootloader

| Region | Range | Contents |
|---|---|---|
| Bootloader | 0x08000000..0x08010000 | `bootloader/` — CDC update port (tag `bl1`) |
| App | 0x08010000..0x080FF800 | this firmware (main + pdmin bins share `firmware/memory.x`) |
| Trailer | 0x080FF800..0x08100000 | image-valid metadata: magic `FNB5`, size, CRC-32, ~CRC |

The trailer is the boot gate: the bootloader jumps only when the
trailer's CRC matches the app bytes actually in flash. Because of that,
`fnb58-full-rNNN.bin` is a **1 MiB install image that embeds the
trailer** (`tools/mkfull.py` computes it from the app binary) — a bare
bl+app concatenation would leave the trailer erased and the device
would wait in update mode forever, even right after a DFU install. Any
CDC upload without a fresh `COMMIT` makes the trailer disagree with
flash again → back to update mode.

The bootloader (tag `bl4`, 21.7 KB) starts when OK (PB4) is held at
reset — or automatically when the trailer says the app is
empty/invalid. It runs the vendor C CDC stack (2E3C:5740, same VID/PID
as the app console), shows an LCD banner + one beep, and serves a text
protocol (`INFO` / `BEGIN <size>` / a back-to-back stream of
2056-byte CRC-checked blocks, one per 2 KiB sector / `COMMIT <size>
<crc8>` / `RESET`) — every failure is a loud `[bl] ERR`/`[bl] NAK`
line. bl4 stays in the Block phase until the announced block count is
consumed (countdown on ACK, NAK'd blocks retried in place) or 3 s of
silence, its writable-window guard covers the whole trailer sector, and
its LCD constructor claims the PB3 backlight gate high (bl1 left the
phase after every block; bl2 half-wrote the trailer via the narrow
guard; bl3 left the backlight floating — screen dark — all found in
the first hardware rounds). The trailer is
written only when the device's own CRC of the written region matches
the host's, so an interrupted or bad upload always leaves update mode
reachable. The bootloader's own 64 KiB is hard-refused by its flash
driver's address guard.

Update flow after the one-time install:

```sh
# one-time install of both images via the BOOT0 DFU (trailer embedded —
# the device boots the app on the FIRST power-up):
python ../fnb58-research/at32dfu.py fnb58-full-r188.bin   # at 0x08000000
# later app-only updates (hold OK while pressing reset):
python tools/fnb58-flash.py COM7 fnb58-app-rNNN.bin       # pip install pyserial
# adopt an app installed by other means (no upload; device CRCs its own
# flash against the local image before writing the trailer):
python tools/fnb58-flash.py COM7 --adopt fnb58-app-rNNN.bin
```

The app console also accepts `run-bootloader` + Enter (r184): it sets
the battery-domain request cookie (BPR DT2 = 0xB007) and soft-resets;
the bootloader consumes the cookie and enters update mode — no
OK-held + reset dance needed. The cookie is cleared when consumed and
does not survive power removal (a power cycle just boots normally).
r188 note: the BPR peripheral needs BOTH APB1 clocks (PWC bit 28 and
BPR bit 27) enabled before its DT registers respond — the missing BPR
clock gate was why every battery-domain cookie read 0 since the r16x
crash-diagnosis rounds; the boot counter (`boots=`) now actually
increments across soft resets.
`tools/fnb58-flash.py` refuses to write anything until the device
answers `INFO` with a `[bl]` line — talking to the app console by
mistake fails loudly, never silently. PC-link (OpenFNB58 HID) moved
from OK-held to **BACK-held (PB9) at boot** in r180, because the
bootloader owns OK-at-boot. The app bins set `VTOR = 0x08010000` as
their first act (cortex-m-rt's `set-vtor` is off; kws-x1 does the same
write).

Flashing: **BOOT0 (pin 44) strapped high at power-on → the AT32 mask-ROM
boot ROM enumerates as a USB DFU device (2E3C:DF11) → flash
`fnb58-firmware.bin` with Artery ICP (DFU mode) or `at32dfu.py`**
(`../fnb58-research/`). The mask ROM is indestructible, so this path
always works — it never touches PA13/PA14. Each round ships a uniquely
named copy (`fnb58-rNNN-tag.bin`) plus the build tag in every heartbeat
— three flashes silently failed to take before that protocol existed.

**⚠ One-way door (for the STOCK firmware only):** the first full flash
erase destroyed the stock FNIRSI firmware *and* its proprietary USB
bootloader — there is no way back to stock (`Fnb58V0.68.ufn` in
`../fnb58-research/` is useless without that bootloader). Re-flashing
*this* firmware, however, is always possible via the BOOT0-high DFU.
Consequence: the SWD pins (PA13/PA14, pins 34/37) are NOT needed for
flashing — the firmware may repurpose them as GPIO by setting the
SWJTAG remap to `100` (full SWJ disabled); the cost is only runtime
SWD debugging, which this project never uses.

## Roadmap

1. PD: the charger acknowledges Get_Source_Cap but never the Request
   (isolated — same with the hand-rolled driver and the vendored
   engine). Needs a wire capture (USB-C breakout + analyzer) or a
   board-level answer (parallel Rd, charger quirk).
2. PD profiles screen: request a chosen PDO, PPS support.
3. RTC driver on the confirmed 32 kHz crystal (PC14/15).
4. KXTJ3 tap/wake features; W25Q128 logging; ADC (ripple/VBUS taps).
5. PC-link sentinels: trace the D+/D− ADC taps (likely among the
   unclaimed PA0–PA7) to replace the last 0 sentinels in the
   OpenFNB58 stream (temperature is live since r136).
