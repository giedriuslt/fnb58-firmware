# fnb58-firmware

Open firmware for the **FNIRSI FNB58** USB tester (Artery AT32F403A),
built on [`embassy-at32`](../at32f403a-embassy-rs) and ported from
[kws-x1-standalone-rs](https://github.com/kralonur/kws-x1-standalone-rs).
Status: **r239g** — working USB CDC console, 320x240 UI with
accelerometer-driven 180° screen flip and click feedback (buzzer),
live V/A/W meter, an auto-contracting PD sink flow on the vendored
`fusb302-pd` stack (the boot session completes the contract on its
own), QC2/QC3 and AFC triggers, the ripple scope, the settings store
with calibration, the energy journal and the offline recorder. The PD
passthrough sniffer is **not working** (see its section below).

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
| 12 | PA2 | Left | ✓ D− read tap — **ADC3**_IN2 (ADC123_IN2), 1 kΩ from D− (qc/afc line reads + AFC wire sampler) |
| 13 | PA3 | Bottom | ✓ D+ read tap — **ADC3**_IN3 (ADC123_IN3), 1 kΩ from D+ |
| 14 | PA4 | Bottom | ✓ D− drive — DAC1_OUT via 100 Ω (QC levels + AFC single-wire) |
| 15 | PA5 | Bottom | ✓ D+ drive — DAC2_OUT via 100 Ω (QC levels, AFC holds 0.6 V) |
| 16 | PA6 | Bottom | ✓ ripple pickup — ADC1_IN6, diode/~1.8 V clamp + 100 k pull-up node (idles ≈1.79 V) |
| 17 | PA7 | Bottom | shorted to PA6 (same clamp node); otherwise unclaimed |
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
| 39 | PB3 | Top | ✓ LCD backlight — **TMR2 CH2 PWM** (IOMUX TMR2 remap 01; idle-dimmed to 20% after 60 s, r216) |
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

r104-era known issue, resolved since: the Request used to come back
`r=2` (the transmitter would not start for the second back-to-back
message) and the contract did not hold. The flow now runs as the
automatic boot session (r138): it completes contracts on its own —
contracted receipts through r239g — with every contract clearing BUSY
(r179). The FUSB302's CC2 line is the charger's active line on this
unit.

## Console

115200 8N1 CDC. Boot banner, `[i2c]`/`[meter]` bring-up lines,
`[boot] rst=…` (fresh CRM reset-cause flags, sticky ones cleared each
boot) with the LEXT probe result, `[exti] isr line=N ok` once per
interrupt line (buttons = the canary), `[diag rNNN] t= q= f= i= pd=…`
heartbeats, and the PD lines above. The console queue is non-blocking;
a slow host delays lines instead of losing them.

Line commands (`<cmd>` + Enter; each is echoed as `[flash] cmd: …`
first — echo seen = the bytes arrived): `i` = W25Q128 JEDEC ID,
`f<sector>` = 4 KiB sector hex dump, `run-bootloader` = soft-reset
into the update mode. Diagnostics (r204–r215): `a` = one-shot scan of
the ADC1 pin block PA0..PA7 + VREF with forced long sample windows;
`rs` = one DMA burst of CH17 (VREFINT, ≈1485 counts proves the whole
ADC1+DMA1+analysis chain) followed by one on the PA6 pickup; `r2` =
the 32-bit-wide variant; `m` = a DMA memory-to-memory copy (engine
test); `af`/`ad`/`ap` = the AFC handshake-path commit, direct-entry
commit, and ping-width sweep (see the AFC section). `b` = toggle the idle dim (r216).
`c` = the applied calibration fields + the meter's raw registers
(r221: `[cal] z=… c=… v=… raw=<bus>/<cur> f=<fresh>`). `x` = advance
one screen forward (r225 — the remote navigation the screenshot
workflow drives). `shot` =
arm the screenshot capture — the next loop tick forces a full repaint
of the CURRENT screen with the LCD's capture sink on, streaming one
line per draw primitive (`F x0 y0 x1 y1 cccc` / `C x y s fg bg hh`);
`tools/fnb58-shot.py COM6 out.png` collects the stream and replays it
into a PNG (the panel's data line is write-only — no MISO — so the
primitive stream is the framebuffer; a PD exchange in flight refuses
with `[shot] refused: pd busy`, and the forced repaint never disturbs
selection modes or the calibration wizard).

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

## QC2.0/QC3.0/AFC trigger (r189–r215, screens r230/r252)

The QC screen (3rd in the LEFT/RIGHT cycle) triggers QC2 voltages
5/9/12/20 V; the QC3 screen (4th) triggers continuous-mode targets in
native 200 mV steps (3.6–20 V) — the charger steps one pulse at a time
toward the target (its SET VOLTAGE card shows the selected target; the
LINES pill reads STEPPING until the ramp reaches it, ACTIVE then — r252).
Both work through the
traced data-line front end: **PA5 (DAC2_OUT) drives D+ via 100 Ω, PA4
(DAC1_OUT) drives D− via 100 Ω; PA3/PA2 read D+/D− via 1 kΩ each on
ADC3** (r201: the taps moved from ADC1 to ADC3 — PA2/PA3 are
ADC123_IN2/IN3, which isolates the timing-critical reads from ADC1's
ripple bursts; PA2/PA3 also sit in true analog mode since). The
protocol state machines are the vendored `qc-protocol` crate (pure
logic, host-tested); the board layer is `dac.rs`
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
- **AFC** (r198–r215, the 5th screen): Samsung Adaptive Fast Charging
  over the same front end — after the DCP handshake, D+ stays at 0.6 V
  and a single-wire exchange runs on D− (the
  `single-wire-protocols` crate — a git dependency until r252, now the
  copy under `vendor/` that adds the 12 V request; 160 µs UI,
  DWT-cycle-counter timing —
  start-relative since r210 — cached DAC codes, a 1.8 V ping level
  since r214 matching the reference phone's GPIO, and a fast
  single-conversion tap read for the wire budget). **Screen flow**
  (r252: laid out as the QC2 screen — one row per request, the pending
  one framed in the screen's accent, ACTIVE on the row the charger is
  holding, the state and verdict on a status row): LEFT/RIGHT in
  selection steps the pending request through the rows (`5V/9V/12V`,
  `[afc] tgt=` on the console); OK commits — the task releases the
  lines, runs the DCP entry, then the exchange. **Task states** (the
  shared `STATE` atomic + the AFC screen's names): OFF → RELEASING →
  ENTRY (DCP handshake, `[qc] entering`) → DETECT (the wire exchange)
  → AFC (session active, requests followed on VBUS) with FAIL + a
  reason (`SILENT` = the adapter never answered the wire, `WIRE` = a
  damaged exchange, `NODCP`/`NOENTRY` = entry failed). A commit while
  active retargets (9 V ↔ 12 V) or stops (5 V + the crate's 100-UI reset
  pulse); a second OK on the request already held does nothing (r252 —
  it used to stop the live session); a commit from another screen stops
  AFC first. A retarget that lands before the session's first request
  has been sent is refused with a log line (`[afc] retarget refused`,
  OK again) instead of being dropped. A `SILENT`
  result auto-retries once through the direct entry (no QC handshake,
  r211). **Status on this charger** (r212–r215, partner-side verdict):
  the adapter answers no D− single-wire ping at any width (1–60 ms),
  level (1.8–3.2 V) or entry path — our side is verified against the
  Samsung reference driver end to end (`afc_charger_intf.c`).
  **Console probes**: `af` = full AFC commit (handshake path), `ad` =
  direct entry (no handshake), `ap` = ping-width sweep — each logs the
  wire envelope `[afc] wire ping= rx_lo= rx_hi= n=` (raw counts).
- **The 12 V request (r252) is an fnb58 extension, not a reference
  value.** The reference driver, kws-x1 and the `single-wire-protocols`
  revision kws-x1 pins (`cdd1101`) carry only 5 V (`0x08`) and 9 V
  (`0x46`). Both decode under HiSilicon's AFC `V_I_Byte` layout — "16
  voltages (5-20V) and 16 current levels (0.75-3A) can be specified
  using one Fast Charge V_I_Byte" (NXP FTF 2016, *Smart USB: Fast
  Charging Products*, slide 23): the high nibble is the voltage code
  (5 V + code), the low nibble the current code (0.75 A + 0.15 A ×
  code). `0x08` is 5 V at code 8 (1.95 A, the 2 A request) and `0x46`
  is 9 V at code 6 (1.65 A, the 1.67 A request), so 12 V is voltage
  code 7. No reference gives the current code of a 12 V request: it is
  a choice, `5` = 1.5 A, so the byte is `0x75`
  (`V12_CURRENT_CODE` in `vendor/single-wire-protocols/src/afc.rs`).
  **No adapter has been verified to honour it** — the charger below
  answers no AFC ping at all, so on it the 12 V row ends in `FAIL
  SILENT` like 5 V and 9 V do; a 9 V-only adapter is expected to leave
  the output where it was (`OUTPUT IGNORED`). The crate is vendored for
  it (`vendor/single-wire-protocols` + a `[patch]` entry in the
  workspace manifest): the same bytes as `cdd1101` except `src/afc.rs`,
  one doc line in `src/lib.rs`, `tests/afc.rs` (whose golden-edge tests
  pin `0x75` on the wire; `cargo test -p single-wire-protocols`, host)
  and the manifest's two `workspace = true` dependencies, pinned here to
  the versions that workspace declares.
- **The three trigger screens (r252)** draw their chrome once per
  identity change and overdraw live values in place. QC2 and AFC: the
  pending request framed in the screen's accent, ACTIVE on the row
  being held (the session's own target, `qc_task::AFC_SET_MV` — the
  frame may have moved on), the state and verdict on a status row. QC3:
  the STEPS card; SET VOLTAGE (the selected target, cyan) beside VBUS
  (yellow); the LINES card with the state pill. Fixed here: the status
  rows went through the `text()` helper, which truncates at 23
  characters ("…OUTPUT FOLLO"), and QC2's failure line said "QC FAIL
  FAIL NODCP"; the QC3 value pads (12 cells at scale 3) ran through
  the card borders and its badge off the panel; the overview's badge
  gate never admitted the AFC session's state; and the r247 QC3
  two-gate split, lost in the r248 cleanup and in no commit, is
  restored (the old gate keyed on the ramp position, so LEFT/RIGHT
  never reached the screen and every ramp pulse repainted the body).
  `fnb58-research/ui-hostcheck` compiles the real `ui.rs` on the host
  against a recording fake of `Lcd` and checks these screens pixel by
  pixel (`cargo run -- <out-dir>`).
- **Verification is closed-loop**: the request levels are read back on
  the taps (screen + diag `dp=`/`dm=`) and the output is followed on
  the published TPA626 bus voltage (`vb=`) — `FOLLOWED`/`IGNORED`
  verdicts per fixed request.

The `run-bootloader` console command (r184–r188, BPR battery-domain
cookie + the BPR *peripheral clock* fix) reboots into the update
bootloader without the OK button; `tools/fnb58-flash.py` installs a
new app over the bootloader's CDC link — and (r198) when it finds the
tester in log mode it sends `run-bootloader` itself, waits for the
re-enumeration (rescanning if Windows renumbers the COM port) and
flashes: one command from any state.

## Ripple scope (r199–r209)

The 6th screen is a port of kws-x1's ripple tab. Analysis is a 1:1
port into the `ripple-core` workspace crate (host-tested, 11 tests) of
kws-x1's `firmware-core` ripple/spectrum modules; the acquisition task
(`ripple.rs`) follows their dedicated-ADC arrangement as far as the
chip allows:

- **Hardware** (user trace): **PA6 / ADC12_IN6 / LQFP48 pin 16** — a
  diode to GND with a 100 k pull-up biases the node to ~0.6 V, so a
  raw reading near ~745 counts is the *expected* idle, and the
  analysis is mean-centered (exact DC removal) so the bias drops out.
- **Why ADC1 + DMA1 ch1, shared with the internal channels — not a
  dedicated ADC (r200/r201, RM receipts)**: the first hardware round
  (r199, ADC2 + DMA1 ch3) failed every capture, and the RM explains
  why: **Table 9-5 (p.133) lists the DMA request sources as 0x00=No
  select, 0x01=ADC1, 0x02=RESERVED, 0x03=ADC3 — the AT32F403A gives
  ADC2 no DMA request line** (p.323), and ADC3's external channels are
  only IN0..IN3 and IN10..IN13 (p.318), so it cannot see IN6/PA6.
  The r199 build also exposed that the embassy-at32 HAL's
  `dma::Request` ADC+I2C codes were off by a row (Adc1=0x00 is "No
  select"); the HAL enum now carries the RM's values. The r200 log
  then proved the chip runs the FIXED request mapping regardless (the
  SRC_SEL registers read back 0 after write) — RM Table 9-3 (p.132)
  puts ADC1 on **DMA1 channel 1**, so the burst rides ch1: the
  vendor's repeat_conversion_loop_transfer pairing. The D+/D− taps
  moved to **ADC3** (r201, user-directed; PA2/PA3 are ADC123_IN2/IN3
  per the datasheet), which removes every timing-critical reader from
  ADC1: bursts are bracketed by adc.rs's `BURST` fail-fast flag, the
  only co-tenant is the 2 s temperature/VREFINT read (ADC1-only
  peripherals) with a cached-VDDA fallback for the tap scaling, and no
  long-held lock ever exists (SysTick-safe). The ratiometric reference
  is ADC1's own VREFINT (CH17) read before each burst. The r200 log
  also exposed a lost-wakeup race in the HAL's DMA `wait()` (the ISR
  cleared the completion flags before the woken poll could read them)
  — the ISR now wakes only and the poll consumes the flags.
- **kws-x1 semantics kept**: 5 timebases (20/50/100/200/500 µs/div, a
  nominal `~` label), rising-edge trigger with hysteresis (never
  wrapping the capture), 112-column min/max envelope trace over 8×4
  divisions, DC-removed RMS + PeakHistory-smoothed p-p (65/35
  mean/max blend over 30 captures), Hann-windowed FFT with 224 bars
  behind the trace and 3 labelled peaks, 9 AUTO/LOCK mV/div scales,
  RUN/HOLD, CLIPPED/OFF SCALE/NO REF flags. Buttons: OK opens/cycles
  the focused setting (time → scale → run/hold), LEFT/RIGHT adjust it,
  BACK closes; changing the timebase drops the hold (a held capture
  cannot be re-timed), and a hold lasts only while the screen shows.
- **Documented deviations** (hardware-forced): no AT32 PGA →
  gain 1 (voltages are pin-referred and deliberately NOT
  bus-calibrated, as in kws-x1); the AT32's 8 discrete sample windows
  replace the HC32's free cycle count (rate math in half-cycles,
  12.5-cycle conversion); the shared (not dedicated) ADC above.
  Drawing deviations: 2-px envelope bars without connector segments,
  and the numbered peak list replaces the per-bar digit markers. There
  is no long-press, so kws-x1's long-OK reset-to-defaults is absent.
- Console evidence: `[ripple] adc1+dma1c1 up` at boot, a
  `[ripple] tb= pp= rms= mn= mx= tr=` line on the 2 s heartbeat while
  the screen is shown (`mn`/`mx` are raw counts — the ~745 diode idle
  proves the front end), and every failure path logs with a register
  dump: `[ripple] fail <cause> c2= sts= ch= cnt=` + `sel= s0= s1023=`
  (first failure and every 20th after), `ok fails=N` on recovery, and
  `unwake salvage` if the DMA interrupt ever fails to wake a finished
  transfer (the capture is salvaged by checking the flag).

## Backlight idle dim (r216)

PB3 (the backlight gate, high = on) moved from a plain GPIO output to
**TMR2 CH2 PWM** — the only timer function PB3 has (IOMUX TMR2 partial
remap 01; PA15, CH1's remapped pin, is the LEFT button and unaffected).
20 kHz (above audible/flicker), PWM mode A active-high with buffered
compare, 192 MHz timer clock → 9600 ticks/period. After **60 s without
a button press** the backlight dims to **20%**; any press restores 100%
(`[bl] dim 20%` / `[bl] wake` on the console). Console `b` toggles the
dim immediately (hardware check without waiting the minute). PC-link
mode keeps full brightness.

## Settings store (r218)

The first 20 sectors of the external W25Q128 (0x000000..0x014000) are
the settings area — a port of kws-x1's rotating-record scheme
(settings_store.rs, the record format lives in the host-tested
`settings-core` crate): a sector is claimed with "KWST" (0x4b57_5354)
at its base, 8-byte `[word, !word]` records append after the claim,
claims advance from sector 0 upward (r220 — kws-x1's direction; the
r218 build had it inverted), the scan walks claims oldest-first with
the later record winning, a nearly-full sector is padded with (0, 0)
pairs, and only the newest claimed sector ever has room. **Deviation**
(hardware-forced): kws-x1 never erases (blank internal flash); the
W25Q128 carries the stock firmware's littlefs, so claiming ERASES the
sector first — once per ~10,240 records, never during saves. The
region boundary protects everything beyond sector 20.

## Offline recorder (r239)

The REC screen records calibrated voltage/current samples into the
external flash's FREE area — sector 20 is the log's meta (magic `FNRL`,
the interval seconds, an `FEND` word programmed only by an explicit
stop), sectors 21..4095 hold the samples, 4 bytes each (u16 mV LE,
u16 mA LE, magnitudes — the overview's own units), 1024 per sector:
capacity ~4.17M samples (~48 days at 1 s). The interval is a settings
row (SET: REC INT, 1-2-3-5-10-20-30-60 s) captured at START; RESUME
keeps the log's own interval, so a log is single-interval and the time
axis is sample-index x interval. An interrupted log (no FEND — a VBUS
dip) auto-resumes at boot; the cursor is recovered by a binary search
over sector blankness plus one word-scan. CLEAR erases one used sector
per main-loop tick (UI stays live, off PD-exchange windows). Export:
console `ri` (info) and `rd <first> <count>` (hex lines, 6 records per
64-byte line), and `tools/fnb58-rec-export.py COMx out.csv` writes CSV
columns t_s,v_v,a_a,w_w.

**Screen 10/11 (SET, r226)**: kws-x1's two-level page — LEFT/RIGHT
always navigate (the r218 always-adjusting screen blocked reaching the
later screens; r222 gated the adjust, r226 grew the rows); **OK enters
Row mode** (hint `OK:SET` → `LR:ROW OK:EDIT BK:X`), LEFT/RIGHT move the
cursor over the two rows (DIM IDLE, ENERGY SAVE), **OK opens the row's
editor** (`LR:ADJ OK:SAVE BK:X`), LEFT/RIGHT change that row's value,
OK saves it (`[set] dim=N min EN=<interval> saved s=<sect>` on the
console) and returns to Row mode, BACK steps back a level. The dim
timeout applies immediately; the energy interval arms the journal's
checkpoint cadence (previewing never checkpoints per arrow tap). Boot
evidence: `[set] stored dim=N min` + `[set] energy <interval>` or
`[set] no stored settings (default dim=1)`; a JEDEC mismatch logs
`[set] flash JEDEC FAIL` and leaves defaults.

## Calibration (r221/r221b/r226)

Screen 11/11 (CAL, r226 landing) — the screen first shows the APPLIED
card (the stored zero/gain fields) and **OK enters the wizard**:
kws-x1's calibration steps — a zero offset subtracted first, then gain
corrections in ppm, found from **user-supplied reference values** (each
entry starts at this meter's own averaged reading; walk it to what your
reference meter shows — no fixed calibration points). Inside the
wizard LEFT/RIGHT adjust (±1 per press, hold for ±10), OK walks, BACK
cancels to the landing. Three steps, nothing applied until the last OK:

1. **0 CURRENT** — unplug the load, OK takes the zero (refused above
   20 mA: the message stays and the step repeats).
2. **CURRENT** — a steady load ≥ 0.5 A: walk the entered amps to your
   reference, OK confirms; > ±5% off refuses.
3. **VOLTAGE** — ≥ 4 V on the bus: same walk, OK applies and saves
   (`[set] cal z=… c=… v=…` on the console).

Records ride the settings store (one per field, kws-x1's `0xCA` codec,
later records win); the applied values feed the power screen, the
PC-link report (at native 10 µV/100 µA resolution) and console `c` —
the protocol paths stay raw. Refusal messages: `NO STEADY READ YET`,
`CURRENT FLOWS: UNPLUG`, `NEEDS 0.5A OR MORE`, `NEEDS 4V OR MORE`,
`OVER 5 OUT OF RANGE`.

**Current range (r221b)**: the TPA626's CAL register sets the current
register's span — one count = (2048/CAL) × 2.5 µV of shunt. r46's
CAL=10240 (50 µA/count) capped the register at ±1.638 A through the
10 mΩ shunt; r222 programs **CAL=2048** (kws-x1's own value for their
5 mΩ/500 µA setup) = 250 µA/count, the register maps the shunt
register 1:1, and the span is **±8.19 A** — the 0–7 A range with
margin. Sub-0.25 mA resolution is traded away.

## Graphical screens (r224–r227)

kws-x1's screen designs ported onto the 320×240 panel, all built from
rounded outlined cards with notched labels (r224's `card()` helper) so
the screenshot capture sees every primitive:

- **Screen 1/11 (OVERVIEW, r224)**: the header (title in the tab's
  colour, four group-tab pills, page dots, rule), the VOLTAGE /
  CURRENT / POWER cards, the MAX card (session peaks — OK resets them)
  and the PROTOCOL strip (the live badge: PD contract, QC2/QC3/AFC
  trigger, USB idle). The power register read is dropped — power
  derives from the calibrated V×I.
- **Screen 2/11 (GRAPH, r227)**: the NOW/MAX card (the latest second
  and the window's highest V/A/W) over a plot of the per-second
  history — one column per second (highest per pixel column when the
  window outgrows the plot) in the trace's colour over its tint, the
  auto-scaled full-scale label (peak +10% headroom on the
  10/12/15/20/25/30/40/50/60/80 steps), OK walks the focus
  None → trace → window → None, LEFT/RIGHT change the focused value.
  RAM only — a restart clears it.
- **Screen 3/11 (LINES, r225)**: kws-x1's Lines page with only the
  D+/D− cards (this tester's CC lines are not wired to the MCU); live
  20 Hz tap readings, "approximate" by design.
- **Screen 4/11 (ENERGY, r226)**: the CHARGE / ENERGY / TIME totals in
  stacked cards; OK resets them (and checkpoints the zeros).
- The 5×7 font grew `<`, `>`, `+` and `h` (append-only; the shot
  tool's table is kept in sync).
- **Border lesson (r227b)**: padded text overdraws must be bounded by
  the card interior, not the formatter's 12-char cap — the value rows'
  padded trailing spaces were opaque black cells that erased the
  cards' right borders every redraw (the flicker).

## PD passthrough sniffer (r226–r239) — NOT WORKING

The settings store carries a **PD PASS** row (SET screen) that is
meant to select kws-x1's receive-only passthrough sniffer at boot:
the meter releases its CC pull-downs (Rd), listens on the charger's
CC line without ever transmitting or acknowledging (auto-CRC off),
and the SNIFF screen renders the heard PD traffic (kws-x1's
`pd_sniff` describe()). The mode is boot-chosen — the flag applies at
the next boot.

**It does not work on this hardware.** As of r239g a pass=ON boot
still runs the normal sink session: the boot dispatch reads the
setting before the flash scan applies it, so it always sees OFF. The
r240–r241 attempts were reverted by user order after bench testing —
the sniffer still never ran with the full chain (charger + downstream
device) attached, and repeated power cycles did not change it. Facts
recorded for any retry:

- The FNB58 is **VBUS-powered** (kws-x1 is battery powered): an
  unconditional Rd release with no downstream device attached makes
  the charger cut VBUS and the whole meter power-cycles — kws-x1's
  release-first boot does not transfer.
- The vendored sniffer's orientation picks a CC only when exactly one
  CC carries an Rd-loaded Rp (the downstream device's Rd through the
  shared CC path); with no device it refuses (the observed
  `err=O` starts).
- The FUSB302's I2C toward 0x22 degraded in this mode on the bench
  (timeouts writing SWITCHES0) while the TPA626 at 0x40 kept
  answering — the r239f wedge profile.

The code is reverted to r239g (the device runs it); the passthrough
remains a stock-firmware feature only until someone picks this up
with hardware in hand.

## Energy journal (r226)

The energy totals checkpoint to a two-sector journal (kws-x1's
`energy_store` durability core: "EN02" pair on W25Q sectors 18/19 —
the top of the settings region — sequence + word-complements + COMMIT
records, power-cut recovery scanning past torn records,
erase-partner-then-claim rollover; the flash backend is passed per
call because the W25Q is shared, and no legacy-log migration exists
here). The interval is the ENERGY SAVE setting: OFF never writes
(RAM only), 5/10/30/60 s trade flash endurance for less power-loss
lag. Boot evidence: `[en] pair ready`, `[en] loaded mah=… mwh=…`;
failures log `[en] save FAIL`.

## Screenshot tool (r223)

`shot` arms a full repaint with the LCD's capture sink on (see the
console list); **`x` advances one screen** — the two together let a
host navigate and capture every screen without touching the buttons:
`python tools/fnb58-shot.py COM6 out.png`.

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

1. PD passthrough sniffer: **not working** (see its section) — needs a
   boot dispatch that honors the stored flag plus a release strategy
   that keeps the VBUS-powered meter alive without a downstream
   device.
2. ~~PD: the charger acknowledges Get_Source_Cap but never the
   Request~~ — resolved (the boot session completes contracts,
   r138–r179). ~~PD profiles screen / PPS~~ — shipped (r228–r237).
3. RTC driver on the confirmed 32 kHz crystal (PC14/15).
4. KXTJ3 tap/wake features. ~~W25Q128 logging~~ (the offline
   recorder, r239). ~~ADC (ripple/VBUS taps)~~ (the ripple scope,
   r199–r209).
5. PC-link sentinels: trace the D+/D− ADC taps (likely among the
   unclaimed PA0–PA7) to replace the last 0 sentinels in the
   OpenFNB58 stream (temperature is live since r136).
