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

## Verified hardware pin map (probed on the real board)

| Function | Pins | Verified by |
|---|---|---|
| LCD ST7789V 320x240 | SPI2: SCK=PB13, MOSI=PB15, CS=PB12, DC=PB11, RST=PB10 | r40-r53: full driver + pattern on glass |
| LCD backlight | PB3 (active high) | user trace, r42+ |
| I2C1 bus | PB6=SCL, PB7=SDA | r18-r21 scans; devices below |
| TPA626 power monitor | I2C 0x40 (mfr 0x5549, die 0x2260 = INA226-compatible), 10 mΩ shunt assumed | r20-r23 live VBUS, r62+ V/A/W |
| FUSB302B PD PHY | I2C 0x22, **INT_N = PC13** (LQFP48 pin 2, open-drain, needs pull-up) | r20 id=91; PC13 traced by user, r102 EXTINT proven |
| KXTJ3-1057 accelerometer | I2C 0x0E (whoami 0x35) | r20 |
| "Mystery" 0x0C | **SMBus Alert Response Address of the TPA626** — its ALERT function shares the I2C bus; ACKs 0x0C and ignores all reads (that is what the ARA does) | r20-r22 dump behavior; identified later |
| Buttons (to GND, internal pull-ups, active-low) | LEFT=PA15, OK=PB4, BACK=PB9, RIGHT=PB8 | r59-r61 tracer; r102 EXINT |
| FUSB302 INT_N | PC13 → EXINT13 falling | r102: `isr line=13 ok` |
| USB FS | PA11=D-, PA12=D+ | working CDC since r15 |
| SWD | PA13=SWDIO, PA14=SWCLK | ICP flashing path |
| Crystal | 8 MHz on PD0/PD1 (OSC_IN/OUT — NOT PC14/15) | vendor example + r15 |
| 32 kHz crystal (LEXT) | PC14/PC15 — **CONFIRMED present and oscillating** (`lext=1`, r126). The r86-era `lext=0` was a false negative: the probe polled the wrong flag (CLKINT.LEXTSTBLF, the interrupt flag) and skipped the backup-domain unlock (PWC BPWEN). PC14/15 are RESERVED for the RTC; an RTC driver is now a real option. | r126 fixed probe |
| Buzzer | PA8 (LQFP48 pin 29) — square wave 2 kHz/500 Hz via busy toggle | r121-r122: audible on hw |
| RS2228 USB2 mux | present on the board — **pins and purpose unidentified** | board inspection |
| W25Q128 flash | shares SPI2 (SCK=PB13, MISO=PB14, MOSI=PB15), CS unconfirmed | r33 probe |
| PD power path | side slider isolates the CC load ("PD switch"); internal FUSB302 Rd (SWITCHES0 PDWN bits) also presents a sink | r84+ behavior |

LQFP48 leftovers (unclaimed, untraced): PA0-PA7, PA9, PA10,
PB0/PB1/PB2/PB5, and the load-bank FET gates — **never drive them
blindly** (r24/25 heated the board doing that). (PA8 = buzzer, PA15 =
LEFT, PB4 = OK, PB9 = BACK, PB8 = RIGHT, PB6/PB7 = I2C1, PC13 =
FUSB302 INT_N, PC14/15 = LEXT crystal are claimed.)

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

## Build and flash

```sh
./make-bin.sh    # cargo build --release + rust-objcopy -> fnb58-firmware.bin
```

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
