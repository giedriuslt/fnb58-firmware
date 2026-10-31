# fnb58-firmware

Open firmware scaffold for the **FNIRSI FNB58** USB tester (Artery
AT32F403A), built on [`embassy-at32`](../at32f403a-embassy-rs) and ported
from [kws-x1-standalone-rs](https://github.com/kralonur/kws-x1-standalone-rs).

## Current state: USB CDC smoke test

The firmware enumerates as a **CDC serial port** (VID 0x2e3c, PID 0x5859)
and echoes received bytes plus a one-per-second `FNB58 alive <n>`
heartbeat. If the port appears and echoes, the MCU, flash execution, the
240 MHz clock tree, the 48 MHz USB clock, the USBFS cell and its
interrupt all work — the bring-up baseline, independent of the unverified
display pins. The ST7789 panel transport (`panel.rs`) is kept for the
next stage but not started.

Build a flashable binary (vector table at 0x08000000):

```sh
./make-bin.sh    # cargo build + rust-objcopy -O binary
```

## Verified hardware pin map (probed on the real board)

| Function | Pins | Verified by |
|---|---|---|
| LCD ST7789V 320x240 | SPI2: SCK=PB13, MOSI=PB15, CS=PB12, DC=PB11, RST=PB10 | r40-r53: full driver + pattern on glass |
| LCD backlight | PB3 (active high) | user trace, r42+ |
| I2C1 bus | PB6=SCL, PB7=SDA | r18-r21 scans; TPA626@0x40, FUSB302B@0x22, KXTJ3@0x0E, mystery@0x0C (write-only) |
| Buttons (to GND, need internal pull-ups) | LEFT=PA15, OK=PB4, BACK=PB9, RIGHT=PB8 | r59-r61 tracer (HIT with pull-up window) |
| USB FS | PA11=D-, PA12=D+ | working CDC since r15 |
| SWD | PA13=SWDIO, PA14=SWCLK | ICP flashing path |
| Crystal | 8 MHz HEXT | vendor example boots; 192 MHz PLL |
| W25Q128 flash | shares SPI2 (SCK=PB13, MISO=PB14, MOSI=PB15), CS unconfirmed | r33 probe |

Button notes: PA15 and PB4 are the freed-JTAG pins (the firmware's
SWJTAG remap releases PA15+PB3+PB4); PB8/PB9 are the I2C1-remap
alternate pins, unused because I2C1 runs on PB6/PB7. None of the four
buttons has an external pull-up — a button pin reads low when floating,
so the driver must enable an internal pull-up on each button pin and
treat active-low as pressed.

`firmware/src/board.rs` still contains the original UNVERIFIED guess
block (SPI1 pins etc.) — historical only; the verified map above and
the C glue (`c/fnb58_lcd.c`, `c/fnb58_i2c.c`) are the source of truth.

The backlight is driven as a plain GPIO for now; move it to a
`embassy_at32::pwm` TMR3 channel once its true pin (and timer channel) is
known.

## Clocks

Internal-HICK profile (no crystal dependency): PLL input HICK/2 = 4 MHz ×
60 → SCLK/AHB 240 MHz, APB1/APB2 120 MHz, timers 240 MHz, SPI1 SCK 30 MHz
(conservative first-light rate; /2 = 60 MHz is near the ST7789 write limit
and can be tried next). If the board carries a crystal, switch
`board::CLOCK` to `PllSource::Hext`.

## Flashing

Not yet possible over USB: the stock FNIRSI bootloader only accepts
encrypted `.ufn` images (see the bootloader investigation notes in the
parent project). The image targets SWD (full flash takeover at
`0x08000000`, 1 MiB / 96 KiB SRAM assumed) — Artery ICP Programmer with an
AT-Link or compatible CMSIS-DAP probe. Mass-erasing destroys the stock
firmware; the stock `v0.68` `.ufn` is archived in `../fnb58-research/` for
restoration over USB.

## Roadmap

1. Verify the display pin map on hardware → first light.
2. I²C bus bring-up: TPA626 monitor (INA226-compatible), KXTJ3-1057
   accelerometer, FUSB302B PD controller (the `fusb302-pd` crate ports
   as-is).
3. Port the KWS-X1 board layer: gated I²C bus, PD task, buttons, UI.
4. `embassy-at32` ADC module (line voltages, ripple) and flash storage.
