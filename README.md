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

## ⚠️ The pin map is UNVERIFIED

`firmware/src/board.rs` holds every hardware assumption in one block:
display pins (SPI1 SCK/MOSI, D/C, RST, CS, backlight), the DMA channel and
the clock profile. These are the F1-style defaults for SPI1 (PA5/PA7) and
adjacent pins — **not traced from an actual FNB58 board yet**. First light
requires correcting them against the real 12-pin display connector
(continuity from the connector to the AT32's LQFP48 pins), after which the
same block records the verified map.

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
