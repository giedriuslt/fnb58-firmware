# fnb58-firmware

Open firmware for the **FNIRSI FNB58** USB tester (Artery AT32F403A),
built on [`embassy-at32`](../at32f403a-embassy-rs) and ported from
[kws-x1-standalone-rs](https://github.com/kralonur/kws-x1-standalone-rs).
Status: **r104** — working USB CDC console, 320x240 UI, live V/A/W
meter, and a PD sink scan flow that reads source capabilities from a
charger on demand.

## Architecture

- **USB CDC**: the vendor AT32 C stack (usbd_core/cdc_class) compiled
  into the firmware by `build.rs` (`c/fnb58_usb.c` glue). Proven path —
  the hand-rolled Rust USB stack was retired after rounds 1-14.
- **I²C1**: the vendor C `i2c_application` library (`c/fnb58_i2c.c`
  glue, PB6/PB7). The Rust embassy I2C driver also works and runs the
  TPA626 meter transactions.
- **Everything else is Rust** on `embassy-at32`: clocks (192 MHz PLL
  from the 8 MHz HEXT crystal + USB DIV_4), GPIO, EXINT, the UI, the
  PD driver (`pd.rs`, register-level FUSB302B), buttons, console queue.
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
| Mystery device | I2C 0x0C — ACKs address, ignores all reads (write-only) | r20-r22 |
| Buttons (to GND, internal pull-ups, active-low) | LEFT=PA15, OK=PB4, BACK=PB9, RIGHT=PB8 | r59-r61 tracer; r102 EXINT |
| FUSB302 INT_N | PC13 → EXINT13 falling | r102: `isr line=13 ok` |
| USB FS | PA11=D-, PA12=D+ | working CDC since r15 |
| SWD | PA13=SWDIO, PA14=SWCLK | ICP flashing path |
| Crystal | 8 MHz on PD0/PD1 (OSC_IN/OUT — NOT PC14/15) | vendor example + r15 |
| LEXT 32 kHz footprint | PC14/PC15 — **not populated** (r86 lext=0) | r86 probe |
| W25Q128 flash | shares SPI2 (SCK=PB13, MISO=PB14, MOSI=PB15), CS unconfirmed | r33 probe |
| PD power path | side slider isolates the CC load ("PD switch"); internal FUSB302 Rd (SWITCHES0 PDWN bits) also presents a sink | r84+ behavior |

LQFP48 leftovers (unclaimed, untraced): PA0-PA10, PB0/PB1/PB2/PB5, and
the load-bank FET gates — **never drive them blindly** (r24/25 heated
the board doing that).

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

Flash `fnb58-firmware.bin` at 0x08000000 with Artery ICP over SWD
(AT-Link). Each round ships a uniquely named copy (`fnb58-rNNN-tag.bin`)
plus the build tag in every heartbeat — three flashes silently failed
to take before that protocol existed.

## Roadmap

1. Fix the Request transmission (r=2) so the sink contract holds and
   the charger stops power-cycling VBUS.
2. PD profiles screen: request a chosen PDO (the RDO builder is in
   place), PPS support.
3. KXTJ3 accelerometer driver, W25Q128 logging, ADC (ripple/VBUS taps).
