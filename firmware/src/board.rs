//! FNB58 board constants: clock profile and the display connector's pin
//! map.
//!
//! **EVERY PIN HERE IS UNVERIFIED.** The FNB58's display is a 320x240
//! ST7789-class panel on a 12-pin connector (user report); the AT32F403A
//! pin assignment below is the common F1-style default group for SPI1 and
//! adjacent control lines, chosen so that correcting it after hardware RE
//! is a one-block edit. `CS` is driven low rather than tied, `DC` and `RST`
//! are plain push-pull outputs, and the backlight starts as a digital line
//! (a PWM timer can replace it once the real pin is known).
//!
//! Clock strategy for USB: `clocks::configure_usb(HEXT_HZ)` runs the PLL
//! from the board's crystal so the USB bit rate is crystal-accurate (USB
//! full-speed needs about ±0.25%; the ±1% HICK drifts several bit times
//! per packet, which the host reads as corruption — a device that
//! attaches but fails every descriptor request). When no crystal
//! oscillates, it falls back to the 48 MHz HICK profile, whose USB
//! timing stays out of spec until ACC calibration lands. `CLOCK` below
//! records the pure-HICK PLL variant for logic-only work.

use embassy_at32::clocks::{AhbDiv, ApbDiv, Config, PllSource};
use embassy_at32::gpio::{Drive, Port};

/// The board crystal frequency. **UNVERIFIED**: 8 MHz is the standard
/// AT32 board value, but the FNB58's crystal (if present) has not been
/// identified. `configure_usb` waits a bounded time for HEXT to
/// stabilize — if no crystal exists, it falls back to HICK — but if a
/// crystal oscillates at a *different* frequency than this constant, the
/// PLL math produces a wrong system clock and USB fails again. If
/// enumeration still fails, verify this value against the board (the
/// crystal sits near the AT32, usually marked e.g. "8.000").
#[allow(dead_code)]
pub const HEXT_HZ: u32 = 8_000_000;

/// Panel geometry: the ST7789's native 240x320 memory, rotated to
/// landscape. `0x60` is MX | MV (as the KWS-X1 panel used); a full-size
/// panel needs no window offsets in this rotation.
pub const WIDTH: u16 = 320;
pub const HEIGHT: u16 = 240;
pub const MADCTL: u8 = 0x60;
/// 180-degree flipped landscape, once orientation is confirmable on glass.
#[allow(dead_code)]
pub const MADCTL_FLIPPED: u8 = 0xA0;
pub const X_OFFSET: u16 = 0;
pub const Y_OFFSET: u16 = 0;

// --- VERIFIED PINS (hardware-confirmed; full map in the README) ---

/// Buttons (verified r59-r61 tracer): to-GND switches with NO external
/// pull-ups. Enable an internal pull-up on each and treat active-low as
/// pressed. PA15 and PB4 are the freed-JTAG pins (SWJTAG remap); PB8/PB9
/// are the unused I2C1-remap alternates (I2C1 runs on PB6/PB7).
#[allow(dead_code)]
pub const PIN_BTN_LEFT: (Port, u8) = (Port::A, 15);
#[allow(dead_code)]
pub const PIN_BTN_OK: (Port, u8) = (Port::B, 4);
#[allow(dead_code)]
pub const PIN_BTN_BACK: (Port, u8) = (Port::B, 9);
#[allow(dead_code)]
pub const PIN_BTN_RIGHT: (Port, u8) = (Port::B, 8);

// --- UNVERIFIED PIN MAP (historical guesses; see README) ---

/// SPI1 SCK (F1-style fixed alternate function).
pub const PIN_SPI_SCK: (Port, u8) = (Port::A, 5);
/// SPI1 MOSI (F1-style fixed alternate function).
pub const PIN_SPI_MOSI: (Port, u8) = (Port::A, 7);
/// Panel data/command select.
pub const PIN_DC: (Port, u8) = (Port::B, 0);
/// Panel reset.
pub const PIN_RST: (Port, u8) = (Port::B, 1);
/// Panel chip select, held low while the display is driven.
pub const PIN_CS: (Port, u8) = (Port::A, 4);
/// Backlight enable (simple digital on; move to a timer channel when the
/// real pin is known).
pub const PIN_BACKLIGHT: (Port, u8) = (Port::B, 3);

/// SPI mode for the ST7789 (CPOL 0, CPHA 0; the KWS-X1 also used the
/// sampled-on-first-edge family).
pub const SPI_MODE: embassy_at32::spi::Mode = embassy_at32::spi::Mode::Mode0;
/// 96 MHz APB2 / 4 = 24 MHz SCK: a conservative first-light rate; /2
/// (48 MHz) is near the ST7789's write limit and can be tried next.
pub const SPI_DIVIDER: embassy_at32::spi::Divider = embassy_at32::spi::Divider::Div4;

/// Initial output levels: CS active low, DC idle low, reset released high,
/// backlight off until the panel is initialized.
pub const CS_LEVEL: bool = false;
pub const DC_LEVEL: bool = false;
pub const RST_LEVEL: bool = true;
pub const BACKLIGHT_LEVEL: bool = false;
pub const PIN_DRIVE: Drive = Drive::Moderate;

/// DMA unit and channel (1-based) used for panel pixel bursts.
pub const PANEL_DMA: (embassy_at32::dma::Unit, u8) = (embassy_at32::dma::Unit::Dma1, 1);

/// Power-path hardware (user board trace, 2026-10-30): four ~20-ohm
/// power resistors and one MOSFET sit directly behind the AT32. The
/// working theory: a switchable current-pulse load bank for MediaTek
/// Pump Express detection/triggering (PE works by the device drawing
/// current pulses; 20 ohm across 5 V = 250 mA per leg, 1.25 W - hot!),
/// with MOSFET gates driven by (unidentified) GPIOs. DANGER: enabling a
/// pull-up on a gate pin turns a leg on continuously. The round-24/25
/// button monitor did exactly that (chip heated, vbus drooped to 4.6 V).
/// Rule: never pull or drive any untraced pin on this board.
///
/// The 192 MHz PLL profile (kept for later stages; the smoke test runs
/// on `configure_hick48` instead).
#[allow(dead_code)]
pub const CLOCK: Config = Config {
    hext_hz: 8_000_000,
    pll_source: PllSource::Hick,
    pll_multiplier: 48,
    ahb: AhbDiv::Div1,
    apb1: ApbDiv::Div2,
    apb2: ApbDiv::Div2,
};

/// Timer-clock input for APB1 timers (PWM candidates) under [`CLOCK`]:
/// divided buses double, so 96 MHz x 2.
#[allow(dead_code)]
pub const TIMER1_HZ: u32 = 192_000_000;
