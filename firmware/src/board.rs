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
//! The clock profile uses the internal 8 MHz HICK (PLL input fixed at
//! HICK/2 = 4 MHz, times 60) so the scaffold does not depend on the board's
//! crystal: SCLK/AHB 240 MHz, APB1/APB2 120 MHz (timers 240 MHz). If the
//! board has a crystal, switch `PLL_SOURCE` to `Hext` and set `HEXT_HZ`.

use embassy_at32::clocks::{AhbDiv, ApbDiv, Config, PllSource};
use embassy_at32::gpio::{Drive, Port};

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

// --- UNVERIFIED PIN MAP (correct here after hardware RE) ---

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
pub const PIN_BACKLIGHT: (Port, u8) = (Port::A, 8);

/// SPI mode for the ST7789 (CPOL 0, CPHA 0; the KWS-X1 also used the
/// sampled-on-first-edge family).
pub const SPI_MODE: embassy_at32::spi::Mode = embassy_at32::spi::Mode::Mode0;
/// 120 MHz APB2 / 4 = 30 MHz SCK: a conservative first-light rate; /2
/// (60 MHz) is near the ST7789's write limit and can be tried next.
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

/// The clock profile described above.
pub const CLOCK: Config = Config {
    hext_hz: 8_000_000,
    pll_source: PllSource::Hick,
    pll_multiplier: 60,
    ahb: AhbDiv::Div1,
    apb1: ApbDiv::Div2,
    apb2: ApbDiv::Div2,
};

/// Timer-clock input for APB1 timers (PWM candidates) under [`CLOCK`]:
/// divided buses double, so 120 MHz x 2.
#[allow(dead_code)]
pub const TIMER1_HZ: u32 = 240_000_000;
