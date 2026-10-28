//! ST7789 panel transport over the AT32F403A's SPI1 with DMA pixel bursts,
//! ported from the KWS-X1 board layer. Geometry (320x240 landscape,
//! MADCTL `0x60`, no offsets) lives in [`crate::board`].
//!
//! Commands go out byte-wise on the SPI (D/C low for the command byte,
//! high for its data), full rows stream through the DMA channel, and every
//! frame operation ends with an idle fence so the D/C line never changes
//! mid-byte. The init sequence is the KWS-X1's proven generic ST7789
//! register block (identical controller family, `firmware-core/src/panel.rs`
//! of kws-x1-standalone-rs).

use embassy_at32::dma::{self, Channel as DmaChannel};
use embassy_at32::gpio::{AlternatePin, OutputPin};
use embassy_at32::spi::Spi;
use embassy_time::Timer;

use crate::board;

/// Generic ST7789 initialization, transcribed from the KWS-X1 firmware
/// (`firmware_core::panel::INIT`): sleep out, 16-bit pixels, porch/power/
/// gamma registers, inversion on.
const INIT: &[(u8, &[u8])] = &[
    (CMD_SLEEP_OUT, &[]),
    (CMD_PIXEL_FORMAT, &[0x05]),
    (0xc5, &[0x1a]),
    (0xb2, &[0x05, 0x05, 0x00, 0x33, 0x33]),
    (0xb7, &[0x05]),
    (0xbb, &[0x3f]),
    (0xc0, &[0x2c]),
    (0xc2, &[0x01]),
    (0xc3, &[0x0f]),
    (0xc4, &[0x20]),
    (0xc6, &[0x01]),
    (0xd0, &[0xa4, 0xa1]),
    (0xe8, &[0x03]),
    (0xe9, &[0x09, 0x09, 0x08]),
    (
        0xe0,
        &[
            0xd0, 0x05, 0x09, 0x09, 0x08, 0x14, 0x28, 0x33, 0x3f, 0x07, 0x13, 0x14, 0x28, 0x30,
        ],
    ),
    (
        0xe1,
        &[
            0xd0, 0x05, 0x09, 0x09, 0x08, 0x03, 0x24, 0x32, 0x32, 0x3b, 0x14, 0x13, 0x28, 0x2f,
        ],
    ),
    (CMD_INVERSION_ON, &[]),
];

pub const CMD_SLEEP_OUT: u8 = 0x11;
pub const CMD_INVERSION_ON: u8 = 0x21;
pub const CMD_DISPLAY_ON: u8 = 0x29;
pub const CMD_COLUMN_ADDRESS: u8 = 0x2a;
pub const CMD_ROW_ADDRESS: u8 = 0x2b;
#[allow(dead_code)]
pub const CMD_MEMORY_WRITE: u8 = 0x2c;
pub const CMD_MEMORY_ACCESS_CONTROL: u8 = 0x36;
pub const CMD_PIXEL_FORMAT: u8 = 0x3a;
/// Wait after Sleep Out (ST 9.1.12: 5 ms for commands, 120 ms covers all).
pub const SLEEP_OUT_MS: u32 = 120;
/// Reset pulse and settle, in ms (the controller needs 10 us and 5 ms).
const RESET_MS: u32 = 10;

/// Bytes in one RGB565 row.
pub const ROW_BYTES: usize = board::WIDTH as usize * 2;

pub struct Panel {
    spi: Spi,
    dma: DmaChannel,
    dc: OutputPin,
    reset: OutputPin,
    _cs: OutputPin,
    _sck: AlternatePin,
    _mosi: AlternatePin,
    backlight: OutputPin,
}

/// Build the panel transport and bring the controller up.
///
/// # Safety
///
/// Exclusive-startup contract: claims SPI1, the panel's pins and the
/// [`board::PANEL_DMA`] channel; the board's `SPI1` and `DMA1_CHANNELn`
/// vectors must be routed to the HAL's `on_interrupt` hooks and unmasked
/// before the first transfer. Requires the clock profile applied and the
/// Embassy time driver running.
pub async unsafe fn init() -> Option<Panel> {
    let (unit, number) = board::PANEL_DMA;
    let channels = unsafe { dma::Channel::split(unit).ok()? };
    let dma = channels.into_iter().nth(number as usize - 1)?;

    // SAFETY: forwarded from this function's exclusive-startup contract;
    // each pin is claimed exactly once here.
    let dc = unsafe {
        OutputPin::new_push_pull(
            board::PIN_DC.0,
            board::PIN_DC.1,
            board::DC_LEVEL,
            board::PIN_DRIVE,
        )
        .ok()?
    };
    let reset = unsafe {
        OutputPin::new_push_pull(
            board::PIN_RST.0,
            board::PIN_RST.1,
            board::RST_LEVEL,
            board::PIN_DRIVE,
        )
        .ok()?
    };
    let cs = unsafe {
        OutputPin::new_push_pull(
            board::PIN_CS.0,
            board::PIN_CS.1,
            board::CS_LEVEL,
            board::PIN_DRIVE,
        )
        .ok()?
    };
    let backlight = unsafe {
        OutputPin::new_push_pull(
            board::PIN_BACKLIGHT.0,
            board::PIN_BACKLIGHT.1,
            board::BACKLIGHT_LEVEL,
            board::PIN_DRIVE,
        )
        .ok()?
    };
    let sck = unsafe {
        AlternatePin::new_push_pull(board::PIN_SPI_SCK.0, board::PIN_SPI_SCK.1, board::PIN_DRIVE)
            .ok()?
    };
    let mosi = unsafe {
        AlternatePin::new_push_pull(
            board::PIN_SPI_MOSI.0,
            board::PIN_SPI_MOSI.1,
            board::PIN_DRIVE,
        )
        .ok()?
    };

    // SAFETY: SPI1 is exclusively owned from here on.
    let spi = unsafe { Spi::new_spi1(board::SPI_MODE, board::SPI_DIVIDER) };

    let mut panel = Panel {
        spi,
        dma,
        dc,
        reset,
        _cs: cs,
        _sck: sck,
        _mosi: mosi,
        backlight,
    };

    panel.hard_reset().await;
    if !panel.init_sequence().await {
        return None;
    }
    panel.backlight.set_high();
    Some(panel)
}

impl Panel {
    async fn hard_reset(&mut self) {
        self.reset.set_low();
        Timer::after_millis(u64::from(RESET_MS)).await;
        self.reset.set_high();
        Timer::after_millis(u64::from(RESET_MS)).await;
    }

    async fn command(&mut self, cmd: u8, data: &[u8]) -> bool {
        if self.spi.flush().await.is_err() {
            return false;
        }
        self.dc.set_low();
        if self.spi.write(&[cmd]).await.is_err() {
            return false;
        }
        if data.is_empty() {
            return true;
        }
        if self.spi.flush().await.is_err() {
            return false;
        }
        self.dc.set_high();
        self.spi.write(data).await.is_ok()
    }

    async fn init_sequence(&mut self) -> bool {
        for &(cmd, data) in INIT {
            if !self.command(cmd, data).await {
                return false;
            }
            if cmd == CMD_SLEEP_OUT {
                Timer::after_millis(SLEEP_OUT_MS as u64).await;
            }
        }
        self.command(CMD_MEMORY_ACCESS_CONTROL, &[board::MADCTL])
            .await
            && self.command(CMD_DISPLAY_ON, &[]).await
            && self.spi.flush().await.is_ok()
    }

    /// Set the drawing window to the full panel.
    pub async fn full_window(&mut self) -> bool {
        let x0 = board::X_OFFSET;
        let x1 = board::X_OFFSET + board::WIDTH - 1;
        let y0 = board::Y_OFFSET;
        let y1 = board::Y_OFFSET + board::HEIGHT - 1;
        self.command(CMD_COLUMN_ADDRESS, &span(x0, x1)).await
            && self.command(CMD_ROW_ADDRESS, &span(y0, y1)).await
            && self.command(CMD_MEMORY_WRITE, &[]).await
            && {
                // RAMWR sent: everything that follows is pixel data.
                self.dc.set_high();
                true
            }
    }

    /// Stream one RGB565 row through DMA. The caller keeps the row alive
    /// until this future completes; `full_window` must have been sent and
    /// the D/C line left in data mode.
    pub async fn write_row(&mut self, row: &[u8]) -> bool {
        if row.len() != ROW_BYTES {
            return false;
        }
        self.spi.write_dma(&mut self.dma, row).await.is_ok()
    }

    /// Backlight control while the panel runs.
    pub fn set_backlight(&mut self, on: bool) {
        if on {
            self.backlight.set_high();
        } else {
            self.backlight.set_low();
        }
    }
}

/// A CASET/PASET bounds pair, big-endian.
const fn span(start: u16, end: u16) -> [u8; 4] {
    [(start >> 8) as u8, start as u8, (end >> 8) as u8, end as u8]
}
