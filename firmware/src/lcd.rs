//! ST7789V 320x240 LCD driver in Rust, on the embassy HAL (SPI2 + GPIO).
//!
//! Ported 1:1 from the proven vendor C glue (`c/fnb58_lcd.c`, r40-r53):
//! same init sequence, same geometry, same 5x7 font, same 12 MHz SPI2
//! rate (APB1 48 MHz / 4, mode 0). The pins are the user-traced map:
//! PB13=SCK, PB15=MOSI (write-only), CS=PB12, DC=PB11, RST=PB10,
//! backlight gate PB3 (high = on).
//!
//! The driver performs the SWJTAG remap in `new()` (releasing PA15/PB3/PB4
//! from JTAG) — it must be constructed before anything configures those
//! pins. Bulk pixel data goes out through [`Spi::write_busy`] (tight
//! polling, executor-blocking like the C driver was; 12 MHz SCK is
//! ~0.7 us/byte).

use embassy_at32::gpio::{AlternatePin, Drive, OutputPin, Port};
use embassy_at32::spi::{Divider, Mode, Spi};
use embassy_time::Timer;

/// Screen geometry: landscape 320x240.
pub const W: u16 = 320;
pub const H: u16 = 240;

pub struct Lcd {
    spi: Spi,
    dc: OutputPin,
    cs: OutputPin,
    #[allow(dead_code)]
    rst: OutputPin,
    #[allow(dead_code)]
    backlight: OutputPin,
}

impl Lcd {
    /// Claim the pins and SPI2, and release PA15/PB3/PB4 from JTAG (the
    /// SWJTAG remap the C driver performed — without it PB3 stays JTDO and
    /// the button pins are inaccessible).
    ///
    /// # Safety
    ///
    /// Exclusive-startup contract: claims SPI2, PB3/PB10-PB12/PB13/PB15 and
    /// the IOMUX remap register; call once, before anything touches those
    /// pins.
    pub unsafe fn new() -> Self {
        // SWJTAG remap = vendor gpio_pin_remap_config(SWJTAG_GMUX_010):
        // IOMUX_REMAP7.SWJTAG_GMUX = 010 (JTAG-DP disabled, SW-DP enabled).
        let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
        crm.apb2en().modify(|_, w| w.iomux().bit(true));
        let iomux = unsafe { embassy_at32::pac::at32f403a::IOMUX::steal() };
        iomux.remap7().modify(|_, w| unsafe { w.swjtag_gmux().bits(0x02) });

        // Control pins: CS/DC/RST push-pull, idle high; backlight gate PB3.
        let cs = unsafe { OutputPin::new_push_pull(Port::B, 12, true, Drive::Moderate) }
            .expect("PB12 claim");
        let dc = unsafe { OutputPin::new_push_pull(Port::B, 11, true, Drive::Moderate) }
            .expect("PB11 claim");
        let rst = unsafe { OutputPin::new_push_pull(Port::B, 10, true, Drive::Moderate) }
            .expect("PB10 claim");
        let backlight = unsafe { OutputPin::new_push_pull(Port::B, 3, true, Drive::Moderate) }
            .expect("PB3 claim");
        let _sck =
            unsafe { AlternatePin::new_push_pull(Port::B, 13, Drive::Moderate) }.expect("PB13");
        let _mosi =
            unsafe { AlternatePin::new_push_pull(Port::B, 15, Drive::Moderate) }.expect("PB15");

        // SPI2 master, mode 0, 8-bit MSB, software CS; APB1 48 MHz / 4 =
        // 12 MHz (the C driver's exact rate).
        let spi = unsafe { Spi::new_spi2(Mode::Mode0, Divider::Div4) };
        Self {
            spi,
            dc,
            cs,
            rst,
            backlight,
        }
    }

    /// One command byte (D/C low), CS toggled around it.
    async fn cmd(&mut self, cmd: u8) {
        let _ = self.spi.flush().await;
        self.dc.set_low();
        self.cs.set_low();
        let _ = self.spi.write(&[cmd]).await;
        self.cs.set_high();
        self.dc.set_high();
    }

    /// Command plus its data bytes (D/C high for the data).
    async fn cmd_data(&mut self, cmd: u8, data: &[u8]) {
        self.cmd(cmd).await;
        self.cs.set_low();
        self.dc.set_high();
        let _ = self.spi.write(data).await;
        self.cs.set_high();
    }

    /// ST7789V power/gamma init — the C driver's register block verbatim
    /// (Adafruit-style defaults, proven on this panel).
    async fn init_regs(&mut self) {
        self.cmd_data(0xB2, &[0x0C, 0x0C, 0x00, 0x33, 0x33]).await; // porch
        self.cmd_data(0xB7, &[0x35]).await; // gate ctrl
        self.cmd_data(0xBB, &[0x19]).await; // VCOMS
        self.cmd_data(0xC0, &[0x2C]).await; // LCMCTRL
        self.cmd_data(0xC2, &[0x01]).await; // VDV/VRH enable
        self.cmd_data(0xC3, &[0x12]).await; // VRHS
        self.cmd_data(0xC4, &[0x20]).await; // VDVS
        self.cmd_data(0xC6, &[0x0F]).await; // FRCTRL2 60 Hz
        self.cmd_data(0xD0, &[0xA4, 0xA1]).await; // power ctrl 1
        self.cmd_data(
            0xE0,
            &[
                0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F,
                0x23,
            ],
        )
        .await; // positive gamma
        self.cmd_data(
            0xE1,
            &[
                0xD0, 0x04, 0x0E, 0x11, 0x12, 0x2A, 0x3D, 0x51, 0x4F, 0x1B, 0x0E, 0x0B, 0x1F,
                0x23,
            ],
        )
        .await; // negative gamma
    }

    /// Bring the controller up (hardware reset, wake, registers, display
    /// on) — the C `fnb58_lcd_init` sequence verbatim.
    pub async fn init(&mut self) {
        // Hardware reset with generous margins.
        self.rst.set_high();
        Timer::after_millis(50).await;
        self.rst.set_low();
        Timer::after_millis(20).await;
        self.rst.set_high();
        Timer::after_millis(150).await;

        self.cmd(0x01).await; // SWRESET
        Timer::after_millis(150).await;
        self.cmd(0x11).await; // SLPOUT
        Timer::after_millis(120).await;

        self.init_regs().await;

        self.cmd_data(0x3A, &[0x55]).await; // RGB565
        self.cmd_data(0x36, &[0x60]).await; // MADCTL: landscape
        self.cmd(0x21).await; // INVON: IPS panels need inversion

        self.fill(0x0000).await;
        self.cmd(0x29).await; // DISPON
        Timer::after_millis(100).await;
    }

    /// Address window (landscape: x 0-319, y 0-239); RAMWR follows.
    async fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        let ca = [(x0 >> 8) as u8, x0 as u8, (x1 >> 8) as u8, x1 as u8];
        let ra = [(y0 >> 8) as u8, y0 as u8, (y1 >> 8) as u8, y1 as u8];
        self.cmd_data(0x2A, &ca).await; // CASET
        self.cmd_data(0x2B, &ra).await; // RASET
        self.cmd(0x2C).await; // RAMWR (pixel data follows)
    }

    /// Fill the window with one RGB565 color (chunked busy-write).
    pub async fn fill_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, color: u16) {
        self.set_window(x0, y0, x1, y1).await;
        let n = (u32::from(x1 - x0 + 1)) * (u32::from(y1 - y0 + 1));
        let hi = (color >> 8) as u8;
        let lo = color as u8;
        let mut chunk = [0u8; 512];
        for pair in chunk.chunks_mut(2) {
            pair[0] = hi;
            pair[1] = lo;
        }
        self.cs.set_low();
        self.dc.set_high();
        let mut left = n as usize * 2;
        while left > 0 {
            let take = left.min(chunk.len());
            let _ = self.spi.write_busy(&chunk[..take]);
            left -= take;
        }
        self.cs.set_high();
    }

    /// Full-screen fill.
    pub async fn fill(&mut self, color: u16) {
        self.fill_window(0, 0, W - 1, H - 1, color).await;
    }

    /// Eight 40px vertical color bars: W R G B C M Y K.
    pub async fn test_pattern(&mut self) {
        const BARS: [u16; 8] = [
            0xFFFF, 0xF800, 0x07E0, 0x001F, 0x07FF, 0xF81F, 0xFFE0, 0x0000,
        ];
        for (i, &bar) in BARS.iter().enumerate() {
            let x0 = i as u16 * 40;
            self.fill_window(x0, 0, x0 + 39, H - 1, bar).await;
        }
    }

    /// Draw one character at (x, y), `scale` pixels per font pixel, fg on
    /// bg. Returns the character width (5*scale). The glyph pixels are
    /// composed into a line buffer (max scale 3: 15x21x2 = 630 B) and
    /// streamed with one busy write.
    pub async fn draw_char(&mut self, x: u16, y: u16, ch: u8, scale: u8, fg: u16, bg: u16) -> u16 {
        const FONT_CHARS: &[u8] = b"0123456789. VAWmu-PSCDBEFGHIKLMNORTUY:/";
        static FONT_5X7: [[u8; 5]; 39] = [
            [0x3E, 0x51, 0x49, 0x45, 0x3E], // 0
            [0x00, 0x42, 0x7F, 0x40, 0x00], // 1
            [0x62, 0x51, 0x49, 0x49, 0x46], // 2
            [0x41, 0x49, 0x49, 0x49, 0x36], // 3
            [0x18, 0x14, 0x12, 0x7F, 0x10], // 4
            [0x27, 0x45, 0x45, 0x45, 0x39], // 5
            [0x3C, 0x4A, 0x49, 0x49, 0x30], // 6
            [0x01, 0x71, 0x09, 0x05, 0x03], // 7
            [0x36, 0x49, 0x49, 0x49, 0x36], // 8
            [0x06, 0x49, 0x49, 0x29, 0x1E], // 9
            [0x00, 0x60, 0x60, 0x00, 0x00], // .
            [0x00, 0x00, 0x00, 0x00, 0x00], // space
            [0x1F, 0x20, 0x40, 0x20, 0x1F], // V
            [0x7E, 0x09, 0x09, 0x09, 0x7E], // A
            [0x3F, 0x40, 0x38, 0x40, 0x3F], // W
            [0x7C, 0x08, 0x10, 0x08, 0x7C], // m
            [0x3C, 0x40, 0x40, 0x20, 0x7C], // u
            [0x08, 0x08, 0x08, 0x08, 0x08], // -
            [0x7F, 0x09, 0x09, 0x09, 0x06], // P
            [0x46, 0x49, 0x49, 0x49, 0x31], // S
            [0x3E, 0x41, 0x41, 0x41, 0x22], // C
            [0x7F, 0x41, 0x41, 0x41, 0x3E], // D
            [0x7F, 0x49, 0x49, 0x49, 0x36], // B
            [0x7F, 0x49, 0x49, 0x49, 0x41], // E
            [0x7F, 0x09, 0x09, 0x09, 0x01], // F
            [0x3E, 0x41, 0x41, 0x51, 0x32], // G
            [0x7F, 0x08, 0x08, 0x08, 0x7F], // H
            [0x00, 0x41, 0x7F, 0x41, 0x00], // I
            [0x7F, 0x08, 0x14, 0x22, 0x41], // K
            [0x7F, 0x40, 0x40, 0x40, 0x40], // L
            [0x7F, 0x02, 0x0C, 0x02, 0x7F], // M
            [0x7F, 0x04, 0x08, 0x10, 0x7F], // N
            [0x3E, 0x41, 0x41, 0x41, 0x3E], // O
            [0x7F, 0x09, 0x19, 0x29, 0x46], // R
            [0x01, 0x01, 0x7F, 0x01, 0x01], // T
            [0x3F, 0x40, 0x40, 0x40, 0x3F], // U
            [0x03, 0x04, 0x78, 0x04, 0x03], // Y
            [0x00, 0x36, 0x36, 0x00, 0x00], // :
            [0x00, 0x60, 0x18, 0x06, 0x01], // /
        ];
        let idx = FONT_CHARS
            .iter()
            .position(|&c| c == ch)
            .unwrap_or(11); // default: blank
        let glyph = &FONT_5X7[idx];

        let w = 5 * u16::from(scale);
        let h = 7 * u16::from(scale);
        debug_assert!(scale >= 1 && scale <= 3, "glyph buffer is sized for scale 3");
        self.set_window(x, y, x + w - 1, y + h - 1).await;

        let mut buf = [0u8; 640];
        let mut i = 0;
        for r in 0..h {
            for c in 0..w {
                let col = glyph[(c / u16::from(scale)) as usize];
                let color = if col & (1 << (r / u16::from(scale))) != 0 {
                    fg
                } else {
                    bg
                };
                buf[i] = (color >> 8) as u8;
                buf[i + 1] = color as u8;
                i += 2;
            }
        }

        self.cs.set_low();
        self.dc.set_high();
        let _ = self.spi.write_busy(&buf[..i]);
        self.cs.set_high();
        w
    }

    /// Draw a string; 1px letter spacing (scale pixels), unsupported
    /// characters render as blanks. Returns the final x.
    pub async fn draw_text(&mut self, x: u16, y: u16, s: &[u8], scale: u8, fg: u16, bg: u16) -> u16 {
        let mut x = x;
        for &ch in s {
            if ch == 0 {
                break;
            }
            x += self.draw_char(x, y, ch, scale, fg, bg).await + u16::from(scale);
        }
        x
    }
}
