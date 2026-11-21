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

use crate::console;
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
    /// r223: the screenshot capture — when armed, every draw primitive
    /// ALSO streams to the console as a line (tools/fnb58-shot.py
    /// replays them into an image). The panel's data line is write-only
    /// (no MISO, the r45 trace), so GRAM read-back is impossible and
    /// the primitive stream IS the framebuffer. Consumed by the main
    /// loop's 'shot' block: it forces a full repaint with this armed.
    pub capture: bool,
    /// Primitives streamed since the capture was armed (the done line).
    pub capture_n: u32,
    // r216: the backlight (PB3) moved to backlight.rs — TMR2 CH2 PWM
    // for idle dimming; a plain GPIO output would keep the pad off the
    // timer. PB3 is no longer claimed here.
}

/// r242: the boot-primitive recorder — every draw primitive from the
/// very first one lands here as its capture line (a 16 KiB RAM ring,
/// recording stops at full), so a boot-time draw that the forced
/// repaint hides (the user's "s 03"/"s 05" header artifact: a real
/// primitive, which the shot's own black fill erases before any
/// capture) can be dumped with console `bootcap` line-for-line.
/// `static mut` + fresh each boot (.bss cleared at the reset vector);
/// main-task use only — the same contract as console's BOOT_LOG.
pub(crate) struct BootCap {
    pub(crate) buf: [u8; 16384],
    pub(crate) n: usize,
}

static mut BOOT_CAP: BootCap = BootCap {
    buf: [0; 16384],
    n: 0,
};

pub(crate) fn bootcap() -> &'static mut BootCap {
    unsafe { &mut *core::ptr::addr_of_mut!(BOOT_CAP) }
}

/// Append one capture line (stops at full).
pub(crate) fn bootcap_record(d: &[u8]) {
    let bc = bootcap();
    let room = bc.buf.len() - bc.n;
    let take = d.len().min(room);
    if take == 0 {
        return;
    }
    bc.buf[bc.n..bc.n + take].copy_from_slice(&d[..take]);
    bc.n += take;
}

impl Lcd {
    /// Claim the pins and SPI2, and release PA15/PB3/PB4 from JTAG (the
    /// SWJTAG remap the C driver performed — without it PB3 stays JTDO and
    /// the button pins are inaccessible).
    ///
    /// # Safety
    ///
    /// Exclusive-startup contract: claims SPI2, PB10-PB12/PB13/PB15 and
    /// the IOMUX remap register; call once, before anything touches those
    /// pins. PB3 belongs to backlight.rs since r216 (TMR2 CH2 PWM).
    pub unsafe fn new(spi: Spi) -> Self {
        // SWJTAG remap = vendor gpio_pin_remap_config(SWJTAG_GMUX_010):
        // IOMUX_REMAP7.SWJTAG_GMUX = 010 (JTAG-DP disabled, SW-DP enabled).
        let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
        crm.apb2en().modify(|_, w| w.iomux().bit(true));
        let iomux = unsafe { embassy_at32::pac::at32f403a::IOMUX::steal() };
        // Remap 100 = full SWJ disabled: PA13/PA14 join PA15/PB3/PB4 as
        // GPIO (PA13 is the W25Q128 flash chip-select). Flashing is
        // unaffected — it uses the BOOT0-high USB DFU (PA11/PA12), which
        // never touches the SWD pins.
        iomux.remap7().modify(|_, w| unsafe { w.swjtag_gmux().bits(0x04) });

        // Control pins: CS/DC/RST push-pull, idle high. The backlight is
        // backlight.rs's PWM (PB3, TMR2 CH2).
        let cs = unsafe { OutputPin::new_push_pull(Port::B, 12, true, Drive::Moderate) }
            .expect("PB12 claim");
        let dc = unsafe { OutputPin::new_push_pull(Port::B, 11, true, Drive::Moderate) }
            .expect("PB11 claim");
        let rst = unsafe { OutputPin::new_push_pull(Port::B, 10, true, Drive::Moderate) }
            .expect("PB10 claim");
        let _sck =
            unsafe { AlternatePin::new_push_pull(Port::B, 13, Drive::Moderate) }.expect("PB13");
        let _mosi =
            unsafe { AlternatePin::new_push_pull(Port::B, 15, Drive::Moderate) }.expect("PB15");

        // SPI2 master, mode 0, 8-bit MSB, software CS; APB1 48 MHz / 4 =
        // 12 MHz (the C driver's exact rate). The handle is passed in so
        // the W25Q128 flash driver can share the bus.
        Self {
            spi,
            dc,
            cs,
            rst,
            capture: false,
            capture_n: 0,
        }
    }

    /// r223: stream one fill primitive (`F x0 y0 x1 y1 cccc`). Sent via
    /// send_direct — the console queue's 20 slots drain one line per
    /// tick and would drop a capture burst mid-draw; send_direct blocks
    /// until the endpoint accepts, and a whole capture is well under
    /// the stall watch's 2 s window.
    fn capture_fill(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, color: u16) {
        let mut d = [0u8; 32];
        let mut n = 0;
        for &b in b"F " {
            d[n] = b;
            n += 1;
        }
        for v in [x0, y0, x1, y1] {
            n = console::push_dec(&mut d, n, u32::from(v));
            d[n] = b' ';
            n += 1;
        }
        n = console::push_hex16(&mut d, n, u32::from(color));
        d[n] = b'\r';
        d[n + 1] = b'\n';
        bootcap_record(&d[..n + 2]);
        if self.capture {
            self.capture_n += 1;
            crate::console::send_direct(&d[..n + 2]);
        }
    }

    /// r223: stream one glyph primitive (`C x y s <fg4> <bg4> <hh>`) —
    /// the char itself as two hex digits, so any byte survives the
    /// text-line console.
    fn capture_char(&mut self, x: u16, y: u16, scale: u8, fg: u16, bg: u16, ch: u8) {
        let mut d = [0u8; 32];
        let mut n = 0;
        for &b in b"C " {
            d[n] = b;
            n += 1;
        }
        for v in [x, y] {
            n = console::push_dec(&mut d, n, u32::from(v));
            d[n] = b' ';
            n += 1;
        }
        n = console::push_dec(&mut d, n, u32::from(scale));
        d[n] = b' ';
        n += 1;
        n = console::push_hex16(&mut d, n, u32::from(fg));
        d[n] = b' ';
        n += 1;
        n = console::push_hex16(&mut d, n, u32::from(bg));
        d[n] = b' ';
        n += 1;
        n = console::push_hex8(&mut d, n, u32::from(ch));
        d[n] = b'\r';
        d[n + 1] = b'\n';
        bootcap_record(&d[..n + 2]);
        if self.capture {
            self.capture_n += 1;
            crate::console::send_direct(&d[..n + 2]);
        }
    }

    /// Integer square root (corner radii and dot radii are single
    /// digits; Newton on u32, converges immediately at this size).
    fn isqrt(v: u32) -> u32 {
        if v == 0 {
            return 0;
        }
        let mut g = v;
        loop {
            let n = (g + v / g) / 2;
            if n >= g {
                return g;
            }
            g = n;
        }
    }

    /// The corner-row inset of a rounded shape's left boundary: the
    /// quarter circle centered (x0+r, y0+r), one pixel-step per row —
    /// at dy=0 the inset is r (the top edge starts at x0+r), at dy=r
    /// it is 0 (the left edge begins). kws-x1's rounded_outline /
    /// RoundedRectangle geometry.
    fn corner_inset(r: u16, dy: u16) -> u16 {
        let d = r - dy;
        r - Self::isqrt(u32::from(r) * u32::from(r) - u32::from(d) * u32::from(d)) as u16
    }

    /// r224: a 1 px outline with rounded corners (kws-x1's
    /// `rounded_outline` — the card look). Everything decomposes into
    /// fill_window calls, so the screenshot capture sees it all.
    pub async fn rounded_outline(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
        r: u16,
        color: u16,
    ) {
        // Straight edges (the corner arcs end where these begin).
        self.fill_window(x0 + r, y0, x1 - r, y0, color).await;
        self.fill_window(x0 + r, y1, x1 - r, y1, color).await;
        self.fill_window(x0, y0 + r, x0, y1 - r, color).await;
        self.fill_window(x1, y0 + r, x1, y1 - r, color).await;
        // Four corner arcs, continuous: each row spans horizontally from
        // its own inset back to the previous row's. The chord inset can
        // jump two-three columns between adjacent rows (r=5: 5,2,1,1,1),
        // and a one-pixel-per-row arc left HOLES there — the corners
        // rendered discontinuous (the user's broken-corner report).
        // The row's own inset stays the span's outer end, so the arc's
        // outer boundary still matches rounded_fill's corner cut.
        let mut prev = r;
        for dy in 1..r {
            let inset = Self::corner_inset(r, dy);
            let lo = inset.min(prev);
            let hi = inset.max(prev);
            self.fill_window(x0 + lo, y0 + dy, x0 + hi, y0 + dy, color).await;
            self.fill_window(x1 - hi, y0 + dy, x1 - lo, y0 + dy, color).await;
            self.fill_window(x0 + lo, y1 - dy, x0 + hi, y1 - dy, color).await;
            self.fill_window(x1 - hi, y1 - dy, x1 - lo, y1 - dy, color).await;
            prev = inset;
        }
        // The r-inset pixel where each arc meets an edge (dy = r is the
        // edge start, already drawn; the column arcs between the last
        // arc row and the straight edge need their own pixels).
        self.fill_window(x0, y0 + r, x0, y0 + r, color).await;
        self.fill_window(x1, y0 + r, x1, y0 + r, color).await;
        self.fill_window(x0, y1 - r, x0, y1 - r, color).await;
        self.fill_window(x1, y1 - r, x1, y1 - r, color).await;
    }

    /// r224: a filled rounded rectangle (row spans with corner insets) —
    /// the header's tab bars and the strip's heartbeat square.
    pub async fn rounded_fill(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
        r: u16,
        color: u16,
    ) {
        for dy in 0..=(y1 - y0) {
            let inset = if dy < r && y1 - y0 >= r {
                Self::corner_inset(r, dy)
            } else if y1 - dy < r && y1 - y0 >= r {
                Self::corner_inset(r, y1 - dy)
            } else {
                0
            };
            self.fill_window(x0 + inset, y0 + dy, x1 - inset, y0 + dy, color)
                .await;
        }
    }

    /// r227: a 1 px line between two points (kws-x1's stroked Line),
    /// drawn as one vertical span per column — a line is single-valued
    /// in x, so each column's [y(x_left), y(x_right)] range covers all
    /// its pixels with no gaps; a horizontal-ish line gets thin spans, a
    /// steep one thick ones. Each span is a fill_window, so the
    /// screenshot capture sees the whole line.
    pub async fn line(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, color: u16) {
        let (x0, y0, x1, y1) = if x1 < x0 { (x1, y1, x0, y0) } else { (x0, y0, x1, y1) };
        let dx = i32::from(x1) - i32::from(x0);
        let dy = i32::from(y1) - i32::from(y0);
        if dx == 0 {
            // Vertical: one span.
            let (a, b) = if y0 <= y1 { (y0, y1) } else { (y1, y0) };
            self.fill_window(x0, a, x0, b, color).await;
            return;
        }
        for x in x0..=x1 {
            // The line's y over this column's 1-px x interval: y at the
            // left and right edges, spanned (kws-x1's stroke, no gaps).
            let ya = (i32::from(x) - i32::from(x0)) as i64 * dy as i64 / dx as i64;
            let yb = (i32::from(x) + 1 - i32::from(x0)) as i64 * dy as i64 / dx as i64;
            let (ya, yb) = (ya.min(yb), ya.max(yb));
            let a = (i64::from(y0) + ya).max(0).min(239) as u16;
            let b = (i64::from(y0) + yb).max(0).min(239) as u16;
            self.fill_window(x, a, x, b, color).await;
        }
    }

    /// r224: a filled circle via row spans (the header's page dots).
    pub async fn dot(&mut self, cx: u16, cy: u16, r: u16, color: u16) {
        for dy in 0..=r {
            let half = Self::isqrt(
                u32::from(r) * u32::from(r) - u32::from(dy) * u32::from(dy),
            ) as u16;
            self.fill_window(cx - half, cy + dy, cx + half, cy + dy, color)
                .await;
            if dy > 0 {
                self.fill_window(cx - half, cy - dy, cx + half, cy - dy, color)
                    .await;
            }
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
        self.capture_fill(x0, y0, x1, y1, color);
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

    /// 180-degree orientation: MADCTL 0x60 (normal: MX|MV) vs 0xA0
    /// (inverted: MY|MV — MX cleared, MY set, so BOTH scan directions are
    /// flipped relative to normal = a true 180-degree rotation; adding MY
    /// alone r124-style only mirrored the screen). The controller does the
    /// mirroring, so every window coordinate keeps working unchanged.
    pub async fn set_inverted(&mut self, inverted: bool) {
        let madctl: u8 = if inverted { 0x60 ^ 0xC0 } else { 0x60 };
        self.cmd_data(0x36, &[madctl]).await;
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
        self.capture_char(x, y, scale, fg, bg, ch);
        // r189: Q and X appended (the QC screen title + the BK:EX hint);
        // r199: k s d i v p ~ appended (the ripple screen's kHz / us /
        // div / p-p / nominal-timebase labels); r238: b ? , _ appended
        // (the cable page's Gbps / "A ?" / "YES, ACTIVE" / GET_STATUS
        // strings); the blank fallback stays at index 11 — append only
        // at the end.
        const FONT_CHARS: &[u8] = b"0123456789. VAWmu-PSCDBEFGHIKLMNORTUY:/QXksdivp~<>+hb?,_";
        static FONT_5X7: [[u8; 5]; 56] = [
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
            [0x3E, 0x51, 0x49, 0x45, 0x3B], // Q (r189: tail on the O)
            [0x33, 0x33, 0x1E, 0x33, 0x33], // X (r189)
            [0x7C, 0x10, 0x28, 0x44, 0x00], // k (r199, x-height like m/u)
            [0x58, 0x14, 0x54, 0x14, 0x70], // s (r199)
            [0x78, 0x44, 0x44, 0x44, 0x7F], // d (r199, tall right stem)
            [0x00, 0x00, 0x7A, 0x00, 0x00], // i (r199, dot + stem one column)
            [0x0C, 0x1C, 0x70, 0x1C, 0x0C], // v (r199)
            [0x7F, 0x24, 0x24, 0x24, 0x3C], // p (r199, bowl no descender)
            [0x08, 0x04, 0x08, 0x10, 0x08], // ~ (r199, nominal-value mark)
            [0x08, 0x14, 0x22, 0x00, 0x00], // < (r224, the overview hint)
            [0x00, 0x00, 0x22, 0x14, 0x08], // > (r224)
            [0x08, 0x08, 0x3E, 0x08, 0x08], // + (r225, the D+ card label)
            [0x7C, 0x04, 0x04, 0x78, 0x00], // h (r226, the Ah/Wh units)
            [0x7E, 0x48, 0x48, 0x78, 0x30], // b (r238, the cable page's Gbps)
            [0x02, 0x01, 0x51, 0x09, 0x06], // ? (r238, the reserved-rating mark)
            [0x00, 0x40, 0x20, 0x00, 0x00], // , (r238, "YES, ACTIVE")
            [0x40, 0x40, 0x40, 0x40, 0x40], // _ (r238, "PD2 NO GET_STATUS")
        ];
        let idx = FONT_CHARS
            .iter()
            .position(|&c| c == ch)
            .unwrap_or(11); // default: blank
        let glyph = &FONT_5X7[idx];

        let w = 5 * u16::from(scale);
        let h = 7 * u16::from(scale);
        debug_assert!(scale >= 1 && scale <= 4, "glyph buffer is sized for scale 4");
        self.set_window(x, y, x + w - 1, y + h - 1).await;

        let mut buf = [0u8; 1120];
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
